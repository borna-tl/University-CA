`timescale 1ns/1ns

module DataMem(input clk, MemWrite, MemRead, input [31:0] Address, WriteData, output reg [31:0] ReadData);
	wire [12:0] adr;
	assign adr = Address[12:0];
	reg [31:0] Mem [0:8191];

	initial begin
    		$readmemb("Mem.data", Mem);
  	end

 	always@(MemRead, adr) begin
    		if(MemRead) ReadData = Mem[adr];
  	end

	integer fd, i;
  	always@(posedge clk) begin
    		if(MemWrite)begin
      			Mem[adr] = WriteData;
			fd = $fopen("Mem.data", "w");  
			for (i = 0; i < 8190; i = i + 1) begin	
				$fwrite(fd, "%b\n", Mem[i]);
			end
			$fwrite(fd, "%b", Mem[8191]);
      			$display("Store in Address: %d", Address);
      			$display("Number: %d", WriteData);
    		end
  	end
endmodule


module RegisterFile(input clk, regWrite, input [4:0] ReadRegister1, ReadRegister2, WriteRegister, input [31:0] WriteData,
		    output [31:0] ReadData1, ReadData2);

	reg [31:0] register [31:0];

	integer i;
	initial begin
		for(i = 0; i < 32; i = i + 1)
			register[i] <= 32'd0;
	end

	always @(posedge clk)begin
		if(regWrite) begin
			if(WriteRegister == 5'd0) register[WriteRegister] <= 32'd0;
			else register[WriteRegister] <= WriteData;

		end
	end

	assign ReadData1 = register[ReadRegister1];
	assign ReadData2 = register[ReadRegister2];
endmodule


module ALU(input[2:0] ALUOperation, input [31:0] A, B, output zero, output reg [31:0] ALUResult);

	//always @(posedge clk)begin
	always@(ALUOperation, A, B) begin
		ALUResult <= 32'd0;
		case(ALUOperation) 
			3'b000: ALUResult <= A & B;//and
			3'b001: ALUResult <= A | B;//or
			3'b010: ALUResult <= A + B;//add
			3'b110: ALUResult <= A - B;//sub
			3'b111: begin //slt
				if(A >= B) ALUResult <= 32'd0;
				else ALUResult <= 32'd1;
    				end
				
		endcase
	end

	assign zero = (ALUResult == 32'd0) ? 1'b1 : 1'b0;
	
endmodule


module signExtend(input [15:0] in, output [31:0] out);

	assign out = in[15] ? {16'b1111111111111111,in} : {16'b0000000000000000,in};

endmodule


module mux5b_3in(input [1:0] s, input [4:0] A, B, C, output [4:0] out);//0 -> A ; 1 -> B

	assign out = (s == 2'b00) ? A : 
		     (s == 2'b01) ? B : C;
	
endmodule

module mux32b_3in(input [1:0]s, input [31:0] A, B, C, output [31:0] out);//0 -> A ; 1 -> B

	assign out = (s == 2'b00) ? A : 
		     (s == 2'b01) ? B : C;
	
endmodule

module mux32b_2in(input s, input [31:0] A, B, output [31:0] out);//0 -> A ; 1 -> B

	assign out = (s == 1'b0) ? A : B;
		     
endmodule

module InstMem(input [31:0] Address, output [31:0] Instruction);

	wire [12:0] adr;
	assign adr = Address[12:0];

	reg [31:0] mem[0:8191];

	initial
	begin
		$readmemb("test.data", mem);
	end

	assign Instruction = mem[adr];	

endmodule


module PC(input clk, rst, input [31:0] in, output reg [31:0] out);

	always @(posedge clk)begin
		if(rst) out <= 32'd0;
		else out <= in;
	end

endmodule 

module add32b(input [31:0]A,[31:0]B, output [31:0]S);

  	assign S = A + B;

endmodule

module ShiftL2_26in(input [25:0]inp, output [27:0]res);

  	assign res = {inp,2'b00};

endmodule

module ShiftL2_32in(input [31:0]inp, output [31:0]res);

  	assign res = {inp[29:0],2'b00};

endmodule


module datapath(input clk, PCinit, MemRead, MemWrite, MemToReg, ALUSrc, 
		input RegWrite, jmp_sel, jr_sel, PCSrc, input[1:0] RegDst, WriteData_sel,
 	        input [2:0]ALUoperation, output ALUZero, output [5:0]func, opcode);

 	wire [31:0] nextPC, PCAdd, nextAdd, Instruction, ReadData1, ReadData2, WriteData, ExtData, ALUres;
  	wire [31:0]ShiftedLabel,LabelAdr,PC1,PC2,data,DatatoReg, ALU_B, mem_ReadData, mem_mux_out, 
	ExtData_sh, PCSrc_in, jump_in, jr_out;
  	wire [27:0]ShiftedAdr; 
  	wire [4:0] WriteRegister, DstReg;
	

	//PC(input clk, rst, input [31:0] in, output reg [31:0] out);
  	PC pc(clk, PCinit, nextPC, PCAdd);
	
	//add32b(input [31:0]A,[31:0]B, output [31:0]S);
  	add32b AddPC(PCAdd, 32'd4, nextAdd);
	
  	InstMem inst_mem(PCAdd, Instruction);

	//module RegisterFile(input clk, regWrite, input [4:0] ReadRegister1, ReadRegister2, WriteRegister, input [31:0] WriteData,
		    //output [31:0] ReadData1, ReadData2);
	RegisterFile RegFile(clk, RegWrite, Instruction[25:21], Instruction[20:16], 
			     WriteRegister, WriteData, ReadData1, ReadData2);

	mux32b_2in mux(ALUSrc, ReadData2, ExtData, ALU_B);
	
	//module ALU(input[2:0] ALUOperation, input [31:0] A, B, output zero, output reg [31:0] ALUResult);

	ALU alu(ALUoperation, ReadData1, ALU_B, ALUZero, ALUres);

	signExtend SgnExt(Instruction[15:0], ExtData);
	
	
	//module DataMem(input clk, MemWrite, MemRead, input [31:0] Address, WriteData, output reg [31:0] ReadData);
	
	DataMem data_mem(clk, MemWrite, MemRead, ALUres, ReadData2, mem_ReadData);

	mux32b_2in mux2(MemToReg, ALUres, mem_ReadData, mem_mux_out);

	mux32b_3in mux3(WriteData_sel, mem_mux_out, nextAdd, ALUres, WriteData);

	mux5b_3in mux4(RegDst, Instruction[20:16], Instruction[15:11], 5'd31, WriteRegister);

	ShiftL2_32in sh2(ExtData, ExtData_sh);

	add32b add_(ExtData_sh, nextAdd, PCSrc_in);
	
	//check zero
	mux32b_2in mux5(PCSrc, nextAdd, PCSrc_in, jump_in);

	ShiftL2_26in sh2_26(Instruction[25:0], ShiftedAdr);

	mux32b_2in mux6(jr_sel, {nextAdd[31:28], ShiftedAdr} , ReadData1, jr_out);

	mux32b_2in mux7(jmp_sel, jump_in, jr_out, nextPC);

	
	
/*
  	MUX5B RegFileMUX1(Instruction[20:16],Instruction[15:11],RegDst,DstReg);
  	MUX5B RegFileMUX2(DstReg,5'b11111,WRsel,WReg);
  MUX32B RegFileMUX3(ALUres,data,MemtoReg,DatatoReg);
  MUX32B RegFileMUX4(nextAdd,DatatoReg,WDsel,WData);
  RegFile RegFile(clk,RegWrite,Instruction[25:21],Instruction[20:16],WReg,WData,readData1,readData2);
    MUX32B ALUMUX(readData2,ExtData,ALUsrc,ALUop2);
  
  ShiftL26to28 ShjAdr(Instruction[25:0],ShiftedAdr);
  ShiftL32 ShBrAdr(ExtData,ShiftedLabel);
  Add32B BranchAdd(nextAdd,ShiftedLabel,LabelAdr);
  MUX32B PCMUX1(nextAdd,LabelAdr,PCsrc,PC1);
  MUX32B PCMUX2(PC1,{PC1[31:28],ShiftedAdr},jsel,PC2);
  MUX32B PCMUX3(PC2,readData1,jrsel,nextPC);
  */
  	assign opcode = Instruction[31:26];
  	assign func = Instruction[5:0];

endmodule

		