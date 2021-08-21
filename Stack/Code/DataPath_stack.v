`timescale 1ns/1ns

/*module Memory(Address, WriteData, MemWrite, MemRead, clk, ReadData);
  input [4:0] Address, WriteData;
  input MemWrite, MemRead, clk;
  output [7:0] ReadData;
  reg [7:0] memory [0:31];
  always @(posedge clk)begin
    if(MemWrite)
      memory[Address] <= WriteData;
  end
  initial begin
    $readmemb("array.txt",memory);
  end
  assign ReadData = MemRead ? memory[Address] : 8'd0;
endmodule*/

module DataMem(input clk, MemWrite, MemRead, input [4:0] adr, input [7:0] WriteData, output reg [7:0] ReadData);
    reg [7:0] mem [0:31];
    initial begin
        $readmemb("Instructions.txt", mem);
    end

    initial begin
        $readmemb("Data.txt", mem, 27);
    end

    always @(posedge clk) begin
        if(MemWrite) mem[adr] <= WriteData;
    end

    assign ReadData = MemRead ? mem[adr] : 8'dz;
endmodule
/*
module DataMem(input clk, MemWrite, MemRead, input [4:0] adr, input [7:0] WriteData, output reg [7:0] ReadData);
	reg [7:0] Mem [0:31];

	initial begin
    		$readmemb("random.data", Mem);
  	end
	
 	always@(MemRead, adr) begin
		
    		if(MemRead) ReadData = Mem[adr];
  	end

	integer fd, i;
  	always@(posedge clk) begin
    		if(MemWrite)begin
      			Mem[adr] = WriteData;
			fd = $fopen("random.data", "w");  
			for (i = 0; i < 30; i = i + 1) begin	
				$fwrite(fd, "%b\n", Mem[i]);
			end
			$fwrite(fd, "%b", Mem[31]);
      			$display("Store in Address: %d", adr);
      			$display("Number: %d", WriteData);
    		end
  	end

	//assign ReadData = MemRead ? Mem[adr] : 8'd0;
endmodule
*/

module stack(input clk, rst, push, pop, tos, input [7:0] d_in, output reg[7:0] d_out);

	reg [4:0] pointer;
	reg [7:0] register [0:31];

	/*integer i;
	initial begin
		for(i = 0; i < 32; i = i + 1)
			register[i] <= 8'bxxxxxxxx;
	end*/

	/*always @(posedge clk /*push, pop, tos)begin
		if(push == 1'b1)begin
			register[pointer] <= d_in;
			pointer <= pointer + 1;
		end
		else if(pop == 1'b1)begin
			d_out <= register[pointer];
			pointer <= pointer - 1;
		end
		else if(tos == 1'b1)begin
			d_out <= register[pointer];
		end
	end*/

	always @(posedge clk, posedge rst) begin
		if (rst) pointer <= 5'd0;
		else begin
			if (push) begin
				register[pointer] = d_in;
				pointer = pointer + 1;
			end
			else if (pop) begin
				pointer = pointer - 1;
				d_out = register[pointer];
			end
			else if (tos) begin
				d_out = register[pointer - 1];
			end
		end
	end
endmodule

module ALU(input[2:0] ALUOperation, input [7:0] A, B, output zero, output reg [7:0] ALUResult);

	
	always@(ALUOperation, A, B) begin
		ALUResult <= 8'd0;
		case(ALUOperation) 
			3'b000: ALUResult <= A + B;//add
			3'b001: ALUResult <= B - A;//sub
			3'b010: ALUResult <= A & B;//and
			3'b011: ALUResult <= ~A;//not
			3'b111: ALUResult <= A;//JZ
				
		endcase
	end

	assign zero = (ALUResult == 8'd0) ? 1'b1 : 1'b0;
	
endmodule

module register8b_ld(input clk,rst, ld, input [7:0] in, output reg [7:0] out);
	
	always@(posedge clk, posedge rst)begin
		if(rst)
			out <= 8'd0;
		else if(ld)
			out <= in;
	end

endmodule

module register8b(input clk, rst,input [7:0] in, output reg[7:0] out);
	
	always@(posedge clk, posedge rst)begin
		if(rst)
			out <= 8'd0;
		else
			out <= in;
	end

endmodule


/*module signExtend(input [15:0] in, output [31:0] out);

	assign out = in[15] ? {16'b1111111111111111,in} : {16'b0000000000000000,in};

endmodule*/


module mux5b(input s, input [4:0] A, B, output [4:0] out);//0 -> A ; 1 -> B

	assign out = (s == 1'b0) ? A : B;
		     
endmodule

module mux8b(input s, input [7:0] A, B, output [7:0] out);//0 -> A ; 1 -> B

	assign out = (s == 1'b0) ? A : B;
		     
endmodule

module InstMem(input [4:0] Address, output [7:0] Instruction);

	reg [7:0] mem[0:31];

	initial
	begin
		$readmemb("stack_test.data", mem);
	end

	assign Instruction = mem[Address];	

endmodule


module PC(input clk, rst, ld, input [4:0] in, output reg [4:0] out);

	always @(posedge rst, posedge clk)begin
		if(rst) out <= 5'd0;
		else if(ld) out <= in;
		
	end

endmodule 

module datapath(input clk, rst, PCSrc, pc_write, pc_write_con, Z, mem_sel, Mem_read, Mem_write, 
		IR_write, stack_sel, load_A, A_sel, B_sel, push, pop, tos, input [2:0]ALUOP, 
		output ALUZero, output [2:0] opcode);

 	wire [4:0] nextPC, PCAdd, address_in;
	wire [7:0] d_in, d_out, register_A_out, ReadData_out, MDR_out, ALU_out, 
		   Res_out, stack_in, stack_out, register_B_out, ALU_B, ALU_A, IR_out;
	

	PC pc(clk, rst, (Z & pc_write_con) | pc_write, nextPC, PCAdd);

	mux5b mux_mem(mem_sel, PCAdd, IR_out[4:0], address_in);
	
  	//InstMem inst_mem(PCAdd, Instruction);

	DataMem data_mem(clk, Mem_write, Mem_read, address_in, register_A_out, ReadData_out);
	
	register8b MDR(clk, rst, ReadData_out, MDR_out);

	//mux8b test(PCSrc, ALU_out, IR_out, nextPC);//test

	register8b_ld IR(clk,rst, IR_write, ReadData_out, IR_out);

	mux5b mux_pc(PCSrc, ALU_out[4:0], IR_out[4:0], nextPC);

	mux8b mux_stack(stack_sel, Res_out, MDR_out, stack_in);

	stack STACK(clk, rst, push, pop, tos, stack_in, stack_out);

	register8b_ld A(clk,rst, load_A, stack_out, register_A_out);

	register8b B(clk,rst, stack_out, register_B_out);

	mux8b B_mux(B_sel, 8'd1, register_B_out, ALU_B);

	mux8b A_mux(A_sel, {3'b000, PCAdd}, register_A_out, ALU_A);

	ALU alu(ALUOP, ALU_A, ALU_B, ALUZero, ALU_out);

	register8b RRES(clk, rst,ALU_out, Res_out);


  	assign opcode = IR_out[7:5];
  	

endmodule

		