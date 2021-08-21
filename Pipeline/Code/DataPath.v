`timescale 1ns/1ns

module Add32B(input [31:0]A,[31:0]B, output [31:0]S);
  assign S = A + B;
endmodule

module ALU(input [31:0]A,B,input [2:0]ALUoperation,output reg [31:0]O);
  always@(ALUoperation,A,B) begin
    case(ALUoperation)
    3'b000: O = A & B;
    3'b001: O = A | B;
    3'b010: O = A + B;
    3'b110: O = A - B;
    3'b111: begin
      if(A >= B) O = 32'd0;
      else O = 32'd1;
     	
    end
    endcase
  end
endmodule

module checkEQ(input [31:0]A,[31:0]B, output isEqual);
  assign isEqual = (A == B)? 1'b1 : 1'b0;
endmodule

module DataMem(input [31:0]Add,WData, input clk,MemRead,MemWrite, output reg [31:0]RData);
  wire [12:0]UsefulAdd;
  assign UsefulAdd = Add[12:0];
  reg [31:0] MemByte[0:8191];
  initial begin
    $readmemb("mem.data", MemByte);
  end
  always@(MemRead,UsefulAdd) begin
    if(MemRead) RData = MemByte[UsefulAdd];
  end

  integer fd, i;
  always@(posedge clk) begin
    if(MemWrite)begin
      MemByte[UsefulAdd] <= WData;
      fd = $fopen("mem.data", "w");  
	for (i = 0; i < 8190; i = i + 1) begin	
		$fwrite(fd, "%b\n", MemByte[i]);
	end
	$fwrite(fd, "%b", MemByte[8191]);
  
      $display("store in address :%d",Add);
      $display("number : %d" , WData);
    end
  end
endmodule

module InstMem(input [31:0] Address,output [31:0]Instruction); 
  wire [12:0]UsefulAdr;
  assign UsefulAdr = Address[12:0];
  reg [31:0] mem[0:8191];
	initial
	begin
		$readmemb("test1.data",mem);//test1
	end
	assign Instruction = mem[UsefulAdr];	
endmodule

module MUX3to1(input [31:0]I1,[31:0]I2,[31:0]I3, input[1:0]s, output [31:0]muxOut);
  assign muxOut = (s == 2'b00)? I1 : (s == 2'b01)? I2 : (s == 2'b10)? I3 : 32'bx;
endmodule

module MUX5B(input [4:0]I1,[4:0]I2, input s, output [4:0]muxOut);
  assign muxOut = s ? I2 : I1;
endmodule

module MUX9B(input [8:0]I1,[8:0]I2, input s, output [8:0]muxOut);
  assign muxOut = s ? I2 : I1;
endmodule

module MUX32B(input [31:0]I1,[31:0]I2, input s, output [31:0]muxOut);
  assign muxOut = s ? I2 : I1;
endmodule

module PCreg(input [31:0]iBus,clk,ld,init, output reg [31:0]oBus);
  always@(posedge clk) begin
    if(init) oBus <= 32'b0;
    else if(ld) oBus <= iBus; 
  end
endmodule

module RegFile(input clk,input RegWrite,input [4:0]readReg1,readReg2,writeReg,input[31:0]writeData,output [31:0]readData1,readData2);
  
  reg [31:0]Registers[0:31];
  integer k;
	initial begin
	 for (k = 0; k < 32; k = k + 1)begin
	   Registers[k] = 0;
	 end
	end
	always @(negedge clk)
	begin
		if (RegWrite == 1) 
		begin
		  if(writeReg == 0) Registers[writeReg] <= 32'b00000000000000000000000000000000;
		  else Registers[writeReg] <= writeData;
		end
	end
	assign readData1 = Registers[readReg1];
	assign readData2 = Registers[readReg2];
endmodule

module ShiftL26to28(input [25:0]inp, output [27:0]res);
  assign res = {inp,2'b00};
endmodule

module ShiftL32(input [31:0]inp, output [31:0]res);
  assign res = {inp[29:0],2'b00};
endmodule

module SignExtend(input [15:0]A, output [31:0]Res);
  assign Res = A[15] ? {16'b1111111111111111,A} : {16'b0000000000000000,A};
endmodule

module superReg1(input [31:0]inputAdr,inputInst,input clk,Instld,clr,output reg [31:0]outputAdr,outputInst);
  always@(posedge clk) begin
    if(clr) outputInst <= 32'b00000000000000000000000000100000;
    else begin
      outputAdr <= inputAdr;
      if(Instld) outputInst <= inputInst; 
    end
  end
endmodule

module superReg2(input [4:0] pc, input [8:0]cntIn,input [31:0]RDataIn1,RDataIn2,ExtDataIn,input [4:0]RtIn,RsIn,RdIn,input clk,
  output reg [8:0]cntOut,output reg [31:0]RDataOut1,RDataOut2,ExtDataOut,output reg [4:0]RtOut,RsOut,RdOut, pcOut);
  always@(posedge clk) begin
    cntOut <= cntIn;
    RDataOut1 <= RDataIn1;
    RDataOut2 <= RDataIn2;
    ExtDataOut <= ExtDataIn;
    RtOut <= RtIn;
    RsOut <= RsIn;
    RdOut <= RdIn;
    pcOut <= pc;
  end
endmodule

module superReg3(input [4:0] pc, input [3:0]cntIn,input [31:0]ALUResIn,WDataIn,input [4:0]DstRegIn,input clk,
  output reg [3:0]cntOut,output reg [31:0]ALUResOut,WDataOut,output reg [4:0]DstRegOut, pcOut);
  always@(posedge clk) begin
    cntOut <= cntIn;
    ALUResOut <= ALUResIn;
    WDataOut <= WDataIn;
    DstRegOut <= DstRegIn;
    pcOut <= pc;
  end
endmodule

module superReg4(input [4:0] pc, input [1:0]cntIn,input [31:0]ALUResIn,dataIn,input [4:0]DstRegIn,input clk,
  output reg [1:0]cntOut,output reg [31:0]ALUResOut,dataOut,output reg [4:0]DstRegOut, pcOut);
  always@(posedge clk) begin
    cntOut <= cntIn;
    ALUResOut <= ALUResIn;
    dataOut <= dataIn;
    DstRegOut <= DstRegIn;
    pcOut <= pc;
  end
endmodule

module FRWUnit(input [4:0]EX_Rs,EX_Rt,MEM_WReg,WB_WReg, input MEM_RegWrite,WB_RegWrite, output reg [1:0]FrwA, FrwB);
  reg Flag1,Flag2;
  always@(EX_Rs,MEM_WReg,WB_WReg,MEM_RegWrite,WB_RegWrite) begin
    FrwA = 2'b00;
    Flag1 = 0;
    if(MEM_RegWrite == 1 && MEM_WReg == EX_Rs && MEM_WReg != 5'b0) begin
      Flag1 = 1'b1;
      FrwA = 2'b01;
    end 
    if(WB_RegWrite == 1 && WB_WReg == EX_Rs && WB_WReg != 5'b0 && Flag1 == 0) begin
      FrwA = 2'b10;
    end
  end
  
  always@(EX_Rt,MEM_WReg,WB_WReg,MEM_RegWrite,WB_RegWrite) begin
    FrwB = 2'b00;
    Flag2 = 0;
    if(MEM_RegWrite == 1 && MEM_WReg == EX_Rt && MEM_WReg != 5'b0) begin
      Flag2 = 1'b1;
      FrwB = 2'b01;
    end 
    if(WB_RegWrite == 1 && WB_WReg == EX_Rt && WB_WReg != 5'b0 && Flag2 == 0) begin
      FrwB = 2'b10;
    end
  end
endmodule

module HazardUnit(input PCinit,ID_EX_MemRead ,input [4:0] ID_EX_Rt , Rt,Rs , input [5:0] opcode ,output reg PcWrite,InstLd,SignalSel);
  always@(opcode,ID_EX_MemRead,ID_EX_Rt,Rt,Rs)begin
    SignalSel = 1'b1;
    InstLd = 1'b1;
    PcWrite = 1'b1;
    if(opcode == 6'b000010)begin
      SignalSel = 1'b0;
    end
    if(ID_EX_MemRead == 1'b1 && ID_EX_Rt != 5'b00000)begin
      if(opcode == 6'b100011)begin
        if(ID_EX_Rt == Rs)begin
          InstLd = 1'b0;
          PcWrite = 1'b0;
          SignalSel = 1'b0;
        end
      end
      
      	else begin
	  //if(opcode != 6'd0)begin
           if(ID_EX_Rt == Rs || ID_EX_Rt == Rt)begin
             InstLd = 1'b0;
            PcWrite = 1'b0;
            SignalSel = 1'b0;
        end
      //end
	end
    end
  end
  always@(posedge PCinit)begin
    InstLd = 1'b1;
    PcWrite = 1'b1;
  end
  
endmodule

module datapath(input clk,jal,jal_write,PCinit,PCsrc,input [1:0] jsel,input RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite,clr,input [2:0]ALUoperation, 
  output [5:0]func,opcode, output eq);
  

  wire PCld,Instld,SignalSel;
  wire [31:0]nextPC,PCAdr,nextAdr,Instruction,ID_Adr,ID_Inst,readData1,readData2,ExtData,ShiftedLabel,LabelAdr,condPC,ALURes;
  wire [31:0]EX_RData1,EX_RData2,EX_Data,ALUop1,ALUop2,MEM_ALURes,MEM_WData,EX_WData,MEM_data,WB_ALURes,WB_dataOut,WB_WData,data_out;
  wire [27:0]ShiftedAdr; 
  wire [4:0]EX_Rt,EX_Rs,EX_Rd,EX_WReg,MEM_WReg,WB_WReg,jal_out, pc_out2, pc_out3, pc_out4;
  wire [8:0]ID_cnt,EX_cnt;
  wire [3:0]MEM_cnt;
  wire [1:0]WB_cnt,ForwardA,ForwardB;
  wire [8:0]cntSignals;
  assign cntSignals = {ALUoperation,RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite};

  PCreg PC(nextPC,clk,PCld,PCinit,PCAdr);

  Add32B AddPC(PCAdr,32'd4,nextAdr);

  InstMem InstMem(PCAdr,Instruction);

  superReg1 IF_ID(nextAdr,Instruction,clk,Instld,clr,ID_Adr,ID_Inst);

  RegFile RegFile(clk,WB_cnt[0],ID_Inst[25:21],ID_Inst[20:16],WB_WReg,WB_WData,readData1,readData2);

   SignExtend SgnExt(ID_Inst[15:0],ExtData);

  ShiftL32 ShBranchAdr(ExtData,ShiftedLabel);

  Add32B BranchAdd(ID_Adr,ShiftedLabel,LabelAdr);

  ShiftL26to28 ShjAdr(ID_Inst[25:0],ShiftedAdr);

  //MUX32B JMUX(LabelAdr,{PCAdr[31:28],ShiftedAdr},jsel,condPC);///

  //module MUX3to1(input [31:0]I1,[31:0]I2,[31:0]I3, input[1:0]s, output [31:0]muxOut);

   MUX3to1 JMUX(LabelAdr,{PCAdr[31:28],ShiftedAdr},readData1, jsel, condPC);

  MUX32B PCMUX(nextAdr,condPC,PCsrc,nextPC);

  checkEQ EQ(readData1,readData2,eq);
  
  MUX9B controllerMUX(9'd0,cntSignals,SignalSel,ID_cnt);
  //HAZARD UNIT
  HazardUnit HAZARD_UNIT(PCinit,EX_cnt[2],EX_Rt,ID_Inst[20:16],ID_Inst[25:21],ID_Inst[31:26],PCld,Instld,SignalSel);

  
  superReg2 ID_EX(nextAdr, ID_cnt,readData1,readData2,ExtData,ID_Inst[20:16],ID_Inst[25:21],ID_Inst[15:11],clk,EX_cnt,EX_RData1,EX_RData2,EX_Data,EX_Rt,EX_Rs,EX_Rd,pc_out2);

  MUX3to1 ALUMUX1(EX_RData1,MEM_ALURes,WB_WData,ForwardA,ALUop1);

  MUX3to1 ALUMUX2(EX_RData2,MEM_ALURes,WB_WData,ForwardB,EX_WData);

  MUX32B ALUMUX3(EX_WData,EX_Data,EX_cnt[4],ALUop2);

  ALU ALU(ALUop1,ALUop2,EX_cnt[8:6],ALURes);

  MUX5B DstReg(EX_Rd,EX_Rt,EX_cnt[5],EX_WReg);

  //module MUX5B(input [4:0]I1,[4:0]I2, input s, output [4:0]muxOut);
  MUX5B JAL(EX_WReg, 5'd31, jal, jal_out);
  
  //FORWARDING UNIT
  FRWUnit FRW_UNIT(EX_Rs,EX_Rt,MEM_WReg,WB_WReg,MEM_cnt[0],WB_cnt[0],ForwardA,ForwardB);
  
  superReg3 EX_MEM(pc_out2, EX_cnt[3:0],ALURes,EX_WData,jal_out,clk,MEM_cnt,MEM_ALURes,MEM_WData,MEM_WReg, pc_out3);
  DataMem DataMem(MEM_ALURes,MEM_WData,clk,MEM_cnt[2],MEM_cnt[3],MEM_data);
  
  superReg4 MEM_WB(pc_out3,MEM_cnt[1:0],MEM_ALURes,MEM_data,MEM_WReg,clk,WB_cnt,WB_ALURes,WB_dataOut,WB_WReg,pc_out4);
  
  MUX32B DATAMUX(WB_ALURes,WB_dataOut,WB_cnt[1],data_out);
  MUX32B jal_pc(data_out,pc_out4, jal_write, WB_WData);
  assign opcode = ID_Inst[31:26];
  assign func = ID_Inst[5:0];
  /*
  wire [31:0]nextPC,PCAdd,nextAdd,InstructionExtData,ALUop2,ALUres;
  wire [31:0]ShiftedLabel,LabelAdr,PC1,PC2,data,DatatoReg;
  wire [4:0]WReg,DstReg;
  
  MUX5B RegFileMUX1(Instruction[20:16],Instruction[15:11],RegDst,DstReg);
  
  MUX32B RegFileMUX3(ALUres,data,MemtoReg,DatatoReg);
  MUX32B RegFileMUX4(nextAdd,DatatoReg,WDsel,WData);
  
  
  MUX32B ALUMUX(readData2,ExtData,ALUsrc,ALUop2);
  
  
  
  MUX32B PCMUX3(PC2,readData1,jrsel,nextPC);
  
  */
endmodule


