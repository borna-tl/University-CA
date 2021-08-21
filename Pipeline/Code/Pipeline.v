`timescale 1ns/1ns
module MIPS(input clk,PCinit);
  wire [5:0]opcode,func;
  wire [2:0]ALUoperation;
  wire RegDst,ALUsrc,MemtoReg,MemWrite,MemRead,RegWrite,PCsrc,eq,clr,jal,jal_write;
  wire [1:0] jsel;
  controller CU(opcode,func,PCinit,eq,jal,jal_write,RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite,jsel,PCsrc,clr,ALUoperation);
  datapath DP(clk,jal,jal_write,PCinit,PCsrc,jsel,RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite,clr,ALUoperation,func,opcode,eq);
endmodule