`timescale 1ns/1ns

module controller(input [5:0]opcode,func,input PCinit,eq,output reg jal, jal_write,RegDst,ALUsrc,MemWrite,MemRead,MemtoReg,RegWrite,output reg [1:0]jsel, output reg PCsrc,clr,output reg[2:0]ALUoperation);
  reg [2:0]ALUop;
  reg Branch;
  always @(opcode)begin
    {RegDst,RegWrite,ALUsrc,MemtoReg,MemWrite,MemRead,Branch, jal, jal_write}=9'b0000000000;
    ALUop = 3'b000;
    case (opcode)
      6'b000000 :begin //R_type
        {ALUsrc,RegDst,MemWrite,MemRead,MemtoReg,RegWrite,Branch}=7'b0000010;
        ALUop = 3'b010;
		
      end
      6'b100011:begin //lw
        {ALUsrc,RegDst,MemWrite,MemRead,MemtoReg,RegWrite,Branch}=7'b1101110;
        ALUop = 3'b000;
      end
      6'b101011:begin //sw
       {ALUsrc,MemWrite,MemRead,RegWrite,Branch}=5'b11000;
        ALUop = 3'b000;
      end
      6'b000010 :begin //J
        Branch =1'b0;
      end
      6'b000100 :begin // beq
        {ALUsrc,MemWrite,MemRead,RegWrite,Branch}=5'b00001;
        ALUop = 3'b001;
      end
      6'b000101 :begin // bne
        {ALUsrc,MemWrite,MemRead,RegWrite,Branch}=5'b00001;
        ALUop = 3'b001;
      end
      6'b001000 :begin // addi
       {ALUsrc,RegDst,MemWrite,MemRead,MemtoReg,RegWrite,Branch}=7'b1100010;
        ALUop = 3'b011;
      end
      6'b001010 :begin // slti
	{ALUsrc,RegDst,MemWrite,MemRead,MemtoReg,RegWrite,Branch}=7'b1100010;
	 ALUop = 3'b111;
	end
	
      6'b000011 :begin //jal
	{ALUsrc,RegDst,MemWrite,MemRead,MemtoReg,RegWrite,Branch, jal, jal_write}=9'b000001011;
      end
    endcase
  end
  always@(ALUop,func)begin
    ALUoperation = 3'b101;
    case (ALUop)
      3'b000 :begin
        ALUoperation = 3'b010;
      end
      3'b001 :begin
        ALUoperation = 3'b110;
      end
      3'b010 :begin
        if(func == 6'b100000) ALUoperation = 3'b010;//add 
        if(func == 6'b100010) ALUoperation = 3'b110;//sub
        if(func == 6'b100100) ALUoperation = 3'b000;//and **
        if(func == 6'b100101) ALUoperation = 3'b001;//or **
        if(func == 6'b101010) ALUoperation = 3'b111;//slt
	if(func == 6'b001000) begin jsel = 2'b10; PCsrc = 1'b1; end
	 
      end 
      3'b011 :begin
        ALUoperation = 3'b010;
      end
      3'b100 :begin
        ALUoperation = 3'b000;
      end 
      3'b111 :begin
	ALUoperation = 3'b111;
      end
    endcase
  end
  always@(Branch,opcode,eq)begin
    PCsrc = 1'b0;
    //jsel = 1'b0;
    jsel = 2'b00;
    if(Branch == 1'b0)begin
      if(opcode == 6'b000010)begin
        //jsel = 1'b1;
	jsel = 2'b01;
        PCsrc = 1'b1;
      end
/*
      if(opcode == 6'b000000)begin
	jsel = 2'b10;
	PCsrc = 1'b1;
      end*/
    end
    else begin
      if(opcode == 6'b000100)PCsrc = eq;
      if(opcode == 6'b000101)PCsrc = ~eq;
    end
  end
  always@(posedge PCinit)begin
    PCsrc = 1'b0;
  end
  always@(eq)begin
    clr = (opcode == 6'b000100 && eq) || (opcode == 6'b000101 && ~eq)? 1 : 0;
  end
  always@(jsel)begin
    clr = 1'b0;
    if(jsel) clr = 1'b1;
  end
endmodule
