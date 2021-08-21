`timescale 1ns/1ns

module controller(input Zero, input[5:0] func, opcode, output reg MemRead, MemWrite, MemToReg, ALUSrc, RegWrite,
		jmp_sel, jr_sel, PCSrc, output reg[1:0] RegDst, WriteData_sel, output reg[2:0] ALUoperation);
	reg [1:0]ALUop;
	reg Branch;
	//assign PCSrc = &{Branch, Zero};
	always @(opcode)begin
		// 2         1       1           1      1      1           1        1         2            1
		{RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, WriteData_sel, Branch, PCSrc} = 13'b00000000000;
   		ALUop = 2'b00;
    		case (opcode)
      			6'b000000: begin //add-sub-slt-jr R3, R2, R1 (RTYPE)
			RegDst = 2'b01; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00001000;
        		ALUop = 2'b10;
			jmp_sel = (func == 6'b001000) ? 1'b1 : 1'b0;
			jr_sel = (func == 6'b001000) ? 1'b1 : 1'b0; //not too sure
      			end
			6'b001000: begin //addi R3, adr
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00011000;
        		ALUop = 2'b00;
      			end
			6'b001010: begin //slti R3, R2, adr //  r2 = (R3 > adr)
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00011000;
        		ALUop = 2'b10;
      			end
			6'b100011: begin //lw R3, R2, adr //  r2 = *r[(R3 + adr)]
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b10111000;
        		ALUop = 2'b00;
      			end
			6'b101011: begin //sw R3, R2, adr //  *r[(R3 + adr)] <= r2
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b01010000;
        		ALUop = 2'b00;
      			end
			6'b000010: begin //j adr //pc = adr
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00000100;
        		ALUop = 2'b00; //value doesn't matter
      			end
			6'b000011: begin //jal adr //R31 = pc + 4, pc = adr
			RegDst = 2'b10; WriteData_sel = 2'b01;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00001100;
        		ALUop = 2'b00; //value doesn't matter
      			end
			6'b000100: begin //beq if(R[rs]==R[rt]) PC=PC+4+BranchAddr
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00000001;
        		ALUop = 2'b01; //ALU OP OF 01 IS SUB
			//PCSrc = &{Branch, Zero};
      			end
			6'b000101: begin //bne if(R[rs]!=R[rt]) PC=PC+4+BranchAddr
			RegDst = 2'b00; WriteData_sel = 2'b00;
        		{MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, jmp_sel, jr_sel, Branch} = 8'b00000001;
        		ALUop = 2'b01; //ALU OP OF 01 IS SUB
			//PCSrc = &{Branch, ~Zero};
      			end
		endcase
	end
	always @(ALUop, func) begin
    		ALUoperation = 3'b101;
    		case (ALUop)
      			2'b00: begin
				ALUoperation = 3'b010; //add
 			end
			2'b01: begin
        			ALUoperation = 3'b110; //sub
      			end
      			2'b10: begin
        			if (func == 6'b100000) ALUoperation = 3'b010; //add
        			if (func == 6'b100010) ALUoperation = 3'b110; //sub
        			if (func == 6'b100100) ALUoperation = 3'b000; //and
        			if (func == 6'b100101) ALUoperation = 3'b001; //or
        			if (func == 6'b101010) ALUoperation = 3'b111; //slt
      			end 
    		endcase
	end
	always@(Branch, opcode, Zero) begin
    		PCSrc = 1'b0;
    		if(Branch == 1'b0) PCSrc = 1'b0;
    		else begin
      			if (opcode == 6'b000100)PCSrc = Zero;
      			if (opcode == 6'b000101)PCSrc = ~Zero;
    		end
  	end

endmodule
