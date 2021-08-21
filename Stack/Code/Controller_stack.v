`timescale 1ns/1ns

module controller(input clk, rst, input[2:0] opcode, input ALUZero, output reg PCSrc, pc_write, pc_write_con, mem_sel, Mem_read, Mem_write, 
		IR_write, stack_sel, load_A, A_sel, B_sel, push, pop, tos, output reg [2:0]ALUOP);

	reg [4:0] ps, ns;
	parameter [5:0] A = 6'd0, B = 6'd1, C_TEMP = 6'd2, C = 6'd3, D = 6'd4, E = 6'd5, F = 6'd6, G = 6'd7,
			H_NOT = 6'd8, H = 6'd9, I = 6'd10, J = 6'd11, P = 6'd12, M = 6'd13, N_NOT = 6'd14,
			N = 6'd15, O = 6'd16, R_NOT = 6'd17, R = 6'd18, DONE = 6'd19, IDLE = 6'd20, G_TEMP = 6'd21, N_tmp = 6'd22;

	always @(ps, ALUZero, opcode)begin
		{PCSrc, pc_write, pc_write_con, mem_sel, Mem_read, Mem_write, 
		IR_write, stack_sel, load_A, A_sel, B_sel, push, pop, tos} = 15'd0;
   		ns = A;
    		case (ps)
			/*IDLE: begin
				ns = rst ? IDLE : A;
			end
			DONE: begin
				ns = DONE;
			end*/
      			A: begin //instruction fetch
				{Mem_read, IR_write, pc_write, mem_sel} = 4'b1110;
				ALUOP = 3'd0;
				ns = B;
			
      			end
			B: begin //instrudtion decode
				//ALUOP = 3'd0;
				{mem_sel, Mem_read} = 2'b11;
				ns = (opcode == 3'b100) ? C_TEMP : 
				     (opcode == 3'b101) ? D :
				     (opcode == 3'b111) ? E : 
				     (opcode == 3'b110) ? F : 
				     (opcode == 3'b000 || opcode == 3'b001 ||
					 opcode == 3'b010 || opcode == 3'b011) ? G : B;
      			end
			C_TEMP: begin //push
				ns = A;
				{push, stack_sel} = 2'b11;
			end
			C: begin //push dummy state
				{push, stack_sel, pc_write, mem_sel, Mem_read} = 5'b11000;
				ns = A;
      			end
			D: begin //pop
				pop = 1'b1;
				ns = I;
      			end
			E: begin //jz
				tos = 1'b1;
				ns = ALUZero == 1'b1 ? J : A;
      			end
			F: begin //j
				{pc_write, PCSrc} = 2'b11;
				ns = A;
      			end
			G: begin //add, and, sub, not
				pop = 1;
				ns = G_TEMP;
			end
			G_TEMP: begin //add, and, sub, not
				//pop = 1;
				load_A = 1'b1;
				ns = (opcode == 3'd011) ? H_NOT : H;
			end
			H_NOT: begin //not
				ALUOP = opcode; //alucontrolsrc check she
				{A_sel, B_sel} = 2'b11; //
				ns = N_NOT;
			end
			H: begin //add, and, sub
				pop = 1;
				ns = N_tmp;//state haye bishtar baraye har opcode sum/and/not ezafe shavad
			end
			I: begin //pop
				{load_A} = 1'b1;
				ns = O;
			end
			J: begin //jz
				load_A = 1'b1;
				ns = P;
			end
			P: begin //jz to be chcked
				{pc_write_con, PCSrc, A_sel} = 3'b111;
				ns = A;
			end
			M: begin //add, sub, and dummy
				A_sel = 1'b1;
				ALUOP = opcode;
				ns = R;
			end
			N_NOT: begin //not
				ALUOP = opcode;
				stack_sel = 1'b0;
				push = 1'b1;
				ns = A;
			end
			N_tmp: begin
				ns = N;
			end
			N: begin //add, sub, and
				ALUOP = opcode;
				{A_sel, B_sel} = 2'b11;
				ns = N_NOT;
			end
			O: begin //pop
				{mem_sel, Mem_write} = 2'b11;
				ns = A;
			end
			R_NOT: begin //not dummy
				ALUOP = opcode;
				A_sel = 1'b1;
				ns = A;
			end
			R: begin //add, sub, and
				push = 1'b1;
				stack_sel = 1'b0;
				ns = A;
			end
		endcase
	end

	always @(posedge clk, posedge rst)begin
		if(rst)
			ps <= A;
		else
			ps <= ns;
	end

endmodule
