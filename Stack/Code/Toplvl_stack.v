`timescale 1ns/1ns

module toplevel_STACK(input clk, rst);

	wire [2:0] opcode, ALUOP;
	wire PCSrc, pc_write, pc_write_con, mem_sel, Mem_read, Mem_write, 
		IR_write, stack_sel, load_A, A_sel, B_sel, push, pop, tos, ALUZero;

	datapath dp(clk, rst, PCSrc, pc_write, pc_write_con, ALUZero, mem_sel, Mem_read, Mem_write, 
		IR_write, stack_sel, load_A, A_sel, B_sel, push, pop, tos, ALUOP, 
		 ALUZero, opcode);


	controller cont(clk, rst, opcode, ALUZero, PCSrc, pc_write, pc_write_con, 
		mem_sel, Mem_read, Mem_write, IR_write, stack_sel, 
		load_A, A_sel, B_sel, push, pop, tos, ALUOP);



endmodule


module stack_tb();

	/*initial begin
	$display ("Hello cruel world"); end
	reg rst = 1'b1, clk = 1'b0;
	toplevel_STACK CUT(clk, rst);
	always #100 clk = ~clk;
	initial begin
		#490 rst = 1'b0; //reset PC address to 0
		
 		#50000 $stop;
  	end
	*/
    reg clk = 1, rst = 1;

    toplevel_STACK sbp(.clk(clk), .rst(rst));
    always #10 clk = ~clk;

    initial begin
        #15 rst = 0;
        #1000 $stop;
    end

endmodule


