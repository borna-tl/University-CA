`timescale 1ns/1ns
module MIPS(input clk, PCinit);
	wire [5:0]opcode, func;
  	wire [2:0] aluoperation;
	wire [1:0] regdst, writedata_sel;
  	wire zero, memread, memwrite, memtoreg, alusrc, regwrite, jmp_sel, jr_sel, pcsrc;
  	controller con(zero, func, opcode, memread, memwrite, memtoreg, alusrc, regwrite, jmp_sel, jr_sel, pcsrc,
			regdst, writedata_sel, aluoperation);
	
	datapath dat(clk, PCinit, memread, memwrite, memtoreg, alusrc, regwrite, jmp_sel, jr_sel, pcsrc,
			regdst, writedata_sel, aluoperation, zero, func, opcode);
endmodule

module TB();
	initial begin
	$display ("Hello cruel world"); end
	reg PCinit, clk = 1'b0;
	MIPS mips(clk, PCinit);
	always #100 clk = ~clk;
	initial begin
		#20 PCinit = 1; //reset PC address to 0
		#120 PCinit = 0;
 		#50000 $stop;
  	end

endmodule

