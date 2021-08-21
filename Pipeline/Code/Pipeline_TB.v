`timescale 1ns/1ns

module testMIPS();
  reg PCinit,clk;
  MIPS mips(clk,PCinit);
  initial begin
    clk=0;
      forever #100 clk = ~clk;  
  end 
  initial begin
    #20
    PCinit = 1;
    #120
    PCinit = 0;
    #100000
    $stop;
  end
endmodule