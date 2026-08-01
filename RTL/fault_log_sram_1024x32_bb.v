(* blackbox *)
module fault_log_sram_1024x32 (clk0, csb0, web0, wmask0, addr0, din0, dout0,
                               clk1, csb1, addr1, dout1);
  input  clk0, csb0, web0, clk1, csb1;
  input  [3:0]  wmask0;
  input  [9:0]  addr0, addr1;
  input  [31:0] din0;
  output [31:0] dout0, dout1;
endmodule
