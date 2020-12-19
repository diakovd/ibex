
 module top_artya7_100_tb;
 
 logic       IO_CLK = 0;
 logic       IO_RST_N = 0;
 logic [3:0] LED;
 
 top_artya7_100 top_artya7_100_inst(
	.IO_CLK(IO_CLK),
	.IO_RST_N(IO_RST_N),
	.LED(LED)
 );
 
 always #4 IO_CLK <= ~IO_CLK;
 initial #200 IO_RST_N = 1; 

 endmodule