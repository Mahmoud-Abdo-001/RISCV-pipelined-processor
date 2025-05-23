
module tb;
reg clk;
reg reset;


TOP_DUT DUT (
    .clk(clk),
    .reset(reset)
    );

// localparam CLK_PERIOD = 10;
// always #(CLK_PERIOD/2) clk=~clk;

initial
begin
clk = 0;
forever #2 clk=!clk;
end



//generate stim
initial begin
    // #1 reset <=1'bx;clk<=1'bx;
	
    // // #(CLK_PERIOD*3) resetn<=1;
    // #(CLK_PERIOD*3) resetn<=0;clk<=0;
    // // repeat(5) @(posedge clk);
	// @(negedge clk);
    // reset<=1;
    // // @(posedge clk);
    // repeat(1000) @(posedge clk);
    // // $finish(2);
	// $stop;
	reset <= 1'b1;
	#8;
	reset <= 1'b0;
	#800;
	$stop;
	
end

initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
end


endmodule
