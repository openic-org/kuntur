//testbench
//
//	Testbench for the console's binary-to-decimal converter (bin2dec),
//	the block console_fmt uses for F2 (kuntur-notes
//	docs/interfaces/companion-console.md §8.3). bin2dec's interface is
//	Manuel's (not in the spec); this tb assumes:
//
//	  din[31:0], start ─► bin2dec ─► dout[39:0]  10 BCD digits, digit 0
//	                                             (least significant) in [3:0]
//	                               ─► len[3:0]   significant digits, 1..10
//	                                             (1 for zero)
//	                               ─► rdy        high when idle
//
//	  - start is a 1-clock pulse while rdy; din is sampled with it;
//	  - rdy falls within 2 clocks of the start (it may still be high in
//	    the clock of the start itself), and rises again when dout and len
//	    are valid;
//	  - dout and len hold their values until the next start.
//
//	Checks
//	  1. Value: dout = BCD of din and len = its digit count, for 0, every
//	     10^k - 1 and 10^k (k = 1..9), FFFFFFFF, all of 0..1100, and 5000
//	     random values (half of them small, to cover short lengths).
//	  2. Time: rdy back within 40 clocks of the start (32 bits + margin).
//	  3. Hold: dout and len unchanged for a random 0..60 clocks after rdy,
//	     with din scrambled meanwhile.
//	  4. Reset: rdy = 1 after an asynchronous reset, also mid-conversion,
//	     and the next conversion is right.
//
//	Plusargs: +seed=<n>.
//	Define: +define+REF_MODEL replaces the DUT by a model; +define+MUT=<k>
//	  breaks it (with REF_MODEL), each must FAIL:
//	    1 len one short at powers of ten; 2 result overwritten in idle;
//	    3 bit 31 dropped; 4 len 0 for zero.
//
//	Run (from this directory):
//	  tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module bin2dec /,/^endmodule/' > /tmp/b2d_rtl.v
//	  iverilog -g2012 -Wall -o b2d.vvp bin2dec_tb.sv /tmp/b2d_rtl.v && vvp b2d.vvp
//	  iverilog -g2012 -Wall -DREF_MODEL -o b2d.vvp bin2dec_tb.sv && vvp b2d.vvp
//	  for k in 1 2 3 4; do iverilog -g2012 -DREF_MODEL -DMUT=$k -o b2d.vvp bin2dec_tb.sv && vvp b2d.vvp | grep bin2dec_tb:; done
//	Expected: "bin2dec_tb: ALL PASS" for the RTL and the model; FAIL for
//	  every mutant.

`timescale 1ns/1ps

`ifdef REF_MODEL
`ifndef MUT
	`define MUT 0
`endif
module bin2dec (
	input  wire	rst_n,
	input  wire	clk,
	input  wire	[31:0]	din,
	input  wire	start,
	output reg	[39:0]	dout,
	output reg	[3:0]	len,
	output wire	rdy
);
	reg busy;
	reg [5:0] n;
	reg [31:0] v;
	assign rdy = !busy;
	integer k;
	reg [39:0] d;
	always @(negedge rst_n or posedge clk)
		if (!rst_n) begin busy <= 0; n <= 0; dout <= 0; len <= 1; end
		else if (start && !busy) begin
			busy <= 1; n <= 0; v <= (`MUT == 3) ? {1'b0, din[30:0]} : din;
		end else if (busy) begin
			n <= n + 1;
			if (n == 33) begin
				busy <= 0;
				d = 0;
				for (k = 0; k < 10; k = k + 1) begin d[4*k +: 4] = v % 10; v = v / 10; end
				dout <= d;
				len <= 1;
				for (k = 1; k < 10; k = k + 1) if (d[4*k +: 4] != 0) len <= k + 1;
				if (`MUT == 4 && d == 0) len <= 0;
				if (`MUT == 1) begin
					for (k = 1; k < 10; k = k + 1)
						if (d[4*k +: 4] == 1 && (d & ((40'd1 << (4*k)) - 1)) == 0 && (d >> (4*k+4)) == 0) len <= k;
				end
			end
		end else if (`MUT == 2)
			dout <= {8'd0, din};
endmodule
`endif

module bin2dec_tb;
	localparam real T = 1000.0 / 12.0;
	reg clk = 0, rst_n = 1, start = 0;
	reg [31:0] din = 0;
	wire [39:0] dout;
	wire [3:0] len;
	wire rdy;
	integer fails = 0, seed = 1, convs = 0, tmax = 0;

	bin2dec dut (.rst_n(rst_n), .clk(clk), .din(din), .start(start), .dout(dout), .len(len), .rdy(rdy));
	always #(T/2) clk = ~clk;

	task fail(input [8*120-1:0] msg);
		begin fails = fails + 1; if (fails <= 20) $display("FAIL  %0s  (t=%0.3f us)", msg, $realtime/1000.0); end
	endtask

	function [39:0] bcd(input [31:0] x);
		integer k; reg [31:0] v;
		begin v = x; bcd = 0; for (k = 0; k < 10; k = k + 1) begin bcd[4*k +: 4] = v % 10; v = v / 10; end end
	endfunction
	function [3:0] ndig(input [31:0] x);
		reg [31:0] v;
		begin ndig = 1; v = x / 10; while (v != 0) begin ndig = ndig + 1; v = v / 10; end end
	endfunction

	// one conversion; inputs change on the falling edge
	task conv(input [31:0] x);
		integer n, h, i;
		reg [39:0] d0; reg [3:0] l0;
		begin
			@(negedge clk);
			n = 0;
			while (!rdy && n < 200) begin @(negedge clk); n = n + 1; end
			if (!rdy) fail("rdy never high before a start");
			din = x; start = 1;
			@(negedge clk) start = 0; din = $random(seed);
			n = 1;
			while (rdy && n < 3) begin @(negedge clk); n = n + 1; end
			if (rdy) fail("rdy did not fall within 2 clocks of start");
			while (!rdy && n < 200) begin @(negedge clk); n = n + 1; end
			if (n > tmax) tmax = n;
			if (n > 40) fail("conversion longer than 40 clocks");
			if (dout !== bcd(x) || len !== ndig(x)) begin
				fail("wrong result");
				if (fails <= 20) $display("      din %0d: dout %h len %0d, expected %h len %0d", x, dout, len, bcd(x), ndig(x));
			end
			d0 = dout; l0 = len;
			h = {$random(seed)} % 61;
			for (i = 0; i < h; i = i + 1) begin
				@(negedge clk) din = $random(seed);
				if (dout !== d0 || len !== l0) begin fail("result not held until the next start"); i = h; end
			end
			convs = convs + 1;
		end
	endtask

	integer i, k;
	reg [31:0] p;
	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		#0.05 rst_n = 0;
		#(T * 3.3) rst_n = 1;
		#0.1 if (rdy !== 1'b1) fail("rdy not 1 after reset");

		conv(0);
		p = 1;
		for (k = 1; k <= 9; k = k + 1) begin p = p * 10; conv(p - 1); conv(p); conv(p + 1); end
		conv(32'hFFFFFFFF); conv(32'hFFFFFFFE); conv(32'h80000000); conv(32'h7FFFFFFF);
		for (i = 0; i <= 1100; i = i + 1) conv(i);
		for (i = 0; i < 5000; i = i + 1)
			conv((i % 2) ? $random(seed) : ({$random(seed)} >> ({$random(seed)} % 32)));
		$display("      %0d conversions, longest %0d clocks", convs, tmax);

		// reset mid-conversion
		@(negedge clk) din = 32'd123456789; start = 1;
		@(negedge clk) start = 0;
		repeat (5) @(negedge clk);
		#(T / 5) rst_n = 0;
		#0.1 if (rdy !== 1'b1) fail("rdy not 1 at once on reset");
		#(T * 2) rst_n = 1;
		conv(32'd987654321);

		if (fails == 0) $display("bin2dec_tb: ALL PASS (%0d conversions)", convs);
		else $display("bin2dec_tb: %0d FAIL", fails);
		$finish;
	end
endmodule
