/*********************************************************************************
	Author: Manuel Monge
	Description:
		Testbench-only stub standing in for a real RHD2164 on spi1. Cycles
		two canned 4-word arrays (data_miso_01/02) selected by sck/csb
		edge counts — does NOT decode MOSI, has no register file, and has
		no response pipeline, so it cannot support any of the A.1.1
		verification-ladder rungs. PLAN.md B.1's "A.1.1 verification
		ladder — the simulation half" already flags this as needing to
		become a proper rhd2164_bfm before that work can run in
		simulation; this is where that replacement lands. Split out of
		kuntur_tb.sv into its own file, 2026-08-26 (PLAN.md B.3).
*********************************************************************************/

module rhd2164_model(
	input  wire rstb,
	// Input from master
	input  wire spi1_csb,
	input  wire spi1_sck,
	input  wire spi1_mosi,
	// Output to master
	output reg  spi1_miso0
	);

	//parameter data_miso_01 = 16'he7e7;
	//parameter data_miso_02 = 16'h8181;

	// data to be sent
	reg [15:0] data_miso_01 [0:3];
	reg [15:0] data_miso_02 [0:3];

	reg [4:0] cnt0,cnt1;
	reg [1:0] cnt2;
	reg aux0,sel0;
	wire spi1_miso0_0,spi1_miso0_1,spi1_miso0_2,spi1_miso0_3;

	// spi1_csb
	// 6-bit cfg-counter
	always@(negedge rstb or posedge spi1_csb or negedge spi1_csb or negedge spi1_sck)
		if (!rstb) begin
			cnt0 <= 0;
			aux0 <= 0;
		end
		else if (spi1_csb)
			aux0 <= 0;
		else if (!spi1_csb && aux0==0) begin
			cnt0 <= 5'd16;
			aux0 <= 1;
		end
		else if (!spi1_sck)
			cnt0 <= cnt0 - 1'b1;

	// 6-bit cfg-counter
	always@(negedge rstb or negedge spi1_csb or posedge spi1_sck)
		if (!rstb)
			cnt1 <= 0;
		else if (spi1_sck)
			cnt1 <= cnt1 - 1'b1;
		else if (!spi1_csb)
			cnt1 <= 5'd16;

	// 2-bit cfg-counter
	always@(negedge rstb or posedge spi1_csb)
		if (!rstb)
			cnt2 <= 0;
		else if (spi1_csb)
			cnt2 <= cnt2 + 1'b1;

	always #10 sel0 <= spi1_sck;

	assign spi1_miso0_0 = (sel0) ? data_miso_02[0][cnt1[3:0]]:data_miso_01[0][cnt0[3:0]];
	assign spi1_miso0_1 = (sel0) ? data_miso_02[1][cnt1[3:0]]:data_miso_01[1][cnt0[3:0]];
	assign spi1_miso0_2 = (sel0) ? data_miso_02[2][cnt1[3:0]]:data_miso_01[2][cnt0[3:0]];
	assign spi1_miso0_3 = (sel0) ? data_miso_02[3][cnt1[3:0]]:data_miso_01[3][cnt0[3:0]];

	always@(cnt2 or spi1_miso0_0 or spi1_miso0_1 or spi1_miso0_2 or spi1_miso0_3)
		case(cnt2)
			2'd0: spi1_miso0 <= spi1_miso0_0;
			2'd1: spi1_miso0 <= spi1_miso0_1;
			2'd2: spi1_miso0 <= spi1_miso0_2;
			2'd3: spi1_miso0 <= spi1_miso0_3;
			default: spi1_miso0 <= spi1_miso0_0;
		endcase

	// initial values

	initial
	begin
		sel0 = 0;
		data_miso_01[0] = 16'he7e6;
		data_miso_01[1] = 16'he7e7;
		data_miso_01[2] = 16'he7e8;
		data_miso_01[3] = 16'he7e9;
		data_miso_02[0] = 16'h8181;
		data_miso_02[1] = 16'h8182;
		data_miso_02[2] = 16'h8183;
		data_miso_02[3] = 16'h8184;
	end

endmodule
