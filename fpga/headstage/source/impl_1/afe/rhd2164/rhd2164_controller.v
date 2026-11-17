/*********************************************************************************
	Author: Manuel Monge
	Description:
		Walks the RHD2164 sampling cycle: 33 slots per frame (32 channel
		conversions + 1 alternate-command placeholder), fetching each
		slot's command word from app/regbank.v's config/sampling table.
		Needs regbank_map.vh for RB_CONFIG_BASE/RB_SAMPLING_BASE/etc. —
		not RHD2164 command encoding itself (this module doesn't touch
		rhd2164_defs.vh), just where those tables live in Kuntur's memory
		map.
*********************************************************************************/

`include "../../app/regbank_map.vh"

module rhd2164_controller (
	input  wire	rstb,						// Active-low asyncrhonous reset
	input  wire	clk,						// Input clock
	output reg	rhd_start,					// Active-high output that starts RHD2164x2 SPI cycle
	input  wire rhd_done,					// Active-high input that indicates SPI cycle has finished
	output wire	[7:0]	rb_addr1,
	output wire	[5:0]	ch_cnt,
	output reg	cfg_done,					// Active-high; config table played, sampling cycle running
	output reg	xfer_done
);
	
	// state
	localparam idle = 4'd0;
	localparam op0a = 4'd1; // Wait 1-cycle for SRAM reading
	localparam op0b = 4'd2; // Start SPI-Frame
	localparam op0c = 4'd3; // Wait until SPI-Frame is done
	localparam op0d = 4'd4; // Increase counter. Done with configuration?
	localparam op1a = 4'd5; // Wait 1-cycle for SRAM reading
	localparam op1b = 4'd6; // Start SPI-Frame
	localparam op1c = 4'd7; // Wait until SPI-Frame is done
	localparam op1d = 4'd8; // Increase counter

	reg [3:0] current_state,next_state;
	
	// wire & regs
	reg [5:0] cnt0;
	wire [5:0] max;
	reg cnt0_en,cnt1_en;
	wire cnt0_is_max;
	reg	rhd_dtx_sel;				// Output selector to the data_sel mux
	
	
	// cfg_done
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cfg_done <= 1'b0;
		else if ( (current_state == op0d) && (cnt0_is_max))
			cfg_done <= 1'b1;
	
	
	// rb_addr1
	//assign rb_addr1 = {1'b0, rhd_dtx_sel, cnt0};
	assign rb_addr1 = (rhd_dtx_sel) ? (`RB_SAMPLING_BASE + cnt0):(`RB_CONFIG_BASE + cnt0);
	
	// 6-bit cfg-counter
	always@(negedge rstb or posedge clk)
		if (!rstb)
			cnt0 <= 0;
		else if (cnt0_en || cnt1_en)
			if (cnt0_is_max)
				cnt0 <= 0;
			else
				cnt0 <= cnt0 + 1'b1;
	
	// comparator to max
	assign max = (rhd_dtx_sel) ? (`RB_SAMPLING_MAX):`RB_CONFIG_MAX;
	assign cnt0_is_max = (cnt0 == max) ? 1:0;
		
	
	// ch_cnt enabled only during sampling
	assign ch_cnt = (rhd_dtx_sel) ? cnt0:0;


	//FSM
	
	// state register
	always@(negedge rstb or posedge clk)
		if (!rstb)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(current_state or rhd_done or cnt0_is_max) begin
		// default values
		cnt0_en = 1'b0;
		cnt1_en = 1'b0;
		rhd_start = 1'b0;
		rhd_dtx_sel = 1'b0;
		xfer_done = 1'b0;
		
		case (current_state)
			idle: begin
				next_state = op0a;
			end
			
			// RHD2164 Configuration
			op0a: begin
				next_state = op0b; // wait for SRAM reading to finish
			end
			op0b: begin
				next_state = op0c;
				rhd_start = 1'b1;
			end
			op0c: begin
				if (rhd_done)
					next_state = op0d;
				else
					next_state = op0c;
			end
			op0d: begin
				if (cnt0_is_max)
					next_state = op1a;
				else
					next_state = op0a;
				cnt0_en = 1'b1;
			end
			
			// RHD2164 Sampling Cycle
			op1a: begin
				next_state = op1b; // wait for SRAM reading to finish
				rhd_dtx_sel = 1'b1;
			end
			op1b: begin
				next_state = op1c;
				rhd_start = 1'b1;
				rhd_dtx_sel = 1'b1;
			end
			op1c: begin
				if (rhd_done)
					next_state = op1d;
				else
					next_state = op1c;
				rhd_dtx_sel = 1'b1;
			end
			op1d: begin
				next_state = op1a;
				cnt1_en = 1'b1;
				rhd_dtx_sel = 1'b1;
				xfer_done = 1'b1;
			end
			
			
			// Other States
			default: begin
				next_state = idle;
			end
		endcase
	end
	
endmodule
