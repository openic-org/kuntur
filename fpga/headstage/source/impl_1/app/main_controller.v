/*********************************************************************************
	Author: Manuel Monge
	Description:
		SPI0 command FSM (A.1.1g) — decodes the 2-bit opcode on each SPI0
		transfer from the MCU: POP (2 transfers, FIFO ChA then ChB), WRITE
		(3 transfers, tag-checked: addr / high byte / commit), READ
		(1 transfer, self-addressing — value lands on the transfer after),
		NOP (1 transfer, no side effects, never swallows the next one). All
		three tag mismatches and a broken POP pair abort to op_nop0, one
		clock back to op_decode0 — every transfer either starts a new
		command or is an identified part of a sequence, none is silently
		consumed. See docs/interfaces/channel-selection-control-plane.md.
*********************************************************************************/

module main_controller (
	input  wire	rstb,						// Active-low asyncrhonous reset
	input  wire	clk,						// Input clock
	input  wire	fifo_empty,					// Active-high Input; indicates if FIFO is empty
	output reg	fifo_ren,					// Active-high Output; FIFO read-enable
	output reg	[1:0]	mcu_dtx_sel,
	output reg	mcu_dtx_en,
	input  wire	mcu_done,					// Active-high Input; indicates SPI communication with MCU is done
	input  wire	[1:0]	opcode,				// Operating Code: READ (00)
	output reg	regbank_wen,					// Active-high Output; SRAM write enable
	output reg	regbank_port_en,
	output reg	[1:0] regbank_port_sel,
	input  wire	[5:0] regbank_port_addr
);
	
	// state
	localparam idle = 5'd0;
	localparam op_decode0 = 5'd1; // Start SPI-Frame
	localparam op_decode1 = 5'd2; // Wait until SPI-Frame is done
	localparam op_write0 = 5'd3; // 
	localparam op_write1 = 5'd4; // 
	localparam op_write2 = 5'd5; // 
	localparam op_write3 = 5'd6; // 
	localparam op_write4 = 5'd7; // 
	localparam op_write5 = 5'd8; // 
	localparam op_write6 = 5'd9; // 
	localparam op_read0 = 5'd10; // 
	localparam op_read1 = 5'd11; // 
	localparam op_pop0 = 5'd12; // 
	localparam op_pop1 = 5'd13; // 
	localparam op_pop2 = 5'd14; // 
	localparam op_pop3 = 5'd15; // 
	localparam op_nop0 = 5'd16; // 
	
	// opcodes
	// 00=POP/01=WRITE (existing)
	// 10=REG_READ (new — wire regbank_dout0 to the TX mux)
	// 11=NOP (new, no side effects).
	localparam opcode_pop = 2'b00;
	localparam opcode_write = 2'b01;
	localparam opcode_read = 2'b10;
	localparam opcode_nop = 2'b11;
	

	reg [4:0] current_state,next_state;
	wire redge_mcu_done, opcode_is_pop, opcode_is_write, opcode_is_read, opcode_is_nop;
	reg mcu_done_reg;
	wire port_addr_is_1, port_addr_is_2, port_addr_is_3;
	
	
	// Detect rising-edge of mcu_done
	always@(negedge rstb or posedge clk)
		if (!rstb)
			mcu_done_reg <= 0;
		else
			mcu_done_reg <= mcu_done;
			
	assign redge_mcu_done = (~mcu_done_reg) && (mcu_done);
	
	// OpCode
	assign opcode_is_pop = (opcode_pop == opcode);
	assign opcode_is_write = (opcode_write == opcode);
	assign opcode_is_read = (opcode_read == opcode);
	assign opcode_is_nop = (opcode_nop == opcode);
	
	// Port Address
	assign port_addr_is_1 = (regbank_port_addr == 6'd1);
	assign port_addr_is_2 = (regbank_port_addr == 6'd2);
	assign port_addr_is_3 = (regbank_port_addr == 6'd3);
	
	// FSM
	
	// state register
	always@(negedge rstb or posedge clk)
		if (!rstb)
			current_state <= idle;
		else
			current_state <= next_state;
	
	// next_state logic
	always@(*) begin
		// default values
		fifo_ren = 1'b0;
		mcu_dtx_sel = 2'd0;
		mcu_dtx_en = 1'b0;
		regbank_wen = 1'b0;
		regbank_port_en = 1'b0;
		regbank_port_sel = 2'd1;
		
		case (current_state)
			idle: begin
				next_state = op_decode0;
			end
			
			
			// FIFO reading & SRAM Writing/Reading
			op_decode0: begin
				if (redge_mcu_done)
					next_state = op_decode1;
				else
					next_state = op_decode0;
			end
			
			
			// Decoder
			op_decode1: begin
				if ((opcode_is_write) && (port_addr_is_1)) begin
					regbank_port_en = 1'b1; // RegBank Port 1
					next_state = op_write0;
				end
				else if ((opcode_is_read) && (port_addr_is_1)) begin
					regbank_port_en = 1'b1; // RegBank Port 1
					next_state = op_read0; // SRAM Read
				end
				else if (opcode_is_pop) begin
					fifo_ren = 1'b1; // FIFO POP
					next_state = op_pop0;
				end
				else if (opcode_is_nop) begin
					next_state = op_nop0;
				end
				else begin
					next_state = op_nop0;
				end
				regbank_port_sel = 2'd1; // SRAM Port Selector
			end
			
			
			// SRAM Write - 3 transfers
			op_write0: begin
				next_state = op_write1;
				mcu_dtx_sel = 2'd2; // SRAM
				mcu_dtx_en = 1'b1;
				regbank_port_sel = 2'd1;
			end
			op_write1: begin
				if (redge_mcu_done)
					next_state = op_write2;
				else
					next_state = op_write1;
				regbank_port_sel = 2'd2;
			end
			op_write2: begin
				if ((opcode_is_write) && (port_addr_is_2)) begin
					regbank_port_en = 1'b1; // Write RegBank Port 2
					next_state = op_write3;
				end
				else begin
					next_state = op_nop0;
				end
				regbank_port_sel = 2'd2;
			end
			op_write3: begin
				next_state = op_write4;
				mcu_dtx_sel = 2'd2; // SRAM
				mcu_dtx_en = 1'b1;
				regbank_port_sel = 2'd2;
			end
			op_write4: begin
				if (redge_mcu_done)
					next_state = op_write5;
				else
					next_state = op_write4;
			end
			op_write5: begin
				if ((opcode_is_write) && (port_addr_is_3)) begin
					regbank_wen = 1'b1; // SRAM Write
					next_state = op_write6;
				end
				else begin
					next_state = op_nop0;
				end
			end
			op_write6: begin
				next_state = op_decode0;
				mcu_dtx_sel = 2'd2; // SRAM
				mcu_dtx_en = 1'b1;
			end
			
			
			// SRAM Read - 1 Transfer
			op_read0: begin
				next_state = op_read1;
				mcu_dtx_sel = 2'd2; // SRAM
				regbank_port_sel = 2'd1;
			end
			op_read1: begin
				next_state = op_decode0;
				mcu_dtx_sel = 2'd2; // SRAM
				mcu_dtx_en = 1'b1;
				regbank_port_sel = 2'd1;
			end
			
			
			// FIFO POP: Single POP and Two SPI operations
			op_pop0: begin
				next_state = op_pop1;
				mcu_dtx_sel = 2'd0; // ChA
				mcu_dtx_en = 1'b1;	
			end
			op_pop1: begin
				if (redge_mcu_done)
					next_state = op_pop2;
				else
					next_state = op_pop1;
			end
			op_pop2: begin
				if (opcode_is_pop) begin
					fifo_ren = 1'b0;
					next_state = op_pop3;
				end
				else begin
					next_state = op_nop0;
				end
			end
			op_pop3: begin
				next_state = op_decode0;
				mcu_dtx_sel = 2'd1; // ChB
				mcu_dtx_en = 1'b1;	
			end
			
			
			// NOP: 1 Transfer
			op_nop0: begin
				next_state = op_decode0;
				mcu_dtx_sel = 2'd0; // SRAM
				mcu_dtx_en = 1'b0;
			end
			
			// Other States
			default: begin
				next_state = idle;
			end
		endcase
	end
	
endmodule
