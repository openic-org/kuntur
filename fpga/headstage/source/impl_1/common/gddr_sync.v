// Copied unmodified below this note from Radiant 2025.2.1,
// ip/lifcl/ddr/rtl/gddr_sync.v (2026-10-29). Clock start-up sequence for
// the GDDRX4 tunnel on both FPGAs: vega docs/interfaces/lvds-tunnel.md 6.4.

// =============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// -----------------------------------------------------------------------------
//   Copyright (c) 2022 by Lattice Semiconductor Corporation
//   ALL RIGHTS RESERVED 
// -----------------------------------------------------------------------------
//
//   Permission:
//
//      Lattice SG Pte. Ltd. grants permission to use this code
//      pursuant to the terms of the Lattice Reference Design License Agreement. 
//
//
//   Disclaimer:
//
//      This VHDL or Verilog source code is intended as a design reference
//      which illustrates how these types of functions can be implemented.
//      It is the user's responsibility to verify their design for
//      consistency and functionality through the use of formal
//      verification methods.  Lattice provides no warranty
//      regarding the use or functionality of this code.
//
// -----------------------------------------------------------------------------
//
//                  Lattice SG Pte. Ltd.
//                  101 Thomson Road, United Square #07-02 
//                  Singapore 307591
//
//
//                  TEL: 1-800-Lattice (USA and Canada)
//                       +65-6631-2000 (Singapore)
//                       +1-503-268-8001 (other locations)
//
//                  web: http://www.latticesemi.com/
//                  email: techsupport@latticesemi.com
//
// -----------------------------------------------------------------------------
//
// =============================================================================
//                         FILE DETAILS         
// Project               : 
// File                  : gddr_sync.v
// Title                 : 
// Dependencies          :
// Description           : 
// =============================================================================
//                        REVISION HISTORY
// Version               : 1.0.0.
// Author(s)             : 
// Mod. Date             : 03.20.2018
// Changes Made          : Initial release.
// =============================================================================
`ifndef __GDDR_SYNC__
`define __GDDR_SYNC__


module gddr_sync (

// -----------------------------------------------------------------------------
// Input Ports
// -----------------------------------------------------------------------------
  rst,             // Asynchronous reset
  sync_clk,        // oscillator clk or other constant running low speed clk.
                   // note that this clk should not be coming from clk sources
                   // that this module will stop or reset (e.g. ECLKSYNC, CLKDIV)
  start,           // Initialize the sync process
  
// -----------------------------------------------------------------------------
// Output Ports
// -----------------------------------------------------------------------------
  // outputs
  stop,            // ECLKSYNC.stop signal
  ddr_reset,       // DDR and CLKDIV reset signal
  ready            // READY signal; clock sync is done.
);

//-----------------------------------------------------------------------------
// PORTS DECLARATIONS
//-----------------------------------------------------------------------------
// Input ports
input       rst;
input       sync_clk;
input       start;

// Output ports
output      stop;
output      ddr_reset;
output      ready;

// -----------------------------------------------------------------------------
// Local Parameters
// -----------------------------------------------------------------------------
// Local parameters: States
/* Aldec enum gddr_sync_main_sm STATES=
          INIT,
          STOP,
          RESET,
          READY
*/
localparam INIT   = 3'b000;
localparam STOP   = 3'b001;
localparam RESET  = 3'b011;
localparam READY  = 3'b100;

// -----------------------------------------------------------------------------
// Wire Declarations
// -----------------------------------------------------------------------------
wire        rst;
wire        sync_clk;
wire        start;
wire        ddr_reset;
wire        stop;
wire        ready;

reg         ddr_reset_d;
reg   [3:0] ctrl_cnt;                                    // control counter
reg   [2:0] stop_assert;                                 // stop signal counter
/* Aldec state_vector cs_gddr_sync enum gddr_sync_main_sm */
reg   [2:0] cs_gddr_sync /*synthesis syn_preserve=1*/ ;  // current state
reg   [2:0] ns_gddr_sync;                                // next state
reg         reset_flag;                                  // flag signal that
                                                         // indicates that RESET
                                                         // is already done


// -----------------------------------------------------------------------------
// Assign Statements
// -----------------------------------------------------------------------------
assign stop      = cs_gddr_sync[0];
assign ddr_reset = cs_gddr_sync[1] | ddr_reset_d;
assign ready     = cs_gddr_sync[2];

// -----------------------------------------------------------------------------
// Sequential Blocks
// -----------------------------------------------------------------------------
always @(posedge sync_clk or posedge rst) begin
  if (rst==1'b1)
  begin
    cs_gddr_sync  <= INIT;
    ctrl_cnt      <= 4'd0;
    stop_assert   <= 3'd0;
    reset_flag    <= 1'b0;
    ddr_reset_d   <= 1'b1;
  end
  else
  begin
    cs_gddr_sync  <= ns_gddr_sync;
    ddr_reset_d   <= 1'b0;

    // CTRL_CNT for state machines
    if (((cs_gddr_sync==INIT)&&(reset_flag==1'b0))||((ctrl_cnt == 4'd3)
         &&(cs_gddr_sync!=INIT)))
    begin
      ctrl_cnt <= 4'd0;
    end
    else if (ctrl_cnt < 4'd8)
    begin
      ctrl_cnt <= ctrl_cnt + 4'd1;
    end

    // STOP signal will then be asserted 4T after rstn
    if ((!rst)&&(start)&&(stop_assert<3'd4)&&(reset_flag==1'b0))
    begin
      stop_assert <= stop_assert + 3'd1;
    end

    // Asserts the reset_flag after RESET state
    if ((cs_gddr_sync==RESET)&&(ns_gddr_sync == STOP))
    begin
      reset_flag <= 1'b1;
    end

    // Deasserts the reset_flag after READY state
    if ((cs_gddr_sync==READY)&&(ns_gddr_sync == INIT))
    begin
      reset_flag <= 1'b0;
    end
  end
end

// GDDR_SYNC State machine
always @(*) begin

  case (cs_gddr_sync)
    INIT:  // INIT state 0
    begin
      if ((start)&&(stop_assert==3'd3)&&(reset_flag==1'b0))
      begin
        ns_gddr_sync = STOP;
      end
      else if ((reset_flag==1'b1)&&(ctrl_cnt == 4'd7)&&(start))
      begin
        ns_gddr_sync = READY;
      end
      else
      begin
        ns_gddr_sync = INIT;
      end
    end

    STOP:  //STOP state 1
    begin
      if (ctrl_cnt == 4'd3)
      begin
        if (reset_flag ==1'b1)
        begin
          ns_gddr_sync  = INIT;
        end
        else
        begin
          ns_gddr_sync  = RESET;
        end
      end
      else
      begin
        ns_gddr_sync = STOP;
      end
    end

    RESET:  // RESET state 2
    begin
      if (ctrl_cnt == 4'd3)
      begin
        ns_gddr_sync = STOP;
      end
      else
      begin
        ns_gddr_sync = RESET;
      end
    end

    READY:  // READY state 5
    begin
      if ((!start))
      begin
        ns_gddr_sync = INIT;
      end
      else
      begin
        ns_gddr_sync = READY;
      end
    end

    default:
    begin
      ns_gddr_sync = cs_gddr_sync;
    end

  endcase
end

endmodule // gddr_sync
`endif