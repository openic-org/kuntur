//testbench
//
//	Testbench for the companion's tunnel receiver (the "CDR").
//	Contract: vega docs/interfaces/lvds-tunnel.md §6.4 (receive clock chain,
//	PHASESTEP handshake) and §6.6 (byte alignment, checker, sweep controller).
//
//	  Kuntur side                     link                 companion side
//	  block_gen (real RTL) ─ ser ─► data: +d_data ±jitter ─► IDDRX4 model ─► rx_ref
//	  tx_eclk ─────────────────────► clk:  +d_clk ──► PLL model (+ins + phase)
//	                                                    └─► rx_eclk, rx_wclk
//	  clkin (12 MHz, unrelated) ─────────────────────► cdr's PHASESTEP synchroniser
//	  part 5: reset_gen_adapter + gddr_sync (real RTL) on clkin ─► STOP, DIVRST, cdr_rstb
//	  part 6: reset_gen_kuntur + gddr_sync (real RTL) on Kuntur's clkin (32 MHz)
//	          ─► tx STOP, DIVRST / ODDRX4.RST, block_gen's rstb
//
//	The tb runs in six parts.
//	  1. Reference sweep. The tb steps the receive phase through a full turn
//	     (STEPS steps of 360°/STEPS), with cdr held in reset. At every step
//	     it re-locks the reference receiver rx_ref and counts bit errors
//	     over n blocks, then picks the centre of the longest error-free run
//	     (circular) and checks it against the centre the link timing
//	     predicts. DUT 1 runs beside it at every step.
//	  2. cdr sweeps on its own: its PHASESTEP pin drives the PLL model. It
//	     must measure every step once, choose the centre of its longest
//	     clean run, go there and lock.
//	  3. No data: as 2, with cdr's din held at 0. One turn, then give up.
//	  4. Lock lost a third of the way into a sweep: back to idle, then a
//	     whole new sweep, ending where part 2 ended.
//	  5. The companion's reset chain in the loop (lvds-tunnel.md §6.4,
//	     system-startup.md R-C8): the real reset_gen_adapter and gddr_sync,
//	     wired as in intan_rec_adapter. gddr_sync's STOP gates the receive
//	     ECLK (ECLKSYNC), its DDR_RESET resets the ECLKDIV and IDDRX4
//	     models, and cdr takes reset_gen_adapter's cdr_rstb and the PLL
//	     model's LOCK. Four events, each from a locked link:
//	       a. SW2 pressed 100 µs with Kuntur unlocked; LOCK 20 µs after;
//	       b. LOCK low 1 µs, clock running;
//	       c. cable out 50 µs (no TUN_CLK; LOCK falls 1 µs after, returns
//	          20 µs after the clock);
//	       d. SW2 pressed 100 µs, LOCK held.
//	     After each: READY fell and rose again; cdr was reset; the last
//	     DDR_RESET release before READY came with ECLK stopped; no
//	     PHASESTEP pulse while cdr was in reset; then one whole sweep,
//	     judged as part 2, locked at its centre after STEPS + centre
//	     pulses; bitsel = a fresh rx_ref's offset at that step (DIVRST
//	     moves the word boundary, so part 1's offsets no longer apply); the
//	     emulator reset (rst_n_emu) moved only for SW2.
//	     Skipped with +inj (cdr cannot lock).
//	  6. Kuntur's start-up chain in front (system-startup.md R-K7,
//	     lvds-tunnel.md §6.4), with part 5's chain still in the loop: the
//	     real reset_gen_kuntur and gddr_sync, wired as in kuntur_fpga.v.
//	     Kuntur's gddr_sync gates the transmit ECLK (ECLKSYNC STOP), resets
//	     the transmit ECLKDIV and both ODDRX4 (TUN_CLK and data low), and
//	     block_gen takes blk_rstb. The companion's LOCK now follows its
//	     reference: low after 1 µs without a TUN_CLK edge, high once it has
//	     run 20 µs. Three events at Kuntur, each from a locked link:
//	       e. rstb (E3, the MCU's PA10) low 100 µs;
//	       f. pll0 LOCK low 1 µs, clock running;
//	       g. MCO stops 100 µs: the PLL output stops 1 µs later, LOCK falls
//	          4 µs after that; LOCK returns 20 µs after the clocks.
//	     After each, part 5's checks (the emulator must not be reset) and:
//	     Kuntur's READY fell and rose and block_gen was reset; the last
//	     DDR_RESET release before READY came with the transmit ECLK
//	     stopped; TUN_CLK stopped for at least the 1.024 ms hold-off;
//	     block_gen's first 16 bytes after its reset are two whole blocks,
//	     cnt 0 and 1. Reported, not judged: the TUN_CLK burst before STOP
//	     (gddr_sync's INIT state releases DDR_RESET once with STOP low).
//	     Skipped with +inj.
//
//	What is modelled, and how simply
//	  - Transmitter: the real block_gen from kuntur_fpga.v, plus an 8:1
//	    serialiser sending D0 first (as in block_gen_tb.sv). TUN_CLK is the
//	    transmit ECLK, so clock and data are edge-aligned.
//	  - Link: a fixed delay on each pair, and uniform random jitter of
//	    ±jitter on each data transition.
//	  - PLL: rx_eclk = TUN_CLK delayed by ins + step × T/STEPS. 1:1, no PLL
//	    jitter. In part 1 the tb sets step directly. In parts 2-4 every
//	    falling edge of cdr's PHASESTEP adds one step 2.5 ns later
//	    (TN-02095 §14.5.9), so a glitch is a step, as on the real PLL;
//	    pulses shorter than 4 VCO cycles (2.5 ns) low or high fail.
//	    PHASEDIR must be 0 (delay).
//	  - Full turns. A turn delays the clock by one period while ECLKDIV
//	    keeps counting, so the word boundary moves 2 bits and the right
//	    bitsel at a given step drops by 2 per turn. The tb counts turns.
//	  - IDDRX4: samples D on both edges of rx_eclk, 8 bits per rx_wclk,
//	    first bit received in Q0 (+q7first puts it in Q7). Q changes
//	    100 ps after the rising edge of rx_wclk (SCLK). Any data
//	    transition within the window W before a sampling edge makes that
//	    sample random (a crude setup/hold model).
//	  - rx_ref: a behavioural aligner + checker, written from §6.6. It is
//	    the reference, not the design.
//	  - DUT 1: frame_aligner alone, the real RTL. At every part-1 step:
//	    reset, wait for rdy. Where rx_ref is error-free it must report valid
//	    with bitsel = rx_ref's offset s. After a match the tb also reads
//	    POS_CHECK bytes of dout/pos: every fixed byte (pos 0-3, 6, 7) must
//	    hold its pattern value.
//	  - DUT 2: cdr whole (§6.6 Sweep controller), on rx_wclk and clkin. Its
//	    per-step outputs (meas_en, meas_phase, meas_no_lock, meas_errors)
//	    are judged against rx_ref at the same absolute step:
//	      - eye interior (rx_ref clean at the step and 2 steps either side):
//	        locked, 0 errors (beyond the injected ones);
//	      - closed (rx_ref no-lock): not clean;
//	      - edge steps are reported, not judged: the two count different
//	        windows of a jittered link, so near the edge they legitimately
//	        differ.
//	    Its eye_center and eye_width must equal the longest clean run of its
//	    own measurements, and be within 1 and 4 steps of one of rx_ref's
//	    runs that is about as long as its longest (the two eyes 180° apart
//	    are usually equal, and either centre is valid, §6.6). It must end
//	    locked at the centre after STEPS + centre pulses, with bitsel =
//	    rx_ref's offset there, less 2 per turn.
//	    Limitation: with two equal eyes the tie rule (the earlier run
//	    stays) never picks a run that wraps from step 279 to 0, so the
//	    wrap-join in cdr's run tracking is exercised only when the wrapped
//	    run is strictly the longest.
//	    cdr's checker counts over CHK_N blocks (defparam NB_ERRORS).
//	  - ECLKSYNC / ECLKDIV (part 5): STOP takes effect on a falling edge of
//	    the PLL output, so ECLK stops low, glitch-free. DIVRST holds the
//	    divider (wclk low, Q = 0); released, it counts from the next ECLK
//	    edge. In parts 1-4 neither reaches the models.
//	  - Power-up: nothing is zeroed at t = 0. cdr's rstb and the resets it
//	    issues from idle clear every register these checks read.
//
//	Predicted eye, from the model above
//	  A sample taken o ps after a nominal data transition is clean when
//	  W + J < o < UI - J. So the eye is UI - W - 2J wide, centred at
//	  o = (UI + W) / 2. The sampling edge sits at
//	  o = d_clk + ins + phase - d_data (mod UI). That gives two centres per
//	  turn, 180° apart.
//
//	Plusargs (times in ps): +n=<blocks per step, rx_ref> (default 100)
//	  +d_data= +d_clk= +ins= +jitter= +win= +seed= +q7first
//	  +start=<step>: the PLL step at which parts 2-4 begin (default 0)
//	  +inj=<k>: flip bit 7 of cnt[7:0] in every k-th block at the
//	    transmitter, so any window of N blocks holds exactly N/k bit errors
//	    (n and CHK_N must be multiples of k). That base is subtracted from
//	    both counts, so the per-step checks are unchanged and test that cdr
//	    counts exactly: one that never counted would otherwise pass. No step
//	    is then error-free, so cdr must give up after one turn; part 4 is
//	    skipped. A cnt byte is used because the dout/pos check skips it.
//	  Define: +define+CHK_N=<blocks> (default 20) for cdr's N.
//	  Define: +define+MUT5=<k>, tb mutants that part 5 must catch:
//	    1 ECLKSYNC ignores STOP; 2 cdr reset by rstb only (not READY);
//	    3 gddr_sync RST = !rstb only (no rerun on lock loss).
//	  Define: +define+MUT6=<k>, mutants that part 6 must catch:
//	    1 Kuntur's ECLKSYNC ignores STOP; 2 block_gen reset by rstb only
//	    (not blk_rstb); 3 the companion's LOCK held through a Kuntur
//	    restart; 4 Kuntur's gddr_sync RST = !rstb only.
//
//	Run
//	  Icarus (from this directory). intan_rec_adapter.v and kuntur_fpga.v
//	    both `include the common modules and cannot share a compilation
//	    unit, so the receiver's modules are cut out of intan_rec_adapter.v
//	    on every run (tr: the file mixes CR and CRLF line endings).
//	    gddr_sync.v comes in through kuntur_fpga.v (and reset_gen_kuntur
//	    with it).
//	    tr '\r' '\n' < ../intan_rec_adapter.v | awk '/^module (cdr|cdr_fsm|frame_aligner|frame_checker|reset_gen_adapter) /,/^endmodule/' > /tmp/cdr_rtl.v
//	    iverilog -g2012 -grelative-include -s cdr_tb -o cdr.vvp cdr_tb.sv /tmp/cdr_rtl.v
//	    vvp cdr.vvp
//
//	Accepted when both are ALL PASS:
//	    vvp cdr.vvp +n=20
//	    vvp cdr.vvp +n=20 +inj=4
//	and each +define+MUT5=1, 2, 3 build must FAIL in part 5 (2026-10-29:
//	4, 6 and 13 failures; MUT5=3 runs to the timeout, a few minutes), and
//	each +define+MUT6=1..4 build in part 6.
//	Expected with the defaults: two error-free runs about 140 steps apart,
//	each about 128 steps wide. With +q7first no step locks, because 01 80
//	arrives as 80 01; the tb then reports FAIL (§6.6, bit order).

`timescale 1ns/1ps
`include "../../../../headstage/source/impl_1/kuntur_fpga.v"

// Behavioural reference receiver: fabric aligner + pattern checker (§6.6).
// Clocked by rx_wclk. Input q is one IDDRX4 word, first-received bit in q[0].
module rx_ref (
	input  wire       clk,
	input  wire       rst,		// synchronous: back to hunt, clears everything
	input  wire       count_en,	// count bit errors only while high
	input  wire [7:0] q,
	output reg        locked,
	output integer    errors,
	output integer    blocks	// blocks completed while locked and counting
);
	localparam [31:0] PRE = 32'h5AA55AA5;	// A5 5A A5 5A, oldest byte in [7:0]

	reg  [39:0] h = 0;		// last 5 words, newest in [39:32]
	reg  [63:0] blk;		// aligned block being assembled, oldest byte in [7:0]
	reg  [2:0]  s;			// bit offset
	reg  [2:0]  pos;		// position in block of the NEXT aligned byte
	reg  [15:0] ecnt;		// expected cnt (own counter once locked)
	reg  [15:0] pcnt;
	integer     state, clean, k;

	localparam HUNT = 0, ACQ = 1, LOCK = 2;

	function integer popcount8(input [7:0] v);
		integer i;
		begin
			popcount8 = 0;
			for (i = 0; i < 8; i = i + 1) popcount8 = popcount8 + v[i];
		end
	endfunction

	reg [39:0] hn;
	reg [7:0]  ab, eb;
	reg        found;

	initial begin locked = 0; errors = 0; blocks = 0; state = HUNT; end

	always @(posedge clk) begin
		hn = {q, h[39:8]};
		h <= hn;
		if (rst) begin
			state = HUNT; locked = 0; errors = 0; blocks = 0;
		end else case (state)
		HUNT: begin
			// The preamble has just completed in the aligned stream at offset s.
			found = 0;
			for (k = 7; k >= 0; k = k - 1)
				if (hn[k +: 32] == PRE) begin found = 1; s = k; end
			if (found) begin
				blk   = {PRE, 32'h0};
				pos   = 4;
				clean = 0;
				state = ACQ;
			end
		end
		default: begin
			// Aligned byte at offset s: the newest whole byte in hn[s+31:s].
			ab  = hn[s + 24 +: 8];
			blk = {ab, blk[63:8]};
			if (state == LOCK && count_en) begin
				case (pos)
					3'd0, 3'd2: eb = 8'hA5;
					3'd1, 3'd3: eb = 8'h5A;
					3'd4:       eb = ecnt[7:0];
					3'd5:       eb = ecnt[15:8];
					3'd6:       eb = 8'h01;
					default:    eb = 8'h80;
				endcase
				errors = errors + popcount8(ab ^ eb);
			end
			if (pos == 3'd7) begin
				if (state == ACQ) begin
					// Lock = two consecutive clean blocks, cnt + 1 (§6.6).
					if (blk[31:0] == PRE && blk[63:48] == 16'h8001 &&
					    (clean == 0 || blk[47:32] == pcnt + 16'd1)) begin
						clean = clean + 1;
						pcnt  = blk[47:32];
						if (clean == 2) begin
							state  = LOCK;
							locked = 1;
							ecnt   = pcnt + 16'd1;
						end
					end else
						state = HUNT;
				end else begin
					ecnt = ecnt + 16'd1;
					if (count_en) blocks = blocks + 1;
				end
			end
			pos = pos + 3'd1;
		end
		endcase
	end
endmodule


module cdr_tb();
	// Link timing, ps. clk = 45.539955 MHz (pll0 CLKOP before the A.7 retune).
	localparam real T_PS  = 1.0e6 / 45.539955;	// ECLK period, 21958.7 ps
	localparam real UI_PS = T_PS / 2.0;		// one bit, DDR
	localparam integer STEPS = 280;			// 8 × (DIVOS_ACTUAL_STR + 1), §6.4

	integer n_blocks, seed, inj;
	real d_data, d_clk, ins, jit, win;
	reg  q7first;

	// ---- Transmitter: real block_gen + D0-first serialiser on tx_eclk ----
	// tx_pll is pll0's CLKOP; ECLKSYNC turns it into tx_eclk. In part 6
	// Kuntur's gddr_sync drives STOP and DDR_RESET (ECLKDIV.DIVRST,
	// ODDRX4.RST) and reset_gen_kuntur resets block_gen; before that
	// neither reaches the models and block_gen takes the tb's rstb.
	reg  tx_pll = 0, rstb = 0;
	reg  k_pll_run = 1;		// part 6: 0 = Kuntur's PLL output stopped (MCO gone)
	reg  p6 = 0;
	wire k_stop, k_ddr_rst, k_blk_rstb;
	reg  [2:0] ph = 0;
	reg  tx_wclk = 0;
	wire [7:0] dout;
	reg  [7:0] txb = 0;
	reg  tx_bit = 0;

`ifdef MUT6
	wire tx_gen_rstb = (p6 && `MUT6 != 2) ? k_blk_rstb : rstb;
	wire k_stop_m = (`MUT6 == 1) ? 1'b0 : p6 & k_stop;
`else
	wire tx_gen_rstb = p6 ? k_blk_rstb : rstb;
	wire k_stop_m = p6 & k_stop;
`endif
	wire k_divrst = p6 & k_ddr_rst;
	block_gen tx_gen (.rstb(tx_gen_rstb), .clk(tx_wclk), .dout(dout));

	always #(T_PS / 2000.0) tx_pll = k_pll_run ? ~tx_pll : 1'b0;
	// ECLKSYNC: STOP acts on a falling edge, as on the receive side
	reg  tx_en = 1;
	always @(negedge tx_pll) tx_en <= !k_stop_m;
	wire tx_eclk = tx_pll & tx_en;

	// DIVRST / ODDRX4.RST: hold the divider, wclk low, both outputs low.
	always @(posedge k_divrst) begin ph = 3'd7; tx_wclk <= 0; tx_bit <= 0; end

	// A new bit at every tx_eclk edge, so the clock is edge-aligned with data.
	always @(tx_eclk) if (!k_divrst) begin
		ph      <= ph + 3'd1;
		tx_wclk <= (ph + 3'd1) >= 3'd4;
		if (ph == 3'd7)		// dout is registered: cnt0 == 5 means it holds cnt[7:0]
			txb <= dout ^ ((inj > 0 && tx_gen.cnt0 == 3'd5 && tx_gen.cnt1 % inj == 0) ? 8'h80 : 8'h00);
		tx_bit  <= (ph == 3'd7) ? dout[0] : txb[ph + 3'd1];
	end

	// ---- Link: fixed delays, jitter on data transitions (transport) ----
	reg  data_line = 0, clk_line = 0;
	real last_change = -1.0e9;
	real jd;

	always @(tx_bit) begin
		jd = ($urandom % 2001) / 1000.0 - 1.0;		// -1 .. +1
		data_line <= #((d_data + jit * jd) / 1000.0) tx_bit;
	end
	reg  cable = 1;		// part 5: 0 = cable out, no TUN_CLK
	wire tun_clk = tx_eclk & !k_divrst;	// ODDRX4 oddrx4_0: 1,0,1,0 on tx_eclk; 0 in RST
	always @(tun_clk) clk_line <= #(d_clk / 1000.0) (cable & tun_clk);
	always @(data_line) last_change = $realtime;

	// ---- PLL model: rx_eclk = clk_line delayed by ins + phase ----
	integer step = 0;
	reg rx_eclk = 0;
	always @(clk_line)
		rx_eclk <= #((ins + step * T_PS / STEPS) / 1000.0) clk_line;

	// PHASESTEP pin (parts 2-4), modelled as TN-02095 §14.5.9 states it:
	// every falling edge is one delaying step, taking effect 4 VCO cycles
	// (2.5 ns) later. So a glitch IS a step here. Pulse widths are checked
	// against the 4-VCO-cycle minimum, low and high (ps_viol).
	// The wrap from STEPS-1 to 0 shortens the delay by T - T/STEPS; every
	// edge then lands T/STEPS after one already scheduled with the same
	// value, so it is seamless.
	localparam real PS_MIN_NS = 2.5;		// 4 VCO cycles at 1594 MHz
	wire    cdr_phasedir, cdr_phasestep;
	reg     pll_dyn = 0;
	integer ps_pulses = 0, ps_viol = 0;
	real    ps_fall = -1.0e9, ps_rise = -1.0e9, ps_wmin = 1.0e9;
	// Full turns since t = 0. A turn delays the clock by one period while
	// ECLKDIV keeps counting edges, so the word boundary moves 2 bits along
	// the data, and the right bitsel at a given step drops by 2 per turn.
	// The real PLL does the same; rx_ref's offsets were measured in turn 0.
	integer turns = 0;
	always @(negedge cdr_phasestep) if (pll_dyn) begin
		if ($realtime - ps_rise < PS_MIN_NS) ps_viol = ps_viol + 1;
		if ($realtime - ps_rise < ps_wmin) ps_wmin = $realtime - ps_rise;
		ps_fall = $realtime;
		ps_pulses = ps_pulses + 1;
		#(PS_MIN_NS);
		step = (step + 1) % STEPS;
		if (step == 0) turns = turns + 1;
	end
	always @(posedge cdr_phasestep) if (pll_dyn) begin
		if ($realtime - ps_fall < PS_MIN_NS) ps_viol = ps_viol + 1;
		if ($realtime - ps_fall < ps_wmin) ps_wmin = $realtime - ps_fall;
		ps_rise = $realtime;
	end

	// clkin: the 12 MHz board oscillator, unrelated to the link clocks
	reg clkin = 0;
	always #(1.0e3 / 12.0 / 2.0) clkin = ~clkin;
	integer ref_base = 0, inj_under = 0;

	// ---- ECLKSYNC + ECLKDIV ÷4 (arbitrary start phase) + IDDRX4 model ----
	// Part 5 only (p5): gddr_sync's STOP and DDR_RESET reach them.
	reg  p5 = 0;
	wire gs_stop, gs_ddr_rst;
`ifdef MUT5
	wire stop_m = (`MUT5 == 1) ? 1'b0 : p5 & gs_stop;
`else
	wire stop_m = p5 & gs_stop;
`endif
	wire divrst = p5 & gs_ddr_rst;
	// ECLKSYNC: STOP acts on a falling edge, so ECLK stops low and restarts
	// with a whole high phase.
	reg  eclk_en = 1;
	always @(negedge rx_eclk) eclk_en <= !stop_m;
	wire rx_eclk_g = rx_eclk & eclk_en;

	reg  [2:0] rph;
	reg  rx_wclk = 0;
	reg  [7:0] sh = 0, q = 0;
	reg  smp;

	// DIVRST / IDDRX4.RST: hold the divider, wclk low, Q = 0. Released, it
	// counts from the next ECLK edge: a new word boundary.
	always @(posedge divrst) begin rph = 3'd7; rx_wclk <= 0; q <= 0; end

	always @(rx_eclk_g) if (!divrst) begin
		// Sample: random if a transition fell inside the window before the edge.
		if ($realtime - last_change < win / 1000.0) smp = $urandom % 2;
		else smp = data_line;
		sh = {smp, sh[7:1]};			// first bit received ends in sh[0]
		rph = rph + 3'd1;
		// Q is registered on SCLK: it changes just after the rising edge.
		if (rph == 3'd4)
			q <= #0.1 (q7first ? {sh[0], sh[1], sh[2], sh[3], sh[4], sh[5], sh[6], sh[7]} : sh);
		rx_wclk <= rph[2];
	end

	// ---- Reference receiver ----
	reg  rx_rst = 1, count_en = 0;
	wire locked;
	integer errors, blocks;
	rx_ref ref0 (.clk(rx_wclk), .rst(rx_rst), .count_en(count_en), .q(q),
	             .locked(locked), .errors(errors), .blocks(blocks));

	// ---- DUT 1: frame_aligner (intan_rec_adapter.v) ----
	// The aligner hunts all 8 bitsels on its own. Reset, wait for rdy, read
	// valid and bitsel. After a match, check POS_CHECK bytes of dout against
	// the pattern at pos.
	localparam integer AL_TIMEOUT = 8 * 16 + 32;	// wclk to wait for rdy: 8 windows + slack
	localparam integer POS_CHECK  = 16;		// bytes of dout/pos checked per match
	reg        al_rst = 1;
	wire [7:0] al_dout;
	wire [8:0] al_pos;
	wire [2:0] al_bitsel;
	wire       al_valid, al_rdy;
	frame_aligner al0 (.rst_sync(al_rst), .clk(rx_wclk), .din(q),
	                   .dout(al_dout), .pos(al_pos), .bitsel(al_bitsel),
	                   .valid(al_valid), .rdy(al_rdy));

	integer al_tmax = 0;	// longest search seen, whole run
	integer al_pe;		// fixed bytes of dout that disagreed with pos, this step
	integer pos_bad_steps = 0, pos_first_bad = -1;
	reg [7:0] pos_bad_dout, pos_dout_tab [0:STEPS-1];
	reg [8:0] pos_bad_pos,  pos_pos_tab  [0:STEPS-1];
	reg       pos_bad_tab [0:STEPS-1];			// per step: dout/pos disagreed

	// Expected byte at a fixed position; returns 0 for pos 4, 5 (cnt).
	function fixed_pos(input [8:0] p);
		fixed_pos = (p <= 3) || (p == 6) || (p == 7);
	endfunction
	function [7:0] pat_byte(input [8:0] p);
		case (p)
			0, 2:    pat_byte = 8'hA5;
			1, 3:    pat_byte = 8'h5A;
			6:       pat_byte = 8'h01;
			default: pat_byte = 8'h80;
		endcase
	endfunction

	// One character per step: '0'..'7' = valid at that bitsel; '-' = rdy
	// without valid (all 8 failed); 'H' = rdy never came.
	reg [7:0] al_tab [0:STEPS-1];
	task al_scan(input integer st);
		integer cyc;
		begin
			@(posedge rx_wclk); #1 al_rst = 1;
			@(posedge rx_wclk); #1 al_rst = 0;
			cyc = 0;
			while (!al_rdy && cyc < AL_TIMEOUT) begin @(posedge rx_wclk); #1 cyc = cyc + 1; end
			if (!al_rdy)       al_tab[st] = "H";
			else if (al_valid) al_tab[st] = "0" + al_bitsel;
			else               al_tab[st] = "-";
			if (al_rdy && cyc > al_tmax) al_tmax = cyc;
			al_pe = 0;
			if (al_rdy && al_valid)
				repeat (POS_CHECK) begin
					if (al_pos > 7 || (fixed_pos(al_pos) && al_dout != pat_byte(al_pos))) begin
						if (al_pe == 0) begin pos_bad_dout = al_dout; pos_bad_pos = al_pos; end
						al_pe = al_pe + 1;
					end
					@(posedge rx_wclk); #1;
				end
			pos_bad_tab[st]  = al_pe != 0;
			pos_dout_tab[st] = pos_bad_dout;
			pos_pos_tab[st]  = pos_bad_pos;
		end
	endtask

	// ---- DUT 2: cdr, whole (§6.6 sweep controller) ----
	// N for cdr's checker. Small, so the tb runs in minutes; +define+CHK_N=
	// to change it (n and CHK_N must both be multiples of +inj).
`ifndef CHK_N
`define CHK_N 20
`endif
	localparam integer CHK_N = `CHK_N;
	defparam cdr0.checker0.NB_ERRORS = CHK_N;
	reg  cdr_rstb = 0;
	reg  cdr_lock = 1;
	reg  data_off = 0;					// part 3: cdr sees no signal
	wire [7:0] cdr_din = data_off ? 8'h00 : q;
	wire       cdr_rdy, cdr_valid, meas_en, meas_no_lock;
	wire [8:0] cdr_centre, cdr_eye_width, meas_phase;
	wire [15:0] meas_errors;
	// Part 5: cdr is reset by reset_gen_adapter and sees the PLL model's LOCK.
	wire       rg_cdr_rstb;
	reg        pll_lock_m = 1;
	reg        c_rstb = 1;			// the companion's SW2 (G14)
`ifdef MUT5
	wire       cdr_rstb_dut = p5 ? ((`MUT5 == 2) ? c_rstb : rg_cdr_rstb) : cdr_rstb;
`else
	wire       cdr_rstb_dut = p5 ? rg_cdr_rstb : cdr_rstb;
`endif
	wire       cdr_lock_dut = p5 ? pll_lock_m : cdr_lock;
	cdr cdr0 (.rstb(cdr_rstb_dut), .wclk(rx_wclk), .clk(clkin), .lock(cdr_lock_dut), .din(cdr_din),
	          .pll_phasedir(cdr_phasedir), .pll_phasestep_sync(cdr_phasestep),
	          .rdy(cdr_rdy), .valid(cdr_valid), .eye_center(cdr_centre), .eye_width(cdr_eye_width),
	          .meas_en(meas_en), .meas_phase(meas_phase), .meas_no_lock(meas_no_lock),
	          .meas_errors(meas_errors));

	// Per-step measurements of the current cdr sweep, indexed by meas_phase.
	// m_seen counts pulses per phase; m_abs is the PLL model's step at the
	// pulse (absolute, to compare with rx_ref).
	integer m_err [0:STEPS-1];
	reg     m_nl  [0:STEPS-1];
	integer m_seen [0:STEPS-1];
	integer m_abs  [0:STEPS-1];
	integer m_count = 0, m_phase_bad = 0, m_restarts = 0, m_last = -1;
	integer sw_start = 0;			// PLL step at which the current sweep began
	task m_clear;
		integer k;
		begin
			for (k = 0; k < STEPS; k = k + 1) begin m_seen[k] = 0; m_err[k] = 65535; m_nl[k] = 1; end
			m_count = 0; m_phase_bad = 0; m_last = -1;
		end
	endtask
	// The PLL step as of the previous rx_wclk edge. meas_en is registered, so
	// the tb sees it one wclk after the FSM decided; on the no-lock paths the
	// FSM starts that step's PHASESTEP pulse in the same cycle, and through
	// the 12 MHz synchroniser it can reach the pin before this edge.
	integer step_prev = 0;
	always @(posedge rx_wclk) step_prev <= step;
	always @(posedge rx_wclk) if (meas_en) begin
		if (p5 && m_last < 0) sw_start = step_prev;	// part 5: the sweep starts where cdr was released
		if (meas_phase == 0 && m_last >= 0) begin
			// a new sweep (after a lock loss): keep only this one
			m_clear; m_restarts = m_restarts + 1; sw_start = step_prev;
		end
		if (meas_phase >= STEPS) m_phase_bad = m_phase_bad + 1;
		else begin
			m_seen[meas_phase] = m_seen[meas_phase] + 1;
			m_err[meas_phase]  = meas_errors;
			m_nl[meas_phase]   = meas_no_lock;
			m_abs[meas_phase]  = step_prev;
			if (step_prev != (sw_start + meas_phase) % STEPS) m_phase_bad = m_phase_bad + 1;
		end
		m_count = m_count + 1;
		m_last  = meas_phase;
	end

	// ---- Part 5: the companion's reset chain, real RTL ----
	// Wired as in intan_rec_adapter (lvds-tunnel.md §6.4). It runs from
	// t = 0 with c_rstb and LOCK high; its outputs reach the clock models
	// and cdr only once p5 is set.
	wire rg_rst_n_sup, rg_lock_sync, rg_gddr_rst, rg_rst_n_emu, gs_ready;
	reset_gen_adapter rg0 (.clkin(clkin), .rstb(c_rstb), .pll_lock(pll_lock_m), .wclk(rx_wclk),
		.gddr_rdy(gs_ready), .rst_n_sup(rg_rst_n_sup), .lock_sync(rg_lock_sync),
		.gddr_rst(rg_gddr_rst), .cdr_rstb(rg_cdr_rstb), .rst_n_emu(rg_rst_n_emu));
	// 1 ps late, as in reset_gen_adapter_tb.sv: GSR resets gddr_sync before
	// its always blocks could see a rising rst at t = 0.
`ifdef MUT5
	wire #0.001 gs_rst = (`MUT5 == 3) ? !c_rstb : rg_gddr_rst;
`else
	wire #0.001 gs_rst = rg_gddr_rst;
`endif
	gddr_sync gs0 (.rst(gs_rst), .sync_clk(clkin), .start(rg_lock_sync),
		.stop(gs_stop), .ddr_reset(gs_ddr_rst), .ready(gs_ready));
	// GSR stand-in: reset_gen_adapter's power-up values (none in its RTL)
	initial begin
		rg0.cnt0 = 0; rg0.rst_n_sup = 0; rg0.pll_lock_reg = 0; rg0.lock_sync = 0;
		rg0.gddr_rst = 1; rg0.cdr_rstb_1 = 0; rg0.cdr_rstb = 0;
	end

	// Part 5 monitors, cleared per event
	integer p5_ps_in_rst, p5_ready_falls, p5_ready_rises, p5_cdr_resets, p5_rel_running, p5_emu_resets;
	reg     p5_rel_stopped = 0;
	real    t_ready = 0, t_cdr_rel = 0;
	always @(negedge cdr_phasestep) if (p5 && !cdr_rstb_dut) p5_ps_in_rst = p5_ps_in_rst + 1;
	always @(negedge gs_ddr_rst) if (p5) p5_rel_stopped = !eclk_en;
	always @(posedge gs_ready) if (p5) begin
		p5_ready_rises = p5_ready_rises + 1; t_ready = $realtime;
		if (!p5_rel_stopped) p5_rel_running = p5_rel_running + 1;
	end
	always @(negedge gs_ready) if (p5) p5_ready_falls = p5_ready_falls + 1;
	always @(negedge cdr_rstb_dut) if (p5) p5_cdr_resets = p5_cdr_resets + 1;
	always @(posedge cdr_rstb_dut) if (p5) begin t_cdr_rel = $realtime; ps_pulses = 0; end
	always @(negedge rg_rst_n_emu) if (p5) p5_emu_resets = p5_emu_resets + 1;

	// ---- Part 6: Kuntur's start-up chain, real RTL ----
	// reset_gen_kuntur and gddr_sync, wired as in kuntur_fpga.v, on Kuntur's
	// clkin (32 MHz MCO, unrelated to everything else here). Like part 5's
	// chain it runs from t = 0, PLL locked, and reaches the transmitter
	// models and block_gen only once p6 is set.
	localparam real K_T_CLKIN = 31.25;
	localparam real K_HOLDOFF_NS = 32768 * K_T_CLKIN;	// R-K7 hold-off, 1.024 ms
	reg  kclkin = 0, k_mco = 1, k_rstb = 1, k_lock = 1;
	always #(K_T_CLKIN / 2.0) kclkin = k_mco ? ~kclkin : 1'b0;
	wire k_lock_sync, k_rst_n_clkin, k_gddr_rst, k_rst_n_clk, k_ready;
	reset_gen_kuntur krg (.clkin(kclkin), .rstb(k_rstb), .pll_lock(k_lock), .clk(tx_pll), .wclk(tx_wclk),
		.gddr_rdy(k_ready), .lock_sync(k_lock_sync), .rst_n_clkin(k_rst_n_clkin),
		.gddr_rst(k_gddr_rst), .rst_n_clk(k_rst_n_clk), .blk_rstb(k_blk_rstb));
`ifdef MUT6
	wire #0.001 k_gs_rst = (`MUT6 == 4) ? !k_rstb : k_gddr_rst;
`else
	wire #0.001 k_gs_rst = k_gddr_rst;	// 1 ps late, as gs_rst
`endif
	gddr_sync kgs (.rst(k_gs_rst), .sync_clk(kclkin), .start(k_lock_sync),
		.stop(k_stop), .ddr_reset(k_ddr_rst), .ready(k_ready));
	// GSR stand-in: reset_gen_kuntur's power-up values (none in its RTL)
	initial begin
		krg.cnt0 = 0; krg.rst_n_clkin = 0; krg.pll_lock_reg = 0; krg.lock_sync = 0;
		krg.gddr_rst = 1; krg.rst_n_clk_reg = 0; krg.rst_n_clk = 0;
		krg.blk_rstb_1 = 0; krg.blk_rstb = 0;
	end

	// The companion's PLL LOCK in part 6, from its reference (clk_line):
	// low after 1 µs without an edge, high again once the clock has run
	// 20 µs (part 5c's numbers). +define+MUT6=3 holds it high.
	reg  p6_lock_auto = 0;
	real t_ref_edge = 0, t_ref_run = 0;
	always @(clk_line) begin
		if ($realtime - t_ref_edge > 100.0) t_ref_run = $realtime;
		t_ref_edge = $realtime;
	end
	always #100 if (p6_lock_auto) begin
		if ($realtime - t_ref_edge > 1000.0) pll_lock_m = 0;
		else if ($realtime - t_ref_run >= 20000.0) pll_lock_m = 1;
	end

	// Part 6 monitors, cleared per event
	integer k_ready_falls, k_ready_rises, k_blk_falls, k_rel_running, k_fb_n, k_fb_lead, k_burst;
	real    t_tun_edge = 0, k_gap_max = 0, t_burst0 = 0, t_burst1 = 0;
	reg [7:0] k_fb [0:15];
	// As in part 5, the last DDR_RESET release before READY must come with
	// ECLK stopped. gddr_sync's INIT state releases it once before that,
	// with STOP low, as soon as rst falls: TUN_CLK runs until STOP. That
	// burst is measured (k_burst, rising edges before STOP first rises),
	// not judged.
	reg     k_rel_stopped = 0, k_stop_seen = 0;
	always @(posedge k_stop) if (p6) k_stop_seen = 1;
	always @(negedge k_ddr_rst) if (p6) k_rel_stopped = !tx_en;
	always @(posedge k_ready) if (p6) begin
		k_ready_rises = k_ready_rises + 1;
		if (!k_rel_stopped) k_rel_running = k_rel_running + 1;
	end
	always @(posedge tun_clk) if (p6 && !k_ready && !k_stop_seen) begin
		if (k_burst == 0) t_burst0 = $realtime;
		t_burst1 = $realtime;
		k_burst = k_burst + 1;
	end
	always @(negedge k_ready) if (p6) k_ready_falls = k_ready_falls + 1;
	always @(negedge tx_gen_rstb) if (p6) k_blk_falls = k_blk_falls + 1;
	// Longest time without a TUN_CLK edge at Kuntur's pin
	always @(tun_clk) if (p6) begin
		if ($realtime - t_tun_edge > k_gap_max) k_gap_max = $realtime - t_tun_edge;
		t_tun_edge = $realtime;
	end
	// The bytes block_gen sends after its reset: leading zeros, then 16 bytes.
	always @(posedge tx_gen_rstb) if (p6) begin k_fb_n = 0; k_fb_lead = 0; end
	always @(posedge tx_wclk) if (p6 && tx_gen_rstb && k_fb_n < 16) begin
		if (k_fb_n == 0 && dout == 8'h00) k_fb_lead = k_fb_lead + 1;
		else begin k_fb[k_fb_n] = dout; k_fb_n = k_fb_n + 1; end
	end
	// Two whole blocks, cnt 0 and 1
	function [7:0] fb_exp(input integer j);
		case (j % 8)
			0, 2:    fb_exp = 8'hA5;
			1, 3:    fb_exp = 8'h5A;
			4:       fb_exp = j / 8;
			5:       fb_exp = 8'h00;
			6:       fb_exp = 8'h01;
			default: fb_exp = 8'h80;
		endcase
	endfunction

	// Longest circular run of ok_v[] = 1: sets run_len, run_start.
	reg     ok_v [0:STEPS-1];
	integer run_len, run_start;
	task longest_run;
		integer st, r;
		begin
			run_len = 0; run_start = 0;
			for (st = 0; st < STEPS; st = st + 1)
				if (ok_v[st] && !ok_v[(st + STEPS - 1) % STEPS]) begin
					r = 0;
					while (r < STEPS && ok_v[(st + r) % STEPS]) r = r + 1;
					if (r > run_len) begin run_len = r; run_start = st; end
				end
			if (run_len == 0 && ok_v[0]) begin run_len = STEPS; run_start = 0; end
		end
	endtask

	// rx_ref's error-free runs (circular), filled after part 1. With equal
	// eyes 180° apart (§6.6) either centre is valid, so cdr's centre is
	// judged against every run about as long as the longest.
	integer rr_n = 0;
	integer rr_start [0:15], rr_len [0:15];
	task ref_runs;
		integer st, r;
		begin
			rr_n = 0;
			for (st = 0; st < STEPS; st = st + 1)
				if (ok_v[st] && !ok_v[(st + STEPS - 1) % STEPS] && rr_n < 16) begin
					r = 0;
					while (r < STEPS && ok_v[(st + r) % STEPS]) r = r + 1;
					rr_start[rr_n] = st; rr_len[rr_n] = r; rr_n = rr_n + 1;
				end
			if (rr_n == 0 && ok_v[0]) begin rr_start[0] = 0; rr_len[0] = STEPS; rr_n = 1; end
		end
	endtask
	// 1 if centre cen (absolute) and width w match an rx_ref run within 1
	// and 4 steps, and that run is within 4 steps of the longest.
	function ref_match(input integer cen, input integer w);
		integer j;
		begin
			ref_match = 0;
			for (j = 0; j < rr_n; j = j + 1)
				if (rr_len[j] >= ref_len - 4 && w - rr_len[j] <= 4 && rr_len[j] - w <= 4 &&
				    cdist(cen, (rr_start[j] + rr_len[j] / 2) % STEPS) <= 1)
					ref_match = 1;
		end
	endfunction

	// Walk the PLL model to abs one step at a time, as PHASESTEP would. A
	// jump of many steps reorders rx_eclk edges in this model, which moves
	// the ECLKDIV word boundary, and rx_ref's offsets would no longer apply.
	task walk_to(input integer abs);
		begin
			while (step != abs) begin
				repeat (2) @(posedge rx_wclk);
				step = (step + 1) % STEPS;
				if (step == 0) turns = turns + 1;
			end
			repeat (16) @(posedge rx_wclk);		// let the new delay settle
		end
	endtask

	// One closed-loop run of cdr from PLL step abs: reset, release, wait for
	// rdy (or the drop/restore of lock, if drop_at >= 0: lock goes low after
	// that many measurements, for drop_len wclk). Sets cl_* below.
	localparam integer STEP_TIMEOUT = AL_TIMEOUT + 8 * (100 + CHK_N + 8) + 64;	// wclk, one phase
	integer cl_cyc, cl_final, cl_pulses, cl_turns;
	reg     cl_rdy, cl_valid, cl_idle_seen;
	reg [2:0] cl_bitsel;
	task cdr_sweep(input integer abs, input integer drop_at, input integer drop_len);
		begin
			cdr_rstb = 0; cdr_lock = 1; pll_dyn = 0;
			walk_to(abs);
			m_clear; m_restarts = 0; sw_start = abs;
			ps_pulses = 0;
			@(posedge rx_wclk); #1 pll_dyn = 1;
			@(posedge rx_wclk); @(posedge rx_wclk); #1 cdr_rstb = 1;
			cl_cyc = 0; cl_idle_seen = 0;
			while (!cdr_rdy && cl_cyc < (2 * STEPS + 2) * STEP_TIMEOUT) begin
				@(posedge rx_wclk); #1 cl_cyc = cl_cyc + 1;
				if (drop_at >= 0 && m_count == drop_at && cdr_lock) begin
					cdr_lock = 0;
					repeat (drop_len) @(posedge rx_wclk);
					#1 cl_idle_seen = !meas_en && !cdr_rdy;
					sw_start = step;			// the new sweep starts here
					cdr_lock = 1;
					drop_at = -1;
				end
			end
			// let a last PHASESTEP pulse finish in the model
			repeat (8) @(posedge rx_wclk);
			cl_rdy    = cdr_rdy;
			cl_valid  = cdr_valid;
			cl_bitsel = cdr0.aligner0.bitsel;
			cl_final  = step;
			cl_pulses = ps_pulses;
			cl_turns  = turns;
			pll_dyn = 0;
		end
	endtask

	// ---- Tables and results ----
	integer err_tab [0:STEPS-1];
	reg     nolock_tab [0:STEPS-1];
	reg [7:0] s_tab [0:STEPS-1];
	integer al_bad, al_clean;
	integer chk_base = 0;
	integer i, t, best_len, best_start, run, start, c, cen;
	real    pred0, pred1, o0;
	integer pstep0, pstep1, d0, d1, dmin, fails, ok_width_lo;
	integer start_step, ref_cen, ref_len, m_cen, m_len, exp_abs, exp_bit, k;
	integer core, bad_core, bad_closed, edg, edg_agree, under, missing;
	reg     ref_clean, m_clean;
	integer p2_final, p2_cen;
	reg [8*STEPS-1:0] line;

	function integer cdist(input integer a, input integer b);	// circular distance
		integer d;
		begin
			d = a - b; if (d < 0) d = -d;
			if (STEPS - d < d) d = STEPS - d;
			cdist = d;
		end
	endfunction

	task show_tab;		// prints line, 70 characters per row
		integer r;
		for (r = 0; r < STEPS; r = r + 70)
			$display("  %3d  %0s", r, line[8*(STEPS-r)-1 -: 8*70]);
	endtask

	// Character for an error count: '.' 0, '1'..'5' log10 + 1, '#' >= 10^5.
	function [7:0] err_char(input integer e);
		if (e == 0)           err_char = ".";
		else if (e < 10)      err_char = "1";
		else if (e < 100)     err_char = "2";
		else if (e < 1000)    err_char = "3";
		else if (e < 10000)   err_char = "4";
		else if (e < 100000)  err_char = "5";
		else                  err_char = "#";
	endfunction

	// Judge one cdr sweep's per-step measurements against rx_ref, and its
	// centre against both rx_ref and its own measurements.
	task judge_sweep(input [8*16-1:0] name);
		begin
			$display("  %0s: cdr per step, by absolute step ('.' 0 errors beyond injected, '1'..'5' log10+1, 'X' no lock, 'U' under-count, ' ' no measurement):", name);
			missing = 0;
			for (i = 0; i < STEPS; i = i + 1) begin
				k = (i - sw_start + STEPS) % STEPS;	// relative phase of absolute step i
				if (m_seen[k] != 1)          begin c = " "; missing = missing + 1; end
				else if (m_nl[k])            c = "X";
				else if (m_err[k] < chk_base) c = "U";
				else                         c = err_char(m_err[k] - chk_base);
				line[8*(STEPS-1-i) +: 8] = c;
			end
			show_tab;
			if (m_count == STEPS && missing == 0 && m_phase_bad == 0)
				$display("PASS  %0s: one measurement per step, %0d, meas_phase = PLL steps since the sweep began", name, m_count);
			else begin
				$display("FAIL  %0s: %0d measurements, %0d steps without exactly one, %0d with meas_phase off the PLL", name, m_count, missing, m_phase_bad);
				fails = fails + 1;
			end

			core = 0; bad_core = 0; bad_closed = 0; edg = 0; edg_agree = 0; under = 0;
			for (i = 0; i < STEPS; i = i + 1) begin
				k = (i - sw_start + STEPS) % STEPS;
				m_clean = m_seen[k] == 1 && !m_nl[k] && m_err[k] == chk_base;
				if (m_seen[k] == 1 && !m_nl[k] && m_err[k] < chk_base) under = under + 1;
				ref_clean = 1;
				for (t = -2; t <= 2; t = t + 1)
					if (nolock_tab[(i + t + STEPS) % STEPS] || err_tab[(i + t + STEPS) % STEPS] != 0) ref_clean = 0;
				if (ref_clean) begin
					core = core + 1;
					if (!m_clean) bad_core = bad_core + 1;
				end else if (nolock_tab[i]) begin
					if (m_clean) bad_closed = bad_closed + 1;
				end else begin
					edg = edg + 1;
					if ((err_tab[i] == 0) == m_clean) edg_agree = edg_agree + 1;
				end
			end
			if (core > 0 && bad_core == 0)
				$display("PASS  %0s: locked with %0d errors%0s at all %0d eye-interior steps", name, chk_base,
					(inj > 0) ? " (= injected)" : "", core);
			else begin
				$display("FAIL  %0s: not a clean measurement at %0d of %0d eye-interior steps", name, bad_core, core);
				fails = fails + 1;
			end
			if (bad_closed == 0)
				$display("PASS  %0s: no clean measurement at any rx_ref no-lock step", name);
			else begin
				$display("FAIL  %0s: clean measurement at %0d rx_ref no-lock steps", name, bad_closed);
				fails = fails + 1;
			end
			if (under == 0)
				$display("PASS  %0s: no step counted fewer errors than injected", name);
			else begin
				$display("FAIL  %0s: %0d steps counted fewer errors than injected", name, under);
				fails = fails + 1;
			end
			$display("  info  %0s: edge steps %0d; clean/not clean as rx_ref at %0d", name, edg, edg_agree);

			// the centre it should have chosen from its own measurements
			for (i = 0; i < STEPS; i = i + 1) ok_v[i] = m_seen[i] == 1 && !m_nl[i] && m_err[i] == 0;
			longest_run; m_len = run_len; m_cen = (run_start + run_len / 2) % STEPS;
		end
	endtask

	// Part 5: one event from a locked link. sw2 = 1: an SW2 press (the
	// emulator reset must move); 0: a link event (it must not).
	integer p5_bitsel_prev, p5_cyc, p5_s;
	real    t_go;
	// name: at most 16 characters (judge_sweep's width).
	task p5_event(input [7:0] ev, input [8*16-1:0] name, input integer sw2);
		begin
			m_clear; m_restarts = 0;
			p5_ps_in_rst = 0; p5_ready_falls = 0; p5_ready_rises = 0; p5_cdr_resets = 0;
			p5_rel_running = 0; p5_emu_resets = 0;
			k_ready_falls = 0; k_ready_rises = 0; k_blk_falls = 0; k_rel_running = 0; k_burst = 0; k_stop_seen = 0;
			k_fb_n = 0; k_fb_lead = 0; k_gap_max = 0; t_tun_edge = $realtime;	// with no reset, k_fb holds mid-stream bytes
			case (ev)
			"a": begin
				c_rstb = 0; pll_lock_m = 0; #100000;
				c_rstb = 1; #20000;
				pll_lock_m = 1;
			end
			"b": begin
				pll_lock_m = 0; #1000;
				pll_lock_m = 1;
			end
			"c": begin
				cable = 0; #1000;
				pll_lock_m = 0; #49000;
				cable = 1; #20000;
				pll_lock_m = 1;
			end
			"d": begin
				c_rstb = 0; #100000;
				c_rstb = 1;
			end
			// Part 6, at Kuntur: the companion's LOCK follows its reference
			"e": begin				// rstb (E3, the MCU's PA10) low 100 µs
				k_rstb = 0; #100000;
				k_rstb = 1;
			end
			"f": begin				// pll0 LOCK low 1 µs, clock running
				k_lock = 0; #1000;
				k_lock = 1;
			end
			"g": begin				// MCO stops 100 µs, as in reset_gen_kuntur_tb
				k_mco = 0; #1000;
				k_pll_run = 0; #4000;
				k_lock = 0; #95000;
				k_mco = 1; k_pll_run = 1; #20000;
				k_lock = 1;
			end
			endcase
			t_go = $realtime;
			// Wait (on clkin: wclk stops in reset) for cdr to finish again.
			p5_cyc = 0;
			while (!(p5_cdr_resets > 0 && cdr_rstb_dut && cdr_rdy) && p5_cyc < 4 * STEPS * STEP_TIMEOUT) begin
				@(posedge clkin); p5_cyc = p5_cyc + 1;
			end
			repeat (8) @(posedge rx_wclk);		// let a last PHASESTEP pulse finish
			$display("  part 5 %0s: READY %0.0f clkin after the last release, cdr_rstb %0.1f wclk later; cdr rdy after %0.0f us; %0d pulses",
				name, (t_ready - t_go) / (1.0e3 / 12.0), (t_cdr_rel - t_ready) / (4.0 * T_PS / 1000.0),
				($realtime - t_cdr_rel) / 1000.0, ps_pulses);

			if (p5_ready_falls >= 1 && p5_ready_rises >= 1 && p5_cdr_resets >= 1 && cdr_rstb_dut)
				$display("PASS  %0s: READY fell and rose, cdr reset and released", name);
			else begin
				$display("FAIL  %0s: READY falls %0d rises %0d, cdr resets %0d, cdr_rstb now %b",
					name, p5_ready_falls, p5_ready_rises, p5_cdr_resets, cdr_rstb_dut);
				fails = fails + 1;
			end
			if (p5_ready_rises >= 1 && p5_rel_running == 0)
				$display("PASS  %0s: DDR_RESET released with ECLK stopped before every READY", name);
			else begin
				$display("FAIL  %0s: DDR_RESET released with ECLK running before %0d READY rise(s)", name, p5_rel_running);
				fails = fails + 1;
			end
			if (p5_ps_in_rst == 0)
				$display("PASS  %0s: no PHASESTEP pulse while cdr was in reset", name);
			else begin
				$display("FAIL  %0s: %0d PHASESTEP pulses while cdr was in reset", name, p5_ps_in_rst);
				fails = fails + 1;
			end
			if (p5_emu_resets == sw2)
				$display("PASS  %0s: emulator reset (rst_n_emu) fell %0d time(s)", name, sw2);
			else begin
				$display("FAIL  %0s: emulator reset fell %0d time(s), expected %0d", name, p5_emu_resets, sw2);
				fails = fails + 1;
			end
			if (ev >= "e") begin
				// Kuntur's side of the same event
				if (k_ready_falls >= 1 && k_ready_rises >= 1 && k_blk_falls >= 1 && tx_gen_rstb)
					$display("PASS  %0s: Kuntur READY fell and rose, block_gen reset and released", name);
				else begin
					$display("FAIL  %0s: Kuntur READY falls %0d rises %0d, block_gen resets %0d, its rstb now %b",
						name, k_ready_falls, k_ready_rises, k_blk_falls, tx_gen_rstb);
					fails = fails + 1;
				end
				if (k_ready_rises >= 1 && k_rel_running == 0)
					$display("PASS  %0s: Kuntur DDR_RESET released with its ECLK stopped before every READY", name);
				else begin
					$display("FAIL  %0s: Kuntur DDR_RESET released with its ECLK running before %0d READY rise(s)", name, k_rel_running);
					fails = fails + 1;
				end
				$display("  info  %0s: after the hold-off, TUN_CLK ran %0d cycles over %0.0f ns before STOP (gddr_sync INIT)",
					name, k_burst, (k_burst > 0) ? t_burst1 - t_burst0 : 0.0);
				if (k_gap_max >= K_HOLDOFF_NS)
					$display("PASS  %0s: TUN_CLK stopped for %0.3f ms (hold-off %0.3f ms)", name, k_gap_max / 1.0e6, K_HOLDOFF_NS / 1.0e6);
				else begin
					$display("FAIL  %0s: TUN_CLK stopped for at most %0.3f ms, less than the hold-off %0.3f ms",
						name, k_gap_max / 1.0e6, K_HOLDOFF_NS / 1.0e6);
					fails = fails + 1;
				end
				k = 0;
				for (i = 0; i < 16; i = i + 1) if (i < k_fb_n && k_fb[i] != fb_exp(i)) k = k + 1;
				if (k_fb_n == 16 && k == 0 && k_fb_lead <= 2)
					$display("PASS  %0s: block_gen's first bytes after its reset: two whole blocks, cnt 0 and 1 (after %0d zero byte)", name, k_fb_lead);
				else begin
					$display("FAIL  %0s: block_gen's first %0d bytes after its reset: %0d wrong, %0d leading zero bytes; first: %02h %02h %02h %02h %02h %02h %02h %02h",
						name, k_fb_n, k, k_fb_lead, k_fb[0], k_fb[1], k_fb[2], k_fb[3], k_fb[4], k_fb[5], k_fb[6], k_fb[7]);
					fails = fails + 1;
				end
			end

			judge_sweep(name);
			cl_final = step;
			cl_bitsel = cdr0.aligner0.bitsel;
			p2_cen = (sw_start + cdr_centre) % STEPS;
			if (cdr_rdy && cdr_valid && m_restarts == 0 && cdr_centre == m_cen && cdr_eye_width == m_len &&
			    ref_match(p2_cen, m_len) && cl_final == p2_cen && ps_pulses == STEPS + cdr_centre)
				$display("PASS  %0s: one sweep from step %0d, locked at its centre, step %0d, after %0d + %0d pulses",
					name, sw_start, cl_final, STEPS, cdr_centre);
			else begin
				$display("FAIL  %0s: rdy %0d valid %0d, %0d restarts; centre %0d width %0d (own %0d, %0d), at step %0d (expected %0d), %0d pulses (expected %0d)",
					name, cdr_rdy, cdr_valid, m_restarts, cdr_centre, cdr_eye_width, m_cen, m_len,
					cl_final, p2_cen, ps_pulses, STEPS + cdr_centre);
				fails = fails + 1;
			end

			// A fresh rx_ref at the final step gives the word boundary as it is now.
			@(posedge rx_wclk); #1 rx_rst = 1;
			@(posedge rx_wclk); #1 rx_rst = 0;
			t = 0;
			while (!locked && t < 100 * 8) begin @(posedge rx_wclk); t = t + 1; end
			p5_s = locked ? ref0.s : -1;
			if (cdr_valid && cl_bitsel == p5_s)
				$display("PASS  %0s: bitsel %0d = rx_ref's offset now (before the event: %0d)", name, cl_bitsel, p5_bitsel_prev);
			else begin
				$display("FAIL  %0s: bitsel %0d, rx_ref's offset now %0d", name, cl_bitsel, p5_s);
				fails = fails + 1;
			end
			p5_bitsel_prev = cl_bitsel;
		end
	endtask

	initial begin
		if (!$value$plusargs("n=%d", n_blocks)) n_blocks = 100;
		if (!$value$plusargs("d_data=%f", d_data)) d_data = 1000.0;
		if (!$value$plusargs("d_clk=%f", d_clk)) d_clk = 1300.0;
		if (!$value$plusargs("ins=%f", ins)) ins = 2500.0;
		if (!$value$plusargs("jitter=%f", jit)) jit = 400.0;
		if (!$value$plusargs("win=%f", win)) win = 150.0;
		if (!$value$plusargs("seed=%d", seed)) seed = 1;
		if (!$value$plusargs("inj=%d", inj)) inj = 0;
		if (!$value$plusargs("start=%d", start_step)) start_step = 0;
		ref_base = (inj > 0) ? n_blocks / inj : 0;
		chk_base = (inj > 0) ? CHK_N / inj : 0;
		if (inj > 0 && (n_blocks % inj != 0 || CHK_N % inj != 0)) begin
			$display("cdr_tb: +inj=%0d needs n (%0d) and CHK_N (%0d) to be multiples of it", inj, n_blocks, CHK_N);
			$finish;
		end
		q7first = $test$plusargs("q7first");
		i = $urandom(seed);
		rph = $urandom % 8;			// ECLKDIV starts at an arbitrary phase

		$display("cdr_tb: T=%0.1f ps UI=%0.1f ps step=%0.1f ps, N=%0d blocks/step (rx_ref), %0d (cdr)%s",
			T_PS, UI_PS, T_PS / STEPS, n_blocks, CHK_N, q7first ? ", Q7 first" : "");
		$display("  d_data=%0.0f d_clk=%0.0f ins=%0.0f jitter=±%0.0f win=%0.0f ps",
			d_data, d_clk, ins, jit, win);
		if (inj > 0)
			$display("  injecting 1 bit error every %0d blocks: base %0d (rx_ref), %0d (cdr) subtracted", inj, ref_base, chk_base);

		#500 rstb = 1;
		#2000;
		fails = 0;

		// ==== Part 1: reference sweep (tb sets the phase; cdr held in reset) ====
		for (step = 0; step < STEPS; step = step + 1) begin
			// Let in-flight edges settle, then re-lock from scratch.
			repeat (16) @(posedge rx_wclk);
			fork
				al_scan(step);
				begin
					rx_rst = 1; @(posedge rx_wclk); #1 rx_rst = 0;
					t = 0;
					while (!locked && t < 100 * 8) begin @(posedge rx_wclk); t = t + 1; end
					if (!locked) begin
						nolock_tab[step] = 1; err_tab[step] = 65535;
					end else begin
						nolock_tab[step] = 0;
						@(posedge rx_wclk); #1 count_en = 1;
						while (blocks < n_blocks) @(posedge rx_wclk);
						#1 count_en = 0;
						if (errors < ref_base) begin
							inj_under = inj_under + 1;		// tb fault: injection not seen
							err_tab[step] = 65535;
						end else
							err_tab[step] = (errors - ref_base > 65535) ? 65535 : errors - ref_base;
					end
				end
			join
			s_tab[step] = nolock_tab[step] ? "X" : "0" + ref0.s;
		end
		step = 0; turns = 1;		// 279 -> 0: the sweep ends one turn on

		$display("  rx_ref bathtub ('.' 0 errors, '1'..'5' log10+1, '#', 'X' no lock):");
		for (i = 0; i < STEPS; i = i + 1)
			line[8*(STEPS-1-i) +: 8] = nolock_tab[i] ? "X" : err_char(err_tab[i]);
		show_tab;

		al_bad = 0; al_clean = 0;
		for (i = 0; i < STEPS; i = i + 1)
			if (!nolock_tab[i] && err_tab[i] == 0) begin
				al_clean = al_clean + 1;
				if (al_tab[i] != s_tab[i]) al_bad = al_bad + 1;
				if (pos_bad_tab[i]) begin
					if (pos_first_bad < 0) begin
						pos_first_bad = i; pos_bad_dout = pos_dout_tab[i]; pos_bad_pos = pos_pos_tab[i];
					end
					pos_bad_steps = pos_bad_steps + 1;
				end
			end
		$display("  rx_ref offset s per step ('X' no lock):");
		for (i = 0; i < STEPS; i = i + 1) line[8*(STEPS-1-i) +: 8] = s_tab[i];
		show_tab;
		$display("  frame_aligner bitsel per step ('-' none, 'H' no rdy); longest search %0d wclk:", al_tmax);
		for (i = 0; i < STEPS; i = i + 1) line[8*(STEPS-1-i) +: 8] = al_tab[i];
		show_tab;

		// rx_ref's longest circular error-free run
		for (i = 0; i < STEPS; i = i + 1) ok_v[i] = !nolock_tab[i] && err_tab[i] == 0;
		longest_run; best_len = run_len; best_start = run_start;
		ref_len = run_len; ref_cen = (run_start + run_len / 2) % STEPS;
		ref_runs;
		for (i = 0; i < rr_n; i = i + 1)
			$display("  rx_ref error-free run: steps %0d..%0d, %0d steps, centre %0d",
				rr_start[i], (rr_start[i] + rr_len[i] - 1) % STEPS, rr_len[i], (rr_start[i] + rr_len[i] / 2) % STEPS);

		// ---- Prediction ----
		o0    = d_clk + ins - d_data;				// sampling offset at step 0
		pred0 = (UI_PS + win) / 2.0 - o0;
		while (pred0 < 0) pred0 = pred0 + UI_PS;
		while (pred0 >= UI_PS) pred0 = pred0 - UI_PS;
		pred1 = pred0 + UI_PS;
		pstep0 = pred0 / (T_PS / STEPS);		// real to integer rounds
		pstep1 = pred1 / (T_PS / STEPS);
		$display("  predicted centres: steps %0d and %0d; predicted eye %0.0f ps = %0.1f steps",
			pstep0 % STEPS, pstep1 % STEPS, UI_PS - win - 2.0 * jit,
			(UI_PS - win - 2.0 * jit) / (T_PS / STEPS));

		if (best_len == 0) begin
			$display("FAIL  rx_ref: no error-free step anywhere");
			fails = fails + 1;
		end else begin
			cen = ref_cen;
			d0 = cdist(cen, pstep0 % STEPS);
			d1 = cdist(cen, pstep1 % STEPS);
			dmin = (d0 < d1) ? d0 : d1;
			$display("  rx_ref longest error-free run: steps %0d..%0d, %0d steps = %0.0f ps; centre step %0d (%0.1f deg)",
				best_start, (best_start + best_len - 1) % STEPS, best_len,
				best_len * T_PS / STEPS, cen, cen * 360.0 / STEPS);
			if (dmin <= 3) $display("PASS  rx_ref: centre within %0d steps of a predicted centre", dmin);
			else begin
				$display("FAIL  rx_ref: centre is %0d steps from the nearest predicted centre", dmin);
				fails = fails + 1;
			end
			// The measured run may exceed the worst-case eye (finite N), never fall short of it.
			ok_width_lo = (UI_PS - win - 2.0 * jit) / (T_PS / STEPS) - 2;
			if (best_len >= ok_width_lo && best_len * T_PS / STEPS <= UI_PS - win + 2.0 * T_PS / STEPS)
				$display("PASS  rx_ref: eye width %0d steps is within the predicted bounds", best_len);
			else begin
				$display("FAIL  rx_ref: eye width %0d steps is outside [%0d, %0.0f] steps", best_len,
					ok_width_lo, (UI_PS - win) / (T_PS / STEPS) + 2);
				fails = fails + 1;
			end
		end

		if (al_clean > 0 && al_bad == 0)
			$display("PASS  frame_aligner: valid, bitsel = rx_ref offset at all %0d error-free steps", al_clean);
		else begin
			$display("FAIL  frame_aligner: wrong or no bitsel at %0d of %0d error-free steps", al_bad, al_clean);
			fails = fails + 1;
		end
		if (al_clean > 0 && pos_bad_steps == 0)
			$display("PASS  frame_aligner: dout matches pos on every fixed byte at all %0d error-free steps", al_clean);
		else begin
			$display("FAIL  frame_aligner: dout disagrees with pos at %0d of %0d error-free steps (first step %0d: pos=%0d dout=%02h, expected %02h)",
				pos_bad_steps, al_clean, pos_first_bad, pos_bad_pos, pos_bad_dout, pat_byte(pos_bad_pos));
			fails = fails + 1;
		end
		if (inj > 0) begin
			if (inj_under == 0)
				$display("PASS  injection: rx_ref saw at least the injected errors at every locked step");
			else begin
				$display("FAIL  injection: rx_ref saw fewer than the injected errors at %0d steps (tb fault)", inj_under);
				fails = fails + 1;
			end
		end

		// ==== Part 2: cdr sweeps on its own (PHASESTEP drives the PLL) ====
		cdr_sweep(start_step, -1, 0);
		judge_sweep("sweep");
		p2_final = cl_final;
		$display("  sweep from step %0d: %0d PHASESTEP pulses in %0d wclk; ended rdy %0d valid %0d at step %0d, centre %0d, eye_width %0d, bitsel %0d",
			start_step, cl_pulses, cl_cyc, cl_rdy, cl_valid, cl_final, cdr_centre, cdr_eye_width, cl_bitsel);
		if (ps_viol == 0)
			$display("PASS  PHASESTEP: every pulse low and high for >= %0.1f ns (shortest %0.1f ns)", PS_MIN_NS, ps_wmin);
		else begin
			$display("FAIL  PHASESTEP: %0d pulse widths under %0.1f ns (shortest %0.3f ns): a glitch is a step", ps_viol, PS_MIN_NS, ps_wmin);
			fails = fails + 1;
		end
		if (cdr_phasedir !== 1'b0) begin
			$display("FAIL  PHASEDIR = %b, expected 0 (delay; an advance may glitch)", cdr_phasedir);
			fails = fails + 1;
		end
		if (inj > 0) begin
			// every step holds injected errors, so no step is error-free: cdr must give up
			if (cl_rdy && !cl_valid && cl_pulses == STEPS)
				$display("PASS  sweep: with errors at every step, gave up (rdy, not valid) after one turn");
			else begin
				$display("FAIL  sweep: with errors at every step, expected rdy, not valid after %0d pulses (got rdy %0d valid %0d, %0d pulses)",
					STEPS, cl_rdy, cl_valid, cl_pulses);
				fails = fails + 1;
			end
		end else begin
			p2_cen = (sw_start + cdr_centre) % STEPS;	// absolute
			if (cdr_rdy && m_len > 0 && cdr_centre == m_cen && cdr_eye_width == m_len)
				$display("PASS  sweep: centre %0d and eye_width %0d = its own measurements' longest clean run", cdr_centre, cdr_eye_width);
			else begin
				$display("FAIL  sweep: centre %0d, eye_width %0d; its measurements give centre %0d, width %0d", cdr_centre, cdr_eye_width, m_cen, m_len);
				fails = fails + 1;
			end
			if (m_len > 0 && ref_match(p2_cen, m_len))
				$display("PASS  sweep: centre at step %0d, width %0d steps: matches an rx_ref run (longest %0d)", p2_cen, m_len, ref_len);
			else begin
				$display("FAIL  sweep: centre at step %0d, width %0d steps: no rx_ref run within 1 / 4 steps (longest %0d)", p2_cen, m_len, ref_len);
				fails = fails + 1;
			end
			exp_bit = (nolock_tab[cl_final]) ? -1 : (s_tab[cl_final] - "0" + 8 * STEPS - 2 * cl_turns) % 8;
			if (cl_rdy && cl_valid && cl_final == p2_cen && cl_pulses == STEPS + cdr_centre && cl_bitsel == exp_bit)
				$display("PASS  sweep: locked at the centre after %0d + %0d pulses, bitsel %0d (rx_ref offset - 2 per turn)", STEPS, cdr_centre, cl_bitsel);
			else begin
				$display("FAIL  sweep: rdy %0d valid %0d at step %0d (centre %0d) after %0d pulses (expected %0d), bitsel %0d (expected %0d)",
					cl_rdy, cl_valid, cl_final, p2_cen, cl_pulses, STEPS + cdr_centre, cl_bitsel, exp_bit);
				fails = fails + 1;
			end
		end

		// ==== Part 3: no data: one turn, then give up ====
		data_off = 1;
		cdr_sweep(start_step, -1, 0);
		data_off = 0;
		$display("  no data from step %0d: %0d PHASESTEP pulses in %0d wclk, %0d measurements; rdy %0d valid %0d, at step %0d",
			start_step, cl_pulses, cl_cyc, m_count, cl_rdy, cl_valid, cl_final);
		k = 0;
		for (i = 0; i < STEPS; i = i + 1) if (!(m_seen[i] == 1 && m_nl[i])) k = k + 1;
		if (cl_rdy && !cl_valid && cl_pulses == STEPS && cl_final == start_step && m_count == STEPS && k == 0)
			$display("PASS  no data: %0d no-lock measurements, gave up (rdy, not valid), back at the start phase", STEPS);
		else begin
			$display("FAIL  no data: expected %0d no-lock measurements, %0d pulses, rdy and not valid at step %0d", STEPS, STEPS, start_step);
			fails = fails + 1;
		end

		// ==== Part 4: lock lost mid-sweep ====
		if (inj == 0) begin
			cdr_sweep(start_step, STEPS / 3, 64);
			$display("  lock dropped after %0d measurements for 64 wclk: %0d restart(s); new sweep from step %0d; %0d pulses in all; rdy %0d valid %0d at step %0d, centre %0d",
				STEPS / 3, m_restarts, sw_start, cl_pulses, cl_rdy, cl_valid, cl_final, cdr_centre);
			judge_sweep("after lock loss");
			p2_cen = (sw_start + cdr_centre) % STEPS;
			if (cl_idle_seen && m_restarts == 1 && cl_rdy && cl_valid && cl_final == p2_cen &&
			    cdr_centre == m_cen && cdr_eye_width == m_len && ref_match(cl_final, m_len))
				$display("PASS  lock loss: back to idle, one new sweep from 0, locked at its centre, step %0d (part 2: %0d)", cl_final, p2_final);
			else begin
				$display("FAIL  lock loss: idle seen %0d, restarts %0d, rdy %0d valid %0d, ended at step %0d (part 2: %0d)",
					cl_idle_seen, m_restarts, cl_rdy, cl_valid, cl_final, p2_final);
				fails = fails + 1;
			end
		end

		// ==== Part 5: reset_gen_adapter + gddr_sync in the loop ====
		if (inj == 0) begin
			// From part 4: locked at the centre, the chain up since t = 0.
			p5 = 1; pll_dyn = 1;
			p5_bitsel_prev = cl_bitsel;
			p5_event("a", "5a SW2 late LOCK", 1);
			p5_event("b", "5b LOCK low 1us", 0);
			p5_event("c", "5c cable out", 0);
			p5_event("d", "5d SW2 LOCK held", 1);

			// ==== Part 6: Kuntur's start-up chain in front (transmitter side) ====
			if (k_blk_rstb && k_ready && !k_stop && !k_ddr_rst && !k_divrst)
				$display("PASS  part 6: Kuntur's chain up since t = 0 (block_gen's rstb, READY high; STOP, DDR_RESET low)");
			else begin
				$display("FAIL  part 6: Kuntur's chain not up: blk_rstb %b READY %b STOP %b DDR_RESET %b",
					k_blk_rstb, k_ready, k_stop, k_ddr_rst);
				fails = fails + 1;
			end
			p6 = 1;
`ifdef MUT6
			p6_lock_auto = (`MUT6 != 3);
`else
			p6_lock_auto = 1;
`endif
			p5_event("e", "6e Kuntur rstb", 0);
			p5_event("f", "6f Kuntur LOCK", 0);
			p5_event("g", "6g MCO stop", 0);
			p6_lock_auto = 0;
			pll_dyn = 0;
		end

		if (fails == 0) $display("cdr_tb: ALL PASS");
		else            $display("cdr_tb: %0d FAILED", fails);
		$finish;
	end
endmodule
