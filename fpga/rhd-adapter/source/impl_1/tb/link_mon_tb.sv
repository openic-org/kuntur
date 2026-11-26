/*********************************************************************************
	link_mon_tb: unit testbench for T5.6, the link-down detectors (lvds-tunnel.md
	§9.6 T5.6, "Added 2026-11-25", Q1-Q6; §7.1).

	DUT: the T5.6 module alone, by its spec ports: rstb_c, clk, pll_lock_s, tclk,
	rstb_w, wclk, smp_commit, locked -> link_down, down_events[31:0],
	down_cause[3:0] = {smp_lost, !locked_s, !pll_lock_s, hb_lost}. Module name
	`link_mon` (change LINK_MON below if it is called otherwise). down_events
	must be an `output reg` (the saturation test presets it).

	Rig
	  clk 12 MHz (83.333 ns); tclk 37.95 MHz (26.350 ns); wclk 105.402 ns
	  (tclk/4 in rate; its phase does not matter, the DUT treats them as
	  asynchronous). A stop finishes the high half and holds the clock low.
	  Stops: tclk and wclk together (cable-out, Kuntur reset), tclk alone
	  (receive PLL free-running, Q1).
	  smp_commit: one wclk pulse every 379/380 wclk (40 us), gaps set per trial.
	  locked: a wclk-side level; pll_lock_s: changes 1 ns after a clk edge (it
	  is a clkin flop in reset_gen_adapter).
	  Resets as the system does them: SW2 takes rstb_c and rstb_w low together;
	  rstb_c is released first (at a tclk fall + 1 ns, so the heartbeat
	  counter's phase is known), rstb_w 5 us later.

	Reference model (independent of the RTL)
	  Heartbeat toggles: every 32nd tclk rising edge after rstb_c is released
	    (the spec's 6-bit counter, bit 5, reset 0 by rstb_c).
	  Commits: the wclk edges at which smp_commit is high and rstb_w is.
	  Each term is "bad", "good" or "not judged" at every falling clk edge:
	    heartbeat  bad: > 54.5 ticks since the last toggle (48 + 2-3 to cross +
	               the output flop + 1 of tolerance); good: < 46 ticks
	               since it, and the toggle before was < 46 ticks earlier or
	               the last one is > 5 ticks old (time to cross)
	    1 ms       the same with 12005.5 / 11997 ticks; no commit since reset
	               is bad (Q2: the timer starts saturated)
	    locked     bad / good when 0 / 1 for > 4.5 ticks (2 flops)
	    pll_lock_s bad / good when 0 / 1 for > 2.5 ticks (used directly)
	  No term is judged good in the first 5 ticks after rstb_c is released.

	Checks (each failure prints ERROR and counts; the run ends PASS / FAIL)
	  L1 down_cause[i] = 1 where term i is bad, 0 where it is good.
	  L2 link_down = 1 if any term is bad; 0 if all are good.
	  L3 link_down = |down_cause at every falling clk edge.
	  L4 link_down, down_cause and down_events change only at a rising clk
	     edge (flops; catches a combinational output).
	  L5 down_events = the rising edges of link_down seen since rstb_c
	     (saturating at FFFFFFFF; it may lag the edge by 2 clk), so start-up
	     is not an event (Q5).
	  L6 reset values while rstb_c is low: link_down 1, down_cause 1111,
	     down_events 0.
	  Threshold trials, each judged by the gap actually measured:
	  T1 heartbeat gap < 47 ticks: no link-down event; > 50 ticks: exactly
	     one. tclk alone and both clocks.
	  T2 commit gap 11998.75 ticks (9486 wclk plus a half-period stall): no
	     event; 12002-12003.5: exactly one. 2 x ntrial no-event trials, so a
	     timeout 2 ticks short (down at 11999 edges) is seen.
	  T3 locked low >= 0.6 us, pll_lock_s low >= 1 tick: exactly one event.
	  B6 after a cable-out, link_down = 1 by 4.5 us after tclk stopped.
	  The threshold tolerance is +-1 tick ("48 ticks without a toggle" can be
	  built either way round); timeouts 2 ticks off must fail. A missing
	  synchroniser flop is a metastability risk the simulation cannot see.
	  Coverage minimums for every trial kind.

	Run: tb/run_link_mon.sh (Questa). Plusargs: +seed=N, +ntrial=N (trials per
	kind, default 20).
*********************************************************************************/

`timescale 1ns/1ps
`ifndef LINK_MON
`define LINK_MON link_mon
`endif

module link_mon_tb;

	integer errors = 0;
	int n_kind [string];
	task automatic err(input string s);
		string k;
		begin
			errors = errors + 1;
			k = s.substr(0, (s.len() < 24 ? s.len() : 24) - 1);
			if (!n_kind.exists(k)) n_kind[k] = 0;
			n_kind[k]++;
			if (n_kind[k] <= 3) $display("%t ERROR %s", $realtime, s);
			if (n_kind[k] == 3) $display("%t   (further \"%s...\" errors counted, not printed)", $realtime, k);
		end
	endtask
	task automatic err_summary;
		string k;
		if (n_kind.first(k)) do
			if (n_kind[k] > 3) $display("  %0d x \"%s...\"", n_kind[k], k);
		while (n_kind.next(k));
	endtask

	// ---------------------------------------------------------------- DUT
	localparam real T = 1000.0 / 12.0, TT = 26.350, TW = 105.402;
	localparam realtime NEVER = -1.0e15;

	reg	rstb_c = 1'b0, clk = 1'b0, pll_lock_s = 1'b0, tclk = 1'b0;
	reg	rstb_w = 1'b0, wclk = 1'b0, smp_commit = 1'b0, locked = 1'b0;
	wire		link_down;
	wire [31:0]	down_events;
	wire [3:0]	down_cause;

	`LINK_MON dut (
		.rstb_c(rstb_c), .clk(clk), .pll_lock_s(pll_lock_s), .tclk(tclk),
		.rstb_w(rstb_w), .wclk(wclk), .smp_commit(smp_commit), .locked(locked),
		.link_down(link_down), .down_events(down_events), .down_cause(down_cause)
	);

	// ---------------------------------------------------------------- clocks
	bit tclk_en = 1, wclk_en = 1;
	realtime t_clk = -1.0;
	initial forever #(T / 2.0) clk = ~clk;
	initial forever begin #(TT / 2.0); if (tclk_en || tclk) tclk = ~tclk; end
	initial begin #7.3; forever begin #(TW / 2.0); if (wclk_en || wclk) wclk = ~wclk; end end
	always @(posedge clk) t_clk = $realtime;

	// ---------------------------------------------------------------- reference
	int		hb_n = 0;
	realtime	hb_tl = NEVER, hb_tp = NEVER;		// last and previous heartbeat toggle
	realtime	sm_tl = NEVER, sm_tp = NEVER;		// last and previous commit
	realtime	lk_t = 0.0, pl_t = 0.0;			// last change of locked, pll_lock_s
	event		hb_tog, sm_tog;
	bit [31:0]	exp_ev = 0;
	realtime	t_rise = NEVER, t_rel = NEVER;
	bit		presetting = 0;
	always @(posedge rstb_c) t_rel = $realtime;
	int		rises = 0;				// rising edges of link_down since reset

	always @(posedge tclk) if (rstb_c) begin
		hb_n++;
		if (hb_n % 32 == 0) begin hb_tp = hb_tl; hb_tl = $realtime; -> hb_tog; end
	end
	always @(posedge wclk) if (rstb_w && rstb_c && smp_commit) begin
		sm_tp = sm_tl; sm_tl = $realtime; -> sm_tog;
	end
	always @(locked) lk_t = $realtime;
	always @(pll_lock_s) pl_t = $realtime;

	always @(negedge rstb_c) begin
		hb_n = 0; hb_tl = NEVER; hb_tp = NEVER; sm_tl = NEVER; sm_tp = NEVER;
		exp_ev = 0; rises = 0; t_rise = NEVER;
	end

	always @(posedge link_down) if (rstb_c === 1'b1) begin
		rises++; t_rise = $realtime;
		if (exp_ev != 32'hFFFFFFFF) exp_ev++;
	end

	// -1 bad, +1 good, 0 not judged
	function automatic int gap_state(realtime t, realtime tl, realtime tp, real bad, real good);
		if (t - tl > bad * T) return -1;
		if (t - tl < good * T && (t - tl > 5.0 * T || tl - tp < good * T)) return 1;
		return 0;
	endfunction
	function automatic int lvl_state(realtime t, bit v, realtime tc, real n);
		if (t - tc <= n * T) return 0;
		return v ? 1 : -1;
	endfunction

	int n_chk = 0;
	always @(negedge clk) begin
		if (rstb_c === 1'b0) begin
			if (link_down !== 1'b1 || down_cause !== 4'hF || down_events !== 32'd0)
				err($sformatf("L6 reset values: link_down %b, down_cause %b, down_events %h", link_down, down_cause, down_events));
		end
		else begin
			int s [4]; bit any_bad, all_good; realtime t;
			t = $realtime;
			s[0] = gap_state(t, hb_tl, hb_tp, 54.5, 46.0);
			s[1] = lvl_state(t, pll_lock_s, pl_t, 2.5);
			s[2] = lvl_state(t, locked, lk_t, 4.5);
			s[3] = gap_state(t, sm_tl, sm_tp, 12005.5, 11997.0);
			if (t - t_rel < 5.0 * T) for (int i = 0; i < 4; i++) if (s[i] == 1) s[i] = 0;
			any_bad = 0; all_good = 1;
			for (int i = 0; i < 4; i++) begin
				if (s[i] == -1) begin
					any_bad = 1;
					if (down_cause[i] !== 1'b1) err($sformatf("L1 down_cause[%0d] = %b, term bad", i, down_cause[i]));
				end
				if (s[i] != 1) all_good = 0;
				else if (down_cause[i] !== 1'b0) err($sformatf("L1 down_cause[%0d] = %b, term good", i, down_cause[i]));
			end
			if (any_bad && link_down !== 1'b1) err("L2 link_down = 0 with a term bad");
			if (all_good && link_down !== 1'b0) err("L2 link_down = 1 with all terms good");
			if (link_down !== |down_cause) err($sformatf("L3 link_down %b, down_cause %b", link_down, down_cause));
			if (t - t_rise > 2.5 * T) begin
				if (down_events !== exp_ev) err($sformatf("L5 down_events %h, expected %h", down_events, exp_ev));
			end
			else if (down_events !== exp_ev && down_events !== exp_ev - 1)
				err($sformatf("L5 down_events %h, expected %h (just after a rise)", down_events, exp_ev));
			n_chk++;
		end
	end

	always @(link_down or down_cause or down_events) if (rstb_c === 1'b1 && !presetting && $realtime != t_clk)
		err("L4 an output changed off a rising clk edge");

	// ---------------------------------------------------------------- stimulus
	bit	commits_on = 0;
	int	gap_ovr = 0;			// the next commit gap in wclk cycles, 0 = normal
	initial forever begin
		if (!commits_on || !rstb_w) @(posedge wclk);
		else begin
			int m;
			m = gap_ovr ? gap_ovr : 379 + $urandom_range(1); gap_ovr = 0;
			repeat (m - 1) @(posedge wclk);
			#1 smp_commit = 1'b1;
			@(posedge wclk) #1 smp_commit = 1'b0;
		end
	end

	task automatic set_pll(input bit v);
		@(posedge clk) #1 pll_lock_s = v;
	endtask
	task automatic set_locked(input bit v);
		if (wclk_en) @(posedge wclk);
		#1 locked = v;
	endtask

	// SW2 / start-up: both resets low, rstb_c released first, then rstb_w,
	// then the PLL locks, T5.1 locks, commits start
	task automatic sw2(input real hold_us);
		commits_on = 0; locked = 1'b0;
		rstb_c = 1'b0; rstb_w = 1'b0;
		#(hold_us * 1000.0);
		if (tclk_en) @(negedge tclk);
		#1 rstb_c = 1'b1;
		#5000 rstb_w = 1'b1;
		#(20000 + $urandom_range(20000)) set_pll(1);
		commits_on = 1;
		#(100000 + $urandom_range(50000)) set_locked(1);
		#100000;
	endtask

	task automatic settle;			// all terms good again, then wait
		wait (link_down == 1'b0);
		#20000;
	endtask

	int n_t1_no = 0, n_t1_yes = 0, n_t2_no = 0, n_t2_yes = 0, n_lk = 0, n_pl = 0, n_b6 = 0, n_skip = 0;

	// T1: stretch one heartbeat gap to `target` ticks by stopping tclk (and wclk)
	task automatic hb_trial(input real target, input bit both);
		int r0; real g, p;
		@(hb_tog); #(50 + $urandom_range(400));
		r0 = rises;
		p = target * T - 32.0 * TT;
		tclk_en = 0; if (both) wclk_en = 0;
		#(p);
		tclk_en = 1; wclk_en = 1;
		@(hb_tog);
		g = (hb_tl - hb_tp) / T;
		#20000;
		if (g < 47.0) begin
			n_t1_no++;
			if (rises != r0) err($sformatf("T1 heartbeat gap %.2f ticks gave %0d link-down events, want 0", g, rises - r0));
		end
		else if (g > 50.0) begin
			n_t1_yes++;
			if (rises != r0 + 1) err($sformatf("T1 heartbeat gap %.2f ticks gave %0d link-down events, want 1", g, rises - r0));
		end
		else n_skip++;
		settle;
	endtask

	// T2: one commit gap of m wclk cycles
	task automatic smp_trial(input real lo, input real hi, input bit yes);
		int r0, m; real g;
		m = $ceil(lo * T / TW);
		if (m * TW > hi * T) begin err("tb: no wclk count in the window"); return; end
		@(sm_tog); gap_ovr = m; r0 = rises;
		if (!yes) begin			// + half a wclk period: one rising edge comes late
			repeat (100) @(posedge wclk);
			@(negedge wclk) wclk_en = 0;
			#(TW * 0.75) wclk_en = 1;
		end
		@(sm_tog);
		g = (sm_tl - sm_tp) / T;
		#20000;
		if (g < 11999.0 && !yes) begin
			n_t2_no++;
			if (rises != r0) err($sformatf("T2 commit gap %.2f ticks gave %0d link-down events, want 0", g, rises - r0));
		end
		else if (g > 12002.0 - 0.01 && yes) begin
			n_t2_yes++;
			if (rises != r0 + 1) err($sformatf("T2 commit gap %.2f ticks gave %0d link-down events, want 1", g, rises - r0));
		end
		else n_skip++;
		settle;
	endtask

	// B6: cable-out of `us`; link_down by 4.5 us after tclk stops
	task automatic cable_out(input real us);
		realtime t0; int r0;
		#($urandom_range(30000));
		r0 = rises;
		tclk_en = 0; wclk_en = 0; t0 = $realtime;
		if (us * 1000.0 > 4500.0) begin
			#4500;
			if (link_down !== 1'b1) err("B6 link_down not high 4.5 us after tclk stopped");
			n_b6++;
			#(us * 1000.0 - 4500.0);
		end
		else #(us * 1000.0);
		tclk_en = 1; wclk_en = 1;
		#20000;
		if (us > 5.0 && rises == r0) err($sformatf("B6 cable-out of %.1f us gave no link-down event", us));
		settle;
	endtask

	// mixed: random overlapping events, judged by L1-L6 only
	task automatic mixed(input int n);
		repeat (n) begin
			fork
				begin #($urandom_range(200000)); if ($urandom_range(1)) begin tclk_en = 0; if ($urandom_range(1)) wclk_en = 0; #($urandom_range(3000) * 1.0 + 100.0 * $urandom_range(30)); tclk_en = 1; wclk_en = 1; end end
				begin #($urandom_range(200000)); if ($urandom_range(2) == 0) begin set_locked(0); #($urandom_range(100000)); set_locked(1); end end
				begin #($urandom_range(200000)); if ($urandom_range(2) == 0) begin set_pll(0); #($urandom_range(5000)); set_pll(1); end end
				begin #($urandom_range(200000)); if ($urandom_range(3) == 0) begin @(sm_tog) gap_ovr = 9000 + $urandom_range(20000); end end
			join
			settle;
		end
	endtask

	int seed = 1, ntrial = 20;

	task automatic report(input string s);
		$display("%t %-36s errors %0d, events %0d, T1 %0d/%0d, T2 %0d/%0d, locked %0d, pll %0d, B6 %0d, skipped %0d",
			$realtime, s, errors, rises, n_t1_no, n_t1_yes, n_t2_no, n_t2_yes, n_lk, n_pl, n_b6, n_skip);
	endtask

	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		if ($value$plusargs("ntrial=%d", ntrial)) ;
		void'($urandom(seed));
		$display("link_mon_tb: seed %0d, %0d trials per kind", seed, ntrial);

		// start-up: reset with the clocks running, nothing locked
		#(2000 + $urandom_range(3000));
		sw2(1.0);
		settle;
		if (rises != 0) err($sformatf("L5 %0d link-down events counted at start-up", rises));
		report("start-up");

		repeat (ntrial) hb_trial(46.0 + $urandom_range(1000) / 1000.0, $urandom_range(1));
		repeat (ntrial) hb_trial(50.0 + $urandom_range(1000) / 1000.0, $urandom_range(1));
		report("T1 heartbeat gaps");

		repeat (2 * ntrial) smp_trial(11997.5, 11998.5, 0);	// 9486.5 wclk = 11998.75 ticks
		repeat (ntrial / 2) smp_trial(12002.0, 12003.5, 1);
		report("T2 commit gaps");

		repeat (ntrial) begin
			int r0; real d;
			#($urandom_range(30000));
			r0 = rises; d = 0.6 + $urandom_range(500000) / 1000.0;
			set_locked(0); #(d * 1000.0); set_locked(1);
			#20000;
			if (rises != r0 + 1) err($sformatf("T3 locked low %.1f us gave %0d events, want 1", d, rises - r0));
			n_lk++;
			settle;
		end
		repeat (ntrial) begin
			int r0, k;
			#($urandom_range(30000));
			r0 = rises; k = ($urandom_range(2) == 0) ? 1 : 1 + $urandom_range(6000);
			set_pll(0); repeat (k) @(posedge clk); #1 pll_lock_s = 1'b1;
			#20000;
			if (rises != r0 + 1) err($sformatf("T3 pll_lock_s low %0d ticks gave %0d events, want 1", k, rises - r0));
			n_pl++;
			settle;
		end
		report("T3 locked, pll_lock_s");

		// cable-outs: short ones, then up to 2 ms (the 1 ms term too)
		repeat (ntrial) cable_out(1.0 + $urandom_range(20));
		repeat (ntrial) cable_out(5.0 + $urandom_range(2000000) / 1000.0);
		report("B6 cable-outs");

		// tclk alone (PLL free-running), long
		repeat (ntrial / 2) begin
			int r0;
			#($urandom_range(30000));
			r0 = rises; tclk_en = 0; #(5000 + $urandom_range(300000)); tclk_en = 1;
			#20000;
			if (rises != r0 + 1) err($sformatf("T1 tclk stopped alone gave %0d events, want 1", rises - r0));
			settle;
		end
		report("tclk alone");

		mixed(3 * ntrial); report("mixed");

		// SW2 mid event, then start-up again
		repeat (4) begin
			fork
				begin tclk_en = 0; #($urandom_range(20000)); tclk_en = 1; end
				begin #($urandom_range(10000)); sw2(0.5 + $urandom_range(10)); end
			join
			settle;
			if (rises != 0) err($sformatf("L5 %0d link-down events counted after SW2", rises));
		end
		report("SW2 mid event");

		// saturation: preset near the top, then pll_lock_s drops
		@(posedge clk) #1 presetting = 1; dut.down_events = 32'hFFFF_FFF0; exp_ev = 32'hFFFF_FFF0;
		#1 presetting = 0;
		repeat (24) begin set_pll(0); repeat (3) @(posedge clk); #1 pll_lock_s = 1'b1; settle; end
		if (down_events !== 32'hFFFF_FFFF) err($sformatf("L5 down_events %h, not saturated", down_events));
		report("saturation");

		if (n_t1_no < ntrial / 2 || n_t1_yes < ntrial / 2) err($sformatf("coverage: T1 %0d/%0d", n_t1_no, n_t1_yes));
		if (n_t2_no < ntrial || n_t2_yes < ntrial / 4) err($sformatf("coverage: T2 %0d/%0d", n_t2_no, n_t2_yes));
		if (n_b6 < ntrial / 2) err($sformatf("coverage: B6 %0d", n_b6));

		$display("checks %0d; T1 no/yes %0d/%0d, T2 no/yes %0d/%0d, locked %0d, pll %0d, B6 %0d, skipped %0d",
			n_chk, n_t1_no, n_t1_yes, n_t2_no, n_t2_yes, n_lk, n_pl, n_b6, n_skip);
		err_summary;
		if (errors == 0) $display("link_mon_tb: PASS");
		else $display("link_mon_tb: FAIL, %0d errors", errors);
		$finish;
	end

	initial begin #2000ms; err("timeout"); err_summary; $display("link_mon_tb: FAIL"); $finish; end
endmodule
