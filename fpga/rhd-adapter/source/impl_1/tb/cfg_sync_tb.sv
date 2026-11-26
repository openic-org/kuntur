/*********************************************************************************
	cfg_sync_tb: unit testbench for T5.5 cfg_sync (lvds-tunnel.md §9.6 T5.5,
	"Added 2026-11-25", Q1-Q8; ack = req_d, corrected 2026-11-25).

	DUT: cfg_sync alone. The tb plays T5.2 (wclk side: cfg_strobe, cfg_regs) and
	T5.4 (sck side: data_rx_q, cmd_strobe).
	  wclk: free-running at a phase-dependent period (37.1, 105.4, 263.3 ns),
	    stopped for cable-outs.
	  sck: Intan-like words, 16 rising edges at 41.667 ns, a CS-high gap of 2-6
	    periods, and long stops (RHX not running).
	  T5.4 timing: data_rx_q takes word N just after edge 1 of word N+1 and
	    holds it to the end of N+1; cmd_strobe is high for edge 2 only. In the
	    "garbage" phase data_rx_q is random at every edge except edge 2, so the
	    decode must use cmd_strobe.
	  T5.2 timing: cfg_strobe is a one-wclk pulse; cfg_regs changes only in the
	    strobe cycle. Some strobes repeat the last value (the 40 ms refresh).

	Reference model (independent of the RTL)
	  Strobe history: every value the DUT sees strobed since reset, in order.
	  Host shadow: at a rising sck edge with cmd_strobe, WRITE(R, D) with R <= 21
	    (data_rx_q[15:14] = 10): if !written[R] or shadow[R] != D, count + 1
	    (saturating at FFFFFFFF); shadow[R] = D, written[R] = 1.
	  Mismatch: bit R = cfg_valid_s && written[R] && shadow[R] != cfg_regs_s
	    byte R, for R != 3, 6; bits 3 and 6 are 0.

	Checks (each failure prints ERROR and counts; the run ends PASS / FAIL)
	  C1 cfg_regs_s changes only at a rising sck edge, and only to a value from
	     the strobe history, never older than the last one shown (no tearing,
	     no going back).
	  C2 newest wins, with latency: after the last strobe, once 4 rising sck
	     edges, then 4 wclk edges, then 4 rising sck edges have passed (the
	     bound for a strobe that found a request pending: old capture, ack
	     through 2 flops, reload, new capture, each with 1 edge of slack),
	     cfg_regs_s = the last strobed value and cfg_valid_s = 1, and it stays
	     so until the next strobe. Stopped clocks just make the chain wait, so
	     this also covers "taken in the first word after sck restarts".
	  C3 cfg_valid_s changes only at a rising sck edge, never falls except by
	     rstb_s; cfg_valid_s = 0 implies cfg_regs_s = 0, and cfg_valid_s = 1
	     implies cfg_regs_s != 0 (strobed values are never 0), so the two
	     change in the same edge; cfg_valid_s = 1 only after a strobe since
	     reset (R4, Q7).
	  C4 host_write_changes and host_mismatch_regs equal the model at every
	     falling sck edge and at the end of every phase (saturation included).
	  T  (white-box, dut.req) at most 2 req toggles since the last
	     cfg_strobe: a reload consumes its again (Q2). 2, not 1, because the
	     spec allows a strobe in the reload cycle to leave again set (one
	     redundant transfer of the same value).
	  H  (white-box, on state the spec names: cfg_hold, req, req_d) cfg_hold
	     changes only when req == req_d, i.e. when the last request has been
	     captured. A capture of a changing cfg_hold cannot be seen in a
	     zero-delay simulation, so this is the check that the wclk side waits
	     for the capture (Q1).
	  Coverage minimums: captures, "again" reloads (dut.again rising),
	     completed C2 chains, saturation reached, mismatch bits set and
	     cleared, writes to R >= 22, 3 and 6, checks with cfg_valid_s = 1 and
	     a register the host has not written (Q4).

	Phases: before any CONFIG; normal strobes; bursts; bursts with sck
	stopped; cable-out (wclk stopped) with a request pending; garbage
	data_rx_q; fast and slow wclk; resets mid stream; CONFIG before the
	host's first WRITEs; saturation; quiet end.

	Run: tb/run_cfg_sync.sh (Questa). Plusargs: +seed=N, +scale=N (words per
	phase multiplier, default 1).
*********************************************************************************/

`timescale 1ns/1ps

module cfg_sync_tb;

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
	reg		rstb_w = 1'b0, wclk = 1'b0, cfg_strobe = 1'b0;
	reg [175:0]	cfg_regs = 176'd0;
	reg		rstb_s = 1'b0, sck = 1'b0, cmd_strobe = 1'b0;
	reg [15:0]	data_rx_q = 16'hFF00;
	wire [175:0]	cfg_regs_s;
	wire		cfg_valid_s;
	wire [31:0]	host_write_changes;
	wire [21:0]	host_mismatch_regs;

	cfg_sync dut (
		.rstb_w(rstb_w), .wclk(wclk), .cfg_strobe(cfg_strobe), .cfg_regs(cfg_regs),
		.rstb_s(rstb_s), .sck(sck), .data_rx_q(data_rx_q), .cmd_strobe(cmd_strobe),
		.cfg_regs_s(cfg_regs_s), .cfg_valid_s(cfg_valid_s),
		.host_write_changes(host_write_changes), .host_mismatch_regs(host_mismatch_regs)
	);

	// ---------------------------------------------------------------- clocks
	localparam real SPER = 41.667;
	real	wper = 105.374;
	bit	wclk_en = 1;

	// a stop finishes the high half and then holds wclk low (no glitch); a
	// restart resumes on the next half-period tick
	initial forever begin
		#(wper / 2.0);
		if (wclk_en || wclk) wclk = ~wclk;
	end

	// ---------------------------------------------------------------- reference state
	bit [175:0]	hist [$];		// strobed values since reset
	int		last_idx = 0;		// index in hist of the value cfg_regs_s shows
	bit [175:0]	newest;
	bit		strobed = 0;		// a strobe since reset
	int		st = 0, st_n = 0;	// C2 chain: 0 idle, 1 sck, 2 wclk, 3 sck, 4 done
	realtime	t_sck = -1.0;

	bit [7:0]	m_sh [22];
	bit		m_wr [22];
	bit [31:0]	m_cnt = 0;

	int n_cap = 0, n_again = 0, n_chain = 0, n_sat = 0, n_mm_set = 0, n_mm_clr = 0;
	int n_wr = 0, n_wr_hi = 0, n_wr36 = 0, n_strobe = 0, n_chg = 0, n_unwr = 0;
	int m_tog = 0;				// req toggles since the last strobe, for T

	function automatic bit [21:0] m_mismatch;
		m_mismatch = 22'd0;
		for (int r = 0; r < 22; r++)
			if (r != 3 && r != 6)
				m_mismatch[r] = cfg_valid_s && m_wr[r] && (m_sh[r] != cfg_regs_s[8*r +: 8]);
	endfunction

	task automatic model_reset;
		hist.delete(); last_idx = 0; strobed = 0; st = 0; st_n = 0;
		for (int r = 0; r < 22; r++) begin m_sh[r] = 8'd0; m_wr[r] = 1'b0; end
		m_cnt = 32'd0; m_tog = 0;
	endtask

	always @(negedge rstb_s) model_reset;

	// T5.2 side monitor and C2 wclk stage
	always @(posedge wclk) if (rstb_w && rstb_s) begin
		if (cfg_strobe) begin
			hist.push_back(cfg_regs); newest = cfg_regs; strobed = 1;
			st = 1; st_n = 0; n_strobe++; m_tog = 0;
		end
		else if (st == 2) begin
			st_n++;
			if (st_n >= 4) begin st = 3; st_n = 0; end
		end
	end

	// C2 sck stages, host model, edge time
	always @(posedge sck) begin
		t_sck = $realtime;
		if (rstb_s) begin
			if (st == 1 || st == 3) begin
				st_n++;
				if (st_n >= 4) begin
					st = st + 1; st_n = 0;
					if (st == 4) n_chain++;
				end
			end
			if (cmd_strobe && data_rx_q[15:14] == 2'b10) begin
				int r; r = data_rx_q[13:8];
				n_wr++;
				if (r == 3 || r == 6) n_wr36++;
				if (r > 21) n_wr_hi++;
				else begin
					if (!m_wr[r] || m_sh[r] != data_rx_q[7:0]) begin
						n_chg++;
						if (m_cnt != 32'hFFFFFFFF) m_cnt++;
						else n_sat++;
					end
					m_sh[r] = data_rx_q[7:0]; m_wr[r] = 1'b1;
				end
			end
		end
	end

	// C1, C3: changes of the outputs
	always @(cfg_regs_s) if (rstb_s === 1'b1) begin
		int k;
		if ($realtime != t_sck) err("C1 cfg_regs_s changed off a rising sck edge");
		k = -1;
		for (int i = last_idx; i < hist.size(); i++) if (hist[i] == cfg_regs_s) begin k = i; break; end
		if (k < 0) err($sformatf("C1 cfg_regs_s = %h not strobed (or older than the last shown)", cfg_regs_s));
		else last_idx = k;
		n_cap++;
	end

	always @(cfg_valid_s) if (rstb_s === 1'b1) begin
		if ($realtime != t_sck) err("C3 cfg_valid_s changed off a rising sck edge");
		if (!cfg_valid_s) err("C3 cfg_valid_s fell without rstb_s");
	end

	// H: cfg_hold changes only when the last request has been captured
	always @(posedge wclk) if (rstb_w && rstb_s) begin
		bit [175:0] h; bit q, qd;
		h = dut.cfg_hold; q = dut.req; qd = dut.req_d;
		#0.001;
		if (dut.cfg_hold !== h && q !== qd) err("H cfg_hold changed while a request was not yet captured");
	end

	always @(posedge dut.again) if (rstb_w) n_again++;

	// T (the monitor above sees a strobe in the active region of its wclk
	// edge, before the NBA that toggles req)
	always @(dut.req) if (rstb_w === 1'b1 && rstb_s === 1'b1) begin
		m_tog++;
		if (m_tog > 2) err($sformatf("T req toggled %0d times since the last strobe", m_tog));
	end

	// checks at every falling sck edge
	bit [21:0] mm_prev = 0;
	task automatic check_now(input string where);
		bit [21:0] mm;
		if (rstb_s !== 1'b1) return;
		if (!cfg_valid_s && cfg_regs_s !== 176'd0) err({"C3 cfg_valid_s = 0 but cfg_regs_s != 0 ", where});
		if (cfg_valid_s && cfg_regs_s === 176'd0) err({"C3 cfg_valid_s = 1 but cfg_regs_s = 0 ", where});
		if (cfg_valid_s) for (int r = 0; r < 22; r++) if (!m_wr[r] && r != 3 && r != 6) begin n_unwr++; break; end
		if (cfg_valid_s && !strobed) err({"C3 cfg_valid_s = 1 before a CONFIG ", where});
		if (cfg_valid_s !== 1'b0 && cfg_valid_s !== 1'b1) err({"C3 cfg_valid_s not 0/1 ", where});
		if (st == 4 && (cfg_regs_s !== newest || cfg_valid_s !== 1'b1))
			err($sformatf("C2 newest not shown after the latency bound %s: %h, want %h", where, cfg_regs_s, newest));
		if (host_write_changes !== m_cnt)
			err($sformatf("C4 host_write_changes %h, model %h %s", host_write_changes, m_cnt, where));
		mm = m_mismatch();
		if (host_mismatch_regs !== mm)
			err($sformatf("C4 host_mismatch_regs %b, model %b %s", host_mismatch_regs, mm, where));
		n_mm_set += $countones(mm & ~mm_prev); n_mm_clr += $countones(~mm & mm_prev);
		mm_prev = mm;
	endtask

	always @(negedge sck) check_now("(sck fall)");

	// ---------------------------------------------------------------- stimulus: T5.2
	int	s_mode = 0;		// 0 none, 1 normal (gap 5-60 wclk), 2 burst (gap 0-3)
	int	p_rep = 25;		// % of strobes repeating the last value
	bit	s_idle = 1;

	function automatic bit [175:0] rnd176;
		rnd176 = {$urandom, $urandom, $urandom, $urandom, $urandom, $urandom};
		rnd176[0] = 1'b1;	// never 0, so C3 can tell
	endfunction

	initial forever begin
		if (s_mode == 0 || !rstb_w) begin s_idle = 1; #50; end
		else begin
			int gap;
			s_idle = 0;
			gap = (s_mode == 1) ? 5 + $urandom_range(55) : $urandom_range(3);
			repeat (gap) @(negedge wclk);
			if (s_mode != 0 && rstb_w) begin
				@(negedge wclk);
				cfg_strobe = 1'b1;
				if ($urandom_range(99) >= p_rep || !strobed) cfg_regs = rnd176();
				@(negedge wclk);
				cfg_strobe = 1'b0;
			end
		end
	end

	// ---------------------------------------------------------------- stimulus: sck words (T5.4)
	bit	garbage = 0;
	bit [15:0] prev_w = 16'hFF00;
	int	p_match = 30;		// % of WRITEs with D = the byte shown in cfg_regs_s
	int	p_wr = 60;		// % of words that are WRITEs

	function automatic bit [15:0] rnd_word;
		int k, r; bit [7:0] d;
		k = $urandom_range(99);
		if (k < p_wr) begin
			int q; q = $urandom_range(99);
			r = (q < 70) ? $urandom_range(21) : (q < 85) ? ($urandom_range(1) ? 3 : 6) : 22 + $urandom_range(41);
			case ($urandom_range(3))
				0: d = r * 7;
				1: d = r * 7 + 1;
				2: d = (r < 22 && $urandom_range(99) < p_match * 2) ? cfg_regs_s[8*(r % 22) +: 8] : 8'h55;
				default: d = $urandom;
			endcase
			rnd_word = {2'b10, r[5:0], d};
		end
		else if (k < 75 || p_wr >= 75) rnd_word = {2'b11, 6'($urandom_range(63)), 8'd0};		// READ
		else if (k < 93) rnd_word = {2'b00, 6'($urandom_range(63)), 8'd0};		// CONVERT
		else if (k < 96) rnd_word = 16'h5500;						// CALIBRATE
		else rnd_word = 16'h6A00;							// CLEAR
	endfunction

	task automatic word(input bit [15:0] w);
		for (int e = 1; e <= 16; e++) begin
			#(SPER / 2.0) sck = 1'b1;
			#0.1;
			if (e == 1) begin data_rx_q = prev_w; cmd_strobe = 1'b1; end
			else begin
				cmd_strobe = 1'b0;
				if (garbage) data_rx_q = $urandom;
			end
			#(SPER / 2.0 - 0.1) sck = 1'b0;
		end
		prev_w = w;
		#(SPER * (2 + $urandom_range(4)));
		if (garbage) data_rx_q = $urandom;
	endtask

	task automatic words(input int n);
		repeat (n) word(rnd_word());
	endtask

	// a word stream with a random long stop every so often
	task automatic words_stops(input int n, input int p_stop, input real stop_us_max);
		repeat (n) begin
			word(rnd_word());
			if ($urandom_range(999) < p_stop) #(1000.0 * (1.0 + $urandom_range(int'(stop_us_max))));
		end
	endtask

	// ---------------------------------------------------------------- phases
	int scale = 1, seed = 1;

	task automatic report(input string s);
		check_now({"(end of ", s, ")"});
		$display("%t %-44s errors %0d, strobes %0d, captures %0d, chains %0d, again %0d, host changes %0d",
			$realtime, s, errors, n_strobe, n_cap, n_chain, n_again, n_chg);
	endtask

	task automatic quiet;		// strobes off, let the last one through
		s_mode = 0; wait (s_idle); words(30);
		if (st != 4 && strobed) err($sformatf("C2 chain not done after a quiet period (stage %0d)", st));
	endtask

	task automatic do_reset;
		s_mode = 0; wait (s_idle);
		#($urandom_range(200));		// asynchronous to both clocks
		rstb_w = 1'b0; rstb_s = 1'b0;
		#(100 + $urandom_range(300));
		rstb_w = 1'b1; rstb_s = 1'b1;
	endtask

	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		if ($value$plusargs("scale=%d", scale)) ;
		void'($urandom(seed));
		$display("cfg_sync_tb: seed %0d, scale %0d", seed, scale);
		model_reset;
		#333; rstb_w = 1'b1; rstb_s = 1'b1;

		// before any CONFIG: WRITEs come, nothing may show
		words(300 * scale); report("before CONFIG");
		if (cfg_valid_s !== 1'b0) err("C3 cfg_valid_s = 1 before a CONFIG");

		s_mode = 1; words(2000 * scale); report("normal strobes");
		s_mode = 2; words(1000 * scale); quiet; report("bursts");

		// bursts while sck is stopped, then sck restarts
		repeat (20 * scale) begin
			s_mode = 2; #(5000 + $urandom_range(20000));
			s_mode = 0; wait (s_idle);
			words(3);
			if (st != 4) err($sformatf("C2 not taken in the first words after sck restarted (stage %0d)", st));
		end
		report("bursts with sck stopped");

		// cable-out: a request pending while wclk is stopped
		repeat (20 * scale) begin
			s_mode = 2; words(5 + $urandom_range(10));
			wclk_en = 0; s_mode = 0;
			words(20 + $urandom_range(40));
			wclk_en = 1; wait (s_idle);
			words(20);
		end
		quiet; report("cable-out (wclk stopped)");

		s_mode = 1; words_stops(2000 * scale, 20, 100.0); quiet; report("normal, sck stops");

		garbage = 1; s_mode = 1; words(2000 * scale); quiet; report("garbage data_rx_q");
		garbage = 0;

		wper = 37.123; s_mode = 2; words(1000 * scale); s_mode = 1; words(1000 * scale); quiet; report("fast wclk (37.1 ns)");
		wper = 263.31; s_mode = 2; words(1000 * scale); s_mode = 1; words(1000 * scale); quiet; report("slow wclk (263.3 ns)");
		wper = 105.374;

		// resets mid stream
		repeat (3) begin
			s_mode = 1;
			fork
				words(300 * scale);
				begin #(1000 + $urandom_range(5000)); do_reset; end
			join
			if (cfg_valid_s) err("C3 cfg_valid_s = 1 after reset without a CONFIG");
			words(200 * scale); report("before CONFIG, after reset");
			s_mode = 1; words(500 * scale); quiet; report("reset mid stream, then strobes");
		end

		// CONFIG before the host's first WRITEs (Q4): READ / CONVERT only, then
		// WRITEs come in slowly
		do_reset;
		p_wr = 0; s_mode = 1; words(500 * scale);
		p_wr = 2; words(1500 * scale); p_wr = 60; quiet; report("CONFIG before the host writes");

		// saturation: sck stopped, preset the counter near the top
		#200;
		dut.host_write_changes = 32'hFFFF_FFF0; m_cnt = 32'hFFFF_FFF0;
		s_mode = 1; words(2000 * scale); quiet; report("saturation");
		if (host_write_changes !== 32'hFFFF_FFFF) err("C4 host_write_changes not saturated");

		s_mode = 2; words(500 * scale); quiet; report("quiet end");

		// coverage minimums
		if (n_cap < 200) err($sformatf("coverage: %0d captures", n_cap));
		if (n_again < 50) err($sformatf("coverage: %0d again reloads", n_again));
		if (n_chain < 50) err($sformatf("coverage: %0d C2 chains", n_chain));
		if (n_sat < 10) err($sformatf("coverage: %0d saturated changes", n_sat));
		if (n_mm_set < 50 || n_mm_clr < 50) err($sformatf("coverage: mismatch set %0d, cleared %0d", n_mm_set, n_mm_clr));
		if (n_unwr < 500) err($sformatf("coverage: %0d checks with an unwritten register", n_unwr));
		if (n_wr_hi < 50 || n_wr36 < 50) err($sformatf("coverage: writes R>=22 %0d, R=3/6 %0d", n_wr_hi, n_wr36));

		$display("strobes %0d, captures %0d, again %0d, C2 chains %0d; WRITEs %0d (R>=22 %0d, R=3/6 %0d), changes %0d, saturated %0d; mismatch bits set %0d, cleared %0d",
			n_strobe, n_cap, n_again, n_chain, n_wr, n_wr_hi, n_wr36, n_chg, n_sat, n_mm_set, n_mm_clr);
		err_summary;
		if (errors == 0) $display("cfg_sync_tb: PASS");
		else $display("cfg_sync_tb: FAIL, %0d errors", errors);
		$finish;
	end

	initial begin #2000ms; err("timeout"); err_summary; $display("cfg_sync_tb: FAIL"); $finish; end
endmodule
