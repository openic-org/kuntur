/*********************************************************************************
	frame_parse_tb: unit testbench for T5.2 frame_parse (lvds-tunnel.md §9.6
	T5.2, "Added 2026-11-23"; frame format §4.1, §4.3, §4.4, §4.4a, §4.6;
	handshake with T5.1 in §9.6 T5.1, "Added 2026-11-21").

	Two parts, two frame_parse instances, run one after the other.

	Part A, frame_parse alone (instance pa). The tb plays T5.1: it drives din
	and hdr_start the way frame_rx does (registered on wclk; hdr_start in the
	cycle din holds header byte 0, after the 4 preamble bytes), and puts
	random bytes, fake preambles included, between frames without hdr_start.
	Every byte driven carries a tag (frame, byte number), so each output can be
	traced to the byte that caused it.

	Part B, frame_rx + frame_parse together (instances rx, pb), fed with
	Kuntur's bit stream (§4.7: SAMPLE + aux frame back to back, 379/380 bytes
	per period, bit 0 first, stream offset +off). It checks the real T5.1/T5.2
	handshake: lock is reached and held, and every frame sent arrives.

	Checks (each failure prints ERROR and counts; the run ends PASS / FAIL)
	  P1 timing: hdr_bad at most 3 wclk after header byte 7 was on din;
	     frame_end at most 3 wclk after the last CRC byte; smp_commit,
	     smp_invalid, cfg_strobe, sts_strobe at most 3 wclk after the byte
	     that decides them (header byte 7, the last CRC byte, or the hdr_start
	     that cuts a frame short). All pulses one wclk wide.
	  P2 frame_end: crc_ok right; with crc_ok, is_sample and frame_index right.
	     No frame_end for a bad header or a frame cut short; no hdr_bad for a
	     good header (R3: (01,256), (02,22), (03,18) only, length LE).
	  P3 SAMPLE: buf_we words = byte pairs 2k, 2k+1 of the payload, low byte
	     first, at buf_waddr k, each within 3 wclk of its second byte, only
	     for a SAMPLE with a good header; at smp_commit all 128 words of that
	     frame written, none after it; smp_index, smp_flags = the header's.
	  P4 good / failed: smp_commit only for a good SAMPLE; smp_invalid for
	     every failed frame whose type byte read 01 (bad CRC, hdr_bad, cut
	     short: Q1, Q2), never for any other.
	  P5 CONFIG / STATUS: cfg_regs (reg[i] in bits 8i+7..8i) and sts_* (§4.4a
	     fields, LE) equal the payload at the strobe; strobe only on a good
	     CRC; the outputs change only in their strobe cycle (a failed frame
	     leaves them as they were).
	  P6 counters: frames_received (good SAMPLEs), frames_crc_failed (bad
	     CRC, hdr_bad, cut short; any type), frame_index_gaps (good SAMPLE
	     whose index is not the previous good one + 1, mod 2^32; none on the
	     first after reset) equal the tb's count, checked whenever 6 wclk have
	     passed since the last event. Saturation (never wrap) with
	     +define+FP_SAT, which deposits values near 2^32 into the DUT's
	     counter registers (needs them to be output regs of those names).
	  P7 only hdr_start starts a frame: the bytes between frames (random, fake
	     preambles) produce nothing.
	  B1 with frame_rx: lock within LOCK_LIM periods; in clean stretches every
	     good SAMPLE committed (index found in the sent set, all 128 words and
	     flags right), every aux frame strobed with its payload, no invalid,
	     counters as sent, lock held.
	  B2 one bad CRC / one bad header on a SAMPLE: smp_invalid + 1,
	     frames_crc_failed + 1, frame_index_gaps + 1, lock held. One SAMPLE
	     with a corrupted preamble: no invalid, no failure, gaps + 1 (the case
	     Q1 leaves to frame_index_gaps). 8 consecutive bad CRCs (SAMPLE and
	     aux): unlock, relock, clean.

	Run: tb/run_frame_parse.sh (Questa). Plusargs: +seed=N, +off=0..7 (part B
	stream offset, default 3), +soak=N (part A random frames, default 400).
*********************************************************************************/

`timescale 1ns/1ps

module frame_parse_tb;
	localparam real T_W = 1000.0 / 9.4875;	// wclk, ns
	localparam real T_C = 1000.0 / 12.0;	// clk, ns
	localparam integer MAXLAT = 3;			// wclk, §9.6 T5.1 handshake
	localparam integer QUIET = 6;			// wclk after an event before counters are compared
	localparam integer CFG_EVERY = 4, STS_EVERY = 3;
	localparam integer LOCK_LIM = 25;		// periods
	localparam [31:0] MAXC = 32'hFFFFFFFF;

	reg wclk = 0, clk = 0;
	always #(T_W / 2) wclk = ~wclk;
	always #(T_C / 2) clk = ~clk;

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

	function automatic [31:0] crc_byte(input [31:0] c, input [7:0] b);
		integer k;
		begin
			c = c ^ b;
			for (k = 0; k < 8; k = k + 1) c = c[0] ? (c >> 1) ^ 32'hEDB88320 : c >> 1;
			crc_byte = c;
		end
	endfunction

	function automatic [31:0] sat_inc(input [31:0] v);
		sat_inc = (v == MAXC) ? v : v + 1;
	endfunction

	function automatic bit pair_ok(input [7:0] t, input [15:0] l);
		pair_ok = (t == 8'h01 && l == 16'd256) || (t == 8'h02 && l == 16'd22) || (t == 8'h03 && l == 16'd18);
	endfunction

	// ======================================================================
	// Part A: frame_parse alone
	// ======================================================================
	reg rstb_a = 0;
	reg [7:0] din_a = 0;
	reg hs_a = 0;
	wire a_hdr_bad, a_frame_end, a_crc_ok, a_is_sample;
	wire [31:0] a_frame_index;
	wire a_buf_we;
	wire [6:0] a_buf_waddr;
	wire [15:0] a_buf_wdata;
	wire a_smp_commit, a_smp_invalid;
	wire [31:0] a_smp_index;
	wire [7:0] a_smp_flags;
	wire a_cfg_strobe;
	wire [175:0] a_cfg_regs;
	wire a_sts_strobe;
	wire [15:0] a_sts_design_id, a_sts_alt_cmd;
	wire [7:0] a_sts_valid;
	wire [63:0] a_sts_alt_answer;
	wire [31:0] a_sts_fifo0_overflow;
	wire [31:0] a_frames_received, a_frames_crc_failed, a_frame_index_gaps;

	frame_parse pa (
		.rstb(rstb_a), .wclk(wclk), .din(din_a), .hdr_start(hs_a),
		.hdr_bad(a_hdr_bad), .frame_end(a_frame_end), .crc_ok(a_crc_ok),
		.is_sample(a_is_sample), .frame_index(a_frame_index),
		.buf_we(a_buf_we), .buf_waddr(a_buf_waddr), .buf_wdata(a_buf_wdata),
		.smp_commit(a_smp_commit), .smp_invalid(a_smp_invalid),
		.smp_index(a_smp_index), .smp_flags(a_smp_flags),
		.cfg_strobe(a_cfg_strobe), .cfg_regs(a_cfg_regs),
		.sts_strobe(a_sts_strobe), .sts_design_id(a_sts_design_id),
		.sts_valid(a_sts_valid), .sts_alt_cmd(a_sts_alt_cmd),
		.sts_alt_answer(a_sts_alt_answer), .sts_fifo0_overflow(a_sts_fifo0_overflow),
		.frames_received(a_frames_received), .frames_crc_failed(a_frames_crc_failed),
		.frame_index_gaps(a_frame_index_gaps)
	);

	// ---------------- frames and the byte queue ----------------
	// frm: the frames as the parser sees them, built by the driver from the
	// bytes actually on din from each hdr_start (a frame cut short takes the
	// next frame's preamble bytes as its own, exactly as behind frame_rx).
	typedef struct {
		bit [7:0]  typ, flags;
		bit [31:0] idx;
		bit [15:0] len;			// payload_len as seen
		bit [7:0]  by[$];		// bytes seen, from header byte 0
		bit        hdr_ok;		// (type, len) accepted
		bit        crc_good;	// CRC over the bytes seen matches
		bit        done;		// decided (end, hdr_bad or cut)
		bit        committed;
		string     what;
	} frm_t;
	frm_t frm[$];
	string labels[$];			// what the tb meant to send, by push

	typedef struct { bit [7:0] b; bit hs; int lab; } ent_t;
	ent_t q[$];

	// expected pulses
	typedef enum { E_HB, E_FE, E_COMMIT, E_INV, E_CFG, E_STS } ek_t;
	typedef struct { int lo; int hi; int fid; } exp_t;
	exp_t ex[6][$];
	string ename[6] = '{"hdr_bad", "frame_end", "smp_commit", "smp_invalid", "cfg_strobe", "sts_strobe"};
	integer maxlat[6] = '{0, 0, 0, 0, 0, 0};

	// tb model of the counters
	reg [31:0] m_recv = 0, m_fail = 0, m_gaps = 0, m_prev = 0;
	bit m_have_prev = 0;
	integer quiet_after = 0;
	integer cyc_a = 0;
	integer cur = -1;					// frame of the last hdr_start
	int tag_fid [0:7], tag_bno [0:7];	// tags of the last 8 cycles, by cyc_a % 8
	integer written_by [0:127];			// frame whose word k the DUT wrote last
	reg [15:0] wword [0:127];

	task automatic expect_ev(input int k, input int c, input int f);
		exp_t e;
		e.lo = c; e.hi = c + MAXLAT; e.fid = f;
		ex[k].push_back(e);
		quiet_after = c + QUIET;
	endtask

	// a frame is decided in cycle c
	task automatic decide_hdr_bad(input int f, input int c);
		expect_ev(E_HB, c, f);
		if (frm[f].typ == 8'h01) expect_ev(E_INV, c, f);
		m_fail = sat_inc(m_fail);
		frm[f].done = 1;
	endtask
	task automatic decide_end(input int f, input int c);
		expect_ev(E_FE, c, f);
		if (!frm[f].crc_good) begin
			if (frm[f].typ == 8'h01) expect_ev(E_INV, c, f);
			m_fail = sat_inc(m_fail);
		end else case (frm[f].typ)
			8'h01: begin
				expect_ev(E_COMMIT, c, f);
				m_recv = sat_inc(m_recv);
				if (m_have_prev && frm[f].idx != m_prev + 32'd1) m_gaps = sat_inc(m_gaps);
				m_prev = frm[f].idx; m_have_prev = 1;
			end
			8'h02: expect_ev(E_CFG, c, f);
			8'h03: expect_ev(E_STS, c, f);
		endcase
		frm[f].done = 1;
	endtask
	task automatic decide_cut(input int f, input int c);
		if (frm[f].typ == 8'h01) expect_ev(E_INV, c, f);
		m_fail = sat_inc(m_fail);
		quiet_after = c + QUIET;
		frm[f].done = 1;
	endtask

	// driver: one byte per wclk, registered like frame_rx's dout / hdr_start
	always @(posedge wclk) begin
		ent_t e;
		frm_t fr;
		int f, b, k;
		bit [31:0] c, rc;
		cyc_a++;
		if (q.size() > 0) e = q.pop_front();
		else begin e.b = $urandom; e.hs = 0; e.lab = -1; end
		din_a <= #1 e.b;
		hs_a <= #1 e.hs;
		tag_fid[cyc_a % 8] = -1; tag_bno[cyc_a % 8] = -1;
		if (e.hs) begin
			if (cur >= 0 && !frm[cur].done) decide_cut(cur, cyc_a);
			fr.by.delete(); fr.done = 0; fr.committed = 0; fr.hdr_ok = 0; fr.crc_good = 0;
			fr.what = (e.lab >= 0) ? labels[e.lab] : "?";
			cur = frm.size();
			frm.push_back(fr);
		end
		if (cur >= 0 && !frm[cur].done) begin
			f = cur;
			b = frm[f].by.size();
			frm[f].by.push_back(e.b);
			tag_fid[cyc_a % 8] = f; tag_bno[cyc_a % 8] = b;
			case (b)
				0: frm[f].typ = e.b; 1: frm[f].flags = e.b;
				2: frm[f].idx[7:0] = e.b;   3: frm[f].idx[15:8] = e.b;
				4: frm[f].idx[23:16] = e.b; 5: frm[f].idx[31:24] = e.b;
				6: frm[f].len[7:0] = e.b;   7: frm[f].len[15:8] = e.b;
			endcase
			if (b == 7) begin
				frm[f].hdr_ok = pair_ok(frm[f].typ, frm[f].len);
				if (!frm[f].hdr_ok) decide_hdr_bad(f, cyc_a);
			end else if (b > 7 && b == 8 + frm[f].len + 3) begin
				c = 32'hFFFFFFFF;
				for (k = 0; k < 8 + frm[f].len; k = k + 1) c = crc_byte(c, frm[f].by[k]);
				for (k = 0; k < 4; k = k + 1) rc[8*k +: 8] = frm[f].by[8 + frm[f].len + k];
				frm[f].crc_good = (~c == rc);
				decide_end(f, cyc_a);
			end
		end
	end

	// gap bytes: random, a fake preamble now and then, never with hdr_start
	task automatic push_gap(input int n);
		ent_t e;
		int k;
		e.hs = 0; e.lab = -1;
		for (k = 0; k < n; k = k + 1) begin e.b = $urandom; q.push_back(e); end
		if (n >= 6 && ($urandom % 3) == 0) begin
			// overwrite the last 4 gap bytes with a preamble that no hdr_start follows
			q[q.size() - 5].b = 8'hA5; q[q.size() - 4].b = 8'h5A;
			q[q.size() - 3].b = 8'hA5; q[q.size() - 2].b = 8'h5A;
		end
	endtask

	// corrupt: 0 none, 1 payload bit, 2 flags bit (header), 3 CRC byte bit
	// cut: number of frame bytes sent before the next hdr_start (0: whole frame)
	// npl: payload bytes sent for a bad header (ignored for a good one)
	task automatic push_frame(input [7:0] typ, input [15:0] len, input [31:0] idx,
			input int corrupt, input int cut, input int npl, input string what,
			output int fid);
		bit [7:0] by[$];
		bit [31:0] c;
		ent_t e;
		int k, n, pos;
		begin
			if (pair_ok(typ, len)) npl = len;
			by.push_back(typ); by.push_back($urandom);
			for (k = 0; k < 4; k = k + 1) by.push_back(idx[8*k +: 8]);
			by.push_back(len[7:0]); by.push_back(len[15:8]);
			for (k = 0; k < npl; k = k + 1) by.push_back($urandom);
			c = 32'hFFFFFFFF;
			for (k = 0; k < 8 + npl; k = k + 1) c = crc_byte(c, by[k]);
			c = ~c;
			for (k = 0; k < 4; k = k + 1) by.push_back(c[8*k +: 8]);
			if (corrupt == 1 && npl > 0) begin pos = 8 + ($urandom % npl); by[pos] ^= 8'h01 << ($urandom % 8); end
			if (corrupt == 2) by[1] ^= 8'h01 << ($urandom % 8);
			if (corrupt == 3) begin pos = 8 + npl + ($urandom % 4); by[pos] ^= 8'h01 << ($urandom % 8); end
			fid = labels.size();
			labels.push_back(what);
			e.hs = 0; e.lab = -1;
			e.b = 8'hA5; q.push_back(e); e.b = 8'h5A; q.push_back(e);
			e.b = 8'hA5; q.push_back(e); e.b = 8'h5A; q.push_back(e);
			n = (cut > 0 && cut < by.size()) ? cut : by.size();
			for (k = 0; k < n; k = k + 1) begin
				e.b = by[k]; e.hs = (k == 0); e.lab = (k == 0) ? fid : -1;
				q.push_back(e);
			end
		end
	endtask

	// convenience
	int fid_x;
	reg [31:0] a_idx = 32'd1000;
	task automatic good(input [7:0] typ, input int gap, input string what);
		push_gap(gap);
		push_frame(typ, typ == 8'h01 ? 16'd256 : typ == 8'h02 ? 16'd22 : 16'd18,
			a_idx, 0, 0, 0, what, fid_x);
		if (typ == 8'h01) a_idx = a_idx + 1;
	endtask
	task automatic drain(input int extra);
		wait (q.size() == 0);
		repeat (extra) @(posedge wclk);
	endtask

	// ---------------- Part A checker (negedge: the DUT's outputs are settled) ----------------
	reg [175:0] cfg_prev = 0;
	reg [15:0] sd_prev = 0, sc_prev = 0;
	reg [7:0] sv_prev = 0;
	reg [63:0] sa_prev = 0;
	reg [31:0] so_prev = 0;
	reg checks_a = 0;			// on after reset
	integer n_commit_a = 0, n_inv_a = 0, n_cfg_a = 0, n_sts_a = 0, n_we_a = 0;

	task automatic check_pulse(input int k, input bit p, output int fid);
		fid = -1;
		while (ex[k].size() > 0 && ex[k][0].hi < cyc_a) begin
			err($sformatf("P1 %s missing for frame %0d (%s), due in cycles %0d..%0d",
				ename[k], ex[k][0].fid, frm[ex[k][0].fid].what, ex[k][0].lo, ex[k][0].hi));
			void'(ex[k].pop_front());
		end
		if (p) begin
			if (ex[k].size() > 0 && ex[k][0].lo <= cyc_a) begin
				fid = ex[k][0].fid;
				if (cyc_a - ex[k][0].lo > maxlat[k]) maxlat[k] = cyc_a - ex[k][0].lo;
				void'(ex[k].pop_front());
			end else
				err($sformatf("P2/P4 unexpected %s in cycle %0d (last hdr_start: frame %0d, %s)",
					ename[k], cyc_a, cur, cur >= 0 ? frm[cur].what : "-"));
		end
	endtask

	always @(negedge wclk) if (checks_a) begin
		int f, k, j, found;
		bit [15:0] w;
		bit [175:0] cr;
		// P1/P2 hdr_bad, frame_end
		check_pulse(E_HB, a_hdr_bad, f);
		check_pulse(E_FE, a_frame_end, f);
		if (f >= 0) begin
			if (a_crc_ok !== frm[f].crc_good)
				err($sformatf("P2 frame_end crc_ok %b, expected %b (frame %0d, %s)", a_crc_ok, frm[f].crc_good, f, frm[f].what));
			else if (frm[f].crc_good) begin
				if (a_is_sample !== (frm[f].typ == 8'h01))
					err($sformatf("P2 frame_end is_sample %b for type %h (%s)", a_is_sample, frm[f].typ, frm[f].what));
				if (a_frame_index !== frm[f].idx)
					err($sformatf("P2 frame_end frame_index %h, expected %h (%s)", a_frame_index, frm[f].idx, frm[f].what));
			end
		end
		// P3 writes
		if (a_buf_we === 1'b1) begin
			n_we_a++;
			k = a_buf_waddr;
			found = -1;
			for (j = 0; j <= MAXLAT; j = j + 1)
				if (cyc_a - j >= 1 && tag_fid[(cyc_a - j) % 8] >= 0 && tag_bno[(cyc_a - j) % 8] == 8 + 2 * k + 1
				    && frm[tag_fid[(cyc_a - j) % 8]].typ == 8'h01 && frm[tag_fid[(cyc_a - j) % 8]].hdr_ok
				    && 8 + 2 * k + 1 < 8 + frm[tag_fid[(cyc_a - j) % 8]].len)
					found = tag_fid[(cyc_a - j) % 8];
			if (found < 0)
				err($sformatf("P3 buf_we at word %0d with no SAMPLE byte %0d on din in the last %0d cycles (last hdr_start: %s)",
					k, 8 + 2 * k + 1, MAXLAT + 1, cur >= 0 ? frm[cur].what : "-"));
			else begin
				w = {frm[found].by[8 + 2 * k + 1], frm[found].by[8 + 2 * k]};
				if (a_buf_wdata !== w)
					err($sformatf("P3 buf_wdata %h at word %0d, expected %h (%s)", a_buf_wdata, k, w, frm[found].what));
				if (frm[found].committed) err($sformatf("P3 buf_we after smp_commit (%s)", frm[found].what));
				written_by[k] = found; wword[k] = a_buf_wdata;
			end
		end
		// P3/P4 commit, invalid
		check_pulse(E_COMMIT, a_smp_commit, f);
		if (f >= 0) begin
			n_commit_a++;
			frm[f].committed = 1;
			if (a_smp_index !== frm[f].idx) err($sformatf("P3 smp_index %h, expected %h (%s)", a_smp_index, frm[f].idx, frm[f].what));
			if (a_smp_flags !== frm[f].flags) err($sformatf("P3 smp_flags %h, expected %h (%s)", a_smp_flags, frm[f].flags, frm[f].what));
			found = 0;
			for (k = 0; k < 128; k = k + 1) if (written_by[k] != f) found++;
			if (found) err($sformatf("P3 smp_commit with %0d of 128 words not written by this frame (%s)", found, frm[f].what));
		end
		check_pulse(E_INV, a_smp_invalid, f);
		if (f >= 0) n_inv_a++;
		// P5 CONFIG, STATUS
		check_pulse(E_CFG, a_cfg_strobe, f);
		if (f >= 0) begin
			n_cfg_a++;
			for (k = 0; k < 22; k = k + 1) cr[8*k +: 8] = frm[f].by[8 + k];
			if (a_cfg_regs !== cr) err($sformatf("P5 cfg_regs %h, expected %h (%s)", a_cfg_regs, cr, frm[f].what));
		end else if (a_cfg_regs !== cfg_prev) err("P5 cfg_regs changed outside a cfg_strobe");
		cfg_prev = a_cfg_regs;
		check_pulse(E_STS, a_sts_strobe, f);
		if (f >= 0) begin
			n_sts_a++;
			if (a_sts_design_id !== {frm[f].by[9], frm[f].by[8]}) err($sformatf("P5 sts_design_id %h (%s)", a_sts_design_id, frm[f].what));
			if (a_sts_valid !== frm[f].by[10]) err($sformatf("P5 sts_valid %h (%s)", a_sts_valid, frm[f].what));
			if (a_sts_alt_cmd !== {frm[f].by[13], frm[f].by[12]}) err($sformatf("P5 sts_alt_cmd %h (%s)", a_sts_alt_cmd, frm[f].what));
			if (a_sts_alt_answer !== {frm[f].by[21], frm[f].by[20], frm[f].by[19], frm[f].by[18],
			                          frm[f].by[17], frm[f].by[16], frm[f].by[15], frm[f].by[14]})
				err($sformatf("P5 sts_alt_answer %h (%s)", a_sts_alt_answer, frm[f].what));
			if (a_sts_fifo0_overflow !== {frm[f].by[25], frm[f].by[24], frm[f].by[23], frm[f].by[22]})
				err($sformatf("P5 sts_fifo0_overflow %h (%s)", a_sts_fifo0_overflow, frm[f].what));
		end else if (a_sts_design_id !== sd_prev || a_sts_valid !== sv_prev || a_sts_alt_cmd !== sc_prev ||
		             a_sts_alt_answer !== sa_prev || a_sts_fifo0_overflow !== so_prev)
			err("P5 sts_* changed outside an sts_strobe");
		sd_prev = a_sts_design_id; sv_prev = a_sts_valid; sc_prev = a_sts_alt_cmd;
		sa_prev = a_sts_alt_answer; so_prev = a_sts_fifo0_overflow;
		// P6 counters, once quiet
		if (cyc_a > quiet_after) begin
			// on a mismatch: report once, then follow the DUT, so later events are still checked
			if (a_frames_received !== m_recv) begin
				err($sformatf("P6 frames_received %0d, expected %0d (last hdr_start: %s)", a_frames_received, m_recv, cur >= 0 ? frm[cur].what : "-"));
				m_recv = a_frames_received;
			end
			if (a_frames_crc_failed !== m_fail) begin
				err($sformatf("P6 frames_crc_failed %0d, expected %0d (last hdr_start: %s)", a_frames_crc_failed, m_fail, cur >= 0 ? frm[cur].what : "-"));
				m_fail = a_frames_crc_failed;
			end
			if (a_frame_index_gaps !== m_gaps) begin
				err($sformatf("P6 frame_index_gaps %0d, expected %0d (last hdr_start: %s)", a_frame_index_gaps, m_gaps, cur >= 0 ? frm[cur].what : "-"));
				m_gaps = a_frame_index_gaps;
			end
		end
	end

	// ======================================================================
	// Part B: frame_rx + frame_parse on Kuntur's bit stream
	// ======================================================================
	reg rstb_b = 0, lock_b = 0;
	reg [7:0] din_b = 0;
	wire b_pll_phasestep, b_pll_phasedir, b_hdr_start, b_dout_en, b_locked, b_stepping;
	wire [7:0] b_dout;
	wire [2:0] b_bitsel;
	wire [31:0] b_relocks;
	wire b_hdr_bad, b_frame_end, b_crc_ok, b_is_sample;
	wire [31:0] b_frame_index;
	wire b_buf_we;
	wire [6:0] b_buf_waddr;
	wire [15:0] b_buf_wdata;
	wire b_smp_commit, b_smp_invalid;
	wire [31:0] b_smp_index;
	wire [7:0] b_smp_flags;
	wire b_cfg_strobe;
	wire [175:0] b_cfg_regs;
	wire b_sts_strobe;
	wire [15:0] b_sts_design_id, b_sts_alt_cmd;
	wire [7:0] b_sts_valid;
	wire [63:0] b_sts_alt_answer;
	wire [31:0] b_sts_fifo0_overflow;
	wire [31:0] b_frames_received, b_frames_crc_failed, b_frame_index_gaps;

	frame_rx #(.STORED_PHASE(2), .LOCK_TIMEOUT(9'd100), .N_FAIL(4'd8)) rx (
		.rstb(rstb_b), .clk(clk), .wclk(wclk), .lock(lock_b), .din(din_b),
		.hdr_bad(b_hdr_bad), .frame_end(b_frame_end), .crc_ok(b_crc_ok),
		.is_sample(b_is_sample), .frame_index(b_frame_index),
		.pll_phasestep(b_pll_phasestep), .pll_phasedir(b_pll_phasedir),
		.dout(b_dout), .hdr_start(b_hdr_start), .dout_en(b_dout_en),
		.locked(b_locked), .stepping(b_stepping), .bitsel(b_bitsel), .relocks(b_relocks)
	);

	frame_parse pb (
		.rstb(rstb_b), .wclk(wclk), .din(b_dout), .hdr_start(b_hdr_start),
		.hdr_bad(b_hdr_bad), .frame_end(b_frame_end), .crc_ok(b_crc_ok),
		.is_sample(b_is_sample), .frame_index(b_frame_index),
		.buf_we(b_buf_we), .buf_waddr(b_buf_waddr), .buf_wdata(b_buf_wdata),
		.smp_commit(b_smp_commit), .smp_invalid(b_smp_invalid),
		.smp_index(b_smp_index), .smp_flags(b_smp_flags),
		.cfg_strobe(b_cfg_strobe), .cfg_regs(b_cfg_regs),
		.sts_strobe(b_sts_strobe), .sts_design_id(b_sts_design_id),
		.sts_valid(b_sts_valid), .sts_alt_cmd(b_sts_alt_cmd),
		.sts_alt_answer(b_sts_alt_answer), .sts_fifo0_overflow(b_sts_fifo0_overflow),
		.frames_received(b_frames_received), .frames_crc_failed(b_frames_crc_failed),
		.frame_index_gaps(b_frame_index_gaps)
	);

	// ---------------- stream generator (as frame_rx_tb, payloads kept) ----------------
	integer s_off = 3;
	integer bad_crc_n = 0;		// next n frames (any type): one payload bit flipped
	integer bad_hdr_n = 0;		// next n SAMPLEs: payload_len 255
	integer bad_pre_n = 0;		// next n SAMPLEs: preamble byte 2 corrupted
	bit b_on = 0;				// serialiser running

	bit bits[$];
	typedef struct { int unsigned bitpos; int kind; } mark_t;	// 0 period, 1 good SAMPLE, 2 good aux
	mark_t marks[$];
	int unsigned bits_out = 0, bits_in = 0;
	reg [31:0] gen_index = 32'h0000_0100;
	integer gen_period = 0;
	integer sent_periods = 0, sent_smp = 0, sent_aux = 0;

	typedef bit [15:0] words_t [0:127];
	words_t sent_words [bit [31:0]];
	bit [7:0] sent_flags [bit [31:0]];
	bit [175:0] sent_cfg_q[$], sent_sts_q[$];	// aux payloads sent intact, reg[i] / byte i in bits 8i+7..8i

	task automatic push_byte(input [7:0] b);
		integer k;
		for (k = 0; k < 8; k = k + 1) begin bits.push_back(b[k]); bits_in++; end
	endtask

	task automatic push_frame_b(input [7:0] typ, input [15:0] len, output integer nbytes);
		reg [7:0] hdr [0:7];
		reg [7:0] pl [0:255];
		reg [31:0] c;
		reg cc, ch, cp;
		integer k, flip;
		mark_t m;
		bit [175:0] a;
		words_t w;
		begin
			cc = 0; ch = 0; cp = 0;
			if (bad_crc_n > 0) begin cc = 1; bad_crc_n--; end	// any type: the next is always a SAMPLE
			else if (typ == 8'h01 && bad_hdr_n > 0) begin ch = 1; bad_hdr_n--; end
			else if (typ == 8'h01 && bad_pre_n > 0) begin cp = 1; bad_pre_n--; end
			hdr[0] = typ; hdr[1] = $urandom;
			{hdr[5], hdr[4], hdr[3], hdr[2]} = gen_index;
			{hdr[7], hdr[6]} = ch ? 16'd255 : len;
			for (k = 0; k < len; k = k + 1) begin
				pl[k] = $urandom;
				if (k >= 3 && pl[k] == 8'h5A && pl[k-1] == 8'hA5 && pl[k-2] == 8'h5A && pl[k-3] == 8'hA5)
					pl[k] = 8'h00;
			end
			c = 32'hFFFFFFFF;
			for (k = 0; k < 8; k = k + 1) c = crc_byte(c, hdr[k]);
			for (k = 0; k < len; k = k + 1) c = crc_byte(c, pl[k]);
			c = ~c;
			if (cc) begin flip = $urandom % len; pl[flip] = pl[flip] ^ (8'h01 << ($urandom % 8)); end
			push_byte(8'hA5); push_byte(8'h5A);
			push_byte(cp ? 8'hA4 : 8'hA5); push_byte(8'h5A);
			for (k = 0; k < 8; k = k + 1) push_byte(hdr[k]);
			for (k = 0; k < len; k = k + 1) push_byte(pl[k]);
			push_byte(c[7:0]); push_byte(c[15:8]); push_byte(c[23:16]); push_byte(c[31:24]);
			if (!(cc || ch || cp)) begin
				m.bitpos = bits_in; m.kind = (typ == 8'h01) ? 1 : 2;
				marks.push_back(m);
				if (typ == 8'h01) begin
					for (k = 0; k < 128; k = k + 1) w[k] = {pl[2*k+1], pl[2*k]};
					sent_words[gen_index] = w; sent_flags[gen_index] = hdr[1];
				end else begin
					a = 0;
					for (k = 0; k < len; k = k + 1) a[8*k +: 8] = pl[k];
					if (typ == 8'h02) sent_cfg_q.push_back(a);
					else begin a[31:24] = 0; sent_sts_q.push_back(a); end	// STATUS byte 3 reserved: not compared
				end
			end
			nbytes = 4 + 8 + len + 4;
		end
	endtask

	task automatic gen_one_period;
		integer used, n, k, slot;
		mark_t m;
		begin
			slot = (gen_period % 2) ? 380 : 379;
			m.bitpos = bits_in; m.kind = 0; marks.push_back(m);
			used = 0;
			push_frame_b(8'h01, 16'd256, n); used = used + n;
			if (gen_period % CFG_EVERY == 1) begin push_frame_b(8'h02, 16'd22, n); used = used + n; end
			else if (gen_period % STS_EVERY == 2) begin push_frame_b(8'h03, 16'd18, n); used = used + n; end
			for (k = used; k < slot; k = k + 1) push_byte(8'h00);
			gen_index = gen_index + 1;
			gen_period++;
		end
	endtask

	always @(posedge wclk) if (b_on) begin
		reg [7:0] w;
		integer k;
		while (bits.size() < 4000) gen_one_period;
		for (k = 0; k < 8; k = k + 1) w[k] = bits.pop_front();
		bits_out = bits_out + 8;
		while (marks.size() > 0 && marks[0].bitpos <= bits_out) begin
			case (marks[0].kind) 0: sent_periods++; 1: sent_smp++; 2: sent_aux++; endcase
			void'(marks.pop_front());
		end
		din_b <= #1 w;
	end

	// ---------------- Part B checker ----------------
	bit checks_b = 0;
	integer n_commit_b = 0, n_inv_b = 0, n_aux_b = 0;
	reg [15:0] bank_b [0:127];
	bit wr_b [0:127];
	reg [31:0] last_commit_b = 0;
	bit have_commit_b = 0;

	// the strobed payload must be one sent intact; older ones (lost before lock) are dropped
	task automatic match_aux(input bit is_cfg, input [175:0] v, input string what);
		bit ok;
		ok = 0;
		if (is_cfg) while (sent_cfg_q.size() > 0 && !ok) begin ok = (sent_cfg_q[0] == v); void'(sent_cfg_q.pop_front()); end
		else        while (sent_sts_q.size() > 0 && !ok) begin ok = (sent_sts_q[0] == v); void'(sent_sts_q.pop_front()); end
		if (!ok) err($sformatf("B1 %s payload %h matches no %s sent", what, v, is_cfg ? "CONFIG" : "STATUS"));
	endtask

	always @(negedge wclk) if (checks_b) begin
		int k, bad;
		if (b_buf_we === 1'b1) begin bank_b[b_buf_waddr] = b_buf_wdata; wr_b[b_buf_waddr] = 1; end
		if (b_smp_commit === 1'b1) begin
			n_commit_b++;
			if (!sent_words.exists(b_smp_index))
				err($sformatf("B1 smp_commit of index %h, never sent intact", b_smp_index));
			else begin
				bad = 0;
				for (k = 0; k < 128; k = k + 1) if (!wr_b[k] || bank_b[k] !== sent_words[b_smp_index][k]) bad++;
				if (bad) err($sformatf("B1 smp_commit index %h: %0d of 128 words wrong or not written", b_smp_index, bad));
				if (b_smp_flags !== sent_flags[b_smp_index]) err($sformatf("B1 smp_flags %h, sent %h", b_smp_flags, sent_flags[b_smp_index]));
			end
			if (have_commit_b && b_smp_index <= last_commit_b)
				err($sformatf("B1 smp_commit index %h after %h", b_smp_index, last_commit_b));
			last_commit_b = b_smp_index; have_commit_b = 1;
		end
		if (b_smp_commit === 1'b1 || b_smp_invalid === 1'b1) for (k = 0; k < 128; k = k + 1) wr_b[k] = 0;
		if (b_smp_invalid === 1'b1) n_inv_b++;
		if (b_cfg_strobe === 1'b1) begin n_aux_b++; match_aux(1, b_cfg_regs, "cfg_strobe"); end
		if (b_sts_strobe === 1'b1) begin
			n_aux_b++;
			match_aux(0, {32'd0, b_sts_fifo0_overflow, b_sts_alt_answer, b_sts_alt_cmd, 8'h00, b_sts_valid, b_sts_design_id}, "sts_strobe");
		end
	end

	task automatic wait_periods(input integer n);
		integer p0;
		begin p0 = sent_periods; wait (sent_periods >= p0 + n); end
	endtask

	task automatic expect_lock(input string what, input integer lim);
		integer p0;
		begin
			p0 = sent_periods;
			fork : wl
				wait (b_locked === 1'b1);
				wait (sent_periods >= p0 + lim);
			join_any
			disable wl;
			if (b_locked !== 1'b1) err($sformatf("B1 %s: no lock within %0d periods", what, lim));
			else $display("%t   %s: locked after %0d periods, bitsel %0d", $realtime, what, sent_periods - p0, b_bitsel);
		end
	endtask

	task automatic expect_unlock(input string what, input integer lim);
		integer p0;
		begin
			p0 = sent_periods;
			fork : wu
				wait (b_locked === 1'b0);
				wait (sent_periods >= p0 + lim);
			join_any
			disable wu;
			if (b_locked !== 1'b0) err($sformatf("B2 %s: still locked after %0d periods", what, lim));
		end
	endtask

	// a stretch of n periods: deltas of everything against what was sent
	integer l_fall_b = 0;
	reg locked_bq = 0;
	always @(posedge wclk) begin if (!b_locked && locked_bq) l_fall_b++; locked_bq <= b_locked; end

	task automatic stretch(input string what, input integer n, input integer d_inv, input integer d_fail, input integer d_gaps);
		integer s0, a0, c0, x0, i0, l0;
		reg [31:0] r0, f0, g0;
		begin
			wait_periods(1);
			s0 = sent_smp; a0 = sent_aux; c0 = n_commit_b; x0 = n_aux_b; i0 = n_inv_b; l0 = l_fall_b;
			r0 = b_frames_received; f0 = b_frames_crc_failed; g0 = b_frame_index_gaps;
			wait_periods(n);
			if (n_commit_b - c0 != sent_smp - s0)
				err($sformatf("B %s: %0d good SAMPLEs sent, %0d committed", what, sent_smp - s0, n_commit_b - c0));
			if (n_aux_b - x0 != sent_aux - a0)
				err($sformatf("B %s: %0d aux frames sent, %0d strobed", what, sent_aux - a0, n_aux_b - x0));
			if (n_inv_b - i0 != d_inv) err($sformatf("B %s: smp_invalid + %0d, expected + %0d", what, n_inv_b - i0, d_inv));
			if (b_frames_received - r0 != n_commit_b - c0)
				err($sformatf("B %s: frames_received + %0d, commits + %0d", what, b_frames_received - r0, n_commit_b - c0));
			if (b_frames_crc_failed - f0 != d_fail)
				err($sformatf("B %s: frames_crc_failed + %0d, expected + %0d", what, b_frames_crc_failed - f0, d_fail));
			if (b_frame_index_gaps - g0 != d_gaps)
				err($sformatf("B %s: frame_index_gaps + %0d, expected + %0d", what, b_frame_index_gaps - g0, d_gaps));
			if (l_fall_b != l0) err($sformatf("B %s: lock lost", what));
			$display("%t   %s: %0d periods, %0d SAMPLEs committed, %0d aux, %0d invalid", $realtime, what, n,
				n_commit_b - c0, n_aux_b - x0, n_inv_b - i0);
		end
	endtask

	// ======================================================================
	// scenario
	// ======================================================================
	integer seed = 1, soak = 400, r0, k, i;
	initial begin
		if ($value$plusargs("seed=%d", seed)) ;
		if ($value$plusargs("off=%d", s_off)) ;
		if ($value$plusargs("soak=%d", soak)) ;
		r0 = $urandom(seed);
		for (k = 0; k < 128; k = k + 1) written_by[k] = -1;
		for (k = 0; k < 8; k = k + 1) tag_fid[k] = -1;
		$display("frame_parse_tb: seed %0d, part B offset %0d, soak %0d", seed, s_off, soak);
		begin
			reg [31:0] c; reg [8*9-1:0] s; c = 32'hFFFFFFFF; s = "123456789";
			for (k = 8; k >= 0; k = k - 1) c = crc_byte(c, s[8*k +: 8]);
			if (~c != 32'hCBF43926) err("tb CRC self-test");
		end

		// ---------------- Part A ----------------
		#(10 * T_W) rstb_a = 1;
		@(negedge wclk); checks_a = 1;
		cfg_prev = a_cfg_regs; sd_prev = a_sts_design_id; sv_prev = a_sts_valid; sc_prev = a_sts_alt_cmd;
		sa_prev = a_sts_alt_answer; so_prev = a_sts_fifo0_overflow;
		push_gap(50);

		$display("A1 good frames of each type, preamble-only and random gaps");
		good(8'h01, 0, "A1 first SAMPLE (index 1000, no gap count)");
		good(8'h02, 0, "A1 CONFIG right after");
		good(8'h01, 120, "A1 SAMPLE");
		good(8'h03, 0, "A1 STATUS right after");
		good(8'h01, 95, "A1 SAMPLE");
		good(8'h01, 3, "A1 SAMPLE");
		good(8'h02, 40, "A1 CONFIG alone");
		good(8'h03, 7, "A1 STATUS alone");
		good(8'h01, 0, "A1 SAMPLE");
		drain(20);

		$display("A2 frame_index gaps");
		a_idx = 32'd2000; good(8'h01, 10, "A2 index jump +994 (gap)");
		good(8'h01, 10, "A2 +1");
		a_idx = 32'd1999; good(8'h01, 10, "A2 index back by 2 (gap)");
		a_idx = 32'hFFFFFFFF; good(8'h01, 10, "A2 index FFFFFFFF (gap)");
		good(8'h01, 10, "A2 index 0 after FFFFFFFF (no gap)");
		good(8'h01, 10, "A2 index 1");
		drain(20);

		$display("A3 bad CRC: SAMPLE, CONFIG, STATUS; payload, flags, CRC byte");
		for (i = 1; i <= 3; i = i + 1) begin
			push_gap(20); push_frame(8'h01, 16'd256, a_idx, i, 0, 0, $sformatf("A3 SAMPLE bad CRC (%0d)", i), fid_x); a_idx++;
			push_gap(20); push_frame(8'h02, 16'd22, a_idx, i, 0, 0, $sformatf("A3 CONFIG bad CRC (%0d)", i), fid_x);
			push_gap(20); push_frame(8'h03, 16'd18, a_idx, i, 0, 0, $sformatf("A3 STATUS bad CRC (%0d)", i), fid_x);
		end
		good(8'h01, 10, "A3 good SAMPLE after (gap)");
		good(8'h02, 0, "A3 good CONFIG after");
		good(8'h03, 0, "A3 good STATUS after");
		drain(20);

		$display("A4 bad headers (R3)");
		push_gap(10); push_frame(8'h01, 16'd255, a_idx, 0, 0, 255, "A4 (01,255)", fid_x);
		push_gap(10); push_frame(8'h01, 16'd22, a_idx, 0, 0, 22, "A4 (01,22)", fid_x);
		push_gap(10); push_frame(8'h01, 16'h0001, a_idx, 0, 0, 30, "A4 (01, len bytes 01 00 = 1: LE check)", fid_x);
		push_gap(10); push_frame(8'h01, 16'hFFFF, a_idx, 0, 0, 40, "A4 (01,65535), cut by the next frame", fid_x);
		push_gap(0);  push_frame(8'h02, 16'd256, a_idx, 0, 0, 20, "A4 (02,256)", fid_x);
		push_gap(10); push_frame(8'h02, 16'h1600, a_idx, 0, 0, 20, "A4 (02, len bytes 00 16: LE check)", fid_x);
		push_gap(10); push_frame(8'h03, 16'd22, a_idx, 0, 0, 20, "A4 (03,22)", fid_x);
		push_gap(10); push_frame(8'h05, 16'd256, a_idx, 0, 0, 20, "A4 (05,256)", fid_x);
		push_gap(10); push_frame(8'h00, 16'd0, a_idx, 0, 0, 0, "A4 (00,0)", fid_x);
		good(8'h01, 10, "A4 good SAMPLE after");
		good(8'h03, 0, "A4 good STATUS after");
		drain(20);

		$display("A5 frames cut short by hdr_start (Q2)");
		begin
			int cuts[$];
			cuts = '{1, 3, 5, 7, 8, 9, 100, 263, 264, 267};
			foreach (cuts[j]) begin
				push_gap(10);
				push_frame(8'h01, 16'd256, a_idx, 0, cuts[j], 0, $sformatf("A5 SAMPLE cut after %0d bytes", cuts[j]), fid_x);
				a_idx++;
				good(8'h01, 0, $sformatf("A5 good SAMPLE cutting it (after %0d)", cuts[j]));
			end
			cuts = '{1, 3, 8, 20, 33};
			foreach (cuts[j]) begin
				push_gap(10); push_frame(8'h02, 16'd22, a_idx, 0, cuts[j], 0, $sformatf("A5 CONFIG cut after %0d", cuts[j]), fid_x);
				good(8'h01, 0, "A5 SAMPLE cutting a CONFIG");
				push_gap(10); push_frame(8'h03, 16'd18, a_idx, 0, cuts[j] > 29 ? 29 : cuts[j], 0, $sformatf("A5 STATUS cut after %0d", cuts[j]), fid_x);
				good(8'h03, 0, "A5 STATUS cutting a STATUS");
			end
			push_gap(10); push_frame(8'h01, 16'd255, a_idx, 0, 9, 255, "A5 bad header, then hdr_start (already failed: not cut)", fid_x);
			good(8'h01, 0, "A5 SAMPLE after a bad header");
		end
		drain(20);

		$display("A6 random soak, %0d frames", soak);
		for (i = 0; i < soak; i = i + 1) begin
			int r, t, cor, cut, npl;
			bit [15:0] len;
			r = $urandom % 100;
			t = ($urandom % 3) + 1;
			len = (t == 1) ? 256 : (t == 2) ? 22 : 18;
			cor = 0; cut = 0; npl = 0;
			if (r < 12) cor = ($urandom % 3) + 1;
			else if (r < 20) begin len = $urandom % 300; npl = $urandom % 40; end
			else if (r < 26) cut = ($urandom % (8 + len + 4 - 1)) + 1;
			else if (r < 28) t = $urandom % 256;
			push_gap(($urandom % 3 == 0) ? 0 : $urandom % 30);
			push_frame(t, len, a_idx, cor, cut, npl, $sformatf("A6 #%0d", i), fid_x);
			if (t == 1 && ($urandom % 10) != 0) a_idx++;
			if (q.size() > 2000) wait (q.size() < 500);
		end
		push_gap(10); good(8'h01, 0, "A6 last");
		drain(30);

`ifdef FP_SAT
		$display("A7 counters saturate (FP_SAT)");
		@(negedge wclk);
		pa.frames_received = MAXC - 1; pa.frames_crc_failed = MAXC - 1; pa.frame_index_gaps = MAXC - 1;
		m_recv = MAXC - 1; m_fail = MAXC - 1; m_gaps = MAXC - 1;
		for (i = 0; i < 3; i = i + 1) begin
			a_idx = a_idx + 2; good(8'h01, 10, "A7 SAMPLE with a gap");
			push_gap(10); push_frame(8'h01, 16'd256, a_idx, 1, 0, 0, "A7 bad CRC", fid_x);
		end
		drain(30);
		if (a_frames_received !== MAXC || a_frames_crc_failed !== MAXC || a_frame_index_gaps !== MAXC)
			err($sformatf("P6 counters not saturated: %h %h %h", a_frames_received, a_frames_crc_failed, a_frame_index_gaps));
`endif
		drain(10);
		checks_a = 0;
		for (k = 0; k < 6; k = k + 1)
			if (ex[k].size() > 0) err($sformatf("P1 %0d %s still expected at the end of part A", ex[k].size(), ename[k]));
		$display("A: %0d frames, %0d commits, %0d invalid, %0d CONFIG, %0d STATUS, %0d writes; counters %0d / %0d / %0d",
			frm.size(), n_commit_a, n_inv_a, n_cfg_a, n_sts_a, n_we_a, a_frames_received, a_frames_crc_failed, a_frame_index_gaps);
		$display("A: latency (wclk after the deciding byte): hdr_bad %0d, frame_end %0d, commit %0d, invalid %0d, cfg %0d, sts %0d",
			maxlat[E_HB], maxlat[E_FE], maxlat[E_COMMIT], maxlat[E_INV], maxlat[E_CFG], maxlat[E_STS]);

		// ---------------- Part B ----------------
		$display("B1 frame_rx + frame_parse, stream offset %0d", s_off);
		for (k = 0; k < s_off; k = k + 1) begin bits.push_back(1'b0); bits_in++; end
		b_on = 1; lock_b = 1;
		#(20 * T_W) rstb_b = 1;
		@(negedge wclk); checks_b = 1;
		expect_lock("B1", LOCK_LIM);
		stretch("B1 clean", 2 * CFG_EVERY * STS_EVERY, 0, 0, 0);

		$display("B2 one bad-CRC SAMPLE, one bad-header SAMPLE, one SAMPLE without preamble");
		bad_crc_n = 1;
		stretch("B2 bad CRC", 4, 1, 1, 1);
		bad_hdr_n = 1;
		stretch("B2 bad header", 4, 1, 1, 1);
		bad_pre_n = 1;
		stretch("B2 no preamble", 4, 0, 0, 1);
		stretch("B2 clean", 6, 0, 0, 0);

		$display("B3 8 consecutive bad-CRC frames: unlock, relock");
		bad_crc_n = 8;
		expect_unlock("B3", 14);
		expect_lock("B3 relock", LOCK_LIM);
		stretch("B3 clean", 12, 0, 0, 0);

		checks_b = 0;
		$display("B: %0d commits, %0d aux, %0d invalid; counters %0d / %0d / %0d; relocks %0d",
			n_commit_b, n_aux_b, n_inv_b, b_frames_received, b_frames_crc_failed, b_frame_index_gaps, b_relocks);
		err_summary;
		if (errors == 0) $display("frame_parse_tb: PASS");
		else $display("frame_parse_tb: FAIL, %0d errors", errors);
		$finish;
	end

	initial begin #400ms; err("timeout"); err_summary; $display("frame_parse_tb: FAIL"); $finish; end
endmodule
