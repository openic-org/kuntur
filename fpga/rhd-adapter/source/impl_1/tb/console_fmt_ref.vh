// console_fmt_ref.vh — the tb's own console line formatter (companion-console.md
// §3, §8.3 F0-F4), included by console_fmt_tb.sv in the tb and in its
// REF_MODEL. Byte arrays only: Icarus 12's SV strings crash on this code.
// mkline() fills lb[0:ll-1] with one line, \r\n included.
// mut: 0 = the spec; others = console_fmt_tb's mutants (see its header).

reg [7:0] lb [0:255];
integer ll;

task put(input [7:0] c);
	begin lb[ll] = c; ll = ll + 1; end
endtask

// a literal, right-aligned in 80 bytes; leading NUL bytes are skipped
task puts(input [8*80-1:0] s);
	integer i;
	begin for (i = 79; i >= 0; i = i - 1) if (s[8*i +: 8] != 8'h00) put(s[8*i +: 8]); end
endtask

task putd(input [31:0] v, input integer mut, input integer width);
	reg [7:0] d [0:9];
	reg [31:0] x;
	integer n, i;
	begin
		x = v; n = 0;
		begin : digits
			forever begin
				d[n] = 8'h30 + x % 10; x = x / 10; n = n + 1;
				if (x == 0) disable digits;
			end
		end
		while (n < width) begin d[n] = "0"; n = n + 1; end
		if (mut == 1) for (i = 0; i < n; i = i + 1) put(d[i]);
		else for (i = n - 1; i >= 0; i = i - 1) put(d[i]);
	end
endtask

function [7:0] hexc(input [3:0] h, input integer lower);
	hexc = (h < 10) ? 8'h30 + h : ((lower ? 8'h61 : 8'h41) + h - 10);
endfunction

task mkline(input [31:0] t, input lock, input snap_ok, input rdy, input valid,
		input [8:0] ph, input [8:0] ec, input [8:0] ew, input [31:0] rx,
		input [31:0] crc, input mode_pattern, input integer mut);
	reg [7:0] x;
	integer i;
	begin
		ll = 0;
		puts("CMP t="); putd(t, mut, (mut == 8) ? 10 : 0);
		puts(" link=- lock="); put(lock ? "1" : "0");
		puts(" ph="); putd(ph, mut, 0); puts("/336 eye=");
		if (!rdy && mut != 3) puts("---/---");
		else if (!valid) puts("0/0");
		else begin putd(ec, mut, 0); put("/"); putd(ew, mut, 0); end
		puts(" mode="); if (mode_pattern) puts("pattern"); else puts("frames");
		puts(" rx="); putd(rx, mut, 0); puts(" crc="); putd(crc, mut, 0);
		puts(" sent=- rep=- skip=- gap=- down=- hw=- mm=- design=- flags=- snap=");
		if (snap_ok ^ (mut == 6)) puts("ok"); else puts("stale");
		put(" ");
		x = 0;
		for (i = 0; i < ll; i = i + 1) x = x ^ lb[i];
		if (mut == 2) x = x ^ "*";
		put("*"); put(hexc(x[7:4], mut == 7)); put(hexc(x[3:0], mut == 7));
		put(8'h0D); put(8'h0A);
	end
endtask
