//======================================================================
//  tb_linebuf_equiv.v  行缓存等价性验证
//  新的 BRAM 环形缓冲版 extract_region_c_shiftram_xtap 与原移位链版
//  (sim/ref/ref_shiftram_xtap.v) 在随机 clken、随机数据下逐拍对比 taps
//  覆盖: 小间距(DISTANCE=2/5)与实际图像宽度(1280), 2/3 个抽头
//======================================================================
`timescale 1ns/1ps
module tb_linebuf_equiv;
	reg clk = 0;
	always #5 clk = ~clk;
	integer errors = 0, checks = 0;

	`define DUT(NAME, D, W, T) \
	reg        en_``NAME; \
	reg [W-1:0] din_``NAME; \
	wire [T*W-1:0] tn_``NAME, tr_``NAME; \
	extract_region_c_shiftram_xtap #(.DISTANCE(D),.DWIDTH(W),.TAPS_NUM(T)) u_new_``NAME( \
		.clken(en_``NAME),.clock(clk),.shiftin(din_``NAME),.shiftout(),.taps(tn_``NAME)); \
	ref_shiftram_xtap #(.DISTANCE(D),.DWIDTH(W),.TAPS_NUM(T)) u_ref_``NAME( \
		.clken(en_``NAME),.clock(clk),.shiftin(din_``NAME),.shiftout(),.taps(tr_``NAME)); \
	always @(negedge clk) begin \
		if (^tr_``NAME !== 1'bx) begin \
			checks = checks + 1; \
			if (tn_``NAME !== tr_``NAME) begin \
				errors = errors + 1; \
				if (errors < 10) $display("  [FAIL] %s t=%0t new=%h ref=%h", `"NAME`", $time, tn_``NAME, tr_``NAME); \
			end \
		end \
	end \
	always @(posedge clk) begin \
		en_``NAME  <= ($random & 3) != 0; \
		din_``NAME <= $random; \
	end

	`DUT(a, 2, 8, 2)
	`DUT(b, 5, 3, 3)
	`DUT(c, 1280, 14, 2)

	initial begin
		en_a = 0; en_b = 0; en_c = 0;
		repeat (40000) @(posedge clk);
		$display("=========================================================");
		$display(" linebuf equivalence: %0d compares, %0d errors", checks, errors);
		if (errors == 0) $display(" RESULT: ALL CHECKS PASSED");
		else             $display(" RESULT: FAILED");
		$display("=========================================================");
		$finish;
	end
endmodule
