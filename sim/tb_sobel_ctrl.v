`timescale 1ns/1ps
//======================================================================
// sobel_ctrl 单元测试 (两键方案)
//   K1: 阈值-   K2: 阈值+   K1+K2同时按: 切显示模式
//
// 说明: 测试里把 CLK_FREQ 设成 1000, 这样 1 个时钟 = 1ms 模型时间,
//       消抖20 / 连发起始500 / 连发间隔80 都按毫秒直接对应周期数。
//   T1 复位默认值        阈值256 模式2
//   T2 K1短按            阈值 -8, 只减一次
//   T3 K2短按            阈值 +8
//   T4 按键抖动          不应产生任何变化
//   T5 按住K1 连减       阈值到 0 后钳位不再变
//   T6 按住K2 连加       阈值到 2040 后钳位不再变
//   T7 K1+K2 组合键      模式循环 2->0->1->2, 且组合期间阈值不变
//   T8 cfg_toggle        配置变化后翻转(跨时钟域握手用)
//
// 运行: run_sim.bat
//======================================================================
module tb_sobel_ctrl;

localparam CLK_FREQ = 1000;		//模型频率, 1个时钟=1ms

integer errors = 0;

reg clk = 1'b0;
reg rst_n = 1'b0;
always #5 clk = ~clk;

reg key1_n = 1'b1;
reg key2_n = 1'b1;

wire [10:0] threshold;
wire [1:0]  mode;
wire        cfg_toggle;

sobel_ctrl #(
	.CLK_FREQ(CLK_FREQ)
) dut (
	.clk(clk), .rst_n(rst_n),
	.key1_n(key1_n), .key2_n(key2_n),
	.threshold(threshold), .mode(mode), .cfg_toggle(cfg_toggle)
);

//---------------- 驱动任务 (单位: 时钟周期 = 模型ms) ----------------
task k1_press(input integer ms);	//按住K1 ms 毫秒
begin
	@(negedge clk); key1_n = 1'b0;
	repeat(ms) @(posedge clk);
	@(negedge clk); key1_n = 1'b1;
	repeat(30) @(posedge clk);		//等松开的消抖
end
endtask

task k2_press(input integer ms);	//按住K2 ms 毫秒
begin
	@(negedge clk); key2_n = 1'b0;
	repeat(ms) @(posedge clk);
	@(negedge clk); key2_n = 1'b1;
	repeat(30) @(posedge clk);
end
endtask

task combo_press(input integer ms);	//同时按住两个键
begin
	@(negedge clk); key1_n = 1'b0; key2_n = 1'b0;
	repeat(ms) @(posedge clk);
	@(negedge clk); key1_n = 1'b1; key2_n = 1'b1;
	repeat(30) @(posedge clk);
end
endtask

task chk(input integer cond, input [8*64:1] name);
begin
	if(cond) $display("   [PASS] %0s", name);
	else begin
		$display("   [FAIL] %0s  (threshold=%0d mode=%0d)", name, threshold, mode);
		errors = errors + 1;
	end
end
endtask

integer t0;

initial begin
	$display("=========================================================");
	$display(" sobel_ctrl unit test (2 keys)");
	$display("=========================================================");

	rst_n = 1'b0;
	repeat(20) @(posedge clk);
	rst_n = 1'b1;
	repeat(10) @(posedge clk);

	//---- T1 复位默认值 ----
	$display("");
	$display("[T1] reset default");
	$display("   threshold=%0d mode=%0d", threshold, mode);
	chk(threshold == 11'd256, "default threshold = 256");
	chk(mode == 2'd2, "default mode = 2 (binary edge)");

	//---- T2 K1 短按 ----
	//---- T1b 上电自动推送配置 ----
	// 	复位后即使不按键, cfg_toggle 也应该自动翻转一次(否则目的域会停在旧默认值)
	begin : pwr
		reg tog0;
		tog0 = cfg_toggle;
		repeat(40) @(posedge clk);
		$display("   cfg_toggle after reset: %0d -> %0d", tog0, cfg_toggle);
		chk(cfg_toggle != tog0, "power-on config push happened");
	end

	$display("");
	$display("[T2] K1 short press -> threshold -8");
	t0 = threshold;
	k1_press(60);
	$display("   threshold %0d -> %0d", t0, threshold);
	chk(threshold == (t0-11'd8), "K1 short press decreases once");

	//---- T3 K2 短按 ----
	$display("");
	$display("[T3] K2 short press -> threshold +8");
	t0 = threshold;
	k2_press(60);
	$display("   threshold %0d -> %0d", t0, threshold);
	chk(threshold == (t0+11'd8), "K2 short press increases once");

	//---- T4 按键抖动 ----
	$display("");
	$display("[T4] jitter on K1 (5ms low / 5ms high x4) -> no change");
	t0 = threshold;
	begin : jitter
		integer j;
		for(j=0;j<4;j=j+1) begin
			@(negedge clk); key1_n = 1'b0; repeat(5) @(posedge clk);
			@(negedge clk); key1_n = 1'b1; repeat(5) @(posedge clk);
		end
	end
	@(negedge clk); key1_n = 1'b1;
	repeat(40) @(posedge clk);
	$display("   threshold %0d -> %0d", t0, threshold);
	chk(threshold == t0, "jitter ignored (<debounce time)");

	//---- T5 按住 K1 连减 + 下限钳位 ----
	$display("");
	$display("[T5] hold K1 -> repeat down until 0");
	k1_press(3000);		//500ms后每80ms一次, 足够减到0
	$display("   threshold after hold = %0d", threshold);
	chk(threshold == 11'd0, "hold K1 reaches 0");
	k1_press(500);		//继续按住, 不应低于0
	chk(threshold == 11'd0, "threshold clamped at 0");

	//---- T6 按住 K2 连加 + 上限钳位 ----
	$display("");
	$display("[T6] hold K2 -> repeat up until 2040");
	k2_press(21000);
	$display("   threshold after hold = %0d", threshold);
	chk(threshold == 11'd2040, "hold K2 reaches 2040");
	k2_press(500);
	chk(threshold == 11'd2040, "threshold clamped at 2040");

	//---- T7 组合键切模式 ----
	$display("");
	$display("[T7] K1+K2 combo -> mode cycle 2->0->1->2 (threshold unchanged)");
	t0 = threshold;
	combo_press(100);
	$display("   mode = %0d", mode);
	chk(mode == 2'd0, "combo 1 : mode 2 -> 0 (raw)");
	combo_press(100);
	$display("   mode = %0d", mode);
	chk(mode == 2'd1, "combo 2 : mode 0 -> 1 (strength)");
	combo_press(100);
	$display("   mode = %0d", mode);
	chk(mode == 2'd2, "combo 3 : mode 1 -> 2 (binary)");
	chk(threshold == t0, "threshold not changed by combo");

	//---- T8 cfg_toggle ----
	$display("");
	$display("[T8] cfg_toggle flips after any config change");
	begin : tog
		reg tog_before;
		tog_before = cfg_toggle;
		k1_press(60);		//改阈值(此时为2040, 用K1减才有效)
		repeat(10) @(posedge clk);
		$display("   cfg_toggle %0d -> %0d", tog_before, cfg_toggle);
		chk(cfg_toggle != tog_before, "cfg_toggle toggled after change");
	end

	//---- 总结 ----
	$display("");
	$display("=========================================================");
	if(errors == 0) $display(" RESULT: ALL CHECKS PASSED (0 error)");
	else            $display(" RESULT: %0d CHECK(S) FAILED", errors);
	$display("=========================================================");
	$finish;
end

initial begin
	#5_000_000;
	$display(" TIMEOUT!");
	$finish;
end

endmodule
