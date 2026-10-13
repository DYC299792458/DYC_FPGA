//======================================================================
//  tb_display_overlay.v   显示端叠加层仿真: lcd_driver 真实时序 + FWFT 读FIFO模型
//
//  模拟 example_top.v 的显示通路: DDR 读回的 16bit 打包像素 -> 分屏(左彩色+红边/右二值)
//  -> shape_overlay(包围盒 + 中文提示) -> lcd_driver, 把 lcd_en 有效期间的 RGB 写成 PPM。
//  帧缓存内容: 背景 + 一个黄色圆(带红色边缘标志), 识别结果给 圆形 + 包围盒。
//  自动检查: 包围盒四条边恰好落在给定坐标上 (验证 lcd_xpos/ypos 与数据同拍, X_SHIFT=0 正确)。
//  python3 sim/ppm2png.py 可把 overlay.ppm 转成 PNG 目检。
//======================================================================
`timescale 1ns/1ps
`include "lcd_para.v"
module tb_display_overlay;
	localparam W = 1280, H = 720;
	localparam BX0 = 500, BX1 = 780, BY0 = 220, BY1 = 500;	// 包围盒(图像坐标)

	reg clk = 0, rst_n = 0;
	always #5 clk = ~clk;

	wire        lcd_request, lcd_hs, lcd_vs, lcd_de;
	wire [11:0] lcd_xpos, lcd_ypos;
	wire [7:0]  r, g, b, r2, g2, b2;

	// ---- FWFT 读 FIFO 模型: rdata 一直给出队头, rd_en 时出队 ----
	reg  [31:0] rd_idx = 0;
	function [15:0] fb_pix;		// 帧缓存第 i 个像素: {RGB555, 边缘}
		input integer i;
		integer x, y, dx, dy, d2;
		begin
			x = i % W; y = i / W; dx = x - 640; dy = y - 360; d2 = dx*dx + dy*dy;
			if (d2 <= 130*130 && d2 >= 127*127) fb_pix = {5'd31,5'd31,5'd0,1'b1};	// 轮廓: 边缘标志
			else if (d2 < 127*127)              fb_pix = {5'd31,5'd31,5'd0,1'b0};	// 黄色
			else                                fb_pix = {5'd5,5'd10,5'd8,1'b0};	// 背景
		end
	endfunction
	wire [15:0] lcd_data = fb_pix(rd_idx);
	always @(posedge clk) begin
		if (lcd_vs == 1'b0) rd_idx <= 0;			// 场同步: FIFO 复位
		else if (lcd_request) rd_idx <= rd_idx + 1;
	end

	// ---- 与 example_top.v 相同的显示通路 ----
	wire [23:0] lcd_rgb888 = {lcd_data[15:11], lcd_data[13:11], lcd_data[10:6], lcd_data[8:6], lcd_data[5:1], lcd_data[3:1]};
	wire [23:0] lcd_left   = lcd_data[0] ? 24'hFF0000 : lcd_rgb888;
	wire [23:0] lcd_right  = lcd_data[0] ? 24'hFFFFFF : 24'h000000;
	wire [23:0] lcd_pixel  = (lcd_xpos < 12'd640) ? lcd_left : lcd_right;
	wire [23:0] lcd_ov;
	shape_overlay #(.S(2), .X_SHIFT(0), .LABEL_X(24), .LABEL_Y(24), .BOX_THICK(2)) u_ov (
		.clk(clk), .rst_n(rst_n), .de(lcd_request), .xpos(lcd_xpos), .ypos(lcd_ypos), .base(lcd_pixel),
		.shape(2'd1), .bx0(BX0), .bx1(BX1), .by0(BY0), .by1(BY1), .valid(1'b1), .rgb(lcd_ov));
	lcd_driver u_lcd (
		.clk(clk), .rst_n(rst_n), .lcd_dclk(), .lcd_blank(), .lcd_sync(),
		.lcd_request(lcd_request), .lcd_hs(lcd_hs), .lcd_vs(lcd_vs), .lcd_en(lcd_de),
		.lcd_rgb({r2,g2,b2,r,g,b}), .lcd_xpos(lcd_xpos), .lcd_ypos(lcd_ypos), .lcd_data({lcd_ov, lcd_ov}));

	// ---- 抓一帧 ----
	integer fd, px = 0, py = 0, frames = 0, errors = 0;
	reg prev_vs = 1, capturing = 0, prev_de = 0;
	always @(posedge clk) begin
		prev_vs <= lcd_vs; prev_de <= lcd_de;
		if (!prev_vs && lcd_vs) begin			// 场同步结束
			frames = frames + 1;
			if (frames == 2) begin capturing = 1; px = 0; py = 0; end
			if (frames == 3) begin capturing = 0; $fclose(fd); finish_check; end
		end
		if (capturing && lcd_de) begin
			$fwrite(fd, "%c%c%c", r, g, b);
			// 包围盒边线(颜色=圆形青色 00FFFF), 检查第 BY0+5 行的左右边线位置
			if (py == BY0 + 5) begin
				if ((px == BX0 || px == BX0+1 || px == BX1 || px == BX1-1) && {r,g,b} !== 24'h00FFFF) begin
					errors = errors + 1; $display("  [FAIL] box border missing at (%0d,%0d): %h", px, py, {r,g,b});
				end
				if ((px == BX0-1 || px == BX1+1 || px == BX0+2) && {r,g,b} === 24'h00FFFF) begin
					errors = errors + 1; $display("  [FAIL] box border off by one at (%0d,%0d)", px, py);
				end
			end
			px = px + 1;
			if (px == W) begin px = 0; py = py + 1; end
		end
	end
	task finish_check;
		begin
			$display("=====================================================");
			$display(" tb_display_overlay: captured %0d lines, %0d errors", py, errors);
			if (errors == 0 && py == H) $display(" RESULT: ALL CHECKS PASSED");
			else                        $display(" RESULT: FAILED");
			$display("=====================================================");
			$finish;
		end
	endtask
	initial begin
		fd = $fopen("overlay.ppm", "wb");
		$fwrite(fd, "P6\n%0d %0d\n255\n", W, H);
		repeat (10) @(posedge clk); rst_n = 1;
	end
endmodule
