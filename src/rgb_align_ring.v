//======================================================================
//  rgb_align_ring.v   彩色原图延迟线 (按像素序号对齐), 块RAM环形缓冲
//
//  作用: 把输入端的 RGB565 原图延迟到与"边缘流水线输出"逐像素对齐,
//        再和二值边缘一起打包写 DDR (彩色叠加 / 分屏显示用)。
//
//  对齐原理 (与时钟延迟、valid 是否连续都无关):
//    * 写侧: 每个输入像素(in_valid)写一格, 帧首像素(in_sop)记下其地址 base
//    * 流水线每一级都是"一进一出", 输出帧第 n 个像素对应输入帧第 n - LAG 个像素
//      (每级 3x3 窗口序号滞后 WIDTH+1, 见 sim/tb_pipeline_latency.v、sim/tb_canny_edge.v)
//    * 读侧: 输出帧首像素(out_sop)时读地址 = base - LAG, 之后每个输出像素 +1
//    * 环深 2^AW, 地址减法自然回绕; 要求 LAG + 流水线在途像素数 < 2^AW
//
//  时序: 读 RAM 需要一拍, 所以 out_* 进来后打一拍, 在 pk_* 上与 pk_rgb 同拍输出。
//  仿真: sim/tb_edge_system.v (1280 宽整链路, valid 随机断续, 逐像素核对)
//======================================================================
module rgb_align_ring #(
	parameter        AW  = 13,			// 环深 2^AW
	parameter [31:0] LAG = 7686			// 流水线序号滞后(像素)
)(
	input				clk,
	// 写侧: 原图输入流
	input				in_valid,
	input				in_sop,
	input		[15:0]	in_rgb,
	// 读侧: 边缘流水线输出
	input				out_valid,
	input				out_sop,
	input				out_eop,
	input				out_edge,
	// 打包输出 (比 out_* 晚一拍)
	output	reg			pk_valid = 1'b0,
	output	reg			pk_sop   = 1'b0,
	output	reg			pk_eop   = 1'b0,
	output	reg			pk_edge  = 1'b0,
	output	reg	[15:0]	pk_rgb   = 16'd0
);
	reg  [15:0]   mem [0:(1<<AW)-1];
	reg  [AW-1:0] wptr = {AW{1'b0}};
	reg  [AW-1:0] base = {AW{1'b0}};
	reg  [AW-1:0] rptr = {AW{1'b0}};

	always @(posedge clk) begin
		if (in_valid) begin
			mem[wptr] <= in_rgb;
			if (in_sop) base <= wptr;
			wptr <= wptr + 1'b1;
		end
	end

	wire [AW-1:0] raddr = (out_valid && out_sop) ? (base - LAG[AW-1:0]) : rptr;

	always @(posedge clk) begin
		if (out_valid) rptr <= raddr + 1'b1;
		pk_rgb   <= mem[raddr];		// 同步读, 推断块RAM
		pk_valid <= out_valid;
		pk_sop   <= out_valid & out_sop;
		pk_eop   <= out_valid & out_eop;
		pk_edge  <= out_edge;
	end
endmodule
