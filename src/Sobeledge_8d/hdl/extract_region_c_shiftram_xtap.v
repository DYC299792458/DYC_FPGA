//======================================================================
//  extract_region_c_shiftram_xtap.v   行缓存 (抽头延迟线), Efinity 可综合
//
//  用于替代 Altera 的 altshift_taps(linebuffers), 端口与行为保持不变:
//    taps 第 t 个抽头 = shiftin 延迟 (t+1)*DISTANCE 个 clken 节拍后的数据
//
//  [dyc 优化] 原实现是 TAPS_NUM*DISTANCE 级移位寄存器链,
//    一个 1280 宽、8bit、2 抽头的行缓存要吃掉 2*1280*8 = 20480 个触发器,
//    中值 + Sobel 两个窗口就要 4 万个, 再加 Canny 的 4 个窗口 Ti60 根本放不下。
//    现改为"每个抽头一块 RAM 的环形缓冲", 综合成块 RAM(BRAM),
//    触发器只剩地址计数器和输出寄存器, 时序行为与原移位链逐拍一致
//    (sim/tb_linebuf_equiv.v 用随机 clken + 随机数据逐拍对比验证)。
//
//  原理 (每个抽头一块深度 = DISTANCE 的 RAM, 写指针 wptr 循环):
//    clken 时: mem[wptr] <= din;  q <= mem[wptr+1];  wptr++
//    wptr+1 处的数据恰好是 DISTANCE-1 个节拍前写入的, 读出后再打一拍,
//    q 就等于"din 延迟 DISTANCE 拍", 与移位链 shift_ram[DISTANCE-1] 完全相同。
//    读写地址永远不同, 不依赖 RAM 的读写冲突模式(read-first/write-first)。
//    第 t+1 个抽头的 RAM 输入接第 t 个抽头的输出, 级联即可。
//======================================================================
module extract_region_c_shiftram_xtap #(
	parameter DISTANCE = 1280,   // 抽头间间距 (>=2)
	parameter DWIDTH   = 8,      // 数据位宽
	parameter TAPS_NUM = 2       // 抽头数
)(
	input	                     clken,
	input	                     clock,
	input	[DWIDTH-1:0]	         shiftin,
	output	[DWIDTH-1:0]	         shiftout,
	output	[TAPS_NUM*DWIDTH-1:0] taps
);

	function integer clog2;
		input integer v;
		integer n;
		begin
			n = 0;
			while ((1 << n) < v) n = n + 1;
			clog2 = (n == 0) ? 1 : n;
		end
	endfunction

	localparam AW = clog2(DISTANCE);

	// 写指针 / 读指针(=写指针+1), 所有抽头共用
	reg [AW-1:0] wptr = {AW{1'b0}};
	reg [AW-1:0] rptr = (DISTANCE > 1) ? 1 : 0;
	always @(posedge clock) begin
		if (clken) begin
			wptr <= (wptr == DISTANCE-1) ? {AW{1'b0}} : wptr + 1'b1;
			rptr <= (rptr == DISTANCE-1) ? {AW{1'b0}} : rptr + 1'b1;
		end
	end

	genvar t;
	generate
		for (t = 0; t < TAPS_NUM; t = t + 1) begin : gen_tap
			(* syn_ramstyle = "block_ram" *)
			reg [DWIDTH-1:0] mem [0:DISTANCE-1];
			reg [DWIDTH-1:0] q = {DWIDTH{1'b0}};
			wire [DWIDTH-1:0] din;

			integer k;
			initial begin
				for (k = 0; k < DISTANCE; k = k + 1)
					mem[k] = {DWIDTH{1'b0}};
			end

			if (t == 0) begin : g_first
				assign din = shiftin;
			end
			else begin : g_cascade
				assign din = gen_tap[t-1].q;
			end

			always @(posedge clock) begin
				if (clken) begin
					mem[wptr] <= din;
					q         <= mem[rptr];
				end
			end

			assign taps[(t+1)*DWIDTH-1 -: DWIDTH] = q;
		end
	endgenerate

	assign shiftout = taps[TAPS_NUM*DWIDTH-1 -: DWIDTH];

endmodule
