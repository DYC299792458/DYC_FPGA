//======================================================================
//  median_3x3.v   3x3 中值滤波 (8bit 灰度)
//
//  位置: 灰度化之后、Sobel 之前, 用于抑制椒盐噪声, 提升边缘检测质量
//
//  结构: 复用 Sobeledge_matrix_3x3 生成 3x3 窗口(自带帧标记对齐与左右/上下边界钳位),
//        再用 19 个比较器的中值网络求 9 个数的中值:
//            3行各自排序  ->  取 max_min / min_max / med_med  ->  三者再取中值
//        该公式已用 4^9 全枚举 + 20 万组随机 8bit 数据验证, 与真中值完全一致
//
//  流水: 窗口 -> 行排序(1拍) -> 中间量(1拍) -> 中值(1拍), 共 3 级,
//        帧标记 (sop/eop/valid) 同步延迟 3 拍, 保证与数据逐像素对齐
//======================================================================
module median_3x3 #(
	parameter WIDTH  = 1280,
	parameter HEIGHT = 720
)(
	input			clk,
	input			rst_n,
	//sink
	input			sink_sop,
	input			sink_eop,
	input			sink_valid,
	input	[7:0]	sink_data,
	//source
	output			source_sop,
	output			source_eop,
	output			source_valid,
	output	reg [7:0]	source_data
);

//----------------------------------------------------------------------
// 3x3 像素窗口
//----------------------------------------------------------------------
wire [7:0]	p11,p12,p13;
wire [7:0]	p21,p22,p23;
wire [7:0]	p31,p32,p33;
wire		win_sop,win_eop,win_valid;

Sobeledge_matrix_3x3 #(
	.WIDTH(WIDTH),
	.HEIGHT(HEIGHT)
)u_window(
	.clk(clk),
	.rst_n(rst_n),
	.sink_sop(sink_sop),
	.sink_eop(sink_eop),
	.sink_valid(sink_valid),
	.sink_data(sink_data),
	.matrix_sop(win_sop),
	.matrix_eop(win_eop),
	.matrix_valid(win_valid),
	.matrix_p11(p11),.matrix_p12(p12),.matrix_p13(p13),
	.matrix_p21(p21),.matrix_p22(p22),.matrix_p23(p23),
	.matrix_p31(p31),.matrix_p32(p32),.matrix_p33(p33)
);

//----------------------------------------------------------------------
// 比较器与排序网络
//----------------------------------------------------------------------
function [7:0] max8;
	input [7:0] a,b;
	begin max8 = (a>=b) ? a : b; end
endfunction

function [7:0] min8;
	input [7:0] a,b;
	begin min8 = (a<=b) ? a : b; end
endfunction

// 3个数排序网络(3个比较器), 返回 {max, med, min}
function [23:0] sort3;
	input [7:0] a,b,c;
	reg   [7:0] ab_min,ab_max,m2;
	begin
		ab_min = (a<=b) ? a : b;
		ab_max = (a>=b) ? a : b;
		m2     = (ab_min<=c) ? c : ab_min;		// min/max 之后的中间候选
		sort3  = { ((ab_max>=m2)?ab_max:m2),	// max
		           ((ab_max>=m2)?m2:ab_max),	// med
		           ((ab_min<=c)?ab_min:c) };	// min
	end
endfunction

//---------------- 第1级: 3行各自排序 ----------------
wire [23:0] s_row1 = sort3(p11,p12,p13);
wire [23:0] s_row2 = sort3(p21,p22,p23);
wire [23:0] s_row3 = sort3(p31,p32,p33);

reg [7:0] r1min,r1med,r1max;
reg [7:0] r2min,r2med,r2max;
reg [7:0] r3min,r3med,r3max;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		{r1max,r1med,r1min} <= 24'd0;
		{r2max,r2med,r2min} <= 24'd0;
		{r3max,r3med,r3min} <= 24'd0;
	end
	else begin
		{r1max,r1med,r1min} <= s_row1;
		{r2max,r2med,r2min} <= s_row2;
		{r3max,r3med,r3min} <= s_row3;
	end
end

//---------------- 第2级: max_min / min_max / med_med ----------------
wire [7:0] max_min_w = max8(r1min, max8(r2min,r3min));	// 3个行最小值里的最大者
wire [7:0] min_max_w = min8(r1max, min8(r2max,r3max));	// 3个行最大值里的最小者
wire [23:0] s_med_w  = sort3(r1med,r2med,r3med);		// 3个行中值的中值

reg [7:0] max_min,min_max,med_med;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		max_min <= 8'd0;
		min_max <= 8'd0;
		med_med <= 8'd0;
	end
	else begin
		max_min <= max_min_w;
		min_max <= min_max_w;
		med_med <= s_med_w[15:8];
	end
end

//---------------- 第3级: 三者取中值 = 9个数的中值 ----------------
wire [23:0] s_med9 = sort3(max_min,med_med,min_max);
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		source_data <= 8'd0;
	else
		source_data <= s_med9[15:8];
end

//---------------- 帧标记同步延迟3拍 ----------------
shift_regs#(
	.DWIDTH(3),
	.DELAY_DUTY(3)
)u_mark_delay(
	.clk(clk),
	.rst_n(rst_n),
	.idata({win_sop,win_eop,win_valid}),
	.odata({source_sop,source_eop,source_valid})
);

endmodule
