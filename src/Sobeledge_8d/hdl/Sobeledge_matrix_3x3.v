// 实现3*3像素矩阵输出，忽略图像边界2像素宽度数据
// 根据有效数据计数生成矩阵的sop eop valid信号
module Sobeledge_matrix_3x3 #(
parameter	WIDTH=480,
parameter	HEIGHT=320
)(
	input			clk,
	input			rst_n,
	
	input			sink_sop,
	input			sink_eop,
	input			sink_valid,
	input	[7:0]	sink_data,
	
	output			matrix_sop,
	output			matrix_eop,
	output			matrix_valid,
	output	[7:0]	matrix_p11,matrix_p12,matrix_p13,
	output	[7:0]	matrix_p21,matrix_p22,matrix_p23,
	output	[7:0]	matrix_p31,matrix_p32,matrix_p33
);

wire [7:0]	row1_data,row2_data;
wire  matrix_en;
reg	[7:0]	row3_data;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		row3_data<='d0;
	end
	else begin
		if(sink_valid)
			row3_data<=sink_data;
		else
			row3_data<=row3_data;
	end
end
// 缓存两行，与当前输入行构成三行输出
extract_region_c_shiftram_xtap #(
	.DISTANCE(WIDTH),	// 抽头间间距 = 图像宽度(原来硬编码1280, 改成参数以便复用小尺寸仿真)
	.DWIDTH(8),			// 数据位宽
	.TAPS_NUM(2)		// 抽头数
)u_linebuffers(
	.clken(sink_valid),
	.clock(clk),
	.shiftin(row3_data),
	.shiftout(),
	.taps({row1_data,row2_data})
);

// sink_sop/sink_eop/sink_valid 延迟一个clk, 得到与3x3窗口对齐的
// matrix_sop/matrix_eop/matrix_en
// 注意: 输入是"帧级纯像素流"(只有整帧首尾有sop/eop, 中间没有行标记),
//       所以不能用自由计数器去数行, 直接透传帧标记最稳, 保证每帧输出
//       像素数严格等于输入的 WIDTH*HEIGHT, 不会丢帧首的一行。
shift_regs#(
.DWIDTH(3),     
.DELAY_DUTY(1)
)u2_shift_regs(
    .clk(clk),
	.rst_n(rst_n),
	.idata({sink_sop,sink_eop,sink_valid}),
	.odata({matrix_sop,matrix_eop,matrix_en})
);
// 帧首前2行: 行缓存的"上一行/上两行"抽头还没填满, 用当前行数据顶替,
// 否则顶部会出现一条横贯全屏的假边缘。fill_cnt 每帧清0, 数满 2*WIDTH 后停住。
reg [12:0] fill_cnt;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		fill_cnt<=13'd0;
	else if(matrix_sop)
		fill_cnt<=13'd0;
	else if(matrix_en && (fill_cnt<2*WIDTH))
		fill_cnt<=fill_cnt+1'b1;
end
wire row_clamp = (fill_cnt < 2*WIDTH);
wire [7:0] row1_win = row_clamp ? row3_data : row1_data;
wire [7:0] row2_win = row_clamp ? row3_data : row2_data;
// 列计数: xcol = 窗口"最新一列"(即 row3_data/row1_win/row2_win 所在列)的列号
// 行缓存是移位链, 换行时窗口左列会接到上一行末尾(回绕), 会造成屏幕最左侧一条竖白边,
// 所以在 xcol=0/1 时把窗口左侧列钳位到本行第0列
reg [11:0] xcol;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)
		xcol<=12'd0;
	else if(matrix_sop)
		xcol<=12'd0;
	else if(matrix_en)
		xcol<=(xcol>=WIDTH-1)?12'd0:xcol+1'b1;
end
// 生成3*3像素矩阵
reg	[7:0]	m11,m12,m13,m21,m22,m23,m31,m32,m33;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		{m11,m12,m13}<='d0;
		{m21,m22,m23}<='d0;
		{m31,m32,m33}<='d0;
	end
	else begin
		if(matrix_en)begin
			{m11,m12,m13}<={m12,m13,row1_win};
			{m21,m22,m23}<={m22,m23,row2_win};
			{m31,m32,m33}<={m32,m33,row3_data};
		end
		else begin
			{m11,m12,m13}<={m11,m12,m13};
			{m21,m22,m23}<={m21,m22,m23};
			{m31,m32,m33}<={m31,m32,m33};
		end
	end
end
// 左边界钳位: xcol=0 时窗口覆盖列 -2,-1,0, 全部用第0列; xcol=1 时左列用第0列
assign matrix_p11 = (xcol==12'd0) ? m13 : ((xcol==12'd1) ? m12 : m11);
assign matrix_p12 = (xcol==12'd0) ? m13 : m12;
assign matrix_p13 = m13;
assign matrix_p21 = (xcol==12'd0) ? m23 : ((xcol==12'd1) ? m22 : m21);
assign matrix_p22 = (xcol==12'd0) ? m23 : m22;
assign matrix_p23 = m23;
assign matrix_p31 = (xcol==12'd0) ? m33 : ((xcol==12'd1) ? m32 : m31);
assign matrix_p32 = (xcol==12'd0) ? m33 : m32;
assign matrix_p33 = m33;

// 每个有效输入像素都输出一个结果(不丢帧首一行), matrix_sop/eop 由帧标记透传得到
assign matrix_valid = matrix_en;


endmodule

