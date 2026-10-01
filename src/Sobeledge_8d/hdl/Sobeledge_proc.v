// 实现8位灰度图像数据的Sobel边缘检测
// WIDTH、HEIGHT为图像参数
// SOBEL_MODE_4DIR: 0 = G = |Gx| + |Gy|   (赛题要求, 默认)
//                  1 = G = max(|Gx|,|Gy|,|G45|,|G135|) (旧方案, 可选保留)
module Sobeledge_8d_proc#(
parameter WIDTH=480,
parameter HEIGHT=320,
parameter SOBEL_MODE_4DIR = 0
)(
	input			    clk,
	input			    rst_n,
	input      [1:0]  mode,			//输出模式: 0=原图直通 1=边缘强度 2=二值化边缘
	input      [10:0] threshold,	//二值化阈值, 范围 0~2040 (|Gx|+|Gy| 最大 2040)
	//sink 
	input           sink_sop,
	input           sink_eop,
	input           sink_valid,
	input 	[7:0]  sink_data,			//原始数据
	//source
	output          source_sop,
	output          source_eop,
	output          source_valid,
	output  [7:0]   source_data			//Sobel边缘检测结果
);

// 3x3像素矩阵
wire [7:0]		matrix_p11,matrix_p12,matrix_p13;//窗口第一行数据
wire [7:0]		matrix_p21,matrix_p22,matrix_p23;//窗口第二行数据
wire [7:0]		matrix_p31,matrix_p32,matrix_p33;//窗口第三行数据
wire	matrix_sop,matrix_eop,matrix_valid;
wire [7:0]     matrix_p22_dly;//对matrix_p22延迟4个时钟，和Gmax对齐
Sobeledge_matrix_3x3 #(
.WIDTH(WIDTH),
.HEIGHT(HEIGHT)
)u_Sobeledge_matrix_3x3(
	.clk(clk),
	.rst_n(rst_n),
	
	.sink_sop(sink_sop),
	.sink_eop(sink_eop),
	.sink_valid(sink_valid),
	.sink_data(sink_data),
	
	.matrix_sop(matrix_sop),
	.matrix_eop(matrix_eop),
	.matrix_valid(matrix_valid),
	.matrix_p11(matrix_p11),
	.matrix_p12(matrix_p12),
	.matrix_p13(matrix_p13),
	.matrix_p21(matrix_p21),
	.matrix_p22(matrix_p22),
	.matrix_p23(matrix_p23),
	.matrix_p31(matrix_p31),
	.matrix_p32(matrix_p32),
	.matrix_p33(matrix_p33)
);
// Step 1 计算卷积并求绝对值
// Sobel算子
//																
// 		[-1	-2		-1]			[-2	-1		 0]		[-1		0	   +1]		 [0	+1		+2]
// 0°	   [0		0		 0]	45°	[-1	0		+1] 90°  [-2		0	   +2]  135° [-1	0		+1]
//		   [+1	+2		+1]			[0		+1		+2]		[-1		0	   +1]		 [-2	-1		 0]

// 		[+1	+2		+1]			[+2	+1		 0]		[+1		0	   -1]		 [0	-1		-2]
// 180°	[0		0		 0]	225°  [+1	0		-1] 270° [+2		0	   -2]  315° [+1	0		-1]
//		   [-1	-2		-1]			[0		-1		-2]		[+1		0	   -1]		 [+2	+1		 0]
//通过观察可以发现，180°和0°的算子是对称的，二者结果的绝对值是相等的，同理225°、270°、315°
//因此只需要计算4组算子的卷积即可
reg	[9:0]	G0_p,G0_n,G0;
reg	[9:0]	G45_p,G45_n,G45;
reg	[9:0]	G90_p,G90_n,G90;
reg	[9:0]	G135_p,G135_n,G135;
//0°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G0_p <= 'd0;
		G0_n <= 'd0;
		G0   <= 'd0;
	end
	else begin
		G0_p <= matrix_p11 + ({1'b0,matrix_p12,1'b0}) + matrix_p13;
		G0_n <= matrix_p31 + ({1'b0,matrix_p32,1'b0}) + matrix_p33;
		G0   <= (G0_p>=G0_n) ? G0_p-G0_n:G0_n-G0_p;
	end	
end
//45°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G45_p <= 'd0;
		G45_n <= 'd0;
		G45   <= 'd0;
	end
	else begin
		G45_p <= matrix_p23 + ({1'b0,matrix_p33,1'b0}) + matrix_p32;
		G45_n <= matrix_p12 + ({1'b0,matrix_p11,1'b0}) + matrix_p21;	
		G45   <= (G45_p>=G45_n) ? G45_p-G45_n:G45_n-G45_p;
	end	
end
//90°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G90_p <= 'd0;
		G90_n <= 'd0;
		G90   <= 'd0;
	end
	else begin
		G90_p <= matrix_p13 + ({1'b0,matrix_p23,1'b0}) + matrix_p33;
		G90_n <= matrix_p11 + ({1'b0,matrix_p21,1'b0}) + matrix_p31;
		G90   <= (G90_p>=G90_n) ? G90_p-G90_n:G90_n-G90_p;
	end	
end
//135°
always@(posedge clk or negedge rst_n)
begin
	if(!rst_n)
	begin
		G135_p <= 'd0;
		G135_n <= 'd0;
		G135   <= 'd0;
	end
	else begin
		G135_p <= matrix_p12 + ({1'b0,matrix_p13,1'b0}) + matrix_p23;
		G135_n <= matrix_p21 + ({1'b0,matrix_p31,1'b0}) + matrix_p32;
		G135   <= (G135_p>=G135_n) ? G135_p-G135_n:G135_n-G135_p;
	end	
end
// Step 2 找最大的G
// 注: G0 分支 = |(p11+2p12+p13)-(p31+2p32+p33)| = |Gy|  垂直方向梯度
//     G90分支 = |(p13+2p23+p33)-(p11+2p21+p31)| = |Gx|  水平方向梯度
// 赛题要求(默认): G = |Gx| + |Gy|, 最大 2040, 所以用 11bit
// 可选(SOBEL_MODE_4DIR=1): G = max(|Gx|,|Gy|,|G45|,|G135|) 保留原四方向方案
wire [10:0] Gsum = {1'b0,G90} + {1'b0,G0};
// Gmax = max(G0,G45,G90,G135)
reg [9:0] Gmax_0_45,Gmax_90_135;
reg [10:0] Gedge;//边缘强度, 第4级寄存, 与 matrix_p22_dly 对齐
always@(posedge clk or negedge rst_n)
begin
if(!rst_n)
    begin
    Gmax_0_45   <= 'd0;
    Gmax_90_135 <= 'd0;
    end
else
    begin
    Gmax_0_45   <= (G0 >= G45)?G0:G45;
    Gmax_90_135 <= (G90 >= G135)?G90:G135;
    end
end
always@(posedge clk or negedge rst_n)
begin
if(!rst_n)
	Gedge <= 'd0;
else
	Gedge <= (SOBEL_MODE_4DIR != 0) ? ((Gmax_0_45 >= Gmax_90_135)?{1'b0,Gmax_0_45}:{1'b0,Gmax_90_135}) : Gsum;
end
// Step 3 输出选择: 原图 / 边缘强度 / 二值化边缘
// mode=1: 边缘强度 = Gedge/8 (2040>>3 = 255, 刚好铺满 8bit)
// mode=2: 二值化边缘, Gedge >= threshold 输出 255, 否则 0 (阈值 0~2040 可配置)
// mode=0: 原图直通(matrix_p22_dly 与 Gedge 同延迟)
assign 	source_data = (mode==2'd1) ? Gedge[10:3] :
                       (mode==2'd2) ? ((Gedge >= threshold)?8'hff:8'h00) :
                                      matrix_p22_dly;
shift_regs#(
.DWIDTH(3),     
.DELAY_DUTY(4)
)u_shift_regs_0(
    .clk(clk),
	.rst_n(rst_n),
	.idata({matrix_sop,matrix_valid,matrix_eop}),
	.odata({source_sop,source_valid,source_eop})
);
shift_regs#(
.DWIDTH(8),     
.DELAY_DUTY(4)
)u_shift_regs_1(
    .clk(clk),
	.rst_n(rst_n),
	.idata(matrix_p22),
	.odata(matrix_p22_dly)
);
endmodule
