//======================================================================
//  canny_edge.v   Canny 边缘检测 (赛题四 高阶挑战4)
//
//  全流水线, 每个时钟处理一个像素, 4 个 3x3 窗口级联(都复用 Sobeledge_matrix_3x3):
//
//    gray --> [1 高斯滤波] --> [2 梯度幅值+方向] --> [3 非极大值抑制+双阈值] --> [4 滞后边缘连接] --> edge
//
//   1 高斯滤波   : 3x3 核 [1 2 1; 2 4 2; 1 2 1]/16, 全部移位加法, 带四舍五入
//   2 梯度       : Sobel Gx/Gy, 幅值 |Gx|+|Gy| (与基础要求一致, 免开方)
//                  方向量化成 4 个方向(0/45/90/135 度), 用 tan22.5≈106/256 比较:
//                     |Gy|*256 <= |Gx|*106  -> 0   水平梯度, 比较左右邻点
//                     |Gx|*256 <  |Gy|*106  -> 2   垂直梯度, 比较上下邻点
//                     否则 Gx、Gy 同号      -> 1   比较左上/右下 (图像 y 轴向下)
//                          Gx、Gy 异号      -> 3   比较右上/左下
//                  乘 106 用 (a<<6)+(a<<5)+(a<<3)+(a<<1) 实现, 不占 DSP
//   3 NMS+双阈值 : 沿梯度方向只保留局部极大值, 边缘细化成单像素宽;
//                  极大值点  幅值>=th_high 判"强边缘", >=th_low 判"弱边缘"
//                  (平台/相等时一侧用 > 一侧用 >=, 避免两个相等像素都被保留成双线)
//   4 滞后连接   : 强边缘直接输出; 弱边缘只有 8 邻域内存在强边缘才输出。
//                  标准 Canny 的滞后连接是连通域跟踪(需要多遍), 这里用单遍 8 邻域近似,
//                  不需要帧缓存, 仍是行流水线。
//
//  side_in/side_out: 旁路 1bit, 与像素一起穿过 4 级流水线, 保证输出时与 edge 逐像素对齐。
//                    顶层用它携带 Sobel 二值结果, 这样 Sobel/Canny 可以按键实时切换, 无需另做延迟对齐。
//
//  时序/对齐: 每级窗口的帧标记与数据严格同步(见 Sobeledge_matrix_3x3 说明), 每帧输出像素数
//             恒等于 WIDTH*HEIGHT; 每级输出相对输入平移 1 行 1 列, 4 级共平移 4 行 4 列。
//  仿真: sim/tb_canny_edge.v + sim/canny_model.py 逐像素对比金标准模型(含 valid 随机断续)
//======================================================================
module canny_edge #(
	parameter WIDTH  = 1280,
	parameter HEIGHT = 720
)(
	input			clk,
	input			rst_n,
	input	[10:0]	th_high,		//强边缘阈值 (|Gx|+|Gy| 尺度, 0~2040)
	input	[10:0]	th_low,			//弱边缘阈值 (一般取 th_high/2)
	//sink
	input			sink_sop,
	input			sink_eop,
	input			sink_valid,
	input	[7:0]	sink_data,
	input			side_in,
	//source
	output			source_sop,
	output			source_eop,
	output			source_valid,
	output	reg		source_edge,	//Canny 二值边缘
	output	reg		side_out		//与 source_edge 对齐的旁路位
);

//======================================================================
// 第1级: 高斯滤波
//======================================================================
wire [8:0]	g1_p11,g1_p12,g1_p13,g1_p21,g1_p22,g1_p23,g1_p31,g1_p32,g1_p33;
wire		g1_sop,g1_eop,g1_valid;
Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DWIDTH(9)) u_win_gauss(
	.clk(clk),.rst_n(rst_n),
	.sink_sop(sink_sop),.sink_eop(sink_eop),.sink_valid(sink_valid),.sink_data({side_in,sink_data}),
	.matrix_sop(g1_sop),.matrix_eop(g1_eop),.matrix_valid(g1_valid),
	.matrix_p11(g1_p11),.matrix_p12(g1_p12),.matrix_p13(g1_p13),
	.matrix_p21(g1_p21),.matrix_p22(g1_p22),.matrix_p23(g1_p23),
	.matrix_p31(g1_p31),.matrix_p32(g1_p32),.matrix_p33(g1_p33));

reg  [9:0]	gs_corner, gs_edge;		//四角和 / 四边和
reg  [7:0]	gs_center;
reg  [1:0]	gs_side;
reg  [7:0]	gauss;
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		gs_corner<=10'd0; gs_edge<=10'd0; gs_center<=8'd0; gs_side<=2'd0; gauss<=8'd0;
	end
	else begin
		gs_corner <= g1_p11[7:0] + g1_p13[7:0] + g1_p31[7:0] + g1_p33[7:0];
		gs_edge   <= g1_p12[7:0] + g1_p21[7:0] + g1_p23[7:0] + g1_p32[7:0];
		gs_center <= g1_p22[7:0];
		gs_side   <= {gs_side[0], g1_p22[8]};
		// (corner + 2*edge + 4*center + 8) >> 4, 最大 (1020+2040+1020+8)>>4 = 255, 不会溢出
		gauss     <= ({2'b0,gs_corner} + {1'b0,gs_edge,1'b0} + {2'b0,gs_center,2'b0} + 12'd8) >> 4;
	end
end
wire g1o_sop,g1o_eop,g1o_valid;
shift_regs #(.DWIDTH(3),.DELAY_DUTY(2)) u_dly_gauss(
	.clk(clk),.rst_n(rst_n),
	.idata({g1_sop,g1_eop,g1_valid}),.odata({g1o_sop,g1o_eop,g1o_valid}));

//======================================================================
// 第2级: Sobel 梯度幅值 + 方向
//======================================================================
wire [8:0]	g2_p11,g2_p12,g2_p13,g2_p21,g2_p22,g2_p23,g2_p31,g2_p32,g2_p33;
wire		g2_sop,g2_eop,g2_valid;
Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DWIDTH(9)) u_win_grad(
	.clk(clk),.rst_n(rst_n),
	.sink_sop(g1o_sop),.sink_eop(g1o_eop),.sink_valid(g1o_valid),.sink_data({gs_side[1],gauss}),
	.matrix_sop(g2_sop),.matrix_eop(g2_eop),.matrix_valid(g2_valid),
	.matrix_p11(g2_p11),.matrix_p12(g2_p12),.matrix_p13(g2_p13),
	.matrix_p21(g2_p21),.matrix_p22(g2_p22),.matrix_p23(g2_p23),
	.matrix_p31(g2_p31),.matrix_p32(g2_p32),.matrix_p33(g2_p33));

// a: 正负部分和
reg  [9:0]	gx_p,gx_n,gy_p,gy_n;
// b: 绝对值与符号
reg  [9:0]	ax,ay;
reg			sx,sy;			//1 = 该分量 >= 0
// c: 幅值与方向
reg  [10:0]	mag;
reg  [1:0]	dir;
reg  [2:0]	gr_side;
wire [17:0]	ax_k  = {ax,6'd0} + {1'b0,ax,5'd0} + {3'b0,ax,3'd0} + {5'b0,ax,1'b0};	// ax*106
wire [17:0]	ay_k  = {ay,6'd0} + {1'b0,ay,5'd0} + {3'b0,ay,3'd0} + {5'b0,ay,1'b0};	// ay*106
wire [17:0]	ax_256 = {ax,8'd0};
wire [17:0]	ay_256 = {ay,8'd0};
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		gx_p<=10'd0; gx_n<=10'd0; gy_p<=10'd0; gy_n<=10'd0;
		ax<=10'd0; ay<=10'd0; sx<=1'b0; sy<=1'b0;
		mag<=11'd0; dir<=2'd0; gr_side<=3'd0;
	end
	else begin
		gx_p <= g2_p13[7:0] + {1'b0,g2_p23[7:0],1'b0} + g2_p33[7:0];
		gx_n <= g2_p11[7:0] + {1'b0,g2_p21[7:0],1'b0} + g2_p31[7:0];
		gy_p <= g2_p31[7:0] + {1'b0,g2_p32[7:0],1'b0} + g2_p33[7:0];
		gy_n <= g2_p11[7:0] + {1'b0,g2_p12[7:0],1'b0} + g2_p13[7:0];

		ax <= (gx_p>=gx_n) ? gx_p-gx_n : gx_n-gx_p;
		ay <= (gy_p>=gy_n) ? gy_p-gy_n : gy_n-gy_p;
		sx <= (gx_p>=gx_n);
		sy <= (gy_p>=gy_n);

		mag <= {1'b0,ax} + {1'b0,ay};
		if(ay_256 <= ax_k)       dir <= 2'd0;
		else if(ax_256 < ay_k)   dir <= 2'd2;
		else if(sx == sy)        dir <= 2'd1;
		else                     dir <= 2'd3;

		gr_side <= {gr_side[1:0], g2_p22[8]};
	end
end
wire g2o_sop,g2o_eop,g2o_valid;
shift_regs #(.DWIDTH(3),.DELAY_DUTY(3)) u_dly_grad(
	.clk(clk),.rst_n(rst_n),
	.idata({g2_sop,g2_eop,g2_valid}),.odata({g2o_sop,g2o_eop,g2o_valid}));

//======================================================================
// 第3级: 非极大值抑制 + 双阈值
//   窗口数据 14bit = {side, dir[1:0], mag[10:0]}
//======================================================================
wire [13:0]	g3_p11,g3_p12,g3_p13,g3_p21,g3_p22,g3_p23,g3_p31,g3_p32,g3_p33;
wire		g3_sop,g3_eop,g3_valid;
Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DWIDTH(14)) u_win_nms(
	.clk(clk),.rst_n(rst_n),
	.sink_sop(g2o_sop),.sink_eop(g2o_eop),.sink_valid(g2o_valid),.sink_data({gr_side[2],dir,mag}),
	.matrix_sop(g3_sop),.matrix_eop(g3_eop),.matrix_valid(g3_valid),
	.matrix_p11(g3_p11),.matrix_p12(g3_p12),.matrix_p13(g3_p13),
	.matrix_p21(g3_p21),.matrix_p22(g3_p22),.matrix_p23(g3_p23),
	.matrix_p31(g3_p31),.matrix_p32(g3_p32),.matrix_p33(g3_p33));

// 沿梯度方向取两个邻点: nA 在"左/上"一侧(用 >), nB 在"右/下"一侧(用 >=)
reg [10:0] nA_w, nB_w;
always@(*)begin
	case(g3_p22[12:11])
		2'd0:    begin nA_w = g3_p21[10:0]; nB_w = g3_p23[10:0]; end	//水平梯度: 左/右
		2'd1:    begin nA_w = g3_p11[10:0]; nB_w = g3_p33[10:0]; end	//左上/右下
		2'd2:    begin nA_w = g3_p12[10:0]; nB_w = g3_p32[10:0]; end	//垂直梯度: 上/下
		default: begin nA_w = g3_p13[10:0]; nB_w = g3_p31[10:0]; end	//右上/左下
	endcase
end

reg  [10:0]	nm_mag;
reg			nm_keep;
reg  [1:0]	nm_side;
reg  [1:0]	nm_cls;			//0=非边缘 1=弱边缘 2=强边缘
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		nm_mag<=11'd0; nm_keep<=1'b0; nm_side<=2'd0; nm_cls<=2'd0;
	end
	else begin
		nm_mag  <= g3_p22[10:0];
		nm_keep <= (g3_p22[10:0] > nA_w) && (g3_p22[10:0] >= nB_w);	//局部极大(隐含 mag>0)
		nm_side <= {nm_side[0], g3_p22[13]};
		if(nm_keep && (nm_mag >= th_high))     nm_cls <= 2'd2;
		else if(nm_keep && (nm_mag >= th_low)) nm_cls <= 2'd1;
		else                                   nm_cls <= 2'd0;
	end
end
wire g3o_sop,g3o_eop,g3o_valid;
shift_regs #(.DWIDTH(3),.DELAY_DUTY(2)) u_dly_nms(
	.clk(clk),.rst_n(rst_n),
	.idata({g3_sop,g3_eop,g3_valid}),.odata({g3o_sop,g3o_eop,g3o_valid}));

//======================================================================
// 第4级: 滞后边缘连接(单遍 8 邻域)
//   窗口数据 3bit = {side, cls[1:0]}
//======================================================================
wire [2:0]	g4_p11,g4_p12,g4_p13,g4_p21,g4_p22,g4_p23,g4_p31,g4_p32,g4_p33;
wire		g4_sop,g4_eop,g4_valid;
Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.DWIDTH(3)) u_win_hyst(
	.clk(clk),.rst_n(rst_n),
	.sink_sop(g3o_sop),.sink_eop(g3o_eop),.sink_valid(g3o_valid),.sink_data({nm_side[1],nm_cls}),
	.matrix_sop(g4_sop),.matrix_eop(g4_eop),.matrix_valid(g4_valid),
	.matrix_p11(g4_p11),.matrix_p12(g4_p12),.matrix_p13(g4_p13),
	.matrix_p21(g4_p21),.matrix_p22(g4_p22),.matrix_p23(g4_p23),
	.matrix_p31(g4_p31),.matrix_p32(g4_p32),.matrix_p33(g4_p33));

wire strong_nb = (g4_p11[1:0]==2'd2) | (g4_p12[1:0]==2'd2) | (g4_p13[1:0]==2'd2) |
                 (g4_p21[1:0]==2'd2) |                        (g4_p23[1:0]==2'd2) |
                 (g4_p31[1:0]==2'd2) | (g4_p32[1:0]==2'd2) | (g4_p33[1:0]==2'd2);
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		source_edge <= 1'b0; side_out <= 1'b0;
	end
	else begin
		source_edge <= (g4_p22[1:0]==2'd2) | ((g4_p22[1:0]==2'd1) & strong_nb);
		side_out    <= g4_p22[2];
	end
end
shift_regs #(.DWIDTH(3),.DELAY_DUTY(1)) u_dly_hyst(
	.clk(clk),.rst_n(rst_n),
	.idata({g4_sop,g4_eop,g4_valid}),.odata({source_sop,source_eop,source_valid}));

endmodule
