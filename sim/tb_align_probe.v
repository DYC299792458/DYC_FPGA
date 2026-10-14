`timescale 1ns/1ps
//----------------------------------------------------------------------
// 默认参数(可不加 +define+ 直接编译):
//   TBP_W       图像宽度(小一点跑得快; 结论与宽度成线性)
//   TBP_H       图像高度
//   TBP_RGB_DLY 彩色延迟环的延迟量, 默认复刻 example_top 里的 7+(2W-1)
//   TBP_NO_MEDIAN 定义后旁路中值滤波(对应 example_top 里 MEDIAN_EN=0)
//----------------------------------------------------------------------
`ifndef TBP_W
 `define TBP_W 32
`endif
`ifndef TBP_H
 `define TBP_H 128
`endif

//======================================================================
//  tb_align_probe.v   彩色通道 / 边缘位 对齐探针
//
//  目的: 量出整链 "rgb2gray -> median_3x3 -> Sobeledge_8d_proc" 的两个延迟
//
//    L_gray = 输出流第 k 个有效像素对应的场景像素序号(k - L_gray)
//             => matrix_p22_dly / 原图灰度通道的滞后
//    L_bin  = 同一个输出词里的二值边缘位(sobel_bin)对应的场景像素序号
//             => example_top 里 RGB_DLY 必须等于这个值, 否则红边会相对
//                彩色原图左右漂移; 漂移量 = L_bin - RGB_DLY (正=红边偏右)
//
//  量法: 场景用"随机宽度竖条纹"(中值滤波对它不变, 且没有小周期),
//        把输出流跟输入场景做逐像素比对, 扫候选滞后量取不匹配最少的 L。
//
//  重点看这一行:
//    ALIGN2: red(d=0) matches color-edge shifted by N px
//      => 红边(边缘位)相对彩色图自身的边缘偏了 N 像素(正=偏右)
//    用 -D/+define TBP_RGB_DLY=<值> 扫几档, 找到 N=0 的那一档就是
//    example_top 里 RGB_DLY 应该写多少(再线性放大到 1280 宽)。
//  实测(W=32/64, 中值启用): 对齐点 = 2*W + 9~10  [不含 rgb2gray 的 7 拍]
//    即 W=1280 时 RGB_DLY 应 ≈ 7 + 2*1280 + 9 = 2576 (当前代码是 2566)
//
//  注意: L_gray / L_color / L_bin 打印出来的是"帧内位置滞后 mod W",
//        竖条纹场景下同一个列偏移会以 ±W 的倍数出现, 所以只看它 mod W 即可。
//
//  编译: vlog +define+TBP_W=16 +define+TBP_H=64  ...
//  运行: vsim -c -do "run -all; quit -f" <lib>.tb_align_probe
//======================================================================
module tb_align_probe;

localparam W    = `TBP_W;
localparam H    = `TBP_H;
localparam NPIX = W*H;
localparam THR  = 64;		//二值化阈值, 与例化保持一致
localparam BLK  = 8;		//棋盘块边长

reg clk = 0, rst_n = 0;
always #5 clk = ~clk;

reg [7:0] scene [0:NPIX-1];

//---------------- 激励: 场景 -> 像素流 ----------------
//  注: 这里跳过 vip_rgb2gray_proc, 直接把激励接到 median 的 sink。
//      原因: vip_rgb2gray_mult 里 8/10bit 信号接 16bit 乘法器端口, ModelSim
//      会把未连接的高位当 z, 乘法器输出全 X(纯仿真问题)。rgb2gray 自身
//      数据延迟=7、sop/eop/valid 延迟也=7(vip_rgb2gray_shift_regs), 两者相等,
//      对"流内位置"没有额外偏移, 所以最后统一 +7 即可。
reg  [7:0] din  = 0;
reg        vin  = 0, vsop = 0, veop = 0;

wire [7:0] g_data   = din;
wire       g_sop    = vsop;
wire       g_eop    = veop;
wire       g_valid  = vin;

//---------------- median ----------------
`ifdef TBP_NO_MEDIAN
//  旁路中值滤波, 只量 Sobel 一段(对应 example_top 里 MEDIAN_EN=0)
wire [7:0] m_data   = g_data;
wire       m_sop    = g_sop;
wire       m_eop    = g_eop;
wire       m_valid  = g_valid;
`else
wire [7:0] m_data;
wire       m_sop, m_eop, m_valid;

median_3x3 #(.WIDTH(W),.HEIGHT(H)) u_med (
	.clk         (clk),
	.rst_n       (rst_n),
	.sink_sop    (g_sop),
	.sink_eop    (g_eop),
	.sink_valid  (g_valid),
	.sink_data   (g_data),
	.source_sop  (m_sop),
	.source_eop  (m_eop),
	.source_valid(m_valid),
	.source_data (m_data)
);
`endif

//---------------- Sobel ----------------
wire [7:0] sob_data, sob_gray, sob_bin;
wire       s_sop, s_eop, s_valid;

Sobeledge_8d_proc #(
	.WIDTH(W),
	.HEIGHT(H),
	.SOBEL_MODE_4DIR(1'b0)
) u_sob (
	.clk         (clk),
	.rst_n       (rst_n),
	.mode        (2'd0),			//原图直通, 便于量灰度通道滞后
	.threshold   (THR),
	.sink_sop    (m_sop),
	.sink_eop    (m_eop),
	.sink_valid  (m_valid),
	.sink_data   (m_data),
	.source_sop  (s_sop),
	.source_eop  (s_eop),
	.source_valid(s_valid),
	.source_data (sob_data),
	.source_gray (sob_gray),
	.source_bin  (sob_bin)
);

//---------------- 抓输出流 ----------------
reg [7:0] cap_gray [0:NPIX-1];
reg       cap_bin  [0:NPIX-1];
integer   cap_n = 0;

//---------------- 复刻 example_top 的彩色延迟环 + 打包 ----------------
//  RGB_DLY 用来复现当前代码里的取值: 7 + (2*WIDTH-1)
`ifdef TBP_RGB_DLY
localparam RGB_DLY = `TBP_RGB_DLY;
`else
localparam RGB_DLY = 7 + (2*W - 1);
`endif
reg [7:0]  cring  [0:4095];
reg [11:0] cwptr  = 12'd0;
reg [11:0] craddr = 12'd0;
reg [7:0]  cval   = 8'd0;
always @(posedge clk) begin
	if(vin) begin
		cring[cwptr] <= din;
		cwptr  <= cwptr + 1'b1;
		craddr <= cwptr - (RGB_DLY - 1);
	end
	cval <= cring[craddr];
end

reg [7:0] cap_color [0:NPIX-1];

always @(posedge clk) begin
	if(rst_n && s_valid && (cap_n < NPIX)) begin
		cap_gray[cap_n] = sob_gray;
		cap_bin [cap_n] = (sob_bin >= 8'd128);
		cap_color[cap_n] = cval;
		cap_n = cap_n + 1;
	end
end

//---------------- 扫参 ----------------
integer i, k, L, mm, bestL, bestMM, r, c, ir, ic, firstR, firstB;
integer p11,p12,p13,p21,p22,p23,p31,p32,p33,gx,gy,mag,exp_bin,valid_cnt;

integer dbg = 0;
always @(posedge clk) begin
	if(rst_n && g_valid && (dbg < 12)) begin
		$display("  dbg%0d din=%0d g_data=%0d gv=%b m_data=%0d mv=%b gray=%0d bin=%0d sv=%b",
		         dbg, din, g_data, g_valid, m_data, m_valid, sob_gray, sob_bin, s_valid);
		dbg = dbg + 1;
	end
end

reg [7:0] scol [0:2047];

initial begin
	$display("===== align probe  W=%0d H=%0d =====", W, H);

	// 伪随机宽度竖条纹(宽 4..11), 值 0/255:
	//   - 中值滤波对整列同值的竖条纹是不变的, 所以 median 不改变图像
	//   - 宽度随机 => 没有小周期, 不会出现"匹配到别的平移量"的歧义
	begin : gen_scene
		reg [15:0] lfsr;
		reg [7:0]  cv;
		integer    ccol, wrun, j;
		lfsr = 16'hACE1;
		ccol = 0; cv = 8'd0;
		while(ccol < W) begin
			lfsr = {lfsr[14:0], lfsr[15]^lfsr[13]^lfsr[12]^lfsr[10]};
			wrun = 4 + {lfsr[2:0]};
			for(j=0;(j<wrun)&&(ccol<W);j=j+1) begin
				scol[ccol] = cv;
				ccol = ccol + 1;
			end
			cv = ~cv;
		end
		for(i=0;i<NPIX;i=i+1)
			scene[i] = scol[i%W];
	end

	repeat(20) @(posedge clk);
	rst_n = 1'b1;
	repeat(5)  @(posedge clk);

	for(i=0;i<NPIX;i=i+1) begin
		@(negedge clk);
		din  = scene[i];
		vin  = 1'b1;
		vsop = (i==0);
		veop = (i==NPIX-1);
	end
	@(negedge clk);
	vin = 0; vsop = 0; veop = 0;
	repeat(4*W+8000) @(posedge clk);

	$display("  captured pixels = %0d (expect %0d)", cap_n, NPIX);

	//----- L_gray: 灰度通道(原图)滞后 -----
	bestMM = 1<<30; bestL = -1;
	for(L=0;L<=4*W;L=L+1) begin
		mm = 0;
		for(k=L;k<NPIX;k=k+1) begin
			r = k/W; c = k%W;
			if(r>=3 && r<=H-4 && c>=3 && c<=W-4)
				if(cap_gray[k] !== scene[k-L]) mm = mm + 1;
		end
		if(mm < bestMM) begin bestMM = mm; bestL = L; end
	end
	$display("  L_gray = %0d  (mismatch %0d / %0d)", bestL, bestMM, NPIX);
	$display("  --- dump rows 0..9 (out vs scene shifted by L_gray) ---");
	for(r=0;r<10;r=r+1) begin
		$write("   r%0d out:", r);
		for(c=0;c<W;c=c+1) $write("%s", (cap_gray[r*W+c]>=8'd128) ? "#" : ".");
		$write("  exp:");
		for(c=0;c<W;c=c+1) begin
			k = r*W+c;
			if(k>=bestL) $write("%s", (scene[k-bestL]>=8'd128) ? "#" : ".");
			else         $write("?");
		end
		$write("\n");
	end
	$display("  --- scene rows 0..9 ---");
	for(r=0;r<10;r=r+1) begin
		$write("   s%0d   :", r);
		for(c=0;c<W;c=c+1) $write("%s", (scene[r*W+c]>=8'd128) ? "#" : ".");
		$write("\n");
	end

	//----- L_bin: 二值边缘位滞后 -----
	bestMM = 1<<30; bestL = -1;
	for(L=0;L<=4*W;L=L+1) begin
		mm = 0; valid_cnt = 0;
		for(k=L;k<NPIX;k=k+1) begin
			r = k/W; c = k%W;
			if(r>=3 && r<=H-4 && c>=3 && c<=W-4) begin
				ir = (k-L)/W; ic = (k-L)%W;
				if(ir>=1 && ir<=H-2 && ic>=1 && ic<=W-2) begin
					p11 = scene[(ir-1)*W+(ic-1)]; p12 = scene[(ir-1)*W+ic]; p13 = scene[(ir-1)*W+(ic+1)];
					p21 = scene[ ir   *W+(ic-1)]; p22 = scene[ ir   *W+ic]; p23 = scene[ ir   *W+(ic+1)];
					p31 = scene[(ir+1)*W+(ic-1)]; p32 = scene[(ir+1)*W+ic]; p33 = scene[(ir+1)*W+(ic+1)];
					gx  = (p13 + 2*p23 + p33) - (p11 + 2*p21 + p31);
					gy  = (p31 + 2*p32 + p33) - (p11 + 2*p12 + p13);
					if(gx < 0) gx = -gx;
					if(gy < 0) gy = -gy;
					mag = gx + gy;
					exp_bin = (mag >= THR) ? 1 : 0;
					if(cap_bin[k] !== exp_bin[0]) mm = mm + 1;
					valid_cnt = valid_cnt + 1;
				end
			end
		end
		if(mm < bestMM) begin bestMM = mm; bestL = L; end
	end
	$display("  L_bin  = %0d  (mismatch %0d / %0d scored)", bestL, bestMM, valid_cnt);
	$display("  => need RGB_DLY = L_bin = %0d ; drift(right +) = L_bin - RGB_DLY", bestL);

	//----- L_color: 彩色延迟环实际滞后 -----
	bestMM = 1<<30; bestL = -1;
	for(L=0;L<=4*W;L=L+1) begin
		mm = 0;
		for(k=L;k<NPIX;k=k+1) begin
			r = k/W; c = k%W;
			if(r>=3 && r<=H-4 && c>=3 && c<=W-4)
				if(cap_color[k] !== scene[k-L]) mm = mm + 1;
		end
		if(mm < bestMM) begin bestMM = mm; bestL = L; end
	end
	$display("  L_color = %0d  (mismatch %0d)   [RGB_DLY 设为 %0d]", bestL, bestMM, RGB_DLY);

	//----- 屏幕仿真: 彩色(经过 RGB_DLY 延迟环) + 红边(sobel_bin) -----
	$display("  --- screen emulation: color delayed by RGB_DLY=%0d, R = edge bit ---", RGB_DLY);
	$display("  --- raw: color only (left) / edge bit only (right) ---");
	for(r=2;r<5;r=r+1) begin
		$write("   c%0d :", r);
		for(c=0;c<W;c=c+1) $write("%s", (cap_color[r*W+c]>=8'd128) ? "#" : ".");
		$write("   e%0d :", r);
		for(c=0;c<W;c=c+1) $write("%s", cap_bin[r*W+c] ? "#" : ".");
		$write("\n");
	end
	for(r=0;r<10;r=r+1) begin
		$write("   r%0d :", r);
		for(c=0;c<W;c=c+1) begin
			k = r*W+c;
			if(cap_bin[k])                        $write("R");
			else if(cap_color[k] >= 8'd128)       $write("#");
			else                                  $write(".");
		end
		$write("\n");
	end
	r = H/2;
	firstR = -1; firstB = -1;
	for(c=0;c<W;c=c+1) begin
		k = r*W + c;
		if((firstR < 0) && cap_bin[k])          firstR = c;
		if((firstB < 0) && (cap_color[k]>=8'd128)) firstB = c;
	end
	$display("  row %0d: first red col=%0d, first bright color col=%0d, (red - color)=%0d",
	         r, firstR, firstB, firstR - firstB);

	//----- 数值化: 红边相对"彩色跳变位置"的整体偏移(单位: 半像素) -----
	begin : align2
		integer d, hit, tot, besthit, bestd, cc, ek, col_edge, rr;
		besthit = -1; bestd = 0; tot = 0;
		for(d=0; d<W; d=d+1) begin
			hit = 0; tot = 0;
			for(rr=3; rr<H-4; rr=rr+1) begin
				for(c=3; c<W-4; c=c+1) begin
					k  = rr*W + c;
					cc = c - d;  if(cc < 0) cc = cc + W;
					col_edge = (cap_color[rr*W+cc] !== cap_color[rr*W+((cc+W-1)%W)]);
					if(cap_bin[k] === col_edge) hit = hit + 1;
					tot = tot + 1;
				end
			end
			if(hit > besthit) begin besthit = hit; bestd = d; end
		end
		$display("  ALIGN2: red(d=0) matches color-edge shifted by %0d px  (hit %0d/%0d)  => red is %0d px right of color edge",
		         bestd, besthit, tot, bestd);
	end

	begin : meas_off
		integer c0, c1, bnd, bestb, dev, sum, cnt, run;
		r = H/2;
		sum = 0; cnt = 0; c = 2;
		while(c < W-2) begin
			if(cap_bin[r*W+c]) begin
				c0 = c;
				while((c < W-1) && cap_bin[r*W+c]) c = c + 1;
				c1 = c - 1;
				if((c1-c0) <= 3) begin
					// 彩色跳变: cap_color[c] != cap_color[c-1] => 边界在 2*c-1
					bestb = -1; dev = 1<<28;
					for(k=3;k<=W-3;k=k+1) begin
						if(cap_color[r*W+k] !== cap_color[r*W+k-1]) begin
							bnd = 2*k - 1;
							if( (c0+c1+1-bnd) < 0 ) run = -(c0+c1+1-bnd);
							else                     run =  (c0+c1+1-bnd);
							if(run < dev) begin dev = run; bestb = bnd; end
						end
					end
					if(bestb >= 0) begin
						sum = sum + (c0+c1+1-bestb);
						cnt = cnt + 1;
					end
				end
				c = c1 + 1;
			end
			else c = c + 1;
		end
		if(cnt > 0)
			$display("  ALIGN: red vs color edge offset = %0d half-px  (%0d red runs)  => %0d.%0d px (right +)",
			         sum/cnt, cnt, (sum/cnt)/2, ((sum/cnt)%2)*5);
		else
			$display("  ALIGN: no red run measured");
	end
	$finish;
end

endmodule
