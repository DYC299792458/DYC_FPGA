`timescale 1ns/1ps
//======================================================================
// median_3x3 单元测试
//
// 做法: 把输入整帧存入 in_img, 把输出整帧存入 out_img,
//       然后对"窗口完全落在帧内"的内部区域逐像素和真中值比对
//       (内部区域取 x=2..WIDTH-3, y=2..HEIGHT-1, 避开边界钳位)
//
//   T1 平坦图          全 0x40
//   T2 椒盐噪声        0x40 背景 + 孤立 0xFF/0x00 噪点 -> 中间区域应被滤平
//   T3 垂直边          左 0x20 右 0xC0 -> 中值滤波应保持边缘
//   T4 伪随机图        检验中值网络本身是否正确
//======================================================================
module tb_median_3x3;

localparam WIDTH  = 16;
localparam HEIGHT = 8;
localparam NPIX   = WIDTH*HEIGHT;

localparam PAT_FLAT  = 0;
localparam PAT_SALT  = 1;
localparam PAT_VEDGE = 2;
localparam PAT_RAND  = 3;
localparam PAT_HBAND = 4;	// 第3,4行亮带 -> 用来定位窗口的行位置

integer errors = 0;

reg clk = 1'b0;
reg rst_n = 1'b0;
always #5 clk = ~clk;

reg  [7:0] din = 8'h00;
reg        vin = 1'b0, vsop = 1'b0, veop = 1'b0;
wire [7:0] dout;
wire       vout, sopout, eopout;

median_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) dut (
	.clk(clk), .rst_n(rst_n),
	.sink_sop(vsop), .sink_eop(veop), .sink_valid(vin), .sink_data(din),
	.source_sop(sopout), .source_eop(eopout), .source_valid(vout), .source_data(dout)
);

reg [7:0] in_img  [0:NPIX-1];
reg [7:0] out_img [0:NPIX-1];

integer chk_on = 0;
integer o_cnt  = 0;
integer x, y, i, j, nb;
reg [31:0] rnd = 32'h1234_5678;
reg [7:0]  v, tmp;

always @(posedge clk) begin
	if(rst_n && chk_on && vout) begin
		out_img[o_cnt] = dout;
		o_cnt = o_cnt + 1;
	end
end

//---------------- 参考模型: 9个数排序取中间值 ----------------
function [7:0] med9;
	input [7:0] a0,a1,a2,a3,a4,a5,a6,a7,a8;
	reg [7:0] vt [0:8];
	integer m,n;
	begin
		vt[0]=a0; vt[1]=a1; vt[2]=a2;
		vt[3]=a3; vt[4]=a4; vt[5]=a5;
		vt[6]=a6; vt[7]=a7; vt[8]=a8;
		for(m=0;m<9;m=m+1)
			for(n=0;n<8;n=n+1)
				if(vt[n] > vt[n+1]) begin
					tmp = vt[n]; vt[n] = vt[n+1]; vt[n+1] = tmp;
				end
		med9 = vt[4];
	end
endfunction

task chk(input integer cond, input [8*56:1] name);
begin
	if(cond) $display("   [PASS] %0s", name);
	else begin
		$display("   [FAIL] %0s", name);
		errors = errors + 1;
	end
end
endtask

//---------------- 发送一帧, 同时记录到 in_img ----------------
task send_frame(input integer pat);
integer xx, yy;
reg [7:0] val;
begin
	for(yy=0;yy<HEIGHT;yy=yy+1) begin
		for(xx=0;xx<WIDTH;xx=xx+1) begin
			case(pat)
			PAT_FLAT : val = 8'h40;
			PAT_SALT : begin
				if((xx==3 && yy==2) || (xx==10 && yy==5))
					val = 8'hff;						// 盐噪点
				else if((xx==7 && yy==4) || (xx==12 && yy==2))
					val = 8'h00;						// 椒噪点
				else
					val = 8'h40;
			end
			PAT_VEDGE: val = (xx < (WIDTH/2)) ? 8'h20 : 8'hc0;
			PAT_RAND : begin
				rnd = rnd * 32'd1103515245 + 32'd12345;
				val = rnd[23:16];
			end
			PAT_HBAND: val = ((yy==3)||(yy==4)) ? 8'hff : 8'h00;
			default  : val = 8'h40;
			endcase
			in_img[yy*WIDTH+xx] = val;
			@(negedge clk);
			din  = val;
			vin  = 1'b1;
			vsop = (yy==0) && (xx==0);
			veop = (yy==HEIGHT-1) && (xx==WIDTH-1);
		end
	end
	@(negedge clk);
	vin = 1'b0; vsop = 1'b0; veop = 1'b0;
end
endtask

//---------------- 内部区域与真中值逐像素比对 ----------------
// 说明: 3x3 窗口的中心天生比输入流晚 1 行 1 列, 输出帧因此相对输入帧整体平移,
//       即 DUT 在 (x,y) 的输出 = 以 (x-1,y-1) 为中心的 3x3 窗口的中值。
//       [dyc] 窗口帧标记修正(与 valid 是否连续无关)后多了 1 列平移, 原来是 (x,y-1)。
//       比对范围避开边界钳位: x=3..WIDTH-2, y=2..HEIGHT-2
task check_interior(input [8*56:1] name);
reg [7:0] exp;
begin
	nb = 0;
	for(y=2;y<=HEIGHT-2;y=y+1)
		for(x=3;x<=WIDTH-2;x=x+1) begin
			exp = med9(in_img[(y-2)*WIDTH+x-2], in_img[(y-2)*WIDTH+x-1], in_img[(y-2)*WIDTH+x],
			           in_img[(y-1)*WIDTH+x-2], in_img[(y-1)*WIDTH+x-1], in_img[(y-1)*WIDTH+x],
			           in_img[ y   *WIDTH+x-2], in_img[ y   *WIDTH+x-1], in_img[ y   *WIDTH+x]);
			if(out_img[y*WIDTH+x] !== exp) nb = nb + 1;
		end
	$display("   interior checked x%0d..x%0d y2..y%0d, mismatch = %0d",
	         3, WIDTH-2, HEIGHT-2, nb);
	chk(nb == 0, name);
end
endtask

task run_case(input integer pat, input [8*56:1] name);
begin
	o_cnt = 0; chk_on = 1;
	send_frame(pat);
	repeat(40) @(posedge clk);		//等流水排空
	chk_on = 0;
	$display("   output pixels = %0d", o_cnt);
	check_interior(name);
end
endtask

initial begin
	$display("=========================================================");
	$display(" median_3x3 unit test   WIDTH=%0d HEIGHT=%0d", WIDTH, HEIGHT);
	$display("=========================================================");

	rst_n = 1'b0;
	repeat(20) @(posedge clk);
	rst_n = 1'b1;
	repeat(5) @(posedge clk);

	$display("");
	$display("[T1] flat image 0x40 -> all 0x40");
	run_case(PAT_FLAT, "flat image passes through");

	$display("");
	$display("[T2] salt and pepper noise -> noise removed in interior");
	run_case(PAT_SALT, "salt/pepper removed (interior == background)");
	// 单独确认噪点位置周围的输出确实是背景值
	begin : noise
		integer k;
		k = 0;
		// 噪点在 (x,y), 结果出现在输出的 (x+1,y+1) —— 因为窗口中心比输入流晚一行一列
		if(out_img[3*WIDTH+4] !== 8'h40) k = k + 1;	// 盐噪点(3,2)
		if(out_img[6*WIDTH+11] !== 8'h40) k = k + 1;	// 盐噪点(10,5)
		if(out_img[5*WIDTH+8] !== 8'h40) k = k + 1;	// 椒噪点(7,4)
		$display("   noise pixel outputs: %0d %0d %0d",
		         out_img[3*WIDTH+4], out_img[6*WIDTH+11], out_img[5*WIDTH+8]);
		chk(k == 0, "noise pixels become background value");
	end

	$display("");
	$display("[T3] vertical edge -> edge preserved");
	run_case(PAT_VEDGE, "vertical edge matches reference median");

	$display("");
	$display("[T4] pseudo random image -> median network correct");
	run_case(PAT_RAND, "random image matches reference median");

	$display("");
	$display("[T5] horizontal bright band on rows 3,4 -> locate window row position");
	run_case(PAT_HBAND, "band pattern matches reference median");
	begin : band
		integer r;
		$display("   col8 output by row:");
		for(r=0;r<HEIGHT;r=r+1)
			$display("     row%0d : %0d   (input %0d)", r, out_img[r*WIDTH+8], in_img[r*WIDTH+8]);
	end

	$display("");
	$display("=========================================================");
	if(errors == 0) $display(" RESULT: ALL CHECKS PASSED (0 error)");
	else            $display(" RESULT: %0d CHECK(S) FAILED", errors);
	$display("=========================================================");
	$finish;
end

initial begin
	#2_000_000;
	$display(" TIMEOUT!");
	$finish;
end

endmodule
