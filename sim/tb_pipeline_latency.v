`timescale 1ns/1ps
// 探针: 量 median_3x3 -> Sobeledge_8d_proc 的延迟(以像素为单位)
// [dyc] 原来用"每行一个常数"的测试图, 只能分辨行, 列方向有 +-1 的模糊(相邻滞后量匹配数几乎打平)。
//       也不能用逐像素随机图: 中值滤波的输出本来就不等于输入, 只有约 1/9 的像素会碰巧相等, 测出的滞后量是噪声。
//       现改为 8x8 随机色块图: 块内部及直边处中值 = 中心像素, 只有块角处略有差异,
//       真实滞后量匹配率约 94%, 错一列只有约 87%, 能唯一区分。
// 实测(窗口帧标记修正后): 每级 3x3 窗口滞后 WIDTH+1 像素(1行1列), 两级共 2*WIDTH+2
//       Canny 4 级窗口再加 4*(WIDTH+1), 见 tb_canny_edge 的旁路位对拍
//       WIDTH=1280: 中值+Sobel 2562, +Canny 5124, +rgb2gray 7 => 顶层 RGB_DLY = 7693
module tb_pipeline_latency;

localparam WIDTH  = 64;
localparam HEIGHT = 64; 		// 行数足够长, 保证延迟后仍落在帧内
localparam NPIX   = WIDTH*HEIGHT;

reg clk = 0, rst_n = 0;
always #5 clk = ~clk;

reg  [7:0] din = 0;
reg        vin = 0, vsop = 0, veop = 0;

wire [7:0] med_data;
wire       med_sop, med_eop, med_valid;

median_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_med (
	.clk(clk), .rst_n(rst_n),
	.sink_sop(vsop), .sink_eop(veop), .sink_valid(vin), .sink_data(din),
	.source_sop(med_sop), .source_eop(med_eop), .source_valid(med_valid), .source_data(med_data)
);

wire [7:0] gray, edge_data;
wire       ve, se, ee;

Sobeledge_8d_proc #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SOBEL_MODE_4DIR(0)) u_sob (
	.clk(clk), .rst_n(rst_n), .mode(2'd0), .threshold(11'd256),
	.sink_sop(med_sop), .sink_eop(med_eop), .sink_valid(med_valid), .sink_data(med_data),
	.source_sop(se), .source_eop(ee), .source_valid(ve), .source_data(edge_data), .source_gray(gray)
);

reg [7:0] in_img  [0:NPIX-1];
reg [7:0] out_img [0:NPIX-1];
integer   o_cnt = 0;
integer   i, k;
reg [7:0] blk [0:(WIDTH/8)*(HEIGHT/8)-1];
initial for(k=0;k<(WIDTH/8)*(HEIGHT/8);k=k+1) blk[k] = $random;

always @(posedge clk) begin
	if(rst_n && ve) begin
		out_img[o_cnt] = gray;
		o_cnt = o_cnt + 1;
	end
end

initial begin
	$display("===== pipeline latency probe   WIDTH=%0d HEIGHT=%0d =====", WIDTH, HEIGHT);
	repeat(20) @(posedge clk);
	rst_n = 1'b1;
	repeat(5) @(posedge clk);

	for(i=0;i<NPIX;i=i+1) begin
		in_img[i] = blk[((i/WIDTH)/8)*(WIDTH/8) + (i%WIDTH)/8];	// 8x8 随机色块
		@(negedge clk);
		din  = in_img[i];
		vin  = 1'b1;
		vsop = (i==0);
		veop = (i==NPIX-1);
	end
	@(negedge clk); vin=0; vsop=0; veop=0;
	repeat(6000) @(posedge clk);

	$display("  output pixels captured = %0d  (expect %0d)", o_cnt, NPIX);
	// 扫描候选滞后量 L, 看 out_img[k] 是否等于 in_img[k-L], 找匹配最多的那个 L
	begin : scan
		integer L, hit, best_l, best_hit;
		best_l = -1; best_hit = -1;
		for(L=0;L<=6*WIDTH;L=L+1) begin
			hit = 0;
			for(k=L;k<NPIX;k=k+1)
				if(out_img[k] === in_img[k-L]) hit = hit + 1;
			if(hit > best_hit) begin best_hit = hit; best_l = L; end
		end
		$display("  best lag = %0d pixels  ( %0d rows + %0d columns ), matches %0d / %0d",
		          best_l, best_l/WIDTH, best_l%WIDTH, best_hit, NPIX-best_l);
		if(best_l == 2*WIDTH+2 && best_hit*100 > 90*(NPIX-best_l))
			$display("  RESULT: PASS  (lag = 2*WIDTH+2 as designed)");
		else
			$display("  RESULT: FAIL  (expect lag %0d)", 2*WIDTH+2);
	end
	$finish;
end

endmodule
