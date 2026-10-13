//======================================================================
//  tb_canny_edge.v   Canny 单元测试(配合 sim/canny_model.py 对拍)
//
//  1) python3 canny_model.py gen W H TH_HI TH_LO <dir>  生成测试图与期望结果
//  2) 本 testbench 读 canny_in.hex ({side,gray}), 连送两帧, 记录第二帧输出到 canny_rtl.txt
//     GAPPY=1 时 valid 随机断续, 用来验证对齐与 valid 是否连续无关
//  3) python3 canny_model.py check <dir> 逐像素比较(去掉边界钳位区)
//  另外在本 testbench 内检查: 每帧输出像素数 == W*H, sop/eop 各一个
//======================================================================
`timescale 1ns/1ps
module tb_canny_edge;
	parameter W      = 64;
	parameter H      = 48;
	parameter TH_HI  = 160;
	parameter TH_LO  = 80;
	parameter GAPPY  = 0;
	localparam N = W*H;

	reg clk = 0, rst_n = 0;
	always #5 clk = ~clk;

	reg        sop = 0, eop = 0, valid = 0, side = 0;
	reg  [7:0] data = 0;
	wire       osop, oeop, ovalid, oedge, oside;

	canny_edge #(.WIDTH(W),.HEIGHT(H)) dut(
		.clk(clk),.rst_n(rst_n),.th_high(TH_HI[10:0]),.th_low(TH_LO[10:0]),
		.sink_sop(sop),.sink_eop(eop),.sink_valid(valid),.sink_data(data),.side_in(side),
		.source_sop(osop),.source_eop(oeop),.source_valid(ovalid),.source_edge(oedge),.side_out(oside));

	reg [8:0] img [0:N-1];
	integer frame_out = 0, cnt = 0, nsop = 0, neop = 0, errors = 0, fd, k, f;

	always @(posedge clk) if (ovalid) begin
		if (osop) begin cnt = 0; nsop = nsop + 1; end
		if (frame_out == 1) $fwrite(fd, "%0d %0d\n", oedge, oside);
		cnt = cnt + 1;
		if (oeop) begin
			neop = neop + 1;
			if (cnt != N) begin errors = errors + 1; $display("  [FAIL] frame %0d has %0d pixels, expect %0d", frame_out, cnt, N); end
			frame_out = frame_out + 1;
		end
	end

	initial begin
		$readmemh("canny_in.hex", img);
		fd = $fopen("canny_rtl.txt", "w");
		repeat (5) @(posedge clk); rst_n <= 1; repeat (5) @(posedge clk);
		for (f = 0; f < 3; f = f + 1) begin
			for (k = 0; k < N; k = k + 1) begin
				if (GAPPY) begin
					sop <= 0; eop <= 0; valid <= 0;
					while (($random & 3) == 0) @(posedge clk);
				end
				sop <= (k == 0); eop <= (k == N-1); valid <= 1;
				{side, data} <= img[k];
				@(posedge clk);
			end
			sop <= 0; eop <= 0; valid <= 0;
			repeat (2*W) @(posedge clk);	// 场消隐
		end
		repeat (50) @(posedge clk);
		$fclose(fd);
		if (nsop != 3 || neop != 3) begin errors = errors + 1; $display("  [FAIL] sop=%0d eop=%0d, expect 3/3", nsop, neop); end
		$display(" frames out=%0d, frame-structure errors=%0d (GAPPY=%0d)", frame_out, errors, GAPPY);
		$finish;
	end
endmodule
