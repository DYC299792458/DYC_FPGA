//======================================================================
//  tb_matrix_gap.v   3x3 窗口对 valid 间隔的鲁棒性测试
//  同一帧图像送两次: (A) valid 连续  (B) valid 随机断续(模拟行尾/FIFO 空)
//  每个 matrix_valid 时刻记录完整 3x3 窗口, 两种情况的输出序列必须完全相同。
//  若帧标记与窗口数据的对齐依赖于 valid 是否连续, 本测试会报错。
//======================================================================
`timescale 1ns/1ps
module tb_matrix_gap;
	localparam W = 16, H = 10, N = W*H;
	reg clk = 0, rst_n = 0;
	always #5 clk = ~clk;

	reg        sop, eop, valid;
	reg  [7:0] data;
	wire [7:0] p11,p12,p13,p21,p22,p23,p31,p32,p33;
	wire       msop, meop, mvalid;

	Sobeledge_matrix_3x3 #(.WIDTH(W),.HEIGHT(H),.DWIDTH(8)) dut(
		.clk(clk),.rst_n(rst_n),
		.sink_sop(sop),.sink_eop(eop),.sink_valid(valid),.sink_data(data),
		.matrix_sop(msop),.matrix_eop(meop),.matrix_valid(mvalid),
		.matrix_p11(p11),.matrix_p12(p12),.matrix_p13(p13),
		.matrix_p21(p21),.matrix_p22(p22),.matrix_p23(p23),
		.matrix_p31(p31),.matrix_p32(p32),.matrix_p33(p33));

	reg [7:0] img [0:N-1];
	reg [71:0] capA [0:N-1];
	reg [71:0] capB [0:N-1];
	integer cntA, cntB, phase, i, errors;

	always @(posedge clk) if (mvalid) begin
		if (phase == 0) begin capA[cntA] <= {p11,p12,p13,p21,p22,p23,p31,p32,p33}; cntA <= cntA + 1; end
		else            begin capB[cntB] <= {p11,p12,p13,p21,p22,p23,p31,p32,p33}; cntB <= cntB + 1; end
	end

	task send_frame(input integer gappy);
		integer k;
		begin
			for (k = 0; k < N; k = k + 1) begin
				if (gappy) begin
					sop <= 0; eop <= 0; valid <= 0;
					while (($random & 3) == 0) @(posedge clk);
				end
				sop <= (k == 0); eop <= (k == N-1); valid <= 1; data <= img[k];
				@(posedge clk);
			end
			sop <= 0; eop <= 0; valid <= 0;
			repeat (20) @(posedge clk);
		end
	endtask

	initial begin
		for (i = 0; i < N; i = i + 1) img[i] = $random;
		sop = 0; eop = 0; valid = 0; data = 0; cntA = 0; cntB = 0; phase = 0; errors = 0;
		repeat (5) @(posedge clk); rst_n <= 1; repeat (5) @(posedge clk);
		// 每种情况送两帧, 只比较第二帧(第一帧行缓存内容与上一帧有关)
		phase = 0; send_frame(0); cntA = 0; send_frame(0);
		phase = 1; send_frame(1); cntB = 0; send_frame(1);
		if (cntA != N || cntB != N) begin
			$display("  [FAIL] pixel count A=%0d B=%0d expect %0d", cntA, cntB, N); errors = errors + 1;
		end
		for (i = 0; i < N; i = i + 1)
			if (capA[i] !== capB[i]) begin
				errors = errors + 1;
				if (errors <= 6) $display("  [FAIL] pix %0d (row %0d col %0d): continuous=%h gappy=%h", i, i/W, i%W, capA[i], capB[i]);
			end
		$display("=========================================================");
		$display(" matrix valid-gap test: %0d mismatching windows", errors);
		if (errors == 0) $display(" RESULT: ALL CHECKS PASSED");
		else             $display(" RESULT: FAILED");
		$display("=========================================================");
		$finish;
	end
endmodule
