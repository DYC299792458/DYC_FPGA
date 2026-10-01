`timescale 1ns/1ps
//======================================================================
// Sobeledge_8d_proc 单元测试 (配套赛题四基础要求第3条改造)
//
// 验证内容:
//   T1 平坦图 + 二值模式      -> 平坦区域不应出现假边缘
//   T2 垂直边 + 原图模式      -> 左暗右亮, 原图直通正确
//   T3 垂直边 + 边缘强度模式  -> 幅值 G = |Gx|+|Gy| = 4*(0x80-0x10) = 448
//                                输出 = 448>>3 = 0x38
//   T4 垂直边 + 二值模式 阈值256 -> 边缘两列出白(>=标准)
//   T5 垂直边 + 二值模式 阈值448 -> 仍出白(边界: 等于阈值算边缘)
//   T6 垂直边 + 二值模式 阈值456 -> 不出白
//   T7 45度斜边 + 边缘强度模式 -> 求和 672(0x54) 明显大于 四方向取最大 448(0x38)
//                                (验证 SOBEL_MODE_4DIR 参数确实切换了算法)
//   T8 每帧有效像素数 -> INFO (当前版本会少一整行, 属已知问题, 第3步修复)
//
// 运行: 双击同目录 run_sim.bat
//======================================================================
module tb_Sobeledge_8d_proc;

localparam WIDTH  = 16;
localparam HEIGHT = 8;

localparam PAT_FLAT  = 0;
localparam PAT_VEDGE = 1;
localparam PAT_DIAG  = 2;

localparam V_DARK   = 8'h10;	// 亮暗差 0x70 = 112 -> 4*112 = 448
localparam V_BRIGHT = 8'h80;
localparam V_FLAT   = 8'h55;

localparam EXP_MAX_SUM = 8'h38;	// 448>>3 = 56
localparam EXP_DIAG_SUM= 8'h54;	// 672>>3 = 84

integer errors = 0;

//---------------- 时钟与激励 ----------------
reg clk = 1'b0;
reg rst_n = 1'b0;
always #5 clk = ~clk;			// 100MHz

reg  [1:0]  mode = 2'd2;
reg  [10:0] thr  = 11'd256;
reg  [7:0]  din  = 8'h00;
reg         vin  = 1'b0;
reg         vsop = 1'b0;
reg         veop = 1'b0;

wire [7:0] d_sum, d_4d;
wire       v_sum, v_4d;
wire       sop_sum, eop_sum, sop_4d, eop_4d;

// 两个 DUT: 一个走 |Gx|+|Gy|, 一个走四方向取最大
Sobeledge_8d_proc #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SOBEL_MODE_4DIR(0)) dut_sum (
	.clk(clk), .rst_n(rst_n), .mode(mode), .threshold(thr),
	.sink_sop(vsop), .sink_eop(veop), .sink_valid(vin), .sink_data(din),
	.source_sop(sop_sum), .source_valid(v_sum), .source_data(d_sum), .source_eop(eop_sum)
);
Sobeledge_8d_proc #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SOBEL_MODE_4DIR(1)) dut_4d (
	.clk(clk), .rst_n(rst_n), .mode(mode), .threshold(thr),
	.sink_sop(vsop), .sink_eop(veop), .sink_valid(vin), .sink_data(din),
	.source_sop(sop_4d), .source_valid(v_4d), .source_data(d_4d), .source_eop(eop_4d)
);

//---------------- 输出统计 ----------------
integer    chk_on = 0;
integer    o_cnt  = 0;				// 本帧有效输出像素数
integer    o_col  = 0;
integer    dbg_n  = 0;				// 调试打印计数
integer    i;
reg [15:0] col_white [0:WIDTH-1];	// 二值: 每列白点数
reg [7:0]  col_max   [0:WIDTH-1];	// 强度: 每列最大值
reg [7:0]  col_max4d [0:WIDTH-1];	// 强度(四方向): 每列最大值
reg [7:0]  col_raw   [0:WIDTH-1];	// 原图: 每列采样值

task clr_stats;
begin
	o_cnt = 0;
	for(i=0;i<WIDTH;i=i+1) begin
		col_white[i] = 16'd0;
		col_max[i]   = 8'h00;
		col_max4d[i] = 8'h00;
		col_raw[i]   = 8'h00;
	end
end
endtask

always @(posedge clk) begin
	if(rst_n && chk_on && v_sum) begin
		o_col = o_cnt % WIDTH;
		col_raw[o_col] = d_sum;
		if(d_sum > col_max[o_col])   col_max[o_col]   <= d_sum;
		if(d_4d  > col_max4d[o_col]) col_max4d[o_col] <= d_4d;
		if(d_sum == 8'hff) col_white[o_col] <= col_white[o_col] + 1'b1;
		if(dbg_n > 0) begin
			$display("      dbg: k=%0d col=%0d din=%02h d_sum=%02h d_4d=%02h", o_cnt, o_col, din, d_sum, d_4d);
			dbg_n = dbg_n - 1;
		end
		o_cnt <= o_cnt + 1;
	end
end

//---------------- 激励任务 ----------------
task idle(input integer n);
begin
	vin = 1'b0; vsop = 1'b0; veop = 1'b0;
	repeat(n) @(posedge clk);
end
endtask

task send_frame(input integer pat);
integer x, y;
reg [7:0] v;
begin
	for(y=0;y<HEIGHT;y=y+1) begin
		for(x=0;x<WIDTH;x=x+1) begin
			case(pat)
			PAT_FLAT : v = V_FLAT;
			PAT_VEDGE: v = (x < (WIDTH/2)) ? V_DARK : V_BRIGHT;
			PAT_DIAG : v = ((x+y) >= (WIDTH/2)) ? V_BRIGHT : V_DARK;
			default  : v = V_FLAT;
			endcase
			@(negedge clk);
			din  = v;
			vin  = 1'b1;
			vsop = (y==0) && (x==0);
			veop = (y==HEIGHT-1) && (x==WIDTH-1);
		end
	end
	@(negedge clk);
	vin = 1'b0; vsop = 1'b0; veop = 1'b0;
end
endtask

task chk(input integer cond, input [8*64:1] name);
begin
	if(cond) $display("   [PASS] %0s", name);
	else begin
		$display("   [FAIL] %0s", name);
		errors = errors + 1;
	end
end
endtask

//---------------- 主流程 ----------------
integer c, nwhite, ncol, maxv, max4d, frame_pix;

initial begin
	$display("=========================================================");
	$display(" Sobeledge_8d_proc unit test   WIDTH=%0d HEIGHT=%0d", WIDTH, HEIGHT);
	$display("=========================================================");

	rst_n = 1'b0;
	clr_stats;
	repeat(20) @(posedge clk);
	rst_n = 1'b1;
	repeat(5) @(posedge clk);

	//--------------- T1 平坦图 + 二值 --------------
	$display("");
	$display("[T1] flat image + binary mode + thr=256  -> expect: no white pixel");
	mode = 2'd2; thr = 11'd256;
	clr_stats; chk_on = 1;
	send_frame(PAT_FLAT);
	repeat(30) @(posedge clk);
	chk_on = 0;
	nwhite = 0;
	for(c=2;c<=12;c=c+1) nwhite = nwhite + col_white[c];
	frame_pix = o_cnt;
	chk(nwhite == 0, "flat area : no false edge");

	//--------------- T2 垂直边 + 原图 --------------
	$display("");
	$display("[T2] vertical edge + raw mode -> expect: left 0x10, right 0x80");
	mode = 2'd0; thr = 11'd256;
	idle(20);
	clr_stats; chk_on = 1;
	send_frame(PAT_VEDGE);
	repeat(30) @(posedge clk);
	chk_on = 0;
	chk((col_raw[2]==V_DARK) && (col_raw[3]==V_DARK) && (col_raw[4]==V_DARK),
	    "raw passthrough : left 3 columns = 0x10");
	chk((col_raw[10]==V_BRIGHT) && (col_raw[11]==V_BRIGHT) && (col_raw[12]==V_BRIGHT),
	    "raw passthrough : right 3 columns = 0x80");

	//--------------- T3 垂直边 + 边缘强度 --------------
	$display("");
	$display("[T3] vertical edge + strength mode -> expect: max 0x38 (=448>>3)");
	mode = 2'd1; thr = 11'd256;
	idle(20);
	clr_stats; chk_on = 1;
	send_frame(PAT_VEDGE);
	repeat(30) @(posedge clk);
	chk_on = 0;
	maxv = 0; ncol = 0;
	for(c=2;c<=12;c=c+1) begin
		if(col_max[c] > maxv) maxv = col_max[c];
		if(col_max[c] != 0)  ncol = ncol + 1;
	end
	$display("   max=%0d (0x%h), columns with data=%0d", maxv, maxv, ncol);
	chk(maxv == EXP_MAX_SUM, "|Gx|+|Gy| magnitude correct (448>>3=0x38)");
	chk(ncol == 2, "vertical edge affects exactly 2 columns");

	//--------------- T4 阈值 256 --------------
	$display("");
	$display("[T4] vertical edge + binary mode + thr=256 -> expect: 2 white columns");
	mode = 2'd2; thr = 11'd256;
	idle(20);
	clr_stats; chk_on = 1;
	send_frame(PAT_VEDGE);
	repeat(30) @(posedge clk);
	chk_on = 0;
	ncol = 0;
	for(c=2;c<=12;c=c+1) if(col_white[c] != 0) ncol = ncol + 1;
	chk(ncol == 2, "thr=256 : exactly 2 columns white");

	//--------------- T5 阈值 448 (边界, >= 应出白) --------------
	$display("");
	$display("[T5] vertical edge + binary mode + thr=448 -> expect: still white (>= thr)");
	thr = 11'd448;
	idle(20);
	clr_stats; chk_on = 1;
	send_frame(PAT_VEDGE);
	repeat(30) @(posedge clk);
	chk_on = 0;
	ncol = 0;
	for(c=2;c<=12;c=c+1) if(col_white[c] != 0) ncol = ncol + 1;
	chk(ncol == 2, "thr=448 : boundary value still edge");

	//--------------- T6 阈值 456 (应不出白) --------------
	$display("");
	$display("[T6] vertical edge + binary mode + thr=456 -> expect: all black");
	thr = 11'd456;
	idle(20);
	clr_stats; chk_on = 1;
	send_frame(PAT_VEDGE);
	repeat(30) @(posedge clk);
	chk_on = 0;
	nwhite = 0;
	for(c=0;c<WIDTH;c=c+1) nwhite = nwhite + col_white[c];
	chk(nwhite == 0, "thr=456 : no white pixel in whole frame");

	//--------------- T7 45度斜边: 对比两种算法 --------------
	$display("");
	$display("[T7] 45deg edge + strength -> expect: sum 0x54 > 4-direction 0x38");
	mode = 2'd1; thr = 11'd0;
	idle(20);
	clr_stats; chk_on = 1;
	send_frame(PAT_DIAG);
	repeat(30) @(posedge clk);
	chk_on = 0;
	maxv = 0; max4d = 0;
	for(c=2;c<=12;c=c+1) begin
		if(col_max[c]   > maxv)  maxv  = col_max[c];
		if(col_max4d[c] > max4d) max4d = col_max4d[c];
	end
	$display("   sum mode max=0x%h (expect 0x54), 4div mode max=0x%h (expect 0x38)", maxv, max4d);
	chk(maxv > max4d, "SOBEL_MODE_4DIR really switches the algorithm");
	chk(maxv == EXP_DIAG_SUM, "sum mode diagonal magnitude correct (672>>3=0x54)");

	//--------------- T8 每帧像素数 --------------
	$display("");
	$display("[T8] valid pixels per frame");
	$display("   output %0d pixels, input frame %0d pixels", frame_pix, WIDTH*HEIGHT);
	$display("   (NOTE: current version drops the first line - known issue, fixed in step 3)");

	//--------------- 总结 --------------
	$display("");
	$display("=========================================================");
	if(errors == 0) $display(" RESULT: ALL CHECKS PASSED (0 error)");
	else            $display(" RESULT: %0d CHECK(S) FAILED", errors);
	$display("=========================================================");
	$finish;
end

// 超时保护
initial begin
	#2_000_000;
	$display(" TIMEOUT!");
	$finish;
end

endmodule
