`timescale 1ns/1ps
// 临时探针: 送一帧"数据=像素序号"的图, 打印 matrix_valid 有效时窗口里的9个值,
// 从而直接量出窗口相对于帧标记的真实位置(行/列偏移)
module tb_matrix_probe;

localparam WIDTH  = 16;
localparam HEIGHT = 8;

reg clk = 0, rst_n = 0;
always #5 clk = ~clk;

reg [7:0] din = 0;
reg vin = 0, vsop = 0, veop = 0;
wire [7:0] p11,p12,p13,p21,p22,p23,p31,p32,p33;
wire vsop_o, veop_o, vvalid;

Sobeledge_matrix_3x3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) dut (
	.clk(clk), .rst_n(rst_n),
	.sink_sop(vsop), .sink_eop(veop), .sink_valid(vin), .sink_data(din),
	.matrix_sop(vsop_o), .matrix_eop(veop_o), .matrix_valid(vvalid),
	.matrix_p11(p11),.matrix_p12(p12),.matrix_p13(p13),
	.matrix_p21(p21),.matrix_p22(p22),.matrix_p23(p23),
	.matrix_p31(p31),.matrix_p32(p32),.matrix_p33(p33)
);

integer i;
integer cnt = 0;
integer got_sop = 0;

always @(posedge clk) begin
	if(rst_n && vvalid) begin
		if(vsop_o) got_sop = got_sop + 1;
		if(cnt < 10) begin
			$display("  mark#%0d  win=[%0d %0d %0d | %0d %0d %0d | %0d %0d %0d]  sop=%0d",
				cnt, p11,p12,p13,p21,p22,p23,p31,p32,p33, vsop_o);
		end
		cnt = cnt + 1;
	end
end

initial begin
	$display("=========== matrix window probe: data = pixel index ===========");
	repeat(20) @(posedge clk);
	rst_n = 1;
	repeat(5) @(posedge clk);

	for(i=0;i<WIDTH*HEIGHT;i=i+1) begin
		@(negedge clk);
		din  = i[7:0];
		vin  = 1'b1;
		vsop = (i==0);
		veop = (i==WIDTH*HEIGHT-1);
	end
	@(negedge clk); vin=0; vsop=0; veop=0;
	repeat(300) @(posedge clk);

	$display("  total valid marks = %0d, sop marks = %0d", cnt, got_sop);
	$finish;
end

endmodule
