//======================================================================
//  sobel_ctrl.v   Sobel 阈值 / 显示模式 按键控制 (两键方案)
//
//   K1 (key1_n) : 短按 阈值 -8 , 按住 0.5s 后每 80ms 连减
//   K2 (key2_n) : 短按 阈值 +8 , 按住 0.5s 后每 80ms 连加
//   K1+K2 同时按住 : 显示模式循环  原图(0) -> 边缘强度(1) -> 二值边缘(2)
//                    (组合键触发一次后需松开两键才能再次触发, 组合期间不动阈值)
//
//   按键低有效(按下=0), 内部两级同步 + 20ms 消抖
//   threshold : 0 ~ THR_MAX(=2040, 即 |Gx|+|Gy| 的最大值), 步长 8
//   mode      : 0=原图直通 1=边缘强度 2=二值化边缘
//   cfg_toggle: 配置变化标志(clk 域), 供跨时钟域时用"翻转+握手"搬到像素时钟域
//======================================================================
module sobel_ctrl #(
	parameter CLK_FREQ      = 96_000_000,	//本模块工作时钟频率
	parameter DEBOUNCE_MS   = 20,			//消抖时间(ms)
	parameter REPEAT_DLY_MS = 500,			//按住多久开始连发(ms)
	parameter REPEAT_INT_MS = 80,			//连发间隔(ms)
	parameter [10:0] THR_DEFAULT = 11'd256,	//上电默认阈值
	parameter [10:0] THR_STEP    = 11'd8,	//阈值步长
	parameter [10:0] THR_MAX     = 11'd2040,//阈值上限
	parameter [1:0]  MODE_DEFAULT= 2'd2		//上电默认模式: 二值化边缘
)(
	input              clk,
	input              rst_n,
	input              key1_n,		//K1: 阈值减
	input              key2_n,		//K2: 阈值加
	output reg  [10:0] threshold,	//当前阈值
	output reg  [1:0]  mode,		//当前显示模式
	output reg         cfg_toggle	//配置变化时翻转(跨时钟域握手用)
);

	localparam CNT_DB  = (CLK_FREQ/1000)*DEBOUNCE_MS;	//消抖计数
	localparam CNT_RDY = (CLK_FREQ/1000)*REPEAT_DLY_MS;	//连发起始延迟
	localparam CNT_INT = (CLK_FREQ/1000)*REPEAT_INT_MS;	//连发间隔

//----------------------------------------------------------------------
// 两级同步 (按键是异步输入)
//----------------------------------------------------------------------
	reg k1_s1,k1_s2,k2_s1,k2_s2;
	always @(posedge clk or negedge rst_n)
		if(!rst_n) begin k1_s1<=1'b1; k1_s2<=1'b1; k2_s1<=1'b1; k2_s2<=1'b1; end
		else begin
			k1_s1<=key1_n;  k1_s2<=k1_s1;
			k2_s1<=key2_n;  k2_s2<=k2_s1;
		end

//----------------------------------------------------------------------
// 消抖: 采样值与稳定值不同则计时, 连续 CNT_DB 个时钟一致才更新
//       消抖后 1=松开, 0=按下(低有效)
//----------------------------------------------------------------------
	reg        k1_clean,k2_clean;
	reg [20:0] db1_cnt,db2_cnt;
	always @(posedge clk or negedge rst_n)
		if(!rst_n) begin
			k1_clean<=1'b1;  k2_clean<=1'b1;
			db1_cnt<=21'd0;  db2_cnt<=21'd0;
		end
		else begin
			if(k1_s2 != k1_clean) begin
				if(db1_cnt >= CNT_DB-1) begin k1_clean<=k1_s2; db1_cnt<=21'd0; end
				else db1_cnt <= db1_cnt + 1'b1;
			end
			else db1_cnt <= 21'd0;

			if(k2_s2 != k2_clean) begin
				if(db2_cnt >= CNT_DB-1) begin k2_clean<=k2_s2; db2_cnt<=21'd0; end
				else db2_cnt <= db2_cnt + 1'b1;
			end
			else db2_cnt <= 21'd0;
		end

	wire p1   = ~k1_clean;		//K1 按下
	wire p2   = ~k2_clean;		//K2 按下
	wire both = p1 & p2;		//两个键同时按下 -> 切模式
	wire only1= p1 & ~p2;		//只按 K1 -> 阈值减
	wire only2= p2 & ~p1;		//只按 K2 -> 阈值加

//----------------------------------------------------------------------
// 组合键: 两键同时按下的上升沿触发一次模式切换
//----------------------------------------------------------------------
	reg both_d;
	always @(posedge clk or negedge rst_n)
		if(!rst_n) both_d <= 1'b0;
		else       both_d <= both;
	wire combo_hit = both & ~both_d;

//----------------------------------------------------------------------
// 单键按下沿 (用于阈值一步)
//----------------------------------------------------------------------
	reg p1_d,p2_d;
	always @(posedge clk or negedge rst_n)
		if(!rst_n) begin p1_d<=1'b0; p2_d<=1'b0; end
		else begin p1_d<=p1; p2_d<=p2; end
	wire dn_first = only1 & ~p1_d;		//K1 刚按下
	wire up_first = only2 & ~p2_d;		//K2 刚按下

//----------------------------------------------------------------------
// 连发: 按住 only1/only2 超过 CNT_RDY 后, 每 CNT_INT 产生一个脉冲
//----------------------------------------------------------------------
	reg [25:0] dn_cnt,up_cnt;
	reg        dn_hold,up_hold;
	reg        dn_tick,up_tick;
	always @(posedge clk or negedge rst_n)
		if(!rst_n) begin
			dn_hold<=1'b0; dn_cnt<=26'd0; dn_tick<=1'b0;
			up_hold<=1'b0; up_cnt<=26'd0; up_tick<=1'b0;
		end
		else begin
			//---- K1 连减 ----
			if(!only1) begin dn_hold<=1'b0; dn_cnt<=26'd0; dn_tick<=1'b0; end
			else if(!dn_hold) begin
				dn_tick<=1'b0;
				if(dn_cnt >= CNT_RDY-1) begin dn_hold<=1'b1; dn_cnt<=26'd0; end
				else dn_cnt <= dn_cnt + 1'b1;
			end
			else begin
				if(dn_cnt >= CNT_INT-1) begin dn_cnt<=26'd0; dn_tick<=1'b1; end
				else begin dn_cnt <= dn_cnt + 1'b1; dn_tick<=1'b0; end
			end
			//---- K2 连加 ----
			if(!only2) begin up_hold<=1'b0; up_cnt<=26'd0; up_tick<=1'b0; end
			else if(!up_hold) begin
				up_tick<=1'b0;
				if(up_cnt >= CNT_RDY-1) begin up_hold<=1'b1; up_cnt<=26'd0; end
				else up_cnt <= up_cnt + 1'b1;
			end
			else begin
				if(up_cnt >= CNT_INT-1) begin up_cnt<=26'd0; up_tick<=1'b1; end
				else begin up_cnt <= up_cnt + 1'b1; up_tick<=1'b0; end
			end
		end

	wire dn_step = dn_first | dn_tick;	//阈值减一步
	wire up_step = up_first | up_tick;	//阈值加一步

//----------------------------------------------------------------------
// 阈值寄存器 (带上下限钳位)
//----------------------------------------------------------------------
	always @(posedge clk or negedge rst_n)
		if(!rst_n) threshold <= THR_DEFAULT;
		else if(dn_step) threshold <= (threshold > THR_STEP) ? (threshold - THR_STEP) : 11'd0;
		else if(up_step) threshold <= (threshold < (THR_MAX-THR_STEP)) ? (threshold + THR_STEP) : THR_MAX;

//----------------------------------------------------------------------
// 模式寄存器: 2(二值) -> 0(原图) -> 1(强度) -> 2(二值) ...
//----------------------------------------------------------------------
	always @(posedge clk or negedge rst_n)
		if(!rst_n) mode <= MODE_DEFAULT;
		else if(combo_hit) mode <= (mode==2'd2) ? 2'd0 : (mode + 1'b1);

//----------------------------------------------------------------------
// 配置变化标志: 数据变化后延迟3拍再翻转, 保证跨时钟域时数据已稳定
//----------------------------------------------------------------------
	//	上电配置推送: 复位释放后自动推一次 {mode,threshold} 到像素时钟域,
	//	否则目的域会一直停在它自己的复位值上, 改了 THR_DEFAULT 也不会生效
	reg [4:0] pwr_cnt = 5'd0;
	always @(posedge clk or negedge rst_n)
		if(!rst_n)                pwr_cnt <= 5'd0;
		else if(pwr_cnt < 5'd31)  pwr_cnt <= pwr_cnt + 1'b1;

	reg [10:0] thr_d;
	reg [1:0]  mode_d;
	reg        chg,chg1,chg2,chg3;
	always @(posedge clk or negedge rst_n)
		if(!rst_n) begin
			thr_d<=THR_DEFAULT; mode_d<=MODE_DEFAULT;
			chg<=1'b0; chg1<=1'b0; chg2<=1'b0; chg3<=1'b0;
		end
		else begin
			thr_d <= threshold;
			mode_d<= mode;
			chg   <= (thr_d!=threshold) || (mode_d!=mode) || (pwr_cnt==5'd15);
			chg1  <= chg;
			chg2  <= chg1;
			chg3  <= chg2;
		end

	always @(posedge clk or negedge rst_n)
		if(!rst_n)        cfg_toggle <= 1'b0;
		else if(chg3)     cfg_toggle <= ~cfg_toggle;

endmodule
