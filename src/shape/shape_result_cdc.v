//==============================================================================
//  shape_result_cdc.v
//  ---------------------------------------------------------------------------
//  识别结果跨时钟域传递：摄像头像素时钟 (cmos_pclk)  ->  显示像素时钟 (clk_pixel)
//
//  识别结果每帧只更新一次，更新后至少保持一整帧时间(~16.7ms)不变，
//  因此采用「数据先锁存、再用电平翻转标志做握手」的经典做法：
//      * 源端：load 有效时把结果打进 *_s 寄存器，同时翻转 tog
//      * 目的端：tog 经两级触发器同步后做边沿检测，边沿到来时把稳定的 *_s
//                数据打入 *_b 寄存器
//  数据在 tog 翻转前已经稳定，且在下一次翻转前不会变化，因此不存在
//  多比特同时跳变带来的亚稳态/数据撕裂问题，也无需多周期路径约束。
//==============================================================================
`timescale 1ns/1ps

module shape_result_cdc #(
    parameter XW = 11,
    parameter YW = 10
)(
    // ---------------- 源端：摄像头像素时钟域 ----------------
    input  wire             clk_a,
    input  wire             rst_a_n,
    input  wire             load,           // 单周期脉冲，指示结果已稳定
    input  wire      [1:0]  shape_a,
    input  wire [XW-1:0]    x0_a, x1_a,
    input  wire [YW-1:0]    y0_a, y1_a,
    input  wire [15:0]      ratio_a,

    // ---------------- 目的端：显示像素时钟域 ----------------
    input  wire             clk_b,
    input  wire             rst_b_n,
    output reg       [1:0]  shape_b,
    output reg  [XW-1:0]    x0_b, x1_b,
    output reg  [YW-1:0]    y0_b, y1_b,
    output reg  [15:0]      ratio_b,
    output reg              valid_b
);

    // ---------------- 源端：锁存 + 翻转 ----------------
    reg  [1:0]   shape_s;
    reg  [XW-1:0] x0_s, x1_s;
    reg  [YW-1:0] y0_s, y1_s;
    reg  [15:0]  ratio_s;
    reg          tog_s;

    always @(posedge clk_a or negedge rst_a_n) begin
        if (!rst_a_n) begin
            shape_s <= 2'd0;
            x0_s <= {XW{1'b0}};
            x1_s <= {XW{1'b0}};
            y0_s <= {YW{1'b0}};
            y1_s <= {YW{1'b0}};
            ratio_s <= 16'd0;
            tog_s   <= 1'b0;
        end else if (load) begin
            shape_s <= shape_a;
            x0_s    <= x0_a;
            x1_s    <= x1_a;
            y0_s    <= y0_a;
            y1_s    <= y1_a;
            ratio_s <= ratio_a;
            tog_s   <= ~tog_s;
        end
    end

    // ---------------- 目的端：两级同步 + 边沿检测 ----------------
    reg tog_meta, tog_sync, tog_sync_d;

    always @(posedge clk_b or negedge rst_b_n) begin
        if (!rst_b_n) begin
            tog_meta   <= 1'b0;
            tog_sync   <= 1'b0;
            tog_sync_d <= 1'b0;
        end else begin
            tog_meta   <= tog_s;
            tog_sync   <= tog_meta;
            tog_sync_d <= tog_sync;
        end
    end

    wire new_result = (tog_sync ^ tog_sync_d);   // 任一边沿都表示有新结果

    always @(posedge clk_b or negedge rst_b_n) begin
        if (!rst_b_n) begin
            shape_b <= 2'd0;
            x0_b <= {XW{1'b0}};
            x1_b <= {XW{1'b0}};
            y0_b <= {YW{1'b0}};
            y1_b <= {YW{1'b0}};
            ratio_b <= 16'd0;
            valid_b <= 1'b0;
        end else begin
            valid_b <= new_result;
            if (new_result) begin
                shape_b <= shape_s;
                x0_b    <= x0_s;
                x1_b    <= x1_s;
                y0_b    <= y0_s;
                y1_b    <= y1_s;
                ratio_b <= ratio_s;
            end
        end
    end

endmodule
