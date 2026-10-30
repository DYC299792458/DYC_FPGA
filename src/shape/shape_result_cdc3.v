`timescale 1ns/1ps

// 单个多类别目标结果的跨时钟传递；三个目标各实例化一次。
module shape_result_cdc3 #(
    parameter XW=11,
    parameter YW=10
)(
    input wire clk_a, rst_a_n, load,
    input wire [2:0] shape_a,
    input wire [XW-1:0] x0_a, x1_a,
    input wire [YW-1:0] y0_a, y1_a,
    input wire clk_b, rst_b_n,
    output reg [2:0] shape_b,
    output reg [XW-1:0] x0_b, x1_b,
    output reg [YW-1:0] y0_b, y1_b,
    output reg valid_b
);
    reg [2:0] shape_s;
    reg [XW-1:0] x0_s, x1_s;
    reg [YW-1:0] y0_s, y1_s;
    reg tog_s;

    always @(posedge clk_a or negedge rst_a_n) begin
        if (!rst_a_n) begin
            shape_s<=0; x0_s<=0; x1_s<=0; y0_s<=0; y1_s<=0; tog_s<=0;
        end else if (load) begin
            shape_s<=shape_a; x0_s<=x0_a; x1_s<=x1_a;
            y0_s<=y0_a; y1_s<=y1_a; tog_s<=~tog_s;
        end
    end

    reg tog_meta, tog_sync, tog_d;
    wire new_result = tog_sync ^ tog_d;
    always @(posedge clk_b or negedge rst_b_n) begin
        if (!rst_b_n) begin
            tog_meta<=0; tog_sync<=0; tog_d<=0;
        end else begin
            tog_meta<=tog_s; tog_sync<=tog_meta; tog_d<=tog_sync;
        end
    end

    always @(posedge clk_b or negedge rst_b_n) begin
        if (!rst_b_n) begin
            shape_b<=0; x0_b<=0; x1_b<=0; y0_b<=0; y1_b<=0; valid_b<=0;
        end else begin
            valid_b<=new_result;
            if (new_result) begin
                shape_b<=shape_s; x0_b<=x0_s; x1_b<=x1_s;
                y0_b<=y0_s; y1_b<=y1_s;
            end
        end
    end
endmodule
