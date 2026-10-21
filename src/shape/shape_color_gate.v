// 从对齐的 RGB565 提取深色图形；连续三个深色像素可抑制孤立噪点。
// 识别输入可切回原始边缘；显示通路不经过本模块。
module shape_color_gate #(
    parameter ENABLE = 1'b1,
    parameter [4:0] MAX_R = 5'd13,
    parameter [5:0] MAX_G = 6'd26,
    parameter [4:0] MAX_B = 5'd13
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid,
    input  wire        sop,
    input  wire        edge_in,
    input  wire [15:0] rgb565,
    output wire        edge_out
);
    wire dark_ink = (rgb565[15:11] <= MAX_R) &&
                    (rgb565[10:5]  <= MAX_G) &&
                    (rgb565[4:0]   <= MAX_B);
    reg dark_prev1, dark_prev2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dark_prev1 <= 1'b0;
            dark_prev2 <= 1'b0;
        end else if (valid) begin
            dark_prev1 <= dark_ink;
            dark_prev2 <= sop ? 1'b0 : dark_prev1;
        end
    end
    assign edge_out = ENABLE ? (dark_ink && dark_prev1 && dark_prev2 && !sop) : edge_in;
endmodule
