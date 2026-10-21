`timescale 1ns/1ps

// 深色图形放在浅色纸张/蓝色背景上时，检验背景边缘不会改变分类。
module tb_shape_color_gate;
    localparam H = 128, V = 96;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg sop = 0, eop = 0, valid = 0, raw_edge = 0;
    reg [15:0] rgb = 16'h001f;
    wire filtered_edge;
    wire [1:0] shape, raw_shape;
    wire result_valid, raw_valid;
    wire [15:0] ratio, raw_ratio;

    shape_color_gate u_gate (
        .clk(clk), .rst_n(rst_n), .valid(valid), .sop(sop),
        .edge_in(raw_edge), .rgb565(rgb), .edge_out(filtered_edge));

    shape_recognizer #(
        .H_ACTIVE(H), .V_ACTIVE(V), .XW(8), .YW(8), .CW(24),
        .MIN_EDGE(24'd20), .ROI_X0(0), .ROI_X1(H-1),
        .ROI_Y0(0), .ROI_Y1(V-1)) u_filtered (
        .clk(clk), .rst_n(rst_n), .st_sop(sop), .st_eop(eop),
        .st_valid(valid), .st_data({8{filtered_edge}}),
        .thresh(8'h80), .roi_en(1'b1), .shape(shape),
        .feat_ratio_q(ratio), .result_valid(result_valid));

    shape_recognizer #(
        .H_ACTIVE(H), .V_ACTIVE(V), .XW(8), .YW(8), .CW(24),
        .MIN_EDGE(24'd20), .ROI_X0(0), .ROI_X1(H-1),
        .ROI_Y0(0), .ROI_Y1(V-1)) u_raw (
        .clk(clk), .rst_n(rst_n), .st_sop(sop), .st_eop(eop),
        .st_valid(valid), .st_data({8{raw_edge}}),
        .thresh(8'h80), .roi_en(1'b1), .shape(raw_shape),
        .feat_ratio_q(raw_ratio), .result_valid(raw_valid));

    function shape_pixel;
        input [1:0] kind;
        input integer x, y;
        integer dx, dy, ax, ay, d2, lx, rx;
        begin
            dx = x - 64; dy = y - 48;
            ax = dx < 0 ? -dx : dx;
            ay = dy < 0 ? -dy : dy;
            shape_pixel = 0;
            case (kind)
                1: begin
                    d2 = dx*dx + dy*dy;
                    shape_pixel = (d2 >= 900 && d2 <= 1156);
                end
                2: shape_pixel = (ax <= 32 && ay <= 24 &&
                                  (32-ax < 2 || 24-ay < 2));
                3: if (y >= 24 && y <= 72) begin
                    lx = 64 - (32*(y-24))/48;
                    rx = 64 + (32*(y-24))/48;
                    shape_pixel = ((x >= lx-2 && x <= lx+2) ||
                                   (x >= rx-2 && x <= rx+2) ||
                                   (y >= 71 && x >= lx-2 && x <= rx+2));
                end
            endcase
        end
    endfunction

    function target_fill;
        input [1:0] kind;
        input integer x, y;
        integer dx, dy, lx, rx;
        begin
            dx = x - 64; dy = y - 48;
            target_fill = 0;
            case (kind)
                1: target_fill = (dx*dx + dy*dy <= 1024);
                2: target_fill = (dx >= -32 && dx <= 32 && dy >= -24 && dy <= 24);
                3: if (y >= 24 && y <= 72) begin
                    lx = 64 - (32*(y-24))/48;
                    rx = 64 + (32*(y-24))/48;
                    target_fill = (x >= lx && x <= rx);
                end
            endcase
        end
    endfunction

    task send_scene;
        input [1:0] kind;
        integer x, y;
        reg target, paper_edge, filled;
        begin
            for (y = 0; y < V; y = y + 1)
                for (x = 0; x < H; x = x + 1) begin
                    target = shape_pixel(kind, x, y);
                    filled = target_fill(kind, x, y);
                    paper_edge = (x == 16 && ((y >= 10 && y <= 20) ||
                                              (y >= 76 && y <= 86)));
                    @(negedge clk);
                    valid = 1;
                    sop = (x == 0 && y == 0);
                    eop = (x == H-1 && y == V-1);
                    raw_edge = target || paper_edge;
                    rgb = filled ? 16'h2104 :
                          paper_edge ? 16'hffff : 16'h001f;
                end
            @(negedge clk);
            valid = 0; sop = 0; eop = 0; raw_edge = 0;
        end
    endtask

    task expect_shape;
        input [1:0] expected;
        integer waited;
        begin
            waited = 0;
            while (!result_valid && waited < 100) begin
                @(negedge clk);
                waited = waited + 1;
            end
            if (!result_valid || shape !== expected) begin
                $display("FAIL: expected=%0d got=%0d ratio=%0d", expected, shape, ratio);
                $fatal(1);
            end
            if ((expected == 1 || expected == 2) &&
                (!raw_valid || raw_shape !== 3)) begin
                $display("FAIL: background edge did not reproduce triangle misclassification");
                $fatal(1);
            end
            $display("PASS: expected=%0d got=%0d ratio=%0d raw=%0d raw_ratio=%0d",
                     expected, shape, ratio, raw_shape, raw_ratio);
            repeat (4) @(negedge clk);
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;
        send_scene(1); expect_shape(1);
        send_scene(2); expect_shape(2);
        send_scene(3); expect_shape(3);
        send_scene(0); expect_shape(0);
        $display("ALL COLOR GATE TESTS PASSED");
        $finish;
    end
endmodule
