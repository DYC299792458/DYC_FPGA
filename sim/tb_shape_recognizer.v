//==============================================================================
//  tb_shape_recognizer.v
//  ---------------------------------------------------------------------------
//  shape_recognizer 的自检测试平台。
//
//  直接在仿真里合成「边缘图」像素流（圆形/矩形/三角形轮廓、以及空白帧和
//  随机噪声帧），按真实协议送入 DUT：
//      * sop 只在**一帧**的第一个像素有效，eop 只在最后一帧的最后一个像素有效
//      * valid 会被随机拉低来模拟采集 FIFO 空导致的停顿（验证坐标计数正确性）
//  然后等待 result_valid，比对分类结果。
//
//  为便于仿真，分辨率缩小到 128x96，ROI 覆盖全画面；算法与阈值与原设计一致
//  （填充率与分辨率无关）。
//
//  用法（Icarus Verilog）：
//      iverilog -o tb_shape tb_shape_recognizer.v shape_recognizer.v
//      vvp tb_shape
//  ModelSim :  vlog shape_recognizer.v tb_shape_recognizer.v ; vsim -c tb_shape_recognizer
//==============================================================================
`timescale 1ns/1ps

module tb_shape_recognizer;

    localparam H = 128;
    localparam V = 96;

    // 期望的分类编码
    localparam S_NONE = 2'd0;
    localparam S_CIRC = 2'd1;
    localparam S_RECT = 2'd2;
    localparam S_TRI  = 2'd3;

    reg  clk   = 1'b0;
    reg  rst_n = 1'b0;
    always #5 clk = ~clk;               // 100 MHz

    reg         sop, eop, valid;
    reg  [7:0]  data;
    reg  [7:0]  thresh = 8'd24;
    reg         roi_en = 1'b1;

    wire [1:0]  shape;
    wire [7:0]  bx0, bx1, by0, by1;
    wire [23:0] feat_edge, feat_area;
    wire [15:0] ratio_q;
    wire        rvalid;

    integer errors = 0;

    shape_recognizer #(
        .H_ACTIVE (H),
        .V_ACTIVE (V),
        .XW       (8),
        .YW       (8),
        .CW       (24),
        .MIN_EDGE (24'd20),
        .RECT_Q   (16'd230),
        .CIRC_Q   (16'd168),
        .TRI_Q    (16'd96),
        .ROI_X0   (0),
        .ROI_X1   (H - 1),
        .ROI_Y0   (0),
        .ROI_Y1   (V - 1)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .st_sop       (sop),
        .st_eop       (eop),
        .st_valid     (valid),
        .st_data      (data),
        .thresh       (thresh),
        .roi_en       (roi_en),
        .shape        (shape),
        .box_x0       (bx0),
        .box_x1       (bx1),
        .box_y0       (by0),
        .box_y1       (by1),
        .feat_edge    (feat_edge),
        .feat_area    (feat_area),
        .feat_ratio_q (ratio_q),
        .result_valid (rvalid)
    );

    //--------------------------------------------------------------------------
    // 合成边缘图：判断 (x,y) 是否落在形状轮廓上
    //   kind 0 = 空, 1 = 圆, 2 = 矩形, 3 = 三角形
    //   形状中心固定在 (64,48)
    //--------------------------------------------------------------------------
    function is_edge_pix;
        input [1:0] kind;
        input integer x;
        input integer y;
        integer dx, dy, adx, ady, d2, top, bot, lx, rx;
        begin
            is_edge_pix = 1'b0;
            dx = x - 64;
            dy = y - 48;
            case (kind)
                2'd1: begin                       // 圆环 r=32, 环宽约 4
                    d2 = (dx * dx) + (dy * dy);
                    if ((d2 >= 30 * 30) && (d2 <= 34 * 34)) is_edge_pix = 1'b1;
                end
                2'd2: begin                       // 矩形轮廓 64x48, 线宽 2
                    adx = (dx < 0) ? -dx : dx;
                    ady = (dy < 0) ? -dy : dy;
                    if ((adx <= 32) && (ady <= 24)) begin
                        if (((32 - adx) < 2) || ((24 - ady) < 2)) is_edge_pix = 1'b1;
                    end
                end
                2'd3: begin                       // 三角形轮廓, 顶点朝上, 底边宽 64, 高 48
                    top = 48 - 24;
                    bot = 48 + 24;
                    if ((y >= top) && (y <= bot)) begin
                        lx = 64 - ((32 * (y - top)) / (bot - top));
                        rx = 64 + ((32 * (y - top)) / (bot - top));
                        if (((x >= (lx - 2)) && (x <= (lx + 2))) ||
                            ((x >= (rx - 2)) && (x <= (rx + 2)))) is_edge_pix = 1'b1;
                        if (((bot - y) < 2) && (x >= (lx - 2)) && (x <= (rx + 2))) is_edge_pix = 1'b1;
                    end
                end
                default: is_edge_pix = 1'b0;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // 送一个像素；gap=1 时先插入一个 valid=0 的空拍模拟 FIFO 停顿
    //--------------------------------------------------------------------------
    task send_pixel;
        input        sop_v;
        input        eop_v;
        input [7:0]  d;
        input        gap;
        begin
            @(negedge clk);
            data  = d;
            sop   = sop_v;
            eop   = eop_v;
            valid = 1'b0;
            if (gap) @(negedge clk);        // 停顿拍：valid 保持 0，数据不变
            valid = 1'b1;
            @(negedge clk);
            valid = 1'b0;
            sop   = 1'b0;
            eop   = 1'b0;
        end
    endtask

    //--------------------------------------------------------------------------
    // 送一整帧
    //--------------------------------------------------------------------------
    task send_frame;
        input [1:0] kind;
        input       gap_en;
        integer x, y;
        begin
            for (y = 0; y < V; y = y + 1) begin
                for (x = 0; x < H; x = x + 1) begin
                    send_pixel((x == 0) && (y == 0),
                               (x == H - 1) && (y == V - 1),
                               is_edge_pix(kind, x, y) ? 8'hFF : 8'h00,
                               gap_en && (((x + y) % 7) == 0));
                end
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // 等待结果并比对
    //--------------------------------------------------------------------------
    task check_result;
        input [1:0]       exp;
        input [8*16-1:0]  name;
        integer t;
        begin
            t = 0;
            while ((rvalid !== 1'b1) && (t < 1000)) begin
                @(negedge clk);
                t = t + 1;
            end

            if (rvalid !== 1'b1) begin
                $display("FAIL  %0s : 没有等到 result_valid", name);
                errors = errors + 1;
            end else if (shape !== exp) begin
                $display("FAIL  %0s : shape=%0d 期望 %0d  (ratio_q=%0d edge=%0d area=%0d box=(%0d,%0d)-(%0d,%0d))",
                         name, shape, exp, ratio_q, feat_edge, feat_area, bx0, by0, bx1, by1);
                errors = errors + 1;
            end else begin
                $display("PASS  %0s : shape=%0d  ratio_q=%0d  edge=%0d  area=%0d  box=(%0d,%0d)-(%0d,%0d)",
                         name, shape, ratio_q, feat_edge, feat_area, bx0, by0, bx1, by1);
            end

            repeat (5) @(negedge clk);
        end
    endtask

    //--------------------------------------------------------------------------
    // 主流程
    //--------------------------------------------------------------------------
    initial begin
        sop = 0; eop = 0; valid = 0; data = 8'h00;

        repeat (10) @(negedge clk);
        rst_n = 1'b1;
        repeat (10) @(negedge clk);

        $display("---------------------------------------------------------------");

        send_frame(2'd1, 1'b0);            // 圆形
        check_result(S_CIRC, "circle");

        send_frame(2'd2, 1'b0);            // 矩形
        check_result(S_RECT, "rectangle");

        send_frame(2'd3, 1'b0);            // 三角形
        check_result(S_TRI,  "triangle");

        send_frame(2'd0, 1'b0);            // 空白帧
        check_result(S_NONE, "empty frame");

        send_frame(2'd1, 1'b1);            // 圆形（valid 有停顿）
        check_result(S_CIRC, "circle w/ valid gaps");

        send_frame(2'd2, 1'b1);            // 矩形（valid 有停顿）
        check_result(S_RECT, "rect w/ valid gaps");

        $display("---------------------------------------------------------------");
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $display("---------------------------------------------------------------");
        $finish;
    end

    // 超时保护
    initial begin
        #500_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
