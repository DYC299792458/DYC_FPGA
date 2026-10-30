`timescale 1ns/1ps

// 多目标识别自检：同一帧三个横向分离目标，显示仍为完整画面。
module tb_shape_multi_recognizer;
    localparam H = 96;
    localparam V = 72;

    localparam [2:0] S_NONE = 3'd0;
    localparam [2:0] S_CIRC = 3'd1;
    localparam [2:0] S_RECT = 3'd2;
    localparam [2:0] S_TRI  = 3'd3;
    localparam [2:0] S_PARA = 3'd4;
    localparam [2:0] S_STAR = 3'd5;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg sop, eop, valid, edge_in;
    wire [2:0] shape0, shape1, shape2;
    wire [7:0] x00, x01, y00, y01;
    wire [7:0] x10, x11, y10, y11;
    wire [7:0] x20, x21, y20, y21;
    wire result_valid;
    integer errors = 0;

    shape_multi_recognizer #(
        .H_ACTIVE(H), .V_ACTIVE(V), .XW(8), .YW(8), .CW(20),
        .MIN_EDGE(20), .TURN_EPS(2)
    ) dut (
        .clk(clk), .rst_n(rst_n), .st_sop(sop), .st_eop(eop),
        .st_valid(valid), .st_edge(edge_in),
        .shape0(shape0), .box0_x0(x00), .box0_x1(x01), .box0_y0(y00), .box0_y1(y01),
        .shape1(shape1), .box1_x0(x10), .box1_x1(x11), .box1_y0(y10), .box1_y1(y11),
        .shape2(shape2), .box2_x0(x20), .box2_x1(x21), .box2_y0(y20), .box2_y1(y21),
        .result_valid(result_valid)
    );

    function edge_for_kind;
        input integer kind, lane, x, y;
        integer cx, dx, dy, adx, ady, d2, top_y, bot_y;
        integer left_x, right_x, half_w, ay;
        begin
            cx = lane * 32 + 16;
            dx = x - cx;
            dy = y - 36;
            adx = (dx < 0) ? -dx : dx;
            ady = (dy < 0) ? -dy : dy;
            edge_for_kind = 0;
            case (kind)
                1: begin // 圆
                    d2 = dx*dx + dy*dy;
                    edge_for_kind = (d2 >= 11*11) && (d2 <= 13*13);
                end
                2: begin // 矩形
                    edge_for_kind = (adx <= 11) && (ady <= 15) &&
                                    ((adx >= 10) || (ady >= 14));
                end
                3: begin // 三角形
                    top_y = 20; bot_y = 52;
                    if ((y >= top_y) && (y <= bot_y)) begin
                        half_w = (12 * (y - top_y)) / (bot_y - top_y);
                        edge_for_kind = (x == cx-half_w) || (x == cx+half_w) ||
                                        ((y >= bot_y-1) && (x >= cx-half_w) && (x <= cx+half_w));
                    end
                end
                4: begin // 平行四边形
                    top_y = 20; bot_y = 52;
                    left_x = cx - 12 + ((y-top_y) / 4);
                    right_x = left_x + 20;
                    if ((y >= top_y) && (y <= bot_y))
                        edge_for_kind = (x == left_x) || (x == right_x) ||
                                        (((y == top_y) || (y == bot_y)) && (x >= left_x) && (x <= right_x));
                end
                5: begin // 五角星的外包络：宽度沿纵向多次反转
                    ay = y - 18;
                    if ((ay >= 0) && (ay <= 36)) begin
                        if      (ay <= 7)  half_w = 1 + ay;
                        else if (ay <= 13) half_w = 20 - ay;
                        else if (ay <= 18) half_w = 7 + (ay-13)*2;
                        else if (ay <= 25) half_w = 17 - (ay-18);
                        else               half_w = 10 - ((ay-25)*8/11);
                        edge_for_kind = (x == cx-half_w) || (x == cx+half_w);
                    end
                end
            endcase
        end
    endfunction

    task send_frame;
        input integer k0, k1, k2;
        integer x, y, k;
        begin
            for (y=0; y<V; y=y+1) begin
                for (x=0; x<H; x=x+1) begin
                    k = (x < 32) ? k0 : ((x < 64) ? k1 : k2);
                    @(negedge clk);
                    sop = (x==0 && y==0);
                    eop = (x==H-1 && y==V-1);
                    edge_in = edge_for_kind(k, x/32, x, y);
                    valid = 1;
                end
            end
            @(negedge clk); valid=0; sop=0; eop=0; edge_in=0;
        end
    endtask

    task check3;
        input [2:0] e0, e1, e2;
        input [8*24-1:0] name;
        integer t;
        begin
            t=0;
            while (!result_valid && t<500) begin @(negedge clk); t=t+1; end
            if (!result_valid || shape0!==e0 || shape1!==e1 || shape2!==e2) begin
                $display("FAIL %0s got=%0d,%0d,%0d expected=%0d,%0d,%0d", name,
                         shape0,shape1,shape2,e0,e1,e2);
                errors=errors+1;
            end else $display("PASS %0s shapes=%0d,%0d,%0d",name,shape0,shape1,shape2);
            repeat(4) @(negedge clk);
        end
    endtask

    initial begin
        sop=0; eop=0; valid=0; edge_in=0;
        repeat(8) @(negedge clk); rst_n=1; repeat(4) @(negedge clk);
        send_frame(1,4,5); check3(S_CIRC,S_PARA,S_STAR,"circle parallelogram star");
        send_frame(3,2,0); check3(S_TRI,S_RECT,S_NONE,"triangle rectangle empty");
        if (errors==0) $display("ALL TESTS PASSED");
        else $display("%0d TEST(S) FAILED",errors);
        $finish;
    end
endmodule
