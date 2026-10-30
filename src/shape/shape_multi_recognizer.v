//==============================================================================
// 同屏三目标几何识别。
// 画面内部划为三个不可见的横向搜索区，每区独立统计一个目标；显示仍是完整画面。
// 目标编码：0未识别、1圆形、2矩形、3三角形、4平行四边形、5五角星。
//==============================================================================
`timescale 1ns/1ps

module shape_multi_recognizer #(
    parameter H_ACTIVE = 1280,
    parameter V_ACTIVE = 720,
    parameter XW = 11,
    parameter YW = 10,
    parameter CW = 24,
    parameter [CW-1:0] MIN_EDGE = 24'd180,
    parameter TURN_EPS = 4,
    parameter ROI_Y0 = 24,
    parameter ROI_Y1 = V_ACTIVE-25
)(
    input  wire clk,
    input  wire rst_n,
    input  wire st_sop,
    input  wire st_eop,
    input  wire st_valid,
    input  wire st_edge,

    output reg [2:0] shape0,
    output reg [XW-1:0] box0_x0, box0_x1,
    output reg [YW-1:0] box0_y0, box0_y1,
    output reg [2:0] shape1,
    output reg [XW-1:0] box1_x0, box1_x1,
    output reg [YW-1:0] box1_y0, box1_y1,
    output reg [2:0] shape2,
    output reg [XW-1:0] box2_x0, box2_x1,
    output reg [YW-1:0] box2_y0, box2_y1,
    output reg result_valid
);
    localparam SLOT0_X1 = H_ACTIVE/3 - 1;
    localparam SLOT1_X1 = (H_ACTIVE*2)/3 - 1;

    reg [XW-1:0] x_r;
    reg [YW-1:0] y_r;
    wire [XW-1:0] x_cur = st_sop ? {XW{1'b0}} : x_r;
    wire [YW-1:0] y_cur = st_sop ? {YW{1'b0}} : y_r;
    wire [1:0] slot_cur = (x_cur <= SLOT0_X1) ? 2'd0 :
                          (x_cur <= SLOT1_X1) ? 2'd1 : 2'd2;
    wire in_y_roi = (y_cur >= ROI_Y0) && (y_cur <= ROI_Y1);
    wire edge_now = st_valid && st_edge && in_y_roi;
    wire row_close = st_valid && (x_cur == H_ACTIVE-1);
    wire frame_end = st_valid && st_eop;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_r <= 0; y_r <= 0;
        end else if (st_valid) begin
            if (st_sop) begin
                x_r <= 1; y_r <= 0;
            end else if (x_r == H_ACTIVE-1) begin
                x_r <= 0;
                y_r <= (y_r == V_ACTIVE-1) ? 0 : y_r + 1'b1;
            end else x_r <= x_r + 1'b1;
        end
    end

    reg fe_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fe_d <= 1'b0;
        else fe_d <= frame_end;
    end

    reg row_has [0:2];
    reg [XW-1:0] row_first [0:2];
    reg [XW-1:0] row_last [0:2];
    reg acc_any [0:2];
    reg [XW-1:0] acc_minx [0:2], acc_maxx [0:2];
    reg [YW-1:0] acc_miny [0:2], acc_maxy [0:2];
    reg [CW-1:0] acc_edge [0:2], acc_fill [0:2];
    reg [CW-1:0] span_var [0:2];
    reg [CW-1:0] prev_span [0:2], first_span [0:2], last_span [0:2];
    reg [XW-1:0] first_left [0:2], last_left [0:2];
    reg [1:0] prev_dir [0:2];
    reg [3:0] width_turns [0:2];
    reg row_seen [0:2];

    reg eff_any [0:2];
    reg [XW-1:0] eff_left [0:2], eff_right [0:2];
    reg [CW-1:0] eff_span [0:2];
    integer i;
    always @(*) begin
        for (i=0; i<3; i=i+1) begin
            eff_any[i] = row_has[i] || (edge_now && (slot_cur == i));
            eff_left[i] = row_has[i] ? row_first[i] : x_cur;
            eff_right[i] = (edge_now && (slot_cur == i)) ? x_cur : row_last[i];
            eff_span[i] = eff_any[i] ? (eff_right[i] - eff_left[i] + 1'b1) : {CW{1'b0}};
        end
    end

    function [CW-1:0] abs_span;
        input [CW-1:0] a, b;
        begin abs_span = (a >= b) ? (a-b) : (b-a); end
    endfunction

    function [31:0] fill_num;
        input [CW-1:0] value;
        begin fill_num = {{(32-CW){1'b0}},value} << 8; end
    endfunction

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k=0; k<3; k=k+1) begin
                row_has[k] <= 0; row_first[k] <= 0; row_last[k] <= 0;
                acc_any[k] <= 0; acc_minx[k] <= 0; acc_maxx[k] <= 0;
                acc_miny[k] <= 0; acc_maxy[k] <= 0;
                acc_edge[k] <= 0; acc_fill[k] <= 0;
                span_var[k] <= 0; prev_span[k] <= 0; first_span[k] <= 0;
                last_span[k] <= 0; first_left[k] <= 0; last_left[k] <= 0;
                prev_dir[k] <= 0; width_turns[k] <= 0; row_seen[k] <= 0;
            end
        end else if (fe_d) begin
            for (k=0; k<3; k=k+1) begin
                row_has[k] <= 0; row_first[k] <= 0; row_last[k] <= 0;
                acc_any[k] <= 0; acc_edge[k] <= 0; acc_fill[k] <= 0;
                span_var[k] <= 0; prev_span[k] <= 0; first_span[k] <= 0;
                last_span[k] <= 0; first_left[k] <= 0; last_left[k] <= 0;
                prev_dir[k] <= 0; width_turns[k] <= 0; row_seen[k] <= 0;
            end
        end else if (st_valid) begin
            if (edge_now) begin
                acc_edge[slot_cur] <= acc_edge[slot_cur] + 1'b1;
                if (!acc_any[slot_cur]) begin
                    acc_any[slot_cur] <= 1'b1;
                    acc_minx[slot_cur] <= x_cur; acc_maxx[slot_cur] <= x_cur;
                    acc_miny[slot_cur] <= y_cur; acc_maxy[slot_cur] <= y_cur;
                end else begin
                    if (x_cur < acc_minx[slot_cur]) acc_minx[slot_cur] <= x_cur;
                    if (x_cur > acc_maxx[slot_cur]) acc_maxx[slot_cur] <= x_cur;
                    if (y_cur > acc_maxy[slot_cur]) acc_maxy[slot_cur] <= y_cur;
                end
            end

            if (row_close) begin
                for (k=0; k<3; k=k+1) begin
                    if (eff_any[k]) begin
                        acc_fill[k] <= acc_fill[k] + eff_span[k];
                        if (!row_seen[k]) begin
                            row_seen[k] <= 1'b1;
                            first_span[k] <= eff_span[k];
                            first_left[k] <= eff_left[k];
                        end else begin
                            span_var[k] <= span_var[k] + abs_span(eff_span[k], prev_span[k]);
                            if ((eff_span[k] > prev_span[k] + TURN_EPS) &&
                                (prev_dir[k] == 2'd2))
                                width_turns[k] <= width_turns[k] + 1'b1;
                            else if ((prev_span[k] > eff_span[k] + TURN_EPS) &&
                                     (prev_dir[k] == 2'd1))
                                width_turns[k] <= width_turns[k] + 1'b1;

                            if (eff_span[k] > prev_span[k] + TURN_EPS) prev_dir[k] <= 2'd1;
                            else if (prev_span[k] > eff_span[k] + TURN_EPS) prev_dir[k] <= 2'd2;
                        end
                        prev_span[k] <= eff_span[k];
                        last_span[k] <= eff_span[k];
                        last_left[k] <= eff_left[k];
                    end
                    row_has[k] <= 1'b0;
                    row_first[k] <= 0;
                    row_last[k] <= 0;
                end
            end else if (edge_now) begin
                row_has[slot_cur] <= 1'b1;
                if (!row_has[slot_cur]) begin
                    row_first[slot_cur] <= x_cur;
                    row_last[slot_cur] <= x_cur;
                end else row_last[slot_cur] <= x_cur;
            end
        end
    end

    reg snap_any [0:2];
    reg [CW-1:0] snap_edge [0:2], snap_fill [0:2], snap_span_var [0:2];
    reg [CW-1:0] snap_first_span [0:2], snap_last_span [0:2];
    reg [XW-1:0] snap_first_left [0:2], snap_last_left [0:2];
    reg [XW-1:0] snap_minx [0:2], snap_maxx [0:2];
    reg [YW-1:0] snap_miny [0:2], snap_maxy [0:2];
    reg [3:0] snap_turns [0:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            box0_x0 <= 0; box0_x1 <= 0; box0_y0 <= 0; box0_y1 <= 0;
            box1_x0 <= 0; box1_x1 <= 0; box1_y0 <= 0; box1_y1 <= 0;
            box2_x0 <= 0; box2_x1 <= 0; box2_y0 <= 0; box2_y1 <= 0;
            for (k=0; k<3; k=k+1) begin
                snap_any[k] <= 0; snap_edge[k] <= 0; snap_fill[k] <= 0;
                snap_span_var[k] <= 0; snap_first_span[k] <= 0; snap_last_span[k] <= 0;
                snap_first_left[k] <= 0; snap_last_left[k] <= 0;
                snap_minx[k] <= 0; snap_maxx[k] <= 0;
                snap_miny[k] <= 0; snap_maxy[k] <= 0; snap_turns[k] <= 0;
            end
        end else if (fe_d) begin
            for (k=0; k<3; k=k+1) begin
                snap_any[k] <= acc_any[k]; snap_edge[k] <= acc_edge[k];
                snap_fill[k] <= acc_fill[k]; snap_span_var[k] <= span_var[k];
                snap_first_span[k] <= first_span[k]; snap_last_span[k] <= last_span[k];
                snap_first_left[k] <= first_left[k]; snap_last_left[k] <= last_left[k];
                snap_minx[k] <= acc_minx[k]; snap_maxx[k] <= acc_maxx[k];
                snap_miny[k] <= acc_miny[k]; snap_maxy[k] <= acc_maxy[k];
                snap_turns[k] <= width_turns[k];
            end
            box0_x0 <= acc_minx[0]; box0_x1 <= acc_maxx[0];
            box0_y0 <= acc_miny[0]; box0_y1 <= acc_maxy[0];
            box1_x0 <= acc_minx[1]; box1_x1 <= acc_maxx[1];
            box1_y0 <= acc_miny[1]; box1_y1 <= acc_maxy[1];
            box2_x0 <= acc_minx[2]; box2_x1 <= acc_maxx[2];
            box2_y0 <= acc_miny[2]; box2_y1 <= acc_maxy[2];
        end
    end

    wire [31:0] area0 = (snap_maxx[0]-snap_minx[0]+1'b1) *
                         (snap_maxy[0]-snap_miny[0]+1'b1);
    wire [31:0] area1 = (snap_maxx[1]-snap_minx[1]+1'b1) *
                         (snap_maxy[1]-snap_miny[1]+1'b1);
    wire [31:0] area2 = (snap_maxx[2]-snap_minx[2]+1'b1) *
                         (snap_maxy[2]-snap_miny[2]+1'b1);

    reg [1:0] div_lane;
    reg [1:0] div_state;
    reg [5:0] div_count;
    reg [31:0] div_num, div_den, div_rem, div_quo;

    function [2:0] classify;
        input [15:0] ratio;
        input [CW-1:0] edge_count;
        input [3:0] turns;
        input [XW-1:0] minx, maxx, first_l, last_l;
        input [YW-1:0] miny, maxy;
        input [CW-1:0] first_s, last_s, variation;
        reg [XW:0] bw;
        reg [YW:0] bh;
        reg [XW:0] left_shift;
        reg [CW-1:0] end_delta;
        begin
            bw = maxx-minx+1'b1;
            bh = maxy-miny+1'b1;
            left_shift = (first_l >= last_l) ? (first_l-last_l) : (last_l-first_l);
            end_delta = (first_s >= last_s) ? (first_s-last_s) : (last_s-first_s);
            if (edge_count < MIN_EDGE)
                classify = 3'd0;
            else if (turns >= 2)
                classify = 3'd5;
            else if ((left_shift >= 3) && (end_delta <= (bw>>2)) &&
                     (variation <= (bh<<1)))
                classify = 3'd4;
            else if (ratio >= 16'd220)
                classify = 3'd2;
            else if (ratio >= 16'd155)
                classify = 3'd1;
            else if (ratio >= 16'd75)
                classify = 3'd3;
            else
                classify = 3'd0;
        end
    endfunction

    task load_divider;
        input [1:0] lane;
        begin
            case (lane)
                0: begin div_num <= fill_num(snap_fill[0]); div_den <= area0; end
                1: begin div_num <= fill_num(snap_fill[1]); div_den <= area1; end
                default: begin div_num <= fill_num(snap_fill[2]); div_den <= area2; end
            endcase
            div_rem <= 0; div_quo <= 0; div_count <= 32;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shape0 <= 0; shape1 <= 0; shape2 <= 0; result_valid <= 0;
            div_lane <= 0; div_state <= 0; div_count <= 0;
            div_num <= 0; div_den <= 0; div_rem <= 0; div_quo <= 0;
        end else begin
            result_valid <= 1'b0;
            case (div_state)
                0: if (fe_d) begin
                    div_lane <= 0;
                    div_state <= 1;
                    div_num <= fill_num(acc_fill[0]);
                    div_den <= (acc_maxx[0]-acc_minx[0]+1'b1) *
                               (acc_maxy[0]-acc_miny[0]+1'b1);
                    div_rem <= 0; div_quo <= 0; div_count <= 32;
                end
                1: begin
                    if (div_den == 0) begin
                        div_quo <= 0; div_state <= 2;
                    end else begin
                        if ({div_rem[30:0],div_num[31]} >= div_den) begin
                            div_rem <= {div_rem[30:0],div_num[31]} - div_den;
                            div_quo <= {div_quo[30:0],1'b1};
                        end else begin
                            div_rem <= {div_rem[30:0],div_num[31]};
                            div_quo <= {div_quo[30:0],1'b0};
                        end
                        div_num <= {div_num[30:0],1'b0};
                        div_count <= div_count-1'b1;
                        if (div_count == 1) div_state <= 2;
                    end
                end
                default: begin
                    case (div_lane)
                        0: shape0 <= snap_any[0] ? classify(div_quo[15:0],snap_edge[0],snap_turns[0],
                            snap_minx[0],snap_maxx[0],snap_first_left[0],snap_last_left[0],
                            snap_miny[0],snap_maxy[0],snap_first_span[0],snap_last_span[0],snap_span_var[0]) : 0;
                        1: shape1 <= snap_any[1] ? classify(div_quo[15:0],snap_edge[1],snap_turns[1],
                            snap_minx[1],snap_maxx[1],snap_first_left[1],snap_last_left[1],
                            snap_miny[1],snap_maxy[1],snap_first_span[1],snap_last_span[1],snap_span_var[1]) : 0;
                        2: shape2 <= snap_any[2] ? classify(div_quo[15:0],snap_edge[2],snap_turns[2],
                            snap_minx[2],snap_maxx[2],snap_first_left[2],snap_last_left[2],
                            snap_miny[2],snap_maxy[2],snap_first_span[2],snap_last_span[2],snap_span_var[2]) : 0;
                    endcase
                    if (div_lane == 2) begin
                        result_valid <= 1'b1;
                        div_state <= 0;
                    end else begin
                        div_lane <= div_lane + 1'b1;
                        load_divider(div_lane + 1'b1);
                        div_state <= 1;
                    end
                end
            endcase
        end
    end
endmodule
