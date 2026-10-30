`timescale 1ns/1ps

// 三目标叠加：完整画面上绘制三个独立框，并在框上方标注类别。
module shape_overlay_multi #(
    parameter S=1,
    parameter X_SHIFT=0,
    parameter BOX_THICK=3,
    parameter PAD=3
)(
    input wire clk, rst_n, de,
    input wire [11:0] xpos, ypos,
    input wire [23:0] base,
    input wire [2:0] shape0,
    input wire [11:0] bx00, bx01, by00, by01,
    input wire [2:0] shape1,
    input wire [11:0] bx10, bx11, by10, by11,
    input wire [2:0] shape2,
    input wire [11:0] bx20, bx21, by20, by21,
    input wire valid,
    output reg [23:0] rgb
);
    localparam GSZ = 16*S;
    localparam SLG = (S==1) ? 0 : ((S==2) ? 1 : 2);
    wire [11:0] px = xpos-X_SHIFT;
    wire [11:0] py = ypos;

    reg have_result;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) have_result<=0;
        else if (valid) have_result<=1;
    end
    wire en = de && have_result;

    function border_hit;
        input [2:0] s;
        input [11:0] x0,x1,y0,y1;
        begin
            border_hit = (s!=0) && (px>=x0) && (px<=x1) && (py>=y0) && (py<=y1) &&
                (((px-x0)<BOX_THICK) || ((x1-px)<BOX_THICK) ||
                 ((py-y0)<BOX_THICK) || ((y1-py)<BOX_THICK));
        end
    endfunction

    function [3:0] name_len;
        input [2:0] s;
        begin
            case(s)
                1,2: name_len=2;
                3,5: name_len=3;
                4: name_len=5;
                default: name_len=0;
            endcase
        end
    endfunction

    function [23:0] shape_color;
        input [2:0] s;
        begin
            case(s)
                1: shape_color=24'h00FFFF;
                2: shape_color=24'h00FF40;
                3: shape_color=24'hFFFF00;
                4: shape_color=24'hFF40FF;
                5: shape_color=24'hFF8000;
                default: shape_color=24'hFF4040;
            endcase
        end
    endfunction

    wire b0 = border_hit(shape0,bx00,bx01,by00,by01);
    wire b1 = border_hit(shape1,bx10,bx11,by10,by11);
    wire b2 = border_hit(shape2,bx20,bx21,by20,by21);

    wire [11:0] ly0 = (by00 > GSZ+PAD+2) ? by00-GSZ-PAD-2 : by00+PAD;
    wire [11:0] ly1 = (by10 > GSZ+PAD+2) ? by10-GSZ-PAD-2 : by10+PAD;
    wire [11:0] ly2 = (by20 > GSZ+PAD+2) ? by20-GSZ-PAD-2 : by20+PAD;
    wire [11:0] lx0 = bx00;
    wire [11:0] lx1 = bx10;
    wire [11:0] lx2 = bx20;
    wire [3:0] nl0=name_len(shape0), nl1=name_len(shape1), nl2=name_len(shape2);

    wire p0 = (shape0!=0) && px>=lx0-PAD && px<lx0+(nl0<< (4+SLG))+PAD &&
              py>=ly0-PAD && py<ly0+GSZ+PAD;
    wire p1 = (shape1!=0) && px>=lx1-PAD && px<lx1+(nl1<< (4+SLG))+PAD &&
              py>=ly1-PAD && py<ly1+GSZ+PAD;
    wire p2 = (shape2!=0) && px>=lx2-PAD && px<lx2+(nl2<< (4+SLG))+PAD &&
              py>=ly2-PAD && py<ly2+GSZ+PAD;

    reg [1:0] sel;
    reg [2:0] sel_shape;
    reg [11:0] sel_lx, sel_ly;
    reg [3:0] sel_len;
    always @(*) begin
        sel=3; sel_shape=0; sel_lx=0; sel_ly=0; sel_len=0;
        if (p0) begin sel=0; sel_shape=shape0; sel_lx=lx0; sel_ly=ly0; sel_len=nl0; end
        else if (p1) begin sel=1; sel_shape=shape1; sel_lx=lx1; sel_ly=ly1; sel_len=nl1; end
        else if (p2) begin sel=2; sel_shape=shape2; sel_lx=lx2; sel_ly=ly2; sel_len=nl2; end
    end

    wire [11:0] tx = px-sel_lx;
    wire [11:0] ty = py-sel_ly;
    wire [3:0] gidx = tx >> (4+SLG);
    wire [3:0] gcol = (tx >> SLG) & 4'hf;
    wire [3:0] grow = (ty >> SLG) & 4'hf;
    reg [4:0] glyph;
    always @(*) begin
        glyph=0;
        case ({sel_shape,gidx})
            7'h10: glyph=3;  7'h11: glyph=4;                 // 圆形
            7'h20: glyph=5;  7'h21: glyph=4;                 // 矩形
            7'h30: glyph=9;  7'h31: glyph=10; 7'h32: glyph=4;// 三角形
            7'h40: glyph=13; 7'h41: glyph=14; 7'h42: glyph=15;
            7'h43: glyph=16; 7'h44: glyph=4;                 // 平行四边形
            7'h50: glyph=17; 7'h51: glyph=10; 7'h52: glyph=18;// 五角星
            default: glyph=0;
        endcase
    end
    wire [15:0] font_bits;
    font_cn_rom u_font(.char_idx(glyph),.row(grow),.bits(font_bits));
    wire text_area = (sel!=3) && px>=sel_lx && py>=sel_ly &&
                     py<sel_ly+GSZ && gidx<sel_len;
    wire text_pix = text_area && font_bits[15-gcol];
    wire [23:0] label_color=shape_color(sel_shape);

    always @(*) begin
        if (!en) rgb=base;
        else if (text_pix) rgb=label_color;
        else if (p0 || p1 || p2) rgb=24'h101018;
        else if (b0) rgb=shape_color(shape0);
        else if (b1) rgb=shape_color(shape1);
        else if (b2) rgb=shape_color(shape2);
        else rgb=base;
    end
endmodule
