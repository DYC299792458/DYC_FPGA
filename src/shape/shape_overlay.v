//==============================================================================
//  shape_overlay.v
//  ---------------------------------------------------------------------------
//  识别结果显示叠加层（赛题四 高阶挑战 6：将识别结果显示在屏幕上）
//
//  工作在显示像素时钟域 (clk_pixel)，在 LCD/HDMI 输出通路上实时叠加：
//      1) 用彩色矩形框标出被识别目标的包围盒
//      2) 在画面左上角绘制中文提示，如「检测到圆形」「检测到矩形」
//         「检测到三角形」「未识别」
//
//  中文点阵取自 font_cn_rom.v（16x16 宋体，共 13 个字模）。
//  文字按 S 倍放大，S 必须是 2 的幂(1/2/4)，这样字模寻址只需移位和掩码，
//  完全不消耗除法器或乘法器资源。
//
//  叠加层不关心底图是怎么来的：它接收显示端已经算好的 24bit RGB888（base），
//  只对命中像素改写颜色。因此无论是纯灰度图、队友的分屏彩色叠加结果，
//  都不需要改动 DDR 帧缓存的数据宽度。
//==============================================================================
`timescale 1ns/1ps

module shape_overlay #(
    parameter S          = 2,       // 文字放大倍数，必须是 2 的幂：1/2/4
    parameter X_SHIFT    = 1,       // lcd_xpos 比图像列号大 1（见 lcd_driver 的 H_AHEAD）
    parameter LABEL_X    = 24,      // 提示文字左上角（图像坐标）
    parameter LABEL_Y    = 24,
    parameter PAD        = 6,       // 文字底板外扩
    parameter BOX_THICK  = 2,       // 包围盒线宽
    parameter MAX_CHARS  = 6        // 最长提示的字数（检测到三角形 = 6）
)(
    input  wire         clk,
    input  wire         rst_n,

    // 显示像素流（来自 lcd_driver / DDR 读回）
    input  wire         de,             // 数据有效
    input  wire [11:0]  xpos,           // lcd_driver 输出的列坐标
    input  wire [11:0]  ypos,           // lcd_driver 输出的行坐标
    input  wire [23:0]  base,           // 底图 RGB888（分屏/彩色叠加后的结果）
                                        // 纯灰度场景传 {gray,gray,gray} 即可

    // 来自 shape_result_cdc 的识别结果（显示时钟域）
    input  wire [1:0]   shape,          // 0=未识别 1=圆形 2=矩形 3=三角形
    input  wire [11:0]  bx0, bx1,
    input  wire [11:0]  by0, by1,
    input  wire         valid,          // 首次出现有效结果后置位

    output reg  [23:0]  rgb             // 叠加后的 RGB888
);

    localparam GSZ = 16 * S;                        // 单个字模的像素尺寸
    localparam SLG = (S == 1) ? 0 : ((S == 2) ? 1 : 2);   // log2(S)

    //--------------------------------------------------------------------------
    // 1. 结果有效标志（粘滞）
    //--------------------------------------------------------------------------
    reg have_result;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            have_result <= 1'b0;
        else if (valid)        have_result <= 1'b1;
    end

    //--------------------------------------------------------------------------
    // 2. 坐标对齐：把 lcd_xpos 换算成图像列号
    //--------------------------------------------------------------------------
    wire [11:0] px = xpos - X_SHIFT;
    wire [11:0] py = ypos;
    wire        en = de && have_result;

    //--------------------------------------------------------------------------
    // 3. 包围盒边框
    //--------------------------------------------------------------------------
    wire in_box = (px >= bx0) && (px <= bx1) && (py >= by0) && (py <= by1);

    wire on_border = in_box &&
                     (((px - bx0) < BOX_THICK) || ((bx1 - px) < BOX_THICK) ||
                      ((py - by0) < BOX_THICK) || ((by1 - py) < BOX_THICK));

    //--------------------------------------------------------------------------
    // 4. 中文提示：字模索引
    //--------------------------------------------------------------------------
    wire [11:0] txr = px - LABEL_X;
    wire [11:0] tyr = py - LABEL_Y;

    wire in_text_x = (px >= LABEL_X) && (px < LABEL_X + MAX_CHARS * GSZ);
    wire in_text_y = (py >= LABEL_Y) && (py < LABEL_Y + GSZ);

    wire [3:0]  gidx = txr[11:0] >> (4 + SLG);              // 第几个字
    wire [3:0]  gcol = (txr >> SLG) & 4'hF;                 // 字内列
    wire [3:0]  grow = (tyr >> SLG) & 4'hF;                 // 字内行

    // 每个类别对应的提示语
    reg [3:0] msg_len;
    always @(*) begin
        case (shape)
            2'd0:    msg_len = 4'd3;    // 未识别
            2'd1:    msg_len = 4'd5;    // 检测到圆形
            2'd2:    msg_len = 4'd5;    // 检测到矩形
            default: msg_len = 4'd6;    // 检测到三角形
        endcase
    end

    // {shape, 第几个字} -> 字模编号（见 font_cn_rom.v 的索引表）
    reg [3:0] msg_gl;
    always @(*) begin
        case ({shape, gidx})
            // 未识别 : 未(6) 识(7) 别(8)
            6'd0:   msg_gl = 4'd6;
            6'd1:   msg_gl = 4'd7;
            6'd2:   msg_gl = 4'd8;
            // 检测到圆形 : 检(0) 测(1) 到(2) 圆(3) 形(4)
            6'd16:  msg_gl = 4'd0;
            6'd17:  msg_gl = 4'd1;
            6'd18:  msg_gl = 4'd2;
            6'd19:  msg_gl = 4'd3;
            6'd20:  msg_gl = 4'd4;
            // 检测到矩形 : 检(0) 测(1) 到(2) 矩(5) 形(4)
            6'd32:  msg_gl = 4'd0;
            6'd33:  msg_gl = 4'd1;
            6'd34:  msg_gl = 4'd2;
            6'd35:  msg_gl = 4'd5;
            6'd36:  msg_gl = 4'd4;
            // 检测到三角形 : 检(0) 测(1) 到(2) 三(9) 角(10) 形(4)
            6'd48:  msg_gl = 4'd0;
            6'd49:  msg_gl = 4'd1;
            6'd50:  msg_gl = 4'd2;
            6'd51:  msg_gl = 4'd9;
            6'd52:  msg_gl = 4'd10;
            6'd53:  msg_gl = 4'd4;
            default: msg_gl = 4'd0;
        endcase
    end

    wire [15:0] font_bits;
    font_cn_rom u_font_cn_rom (
        .char_idx (msg_gl),
        .row      (grow),
        .bits     (font_bits)
    );

    wire glyph_active = (gidx < msg_len);
    wire font_pix     = font_bits[15 - gcol];
    wire text_pix     = en && in_text_x && in_text_y && glyph_active && font_pix;

    //--------------------------------------------------------------------------
    // 5. 文字底板（按提示语实际长度收放）
    //--------------------------------------------------------------------------
    wire [11:0] panel_w = (msg_len << (4 + SLG));           // msg_len * GSZ

    wire in_panel = en &&
                    (px >= LABEL_X - PAD) && (px < LABEL_X + panel_w + PAD) &&
                    (py >= LABEL_Y - PAD) && (py < LABEL_Y + GSZ + PAD);

    //--------------------------------------------------------------------------
    // 6. 类别配色
    //--------------------------------------------------------------------------
    reg [23:0] c_shape;
    always @(*) begin
        case (shape)
            2'd1:    c_shape = 24'h00FFFF;      // 圆形   —— 青
            2'd2:    c_shape = 24'h00FF00;      // 矩形   —— 绿
            2'd3:    c_shape = 24'hFFFF00;      // 三角形 —— 黄
            default: c_shape = 24'hFF4040;      // 未识别 —— 红
        endcase
    end

    //--------------------------------------------------------------------------
    // 7. 叠加输出（优先级：文字 > 底板 > 边框 > 底图）
    //--------------------------------------------------------------------------
    always @(*) begin
        if (!en)
            rgb = base;
        else if (text_pix)
            rgb = c_shape;
        else if (in_panel)
            rgb = 24'h101018;
        else if (on_border)
            rgb = c_shape;
        else
            rgb = base;
    end

endmodule
