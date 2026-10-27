`timescale 1ns/1ps
//============================================================================
// osd_overlay.v
//   独立 OSD 图层（HDMI 像素时钟域），插在 visual_alg 与 hdmi_tx 之间。
//   - 叠加“安路”Logo（16x16 点阵 x2）
//   - 标题：FPGA BALL CONTROL SYSTEM
//   - MODE   : 当前算法
//   - BALL X : 球心 X 像素坐标
//   - SPEED  : 球速（px/s，带符号）
//   - STATE  : VALID / INVALID
//   - 球心十字准星
//   时序优化：动态数值（球X、速度、模式名、状态名）用“预解码寄存器”登记，
//   除法与深层 case 不落在逐像素路径上；逐像素路径只做浅层选字 + 查字模。
//============================================================================
module osd_overlay #(
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter OSD_FG     = 24'hFFFF00,  // 黄色文字
    parameter OSD_BG     = 24'h000000   // 黑色字符底
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_vsync,
    input wire I_hsync,
    input wire I_de,
    input wire [23:0] I_data,          // 已处理视频（visual_alg 输出）

    // HDMI 时钟域（已由 ball_info_cdc 同步）
    input wire [2:0]  I_mode,
    input wire [10:0] I_ball_x,        // 球心 X 像素坐标
    input wire [10:0] I_ball_y,        // 球心 Y 像素坐标
    input wire [15:0] I_speed,         // 球速（带符号 px/s）
    input wire        I_ball_valid,    // 球识别状态

    output reg O_vsync,
    output reg O_hsync,
    output reg O_de,
    output reg [23:0] O_data
);

    //----------------------------------------------------------------------
    // 像素坐标
    //----------------------------------------------------------------------
    reg [11:0] px, py;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin px <= 12'd0; py <= 12'd0; end
        else if(I_vsync) begin px <= 12'd0; py <= 12'd0; end
        else if(I_de) begin
            if(px == IMG_WIDTH-1) begin px <= 12'd0; py <= py + 12'd1; end
            else px <= px + 12'd1;
        end
    end

    //----------------------------------------------------------------------
    // 字符集编码
    //----------------------------------------------------------------------
    //  0x00 安  0x01 路
    //  0x10 '+' 0x11 '-' 0x12 ':' 0x13 空格
    //  0x20-0x29 数字 0-9
    //  0x30-0x49 字母 A-Z（跳过 J/Q/W 为空）
    //  0x50 '.' 0x51 ',' 0x52 '/'
    function [15:0] glyph_row;
        input [7:0] code;
        input [3:0] row;
        reg [255:0] glyph;
        begin
            case(code)
            8'h00: glyph = {16'h0000,16'h3C06,16'h0F1C,16'h01F0,16'h03E0,16'h0E60,16'h0C30,
                            16'h7FFE,16'h7FFE,16'h0200,16'h2304,16'h300C,16'h3FFC,16'h0180,
                            16'h0180,16'h0000}; // 安
            8'h01: glyph = {16'h0000,16'h408C,16'h78FC,16'h3E8C,16'h2884,16'h28CC,16'h2CFC,
                            16'h2FC6,16'h087C,16'h1C30,16'h3F78,16'h27C8,16'h26CC,16'h3EFC,
                            16'h0040,16'h0000}; // 路
            8'h10: glyph = {16'h0000,16'h0000,16'h0000,16'h0C20,16'h0660,16'h0240,16'h03C0,
                            16'h0180,16'h0180,16'h0180,16'h03C0,16'h0640,16'h0660,16'h0420,
                            16'h0000,16'h0000}; // +
            8'h11: glyph = {16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,
                            16'h0000,16'h0000,16'h0000,16'h0FE0,16'h0000,16'h0000,16'h0000,
                            16'h0000,16'h0000}; // -
            8'h12: glyph = {16'h0000,16'h0000,16'h0000,16'h0000,16'h0100,16'h0180,16'h0000,
                            16'h0000,16'h0000,16'h0000,16'h0000,16'h0180,16'h0180,16'h0000,
                            16'h0000,16'h0000}; // :
            8'h13: glyph = 256'h00000000000000000000000000000000; // space
            // 数字 0-9
            8'h20: glyph = {16'h0000,16'h0000,16'h0000,16'h03C0,16'h0660,16'h0460,16'h0420,
                            16'h0C20,16'h0C20,16'h0C20,16'h0420,16'h0660,16'h03C0,16'h0180,
                            16'h0000,16'h0000}; // 0
            8'h21: glyph = {16'h0000,16'h0000,16'h0000,16'h0180,16'h0180,16'h0180,16'h0180,
                            16'h0180,16'h0180,16'h0580,16'h0380,16'h0180,16'h0080,16'h0000,
                            16'h0000,16'h0000}; // 1
            8'h22: glyph = {16'h0000,16'h0000,16'h0000,16'h07E0,16'h0600,16'h0300,16'h0180,
                            16'h00C0,16'h0040,16'h0060,16'h0060,16'h0460,16'h07E0,16'h0380,
                            16'h0000,16'h0000}; // 2
            8'h23: glyph = {16'h0000,16'h0000,16'h0000,16'h03C0,16'h0660,16'h0420,16'h0020,
                            16'h0060,16'h01C0,16'h0060,16'h0060,16'h0460,16'h07E0,16'h0180,
                            16'h0000,16'h0000}; // 3
            8'h24: glyph = {16'h0000,16'h0000,16'h0000,16'h0040,16'h0040,16'h0FF0,16'h0CE0,
                            16'h0440,16'h0240,16'h0340,16'h01C0,16'h00C0,16'h00C0,16'h0040,
                            16'h0000,16'h0000}; // 4
            8'h25: glyph = {16'h0000,16'h0000,16'h0000,16'h07C0,16'h0C60,16'h0C20,16'h0020,
                            16'h0060,16'h0660,16'h07C0,16'h0400,16'h0600,16'h07E0,16'h03C0,
                            16'h0000,16'h0000}; // 5
            8'h26: glyph = {16'h0000,16'h0000,16'h0000,16'h03E0,16'h0660,16'h0430,16'h0C30,
                            16'h0430,16'h07E0,16'h03C0,16'h0300,16'h0180,16'h0080,16'h00C0,
                            16'h0000,16'h0000}; // 6
            8'h27: glyph = {16'h0000,16'h0000,16'h0000,16'h0300,16'h0300,16'h0100,16'h0180,
                            16'h0180,16'h0080,16'h00C0,16'h0040,16'h0020,16'h07E0,16'h07E0,
                            16'h0000,16'h0000}; // 7
            8'h28: glyph = {16'h0000,16'h0000,16'h0000,16'h07C0,16'h0460,16'h0C20,16'h0C20,
                            16'h0660,16'h03C0,16'h0660,16'h0460,16'h0460,16'h07C0,16'h0380,
                            16'h0000,16'h0000}; // 8
            8'h29: glyph = {16'h0000,16'h0000,16'h0000,16'h0300,16'h0180,16'h0180,16'h00C0,
                            16'h07C0,16'h0460,16'h0C60,16'h0C20,16'h0C60,16'h07C0,16'h0380,
                            16'h0000,16'h0000}; // 9
            // 字母 A-Z（code = 0x30 + (ASCII-'A')）
            8'h30: glyph = {16'h0000,16'h0000,16'h0180,16'h0180,16'h0180,16'h03C0,16'h03C0,16'h0240,16'h06C0,16'h07E0,16'h0660,16'h0420,16'h0C30,16'h0000,16'h0000,16'h0000}; // A
            8'h31: glyph = {16'h0000,16'h0000,16'h0780,16'h07E0,16'h0460,16'h0420,16'h0460,16'h07C0,16'h0460,16'h0420,16'h0420,16'h0460,16'h07C0,16'h0000,16'h0000,16'h0000}; // B
            8'h32: glyph = {16'h0000,16'h0000,16'h0180,16'h03E0,16'h0620,16'h0420,16'h0400,16'h0C00,16'h0C00,16'h0430,16'h0420,16'h0660,16'h03C0,16'h0000,16'h0000,16'h0000}; // C
            8'h33: glyph = {16'h0000,16'h0000,16'h0700,16'h07C0,16'h0460,16'h0460,16'h0420,16'h0420,16'h0420,16'h0420,16'h0460,16'h04E0,16'h07C0,16'h0000,16'h0000,16'h0000}; // D
            8'h34: glyph = {16'h0000,16'h0000,16'h07E0,16'h07E0,16'h0400,16'h0400,16'h0400,16'h07E0,16'h0600,16'h0400,16'h0400,16'h0400,16'h07E0,16'h0000,16'h0000,16'h0000}; // E
            8'h35: glyph = {16'h0000,16'h0000,16'h07E0,16'h07E0,16'h0400,16'h0400,16'h0400,16'h07C0,16'h07C0,16'h0400,16'h0400,16'h0400,16'h0400,16'h0000,16'h0000,16'h0000}; // F
            8'h36: glyph = {16'h0000,16'h0000,16'h0180,16'h03C0,16'h0660,16'h0420,16'h0400,16'h0C00,16'h0CE0,16'h0420,16'h0420,16'h0660,16'h03E0,16'h0000,16'h0000,16'h0000}; // G
            8'h37: glyph = {16'h0000,16'h0000,16'h0420,16'h0420,16'h0420,16'h0420,16'h0420,16'h07E0,16'h0460,16'h0420,16'h0420,16'h0420,16'h0420,16'h0000,16'h0000,16'h0000}; // H
            8'h38: glyph = {16'h0000,16'h0000,16'h0100,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0000,16'h0000,16'h0000}; // I
            8'h3A: glyph = {16'h0000,16'h0000,16'h0420,16'h0460,16'h04C0,16'h0580,16'h0700,16'h0780,16'h0780,16'h04C0,16'h04C0,16'h0460,16'h0420,16'h0000,16'h0000,16'h0000}; // K
            8'h3B: glyph = {16'h0000,16'h0000,16'h0400,16'h0400,16'h0400,16'h0400,16'h0400,16'h0400,16'h0400,16'h0400,16'h0400,16'h0400,16'h07E0,16'h0000,16'h0000,16'h0000}; // L
            8'h3C: glyph = {16'h0000,16'h0000,16'h0420,16'h0E60,16'h0E60,16'h0E60,16'h0E60,16'h0FE0,16'h0FE0,16'h0DA0,16'h0DA0,16'h0DA0,16'h0DA0,16'h0000,16'h0000,16'h0000}; // M
            8'h3D: glyph = {16'h0000,16'h0000,16'h0420,16'h0620,16'h0620,16'h0720,16'h0F20,16'h0520,16'h05A0,16'h04E0,16'h04E0,16'h0460,16'h0C60,16'h0000,16'h0000,16'h0000}; // N
            8'h3E: glyph = {16'h0000,16'h0000,16'h0180,16'h07C0,16'h0660,16'h0420,16'h0C20,16'h0C20,16'h0C20,16'h0C20,16'h0420,16'h0660,16'h03C0,16'h0000,16'h0000,16'h0000}; // O
            8'h3F: glyph = {16'h0000,16'h0000,16'h0780,16'h07E0,16'h0460,16'h0420,16'h0420,16'h0660,16'h07C0,16'h0400,16'h0400,16'h0400,16'h0400,16'h0000,16'h0000,16'h0000}; // P
            8'h41: glyph = {16'h0000,16'h0000,16'h0780,16'h07E0,16'h0460,16'h0420,16'h0460,16'h07E0,16'h07C0,16'h04C0,16'h0440,16'h0460,16'h0420,16'h0000,16'h0000,16'h0000}; // R
            8'h42: glyph = {16'h0000,16'h0000,16'h0180,16'h07C0,16'h0660,16'h0420,16'h0600,16'h0380,16'h00E0,16'h0060,16'h0C20,16'h0660,16'h07C0,16'h0000,16'h0000,16'h0000}; // S
            8'h43: glyph = {16'h0000,16'h0000,16'h07E0,16'h07E0,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0000,16'h0000,16'h0000}; // T
            8'h44: glyph = {16'h0000,16'h0000,16'h0420,16'h0420,16'h0420,16'h0420,16'h0420,16'h0420,16'h0420,16'h0420,16'h0420,16'h0660,16'h07C0,16'h0000,16'h0000,16'h0000}; // U
            8'h45: glyph = {16'h0000,16'h0000,16'h0820,16'h0C30,16'h0460,16'h0660,16'h0660,16'h0640,16'h02C0,16'h03C0,16'h0380,16'h0180,16'h0180,16'h0000,16'h0000,16'h0000}; // V
            8'h46: glyph = {16'h0000,16'h0000,16'h0900,16'h0DB0,16'h0DB0,16'h0DA0,16'h0FA0,16'h07E0,16'h06E0,16'h06E0,16'h0660,16'h0660,16'h0660,16'h0000,16'h0000,16'h0000}; // W
            8'h47: glyph = {16'h0000,16'h0000,16'h0420,16'h0660,16'h0640,16'h03C0,16'h0180,16'h0180,16'h0180,16'h03C0,16'h0240,16'h0660,16'h0C20,16'h0000,16'h0000,16'h0000}; // X
            8'h48: glyph = {16'h0000,16'h0000,16'h0420,16'h0420,16'h0660,16'h0240,16'h03C0,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0180,16'h0000,16'h0000,16'h0000}; // Y
            8'h49: glyph = {16'h0000,16'h0000,16'h07E0,16'h07E0,16'h0060,16'h00C0,16'h0080,16'h0180,16'h0100,16'h0300,16'h0600,16'h0400,16'h0FE0,16'h0000,16'h0000,16'h0000}; // Z
            8'h50: glyph = {16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0600,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000}; // .
            8'h51: glyph = {16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0600,16'h0600,16'h0600,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000,16'h0000}; // ,
            8'h52: glyph = {16'h0000,16'h0000,16'h0020,16'h0020,16'h0040,16'h0040,16'h0080,16'h0180,16'h0100,16'h0300,16'h0200,16'h0400,16'h0400,16'h0800,16'h0000,16'h0000}; // /
            default: glyph = 256'h00000000000000000000000000000000;
            endcase
            glyph_row = (code==8'h00 || code==8'h01) ? // AnLu keep
                    glyph[row*16 +: 16] :
                    glyph[(15-row)*16 +: 16]; // english y-mirror
        end
    endfunction

    //----------------------------------------------------------------------
    // 字符常量
    //----------------------------------------------------------------------
    localparam SP = 8'h13;   // 空格
    localparam COL= 8'h12;   // ':'
    localparam [7:0] A=8'h30,B=8'h31,C=8'h32,D=8'h33,E=8'h34,F=8'h35,G=8'h36,H=8'h37,
                     I_=8'h38,K=8'h3A,L=8'h3B,M=8'h3C,N=8'h3D,O=8'h3E,P=8'h3F,
                     R=8'h41,S=8'h42,T=8'h43,U=8'h44,V=8'h45,W=8'h46,X=8'h47,Y=8'h48;

    //----------------------------------------------------------------------
    // 动态字符预解码（登记到寄存器，把除法/深层 case 移出逐像素路径）
    //   输入 I_ball_x / I_speed / I_mode / I_ball_valid 均为稳定信号
    //   （ball_info_cdc 每帧更新一次），因此可每拍采样并登记。
    //----------------------------------------------------------------------
    reg [7:0] bx_d[0:3];       // BALL X 四位数字字符
    reg [7:0] sp_sgn;          // SPEED 符号（+ / -）
    reg [7:0] sp_d[0:3];       // SPEED 四位数字字符
    reg [7:0] mn[0:9];         // MODE 名称字符（最长 BALL TRACK = 10）
    reg [7:0] stn[0:6];        // STATE 名称字符（最长 INVALID = 7）
    reg [15:0] spd_abs;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            bx_d[0]<=8'h20; bx_d[1]<=8'h20; bx_d[2]<=8'h20; bx_d[3]<=8'h20;
            sp_sgn<=8'h10;  sp_d[0]<=8'h20; sp_d[1]<=8'h20; sp_d[2]<=8'h20; sp_d[3]<=8'h20;
            mn[0]<=SP; mn[1]<=SP; mn[2]<=SP; mn[3]<=SP; mn[4]<=SP;
            mn[5]<=SP; mn[6]<=SP; mn[7]<=SP; mn[8]<=SP; mn[9]<=SP;
            stn[0]<=SP; stn[1]<=SP; stn[2]<=SP; stn[3]<=SP; stn[4]<=SP; stn[5]<=SP; stn[6]<=SP;
            spd_abs<=16'd0;
        end else begin
            spd_abs <= I_speed[15] ? (~I_speed + 16'd1) : I_speed;
            bx_d[3] <= 8'h20 + I_ball_x/1000;
            bx_d[2] <= 8'h20 + (I_ball_x/100)%10;
            bx_d[1] <= 8'h20 + (I_ball_x/10)%10;
            bx_d[0] <= 8'h20 + I_ball_x%10;
            sp_sgn  <= I_speed[15] ? 8'h11 : 8'h10;
            sp_d[3] <= 8'h20 + ((I_speed[15]?(~I_speed+16'd1):I_speed)/1000);
            sp_d[2] <= 8'h20 + (((I_speed[15]?(~I_speed+16'd1):I_speed)/100)%10);
            sp_d[1] <= 8'h20 + (((I_speed[15]?(~I_speed+16'd1):I_speed)/10)%10);
            sp_d[0] <= 8'h20 + ((I_speed[15]?(~I_speed+16'd1):I_speed)%10);
            case(I_mode)
            3'd0: begin mn[0]<=R; mn[1]<=A; mn[2]<=W; mn[3]<=SP; mn[4]<=SP; mn[5]<=SP; mn[6]<=SP; mn[7]<=SP; mn[8]<=SP; mn[9]<=SP; end
            3'd1: begin mn[0]<=G; mn[1]<=R; mn[2]<=A; mn[3]<=Y; mn[4]<=SP; mn[5]<=SP; mn[6]<=SP; mn[7]<=SP; mn[8]<=SP; mn[9]<=SP; end
            3'd2: begin mn[0]<=B; mn[1]<=I_; mn[2]<=N; mn[3]<=A; mn[4]<=R; mn[5]<=Y; mn[6]<=SP; mn[7]<=SP; mn[8]<=SP; mn[9]<=SP; end
            3'd3: begin mn[0]<=S; mn[1]<=O; mn[2]<=B; mn[3]<=E; mn[4]<=L; mn[5]<=SP; mn[6]<=SP; mn[7]<=SP; mn[8]<=SP; mn[9]<=SP; end
            3'd4: begin mn[0]<=B; mn[1]<=A; mn[2]<=L; mn[3]<=L; mn[4]<=SP; mn[5]<=M; mn[6]<=A; mn[7]<=S; mn[8]<=K; mn[9]<=SP; end
            default: begin mn[0]<=B; mn[1]<=A; mn[2]<=L; mn[3]<=L; mn[4]<=SP; mn[5]<=T; mn[6]<=R; mn[7]<=A; mn[8]<=C; mn[9]<=K; end // BALL TRACK
            endcase
            if(I_ball_valid) begin
                stn[0]<=V; stn[1]<=A; stn[2]<=L; stn[3]<=I_; stn[4]<=D; stn[5]<=SP; stn[6]<=SP;
            end else begin
                stn[0]<=I_; stn[1]<=N; stn[2]<=V; stn[3]<=A; stn[4]<=L; stn[5]<=I_; stn[6]<=D;
            end
        end
    end

    //----------------------------------------------------------------------
    // 各行矩形判定（组合，逐行）
    //----------------------------------------------------------------------
    localparam X0=24;                 // 文本左对齐
    localparam Y_LOGO=12, Y_TITLE=32, Y_MODE=58, Y_BALLX=82, Y_SPEED=106, Y_STATE=130;
    localparam N_TITLE=25, N_MODE=18, N_BALLX=12, N_SPEED=18, N_STATE=15;

    wire de_on = I_de;
    wire in_logo   = de_on && (py>=Y_LOGO  && py<Y_LOGO +16 && px>=X0 && px<X0+ 2*16);
    wire in_title  = de_on && (py>=Y_TITLE && py<Y_TITLE+16 && px>=X0 && px<X0+N_TITLE*16);
    wire in_mode   = de_on && (py>=Y_MODE  && py<Y_MODE +16 && px>=X0 && px<X0+N_MODE*16);
    wire in_ballx  = de_on && (py>=Y_BALLX && py<Y_BALLX+16 && px>=X0 && px<X0+N_BALLX*16);
    wire in_speed  = de_on && (py>=Y_SPEED && py<Y_SPEED+16 && px>=X0 && px<X0+N_SPEED*16);
    wire in_state  = de_on && (py>=Y_STATE && py<Y_STATE+16 && px>=X0 && px<X0+N_STATE*16);
    wire in_any    = in_logo | in_title | in_mode | in_ballx | in_speed | in_state;

    wire [4:0] c_logo  = (px-X0)>>4;
    wire [4:0] c_title = (px-X0)>>4;
    wire [4:0] c_mode  = (px-X0)>>4;
    wire [4:0] c_ballx = (px-X0)>>4;
    wire [4:0] c_speed = (px-X0)>>4;
    wire [4:0] c_state = (px-X0)>>4;
    wire [3:0] b   = (px-X0)&4'hF;
    wire [3:0] rl  = py - Y_LOGO;
    wire [3:0] rt  = py - Y_TITLE;
    wire [3:0] rm  = py - Y_MODE;
    wire [3:0] rb  = py - Y_BALLX;
    wire [3:0] rsp = py - Y_SPEED;
    wire [3:0] rst = py - Y_STATE;

    // 返回 (line,col) 处字符码（浅层：静态常量 / 预解码寄存器）
    function [7:0] char_code;
        input [2:0] line;
        input [4:0] col;
        begin
            case(line)
            3'd0: char_code = (col==5'd0) ? 8'h00 : 8'h01;          // 安路
            3'd1: begin  // FPGA BALL CONTROL SYSTEM
                case(col)
                5'd0: char_code=F; 5'd1: char_code=P; 5'd2: char_code=G; 5'd3: char_code=A; 5'd4: char_code=SP;
                5'd5: char_code=B; 5'd6: char_code=A; 5'd7: char_code=L; 5'd8: char_code=L; 5'd9: char_code=SP;
                5'd10: char_code=C; 5'd11: char_code=O; 5'd12: char_code=N; 5'd13: char_code=T; 5'd14: char_code=R;
                5'd15: char_code=O; 5'd16: char_code=L; 5'd17: char_code=SP; 5'd18: char_code=S; 5'd19: char_code=Y;
                5'd20: char_code=S; 5'd21: char_code=T; 5'd22: char_code=E; 5'd23: char_code=M;
                default: char_code = SP;
                endcase
            end
            3'd2: begin  // MODE  : <预解码 mn>
                case(col)
                5'd0: char_code=M; 5'd1: char_code=O; 5'd2: char_code=D; 5'd3: char_code=E;
                5'd4: char_code=SP; 5'd5: char_code=SP; 5'd6: char_code=COL; 5'd7: char_code=SP;
                default: char_code = mn[col-5'd8];
                endcase
            end
            3'd3: begin  // BALL X: <预解码 bx_d>
                case(col)
                5'd0: char_code=B; 5'd1: char_code=A; 5'd2: char_code=L; 5'd3: char_code=L;
                5'd4: char_code=SP; 5'd5: char_code=X; 5'd6: char_code=COL; 5'd7: char_code=SP;
                5'd8: char_code=bx_d[3]; 5'd9: char_code=bx_d[2];
                5'd10: char_code=bx_d[1]; 5'd11: char_code=bx_d[0];
                default: char_code = SP;
                endcase
            end
            3'd4: begin  // SPEED : <预解码 sp_sgn/sp_d> px/s
                case(col)
                5'd0: char_code=S; 5'd1: char_code=P; 5'd2: char_code=E; 5'd3: char_code=E; 5'd4: char_code=D;
                5'd5: char_code=SP; 5'd6: char_code=COL; 5'd7: char_code=SP;
                5'd8: char_code=sp_sgn;
                5'd9: char_code=sp_d[3]; 5'd10: char_code=sp_d[2];
                5'd11: char_code=sp_d[1]; 5'd12: char_code=sp_d[0];
                5'd13: char_code=SP; 5'd14: char_code=P; 5'd15: char_code=X; 5'd16: char_code=8'h52; 5'd17: char_code=S;
                default: char_code = SP;
                endcase
            end
            3'd5: begin  // STATE : <预解码 stn>
                case(col)
                5'd0: char_code=S; 5'd1: char_code=T; 5'd2: char_code=A; 5'd3: char_code=T; 5'd4: char_code=E;
                5'd5: char_code=SP; 5'd6: char_code=COL; 5'd7: char_code=SP;
                default: char_code = stn[col-5'd8];
                endcase
            end
            default: char_code = SP;
            endcase
        end
    endfunction

    // 逐行前景位（纯函数 + wire）
    wire [7:0]  code_logo  = char_code(3'd0, c_logo);
    wire [7:0]  code_title = char_code(3'd1, c_title);
    wire [7:0]  code_mode  = char_code(3'd2, c_mode);
    wire [7:0]  code_ballx = char_code(3'd3, c_ballx);
    wire [7:0]  code_speed = char_code(3'd4, c_speed);
    wire [7:0]  code_state = char_code(3'd5, c_state);
    wire [15:0] gg_logo  = glyph_row(code_logo,  rl);
    wire [15:0] gg_title = glyph_row(code_title, rt);
    wire [15:0] gg_mode  = glyph_row(code_mode,  rm);
    wire [15:0] gg_ballx = glyph_row(code_ballx, rb);
    wire [15:0] gg_speed = glyph_row(code_speed, rsp);
    wire [15:0] gg_state = glyph_row(code_state, rst);
    wire fg_logo  = in_logo  && gg_logo[15-b];
    wire fg_title = in_title && gg_title[15-b];
    wire fg_mode  = in_mode  && gg_mode[15-b];
    wire fg_ballx = in_ballx && gg_ballx[15-b];
    wire fg_speed = in_speed && gg_speed[15-b];
    wire fg_state = in_state && gg_state[15-b];

    wire fg = fg_logo | fg_title | fg_mode | fg_ballx | fg_speed | fg_state;

    //----------------------------------------------------------------------
    // 球心十字准星（在球心处画十字）
    //----------------------------------------------------------------------
    localparam HALF = 20;
    wire crosshair = I_ball_valid && de_on &&
        ( (py>=I_ball_y-1 && py<=I_ball_y+1 && px>=I_ball_x-HALF && px<=I_ball_x+HALF) ||
          (px>=I_ball_x-1 && px<=I_ball_x+1 && py>=I_ball_y-HALF && py<=I_ball_y+HALF) );

    //----------------------------------------------------------------------
    // 输出（零延迟，同步直通）
    //----------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_vsync<=1'b0; O_hsync<=1'b0; O_de<=1'b0; O_data<=24'd0;
        end else begin
            O_vsync <= I_vsync;
            O_hsync <= I_hsync;
            O_de    <= I_de;
            if(crosshair)
                O_data <= OSD_FG;
            else if(in_any)
                O_data <= fg ? OSD_FG : OSD_BG;
            else
                O_data <= I_data;
        end
    end

endmodule
