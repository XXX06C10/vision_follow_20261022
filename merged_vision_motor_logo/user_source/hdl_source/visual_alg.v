`timescale 1ns/1ps
//============================================================================
// visual_alg.v
//   FPGA 视觉算法流水（RGB888 像素域，HDMI 像素时钟）。
//   插在 hdmi_mixer 与 osd_overlay 之间。
//   支持实时切换的模式（I_mode[2:0]）：
//      0 RAW       原画面直通
//      1 GRAY      灰度  Y = (77R+150G+29B)>>8
//      2 BINARY    二值化 Y > BINARY_TH 白 / 否则黑
//      3 SOBEL     边缘检测（Y 通道 3x3 Sobel，双行缓存）
//      4 BALL MASK 钢球分割掩码（球框内原色，框外压暗）
//      5 BALL TRACK 直通视频（检测框已由 detector 叠加 + OSD 十字）
//
//   所有模式共享同一流水延迟（统一 de/vs/hs 与数据延迟），切换无花屏错位。
//   SOBEL 结果对应 3x3 窗口中心（上移 1 行/左移 1 像素），为流式边缘检测惯例，
//   属恒定偏移，不影响同步稳定。
//============================================================================
module visual_alg #(
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter BINARY_TH  = 128,   // 二值化阈值（0-255）
    parameter SOBEL_TH   = 100    // 边缘幅值阈值（0-1020）
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_vsync,
    input wire I_hsync,
    input wire I_de,
    input wire [23:0] I_data,          // RGB888 {R,G,B}
    input wire [2:0] I_mode,

    // BALL MASK 需要球框（已同步到本时钟域）
    input wire I_box_valid,
    input wire [10:0] I_box_left,
    input wire [10:0] I_box_right,
    input wire [10:0] I_box_top,
    input wire [10:0] I_box_bottom,

    output reg O_vsync,
    output reg O_hsync,
    output reg O_de,
    output reg [23:0] O_data
);

    integer i;   // 循环变量（用于 S3 窗口复位与延迟链）

    //----------------------------------------------------------------------
    // S1：输入寄存器 + Y 亮度计算（统一起点）
    //----------------------------------------------------------------------
    reg        s_de1, s_vs1, s_hs1;
    reg [23:0] s_rgb1;
    reg [7:0]  s_y1;

    // Y ~= 0.30R + 0.56G + 0.13B, implemented as shift-adds that all fit
    // in 8 bits (max 63+127+31+31 = 252) so no wide adders are needed.
    // (The old 8-bit shift version wrapped to 0, making GRAY/BINARY/SOBEL black.)
    wire [7:0] y_r2 = I_data[23:16] >> 2;
    wire [7:0] y_g1 = I_data[15:8]  >> 1;
    wire [7:0] y_g3 = I_data[15:8]  >> 3;
    wire [7:0] y_b3 = I_data[7:0]   >> 3;
    wire [8:0] y_sum = y_r2 + y_g1 + y_g3 + y_b3;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            s_de1<=1'b0; s_vs1<=1'b0; s_hs1<=1'b0;
            s_rgb1<=24'd0; s_y1<=8'd0;
        end else begin
            s_de1<=I_de; s_vs1<=I_vsync; s_hs1<=I_hsync;
            s_rgb1<=I_data;
            s_y1 <= y_sum[7:0];
        end
    end

    //----------------------------------------------------------------------
    // 行缓存（双 RAM，提供上一行与上两行 Y），列地址与 S1 对齐
    //----------------------------------------------------------------------
    reg [10:0] waddr;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) waddr <= 11'd0;
        else if(!s_de1) waddr <= 11'd0;
        else if(waddr < IMG_WIDTH-1) waddr <= waddr + 11'd1;
    end

    // 行缓存（block RAM，同步读；列对齐见写 ld2 注释）
    (* ram_style="block" *) reg [7:0] ld1_mem [0:IMG_WIDTH-1];   // 上一行 N-1
    (* ram_style="block" *) reg [7:0] ld2_mem [0:IMG_WIDTH-1];   // 上两行 N-2

    // 同步读（延迟1拍，读到写前旧值 = 对应列上一行/上两行）
    // cur_q / ld1_rdq / ld2_rdq 在同一拍对齐到同一列 c
    reg [7:0] ld1_rdq, ld2_rdq;
    reg [7:0] cur_q;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin ld1_rdq<=8'd0; ld2_rdq<=8'd0; cur_q<=8'd0; end
        else begin
            ld1_rdq <= ld1_mem[waddr];
            ld2_rdq <= ld2_mem[waddr];
            cur_q   <= s_y1;               // 当前行 N, 列 waddr
        end
    end

    // 写 ld1（读写同地址 waddr，读旧值 = 上一行 N-1）
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin end
        else if(s_de1) ld1_mem[waddr] <= s_y1;
    end

    // 写 ld2（用延迟写地址 waddr_d 对齐列：ld1_rdq 为上一行 N-1 在 waddr_d 列）
    reg [10:0] waddr_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) waddr_d <= 11'd0;
        else waddr_d <= waddr;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin end
        else if(s_de1) ld2_mem[waddr_d] <= ld1_rdq;   // 写入 N-1（下轮成为 N-2）
    end

    //----------------------------------------------------------------------
    // S3：3x3 窗口列移位（用对齐后的 cur_q / ld1_rdq / ld2_rdq）
    //----------------------------------------------------------------------
    reg [7:0] win [0:8];   // win[r*3+c]，r=0 上两行 N-2, r=1 上一行 N-1, r=2 当前行 N
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for(i=0;i<9;i=i+1) win[i] <= 8'd0;
        end else if(s_de1) begin
            win[0+2]<=win[0+1]; win[0+1]<=win[0+0]; win[0+0]<=ld2_rdq;
            win[3+2]<=win[3+1]; win[3+1]<=win[3+0]; win[3+0]<=ld1_rdq;
            win[6+2]<=win[6+1]; win[6+1]<=win[6+0]; win[6+0]<=cur_q;
        end
    end

    //----------------------------------------------------------------------
    // S4：Sobel 幅值
    //----------------------------------------------------------------------
    wire signed [12:0] gx =
        (win[6+0] + (win[6+1]<<1) + win[6+2])
      - (win[0+0] + (win[0+1]<<1) + win[0+2]);
    wire signed [12:0] gy =
        (win[0+0] + (win[3+0]<<1) + win[6+0])
      - (win[0+2] + (win[3+2]<<1) + win[6+2]);
    reg [13:0] mag;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) mag <= 14'd0;
        else mag <= (gx<0 ? (0-gx) : gx) + (gy<0 ? (0-gy) : gy);
    end

    // S5：边缘二值（强度≥阈值 → 白）
    reg [7:0] sobel_y;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) sobel_y <= 8'd0;
        else sobel_y <= (mag >= SOBEL_TH) ? 8'hFF : 8'h00;
    end

    //----------------------------------------------------------------------
    // 统一延迟：所有模式输出对齐到 S5（sobel_y 所在级）
    //  de/vs/hs 延迟 5 级；rgb/gray/binary 从 S1 再延迟 4 级（共 5 级）
    //----------------------------------------------------------------------
    reg        de_d[0:4], vs_d[0:4], hs_d[0:4];
    reg [23:0] rgb_d[0:4];
    reg [7:0]  y_d[0:4];
    reg [7:0]  bin_d[0:4];
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            for(i=0;i<5;i=i+1) begin de_d[i]<=1'b0; vs_d[i]<=1'b0; hs_d[i]<=1'b0; end
            for(i=0;i<5;i=i+1) begin rgb_d[i]<=24'd0; y_d[i]<=8'd0; bin_d[i]<=8'd0; end
        end else begin
            de_d[0]<=s_de1; vs_d[0]<=s_vs1; hs_d[0]<=s_hs1;
            for(i=1;i<5;i=i+1) begin
                de_d[i]<=de_d[i-1]; vs_d[i]<=vs_d[i-1]; hs_d[i]<=hs_d[i-1];
            end
            rgb_d[0]<=s_rgb1;
            y_d[0]  <=s_y1;
            bin_d[0]<=(s_y1 > BINARY_TH) ? 8'hFF : 8'h00;
            for(i=1;i<5;i=i+1) begin
                rgb_d[i]<=rgb_d[i-1];
                y_d[i]  <=y_d[i-1];
                bin_d[i]<=bin_d[i-1];
            end
        end
    end

    //----------------------------------------------------------------------
    // BALL MASK：输出像素坐标（对齐 S5），球框内原色，框外压暗
    //----------------------------------------------------------------------
    reg [11:0] px, py;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin px<=12'd0; py<=12'd0; end
        else if(vs_d[4]) begin px<=12'd0; py<=12'd0; end
        else if(de_d[4]) begin
            if(px==IMG_WIDTH-1) begin px<=12'd0; py<=py+12'd1; end
            else px<=px+12'd1;
        end
    end
    wire in_box = de_d[4] && I_box_valid &&
                  (px>=I_box_left) && (px<=I_box_right) &&
                  (py>=I_box_top)  && (py<=I_box_bottom);

    // SOBEL 前两行无有效窗口，置黑
    wire sobel_valid_line = (py >= 12'd2);

    //----------------------------------------------------------------------
    // 输出（所有信号均来自寄存器，组合 mux，无环路）
    //----------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_vsync<=1'b0; O_hsync<=1'b0; O_de<=1'b0; O_data<=24'd0;
        end else begin
            O_vsync <= vs_d[4];
            O_hsync <= hs_d[4];
            O_de    <= de_d[4];
            case(I_mode)
                3'd0: O_data <= rgb_d[4];                          // RAW
                3'd1: O_data <= {y_d[4], y_d[4], y_d[4]};          // GRAY
                3'd2: O_data <= {bin_d[4], bin_d[4], bin_d[4]};    // BINARY
                3'd3: O_data <= sobel_valid_line ?                 // SOBEL
                       {sobel_y, sobel_y, sobel_y} : 24'h000000;
                3'd4: O_data <= in_box ? rgb_d[4] : 24'h080808;    // BALL MASK
                default: O_data <= rgb_d[4];                       // RAW / BALL TRACK
            endcase
        end
    end

endmodule
