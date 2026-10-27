`timescale 1ns/1ps
//============================================================================
// logo_overlay.v
//   Overlays the Anlogic logo image on the HDMI pixel stream.
//   Placed after osd_overlay, so it stays visible in every visual_alg mode.
//   The logo ROM has one cycle of read latency, so video is delayed 2 cycles.
//============================================================================
module logo_overlay #(
    parameter IMG_WIDTH  = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter LOGO_X     = 1744,
    parameter LOGO_Y     = 24,
    parameter LOGO_W     = 146,
    parameter LOGO_H     = 94
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_hsync,
    input  wire        I_de,
    input  wire [23:0] I_data,

    output reg         O_vsync,
    output reg         O_hsync,
    output reg         O_de,
    output reg  [23:0] O_data
);

    reg [11:0] px, py;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            px <= 12'd0;
            py <= 12'd0;
        end else if(I_vsync) begin
            px <= 12'd0;
            py <= 12'd0;
        end else if(I_de) begin
            if(px == IMG_WIDTH-1) begin
                px <= 12'd0;
                py <= py + 12'd1;
            end else begin
                px <= px + 12'd1;
            end
        end
    end

    wire in_logo = I_de &&
                   (px >= LOGO_X) && (px < LOGO_X + LOGO_W) &&
                   (py >= LOGO_Y) && (py < LOGO_Y + LOGO_H);

    wire [11:0] lx = px - LOGO_X;
    wire [11:0] ly = py - LOGO_Y;
    // LOGO_W = 146 = 128 + 16 + 2
    wire [13:0] logo_addr = (ly << 7) + (ly << 4) + (ly << 1) + lx;

    wire [24:0] logo_pixel;
    logo_rom u_logo_rom (
        .I_clk   ( I_clk      ),
        .I_addr  ( logo_addr  ),
        .O_pixel ( logo_pixel )
    );

    reg        de_d, vs_d, hs_d, hit_d;
    reg [23:0] data_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            de_d <= 1'b0;
            vs_d <= 1'b0;
            hs_d <= 1'b0;
            hit_d <= 1'b0;
            data_d <= 24'd0;
        end else begin
            de_d <= I_de;
            vs_d <= I_vsync;
            hs_d <= I_hsync;
            hit_d <= in_logo;
            data_d <= I_data;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_vsync <= 1'b0;
            O_hsync <= 1'b0;
            O_de <= 1'b0;
            O_data <= 24'd0;
        end else begin
            O_vsync <= vs_d;
            O_hsync <= hs_d;
            O_de <= de_d;
            O_data <= (hit_d && logo_pixel[24]) ? logo_pixel[23:0] : data_d;
        end
    end

endmodule
