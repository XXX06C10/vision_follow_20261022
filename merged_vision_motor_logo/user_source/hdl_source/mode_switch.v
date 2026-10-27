`timescale 1ns/1ps
//============================================================================
// mode_switch.v
//   按键实时切换视觉算法模式。
//   按键原始输入（低有效）在 I_sys_clk 域消抖产生单脉冲，
//   再 CDC 到 HDMI 像素时钟域，递增模式计数器（0..MODE_NUM-1，循环）。
//============================================================================
module mode_switch #(
    parameter MODE_NUM = 6
)(
    input wire I_sys_clk,
    input wire I_sys_rst_n,
    input wire I_button_raw,    // 低有效按下，内部消抖

    input wire O_hdmi_clk,
    input wire O_hdmi_rst_n,

    output reg [2:0] O_mode,
    output reg O_mode_pulse     // HDMI 域，切换瞬间 1 拍
);

    wire key_pulse;
    key_remove_shakes u_db (
        .I_clk(I_sys_clk), .I_rst_n(I_sys_rst_n),
        .I_key_in(I_button_raw), .O_key_trig_out(key_pulse)
    );

    // 单脉冲 CDC：2 级同步 + 边沿检测
    reg [1:0] p_sync;
    reg       p_prev;
    wire hdmi_pulse = p_sync[1] && !p_prev;
    always @(posedge O_hdmi_clk or negedge O_hdmi_rst_n) begin
        if(!O_hdmi_rst_n) begin
            p_sync <= 2'b00;
            p_prev <= 1'b0;
        end else begin
            p_sync <= {p_sync[0], key_pulse};
            p_prev <= p_sync[1];
        end
    end

    always @(posedge O_hdmi_clk or negedge O_hdmi_rst_n) begin
        if(!O_hdmi_rst_n) begin
            O_mode <= 3'd0;
            O_mode_pulse <= 1'b0;
        end else begin
            O_mode_pulse <= 1'b0;
            if(hdmi_pulse) begin
                if(O_mode == MODE_NUM-1) O_mode <= 3'd0;
                else O_mode <= O_mode + 3'd1;
                O_mode_pulse <= 1'b1;
            end
        end
    end

endmodule
