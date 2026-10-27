`timescale 1ns/1ps
//============================================================================
// ball_speed.v
//   钢球速度计算（CSI 时钟域）。
//   每个 frame_done 捕获一次球心 X，速度 = 帧间位移 x 帧率（px/s）。
//   带符号输出；球未确认时速度无效并清零。
//============================================================================
module ball_speed #(
    parameter FPS = 30          // 名义帧率（约 30fps）
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire [10:0] I_ball_x,     // 球心 X（像素）
    input wire        I_ball_valid, // 球已确认
    input wire        I_frame_done, // 每帧一个脉冲
    output reg [15:0] O_speed,      // 带符号 px/s
    output reg        O_speed_valid
);

    reg [10:0] prev_x;
    reg        prev_valid;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            prev_x     <= 11'd0;
            prev_valid <= 1'b0;
            O_speed    <= 16'd0;
            O_speed_valid <= 1'b0;
        end else if(I_frame_done) begin
            if(I_ball_valid && prev_valid) begin
                // 帧间位移（带符号）x 帧率
                O_speed <= (($signed({5'b0, I_ball_x}) - $signed({5'b0, prev_x})) * FPS);
                O_speed_valid <= 1'b1;
            end else begin
                O_speed <= 16'd0;
                O_speed_valid <= 1'b0;
            end
            prev_x     <= I_ball_x;
            prev_valid <= I_ball_valid;
        end
    end

endmodule
