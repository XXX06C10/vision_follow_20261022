`timescale 1ns/1ps
//============================================================================
// ball_info_cdc.v
//   钢球识别信息跨时钟域（CSI -> HDMI），4 相握手，每帧一次。
//   一次性同步：球心 X/Y、速度、识别状态、球框坐标，供 OSD 与 BALL MASK 使用。
//============================================================================
module ball_info_cdc #(
    parameter W = 8             // 数据位宽
)(
    // CSI 源时钟域
    input wire I_csi_clk,
    input wire I_csi_rst_n,
    input wire I_frame_done,    // 每帧一个脉冲
    input wire [W-1:0] I_data,  // 待同步数据（源端保持到握手完成）

    // HDMI 目标时钟域
    input wire O_hdmi_clk,
    input wire O_hdmi_rst_n,
    output reg [W-1:0] O_data,  // 稳定后的数据
    output reg O_valid          // 新数据捕获脉冲（目标时钟域 1 拍）
);

    // ---- 源端 ----
    reg [W-1:0] hold_data;
    reg         src_req;
    reg [1:0]   ack_sync;
    wire        ack_synced = ack_sync[1];
    always @(posedge I_csi_clk or negedge I_csi_rst_n) begin
        if(!I_csi_rst_n) begin
            hold_data <= {W{1'b0}};
            src_req   <= 1'b0;
            ack_sync  <= 2'b00;
        end else begin
            ack_sync <= {ack_sync[0], O_ack};
            if(I_frame_done && (src_req == ack_synced)) begin
                hold_data <= I_data;
                src_req   <= ~src_req;
            end
        end
    end

    // ---- 目标端 ----
    reg [1:0]   req_sync;
    reg         req_seen;
    reg [W-1:0] hold_s1, hold_s;
    reg         O_ack;   // 握手内部反馈（源端需读取），非对外端口
    always @(posedge O_hdmi_clk or negedge O_hdmi_rst_n) begin
        if(!O_hdmi_rst_n) begin
            req_sync <= 2'b00;
            req_seen <= 1'b0;
            O_ack    <= 1'b0;
            O_data   <= {W{1'b0}};
            O_valid  <= 1'b0;
            hold_s1  <= {W{1'b0}};
            hold_s   <= {W{1'b0}};
        end else begin
            req_sync <= {req_sync[0], src_req};
            hold_s1  <= hold_data;
            hold_s   <= hold_s1;
            if(req_sync[1] != req_seen) begin
                O_data  <= hold_s;
                req_seen<= req_sync[1];
                O_ack   <= req_sync[1];
                O_valid <= 1'b1;
            end else begin
                O_valid <= 1'b0;
            end
        end
    end

endmodule
