module steel_ball_motion #(
    parameter CLK_HZ = 50000000,
    parameter SCREEN_LEFT = 190,
    parameter SCREEN_RIGHT = 1860,
    parameter TRAVEL_TENTH_MM = 1300
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_frame,
    input wire I_confirmed,
    input wire [10:0] I_ball_x,
    output reg [1:0] O_valid,
    output reg [10:0] O_position_tenth_mm,
    output reg signed [15:0] O_speed_mm_s
);
    localparam integer MM_SCALE = (TRAVEL_TENTH_MM * 65536 + (SCREEN_RIGHT-SCREEN_LEFT)/2) / (SCREEN_RIGHT-SCREEN_LEFT);
    localparam integer MS_DIV = CLK_HZ / 1000;
    reg [15:0] ms_count;
    reg [7:0] frame_ms;
    reg seen_frame;
    reg [1:0] history_count;
    reg [10:0] position_1, position_2;
    wire [10:0] ball_offset = I_ball_x <= SCREEN_LEFT ? 11'd0 :
        I_ball_x >= SCREEN_RIGHT ? SCREEN_RIGHT-SCREEN_LEFT : I_ball_x-SCREEN_LEFT;
    wire [31:0] position_product = ball_offset * MM_SCALE;
    wire [10:0] position_now = (position_product + 32'd32768) >> 16;
    wire interval_ok = seen_frame && frame_ms >= 8'd25 && frame_ms <= 8'd45;
    wire signed [15:0] speed_delta = $signed({1'b0,position_now}) - $signed({1'b0,position_2});
    wire signed [17:0] speed_scaled = speed_delta * 18'sd3;
    wire signed [15:0] speed_now = speed_scaled / 18'sd2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            ms_count <= 0;
            frame_ms <= 0;
            seen_frame <= 0;
            history_count <= 0;
            position_1 <= 0;
            position_2 <= 0;
            O_valid <= 0;
            O_position_tenth_mm <= 0;
            O_speed_mm_s <= 0;
        end else begin
            if(ms_count == MS_DIV-1) begin
                ms_count <= 0;
                if(frame_ms != 8'hff) frame_ms <= frame_ms+1'b1;
            end else ms_count <= ms_count+1'b1;
            if(I_frame) begin
                ms_count <= 0;
                frame_ms <= 0;
                seen_frame <= 1;
                if(!I_confirmed) begin
                    history_count <= 0;
                    O_valid <= 0;
                    O_position_tenth_mm <= 0;
                    O_speed_mm_s <= 0;
                end else begin
                    O_valid[0] <= 1;
                    O_position_tenth_mm <= position_now;
                    position_2 <= position_1;
                    position_1 <= position_now;
                    if(!interval_ok || !O_valid[0]) begin
                        history_count <= 2'd1;
                        O_valid[1] <= 0;
                        O_speed_mm_s <= 0;
                    end else if(history_count >= 2'd2) begin
                        O_valid[1] <= 1;
                        O_speed_mm_s <= speed_now;
                    end else begin
                        history_count <= history_count+1'b1;
                        O_valid[1] <= 0;
                        O_speed_mm_s <= 0;
                    end
                end
            end
        end
    end
endmodule
