module steel_ball_roi_uart #(
    parameter CLK_HZ = 50000000,
    parameter BAUD = 115200
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_rx,
    output reg [10:0] O_left,
    output reg [10:0] O_top,
    output reg [10:0] O_right,
    output reg [10:0] O_bottom,
    output reg O_toggle,
    output reg O_motor_cmd_valid,
    output reg [2:0] O_motor_cmd
);
    localparam BIT_CYCLES = CLK_HZ / BAUD;
    localparam HALF_CYCLES = BIT_CYCLES / 2;
    reg rx0, rx1;
    reg [1:0] rx_state;
    reg [15:0] rx_count;
    reg [2:0] rx_bit;
    reg [7:0] rx_shift, rx_byte;
    reg rx_valid;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            rx0<=1; rx1<=1; rx_state<=0; rx_count<=0;
            rx_bit<=0; rx_shift<=0; rx_byte<=0; rx_valid<=0;
        end else begin
            rx0<=I_rx;
            rx1<=rx0;
            rx_valid<=0;
            case(rx_state)
                0: if(!rx1) begin rx_state<=1; rx_count<=HALF_CYCLES-1; end
                1: if(rx_count==0) begin
                       if(!rx1) begin rx_state<=2; rx_count<=BIT_CYCLES-1; rx_bit<=0; end
                       else rx_state<=0;
                   end else rx_count<=rx_count-1'b1;
                2: if(rx_count==0) begin
                       rx_shift[rx_bit]<=rx1;
                       rx_count<=BIT_CYCLES-1;
                       if(rx_bit==7) rx_state<=3;
                       else rx_bit<=rx_bit+1'b1;
                   end else rx_count<=rx_count-1'b1;
                3: if(rx_count==0) begin
                       rx_state<=0;
                       if(rx1) begin rx_byte<=rx_shift; rx_valid<=1; end
                   end else rx_count<=rx_count-1'b1;
            endcase
        end
    end

    function [4:0] hex_digit;
        input [7:0] ch;
        begin
            if(ch>=8'h30 && ch<=8'h39) hex_digit={1'b1,ch[3:0]};
            else if(ch>=8'h41 && ch<=8'h46) hex_digit={1'b1,ch[3:0]+4'd9};
            else if(ch>=8'h61 && ch<=8'h66) hex_digit={1'b1,ch[3:0]+4'd9};
            else hex_digit=5'd0;
        end
    endfunction
    reg in_packet;
    reg motor_prefix, motor_ready;
    reg [2:0] motor_code;
    reg [3:0] digit_count;
    reg [47:0] digits;
    wire [4:0] decoded=hex_digit(rx_byte);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            in_packet<=0; digit_count<=0; digits<=0;
            O_left<=0; O_top<=0; O_right<=11'd1279; O_bottom<=11'd719;
            O_toggle<=0;
        end else if(rx_valid) begin
            if(rx_byte==8'h52) begin
                in_packet<=1; digit_count<=0; digits<=0;
            end else if(in_packet && digit_count<12 && decoded[4]) begin
                digits<={digits[43:0],decoded[3:0]};
                digit_count<=digit_count+1'b1;
            end else if(in_packet && digit_count==12 && rx_byte==8'h0a) begin
                in_packet<=0;
                if(digits[47:36]<12'd1280 && digits[23:12]<12'd1280 &&
                   digits[35:24]<12'd720 && digits[11:0]<12'd720 &&
                   digits[47:36]<digits[23:12] && digits[35:24]<digits[11:0]) begin
                    O_left<=digits[46:36]; O_top<=digits[34:24];
                    O_right<=digits[22:12]; O_bottom<=digits[10:0];
                    O_toggle<=~O_toggle;
                end
            end else begin
                in_packet<=0;
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            motor_prefix<=0; motor_ready<=0; motor_code<=0;
            O_motor_cmd_valid<=0; O_motor_cmd<=0;
        end else begin
            O_motor_cmd_valid<=0;
            if(rx_valid) begin
                if(rx_byte==8'h4d) begin
                    motor_prefix<=1; motor_ready<=0;
                end else if(motor_prefix) begin
                    motor_prefix<=0;
                    motor_ready<=1;
                    case(rx_byte)
                        8'h41: motor_code<=3'd1;
                        8'h44: motor_code<=3'd2;
                        8'h5a: motor_code<=3'd3;
                        8'h2b: motor_code<=3'd4;
                        8'h2d: motor_code<=3'd5;
                        8'h53: motor_code<=3'd6;
                        default: motor_ready<=0;
                    endcase
                end else if(motor_ready) begin
                    motor_ready<=0;
                    if(rx_byte==8'h0a) begin
                        O_motor_cmd<=motor_code;
                        O_motor_cmd_valid<=1;
                    end
                end
            end
        end
    end
endmodule
