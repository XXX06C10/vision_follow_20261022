// 115200 8N1. P + seq16/center16/deadzone16/gain8/limit8/checksum8 as HEX + LF.
// XOR checksum covers the eight payload bytes; commit only on the terminating LF.
module vision_config_rx #(parameter integer UART_DIV=434,PACKET_TIMEOUT_CYCLES=5000000,DEFAULT_GAIN=4,DEFAULT_LIMIT=12,DEFAULT_MODE=1)(
    input wire clk,rst_n,rx,
    output reg [15:0] cfg_sequence=0,center=960,deadzone=20,
    output reg [7:0] gain=DEFAULT_GAIN,limit=DEFAULT_LIMIT,
    output reg [15:0] errors=0,
    output reg applied=0,
    output reg [15:0] cam_sequence=0,exposure=3600,sensor_gain=143,
    output reg [15:0] roi_left=0,roi_top=0,roi_right=1279,roi_bottom=719,
    input wire center_capture,input wire [15:0] captured_center,
    output reg center_applied=0,center_conflict=0,parameter_applied=0,
    output reg [15:0] follow_sequence=0,runtime_seconds=30,
    output reg [7:0] operating_mode=DEFAULT_MODE,damping=12,slew=2
);
    reg sync1=1,sync2=1;
    reg [1:0] rx_state=0;
    reg [15:0] baud_count=0;
    reg [2:0] bit_index=0;
    reg [7:0] shift=0,byte_value=0;
    reg byte_valid=0;
    always @(posedge clk) begin
        if(!rst_n) begin
            sync1<=1;sync2<=1;rx_state<=0;baud_count<=0;bit_index<=0;shift<=0;byte_value<=0;byte_valid<=0;
        end else begin
            sync1<=rx;sync2<=sync1;byte_valid<=0;
            case(rx_state)
                0:if(!sync2) begin baud_count<=UART_DIV/2-1;rx_state<=1;end
                1:if(baud_count==0) begin
                    if(!sync2) begin rx_state<=2;baud_count<=UART_DIV-1;bit_index<=0;end
                    else rx_state<=0;
                end else baud_count<=baud_count-1'b1;
                2:if(baud_count==0) begin
                    shift[bit_index]<=sync2;baud_count<=UART_DIV-1;
                    if(bit_index==7) rx_state<=3;else bit_index<=bit_index+1'b1;
                end else baud_count<=baud_count-1'b1;
                3:if(baud_count==0) begin
                    rx_state<=0;
                    if(sync2) begin byte_value<=shift;byte_valid<=1;end
                end else baud_count<=baud_count-1'b1;
            endcase
        end
    end
    function [4:0] hex_digit;
        input [7:0] ch;
        begin
            if(ch>="0" && ch<="9")hex_digit={1'b1,ch[3:0]};
            else if(ch>="A" && ch<="F")hex_digit={1'b1,ch[3:0]+4'd9};
            else if(ch>="a" && ch<="f")hex_digit={1'b1,ch[3:0]+4'd9};
            else hex_digit=0;
        end
    endfunction
    reg collecting=0,discard=0;
    reg [4:0] digits=0;
    reg [119:0] packet=0;
    reg camera_packet=0,control_packet=0;
    reg [31:0] packet_age=0;
    wire [4:0] digit=hex_digit(byte_value);
    wire [7:0] checksum=packet[71:64]^packet[63:56]^packet[55:48]^packet[47:40]^
        packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire bounds_ok=packet[55:40]<=1919 && packet[39:24]<=200 &&
        packet[22:16]<=12 && packet[15:8]<=60;
    wire [7:0] camera_checksum=packet[119:112]^packet[111:104]^packet[103:96]^packet[95:88]^
        packet[87:80]^packet[79:72]^packet[71:64]^packet[63:56]^packet[55:48]^packet[47:40]^
        packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire camera_bounds=packet[103:88]>=2 && packet[103:88]<=3600 &&
        packet[87:72]>=16 && packet[87:72]<=143 &&
        packet[71:56]<packet[39:24] && packet[39:24]<=1279 &&
        packet[55:40]<packet[23:8] && packet[23:8]<=719;
    wire [7:0] follow_checksum=packet[63:56]^packet[55:48]^packet[47:40]^packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire follow_bounds=packet[47:40]<=2 && packet[39:32]<=16 && packet[31:24]>=1 && packet[31:24]<=8 && packet[23:8]<=300;
    wire expired=collecting && packet_age>=PACKET_TIMEOUT_CYCLES-1;
    // A valid serial commit wins a same-cycle K3 collision. One center owner.
    wire uart_commit=byte_valid && !expired && collecting && !discard && byte_value==8'h0a &&
        ((!camera_packet && !control_packet && digits==18 && checksum==packet[7:0] && bounds_ok) ||
         (camera_packet && digits==30 && camera_checksum==packet[7:0] && camera_bounds) ||
         (control_packet && digits==16 && follow_checksum==packet[7:0] && follow_bounds));
    always @(posedge clk) begin
        if(!rst_n) begin
            cfg_sequence<=0;center<=960;deadzone<=20;gain<=DEFAULT_GAIN;limit<=DEFAULT_LIMIT;errors<=0;applied<=0;
            cam_sequence<=0;exposure<=3600;sensor_gain<=143;
            follow_sequence<=0;runtime_seconds<=30;operating_mode<=DEFAULT_MODE;damping<=12;slew<=2;control_packet<=0;
            roi_left<=0;roi_top<=0;roi_right<=1279;roi_bottom<=719;camera_packet<=0;
            collecting<=0;discard<=0;digits<=0;packet<=0;packet_age<=0;center_applied<=0;center_conflict<=0;parameter_applied<=0;
        end else begin
            applied<=0;center_applied<=0;center_conflict<=0;parameter_applied<=0;
            if(center_capture && captured_center<=1919)begin
                if(uart_commit)center_conflict<=1;
                else begin center<=captured_center;applied<=1;center_applied<=1;end
            end
            if(collecting) begin
                if(packet_age>=PACKET_TIMEOUT_CYCLES-1) begin
                    collecting<=0;discard<=!(byte_valid && byte_value==8'h0a);packet_age<=0;
                    if(errors!=16'hffff)errors<=errors+1'b1;
                end else packet_age<=packet_age+1'b1;
            end
            if(byte_valid && !expired) begin
                if(discard) begin
                    if(byte_value==8'h0a)discard<=0;
                end else if(!collecting) begin
                    if(byte_value=="P" || byte_value=="V" || byte_value=="S") begin
                        collecting<=1;digits<=0;packet<=0;packet_age<=0;camera_packet<=(byte_value=="V");control_packet<=(byte_value=="S");
                    end
                end else if(byte_value==8'h0a) begin
                    collecting<=0;packet_age<=0;
                    if(!camera_packet && !control_packet && digits==18 && checksum==packet[7:0] && bounds_ok) begin
                        cfg_sequence<=packet[71:56];center<=packet[55:40];deadzone<=packet[39:24];
                        gain<=packet[23:16];limit<=packet[15:8];applied<=1;parameter_applied<=1;
                    end else if(camera_packet && digits==30 && camera_checksum==packet[7:0] && camera_bounds)begin
                        applied<=1;cam_sequence<=packet[119:104];exposure<=packet[103:88];sensor_gain<=packet[87:72];
                        roi_left<=packet[71:56];roi_top<=packet[55:40];roi_right<=packet[39:24];roi_bottom<=packet[23:8];
                    end else if(control_packet && digits==16 && follow_checksum==packet[7:0] && follow_bounds)begin
                        follow_sequence<=packet[63:48];operating_mode<=packet[47:40];damping<=packet[39:32];
                        slew<=packet[31:24];runtime_seconds<=packet[23:8];applied<=1;
                    end else if(errors!=16'hffff)errors<=errors+1'b1;
                end else if(digit[4] && digits<(control_packet ? 16 : (camera_packet ? 30 : 18))) begin
                    packet<={packet[115:0],digit[3:0]};digits<=digits+1'b1;
                end else begin
                    collecting<=0;discard<=1;packet_age<=0;
                    if(errors!=16'hffff)errors<=errors+1'b1;
                end
            end
        end
    end
endmodule
