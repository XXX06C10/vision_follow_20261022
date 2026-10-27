module steel_ball_uart_diag #(
    parameter CLK_HZ = 112500000,
    parameter BAUD = 115200,
    parameter MOTOR_DIAG = 0
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_frame_done,
    input wire [21:0] I_red_pixels,
    input wire [21:0] I_largest_area,
    input wire [23:0] I_largest_perimeter,
    input wire [10:0] I_xmin,
    input wire [10:0] I_xmax,
    input wire [10:0] I_ymin,
    input wire [10:0] I_ymax,
    input wire [3:0] I_geometry,
    input wire [8:0] I_labels,
    input wire [16:0] I_fifo_peak,
    input wire [21:0] I_input_words,
    input wire [11:0] I_input_lines,
    input wire [15:0] I_detect_frames,
    input wire [11:0] I_detect_lines,
    input wire I_bright,
    input wire I_frame_hit,
    input wire I_box_valid,
    input wire I_ball,
    input wire I_bad_frame,
    input wire I_label_exhausted,
    input wire I_fault,
    input wire [1:0] I_error_code,
    input wire [10:0] I_ball_x,
    input wire [10:0] I_ball_y,
    input wire I_candidate,
    input wire I_confirmed,
    input wire [3:0] I_confidence,
    input wire [1:0] I_motion_valid,
    input wire [10:0] I_position_tenth_mm,
    input wire signed [15:0] I_speed_mm_s,
    input wire [15:0] I_ae,
    input wire [7:0] I_ag,
    input wire I_motor_en,
    input wire I_motor_busy,
    input wire I_motor_zero_valid,
    input wire signed [6:0] I_motor_position,
    input wire [7:0] I_motor_command_count,
    output wire O_tx
);
    localparam BAUD_DIV = (CLK_HZ + BAUD/2) / BAUD;
    localparam BAUD_BITS = $clog2(BAUD_DIV);
    reg [BAUD_BITS-1:0] baud_count;
    reg [3:0] bit_index;
    reg [9:0] shift_reg;
    reg active;
    reg pending;
    reg [7:0] char_index;
    reg [31:0] timeout_count;
    reg [15:0] frame_count;
    reg [15:0] frame_latch;
    reg [23:0] red_latch, area_latch, perimeter_latch;
    reg [11:0] xmin_latch, xmax_latch, ymin_latch, ymax_latch, labels_latch;
    reg [19:0] fifo_latch;
    reg [23:0] input_words_latch;
    reg [11:0] input_lines_latch, detect_lines_latch;
    reg [15:0] detect_frames_latch;
    reg [3:0] geometry_latch, error_latch;
    reg [7:0] status_latch;
    reg [11:0] ball_x_latch, ball_y_latch;
    reg [7:0] cv_latch, conf_latch;
    reg [7:0] motion_latch, ag_latch;
    reg [15:0] position_latch, ae_latch;
    reg [11:0] speed_latch;
    reg speed_negative_latch;
    reg motor_en_latch, motor_busy_latch, motor_zero_latch, motor_negative_latch;
    reg [7:0] motor_position_latch, motor_command_latch;
    reg [7:0] next_char;
    wire capture = !pending && !active && (I_frame_done || timeout_count==CLK_HZ-1);
    wire [15:0] speed_magnitude = I_speed_mm_s[15] ? -I_speed_mm_s : I_speed_mm_s;
    wire [6:0] motor_magnitude = I_motor_position[6] ? -I_motor_position : I_motor_position;
    assign O_tx = active ? shift_reg[0] : 1'b1;

    function [7:0] hex_char;
        input [3:0] value;
        begin
            hex_char = value<10 ? 8'h30+value : 8'h41+value-10;
        end
    endfunction

    always @* begin
        next_char=8'h20;
        case(char_index)
            0: next_char="F";
            1: next_char="=";
            7: next_char="R";
            8: next_char="=";
            16: next_char="A";
            17: next_char="=";
            25: next_char="P";
            26: next_char="=";
            34: next_char="X";
            35: next_char="=";
            39: next_char="-";
            44: next_char="Y";
            45: next_char="=";
            49: next_char="-";
            54: next_char="G";
            55: next_char="=";
            58: next_char="N";
            59: next_char="=";
            64: next_char="Q";
            65: next_char="=";
            72: next_char="S";
            73: next_char="=";
            77: next_char="E";
            78: next_char="=";
            81: next_char="V";
            82: next_char="=";
            90: next_char="T";
            91: next_char="=";
            96: next_char="D";
            97: next_char="=";
            103: next_char="L";
            104: next_char="=";
            109: next_char="B";
            110: next_char="=";
            114: next_char="-";
            119: next_char="C";
            120: next_char="=";
            124: next_char="F";
            125: next_char="=";
            129: next_char="K";
            130: next_char="=";
            134: next_char="U";
            135: next_char="=";
            141: next_char="W";
            142: next_char="=";
            143: next_char=speed_negative_latch ? "-" : "+";
            148: next_char="A";
            149: next_char="E";
            150: next_char="=";
            156: next_char="A";
            157: next_char="G";
            158: next_char="=";
            161: next_char=MOTOR_DIAG ? 8'h20 : 8'h0d;
            162: next_char=MOTOR_DIAG ? "M" : 8'h0a;
            163: next_char="E";
            164: next_char="=";
            165: next_char=hex_char({3'd0,motor_en_latch});
            167: next_char="M";
            168: next_char="B";
            169: next_char="=";
            170: next_char=hex_char({3'd0,motor_busy_latch});
            172: next_char="M";
            173: next_char="Z";
            174: next_char="=";
            175: next_char=hex_char({3'd0,motor_zero_latch});
            177: next_char="M";
            178: next_char="P";
            179: next_char="=";
            180: next_char=motor_negative_latch ? "-" : "+";
            184: next_char="M";
            185: next_char="C";
            186: next_char="=";
            189: next_char=8'h0d;
            190: next_char=8'h0a;
            default: next_char=8'h20;
        endcase
        if(char_index>=2 && char_index<=5)
            next_char=hex_char(frame_latch[4*(5-char_index) +: 4]);
        else if(char_index>=9 && char_index<=14)
            next_char=hex_char(red_latch[4*(14-char_index) +: 4]);
        else if(char_index>=18 && char_index<=23)
            next_char=hex_char(area_latch[4*(23-char_index) +: 4]);
        else if(char_index>=27 && char_index<=32)
            next_char=hex_char(perimeter_latch[4*(32-char_index) +: 4]);
        else if(char_index>=36 && char_index<=38)
            next_char=hex_char(xmin_latch[4*(38-char_index) +: 4]);
        else if(char_index>=40 && char_index<=42)
            next_char=hex_char(xmax_latch[4*(42-char_index) +: 4]);
        else if(char_index>=46 && char_index<=48)
            next_char=hex_char(ymin_latch[4*(48-char_index) +: 4]);
        else if(char_index>=50 && char_index<=52)
            next_char=hex_char(ymax_latch[4*(52-char_index) +: 4]);
        else if(char_index==56)
            next_char=hex_char(geometry_latch);
        else if(char_index>=60 && char_index<=62)
            next_char=hex_char(labels_latch[4*(62-char_index) +: 4]);
        else if(char_index>=66 && char_index<=70)
            next_char=hex_char(fifo_latch[4*(70-char_index) +: 4]);
        else if(char_index>=74 && char_index<=75)
            next_char=hex_char(status_latch[4*(75-char_index) +: 4]);
        else if(char_index==79)
            next_char=hex_char(error_latch);
        else if(char_index>=83 && char_index<=88)
            next_char=hex_char(input_words_latch[4*(88-char_index) +: 4]);
        else if(char_index>=92 && char_index<=94)
            next_char=hex_char(input_lines_latch[4*(94-char_index) +: 4]);
        else if(char_index>=98 && char_index<=101)
            next_char=hex_char(detect_frames_latch[4*(101-char_index) +: 4]);
        else if(char_index>=105 && char_index<=107)
            next_char=hex_char(detect_lines_latch[4*(107-char_index) +: 4]);
        else if(char_index>=111 && char_index<=113)
            next_char=hex_char(ball_x_latch[4*(113-char_index) +: 4]);
        else if(char_index>=115 && char_index<=117)
            next_char=hex_char(ball_y_latch[4*(117-char_index) +: 4]);
        else if(char_index>=121 && char_index<=122)
            next_char=hex_char(cv_latch[4*(122-char_index) +: 4]);
        else if(char_index>=126 && char_index<=127)
            next_char=hex_char(conf_latch[4*(127-char_index) +: 4]);
        else if(char_index>=131 && char_index<=132)
            next_char=hex_char(motion_latch[4*(132-char_index) +: 4]);
        else if(char_index>=136 && char_index<=139)
            next_char=hex_char(position_latch[4*(139-char_index) +: 4]);
        else if(char_index>=144 && char_index<=146)
            next_char=hex_char(speed_latch[4*(146-char_index) +: 4]);
        else if(char_index>=151 && char_index<=154)
            next_char=hex_char(ae_latch[4*(154-char_index) +: 4]);
        else if(char_index>=159 && char_index<=160)
            next_char=hex_char(ag_latch[4*(160-char_index) +: 4]);
        else if(char_index>=181 && char_index<=182)
            next_char=hex_char(motor_position_latch[4*(182-char_index) +: 4]);
        else if(char_index>=187 && char_index<=188)
            next_char=hex_char(motor_command_latch[4*(188-char_index) +: 4]);
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            baud_count<=0; bit_index<=0; shift_reg<=10'h3ff;
            active<=0; pending<=0; char_index<=0;
            timeout_count<=0; frame_count<=0; frame_latch<=0;
            red_latch<=0; area_latch<=0; perimeter_latch<=0;
            xmin_latch<=0; xmax_latch<=0; ymin_latch<=0; ymax_latch<=0;
            labels_latch<=0; fifo_latch<=0;
            input_words_latch<=0; input_lines_latch<=0;
            detect_frames_latch<=0; detect_lines_latch<=0;
            geometry_latch<=0; error_latch<=0; status_latch<=0;
            ball_x_latch<=0; ball_y_latch<=0; cv_latch<=0; conf_latch<=0;
            motion_latch<=0; position_latch<=0; speed_latch<=0;
            speed_negative_latch<=0; ae_latch<=0; ag_latch<=0;
            motor_en_latch<=0; motor_busy_latch<=0; motor_zero_latch<=0;
            motor_negative_latch<=0; motor_position_latch<=0; motor_command_latch<=0;
        end else begin
            if(I_frame_done) timeout_count<=0;
            else if(capture) timeout_count<=0;
            else if(timeout_count<CLK_HZ-1) timeout_count<=timeout_count+1'b1;
            if(capture) begin
                frame_latch<=frame_count;
                if(I_frame_done) frame_count<=frame_count+1'b1;
                red_latch<={2'b0,I_red_pixels};
                area_latch<={2'b0,I_largest_area};
                perimeter_latch<=I_largest_perimeter;
                xmin_latch<={1'b0,I_xmin}; xmax_latch<={1'b0,I_xmax};
                ymin_latch<={1'b0,I_ymin}; ymax_latch<={1'b0,I_ymax};
                geometry_latch<=I_geometry;
                labels_latch<={3'b0,I_labels};
                fifo_latch<={3'b0,I_fifo_peak};
                input_words_latch<={2'b0,I_input_words};
                input_lines_latch<=I_input_lines;
                detect_frames_latch<=I_detect_frames;
                detect_lines_latch<=I_detect_lines;
                status_latch<={!I_frame_done,I_fault,I_label_exhausted,I_bad_frame,
                    I_ball,I_box_valid,I_frame_hit,I_bright};
                error_latch<={2'b0,I_error_code};
                ball_x_latch<=I_frame_done ? {1'b0,I_ball_x} : 0;
                ball_y_latch<=I_frame_done ? {1'b0,I_ball_y} : 0;
                cv_latch<=I_frame_done ? {6'b0,I_confirmed,I_candidate} : 0;
                conf_latch<=I_frame_done ? {4'b0,I_confidence} : 0;
                motion_latch<=I_frame_done ? {6'b0,I_motion_valid} : 0;
                position_latch<=I_frame_done ? {5'b0,I_position_tenth_mm} : 0;
                speed_latch<=I_frame_done ? speed_magnitude[11:0] : 0;
                speed_negative_latch<=I_frame_done && I_motion_valid[1] && I_speed_mm_s[15];
                ae_latch<=I_ae;
                ag_latch<=I_ag;
                motor_en_latch<=I_motor_en;
                motor_busy_latch<=I_motor_busy;
                motor_zero_latch<=I_motor_zero_valid;
                motor_negative_latch<=I_motor_position[6];
                motor_position_latch<={1'b0,motor_magnitude};
                motor_command_latch<=I_motor_command_count;
                char_index<=0; pending<=1;
            end
            if(active) begin
                if(baud_count==BAUD_DIV-1) begin
                    baud_count<=0;
                    shift_reg<={1'b1,shift_reg[9:1]};
                    if(bit_index==9) active<=0;
                    else bit_index<=bit_index+1'b1;
                end else baud_count<=baud_count+1'b1;
            end else if(pending) begin
                shift_reg<={1'b1,next_char,1'b0};
                active<=1;
                baud_count<=0;
                bit_index<=0;
                if(char_index==(MOTOR_DIAG ? 190 : 162)) pending<=0;
                else char_index<=char_index+1'b1;
            end
        end
    end
endmodule
