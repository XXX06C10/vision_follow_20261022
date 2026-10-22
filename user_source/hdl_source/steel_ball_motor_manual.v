module steel_ball_motor_manual #(
    parameter CLK_HZ = 50000000,
    parameter MOVE_STEPS = 4,
    parameter LIMIT_STEPS = 16,
    parameter HALF_PERIOD_CYCLES = CLK_HZ / 200,
    parameter WAKE_CYCLES = CLK_HZ / 50,
    parameter DIR_SETUP_CYCLES = CLK_HZ / 1000
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_cmd_valid,
    input wire [2:0] I_cmd,
    output reg O_step,
    output reg O_dir,
    output reg O_en,
    output wire O_busy,
    output reg O_zero_valid,
    output reg signed [6:0] O_position,
    output reg [7:0] O_command_count
);
    localparam [1:0] IDLE=0, DIR_WAIT=1, HIGH=2, LOW=3;
    localparam integer PW = (HALF_PERIOD_CYCLES<=1) ? 1 : $clog2(HALF_PERIOD_CYCLES);
    localparam integer WW = (WAKE_CYCLES<=1) ? 1 : $clog2(WAKE_CYCLES+1);
    localparam integer DW = (DIR_SETUP_CYCLES<=1) ? 1 : $clog2(DIR_SETUP_CYCLES);
    localparam integer SW = (MOVE_STEPS<=1) ? 1 : $clog2(MOVE_STEPS+1);
    reg [1:0] state;
    reg [PW-1:0] phase_count;
    reg [WW-1:0] wake_count;
    reg [DW-1:0] dir_count;
    reg [SW-1:0] steps_left;
    reg stop_pending;
    wire jog_plus = I_cmd==3'd4;
    wire jog_minus = I_cmd==3'd5;
    wire can_jog = O_en && O_zero_valid && state==IDLE &&
        wake_count==WAKE_CYCLES &&
        ((jog_plus && O_position<=LIMIT_STEPS-MOVE_STEPS) ||
         (jog_minus && O_position>=-LIMIT_STEPS+MOVE_STEPS));
    wire take_arm = I_cmd_valid && I_cmd==3'd1 && !O_en;
    wire take_disarm = I_cmd_valid && I_cmd==3'd2;
    wire take_zero = I_cmd_valid && I_cmd==3'd3 && O_en &&
        state==IDLE && wake_count==WAKE_CYCLES;
    wire take_jog = I_cmd_valid && (jog_plus || jog_minus) && can_jog;
    wire take_stop = I_cmd_valid && I_cmd==3'd6;
    assign O_busy = state!=IDLE;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_step<=0; O_dir<=0; O_en<=0; O_zero_valid<=0;
            O_position<=0; O_command_count<=0;
            state<=IDLE; phase_count<=0; wake_count<=0;
            dir_count<=0; steps_left<=0; stop_pending<=0;
        end else begin
            if(!O_en) wake_count<=0;
            else if(wake_count<WAKE_CYCLES) wake_count<=wake_count+1'b1;
            if(take_disarm) begin
                O_en<=0; O_step<=0; O_zero_valid<=0; O_position<=0;
                state<=IDLE; phase_count<=0; wake_count<=0;
                dir_count<=0; steps_left<=0; stop_pending<=0;
                O_command_count<=O_command_count+1'b1;
            end else if(take_arm) begin
                O_en<=1; O_step<=0; O_zero_valid<=0; O_position<=0;
                state<=IDLE; phase_count<=0; wake_count<=0;
                dir_count<=0; steps_left<=0; stop_pending<=0;
                O_command_count<=O_command_count+1'b1;
            end else if(take_stop) begin
                O_command_count<=O_command_count+1'b1;
                if(state==HIGH) stop_pending<=1;
                else begin
                    O_step<=0; state<=IDLE; steps_left<=0;
                    phase_count<=0; dir_count<=0; stop_pending<=0;
                end
            end else if(take_zero) begin
                O_position<=0; O_zero_valid<=1;
                O_command_count<=O_command_count+1'b1;
            end else if(take_jog) begin
                O_dir<=jog_plus;
                steps_left<=MOVE_STEPS;
                dir_count<=0; phase_count<=0;
                state<=DIR_WAIT;
                O_command_count<=O_command_count+1'b1;
            end else begin
                case(state)
                    IDLE: O_step<=0;
                    DIR_WAIT: begin
                        if(dir_count==DIR_SETUP_CYCLES-1) begin
                            O_step<=1;
                            O_position<=O_position+(O_dir ? 7'sd1 : -7'sd1);
                            steps_left<=steps_left-1'b1;
                            phase_count<=0;
                            state<=HIGH;
                        end else dir_count<=dir_count+1'b1;
                    end
                    HIGH: begin
                        if(phase_count==HALF_PERIOD_CYCLES-1) begin
                            O_step<=0; phase_count<=0;
                            if(stop_pending || steps_left==0) begin
                                state<=IDLE; steps_left<=0; stop_pending<=0;
                            end else state<=LOW;
                        end else phase_count<=phase_count+1'b1;
                    end
                    LOW: begin
                        if(phase_count==HALF_PERIOD_CYCLES-1) begin
                            O_step<=1;
                            O_position<=O_position+(O_dir ? 7'sd1 : -7'sd1);
                            steps_left<=steps_left-1'b1;
                            phase_count<=0; state<=HIGH;
                        end else phase_count<=phase_count+1'b1;
                    end
                    default: begin
                        O_en<=0; O_step<=0; O_zero_valid<=0;
                        state<=IDLE;
                    end
                endcase
            end
        end
    end
endmodule
