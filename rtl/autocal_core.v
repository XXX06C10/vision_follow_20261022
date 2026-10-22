// One-shot, finite 3200-pulse out-and-back ratio measurement. Not homing.
// Encoder is monitored throughout; no extra correction pulses alter the ratio.
module autocal_core #(
    parameter integer DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,
    SETUP_CYCLES=1000000,HALF_CYCLES=125000,SETTLE_CYCLES=50000000,
    LEG_TIMEOUT_CYCLES=1000000000,PULSES=3200,WINDOW_PULSES=16,
    MIN_WINDOW_COUNTS=4,MAX_WINDOW_COUNTS=48,MIN_LEG_COUNTS=3000,MAX_LEG_COUNTS=5000,
    POSITION_MAX=6000,POSITION_MIN=-64,RETURN_TOLERANCE=8
)(
    input wire clk,rst_n,
    input wire [3:0] key_n,
    input wire signed [31:0] encoder_count,
    input wire [15:0] encoder_errors,
    output wire motor_en,motor_dir,motor_step,busy,
    output reg [3:0] state=0,fault=0,
    output reg done=0,reverse=0,
    output reg signed [31:0] command_position=0,return_error=0,
    output reg [15:0] leg_emitted=0,total_emitted=0,
    output reg [15:0] forward_count=0,reverse_count=0,estimate_count=0
);
    localparam [3:0] OFF=0,ARM=1,READY=2,SETUP=3,HIGH_PHASE=4,LOW_PHASE=5,
                     END_SETTLE=6,RESULT=7,DONE=8,FAULT=9,ALIGN=10;
    wire enable_raw=(~key_n[0]) & key_n[3];
    reg [1:0] sync1=0,sync2=0,candidate=0,stable_keys=0;
    reg [31:0] debounce_count=0;
    reg stable_valid=0;
    always @(posedge clk) begin
        if(!rst_n || !enable_raw) begin
            sync1<=0; sync2<=0; candidate<=0; stable_keys<=0;
            debounce_count<=0; stable_valid<=0;
        end else begin
            sync1<=~key_n[2:1]; sync2<=sync1;
            if(sync2!=candidate) begin candidate<=sync2; debounce_count<=0; end
            else if(debounce_count<DEBOUNCE_CYCLES-1) debounce_count<=debounce_count+1'b1;
            else begin stable_keys<=candidate; stable_valid<=1; end
        end
    end
    reg enabled=0,armed=0,started=0,dir_r=0,step_r=0;
    reg [31:0] enable_count=0,phase_count=0,leg_timer=0;
    reg [15:0] window_emitted=0,error_reference=0;
    reg signed [31:0] start_count=0,forward_end=0,window_reference=0;
    wire signed [32:0] position=$signed({start_count[31],start_count})-$signed({encoder_count[31],encoder_count});
    wire signed [32:0] progress=reverse ?
        $signed({encoder_count[31],encoder_count})-$signed({window_reference[31],window_reference}) :
        $signed({window_reference[31],window_reference})-$signed({encoder_count[31],encoder_count});
    wire signed [32:0] back_count=$signed({encoder_count[31],encoder_count})-$signed({forward_end[31],forward_end});
    wire signed [32:0] count_sum=$signed({17'b0,forward_count})+back_count;
    wire signed [31:0] leg_difference=$signed({16'b0,forward_count})-$signed({16'b0,reverse_count});
    assign motor_en=rst_n & enable_raw & enabled & (fault==0);
    assign motor_dir=motor_en & dir_r;
    assign motor_step=motor_en & step_r;
    assign busy=motor_en && started && !done;

    always @(posedge clk) begin
        if(!rst_n) begin
            enabled<=0; armed<=0; started<=0; dir_r<=0; step_r<=0;
            state<=OFF; fault<=0; done<=0; reverse<=0;
            enable_count<=0; phase_count<=0; leg_timer<=0;
            command_position<=0; return_error<=0;
            leg_emitted<=0; total_emitted<=0; forward_count<=0; reverse_count<=0; estimate_count<=0;
            window_emitted<=0; error_reference<=0; start_count<=0; forward_end<=0; window_reference<=0;
        end else if(!enable_raw) begin
            enabled<=0; armed<=0; step_r<=0; enable_count<=0; phase_count<=0;
            if(started && !done && fault==0) begin fault<=9; state<=FAULT; end
            else state<=(fault!=0 ? FAULT : (done ? DONE : OFF));
        end else if(fault!=0) begin
            enabled<=0; armed<=0; step_r<=0; state<=FAULT;
        end else if(done) begin
            step_r<=0; armed<=0; state<=DONE;
            // Re-enable HOLD after a completed test, never repeat its pulses.
            if(!enabled) begin
                if(stable_valid && stable_keys==0) begin
                    if(enable_count==ENABLE_CYCLES-1) begin enabled<=1; enable_count<=0; end
                    else enable_count<=enable_count+1'b1;
                end else enable_count<=0;
            end
        end else if(!enabled) begin
            state<=ARM; step_r<=0; armed<=0;
            if(stable_valid && stable_keys==0) begin
                if(enable_count==ENABLE_CYCLES-1) begin
                    enabled<=1; enable_count<=0; phase_count<=0; state<=ALIGN;
                end else enable_count<=enable_count+1'b1;
            end else enable_count<=0;
        end else if(state==ALIGN) begin
            if(phase_count==ENABLE_CYCLES-1) begin phase_count<=0; state<=READY; end
            else phase_count<=phase_count+1'b1;
        end else if(started && encoder_errors!=error_reference) begin
            fault<=3; step_r<=0; state<=FAULT;
        end else if(started && (position>POSITION_MAX || position<POSITION_MIN)) begin
            fault<=7; step_r<=0; state<=FAULT;
        end else if(started && leg_timer>=LEG_TIMEOUT_CYCLES-1) begin
            fault<=4; step_r<=0; state<=FAULT;
        end else if(started && (command_position<0 || command_position>PULSES || total_emitted>2*PULSES)) begin
            fault<=5; step_r<=0; state<=FAULT;
        end else begin
            if(started) leg_timer<=leg_timer+1'b1;
            case(state)
                READY: begin
                    step_r<=0;
                    if(stable_valid) begin
                        if(stable_keys==0) armed<=1;
                        else if(stable_keys==3) armed<=0;
                        else if(armed) begin
                            armed<=0;
                            if(stable_keys==1) begin
                                start_count<=encoder_count; window_reference<=encoder_count;
                                error_reference<=encoder_errors;
                                started<=1; reverse<=0; dir_r<=1; phase_count<=0; leg_timer<=0;
                                state<=SETUP;
                            end
                        end
                    end
                end
                SETUP: begin
                    if(phase_count==SETUP_CYCLES-1) begin
                        phase_count<=0;
                        if(total_emitted>=2*PULSES || (!reverse && command_position>=PULSES) ||
                           (reverse && command_position<=0)) begin fault<=5; state<=FAULT; end
                        else begin
                            step_r<=1; command_position<=command_position+(reverse ? -32'sd1 : 32'sd1);
                            leg_emitted<=leg_emitted+1'b1; total_emitted<=total_emitted+1'b1;
                            window_emitted<=window_emitted+1'b1; state<=HIGH_PHASE;
                        end
                    end else phase_count<=phase_count+1'b1;
                end
                HIGH_PHASE: begin
                    if(phase_count==HALF_CYCLES-1) begin phase_count<=0; step_r<=0; state<=LOW_PHASE; end
                    else phase_count<=phase_count+1'b1;
                end
                LOW_PHASE: begin
                    if(phase_count==HALF_CYCLES-1) begin
                        phase_count<=0;
                        if(window_emitted>=WINDOW_PULSES && progress< -2) begin fault<=2; state<=FAULT; end
                        else if(window_emitted>=WINDOW_PULSES && progress<=2) begin fault<=1; state<=FAULT; end
                        else if(window_emitted>=WINDOW_PULSES &&
                                (progress<MIN_WINDOW_COUNTS || progress>MAX_WINDOW_COUNTS)) begin fault<=8; state<=FAULT; end
                        else begin
                            if(window_emitted>=WINDOW_PULSES) begin
                                window_reference<=encoder_count; window_emitted<=0;
                            end
                            if(leg_emitted==PULSES) state<=END_SETTLE;
                            else if(leg_emitted>PULSES || total_emitted>=2*PULSES ||
                                    (!reverse && command_position>=PULSES) || (reverse && command_position<=0)) begin
                                fault<=5; state<=FAULT;
                            end else begin
                                step_r<=1; command_position<=command_position+(reverse ? -32'sd1 : 32'sd1);
                                leg_emitted<=leg_emitted+1'b1; total_emitted<=total_emitted+1'b1;
                                window_emitted<=(window_emitted>=WINDOW_PULSES ? 16'd1 : window_emitted+1'b1);
                                state<=HIGH_PHASE;
                            end
                        end
                    end else phase_count<=phase_count+1'b1;
                end
                END_SETTLE: begin
                    if(phase_count==SETTLE_CYCLES-1) begin
                        phase_count<=0;
                        if(!reverse) begin
                            if(position<MIN_LEG_COUNTS || position>MAX_LEG_COUNTS) begin fault<=6; state<=FAULT; end
                            else begin
                                forward_count<=position[15:0]; forward_end<=encoder_count;
                                reverse<=1; dir_r<=0; leg_emitted<=0; window_emitted<=0;
                                window_reference<=encoder_count; leg_timer<=0; state<=SETUP;
                            end
                        end else begin
                            if(back_count<0 || back_count>65535) begin fault<=6; state<=FAULT; end
                            else begin
                                reverse_count<=back_count[15:0]; return_error<=position[31:0];
                                estimate_count<=count_sum>>1; state<=RESULT;
                            end
                        end
                    end else phase_count<=phase_count+1'b1;
                end
                RESULT: begin
                    if(reverse_count<MIN_LEG_COUNTS || reverse_count>MAX_LEG_COUNTS ||
                       return_error>RETURN_TOLERANCE || return_error< -RETURN_TOLERANCE ||
                       leg_difference>RETURN_TOLERANCE || leg_difference< -RETURN_TOLERANCE ||
                       command_position!=0 || total_emitted!=2*PULSES) begin fault<=6; state<=FAULT; end
                    else begin done<=1; state<=DONE; end
                end
                default: begin fault<=5; step_r<=0; state<=FAULT; end
            endcase
        end
    end
endmodule
