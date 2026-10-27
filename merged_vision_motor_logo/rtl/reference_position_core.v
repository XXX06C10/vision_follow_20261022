// Small, finite, encoder-feedback moves. Relative origin is captured ONCE.
// No automatic resume or continuous hold correction. Faults latch until reload.
module position_core #(
    parameter integer DEBOUNCE_CYCLES=1000000,
    parameter integer ENABLE_CYCLES=5000000,
    parameter integer SETUP_CYCLES=1000000,
    parameter integer HALF_CYCLES=125000,
    parameter integer SETTLE_CYCLES=2500000,
    parameter integer TIMEOUT_CYCLES=100000000,
    parameter integer POSITION_LIMIT=16,
    parameter integer FEEDBACK_LIMIT=22,
    parameter integer MOVE_BUDGET=12,
    parameter integer NO_RESPONSE_LIMIT=3,
    parameter integer TARGET_STEP=5,
    parameter integer TARGET_LIMIT=5,
    parameter integer DEADBAND=2
)(
    input wire clk,rst_n,
    input wire [3:0] key_n,
    input wire signed [31:0] encoder_count,
    input wire [15:0] encoder_errors,
    output wire motor_en,motor_dir,motor_step,
    output reg signed [31:0] command_position=0,
    output reg signed [31:0] target=0,
    output wire signed [31:0] position,error,
    output reg [3:0] state=0,fault=0,
    output reg [7:0] spent=0,
    output wire reached,busy
);
    localparam [3:0] OFF=0,ARM=1,READY=2,SETUP=3,HIGH_PHASE=4,
                     LOW_PHASE=5,SETTLE=6,CHECK=7,FAULT=8,ALIGN=9;
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
    reg enabled=0,origin_valid=0,armed=0,dir_r=0,step_r=0;
    reg signed [31:0] origin=0;
    reg [15:0] error_reference=0;
    reg [31:0] enable_count=0,phase_count=0,action_timer=0;
    reg signed [32:0] before_position=0;
    reg [7:0] no_response=0,wrong_direction=0;
    wire signed [32:0] wide_position=origin_valid ?
        $signed({origin[31],origin})-$signed({encoder_count[31],encoder_count}) : 33'sd0;
    wire signed [32:0] wide_error=$signed({target[31],target})-wide_position;
    wire signed [32:0] displacement=wide_position-before_position;
    wire in_band=(wide_error<=DEADBAND && wide_error>=-DEADBAND);
    assign position=wide_position[31:0];
    assign error=wide_error[31:0];
    assign motor_en=rst_n & enable_raw & enabled & (fault==0);
    assign motor_step=motor_en & step_r;
    assign motor_dir=motor_en & dir_r;
    assign busy=motor_en && state>=SETUP && state<=CHECK;
    assign reached=motor_en && state==READY && in_band;

    always @(posedge clk) begin
        if(!rst_n) begin
            enabled<=0; origin_valid<=0; origin<=0; error_reference<=0;
            armed<=0; dir_r<=0; step_r<=0; state<=OFF; fault<=0;
            command_position<=0; target<=0; spent<=0;
            enable_count<=0; phase_count<=0; action_timer<=0;
            before_position<=0; no_response<=0; wrong_direction<=0;
        end else if(!enable_raw) begin
            enabled<=0; armed<=0; step_r<=0; state<=(fault==0 ? OFF : FAULT);
            enable_count<=0; phase_count<=0; action_timer<=0;
            // Preserve reference, target, command count, and fault across stops.
        end else if(fault!=0) begin
            enabled<=0; armed<=0; step_r<=0; state<=FAULT;
        end else if(!enabled) begin
            state<=ARM; step_r<=0; armed<=0;
            if(stable_valid && stable_keys==0) begin
                if(enable_count==ENABLE_CYCLES-1) begin
                    enabled<=1; state<=(origin_valid ? READY : ALIGN);
                    enable_count<=0; phase_count<=0;
                end else enable_count<=enable_count+1'b1;
            end else enable_count<=0;
        end else if(!origin_valid) begin
            // First enable may align the rotor electrically. Capture relative
            // zero AFTER EN has been high for a settling interval, never before.
            state<=ALIGN; step_r<=0; armed<=0;
            if(phase_count==ENABLE_CYCLES-1) begin
                origin<=encoder_count; error_reference<=encoder_errors;
                origin_valid<=1; phase_count<=0; state<=READY;
            end else phase_count<=phase_count+1'b1;
        end else if(encoder_errors!=error_reference) begin
            fault<=3; step_r<=0; state<=FAULT;
        end else if(wide_position>FEEDBACK_LIMIT || wide_position<-FEEDBACK_LIMIT) begin
            fault<=7; step_r<=0; state<=FAULT;
        end else if(busy && action_timer>=TIMEOUT_CYCLES-1) begin
            fault<=4; step_r<=0; state<=FAULT;
        end else begin
            if(busy) action_timer<=action_timer+1'b1;
            case(state)
                READY: begin
                    step_r<=0; phase_count<=0;
                    if(stable_valid) begin
                        if(stable_keys==0) armed<=1;
                        else if(stable_keys==3) armed<=0;
                        else if(armed) begin
                            armed<=0;
                            if((stable_keys[0] && target<TARGET_LIMIT) ||
                               (stable_keys[1] && target>-TARGET_LIMIT)) begin
                                target<=target+(stable_keys[0] ? TARGET_STEP : -TARGET_STEP);
                                spent<=0; no_response<=0; wrong_direction<=0;
                                action_timer<=0; state<=CHECK;
                            end
                        end
                    end
                end
                CHECK: begin
                    if(in_band) state<=READY;
                    else if(spent>=MOVE_BUDGET) begin fault<=5; state<=FAULT; end
                    else if((wide_error>0 && command_position>=POSITION_LIMIT) ||
                            (wide_error<0 && command_position<=-POSITION_LIMIT)) begin
                        fault<=6; state<=FAULT;
                    end else begin
                        dir_r<=(wide_error>0); before_position<=wide_position;
                        phase_count<=0; state<=SETUP;
                    end
                end
                SETUP: begin
                    if(phase_count==SETUP_CYCLES-1) begin
                        phase_count<=0; step_r<=1;
                        command_position<=command_position+(dir_r ? 32'sd1 : -32'sd1);
                        spent<=spent+1'b1; state<=HIGH_PHASE;
                    end else phase_count<=phase_count+1'b1;
                end
                HIGH_PHASE: begin
                    if(phase_count==HALF_CYCLES-1) begin
                        phase_count<=0; step_r<=0; state<=LOW_PHASE;
                    end else phase_count<=phase_count+1'b1;
                end
                LOW_PHASE: begin
                    if(phase_count==HALF_CYCLES-1) begin
                        phase_count<=0; state<=SETTLE;
                    end else phase_count<=phase_count+1'b1;
                end
                SETTLE: begin
                    if(phase_count==SETTLE_CYCLES-1) begin
                        phase_count<=0;
                        if(displacement==0) begin
                            no_response<=no_response+1'b1;
                            if(no_response>=NO_RESPONSE_LIMIT-1) begin fault<=1; state<=FAULT; end
                            else state<=CHECK;
                        end else if((dir_r && displacement<0) || (!dir_r && displacement>0)) begin
                            wrong_direction<=wrong_direction+1'b1;
                            no_response<=0;
                            if(wrong_direction>=1) begin fault<=2; state<=FAULT; end
                            else state<=CHECK;
                        end else begin
                            no_response<=0; wrong_direction<=0; state<=CHECK;
                        end
                    end else phase_count<=phase_count+1'b1;
                end
                default: begin fault<=8; step_r<=0; state<=FAULT; end
            endcase
        end
    end
endmodule
