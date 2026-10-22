// Guarded manual / continuous PD / explicit software-reference return modes.
module vision_monitor #(
    parameter integer DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,POR_CYCLES=512,
    UART_DIV=434,REPORT_CYCLES=12500000,MS_CYCLES=50000,STALE_MS=200,PACKET_TIMEOUT_CYCLES=5000000,
    SETUP_CYCLES=1000000,HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,DEFAULT_MODE=1,DEFAULT_GAIN=4,DEFAULT_LIMIT=12,CONTROL_MS=100,LOSS_GRACE_MS=500
)(
    input wire clk,external_rst_n,
    input wire [3:0] key_n,
    input wire [1:0] encoder_j1,encoder_j2,
    input wire frame,
    input wire [10:0] ball_x,ball_y,
    input wire confirmed,bad_frame,
    input wire [3:0] confidence,
    input wire rx,
    output wire tx,motor_en,motor_dir,motor_step,
    output wire [3:0] led,
    input wire [15:0] camera_applied_seq,roi_applied_seq,
    input wire settings_busy,
    output wire [15:0] cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom
);
    reg [31:0] por_count=0;
    always @(posedge clk)begin
        if(!external_rst_n)por_count<=0;
        else if(por_count<POR_CYCLES)por_count<=por_count+1'b1;
    end
    wire rst_n=external_rst_n && por_count>=POR_CYCLES;
    wire signed [31:0] count1,count2;
    wire [15:0] errors1,errors2;
    wire [1:0] ab1,ab2;
    encoder_ab_counter counter1(clk,rst_n,encoder_j1,count1,errors1,ab1);
    encoder_ab_counter counter2(clk,rst_n,encoder_j2,count2,errors2,ab2);
    wire [15:0] cfg_sequence,center,deadzone,rx_errors;
    wire [7:0] gain,limit;
    wire [10:0] x,y;
    wire center_applied,center_conflict;
    wire applied,parameter_applied;
    wire [15:0] follow_sequence,runtime_seconds;
    wire [7:0] operating_mode,damping,slew;
    reg center_confirmed=0;
    reg result_frame=0;
    always @(posedge clk)begin
        if(!rst_n)begin center_confirmed<=0;result_frame<=0;end
        else begin result_frame<=frame;if(center_applied)center_confirmed<=1;end
    end
    wire center_event,center_request;
    reg [15:0] center_count=0;
    reg [3:0] center_status=0;
    vision_config_rx #(.UART_DIV(UART_DIV),.PACKET_TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_MODE(DEFAULT_MODE),.DEFAULT_GAIN(DEFAULT_GAIN),.DEFAULT_LIMIT(DEFAULT_LIMIT)) config_rx(
        clk,rst_n,rx,cfg_sequence,center,deadzone,gain,limit,rx_errors,applied,
        cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,
        center_request,{5'b0,x},center_applied,center_conflict,parameter_applied,follow_sequence,runtime_seconds,operating_mode,damping,slew);
    wire valid;
    wire [15:0] age,frames;
    wire signed [7:0] manual_target,target,drive_target;
    wire [3:0] frame_confidence;
    wire frame_bad;
    vision_shadow_target #(.MS_CYCLES(MS_CYCLES),.STALE_MS(STALE_MS)) shadow(
        clk,rst_n,frame,ball_x,ball_y,confirmed,bad_frame,confidence,center,deadzone,gain,limit,
        x,y,valid,age,frames,manual_target,frame_confidence,frame_bad);
    reg fresh_after_settings=0;
    always @(posedge clk)begin
        if(!rst_n || applied || center_request || settings_busy)fresh_after_settings<=0;
        else if(frame && confirmed && !bad_frame && confidence>=8 && ball_x<=1919 && ball_y<=1079)fresh_after_settings<=1;
    end
    wire settings_ready=!settings_busy && fresh_after_settings;
    wire [3:0] state,fault,cancel_reason,reject_reason,core_reject,supervisor_reject,stop_reason;
    wire auto_run,request,following,external_cancel,key2_event;
    wire [3:0] external_cancel_reason;
    wire signed [15:0] velocity;
    wire [15:0] elapsed_seconds;
    wire [31:0] total_pulses;
    assign reject_reason=supervisor_reject!=0 ? supervisor_reject : core_reject;
    wire enable_request,origin_valid,ready,busy,reached;
    wire [7:0] spent;
    wire [15:0] accepted_count,completed_count;
    wire signed [31:0] command_position,locked_target,position,move_error;
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) center_key(clk,rst_n,key_n[2],center_event);
    wire hold_valid;
    vision_result_guard #(.MS_CYCLES(MS_CYCLES),.LOSS_GRACE_MS(LOSS_GRACE_MS)) loss_guard(
        .clk(clk),.rst_n(rst_n),.clear(applied || center_request || settings_busy),
        .frame(result_frame),.valid(valid),.hold_valid(hold_valid));
    wire incoming_bad=frame && (!confirmed || bad_frame || confidence<8 || ball_x>1919 || ball_y>1079);
    wire executor_valid=(auto_run && operating_mode==1) ? hold_valid : (valid && !incoming_bad);
    assign center_request=center_event && key_n[3] && fault==0 && !busy && !auto_run && settings_ready &&
        !applied && valid && age<STALE_MS && !incoming_bad;
    always @(posedge clk)begin
        if(!rst_n)begin center_count<=0;center_status<=0;end
        else if(center_applied)begin center_count<=center_count+1'b1;center_status<=1;end
        else if(center_conflict)center_status<=7;
        else if(center_event)begin
            if(!key_n[3])center_status<=6;
            else if(fault!=0)center_status<=5;
            else if(busy || auto_run)center_status<=4;
            else if(!settings_ready || applied)center_status<=3;
            else if(!valid || age>=STALE_MS || incoming_bad)center_status<=2;
        end
    end
    enable_latch #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) enable_control(
        clk,rst_n,key_n[0],key_n[3],fault!=0,enable_request);
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) key2(clk,rst_n,key_n[1],key2_event);
    vision_follow_control #(.MS_CYCLES(MS_CYCLES),.CONTROL_MS(CONTROL_MS)) supervisor(
        .clk(clk),.rst_n(rst_n),.key_event(key2_event),.k4_n(key_n[3]),.enable_request(enable_request),
        .config_changed(applied || center_request),.visual_valid(valid && !incoming_bad),.follow_valid(hold_valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.center_confirmed(center_confirmed),.frame(result_frame),
        .x(x),.center(center),.deadzone(deadzone),.runtime_seconds(runtime_seconds),.gain(gain),.limit(limit),
        .mode(operating_mode),.damping(damping),.slew(slew),.manual_target(manual_target),
        .core_ready(ready),.core_busy(busy),.fault(fault),.step(motor_step),.position(position),
        .request(request),.following(following),.external_cancel(external_cancel),.external_cancel_reason(external_cancel_reason),
        .run(auto_run),.stop_reason(stop_reason),.reject_reason(supervisor_reject),.drive_target(drive_target),
        .candidate(target),.velocity(velocity),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses));
    vision_servo_core #(.EXTERNAL_CONTROL(1),.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.ENABLE_CYCLES(ENABLE_CYCLES),
        .SETUP_CYCLES(SETUP_CYCLES),.HALF_CYCLES(HALF_CYCLES),.SETTLE_CYCLES(SETTLE_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)) motor_core(
        .clk(clk),.rst_n(rst_n),.enable_request(enable_request),.k4_n(key_n[3]),.k2_n(1'b1),
        .request(request),.following(following),.external_cancel(external_cancel),.external_cancel_reason(external_cancel_reason),
        .encoder_count(count1),.encoder_errors(errors1),.visual_valid(executor_valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.config_changed(applied || center_request),.visual_target(drive_target),
        .motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
        .command_position(command_position),.locked_target(locked_target),.position(position),.error(move_error),
        .state(state),.fault(fault),.cancel_reason(cancel_reason),.reject_reason(core_reject),.spent(spent),
        .origin_valid(origin_valid),.ready(ready),.busy(busy),.reached(reached),
        .accepted_count(accepted_count),.completed_count(completed_count));
    wire request_ready=(auto_run || (ready && valid && !incoming_bad && age<STALE_MS)) && settings_ready && !applied && !center_request && fault==0 && (operating_mode!=1 || center_confirmed);
    assign led={fault!=0,request_ready,(busy || auto_run),motor_en};
    vision_report_tx #(.UART_DIV(UART_DIV),.REPORT_CYCLES(REPORT_CYCLES)) report_tx(
        .clk(clk),.rst_n(rst_n),.x(x),.y(y),.valid(valid),.age(age),.frames(frames),
        .enabled(motor_en),.origin_valid(origin_valid),.fault(fault),.count1(count1),.count2(count2),
        .errors1(errors1),.errors2(errors2),.position(position),.target(target),
        .center(center),.deadzone(deadzone),.gain(gain),.limit(limit),.cfg_sequence(cfg_sequence),.rx_errors(rx_errors),
        .quality(frame_confidence),.bad(frame_bad),.cam_sequence(cam_sequence),.exposure(exposure),.sensor_gain(sensor_gain),
        .roi_left(roi_left),.roi_top(roi_top),.roi_right(roi_right),.roi_bottom(roi_bottom),
        .camera_applied_seq(camera_applied_seq),.roi_applied_seq(roi_applied_seq),.settings_busy(settings_busy),.locked_target(locked_target),.move_error(move_error),.move_state(state),
        .move_busy(busy),.reached(reached),.request_ready(request_ready),.spent(spent),.command_position(command_position),
        .cancel_reason(cancel_reason),.reject_reason(reject_reason),.accepted_count(accepted_count),.completed_count(completed_count),
        .center_count(center_count),.center_status(center_status),
        .operating_mode(operating_mode),.auto_run(auto_run),.damping(damping),.slew(slew),.runtime_seconds(runtime_seconds),
        .follow_sequence(follow_sequence),.center_confirmed(center_confirmed),.velocity(velocity),.drive_target(drive_target),
        .stop_reason(stop_reason),.auto_reject(supervisor_reject),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses),.tx(tx));
endmodule
