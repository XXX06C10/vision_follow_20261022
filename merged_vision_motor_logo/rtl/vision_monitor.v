// Guarded manual / continuous PD / explicit software-reference return modes.
module vision_monitor #(
    parameter integer DEBOUNCE_CYCLES=1000000,ENABLE_CYCLES=5000000,POR_CYCLES=512,
    UART_DIV=434,REPORT_CYCLES=12500000,MS_CYCLES=50000,STALE_MS=200,PACKET_TIMEOUT_CYCLES=5000000,
    SETUP_CYCLES=1000000,HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=250000000,DEFAULT_MODE=1,DEFAULT_GAIN=3,DEFAULT_LIMIT=32,CONTROL_MS=20,LOSS_GRACE_MS=500,
    DEFAULT_DEADZONE=5,DEFAULT_SLEW=8,DEFAULT_RUNTIME=60,CASCADE_DEFAULT=0,CASCADE_PRESET=0,REGION_PROBE=0,REGION_RETURN=0,ASYM_RANGE=0,
    FOLLOW_SETUP_CYCLES=(SETUP_CYCLES>=4 ? SETUP_CYCLES/4 : 1),FOLLOW_SETTLE_CYCLES=(SETTLE_CYCLES>=5 ? SETTLE_CYCLES/5 : 1)
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
    localparam TW=ASYM_RANGE ? 16 : 8;
    wire limits_ready;wire signed [31:0] limit_anchor,command_anchor,work_min,work_max,feedback_min,feedback_max,command_min,command_max;
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
    vision_config_rx #(.UART_DIV(UART_DIV),.PACKET_TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_MODE(DEFAULT_MODE),.DEFAULT_GAIN(DEFAULT_GAIN),.DEFAULT_LIMIT(DEFAULT_LIMIT),
        .DEFAULT_DEADZONE(DEFAULT_DEADZONE),.DEFAULT_SLEW(DEFAULT_SLEW),.DEFAULT_RUNTIME(DEFAULT_RUNTIME)) config_rx(
        clk,rst_n,rx,cfg_sequence,center,deadzone,gain,limit,rx_errors,applied,
        cam_sequence,exposure,sensor_gain,roi_left,roi_top,roi_right,roi_bottom,
        center_request,{5'b0,x},center_applied,center_conflict,parameter_applied,follow_sequence,runtime_seconds,operating_mode,damping,slew);
    wire valid;
    wire [15:0] age,frames;
    wire signed [7:0] manual_target;wire signed [TW-1:0] target,drive_target;
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
    wire auto_run,request,following,external_cancel,key2_event,neutral_ready;
    wire [3:0] external_cancel_reason;
    wire signed [15:0] velocity;
    wire [15:0] elapsed_seconds;
    wire [31:0] total_pulses;
    assign reject_reason=supervisor_reject!=0 ? supervisor_reject : core_reject;
    wire enable_request,origin_valid,ready,busy,reached;
    wire [7:0] spent;
    wire [15:0] accepted_count,completed_count;
    wire c_applied,c_commit,c_diagnostics,c_polarity;
    wire [15:0] c_sequence,c_errors,c_kp,c_kv,c_ki,c_inner_kp,c_ff,c_max_hz,c_slew;
    wire signed [15:0] c_bias;
    wire [7:0] c_selection,c_delay,c_amplitude;
    wire c_selected=c_selection!=0 && operating_mode==1;
    wire effective_settings_ready=c_selected && c_selection==2 ? !settings_busy : settings_ready;
    wire l_run,l_request,l_following,l_external_cancel,l_neutral_ready;
    wire [3:0] l_external_reason,l_stop,l_reject;
    wire signed [7:0] l_target,l_drive;
    wire signed [15:0] l_velocity;
    wire [15:0] l_elapsed;wire [31:0] l_pulses;
    wire n_run,n_request,n_following,n_external_cancel,n_neutral_ready,n_updated,n_stable,n_checkpoint,n_quiescing;
    wire [1:0] n_control_phase,n_outer_phase,n_region_phase;
    wire signed [31:0] n_outer_raw,n_region_raw,n_region_velocity;
    wire n_region_changed,n_region_holding,n_region_active;
    wire [15:0] n_region_hz,n_region_slew;
    wire [3:0] n_external_reason,n_stop,n_reject,n_cal_status,n_cal_phase;
    wire signed [TW-1:0] n_target,n_drive,n_neutral;
    wire signed [15:0] n_velocity_legacy,n_frequency;
    wire signed [31:0] n_raw_q8,n_reference_q8,n_velocity;
    wire [15:0] n_elapsed;wire [31:0] n_pulses,n_time,n_frame_time;
    wire legacy_tx,legacy_busy,legacy_wants,diag_tx,diag_busy,event_tx,event_busy,event_wants,event_overflow;
    wire p_run,p_request,p_cancel;wire [3:0] p_status,p_phase,p_reason,p_reject;wire signed [7:0] p_target;
    wire [7:0] d_pending;wire signed [15:0] d_commands,d_delta;wire d_consistent,d_wrong;
    reg c_fresh=0,c_profile_ready=CASCADE_PRESET,c_builtin=CASCADE_PRESET;
    assign n_region_active=REGION_RETURN && c_selected && c_builtin;
    assign n_raw_q8=ASYM_RANGE ? n_region_raw : n_region_active ? n_region_raw : n_outer_raw;
    assign n_control_phase=ASYM_RANGE ? n_region_phase : n_region_active ? n_region_phase : n_outer_phase;
    always @(posedge clk)begin
      if(!rst_n)begin c_profile_ready<=CASCADE_PRESET;c_builtin<=CASCADE_PRESET;end
      else begin
        if(c_applied)begin c_builtin<=0;c_profile_ready<=c_selection==1 && (c_kp!=0 || c_kv!=0);end
        else if(applied || center_request || settings_busy)c_profile_ready<=c_builtin;
      end
      if(!rst_n || c_applied || applied || settings_busy)c_fresh<=0;
      else if(result_frame && valid)c_fresh<=1;
    end
    assign auto_run=REGION_PROBE ? p_run : c_selected ? n_run : l_run;
    assign request=REGION_PROBE ? p_request : c_selected ? n_request : l_request;
    assign following=REGION_PROBE ? 1'b0 : c_selected ? n_following : l_following;
    assign external_cancel=REGION_PROBE ? p_cancel : c_selected ? n_external_cancel : l_external_cancel;
    assign external_cancel_reason=REGION_PROBE ? p_reason : c_selected ? n_external_reason : l_external_reason;
    assign neutral_ready=c_selected ? n_neutral_ready : l_neutral_ready;
    assign stop_reason=REGION_PROBE ? p_reason : c_selected ? n_stop : l_stop;
    assign supervisor_reject=REGION_PROBE ? p_reject : c_selected ? n_reject : l_reject;
    assign drive_target=REGION_PROBE ? p_target : c_selected ? n_drive : l_drive;
    assign target=REGION_PROBE ? p_target : c_selected ? ((valid || n_run || c_selection==2) ? n_target : 8'sd0) : l_target;
    assign velocity=c_selected ? n_velocity_legacy : l_velocity;
    assign elapsed_seconds=c_selected ? n_elapsed : l_elapsed;
    assign total_pulses=c_selected ? n_pulses : l_pulses;
    wire signed [(ASYM_RANGE ? 31 : 7):0] report_target=target,report_drive=drive_target,report_neutral=n_neutral;
    assign tx=legacy_busy ? legacy_tx : (event_busy ? event_tx : diag_tx);

    wire signed [31:0] command_position,locked_target,position,move_error;
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) center_key(clk,rst_n,key_n[2],center_event);
    wire hold_valid;
    vision_result_guard #(.MS_CYCLES(MS_CYCLES),.LOSS_GRACE_MS(LOSS_GRACE_MS)) loss_guard(
        .clk(clk),.rst_n(rst_n),.clear(applied || center_request || settings_busy),
        .frame(result_frame),.valid(valid),.hold_valid(hold_valid));
    wire incoming_bad=frame && (!confirmed || bad_frame || confidence<8 || ball_x>1919 || ball_y>1079);
    wire executor_valid=REGION_PROBE ? (p_run && c_selection==2 ? 1'b1 : (valid && !incoming_bad)) : (c_selected && c_selection==2 && n_run) ? 1'b1 : ((auto_run && operating_mode==1) ? hold_valid : (valid && !incoming_bad));
    assign center_request=center_event && !c_commit && key_n[3] && fault==0 && !busy && !auto_run && settings_ready &&
        !applied && valid && age<STALE_MS && !incoming_bad && (!ASYM_RANGE || ready);
    always @(posedge clk)begin
        if(!rst_n)begin center_count<=0;center_status<=0;end
        else if(center_applied)begin center_count<=center_count+1'b1;center_status<=(position>(ASYM_RANGE ? work_max : 60) || position<(ASYM_RANGE ? work_min : -60)) ? 8 : 1;end
        else if(center_conflict || (center_event && c_commit))center_status<=7;
        else if(center_event)begin
            if(!key_n[3])center_status<=6;
            else if(fault!=0)center_status<=5;
            else if(busy || auto_run)center_status<=4;
            else if(!settings_ready || applied)center_status<=3;
            else if(!valid || age>=STALE_MS || incoming_bad)center_status<=2;
        end
    end
    enable_latch #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.HOLD_ON_FAULT(1)) enable_control(
        clk,rst_n,key_n[0],key_n[3],fault!=0,enable_request);
    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) key2(clk,rst_n,key_n[1],key2_event);
    vision_follow_control #(.MS_CYCLES(MS_CYCLES),.CONTROL_MS(CONTROL_MS)) supervisor(
        .clk(clk),.rst_n(rst_n),.key_event(key2_event && !c_selected && !REGION_PROBE && !ASYM_RANGE),.k4_n(key_n[3]),.enable_request(enable_request),
        .config_changed(applied || center_request || c_applied),.visual_valid(valid && !incoming_bad),.follow_valid(hold_valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.center_confirmed(center_confirmed),.frame(result_frame),.neutral_capture(center_applied),
        .x(x),.center(center),.deadzone(deadzone),.runtime_seconds(runtime_seconds),.gain(gain),.limit(limit),
        .mode(operating_mode),.damping(damping),.slew(slew),.manual_target(manual_target),
        .core_ready(ready),.core_busy(busy),.fault(fault),.step(motor_step),.position(position),
        .request(l_request),.following(l_following),.neutral_ready(l_neutral_ready),.external_cancel(l_external_cancel),.external_cancel_reason(l_external_reason),
        .run(l_run),.stop_reason(l_stop),.reject_reason(l_reject),.drive_target(l_drive),
        .candidate(l_target),.velocity(l_velocity),.elapsed_seconds(l_elapsed),.total_pulses(l_pulses));
    vision_servo_core #(.TARGET_BITS(TW),.ASYM_LIMITS(ASYM_RANGE),.EXTERNAL_CONTROL(1),.HOLD_ON_FAULT(1),.FAST_CAPABLE(1),.CHECKPOINT_CAPABLE(1),.FAST_MS_CYCLES(MS_CYCLES),.TAIL_CHECKPOINT(REGION_RETURN),.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.ENABLE_CYCLES(ENABLE_CYCLES),
        .SETUP_CYCLES(SETUP_CYCLES),.HALF_CYCLES(HALF_CYCLES),.SETTLE_CYCLES(SETTLE_CYCLES),
        .FOLLOW_SETUP_CYCLES(FOLLOW_SETUP_CYCLES),.FOLLOW_SETTLE_CYCLES(FOLLOW_SETTLE_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)) motor_core(
        .clk(clk),.rst_n(rst_n),.enable_request(enable_request),.k4_n(key_n[3]),.k2_n(1'b1),
        .request(request),.following(following),.external_cancel(external_cancel),.external_cancel_reason(external_cancel_reason),
        .fast_mode(!REGION_PROBE && c_selected && (n_run || busy)),.fast_rate_hz(n_frequency),.checkpoint(!REGION_PROBE && c_selected && n_checkpoint),
        .encoder_count(count1),.encoder_errors(errors1),.visual_valid(executor_valid),.visual_stale(c_selected && c_selection==2 ? 1'b0 : age>=STALE_MS),
        .target_min(work_min),.target_max(work_max),.feedback_min(feedback_min),.feedback_max(feedback_max),.command_min(command_min),.command_max(command_max),
        .settings_ready(effective_settings_ready),.config_changed(applied || center_request || c_applied),.visual_target(drive_target),
        .motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
        .command_position(command_position),.locked_target(locked_target),.position(position),.error(move_error),
        .state(state),.fault(fault),.cancel_reason(cancel_reason),.reject_reason(core_reject),.spent(spent),
        .origin_valid(origin_valid),.ready(ready),.busy(busy),.reached(reached),
        .accepted_count(accepted_count),.completed_count(completed_count),.diag_pending(d_pending),.diag_commands(d_commands),.diag_delta(d_delta),.diag_consistent(d_consistent),.diag_wrong(d_wrong));
    wire request_ready=(!ASYM_RANGE || (n_region_active && limits_ready)) && (c_selected ? (ready && (c_selection==2 || (valid && !incoming_bad && age<STALE_MS && c_fresh))) : (auto_run || (ready && valid && !incoming_bad && age<STALE_MS))) && effective_settings_ready && !applied && !c_applied && !center_request && fault==0 && (operating_mode!=1 || (center_confirmed && neutral_ready && (!c_selected || c_selection!=1 || c_profile_ready)));
    assign led={fault!=0,request_ready,(busy || auto_run),motor_en};
    vision_report_tx #(.WIDE_RANGE(ASYM_RANGE),.UART_DIV(UART_DIV),.REPORT_CYCLES(REPORT_CYCLES),.SHARED_TX(1)) report_tx(
        .clk(clk),.rst_n(rst_n),.x(x),.y(y),.valid(valid),.age(age),.frames(frames),
        .enabled(motor_en),.origin_valid(origin_valid),.fault(fault),.count1(count1),.count2(count2),
        .errors1(errors1),.errors2(errors2),.position(position),.target(report_target),
        .center(center),.deadzone(deadzone),.gain(gain),.limit(limit),.cfg_sequence(cfg_sequence),.rx_errors(rx_errors),
        .quality(frame_confidence),.bad(frame_bad),.cam_sequence(cam_sequence),.exposure(exposure),.sensor_gain(sensor_gain),
        .roi_left(roi_left),.roi_top(roi_top),.roi_right(roi_right),.roi_bottom(roi_bottom),
        .camera_applied_seq(camera_applied_seq),.roi_applied_seq(roi_applied_seq),.settings_busy(settings_busy),.locked_target(locked_target),.move_error(move_error),.move_state(state),
        .move_busy(busy),.reached(reached),.request_ready(request_ready),.spent(spent),.command_position(command_position),
        .cancel_reason(cancel_reason),.reject_reason(reject_reason),.accepted_count(accepted_count),.completed_count(completed_count),
        .center_count(center_count),.center_status(center_status),
        .operating_mode(operating_mode),.auto_run(auto_run),.damping(damping),.slew(slew),.runtime_seconds(runtime_seconds),
        .follow_sequence(follow_sequence),.center_confirmed(center_confirmed),.velocity(velocity),.drive_target(report_drive),
        .stop_reason(stop_reason),.auto_reject(supervisor_reject),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses),.grant(!diag_busy && !event_busy),.busy(legacy_busy),.wants(legacy_wants),.tx(legacy_tx));

    vision_cascade_config #(.UART_DIV(UART_DIV),.TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_SELECTION(CASCADE_DEFAULT),.DEFAULT_PRESET(CASCADE_PRESET)) cascade_config(
     .clk(clk),.rst_n(rst_n),.rx(rx),.applied(c_applied),.commit_now(c_commit),.config_seq(c_sequence),.errors(c_errors),.selection(c_selection),
     .diagnostics(c_diagnostics),.polarity(c_polarity),.kp(c_kp),.kv(c_kv),.ki(c_ki),.inner_kp(c_inner_kp),.feedforward(c_ff),
     .max_hz(c_max_hz),.slew_rate(c_slew),.bias(c_bias),.delay_ms(c_delay),.amplitude(c_amplitude));
    vision_cascade_outer #(.WIDE_RANGE(ASYM_RANGE),.NEUTRAL_BITS(TW),.MS_CYCLES(MS_CYCLES),.REGION_VELOCITY(REGION_RETURN)) cascade_outer(
     .clk(clk),.rst_n(rst_n),.run(n_run && c_selected),.frame(result_frame),.valid(valid && !incoming_bad),.stale(age>=STALE_MS),
     .clear(applied || c_applied || settings_busy),.neutral_capture(center_applied),.polarity(c_polarity),.x(x),.center(center),.deadzone(deadzone),
     .kp(c_kp),.kv(c_kv),.ki(c_ki),.limit(limit),.delay_ms(c_delay),.bias(c_bias),.position(position),.work_min(work_min),.work_max(work_max),.target_q8(n_outer_raw),
     .velocity_q4(n_velocity),.legacy_velocity_q4(n_velocity_legacy),.neutral(n_neutral),.neutral_ready(n_neutral_ready),
     .updated(n_updated),.stable(n_stable),.time_ms(n_time),.frame_time_ms(n_frame_time),
     .predictive(ASYM_RANGE ? 1'b1 : (c_selected && c_builtin)),.control_phase(n_outer_phase));
    vision_cascade_control #(.WIDE_RANGE(ASYM_RANGE),.TARGET_BITS(TW),.MS_CYCLES(MS_CYCLES)) cascade_control(
     .clk(clk),.rst_n(rst_n),.key_event(key2_event && c_selected && !REGION_PROBE),.k4_n(key_n[3]),.enable_request(enable_request),
     .settings_ready(effective_settings_ready && (c_selection==2 || c_fresh)),.center_confirmed(center_confirmed),
     .valid(valid && !incoming_bad),.hold_valid(hold_valid),.stale(age>=STALE_MS),.frame(result_frame),
     .config_changed(applied || c_applied || center_request || !c_selected),.neutral_capture(center_applied),
     .region_holding(n_region_holding),.profile_ready(c_profile_ready && (!ASYM_RANGE || (limits_ready && c_builtin))),.selection(c_selection),.amplitude(c_amplitude),.limit(limit),.max_hz(n_region_active && n_region_hz<c_max_hz ? n_region_hz : c_max_hz),.inner_kp(c_inner_kp),.feedforward(c_ff),
     .slew_rate(n_region_active && n_region_slew<c_slew ? n_region_slew : c_slew),.runtime_seconds(runtime_seconds),.kp(c_kp),.kv(c_kv),.roi_left(roi_left),.roi_right(roi_right),.center(center),.x(x),
     .position(position),.command_position(command_position),.raw_target_q8(n_raw_q8),.outer_updated(n_region_active ? n_region_changed : n_updated),.neutral_ready(n_neutral_ready),.neutral(n_neutral),.work_min(work_min),.work_max(work_max),
     .core_ready(ready),.core_busy(busy),.core_spent(spent),.fault(fault),.step(motor_step),.checkpoint(n_checkpoint),.diagnostic_quiescing(n_quiescing),.run(n_run),.request(n_request),.following(n_following),
     .external_cancel(n_external_cancel),.external_cancel_reason(n_external_reason),.stop_reason(n_stop),.reject_reason(n_reject),
     .cal_status(n_cal_status),.cal_phase(n_cal_phase),.reference_q8(n_reference_q8),.frequency_hz(n_frequency),.target(n_target),.drive_target(n_drive),
     .elapsed_seconds(n_elapsed),.total_pulses(n_pulses));
    vision_region_hold #(.WIDE_RANGE(ASYM_RANGE),.NEUTRAL_BITS(TW)) region_hold(
     .clk(clk),.rst_n(rst_n),.run(n_run && n_region_active),.updated(n_updated),.valid(valid && !incoming_bad),.stale(age>=STALE_MS),
     .clear(applied || c_applied || settings_busy),.neutral_capture(center_applied),.polarity(c_polarity),.x(x),.center(center),.limit(limit),
     .velocity_q4(n_velocity),.work_min(work_min),.work_max(work_max),.neutral(n_neutral),.target_q8(n_region_raw),.changed(n_region_changed),.holding(n_region_holding),.phase(n_region_phase),
     .max_hz(n_region_hz),.slew_rate(n_region_slew),.filtered_velocity_q4(n_region_velocity));
    vision_asym_limits asym_limits(.clk(clk),.rst_n(rst_n),.capture(center_applied && ASYM_RANGE),.ready(ready),.motor_en(motor_en),.position(position),.command_position(command_position),
     .qualified(limits_ready),.anchor(limit_anchor),.command_anchor(command_anchor),.work_min(work_min),.work_max(work_max),.feedback_min(feedback_min),.feedback_max(feedback_max),.command_min(command_min),.command_max(command_max));
    vision_region_probe #(.MS_CYCLES(MS_CYCLES)) region_probe(
     .clk(clk),.rst_n(rst_n),.key_event(key2_event && REGION_PROBE),.k4_n(key_n[3]),.enable_request(enable_request),
     .settings_ready(settings_ready),.valid(valid && !incoming_bad),.stale(age>=STALE_MS),.frame(result_frame),
     .changed(applied || c_applied || center_request),.center_confirmed(center_confirmed),.neutral_ready(n_neutral_ready),
     .core_ready(ready),.core_busy(busy),.event_overflow(event_overflow),.selection(c_selection),.neutral(n_neutral[7:0]),.position(position),.fault(fault),
     .time_ms(n_time),.x(x),.y(y),.center(center),.roi_left(roi_left),.roi_right(roi_right),.roi_top(roi_top),.roi_bottom(roi_bottom),
     .run(p_run),.request(p_request),.cancel(p_cancel),.status(p_status),.phase(p_phase),.reason(p_reason),.reject(p_reject),.target(p_target));
    vision_probe_events #(.UART_DIV(UART_DIV),.MS_CYCLES(MS_CYCLES)) probe_events(
     .clk(clk),.rst_n(rst_n),.enabled(REGION_PROBE!=0),.grant(!legacy_busy && !legacy_wants && !diag_busy),
     .time_ms(n_time),.step(motor_step),.dir(motor_dir),.position(position),.command_position(command_position),.state(state),
     .pending(d_pending),.commands(d_commands),.delta(d_delta),.diagnostic_flags({event_overflow,n_quiescing,d_consistent,d_wrong,fault}),
     .phase(p_phase),.status(p_status),.selection(c_selection),.reason(p_reason),.changed(c_applied || applied || center_request),
     .tx(event_tx),.busy(event_busy),.wants(event_wants),.overflow(event_overflow));
    // [trim] cascade UART diagnostics removed to recover slice area.
    // The legacy report_tx / probe event stream are kept unchanged.
    assign diag_tx   = 1'b1;
    assign diag_busy = 1'b0;
endmodule
