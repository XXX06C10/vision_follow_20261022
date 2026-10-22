from pathlib import Path
root=Path(__file__).resolve().parent
p=root/'rtl/vision_monitor.v';s=p.read_text(encoding='utf-8')
s=s.replace('TIMEOUT_CYCLES=250000000\n','TIMEOUT_CYCLES=250000000,DEFAULT_MODE=1,DEFAULT_GAIN=4,DEFAULT_LIMIT=12,CONTROL_MS=100\n')
s=s.replace('wire applied;','''wire applied,parameter_applied;
    wire [15:0] follow_sequence,runtime_seconds;
    wire [7:0] operating_mode,damping,slew;
    reg center_confirmed=0;
    reg result_frame=0;
    always @(posedge clk)begin
        if(!rst_n)begin center_confirmed<=0;result_frame<=0;end
        else begin result_frame<=frame;if(center_applied || parameter_applied)center_confirmed<=1;end
    end''')
# Move declarations that the new status block consumes before their first use.
s=s.replace('    wire applied,parameter_applied;','    wire center_applied,center_conflict;\n    wire applied,parameter_applied;')
s=s.replace('wire center_event,center_request,center_applied,center_conflict;','wire center_event,center_request;')
s=s.replace('.PACKET_TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES)) config_rx(','.PACKET_TIMEOUT_CYCLES(PACKET_TIMEOUT_CYCLES),.DEFAULT_MODE(DEFAULT_MODE),.DEFAULT_GAIN(DEFAULT_GAIN),.DEFAULT_LIMIT(DEFAULT_LIMIT)) config_rx(')
s=s.replace('center_request,{5\'b0,x},center_applied,center_conflict);',"center_request,{5'b0,x},center_applied,center_conflict,parameter_applied,follow_sequence,runtime_seconds,operating_mode,damping,slew);")
s=s.replace('wire signed [7:0] target;','wire signed [7:0] manual_target,target,drive_target;')
s=s.replace('x,y,valid,age,frames,target,frame_confidence,frame_bad);','x,y,valid,age,frames,manual_target,frame_confidence,frame_bad);')
s=s.replace('wire [3:0] state,fault,cancel_reason,reject_reason;','''wire [3:0] state,fault,cancel_reason,reject_reason,core_reject,supervisor_reject,stop_reason;
    wire auto_run,request,following,external_cancel,key2_event;
    wire [3:0] external_cancel_reason;
    wire signed [15:0] velocity;
    wire [15:0] elapsed_seconds;
    wire [31:0] total_pulses;
    assign reject_reason=supervisor_reject!=0 ? supervisor_reject : core_reject;''')
s=s.replace('assign center_request=center_event && key_n[3] && fault==0 && !busy && settings_ready &&','assign center_request=center_event && key_n[3] && fault==0 && !busy && !auto_run && settings_ready &&')
s=s.replace('else if(busy)center_status<=4;','else if(busy || auto_run)center_status<=4;')
start=s.index('    vision_move_core #(');end=s.index('    wire request_ready=',start)
s=s[:start]+'''    vision_center_key #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) key2(clk,rst_n,key_n[1],key2_event);
    vision_follow_control #(.MS_CYCLES(MS_CYCLES),.CONTROL_MS(CONTROL_MS)) supervisor(
        .clk(clk),.rst_n(rst_n),.key_event(key2_event),.k4_n(key_n[3]),.enable_request(enable_request),
        .config_changed(applied || center_request),.visual_valid(valid),.visual_stale(age>=STALE_MS),
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
        .encoder_count(count1),.encoder_errors(errors1),.visual_valid(valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.config_changed(applied || center_request),.visual_target(drive_target),
        .motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
        .command_position(command_position),.locked_target(locked_target),.position(position),.error(move_error),
        .state(state),.fault(fault),.cancel_reason(cancel_reason),.reject_reason(core_reject),.spent(spent),
        .origin_valid(origin_valid),.ready(ready),.busy(busy),.reached(reached),
        .accepted_count(accepted_count),.completed_count(completed_count));
'''+s[end:]
s=s.replace('wire request_ready=ready && valid && age<STALE_MS && settings_ready && !applied && !center_request;','wire request_ready=(auto_run || ready) && valid && age<STALE_MS && settings_ready && !applied && !center_request && fault==0 && (operating_mode!=1 || center_confirmed);')
s=s.replace('assign led={fault!=0,request_ready,busy,motor_en};','assign led={fault!=0,request_ready,(busy || auto_run),motor_en};')
s=s.replace('.center_count(center_count),.center_status(center_status),.tx(tx));','''.center_count(center_count),.center_status(center_status),
        .operating_mode(operating_mode),.auto_run(auto_run),.damping(damping),.slew(slew),.runtime_seconds(runtime_seconds),
        .follow_sequence(follow_sequence),.center_confirmed(center_confirmed),.velocity(velocity),.drive_target(drive_target),
        .stop_reason(stop_reason),.auto_reject(supervisor_reject),.elapsed_seconds(elapsed_seconds),.total_pulses(total_pulses),.tx(tx));''')
p.write_text(s,encoding='utf-8')
p=root/'generate_uart.py';s=p.read_text(encoding='utf-8')
s=s.replace("('KC',1,'center_status','hex')]", """('KC',1,'center_status','hex'),
 ('MD',1,'operating_mode','hex'),('AR',1,'auto_run','hex'),('KD',2,'damping','hex'),('SL',2,'slew','hex'),
 ('RT',4,'runtime_seconds','hex'),('DS',4,'follow_sequence','hex'),('CZ',1,'center_confirmed','hex'),
 ('VE',4,'velocity','hex'),('AT',2,'drive_target','hex'),('SR',1,'stop_reason','hex'),('AJ',1,'auto_reject','hex'),
 ('EL',4,'elapsed_seconds','hex'),('TS',8,'total_pulses','hex')]""")
s=s.replace("8'h34","8'h35")
s=s.replace('    input wire [3:0] center_status,','''    input wire [3:0] center_status,
    input wire [7:0] operating_mode,damping,slew,
    input wire auto_run,center_confirmed,
    input wire [15:0] runtime_seconds,follow_sequence,elapsed_seconds,
    input wire signed [15:0] velocity,
    input wire signed [7:0] drive_target,
    input wire [3:0] stop_reason,auto_reject,
    input wire [31:0] total_pulses,''')
p.write_text(s,encoding='utf-8')
p=root/'decode_serial.py';s=p.read_text(encoding='utf-8').replace('VIS=4','VIS=5').replace("'4' if kind=='literal'","'5' if kind=='literal'")
s=s.replace("('MC',32)]","('MC',32),('VE',16),('AT',8)]");s=s.replace("'MB','OK','RD']","'MB','OK','RD','AR','CZ']")
p.write_text(s,encoding='utf-8')
print('Integrated supervisor/core and VIS5 metrics')
