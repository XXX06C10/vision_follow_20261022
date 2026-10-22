from pathlib import Path
root=Path(__file__).resolve().parent
p=root/'rtl/vision_monitor.v';text=p.read_text()
text=text.replace('// Milestone 1: observe targets; physical STEP is fixed low. No automatic rotation.', '// K2-triggered finite visual move; no continuous visual following.')
text=text.replace('UART_DIV=434,REPORT_CYCLES=12500000,MS_CYCLES=50000,STALE_MS=200,PACKET_TIMEOUT_CYCLES=5000000',
 'UART_DIV=434,REPORT_CYCLES=12500000,MS_CYCLES=50000,STALE_MS=200,PACKET_TIMEOUT_CYCLES=5000000,\n    SETUP_CYCLES=1000000,HALF_CYCLES=125000,SETTLE_CYCLES=2500000,TIMEOUT_CYCLES=100000000')
start=text.index('    reg [3:0] monitor_fault');end=text.index('    wire [15:0] cfg_sequence',start)
text=text[:start]+text[end:]
start=text.index('    assign led=');end=text.index('    vision_report_tx',start)
text=text[:start]+'''    reg fresh_after_settings=0;
    always @(posedge clk)begin
        if(!rst_n || applied || settings_busy)fresh_after_settings<=0;
        else if(frame)fresh_after_settings<=confirmed && !bad_frame && confidence>=8 && ball_x<=1919 && ball_y<=1079;
    end
    wire settings_ready=!settings_busy && fresh_after_settings;
    wire [3:0] state,fault,cancel_reason,reject_reason;
    wire enable_request,origin_valid,ready,busy,reached;
    wire [7:0] spent;
    wire [15:0] accepted_count,completed_count;
    wire signed [31:0] command_position,locked_target,position,move_error;
    enable_latch #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)) enable_control(
        clk,rst_n,key_n[0],key_n[3],fault!=0,enable_request);
    vision_move_core #(.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.ENABLE_CYCLES(ENABLE_CYCLES),
        .SETUP_CYCLES(SETUP_CYCLES),.HALF_CYCLES(HALF_CYCLES),.SETTLE_CYCLES(SETTLE_CYCLES),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)) motor_core(
        .clk(clk),.rst_n(rst_n),.enable_request(enable_request),.k4_n(key_n[3]),.k2_n(key_n[1]),
        .encoder_count(count1),.encoder_errors(errors1),.visual_valid(valid),.visual_stale(age>=STALE_MS),
        .settings_ready(settings_ready),.config_changed(applied),.visual_target(target),
        .motor_en(motor_en),.motor_dir(motor_dir),.motor_step(motor_step),
        .command_position(command_position),.locked_target(locked_target),.position(position),.error(move_error),
        .state(state),.fault(fault),.cancel_reason(cancel_reason),.reject_reason(reject_reason),.spent(spent),
        .origin_valid(origin_valid),.ready(ready),.busy(busy),.reached(reached),
        .accepted_count(accepted_count),.completed_count(completed_count));
    wire request_ready=ready && valid && age<STALE_MS && settings_ready && !applied;
    assign led={fault!=0,request_ready,busy,motor_en};
'''+text[end:]
text=text.replace('.settings_busy(settings_busy),.tx(tx));', '''.settings_busy(settings_busy),.locked_target(locked_target),.move_error(move_error),.move_state(state),
        .move_busy(busy),.reached(reached),.request_ready(request_ready),.spent(spent),.command_position(command_position),
        .cancel_reason(cancel_reason),.reject_reason(reject_reason),.accepted_count(accepted_count),.completed_count(completed_count),.tx(tx));''')
p.write_text(text)
p=root/'rtl/vision_config_rx.v';text=p.read_text().replace('cam_sequence<=packet[119:104];exposure<=packet[103:88];sensor_gain<=packet[87:72];',
 'applied<=1;cam_sequence<=packet[119:104];exposure<=packet[103:88];sensor_gain<=packet[87:72];');p.write_text(text)
p=root/'compose.py';text=p.read_text().replace("'autocal_core.v'","'vision_move_core.v'");p.write_text(text)
p=root/'generate_uart.py';text=p.read_text().replace("('CB',1,'settings_busy','hex')]", """('CB',1,'settings_busy','hex'),
 ('MT',8,'locked_target','hex'),('ME',8,'move_error','hex'),('MS',1,'move_state','hex'),('MB',1,'move_busy','hex'),
 ('OK',1,'reached','hex'),('RD',1,'request_ready','hex'),('SP',2,'spent','hex'),('MC',8,'command_position','hex'),
 ('CR',1,'cancel_reason','hex'),('RJ',1,'reject_reason','hex'),('AC',4,'accepted_count','hex'),('DC',4,'completed_count','hex')]""")
text=text.replace('8\'h32','8\'h33').replace('reg [7:0] char_index','reg [8:0] char_index').replace('input [7:0] index;','input [8:0] index;')
text=text.replace('    input wire settings_busy,','''    input wire settings_busy,
    input wire signed [31:0] locked_target,move_error,command_position,
    input wire [3:0] move_state,cancel_reason,reject_reason,
    input wire move_busy,reached,request_ready,
    input wire [7:0] spent,
    input wire [15:0] accepted_count,completed_count,''')
p.write_text(text)
# This project explicitly implements the authorized finite-motion milestone.
p=root/'AGENTS.md';text=p.read_text();start=text.index('## Current milestone status')
text=text[:start]+'''## Authorized milestone (2026-10-21)

The user explicitly approved unloaded, K2-triggered single visual moves. Archive the previous Chinese observation version first. Implement finite moves using position_core lineage, K1 latch/K4 raw stop, targets +/-5, deadband2, pulse budget12, timeout2s and the original fault guards. No continuous follow or automatic restart. Vision invalid/stale or any parameter commit cancels without returning to zero and requires a new released-then-pressed K2. Preserve origin and command count across stops. Keep exposure3600/gain143/ROI GUI and protocol packets. Deliver VIS=3 plus matching Chinese panel/bit and board steps. New move firmware is not physically validated yet.
'''
p.write_text(text)
print('Integrated finite visual command core and VIS=3 generator')
