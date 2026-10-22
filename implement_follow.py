"""One-time source migration; normal rebuilds use compose/build only."""
from pathlib import Path
root=Path(__file__).resolve().parent
assert not (root/'rtl/vision_servo_core.v').exists(),'Already migrated'
p=root/'rtl/vision_move_core.v';s=p.read_text(encoding='utf-8')
s=s.replace('module vision_move_core','module vision_servo_core').replace('TARGET_LIMIT=60,DEADBAND=2','TARGET_LIMIT=60,DEADBAND=2,EXTERNAL_CONTROL=0')
s=s.replace('input wire clk,rst_n,enable_request,k4_n,k2_n,','input wire clk,rst_n,enable_request,k4_n,k2_n,\n input wire request,following,external_cancel,input wire [3:0] external_cancel_reason,')
s=s.replace('wire permit=visual_valid && !visual_stale && settings_ready && !config_changed;','wire extended_cancel=EXTERNAL_CONTROL && external_cancel;\n wire permit=visual_valid && !visual_stale && settings_ready && !config_changed && !extended_cancel;')
s=s.replace('reg previous_pressed=0,press_event=0;','reg previous_pressed=0,press_event=0;\n wire command_event=EXTERNAL_CONTROL ? request : press_event;\n wire command_armed=EXTERNAL_CONTROL ? 1\'b1 : key_armed;')
# Declare before command_armed references this register.
s=s.replace('reg previous_pressed=0,press_event=0;','reg key_armed=0;\n reg previous_pressed=0,press_event=0;').replace('reg enabled=0,key_armed=0,active=0','reg enabled=0,active=0')
s=s.replace('if(press_event)begin','if(command_event)begin').replace('else if(!key_armed)reject_reason','else if(!command_armed)reject_reason').replace('if(press_event && key_armed &&','if(command_event && command_armed &&')
s=s.replace('cancel_reason<=config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3);','cancel_reason<=extended_cancel ? external_cancel_reason : (config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3));')
s=s.replace('CHECK:begin\n      if(in_band)', '''CHECK:begin
      if(EXTERNAL_CONTROL && following && (visual_target>TARGET_LIMIT || visual_target< -TARGET_LIMIT))begin
       fault<=8;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
      end else if(EXTERNAL_CONTROL && following && locked_target!={{24{visual_target[7]}},visual_target})begin
       // Retarget only after the previous pulse/feedback phase. Never reset budget/time.
       locked_target<={{24{visual_target[7]}},visual_target};
      end else if(in_band)''')
(root/'rtl/vision_servo_core.v').write_text(s,encoding='utf-8')
p=root/'rtl/vision_config_rx.v';s=p.read_text(encoding='utf-8')
s=s.replace('PACKET_TIMEOUT_CYCLES=5000000)','PACKET_TIMEOUT_CYCLES=5000000,DEFAULT_GAIN=4,DEFAULT_LIMIT=12,DEFAULT_MODE=1)')
s=s.replace('gain=7,limit=5','gain=DEFAULT_GAIN,limit=DEFAULT_LIMIT')
s=s.replace('output reg center_applied=0,center_conflict=0','''output reg center_applied=0,center_conflict=0,parameter_applied=0,
    output reg [15:0] follow_sequence=0,runtime_seconds=30,
    output reg [7:0] operating_mode=DEFAULT_MODE,damping=6,slew=2''')
s=s.replace('reg camera_packet=0;','reg camera_packet=0,control_packet=0;')
insert='''    wire [7:0] follow_checksum=packet[63:56]^packet[55:48]^packet[47:40]^packet[39:32]^packet[31:24]^packet[23:16]^packet[15:8];
    wire follow_bounds=packet[47:40]<=2 && packet[39:32]<=16 && packet[31:24]>=1 && packet[31:24]<=8 && packet[23:8]<=300;
'''
s=s.replace('    wire expired=',insert+'    wire expired=')
s=s.replace('(!camera_packet && digits==18','(!camera_packet && !control_packet && digits==18')
s=s.replace('(camera_packet && digits==30 && camera_checksum==packet[7:0] && camera_bounds));','(camera_packet && digits==30 && camera_checksum==packet[7:0] && camera_bounds) ||\n         (control_packet && digits==16 && follow_checksum==packet[7:0] && follow_bounds));')
s=s.replace('gain<=7;limit<=5;','gain<=DEFAULT_GAIN;limit<=DEFAULT_LIMIT;')
s=s.replace('center_conflict<=0;','center_conflict<=0;parameter_applied<=0;')
s=s.replace('cam_sequence<=0;exposure<=3600;sensor_gain<=143;','cam_sequence<=0;exposure<=3600;sensor_gain<=143;\n            follow_sequence<=0;runtime_seconds<=30;operating_mode<=DEFAULT_MODE;damping<=6;slew<=2;control_packet<=0;')
s=s.replace('byte_value=="P" || byte_value=="V"','byte_value=="P" || byte_value=="V" || byte_value=="S"')
s=s.replace('camera_packet<=(byte_value=="V");','camera_packet<=(byte_value=="V");control_packet<=(byte_value=="S");')
s=s.replace('if(!camera_packet && digits==18','if(!camera_packet && !control_packet && digits==18')
s=s.replace('gain<=packet[23:16];limit<=packet[15:8];applied<=1;','gain<=packet[23:16];limit<=packet[15:8];applied<=1;parameter_applied<=1;')
s=s.replace("end else if(errors!=16'hffff)errors<=errors+1'b1;", """end else if(control_packet && digits==16 && follow_checksum==packet[7:0] && follow_bounds)begin
                        follow_sequence<=packet[63:48];operating_mode<=packet[47:40];damping<=packet[39:32];
                        slew<=packet[31:24];runtime_seconds<=packet[23:8];applied<=1;
                    end else if(errors!=16'hffff)errors<=errors+1'b1;""",1)
s=s.replace('digits<(camera_packet ? 30 : 18)','digits<(control_packet ? 16 : (camera_packet ? 30 : 18))')
p.write_text(s,encoding='utf-8')
p=root/'compose.py';s=p.read_text(encoding='utf-8').replace("'vision_move_core.v','vision_cdc_bridge.v'","'vision_move_core.v','vision_servo_core.v','vision_follow_control.v','vision_cdc_bridge.v'");p.write_text(s,encoding='utf-8')
p=root/'AGENTS.md';s=p.read_text(encoding='utf-8');s='''# Current continuous-control authorization (2026-10-22)

User explicitly requested completing remaining vision-loop development. This independent project preserves the large60 parent and original actuator. Add default continuous PD mode, manual single-move and explicit software-reference-return modes, K2 start/stop, K4 raw stop, K3 visual center, Chinese configuration/curves/logging, finite burst guards and safe retargeting at CHECK only. No physical stability or mechanical homing claim from simulation. No per-frame resetting of actuator pulse budgets/timeouts. Loss/config/disable/fault stops the session without automatic resume or homing. Confirm visual center before AUTO. Keep the official PH1P35 pins/clocks and pulse timing lineage. Preserve 64 pulse/5s active-burst and +/-64 command/feedback guards. Older restrictions below describe parent stages; new automatic motion is explicitly authorized. Direction observation is motor down -> ball right; absolute target sign cannot replace actual P delta. Default feedback orientation must be documented from logs/human confirmation and remain configurable. Only hardware K2 starts motion; S packet configures settings only.

'''+s;p.write_text(s,encoding='utf-8')
print('Created external/retargetable guarded core and backward-compatible S parameter RX')
