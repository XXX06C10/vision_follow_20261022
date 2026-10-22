"""One-time verifier update for the independently archived follow project."""
from pathlib import Path
p=Path(__file__).resolve().parent/'verify.py';s=p.read_text()
s=s.replace('VISION_K3_CENTER_20261022_BUILD_COMPLETE','VISION_FOLLOW_20261022_BUILD_COMPLETE')
a=s.index("require(read(root/'rtl/vision_monitor.v')==")
b=s.index('\ndef ps_function',a)
s=s[:a]+'''
follow_lineage=json.loads(read(root/'follow_lineage.json'));follow_parent=Path(follow_lineage['parent_project'])
for rel,digest in follow_lineage['parent_files'].items():require(sha(follow_parent/rel)==digest,'large60 parent changed: '+rel)
follow_archive=Path(follow_lineage['archive'])
for rel,digest in json.loads(read(follow_archive/'manifest.json'))['files'].items():require(sha(follow_archive/rel)==digest,'large60 archive changed: '+rel)
with zipfile.ZipFile(follow_lineage['archive_zip']) as z:require(z.testzip() is None,'large60 ZIP CRC failed')
require(sha(root/'rtl/vision_move_core.v')==sha(follow_parent/'rtl/vision_move_core.v'),'frozen finite executor changed')
# Reconstruct the exact, reviewable adaptation from the unchanged finite executor.
expected_servo=read(follow_parent/'rtl/vision_move_core.v')
expected_servo=expected_servo.replace('// New commands are K2 edges sampling a visual target, never a continuous setpoint.','// External requests share the finite guard sequence; follow retargets only at CHECK.')
expected_servo=expected_servo.replace('module vision_move_core','module vision_servo_core').replace('TARGET_LIMIT=60,DEADBAND=2','TARGET_LIMIT=60,DEADBAND=2,EXTERNAL_CONTROL=0,FOLLOW_DEADBAND=1')
expected_servo=expected_servo.replace('input wire clk,rst_n,enable_request,k4_n,k2_n,','input wire clk,rst_n,enable_request,k4_n,k2_n,\\n input wire request,following,external_cancel,input wire [3:0] external_cancel_reason,')
expected_servo=expected_servo.replace('wire permit=visual_valid && !visual_stale && settings_ready && !config_changed;','wire extended_cancel=EXTERNAL_CONTROL && external_cancel;\\n wire permit=visual_valid && !visual_stale && settings_ready && !config_changed && !extended_cancel;')
expected_servo=expected_servo.replace('reg previous_pressed=0,press_event=0;',"reg key_armed=0;\\n reg previous_pressed=0,press_event=0;\\n wire command_event=EXTERNAL_CONTROL ? request : press_event;\\n wire command_armed=EXTERNAL_CONTROL ? 1'b1 : key_armed;")
expected_servo=expected_servo.replace('reg enabled=0,key_armed=0,active=0','reg enabled=0,active=0')
expected_servo=expected_servo.replace('wire in_band=wide_error<=DEADBAND && wide_error>=-DEADBAND;', 'wire [31:0] allowed_error=EXTERNAL_CONTROL && following ? FOLLOW_DEADBAND : DEADBAND;\\n wire in_band=wide_error<=$signed(allowed_error) && wide_error>=-$signed(allowed_error);')
expected_servo=expected_servo.replace('if(press_event)begin','if(command_event)begin').replace('else if(!key_armed)reject_reason','else if(!command_armed)reject_reason').replace('if(press_event && key_armed &&','if(command_event && command_armed &&')
expected_servo=expected_servo.replace('cancel_reason<=config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3);','cancel_reason<=extended_cancel ? external_cancel_reason : (config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3));')
expected_servo=expected_servo.replace('CHECK:begin\\n      if(in_band)', """CHECK:begin
      if(EXTERNAL_CONTROL && following && (visual_target>TARGET_LIMIT || visual_target< -TARGET_LIMIT))begin
       fault<=8;step_r<=0;active<=0;done<=0;state<=FAULT;cancel_reason<=6;
      end else if(EXTERNAL_CONTROL && following && locked_target!={{24{visual_target[7]}},visual_target})begin
       // Retarget only after the previous pulse/feedback phase. Never reset budget/time.
       locked_target<={{24{visual_target[7]}},visual_target};
      end else if(in_band)""")
require(read(root/'rtl/vision_servo_core.v')==expected_servo,'servo changed beyond documented request/cancel/CHECK retarget/auto deadband adaptation')
for rel in ['rtl/vision_shadow_target.v','rtl/vision_center_key.v','user_source/constraints_source/pin.adc']:
    require(sha(root/rel)==sha(follow_parent/rel),'retained source/constraints changed: '+rel)
for ext in ['al','sdc']:
    require(read(root/'td_project'/f'{name}.{ext}')==read(follow_parent/'td_project'/f'{follow_parent.name}.{ext}').replace(follow_parent.name,name),'project changed beyond isolated project name: '+ext)
''' +s[b:]
s=s.replace("'vision_move_core.v','vision_cdc_bridge.v'", "'vision_move_core.v','vision_servo_core.v','vision_follow_control.v','vision_cdc_bridge.v'")
s=s.replace("('tb_large','LARGE_TARGET_PASS')", "('tb_large','LARGE_TARGET_PASS'),('tb_follow','FOLLOW_ALL_PASS'),('tb_plant','PLANT_ALL_PASS')")
s=s.replace('==342','==429')
at=s.index("top=read(root/'user_source")
s=s[:at]+'''
require(rows[0]['VE']==-4 and rows[0]['AT']==-5 and rows[0]['RU']==30 and rows[0]['RT']==200,'new signed/unique report fields failed')
follow_log=read(root/'sim/results/tb_follow.log')
require('FOLLOW_CONTROL_PASS' in follow_log and 'RETARGET_GUARD_PASS fault=4' in follow_log and 'RETARGET_GUARD_PASS fault=5' in follow_log,'follow safety paths not covered')
follow_lines=[line.split(' ',1)[1] for line in follow_log.splitlines() if line.startswith('UART_FOLLOW ')]
require(len(follow_lines)==1 and len(follow_lines[0])+2==429,'follow wire frame absent')
follow_row=parse_record(follow_lines[0]);require(follow_row is not None,'follow wire decode failed')
for key,value in {'MD':1,'AR':1,'RU':0,'DS':3,'CZ':1,'F':0,'AT':0,'RT':0,'SR':0}.items():
    require(follow_row[key]==value,'follow wire mismatch: '+key)
plant_log=read(root/'sim/results/tb_plant.log')
plant_cases=[{'id':int(i),'coupling':float(c),'peak_pixels':float(p),'final_error_pixels':float(e),'final_velocity_pixels_per_frame':float(v),'rms_tail_pixels':float(r),'P':int(pos)} for i,c,p,e,v,r,pos in re.findall(r'PLANT_MODEL_PASS id=(\\d+) coupling=([\\d.]+) peak=([\\d.]+) final_error=([-\\d.]+) final_velocity=([-\\d.]+) rms_tail=([\\d.]+) P=(-?\\d+)',plant_log)]
require(len(plant_cases)==3 and all(abs(c['final_error_pixels'])<=80 and abs(c['final_velocity_pixels_per_frame'])<=1 for c in plant_cases),'assumed plant acceptance failed')
(out/'simulated_uart_frames.txt').write_text('\\n'.join(wire_lines+camera_lines+center_lines+follow_lines)+'\\n',encoding='ascii')
''' +s[at:]
s=s.replace("'powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-STA','-File','host/vision_panel_cn.ps1'", "'cmd.exe','/c','host\\\\start_panel.cmd'")
a=s.index("report={'status':")
s=s[:a]+'''report={'status':'SOFTWARE_BUILD_PASS_AWAITING_PHYSICAL_CONTINUOUS_CONTROL_TEST','date':'2026-10-22',
 'device':'PH1P35MDG324','bit':str(bit),'delivery_bit':str(bit),'sha256':sha(bit),'physical_test':'PENDING_USER_BOARD_TEST',
 'automatic_motion':{'continuous_PD':True,'manual_K2_snapshot':True,'explicit_software_reference_return':True,'K2_required_to_start':True,'automatic_resume':False,'automatic_homing':False},
 'direction':{'human_confirmed':'motor down -> ball right','P_delta_human':'not recalled; user authorized log default','default':'normal, positive P reference tilt assumed to move ball left, from log trend; configurable P gain bit7','physical_dynamic_calibration':False},
 'parent_large60_bit_sha256':follow_lineage['parent_bit_sha256'],'parent_large60_archive':str(follow_archive),
 'parent_observations':'large60 logs show finite moves, not measured ball-loop stability or complete mechanical calibration',
 'flash_cold_boot':'not tested; SRAM bit delivery; no hardware programming performed',
 'motion_bounds':{'target_counts':[-60,60],'manual_deadband_counts':2,'follow_deadband_counts':1,'active_burst_pulse_budget':64,'active_burst_timeout_seconds':5,'no_response_pulses':3,'wrong_direction_observations':2,'net_command_limit_pulses':64,'feedback_limit_counts':64,'stale_ms':200,'CHECK_retarget_does_not_reset_budget_or_timer':True},
 'control_parameters':{'gain_range':[0,12],'limit_range':[0,60],'damping_range':[0,16],'slew_range':[1,8],'runtime_seconds_range':[0,300],'defaults':{'mode':1,'gain':4,'limit':12,'damping':12,'slew':2,'runtime_seconds':30},'control_tick_ms':100,'velocity_format':'signed Q4 pixels/frame; filtered (3*VE + delta*16)/4; delta clipped +/-256','P_V_format_checksum_unchanged':True,'S_config_only':True},
 'K3_center':{'requires_idle_session':True,'center_confirmed_required_for_auto':True,'K1_required':False,'motor_motion':False,'resets_encoder_reference':False,'same_cycle_UART_commit_priority':True,'nonvolatile':False},
 'software_reference':'origin captured once after first K1 enable settles; not a mechanical home or measured platform level',
 'simulations':['retained finite executor with eight latched fault guards','large +60/0/-60/0 at1.28 counts/pulse and production timing; direct cross-end budget fault','K3 capture/reject/collision and coherent wire echo','real camera RX and I2C completion/ROI CDC','full720p detection integration','full automatic session, safe CHECK retarget, K2 held/toggle/stop, manual/return, loss/stale/settings/K1/K4/fault, runtime and no resume','moving targets cannot reset timeout or64pulse budget','three explicitly assumed rolling-ball models with disturbance; no measured physical calibration'],
 'large_target_regression':large_moves,'assumed_plant_cases':plant_cases,
 'plant_limits':'final abs(error)<=80px and abs(velocity)<=1px/frame at28s, with residual oscillation/quantization; not a physical accuracy claim',
 'report_version':5,'wire_UART_bytes':429,'parameter_packet_bytes':20,'camera_packet_bytes':32,'follow_packet_bytes':18,
 'source_manifest':str(root/'build_sources.json'),'baseline_files_frozen':37,'camera_IP_files_byte_identical':len(lineage['unchanged_sources']),
 'executor_lineage':'finite executor byte-identical to archived large60; exact documented external interface/CHECK retarget/auto DB1 adaptation checked; reference '+motion_lineage['original_position_core_sha256'],
 'camera_defaults':{'exposure':3600,'sensor_gain':143},'ROI':'1920x1080 host ->1280x720 sensor, existing frame-boundary commit',
 'resources':resources,'timing_ns':times,'STA_coverage_percent':float(re.search(r'STA coverage\\s*:\\s*([\\d.]+)%',timing)[1]),
 'timing_limitations':'original clocks/groups retained; held-data camera mailboxes bounded10ns; positive routed timing is not board validation',
 'routed_pins':routed,'routed_motor_drivers':drivers,'official_pin_reference':str(official),
 'host_tests':[host,smoke],'remaining_camera_IP_undriven_warnings':warnings,'reference_clock_routing_notices':clock_notices,
 'camera_confirmation':'configuration-flow completion, not electrical register/ACK readback',
 'pending_physical':['correction direction check','actual plant tuning and stability','mechanical platform level/calibration','contest performance','power-cycle/Flash persistence']}
(root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:report[k] for k in ['status','sha256','resources','timing_ns','STA_coverage_percent']},indent=2))
'''
p.write_text(s)
print('Updated follow verifier')
