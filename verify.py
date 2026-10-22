from pathlib import Path
import hashlib,json,re,subprocess,sys,zipfile
from decode_serial import parse_record
sys.stdout.reconfigure(encoding='utf-8')
root=Path(__file__).resolve().parent
name=root.name;run=root/f'{name}_Runs/phy_1';out=root/'output'
def read(p):
    raw=p.read_bytes()
    text=raw.decode('utf-16') if raw.startswith((b'\xff\xfe',b'\xfe\xff')) else raw.decode('utf-8-sig')
    return text.replace('\r\n','\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def require(ok,message):
    if not ok:raise RuntimeError(message)
def execute(*args):
    result=subprocess.run(args,cwd=root,check=True,capture_output=True,text=True,encoding='utf-8',errors='replace')
    return result.stdout.strip()
execute(sys.executable,'record_contract.py');execute(sys.executable,'source_manifest.py','--check')
bit=out/f'{name}.bit'
require(b'# Package: PH1P35MDG324' in bit.read_bytes()[:500],'bit package mismatch')
require(sha(bit)==sha(run/f'{name}.bit'),'output bit is not current routed bit')
require('VISION_FOLLOW_20261022_BUILD_COMPLETE' in read(root/'build.log'),'TD completion marker missing')
require('ERROR:' not in read(root/'build.log'),'TD errors present')
require('HDL-7225 CRITICAL' not in read(root/'build.log'),'implicit re-declarations present')
baseline=root.parent/'d36a_autocal_latch_20261020'
large_lineage=json.loads(read(root/'large60_lineage.json'))
large_archive=Path(large_lineage['archive'])
for rel,digest in json.loads(read(large_archive/'manifest.json'))['files'].items():require(sha(large_archive/rel)==digest,'pre-large60 archive changed: '+rel)
with zipfile.ZipFile(large_lineage['archive_zip']) as z:require(z.testzip() is None,'pre-large60 ZIP CRC failed')
before=large_archive/'project'
expected_core=read(before/'rtl/vision_move_core.v').replace('TIMEOUT_CYCLES=100000000','TIMEOUT_CYCLES=250000000').replace('POSITION_LIMIT=16,FEEDBACK_LIMIT=22,MOVE_BUDGET=12,NO_RESPONSE_LIMIT=3,TARGET_LIMIT=5,DEADBAND=2','POSITION_LIMIT=64,FEEDBACK_LIMIT=64,MOVE_BUDGET=64,NO_RESPONSE_LIMIT=3,TARGET_LIMIT=60,DEADBAND=2')
require(read(root/'rtl/vision_move_core.v')==expected_core,'executor changed beyond authorized defaults')

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
expected_servo=expected_servo.replace('input wire clk,rst_n,enable_request,k4_n,k2_n,','input wire clk,rst_n,enable_request,k4_n,k2_n,\n input wire request,following,external_cancel,input wire [3:0] external_cancel_reason,')
expected_servo=expected_servo.replace('wire permit=visual_valid && !visual_stale && settings_ready && !config_changed;','wire extended_cancel=EXTERNAL_CONTROL && external_cancel;\n wire permit=visual_valid && !visual_stale && settings_ready && !config_changed && !extended_cancel;')
expected_servo=expected_servo.replace('reg previous_pressed=0,press_event=0;',"reg key_armed=0;\n reg previous_pressed=0,press_event=0;\n wire command_event=EXTERNAL_CONTROL ? request : press_event;\n wire command_armed=EXTERNAL_CONTROL ? 1'b1 : key_armed;")
expected_servo=expected_servo.replace('reg enabled=0,key_armed=0,active=0','reg enabled=0,active=0')
expected_servo=expected_servo.replace('wire in_band=wide_error<=DEADBAND && wide_error>=-DEADBAND;', 'wire [31:0] allowed_error=EXTERNAL_CONTROL && following ? FOLLOW_DEADBAND : DEADBAND;\n wire in_band=wide_error<=$signed(allowed_error) && wide_error>=-$signed(allowed_error);')
expected_servo=expected_servo.replace('if(press_event)begin','if(command_event)begin').replace('else if(!key_armed)reject_reason','else if(!command_armed)reject_reason').replace('if(press_event && key_armed &&','if(command_event && command_armed &&')
expected_servo=expected_servo.replace('cancel_reason<=config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3);','cancel_reason<=extended_cancel ? external_cancel_reason : (config_changed || !settings_ready ? 5 : (visual_stale ? 4 : 3));')
expected_servo=expected_servo.replace('CHECK:begin\n      if(in_band)', """CHECK:begin
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

def ps_function(source,name):return re.search(r'function '+re.escape(name)+r'\([^\n]*\)\{[\s\S]*?\n\}',source)[0]
old_host=read(before/'host/vision_panel_cn.ps1');new_host=read(root/'host/vision_panel_cn.ps1')
require(ps_function(new_host,'New-ParameterPacket')==ps_function(old_host,'New-ParameterPacket').replace('($Gain -band 127) -lt 4','($Gain -band 127) -lt 0').replace('$Limit -gt 5','$Limit -gt 60'),'P packet changed beyond bounds')
require(ps_function(new_host,'New-CameraPacket')==ps_function(old_host,'New-CameraPacket'),'V packet changed')
for rel in ['enable_latch.v','encoder_ab_counter.v']:
    require(sha(root/'rtl'/rel)==sha(baseline/'rtl'/rel),'motor source changed: '+rel)
center_lineage=json.loads(read(root/'center_lineage.json'))
center_parent=Path(center_lineage['parent_project'])
for rel,digest in center_lineage['parent_files'].items():require(sha(center_parent/rel)==digest,'K2 parent changed: '+rel)
for rel in ['enable_latch.v','encoder_ab_counter.v','vision_shadow_target.v','vision_camera_control.v']:
    require(sha(root/'rtl'/rel)==sha(center_parent/'rtl'/rel),'retained source changed: '+rel)
center_archive=Path(center_lineage['archive'])
for rel,digest in json.loads(read(center_archive/'manifest.json'))['files'].items():require(sha(center_archive/rel)==digest,'K2 archive changed: '+rel)
with zipfile.ZipFile(center_lineage['archive_zip']) as z:require(z.testzip() is None,'K2 archive ZIP CRC failed')
motion_lineage=json.loads(read(root/'motion_lineage.json'))
parent=Path(motion_lineage['parent_project'])
for rel,digest in motion_lineage['parent_files'].items():require(sha(parent/rel)==digest,'preserved visual parent changed: '+rel)
require(sha(parent/'output/vision_control_20261021_camera_roi.bit')==motion_lineage['parent_bit_sha256'],'saved visual bit changed')
require(sha(root/'rtl/reference_position_core.v')==motion_lineage['original_position_core_sha256']==sha(root.parent/'d36a_position5_20261020/rtl/position_core.v'),'passed executor reference changed')
visual_archive=Path(motion_lineage['archive'])
for rel,digest in json.loads(read(visual_archive/'manifest.json'))['files'].items():require(sha(visual_archive/rel)==digest,'visual archive changed: '+rel)
with zipfile.ZipFile(motion_lineage['archive_zip']) as z:require(z.testzip() is None,'visual archive ZIP CRC failed')
archive=Path('D:/AAAProject/FPGA/AAA_LookHere/成功版本归档/20261020_K1点按保持_实板通过')
archive_manifest=json.loads(read(archive/'manifest.json'))
for rel,digest in archive_manifest['files'].items():require(sha(archive/rel)==digest,'archive changed: '+rel)
with zipfile.ZipFile(archive.with_suffix('.zip')) as z:require(z.testzip() is None,'golden ZIP CRC failed')
lineage=json.loads(read(root/'camera_lineage.json'));reference=Path(lineage['reference'])
for rel,digest in lineage['unchanged_sources'].items():
    require(sha(root/rel)==digest==sha(reference/rel),'camera/IP source changed: '+rel)
parts=['enable_latch.v','encoder_ab_counter.v','vision_move_core.v','vision_servo_core.v','vision_follow_control.v','vision_cdc_bridge.v','vision_config_rx.v','vision_center_key.v',
       'vision_shadow_target.v','vision_report_tx.v','vision_monitor.v','vision_value_cdc.v','vision_camera_control.v']
composed='\n'.join('// Source: '+p+'\n'+read(root/'rtl'/p) for p in parts)
require(read(root/'rtl/vision_monitor_all.v')==composed,'composition stale')
for bench,marker in [('tb_full','ALL_TESTS_PASS'),('tb_detector','DETECTOR_INTEGRATION_PASS'),('tb_camera','CAMERA_CONTROL_PASS'),('tb_motion','MOTION_GUARDS_PASS'),('tb_center','CENTER_CAPTURE_PASS'),('tb_large','LARGE_TARGET_PASS'),('tb_follow','FOLLOW_ALL_PASS'),('tb_plant','PLANT_ALL_PASS')]:
    log=read(root/f'sim/results/{bench}.log')
    require(marker in log and 'FATAL' not in log and 'FAIL:' not in log,'simulation failed: '+bench)
    require(composed in read(root/f'sim/results/{bench}.combined.v'),'simulation source stale: '+bench)
motion_log=read(root/'sim/results/tb_motion.log')
require('PASS finite motion:' in motion_log,'finite motion test missing')
for code in range(1,9):require(f'PASS fault {code}:' in motion_log,'fault fixture missing: '+str(code))
large_log=read(root/'sim/results/tb_large.log')
require('LARGE_NORMAL_PASS' in large_log and 'LARGE_BUDGET_PASS' in large_log,'large-target timing/budget regression absent')
large_moves=[dict(zip(['target','position','error','pulses','scaled_cycles'],map(int,m))) for m in re.findall(r'LARGE_REACHED target=(-?\d+) P=(-?\d+) error=(-?\d+) pulses=(\d+) elapsed_scaled=(\d+)',large_log)]
require([row['target'] for row in large_moves[:4]]==[60,0,-60,0],'large-target sequence absent')
require(all(abs(row['error'])<=2 and row['pulses']<=64 and 200000<row['scaled_cycles']<500000 for row in large_moves),'large-target arrival or5s timing failed')
pixel_fixture=json.loads(read(root/'sim/detector_fixture_lineage.json'))
require(sha(Path(pixel_fixture['reference']))==pixel_fixture['sha256'],'reference pixel fixture changed')
simlog=read(root/'sim/results/tb_full.log')
wire_lines=[line.split(' ',1)[1] for line in simlog.splitlines() if line.startswith(('UART ','UART_FIXTURE '))]
rows=[parse_record(line) for line in wire_lines]
require(len(rows)>=4 and all(rows) and sum(line.startswith('UART_FIXTURE ') for line in simlog.splitlines())==2,'wire UART decode failed')
require(all(len(line)+2==429 for line in wire_lines),'wire UART frame length mismatch')
monitor_rows=[parse_record(line.split(' ',1)[1]) for line in simlog.splitlines() if line.startswith('UART ')]
require(monitor_rows[-1]['SEQ']==4 and monitor_rows[-1]['C']==960 and monitor_rows[-1]['G']==7,'real TX config acknowledgement failed')
require(rows[0]['E1']==-4 and rows[0]['T']==-5 and rows[0]['P']==4,'signed wire data incorrect')
require(rows[0]['MT']==-5 and rows[0]['ME']==-9 and rows[0]['MC']==-4,'signed motion UART fields incorrect')
require(rows[0]['CC']==0xabcd and rows[0]['KC']==1,'center fields missing from coherent serializer snapshot')
require(monitor_rows[-1]['DC']>=8 and monitor_rows[-1]['AC']>=monitor_rows[-1]['DC'] and monitor_rows[-1]['MB']==0 and monitor_rows[-1]['EN']==0 and monitor_rows[-1]['CR']==2,'real feedback movement/stop evidence missing')
require('LARGE_INTEGRATION_PASS' in simlog and any(row['T']==60 and row['MT']==60 and row['G']==0 and row['L']==60 for row in monitor_rows) and any(row['T']==-60 and row['MT']==-60 for row in monitor_rows),'real UART large target evidence missing')
require(all(row['F']==0 for row in monitor_rows),'unexpected fault in integrated normal/cancellation flow')
require(all(r['T']==0 for r in rows if not r['V']),'invalid target not zero')
(out/'simulated_uart_frames.txt').write_text('\n'.join(wire_lines)+'\n',encoding='ascii')
camera_lines=[line.split(' ',1)[1] for line in read(root/'sim/results/tb_camera.log').splitlines() if line.startswith('UART_CAMERA ')]
require(len(camera_lines)==1,'camera-flow UART fixture absent')
camera_row=parse_record(camera_lines[0]);require(camera_row is not None,'camera-flow UART decode failed')
require(len(camera_lines[0])+2==429,'camera-flow UART length wrong')
for key,value in {'CS':4,'CA':4,'RA':4,'CB':0,'AE':3600,'AG':143,'RL':20,'RT':30,'RR':1000,'RB':600,'EN':1,'F':0,'MB':0,'MC':0}.items():
    require(camera_row[key]==value,'camera-flow UART mismatch: '+key)
center_lines=[line.split(' ',1)[1] for line in read(root/'sim/results/tb_center.log').splitlines() if line.startswith('UART_CENTER ')]
require(len(center_lines)==1 and len(center_lines[0])+2==429,'center wire fixture absent/wrong length')
center_row=parse_record(center_lines[0]);require(center_row is not None,'center wire decode failed')
for key,value in {'C':500,'BX':500,'T':0,'CC':7,'KC':1,'F':0,'MB':0,'AC':2,'DC':2,'P':2}.items():
    require(center_row[key]==value,'center wire mismatch: '+key)
(out/'simulated_uart_frames.txt').write_text('\n'.join(wire_lines+camera_lines+center_lines)+'\n',encoding='ascii')

require(rows[0]['VE']==-4 and rows[0]['AT']==-5 and rows[0]['RU']==30 and rows[0]['RT']==200,'new signed/unique report fields failed')
follow_log=read(root/'sim/results/tb_follow.log')
require('FOLLOW_CONTROL_PASS' in follow_log and 'PD_ARITHMETIC_PASS' in follow_log and 'RETARGET_GUARD_PASS fault=4' in follow_log and 'RETARGET_GUARD_PASS fault=5' in follow_log,'follow safety/arithmetic paths not covered')
follow_lines=[line.split(' ',1)[1] for line in follow_log.splitlines() if line.startswith('UART_FOLLOW ')]
require(len(follow_lines)==1 and len(follow_lines[0])+2==429,'follow wire frame absent')
follow_row=parse_record(follow_lines[0]);require(follow_row is not None,'follow wire decode failed')
from analyze_follow import analyze
evaluation=analyze([follow_lines[0],follow_lines[0].replace(' BX=03C0 ',' BX=03D4 '),follow_lines[0].replace(' AR=1 ',' AR=0 ').replace(' SR=0 ',' SR=7 ')])
require(len(evaluation['automatic_sessions'])==1 and evaluation['automatic_sessions'][0]['stop_reason']==7 and evaluation['automatic_sessions'][0]['whole']['error_pixels']['rms']==14.142,'capture analysis/session metrics failed')
for key,value in {'MD':1,'AR':1,'RU':0,'DS':3,'CZ':1,'F':0,'AT':0,'RT':0,'SR':0}.items():
    require(follow_row[key]==value,'follow wire mismatch: '+key)
plant_log=read(root/'sim/results/tb_plant.log')
plant_cases=[{'id':int(i),'coupling':float(c),'peak_pixels':float(p),'final_error_pixels':float(e),'final_velocity_pixels_per_frame':float(v),'rms_tail_pixels':float(r),'P':int(pos)} for i,c,p,e,v,r,pos in re.findall(r'PLANT_MODEL_PASS id=(\d+) coupling=([\d.]+) peak=([\d.]+) final_error=([-\d.]+) final_velocity=([-\d.]+) rms_tail=([\d.]+) P=(-?\d+)',plant_log)]
require(len(plant_cases)==3 and all(abs(c['final_error_pixels'])<=80 and abs(c['final_velocity_pixels_per_frame'])<=1 for c in plant_cases),'assumed plant acceptance failed')
(out/'simulated_uart_frames.txt').write_text('\n'.join(wire_lines+camera_lines+center_lines+follow_lines)+'\n',encoding='ascii')
top=read(root/'user_source/hdl_source/design_top_wrapper.v')
require(sha(root/'user_source/hdl_source/design_top_wrapper.v')==sha(parent/'user_source/hdl_source/design_top_wrapper.v'),'parent camera top changed')
require('ae_set u_ae_set' not in top and 'vision_camera_control u_camera_control' in top,'camera button/control conflict')
require(top.index('wire S_detect_rst_n')<top.index('vision_camera_control u_camera_control'),'reset declared after use')
require(top.index('wire [10:0] S_roi_left')<top.index('vision_camera_control u_camera_control'),'ROI declared after use')
require('.src_frame(S_detect_done)' in top,'CDC does not use completed detection results')
require('u_steel_ball_motor_manual' not in top and 'u_steel_ball_motion' not in top,'legacy motor/motion instantiated')
al=read(root/'td_project'/f'{name}.al')
for forbidden in ['steel_ball_motor_manual.v','steel_ball_motion.v','steel_ball_uart_diag.v','ae_set.v','steel_ball_roi_uart.v']:
    require(forbidden not in al,'legacy source remains in project: '+forbidden)
for path in re.findall(r'<File Path="([^"]+)"',al):require((root/'td_project'/path).exists(),'missing project source '+path)
def adc(path):
    result={}
    for line in read(path).splitlines():
        if line.lstrip().startswith('#'):continue
        m=re.search(r'set_pin_assignment\s*\{\s*(.*?)\s*\}\s*\{(.*?)\}',line)
        if m:result[m[1]]={k.strip():v.strip() for k,v in re.findall(r'(\w+)\s*=\s*([^;]+);',m[2])}
    return result
official=root.parents[1]/'2_Example/32_cs500_mipi_hdmi/pin/pin.adc'
official_pins=adc(official);constraints=adc(root/'user_source/constraints_source/pin.adc')
aliases={'I_sys_clk_25m':'I_sys_clk','O_clk_24m':'O_cam_24m'}
for port,fields in official_pins.items():
    dest=aliases.get(port,port)
    require(dest in constraints,'missing official camera port: '+dest)
    for key in ['LOCATION','IOSTANDARD']:require(constraints[dest][key]==fields[key],'official pin mismatch: '+dest)
def pins(path):
    result={}
    for line in read(path).splitlines():
        f=line.split()
        if len(f)>=8 and f[1] in ('INPUT','OUTPUT','INOUT'):
            result[f[0]]={'pin':f[2],'direction':f[1],'standard':f[3],'pull':f[6]}
    return result
area_path=run/f'{name}_phy.area';area=read(area_path);routed=pins(area_path)
reference_pins=pins(reference/'td_project/camera_to_dsi_display_Runs/phy_1/camera_to_dsi_display_phy.area')
for port,fields in reference_pins.items():
    require(port in routed,'reference IO lost: '+port)
    for key in ['pin','standard','direction']:require(routed[port][key]==fields[key],'reference IO changed: '+port)
for port,fields in constraints.items():
    require(port in routed and routed[port]['pin']==fields['LOCATION'] and routed[port]['standard']==fields['IOSTANDARD'],'routed constraint mismatch: '+port)
    require(routed[port]['pull']==fields['PULLTYPE'],'routed pull mismatch: '+port)
require(len({r['pin'] for r in routed.values()})==len(routed),'duplicate IO location')
golden_by_pin={fields['LOCATION']:fields for fields in adc(baseline/f'{baseline.name}.adc').values()}
for port in ['O_motor_step','O_motor_step_v2','O_motor_dir','O_motor_dir_v1','O_motor_en','O_motor_en_u2',
             'I_encoder_j1[1]','I_encoder_j1[0]','I_encoder_j2[1]','I_encoder_j2[0]']:
    old=golden_by_pin[routed[port]['pin']]
    require(routed[port]['pull']==old['PULLTYPE'] and routed[port]['standard']==old['IOSTANDARD'],'golden electrical settings changed: '+port)
netlist=read(root/'routed_netlist.v');drivers={}
for port in ['O_motor_step','O_motor_step_v2','O_motor_dir','O_motor_dir_v1','O_motor_en','O_motor_en_u2']:
    match=re.search(r'\b'+port+r'_syn_\d+\s*\(\s*\.doq\(\{([^}]+)\}\),\s*\.opad\('+port+r'\)\);',netlist)
    require(match is not None,'output pad driver not found: '+port)
    drivers[port]=match[1].split(',')[-1].strip()
for primary,mirror in [('O_motor_step','O_motor_step_v2'),('O_motor_dir','O_motor_dir_v1'),('O_motor_en','O_motor_en_u2')]:
    require(drivers[primary]==drivers[mirror] and drivers[primary] not in ("1'b0","1'b1"),'active motor mirror driver mismatch: '+primary)
timing=read(run/f'{name}_pr.timing')
times={key:float(re.search(r'\b'+key+r':\s*([-\d.]+)ns',timing)[1]) for key in ['SWNS','STNS','HWNS','HTNS']}
require(times['SWNS']>=0 and times['HWNS']>=0 and times['STNS']==times['HTNS']==0,'timing failed')
require('Confidence          : Routed' in timing and 'Device              : PH1P35MDG324' in timing,'timing/device wrong')
clock_notices=[]
for line in read(root/'build.log').splitlines():
    if 'CRITICAL-WARNING' not in line:continue
    known=re.match(r'^PHY-5060 CRITICAL-WARNING: Clock net candidate (u_uicfgcs500/uii2c_inst/scl_clk|O_cam_24m_syn_\d+) has mixed fanouts',line) or re.match(r'^PHY-5079 CRITICAL-WARNING: Marked \d+ clock net\(s\) utilize the local routing resources',line)
    require(known,'unreviewed critical diagnostic: '+line);clock_notices.append(line)
sdc=read(root/'td_project'/f'{name}.sdc');ref_sdc=read(reference/'td_project/camera_to_dsi_display.sdc')
clock_commands=lambda text:[line for line in text.splitlines() if line.startswith(('create_clock','rename_clock','derive_clocks'))]
require(clock_commands(sdc)==clock_commands(ref_sdc),'reference clocks/periods changed')
clock_groups=lambda text:re.findall(r'set_clock_groups[\s\S]*?(?=\n\s*\n)',text)
require(clock_groups(sdc)==clock_groups(ref_sdc),'reference asynchronous clock groups changed')
require('set_false_path -from [get_clocks' not in sdc,'legacy blanket clock false path retained')
require(sdc.count('set_max_delay -datapath_only')==2 and sdc.count('10.000')>=2,'bounded camera mailboxes not constrained')
require('CAMERA_CDC_BOUNDED_CLOCK_PAIRS 10.000ns' in read(root/'build.log'),'bounded camera CDC constraints absent')
for start,end in re.findall(r'Begin Point\s*:\s*(.*?)\nEnd Point\s*:\s*(.*?)\n',timing):
    if ('sys_clk_50m' in start and 'pll_clk_24m' in end) or ('pll_clk_24m' in start and 'sys_clk_50m' in end):
        require('u_camera_control/' in start and 'u_camera_control/' in end,'non-camera crossing entered bounded mailbox clock pair')
host=execute('cmd.exe','/c','host\\start_panel.cmd','-SelfTest')
require('HOST_SELF_TEST_PASS' in host,'host protocol self test failed')
smoke=execute('cmd.exe','/c','host\\start_panel.cmd','-SmokeTest')
require('HOST_UI_SMOKE_PASS' in smoke,'host UI did not render')
resources={key:int(re.search(r'^#'+label+r'\s+(\d+)',area,re.M)[1]) for key,label in [('slice','slice'),('register','reg'),('IO','IO'),('ERAM','eram'),('DSP','dsp')]}
require(resources['slice']>15000 and resources['ERAM']>=45 and resources['DSP']>=17,'camera/ISP logic unexpectedly trimmed')
for instance in ['u_isp_top','u_steel_ball_stream_bbox','u_steel_ball_detect','u_camera_control','motor_core']:
    require(instance in area,'required active hierarchy missing: '+instance)
warnings=[line for line in read(root/'build.log').splitlines() if 'WARNING:' in line and ('Undriven' in line or 'undriven' in line)]
require(not any('S_uart_' in line or 'u_vision_monitor' in line or 'u_camera_control' in line for line in warnings),'new integration undriven net')
report={'status':'SOFTWARE_BUILD_PASS_AWAITING_PHYSICAL_CONTINUOUS_CONTROL_TEST','date':'2026-10-22',
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
 'resources':resources,'timing_ns':times,'STA_coverage_percent':float(re.search(r'STA coverage\s*:\s*([\d.]+)%',timing)[1]),
 'timing_limitations':'original clocks/groups retained; held-data camera mailboxes bounded10ns; positive routed timing is not board validation',
 'routed_pins':routed,'routed_motor_drivers':drivers,'official_pin_reference':str(official),
 'host_tests':[host,smoke],'remaining_camera_IP_undriven_warnings':warnings,'reference_clock_routing_notices':clock_notices,
 'camera_confirmation':'configuration-flow completion, not electrical register/ACK readback',
 'pending_physical':['correction direction check','actual plant tuning and stability','mechanical platform level/calibration','contest performance','power-cycle/Flash persistence']}
(root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:report[k] for k in ['status','sha256','resources','timing_ns','STA_coverage_percent']},indent=2))
