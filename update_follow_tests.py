from pathlib import Path
root=Path(__file__).resolve().parent
tail=' MD=1 AR=1 KD=06 SL=02 RT=001E DS=1234 CZ=1 VE=FFFC AT=FB SR=7 AJ=A EL=0003 TS=12345678'
for name in ('tb_full','tb_center','tb_camera'):
 p=root/'sim'/(name+'.v');s=p.read_text(encoding='utf-8')
 s=s.replace('vision_monitor #(', 'vision_monitor #(.DEFAULT_MODE(0),.DEFAULT_GAIN(7),.DEFAULT_LIMIT(5),')
 s=s.replace('VIS=4','VIS=5').replace('342','429')
 if name=='tb_full':
  s=s.replace('KC=1\\015','KC=1'+tail+'\\015')
  s=s.replace(".center_count(16'habcd),", ".operating_mode(8'd1),.auto_run(1'b1),.damping(8'd6),.slew(8'd2),.runtime_seconds(16'd30),.follow_sequence(16'h1234),.center_confirmed(1'b1),.velocity(-16'sd4),.drive_target(-8'sd5),.stop_reason(4'd7),.auto_reject(4'd10),.elapsed_seconds(16'd3),.total_pulses(32'h12345678),.center_count(16'habcd),")
 p.write_text(s,encoding='utf-8')
# Keep the passed original module in rtl unchanged; test the new guarded core as well.
for name in ('tb_motion','tb_large'):
 p=root/'sim'/(name+'.v');s=p.read_text(encoding='utf-8').replace('vision_move_core #(','vision_servo_core #(');p.write_text(s,encoding='utf-8')
p=root/'run_sim.ps1';s=p.read_text(encoding='utf-8').replace("'tb_center','tb_large'","'tb_center','tb_large','tb_follow','tb_plant'");p.write_text(s,encoding='utf-8')
print('Updated legacy/manual and wire fixtures to VIS5')
