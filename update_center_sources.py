"""One-time migration of copied VIS=3 project; use normal build scripts afterwards."""
from pathlib import Path
root=Path(__file__).resolve().parent
def edit(name,changes):
 p=root/name;s=p.read_text(encoding='utf-8-sig')
 for old,new in changes:
  assert old in s,(name,old[:70]);s=s.replace(old,new)
 p.write_text(s,encoding='utf-8-sig' if name.endswith('vision_panel_cn.ps1') else 'utf-8')
assert "('CC'," not in (root/'generate_uart.py').read_text(encoding='utf-8'),'Already migrated'
edit('compose.py',[("'vision_config_rx.v','vision_shadow_target.v'","'vision_config_rx.v','vision_center_key.v','vision_shadow_target.v'")])
edit('generate_uart.py',[("('DC',4,'completed_count','hex')]","('DC',4,'completed_count','hex'),('CC',4,'center_count','hex'),('KC',1,'center_status','hex')]"),
 ("8'h33","8'h34"),('input wire [15:0] accepted_count,completed_count,','input wire [15:0] accepted_count,completed_count,center_count,\n    input wire [3:0] center_status,')])
edit('decode_serial.py',[('VIS=3','VIS=4'),("'3' if kind=='literal'","'4' if kind=='literal'")])
edit('run_sim.ps1',[("'tb_camera','tb_motion'","'tb_camera','tb_motion','tb_center'")])
edit('sim/tb_full.v',[
 ('click(2);tick(100);check(pulses==0,"K3 has no motion function");','tick(100);check(pulses==0,"no enable-only movement");'),
 ('VIS=3','VIS=4'),('329','342'),('DC=0001\\015','DC=0001 CC=ABCD KC=1\\015'),
 ('.quality(4\'hf)',".center_count(16'habcd),.center_status(4'd1),.quality(4'hf)")])
edit('sim/tb_camera.v',[('329','342')])
edit('host/vision_panel_cn.ps1',[
 ("'AC:4','DC:4')","'AC:4','DC:4','CC:4','KC:1')"),('VIS=3','VIS=4'),('VIS3','VIS4'),
 ('AC=0002 DC=0001\'','AC=0002 DC=0001 CC=0001 KC=1\''),
 ('K1使能·K2执行·K4停止','K1使能·K2执行·K3定中心·K4停止'),
 ('K1保持使能，K2执行一次，K4停止。','K1使能，K2执行，K3记录球的中心，K4停止。'),
 ("Drawing.Size(1180,900)","Drawing.Size(1180,932)"),("Drawing.Size(1196,939)","Drawing.Size(1196,971)"),
 ("$details=Label-At '等待新版 VIS=4 串口数据……' 360 709 800 105", "$centerAck=Label-At '控制中心：默认960。把球放到实际中心，按板上K3确认；重新下载/断电后需重设。' 360 707 800 30\n$details=Label-At '等待新版 VIS=4 串口数据……' 360 741 800 105"),
 ('SetBounds(360,820,800,35)','SetBounds(360,852,800,35)'),(" 20 870 1140"," 20 902 1140"),
 ("$script:lastRecord=$R;$script:lastRx=[DateTime]::Now", """$script:lastRecord=$R;$script:lastRx=[DateTime]::Now
 if($null -eq $script:lastCenterCount -or $script:lastCenterCount -ne $R.CC){
  $center.Value=$R.C;$script:lastCenterCount=$R.CC
  if($script:pending -and $script:pending.C -ne $R.C -and $R.KC -eq 1){$ack.Text='K3中心已更新；待确认的旧中心修改已取消，请核对后再应用';$script:pending=$null}
 }
 $centerAck.Text=('已确认中心 X：{0}　K3：{1}　记录次数：{2}（重新下载/断电后需重设）' -f $R.C,(Center-Text $R.KC),$R.CC)"""),
 ("function Motion-Text($R){", """function Center-Text([int]$Code){
 $names=@('尚未按键记录','已记录','识别无效或过期，未记录','等待配置/有效新帧','正在动作，未记录','故障锁存，未记录','K4按下，未记录','与串口提交冲突，请重按')
 if($Code -ge 0 -and $Code -lt $names.Count){return $names[$Code]};return ('未知 '+$Code)
}
function Motion-Text($R){"""),
 ("-or $line.StartsWith('VIS=2'))", "-or $line.StartsWith('VIS=2') -or $line.StartsWith('VIS=3'))"),
 ('旧版观察固件','旧版固件'),('空载，K2单次动作','K3定中心，K2单次动作')])
# Authorize the new key behavior in this isolated project, retaining parent history.
p=root/'AGENTS.md';s=p.read_text(encoding='utf-8');s+='''
## Current K3 center milestone (2026-10-22)

The user explicitly requested and selected board K3 to record the current ball X as the visual control center. Work here independently; preserve the VIS=3 parent and archives. K3 accepts only fresh valid vision, completed settings, no fault, no K4 and idle motor; K1 may be off. K3 never moves the motor or changes the encoder origin/limits. Held-at-boot/long-held keys cannot replay. Parameter freshness and K2 release qualification remain mandatory. Valid UART commits win exact-cycle collisions. Deliver matching VIS=4 host/bit and board steps. User's parent confirmation is qualitative plus limited UART evidence, not full four-direction acceptance. Later F7 at P76 must not be erased by K3; do not loosen +/-22 protection.
''';p.write_text(s,encoding='utf-8')
print('Migrated VIS=4 center reporting, host and retained tests')
