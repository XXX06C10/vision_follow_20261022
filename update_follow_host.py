"""One-time VIS5 Chinese panel migration. Normal use starts host/start_panel.cmd."""
from pathlib import Path
import json
p=Path(__file__).resolve().parent/'host/vision_panel_cn.ps1'
s=p.read_text(encoding='utf-8-sig')
assert "'^VIS=4 '" in s, 'Already migrated'
layout=json.loads((p.parent.parent/'frame_layout.json').read_text())['layout']
schema=','.join(repr(f'{k}:{n}') for k,n,_,kind in layout if kind!='literal')
start=s.index('$schema=');end=s.index('\n',start)
s=s[:start]+f'$schema=@({schema})'+s[end:]
s=s.replace('VIS=4','VIS=5').replace('VIS4','VIS5')
s=s.replace("'OK','RD'))", "'OK','RD','AR','CZ'))")
s=s.replace('if($r.T -ge 128){$r.T-=256};return $r', 'foreach($k in @(\'T\',\'AT\')){if($r[$k] -ge 128){$r[$k]-=256}};if($r.VE -ge 32768){$r.VE-=65536};return $r')
extra=r'''
function New-FollowPacket([int]$Seq,[int]$Mode,[int]$Damping,[int]$Slew,[int]$Runtime){
 if($Seq -lt 0 -or $Seq -gt 65535 -or $Mode -lt 0 -or $Mode -gt 2 -or $Damping -lt 0 -or $Damping -gt 16 -or $Slew -lt 1 -or $Slew -gt 8 -or $Runtime -lt 0 -or $Runtime -gt 300){throw '连续控制参数超出FPGA范围'}
 $sum=($Seq -shr 8) -bxor ($Seq -band 255) -bxor $Mode -bxor $Damping -bxor $Slew -bxor ($Runtime -shr 8) -bxor ($Runtime -band 255)
 return ('S{0:X4}{1:X2}{2:X2}{3:X2}{4:X4}{5:X2}' -f $Seq,$Mode,$Damping,$Slew,$Runtime,$sum)+"`n"
}
function Test-FollowReadback($R,$P){
 foreach($k in @('DS','MD','KD','SL','RU')){if($R[$k] -ne $P[$k]){return $false}};return $true
}
function Mode-Text([int]$Code){return @('单次视觉动作','连续PD控制','回软件参考位')[$Code]}
'''
s=s.replace('function Convert-RoiToSensor',extra+'\nfunction Convert-RoiToSensor')
s=s.replace("'参数改变/等待配置','故障停止')", "'参数改变/等待配置','故障停止','K2停止会话','会话时限到达')")
s=s.replace("'请先松开K2再按','故障锁存')", "'请先松开K2再按','故障锁存','先用板上K3确认中心')")
s=s.replace("if($R.F -ne 0){return '故障锁存'};", "if($R.F -ne 0){return '故障锁存'};if($R.AR -eq 1){return '连续会话运行中'};")
s=s.replace(" CC=0001 KC=1'", " CC=0001 KC=1 MD=1 AR=1 KD=0C SL=02 RU=001E DS=1234 CZ=1 VE=FFFC AT=FB SR=7 AJ=A EL=0003 TS=12345678'")
s=s.replace(" $capture=Convert-Record $sample;",r''' if((New-FollowPacket 1 1 12 2 30) -cne "S0001010C02001E10`n"){throw 'S报文协议错误'}
 $sb=0;foreach($args in @(@(1,3,12,2,30),@(1,1,17,2,30),@(1,1,12,0,30),@(1,1,12,2,301))){try{New-FollowPacket @args | Out-Null}catch{$sb++}}
 if($sb -ne 4 -or (New-FollowPacket 65535 2 0 8 0).Length -ne 18){throw 'S报文边界错误'}
 $sr=Convert-Record $sample;if($sr.VE -ne -4 -or $sr.AT -ne -5 -or $sr.RT -ne 144 -or $sr.RU -ne 30 -or !(Test-FollowReadback $sr @{DS=4660;MD=1;KD=12;SL=2;RU=30})){throw 'VIS5新增字段/蓝框顶部与会话时限解析错误'}
 $capture=Convert-Record $sample;''')
s=s.replace('HOST_SELF_TEST_PASS: VIS5 large60/gain0..12 bounds and signed decode, unchanged P/V format/checksum, ROI scaling and completion ack', 'HOST_SELF_TEST_PASS: VIS5 signed Q4 velocity, unique RT/RU fields, S bounds/checksum/readback, unchanged P/V packets, ROI and completion ack')
s=s.replace('PH1P35 视觉单次动作±60｜K1使能·K2执行·K3定中心·K4停止','PH1P35 钢球视觉闭环｜K3定中心·K1使能·K2开始/停止·K4急停')
s=s.replace('空载：K1使能、K2执行、K3定中心、K4停止；丢球/改参数取消动作。大幅度先应用限幅60、衰减档2。','先K3确认实际中心，再K1使能；连续模式按K2开始/停止，K4立即停机。改参数/丢球会停止，需重新按K2。')
s=s.replace("'视觉目标参数（K2按下时采样）'", "'视觉目标参数（连续模式实时更新）'")
s=s.replace("656 0 12 7", "656 0 12 4").replace("694 0 60 5", "694 0 60 12")
anchor="$script:serial=$null;$script:writer=$null;$script:buffer='';$script:lastRecord=$null;$script:lastRx=[DateTime]::MinValue"
ui=r'''
# Keep the proven ROI controls and mouse handling; organize controls into two tabs.
$leftControls=@($form.Controls | Where-Object {$_.Left -lt 350 -and $_.Top -ge 100 -and $_.Top -lt 850})
$leftTabs=New-Object Windows.Forms.TabControl;$leftTabs.SetBounds(10,100,330,780);$form.Controls.Add($leftTabs)
$cameraTab=New-Object Windows.Forms.TabPage('摄像头 / 蓝框');$controlTab=New-Object Windows.Forms.TabPage('闭环控制')
[void]$leftTabs.TabPages.Add($cameraTab);[void]$leftTabs.TabPages.Add($controlTab);$leftTabs.SelectedIndex=1
foreach($v in $leftControls){$oldY=$v.Top;$v.Left-=10;if($oldY -lt 545){$v.Top-=100;$cameraTab.Controls.Add($v)}else{$v.Top-=540;$controlTab.Controls.Add($v)}}
Label-At '运行模式（仅设置；板上K2启动）' 20 850 305 | Out-Null
$mode=New-Object Windows.Forms.ComboBox;$mode.DropDownStyle='DropDownList';[void]$mode.Items.AddRange(@('0 单次视觉动作','1 连续PD控制','2 回软件参考位'));$mode.SelectedIndex=1;$mode.SetBounds(10,350,305,30);$controlTab.Controls.Add($mode)
$damping=Number-At '速度阻尼（0～16）' 850 0 16 12;$slew=Number-At '目标缓变（计数/100ms）' 850 1 8 2;$runtime=Number-At '会话时限秒（0=不限）' 850 0 300 30
$applyFollow=Button-At '应用运行模式 / 阻尼 / 时限' 20 850 305
$stopFollow=Button-At '停止会话（保持使能）' 20 850 305
# Reparent newly created helpers; original coordinates are irrelevant after this step.
$newLabels=@($form.Controls | Where-Object {$_.Top -eq 850 -and $_ -is [Windows.Forms.Label]})
$j=0;foreach($v in $newLabels){$v.Left=10;$v.Top=@(315,395,433,471)[$j];$controlTab.Controls.Add($v);$j++}
$damping.SetBounds(195,395,120,28);$slew.SetBounds(195,433,120,28);$runtime.SetBounds(195,471,120,28)
foreach($v in @($damping,$slew,$runtime,$applyFollow,$stopFollow)){$controlTab.Controls.Add($v)}
$applyFollow.SetBounds(10,515,305,32);$stopFollow.SetBounds(10,557,305,32)
$followAck=New-Object Windows.Forms.Label;$followAck.SetBounds(10,600,305,65);$followAck.Text='默认：连续PD，G4 / L12，阻尼12。配置不会启动电机。';$controlTab.Controls.Add($followAck)
$tip=New-Object Windows.Forms.Label;$tip.SetBounds(10,675,305,65);$tip.Text='回参考位是K2确认的软件零位动作。无机械原点传感器。反向设置请先停机再应用。';$tip.ForeColor=[Drawing.Color]::DimGray;$controlTab.Controls.Add($tip)
$plotTabs=New-Object Windows.Forms.TabControl;$plotTabs.SetBounds(356,80,808,480);$form.Controls.Add($plotTabs)
$positionTab=New-Object Windows.Forms.TabPage('位置 / 蓝框');$curveTab=New-Object Windows.Forms.TabPage('误差 / 电机曲线')
[void]$plotTabs.TabPages.Add($positionTab);[void]$plotTabs.TabPages.Add($curveTab);$positionTab.Controls.Add($plot);$plot.SetBounds(0,0,800,450)
$curve=New-Object Windows.Forms.Panel;$curve.SetBounds(0,0,800,450);$curve.BackColor=[Drawing.Color]::FromArgb(24,31,44);$curveTab.Controls.Add($curve)
$script:history=New-Object 'Collections.Generic.Queue[object]';$script:followPending=$null;$script:lastFollowSequence=$null
$details.Font=New-Object Drawing.Font('Microsoft YaHei UI',9)
'''
s=s.replace(anchor,ui+'\n'+anchor)
s=s.replace("$script:pending=$null;$script:cameraPending=$null;$script:lastRecord=$null;", "$script:pending=$null;$script:cameraPending=$null;$script:followPending=$null;$script:lastFollowSequence=$null;$script:history.Clear();$script:lastCenterCount=$null;$script:lastRecord=$null;")
s=s.replace("$plot.Invalidate()\n}\nfunction Require-Link", "$plot.Invalidate();$curve.Invalidate()\n}\nfunction Require-Link")
s=s.replace(" $script:lastRecord=$R;$script:lastRx=[DateTime]::Now",r''' $script:lastRecord=$R;$script:lastRx=[DateTime]::Now
 $script:history.Enqueue(@{Error=($R.BX-$R.C);P=$R.P;AT=$R.AT;MT=$R.MT;Valid=($R.V -eq 1 -and $R.AGE -le 200)})
 while($script:history.Count -gt 240){[void]$script:history.Dequeue()}
 if($null -eq $script:lastFollowSequence -or $script:lastFollowSequence -ne $R.DS){
  if($R.MD -le 2 -and $R.KD -le 16 -and $R.SL -ge 1 -and $R.SL -le 8 -and $R.RU -le 300){$mode.SelectedIndex=$R.MD;$damping.Value=$R.KD;$slew.Value=$R.SL;$runtime.Value=$R.RU}
  $script:lastFollowSequence=$R.DS
 }
 if($script:followPending -and (Test-FollowReadback $R $script:followPending)){$followAck.Text='运行设置已完整确认，序号 '+$R.DS+'。按板上K2重新启动。';$script:followPending=$null}
 $curve.Invalidate()''')
s=s.replace("（重新下载/断电后需重设）' -f $R.C,(Center-Text $R.KC),$R.CC)", "；连续中心确认：{3}' -f $R.C,(Center-Text $R.KC),$R.CC,$R.CZ)")
st=s.index(" $details.Text=");en=s.index('\n $script:latestLines.Enqueue',st)
s=s[:st]+r''' $details.Text=('{0}　会话AR：{1}　已运行：{2}s / {3}s（0不限）　速度：{4:N2}像素/帧' -f (Mode-Text $R.MD),$R.AR,$R.EL,$R.RU,($R.VE/16.0))+"`r`n"+
  ('候选T：{0}　缓变AT：{1}　执行MT：{2}　位置P：{3}　误差ME：{4}　发步：{5}/64' -f $R.T,$R.AT,$R.MT,$R.P,$R.ME,$R.SP)+"`r`n"+
  ('球：({0}, {1})　中心：{2}　有效：{3}　帧龄：{4}ms　置信度：{5}/15' -f $R.BX,$R.BY,$R.C,$R.V,$R.AGE,$R.Q)+"`r`n"+
  ('{0}　K2就绪：{1}　发步段接受/完成：{2}/{3}　累计发步TS：{4}　故障：{5}' -f (Motion-Text $R),$R.RD,$R.AC,$R.DC,$R.TS,$R.F)+"`r`n"+
  ('会话停止：{0}　执行器停止：{1}　K2拒绝：{2}　AB错误：{3}' -f (Cancel-Text $R.SR),(Cancel-Text $R.CR),(Reject-Text $R.RJ),$R.X1)
''' +s[en:]
# PC setting operations never start motion, and any valid S atomically cancels a session.
at=s.index('foreach($control in @($roiLeft')
s=s[:at]+r'''
function Send-FollowSettings([bool]$StopOnly){
 Require-Link;if($script:followPending){throw '上一组运行设置尚待确认'}
 $r=$script:lastRecord
 $p=if($StopOnly){@{DS=(($r.DS+1) -band 65535);MD=$r.MD;KD=$r.KD;SL=$r.SL;RU=$r.RU;Sent=[DateTime]::Now}}else{@{DS=(($r.DS+1) -band 65535);MD=$mode.SelectedIndex;KD=[int]$damping.Value;SL=[int]$slew.Value;RU=[int]$runtime.Value;Sent=[DateTime]::Now}}
 $script:serial.Write((New-FollowPacket $p.DS $p.MD $p.KD $p.SL $p.RU));$script:followPending=$p;$followAck.Text='运行设置已发送，等待全部字段回读；连续会话停止，保持使能。'
}
$applyFollow.Add_Click({try{Send-FollowSettings $false}catch{[void][Windows.Forms.MessageBox]::Show($_.Exception.Message,'运行设置')}})
$stopFollow.Add_Click({try{Send-FollowSettings $true}catch{[void][Windows.Forms.MessageBox]::Show($_.Exception.Message,'停止会话')}})
$curve.Add_Paint({
 param($sender,$e);$g=$e.Graphics;$g.SmoothingMode='AntiAlias';$rows=@($script:history.ToArray());$w=740;$left=48
 $g.DrawString('近240条报告（约60秒）；蓝：球横向误差；绿：实际P；橙：缓变AT；灰：执行MT',$form.Font,[Drawing.Brushes]::White,12,8)
 $grid=New-Object Drawing.Pen([Drawing.Color]::FromArgb(65,76,96),1)
 foreach($y in @(60,140,220,265,345,425)){$g.DrawLine($grid,$left,$y,788,$y)};$grid.Dispose()
 foreach($v in @(@('400',55),@('0px',130),@('-400',207),@('+64',255),@('0',335),@('-64',410))){$g.DrawString($v[0],$form.Font,[Drawing.Brushes]::Silver,2,[single]$v[1])}
 if($rows.Count -lt 2){return}
 foreach($spec in @(@('Error',[Drawing.Color]::DeepSkyBlue,140,0.2),@('P',[Drawing.Color]::LimeGreen,345,1.25),@('AT',[Drawing.Color]::Orange,345,1.25),@('MT',[Drawing.Color]::Silver,345,1.25))){
  $pen=New-Object Drawing.Pen($spec[1],2);$previous=$null
  for($i=0;$i -lt $rows.Count;$i++){
   if($spec[0] -eq 'Error' -and !$rows[$i].Valid){$previous=$null;continue}
   $value=[Math]::Max(-400,[Math]::Min(400,[double]$rows[$i][$spec[0]]));$point=New-Object Drawing.PointF([single]($left+$i*$w/239.0),[single]($spec[2]-$value*$spec[3]))
   if($previous){$g.DrawLine($pen,$previous,$point)};$previous=$point
  };$pen.Dispose()
 }
})
''' +s[at:]
s=s.replace("$line.StartsWith('VIS=3')", "$line.StartsWith('VIS=3') -or $line.StartsWith('VIS=4')")
s=s.replace('请烧入配套VIS=5新bit后再测试动作','请烧入配套VIS=5新bit')
s=s.replace("  if($script:cameraPending -and ([DateTime]", "  if($script:followPending -and ([DateTime]::Now-$script:followPending.Sent).TotalSeconds -gt 2){$followAck.Text='运行设置未确认；请用板上K2停止或K4停机，未自动重发';$script:followPending=$null}\n  if($script:cameraPending -and ([DateTime]")
s=s.replace('正在接收VIS=5｜K3定中心，K2单次动作','正在接收VIS=5｜K2按运行模式开始/停止')
s=s.replace(" if($limit.Maximum -ne 60", " if($mode.SelectedIndex -ne 1 -or $damping.Value -ne 12 -or $runtime.Value -ne 30){throw '连续模式默认值错误'}\n if($limit.Maximum -ne 60")
s=s.replace(" $form.Opacity=0;",r''' for($j=0;$j -lt 240;$j++){$q=Convert-Record $capturedSample;$q.BX=450+[int](120*[Math]::Sin($j/22.0));$q.P=[int](8*[Math]::Cos($j/22.0));$q.AT=$q.P+1;Show-Record $q $capturedSample}
 $form.Opacity=0;''')
s=s.replace("$bitmap.Dispose();$form.Close();",r'''$bitmap.Dispose()
 $plotTabs.SelectedIndex=1;[Windows.Forms.Application]::DoEvents();$bitmap=New-Object Drawing.Bitmap($form.Width,$form.Height);$form.DrawToBitmap($bitmap,$bounds);$bitmap.Save((Join-Path $PSScriptRoot 'panel_curves_preview.png'),[Drawing.Imaging.ImageFormat]::Png);$bitmap.Dispose();$form.Close();''')
s=s.replace('Chinese form, camera settings and ROI preview rendered; no serial opened','VIS5 Chinese modes/damping/runtime, center readback, ROI draw/move and 240-point curves rendered; no serial opened')
p.write_text(s,encoding='utf-8-sig')
print('Updated Chinese VIS5 panel')
