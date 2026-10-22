param([switch]$SelfTest,[switch]$SmokeTest)
$ErrorActionPreference='Stop'
$schema=@('BX:4','BY:4','V:1','AGE:4','FR:4','EN:1','Z:1','F:1','E1:8','E2:8','X1:4','X2:4','P:8','T:2','C:4','DZ:4','G:2','L:2','SEQ:4','RXE:4','Q:1','B:1','CS:4','AE:4','AG:4','RL:4','RT:4','RR:4','RB:4','CA:4','RA:4','CB:1','MT:8','ME:8','MS:1','MB:1','OK:1','RD:1','SP:2','MC:8','CR:1','RJ:1','AC:4','DC:4','CC:4','KC:1','MD:1','AR:1','KD:2','SL:2','RU:4','DS:4','CZ:1','VE:4','AT:2','SR:1','AJ:1','EL:4','TS:8')
$script:Pattern='^VIS=5 '+(($schema|ForEach-Object{$a=$_.Split(':');$a[0]+'=([0-9A-F]{'+$a[1]+'})'}) -join ' ')+'$'
$script:Names=@($schema|ForEach-Object{$_.Split(':')[0]})
function Convert-Record([string]$Line){
 $m=[regex]::Match($Line.Trim(),$script:Pattern);if(!$m.Success){return $null}
 $r=@{};for($i=0;$i -lt $script:Names.Count;$i++){$r[$script:Names[$i]]=[Convert]::ToInt64($m.Groups[$i+1].Value,16)}
 foreach($k in @('V','EN','Z','B','CB','MB','OK','RD','AR','CZ')){if($r[$k] -gt 1){return $null}}
 if($r.MD -gt 2 -or $r.KD -gt 16 -or $r.SL -lt 1 -or $r.SL -gt 8 -or $r.RU -gt 300){return $null}
 foreach($k in @('E1','E2','P','MT','ME','MC')){if($r[$k] -ge 2147483648){$r[$k]-=4294967296}}
 foreach($k in @('T','AT')){if($r[$k] -ge 128){$r[$k]-=256}};if($r.VE -ge 32768){$r.VE-=65536};return $r
}
function New-ParameterPacket([int]$Seq,[int]$Center,[int]$Deadzone,[int]$Gain,[int]$Limit){
 if($Seq -lt 0 -or $Seq -gt 65535 -or $Center -lt 0 -or $Center -gt 1919 -or $Deadzone -lt 0 -or $Deadzone -gt 200 -or ($Gain -band 127) -lt 0 -or ($Gain -band 127) -gt 12 -or $Gain -lt 0 -or $Gain -gt 255 -or $Limit -lt 0 -or $Limit -gt 60){throw '控制参数超出FPGA允许范围'}
 $sum=0;foreach($v in @(($Seq -shr 8),($Seq -band 255),($Center -shr 8),($Center -band 255),($Deadzone -shr 8),($Deadzone -band 255),$Gain,$Limit)){$sum=$sum -bxor $v}
 return ('P{0:X4}{1:X4}{2:X4}{3:X2}{4:X2}{5:X2}' -f $Seq,$Center,$Deadzone,$Gain,$Limit,$sum)+"`n"
}

function New-FollowPacket([int]$Seq,[int]$Mode,[int]$Damping,[int]$Slew,[int]$Runtime){
 if($Seq -lt 0 -or $Seq -gt 65535 -or $Mode -lt 0 -or $Mode -gt 2 -or $Damping -lt 0 -or $Damping -gt 16 -or $Slew -lt 1 -or $Slew -gt 8 -or $Runtime -lt 0 -or $Runtime -gt 300){throw '连续控制参数超出FPGA范围'}
 $sum=($Seq -shr 8) -bxor ($Seq -band 255) -bxor $Mode -bxor $Damping -bxor $Slew -bxor ($Runtime -shr 8) -bxor ($Runtime -band 255)
 return ('S{0:X4}{1:X2}{2:X2}{3:X2}{4:X4}{5:X2}' -f $Seq,$Mode,$Damping,$Slew,$Runtime,$sum)+"`n"
}
function Test-FollowReadback($R,$P){
 foreach($k in @('DS','MD','KD','SL','RU')){if($R[$k] -ne $P[$k]){return $false}};return $true
}
function Mode-Text([int]$Code){return @('单次视觉动作','连续PD控制','回软件参考位')[$Code]}

function Convert-RoiToSensor([int]$Left,[int]$Top,[int]$Right,[int]$Bottom){
 if($Left -lt 0 -or $Top -lt 0 -or $Right -gt 1919 -or $Bottom -gt 1079 -or $Left -ge $Right -or $Top -ge $Bottom){throw '蓝框必须满足：左小于右、上小于下，并且在1920×1080画面内'}
 $r=@{RL=[int][Math]::Floor($Left*2.0/3);RT=[int][Math]::Floor($Top*2.0/3);RR=[int][Math]::Floor($Right*2.0/3);RB=[int][Math]::Floor($Bottom*2.0/3)}
 if($r.RL -ge $r.RR -or $r.RT -ge $r.RB){throw '蓝框太窄：转换到摄像头像素后宽度和高度必须大于0'};return $r
}
function New-CameraPacket([int]$Seq,[int]$Exposure,[int]$SensorGain,[int]$Left,[int]$Top,[int]$Right,[int]$Bottom){
 if($Seq -lt 0 -or $Seq -gt 65535 -or $Exposure -lt 2 -or $Exposure -gt 3600 -or $SensorGain -lt 16 -or $SensorGain -gt 143 -or $Left -lt 0 -or $Top -lt 0 -or $Right -gt 1279 -or $Bottom -gt 719 -or $Left -ge $Right -or $Top -ge $Bottom){throw '曝光、传感器增益或蓝框超出允许范围'}
 $sum=0;$packet='V';foreach($v in @($Seq,$Exposure,$SensorGain,$Left,$Top,$Right,$Bottom)){$packet+=('{0:X4}' -f $v);$sum=$sum -bxor ($v -shr 8) -bxor ($v -band 255)}
 return $packet+('{0:X2}' -f $sum)+"`n"
}
function Test-Readback($R,$P){return $R.SEQ -eq $P.SEQ -and $R.C -eq $P.C -and $R.DZ -eq $P.DZ -and $R.G -eq $P.G -and $R.L -eq $P.L}
function Test-CameraReadback($R,$P){
 if($R.CB -ne 0 -or $R.CS -ne $P.CS -or $R.CA -ne $P.CS -or $R.RA -ne $P.CS){return $false}
 foreach($k in @('AE','AG','RL','RT','RR','RB')){if($R[$k] -ne $P[$k]){return $false}};return $true
}
function Cancel-Text([int]$Code){
 $names=@('无','K1关闭使能','K4停止','识别失效/坏帧','超过200ms无新结果','参数改变/等待配置','故障停止','K2停止会话','会话时限到达')
 if($Code -ge 0 -and $Code -lt $names.Count){return $names[$Code]};return ('未知原因 '+$Code)
}
function Reject-Text([int]$Code){
 $names=@('无','尚未使能','尚未就绪','识别无效','视觉结果过期','等待配置后的有效新帧','正在动作，不排队','目标超过±60','请先松开K2再按','故障锁存','先用板上K3确认中心')
 if($Code -ge 0 -and $Code -lt $names.Count){return $names[$Code]};return ('未知原因 '+$Code)
}
function Center-Text([int]$Code){
 $names=@('尚未按键记录','已记录','识别无效或过期，未记录','等待配置/有效新帧','正在动作，未记录','故障锁存，未记录','K4按下，未记录','与串口提交冲突，请重按')
 if($Code -ge 0 -and $Code -lt $names.Count){return $names[$Code]};return ('未知 '+$Code)
}
function Motion-Text($R){
 if($R.F -ne 0){return '故障锁存'};if($R.AR -eq 1 -and $R.V -eq 0){return '短丢保持，最长500ms'};if($R.AR -eq 1){return '连续会话运行中'};if($R.MB -eq 1){return '执行中'};if($R.EN -eq 0){return '未使能'}
 if($R.Z -eq 0){return '建立软件参考'};if($R.OK -eq 1){return '本次已到位'};if($R.RD -eq 1){return '就绪，可松按K2'}
 return '等待有效识别/配置'
}
$sample='VIS=5 BX=01C2 BY=021C V=1 AGE=0001 FR=0002 EN=1 Z=1 F=0 E1=FFFFFFFC E2=00000000 X1=0000 X2=0000 P=00000004 T=FB C=03C0 DZ=0014 G=06 L=05 SEQ=1234 RXE=0000 Q=F B=0 CS=0001 AE=0E10 AG=008F RL=0100 RT=0090 RR=0400 RB=0240 CA=0001 RA=0001 CB=0 MT=00000005 ME=00000001 MS=2 MB=0 OK=1 RD=1 SP=03 MC=00000003 CR=0 RJ=0 AC=0002 DC=0001 CC=0001 KC=1 MD=1 AR=1 KD=0C SL=02 RU=001E DS=1234 CZ=1 VE=FFFC AT=FB SR=7 AJ=A EL=0003 TS=12345678'
if($SelfTest){
 if((New-FollowPacket 1 1 12 2 30) -cne "S0001010C02001E10`n"){throw 'S报文协议错误'}
 $sb=0;foreach($args in @(@(1,3,12,2,30),@(1,1,17,2,30),@(1,1,12,0,30),@(1,1,12,2,301))){try{New-FollowPacket @args | Out-Null}catch{$sb++}}
 if($sb -ne 4 -or (New-FollowPacket 65535 2 0 8 0).Length -ne 18){throw 'S报文边界错误'}
 $sr=Convert-Record $sample;if($sr.VE -ne -4 -or $sr.AT -ne -5 -or $sr.RT -ne 144 -or $sr.RU -ne 30 -or !(Test-FollowReadback $sr @{DS=4660;MD=1;KD=12;SL=2;RU=30})){throw 'VIS5新增字段/蓝框顶部与会话时限解析错误'}
 $coast=Convert-Record ($sample.Replace(' V=1 ',' V=0 ').Replace(' Q=F ',' Q=0 '));if(!$coast -or $coast.T -ne -5 -or (Motion-Text $coast) -notlike '*500ms*'){throw '短丢状态或历史目标解析错误'}
 $capture=Convert-Record $sample;if($capture.CC -ne 1 -or $capture.KC -ne 1){throw 'K3中心状态解析失败'}
 $r=Convert-Record $sample;if(!$r -or $r.E1 -ne -4 -or $r.T -ne -5 -or $r.AG -ne 143 -or $r.AE -ne 3600 -or $r.MT -ne 5 -or $r.ME -ne 1 -or $r.MC -ne 3){throw '中文界面解析测试失败'}
 $negative=Convert-Record ($sample.Replace('MT=00000005','MT=FFFFFFFB').Replace('ME=00000001','ME=FFFFFFF7').Replace('MC=00000003','MC=FFFFFFFC'))
 if(!$negative -or $negative.MT -ne -5 -or $negative.ME -ne -9 -or $negative.MC -ne -4){throw '负数运动目标/误差/指令解析失败'}
 if($null -ne (Convert-Record ($sample+'x')) -or $null -ne (Convert-Record $sample.Substring(0,100))){throw '错误或截断帧未拒绝'}
 if((New-ParameterPacket 1 960 20 7 5) -cne "P000103C000140705D4`n"){throw '控制报文与已验证协议不一致'}
 if((New-ParameterPacket 1 960 20 0 60) -cne "P000103C00014003CEA`n"){throw '大目标P报文格式或校验变化'}
 foreach($g in @(0,1,2,128,129,130)){if((New-ParameterPacket 1 960 20 $g 60).Length -ne 20){throw '低衰减档或反向位不兼容'}}
 $rejected=0;foreach($pair in @(@(13,60),@(255,60),@(0,61),@(0,-1))){try{New-ParameterPacket 1 960 20 $pair[0] $pair[1] | Out-Null}catch{$rejected++}}
 if($rejected -ne 4){throw '大目标参数边界测试失败'}
 $large=Convert-Record ($sample.Replace(' T=FB ',' T=C4 ').Replace('MT=00000005','MT=FFFFFFC4').Replace('SP=03','SP=2F'))
 if(!$large -or $large.T -ne -60 -or $large.MT -ne -60 -or $large.SP -ne 47){throw '大目标有符号报告解析失败'}
 $roi=Convert-RoiToSensor 0 0 1919 1079;if($roi.RR -ne 1279 -or $roi.RB -ne 719){throw '蓝框坐标缩放失败'}
 $packet=New-CameraPacket 1 3600 143 0 0 1279 719
 if($packet.Length -ne 32 -or $packet -cne "V00010E10008F0000000004FF02CFA6`n"){throw ('摄像头报文不匹配：'+$packet)}
 $p=@{CS=1;AE=3600;AG=143;RL=256;RT=144;RR=1024;RB=576}
 if(!(Test-CameraReadback $r $p)){throw '完整完成回读未确认'}
 $r.CA=0;if(Test-CameraReadback $r $p){throw '摄像头未完成却误确认'};$r.CA=1;$r.RA=0;if(Test-CameraReadback $r $p){throw '蓝框未应用却误确认'}
 $r.RA=1;$r.CB=1;if(Test-CameraReadback $r $p){throw '配置忙却误确认'}
 $bad=0;foreach($values in @(@(1,3601,143,0,0,1279,719),@(1,3600,144,0,0,1279,719),@(1,3600,143,500,0,500,719))){try{New-CameraPacket @values | Out-Null}catch{$bad++}}
 if($bad -ne 3){throw '边界测试失败'}
 Write-Output 'HOST_SELF_TEST_PASS: VIS5 signed Q4 velocity, unique RT/RU fields, S bounds/checksum/readback, unchanged P/V packets, ROI, completion ack and raw-invalid coasting status';exit
}
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[Windows.Forms.Application]::EnableVisualStyles()
$form=New-Object Windows.Forms.Form;$form.Text='PH1P35 钢球视觉闭环｜K3定中心·K1使能·K2开始/停止·K4急停｜短丢500ms'
$form.AutoScaleMode='None';$form.ClientSize=New-Object Drawing.Size(1180,932);$form.MinimumSize=New-Object Drawing.Size(1196,971)
$form.Font=New-Object Drawing.Font('Microsoft YaHei UI',10)
function Label-At([string]$Text,[int]$X,[int]$Y,[int]$W=300,[int]$H=28){$v=New-Object Windows.Forms.Label;$v.Text=$Text;$v.SetBounds($X,$Y,$W,$H);$form.Controls.Add($v);return $v}
function Button-At([string]$Text,[int]$X,[int]$Y,[int]$W=140){$v=New-Object Windows.Forms.Button;$v.Text=$Text;$v.SetBounds($X,$Y,$W,32);$form.Controls.Add($v);return $v}
function Number-At([string]$Text,[int]$Y,[int]$Min,[int]$Max,[int]$Value){
 Label-At $Text 20 $Y 170 | Out-Null;$v=New-Object Windows.Forms.NumericUpDown;$v.SetBounds(195,$Y,130,28);$v.Minimum=$Min;$v.Maximum=$Max;$v.Value=$Value;$form.Controls.Add($v);return $v
}
Label-At '串口' 20 22 65 | Out-Null
$ports=New-Object Windows.Forms.ComboBox;$ports.SetBounds(90,19,140,28);$ports.DropDownStyle='DropDownList';$form.Controls.Add($ports)
$refresh=Button-At '刷新串口' 245 17 100;$connect=Button-At '连接' 360 17 100
$status=Label-At '未连接｜115200，8N1' 480 22 670
Label-At '先K3确认实际中心，再K1使能；连续模式按K2开始/停止，K4立即停机。短丢保持500ms；断流200ms/改参数停止，需重新K2。' 20 64 1140 30 | Out-Null
Label-At '摄像头亮度' 20 105 | Out-Null
$exposure=Number-At '曝光值（2～3600）' 140 2 3600 3600
$sensorGain=Number-At '传感器增益（16～143）' 178 16 143 143
$maximum=Button-At '填入最大亮度：3600 / 143' 20 216 305
Label-At '蓝框识别范围（HDMI像素）' 20 260 | Out-Null
$roiLeft=Number-At '左边界 X' 295 0 1919 0;$roiTop=Number-At '上边界 Y' 333 0 1079 0
$roiRight=Number-At '右边界 X' 371 0 1919 1919;$roiBottom=Number-At '下边界 Y' 409 0 1079 1079
$full=Button-At '恢复全画面' 20 449 145;$roiCenter=Button-At '控制中心对齐蓝框' 180 449 145
$applyCamera=Button-At '应用曝光 / 增益 / 蓝框' 20 491 305
Label-At '视觉目标参数（连续模式实时更新）' 20 545 | Out-Null
$center=Number-At '控制中心 X' 580 0 1919 960;$deadzone=Number-At '中心死区（像素）' 618 0 200 20
$gain=Number-At '控制衰减档（越大越弱）' 656 0 12 4;$limit=Number-At '目标限幅（编码器计数）' 694 0 60 12
$invert=New-Object Windows.Forms.CheckBox;$invert.Text='反转候选目标方向';$invert.SetBounds(20,733,305,30);$form.Controls.Add($invert)
$apply=Button-At '应用视觉目标参数' 20 773 305
$plot=New-Object Windows.Forms.Panel;$plot.SetBounds(360,110,800,450);$plot.BackColor=[Drawing.Color]::FromArgb(24,31,44);$form.Controls.Add($plot)
$drawMode=New-Object Windows.Forms.CheckBox;$drawMode.Text='重新画框（取消勾选后可拖动平移）';$drawMode.Checked=$true;$drawMode.SetBounds(360,570,400,28);$form.Controls.Add($drawMode)
Label-At '蓝实线：已应用；青虚线：预览；橙虚线：控制中心' 360 602 800 | Out-Null
$cameraAck=Label-At '摄像头/蓝框：尚未发送修改。新bit默认曝光3600、增益143。' 360 636 800 30
$ack=Label-At '控制参数：尚未发送修改' 360 671 800 30
$centerAck=Label-At '控制中心：默认960。把球放到实际中心，按板上K3确认；重新下载/断电后需重设。' 360 707 800 30
$details=Label-At '等待新版 VIS=5 串口数据……' 360 741 800 105
$raw=New-Object Windows.Forms.TextBox;$raw.Multiline=$true;$raw.ReadOnly=$true;$raw.ScrollBars='Vertical';$raw.WordWrap=$false;$raw.Font=New-Object Drawing.Font('Consolas',9);$raw.SetBounds(360,852,800,35);$form.Controls.Add($raw)
$logLabel=Label-At '连接后自动保存原始日志到 host/logs/' 20 902 1140

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

$script:serial=$null;$script:writer=$null;$script:buffer='';$script:lastRecord=$null;$script:lastRx=[DateTime]::MinValue
$script:pending=$null;$script:cameraPending=$null;$script:syncControls=$true;$script:actualRoi=$null
$script:latestLines=New-Object 'Collections.Generic.Queue[string]';$script:drag=$null
function Refresh-Ports{$chosen=$ports.Text;$ports.Items.Clear();foreach($p in [IO.Ports.SerialPort]::GetPortNames()|Sort-Object){[void]$ports.Items.Add($p)};if($ports.Items.Contains($chosen)){$ports.SelectedItem=$chosen}elseif($ports.Items.Count -gt 0){$ports.SelectedIndex=0}}
function Disconnect-Port{
 if($script:serial){$script:serial.Dispose();$script:serial=$null};if($script:writer){$script:writer.Dispose();$script:writer=$null}
 $script:pending=$null;$script:cameraPending=$null;$script:followPending=$null;$script:lastFollowSequence=$null;$script:history.Clear();$script:lastCenterCount=$null;$script:lastRecord=$null;$script:actualRoi=$null;$script:buffer='';$script:syncControls=$true;$script:lastRx=[DateTime]::MinValue
 $connect.Text='连接';$status.Text='已断开';$ack.Text='控制参数：无待确认修改';$cameraAck.Text='摄像头/蓝框：无待确认修改';$followAck.Text='运行设置：无待确认修改';$plot.Invalidate();$curve.Invalidate()
}
function Require-Link{
 if(!$script:serial -or !$script:serial.IsOpen){throw '请先连接串口'}
 if(!$script:lastRecord -or ([DateTime]::Now-$script:lastRx).TotalSeconds -gt 1.5){throw '需要先收到新版VIS=5实时数据，请确认已烧入配套新bit'}
}
function Show-Record($R,[string]$Line){
 $script:lastRecord=$R;$script:lastRx=[DateTime]::Now
 $script:history.Enqueue(@{Error=($R.BX-$R.C);P=$R.P;AT=$R.AT;MT=$R.MT;Valid=($R.V -eq 1 -and $R.AGE -le 200)})
 while($script:history.Count -gt 240){[void]$script:history.Dequeue()}
 if($null -eq $script:lastFollowSequence -or $script:lastFollowSequence -ne $R.DS){
  if($R.MD -le 2 -and $R.KD -le 16 -and $R.SL -ge 1 -and $R.SL -le 8 -and $R.RU -le 300){$mode.SelectedIndex=$R.MD;$damping.Value=$R.KD;$slew.Value=$R.SL;$runtime.Value=$R.RU}
  $script:lastFollowSequence=$R.DS
 }
 if($script:followPending -and (Test-FollowReadback $R $script:followPending)){$followAck.Text='运行设置已完整确认，序号 '+$R.DS+'。按板上K2重新启动。';$script:followPending=$null}
 $curve.Invalidate()
 if($null -eq $script:lastCenterCount -or $script:lastCenterCount -ne $R.CC){
  $center.Value=$R.C;$script:lastCenterCount=$R.CC
  if($script:pending -and $script:pending.C -ne $R.C -and $R.KC -eq 1){$ack.Text='K3中心已更新；待确认的旧中心修改已取消，请核对后再应用';$script:pending=$null}
 }
 $centerAck.Text=('已确认中心 X：{0}　K3：{1}　记录次数：{2}；连续中心确认：{3}' -f $R.C,(Center-Text $R.KC),$R.CC,$R.CZ)
 if($script:syncControls){
  if($R.AE -ge 2 -and $R.AE -le 3600 -and $R.AG -ge 16 -and $R.AG -le 143 -and $R.RR -le 1279 -and $R.RB -le 719){
   $exposure.Value=$R.AE;$sensorGain.Value=$R.AG;$roiLeft.Value=[Math]::Floor($R.RL*1.5);$roiTop.Value=[Math]::Floor($R.RT*1.5);$roiRight.Value=[Math]::Floor($R.RR*1.5);$roiBottom.Value=[Math]::Floor($R.RB*1.5)
   $center.Value=$R.C;$deadzone.Value=$R.DZ;$gain.Value=($R.G -band 127);$limit.Value=$R.L;$invert.Checked=($R.G -band 128) -ne 0;$script:syncControls=$false
  }
 }
 if($R.CB -eq 0 -and $R.CS -eq $R.RA){$script:actualRoi=@{Left=$R.RL*1.5;Top=$R.RT*1.5;Right=$R.RR*1.5;Bottom=$R.RB*1.5}}
 $details.Text=('{0}　会话AR：{1}　已运行：{2}s / {3}s（0不限）　速度：{4:N2}像素/帧' -f (Mode-Text $R.MD),$R.AR,$R.EL,$R.RU,($R.VE/16.0))+"`r`n"+
  ('候选T：{0}　缓变AT：{1}　执行MT：{2}　位置P：{3}　误差ME：{4}　发步：{5}/64' -f $R.T,$R.AT,$R.MT,$R.P,$R.ME,$R.SP)+"`r`n"+
  ('球：({0}, {1})　中心：{2}　有效：{3}　帧龄：{4}ms　置信度：{5}/15' -f $R.BX,$R.BY,$R.C,$R.V,$R.AGE,$R.Q)+"`r`n"+
  ('{0}　K2就绪：{1}　发步段接受/完成：{2}/{3}　累计发步TS：{4}　故障：{5}' -f (Motion-Text $R),$R.RD,$R.AC,$R.DC,$R.TS,$R.F)+"`r`n"+
  ('会话停止：{0}　执行器停止：{1}　K2拒绝：{2}　AB错误：{3}' -f (Cancel-Text $R.SR),(Cancel-Text $R.CR),(Reject-Text $R.RJ),$R.X1)

 $script:latestLines.Enqueue($Line);while($script:latestLines.Count -gt 8){[void]$script:latestLines.Dequeue()};$raw.Lines=$script:latestLines.ToArray()
 if($script:pending -and (Test-Readback $R $script:pending)){$ack.Text='控制参数：FPGA已确认全部参数，序号 '+$R.SEQ;$script:pending=$null}
 if($script:cameraPending -and (Test-CameraReadback $R $script:cameraPending)){$cameraAck.Text='摄像头/蓝框：配置流程和新帧应用均已完成，序号 '+$R.CS;$script:cameraPending=$null}
 $plot.Invalidate()
}
$refresh.Add_Click({Refresh-Ports})
$connect.Add_Click({
 if($script:serial){Disconnect-Port;return}
 try{
  if(!$ports.Text){throw '请选择串口'};$script:serial=New-Object IO.Ports.SerialPort($ports.Text,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
  $script:serial.DtrEnable=$false;$script:serial.RtsEnable=$false;$script:serial.ReadTimeout=50;$script:serial.WriteTimeout=500;$script:serial.Open();$script:serial.DiscardInBuffer()
  $logDir=Join-Path $PSScriptRoot 'logs';New-Item -ItemType Directory -Force -Path $logDir | Out-Null;$path=Join-Path $logDir ((Get-Date -Format 'yyyyMMdd_HHmmss_fff')+'.txt')
  $script:writer=New-Object IO.StreamWriter($path,$false,[Text.Encoding]::ASCII);$script:writer.AutoFlush=$true;$logLabel.Text='日志：'+$path
  $connect.Text='断开';$status.Text='已连接，等待新版数据……'
 }catch{Disconnect-Port;[void][Windows.Forms.MessageBox]::Show($_.Exception.Message,'串口连接失败')}
})
$maximum.Add_Click({$exposure.Value=3600;$sensorGain.Value=143;$cameraAck.Text='已填入最大亮度，点击“应用曝光 / 增益 / 蓝框”生效'})
$full.Add_Click({$roiLeft.Value=0;$roiTop.Value=0;$roiRight.Value=1919;$roiBottom.Value=1079;$plot.Invalidate()})
$roiCenter.Add_Click({$center.Value=[Math]::Floor(($roiLeft.Value+$roiRight.Value)/2);$ack.Text='已填入蓝框水平中心，点击“应用视觉目标参数”生效'})
$apply.Add_Click({
 try{Require-Link;if($script:pending){throw '上一组控制参数尚待确认'}
  $g=[int]$gain.Value;if($invert.Checked){$g=$g -bor 128};$p=@{SEQ=(([int]$script:lastRecord.SEQ+1) -band 65535);C=[int]$center.Value;DZ=[int]$deadzone.Value;G=$g;L=[int]$limit.Value;Sent=[DateTime]::Now}
  $script:serial.Write((New-ParameterPacket $p.SEQ $p.C $p.DZ $p.G $p.L));$script:pending=$p;$ack.Text='控制参数：已发送，等待完整回读，序号 '+$p.SEQ
 }catch{[void][Windows.Forms.MessageBox]::Show($_.Exception.Message,'控制参数')}
})
$applyCamera.Add_Click({
 try{Require-Link;if($script:cameraPending){throw '上一组曝光/蓝框修改尚待完成'}
  $roi=Convert-RoiToSensor $roiLeft.Value $roiTop.Value $roiRight.Value $roiBottom.Value
  $p=@{CS=(([int]$script:lastRecord.CS+1) -band 65535);AE=[int]$exposure.Value;AG=[int]$sensorGain.Value;RL=$roi.RL;RT=$roi.RT;RR=$roi.RR;RB=$roi.RB;Sent=[DateTime]::Now}
  $script:serial.Write((New-CameraPacket $p.CS $p.AE $p.AG $p.RL $p.RT $p.RR $p.RB));$script:cameraPending=$p;$cameraAck.Text='摄像头/蓝框：已发送，等待配置流程和新帧应用，序号 '+$p.CS
 }catch{[void][Windows.Forms.MessageBox]::Show($_.Exception.Message,'曝光、增益与蓝框')}
})

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
foreach($control in @($roiLeft,$roiTop,$roiRight,$roiBottom)){$control.Add_ValueChanged({$plot.Invalidate()})}
function Mouse-Point($E){return @{X=[int][Math]::Max(0,[Math]::Min(1919,[Math]::Floor($E.X*1920.0/$plot.Width)));Y=[int][Math]::Max(0,[Math]::Min(1079,[Math]::Floor($E.Y*1080.0/$plot.Height)))}}
$plot.Add_MouseDown({param($sender,$e);if($e.Button -ne [Windows.Forms.MouseButtons]::Left){return};$p=Mouse-Point $e;$script:drag=@{X=$p.X;Y=$p.Y;L=[int]$roiLeft.Value;T=[int]$roiTop.Value;R=[int]$roiRight.Value;B=[int]$roiBottom.Value;Draw=$drawMode.Checked};$plot.Capture=$true})
$plot.Add_MouseMove({
 param($sender,$e);if(!$script:drag){return};$p=Mouse-Point $e;$d=$script:drag
 if($d.Draw){$roiLeft.Value=[Math]::Min($d.X,$p.X);$roiTop.Value=[Math]::Min($d.Y,$p.Y);$roiRight.Value=[Math]::Max($d.X,$p.X);$roiBottom.Value=[Math]::Max($d.Y,$p.Y)}
 else{$dx=[Math]::Max(-$d.L,[Math]::Min(1919-$d.R,$p.X-$d.X));$dy=[Math]::Max(-$d.T,[Math]::Min(1079-$d.B,$p.Y-$d.Y));$roiLeft.Value=$d.L+$dx;$roiRight.Value=$d.R+$dx;$roiTop.Value=$d.T+$dy;$roiBottom.Value=$d.B+$dy}
 $plot.Invalidate()
})
$plot.Add_MouseUp({$script:drag=$null;$plot.Capture=$false})
$plot.Add_Paint({
 param($sender,$e);$g=$e.Graphics;$g.SmoothingMode='AntiAlias';$sx=$plot.Width/1920.0;$sy=$plot.Height/1080.0
 $pen=New-Object Drawing.Pen([Drawing.Color]::FromArgb(64,76,96),1);for($i=1;$i -lt 4;$i++){$g.DrawLine($pen,[single]($i*$plot.Width/4),0,[single]($i*$plot.Width/4),$plot.Height);$g.DrawLine($pen,0,[single]($i*$plot.Height/4),$plot.Width,[single]($i*$plot.Height/4))};$pen.Dispose()
 if($script:actualRoi){$a=$script:actualRoi;$pen=New-Object Drawing.Pen([Drawing.Color]::DeepSkyBlue,3);$g.DrawRectangle($pen,[single]($a.Left*$sx),[single]($a.Top*$sy),[single](($a.Right-$a.Left)*$sx),[single](($a.Bottom-$a.Top)*$sy));$pen.Dispose()}
 $pen=New-Object Drawing.Pen([Drawing.Color]::Cyan,1);$pen.DashStyle='Dash';$g.DrawRectangle($pen,[single]($roiLeft.Value*$sx),[single]($roiTop.Value*$sy),[single][Math]::Max(0,($roiRight.Value-$roiLeft.Value)*$sx),[single][Math]::Max(0,($roiBottom.Value-$roiTop.Value)*$sy));$pen.Dispose()
 if($script:lastRecord){$r=$script:lastRecord;$pen=New-Object Drawing.Pen([Drawing.Color]::Orange,2);$pen.DashStyle='Dash';$g.DrawLine($pen,[single]($r.C*$sx),0,[single]($r.C*$sx),$plot.Height);$pen.Dispose()
  $fresh=([DateTime]::Now-$script:lastRx).TotalSeconds -le 1.5;$brush=if($r.V -eq 1 -and $fresh){[Drawing.Brushes]::LimeGreen}else{[Drawing.Brushes]::Gray}
  $g.FillEllipse($brush,[single]($r.BX*$sx-7),[single]($r.BY*$sy-7),14,14);$g.DrawString(('球：({0}, {1})　实时目标T：{2}　有效：{3}' -f $r.BX,$r.BY,$r.T,$r.V),$form.Font,[Drawing.Brushes]::White,12,12)
 }else{$g.DrawString('等待钢球坐标，可先拖动预览蓝框',$form.Font,[Drawing.Brushes]::White,12,12)}
})
$timer=New-Object Windows.Forms.Timer;$timer.Interval=50
$timer.Add_Tick({
 try{if($script:serial -and $script:serial.IsOpen){
  $incoming=$script:serial.ReadExisting();if($incoming){$script:writer.Write($incoming);$script:buffer+=$incoming}
  while($script:buffer.Contains("`n")){$at=$script:buffer.IndexOf("`n");$line=$script:buffer.Substring(0,$at).Trim();$script:buffer=$script:buffer.Substring($at+1);$r=Convert-Record $line;if($r){Show-Record $r $line}elseif($line.StartsWith('VIS=1') -or $line.StartsWith('VIS=2') -or $line.StartsWith('VIS=3') -or $line.StartsWith('VIS=4')){$cameraAck.Text='检测到旧版固件，请烧入配套VIS=5新bit'}}
  if($script:buffer.Length -gt 8192){$script:buffer='';$status.Text='已丢弃过长或不完整的数据片段'}
  if($script:pending -and ([DateTime]::Now-$script:pending.Sent).TotalSeconds -gt 2){$ack.Text='控制参数：未确认，请检查串口RX连接；未自动重发';$script:pending=$null}
  if($script:followPending -and ([DateTime]::Now-$script:followPending.Sent).TotalSeconds -gt 2){$followAck.Text='运行设置未确认；请用板上K2停止或K4停机，未自动重发';$script:followPending=$null}
  if($script:cameraPending -and ([DateTime]::Now-$script:cameraPending.Sent).TotalSeconds -gt 5){$cameraAck.Text='摄像头/蓝框：尚未完成，请检查RX、摄像头与新帧；未自动重发';$script:cameraPending=$null}
  if($script:lastRecord -and ([DateTime]::Now-$script:lastRx).TotalSeconds -le 1.5){$status.Text='正在接收VIS=5｜K2按运行模式开始/停止'}else{$status.Text='未收到新版实时数据，请检查bit、串口及波特率'};$plot.Invalidate()
 }}catch{Disconnect-Port;$status.Text='串口错误：'+$_.Exception.Message}
})
$form.Add_FormClosed({$timer.Stop();Disconnect-Port;$timer.Dispose()});Refresh-Ports
if($SmokeTest){
 if($mode.SelectedIndex -ne 1 -or $damping.Value -ne 12 -or $runtime.Value -ne 30){throw '连续模式默认值错误'}
 if($limit.Maximum -ne 60 -or $gain.Minimum -ne 0){throw '大目标面板控件范围错误'}
 Show-Record (Convert-Record $sample) $sample
 $deadzone.Value=33;$script:pending=@{C=960}
 $capturedSample=$sample.Replace(' C=03C0 ',' C=01C2 ').Replace(' T=FB ',' T=00 ').Replace('CC=0001','CC=0002').Replace(' L=05 ',' L=3C ').Replace(' G=06 ',' G=02 ')
 Show-Record (Convert-Record $capturedSample) $capturedSample
 if($center.Value -ne 450 -or $deadzone.Value -ne 33 -or $script:pending -or $script:lastRecord.P -ne 4 -or $script:lastRecord.MT -ne 5){throw 'K3中心回读同步或保留未发送参数/运动状态失败'}
 $deadzone.Value=20;$gain.Value=2;$limit.Value=60
 # Exercise the real mouse handlers, including coordinate conversion and translation.
 Add-Type -ReferencedAssemblies System.Windows.Forms -TypeDefinition @'
using System.Reflection;
using System.Windows.Forms;
public static class UiMouseTest {
 public static void Send(Control c,string name,int x,int y) {
  var e=new MouseEventArgs(MouseButtons.Left,1,x,y,0);
  typeof(Control).GetMethod("On"+name,BindingFlags.Instance|BindingFlags.NonPublic).Invoke(c,new object[]{e});
 }
}
'@
 [UiMouseTest]::Send($plot,'MouseDown',100,100)
 [UiMouseTest]::Send($plot,'MouseMove',400,300)
 [UiMouseTest]::Send($plot,'MouseUp',400,300)
 if($roiLeft.Value -ne 240 -or $roiTop.Value -ne 240 -or $roiRight.Value -ne 960 -or $roiBottom.Value -ne 720){throw '鼠标画框测试失败'}
 $drawMode.Checked=$false
 [UiMouseTest]::Send($plot,'MouseDown',200,200)
 [UiMouseTest]::Send($plot,'MouseMove',300,250)
 [UiMouseTest]::Send($plot,'MouseUp',300,250)
 if($roiLeft.Value -ne 480 -or $roiTop.Value -ne 360 -or $roiRight.Value -ne 1200 -or $roiBottom.Value -ne 840){throw '鼠标平移测试失败'}
 $drawMode.Checked=$true;$roiLeft.Value=600;$roiTop.Value=300;$roiRight.Value=1600;$roiBottom.Value=900
 $cameraAck.Text='摄像头/蓝框：配置流程和新帧应用均已完成，序号 1';$ack.Text='控制参数：演示界面，未打开串口'
 for($j=0;$j -lt 240;$j++){$q=Convert-Record $capturedSample;$q.BX=450+[int](120*[Math]::Sin($j/22.0));$q.P=[int](8*[Math]::Cos($j/22.0));$q.AT=$q.P+1;Show-Record $q $capturedSample}
 $coastSample=$capturedSample.Replace(' V=1 ',' V=0 ').Replace(' Q=F ',' Q=0 ').Replace(' T=00 ',' T=0C ');Show-Record (Convert-Record $coastSample) $coastSample
 if($details.Text -notlike '*短丢保持*'){throw '短丢保持状态未显示'}
 $form.Opacity=0;$form.ShowInTaskbar=$false;$form.Show();[Windows.Forms.Application]::DoEvents()
 $bitmap=New-Object Drawing.Bitmap($form.Width,$form.Height);$bounds=New-Object Drawing.Rectangle(0,0,$form.Width,$form.Height);$form.DrawToBitmap($bitmap,$bounds)
 $bitmap.Save((Join-Path $PSScriptRoot 'panel_preview.png'),[Drawing.Imaging.ImageFormat]::Png);$bitmap.Dispose()
 $plotTabs.SelectedIndex=1;[Windows.Forms.Application]::DoEvents();$bitmap=New-Object Drawing.Bitmap($form.Width,$form.Height);$form.DrawToBitmap($bitmap,$bounds);$bitmap.Save((Join-Path $PSScriptRoot 'panel_curves_preview.png'),[Drawing.Imaging.ImageFormat]::Png);$bitmap.Dispose();$form.Close();Write-Output 'HOST_UI_SMOKE_PASS: VIS5 Chinese modes/damping/runtime, center readback, ROI draw/move and 240-point curves rendered; no serial opened';exit
}
$timer.Start();[void]$form.ShowDialog();$form.Dispose()
