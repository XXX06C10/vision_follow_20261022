from pathlib import Path
root=Path(__file__).resolve().parent
p=root/'host/vision_panel_cn.ps1';text=p.read_text(encoding='utf-8-sig')
text=text.replace("'CB:1')", "'CB:1','MT:8','ME:8','MS:1','MB:1','OK:1','RD:1','SP:2','MC:8','CR:1','RJ:1','AC:4','DC:4')")
text=text.replace('VIS=2','VIS=3').replace("@('V','EN','Z','B','CB')", "@('V','EN','Z','B','CB','MB','OK','RD')")
text=text.replace("@('E1','E2','P')", "@('E1','E2','P','MT','ME','MC')")
text=text.replace('BX=04B0','BX=01C2').replace('G=07 L=05 SEQ=1234','G=06 L=05 SEQ=1234')
text=text.replace("RA=0001 CB=0'", "RA=0001 CB=0 MT=00000005 ME=00000001 MS=2 MB=0 OK=1 RD=1 SP=03 MC=00000003 CR=0 RJ=0 AC=0002 DC=0001'")
text=text.replace("$r.AE -ne 3600)", "$r.AE -ne 3600 -or $r.MT -ne 5 -or $r.ME -ne 1 -or $r.MC -ne 3)")
pos=text.index('$sample=')
text=text[:pos]+'''function Cancel-Text([int]$Code){
 $names=@('无','K1关闭使能','K4停止','识别失效/坏帧','超过200ms无新结果','参数改变/等待配置','故障停止')
 if($Code -ge 0 -and $Code -lt $names.Count){return $names[$Code]};return ('未知原因 '+$Code)
}
function Reject-Text([int]$Code){
 $names=@('无','尚未使能','尚未就绪','识别无效','视觉结果过期','等待配置后的有效新帧','正在动作，不排队','目标超过±5','请先松开K2再按','故障锁存')
 if($Code -ge 0 -and $Code -lt $names.Count){return $names[$Code]};return ('未知原因 '+$Code)
}
function Motion-Text($R){
 if($R.F -ne 0){return '故障锁存'};if($R.MB -eq 1){return '执行中'};if($R.EN -eq 0){return '未使能'}
 if($R.Z -eq 0){return '建立软件参考'};if($R.OK -eq 1){return '本次已到位'};if($R.RD -eq 1){return '就绪，可松按K2'}
 return '等待有效识别/配置'
}
'''+text[pos:]
text=text.replace("PH1P35 钢球视觉调试｜曝光·增益·蓝框", "PH1P35 视觉单次动作｜K1使能·K2执行·K4停止")
text=text.replace('观察模式：K1保持使能，K4停止，电机不自动转动。蓝框拖动只预览，点击“应用”才更新HDMI画面。',
 '空载单次动作：K1保持使能，K2执行一次，K4停止。丢球或应用参数会取消动作，不自动回零。')
text=text.replace('候选控制参数（尚不驱动电机）','视觉目标参数（K2按下时采样）').replace('应用候选控制参数','应用视觉目标参数')
text=text.replace('$details=Label-At \'等待新版 VIS=3 串口数据……\' 360 709 800 73', "$details=Label-At '等待新版 VIS=3 串口数据……' 360 709 800 105")
text=text.replace('$raw.SetBounds(360,790,800,65)','$raw.SetBounds(360,820,800,35)')
start=text.index(' $details.Text=');end=text.index('\n $script:latestLines.Enqueue',start)
text=text[:start]+''' $details.Text=('实时视觉目标 T：{0}　球：({1}, {2})　有效：{3}　置信度：{4}/15　帧龄：{5}ms' -f $R.T,$R.BX,$R.BY,$R.V,$R.Q,$R.AGE)+"`r`n"+
  ('本次锁定目标：{0}　实际位置：{1}　误差：{2}　脉冲：{3}/12' -f $R.MT,$R.P,$R.ME,$R.SP)+"`r`n"+
  ('动作状态：{0}　可接收K2：{1}　接受：{2}　完成：{3}　故障：{4}' -f (Motion-Text $R),$R.RD,$R.AC,$R.DC,$R.F)+"`r`n"+
  ('上次停止：{0}　上次拒绝：{1}' -f (Cancel-Text $R.CR),(Reject-Text $R.RJ))+"`r`n"+
  ('曝光：{0}　传感器增益：{1}　E1：{2}　AB错误：{3}　配置序号：{4}　接收错误：{5}' -f $R.AE,$R.AG,$R.E1,$R.X1,$R.CS,$R.RXE)
'''+text[end:]
text=text.replace('候选目标：{2}', '实时目标T：{2}').replace('正在接收新版数据｜电机不自动转动','正在接收VIS=3｜空载，K2单次动作')
text=text.replace("elseif($line.StartsWith('VIS=1'))", "elseif($line.StartsWith('VIS=1') -or $line.StartsWith('VIS=2'))")
text=text.replace('检测到旧版VIS=1，请烧入配套新版bit后再调曝光与蓝框', '检测到旧版观察固件，请烧入配套VIS=3新bit后再测试动作')
text=text.replace('Chinese UI protocol, signed fields, camera checksum/bounds, ROI scaling and completion ack', 'VIS3 Chinese motion metrics/signed decode, unchanged P/V packets, ROI scaling and completion ack')
p.write_text(text,encoding='utf-8-sig')
p=root/'decode_serial.py';text=p.read_text(encoding='utf-8').replace('VIS=2','VIS=3').replace("('2' if kind", "('3' if kind")
text=text.replace("['V','EN','Z','B','CB']", "['V','EN','Z','B','CB','MB','OK','RD']")
text=text.replace("('T',8)]", "('T',8),('MT',32),('ME',32),('MC',32)]")
p.write_text(text,encoding='utf-8')
print('Updated matching Chinese VIS=3 single-move panel and capture decoder')
