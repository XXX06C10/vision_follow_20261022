"""Publish local, verified VIS=3 delivery links. Does not program hardware."""
from pathlib import Path
import hashlib,json,zipfile

root=Path(__file__).resolve().parent
look=Path('D:/AAAProject/FPGA/AAA_LookHere')
read=lambda p:p.read_text(encoding='utf-8-sig')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report=json.loads(read(root/'verification.json'))
assert report['status']=='SOFTWARE_BUILD_PASS_AWAITING_PHYSICAL_K2_SINGLE_MOVE_TEST'
assert report['report_version']==3 and report['wire_UART_bytes']==329
bit=Path(report['delivery_bit']);digest=report['sha256']
assert sha(bit)==digest
assert report['timing_ns']['SWNS']>=0 and report['timing_ns']['HWNS']>=0
assert report['automatic_motion']=={'K2_single_visual_target':True,'continuous_follow':False,'automatic_resume':False}
base=root.as_posix()
entry=f'''# 本轮：空载、K2触发一次视觉动作

2026-10-21。**新bit和配套中文上位机已更新，VIS=3；这版按K2会驱动电机。** 原来你确认效果不错的视觉版本已先归档，旧工程保留。

- [新bit]({bit.as_posix()})
- [启动配套中文上位机]({base}/host/start_panel.cmd)
- [详细接线前提、四次测试和预期]({base}/测试说明.md)
- [恢复原视觉版](D:/AAAProject/FPGA/AAA_LookHere/成功版本归档/20261021_视觉亮度蓝框_用户确认/恢复说明.md)

先空载，保持原接线和000拨码，松开所有键加载新bit，关闭旧串口程序，打开本轮中文界面连接原COM。K1点按保持使能，等待就绪；将球依次放右侧、中心、左侧、中心，每次看清候选T后按一下K2，等到位再继续。侧向选±3～±5，中心为0，允许误差±2；小转动看串口位置与完成次数即可。K4立即停止并清除使能。

动作期间锁定MT，不追随新的实时T；长按只接受一次、忙时不排队、K3无动作。丢球/坏帧/超过200ms无新结果/应用参数会取消，保留K1保持，不自动回零；恢复后仍须松开再按K2。曝光3600、增益143、蓝框调节保留。故障锁存，重新加载才清除。

四组仿真、真实AB反馈模型、八类故障保护、329字节串口和中文界面验证通过；完整布线PH1P35MDG324，17407 Slice、19 DSP、45 ERAM，SWNS +0.103ns、HWNS +0.020ns，总负裕量0。双口STEP/DIR/EN、编码器与官方相机引脚通过核对，没有放宽时钟要求。

本轮实板动作仍待验证，没有自动下载或写Flash。完成四次后保留本工程host/logs里的日志，用于确认AC/DC、MT/P/ME、F和X1；通过后再推进连续跟随与机构回中。

SHA256：`{digest}`。源码/归档/构建/验证见[工程说明]({base}/README.md)和[验证报告]({base}/verification.json)。
'''
(look/'下一步_视觉单次动作.md').write_text(entry,encoding='utf-8')

def prepend_section(path,heading,body):
    text=read(path)
    if heading in text:return
    first=text.find('\n')+1
    path.write_text(text[:first]+'\n'+heading+'\n\n'+body+'\n\n以下保留此前阶段记录。\n'+text[first:],encoding='utf-8')

body=('2026-10-21：用户确认此前中文视觉亮度/蓝框“效果不错”，已保存其源码、bit、上位机和哈希，'
      '确认范围不扩大为精密运动验收。独立新工程从已通过position_core改造K2单次±5目标/±2到位，'
      '保留K1/K4及原故障保护；丢球/过期/参数提交取消且不自动回零。VIS=3配套中文界面和新bit已交付，'
      '四组仿真、全部故障模型、完整布线/引脚/非负时序通过。**本轮空载实板动作待验证，连续跟随未启用。** '
      '[当前测试入口](D:/AAAProject/FPGA/AAA_LookHere/下一步_视觉单次动作.md)。新bit SHA256 `'+digest+'`。')
prepend_section(look/'实施计划_20261019/目标验收状态.md','## 最新结果：VIS=3单次视觉动作已交付，待空载实板验证',body)
prepend_section(root.parent/'D36A_STEPPER_调试进度总结.md','## 2026-10-21 最新：空载K2单次视觉动作固件',body)

contract=root/'baseline_contract.json';data=json.loads(read(contract))
data.update(status=report['status'],date='2026-10-21',delivered_bit_sha256=digest,report_version=3,
            no_new_bit_delivered_this_turn=False,camera_defaults=report['camera_defaults'],
            next_milestone='physical unloaded K2 right/center/left/center finite visual moves; then assess continuous following',
            physical_test='PENDING_USER_BOARD_TEST')
contract.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')

# Pair the bit with exactly these host/doc/report bytes. Original archives stay immutable.
files=['output/'+root.name+'.bit','host/start_panel.cmd','host/vision_panel_cn.ps1','host/vision_panel.ps1',
       'host/panel_preview.png','README.md','测试说明.md','实现与协议.md','verification.json','frame_layout.json',
       'decode_serial.py','motion_lineage.json','build_sources.json','baseline_contract.json',
       'sim/results/tb_motion.log','sim/results/tb_full.log','sim/results/tb_detector.log','sim/results/tb_camera.log']
manifest={'date':'2026-10-21','status':report['status'],'report_version':3,'bit_sha256':digest,
          'files':{rel:sha(root/rel) for rel in files},'hardware_programmed':False}
(root/'delivery_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
bundle=look/'视觉单次动作_VIS3_20261021.zip'
with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
    for rel in files+['delivery_manifest.json']:z.write(root/rel,rel)
with zipfile.ZipFile(bundle) as z:assert z.testzip() is None
print('Published local VIS=3 delivery, test entry and paired bit/GUI bundle. Physical test pending.')
