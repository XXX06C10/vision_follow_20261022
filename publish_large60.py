"""Publish the verified large60 revision in the requested project, no hardware writes."""
from pathlib import Path
import hashlib,json,zipfile
root=Path(__file__).resolve().parent;look=Path('D:/AAAProject/FPGA/AAA_LookHere');base=root.as_posix()
read=lambda p:p.read_text(encoding='utf-8-sig')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads(read(root/'verification.json'));lineage=json.loads(read(root/'large60_lineage.json'))
assert r['status']=='SOFTWARE_BUILD_PASS_AWAITING_PHYSICAL_LARGE60_TEST'
assert r['motion_bounds']['target_counts']==[-60,60] and r['motion_bounds']['timeout_seconds']==5
bit=Path(r['delivery_bit']);digest=r['sha256'];assert sha(bit)==digest
archive=Path(lineage['archive']).as_posix();res=r['resources'];t=r['timing_ns']
links=f'''- [新bit（large60）]({bit.as_posix()})
- [启动配套中文面板]({base}/host/start_panel.cmd)
- [恢复扩大前的K3版本]({archive}/恢复说明.md)
'''
metrics=f"{res['slice']} Slice、{res['register']}寄存器、{res['IO']} IO、{res['DSP']} DSP、{res['ERAM']} ERAM；SWNS +{t['SWNS']:.3f}ns、HWNS +{t['HWNS']:.3f}ns，STNS/HTNS 0，STA覆盖{r['STA_coverage_percent']:.2f}%"
(root/'测试说明.md').write_text(f'''# K2大幅度目标：约5°空载测试

2026-10-22。本工程已原位更新为large60版本，VIS=4、P/V报文格式保留。**新bit已生成；上限扩大到60计数，实际幅度还需要在上位机应用新参数。**

{links}
## 参数变化与单位

| 项目 | 原值 | 本轮值 | 单位 |
|---|---|---|---|
| TARGET_LIMIT | 5 | 60 | 相对软件参考的编码器目标计数 |
| MOVE_BUDGET | 12 | 64 | 单次最多STEP脉冲 |
| POSITION_LIMIT（MC） | ±16 | ±64 | 正负方向累加的净指令脉冲数 |
| FEEDBACK_LIMIT（P） | ±22 | ±64 | 软件反馈编码器计数 |
| 动作超时 | 2 | 5 | 秒 |
| 到位带 / 无反馈限制 | ±2 / 3 | 保持 | 计数 / 连续无反馈脉冲 |

按你当前000拨码/16细分及4096计数/圈，60计数约5.27°，57计数约5.01°，从0到60约47脉冲。到位允许±2，实际最终位置在MT±2内就结束。这是电机/编码器侧的名义角度，不是机构杆角标定。

原方向准备20ms、高低各2.5ms、反馈等待50ms保持，每脉冲约75ms；约47脉冲需3.5秒，原2秒会误超时，所以核心和monitor覆盖参数都改为5秒。64脉冲的完整阶段约4.8秒，仍有时间边界。

## 先加载并设置

1. 保持电机空载、原供电/接线和000拨码；松开所有按键，把上面带`large60`的新bit按原成功方法加载到SRAM。器件必须PH1P35MDG324；本轮没有写Flash。
2. 关闭旧串口程序，启动本工程新版面板，连接原COM，115200/8N1。新版标题含“±60”，目标限幅最大60、控制衰减档最小0。
3. **将“目标限幅”设为60、“控制衰减档”设为2，点“应用视觉目标参数”，等待完整回读确认和有效新帧。** 若希望更接近5.0°，可用限幅57。默认仍是限幅5、衰减档7，未应用新参数时仍会小幅运动。衰减档越小，球偏移相同距离得到的T越大；0/1/2均可用，首次用2。
4. K1先关闭，球放在实际中心，识别有效且配置完成后，按一下K3再松开。确认中心C与球BX一致、K3记录成功、T=0。曝光3600、传感器增益143、蓝框调节保持。
5. 按一下K1并松开，等待使能、软件参考和就绪。K3不重置编码器原点，初次参考仍由K1使能后建立。

## 按这个顺序做四次

目标仍是**相对软件原点的绝对位置**，每次先看候选T，再按一次K2，等待到位后继续：

| 顺序 | 球位置 / 候选T | 按K2后的预期 |
|---|---|---|
| 1 | 右侧，T约+57..+60 | MT锁定正目标，P进入MT±2，DC加1 |
| 2 | K3标定的中心，T=0 | MT=0，P进入−2..+2，DC再加1 |
| 3 | 左侧，T约−57..−60 | MT锁定负目标，P进入MT±2，DC再加1 |
| 4 | 标定中心，T=0 | P回到−2..+2，再完成一次 |

正常应F=0、X1=0，SP≤64，MB最终0、OK=1，ME在±2内。典型约3～4.5秒完成，等到位再按下一次；仿真的1.28计数/脉冲模型正/负60分别46/48脉冲，仅是模型结果，不作为实板固定脉冲要求。

**本轮64脉冲预算不足以保证从+60直接跨到−60**：相距约120计数、约94脉冲，可能触发F=5，这是保留的预算保护。先按上面的正→0→负→0测试。若后续需要一次跨越两端，须另行设计更大的预算或分段轨迹，本轮没有擅自提高到94以上。

## 原有行为保持

K2长按只接受一次、忙时不排队，执行中不追随新球坐标。K1关闭或K4立即停止；丢球、坏帧、200ms无新结果、参数提交取消动作，不自动回零。恢复后需新的K2。K3只在有效、空闲、无故障时记录中心，不发步、不改P/MC/参考、不清故障；断电/重载后中心需重设。

若F非0，保存新日志，查明原因后重载；不要用K3清除保护。原日志出现过P76触发F7；本轮P保护为±64，P76仍越界。上位机日志在host/logs，反馈时附本轮新日志，可检查MT/P/ME、SP、MC、AC/DC、F和X1。

## 已完成的检查与恢复

tb_motion八类锁存故障、tb_full真实P参数→候选→K2→AB反馈的±60/0及取消/停止路径、tb_large物理时序比例和1.28反馈模型、K3专项、完整720p检测、相机/I2C六组仿真均通过。上位机通过launcher的SelfTest/SmokeTest；原P报文逐字节不变，新G0/L60为 `P000103C00014003CEA` 加LF，仍20字节。

完整TD构建、PH1P35MDG324、全部实际IO及双口STEP/DIR/EN、时钟/SDC不变与非负时序通过：{metrics}。源码改动和新bit一致性检查见[verification.json]({base}/verification.json)。**实板大目标动作待用户验证，未宣称实板已通过。**

新bit SHA256：`{digest}`。

扩大前K3版的源码、bit、面板、日志和哈希已保存在[归档zip]({archive}.zip)，原bit SHA256 `31c95d3abc5c0cf1bca8078fc873818f8ef735f5e61e4b90c0865670c1e26efc`。不要用旧面板配置大目标。
''',encoding='utf-8')
(root/'README.md').write_text(f'''# K3中心 + K2大目标60计数

2026-10-22按用户要求在本工程原位更新。新目标上限60、单次64脉冲、MC±64净脉冲/P±64反馈计数，超时5秒；到位±2、无反馈3和原K1/K2/K3/K4行为保持。P/V格式、VIS=4报告342字节、所有引脚与时钟约束不变。

{links}
- [本轮设置、测试与预期]({base}/测试说明.md)
- [改动与协议说明]({base}/实现与协议.md)

**要转大：在新版面板应用限幅60、衰减档2。默认5/7保持。** 目标是绝对软件位置，按0→+60→0→−60→0测试；直接两端跨越可能超过64脉冲预算。

六组仿真和完整布线检查通过：{metrics}。新bit SHA256 `{digest}`；实板large60待验证，没有自动下载或写Flash。扩大前版本见large60_lineage.json及恢复入口。

重建：`./build.ps1`；仿真：`./run_sim.ps1 tb_motion`、`tb_full`、`tb_large`、`tb_center`、`tb_camera`、`tb_detector`；完整检查：`python verify.py`。中文面板自检：`host/start_panel.cmd -SelfTest` 或 `-SmokeTest`。正常启动无参数。发布：先verify，再`python publish_large60.py`。不要运行旧的一次性迁移或旧版本发布脚本。
''',encoding='utf-8')
(root/'实现与协议.md').write_text(f'''# large60修改和验证说明

本次只改变运动核心的目标/预算/MC/P界限与超时、monitor超时覆盖、FPGA/面板参数合法范围及配套显示/验证。核心运行逻辑逐字节按归档比较，除参数常量外不变；vision_shadow_target未改，位宽仍是有符号8位，60可表示。K3逻辑、相机193份源文件、P/V组包和XOR、报告布局、ADC/SDC、构建脚本均复核保持。

实际旧vision_config_rx有 `gain低7位>=4且<=12、limit<=5` 硬校验。本轮必须把它同步为低7位0..12、limit0..60，否则主机发G0/L60也会被拒绝。这里只改合法范围，不改报文和提交方式。默认G7/L5保持。

运动核心 TARGET_LIMIT60 / MOVE_BUDGET64 / POSITION_LIMIT64 / FEEDBACK_LIMIT64；DEADBAND2和NO_RESPONSE_LIMIT3不变。方向准备、脉冲高/低、反馈等待均保持20/2.5/2.5/50ms，每脉冲约75ms。TIMEOUT_CYCLES在核心和monitor均由100000000改为250000000，在50MHz下为5秒，以容纳约47脉冲；没有放宽FPGA时钟周期。

P协议仍为ASCII `P + seq16 + center16 + deadzone16 + gain8 + limit8 + xor8 + LF`，20字节；gain的bit7仍反向，低7位0..12。V报文32字节、VIS=4报告342字节及9位字符索引保留；T为有符号8位，±60分别3C/C4；MT/P/ME/MC为有符号32位。SP2位十六进制可覆盖64，面板改为显示“/64”。

MC为按DIR正负累加的净指令脉冲数；P和T为编码器计数，两者不是同一单位。目标相对软件原点，是绝对位置；从0到60约47脉冲，但两端+60到−60约94脉冲，本轮不扩大单次64预算。这个超预算转移会故意锁存F5；需要通过0或另设计轨迹。

故障码仍为1无反馈3次、2反向反馈2次、3非法AB、4动作5秒超时、5单次64脉冲不足、6净指令±64边界、7反馈±64越界、8非法状态。锁存、取消、K4立即停止、K1保持和K3视觉中心含义保持，不自动回零/重启。中心断电/重载后仍需重设。

验证：tb_motion保留八类故障并测实际MC64/P65界限；tb_full先保留原小目标回归，再用真实串口P设置G0/1/2及反向、L60，经过原候选映射和真实AB解码器完成+60/0/−60/0，之后继续全部取消/停止/错误报文测试。tb_large把全部生产运动阶段和5秒超时统一缩小500倍，模型32/25计数/脉冲，到位分别P58/P2/P−58/P−2；动作均超过旧2秒但小于新5秒，正/负46/48脉冲。另测直接跨端精确64脉冲锁存F5。其余K3、720p/ROI与相机/I2C三组回归通过。

主机自检检查旧标准P报文不变、新G0/L60精确字符串和XOR、G0/1/2及反向范围、越界拒绝、±60解析和真实界面渲染。start_panel.cmd新增参数透传，可直接运行-SelfTest/-SmokeTest；无参数启动行为保持。

完整构建：{metrics}。器件PH1P35MDG324；双口STEP C18/V2、DIR D15/V1、EN E12/U2、E1 E16/E17及全部原IO保持。参考时钟周期、异步组、10ns相机邮箱约束和已分类时钟布线提示保持。报告与源码一致性见verification.json，恢复参考见large60_lineage.json。

新bit SHA256 `{digest}`。这是软件/仿真/布线通过，实板large60仍待验证；未新增连续跟随、机械标定或Flash写入。
''',encoding='utf-8')
entry=f'''# K2约5°动作：large60

2026-10-22，新bit和配套面板已更新。目标限幅最大60计数、单次64脉冲、MC/P各±64（单位不同），超时5秒；到位±2与无反馈3保持。K3定中心功能保留。

{links}
- [本轮步骤]({base}/测试说明.md)

**先应用限幅60、衰减档2**（默认仍5/7），再K3定中心，K1使能，按0→+60→0→−60→0逐次K2测试。直接正端到负端可能超64脉冲，按预算保护停止。

六组仿真、面板SelfTest/SmokeTest、完整器件/IO/时序检查通过；{metrics}。实板大目标待测。SHA256 `{digest}`。
'''
(look/'下一步_K2约5度动作.md').write_text(entry,encoding='utf-8')
def prepend(path,heading):
 s=read(path)
 if heading in s:return
 n=s.find('\n')+1;body='2026-10-22：用户明确要求扩目标60/预算64/MC和P64。已归档原K3版，原位修改并补齐真实RX边界和5秒超时，协议/引脚/时钟保持；六组仿真、主机及完整布线检查通过，large60实板待验证。[当前入口](D:/AAAProject/FPGA/AAA_LookHere/下一步_K2约5度动作.md)。SHA256 `'+digest+'`。'
 path.write_text(s[:n]+'\n'+heading+'\n\n'+body+'\n\n以下保留历史记录。\n'+s[n:],encoding='utf-8')
prepend(look/'实施计划_20261019/目标验收状态.md','## 最新结果：K2目标60计数版本已交付，待实板验证')
prepend(root.parent/'D36A_STEPPER_调试进度总结.md','## 2026-10-22 最新：K2目标上限60、64脉冲预算')
contract=root/'baseline_contract.json';data=json.loads(read(contract));data.update(status=r['status'],date='2026-10-22',delivered_bit_sha256=digest,physical_test='PENDING_USER_LARGE60_BOARD_TEST',next_milestone='unloaded0/+60/0/-60/0 K2 moves with calibrated K3 center and explicitly applied large-target parameters')
contract.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
files=[bit.relative_to(root).as_posix(),'host/start_panel.cmd','host/vision_panel_cn.ps1','host/vision_panel.ps1','host/panel_preview.png','README.md','测试说明.md','实现与协议.md','verification.json','large60_lineage.json','frame_layout.json','decode_serial.py','build_sources.json','baseline_contract.json']
files+=['sim/results/'+name+'.log' for name in ('tb_motion','tb_full','tb_large','tb_center','tb_camera','tb_detector')]
manifest={'date':'2026-10-22','status':r['status'],'variant':'large60','report_version':4,'bit_sha256':digest,'hardware_programmed':False,'files':{rel:sha(root/rel) for rel in files}}
(root/'delivery_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
bundle=look/'K2约5度_large60_20261022.zip'
with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
 for rel in files+['delivery_manifest.json']:z.write(root/rel,rel)
with zipfile.ZipFile(bundle) as z:assert z.testzip() is None
print('Published large60 bit/host/test/recovery and paired hashes. Physical test pending.')
