"""Publish only a verified follow bit and its matching Chinese host; no board writes."""
from pathlib import Path
import hashlib,json,zipfile
root=Path(__file__).resolve().parent;look=Path('D:/AAAProject/FPGA/AAA_LookHere');base=root.as_posix()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:p.read_text(encoding='utf-8-sig')
r=json.loads(read(root/'verification.json'));lineage=json.loads(read(root/'follow_lineage.json'))
assert r['status']=='SOFTWARE_BUILD_PASS_AWAITING_PHYSICAL_CONTINUOUS_CONTROL_TEST'
bit=Path(r['delivery_bit']);assert sha(bit)==r['sha256'];digest=r['sha256']
archive=Path(lineage['archive']).as_posix();res=r['resources'];t=r['timing_ns']
metrics=f"{res['slice']} Slice、{res['register']}寄存器、{res['IO']} IO、{res['DSP']} DSP、{res['ERAM']} ERAM；SWNS +{t['SWNS']:.3f}ns、HWNS +{t['HWNS']:.3f}ns，STNS/HTNS 0，STA覆盖{r['STA_coverage_percent']:.2f}%"
links=f'''- [新bit（VIS=5，连续控制）]({bit.as_posix()})
- [配套中文上位机]({base}/host/start_panel.cmd)
- [测试步骤]({base}/测试说明.md)
- [恢复此前large60版]({archive}/恢复说明.md)
'''
direction='''用户确认：K1使能后按K2，电机向下、球向右，当时球在中心右侧。**这只能确定机械关系“向下→球向右”**。T/MT为绝对软件目标，DIR由MT−P决定，不能把MT正负直接当成运动方向。

用户记不清P的变化，并明确选择“先按日志配置”。163418日志存在P约−5→−38、BX约768→826的趋势，因此本版默认不反向：球偏右→正P参考倾角，假设正P倾角使球趋向左；P=参考E1−当前E1。该关系尚非独立动态标定，保留面板“反转候选目标方向”（P报文gain bit7）。如实物初始纠偏使球更远离中心，K4停止后修改反向配置，再重新K1/K2。'''
steps=f'''# VIS5：从K2单次动作推进到连续钢球控制

2026-10-22。新独立工程`vision_follow_20261022`；此前large60工程、bit、中文面板、日志和源码已归档且保持哈希不变。**这是新bit，需要配套VIS5上位机。** 已完成软件、模型与完整布线验证，实板纠偏方向和参数稳定性待本轮日志确认。

{links}
## 这次可以做什么

| 模式 | 板上K2的作用 | 控制方式 |
|---|---|---|
| 0 单次视觉动作 | 每次按下锁定一次T | 保留原绝对位置执行，到位±2计数；上限可设60 |
| 1 连续PD（默认） | 第一次开始，再按停止 | 球横向偏差与速度阻尼实时形成倾角参考，100ms缓变，编码器跟踪；到位带±1计数 |
| 2 回软件参考位 | 每次按下执行目标0 | 明确按K2才返回首次建立的软件参考，到位±2；不代表机械寻零 |

K1只切换使能，不启动动作。K3在会话停止、识别有效时确认实际球中心。连续模式必须先有一次K3成功记录（CZ=1）。K4直接停止并清除使能保持，松开不会恢复。**上位机只设置参数和显示，不通过串口启动动作。**

## 第一次按这个顺序操作

1. 松开所有按键，按原成功方法加载上面的新bit到SRAM，确认器件仍为PH1P35MDG324。沿用全部原接线、供电、000拨码与编码器A/B；不需要补接PWM/Z。
2. 关闭旧串口软件，运行配套`host/start_panel.cmd`，选择原COM、115200/8N1。应显示VIS=5；默认曝光3600、传感器增益143，蓝框调节与鼠标画框/平移保留。
3. **先把机构摆到你希望的软件参考/近似水平位置，再第一次按K1。** 参考只在首次使能稳定后建立一次，之后K1/K4/K3都不重新归零。没有水平传感器，本版不能自动知道机构真实水平。
4. 球放到你认定的实际中心，保持识别框有效、配置完成；按K3再松开，面板确认中心C≈球BX、CZ=1。K3不动电机、不清故障。可在K1关闭时先做这一步。
5. 第一次采用默认：模式1、衰减档G4、限幅L12计数、中心死区20像素、阻尼12、缓变2计数/100ms、会话30秒。新bit已默认这些值；如修改，分别点“应用视觉目标参数”和“应用运行模式/阻尼/时限”，等完整回读。修改会停止已有会话，需重新按K2。
6. K1使能就绪后，球先靠近中心，按K2一次并松开。看AR=1，T为实时PD候选、AT为缓变参考、MT为执行器当前目标，P跟随。球偏右/左时应逐步趋向实际中心。**本轮先用12计数约1.05°参考倾角，不要一开始把连续限幅调到60。**
7. 再按K2应AR=0、停止发步并保持使能。运行满30秒也会停止（SR=8）；球/画面恢复不会自动启动。K4应立即EN=0并清除AR。再启动需K1、K2按实际状态重新操作。

## 本轮方向依据

{direction}

## 看哪些现象判断效果

上位机“误差/电机曲线”同时画球横向误差BX−C、实际P、缓变AT、执行MT，保留最近240条报告（通常约60秒）。正误差表示球在中心右侧。AC/DC在连续模式是**执行器发步段**接受/完成数，不等于按K2的次数；MB=0也可能AR=1，这是等待下一次纠偏。TS累计实际STEP上升沿，在启动连续会话时清零。

有效结果应F=0、X1=0、帧龄<200ms，P围绕有限倾角活动，球偏差逐渐受控；不能要求球绝对静止或P永远0。参数/机构摩擦、死区、量化和原75ms单脉冲阶段会影响效果。仿真模型末段仍有约±30～47像素偏差及量化摆动，未据此宣称真实机构精密居中。

连接后原始日志自动保存在`host/logs/`。测试完在工程目录运行`python analyze_follow.py`，自动分析最新日志；或`python analyze_follow.py "完整日志路径"`。输出同名`.analysis.json`，按AR会话给出偏差RMS、平均绝对值、峰值、死区内占比、末40条报告及停止原因。时长按250ms报告周期估算，掉行会影响时间；不把像素统计换算成未标定的毫米/机械角度。也可直接把日志发来继续调整。

## 调整规则

先确认纠偏方向，再调强弱。G越大控制越弱；L限制目标倾角，KD抑制滚动速度，SL限制参考变化。连续模式默认G4/L12/KD12/SL2，首次保持这些值。若反复来回摆动，可停机后先把KD从12增至14，仍摆动再将G从4增至5；每次只改一项并保留日志。若纠偏过弱且方向正确，可再逐步减小G或增加L。上限60约5.27°只指电机编码器侧；没有机构比例标定，不能当杆倾角。

运行时限0表示持续运行，但仍有每个活动发步段的64脉冲/5秒保护、MC±64净脉冲、P±64计数等全部保护。默认30秒便于先收集一次可比较的数据。

## 停止、取消与故障

丢球、坏帧、200ms无新结果、P/V/S有效提交、相机配置忙、K1关闭或K4按下会取消会话/发步，不自动回参考、不自动续跑。识别恢复后仍需新的K2。K2停止或30秒结束保留EN保持；K1/K4或故障关闭使能。

| F | 含义 |
|---|---|
| 1 | 连续3次STEP未得到编码器响应 |
| 2 | 连续2次反馈方向与STEP方向相反 |
| 3 | 编码器A/B非法跳变 |
| 4 | 当前活动发步段超过5秒 |
| 5 | 当前活动发步段超过64脉冲预算 |
| 6 | MC净指令位置将超±64脉冲 |
| 7 | P反馈位置超±64计数 |
| 8 | 状态或目标保护 |

故障锁存，K3和改参数都不能清除，查明原因后重新加载。原大目标两端直接跨越仍可能超过64脉冲；保留该保护。停止使能后移动机构也可能使保留参考越界，下次使能时触发F7；本版不偷偷重置原点绕过。

## 已验证与恢复

八组仿真：原八类故障与停止、±60/0真实AB模型、K3专项、相机实际I2C配置/ROI跨域、720p检测、VIS5真实429字节UART波形、连续会话及Q4慢速阻尼、动态改目标仍触发5秒/64脉冲、三种明确假设的滚动模型及外部扰动。中文启动器SelfTest/SmokeTest通过，曲线和画框有渲染检查；P20字节/V32字节格式与XOR保持。完整构建：{metrics}，全部路由IO/双口镜像、官方摄像头引脚、时钟约束、源码/bit一致性均通过。

新bit SHA256：`{digest}`。

恢复此前版本见[恢复说明]({archive}/恢复说明.md)，父bit SHA256 `{lineage['parent_bit_sha256']}`。本轮未自动下载、未写Flash，也未把模型结果当成实板验收。需要实板完成的是方向、水平参考、参数整定及比赛指标；持续运行软件已交付。
'''
(root/'测试说明.md').write_text(steps,encoding='utf-8')
(root/'README.md').write_text(f'''# 单轴视觉闭环：手动 / 连续PD / 软件参考返回

2026-10-22。沿已实板运行成功的运动脚本和核心改造，独立于large60父工程。

{links}
默认先K3确认实际中心→K1使能→K2连续开始/停止；K4立即停止。G4/L12、KD12、2计数/100ms缓变、30秒会话；曝光3600/增益143和蓝框调节保留。面板可改为单次±60动作或K2确认返回软件参考。没有自动续跑/自动寻零。

八套仿真、中文面板自检/渲染、完整TD布线通过：{metrics}。**实物连续闭环稳定性待日志确认**，模型有残余摆动，不能当作精密居中验收。当前bit SHA256 `{digest}`。

源码：`rtl/vision_follow_control.v`为外环，`rtl/vision_servo_core.v`为守护执行器，`servo_adaptation.diff`记录相对冻结父核心的完整变更；相机/IP未改。接口见[实现与协议.md]({base}/实现与协议.md)，检测统计用`python analyze_follow.py`。

重建：`./build.ps1`；八套仿真：`./run_sim.ps1 tb_motion`、`tb_large`、`tb_full`、`tb_center`、`tb_camera`、`tb_detector`、`tb_follow`、`tb_plant`；完整核对：`python verify.py`；发布：`python publish_follow_delivery.py`。面板自检：`host/start_panel.cmd -SelfTest`和`-SmokeTest`。历史一次性迁移/发布脚本保留溯源，不用于当前重建。
''',encoding='utf-8')
(root/'实现与协议.md').write_text(f'''# 连续控制实现、接口和证据范围

## 从父版本保留的部分

large60父工程及其bit/GUI/日志已冻结到`{archive}`，哈希全检、zip CRC均通过。原`vision_move_core.v`字节不变，`vision_servo_core.v`从该核心逐项适配外部请求、取消和CHECK阶段改目标，完整差异见servo_adaptation.diff；verify.py从父核心重建期望版本逐字核对。K1保持、编码器计数、相机/ISP/HDMI/DDR、ROI等原源码保持；相机/IP逐文件核对{r['camera_IP_files_byte_identical']}份。

每次STEP仍20ms方向准备、高/低各2.5ms、50ms反馈等待，单脉冲约75ms。新目标只在CHECK阶段接受，脉冲高电平不改DIR；活动段内改变目标不重置SP或5秒计时。连续内环到位带改为±1计数，手动/返回仍±2，这项差异用于减小残留参考倾角，不宣称绝对静止。

## 外环

仅K2边沿启动/停止会话，长按不重发。先K3中心确认，K1及原使能等待完成。帧同步后使用同一份X/有效性。位移delta裁剪±256像素/帧，速度VE用有符号Q4，更新`VE = (3*VE + delta*16)/4`，保留慢速分量；`prediction = X-C + VE*KD/16`。按prediction符号、死区、G移位、L夹取，最后应用gain bit7反向。没有积分。每100ms且有新帧时，AT向候选T按SL限速；反馈执行器将P跟到AT。首次启动AT从当前P开始，避免突然跳目标。

三种模型使用`v=0.975*v-coupling*P, error+=v`，coupling0.04/0.06/0.08，非实测；相机约33ms，执行器保持生产时序比例，编码器32/25计数/脉冲。约11.5秒加扰动，28秒末要求|error|≤80像素且|v|≤1像素/帧，期间F=0和误差<800。结果见verification.json和sim/results/plant_trace_*.csv。该要求只验证当前模型的有界响应，残余偏差/摆动仍存在，实物需整定。初版整数速度丢失慢速阻尼，已改Q4并加正负/衰减/反向专项。

## 串口兼容性

115200/8N1，ASCII十六进制，LF末尾原子提交，XOR校验，FPGA自身校验范围，整包超时100ms。P和V格式/校验函数与父版逐字核对不变：

- `P + seq16/center16/deadzone16/gain8/limit8/xor8 + LF`，20字节。G低7位0..12，bit7反向，L0..60。
- `V + seq16/exposure16/sensor_gain16/left16/top16/right16/bottom16/xor8 + LF`，32字节。曝光2..3600、增益16..143，ROI传感器坐标1280×720；面板以1920×1080坐标缩放。
- 新增`S + seq16/mode8/damping8/slew8/runtime16/xor8 + LF`，18字节，7个payload字节异或。mode0..2、KD0..16、SL1..8、runtime0..300秒（0不限）。例默认值seq1：`S0001010C02001E10`加LF。

所有有效配置提交取消运行，只配置，不启动电机；必须新的K2。S回读DS/MD/KD/SL/RU全部匹配才确认，P回读SEQ/C/DZ/G/L，V等待CS/CA/RA一致和CB=0；不自动重发。有效P/V/S串口提交优先于同拍K3中心。

## VIS=5回读

429字节含CRLF，9位字符索引，完整串口波形已验证。VIS4原字段顺序保留，追加MD/AR/KD/SL/**RU**/DS/CZ/VE/AT/SR/AJ/EL/TS。`RT`始终是蓝框上边界，`RU`才是会话时限，避免重名字段。

T为PD或手动实时候选，AT为缓变参考，MT为当前脉冲执行目标，P为编码器相对参考位置，ME=MT−P。VE为Q4像素/帧，面板除16显示；T/AT为signed8，VE signed16，P/MT/ME/MC signed32。MD0单次、1连续、2参考返回；AR会话运行；CZ成功K3确认；DS为S序号；EL会话秒；TS实际STEP上升沿数，连续启动清零。AC/DC在自动模式数发步段。

SR与CR原因0无/1K1/2K4/3无效/4过期/5配置/6故障/7K2停止/8时限。AJ/RJ 0无、1未使能、2未就绪、3无效、4过期、5配置、6忙、7越界、8需松按、9锁存故障、A先K3确认。历史原因保持到下一次事件，不等同当前状态。

## 方向、参考和构建

{direction}

K3只改视觉中心，不改P原点或物理水平。原点首次K1稳定后一次捕获，开关K1/停机保留，重新下载才清。软件参考返回必须明确选mode2并按K2，在有效视觉下完成，不是传感器寻零。

器件PH1P35MDG324，STEP C18/V2、DIR D15/V1、EN E12/U2，E1 A/B E16/E17。实际路由全部IO及镜像同源、官方参考引脚/时钟/SDC核对通过，没有放宽时钟或新增假路径。{metrics}。TD参考摄像头/IP时钟走线提示在verification.json单独列明，非屏蔽；报告保留既有IP未驱动诊断，新增视觉/控制路径无未驱动信号。

交付bit `{digest}`；状态为软件完成、实板连续控制待验收。未自动烧录、未写Flash；掉电中心与参数不保存。恢复父版bit前，关使能并用归档配套面板。
''',encoding='utf-8')
(look/'方向实测记录_20261022.md').write_text('# 方向实测记录\n\n2026-10-22。\n\n'+direction+'\n',encoding='utf-8')
(look/'下一步_连续视觉闭环.md').write_text(f'''# 当前交付：连续视觉闭环VIS5

{links}
已完成：连续PD、K2开始/停止、K3中心、单次/参考返回模式、中文曲线/调参/自动日志统计、全部原保护。父版已冻结。默认G4/L12/KD12/SL2/30秒，先机构近似水平→K3定球中心→K1使能→K2开始；再K2停止保持、K4关闭使能。

方向记为“电机向下→球向右”；P方向按现有日志默认，保留反转。八套仿真和完整TD构建通过：{metrics}。实物稳定性和机构标定待验证，模型误差不等于实物精度。

SHA256 `{digest}`。自动分析最新测试：在工程目录`python analyze_follow.py`。本次代码、bit和面板完整配套；旧large60入口仍可恢复。未写Flash。
''',encoding='utf-8')
contract=json.loads(read(root/'baseline_contract.json'));contract.update(status=r['status'],date='2026-10-22',next_milestone='physical continuous-control direction, platform reference and tuning with captured VIS5 metrics',no_new_bit_delivered_this_turn=False)
(root/'baseline_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2),encoding='utf-8')
summary=f'''\n\n## 2026-10-22 连续视觉闭环VIS5软件交付，实物整定待验收

独立工程vision_follow_20261022，父large60完整冻结。K3实际中心、K1保持使能，K2开始/停止连续PD；默认G4/L12、速度阻尼12、2计数/100ms缓变、30秒会话；保留单次动作及K2软件参考返回模式。P/V不变，新增只配置不启动的S，VIS5报告429字节和中文曲线/日志统计。丢球/过期/改参数取消不自动续跑/回零，原64脉冲/5秒、MC/P±64及故障锁存保留。连续到位±1，手动/返回±2。

用户确认电机向下→球向右；不记得P方向，授权按日志默认，反向可切换。八套仿真、真实串口、Q4阻尼专项、三种假设动力学模型和完整构建通过；{metrics}。**模型不是实物标定，持续稳定/真实水平/比赛精度待用户日志**。

[新bit]({bit.as_posix()})，[配套步骤]({base}/测试说明.md)，[中文面板]({base}/host/start_panel.cmd)。SHA256 `{digest}`。父版及恢复入口见follow_lineage.json，未自动下载或写Flash。
'''
progress=root.parent/'D36A_STEPPER_调试进度总结.md'
if '2026-10-22 连续视觉闭环VIS5软件交付' not in read(progress):
    with progress.open('a',encoding='utf-8') as f:f.write(summary)
agent=root/'AGENTS.md';text=read(agent)
if '# Follow release status (2026-10-22)' not in text:
    agent.write_text(f'''# Follow release status (2026-10-22)

Current VIS5 bit SHA256 `{digest}`. Eight current-source suites, exact servo adaptation, inherited baseline/archive hashes, P/V compatibility, S bounds,429-byte actual UART, Chinese launcher/render, all routed IO/mirrors and nonnegative timing pass. {metrics}. Default mode1/G4/L12/KD12/SL2/30s, VE signed Q4. Auto inner deadband1 is an explicit adaptation; manual/return deadband2 preserved. The user authorized log-based direction default; physical dynamic calibration and actual stability remain pending. No automatic board programming or Flash. Rebuild with build.ps1, verify.py and publish_follow_delivery.py; older scripts/history do not override current documented behavior.

'''+text,encoding='utf-8')
# Pair delivered artifacts and all reproducible sources; omit TD databases and combined sim copies.
files=[]
for folder in ['rtl','user_source','host','sim','td_project']:
    for f in (root/folder).rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(root)
        if any(part in ('__pycache__','logs') or part.endswith('_Runs') for part in rel.parts):continue
        if f.suffix in ('.vvp','.vcd','.db') or f.name.endswith('.combined.v'):continue
        if folder=='td_project' and f.suffix not in ('.al','.sdc'):continue
        files.append(f)
files += list(root.glob('*.py'))+list(root.glob('*.ps1'))+list(root.glob('*.tcl'))+list(root.glob('*.md'))+list(root.glob('*.diff'))
files += [root/n for n in ['verification.json','follow_lineage.json','baseline_contract.json','camera_lineage.json','center_lineage.json','large60_lineage.json','motion_lineage.json','build_sources.json','frame_layout.json','build.log']]
files += list((root/'output').glob('*.bit'))+list((root/'output').glob('*.txt'))
run=root/f'{root.name}_Runs/phy_1'
files += [run/f'{root.name}{suffix}' for suffix in ['_phy.area','_phy.ts','_pr.timing']]
files=sorted(set(files));manifest={'status':r['status'],'date':'2026-10-22','bit_sha256':digest,'parent_archive':archive,'files':{f.relative_to(root).as_posix():sha(f) for f in files}}
(root/'delivery_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
bundle=look/'连续视觉闭环_VIS5_20261022.zip'
with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
    for f in files+[root/'delivery_manifest.json']:z.write(f,f.relative_to(root).as_posix())
with zipfile.ZipFile(bundle) as z:
    assert z.testzip() is None
    assert hashlib.sha256(z.read(bit.relative_to(root).as_posix())).hexdigest()==digest
    for rel,h in manifest['files'].items():assert hashlib.sha256(z.read(rel)).hexdigest()==h,rel
print(json.dumps({'bit':str(bit),'sha256':digest,'bundle':str(bundle),'paired_files':len(files),'bundle_sha256':sha(bundle)},ensure_ascii=False,indent=2))
