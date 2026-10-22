"""Publish a verified K3 center release locally; never program the board."""
from pathlib import Path
import hashlib,json,zipfile
root=Path(__file__).resolve().parent
look=Path('D:/AAAProject/FPGA/AAA_LookHere');base=root.as_posix()
read=lambda p:p.read_text(encoding='utf-8-sig')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads(read(root/'verification.json'))
assert r['status']=='SOFTWARE_BUILD_PASS_AWAITING_PHYSICAL_K3_CENTER_TEST'
assert r['report_version']==4 and r['wire_UART_bytes']==342
bit=Path(r['delivery_bit']);digest=r['sha256'];assert sha(bit)==digest
assert r['timing_ns']['SWNS']>=0 and r['timing_ns']['HWNS']>=0
parent=json.loads(read(root/'center_lineage.json'));archive=Path(parent['archive']).as_posix()
links=f'''- [新bit]({bit.as_posix()})
- [启动配套中文上位机]({base}/host/start_panel.cmd)
- [恢复中心校准前的VIS=3版本]({archive}/恢复说明.md)
'''
test=f'''# K3记录实际中心：本轮测试

2026-10-22。**新bit已生成，配套界面为VIS=4。** 在原来K2单次动作版增加板上K3确认中心。曝光3600、传感器增益143、蓝框和原接线保留。

{links}
## 先只验证中心，不启动动作

1. 电机保持空载，所有按键松开，加载本轮新bit到SRAM，器件PH1P35MDG324。关闭旧串口程序，启动上面的配套中文上位机，连接原COM，115200、8N1；数据必须为VIS=4。本轮没有写Flash。
2. 先不按K1。等相机设置完成、识别有效，把球放到你认为的**实际中心位置**，保持稳定。
3. 按一下板上 **K3**，再松开。上位机应显示“已确认中心X”约等于当时球的BX、“K3：已记录”，记录次数增加1；橙色控制中心线移动到这个横坐标，左侧“控制中心X”也同步。
4. 球仍在该位置时，实时候选T应为0；向左右移动球时，候选T分别为负/正（未勾反向），已确认中心C保持不变。K3本身不会启用或驱动电机。

记录的是钢球横坐标，不是蓝框的几何中心。蓝框识别范围保持现有设置。按K3时无须连接电脑，电脑仅用于查看回读；它不需要串口命令才能完成记录。

K3只接受识别有效、新结果未超过200ms、设置完成、无故障且电机空闲的按键。K1可以关闭。长按只记录一次，下载时一直按住不会记录；未满足条件时显示原因，不排队，恢复后松开再按K3。

## 中心正确后，再验证一次小动作

保持空载，按K1并松开，等使能/参考/就绪。将球移到一侧，使候选T达到+3～+5或−3～−5，按一下K2并松开。锁定目标MT应等于本次候选T，等待MB=0、OK=1、误差ME在±2内，完成次数DC增加。

再把球移回刚才用K3记录的位置，确认T=0，按一下K2执行中心目标。**K3仅更新视觉中心，执行中心目标仍需新的K2。** 它不会自动把电机送回0，也不会重置编码器软件参考、P、MC或故障。忙时按K3会拒绝，待动作结束后重新按。

K1保持使能、K2单次执行、K4立即停止的行为保留。丢球/坏帧/200ms无新结果/应用参数取消动作，不自动回零；恢复后仍要重新按K2。目标±5、到位±2、单次12脉冲/2秒、无反馈/反向反馈和累计范围保护不变。

## 需要记住的两点

- **本轮中心保存在运行中的FPGA里。重新加载或断电后恢复默认960，需要重新按K3。** 关闭/打开上位机或切换K1不会清除中心。PC手工修改控制中心并应用P参数时，新的显式设置会生效；之后可再用K3记录。
- 上次日志末段出现过F=7：反馈位置到了76，超出±22保护范围。K3不会清除这个故障。若界面仍显示故障，保存日志后重新加载本轮bit，再开始上述步骤；建立电机参考后保持原小范围测试，避免手动大幅转轴。故障原因不能仅凭中心偏移确定。

K3与K2同时按时，中心请求阻止动作启动。若K3恰与有效串口参数提交同一控制周期发生，串口优先，界面提示冲突，松开后重新按K3即可。设置后等待有效新帧才接收下一动作。

本轮主要反馈：K3前后球BX、中心C、记录次数/状态，以及回到标定位置时T是否为0。日志自动保存在本工程host/logs。此前用户确认“还行”及有限动作证据已归档，没有扩展为完整四向验收。

## 构建及恢复

五组仿真（含K3专项、原八类故障、真实AB反馈、完整720p检测和相机/I2C流程）及中文界面检查通过。完整布线、器件/引脚和非负时序通过；[检查报告]({base}/verification.json)。本轮K3实板结果仍待验证。

新bit SHA256：`{digest}`。

原K2版本源码、bit、上位机、三份用户串口日志和哈希已冻结到[归档zip]({archive}.zip)。原bit SHA256为`{parent['parent_bit_sha256']}`，仍可恢复使用，K3在原版没有作用。
'''
(root/'测试说明.md').write_text(test,encoding='utf-8')
res=r['resources'];t=r['timing_ns']
metrics=f"{res['slice']} Slice、{res['register']}寄存器、{res['IO']} IO、{res['DSP']} DSP、{res['ERAM']} ERAM；SWNS +{t['SWNS']:.3f}ns、HWNS +{t['HWNS']:.3f}ns，STNS/HTNS为0，STA覆盖{r['STA_coverage_percent']:.2f}%"
(root/'README.md').write_text(f'''# K3确认实际中心，K2执行单次动作

2026-10-22独立工程。K3用当前有效钢球横坐标设置视觉中心；空闲时才接受，本身不驱动电机。K1保持使能、K2单次锁定±5目标/±2到位、K4停止以及曝光3600/增益143和蓝框保留。中心在断电/重新加载后需重设。

{links}
- [本轮测试说明]({base}/测试说明.md)
- [实现与协议]({base}/实现与协议.md)

**新bit已构建，软件验证通过，K3实板待验证。** VIS=4报告342字节，新字段CC记录次数、KC记录/拒绝状态；P/V报文兼容。

{metrics}。时钟周期和保护界限没有放宽。相机/IP 193份源文件不变；运动执行器、K1保持、AB计数、候选映射与相机控制均与K2父工程原字节一致。父工程完整归档，见center_lineage.json。

重新构建用 `./build.ps1`；仿真用 `./run_sim.ps1 tb_center` 等五组；完整检查用 `python verify.py`。生产流程不运行一次性迁移脚本。交付配对哈希见delivery_manifest.json。

新bit SHA256：`{digest}`。未自动下载或写入Flash。
''',encoding='utf-8')
(root/'实现与协议.md').write_text(f'''# K3中心确认：实现与接口

## 这次改动

父工程为../vision_k2_once_20261021，源、bit、中文上位机、三份用户日志和哈希归档于center_lineage.json记录的位置。用户确认“还行”，同时指出中心与实际不一致；未宣称完整四向动作通过。

vision_center_key新增K3输入同步、20ms消抖与松开资格；上电按住不触发，长按只有一次事件。vision_monitor仅在K4松开、无故障、空闲、配置完成及有效新帧未过期时提出中心更新，K1可关闭。还拒绝同周期传入的坏帧。采样的是一致快照中的球X，范围0..1919。

vision_config_rx仍为中心参数的唯一写入者，新增本地中心请求。记录中心时仅修改center，并发出与原P/V一致的参数改变脉冲；不增加串口配置序号SEQ，CC用于独立确认硬件记录。与有效UART P/V提交同周期碰撞时，UART优先，本地请求不增加CC且KC=7。之后最后成功提交的明确参数更新生效。

K3请求和提交均阻止同周期K2启动；清除配置后旧帧资格，等待之后有效新帧，K2仍需新的释放/按下。忙时K3只拒绝，不改本次目标、不打断当前有限动作。K3不重设编码器原点、P、MC、MT或故障，不自动回零。中心为易失寄存器，重新下载/断电回到960；不写Flash。

原vision_move_core、enable_latch、encoder_ab_counter、vision_shadow_target、vision_camera_control与父工程字节一致；所有原运动/停止保护保持。参数提交、视觉失效取消等完整定义及P/V协议见[父工程协议](D:/AAAProject/FPGA/HX1P35A_Contest_202606/exercise/vision_k2_once_20261021/实现与协议.md)。旧版“K3无动作”和VIS=3字段长度仅适用于父版本。

## VIS=4串口

115200、8N1，342字节含CRLF。保留原字段顺序，末尾新增 ` CC=hhhh KC=h`。CC为成功记录中心次数，16位循环；KC为最近K3结果：

| KC | 含义 |
|---|---|
| 0 | 本次加载后尚未记录 |
| 1 | 已记录 |
| 2 | 识别无效、坏帧或结果过期 |
| 3 | 等待配置或配置后的有效新帧 |
| 4 | 电机动作中 |
| 5 | 故障锁存 |
| 6 | K4按下 |
| 7 | 与串口提交同周期冲突，串口优先 |

C是当前实际控制中心，BX是球坐标。所有线上的数字仍为十六进制；中文界面显示十进制。现有9位字符索引可覆盖新长度。每份报告仍锁存一致快照，实际8N1波形仿真检查全部342字节及CC/KC。P20字节、V32字节格式、边界和XOR校验不变；没有新增PC运动命令。

上位机显示实际C、K3结果及次数。CC变化时同步左侧中心输入和橙色实际中心线，保留其他尚未发送参数；发现旧中心的待确认提交被K3更新时提示重新核对。断开/重新连接同步实际回读，关闭PC界面不清除FPGA中心。

## 验证

tb_center覆盖上电按住、按键抖动/长按、关闭使能时有效采样、丢球/坏帧/弱置信度/边界外/200ms过期/设置忙/旧帧/动作中/K4/故障拒绝；验证不发步、不改参考/累计指令、K2/K3同时按、UART同周期仲裁、0/1919边界、重置默认和线上的回读。实际串口回读示例为C=500、BX=500、T=0、CC=7、KC=1。

同时保留tb_motion八类故障、tb_full真实AB反馈有限动作/取消/参数/CDC、tb_detector完整720p识别与ROI、tb_camera原实际I2C与三时钟配置流程。中文GUI测试检查K3中心回读同步、保留未发送参数、负数/报文兼容和实际渲染。

完整构建：{metrics}。目标PH1P35MDG324，官方相机引脚与双口STEP C18/V2、DIR D15/V1、EN E12/U2和活动E1 E16/E17复核。时钟周期/组和相机10ns邮箱约束保持；原时钟布线提示分类保留。详见verification.json。

本轮软件/布线检查通过；K3实板、连续跟随、机构标定与机械回中未宣称通过。新bit SHA256：`{digest}`。
''',encoding='utf-8')
(root/'父版本实测分析_20261022.md').write_text('''# K2父版本已有记录的范围

用户确认“测试了一下，还行”，同时指出默认中心不符合实际。已读取父工程host/logs中的三份VIS=3日志：548、1201、1178条完整记录，C均为960。前两份AC/DC均为0；第三份最多AC=7/DC=7，X1始终0。

第三份的7次完成中，6次SP=0，目标已在±2到位带内；一次MT=+5/P=+3/ME=+2/SP=4，有实际发步并进入到位带的证据。没有据此宣称完整正负/中心四步通过。

第三份后段在EN=0时反馈已到P=76，之后F=7锁存，MC仍为4。整个日志P范围−195..+241，明显超过小动作允许反馈±22。不能仅凭日志断定外力/机械移动的具体原因；K3只改视觉中心，原保护保持。重新进入使能检查时越界会触发该保护。

原三份日志与bit已经复制到“20261022_K2单次视觉动作_中心校准前”归档，哈希和恢复入口见center_lineage.json。归档不再改动。
''',encoding='utf-8')
entry=f'''# K3按键确认实际中心

2026-10-22。**新bit与配套中文上位机已更新为VIS=4。** 把球放到你认可的实际中心，按板上K3记录当前横坐标；K3本身不会让电机动作。无须把中心固定为画面960。

{links}
- [这次只需照做的步骤]({base}/测试说明.md)

先不按K1，等有效识别，把球摆在实际中心，按一下K3。上位机应显示记录成功、中心C与球BX一致、T=0；移动球后C保持。之后仍用K1使能、K2执行单次动作、K4停止。重新下载或断电后需要重设中心。

五组仿真、中文界面、完整构建/器件/引脚/非负时序通过。{metrics}。没有修改原运动核心或放宽保护。K3实板结果待反馈，连续跟随未启用。

原VIS=3版本和日志已归档。已观察到一次目标5/P3/误差2的有限动作；日志后段还有P76导致F7的保护记录，K3不会清除编码器故障。详细范围见[父版本分析]({base}/父版本实测分析_20261022.md)。

SHA256：`{digest}`。
'''
(look/'下一步_K3确定实际中心.md').write_text(entry,encoding='utf-8')
def prepend(path,heading,body):
 s=read(path)
 if heading in s:return
 first=s.find('\n')+1;path.write_text(s[:first]+'\n'+heading+'\n\n'+body+'\n\n以下保留此前记录。\n'+s[first:],encoding='utf-8')
body=('2026-10-22：用户选择板上K3确定实际中心。新独立工程记录有效球X，空闲且无故障时才接受；K3不发步、不重设编码器参考，原保护保持。VIS=4 bit与中文界面已生成，五组仿真和完整布线检查通过，实板K3待验。中心为易失状态，重新下载/断电需重设。[当前入口](D:/AAAProject/FPGA/AAA_LookHere/下一步_K3确定实际中心.md)。SHA256 `'+digest+'`。')
prepend(look/'实施计划_20261019/目标验收状态.md','## 最新结果：K3确定实际中心版本已交付，待实板确认',body)
prepend(root.parent/'D36A_STEPPER_调试进度总结.md','## 2026-10-22 最新：板上K3记录实际视觉中心',body)
contract=root/'baseline_contract.json';data=json.loads(read(contract));data.update(status=r['status'],date='2026-10-22',report_version=4,
 delivered_bit_sha256=digest,no_new_bit_delivered_this_turn=False,physical_test='PENDING_USER_K3_BOARD_TEST',next_milestone='physical K3 center capture, then limited K2 moves around calibrated center')
contract.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
files=['output/'+root.name+'.bit','host/start_panel.cmd','host/vision_panel_cn.ps1','host/vision_panel.ps1','host/panel_preview.png',
 'README.md','测试说明.md','实现与协议.md','父版本实测分析_20261022.md','verification.json','center_lineage.json','frame_layout.json','decode_serial.py','build_sources.json','baseline_contract.json']
files+=['sim/results/'+name+'.log' for name in ('tb_center','tb_full','tb_motion','tb_camera','tb_detector')]
manifest={'date':'2026-10-22','status':r['status'],'report_version':4,'bit_sha256':digest,'hardware_programmed':False,
 'files':{rel:sha(root/rel) for rel in files}}
(root/'delivery_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
bundle=look/'K3中心确认_VIS4_20261022.zip'
with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
 for rel in files+['delivery_manifest.json']:z.write(root/rel,rel)
with zipfile.ZipFile(bundle) as z:assert z.testzip() is None
print('Published K3 center bit/GUI/test entry and hash-paired bundle; physical K3 test pending')
