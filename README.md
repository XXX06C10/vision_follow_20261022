# 单轴视觉闭环：手动 / 连续PD / 软件参考返回

> **仓库目录区分**
> - 仓库根目录：2026-10-22 的 `vision_follow_20261022` 源码快照（含上位机、仿真、验证/发布脚本）。
> - [`merged_vision_motor_logo/`](merged_vision_motor_logo/)：2026-10-27 的“视觉 + 电机 + HDMI OSD/Logo”合并版本，独立构建，不含上位机/仿真/验证脚本，仅通过 TD 布线时序检查，未做实物验证。该目录不会替换根目录快照的结论。

2026-10-22。沿已实板运行成功的运动脚本和核心改造，独立于large60父工程。

- [新bit（VIS=5，连续控制）](D:/AAAProject/FPGA/HX1P35A_Contest_202606/exercise/vision_follow_20261022/output/vision_follow_20261022.bit)
- [配套中文上位机](D:/AAAProject/FPGA/HX1P35A_Contest_202606/exercise/vision_follow_20261022/host/start_panel.cmd)
- [测试步骤](D:/AAAProject/FPGA/HX1P35A_Contest_202606/exercise/vision_follow_20261022/测试说明.md)
- [恢复此前large60版](D:/AAAProject/FPGA/AAA_LookHere/成功版本归档/20261022_large60_连续控制前/恢复说明.md)

默认先K3确认实际中心→K1使能→K2连续开始/停止；K4立即停止。G4/L12、KD12、2计数/100ms缓变、30秒会话；曝光3600/增益143和蓝框调节保留。面板可改为单次±60动作或K2确认返回软件参考。没有自动续跑/自动寻零。

八套仿真、中文面板自检/渲染、完整TD布线通过：18132 Slice、18073寄存器、76 IO、21 DSP、45 ERAM；SWNS +0.011ns、HWNS +0.020ns，STNS/HTNS 0，STA覆盖97.03%。**实物连续闭环稳定性待日志确认**，模型有残余摆动，不能当作精密居中验收。当前bit SHA256 `1890c0545bd035ef84b0b8406653483c17826084f8b423d0f2ee671327f2c062`。

源码：`rtl/vision_follow_control.v`为外环，`rtl/vision_servo_core.v`为守护执行器，`servo_adaptation.diff`记录相对冻结父核心的完整变更；相机/IP未改。接口见[实现与协议.md](D:/AAAProject/FPGA/HX1P35A_Contest_202606/exercise/vision_follow_20261022/实现与协议.md)，检测统计用`python analyze_follow.py`。

重建：`./build.ps1`；八套仿真：`./run_sim.ps1 tb_motion`、`tb_large`、`tb_full`、`tb_center`、`tb_camera`、`tb_detector`、`tb_follow`、`tb_plant`；完整核对：`python verify.py`；发布：`python publish_follow_delivery.py`。面板自检：`host/start_panel.cmd -SelfTest`和`-SmokeTest`。历史一次性迁移/发布脚本保留溯源，不用于当前重建。
