# merged_vision_motor_logo（2026-10-27）

本目录是 **2026-10-27** 的“视觉 + 电机 + HDMI OSD/Logo”合并版本源码快照，来源工作目录 `D:/AAAProject/FPGA/HX1P35A_Contest_202606/exercise/merged_vision_motor_logo`。

它是仓库根目录 2026-10-22 `vision_follow_20261022` 快照的**并行实验分支**，不是根目录源码的替换版本。

## 与根目录版本的区分

| 项目 | 根目录（2026-10-22 快照） | 本目录（2026-10-27 合并版） |
| --- | --- | --- |
| 定位 | 已建档的视觉闭环/电机控制源码快照 | 视觉、电机、HDMI 显示与 Logo 的合并集成 |
| RTL 数量 | 17 个文件 | 25 个文件（新增 8 个级联/区域控制模块） |
| 上位机 | `host/` 中文面板与启动脚本 | 无，仅 FPGA 源码与构建脚本 |
| 仿真/核对 | `sim/`、`verify.py`、`publish_*.py`、lineage JSON | 无仿真/核对脚本，只有 TD 构建与综合检查 Tcl |
| HDMI 视觉通路 | 无 | `hdmi_mixer -> visual_alg -> osd_overlay -> logo_overlay -> hdmi_tx` |
| 默认控制参数 | 旧版默认 | `DEFAULT_RUNTIME=0`、`DEFAULT_DEADZONE=3`、`DEFAULT_LIMIT=60`、`CASCADE_DEFAULT=1`、`CASCADE_PRESET=1`、`REGION_RETURN=1`、`ASYM_RANGE=1` |
| 验证状态 | 已有完整仿真、验证与历史交付记录 | 仅完成 TD 综合/布线/时序检查；**未做实物验证** |

根目录的 2026-10-22 快照及历史说明仍然有效；本目录记录的是后续合并实验，不覆盖根目录的结论。

## 本版本新增/主要变化

### HDMI 视觉与 OSD

- `user_source/hdl_source/visual_alg.v`：6 种实时算法模式。
  - 0 `RAW`：原始直通
  - 1 `GRAY`：灰度
  - 2 `BINARY`：二值化（阈值 128）
  - 3 `SOBEL`：3x3 Sobel 边缘检测（阈值 100）
  - 4 `BALL MASK`：钢球区域掩码/高亮
  - 5 `BALL TRACK`：视频直通 + 检测框 + OSD 十字
- `user_source/hdl_source/osd_overlay.v`：显示模式、球 X 坐标、速度（px/s）、`VALID/INVALID` 状态和十字准星。
- `user_source/hdl_source/logo_overlay.v`：在 OSD 之后叠加 146x94 的 Anlogic Logo，位置 `(1744, 24)`，因此所有视觉模式下都可见。
- `user_source/hdl_source/logo_rom.v` + `logo_rom.hex`：25 位 `{valid, R, G, B}` Logo ROM，13724 像素，使用块 RAM。
- `ball_speed.v` / `ball_info_cdc.v`：30fps 下的球速计算，以及 CSI 时钟域到 HDMI 时钟域的 84 位球信息同步。
- 检测分辨率 1280x720，OSD/输出为 1920x1080，坐标按 1.5 倍缩放。

### 模式切换按键

`mode_switch` 使用顶层 `I_button[3]`（约束引脚 `C7`）。该按键同时是电机侧 `K4` 停止输入，因此每次按下会**同时**切换视觉显示模式并触发 K4 停止逻辑；这是本合并版本的按键复用特性。

### 控制/级联模块

新增：

- `vision_cascade_outer.v`：帧间速度与流水线外环。
- `vision_cascade_control.v`：物理 K2 控制连续运行与有限标定流程。
- `vision_cascade_config.v`：`C01` 配置帧（固定 25 字节 payload + XOR8，hex + LF，共 54 字节）。
- `vision_cascade_diag.v`：`D1` 紧凑诊断帧（94 字节）。
- `vision_region_hold.v`：内置区域回中，带位置+速度判据和迟滞。
- `vision_region_probe.v`：有限测量图像与 C 配置选择。
- `vision_asym_limits.v`：非对称机械范围/参考锁定。
- `vision_probe_events.v`：`E1` 探针事件上报。

修改过的核心文件包括 `vision_monitor.v`、`vision_servo_core.v`、`vision_follow_control.v`、`vision_config_rx.v`、`vision_report_tx.v`、`vision_center_key.v`、`vision_cdc_bridge.v`、`vision_value_cdc.v`、`vision_shadow_target.v`、`vision_result_guard.v`、`vision_move_core.v`、`vision_camera_control.v`、`enable_latch.v`、`reference_position_core.v` 等。`vision_monitor_all.v` 由 `compose.py` 拼接生成。

电机底层仍沿用已实板验证过的 `position_core` 脉冲相位和守护逻辑的改编版本，但**不能把父版本的实板结论直接套用到本合并版本**。

## 目录结构

```
merged_vision_motor_logo/
├── rtl/                 # 25 个视觉/电机控制 RTL（vision_monitor_all.v 为拼接文件）
├── td_project/          # TD 工程 .al、时序约束 .sdc、logo_rom.hex
├── user_source/         # 摄像头/ISP/MIPI/DDR/HDMI 等平台源码与新增视觉模块
├── build_logo.tcl       # 完整流程：综合、布局、布线、时序、bitgen
├── syn_check_logo.tcl   # 只做到 gate 综合，快速检查面积
├── compose.py           # 生成 rtl/vision_monitor_all.v
└── README.md
```

## 构建

环境：Anlogic Tang Dynasty（TD）6.2.168116，器件 `PH1P35MDG324`，速度等级 `-3`。

1. 先确保 `rtl/vision_monitor_all.v` 与分散的 RTL 同步（可选）：
   ```
   python compose.py
   ```
2. 在 TD Tcl 命令行中执行：
   ```
   source build_logo.tcl
   ```
   `build_logo.tcl` 会切换到 `td_project`，打开 `vision_follow_20261022.al`，并输出到 `logo_Runs/phy_1`。成功标志为：
   ```
   MERGED_VISION_MOTOR_LOGO_BUILD_COMPLETE
   ```
3. 只做综合面积检查时执行：
   ```
   source syn_check_logo.tcl
   ```

注意：

- `vision_follow_20261022.al` 内的 `Path` 属性是原机器的绝对路径；换机器后请让 TD 重新定位工程，或把该属性改为本机路径。其余源码路径均为相对路径。
- `logo_rom.v` 使用 `$readmemh("logo_rom.hex", mem)`；工程内 `td_project/logo_rom.hex` 与 `user_source/hdl_source/logo_rom.hex` 都保留了一份，构建时需保证工作目录能找到该文件。
- 本仓库按根目录 `.gitignore` 的约定**不提交**生成的 `logo_Runs/`、`logo_routed_netlist.v`、`.bit`、`.db`、日志等构建产物。

## 本次构建结果（2026-10-27）

来源 `logo_Runs/phy_1/final_timing.rpt`：

- 布线后时序：`SWNS +0.120ns`、`STNS 0`、`HWNS +0.020ns`、`HTNS 0`，0 个违例端点，STA 覆盖率 `97.52%`。
- 资源（gate 阶段）：`20444` 寄存器、`18667` LUT、`7344` Slice（`34.62%`）、`65` RAM、`37` DSP、`86` IO。
- 布线前 gate QoR 曾显示 `Setup WNS -0.469ns`、4 个违例端点；最终以布线后报告为准。
- 生成 bit：`merged_vision_motor_logo.bit`，1,789,539 字节，SHA256：
  `9272A5EE37197F608F1ACA3CB17F5A901B07D299E6F96B363C9EC789401E94F3`。
- 布线网表 SHA256：
  `C54596D78984AEBF59C595BFC01B67B46B9A1AF4DA3F7BC791580D6007AD1E70`。

以上只能说明本版本通过 TD 综合/布线/时序检查，**不代表实物功能、闭环稳定性、按键或显示效果已经验证**。
