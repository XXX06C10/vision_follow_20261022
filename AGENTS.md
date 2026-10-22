# Authorized loss500 revision (2026-10-22)

The user explicitly approved local red tracking hysteresis,2 missing-frame recovery, and500ms valid-ball grace ONLY for an already running continuous session. Camera result watchdog remains200ms. Preserve raw V/Q/B/AGE and VIS5/P/V/S wire format, all motor bounds and fault guards. Initial K2/K3/manual/return still require real fresh vision. Archive recorded in loss500_lineage.json is immutable. This supersedes earlier immediate-loss AUTO stop language; never fake a valid frame or reset burst guards during coasting. Rebuild/deliver new loss500 bit and matching Chinese panel. No automatic board programming.

# Follow release status (2026-10-22)

Current VIS5 bit SHA256 `1890c0545bd035ef84b0b8406653483c17826084f8b423d0f2ee671327f2c062`. Eight current-source suites, exact servo adaptation, inherited baseline/archive hashes, P/V compatibility, S bounds,429-byte actual UART, Chinese launcher/render, all routed IO/mirrors and nonnegative timing pass. 18132 Slice、18073寄存器、76 IO、21 DSP、45 ERAM；SWNS +0.011ns、HWNS +0.020ns，STNS/HTNS 0，STA覆盖97.03%. Default mode1/G4/L12/KD12/SL2/30s, VE signed Q4. Auto inner deadband1 is an explicit adaptation; manual/return deadband2 preserved. The user authorized log-based direction default; physical dynamic calibration and actual stability remain pending. No automatic board programming or Flash. Rebuild with build.ps1, verify.py and publish_follow_delivery.py; older scripts/history do not override current documented behavior.

# Current continuous-control authorization (2026-10-22)

User explicitly requested completing remaining vision-loop development. This independent project preserves the large60 parent and original actuator. Add default continuous PD mode, manual single-move and explicit software-reference-return modes, K2 start/stop, K4 raw stop, K3 visual center, Chinese configuration/curves/logging, finite burst guards and safe retargeting at CHECK only. No physical stability or mechanical homing claim from simulation. No per-frame resetting of actuator pulse budgets/timeouts. Loss/config/disable/fault stops the session without automatic resume or homing. Confirm visual center before AUTO. Keep the official PH1P35 pins/clocks and pulse timing lineage. Preserve 64 pulse/5s active-burst and +/-64 command/feedback guards. Older restrictions below describe parent stages; new automatic motion is explicitly authorized. Direction observation is motor down -> ball right; absolute target sign cannot replace actual P delta. Default feedback orientation must be documented from logs/human confirmation and remain configurable. Only hardware K2 starts motion; S packet configures settings only.

# Vision control integration rules

## Current authorized large-target revision (2026-10-22)

The user explicitly requested TARGET_LIMIT=60, MOVE_BUDGET=64, POSITION_LIMIT=64 and FEEDBACK_LIMIT=64, and gain range0..12 with limit0..60. These values supersede earlier +/-5 / +/-22 restrictions below. Preserve the protocol, debounce, K1/K2/K3/K4 roles, deadband2 and no-response3. Actual FPGA RX bounds must match the host; its old bounds were not absent as the user assumed. Existing 75ms/pulse phases require a 5-second timeout instead of2 seconds, at both the core and monitor override. Preserve the previous release in large60_lineage.json's immutable archive before editing in place. Targets remain absolute relative to the software origin; test0/+60/0/-60/0, and retain64-pulse protection for an oversized direct cross-end move. Default runtime gain7/limit5 remains unless the user applies new parameters. New bit, GUI, source hashes and full nonnegative routed timing must be delivered together. Do not run old one-time migration/publish scripts over this revision.

Large60 delivered bit SHA256 `2c4d1f78aea341f11e157d289467434596a03739af1769e3c8109b875f9099eb`, alias output/vision_k3_center_20261022_large60.bit. Six suites, launcher SelfTest/SmokeTest, exact parameter-only core comparison, P/V format/checksum comparison and full routing checks pass. 17528 Slice, SWNS +0.107ns / HWNS +0.020ns, no clock/ADC changes. Large-target physical test remains pending; no automatic programming/Flash occurred. Current publish script is publish_large60.py, current report verification.json and steps 测试说明.md. Earlier delivery paragraphs below are history, not the current parameter limits.

These rules record the user's explicit 2026-10-20 instruction: the old vision/motor/GUI project is reference material and its code is not a trusted working motor baseline. Development must follow the motor scripts and behavior established during this conversation.

## Trusted baselines

- `../d36a_autocal_latch_20261020`: user-confirmed K1 toggle / K4 stop behavior. Bit SHA256 `3c76cc6bed6f8219714435afb2aa3b23462b2408751be2fa0afd2a80ff323906`.
- `../d36a_position5_20261020`: physically observed small-range encoder feedback moves within +/-2 count deadband. Use its pulse phases, guards and error handling when adapting an external-target executor.
- `../d36a_autocal_20261020`: physically measured 4095/4097 counts per 3200-pulse leg, mean4096, return residual-2. This finite calibration sequencer is not a continuously driven visual servo.
- Golden archives are under `D:/AAAProject/FPGA/AAA_LookHere/成功版本归档/`. Do not edit the archived files.

## Implementation

1. Use the validated motor-side module/script lineage. Borrow camera/ISP/recognition and GUI ideas from `../lab_hd_1_mipi_hdmi_1080p_steel_ball_motor_manual`, but do not use its `steel_ball_motor_manual` implementation or old motor commands as a trusted control path.
2. Keep K1 debounced toggle, K4 immediate stop/clear, default disabled, no held-key auto-enable, no fault bypass or uncommanded restart. Camera exposure/gain must have a separate control path; K1/K4 must not simultaneously change exposure/gain.
3. Preserve known STEP/DIR/EN mirror roles C18/V2, D15/V1, E12/U2 and active E1 encoder E16/E17. Match the current proven ADC pull settings. Verify all integrated IO, including camera/HDMI/DDR IP resources, after routing.
4. Active feedback sign is `position = E0 - E1`. A software reference is not mechanical homing. Mean4096 counts/nominal revolution and1.28 counts/pulse are supported by the automatic ratio measurement, with nominal step/microstep assumptions; no independent Z/index calibration was performed.
5. First integrated milestone observes coherent ball coordinates, confidence/freshness, encoder position and proposed bounded target in one telemetry stream. Do not silently connect the old automatic 3200-pulse test to visual target updates. Enable motor motion only in the explicitly identified limited-motion milestone.
6. FPGA performs real-time control; the PC configures bounded parameters and displays/logs data. Validate parameter bounds on FPGA, apply atomically, echo sequence/readback; do not trust host range checks alone. One serial TX owner, and documented RX protocol.
7. Use per-frame coherent clock-domain transfer and a watchdog for missing frames. Existing valid bits are not sufficient if frames stop. Unconfirmed/lost/stale detections cannot keep issuing movement targets. Define stop/hold behavior explicitly.
8. Start with the previously validated small target/pulse bounds. Verify direction against mechanism response before closing the ball loop. Do not assume motor angle equals rod angle, or use old pixel/mm estimates as completed mechanical calibration.

## Build and delivery

- Follow the working `build.ps1` -> `build.tcl` pipeline, TD executable, PH1P35MDG324 device check, completion-marker check, isolated output folder, SHA256 and verification report. Camera/DDR integration may require additional IP ADC/SDC reads; record these as explicit additions to the proven flow.
- Never bypass device mismatch, relax clocks or add false paths just to obtain a passing report. Recheck full integrated resource/timing results rather than adding separate project resource counts.
- Run meaningful simulation before building: retained motor guards, K1/K4/fault behavior, target bounds, stale/lost vision, CDC snapshots and parameter parser/ack tests. Adaptation differences from the physical baselines must be recorded and tested.
- Every new board test must include its corresponding bit, exact steps, expected results and recovery baseline. Say clearly whether the bit changed. Do not ask the user to repeat passed GPIO/round-trip tests without a new unresolved concern.
- Preserve the source project and successful bit. Work in this independent directory. Report hardware evidence only for what the user actually confirmed; don't extend K1/K4 confirmation to a repeated full automatic measurement.

## Authorized finite-motion milestone (2026-10-21)

The user explicitly approved unloaded K2-triggered single visual moves. Use the passed position_core lineage, K1 latch/K4 immediate stop, +/-5 targets, deadband2, pulse budget12, timeout2s and existing fault guards. No continuous follow or automatic resume. Loss/staleness or any parameter commit cancels without driving back to zero; a fresh released-then-pressed K2 is required. Keep exposure3600/gain143/ROI and the parameter packets. Deliver VIS=3 plus matching Chinese panel, bit and board steps. Motion is authorized in this independent project; prior observation-only restrictions describe the archived parent version. New physical motion validation remains pending.

## Current delivery status (2026-10-21)

VIS=3 single-move bit SHA256 `4f376faa5b81fbca71b774c2fb9081b7fca43354595a2c416f560815db486ca2`. Four simulation suites, eight fault fixtures, real quadrature feedback model, 329-byte serializer, Chinese host rendering, source lineage and full routing checks passed. Routed SWNS +0.103ns / HWNS +0.020ns, no clock relaxation. Both physical STEP/DIR/EN mirror pairs are active executor signals, rather than the archived parent's constants. The user confirmed the parent's brightness/ROI effect good; its source/bit/host/hash archive is immutable. This new K2 move release still awaits unloaded physical testing. See README.md, 测试说明.md, 实现与协议.md and verification.json. No automatic board programming or Flash write occurred.

## Archived parent milestone (2026-10-21)

Chinese camera-control observation release: bit SHA256 `32e2a18714fd050a9c2859e636d75e845a493d116b11ecd463e1dc4c7a4a85ba`. Source motor modules remain byte-identical. Startup exposure/gain is 3600/143. New checked V packets and VIS=2 telemetry configure exposure/gain/ROI and require both I2C-flow completion and frame-boundary ROI application. Legacy ae_set and steel_ball_roi_uart are no longer compiled. Both new camera clock-domain mailbox paths are bounded at10ns; the old24MHz->50MHz blanket false path was removed. Clock periods and reference clock groups are unchanged. Classify the documented reference PHY-5060/PHY-5079 clock-routing notices rather than hiding unreviewed critical errors. See verification.json and 相机蓝框实现说明.md.

Routed STEP/DIR pads remain constant zero. Control/UART/CDC, actual full-720p ROI exclusion, actual reference I2C configuration, and Chinese GUI draw/move/readback/render tests passed. Physical fusion/brightness/ROI/RX are pending user confirmation. Do not mark the release physically passed or silently enable automatic motion. The previous observation V1 is archived in releases/observation_v1_before_camera_controls.zip; it is not a physically passed golden. Golden motor archives remain immutable.

## Current K3 center milestone (2026-10-22)

The user explicitly requested and selected board K3 to record the current ball X as the visual control center. Work here independently; preserve the VIS=3 parent and archives. K3 accepts only fresh valid vision, completed settings, no fault, no K4 and idle motor; K1 may be off. K3 never moves the motor or changes the encoder origin/limits. Held-at-boot/long-held keys cannot replay. Parameter freshness and K2 release qualification remain mandatory. Valid UART commits win exact-cycle collisions. Deliver matching VIS=4 host/bit and board steps. User's parent confirmation is qualitative plus limited UART evidence, not full four-direction acceptance. Later F7 at P76 must not be erased by K3; do not loosen +/-22 protection.

K3 release now built and verified: SHA256 `31c95d3abc5c0cf1bca8078fc873818f8ef735f5e61e4b90c0865670c1e26efc`, 17599 Slice, 19 DSP, 45 ERAM, SWNS +0.185ns / HWNS +0.020ns. All five suites, source lineage, routed pins/mirrors and Chinese GUI tests pass. Physical K3 confirmation remains pending. No automatic programming or Flash write occurred. Current entry is 测试说明.md; report is verification.json, paired delivery hashes in delivery_manifest.json. The older delivery paragraphs above record parent history.
