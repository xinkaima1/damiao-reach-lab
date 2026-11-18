# v8.3 ORCA Hand 末端执行器版本（仿生臂本体口径不变）

## 1. 口径变更

**课设立论为仿生机械臂本体的定位精度验证；末端执行器为可替换模块。** v8.2 → v8.3 只换末端执行器：v8.2 采用卷曲仿生章鱼爪纯视觉演示层，v8.3 改用 **ORCA Hand 官方 URDF 几何**（固定子装配），**两者均不参与精度判定**。`link7` 精度判定点不变——ORCA 以 fixed joint `joint_orca_wrist` 挂接于 `link7` 之后，不进 IK 链、不改 5 个目标点、不改预筛包络、不改 CSV 的误差口径与 5 mm 阈值，v8.2 的判据全部沿用。

## 2. 模型来源与许可

- **ORCA Hand 描述文件**：`srl-ethz/orcahand_description`（**MIT License**）；官方 mesh 几何未修改；
- 硬件制造包 `orcahand/orca_files`（**CC BY 4.0**）：仅作参考、无 URDF、本工程不使用；
- 达妙官方开源七轴臂 `left_arm_dm`：**BSD-3-Clause**（机械图纸无独立 LICENSE）；
- 中科大 OpenSpiRobs：PolyForm-Noncommercial（`--legacy-tentacle` 分支仍保留，仅课程使用）；
- 仓库内 16 个达妙 STL 与 68 个 ORCA STL 均为官方原始文件，未作任何修改。

## 3. URDF 处理

- 官方 `orcahand.urdf.xacro` 经 **`expand_orcahand_xacro.py` 确定性展开**（`chirality:=right prefix:=orca_ extension:=true`；提示词示例命令 `extension:=false` 仅 62 处 mesh 引用，与验收要求「68 处、34+34 STL 全部可达」矛盾，按验收口径取 `extension:=true` 并已在变更记录登记），展开 URDF 与原 xacro 均提交于 `third_party/orcahand/`；
- **68 处 `package://` 引用全部改写**为工程相对路径 `meshes/orcahand/{visual,collision}/`（34 + 34 个官方 STL 原样随附）；
- 派生内容仅三类：mesh 路径改写、惯量置零、17 个 revolute 锁定为预设握合姿态（均在官方限位内）；
- 自检：`derive_orcahand_urdf.py` **ORCA DERIVE PASS**（83 个 link/joint 块与官方展开结果一致、无 `package://` 残留、68 个 mesh 文件全部存在）；`derive_damiao_urdf.py` **DERIVE PASS**（达妙 15 个保留块与官方逐字一致 + ORCA 子装配校验）；`test_load.py` **ALL PASS**（49 link / 7 revolute、零位 FK 0.000 mm、ORCA 惯量 / 碰撞过滤 / 包围盒断言全过）。

## 4. 腱绳耦合与惯量裁决（如实说明）

- ORCA 实物为 17-DoF 腱绳驱动，但**官方 URDF 未建模腱绳耦合**（实测无 mimic、无 transmission，17 个 revolute 各自独立）。本仿真将其作为**固定子装配**呈现：17 个 revolute 锁定为预设握合姿态（示意值，均在官方限位内），**不含腱绳驱动、接触力学与夹持力，不代表已仿真 17-DoF 灵巧抓取**；
- ORCA 各 link **惯量置零**，以维持 v8.2 的 `setGravity(0,0,0)` 无重力基线与位置伺服口径——**末端执行器几何取自官方 URDF，惯量置零维持 v8.2 轨迹跟踪基线，不代表带载精度**；
- 可选 `--with-orca-mass` 分支保留官方质量（tower 0.9 kg + 全惯量，合计 1.135 kg）并启用重力，独立脚本开关 + 独立 CSV + 静态持重力矩重算，**不与无重力基线数字混报**。

## 5. 重跑实测（2026-11-18，WSL2 / Python 3.11.18 / pybullet 3.2.7）

| 项 | 实测值 |
|---|---|
| `test_load.py` | **ALL PASS**（49 link、7 组官方限位逐项 PASS、`link7` 索引按名反查 = 6、零位 FK 偏差 0.000 mm） |
| `select_targets.py` | 5 点复算**与 v8.2 记录逐位一致** |
| 完整运行 | **FULL_EXIT=0**；**5/5 通过**；`err_ik` / `err_track` **0.000–0.001 mm**（阈值 5 mm 未放宽）；`converge_step=59`；限位余量 **0.308–0.730 rad**；**976 帧 @ 960×720**；渲染 **265.4 s**；全程 266.0 s；CSV **5 行 21 列** |
| `--check-out-of-range` | **退出码 2**（越界点在 IK 前被预筛拒绝） |
| `--legacy-tentacle` | 5/5 通过；降分辨率抽样渲染（320×240 / 491 帧）确认章鱼爪三阶段动画完整 |
| `--with-orca-mass` | 快速验证 5/5 通过（独立 CSV）；静态持重力矩峰值 J2 7.910 / J4 2.899 / J3 2.680 N·m（PyBullet 实算） |
| MP4 校验 | 完整解码 976 帧（30 fps / 32.53 s）；**6 帧抽样判读**正常（抽样，非全量） |

## 6. 与 v8.2 的差异

| 项 | v8.2 | v8.3 |
|---|---|---|
| 末端执行器 | 卷曲章鱼爪纯视觉演示层（不在 URDF 内） | **ORCA Hand 官方 URDF 几何（固定子装配）** |
| 模型文件 | 8 link / 7 revolute | **50 link / 49 joint**（+42 ORCA link、+41 fixed、+1 挂接 fixed，不引入可动关节） |
| 演示层默认 | 章鱼爪三阶段 | **ORCA 握合呈现**（章鱼爪保留为 `--legacy-tentacle` 分支） |
| 带载对照 | 无 | 新增 `--with-orca-mass`（独立 CSV + 力矩实算） |
| **达妙臂主体、5 个目标点、两段式 IK、预筛包络、5 mm 阈值、伺服判据、CSV 字段** | — | **均不变** |
| 实跑数字 | v8.2 重跑值（976 帧 / 237.0 s，已废弃不作本轮引用） | **本版重跑：976 帧 / 265.4 s / 5/5 / 0.000–0.001 mm** |

## 7. 边界声明

- **无重力设定**（`setGravity(0,0,0)`）：验证的是轨迹规划与位置伺服的指令跟踪与定位收敛；
- ORCA 子装配**惯量置零**：**不代表带载精度**；
- **腱绳耦合未建模**：无腱绳驱动、无接触力学、无夹持力；预设握合姿态为示意值，不代表真实抓取姿态；
- 果实搬运为**纯视觉移动**：**不代表真实抓取 / 夹持力 / 接触力学 / 抓取成功率**；
- `--with-orca-mass` 带载数字与无重力基线数字**分列记录，禁止混用**；
- 官方未公开的指标（额定负载、重复精度、法兰、通信协议等）一律标「待核实」。

## 8. 一键复跑

```bash
conda activate robot
cd damiao_reach_lab
python third_party/orcahand/expand_orcahand_xacro.py
python derive_orcahand_urdf.py --src third_party/orcahand/orcahand_right.urdf --dst third_party/orcahand/orcahand_right_sim.urdf
python derive_damiao_urdf.py --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf --dst damiao_reach.urdf
python test_load.py                              # 须 ALL PASS
python select_targets.py                         # 目标点复算（与 v8.2 一致）
python damiao_reach_demo.py --no-video           # 快速验证
python damiao_reach_demo.py --check-out-of-range # 负向验证，预期退出码 2
python damiao_reach_demo.py                      # 完整运行（默认 ORCA Hand；最后执行以生成主 CSV/MP4）
python make_demo_assets.py                       # GIF + 关键帧拼图
python make_damiao_guide.py                      # 重生成实验指导书（内嵌代码 md5 一致）
```

**开关说明**：
- `--legacy-tentacle`：复现 v8.2 卷曲章鱼爪演示层（臂本体与判据不变）；
- `--with-orca-mass`：带载对照实验（重力 9.81 m/s² + ORCA 官方质量 1.135 kg），产物写入独立 CSV `results/reach_results_orca_mass.csv` 与 `results/run_stats_orca_mass.json`，**与无重力基线数字分列，禁止混报**；
- `--no-video`：快速验证（只出 CSV + 截图，统计写 `results/run_stats_no_video.json`）；`--check-out-of-range`：负向验证（退出码 2）；`--res 640x480 --every 16`：渲染降级。
