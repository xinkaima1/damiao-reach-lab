# damiao_reach_lab —— 达妙七轴谐波机械臂 草莓定点到达仿真

> **版本**：v8（2026-11-18）。本工程由 `xarm6-reach-lab`（UFACTORY xArm6 六轴）整体迁移而来，**原工程保留在同级目录作为只读对照基线**。
> **模型来源**：达妙科技开源七轴谐波机械臂 `left_arm_dm`（ROS 2 Humble，BSD-3-Clause）。**不自建任何几何**——派生 URDF 只做两件事：删除官方夹爪子树、改写 mesh 引用路径。
> **末端执行器**：仿真内已有的**卷曲仿生章鱼爪**演示层（纯视觉），**不采用达妙官方夹爪**。

## 1. 与 xArm6 版的差异（换臂必改项）

| 项 | xArm6 版（已废弃） | 达妙七轴版（本工程） |
|---|---|---|
| URDF | 手写 `xarm6_reach.urdf` | 派生 `damiao_reach.urdf`（官方几何/惯量/限位逐字未改） |
| 自由度 | 6 | **7** |
| 末端 link | `link6`（索引硬编码 5） | **`link7`（索引按名反查 = 6）** |
| 零位末端 | (0.207, 0, 0.112) m | **(0, -0.03325, 0.4981) m**（= `演算基准.json` 的 `zero_pose.link7_tcp_m`） |
| 关节限位 | xArm6 官方 6 组 | **达妙官方 7 组**（与官方 README §1.1 机械限位一致） |
| 力矩上限 FORCES | 163.2/163.2/81.6/81.6/30.6/30.6 N·m | **50/50/50/50/12/12/12 N·m**（取自官方 URDF `<limit effort>`，未核实是否含传动折算） |
| HOME / restPoses | `[0,-0.5,-1.0,0,0.5,0]` | **官方零位 `[0]*7`** |
| 目标点 | 原 P1–P5（在达妙臂上残差 146–200 mm，**全部不可达**） | **重选 5 点**（`select_targets.py`，残差 0.000–0.001 mm） |
| 预筛包络 | r ≤ 0.70 m、z ∈ [0.10, 0.75] | **r ≤ 0.389 m、z ∈ [-0.111, 0.507]**（实测 `workspace.link7`） |
| IK | 内置单次 | **内置粗解 + 位置-only 数值精修（LM 式阻尼递增）**；初值 = 上一点实际关节角 |
| CSV | q1–q6（20 列） | **q1–q7（21 列）** |
| 演示层 | 3 根 × 6 段触手三阶段 + 抓放循环 | **原样保留**（常量与函数逐字不动，仅挂载点索引换为实测 link7 索引） |

## 2. 目录

```
damiao_reach_lab/
├── derive_damiao_urdf.py        # 从官方 URDF 派生仿真专用 URDF（自检：15 个保留块逐字一致）
├── damiao_reach.urdf             # 派生结果（8 link / 7 revolute，无夹爪）
├── select_targets.py             # 可达空间扫描 + 5 目标点选取（可复算）
├── test_load.py                  # 加载自检（须 ALL PASS）
├── damiao_reach_demo.py          # 主脚本
├── make_damiao_guide.py          # 实验指导书口径转换脚本（文档 = 工程）
├── meshes/damiao/visual/*.STL    # 官方 visual 网格 8 个
├── meshes/damiao/collision/*.STL # 官方 collision 网格 8 个
├── 实验指导书_damiao七轴草莓定点到达.md   # 当前执行基线（内嵌代码 = 工程实际文件）
├── 实验指导书_xarm6草莓定点到达.md      # xArm6 旧版留作对照，同时是 make_damiao_guide.py 的转换输入
├── 变更记录_达妙七轴迁移_20261118.md     # 旧值 → 新值 → 原因 → 重跑结果
├── 答辩材料/                             # 达妙口径报告 md+docx、说明书、口径说明
└── results/                      # 实跑产物
```

## 3. 运行

```bash
conda activate robot
cd damiao_reach_lab

python derive_damiao_urdf.py --src \
  ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \
  --dst damiao_reach.urdf          # 期望 == DERIVE PASS ==

python test_load.py                 # 期望 == ALL PASS ==
python select_targets.py            # 可达空间扫描 + 目标点选取（可复算）
python damiao_reach_demo.py --no-video            # 快速验证，5/5 通过
python damiao_reach_demo.py --check-out-of-range   # 负向验证，退出码 2
python damiao_reach_demo.py        # 完整运行，产出 CSV + MP4 + 截图
```

## 4. 实跑结果（2026-11-18，全量重跑）

| 项 | 值 |
|---|---|
| 加载自检 | `test_load.py` → **ALL PASS**（7 关节 / 7 组官方限位逐项 PASS / 末端 `link7` 索引 6 / 零位 FK 偏差 0.000 mm / 渲染 307200 bytes） |
| 到位判定 | **5/5 点通过**（IK 残差、限位、伺服跟踪三判据全过），收敛步全为 59 |
| IK 残差 / 伺服误差 | **0.000–0.001 mm**（阈值 5 mm，未放宽） |
| 限位余量 | P1 0.407 / P2 0.308 / P3 0.490 / P4 0.730 / P5 0.636 rad |
| 负向验证 | `--check-out-of-range` → **退出码 2**（P_BAD_H 水平 0.80 m > 0.389 m；P_BAD_Z z=0.80 m ∉ [-0.111, 0.507]，均在 IK 前被预筛拒绝） |
| 渲染 | 976 帧 @ 960×720 / 30 fps（32.53 s），渲染耗时 **235.6 s**，全程 235.8 s |
| 产物 | `results/reach_results.csv`（5 行 21 列）、`results/reach_demo.mp4`（2.7 MB）、`results/scene.png`、`results/run_stats.json`、`results/target_selection.json` |
| 环境 | WSL2 Ubuntu / conda `robot` / Python 3.11.18 / pybullet 3.2.7 / numpy 2.4.6 / imageio 2.38.1 |

## 5. 口径与边界（答辩必读）

1. **无重力基线**：`setGravity(0, 0, 0)`。本仿真验证的是**轨迹规划与位置伺服的指令跟踪与定位收敛**，**不代表带载精度、不代表抓取成功率**。达妙实物有重力；按官方 URDF 惯量做的静态重力矩校核（numpy，g=9.81，6000 组限位内采样）得 J2 峰值 8.83、J3 2.67、J4 2.74、J5 0.65、J6 0.68、J7 0.24 N·m，对照电机上限 50/50/50/50/12/12/12 → **J5–J7 裕度约 11 N·m，全程静态不超限**；该值是**静态演算**（未含减速器效率与动态项），**不是 PyBullet 实跑、不是带载精度验证**。
2. **末端执行器是纯视觉演示层**：章鱼爪为 3 根 × 6 段无质量无碰撞视觉体，按 `rho = a·exp(b·θ)` 后向卷曲 → 到位反向展开 → 沿果实球面缠绕包覆，三阶段逐颗循环、放果后回卷；**不进运动学链、无接触/夹持力、不写 CSV、不参与误差判定**。几何为示意值，非中科大实物参数。
3. **不采用达妙官方夹爪**：官方 `tool_link` / `gripper_left` / `gripper_right` 与 `tool_joint` / `gripper_joint1` / `gripper_joint2` 已在派生 URDF 中整段裁剪；末端参考点固定为 `link7` 原点，**不含 0.157 m 夹爪偏移**（官方 `tool_link` 数据仅存档）。
4. **换主控不影响仿真**：仿真内没有 STM32 / CAN / ros2_control / 力矩环 / 编码器，指令关节角直接喂 PyBullet 抽象关节电机；更换主控、控制器、通信协议时**仿真代码无需改动**，但说明书必须写清这条边界（仿真未验证主控通信）。
5. **换臂必须改**：模型、限位、零位、力矩上限、末端 link、目标点、预筛包络、CSV 字段、演示层挂载点全部要换，本轮换臂清单见《变更记录_达妙七轴迁移_20261118.md》。
6. **两处待用户裁决**：①是否新增**带重力对照运行**（必须独立脚本与独立 CSV，不得与本组无重力数字混报）；②旧口径材料（xArm6 / 泰科 TA6-R3）的保留策略。

## 6. 许可与来源

- 达妙开源七轴臂 `left_arm_dm`：BSD-3-Clause（软件部分）；**机械图纸无独立 LICENSE**，商业分发前需向作者确认。
- 中科大 OpenSpiRobs（章鱼手参数来源）：PolyForm-Noncommercial，**仅课程使用**；官方无卷曲态/6 臂阵列原生 STEP，不得冒充。
- 本工程内 16 个 STL 均为官方原始文件，**未作任何修改**。
