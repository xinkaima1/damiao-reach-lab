# 达妙七轴 草莓定点到达仿真（WSL2 + PyBullet，v8.3 ORCA Hand 末端执行器版）

课设《设施草莓采摘仿生机械臂控制系统设计》的定位精度仿真验证部分。**唯一执行基线为同目录《实验指导书_damiao七轴草莓定点到达.md》（v8.3 修正版）**。
模型为**达妙科技官方开源七轴谐波机械臂** URDF 派生版（`left_arm_dm`，BSD-3-Clause）；**末端执行器为 ORCA Hand 官方 URDF 几何**（`srl-ethz/orcahand_description`，MIT License），**手基面与机械臂末端法兰面相切**装配；原 xArm6 版本保留在 [xinkaima1/xarm6-reach-lab](https://github.com/xinkaima1/xarm6-reach-lab) 作只读对照基线，v8.2 达妙七轴版见 Release `v8.2-damiao-7axis`。

## 范围声明

- **课设立论为仿生机械臂本体的定位精度验证；末端执行器为可替换模块**。v8.2 采用卷曲仿生章鱼爪纯视觉演示层，v8.3 改用 ORCA Hand 官方 URDF 几何；
- **末端执行器（v8.3）**：几何与关节结构取自 ORCA Hand 官方描述仓库（MIT）。**按实际拼装挂接**（2026-11-18 用户两次指正后重算）：
  - 手的安装基面（塔座底面，root 系 z=-0.01821）与**机械臂末端法兰面**（link7 腕筒顶面，link7 系 z=0.0907）**相切**（AABB 断言 Δ<0.1 mm），手指沿工具轴（link7 +z）外伸、掌面朝 link7 +x；
  - **精度判定点 = TCP = 果实被持点 = ORCA 握合腔中心**（link7 系 `TCP_OFFSET=(0.048, 0, 0.32091)` m）——拼装后到达果实的是末端执行器的握合腔而非法兰原点；
  - **5 个目标点 = 果实位置，由 `select_targets.py` 按 TCP 口径重算**（40000 组采样、果实点云 2628 点、前方带包络 x≥0.25 / z∈[0.12,0.45] / r≤0.55 m，对应 link7 仍在官方 r≤0.389 包络内）；
  - IK = 带姿态约束粗解（工具轴指向果实；确定性姿态网格内取**限位余量最大**者）+ **TCP 数值精修**（`tcp_refine`，对 TCP 直接做雅可比，实测残差 0.000 mm）；
  - **惯量置零以维持 v8.2 无重力轨迹跟踪基线，不代表带载精度**；17 个 revolute **锁定为预设握合姿态**（均在官方限位内；官方 URDF 未建模腱绳耦合——无 mimic/transmission——本仿真不含腱绳驱动、接触力学与夹持力）；collision mesh 原样保留于 URDF 但运行时碰撞过滤屏蔽（不参与仿真接触）。
- **仿真不采用达妙官方夹爪**——官方 URDF 的 `tool_link`（法兰转接板）/ `gripper_left` / `gripper_right` 与 `tool_joint` / `gripper_joint1` / `gripper_joint2` 已在派生 URDF 中整段裁剪（法兰面位置按官方 `tool_joint` 定义与 mesh 实测确定）；
- 不含夹爪开合、采摘接触力学、状态机、视觉/CLIP、力控；
- 精度验证对象只有一件事：**末端执行器的握合腔（TCP）依次到达 5 个写死的果实空间目标点**，误差口径为 TCP（果实被持点）；
- 仿真动力学为**无重力设定**（`setGravity(0,0,0)`）+ 轨迹段速度前馈，验证轨迹规划与位置伺服的指令跟踪与定位收敛。v8.3 另有 `--with-orca-mass` 带载对照实验（含 ORCA 官方质量 1.135 kg 的静态持重力矩 PyBullet 实算），**两套数字独立记录、禁止混报**；
- **演示层抓放循环**：每个果实点到达判定通过后，果实随 TCP 搬到篮子放果点、末端下降示意「低头放果」、果实入篮留存、前往下一点，逐颗循环。**果实搬运是纯视觉移动**：无接触/约束、无夹持力、不约束末端姿态、不写入 CSV、不参与两重静态判定与伺服跟踪判定，**不代表抓取成功率或果实识别能力**；
- v8.2 的卷曲章鱼爪演示层保留为 `--legacy-tentacle` 分支（默认关闭），保证 v8.2 可复现。

## 版本边界

| 版本 | 内容 | 状态 |
|---|---|---|
| v8.1 | UFACTORY xArm6 六轴（旧，只读对照） | `xarm6-reach-lab` 仓库 |
| v8.2 | 达妙科技开源七轴谐波臂 + 章鱼爪演示层 | Release `v8.2-damiao-7axis`（保持原样不动） |
| **v8.3（本版，含拼装/判定口径修正）** | **达妙七轴 + ORCA Hand 官方 URDF 末端执行器（法兰相切装配、TCP 判定）** | Release `v8.3-orcahand` |

**数字口径**：本 README 的 5/5 通过、0.000–0.001 mm、976 帧 / 262.1 s 等均为 **2026-11-18 v8.3 修正后重跑实测**（`results/reach_results.csv` 与 `results/run_stats.json`）；v8.2 的历史数字（976 帧 / 237.0 s 等）仅存于其 Release 与 §14.1–14.6 历史记录，不与本版混引。

## 演示素材（答辩可用）

> 以下动图与图片均由 **2026-11-18 v8.3 修正后同一次完整运行**（`results/reach_demo.mp4`，976 帧 / 960×720 / 32.53 s）派生；画面已**抽样判读 6 帧**（第 0/100/200/430/700/975 帧，覆盖待机 / 接近 / 到位 / 包拢 / 入篮 / 终态），非全量逐帧校验。答辩引用数字一律以 `results/reach_results.csv` 与《实验指导书》v8.3 §8.3 为准。

![达妙七轴 + ORCA Hand 逐颗采摘：握合腔到达果实点 → 放入篮子 → 下一颗（循环动图，约 27.4 s）](results/reach_demo.gif)

**关键帧拼图**（6 帧覆盖起始、接近、到位、包拢、入篮、终态；PPT 一页讲清全过程）：

![关键帧拼图](results/reach_keyframes.png)

| 素材 | 内容 | 规格 | 建议用途 |
|------|------|------|----------|
| `results/reach_demo.gif` | 循环动图，5 颗果实逐颗到达 / 搬运入篮（ORCA Hand 呈现） | 400×300 / 171 帧 / 160 ms 每帧（≈6.2 fps，循环约 27.4 s）/ 32 色 / 1022 KB | README 自动播放、现场快速展示 |
| `results/reach_demo.mp4` | 完整运行录像（ORCA Hand，法兰相切装配） | 960×720 / 976 帧 / 30 fps / 32.53 s / 4.7 MB | 答辩现场播放、备用录屏 |
| `results/reach_demo_legacy_tentacle.mp4` | v8.2 章鱼爪复现分支抽样录像（320×240 / 每 16 步） | 491 帧 / 57.8 s 渲染 | `--legacy-tentacle` 分支可复现的证据 |
| `results/reach_keyframes.png` | 6 帧拼图（含帧号、时间戳与中文标签） | 1440×784 / 317 KB | PPT 过程页 |
| `results/scene.png` | 终态截图（篮中 5 颗果实，由主 MP4 终态帧重建） | 960×720 | 结果页配图 |
| `results/reach_results.csv` | 逐点误差与收敛数据（5 行、21 列，TCP 口径） | ~820 B | 答辩数字唯一来源 |
| `results/run_stats.json` | 实跑统计（帧数、耗时、限位余量） | ~660 B | 口径自查 |
| `results/target_selection.json` | 果实点选取记录（TCP 残差 / 限位余量 / 关节解 / 包络，v8.3 重算） | 1.3 KB | 目标点可复算依据 |
| `results/reach_results_orca_mass.csv` | **带载对照独立 CSV**（`--with-orca-mass`，禁止与无重力基线混报） | ~830 B | 带载口径查证 |

**画面语义（答辩务必说清，避免误读）**：红球 = 待到达果实点、黄球 = 当前正在到达的目标、绿球 = 已通过判定的目标；棕色开口箱 = 篮子，绿色薄板为栽培架示意，蓝白棋盘为仿真地面；**末端白灰色灵巧手 = ORCA Hand 官方 URDF 几何**（基面与法兰相切、沿工具轴外伸，**握合腔中心 = TCP = 果实被持点**，预设握合姿态、**无开合动画**）。这些球是**位置标记，不是草莓外观或果实模型**；果实搬运为纯视觉移动，**不含抓取接触、夹持力或接触力学**；「低头放果」只是末端位置下降，不约束末端姿态。本轮精度验证的仍是 **TCP（果实被持点）依次到位**（无重力设定下的轨迹跟踪与定位收敛），不是抓取成功率或果实识别能力。

**建议口径**：现场播 MP4（或看动图）+ 关键帧拼图讲「梯形轨迹规划 + 位置伺服」两层结构，再说明末端执行器是可替换模块、ORCA Hand 为官方 URDF 几何且**基面与臂端法兰相切装配**；数字只引用 TCP 伺服误差 **0.000–0.001 mm < 5 mm**、**5/5 收敛**、976 帧 / 渲染 262.1 s、限位余量 0.268–0.647 rad；主动声明无重力设定、ORCA 惯量置零、腱绳耦合未建模、范围边界（指导书 §0 / §12 / §14.7 / §14.8）。

## 当前状态（截至 2026-11-18 23:58）

### 已实测（v8.3 修正后，2026-11-18）

- WSL2 Ubuntu，conda `robot` 环境：Python 3.11.18、pybullet 3.2.7（manylinux cp311 wheel）、numpy 2.4.6、imageio 2.38.1、pillow 13.0.0；
- 工程文件就位：`damiao_reach.urdf`（达妙官方派生 8 link / 7 revolute + ORCA 子装配 42 link / 41 fixed + 1 挂接 fixed = **50 link / 49 joint**）、`meshes/damiao/{visual,collision}/` 各 8 个官方 STL（33 MB，含法兰转接板备份）、`meshes/orcahand/{visual,collision}/` 各 34 个官方 STL（7.1 MB，均原样未改）；
- 派生自检 `derive_orcahand_urdf.py`：`== ORCA DERIVE PASS ==`（68 处 mesh 路径改写全部可达、惯量置零、17 个 revolute 锁定预设握合姿态、83 个块与官方展开结果一致、无 `package://` 残留、无 mimic/transmission）；
- 派生自检 `derive_damiao_urdf.py`：`== DERIVE PASS ==`（达妙 15 个保留块与官方逐字一致、无夹爪残留、ORCA 子装配校验通过、挂接 origin 校验通过）；
- 加载自检 `test_load.py`：`== ALL PASS ==`，**7 组官方限位逐项 PASS**、末端 link 名 `link7`（索引按名反查 = 6）、**零位 link7 原点 (0, -0.03325, 0.49810) m |Δ|=0.000 mm 且零位 TCP (0.048, -0.03325, 0.81901) m |Δ|=0.000 mm**、**拼装断言**（挂接 xyz=(0,0,0.10891)、rpy=(0,0,-π/2)）、惯量置零 / 碰撞过滤 / 包围盒 0.212×0.200×0.299 m / **手基面 z 0.58880 = 法兰面 z 0.58880（相切 Δ<0.1 mm）**、渲染 307200 bytes；
- 目标点重算 `select_targets.py`（TCP 口径）：果实点云 2628 点，5 点全部通过（TCP 残差 0.000 mm、选点阶段限位余量 0.165–0.522 rad）：P1 (0.4196,0.0078,0.3354)、P2 (0.3614,0.1476,0.2506)、P3 (0.3726,-0.1973,0.2630)、P4 (0.4794,0.0654,0.4014)、P5 (0.4618,-0.0564,0.1786)；
- 快速验证 / 负向验证：**5/5 点通过**（no-video 全程 33.5 s，含姿态网格搜索）、负向模式**退出码 2**（P_BAD_H 水平 0.80 m > 0.55 m、P_BAD_Z z=0.80 m ∉ [0.12, 0.45]，均在 IK 前被预筛拒绝）；
- **完整运行（v8.3 修正后 2026-11-18 23:58）**：5/5 点到达，TCP-IK 残差 / TCP 伺服误差 **0.000–0.001 mm**（阈值 5 mm 未放宽），`converge_step=59`，**限位余量 0.268–0.647 rad**（姿态网格取余量最大者），976 帧 @ 960×720，渲染耗时 **262.1 s**，全程 **293.8 s**（含 TCP 姿态网格搜索与 IK 约 31.7 s）；`render_time_s=262.1`；演示层 5 段抓放，放果点 TCP 残差 0.00 mm、限位 5/5 OK；`FULL_EXIT=0`；
- **`--legacy-tentacle` 分支**：快速验证 5/5 通过；降分辨率抽样渲染（320×240 / 每 16 步，491 帧 / 57.8 s）确认章鱼爪三阶段动画完整（`results/reach_demo_legacy_tentacle.mp4`）；
- **`--with-orca-mass` 带载对照**（独立口径）：`--no-video` 快速验证 5/5 通过（独立 CSV `results/reach_results_orca_mass.csv`），重力 9.81 m/s² + ORCA 官方质量 1.135 kg，逐点**静态持重力矩 PyBullet 实算**峰值 **J4 6.048 / J2 4.249 / J6 3.030 N·m**（对照 12/50/50/50/12/12/12 上限）；完整渲染未执行；
- MP4 实测可完整解码 976 帧（30 fps / 32.53 s）；画面抽样判读 6 帧正常（标注为抽样、非全量）；派生 GIF（171 帧 / 1022 KB）与关键帧拼图（317 KB）已生成并目检；`scene.png` 由主 MP4 终态帧重建。

### 未实测（不得当作结论引用）

- 降级渲染分支（`--res 640x480 --every 16`）在默认模式下的实际耗时；
- `--threshold` 换阈值复跑；
- `--with-orca-mass` 带载对照的完整渲染（仅完成快速验证）；
- 全新机器按指导书从零环境复跑（零探索验证）；
- MP4 画面内容的**全量逐帧**人工判读（已完成 6 帧抽样，非全量，不作逐帧正确性结论）；
- **带重力纯臂对照运行**（不带 ORCA 质量；需单独裁决，若做必须独立脚本 + 独立 CSV，不得与本组无重力数字混报）；
- ORCA 腱绳耦合建模、真实接触抓取 / 夹持力（均在本实验范围外）。

## 一键复跑

```bash
conda activate robot
cd damiao_reach_lab

python third_party/orcahand/expand_orcahand_xacro.py   # v8.3：ORCA 官方 xacro 展开（可复算）
python derive_orcahand_urdf.py --src third_party/orcahand/orcahand_right.urdf \
  --dst third_party/orcahand/orcahand_right_sim.urdf   # ORCA 仿真派生，须 ORCA DERIVE PASS
python derive_damiao_urdf.py --src \
  ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \
  --dst damiao_reach.urdf                              # 达妙派生 + ORCA 子装配追加，须 DERIVE PASS
python test_load.py                 # 模型加载自检，须 ALL PASS（含拼装/TCP 断言）
python select_targets.py            # TCP 果实点选取（可复算，输出 target_selection.json）
python damiao_reach_demo.py --no-video            # 快速验证：只出 CSV + 截图（统计写 run_stats_no_video.json）
python damiao_reach_demo.py --check-out-of-range  # 负向验证：预期退出码 2
python damiao_reach_demo.py        # 完整运行：CSV + MP4 + 截图（默认 ORCA Hand；最后跑，覆盖主 CSV）
python make_demo_assets.py          # 由 MP4 派生 GIF 与关键帧拼图
python make_damiao_guide.py         # 重生成实验指导书（内嵌代码与工程文件 md5 一致）
```

可选开关：`--legacy-tentacle`（v8.2 章鱼爪复现分支）、`--with-orca-mass`（带载对照实验：重力 + ORCA 官方质量，独立 CSV，**数字不与无重力基线混报**）。

固定解释器路径（免激活 conda）：`/home/slphy_kai/miniconda3/envs/robot/bin/python`。脚本依赖相对路径的 URDF / meshes / results，**必须在工程根目录内执行**。

阈值默认 5 mm（`--threshold` 可配）；渲染降级用 `--res 640x480 --every 16`。完整参数含义与数值来源见实验指导书附录 B。

## 目录

| 路径 | 说明 |
|------|------|
| `damiao_reach.urdf` | 达妙官方 URDF 派生版 + ORCA Hand 子装配（达妙部分几何/惯量/限位逐字未改；手基面与法兰相切） |
| `derive_damiao_urdf.py` | 达妙派生脚本（含「保留块与官方逐字一致」强校验 + ORCA 子装配追加与挂接校验） |
| `derive_orcahand_urdf.py` | ORCA Hand 仿真专用派生脚本（mesh 路径改写 / 惯量置零 / revolute 锁定 + 强校验） |
| `third_party/orcahand/` | ORCA 官方 xacro（原样）、展开脚本、展开 URDF、仿真派生 URDF、MIT LICENSE 副本 |
| `test_load.py` | 加载自检（关节数 / 7 组限位 / 末端 link / 零位 link7 与 TCP / 拼装 origin / 相切 / 包围盒 / 渲染通路） |
| `damiao_reach_demo.py` | 主实验：TCP 口径 IK（姿态网格 + TCP 精修）+ 梯形轨迹 + 位置伺服 + ORCA 演示层 + CSV + run_stats 回填 |
| `select_targets.py` | 果实点选取（TCP 点云扫描 / 前方带包络 / 锚点择优 / 残差与余量约束） |
| `make_demo_assets.py` | 由完整运行录像派生 GIF 与六帧关键帧拼图（ORCA 口径标签，`--legacy-tentacle` 可切换旧标签） |
| `make_damiao_guide.py` | 实验指导书口径转换脚本（保证「文档 = 工程」，内嵌代码与工程文件 md5 一致） |
| `meshes/damiao/{visual,collision}/` | 达妙官方 STL 各 8 个，原样未改 |
| `meshes/orcahand/{visual,collision}/` | ORCA 官方 STL 各 34 个，原样未改 |
| `results/` | 实验产物与演示素材（CSV / MP4 / GIF / 关键帧拼图 / 终态截图 / 统计 / 带载对照独立 CSV） |
| `答辩材料/` | 达妙口径报告 md+docx、说明书、口径说明（本次未改动） |
| `实验指导书_damiao七轴草莓定点到达.md` | 唯一执行基线 v8.3 修正版（内嵌派生脚本 / 自检脚本 / 主脚本全文） |
| `实验指导书_xarm6草莓定点到达.md` | 旧版对照（只读） |
| `变更记录_达妙七轴迁移_20261118.md` | v8.2 变更记录（保持原样） |
| `变更记录_ORCA末端执行器_v8.3_20261118.md` | v8.3 变更记录（含「八、拼装与判定口径修正」） |
| `RELEASE_NOTES_v8.3.md` | Release `v8.3-orcahand` 说明 |
| `README.md` | 本文件 |

## 与 v8.2 的差异（只换末端执行器并按其几何重算判定，臂本体不变）

| 项 | v8.2（达妙七轴 + 章鱼爪） | v8.3（达妙七轴 + ORCA Hand，修正版） |
|---|---|---|
| 末端执行器 | 卷曲仿生章鱼爪纯视觉演示层（不在 URDF 内） | **ORCA Hand 官方 URDF 几何**（fixed 挂接 `joint_orca_wrist`，**基面与法兰相切**） |
| 模型文件 | 8 link / 7 revolute | **50 link / 49 joint**（+42 ORCA link、+41 fixed、+1 挂接 fixed，不引入可动关节） |
| 判定点 | `link7` 原点（法兰） | **TCP = 果实被持点 = 握合腔中心（link7 系 (0.048, 0, 0.32091)）** |
| 目标点 | link7 可达空间 5 点 | **TCP 口径果实点重算 5 点**（预筛包络改为前方带 x≥0.25 / z∈[0.12,0.45] / r≤0.55） |
| IK | 两段式位置-only | **带姿态约束粗解（余量最大姿态网格）+ TCP 数值精修** |
| 演示层 | 章鱼爪三阶段（默认） | **默认 = ORCA 握合呈现**；章鱼爪保留为 `--legacy-tentacle` 分支 |
| 带载对照 | 无 | 新增 `--with-orca-mass`（独立 CSV + 静态持重力矩实算，峰值 J4 6.048 N·m） |
| 达妙臂主体 / 判据 / 限位 / 阈值 / CSV 字段 | — | **均不变**（几何与 16 个 STL 未动） |
| 实跑数字（v8.3 修正重跑） | 976 帧 / 237.0 s（v8.2 实测，已废弃不作本轮引用） | **976 帧 / 262.1 s / 5/5 / 0.000–0.001 mm / 限位余量 0.268–0.647 rad** |

## 许可与来源

- 达妙官方开源七轴臂 `left_arm_dm`：**BSD-3-Clause**（软件部分）；**机械图纸无独立 LICENSE**，商业分发前需向作者确认；
- **ORCA Hand 描述文件 `srl-ethz/orcahand_description`：MIT License**（本工程使用的 URDF/xacro 与 68 个 STL 来源）；硬件制造包 `orcahand/orca_files`：**CC BY 4.0，仅作参考、无 URDF、本工程不使用**；
- 中科大 OpenSpiRobs（章鱼手参数来源）：PolyForm-Noncommercial，**仅课程使用**（因 `--legacy-tentacle` 分支仍保留，仅课程演示使用）；
- 仓库内 16 个达妙 STL 与 68 个 ORCA STL 均为官方原始文件，**未作任何修改**；
- ORCA 官方 URDF 未建模腱绳耦合（无 mimic/transmission）；官方未公开的指标（额定负载、重复精度、法兰、通信协议等）一律标「待核实」，不冒充官方数据；
- 关联仓库：[xinkaima1/xarm6-reach-lab](https://github.com/xinkaima1/xarm6-reach-lab)（原 xArm6 版，只读对照基线）；本仓库 Release `v8.2-damiao-7axis`（保持原样）、`v8.3-orcahand`（本版）附仿真产物包。
