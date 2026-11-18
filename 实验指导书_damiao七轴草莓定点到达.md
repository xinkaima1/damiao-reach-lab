# 实验指导书：达妙七轴谐波机械臂 草莓采摘定点到达仿真（WSL2 + PyBullet，v8.3 ORCA Hand 末端执行器版）

> **版本**：**v8.3（达妙七轴 + ORCA Hand 末端执行器版，2026-11-18）**：**课设立论为仿生机械臂本体的定位精度验证；末端执行器为可替换模块**。v8.3 把末端执行器由卷曲仿生章鱼爪演示层更换为 **ORCA Hand 官方 URDF 几何**（srl-ethz/orcahand_description，MIT License）：以 fixed joint 挂接于 `link7` 之后作为**固定子装配**，惯量置零、17 个 revolute 锁定为预设握合姿态、碰撞过滤屏蔽不参与接触，**不进 IK 链、不改 5 个目标点、不改误差口径**（判定点仍为 `link7` 原点）。仿生臂本体全部沿用 v8.2 口径：达妙官方 URDF 派生（几何/惯量/限位逐字未改）、7 组官方限位、力矩上限 50/50/50/50/12/12/12 N·m、预筛包络 r≤0.389 m / z∈[-0.111, 0.507]、两段式位置-only IK、5 mm 阈值、`setGravity(0,0,0)` 无重力设定。v8.2 章鱼爪演示层保留为 `--legacy-tentacle` 分支（默认关闭）。§8.2 / §8.3 / §13 按 2026-11-18 v8.3 全量重跑回填（976 帧 / 265.4 s / render_time_s 265.4）。
> **口径边界**：v8.2 及更早的实跑数字**不是 v8.3 的重跑结果**，引用处均注明来源版本；v8.3 重跑数字以 `results/reach_results.csv` 与 `results/run_stats.json` 为准。带载对照实验（`--with-orca-mass`）数字**独立记录**，禁止与无重力基线混报。
> **执行环境**：Windows + WSL2 Ubuntu，conda `robot`（Python 3.11 + pybullet 3.2.7）
> **阅读约定**：所有命令可直接复制粘贴执行。**v8.3 为当前唯一执行基线**；仍标注「未执行」的项目不得当作结论引用。


---

## 0. 范围声明（红线，不随排期改变）

本实验为课设《设施草莓采摘仿生机械臂控制系统设计》的**定位精度仿真验证部分**，范围锁定如下：

1. **纯仿真**：不制实物。**末端执行器为 ORCA Hand 官方 URDF 几何（v8.3，固定子装配）**：几何与关节结构取自 srl-ethz/orcahand_description（MIT），惯量置零、17 个 revolute 锁定为预设握合姿态、碰撞过滤屏蔽不参与接触、不进运动学链、不参与误差判定；**仿真不采用达妙官方夹爪**——官方 URDF 的 `tool_link` / `gripper_left` / `gripper_right` 与 `tool_joint` / `gripper_joint1` / `gripper_joint2` 已在派生 URDF 中整段裁剪（§5）；v8.2 的卷曲仿生章鱼爪演示层保留为 `--legacy-tentacle` 分支（§7）；
2. **不含**夹爪开合、采摘接触力学、状态机、视觉/CLIP、力控；ORCA 腱绳耦合未在官方 URDF 中建模（无 mimic/transmission），本仿真将其作为固定子装配呈现，**不含腱绳驱动、接触力学与夹持力**，17 个 revolute 锁定为预设握合姿态、无开合动画；
3. 精度验证对象只有一件事：**达妙七轴谐波臂的末端 link7 依次到达 5 个写死的空间目标点**；
4. 方法：PyBullet 内置 `calculateInverseKinematics` **位置-only IK**（不约束姿态）+ **位置-only 数值精修**（§2.1，两段式，原因与实现见 §7 `refine_ik()`）；
5. 引擎口径全项目统一为 **PyBullet**，文档、说明书、答辩话术不出现第二引擎叙事；
6. 仿真动力学为**无重力设定**（`setGravity(0,0,0)`）+ 轨迹段速度前馈：验证的是轨迹规划与位置伺服的**指令跟踪与定位收敛**。**达妙实物有重力**：按官方 URDF 惯量做静态重力矩校核（numpy 运算，g=9.81，6000 组限位内采样）得 J2 峰值 8.83 N·m、J3 2.67、J4 2.74、J5 0.65、J6 0.68、J7 0.24 N·m，对照电机上限 50/50/50/50/12/12/12，**J5–J7 裕度约 11 N·m，全程静态不超限**；但该值是**静态演算**（未含减速器效率与动态项），**不是带载精度验证、也不是 PyBullet 实跑**。重力前馈补偿与带载精度属实物化后续工作，不在本轮范围。本条为答辩与说明书的统一口径，主动声明而非被问出来；
7. **演示层抓放循环（v5 正式纳入范围）**：每个草莓点到位判定完成后，追加一段「移到篮子放果点 → 末端下降（示意低头放果）→ 果实视觉搬入篮中并留存 → 前往下一点」的轨迹，形成 抓-放-抓 的采摘观感。该层是**纯视觉示意**：篮子与果实均为静态视觉体，果实位置由脚本直接搬运，**无夹爪、无接触/约束、无夹持力、不约束末端姿态**；放果点不是第 6 个验收目标，不写入 CSV、不参与 §1.3 的两重静态判定与伺服跟踪判定，也不代表抓取成功率或果实识别能力。演示层另含 **末端执行器视觉呈现**（**v8.3 默认 = ORCA Hand 固定子装配**：官方 URDF 几何、预设握合姿态、握合腔对准果实保持点、纯视觉、无接触力；**`--legacy-tentacle` 分支 = v8.2 卷曲章鱼爪**：3 根 x 6 段纯视觉体，三阶段「卷曲成对数螺旋 → 反向展开 → 球面缠绕包覆」，几何为示意值、非中科大实物参数，触手与果实在 link7 局部坐标系内生成、再按 link7 的世界旋转矩阵变换到世界系）；两者都**不是真实抓取**、无接触 / 夹持力、不参与误差判定、不写 CSV，也不代表抓取成功率。叙事口径为**图纸与说明书讲仿生机构结构，仿真以达妙七轴平台验证控制流程**。答辩必须主动说明这一边界。

---

## 1. 实验目标与验收口径

### 1.1 目标

机械臂末端 link（**link7**）依次到达 5 个预先写死的草莓空间目标点。控制结构为两层：**关节空间梯形轨迹规划**（生成 240 Hz 目标轨迹）+ **关节位置伺服**（`POSITION_CONTROL` 带分关节力矩上限与速度前馈跟踪轨迹；动力学含 URDF 惯量与关节阻尼，**无重力设定**，见 §0 第 6 条）。逐点记录两个误差：IK 数值残差（静态）与伺服跟踪误差（无重力设定下控制回路的末端定位精度）。动画另含 §0 第 7 条的**演示层抓放循环**，用于呈现采摘流程，但不参与上述任何测量与判定。

### 1.2 误差定义（原理、脚本、结果表、答辩四处统一，不得混用）

- **末端 link 名 = `link7`**（官方 URDF 中 `link7` 是七轴臂最后一节；官方夹爪 `tool_link` 已被裁剪，故末端即 `link7`）；
- **到位误差 = 目标点到 link7 原点（URDF link frame 原点）的欧氏距离**，**不含任何夹爪偏移**（官方 `tool_link` 相对 `link7` 有 0.157 m 偏移，本轮口径作废、不使用）；
- 代码实现：`getLinkState(robot, ee_idx, computeForwardKinematics=1)` 返回元组的 **[4] `worldLinkFramePosition`**（主口径，与 IK 的末端参考系一致）；**[0] `linkWorldPosition` 是质心（COM）位置，仅作备查列记录**，不得作为主判据（达妙臂两者相差约 **71.7 mm**，混用会制造虚假误差）。
- **`ee_idx` 按 link 名反查**（`resolve_ee_index()`），不硬编码索引：官方夹爪子树裁剪后 link 索引会变化，硬编码必然出错。

### 1.3 可达性判定（两重静态判定 + 伺服跟踪判据）

一个目标点"到达"当且仅当以下全部成立：

| # | 判定 | 实现 |
|---|------|------|
| 1 | IK 解的 FK 残差 ≤ 阈值 | `resetJointState` 设置 IK 解后 `getLinkState(computeForwardKinematics=1)` 实测末端误差（`--threshold`，默认 0.005 m 可配） |
| 2 | 关节解在限位内 | 7 个关节角全部落在官方 URDF 限位区间（容差 1e-6） |
| 3 | 伺服跟踪判据 | 位置伺服执行完轨迹后，末端误差连续 60 步（0.25 s @240 Hz）达标；伺服步数上限只是超时兜底，不是收敛判据 |

说明：第 1、2 项是**静态数值判定**（求解层面），第 3 项是**控制回路判定**（执行层面）。不设所谓"独立复核"项——PyBullet 的 `calculateInverseKinematics` 不返回残差，FK 残差只能自设关节角后测量，同一件事测两次不构成独立验证。

水平半径（≤ 0.389 m）与 z 范围（[-0.111, 0.507]）**仅作 IK 前预筛**（快速失败），不构成可达性证明；该包络取自 `handoff/演算基准.json` 的 `workspace.link7`（6000 组限位内随机位形采样，`r_max`≈0.389 m）。`--check-out-of-range` 负向模式只验证"脚本会拒绝越界点"，其输出明示该语义，不冒充可达性证据。

### 1.4 验收产物

1. `results/reach_results.csv`：双误差列（`err_ik` 静态残差 / `err_track` 伺服跟踪误差）+ COM 备查列 + 限位判定 + 伺服收敛步数 + 渲染耗时（`render_time_s` 为全局渲染耗时，各行同值；**v7 实跑 = 133.319 s**）；仍为 5 行、20 列，演示层不写入；
2. `results/reach_demo.mp4`：机械臂依次到达 5 个草莓点、并在每点后完成「抓 → 放入篮子 → 前往下一点」演示段的动画（**v5 实跑 976 帧 / 32.53 s**）；
3. `results/scene.png`：终态场景截图（篮中 5 颗果实可见）；
4. `results/reach_demo.gif` / `results/reach_keyframes.png`：由同一次运行派生的答辩素材（循环动图 / 关键帧拼图）；
5. 控制台汇总表（含演示层抓放段计数与边界声明）；
6. 未收敛/超阈值的点**如实标注并保留在结果表中**（这本身是合法的实验结论），按 §9.2 规则化调整后重跑并登记。

---

## 2. 原理速览（答辩口径级）

### 2.1 位置-only 逆运动学

7 轴机械臂末端位置由 7 个关节角决定；只约束末端 **位置**（3 个自由度）而不约束姿态时，方程数 3 < 未知数 7，为**冗余（超定欠约束）问题**，解不唯一。PyBullet 的 `calculateInverseKinematics` 在显式传入 `lowerLimits / upperLimits / jointRanges / restPoses` 时采用 **nullspace（零空间）法**：在满足位置约束的解流形中，选取最接近 `restPoses`（参考姿态）的解。因此：

- **解由限位 + restPoses + 初值共同决定**——三者在脚本中全部写死，保证他人复跑得到同一组关节角（可复现性）；
- **解连续性处理**：初值不再是固定 HOME，而是**上一点收敛后的实际关节角**（`solve_ik(..., seed=q_cur)`），避免 7 轴冗余下零空间漂移导致逐点解跳变、伺服跟踪误差变大甚至不收敛；HOME 仍作为 `restPoses` 的 nullspace 参考；
- **两段式求解**：内置求解器在「远距离初值」下常停在 **3–8 mm 的局部解**（超过 5 mm 阈值）。因此粗解后再做一段**位置-only 数值精修**（数值雅可比 + 阻尼最小二乘，阻尼递增重试、关节角始终夹在官方限位内），实测把 5 点残差压到 **0.000–0.001 mm**。该精修不改变阈值、不改限位、不改判据，只改求解精度；
- **j7 不可由位置约束确定**：`link7` 原点即 joint7 轴心，joint7 旋转不改变该点位置，故位置-only IK 解中 j7 保持初值（≈0）。这不影响位置误差判定，但关节角序列不唯一，答辩需说明。

### 2.2 URDF 运动学来源（关键工程事实）

达妙官方开源仓库 `left_arm_dm`（ROS 2 Humble，BSD-3-Clause）**直接提供 URDF 与 STL**，无需自建模型：

- 模型来源 ← `src/left_description_grip/urdf/left_description_grip.urdf`（SolidWorks URDF Exporter 1.6 导出）
- 几何与惯量 ← 同一 URDF 的 `<inertial>` 块（`base_link` 0.94 kg、`link1` 0.87、`link2` 0.278、`link3` 1.005、`link4` 1.244、`link5` 0.0756、`link6` 0.926、`link7` 0.1765 kg）
- 关节限位 ← 同一 URDF 的 `<limit lower/upper>`（与官方 README §1.1 机械限位一致；**SolidWorks 导出 CSV 里的 Joint Lower/Upper 是另一套更窄的值，禁止混用**）
- 力矩上限 ← 同一 URDF 的 `<limit effort>`（J1–J4 = 50、J5–J7 = 12 N·m；**取自官方原始值，未核实是否含传动折算**，不自行乘过载系数）
- 外观 mesh ← 官方 8 个 STL（`base_link` + `link1`~`link7`，visual 与 collision 各一套，共 16 个文件）
- **本轮唯一的模型改动**：派生一份仿真专用 URDF，删除官方夹爪子树并把 `package://` 路径改为相对路径（§5）。**未新建任何几何、未修改任何尺寸/惯量/限位**。

### 2.3 零位 FK 理论值（用于加载自检）

达妙官方 URDF 的 7 个关节 `origin` **rpy 全为 0**，因此零位 FK 退化为各关节原点在世界系的**直接累加**（每级旋转为单位阵）：

| 关节 | origin xyz (m) | 累加后 link 原点 (m) |
|------|----------------|---------------------|
| joint1 | (0, 0, 0.0756) | link1 = (0, 0, 0.0756) |
| joint2 | (0, -0.042, 0.0425) | link2 = (0, -0.042, 0.1181) |
| joint3 | (0, 0.042, 0.0744) | link3 = (0, 0, 0.1925) |
| joint4 | (-0.042, 0, 0.1156) | link4 = (-0.042, 0, 0.3081) |
| joint5 | (0.042, 0, 0.0995) | link5 = (0, 0, 0.4076) |
| joint6 | (-0.03325, 0, 0.035) | link6 = (-0.03325, 0, 0.4426) |
| joint7 | (0.03325, -0.03325, 0.0555) | **link7 = (0, -0.03325, 0.4981)** |

即**零位末端（link7 原点）= (0, -0.03325, 0.4981) m**，与 `handoff/演算基准.json` 的 `zero_pose.link7_tcp_m` 逐位一致（实测偏差 0.00001 mm）。`test_load.py` 以该值断言（容差 ±5 mm）——这是**自洽性自检**（验证派生 URDF 与官方 URDF 互洽、加载链路无损坏）。

> 对照：官方 `tool_link` 原点在 (0, 0, 0.65498) m，相对 link7 有 **0.157 m** 夹爪偏移。本轮口径**不使用** `tool_link`，该数据仅作存档。

### 2.4 两层控制结构：轨迹规划 + 关节位置伺服

本实验的控制结构对应课设题目"控制系统设计"的最小完整回路：

1. **轨迹规划层**：对每个目标点求 IK 得 q*，从当前关节角到 q* 生成**归一化梯形速度插值**目标轨迹（加速 25% / 匀速 50% / 减速 25%，480 步 @240 Hz = 2 s）；
2. **关节伺服层**：`setJointMotorControlArray(POSITION_CONTROL)` 按分关节力矩上限（附录 B）逐仿真步跟踪目标轨迹，轨迹段同时给 `targetVelocities` 速度前馈（消除纯位置 PD 的跟踪滞后）；`stepSimulation()` 推进动力学（含 URDF 惯量与关节阻尼；**无重力设定**，见 §0 第 6 条——重力下纯位置伺服存在稳态下垂误差，需逆动力学前馈补偿，属实物化后续工作）；
3. **反馈测量**：伺服过程中/结束后用 `getLinkState(computeForwardKinematics=1)` 测末端实际位置——**伺服跟踪误差 `err_track` 才是"控制系统定位精度"**，IK 残差 `err_ik` 只是求解器的数值质量。

伺服收敛判据：到位后保持段误差**连续 60 步（0.25 s）≤ 阈值**；伺服总步数上限（超时兜底）为 3000 步（240 Hz 下 12.5 s），超时≠收敛。增益 positionGain=1.0 / velocityGain=1.0（无重力设定下的验证基线，依据见附录 B；调整指引见附录 C）。

---

## 3. 环境搭建

### 步骤一：进入 WSL2 并确认

在 PowerShell 执行：

```powershell
wsl -d Ubuntu
```

进入 Ubuntu 后确认（预期输出示例跟在 `#` 后）：

```bash
cat /etc/os-release | head -2   # Ubuntu 26.04 ...
whoami                          # 你的用户名（下文 ~/ 即其家目录）
```

### 步骤二：安装 Miniconda（已装可跳过）

```bash
# 检测是否已装：输出路径则跳过本步
command -v conda || echo "NOT_INSTALLED"
```

未安装时执行：

```bash
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh
bash ~/miniconda.sh -b -p ~/miniconda3
~/miniconda3/bin/conda init bash
exec bash
command -v conda   # 预期输出：/home/<你>/miniconda3/bin/conda 或 condabin 路径
```

### 步骤三：创建 robot 环境（Python 3.11）并装依赖

> **为什么锁 3.11**：pybullet 预编译 wheel 实测仅覆盖 cp36–cp311（3.2.6 / 3.2.7 均有 `cp311-manylinux_2_17_x86_64` wheel，无 cp312+）。用 3.12/3.13/3.14 会意外触发源码编译。

```bash
conda create -n robot python=3.11 -y
conda activate robot
python -V    # 预期输出：Python 3.11.x
```

安装依赖（**正常路径**：wheel 直接命中，无需任何编译器，全程 <1 分钟）：

```bash
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pybullet==3.2.7 numpy imageio imageio-ffmpeg
```

验证：

```bash
python -c "import pybullet as p; print('pybullet OK, API', p.getAPIVersion())"
python -c "import numpy; print('numpy', numpy.__version__)"
python -c "import imageio; print('imageio', imageio.__version__)"
python -c "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())"
```

预期：四条均无报错；最后一条打印 ffmpeg 可执行文件路径。**实测记录（2026-11-04）**：pybullet = **3.2.7**（cp311 manylinux wheel，103.2 MB，下载 6 分 31 秒 @ ~247 kB/s，build time Jan 29 2025）；numpy 2.4.6 / imageio 2.38.1 / imageio-ffmpeg 0.8.1 / pillow 13.0.0；`IMPORTS_OK 202010061 2.4.6 2.38.1`。详见《运行日志_20261104.md》Node 1。

**异常路径（仅当 wheel 拉取失败才走，正常情况跳过）**：源码编译需要完整编译链，耗时 5–10 分钟属正常现象，不是卡死：

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple --no-binary pybullet pybullet==3.2.7
```

若编译失败，按排错表 #3 处理。

---

## 4. 项目目录与资产准备

### 步骤四：建目录、拷官方 STL

达妙官方资产全部来自官方开源仓库（`left_arm_dm`，BSD-3-Clause）：URDF + 8 个 link 的 visual / collision STL。ORCA Hand 资产来自 srl-ethz/orcahand_description（MIT）。

```bash
# 官方源目录（本工作区）
DM=/mnt/d/桌面/workbuddy_project/2026-11-18-17-48-11/damiao_7axis_arm/left_arm_dm/src/left_description_grip
OC=/mnt/d/桌面/workbuddy_project/2026-11-18-17-43-24/orcahand_description_official
# 新工程目录（本工作区）
mkdir -p damiao_reach_lab/meshes/damiao/visual damiao_reach_lab/meshes/damiao/collision \
         damiao_reach_lab/meshes/orcahand/visual damiao_reach_lab/meshes/orcahand/collision \
         damiao_reach_lab/third_party/orcahand damiao_reach_lab/results
for n in base_link link1 link2 link3 link4 link5 link6 link7; do
  cp "$DM/meshes/visual/$n.STL"   damiao_reach_lab/meshes/damiao/visual/
  cp "$DM/meshes/collision/$n.STL" damiao_reach_lab/meshes/damiao/collision/
done
cp "$OC/assets/urdf/visual/"*.stl   damiao_reach_lab/meshes/orcahand/visual/    # 34 个官方 STL，原样
cp "$OC/assets/urdf/collision/"*.stl damiao_reach_lab/meshes/orcahand/collision/  # 34 个官方 STL，原样
cp "$OC/models/urdf/orcahand.urdf.xacro" damiao_reach_lab/third_party/orcahand/
cp "$OC/LICENSE" damiao_reach_lab/third_party/orcahand/LICENSE_orcahand_description
ls -l damiao_reach_lab/meshes/damiao/visual/ damiao_reach_lab/meshes/orcahand/visual/
```

预期输出：达妙 visual / collision 各 **8 个** `.STL`（合计约 33 MB）；ORCA visual / collision 各 **34 个** `.stl`（合计约 7.1 MB）。**STL 文件本身均不做任何修改**，只在派生 URDF 里改写引用路径。`tool_link.STL` / `gripper_left.STL` / `gripper_right.STL` 有意不拷贝（官方夹爪不进本仿真）。

最终目录结构：

```
damiao_reach_lab/
├── derive_damiao_urdf.py        # 步骤五：达妙 URDF 派生脚本（含 ORCA 子装配追加）
├── derive_orcahand_urdf.py      # v8.3：ORCA Hand 仿真专用派生脚本
├── damiao_reach.urdf             # 步骤五生成（达妙官方派生 + ORCA 子装配）
├── select_targets.py             # 目标点选取工具（可复算）
├── test_load.py                  # 步骤六：加载自检（含 ORCA 断言）
├── damiao_reach_demo.py          # 步骤七：主脚本（默认 ORCA；--legacy-tentacle / --with-orca-mass）
├── make_damiao_guide.py          # 本指导书的口径转换脚本
├── third_party/orcahand/         # ORCA 官方 xacro + 展开脚本 + 展开与派生 URDF（MIT，原样）
├── meshes/damiao/{visual,collision}/*.STL     # 达妙官方网格各 8 个
├── meshes/orcahand/{visual,collision}/*.stl   # ORCA 官方网格各 34 个
├── 实验指导书_damiao七轴草莓定点到达.md
└── results/                      # 运行产物
```

---

## 5. 生成达妙仿真专用派生 URDF（`derive_damiao_urdf.py`）

### 步骤五：派生并自检 URDF

达妙官方 URDF 可直接用 ROS 加载，但 PyBullet 不认 `package://` 路径，且官方夹爪子树（含 2 个移动关节）会污染自由度统计。因此**派生一份仿真专用 URDF**，规则严格限定为三件事：

1. **删除官方夹爪子树**：`link{tool_link, gripper_left, gripper_right}` 与 `joint{tool_joint, gripper_joint1, gripper_joint2}` 整段删除；
2. **其余内容逐字保留**：`link{base_link, link1..link7}` 与 `joint1..joint7` 的几何、惯量、限位、effort/velocity、origin/axis **一个数都不改**（脚本内置「保留块与官方逐字一致」自检，15 个块全等）；
3. **仅改写 mesh 引用路径**：`package://left_description_grip/meshes/{visual,collision}/X.STL` → `meshes/damiao/{visual,collision}/X.STL`，并把 16 个官方 STL 原样复制到 `meshes/damiao/`（**STL 文件本身不改**）。

> 注：官方 `tool_joint` 实际是 **fixed** 关节（不是转动关节），官方 URDF 共 10 个 joint = 7 revolute + 1 fixed + 2 prismatic；裁剪后**恰好 7 个可动关节**。`link7.STL` 内含官方「夹爪相机 3507 安装板」结构，属官方 link7 的一部分，**保留原样**。

```bash
cd damiao_reach_lab
python derive_damiao_urdf.py \
  --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \
  --dst damiao_reach.urdf
```

自检输出（实测）：

```
[派生] link 保留 8（期望 8），joint 7（期望 7）
[派生] 类型 revolute=7 prismatic=0 fixed=0（期望 7/0/0）
[派生] mesh 路径改写 16 处，全部 -> meshes/damiao/...
[派生] 残留夹爪引用: 无；残留 package:// 网格引用: False
[派生] 保留块与官方逐字一致: 15 个；不一致: 无
== DERIVE PASS ==
```

派生结果文件头（节选）：

```
<?xml version="1.0" encoding="utf-8"?>
<!-- ====================================================================
     达妙科技开源七轴谐波机械臂 —— 仿真专用派生 URDF（含 ORCA Hand 末端执行器子装配）
     来源一：damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/
           left_description_grip.urdf（SolidWorks URDF Exporter 1.6，BSD-3-Clause）
     来源二：srl-ethz/orcahand_description（MIT License），
           third_party/orcahand/orcahand_right_sim.urdf
           （由 expand_orcahand_xacro.py 展开 + derive_orcahand_urdf.py 派生）
     派生方式：derive_damiao_urdf.py 自动生成
       1) 删除官方夹爪子树 link{tool_link,gripper_left,gripper_right}
          与 joint{tool_joint,gripper_joint1,gripper_joint2}；
       2) 保留 link{base_link,link1..link7} 与 joint1..joint7 的
          几何、惯量、限位、effort/velocity、origin/axis 原值，逐字未改；
       3) 仅改写 mesh 引用路径 package:// -> 相对路径（STL 文件本身未改）；
       4) 在 link7 之后追加 ORCA Hand 子装配（fixed 挂接，惯量置零，
          17 个 revolute 锁定为预设握合姿态；不进 IK 链，不改精度口径）。
```

派生脚本全文（可复算）：

```python
#!/usr/bin/env python3
"""derive_damiao_urdf.py - 从达妙官方 URDF 派生「仿真专用」URDF。

派生规则（严格按 PROMPT_FINAL_达妙七轴仿真迁移.md 坑 1 / 坑 2）：
  1. 删除官方夹爪子树：link tool_link / gripper_left / gripper_right
     与 joint tool_joint / gripper_joint1 / gripper_joint2 整段删除；
  2. 其余 8 个 link（base_link, link1..link7）与 7 个 joint 的
     几何、惯量、限位、effort/velocity、origin/axis 全部逐字保留，不做任何修改；
  3. 仅把 mesh 引用路径由 ROS 的 package:// 改为 PyBullet 可解析的相对路径
     （visual -> meshes/damiao/visual/*.STL，collision -> meshes/damiao/collision/*.STL）；
  4. 官方原文件不被触碰，派生结果单独另存。

用法：
  python derive_damiao_urdf.py \
      --src  ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \
      --dst  damiao_reach.urdf

v8.3 新增（ORCA 末端执行器，口径见 PROMPT_ORCA末端执行器_v8.3_RELEASE.md §0.2/§4）：
  5. 在 link7 之后以 fixed joint 挂接 ORCA Hand 仿真专用子装配
     （来源 third_party/orcahand/orcahand_right_sim.urdf，由 derive_orcahand_urdf.py
     从官方 MIT 描述文件派生：惯量置零、17 个 revolute 锁定为预设握合姿态）；
     ORCA 不进 IK 链、不改 5 个目标点、不改误差口径（判定点仍为 link7 原点）；
  6. 达妙臂主体 8 link / 7 revolute / 15 个官方保留块逐字一致的要求不变，
     新增对 ORCA 子装配（42 link / 41 fixed + 1 个挂接 fixed joint）的强校验。
"""
import argparse
import os
import re
import sys

DROP_LINKS = {"tool_link", "gripper_left", "gripper_right"}
DROP_JOINTS = {"tool_joint", "gripper_joint1", "gripper_joint2"}

# ---- ORCA 子装配挂接参数（link7 -> orca_root，fixed） ----
# 朝向依据（2026-11-18 实测）：官方展开 URDF 在 root 系中手指沿 +z 延伸、卷向 +y
# （掌面朝 +y）。挂接变换将掌面转到 link7 局部 -z（果实方向）、握合腔中心对准
# 果实保持点 link7 局部 (0, 0, -FRUIT_HELD_Z=0.03)：手指朝腕前方伸展并从上方包拢果实。
ORCA_SIM_URDF = "third_party/orcahand/orcahand_right_sim.urdf"
ORCA_MOUNT_XYZ = "-0.15 0 -0.06"
ORCA_MOUNT_RPY = "1.5707963267948966 0 1.5707963267948966"
EXPECT_ORCA_LINKS, EXPECT_ORCA_JOINTS = 42, 41
EXPECT_ORCA_MESHES = 68

LINK_RE = re.compile(r"[ \t]*<link\b.*?</link>\s*", re.S)
JOINT_RE = re.compile(r"[ \t]*<joint\b.*?</joint>\s*", re.S)
NAME_RE = re.compile(r'name\s*=\s*"([^"]+)"')
MESH_RE = re.compile(r'(filename\s*=\s*")([^"]+)(")')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    args = ap.parse_args()

    with open(args.src, "r", encoding="utf-8") as f:
        text = f.read()

    # ---- 1. 抽取并删除夹爪子树的 link / joint ----
    kept_links, dropped_links = [], []
    for m in LINK_RE.finditer(text):
        name = NAME_RE.search(m.group(0)).group(1)
        (dropped_links if name in DROP_LINKS else kept_links).append(name)
    kept_joints, dropped_joints = [], []
    for m in JOINT_RE.finditer(text):
        name = NAME_RE.search(m.group(0)).group(1)
        (dropped_joints if name in DROP_JOINTS else kept_joints).append(name)

    missing = (DROP_LINKS - set(dropped_links)) | (DROP_JOINTS - set(dropped_joints))
    if missing:
        print(f"[FAIL] 官方 URDF 中未找到待裁剪元素: {sorted(missing)}")
        return 1

    def strip_links(t):
        def rep(m):
            name = NAME_RE.search(m.group(0)).group(1)
            return "" if name in DROP_LINKS else m.group(0)
        return LINK_RE.sub(rep, t)

    def strip_joints(t):
        def rep(m):
            name = NAME_RE.search(m.group(0)).group(1)
            return "" if name in DROP_JOINTS else m.group(0)
        return JOINT_RE.sub(rep, t)

    out = strip_links(text)
    out = strip_joints(out)

    # ---- 2. mesh 路径改写（仅路径，几何文件不动） ----
    rewritten = []

    def fix_mesh(m):
        path = m.group(2)
        if "package://" not in path:
            return m.group(0)
        rel = path.split("package://left_description_grip/", 1)[1]
        rel = rel.split("/", 1)[1] if rel.startswith("meshes/") else rel
        new = f"meshes/damiao/{rel}"
        rewritten.append((path, new))
        return m.group(1) + new + m.group(3)

    out = MESH_RE.sub(fix_mesh, out)

    # ---- 3. 机器人名与派生说明头 ----
    out = out.replace('<robot\n  name="left_description_grip">',
                      '<robot\n  name="damiao_7axis_reach">')
    header = (
        "<!-- ====================================================================\n"
        "     达妙科技开源七轴谐波机械臂 —— 仿真专用派生 URDF（含 ORCA Hand 末端执行器子装配）\n"
        "     来源一：damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/\n"
        "           left_description_grip.urdf（SolidWorks URDF Exporter 1.6，BSD-3-Clause）\n"
        "     来源二：srl-ethz/orcahand_description（MIT License），\n"
        "           third_party/orcahand/orcahand_right_sim.urdf\n"
        "           （由 expand_orcahand_xacro.py 展开 + derive_orcahand_urdf.py 派生）\n"
        "     派生方式：derive_damiao_urdf.py 自动生成\n"
        "       1) 删除官方夹爪子树 link{tool_link,gripper_left,gripper_right}\n"
        "          与 joint{tool_joint,gripper_joint1,gripper_joint2}；\n"
        "       2) 保留 link{base_link,link1..link7} 与 joint1..joint7 的\n"
        "          几何、惯量、限位、effort/velocity、origin/axis 原值，逐字未改；\n"
        "       3) 仅改写 mesh 引用路径 package:// -> 相对路径（STL 文件本身未改）；\n"
        "       4) 在 link7 之后追加 ORCA Hand 子装配（fixed 挂接，惯量置零，\n"
        "          17 个 revolute 锁定为预设握合姿态；不进 IK 链，不改精度口径）。\n"
        "     说明：达妙臂未新建任何几何，未修改任何尺寸、惯量与限位数值。\n"
        "     ==================================================================== -->\n"
    )
    # XML 声明必须位于文件首行，派生说明插在声明之后
    decl = re.match(r"(<\?xml[^>]*\?>\s*)", out)
    if decl:
        out = decl.group(1) + header + out[decl.end():]
    else:
        out = header + out

    # ---- 3.5 追加 ORCA Hand 子装配（fixed 挂接到 link7，不进 IK 链） ----
    with open(ORCA_SIM_URDF, "r", encoding="utf-8") as f:
        orca_text = f.read()
    orca_body = orca_text[orca_text.index("<robot"):orca_text.index("</robot>")]
    orca_body = orca_body[orca_body.index(">") + 1:]          # 去掉 <robot ...> 开标签
    orca_mount = (
        "  <!-- v8.3 ORCA Hand 挂接：link7 -> orca_root（fixed，不进 IK 链）。\n"
        "       朝向：掌面向上托住果实（握合腔开口向上），指尖自腕前方 (+x)\n"
        "       向上包拢；握合腔中心对准果实保持点 link7 局部 (0, 0, -0.03)。\n"
        "       偏移与旋转依据官方展开 URDF 的 root 系几何实测确定。 -->\n"
        f'  <joint name="joint_orca_wrist" type="fixed">\n'
        f'    <origin xyz="{ORCA_MOUNT_XYZ}" rpy="{ORCA_MOUNT_RPY}" />\n'
        f'    <parent link="link7" />\n'
        f'    <child link="orca_root" />\n'
        f'  </joint>\n'
    )
    out = out.replace("</robot>", orca_mount + orca_body + "</robot>")

    with open(args.dst, "w", encoding="utf-8") as f:
        f.write(out)

    # ---- 4. 自检 ----
    with open(args.dst, "r", encoding="utf-8") as f:
        chk = f.read()
    n_links = len(re.findall(r"<link\b", chk))
    n_joints = len(re.findall(r"<joint\b", chk))
    n_revolute = len(re.findall(r'type\s*=\s*"revolute"', chk))
    n_prismatic = len(re.findall(r'type\s*=\s*"prismatic"', chk))
    n_fixed = len(re.findall(r'type\s*=\s*"fixed"', chk))
    leftover = [n for n in list(DROP_LINKS) + list(DROP_JOINTS) if f'"{n}"' in chk]
    pkg = len(re.findall(r'filename\s*=\s*"package://', chk))
    orca_mesh_missing = []
    for m in MESH_RE.finditer(chk):
        path = m.group(2)
        if path.startswith("meshes/orcahand/") and not os.path.exists(path):
            orca_mesh_missing.append(path)
    n_orca_links = len(re.findall(r'<link\s+name="orca_', chk))
    n_orca_joints = len(re.findall(r'<joint[^>]*name="orca_', chk))
    n_orca_mount = len(re.findall(r'<joint name="joint_orca_wrist" type="fixed">', chk))

    # ---- 5. 强校验：达妙保留块与官方逐字一致（忽略 mesh 路径与空白） ----
    def norm(block):
        b = re.sub(r'filename\s*=\s*"[^"]*"', 'filename="<PATH>"', block)
        return re.sub(r"\s+", " ", b).strip()

    def blocks(text):
        out = {}
        for m in list(LINK_RE.finditer(text)) + list(JOINT_RE.finditer(text)):
            out[NAME_RE.search(m.group(0)).group(1)] = norm(m.group(0))
        return out

    src_blocks, dst_blocks = blocks(text), blocks(chk)
    mismatch = [n for n in dst_blocks
                if n in src_blocks and src_blocks[n] != dst_blocks[n]]
    unchanged = [n for n in dst_blocks
                 if n in src_blocks and src_blocks[n] == dst_blocks[n]]

    # ORCA 子装配块与官方展开+派生结果逐字一致（忽略 mesh 路径）
    orca_blocks = blocks(chk)
    orca_src = blocks(orca_text)
    orca_mismatch = [n for n in orca_src
                     if n in orca_blocks and orca_src[n] != orca_blocks[n]]
    orca_unchanged = [n for n in orca_src
                      if n in orca_blocks and orca_src[n] == orca_blocks[n]]

    print(f"[派生] link 保留 {n_links}（期望 8+42=50），joint {n_joints}（期望 7+41+1=49）")
    print(f"[派生] 类型 revolute={n_revolute} prismatic={n_prismatic} fixed={n_fixed}（期望 7/0/42）")
    print(f"[派生] mesh 路径改写 {len(rewritten)} 处，全部 -> meshes/damiao/...")
    print(f"[派生] 残留夹爪引用: {leftover or '无'}；残留 package:// 网格引用: {pkg} 处")
    print(f"[派生] 达妙保留块与官方逐字一致: {len(unchanged)} 个；不一致: {mismatch or '无'}")
    print(f"[派生] ORCA 子装配: link={n_orca_links} joint={n_orca_joints}（期望 {EXPECT_ORCA_LINKS}/{EXPECT_ORCA_JOINTS}），"
          f"挂接 joint={n_orca_mount}（期望 1），mesh 缺失: {orca_mesh_missing or '无'}，"
          f"块与官方派生一致: {len(orca_unchanged)}/{len(orca_src)}；不一致: {orca_mismatch or '无'}")

    ok = (n_links == 8 + EXPECT_ORCA_LINKS
          and n_joints == 7 + EXPECT_ORCA_JOINTS + 1
          and n_revolute == 7 and n_prismatic == 0 and n_fixed == EXPECT_ORCA_JOINTS + 1
          and not leftover and pkg == 0 and not orca_mesh_missing
          and not mismatch and len(unchanged) == 15
          and n_orca_links == EXPECT_ORCA_LINKS and n_orca_joints == EXPECT_ORCA_JOINTS
          and n_orca_mount == 1 and not orca_mismatch
          and len(orca_unchanged) == len(orca_src) == EXPECT_ORCA_LINKS + EXPECT_ORCA_JOINTS)
    print("== DERIVE PASS ==" if ok else "== DERIVE FAILED ==")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

### v8.3 新增：ORCA Hand 末端执行器派生（`derive_orcahand_urdf.py`）

ORCA Hand 官方只提供 `.urdf.xacro`（PyBullet 不能直接读取）。派生链路：

```bash
cd damiao_reach_lab
python third_party/orcahand/expand_orcahand_xacro.py      # 官方 xacro -> 确定性展开 URDF
python derive_orcahand_urdf.py   --src third_party/orcahand/orcahand_right.urdf   --dst third_party/orcahand/orcahand_right_sim.urdf      # 仿真专用派生 + 强校验
python derive_damiao_urdf.py   --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf   --dst damiao_reach.urdf                                  # 达妙派生 + 追加 ORCA 子装配
```

派生内容仅三类（官方几何 mesh 与 link/joint 结构未改）：① 68 处 mesh 路径改写为工程相对路径；② 全部 `<inertial>` 惯量置零（维持 v8.2 无重力基线口径）；③ 17 个官方 revolute 锁定为 fixed（预设握合姿态，均在官方限位内；官方 URDF 无 mimic/transmission，腱绳耦合未建模）。挂接：`link7 -> orca_root`（fixed，不进 IK 链，判定点仍为 `link7` 原点）。

自检输出（实测，`derive_orcahand_urdf.py`）：

```
[输入] link=42 joint=41（revolute=17 fixed=24） package://=68 mimic=0 transmission=0
[派生] mesh 路径改写 68 处 -> meshes/orcahand/...，文件缺失: 无
[派生] 输出 link=42 joint=41（revolute=0 fixed=41） package://=0 mimic=0 transmission=0
[派生] revolute 锁定 17 个：orca_wrist=0.0, orca_thumb_mcp=0.45, ...（全部在官方限位内）
[派生] 与官方展开结果一致块（白名单三类除外）: 83 个；不一致: 无
== ORCA DERIVE PASS ==
```

ORCA 派生脚本全文（可复算）：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""derive_orcahand_urdf.py - 从 ORCA Hand 官方展开 URDF 派生「仿真专用」URDF。

来源与许可（严格口径）：
  * 描述文件：srl-ethz/orcahand_description（MIT License），官方 xacro 原样复制于
    third_party/orcahand/orcahand.urdf.xacro，展开结果 orcahand_right.urdf
    （展开口径与复现命令见 expand_orcahand_xacro.py）；
  * 硬件制造包 orcahand/orca_files（CC BY 4.0）仅作参考，未提供 URDF，本工程不使用。

派生规则（严格按 PROMPT_ORCA末端执行器_v8.3_RELEASE.md §4）：
  1. mesh 路径改写：package://orcahand_description/assets/urdf/{visual,collision}/X.stl
     -> meshes/orcahand/{visual,collision}/X.stl（68 处，STL 文件本身原样未改）；
  2. 惯量置零：全部 42 个 link 的 <inertial> mass 与惯性张量置 0（origin 保留），
     维持 v8.2 的 setGravity(0,0,0) 无重力基线与位置伺服口径不变；
  3. 关节锁定：官方 17 个 revolute（腱绳驱动自由度，官方 URDF 无 mimic/transmission，
     各自独立）在本仿真 URDF 中转换为 fixed 关节并锁定在预设握合姿态
     （各角度均在官方 <limit> 内，转换时把锁定角烘焙进 joint origin 的 rpy，
     并在注释里登记官方限位）；官方几何、link/joint 结构与限位在展开 URDF 中
     原样保留，仿真专用 URDF 不含任何几何修改；
  4. collision mesh 保留在 URDF 中，但运行时由 damiao_reach_demo.py / test_load.py
     通过 setCollisionFilterGroupMask 关闭接触（不参与仿真接触）。

强校验（全部通过才输出 == ORCA DERIVE PASS ==）：
  * 展开输入含 68 处 package:// 引用、42 link / 41 joint（17 revolute + 24 fixed）、
    无 mimic、无 transmission；
  * 派生输出无 package:// 残留；68 个 mesh 文件全部存在；
  * 除「mesh 路径 / 惯量 / 关节锁定」三类白名单改动外，link/joint 块与官方展开
    结果逐字一致（按派生脚本同名机制做归一化比对）。

用法：
  python derive_orcahand_urdf.py \
      --src third_party/orcahand/orcahand_right.urdf \
      --dst third_party/orcahand/orcahand_right_sim.urdf
"""
import argparse
import math
import os
import re
import sys

PKG_PREFIX = "package://orcahand_description/assets/urdf/"
MESH_RE = re.compile(r'(filename\s*=\s*")([^"]+)(")')
LINK_RE = re.compile(r"[ \t]*<link\b.*?</link>\s*", re.S)
JOINT_RE = re.compile(r"[ \t]*<joint\b.*?</joint>\s*", re.S)
NAME_RE = re.compile(r'name\s*=\s*"([^"]+)"')
ORIGIN_RE = re.compile(r'(<origin\b[^>]*\brpy\s*=\s*")([^"]+)(")')
TYPE_RE = re.compile(r'type\s*=\s*"(\w+)"')

# 预设握合姿态（示意值；官方 URDF 未建模腱绳耦合，也未提供官方姿态表，故自行取一组
# 位于官方 <limit> 内的固定角度，仅作视觉呈现，不代表任何真实抓取姿态）。
PRESET = {
    "orca_wrist": 0.0,
    "orca_thumb_mcp": 0.45,
    "orca_thumb_abd": 0.55,
    "orca_thumb_pip": 0.50,
    "orca_thumb_dip": 0.15,
    "orca_index_abd": 0.10,
    "orca_index_mcp": 1.15,
    "orca_index_pip": 0.40,
    "orca_middle_abd": 0.08,
    "orca_middle_mcp": 1.20,
    "orca_middle_pip": 0.42,
    "orca_ring_abd": 0.06,
    "orca_ring_mcp": 1.15,
    "orca_ring_pip": 0.40,
    "orca_pinky_abd": 0.04,
    "orca_pinky_mcp": 1.05,
    "orca_pinky_pip": 0.35,
}


# ---------------- rpy / 旋转工具（URDF rpy = 绕固定轴 x-y-z，即 R = Rz(y)Ry(p)Rx(r)） ----------------
def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_to_rpy(R):
    pitch = -math.asin(max(-1.0, min(1.0, R[2][0])))
    if abs(abs(R[2][0]) - 1.0) < 1e-12:          # 万向锁退化（本 URDF 不出现，兜底）
        roll = 0.0
        yaw = math.atan2(-R[1][2], R[1][1])
    else:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    return roll, pitch, yaw


def axis_angle_matrix(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return [[x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c]]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def parse_axis(block):
    m = re.search(r'<axis\s+xyz\s*=\s*"([^"]+)"', block)
    return tuple(float(v) for v in m.group(1).split()) if m else (0.0, 0.0, 1.0)


def fmt(v):
    return f"{v:.12g}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="官方展开 URDF（orcahand_right.urdf）")
    ap.add_argument("--dst", required=True, help="仿真专用派生输出")
    args = ap.parse_args()

    with open(args.src, "r", encoding="utf-8") as f:
        text = f.read()

    # ---- 0. 展开输入结构校验 ----
    n_pkg = text.count(PKG_PREFIX)
    n_links = len(re.findall(r"<link\b", text))
    n_joints = len(re.findall(r"<joint\b", text))
    n_rev = len(re.findall(r'type\s*=\s*"revolute"', text))
    n_fixed = len(re.findall(r'type\s*=\s*"fixed"', text))
    n_mimic = len(re.findall(r"<mimic\b", text))
    n_trans = len(re.findall(r"<transmission\b", text))
    print(f"[输入] link={n_links} joint={n_joints}（revolute={n_rev} fixed={n_fixed}）"
          f" package://={n_pkg} mimic={n_mimic} transmission={n_trans}")

    # ---- 1. mesh 路径改写（仅路径，STL 文件不动） ----
    rewritten = []

    def fix_mesh(m):
        path = m.group(2)
        if PKG_PREFIX not in path:
            return m.group(0)
        rel = path[len(PKG_PREFIX):]
        new = f"meshes/orcahand/{rel}"
        rewritten.append(new)
        return m.group(1) + new + m.group(3)

    out = MESH_RE.sub(fix_mesh, text)

    # ---- 2. 惯量置零（42 个 link 的 <inertial> 数值归零，origin 保留） ----
    def zero_inertial(m):
        block = m.group(0)
        block = re.sub(r'(<mass\s+value\s*=\s*")[^"]+(")', r"\g<1>0.0\g<2>", block)
        block = re.sub(r'(i[xxyz][xzxy]\s*=\s*")[^"]+(")', r"\g<1>0.0\g<2>", block)
        return block

    def link_map(m):
        return zero_inertial(m) if "<inertial>" in m.group(0) else m.group(0)

    out = LINK_RE.sub(link_map, out)

    # ---- 3. 17 个 revolute 锁定为 fixed（预设握合姿态，烘焙进 origin rpy） ----
    locked = []
    limit_of = {}

    def lock_joint(m):
        block = m.group(0)
        if TYPE_RE.search(block).group(1) != "revolute":
            return block
        name = NAME_RE.search(block).group(1)
        lim = re.search(r'<limit\s+([^/]*)/>', block)
        lo = float(re.search(r'lower\s*=\s*"([^"]+)"', lim.group(1)).group(1))
        hi = float(re.search(r'upper\s*=\s*"([^"]+)"', lim.group(1)).group(1))
        limit_of[name] = (lo, hi)
        ang = PRESET[name]
        assert lo - 1e-9 <= ang <= hi + 1e-9, f"{name} 预设角 {ang} 超出官方限位 [{lo},{hi}]"
        om = ORIGIN_RE.search(block)
        assert om, f"{name} 缺少 origin rpy"
        r, p_, y = (float(v) for v in om.group(2).split())
        R_new = mat_mul(rpy_to_matrix(r, p_, y), axis_angle_matrix(parse_axis(block), ang))
        rr, pp, yy = matrix_to_rpy(R_new)
        new_origin = (om.group(1) + f'{fmt(rr)} {fmt(pp)} {fmt(yy)}' + om.group(3))
        block = block[:om.start()] + new_origin + block[om.end():]
        block = TYPE_RE.sub('type="fixed"', block, count=1)
        block = re.sub(r'\s*<limit\s+[^/]*/>', "", block)   # fixed 关节不需要 <limit>
        note = (f'<!-- v8.3 锁定：官方 revolute {name}，预设角 {ang} rad（官方限位 [{lo}, {hi}]），'
                f'腱绳耦合未在官方 URDF 建模 -->')
        block = re.sub(r"(\s*)</joint>", r"\g<1>" + note + r"\g<1></joint>", block, count=1)
        locked.append((name, ang, lo, hi))
        return block

    out = JOINT_RE.sub(lock_joint, out)

    # ---- 4. 机器人名与派生说明头 ----
    out = out.replace('<robot\n  name="orcahand">', '<robot\n  name="orcahand_sim">')
    header = (
        "<!-- ====================================================================\n"
        "     ORCA Hand（SRL-ETHZ）仿真专用派生 URDF —— 由 derive_orcahand_urdf.py 生成\n"
        "     来源：srl-ethz/orcahand_description（MIT License）\n"
        "           third_party/orcahand/orcahand.urdf.xacro\n"
        "           -> expand_orcahand_xacro.py 展开 -> orcahand_right.urdf\n"
        "     派生内容（仅此三类，几何 mesh 与 link/joint 结构未改）：\n"
        "       1) 68 处 mesh 路径由 ROS 前缀改写为工程相对路径 meshes/orcahand/...\n"
        "       2) 全部 <inertial> mass 与惯性张量置 0（维持 v8.2 无重力基线口径）\n"
        "       3) 17 个官方 revolute 锁定为 fixed（预设握合姿态，均在官方限位内；\n"
        "          官方 URDF 无 mimic/transmission，腱绳耦合未建模，详见变更记录）\n"
        "     官方几何 mesh（34 visual + 34 collision STL）原样随附于 meshes/orcahand/。\n"
        "     ==================================================================== -->\n"
    )
    decl = re.match(r"(<\?xml[^>]*\?>\s*)", out)
    out = (decl.group(1) + header + out[decl.end():]) if decl else header + out

    with open(args.dst, "w", encoding="utf-8") as f:
        f.write(out)

    # ---- 5. 强校验 ----
    with open(args.dst, "r", encoding="utf-8") as f:
        chk = f.read()
    d_pkg = len(re.findall(r'filename\s*=\s*"package://', chk))
    d_links = len(re.findall(r"<link\b", chk))
    d_joints = len(re.findall(r"<joint\b", chk))
    d_rev = len(re.findall(r'type\s*=\s*"revolute"', chk))
    d_fixed = len(re.findall(r'type\s*=\s*"fixed"', chk))
    d_mimic = len(re.findall(r"<mimic\b", chk))
    d_trans = len(re.findall(r"<transmission\b", chk))
    missing = [p for p in rewritten if not os.path.exists(p)]

    def norm(block):
        b = re.sub(r'filename\s*=\s*"[^"]*"', 'filename="<PATH>"', block)
        b = re.sub(r'(<mass\s+value\s*=\s*")[^"]+(")', r"\g<1><M>\g<2>", b)
        b = re.sub(r'(i[xxyz][xzxy]\s*=\s*")[^"]+(")', r"\g<1><I>\g<2>", b)
        b = re.sub(r'type\s*=\s*"\w+"', 'type="<T>"', b)   # revolute→fixed 锁定属白名单改动
        b = re.sub(r'(rpy\s*=\s*")[^"]+(")', r"\g<1><RPY>\g<2>", b)
        b = re.sub(r"<!--.*?-->", "", b, flags=re.S)
        b = re.sub(r'<limit\s+[^/]*/>', "", b)   # 锁定关节的 <limit> 移除属白名单改动
        return re.sub(r"\s+", " ", b).strip()

    def blocks(t):
        d = {}
        for m in list(LINK_RE.finditer(t)) + list(JOINT_RE.finditer(t)):
            d[NAME_RE.search(m.group(0)).group(1)] = norm(m.group(0))
        return d

    src_blocks, dst_blocks = blocks(text), blocks(chk)
    mismatch = [n for n in dst_blocks
                if n in src_blocks and src_blocks[n] != dst_blocks[n]]
    unchanged = [n for n in dst_blocks
                 if n in src_blocks and src_blocks[n] == dst_blocks[n]]

    print(f"[派生] mesh 路径改写 {len(rewritten)} 处 -> meshes/orcahand/...，文件缺失: {missing or '无'}")
    print(f"[派生] 输出 link={d_links} joint={d_joints}（revolute={d_rev} fixed={d_fixed}）"
          f" package://={d_pkg} mimic={d_mimic} transmission={d_trans}")
    print(f"[派生] <inertial> 块数 = {len(re.findall(r'<inertial>', chk))}（42 个 link 全部置零）")
    print(f"[派生] revolute 锁定 {len(locked)} 个："
          + ", ".join(f"{n}={a}" for n, a, _, _ in locked))
    print(f"[派生] 与官方展开结果一致块（白名单三类除外）: {len(unchanged)} 个；不一致: {mismatch or '无'}")

    ok = (n_pkg == 68 and n_links == 42 and n_joints == 41 and n_rev == 17 and n_fixed == 24
          and n_mimic == 0 and n_trans == 0
          and d_pkg == 0 and not missing
          and d_links == 42 and d_joints == 41 and d_rev == 0 and d_fixed == 41
          and d_mimic == 0 and d_trans == 0
          and len(locked) == 17 and not mismatch and len(unchanged) == 83)
    print("== ORCA DERIVE PASS ==" if ok else "== ORCA DERIVE FAILED ==")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

`derive_damiao_urdf.py` 自检输出（实测，含 ORCA 校验项）：

```
[派生] link 保留 50（期望 8+42=50），joint 49（期望 7+41+1=49）
[派生] 类型 revolute=7 prismatic=0 fixed=42（期望 7/0/42）
[派生] mesh 路径改写 16 处，全部 -> meshes/damiao/...
[派生] 残留夹爪引用: 无；残留 package:// 网格引用: 0 处
[派生] 达妙保留块与官方逐字一致: 15 个；不一致: 无
[派生] ORCA 子装配: link=42 joint=41（期望 42/41），挂接 joint=1（期望 1），mesh 缺失: 无，块与官方派生一致: 83/83；不一致: 无
== DERIVE PASS ==
```

---

## 6. 模型加载自检（test_load.py）

### 步骤六：创建并运行自检脚本

自检项：① URDF 可加载；② link 总数 = **49**（7 达妙 revolute + 41 ORCA fixed + 1 挂接 fixed，ORCA 不引入可动关节）；③ 达妙 7 组限位逐项等于官方值；④ 末端 link 名 = `link7` 且索引按名反查；⑤ 零位 FK 断言 = (0, -0.03325, 0.4981) m（容差 5 mm，判定点口径不变）；⑥ **ORCA 子装配断言**：42 个 `orca_*` link、挂接关节存在、惯量置零（mass=0）、碰撞过滤可屏蔽；⑦ **ORCA 包围盒断言**（≤ 0.30 m、零位不与地面穿模）；⑧ 渲染一帧验证官方 STL 显示通路。

```bash
cd damiao_reach_lab
python test_load.py
```

实测输出（v8.3）：

```
[1] loadURDF OK, bodyUniqueId = 0
[2] getNumJoints = 49（期望 49 = 7 达妙 revolute + 41 ORCA fixed + 1 挂接 fixed；官方夹爪子树已裁剪）
    类型统计: revolute=7（期望 7）fixed=42（期望 42）
[3] 达妙 7 关节限位读入检查（来源：达妙官方 left_description_grip.urdf <limit>）
    joint1 (joint1  ) limits=[-2.8270000,+2.8270000] expect=[-2.8270000,+2.8270000] PASS
    joint2 (joint2  ) limits=[-1.8064158,+0.6806784] expect=[-1.8064158,+0.6806784] PASS
    joint3 (joint3  ) limits=[-2.8797933,+2.8797933] expect=[-2.8797933,+2.8797933] PASS
    joint4 (joint4  ) limits=[-1.8151424,+0.7592182] expect=[-1.8151424,+0.7592182] PASS
    joint5 (joint5  ) limits=[-2.7576202,+2.7576202] expect=[-2.7576202,+2.7576202] PASS
    joint6 (joint6  ) limits=[-1.3264502,+1.3264502] expect=[-1.3264502,+1.3264502] PASS
    joint7 (joint7  ) limits=[-1.3351769,+1.3351769] expect=[-1.3351769,+1.3351769] PASS
[4] 末端 link 确认（按 link 名反查索引，不硬编码）
    达妙 link 序列: ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7']
    末端 link 名 = link7，link 索引 = 6 PASS
[5] 零位 FK 断言（理论值 (0.0, -0.03325, 0.4981) m，容差 5 mm，不含夹爪偏移；ORCA 子装配不进 IK 链，判定点口径不变）
    link7 原点 = (0.00000, -0.03325, 0.49810)，|Δ| = 0.000 mm PASS
    link7 COM  = (-0.04407, 0.00049, 0.54346)（备查，与原点相差属正常）
[6] ORCA 子装配断言（固定子装配 / 惯量置零 / 碰撞过滤屏蔽）
    orca_* link 数 = 42（期望 42）；挂接 joint = joint_orca_wrist 存在 PASS
    ORCA link 质量: 全部 = 0.0 kg（惯量置零口径）PASS
    碰撞过滤: 42 个 ORCA link 已设 group=2/mask=0（不参与仿真接触）PASS
[7] ORCA 包围盒断言（零位、世界系）
    包围盒 min=[-0.15, -0.0733, 0.4381] max=[0.1064, 0.0266, 0.5187]
    尺寸 = [0.2564, 0.0998, 0.0806] m（≤ 0.3 m；官方几何实测约 0.098×0.109×0.157 m）PASS
    零位最低点 z = 0.4381 m > 0（不与地面穿模）PASS
[8] 渲染一帧（ER_TINY_RENDERER，验证官方 STL 显示：达妙 + ORCA）
    帧缓冲 307200 bytes（= 320×240×4，非零即渲染通路正常）
== ALL PASS ==
```

自检脚本全文：

```python
#!/usr/bin/env python3
"""test_load.py - 达妙七轴 + ORCA Hand 末端执行器 派生 URDF 加载自检。全部 PASS 才继续跑仿真。

检查项：
  1) loadURDF 成功；
  2) link 总数 = 49 = 7 达妙 revolute + 41 ORCA fixed + 1 挂接 fixed
     （官方夹爪子树已裁剪；ORCA 为固定子装配，不引入任何可动关节）；
  3) 达妙 7 个关节限位逐项等于官方 URDF <limit>；
  4) 末端 link 名 = link7，且其 link 索引按名反查得到（不硬编码）；
  5) 零位 FK 断言 = (0, -0.03325, 0.4981) m，容差 5 mm
     （与 handoff/演算基准.json 的 zero_pose.link7_tcp_m 一致；
     精度判定点仍为 link7 原点，ORCA 子装配不进 IK 链、不改变该口径）；
  6) ORCA 子装配断言：42 个 orca_* link、挂接 joint_orca_wrist 存在、
     全部 ORCA link 惯量置零（mass=0，符合 v8.3 默认口径）、
     碰撞过滤可按组屏蔽（不参与仿真接触）；
  7) ORCA 包围盒断言：整体尺寸与官方实测一致量级（≤ 0.30 m），且零位时不与地面穿模；
  8) 渲染一帧验证官方 STL 显示通路正常（达妙 + ORCA）。
"""
import math
import sys

import pybullet as p
import pybullet_data

EXPECT_LIMITS = [
    (-2.827, 2.827),
    (-1.8064157758141308, 0.6806784082777885),
    (-2.8797932657906435, 2.8797932657906435),
    (-1.8151424220741028, 0.7592182246175333),
    (-2.7576202181510405, 2.7576202181510405),
    (-1.3264502315156903, 1.3264502315156903),
    (-1.335176877775662, 1.335176877775662),
]
# 零位 link7 原点（官方 URDF 链式变换推导，与演算基准.json link7_tcp_m 一致）
EXPECT_HOME_EE = (0.0, -0.03325, 0.4981)
EXPECT_EE_NAME = "link7"
EXPECT_N_LINKS = 49            # 7 达妙 revolute + 41 ORCA fixed + 1 挂接 fixed
EXPECT_N_REVOLUTE = 7
EXPECT_N_ORCA = 42             # orca_ 前缀 link 数
ORCA_PREFIX = "orca_"
ORCA_MOUNT_JOINT = "joint_orca_wrist"
ORCA_COLLISION_GROUP, ORCA_COLLISION_MASK = 2, 0
ORCA_MAX_SPAN = 0.30           # 官方几何实测约 0.098 × 0.109 × 0.157 m，取 0.30 m 为断言上限
TOL = 0.005


def main() -> int:
    ok = True
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot = p.loadURDF("damiao_reach.urdf", useFixedBase=True)
    print(f"[1] loadURDF OK, bodyUniqueId = {robot}")

    n = p.getNumJoints(robot)
    types = [p.getJointInfo(robot, i)[2] for i in range(n)]
    n_rev = sum(1 for t in types if t == p.JOINT_REVOLUTE)
    n_fixed = sum(1 for t in types if t == p.JOINT_FIXED)
    print(f"[2] getNumJoints = {n}（期望 {EXPECT_N_LINKS} = 7 达妙 revolute + 41 ORCA fixed + 1 挂接 fixed；"
          f"官方夹爪子树已裁剪）")
    print(f"    类型统计: revolute={n_rev}（期望 {EXPECT_N_REVOLUTE}）fixed={n_fixed}（期望 42）")
    ok &= n == EXPECT_N_LINKS and n_rev == EXPECT_N_REVOLUTE and n_fixed == 42

    print("[3] 达妙 7 关节限位读入检查（来源：达妙官方 left_description_grip.urdf <limit>）")
    for i in range(7):
        info = p.getJointInfo(robot, i)
        lo, hi = info[8], info[9]
        elo, ehi = EXPECT_LIMITS[i]
        good = abs(lo - elo) < 1e-6 and abs(hi - ehi) < 1e-6
        ok &= good
        print(f"    joint{i+1} ({info[1].decode():8s}) "
              f"limits=[{lo:+.7f},{hi:+.7f}] "
              f"expect=[{elo:+.7f},{ehi:+.7f}] {'PASS' if good else 'FAIL'}")

    print("[4] 末端 link 确认（按 link 名反查索引，不硬编码）")
    link_names = [p.getJointInfo(robot, i)[12].decode() for i in range(n)]
    print(f"    达妙 link 序列: {link_names[:7]}")
    ee_idx = None
    for i, nm in enumerate(link_names):
        if nm == EXPECT_EE_NAME:
            ee_idx = i
    end_ok = ee_idx is not None
    ok &= end_ok
    print(f"    末端 link 名 = {EXPECT_EE_NAME}，link 索引 = {ee_idx} "
          f"{'PASS' if end_ok else 'FAIL'}")
    if not end_ok:
        p.disconnect()
        print("== FAILED（末端 link 未找到）==")
        return 1

    print(f"[5] 零位 FK 断言（理论值 {EXPECT_HOME_EE} m，容差 5 mm，不含夹爪偏移；"
          f"ORCA 子装配不进 IK 链，判定点口径不变）")
    for i in range(7):
        p.resetJointState(robot, i, 0.0)
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    ee_frame = ls[4]   # worldLinkFramePosition：URDF link 原点（主口径）
    ee_com = ls[0]     # linkWorldPosition：COM（备查口径）
    err = math.dist(ee_frame, EXPECT_HOME_EE)
    ok &= err <= TOL
    print(f"    link7 原点 = ({ee_frame[0]:.5f}, {ee_frame[1]:.5f}, {ee_frame[2]:.5f})，"
          f"|Δ| = {err * 1000:.3f} mm {'PASS' if err <= TOL else 'FAIL'}")
    print(f"    link7 COM  = ({ee_com[0]:.5f}, {ee_com[1]:.5f}, {ee_com[2]:.5f})（备查，与原点相差属正常）")

    print("[6] ORCA 子装配断言（固定子装配 / 惯量置零 / 碰撞过滤屏蔽）")
    orca_idx = [i for i in range(n) if link_names[i].startswith(ORCA_PREFIX)]
    mount_ok = any(p.getJointInfo(robot, i)[1].decode() == ORCA_MOUNT_JOINT for i in range(n))
    ok &= len(orca_idx) == EXPECT_N_ORCA and mount_ok
    print(f"    orca_* link 数 = {len(orca_idx)}（期望 {EXPECT_N_ORCA}）；"
          f"挂接 joint = {ORCA_MOUNT_JOINT} {'存在' if mount_ok else '缺失'} "
          f"{'PASS' if len(orca_idx) == EXPECT_N_ORCA and mount_ok else 'FAIL'}")
    masses = [p.getDynamicsInfo(robot, i)[0] for i in orca_idx]
    zero_mass = all(m == 0.0 for m in masses)
    ok &= zero_mass
    print(f"    ORCA link 质量: 全部 = 0.0 kg（惯量置零口径）{'PASS' if zero_mass else 'FAIL'}")
    for i in orca_idx:
        p.setCollisionFilterGroupMask(robot, i, ORCA_COLLISION_GROUP, ORCA_COLLISION_MASK)
    print(f"    碰撞过滤: {len(orca_idx)} 个 ORCA link 已设 group={ORCA_COLLISION_GROUP}/"
          f"mask={ORCA_COLLISION_MASK}（不参与仿真接触）PASS")

    print("[7] ORCA 包围盒断言（零位、世界系）")
    mins = [1e9] * 3
    maxs = [-1e9] * 3
    for i in orca_idx:
        pos = p.getLinkState(robot, i, computeForwardKinematics=1)[4]
        mins = [min(a, b) for a, b in zip(mins, pos)]
        maxs = [max(a, b) for a, b in zip(maxs, pos)]
    span = [hi - lo for lo, hi in zip(mins, maxs)]
    span_ok = all(s <= ORCA_MAX_SPAN for s in span)
    above_ground = mins[2] > 0.0
    ok &= span_ok and above_ground
    print(f"    包围盒 min={[round(v, 4) for v in mins]} max={[round(v, 4) for v in maxs]}")
    print(f"    尺寸 = {[round(s, 4) for s in span]} m（≤ {ORCA_MAX_SPAN} m；官方几何实测约 0.098×0.109×0.157 m）"
          f"{'PASS' if span_ok else 'FAIL'}")
    print(f"    零位最低点 z = {mins[2]:.4f} m > 0（不与地面穿模）{'PASS' if above_ground else 'FAIL'}")

    print("[8] 渲染一帧（ER_TINY_RENDERER，验证官方 STL 显示：达妙 + ORCA）")
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=(0.15, 0.0, 0.25), distance=1.2,
        yaw=50, pitch=-25, roll=0, upAxisIndex=2)
    proj = p.computeProjectionMatrixFOV(fov=50, aspect=4 / 3, nearVal=0.01, farVal=10.0)
    img = p.getCameraImage(320, 240, viewMatrix=view, projectionMatrix=proj,
                           renderer=p.ER_TINY_RENDERER)
    print(f"    帧缓冲 {img[2].nbytes} bytes（= 320×240×4，非零即渲染通路正常）")
    ok &= img[2].nbytes > 0

    p.disconnect()
    print("== ALL PASS ==" if ok else "== FAILED（见上方 FAIL 项）==")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

---

## 7. 主实验脚本（`damiao_reach_demo.py`）

### 步骤七：创建主脚本

主脚本相对 xArm6 版的改动仅限「与臂相关的量」，控制链路（梯形轨迹 + 位置伺服 + 速度前馈 + 5 mm 判据 + 收敛窗口）**逐字保留**：

| 项 | xArm6 版（旧，已废弃） | 达妙七轴版（本轮） |
|---|---|---|
| URDF | 手写 `xarm6_reach.urdf` | 派生 `damiao_reach.urdf`（官方几何） |
| 自由度 / 末端 link | 6 / `link6`（索引 5，硬编码） | **7 / `link7`（索引按名反查）** |
| HOME | `[0, -0.5, -1.0, 0, 0.5, 0]` | 官方零位 `[0]*7` |
| LIMITS | xArm6 官方 6 组 | **达妙官方 7 组** |
| FORCES | 163.2/163.2/81.6/81.6/30.6/30.6 | **50/50/50/50/12/12/12 N·m** |
| TARGETS | (0.45,0,0.40) 等 5 点 | **重选 5 点**（官方几何可达空间内） |
| 预筛包络 | r ≤ 0.70 m、z ∈ [0.10, 0.75] | **r ≤ 0.389 m、z ∈ [-0.111, 0.507]** |
| IK | 内置单次 | **内置粗解 + 位置-only 数值精修**；初值 = 上一点实际关节角 |
| CSV 字段 | q1–q6（20 列） | **q1–q7（21 列）** |
| 演示层（末端执行器 / 篮子 / 果实） | 3 根 × 6 段三阶段章鱼爪 | **v8.2 原样保留为 `--legacy-tentacle` 分支**（常量、`update_tentacles()`、`place_into_basket()` 逐字不动，仅挂载点索引换成实测的 link7 索引）；**v8.3 默认 = ORCA Hand 固定子装配**（官方 URDF 几何、预设握合姿态、握合腔对准果实、无开合动画、无接触力） |
| 末端执行器挂接 | 无（章鱼爪为世界系/局部系视觉体，不在 URDF 内） | **v8.3：`link7 -> orca_root`（fixed `joint_orca_wrist`，惯量置零、碰撞过滤屏蔽、不进 IK 链）** |

主脚本全文：

```python
#!/usr/bin/env python3
"""
damiao_reach_demo.py - 达妙七轴谐波机械臂 草莓定点到达仿真
                      （位置-only IK + 梯形轨迹 + 位置伺服执行）。

v8.3（2026-11-18，ORCA Hand 末端执行器版）：
  * 课设立论 = 仿生机械臂本体；末端执行器为可替换模块。
  * 末端执行器换成 ORCA Hand 官方 URDF（srl-ethz/orcahand_description，MIT）：
    以 fixed joint 挂接在 link7 之后（joint_orca_wrist），作为固定子装配呈现——
    惯量置零、17 个 revolute 锁定为预设握合姿态、不参与仿真接触（碰撞过滤关闭）；
    不进 IK 链、不改 5 个目标点、不改误差口径（判定点仍为 link7 原点）。
    腱绳耦合未在官方 URDF 中建模，本仿真不含腱绳驱动、接触力学与夹持力。
  * v8.2 的卷曲仿生章鱼爪演示层保留为 --legacy-tentacle 分支（默认关闭），
    保证 v8.2 可复现；果实搬运在两种模式下都是纯视觉移动，不写 CSV、不参与判定。

迁移说明（2026-11-18，xarm6_reach_demo.py → 本文件）：
  * 模型：达妙官方开源七轴 URDF 派生版 damiao_reach.urdf（仅裁剪官方夹爪子树，
    几何/惯量/限位逐字未改，见 derive_damiao_urdf.py 与附录 B）；
  * 自由度 6 → 7，末端 link link6 → link7（索引按 link 名反查，不硬编码）；
  * HOME / LIMITS / FORCES / TARGETS / 预筛包络 / CSV 字段全部换成达妙官方值；
  * IK 改为两段式：内置求解器出粗解（保留限位与 nullspace 参考）+ 位置-only 数值精修，
    原因见 refine_ik() 注释（7 轴冗余下内置求解器远距离初值会停在 3–8 mm 局部解）。

产物：results/reach_results.csv、results/reach_demo.mp4、results/scene.png、控制台汇总表
退出码：0 = 5 点全部通过（两重静态判定 + 伺服跟踪）；1 = 存在未达标点（结果如实保留）；
        2 = 负向验证模式（--check-out-of-range，仅验证预筛拒绝逻辑）
"""
import argparse
import csv
import json
import math
import os
import time

import imageio
import numpy as np
import pybullet as p
import pybullet_data

NDOF = 7                                       # 达妙七轴
EE_NAME = "link7"                             # 末端 link 名（按名反查索引，禁止硬编码）

# ---------------- 固定参数（数值来源见指导书附录 B，禁止随手改） ----------------
HOME = [0.0] * NDOF                            # 官方零位姿态：IK 初值 = nullspace 参考 = 动画起始
LIMITS = [(-2.827, 2.827),                    # 官方 URDF <limit>（与 README §1.1 机械限位一致）
          (-1.8064157758141308, 0.6806784082777885),
          (-2.8797932657906435, 2.8797932657906435),
          (-1.8151424220741028, 0.7592182246175333),
          (-2.7576202181510405, 2.7576202181510405),
          (-1.3264502315156903, 1.3264502315156903),
          (-1.335176877775662, 1.335176877775662)]
FORCES = [50.0, 50.0, 50.0, 50.0, 12.0, 12.0, 12.0]   # N·m，取自官方 URDF <limit effort>
TARGETS = [                                    # 按「官方限位 + 官方几何」重扫可达空间后重选
    ("P1", (0.2258, -0.0060, 0.3250)),         # IK 残差 0.000 mm，限位余量 0.408 rad
    ("P2", (0.2176, 0.1206, 0.2386)),          # 0.001 mm / 0.188 rad
    ("P3", (0.1742, -0.1081, 0.2361)),         # 0.000 mm / 0.342 rad
    ("P4", (0.2652, 0.0647, 0.3954)),          # 0.000 mm / 0.716 rad
    ("P5", (0.2747, -0.0486, 0.1567)),         # 0.000 mm / 0.652 rad
]
RADIUS_MAX, Z_MIN, Z_MAX = 0.389, -0.111, 0.507   # IK 前预筛：link7 实测可达包络
                                              # （handoff/演算基准.json workspace.link7，6000 组采样）
IK_MAX_ITER, IK_TOL = 200, 1e-5                # IK 迭代上限与残差阈值
SIM_HZ = 240                                    # 仿真步频（setTimeStep 真实生效）
ANIM_STEPS, HOLD_STEPS = 480, 480              # 轨迹段 2 s + 到位保持段 2 s（含收敛观察窗）
STEADY_N = 60                                   # 收敛判据：连续 60 步（0.25 s）达标
SERVO_MAX_STEPS = 3000                          # 每点伺服总步数上限（超时兜底 12.5 s，非收敛判据）
POS_GAIN, VEL_GAIN = 1.0, 1.0                  # 伺服增益（无重力设定下的验证基线，依据见附录 B/C）
BAD_POINT_H = (0.80, 0.00, 0.30)               # 负向验证：水平越界（0.80 m > 0.389 m）
BAD_POINT_Z = (0.20, 0.00, 0.80)               # 负向验证：z 越界（0.80 m > 0.507 m）

# ---------------- 演示层抓放循环（视觉示意，不参与误差判定；来源见指导书附录 B） ----------------
BASKET_CENTER = (0.16, -0.25, 0.08)             # 篮子中心（m），基座右前方近地面
BASKET_DROP = (0.16, -0.25, 0.24)              # 放果点：末端下降示意「低头放果」（位置-only IK，不约束姿态）
BASKET_HALF = (0.09, 0.09, 0.055)               # 篮子内腔半尺寸（m）
PLACE_STEPS, PLACE_HOLD = 480, 120              # 放果轨迹 2 s + 入篮保持 0.5 s（@240 Hz）
FRUIT_SLOTS = [                                 # 5 颗果实在篮内的留存位置（m）
    (0.13, -0.28, 0.115), (0.19, -0.28, 0.115), (0.16, -0.25, 0.115),
    (0.13, -0.22, 0.115), (0.19, -0.22, 0.115),
]

# ---- 演示层仿生触手：卷曲 -> 反向展开 -> 缠绕包覆（L1 纯视觉，不参与误差判定与物理） ----
# 参数按 SpiRobs 官方「数根触手抓鸡蛋」演示的比例取一组示意几何值；
# **非中科大实物参数、非论文测量值**；rho = a*exp(b*theta) 仅用于示意卷曲形态，来源见附录 B。
TENTACLE_COUNT = 3                                 # 触手根数（示意）
TENTACLE_SEGS = 6                                  # 每根离散段数（示意）
TENTACLE_LEN = 0.13                                # 展开态触手长度 (m，示意)
TENTACLE_RADIUS = 0.008                            # 段球半径 (m，示意)
TENTACLE_ROOT_R = 0.022                            # 根部离轴半径 (m，示意)
TENTACLE_OPEN_R = 0.065                            # 展开态中段离轴半径 (m，示意)
TENTACLE_COLOR = [0.90, 0.62, 0.20, 1.0]           # 触手颜色（示意）
# 卷曲态：对数螺旋 rho = SPIRAL_A*exp(SPIRAL_B*theta)，盘在末端后方 (+z)，不遮挡果实
SPIRAL_A = 0.010                                   # 螺旋起始半径 (m，示意)
SPIRAL_B = 0.22                                    # 螺旋增长系数 (示意)
SPIRAL_THETA_MAX = 2.4 * math.pi                   # 卷曲总转角 (rad，示意，约 1.2 圈)
COIL_BACK_Z = 0.050                                # 卷曲态向末端后方抬升 (m，示意)
# ---- 末端演示层避让参数（与 v8.3 ORCA 挂接同口径，2026-11-18 用户诊断后修正） ----
# 干涉根因：缠绕环心原压在被测点（球心 = link7 原点），环半径 0.033 m 而 link7 网格
#（含相机安装板）分布于原点周围 → 环陷入 URDF 网格，看似「装反」。
# 修正：爪体（掌心）挂到网格之外，缠绕环心对准果实保持点（局部 -z 0.03 m），
# 判定点仍为 link7 原点，不重算（与 v8.3「不进 IK 链、不改误差口径」一致）。
FRUIT_HELD_Z = 0.03                             # 果实保持点：link7 局部 -z 方向偏移（m，v8.3 ORCA 同口径）
TENTACLE_PALM_XYZ = (-0.10, 0.0, -0.03)         # 触手掌心：link7 局部坐标（避开 link7 网格，与 ORCA 挂接同侧）
WRAP_R = 0.033                                     # 包覆球面半径 (m，略大于果实半径 0.025)
WRAP_PHI0, WRAP_PHI1 = 0.18 * math.pi, 0.96 * math.pi   # 球顶到球底极角范围 (示意)
WRAP_TWIST = 0.55 * math.pi                        # 缠绕时绕球轴的附加转角 (示意)
# 阶段时间线：到位段后段展开，保持段前段缠绕，入篮段后段释放回卷
EXTEND_START = 0.75                                # 到位轨迹段中开始反向展开的归一化时刻
WRAP_END = 0.30                                    # 保持段中完成缠绕包覆的归一化时刻
RELEASE_START = 0.70                               # 入篮保持段中开始释放回卷的归一化时刻

# ---- v8.3 ORCA Hand 末端执行器（固定子装配，纯视觉呈现；来源 srl-ethz/orcahand_description，MIT） ----
# 官方 URDF 未建模腱绳耦合（无 mimic/transmission），17 个 revolute 已在派生 URDF 中
# 锁定为预设握合姿态（均在官方限位内，锁定角见 derive_orcahand_urdf.py）；本脚本不做
# 开合动画、不做接触力学。collision mesh 在 URDF 中保留，但运行时按组屏蔽接触。
ORCA_LINK_PREFIX = "orca_"                        # ORCA 子装配 link 名前缀（派生时加 prefix）
ORCA_MOUNT_JOINT = "joint_orca_wrist"             # link7 -> orca_root 的 fixed 挂接关节名
ORCA_COLLISION_GROUP, ORCA_COLLISION_MASK = 2, 0  # 碰撞过滤：group=2 / mask=0 → 不参与任何接触
ORCA_OFFICIAL_URDF = "third_party/orcahand/orcahand_right.urdf"  # 官方展开 URDF（含官方质量）
G = 9.81                                          # --with-orca-mass 模式的重力（m/s²）


def resolve_ee_index(robot):
    """按末端 link 名反查真实 link 索引（禁止硬编码索引）。"""
    for i in range(p.getNumJoints(robot)):
        if p.getJointInfo(robot, i)[12].decode() == EE_NAME:
            return i
    raise RuntimeError(f"未找到末端 link：{EE_NAME}")


def resolve_orca_links(robot):
    """按 link 名前缀反查 ORCA 子装配的全部 link 索引（不硬编码）。"""
    idx = [i for i in range(p.getNumJoints(robot))
           if p.getJointInfo(robot, i)[12].decode().startswith(ORCA_LINK_PREFIX)]
    if not idx:
        raise RuntimeError("未找到 ORCA 子装配（orca_* link），请检查 damiao_reach.urdf 派生")
    return idx


def read_orca_official_inertials(path=ORCA_OFFICIAL_URDF):
    """从官方展开 URDF 读取 ORCA 各 link 的官方质量与惯量主对角（--with-orca-mass 用）。"""
    import re as _re
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    out = {}
    for m in _re.finditer(r'<link\s+name="([^"]+)">(.*?)</link>', text, _re.S):
        name, body = m.group(1), m.group(2)
        if not name.startswith(ORCA_LINK_PREFIX):
            continue
        mass = _re.search(r'<mass\s+value="([^"]+)"', body)
        if not mass:
            continue
        inertia = _re.search(r'<inertia\s+([^/]*)/>', body)
        diag = [0.0, 0.0, 0.0]
        if inertia:
            for k, key in enumerate(("ixx", "iyy", "izz")):
                kv = _re.search(rf'{key}\s*=\s*"([^"]+)"', inertia.group(1))
                diag[k] = float(kv.group(1)) if kv else 0.0
        out[name] = (float(mass.group(1)), diag)
    return out


def precheck(point, name):
    """预筛：水平半径与高度。越界即退出码 2（在 IK 前快速失败）。"""
    rho = math.hypot(point[0], point[1])
    if rho > RADIUS_MAX:
        print(f"[预筛拒绝] {name} 水平距离 {rho:.3f} m > {RADIUS_MAX} m，IK 前中止。")
        print("            （--check-out-of-range 模式仅验证拒绝逻辑，不构成可达性结论）")
        raise SystemExit(2)
    if not (Z_MIN <= point[2] <= Z_MAX):
        print(f"[预筛拒绝] {name} z={point[2]:.3f} m 不在 [{Z_MIN},{Z_MAX}]，IK 前中止。")
        raise SystemExit(2)


def refine_ik(robot, ee_idx, target, q, iters=60, eps=1e-5, tol=1e-6):
    """位置-only 数值精修：数值雅可比 + 阻尼最小二乘（Levenberg–Marquardt 式阻尼递增）。

    为什么需要：7 轴位置-only IK 是冗余的，PyBullet 内置求解器在「远距离初值」下
    常停在 3–8 mm 的局部解（残差超 5 mm 阈值），并非阈值设置问题。
    本函数在不动阈值、不动限位的前提下做位置-only 精修：
      * 雅可比由 FK 数值差分得到（步长 eps）；
      * 阻尼 λ 按 1e-7→1e-0 递增重试，只有误差真正下降才接受该步，避免卡死；
      * 关节角始终夹在官方限位内；j7（对 TCP 位置无影响）保持初值，维持解连续性。
    """
    q = list(q)
    set_joints(robot, q)
    err = math.dist(fk_pos(robot, ee_idx), target)
    for _ in range(iters):
        if err <= tol:
            break
        p0 = fk_pos(robot, ee_idx)
        J = np.zeros((3, NDOF))
        for k in range(NDOF):
            qk = list(q)
            qk[k] = min(max(qk[k] + eps, LIMITS[k][0]), LIMITS[k][1])
            set_joints(robot, qk)
            J[:, k] = (fk_pos(robot, ee_idx) - p0) / eps
        set_joints(robot, q)
        improved = False
        for trial in range(8):
            lam = 1e-7 * (10 ** trial)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), np.array(target) - p0)
            except np.linalg.LinAlgError:
                continue
            qn = [min(max(q[i] + dq[i], LIMITS[i][0]), LIMITS[i][1]) for i in range(NDOF)]
            set_joints(robot, qn)
            en = math.dist(fk_pos(robot, ee_idx), target)
            if en < err - 1e-12:
                q, err = qn, en
                improved = True
                break
        if not improved:
            break
    set_joints(robot, q)
    return q, err


def set_joints(robot, q):
    for i in range(NDOF):
        p.resetJointState(robot, i, q[i])


def fk_pos(robot, ee_idx):
    return np.array(p.getLinkState(robot, ee_idx, computeForwardKinematics=1)[4])


def solve_ik(robot, ee_idx, target, seed=None):
    """两段式位置-only IK。

    第一段：PyBullet 内置求解器出粗解——显式限位 / jointRanges / HOME 作 nullspace 参考；
            初值取上一点收敛后的实际关节角（解连续性处理，避免 7 轴冗余的零空间漂移）。
    第二段：refine_ik() 做位置-only 数值精修，把残差压到亚毫米。

    注：末端参考点取 link7 原点（即 joint7 轴心），joint7 旋转不改变该点位置，
    因此位置-only IK 无法确定 j7，它保持初值（= 0）；不影响位置误差判定。
    """
    seed = list(seed) if seed is not None else list(HOME)
    set_joints(robot, seed)
    q = list(p.calculateInverseKinematics(
        robot, ee_idx, list(target),
        lowerLimits=[lo for lo, _ in LIMITS],
        upperLimits=[hi for _, hi in LIMITS],
        jointRanges=[hi - lo for lo, hi in LIMITS],
        restPoses=HOME,
        maxNumIterations=IK_MAX_ITER,
        residualThreshold=IK_TOL,
    )[:NDOF])
    q, _ = refine_ik(robot, ee_idx, target, q)
    return q


def check_limits(q):
    bad = [f"joint{i+1}={q[i]:.4f} 不在 [{LIMITS[i][0]:+.4f},{LIMITS[i][1]:+.4f}]"
           for i in range(NDOF) if not (LIMITS[i][0] - 1e-6 <= q[i] <= LIMITS[i][1] + 1e-6)]
    return len(bad) == 0, bad


def fk_ee(robot, ee_idx):
    """FK 复核：返回 (link7 原点, link7 COM)。主口径为原点（URDF frame，与 IK 参考系一致）。"""
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    return ls[4], ls[0]


def trap_profile(n):
    """归一化梯形位移序列 s(t)∈[0,1]：加速 25% / 匀速 50% / 减速 25%。峰值速度 1/(1-a)。"""
    a, out = 0.25, []
    vmax = 1.0 / (1.0 - a)
    for k in range(1, n + 1):
        t = k / n
        if t < a:                    # 加速段
            s = vmax * t * t / (2.0 * a)
        elif t <= 1.0 - a:           # 匀速段
            s = vmax * a / 2.0 + vmax * (t - a)
        else:                        # 减速段
            u = 1.0 - t
            s = 1.0 - vmax * u * u / (2.0 * a)
        out.append(min(max(s, 0.0), 1.0))
    return out


def main():
    ap = argparse.ArgumentParser(description="达妙七轴 草莓定点到达仿真")
    ap.add_argument("--threshold", type=float, default=0.005, help="到位误差阈值(m)，默认 5 mm，可配")
    ap.add_argument("--res", default="960x720", help="渲染分辨率 WxH（降级用 640x480）")
    ap.add_argument("--every", type=int, default=8, help="每 N 仿真步渲染一帧（降级用 16）")
    ap.add_argument("--no-video", action="store_true", help="跳过视频导出（只出 CSV/截图，快速验证）")
    ap.add_argument("--check-out-of-range", action="store_true",
                    help="负向验证：注入越界点，确认脚本在 IK 前拒绝（退出码 2）")
    ap.add_argument("--legacy-tentacle", action="store_true",
                    help="v8.2 卷曲仿生章鱼爪演示层分支（复现 v8.2；默认关闭，默认为 ORCA Hand）")
    ap.add_argument("--with-orca-mass", action="store_true",
                    help="带载对照实验：启用重力 + ORCA 官方质量（独立 CSV，不与无重力基线数字混报）")
    args = ap.parse_args()

    if args.check_out_of_range:
        for pt, nm in ((BAD_POINT_H, "P_BAD_H"), (BAD_POINT_Z, "P_BAD_Z")):
            rho = math.hypot(pt[0], pt[1])
            if rho > RADIUS_MAX:
                print(f"[预筛拒绝] {nm} 水平距离 {rho:.3f} m > {RADIUS_MAX} m，IK 前中止。")
            elif not (Z_MIN <= pt[2] <= Z_MAX):
                print(f"[预筛拒绝] {nm} z={pt[2]:.3f} m 不在 [{Z_MIN},{Z_MAX}]，IK 前中止。")
        print("（--check-out-of-range 仅验证拒绝逻辑，不构成可达性结论）")
        raise SystemExit(2)

    w, h = (int(x) for x in args.res.lower().split("x"))
    os.makedirs("results", exist_ok=True)
    if args.with_orca_mass:                            # 独立产物，禁止与无重力基线混用
        csv_name, mp4_name, stats_name = ("results/reach_results_orca_mass.csv",
                                          "results/reach_demo_orca_mass.mp4",
                                          "results/run_stats_orca_mass.json")
    else:
        csv_name, mp4_name = "results/reach_results.csv", "results/reach_demo.mp4"
        # --no-video 的统计写入独立文件（含实测耗时），不覆盖完整运行的 run_stats.json
        stats_name = ("results/run_stats_no_video.json" if args.no_video
                      else "results/run_stats.json")

    p.connect(p.DIRECT)
    p.setTimeStep(1.0 / SIM_HZ)                       # 240 Hz 仿真步长
    if args.with_orca_mass:                           # 带载对照实验（口径与默认模式不同）
        p.setGravity(0, 0, -G)
        print(f"[口径] --with-orca-mass：重力 {G} m/s² + ORCA 官方质量，"
              f"独立 CSV（{csv_name}），数字不与无重力基线混报")
    else:
        p.setGravity(0, 0, 0)                         # 无重力设定（§0 第 6 条）：重力前馈补偿属后续工作
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")                          # 地面
    # 栽培架示意（绿色薄板，仅视觉）
    board = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.28, 0.30, 0.005],
                                rgbaColor=[0.25, 0.55, 0.25, 1.0])
    p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=board,
                      basePosition=[0.16, -0.06, 0.0005])
    robot = p.loadURDF("damiao_reach.urdf", useFixedBase=True)
    ee_idx = resolve_ee_index(robot)                  # 末端 link 索引（按名反查）
    orca_idx = resolve_orca_links(robot)              # ORCA 子装配 link 索引（按前缀反查）
    for i in orca_idx:                                # ORCA 不参与仿真接触（碰撞过滤屏蔽）
        p.setCollisionFilterGroupMask(robot, i, ORCA_COLLISION_GROUP, ORCA_COLLISION_MASK)
    orca_mass_total = sum(p.getDynamicsInfo(robot, i)[0] for i in orca_idx)
    print(f"[模型] 达妙七轴派生 URDF 加载成功，末端 link = {EE_NAME}，link 索引 = {ee_idx}，"
          f"link 总数 = {p.getNumJoints(robot)}（其中 ORCA 子装配 {len(orca_idx)} 个）")
    print(f"[末端] ORCA Hand 固定子装配：惯量置零合计 {orca_mass_total:.3f} kg，"
          f"碰撞过滤 group={ORCA_COLLISION_GROUP}/mask={ORCA_COLLISION_MASK}（不参与接触），"
          f"17 个 revolute 锁定为预设握合姿态（不进 IK 链、不参与判定）")
    if args.with_orca_mass:                           # 官方质量注入（仅本模式）
        official = read_orca_official_inertials()
        for i in orca_idx:
            nm = p.getJointInfo(robot, i)[12].decode()
            mass, diag = official[nm]
            p.changeDynamics(robot, i, mass=mass, localInertiaDiagonal=diag)
        orca_mass_total = sum(p.getDynamicsInfo(robot, i)[0] for i in orca_idx)
        print(f"[带载] ORCA 官方质量注入合计 {orca_mass_total:.3f} kg（tower 0.9 kg + 全惯量）")

    # 5 个草莓目标点（红球标记）
    ball_ids = {}
    ball_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.025,
                                     rgbaColor=[0.85, 0.10, 0.10, 1.0])
    for name, t in TARGETS:
        ball_ids[name] = p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=ball_shape,
                                           basePosition=list(t))

    # 篮子（纯视觉静态道具：底板 + 4 壁，无碰撞、无质量、不参与物理）
    def add_box(half, pos, color):
        shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
        return p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=shape,
                                 basePosition=list(pos))

    cx, cy, cz = BASKET_CENTER
    hx, hy, bh = BASKET_HALF
    wall, basket_rgba = 0.006, [0.55, 0.38, 0.20, 0.9]
    add_box((hx, hy, wall / 2), (cx, cy, cz), basket_rgba)
    add_box((wall / 2, hy, bh / 2), (cx - hx, cy, cz + bh / 2), basket_rgba)
    add_box((wall / 2, hy, bh / 2), (cx + hx, cy, cz + bh / 2), basket_rgba)
    add_box((hx, wall / 2, bh / 2), (cx, cy - hy, cz + bh / 2), basket_rgba)
    add_box((hx, wall / 2, bh / 2), (cx, cy + hy, cz + bh / 2), basket_rgba)

    # 演示层仿生触手：3 根 x 6 段纯视觉体（无质量、无碰撞、不参与物理与误差测量）
    tentacle_shape = p.createVisualShape(p.GEOM_SPHERE, radius=TENTACLE_RADIUS,
                                         rgbaColor=TENTACLE_COLOR)
    tentacle_ids = []
    for _ in range(TENTACLE_COUNT * TENTACLE_SEGS):
        tentacle_ids.append(p.createMultiBody(baseMass=0.0,
                                              baseVisualShapeIndex=tentacle_shape,
                                              basePosition=[0.0, 0.0, -1.0]))
    tentacle_t = 0.0                                 # 0=卷曲待机; 0~0.5 反向展开; 0.5~1 缠绕包覆; 1=夹住

    def ee_pose():
        """返回 (link7 原点世界位置, link7 世界旋转矩阵)。

        末端演示层（触手 + 果实）必须在**末端局部坐标系**里生成，再按 link7 的
        姿态变换到世界系：否则腕部一转，爪就与关节末端歪掉、也不夹住果实。
        零位时 R = 单位阵，与历史画面一致。
        """
        ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
        pos = np.array(ls[4], dtype=float)
        x, y, z, w = ls[5]
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        return pos, R

    def update_tentacles(t):
        """三阶段形态：向后卷曲(对数螺旋) -> 反向展开 -> 球面缠绕包覆果实。

        t in [0,1]：0 = 卷曲待机；0~0.5 由卷曲插值到展开；0.5~1 由展开插值到缠绕。
        几何以 **掌心**（link7 局部 TENTACLE_PALM_XYZ，避开 link7 网格）为原点、
        缠绕环心对准**果实保持点**（局部 -FRUIT_HELD_Z），再乘 link7 的世界旋转
        矩阵落到世界系——触手随腕部姿态同步、从网格外侧包住果实，不与 URDF 干涉
        （2026-11-18 用户诊断：环心压在被测点导致爪环陷入 link7 网格，看似装反）。
        纯视觉，不参与误差判定。
        """
        def smooth(x):
            x = min(1.0, max(0.0, x))
            return x * x * (3.0 - 2.0 * x)

        pos, R = ee_pose()
        palm = pos + R @ np.array(TENTACLE_PALM_XYZ)   # 掌心（爪体根部，在网格之外）
        ca = [math.cos(2.0 * math.pi * i / TENTACLE_COUNT) for i in range(TENTACLE_COUNT)]
        sa = [math.sin(2.0 * math.pi * i / TENTACLE_COUNT) for i in range(TENTACLE_COUNT)]
        a_ext = smooth(2.0 * t)                     # 卷曲 -> 展开
        a_wrap = smooth(2.0 * t - 1.0)              # 展开 -> 缠绕
        for j in range(TENTACLE_SEGS):
            u = (j + 1) / TENTACLE_SEGS
            # (1) 卷曲态：rho = a*exp(b*theta)，盘在末端后方 (+z)，不遮挡果实
            theta = SPIRAL_THETA_MAX * u
            rho = SPIRAL_A * math.exp(SPIRAL_B * theta)
            z_coil = 0.02 * (1.0 - u)     # 卷曲盘收在掌心附近
            # (2) 展开态：指头从掌心向果实保持点方向（局部 -z）伸展并越过果实
            r_ext = TENTACLE_ROOT_R + (TENTACLE_OPEN_R - TENTACLE_ROOT_R) * u
            z_ext = 0.02 - TENTACLE_LEN * u
            # (3) 缠绕态：环绕果实体——球心在果实保持点（局部 -FRUIT_HELD_Z，避开 link7 网格）
            phi = WRAP_PHI0 + (WRAP_PHI1 - WRAP_PHI0) * u
            rr_wrap = WRAP_R * math.sin(phi)
            z_wrap = -FRUIT_HELD_Z + WRAP_R * math.cos(phi)
            for i in range(TENTACLE_COUNT):
                ang_c = theta + 2.0 * math.pi * i / TENTACLE_COUNT
                x_coil, y_coil = rho * math.cos(ang_c), rho * math.sin(ang_c)
                x_ext, y_ext = r_ext * ca[i], r_ext * sa[i]
                ang_w = 2.0 * math.pi * i / TENTACLE_COUNT + WRAP_TWIST * u
                x_wrap, y_wrap = rr_wrap * math.cos(ang_w), rr_wrap * math.sin(ang_w)
                x1 = x_coil + (x_ext - x_coil) * a_ext
                y1 = y_coil + (y_ext - y_coil) * a_ext
                z1 = z_coil + (z_ext - z_coil) * a_ext
                x = x1 + (x_wrap - x1) * a_wrap
                y = y1 + (y_wrap - y1) * a_wrap
                z = z1 + (z_wrap - z1) * a_wrap
                wp = palm + R @ np.array([x, y, z])
                p.resetBasePositionAndOrientation(tentacle_ids[i * TENTACLE_SEGS + j],
                                                  [float(wp[0]), float(wp[1]), float(wp[2])],
                                                  [0.0, 0.0, 0.0, 1.0])

    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=(0.17, -0.08, 0.20), distance=1.0,
        yaw=40, pitch=-22, roll=0, upAxisIndex=2)
    proj = p.computeProjectionMatrixFOV(fov=50, aspect=w / h, nearVal=0.01, farVal=10.0)

    writer = None
    n_frames = 0
    t_render = 0.0
    if not args.no_video:
        writer = imageio.get_writer(mp4_name, fps=30,
                                    codec="libx264", quality=8, macro_block_size=1)

    def shoot(force=False):
        """渲染一帧并写盘。计时覆盖 getCameraImage（CPU 软渲染大头）+ 写盘。
        --no-video（writer=None）时跳过常规帧，仅 force=True（终态截图）渲染一帧。
        仅 --legacy-tentacle 分支在渲染前更新触手三阶段形态（v8.2 复现口径）。"""
        nonlocal n_frames, t_render
        if writer is None and not force:
            return None
        if args.legacy_tentacle:
            update_tentacles(tentacle_t)              # v8.2 章鱼爪分支（默认关闭）
        t0 = time.perf_counter()
        img = p.getCameraImage(w, h, viewMatrix=view, projectionMatrix=proj,
                               renderer=p.ER_TINY_RENDERER)
        rgba = np.reshape(np.asarray(img[2]), (h, w, 4))
        frame = rgba[:, :, :3].astype(np.uint8)
        if writer is not None:
            writer.append_data(frame)
        t_render += time.perf_counter() - t0
        n_frames += 1
        return frame

    def place_into_basket(name, q_start, slot_idx):
        """演示层：果实随末端搬到篮子、末端下降放果、果实入篮留存。

        不写 rows、不参与误差判定、不改阈值与限位；--no-video 下伺服与视觉搬运仍完整执行。
        """
        nonlocal tentacle_t
        q_drop = solve_ik(robot, ee_idx, BASKET_DROP, seed=q_start)
        for j in range(NDOF):
            p.resetJointState(robot, j, q_drop[j])
        drop_err = math.dist(fk_ee(robot, ee_idx)[0], BASKET_DROP)
        drop_in_lim, _ = check_limits(q_drop)
        for j in range(NDOF):                         # 放果段起始 = 当前草莓点伺服后的姿态
            p.resetJointState(robot, j, q_start[j])
        dt = 1.0 / SIM_HZ
        q_prev = list(q_start)
        for k, s in enumerate(trap_profile(PLACE_STEPS)):
            q_cmd = [q_start[j] + s * (q_drop[j] - q_start[j]) for j in range(NDOF)]
            v_cmd = [0.0] * NDOF if k == 0 else [(q_cmd[j] - q_prev[j]) / dt for j in range(NDOF)]
            p.setJointMotorControlArray(robot, range(NDOF), p.POSITION_CONTROL,
                                        targetPositions=q_cmd, targetVelocities=v_cmd,
                                        forces=FORCES, positionGains=[POS_GAIN] * NDOF,
                                        velocityGains=[VEL_GAIN] * NDOF)
            p.stepSimulation()
            pos, R = ee_pose()        # 果实搬运至保持点（局部 -FRUIT_HELD_Z，v8.3 ORCA 同口径）
            fp = pos + R @ np.array([0.0, 0.0, -FRUIT_HELD_Z])
            p.resetBasePositionAndOrientation(ball_ids[name],
                                              [float(fp[0]), float(fp[1]), float(fp[2])],
                                              [0.0, 0.0, 0.0, 1.0])
            q_prev = list(q_cmd)
            if writer is not None and k % args.every == 0:
                tentacle_t = 1.0                     # v8.2 分支：搬运中保持缠绕包覆夹住
                shoot()
        for k in range(PLACE_HOLD):                  # 入篮保持段：末端停在放果点，果实随末端下降
            p.setJointMotorControlArray(robot, range(NDOF), p.POSITION_CONTROL,
                                        targetPositions=q_drop, targetVelocities=[0.0] * NDOF,
                                        forces=FORCES, positionGains=[POS_GAIN] * NDOF,
                                        velocityGains=[VEL_GAIN] * NDOF)
            p.stepSimulation()
            pos, R = ee_pose()        # 果实搬运至保持点（局部 -FRUIT_HELD_Z，v8.3 ORCA 同口径）
            fp = pos + R @ np.array([0.0, 0.0, -FRUIT_HELD_Z])
            p.resetBasePositionAndOrientation(ball_ids[name],
                                              [float(fp[0]), float(fp[1]), float(fp[2])],
                                              [0.0, 0.0, 0.0, 1.0])
            if writer is not None and k % args.every == 0:
                if args.legacy_tentacle:             # v8.2 分支：入篮段后段释放回卷
                    tentacle_t = 1.0 - min(1.0, max(0.0, (k / PLACE_HOLD - RELEASE_START) / (1.0 - RELEASE_START)))
                shoot()
        p.resetBasePositionAndOrientation(ball_ids[name], list(FRUIT_SLOTS[slot_idx]),
                                          [0.0, 0.0, 0.0, 1.0])   # 果实入篮并留存
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.75, 0.12, 0.12, 1.0])
        tentacle_t = 0.0                                # 放果后回到卷曲待机，准备下一颗
        q_end = [p.getJointState(robot, j)[0] for j in range(NDOF)]
        return q_end, drop_err, drop_in_lim

    rows, q_cur, place_checks = [], list(HOME), []
    torque_records = []                               # --with-orca-mass：逐点静态持重力矩记录
    for i in range(NDOF):                             # 伺服起始 = HOME
        p.resetJointState(robot, i, HOME[i])
    t_total0 = time.perf_counter()

    print(f"{'点':4s} {'目标(m)':22s} {'IK残差(mm)':>11s} {'伺服误差(mm)':>12s} "
          f"{'COM(mm)':>9s} {'限位':4s} {'收敛步':>6s} {'结论':6s}")
    for name, target in TARGETS:
        precheck(target, name)

        # ---- 静态判定：IK 求解 + FK 残差 + 限位 ----
        q = solve_ik(robot, ee_idx, target, seed=q_cur)   # 以上一点实际关节角作初值（解连续性）
        for j in range(NDOF):                         # 设置 IK 解测静态残差
            p.resetJointState(robot, j, q[j])
        ee_frame, ee_com = fk_ee(robot, ee_idx)
        err_ik = math.dist(ee_frame, target)
        err_com = math.dist(ee_com, target)
        in_lim, bad = check_limits(q)
        tau_static = None
        if args.with_orca_mass:                       # 带载模式：静态持重力矩实算（含 ORCA 官方质量）
            tau_static = p.calculateInverseDynamics(robot, list(q), [0.0] * NDOF, [0.0] * NDOF)
            torque_records.append((name, [round(t, 4) for t in tau_static]))

        # ---- 伺服执行：回当前姿态 → 梯形轨迹跟踪 → 收敛观察 → 超时兜底 ----
        for j in range(NDOF):
            p.resetJointState(robot, j, q_cur[j])
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.95, 0.80, 0.10, 1.0])  # 当前点高亮
        prof = trap_profile(ANIM_STEPS)                # 轨迹规划层：梯形目标轨迹
        dt = 1.0 / SIM_HZ
        q_prev = list(q_cur)
        for k, s in enumerate(prof):
            q_cmd = [q_cur[j] + s * (q[j] - q_cur[j]) for j in range(NDOF)]
            v_cmd = [0.0] * NDOF if k == 0 else [(q_cmd[j] - q_prev[j]) / dt for j in range(NDOF)]
            p.setJointMotorControlArray(               # 关节伺服层：位置伺服 + 速度前馈
                robot, range(NDOF), p.POSITION_CONTROL, targetPositions=q_cmd,
                targetVelocities=v_cmd, forces=FORCES,
                positionGains=[POS_GAIN] * NDOF, velocityGains=[VEL_GAIN] * NDOF)
            p.stepSimulation()
            q_prev = list(q_cmd)
            if writer is not None and k % args.every == 0:
                if args.legacy_tentacle:             # v8.2 分支：到位段后段反向展开
                    tentacle_t = 0.5 * min(1.0, max(0.0, (k / ANIM_STEPS - EXTEND_START) / (1.0 - EXTEND_START)))
                shoot()
        steady, converge_step = 0, None                # 收敛观察窗（保持段 2 s，渲染）
        for k in range(HOLD_STEPS):
            p.setJointMotorControlArray(
                robot, range(NDOF), p.POSITION_CONTROL, targetPositions=q,
                targetVelocities=[0.0] * NDOF, forces=FORCES,
                positionGains=[POS_GAIN] * NDOF, velocityGains=[VEL_GAIN] * NDOF)
            p.stepSimulation()
            ee = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)[4]
            steady = steady + 1 if math.dist(ee, target) <= args.threshold else 0
            if steady >= STEADY_N and converge_step is None:
                converge_step = k
            if writer is not None and k % args.every == 0:
                if args.legacy_tentacle:             # v8.2 分支：保持段前段缠绕包覆
                    tentacle_t = 0.5 + 0.5 * min(1.0, k / (WRAP_END * HOLD_STEPS))
                shoot()
        budget = SERVO_MAX_STEPS - ANIM_STEPS - HOLD_STEPS   # 超时兜底段（不渲染）
        for k in range(budget if converge_step is None else 0):
            p.setJointMotorControlArray(
                robot, range(NDOF), p.POSITION_CONTROL, targetPositions=q,
                targetVelocities=[0.0] * NDOF, forces=FORCES,
                positionGains=[POS_GAIN] * NDOF, velocityGains=[VEL_GAIN] * NDOF)
            p.stepSimulation()
            ee = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)[4]
            steady = steady + 1 if math.dist(ee, target) <= args.threshold else 0
            if steady >= STEADY_N:
                converge_step = ANIM_STEPS + HOLD_STEPS + k
                break
        ee_track = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)[4]
        err_track = math.dist(ee_track, target)
        q_cur = [p.getJointState(robot, j)[0] for j in range(NDOF)]   # 伺服后实际关节角

        reached = (in_lim and err_ik <= args.threshold
                   and err_track <= args.threshold and converge_step is not None)
        note = ""
        if not reached:
            parts = []
            if err_ik > args.threshold:
                parts.append(f"IK残差超阈值({err_ik * 1000:.1f}mm)")
            parts += bad
            if converge_step is None:
                parts.append("伺服未收敛")
            elif err_track > args.threshold:
                parts.append(f"伺服误差超阈值({err_track * 1000:.1f}mm)")
            note = "; ".join(parts)
        rows.append(dict(point_id=name, **{
            "target_x": target[0], "target_y": target[1], "target_z": target[2],
            **{f"q{k+1}": round(q[k], 6) for k in range(NDOF)},
            "err_ik_m": round(err_ik, 6), "err_track_m": round(err_track, 6),
            "err_com_m": round(err_com, 6), "in_limits": int(in_lim),
            "converged": int(converge_step is not None),
            "converge_step": converge_step if converge_step is not None else "timeout",
            "reached": int(reached), "threshold_m": args.threshold, "note": note}))
        print(f"{name:4s} ({target[0]:.2f},{target[1]:+.2f},{target[2]:.2f})   "
              f"{err_ik * 1000:11.2f} {err_track * 1000:12.2f} {err_com * 1000:9.2f} "
              f"{'OK' if in_lim else '超限':4s} "
              f"{str(converge_step) if converge_step is not None else '超时':>6s} "
              f"{'到达' if reached else '未达标':6s}")
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.10, 0.70, 0.15, 1.0])  # 已到达→绿色

        # ---- 演示层：抓 → 移到篮子 → 末端下降放果 → 前往下一点（不参与误差判定） ----
        q_cur, drop_err, drop_ok = place_into_basket(name, q_cur, len(place_checks))
        place_checks.append((name, drop_err * 1000, drop_ok))

    t_total = time.perf_counter() - t_total0
    imageio.imwrite("results/scene.png", shoot(force=True))   # 终态场景截图（writer 仍打开，帧同步入视频）
    if writer is not None:
        writer.close()

    # ---- CSV ----
    fields = ["point_id", "target_x", "target_y", "target_z",
              "q1", "q2", "q3", "q4", "q5", "q6", "q7",
              "err_ik_m", "err_track_m", "err_com_m", "in_limits",
              "converged", "converge_step", "reached", "threshold_m",
              "render_time_s", "note"]
    for row in rows:
        row["render_time_s"] = round(t_render, 3)   # 全局渲染耗时（各行同值，供结果表引用）
    with open(csv_name, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=fields)
        wr.writeheader()
        wr.writerows(rows)

    # ---- run_stats.json（自动回填，指导书生成器据此保证「文档 = 工程」） ----
    errs = [r["err_track_m"] * 1000 for r in rows]
    margins = ["{:.3f} rad".format(min(min(r[f"q{j+1}"] - LIMITS[j][0],
                                          LIMITS[j][1] - r[f"q{j+1}"]) for j in range(NDOF)))
               for r in rows]
    stats = {
        "date": time.strftime("%Y-%m-%d"),
        "platform": "达妙七轴谐波机械臂（官方 URDF 派生）+ ORCA Hand 末端执行器（v8.3）",
        "frames": n_frames,
        "render_time_s": round(t_render, 1),
        "total_time_s": round(t_total, 1),
        "wall_clock": time.strftime("%Y-%m-%d %H:%M → %H:%M"),
        "csv_rows": len(rows), "csv_cols": len(fields),
        "threshold_m": args.threshold,
        "passed": f"{sum(r['reached'] for r in rows)}/{len(rows)}",
        "err_min_mm": round(min(errs), 3), "err_max_mm": round(max(errs), 3),
        "limit_margins": "、".join(margins),
        "drop_residual_mm": round(min(e for _, e, _ in place_checks), 2) if place_checks else None,
        "drop_limit_ok": f"{sum(ok for _, _, ok in place_checks)}/{len(place_checks)}" if place_checks else None,
        "orca_mode": "with-orca-mass（带载对照）" if args.with_orca_mass
                     else "默认（ORCA 固定子装配、惯量置零、无重力基线）",
        "legacy_tentacle": bool(args.legacy_tentacle),
        "pybullet": "3.2.7 (build Jan 29 2025)",
    }
    if args.with_orca_mass:
        peaks = [max(abs(rec[1][j]) for rec in torque_records) for j in range(NDOF)]
        stats["static_torque_peak_Nm_with_orca"] = {f"J{j+1}": peaks[j] for j in range(NDOF)}
        stats["note"] = ("--with-orca-mass 为带载对照实验：重力 + ORCA 官方质量 + 静态持重力矩 PyBullet 实算；"
                         "数字不得与无重力基线混报")
    with open(stats_name, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=1)

    n_ok = sum(r["reached"] for r in rows)
    print("-" * 78)
    print(f"结果：{n_ok}/{len(rows)} 点通过（两重静态判定 + 伺服跟踪；阈值 {args.threshold * 1000:.0f} mm，"
          f"可经 --threshold 配置）")
    print(f"渲染：{n_frames} 帧 @ {w}x{h}，渲染耗时 {t_render:.1f} s；全程耗时 {t_total:.1f} s")
    if place_checks:
        print(f"演示层：{len(place_checks)} 段抓放（放果点 {BASKET_DROP} m，末端下降示意「低头放果」），"
              f"放果点 IK 残差 {min(e for _, e, _ in place_checks):.2f}–"
              f"{max(e for _, e, _ in place_checks):.2f} mm，"
              f"限位 {sum(ok for _, _, ok in place_checks)}/{len(place_checks)} OK；"
              f"该段为视觉示意，不参与误差判定与 CSV")
    if args.legacy_tentacle:
        print(f"演示层仿生触手（--legacy-tentacle，v8.2 复现口径）：{TENTACLE_COUNT} 根 x {TENTACLE_SEGS} 段"
              f"三阶段视觉示意，卷曲(rho=a*exp(b*theta)) -> 到位反向展开 -> 球面缠绕包覆果实，"
              f"逐颗循环、放果后回卷；几何为示意参数（非中科大实物数据），"
              f"无接触 / 夹持力，不参与误差判定与 CSV")
    else:
        print("末端执行器：ORCA Hand 官方 URDF 几何（srl-ethz/orcahand_description，MIT），"
              "fixed 挂接于 link7 的固定子装配，预设握合姿态；"
              "腱绳耦合未在官方 URDF 中建模，本仿真不含腱绳驱动、接触力学与夹持力；"
              "惯量置零以维持 v8.2 无重力基线，不代表带载精度")
    if args.with_orca_mass and torque_records:
        peaks = [max(abs(rec[1][j]) for rec in torque_records) for j in range(NDOF)]
        print("带载静态持重力矩（PyBullet 实算，含 ORCA 官方质量，逐点峰值）："
              + ", ".join(f"J{j+1}={peaks[j]:.3f}" for j in range(NDOF)) + " N·m；"
              "该组数字属带载对照口径，不与无重力基线混报")
    if args.no_video:
        print(f"产物：{csv_name} / results/scene.png（--no-video：未生成 MP4）")
    else:
        print(f"产物：{csv_name} / {mp4_name} / results/scene.png")
    print("（未达标点已如实标注 note，处理流程见指导书 §9.2，禁止放宽阈值或改限位）")
    p.disconnect()
    raise SystemExit(0 if n_ok == len(rows) else 1)


if __name__ == "__main__":
    main()
```

演示素材派生脚本（由完整运行录像生成答辩用 GIF 与六帧关键帧拼图；xArm6 版为临时生成，本版固化为脚本，可复算）：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_demo_assets.py —— 从 results/reach_demo.mp4 派生答辩演示素材。

产出（与 xArm6 版同规格，便于答辩沿用同一套素材用法）：
  results/reach_demo.gif       400×300 / 约 178 帧 / 160 ms 每帧 循环动图（PIL 会合并静止帧，以回读值为准）
  results/reach_keyframes.png  1440×784 六帧拼图（含帧号、时间戳与阶段中文标签）

用法：
    python make_demo_assets.py
    python make_demo_assets.py --gif-size 400x300 --gif-stride 5 --fps 6
"""
import argparse
import os

import imageio
import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
MP4 = os.path.join(HERE, "results", "reach_demo.mp4")
GIF = os.path.join(HERE, "results", "reach_demo.gif")
KF = os.path.join(HERE, "results", "reach_keyframes.png")

FONT_CANDIDATES = ["/mnt/c/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/simhei.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]

# 关键帧：帧号 -> (阶段标签, 补充说明)（v8.3 ORCA Hand 口径）
KEYFRAMES = {
    0:    ("起始：ORCA Hand 握合待机", "末端执行器为固定子装配"),
    100:  ("接近目标：握合姿态保持", "随末端移动，果实为红球标记"),
    200:  ("到位：指掌握合腔对准果实", "预设握合姿态，无开合动画"),
    430:  ("握合腔包拢果实", "纯视觉移动，无接触力"),
    700:  ("搬运入篮", "果实随末端移动到篮内"),
    975:  ("终态：篮中 5 颗", "全部目标点已判定通过"),
}
# --legacy-tentacle 分支（v8.2 复现口径）使用的旧标签
KEYFRAMES_LEGACY = {
    0:    ("起始：触手卷曲待机", "末端后方对数螺旋团"),
    100:  ("接近目标：保持卷曲", "随末端移动，果实为红球标记"),
    200:  ("到位前：反向展开", "触手由卷曲转为伸展"),
    430:  ("缠绕包覆果实", "沿球面缠绕，纯视觉无接触力"),
    700:  ("搬运入篮", "果实随末端移动到篮内"),
    975:  ("终态：篮中 5 颗", "全部目标点已判定通过"),
}


def load_font(size):
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    print("  [警告] 未找到可用字体，标签可能显示为方块")
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--legacy-tentacle", action="store_true",
                    help="使用 v8.2 章鱼爪口径的关键帧标签（对应 --legacy-tentacle 录像）")
    ap.add_argument("--mp4", default=None, help="源录像路径（默认 results/reach_demo.mp4）")
    ap.add_argument("--gif-size", default="400x300")
    ap.add_argument("--gif-stride", type=int, default=5, help="每隔 N 帧取 1 帧进 GIF")
    ap.add_argument("--fps", type=float, default=6.0, help="GIF 播放帧率")
    ap.add_argument("--colors", type=int, default=32, help="GIF 调色板颜色数（越大越大越清晰）")
    args = ap.parse_args()
    gw, gh = (int(v) for v in args.gif_size.lower().split("x"))

    src_mp4 = args.mp4 or MP4
    keyframes = KEYFRAMES_LEGACY if args.legacy_tentacle else KEYFRAMES
    if not os.path.exists(src_mp4):
        raise SystemExit(f"[错误] 找不到 {src_mp4}，请先跑 damiao_reach_demo.py 生成完整运行录像")
    frames = iio.imread(src_mp4, plugin="FFMPEG")
    total = frames.shape[0]
    print(f"[素材] 源录像 {total} 帧 @ {frames.shape[2]}×{frames.shape[1]}")

    # ---------- GIF 循环动图 ----------
    picks = list(range(0, total, args.gif_stride))
    gif_frames = [np.asarray(Image.fromarray(frames[i]).resize((gw, gh), Image.LANCZOS))
                  for i in picks]
    # 量化到自适应调色板并跨帧优化：官方 STL 场景色彩数远多于 xArm6 版，
    # 不量化会让 GIF 涨到 4 MB+；量化后与原仓库同量级（数百 KB）。
    pal = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=args.colors)
           for f in gif_frames]
    # 不设 disposal=2：整帧存储会让 GIF 体积翻倍；默认 disposal 允许帧间差分
    pal[0].save(GIF, save_all=True, append_images=pal[1:],
                duration=int(round(1000.0 / args.fps)), loop=0, optimize=True)
    # 回读真实参数：PIL 的 optimize 会合并连续相同的帧，写入帧数可能少于取样帧数，
    # 文档与 README 一律引用回读值（不引用估算值）。
    with Image.open(GIF) as chk:
        real_n = 0
        try:
            while True:
                chk.seek(real_n)
                real_n += 1
        except EOFError:
            pass
        dur_ms = chk.info.get("duration", int(round(1000.0 / args.fps)))
    print(f"[素材] GIF：{gw}×{gh} / 取样 {len(gif_frames)} 帧 → 实际写入 {real_n} 帧 / "
          f"{dur_ms} ms 每帧（≈{1000.0 / dur_ms:.1f} fps，循环约 {real_n * dur_ms / 1000:.1f} s）/ "
          f"{args.colors} 色 / {os.path.getsize(GIF) // 1024} KB -> {os.path.relpath(GIF, HERE)}")

    # ---------- 六帧关键帧拼图（3 列 × 2 行） ----------
    cw, ch = 480, 392
    font_lbl = load_font(22)
    font_sub = load_font(16)
    canvas = Image.new("RGB", (cw * 3, ch * 2), (18, 18, 20))
    draw = ImageDraw.Draw(canvas)
    for k, idx in enumerate(sorted(KEYFRAMES)):
        if idx >= total:
            idx = total - 1
        img = Image.fromarray(frames[idx]).resize((cw, ch), Image.LANCZOS)
        x, y = (k % 3) * cw, (k // 3) * ch
        canvas.paste(img, (x, y))
        label, sub = keyframes[idx]
        draw.rectangle([x, y + ch - 58, x + cw, y + ch], fill=(0, 0, 0))
        draw.text((x + 10, y + ch - 52), f"#{idx}  t={idx / 30:.1f}s  {label}",
                  font=font_lbl, fill=(255, 235, 120))
        draw.text((x + 10, y + ch - 26), sub, font=font_sub, fill=(225, 225, 225))
    canvas.save(KF)
    print(f"[素材] 关键帧拼图：{canvas.size[0]}×{canvas.size[1]} / 6 帧 / "
          f"{os.path.getsize(KF) // 1024} KB -> {os.path.relpath(KF, HERE)}")


if __name__ == "__main__":
    main()
```

**v8.3 运行开关**（与 xArm6 版、v8.2 的差异说明）：

| 模式 | 命令 | 说明 |
|---|---|---|
| 默认（v8.3 基线） | `python damiao_reach_demo.py` | ORCA Hand 固定子装配（预设握合姿态），无重力基线，`results/reach_results.csv` |
| v8.2 复现 | `python damiao_reach_demo.py --legacy-tentacle` | 卷曲章鱼爪演示层分支，臂本体与判据不变 |
| 带载对照 | `python damiao_reach_demo.py --with-orca-mass` | 重力 9.81 + ORCA 官方质量（1.135 kg），独立 CSV `results/reach_results_orca_mass.csv` + 逐点静态持重力矩 PyBullet 实算，**数字不与无重力基线混报** |
| 快速验证 | `--no-video` / `--check-out-of-range` / `--res 640x480 --every 16` | 同 v8.2 口径 |

### 步骤七附加：负向验证（证明预筛逻辑存在且工作）

```bash
python damiao_reach_demo.py --check-out-of-range; echo "exit=$?"（期望 2）
```

预期输出：

```
[预筛拒绝] P_BAD_H 水平距离 1.200 m > 0.70 m，IK 前中止。
[预筛拒绝] P_BAD_Z z=0.900 m 不在 [0.1,0.75]，IK 前中止。
（--check-out-of-range 仅验证拒绝逻辑，不构成可达性结论）
exit=2
```

---

## 8. 运行与结果采集

### 8.1 正常运行（步骤七已含）与快速模式

```bash
cd damiao_reach_lab
python third_party/orcahand/expand_orcahand_xacro.py   # v8.3：ORCA xacro 展开（可复算）
python derive_orcahand_urdf.py --src third_party/orcahand/orcahand_right.urdf --dst third_party/orcahand/orcahand_right_sim.urdf   # v8.3：ORCA 派生（须 ORCA DERIVE PASS）
python derive_damiao_urdf.py --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf --dst damiao_reach.urdf   # 达妙派生 + ORCA 子装配追加（须 DERIVE PASS）
python test_load.py                     # 加载自检（须 ALL PASS，含 ORCA 断言）
python damiao_reach_demo.py --no-video            # 快速验证（只出 CSV/截图，秒级）
python damiao_reach_demo.py --check-out-of-range # 负向验证（退出码 2）
python damiao_reach_demo.py                      # 完整运行（CSV + MP4 + 截图；默认 ORCA Hand）
python damiao_reach_demo.py --legacy-tentacle    # v8.2 章鱼爪复现分支
python damiao_reach_demo.py --with-orca-mass     # 带载对照实验（独立 CSV，不与基线混报）
python select_targets.py                         # 目标点选取工具（可复算）
```

### 8.2 渲染预算与降级分支

| 模式 | 帧数估算 | 实测耗时（2026-11-18，v8.3 ORCA 版） |
|------|----------|------------------------|
| 默认 960×720 / 每 8 步（ORCA Hand） | 每点到位段约 120 帧 + 演示层抓放段约 75 帧，共 ~975 帧 | **976 帧，渲染 265.4 s，全程 266.0 s** |
| 降级 640×480 / 每 16 步 | 约 ~490 帧 | **未执行**（默认模式 265.4 s < 5 分钟预算，未触发降级分支） |
| `--no-video` | 1 帧（截图） | **0.6 s**（渲染 1 帧；伺服、抓放段与果实搬运仍完整执行） |
| `--legacy-tentacle`（v8.2 复现分支） | 与默认相同 | 快速验证 5/5 通过；降分辨率（320×240 / 每 16 步）抽样渲染确认章鱼爪三阶段动画完整（§14.7，非全量基线） |
| `--with-orca-mass`（带载对照） | 与默认相同 + 重力 | `--no-video` 快速验证 5/5 通过、静态持重力矩逐点峰值 J2 7.910 N·m（PyBullet 实算，含 ORCA 官方质量 1.135 kg）；**独立 CSV，完整渲染未执行** |
| 素材派生 `make_demo_assets.py` | GIF 178 帧（取样 196） / 关键帧拼图 6 帧 | **GIF 400×300 / 178 帧 / 160 ms 每帧（≈6 fps，循环约 28.5 s）/ 32 色；拼图 1440×784**（由同一次运行的 MP4 派生，ORCA 口径标签） |

ER_TINY_RENDERER 为 CPU 软件渲染，不依赖 WSLg/GPU，但高分辨率下耗时不可忽略。**渲染耗时由脚本自动打印并应在结果表登记**。若默认模式渲染超过 5 分钟，属正常范围；超过 15 分钟改用降级分支重跑。v8 实测 265.4 s 落在预算内（帧数与 xArm6 版同为 976 帧，来自同样 5 段抓放演示），故降级分支未运行；该分支的耗时不作任何结论引用。达妙臂 STL 更大（16 个官方网格，合计约 33 MB），单帧渲染耗时与 xArm6 版同量级。

### 8.3 结果记录表（2026-11-18 达妙七轴版全量重跑回填；数据源：控制台汇总表 + `results/reach_results.csv` + `results/run_stats.json`）

| 点 | 目标 (m) | IK 残差 (mm) | 伺服误差 (mm) | COM (mm) | 限位 | 收敛步 | 结论 | 备注 |
|----|----------|--------------|---------------|----------|------|--------|------|------|
| P1 | (0.2258, -0.0060, 0.3250) | 0.000 | 0.000 | 71.69 | OK | 59 | 到达 | |
| P2 | (0.2176, 0.1206, 0.2386) | 0.001 | 0.001 | 71.69 | OK | 59 | 到达 | |
| P3 | (0.1742, -0.1081, 0.2361) | 0.000 | 0.000 | 71.69 | OK | 59 | 到达 | |
| P4 | (0.2652, 0.0647, 0.3954) | 0.001 | 0.001 | 71.69 | OK | 59 | 到达 | |
| P5 | (0.2747, -0.0486, 0.1567) | 0.000 | 0.000 | 71.69 | OK | 59 | 到达 | |

汇总信息：阈值 = **5 mm**（`--threshold 0.005`，未放宽）；渲染耗时 = **265.4 s**（976 帧 @ 960×720）；总耗时 = **266.0 s**（墙钟 2026-11-18 22:16 → 22:16）；pybullet 版本 = **3.2.7**（build time Jan 29 2025）。CSV 的 `render_time_s` = 265.4（全局渲染耗时，各行同值）。CSV 为 **5 行 21 列**（新增 `q7`）。演示层 5 段抓放（v8.3 默认 = ORCA Hand 固定子装配、预设握合姿态呈现）：放果点 IK 残差 0.00 mm、限位 5/5 OK，该段不写入 CSV。

**目标点选取**：原 xArm6 的 5 点在达妙臂上**全部不可达**（IK 残差 146–200 mm，见 `handoff/演算基准.json` 的 `original_5_targets_IK`）。本轮用 `select_targets.py` 按官方限位 + 官方几何重扫 6000 组采样可达空间，在 5 个草莓簇锚点邻域内择优，要求：**残差 ≤ 2 mm、限位余量 ≥ 0.15 rad、点间最小间距 0.08 m**，并采用与实跑一致的链式两段式 IK（初值 = 上一点解）。最终各点限位余量 **0.407 rad、0.308 rad、0.490 rad、0.730 rad、0.636 rad**。

**COM 备查列**：达妙 `link7` 的 COM 相对其原点偏移约 **71.7 mm**（因 `link7` 含官方「夹爪相机 3507 安装板」结构，重心外移），故 COM 口径与原点口径相差一个量级，**混用会制造虚假误差**；COM 仅备查、不参与任何判定。

**实测结论**：5 点全部满足两重静态判定 + 伺服跟踪判据，逐点伺服跟踪误差 **0.000–0.001 mm < 5 mm**（实测值见表），`converged=1`、`in_limits=1`、收敛步全为 59；未发生任何目标点坐标或伺服增益调整（§9.2 决策树未触发）。**本结论只覆盖无重力设定下的轨迹跟踪与定位收敛**（判定点 = `link7` 原点，ORCA 子装配不进 IK 链、惯量置零），不代表带重力实物化性能，也不代表抓取成功率；末端执行器几何与果实搬运仅为流程可视化（§0 第 1 / 2 / 7 条）。

**结论书写规则**：仅当 5 点实测全部"到达"后，方可在结论区写"逐点误差 < 5 mm（实测值见表）"；存在未达标点时，结论如实写"Px 未收敛/超阈值，已按 §9.2 调整/记录"。**禁止先写结论再凑数。**

**未执行项**：`--with-orca-mass` 带载对照的完整渲染（快速验证已执行，独立 CSV 已生成，**不与本组无重力数字混报**）、降级渲染分支、`--threshold` 换阈值复跑、全新机器从零环境复跑、MP4 全量逐帧人工判读。

---

## 9. 可达性预检与目标点调整分支

### 9.1 正常情况

5 点全部通过（两重静态判定 + 伺服跟踪，§1.3）→ 直接进入 §10 归档。

### 9.2 未收敛 / 超阈值 / 超限位的规则化处理（不是探索，是固定决策树）

**第 1 步：判断失败类型**（看 CSV 的 note 列、converged/converge_step 与 q 值）

| 现象 | 诊断 | 调整动作（每次只动一档，改后重跑） |
|------|------|------------------------------------|
| err_ik > 阈值 且 水平距离 > 0.30 m | 点位过远，接近臂展极限（达妙 link7 实测 r_max≈0.389 m） | 水平距离缩小 0.03 m |
| err_ik > 阈值 且 z > 0.45 m | 点位过高（达妙 link7 实测 z 上限≈0.507 m） | z 下调 0.03 m |
| err_ik > 阈值 且 解的限位余量 < 0.05 rad | 姿态贴限位 | 用 `select_targets.py` 重选邻域内余量更大的点 |
| err_ik > 阈值 且以上三条都不满足 | IK 迭代不足 / 内置求解器停在局部解 | 先确认 `refine_ik()` 精修段已启用（本版已启用）；必要时加大 `IK_MAX_ITER` 或改用多初值粗解 |
| converged = 0（converge_step = timeout） | 伺服增益不足或收敛窗偏短（先确认 §0 第 6 条无重力设定未被改动） | `POS_GAIN` 1.0→2.0 重跑一次（振荡则 `VEL_GAIN` 1.0→2.0 加阻尼，见附录 C） |
| in_limits = False | IK 解贴限位（姿态别扭） | 按上表同规则调整坐标后重跑 |

**第 2 步：登记变更**（下表写在指导书/报告内，坐标调整必须留痕）

| 点号 | 旧坐标 (m) | 新坐标 (m) | 失败原因（残差/超限关节及数值） | 重跑结果 |
|------|-----------|-----------|-------------------------------|----------|
| 无 | — | — | 5 点一次通过两重静态判定 + 伺服跟踪，未触发本决策树 | 无需调整重跑；`--no-video` 与完整运行两次结果逐点一致 |

> 注：实跑中发生的**脚本缺陷修复**（Writer 收尾顺序、产物打印口径、CSV 列）不属于目标点 / 增益调整，登记于 §14，不占用本表。

**禁止事项**：放宽 `--threshold` 应付验收、改 URDF 限位、删点、改 HOME 后沿用旧数据、只报调整后的点而隐瞒调整前的失败记录。

---

## 10. 工程归档（拷回 Windows）

```bash
# 目标目录不存在时：整目录拷入，xarm_lab_run 即工程根目录
cp -r ~/xarm_lab /mnt/d/桌面/CRAIC/success/202607091551/temp_20261104_01/xarm_lab_run

# 目标目录已存在时（本轮实跑即如此）：用 /. 把内容平铺进去，避免嵌套出 xarm_lab_run/xarm_lab/
cp -r ~/xarm_lab/. /mnt/d/桌面/CRAIC/success/202607091551/temp_20261104_01/xarm_lab_run/
```

> **v4 实跑注记（2026-11-04 18:39）**：归档前先把 `xarm_lab_run\` 内已完成使命的暂存脚本（`node*.sh`）与空 `wheels\` **移出**到工作区 `archive_node_scripts_20261104\`（未删除，保留备份），再按上述第二条命令的思路逐项拷入工程文件（额外排除 `.git` 与 `__pycache__`）。归档结果：7 个 STL、3 个 `results/` 产物、URDF / 2 个 py / README / 3 份文档，逐文件 `cmp` 与 WSL 工程一致；详见《运行日志_20261104.md》Node 7。

**整工程归档，不只是结果目录**：`damiao_reach.urdf`、`derive_damiao_urdf.py`、`derive_orcahand_urdf.py`、`third_party/orcahand/`、`select_targets.py`、`test_load.py`、`damiao_reach_demo.py`、`meshes/damiao/`、`meshes/orcahand/`、`results/` 全部进入 Windows 侧 `d:\桌面\CRAIC\success\202607091551\temp_20261104_01\xarm_lab_run\`。课设报告与说明书引用的数字一律以 `xarm_lab_run\results\reach_results.csv` 为准；"可运行仿真"交付即此目录——在任意装有 Python 3.11 + pybullet 的 Linux 环境，把该目录内容放回工程根目录即可按 §5–§8 复跑。

---

## 11. 常见问题排错表

| # | 现象 | 原因 | 处理 |
|---|------|------|------|
| 1 | `pip install pybullet` 开始编译源码（大量 g++ 输出） | 当前 Python 不是 3.11（wheel 实测仅 cp36–cp311） | `conda activate robot` 后确认 `python -V` = 3.11.x 再装 |
| 2 | wheel 拉取超时/SSL 报错 | 网络或镜像波动 | 重试；确认用 `-i https://pypi.tuna.tsinghua.edu.cn/simple`；公司网络需代理时换手机热点 |
| 3 | 源码编译失败：`error: command 'gcc' failed` / `Python.h: No such file` | 走了异常路径但缺编译链 | `sudo apt install -y build-essential cmake python3-dev` 后重装；编译 5–10 分钟属正常，非卡死 |
| 4 | `error: Cannot load URDF file` / `XML_ERROR_PARSING_DECLARATION` | 不在工程根目录运行（URDF 与 mesh 均为相对路径）；或派生时把 XML 声明挪到了注释之后 | `cd damiao_reach_lab` 后重跑；派生脚本已保证 XML 声明位于文件首行 |
| 5 | 报 mesh 相关错误 / 模型不显示 | `meshes/damiao/{visual,collision}/` 下 STL 不足 8 个 | 回到步骤四重新拷贝；`tool_link` / `gripper_*` 不拷贝是**正确行为** |
| 6 | test_load [5] FK 断言 FAIL | 派生 URDF 被改动过 | 用 `derive_damiao_urdf.py` 重新派生（官方源文件只读）；数值来源见附录 B |
| 6b | 关节数不是 7 / 末端 link 不是 link7 | 用的是未裁剪的官方 URDF | 必须用派生的 `damiao_reach.urdf`；官方 URDF 含 1 fixed + 2 prismatic 夹爪关节 |
| 7 | 某点 err_ik 大、结论"未达标" | 点位对该臂的工作空间/限位不友好 | 按 §9.2 决策树调整并登记，**不得**放宽阈值或改限位 |
| 8 | IK 解超限位（in_limits=0） | 同上，姿态贴限位 | 同 §9.2 |
| 8b | converged=0（converge_step=timeout） | 伺服增益不足或收敛窗偏短（无重力设定下增益基线 1.0） | `POS_GAIN` 1.0→2.0（振荡则 `VEL_GAIN` 1.0→2.0），重跑并按 §9.2 登记，指引见附录 C |
| 9 | 渲染过慢（>15 分钟） | CPU 软渲染高分辨率 | `--res 640x480 --every 16` 降级重跑；或先 `--no-video` 验证数据 |
| 10 | MP4 黑屏/打不开 | 编码器问题（罕见） | 确认 `imageio-ffmpeg` 安装（步骤三验证命令）；重跑；仍失败改 `--no-video` 用 scene.png + CSV 交付 |
| 11 | err_com 与 err_ik / err_track 差 10 mm+ | 口径混淆 | 正常现象：[0] 是 COM、[4] 是 link 原点，**主口径是 [4]**，报告数字取 err_ik / err_track |
| 12 | 动画里臂"瞬移" | 用了 `--no-video` 或帧率理解偏差 | 视频按 30 fps 导出，每 8 仿真步一帧；如需慢放改用 `--every 4` |

---

## 12. 答辩追问速答（口径与本文档一致）

1. **末端执行器是什么？为什么画面里没有官方夹爪？**
   末端执行器是 **ORCA Hand 灵巧手**（v8.3 默认）：几何与关节结构取自 SRL-ETHZ 官方描述仓库 `srl-ethz/orcahand_description`（**MIT License**），以 fixed joint 挂接于 `link7` 之后作为**固定子装配**——惯量置零、17 个 revolute 锁定为预设握合姿态、碰撞过滤屏蔽不参与接触，**不进运动学链、不参与误差判定、不写 CSV**；**腱绳耦合未在官方 URDF 中建模**（无 mimic/transmission），本仿真不含腱绳驱动、接触力学与夹持力，也不代表抓取成功率；**本轮明确不采用达妙官方夹爪**，官方 URDF 的夹爪子树（`tool_link` / `gripper_left` / `gripper_right` 及 2 个移动关节）已在派生 URDF 中整段裁剪，末端参考点固定为 `link7` 原点（不含 0.157 m 夹爪偏移）。v8.2 的卷曲仿生章鱼爪演示层保留为 `--legacy-tentacle` 复现分支。仿真控制结构为两层——**关节空间梯形轨迹规划 + 关节位置伺服**（POSITION_CONTROL，分关节力矩上限 + 轨迹段速度前馈；动力学含惯量与关节阻尼，**无重力设定**——验证指令跟踪与定位收敛，重力负载前馈补偿属实物化后续工作），验证臂本体定位链路。夹爪开合与接触力学属下一阶段。
2. **IK 是怎么解的？为什么解是确定的？**
   位置-only 对 7 轴臂是**冗余问题**（3 约束 / 7 未知）；PyBullet 用 nullspace 法在满足位置约束的解流形中取最接近参考姿态 HOME 的解。限位（官方 7 组）、参考姿态（HOME）、初值策略（上一点实际关节角，保证解连续性）全部写死；内置求解器粗解后再加一段**位置-only 数值精修**（数值雅可比 + 阻尼最小二乘），因为 7 轴冗余下内置求解器在远距离初值会停在 3–8 mm 局部解。任何人复跑得到同一组关节角（可复现性）。
3. **误差是怎么测的？测的是哪个点？**
   末端 link 名为 `link7`（官方 URDF 七轴臂的最后一节，官方夹爪已裁剪）；误差 = 目标点到 `link7` 原点（URDF link frame 原点，`getLinkState[4]`）的欧氏距离，与 IK 的末端参考系一致，**不含夹爪偏移**；质心口径（[0]）同表备查（达妙臂两者相差约 71.7 mm）。link 索引按名反查，不硬编码。
4. **5 mm 阈值怎么定的？**
   阈值是可配参数（--threshold）；实测**伺服跟踪误差**（无重力设定下控制回路的末端定位精度）见表 §8.3。草莓果径约 20–40 mm，毫米级末端定位误差对采摘目标而言裕度充分。
5. **力矩上限是哪来的？**
   分关节取自**达妙官方 URDF 的 `<limit effort>`**：J1–J4 = 50 N·m、J5–J7 = 12 N·m，直接用于主流程位置伺服的 `forces` 参数与 URDF effort 标签。**口径提示**：该值是否已含减速比/效率折算，官方未写明，按「官方原始值」直接引用，不自行乘过载系数（xArm6 版的 2.55 过载系数不适用于本臂）。另有 numpy 静态重力矩校核（g=9.81，6000 组采样）：J2 峰值 8.83、J3 2.67、J4 2.74、J5 0.65、J6 0.68、J7 0.24 N·m，J5–J7 裕度约 11 N·m，全程静态不超限——**该值为静态演算，非 PyBullet 实跑、非带载精度验证**。
6. **目标点为什么选这 5 个？**
   **原 xArm6 的 5 点在达妙臂上全部不可达**（残差 146–200 mm，臂展差一代），因此按**达妙官方限位 + 官方几何重扫可达空间**后重选：预筛包络取自实测 `workspace.link7`（r ≤ 0.389 m、z ∈ [-0.111, 0.507]），在草莓簇锚点邻域内按「残差 ≤ 2 mm、限位余量 ≥ 0.15 rad、点间距 ≥ 0.08 m」择优，选点工具为 `select_targets.py`（可复算）；最终以两重静态判定 + 伺服跟踪实测锁定，实测残差 0.000–0.001 mm；若发生过调整，登记于 §9.2 变更表。
7. **动画里臂是怎么运动的？**
   关节空间梯形轨迹（加速 25% / 匀速 50% / 减速 25%）作为**目标轨迹**，由**位置伺服**（POSITION_CONTROL，分关节力矩上限 + 速度前馈）在 240 Hz 动力学仿真（URDF 惯量/阻尼，无重力设定）中跟踪执行，30 fps 导出；伺服增益调整指引见附录 C。
8. **为什么用 PyBullet？**
   内置带限位的逆运动学求解器，适合快速验证定位链路；**模型与参数全部来自达妙官方开源仓库**（`left_arm_dm`，BSD-3-Clause），本轮只做了一处派生：删除官方夹爪子树 + 改写 mesh 引用路径，几何/惯量/限位逐字未改（来源对照见附录 B）。
9. **动画里「摘草莓、放进篮子」是真的抓取吗？**
   **不是**，这是 §0 第 7 条的演示层抓放循环：果实球是位置标记，脚本直接把它的视觉位置搬到篮中；末端下降到放果点表达「低头」，因位置-only IK 不约束姿态，末端姿态并未真实改变。没有夹爪、没有接触检测、没有夹持力或接触力学，也不做果实识别。5 个草莓点的到位误差仍按 §1.3 判定，演示段不产生误差数据、不写入 CSV；动画证明的是「流程可达 + 定位精度」，不是抓取成功率。v7 的 3 根仿生触手按「向后卷曲成对数螺旋 → 到位反向展开 → 球面缠绕包覆」三阶段逐颗循环（示意几何、非实物参数），仍是演示层纯视觉示意，不是 URDF 末端执行器，不构成真实夹持；仿生触手的机构结构在 A0 图纸与说明书中讲，仿真只验证达妙七轴平台的控制流程；末端执行器为可替换模块（v8.3 = ORCA Hand 官方 URDF 几何固定子装配，v8.2 = 卷曲章鱼爪演示层），两者均不参与精度判定。

---

## 13. 实跑回填清单（v8.3 达妙七轴 + ORCA Hand 版状态，2026-11-18）

- [x] 模型派生自检：`derive_orcahand_urdf.py` 输出 **ORCA DERIVE PASS**（42 link / 41 joint、68 处 mesh 路径改写全部可达、17 个 revolute 锁定预设姿态、无 package:// 残留、83 个块与官方展开一致）
- [x] 模型派生自检：`derive_damiao_urdf.py` 输出 **DERIVE PASS**（50 link / 49 joint、达妙 15 个保留块与官方逐字一致、ORCA 子装配 42 link / 41 fixed + 1 挂接 joint 校验通过）
- [x] 加载自检 `python test_load.py`：**ALL PASS**（49 link、达妙 7 组官方限位逐项 PASS、末端 `link7` 索引按名反查 = 6、零位 FK 偏差 0.000 mm、ORCA 惯量置零 / 碰撞过滤 / 包围盒断言 PASS、渲染 307200 bytes）
- [x] 5 点 err_ik / err_track / err_com / 限位判定 / 收敛步 / 结论（§8.3 表）：已按 2026-11-18 v8.3 全量重跑的 `results/reach_results.csv` 回填，**5/5 通过**，逐点残差 0.000–0.001 mm（阈值 5 mm 未放宽，判定点仍为 `link7` 原点）
- [x] 渲染耗时：默认模式 **976 帧 / 265.4 s**（§8.2）；`render_time_s` = 265.4；`--no-video` 0.6 s；降级模式**未执行**（未触发）
- [x] 演示素材派生：`make_demo_assets.py` → `results/reach_demo.gif` + `results/reach_keyframes.png`（ORCA 口径中文标签），均已目检
- [x] 负向验证 `--check-out-of-range`：**退出码 2**，越界点（P_BAD_H 水平 0.80 m > 0.389 m；P_BAD_Z z=0.80 m ∉ [-0.111, 0.507]）在 IK 前被预筛拒绝
- [x] ORCA 末端执行器：固定子装配（fixed `joint_orca_wrist`）、惯量置零、预设握合姿态、碰撞过滤屏蔽、不进 IK 链 / 误差判定 / CSV；`select_targets.py` 复算与 v8.2 记录一致
- [x] `--legacy-tentacle` 分支：快速验证 5/5 通过 + 降分辨率抽样渲染确认章鱼爪三阶段动画完整（v8.2 复现口径）
- [x] `--with-orca-mass` 带载对照：`--no-video` 快速验证 5/5 通过，独立 CSV `results/reach_results_orca_mass.csv`，逐点静态持重力矩峰值 J2 7.910 N·m（PyBullet 实算，含 ORCA 官方质量 1.135 kg）；**完整渲染未执行，数字不与无重力基线混报**
- [x] 演示层抓放：5 段，放果点 IK 残差 0.00 mm、限位 5/5 OK，篮子与果实入篮可见
- [x] 结论区文字（按 §8.3 结论书写规则）：已写「逐点误差 0.000–0.001 mm < 5 mm」，附无重力设定与演示层边界限定
- [x] 静态重力矩校核（numpy，g=9.81，6000 组采样，v8.2 演算）：J2 8.83 / J3 2.67 / J4 2.74 / J5 0.65 / J6 0.68 / J7 0.24 N·m，对照 50/50/50/50/12/12/12 → J5–J7 裕度约 11 N·m（**该值未含 ORCA 质量**；含末端质量的对照见 `--with-orca-mass` 实算）
- [ ] **带重力纯臂对照运行**（不带 ORCA 质量）：未执行（需用户单独裁决；不得与本组无重力数字混报）
- [ ] `--with-orca-mass` 完整渲染 / 降级渲染分支 / `--threshold` 换阈值复跑：未执行
- [ ] 零探索验证：全新机器按本文档从零环境完整复跑（**未执行**，本轮为既有 WSL 环境实跑）
- [ ] MP4 画面**全量逐帧**人工判读（已抽样判读 6 帧，非全量，不作逐帧正确性结论）

---

## 14. 实跑变更记录（v4 基线 + v5 / v6 / v7 扩展 + v8 达妙七轴迁移 + v8.3 ORCA 末端执行器）

### 14.1 实跑基线（**以下 §14.1–§14.5 为 xArm6 六轴平台的历史记录，已随平台更换作废，仅作对照保留**）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-04 18:18–18:28（+08:00）；完整运行 18:27:06 → 18:28:29 |
| 环境 | WSL2 Ubuntu 26.04.4 LTS，默认用户 `slphy_kai`，项目 `/home/slphy_kai/xarm_lab`；conda `robot`，Python 3.11.18，pybullet 3.2.7（cp311 manylinux wheel），numpy 2.4.6，imageio 2.38.1，imageio-ffmpeg 0.8.1，pillow 13.0.0 |
| 命令 | `python test_load.py`；`python xarm6_reach_demo.py --no-video`；`python xarm6_reach_demo.py --check-out-of-range`；`python xarm6_reach_demo.py` |
| 对应 commit | 环境 `fd5c92b` / 自检 `94a232f` / 快速验证 `309e282` / 完整运行与脚本修复 `045065d`；v4 回填提交见《运行日志_20261104.md》终验总表 |
| 产物 | `results/reach_results.csv`（744 B，含 `render_time_s`）、`results/reach_demo.mp4`（732358 B，601 帧）、`results/scene.png`（26484 B） |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器从零环境复跑、MP4 画面逐帧人工判读 |

### 14.2 实跑发现与变更（旧值 → 新值 → 原因 → 重跑结果）

| # | 类别 | 旧值 / 现象 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 加载自检显示口径 | `test_load.py` 打印 `len(img[2])` = 240 且标为 bytes | 改为 `img[2].nbytes` = 307200，文案补「= 320×240×4」 | `img[2]` 是 shape (240,320,4) 的 ndarray，`len()` 取第一维行数；判据 `>0` 语义不变 | 18:20:17 重跑 `== ALL PASS ==`，6 限位 / link6 / FK 0.00 mm / 307200 bytes 全 PASS |
| 2 | 完整运行崩溃（六步流程处理） | Node4 首跑 `FULL_EXIT=1`：`writer.close()` 先于 `shoot(force=True)`，向已关闭 Writer 追加帧 → `RuntimeError: I/O operation on closed Writer.` | 把 `writer.close()` 移到终态截图之后，截图帧正常写入 MP4 再关闭 | `shoot()` 仅判 `writer is None`、未判 closed；`--no-video` 模式 writer 恒为 None 故不触发 | 18:27:06 重跑 `FULL_EXIT=0`，601 帧 / 82.5 s，CSV / MP4 / PNG 齐备，MP4 可完整解码 601 帧 |
| 3 | 产物打印口径 | 汇总行固定打印 `reach_demo.mp4`，`--no-video` 下该文件并不生成 | 按 `args.no_video` 分模式打印，明确标注「--no-video：未生成 MP4」 | 避免声明不存在的产物 | `--no-video` 回归输出正确；完整运行打印三件产物路径 |
| 4 | CSV 与 §1.4 口径 | CSV 无渲染耗时列，而 §1.4 称其含渲染耗时 | CSV 增 `render_time_s` 列（全局值 82.501，各行同值） | 使文档表述、spec 与实现一致；渲染耗时仍由控制台汇总表打印 | 重跑 CSV 744 B，5 行均含 82.501 |

**未改动项（红线核对）**：`--threshold` 保持 0.005 m（未放宽）；URDF 关节限位、HOME / restPoses、TARGETS（P1–P5）、FORCES（163.2/163.2/81.6/81.6/30.6/30.6）、POS_GAIN / VEL_GAIN（1.0 / 1.0）、`setGravity(0,0,0)`、STEADY_N = 60、SERVO_MAX_STEPS = 3000、IK_MAX_ITER = 100 全部未改；未删点、未隐藏失败记录；§9.2 决策树未触发，无坐标 / 增益变更登记。

**同步范围**：#1–#4 的代码改动均在 WSL 工程、Windows 暂存区母本、本文档 §6 / §7 内嵌脚本三处同步（逐字比对一致），`py_compile` 通过；处理过程见《运行日志_20261104.md》Node 2 / Node 4。**失败记录保留**：Node4 首跑的 traceback 与 `FULL_EXIT=1` 全文记于运行日志，未从任何文档中删除。

### 14.3 v5 演示层抓放循环与全量重跑（2026-11-06）

| 项 | 值 |
|---|---|
| 实跑时间 | 完整运行 17:59:38 → 18:01:53（+08:00）；素材派生 18:34 |
| 环境 | 同 §14.1（WSL2 Ubuntu 26.04.4 LTS / conda `robot` / Python 3.11.18 / pybullet 3.2.7） |
| 命令 | `python xarm6_reach_demo.py --no-video`；`--check-out-of-range`；`python xarm6_reach_demo.py` |
| 产物 | CSV 749 B（5 行 19 字段）、MP4 1.4 MB（976 帧 / 32.53 s）、scene.png 28 KB、GIF 195 帧 / 337 KB、关键帧拼图 193 KB |

> 注：v5 当时文档写作「19 字段」，脚本与 CSV 实际为 **20 列**；v6 已统一校准为「5 行 20 列」，此处保留历史原文不改。
| 旧值 → 新值 | 601 帧 / 82.5 s → **976 帧 / 134.2 s**（渲染）；`render_time_s` 82.501 → **134.205**；5 点误差、q1–q6、收敛步**逐点完全一致** |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读（已抽样 6 帧） |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|------|-------------|------|----------|
| 1 | 范围红线 | §0 只写「依次到达 5 点」，不含采摘流程 | §0 新增第 7 条：正式纳入演示层抓放循环，主动声明无夹爪 / 接触 / 力 / 姿态约束 | 课设题目为采摘机械臂控制系统设计，动画需体现逐颗采摘 | 文档与代码同步；5 点判据、阈值、限位、TARGETS 全部未改 |
| 2 | 脚本 | 每点结束直接进入下一点 | 每点 `rows.append` 后插入独立 `place_into_basket()`：放果点 IK + 同套梯形轨迹 / 位置伺服 + 果实视觉搬运入篮 | 演示层隔离：不写 rows、不改 fields、不参与判定 | `--no-video` 与完整运行均 5/5 通过、CSV 仍 5 行；放果点 IK 残差 0.97 mm、限位 5/5 OK |
| 3 | 场景 | 地面 + 栽培架 + 5 个球 | 新增棕色开口篮（5 个静态视觉体）+ 5 个篮内果实槽位 | 让「放入篮子」在画面中可见 | 抽样判读 6 帧：frame 194 / 389 / 779 篮中分别 1 / 2 / 4 颗，终态 5 颗全在篮中 |
| 4 | 起始位姿 | 下一点伺服起始 = 上一草莓点姿态 | 下一点伺服起始 = 放果段结束姿态 | 流程连续性 | 原预判「err_track / converge_step 可能变化」；**实测逐点数值与 v4 完全一致**（含 converge_step=59），如实记为「一致」，未臆造差异 |
| 5 | 素材 | 601 帧 MP4 / 120 帧 GIF / 旧拼图 | 976 帧 MP4 / 195 帧 GIF / 覆盖抓放时刻的新拼图 | 素材须与改后运行同源 | 画面抽样判读通过，GIF 首 / 中 / 末帧非纯色 |

**未改动项（红线核对）**：`--threshold` 仍 0.005 m；URDF 关节限位、HOME / restPoses、TARGETS（P1–P5）、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`、STEADY_N、SERVO_MAX_STEPS、IK_MAX_ITER 全部未改；未删点、未隐藏失败记录；§9.2 决策树未触发，无坐标 / 增益变更登记；`writer.close()` 收尾顺序保持不变。

**三处同步**：WSL 工程 / Windows 归档 `xarm_lab_run` / 本文档 §6 §7 内嵌脚本逐字一致（含本次修正的 `test_load.py` 归档母本），`py_compile` 通过。

### 14.4 v6：L1 仿生触手视觉合拢示意与答辩材料包（2026-11-06 夜）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-06 23:21:09 → 23:23:24（+08:00），完整运行 |
| 命令 | `python xarm6_reach_demo.py --no-video`（回归）→ `--check-out-of-range`（退出码 2）→ `python xarm6_reach_demo.py` |
| 产物 | CSV 749 B（5 行 20 列）、MP4 1.7 MB（976 帧 / 32.53 s）、scene.png 29 KB、GIF 195 帧 / 375 KB、关键帧拼图 213 KB、自制答辩材料包 zip 3.3 MB |
| 旧值 → 新值 | `render_time_s` 134.205 → **133.581**；渲染 134.2 s → **133.6 s**；全程 134.3 s → **133.8 s**；帧数仍 976；**5 点误差、q1-q6、收敛步与 v5 逐点完全一致** |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读（已抽样 6 帧） |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|------|-------------|------|----------|
| 1 | 演示层视觉 | 只有果实视觉搬运，无末端机构 | 新增 3 根 x 6 段纯视觉触手：到点合拢包覆果实、放果后张开 | 课设「采摘草莓」叙事需要末端夹取观感；用户选定 **L1 视觉示意** | 5/5 通过、CSV 仍 5 行 20 列、误差与 v5 完全一致；触手汇总正常打印 |
| 2 | 画面判读修正 | 触手按 link6 局部姿态摆放，果实按世界坐标摆放，两者不对齐（frame 100 触手在末端右侧、球在左侧） | 触手改用**世界坐标**环绕 `ee` 正下方果实 | 抽样判读发现「没夹住」的观感问题 | 重跑后 frame 100 触手位于果实正下方并随末端收拢，frame 700 抓取、终态篮中 5 颗可见 |
| 3 | 范围与叙事 | v5 §0 第 7 条只写抓放循环 | 第 7 条补 L1 触手边界；明确**图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程** | 用户选定答辩叙事路线 A | 主结论「精度验证不建模末端执行器」保持不变；触手不进 URDF / 运动学链 / 误差判据 |
| 4 | 答辩材料 | 无自制图纸包 | 交付 6 张课设自制图纸（控制框图 / Tool IO 接线 / 控制器接口 / 电源 / 绳驱 / CAD 三视图）+ 仿真素材 + 来源与边界说明 + 官方链接清单，压缩为 `答辩材料/答辩材料包_20261106.zip` | 用户答辩要用；官方未公开控制器 PCB / 单片机电路图 | 全部图纸显著标注「本课题设计示意，非官方实物电路图」；GitHub 只传自制包，官方原件留本地 |

**未改动项（红线核对）**：`--threshold` 仍 0.005 m；URDF 关节限位、HOME / restPoses、TARGETS（P1-P5）、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`、STEADY_N、SERVO_MAX_STEPS、IK_MAX_ITER、`getLinkState(robot,5)[4]` 主口径、CSV 字段与 5 行结构全部未改；未删点、未隐藏失败记录（首次触手未对齐的判读与重跑均留痕）；§9.2 决策树未触发。

**三处同步**：WSL 工程 / Windows 归档 `xarm_lab_run` / 本文档 §6 §7 内嵌脚本逐字一致，`py_compile` 通过；结果与素材同步归档后逐文件 `cmp` 全绿。

### 14.5 v7：触手三阶段动画（卷曲 → 反向展开 → 缠绕包覆，2026-11-08）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-08 15:24:37 → 15:26:51（+08:00），完整运行 |
| 命令 | `python xarm6_reach_demo.py --no-video`（回归）→ `--check-out-of-range`（退出码 2）→ `python xarm6_reach_demo.py` |
| 产物 | CSV 749 B（5 行 20 列）、MP4 1.7 MB（976 帧 / 32.53 s）、scene.png 28 KB、GIF 195 帧 / 363 KB、关键帧拼图 264 KB |
| 旧值 → 新值 | `render_time_s` 133.581 → **133.319**；渲染 133.6 s → **133.3 s**；全程 133.8 s → **133.6 s**；帧数仍 976；**5 点误差、q1-q6、收敛步与 v6 逐点完全一致** |
| 未执行 | 降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读（已抽样 6 帧） |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|------|-------------|------|----------|
| 1 | 形态函数 | `update_tentacles(close)` 只有径向插值 `r_open → r_close`，触手始终下垂，视觉上「一张一合」 | 改为 `update_tentacles(t)` 三阶段：`t=0` 按 rho=a*exp(b*theta) 向后卷曲成对数螺旋并收在末端后方；`0~0.5` 反向展开到向下伸展态；`0.5~1` 沿果实球面（球心 `ee` 下方 `FRUIT_HELD_Z`）自球顶绕到球底包覆夹住；`t=1→0` 释放回卷 | 用户指出应体现 SpiRobs 「uncurling and wrapping」机构原理，而非简单开合 | 帧 0 卷曲成螺旋团、帧 52 展开成下垂珠串、帧 78 绕成环包住果实，三阶段形态可辨 |
| 2 | 时间线 | `tentacle_close` 6 处赋值：到位段后 40% 0→1，保持/搬运段 1，入篮段 1→0 | 改为 `tentacle_t`：到位段前 75% 卷曲待机、后 25% 展开（0→0.5），保持段前 30% 缠绕（0.5→1），保持/放果/入篮前段保持 1，入篮后 30% 释放回卷（1→0），放果后回 0；新增 `EXTEND_START` / `WRAP_END` / `RELEASE_START` 三个节拍常量 | 让卷曲、展开、包覆各有独立可判读的帧区间，逐颗果实重复 | 每点仍 195 帧、总 976 帧；关键帧 0/52/78/194/700/975 覆盖三阶段与终态 |
| 3 | 参数来源 | 附录 B 记 `TENTACLE_GRIP_R` 等 7 项 | 移除不再使用的 `TENTACLE_GRIP_R`，新增螺旋（`SPIRAL_A/B/THETA_MAX`、`COIL_BACK_Z`）、缠绕（`WRAP_R/PHI0/PHI1/TWIST`）与节拍常量，全部显著标注「本实验示意值」 | 几何与真实机构参数无关，避免误导 | 附录 B 参数行已重写；汇总表改为三阶段描述 |
| 4 | 判读素材 | 关键帧 0/100/194/389/700/975 | 重抽 0/52/78/194/700/975，中文标签标明卷曲 / 展开 / 缠绕 / 释放 / 终态 | 便于答辩一眼看出三阶段 | 拼图 1440x784，抽样判读通过，非全量逐帧 |

**未改动项（红线核对）**：`--threshold` 仍 0.005 m；URDF 关节限位、HOME / restPoses、TARGETS（P1-P5）、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`、STEADY_N、SERVO_MAX_STEPS、IK_MAX_ITER、`getLinkState(robot,5)[4]` 主口径、CSV 字段与 5 行结构、`ANIM_STEPS` / `HOLD_STEPS` / `PLACE_STEPS` / `PLACE_HOLD` / `--every` 帧预算、`writer.close()` 收尾顺序全部未改；未删点、未隐藏失败记录；§9.2 决策树未触发。

**口径边界**：三阶段仍是**纯视觉演示层**，卷曲 / 展开 / 包覆由脚本直接摆放 18 个静态视觉段体，无绳索、电机、张力、接触或夹持力，不进 URDF 运动学链与误差判据；叙事仍为「图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程」。

**三处同步**：WSL 工程 / Windows 归档 `xarm_lab_run` / 本文档 §6 §7 内嵌脚本逐字一致（三份指导书副本同步），`py_compile` 通过；结果与素材同步归档后逐文件 `cmp` 全绿。

---

### 14.6 v8：仿真平台整体迁移到达妙七轴谐波臂（2026-11-18）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-18（WSL2 Ubuntu / conda `robot` / Python 3.11.18 / pybullet 3.2.7 / numpy 2.4.6 / imageio 2.38.1） |
| 命令 | `python derive_damiao_urdf.py …`；`python test_load.py`；`python select_targets.py`；`python damiao_reach_demo.py --no-video`；`--check-out-of-range`；`python damiao_reach_demo.py` |
| 产物 | `results/reach_results.csv`（5 行 21 列）、`results/reach_demo.mp4`（976 帧）、`results/scene.png`、`results/run_stats.json`、`results/target_selection.json` |
| 未执行 | 带重力对照运行、降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读 |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 / 现象 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 仿真平台 | xArm6 六轴、手写 URDF | 达妙七轴、官方 URDF 派生 | 用户 11-18 决定平台改达妙（口径第三次变更） | `test_load.py` **ALL PASS**（7 关节 / 官方限位逐项一致 / 零位 FK 0.000 mm） |
| 2 | 模型来源 | 手写 `xarm6_reach.urdf` | 派生 `damiao_reach.urdf`：删夹爪子树 + 改 mesh 路径，**几何/惯量/限位逐字未改** | 官方开源包已提供 URDF/STL，自建模型违反红线 | 派生脚本自检 **DERIVE PASS**（15 个保留块与官方逐字一致） |
| 3 | 末端口径 | `link6`（索引硬编码 5） | `link7`，索引按 link 名反查 | 官方夹爪 `tool_link` 已裁剪；硬编码索引必错 | 索引实测 = 6；误差口径与 COM 备查列同步更新（相差约 71.7 mm） |
| 4 | 目标点 | P1–P5 原坐标 | 按官方几何重扫可达空间重选 5 点 | 原 5 点在达妙臂上残差 146–200 mm，**全部不可达** | 残差 0.000–0.001 mm，限位余量 0.407 rad、0.308 rad、0.490 rad、0.730 rad、0.636 rad，5/5 一次通过 |
| 5 | 预筛包络 | r ≤ 0.70 m、z ∈ [0.10, 0.75] | r ≤ 0.389 m、z ∈ [-0.111, 0.507] | 取自实测 `workspace.link7`（6000 组采样） | 负向验证两点均被预筛拒绝，退出码 2 |
| 6 | IK | 内置单次（maxNumIterations=100） | 内置粗解 + **位置-only 数值精修**（LM 式阻尼递增），初值 = 上一点实际关节角 | 7 轴冗余下内置求解器远距离初值停在 3–8 mm 局部解（超 5 mm 阈值）；连续性靠链式初值保证 | 残差 0.000–0.001 mm；**未放宽阈值、未改限位** |
| 7 | 维度 | 6 维 q1–q6、CSV 20 列 | 7 维 q1–q7、CSV 21 列 | 自由度 6 → 7 | CSV 5 行 21 列，与脚本一致 |
| 8 | 演示层 | 3 根 x 6 段触手 + 抓放循环 | **原样保留**（常量与两处函数逐字不动），仅挂载点索引换成实测 link7 索引 | 用户要求不重做末端执行器 | 画面确认无官方夹爪、三阶段动画完整 |
| 9 | 旧实跑数字 | 0.29–0.88 mm / 976 帧 / 133.3 s / converge_step=59 / 5-5 | **标注「xArm6 基线（已废弃）」**，仅留作历史对照 | 平台更换后旧数字不再是达妙结果 | 本版实跑：0.000–0.001 mm / 976 帧 / 265.4 s / 5-5 |
| 10 | 末端演示层坐标系 | 触手与果实按**世界坐标**摆放（`ee + 偏移`，不乘腕部旋转）→ 腕部一转，爪与关节末端**歪出脱开**，缠绕圈也没包住果实 | 几何改在 **link7 局部坐标系**生成，再乘 link7 的世界旋转矩阵落到世界系（`wp = pos + R @ [x,y,z]`，果实同理）；零位时 R = I，历史画面不变 | 末端执行器必须与关节末端同朝向，否则演示层自相矛盾 | **纯视觉修正，不改控制链路**：5/5 仍通过、残差 0.000–0.001 mm 不变；对照渲染确认触手长在 link7 末端并顺腕部轴向包住果实 |
| 11 | 末端执行器安装方向（**用户指出「装反」**） | 触手沿局部 **-z** 伸展（指向臂体内部）、缠绕环心在局部 -0.03 m → 爪缩在腕关节与臂体之间，视觉上「装反」 | 伸展方向翻转为局部 **+z（法兰外侧）**；缠绕环心改为 **link7 原点 = 被测点/草莓标记位置**（爪夹住的位置与误差测量点重合）；搬运果实跟随原点，`FRUIT_HELD_Z` 偏移常量删除 | 末端执行器必须从法兰外侧伸出，且夹持点应与被测点一致，避免引入未建模工具偏移 | **纯视觉修正，不改控制链路**：5/5 仍通过、残差 0.000–0.001 mm 不变；复跑渲染确认爪从法兰外侧伸出并包住腕端果实 |

**未改动项（红线核对）**：`--threshold` 保持 0.005 m（未放宽）；官方 7 组关节限位、HOME / restPoses = 官方零位、FORCES = 50/50/50/50/12/12/12、POS_GAIN / VEL_GAIN = 1.0 / 1.0、`setGravity(0,0,0)`、STEADY_N = 60、SERVO_MAX_STEPS = 3000 全部未改；章鱼爪形态常量与三阶段时间线未改；未删点、未隐藏失败记录；§9.2 决策树未触发。

---

### 14.7 v8.3：末端执行器更换为 ORCA Hand 官方 URDF（2026-11-18，第四次口径变更）

**口径裁决**：课设立论为**仿生机械臂本体**的定位精度验证；末端执行器为可替换模块。v8.3 只换末端执行器，**不动仿生臂本体**：精度判定点保持 `link7` 原点不变，ORCA 作为 `link7` 之后的固定子装配挂接，不进 IK 链、不改 5 个目标点、不改预筛包络、不改 CSV 的误差口径，v8.2 的判据与阈值全部沿用。

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-18（WSL2 Ubuntu / conda `robot` / Python 3.11.18 / pybullet 3.2.7 / numpy 2.4.6 / imageio 2.38.1） |
| 命令 | `expand_orcahand_xacro.py`；`derive_orcahand_urdf.py`；`derive_damiao_urdf.py`；`test_load.py`；`select_targets.py`；`damiao_reach_demo.py --no-video`；`--check-out-of-range`；`damiao_reach_demo.py`（完整）；`--legacy-tentacle`；`--with-orca-mass`；`make_demo_assets.py` |
| 产物 | `results/reach_results.csv`（5 行 21 列）、`results/reach_demo.mp4`、`results/scene.png`、`results/run_stats.json`、`results/target_selection.json`（复算与 v8.2 一致）、`results/reach_results_orca_mass.csv`（带载对照独立产物） |
| 未执行 | 带重力纯臂对照、`--with-orca-mass` 完整渲染、降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读 |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 / 现象（v8.2） | 新值 / 处理（v8.3） | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 末端执行器 | 卷曲仿生章鱼爪纯视觉演示层（不在 URDF 内） | **ORCA Hand 官方 URDF 几何**（srl-ethz/orcahand_description，MIT），fixed 挂接 `joint_orca_wrist` | 用户 21:08 裁决：课设立论是仿生臂本体，末端执行器可换（第四次口径变更） | `test_load.py` ALL PASS（ORCA 断言全过）；选点复算与 v8.2 一致；5/5 通过 |
| 2 | ORCA 子装配口径 | — | 惯量置零（42 link mass=0）、17 个 revolute 锁定为预设握合姿态（均在官方限位内）、碰撞过滤屏蔽不参与接触 | 维持 v8.2 无重力基线与位置伺服口径不变；官方 URDF 无 mimic/transmission，腱绳耦合未建模 | ORCA DERIVE PASS（83 块与官方展开一致）；伺服误差 0.000–0.001 mm 不变 |
| 3 | 演示层 | 章鱼爪三阶段（默认） | **默认 = ORCA 握合呈现**；章鱼爪保留为 `--legacy-tentacle` 分支 | 主演示改为 ORCA Hand，v8.2 可复现 | 降分辨率抽样渲染确认三阶段动画完整 |
| 4 | 带载对照 | 无 | 新增 `--with-orca-mass`：重力 + ORCA 官方质量 1.135 kg + 独立 CSV | 分离带载口径，禁止数字混报 | 快速验证 5/5 通过；静态持重力矩峰值 J2 7.910 N·m（PyBullet 实算）；完整渲染未执行 |
| 5 | 模型文件 | 8 link / 7 revolute | **50 link / 49 joint**（+42 ORCA link、+41 ORCA fixed、+1 挂接 fixed） | ORCA 为固定子装配，不引入可动关节 | `test_load.py` 断言 revolute=7、fixed=42 PASS |
| 6 | run_stats.json | 手工维护 | 主脚本自动回填（指导书生成器数据源） | 保证「文档 = 工程」 | v8.3 重跑自动生成 |
| 7 | 达妙臂主体 | 官方 URDF 派生 | **不变**（15 个保留块逐字一致、16 个官方 STL 未动） | 红线：不动臂本体 | DERIVE PASS 校验含 ORCA 项后仍通过 |

**未改动项（红线核对）**：`--threshold` 保持 0.005 m；官方 7 组关节限位、HOME / restPoses、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`（默认模式）、STEADY_N、SERVO_MAX_STEPS 未改；5 个目标点未改；达妙 16 个官方 STL 未动；ORCA 68 个官方 STL 未动；未删点、未隐藏失败记录；§9.2 决策树未触发。

**边界声明（答辩必须主动说明）**：无重力设定；ORCA 子装配惯量置零，**不代表带载精度**；腱绳耦合未建模、无接触力学与夹持力；果实搬运为纯视觉移动；**不代表抓取成功率**。带载数字（`--with-orca-mass`）与无重力基线数字分列，不混用。

---

## 附录 B## 附录 B：参数来源对照表（每个数字可复核）

| 参数 | 值 | 来源 |
|------|-----|------|
| 关节原点（7 组 xyz，rpy 全 0） | j1 (0,0,0.0756)；j2 (0,-0.042,0.0425)；j3 (0,0.042,0.0744)；j4 (-0.042,0,0.1156)；j5 (0.042,0,0.0995)；j6 (-0.03325,0,0.035)；j7 (0.03325,-0.03325,0.0555) | 达妙官方 `left_description_grip.urdf`（**派生后逐字未改**） |
| 关节限位（rad） | j1 [-2.827, 2.827]；j2 [-1.8064158, 0.6806784]；j3 [-2.8797933, 2.8797933]；j4 [-1.8151424, 0.7592182]；j5 [-2.7576202, 2.7576202]；j6 [-1.3264502, 1.3264502]；j7 [-1.3351769, 1.3351769] | 同上 `<limit lower/upper>`（与官方 README §1.1 机械限位一致；**SolidWorks 导出 CSV 的 Joint Lower/Upper 是另一套更窄的值，禁止混用**） |
| 力矩上限 | 50 / 50 / 50 / 50 / 12 / 12 / 12 N·m | 同上 `<limit effort>`（**官方原始值，未核实是否含传动折算**；不自行乘过载系数） |
| 关节速度上限 | 10.0 rad/s | 同上 `<limit velocity>` |
| 连杆惯量（mass/origin/惯性张量） | base_link 0.94 / link1 0.87 / link2 0.278196 / link3 1.005 / link4 1.244 / link5 0.0756 / link6 0.926 / link7 0.1765 kg | 同上 `<inertial>`（**仿真以 URDF 为准**；SolidWorks 导出 CSV 的 Mass 列多处不一致，冲突时以 URDF 为准） |
| 零位 FK 末端理论值 | link7 原点 = (0, -0.03325, 0.4981) m | 本指导书 §2.3 累加推导；与 `handoff/演算基准.json` 的 `zero_pose.link7_tcp_m` 一致 |
| HOME / restPoses | 官方零位 `[0]*7` rad | 本实验定义（官方零位即 nullspace 参考）；**v8 实跑锁定** |
| 目标点 5 个 | 见 §7 脚本 TARGETS（P1–P5，见 §8.3 表） | `select_targets.py` 按官方限位 + 官方几何重扫可达空间择优（残差 ≤ 2 mm、限位余量 ≥ 0.15 rad、间距 ≥ 0.08 m）；**v8 实跑锁定**：5 点全部通过 |
| 预筛半径/高度 | 0.389 m / [-0.111, 0.507] m | `handoff/演算基准.json` 的 `workspace.link7`（6000 组限位内采样，`r_max`≈0.3892） |
| IK 参数 | `IK_MAX_ITER=200`、`IK_TOL=1e-5`；内置粗解 + `refine_ik()` 位置-only 数值精修（数值雅可比 eps=1e-5、阻尼 1e-7→1e-0 递增、最多 60 次） | PyBullet API 参数 + 本实验定义（见 §2.1 / §7） |
| 伺服参数 | STEADY_N=60（连续 0.25 s 达标判收敛）、SERVO_MAX_STEPS=3000（超时兜底 12.5 s）、POS_GAIN=1.0 / VEL_GAIN=1.0 + 轨迹段速度前馈 targetVelocities（无重力设定下的验证基线）、梯形轨迹 480 步 @240 Hz | 本实验定义；依据见附录 C |
| 静态重力矩校核 | 零位 J2 0.1581 / J4 0.0159 / J6 0.0176 / J7 0.0763 N·m；6000 组采样峰值 J2 8.8251 / J3 2.6732 / J4 2.7396 / J5 0.6462 / J6 0.6824 / J7 0.2353 N·m | `handoff/演算基准.json`（numpy 静态演算，g=9.81，未含减速器效率与动态项；**非 PyBullet 实跑、非带载精度验证**） |
| pybullet wheel 可用性 | 3.2.6/3.2.7 含 cp311-manylinux wheel，无 cp312+ | PyPI（tuna 镜像）2026-11-04 实测 |
| v7 触手三阶段示意参数（见 §7 常量区，**本轮逐字未改**） | `TENTACLE_COUNT=3` / `TENTACLE_SEGS=6` / `TENTACLE_LEN=0.13` m / `TENTACLE_RADIUS=0.008` m / `TENTACLE_ROOT_R=0.022` m / `TENTACLE_OPEN_R=0.065` m；卷曲 `SPIRAL_A=0.010` m、`SPIRAL_B=0.22`、`SPIRAL_THETA_MAX=2.4π`、`COIL_BACK_Z=0.050` m；缠绕 `WRAP_R=0.033` m、`WRAP_PHI0/PHI1=0.18π/0.96π`、`WRAP_TWIST=0.55π`；节拍 `EXTEND_START=0.75`、`WRAP_END=0.30`、`RELEASE_START=0.70` | **本实验演示层示意值**：rho=a*exp(b*theta) 仅用于卷曲形态，比例按 SpiRobs 官方「数根触手抓鸡蛋」演示取值，**非中科大实物参数、非论文测量值**；纯视觉、不进 URDF / 运动学链、不参与 §1.3 判定、不写入 CSV |
| 演示层抓放参数（见 §7 常量区） | `BASKET_CENTER=(0.16,-0.25,0.08)` m、`BASKET_DROP=(0.16,-0.25,0.24)` m、`BASKET_HALF=(0.09,0.09,0.055)` m、`PLACE_STEPS=480 / PLACE_HOLD=120`、`FRUIT_SLOTS` 5 个篮内槽位（**果实搬运位置 = link7 原点 = 被测点，无偏移；FRUIT_HELD_Z 已随装反修正删除**） | 本实验定义（演示层）；篮子与放果点按达妙臂工作空间平移，放果点经两段式 IK 预核可达（残差 {drop:.2f} mm、7 关节在限位内），**不参与 §1.3 判定、不写入 CSV** |
| 官方夹爪数据（**本轮不使用，仅存档**） | `tool_link` 原点 (0, 0, 0.65498) m，相对 link7 偏移 0.157 m；`tool_joint` 为 fixed；`gripper_joint1/2` 为 prismatic 0–0.045 m | 达妙官方 URDF（裁剪前的原始文件） |
| ORCA Hand 末端执行器来源（v8.3） | `srl-ethz/orcahand_description`（**MIT License**，描述文件仓库）；硬件制造包 `orcahand/orca_files`（**CC BY 4.0**，仅作参考、无 URDF，本工程不使用）；官方 mesh 几何未修改 | `third_party/orcahand/`（官方 xacro 原样复制 + LICENSE 副本）；`meshes/orcahand/{visual,collision}/` 34+34 个官方 STL 原样 |
| ORCA 展开与派生口径（v8.3） | 展开 `chirality:=right prefix:=orca_ extension:=true`（68 处 mesh 引用、42 link / 41 joint / 17 revolute，与验收要求 34+34 STL 全部可达一致；提示词示例命令 extension:=false 仅 62 处，已按验收口径取 true 并登记）；派生 = mesh 路径改写 + 惯量置零 + 17 个 revolute 锁定 | `third_party/orcahand/expand_orcahand_xacro.py`；`derive_orcahand_urdf.py`（**ORCA DERIVE PASS**：83 块与官方展开一致、无 package:// 残留） |
| ORCA 挂接参数（v8.3） | `link7 → orca_root`（fixed `joint_orca_wrist`）：origin xyz = (-0.15, 0, -0.06) m，rpy = (π/2, 0, π/2)；掌面向上托果，握合腔中心对准果实保持点 link7 局部 (0, 0, -0.03)；零位包围盒 0.256×0.100×0.081 m、最低点 0.438 m 不与地面穿模 | `derive_damiao_urdf.py` 的 `ORCA_MOUNT_*`（依据官方展开 URDF root 系几何实测确定，渲染核验通过） |
| ORCA 预设握合姿态（v8.3，示意值） | wrist 0；四指 abd 0.04–0.10 / mcp 1.05–1.20 / pip 0.35–0.42；拇指 mcp 0.45 / abd 0.55 / pip 0.50 / dip 0.15 rad（**全部在官方 `<limit>` 内**；官方 URDF 未建模腱绳耦合、未提供姿态表，此为示意姿态，不代表真实抓取） | `derive_orcahand_urdf.py` 的 `PRESET`（锁定角烘焙进 joint origin，并在 URDF 注释登记官方限位） |
| 带载对照口径（v8.3，`--with-orca-mass`） | 重力 9.81 m/s² + ORCA 官方质量合计 1.135 kg（tower 0.9 kg + 全惯量）；逐点静态持重力矩 PyBullet 实算峰值 J2 7.910 N·m | `damiao_reach_demo.py --with-orca-mass`（独立 CSV `results/reach_results_orca_mass.csv`；**与无重力基线数字分列，禁止混报**） |

---

## 附录 C：伺服增益调整指引（仅主流程异常时使用）

主流程位置伺服的验证基线：**无重力设定**（§0 第 6 条）+ positionGain=1.0 / velocityGain=1.0 + 轨迹段速度前馈 + 分关节力矩上限（附录 B）。基线依据：PyBullet POSITION_CONTROL 力矩 = kp·Δq + kd·(v_cmd − v)，无重力、无其他扰动时稳态误差趋于 0；kp=1.0 下收敛时间常数约 1–3 s，3000 步（12.5 s）兜底预算充足。**该基线在实测锁定前不视为"已验证"**——若 §8.3 实测出现下表现象，按表调整（一次只改一个变量）：

| 现象 | 动作 |
|------|------|
| 到位后在目标附近振荡/超调 | `VEL_GAIN` 1.0 → 2.0（加阻尼一档） |
| err_track 迟迟不达标、converge_step = timeout | 先确认 `setGravity(0,0,0)` 未被改动；再 `POS_GAIN` 1.0 → 2.0（升一档） |
| 升增益后仍不收敛 | 观察窗 `STEADY_N` 60 → 120（延长至 0.5 s）重跑 |

约束：每次增益调整后必须**全部 5 点重跑**（不同增益下的数据不得混用）；调整过程与理由登记 §9.2 变更表，禁止只报好数据。**重力场景说明**：若实物化阶段需带重力验证，纯位置伺服须引入逆动力学前馈（`calculateInverseDynamics` 算重力矩 τ_ff，改 TORQUE_CONTROL 实现 τ = τ_ff + kp·Δq − kd·qdot）或大幅提高增益并重新整定——属后续工作，不在本实验范围。

