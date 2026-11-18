# 变更记录：末端执行器更换为 ORCA Hand 官方 URDF（v8.3，2026-11-18）

> 依据《PROMPT_ORCA末端执行器_v8.3_RELEASE.md》（生成时间 2026-11-18 21:08）执行。
> **第四次口径变更**：课设立论为**仿生机械臂本体**的定位精度验证；末端执行器为可替换模块，v8.3 用 ORCA Hand 灵巧手。只换末端执行器，**不动仿生臂本体**。

## 一、口径裁决（继承 v8.3 提示词三条默认裁决）

1. **精度判定点保持 `link7` 原点不变**：ORCA Hand 作为 `link7` 之后的固定子装配挂接，不进 IK 链、不改 5 个目标点、不改预筛包络、不改 CSV 的误差口径。v8.2 的 5/5 通过、0.000–0.001 mm 结果可完整复现（v8.3 重跑实测一致）；
2. **ORCA 子装配惯量置零、不启用接触**：保留官方几何 mesh 与 link/joint 结构，全部 `<inertial>` mass 置 0，collision mesh 保留但运行时碰撞过滤屏蔽（group=2/mask=0）。另做 `--with-orca-mass` 带载对照分支（独立脚本开关 + 独立 CSV + 静态力矩重算），不与无重力基线数字混报；
3. **主演示改为 ORCA Hand，章鱼爪层保留为 `--legacy-tentacle` 开关**：默认运行必须是 ORCA Hand。

## 二、变更登记（旧值 → 新值 → 原因 → 重跑结果）

| # | 类别 | 旧值 / 现象（v8.2） | 新值 / 处理（v8.3） | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 末端执行器 | 卷曲仿生章鱼爪纯视觉演示层（不在 URDF 内） | **ORCA Hand 官方 URDF 几何**（srl-ethz/orcahand_description，MIT），fixed 挂接 `joint_orca_wrist` | 用户 2026-11-18 21:08 裁决（第四次口径变更） | `test_load.py` ALL PASS（ORCA 断言全过）；选点复算与 v8.2 逐位一致；5/5 通过 |
| 2 | ORCA 子装配口径 | — | 惯量置零（42 link mass=0）、17 个 revolute 锁定为预设握合姿态（均在官方限位内）、碰撞过滤屏蔽不参与接触 | 维持 v8.2 无重力基线与位置伺服口径；官方 URDF 无 mimic/transmission，腱绳耦合未建模 | ORCA DERIVE PASS（83 块与官方展开一致）；伺服误差 0.000–0.001 mm 不变 |
| 3 | 演示层 | 章鱼爪三阶段（默认） | **默认 = ORCA 握合呈现**；章鱼爪保留为 `--legacy-tentacle` 分支 | 主演示改为 ORCA Hand，v8.2 可复现 | 降分辨率抽样渲染（320×240）确认三阶段动画完整 |
| 4 | 带载对照 | 无 | 新增 `--with-orca-mass`：重力 9.81 m/s² + ORCA 官方质量 1.135 kg + 独立 CSV | 分离带载口径，禁止数字混报 | 快速验证 5/5 通过；静态持重力矩峰值 J2 7.910 N·m（PyBullet 实算）；完整渲染未执行 |
| 5 | 模型文件 | 8 link / 7 revolute | **50 link / 49 joint**（+42 ORCA link、+41 ORCA fixed、+1 挂接 fixed） | ORCA 为固定子装配，不引入可动关节 | `test_load.py` 断言 revolute=7、fixed=42 PASS |
| 6 | mesh 资产 | 达妙 16 个官方 STL | + ORCA 68 个官方 STL（34 visual + 34 collision，原样未改，7.1 MB） | 官方几何不得修改 | 派生校验 68 处路径全部可达 |
| 7 | run_stats.json | 手工维护 | 主脚本自动回填（指导书生成器数据源）；`--no-video` 写独立 `run_stats_no_video.json` | 保证「文档 = 工程」 | v8.3 重跑自动生成 |
| 8 | 达妙臂主体 | 官方 URDF 派生 | **不变**（15 个保留块逐字一致、16 个官方 STL 未动） | 红线：不动臂本体 | DERIVE PASS 校验含 ORCA 项后仍通过 |

### 提示词内部矛盾登记

- 提示词 §4.2 示例命令为 `extension:=false`（展开后仅 62 处 mesh 引用），与 §4.3 / §八 验收要求的「68 处 package:// 改写、34+34 STL 全部可达」矛盾；本工程**按验收口径取 `extension:=true`**（68 处 mesh 引用、34+34 官方 STL 全部被引用），已在 `expand_orcahand_xacro.py` 注释与本记录登记；
- 提示词 §4.3 称 xacro 为「1164 行 / 68 处 mesh」：实测源文件 1164 行 ✓；68 处 mesh 引用对应 `extension:=true` ✓。

## 三、挂接与预设姿态依据（如实说明）

- **挂接变换**：`link7 → orca_root`（fixed `joint_orca_wrist`），origin xyz = (-0.15, 0, -0.06) m，rpy = (π/2, 0, π/2)。依据：官方展开 URDF 在 root 系中的几何实测（手指沿 root +z 延伸、卷向 +y、掌面朝 +y）；挂接后掌面向上托果、握合腔中心对准果实保持点 link7 局部 (0, 0, -0.03)；渲染核验目标球可见、零位不与地面穿模（最低点 0.438 m）；
- **预设握合姿态（示意值）**：官方 URDF 未建模腱绳耦合、也未提供官方姿态表，故取一组**位于官方限位内**的固定角度：wrist 0；四指 abd 0.04–0.10 / mcp 1.05–1.20 / pip 0.35–0.42；拇指 mcp 0.45 / abd 0.55 / pip 0.50 / dip 0.15 rad。锁定角烘焙进 joint origin 并在 URDF 注释登记官方限位；**不代表任何真实抓取姿态**；
- **ORCA 关节限位**：官方 17 个 revolute 的 `<limit>` 在展开 URDF（`third_party/orcahand/orcahand_right.urdf`）中原样保留；仿真派生 URDF 锁定为 fixed（注释登记官方限位数值）。

## 四、文件清单

| 动作 | 文件 |
|---|---|
| **保留逐字不动** | `damiao_reach.urdf` 达妙官方 15 个保留块（派生校验）、`meshes/damiao/` 16 个官方 STL、两段式 IK / 梯形轨迹 / 位置伺服判据、5 mm 阈值、5 个目标点（复算一致）、`实验指导书_xarm6草莓定点到达.md`、`答辩材料/`、`变更记录_达妙七轴迁移_20261118.md` |
| **修改** | `damiao_reach.urdf`（追加 ORCA 子装配 + 挂接 joint）、`damiao_reach_demo.py`（ORCA 演示层 + `--legacy-tentacle` + `--with-orca-mass` + run_stats 自动回填）、`derive_damiao_urdf.py`（追加 ORCA 子装配与校验）、`test_load.py`（49 link 断言 + ORCA 断言 + 包围盒断言）、`README.md`、`实验指导书_damiao七轴草莓定点到达.md`（经 `make_damiao_guide.py` 重新生成）、`make_damiao_guide.py`、`make_demo_assets.py`（ORCA 口径标签 + `--legacy-tentacle`/`--mp4` 参数） |
| **新增** | `derive_orcahand_urdf.py`、`third_party/orcahand/{orcahand.urdf.xacro, expand_orcahand_xacro.py, orcahand_right.urdf, orcahand_right_sim.urdf, LICENSE_orcahand_description}`、`meshes/orcahand/{visual,collision}/`（34+34 STL）、`results/reach_results_orca_mass.csv`、`results/run_stats_no_video.json`、`results/run_stats_orca_mass.json`、`results/reach_demo_legacy_tentacle.mp4`、本文件、`RELEASE_NOTES_v8.3.md` |
| **重新生成** | `results/reach_demo.mp4`、`reach_demo.gif`、`reach_keyframes.png`、`scene.png`、`reach_results.csv`、`run_stats.json`、`target_selection.json`（复算，与 v8.2 一致） |
| **未动** | `v8.2-damiao-7axis` Release、`xarm6-reach-lab` 仓库、课设交付物目录、答辩材料 |

## 五、v8.3 重跑实测汇总（2026-11-18，WSL2 / conda robot / Python 3.11.18 / pybullet 3.2.7）

| 检查项 | 结果 |
|---|---|
| `derive_orcahand_urdf.py` | **ORCA DERIVE PASS**（68 处 mesh 改写全部可达、42 link / 41 fixed、无 package:// 残留、83 块一致） |
| `derive_damiao_urdf.py` | **DERIVE PASS**（50 link / 49 joint、达妙 15 保留块逐字一致、ORCA 校验全过） |
| `test_load.py` | **ALL PASS**（49 link、7 限位逐项 PASS、link7 索引 = 6、零位 FK 0.000 mm、ORCA 断言 / 包围盒 PASS、渲染 307200 bytes） |
| `select_targets.py` | 5 点坐标 / 残差 / 限位余量**与 v8.2 记录逐位一致** |
| 快速验证 `--no-video` | **5/5 通过**（0.6 s） |
| 负向验证 `--check-out-of-range` | **退出码 2**（两点均在 IK 前被预筛拒绝） |
| **完整运行** | **FULL_EXIT=0**，5/5 到达，`err_ik`/`err_track` **0.000–0.001 mm**（阈值 5 mm 未放宽），`converge_step=59`，限位余量 0.308–0.730 rad，**976 帧 @ 960×720**，渲染 **265.4 s**，全程 266.0 s，CSV **5 行 21 列** |
| `--legacy-tentacle` | 快速验证 5/5 通过；降分辨率抽样渲染 491 帧 / 54.3 s，三阶段动画完整 |
| `--with-orca-mass`（独立口径） | 快速验证 5/5 通过；静态持重力矩峰值 J2 7.910 / J3 2.680 / J4 2.899 / J5 0.860 / J6 0.354 / J7 0.838 N·m（PyBullet 实算，含 ORCA 官方质量 1.135 kg）；完整渲染未执行 |
| MP4 校验 | 完整解码 976 帧（30 fps / 32.53 s）；**6 帧抽样判读正常（抽样，非全量）** |
| 指导书一致性 | `make_damiao_guide.py` 重新生成，内嵌 5 个脚本与工程文件**逐字一致**（md5 校验通过） |

## 六、口径边界声明（答辩必须主动说明）

- 无重力设定（`setGravity(0,0,0)`）；ORCA 子装配**惯量置零**以维持 v8.2 轨迹跟踪基线，**不代表带载精度**；
- **腱绳耦合未在官方 URDF 中建模**（无 mimic/transmission）：本仿真将 ORCA 作为固定子装配呈现，**不含腱绳驱动、接触力学与夹持力**；预设握合姿态为示意值；
- 果实搬运为**纯视觉移动**；**不代表真实抓取 / 抓取成功率**；
- `--with-orca-mass` 带载数字与无重力基线数字**分列记录、禁止混报**；
- 官方未公开的指标（ORCA 额定负载、重复精度、达妙减速器效率折算等）一律标「待核实」，不冒充官方数据。

## 七、本次未做、需后续单独裁决的事项

1. 带重力**纯臂**对照运行（不带 ORCA 质量）；
2. `--with-orca-mass` 带载对照的完整渲染与素材派生；
3. ORCA 腱绳耦合建模（官方 URDF 无该信息，需另行设计）；
4. 真实接触抓取仿真（接触力学 / 夹持力 / 抓取成功率）；
5. 课设答辩材料（PPT / 报告 / 说明书）与 v8.3 口径的同步（本次范围外）；
6. 全新机器零探索复跑。
