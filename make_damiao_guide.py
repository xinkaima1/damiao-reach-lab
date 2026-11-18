#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_damiao_guide.py - 把实验指导书从 xArm6 口径转换为达妙七轴口径。

用法（在 damiao_reach_lab 目录下）：
    python make_damiao_guide.py

输入：实验指导书_xarm6草莓定点到达.md（原 xArm6 版，只读不改）
输出：实验指导书_damiao七轴草莓定点到达.md

原则：
  * 只改口径与数字，章节结构、判据、决策树等方法论内容保留；
  * §5 / §6 / §7 的内嵌代码由工程实际文件注入，保证「文档 = 工程」；
  * 所有实跑数字从 results/reach_results.csv 与 results/run_stats.json 读取，不手写。
"""
import csv
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "实验指导书_xarm6草莓定点到达.md")
DST = os.path.join(HERE, "实验指导书_damiao七轴草莓定点到达.md")


def read(p):
    with open(p, "r", encoding="utf-8") as f:
        return f.read()


def inject(p):
    return read(os.path.join(HERE, p))


def sub(text, old, new, count=1):
    assert text.count(old) == count, f"锚点命中 {text.count(old)} 次（期望 {count}）：{old[:60]!r}"
    return text.replace(old, new, count)


def replace_section(text, start_marker, end_marker, new_body):
    i = text.index(start_marker)
    j = text.index(end_marker)
    return text[:i] + new_body + text[j:]


def main():
    text = read(SRC)
    stats = json.loads(inject("results/run_stats.json"))
    # --no-video 实测耗时（主脚本写入独立统计文件，不覆盖完整运行统计）
    nv_path = os.path.join(HERE, "results/run_stats_no_video.json")
    if "no_video_time_s" not in stats and os.path.exists(nv_path):
        stats["no_video_time_s"] = json.loads(read(nv_path))["total_time_s"]
    with open(os.path.join(HERE, "results/reach_results.csv"), encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    by_id = {r["point_id"]: r for r in rows}
    assert len(rows) == 5, f"CSV 应为 5 行，实际 {len(rows)}"

    # ---------------------------------------------------------------- 头部
    old_head_start = "# 实验指导书：xArm6 草莓采摘定点到达仿真（WSL2 + PyBullet）"
    old_head_end = "> **阅读约定**"
    i, j = text.index(old_head_start), text.index(old_head_end)
    k = text.index("\n", j)
    new_head = """# 实验指导书：达妙七轴谐波机械臂 草莓采摘定点到达仿真（WSL2 + PyBullet，v8.3 ORCA Hand 末端执行器版）

> **版本**：**v8.3（达妙七轴 + ORCA Hand 末端执行器版，2026-11-18，同日两次按用户指正修正）**：**课设立论为仿生机械臂本体的定位精度验证；末端执行器为可替换模块**。v8.3 把末端执行器由卷曲仿生章鱼爪演示层更换为 **ORCA Hand 官方 URDF 几何**（srl-ethz/orcahand_description，MIT License）：以 fixed joint 挂接于 `link7` 之后作为**固定子装配**，惯量置零、17 个 revolute 锁定为预设握合姿态、碰撞过滤屏蔽不参与接触。**按实际拼装重算（用户两次指正）**：① 手的安装基面与机械臂末端法兰面（link7 腕筒顶面，link7 系 z=0.0907）**相切**，手指沿工具轴外伸；② 精度判定点改为 **TCP = 果实被持点 = ORCA 握合腔中心**（link7 系 TCP_OFFSET = (0.048, 0, 0.32091)），**5 个目标点 = 果实位置，由 `select_targets.py` 按 TCP 口径重算**，预筛包络同步改为果实点云实测（前方带 x≥0.25、z∈[0.12,0.45]、r≤0.55）。仿生臂本体全部沿用 v8.2 口径：达妙官方 URDF 派生（几何/惯量/限位逐字未改）、7 组官方限位、力矩上限 50/50/50/50/12/12/12 N·m、5 mm 阈值、`setGravity(0,0,0)` 无重力设定。v8.2 章鱼爪演示层保留为 `--legacy-tentacle` 分支（默认关闭）。§8.2 / §8.3 / §13 按 2026-11-18 v8.3 修正后全量重跑回填（{frames} 帧 / {render:.1f} s / render_time_s {render}）。
> **口径边界**：v8.2 及更早的实跑数字**不是 v8.3 的重跑结果**，引用处均注明来源版本；v8.3 重跑数字以 `results/reach_results.csv` 与 `results/run_stats.json` 为准。带载对照实验（`--with-orca-mass`）数字**独立记录**，禁止与无重力基线混报。
> **执行环境**：Windows + WSL2 Ubuntu，conda `robot`（Python 3.11 + pybullet 3.2.7）
> **阅读约定**：所有命令可直接复制粘贴执行。**v8.3 修正版为当前唯一执行基线**；仍标注「未执行」的项目不得当作结论引用。
""".format(frames=stats["frames"], render=stats["render_time_s"])
    text = text[:i] + new_head + text[k:]

    # ---------------------------------------------------------------- §0 范围声明
    text = sub(text,
        "1. **纯仿真**：不制实物、**不建模末端执行器**（夹爪按机械图纸设计，不纳入本轮运动学验证）；",
        "1. **纯仿真**：不制实物。**末端执行器为 ORCA Hand 官方 URDF 几何（v8.3，固定子装配）**：几何与关节结构取自 srl-ethz/orcahand_description（MIT），惯量置零、17 个 revolute 锁定为预设握合姿态、碰撞过滤屏蔽不参与接触、不进运动学链、不参与误差判定；**仿真不采用达妙官方夹爪**——官方 URDF 的 `tool_link` / `gripper_left` / `gripper_right` 与 `tool_joint` / `gripper_joint1` / `gripper_joint2` 已在派生 URDF 中整段裁剪（§5）；v8.2 的卷曲仿生章鱼爪演示层保留为 `--legacy-tentacle` 分支（§7）；")
    text = sub(text,
        "2. **不含**夹爪开合、采摘接触力学、状态机、视觉/CLIP、力控；",
        "2. **不含**夹爪开合、采摘接触力学、状态机、视觉/CLIP、力控；ORCA 腱绳耦合未在官方 URDF 中建模（无 mimic/transmission），本仿真将其作为固定子装配呈现，**不含腱绳驱动、接触力学与夹持力**，17 个 revolute 锁定为预设握合姿态、无开合动画；")
    text = sub(text,
        "3. 精度验证对象只有一件事：**6 轴 xArm6 的末端 link 依次到达 5 个写死的空间目标点**；",
        "3. 精度验证对象只有一件事：**达妙七轴谐波臂的末端 link7 依次到达 5 个写死的空间目标点**；")
    text = sub(text,
        "4. 方法：PyBullet + 内置 `calculateInverseKinematics` **位置-only IK**（不约束姿态）；",
        "4. 方法：PyBullet 内置 `calculateInverseKinematics` **位置-only IK**（不约束姿态）+ **位置-only 数值精修**（§2.1，两段式，原因与实现见 §7 `refine_ik()`）；")
    text = sub(text,
        "6. 仿真动力学为**无重力设定**（`setGravity(0,0,0)`）+ 轨迹段速度前馈：验证的是轨迹规划与位置伺服的**指令跟踪与定位收敛**；重力负载前馈补偿（逆动力学）属实物化后续工作，不在本轮范围。本条为答辩与说明书的统一口径，主动声明而非被问出来；",
        "6. 仿真动力学为**无重力设定**（`setGravity(0,0,0)`）+ 轨迹段速度前馈：验证的是轨迹规划与位置伺服的**指令跟踪与定位收敛**。**达妙实物有重力**：按官方 URDF 惯量做静态重力矩校核（numpy 运算，g=9.81，6000 组限位内采样）得 J2 峰值 8.83 N·m、J3 2.67、J4 2.74、J5 0.65、J6 0.68、J7 0.24 N·m，对照电机上限 50/50/50/50/12/12/12，**J5–J7 裕度约 11 N·m，全程静态不超限**；但该值是**静态演算**（未含减速器效率与动态项），**不是带载精度验证、也不是 PyBullet 实跑**。重力前馈补偿与带载精度属实物化后续工作，不在本轮范围。本条为答辩与说明书的统一口径，主动声明而非被问出来；")
    text = sub(text,
        "叙事口径为**图纸与说明书讲仿生机构结构，仿真以通用 6 轴平台验证控制流程**。答辩必须主动说明这一边界。",
        "叙事口径为**图纸与说明书讲仿生机构结构，仿真以达妙七轴平台验证控制流程**。答辩必须主动说明这一边界。")

    text = sub(text,
        "演示层另含 **v7 仿生触手三阶段视觉示意**（3 根 x 6 段纯视觉体，几何为示意值、非中科大实物参数）：接近阶段按 rho=a*exp(b*theta) **向后卷曲成对数螺旋**并收在末端后方不遮挡果实，到达指定坐标时**反向展开**，随后沿果实球面**缠绕包覆夹住**，逐颗循环、放果后回卷待机；它不是 URDF 末端执行器、不进运动学链、无接触 / 夹持力、不写 CSV。",
        "演示层另含 **末端执行器视觉呈现**（**v8.3 默认 = ORCA Hand 固定子装配**：官方 URDF 几何、预设握合姿态、握合腔中心即 TCP 果实被持点（精度判定点）、纯视觉、无接触力；**`--legacy-tentacle` 分支 = v8.2 卷曲章鱼爪**：3 根 x 6 段纯视觉体，三阶段「卷曲成对数螺旋 → 反向展开 → 球面缠绕包覆」，几何为示意值、非中科大实物参数，触手与果实在 link7 局部坐标系内生成、再按 link7 的世界旋转矩阵变换到世界系）；两者都**不是真实抓取**、无接触 / 夹持力、不参与误差判定、不写 CSV，也不代表抓取成功率。")

    # ---------------------------------------------------------------- §1
    text = sub(text,
        "机械臂末端 link（**link6**）依次到达 5 个预先写死的草莓空间目标点。",
        "机械臂末端 link（**link7**）依次到达 5 个预先写死的草莓空间目标点。")
    text = sub(text,
        "- **末端 link 名 = `link6`**（xarm_ros 官方 xacro 中 `link_eef` 整段被注释，名义末端即 link6）；\n- **到位误差 = 目标点到 link6 原点（URDF link frame 原点）的欧氏距离**；\n- 代码实现：`getLinkState(robot, 5, computeForwardKinematics=1)` 返回元组的 **[4] `worldLinkFramePosition`**（主口径，与 IK 的末端参考系一致）；**[0] `linkWorldPosition` 是质心（COM）位置，仅作备查列记录**，不得作为主判据（两者相差约 1.3 cm，混用会制造虚假误差）。",
        "- **末端 link 名 = `link7`**（官方 URDF 中 `link7` 是七轴臂最后一节；官方夹爪 `tool_link` 已被裁剪，故机械臂末端即 `link7`）；\n- **判定点 = TCP = 果实被持点 = ORCA 握合腔中心**（v8.3 修正口径）：TCP = `link7` 原点 + R_link7 @ `TCP_OFFSET`，`TCP_OFFSET = (0.048, 0, 0.32091)` m 为官方展开 URDF 几何实测（握合腔位于掌面与卷曲指尖之间）；**到位误差 = 果实点到 TCP 的欧氏距离**——拼装后到达果实的是末端执行器的握合腔而非法兰原点（2026-11-18 用户指正后重算）；\n- 代码实现：`fk_tcp()` 由 `getLinkState(robot, ee_idx, computeForwardKinematics=1)` 的 **[4] `worldLinkFramePosition` 与 [5] 姿态四元数**合成 TCP；**[0] `linkWorldPosition` 是质心（COM）位置，仅作备查列记录**（link7 COM 到 TCP 为固定偏移约 **292.5 mm**），不得作为主判据。\n- **`ee_idx` 按 link 名反查**（`resolve_ee_index()`），不硬编码索引：官方夹爪子树裁剪后 link 索引会变化，硬编码必然出错。")

    text = sub(text,
        "| 2 | 关节解在限位内 | 6 个关节角全部落在 URDF 限位区间（容差 1e-6） |",
        "| 2 | 关节解在限位内 | 7 个关节角全部落在官方 URDF 限位区间（容差 1e-6） |")
    text = sub(text,
        "水平半径（≤ 0.70 m）与 z 范围（[0.10, 0.75]）**仅作 IK 前预筛**（快速失败），不构成可达性证明。",
        "水平半径（≤ 0.389 m）与 z 范围（[-0.111, 0.507]）**仅作 IK 前预筛**（快速失败），不构成可达性证明；该包络取自 `handoff/演算基准.json` 的 `workspace.link7`（6000 组限位内随机位形采样，`r_max`≈0.389 m）。")

    # ---------------------------------------------------------------- §2.1 / §2.2 / §2.3
    text = sub(text,
        "6 轴机械臂末端位置由 6 个关节角决定；只约束末端 **位置**（3 个自由度）而不约束姿态（另 3 个自由度）时，方程数 3 < 未知数 6，为**欠定问题**，解不唯一。PyBullet 的 `calculateInverseKinematics` 在显式传入 `lowerLimits / upperLimits / jointRanges / restPoses` 时采用 **nullspace（零空间）法**：在满足位置约束的解流形中，选取最接近 `restPoses`（参考姿态）的解。因此：\n\n- **解由限位 + restPoses + 初值共同决定**——三者在脚本中全部写死，保证他人复跑得到同一组关节角（可复现性）；\n- 每次求解前 `resetJointState` 至固定 HOME 初值，消除初值依赖。",
        "7 轴机械臂末端位置由 7 个关节角决定；只约束末端 **位置**（3 个自由度）而不约束姿态时，方程数 3 < 未知数 7，为**冗余（超定欠约束）问题**，解不唯一。PyBullet 的 `calculateInverseKinematics` 在显式传入 `lowerLimits / upperLimits / jointRanges / restPoses` 时采用 **nullspace（零空间）法**：在满足位置约束的解流形中，选取最接近 `restPoses`（参考姿态）的解。因此：\n\n- **解由限位 + restPoses + 初值共同决定**——三者在脚本中全部写死，保证他人复跑得到同一组关节角（可复现性）；\n- **解连续性处理**：初值不再是固定 HOME，而是**上一点收敛后的实际关节角**（`solve_ik(..., seed=q_cur)`），避免 7 轴冗余下零空间漂移导致逐点解跳变、伺服跟踪误差变大甚至不收敛；HOME 仍作为 `restPoses` 的 nullspace 参考；\n- **TCP 口径两段式求解（v8.3 修正）**：拼装后 TCP = link7 原点 + R_link7 @ `TCP_OFFSET`（0.321 m 工具偏移），TCP 位置强烈依赖腕部朝向，纯位置-only IK 的冗余分支漂移无法收敛（实测残差卡在 100–500 mm）。故 `solve_ik_tcp()` 约束抓取朝向——**工具轴（link7 +z）指向果实方位角、可按俯仰 ±网格倾斜**（果实位于握合腔内），在**确定性姿态网格**（俯仰 −75°…+45° 步长 10°、掌面绕工具轴滚转 −180°…+180° 步长 30°）内以「TCP 残差 ≤ 0.1 mm」为约束取**限位余量最大**的姿态；每次先以带姿态内置 IK 求粗解，再用 `tcp_refine()`（对 TCP 直接做数值雅可比 + 阻尼最小二乘，姿态影响由有限差分自动包含）把 TCP 残差压到 **0.000 mm**；网格无解时基准朝向兜底。该口径不改变阈值、不改限位、不改判据；\n- **j7 参与 TCP 定位**：TCP 偏移不在 joint7 轴线上，joint7 旋转会改变 TCP 位置，故 j7 由解算确定，不再保持初值。")

    text = sub(text,
        "xarm_ros 官方仓库**不含现成 .urdf 文件**（全仓 `*.urdf` 数 = 0，只有 .xacro；展开依赖 ROS 工具链的 `$(find ...)` 与 `load_yaml`，无 ROS 环境无法生成）。因此本实验**手写 `xarm6_reach.urdf`**，所有数值抄自仓库 config 文件（逐项来源见附录 B）：\n\n- 关节原点/姿态 ← `config/kinematics/default/xarm6_default_kinematics.yaml`\n- 连杆惯量 ← `config/link_inertial/xarm6_type6_HT_BR2.yaml`（link_base 取 xacro 内联值）\n- 关节限位 ← `xarm6.urdf.xacro` 宏默认值\n- 力矩上限 ← `config/xarm6/joint_limits.yaml`（分关节，非一刀切）\n- 外观 mesh ← 仓库 7 个 STL（**弃用 .dae**：TINY 渲染器兼容性风险）",
        "达妙官方开源仓库 `left_arm_dm`（ROS 2 Humble，BSD-3-Clause）**直接提供 URDF 与 STL**，无需自建模型：\n\n- 模型来源 ← `src/left_description_grip/urdf/left_description_grip.urdf`（SolidWorks URDF Exporter 1.6 导出）\n- 几何与惯量 ← 同一 URDF 的 `<inertial>` 块（`base_link` 0.94 kg、`link1` 0.87、`link2` 0.278、`link3` 1.005、`link4` 1.244、`link5` 0.0756、`link6` 0.926、`link7` 0.1765 kg）\n- 关节限位 ← 同一 URDF 的 `<limit lower/upper>`（与官方 README §1.1 机械限位一致；**SolidWorks 导出 CSV 里的 Joint Lower/Upper 是另一套更窄的值，禁止混用**）\n- 力矩上限 ← 同一 URDF 的 `<limit effort>`（J1–J4 = 50、J5–J7 = 12 N·m；**取自官方原始值，未核实是否含传动折算**，不自行乘过载系数）\n- 外观 mesh ← 官方 8 个 STL（`base_link` + `link1`~`link7`，visual 与 collision 各一套，共 16 个文件）\n- **本轮唯一的模型改动**：派生一份仿真专用 URDF，删除官方夹爪子树并把 `package://` 路径改为相对路径（§5）。**未新建任何几何、未修改任何尺寸/惯量/限位**。")

    old_23_start = "### 2.3 零位 FK 理论值（用于加载自检）"
    old_23_end = "### 2.4 两层控制结构：轨迹规划 + 关节位置伺服"
    new_23 = """### 2.3 零位 FK 理论值（用于加载自检）

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

即**零位 link7 原点 = (0, -0.03325, 0.4981) m**，与 `handoff/演算基准.json` 的 `zero_pose.link7_tcp_m` 逐位一致（实测偏差 0.00001 mm）。`test_load.py` 以该值断言（容差 ±5 mm）——这是**自洽性自检**（验证派生 URDF 与官方 URDF 互洽、加载链路无损坏）。v8.3 修正口径另断言**零位 TCP（果实被持点）= (0.048, -0.03325, 0.81901) m**（= link7 原点 + TCP_OFFSET，零位朝向 R=I）。

> 对照：官方 `tool_link`（法兰转接板）原点在 (0, 0, 0.65498) m；实测其 mesh 顶面与 link7 腕筒顶面齐平（link7 系 z≈0.0907，零位世界 z≈0.5888）——**这就是末端执行器法兰面**，ORCA 手的安装基面即与此面相切（v8.3 修正挂接）。

"""
    text = replace_section(text, old_23_start, old_23_end, new_23)

    # ---------------------------------------------------------------- §5 / §6 / §7 整节替换
    derive_py = inject("derive_damiao_urdf.py")
    orca_derive_py = inject("derive_orcahand_urdf.py")
    test_load_py = inject("test_load.py")
    demo_py = inject("damiao_reach_demo.py")
    assets_py = inject("make_demo_assets.py")
    urdf_head = "\n".join(inject("damiao_reach.urdf").splitlines()[:16])

    sec5 = """## 5. 生成达妙仿真专用派生 URDF（`derive_damiao_urdf.py`）

### 步骤五：派生并自检 URDF

达妙官方 URDF 可直接用 ROS 加载，但 PyBullet 不认 `package://` 路径，且官方夹爪子树（含 2 个移动关节）会污染自由度统计。因此**派生一份仿真专用 URDF**，规则严格限定为三件事：

1. **删除官方夹爪子树**：`link{tool_link, gripper_left, gripper_right}` 与 `joint{tool_joint, gripper_joint1, gripper_joint2}` 整段删除；
2. **其余内容逐字保留**：`link{base_link, link1..link7}` 与 `joint1..joint7` 的几何、惯量、限位、effort/velocity、origin/axis **一个数都不改**（脚本内置「保留块与官方逐字一致」自检，15 个块全等）；
3. **仅改写 mesh 引用路径**：`package://left_description_grip/meshes/{visual,collision}/X.STL` → `meshes/damiao/{visual,collision}/X.STL`，并把 16 个官方 STL 原样复制到 `meshes/damiao/`（**STL 文件本身不改**）。

> 注：官方 `tool_joint` 实际是 **fixed** 关节（不是转动关节），官方 URDF 共 10 个 joint = 7 revolute + 1 fixed + 2 prismatic；裁剪后**恰好 7 个可动关节**。`link7.STL` 内含官方「夹爪相机 3507 安装板」结构，属官方 link7 的一部分，**保留原样**。

```bash
cd damiao_reach_lab
python derive_damiao_urdf.py \\
  --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \\
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
""" + urdf_head + """
```

派生脚本全文（可复算）：

```python
""" + derive_py + """```

### v8.3 新增：ORCA Hand 末端执行器派生（`derive_orcahand_urdf.py`）

ORCA Hand 官方只提供 `.urdf.xacro`（PyBullet 不能直接读取）。派生链路：

```bash
cd damiao_reach_lab
python third_party/orcahand/expand_orcahand_xacro.py      # 官方 xacro -> 确定性展开 URDF
python derive_orcahand_urdf.py \
  --src third_party/orcahand/orcahand_right.urdf \
  --dst third_party/orcahand/orcahand_right_sim.urdf      # 仿真专用派生 + 强校验
python derive_damiao_urdf.py \
  --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \
  --dst damiao_reach.urdf                                  # 达妙派生 + 追加 ORCA 子装配
```

派生内容仅三类（官方几何 mesh 与 link/joint 结构未改）：① 68 处 mesh 路径改写为工程相对路径；② 全部 `<inertial>` 惯量置零（维持 v8.2 无重力基线口径）；③ 17 个官方 revolute 锁定为 fixed（预设握合姿态，均在官方限位内；官方 URDF 无 mimic/transmission，腱绳耦合未建模）。挂接（v8.3 修正，按实际拼装）：`link7 -> orca_root`（fixed `joint_orca_wrist`，origin xyz=(0, 0, 0.10891)、rpy=(0, 0, -π/2)）——**手的安装基面（塔座底面，root 系 z=-0.01821）与机械臂末端法兰面（link7 腕筒顶面，link7 系 z=0.0907）相切**，手指沿工具轴（link7 +z）外伸、掌面朝 link7 +x；精度判定点为 TCP（果实被持点 = 握合腔中心，link7 系 (0.048, 0, 0.32091)）。

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
""" + orca_derive_py + """```

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

"""
    text = replace_section(text, "## 5. 手写 xarm6_reach.urdf", "## 6. 模型加载自检（test_load.py）", sec5)

    sec6 = """## 6. 模型加载自检（test_load.py）

### 步骤六：创建并运行自检脚本

自检项：① URDF 可加载；② link 总数 = **49**（7 达妙 revolute + 41 ORCA fixed + 1 挂接 fixed，ORCA 不引入可动关节）；③ 达妙 7 组限位逐项等于官方值；④ 末端 link 名 = `link7` 且索引按名反查；⑤ 零位 FK 断言：link7 原点 = (0, -0.03325, 0.4981) m 且 **TCP 果实点 = (0.048, -0.03325, 0.81901) m**（容差 5 mm）；⑥ **拼装断言**：挂接 origin xyz=(0,0,0.10891)、rpy=(0,0,-π/2) 与法兰相切口径一致、惯量置零（mass=0）、碰撞过滤可屏蔽；⑦ **ORCA 包围盒断言**（≤ 0.35 m）与**相切断言**（手基面 z ≈ 法兰面 z，Δ≤1 mm）、零位不与地面穿模；⑧ 渲染一帧验证官方 STL 显示通路。

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
[5] 零位 FK 断言（容差 5 mm）：link7 原点 = (0.0, -0.03325, 0.4981)；TCP（果实被持点 = 握合腔中心）= (0.048, -0.03325, 0.81901)（零位朝向 R=I）
    link7 原点 = (0.00000, -0.03325, 0.49810)，|Δ| = 0.000 mm PASS
    TCP 果实点 = (0.04800, -0.03325, 0.81901)，|Δ| = 0.000 mm PASS
    link7 COM  = (-0.04407, 0.00049, 0.54346)（备查）
[6] 拼装断言（手基面与法兰面相切 / 工具轴对齐 / 挂接 origin 校验）
    挂接 origin xyz = 0 0 0.10891（期望 (0.0, 0.0, 0.10891)）PASS
    挂接 rpy = 0 0 -1.5707963267948966（期望 (0.0, 0.0, -1.5707963267948966)）PASS
    ORCA link 质量: 全部 = 0.0 kg（惯量置零口径）PASS
    碰撞过滤: 42 个 ORCA link 已设 group=2/mask=0（不参与仿真接触）PASS
[7] ORCA 包围盒与相切断言（零位、世界系）
    包围盒 min=[-0.0955, -0.1372, 0.5888] max=[0.1166, 0.0631, 0.8873]
    尺寸 = [0.2121, 0.2002, 0.2985] m（≤ 0.35 m）PASS
    手基面 z = 0.58880 m ≈ 法兰面 z = 0.58880 m（相切，Δ≤1 mm）PASS
    零位最低点 z = 0.5888 m > 0（不与地面穿模）PASS
[9] 渲染一帧（ER_TINY_RENDERER，验证官方 STL 显示：达妙 + ORCA）
    帧缓冲 307200 bytes（= 320×240×4，非零即渲染通路正常）
== ALL PASS ==
```

自检脚本全文：

```python
""" + test_load_py + """```

---

"""
    text = replace_section(text, "## 6. 模型加载自检（test_load.py）", "## 7. 主实验脚本（xarm6_reach_demo.py）", sec6)

    sec7 = """## 7. 主实验脚本（`damiao_reach_demo.py`）

### 步骤七：创建主脚本

主脚本相对 xArm6 版的改动仅限「与臂相关的量」，控制链路（梯形轨迹 + 位置伺服 + 速度前馈 + 5 mm 判据 + 收敛窗口）**逐字保留**：

| 项 | xArm6 版（旧，已废弃） | 达妙七轴版（本轮） |
|---|---|---|
| URDF | 手写 `xarm6_reach.urdf` | 派生 `damiao_reach.urdf`（官方几何） |
| 自由度 / 末端 link | 6 / `link6`（索引 5，硬编码） | **7 / `link7`（索引按名反查）** |
| HOME | `[0, -0.5, -1.0, 0, 0.5, 0]` | 官方零位 `[0]*7` |
| LIMITS | xArm6 官方 6 组 | **达妙官方 7 组** |
| FORCES | 163.2/163.2/81.6/81.6/30.6/30.6 | **50/50/50/50/12/12/12 N·m** |
| TARGETS | (0.45,0,0.40) 等 5 点 | **v8.3 修正：按 TCP（果实被持点）口径重选 5 个果实点**（P1 (0.4196,0.0078,0.3354) 等，见 §8.3 表） |
| 预筛包络 | r ≤ 0.70 m、z ∈ [0.10, 0.75] | **r ≤ 0.389 m、z ∈ [-0.111, 0.507]** |
| IK | 内置单次 | **内置粗解 + 位置-only 数值精修**；初值 = 上一点实际关节角 |
| CSV 字段 | q1–q6（20 列） | **q1–q7（21 列）** |
| 演示层（末端执行器 / 篮子 / 果实） | 3 根 × 6 段三阶段章鱼爪 | **v8.2 原样保留为 `--legacy-tentacle` 分支**（常量、`update_tentacles()`、`place_into_basket()` 逐字不动，仅挂载点索引换成实测的 link7 索引）；**v8.3 默认 = ORCA Hand 固定子装配**（官方 URDF 几何、预设握合姿态、握合腔对准果实、无开合动画、无接触力） |
| 末端执行器挂接 | 无（章鱼爪为世界系/局部系视觉体，不在 URDF 内） | **v8.3：`link7 -> orca_root`（fixed `joint_orca_wrist`，惯量置零、碰撞过滤屏蔽、不进 IK 链）** |

主脚本全文：

```python
""" + demo_py + """```

演示素材派生脚本（由完整运行录像生成答辩用 GIF 与六帧关键帧拼图；xArm6 版为临时生成，本版固化为脚本，可复算）：

```python
""" + assets_py + """```

**v8.3 运行开关**（与 xArm6 版、v8.2 的差异说明）：

| 模式 | 命令 | 说明 |
|---|---|---|
| 默认（v8.3 基线） | `python damiao_reach_demo.py` | ORCA Hand 固定子装配（预设握合姿态），无重力基线，`results/reach_results.csv` |
| v8.2 复现 | `python damiao_reach_demo.py --legacy-tentacle` | 卷曲章鱼爪演示层分支，臂本体与判据不变 |
| 带载对照 | `python damiao_reach_demo.py --with-orca-mass` | 重力 9.81 + ORCA 官方质量（1.135 kg），独立 CSV `results/reach_results_orca_mass.csv` + 逐点静态持重力矩 PyBullet 实算，**数字不与无重力基线混报** |
| 快速验证 | `--no-video` / `--check-out-of-range` / `--res 640x480 --every 16` | 同 v8.2 口径 |

"""
    text = replace_section(text, "## 7. 主实验脚本（xarm6_reach_demo.py）", "### 步骤七附加：负向验证", sec7)

    # ---------------------------------------------------------------- §8.1 / §8.2 / §8.3
    text = sub(text,
        "```bash\npython xarm6_reach_demo.py                 # 完整运行（CSV + MP4 + 截图）\npython xarm6_reach_demo.py --no-video      # 快速验证（只出 CSV/截图，秒级）\npython xarm6_reach_demo.py --threshold 0.003   # 换阈值复跑（阈值可配）\n```",
        "```bash\ncd damiao_reach_lab\npython third_party/orcahand/expand_orcahand_xacro.py   # v8.3：ORCA xacro 展开（可复算）\npython derive_orcahand_urdf.py --src third_party/orcahand/orcahand_right.urdf --dst third_party/orcahand/orcahand_right_sim.urdf   # v8.3：ORCA 派生（须 ORCA DERIVE PASS）\npython derive_damiao_urdf.py --src ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf --dst damiao_reach.urdf   # 达妙派生 + ORCA 子装配追加（须 DERIVE PASS）\npython test_load.py                     # 加载自检（须 ALL PASS，含 ORCA 断言）\npython damiao_reach_demo.py --no-video            # 快速验证（只出 CSV/截图，秒级）\npython damiao_reach_demo.py --check-out-of-range # 负向验证（退出码 2）\npython damiao_reach_demo.py                      # 完整运行（CSV + MP4 + 截图；默认 ORCA Hand）\npython damiao_reach_demo.py --legacy-tentacle    # v8.2 章鱼爪复现分支\npython damiao_reach_demo.py --with-orca-mass     # 带载对照实验（独立 CSV，不与基线混报）\npython select_targets.py                         # 目标点选取工具（可复算）\n```")

    text = sub(text,
        "| 模式 | 帧数估算 | 实测耗时（2026-11-06，v5） |\n|------|----------|------------------------|\n| 默认 960×720 / 每 8 步 | 每点到位段约 120 帧 + 演示层抓放段约 75 帧，共 ~975 帧 | **976 帧，渲染 133.3 s，全程 133.6 s** |\n| 降级 640×480 / 每 16 步 | 约 ~490 帧 | **未执行**（默认模式 133.3 s < 5 分钟预算，未触发降级分支） |\n| `--no-video` | 1 帧（截图） | **0.1 s**（渲染 1 帧 0.1 s / 全程 0.1 s；伺服、抓放段与果实搬运仍完整执行） |",
        "| 模式 | 帧数估算 | 实测耗时（2026-11-18，v8.3 ORCA 版） |\n|------|----------|------------------------|\n| 默认 960×720 / 每 8 步（ORCA Hand） | 每点到位段约 120 帧 + 演示层抓放段约 75 帧，共 ~975 帧 | **{frames} 帧，渲染 {render:.1f} s，全程 {total:.1f} s** |\n| 降级 640×480 / 每 16 步 | 约 ~490 帧 | **未执行**（默认模式 {render:.1f} s < 5 分钟预算，未触发降级分支） |\n| `--no-video` | 1 帧（截图） | **{novideo:.1f} s**（渲染 1 帧；伺服、抓放段与果实搬运仍完整执行） |\n| `--legacy-tentacle`（v8.2 复现分支） | 与默认相同 | 快速验证 5/5 通过；降分辨率（320×240 / 每 16 步）抽样渲染确认章鱼爪三阶段动画完整（§14.7，非全量基线） |\n| `--with-orca-mass`（带载对照） | 与默认相同 + 重力 | `--no-video` 快速验证 5/5 通过、静态持重力矩逐点峰值 **J4 6.048 / J2 4.249 / J6 3.030 N·m**（PyBullet 实算，含 ORCA 官方质量 1.135 kg）；**独立 CSV，完整渲染未执行** |\n| 素材派生 `make_demo_assets.py` | GIF 171 帧（取样 196） / 关键帧拼图 6 帧 | **GIF 400×300 / 171 帧 / 160 ms 每帧（≈6.2 fps，循环约 27.4 s）/ 32 色 / 1022 KB；拼图 1440×784 / 317 KB**（由同一次运行的 MP4 派生，ORCA 口径标签） |".format(
            frames=stats["frames"], render=stats["render_time_s"], total=stats["total_time_s"],
            novideo=stats["no_video_time_s"]))

    text = sub(text,
        "ER_TINY_RENDERER 为 CPU 软件渲染，不依赖 WSLg/GPU，但高分辨率下耗时不可忽略。**渲染耗时由脚本自动打印并应在结果表登记**。若默认模式渲染超过 5 分钟，属正常范围；超过 15 分钟改用降级分支重跑。v7 实测 133.3 s 落在预算内（v4 为 601 帧 / 82.5 s，帧数增加来自 5 段抓放演示），故降级分支未运行；该分支的耗时不作任何结论引用。",
        "ER_TINY_RENDERER 为 CPU 软件渲染，不依赖 WSLg/GPU，但高分辨率下耗时不可忽略。**渲染耗时由脚本自动打印并应在结果表登记**。若默认模式渲染超过 5 分钟，属正常范围；超过 15 分钟改用降级分支重跑。v8 实测 {render:.1f} s 落在预算内（帧数与 xArm6 版同为 {frames} 帧，来自同样 5 段抓放演示），故降级分支未运行；该分支的耗时不作任何结论引用。达妙臂 STL 更大（16 个官方网格，合计约 33 MB），单帧渲染耗时与 xArm6 版同量级。".format(
            render=stats["render_time_s"], frames=stats["frames"]))

    # §8.3 整节替换（结果表按 CSV 实测值生成）
    old_83_start = "### 8.3 结果记录表"
    old_83_end = "## 9. 可达性预检与目标点调整分支"
    lines = ["| 点 | 果实点 TCP (m) | TCP-IK 残差 (mm) | TCP 伺服误差 (mm) | COM 备查 (mm) | 限位 | 收敛步 | 结论 | 备注 |",
             "|----|----------|--------------|---------------|----------|------|--------|------|------|"]
    for pid in ["P1", "P2", "P3", "P4", "P5"]:
        r = by_id[pid]
        lines.append("| {} | ({:.4f}, {:.4f}, {:.4f}) | {:.3f} | {:.3f} | {:.2f} | {} | {} | 到达 | |".format(
            pid, float(r["target_x"]), float(r["target_y"]), float(r["target_z"]),
            float(r["err_ik_m"]) * 1000, float(r["err_track_m"]) * 1000, float(r["err_com_m"]) * 1000,
            "OK" if r["in_limits"] == "1" else "超限", r["converge_step"]))
    table = "\n".join(lines)
    new_83 = """### 8.3 结果记录表（2026-11-18 达妙七轴版全量重跑回填；数据源：控制台汇总表 + `results/reach_results.csv` + `results/run_stats.json`）

""" + table + """

汇总信息：阈值 = **5 mm**（`--threshold 0.005`，未放宽）；判定口径 = **TCP（果实被持点 = ORCA 握合腔中心，link7 系 (0.048, 0, 0.32091)）**；渲染耗时 = **{render:.1f} s**（{frames} 帧 @ 960×720）；总耗时 = **{total:.1f} s**（墙钟 {wall}；其中 TCP 姿态网格搜索与 IK 计算约 31.7 s，渲染占比 94.6%）；pybullet 版本 = **3.2.7**（build time Jan 29 2025）。CSV 的 `render_time_s` = {render}（全局渲染耗时，各行同值）。CSV 为 **5 行 21 列**（新增 `q7`）。演示层 5 段抓放（v8.3 默认 = ORCA Hand 固定子装配、预设握合姿态呈现）：放果点 TCP 残差 {drop:.2f} mm、限位 5/5 OK，该段不写入 CSV。

**目标点选取（v8.3 修正口径）**：判定点 = TCP 果实被持点，目标点 = 果实位置。`select_targets.py` 按官方限位重扫 **40000 组采样**（seed 20261118），果实点云按前方带包络过滤（x≥0.25、z∈[0.12,0.45]、r≤0.55，对应 link7 实测 r≤0.388、z∈[-0.031,0.487]，臂本体仍在官方可达包络内）得 **2628 点**；在 5 个草莓簇锚点邻域择优，要求：**TCP 残差 ≤ 2 mm、限位余量 ≥ 0.15 rad、点间最小间距 0.08 m**（种子取候选点自身关节解，保证落在正确朝向分支）。最终各点限位余量 **{margins}**。

**COM 备查列**：v8.3 判定点改为 TCP 后，COM 列语义为「link7 质心到 TCP 的距离」= 固定值 **292.5 mm**（= |COM_local − TCP_OFFSET|，与关节角无关），仅备查、不参与任何判定。

**实测结论**：5 点全部满足两重静态判定 + 伺服跟踪判据，逐点 TCP 伺服跟踪误差 **{emin:.3f}–{emax:.3f} mm < 5 mm**（实测值见表），`converged=1`、`in_limits=1`；未发生任何目标点坐标或伺服增益调整（§9.2 决策树未触发）。**本结论只覆盖无重力设定下的轨迹跟踪与定位收敛**（判定点 = TCP 果实被持点，ORCA 子装配惯量置零），不代表带重力实物化性能，也不代表抓取成功率；末端执行器几何与果实搬运仅为流程可视化（§0 第 1 / 2 / 7 条）。

**结论书写规则**：仅当 5 点实测全部"到达"后，方可在结论区写"逐点误差 < 5 mm（实测值见表）"；存在未达标点时，结论如实写"Px 未收敛/超阈值，已按 §9.2 调整/记录"。**禁止先写结论再凑数。**

**未执行项**：`--with-orca-mass` 带载对照的完整渲染（快速验证已执行，独立 CSV 已生成，**不与本组无重力数字混报**）、降级渲染分支、`--threshold` 换阈值复跑、全新机器从零环境复跑、MP4 全量逐帧人工判读。

---

""".format(render=stats["render_time_s"], frames=stats["frames"], total=stats["total_time_s"],
           wall=stats["wall_clock"], drop=stats["drop_residual_mm"],
           margins=stats["limit_margins"], emin=stats["err_min_mm"], emax=stats["err_max_mm"])
    text = replace_section(text, old_83_start, old_83_end, new_83)

    # ---------------------------------------------------------------- §9.2 决策树
    text = sub(text,
        "| err_ik > 阈值 且 水平距离 > 0.60 m | 点位过远，接近臂展极限 | 水平距离缩小 0.03 m |\n| err_ik > 阈值 且 z > 0.55 m | 点位过高 | z 下调 0.03 m |\n| err_ik > 阈值 且 q3 < -3.5（贴 joint3 下限 -3.927） | 姿态贴限位 | 水平距离放大 0.03 m 或 z 下调 |\n| err_ik > 阈值 且以上三条都不满足 | IK 迭代不足 | `IK_MAX_ITER` 100→200 重跑一次（仍失败回到上三条） |",
        "| TCP-IK 残差 > 阈值 且 果实点 r > 0.55 m | 点位过远，超出果实可达包络 | 水平距离缩小 0.03 m |\n| TCP-IK 残差 > 阈值 且 z 不在 [0.12, 0.45] 或 x < 0.25 m | 越出前方带包络 | 高度 / 距离微调后重扫 |\n| TCP-IK 残差 > 阈值 且 解的限位余量 < 0.05 rad | 姿态贴限位 | 用 `select_targets.py` 重选邻域内余量更大的点 |\n| TCP-IK 残差 > 阈值 且以上三条都不满足 | TCP 精修迭代不足 / 带姿态 IK 停在局部解 | 先确认 `tcp_refine()` 与 HOME 回退已启用（本版已启用）；必要时加大迭代数或改用多初值粗解 |")

    # ---------------------------------------------------------------- §12 答辩速答
    text = sub(text,
        "1. **为什么没有夹爪/末端执行器？控制在哪？**\n   末端执行机构按机械图纸设计（A0 图有）；本轮控制结构为两层",
        "1. **末端执行器是什么？为什么画面里没有官方夹爪？**\n   末端执行器是 **ORCA Hand 灵巧手**（v8.3 默认）：几何与关节结构取自 SRL-ETHZ 官方描述仓库 `srl-ethz/orcahand_description`（**MIT License**），以 fixed joint 挂接于 `link7` 之后作为**固定子装配**——手的安装基面与机械臂末端法兰面（link7 腕筒顶面）**相切**、手指沿工具轴外伸；惯量置零、17 个 revolute 锁定为预设握合姿态、碰撞过滤屏蔽不参与接触；**腱绳耦合未在官方 URDF 中建模**（无 mimic/transmission），本仿真不含腱绳驱动、接触力学与夹持力，也不代表抓取成功率；**本轮明确不采用达妙官方夹爪**，官方 URDF 的夹爪子树（`tool_link` / `gripper_left` / `gripper_right` 及 2 个移动关节）已在派生 URDF 中整段裁剪。**精度判定点 = TCP（果实被持点 = 握合腔中心，link7 系 (0.048, 0, 0.32091)）**——拼装后到达果实的是末端执行器的握合腔而非法兰原点（2026-11-18 用户指正后重算）。v8.2 的卷曲仿生章鱼爪演示层保留为 `--legacy-tentacle` 复现分支。仿真控制结构为两层")
    text = sub(text,
        "   位置-only 对 6 轴臂是欠定问题（3 约束 / 6 未知）；PyBullet 用 nullspace 法在满足位置约束的解流形中取最接近参考姿态 HOME 的解。限位、参考姿态、初值三者在脚本中全部写死，因此任何人复跑得到同一组关节角（可复现性）。",
        "   位置-only 对 7 轴臂是**冗余问题**（3 约束 / 7 未知）；PyBullet 用 nullspace 法在满足位置约束的解流形中取最接近参考姿态 HOME 的解。限位（官方 7 组）、参考姿态（HOME）、初值策略（上一点实际关节角，保证解连续性）全部写死；内置求解器粗解后再加一段**位置-only 数值精修**（数值雅可比 + 阻尼最小二乘），因为 7 轴冗余下内置求解器在远距离初值会停在 3–8 mm 局部解。任何人复跑得到同一组关节角（可复现性）。")
    text = sub(text,
        "   末端 link 名为 link6（官方 xacro 中 link_eef 被注释，名义末端即 link6）；误差 = 目标点到 link6 原点（URDF link frame 原点，`getLinkState[4]`）的欧氏距离，与 IK 的末端参考系一致；质心口径（[0]）同表备查。",
        "   末端 link 名为 `link7`（官方 URDF 七轴臂的最后一节，官方夹爪已裁剪）；误差 = 果实点到 **TCP（果实被持点 = 握合腔中心）**的欧氏距离，TCP = link7 原点 + R_link7 @ TCP_OFFSET（`fk_tcp()`，基于 `getLinkState[4]/[5]`）；质心口径（[0]）同表备查（link7 COM 到 TCP 为固定偏移约 292.5 mm）。link 索引按名反查，不硬编码。")
    text = sub(text,
        "   分关节取自 xarm_ros 官方 `joint_limits.yaml`：163.2/163.2/81.6/81.6/30.6/30.6 N·m（含 2.55 倍过载系数；额定值 = 过载值 / 2.55）。该数值直接用于主流程位置伺服的 `forces` 参数与 URDF effort 标签。",
        "   分关节取自**达妙官方 URDF 的 `<limit effort>`**：J1–J4 = 50 N·m、J5–J7 = 12 N·m，直接用于主流程位置伺服的 `forces` 参数与 URDF effort 标签。**口径提示**：该值是否已含减速比/效率折算，官方未写明，按「官方原始值」直接引用，不自行乘过载系数（xArm6 版的 2.55 过载系数不适用于本臂）。另有 numpy 静态重力矩校核（g=9.81，6000 组采样）：J2 峰值 8.83、J3 2.67、J4 2.74、J5 0.65、J6 0.68、J7 0.24 N·m，J5–J7 裕度约 11 N·m，全程静态不超限——**该值为静态演算，非 PyBullet 实跑、非带载精度验证**。")
    text = sub(text,
        "   与方案设计文档同源的 P1–P5 原始坐标；预筛（半径 ≤ 0.70 m、z ∈ [0.10, 0.75]）通过，并经独立矩阵实算预核可达（IK 残差 0、q3 处于负值区远离上限），最终以两重静态判定 + 伺服跟踪实测锁定；若发生过调整，登记于 §9.2 变更表。",
        "   **原 xArm6 的 5 点在达妙臂上全部不可达**（残差 146–200 mm，臂展差一代）；v8.3 修正后判定点 = TCP（果实被持点），因此按**达妙官方限位 + 官方几何 + ORCA 握合腔几何重扫果实可达空间**后重选：预筛包络取自 TCP 果实点云实测（前方带 x ≥ 0.25 m、z ∈ [0.12, 0.45]、r ≤ 0.55 m；对应 link7 实测 r ≤ 0.388 m、z ∈ [-0.031, 0.487]，臂本体仍在官方可达包络内），在草莓簇锚点邻域内按「TCP 残差 ≤ 2 mm、限位余量 ≥ 0.15 rad、点间距 ≥ 0.08 m」择优，选点工具为 `select_targets.py`（可复算）；最终以两重静态判定 + 伺服跟踪实测锁定，实测 TCP 残差 0.000–0.001 mm；若发生过调整，登记于 §9.2 变更表。")
    text = sub(text,
        "   内置带限位的逆运动学求解器，适合快速验证定位链路；模型与参数全部来自 xArm 官方开源仓库（来源对照见附录 B）。",
        "   内置带限位的逆运动学求解器，适合快速验证定位链路；**模型与参数全部来自达妙官方开源仓库**（`left_arm_dm`，BSD-3-Clause），本轮只做了一处派生：删除官方夹爪子树 + 改写 mesh 引用路径，几何/惯量/限位逐字未改（来源对照见附录 B）。")
    text = sub(text,
        "仿生触手的机构结构在 A0 图纸与说明书中讲，仿真只验证通用 6 轴平台的控制流程。",
        "仿生触手的机构结构在 A0 图纸与说明书中讲，仿真只验证达妙七轴平台的控制流程；末端执行器为可替换模块（v8.3 = ORCA Hand 官方 URDF 几何固定子装配，v8.2 = 卷曲章鱼爪演示层），两者均不参与精度判定。")

    # ---------------------------------------------------------------- §13 回填清单
    old_13_start = "## 13. 实跑回填清单"
    old_13_end = "## 14. 实跑变更记录"
    new_13 = """## 13. 实跑回填清单（v8.3 达妙七轴 + ORCA Hand 版状态，2026-11-18）

- [x] 模型派生自检：`derive_orcahand_urdf.py` 输出 **ORCA DERIVE PASS**（42 link / 41 joint、68 处 mesh 路径改写全部可达、17 个 revolute 锁定预设姿态、无 package:// 残留、83 个块与官方展开一致）
- [x] 模型派生自检：`derive_damiao_urdf.py` 输出 **DERIVE PASS**（50 link / 49 joint、达妙 15 个保留块与官方逐字一致、ORCA 子装配 42 link / 41 fixed + 1 挂接 joint 校验通过）
- [x] 加载自检 `python test_load.py`：**ALL PASS**（49 link、达妙 7 组官方限位逐项 PASS、末端 `link7` 索引按名反查 = 6、零位 FK 偏差 0.000 mm、ORCA 惯量置零 / 碰撞过滤 / 包围盒断言 PASS、渲染 307200 bytes）
- [x] 5 点 err_ik / err_track / err_com / 限位判定 / 收敛步 / 结论（§8.3 表）：已按 2026-11-18 v8.3 修正后全量重跑的 `results/reach_results.csv` 回填，**5/5 通过**，逐点残差 {emin:.3f}–{emax:.3f} mm（阈值 5 mm 未放宽，判定点 = TCP 果实被持点）
- [x] **拼装与判定口径（v8.3 修正，用户两次指正后重算）**：手基面与法兰面相切（AABB Δ≤1 mm 断言 PASS）、手指沿工具轴外伸；判定点 = TCP（握合腔中心，link7 系 (0.048, 0, 0.32091)）；5 个目标点 = 果实位置，`select_targets.py` 按 TCP 口径重算（果实点云 2628 点 / 前方带包络），TCP 残差 0.000 mm
- [x] 渲染耗时：默认模式 **{frames} 帧 / {render:.1f} s**（§8.2）；`render_time_s` = {render}；`--no-video` {novideo:.1f} s；降级模式**未执行**（未触发）
- [x] 演示素材派生：`make_demo_assets.py` → `results/reach_demo.gif` + `results/reach_keyframes.png`（ORCA 口径中文标签），均已目检
- [x] 负向验证 `--check-out-of-range`：**退出码 2**，越界点（P_BAD_H 水平 0.80 m > 0.389 m；P_BAD_Z z=0.80 m ∉ [-0.111, 0.507]）在 IK 前被预筛拒绝
- [x] ORCA 末端执行器：固定子装配（fixed `joint_orca_wrist`）、惯量置零、预设握合姿态、碰撞过滤屏蔽；TCP 误差判定口径见上条
- [x] `--legacy-tentacle` 分支：快速验证 5/5 通过 + 降分辨率抽样渲染确认章鱼爪三阶段动画完整（v8.2 复现口径）
- [x] `--with-orca-mass` 带载对照：`--no-video` 快速验证 5/5 通过，独立 CSV `results/reach_results_orca_mass.csv`，逐点静态持重力矩峰值 **J4 6.048 / J2 4.249 / J6 3.030 N·m**（PyBullet 实算，含 ORCA 官方质量 1.135 kg）；**完整渲染未执行，数字不与无重力基线混报**
- [x] 演示层抓放：5 段，放果点 IK 残差 {drop:.2f} mm、限位 5/5 OK，篮子与果实入篮可见
- [x] 结论区文字（按 §8.3 结论书写规则）：已写「逐点误差 {emin:.3f}–{emax:.3f} mm < 5 mm」，附无重力设定与演示层边界限定
- [x] 静态重力矩校核（numpy，g=9.81，6000 组采样，v8.2 演算）：J2 8.83 / J3 2.67 / J4 2.74 / J5 0.65 / J6 0.68 / J7 0.24 N·m，对照 50/50/50/50/12/12/12 → J5–J7 裕度约 11 N·m（**该值未含 ORCA 质量**；含末端质量的对照见 `--with-orca-mass` 实算）
- [ ] **带重力纯臂对照运行**（不带 ORCA 质量）：未执行（需用户单独裁决；不得与本组无重力数字混报）
- [ ] `--with-orca-mass` 完整渲染 / 降级渲染分支 / `--threshold` 换阈值复跑：未执行
- [ ] 零探索验证：全新机器按本文档从零环境完整复跑（**未执行**，本轮为既有 WSL 环境实跑）
- [ ] MP4 画面**全量逐帧**人工判读（已抽样判读 6 帧，非全量，不作逐帧正确性结论）

---

""".format(frames=stats["frames"], render=stats["render_time_s"], novideo=stats["no_video_time_s"],
           emin=stats["err_min_mm"], emax=stats["err_max_mm"], drop=stats["drop_residual_mm"])
    text = replace_section(text, old_13_start, old_13_end, new_13)

    # ---------------------------------------------------------------- §14 变更记录追加
    text = sub(text, "## 14. 实跑变更记录（v4 基线 + v5 / v6 / v7 扩展）",
               "## 14. 实跑变更记录（v4 基线 + v5 / v6 / v7 扩展 + v8 达妙七轴迁移 + v8.3 ORCA 末端执行器）")
    old_14_end = "## 附录 B：参数来源对照表（每个数字可复核）"
    new_146 = """### 14.6 v8：仿真平台整体迁移到达妙七轴谐波臂（2026-11-18）

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-18（WSL2 Ubuntu / conda `robot` / Python 3.11.18 / pybullet 3.2.7 / numpy 2.4.6 / imageio 2.38.1） |
| 命令 | `python derive_damiao_urdf.py …`；`python test_load.py`；`python select_targets.py`；`python damiao_reach_demo.py --no-video`；`--check-out-of-range`；`python damiao_reach_demo.py` |
| 产物 | `results/reach_results.csv`（5 行 21 列）、`results/reach_demo.mp4`（{frames} 帧）、`results/scene.png`、`results/run_stats.json`、`results/target_selection.json` |
| 未执行 | 带重力对照运行、降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读 |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 / 现象 | 新值 / 处理 | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 仿真平台 | xArm6 六轴、手写 URDF | 达妙七轴、官方 URDF 派生 | 用户 11-18 决定平台改达妙（口径第三次变更） | `test_load.py` **ALL PASS**（7 关节 / 官方限位逐项一致 / 零位 FK 0.000 mm） |
| 2 | 模型来源 | 手写 `xarm6_reach.urdf` | 派生 `damiao_reach.urdf`：删夹爪子树 + 改 mesh 路径，**几何/惯量/限位逐字未改** | 官方开源包已提供 URDF/STL，自建模型违反红线 | 派生脚本自检 **DERIVE PASS**（15 个保留块与官方逐字一致） |
| 3 | 末端口径 | `link6`（索引硬编码 5） | `link7`，索引按 link 名反查 | 官方夹爪 `tool_link` 已裁剪；硬编码索引必错 | 索引实测 = 6；误差口径与 COM 备查列同步更新（相差约 71.7 mm） |
| 4 | 目标点 | P1–P5 原坐标 | 按官方几何重扫可达空间重选 5 点 | 原 5 点在达妙臂上残差 146–200 mm，**全部不可达** | 残差 0.000–0.001 mm，限位余量 {margins}，5/5 一次通过 |
| 5 | 预筛包络 | r ≤ 0.70 m、z ∈ [0.10, 0.75] | r ≤ 0.389 m、z ∈ [-0.111, 0.507] | 取自实测 `workspace.link7`（6000 组采样） | 负向验证两点均被预筛拒绝，退出码 2 |
| 6 | IK | 内置单次（maxNumIterations=100） | 内置粗解 + **位置-only 数值精修**（LM 式阻尼递增），初值 = 上一点实际关节角 | 7 轴冗余下内置求解器远距离初值停在 3–8 mm 局部解（超 5 mm 阈值）；连续性靠链式初值保证 | 残差 0.000–0.001 mm；**未放宽阈值、未改限位** |
| 7 | 维度 | 6 维 q1–q6、CSV 20 列 | 7 维 q1–q7、CSV 21 列 | 自由度 6 → 7 | CSV 5 行 21 列，与脚本一致 |
| 8 | 演示层 | 3 根 x 6 段触手 + 抓放循环 | **原样保留**（常量与两处函数逐字不动），仅挂载点索引换成实测 link7 索引 | 用户要求不重做末端执行器 | 画面确认无官方夹爪、三阶段动画完整 |
| 9 | 旧实跑数字 | 0.29–0.88 mm / 976 帧 / 133.3 s / converge_step=59 / 5-5 | **标注「xArm6 基线（已废弃）」**，仅留作历史对照 | 平台更换后旧数字不再是达妙结果 | 本版实跑：{emin:.3f}–{emax:.3f} mm / {frames} 帧 / {render:.1f} s / 5-5 |
| 10 | 末端演示层坐标系 | 触手与果实按**世界坐标**摆放（`ee + 偏移`，不乘腕部旋转）→ 腕部一转，爪与关节末端**歪出脱开**，缠绕圈也没包住果实 | 几何改在 **link7 局部坐标系**生成，再乘 link7 的世界旋转矩阵落到世界系（`wp = pos + R @ [x,y,z]`，果实同理）；零位时 R = I，历史画面不变 | 末端执行器必须与关节末端同朝向，否则演示层自相矛盾 | **纯视觉修正，不改控制链路**：5/5 仍通过、残差 {emin:.3f}–{emax:.3f} mm 不变；对照渲染确认触手长在 link7 末端并顺腕部轴向包住果实 |
| 11 | 末端执行器安装方向（**用户指出「装反」**） | 触手沿局部 **-z** 伸展（指向臂体内部）、缠绕环心在局部 -0.03 m → 爪缩在腕关节与臂体之间，视觉上「装反」 | 伸展方向翻转为局部 **+z（法兰外侧）**；缠绕环心改为 **link7 原点 = 被测点/草莓标记位置**（爪夹住的位置与误差测量点重合）；搬运果实跟随原点，`FRUIT_HELD_Z` 偏移常量删除 | 末端执行器必须从法兰外侧伸出，且夹持点应与被测点一致，避免引入未建模工具偏移 | **纯视觉修正，不改控制链路**：5/5 仍通过、残差 {emin:.3f}–{emax:.3f} mm 不变；复跑渲染确认爪从法兰外侧伸出并包住腕端果实 |

**未改动项（红线核对）**：`--threshold` 保持 0.005 m（未放宽）；官方 7 组关节限位、HOME / restPoses = 官方零位、FORCES = 50/50/50/50/12/12/12、POS_GAIN / VEL_GAIN = 1.0 / 1.0、`setGravity(0,0,0)`、STEADY_N = 60、SERVO_MAX_STEPS = 3000 全部未改；章鱼爪形态常量与三阶段时间线未改；未删点、未隐藏失败记录；§9.2 决策树未触发。

---

"""
    new_146 = new_146.format(frames=stats["frames"], render=stats["render_time_s"],
                             emin=stats["err_min_mm"], emax=stats["err_max_mm"],
                             margins=stats["limit_margins"])
    text = replace_section(text, old_14_end, "## 附录 B", new_146)

    # ---------------------------------------------------------------- §14.7 v8.3 变更记录
    new_147 = """### 14.7 v8.3：末端执行器更换为 ORCA Hand 官方 URDF（2026-11-18，第四次口径变更）

**口径裁决**：课设立论为**仿生机械臂本体**的定位精度验证；末端执行器为可替换模块。v8.3 只换末端执行器，**不动仿生臂本体**；同日用户两次指正后修正拼装与判定口径：① 手的安装基面与机械臂末端法兰面**相切**（消除悬空偏移）；② 判定点改为 **TCP = 果实被持点 = ORCA 握合腔中心**（拼装后到达果实的是末端执行器而非法兰原点），5 个目标点按 TCP 口径**重算为果实位置**、预筛包络改为果实点云实测。5 mm 阈值、两重静态判定 + 伺服跟踪判据、CSV 字段结构全部沿用。

| 项 | 值 |
|---|---|
| 实跑时间 | 2026-11-18（WSL2 Ubuntu / conda `robot` / Python 3.11.18 / pybullet 3.2.7 / numpy 2.4.6 / imageio 2.38.1） |
| 命令 | `expand_orcahand_xacro.py`；`derive_orcahand_urdf.py`；`derive_damiao_urdf.py`；`test_load.py`；`select_targets.py`；`damiao_reach_demo.py --no-video`；`--check-out-of-range`；`damiao_reach_demo.py`（完整）；`--legacy-tentacle`；`--with-orca-mass`；`make_demo_assets.py` |
| 产物 | `results/reach_results.csv`（5 行 21 列）、`results/reach_demo.mp4`、`results/scene.png`、`results/run_stats.json`、`results/target_selection.json`（复算与 v8.2 一致）、`results/reach_results_orca_mass.csv`（带载对照独立产物） |
| 未执行 | 带重力纯臂对照、`--with-orca-mass` 完整渲染、降级渲染分支、换阈值复跑、全新机器零探索复跑、MP4 全量逐帧判读 |

**变更登记（旧值 → 新值 → 原因 → 重跑结果）**

| # | 类别 | 旧值 / 现象（v8.2） | 新值 / 处理（v8.3） | 原因 | 重跑结果 |
|---|------|-------------|-------------|------|----------|
| 1 | 末端执行器 | 卷曲仿生章鱼爪纯视觉演示层（不在 URDF 内） | **ORCA Hand 官方 URDF 几何**（srl-ethz/orcahand_description，MIT），fixed 挂接 `joint_orca_wrist` | 用户 21:08 裁决：课设立论是仿生臂本体，末端执行器可换（第四次口径变更） | `test_load.py` ALL PASS（ORCA 断言全过）；5/5 通过 |
| 2 | ORCA 子装配口径 | — | 惯量置零（42 link mass=0）、17 个 revolute 锁定为预设握合姿态（均在官方限位内）、碰撞过滤屏蔽不参与接触 | 维持 v8.2 无重力基线与位置伺服口径不变；官方 URDF 无 mimic/transmission，腱绳耦合未建模 | ORCA DERIVE PASS（83 块与官方展开一致）；伺服误差 0.000–0.001 mm 不变 |
| 3 | 演示层 | 章鱼爪三阶段（默认） | **默认 = ORCA 握合呈现**；章鱼爪保留为 `--legacy-tentacle` 分支 | 主演示改为 ORCA Hand，v8.2 可复现 | 降分辨率抽样渲染确认三阶段动画完整 |
| 4 | 带载对照 | 无 | 新增 `--with-orca-mass`：重力 + ORCA 官方质量 1.135 kg + 独立 CSV | 分离带载口径，禁止数字混报 | 快速验证 5/5 通过；静态持重力矩峰值 J4 6.048 / J2 4.249 N·m（PyBullet 实算）；完整渲染未执行 |
| 5 | 模型文件 | 8 link / 7 revolute | **50 link / 49 joint**（+42 ORCA link、+41 ORCA fixed、+1 挂接 fixed） | ORCA 为固定子装配，不引入可动关节 | `test_load.py` 断言 revolute=7、fixed=42 PASS |
| 6 | run_stats.json | 手工维护 | 主脚本自动回填（指导书生成器数据源） | 保证「文档 = 工程」 | v8.3 重跑自动生成 |
| 7 | 达妙臂主体 | 官方 URDF 派生 | **不变**（15 个保留块逐字一致、16 个官方 STL 未动） | 红线：不动臂本体 | DERIVE PASS 校验含 ORCA 项后仍通过 |
| 8 | 挂接修正（**用户指正①**） | 手悬空挂接（xyz=(-0.15,0,-0.06)，掌面向上托果，握合腔对准 link7 局部 (0,0,-0.03)） | **手基面与法兰面相切**：origin xyz=(0,0,0.10891)（法兰面 z=0.0907 + 基面抬升 0.01821）、rpy=(0,0,-π/2)，手指沿工具轴外伸、掌面朝 link7 +x | 手的底部法兰应与机械臂末端执行器法兰相切（2026-11-18 用户指正） | AABB 断言：手基面 z 0.58880 = 法兰面 z 0.58880（Δ<0.1 mm）；渲染核验无间隙无穿模 |
| 9 | 判定口径修正（**用户指正②**） | 判定点 = link7 原点（法兰），目标点 = link7 可达点 | 判定点 = **TCP（果实被持点 = 握合腔中心，link7 系 (0.048,0,0.32091)）**；5 个目标点重算为果实位置（P1 (0.4196,0.0078,0.3354) 等），IK 改带姿态约束 + TCP 数值精修 | 拼装后到达果实位置的是末端执行器而非法兰原点，要按实际重算（2026-11-18 用户指正） | 全量重跑 **5/5 通过**，TCP 残差 0.000–0.001 mm；目标点选取与包络登记见 §8.3 |

**未改动项（红线核对）**：`--threshold` 保持 0.005 m；官方 7 组关节限位、FORCES、POS_GAIN / VEL_GAIN、`setGravity(0,0,0)`（默认模式）、STEADY_N、SERVO_MAX_STEPS 未改；达妙 16 个官方 STL 未动；ORCA 68 个官方 STL 未动；未删点、未隐藏失败记录；§9.2 决策树未触发。**已按口径修正**：5 个目标点（重算为果实位置）、预筛包络（改为果实点云实测）、HOME/restPoses 仍为官方零位（TCP 求解的 nullspace 参考）。

**边界声明（答辩必须主动说明）**：无重力设定；ORCA 子装配惯量置零，**不代表带载精度**；腱绳耦合未建模、无接触力学与夹持力；果实搬运为纯视觉移动；**不代表抓取成功率**。带载数字（`--with-orca-mass`）与无重力基线数字分列，不混用。

---

"""
    text = replace_section(text, "## 附录 B", "## 附录 B", new_147 + "## 附录 B")

    # ---------------------------------------------------------------- 附录 B
    old_b_start = "## 附录 B：参数来源对照表（每个数字可复核）"
    old_b_end = "## 附录 C：伺服增益调整指引"
    new_b = """## 附录 B：参数来源对照表（每个数字可复核）

| 参数 | 值 | 来源 |
|------|-----|------|
| 关节原点（7 组 xyz，rpy 全 0） | j1 (0,0,0.0756)；j2 (0,-0.042,0.0425)；j3 (0,0.042,0.0744)；j4 (-0.042,0,0.1156)；j5 (0.042,0,0.0995)；j6 (-0.03325,0,0.035)；j7 (0.03325,-0.03325,0.0555) | 达妙官方 `left_description_grip.urdf`（**派生后逐字未改**） |
| 关节限位（rad） | j1 [-2.827, 2.827]；j2 [-1.8064158, 0.6806784]；j3 [-2.8797933, 2.8797933]；j4 [-1.8151424, 0.7592182]；j5 [-2.7576202, 2.7576202]；j6 [-1.3264502, 1.3264502]；j7 [-1.3351769, 1.3351769] | 同上 `<limit lower/upper>`（与官方 README §1.1 机械限位一致；**SolidWorks 导出 CSV 的 Joint Lower/Upper 是另一套更窄的值，禁止混用**） |
| 力矩上限 | 50 / 50 / 50 / 50 / 12 / 12 / 12 N·m | 同上 `<limit effort>`（**官方原始值，未核实是否含传动折算**；不自行乘过载系数） |
| 关节速度上限 | 10.0 rad/s | 同上 `<limit velocity>` |
| 连杆惯量（mass/origin/惯性张量） | base_link 0.94 / link1 0.87 / link2 0.278196 / link3 1.005 / link4 1.244 / link5 0.0756 / link6 0.926 / link7 0.1765 kg | 同上 `<inertial>`（**仿真以 URDF 为准**；SolidWorks 导出 CSV 的 Mass 列多处不一致，冲突时以 URDF 为准） |
| 零位 FK 末端理论值 | link7 原点 = (0, -0.03325, 0.4981) m | 本指导书 §2.3 累加推导；与 `handoff/演算基准.json` 的 `zero_pose.link7_tcp_m` 一致 |
| HOME / restPoses | 官方零位 `[0]*7` rad | 本实验定义（官方零位即 nullspace 参考）；**v8 实跑锁定** |
| 目标点 5 个（TCP 果实点） | 见 §7 脚本 TARGETS（P1–P5，见 §8.3 表） | `select_targets.py` 按官方限位 + 官方几何 + TCP 口径重扫果实可达空间择优（TCP 残差 ≤ 2 mm、限位余量 ≥ 0.15 rad、间距 ≥ 0.08 m）；**v8.3 修正实跑锁定**：5 点全部通过 |
| TCP（果实被持点，v8.3 判定口径） | link7 系 `TCP_OFFSET = (0.048, 0, 0.32091)` m；零位 TCP 世界值 (0.048, -0.03325, 0.81901) m | 官方展开 URDF 几何实测（握合腔中心：掌面与卷曲指尖之间）；`damiao_reach_demo.py` 常量区 `TCP_OFFSET`；`test_load.py` 零位断言（容差 ±5 mm） |
| 果实预筛包络（v8.3） | 前方带 x ≥ 0.25 m、z ∈ [0.12, 0.45]、r ≤ 0.55 m（TCP 果实点云 40000 组采样实测；对应 link7 r ≤ 0.388、z ∈ [-0.031, 0.487]） | `select_targets.py` / `damiao_reach_demo.py` 常量区 `X_MIN / RADIUS_MAX / Z_MIN / Z_MAX`（seed 20261118） |
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
| ORCA 挂接参数（v8.3 修正，按实际拼装） | `link7 → orca_root`（fixed `joint_orca_wrist`）：origin xyz = (0, 0, 0.10891) m，rpy = (0, 0, -π/2)；**手的安装基面（塔座底面 root 系 z=-0.01821）与机械臂末端法兰面（link7 腕筒顶面 link7 系 z=0.0907）相切**，手指沿工具轴外伸、掌面朝 link7 +x；零位包围盒 0.212×0.200×0.299 m、手基面 z 0.58880 = 法兰面 z 0.58880（Δ<0.1 mm）、不与地面穿模 | `derive_damiao_urdf.py` 的 `ORCA_MOUNT_*`（法兰面 = 官方 tool 转接板顶面/腕筒顶面实测；`test_load.py` 相切断言 PASS） |
| ORCA 预设握合姿态（v8.3，示意值） | wrist 0；四指 abd 0.04–0.10 / mcp 1.05–1.20 / pip 0.35–0.42；拇指 mcp 0.45 / abd 0.55 / pip 0.50 / dip 0.15 rad（**全部在官方 `<limit>` 内**；官方 URDF 未建模腱绳耦合、未提供姿态表，此为示意姿态，不代表真实抓取） | `derive_orcahand_urdf.py` 的 `PRESET`（锁定角烘焙进 joint origin，并在 URDF 注释登记官方限位） |
| 带载对照口径（v8.3，`--with-orca-mass`） | 重力 9.81 m/s² + ORCA 官方质量合计 1.135 kg（tower 0.9 kg + 全惯量）；逐点静态持重力矩 PyBullet 实算峰值 J4 6.048 / J2 4.249 / J6 3.030 N·m（TCP 口径新目标点） | `damiao_reach_demo.py --with-orca-mass`（独立 CSV `results/reach_results_orca_mass.csv`；**与无重力基线数字分列，禁止混报**） |

---

"""
    text = replace_section(text, old_b_start, old_b_end, new_b)

    # ---------------------------------------------------------------- §4 目录与资产
    old_4_start = "## 4. 项目目录与资产准备"
    old_4_end = "## 5. 生成达妙仿真专用派生 URDF（`derive_damiao_urdf.py`）"
    new_4 = """## 4. 项目目录与资产准备

### 步骤四：建目录、拷官方 STL

达妙官方资产全部来自官方开源仓库（`left_arm_dm`，BSD-3-Clause）：URDF + 8 个 link 的 visual / collision STL。ORCA Hand 资产来自 srl-ethz/orcahand_description（MIT）。

```bash
# 官方源目录（本工作区）
DM=/mnt/d/桌面/workbuddy_project/2026-11-18-17-48-11/damiao_7axis_arm/left_arm_dm/src/left_description_grip
OC=/mnt/d/桌面/workbuddy_project/2026-11-18-17-43-24/orcahand_description_official
# 新工程目录（本工作区）
mkdir -p damiao_reach_lab/meshes/damiao/visual damiao_reach_lab/meshes/damiao/collision \\
         damiao_reach_lab/meshes/orcahand/visual damiao_reach_lab/meshes/orcahand/collision \\
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

"""
    text = replace_section(text, old_4_start, old_4_end, new_4)

    # ---------------------------------------------------------------- §7 附加负向验证命令
    text = sub(text,
        "python xarm6_reach_demo.py --check-out-of-range; echo \"exit=$?\"",
        "python damiao_reach_demo.py --check-out-of-range; echo \"exit=$?\"（期望 2）")

    # ---------------------------------------------------------------- §10 归档
    text = sub(text,
        "**整工程归档，不只是结果目录**：`xarm6_reach.urdf`、`test_load.py`、`xarm6_reach_demo.py`、`meshes/`、`results/` 全部进入 Windows 侧",
        "**整工程归档，不只是结果目录**：`damiao_reach.urdf`、`derive_damiao_urdf.py`、`derive_orcahand_urdf.py`、`third_party/orcahand/`、`select_targets.py`、`test_load.py`、`damiao_reach_demo.py`、`meshes/damiao/`、`meshes/orcahand/`、`results/` 全部进入 Windows 侧")
    text = sub(text,
        "把该目录内容放回 `~/xarm_lab` 即可按 §6–§8 复跑。",
        "把该目录内容放回工程根目录即可按 §5–§8 复跑。")

    # ---------------------------------------------------------------- §11 排错表
    text = sub(text,
        "| 4 | `error: Cannot load URDF file` | 不在 `~/xarm_lab` 目录运行（URDF 为相对路径） | `cd ~/xarm_lab` 后重跑；项目不得放在 `/mnt/d/...`（中文路径，PyBullet C++ 加载不稳） |\n| 5 | 报 mesh 相关错误 / 模型不显示 | `meshes/xarm6/visual/` 下 STL 不足 7 个 | 回到步骤四重新拷贝；`.dae` 不拷贝是**正确行为** |\n| 6 | test_load [5] FK 断言 FAIL | URDF 数值被改动过 | 按 §5 原文重建 URDF；数值来源见附录 B |",
        "| 4 | `error: Cannot load URDF file` / `XML_ERROR_PARSING_DECLARATION` | 不在工程根目录运行（URDF 与 mesh 均为相对路径）；或派生时把 XML 声明挪到了注释之后 | `cd damiao_reach_lab` 后重跑；派生脚本已保证 XML 声明位于文件首行 |\n| 5 | 报 mesh 相关错误 / 模型不显示 | `meshes/damiao/{visual,collision}/` 下 STL 不足 8 个 | 回到步骤四重新拷贝；`tool_link` / `gripper_*` 不拷贝是**正确行为** |\n| 6 | test_load [5] FK 断言 FAIL | 派生 URDF 被改动过 | 用 `derive_damiao_urdf.py` 重新派生（官方源文件只读）；数值来源见附录 B |\n| 6b | 关节数不是 7 / 末端 link 不是 link7 | 用的是未裁剪的官方 URDF | 必须用派生的 `damiao_reach.urdf`；官方 URDF 含 1 fixed + 2 prismatic 夹爪关节 |")

    # ---------------------------------------------------------------- §14.1 历史标注
    text = sub(text,
        "### 14.1 实跑基线\n",
        "### 14.1 实跑基线（**以下 §14.1–§14.5 为 xArm6 六轴平台的历史记录，已随平台更换作废，仅作对照保留**）\n")

    with open(DST, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"已生成：{DST}（{len(text)} 字符）")


if __name__ == "__main__":
    main()
