#!/usr/bin/env python3
"""
damiao_reach_demo.py - 达妙七轴谐波机械臂 草莓定点到达仿真
                      （位置-only IK + 梯形轨迹 + 位置伺服执行）。

v8.3（2026-11-18，ORCA Hand 末端执行器版；同日两次按用户指正修正）：
  * 课设立论 = 仿生机械臂本体；末端执行器为可替换模块。
  * 末端执行器换成 ORCA Hand 官方 URDF（srl-ethz/orcahand_description，MIT）：
    以 fixed joint 挂接在 link7 之后（joint_orca_wrist），作为固定子装配呈现——
    惯量置零、17 个 revolute 锁定为预设握合姿态、不参与仿真接触（碰撞过滤关闭）。
    腱绳耦合未在官方 URDF 中建模，本仿真不含腱绳驱动、接触力学与夹持力。
  * 按实际拼装重算（用户指正：① 手基面须与机械臂末端法兰相切；② 拼装后到达
    果实位置的是末端执行器而非法兰原点）：
      - 挂接：手安装基面（塔座底面）与 link7 腕筒法兰面（z=0.0907）相切，
        手指延伸方向对齐工具轴（link7 +z），掌面朝 link7 +x；
      - 判定点改为 **TCP = 果实被持点 = ORCA 握合腔中心**（link7 系 TCP_OFFSET）；
      - 目标点 = 果实位置，由 select_targets.py 按 TCP 口径重算（预筛包络
        同步改为果实点云实测：前方带 x≥0.25、z∈[0.12,0.45]、r≤0.55）；
      - IK 加 TCP 外环（solve_ik_tcp）：内环仍为两段式位置-only IK，外环按
        TCP 残差修正 link7 目标。
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
TARGETS = [                                    # v8.3 修正：TCP（果实被持点）口径，select_targets.py 重算
    ("P1", (0.4196, 0.0078, 0.3354)),          # TCP 残差 0.000 mm，限位余量 0.226 rad
    ("P2", (0.3614, 0.1476, 0.2506)),          # 0.000 mm / 0.181 rad
    ("P3", (0.3726, -0.1973, 0.2630)),         # 0.000 mm / 0.167 rad
    ("P4", (0.4794, 0.0654, 0.4014)),          # 0.000 mm / 0.522 rad
    ("P5", (0.4618, -0.0564, 0.1786)),         # 0.000 mm / 0.165 rad
]
# ---- v8.3 修正口径：精度判定点 = TCP（果实被持点 = ORCA 握合腔中心） ----
# 按实际拼装重算：手基面与 link7 法兰面相切、手指沿工具轴外伸后，握合腔中心位于
# link7 系 (0.048, 0, 0.32091)（官方展开 URDF 几何实测：掌面与卷曲指尖之间）。
# 判定/预筛/伺服误差全部以 TCP 为口径（用户 2026-11-18 指正后修正）。
TCP_OFFSET = (0.048, 0.0, 0.32091)             # TCP = link7 原点 + R_link7 @ TCP_OFFSET
TCP_TOL = 1e-6                                 # TCP 数值精修收敛容差（tcp_refine 内部）
# 果实点预筛包络（TCP 可达云实测，40000 组限位内采样，seed 20261118）：
# 基座前方带 x≥0.25、z∈[0.12,0.45]、r≤0.55；对应 link7 实测 r≤0.388、z∈[-0.031,0.487]，
# 即臂本体仍在官方可达包络（r≤0.389、z∈[-0.111,0.507]）内。
X_MIN, RADIUS_MAX, Z_MIN, Z_MAX = 0.25, 0.55, 0.12, 0.45
IK_MAX_ITER, IK_TOL = 200, 1e-5                # IK 迭代上限与残差阈值
SIM_HZ = 240                                    # 仿真步频（setTimeStep 真实生效）
ANIM_STEPS, HOLD_STEPS = 480, 480              # 轨迹段 2 s + 到位保持段 2 s（含收敛观察窗）
STEADY_N = 60                                   # 收敛判据：连续 60 步（0.25 s）达标
SERVO_MAX_STEPS = 3000                          # 每点伺服总步数上限（超时兜底 12.5 s，非收敛判据）
POS_GAIN, VEL_GAIN = 1.0, 1.0                  # 伺服增益（无重力设定下的验证基线，依据见附录 B/C）
BAD_POINT_H = (0.80, 0.00, 0.30)               # 负向验证：水平越界（0.80 m > 0.55 m）
BAD_POINT_Z = (0.40, 0.00, 0.80)               # 负向验证：z 越界（0.80 m > 0.45 m）

# ---------------- 演示层抓放循环（视觉示意，不参与误差判定；来源见指导书附录 B） ----------------
BASKET_CENTER = (0.16, -0.25, 0.08)             # 篮子中心（m），基座右前方近地面
BASKET_DROP = (0.30, -0.26, 0.24)              # 放果点：TCP（果实被持点）到达的篮口上方点（TCP-IK，不约束姿态）
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
# ---- 末端演示层避让参数（仅 --legacy-tentacle 分支使用） ----
# 干涉根因：缠绕环心原压在被测点（球心 = link7 原点），环半径 0.033 m 而 link7 网格
#（含相机安装板）分布于原点周围 → 环陷入 URDF 网格，看似「装反」。
# 修正：爪体（掌心）挂到网格之外，缠绕环心对准果实保持点（局部 -z 0.03 m）。
# v8.3 修正后精度判定点为 TCP（果实被持点），本分支的缠绕环心仅供 v8.2 演示复现。
FRUIT_HELD_Z = 0.03                             # 果实保持点：link7 局部 -z 偏移（m；仅 --legacy-tentacle 分支缠绕环心用）
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
    """预筛（TCP=果实被持点口径）：前方带 x≥X_MIN、水平半径 r≤RADIUS_MAX、高度窗。
    越界即退出码 2（在 IK 前快速失败）。"""
    rho = math.hypot(point[0], point[1])
    if rho > RADIUS_MAX:
        print(f"[预筛拒绝] {name} 水平距离 {rho:.3f} m > {RADIUS_MAX} m，IK 前中止。")
        print("            （--check-out-of-range 模式仅验证拒绝逻辑，不构成可达性结论）")
        raise SystemExit(2)
    if not (Z_MIN <= point[2] <= Z_MAX):
        print(f"[预筛拒绝] {name} z={point[2]:.3f} m 不在 [{Z_MIN},{Z_MAX}]，IK 前中止。")
        raise SystemExit(2)
    if point[0] < X_MIN:
        print(f"[预筛拒绝] {name} x={point[0]:.3f} m < 前方带 {X_MIN} m，IK 前中止。")
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
    """FK 复核：返回 (link7 原点, link7 COM)。备查口径（判定点已改为 TCP）。"""
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    return ls[4], ls[0]


def fk_tcp(robot, ee_idx):
    """TCP = 果实被持点（ORCA 握合腔中心）= link7 原点 + R_link7 @ TCP_OFFSET。
    返回 (tcp 位置, link7 旋转矩阵, link7 原点)。主误差口径。"""
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    R = np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)
    return np.array(ls[4]) + R @ np.array(TCP_OFFSET), R, np.array(ls[4])


def quat_from_matrix(R):
    """旋转矩阵 → 四元数 [x,y,z,w]（标准 Shepperd 法，数值稳定分支）。"""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    if m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    if m[1, 1] >= m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]


def tcp_refine(robot, ee_idx, fruit, q, iters=80, eps=1e-5, tol=1e-6):
    """TCP 数值精修：对 TCP（果实被持点）位置做数值雅可比 + 阻尼最小二乘。

    与 refine_ik() 同一套阻尼策略，区别仅在 FK 用 TCP（含姿态对 0.321 m 偏移的
    影响，由有限差分自动包含）；关节角始终夹在官方限位内。
    """
    q = list(q)
    set_joints(robot, q)
    err = math.dist(fk_tcp(robot, ee_idx)[0], fruit)
    for _ in range(iters):
        if err <= tol:
            break
        tcp0, _, _ = fk_tcp(robot, ee_idx)
        J = np.zeros((3, NDOF))
        for k in range(NDOF):
            qk = list(q)
            qk[k] = min(max(qk[k] + eps, LIMITS[k][0]), LIMITS[k][1])
            set_joints(robot, qk)
            J[:, k] = (fk_tcp(robot, ee_idx)[0] - tcp0) / eps
        set_joints(robot, q)
        improved = False
        for trial in range(8):
            lam = 1e-7 * (10 ** trial)
            try:
                dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), np.array(fruit) - tcp0)
            except np.linalg.LinAlgError:
                continue
            qn = [min(max(q[i] + dq[i], LIMITS[i][0]), LIMITS[i][1]) for i in range(NDOF)]
            set_joints(robot, qn)
            en = math.dist(fk_tcp(robot, ee_idx)[0], fruit)
            if en < err - 1e-12:
                q, err = qn, en
                improved = True
                break
        if not improved:
            break
    set_joints(robot, q)
    return q, err


def tcp_orientation_matrix(fruit, tilt_deg, roll_deg):
    """抓取朝向：工具轴（link7 +z）水平方位角指向果实、抬升 tilt_deg；
    掌面（link7 +x）以 -z 为基准绕工具轴滚转 roll_deg。返回 R_des。"""
    fruit = np.array(fruit, dtype=float)
    psi = math.atan2(fruit[1], fruit[0])
    uh = np.array([math.cos(psi), math.sin(psi), 0.0])
    th = math.radians(tilt_deg)
    u = uh * math.cos(th) + np.array([0.0, 0.0, 1.0]) * math.sin(th)
    base = np.array([0.0, 0.0, -1.0])
    base -= u * (base @ u)
    if np.linalg.norm(base) < 1e-6:
        base = np.array([1.0, 0.0, 0.0])
    base /= np.linalg.norm(base)
    v = np.cross(u, base)
    rd = math.radians(roll_deg)
    x_l = base * math.cos(rd) + v * math.sin(rd)
    z_l = u
    y_l = np.cross(z_l, x_l)
    return np.column_stack([x_l, y_l, z_l])


def solve_tcp_pose(robot, ee_idx, fruit, tilt_deg, roll_deg, seed=None):
    """固定朝向（tilt/roll）下的 TCP-IK：带姿态粗解 + TCP 数值精修。返回 (q, tcp 残差)。"""
    fruit = np.array(fruit, dtype=float)
    R_des = tcp_orientation_matrix(fruit, tilt_deg, roll_deg)
    quat = quat_from_matrix(R_des)
    pos7 = fruit - R_des @ np.array(TCP_OFFSET)
    seed_list = list(seed) if seed is not None else list(HOME)
    set_joints(robot, seed_list)
    q = list(p.calculateInverseKinematics(
        robot, ee_idx, list(pos7), quat,
        lowerLimits=[lo for lo, _ in LIMITS],
        upperLimits=[hi for _, hi in LIMITS],
        jointRanges=[hi - lo for lo, hi in LIMITS],
        restPoses=seed_list,
        maxNumIterations=IK_MAX_ITER,
        residualThreshold=IK_TOL,
    )[:NDOF])
    return tcp_refine(robot, ee_idx, fruit, q)


def solve_ik_tcp(robot, ee_idx, fruit, seed=None, search_orientation=False):
    """TCP 口径 IK（v8.3 修正，带姿态约束）。

    拼装后握合腔中心在工具轴上 0.321 m、掌面侧 0.048 m 处，TCP 位置强烈依赖
    link7 朝向；纯位置-only IK 无法收敛（冗余分支漂移，实测残差 100–500 mm）。
    故约束抓取朝向（工具轴指向果实）并用带姿态 IK 出粗解、tcp_refine() 对 TCP
    数值精修（姿态影响由差分自动包含）。

    search_orientation=True（主实验用）：在确定性网格
    tilt ∈ [-75°, +45°] 步长 10°、roll ∈ [-180°, +180°) 步长 30° 内，
    以「TCP 残差 ≤ 0.1 mm」为约束、取**限位余量最大**的姿态（避免腕部贴限位）；
    网格与次序写死，结果可复现。False（选点等批量场景）：仅用基准朝向 tilt=0/roll=0。
    返回 (q, tcp 残差)。"""
    if not search_orientation:
        q, err = solve_tcp_pose(robot, ee_idx, fruit, 0, 0, seed=seed)
        if err > 1e-3 and seed is not None:            # 确定性回退 HOME 重解
            q2, err2 = solve_tcp_pose(robot, ee_idx, fruit, 0, 0, seed=None)
            if err2 < err:
                q, err = q2, err2
        set_joints(robot, q)
        return q, err

    best = None                                          # (margin, tilt, roll, q, err)
    for tilt in range(-75, 46, 10):
        for roll in range(-180, 180, 30):
            q, err = solve_tcp_pose(robot, ee_idx, fruit, tilt, roll, seed=None)
            if err > 1e-3:
                continue
            margin = min(min(q[i] - LIMITS[i][0], LIMITS[i][1] - q[i]) for i in range(NDOF))
            if best is None or margin > best[0] + 1e-9:
                best = (margin, tilt, roll, q, err)
    if best is None:                                     # 网格内无可行解 → 基准朝向兜底
        q, err = solve_tcp_pose(robot, ee_idx, fruit, 0, 0, seed=seed)
        set_joints(robot, q)
        return q, err
    _, _, _, q, err = best
    set_joints(robot, q)
    return q, err


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
    print(f"[末端] ORCA Hand 固定子装配：基面与法兰相切、沿工具轴外伸；惯量置零合计 "
          f"{orca_mass_total:.3f} kg，碰撞过滤 group={ORCA_COLLISION_GROUP}/"
          f"mask={ORCA_COLLISION_MASK}（不参与接触），17 个 revolute 锁定为预设握合姿态；"
          f"判定点 = TCP（果实被持点 = 握合腔中心，link7 系 {TCP_OFFSET}）")
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
        # 果实被持点偏移：默认 = TCP_OFFSET（ORCA 握合腔中心）；--legacy-tentacle 分支保持
        # v8.2 口径（link7 局部 -z FRUIT_HELD_Z，缠绕环心）。
        hold_off = np.array([0.0, 0.0, -FRUIT_HELD_Z]) if args.legacy_tentacle \
            else np.array(TCP_OFFSET)
        q_drop, drop_err = solve_ik_tcp(robot, ee_idx, BASKET_DROP, seed=q_start,
                                        search_orientation=True)
        for j in range(NDOF):
            p.resetJointState(robot, j, q_drop[j])
        tcp_d, _, _ = fk_tcp(robot, ee_idx)
        drop_err = math.dist(tcp_d, BASKET_DROP)
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
            pos, R = ee_pose()        # 果实搬运至被持点（默认 = TCP 握合腔中心；legacy = 局部 -z）
            fp = pos + R @ hold_off
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
            pos, R = ee_pose()        # 果实搬运至被持点（默认 = TCP 握合腔中心；legacy = 局部 -z）
            fp = pos + R @ hold_off
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

    print(f"{'点':4s} {'果实点TCP(m)':22s} {'TCP-IK残差(mm)':>13s} {'TCP伺服误差(mm)':>14s} "
          f"{'COM(mm)':>9s} {'限位':4s} {'收敛步':>6s} {'结论':6s}")
    for name, target in TARGETS:
        precheck(target, name)

        # ---- 静态判定：TCP-IK 求解 + TCP 残差 + 限位 ----
        # TCP-IK：确定性姿态网格内取限位余量最大者（避免腕部贴限位）；q_cur 仅作兜底种子
        q, err_ik = solve_ik_tcp(robot, ee_idx, target, seed=q_cur, search_orientation=True)
        for j in range(NDOF):                         # 设置 IK 解测静态残差
            p.resetJointState(robot, j, q[j])
        tcp_static, _, pos7 = fk_tcp(robot, ee_idx)
        ee_frame, ee_com = fk_ee(robot, ee_idx)
        err_com = math.dist(ee_com, tcp_static)       # 备查口径：link7 质心到 TCP 的固定偏移
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
            tcp_now, _, _ = fk_tcp(robot, ee_idx)
            steady = steady + 1 if math.dist(tcp_now, target) <= args.threshold else 0
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
            tcp_now, _, _ = fk_tcp(robot, ee_idx)
            steady = steady + 1 if math.dist(tcp_now, target) <= args.threshold else 0
            if steady >= STEADY_N:
                converge_step = ANIM_STEPS + HOLD_STEPS + k
                break
        ee_track, _, _ = fk_tcp(robot, ee_idx)
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
        "platform": "达妙七轴谐波机械臂（官方 URDF 派生）+ ORCA Hand 末端执行器（v8.3，TCP=果实被持点口径）",
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
