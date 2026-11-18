#!/usr/bin/env python3
"""
damiao_reach_demo.py - 达妙七轴谐波机械臂 草莓定点到达仿真
                      （位置-only IK + 梯形轨迹 + 位置伺服执行）。

迁移说明（2026-11-18，xarm6_reach_demo.py → 本文件）：
  * 模型：达妙官方开源七轴 URDF 派生版 damiao_reach.urdf（仅裁剪官方夹爪子树，
    几何/惯量/限位逐字未改，见 derive_damiao_urdf.py 与附录 B）；
  * 自由度 6 → 7，末端 link link6 → link7（索引按 link 名反查，不硬编码）；
  * HOME / LIMITS / FORCES / TARGETS / 预筛包络 / CSV 字段全部换成达妙官方值；
  * IK 改为两段式：内置求解器出粗解（保留限位与 nullspace 参考）+ 位置-only 数值精修，
    原因见 refine_ik() 注释（7 轴冗余下内置求解器远距离初值会停在 3–8 mm 局部解）；
  * 末端执行器仍为仿真内已有的卷曲仿生章鱼爪演示层（纯视觉），原样保留，
    未引入达妙官方夹爪。

产物：results/reach_results.csv、results/reach_demo.mp4、results/scene.png、控制台汇总表
退出码：0 = 5 点全部通过（两重静态判定 + 伺服跟踪）；1 = 存在未达标点（结果如实保留）；
        2 = 负向验证模式（--check-out-of-range，仅验证预筛拒绝逻辑）
"""
import argparse
import csv
import math
import os
import time

import imageio
import numpy as np
import pybullet as p
import pybullet_data

NDOF = 7                                       # 达妙七轴
EE_NAME = "link7"                             # 末端 link 名（按名反查索引，禁止硬编码）
TOOL_OFFSET_Z = 0.11                           # 工具偏移：夹持中心 TCP = link7 原点 + R·[0,0,L]（m）
                                               # L=0.11 > link7 网格沿 +z 上限 0.0877（相机安装板），爪环不与 URDF 干涉

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
TARGETS = [                                    # 按 TCP 口径重扫可达空间后重选（results/tcp_workspace.json）
    ("P1", (0.2582, -0.0324, 0.3198)),         # IK 残差 0.000 mm，限位余量 0.165 rad
    ("P2", (0.2654, -0.1176, 0.2665)),         # 0.000 mm / 0.326 rad
    ("P3", (0.1912, -0.1395, 0.2209)),         # 0.000 mm / 0.153 rad
    ("P4", (0.2809, 0.0567, 0.3962)),          # 0.000 mm / 0.431 rad
    ("P5", (0.2771, -0.0611, 0.1826)),         # 0.000 mm / 0.157 rad
]
RADIUS_MAX, Z_MIN, Z_MAX = 0.49, -0.20, 0.60   # IK 前预筛：TCP 实测可达包络（含工具偏移 0.11 m）
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
# 缠绕态：沿果实球面（球心 = link7 原点 = 被测点）自球顶绕到球底
WRAP_R = 0.033                                     # 包覆球面半径 (m，略大于果实半径 0.025)
WRAP_PHI0, WRAP_PHI1 = 0.18 * math.pi, 0.96 * math.pi   # 球顶到球底极角范围 (示意)
WRAP_TWIST = 0.55 * math.pi                        # 缠绕时绕球轴的附加转角 (示意)
# 阶段时间线：到位段后段展开，保持段前段缠绕，入篮段后段释放回卷
EXTEND_START = 0.75                                # 到位轨迹段中开始反向展开的归一化时刻
WRAP_END = 0.30                                    # 保持段中完成缠绕包覆的归一化时刻
RELEASE_START = 0.70                               # 入篮保持段中开始释放回卷的归一化时刻


def resolve_ee_index(robot):
    """按末端 link 名反查真实 link 索引（禁止硬编码索引）。"""
    for i in range(p.getNumJoints(robot)):
        if p.getJointInfo(robot, i)[12].decode() == EE_NAME:
            return i
    raise RuntimeError(f"未找到末端 link：{EE_NAME}")


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


def frame_from_z(d):
    """单位向量 d → 旋转矩阵（局部 +z 对准 d，无额外翻滚）。"""
    z = np.asarray(d, dtype=float)
    z = z / np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def mat_quat(R):
    """旋转矩阵 → 四元数 (x,y,z,w)（Shepperd 法，本 pybullet 无 Matrix→Quaternion API）。"""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 2.0) * 2
        w, x = 0.25 * S, (m[2, 1] - m[1, 2]) / S
        y, z = (m[0, 2] - m[2, 0]) / S, (m[1, 0] - m[0, 1]) / S
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / S, 0.25 * S
        y, z = (m[0, 1] + m[1, 0]) / S, (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / S, (m[0, 1] + m[1, 0]) / S
        y, z = 0.25 * S, (m[1, 2] + m[2, 1]) / S
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / S, (m[0, 2] + m[2, 0]) / S
        y, z = (m[1, 2] + m[2, 1]) / S, 0.25 * S
    return [x, y, z, w]


def quat_R(quat):
    """四元数 (x,y,z,w) → 3×3 旋转矩阵。"""
    x, y, z, w = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def fk_pos(robot, ee_idx):
    """末端夹持中心（TCP）世界坐标 = link7 原点 + R·[0,0,TOOL_OFFSET_Z]。

    口径变更（2026-11-18，用户诊断）：夹持中心不能与被测点/草莓重合压在腕上——
    那样爪环必然陷入 link7 网格（相机安装板沿 +z 伸至 0.0877 m）。TCP 定义在
    法兰外 L=0.11 m 处：腕部停在果实体外，爪从法兰伸出正好包住草莓。
    """
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    return np.array(ls[4], dtype=float) + quat_R(ls[5]) @ np.array([0.0, 0.0, TOOL_OFFSET_Z])


def solve_ik(robot, ee_idx, target, seed=None, face=None):
    """两段式位置-only IK（TCP 口径 + 可选朝向 face）。

    第一段：PyBullet 内置求解器出粗解——显式限位 / jointRanges / HOME 作 nullspace 参考；
            初值取上一点收敛后的实际关节角（解连续性处理，避免 7 轴冗余的零空间漂移）；
            face 给定时改用 6D IK（targetOrientation = 爪轴对准 face 方向），
            腕部目标 = 目标 − R_face·[0,0,TOOL_OFFSET_Z]（腕部停在果实体外）。
    第二段：refine_ik() 以 TCP 为测量点做位置-only 数值精修，把残差压到亚毫米。

    注：末端参考点取 link7 原点（即 joint7 轴心），joint7 旋转不改变该点位置，
    因此位置-only IK 无法确定 j7，它保持初值（= 0）；不影响位置误差判定。
    """
    seed = list(seed) if seed is not None else list(HOME)
    set_joints(robot, seed)
    tgt = np.asarray(target, dtype=float)
    kw = {}
    if face is not None:
        R_f = frame_from_z(face)
        kw["targetOrientation"] = mat_quat(R_f)
        tgt = tgt - R_f @ np.array([0.0, 0.0, TOOL_OFFSET_Z])
    q = list(p.calculateInverseKinematics(
        robot, ee_idx, [float(v) for v in tgt],
        **kw,
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

    p.connect(p.DIRECT)
    p.setTimeStep(1.0 / SIM_HZ)                       # 240 Hz 仿真步长
    p.setGravity(0, 0, 0)                             # 无重力设定（§0 第 6 条）：重力前馈补偿属后续工作
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")                          # 地面
    # 栽培架示意（绿色薄板，仅视觉）
    board = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.28, 0.30, 0.005],
                                rgbaColor=[0.25, 0.55, 0.25, 1.0])
    p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=board,
                      basePosition=[0.16, -0.06, 0.0005])
    robot = p.loadURDF("damiao_reach.urdf", useFixedBase=True)
    ee_idx = resolve_ee_index(robot)                  # 末端 link 索引（按名反查）
    print(f"[模型] 达妙七轴派生 URDF 加载成功，末端 link = {EE_NAME}，link 索引 = {ee_idx}，"
          f"可动关节数 = {p.getNumJoints(robot)}")

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
        几何以**夹持中心 TCP**（link7 局部 [0,0,TOOL_OFFSET_Z]=0.11 m，在 link7
        网格之外）为缠绕环心：腕部停在果实体外，触手从法兰伸出越过腕部包住草莓——
        环不再陷入 link7 网格（2026-11-18 用户诊断的干涉根因）。
        纯视觉，不参与误差判定。
        """
        def smooth(x):
            x = min(1.0, max(0.0, x))
            return x * x * (3.0 - 2.0 * x)

        pos, R = ee_pose()
        ca = [math.cos(2.0 * math.pi * i / TENTACLE_COUNT) for i in range(TENTACLE_COUNT)]
        sa = [math.sin(2.0 * math.pi * i / TENTACLE_COUNT) for i in range(TENTACLE_COUNT)]
        a_ext = smooth(2.0 * t)                     # 卷曲 -> 展开
        a_wrap = smooth(2.0 * t - 1.0)              # 展开 -> 缠绕
        for j in range(TENTACLE_SEGS):
            u = (j + 1) / TENTACLE_SEGS
            # (1) 卷曲态：螺旋袖套卷在腕部外围（避开 link7 网格径向），不遮挡前方果实
            theta = SPIRAL_THETA_MAX * u
            rho = 0.055 + 0.02 * u
            z_coil = 0.03 - 0.09 * u
            # (2) 展开态：沿法兰外侧 +z 向夹持中心（TCP，z=0.11）伸展并越过果实
            r_ext = 0.015 + 0.02 * u
            z_ext = 0.02 + 0.14 * u
            # (3) 缠绕态：环绕果实体——球心在夹持中心 TCP（局部 [0,0,TOOL_OFFSET_Z]）
            phi = WRAP_PHI0 + (WRAP_PHI1 - WRAP_PHI0) * u
            rr_wrap = WRAP_R * math.sin(phi)
            z_wrap = TOOL_OFFSET_Z + WRAP_R * math.cos(phi)
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
                wp = pos + R @ np.array([x, y, z])
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
        writer = imageio.get_writer("results/reach_demo.mp4", fps=30,
                                    codec="libx264", quality=8, macro_block_size=1)

    def shoot(force=False):
        """渲染一帧并写盘。计时覆盖 getCameraImage（CPU 软渲染大头）+ 写盘。
        --no-video（writer=None）时跳过常规帧，仅 force=True（终态截图）渲染一帧。"""
        nonlocal n_frames, t_render
        if writer is None and not force:
            return None
        update_tentacles(tentacle_t)                  # 渲染前更新触手三阶段形态
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

    # ---- 演示层相位引擎：观察→确定位置→夹取→合拢→抓住→移动→松开→掉落→回预设 ----
    FACE_DIR = (0.42, 0.20, -0.885)                # 末端朝向（确定性）：爪轴指向前下方
    PRESET_TCP = (0.20, -0.05, 0.42)               # 预设/待机位（TCP 坐标）
    OBS_BACK = 0.09                                # 观察位：TCP 沿爪轴后撤距离（m）
    DROP_ABOVE = 0.12                              # 松开点：篮内槽位正上方高度（m）
    G_ANIM = 9.81                                  # 掉落动画重力加速度（m/s²）

    def servo_to(q_to, dur_s, t_start, t_end, carry_ball=None):
        """关节伺服沿梯形轨迹到 q_to；触手形态 t_start→t_end；carry_ball 跟随 TCP。"""
        nonlocal tentacle_t
        steps = max(1, int(round(dur_s * SIM_HZ)))
        prof = trap_profile(steps)
        dt = 1.0 / SIM_HZ
        q_prev = [p.getJointState(robot, j)[0] for j in range(NDOF)]
        for k, sk in enumerate(prof):
            q_cmd = [q_prev[j] + sk * (q_to[j] - q_prev[j]) for j in range(NDOF)]
            v_cmd = [0.0] * NDOF if k == 0 else [(q_cmd[j] - q_prev[j]) / dt for j in range(NDOF)]
            p.setJointMotorControlArray(robot, range(NDOF), p.POSITION_CONTROL,
                                        targetPositions=q_cmd, targetVelocities=v_cmd,
                                        forces=FORCES, positionGains=[POS_GAIN] * NDOF,
                                        velocityGains=[VEL_GAIN] * NDOF)
            p.stepSimulation()
            tentacle_t = t_start + (t_end - t_start) * (k / max(1, steps - 1))
            if carry_ball is not None:
                tcp = fk_pos(robot, ee_idx)
                p.resetBasePositionAndOrientation(carry_ball,
                                                  [float(tcp[0]), float(tcp[1]), float(tcp[2])],
                                                  [0.0, 0.0, 0.0, 1.0])
            if writer is not None and k % args.every == 0:
                shoot()
        tentacle_t = t_end

    def drop_fruit(ball, p0, p1):
        """松开后果实自由坠落至篮内槽位（脚本化抛体，g=9.81，纯视觉）。"""
        dz = max(1e-4, p0[2] - p1[2])
        t_fall = math.sqrt(2.0 * dz / G_ANIM)
        steps = max(2, int(round(t_fall * SIM_HZ)))
        vx = (p1[0] - p0[0]) / t_fall
        vy = (p1[1] - p0[1]) / t_fall
        for k in range(steps):
            tt = (k + 1) / steps * t_fall
            p.resetBasePositionAndOrientation(ball,
                                              [p0[0] + vx * tt, p0[1] + vy * tt,
                                               max(p1[2], p0[2] - 0.5 * G_ANIM * tt * tt)],
                                              [0.0, 0.0, 0.0, 1.0])
            if writer is not None and k % 2 == 0:
                shoot()
        p.resetBasePositionAndOrientation(ball, [float(p1[0]), float(p1[1]), float(p1[2])],
                                          [0.0, 0.0, 0.0, 1.0])

    rows, q_cur, place_checks = [], list(HOME), []
    for i in range(NDOF):                             # 伺服起始 = HOME
        p.resetJointState(robot, i, HOME[i])
    t_total0 = time.perf_counter()

    print(f"{'点':4s} {'目标(m)':22s} {'IK残差(mm)':>11s} {'伺服误差(mm)':>12s} "
          f"{'COM(mm)':>9s} {'限位':4s} {'收敛步':>6s} {'结论':6s}")
    for name, target in TARGETS:
        precheck(target, name)

        # ---- 采摘流程：观察→确定位置→夹取→合拢→抓住→移动→松开→掉落→回预设 ----
        slot = FRUIT_SLOTS[len(place_checks)]
        p_drop = (slot[0], slot[1], slot[2] + DROP_ABOVE)
        d = np.asarray(FACE_DIR, dtype=float)
        d = d / np.linalg.norm(d)
        p_obs = (target[0] - d[0] * OBS_BACK, target[1] - d[1] * OBS_BACK,
                 target[2] - d[2] * OBS_BACK)

        # ① 观察：转向果实观察位（爪轴朝向果实方向）
        q_obs = solve_ik(robot, ee_idx, p_obs, seed=q_cur, face=d)
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.95, 0.80, 0.10, 1.0])
        servo_to(q_obs, 0.9, 0.0, 0.0)
        # ② 确定位置：目标球脉冲 3 次（视觉确认）
        for c in range(3):
            p.changeVisualShape(ball_ids[name], -1,
                                rgbaColor=[0.85, 0.10, 0.10, 1.0] if c % 2 else [0.95, 0.80, 0.10, 1.0])
            for k in range(24):
                p.stepSimulation()
                if writer is not None and k % args.every == 0:
                    shoot()

        # ③ 夹取：TCP 从观察位沿爪轴推进到果实
        q = solve_ik(robot, ee_idx, target, seed=q_obs, face=d)
        servo_to(q, 1.2, 0.0, 0.5)

        # 静态判定（TCP 口径）
        ee_com = fk_ee(robot, ee_idx)[1]              # COM 备查列（相对 TCP，仅记录不判定）
        err_com = math.dist(ee_com, target)           # COM 备查距离
        err_ik = math.dist(fk_pos(robot, ee_idx), target)   # TCP 口径
        in_lim, bad = check_limits(q)

        # ④ 合拢：触手沿球面收拢（0.5→1.0），同步测定 TCP 收敛
        steady, converge_step = 0, None
        steps_close = int(round(0.6 * SIM_HZ))
        for k in range(steps_close):
            tentacle_t = 0.5 + 0.5 * (k / max(1, steps_close - 1))
            p.stepSimulation()
            ee = fk_pos(robot, ee_idx)
            steady = steady + 1 if math.dist(ee, target) <= args.threshold else 0
            if steady >= STEADY_N and converge_step is None:
                converge_step = k
            if writer is not None and k % args.every == 0:
                shoot()

        # ⑤ 抓住：保持段（0.3 s），err_track 在此测定
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.95, 0.80, 0.10, 1.0])
        for k in range(int(round(0.3 * SIM_HZ))):
            p.stepSimulation()
            ee = fk_pos(robot, ee_idx)
            steady = steady + 1 if math.dist(ee, target) <= args.threshold else 0
            if steady >= STEADY_N and converge_step is None:
                converge_step = steps_close + k
            if writer is not None and k % args.every == 0:
                shoot()
        err_track = math.dist(fk_pos(robot, ee_idx), target)
        q_cur = [p.getJointState(robot, j)[0] for j in range(NDOF)]

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

        # ⑥ 移动：果实跟随 TCP 运往篮子松开点（槽位正上方）
        q_drop = solve_ik(robot, ee_idx, p_drop, seed=q_cur, face=d)
        servo_to(q_drop, 1.5, 1.0, 1.0, carry_ball=ball_ids[name])
        # ⑦ 松开：果实固定在松开点，爪张开（1.0→0 回预设形态）
        release_err = math.dist(fk_pos(robot, ee_idx), p_drop)
        servo_to(q_drop, 0.5, 1.0, 0.0)
        # ⑧ 果实掉落：从松开点自由坠落至槽位
        drop_fruit(ball_ids[name], p_drop, slot)
        p.changeVisualShape(ball_ids[name], -1, rgbaColor=[0.10, 0.70, 0.15, 1.0])  # 已采摘→绿色
        # ⑨ 回预设姿势：爪已回卷，末端返回预设位（面向下一个果实的待机朝向）
        q_preset = solve_ik(robot, ee_idx, PRESET_TCP, seed=q_cur, face=d)
        servo_to(q_preset, 1.0, 0.0, 0.0)
        place_checks.append((name, release_err * 1000, True))
        q_cur = [p.getJointState(robot, j)[0] for j in range(NDOF)]

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
    with open("results/reach_results.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=fields)
        wr.writeheader()
        wr.writerows(rows)

    n_ok = sum(r["reached"] for r in rows)
    print("-" * 78)
    print(f"结果：{n_ok}/{len(rows)} 点通过（两重静态判定 + 伺服跟踪；阈值 {args.threshold * 1000:.0f} mm，"
          f"可经 --threshold 配置）")
    print(f"渲染：{n_frames} 帧 @ {w}x{h}，渲染耗时 {t_render:.1f} s；全程耗时 {t_total:.1f} s")
    if place_checks:
        print(f"演示层：{len(place_checks)} 段完整采摘流程（观察→确定位置→夹取→合拢→抓住→移动→"
              f"松开→果实掉落→回预设），松开点 TCP 残差 "
              f"{min(e for _, e, _ in place_checks):.2f}–{max(e for _, e, _ in place_checks):.2f} mm；"
              f"该段为视觉示意，不参与误差判定与 CSV")
    print(f"演示层仿生触手：{TENTACLE_COUNT} 根 x {TENTACLE_SEGS} 段三阶段视觉示意，"
          f"卷曲(rho=a*exp(b*theta)) -> 到位反向展开 -> 球面缠绕包覆果实，"
          f"逐颗循环、放果后回卷；几何为示意参数（非中科大实物数据），"
          f"无接触 / 夹持力，不参与误差判定与 CSV")
    print("（末端执行器为仿真内已有的卷曲仿生章鱼爪纯视觉演示层；"
          "仿真不采用达妙官方夹爪，官方 URDF 的夹爪子树已在派生 URDF 中整段裁剪）")
    if args.no_video:
        print("产物：results/reach_results.csv / results/scene.png（--no-video：未生成 MP4）")
    else:
        print("产物：results/reach_results.csv / results/reach_demo.mp4 / results/scene.png")
    print("（未达标点已如实标注 note，处理流程见指导书 §9.2，禁止放宽阈值或改限位）")
    p.disconnect()
    raise SystemExit(0 if n_ok == len(rows) else 1)


if __name__ == "__main__":
    main()
