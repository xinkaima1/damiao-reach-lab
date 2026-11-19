#!/usr/bin/env python3
"""select_targets.py - 达妙七轴臂 + ORCA Hand 拼装体可达空间扫描与 5 个果实点选取（可复现）。

v8.3 修正口径（2026-11-18 用户指正后重算）：
  * ORCA Hand 基面与 link7 法兰面相切、手指沿工具轴外伸后，精度判定点不再是
    link7 原点，而是 **TCP = 果实被持点 = ORCA 握合腔中心**
    （link7 系 TCP_OFFSET，几何实测：掌面与卷曲指尖之间）；
  * 目标点 = 果实位置：TCP 依赖 link7 朝向（R_link7 @ TCP_OFFSET），
    故用 TCP 外环 IK（见 damiao_reach_demo.solve_ik_tcp）求解；
  * 预筛包络改为 TCP 果实点云实测：基座前方带 x≥0.25、z∈[0.12,0.45]、r≤0.55；
    对应 link7 实测 r≤0.388、z∈[-0.031,0.487]，臂本体仍在官方可达包络内。

方法：
  1) 关节空间随机采样（限位内）→ 正运动学 → TCP 果实点云；
  2) 按前方带包络过滤；
  3) 对预设的草莓簇锚点各取云中最近点作初值，跑 TCP 外环 IK 精修（残差 ≤ 2 mm 才收）；
  4) 打印最终 5 点坐标与 TCP 残差，供写入 damiao_reach_demo.py 的 TARGETS。

用法：python select_targets.py [--map]
"""
import argparse
import json
import math
import random

import numpy as np
import pybullet as p

import damiao_reach_demo as D          # 复用主脚本的两段式 IK + TCP 外环（与实跑同口径）

URDF = "damiao_reach.urdf"
# 官方 URDF 限位（与 left_description_grip.urdf 逐字一致）
LIMITS = [
    (-2.827, 2.827),
    (-1.8064157758141308, 0.6806784082777885),
    (-2.8797932657906435, 2.8797932657906435),
    (-1.8151424220741028, 0.7592182246175333),
    (-2.7576202181510405, 2.7576202181510405),
    (-1.3264502315156903, 1.3264502315156903),
    (-1.335176877775662, 1.335176877775662),
]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # 官方零位姿态
# 果实点（TCP）预筛包络：与主脚本 PRECHECK 口径一致
X_MIN, RADIUS_MAX, Z_MIN, Z_MAX = D.X_MIN, D.RADIUS_MAX, D.Z_MIN, D.Z_MAX
EE_NAME = "link7"
SAMPLES = 40000
SEED = 20261118


def ee_index(robot):
    """按 link 名反查真实 link 索引（不硬编码）。"""
    for i in range(p.getNumJoints(robot)):
        if p.getJointInfo(robot, i)[12].decode() == EE_NAME:
            return i
    raise RuntimeError(f"未找到末端 link {EE_NAME}")


def fk_tcp(robot, idx):
    """返回 (TCP=果实被持点, link7 原点)。"""
    ls = p.getLinkState(robot, idx, computeForwardKinematics=1)
    R = np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)
    return np.array(ls[4]) + R @ np.array(D.TCP_OFFSET), np.array(ls[4])


def in_limits(q):
    return all(LIMITS[i][0] - 1e-9 <= q[i] <= LIMITS[i][1] + 1e-9 for i in range(7))


def refine_tcp(robot, idx, fruit, seed):
    """TCP 外环 IK（与主脚本 solve_ik_tcp 完全同口径），返回 (q, TCP 残差)。"""
    return D.solve_ik_tcp(robot, idx, fruit, seed=seed)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", action="store_true", help="打印可达空间俯视/侧视字符图")
    args = ap.parse_args()

    random.seed(SEED)
    p.connect(p.DIRECT)
    robot = p.loadURDF(URDF, useFixedBase=True)
    idx = ee_index(robot)
    print(f"末端 link 索引（按名反查）= {idx}，link 名 = {EE_NAME}")
    print(f"TCP（果实被持点 = ORCA 握合腔中心）link7 系偏移 = {D.TCP_OFFSET}")

    # ---- 1) 关节空间采样 → TCP 果实点云 ----
    cloud = []
    for _ in range(SAMPLES):
        q = [random.uniform(lo, hi) for lo, hi in LIMITS]
        for i in range(7):
            p.resetJointState(robot, i, q[i])
        tcp, pos7 = fk_tcp(robot, idx)
        r = math.hypot(tcp[0], tcp[1])
        if (Z_MIN <= tcp[2] <= Z_MAX and r <= RADIUS_MAX and tcp[0] >= X_MIN):
            cloud.append((tcp[0], tcp[1], tcp[2], tuple(q)))
    print(f"限位内采样 {SAMPLES} 组，落在前方带 x≥{X_MIN}、z∈[{Z_MIN},{Z_MAX}]、"
          f"r≤{RADIUS_MAX} 的果实可达点 {len(cloud)} 个")
    if not cloud:
        print("[FAIL] 该口径下无可达点，需放宽包络")
        return 1

    if args.map:
        for title, (ax_hi, ay_hi) in {"俯视 x-y": (0.55, 0.45), "侧视 x-z": (0.55, 0.45)}.items():
            print(f"\n--- 果实可达分布 {title} ---")
            grid = [[" "] * 57 for _ in range(25)]
            for c in cloud:
                a, b = (c[0], c[1]) if title.startswith("俯视") else (c[0], c[2])
                col = int((a + ax_hi) / (2 * ax_hi) * 56)
                row = int((ay_hi - b) / (2 * ay_hi) * 24)
                if 0 <= col < 57 and 0 <= row < 25:
                    grid[row][col] = "."
            print("\n".join("".join(r) for r in grid))
            print(f"  横轴 ∈[-{ax_hi},{ax_hi}] m，纵轴 ∈[-{ay_hi},{ay_hi}] m")

    # ---- 2) 草莓簇锚点（基座前方带内、左右对称、两个高度层） ----
    anchors = [
        ("P1", (0.42, 0.00, 0.34)),
        ("P2", (0.36, 0.18, 0.26)),
        ("P3", (0.36, -0.18, 0.26)),
        ("P4", (0.48, 0.08, 0.42)),
        ("P5", (0.46, -0.08, 0.16)),
    ]
    MIN_LIMIT_MARGIN = 0.15        # 最终解距限位至少 0.15 rad（避免贴限位）
    MIN_SEPARATION = 0.08          # 目标点两两最小间距 (m)
    CAND_RADIUS = 0.08             # 锚点邻域搜索半径 (m)
    TCP_RES_MAX = 0.002            # TCP 残差上限 (m，选点阶段从严)

    def limit_margin(q):
        return min(min(q[i] - LIMITS[i][0], LIMITS[i][1] - q[i]) for i in range(7))

    print("\n--- 锚点 → 邻域候选 → TCP 外环 IK（种子=候选点自身关节解）→ 限位余量择优 ---")
    picked, chosen_pts = [], []
    for name, anc in anchors:
        cands = sorted(cloud, key=lambda c: math.dist(c[:3], anc))[:600]
        cands = [c for c in cands if math.dist(c[:3], anc) <= CAND_RADIUS]
        best = None
        for c in cands:
            if any(math.dist(c[:3], o) < MIN_SEPARATION for o in chosen_pts):
                continue
            # 候选解 1：姿态约束解（与主脚本同一套 solve_tcp_pose/tcp_refine）
            q_ref, res_ref = refine_tcp(robot, idx, c[:3], c[3])
            # 候选解 2：候选点自身的关节解（FK 精确产生该 TCP；姿态分支未必满足约束，
            # 但作为可达解合法，仅在残差/限位/余量合格时参与择优）
            D.set_joints(robot, list(c[3]))
            tcp_c, _ = fk_tcp(robot, idx)
            res_c = math.dist(tcp_c, c[:3])
            sols = [(q_ref, res_ref), (list(c[3]), res_c)]
            picked_sol = None
            for q_s, res_s in sols:
                if res_s > TCP_RES_MAX or not in_limits(q_s):
                    continue
                m_s = limit_margin(q_s)
                if m_s < MIN_LIMIT_MARGIN:
                    continue
                if picked_sol is None or m_s > picked_sol[2]:
                    picked_sol = (q_s, res_s, m_s)
            if picked_sol is None:
                continue
            q, res, m = picked_sol
            score = math.dist(c[:3], anc)
            if best is None or score < best[0]:
                best = (score, c[:3], res, q, m)
        if best is None:
            print(f"  {name}: 邻域内无满足余量条件的候选 -> FAIL")
            continue
        _, tgt, res, q, m = best
        print(f"  {name}: 锚点({anc[0]:.2f},{anc[1]:+.2f},{anc[2]:.2f}) -> "
              f"果实点({tgt[0]:.4f},{tgt[1]:.4f},{tgt[2]:.4f}) TCP残差 {res * 1000:.3f} mm "
              f"限位余量 {m:.3f} rad PASS")
        picked.append((name, (round(tgt[0], 4), round(tgt[1], 4), round(tgt[2], 4)), round(res, 6), q))
        chosen_pts.append(tgt)

    print("\n--- 最终 TARGETS 字面量（TCP=果实被持点口径） ---")
    print("TARGETS = [")
    for name, t, res, _ in picked:
        print(f'    ("{name}", ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})),   # TCP残差 {res * 1000:.2f} mm')
    print("]")

    print("\n--- 果实可达点云统计 ---")
    for k, lbl in ((0, "x"), (1, "y"), (2, "z")):
        vals = [c[k] for c in cloud]
        print(f"  {lbl} ∈ [{min(vals):.4f}, {max(vals):.4f}] m")
    rs = [math.hypot(c[0], c[1]) for c in cloud]
    print(f"  r ∈ [{min(rs):.4f}, {max(rs):.4f}] m")

    with open("results/target_selection.json", "w", encoding="utf-8") as f:
        json.dump({"tcp_offset_link7": list(D.TCP_OFFSET),
                   "picked": [{"name": n, "target": list(t), "residual_m": r,
                               "q": [round(x, 6) for x in q]} for n, t, r, q in picked],
                   "cloud_count": len(cloud), "samples": SAMPLES, "seed": SEED,
                   "envelope": {"x_min": X_MIN, "r_max": RADIUS_MAX,
                                "z_min": Z_MIN, "z_max": Z_MAX}},
                  f, ensure_ascii=False, indent=1)
    p.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
