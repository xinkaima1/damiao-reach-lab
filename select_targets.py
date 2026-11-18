#!/usr/bin/env python3
"""select_targets.py - 达妙七轴臂可达空间扫描与 5 个目标点选取（可复现）。

用途：原 xArm6 的 5 个目标点在达妙臂上全部不可达（IK 残差 146–200 mm），
本脚本按「官方 URDF 限位 + 官方几何」重新扫描 link7 可达空间，
并从中挑出 5 个草莓簇状分布、IK 残差 ≤ 5 mm 的目标点。

方法：
  1) 关节空间随机采样（限位内）→ 正运动学 → link7 可达点云；
  2) 按 z∈[Z_MIN,Z_MAX]、r≤RADIUS_MAX 过滤；
  3) 对预设的草莓簇锚点各取云中最近点作 IK 初值，再跑位置-only IK 精修；
  4) 打印最终 5 点坐标与残差，供写入 damiao_reach_demo.py 的 TARGETS。

用法：python select_targets.py [--map]
"""
import argparse
import json
import math
import random

import pybullet as p

import damiao_reach_demo as D          # 复用主脚本的两段式 IK（内置粗解 + 位置-only 数值精修）

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
RADIUS_MAX, Z_MIN, Z_MAX = 0.35, 0.15, 0.45     # 预筛口径（r_max≈0.389 取保守值）
EE_NAME = "link7"
SAMPLES = 40000
SEED = 20261118


def ee_index(robot):
    """按 link 名反查真实 link 索引（不硬编码）。"""
    for i in range(p.getNumJoints(robot)):
        if p.getJointInfo(robot, i)[12].decode() == EE_NAME:
            return i
    raise RuntimeError(f"未找到末端 link {EE_NAME}")


def fk(robot, idx):
    return p.getLinkState(robot, idx, computeForwardKinematics=1)[4]


def in_limits(q):
    return all(LIMITS[i][0] - 1e-9 <= q[i] <= LIMITS[i][1] + 1e-9 for i in range(7))


def refine(robot, idx, target, seed):
    """两段式位置-only IK：PyBullet 内置粗解（含限位与 HOME nullspace 参考）+ 数值精修。
    与主脚本 damiao_reach_demo.solve_ik() 完全一致，保证选点与实跑同口径。"""
    q = D.solve_ik(robot, idx, target, seed=seed)
    return q, math.dist(fk(robot, idx), target)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", action="store_true", help="打印可达空间俯视/侧视字符图")
    args = ap.parse_args()

    random.seed(SEED)
    p.connect(p.DIRECT)
    robot = p.loadURDF(URDF, useFixedBase=True)
    idx = ee_index(robot)
    print(f"末端 link 索引（按名反查）= {idx}，link 名 = {EE_NAME}")

    # ---- 1) 关节空间采样 → 可达点云 ----
    cloud = []
    for _ in range(SAMPLES):
        q = [random.uniform(lo, hi) for lo, hi in LIMITS]
        for i in range(7):
            p.resetJointState(robot, i, q[i])
        pt = fk(robot, idx)
        r = math.hypot(pt[0], pt[1])
        if Z_MIN <= pt[2] <= Z_MAX and r <= RADIUS_MAX:
            cloud.append((pt[0], pt[1], pt[2], tuple(q)))
    print(f"限位内采样 {SAMPLES} 组，落在 z∈[{Z_MIN},{Z_MAX}] 且 r≤{RADIUS_MAX} 的可达点 {len(cloud)} 个")
    if not cloud:
        print("[FAIL] 该口径下无可达点，需放宽 Z/R 范围")
        return 1

    if args.map:
        # 俯视 (x-y) 与侧视 (x-z) 字符图
        for title, (ax, ay, hx, hy, axmax, aymax) in {
            "俯视 x-y": (0, 1, 0.35, 0.35, 0.35, 0.35),
            "侧视 x-z": (0, 2, 0.35, 0.45, 0.35, 0.45),
        }.items():
            print(f"\n--- 可达分布 {title} ---")
            grid = [[" "] * 57 for _ in range(25)]
            for c in cloud:
                x, y, z = c[0], c[1], c[2]
                if title.startswith("俯视"):
                    a, b = x, y
                else:
                    a, b = x, z
                col = int((a + hx) / (2 * hx) * 56)
                row = int((aymax - b) / (2 * aymax) * 24)
                if 0 <= col < 57 and 0 <= row < 25:
                    grid[row][col] = "."
            print("\n".join("".join(r) for r in grid))
            print(f"  横轴 {ax}∈[-{hx},{hx}] m，纵轴 {ay}∈[-{aymax},{aymax}] m")

    # ---- 2) 草莓簇锚点（基座前方 +x、左右对称、两个高度层） ----
    anchors = [
        ("P1", (0.22, 0.00, 0.32)),
        ("P2", (0.17, 0.11, 0.24)),
        ("P3", (0.17, -0.11, 0.24)),
        ("P4", (0.28, 0.05, 0.40)),
        ("P5", (0.28, -0.05, 0.17)),
    ]
    MIN_LIMIT_MARGIN = 0.15        # 最终解距限位至少 0.15 rad（避免贴限位）
    MIN_SEPARATION = 0.08          # 目标点两两最小间距 (m)
    CAND_RADIUS = 0.08             # 锚点邻域搜索半径 (m)

    def limit_margin(q):
        return min(min(q[i] - LIMITS[i][0], LIMITS[i][1] - q[i]) for i in range(7))

    print("\n--- 锚点 → 邻域候选 → 链式两段式 IK（种子=上一点解）→ 限位余量择优 ---")
    picked, chosen_pts = [], []
    chain_seed = list(HOME)
    for name, anc in anchors:
        cands = sorted(cloud, key=lambda c: math.dist(c[:3], anc))[:600]
        cands = [c for c in cands if math.dist(c[:3], anc) <= CAND_RADIUS]
        best = None
        for c in cands:
            if any(math.dist(c[:3], o) < MIN_SEPARATION for o in chosen_pts):
                continue
            q, res = refine(robot, idx, c[:3], chain_seed)   # 链式种子 → 与实跑同口径
            if res > 0.002 or not in_limits(q):
                continue
            m = limit_margin(q)
            if m < MIN_LIMIT_MARGIN:
                continue
            score = math.dist(c[:3], anc)
            if best is None or score < best[0]:
                best = (score, c[:3], res, q, m)
        if best is None:
            print(f"  {name}: 邻域内无满足余量条件的候选 -> FAIL")
            continue
        _, tgt, res, q, m = best
        chain_seed = q
        print(f"  {name}: 锚点({anc[0]:.2f},{anc[1]:+.2f},{anc[2]:.2f}) -> "
              f"目标({tgt[0]:.4f},{tgt[1]:.4f},{tgt[2]:.4f}) 残差 {res * 1000:.3f} mm "
              f"限位余量 {m:.3f} rad PASS")
        picked.append((name, (round(tgt[0], 4), round(tgt[1], 4), round(tgt[2], 4)), round(res, 6), q))
        chosen_pts.append(tgt)

    print("\n--- 最终 TARGETS 字面量 ---")
    print("TARGETS = [")
    for name, t, res, _ in picked:
        print(f'    ("{name}", ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})),   # 残差 {res * 1000:.2f} mm')
    print("]")

    print("\n--- 可达点云统计 ---")
    for k, lbl in ((0, "x"), (1, "y"), (2, "z")):
        vals = [c[k] for c in cloud]
        print(f"  {lbl} ∈ [{min(vals):.4f}, {max(vals):.4f}] m")
    rs = [math.hypot(c[0], c[1]) for c in cloud]
    print(f"  r ∈ [{min(rs):.4f}, {max(rs):.4f}] m")

    with open("results/target_selection.json", "w", encoding="utf-8") as f:
        json.dump({"picked": [{"name": n, "target": list(t), "residual_m": r,
                               "q": [round(x, 6) for x in q]} for n, t, r, q in picked],
                   "cloud_count": len(cloud), "samples": SAMPLES, "seed": SEED},
                  f, ensure_ascii=False, indent=1)
    p.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
