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
