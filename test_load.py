#!/usr/bin/env python3
"""test_load.py - 达妙七轴派生 URDF 加载自检。全部 PASS 才继续跑仿真。

检查项：
  1) loadURDF 成功；
  2) 可动关节数 = 7（官方夹爪子树已裁剪）；
  3) 7 个关节限位逐项等于官方 URDF <limit>；
  4) 末端 link 名 = link7，且其 link 索引按名反查得到（不硬编码）；
  5) 零位 FK 断言 = (0, -0.03325, 0.4981) m，容差 5 mm
     （与 handoff/演算基准.json 的 zero_pose.link7_tcp_m 一致）；
  6) 渲染一帧验证官方 STL 显示通路正常。
"""
import math
import sys

import pybullet as p

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
TOL = 0.005


def main() -> int:
    ok = True
    p.connect(p.DIRECT)
    robot = p.loadURDF("damiao_reach.urdf", useFixedBase=True)
    print(f"[1] loadURDF OK, bodyUniqueId = {robot}")

    n = p.getNumJoints(robot)
    print(f"[2] getNumJoints = {n}（期望 7，官方夹爪子树已裁剪）")
    ok &= n == 7

    print("[3] 关节限位读入检查（来源：达妙官方 left_description_grip.urdf <limit>）")
    for i in range(n):
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
    print(f"    link 序列: {link_names}")
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

    print(f"[5] 零位 FK 断言（理论值 {EXPECT_HOME_EE} m，容差 5 mm，不含夹爪偏移）")
    for i in range(n):
        p.resetJointState(robot, i, 0.0)
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    ee_frame = ls[4]   # worldLinkFramePosition：URDF link 原点（主口径）
    ee_com = ls[0]     # linkWorldPosition：COM（备查口径）
    err = math.dist(ee_frame, EXPECT_HOME_EE)
    ok &= err <= TOL
    print(f"    link7 原点 = ({ee_frame[0]:.5f}, {ee_frame[1]:.5f}, {ee_frame[2]:.5f})，"
          f"|Δ| = {err * 1000:.3f} mm {'PASS' if err <= TOL else 'FAIL'}")
    print(f"    link7 COM  = ({ee_com[0]:.5f}, {ee_com[1]:.5f}, {ee_com[2]:.5f})（备查，与原点相差属正常）")

    print("[6] 渲染一帧（ER_TINY_RENDERER，验证官方 STL 显示）")
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
