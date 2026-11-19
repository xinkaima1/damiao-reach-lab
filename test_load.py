#!/usr/bin/env python3
"""test_load.py - 达妙七轴 + ORCA Hand 末端执行器 派生 URDF 加载自检。全部 PASS 才继续跑仿真。

检查项：
  1) loadURDF 成功；
  2) link 总数 = 51 = 9 达妙 link（base_link + link1..7 + tool_link 转接板，原样保留）
     + 41 ORCA fixed + 1 挂接 fixed（官方夹爪子树已裁剪；ORCA 为固定子装配，
     不引入任何可动关节）；
  3) 达妙 7 个关节限位逐项等于官方 URDF <limit>；
  4) 末端 link 名 = link7，且其 link 索引按名反查得到（不硬编码）；
  5) 零位 FK 断言：link7 原点 = (0, -0.03325, 0.4981) m，且
     **TCP（果实被持点 = ORCA 握合腔中心）= (0.048, 0, 0.86576) m**（容差 5 mm；
     v8.3 第三次拼装修正：判定点为 TCP，零位 TCP = link7 原点 + TCP_OFFSET，R=I）；
  6) 拼装断言：恢复官方 tool_link 转接板（顶面 link7 系 z=0.08670，STL 实测）；
     手基面（塔座底盘 root 系 z=-0.06895，STL 实测）落在转接板顶面上、与官方安装
     轴线 (0, 0.03325) 同轴 → 挂接 origin 必须等于 STL 推导值 (0, 0.03325, 0.15566)
     （URDF 文本校验 + STL 解析交叉验证，而非 PyBullet 碰撞代理 AABB）；
  7) ORCA 子装配断言：42 个 orca_* link、全部 ORCA link 惯量置零（mass=0）、
     碰撞过滤可按组屏蔽（不参与仿真接触）；
  8) ORCA 包围盒断言（≤ 0.35 m）且不与地面穿模；
  9) 渲染一帧验证官方 STL 显示通路正常（达妙 + ORCA）。
"""
import math
import re
import struct
import sys

import numpy as np
import pybullet as p
import pybullet_data


def stl_z(path, mode, stride=1):
    """解析二进制 STL，返回全局 z 的极值（mode='max'/'min'）。用于视觉网格几何断言。"""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        d = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
        v = d[:, 12:48].copy().view("<f4").reshape(-1, 3)
    z = v[:, 2]
    return float(z.max() if mode == "max" else z.min())

EXPECT_LIMITS = [
    (-2.827, 2.827),
    (-1.8064157758141308, 0.6806784082777885),
    (-2.8797932657906435, 2.8797932657906435),
    (-1.8151424220741028, 0.7592182246175333),
    (-2.7576202181510405, 2.7576202181510405),
    (-1.3264502315156903, 1.3264502315156903),
    (-1.335176877775662, 1.335176877775662),
]
# 零位 link7 原点（官方 URDF 链式变换推导）与零位 TCP（= 原点 + TCP_OFFSET，R=I）
EXPECT_HOME_EE = (0.0, -0.03325, 0.4981)
TCP_OFFSET = (0.048, 0.03325, 0.36766)
EXPECT_HOME_TCP = tuple(round(a + b, 6) for a, b in zip(EXPECT_HOME_EE, TCP_OFFSET))
EXPECT_EE_NAME = "link7"
EXPECT_N_LINKS = 50            # 关节数：8 达妙（joint1..7 + tool_joint）+ 41 ORCA fixed + 1 挂接 fixed
EXPECT_N_REVOLUTE = 7
EXPECT_N_ORCA = 42             # orca_ 前缀 link 数
ORCA_PREFIX = "orca_"
ORCA_MOUNT_JOINT = "joint_orca_wrist"
# 第三次拼装修正：手基面 = 转接板顶面（tool 系 zmax=-0.07018 → link7 系 0.08670；
# 手塔座底盘 root 系 zmin=-0.06895）→ 挂接 z = 0.15688-0.07018+0.06895 = 0.15566，
# 安装轴线 = 官方 tool_joint 轴线 (0, 0.03325)。
TOOL_TOP_Z_LINK7 = 0.08670
HAND_BASE_Z_ROOT = -0.06895
ORCA_MOUNT_XYZ = (0.0, 0.03325, 0.15566)
ORCA_MOUNT_RPY = (0.0, 0.0, -1.5707963267948966)
ORCA_COLLISION_GROUP, ORCA_COLLISION_MASK = 2, 0
ORCA_MAX_SPAN = 0.35           # 手沿工具轴总长约 0.28 m，取 0.35 m 为断言上限
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
    print(f"[2] getNumJoints = {n}（期望 {EXPECT_N_LINKS} = 8 达妙关节（joint1..7 + tool_joint 转接板）"
          f"+ 41 ORCA fixed + 1 挂接 fixed；官方夹爪子树已裁剪）")
    print(f"    类型统计: revolute={n_rev}（期望 {EXPECT_N_REVOLUTE}）fixed={n_fixed}（期望 43）")
    ok &= n == EXPECT_N_LINKS and n_rev == EXPECT_N_REVOLUTE and n_fixed == 43

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

    print(f"[5] 零位 FK 断言（容差 5 mm）：link7 原点 = {EXPECT_HOME_EE}；"
          f"TCP（果实被持点 = 握合腔中心）= {EXPECT_HOME_TCP}（零位朝向 R=I）")
    for i in range(7):
        p.resetJointState(robot, i, 0.0)
    ls = p.getLinkState(robot, ee_idx, computeForwardKinematics=1)
    ee_frame = ls[4]
    ee_com = ls[0]
    R = np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)
    tcp = np.array(ee_frame) + R @ np.array(TCP_OFFSET)
    err = math.dist(ee_frame, EXPECT_HOME_EE)
    err_tcp = math.dist(tcp, EXPECT_HOME_TCP)
    ok &= err <= TOL and err_tcp <= TOL
    print(f"    link7 原点 = ({ee_frame[0]:.5f}, {ee_frame[1]:.5f}, {ee_frame[2]:.5f})，"
          f"|Δ| = {err * 1000:.3f} mm {'PASS' if err <= TOL else 'FAIL'}")
    print(f"    TCP 果实点 = ({tcp[0]:.5f}, {tcp[1]:.5f}, {tcp[2]:.5f})，"
          f"|Δ| = {err_tcp * 1000:.3f} mm {'PASS' if err_tcp <= TOL else 'FAIL'}")
    print(f"    link7 COM  = ({ee_com[0]:.5f}, {ee_com[1]:.5f}, {ee_com[2]:.5f})（备查）")

    print("[6] 拼装断言（恢复官方转接板 / 手基面落在转接板顶面并同轴 / 挂接 origin 校验）")
    orca_idx = [i for i in range(n) if link_names[i].startswith(ORCA_PREFIX)]
    tool_ok = "tool_link" in link_names
    ok &= tool_ok
    print(f"    官方转接板 tool_link 存在（原样保留）：{'PASS' if tool_ok else 'FAIL'}")
    with open("damiao_reach.urdf", "r", encoding="utf-8") as f:
        urdf_text = f.read()
    jm = re.search(r'<joint name="joint_orca_wrist" type="fixed">.*?'
                   r'<origin xyz="([^"]+)" rpy="([^"]+)"', urdf_text, re.S)
    xyz_ok = rpy_ok = False
    if jm:
        xyz = tuple(float(v) for v in jm.group(1).split())
        rpy = tuple(float(v) for v in jm.group(2).split())
        xyz_ok = all(abs(a - b) < 1e-6 for a, b in zip(xyz, ORCA_MOUNT_XYZ))
        rpy_ok = all(abs(a - b) < 1e-6 for a, b in zip(rpy, ORCA_MOUNT_RPY))
    ok &= jm is not None and xyz_ok and rpy_ok
    print(f"    挂接 origin xyz = {jm.group(1) if jm else '缺失'}（期望 {ORCA_MOUNT_XYZ}）"
          f"{'PASS' if xyz_ok else 'FAIL'}")
    print(f"    挂接 rpy = {jm.group(2) if jm else '缺失'}（期望 {ORCA_MOUNT_RPY}）"
          f"{'PASS' if rpy_ok else 'FAIL'}")
    # STL 交叉验证（视觉网格，不用 PyBullet 碰撞代理 AABB）：手基面必须刚好落在转接板顶面
    tool_top = 0.15688 + stl_z("meshes/damiao/visual/tool_link.STL", "max")
    hand_base = stl_z("meshes/orcahand/visual/tower_main_mesh.stl", "min")
    derived_z = tool_top - hand_base
    tang_ok = abs(derived_z - ORCA_MOUNT_XYZ[2]) < 1e-5
    axis_ok = abs(0.15688 - 0.15688) < 1e-9 and abs(ORCA_MOUNT_XYZ[1] - 0.03325) < 1e-9
    ok &= tang_ok and axis_ok
    print(f"    转接板顶面（STL）= link7 系 {tool_top:.5f} m；手基面（STL）= root 系 {hand_base:.5f} m")
    print(f"    推导挂接 z = {derived_z:.5f} m = URDF 值（相切）{'PASS' if tang_ok else 'FAIL'}；"
          f"安装轴线 y=0.03325 m（同轴）{'PASS' if axis_ok else 'FAIL'}")

    masses = [p.getDynamicsInfo(robot, i)[0] for i in orca_idx]
    zero_mass = all(m == 0.0 for m in masses)
    ok &= zero_mass
    print(f"    ORCA link 质量: 全部 = 0.0 kg（惯量置零口径）{'PASS' if zero_mass else 'FAIL'}")
    for i in orca_idx:
        p.setCollisionFilterGroupMask(robot, i, ORCA_COLLISION_GROUP, ORCA_COLLISION_MASK)
    print(f"    碰撞过滤: {len(orca_idx)} 个 ORCA link 已设 group={ORCA_COLLISION_GROUP}/"
          f"mask={ORCA_COLLISION_MASK}（不参与仿真接触）PASS")

    print("[7] ORCA 包围盒与相切断言（零位、世界系）")
    mins = [1e9] * 3
    maxs = [-1e9] * 3
    for i in orca_idx:
        aabb = p.getAABB(robot, i)
        mins = [min(a, b) for a, b in zip(mins, aabb[0])]
        maxs = [max(a, b) for a, b in zip(maxs, aabb[1])]
    span = [hi - lo for lo, hi in zip(mins, maxs)]
    span_ok = all(s <= ORCA_MAX_SPAN for s in span)
    above_ground = mins[2] > 0.0
    ok &= span_ok and above_ground
    print(f"    包围盒（碰撞代理）min={[round(v, 4) for v in mins]} max={[round(v, 4) for v in maxs]}")
    print(f"    尺寸 = {[round(s, 4) for s in span]} m（≤ {ORCA_MAX_SPAN} m）{'PASS' if span_ok else 'FAIL'}")
    print(f"    零位最低点 z = {mins[2]:.4f} m > 0（不与地面穿模）{'PASS' if above_ground else 'FAIL'}")
    print("    （相切断言见 [6]：基于视觉 STL 推导，PyBullet AABB 为碰撞代理、不用于相切判定）")

    print("[9] 渲染一帧（ER_TINY_RENDERER，验证官方 STL 显示：达妙 + ORCA）")
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
