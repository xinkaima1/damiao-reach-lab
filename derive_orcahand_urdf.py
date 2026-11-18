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
