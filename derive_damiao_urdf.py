#!/usr/bin/env python3
"""derive_damiao_urdf.py - 从达妙官方 URDF 派生「仿真专用」URDF。

派生规则（严格按 PROMPT_FINAL_达妙七轴仿真迁移.md 坑 1 / 坑 2）：
  1. 删除官方夹爪子树：link tool_link / gripper_left / gripper_right
     与 joint tool_joint / gripper_joint1 / gripper_joint2 整段删除；
  2. 其余 8 个 link（base_link, link1..link7）与 7 个 joint 的
     几何、惯量、限位、effort/velocity、origin/axis 全部逐字保留，不做任何修改；
  3. 仅把 mesh 引用路径由 ROS 的 package:// 改为 PyBullet 可解析的相对路径
     （visual -> meshes/damiao/visual/*.STL，collision -> meshes/damiao/collision/*.STL）；
  4. 官方原文件不被触碰，派生结果单独另存。

用法：
  python derive_damiao_urdf.py \
      --src  ../damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/left_description_grip.urdf \
      --dst  damiao_reach.urdf

v8.3 新增（ORCA 末端执行器，口径见 PROMPT_ORCA末端执行器_v8.3_RELEASE.md §0.2/§4）：
  5. 在 link7 之后以 fixed joint 挂接 ORCA Hand 仿真专用子装配
     （来源 third_party/orcahand/orcahand_right_sim.urdf，由 derive_orcahand_urdf.py
     从官方 MIT 描述文件派生：惯量置零、17 个 revolute 锁定为预设握合姿态）；
     ORCA 不进 IK 链、不改 5 个目标点、不改误差口径（判定点仍为 link7 原点）；
  6. 达妙臂主体 8 link / 7 revolute / 15 个官方保留块逐字一致的要求不变，
     新增对 ORCA 子装配（42 link / 41 fixed + 1 个挂接 fixed joint）的强校验。
"""
import argparse
import os
import re
import sys

DROP_LINKS = {"tool_link", "gripper_left", "gripper_right"}
DROP_JOINTS = {"tool_joint", "gripper_joint1", "gripper_joint2"}

# ---- ORCA 子装配挂接参数（link7 -> orca_root，fixed） ----
# 朝向依据（2026-11-18 实测）：官方展开 URDF 在 root 系中手指沿 +z 延伸、卷向 +y
# （掌面朝 +y）。挂接变换将掌面转到 link7 局部 -z（果实方向）、握合腔中心对准
# 果实保持点 link7 局部 (0, 0, -FRUIT_HELD_Z=0.03)：手指朝腕前方伸展并从上方包拢果实。
ORCA_SIM_URDF = "third_party/orcahand/orcahand_right_sim.urdf"
ORCA_MOUNT_XYZ = "-0.15 0 -0.06"
ORCA_MOUNT_RPY = "1.5707963267948966 0 1.5707963267948966"
EXPECT_ORCA_LINKS, EXPECT_ORCA_JOINTS = 42, 41
EXPECT_ORCA_MESHES = 68

LINK_RE = re.compile(r"[ \t]*<link\b.*?</link>\s*", re.S)
JOINT_RE = re.compile(r"[ \t]*<joint\b.*?</joint>\s*", re.S)
NAME_RE = re.compile(r'name\s*=\s*"([^"]+)"')
MESH_RE = re.compile(r'(filename\s*=\s*")([^"]+)(")')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    args = ap.parse_args()

    with open(args.src, "r", encoding="utf-8") as f:
        text = f.read()

    # ---- 1. 抽取并删除夹爪子树的 link / joint ----
    kept_links, dropped_links = [], []
    for m in LINK_RE.finditer(text):
        name = NAME_RE.search(m.group(0)).group(1)
        (dropped_links if name in DROP_LINKS else kept_links).append(name)
    kept_joints, dropped_joints = [], []
    for m in JOINT_RE.finditer(text):
        name = NAME_RE.search(m.group(0)).group(1)
        (dropped_joints if name in DROP_JOINTS else kept_joints).append(name)

    missing = (DROP_LINKS - set(dropped_links)) | (DROP_JOINTS - set(dropped_joints))
    if missing:
        print(f"[FAIL] 官方 URDF 中未找到待裁剪元素: {sorted(missing)}")
        return 1

    def strip_links(t):
        def rep(m):
            name = NAME_RE.search(m.group(0)).group(1)
            return "" if name in DROP_LINKS else m.group(0)
        return LINK_RE.sub(rep, t)

    def strip_joints(t):
        def rep(m):
            name = NAME_RE.search(m.group(0)).group(1)
            return "" if name in DROP_JOINTS else m.group(0)
        return JOINT_RE.sub(rep, t)

    out = strip_links(text)
    out = strip_joints(out)

    # ---- 2. mesh 路径改写（仅路径，几何文件不动） ----
    rewritten = []

    def fix_mesh(m):
        path = m.group(2)
        if "package://" not in path:
            return m.group(0)
        rel = path.split("package://left_description_grip/", 1)[1]
        rel = rel.split("/", 1)[1] if rel.startswith("meshes/") else rel
        new = f"meshes/damiao/{rel}"
        rewritten.append((path, new))
        return m.group(1) + new + m.group(3)

    out = MESH_RE.sub(fix_mesh, out)

    # ---- 3. 机器人名与派生说明头 ----
    out = out.replace('<robot\n  name="left_description_grip">',
                      '<robot\n  name="damiao_7axis_reach">')
    header = (
        "<!-- ====================================================================\n"
        "     达妙科技开源七轴谐波机械臂 —— 仿真专用派生 URDF（含 ORCA Hand 末端执行器子装配）\n"
        "     来源一：damiao_7axis_arm/left_arm_dm/src/left_description_grip/urdf/\n"
        "           left_description_grip.urdf（SolidWorks URDF Exporter 1.6，BSD-3-Clause）\n"
        "     来源二：srl-ethz/orcahand_description（MIT License），\n"
        "           third_party/orcahand/orcahand_right_sim.urdf\n"
        "           （由 expand_orcahand_xacro.py 展开 + derive_orcahand_urdf.py 派生）\n"
        "     派生方式：derive_damiao_urdf.py 自动生成\n"
        "       1) 删除官方夹爪子树 link{tool_link,gripper_left,gripper_right}\n"
        "          与 joint{tool_joint,gripper_joint1,gripper_joint2}；\n"
        "       2) 保留 link{base_link,link1..link7} 与 joint1..joint7 的\n"
        "          几何、惯量、限位、effort/velocity、origin/axis 原值，逐字未改；\n"
        "       3) 仅改写 mesh 引用路径 package:// -> 相对路径（STL 文件本身未改）；\n"
        "       4) 在 link7 之后追加 ORCA Hand 子装配（fixed 挂接，惯量置零，\n"
        "          17 个 revolute 锁定为预设握合姿态；不进 IK 链，不改精度口径）。\n"
        "     说明：达妙臂未新建任何几何，未修改任何尺寸、惯量与限位数值。\n"
        "     ==================================================================== -->\n"
    )
    # XML 声明必须位于文件首行，派生说明插在声明之后
    decl = re.match(r"(<\?xml[^>]*\?>\s*)", out)
    if decl:
        out = decl.group(1) + header + out[decl.end():]
    else:
        out = header + out

    # ---- 3.5 追加 ORCA Hand 子装配（fixed 挂接到 link7，不进 IK 链） ----
    with open(ORCA_SIM_URDF, "r", encoding="utf-8") as f:
        orca_text = f.read()
    orca_body = orca_text[orca_text.index("<robot"):orca_text.index("</robot>")]
    orca_body = orca_body[orca_body.index(">") + 1:]          # 去掉 <robot ...> 开标签
    orca_mount = (
        "  <!-- v8.3 ORCA Hand 挂接：link7 -> orca_root（fixed，不进 IK 链）。\n"
        "       朝向：掌面向上托住果实（握合腔开口向上），指尖自腕前方 (+x)\n"
        "       向上包拢；握合腔中心对准果实保持点 link7 局部 (0, 0, -0.03)。\n"
        "       偏移与旋转依据官方展开 URDF 的 root 系几何实测确定。 -->\n"
        f'  <joint name="joint_orca_wrist" type="fixed">\n'
        f'    <origin xyz="{ORCA_MOUNT_XYZ}" rpy="{ORCA_MOUNT_RPY}" />\n'
        f'    <parent link="link7" />\n'
        f'    <child link="orca_root" />\n'
        f'  </joint>\n'
    )
    out = out.replace("</robot>", orca_mount + orca_body + "</robot>")

    with open(args.dst, "w", encoding="utf-8") as f:
        f.write(out)

    # ---- 4. 自检 ----
    with open(args.dst, "r", encoding="utf-8") as f:
        chk = f.read()
    n_links = len(re.findall(r"<link\b", chk))
    n_joints = len(re.findall(r"<joint\b", chk))
    n_revolute = len(re.findall(r'type\s*=\s*"revolute"', chk))
    n_prismatic = len(re.findall(r'type\s*=\s*"prismatic"', chk))
    n_fixed = len(re.findall(r'type\s*=\s*"fixed"', chk))
    leftover = [n for n in list(DROP_LINKS) + list(DROP_JOINTS) if f'"{n}"' in chk]
    pkg = len(re.findall(r'filename\s*=\s*"package://', chk))
    orca_mesh_missing = []
    for m in MESH_RE.finditer(chk):
        path = m.group(2)
        if path.startswith("meshes/orcahand/") and not os.path.exists(path):
            orca_mesh_missing.append(path)
    n_orca_links = len(re.findall(r'<link\s+name="orca_', chk))
    n_orca_joints = len(re.findall(r'<joint[^>]*name="orca_', chk))
    n_orca_mount = len(re.findall(r'<joint name="joint_orca_wrist" type="fixed">', chk))

    # ---- 5. 强校验：达妙保留块与官方逐字一致（忽略 mesh 路径与空白） ----
    def norm(block):
        b = re.sub(r'filename\s*=\s*"[^"]*"', 'filename="<PATH>"', block)
        return re.sub(r"\s+", " ", b).strip()

    def blocks(text):
        out = {}
        for m in list(LINK_RE.finditer(text)) + list(JOINT_RE.finditer(text)):
            out[NAME_RE.search(m.group(0)).group(1)] = norm(m.group(0))
        return out

    src_blocks, dst_blocks = blocks(text), blocks(chk)
    mismatch = [n for n in dst_blocks
                if n in src_blocks and src_blocks[n] != dst_blocks[n]]
    unchanged = [n for n in dst_blocks
                 if n in src_blocks and src_blocks[n] == dst_blocks[n]]

    # ORCA 子装配块与官方展开+派生结果逐字一致（忽略 mesh 路径）
    orca_blocks = blocks(chk)
    orca_src = blocks(orca_text)
    orca_mismatch = [n for n in orca_src
                     if n in orca_blocks and orca_src[n] != orca_blocks[n]]
    orca_unchanged = [n for n in orca_src
                      if n in orca_blocks and orca_src[n] == orca_blocks[n]]

    print(f"[派生] link 保留 {n_links}（期望 8+42=50），joint {n_joints}（期望 7+41+1=49）")
    print(f"[派生] 类型 revolute={n_revolute} prismatic={n_prismatic} fixed={n_fixed}（期望 7/0/42）")
    print(f"[派生] mesh 路径改写 {len(rewritten)} 处，全部 -> meshes/damiao/...")
    print(f"[派生] 残留夹爪引用: {leftover or '无'}；残留 package:// 网格引用: {pkg} 处")
    print(f"[派生] 达妙保留块与官方逐字一致: {len(unchanged)} 个；不一致: {mismatch or '无'}")
    print(f"[派生] ORCA 子装配: link={n_orca_links} joint={n_orca_joints}（期望 {EXPECT_ORCA_LINKS}/{EXPECT_ORCA_JOINTS}），"
          f"挂接 joint={n_orca_mount}（期望 1），mesh 缺失: {orca_mesh_missing or '无'}，"
          f"块与官方派生一致: {len(orca_unchanged)}/{len(orca_src)}；不一致: {orca_mismatch or '无'}")

    ok = (n_links == 8 + EXPECT_ORCA_LINKS
          and n_joints == 7 + EXPECT_ORCA_JOINTS + 1
          and n_revolute == 7 and n_prismatic == 0 and n_fixed == EXPECT_ORCA_JOINTS + 1
          and not leftover and pkg == 0 and not orca_mesh_missing
          and not mismatch and len(unchanged) == 15
          and n_orca_links == EXPECT_ORCA_LINKS and n_orca_joints == EXPECT_ORCA_JOINTS
          and n_orca_mount == 1 and not orca_mismatch
          and len(orca_unchanged) == len(orca_src) == EXPECT_ORCA_LINKS + EXPECT_ORCA_JOINTS)
    print("== DERIVE PASS ==" if ok else "== DERIVE FAILED ==")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
