#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""expand_orcahand_xacro.py - 把 ORCA Hand 官方 xacro 展开为确定性 URDF（可复现）。

来源：srl-ethz/orcahand_description（MIT License），官方文件 orcahand.urdf.xacro
      原样复制于本目录（orcahand.urdf.xacro），未做任何修改。

为什么需要：官方只提供 .urdf.xacro（1164 行，含 $(arg) 与 ${} 表达式），
PyBullet 不能直接读取，必须先确定性展开。

展开口径（v8.3）：
  chirality:=right  prefix:=orca_  extension:=true

与《PROMPT_ORCA末端执行器_v8.3_RELEASE.md》§4.2 示例命令（extension:=false）的
差异说明：示例命令展开后只有 62 处 mesh 引用，与同提示词 §4.3 / §八 验收要求的
「68 处 package:// 改写、34+34 个 STL 全部可达」矛盾；本工程按验收要求取
extension:=true（68 处 mesh 引用、34+34 个官方 STL 全部被引用），特此登记。

复现命令（二选一）：
  python expand_orcahand_xacro.py
  # 或等价的 ROS2 命令：
  ros2 run xacro xacro orcahand.urdf.xacro chirality:=right prefix:=orca_ extension:=true > orcahand_right.urdf

输出：orcahand_right.urdf（与本目录 xacro 展开结果逐字一致，后续由工程根目录的
derive_orcahand_urdf.py 派生仿真专用版本）。
"""
import os

import xacro

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "orcahand.urdf.xacro")
DST = os.path.join(HERE, "orcahand_right.urdf")

MAPPINGS = {"chirality": "right", "prefix": "orca_", "extension": "true"}


def main() -> int:
    doc = xacro.process_file(SRC, mappings=MAPPINGS)
    text = doc.toprettyxml(indent="  ")
    with open(DST, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"[展开] {os.path.relpath(SRC, HERE)} -> {os.path.relpath(DST, HERE)}")
    print(f"[展开] mappings = {MAPPINGS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
