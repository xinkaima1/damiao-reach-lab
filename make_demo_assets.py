#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_demo_assets.py —— 从 results/reach_demo.mp4 派生答辩演示素材。

产出（与 xArm6 版同规格，便于答辩沿用同一套素材用法）：
  results/reach_demo.gif       400×300 / 约 178 帧 / 160 ms 每帧 循环动图（PIL 会合并静止帧，以回读值为准）
  results/reach_keyframes.png  1440×784 六帧拼图（含帧号、时间戳与阶段中文标签）

用法：
    python make_demo_assets.py
    python make_demo_assets.py --gif-size 400x300 --gif-stride 5 --fps 6
"""
import argparse
import os

import imageio
import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
MP4 = os.path.join(HERE, "results", "reach_demo.mp4")
GIF = os.path.join(HERE, "results", "reach_demo.gif")
KF = os.path.join(HERE, "results", "reach_keyframes.png")

FONT_CANDIDATES = ["/mnt/c/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/simhei.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]

# 关键帧：帧号 -> (阶段标签, 补充说明)（v8.3 ORCA Hand 口径）
KEYFRAMES = {
    0:    ("起始：ORCA Hand 握合待机", "末端执行器为固定子装配"),
    100:  ("接近目标：握合姿态保持", "随末端移动，果实为红球标记"),
    200:  ("到位：指掌握合腔对准果实", "预设握合姿态，无开合动画"),
    430:  ("握合腔包拢果实", "纯视觉移动，无接触力"),
    700:  ("搬运入篮", "果实随末端移动到篮内"),
    975:  ("终态：篮中 5 颗", "全部目标点已判定通过"),
}
# --legacy-tentacle 分支（v8.2 复现口径）使用的旧标签
KEYFRAMES_LEGACY = {
    0:    ("起始：触手卷曲待机", "末端后方对数螺旋团"),
    100:  ("接近目标：保持卷曲", "随末端移动，果实为红球标记"),
    200:  ("到位前：反向展开", "触手由卷曲转为伸展"),
    430:  ("缠绕包覆果实", "沿球面缠绕，纯视觉无接触力"),
    700:  ("搬运入篮", "果实随末端移动到篮内"),
    975:  ("终态：篮中 5 颗", "全部目标点已判定通过"),
}


def load_font(size):
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    print("  [警告] 未找到可用字体，标签可能显示为方块")
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--legacy-tentacle", action="store_true",
                    help="使用 v8.2 章鱼爪口径的关键帧标签（对应 --legacy-tentacle 录像）")
    ap.add_argument("--mp4", default=None, help="源录像路径（默认 results/reach_demo.mp4）")
    ap.add_argument("--gif-size", default="400x300")
    ap.add_argument("--gif-stride", type=int, default=5, help="每隔 N 帧取 1 帧进 GIF")
    ap.add_argument("--fps", type=float, default=6.0, help="GIF 播放帧率")
    ap.add_argument("--colors", type=int, default=32, help="GIF 调色板颜色数（越大越大越清晰）")
    args = ap.parse_args()
    gw, gh = (int(v) for v in args.gif_size.lower().split("x"))

    src_mp4 = args.mp4 or MP4
    keyframes = KEYFRAMES_LEGACY if args.legacy_tentacle else KEYFRAMES
    if not os.path.exists(src_mp4):
        raise SystemExit(f"[错误] 找不到 {src_mp4}，请先跑 damiao_reach_demo.py 生成完整运行录像")
    frames = iio.imread(src_mp4, plugin="FFMPEG")
    total = frames.shape[0]
    print(f"[素材] 源录像 {total} 帧 @ {frames.shape[2]}×{frames.shape[1]}")

    # ---------- GIF 循环动图 ----------
    picks = list(range(0, total, args.gif_stride))
    gif_frames = [np.asarray(Image.fromarray(frames[i]).resize((gw, gh), Image.LANCZOS))
                  for i in picks]
    # 量化到自适应调色板并跨帧优化：官方 STL 场景色彩数远多于 xArm6 版，
    # 不量化会让 GIF 涨到 4 MB+；量化后与原仓库同量级（数百 KB）。
    pal = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=args.colors)
           for f in gif_frames]
    # 不设 disposal=2：整帧存储会让 GIF 体积翻倍；默认 disposal 允许帧间差分
    pal[0].save(GIF, save_all=True, append_images=pal[1:],
                duration=int(round(1000.0 / args.fps)), loop=0, optimize=True)
    # 回读真实参数：PIL 的 optimize 会合并连续相同的帧，写入帧数可能少于取样帧数，
    # 文档与 README 一律引用回读值（不引用估算值）。
    with Image.open(GIF) as chk:
        real_n = 0
        try:
            while True:
                chk.seek(real_n)
                real_n += 1
        except EOFError:
            pass
        dur_ms = chk.info.get("duration", int(round(1000.0 / args.fps)))
    print(f"[素材] GIF：{gw}×{gh} / 取样 {len(gif_frames)} 帧 → 实际写入 {real_n} 帧 / "
          f"{dur_ms} ms 每帧（≈{1000.0 / dur_ms:.1f} fps，循环约 {real_n * dur_ms / 1000:.1f} s）/ "
          f"{args.colors} 色 / {os.path.getsize(GIF) // 1024} KB -> {os.path.relpath(GIF, HERE)}")

    # ---------- 六帧关键帧拼图（3 列 × 2 行） ----------
    cw, ch = 480, 392
    font_lbl = load_font(22)
    font_sub = load_font(16)
    canvas = Image.new("RGB", (cw * 3, ch * 2), (18, 18, 20))
    draw = ImageDraw.Draw(canvas)
    for k, idx in enumerate(sorted(KEYFRAMES)):
        if idx >= total:
            idx = total - 1
        img = Image.fromarray(frames[idx]).resize((cw, ch), Image.LANCZOS)
        x, y = (k % 3) * cw, (k // 3) * ch
        canvas.paste(img, (x, y))
        label, sub = keyframes[idx]
        draw.rectangle([x, y + ch - 58, x + cw, y + ch], fill=(0, 0, 0))
        draw.text((x + 10, y + ch - 52), f"#{idx}  t={idx / 30:.1f}s  {label}",
                  font=font_lbl, fill=(255, 235, 120))
        draw.text((x + 10, y + ch - 26), sub, font=font_sub, fill=(225, 225, 225))
    canvas.save(KF)
    print(f"[素材] 关键帧拼图：{canvas.size[0]}×{canvas.size[1]} / 6 帧 / "
          f"{os.path.getsize(KF) // 1024} KB -> {os.path.relpath(KF, HERE)}")


if __name__ == "__main__":
    main()
