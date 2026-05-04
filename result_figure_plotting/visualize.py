#!/usr/bin/env python3
"""CLI wrapper: render images (4 angles × N viz types) from a URDF using Blender Cycles,
then composite each angle side-by-side and write a four-row contact sheet.

Usage:
    python plots/visualize.py --urdf data/cache/record_materialization/<id>/model.urdf
    python plots/visualize.py --urdf /abs/path/model.urdf --output plots/ --samples 64
    python plots/visualize.py --urdf ... --modes motion_1,motion_30,part_segmentation,collision
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

BLENDER = "/Applications/Blender.app/Contents/MacOS/blender"
SCRIPT = Path(__file__).parent / "render_urdf_viz.py"
DEFAULT_HDRI = str(Path(__file__).parent / "symmetrical_garden_02_4k.exr")

ALL_MODES = ["motion_1", "motion_30", "part_segmentation", "collision"]
LABELS = {
    "motion_0":          "Rest",
    "motion_1":          "Motion 1 deg",
    "motion_15":         "Motion 15 deg",
    "motion_30":         "Motion 30 deg",
    "motion_half":       "Motion Halfway",
    "part_segmentation": "Part Segmentation",
    "collision":         "Collision",
    "joint_overlay":     "Joint Overlay",
}
ANGLE_LABELS = {
    "angle_0": "Front",
    "angle_1": "Right",
    "angle_2": "Back",
    "angle_3": "Left",
}


def resolve_record_id(urdf: Path) -> str:
    parts = urdf.parts
    for marker in ("record_materialization", "records"):
        if marker in parts:
            idx = parts.index(marker)
            if idx + 1 < len(parts):
                return parts[idx + 1]
    return urdf.parent.name


def make_composite(angle_dir: Path, out_path: Path, modes: list[str]) -> bool:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print("Pillow not installed — skipping composite (pip install pillow)")
        return False

    imgs = []
    for mode in modes:
        p = angle_dir / f"{mode}.png"
        if not p.exists():
            print(f"  Missing {p.name}, cannot composite")
            return False
        imgs.append(Image.open(p).convert("RGBA"))

    W, H = imgs[0].size
    N = len(imgs)
    PAD = 12
    LABEL_H = 44
    FONT_SIZE = 28

    grid_w = W * N + PAD * (N + 1)
    grid_h = H + LABEL_H + PAD * 2
    composite = Image.new("RGBA", (grid_w, grid_h), (255, 255, 255, 0))

    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", FONT_SIZE)
    except Exception:
        font = ImageFont.load_default()

    draw = ImageDraw.Draw(composite)

    for col, (img, mode) in enumerate(zip(imgs, modes)):
        x = PAD + col * (W + PAD)
        y = PAD
        bg = Image.new("RGBA", (W, H), (245, 245, 245, 255))
        bg.paste(img, mask=img)
        composite.paste(bg, (x, y))
        label = LABELS.get(mode, mode)
        bbox = draw.textbbox((0, 0), label, font=font)
        tw = bbox[2] - bbox[0]
        draw.text((x + (W - tw) // 2, y + H + 8), label, fill=(40, 40, 40, 255), font=font)

    composite.save(out_path, "PNG")
    print(f"  Composite → {out_path.name}")
    return True


def make_contact_sheet(composite_paths: list[Path], out_path: Path) -> bool:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return False

    rows = []
    for path in composite_paths:
        if path.exists():
            rows.append((path.parent.name, Image.open(path).convert("RGBA")))
    if not rows:
        return False

    label_w = 96
    pad = 12
    row_w = max(img.size[0] for _, img in rows)
    total_w = label_w + row_w + pad * 3
    total_h = sum(img.size[1] for _, img in rows) + pad * (len(rows) + 1)
    sheet = Image.new("RGBA", (total_w, total_h), (245, 245, 245, 255))

    try:
        font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 26)
    except Exception:
        font = ImageFont.load_default()

    draw = ImageDraw.Draw(sheet)
    y = pad
    for angle_name, img in rows:
        label = ANGLE_LABELS.get(angle_name, angle_name)
        bbox = draw.textbbox((0, 0), label, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        draw.text((pad + (label_w - tw) // 2, y + (img.size[1] - th) // 2),
                  label, fill=(35, 35, 35, 255), font=font)
        sheet.paste(img, (label_w + pad * 2, y))
        y += img.size[1] + pad

    sheet.save(out_path, "PNG")
    print(f"  Contact sheet → {out_path.name}")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", required=True, type=Path, help="Path to model.urdf")
    ap.add_argument("--output", type=Path, default=Path(__file__).parent,
                    help="Directory to write <record_id>/ into (default: plots/)")
    ap.add_argument("--samples", type=int, default=128, help="Cycles samples (default 128)")
    ap.add_argument("--resolution", default="1080x1080", help="WxH (default 1080x1080)")
    ap.add_argument("--hdri", default=DEFAULT_HDRI, help="Path to .exr/.hdr environment map")
    ap.add_argument("--blender", default=BLENDER, help="Blender executable path")
    ap.add_argument("--modes", default=None,
                    help="Comma-separated modes to render (default: all). E.g. motion_1,motion_30")
    ap.add_argument("--include-joint-overlay", action="store_true",
                    help="Append the separate joint_overlay articulation render to the selected modes")
    ap.add_argument("--no-joint-overlay", action="store_true",
                    help="Skip the amber joint overlay baked into motion renders")
    args = ap.parse_args()

    urdf = args.urdf.resolve()
    if not urdf.exists():
        print(f"Error: URDF not found: {urdf}", file=sys.stderr)
        sys.exit(1)

    record_id = resolve_record_id(urdf)
    out_dir = args.output.resolve() / record_id
    out_dir.mkdir(parents=True, exist_ok=True)

    active_modes = [m.strip() for m in args.modes.split(",")] if args.modes else list(ALL_MODES)
    if args.include_joint_overlay and "joint_overlay" not in active_modes:
        active_modes.append("joint_overlay")

    cmd = [
        args.blender, "--background", "--factory-startup",
        "--python", str(SCRIPT),
        "--",
        "--urdf", str(urdf),
        "--output", str(out_dir),
        "--samples", str(args.samples),
        "--resolution", args.resolution,
        "--hdri", args.hdri,
        "--modes", ",".join(active_modes),
    ]
    if args.no_joint_overlay:
        cmd.append("--no-joint-overlay")

    print(f"Record:  {record_id}")
    print(f"Output:  {out_dir}")
    print(f"Modes:   {', '.join(active_modes)}")
    print(f"Running: {' '.join(cmd)}\n")

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"\nBlender exited with code {result.returncode}", file=sys.stderr)
        sys.exit(result.returncode)

    print(f"\nDone. Individual renders in: {out_dir}/angle_*/")

    # If render_urdf_viz emitted joint_overlay__bg.png + joint_overlay__fg.png
    # (always-in-front mode), alpha-composite them into the final overlay PNG.
    try:
        from PIL import Image as _PILImage
        for angle_dir in sorted(out_dir.glob("angle_*")):
            bg = angle_dir / "joint_overlay__bg.png"
            fg = angle_dir / "joint_overlay__fg.png"
            if bg.exists() and fg.exists():
                bg_img = _PILImage.open(bg).convert("RGBA")
                fg_img = _PILImage.open(fg).convert("RGBA")
                _PILImage.alpha_composite(bg_img, fg_img).save(
                    angle_dir / "joint_overlay.png", "PNG")
                bg.unlink()
                fg.unlink()
                print(f"  Composited always-in-front overlay → {angle_dir.name}/joint_overlay.png")
    except ImportError:
        print("  Pillow missing, cannot composite always-in-front overlay")

    composite_paths = []
    for angle_dir in sorted(out_dir.glob("angle_*")):
        if not angle_dir.is_dir():
            continue
        composite_path = angle_dir / "composite.png"
        if make_composite(angle_dir, composite_path, active_modes):
            composite_paths.append(composite_path)

    contact_sheet_path = out_dir / "composite.png"
    make_contact_sheet(composite_paths, contact_sheet_path)

    print(f"\nOutputs:")
    for img in sorted(out_dir.glob("angle_*/*.png")):
        print(f"  {img.relative_to(args.output.resolve())}")
    if contact_sheet_path.exists():
        print(f"  {contact_sheet_path.relative_to(args.output.resolve())}")
    legend = out_dir / "legend.json"
    if legend.exists():
        print(f"  {legend.relative_to(args.output.resolve())}")


if __name__ == "__main__":
    main()
