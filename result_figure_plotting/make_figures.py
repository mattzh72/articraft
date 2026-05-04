#!/usr/bin/env python3
"""End-to-end figure pipeline.

Input: a spec JSON (the "picker" format) listing records with chosen camera
angles. For each cell, render motion_1, motion_half, part_segmentation at the
chosen angle, optionally bake the joint overlay (the I_axis_through
always-in-front style — there is only one overlay variant), crop each record's
images with a shared 5%-margin square window, then build a transparent 3-panel
composite (motion_1 clean, motion_half + overlay, part_segmentation).

Usage:
    python make_figures.py <spec.json> --out <out_dir> [options]

Spec JSON (matches the picker UI output):
    {
      "cells": [
        {"slot": 1, "record_id": "rec_...", "angle_idx": 5, "short_name": "..."},
        ...
      ]
    }

Options:
    --no-overlay           skip the overlay render pass
    --samples N            Cycles samples (default 128)
    --resolution WxH       pixel size (default 2160x2160)
    --margin F             min margin fraction per side (default 0.05)
    --repo PATH            articraft repo root (default: ../)
    --skip-render          reuse cached renders if present
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Iterable

from PIL import Image

HERE = Path(__file__).resolve().parent
DEFAULT_REPO = HERE.parent
VISUALIZE = HERE / "visualize.py"

ALPHA_THR = 8


# ---------------------------------------------------------------------------
# spec loading

def load_cells(spec_path: Path) -> list[dict]:
    data = json.loads(spec_path.read_text())
    if "cells" not in data:
        sys.exit(f"spec missing 'cells': {spec_path}")
    return data["cells"]


def slug(short: str | None, fallback: str) -> str:
    s = (short or fallback).lower()
    for ch in (" ", "-", "/"):
        s = s.replace(ch, "_")
    return s


# ---------------------------------------------------------------------------
# rendering

def call_visualize(urdf: Path, out_dir: Path, modes: str, angle: int,
                   samples: int, resolution: str, overlay: bool) -> None:
    cmd = [
        "python", str(VISUALIZE),
        "--urdf", str(urdf),
        "--output", str(out_dir),
        "--modes", modes,
        "--samples", str(samples),
        "--resolution", resolution,
    ]
    if not overlay:
        cmd.append("--no-joint-overlay")
    env = {
        "CAM_NUM_ANGLES": "8",
        "CAM_MARGIN": "1.25",
        "CAM_ELEVATION_DEG": "35",
        "JOINT_OVERLAY_ANGLES": str(angle),
    }
    if overlay:
        env["JOINT_OVERLAY_STYLE"] = "I_axis_through"
    import os
    full_env = {**os.environ, **env}
    subprocess.run(cmd, env=full_env, check=True)


def render_cell(cell: dict, repo: Path, render_root: Path, *,
                samples: int, resolution: str, overlay: bool,
                skip_existing: bool) -> tuple[Path, Path | None]:
    rec = cell["record_id"]
    angle = cell["angle_idx"]
    urdf = repo / "data" / "cache" / "record_materialization" / rec / "model.urdf"
    if not urdf.is_file():
        sys.exit(f"missing URDF: {urdf}")

    no_dir = render_root / "no_overlay" / rec / f"angle_{angle}"
    needed_no = ["motion_1.png", "motion_half.png", "part_segmentation.png"]
    if not (skip_existing and all((no_dir / f).exists() for f in needed_no)):
        print(f"[render] {rec} angle={angle} (no overlay)")
        call_visualize(urdf, render_root / "no_overlay",
                       "motion_1,motion_half,part_segmentation",
                       angle, samples, resolution, overlay=False)

    ov_dir = None
    if overlay:
        ov_dir = render_root / "overlay" / rec / f"angle_{angle}"
        needed_ov = ["motion_1.png", "motion_half.png"]
        if not (skip_existing and all((ov_dir / f).exists() for f in needed_ov)):
            print(f"[render] {rec} angle={angle} (overlay)")
            call_visualize(urdf, render_root / "overlay",
                           "motion_1,motion_half",
                           angle, samples, resolution, overlay=True)
    return no_dir, ov_dir


# ---------------------------------------------------------------------------
# crop + compose

def alpha_bbox(img: Image.Image):
    alpha = img.convert("RGBA").split()[-1]
    mask = alpha.point(lambda v: 255 if v > ALPHA_THR else 0)
    return mask.getbbox()


def square_window(bbox, img_size, margin: float):
    x0, y0, x1, y1 = bbox
    cx = (x0 + x1) / 2
    cy = (y0 + y1) / 2
    obj_max = max(x1 - x0, y1 - y0)
    side = int(round(obj_max / (1.0 - 2 * margin)))
    W, H = img_size
    side = min(side, W, H)
    half = side / 2
    sx = max(0, min(cx - half, W - side))
    sy = max(0, min(cy - half, H - side))
    return int(round(sx)), int(round(sy)), int(round(sx + side)), int(round(sy + side))


def build_record_tile(cell: dict, no_dir: Path, ov_dir: Path | None,
                      out_root: Path, margin: float, with_overlay: bool) -> Path:
    short = cell.get("short_name") or cell["record_id"]
    sub = f"{cell['slot']:02d}_{slug(short, cell['record_id'])}" if "slot" in cell else slug(short, cell["record_id"])
    tile_dir = out_root / sub
    tile_dir.mkdir(parents=True, exist_ok=True)

    motion_1     = Image.open(no_dir / "motion_1.png").convert("RGBA")
    motion_half  = Image.open(no_dir / "motion_half.png").convert("RGBA")
    part_seg     = Image.open(no_dir / "part_segmentation.png").convert("RGBA")
    motion_1_ov  = Image.open(ov_dir / "motion_1.png").convert("RGBA") if (with_overlay and ov_dir) else None
    motion_half_ov = Image.open(ov_dir / "motion_half.png").convert("RGBA") if (with_overlay and ov_dir) else None

    images = [motion_1, motion_half, part_seg]
    if motion_1_ov: images.append(motion_1_ov)
    if motion_half_ov: images.append(motion_half_ov)

    boxes = [alpha_bbox(im) for im in images]
    boxes = [b for b in boxes if b]
    bbox = (min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes))
    win = square_window(bbox, motion_1.size, margin)

    crop = lambda im: im.crop(win)
    crop(motion_1).save(tile_dir / "motion_1.png", "PNG")
    crop(motion_half).save(tile_dir / "motion_half.png", "PNG")
    crop(part_seg).save(tile_dir / "part_segmentation.png", "PNG")
    if motion_1_ov:
        crop(motion_1_ov).save(tile_dir / "motion_1_overlay.png", "PNG")
    if motion_half_ov:
        crop(motion_half_ov).save(tile_dir / "motion_half_overlay.png", "PNG")

    # 3-panel composite: motion_1 (clean), motion_half + overlay, part_seg
    middle = motion_half_ov if motion_half_ov else motion_half
    panels = [crop(motion_1), crop(middle), crop(part_seg)]
    W, H = panels[0].size
    canvas = Image.new("RGBA", (W * 3, H), (0, 0, 0, 0))
    for col, img in enumerate(panels):
        canvas.paste(img, (col * W, 0), mask=img)
    composite_path = tile_dir / "composite.png"
    canvas.save(composite_path, "PNG")
    print(f"[tile] {sub}  win={win}  side={win[2]-win[0]}px")
    return composite_path


def build_contact_sheet(out_root: Path) -> None:
    composites = sorted(p for p in out_root.glob("*/composite.png"))
    if not composites:
        return
    rows = [Image.open(p).convert("RGBA") for p in composites]
    row_w = max(im.size[0] for im in rows)
    row_h = max(im.size[1] for im in rows)
    sheet = Image.new("RGBA", (row_w, row_h * len(rows)), (0, 0, 0, 0))
    y = 0
    for im in rows:
        sheet.paste(im, ((row_w - im.size[0]) // 2, y), mask=im)
        y += row_h
    sheet.save(out_root / "all_records_contact_sheet.png", "PNG")
    print(f"[contact-sheet] {out_root / 'all_records_contact_sheet.png'}")


# ---------------------------------------------------------------------------
# entry point

def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("spec", type=Path, help="picker JSON spec")
    ap.add_argument("--out", type=Path, required=True, help="output directory")
    ap.add_argument("--repo", type=Path, default=DEFAULT_REPO, help="articraft repo root")
    ap.add_argument("--no-overlay", action="store_true", help="skip overlay render pass")
    ap.add_argument("--samples", type=int, default=128)
    ap.add_argument("--resolution", default="2160x2160")
    ap.add_argument("--margin", type=float, default=0.05)
    ap.add_argument("--skip-render", action="store_true",
                    help="reuse cached renders under <out>/_renders if present")
    args = ap.parse_args(argv)

    cells = load_cells(args.spec)
    args.out.mkdir(parents=True, exist_ok=True)
    render_root = args.out / "_renders"
    print(f"spec     : {args.spec}")
    print(f"out      : {args.out}")
    print(f"renders  : {render_root}")
    print(f"overlay  : {'off' if args.no_overlay else 'on (I_axis_through)'}")
    print(f"quality  : {args.samples} samples, {args.resolution}")
    print(f"records  : {len(cells)}")

    for i, cell in enumerate(cells, 1):
        print(f"--- {i}/{len(cells)}  {cell['record_id']}  angle={cell['angle_idx']} ---")
        no_dir, ov_dir = render_cell(
            cell, args.repo, render_root,
            samples=args.samples, resolution=args.resolution,
            overlay=not args.no_overlay, skip_existing=args.skip_render,
        )
        build_record_tile(cell, no_dir, ov_dir, args.out,
                          margin=args.margin, with_overlay=not args.no_overlay)

    build_contact_sheet(args.out)


if __name__ == "__main__":
    main()
