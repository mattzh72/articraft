#!/usr/bin/env bash
# Standalone object renderer: compile-if-needed → 8 angles × 3 modes.
#
# Usage:
#   bash render_object.sh <record_id> [--output <dir>] [--samples N] [--resolution WxH]
#                                     [--angles N] [--elevation D] [--modes csv] [--high]
#
# Examples:
#   bash render_object.sh rec_branching_tree_with_three_independent_rotary_branches_5587a5...
#   bash render_object.sh rec_xxx --output /tmp/myrender --high
#   bash render_object.sh rec_xxx --modes motion_1,motion_half,part_segmentation,collision
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <record_id> [--output dir] [--samples N] [--resolution WxH] [--angles N] [--elevation D] [--modes csv] [--high]" >&2
    exit 1
fi

REC="$1"; shift

# Defaults — match the picker_renders_8angle preset.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
OUTPUT="$HERE/output"
SAMPLES=64
RESOLUTION="1080x1080"
ANGLES=8
ELEVATION=35
MARGIN=1.25
MODES="motion_1,motion_half,part_segmentation"

while [ $# -gt 0 ]; do
    case "$1" in
        --output)     OUTPUT="$2"; shift 2 ;;
        --samples)    SAMPLES="$2"; shift 2 ;;
        --resolution) RESOLUTION="$2"; shift 2 ;;
        --angles)     ANGLES="$2"; shift 2 ;;
        --elevation)  ELEVATION="$2"; shift 2 ;;
        --margin)     MARGIN="$2"; shift 2 ;;
        --modes)      MODES="$2"; shift 2 ;;
        --high)       SAMPLES=256; RESOLUTION="1440x1440"; shift ;;
        --repo)       REPO="$2"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 1 ;;
    esac
done

URDF="$REPO/data/cache/record_materialization/$REC/model.urdf"
RECORD_DIR="$REPO/data/records/$REC"

if [ ! -f "$URDF" ]; then
    if [ ! -d "$RECORD_DIR" ]; then
        echo "error: no record '$REC' under $REPO/data/records/" >&2
        exit 1
    fi
    echo "[render_object] compiling $REC ..."
    (cd "$REPO" && just compile "data/records/$REC")
fi

if [ ! -f "$URDF" ]; then
    echo "error: compile finished but $URDF still missing" >&2
    exit 1
fi

mkdir -p "$OUTPUT"
echo "[render_object] $REC → $OUTPUT/$REC/"
echo "[render_object] angles=$ANGLES elevation=$ELEVATION samples=$SAMPLES res=$RESOLUTION modes=$MODES"

CAM_NUM_ANGLES="$ANGLES" CAM_MARGIN="$MARGIN" CAM_ELEVATION_DEG="$ELEVATION" \
    python "$HERE/visualize.py" \
        --urdf "$URDF" \
        --output "$OUTPUT" \
        --modes "$MODES" \
        --samples "$SAMPLES" \
        --resolution "$RESOLUTION" \
        --no-joint-overlay
