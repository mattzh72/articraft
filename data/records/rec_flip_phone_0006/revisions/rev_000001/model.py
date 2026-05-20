from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

_SCRIPT_ORIGIN = (
    Path(__spec__.origin)
    if "__spec__" in globals() and __spec__ is not None and __spec__.origin
    else Path(__file__)
)
ASSETS = AssetContext(root=_SCRIPT_ORIGIN.parent)

LOWER_W = 0.064
LOWER_H = 0.064
LOWER_T = 0.018

UPPER_W = 0.050
UPPER_H = 0.056
UPPER_T = 0.015

INNER_GAP = 0.002

HINGE_R = 0.0032
LOWER_KNUCKLE_LEN = 0.019
UPPER_KNUCKLE_LEN = 0.022
LOWER_KNUCKLE_X = 0.0205
HINGE_Y = LOWER_H / 2.0 + 0.0015
HINGE_Z = 0.0045

UPPER_BODY_CENTER = (0.0, -0.026, LOWER_T / 2.0 + INNER_GAP + UPPER_T / 2.0 - HINGE_Z)
DISPLAY_CENTER = (
    0.0,
    -0.026,
    UPPER_BODY_CENTER[2] - UPPER_T / 2.0 + 0.0006,
)
EARPIECE_CENTER = (
    0.0,
    -0.0045,
    UPPER_BODY_CENTER[2] - UPPER_T / 2.0 + 0.0007,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_phone", assets=ASSETS)

    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.18, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.29, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.08, 0.14, 0.18, 1.0))
    silver = model.material("silver", rgba=(0.68, 0.70, 0.72, 1.0))

    lower = model.part("lower_body")
    lower.visual(
        Box((LOWER_W, LOWER_H, LOWER_T)),
        origin=Origin(),
        material=dark_plastic,
        name="body_shell",
    )
    lower.visual(
        Box((0.048, 0.020, 0.0016)),
        origin=Origin(xyz=(0.0, -0.013, LOWER_T / 2.0 + 0.0008)),
        material=graphite,
        name="keypad_cluster",
    )
    lower.visual(
        Cylinder(radius=0.010, length=0.0018),
        origin=Origin(xyz=(0.0, 0.011, LOWER_T / 2.0 + 0.0009)),
        material=silver,
        name="navigation_pad",
    )
    lower.visual(
        Box((0.012, 0.003, 0.0016)),
        origin=Origin(xyz=(0.0, -0.026, LOWER_T / 2.0 + 0.0008)),
        material=charcoal,
        name="microphone_slot",
    )
    lower.visual(
        Cylinder(radius=HINGE_R, length=LOWER_KNUCKLE_LEN),
        origin=Origin(
            xyz=(-LOWER_KNUCKLE_X, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="left_lower_knuckle",
    )
    lower.visual(
        Cylinder(radius=HINGE_R, length=LOWER_KNUCKLE_LEN),
        origin=Origin(
            xyz=(LOWER_KNUCKLE_X, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="right_lower_knuckle",
    )
    lower.inertial = Inertial.from_geometry(
        Box((LOWER_W, LOWER_H, LOWER_T)),
        mass=0.18,
        origin=Origin(),
    )

    upper = model.part("upper_body")
    upper.visual(
        Box((UPPER_W, UPPER_H, UPPER_T)),
        origin=Origin(xyz=UPPER_BODY_CENTER),
        material=dark_plastic,
        name="body_shell",
    )
    upper.visual(
        Box((0.038, 0.030, 0.0012)),
        origin=Origin(xyz=DISPLAY_CENTER),
        material=screen_glass,
        name="display_window",
    )
    upper.visual(
        Box((0.014, 0.003, 0.0014)),
        origin=Origin(xyz=EARPIECE_CENTER),
        material=charcoal,
        name="earpiece_slot",
    )
    upper.visual(
        Cylinder(radius=HINGE_R, length=UPPER_KNUCKLE_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="center_upper_knuckle",
    )
    upper.visual(
        Box((0.016, 0.008, 0.0038)),
        origin=Origin(xyz=(0.0, 0.003, 0.00485)),
        material=graphite,
        name="hinge_bridge",
    )
    upper.inertial = Inertial.from_geometry(
        Box((UPPER_W, UPPER_H, UPPER_T)),
        mass=0.12,
        origin=Origin(xyz=UPPER_BODY_CENTER),
    )

    antenna = model.part("antenna_stub")
    antenna.visual(
        Cylinder(radius=0.0028, length=0.022),
        origin=Origin(
            xyz=(0.0, 0.0, 0.011),
            rpy=(-0.25, 0.0, 0.0),
        ),
        material=graphite,
        name="stub",
    )
    antenna.visual(
        Cylinder(radius=0.0019, length=0.008),
        origin=Origin(
            xyz=(0.0, 0.0025, 0.0225),
            rpy=(-0.25, 0.0, 0.0),
        ),
        material=silver,
        name="stub_tip",
    )
    antenna.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0028, length=0.022),
        mass=0.01,
        origin=Origin(
            xyz=(0.0, 0.0, 0.011),
            rpy=(-0.25, 0.0, 0.0),
        ),
    )

    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-2.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "lower_to_antenna",
        ArticulationType.FIXED,
        parent=lower,
        child=antenna,
        origin=Origin(xyz=(-0.029, 0.024, LOWER_T / 2.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=_SCRIPT_ORIGIN.parent)
    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_body")
    antenna = object_model.get_part("antenna_stub")
    hinge = object_model.get_articulation("flip_hinge")
    lower_shell = lower.get_visual("body_shell")
    upper_shell = upper.get_visual("body_shell")
    left_knuckle = lower.get_visual("left_lower_knuckle")
    right_knuckle = lower.get_visual("right_lower_knuckle")
    upper_knuckle = upper.get_visual("center_upper_knuckle")
    antenna_stub = antenna.get_visual("stub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(upper, lower, axes="x", max_dist=0.001)
    ctx.expect_within(
        upper,
        lower,
        axes="x",
        inner_elem=upper_shell,
        outer_elem=lower_shell,
        margin=0.0,
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        min_overlap=0.040,
        elem_a=upper_shell,
        elem_b=lower_shell,
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        min_gap=0.0015,
        max_gap=0.003,
        positive_elem=upper_shell,
        negative_elem=lower_shell,
    )

    ctx.expect_contact(upper, lower, elem_a=upper_knuckle, elem_b=left_knuckle)
    ctx.expect_contact(upper, lower, elem_a=upper_knuckle, elem_b=right_knuckle)
    ctx.expect_overlap(
        upper,
        lower,
        axes="yz",
        min_overlap=0.006,
        elem_a=upper_knuckle,
        elem_b=left_knuckle,
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="yz",
        min_overlap=0.006,
        elem_a=upper_knuckle,
        elem_b=right_knuckle,
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="x",
        min_gap=0.0,
        max_gap=0.0002,
        positive_elem=upper_knuckle,
        negative_elem=left_knuckle,
    )
    ctx.expect_gap(
        lower,
        upper,
        axis="x",
        min_gap=0.0,
        max_gap=0.0002,
        positive_elem=right_knuckle,
        negative_elem=upper_knuckle,
    )

    ctx.expect_contact(antenna, lower, elem_a=antenna_stub, elem_b=lower_shell)
    ctx.expect_origin_distance(antenna, lower, axes="x", min_dist=0.027, max_dist=0.031)
    ctx.expect_origin_gap(antenna, lower, axis="y", min_gap=0.022, max_gap=0.026)
    ctx.expect_gap(
        upper,
        antenna,
        axis="x",
        min_gap=0.001,
        max_gap=0.003,
        positive_elem=upper_shell,
        negative_elem=antenna_stub,
    )

    limits = hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flip_hinge_open_no_overlap")
            ctx.fail_if_isolated_parts(name="flip_hinge_open_no_floating")
            ctx.expect_contact(upper, lower, elem_a=upper_knuckle, elem_b=left_knuckle)
            ctx.expect_contact(upper, lower, elem_a=upper_knuckle, elem_b=right_knuckle)
            ctx.expect_overlap(
                upper,
                lower,
                axes="yz",
                min_overlap=0.006,
                elem_a=upper_knuckle,
                elem_b=left_knuckle,
            )
            ctx.expect_overlap(
                upper,
                lower,
                axes="yz",
                min_overlap=0.006,
                elem_a=upper_knuckle,
                elem_b=right_knuckle,
            )
            ctx.expect_gap(
                upper,
                antenna,
                axis="x",
                min_gap=0.001,
                max_gap=0.003,
                positive_elem=upper_shell,
                negative_elem=antenna_stub,
                name="open_pose_upper_clears_antenna",
            )
        with ctx.pose({hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="flip_hinge_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="flip_hinge_closed_no_floating")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
