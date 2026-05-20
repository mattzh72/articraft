from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _deck_outline() -> list[tuple[float, float]]:
    return [
        (-0.250, -0.056),
        (-0.238, -0.066),
        (-0.205, -0.070),
        (0.150, -0.070),
        (0.188, -0.062),
        (0.212, -0.045),
        (0.220, -0.020),
        (0.220, 0.020),
        (0.212, 0.045),
        (0.188, 0.062),
        (0.150, 0.070),
        (-0.205, 0.070),
        (-0.238, 0.066),
        (-0.250, 0.056),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_commuter_scooter", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    red = model.material("signal_red", rgba=(0.78, 0.15, 0.12, 1.0))

    deck = model.part("deck")
    deck_shell_mesh = _save_mesh(
        "scooter_deck_shell.obj",
        ExtrudeGeometry.from_z0(_deck_outline(), 0.014, cap=True, closed=True),
    )
    deck.visual(
        deck_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=charcoal,
        name="deck_shell",
    )
    deck.visual(
        Box((0.46, 0.010, 0.030)),
        origin=Origin(xyz=(-0.010, 0.065, 0.052)),
        material=charcoal,
        name="left_rail",
    )
    deck.visual(
        Box((0.46, 0.010, 0.030)),
        origin=Origin(xyz=(-0.010, -0.065, 0.052)),
        material=charcoal,
        name="right_rail",
    )
    deck.visual(
        Box((0.40, 0.120, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.034)),
        material=matte_black,
        name="battery_tray",
    )
    deck.visual(
        Box((0.34, 0.090, 0.002)),
        origin=Origin(xyz=(-0.030, 0.0, 0.075)),
        material=matte_black,
        name="grip_tape",
    )
    deck.visual(
        Box((0.080, 0.120, 0.014)),
        origin=Origin(xyz=(-0.150, 0.0, 0.082), rpy=(0.0, 0.24, 0.0)),
        material=charcoal,
        name="tail_kick",
    )
    deck.visual(
        Box((0.020, 0.100, 0.100)),
        origin=Origin(xyz=(0.219, 0.0, 0.124)),
        material=charcoal,
        name="head_block",
    )
    deck.visual(
        Box((0.018, 0.010, 0.044)),
        origin=Origin(xyz=(0.205, 0.026, 0.196)),
        material=aluminum,
        name="hinge_bracket_left",
    )
    deck.visual(
        Box((0.018, 0.010, 0.044)),
        origin=Origin(xyz=(0.205, -0.026, 0.196)),
        material=aluminum,
        name="hinge_bracket_right",
    )
    deck.visual(
        Box((0.130, 0.008, 0.180)),
        origin=Origin(xyz=(0.285, 0.025, 0.135)),
        material=aluminum,
        name="front_fork_left",
    )
    deck.visual(
        Box((0.130, 0.008, 0.180)),
        origin=Origin(xyz=(0.285, -0.025, 0.135)),
        material=aluminum,
        name="front_fork_right",
    )
    deck.visual(
        Box((0.018, 0.058, 0.018)),
        origin=Origin(xyz=(0.250, 0.0, 0.195)),
        material=aluminum,
        name="front_fork_crown",
    )
    deck.visual(
        Box((0.160, 0.008, 0.160)),
        origin=Origin(xyz=(-0.310, 0.025, 0.118)),
        material=aluminum,
        name="rear_fork_left",
    )
    deck.visual(
        Box((0.160, 0.008, 0.160)),
        origin=Origin(xyz=(-0.310, -0.025, 0.118)),
        material=aluminum,
        name="rear_fork_right",
    )
    deck.visual(
        Box((0.060, 0.008, 0.022)),
        origin=Origin(xyz=(-0.385, 0.025, 0.206)),
        material=aluminum,
        name="rear_fender_support_left",
    )
    deck.visual(
        Box((0.060, 0.008, 0.022)),
        origin=Origin(xyz=(-0.385, -0.025, 0.206)),
        material=aluminum,
        name="rear_fender_support_right",
    )
    deck.visual(
        Box((0.050, 0.050, 0.010)),
        origin=Origin(xyz=(-0.385, 0.0, 0.217)),
        material=red,
        name="rear_fender",
    )
    deck.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(-0.411, 0.0, 0.217)),
        material=red,
        name="rear_light",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    stem.visual(
        Box((0.054, 0.040, 0.092)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=aluminum,
        name="hinge_collar",
    )
    stem.visual(
        Box((0.026, 0.018, 0.036)),
        origin=Origin(xyz=(0.024, 0.0, 0.086)),
        material=matte_black,
        name="folding_latch",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.406)),
        material=aluminum,
        name="stem_tube",
    )
    stem.visual(
        Box((0.050, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.787)),
        material=aluminum,
        name="bar_clamp",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.818), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, 0.175, 0.818), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, -0.175, 0.818), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    stem.visual(
        Box((0.038, 0.022, 0.064)),
        origin=Origin(xyz=(0.026, 0.0, 0.660), rpy=(0.0, 0.20, 0.0)),
        material=matte_black,
        name="display",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=0.100, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.072, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_hub",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=0.100, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.072, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rear_rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_hub",
    )

    model.articulation(
        "stem_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.205, 0.0, 0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-1.45,
            upper=0.05,
        ),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(0.335, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=30.0,
        ),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.355, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=30.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    stem_hinge = object_model.get_articulation("stem_hinge")
    front_axle = object_model.get_articulation("front_axle")
    rear_axle = object_model.get_articulation("rear_axle")

    deck_shell = deck.get_visual("deck_shell")
    hinge_bracket_left = deck.get_visual("hinge_bracket_left")
    hinge_bracket_right = deck.get_visual("hinge_bracket_right")
    front_fork_left = deck.get_visual("front_fork_left")
    front_fork_right = deck.get_visual("front_fork_right")
    rear_fork_left = deck.get_visual("rear_fork_left")
    rear_fork_right = deck.get_visual("rear_fork_right")
    rear_fender = deck.get_visual("rear_fender")
    grip_tape = deck.get_visual("grip_tape")

    hinge_barrel = stem.get_visual("hinge_barrel")
    handlebar = stem.get_visual("handlebar")
    front_hub = front_wheel.get_visual("front_hub")
    rear_hub = rear_wheel.get_visual("rear_hub")
    front_tire = front_wheel.get_visual("front_tire")
    rear_tire = rear_wheel.get_visual("rear_tire")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    stem_limits = stem_hinge.motion_limits
    ctx.check(
        "stem_hinge_axis_is_lateral",
        tuple(stem_hinge.axis) == (0.0, 1.0, 0.0),
        f"stem hinge axis was {stem_hinge.axis}",
    )
    ctx.check(
        "front_axle_axis_is_lateral",
        tuple(front_axle.axis) == (0.0, 1.0, 0.0),
        f"front axle axis was {front_axle.axis}",
    )
    ctx.check(
        "rear_axle_axis_is_lateral",
        tuple(rear_axle.axis) == (0.0, 1.0, 0.0),
        f"rear axle axis was {rear_axle.axis}",
    )
    ctx.check(
        "stem_hinge_fold_range",
        stem_limits is not None
        and stem_limits.lower is not None
        and stem_limits.upper is not None
        and stem_limits.lower <= -1.40
        and stem_limits.upper >= 0.0,
        f"stem hinge limits were {stem_limits}",
    )
    ctx.check(
        "wheel_joints_are_continuous",
        front_axle.articulation_type == ArticulationType.CONTINUOUS
        and rear_axle.articulation_type == ArticulationType.CONTINUOUS,
        (
            "expected continuous wheel rotation, got "
            f"{front_axle.articulation_type} and {rear_axle.articulation_type}"
        ),
    )

    with ctx.pose({stem_hinge: 0.0, front_axle: 0.0, rear_axle: 0.0}):
        ctx.expect_contact(stem, deck, elem_a=hinge_barrel, elem_b=hinge_bracket_left)
        ctx.expect_contact(stem, deck, elem_a=hinge_barrel, elem_b=hinge_bracket_right)
        ctx.expect_contact(front_wheel, deck, elem_a=front_hub, elem_b=front_fork_left)
        ctx.expect_contact(front_wheel, deck, elem_a=front_hub, elem_b=front_fork_right)
        ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_fork_left)
        ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_fork_right)
        ctx.expect_gap(front_wheel, rear_wheel, axis="x", min_gap=0.42)
        ctx.expect_gap(
            stem,
            deck,
            axis="z",
            positive_elem=handlebar,
            negative_elem=deck_shell,
            min_gap=0.84,
        )
        ctx.expect_gap(
            deck,
            rear_wheel,
            axis="z",
            positive_elem=rear_fender,
            negative_elem=rear_tire,
            min_gap=0.010,
            max_gap=0.030,
        )
        ctx.expect_within(
            deck,
            deck,
            axes="xy",
            inner_elem=grip_tape,
            outer_elem=deck_shell,
            margin=0.0,
        )

    with ctx.pose({front_axle: math.pi / 2.0, rear_axle: -math.pi / 3.0}):
        ctx.expect_contact(front_wheel, deck, elem_a=front_hub, elem_b=front_fork_left)
        ctx.expect_contact(rear_wheel, deck, elem_a=rear_hub, elem_b=rear_fork_right)
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")

    if stem_limits is not None and stem_limits.lower is not None and stem_limits.upper is not None:
        with ctx.pose({stem_hinge: stem_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="stem_folded_no_overlap")
            ctx.fail_if_isolated_parts(name="stem_folded_no_floating")
            ctx.expect_gap(
                stem,
                deck,
                axis="z",
                positive_elem=handlebar,
                negative_elem=deck_shell,
                min_gap=0.020,
                max_gap=0.240,
            )
            ctx.expect_overlap(stem, deck, axes="x", min_overlap=0.18)
        with ctx.pose({stem_hinge: stem_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="stem_upright_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="stem_upright_limit_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
