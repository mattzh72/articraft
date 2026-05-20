from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/tmp"


os.getcwd = _safe_getcwd
try:
    os.chdir("/tmp")
except OSError:
    pass

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

ASSETS = AssetContext(Path("/tmp"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_three_wheel_scooter", assets=ASSETS)

    frame_silver = model.material("frame_silver", rgba=(0.66, 0.69, 0.73, 1.0))
    deck_black = model.material("deck_black", rgba=(0.14, 0.15, 0.16, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_black = model.material("grip_black", rgba=(0.11, 0.11, 0.12, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.93, 0.48, 0.16, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.77, 0.79, 0.82, 1.0))

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.014, length=0.236),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="hinge_pin",
    )
    front_assembly.visual(
        Box((0.030, 0.012, 0.220)),
        origin=Origin(xyz=(0.018, 0.055, 0.110)),
        material=frame_silver,
        name="left_hinge_cheek",
    )
    front_assembly.visual(
        Box((0.030, 0.012, 0.220)),
        origin=Origin(xyz=(0.018, -0.055, 0.110)),
        material=frame_silver,
        name="right_hinge_cheek",
    )
    front_assembly.visual(
        Box((0.018, 0.026, 0.120)),
        origin=Origin(xyz=(0.058, 0.112, 0.160)),
        material=frame_silver,
        name="left_fork_leg",
    )
    front_assembly.visual(
        Box((0.018, 0.026, 0.120)),
        origin=Origin(xyz=(0.058, -0.112, 0.160)),
        material=frame_silver,
        name="right_fork_leg",
    )
    front_assembly.visual(
        Box((0.034, 0.080, 0.240)),
        origin=Origin(xyz=(0.048, 0.0, 0.120)),
        material=frame_silver,
        name="center_upright",
    )
    front_assembly.visual(
        Box((0.090, 0.250, 0.030)),
        origin=Origin(xyz=(0.048, 0.0, 0.225)),
        material=frame_silver,
        name="fork_bridge",
    )
    front_assembly.visual(
        Box((0.045, 0.080, 0.055)),
        origin=Origin(xyz=(0.055, 0.0, 0.2675)),
        material=accent_orange,
        name="stem_gusset",
    )
    front_assembly.visual(
        Box((0.070, 0.100, 0.070)),
        origin=Origin(xyz=(0.055, 0.0, 0.305)),
        material=accent_orange,
        name="head_block",
    )
    front_assembly.visual(
        Cylinder(radius=0.028, length=0.360),
        origin=Origin(xyz=(0.055, 0.0, 0.490)),
        material=frame_silver,
        name="outer_sleeve",
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.200, 0.360, 0.680)),
        mass=3.2,
        origin=Origin(xyz=(0.045, 0.0, 0.240)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.023, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=frame_silver,
        name="lower_post",
    )
    handlebar.visual(
        Box((0.055, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=accent_orange,
        name="bar_clamp",
    )
    handlebar.visual(
        Cylinder(radius=0.015, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="handlebar_crossbar",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.0, 0.258, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.0, -0.258, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.070, 0.540, 0.420)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    deck = model.part("deck")
    deck.visual(
        Cylinder(radius=0.024, length=0.096),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="hinge_barrel",
    )
    deck.visual(
        Box((0.115, 0.120, 0.055)),
        origin=Origin(xyz=(-0.060, 0.0, -0.040)),
        material=accent_orange,
        name="hinge_neck",
    )
    deck.visual(
        Box((0.500, 0.150, 0.026)),
        origin=Origin(xyz=(-0.285, 0.0, -0.060)),
        material=deck_black,
        name="deck_shell",
    )
    deck.visual(
        Box((0.390, 0.115, 0.003)),
        origin=Origin(xyz=(-0.295, 0.0, -0.0455)),
        material=grip_black,
        name="grip_tape",
    )
    deck.visual(
        Box((0.110, 0.018, 0.090)),
        origin=Origin(xyz=(-0.585, 0.041, -0.025)),
        material=frame_silver,
        name="left_rear_arm",
    )
    deck.visual(
        Box((0.110, 0.018, 0.090)),
        origin=Origin(xyz=(-0.585, -0.041, -0.025)),
        material=frame_silver,
        name="right_rear_arm",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.660, 0.170, 0.150)),
        mass=2.3,
        origin=Origin(xyz=(-0.330, 0.0, -0.030)),
    )

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.visual(
        Cylinder(radius=0.095, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    left_front_wheel.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="rim",
    )
    left_front_wheel.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="inner_hub",
    )
    left_front_wheel.inertial = Inertial.from_geometry(
        Box((0.190, 0.052, 0.190)),
        mass=0.6,
        origin=Origin(),
    )

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.visual(
        Cylinder(radius=0.095, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    right_front_wheel.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="rim",
    )
    right_front_wheel.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="inner_hub",
    )
    right_front_wheel.inertial = Inertial.from_geometry(
        Box((0.190, 0.052, 0.190)),
        mass=0.6,
        origin=Origin(),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_hub",
    )
    rear_wheel.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_hub",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Box((0.150, 0.064, 0.150)),
        mass=0.55,
        origin=Origin(),
    )

    model.articulation(
        "deck_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=front_assembly,
        child=deck,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.2,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "handlebar_telescoping",
        ArticulationType.PRISMATIC,
        parent=front_assembly,
        child=handlebar,
        origin=Origin(xyz=(0.055, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.50,
            lower=0.0,
            upper=0.180,
        ),
    )
    model.articulation(
        "left_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, 0.145, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )
    model.articulation(
        "right_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, -0.145, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.620, 0.0, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    front_assembly = object_model.get_part("front_assembly")
    handlebar = object_model.get_part("handlebar")
    deck = object_model.get_part("deck")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    deck_fold_hinge = object_model.get_articulation("deck_fold_hinge")
    handlebar_telescoping = object_model.get_articulation("handlebar_telescoping")
    left_front_wheel_spin = object_model.get_articulation("left_front_wheel_spin")
    right_front_wheel_spin = object_model.get_articulation("right_front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")

    outer_sleeve = front_assembly.get_visual("outer_sleeve")
    hinge_pin = front_assembly.get_visual("hinge_pin")
    lower_post = handlebar.get_visual("lower_post")
    handlebar_crossbar = handlebar.get_visual("handlebar_crossbar")
    hinge_barrel = deck.get_visual("hinge_barrel")
    left_rear_arm = deck.get_visual("left_rear_arm")
    right_rear_arm = deck.get_visual("right_rear_arm")
    left_front_tire = left_front_wheel.get_visual("tire")
    right_front_tire = right_front_wheel.get_visual("tire")
    left_front_hub = left_front_wheel.get_visual("inner_hub")
    right_front_hub = right_front_wheel.get_visual("inner_hub")
    rear_tire = rear_wheel.get_visual("tire")
    rear_left_hub = rear_wheel.get_visual("left_hub")
    rear_right_hub = rear_wheel.get_visual("right_hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        deck,
        front_assembly,
        elem_a=hinge_barrel,
        elem_b=hinge_pin,
        reason="the deck hinge barrel rotates around the fixed hinge pin",
    )
    ctx.allow_overlap(
        handlebar,
        front_assembly,
        elem_a=lower_post,
        elem_b=outer_sleeve,
        reason="the telescoping lower post remains nested inside the outer sleeve",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "deck hinge is a bounded revolute at the front axle",
        deck_fold_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(deck_fold_hinge.axis) == (0.0, 1.0, 0.0),
        "The deck should fold about the front axle's lateral axis.",
    )
    ctx.check(
        "handlebar column telescopes vertically",
        handlebar_telescoping.articulation_type == ArticulationType.PRISMATIC
        and tuple(handlebar_telescoping.axis) == (0.0, 0.0, 1.0),
        "The handlebar should slide upward out of the front sleeve.",
    )
    ctx.check(
        "all three wheels spin on continuous axle joints",
        left_front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_front_wheel_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_front_wheel_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(rear_wheel_spin.axis) == (0.0, 1.0, 0.0),
        "Each wheel should rotate freely on its axle.",
    )
    ctx.expect_contact(
        front_assembly,
        left_front_wheel,
        elem_a=hinge_pin,
        elem_b=left_front_hub,
        contact_tol=0.0005,
        name="left front wheel mounts to the fork",
    )
    ctx.expect_contact(
        front_assembly,
        right_front_wheel,
        elem_a=hinge_pin,
        elem_b=right_front_hub,
        contact_tol=0.0005,
        name="right front wheel mounts to the fork",
    )
    ctx.expect_contact(
        deck,
        rear_wheel,
        elem_a=left_rear_arm,
        elem_b=rear_left_hub,
        contact_tol=0.0005,
        name="rear wheel mounts to the left rear arm",
    )
    ctx.expect_contact(
        deck,
        rear_wheel,
        elem_a=right_rear_arm,
        elem_b=rear_right_hub,
        contact_tol=0.0005,
        name="rear wheel mounts to the right rear arm",
    )
    ctx.expect_gap(
        left_front_wheel,
        right_front_wheel,
        axis="y",
        positive_elem=left_front_tire,
        negative_elem=right_front_tire,
        min_gap=0.250,
        name="front wheels sit on a wide fork",
    )
    ctx.expect_within(
        handlebar,
        front_assembly,
        axes="xy",
        inner_elem=lower_post,
        outer_elem=outer_sleeve,
        name="collapsed handlebar post stays inside the sleeve",
    )
    ctx.expect_gap(
        handlebar,
        front_assembly,
        axis="z",
        positive_elem=handlebar_crossbar,
        negative_elem=outer_sleeve,
        min_gap=0.035,
        name="collapsed handlebar still rises above the sleeve",
    )
    ctx.expect_overlap(
        front_assembly,
        deck,
        axes="yz",
        elem_a=hinge_pin,
        elem_b=hinge_barrel,
        min_overlap=0.028,
        name="hinge pin stays aligned with the folding barrel",
    )
    ctx.expect_gap(
        front_assembly,
        rear_wheel,
        axis="x",
        positive_elem=hinge_pin,
        negative_elem=rear_tire,
        min_gap=0.500,
        name="rear wheel trails well behind the front axle",
    )

    deck_limits = deck_fold_hinge.motion_limits
    if deck_limits is not None and deck_limits.lower is not None and deck_limits.upper is not None:
        with ctx.pose({deck_fold_hinge: deck_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="deck_fold_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="deck_fold_lower_no_floating")
        with ctx.pose({deck_fold_hinge: deck_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="deck_fold_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="deck_fold_upper_no_floating")
            ctx.expect_overlap(
                front_assembly,
                deck,
                axes="yz",
                elem_a=hinge_pin,
                elem_b=hinge_barrel,
                min_overlap=0.028,
                name="hinge pin stays aligned while folded",
            )
            ctx.expect_gap(
                rear_wheel,
                front_assembly,
                axis="z",
                positive_elem=rear_tire,
                negative_elem=hinge_pin,
                min_gap=0.430,
                name="folded deck lifts the rear wheel above the front axle",
            )

    handle_limits = handlebar_telescoping.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handlebar_telescoping: handle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handlebar_lower_no_floating")
        with ctx.pose({handlebar_telescoping: handle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handlebar_upper_no_floating")
            ctx.expect_within(
                handlebar,
                front_assembly,
                axes="xy",
                inner_elem=lower_post,
                outer_elem=outer_sleeve,
                name="extended post stays aligned inside the sleeve",
            )
            ctx.expect_gap(
                handlebar,
                front_assembly,
                axis="z",
                positive_elem=handlebar_crossbar,
                negative_elem=outer_sleeve,
                min_gap=0.210,
                name="telescoped handlebar extends clearly upward",
            )

    with ctx.pose({handlebar_telescoping: 0.180}):
        ctx.expect_gap(
            handlebar,
            front_assembly,
            axis="z",
            positive_elem=handlebar_crossbar,
            negative_elem=outer_sleeve,
            min_gap=0.210,
            name="full extension raises the bar well above the sleeve",
        )

    with ctx.pose({deck_fold_hinge: 1.050}):
        ctx.expect_gap(
            rear_wheel,
            front_assembly,
            axis="z",
            positive_elem=rear_tire,
            negative_elem=hinge_pin,
            min_gap=0.360,
            name="folded deck lifts the rear wheel high above the front axle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
