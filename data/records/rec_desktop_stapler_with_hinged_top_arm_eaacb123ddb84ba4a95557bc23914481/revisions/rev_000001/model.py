from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.155
BASE_WIDTH = 0.043
BASE_THICKNESS = 0.004
HINGE_X = -0.060
HINGE_Z = 0.026

ARM_WIDTH = 0.038
ARM_CAVITY_WIDTH = 0.034
ARM_LENGTH = 0.146
ARM_BARREL_RADIUS = 0.0052
ARM_BARREL_LENGTH = 0.020

TRAY_LENGTH = 0.106
TRAY_WIDTH = 0.0256
TRAY_TRAVEL = 0.028
TRAY_FLOOR_BOTTOM = -0.0046


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )

    front_anvil = (
        cq.Workplane("XY")
        .box(0.016, 0.018, 0.003)
        .translate((0.060, 0.0, 0.0055))
    )

    rear_block = (
        cq.Workplane("XY")
        .box(0.032, 0.032, 0.008)
        .translate((HINGE_X - 0.002, 0.0, 0.008))
    )

    left_ear = (
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.019)
        .translate((HINGE_X - 0.002, -0.0155, 0.0135))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.010, 0.006, 0.019)
        .translate((HINGE_X - 0.002, 0.0155, 0.0135))
    )

    left_knuckle = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(ARM_BARREL_RADIUS)
        .extrude(0.006)
        .translate((0.0, -0.0185, 0.0))
    )
    right_knuckle = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(ARM_BARREL_RADIUS)
        .extrude(0.006)
        .translate((0.0, 0.0125, 0.0))
    )

    nose_ramp = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.042, 0.004),
                (0.077, 0.004),
                (0.077, 0.009),
                (0.055, 0.007),
                (0.042, 0.005),
            ]
        )
        .close()
        .extrude(0.012, both=True)
    )

    return (
        plate.union(front_anvil)
        .union(rear_block)
        .union(left_ear)
        .union(right_ear)
        .union(left_knuckle)
        .union(right_knuckle)
        .union(nose_ramp)
    )


def _arm_shell_shape() -> cq.Workplane:
    outer_profile = [
        (0.014, -0.0045),
        (0.026, 0.0050),
        (0.055, 0.0120),
        (0.096, 0.0130),
        (0.128, 0.0085),
        (ARM_LENGTH, 0.0045),
        (ARM_LENGTH, -0.0015),
        (0.118, -0.0035),
        (0.040, -0.0055),
    ]
    inner_profile = [
        (0.022, -0.0052),
        (0.036, 0.0012),
        (0.078, 0.0076),
        (0.122, 0.0048),
        (0.132, 0.0012),
        (0.132, -0.0052),
    ]

    outer = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(ARM_WIDTH / 2.0, both=True)
    )
    inner = (
        cq.Workplane("XZ")
        .polyline(inner_profile)
        .close()
        .extrude(ARM_CAVITY_WIDTH / 2.0, both=True)
    )

    rear_web = (
        cq.Workplane("XY")
        .box(0.018, 0.020, 0.010)
        .translate((0.015, 0.0, -0.0005))
    )

    left_rail = (
        cq.Workplane("XY")
        .box(0.098, 0.004, 0.006)
        .translate((0.079, -0.0148, 0.0006))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(0.098, 0.004, 0.006)
        .translate((0.079, 0.0148, 0.0006))
    )

    front_nose = (
        cq.Workplane("XY")
        .box(0.010, ARM_WIDTH - 0.006, 0.006)
        .translate((ARM_LENGTH - 0.006, 0.0, 0.0015))
    )

    return outer.cut(inner).union(rear_web).union(left_rail).union(right_rail).union(front_nose)


def _tray_shape() -> cq.Workplane:
    floor = (
        cq.Workplane("XY")
        .box(TRAY_LENGTH, 0.020, 0.0016)
        .translate((TRAY_LENGTH / 2.0, 0.0, TRAY_FLOOR_BOTTOM + 0.0008))
    )

    left_flange = (
        cq.Workplane("XY")
        .box(TRAY_LENGTH - 0.010, 0.0018, 0.0050)
        .translate((TRAY_LENGTH / 2.0 - 0.002, -0.0119, TRAY_FLOOR_BOTTOM + 0.0025))
    )
    right_flange = (
        cq.Workplane("XY")
        .box(TRAY_LENGTH - 0.010, 0.0018, 0.0050)
        .translate((TRAY_LENGTH / 2.0 - 0.002, 0.0119, TRAY_FLOOR_BOTTOM + 0.0025))
    )

    front_stop = (
        cq.Workplane("XY")
        .box(0.006, TRAY_WIDTH, 0.0038)
        .translate((TRAY_LENGTH - 0.003, 0.0, TRAY_FLOOR_BOTTOM + 0.0019))
    )

    rear_bridge = (
        cq.Workplane("XY")
        .box(0.012, TRAY_WIDTH, 0.0034)
        .translate((0.006, 0.0, TRAY_FLOOR_BOTTOM + 0.0017))
    )

    left_runner = (
        cq.Workplane("XY")
        .box(0.088, 0.0016, 0.0008)
        .translate((TRAY_LENGTH / 2.0 - 0.004, -0.0114, 0.0007))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(0.088, 0.0016, 0.0008)
        .translate((TRAY_LENGTH / 2.0 - 0.004, 0.0114, 0.0007))
    )

    return (
        floor.union(left_flange)
        .union(right_flange)
        .union(front_stop)
        .union(rear_bridge)
        .union(left_runner)
        .union(right_runner)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    base_metal = model.material("base_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.82, 0.83, 0.85, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base_shell"), material=base_metal, name="base_shell")

    top_arm = model.part("top_arm")
    top_arm.visual(
        mesh_from_cadquery(_arm_shell_shape(), "arm_shell"),
        material=arm_finish,
        name="arm_shell",
    )
    for y_pos, name in ((-0.0135, "hinge_cheek_0"), (0.0135, "hinge_cheek_1")):
        top_arm.visual(
            Box((0.010, 0.004, 0.008)),
            origin=Origin(xyz=(0.004, y_pos, 0.001)),
            material=arm_finish,
            name=name,
        )
    for y_pos, name in ((-0.01125, "hinge_bridge_0"), (0.01125, "hinge_bridge_1")):
        top_arm.visual(
            Box((0.010, 0.0045, 0.004)),
            origin=Origin(xyz=(0.007, y_pos, 0.0035)),
            material=arm_finish,
            name=name,
        )

    staple_tray = model.part("staple_tray")
    staple_tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray_body"),
        material=tray_metal,
        name="tray_body",
    )

    model.articulation(
        "base_to_top_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top_arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(55.0),
        ),
    )

    model.articulation(
        "top_arm_to_staple_tray",
        ArticulationType.PRISMATIC,
        parent=top_arm,
        child=staple_tray,
        origin=Origin(xyz=(0.024, 0.0, TRAY_FLOOR_BOTTOM - 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.12,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    top_arm = object_model.get_part("top_arm")
    staple_tray = object_model.get_part("staple_tray")
    arm_hinge = object_model.get_articulation("base_to_top_arm")
    tray_slide = object_model.get_articulation("top_arm_to_staple_tray")

    arm_limits = arm_hinge.motion_limits
    tray_limits = tray_slide.motion_limits
    hinge_upper = arm_limits.upper if arm_limits is not None and arm_limits.upper is not None else 0.0
    tray_upper = tray_limits.upper if tray_limits is not None and tray_limits.upper is not None else 0.0

    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_overlap(
            top_arm,
            base,
            axes="xy",
            elem_a="arm_shell",
            elem_b="base_shell",
            min_overlap=0.028,
            name="closed arm covers the low-profile base",
        )
        ctx.expect_within(
            staple_tray,
            top_arm,
            axes="y",
            inner_elem="tray_body",
            outer_elem="arm_shell",
            margin=0.003,
            name="tray body stays centered between the arm side guides at rest",
        )
        ctx.expect_gap(
            top_arm,
            staple_tray,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="tray_body",
            max_gap=0.012,
            max_penetration=1e-5,
            name="tray rides just below the upper shell at rest",
        )
        ctx.expect_overlap(
            staple_tray,
            top_arm,
            axes="x",
            elem_a="tray_body",
            elem_b="arm_shell",
            min_overlap=0.100,
            name="tray remains deeply inserted at rest",
        )

    tray_rest = ctx.part_world_position(staple_tray)
    with ctx.pose({tray_slide: tray_upper}):
        ctx.expect_within(
            staple_tray,
            top_arm,
            axes="y",
            inner_elem="tray_body",
            outer_elem="arm_shell",
            margin=0.003,
            name="tray body stays guided laterally when extended",
        )
        ctx.expect_gap(
            top_arm,
            staple_tray,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="tray_body",
            max_gap=0.012,
            max_penetration=1e-5,
            name="extended tray still runs beneath the upper shell",
        )
        ctx.expect_overlap(
            staple_tray,
            top_arm,
            axes="x",
            elem_a="tray_body",
            elem_b="arm_shell",
            min_overlap=0.072,
            name="extended tray still retains insertion in the upper body",
        )
        tray_extended = ctx.part_world_position(staple_tray)

    ctx.check(
        "tray slides rearward along the arm",
        tray_rest is not None and tray_extended is not None and tray_extended[0] < tray_rest[0] - 0.02,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    with ctx.pose({arm_hinge: 0.0}):
        arm_closed_aabb = ctx.part_world_aabb(top_arm)
    with ctx.pose({arm_hinge: hinge_upper}):
        arm_open_aabb = ctx.part_world_aabb(top_arm)

    ctx.check(
        "top arm opens upward at the rear hinge",
        arm_closed_aabb is not None
        and arm_open_aabb is not None
        and arm_open_aabb[1][2] > arm_closed_aabb[1][2] + 0.045,
        details=f"closed={arm_closed_aabb}, open={arm_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
