from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.168
BASE_WIDTH = 0.046
BASE_THICKNESS = 0.011

HINGE_X = -0.060
HINGE_Z = 0.026

ARM_OPEN_ANGLE = 1.00
TRAY_TRAVEL = 0.048


def _build_arm_shell_shape() -> object:
    outer_shell = (
        cq.Workplane("YZ")
        .workplane(offset=0.014)
        .center(0.0, 0.012)
        .rect(0.034, 0.024)
        .workplane(offset=0.136)
        .center(0.0, 0.003)
        .rect(0.024, 0.018)
        .loft(combine=True)
    )

    inner_void = (
        cq.Workplane("YZ")
        .workplane(offset=0.020)
        .center(0.0, 0.011)
        .rect(0.026, 0.018)
        .workplane(offset=0.124)
        .center(0.0, 0.004)
        .rect(0.018, 0.012)
        .loft(combine=True)
    )

    bottom_opening = cq.Workplane("XY").box(0.150, 0.024, 0.014).translate((0.078, 0.0, -0.005))
    rear_opening = cq.Workplane("XY").box(0.030, 0.022, 0.018).translate((0.004, 0.0, 0.001))
    hinge_bridge = cq.Workplane("XY").box(0.014, 0.018, 0.007).translate((0.009, 0.0, 0.0075))

    return outer_shell.cut(inner_void).cut(bottom_opening).cut(rear_opening).union(hinge_bridge)


def _build_tray_shape() -> object:
    tray_plate = cq.Workplane("XY").box(0.132, 0.016, 0.0022).translate((0.061, 0.0, -0.0024))
    left_rail = cq.Workplane("XY").box(0.118, 0.0016, 0.0048).translate((0.066, -0.0072, 0.0008))
    right_rail = cq.Workplane("XY").box(0.118, 0.0016, 0.0048).translate((0.066, 0.0072, 0.0008))
    front_nose = cq.Workplane("XY").box(0.014, 0.012, 0.0058).translate((0.126, 0.0, 0.0008))
    rear_pull = cq.Workplane("XY").box(0.012, 0.018, 0.0075).translate((-0.006, 0.0, 0.0008))

    return tray_plate.union(left_rail).union(right_rail).union(front_nose).union(rear_pull)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_stapler")

    body_graphite = model.material("body_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    tray_steel = model.material("tray_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    anvil_steel = model.material("anvil_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    pad_black = model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=body_graphite,
        name="base_body",
    )
    base.visual(
        Box((0.052, 0.034, 0.006)),
        origin=Origin(xyz=(0.035, 0.0, 0.014)),
        material=body_graphite,
        name="nose_ramp",
    )
    base.visual(
        Box((0.028, 0.020, 0.003)),
        origin=Origin(xyz=(0.060, 0.0, 0.0185)),
        material=anvil_steel,
        name="anvil",
    )
    base.visual(
        Box((0.024, 0.022, 0.007)),
        origin=Origin(xyz=(-0.064, 0.0, 0.0145)),
        material=body_graphite,
        name="rear_bridge",
    )
    for index, side_y in enumerate((-0.018, 0.018)):
        base.visual(
            Box((0.012, 0.010, 0.020)),
            origin=Origin(xyz=(HINGE_X, side_y, 0.021)),
            material=body_graphite,
            name=f"support_{index}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(
                xyz=(HINGE_X, side_y, HINGE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_steel,
            name=f"cap_{index}",
        )
    base.visual(
        Box((0.132, 0.028, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.012)),
        material=pad_black,
        name="base_pad",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_build_arm_shell_shape(), "stapler_arm_shell"),
        material=body_graphite,
        name="arm_shell",
    )
    arm.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="arm_barrel",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_shape(), "stapler_tray"),
        material=tray_steel,
        name="tray_body",
    )

    arm_hinge = model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=2.2,
            lower=0.0,
            upper=ARM_OPEN_ANGLE,
        ),
    )
    tray_slide = model.articulation(
        "arm_to_tray",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tray,
        origin=Origin(xyz=(0.014, 0.0, -0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    arm_hinge.meta["role"] = "primary_opening_motion"
    tray_slide.meta["role"] = "staple_reload_slide"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    arm_hinge = object_model.get_articulation("base_to_arm")
    tray_slide = object_model.get_articulation("arm_to_tray")

    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="anvil",
            min_gap=0.001,
            max_gap=0.020,
            name="closed arm sits just above the anvil",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="xy",
            elem_a="arm_shell",
            elem_b="base_body",
            min_overlap=0.020,
            name="closed arm covers the base footprint",
        )
        ctx.expect_within(
            tray,
            arm,
            axes="y",
            inner_elem="tray_body",
            outer_elem="arm_shell",
            margin=0.0025,
            name="closed tray stays guided under the shell",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            elem_a="tray_body",
            elem_b="arm_shell",
            min_overlap=0.090,
            name="closed tray remains deeply inserted",
        )

    closed_aabb = None
    opened_aabb = None
    with ctx.pose({arm_hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(arm)
    with ctx.pose({arm_hinge: ARM_OPEN_ANGLE}):
        opened_aabb = ctx.part_world_aabb(arm)
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="base_body",
            min_gap=0.010,
            name="opened arm lifts clear of the base",
        )
    ctx.check(
        "arm opens upward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    closed_tray_pos = None
    extended_tray_pos = None
    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({arm_hinge: 0.0, tray_slide: TRAY_TRAVEL}):
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            arm,
            axes="y",
            inner_elem="tray_body",
            outer_elem="arm_shell",
            margin=0.0025,
            name="extended tray stays centered in the shell",
        )
        ctx.expect_overlap(
            tray,
            arm,
            axes="x",
            elem_a="tray_body",
            elem_b="arm_shell",
            min_overlap=0.050,
            name="extended tray keeps retained insertion",
        )
    ctx.check(
        "tray pulls rearward for reload",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] < closed_tray_pos[0] - 0.030,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
