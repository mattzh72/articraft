from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


ARM_TIP = (1.42, 0.0, 1.65)


def _build_arm_mesh():
    arm_path = [
        (0.015, 0.0, 0.020),
        (0.090, 0.0, 0.520),
        (0.340, 0.0, 1.240),
        (0.920, 0.0, 1.730),
        (1.340, 0.0, 1.700),
        (1.385, 0.0, 1.650),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            arm_path,
            radius=0.014,
            samples_per_segment=24,
            radial_segments=22,
            cap_ends=True,
        ),
        "arc_arm_tube",
    )


def _build_shade_shell_mesh():
    outer_profile = [
        (0.079, 0.000),
        (0.084, 0.045),
        (0.086, 0.240),
    ]
    inner_profile = [
        (0.072, 0.010),
        (0.077, 0.045),
        (0.079, 0.240),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "cylindrical_shade_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    marble = model.material("marble", rgba=(0.90, 0.90, 0.88, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.69, 0.70, 0.73, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    switch_black = model.material("switch_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.42, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=marble,
        name="marble_block",
    )
    base.visual(
        Box((0.16, 0.10, 0.012)),
        origin=Origin(xyz=(-0.11, 0.0, 0.086)),
        material=graphite,
        name="hinge_plate",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.088),
        origin=Origin(xyz=(-0.11, 0.0, 0.136)),
        material=satin_steel,
        name="hinge_post",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(-0.11, 0.0, 0.174)),
        material=graphite,
        name="hinge_cap",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="swivel_hub",
    )
    arm.visual(
        Box((0.085, 0.060, 0.050)),
        origin=Origin(xyz=(0.0425, 0.0, 0.025)),
        material=graphite,
        name="hub_shoulder",
    )
    arm.visual(
        _build_arm_mesh(),
        material=satin_steel,
        name="arc_tube",
    )
    arm.visual(
        Box((0.035, 0.050, 0.055)),
        origin=Origin(xyz=(1.4025, 0.0, 1.650)),
        material=graphite,
        name="tip_block",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="neck",
    )
    shade.visual(
        Box((0.024, 0.028, 0.008)),
        origin=Origin(xyz=(0.018, 0.0, 0.019)),
        material=graphite,
        name="switch_mount",
    )
    shade.visual(
        _build_shade_shell_mesh(),
        origin=Origin(xyz=(0.045, 0.0, -0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="shade_shell",
    )

    switch = model.part("switch")
    switch.visual(
        Cylinder(radius=0.003, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_black,
        name="pin",
    )
    switch.visual(
        Box((0.008, 0.016, 0.026)),
        origin=Origin(xyz=(0.001, 0.0, 0.013)),
        material=switch_black,
        name="toggle",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.11, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.8,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=ARM_TIP),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.80,
        ),
    )
    model.articulation(
        "shade_to_switch",
        ArticulationType.REVOLUTE,
        parent=shade,
        child=switch,
        origin=Origin(xyz=(0.018, 0.0, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.20,
            velocity=3.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shade = object_model.get_part("shade")
    switch = object_model.get_part("switch")
    base_to_arm = object_model.get_articulation("base_to_arm")
    arm_to_shade = object_model.get_articulation("arm_to_shade")
    shade_to_switch = object_model.get_articulation("shade_to_switch")

    ctx.expect_origin_gap(
        shade,
        base,
        axis="z",
        min_gap=1.70,
        name="shade sits high above the marble base",
    )
    ctx.expect_contact(
        switch,
        shade,
        elem_a="pin",
        elem_b="switch_mount",
        contact_tol=0.0005,
        name="rear switch is mounted on the shade neck",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({base_to_arm: base_to_arm.motion_limits.upper}):
        swung_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "curved arm swivels around the base hinge",
        rest_shade_pos is not None
        and swung_shade_pos is not None
        and swung_shade_pos[1] > rest_shade_pos[1] + 1.0,
        details=f"rest={rest_shade_pos!r}, swung={swung_shade_pos!r}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({arm_to_shade: arm_to_shade.motion_limits.upper}):
        tilted_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade tilts nose downward",
        rest_shell_aabb is not None
        and tilted_shell_aabb is not None
        and tilted_shell_aabb[0][2] < rest_shell_aabb[0][2] - 0.05,
        details=f"rest={rest_shell_aabb!r}, tilted={tilted_shell_aabb!r}",
    )

    rest_toggle_aabb = ctx.part_element_world_aabb(switch, elem="toggle")
    with ctx.pose({shade_to_switch: shade_to_switch.motion_limits.upper}):
        switched_toggle_aabb = ctx.part_element_world_aabb(switch, elem="toggle")
    ctx.check(
        "rear switch pivots locally",
        rest_toggle_aabb is not None
        and switched_toggle_aabb is not None
        and switched_toggle_aabb[1][0] > rest_toggle_aabb[1][0] + 0.004,
        details=f"rest={rest_toggle_aabb!r}, switched={switched_toggle_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
