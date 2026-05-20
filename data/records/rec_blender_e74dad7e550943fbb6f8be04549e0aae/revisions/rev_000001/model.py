from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="milkshake_mixer")

    enamel = model.material("enamel", rgba=(0.88, 0.17, 0.13, 1.0))
    cream = model.material("cream", rgba=(0.93, 0.91, 0.86, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.22, 0.18, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cream,
        name="base_shell",
    )
    body.visual(
        Box((0.16, 0.13, 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, 0.051)),
        material=enamel,
        name="plinth",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.380),
        origin=Origin(xyz=(-0.045, 0.0, 0.256)),
        material=steel,
        name="column",
    )
    body.visual(
        Box((0.14, 0.09, 0.080)),
        origin=Origin(xyz=(0.015, 0.0, 0.482)),
        material=enamel,
        name="head_shell",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.062),
        origin=Origin(xyz=(0.098, 0.0, 0.468), rpy=(0.0, 1.57079632679, 0.0)),
        material=steel,
        name="nose",
    )
    body.visual(
        Box((0.016, 0.065, 0.180)),
        origin=Origin(xyz=(0.006, 0.0, 0.156)),
        material=steel,
        name="guide",
    )
    body.visual(
        Box((0.078, 0.046, 0.024)),
        origin=Origin(xyz=(-0.019, 0.0, 0.258)),
        material=steel,
        name="brace",
    )
    body.visual(
        Box((0.012, 0.024, 0.020)),
        origin=Origin(xyz=(0.104, 0.0, 0.028)),
        material=dark_trim,
        name="switch_mount",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_trim,
        name="chuck",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.199)),
        material=steel,
        name="beater",
    )

    platform = model.part("platform")
    platform.visual(
        Box((0.024, 0.070, 0.060)),
        origin=Origin(),
        material=steel,
        name="carriage",
    )
    platform.visual(
        Box((0.074, 0.030, 0.018)),
        origin=Origin(xyz=(0.037, 0.0, -0.016)),
        material=steel,
        name="arm",
    )
    platform.visual(
        Cylinder(radius=0.052, length=0.008),
        origin=Origin(xyz=(0.070, 0.0, -0.028)),
        material=dark_trim,
        name="tray",
    )

    toggle = model.part("toggle")
    toggle.visual(
        Box((0.028, 0.012, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=rubber,
        name="lever",
    )
    toggle.visual(
        Box((0.010, 0.016, 0.010)),
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
        material=rubber,
        name="cap",
    )

    model.articulation(
        "body_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(0.098, 0.0, 0.442)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=30.0),
    )
    model.articulation(
        "body_to_platform",
        ArticulationType.PRISMATIC,
        parent=body,
        child=platform,
        origin=Origin(xyz=(0.026, 0.0, 0.138)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.070,
        ),
    )
    model.articulation(
        "body_to_toggle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=toggle,
        origin=Origin(xyz=(0.110, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spindle = object_model.get_part("spindle")
    platform = object_model.get_part("platform")
    toggle = object_model.get_part("toggle")

    lift = object_model.get_articulation("body_to_platform")
    switch = object_model.get_articulation("body_to_toggle")

    lift_upper = 0.070
    switch_upper = 0.35

    with ctx.pose({lift: 0.0}):
        ctx.expect_overlap(
            spindle,
            platform,
            axes="xy",
            elem_a="beater",
            elem_b="tray",
            min_overlap=0.040,
            name="beater stays centered over the tray",
        )
        ctx.expect_gap(
            spindle,
            platform,
            axis="z",
            positive_elem="beater",
            negative_elem="tray",
            min_gap=0.100,
            max_gap=0.130,
            name="rest tray sits well below the beater",
        )
        ctx.expect_gap(
            platform,
            body,
            axis="x",
            positive_elem="carriage",
            negative_elem="guide",
            max_gap=0.0005,
            max_penetration=0.0005,
            name="carriage stays in contact with the guide face",
        )
        ctx.expect_overlap(
            platform,
            body,
            axes="yz",
            elem_a="carriage",
            elem_b="guide",
            min_overlap=0.050,
            name="carriage remains registered to the short guide",
        )
        rest_platform_pos = ctx.part_world_position(platform)

    with ctx.pose({lift: lift_upper}):
        ctx.expect_overlap(
            spindle,
            platform,
            axes="xy",
            elem_a="beater",
            elem_b="tray",
            min_overlap=0.040,
            name="raised tray stays under the spindle",
        )
        ctx.expect_gap(
            spindle,
            platform,
            axis="z",
            positive_elem="beater",
            negative_elem="tray",
            min_gap=0.040,
            max_gap=0.070,
            name="raised tray approaches the beater without contact",
        )
        ctx.expect_gap(
            platform,
            body,
            axis="x",
            positive_elem="carriage",
            negative_elem="guide",
            max_gap=0.0005,
            max_penetration=0.0005,
            name="carriage keeps contact with the guide face at full lift",
        )
        ctx.expect_overlap(
            platform,
            body,
            axes="yz",
            elem_a="carriage",
            elem_b="guide",
            min_overlap=0.050,
            name="guide still captures the carriage at full lift",
        )
        raised_platform_pos = ctx.part_world_position(platform)

    ctx.check(
        "platform lifts upward",
        rest_platform_pos is not None
        and raised_platform_pos is not None
        and raised_platform_pos[2] > rest_platform_pos[2] + 0.060,
        details=f"rest={rest_platform_pos}, raised={raised_platform_pos}",
    )

    with ctx.pose({switch: 0.0}):
        neutral_toggle_aabb = ctx.part_element_world_aabb(toggle, elem="lever")
    with ctx.pose({switch: switch_upper}):
        raised_toggle_aabb = ctx.part_element_world_aabb(toggle, elem="lever")

    neutral_top = None if neutral_toggle_aabb is None else neutral_toggle_aabb[1][2]
    raised_top = None if raised_toggle_aabb is None else raised_toggle_aabb[1][2]
    ctx.check(
        "toggle pivots upward",
        neutral_top is not None and raised_top is not None and raised_top > neutral_top + 0.006,
        details=f"neutral_top={neutral_top}, raised_top={raised_top}",
    )

    spindle_limits = object_model.get_articulation("body_to_spindle").motion_limits
    ctx.check(
        "spindle is authored as continuous rotation",
        spindle_limits is not None
        and spindle_limits.lower is None
        and spindle_limits.upper is None,
        details=f"limits={spindle_limits}",
    )

    return ctx.report()


object_model = build_object_model()
