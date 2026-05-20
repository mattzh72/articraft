from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_drill_press")

    cast = model.material("dark_cast_iron", rgba=(0.08, 0.10, 0.12, 1.0))
    painted = model.material("blue_gray_paint", rgba=(0.18, 0.26, 0.34, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    safety = model.material("safety_yellow", rgba=(0.95, 0.68, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.45, 0.82, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=cast,
        name="base_plate",
    )
    base.visual(
        Box((1.22, 0.62, 0.035)),
        origin=Origin(xyz=(0.08, 0.0, 0.1375)),
        material=painted,
        name="raised_base",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.06),
        origin=Origin(xyz=(-0.50, 0.0, 0.15)),
        material=cast,
        name="column_flange",
    )
    base.visual(
        Cylinder(radius=0.075, length=1.15),
        origin=Origin(xyz=(-0.50, 0.0, 0.695)),
        material=steel,
        name="vertical_column",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.06),
        origin=Origin(xyz=(-0.50, 0.0, 1.30)),
        material=cast,
        name="column_cap",
    )
    for i, (x, y) in enumerate(((-0.63, -0.27), (-0.63, 0.27), (0.58, -0.27), (0.58, 0.27))):
        base.visual(
            Cylinder(radius=0.035, length=0.02),
            origin=Origin(xyz=(x, y, 0.13)),
            material=black,
            name=f"base_bolt_{i}",
        )

    # A column-mounted work table and collar make the scale and drilling area clear.
    base.visual(
        Cylinder(radius=0.13, length=0.08),
        origin=Origin(xyz=(-0.50, 0.0, 0.52)),
        material=cast,
        name="table_collar",
    )
    base.visual(
        Box((0.70, 0.10, 0.08)),
        origin=Origin(xyz=(-0.05, 0.0, 0.52)),
        material=cast,
        name="table_support",
    )
    base.visual(
        Box((0.70, 0.45, 0.06)),
        origin=Origin(xyz=(0.22, 0.0, 0.55)),
        material=painted,
        name="work_table",
    )
    for i, y in enumerate((-0.11, 0.11)):
        base.visual(
            Box((0.60, 0.016, 0.008)),
            origin=Origin(xyz=(0.22, y, 0.584)),
            material=black,
            name=f"table_slot_{i}",
        )

    # The fixed yoke is an open rectangular guide around the telescoping radial arm.
    base.visual(
        Box((0.50, 0.30, 0.035)),
        origin=Origin(xyz=(-0.20, 0.0, 1.51)),
        material=cast,
        name="yoke_top",
    )
    base.visual(
        Box((0.50, 0.30, 0.035)),
        origin=Origin(xyz=(-0.20, 0.0, 1.33)),
        material=cast,
        name="yoke_bottom",
    )
    for i, y in enumerate((-0.075, 0.075)):
        base.visual(
            Box((0.50, 0.04, 0.22)),
            origin=Origin(xyz=(-0.20, y, 1.42)),
            material=cast,
            name=f"yoke_side_{i}",
        )
    for i, y in enumerate((-0.13, 0.13)):
        base.visual(
            Box((0.10, 0.05, 0.22)),
            origin=Origin(xyz=(-0.50, y, 1.39)),
            material=cast,
            name=f"column_yoke_strut_{i}",
        )

    radial_arm = model.part("radial_arm")
    radial_arm.visual(
        Box((1.88, 0.11, 0.11)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=painted,
        name="arm_rail",
    )
    radial_arm.visual(
        Box((0.055, 0.16, 0.16)),
        origin=Origin(xyz=(1.2075, 0.0, 0.0)),
        material=cast,
        name="front_stop",
    )
    radial_arm.visual(
        Box((1.25, 0.014, 0.006)),
        origin=Origin(xyz=(0.34, 0.025, 0.058)),
        material=steel,
        name="travel_scale",
    )
    radial_arm.visual(
        Box((1.15, 0.018, 0.006)),
        origin=Origin(xyz=(0.36, -0.025, 0.058)),
        material=black,
        name="rack_strip",
    )

    drill_head = model.part("drill_head")
    drill_head.visual(
        Box((0.28, 0.25, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=cast,
        name="head_top_saddle",
    )
    drill_head.visual(
        Box((0.28, 0.25, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=cast,
        name="head_bottom_saddle",
    )
    for i, y in enumerate((-0.075, 0.075)):
        drill_head.visual(
            Box((0.28, 0.04, 0.20)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=cast,
            name=f"head_saddle_side_{i}",
        )
    drill_head.visual(
        Box((0.18, 0.20, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=cast,
        name="head_neck",
    )
    drill_head.visual(
        Box((0.30, 0.27, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        material=painted,
        name="head_body",
    )
    drill_head.visual(
        Cylinder(radius=0.12, length=0.34),
        origin=Origin(xyz=(-0.055, 0.0, -0.28), rpy=(pi / 2, 0.0, 0.0)),
        material=painted,
        name="motor_housing",
    )
    drill_head.visual(
        Box((0.24, 0.06, 0.18)),
        origin=Origin(xyz=(0.11, 0.0, -0.28)),
        material=safety,
        name="belt_cover",
    )
    drill_head.visual(
        Cylinder(radius=0.035, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.51)),
        material=steel,
        name="quill",
    )
    drill_head.visual(
        Cylinder(radius=0.052, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.6275)),
        material=cast,
        name="chuck",
    )
    drill_head.visual(
        Cylinder(radius=0.008, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.745)),
        material=steel,
        name="drill_bit",
    )

    feed_lever = model.part("feed_lever")
    feed_lever.visual(
        Cylinder(radius=0.045, length=0.05),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=cast,
        name="lever_hub",
    )
    feed_lever.visual(
        Cylinder(radius=0.012, length=0.32),
        origin=Origin(xyz=(0.0, -0.04, -0.17)),
        material=steel,
        name="lever_bar",
    )
    feed_lever.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.0, -0.04, -0.34)),
        material=black,
        name="lever_knob",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.PRISMATIC,
        parent=base,
        child=radial_arm,
        origin=Origin(xyz=(-0.25, 0.0, 1.42)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.08, lower=0.0, upper=0.45),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=radial_arm,
        child=drill_head,
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.12, lower=0.0, upper=0.45),
    )
    model.articulation(
        "head_to_feed_lever",
        ArticulationType.REVOLUTE,
        parent=drill_head,
        child=feed_lever,
        origin=Origin(xyz=(-0.12, -0.17, -0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.9, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    radial_arm = object_model.get_part("radial_arm")
    drill_head = object_model.get_part("drill_head")
    feed_lever = object_model.get_part("feed_lever")
    arm_slide = object_model.get_articulation("column_to_arm")
    head_slide = object_model.get_articulation("arm_to_head")
    lever_joint = object_model.get_articulation("head_to_feed_lever")

    ctx.expect_gap(
        drill_head,
        base,
        axis="z",
        positive_elem="drill_bit",
        negative_elem="work_table",
        min_gap=0.005,
        max_gap=0.04,
        name="drill bit clears the work table",
    )
    ctx.expect_overlap(
        drill_head,
        base,
        axes="xy",
        elem_a="drill_bit",
        elem_b="work_table",
        min_overlap=0.004,
        name="drill bit is aimed over the table",
    )
    ctx.expect_gap(
        base,
        radial_arm,
        axis="z",
        positive_elem="yoke_top",
        negative_elem="arm_rail",
        min_gap=0.005,
        max_gap=0.04,
        name="radial arm has top guide clearance",
    )
    ctx.expect_overlap(
        radial_arm,
        base,
        axes="x",
        elem_a="arm_rail",
        elem_b="yoke_top",
        min_overlap=0.30,
        name="radial arm is retained in the yoke",
    )

    arm_rest = ctx.part_world_position(radial_arm)
    head_rest = ctx.part_world_position(drill_head)
    with ctx.pose({arm_slide: 0.45, head_slide: 0.45, lever_joint: 0.65}):
        ctx.expect_overlap(
            radial_arm,
            base,
            axes="x",
            elem_a="arm_rail",
            elem_b="yoke_top",
            min_overlap=0.08,
            name="extended arm keeps insertion in the yoke",
        )
        ctx.expect_overlap(
            drill_head,
            radial_arm,
            axes="x",
            elem_a="head_top_saddle",
            elem_b="arm_rail",
            min_overlap=0.20,
            name="head carriage remains on the radial rail",
        )
        arm_extended = ctx.part_world_position(radial_arm)
        head_extended = ctx.part_world_position(drill_head)

    ctx.check(
        "arm slide extends along the radial direction",
        arm_rest is not None and arm_extended is not None and arm_extended[0] > arm_rest[0] + 0.40,
        details=f"rest={arm_rest}, extended={arm_extended}",
    )
    ctx.check(
        "drill head traverses outward along the arm",
        head_rest is not None and head_extended is not None and head_extended[0] > head_rest[0] + 0.80,
        details=f"rest={head_rest}, extended={head_extended}",
    )
    ctx.expect_contact(
        feed_lever,
        drill_head,
        elem_a="lever_hub",
        elem_b="motor_housing",
        contact_tol=0.002,
        name="feed lever hub is mounted on the head",
    )

    return ctx.report()


object_model = build_object_model()
