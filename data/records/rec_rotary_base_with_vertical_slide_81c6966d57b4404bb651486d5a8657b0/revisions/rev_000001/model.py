from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_base_vertical_slide")

    cast_iron = model.material("dark_cast_iron", rgba=(0.07, 0.075, 0.08, 1.0))
    rotary_blue = model.material("painted_turntable_blue", rgba=(0.07, 0.16, 0.32, 1.0))
    column_paint = model.material("machine_column_gray", rgba=(0.32, 0.34, 0.35, 1.0))
    ground_steel = model.material("brushed_guide_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    carriage_orange = model.material("carriage_safety_orange", rgba=(0.95, 0.42, 0.10, 1.0))
    tooling_steel = model.material("machined_tooling_steel", rgba=(0.54, 0.56, 0.56, 1.0))
    black = model.material("black_fasteners", rgba=(0.015, 0.015, 0.015, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.31, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.16, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=cast_iron,
        name="bearing_stator",
    )
    base.visual(
        Cylinder(radius=0.115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=ground_steel,
        name="bearing_race",
    )
    for idx, (x, y) in enumerate(((0.22, 0.0), (-0.22, 0.0), (0.0, 0.22), (0.0, -0.22))):
        base.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.056)),
            material=black,
            name=f"anchor_bolt_{idx}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.235, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=rotary_blue,
        name="rotary_table",
    )
    turntable.visual(
        Box((0.11, 0.012, 0.004)),
        origin=Origin(xyz=(0.145, 0.0, 0.057)),
        material=tooling_steel,
        name="index_mark",
    )
    turntable.visual(
        Box((0.15, 0.16, 0.04)),
        origin=Origin(xyz=(-0.08, 0.0, 0.075)),
        material=column_paint,
        name="column_foot",
    )
    turntable.visual(
        Box((0.10, 0.12, 0.86)),
        origin=Origin(xyz=(-0.08, 0.0, 0.485)),
        material=column_paint,
        name="column_body",
    )
    # Two polished round rails sit on the front of the column.  They are
    # slightly pocketed into the painted column face and tied together by
    # end clamps, so the fixed vertical-slide structure reads as one assembly.
    for idx, y in enumerate((-0.045, 0.045)):
        turntable.visual(
            Cylinder(radius=0.014, length=0.80),
            origin=Origin(xyz=(-0.018, y, 0.50)),
            material=ground_steel,
            name=f"guide_rail_{idx}",
        )
    turntable.visual(
        Box((0.04, 0.155, 0.032)),
        origin=Origin(xyz=(-0.018, 0.0, 0.118)),
        material=tooling_steel,
        name="lower_rail_clamp",
    )
    turntable.visual(
        Box((0.04, 0.155, 0.032)),
        origin=Origin(xyz=(-0.018, 0.0, 0.882)),
        material=tooling_steel,
        name="upper_rail_clamp",
    )
    for idx, y in enumerate((-0.075, 0.075)):
        turntable.visual(
            Box((0.08, 0.028, 0.16)),
            origin=Origin(xyz=(-0.04, y, 0.13)),
            material=column_paint,
            name=f"base_gusset_{idx}",
        )

    carriage = model.part("carriage")
    for idx, y in enumerate((-0.045, 0.045)):
        carriage.visual(
            Box((0.030, 0.052, 0.125)),
            origin=Origin(xyz=(0.011, y, 0.070)),
            material=tooling_steel,
            name=f"bearing_shoe_{idx}",
        )
    carriage.visual(
        Box((0.060, 0.205, 0.145)),
        origin=Origin(xyz=(0.055, 0.0, 0.0725)),
        material=carriage_orange,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.026, 0.165, 0.185)),
        origin=Origin(xyz=(0.098, 0.0, 0.075)),
        material=tooling_steel,
        name="tooling_face",
    )
    for idx, (y, z) in enumerate(((-0.052, 0.032), (0.052, 0.032), (-0.052, 0.118), (0.052, 0.118))):
        carriage.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=Origin(xyz=(0.1145, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"tool_bolt_{idx}",
        )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-pi, upper=pi),
        motion_properties=MotionProperties(damping=0.8, friction=0.2),
    )
    model.articulation(
        "turntable_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.38),
        motion_properties=MotionProperties(damping=1.5, friction=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    rotary_joint = object_model.get_articulation("base_to_turntable")
    slide_joint = object_model.get_articulation("turntable_to_carriage")

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem="rotary_table",
        negative_elem="bearing_stator",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotary table sits on the bearing stator",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="xy",
        elem_a="rotary_table",
        elem_b="bearing_stator",
        min_overlap=0.12,
        name="turntable is centered over the grounded bearing",
    )
    for idx in (0, 1):
        ctx.expect_contact(
            carriage,
            turntable,
            elem_a=f"bearing_shoe_{idx}",
            elem_b=f"guide_rail_{idx}",
            contact_tol=0.002,
            name=f"bearing shoe {idx} rides on its guide rail at the lower stop",
        )
        ctx.expect_overlap(
            carriage,
            turntable,
            axes="z",
            elem_a=f"bearing_shoe_{idx}",
            elem_b=f"guide_rail_{idx}",
            min_overlap=0.11,
            name=f"bearing shoe {idx} has retained rail engagement at the lower stop",
        )

    rest_pos = ctx.part_world_position(carriage)
    rest_mark_aabb = ctx.part_element_world_aabb(turntable, elem="index_mark")

    with ctx.pose({rotary_joint: pi / 2.0}):
        rotated_mark_aabb = ctx.part_element_world_aabb(turntable, elem="index_mark")

    if rest_mark_aabb is not None and rotated_mark_aabb is not None:
        rest_center = tuple((rest_mark_aabb[0][i] + rest_mark_aabb[1][i]) / 2.0 for i in range(3))
        rotated_center = tuple((rotated_mark_aabb[0][i] + rotated_mark_aabb[1][i]) / 2.0 for i in range(3))
    else:
        rest_center = rotated_center = None
    ctx.check(
        "revolute base rotates the table about the vertical axis",
        rest_center is not None
        and rotated_center is not None
        and rest_center[0] > 0.09
        and abs(rest_center[1]) < 0.02
        and rotated_center[1] > 0.09
        and abs(rotated_center[0]) < 0.02,
        details=f"index mark center rest={rest_center}, rotated={rotated_center}",
    )

    with ctx.pose({slide_joint: 0.38}):
        raised_pos = ctx.part_world_position(carriage)
        for idx in (0, 1):
            ctx.expect_contact(
                carriage,
                turntable,
                elem_a=f"bearing_shoe_{idx}",
                elem_b=f"guide_rail_{idx}",
                contact_tol=0.002,
                name=f"bearing shoe {idx} remains on its guide rail at full lift",
            )
            ctx.expect_overlap(
                carriage,
                turntable,
                axes="z",
                elem_a=f"bearing_shoe_{idx}",
                elem_b=f"guide_rail_{idx}",
                min_overlap=0.11,
                name=f"bearing shoe {idx} retains rail engagement at full lift",
            )

    ctx.check(
        "prismatic carriage lifts along the column axis",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.35
        and abs(raised_pos[0] - rest_pos[0]) < 0.002
        and abs(raised_pos[1] - rest_pos[1]) < 0.002,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
