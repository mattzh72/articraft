from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_rotary_table")

    cast_iron = model.material("blued_cast_iron", rgba=(0.12, 0.16, 0.18, 1.0))
    dark_iron = model.material("darkened_iron", rgba=(0.035, 0.04, 0.045, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.44, 0.46, 0.46, 1.0))
    slot_black = model.material("blackened_slots", rgba=(0.01, 0.011, 0.012, 1.0))
    bolt_dark = model.material("oiled_fasteners", rgba=(0.02, 0.022, 0.024, 1.0))

    # Fixed machine base with two rigid side cheeks carrying the trunnion axis.
    base = model.part("base")
    base.visual(
        Box((1.18, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.96, 0.50, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=cast_iron,
        name="raised_plinth",
    )
    for index, x in enumerate((0.50, -0.50)):
        base.visual(
            Box((0.12, 0.18, 0.50)),
            origin=Origin(xyz=(x, 0.0, 0.36)),
            material=cast_iron,
            name=f"side_support_{index}",
        )
        # Exterior bearing cap: a visible circular register on the outside of
        # each side cheek, aligned with the trunnion shaft.
        cap_x = x + (0.072 if x > 0 else -0.072)
        base.visual(
            Cylinder(radius=0.115, length=0.028),
            origin=Origin(xyz=(cap_x, 0.0, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"outer_bearing_cap_{index}",
        )
        base.visual(
            Cylinder(radius=0.055, length=0.032),
            origin=Origin(xyz=(cap_x + (0.004 if x > 0 else -0.004), 0.0, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_iron,
            name=f"bearing_bore_{index}",
        )
        # Broad ribbing makes the side cheeks feel cast and well supported.
        for y in (0.135, -0.135):
            base.visual(
                Box((0.050, 0.090, 0.34)),
                origin=Origin(xyz=(x, y, 0.29)),
                material=cast_iron,
                name=f"cheek_rib_{index}_{0 if y > 0 else 1}",
            )

    # Tilting cradle: its part frame is exactly on the horizontal trunnion axis.
    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Box((0.78, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=dark_iron,
        name="cross_saddle",
    )
    tilt_frame.visual(
        Cylinder(radius=0.240, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_iron,
        name="rotary_bearing",
    )
    tilt_frame.visual(
        Cylinder(radius=0.285, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=bearing_steel,
        name="bearing_race",
    )
    for index, x in enumerate((0.410, -0.410)):
        tilt_frame.visual(
            Cylinder(radius=0.098, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"pivot_hub_{index}",
        )
        tilt_frame.visual(
            Cylinder(radius=0.062, length=0.055),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_iron,
            name=f"hub_boss_{index}",
        )

    # The rotary faceplate is a separate rotating part mounted on the cradle's
    # vertical bearing.  Surface details are attached to the rotating part, so
    # they visibly sweep around when the table is indexed.
    table = model.part("turntable")
    table.visual(
        Cylinder(radius=0.325, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=ground_steel,
        name="turntable_disk",
    )
    table.visual(
        Cylinder(radius=0.340, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=bearing_steel,
        name="dark_outer_rim",
    )
    table.visual(
        Cylinder(radius=0.050, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=slot_black,
        name="center_bore",
    )
    table.visual(
        Cylinder(radius=0.118, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=bearing_steel,
        name="center_register",
    )
    for index, x in enumerate((-0.145, 0.0, 0.145)):
        table.visual(
            Box((0.028, 0.500, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.071)),
            material=slot_black,
            name=f"slot_{index}",
        )
        table.visual(
            Box((0.070, 0.500, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.071)),
            material=dark_iron,
            name=f"slot_shoulder_{index}",
        )
    bolt_radius = 0.220
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = bolt_radius * math.cos(angle)
        y = bolt_radius * math.sin(angle)
        table.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.071)),
            material=bolt_dark,
            name=f"fixture_hole_{index}",
        )

    model.articulation(
        "base_to_tilt_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tilt_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.60, lower=-1.05, upper=1.05),
        motion_properties=MotionProperties(damping=12.0, friction=4.0),
    )
    model.articulation(
        "tilt_frame_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=tilt_frame,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.20),
        motion_properties=MotionProperties(damping=4.0, friction=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tilt_frame = object_model.get_part("tilt_frame")
    turntable = object_model.get_part("turntable")
    tilt_joint = object_model.get_articulation("base_to_tilt_frame")
    rotary_joint = object_model.get_articulation("tilt_frame_to_turntable")

    ctx.check(
        "table has continuous rotary axis",
        rotary_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={rotary_joint.articulation_type}",
    )
    ctx.check(
        "trunnion tilt is limited like a machine fixture",
        tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower is not None
        and tilt_joint.motion_limits.upper is not None
        and tilt_joint.motion_limits.lower < -0.9
        and tilt_joint.motion_limits.upper > 0.9,
        details=f"limits={tilt_joint.motion_limits}",
    )

    # The cradle is seated between the side supports, and the turntable rests on
    # its bearing race at the neutral pose.
    ctx.expect_contact(
        tilt_frame,
        base,
        elem_a="pivot_hub_0",
        elem_b="side_support_0",
        contact_tol=0.002,
        name="right trunnion hub bears on support cheek",
    )
    ctx.expect_contact(
        tilt_frame,
        base,
        elem_a="pivot_hub_1",
        elem_b="side_support_1",
        contact_tol=0.002,
        name="left trunnion hub bears on support cheek",
    )
    ctx.expect_contact(
        turntable,
        tilt_frame,
        elem_a="turntable_disk",
        elem_b="rotary_bearing",
        contact_tol=0.002,
        name="turntable is seated on rotary bearing",
    )

    rest_slot = ctx.part_element_world_aabb(turntable, elem="slot_0")
    with ctx.pose({rotary_joint: math.pi / 2.0}):
        spun_slot = ctx.part_element_world_aabb(turntable, elem="slot_0")
    if rest_slot is None or spun_slot is None:
        ctx.fail("rotary motion moves visible fixture slot", "missing slot AABB")
    else:
        rest_x = rest_slot[1][0] - rest_slot[0][0]
        spun_x = spun_slot[1][0] - spun_slot[0][0]
        ctx.check(
            "rotary motion moves visible fixture slot",
            spun_x > rest_x + 0.20,
            details=f"rest_x={rest_x:.3f}, spun_x={spun_x:.3f}",
        )

    rest_aabb = ctx.part_world_aabb(turntable)
    with ctx.pose({tilt_joint: 0.80}):
        tilted_aabb = ctx.part_world_aabb(turntable)
    if rest_aabb is None or tilted_aabb is None:
        ctx.fail("trunnion tilt changes table attitude", "missing turntable AABB")
    else:
        rest_z = rest_aabb[1][2] - rest_aabb[0][2]
        tilted_z = tilted_aabb[1][2] - tilted_aabb[0][2]
        ctx.check(
            "trunnion tilt changes table attitude",
            tilted_z > rest_z + 0.15,
            details=f"rest_z={rest_z:.3f}, tilted_z={tilted_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
