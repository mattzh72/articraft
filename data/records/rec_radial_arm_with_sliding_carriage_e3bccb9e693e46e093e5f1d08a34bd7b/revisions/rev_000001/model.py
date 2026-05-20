from __future__ import annotations

import math

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
    model = ArticulatedObject(name="side_wall_radial_arm_module")

    painted_plate = model.material("graphite_powdercoat", rgba=(0.08, 0.09, 0.10, 1.0))
    bearing_black = model.material("blackened_bearing", rgba=(0.015, 0.015, 0.014, 1.0))
    bolt_steel = model.material("brushed_bolt_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    safety_yellow = model.material("safety_yellow_arm", rgba=(0.95, 0.66, 0.08, 1.0))
    dark_rail = model.material("hardened_slide_rail", rgba=(0.16, 0.17, 0.18, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.05, 0.18, 0.34, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.46, 0.48, 0.48, 1.0))

    # The side plate is the grounded wall-mounted datum.  Its local origin is
    # the rotary axis center on the plate face, so the bearing and arm joint are
    # easy to read and verify.
    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.050, 0.480, 0.620)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=painted_plate,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.018, 0.420, 0.030)),
        origin=Origin(xyz=(0.008, 0.0, 0.245)),
        material=painted_plate,
        name="upper_stiffener",
    )
    side_plate.visual(
        Box((0.018, 0.420, 0.030)),
        origin=Origin(xyz=(0.008, 0.0, -0.245)),
        material=painted_plate,
        name="lower_stiffener",
    )
    side_plate.visual(
        Box((0.018, 0.030, 0.520)),
        origin=Origin(xyz=(0.008, 0.195, 0.0)),
        material=painted_plate,
        name="side_stiffener_0",
    )
    side_plate.visual(
        Box((0.018, 0.030, 0.520)),
        origin=Origin(xyz=(0.008, -0.195, 0.0)),
        material=painted_plate,
        name="side_stiffener_1",
    )
    side_plate.visual(
        Cylinder(radius=0.118, length=0.018),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_black,
        name="bearing_ring",
    )
    for index, (y, z) in enumerate(
        ((0.165, 0.205), (-0.165, 0.205), (0.165, -0.205), (-0.165, -0.205))
    ):
        side_plate.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(0.006, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"anchor_bolt_{index}",
        )

    # The rotating child combines the round boss, a straight radial arm, and the
    # fixed slide rails carried on the outward-facing arm face.
    radial_arm = model.part("radial_arm")
    radial_arm.visual(
        Cylinder(radius=0.086, length=0.056),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="hub_drum",
    )
    radial_arm.visual(
        Cylinder(radius=0.096, length=0.016),
        origin=Origin(xyz=(0.062, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rail,
        name="hub_face",
    )
    radial_arm.visual(
        Box((0.050, 0.700, 0.070)),
        origin=Origin(xyz=(0.080, 0.350, 0.0)),
        material=safety_yellow,
        name="arm_web",
    )
    radial_arm.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.080, 0.700, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_yellow,
        name="rounded_arm_end",
    )
    radial_arm.visual(
        Box((0.018, 0.580, 0.012)),
        origin=Origin(xyz=(0.114, 0.360, 0.030)),
        material=dark_rail,
        name="guide_rail_upper",
    )
    radial_arm.visual(
        Box((0.018, 0.580, 0.012)),
        origin=Origin(xyz=(0.114, 0.360, -0.030)),
        material=dark_rail,
        name="guide_rail_lower",
    )
    radial_arm.visual(
        Box((0.012, 0.070, 0.110)),
        origin=Origin(xyz=(0.113, 0.075, 0.0)),
        material=dark_rail,
        name="root_stop",
    )
    radial_arm.visual(
        Box((0.012, 0.070, 0.110)),
        origin=Origin(xyz=(0.113, 0.645, 0.0)),
        material=dark_rail,
        name="tip_stop",
    )

    rotary_joint = model.articulation(
        "plate_to_arm",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=radial_arm,
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.35, upper=1.35),
    )

    # A compact carriage rides on the arm face.  The child frame is the center of
    # the carriage at the inboard end of travel; positive prismatic motion moves
    # it toward the arm tip.
    carriage = model.part("tool_carriage")
    carriage.visual(
        Box((0.040, 0.120, 0.110)),
        origin=Origin(xyz=(0.143, 0.0, 0.0)),
        material=carriage_blue,
        name="slide_plate",
    )
    carriage.visual(
        Box((0.045, 0.115, 0.018)),
        origin=Origin(xyz=(0.144, 0.0, 0.049)),
        material=carriage_blue,
        name="upper_guide_shoe",
    )
    carriage.visual(
        Box((0.045, 0.115, 0.018)),
        origin=Origin(xyz=(0.144, 0.0, -0.049)),
        material=carriage_blue,
        name="lower_guide_shoe",
    )
    carriage.visual(
        Box((0.030, 0.060, 0.070)),
        origin=Origin(xyz=(0.178, 0.0, 0.0)),
        material=carriage_blue,
        name="tool_block",
    )
    carriage.visual(
        Cylinder(radius=0.024, length=0.058),
        origin=Origin(xyz=(0.222, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tool_steel,
        name="tool_socket",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.291, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rail,
        name="tool_bit",
    )

    carriage_joint = model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=radial_arm,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.160, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.360),
    )

    # Preserve semantic handles for the tests without relying on string literals
    # for every authoring detail.
    model.meta["primary_joints"] = (rotary_joint.name, carriage_joint.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    radial_arm = object_model.get_part("radial_arm")
    carriage = object_model.get_part("tool_carriage")
    rotary_joint = object_model.get_articulation("plate_to_arm")
    carriage_joint = object_model.get_articulation("arm_to_carriage")

    ctx.check(
        "module has rotary boss and prismatic carriage",
        object_model.meta.get("primary_joints") == ("plate_to_arm", "arm_to_carriage"),
        details=f"primary_joints={object_model.meta.get('primary_joints')}",
    )

    ctx.expect_contact(
        side_plate,
        radial_arm,
        elem_a="bearing_ring",
        elem_b="hub_drum",
        contact_tol=0.0015,
        name="rotary boss seats against side plate bearing",
    )
    ctx.expect_contact(
        carriage,
        radial_arm,
        elem_a="slide_plate",
        elem_b="guide_rail_upper",
        contact_tol=0.0015,
        name="carriage bears on upper rail",
    )
    ctx.expect_contact(
        carriage,
        radial_arm,
        elem_a="slide_plate",
        elem_b="guide_rail_lower",
        contact_tol=0.0015,
        name="carriage bears on lower rail",
    )
    ctx.expect_overlap(
        carriage,
        radial_arm,
        axes="y",
        elem_a="slide_plate",
        elem_b="guide_rail_upper",
        min_overlap=0.10,
        name="inboard carriage remains on rail",
    )

    rest_arm_aabb = ctx.part_element_world_aabb(radial_arm, elem="arm_web")
    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({rotary_joint: 0.80}):
        rotated_arm_aabb = ctx.part_element_world_aabb(radial_arm, elem="arm_web")
    with ctx.pose({carriage_joint: 0.360}):
        ctx.expect_overlap(
            carriage,
            radial_arm,
            axes="y",
            elem_a="slide_plate",
            elem_b="guide_rail_upper",
            min_overlap=0.10,
            name="outboard carriage remains on rail",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    if rest_arm_aabb is not None and rotated_arm_aabb is not None:
        rest_z = (rest_arm_aabb[0][2] + rest_arm_aabb[1][2]) / 2.0
        rotated_z = (rotated_arm_aabb[0][2] + rotated_arm_aabb[1][2]) / 2.0
        ctx.check(
            "positive rotary motion swings arm upward",
            rotated_z > rest_z + 0.20,
            details=f"rest_z={rest_z:.3f}, rotated_z={rotated_z:.3f}",
        )
    else:
        ctx.fail("positive rotary motion swings arm upward", "arm_web AABB was unavailable")

    ctx.check(
        "positive prismatic motion drives carriage toward arm tip",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[1] > rest_carriage_pos[1] + 0.34,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    return ctx.report()


object_model = build_object_model()
