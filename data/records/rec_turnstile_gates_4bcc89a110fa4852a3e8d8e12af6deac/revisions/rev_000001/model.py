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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    frame_metal = model.material("powder_coated_dark_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", rgba=(0.66, 0.69, 0.70, 1.0))
    bearing_black = model.material("black_bearing_housing", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("black_rubber_caps", rgba=(0.01, 0.01, 0.01, 1.0))
    safety_yellow = model.material("safety_yellow_labels", rgba=(1.0, 0.78, 0.06, 1.0))

    frame = model.part("frame")

    # Broad fixed support: floor plate, four guard posts, upper/lower rails, and
    # the central pedestal that carries the rotating hub bearing.
    frame.visual(
        Box((2.35, 1.55, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=frame_metal,
        name="floor_plate",
    )
    for i, x in enumerate((-1.05, 1.05)):
        for j, y in enumerate((-0.68, 0.68)):
            frame.visual(
                Cylinder(radius=0.045, length=1.50),
                origin=Origin(xyz=(x, y, 0.81)),
                material=frame_metal,
                name=f"corner_post_{i}_{j}",
            )

    for y in (-0.68, 0.68):
        frame.visual(
            Box((2.18, 0.070, 0.080)),
            origin=Origin(xyz=(0.0, y, 1.55)),
            material=frame_metal,
            name=f"top_side_rail_{'rear' if y > 0 else 'front'}",
        )
        frame.visual(
            Box((2.18, 0.060, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.35)),
            material=frame_metal,
            name=f"lower_side_rail_{'rear' if y > 0 else 'front'}",
        )
    for x in (-1.05, 1.05):
        frame.visual(
            Box((0.070, 1.42, 0.080)),
            origin=Origin(xyz=(x, 0.0, 1.55)),
            material=frame_metal,
            name=f"top_end_rail_{'positive' if x > 0 else 'negative'}",
        )
        frame.visual(
            Box((0.060, 1.42, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.35)),
            material=frame_metal,
            name=f"lower_end_rail_{'positive' if x > 0 else 'negative'}",
        )

    frame.visual(
        Cylinder(radius=0.075, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=frame_metal,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.160, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=bearing_black,
        name="lower_bearing",
    )
    frame.visual(
        Cylinder(radius=0.115, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=frame_metal,
        name="column_collar",
    )

    # Small warning labels are seated on the outside faces of two guard posts.
    for x in (-1.05, 1.05):
        frame.visual(
            Box((0.055, 0.012, 0.220)),
            origin=Origin(xyz=(x, -0.727, 1.02)),
            material=safety_yellow,
            name=f"warning_label_{'positive' if x > 0 else 'negative'}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.110, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="hub_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.138, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=brushed_steel,
        name="top_hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.138, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=brushed_steel,
        name="bottom_hub_flange",
    )

    arm_radius = 0.030
    arm_length = 0.570
    arm_mid_radius = 0.390
    arm_tip_radius = 0.690
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        ux = math.cos(angle)
        uy = math.sin(angle)
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(arm_mid_radius * ux, arm_mid_radius * uy, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brushed_steel,
            name=f"arm_{index}",
        )
        rotor.visual(
            Sphere(radius=0.046),
            origin=Origin(xyz=(arm_tip_radius * ux, arm_tip_radius * uy, 0.0)),
            material=rubber,
            name=f"end_cap_{index}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "rotor joint is continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={joint.articulation_type}",
    )
    ctx.check(
        "rotor spins on vertical center axis",
        tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="bottom_hub_flange",
        elem_b="lower_bearing",
        name="hub flange sits on fixed bearing",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.0,
        name="three arms sit inside the fixed frame footprint",
    )
    frame_aabb = ctx.part_world_aabb(frame)
    rotor_aabb = ctx.part_world_aabb(rotor)
    if frame_aabb is not None and rotor_aabb is not None:
        frame_min, frame_max = frame_aabb
        rotor_min, rotor_max = rotor_aabb
        frame_span_x = frame_max[0] - frame_min[0]
        frame_span_y = frame_max[1] - frame_min[1]
        rotor_span_x = rotor_max[0] - rotor_min[0]
        rotor_span_y = rotor_max[1] - rotor_min[1]
        ctx.check(
            "rotating member is smaller than broad support",
            rotor_span_x < 0.70 * frame_span_x and rotor_span_y < 0.95 * frame_span_y,
            details=(
                f"rotor_span=({rotor_span_x:.3f}, {rotor_span_y:.3f}), "
                f"frame_span=({frame_span_x:.3f}, {frame_span_y:.3f})"
            ),
        )
    else:
        ctx.fail("rotating member is smaller than broad support", "AABB unavailable")

    arm_aabb_rest = ctx.part_element_world_aabb(rotor, elem="arm_0")
    with ctx.pose({joint: math.pi / 2.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.0,
            name="rotated arms remain inside fixed frame",
        )
        arm_aabb_rotated = ctx.part_element_world_aabb(rotor, elem="arm_0")
    if arm_aabb_rest is not None and arm_aabb_rotated is not None:
        rest_min, rest_max = arm_aabb_rest
        rotated_min, rotated_max = arm_aabb_rotated
        rest_center = ((rest_min[0] + rest_max[0]) / 2.0, (rest_min[1] + rest_max[1]) / 2.0)
        rotated_center = ((rotated_min[0] + rotated_max[0]) / 2.0, (rotated_min[1] + rotated_max[1]) / 2.0)
        ctx.check(
            "arm geometry follows hub rotation",
            abs(rest_center[0] - rotated_center[0]) > 0.10 and abs(rest_center[1] - rotated_center[1]) > 0.10,
            details=f"rest_center={rest_center}, rotated_center={rotated_center}",
        )
    else:
        ctx.fail("arm geometry follows hub rotation", "arm_0 AABB unavailable")

    return ctx.report()


object_model = build_object_model()
