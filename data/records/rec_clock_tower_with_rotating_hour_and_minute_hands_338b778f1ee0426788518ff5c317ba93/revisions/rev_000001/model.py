from __future__ import annotations

import math

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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heritage_platform_clock")

    cast_iron = Material("dark_green_cast_iron", rgba=(0.02, 0.075, 0.055, 1.0))
    highlighted_iron = Material("worn_highlighted_iron", rgba=(0.055, 0.12, 0.095, 1.0))
    dial_ivory = Material("aged_ivory_enamel", rgba=(0.93, 0.88, 0.72, 1.0))
    black_enamel = Material("black_enamel", rgba=(0.005, 0.005, 0.004, 1.0))
    brass = Material("aged_brass", rgba=(0.63, 0.43, 0.16, 1.0))

    body = model.part("clock_body")

    # Street-platform proportions: a little over three meters tall with a slim
    # cast-iron column and a two-sided drum clock at platform-eye height.
    body.visual(Cylinder(radius=0.17, length=0.075), origin=Origin(xyz=(0.0, 0.0, 0.0375)), material=cast_iron, name="round_plinth")
    body.visual(Cylinder(radius=0.115, length=0.105), origin=Origin(xyz=(0.0, 0.0, 0.125)), material=highlighted_iron, name="base_torus_proxy")
    body.visual(Cylinder(radius=0.078, length=2.34), origin=Origin(xyz=(0.0, 0.0, 1.315)), material=cast_iron, name="column_core")

    # Raised half-round ribs around the column create the fluted cast-iron read.
    for i in range(16):
        angle = 2.0 * math.pi * i / 16
        x = 0.080 * math.cos(angle)
        y = 0.080 * math.sin(angle)
        body.visual(
            Cylinder(radius=0.0065, length=2.18),
            origin=Origin(xyz=(x, y, 1.39)),
            material=highlighted_iron,
            name=f"column_flute_{i}",
        )

    # Capital and neck connecting the column directly into the clock case.
    body.visual(Cylinder(radius=0.105, length=0.080), origin=Origin(xyz=(0.0, 0.0, 2.515)), material=highlighted_iron, name="column_cap")
    body.visual(Cylinder(radius=0.060, length=0.210), origin=Origin(xyz=(0.0, 0.0, 2.66)), material=cast_iron, name="neck_post")
    body.visual(Box((0.33, 0.105, 0.075)), origin=Origin(xyz=(0.0, 0.0, 2.735)), material=cast_iron, name="under_case_saddle")

    # Double-sided circular clock housing.  Cylinders are rotated so their axis
    # runs along Y, making the faces oppose one another across the platform.
    y_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    body.visual(
        Cylinder(radius=0.435, length=0.238),
        origin=Origin(xyz=(0.0, 0.0, 2.95), rpy=y_axis.rpy),
        material=cast_iron,
        name="round_clock_case",
    )
    body.visual(Cylinder(radius=0.050, length=0.165), origin=Origin(xyz=(0.0, 0.0, 3.405)), material=cast_iron, name="top_stem")
    body.visual(Sphere(radius=0.070), origin=Origin(xyz=(0.0, 0.0, 3.505)), material=highlighted_iron, name="top_finial")
    body.visual(Cylinder(radius=0.090, length=0.048), origin=Origin(xyz=(0.0, 0.0, 3.355)), material=highlighted_iron, name="finial_base")

    face_z = 2.95
    face_radius = 0.335
    for side_name, side in (("front", -1.0), ("rear", 1.0)):
        body.visual(
            Cylinder(radius=0.397, length=0.022),
            origin=Origin(xyz=(0.0, side * 0.126, face_z), rpy=y_axis.rpy),
            material=black_enamel,
            name=f"{side_name}_outer_bezel",
        )
        body.visual(
            Cylinder(radius=0.356, length=0.010),
            origin=Origin(xyz=(0.0, side * 0.136, face_z), rpy=y_axis.rpy),
            material=dial_ivory,
            name=f"{side_name}_ivory_face",
        )
        body.visual(
            Cylinder(radius=0.026, length=0.004),
            origin=Origin(xyz=(0.0, side * 0.141, face_z), rpy=y_axis.rpy),
            material=brass,
            name=f"{side_name}_center_bush",
        )

        for hour in range(12):
            theta = 2.0 * math.pi * hour / 12.0
            marker_r = 0.276
            radial_len = 0.070 if hour % 3 == 0 else 0.045
            tangential = 0.018 if hour % 3 == 0 else 0.011
            body.visual(
                Box((tangential, 0.004, radial_len)),
                origin=Origin(
                    xyz=(marker_r * math.sin(theta), side * 0.141, face_z + marker_r * math.cos(theta)),
                    rpy=(0.0, theta, 0.0),
                ),
                material=black_enamel,
                name=f"{side_name}_hour_mark_{hour}",
            )

    def add_hand_pair(side_name: str, side: float, hour_angle: float, minute_angle: float) -> None:
        joint_y = side * 0.149
        axis = (0.0, side, 0.0)

        hour_hand = model.part(f"{side_name}_hour_hand")
        hour_layer = -side * 0.004
        hour_len = 0.205
        hour_tail = 0.052
        hour_hand.visual(
            Box((0.026, 0.003, hour_len)),
            origin=Origin(
                xyz=(math.sin(hour_angle) * hour_len / 2.0, hour_layer, math.cos(hour_angle) * hour_len / 2.0),
                rpy=(0.0, hour_angle, 0.0),
            ),
            material=black_enamel,
            name="hour_blade",
        )
        hour_hand.visual(
            Box((0.018, 0.003, hour_tail)),
            origin=Origin(
                xyz=(-math.sin(hour_angle) * hour_tail / 2.0, hour_layer, -math.cos(hour_angle) * hour_tail / 2.0),
                rpy=(0.0, hour_angle, 0.0),
            ),
            material=black_enamel,
            name="hour_counterweight",
        )
        hour_hand.visual(
            Cylinder(radius=0.024, length=0.004),
            origin=Origin(xyz=(0.0, hour_layer, 0.0), rpy=y_axis.rpy),
            material=brass,
            name="hour_hub",
        )
        model.articulation(
            f"{side_name}_hour_axis",
            ArticulationType.REVOLUTE,
            parent=body,
            child=hour_hand,
            origin=Origin(xyz=(0.0, joint_y, face_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=0.4, velocity=0.20, lower=0.0, upper=2.0 * math.pi),
        )

        minute_hand = model.part(f"{side_name}_minute_hand")
        minute_layer = side * 0.003
        minute_len = 0.285
        minute_tail = 0.070
        minute_hand.visual(
            Box((0.016, 0.003, minute_len)),
            origin=Origin(
                xyz=(math.sin(minute_angle) * minute_len / 2.0, minute_layer, math.cos(minute_angle) * minute_len / 2.0),
                rpy=(0.0, minute_angle, 0.0),
            ),
            material=black_enamel,
            name="minute_blade",
        )
        minute_hand.visual(
            Box((0.012, 0.003, minute_tail)),
            origin=Origin(
                xyz=(-math.sin(minute_angle) * minute_tail / 2.0, minute_layer, -math.cos(minute_angle) * minute_tail / 2.0),
                rpy=(0.0, minute_angle, 0.0),
            ),
            material=black_enamel,
            name="minute_counterweight",
        )
        minute_hand.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(0.0, minute_layer, 0.0), rpy=y_axis.rpy),
            material=brass,
            name="minute_hub",
        )
        model.articulation(
            f"{side_name}_minute_axis",
            ArticulationType.REVOLUTE,
            parent=body,
            child=minute_hand,
            origin=Origin(xyz=(0.0, joint_y, face_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=0.25, velocity=1.0, lower=0.0, upper=2.0 * math.pi),
        )

    add_hand_pair("front", -1.0, math.radians(-52.0), math.radians(58.0))
    add_hand_pair("rear", 1.0, math.radians(52.0), math.radians(-58.0))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_names = (
        "front_hour_axis",
        "front_minute_axis",
        "rear_hour_axis",
        "rear_minute_axis",
    )
    hand_joints = [object_model.get_articulation(name) for name in joint_names]
    ctx.check(
        "four hand revolute joints",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in hand_joints),
        details=f"joints={[joint.articulation_type for joint in hand_joints]}",
    )

    body = object_model.get_part("clock_body")
    front_hour = object_model.get_part("front_hour_hand")
    front_minute = object_model.get_part("front_minute_hand")
    rear_hour = object_model.get_part("rear_hour_hand")
    rear_minute = object_model.get_part("rear_minute_hand")

    ctx.expect_origin_distance(front_hour, front_minute, axes="xyz", max_dist=0.001, name="front hand axes are concentric")
    ctx.expect_origin_distance(rear_hour, rear_minute, axes="xyz", max_dist=0.001, name="rear hand axes are concentric")
    ctx.expect_contact(body, front_hour, elem_a="front_center_bush", elem_b="hour_hub", name="front hour hand rides on dial bushing")
    ctx.expect_contact(front_hour, front_minute, elem_a="hour_hub", elem_b="minute_hub", name="front hands share coaxial arbor")
    ctx.expect_contact(body, rear_hour, elem_a="rear_center_bush", elem_b="hour_hub", name="rear hour hand rides on dial bushing")
    ctx.expect_contact(rear_hour, rear_minute, elem_a="hour_hub", elem_b="minute_hub", name="rear hands share coaxial arbor")

    rest_box = ctx.part_element_world_aabb(front_minute, elem="minute_blade")
    with ctx.pose({"front_minute_axis": math.pi / 2.0}):
        turned_box = ctx.part_element_world_aabb(front_minute, elem="minute_blade")
    if rest_box is not None and turned_box is not None:
        rest_center = tuple((rest_box[0][i] + rest_box[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_box[0][i] + turned_box[1][i]) / 2.0 for i in range(3))
        moved = math.hypot(rest_center[0] - turned_center[0], rest_center[2] - turned_center[2]) > 0.10
    else:
        moved = False
        rest_center = turned_center = None
    ctx.check("front minute hand sweeps around face", moved, details=f"rest={rest_center}, turned={turned_center}")

    return ctx.report()


object_model = build_object_model()
