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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_panel_three_branch_bracket")

    cast_steel = Material("powder_coated_steel", color=(0.18, 0.20, 0.22, 1.0))
    edge_steel = Material("dark_edge_steel", color=(0.07, 0.08, 0.09, 1.0))
    pod_blue = Material("blue_service_pods", color=(0.05, 0.22, 0.43, 1.0))
    plate_gray = Material("zinc_plated_end_plates", color=(0.62, 0.66, 0.68, 1.0))
    fastener = Material("black_socket_fasteners", color=(0.015, 0.015, 0.018, 1.0))
    warning = Material("yellow_service_mark", color=(0.95, 0.72, 0.12, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.035, 0.50, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=cast_steel,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.055, 0.54, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=edge_steel,
        name="ground_flange",
    )
    backplate.visual(
        Box((0.041, 0.090, 0.640)),
        origin=Origin(xyz=(0.037, 0.0, 0.360)),
        material=edge_steel,
        name="vertical_spine",
    )
    backplate.visual(
        Box((0.046, 0.012, 0.595)),
        origin=Origin(xyz=(0.045, -0.050, 0.360)),
        material=cast_steel,
        name="spine_rib_0",
    )
    backplate.visual(
        Box((0.046, 0.012, 0.595)),
        origin=Origin(xyz=(0.045, 0.050, 0.360)),
        material=cast_steel,
        name="spine_rib_1",
    )

    screw_positions = (
        (-0.185, 0.640),
        (0.185, 0.640),
        (-0.185, 0.080),
        (0.185, 0.080),
    )
    for idx, (y, z) in enumerate(screw_positions):
        backplate.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(0.0205, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"wall_screw_{idx}",
        )
        backplate.visual(
            Cylinder(radius=0.006, length=0.009),
            origin=Origin(xyz=(0.025, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=edge_steel,
            name=f"screw_socket_{idx}",
        )

    pivot_zs = (0.580, 0.360, 0.140)
    boss_names = ("upper_boss", "middle_boss", "lower_boss")
    for z, boss_name in zip(pivot_zs, boss_names):
        backplate.visual(
            Box((0.042, 0.118, 0.030)),
            origin=Origin(xyz=(0.060, 0.0, z)),
            material=cast_steel,
            name=f"{boss_name}_pad",
        )
        backplate.visual(
            Cylinder(radius=0.033, length=0.031),
            origin=Origin(xyz=(0.072, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=edge_steel,
            name=boss_name,
        )
        backplate.visual(
            Cylinder(radius=0.014, length=0.034),
            origin=Origin(xyz=(0.073, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"{boss_name}_pin_face",
        )

    branch_specs = (
        ("upper_pod", "upper_plate", "upper_mount", "upper_pivot", 0.580, -0.185, 0.105, -24.0),
        ("middle_pod", "middle_plate", "middle_mount", "middle_pivot", 0.360, 0.220, 0.000, 0.0),
        ("lower_pod", "lower_plate", "lower_mount", "lower_pivot", 0.140, -0.185, -0.105, 24.0),
    )
    joint_x = 0.0875
    tip_joint_x = 0.046

    for part_name, plate_name, mount_name, joint_name, pivot_z, tip_y, tip_z, stripe_angle in branch_specs:
        pod = model.part(part_name)
        dy = tip_y
        dz = tip_z
        length = math.sqrt(dy * dy + dz * dz)
        arm_angle = math.atan2(dz, dy)

        pod.visual(
            Cylinder(radius=0.026, length=0.030),
            origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pod_blue,
            name="hub",
        )
        pod.visual(
            Cylinder(radius=0.016, length=0.034),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name="bearing_cap",
        )
        pod.visual(
            Box((0.032, length + 0.035, 0.036)),
            origin=Origin(xyz=(0.028, dy / 2.0, dz / 2.0), rpy=(arm_angle, 0.0, 0.0)),
            material=pod_blue,
            name="arm_web",
        )
        pod.visual(
            Box((0.010, length * 0.55, 0.014)),
            origin=Origin(xyz=(0.044, dy * 0.36, dz * 0.36), rpy=(arm_angle, 0.0, 0.0)),
            material=edge_steel,
            name="front_rib",
        )
        pod.visual(
            Cylinder(radius=0.025, length=0.028),
            origin=Origin(xyz=(0.032, dy, dz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pod_blue,
            name="tip_socket",
        )

        plate = model.part(plate_name)
        plate.visual(
            Cylinder(radius=0.016, length=0.022),
            origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=edge_steel,
            name="journal",
        )
        plate.visual(
            Box((0.020, 0.096, 0.074)),
            origin=Origin(xyz=(0.018, 0.0, 0.0)),
            material=plate_gray,
            name="tip_plate",
        )
        plate.visual(
            Box((0.006, 0.010, 0.065)),
            origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(stripe_angle))),
            material=warning,
            name="service_stripe",
        )

        for idx, (sy, sz) in enumerate(((-0.030, -0.022), (0.030, -0.022), (-0.030, 0.022), (0.030, 0.022))):
            plate.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(
                    xyz=(0.031, sy, sz),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=fastener,
                name=f"tip_screw_{idx}",
            )

        model.articulation(
            mount_name,
            ArticulationType.FIXED,
            parent=backplate,
            child=pod,
            origin=Origin(xyz=(joint_x, 0.0, pivot_z)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=pod,
            child=plate,
            origin=Origin(xyz=(tip_joint_x, dy, dz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.70, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    backplate = object_model.get_part("backplate")
    upper = object_model.get_part("upper_pod")
    middle = object_model.get_part("middle_pod")
    lower = object_model.get_part("lower_pod")
    upper_plate = object_model.get_part("upper_plate")
    middle_plate = object_model.get_part("middle_plate")
    lower_plate = object_model.get_part("lower_plate")
    upper_joint = object_model.get_articulation("upper_pivot")
    middle_joint = object_model.get_articulation("middle_pivot")
    lower_joint = object_model.get_articulation("lower_pivot")

    joints = (upper_joint, middle_joint, lower_joint)
    revolute_count = sum(
        1
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check(
        "three independent revolute branch pivots",
        revolute_count == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and len({j.child for j in joints}) == 3
        and all(getattr(j, "mimic", None) is None for j in joints),
        details="Expected exactly three separate revolute joints with distinct end-plate children and no mimic/shared drive.",
    )

    for pod, boss_name, check_name in (
        (upper, "upper_boss", "upper hinge seated"),
        (middle, "middle_boss", "middle hinge seated"),
        (lower, "lower_boss", "lower hinge seated"),
    ):
        ctx.expect_gap(
            pod,
            backplate,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="hub",
            negative_elem=boss_name,
            name=check_name,
        )

    for pod, plate, check_name in (
        (upper, upper_plate, "upper tip journal seated"),
        (middle, middle_plate, "middle tip journal seated"),
        (lower, lower_plate, "lower tip journal seated"),
    ):
        ctx.expect_gap(
            plate,
            pod,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="journal",
            negative_elem="tip_socket",
            name=check_name,
        )

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    upper_tip = elem_center(upper_plate, "tip_plate")
    middle_tip = elem_center(middle_plate, "tip_plate")
    lower_tip = elem_center(lower_plate, "tip_plate")
    ctx.check(
        "three tips arranged around vertical spine",
        upper_tip is not None
        and middle_tip is not None
        and lower_tip is not None
        and upper_tip[2] > middle_tip[2] > lower_tip[2]
        and upper_tip[1] < -0.12
        and middle_tip[1] > 0.16
        and lower_tip[1] < -0.12,
        details=f"tip centers upper={upper_tip}, middle={middle_tip}, lower={lower_tip}",
    )

    rest_middle_screw = elem_center(middle_plate, "tip_screw_3")
    rest_upper_screw = elem_center(upper_plate, "tip_screw_3")
    with ctx.pose({middle_joint: 0.65}):
        moved_middle_screw = elem_center(middle_plate, "tip_screw_3")
        moved_upper_screw = elem_center(upper_plate, "tip_screw_3")
    ctx.check(
        "middle pivot moves without driving other plates",
        rest_middle_screw is not None
        and moved_middle_screw is not None
        and rest_upper_screw is not None
        and moved_upper_screw is not None
        and abs(moved_middle_screw[2] - rest_middle_screw[2]) > 0.010
        and abs(moved_upper_screw[2] - rest_upper_screw[2]) < 0.002,
        details=f"middle rest={rest_middle_screw}, middle moved={moved_middle_screw}, upper moved={moved_upper_screw}",
    )

    return ctx.report()


object_model = build_object_model()
