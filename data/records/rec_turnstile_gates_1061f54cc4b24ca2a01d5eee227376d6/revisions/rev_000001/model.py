from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate(width: float, depth: float, height: float, radius: float):
    return ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=12),
        height,
        cap=True,
        center=True,
    )


def _annular_collar(outer_radius: float, inner_radius: float, height: float):
    half = height * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=3,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tripod_turnstile_gate")

    painted_metal = model.material("painted_graphite_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    satin_metal = model.material("satin_brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    bearing_steel = model.material("dark_bearing_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    polymer = model.material("warm_black_polymer", rgba=(0.055, 0.058, 0.062, 1.0))
    elastomer = model.material("soft_black_elastomer", rgba=(0.015, 0.015, 0.014, 1.0))
    smoked_polycarbonate = model.material("smoked_polycarbonate", rgba=(0.25, 0.32, 0.36, 0.38))
    seam_shadow = model.material("recessed_seam_shadow", rgba=(0.025, 0.027, 0.030, 1.0))

    floor_plate_mesh = _mesh("rounded_floor_plinth", _rounded_plate(1.62, 1.78, 0.060, 0.145))
    side_pod_mesh = _mesh("side_pod_rounded_shell", _rounded_plate(1.42, 0.145, 0.700, 0.055))
    rail_mesh = _mesh(
        "side_rail_tube",
        CapsuleGeometry(0.024, 1.26, radial_segments=28, height_segments=8)
        .rotate_y(math.pi / 2.0),
    )
    collar_mesh = _mesh("bearing_collar_annulus", _annular_collar(0.170, 0.073, 0.080))
    post_profile = [
        (0.0, 0.0),
        (0.240, 0.0),
        (0.240, 0.036),
        (0.206, 0.066),
        (0.172, 0.220),
        (0.154, 0.560),
        (0.132, 0.715),
        (0.0, 0.715),
    ]
    post_mesh = _mesh("tapered_center_post", LatheGeometry(post_profile, segments=88))
    lower_trim_ring_mesh = _mesh(
        "lower_torus_trim_ring",
        TorusGeometry(radius=0.185, tube=0.008, radial_segments=12, tubular_segments=72),
    )
    upper_trim_ring_mesh = _mesh(
        "upper_torus_trim_ring",
        TorusGeometry(radius=0.154, tube=0.008, radial_segments=12, tubular_segments=72),
    )

    arm_tube_mesh = _mesh(
        "radial_arm_capsule_tube",
        CapsuleGeometry(0.032, 0.426, radial_segments=32, height_segments=10)
        .rotate_y(math.pi / 2.0)
        .translate(0.355, 0.0, 0.0),
    )
    arm_grip_mesh = _mesh(
        "radial_arm_elastomer_grip",
        CapsuleGeometry(0.038, 0.084, radial_segments=32, height_segments=10)
        .rotate_y(math.pi / 2.0)
        .translate(0.552, 0.0, 0.0),
    )
    socket_mesh = _mesh(
        "hub_arm_socket",
        CylinderGeometry(0.055, 0.180, radial_segments=36, closed=True)
        .rotate_y(math.pi / 2.0)
        .translate(0.112, 0.0, 0.0),
    )

    frame = model.part("frame")
    frame.visual(
        floor_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=painted_metal,
        name="floor_plinth",
    )
    frame.visual(
        Box((1.18, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.78, 0.064)),
        material=elastomer,
        name="front_side_floor_grip",
    )
    frame.visual(
        Box((1.18, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, -0.78, 0.064)),
        material=elastomer,
        name="rear_side_floor_grip",
    )
    for side_index, y in enumerate((0.780, -0.780)):
        frame.visual(
            side_pod_mesh,
            origin=Origin(xyz=(0.0, y, 0.410)),
            material=polymer,
            name=f"side_pod_{side_index}",
        )
        frame.visual(
            Box((1.18, 0.018, 0.470)),
            origin=Origin(xyz=(0.0, math.copysign(0.858, y), 0.425)),
            material=smoked_polycarbonate,
            name=f"side_panel_{side_index}",
        )
        frame.visual(
            Box((1.30, 0.012, 0.030)),
            origin=Origin(xyz=(0.0, y * 0.992, 0.775)),
            material=seam_shadow,
            name=f"pod_top_seam_{side_index}",
        )
        frame.visual(
            rail_mesh,
            origin=Origin(xyz=(0.0, y, 0.885)),
            material=satin_metal,
            name=f"hand_rail_{side_index}",
        )
        for x in (-0.550, 0.550):
            frame.visual(
                Cylinder(radius=0.022, length=0.145),
                origin=Origin(xyz=(x, y, 0.812)),
                material=satin_metal,
                name=f"rail_stanchion_{side_index}_{'front' if x > 0 else 'rear'}",
            )

    frame.visual(post_mesh, origin=Origin(xyz=(0.0, 0.0, 0.000)), material=painted_metal, name="center_post")
    frame.visual(lower_trim_ring_mesh, origin=Origin(xyz=(0.0, 0.0, 0.155)), material=satin_metal, name="lower_trim_ring")
    frame.visual(upper_trim_ring_mesh, origin=Origin(xyz=(0.0, 0.0, 0.590)), material=satin_metal, name="upper_trim_ring")
    frame.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=bearing_steel,
        name="lower_bearing_collar",
    )
    frame.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.060)),
        material=bearing_steel,
        name="upper_bearing_collar",
    )
    frame.visual(
        Cylinder(radius=0.035, length=1.020),
        origin=Origin(xyz=(0.0, -0.760, 0.570)),
        material=painted_metal,
        name="rear_bearing_mast",
    )
    frame.visual(
        Box((0.090, 0.585, 0.065)),
        origin=Origin(xyz=(0.0, -0.462, 1.080)),
        material=painted_metal,
        name="upper_bearing_bridge",
    )
    frame.visual(
        Box((0.070, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, -0.735, 0.090)),
        material=painted_metal,
        name="mast_foot",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.62, 1.78, 1.12)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=bearing_steel,
        name="spindle",
    )
    rotor.visual(
        Cylinder(radius=0.160, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=bearing_steel,
        name="lower_rotating_race",
    )
    rotor.visual(
        Cylinder(radius=0.145, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=painted_metal,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.080, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.302)),
        material=satin_metal,
        name="top_retaining_cap",
    )
    for arm_index in range(3):
        yaw = 2.0 * math.pi * arm_index / 3.0
        rotor.visual(
            socket_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=painted_metal,
            name=f"arm_{arm_index}_socket",
        )
        rotor.visual(
            arm_tube_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=satin_metal,
            name=f"arm_{arm_index}_tube",
        )
        rotor.visual(
            arm_grip_mesh,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            material=elastomer,
            name=f"arm_{arm_index}_grip",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.40),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    model.articulation(
        "spindle_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.880)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spindle_rotation = object_model.get_articulation("spindle_rotation")

    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="spindle",
        outer_elem="upper_bearing_collar",
        margin=0.0,
        name="spindle is centered inside upper bearing collar",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="spindle",
        outer_elem="lower_bearing_collar",
        margin=0.0,
        name="spindle is centered inside lower bearing collar",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="lower_rotating_race",
        negative_elem="lower_bearing_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotating race bears on lower collar",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")
    rest_center_y = None
    if rest_aabb is not None:
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    with ctx.pose({spindle_rotation: math.pi / 3.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")
        turned_center_y = None
        if turned_aabb is not None:
            turned_center_y = (turned_aabb[0][1] + turned_aabb[1][1]) * 0.5
    ctx.check(
        "radial arm rotates about spindle axis",
        rest_center_y is not None
        and turned_center_y is not None
        and turned_center_y > rest_center_y + 0.18,
        details=f"rest_y={rest_center_y}, turned_y={turned_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
