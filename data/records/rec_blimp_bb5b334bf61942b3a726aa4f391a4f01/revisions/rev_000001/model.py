from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _elliptic_loft(
    sections: list[tuple[float, float, float, float]],
    *,
    segments: int = 64,
    exponent: float = 2.0,
) -> MeshGeometry:
    """Loft a closed superellipse body along X.

    Each section is (x, half_width_y, half_height_z, z_center).
    """
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, half_w, half_h, zc in sections:
        ring: list[int] = []
        for i in range(segments):
            t = 2.0 * math.pi * i / segments
            c = math.cos(t)
            s = math.sin(t)
            y = half_w * math.copysign(abs(c) ** (2.0 / exponent), c)
            z = zc + half_h * math.copysign(abs(s) ** (2.0 / exponent), s)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    first_center = geom.add_vertex(sections[0][0], 0.0, sections[0][3])
    last_center = geom.add_vertex(sections[-1][0], 0.0, sections[-1][3])
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(first_center, rings[0][j], rings[0][i])
        geom.add_face(last_center, rings[-1][i], rings[-1][j])
    return geom


def _prism_from_profile(
    points: list[tuple[float, float]],
    *,
    thickness: float,
    plane: str,
) -> MeshGeometry:
    """Extrude a 2D profile into a thin fin.

    plane="xz" extrudes through Y.  plane="xy" extrudes through Z.
    """
    geom = MeshGeometry()
    half = thickness / 2.0
    front: list[int] = []
    back: list[int] = []
    for a, b in points:
        if plane == "xz":
            front.append(geom.add_vertex(a, -half, b))
            back.append(geom.add_vertex(a, half, b))
        else:
            front.append(geom.add_vertex(a, b, -half))
            back.append(geom.add_vertex(a, b, half))

    n = len(points)
    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_blimp")

    envelope_mat = model.material("pearl_envelope", rgba=(0.92, 0.91, 0.84, 1.0))
    cabin_mat = model.material("brushed_cabin", rgba=(0.43, 0.48, 0.50, 1.0))
    glass_mat = model.material("smoked_window", rgba=(0.06, 0.12, 0.18, 0.92))
    strut_mat = model.material("dark_struts", rgba=(0.10, 0.11, 0.12, 1.0))
    pod_mat = model.material("silver_pod", rgba=(0.62, 0.65, 0.64, 1.0))
    prop_mat = model.material("matte_propeller", rgba=(0.03, 0.035, 0.04, 1.0))
    red_mat = model.material("tail_warning_red", rgba=(0.62, 0.06, 0.04, 1.0))

    hull = model.part("hull")

    envelope = _elliptic_loft(
        [
            (-36.0, 0.10, 0.10, 14.0),
            (-34.0, 2.10, 2.00, 14.05),
            (-29.0, 5.90, 5.70, 14.15),
            (-20.0, 8.30, 8.10, 14.20),
            (-8.0, 9.20, 9.00, 14.15),
            (6.0, 9.00, 8.80, 14.05),
            (18.0, 7.50, 7.25, 14.00),
            (28.0, 5.10, 5.00, 13.98),
            (34.0, 1.80, 1.75, 13.96),
            (36.0, 0.08, 0.08, 13.95),
        ],
        segments=72,
        exponent=2.08,
    )
    hull.visual(
        mesh_from_geometry(envelope, "streamlined_envelope"),
        material=envelope_mat,
        name="envelope",
    )

    cabin_body = _elliptic_loft(
        [
            (-17.5, 0.12, 0.10, 3.20),
            (-16.0, 1.55, 1.05, 3.25),
            (-13.0, 2.45, 1.65, 3.30),
            (10.5, 2.55, 1.70, 3.30),
            (13.8, 1.55, 1.10, 3.25),
            (15.5, 0.12, 0.10, 3.15),
        ],
        segments=48,
        exponent=3.2,
    )
    hull.visual(
        mesh_from_geometry(cabin_body, "long_passenger_cabin"),
        material=cabin_mat,
        name="cabin_body",
    )

    # Surface-mounted window strips, slightly embedded into the cabin skin.
    for side, y in (("left", 2.46), ("right", -2.46)):
        for i, x in enumerate([-12.5, -9.8, -7.1, -4.4, -1.7, 1.0, 3.7, 6.4, 9.1]):
            hull.visual(
                Box((1.25, 0.22, 0.55)),
                origin=Origin(xyz=(x, y, 3.65)),
                material=glass_mat,
                name=f"{side}_window_{i}",
            )

    # Substantial pylons visually and physically tie the cabin to the envelope.
    for x in (-12.0, -4.0, 4.0, 12.0):
        for y in (-1.45, 1.45):
            hull.visual(
                Box((0.55, 0.42, 2.15)),
                origin=Origin(xyz=(x, y, 5.25)),
                material=strut_mat,
                name=f"cabin_pylon_{x}_{y}",
            )
    hull.visual(
        Box((27.0, 0.60, 0.48)),
        origin=Origin(xyz=(-1.0, 0.0, 4.95)),
        material=strut_mat,
        name="cabin_keel_saddle",
    )

    # Side propeller pods and their structural booms are fixed to the hull.
    pod_mesh = mesh_from_geometry(CapsuleGeometry(0.76, 0.30, radial_segments=36), "side_pod_capsule")
    for side, sign in (("left", 1.0), ("right", -1.0)):
        hull.visual(
            Box((5.0, 2.55, 0.38)),
            origin=Origin(xyz=(-2.0, sign * 3.72, 4.10)),
            material=strut_mat,
            name=f"{side}_pod_boom",
        )
        hull.visual(
            pod_mesh,
            origin=Origin(xyz=(-2.0, sign * 5.70, 4.10), rpy=(-sign * math.pi / 2.0, 0.0, 0.0)),
            material=pod_mat,
            name=f"{side}_pod",
        )
        hull.visual(
            Cylinder(radius=0.18, length=0.45),
            origin=Origin(xyz=(-2.0, sign * 6.68, 4.10), rpy=(-sign * math.pi / 2.0, 0.0, 0.0)),
            material=strut_mat,
            name=f"{side}_prop_shaft",
        )

    # Balanced cruciform fixed tail surfaces: one vertical pair and one horizontal pair.
    hinge_x = 31.5
    top_fin = _prism_from_profile(
        [(22.0, 18.2), (24.5, 23.9), (hinge_x, 22.5), (hinge_x, 19.35)],
        thickness=0.62,
        plane="xz",
    )
    bottom_fin = _prism_from_profile(
        [(22.0, 9.8), (hinge_x, 8.65), (hinge_x, 5.5), (24.5, 4.1)],
        thickness=0.62,
        plane="xz",
    )
    left_fin = _prism_from_profile(
        [(22.0, 4.45), (24.5, 9.35), (hinge_x, 8.30), (hinge_x, 4.75)],
        thickness=0.62,
        plane="xy",
    ).translate(0.0, 0.0, 14.0)
    right_fin = _prism_from_profile(
        [(22.0, -4.45), (hinge_x, -4.75), (hinge_x, -8.30), (24.5, -9.35)],
        thickness=0.62,
        plane="xy",
    ).translate(0.0, 0.0, 14.0)
    for geom, name in (
        (top_fin, "top_fixed_fin"),
        (bottom_fin, "bottom_fixed_fin"),
        (left_fin, "left_fixed_fin"),
        (right_fin, "right_fixed_fin"),
    ):
        hull.visual(
            mesh_from_geometry(geom, name),
            material=envelope_mat,
            name=name,
        )

    # Red tail tips make the four moving surfaces easy to read while preserving symmetry.
    rudder_top = model.part("top_rudder")
    rudder_top.visual(
        Box((3.60, 0.42, 4.20)),
        origin=Origin(xyz=(1.80, 0.0, 0.0)),
        material=envelope_mat,
        name="rudder_panel",
    )
    rudder_top.visual(
        Box((0.18, 0.46, 4.20)),
        origin=Origin(xyz=(3.51, 0.0, 0.0)),
        material=red_mat,
        name="rudder_tip",
    )

    rudder_bottom = model.part("bottom_rudder")
    rudder_bottom.visual(
        Box((3.60, 0.42, 4.20)),
        origin=Origin(xyz=(1.80, 0.0, 0.0)),
        material=envelope_mat,
        name="rudder_panel",
    )
    rudder_bottom.visual(
        Box((0.18, 0.46, 4.20)),
        origin=Origin(xyz=(3.51, 0.0, 0.0)),
        material=red_mat,
        name="rudder_tip",
    )

    elevator_left = model.part("left_elevator")
    elevator_left.visual(
        Box((3.60, 3.55, 0.42)),
        origin=Origin(xyz=(1.80, 0.0, 0.0)),
        material=envelope_mat,
        name="elevator_panel",
    )
    elevator_left.visual(
        Box((0.18, 3.55, 0.46)),
        origin=Origin(xyz=(3.51, 0.0, 0.0)),
        material=red_mat,
        name="elevator_tip",
    )

    elevator_right = model.part("right_elevator")
    elevator_right.visual(
        Box((3.60, 3.55, 0.42)),
        origin=Origin(xyz=(1.80, 0.0, 0.0)),
        material=envelope_mat,
        name="elevator_panel",
    )
    elevator_right.visual(
        Box((0.18, 3.55, 0.46)),
        origin=Origin(xyz=(3.51, 0.0, 0.0)),
        material=red_mat,
        name="elevator_tip",
    )

    rotor_geometry = FanRotorGeometry(
        1.28,
        0.28,
        7,
        thickness=0.35,
        blade_pitch_deg=34.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.18),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.14, rear_collar_radius=0.22),
    )
    rotor_mesh = mesh_from_geometry(rotor_geometry, "seven_blade_propeller")

    left_propeller = model.part("left_propeller")
    left_propeller.visual(rotor_mesh, material=prop_mat, name="rotor")
    left_propeller.visual(
        Cylinder(radius=0.24, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=prop_mat,
        name="hub_socket",
    )
    right_propeller = model.part("right_propeller")
    right_propeller.visual(rotor_mesh, material=prop_mat, name="rotor")
    right_propeller.visual(
        Cylinder(radius=0.24, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=prop_mat,
        name="hub_socket",
    )

    prop_limits = MotionLimits(effort=250.0, velocity=80.0)
    model.articulation(
        "left_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=left_propeller,
        origin=Origin(xyz=(-2.0, 7.075, 4.10), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=prop_limits,
    )
    model.articulation(
        "right_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=right_propeller,
        origin=Origin(xyz=(-2.0, -7.075, 4.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=prop_limits,
    )

    tail_limits = MotionLimits(effort=120.0, velocity=2.5, lower=-0.45, upper=0.45)
    model.articulation(
        "top_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder_top,
        origin=Origin(xyz=(hinge_x, 0.0, 20.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=tail_limits,
    )
    model.articulation(
        "bottom_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder_bottom,
        origin=Origin(xyz=(hinge_x, 0.0, 7.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=tail_limits,
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=elevator_left,
        origin=Origin(xyz=(hinge_x, 6.55, 14.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=tail_limits,
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=elevator_right,
        origin=Origin(xyz=(hinge_x, -6.55, 14.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=tail_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hull = object_model.get_part("hull")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    top_rudder = object_model.get_part("top_rudder")
    left_elevator = object_model.get_part("left_elevator")

    def _center_y(part_name: str) -> float:
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        assert aabb is not None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    def _center_z(part_name: str) -> float:
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        assert aabb is not None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    envelope_aabb = ctx.part_element_world_aabb(hull, elem="envelope")
    cabin_aabb = ctx.part_element_world_aabb(hull, elem="cabin_body")
    ctx.check(
        "large streamlined envelope scale",
        envelope_aabb is not None
        and envelope_aabb[1][0] - envelope_aabb[0][0] > 70.0
        and envelope_aabb[1][2] - envelope_aabb[0][2] > 17.0,
        details=f"envelope_aabb={envelope_aabb}",
    )
    ctx.check(
        "substantial under-slung passenger cabin",
        cabin_aabb is not None
        and cabin_aabb[1][0] - cabin_aabb[0][0] > 30.0
        and cabin_aabb[0][2] < envelope_aabb[0][2],
        details=f"cabin_aabb={cabin_aabb}, envelope_aabb={envelope_aabb}",
    )

    for joint_name in ("left_propeller_spin", "right_propeller_spin"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    for joint_name in (
        "top_rudder_hinge",
        "bottom_rudder_hinge",
        "left_elevator_hinge",
        "right_elevator_hinge",
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is a limited tail hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower < 0.0
            and limits.upper > 0.0,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    ctx.expect_contact(
        left_propeller,
        hull,
        elem_a="hub_socket",
        elem_b="left_prop_shaft",
        contact_tol=0.02,
        name="left propeller mounted on shaft",
    )
    ctx.expect_contact(
        right_propeller,
        hull,
        elem_a="hub_socket",
        elem_b="right_prop_shaft",
        contact_tol=0.02,
        name="right propeller mounted on shaft",
    )
    ctx.expect_gap(
        top_rudder,
        hull,
        axis="x",
        positive_elem="rudder_panel",
        negative_elem="top_fixed_fin",
        max_gap=0.001,
        max_penetration=1e-5,
        name="top rudder sits on tail trailing hinge",
    )
    ctx.expect_gap(
        left_elevator,
        hull,
        axis="x",
        positive_elem="elevator_panel",
        negative_elem="left_fixed_fin",
        max_gap=0.001,
        max_penetration=1e-5,
        name="left elevator sits on tail trailing hinge",
    )

    rest_rudder_y = _center_y("top_rudder")
    rest_elevator_z = _center_z("left_elevator")
    with ctx.pose({"top_rudder_hinge": 0.35, "left_elevator_hinge": 0.35}):
        turned_rudder_y = _center_y("top_rudder")
        lifted_elevator_z = _center_z("left_elevator")
    ctx.check(
        "rudder hinge deflects laterally",
        turned_rudder_y > rest_rudder_y + 0.25,
        details=f"rest_y={rest_rudder_y}, turned_y={turned_rudder_y}",
    )
    ctx.check(
        "elevator hinge deflects vertically",
        lifted_elevator_z > rest_elevator_z + 0.25,
        details=f"rest_z={rest_elevator_z}, lifted_z={lifted_elevator_z}",
    )

    return ctx.report()


object_model = build_object_model()
