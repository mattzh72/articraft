from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_JOINT = (0.150, 0.148)

LINK1_DX = 0.255
LINK1_DZ = 0.055
LINK2_DX = 0.210
LINK2_DZ = -0.030
LINK3_DX = 0.170
LINK3_DZ = 0.022

LINK1_ROOT_THK = 0.030
LINK1_WEB_THK = 0.024
LINK1_EAR_THK = 0.010

LINK2_ROOT_THK = 0.022
LINK2_WEB_THK = 0.020
LINK2_EAR_THK = 0.009

LINK3_ROOT_THK = 0.016
LINK3_WEB_THK = 0.016

BASE_EAR_THK = 0.016


def _point_on_segment(start: tuple[float, float], end: tuple[float, float], t: float) -> tuple[float, float]:
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
    )


def _profile_between(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    widths: tuple[float, ...],
    fractions: tuple[float, ...],
    biases: tuple[float, ...] | None = None,
) -> list[tuple[float, float]]:
    if len(widths) != len(fractions):
        raise ValueError("widths and fractions must have the same length")
    if biases is None:
        biases = tuple(0.0 for _ in widths)
    if len(biases) != len(widths):
        raise ValueError("biases and widths must have the same length")

    vx = end[0] - start[0]
    vz = end[1] - start[1]
    length = sqrt(vx * vx + vz * vz)
    if length <= 0.0:
        raise ValueError("profile segment length must be positive")

    ux = vx / length
    uz = vz / length
    px = -uz
    pz = ux

    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []
    for frac, width, bias in zip(fractions, widths, biases):
        cx = start[0] + vx * frac + px * bias
        cz = start[1] + vz * frac + pz * bias
        upper.append((cx + px * width, cz + pz * width))
        lower.append((cx - px * width, cz - pz * width))
    return upper + list(reversed(lower))


def _plate_from_profile(
    profile: list[tuple[float, float]],
    *,
    thickness: float,
    y_center: float = 0.0,
):
    solid = cq.Workplane("XZ").polyline(profile).close().extrude(thickness / 2.0, both=True)
    if abs(y_center) > 1e-9:
        solid = solid.translate((0.0, y_center, 0.0))
    return solid


def _y_cylinder(
    x: float,
    z: float,
    *,
    radius: float,
    thickness: float,
    y_center: float = 0.0,
):
    solid = cq.Workplane("XZ").center(x, z).circle(radius).extrude(thickness / 2.0, both=True)
    if abs(y_center) > 1e-9:
        solid = solid.translate((0.0, y_center, 0.0))
    return solid


def _stepped_joint_boss(
    x: float,
    z: float,
    *,
    core_radius: float,
    core_thickness: float,
    collar_radius: float,
    collar_thickness: float,
    contact_half_span: float,
):
    core = _y_cylinder(x, z, radius=core_radius, thickness=core_thickness)
    collar_center = contact_half_span - collar_thickness / 2.0
    upper = _y_cylinder(x, z, radius=collar_radius, thickness=collar_thickness, y_center=collar_center)
    lower = _y_cylinder(x, z, radius=collar_radius, thickness=collar_thickness, y_center=-collar_center)
    return _union_all(core, upper, lower)


def _union_all(*shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _build_base_shape():
    foot = cq.Workplane("XY").box(0.340, 0.180, 0.028).translate((0.0, 0.0, 0.014))
    heel = cq.Workplane("XY").box(0.126, 0.140, 0.072).translate((-0.092, 0.0, 0.050))
    toe_pad = cq.Workplane("XY").box(0.086, 0.112, 0.012).translate((0.086, 0.0, 0.006))

    rear_wall_profile = [
        (-0.122, 0.028),
        (-0.122, 0.112),
        (-0.066, 0.126),
        (-0.024, 0.112),
        (-0.014, 0.028),
    ]
    rear_wall = _plate_from_profile(rear_wall_profile, thickness=0.048)

    ear_y = 0.018
    ear_radius = 0.026
    rib_profile = _profile_between(
        (-0.024, 0.118),
        BASE_JOINT,
        widths=(0.018, 0.016, 0.014, 0.012),
        fractions=(0.0, 0.34, 0.70, 1.0),
        biases=(0.0, 0.004, 0.003, 0.0),
    )
    brace_profile = _profile_between(
        (-0.056, 0.108),
        (-0.004, 0.134),
        widths=(0.012, 0.013, 0.011),
        fractions=(0.0, 0.50, 1.0),
        biases=(0.0, 0.002, 0.0),
    )
    upper_rib = _plate_from_profile(rib_profile, thickness=BASE_EAR_THK, y_center=ear_y)
    lower_rib = _plate_from_profile(rib_profile, thickness=BASE_EAR_THK, y_center=-ear_y)
    upper_brace = _plate_from_profile(brace_profile, thickness=BASE_EAR_THK, y_center=ear_y)
    lower_brace = _plate_from_profile(brace_profile, thickness=BASE_EAR_THK, y_center=-ear_y)
    root_upper_ear = _y_cylinder(BASE_JOINT[0], BASE_JOINT[1], radius=ear_radius, thickness=BASE_EAR_THK, y_center=ear_y)
    root_lower_ear = _y_cylinder(BASE_JOINT[0], BASE_JOINT[1], radius=ear_radius, thickness=BASE_EAR_THK, y_center=-ear_y)

    return _union_all(
        foot,
        heel,
        toe_pad,
        rear_wall,
        upper_rib,
        lower_rib,
        upper_brace,
        lower_brace,
        root_upper_ear,
        root_lower_ear,
    )


def _build_link1_shape():
    tip = (LINK1_DX, LINK1_DZ)
    main_end = _point_on_segment((0.0, 0.0), tip, 0.70)
    fork_start = _point_on_segment((0.0, 0.0), tip, 0.66)

    root_lug = _y_cylinder(0.0, 0.0, radius=0.021, thickness=0.020)
    main_web = _plate_from_profile(
        _profile_between(
            (0.008, 0.000),
            main_end,
            widths=(0.018, 0.016, 0.013, 0.010),
            fractions=(0.0, 0.28, 0.62, 1.0),
            biases=(0.0, 0.004, -0.003, 0.001),
        ),
        thickness=0.018,
    )

    ear_y = 0.016
    fork_profile = _profile_between(
        fork_start,
        tip,
        widths=(0.010, 0.011, 0.010),
        fractions=(0.0, 0.55, 1.0),
        biases=(0.001, 0.003, 0.0),
    )
    connector_profile = _profile_between(
        _point_on_segment((0.0, 0.0), tip, 0.52),
        _point_on_segment((0.0, 0.0), tip, 0.68),
        widths=(0.008, 0.010, 0.009),
        fractions=(0.0, 0.48, 1.0),
        biases=(0.0, 0.002, 0.0),
    )
    upper_rib = _plate_from_profile(fork_profile, thickness=0.010, y_center=ear_y)
    lower_rib = _plate_from_profile(fork_profile, thickness=0.010, y_center=-ear_y)
    upper_connector = _plate_from_profile(connector_profile, thickness=0.004, y_center=0.010)
    lower_connector = _plate_from_profile(connector_profile, thickness=0.004, y_center=-0.010)
    upper_ear = _y_cylinder(tip[0], tip[1], radius=0.021, thickness=0.010, y_center=ear_y)
    lower_ear = _y_cylinder(tip[0], tip[1], radius=0.021, thickness=0.010, y_center=-ear_y)

    return _union_all(
        root_lug,
        main_web,
        upper_rib,
        lower_rib,
        upper_connector,
        lower_connector,
        upper_ear,
        lower_ear,
    )


def _build_link2_shape():
    tip = (LINK2_DX, LINK2_DZ)
    main_end = _point_on_segment((0.0, 0.0), tip, 0.66)
    fork_start = _point_on_segment((0.0, 0.0), tip, 0.64)

    root_lug = _y_cylinder(0.0, 0.0, radius=0.018, thickness=0.022)
    main_web = _plate_from_profile(
        _profile_between(
            (0.008, 0.000),
            main_end,
            widths=(0.015, 0.013, 0.011, 0.009),
            fractions=(0.0, 0.30, 0.65, 1.0),
            biases=(0.0, -0.003, 0.002, -0.001),
        ),
        thickness=0.016,
    )

    ear_y = 0.012
    fork_profile = _profile_between(
        fork_start,
        tip,
        widths=(0.008, 0.009, 0.008),
        fractions=(0.0, 0.52, 1.0),
        biases=(-0.001, -0.002, 0.0),
    )
    connector_profile = _profile_between(
        _point_on_segment((0.0, 0.0), tip, 0.48),
        _point_on_segment((0.0, 0.0), tip, 0.66),
        widths=(0.0065, 0.008, 0.0075),
        fractions=(0.0, 0.46, 1.0),
        biases=(0.0, -0.001, 0.0),
    )
    upper_rib = _plate_from_profile(fork_profile, thickness=0.008, y_center=ear_y)
    lower_rib = _plate_from_profile(fork_profile, thickness=0.008, y_center=-ear_y)
    upper_connector = _plate_from_profile(connector_profile, thickness=0.004, y_center=0.009)
    lower_connector = _plate_from_profile(connector_profile, thickness=0.004, y_center=-0.009)
    upper_ear = _y_cylinder(tip[0], tip[1], radius=0.018, thickness=0.008, y_center=ear_y)
    lower_ear = _y_cylinder(tip[0], tip[1], radius=0.018, thickness=0.008, y_center=-ear_y)

    return _union_all(
        root_lug,
        main_web,
        upper_rib,
        lower_rib,
        upper_connector,
        lower_connector,
        upper_ear,
        lower_ear,
    )


def _build_link3_shape():
    tip = (LINK3_DX, LINK3_DZ)

    root_boss = _y_cylinder(0.0, 0.0, radius=0.016, thickness=0.016)
    body = _plate_from_profile(
        _profile_between(
            (0.008, 0.0),
            tip,
            widths=(0.012, 0.010, 0.008, 0.007),
            fractions=(0.0, 0.26, 0.68, 1.0),
            biases=(0.0, 0.003, -0.001, 0.0),
        ),
        thickness=0.012,
    )
    tip_pad = _y_cylinder(tip[0], tip[1], radius=0.014, thickness=0.012)

    return _union_all(root_boss, body, tip_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_lever_chain")

    model.material("base_steel", rgba=(0.21, 0.23, 0.25, 1.0))
    model.material("link_root", rgba=(0.52, 0.55, 0.58, 1.0))
    model.material("link_mid", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("link_tip", rgba=(0.72, 0.74, 0.78, 1.0))

    base = model.part("base_foot")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_foot"),
        name="base_body",
        material="base_steel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.180, 0.160)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    link1 = model.part("rocker_link_1")
    link1.visual(
        mesh_from_cadquery(_build_link1_shape(), "rocker_link_1"),
        name="link1_body",
        material="link_root",
    )
    link1.inertial = Inertial.from_geometry(
        Box((0.260, 0.050, 0.090)),
        mass=1.2,
        origin=Origin(xyz=(0.130, 0.0, 0.028)),
    )

    link2 = model.part("rocker_link_2")
    link2.visual(
        mesh_from_cadquery(_build_link2_shape(), "rocker_link_2"),
        name="link2_body",
        material="link_mid",
    )
    link2.inertial = Inertial.from_geometry(
        Box((0.220, 0.042, 0.070)),
        mass=0.82,
        origin=Origin(xyz=(0.106, 0.0, -0.012)),
    )

    link3 = model.part("rocker_link_3")
    link3.visual(
        mesh_from_cadquery(_build_link3_shape(), "rocker_link_3"),
        name="link3_body",
        material="link_tip",
    )
    link3.inertial = Inertial.from_geometry(
        Box((0.175, 0.018, 0.050)),
        mass=0.42,
        origin=Origin(xyz=(0.085, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(BASE_JOINT[0], 0.0, BASE_JOINT[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.05, effort=80.0, velocity=1.4),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_DX, 0.0, LINK1_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=1.15, effort=55.0, velocity=1.6),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_DX, 0.0, LINK2_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.95, upper=1.10, effort=32.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_foot")
    link1 = object_model.get_part("rocker_link_1")
    link2 = object_model.get_part("rocker_link_2")
    link3 = object_model.get_part("rocker_link_3")
    joint1 = object_model.get_articulation("base_to_link1")
    joint2 = object_model.get_articulation("link1_to_link2")
    joint3 = object_model.get_articulation("link2_to_link3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        base,
        link1,
        reason="Simplified clevis-and-lug root joint shares nominal pin-envelope material because the pin itself is not authored as a separate part.",
    )
    ctx.allow_overlap(
        link1,
        link2,
        reason="Interleaved mid-joint cheeks are represented without explicit through-holes or pin hardware, so the hinge barrel envelope is intentionally shared.",
    )
    ctx.allow_overlap(
        link2,
        link3,
        reason="Tip revolute joint uses the same simplified shared pin-envelope representation instead of separate pin and bushing parts.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (base, link1, link2, link3)),
        "Expected base foot and three serial rocker links.",
    )

    ctx.expect_contact(base, link1, name="base_joint_contact")
    ctx.expect_contact(link1, link2, name="middle_joint_contact")
    ctx.expect_contact(link2, link3, name="tip_joint_contact")

    with ctx.pose({joint1: 0.45, joint2: -0.55, joint3: 0.50}):
        ctx.expect_contact(base, link1, name="base_joint_contact_in_bent_pose")
        ctx.expect_contact(link1, link2, name="middle_joint_contact_in_bent_pose")
        ctx.expect_contact(link2, link3, name="tip_joint_contact_in_bent_pose")

    link2_closed = ctx.part_world_position(link2)
    with ctx.pose({joint1: 0.55}):
        link2_lifted = ctx.part_world_position(link2)
    ctx.check(
        "joint1_positive_raises_second_axis",
        bool(link2_closed and link2_lifted and link2_lifted[2] > link2_closed[2] + 0.05),
        f"Expected link2 root z to rise under positive base_to_link1 motion: closed={link2_closed}, lifted={link2_lifted}",
    )

    link3_closed = ctx.part_world_position(link3)
    with ctx.pose({joint2: 0.70}):
        link3_lifted = ctx.part_world_position(link3)
    ctx.check(
        "joint2_positive_raises_third_axis",
        bool(link3_closed and link3_lifted and link3_lifted[2] > link3_closed[2] + 0.035),
        f"Expected link3 root z to rise under positive link1_to_link2 motion: closed={link3_closed}, lifted={link3_lifted}",
    )

    def _elem_center_z(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    link3_body_z_closed = _elem_center_z("rocker_link_3", "link3_body")
    with ctx.pose({joint3: 0.72}):
        link3_body_z_open = _elem_center_z("rocker_link_3", "link3_body")
    ctx.check(
        "joint3_positive_raises_tip_link_body",
        bool(
            link3_body_z_closed is not None
            and link3_body_z_open is not None
            and link3_body_z_open > link3_body_z_closed + 0.015
        ),
        f"Expected link3 body center z to rise under positive link2_to_link3 motion: closed={link3_body_z_closed}, open={link3_body_z_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
