from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


UPPER_CAGE_OFFSET = (1.18, 0.0, 0.0)

LOWER_TRUSS_POINTS = (
    (0.26, 0.24, 0.17),
    (0.26, 0.10, 0.24),
    (0.26, -0.10, 0.24),
    (0.26, -0.24, 0.17),
    (0.26, -0.19, -0.08),
    (0.26, 0.19, -0.08),
)

UPPER_TRUSS_POINTS_LOCAL = (
    (-0.127, 0.304, 0.172),
    (-0.127, 0.140, 0.321),
    (-0.127, -0.140, 0.321),
    (-0.127, -0.304, 0.172),
    (-0.127, -0.226, -0.266),
    (-0.127, 0.226, -0.266),
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (v[0] * scalar, v[1] * scalar, v[2] * scalar)


def _length(v: tuple[float, float, float]) -> float:
    return math.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = _length(v)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _rpy_from_z_axis(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    dx, dy, dz = _normalize(direction)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _lateral_axis(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    dx, dy, _ = _normalize(direction)
    lateral = (-dy, dx, 0.0)
    mag = math.hypot(lateral[0], lateral[1])
    return (lateral[0] / mag, lateral[1] / mag, 0.0)


def _upper_point_world(index: int) -> tuple[float, float, float]:
    return _add(UPPER_CAGE_OFFSET, UPPER_TRUSS_POINTS_LOCAL[index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_truss_dobsonian")

    plywood = model.material("plywood", rgba=(0.63, 0.49, 0.30, 1.0))
    ebony = model.material("ebony", rgba=(0.10, 0.08, 0.07, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.37, 0.39, 0.42, 1.0))
    teflon = model.material("teflon", rgba=(0.94, 0.94, 0.91, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.63, 0.72, 0.78, 1.0))

    ground_profile = [
        (0.52, 0.00),
        (0.18, 0.49),
        (-0.36, 0.39),
        (-0.36, -0.39),
        (0.18, -0.49),
    ]
    ground_board_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(ground_profile, 0.036, cap=True, closed=True),
        "ground_board_panel",
    )
    cage_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.35, segments=72),
            [_circle_profile(0.28, segments=72)],
            0.018,
            center=True,
            closed=True,
        ),
        "focuser_cage_ring",
    )
    ground_board = model.part("ground_board")
    ground_board.visual(ground_board_mesh, material=plywood, name="ground_board_panel")
    for index, foot_pos in enumerate(((0.42, 0.0, -0.012), (-0.16, 0.34, -0.012), (-0.16, -0.34, -0.012))):
        ground_board.visual(
            Cylinder(radius=0.048, length=0.024),
            origin=Origin(xyz=foot_pos),
            material=matte_black,
            name=f"foot_{index:02d}",
        )
    ground_board.visual(
        Cylinder(radius=0.14, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=ebony,
        name="azimuth_laminate",
    )
    ground_board.inertial = Inertial.from_geometry(
        Box((0.90, 0.98, 0.06)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        Cylinder(radius=0.33, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=ebony,
        name="azimuth_disk",
    )
    rocker_box.visual(
        Box((0.74, 0.808, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=plywood,
        name="rocker_floor",
    )
    rocker_box.visual(
        Box((0.62, 0.024, 0.64)),
        origin=Origin(xyz=(0.0, 0.392, 0.360)),
        material=plywood,
        name="rocker_left_wall",
    )
    rocker_box.visual(
        Box((0.62, 0.024, 0.64)),
        origin=Origin(xyz=(0.0, -0.392, 0.360)),
        material=plywood,
        name="rocker_right_wall",
    )
    rocker_box.visual(
        Box((0.018, 0.744, 0.050)),
        origin=Origin(xyz=(0.319, 0.0, 0.065)),
        material=plywood,
        name="front_brace",
    )
    rocker_box.visual(
        Box((0.018, 0.744, 0.050)),
        origin=Origin(xyz=(-0.319, 0.0, 0.065)),
        material=plywood,
        name="rear_brace",
    )
    rocker_box.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.70)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    mirror_assembly = model.part("mirror_assembly")
    mirror_assembly.visual(
        Box((0.52, 0.64, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.291)),
        material=plywood,
        name="mirror_box_floor",
    )
    mirror_assembly.visual(
        Box((0.52, 0.018, 0.56)),
        origin=Origin(xyz=(0.0, 0.311, -0.01)),
        material=plywood,
        name="mirror_box_side_left",
    )
    mirror_assembly.visual(
        Box((0.52, 0.018, 0.56)),
        origin=Origin(xyz=(0.0, -0.311, -0.01)),
        material=plywood,
        name="mirror_box_side_right",
    )
    mirror_assembly.visual(
        Box((0.018, 0.64, 0.50)),
        origin=Origin(xyz=(-0.251, 0.0, -0.04)),
        material=plywood,
        name="mirror_box_back",
    )
    mirror_assembly.visual(
        Box((0.018, 0.64, 0.18)),
        origin=Origin(xyz=(0.251, 0.0, -0.20)),
        material=plywood,
        name="mirror_box_front_lip",
    )
    mirror_assembly.visual(
        Box((0.08, 0.54, 0.03)),
        origin=Origin(xyz=(-0.166, 0.0, -0.17)),
        material=matte_black,
        name="cell_crossbar_horizontal",
    )
    mirror_assembly.visual(
        Box((0.152, 0.03, 0.48)),
        origin=Origin(xyz=(-0.166, 0.0, -0.04)),
        material=matte_black,
        name="cell_crossbar_vertical",
    )
    mirror_assembly.visual(
        Cylinder(radius=0.255, length=0.050),
        origin=Origin(xyz=(-0.08, 0.0, -0.04), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror_glass,
        name="primary_mirror",
    )
    mirror_assembly.visual(
        Cylinder(radius=0.40, length=0.028),
        origin=Origin(xyz=(0.0, 0.366, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plywood,
        name="trunnion_left",
    )
    mirror_assembly.visual(
        Cylinder(radius=0.40, length=0.028),
        origin=Origin(xyz=(0.0, -0.366, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plywood,
        name="trunnion_right",
    )
    mirror_assembly.visual(
        Box((0.12, 0.036, 0.20)),
        origin=Origin(xyz=(0.0, 0.337, 0.0)),
        material=plywood,
        name="left_trunnion_cheek",
    )
    mirror_assembly.visual(
        Box((0.12, 0.036, 0.20)),
        origin=Origin(xyz=(0.0, -0.337, 0.0)),
        material=plywood,
        name="right_trunnion_cheek",
    )
    mirror_assembly.visual(
        Box((0.018, 0.22, 0.48)),
        origin=Origin(xyz=(0.251, 0.250, 0.02)),
        material=plywood,
        name="upper_truss_post_left",
    )
    mirror_assembly.visual(
        Box((0.018, 0.22, 0.48)),
        origin=Origin(xyz=(0.251, -0.250, 0.02)),
        material=plywood,
        name="upper_truss_post_right",
    )
    mirror_assembly.visual(
        Box((0.018, 0.54, 0.08)),
        origin=Origin(xyz=(0.251, 0.0, 0.22)),
        material=plywood,
        name="upper_truss_header",
    )

    for index, lower_point in enumerate(LOWER_TRUSS_POINTS):
        upper_point = _upper_point_world(index)
        direction = _normalize(_sub(upper_point, lower_point))
        lateral = _lateral_axis(direction)
        socket_center = _sub(lower_point, _scale(direction, 0.0175))
        socket_rpy = _rpy_from_z_axis(direction)

        mirror_assembly.visual(
            Cylinder(radius=0.018, length=0.035),
            origin=Origin(xyz=socket_center, rpy=socket_rpy),
            material=steel,
            name=f"lower_socket_{index:02d}",
        )
        anchor_offset = 0.032 if index < 4 else -0.032
        anchor_center = _add(lower_point, _scale(lateral, anchor_offset))
        mirror_assembly.visual(
            Box((0.020, 0.030, 0.022)),
            origin=Origin(xyz=anchor_center, rpy=socket_rpy),
            material=matte_black,
            name=f"lower_anchor_{index:02d}",
        )

    mirror_assembly.inertial = Inertial.from_geometry(
        Box((0.80, 0.78, 0.82)),
        mass=28.0,
        origin=Origin(xyz=(-0.02, 0.0, -0.02)),
    )

    upper_cage = model.part("upper_cage")
    upper_cage.visual(
        cage_ring_mesh,
        origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plywood,
        name="lower_ring",
    )
    upper_cage.visual(
        cage_ring_mesh,
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plywood,
        name="upper_ring",
    )
    for index, strut_pos in enumerate(((0.0, 0.31, 0.0), (0.0, -0.31, 0.0), (0.0, 0.0, 0.31), (0.0, 0.0, -0.31))):
        upper_cage.visual(
            Box((0.20, 0.018, 0.018)),
            origin=Origin(xyz=strut_pos),
            material=matte_black,
            name=f"cage_standoff_{index:02d}",
        )
    upper_cage.visual(
        Box((0.12, 0.024, 0.18)),
        origin=Origin(xyz=(0.03, 0.319, 0.08)),
        material=matte_black,
        name="focuser_board",
    )
    upper_cage.visual(
        Cylinder(radius=0.026, length=0.12),
        origin=Origin(xyz=(0.04, 0.379, 0.09), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="focuser_drawtube",
    )
    upper_cage.visual(
        Cylinder(radius=0.022, length=0.08),
        origin=Origin(xyz=(0.03, 0.479, 0.12), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="eyepiece",
    )
    upper_cage.visual(
        Box((0.002, 0.62, 0.004)),
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
        material=steel,
        name="spider_vane_y",
    )
    upper_cage.visual(
        Box((0.002, 0.004, 0.62)),
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
        material=steel,
        name="spider_vane_z",
    )
    upper_cage.visual(
        Box((0.065, 0.038, 0.010)),
        origin=Origin(xyz=(0.03, 0.01, 0.02), rpy=(0.0, 0.65, math.pi / 4.0)),
        material=mirror_glass,
        name="secondary_mirror",
    )
    upper_cage.inertial = Inertial.from_geometry(
        Box((0.35, 0.72, 0.72)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    for index, lower_point in enumerate(LOWER_TRUSS_POINTS):
        upper_point = _upper_point_world(index)
        direction = _sub(upper_point, lower_point)
        length = _length(direction)
        rest_rpy = _rpy_from_z_axis(direction)

        pole = model.part(f"truss_pole_{index:02d}")
        pole.visual(
            Cylinder(radius=0.011, length=length),
            origin=Origin(xyz=(0.0, 0.0, length / 2.0)),
            material=aluminum,
            name="tube",
        )
        pole.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=steel,
            name="lower_ferrule",
        )
        pole.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, length - 0.025)),
            material=steel,
            name="upper_ferrule",
        )
        pole.inertial = Inertial.from_geometry(
            Cylinder(radius=0.015, length=length),
            mass=0.32,
            origin=Origin(xyz=(0.0, 0.0, length / 2.0)),
        )

        upper_socket = model.part(f"upper_socket_{index:02d}")
        upper_socket.visual(
            Cylinder(radius=0.015, length=0.002),
            origin=Origin(xyz=_scale(_normalize(direction), 0.001), rpy=rest_rpy),
            material=steel,
            name="receiver_tube",
        )
        upper_socket.visual(
            Box((0.018, 0.014, 0.014)),
            origin=Origin(xyz=(0.009, 0.0, 0.0)),
            material=matte_black,
            name="mount_block",
        )
        upper_socket.inertial = Inertial.from_geometry(
            Box((0.020, 0.016, 0.016)),
            mass=0.01,
            origin=Origin(xyz=(0.009, 0.0, 0.0)),
        )

        model.articulation(
            f"mirror_to_truss_pole_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=mirror_assembly,
            child=pole,
            origin=Origin(xyz=lower_point, rpy=rest_rpy),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.22, upper=0.22),
        )
        model.articulation(
            f"cage_to_upper_socket_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=upper_cage,
            child=upper_socket,
            origin=Origin(xyz=UPPER_TRUSS_POINTS_LOCAL[index]),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.22, upper=0.22),
        )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=ground_board,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=mirror_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-0.15, upper=1.45),
    )
    model.articulation(
        "mirror_to_upper_cage",
        ArticulationType.FIXED,
        parent=mirror_assembly,
        child=upper_cage,
        origin=Origin(xyz=UPPER_CAGE_OFFSET),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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
    ctx.fail_if_parts_overlap_in_current_pose()

    expected_part_names = [
        "ground_board",
        "rocker_box",
        "mirror_assembly",
        "upper_cage",
        *[f"truss_pole_{index:02d}" for index in range(6)],
        *[f"upper_socket_{index:02d}" for index in range(6)],
    ]
    parts = {name: object_model.get_part(name) for name in expected_part_names}
    for name in expected_part_names:
        ctx.check(f"part_present_{name}", True)

    azimuth = object_model.get_articulation("azimuth_rotation")
    altitude = object_model.get_articulation("altitude_axis")
    pole_joint_0 = object_model.get_articulation("mirror_to_truss_pole_00")
    socket_joint_0 = object_model.get_articulation("cage_to_upper_socket_00")

    ctx.check(
        "azimuth_axis_vertical",
        tuple(round(value, 6) for value in azimuth.axis) == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {azimuth.axis}",
    )
    ctx.check(
        "altitude_axis_crosswise",
        tuple(round(value, 6) for value in altitude.axis) == (0.0, -1.0, 0.0),
        f"expected negative-y altitude axis, got {altitude.axis}",
    )
    ctx.check(
        "truss_socket_limits_are_compact",
        pole_joint_0.motion_limits is not None
        and socket_joint_0.motion_limits is not None
        and pole_joint_0.motion_limits.lower == -0.22
        and pole_joint_0.motion_limits.upper == 0.22
        and socket_joint_0.motion_limits.lower == -0.22
        and socket_joint_0.motion_limits.upper == 0.22,
        "socket joints should have compact adjustment limits",
    )

    ground_board = parts["ground_board"]
    rocker_box = parts["rocker_box"]
    mirror_assembly = parts["mirror_assembly"]
    upper_cage = parts["upper_cage"]

    ctx.expect_contact(
        rocker_box,
        ground_board,
        elem_a=rocker_box.get_visual("azimuth_disk"),
        elem_b=ground_board.get_visual("ground_board_panel"),
        name="rocker_disk_contacts_ground_board",
    )
    ctx.expect_origin_distance(
        rocker_box,
        ground_board,
        axes="xy",
        max_dist=0.001,
        name="rocker_is_centered_on_ground_board",
    )
    ctx.expect_contact(
        mirror_assembly,
        rocker_box,
        elem_a=mirror_assembly.get_visual("trunnion_left"),
        elem_b=rocker_box.get_visual("rocker_left_wall"),
        name="left_trunnion_contacts_left_wall",
    )
    ctx.expect_contact(
        mirror_assembly,
        rocker_box,
        elem_a=mirror_assembly.get_visual("trunnion_right"),
        elem_b=rocker_box.get_visual("rocker_right_wall"),
        name="right_trunnion_contacts_right_wall",
    )
    ctx.expect_origin_distance(
        mirror_assembly,
        rocker_box,
        axes="y",
        max_dist=0.001,
        name="mirror_box_is_centered_between_walls",
    )
    ctx.expect_gap(
        mirror_assembly,
        ground_board,
        axis="z",
        min_gap=0.05,
        name="mirror_assembly_clears_ground_board",
    )
    ctx.expect_origin_gap(
        upper_cage,
        mirror_assembly,
        axis="x",
        min_gap=1.15,
        max_gap=1.21,
        name="upper_cage_sits_forward_of_mirror_box",
    )
    ctx.expect_origin_distance(
        upper_cage,
        mirror_assembly,
        axes="yz",
        max_dist=0.001,
        name="upper_cage_stays_on_optical_axis",
    )

    for index in range(6):
        pole = parts[f"truss_pole_{index:02d}"]
        upper_socket = parts[f"upper_socket_{index:02d}"]
        ctx.expect_contact(
            pole,
            mirror_assembly,
            elem_a=pole.get_visual("lower_ferrule"),
            elem_b=mirror_assembly.get_visual(f"lower_socket_{index:02d}"),
            name=f"truss_pole_{index:02d}_seats_in_lower_socket",
        )
        ctx.expect_contact(
            pole,
            upper_socket,
            elem_a=pole.get_visual("upper_ferrule"),
            elem_b=upper_socket.get_visual("receiver_tube"),
            name=f"truss_pole_{index:02d}_seats_in_upper_socket",
        )
        ctx.expect_contact(
            upper_socket,
            upper_cage,
            elem_a=upper_socket.get_visual("mount_block"),
            elem_b=upper_cage.get_visual("lower_ring"),
            name=f"upper_socket_{index:02d}_mounts_to_lower_ring",
        )

    with ctx.pose({altitude: 1.10}):
        ctx.expect_origin_gap(
            upper_cage,
            mirror_assembly,
            axis="z",
            min_gap=0.95,
            name="upper_cage_rises_at_high_altitude",
        )
        ctx.expect_gap(
            upper_cage,
            ground_board,
            axis="z",
            min_gap=0.55,
            name="elevated_cage_clears_ground_board",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
