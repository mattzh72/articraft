from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _tray_shell_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    outer_bottom = [
        (-0.23, -0.16, 0.22),
        (0.23, -0.16, 0.22),
        (0.23, 0.42, 0.22),
        (-0.23, 0.42, 0.22),
    ]
    outer_top = [
        (-0.31, -0.28, 0.46),
        (0.31, -0.28, 0.46),
        (0.31, 0.54, 0.46),
        (-0.31, 0.54, 0.46),
    ]
    inner_bottom = [
        (-0.215, -0.145, 0.238),
        (0.215, -0.145, 0.238),
        (0.215, 0.405, 0.238),
        (-0.215, 0.405, 0.238),
    ]
    inner_top = [
        (-0.292, -0.262, 0.444),
        (0.292, -0.262, 0.444),
        (0.292, 0.522, 0.444),
        (-0.292, 0.522, 0.444),
    ]

    ids = [geom.add_vertex(*point) for point in outer_bottom + outer_top + inner_bottom + inner_top]
    ob = ids[0:4]
    ot = ids[4:8]
    ib = ids[8:12]
    it = ids[12:16]

    _add_quad(geom, ob[0], ob[3], ob[2], ob[1])  # outer floor
    _add_quad(geom, ob[0], ob[1], ot[1], ot[0])  # rear outer wall
    _add_quad(geom, ob[1], ob[2], ot[2], ot[1])  # right outer wall
    _add_quad(geom, ob[2], ob[3], ot[3], ot[2])  # front outer wall
    _add_quad(geom, ob[3], ob[0], ot[0], ot[3])  # left outer wall

    _add_quad(geom, ib[0], ib[1], ib[2], ib[3])  # inner floor
    _add_quad(geom, ib[0], it[0], it[1], ib[1])  # rear inner wall
    _add_quad(geom, ib[1], it[1], it[2], ib[2])  # right inner wall
    _add_quad(geom, ib[2], it[2], it[3], ib[3])  # front inner wall
    _add_quad(geom, ib[3], it[3], it[0], ib[0])  # left inner wall

    _add_quad(geom, ob[0], ob[1], ib[1], ib[0])  # bottom rear thickness
    _add_quad(geom, ob[1], ob[2], ib[2], ib[1])  # bottom right thickness
    _add_quad(geom, ob[2], ob[3], ib[3], ib[2])  # bottom front thickness
    _add_quad(geom, ob[3], ob[0], ib[0], ib[3])  # bottom left thickness

    _add_quad(geom, ot[0], it[0], it[1], ot[1])  # rear rim
    _add_quad(geom, ot[1], it[1], it[2], ot[2])  # right rim
    _add_quad(geom, ot[2], it[2], it[3], ot[3])  # front rim
    _add_quad(geom, ot[3], it[3], it[0], ot[0])  # left rim

    return geom


def _wheel_tire_mesh(radius: float, width: float) -> object:
    half_width = width * 0.5
    profile = [
        (radius * 0.58, -half_width * 0.98),
        (radius * 0.78, -half_width * 0.96),
        (radius * 0.92, -half_width * 0.72),
        (radius, -half_width * 0.22),
        (radius, half_width * 0.22),
        (radius * 0.92, half_width * 0.72),
        (radius * 0.78, half_width * 0.96),
        (radius * 0.58, half_width * 0.98),
        (radius * 0.50, half_width * 0.45),
        (radius * 0.48, 0.0),
        (radius * 0.50, -half_width * 0.45),
        (radius * 0.58, -half_width * 0.98),
    ]
    return _mesh(
        "wheelbarrow_tire",
        LatheGeometry(profile, segments=56).rotate_y(pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_paint = model.material("tray_paint", rgba=(0.83, 0.34, 0.10, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    grip = model.material("grip", rgba=(0.18, 0.12, 0.08, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.70, 1.45, 0.78)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.02, 0.18)),
    )

    body.visual(_mesh("wheelbarrow_tray_shell", _tray_shell_mesh()), material=tray_paint, name="tray_shell")

    left_handle_path = [
        (-0.26, -0.67, 0.55),
        (-0.25, -0.30, 0.44),
        (-0.22, 0.02, 0.34),
        (-0.18, 0.14, 0.28),
        (-0.15, 0.19, 0.24),
    ]
    right_handle_path = _mirror_x(left_handle_path)
    body.visual(
        _mesh(
            "wheelbarrow_left_handle",
            tube_from_spline_points(left_handle_path, radius=0.018, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_steel,
        name="left_handle",
    )
    body.visual(
        _mesh(
            "wheelbarrow_right_handle",
            tube_from_spline_points(right_handle_path, radius=0.018, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_steel,
        name="right_handle",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(-0.26, -0.67, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="left_grip",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(0.26, -0.67, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="right_grip",
    )

    body.visual(
        Box((0.44, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, -0.16, 0.28)),
        material=frame_steel,
        name="rear_undertray_brace",
    )
    body.visual(
        Box((0.30, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.19, 0.24)),
        material=frame_steel,
        name="fork_bridge",
    )
    body.visual(
        Box((0.18, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, 0.11, 0.22)),
        material=frame_steel,
        name="front_support_block",
    )

    left_fork_path = [
        (-0.15, 0.19, 0.24),
        (-0.11, 0.12, 0.14),
        (-0.075, 0.03, 0.02),
    ]
    right_fork_path = _mirror_x(left_fork_path)
    body.visual(
        _mesh(
            "wheelbarrow_left_fork",
            tube_from_spline_points(left_fork_path, radius=0.016, samples_per_segment=12, radial_segments=16),
        ),
        material=frame_steel,
        name="left_fork",
    )
    body.visual(
        _mesh(
            "wheelbarrow_right_fork",
            tube_from_spline_points(right_fork_path, radius=0.016, samples_per_segment=12, radial_segments=16),
        ),
        material=frame_steel,
        name="right_fork",
    )
    body.visual(
        Box((0.011, 0.040, 0.080)),
        origin=Origin(xyz=(-0.065, 0.030, 0.0)),
        material=frame_steel,
        name="left_dropout",
    )
    body.visual(
        Box((0.011, 0.040, 0.080)),
        origin=Origin(xyz=(0.065, 0.030, 0.0)),
        material=frame_steel,
        name="right_dropout",
    )

    left_leg_path = [
        (-0.20, -0.13, 0.31),
        (-0.19, -0.26, 0.08),
        (-0.18, -0.39, -0.17),
    ]
    right_leg_path = _mirror_x(left_leg_path)
    body.visual(
        _mesh(
            "wheelbarrow_left_leg",
            tube_from_spline_points(left_leg_path, radius=0.016, samples_per_segment=12, radial_segments=16),
        ),
        material=frame_steel,
        name="left_leg",
    )
    body.visual(
        _mesh(
            "wheelbarrow_right_leg",
            tube_from_spline_points(right_leg_path, radius=0.016, samples_per_segment=12, radial_segments=16),
        ),
        material=frame_steel,
        name="right_leg",
    )
    body.visual(
        Box((0.06, 0.08, 0.02)),
        origin=Origin(xyz=(-0.18, -0.40, -0.1873)),
        material=frame_steel,
        name="left_foot",
    )
    body.visual(
        Box((0.06, 0.08, 0.02)),
        origin=Origin(xyz=(0.18, -0.40, -0.1873)),
        material=frame_steel,
        name="right_foot",
    )

    wheel = model.part("front_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.09),
        mass=3.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(_wheel_tire_mesh(0.19, 0.09), material=rubber, name="tire")
    wheel.visual(
        Cylinder(radius=0.132, length=0.012),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_rim",
    )
    wheel.visual(
        Cylinder(radius=0.132, length=0.012),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_rim",
    )
    wheel.visual(
        Cylinder(radius=0.055, length=0.119),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    for spoke_index, angle in enumerate((0.0, pi / 4.0, pi / 2.0, 3.0 * pi / 4.0), start=1):
        wheel.visual(
            Box((0.012, 0.18, 0.018)),
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=steel,
            name=f"spoke_{spoke_index}",
        )

    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("front_wheel_spin")

    ctx.expect_gap(
        body,
        wheel,
        axis="z",
        positive_elem="tray_shell",
        negative_elem="tire",
        min_gap=0.015,
        max_gap=0.12,
        name="tray clears tire",
    )
    ctx.expect_contact(
        body,
        wheel,
        elem_a="left_dropout",
        elem_b="hub",
        contact_tol=1e-5,
        name="left dropout supports hub",
    )
    ctx.expect_contact(
        body,
        wheel,
        elem_a="right_dropout",
        elem_b="hub",
        contact_tol=1e-5,
        name="right dropout supports hub",
    )

    tray_aabb = ctx.part_element_world_aabb(body, elem="tray_shell")
    wheel_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    left_fork_aabb = ctx.part_element_world_aabb(body, elem="left_fork")
    right_fork_aabb = ctx.part_element_world_aabb(body, elem="right_fork")
    left_foot_aabb = ctx.part_element_world_aabb(body, elem="left_foot")
    right_foot_aabb = ctx.part_element_world_aabb(body, elem="right_foot")
    wheel_pos = ctx.part_world_position(wheel)

    tray_center_y = None if tray_aabb is None else (tray_aabb[0][1] + tray_aabb[1][1]) * 0.5
    wheel_bottom = None if wheel_aabb is None else wheel_aabb[0][2]
    left_foot_bottom = None if left_foot_aabb is None else left_foot_aabb[0][2]
    right_foot_bottom = None if right_foot_aabb is None else right_foot_aabb[0][2]

    fork_clear = (
        wheel_aabb is not None
        and left_fork_aabb is not None
        and right_fork_aabb is not None
        and left_fork_aabb[1][0] < wheel_aabb[0][0] - 0.006
        and right_fork_aabb[0][0] > wheel_aabb[1][0] + 0.006
    )
    ctx.check(
        "wheel sits between fork blades",
        fork_clear,
        details=f"left_fork={left_fork_aabb}, wheel={wheel_aabb}, right_fork={right_fork_aabb}",
    )
    ctx.check(
        "upper support sits slightly forward of wheel center",
        tray_center_y is not None and wheel_pos is not None and tray_center_y > wheel_pos[1] + 0.05,
        details=f"tray_center_y={tray_center_y}, wheel_pos={wheel_pos}",
    )
    ctx.check(
        "rear resting legs reach the same support plane as the wheel",
        wheel_bottom is not None
        and left_foot_bottom is not None
        and right_foot_bottom is not None
        and abs(left_foot_bottom - wheel_bottom) <= 0.02
        and abs(right_foot_bottom - wheel_bottom) <= 0.02,
        details=(
            f"wheel_bottom={wheel_bottom}, left_foot_bottom={left_foot_bottom}, "
            f"right_foot_bottom={right_foot_bottom}"
        ),
    )
    ctx.check(
        "front wheel uses continuous axle rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
