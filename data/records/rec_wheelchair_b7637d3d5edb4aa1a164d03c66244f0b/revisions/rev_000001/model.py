from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _ring_profile(outer_radius: float, inner_radius: float, width: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    return [
        (outer_radius, -half_width),
        (outer_radius, half_width),
        (inner_radius, half_width),
        (inner_radius, -half_width),
        (outer_radius, -half_width),
    ]


def _add_rod(part, mesh_name: str, visual_name: str, start, end, *, radius: float, material) -> None:
    part.visual(
        _mesh(
            mesh_name,
            tube_from_spline_points(
                [start, end],
                radius=radius,
                samples_per_segment=2,
                radial_segments=12,
            ),
        ),
        material=material,
        name=visual_name,
    )


def _add_rear_wheel_geometry(part, *, mesh_prefix: str, side_sign: float, tire_mat, rim_mat, hub_mat) -> None:
    wheel_axis_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    tire = LatheGeometry(_ring_profile(0.305, 0.255, 0.028), segments=72).rotate_x(-pi / 2.0)
    rim = LatheGeometry(_ring_profile(0.258, 0.238, 0.018), segments=72).rotate_x(-pi / 2.0)
    handrim = LatheGeometry(_ring_profile(0.274, 0.269, 0.008), segments=72).rotate_x(-pi / 2.0)
    part.visual(_mesh(f"{mesh_prefix}_tire", tire), material=tire_mat, name="tire")
    part.visual(_mesh(f"{mesh_prefix}_rim", rim), material=rim_mat, name="rim")
    part.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=wheel_axis_origin,
        material=hub_mat,
        name="hub",
    )
    for index in range(12):
        angle = 2.0 * pi * index / 12.0
        _add_rod(
            part,
            f"{mesh_prefix}_spoke_{index}",
            f"spoke_{index}",
            (0.032 * sin(angle), 0.0, 0.032 * cos(angle)),
            (0.248 * sin(angle), 0.0, 0.248 * cos(angle)),
            radius=0.0034,
            material=rim_mat,
        )
    part.visual(
        _mesh(f"{mesh_prefix}_handrim", handrim),
        origin=Origin(xyz=(0.0, 0.028 * side_sign, 0.0)),
        material=hub_mat,
        name="handrim",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        _add_rod(
            part,
            f"{mesh_prefix}_handrim_bracket_{index}",
            f"connector_{index}",
            (0.250 * sin(angle), 0.0, 0.250 * cos(angle)),
            (0.270 * sin(angle), 0.028 * side_sign, 0.270 * cos(angle)),
            radius=0.0030,
            material=hub_mat,
        )


def _add_caster_fork_geometry(part, *, metal_mat) -> None:
    part.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=metal_mat,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=metal_mat,
        name="bearing_housing",
    )
    part.visual(
        Box((0.084, 0.064, 0.012)),
        origin=Origin(xyz=(-0.036, 0.0, -0.025)),
        material=metal_mat,
        name="crown",
    )
    for side_sign in (1.0, -1.0):
        part.visual(
            Box((0.014, 0.010, 0.148)),
            origin=Origin(xyz=(-0.072, 0.031 * side_sign, -0.105)),
            material=metal_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_leg",
        )


def _add_caster_wheel_geometry(part, *, mesh_prefix: str, tire_mat, rim_mat, hub_mat) -> None:
    wheel_axis_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    tire = LatheGeometry(_ring_profile(0.080, 0.055, 0.032), segments=48).rotate_x(-pi / 2.0)
    rim = LatheGeometry(_ring_profile(0.058, 0.038, 0.024), segments=48).rotate_x(-pi / 2.0)
    part.visual(_mesh(f"{mesh_prefix}_tire", tire), material=tire_mat, name="tire")
    part.visual(_mesh(f"{mesh_prefix}_rim", rim), material=rim_mat, name="rim")
    part.visual(
        Cylinder(radius=0.005, length=0.052),
        origin=wheel_axis_origin,
        material=hub_mat,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=wheel_axis_origin,
        material=hub_mat,
        name="hub",
    )
    for index, angle in enumerate((0.0, pi / 2.0)):
        _add_rod(
            part,
            f"{mesh_prefix}_spoke_{index}",
            f"spoke_{index}",
            (0.012 * sin(angle), 0.0, 0.012 * cos(angle)),
            (0.055 * sin(angle), 0.0, 0.055 * cos(angle)),
            radius=0.0028,
            material=hub_mat,
        )


def _add_frame_geometry(part, frame_mat, upholstery_mat, plate_mat) -> None:
    tube_radius = 0.018
    side_y = 0.235
    side_path = [
        (0.335, side_y, 0.225),
        (0.170, side_y, 0.525),
        (-0.155, side_y, 0.525),
        (-0.205, side_y, 0.925),
        (-0.170, side_y, 0.535),
        (-0.055, side_y, 0.310),
    ]

    left_frame = tube_from_spline_points(
        side_path,
        radius=tube_radius,
        samples_per_segment=18,
        radial_segments=20,
    )
    right_frame = tube_from_spline_points(
        _mirror_y(side_path),
        radius=tube_radius,
        samples_per_segment=18,
        radial_segments=20,
    )
    part.visual(_mesh("wheelchair_left_side_frame", left_frame), material=frame_mat, name="left_side_frame")
    part.visual(_mesh("wheelchair_right_side_frame", right_frame), material=frame_mat, name="right_side_frame")

    tube_y = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    for name, length, xyz, radius in [
        ("front_seat_crossbar", 0.520, (0.120, 0.0, 0.505), 0.016),
        ("rear_seat_crossbar", 0.500, (-0.120, 0.0, 0.500), 0.016),
        ("axle_crossbar", 0.515, (-0.055, 0.0, 0.310), 0.018),
        ("back_top_crossbar", 0.470, (-0.192, 0.0, 0.855), 0.014),
    ]:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=tube_y.rpy),
            material=frame_mat,
            name=name,
        )

    for side_sign in (1.0, -1.0):
        y = 0.110 * side_sign
        hanger = tube_from_spline_points(
            [
                (0.175, 0.205 * side_sign, 0.515),
                (0.285, 0.150 * side_sign, 0.180),
                (0.345, y, 0.080),
            ],
            radius=0.014,
            samples_per_segment=12,
            radial_segments=16,
        )
        part.visual(
            _mesh(f"wheelchair_footrest_hanger_{'left' if side_sign > 0 else 'right'}", hanger),
            material=frame_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_footrest_hanger",
        )
        part.visual(
            Box((0.155, 0.100, 0.018)),
            origin=Origin(xyz=(0.355, y, 0.075)),
            material=plate_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_footplate",
        )

    for side_sign in (1.0, -1.0):
        y = side_y * side_sign
        part.visual(
            Cylinder(radius=0.014, length=0.086),
            origin=Origin(xyz=(0.376, y, 0.225), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_caster_mount_arm",
        )
        part.visual(
            Cylinder(radius=0.028, length=0.060),
            origin=Origin(xyz=(0.418, y, 0.225)),
            material=plate_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_caster_socket",
        )
        part.visual(
            Box((0.070, 0.060, 0.120)),
            origin=Origin(xyz=(-0.075, y, 0.330)),
            material=plate_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_axle_plate",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.052),
            origin=Origin(xyz=(-0.075, 0.261 * side_sign, 0.310), rpy=tube_y.rpy),
            material=plate_mat,
            name=f"{'left' if side_sign > 0 else 'right'}_axle_stub",
        )

    part.visual(
        Box((0.430, 0.470, 0.030)),
        origin=Origin(xyz=(0.040, 0.0, 0.520)),
        material=upholstery_mat,
        name="seat",
    )
    part.visual(
        Box((0.030, 0.450, 0.320)),
        origin=Origin(xyz=(-0.175, 0.0, 0.700)),
        material=upholstery_mat,
        name="backrest",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_mat = model.material("frame_mat", rgba=(0.12, 0.13, 0.15, 1.0))
    upholstery_mat = model.material("upholstery_mat", rgba=(0.10, 0.11, 0.14, 1.0))
    plate_mat = model.material("plate_mat", rgba=(0.55, 0.58, 0.62, 1.0))
    tire_mat = model.material("tire_mat", rgba=(0.05, 0.05, 0.06, 1.0))
    rim_mat = model.material("rim_mat", rgba=(0.74, 0.76, 0.79, 1.0))
    hub_mat = model.material("hub_mat", rgba=(0.45, 0.47, 0.50, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.68, 1.00)),
        mass=13.0,
        origin=Origin(xyz=(0.050, 0.0, 0.500)),
    )
    _add_frame_geometry(frame, frame_mat, upholstery_mat, plate_mat)

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.040),
        mass=2.4,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_rear_wheel_geometry(
        left_rear_wheel,
        mesh_prefix="wheelchair_left_rear_wheel",
        side_sign=1.0,
        tire_mat=tire_mat,
        rim_mat=rim_mat,
        hub_mat=hub_mat,
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.040),
        mass=2.4,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_rear_wheel_geometry(
        right_rear_wheel,
        mesh_prefix="wheelchair_right_rear_wheel",
        side_sign=-1.0,
        tire_mat=tire_mat,
        rim_mat=rim_mat,
        hub_mat=hub_mat,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.090, 0.080, 0.220)),
        mass=0.5,
        origin=Origin(xyz=(-0.030, 0.0, -0.110)),
    )
    _add_caster_fork_geometry(left_caster_fork, metal_mat=hub_mat)

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.090, 0.080, 0.220)),
        mass=0.5,
        origin=Origin(xyz=(-0.030, 0.0, -0.110)),
    )
    _add_caster_fork_geometry(right_caster_fork, metal_mat=hub_mat)

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.036),
        mass=0.45,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_geometry(
        left_caster_wheel,
        mesh_prefix="wheelchair_left_caster_wheel",
        tire_mat=tire_mat,
        rim_mat=rim_mat,
        hub_mat=hub_mat,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.036),
        mass=0.45,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_geometry(
        right_caster_wheel,
        mesh_prefix="wheelchair_right_caster_wheel",
        tire_mat=tire_mat,
        rim_mat=rim_mat,
        hub_mat=hub_mat,
    )

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.075, 0.312, 0.310)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.075, -0.312, 0.310)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.418, 0.235, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.418, -0.235, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.072, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.072, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    def _check_mirrored(name: str, left_pos, right_pos, *, tol: float) -> None:
        ok = (
            left_pos is not None
            and right_pos is not None
            and abs(left_pos[0] - right_pos[0]) <= tol
            and abs(left_pos[2] - right_pos[2]) <= tol
            and abs(left_pos[1] + right_pos[1]) <= tol
        )
        ctx.check(name, ok, details=f"left={left_pos}, right={right_pos}, tol={tol}")

    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    left_rear_wheel_spin = object_model.get_articulation("left_rear_wheel_spin")
    right_rear_wheel_spin = object_model.get_articulation("right_rear_wheel_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_wheel_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_caster_wheel_spin = object_model.get_articulation("right_caster_wheel_spin")

    ctx.check(
        "rear wheels use continuous axle rotation",
        left_rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_rear_wheel_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_rear_wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=(
            f"left_type={left_rear_wheel_spin.articulation_type}, left_axis={left_rear_wheel_spin.axis}, "
            f"right_type={right_rear_wheel_spin.articulation_type}, right_axis={right_rear_wheel_spin.axis}"
        ),
    )
    ctx.check(
        "casters swivel about vertical stems",
        left_caster_swivel.articulation_type == ArticulationType.CONTINUOUS
        and right_caster_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_caster_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(right_caster_swivel.axis) == (0.0, 0.0, 1.0),
        details=(
            f"left_type={left_caster_swivel.articulation_type}, left_axis={left_caster_swivel.axis}, "
            f"right_type={right_caster_swivel.articulation_type}, right_axis={right_caster_swivel.axis}"
        ),
    )
    ctx.check(
        "caster wheels spin on transverse axles",
        left_caster_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_caster_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_caster_wheel_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(right_caster_wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=(
            f"left_type={left_caster_wheel_spin.articulation_type}, left_axis={left_caster_wheel_spin.axis}, "
            f"right_type={right_caster_wheel_spin.articulation_type}, right_axis={right_caster_wheel_spin.axis}"
        ),
    )

    ctx.expect_contact(
        left_caster_wheel,
        left_caster_fork,
        elem_a="axle",
        elem_b="left_leg",
        name="left caster axle bears on left fork leg",
    )
    ctx.expect_contact(
        left_caster_wheel,
        left_caster_fork,
        elem_a="axle",
        elem_b="right_leg",
        name="left caster axle bears on right fork leg",
    )
    ctx.expect_contact(
        right_caster_wheel,
        right_caster_fork,
        elem_a="axle",
        elem_b="left_leg",
        name="right caster axle bears on left fork leg",
    )
    ctx.expect_contact(
        right_caster_wheel,
        right_caster_fork,
        elem_a="axle",
        elem_b="right_leg",
        name="right caster axle bears on right fork leg",
    )

    _check_mirrored(
        "rear wheels are mirrored about the chair centerline",
        ctx.part_world_position(left_rear_wheel),
        ctx.part_world_position(right_rear_wheel),
        tol=0.002,
    )
    _check_mirrored(
        "caster forks are mirrored about the chair centerline",
        ctx.part_world_position(left_caster_fork),
        ctx.part_world_position(right_caster_fork),
        tol=0.002,
    )
    _check_mirrored(
        "caster wheels are mirrored about the chair centerline",
        ctx.part_world_position(left_caster_wheel),
        ctx.part_world_position(right_caster_wheel),
        tol=0.002,
    )
    _check_mirrored(
        "footplates are mirrored about the chair centerline",
        _aabb_center(ctx.part_element_world_aabb("frame", elem="left_footplate")),
        _aabb_center(ctx.part_element_world_aabb("frame", elem="right_footplate")),
        tol=0.002,
    )

    left_caster_rest = ctx.part_world_position(left_caster_wheel)
    with ctx.pose({left_caster_swivel: pi / 2.0}):
        left_caster_swiveled = ctx.part_world_position(left_caster_wheel)
    ctx.check(
        "left caster swivel reorients the wheel around the stem",
        left_caster_rest is not None
        and left_caster_swiveled is not None
        and abs(left_caster_rest[0] - left_caster_swiveled[0]) >= 0.05
        and abs(left_caster_rest[1] - left_caster_swiveled[1]) >= 0.05,
        details=f"rest={left_caster_rest}, swiveled={left_caster_swiveled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
