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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _add_rear_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    side_sign: float,
    tire_material,
    rim_material,
    pushrim_material,
    hub_material,
) -> None:
    tire = _mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(radius=0.283, tube=0.022, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0),
    )
    rim = _mesh(
        f"{mesh_prefix}_rim",
        TorusGeometry(radius=0.254, tube=0.008, radial_segments=14, tubular_segments=48).rotate_y(pi / 2.0),
    )
    pushrim = _mesh(
        f"{mesh_prefix}_pushrim",
        TorusGeometry(radius=0.270, tube=0.007, radial_segments=12, tubular_segments=48).rotate_y(pi / 2.0),
    )
    part.visual(tire, material=tire_material, name="tire")
    part.visual(rim, material=rim_material, name="rim")
    part.visual(pushrim, origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.0)), material=pushrim_material, name="pushrim")
    part.visual(
        Cylinder(radius=0.048, length=0.028),
        origin=Origin(xyz=(side_sign * 0.033, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(side_sign * 0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="axle_tube",
    )
    for spoke_index in range(6):
        angle = spoke_index * (pi / 3.0)
        spoke_radius = 0.150
        part.visual(
            Box((0.006, 0.010, 0.210)),
            origin=Origin(
                xyz=(0.0, -spoke_radius * sin(angle), spoke_radius * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=rim_material,
            name=f"spoke_{spoke_index}",
        )
    for brace_index in range(3):
        angle = brace_index * (2.0 * pi / 3.0) + pi / 6.0
        brace_radius = 0.262
        part.visual(
            Box((0.030, 0.006, 0.024)),
            origin=Origin(
                xyz=(side_sign * 0.015, -brace_radius * sin(angle), brace_radius * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=pushrim_material,
            name=f"pushrim_brace_{brace_index}",
        )


def _add_caster_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    tire_material,
    rim_material,
    hub_material,
) -> None:
    tire = _mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(radius=0.067, tube=0.013, radial_segments=14, tubular_segments=40).rotate_y(pi / 2.0),
    )
    rim = _mesh(
        f"{mesh_prefix}_rim",
        TorusGeometry(radius=0.050, tube=0.008, radial_segments=12, tubular_segments=32).rotate_y(pi / 2.0),
    )
    part.visual(tire, material=tire_material, name="tire")
    part.visual(rim, material=rim_material, name="rim")
    part.visual(
        Cylinder(radius=0.020, length=0.036),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    for spoke_index in range(4):
        angle = spoke_index * (pi / 2.0)
        spoke_radius = 0.038
        part.visual(
            Box((0.005, 0.008, 0.074)),
            origin=Origin(
                xyz=(0.0, -spoke_radius * sin(angle), spoke_radius * cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=rim_material,
            name=f"spoke_{spoke_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.76, 0.79, 0.83, 1.0))
    upholstery = model.material("upholstery", rgba=(0.11, 0.12, 0.14, 1.0))
    footplate = model.material("footplate", rgba=(0.17, 0.18, 0.19, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.22, 0.23, 0.24, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_rim = model.material("wheel_rim", rgba=(0.71, 0.73, 0.77, 1.0))
    handrim = model.material("handrim", rgba=(0.84, 0.86, 0.89, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.84, 0.92)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.10, 0.44)),
    )

    left_seat_rail_pts = [
        (0.225, -0.105, 0.505),
        (0.225, 0.040, 0.490),
        (0.235, 0.235, 0.425),
        (0.248, 0.355, 0.345),
    ]
    left_lower_rail_pts = [
        (0.275, -0.120, 0.305),
        (0.265, 0.080, 0.245),
        (0.250, 0.355, 0.180),
    ]
    left_back_cane_pts = [
        (0.275, -0.120, 0.305),
        (0.245, -0.115, 0.555),
        (0.205, -0.095, 0.885),
    ]
    left_front_hanger_pts = [
        (0.205, 0.285, 0.390),
        (0.175, 0.380, 0.185),
        (0.132, 0.455, 0.082),
    ]

    for mesh_name, points, radius in [
        ("left_seat_rail", left_seat_rail_pts, 0.015),
        ("right_seat_rail", _mirror_x(left_seat_rail_pts), 0.015),
        ("left_lower_rail", left_lower_rail_pts, 0.014),
        ("right_lower_rail", _mirror_x(left_lower_rail_pts), 0.014),
        ("left_back_cane", left_back_cane_pts, 0.016),
        ("right_back_cane", _mirror_x(left_back_cane_pts), 0.016),
        ("left_front_hanger", left_front_hanger_pts, 0.012),
        ("right_front_hanger", _mirror_x(left_front_hanger_pts), 0.012),
    ]:
        frame.visual(
            _mesh(
                f"wheelchair_{mesh_name}",
                tube_from_spline_points(
                    points,
                    radius=radius,
                    samples_per_segment=18,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=mesh_name,
        )

    frame.visual(
        Cylinder(radius=0.018, length=0.46),
        origin=Origin(xyz=(0.0, -0.120, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="rear_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=(0.0, 0.045, 0.455), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="front_seat_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.38),
        origin=Origin(xyz=(0.0, -0.055, 0.465), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="rear_seat_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.28),
        origin=Origin(xyz=(0.0, 0.465, 0.082), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
        name="footrest_crossbar",
    )

    for side_sign, visual_name in ((1.0, "left_axle_stub"), (-1.0, "right_axle_stub")):
        frame.visual(
            Cylinder(radius=0.015, length=0.11),
            origin=Origin(xyz=(side_sign * 0.275, -0.120, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_paint,
            name=visual_name,
        )
        frame.visual(
            Cylinder(radius=0.016, length=0.054),
            origin=Origin(xyz=(side_sign * 0.255, 0.390, 0.257)),
            material=frame_paint,
            name=f"{visual_name}_caster_socket",
        )
        frame.visual(
            Box((0.024, 0.060, 0.026)),
            origin=Origin(xyz=(side_sign * 0.228, 0.366, 0.224)),
            material=frame_paint,
            name=f"{visual_name}_caster_bridge",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.070),
            origin=Origin(xyz=(side_sign * 0.126, 0.465, 0.080)),
            material=frame_paint,
            name=f"{visual_name}_foot_support",
        )
        frame.visual(
            Box((0.036, 0.058, 0.050)),
            origin=Origin(xyz=(side_sign * 0.216, 0.275, 0.398)),
            material=frame_paint,
            name=f"{visual_name}_front_connector",
        )
        frame.visual(
            Box((0.034, 0.050, 0.060)),
            origin=Origin(xyz=(side_sign * 0.236, -0.108, 0.525)),
            material=frame_paint,
            name=f"{visual_name}_rear_connector",
        )

    frame.visual(
        Box((0.470, 0.430, 0.032)),
        origin=Origin(xyz=(0.0, 0.045, 0.468)),
        material=upholstery,
        name="seat_sling",
    )
    frame.visual(
        Box((0.450, 0.060, 0.350)),
        origin=Origin(xyz=(0.0, -0.090, 0.680)),
        material=upholstery,
        name="backrest_panel",
    )
    frame.visual(
        Box((0.132, 0.088, 0.012)),
        origin=Origin(xyz=(0.115, 0.468, 0.060)),
        material=footplate,
        name="left_footplate",
    )
    frame.visual(
        Box((0.132, 0.088, 0.012)),
        origin=Origin(xyz=(-0.115, 0.468, 0.060)),
        material=footplate,
        name="right_footplate",
    )
    frame.visual(
        Box((0.110, 0.030, 0.018)),
        origin=Origin(xyz=(0.195, -0.095, 0.883)),
        material=dark_trim,
        name="left_push_handle",
    )
    frame.visual(
        Box((0.110, 0.030, 0.018)),
        origin=Origin(xyz=(-0.195, -0.095, 0.883)),
        material=dark_trim,
        name="right_push_handle",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.044),
        mass=2.3,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_left_wheel,
        mesh_prefix="rear_left_wheel",
        side_sign=1.0,
        tire_material=tire_rubber,
        rim_material=wheel_rim,
        pushrim_material=handrim,
        hub_material=hub_dark,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.044),
        mass=2.3,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_right_wheel,
        mesh_prefix="rear_right_wheel",
        side_sign=-1.0,
        tire_material=tire_rubber,
        rim_material=wheel_rim,
        pushrim_material=handrim,
        hub_material=hub_dark,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.130)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.015, -0.070)),
    )
    left_caster_fork.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=hub_dark,
        name="stem",
    )
    left_caster_fork.visual(
        Box((0.020, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, 0.014, -0.052)),
        material=hub_dark,
        name="stem_yoke",
    )
    left_caster_fork.visual(
        Box((0.052, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.034, -0.065)),
        material=hub_dark,
        name="crown",
    )
    for side_sign in (-1.0, 1.0):
        left_caster_fork.visual(
            Box((0.008, 0.074, 0.128)),
            origin=Origin(xyz=(side_sign * 0.022, 0.042, -0.126)),
            material=hub_dark,
            name=f"fork_leg_{'right' if side_sign > 0 else 'left'}",
        )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.130)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.015, -0.070)),
    )
    right_caster_fork.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=hub_dark,
        name="stem",
    )
    right_caster_fork.visual(
        Box((0.020, 0.028, 0.016)),
        origin=Origin(xyz=(0.0, 0.014, -0.052)),
        material=hub_dark,
        name="stem_yoke",
    )
    right_caster_fork.visual(
        Box((0.052, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.034, -0.065)),
        material=hub_dark,
        name="crown",
    )
    for side_sign in (-1.0, 1.0):
        right_caster_fork.visual(
            Box((0.008, 0.074, 0.128)),
            origin=Origin(xyz=(side_sign * 0.022, 0.042, -0.126)),
            material=hub_dark,
            name=f"fork_leg_{'right' if side_sign > 0 else 'left'}",
        )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.036),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        left_caster_wheel,
        mesh_prefix="left_caster_wheel",
        tire_material=tire_rubber,
        rim_material=wheel_rim,
        hub_material=hub_dark,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.036),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        right_caster_wheel,
        mesh_prefix="right_caster_wheel",
        tire_material=tire_rubber,
        rim_material=wheel_rim,
        hub_material=hub_dark,
    )

    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.315, -0.120, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.315, -0.120, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.255, 0.390, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.255, 0.390, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.045, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.045, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rear_left = object_model.get_part("rear_left_wheel")
    rear_right = object_model.get_part("rear_right_wheel")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_wheel_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_caster_wheel_spin = object_model.get_articulation("right_caster_wheel_spin")

    def _bottom_z(part_name: str, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem)
        return None if aabb is None else aabb[0][2]

    rear_track = None
    caster_track = None
    rear_left_pos = ctx.part_world_position(rear_left)
    rear_right_pos = ctx.part_world_position(rear_right)
    left_caster_pos = ctx.part_world_position(left_caster_wheel)
    right_caster_pos = ctx.part_world_position(right_caster_wheel)
    if rear_left_pos is not None and rear_right_pos is not None:
        rear_track = abs(rear_left_pos[0] - rear_right_pos[0])
    if left_caster_pos is not None and right_caster_pos is not None:
        caster_track = abs(left_caster_pos[0] - right_caster_pos[0])

    ctx.check(
        "wheelchair keeps a wide stance",
        rear_track is not None and caster_track is not None and rear_track >= 0.62 and caster_track >= 0.48,
        details=f"rear_track={rear_track}, caster_track={caster_track}",
    )

    seat_aabb = ctx.part_element_world_aabb(frame, elem="seat_sling")
    seat_top = None if seat_aabb is None else seat_aabb[1][2]
    ctx.check(
        "seat remains low over the wheel centers",
        seat_top is not None and 0.47 <= seat_top <= 0.50,
        details=f"seat_top={seat_top}",
    )

    wheel_bottoms = [
        _bottom_z("rear_left_wheel", "tire"),
        _bottom_z("rear_right_wheel", "tire"),
        _bottom_z("left_caster_wheel", "tire"),
        _bottom_z("right_caster_wheel", "tire"),
    ]
    finite_bottoms = [value for value in wheel_bottoms if value is not None]
    ctx.check(
        "all wheels sit on one ground plane",
        len(finite_bottoms) == 4 and max(finite_bottoms) - min(finite_bottoms) <= 0.006 and abs(sum(finite_bottoms) / 4.0) <= 0.006,
        details=f"wheel_bottoms={wheel_bottoms}",
    )

    ctx.check(
        "rear wheel and caster joints use expected continuous axes",
        rear_left_spin.joint_type == ArticulationType.CONTINUOUS
        and rear_right_spin.joint_type == ArticulationType.CONTINUOUS
        and left_caster_swivel.joint_type == ArticulationType.CONTINUOUS
        and right_caster_swivel.joint_type == ArticulationType.CONTINUOUS
        and left_caster_wheel_spin.joint_type == ArticulationType.CONTINUOUS
        and right_caster_wheel_spin.joint_type == ArticulationType.CONTINUOUS
        and rear_left_spin.axis == (1.0, 0.0, 0.0)
        and rear_right_spin.axis == (1.0, 0.0, 0.0)
        and left_caster_swivel.axis == (0.0, 0.0, 1.0)
        and right_caster_swivel.axis == (0.0, 0.0, 1.0)
        and left_caster_wheel_spin.axis == (1.0, 0.0, 0.0)
        and right_caster_wheel_spin.axis == (1.0, 0.0, 0.0),
        details=(
            f"rear_left_axis={rear_left_spin.axis}, rear_right_axis={rear_right_spin.axis}, "
            f"left_swivel_axis={left_caster_swivel.axis}, right_swivel_axis={right_caster_swivel.axis}, "
            f"left_caster_spin_axis={left_caster_wheel_spin.axis}, right_caster_spin_axis={right_caster_wheel_spin.axis}"
        ),
    )

    rest_pos = ctx.part_world_position(left_caster_wheel)
    with ctx.pose(left_caster_swivel=1.10):
        swiveled_pos = ctx.part_world_position(left_caster_wheel)
    swivel_moved = (
        rest_pos is not None
        and swiveled_pos is not None
        and abs(swiveled_pos[0] - rest_pos[0]) + abs(swiveled_pos[1] - rest_pos[1]) >= 0.020
    )
    ctx.check(
        "left caster swivels around a trailing stem",
        swivel_moved,
        details=f"rest_pos={rest_pos}, swiveled_pos={swiveled_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
