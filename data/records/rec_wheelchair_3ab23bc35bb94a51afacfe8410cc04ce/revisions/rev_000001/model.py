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


def _rotated_torus(radius: float, tube: float):
    return TorusGeometry(radius, tube, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0)


def _add_rear_wheel_visuals(
    part,
    prefix: str,
    *,
    side_sign: float,
    rubber,
    rim_material,
    hub_material,
    pushrim_material,
) -> None:
    part.visual(_mesh(f"{prefix}_tire", _rotated_torus(0.283, 0.017)), material=rubber, name="tire")
    part.visual(_mesh(f"{prefix}_rim", _rotated_torus(0.257, 0.011)), material=rim_material, name="rim")
    part.visual(
        Cylinder(radius=0.028, length=0.036),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(-0.004 * side_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="axle_sleeve",
    )
    pushrim = _rotated_torus(0.274, 0.005).translate(0.028 * side_sign, 0.0, 0.0)
    part.visual(_mesh(f"{prefix}_pushrim", pushrim), material=pushrim_material, name="pushrim")

    for spoke_index in range(8):
        angle = 2.0 * pi * spoke_index / 8.0
        start = (0.0, cos(angle) * 0.020, sin(angle) * 0.020)
        end = (0.0, cos(angle) * 0.257, sin(angle) * 0.257)
        part.visual(
            _mesh(
                f"{prefix}_spoke_{spoke_index}",
                tube_from_spline_points(
                    [start, end],
                    radius=0.0023,
                    samples_per_segment=2,
                    radial_segments=10,
                ),
            ),
            material=hub_material,
        )

    for link_index, angle in enumerate((0.35, 2.45, 4.55)):
        rim_point = (0.009 * side_sign, cos(angle) * 0.250, sin(angle) * 0.250)
        pushrim_point = (0.028 * side_sign, cos(angle) * 0.274, sin(angle) * 0.274)
        part.visual(
            _mesh(
                f"{prefix}_pushrim_link_{link_index}",
                tube_from_spline_points(
                    [rim_point, pushrim_point],
                    radius=0.0026,
                    samples_per_segment=2,
                    radial_segments=10,
                ),
            ),
            material=pushrim_material,
        )


def _add_caster_wheel_visuals(
    part,
    prefix: str,
    *,
    rubber,
    rim_material,
    hub_material,
) -> None:
    part.visual(_mesh(f"{prefix}_tire", _rotated_torus(0.071, 0.014)), material=rubber, name="tire")
    part.visual(_mesh(f"{prefix}_rim", _rotated_torus(0.056, 0.007)), material=rim_material, name="rim")
    part.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )

    for spoke_index in range(4):
        angle = pi / 4.0 + 2.0 * pi * spoke_index / 4.0
        start = (0.0, cos(angle) * 0.008, sin(angle) * 0.008)
        end = (0.0, cos(angle) * 0.056, sin(angle) * 0.056)
        part.visual(
            _mesh(
                f"{prefix}_spoke_{spoke_index}",
                tube_from_spline_points(
                    [start, end],
                    radius=0.0020,
                    samples_per_segment=2,
                    radial_segments=10,
                ),
            ),
            material=hub_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.12, 0.13, 0.15, 1.0))
    foot_plate = model.material("foot_plate", rgba=(0.53, 0.55, 0.57, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_rim = model.material("wheel_rim", rgba=(0.76, 0.78, 0.80, 1.0))
    pushrim_material = model.material("pushrim_material", rgba=(0.84, 0.85, 0.86, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.68, 0.98, 0.96)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.02, 0.48)),
    )

    side_loop_points = [
        (0.24, -0.14, 0.34),
        (0.24, 0.12, 0.34),
        (0.18, 0.30, 0.22),
        (0.18, 0.30, 0.58),
        (0.24, 0.14, 0.58),
        (0.24, -0.18, 0.58),
        (0.24, -0.24, 0.92),
    ]
    frame.visual(
        _mesh(
            "left_side_loop",
            tube_from_spline_points(
                side_loop_points,
                radius=0.013,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_side_loop",
    )
    frame.visual(
        _mesh(
            "right_side_loop",
            tube_from_spline_points(
                _mirror_x(side_loop_points),
                radius=0.013,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_side_loop",
    )

    for name, points, radius in [
        ("rear_lower_cross", [(-0.24, -0.14, 0.34), (0.24, -0.14, 0.34)], 0.012),
        ("front_seat_cross", [(-0.24, 0.14, 0.58), (0.24, 0.14, 0.58)], 0.011),
        ("rear_seat_cross", [(-0.24, -0.14, 0.58), (0.24, -0.14, 0.58)], 0.011),
        ("backrest_cross", [(-0.24, -0.22, 0.78), (0.24, -0.22, 0.78)], 0.011),
        ("footrest_bridge", [(-0.11, 0.39, 0.07), (0.11, 0.39, 0.07)], 0.010),
        ("left_seat_rail", [(0.219, -0.18, 0.50), (0.219, 0.18, 0.50)], 0.010),
        ("right_seat_rail", [(-0.219, -0.18, 0.50), (-0.219, 0.18, 0.50)], 0.010),
    ]:
        frame.visual(
            _mesh(
                name,
                tube_from_spline_points(
                    points,
                    radius=radius,
                    samples_per_segment=2,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=name,
        )

    for name, points in [
        ("left_front_hanger", [(0.18, 0.30, 0.22), (0.11, 0.39, 0.07)]),
        ("right_front_hanger", [(-0.18, 0.30, 0.22), (-0.11, 0.39, 0.07)]),
        ("left_rear_brace", [(0.24, -0.14, 0.34), (0.24, -0.22, 0.72)]),
        ("right_rear_brace", [(-0.24, -0.14, 0.34), (-0.24, -0.22, 0.72)]),
        ("left_front_seat_drop", [(0.24, 0.14, 0.58), (0.219, 0.14, 0.50)]),
        ("right_front_seat_drop", [(-0.24, 0.14, 0.58), (-0.219, 0.14, 0.50)]),
        ("left_rear_seat_drop", [(0.24, -0.14, 0.58), (0.219, -0.14, 0.50)]),
        ("right_rear_seat_drop", [(-0.24, -0.14, 0.58), (-0.219, -0.14, 0.50)]),
        ("left_caster_bracket", [(0.18, 0.30, 0.22), (0.166, 0.30, 0.238)]),
        ("right_caster_bracket", [(-0.18, 0.30, 0.22), (-0.166, 0.30, 0.238)]),
        ("left_caster_bridge_riser", [(0.166, 0.30, 0.238), (0.166, 0.272, 0.24)]),
        ("right_caster_bridge_riser", [(-0.166, 0.30, 0.238), (-0.166, 0.272, 0.24)]),
        ("caster_bridge", [(-0.166, 0.272, 0.24), (0.166, 0.272, 0.24)]),
    ]:
        frame.visual(
            _mesh(
                name,
                tube_from_spline_points(
                    points,
                    radius=0.011,
                    samples_per_segment=2,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=name,
        )

    frame.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.150, 0.300, 0.240)),
        material=dark_steel,
        name="left_caster_headtube",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(-0.150, 0.300, 0.240)),
        material=dark_steel,
        name="right_caster_headtube",
    )

    frame.visual(
        Box((0.050, 0.070, 0.200)),
        origin=Origin(xyz=(0.265, -0.12, 0.31)),
        material=dark_steel,
        name="left_axle_plate",
    )
    frame.visual(
        Box((0.050, 0.070, 0.200)),
        origin=Origin(xyz=(-0.265, -0.12, 0.31)),
        material=dark_steel,
        name="right_axle_plate",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.291, -0.12, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(-0.291, -0.12, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_stub",
    )
    frame.visual(
        Box((0.438, 0.420, 0.012)),
        origin=Origin(xyz=(0.0, 0.00, 0.500)),
        material=seat_fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.410, 0.016, 0.340)),
        origin=Origin(xyz=(0.0, -0.205, 0.700)),
        material=seat_fabric,
        name="backrest_panel",
    )
    frame.visual(
        Box((0.140, 0.110, 0.012)),
        origin=Origin(xyz=(0.110, 0.400, 0.055)),
        material=foot_plate,
        name="left_footplate",
    )
    frame.visual(
        Box((0.140, 0.110, 0.012)),
        origin=Origin(xyz=(-0.110, 0.400, 0.055)),
        material=foot_plate,
        name="right_footplate",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.300, length=0.036),
        mass=2.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel",
        side_sign=1.0,
        rubber=tire_rubber,
        rim_material=wheel_rim,
        hub_material=dark_steel,
        pushrim_material=pushrim_material,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.300, length=0.036),
        mass=2.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel",
        side_sign=-1.0,
        rubber=tire_rubber,
        rim_material=wheel_rim,
        hub_material=dark_steel,
        pushrim_material=pushrim_material,
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        caster = model.part(f"front_{side_name}_caster")
        caster.inertial = Inertial.from_geometry(
            Box((0.050, 0.080, 0.250)),
            mass=0.7,
            origin=Origin(xyz=(0.0, -0.02, -0.13)),
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.100),
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
            material=dark_steel,
            name="stem",
        )
        caster.visual(
            Box((0.040, 0.030, 0.026)),
            origin=Origin(xyz=(0.0, -0.016, -0.112)),
            material=dark_steel,
            name="crown",
        )
        caster.visual(
            Box((0.008, 0.024, 0.116)),
            origin=Origin(xyz=(0.018, -0.032, -0.173)),
            material=dark_steel,
            name="outer_fork_plate",
        )
        caster.visual(
            Box((0.008, 0.024, 0.116)),
            origin=Origin(xyz=(-0.018, -0.032, -0.173)),
            material=dark_steel,
            name="inner_fork_plate",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.021, -0.032, -0.225), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="outer_axle_stub",
        )
        caster.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(-0.021, -0.032, -0.225), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="inner_axle_stub",
        )

        caster_wheel = model.part(f"front_{side_name}_caster_wheel")
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.085, length=0.028),
            mass=0.45,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        _add_caster_wheel_visuals(
            caster_wheel,
            f"front_{side_name}_caster_wheel",
            rubber=tire_rubber,
            rim_material=wheel_rim,
            hub_material=dark_steel,
        )

        model.articulation(
            f"front_{side_name}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(0.15 * side_sign, 0.30, 0.22)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=4.0),
        )
        model.articulation(
            f"front_{side_name}_caster_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, -0.032, -0.225)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.320, -0.12, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.320, -0.12, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")

    ctx.expect_contact(
        front_left_caster,
        frame,
        elem_a="stem",
        elem_b="left_caster_headtube",
        contact_tol=1e-6,
        name="left caster stem seats in the front headtube",
    )
    ctx.expect_contact(
        front_right_caster,
        frame,
        elem_a="stem",
        elem_b="right_caster_headtube",
        contact_tol=1e-6,
        name="right caster stem seats in the front headtube",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        elem_a="axle_sleeve",
        elem_b="left_axle_stub",
        contact_tol=1e-6,
        name="left rear wheel mounts on the axle stub",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        elem_a="axle_sleeve",
        elem_b="right_axle_stub",
        contact_tol=1e-6,
        name="right rear wheel mounts on the axle stub",
    )

    rear_left_pos = ctx.part_world_position(rear_left_wheel)
    rear_right_pos = ctx.part_world_position(rear_right_wheel)
    front_left_pos = ctx.part_world_position(front_left_caster)
    front_right_pos = ctx.part_world_position(front_right_caster)
    rear_track = None
    front_track = None
    if rear_left_pos is not None and rear_right_pos is not None:
        rear_track = abs(rear_left_pos[0] - rear_right_pos[0])
    if front_left_pos is not None and front_right_pos is not None:
        front_track = abs(front_left_pos[0] - front_right_pos[0])
    ctx.check(
        "rear support is broader than front caster support",
        rear_track is not None and front_track is not None and rear_track > front_track + 0.20,
        details=f"rear_track={rear_track}, front_track={front_track}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
