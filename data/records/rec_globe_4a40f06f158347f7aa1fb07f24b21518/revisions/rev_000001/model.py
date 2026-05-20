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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_axis(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _axis_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    return _rpy_for_z_axis((0.0, 0.0, 0.0), axis)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis(a, b)),
        material=material,
        name=name,
    )


def _arc_points(
    *,
    radius: float,
    center_z: float,
    start_angle: float,
    end_angle: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.sin(start_angle + (end_angle - start_angle) * i / (samples - 1)),
            0.0,
            center_z
            + radius
            * math.cos(start_angle + (end_angle - start_angle) * i / (samples - 1)),
        )
        for i in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_globe")

    walnut = model.material("walnut", rgba=(0.31, 0.20, 0.12, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.20, 0.12, 0.07, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.27, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.56, 0.46, 0.23, 1.0))
    parchment = model.material("parchment", rgba=(0.88, 0.82, 0.67, 1.0))
    ink_dark = model.material("ink_dark", rgba=(0.19, 0.16, 0.12, 1.0))

    globe_radius = 0.26
    globe_center_z = 0.99
    meridian_radius = 0.31
    tilt = math.radians(23.5)
    polar_axis = (math.sin(tilt), 0.0, math.cos(tilt))
    axis_rpy = _axis_rpy(polar_axis)

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.23, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_walnut,
        name="floor_disc",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=walnut,
        name="pedestal_plinth",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=aged_brass,
        name="bearing_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.112)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.148, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brass,
        name="turntable_deck",
    )
    stand.visual(
        Cylinder(radius=0.094, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=aged_brass,
        name="turntable_cap",
    )
    stand.visual(
        Cylinder(radius=0.074, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=walnut,
        name="lower_collar",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.564),
        origin=Origin(xyz=(0.0, 0.0, 0.388)),
        material=walnut,
        name="pedestal_column",
    )
    stand.visual(
        Cylinder(radius=0.078, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=aged_brass,
        name="upper_socket",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.182),
        origin=Origin(xyz=(0.0, 0.0, 0.706), rpy=_axis_rpy((0.0, 1.0, 0.0))),
        material=aged_brass,
        name="yoke_crossbar",
    )

    north_angle = tilt
    south_angle = tilt - math.pi
    left_arc_points = _arc_points(
        radius=meridian_radius,
        center_z=globe_center_z,
        start_angle=south_angle + 0.24,
        end_angle=north_angle - 0.18,
        samples=18,
    )
    right_arc_points = _arc_points(
        radius=meridian_radius,
        center_z=globe_center_z,
        start_angle=north_angle + 0.18,
        end_angle=2.30,
        samples=18,
    )
    left_meridian_mesh = mesh_from_geometry(
        tube_from_spline_points(
            left_arc_points,
            radius=0.012,
            samples_per_segment=8,
            radial_segments=20,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "left_meridian_arc_v1",
    )
    right_meridian_mesh = mesh_from_geometry(
        tube_from_spline_points(
            right_arc_points,
            radius=0.012,
            samples_per_segment=8,
            radial_segments=20,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "right_meridian_arc_v1",
    )
    stand.visual(left_meridian_mesh, material=brass, name="left_meridian")
    stand.visual(right_meridian_mesh, material=brass, name="right_meridian")

    left_fork_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -0.091, 0.706),
                (-0.060, -0.152, 0.748),
                (-0.156, -0.168, 0.822),
                left_arc_points[0],
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "left_fork_arm_v5",
    )
    right_fork_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.091, 0.706),
                (0.060, 0.152, 0.748),
                (0.156, 0.168, 0.822),
                right_arc_points[-1],
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "right_fork_arm_v5",
    )
    stand.visual(left_fork_mesh, material=aged_brass, name="left_fork")
    stand.visual(right_fork_mesh, material=aged_brass, name="right_fork")

    north_bearing_center = (
        polar_axis[0] * 0.317,
        0.0,
        globe_center_z + polar_axis[2] * 0.317,
    )
    south_bearing_center = (
        -polar_axis[0] * 0.317,
        0.0,
        globe_center_z - polar_axis[2] * 0.317,
    )

    stand.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=north_bearing_center, rpy=axis_rpy),
        material=aged_brass,
        name="north_bearing",
    )
    stand.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=south_bearing_center, rpy=axis_rpy),
        material=aged_brass,
        name="south_bearing",
    )
    stand.visual(
        Box((0.010, 0.080, 0.008)),
        origin=Origin(xyz=(south_bearing_center[0], -0.055, south_bearing_center[2])),
        material=aged_brass,
        name="south_bearing_bridge",
    )
    _add_member(
        stand,
        (south_bearing_center[0], -0.095, south_bearing_center[2]),
        (-0.156, -0.168, 0.822),
        0.0045,
        aged_brass,
        name="south_bearing_brace",
    )
    _add_member(
        stand,
        left_arc_points[-1],
        north_bearing_center,
        0.008,
        aged_brass,
        name="north_left_boss",
    )
    _add_member(
        stand,
        right_arc_points[0],
        north_bearing_center,
        0.008,
        aged_brass,
        name="north_right_boss",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.64, 0.46, 1.34)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        material=parchment,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(
            xyz=tuple(component * 0.266 for component in polar_axis),
            rpy=axis_rpy,
        ),
        material=ink_dark,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(
            xyz=tuple(-component * 0.266 for component in polar_axis),
            rpy=axis_rpy,
        ),
        material=ink_dark,
        name="south_cap",
    )
    globe.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(
            xyz=tuple(component * 0.287 for component in polar_axis),
            rpy=axis_rpy,
        ),
        material=aged_brass,
        name="north_axle",
    )
    globe.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(
            xyz=tuple(-component * 0.287 for component in polar_axis),
            rpy=axis_rpy,
        ),
        material=aged_brass,
        name="south_axle",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=globe_radius),
        mass=4.0,
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, globe_center_z)),
        axis=polar_axis,
        motion_limits=MotionLimits(effort=2.0, velocity=3.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    globe = object_model.get_part("globe")
    swivel = object_model.get_articulation("base_swivel")
    spin = object_model.get_articulation("globe_spin")

    bearing_plate = base.get_visual("bearing_plate")
    turntable_deck = stand.get_visual("turntable_deck")
    north_bearing = stand.get_visual("north_bearing")
    south_bearing = stand.get_visual("south_bearing")
    north_axle = globe.get_visual("north_axle")
    south_axle = globe.get_visual("south_axle")

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

    ctx.check(
        "swivel_axis_is_vertical",
        swivel.articulation_type == ArticulationType.CONTINUOUS and swivel.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical continuous swivel axis, got type={swivel.articulation_type} axis={swivel.axis}",
    )
    ctx.check(
        "globe_axis_is_polar_tilt",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(spin.axis[0]) > 0.35
        and abs(spin.axis[1]) < 1e-9
        and spin.axis[2] > 0.90,
        details=f"Expected tilted polar axis in xz plane, got type={spin.articulation_type} axis={spin.axis}",
    )

    ctx.expect_contact(
        base,
        stand,
        elem_a=bearing_plate,
        elem_b=turntable_deck,
        name="turntable_bearing_seats",
    )
    ctx.expect_overlap(
        base,
        stand,
        axes="xy",
        elem_a=bearing_plate,
        elem_b=turntable_deck,
        min_overlap=0.20,
        name="turntable_has_broad_support_overlap",
    )
    ctx.expect_contact(
        globe,
        stand,
        elem_a=north_axle,
        elem_b=north_bearing,
        name="north_polar_pivot_contacts",
    )
    ctx.expect_contact(
        globe,
        stand,
        elem_a=south_axle,
        elem_b=south_bearing,
        name="south_polar_pivot_contacts",
    )
    ctx.expect_origin_gap(
        globe,
        base,
        axis="z",
        min_gap=1.00,
        max_gap=1.20,
        name="globe_center_height_is_floor_standing_scale",
    )

    with ctx.pose({spin: 1.8}):
        ctx.expect_contact(globe, stand, elem_a=north_axle, elem_b=north_bearing)
        ctx.expect_contact(globe, stand, elem_a=south_axle, elem_b=south_bearing)

    with ctx.pose({swivel: 1.0}):
        ctx.expect_contact(base, stand, elem_a=bearing_plate, elem_b=turntable_deck)
        ctx.expect_origin_gap(globe, base, axis="z", min_gap=1.00, max_gap=1.20)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
