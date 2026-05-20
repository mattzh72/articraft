from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points):
    return [(-x, y, z) for x, y, z in points]


def _annular_sector_profile(
    *,
    center_y: float,
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    segments: int = 28,
):
    start = radians(start_deg)
    end = radians(end_deg)
    outer = []
    inner = []
    for index in range(segments + 1):
        t = index / segments
        angle = start + (end - start) * t
        outer.append((outer_radius * sin(angle + pi * 0.5), center_y + outer_radius * sin(angle)))
        inner.append((inner_radius * sin(angle + pi * 0.5), center_y + inner_radius * sin(angle)))
    return outer + list(reversed(inner))


def _wheel_tire_mesh(radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.58, -half_width * 0.98),
        (radius * 0.82, -half_width),
        (radius * 0.94, -half_width * 0.74),
        (radius, -half_width * 0.18),
        (radius, half_width * 0.18),
        (radius * 0.94, half_width * 0.74),
        (radius * 0.82, half_width),
        (radius * 0.58, half_width * 0.98),
        (radius * 0.45, half_width * 0.34),
        (radius * 0.42, 0.0),
        (radius * 0.45, -half_width * 0.34),
        (radius * 0.58, -half_width * 0.98),
    ]
    return LatheGeometry(profile, segments=64).rotate_y(pi / 2.0)


def _pivot_eyelet_mesh(*, outer_radius: float, inner_radius: float, length: float):
    half_length = length * 0.5
    outer_profile = [(outer_radius, -half_length), (outer_radius, half_length)]
    inner_profile = [(inner_radius, -half_length), (inner_radius, half_length)]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=40,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylinder_bottle_hand_truck")

    frame_red = model.material("frame_red", rgba=(0.73, 0.11, 0.10, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    steel_light = model.material("steel_light", rgba=(0.70, 0.73, 0.77, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    wheel_radius = 0.205
    wheel_width = 0.065
    axle_z = wheel_radius
    axle_y = -0.025
    rail_half = 0.180
    wheel_center_x = 0.245
    pivot_y = 0.045
    pivot_z = 0.965

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.42, 1.32)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.04, 0.66)),
    )

    left_rail = tube_from_spline_points(
        [
            (-rail_half, -0.020, axle_z - 0.020),
            (-rail_half, -0.004, 0.470),
            (-rail_half, 0.010, 0.860),
            (-rail_half, -0.018, 1.140),
            (-rail_half + 0.020, -0.095, 1.275),
        ],
        radius=0.016,
        samples_per_segment=14,
        radial_segments=18,
    )
    right_rail = tube_from_spline_points(
        _mirror_x(
            [
                (-rail_half, -0.020, axle_z - 0.020),
                (-rail_half, -0.004, 0.470),
                (-rail_half, 0.010, 0.860),
                (-rail_half, -0.018, 1.140),
                (-rail_half + 0.020, -0.095, 1.275),
            ]
        ),
        radius=0.016,
        samples_per_segment=14,
        radial_segments=18,
    )
    frame.visual(_mesh("left_frame_rail", left_rail), material=frame_red, name="left_rail")
    frame.visual(_mesh("right_frame_rail", right_rail), material=frame_red, name="right_rail")

    left_lower_brace = tube_from_spline_points(
        [
            (-rail_half, -0.020, axle_z - 0.020),
            (-0.164, 0.035, 0.120),
            (-0.145, 0.105, 0.052),
        ],
        radius=0.014,
        samples_per_segment=10,
        radial_segments=16,
    )
    right_lower_brace = tube_from_spline_points(
        _mirror_x(
            [
                (-rail_half, -0.020, axle_z - 0.020),
                (-0.164, 0.035, 0.120),
                (-0.145, 0.105, 0.052),
            ]
        ),
        radius=0.014,
        samples_per_segment=10,
        radial_segments=16,
    )
    frame.visual(_mesh("left_lower_brace", left_lower_brace), material=frame_red, name="left_lower_brace")
    frame.visual(_mesh("right_lower_brace", right_lower_brace), material=frame_red, name="right_lower_brace")

    frame.visual(
        Cylinder(radius=0.017, length=0.330),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="axle_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.340),
        origin=Origin(xyz=(0.0, 0.010, 0.620), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="mid_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.0, 0.000, 0.955), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="shoulder_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.306),
        origin=Origin(xyz=(0.0, -0.092, 1.220), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="handle_crossbar",
    )

    frame.visual(
        Cylinder(radius=0.020, length=0.110),
        origin=Origin(xyz=(-0.168, -0.100, 1.246), rpy=(0.0, 0.32, 0.0)),
        material=rubber_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.110),
        origin=Origin(xyz=(0.168, -0.100, 1.246), rpy=(0.0, -0.32, 0.0)),
        material=rubber_black,
        name="right_grip",
    )

    frame.visual(
        Box((0.300, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.115, 0.024)),
        material=steel_dark,
        name="nose_plate",
    )
    frame.visual(
        Box((0.300, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, 0.193, 0.0225)),
        material=steel_dark,
        name="nose_plate_lip",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.280),
        origin=Origin(xyz=(0.0, 0.106, 0.145), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="lower_front_bar",
    )
    frame.visual(
        Box((0.020, 0.022, 0.118)),
        origin=Origin(xyz=(-0.135, 0.106, 0.084)),
        material=frame_red,
        name="left_nose_gusset",
    )
    frame.visual(
        Box((0.020, 0.022, 0.118)),
        origin=Origin(xyz=(0.135, 0.106, 0.084)),
        material=frame_red,
        name="right_nose_gusset",
    )

    cradle_profile = _annular_sector_profile(
        center_y=0.170,
        outer_radius=0.170,
        inner_radius=0.158,
        start_deg=-170.0,
        end_deg=-10.0,
        segments=30,
    )
    cradle_geom = ExtrudeGeometry(cradle_profile, 0.540, center=True)
    frame.visual(
        _mesh("cradle_shell", cradle_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
        material=steel_dark,
        name="cradle_shell",
    )
    frame.visual(
        Box((0.030, 0.060, 0.500)),
        origin=Origin(xyz=(-0.166, 0.040, 0.570)),
        material=frame_red,
        name="left_cradle_tab",
    )
    frame.visual(
        Box((0.030, 0.060, 0.500)),
        origin=Origin(xyz=(0.166, 0.040, 0.570)),
        material=frame_red,
        name="right_cradle_tab",
    )

    frame.visual(
        Box((0.038, 0.070, 0.110)),
        origin=Origin(xyz=(-0.186, axle_y, axle_z)),
        material=frame_red,
        name="left_axle_mount",
    )
    frame.visual(
        Box((0.038, 0.070, 0.110)),
        origin=Origin(xyz=(0.186, axle_y, axle_z)),
        material=frame_red,
        name="right_axle_mount",
    )
    frame.visual(
        Box((0.034, 0.050, 0.110)),
        origin=Origin(xyz=(-rail_half, 0.002, pivot_z - 0.010)),
        material=frame_red,
        name="left_shoulder_lug",
    )
    frame.visual(
        Box((0.034, 0.050, 0.110)),
        origin=Origin(xyz=(rail_half, 0.002, pivot_z - 0.010)),
        material=frame_red,
        name="right_shoulder_lug",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(-0.186, pivot_y, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="left_pivot_washer",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.186, pivot_y, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="right_pivot_washer",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(-rail_half, pivot_y, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="left_pivot_pin",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(rail_half, pivot_y, pivot_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="right_pivot_pin",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.080),
        mass=5.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    left_wheel.visual(
        _mesh("left_wheel_tire", _wheel_tire_mesh(wheel_radius, wheel_width)),
        material=rubber_black,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.152, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.046, length=0.080),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub",
    )
    left_wheel.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.031, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="outer_cap",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=0.080),
        mass=5.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    right_wheel.visual(
        _mesh("right_wheel_tire", _wheel_tire_mesh(wheel_radius, wheel_width)),
        material=rubber_black,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.152, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.046, length=0.080),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="hub",
    )
    right_wheel.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_light,
        name="outer_cap",
    )

    hoop = model.part("retaining_hoop")
    hoop.inertial = Inertial.from_geometry(
        Box((0.390, 0.430, 0.390)),
        mass=3.2,
        origin=Origin(xyz=(0.180, 0.215, -0.180)),
    )
    eyelet_mesh = _mesh(
        "retaining_hoop_eyelet",
        _pivot_eyelet_mesh(outer_radius=0.016, inner_radius=0.0065, length=0.030),
    )
    hoop.visual(
        eyelet_mesh,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=steel_light,
        name="left_eyelet",
    )
    hoop.visual(
        eyelet_mesh,
        origin=Origin(xyz=(0.345, 0.0, 0.0)),
        material=steel_light,
        name="right_eyelet",
    )
    hoop_bar = wire_from_points(
        [
            (0.014, 0.018, -0.010),
            (0.030, 0.092, -0.040),
            (0.046, 0.220, -0.140),
            (0.064, 0.420, -0.360),
            (0.296, 0.420, -0.360),
            (0.314, 0.220, -0.140),
            (0.330, 0.092, -0.040),
            (0.346, 0.018, -0.010),
        ],
        radius=0.011,
        radial_segments=16,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.060,
        corner_segments=12,
    )
    hoop.visual(_mesh("retaining_hoop_bar", hoop_bar), material=frame_red, name="hoop_bar")

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-wheel_center_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(wheel_center_x, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "retaining_hoop_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hoop,
        origin=Origin(xyz=(-rail_half, pivot_y, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    hoop = object_model.get_part("retaining_hoop")

    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")
    hoop_hinge = object_model.get_articulation("retaining_hoop_hinge")

    frame.get_visual("cradle_shell")
    frame.get_visual("nose_plate")
    left_wheel.get_visual("tire")
    right_wheel.get_visual("tire")
    hoop.get_visual("hoop_bar")

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

    ctx.expect_contact(left_wheel, frame, name="left_wheel_contacts_axle_mount")
    ctx.expect_contact(right_wheel, frame, name="right_wheel_contacts_axle_mount")
    ctx.expect_contact(hoop, frame, name="retaining_hoop_attached_to_pivots")

    ctx.check(
        "left_wheel_spin_axis_is_axle_aligned",
        tuple(round(value, 3) for value in left_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={left_spin.axis}",
    )
    ctx.check(
        "right_wheel_spin_axis_is_axle_aligned",
        tuple(round(value, 3) for value in right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={right_spin.axis}",
    )
    ctx.check(
        "retaining_hoop_hinge_axis_is_shoulder_to_shoulder",
        tuple(round(value, 3) for value in hoop_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hoop_hinge.axis}",
    )

    left_rest = ctx.part_world_position(left_wheel)
    right_rest = ctx.part_world_position(right_wheel)
    hoop_closed = ctx.part_world_aabb(hoop)
    assert left_rest is not None
    assert right_rest is not None
    assert hoop_closed is not None

    with ctx.pose({left_spin: pi / 2.0, right_spin: -pi / 3.0}):
        left_spun = ctx.part_world_position(left_wheel)
        right_spun = ctx.part_world_position(right_wheel)
        assert left_spun is not None
        assert right_spun is not None
        ctx.check(
            "left_wheel_spins_about_fixed_mount",
            all(abs(a - b) < 1e-6 for a, b in zip(left_rest, left_spun)),
            details=f"rest={left_rest}, spun={left_spun}",
        )
        ctx.check(
            "right_wheel_spins_about_fixed_mount",
            all(abs(a - b) < 1e-6 for a, b in zip(right_rest, right_spun)),
            details=f"rest={right_rest}, spun={right_spun}",
        )
        ctx.expect_contact(left_wheel, frame, name="left_wheel_stays_mounted_while_spinning")
        ctx.expect_contact(right_wheel, frame, name="right_wheel_stays_mounted_while_spinning")

    with ctx.pose({hoop_hinge: 1.25}):
        hoop_open = ctx.part_world_aabb(hoop)
        assert hoop_open is not None
        ctx.check(
            "retaining_hoop_swings_up_over_load",
            hoop_open[0][2] > hoop_closed[0][2] + 0.22,
            details=f"closed_min_z={hoop_closed[0][2]:.4f}, open_min_z={hoop_open[0][2]:.4f}",
        )
        ctx.expect_contact(hoop, frame, name="retaining_hoop_remains_clipped_when_open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
