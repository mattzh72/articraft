from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _shell_mesh(name: str, outer_profile, inner_profile, *, segments: int = 72):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
        ),
    )


def _build_seat_cushion_mesh():
    return _save_mesh(
        "seat_cushion",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.120, 0.0),
                (0.172, 0.010),
                (0.194, 0.032),
                (0.202, 0.060),
                (0.194, 0.088),
                (0.176, 0.104),
                (0.110, 0.112),
                (0.0, 0.112),
            ],
            segments=88,
        ),
    )


def _build_backrest_pad_mesh():
    return _save_mesh(
        "backrest_pad",
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.290, 0.160, 0.032, corner_segments=8),
            0.045,
            cap=True,
            closed=True,
        )
        .rotate_x(pi / 2.0)
        .translate(0.0, -0.035, 0.270),
    )


def _build_star_leg_mesh():
    return sweep_profile_along_spline(
        [
            (0.072, 0.0, 0.126),
            (0.132, 0.0, 0.118),
            (0.220, 0.0, 0.100),
            (0.306, 0.0, 0.088),
        ],
        profile=rounded_rect_profile(0.060, 0.026, 0.010, corner_segments=6),
        samples_per_segment=14,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _add_caster(
    model: ArticulatedObject,
    base_part,
    *,
    index: int,
    angle: float,
    pivot_radius: float,
    pivot_z: float,
    housing_z: float,
    polished_aluminum,
    dark_metal,
    nylon_black,
) -> None:
    x = pivot_radius * cos(angle)
    y = pivot_radius * sin(angle)

    fork = model.part(f"caster_fork_{index}")
    fork.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_metal,
        name="top_washer",
    )
    fork.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_metal,
        name="stem",
    )
    fork.visual(
        Box((0.032, 0.046, 0.008)),
        origin=Origin(xyz=(0.018, 0.0, -0.012)),
        material=dark_metal,
        name="crown",
    )
    fork.visual(
        Box((0.016, 0.006, 0.048)),
        origin=Origin(xyz=(0.038, 0.020, -0.040)),
        material=dark_metal,
        name="left_arm",
    )
    fork.visual(
        Box((0.016, 0.006, 0.048)),
        origin=Origin(xyz=(0.038, -0.020, -0.040)),
        material=dark_metal,
        name="right_arm",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.060, 0.052, 0.060)),
        mass=0.16,
        origin=Origin(xyz=(0.026, 0.0, -0.030)),
    )

    wheel = model.part(f"caster_wheel_{index}")
    wheel.visual(
        Cylinder(radius=0.0035, length=0.034),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=polished_aluminum,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=nylon_black,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_cap",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Box((0.064, 0.034, 0.064)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        f"base_to_caster_fork_{index}",
        ArticulationType.CONTINUOUS,
        parent=base_part,
        child=fork,
        origin=Origin(xyz=(x, y, pivot_z), rpy=(0.0, 0.0, angle)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )
    model.articulation(
        f"caster_fork_to_wheel_{index}",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.038, 0.0, -0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_office_chair")

    polished_aluminum = model.material("polished_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.88, 0.91, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    nylon_black = model.material("nylon_black", rgba=(0.08, 0.08, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.19, 0.20, 0.22, 1.0))
    back_fabric = model.material("back_fabric", rgba=(0.22, 0.23, 0.26, 1.0))

    seat_cushion_mesh = _build_seat_cushion_mesh()
    backrest_pad_mesh = _build_backrest_pad_mesh()

    star_base = model.part("star_base")
    for jaw_name, geometry, origin in [
        ("socket_jaw_east", Box((0.014, 0.086, 0.120)), Origin(xyz=(0.033, 0.0, 0.092))),
        ("socket_jaw_west", Box((0.014, 0.086, 0.120)), Origin(xyz=(-0.033, 0.0, 0.092))),
        ("socket_jaw_north", Box((0.066, 0.014, 0.120)), Origin(xyz=(0.0, 0.033, 0.092))),
        ("socket_jaw_south", Box((0.066, 0.014, 0.120)), Origin(xyz=(0.0, -0.033, 0.092))),
    ]:
        star_base.visual(geometry, origin=origin, material=dark_metal, name=jaw_name)
    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        _add_member(
            star_base,
            (0.038 * cos(angle), 0.038 * sin(angle), 0.078),
            (0.292 * cos(angle), 0.292 * sin(angle), 0.084),
            radius=0.016,
            material=polished_aluminum,
            name=f"spoke_{index}",
        )
        star_base.visual(
            Cylinder(radius=0.019, length=0.010),
            origin=Origin(xyz=(0.308 * cos(angle), 0.308 * sin(angle), 0.083)),
            material=dark_metal,
            name=f"caster_socket_{index}",
        )
    star_base.inertial = Inertial.from_geometry(
        Box((0.700, 0.700, 0.190)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    gas_column = model.part("gas_column")
    gas_column.visual(
        Cylinder(radius=0.026, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=chrome,
        name="main_post",
    )
    gas_column.visual(
        Cylinder(radius=0.021, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=brushed_steel,
        name="upper_shaft",
    )
    gas_column.inertial = Inertial.from_geometry(
        Box((0.056, 0.056, 0.660)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
    )

    seat_support = model.part("seat_support")
    for jaw_name, geometry, origin in [
        ("column_jaw_east", Box((0.014, 0.074, 0.090)), Origin(xyz=(0.033, 0.0, 0.045))),
        ("column_jaw_west", Box((0.014, 0.074, 0.090)), Origin(xyz=(-0.033, 0.0, 0.045))),
        ("column_jaw_north", Box((0.066, 0.014, 0.090)), Origin(xyz=(0.0, 0.033, 0.045))),
        ("column_jaw_south", Box((0.066, 0.014, 0.090)), Origin(xyz=(0.0, -0.033, 0.045))),
    ]:
        seat_support.visual(geometry, origin=origin, material=dark_metal, name=jaw_name)
    seat_support.visual(
        Box((0.044, 0.132, 0.028)),
        origin=Origin(xyz=(-0.050, 0.0, 0.074)),
        material=charcoal,
        name="left_tilt_housing",
    )
    seat_support.visual(
        Box((0.044, 0.132, 0.028)),
        origin=Origin(xyz=(0.050, 0.0, 0.074)),
        material=charcoal,
        name="right_tilt_housing",
    )
    seat_support.visual(
        Box((0.132, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.064, 0.094)),
        material=charcoal,
        name="front_plate_bridge",
    )
    seat_support.visual(
        Box((0.100, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, -0.064, 0.094)),
        material=charcoal,
        name="rear_plate_bridge",
    )
    seat_support.visual(
        Box((0.064, 0.166, 0.014)),
        origin=Origin(xyz=(-0.060, 0.0, 0.103)),
        material=dark_metal,
        name="left_mount_plate",
    )
    seat_support.visual(
        Box((0.064, 0.166, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, 0.103)),
        material=dark_metal,
        name="right_mount_plate",
    )
    seat_support.visual(
        Box((0.090, 0.220, 0.050)),
        origin=Origin(xyz=(0.0, -0.145, 0.070)),
        material=charcoal,
        name="back_spine",
    )
    seat_support.visual(
        Box((0.078, 0.024, 0.120)),
        origin=Origin(xyz=(0.0, -0.214, 0.138)),
        material=charcoal,
        name="back_bracket",
    )
    seat_support.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(0.076, 0.0, 0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="height_lever",
    )
    seat_support.inertial = Inertial.from_geometry(
        Box((0.210, 0.230, 0.130)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.030, 0.065)),
    )

    seat = model.part("seat")
    seat.visual(
        seat_cushion_mesh,
        material=seat_fabric,
        name="seat_cushion",
    )
    seat.visual(
        Cylinder(radius=0.155, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=charcoal,
        name="underside_pan",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.202, length=0.112),
        mass=5.7,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.090, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=charcoal,
        name="mount_block",
    )
    _add_member(
        backrest,
        (-0.030, 0.0, -0.010),
        (-0.085, -0.030, 0.235),
        radius=0.0075,
        material=polished_aluminum,
        name="left_post",
    )
    _add_member(
        backrest,
        (0.030, 0.0, -0.010),
        (0.085, -0.030, 0.235),
        radius=0.0075,
        material=polished_aluminum,
        name="right_post",
    )
    _add_member(
        backrest,
        (-0.085, -0.030, 0.210),
        (0.085, -0.030, 0.210),
        radius=0.006,
        material=dark_metal,
        name="upper_crossbar",
    )
    backrest.visual(
        backrest_pad_mesh,
        material=back_fabric,
        name="back_pad",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.320, 0.090, 0.340)),
        mass=1.9,
        origin=Origin(xyz=(0.0, -0.018, 0.170)),
    )

    foot_ring = model.part("foot_ring")
    for jaw_name, geometry, origin in [
        ("clamp_jaw_east", Box((0.014, 0.066, 0.048)), Origin(xyz=(0.033, 0.0, 0.0))),
        ("clamp_jaw_west", Box((0.014, 0.066, 0.048)), Origin(xyz=(-0.033, 0.0, 0.0))),
        ("clamp_jaw_north", Box((0.066, 0.014, 0.048)), Origin(xyz=(0.0, 0.033, 0.0))),
        ("clamp_jaw_south", Box((0.066, 0.014, 0.048)), Origin(xyz=(0.0, -0.033, 0.0))),
    ]:
        foot_ring.visual(geometry, origin=origin, material=dark_metal, name=jaw_name)
    foot_ring.visual(
        _save_mesh(
            "foot_ring_hoop",
            TorusGeometry(radius=0.218, tube=0.011, radial_segments=18, tubular_segments=48),
        ),
        material=polished_aluminum,
        name="foot_hoop",
    )
    for brace_index, (inner, outer) in enumerate(
        (
            ((0.038, 0.0, 0.0), (0.207, 0.0, 0.0)),
            ((0.0, 0.038, 0.0), (0.0, 0.207, 0.0)),
            ((-0.038, 0.0, 0.0), (-0.207, 0.0, 0.0)),
            ((0.0, -0.038, 0.0), (0.0, -0.207, 0.0)),
        )
    ):
        _add_member(
            foot_ring,
            inner,
            outer,
            radius=0.006,
            material=polished_aluminum,
            name=f"brace_{brace_index}",
        )
    foot_ring.inertial = Inertial.from_geometry(
        Box((0.460, 0.460, 0.050)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    pivot_radius = 0.308
    pivot_z = 0.083
    housing_z = 0.083
    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        _add_caster(
            model,
            star_base,
            index=index,
            angle=angle,
            pivot_radius=pivot_radius,
            pivot_z=pivot_z,
            housing_z=housing_z,
            polished_aluminum=polished_aluminum,
            dark_metal=dark_metal,
            nylon_black=nylon_black,
        )

    model.articulation(
        "base_to_gas_column",
        ArticulationType.PRISMATIC,
        parent=star_base,
        child=gas_column,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.25, lower=-0.050, upper=0.100),
    )
    model.articulation(
        "gas_column_to_seat_support",
        ArticulationType.CONTINUOUS,
        parent=gas_column,
        child=seat_support,
        origin=Origin(xyz=(0.0, 0.0, 0.548)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "seat_support_to_seat",
        ArticulationType.FIXED,
        parent=seat_support,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )
    model.articulation(
        "seat_support_to_backrest",
        ArticulationType.FIXED,
        parent=seat_support,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.214, 0.218)),
    )
    model.articulation(
        "gas_column_to_foot_ring",
        ArticulationType.PRISMATIC,
        parent=gas_column,
        child=foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=-0.070, upper=0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    for index in range(5):
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_b="axle",
            reason="caster wheel axle intentionally passes through the fork arms",
        )
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="left_arm",
            elem_b="axle",
            reason="caster fork side arm is modeled without an axle hole",
        )
        ctx.allow_overlap(
            f"caster_fork_{index}",
            f"caster_wheel_{index}",
            elem_a="right_arm",
            elem_b="axle",
            reason="caster fork side arm is modeled without an axle hole",
        )

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

    part_map = {part.name: part for part in object_model.parts}
    articulation_map = {joint.name: joint for joint in object_model.articulations}

    required_parts = [
        "star_base",
        "gas_column",
        "seat_support",
        "seat",
        "backrest",
        "foot_ring",
        *[f"caster_fork_{index}" for index in range(5)],
        *[f"caster_wheel_{index}" for index in range(5)],
    ]
    required_articulations = [
        "base_to_gas_column",
        "gas_column_to_seat_support",
        "seat_support_to_seat",
        "seat_support_to_backrest",
        "gas_column_to_foot_ring",
        *[f"base_to_caster_fork_{index}" for index in range(5)],
        *[f"caster_fork_to_wheel_{index}" for index in range(5)],
    ]

    for part_name in required_parts:
        ctx.check(f"part_{part_name}_present", part_name in part_map, f"missing part: {part_name}")
    for articulation_name in required_articulations:
        ctx.check(
            f"articulation_{articulation_name}_present",
            articulation_name in articulation_map,
            f"missing articulation: {articulation_name}",
        )

    if not all(name in part_map for name in required_parts) or not all(
        name in articulation_map for name in required_articulations
    ):
        return ctx.report()

    star_base = part_map["star_base"]
    gas_column = part_map["gas_column"]
    seat_support = part_map["seat_support"]
    seat = part_map["seat"]
    backrest = part_map["backrest"]
    foot_ring = part_map["foot_ring"]

    gas_lift = articulation_map["base_to_gas_column"]
    seat_swivel = articulation_map["gas_column_to_seat_support"]
    foot_ring_slide = articulation_map["gas_column_to_foot_ring"]

    ctx.check(
        "gas_column_axis_vertical",
        gas_lift.axis == (0.0, 0.0, 1.0),
        f"expected vertical gas lift axis, got {gas_lift.axis}",
    )
    ctx.check(
        "seat_swivel_axis_vertical",
        seat_swivel.axis == (0.0, 0.0, 1.0),
        f"expected seat swivel axis, got {seat_swivel.axis}",
    )
    ctx.check(
        "foot_ring_axis_vertical",
        foot_ring_slide.axis == (0.0, 0.0, 1.0),
        f"expected vertical foot ring slide axis, got {foot_ring_slide.axis}",
    )

    ctx.expect_contact(
        gas_column,
        star_base,
        name="gas_column_clipped_into_base",
    )
    ctx.expect_contact(
        gas_column,
        seat_support,
        name="gas_column_clipped_into_seat_support",
    )
    ctx.expect_contact(
        seat,
        seat_support,
        name="seat_supported_by_mechanism",
    )
    ctx.expect_contact(
        backrest,
        seat_support,
        elem_a="mount_block",
        elem_b="back_bracket",
        name="backrest_mounted_to_support",
    )
    ctx.expect_contact(
        foot_ring,
        gas_column,
        name="foot_ring_clamped_to_column",
    )

    ctx.expect_gap(
        seat,
        star_base,
        axis="z",
        min_gap=0.48,
        max_gap=0.78,
        name="seat_is_stool_height_above_base",
    )
    ctx.expect_gap(
        seat,
        foot_ring,
        axis="z",
        min_gap=0.18,
        max_gap=0.42,
        name="foot_ring_sits_below_seat",
    )
    ctx.expect_origin_gap(
        seat,
        backrest,
        axis="y",
        min_gap=0.05,
        name="backrest_is_behind_seat",
    )
    ctx.expect_origin_gap(
        backrest,
        seat,
        axis="z",
        min_gap=0.095,
        name="backrest_rises_above_seat",
    )

    for index in range(5):
        fork = part_map[f"caster_fork_{index}"]
        wheel = part_map[f"caster_wheel_{index}"]
        fork_swivel = articulation_map[f"base_to_caster_fork_{index}"]
        wheel_spin = articulation_map[f"caster_fork_to_wheel_{index}"]

        ctx.check(
            f"caster_fork_{index}_swivel_axis",
            fork_swivel.axis == (0.0, 0.0, 1.0),
            f"expected caster swivel axis (0,0,1), got {fork_swivel.axis}",
        )
        ctx.check(
            f"caster_wheel_{index}_spin_axis",
            wheel_spin.axis == (0.0, 1.0, 0.0),
            f"expected wheel spin axis (0,1,0), got {wheel_spin.axis}",
        )
        ctx.expect_contact(
            fork,
            star_base,
            elem_a="top_washer",
            elem_b=f"caster_socket_{index}",
            name=f"caster_fork_{index}_mounted_to_base",
        )
        ctx.expect_contact(
            wheel,
            fork,
            name=f"caster_wheel_{index}_mounted_in_fork",
        )

    with ctx.pose(
        {
            gas_lift: 0.070,
            seat_swivel: 1.0,
            foot_ring_slide: 0.050,
            articulation_map["base_to_caster_fork_0"]: 0.8,
            articulation_map["caster_fork_to_wheel_0"]: 1.2,
        }
    ):
        ctx.expect_contact(
            gas_column,
            star_base,
            name="gas_column_remains_carried_by_base_when_extended",
        )
        ctx.expect_contact(
            gas_column,
            seat_support,
            name="seat_support_remains_carried_by_column_when_swiveled",
        )
        ctx.expect_contact(
            foot_ring,
            gas_column,
            name="foot_ring_remains_clamped_when_adjusted",
        )
        ctx.expect_contact(
            part_map["caster_fork_0"],
            part_map["caster_wheel_0"],
            name="caster_zero_stays_assembled_in_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
