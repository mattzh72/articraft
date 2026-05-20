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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _profile_loop(
    profile: list[tuple[float, float]],
    *,
    z: float,
    dx: float = 0.0,
    dy: float = 0.0,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
) -> list[tuple[float, float, float]]:
    return [(dx + x * scale_x, dy + y * scale_y, z) for x, y in profile]


def _spoke_mesh() -> object:
    profile = rounded_rect_profile(0.056, 0.024, 0.008, corner_segments=8)
    geom = sweep_profile_along_spline(
        [
            (0.000, 0.000, 0.128),
            (0.130, 0.000, 0.122),
            (0.220, 0.000, 0.115),
            (0.275, 0.000, 0.109),
        ],
        profile=profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    return _mesh("chair_spoke_v2", geom)


def _lower_shroud_mesh() -> object:
    outer = [
        (0.060, 0.000),
        (0.054, 0.090),
        (0.044, 0.180),
        (0.036, 0.247),
    ]
    inner = [
        (0.023, 0.000),
        (0.023, 0.090),
        (0.023, 0.180),
        (0.023, 0.247),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return _mesh("chair_lower_shroud", geom)


def _seat_shell_mesh() -> object:
    base = rounded_rect_profile(0.500, 0.465, 0.052, corner_segments=8)
    mid = rounded_rect_profile(0.520, 0.482, 0.062, corner_segments=8)
    top = rounded_rect_profile(0.492, 0.438, 0.050, corner_segments=8)
    geom = LoftGeometry(
        [
            _profile_loop(base, z=0.000),
            _profile_loop(mid, z=0.028, dy=0.010),
            _profile_loop(top, z=0.060, dy=0.018, scale_x=0.98, scale_y=0.94),
        ],
        cap=True,
        closed=True,
    )
    return _mesh("chair_seat_shell", geom)


def _back_ring_mesh() -> object:
    outer = rounded_rect_profile(0.470, 0.640, 0.085, corner_segments=10)
    inner = rounded_rect_profile(0.360, 0.520, 0.060, corner_segments=10)
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        0.028,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return _mesh("chair_back_ring", geom)


def _back_mesh_panel() -> object:
    panel = rounded_rect_profile(0.376, 0.536, 0.056, corner_segments=10)
    geom = ExtrudeWithHolesGeometry(
        panel,
        [],
        0.004,
        center=True,
        cap=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return _mesh("chair_back_panel_v2", geom)


def _wheel_tread_mesh() -> object:
    profile = [
        (0.000, -0.009),
        (0.018, -0.009),
        (0.022, -0.0075),
        (0.026, -0.003),
        (0.028, 0.000),
        (0.026, 0.003),
        (0.022, 0.0075),
        (0.018, 0.009),
        (0.000, 0.009),
    ]
    geom = LatheGeometry(profile, segments=44)
    geom.rotate_x(math.pi / 2.0)
    return _mesh("chair_caster_wheel_v2", geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mesh_back_office_chair")

    polished_aluminum = model.material("polished_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.15, 0.16, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.17, 0.19, 0.22, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.10, 0.11, 0.12, 0.42))
    chrome = model.material("chrome", rgba=(0.73, 0.75, 0.79, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    star_base = model.part("star_base")
    star_base.visual(
        Cylinder(radius=0.092, length=0.034),
        origin=Origin(xyz=(0.000, 0.000, 0.126)),
        material=polished_aluminum,
        name="hub_shell",
    )
    star_base.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.000, 0.000, 0.109)),
        material=graphite,
        name="hub_core",
    )
    star_base.visual(
        _lower_shroud_mesh(),
        origin=Origin(xyz=(0.000, 0.000, 0.128)),
        material=graphite,
        name="lower_shroud",
    )
    caster_angles = [math.pi / 2.0 + (2.0 * math.pi * index / 5.0) for index in range(5)]
    caster_radius = 0.286
    for index, angle in enumerate(caster_angles):
        spoke_x = math.cos(angle) * 0.150
        spoke_y = math.sin(angle) * 0.150
        tip_x = math.cos(angle) * caster_radius
        tip_y = math.sin(angle) * caster_radius
        star_base.visual(
            Box((0.300, 0.050, 0.018)),
            origin=Origin(xyz=(spoke_x, spoke_y, 0.104), rpy=(0.0, 0.0, angle)),
            material=polished_aluminum,
            name=f"spoke_{index}",
        )
    star_base.inertial = Inertial.from_geometry(
        Box((0.720, 0.720, 0.285)),
        mass=8.0,
        origin=Origin(xyz=(0.000, 0.000, 0.142)),
    )

    column_carriage = model.part("column_carriage")
    column_carriage.visual(
        Cylinder(radius=0.019, length=0.120),
        origin=Origin(xyz=(0.000, 0.000, -0.060)),
        material=chrome,
        name="upper_column",
    )
    column_carriage.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=graphite,
        name="top_plate",
    )
    column_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.132),
        mass=2.4,
        origin=Origin(xyz=(0.000, 0.000, -0.054)),
    )

    seat_frame = model.part("seat_frame")
    seat_frame.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=graphite,
        name="turntable_plate",
    )
    seat_frame.visual(
        Box((0.205, 0.160, 0.028)),
        origin=Origin(xyz=(0.000, -0.010, 0.024)),
        material=dark_plastic,
        name="seat_mechanism",
    )
    seat_frame.visual(
        _seat_shell_mesh(),
        origin=Origin(xyz=(0.000, 0.012, 0.038)),
        material=seat_fabric,
        name="seat_shell",
    )
    seat_frame.visual(
        Box((0.300, 0.028, 0.032)),
        origin=Origin(xyz=(0.000, -0.218, 0.078)),
        material=dark_plastic,
        name="rear_bracket_bridge",
    )
    seat_frame.visual(
        Box((0.010, 0.020, 0.120)),
        origin=Origin(xyz=(-0.155, -0.234, 0.122)),
        material=dark_plastic,
        name="rear_bracket_left",
    )
    seat_frame.visual(
        Box((0.010, 0.020, 0.120)),
        origin=Origin(xyz=(0.155, -0.234, 0.122)),
        material=dark_plastic,
        name="rear_bracket_right",
    )
    seat_frame.visual(
        Box((0.026, 0.130, 0.078)),
        origin=Origin(xyz=(-0.102, -0.150, 0.070)),
        material=dark_plastic,
        name="rear_support_left",
    )
    seat_frame.visual(
        Box((0.026, 0.130, 0.078)),
        origin=Origin(xyz=(0.102, -0.150, 0.070)),
        material=dark_plastic,
        name="rear_support_right",
    )
    seat_frame.inertial = Inertial.from_geometry(
        Box((0.520, 0.500, 0.220)),
        mass=5.2,
        origin=Origin(xyz=(0.000, 0.000, 0.095)),
    )

    back_frame = model.part("back_frame")
    back_frame.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(-0.146, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="left_hinge_boss",
    )
    back_frame.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.146, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="right_hinge_boss",
    )
    back_frame.visual(
        Box((0.040, 0.018, 0.080)),
        origin=Origin(xyz=(-0.128, -0.018, 0.048)),
        material=graphite,
        name="left_lower_link",
    )
    back_frame.visual(
        Box((0.040, 0.018, 0.080)),
        origin=Origin(xyz=(0.128, -0.018, 0.048)),
        material=graphite,
        name="right_lower_link",
    )
    back_frame.visual(
        Box((0.028, 0.024, 0.320)),
        origin=Origin(xyz=(-0.120, -0.018, 0.180)),
        material=graphite,
        name="left_upright",
    )
    back_frame.visual(
        Box((0.028, 0.024, 0.320)),
        origin=Origin(xyz=(0.120, -0.018, 0.180)),
        material=graphite,
        name="right_upright",
    )
    back_frame.visual(
        Box((0.250, 0.010, 0.050)),
        origin=Origin(xyz=(0.000, -0.020, 0.265)),
        material=graphite,
        name="lower_frame_bridge",
    )
    back_frame.visual(
        _back_ring_mesh(),
        origin=Origin(xyz=(0.000, -0.018, 0.400)),
        material=graphite,
        name="back_ring",
    )
    back_frame.visual(
        _back_mesh_panel(),
        origin=Origin(xyz=(0.000, -0.020, 0.405)),
        material=mesh_black,
        name="mesh_panel",
    )
    back_frame.inertial = Inertial.from_geometry(
        Box((0.500, 0.060, 0.690)),
        mass=3.4,
        origin=Origin(xyz=(0.000, -0.018, 0.360)),
    )

    for index, angle in enumerate(caster_angles):
        tip_x = math.cos(angle) * caster_radius
        tip_y = math.sin(angle) * caster_radius

        fork = model.part(f"caster_fork_{index}")
        fork.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(0.000, 0.000, -0.004)),
            material=graphite,
            name="top_cap",
        )
        fork.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(0.000, 0.000, -0.018)),
            material=graphite,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.020, 0.028, 0.012)),
            origin=Origin(xyz=(0.000, 0.000, -0.034)),
            material=dark_plastic,
            name="crown",
        )
        fork.visual(
            Box((0.018, 0.004, 0.032)),
            origin=Origin(xyz=(0.000, -0.012, -0.054)),
            material=dark_plastic,
            name="left_cheek",
        )
        fork.visual(
            Box((0.018, 0.004, 0.032)),
            origin=Origin(xyz=(0.000, 0.012, -0.054)),
            material=dark_plastic,
            name="right_cheek",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.040, 0.040, 0.090)),
            mass=0.24,
            origin=Origin(xyz=(0.000, 0.000, -0.042)),
        )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="wheel_tread",
        )
        wheel.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="wheel_hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.028, length=0.014),
            mass=0.16,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"base_to_caster_fork_{index}",
            ArticulationType.CONTINUOUS,
            parent=star_base,
            child=fork,
            origin=Origin(xyz=(tip_x, tip_y, 0.095)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=8.0),
        )
        model.articulation(
            f"caster_fork_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.000, 0.000, -0.066)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=18.0),
        )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=star_base,
        child=column_carriage,
        origin=Origin(xyz=(0.000, 0.000, 0.375)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.20,
            lower=0.000,
            upper=0.120,
        ),
    )
    model.articulation(
        "column_to_seat",
        ArticulationType.CONTINUOUS,
        parent=column_carriage,
        child=seat_frame,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=3.0,
        ),
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat_frame,
        child=back_frame,
        origin=Origin(xyz=(0.000, -0.234, 0.122)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-0.12,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    star_base = object_model.get_part("star_base")
    column_carriage = object_model.get_part("column_carriage")
    seat_frame = object_model.get_part("seat_frame")
    back_frame = object_model.get_part("back_frame")
    column_slide = object_model.get_articulation("base_to_column")
    seat_swivel = object_model.get_articulation("column_to_seat")
    back_recline = object_model.get_articulation("seat_to_back")

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
        "column_slide_axis",
        tuple(column_slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic axis, got {column_slide.axis}",
    )
    ctx.check(
        "seat_swivel_axis",
        tuple(seat_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical swivel axis, got {seat_swivel.axis}",
    )
    ctx.check(
        "back_recline_axis",
        tuple(back_recline.axis) == (1.0, 0.0, 0.0),
        details=f"expected transverse recline axis, got {back_recline.axis}",
    )

    slide_limits = column_slide.motion_limits
    recline_limits = back_recline.motion_limits
    ctx.check(
        "column_slide_range",
        slide_limits is not None
        and slide_limits.lower == 0.0
        and slide_limits.upper is not None
        and 0.10 <= slide_limits.upper <= 0.14,
        details=f"unexpected column travel limits: {slide_limits}",
    )
    ctx.check(
        "back_recline_range",
        recline_limits is not None
        and recline_limits.lower is not None
        and recline_limits.upper is not None
        and recline_limits.lower < 0.0
        and recline_limits.upper > 0.30,
        details=f"unexpected back recline limits: {recline_limits}",
    )

    ctx.expect_origin_distance(
        seat_frame,
        column_carriage,
        axes="xy",
        max_dist=0.001,
        name="seat_centered_on_column",
    )
    ctx.expect_origin_gap(
        seat_frame,
        star_base,
        axis="z",
        min_gap=0.34,
        max_gap=0.52,
        name="seat_above_star_base",
    )
    ctx.expect_origin_gap(
        seat_frame,
        back_frame,
        axis="y",
        min_gap=0.18,
        max_gap=0.30,
        name="back_hinged_behind_seat",
    )
    ctx.expect_origin_distance(
        seat_frame,
        back_frame,
        axes="x",
        max_dist=0.001,
        name="back_centered_with_seat",
    )
    ctx.expect_contact(
        column_carriage,
        seat_frame,
        name="seat_turntable_contacts_column",
    )
    ctx.expect_contact(
        seat_frame,
        back_frame,
        name="back_hinge_contacts_rear_bracket",
    )
    ctx.expect_overlap(
        seat_frame,
        star_base,
        axes="xy",
        min_overlap=0.05,
        name="seat_projects_over_base",
    )

    for index in range(5):
        fork = object_model.get_part(f"caster_fork_{index}")
        wheel = object_model.get_part(f"caster_wheel_{index}")
        fork_swivel = object_model.get_articulation(f"base_to_caster_fork_{index}")
        wheel_spin = object_model.get_articulation(f"caster_fork_to_wheel_{index}")

        ctx.check(
            f"caster_fork_{index}_axis",
            tuple(fork_swivel.axis) == (0.0, 0.0, 1.0),
            details=f"expected vertical swivel, got {fork_swivel.axis}",
        )
        ctx.check(
            f"caster_wheel_{index}_axis",
            tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
            details=f"expected wheel axle along y, got {wheel_spin.axis}",
        )
        ctx.expect_contact(
            star_base,
            fork,
            name=f"caster_fork_{index}_mounted_to_spoke",
        )
        ctx.expect_contact(
            fork,
            wheel,
            name=f"caster_wheel_{index}_captured_in_fork",
        )
        ctx.expect_origin_distance(
            star_base,
            fork,
            axes="xy",
            min_dist=0.28,
            max_dist=0.34,
            name=f"caster_fork_{index}_at_spoke_tip_radius",
        )
        ctx.expect_origin_gap(
            fork,
            wheel,
            axis="z",
            min_gap=0.06,
            max_gap=0.09,
            name=f"caster_wheel_{index}_below_fork",
        )

    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({column_slide: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="column_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="column_lower_no_floating")
            ctx.expect_contact(
                column_carriage,
                seat_frame,
                name="column_lower_seat_contact",
            )
        with ctx.pose({column_slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="column_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="column_upper_no_floating")
            ctx.expect_contact(
                column_carriage,
                seat_frame,
                name="column_upper_seat_contact",
            )
            ctx.expect_origin_gap(
                seat_frame,
                star_base,
                axis="z",
                min_gap=0.44,
                max_gap=0.58,
                name="seat_height_raises_with_column",
            )

    if recline_limits is not None and recline_limits.lower is not None and recline_limits.upper is not None:
        with ctx.pose({back_recline: recline_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="back_recline_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="back_recline_lower_no_floating")
            ctx.expect_contact(
                seat_frame,
                back_frame,
                name="back_recline_lower_hinge_contact",
            )
        with ctx.pose({back_recline: recline_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="back_recline_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="back_recline_upper_no_floating")
            ctx.expect_contact(
                seat_frame,
                back_frame,
                name="back_recline_upper_hinge_contact",
            )

    with ctx.pose(
        {
            seat_swivel: math.pi / 3.0,
            back_recline: 0.30,
            "base_to_caster_fork_0": math.pi / 2.0,
            "base_to_caster_fork_1": -math.pi / 3.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="compound_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="compound_pose_no_floating")
        ctx.expect_contact(
            column_carriage,
            seat_frame,
            name="compound_pose_seat_turntable_contact",
        )
        ctx.expect_contact(
            seat_frame,
            back_frame,
            name="compound_pose_back_hinge_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
