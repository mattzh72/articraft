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
    MotionLimits,
    rounded_rect_profile,
    section_loft,
    TestContext,
    TestReport,
    Origin,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(width: float, thickness: float, radius: float, x_pos: float):
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width, thickness, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.77, 1.0))
    accent = model.material("accent", rgba=(0.77, 0.19, 0.15, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.10, 0.54, 1.18)),
        mass=42.0,
        origin=Origin(xyz=(0.05, 0.0, 0.59)),
    )

    frame.visual(
        Box((0.12, 0.52, 0.05)),
        origin=Origin(xyz=(-0.40, 0.0, 0.025)),
        material=charcoal,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.12, 0.46, 0.05)),
        origin=Origin(xyz=(0.54, 0.0, 0.025)),
        material=charcoal,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.96, 0.11, 0.09)),
        origin=Origin(xyz=(0.07, 0.0, 0.075)),
        material=graphite,
        name="base_rail",
    )
    frame.visual(
        Box((0.26, 0.035, 0.18)),
        origin=Origin(xyz=(0.20, 0.060, 0.185)),
        material=graphite,
        name="drive_tray_side",
    )
    frame.visual(
        Box((0.22, 0.10, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, 0.092)),
        material=graphite,
        name="bottom_bracket_block",
    )
    bottom_bracket_shell = LatheGeometry.from_shell_profiles(
        [(0.034, -0.055), (0.034, 0.055)],
        [(0.020, -0.051), (0.020, 0.051)],
        segments=40,
    ).rotate_x(-pi / 2.0)
    frame.visual(
        _mesh("bottom_bracket_shell", bottom_bracket_shell),
        origin=Origin(xyz=(-0.03, 0.0, 0.18)),
        material=silver,
        name="bottom_bracket_shell",
    )
    bottom_bracket_cup = LatheGeometry.from_shell_profiles(
        [(0.028, -0.008), (0.028, 0.008)],
        [(0.017, -0.007), (0.017, 0.007)],
        segments=36,
    ).rotate_x(-pi / 2.0)
    frame.visual(
        _mesh("bottom_bracket_cup", bottom_bracket_cup),
        origin=Origin(xyz=(-0.03, -0.060, 0.18)),
        material=silver,
        name="left_bottom_bracket_cup",
    )
    frame.visual(
        _mesh("bottom_bracket_cup_mirror", bottom_bracket_cup),
        origin=Origin(xyz=(-0.03, 0.060, 0.18)),
        material=silver,
        name="right_bottom_bracket_cup",
    )
    frame.visual(
        Box((0.12, 0.10, 0.06)),
        origin=Origin(xyz=(0.06, 0.0, 0.165)),
        material=graphite,
        name="front_spine_bridge",
    )

    rear_stay_left = tube_from_spline_points(
        [
            (-0.31, -0.060, 0.11),
            (-0.28, -0.058, 0.25),
            (-0.23, -0.057, 0.46),
            (-0.18, -0.055, 0.63),
        ],
        radius=0.019,
        samples_per_segment=16,
        radial_segments=18,
    )
    rear_stay_right = tube_from_spline_points(
        [
            (-0.31, 0.060, 0.11),
            (-0.28, 0.058, 0.25),
            (-0.23, 0.057, 0.46),
            (-0.18, 0.055, 0.63),
        ],
        radius=0.019,
        samples_per_segment=16,
        radial_segments=18,
    )
    front_spine = tube_from_spline_points(
        [
            (0.00, 0.0, 0.20),
            (0.10, 0.0, 0.34),
            (0.22, 0.0, 0.52),
            (0.30, 0.0, 0.72),
        ],
        radius=0.037,
        samples_per_segment=18,
        radial_segments=20,
    )
    handlebar_shape = tube_from_spline_points(
        [
            (0.12, -0.21, 0.98),
            (0.20, -0.17, 1.05),
            (0.30, -0.08, 1.09),
            (0.34, 0.0, 1.10),
            (0.30, 0.08, 1.09),
            (0.20, 0.17, 1.05),
            (0.12, 0.21, 0.98),
        ],
        radius=0.016,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.visual(_mesh("rear_stay_left", rear_stay_left), material=graphite, name="rear_stay_left")
    frame.visual(_mesh("rear_stay_right", rear_stay_right), material=graphite, name="rear_stay_right")
    frame.visual(_mesh("front_spine", front_spine), material=graphite, name="front_spine")
    frame.visual(
        Cylinder(radius=0.040, length=0.54),
        origin=Origin(xyz=(0.30, 0.0, 0.66), rpy=(0.0, 0.14, 0.0)),
        material=graphite,
        name="handlebar_mast",
    )
    frame.visual(
        Box((0.12, 0.09, 0.12)),
        origin=Origin(xyz=(0.24, 0.0, 0.42)),
        material=graphite,
        name="mast_base_block",
    )
    frame.visual(
        Box((0.11, 0.06, 0.05)),
        origin=Origin(xyz=(0.33, 0.0, 1.05)),
        material=silver,
        name="handlebar_stem",
    )
    frame.visual(_mesh("handlebar", handlebar_shape), material=graphite, name="handlebar")
    frame.visual(
        Box((0.19, 0.11, 0.05)),
        origin=Origin(xyz=(0.33, 0.0, 0.97)),
        material=charcoal,
        name="console_tray",
    )
    frame.visual(
        Box((0.10, 0.12, 0.14)),
        origin=Origin(xyz=(0.31, 0.0, 0.95)),
        material=graphite,
        name="console_neck",
    )
    frame.visual(
        Box((0.08, 0.12, 0.12)),
        origin=Origin(xyz=(0.31, 0.0, 1.04)),
        material=silver,
        name="bar_clamp",
    )
    seat_sleeve_shell = LatheGeometry.from_shell_profiles(
        [(0.036, 0.0), (0.036, 0.30)],
        [(0.030, 0.004), (0.030, 0.296)],
        segments=40,
    )
    frame.visual(
        _mesh("seat_sleeve", seat_sleeve_shell),
        origin=Origin(xyz=(-0.18, 0.0, 0.395)),
        material=graphite,
        name="seat_sleeve",
    )
    frame.visual(
        Box((0.10, 0.05, 0.06)),
        origin=Origin(xyz=(-0.18, 0.059, 0.69)),
        material=accent,
        name="seat_clamp_top",
    )
    frame.visual(
        Box((0.03, 0.04, 0.04)),
        origin=Origin(xyz=(-0.18, 0.045, 0.69)),
        material=accent,
        name="seat_clamp_bridge",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.28),
        origin=Origin(xyz=(-0.18, 0.0, 0.26)),
        material=graphite,
        name="seat_column",
    )

    housing_cover = LatheGeometry.from_shell_profiles(
        [
            (0.055, 0.000),
            (0.170, 0.000),
            (0.215, 0.012),
            (0.235, 0.030),
            (0.230, 0.054),
            (0.180, 0.080),
            (0.080, 0.092),
        ],
        [
            (0.070, 0.010),
            (0.165, 0.010),
            (0.198, 0.020),
            (0.212, 0.035),
            (0.208, 0.050),
            (0.170, 0.066),
            (0.090, 0.074),
        ],
        segments=56,
    ).rotate_x(-pi / 2.0)
    frame.visual(
        _mesh("flywheel_housing_cover", housing_cover),
        origin=Origin(xyz=(0.30, 0.030, 0.25)),
        material=charcoal,
        name="flywheel_housing_cover",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.30, 0.056, 0.25), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="flywheel_bearing_boss",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.04),
        mass=9.0,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.095, length=0.040),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="flywheel_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.072, length=0.026),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.028, length=0.052),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="flywheel_hub",
    )

    crank = model.part("crank")
    crank.inertial = Inertial.from_geometry(
        Box((0.38, 0.24, 0.30)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crank.visual(
        Cylinder(radius=0.016, length=0.200),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="bottom_bracket_axle",
    )
    crank.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, -0.072, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_bearing_collar",
    )
    crank.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.072, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_bearing_collar",
    )
    crank.visual(
        Cylinder(radius=0.090, length=0.008),
        origin=Origin(xyz=(0.0, 0.086, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="chainring",
    )
    crank.visual(
        Cylinder(radius=0.055, length=0.015),
        origin=Origin(xyz=(0.0, 0.074, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="spider_hub",
    )
    right_crank_arm = tube_from_spline_points(
        [
            (0.0, 0.096, 0.0),
            (0.008, 0.098, -0.040),
            (0.010, 0.104, -0.095),
            (0.006, 0.112, -0.145),
        ],
        radius=0.013,
        samples_per_segment=14,
        radial_segments=16,
    )
    left_crank_arm = tube_from_spline_points(
        [
            (0.0, -0.096, 0.0),
            (-0.008, -0.098, 0.040),
            (-0.010, -0.104, 0.095),
            (-0.006, -0.112, 0.145),
        ],
        radius=0.013,
        samples_per_segment=14,
        radial_segments=16,
    )
    crank.visual(_mesh("right_crank_arm", right_crank_arm), material=graphite, name="right_crank_arm")
    crank.visual(_mesh("left_crank_arm", left_crank_arm), material=graphite, name="left_crank_arm")
    crank.visual(
        Box((0.100, 0.030, 0.020)),
        origin=Origin(xyz=(0.006, 0.122, -0.155)),
        material=rubber,
        name="right_pedal",
    )
    crank.visual(
        Box((0.100, 0.030, 0.020)),
        origin=Origin(xyz=(-0.006, -0.122, 0.155)),
        material=rubber,
        name="left_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, 0.56)),
        mass=5.5,
        origin=Origin(xyz=(0.015, 0.0, 0.28)),
    )
    seat_post.visual(
        Cylinder(radius=0.028, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=silver,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.060, 0.040, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.445)),
        material=graphite,
        name="saddle_stem",
    )
    seat_post.visual(
        Box((0.085, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=graphite,
        name="seat_yoke",
    )
    saddle_shell = section_loft(
        [
            _yz_section(0.16, 0.05, 0.020, -0.11),
            _yz_section(0.18, 0.06, 0.024, -0.03),
            _yz_section(0.13, 0.05, 0.020, 0.06),
            _yz_section(0.06, 0.03, 0.012, 0.14),
        ]
    )
    seat_post.visual(
        _mesh("saddle_shell", saddle_shell),
        origin=Origin(xyz=(0.01, 0.0, 0.49)),
        material=charcoal,
        name="saddle_shell",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.30, 0.0, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=24.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(-0.03, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "seat_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.18, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    flywheel = object_model.get_part("flywheel")
    crank_spin = object_model.get_articulation("crank_spin")
    flywheel_spin = object_model.get_articulation("flywheel_spin")
    seat_slide = object_model.get_articulation("seat_slide")

    ctx.check(
        "crank joint is continuous",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_spin.articulation_type}",
    )
    ctx.check(
        "flywheel joint is continuous",
        flywheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={flywheel_spin.articulation_type}",
    )
    ctx.check(
        "seat post joint is prismatic",
        seat_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={seat_slide.articulation_type}",
    )

    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.010,
        name="seat post stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.28,
        name="seat post remains inserted in sleeve at rest",
    )
    ctx.expect_overlap(
        flywheel,
        frame,
        axes="xz",
        elem_a="flywheel_rim",
        elem_b="flywheel_housing_cover",
        min_overlap=0.18,
        name="flywheel stays nested behind the housing cover silhouette",
    )

    rest_position = ctx.part_world_position(seat_post)
    with ctx.pose({seat_slide: 0.12}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.010,
            name="seat post stays centered in sleeve when raised",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.16,
            name="seat post retains insertion at max extension",
        )
        raised_position = ctx.part_world_position(seat_post)

    ctx.check(
        "seat raises upward",
        rest_position is not None and raised_position is not None and raised_position[2] > rest_position[2] + 0.08,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
