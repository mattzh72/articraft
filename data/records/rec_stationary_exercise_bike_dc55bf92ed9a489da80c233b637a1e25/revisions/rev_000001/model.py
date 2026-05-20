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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
    y_center: float = 0.0,
):
    return [
        (x, y + y_center, z + z_center)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_bike")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    polymer_shell = model.material("polymer_shell", rgba=(0.11, 0.12, 0.13, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.06, 1.0))
    saddle_skin = model.material("saddle_skin", rgba=(0.12, 0.12, 0.13, 1.0))
    alloy = model.material("alloy", rgba=(0.70, 0.72, 0.75, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.25, 0.60, 1.28)),
        mass=48.0,
        origin=Origin(xyz=(0.06, 0.0, 0.62)),
    )

    rail_profile = rounded_rect_profile(0.072, 0.046, 0.016, corner_segments=8)
    slim_profile = rounded_rect_profile(0.054, 0.036, 0.012, corner_segments=8)
    post_profile = rounded_rect_profile(0.040, 0.030, 0.010, corner_segments=8)
    clamp_profile = rounded_rect_profile(0.078, 0.056, 0.014, corner_segments=8)

    frame.visual(
        Box((0.14, 0.54, 0.048)),
        origin=Origin(xyz=(-0.33, 0.0, 0.024)),
        material=frame_paint,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.12, 0.50, 0.046)),
        origin=Origin(xyz=(0.48, 0.0, 0.023)),
        material=frame_paint,
        name="front_stabilizer",
    )

    for x in (-0.33, 0.48):
        for y in (-0.20, 0.20):
            frame.visual(
                Box((0.036, 0.070, 0.012)),
                origin=Origin(xyz=(x, y, 0.006)),
                material=elastomer,
            )

    rear_spine = sweep_profile_along_spline(
        [(-0.33, 0.0, 0.048), (-0.27, 0.0, 0.18), (-0.14, 0.0, 0.29), (-0.060, 0.0, 0.338)],
        profile=rail_profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    top_beam = sweep_profile_along_spline(
        [(0.060, 0.0, 0.352), (0.12, 0.0, 0.40), (0.27, 0.0, 0.52), (0.39, 0.0, 0.64)],
        profile=slim_profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    front_mast = sweep_profile_along_spline(
        [(0.48, 0.0, 0.046), (0.45, 0.0, 0.24), (0.41, 0.0, 0.55), (0.40, 0.0, 0.88)],
        profile=rail_profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    seat_mast = sweep_profile_along_spline(
        [(-0.01, 0.0, 0.36), (-0.07, 0.0, 0.54), (-0.14, 0.0, 0.78)],
        profile=slim_profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    seat_post = sweep_profile_along_spline(
        [(-0.14, 0.0, 0.76), (-0.18, 0.0, 0.93)],
        profile=post_profile,
        samples_per_segment=10,
        cap_profile=True,
    )
    bar_post = sweep_profile_along_spline(
        [(0.40, 0.0, 0.86), (0.41, 0.0, 1.08)],
        profile=post_profile,
        samples_per_segment=10,
        cap_profile=True,
    )
    seat_clamp = sweep_profile_along_spline(
        [(-0.132, 0.0, 0.735), (-0.146, 0.0, 0.795)],
        profile=clamp_profile,
        samples_per_segment=6,
        cap_profile=True,
    )
    bar_clamp = sweep_profile_along_spline(
        [(0.398, 0.0, 0.81), (0.405, 0.0, 0.89)],
        profile=clamp_profile,
        samples_per_segment=6,
        cap_profile=True,
    )

    frame.visual(_mesh("bike_rear_spine", rear_spine), material=frame_paint, name="rear_spine")
    frame.visual(_mesh("bike_top_beam", top_beam), material=frame_paint, name="top_beam")
    frame.visual(_mesh("bike_front_mast", front_mast), material=frame_paint)
    frame.visual(_mesh("bike_seat_mast", seat_mast), material=frame_paint)
    frame.visual(
        _mesh("bike_seat_post", seat_post),
        material=alloy,
        name="seat_post",
    )
    frame.visual(_mesh("bike_seat_clamp", seat_clamp), material=frame_paint, name="seat_clamp")
    frame.visual(_mesh("bike_bar_clamp", bar_clamp), material=frame_paint, name="bar_clamp")
    frame.visual(_mesh("bike_bar_post", bar_post), material=alloy)

    frame.visual(
        Cylinder(radius=0.055, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.33), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="bottom_bracket_shell",
    )
    frame.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.049, 0.33), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="left_bearing_cup",
    )
    frame.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, -0.049, 0.33), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="right_bearing_cup",
    )

    housing = section_loft(
        [
            _yz_section(0.05, 0.072, 0.16, 0.018, z_center=0.35),
            _yz_section(0.18, 0.132, 0.52, 0.034, z_center=0.42),
            _yz_section(0.31, 0.114, 0.48, 0.028, z_center=0.44),
            _yz_section(0.42, 0.084, 0.22, 0.020, z_center=0.33),
        ]
    )
    frame.visual(
        _mesh("bike_flywheel_housing", housing),
        material=polymer_shell,
        name="flywheel_housing",
    )
    drive_side_cap = section_loft(
        [
            _yz_section(0.22, 0.022, 0.090, 0.009, z_center=0.43, y_center=-0.067),
            _yz_section(0.27, 0.088, 0.190, 0.022, z_center=0.43, y_center=-0.067),
            _yz_section(0.33, 0.104, 0.172, 0.020, z_center=0.43, y_center=-0.067),
            _yz_section(0.38, 0.028, 0.074, 0.009, z_center=0.43, y_center=-0.067),
        ]
    )
    frame.visual(
        _mesh("bike_drive_side_cap", drive_side_cap),
        material=polymer_dark,
        name="drive_side_cap",
    )

    frame.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(-0.145, 0.032, 0.776), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="seat_adjuster_pin",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(-0.145, 0.066, 0.776), rpy=(pi / 2.0, 0.0, 0.0)),
        material=polymer_dark,
        name="seat_adjuster",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.405, 0.032, 0.855), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="bar_adjuster_pin",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.405, 0.066, 0.855), rpy=(pi / 2.0, 0.0, 0.0)),
        material=polymer_dark,
        name="bar_adjuster",
    )

    saddle = section_loft(
        [
            _yz_section(-0.12, 0.180, 0.068, 0.022, z_center=0.014),
            _yz_section(-0.02, 0.156, 0.060, 0.019, z_center=0.010),
            _yz_section(0.08, 0.102, 0.046, 0.014, z_center=0.004),
            _yz_section(0.15, 0.050, 0.030, 0.010, z_center=-0.002),
        ]
    )
    frame.visual(
        _mesh("bike_saddle", saddle),
        origin=Origin(xyz=(-0.22, 0.0, 0.952), rpy=(0.0, -0.10, 0.0)),
        material=saddle_skin,
        name="saddle",
    )
    frame.visual(
        Box((0.110, 0.060, 0.026)),
        origin=Origin(xyz=(-0.205, 0.0, 0.927), rpy=(0.0, -0.10, 0.0)),
        material=polymer_dark,
        name="saddle_yoke",
    )

    frame.visual(
        Box((0.060, 0.052, 0.052)),
        origin=Origin(xyz=(0.408, 0.0, 1.103)),
        material=frame_paint,
        name="bar_stem",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.082),
        origin=Origin(xyz=(0.409, 0.0, 1.149)),
        material=frame_paint,
        name="bar_riser",
    )
    handlebar_loop = tube_from_spline_points(
        [
            (0.29, -0.23, 1.03),
            (0.35, -0.20, 1.08),
            (0.42, -0.14, 1.16),
            (0.41, 0.0, 1.18),
            (0.42, 0.14, 1.16),
            (0.35, 0.20, 1.08),
            (0.29, 0.23, 1.03),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(
        _mesh("bike_handlebar_loop", handlebar_loop),
        material=frame_paint,
        name="handlebar_loop",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=(0.31, -0.215, 1.035), rpy=(pi / 2.0, 0.0, 0.0)),
        material=elastomer,
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=(0.31, 0.215, 1.035), rpy=(pi / 2.0, 0.0, 0.0)),
        material=elastomer,
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.310, -0.215, 1.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.310, 0.215, 1.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
    )

    crank_set = model.part("crank_set")
    crank_set.inertial = Inertial.from_geometry(
        Box((0.40, 0.30, 0.36)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crank_set.visual(
        Cylinder(radius=0.017, length=0.128),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.073, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="left_crank_boss",
    )
    crank_set.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, -0.073, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="right_crank_boss",
    )
    crank_set.visual(
        Box((0.176, 0.024, 0.018)),
        origin=Origin(xyz=(0.088, 0.074, 0.0)),
        material=alloy,
        name="left_crank_arm",
    )
    crank_set.visual(
        Box((0.176, 0.024, 0.018)),
        origin=Origin(xyz=(-0.088, -0.074, 0.0)),
        material=alloy,
        name="right_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.010, length=0.084),
        origin=Origin(xyz=(0.176, 0.124, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.010, length=0.084),
        origin=Origin(xyz=(-0.176, -0.124, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Box((0.108, 0.084, 0.024)),
        origin=Origin(xyz=(0.176, 0.160, 0.0)),
        material=polymer_dark,
        name="left_pedal_body",
    )
    crank_set.visual(
        Box((0.108, 0.084, 0.024)),
        origin=Origin(xyz=(-0.176, -0.160, 0.0)),
        material=polymer_dark,
        name="right_pedal_body",
    )
    crank_set.visual(
        Box((0.066, 0.018, 0.012)),
        origin=Origin(xyz=(0.176, 0.188, 0.014)),
        material=elastomer,
        name="left_pedal_tread",
    )
    crank_set.visual(
        Box((0.066, 0.018, 0.012)),
        origin=Origin(xyz=(-0.176, -0.188, 0.014)),
        material=elastomer,
        name="right_pedal_tread",
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_set,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank_set = object_model.get_part("crank_set")
    crank_spin = object_model.get_articulation("crank_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        crank_set,
        frame,
        elem_a="spindle",
        elem_b="bottom_bracket_shell",
        reason="The rotating crank spindle passes through the bottom-bracket shell.",
    )
    ctx.allow_overlap(
        crank_set,
        frame,
        elem_a="spindle",
        elem_b="left_bearing_cup",
        reason="The spindle runs through the left bearing cup support.",
    )
    ctx.allow_overlap(
        crank_set,
        frame,
        elem_a="spindle",
        elem_b="right_bearing_cup",
        reason="The spindle runs through the right bearing cup support.",
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

    for visual_name in (
        "flywheel_housing",
        "seat_post",
        "saddle",
        "seat_clamp",
        "bar_clamp",
        "handlebar_loop",
        "bottom_bracket_shell",
    ):
        ctx.check(
            f"{visual_name}_present",
            frame.get_visual(visual_name) is not None,
            details=f"Expected frame visual '{visual_name}' to exist.",
        )

    for visual_name in (
        "spindle",
        "left_crank_boss",
        "right_crank_boss",
        "left_crank_arm",
        "right_crank_arm",
        "left_pedal_body",
        "right_pedal_body",
    ):
        ctx.check(
            f"{visual_name}_present",
            crank_set.get_visual(visual_name) is not None,
            details=f"Expected crank visual '{visual_name}' to exist.",
        )

    ctx.check(
        "crank_axis_is_lateral",
        abs(crank_spin.axis[0]) < 1e-6 and abs(crank_spin.axis[2]) < 1e-6 and abs(abs(crank_spin.axis[1]) - 1.0) < 1e-6,
        details=f"Crank axis should run laterally along world Y, got {crank_spin.axis}.",
    )
    ctx.expect_contact(
        crank_set,
        frame,
        elem_a="spindle",
        elem_b="bottom_bracket_shell",
        name="crank_supported_by_bottom_bracket",
    )
    ctx.fail_if_articulation_overlaps(max_pose_samples=18, name="crank_clearance_over_cycle")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
