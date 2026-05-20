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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _add_mesh_visual(part, geometry, *, name: str, material: str) -> None:
    part.visual(mesh_from_geometry(geometry, name), material=material, name=name)


def _constant_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
    segments: int = 48,
):
    half = length / 2.0
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_x(pi / 2.0).translate(*center)
    return geom


def _housing_shell_mesh(*, center: tuple[float, float, float]):
    geom = LatheGeometry.from_shell_profiles(
        [
            (0.235, -0.095),
            (0.270, -0.070),
            (0.285, 0.000),
            (0.270, 0.070),
            (0.235, 0.095),
        ],
        [
            (0.200, -0.095),
            (0.232, -0.070),
            (0.245, 0.000),
            (0.232, 0.070),
            (0.200, 0.095),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    geom.rotate_x(pi / 2.0).translate(*center)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_finish = model.material("frame_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    shroud_finish = model.material("shroud_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.56, 0.58, 0.60, 1.0))
    bushing_finish = model.material("bushing_finish", rgba=(0.77, 0.62, 0.17, 1.0))
    saddle_finish = model.material("saddle_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.06, 0.06, 0.06, 1.0))
    pedal_finish = model.material("pedal_finish", rgba=(0.14, 0.14, 0.15, 1.0))

    frame = model.part("frame")
    crank = model.part("crank_assembly")
    saddle_post = model.part("saddle_post")
    handlebar_mast = model.part("handlebar_mast")

    frame.visual(
        Box((0.44, 0.09, 0.04)),
        origin=Origin(xyz=(-0.42, 0.0, 0.02)),
        material=frame_finish,
        name="rear_foot",
    )
    frame.visual(
        Box((0.38, 0.09, 0.04)),
        origin=Origin(xyz=(0.48, 0.0, 0.02)),
        material=frame_finish,
        name="front_foot",
    )

    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (-0.42, 0.0, 0.06),
                (-0.30, 0.0, 0.10),
                (-0.18, 0.0, 0.17),
                (-0.08, 0.0, 0.24),
            ],
            radius=0.034,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="main_beam",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (-0.30, 0.0, 0.10),
                (-0.25, 0.0, 0.26),
                (-0.22, 0.0, 0.46),
                (-0.24, 0.0, 0.60),
            ],
            radius=0.028,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="seat_stay",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (-0.22, -0.060, 0.76),
                (0.06, -0.060, 0.74),
                (0.48, -0.060, 0.84),
            ],
            radius=0.018,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="top_rail_left",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (-0.22, 0.060, 0.76),
                (0.06, 0.060, 0.74),
                (0.48, 0.060, 0.84),
            ],
            radius=0.018,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="top_rail_right",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (0.08, -0.055, 0.38),
                (0.22, -0.060, 0.62),
                (0.48, -0.060, 0.84),
            ],
            radius=0.024,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="front_stay_left",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (0.08, 0.055, 0.38),
                (0.22, 0.060, 0.62),
                (0.48, 0.060, 0.84),
            ],
            radius=0.024,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="front_stay_right",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        tube_from_spline_points(
            [
                (0.08, 0.0, 0.36),
                (0.22, 0.0, 0.26),
                (0.48, 0.0, 0.06),
            ],
            radius=0.028,
            samples_per_segment=18,
            radial_segments=20,
        ),
        name="base_brace",
        material="frame_finish",
    )
    _add_mesh_visual(
        frame,
        _housing_shell_mesh(center=(0.33, 0.0, 0.50)),
        name="flywheel_housing",
        material="shroud_finish",
    )
    frame.visual(
        Box((0.14, 0.148, 0.072)),
        origin=Origin(xyz=(-0.04, 0.0, 0.267)),
        material=hardware_finish,
        name="bottom_bracket_shell",
    )
    frame.visual(
        Box((0.038, 0.100, 0.300)),
        origin=Origin(xyz=(-0.220, 0.0, 0.74)),
        material=hardware_finish,
        name="seat_receiver",
    )
    frame.visual(
        Box((0.044, 0.023, 0.300)),
        origin=Origin(xyz=(-0.137, -0.0345, 0.74)),
        material=hardware_finish,
        name="seat_left_jaw",
    )
    frame.visual(
        Box((0.044, 0.023, 0.300)),
        origin=Origin(xyz=(-0.137, 0.0345, 0.74)),
        material=hardware_finish,
        name="seat_right_jaw",
    )
    frame.visual(
        Box((0.050, 0.028, 0.040)),
        origin=Origin(xyz=(-0.137, 0.058, 0.84)),
        material=hardware_finish,
        name="seat_clamp_ear",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.14),
        origin=Origin(xyz=(-0.084, 0.058, 0.84), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_finish,
        name="seat_clamp_stud",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(-0.002, 0.058, 0.84), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_finish,
        name="seat_clamp_knob",
    )
    frame.visual(
        Box((0.038, 0.100, 0.300)),
        origin=Origin(xyz=(0.478, 0.0, 0.92)),
        material=hardware_finish,
        name="bar_receiver",
    )
    frame.visual(
        Box((0.044, 0.024, 0.300)),
        origin=Origin(xyz=(0.519, -0.037, 0.92)),
        material=hardware_finish,
        name="bar_left_jaw",
    )
    frame.visual(
        Box((0.044, 0.024, 0.300)),
        origin=Origin(xyz=(0.519, 0.037, 0.92)),
        material=hardware_finish,
        name="bar_right_jaw",
    )
    frame.visual(
        Box((0.050, 0.028, 0.040)),
        origin=Origin(xyz=(0.541, 0.058, 1.02)),
        material=hardware_finish,
        name="bar_clamp_ear",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.14),
        origin=Origin(xyz=(0.594, 0.058, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_finish,
        name="bar_clamp_stud",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.676, 0.058, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_finish,
        name="bar_clamp_knob",
    )
    frame.visual(
        Box((0.36, 0.008, 0.44)),
        origin=Origin(xyz=(0.33, -0.099, 0.50)),
        material=hardware_finish,
        name="service_cover",
    )
    for index, (bx, bz) in enumerate(((0.24, 0.36), (0.42, 0.36), (0.24, 0.64), (0.42, 0.64)), start=1):
        frame.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(xyz=(bx, -0.109, bz), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware_finish,
            name=f"service_bolt_{index}",
        )

    crank.visual(
        Cylinder(radius=0.022, length=0.30),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="axle",
    )
    crank.visual(
        Cylinder(radius=0.037, length=0.014),
        origin=Origin(xyz=(0.0, -0.081, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bushing_finish,
        name="right_bearing_sleeve",
    )
    crank.visual(
        Cylinder(radius=0.037, length=0.014),
        origin=Origin(xyz=(0.0, 0.081, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bushing_finish,
        name="left_bearing_sleeve",
    )
    crank.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="hub",
    )
    crank.visual(
        Box((0.012, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, -0.105, 0.0)),
        material=hardware_finish,
        name="right_spider_web",
    )
    crank.visual(
        Box((0.012, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=hardware_finish,
        name="left_spider_web",
    )
    crank.visual(
        Box((0.030, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, -0.150, -0.005)),
        material=hardware_finish,
        name="right_spider_block",
    )
    crank.visual(
        Box((0.030, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, 0.150, -0.005)),
        material=hardware_finish,
        name="left_spider_block",
    )
    crank.visual(
        Cylinder(radius=0.11, length=0.012),
        origin=Origin(xyz=(0.0, -0.164, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shroud_finish,
        name="drive_disc",
    )
    crank.visual(
        Box((0.04, 0.02, 0.19)),
        origin=Origin(xyz=(0.0, -0.1725, -0.105)),
        material=hardware_finish,
        name="right_crank_arm",
    )
    crank.visual(
        Box((0.04, 0.02, 0.19)),
        origin=Origin(xyz=(0.0, 0.1725, -0.105)),
        material=hardware_finish,
        name="left_crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.008, length=0.08),
        origin=Origin(xyz=(0.0, -0.2025, -0.200), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="right_pedal_spindle",
    )
    crank.visual(
        Cylinder(radius=0.008, length=0.08),
        origin=Origin(xyz=(0.0, 0.2025, -0.200), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="left_pedal_spindle",
    )
    crank.visual(
        Box((0.11, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, -0.2425, -0.21)),
        material=pedal_finish,
        name="right_pedal",
    )
    crank.visual(
        Box((0.11, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.2425, -0.21)),
        material=pedal_finish,
        name="left_pedal",
    )

    saddle_post.visual(
        Box((0.042, 0.046, 0.240)),
        origin=Origin(),
        material=bushing_finish,
        name="seat_guide_sleeve",
    )
    saddle_post.visual(
        Cylinder(radius=0.022, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=hardware_finish,
        name="seat_post_tube",
    )
    saddle_post.visual(
        Box((0.08, 0.05, 0.10)),
        origin=Origin(xyz=(-0.01, 0.0, 0.57)),
        material=hardware_finish,
        name="seat_head",
    )
    saddle_post.visual(
        Box((0.20, 0.15, 0.06)),
        origin=Origin(xyz=(-0.04, 0.0, 0.64)),
        material=saddle_finish,
        name="saddle_rear",
    )
    saddle_post.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.10, 0.0, 0.635), rpy=(0.0, pi / 2.0, 0.0)),
        material=saddle_finish,
        name="saddle_nose",
    )

    handlebar_mast.visual(
        Box((0.046, 0.050, 0.220)),
        origin=Origin(),
        material=bushing_finish,
        name="mast_guide_sleeve",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.022, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=hardware_finish,
        name="mast_tube",
    )
    handlebar_mast.visual(
        Box((0.14, 0.04, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.30)),
        material=hardware_finish,
        name="stem_column",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="bar_clamp",
    )
    _add_mesh_visual(
        handlebar_mast,
        tube_from_spline_points(
            [
                (0.06, -0.23, 0.26),
                (0.02, -0.16, 0.32),
                (0.0, 0.0, 0.36),
                (0.02, 0.16, 0.32),
                (0.06, 0.23, 0.26),
            ],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
        ),
        name="handlebar_loop",
        material="hardware_finish",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(0.055, -0.21, 0.27), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_finish,
        name="left_grip",
    )
    handlebar_mast.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(0.055, 0.21, 0.27), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_finish,
        name="right_grip",
    )

    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(-0.02, 0.0, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_post,
        origin=Origin(xyz=(-0.18, 0.0, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.20,
            lower=0.0,
            upper=0.12,
        ),
    )
    model.articulation(
        "frame_to_handlebar_mast",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handlebar_mast,
        origin=Origin(xyz=(0.52, 0.0, 0.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.20,
            lower=0.0,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank = object_model.get_part("crank_assembly")
    saddle_post = object_model.get_part("saddle_post")
    handlebar_mast = object_model.get_part("handlebar_mast")

    crank_joint = object_model.get_articulation("frame_to_crank")
    seat_joint = object_model.get_articulation("frame_to_saddle_post")
    bar_joint = object_model.get_articulation("frame_to_handlebar_mast")

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

    ctx.expect_contact(
        crank,
        frame,
        elem_a="right_bearing_sleeve",
        elem_b="bottom_bracket_shell",
        name="drive-side crank bearing is seated in the shell",
    )
    ctx.expect_contact(
        crank,
        frame,
        elem_a="left_bearing_sleeve",
        elem_b="bottom_bracket_shell",
        name="non-drive crank bearing is seated in the shell",
    )
    ctx.expect_contact(
        saddle_post,
        frame,
        elem_a="seat_guide_sleeve",
        elem_b="seat_receiver",
        name="saddle post guide engages the seat receiver",
    )
    ctx.expect_contact(
        handlebar_mast,
        frame,
        elem_a="mast_guide_sleeve",
        elem_b="bar_receiver",
        name="handlebar mast guide engages the bar receiver",
    )

    with ctx.pose({seat_joint: 0.10}):
        ctx.expect_contact(
            saddle_post,
            frame,
            elem_a="seat_guide_sleeve",
            elem_b="seat_receiver",
            name="saddle post remains guided when raised",
        )
    with ctx.pose({bar_joint: 0.08}):
        ctx.expect_contact(
            handlebar_mast,
            frame,
            elem_a="mast_guide_sleeve",
            elem_b="bar_receiver",
            name="handlebar mast remains guided when raised",
        )

    with ctx.pose({seat_joint: 0.0}):
        seat_low = ctx.part_world_position(saddle_post)
    with ctx.pose({seat_joint: 0.10}):
        seat_high = ctx.part_world_position(saddle_post)
    if seat_low is not None and seat_high is not None:
        ctx.check(
            "seat adjuster lifts upward",
            seat_high[2] > seat_low[2] + 0.09,
            f"seat positions were {seat_low} and {seat_high}",
        )

    with ctx.pose({bar_joint: 0.0}):
        bar_low = ctx.part_world_position(handlebar_mast)
    with ctx.pose({bar_joint: 0.08}):
        bar_high = ctx.part_world_position(handlebar_mast)
    if bar_low is not None and bar_high is not None:
        ctx.check(
            "bar adjuster lifts upward",
            bar_high[2] > bar_low[2] + 0.07,
            f"bar positions were {bar_low} and {bar_high}",
        )

    ctx.check(
        "crank spins on a lateral continuous axis",
        crank_joint.axis == (0.0, 1.0, 0.0)
        and crank_joint.motion_limits is not None
        and crank_joint.motion_limits.lower is None
        and crank_joint.motion_limits.upper is None,
        f"axis={crank_joint.axis}, limits={crank_joint.motion_limits}",
    )
    ctx.check(
        "adjusters are vertical sliders",
        seat_joint.axis == (0.0, 0.0, 1.0) and bar_joint.axis == (0.0, 0.0, 1.0),
        f"seat axis={seat_joint.axis}, bar axis={bar_joint.axis}",
    )

    with ctx.pose({crank_joint: pi / 2.0, seat_joint: 0.10, bar_joint: 0.08}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in service pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
