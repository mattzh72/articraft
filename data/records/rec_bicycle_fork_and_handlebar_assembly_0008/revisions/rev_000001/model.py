from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
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

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent
HEAD_ANGLE = math.radians(19.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xy_section(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _axis_point(distance: float) -> tuple[float, float, float]:
    return (0.0, -math.sin(HEAD_ANGLE) * distance, math.cos(HEAD_ANGLE) * distance)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _aabb_center(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fat_bike_front_end", assets=ASSETS)

    alloy = model.material("alloy", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.28, 0.30, 0.33, 1.0))
    bar_black = model.material("bar_black", rgba=(0.08, 0.08, 0.09, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.16, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.44, 0.58)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.16, 0.03)),
    )

    frame.visual(
        Box((0.010, 0.084, 0.112)),
        origin=Origin(xyz=(-0.039, 0.0, 0.056), rpy=(HEAD_ANGLE, 0.0, 0.0)),
        material=frame_paint,
        name="head_tube_shell",
    )
    frame.visual(
        Box((0.010, 0.084, 0.112)),
        origin=Origin(xyz=(0.039, 0.0, 0.056), rpy=(HEAD_ANGLE, 0.0, 0.0)),
        material=frame_paint,
        name="head_tube_lower_right",
    )
    frame.visual(
        Box((0.010, 0.070, 0.094)),
        origin=Origin(xyz=(-0.033, 0.0, 0.145), rpy=(HEAD_ANGLE, 0.0, 0.0)),
        material=frame_paint,
        name="head_tube_upper_left",
    )
    frame.visual(
        Box((0.010, 0.070, 0.094)),
        origin=Origin(xyz=(0.033, 0.0, 0.145), rpy=(HEAD_ANGLE, 0.0, 0.0)),
        material=frame_paint,
        name="head_tube_upper_right",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=_axis_point(0.009), rpy=(HEAD_ANGLE, 0.0, 0.0)),
        material=dark_alloy,
        name="lower_headset_band",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=_axis_point(0.182), rpy=(HEAD_ANGLE, 0.0, 0.0)),
        material=dark_alloy,
        name="upper_headset_band",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.30),
        origin=Origin(xyz=(0.0, -0.250, 0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="top_tube_stub",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.42),
        origin=Origin(xyz=(0.0, -0.118, -0.030), rpy=(0.74, 0.0, 0.0)),
        material=frame_paint,
        name="down_tube_stub",
    )
    frame.visual(
        Box((0.024, 0.180, 0.050)),
        origin=Origin(xyz=(-0.050, -0.085, 0.150)),
        material=frame_paint,
        name="upper_left_bridge",
    )
    frame.visual(
        Box((0.024, 0.180, 0.050)),
        origin=Origin(xyz=(0.050, -0.085, 0.150)),
        material=frame_paint,
        name="upper_right_bridge",
    )
    frame.visual(
        Box((0.030, 0.100, 0.040)),
        origin=Origin(xyz=(-0.047, -0.085, 0.016)),
        material=frame_paint,
        name="lower_left_bridge",
    )
    frame.visual(
        Box((0.030, 0.100, 0.040)),
        origin=Origin(xyz=(0.047, -0.085, 0.016)),
        material=frame_paint,
        name="lower_right_bridge",
    )
    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.36, 0.28, 1.04)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.03, -0.24)),
    )

    fork.visual(
        Cylinder(radius=0.014, length=0.202),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_alloy,
        name="steerer_tube",
    )
    fork.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_alloy,
        name="lower_crown_race",
    )
    fork.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=dark_alloy,
        name="upper_dust_cap",
    )
    fork.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.197)),
        material=dark_alloy,
        name="spacer_stack",
    )
    fork.visual(
        Box((0.286, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.010, -0.032)),
        material=alloy,
        name="fork_crown",
    )
    fork.visual(
        Box((0.062, 0.044, 0.760)),
        origin=Origin(xyz=(-0.152, 0.050, -0.400), rpy=(0.0, 0.0, 0.0)),
        material=alloy,
        name="left_blade",
    )
    fork.visual(
        Box((0.062, 0.044, 0.760)),
        origin=Origin(xyz=(0.152, 0.050, -0.400), rpy=(0.0, 0.0, 0.0)),
        material=alloy,
        name="right_blade",
    )
    fork.visual(
        Box((0.060, 0.050, 0.100)),
        origin=Origin(xyz=(-0.152, 0.086, -0.782)),
        material=alloy,
        name="left_dropout_socket",
    )
    fork.visual(
        Box((0.060, 0.050, 0.100)),
        origin=Origin(xyz=(0.152, 0.086, -0.782)),
        material=alloy,
        name="right_dropout_socket",
    )
    fork.visual(
        Box((0.024, 0.046, 0.044)),
        origin=Origin(xyz=(-0.152, 0.086, -0.816)),
        material=dark_alloy,
        name="left_dropout",
    )
    fork.visual(
        Box((0.024, 0.046, 0.044)),
        origin=Origin(xyz=(0.152, 0.086, -0.816)),
        material=dark_alloy,
        name="right_dropout",
    )
    fork.visual(
        Cylinder(radius=0.009, length=0.304),
        origin=Origin(xyz=(0.0, 0.086, -0.806), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="thru_axle",
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.86, 0.20, 0.16)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.08, 0.05)),
    )
    handlebar.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_alloy,
        name="stem_riser",
    )
    handlebar.visual(
        Box((0.052, 0.090, 0.036)),
        origin=Origin(xyz=(0.0, 0.048, 0.042)),
        material=dark_alloy,
        name="stem_body",
    )
    handlebar.visual(
        Box((0.062, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, 0.086, 0.048)),
        material=dark_alloy,
        name="faceplate",
    )

    bar_mesh = tube_from_spline_points(
        [
            (-0.395, 0.072, 0.045),
            (-0.285, 0.078, 0.052),
            (-0.155, 0.086, 0.058),
            (-0.040, 0.090, 0.060),
            (0.040, 0.090, 0.060),
            (0.155, 0.086, 0.058),
            (0.285, 0.078, 0.052),
            (0.395, 0.072, 0.045),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handlebar.visual(
        _save_mesh("fat_bike_handlebar.obj", bar_mesh),
        material=bar_black,
        name="handlebar_bar",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(-0.405, 0.070, 0.044), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bar_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(0.405, 0.070, 0.044), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bar_black,
        name="right_grip",
    )

    model.articulation(
        "steerer_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(rpy=(HEAD_ANGLE, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-0.60, upper=0.60),
    )
    model.articulation(
        "bar_mount",
        ArticulationType.FIXED,
        parent=fork,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    frame = object_model.get_part("frame")
    fork = object_model.get_part("fork")
    handlebar = object_model.get_part("handlebar")
    steer = object_model.get_articulation("steerer_turn")

    lower_headset_band = frame.get_visual("lower_headset_band")
    upper_headset_band = frame.get_visual("upper_headset_band")
    head_tube_shell = frame.get_visual("head_tube_shell")
    head_tube_lower_right = frame.get_visual("head_tube_lower_right")
    steerer_tube = fork.get_visual("steerer_tube")
    lower_crown_race = fork.get_visual("lower_crown_race")
    upper_dust_cap = fork.get_visual("upper_dust_cap")
    spacer_stack = fork.get_visual("spacer_stack")
    fork_crown = fork.get_visual("fork_crown")
    left_blade = fork.get_visual("left_blade")
    right_blade = fork.get_visual("right_blade")
    left_dropout = fork.get_visual("left_dropout")
    stem_riser = handlebar.get_visual("stem_riser")
    left_grip = handlebar.get_visual("left_grip")
    right_grip = handlebar.get_visual("right_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        frame,
        fork,
        elem_a=lower_headset_band,
        elem_b=steerer_tube,
        reason="The steerer passes through the lower headset bearing seat; the solid cylinder is a proxy for an annular race.",
    )
    ctx.allow_overlap(
        frame,
        fork,
        elem_a=upper_headset_band,
        elem_b=steerer_tube,
        reason="The steerer passes through the upper headset bearing seat; the solid cylinder is a proxy for an annular race.",
    )
    ctx.allow_overlap(
        frame,
        fork,
        elem_a=upper_headset_band,
        elem_b=upper_dust_cap,
        reason="The upper dust cap nests into the upper headset seat as a simplified telescoping headset stack.",
    )
    ctx.allow_overlap(
        frame,
        fork,
        elem_a=head_tube_shell,
        elem_b=lower_crown_race,
        reason="The lower crown race tucks into the lower lip of the tapered head tube in this simplified headset representation.",
    )
    ctx.allow_overlap(
        frame,
        fork,
        elem_a=head_tube_lower_right,
        elem_b=lower_crown_race,
        reason="The box-wall approximation of the lower head tube side slightly intersects the seated crown race at full steering sweep.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        frame,
        fork,
        elem_a=lower_headset_band,
        elem_b=lower_crown_race,
        name="lower_headset_contact",
    )
    ctx.expect_contact(
        frame,
        fork,
        elem_a=upper_headset_band,
        elem_b=upper_dust_cap,
        name="upper_headset_contact",
    )
    ctx.expect_contact(
        fork,
        handlebar,
        elem_a=spacer_stack,
        elem_b=stem_riser,
        name="stem_mount_contact",
    )

    lower_band_aabb = ctx.part_element_world_aabb(frame, elem=lower_headset_band)
    upper_band_aabb = ctx.part_element_world_aabb(frame, elem=upper_headset_band)
    left_blade_aabb = ctx.part_element_world_aabb(fork, elem=left_blade)
    right_blade_aabb = ctx.part_element_world_aabb(fork, elem=right_blade)
    crown_aabb = ctx.part_element_world_aabb(fork, elem=fork_crown)
    left_dropout_aabb = ctx.part_element_world_aabb(fork, elem=left_dropout)
    left_grip_aabb = ctx.part_element_world_aabb(handlebar, elem=left_grip)
    right_grip_aabb = ctx.part_element_world_aabb(handlebar, elem=right_grip)

    assert lower_band_aabb is not None
    assert upper_band_aabb is not None
    assert left_blade_aabb is not None
    assert right_blade_aabb is not None
    assert crown_aabb is not None
    assert left_dropout_aabb is not None
    assert left_grip_aabb is not None
    assert right_grip_aabb is not None

    lower_head_width = lower_band_aabb[1][0] - lower_band_aabb[0][0]
    upper_head_width = upper_band_aabb[1][0] - upper_band_aabb[0][0]
    blade_inside_gap = right_blade_aabb[0][0] - left_blade_aabb[1][0]
    blade_drop = crown_aabb[0][2] - left_dropout_aabb[1][2]
    grip_span = right_grip_aabb[1][0] - left_grip_aabb[0][0]

    ctx.check(
        "tapered_head_tube",
        lower_head_width > upper_head_width + 0.010,
        details=f"lower={lower_head_width:.3f} upper={upper_head_width:.3f}",
    )
    ctx.check(
        "fat_fork_clear_width",
        blade_inside_gap > 0.18,
        details=f"inside gap={blade_inside_gap:.3f}",
    )
    ctx.check(
        "fork_blade_drop",
        blade_drop > 0.62,
        details=f"blade drop={blade_drop:.3f}",
    )
    ctx.check(
        "wide_flat_bar_span",
        grip_span > 0.82,
        details=f"grip span={grip_span:.3f}",
    )

    rest_left = _aabb_center(left_grip_aabb)
    rest_right = _aabb_center(right_grip_aabb)

    limits = steer.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    with ctx.pose({steer: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="steerer_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="steerer_upper_no_floating")
        ctx.expect_contact(frame, fork, elem_a=upper_headset_band, elem_b=upper_dust_cap)
        ctx.expect_contact(fork, handlebar, elem_a=spacer_stack, elem_b=stem_riser)
        left_upper_aabb = ctx.part_element_world_aabb(handlebar, elem=left_grip)
        right_upper_aabb = ctx.part_element_world_aabb(handlebar, elem=right_grip)
        assert left_upper_aabb is not None
        assert right_upper_aabb is not None
        left_upper = _aabb_center(left_upper_aabb)
        right_upper = _aabb_center(right_upper_aabb)
        ctx.check(
            "steer_upper_sweeps_bar",
            left_upper[1] < rest_left[1] - 0.05 and right_upper[1] > rest_right[1] + 0.05,
            details=(
                f"left_y={left_upper[1]:.3f} rest_left_y={rest_left[1]:.3f} "
                f"right_y={right_upper[1]:.3f} rest_right_y={rest_right[1]:.3f}"
            ),
        )

    with ctx.pose({steer: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="steerer_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="steerer_lower_no_floating")
        ctx.expect_contact(frame, fork, elem_a=lower_headset_band, elem_b=lower_crown_race)
        ctx.expect_contact(fork, handlebar, elem_a=spacer_stack, elem_b=stem_riser)
        left_lower_aabb = ctx.part_element_world_aabb(handlebar, elem=left_grip)
        right_lower_aabb = ctx.part_element_world_aabb(handlebar, elem=right_grip)
        assert left_lower_aabb is not None
        assert right_lower_aabb is not None
        left_lower = _aabb_center(left_lower_aabb)
        right_lower = _aabb_center(right_lower_aabb)
        ctx.check(
            "steer_lower_sweeps_bar",
            left_lower[1] > rest_left[1] + 0.05 and right_lower[1] < rest_right[1] - 0.05,
            details=(
                f"left_y={left_lower[1]:.3f} rest_left_y={rest_left[1]:.3f} "
                f"right_y={right_lower[1]:.3f} rest_right_y={rest_right[1]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
