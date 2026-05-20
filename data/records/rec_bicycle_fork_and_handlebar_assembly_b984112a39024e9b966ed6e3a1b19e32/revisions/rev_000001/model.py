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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_rounded_section(
    center_x: float,
    center_y: float,
    z: float,
    width_x: float,
    depth_y: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, center_y + py, z)
        for px, py in rounded_rect_profile(width_x, depth_y, corner_radius, corner_segments=6)
    ]


def _circle_profile(radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _build_fork_blade_mesh(side_sign: float):
    return section_loft(
        [
            _xy_rounded_section(
                0.047 * side_sign,
                0.010,
                -0.018,
                0.026,
                0.032,
                0.006,
            ),
            _xy_rounded_section(
                0.050 * side_sign,
                0.022,
                -0.220,
                0.020,
                0.024,
                0.0045,
            ),
            _xy_rounded_section(
                0.052 * side_sign,
                0.036,
                -0.430,
                0.014,
                0.018,
                0.003,
            ),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_upright_bike_fork_and_handlebar")

    painted_steel = model.material("painted_steel", rgba=(0.14, 0.15, 0.17, 1.0))
    alloy = model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.0145, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=painted_steel,
        name="steerer_tube",
    )
    fork.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.289)),
        material=alloy,
        name="headset_top",
    )
    fork.visual(
        Box((0.084, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=painted_steel,
        name="crown",
    )
    fork.visual(
        _mesh("fork_left_blade", _build_fork_blade_mesh(1.0)),
        material=painted_steel,
        name="left_blade",
    )
    fork.visual(
        _mesh("fork_right_blade", _build_fork_blade_mesh(-1.0)),
        material=painted_steel,
        name="right_blade",
    )
    fork.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(0.052, 0.038, -0.430)),
        material=matte_black,
        name="left_dropout",
    )
    fork.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(-0.052, 0.038, -0.430)),
        material=matte_black,
        name="right_dropout",
    )
    fork.visual(
        Box((0.060, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, 0.268)),
        material=alloy,
        name="lower_pivot_bridge",
    )
    fork.visual(
        Box((0.060, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, 0.308)),
        material=alloy,
        name="upper_pivot_bridge",
    )
    fork.visual(
        Box((0.010, 0.024, 0.034)),
        origin=Origin(xyz=(-0.033, 0.040, 0.288)),
        material=alloy,
        name="left_pivot_ear",
    )
    fork.visual(
        Box((0.010, 0.024, 0.034)),
        origin=Origin(xyz=(0.033, 0.040, 0.288)),
        material=alloy,
        name="right_pivot_ear",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.76)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.010, -0.055)),
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.014, length=0.056),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="pivot_barrel",
    )
    stem.visual(
        Box((0.030, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.018, 0.020)),
        material=alloy,
        name="pivot_yoke",
    )
    stem.visual(
        _mesh(
            "stem_spine",
            tube_from_spline_points(
                [
                    (0.0, 0.024, 0.016),
                    (0.0, 0.050, 0.088),
                    (0.0, 0.078, 0.164),
                    (0.0, 0.094, 0.214),
                ],
                radius=0.015,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=alloy,
        name="stem_spine",
    )
    bar_clamp_shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.038, 0.042, 0.008, corner_segments=8),
        [_circle_profile(0.0132, segments=36)],
        0.070,
        cap=True,
        center=True,
        closed=True,
    )
    stem.visual(
        _mesh("stem_bar_clamp_shell", bar_clamp_shell.rotate_y(math.pi / 2.0)),
        origin=Origin(xyz=(0.0, 0.110, 0.240)),
        material=alloy,
        name="bar_clamp_shell",
    )
    stem.visual(
        Box((0.058, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.110, 0.2557)),
        material=alloy,
        name="upper_clamp_land",
    )
    stem.visual(
        Box((0.058, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.110, 0.2243)),
        material=alloy,
        name="lower_clamp_land",
    )
    stem.visual(
        Box((0.024, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, 0.084, 0.220)),
        material=alloy,
        name="clamp_neck",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.09, 0.17, 0.29)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.055, 0.125)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        _mesh(
            "left_swept_back_city_bar",
            tube_from_spline_points(
                [
                    (-0.060, 0.000, 0.000),
                    (-0.090, 0.000, 0.000),
                    (-0.155, -0.052, 0.025),
                    (-0.245, -0.120, 0.040),
                    (-0.330, -0.170, 0.050),
                ],
                radius=0.011,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=matte_black,
        name="left_bar_tube",
    )
    handlebar.visual(
        _mesh(
            "right_swept_back_city_bar",
            tube_from_spline_points(
                [
                    (0.060, 0.000, 0.000),
                    (0.090, 0.000, 0.000),
                    (0.155, -0.052, 0.025),
                    (0.245, -0.120, 0.040),
                    (0.330, -0.170, 0.050),
                ],
                radius=0.011,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=matte_black,
        name="right_bar_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.0127, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="clamp_section",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(
            xyz=(-0.318, -0.168, 0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(
            xyz=(0.318, -0.168, 0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.74, 0.30, 0.13)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.085, 0.028)),
    )

    model.articulation(
        "fork_to_stem",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.040, 0.288)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.6,
            lower=-0.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.110, 0.240)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.40,
        ),
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

    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    stem_joint = object_model.get_articulation("fork_to_stem")
    bar_joint = object_model.get_articulation("stem_to_handlebar")

    ctx.expect_contact(
        stem,
        fork,
        elem_a="pivot_barrel",
        elem_b="left_pivot_ear",
        name="stem pivot barrel bears against fork ear",
    )

    stem_lower = stem_joint.motion_limits.lower if stem_joint.motion_limits else None
    stem_upper = stem_joint.motion_limits.upper if stem_joint.motion_limits else None
    low_bar_pos = None
    high_bar_pos = None
    if stem_lower is not None:
        with ctx.pose({stem_joint: stem_lower}):
            low_bar_pos = ctx.part_world_position(handlebar)
    if stem_upper is not None:
        with ctx.pose({stem_joint: stem_upper}):
            high_bar_pos = ctx.part_world_position(handlebar)

    ctx.check(
        "stem angle raises the handlebar clamp",
        low_bar_pos is not None
        and high_bar_pos is not None
        and high_bar_pos[2] > low_bar_pos[2] + 0.08,
        details=f"low={low_bar_pos}, high={high_bar_pos}",
    )

    ctx.expect_within(
        handlebar,
        stem,
        axes="yz",
        inner_elem="clamp_section",
        outer_elem="bar_clamp_shell",
        margin=0.0005,
        name="bar clamp section stays centered inside stem clamp",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="x",
        elem_a="clamp_section",
        elem_b="bar_clamp_shell",
        min_overlap=0.060,
        name="bar clamp section spans the stem clamp width",
    )
    ctx.expect_contact(
        handlebar,
        stem,
        elem_a="clamp_section",
        elem_b="upper_clamp_land",
        name="handlebar bears on the stem clamp land",
    )
    def grip_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            (mins[0] + maxs[0]) * 0.5,
            (mins[1] + maxs[1]) * 0.5,
            (mins[2] + maxs[2]) * 0.5,
        )

    rest_right_grip = grip_center(ctx.part_element_world_aabb(handlebar, elem="right_grip"))
    rolled_right_grip = None
    with ctx.pose({bar_joint: 0.30}):
        ctx.expect_within(
            handlebar,
            stem,
            axes="yz",
            inner_elem="clamp_section",
            outer_elem="bar_clamp_shell",
            margin=0.0005,
            name="rolled bar stays nested in the clamp",
        )
        rolled_right_grip = grip_center(ctx.part_element_world_aabb(handlebar, elem="right_grip"))

    ctx.check(
        "handlebar rotation moves the grip around the clamp axis",
        rest_right_grip is not None
        and rolled_right_grip is not None
        and abs(rolled_right_grip[2] - rest_right_grip[2]) > 0.03,
        details=f"rest={rest_right_grip}, rolled={rolled_right_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
