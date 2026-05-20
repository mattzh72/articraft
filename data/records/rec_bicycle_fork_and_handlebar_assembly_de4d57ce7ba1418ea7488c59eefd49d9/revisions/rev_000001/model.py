from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _build_handlebar_side_mesh(y_sign: float):
    half_path = [
        (-0.002, y_sign * 0.060, 0.000),
        (-0.006, y_sign * 0.105, 0.010),
        (-0.018, y_sign * 0.165, 0.055),
        (-0.030, y_sign * 0.235, 0.115),
        (-0.042, y_sign * 0.315, 0.162),
        (-0.050, y_sign * 0.375, 0.178),
    ]
    return tube_from_spline_points(
        half_path,
        radius=0.0111,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_race_fork_front_end")

    carbon_black = model.material("carbon_black", color=(0.08, 0.08, 0.09))
    alloy_dark = model.material("alloy_dark", color=(0.22, 0.23, 0.25))
    steel_gray = model.material("steel_gray", color=(0.60, 0.61, 0.64))

    fork = model.part("fork")

    steerer_profile = superellipse_profile(0.042, 0.030, exponent=2.8, segments=56)
    steerer_geom = ExtrudeGeometry.from_z0(steerer_profile, 0.195, cap=True, closed=True)
    fork.visual(
        mesh_from_geometry(steerer_geom, "fork_steerer"),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=carbon_black,
        name="steerer_shell",
    )

    crown_geom = ExtrudeGeometry(
        rounded_rect_profile(0.068, 0.118, 0.012, corner_segments=8),
        0.028,
        cap=True,
        center=True,
        closed=True,
    )
    fork.visual(
        mesh_from_geometry(crown_geom, "fork_crown"),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=carbon_black,
        name="crown_shell",
    )

    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        y = y_sign * 0.056
        fork.visual(
            Cylinder(radius=0.014, length=0.040),
            origin=Origin(xyz=(0.0, y, -0.048)),
            material=carbon_black,
            name=f"{side_name}_blade_shoulder",
        )
        fork.visual(
            Cylinder(radius=0.012, length=0.320),
            origin=Origin(xyz=(0.0, y, -0.193)),
            material=carbon_black,
            name=f"{side_name}_blade",
        )
        fork.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(
                xyz=(0.0, y_sign * (0.056 + 0.012 + 0.010), -0.142),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel_gray,
            name=f"{side_name}_pad_mount",
        )
        fork.visual(
            Box((0.020, 0.012, 0.028)),
            origin=Origin(xyz=(0.006, y, -0.367)),
            material=alloy_dark,
            name=f"{side_name}_dropout",
        )

    fork.inertial = Inertial.from_geometry(
        Box((0.120, 0.130, 0.570)),
        mass=0.92,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    stem = model.part("stem")
    for side_name, y in (("left", 0.020), ("right", -0.020)):
        stem.visual(
            Box((0.058, 0.010, 0.044)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=alloy_dark,
            name=f"steerer_clamp_{side_name}",
        )
    stem.visual(
        Box((0.008, 0.030, 0.044)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=alloy_dark,
        name="steerer_clamp_front",
    )
    stem.visual(
        Box((0.008, 0.030, 0.044)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=alloy_dark,
        name="steerer_clamp_rear",
    )
    stem.visual(
        Box((0.056, 0.028, 0.014)),
        origin=Origin(xyz=(0.053, 0.0, -0.024)),
        material=alloy_dark,
        name="extension_beam",
    )
    stem.visual(
        Box((0.050, 0.042, 0.0065)),
        origin=Origin(xyz=(0.075, 0.0, 0.01875)),
        material=alloy_dark,
        name="bar_clamp_top",
    )
    stem.visual(
        Box((0.050, 0.042, 0.0065)),
        origin=Origin(xyz=(0.075, 0.0, -0.01875)),
        material=alloy_dark,
        name="bar_clamp_bottom",
    )
    stem.visual(
        Box((0.0095, 0.042, 0.044)),
        origin=Origin(xyz=(0.09525, 0.0, 0.0)),
        material=alloy_dark,
        name="bar_clamp_front",
    )
    stem.visual(
        Box((0.0095, 0.042, 0.044)),
        origin=Origin(xyz=(0.05475, 0.0, 0.0)),
        material=alloy_dark,
        name="bar_clamp_rear",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.130, 0.060, 0.070)),
        mass=0.34,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0155, length=0.180),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carbon_black,
        name="center_section",
    )
    handlebar.visual(
        mesh_from_geometry(_build_handlebar_side_mesh(1.0), "bmx_handlebar_left"),
        material=carbon_black,
        name="left_bar_side",
    )
    handlebar.visual(
        mesh_from_geometry(_build_handlebar_side_mesh(-1.0), "bmx_handlebar_right"),
        material=carbon_black,
        name="right_bar_side",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.120, 0.780, 0.220)),
        mass=0.66,
        origin=Origin(xyz=(-0.020, 0.0, 0.095)),
    )

    model.articulation(
        "fork_to_stem",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=6.0,
            lower=-1.05,
            upper=1.05,
        ),
    )

    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
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
    fork_steerer = fork.get_visual("steerer_shell")
    stem_steerer_clamp = stem.get_visual("steerer_clamp_front")
    stem_bar_clamp = stem.get_visual("bar_clamp_bottom")
    handlebar_center = handlebar.get_visual("center_section")
    steering = object_model.get_articulation("fork_to_stem")
    bar_roll = object_model.get_articulation("stem_to_handlebar")

    ctx.check(
        "steering joint is vertical revolute",
        steering.joint_type == ArticulationType.REVOLUTE and steering.axis == (0.0, 0.0, 1.0),
        details=f"type={steering.joint_type}, axis={steering.axis}",
    )
    ctx.check(
        "bar roll joint is lateral revolute",
        bar_roll.joint_type == ArticulationType.REVOLUTE and bar_roll.axis == (0.0, 1.0, 0.0),
        details=f"type={bar_roll.joint_type}, axis={bar_roll.axis}",
    )

    ctx.expect_origin_distance(
        stem,
        fork,
        axes="xy",
        max_dist=1e-6,
        name="stem stays centered on the steerer axis",
    )
    ctx.expect_origin_distance(
        handlebar,
        stem,
        axes="yz",
        max_dist=1e-6,
        name="handlebar clamp center stays aligned to the stem clamp",
    )
    ctx.expect_origin_gap(
        handlebar,
        stem,
        axis="x",
        min_gap=0.070,
        max_gap=0.080,
        name="handlebar clamp sits forward of the steerer clamp",
    )
    ctx.expect_contact(
        stem,
        fork,
        elem_a=stem_steerer_clamp,
        elem_b=fork_steerer,
        name="stem clamp bears on the oval steerer",
    )
    ctx.expect_contact(
        stem,
        handlebar,
        elem_a=stem_bar_clamp,
        elem_b=handlebar_center,
        name="stem bar clamp bears on the handlebar center section",
    )

    fork_aabb = ctx.part_world_aabb(fork)
    handlebar_aabb = ctx.part_world_aabb(handlebar)
    fork_width = None if fork_aabb is None else fork_aabb[1][1] - fork_aabb[0][1]
    handlebar_width = None if handlebar_aabb is None else handlebar_aabb[1][1] - handlebar_aabb[0][1]
    handlebar_rise = None if handlebar_aabb is None else handlebar_aabb[1][2] - handlebar_aabb[0][2]

    ctx.check(
        "fork has wide blade stance",
        fork_width is not None and 0.16 <= fork_width <= 0.19,
        details=f"fork_width={fork_width}",
    )
    ctx.check(
        "handlebar reads as a wide riser bar",
        handlebar_width is not None
        and handlebar_width >= 0.72
        and handlebar_rise is not None
        and handlebar_rise >= 0.18,
        details=f"handlebar_width={handlebar_width}, handlebar_rise={handlebar_rise}",
    )

    rest_bar_pos = ctx.part_world_position(handlebar)
    with ctx.pose({steering: 0.60}):
        turned_bar_pos = ctx.part_world_position(handlebar)
    ctx.check(
        "positive steering swings the bar clamp toward positive y",
        rest_bar_pos is not None
        and turned_bar_pos is not None
        and turned_bar_pos[1] > rest_bar_pos[1] + 0.035
        and turned_bar_pos[0] < rest_bar_pos[0] - 0.010,
        details=f"rest={rest_bar_pos}, turned={turned_bar_pos}",
    )

    rest_bar_aabb = ctx.part_world_aabb(handlebar)
    with ctx.pose({bar_roll: 0.28}):
        rolled_bar_aabb = ctx.part_world_aabb(handlebar)
    rest_bar_top = None if rest_bar_aabb is None else rest_bar_aabb[1][2]
    rolled_bar_top = None if rolled_bar_aabb is None else rolled_bar_aabb[1][2]
    ctx.check(
        "positive bar roll lifts the grip line",
        rest_bar_top is not None and rolled_bar_top is not None and rolled_bar_top > rest_bar_top + 0.008,
        details=f"rest_top={rest_bar_top}, rolled_top={rolled_bar_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
