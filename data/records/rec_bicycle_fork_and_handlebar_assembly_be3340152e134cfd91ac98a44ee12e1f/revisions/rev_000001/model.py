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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell(*, outer_radius: float, inner_radius: float, length: float, segments: int = 56):
    half = length * 0.5
    outer_profile = [(outer_radius, -half), (outer_radius, half)]
    inner_profile = [(inner_radius, -half), (inner_radius, half)]
    return LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=segments)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


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
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gravel_bike_front_end")

    carbon_black = model.material("carbon_black", rgba=(0.11, 0.12, 0.13, 1.0))
    stem_black = model.material("stem_black", rgba=(0.13, 0.13, 0.14, 1.0))
    alloy_gray = model.material("alloy_gray", rgba=(0.48, 0.49, 0.52, 1.0))

    fork = model.part("fork")

    fork.visual(
        Cylinder(radius=0.0144, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=carbon_black,
        name="steerer_shell",
    )
    fork.visual(
        _save_mesh(
            "fork_steerer_taper",
            LatheGeometry.from_shell_profiles(
                [(0.019, -0.018), (0.0185, 0.020), (0.0166, 0.060), (0.0144, 0.100)],
                [(0.016, -0.018), (0.0156, 0.020), (0.0138, 0.060), (0.0118, 0.100)],
                segments=72,
            ),
        ),
        material=carbon_black,
        name="steerer_taper",
    )
    fork.visual(
        Box((0.080, 0.036, 0.030)),
        origin=Origin(xyz=(0.0, 0.008, -0.018)),
        material=carbon_black,
        name="crown_body",
    )
    fork.visual(
        Box((0.050, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.024, -0.020)),
        material=carbon_black,
        name="crown_nose",
    )

    blade_profile = rounded_rect_profile(0.018, 0.034, radius=0.0045, corner_segments=8)
    left_blade_path = [
        (0.024, 0.006, -0.010),
        (0.028, 0.009, -0.085),
        (0.036, 0.013, -0.195),
        (0.046, 0.018, -0.315),
        (0.058, 0.022, -0.405),
    ]
    right_blade_path = _mirror_x(left_blade_path)

    fork.visual(
        _save_mesh(
            "fork_left_blade",
            sweep_profile_along_spline(
                left_blade_path,
                profile=blade_profile,
                samples_per_segment=18,
                cap_profile=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
        ),
        material=carbon_black,
        name="left_blade",
    )
    fork.visual(
        _save_mesh(
            "fork_right_blade",
            sweep_profile_along_spline(
                right_blade_path,
                profile=blade_profile,
                samples_per_segment=18,
                cap_profile=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
        ),
        material=carbon_black,
        name="right_blade",
    )

    fork.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.058, 0.022, -0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy_gray,
        name="left_dropout",
    )
    fork.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.058, 0.022, -0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy_gray,
        name="right_dropout",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.160, 0.090, 0.665)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.012, -0.0825)),
    )

    stem = model.part("stem")
    stem.visual(
        Box((0.011, 0.046, 0.046)),
        origin=Origin(xyz=(0.0199, 0.0, 0.0)),
        material=stem_black,
        name="rear_left_cheek",
    )
    stem.visual(
        Box((0.011, 0.046, 0.046)),
        origin=Origin(xyz=(-0.0199, 0.0, 0.0)),
        material=stem_black,
        name="rear_right_cheek",
    )
    stem.visual(
        Box((0.040, 0.014, 0.046)),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=stem_black,
        name="rear_clamp_bridge",
    )
    stem.visual(
        Box((0.028, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.039, 0.0)),
        material=stem_black,
        name="stem_body",
    )
    stem.visual(
        Box((0.024, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.054, -0.016)),
        material=stem_black,
        name="stem_underside",
    )
    stem.visual(
        Box((0.024, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.054, 0.016)),
        material=stem_black,
        name="stem_top_web",
    )
    stem.visual(
        Box((0.050, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.0545, 0.0)),
        material=stem_black,
        name="front_clamp_back",
    )
    stem.visual(
        Box((0.050, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.072, 0.0215)),
        material=stem_black,
        name="front_clamp_top",
    )
    stem.visual(
        Box((0.050, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.072, -0.0215)),
        material=stem_black,
        name="front_clamp_bottom",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.055, 0.110, 0.052)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0155, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon_black,
        name="center_bulge",
    )

    left_outer_path = [
        (0.055, 0.0, 0.0),
        (0.120, 0.010, 0.020),
        (0.195, 0.036, 0.020),
        (0.232, 0.032, -0.008),
        (0.260, 0.010, -0.062),
        (0.278, -0.018, -0.112),
    ]
    right_outer_path = _mirror_x(left_outer_path)
    handlebar.visual(
        _save_mesh(
            "handlebar_left_outer",
            tube_from_spline_points(
                left_outer_path,
                radius=0.0115,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
        ),
        material=carbon_black,
        name="left_outer",
    )
    handlebar.visual(
        _save_mesh(
            "handlebar_right_outer",
            tube_from_spline_points(
                right_outer_path,
                radius=0.0115,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
                up_hint=(0.0, 0.0, 1.0),
            ),
        ),
        material=carbon_black,
        name="right_outer",
    )

    left_grip_start = (0.276, -0.020, -0.112)
    left_grip_end = (0.320, -0.062, -0.056)
    right_grip_start = (-0.276, -0.020, -0.112)
    right_grip_end = (-0.320, -0.062, -0.056)
    handlebar.visual(
        Cylinder(radius=0.0125, length=_distance(left_grip_start, left_grip_end)),
        origin=Origin(
            xyz=_midpoint(left_grip_start, left_grip_end),
            rpy=_rpy_for_cylinder(left_grip_start, left_grip_end),
        ),
        material=carbon_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.0125, length=_distance(right_grip_start, right_grip_end)),
        origin=Origin(
            xyz=_midpoint(right_grip_start, right_grip_end),
            rpy=_rpy_for_cylinder(right_grip_start, right_grip_end),
        ),
        material=carbon_black,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.670, 0.160, 0.165)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.005, -0.040)),
    )

    model.articulation(
        "steerer_to_stem",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=3.0,
            lower=-1.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.076, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    steer_joint = object_model.get_articulation("steerer_to_stem")
    bar_roll_joint = object_model.get_articulation("stem_to_handlebar")

    ctx.expect_contact(
        stem,
        fork,
        elem_a="rear_left_cheek",
        elem_b="steerer_shell",
        contact_tol=0.0005,
        name="stem steerer clamp cheek seats on the steerer",
    )
    ctx.expect_contact(
        handlebar,
        stem,
        elem_a="center_bulge",
        elem_b="front_clamp_back",
        contact_tol=0.0005,
        name="stem clamp body meets the handlebar center section",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="x",
        elem_a="center_bulge",
        elem_b="front_clamp_back",
        min_overlap=0.048,
        name="stem front clamp spans the handlebar bulge",
    )

    left_dropout_center = _aabb_center(ctx.part_element_world_aabb(fork, elem="left_dropout"))
    right_dropout_center = _aabb_center(ctx.part_element_world_aabb(fork, elem="right_dropout"))
    ctx.check(
        "fork blades reach gravel dropout width",
        left_dropout_center is not None
        and right_dropout_center is not None
        and abs(left_dropout_center[0] - right_dropout_center[0]) > 0.10,
        details=f"left={left_dropout_center}, right={right_dropout_center}",
    )

    rest_bar_origin = ctx.part_world_position(handlebar)
    with ctx.pose({steer_joint: 0.45}):
        steered_bar_origin = ctx.part_world_position(handlebar)
    ctx.check(
        "steerer joint swings stem and bar around the fork axis",
        rest_bar_origin is not None
        and steered_bar_origin is not None
        and steered_bar_origin[0] > rest_bar_origin[0] + 0.02,
        details=f"rest={rest_bar_origin}, steered={steered_bar_origin}",
    )

    rest_right_grip = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="right_grip"))
    with ctx.pose({bar_roll_joint: 0.30}):
        rolled_right_grip = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="right_grip"))
    ctx.check(
        "handlebar roll joint reorients the drops in the stem clamp",
        rest_right_grip is not None
        and rolled_right_grip is not None
        and abs(rolled_right_grip[2] - rest_right_grip[2]) > 0.015,
        details=f"rest={rest_right_grip}, rolled={rolled_right_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
