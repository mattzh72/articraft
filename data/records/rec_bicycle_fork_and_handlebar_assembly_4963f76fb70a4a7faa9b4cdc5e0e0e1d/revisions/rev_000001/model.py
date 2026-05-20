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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


HEAD_ANGLE = math.radians(18.0)
HEAD_TUBE_LENGTH = 0.160


def _head_axis_offset(distance: float) -> tuple[float, float, float]:
    return (-math.sin(HEAD_ANGLE) * distance, 0.0, math.cos(HEAD_ANGLE) * distance)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        0.5 * (lo[0] + hi[0]),
        0.5 * (lo[1] + hi[1]),
        0.5 * (lo[2] + hi[2]),
    )


def _shell_sleeve(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    flare: float = 0.0012,
    segments: int = 48,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius - flare, -half),
            (outer_radius, -half * 0.45),
            (outer_radius, half * 0.45),
            (outer_radius - flare, half),
        ],
        [
            (inner_radius, -half),
            (inner_radius, -half * 0.45),
            (inner_radius, half * 0.45),
            (inner_radius, half),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aero_road_bike_front_end")

    carbon = model.material("carbon", rgba=(0.12, 0.13, 0.14, 1.0))
    matte_bar = model.material("matte_bar", rgba=(0.08, 0.08, 0.09, 1.0))
    alloy = model.material("alloy", rgba=(0.64, 0.66, 0.69, 1.0))
    headset_black = model.material("headset_black", rgba=(0.16, 0.16, 0.17, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    head_tube = model.part("head_tube")
    head_tube_shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.064, 0.046, 0.010, corner_segments=8),
        [rounded_rect_profile(0.040, 0.032, 0.006, corner_segments=8)],
        HEAD_TUBE_LENGTH,
        center=True,
    )
    head_tube.visual(
        mesh_from_geometry(head_tube_shell, "head_tube_shell"),
        origin=Origin(
            xyz=_head_axis_offset(HEAD_TUBE_LENGTH * 0.5),
            rpy=(0.0, -HEAD_ANGLE, 0.0),
        ),
        material=carbon,
        name="head_tube_shell",
    )
    head_tube.visual(
        mesh_from_geometry(
            _shell_sleeve(outer_radius=0.0245, inner_radius=0.0140, length=0.014),
            "lower_headset_cup_shell",
        ),
        origin=Origin(
            xyz=_head_axis_offset(0.007),
            rpy=(0.0, -HEAD_ANGLE, 0.0),
        ),
        material=headset_black,
        name="lower_headset_cup",
    )
    head_tube.visual(
        mesh_from_geometry(
            _shell_sleeve(outer_radius=0.0235, inner_radius=0.0140, length=0.012),
            "upper_headset_cup_shell",
        ),
        origin=Origin(
            xyz=_head_axis_offset(HEAD_TUBE_LENGTH - 0.006),
            rpy=(0.0, -HEAD_ANGLE, 0.0),
        ),
        material=headset_black,
        name="upper_headset_cup",
    )
    head_tube.visual(
        Box((0.050, 0.040, 0.030)),
        origin=Origin(
            xyz=(
                _head_axis_offset(0.030)[0] - 0.045,
                0.0,
                _head_axis_offset(0.030)[2] - 0.022,
            ),
            rpy=(0.0, -0.20, 0.0),
        ),
        material=carbon,
        name="down_tube_stub",
    )
    head_tube.visual(
        Box((0.036, 0.034, 0.020)),
        origin=Origin(
            xyz=(
                _head_axis_offset(HEAD_TUBE_LENGTH - 0.028)[0] - 0.038,
                0.0,
                _head_axis_offset(HEAD_TUBE_LENGTH - 0.028)[2] + 0.006,
            ),
            rpy=(0.0, -0.16, 0.0),
        ),
        material=carbon,
        name="top_tube_stub",
    )
    head_tube.inertial = Inertial.from_geometry(
        Box((0.120, 0.060, 0.220)),
        mass=1.0,
        origin=Origin(xyz=(-0.030, 0.0, 0.090)),
    )

    fork = model.part("fork")
    steerer_length = 0.255
    fork.visual(
        Cylinder(radius=0.014, length=steerer_length),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=headset_black,
        name="steerer_tube",
    )
    fork.visual(
        Box((0.076, 0.068, 0.028)),
        origin=Origin(xyz=(0.014, 0.0, -0.020)),
        material=carbon,
        name="fork_crown",
    )
    blade_profile = rounded_rect_profile(0.030, 0.014, 0.004, corner_segments=6)
    left_blade = sweep_profile_along_spline(
        [
            (0.002, 0.037, -0.004),
            (0.008, 0.043, -0.100),
            (0.018, 0.048, -0.205),
            (0.032, 0.052, -0.300),
            (0.044, 0.050, -0.382),
        ],
        profile=blade_profile,
        samples_per_segment=14,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    right_blade = sweep_profile_along_spline(
        [
            (0.002, -0.037, -0.004),
            (0.008, -0.043, -0.100),
            (0.018, -0.048, -0.205),
            (0.032, -0.052, -0.300),
            (0.044, -0.050, -0.382),
        ],
        profile=blade_profile,
        samples_per_segment=14,
        cap_profile=True,
        up_hint=(0.0, -1.0, 0.0),
    )
    fork.visual(
        mesh_from_geometry(left_blade, "left_fork_blade"),
        material=carbon,
        name="left_blade",
    )
    fork.visual(
        mesh_from_geometry(right_blade, "right_fork_blade"),
        material=carbon,
        name="right_blade",
    )
    fork.visual(
        Box((0.024, 0.020, 0.018)),
        origin=Origin(xyz=(0.046, 0.051, -0.383)),
        material=alloy,
        name="left_dropout",
    )
    fork.visual(
        Box((0.024, 0.020, 0.018)),
        origin=Origin(xyz=(0.046, -0.051, -0.383)),
        material=alloy,
        name="right_dropout",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.140, 0.120, 0.680)),
        mass=0.85,
        origin=Origin(xyz=(0.025, 0.0, -0.060)),
    )

    model.articulation(
        "head_tube_to_fork",
        ArticulationType.FIXED,
        parent=head_tube,
        child=fork,
        origin=Origin(rpy=(0.0, -HEAD_ANGLE, 0.0)),
    )

    stem = model.part("stem")
    stem.visual(
        mesh_from_geometry(
            _shell_sleeve(outer_radius=0.0225, inner_radius=0.0140, length=0.052, flare=0.0008),
            "steerer_collar_shell",
        ),
        origin=Origin(
            xyz=(-0.103, 0.0, -0.032),
            rpy=(0.0, -HEAD_ANGLE, 0.0),
        ),
        material=headset_black,
        name="steerer_collar",
    )
    stem.visual(
        Box((0.078, 0.028, 0.012)),
        origin=Origin(xyz=(-0.020, 0.0, 0.024), rpy=(0.0, 0.04, 0.0)),
        material=carbon,
        name="stem_body",
    )
    stem.visual(
        Box((0.086, 0.008, 0.024)),
        origin=Origin(xyz=(-0.061, 0.021, -0.018), rpy=(0.0, 0.04, 0.0)),
        material=carbon,
        name="left_stem_web",
    )
    stem.visual(
        Box((0.086, 0.008, 0.024)),
        origin=Origin(xyz=(-0.061, -0.021, -0.018), rpy=(0.0, 0.04, 0.0)),
        material=carbon,
        name="right_stem_web",
    )
    stem.visual(
        Box((0.040, 0.044, 0.014)),
        origin=Origin(xyz=(-0.032, 0.000, -0.033)),
        material=carbon,
        name="underside_fairing",
    )
    stem.visual(
        mesh_from_geometry(
            _shell_sleeve(outer_radius=0.0210, inner_radius=0.0159, length=0.058, flare=0.0007),
            "bar_clamp_shell",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carbon,
        name="bar_clamp_shell",
    )
    stem.visual(
        Box((0.010, 0.074, 0.040)),
        origin=Origin(xyz=(0.024, 0.000, 0.000)),
        material=carbon,
        name="front_face_plate",
    )
    for index, z_sign in enumerate((-1.0, 1.0)):
        stem.visual(
            Cylinder(radius=0.003, length=0.012),
            origin=Origin(
                xyz=(0.027, 0.019, z_sign * 0.012),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=alloy,
            name=f"left_face_bolt_{index}",
        )
        stem.visual(
            Cylinder(radius=0.003, length=0.012),
            origin=Origin(
                xyz=(0.027, -0.019, z_sign * 0.012),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=alloy,
            name=f"right_face_bolt_{index}",
        )
    stem.inertial = Inertial.from_geometry(
        Box((0.160, 0.060, 0.080)),
        mass=0.45,
        origin=Origin(xyz=(-0.040, 0.0, -0.012)),
    )

    model.articulation(
        "fork_to_stem",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(
            xyz=(0.105, 0.0, 0.235),
            rpy=(0.0, HEAD_ANGLE, 0.0),
        ),
    )

    handlebar = model.part("handlebar")
    drop_bar = tube_from_spline_points(
        [
            (0.010, -0.185, -0.158),
            (0.054, -0.181, -0.132),
            (0.074, -0.176, -0.086),
            (0.076, -0.169, -0.022),
            (0.050, -0.150, 0.006),
            (0.020, -0.112, 0.010),
            (0.000, -0.055, 0.000),
            (0.000, -0.020, 0.000),
            (0.000, 0.020, 0.000),
            (0.000, 0.055, 0.000),
            (0.020, 0.112, 0.010),
            (0.050, 0.150, 0.006),
            (0.076, 0.169, -0.022),
            (0.074, 0.176, -0.086),
            (0.054, 0.181, -0.132),
            (0.010, 0.185, -0.158),
        ],
        radius=0.0118,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    handlebar.visual(
        mesh_from_geometry(drop_bar, "drop_bar_tube"),
        material=matte_bar,
        name="bar_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.0159, length=0.120),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_bar,
        name="center_sleeve",
    )
    handlebar.visual(
        Box((0.032, 0.020, 0.076)),
        origin=Origin(xyz=(0.066, 0.157, -0.042), rpy=(0.18, 0.0, 0.0)),
        material=rubber,
        name="left_hood",
    )
    handlebar.visual(
        Box((0.032, 0.020, 0.076)),
        origin=Origin(xyz=(0.066, -0.157, -0.042), rpy=(0.18, 0.0, 0.0)),
        material=rubber,
        name="right_hood",
    )
    handlebar.visual(
        Cylinder(radius=0.0125, length=0.022),
        origin=Origin(
            xyz=(0.010, 0.185, -0.158),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="left_bar_end",
    )
    handlebar.visual(
        Cylinder(radius=0.0125, length=0.022),
        origin=Origin(
            xyz=(0.010, -0.185, -0.158),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="right_bar_end",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.180, 0.420, 0.240)),
        mass=0.34,
        origin=Origin(xyz=(0.040, 0.0, -0.060)),
    )

    model.articulation(
        "stem_to_handlebar",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=handlebar,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    bar_joint = object_model.get_articulation("stem_to_handlebar")

    ctx.allow_overlap(
        fork,
        head_tube,
        elem_a="steerer_tube",
        elem_b="lower_headset_cup",
        reason="The fork steerer intentionally passes through the lower headset cup bearing shell inside the head tube.",
    )
    ctx.allow_overlap(
        fork,
        head_tube,
        elem_a="steerer_tube",
        elem_b="upper_headset_cup",
        reason="The fork steerer intentionally passes through the upper headset cup bearing shell inside the head tube.",
    )
    ctx.allow_overlap(
        fork,
        stem,
        elem_a="steerer_tube",
        elem_b="steerer_collar",
        reason="The integrated stem collar intentionally clamps around the steerer tube at the top of the fork.",
    )
    ctx.allow_overlap(
        handlebar,
        stem,
        elem_a="center_sleeve",
        elem_b="bar_clamp_shell",
        reason="The handlebar center sleeve is intentionally modeled inside the stem clamp shell so the bar can rotate for angle adjustment.",
    )

    with ctx.pose({bar_joint: 0.0}):
        ctx.expect_within(
            fork,
            head_tube,
            axes="xy",
            inner_elem="steerer_tube",
            outer_elem="head_tube_shell",
            margin=0.015,
            name="fork steerer stays centered in the head tube",
        )
        ctx.expect_overlap(
            stem,
            fork,
            axes="z",
            elem_a="steerer_collar",
            elem_b="steerer_tube",
            min_overlap=0.045,
            name="integrated stem remains engaged on the steerer top",
        )
        ctx.expect_contact(
            stem,
            fork,
            elem_a="steerer_collar",
            elem_b="steerer_tube",
            name="stem clamp bears on the steerer tube",
        )
        ctx.expect_contact(
            stem,
            handlebar,
            elem_a="bar_clamp_shell",
            elem_b="center_sleeve",
            name="handlebar sleeve bears in the stem clamp bore",
        )
        ctx.expect_origin_distance(
            handlebar,
            stem,
            axes="xyz",
            max_dist=0.001,
            name="handlebar clamp axis is coincident with the stem clamp center",
        )

    rest_left_end = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="left_bar_end"))
    with ctx.pose({bar_joint: 0.35}):
        ctx.expect_origin_distance(
            handlebar,
            stem,
            axes="xyz",
            max_dist=0.001,
            name="handlebar rotates in place about the stem clamp axis",
        )
        adjusted_left_end = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="left_bar_end"))

    ctx.check(
        "handlebar adjustment pitches the drops forward around the clamp axis",
        rest_left_end is not None
        and adjusted_left_end is not None
        and adjusted_left_end[0] > rest_left_end[0] + 0.035
        and abs(adjusted_left_end[1] - rest_left_end[1]) < 0.010
        and adjusted_left_end[2] > rest_left_end[2] + 0.005,
        details=f"rest={rest_left_end}, adjusted={adjusted_left_end}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
