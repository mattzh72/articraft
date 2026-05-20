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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_shell(name: str, *, outer_radius: float, inner_radius: float, length: float, segments: int = 56):
    half = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return _mesh(name, geom)


def _rounded_plate(name: str, *, width: float, depth: float, thickness: float, corner_radius: float):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, corner_radius, corner_segments=8),
        thickness,
        center=True,
    )
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_jump_fork")

    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.88, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    headset_black = model.material("headset_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("frame_front")
    frame.visual(
        _tube_shell("bmx_head_tube", outer_radius=0.030, inner_radius=0.024, length=0.150),
        material=frame_paint,
        name="head_tube",
    )
    frame.visual(
        _mesh(
            "bmx_down_tube_stub",
            tube_from_spline_points(
                [
                    (0.0, -0.050, -0.050),
                    (0.0, -0.160, -0.104),
                    (0.0, -0.345, -0.168),
                ],
                radius=0.023,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_paint,
        name="down_tube_stub",
    )
    frame.visual(
        Box((0.014, 0.056, 0.112)),
        origin=Origin(xyz=(-0.028, -0.040, -0.006)),
        material=frame_paint,
        name="left_head_gusset",
    )
    frame.visual(
        Box((0.014, 0.056, 0.112)),
        origin=Origin(xyz=(0.028, -0.040, -0.006)),
        material=frame_paint,
        name="right_head_gusset",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.10, 0.38, 0.34)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.165, -0.020)),
    )

    steering = model.part("steering_assembly")
    steering.visual(
        Cylinder(radius=0.020, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=chrome,
        name="steerer_shaft",
    )
    steering.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=chrome,
        name="threaded_locknut",
    )
    steering.visual(
        _tube_shell("bmx_upper_bearing_race", outer_radius=0.029, inner_radius=0.021, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=chrome,
        name="upper_bearing_race",
    )
    steering.visual(
        _rounded_plate(
            "bmx_lower_crown",
            width=0.265,
            depth=0.080,
            thickness=0.020,
            corner_radius=0.016,
        ),
        origin=Origin(xyz=(0.0, 0.024, -0.105)),
        material=chrome,
        name="lower_crown",
    )
    steering.visual(
        _rounded_plate(
            "bmx_upper_crown",
            width=0.215,
            depth=0.060,
            thickness=0.016,
            corner_radius=0.013,
        ),
        origin=Origin(xyz=(0.0, 0.020, 0.086)),
        material=chrome,
        name="upper_crown",
    )
    steering.visual(
        Box((0.175, 0.022, 0.072)),
        origin=Origin(xyz=(0.0, 0.058, -0.040)),
        material=chrome,
        name="front_bridge",
    )
    steering.visual(
        Cylinder(radius=0.017, length=0.740),
        origin=Origin(xyz=(-0.097, 0.040, -0.260)),
        material=chrome,
        name="left_fork_leg",
    )
    steering.visual(
        Cylinder(radius=0.017, length=0.740),
        origin=Origin(xyz=(0.097, 0.040, -0.260)),
        material=chrome,
        name="right_fork_leg",
    )
    steering.visual(
        Box((0.022, 0.014, 0.050)),
        origin=Origin(xyz=(-0.097, 0.052, -0.578)),
        material=chrome,
        name="left_dropout",
    )
    steering.visual(
        Box((0.022, 0.014, 0.050)),
        origin=Origin(xyz=(0.097, 0.052, -0.578)),
        material=chrome,
        name="right_dropout",
    )
    steering.visual(
        Cylinder(radius=0.026, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=chrome,
        name="stem_socket",
    )
    steering.visual(
        _mesh(
            "bmx_stem_neck",
            tube_from_spline_points(
                [
                    (0.0, 0.000, 0.138),
                    (0.0, 0.012, 0.158),
                    (0.0, 0.026, 0.186),
                    (0.0, 0.018, 0.216),
                ],
                radius=0.013,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=chrome,
        name="stem_neck",
    )
    steering.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, 0.018, 0.220), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="bar_clamp_collar",
    )
    steering.visual(
        _mesh(
            "bmx_riser_bar",
            tube_from_spline_points(
                [
                    (-0.395, -0.080, 0.305),
                    (-0.305, -0.060, 0.296),
                    (-0.205, -0.028, 0.276),
                    (-0.110, 0.000, 0.240),
                    (-0.040, 0.012, 0.220),
                    (0.040, 0.012, 0.220),
                    (0.110, 0.000, 0.240),
                    (0.205, -0.028, 0.276),
                    (0.305, -0.060, 0.296),
                    (0.395, -0.080, 0.305),
                ],
                radius=0.0115,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=chrome,
        name="handlebar_main",
    )
    steering.visual(
        _mesh(
            "bmx_handlebar_crossbar",
            tube_from_spline_points(
                [
                    (-0.180, -0.020, 0.268),
                    (0.0, -0.006, 0.216),
                    (0.180, -0.020, 0.268),
                ],
                radius=0.009,
                samples_per_segment=22,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=chrome,
        name="crossbar",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(-0.410, -0.082, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(0.410, -0.082, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.88, 0.24, 0.94)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.000, -0.140)),
    )

    model.articulation(
        "head_tube_steer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=3.0,
            lower=-0.75,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_front")
    steering = object_model.get_part("steering_assembly")
    steer_joint = object_model.get_articulation("head_tube_steer")

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

    ctx.expect_within(
        steering,
        frame,
        axes="xy",
        inner_elem="steerer_shaft",
        outer_elem="head_tube",
        margin=0.005,
        name="steerer stays centered inside the head tube",
    )
    ctx.expect_overlap(
        steering,
        frame,
        axes="z",
        elem_a="steerer_shaft",
        elem_b="head_tube",
        min_overlap=0.145,
        name="steerer spans the full head tube",
    )
    ctx.expect_gap(
        frame,
        steering,
        axis="z",
        positive_elem="head_tube",
        negative_elem="lower_crown",
        min_gap=0.008,
        max_gap=0.045,
        name="lower triple clamp sits just below the head tube",
    )
    ctx.expect_gap(
        steering,
        frame,
        axis="z",
        positive_elem="threaded_locknut",
        negative_elem="head_tube",
        min_gap=0.006,
        max_gap=0.040,
        name="threaded steerer hardware protrudes above the head tube",
    )
    ctx.expect_contact(
        steering,
        frame,
        elem_a="upper_bearing_race",
        elem_b="head_tube",
        name="upper bearing race supports the steering assembly on the head tube",
    )

    lower_crown_aabb = ctx.part_element_world_aabb(steering, elem="lower_crown")
    bar_aabb = ctx.part_element_world_aabb(steering, elem="handlebar_main")
    crown_width = None if lower_crown_aabb is None else lower_crown_aabb[1][0] - lower_crown_aabb[0][0]
    bar_width = None if bar_aabb is None else bar_aabb[1][0] - bar_aabb[0][0]
    bar_rise = None if bar_aabb is None else bar_aabb[1][2] - bar_aabb[0][2]

    ctx.check(
        "triple-clamp crown reads wide",
        crown_width is not None and crown_width >= 0.24,
        details=f"lower crown width={crown_width}",
    )
    ctx.check(
        "riser handlebars are wide and upswept",
        bar_width is not None and bar_width >= 0.76 and bar_rise is not None and bar_rise >= 0.08,
        details=f"handlebar width={bar_width}, rise={bar_rise}",
    )

    left_grip_rest = ctx.part_element_world_aabb(steering, elem="left_grip")
    right_grip_rest = ctx.part_element_world_aabb(steering, elem="right_grip")
    upper = steer_joint.motion_limits.upper if steer_joint.motion_limits is not None else 0.75
    lower = steer_joint.motion_limits.lower if steer_joint.motion_limits is not None else -0.75

    with ctx.pose({steer_joint: upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at full positive steer")
        ctx.expect_within(
            steering,
            frame,
            axes="xy",
            inner_elem="steerer_shaft",
            outer_elem="head_tube",
            margin=0.005,
            name="steerer remains centered at full positive steer",
        )
        left_grip_positive = ctx.part_element_world_aabb(steering, elem="left_grip")
        right_grip_positive = ctx.part_element_world_aabb(steering, elem="right_grip")

    with ctx.pose({steer_joint: lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at full negative steer")
        ctx.expect_within(
            steering,
            frame,
            axes="xy",
            inner_elem="steerer_shaft",
            outer_elem="head_tube",
            margin=0.005,
            name="steerer remains centered at full negative steer",
        )
        left_grip_negative = ctx.part_element_world_aabb(steering, elem="left_grip")
        right_grip_negative = ctx.part_element_world_aabb(steering, elem="right_grip")

    left_rest_y = _aabb_center_y(left_grip_rest)
    right_rest_y = _aabb_center_y(right_grip_rest)
    left_positive_y = _aabb_center_y(left_grip_positive)
    right_positive_y = _aabb_center_y(right_grip_positive)
    left_negative_y = _aabb_center_y(left_grip_negative)
    right_negative_y = _aabb_center_y(right_grip_negative)

    ctx.check(
        "positive steer swings the bar ends around the steering axis",
        left_rest_y is not None
        and right_rest_y is not None
        and left_positive_y is not None
        and right_positive_y is not None
        and left_positive_y < left_rest_y - 0.12
        and right_positive_y > right_rest_y + 0.12,
        details=(
            f"left_rest_y={left_rest_y}, left_positive_y={left_positive_y}, "
            f"right_rest_y={right_rest_y}, right_positive_y={right_positive_y}"
        ),
    )
    ctx.check(
        "negative steer swings the bar ends in the opposite direction",
        left_rest_y is not None
        and right_rest_y is not None
        and left_negative_y is not None
        and right_negative_y is not None
        and left_negative_y > left_rest_y + 0.12
        and right_negative_y < right_rest_y - 0.12,
        details=(
            f"left_rest_y={left_rest_y}, left_negative_y={left_negative_y}, "
            f"right_rest_y={right_rest_y}, right_negative_y={right_negative_y}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
