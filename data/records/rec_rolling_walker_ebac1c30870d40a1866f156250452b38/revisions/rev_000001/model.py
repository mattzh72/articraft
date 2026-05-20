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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.29, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    tip_rubber = model.material("tip_rubber", rgba=(0.20, 0.20, 0.21, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    fork_gray = model.material("fork_gray", rgba=(0.62, 0.64, 0.67, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.68, 0.52, 0.92)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    left_side_main = tube_from_spline_points(
        [
            (0.212, 0.174, 0.16),
            (0.21, 0.16, 0.28),
            (0.23, 0.10, 0.60),
            (0.26, -0.02, 0.81),
            (0.285, -0.10, 0.865),
            (0.292, -0.14, 0.79),
            (0.292, -0.16, 0.46),
            (0.29, -0.18, 0.05),
        ],
        radius=0.013,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(_save_mesh("walker_left_side_main", left_side_main), material=aluminum, name="left_side_main")
    frame.visual(
        _save_mesh("walker_right_side_main", tube_from_spline_points(
            _mirror_x(
                [
                    (0.212, 0.174, 0.16),
                    (0.21, 0.16, 0.28),
                    (0.23, 0.10, 0.60),
                    (0.26, -0.02, 0.81),
                    (0.285, -0.10, 0.865),
                    (0.292, -0.14, 0.79),
                    (0.292, -0.16, 0.46),
                    (0.29, -0.18, 0.05),
                ]
            ),
            radius=0.013,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        )),
        material=aluminum,
        name="right_side_main",
    )

    frame.visual(
        _save_mesh(
            "walker_left_lower_brace",
            tube_from_spline_points(
                [
                    (0.212, 0.168, 0.14),
                    (0.215, 0.13, 0.17),
                    (0.235, 0.04, 0.22),
                    (0.255, -0.05, 0.245),
                    (0.275, -0.11, 0.255),
                ],
                radius=0.011,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=aluminum,
        name="left_lower_brace",
    )
    frame.visual(
        _save_mesh(
            "walker_right_lower_brace",
            tube_from_spline_points(
                _mirror_x(
                    [
                        (0.212, 0.168, 0.14),
                        (0.215, 0.13, 0.17),
                        (0.235, 0.04, 0.22),
                        (0.255, -0.05, 0.245),
                        (0.275, -0.11, 0.255),
                    ]
                ),
                radius=0.011,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=aluminum,
        name="right_lower_brace",
    )

    frame.visual(
        _save_mesh(
            "walker_front_crossbar",
            tube_from_spline_points(
                [
                    (-0.218, 0.115, 0.575),
                    (-0.08, 0.112, 0.58),
                    (0.08, 0.112, 0.58),
                    (0.218, 0.115, 0.575),
                ],
                radius=0.012,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=aluminum,
        name="front_crossbar",
    )
    frame.visual(
        _save_mesh(
            "walker_lower_crossbar",
            tube_from_spline_points(
                [
                    (-0.225, 0.045, 0.22),
                    (-0.09, 0.05, 0.21),
                    (0.09, 0.05, 0.21),
                    (0.225, 0.045, 0.22),
                ],
                radius=0.011,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=aluminum,
        name="lower_crossbar",
    )
    frame.visual(
        _save_mesh(
            "walker_rear_crossbar",
            tube_from_spline_points(
                [
                    (-0.275, -0.11, 0.255),
                    (-0.11, -0.108, 0.258),
                    (0.11, -0.108, 0.258),
                    (0.275, -0.11, 0.255),
                ],
                radius=0.011,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=aluminum,
        name="rear_crossbar",
    )

    frame.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.20, 0.18, 0.14)),
        material=dark_metal,
        name="left_front_socket",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(-0.20, 0.18, 0.14)),
        material=dark_metal,
        name="right_front_socket",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(0.275, -0.085, 0.86), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(-0.275, -0.085, 0.86), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(xyz=(0.29, -0.18, 0.025)),
        material=tip_rubber,
        name="left_rear_foot",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(xyz=(-0.29, -0.18, 0.025)),
        material=tip_rubber,
        name="right_rear_foot",
    )

    left_caster = model.part("left_caster")
    left_caster.inertial = Inertial.from_geometry(
        Box((0.042, 0.062, 0.092)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.000, -0.046)),
    )
    left_caster.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_metal,
        name="swivel_stem",
    )
    left_caster.visual(
        Box((0.014, 0.038, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, -0.022)),
        material=fork_gray,
        name="yoke_neck",
    )
    left_caster.visual(
        Box((0.040, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.023, -0.030)),
        material=fork_gray,
        name="fork_crown",
    )
    left_caster.visual(
        Box((0.007, 0.060, 0.060)),
        origin=Origin(xyz=(0.016, -0.001, -0.052)),
        material=fork_gray,
        name="fork_blade_outer",
    )
    left_caster.visual(
        Box((0.007, 0.060, 0.060)),
        origin=Origin(xyz=(-0.016, -0.001, -0.052)),
        material=fork_gray,
        name="fork_blade_inner",
    )
    left_caster.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.018, -0.030, -0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_boss_outer",
    )
    left_caster.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(-0.018, -0.030, -0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_boss_inner",
    )

    right_caster = model.part("right_caster")
    right_caster.inertial = Inertial.from_geometry(
        Box((0.042, 0.062, 0.092)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.000, -0.046)),
    )
    right_caster.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_metal,
        name="swivel_stem",
    )
    right_caster.visual(
        Box((0.014, 0.038, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, -0.022)),
        material=fork_gray,
        name="yoke_neck",
    )
    right_caster.visual(
        Box((0.040, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.023, -0.030)),
        material=fork_gray,
        name="fork_crown",
    )
    right_caster.visual(
        Box((0.007, 0.060, 0.060)),
        origin=Origin(xyz=(0.016, -0.001, -0.052)),
        material=fork_gray,
        name="fork_blade_outer",
    )
    right_caster.visual(
        Box((0.007, 0.060, 0.060)),
        origin=Origin(xyz=(-0.016, -0.001, -0.052)),
        material=fork_gray,
        name="fork_blade_inner",
    )
    right_caster.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.018, -0.030, -0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_boss_outer",
    )
    right_caster.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(-0.018, -0.030, -0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_boss_inner",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.022),
        mass=0.18,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    left_wheel.visual(
        Cylinder(radius=0.048, length=0.022),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=fork_gray,
        name="hub",
    )
    left_wheel.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="bearing_core",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.022),
        mass=0.18,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    right_wheel.visual(
        Cylinder(radius=0.048, length=0.022),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=fork_gray,
        name="hub",
    )
    right_wheel.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="bearing_core",
    )

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_caster,
        origin=Origin(xyz=(0.20, 0.18, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_caster,
        origin=Origin(xyz=(-0.20, 0.18, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster,
        child=left_wheel,
        origin=Origin(xyz=(0.0, -0.030, -0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster,
        child=right_wheel,
        origin=Origin(xyz=(0.0, -0.030, -0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=0.38,
        max_dist=0.42,
        name="front caster pair stays narrow",
    )

    left_front = ctx.part_world_position(left_wheel)
    right_front = ctx.part_world_position(right_wheel)
    left_rear_foot = ctx.part_element_world_aabb(frame, elem="left_rear_foot")
    right_rear_foot = ctx.part_element_world_aabb(frame, elem="right_rear_foot")

    if left_rear_foot is not None and right_rear_foot is not None and left_front is not None and right_front is not None:
        rear_left_x = 0.5 * (left_rear_foot[0][0] + left_rear_foot[1][0])
        rear_right_x = 0.5 * (right_rear_foot[0][0] + right_rear_foot[1][0])
        rear_left_y = 0.5 * (left_rear_foot[0][1] + left_rear_foot[1][1])
        rear_right_y = 0.5 * (right_rear_foot[0][1] + right_rear_foot[1][1])
        front_span = abs(left_front[0] - right_front[0])
        rear_span = abs(rear_left_x - rear_right_x)
        rear_y = 0.5 * (rear_left_y + rear_right_y)
        front_y = 0.5 * (left_front[1] + right_front[1])
        ctx.check(
            "rear support pair is broader than the front caster pair",
            rear_span > front_span + 0.12,
            details=f"rear_span={rear_span:.3f}, front_span={front_span:.3f}",
        )
        ctx.check(
            "rear support pair sits behind the front casters",
            rear_y < front_y - 0.25,
            details=f"rear_y={rear_y:.3f}, front_y={front_y:.3f}",
        )
    else:
        ctx.fail("support layout measurements available", "Could not resolve wheel or rear-foot world geometry.")

    ctx.check(
        "caster swivel joints are vertical revolutes",
        left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_swivel.articulation_type == ArticulationType.REVOLUTE
        and right_swivel.articulation_type == ArticulationType.REVOLUTE,
        details=f"left_axis={left_swivel.axis}, right_axis={right_swivel.axis}",
    )
    ctx.check(
        "caster wheel joints spin continuously about their axles",
        left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0)
        and left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"left_axis={left_spin.axis}, right_axis={right_spin.axis}",
    )

    rest_left = ctx.part_world_position(left_wheel)
    rest_right = ctx.part_world_position(right_wheel)
    with ctx.pose({left_swivel: 0.9, right_swivel: -0.9}):
        turned_left = ctx.part_world_position(left_wheel)
        turned_right = ctx.part_world_position(right_wheel)

    ctx.check(
        "left caster swivel changes wheel heading and placement",
        rest_left is not None
        and turned_left is not None
        and turned_left[0] > rest_left[0] + 0.015
        and turned_left[1] > rest_left[1] + 0.005,
        details=f"rest_left={rest_left}, turned_left={turned_left}",
    )
    ctx.check(
        "right caster swivel changes wheel heading and placement",
        rest_right is not None
        and turned_right is not None
        and turned_right[0] < rest_right[0] - 0.015
        and turned_right[1] > rest_right[1] + 0.005,
        details=f"rest_right={rest_right}, turned_right={turned_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
