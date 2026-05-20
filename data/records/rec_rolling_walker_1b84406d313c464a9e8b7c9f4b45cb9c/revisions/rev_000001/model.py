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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    frame_finish = model.material("frame_finish", rgba=(0.77, 0.79, 0.81, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    fork_finish = model.material("fork_finish", rgba=(0.58, 0.60, 0.63, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.71, 0.73, 0.75, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.66, 0.58, 0.86)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    tube_radius = 0.014
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x_pos = side_sign * 0.29
        front_x = side_sign * 0.30
        frame.visual(
            Cylinder(radius=tube_radius, length=0.78),
            origin=Origin(xyz=(x_pos, -0.18, 0.44)),
            material=frame_finish,
            name=f"{side_name}_rear_upright",
        )
        frame.visual(
            Cylinder(radius=tube_radius, length=0.68),
            origin=Origin(xyz=(front_x, 0.20, 0.49)),
            material=frame_finish,
            name=f"{side_name}_front_upright",
        )
        frame.visual(
            Cylinder(radius=tube_radius, length=0.38),
            origin=Origin(xyz=(x_pos, 0.01, 0.83), rpy=(pi / 2.0, 0.0, 0.0)),
            material=frame_finish,
            name=f"{side_name}_top_rail",
        )
        frame.visual(
            Cylinder(radius=tube_radius, length=0.38),
            origin=Origin(xyz=(front_x, 0.01, 0.17), rpy=(pi / 2.0, 0.0, 0.0)),
            material=frame_finish,
            name=f"{side_name}_lower_rail",
        )
        frame.visual(
            Cylinder(radius=0.024, length=0.12),
            origin=Origin(xyz=(x_pos, -0.04, 0.83), rpy=(pi / 2.0, 0.0, 0.0)),
            material=grip_foam,
            name=f"{side_name}_grip",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.050),
            origin=Origin(xyz=(x_pos, -0.18, 0.025)),
            material=rubber,
            name=f"{side_name}_rear_tip",
        )
        frame.visual(
            Cylinder(radius=tube_radius * 1.08, length=0.040),
            origin=Origin(xyz=(front_x, 0.20, 0.170)),
            material=frame_finish,
            name=f"{side_name}_caster_socket",
        )

    frame.visual(
        Cylinder(radius=tube_radius, length=0.64),
        origin=Origin(xyz=(0.0, 0.09, 0.17), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_finish,
        name="front_crossbar",
    )
    frame.visual(
        Cylinder(radius=tube_radius, length=0.64),
        origin=Origin(xyz=(0.0, -0.09, 0.17), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_finish,
        name="rear_crossbar",
    )

    for fork_name in ("left_caster_fork", "right_caster_fork"):
        fork = model.part(fork_name)
        fork.visual(
            Cylinder(radius=0.009, length=0.058),
            origin=Origin(xyz=(0.0, 0.0, -0.029)),
            material=fork_finish,
            name="stem",
        )
        fork.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.063)),
            material=fork_finish,
            name="swivel_head",
        )
        fork.visual(
            Box((0.038, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, -0.010, -0.074)),
            material=fork_finish,
            name="crown",
        )
        fork.visual(
            Box((0.0040, 0.018, 0.072)),
            origin=Origin(xyz=(0.016, -0.010, -0.113)),
            material=fork_finish,
            name="outer_plate",
        )
        fork.visual(
            Box((0.0040, 0.018, 0.072)),
            origin=Origin(xyz=(-0.016, -0.010, -0.113)),
            material=fork_finish,
            name="inner_plate",
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.044, 0.030, 0.120)),
            mass=0.22,
            origin=Origin(xyz=(0.0, -0.007, -0.067)),
        )

    for wheel_name in ("left_caster_wheel", "right_caster_wheel"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.038, length=0.024),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="tread",
        )
        wheel.visual(
            Cylinder(radius=0.022, length=0.026),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_finish,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.034),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=fork_finish,
            name="axle",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.038, length=0.024),
            mass=0.28,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="left_caster_fork",
        origin=Origin(xyz=(-0.30, 0.22, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="right_caster_fork",
        origin=Origin(xyz=(0.30, 0.22, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="left_caster_fork",
        child="left_caster_wheel",
        origin=Origin(xyz=(0.0, -0.010, -0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="right_caster_fork",
        child="right_caster_wheel",
        origin=Origin(xyz=(0.0, -0.010, -0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=18.0),
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
    left_wheel = object_model.get_part("left_caster_wheel")
    right_wheel = object_model.get_part("right_caster_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        min_corner, max_corner = frame_aabb
        ctx.check(
            "walker overall height is realistic",
            0.78 <= max_corner[2] - min_corner[2] <= 0.92,
            details=f"aabb={frame_aabb}",
        )
        ctx.check(
            "walker has a wide low stance",
            (max_corner[0] - min_corner[0]) >= 0.56 and (max_corner[1] - min_corner[1]) >= 0.40,
            details=f"aabb={frame_aabb}",
        )

    ctx.expect_origin_gap(
        left_wheel,
        frame,
        axis="y",
        min_gap=0.15,
        name="left front caster sits ahead of the rear frame mass",
    )
    ctx.expect_origin_gap(
        right_wheel,
        frame,
        axis="y",
        min_gap=0.15,
        name="right front caster sits ahead of the rear frame mass",
    )

    left_tip_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_tip")
    right_tip_aabb = ctx.part_element_world_aabb(frame, elem="right_rear_tip")
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    if None not in (left_tip_aabb, right_tip_aabb, left_wheel_aabb, right_wheel_aabb):
        support_bottoms = [
            left_tip_aabb[0][2],
            right_tip_aabb[0][2],
            left_wheel_aabb[0][2],
            right_wheel_aabb[0][2],
        ]
        ctx.check(
            "all four supports land near one ground plane",
            max(support_bottoms) - min(support_bottoms) <= 0.01,
            details=f"bottoms={support_bottoms}",
        )

    ctx.check(
        "caster swivel joints are vertical",
        left_swivel.axis == (0.0, 0.0, 1.0) and right_swivel.axis == (0.0, 0.0, 1.0),
        details=f"left={left_swivel.axis}, right={right_swivel.axis}",
    )
    ctx.check(
        "caster wheels spin on their axles",
        left_spin.axis == (1.0, 0.0, 0.0) and right_spin.axis == (1.0, 0.0, 0.0),
        details=f"left={left_spin.axis}, right={right_spin.axis}",
    )

    left_wheel_rest = ctx.part_world_position(left_wheel)
    with ctx.pose(left_caster_swivel=pi / 2.0):
        left_wheel_turned = ctx.part_world_position(left_wheel)
    ctx.check(
        "left caster actually swivels about a vertical stem",
        left_wheel_rest is not None
        and left_wheel_turned is not None
        and abs(left_wheel_turned[0] - left_wheel_rest[0]) > 0.008
        and abs(left_wheel_turned[2] - left_wheel_rest[2]) <= 1e-6,
        details=f"rest={left_wheel_rest}, turned={left_wheel_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
