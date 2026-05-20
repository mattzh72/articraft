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
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.83, 0.85, 0.87, 1.0))
    grip_black = model.material("grip_black", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.66, 0.44, 0.92)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.01, 0.46)),
    )

    tube_radius = 0.0125
    half_width = 0.27

    left_side_points = [
        (half_width, -0.17, 0.05),
        (half_width, -0.15, 0.48),
        (half_width, -0.08, 0.89),
        (half_width, 0.12, 0.85),
        (half_width, 0.20, 0.20),
    ]
    right_side_points = [(-x, y, z) for x, y, z in left_side_points]

    frame.visual(
        _save_mesh(
            "left_side_frame",
            wire_from_points(
                left_side_points,
                radius=tube_radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.05,
                corner_segments=12,
            ),
        ),
        material=aluminum,
        name="left_side_frame",
    )
    frame.visual(
        _save_mesh(
            "right_side_frame",
            wire_from_points(
                right_side_points,
                radius=tube_radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.05,
                corner_segments=12,
            ),
        ),
        material=aluminum,
        name="right_side_frame",
    )

    frame.visual(
        Cylinder(radius=0.012, length=0.54),
        origin=Origin(xyz=(0.0, 0.20, 0.175), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="front_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.54),
        origin=Origin(xyz=(0.0, -0.15, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="lower_rear_brace",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(half_width, -0.05, 0.885), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_hand_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(-half_width, -0.05, 0.885), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_hand_grip",
    )

    frame.visual(
        Cylinder(radius=0.017, length=0.05),
        origin=Origin(xyz=(half_width, -0.17, 0.025)),
        material=dark_rubber,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.05),
        origin=Origin(xyz=(-half_width, -0.17, 0.025)),
        material=dark_rubber,
        name="right_rear_tip",
    )

    frame.visual(
        Cylinder(radius=0.017, length=0.05),
        origin=Origin(xyz=(half_width, 0.20, 0.175)),
        material=aluminum,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.05),
        origin=Origin(xyz=(-half_width, 0.20, 0.175)),
        material=aluminum,
        name="right_caster_socket",
    )

    def add_caster_fork(part_name: str) -> None:
        fork = model.part(part_name)
        fork.inertial = Inertial.from_geometry(
            Box((0.05, 0.07, 0.12)),
            mass=0.35,
            origin=Origin(xyz=(0.0, -0.035, -0.055)),
        )
        fork.visual(
            Cylinder(radius=0.008, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, -0.032)),
            material=fork_steel,
            name="stem",
        )
        fork.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=fork_steel,
            name="swivel_race",
        )
        fork.visual(
            Box((0.04, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.004, -0.055)),
            material=fork_steel,
            name="saddle",
        )
        fork.visual(
            Box((0.007, 0.05, 0.012)),
            origin=Origin(xyz=(0.018, -0.034, -0.055)),
            material=fork_steel,
            name="left_neck",
        )
        fork.visual(
            Box((0.007, 0.05, 0.012)),
            origin=Origin(xyz=(-0.018, -0.034, -0.055)),
            material=fork_steel,
            name="right_neck",
        )
        fork.visual(
            Box((0.007, 0.016, 0.05)),
            origin=Origin(xyz=(0.018, -0.065, -0.086)),
            material=fork_steel,
            name="left_arm",
        )
        fork.visual(
            Box((0.007, 0.016, 0.05)),
            origin=Origin(xyz=(-0.018, -0.065, -0.086)),
            material=fork_steel,
            name="right_arm",
        )

    def add_caster_wheel(part_name: str) -> None:
        wheel = model.part(part_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.055, length=0.026),
            mass=0.45,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=0.055, length=0.026),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.037, length=0.022),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_gray,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(0.0125, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=fork_steel,
            name="left_cap",
        )
        wheel.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(-0.0125, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=fork_steel,
            name="right_cap",
        )

    add_caster_fork("left_caster_fork")
    add_caster_fork("right_caster_fork")
    add_caster_wheel("left_caster_wheel")
    add_caster_wheel("right_caster_wheel")

    model.articulation(
        "left_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="left_caster_fork",
        origin=Origin(xyz=(half_width, 0.20, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="right_caster_fork",
        origin=Origin(xyz=(-half_width, 0.20, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="left_caster_fork",
        child="left_caster_wheel",
        origin=Origin(xyz=(0.0, -0.068, -0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="right_caster_fork",
        child="right_caster_wheel",
        origin=Origin(xyz=(0.0, -0.068, -0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_wheel = object_model.get_part("left_caster_wheel")
    right_wheel = object_model.get_part("right_caster_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_wheel_spin")

    ctx.expect_contact(left_fork, left_wheel, name="left caster wheel is supported in its fork")
    ctx.expect_contact(right_fork, right_wheel, name="right caster wheel is supported in its fork")

    frame_aabb = ctx.part_world_aabb(frame)
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    if frame_aabb and left_wheel_aabb and right_wheel_aabb:
        frame_min, frame_max = frame_aabb
        left_min, left_max = left_wheel_aabb
        right_min, right_max = right_wheel_aabb
        ctx.check(
            "front wheels stay within a tight side silhouette",
            left_max[0] <= frame_max[0] + 0.002
            and right_min[0] >= frame_min[0] - 0.002
            and left_max[1] <= frame_max[1] + 0.002
            and right_max[1] <= frame_max[1] + 0.002,
            details=(
                f"frame_aabb={frame_aabb}, "
                f"left_wheel_aabb={left_wheel_aabb}, right_wheel_aabb={right_wheel_aabb}"
            ),
        )
    else:
        ctx.fail(
            "front wheels stay within a tight side silhouette",
            f"missing AABB(s): frame={frame_aabb}, left={left_wheel_aabb}, right={right_wheel_aabb}",
        )

    def _elem_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    left_grip_center = _elem_center("frame", "left_hand_grip")
    right_grip_center = _elem_center("frame", "right_hand_grip")
    left_tip_center = _elem_center("frame", "left_rear_tip")
    right_tip_center = _elem_center("frame", "right_rear_tip")
    if all(center is not None for center in (left_grip_center, right_grip_center, left_tip_center, right_tip_center)):
        ctx.check(
            "rear support pair sits under the handles",
            abs(left_grip_center[0] - left_tip_center[0]) <= 0.01
            and abs(right_grip_center[0] - right_tip_center[0]) <= 0.01
            and left_tip_center[2] < left_grip_center[2] - 0.75
            and right_tip_center[2] < right_grip_center[2] - 0.75,
            details=(
                f"left_grip={left_grip_center}, left_tip={left_tip_center}, "
                f"right_grip={right_grip_center}, right_tip={right_tip_center}"
            ),
        )
    else:
        ctx.fail(
            "rear support pair sits under the handles",
            (
                f"left_grip={left_grip_center}, right_grip={right_grip_center}, "
                f"left_tip={left_tip_center}, right_tip={right_tip_center}"
            ),
        )

    left_rest = ctx.part_world_position(left_wheel)
    right_rest = ctx.part_world_position(right_wheel)
    with ctx.pose({left_swivel: 1.2, right_swivel: -1.2}):
        left_turned = ctx.part_world_position(left_wheel)
        right_turned = ctx.part_world_position(right_wheel)
    ctx.check(
        "caster swivel joints turn the forks around their vertical stems",
        left_rest is not None
        and right_rest is not None
        and left_turned is not None
        and right_turned is not None
        and left_turned[0] > left_rest[0] + 0.04
        and right_turned[0] < right_rest[0] - 0.04
        and abs(left_turned[2] - left_rest[2]) <= 1e-6
        and abs(right_turned[2] - right_rest[2]) <= 1e-6,
        details=(
            f"left_rest={left_rest}, left_turned={left_turned}, "
            f"right_rest={right_rest}, right_turned={right_turned}"
        ),
    )

    wheel_rest = ctx.part_world_position(left_wheel)
    with ctx.pose({left_spin: pi / 2.0}):
        wheel_spun = ctx.part_world_position(left_wheel)
    ctx.check(
        "caster wheel spins in place about its axle",
        wheel_rest is not None
        and wheel_spun is not None
        and all(abs(wheel_rest[i] - wheel_spun[i]) <= 1e-6 for i in range(3)),
        details=f"wheel_rest={wheel_rest}, wheel_spun={wheel_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
