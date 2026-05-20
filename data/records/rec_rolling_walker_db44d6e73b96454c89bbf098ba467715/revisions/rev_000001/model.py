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


def _tube(name: str, points: list[tuple[float, float, float]], *, radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.84, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.64, 0.58, 0.94)),
        mass=6.8,
        origin=Origin(xyz=(0.0, -0.01, 0.47)),
    )

    side_x = 0.255
    front_y = 0.215
    rear_y = -0.225
    handle_z = 0.895

    left_side_path = [
        (side_x, front_y, 0.180),
        (side_x, front_y, 0.720),
        (side_x, 0.165, 0.835),
        (side_x, 0.045, handle_z),
        (side_x, -0.085, handle_z),
        (side_x, -0.180, 0.585),
        (side_x, rear_y, 0.030),
    ]
    right_side_path = [(-x, y, z) for x, y, z in left_side_path]

    frame.visual(_tube("left_side_frame", left_side_path, radius=0.016), material=aluminum, name="left_side_frame")
    frame.visual(_tube("right_side_frame", right_side_path, radius=0.016), material=aluminum, name="right_side_frame")

    frame.visual(
        _tube(
            "left_side_brace",
            [
                (side_x, front_y, 0.505),
                (side_x, 0.020, 0.470),
                (side_x, -0.195, 0.445),
            ],
            radius=0.013,
        ),
        material=aluminum,
        name="left_side_brace",
    )
    frame.visual(
        _tube(
            "right_side_brace",
            [
                (-side_x, front_y, 0.505),
                (-side_x, 0.020, 0.470),
                (-side_x, -0.195, 0.445),
            ],
            radius=0.013,
        ),
        material=aluminum,
        name="right_side_brace",
    )

    frame.visual(
        _tube(
            "upper_crossbar",
            [
                (-side_x, front_y, 0.690),
                (0.0, front_y - 0.020, 0.680),
                (side_x, front_y, 0.690),
            ],
            radius=0.015,
        ),
        material=aluminum,
        name="upper_crossbar",
    )
    frame.visual(
        _tube(
            "lower_crossbar",
            [
                (-side_x, -0.105, 0.455),
                (0.0, -0.130, 0.445),
                (side_x, -0.105, 0.455),
            ],
            radius=0.015,
        ),
        material=aluminum,
        name="lower_crossbar",
    )

    for sign in (-1.0, 1.0):
        x = sign * side_x
        frame.visual(
            Box((0.072, 0.054, 0.052)),
            origin=Origin(xyz=(x, front_y, 0.186)),
            material=charcoal,
            name=f"{'left' if sign > 0 else 'right'}_caster_socket",
        )
        frame.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(x, rear_y, 0.015)),
            material=grip_black,
            name=f"{'left' if sign > 0 else 'right'}_rear_tip",
        )
        frame.visual(
            Cylinder(radius=0.019, length=0.140),
            origin=Origin(xyz=(x, -0.030, handle_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=grip_black,
            name=f"{'left' if sign > 0 else 'right'}_hand_grip",
        )

    def add_caster(side_name: str, x: float) -> None:
        caster = model.part(f"{side_name}_caster")
        caster.inertial = Inertial.from_geometry(
            Box((0.090, 0.115, 0.160)),
            mass=0.60,
            origin=Origin(xyz=(0.0, -0.030, -0.080)),
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=charcoal,
            name="stem",
        )
        caster.visual(
            Box((0.058, 0.036, 0.022)),
            origin=Origin(xyz=(0.0, 0.026, -0.036)),
            material=charcoal,
            name="crown",
        )
        caster.visual(
            Box((0.066, 0.026, 0.048)),
            origin=Origin(xyz=(0.0, 0.020, -0.070)),
            material=charcoal,
            name="fork_neck",
        )
        caster.visual(
            Box((0.018, 0.082, 0.018)),
            origin=Origin(xyz=(0.030, -0.005, -0.094)),
            material=charcoal,
            name="right_fork_arm" if side_name == "left" else "left_fork_arm",
        )
        caster.visual(
            Box((0.018, 0.082, 0.018)),
            origin=Origin(xyz=(-0.030, -0.005, -0.094)),
            material=charcoal,
            name="left_fork_arm" if side_name == "left" else "right_fork_arm",
        )
        caster.visual(
            Box((0.024, 0.022, 0.038)),
            origin=Origin(xyz=(0.030, -0.056, -0.098)),
            material=charcoal,
            name="right_axle_block" if side_name == "left" else "left_axle_block",
        )
        caster.visual(
            Box((0.024, 0.022, 0.038)),
            origin=Origin(xyz=(-0.030, -0.056, -0.098)),
            material=charcoal,
            name="left_axle_block" if side_name == "left" else "right_axle_block",
        )

        wheel = model.part(f"{side_name}_front_wheel")
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.062, length=0.030),
            mass=0.42,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=0.062, length=0.030),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=grip_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.044, length=0.034),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.021, length=0.040),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=charcoal,
            name="hub",
        )

        model.articulation(
            f"{side_name}_caster_swivel",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(x, front_y, 0.160)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=5.0,
                lower=-pi,
                upper=pi,
            ),
        )
        model.articulation(
            f"{side_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.056, -0.098)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=20.0,
            ),
        )

    add_caster("left", side_x)
    add_caster("right", -side_x)

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
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        min_corner, max_corner = frame_aabb
        ctx.check(
            "walker frame has realistic overall size",
            (max_corner[0] - min_corner[0]) >= 0.48
            and (max_corner[1] - min_corner[1]) >= 0.42
            and 0.84 <= (max_corner[2] - min_corner[2]) <= 0.98,
            details=f"aabb={frame_aabb}",
        )

    ctx.check(
        "caster joints use the intended mechanisms",
        left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0)
        and left_spin.joint_type == ArticulationType.CONTINUOUS
        and right_spin.joint_type == ArticulationType.CONTINUOUS,
        details=(
            f"left_swivel_axis={left_swivel.axis}, right_swivel_axis={right_swivel.axis}, "
            f"left_spin_axis={left_spin.axis}, right_spin_axis={right_spin.axis}, "
            f"left_spin_type={left_spin.joint_type}, right_spin_type={right_spin.joint_type}"
        ),
    )

    ctx.expect_gap(
        frame,
        "left_front_wheel",
        axis="z",
        positive_elem="left_caster_socket",
        negative_elem="tire",
        min_gap=0.020,
        max_gap=0.080,
        name="left tire stays below the socket block",
    )
    ctx.expect_gap(
        frame,
        "right_front_wheel",
        axis="z",
        positive_elem="right_caster_socket",
        negative_elem="tire",
        min_gap=0.020,
        max_gap=0.080,
        name="right tire stays below the socket block",
    )

    left_tip_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_tip")
    right_tip_aabb = ctx.part_element_world_aabb(frame, elem="right_rear_tip")
    left_wheel_aabb = ctx.part_world_aabb("left_front_wheel")
    right_wheel_aabb = ctx.part_world_aabb("right_front_wheel")
    if left_tip_aabb is not None and right_tip_aabb is not None and left_wheel_aabb is not None and right_wheel_aabb is not None:
        ctx.check(
            "front casters and rear tips share a ground plane",
            abs(left_tip_aabb[0][2] - left_wheel_aabb[0][2]) <= 0.005
            and abs(right_tip_aabb[0][2] - right_wheel_aabb[0][2]) <= 0.005,
            details=(
                f"left_tip_z={left_tip_aabb[0][2]}, left_wheel_z={left_wheel_aabb[0][2]}, "
                f"right_tip_z={right_tip_aabb[0][2]}, right_wheel_z={right_wheel_aabb[0][2]}"
            ),
        )

    with ctx.pose({left_swivel: 1.0, right_swivel: -1.0}):
        ctx.expect_gap(
            frame,
            "left_front_wheel",
            axis="z",
            positive_elem="left_caster_socket",
            negative_elem="tire",
            min_gap=0.020,
            max_gap=0.080,
            name="turned left caster still clears the socket",
        )
        ctx.expect_gap(
            frame,
            "right_front_wheel",
            axis="z",
            positive_elem="right_caster_socket",
            negative_elem="tire",
            min_gap=0.020,
            max_gap=0.080,
            name="turned right caster still clears the socket",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
