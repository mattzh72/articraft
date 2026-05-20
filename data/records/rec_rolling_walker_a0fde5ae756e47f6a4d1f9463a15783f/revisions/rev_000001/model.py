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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_walker")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    grip_black = model.material("grip_black", rgba=(0.13, 0.13, 0.14, 1.0))
    ferrule_gray = model.material("ferrule_gray", rgba=(0.36, 0.37, 0.39, 1.0))
    caster_gray = model.material("caster_gray", rgba=(0.66, 0.68, 0.71, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.58, 0.60, 0.63, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.38, 0.58, 0.90)),
        mass=6.8,
        origin=Origin(xyz=(-0.01, 0.0, 0.45)),
    )

    tube_radius = 0.0125
    frame_half_width = 0.255

    def side_points(side_y: float) -> list[tuple[float, float, float]]:
        return [
            (-0.165, side_y, 0.025),
            (-0.110, side_y, 0.360),
            (-0.025, side_y, 0.580),
            (0.045, side_y, 0.860),
            (0.165, side_y, 0.860),
            (0.140, side_y, 0.700),
            (0.130, side_y, 0.140),
        ]

    left_side = mesh_from_geometry(
        tube_from_spline_points(
            side_points(frame_half_width),
            radius=tube_radius,
            samples_per_segment=18,
            radial_segments=18,
        ),
        "walker_left_side_frame",
    )
    right_side = mesh_from_geometry(
        tube_from_spline_points(
            side_points(-frame_half_width),
            radius=tube_radius,
            samples_per_segment=18,
            radial_segments=18,
        ),
        "walker_right_side_frame",
    )

    frame.visual(left_side, material=aluminum, name="left_side_frame")
    frame.visual(right_side, material=aluminum, name="right_side_frame")

    crossbar_length = 0.500
    frame.visual(
        Cylinder(radius=tube_radius, length=crossbar_length),
        origin=Origin(xyz=(-0.025, 0.0, 0.580), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_crossbar",
    )
    frame.visual(
        Cylinder(radius=tube_radius, length=crossbar_length),
        origin=Origin(xyz=(0.140, 0.0, 0.700), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_crossbar",
    )

    grip_length = 0.120
    grip_radius = 0.018
    frame.visual(
        Cylinder(radius=grip_radius, length=grip_length),
        origin=Origin(xyz=(0.105, frame_half_width, 0.860), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=grip_radius, length=grip_length),
        origin=Origin(xyz=(0.105, -frame_half_width, 0.860), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_handle_grip",
    )

    tip_radius = 0.017
    tip_height = 0.050
    frame.visual(
        Cylinder(radius=tip_radius, length=tip_height),
        origin=Origin(xyz=(-0.165, frame_half_width, tip_height * 0.5)),
        material=ferrule_gray,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=tip_radius, length=tip_height),
        origin=Origin(xyz=(-0.165, -frame_half_width, tip_height * 0.5)),
        material=ferrule_gray,
        name="right_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.130, frame_half_width, 0.135)),
        material=aluminum,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.130, -frame_half_width, 0.135)),
        material=aluminum,
        name="right_caster_socket",
    )

    def add_caster(swivel_name: str, wheel_name: str, side_y: float) -> None:
        caster = model.part(swivel_name)
        caster.inertial = Inertial.from_geometry(
            Box((0.050, 0.050, 0.110)),
            mass=0.18,
            origin=Origin(xyz=(-0.020, 0.0, -0.055)),
        )
        caster.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=caster_gray,
            name="stem",
        )
        caster.visual(
            Box((0.040, 0.040, 0.012)),
            origin=Origin(xyz=(-0.015, 0.0, -0.030)),
            material=caster_gray,
            name="crown",
        )
        caster.visual(
            Box((0.026, 0.040, 0.014)),
            origin=Origin(xyz=(-0.022, 0.0, -0.033)),
            material=caster_gray,
            name="trail_block",
        )
        caster.visual(
            Box((0.010, 0.005, 0.068)),
            origin=Origin(xyz=(-0.035, 0.018, -0.072)),
            material=caster_gray,
            name="outer_blade",
        )
        caster.visual(
            Box((0.010, 0.005, 0.068)),
            origin=Origin(xyz=(-0.035, -0.018, -0.072)),
            material=caster_gray,
            name="inner_blade",
        )
        caster.visual(
            Box((0.010, 0.006, 0.012)),
            origin=Origin(xyz=(-0.035, 0.017, -0.096)),
            material=caster_gray,
            name="outer_axle_boss",
        )
        caster.visual(
            Box((0.010, 0.006, 0.012)),
            origin=Origin(xyz=(-0.035, -0.017, -0.096)),
            material=caster_gray,
            name="inner_axle_boss",
        )

        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.048, length=0.024),
            mass=0.22,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=0.048, length=0.024),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.031, length=0.018),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=hub_gray,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.028),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=caster_gray,
            name="bearing_core",
        )

        model.articulation(
            f"{swivel_name}_swivel",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(0.130, side_y, 0.140)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=5.0,
                lower=-pi,
                upper=pi,
            ),
        )
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(-0.035, 0.0, -0.092)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=25.0),
        )

    add_caster("left_caster", "left_front_wheel", frame_half_width)
    add_caster("right_caster", "right_front_wheel", -frame_half_width)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_front_wheel")
    right_wheel = object_model.get_part("right_front_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")

    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="left_handle_grip",
        negative_elem="left_rear_tip",
        min_gap=0.76,
        name="left handle sits well above rear support tip",
    )
    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="right_handle_grip",
        negative_elem="right_rear_tip",
        min_gap=0.76,
        name="right handle sits well above rear support tip",
    )

    left_handle_aabb = ctx.part_element_world_aabb(frame, elem="left_handle_grip")
    right_handle_aabb = ctx.part_element_world_aabb(frame, elem="right_handle_grip")
    left_tip_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_tip")
    right_tip_aabb = ctx.part_element_world_aabb(frame, elem="right_rear_tip")
    left_wheel_pos = ctx.part_world_position(left_wheel)
    right_wheel_pos = ctx.part_world_position(right_wheel)

    ctx.check(
        "upper frame sits slightly forward of left wheel center",
        left_handle_aabb is not None
        and left_wheel_pos is not None
        and ((left_handle_aabb[0][0] + left_handle_aabb[1][0]) * 0.5) > left_wheel_pos[0] + 0.008,
        details=f"left_handle_aabb={left_handle_aabb}, left_wheel_pos={left_wheel_pos}",
    )
    ctx.check(
        "upper frame sits slightly forward of right wheel center",
        right_handle_aabb is not None
        and right_wheel_pos is not None
        and ((right_handle_aabb[0][0] + right_handle_aabb[1][0]) * 0.5) > right_wheel_pos[0] + 0.008,
        details=f"right_handle_aabb={right_handle_aabb}, right_wheel_pos={right_wheel_pos}",
    )
    ctx.check(
        "rear support tips sit behind the front caster wheels",
        left_tip_aabb is not None
        and right_tip_aabb is not None
        and left_wheel_pos is not None
        and right_wheel_pos is not None
        and ((left_tip_aabb[0][0] + left_tip_aabb[1][0]) * 0.5) < left_wheel_pos[0] - 0.10
        and ((right_tip_aabb[0][0] + right_tip_aabb[1][0]) * 0.5) < right_wheel_pos[0] - 0.10,
        details=(
            f"left_tip_aabb={left_tip_aabb}, right_tip_aabb={right_tip_aabb}, "
            f"left_wheel_pos={left_wheel_pos}, right_wheel_pos={right_wheel_pos}"
        ),
    )

    left_rest = ctx.part_world_position(left_wheel)
    right_rest = ctx.part_world_position(right_wheel)
    with ctx.pose({left_swivel: 0.9, right_swivel: -0.9}):
        left_turned = ctx.part_world_position(left_wheel)
        right_turned = ctx.part_world_position(right_wheel)

    ctx.check(
        "left caster swivel swings the wheel around the stem",
        left_rest is not None
        and left_turned is not None
        and abs(left_turned[1] - left_rest[1]) > 0.02,
        details=f"left_rest={left_rest}, left_turned={left_turned}",
    )
    ctx.check(
        "right caster swivel swings the wheel around the stem",
        right_rest is not None
        and right_turned is not None
        and abs(right_turned[1] - right_rest[1]) > 0.02,
        details=f"right_rest={right_rest}, right_turned={right_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
