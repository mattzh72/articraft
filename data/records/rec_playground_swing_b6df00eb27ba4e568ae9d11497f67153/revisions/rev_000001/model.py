from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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
    model = ArticulatedObject(name="dual_seat_swing_set")

    frame_green = model.material("frame_green", rgba=(0.20, 0.34, 0.22, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    hanger_black = model.material("hanger_black", rgba=(0.08, 0.08, 0.09, 1.0))
    seat_black = model.material("seat_black", rgba=(0.11, 0.11, 0.12, 1.0))

    beam_length = 2.65
    beam_size = (0.12, beam_length, 0.10)
    beam_center_z = 2.25
    beam_bottom_z = beam_center_z - beam_size[2] * 0.5

    leg_top_z = beam_center_z - 0.01
    leg_foot_x = 0.72
    end_frame_y = 1.10
    leg_radius = 0.045
    leg_length = (leg_top_z**2 + leg_foot_x**2) ** 0.5
    leg_pitch = atan2(leg_foot_x, leg_top_z)

    left_seat_y = -0.60
    right_seat_y = 0.60
    hanger_half_spacing = 0.19
    pivot_z = beam_bottom_z - 0.045

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.60, beam_length, 2.35)),
        mass=125.0,
        origin=Origin(xyz=(0.0, 0.0, 1.175)),
    )
    frame.visual(
        Box(beam_size),
        origin=Origin(xyz=(0.0, 0.0, beam_center_z)),
        material=frame_green,
        name="top_beam",
    )

    for end_y in (-end_frame_y, end_frame_y):
        frame.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(xyz=(leg_foot_x * 0.5, end_y, leg_top_z * 0.5), rpy=(0.0, -leg_pitch, 0.0)),
            material=frame_green,
        )
        frame.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(xyz=(-leg_foot_x * 0.5, end_y, leg_top_z * 0.5), rpy=(0.0, leg_pitch, 0.0)),
            material=frame_green,
        )
        frame.visual(
            Cylinder(radius=0.030, length=0.88),
            origin=Origin(xyz=(0.0, end_y, 1.05), rpy=(0.0, pi * 0.5, 0.0)),
            material=frame_green,
        )

    for seat_center_y in (left_seat_y, right_seat_y):
        for pivot_y in (seat_center_y - hanger_half_spacing, seat_center_y + hanger_half_spacing):
            frame.visual(
                Box((0.008, 0.050, 0.090)),
                origin=Origin(xyz=(0.018, pivot_y, pivot_z)),
                material=bracket_steel,
            )
            frame.visual(
                Box((0.008, 0.050, 0.090)),
                origin=Origin(xyz=(-0.018, pivot_y, pivot_z)),
                material=bracket_steel,
            )

    def add_seat_assembly(name: str) -> None:
        assembly = model.part(name)
        assembly.inertial = Inertial.from_geometry(
            Box((0.24, 0.44, 1.72)),
            mass=7.0,
            origin=Origin(xyz=(0.0, 0.0, -0.82)),
        )

        for side_y in (-hanger_half_spacing, hanger_half_spacing):
            assembly.visual(
                Cylinder(radius=0.014, length=0.028),
                origin=Origin(xyz=(0.0, side_y, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
                material=bracket_steel,
            )
            assembly.visual(
                Box((0.014, 0.030, 1.52)),
                origin=Origin(xyz=(0.0, side_y, -0.76)),
                material=hanger_black,
            )
            assembly.visual(
                Box((0.036, 0.055, 0.085)),
                origin=Origin(xyz=(0.0, side_y, -1.515)),
                material=hanger_black,
            )

        assembly.visual(
            Box((0.17, 0.44, 0.035)),
            origin=Origin(xyz=(0.0, 0.0, -1.57)),
            material=seat_black,
            name="seat_panel",
        )
        assembly.visual(
            Box((0.060, 0.44, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -1.543)),
            material=seat_black,
        )

    add_seat_assembly("left_seat_assembly")
    add_seat_assembly("right_seat_assembly")

    model.articulation(
        "frame_to_left_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="left_seat_assembly",
        origin=Origin(xyz=(0.0, left_seat_y, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.5, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "frame_to_right_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="right_seat_assembly",
        origin=Origin(xyz=(0.0, right_seat_y, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.5, lower=-0.85, upper=0.85),
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
    left_seat = object_model.get_part("left_seat_assembly")
    right_seat = object_model.get_part("right_seat_assembly")
    left_joint = object_model.get_articulation("frame_to_left_swing")
    right_joint = object_model.get_articulation("frame_to_right_swing")

    def seat_center(part, *, joint_positions=None):
        if joint_positions is None:
            aabb = ctx.part_element_world_aabb(part, elem="seat_panel")
        else:
            with ctx.pose(joint_positions):
                aabb = ctx.part_element_world_aabb(part, elem="seat_panel")
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "left hinge axis follows the beam",
        abs(left_joint.axis[0]) < 1e-9 and abs(left_joint.axis[1] + 1.0) < 1e-9 and abs(left_joint.axis[2]) < 1e-9,
        details=f"axis={left_joint.axis}",
    )
    ctx.check(
        "right hinge axis follows the beam",
        abs(right_joint.axis[0]) < 1e-9 and abs(right_joint.axis[1] + 1.0) < 1e-9 and abs(right_joint.axis[2]) < 1e-9,
        details=f"axis={right_joint.axis}",
    )
    ctx.expect_origin_distance(
        left_seat,
        right_seat,
        axes="y",
        min_dist=1.0,
        max_dist=1.3,
        name="the two seats hang as separate side-by-side assemblies",
    )

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_gap(
            frame,
            left_seat,
            axis="z",
            positive_elem="top_beam",
            negative_elem="seat_panel",
            min_gap=1.45,
            name="left seat hangs well below the top beam",
        )
        ctx.expect_gap(
            frame,
            right_seat,
            axis="z",
            positive_elem="top_beam",
            negative_elem="seat_panel",
            min_gap=1.45,
            name="right seat hangs well below the top beam",
        )

    left_rest = seat_center(left_seat, joint_positions={left_joint: 0.0})
    left_forward = seat_center(left_seat, joint_positions={left_joint: 0.55})
    ctx.check(
        "left seat swings forward for positive rotation",
        left_rest is not None and left_forward is not None and left_forward[0] > left_rest[0] + 0.18,
        details=f"rest={left_rest}, swung={left_forward}",
    )

    mixed_left = seat_center(left_seat, joint_positions={left_joint: 0.50, right_joint: -0.45})
    mixed_right = seat_center(right_seat, joint_positions={left_joint: 0.50, right_joint: -0.45})
    ctx.check(
        "the two seats articulate independently",
        mixed_left is not None
        and mixed_right is not None
        and mixed_left[0] > 0.14
        and mixed_right[0] < -0.12,
        details=f"left={mixed_left}, right={mixed_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
