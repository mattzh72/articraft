from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="board_seat_swing")

    beam_length = 1.05
    beam_depth = 0.08
    beam_height = 0.09

    seat_width = 0.34
    seat_depth = 0.12
    seat_thickness = 0.028

    hanger_spacing = 0.30
    hanger_width = 0.022
    hanger_thickness = 0.012
    hanger_length = 0.326

    eye_radius = 0.014
    eye_length = 0.03
    beam_to_swing_axis_z = -beam_height / 2.0
    eye_center_z = -0.014

    seat_top_z = -0.34
    seat_center_z = seat_top_z - seat_thickness / 2.0
    hanger_center_z = (eye_center_z + seat_top_z) / 2.0

    guard_bracket_width = 0.022
    guard_bracket_depth = 0.028
    guard_bracket_height = 0.042
    guard_pivot_y = 0.056
    guard_pivot_z = -0.336

    wood_beam = model.material("beam_wood", rgba=(0.51, 0.34, 0.20, 1.0))
    wood_seat = model.material("seat_wood", rgba=(0.42, 0.27, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.45, 0.47, 0.50, 1.0))
    coated_bar = model.material("coated_bar", rgba=(0.74, 0.18, 0.14, 1.0))

    beam = model.part("top_beam")
    beam.visual(
        Box((beam_length, beam_depth, beam_height)),
        material=wood_beam,
        name="beam_shell",
    )

    swing = model.part("swing_assembly")
    left_x = -hanger_spacing / 2.0
    right_x = hanger_spacing / 2.0

    swing.visual(
        Cylinder(radius=eye_radius, length=eye_length),
        origin=Origin(xyz=(left_x, 0.0, eye_center_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_hanger_eye",
    )
    swing.visual(
        Cylinder(radius=eye_radius, length=eye_length),
        origin=Origin(xyz=(right_x, 0.0, eye_center_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_hanger_eye",
    )
    swing.visual(
        Box((hanger_width, hanger_thickness, hanger_length)),
        origin=Origin(xyz=(left_x, 0.0, hanger_center_z)),
        material=steel,
        name="left_hanger_link",
    )
    swing.visual(
        Box((hanger_width, hanger_thickness, hanger_length)),
        origin=Origin(xyz=(right_x, 0.0, hanger_center_z)),
        material=steel,
        name="right_hanger_link",
    )
    swing.visual(
        Box((seat_width, seat_depth, seat_thickness)),
        origin=Origin(xyz=(0.0, 0.0, seat_center_z)),
        material=wood_seat,
        name="seat_board",
    )
    swing.visual(
        Box((guard_bracket_width, guard_bracket_depth, guard_bracket_height)),
        origin=Origin(
            xyz=(
                -seat_width / 2.0 + guard_bracket_width / 2.0,
                guard_pivot_y - guard_bracket_depth / 2.0,
                guard_pivot_z - guard_bracket_height / 2.0,
            )
        ),
        material=steel,
        name="left_guard_bracket",
    )
    swing.visual(
        Box((guard_bracket_width, guard_bracket_depth, guard_bracket_height)),
        origin=Origin(
            xyz=(
                seat_width / 2.0 - guard_bracket_width / 2.0,
                guard_pivot_y - guard_bracket_depth / 2.0,
                guard_pivot_z - guard_bracket_height / 2.0,
            )
        ),
        material=steel,
        name="right_guard_bracket",
    )

    guard_frame = tube_from_spline_points(
        [
            (-0.12, 0.014, -0.010),
            (-0.118, 0.032, -0.020),
            (-0.114, 0.060, -0.050),
            (-0.104, 0.080, -0.074),
            (0.104, 0.080, -0.074),
            (0.114, 0.060, -0.050),
            (0.118, 0.032, -0.020),
            (0.12, 0.014, -0.010),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    guard_frame.merge(
        BoxGeometry((0.03, 0.03, 0.05)).translate(-0.13, 0.015, -0.03)
    )
    guard_frame.merge(
        BoxGeometry((0.03, 0.03, 0.05)).translate(0.13, 0.015, -0.03)
    )

    safety_bar = model.part("safety_bar")
    safety_bar.visual(
        mesh_from_geometry(guard_frame, "safety_bar_frame"),
        material=coated_bar,
        name="guard_frame",
    )

    model.articulation(
        "beam_to_swing",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, beam_to_swing_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=-0.6,
            upper=0.6,
        ),
    )
    model.articulation(
        "swing_to_safety_bar",
        ArticulationType.REVOLUTE,
        parent=swing,
        child=safety_bar,
        origin=Origin(xyz=(0.0, guard_pivot_y, guard_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("top_beam")
    swing = object_model.get_part("swing_assembly")
    safety_bar = object_model.get_part("safety_bar")
    swing_joint = object_model.get_articulation("beam_to_swing")
    guard_joint = object_model.get_articulation("swing_to_safety_bar")

    swing_limits = swing_joint.motion_limits
    guard_limits = guard_joint.motion_limits

    ctx.check(
        "swing joint uses beam-width hinge axis",
        swing_joint.axis == (1.0, 0.0, 0.0)
        and swing_limits is not None
        and swing_limits.lower == -0.6
        and swing_limits.upper == 0.6,
        details=f"axis={swing_joint.axis}, limits={swing_limits}",
    )
    ctx.check(
        "safety bar opens upward on side pivots",
        guard_joint.axis == (1.0, 0.0, 0.0)
        and guard_limits is not None
        and guard_limits.lower == 0.0
        and guard_limits.upper >= 1.2,
        details=f"axis={guard_joint.axis}, limits={guard_limits}",
    )

    with ctx.pose({swing_joint: 0.0, guard_joint: 0.0}):
        ctx.expect_gap(
            beam,
            swing,
            axis="z",
            min_gap=0.24,
            max_gap=0.34,
            negative_elem="seat_board",
            name="seat board hangs clearly below the beam",
        )
        ctx.expect_within(
            safety_bar,
            swing,
            axes="x",
            outer_elem="seat_board",
            name="safety bar stays within the seat width",
        )

        seat_aabb = ctx.part_element_world_aabb(swing, elem="seat_board")
        guard_aabb = ctx.part_world_aabb(safety_bar)
        bar_is_ahead = (
            seat_aabb is not None
            and guard_aabb is not None
            and guard_aabb[1][1] > seat_aabb[1][1] + 0.035
        )
        ctx.check(
            "closed safety bar sits ahead of the seat front",
            bar_is_ahead,
            details=f"seat_aabb={seat_aabb}, guard_aabb={guard_aabb}",
        )

        rest_seat_aabb = seat_aabb
        rest_guard_aabb = guard_aabb

    with ctx.pose({swing_joint: 0.35, guard_joint: 0.0}):
        swung_seat_aabb = ctx.part_element_world_aabb(swing, elem="seat_board")
        swings_forward = (
            rest_seat_aabb is not None
            and swung_seat_aabb is not None
            and (swung_seat_aabb[0][1] + swung_seat_aabb[1][1]) / 2.0
            > (rest_seat_aabb[0][1] + rest_seat_aabb[1][1]) / 2.0 + 0.09
        )
        ctx.check(
            "positive swing pose moves the seat forward",
            swings_forward,
            details=f"rest={rest_seat_aabb}, swung={swung_seat_aabb}",
        )

    with ctx.pose({swing_joint: 0.0, guard_joint: 1.05}):
        opened_guard_aabb = ctx.part_world_aabb(safety_bar)
        opens_upward = (
            rest_guard_aabb is not None
            and opened_guard_aabb is not None
            and opened_guard_aabb[1][2] > rest_guard_aabb[1][2] + 0.07
        )
        ctx.check(
            "safety bar rotates upward when opened",
            opens_upward,
            details=f"rest={rest_guard_aabb}, opened={opened_guard_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
