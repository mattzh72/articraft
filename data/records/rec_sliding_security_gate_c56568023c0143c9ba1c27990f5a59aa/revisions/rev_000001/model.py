from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    steel_dark = model.material("steel_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.34, 0.36, 0.38, 1.0))
    hardware = model.material("hardware", rgba=(0.56, 0.58, 0.60, 1.0))

    gate_width = 2.20
    gate_height = 1.65
    gate_depth = 0.055
    frame_thickness = 0.08
    slide_travel = 1.00

    post_width = 0.14
    post_depth = 0.10
    post_height = 2.12

    track_wall = 0.012
    track_outer_depth = 0.104
    track_inner_depth = track_outer_depth - 2.0 * track_wall
    track_web_height = 0.12
    track_roof_height = 0.016
    track_cavity_bottom = 1.74
    track_length = gate_width + slide_travel + 0.29
    track_x_min = -post_width
    track_x_max = track_x_min + track_length
    track_center_x = 0.5 * (track_x_min + track_x_max)

    support = model.part("support_frame")
    support.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(-0.5 * post_width, 0.0, 0.5 * post_height)),
        material=steel_dark,
        name="latch_post",
    )
    support.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(track_x_max - 0.5 * post_width, 0.0, 0.5 * post_height)),
        material=steel_dark,
        name="rear_post",
    )
    support.visual(
        Box((track_length, track_outer_depth, track_roof_height)),
        origin=Origin(
            xyz=(track_center_x, 0.0, track_cavity_bottom + track_web_height + 0.5 * track_roof_height)
        ),
        material=steel_dark,
        name="track_roof",
    )
    support.visual(
        Box((track_length, track_wall, track_web_height)),
        origin=Origin(
            xyz=(
                track_center_x,
                -0.5 * track_outer_depth + 0.5 * track_wall,
                track_cavity_bottom + 0.5 * track_web_height,
            )
        ),
        material=steel_dark,
        name="track_left_web",
    )
    support.visual(
        Box((track_length, track_wall, track_web_height)),
        origin=Origin(
            xyz=(
                track_center_x,
                0.5 * track_outer_depth - 0.5 * track_wall,
                track_cavity_bottom + 0.5 * track_web_height,
            )
        ),
        material=steel_dark,
        name="track_right_web",
    )
    support.inertial = Inertial.from_geometry(
        Box((track_x_max + post_width, 0.16, post_height)),
        mass=180.0,
        origin=Origin(xyz=(0.5 * (track_x_max - post_width), 0.0, 0.5 * post_height)),
    )

    gate = model.part("gate_leaf")
    gate.visual(
        Box((frame_thickness, gate_depth, gate_height)),
        origin=Origin(xyz=(0.5 * frame_thickness, 0.0, 0.5 * gate_height)),
        material=steel_dark,
        name="leading_stile",
    )
    gate.visual(
        Box((frame_thickness, gate_depth, gate_height)),
        origin=Origin(xyz=(gate_width - 0.5 * frame_thickness, 0.0, 0.5 * gate_height)),
        material=steel_dark,
        name="trailing_stile",
    )
    gate.visual(
        Box((gate_width - 2.0 * frame_thickness, gate_depth, frame_thickness)),
        origin=Origin(
            xyz=(0.5 * gate_width, 0.0, 0.5 * frame_thickness)
        ),
        material=steel_dark,
        name="bottom_rail",
    )
    gate.visual(
        Box((gate_width - 2.0 * frame_thickness, gate_depth, frame_thickness)),
        origin=Origin(
            xyz=(0.5 * gate_width, 0.0, gate_height - 0.5 * frame_thickness)
        ),
        material=steel_dark,
        name="top_rail",
    )

    inner_width = gate_width - 2.0 * frame_thickness
    picket_width = 0.032
    picket_depth = 0.020
    picket_height = gate_height - 2.0 * frame_thickness
    picket_count = 7
    usable_width = inner_width - picket_width
    for i in range(picket_count):
        x = frame_thickness + picket_width / 2.0 + usable_width * i / (picket_count - 1)
        gate.visual(
            Box((picket_width, picket_depth, picket_height)),
            origin=Origin(xyz=(x, 0.0, 0.5 * gate_height)),
            material=steel_mid,
            name=f"picket_{i+1}",
        )

    brace_length = math.hypot(inner_width, picket_height)
    brace_angle = -math.atan2(picket_height, inner_width)
    gate.visual(
        Box((brace_length, 0.022, 0.05)),
        origin=Origin(xyz=(0.5 * gate_width, 0.0, 0.5 * gate_height), rpy=(0.0, brace_angle, 0.0)),
        material=steel_mid,
        name="diagonal_brace",
    )

    carriage_length = 0.13
    carriage_depth = 0.026
    carriage_height = 0.024
    carriage_z = 1.825
    strap_height = carriage_z - 0.5 * carriage_height - gate_height
    strap_positions = (0.42, 1.68)
    for name, x in zip(("front", "rear"), strap_positions):
        gate.visual(
            Box((0.018, 0.012, strap_height)),
            origin=Origin(xyz=(x, 0.0, gate_height + 0.5 * strap_height)),
            material=hardware,
            name=f"{name}_hanger",
        )
        gate.visual(
            Box((carriage_length, carriage_depth, carriage_height)),
            origin=Origin(xyz=(x, 0.0, carriage_z)),
            material=hardware,
            name=f"{name}_carriage",
        )

    gate.inertial = Inertial.from_geometry(
        Box((gate_width, 0.10, track_cavity_bottom + track_web_height + track_roof_height)),
        mass=85.0,
        origin=Origin(
            xyz=(
                0.5 * gate_width,
                0.0,
                0.5 * (track_cavity_bottom + track_web_height + track_roof_height),
            )
        ),
    )

    model.articulation(
        "support_to_gate",
        ArticulationType.PRISMATIC,
        parent=support,
        child=gate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.45,
            lower=0.0,
            upper=slide_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("support_to_gate")

    for carriage in ("front_carriage", "rear_carriage"):
        ctx.expect_gap(
            support,
            gate,
            axis="y",
            positive_elem="track_right_web",
            negative_elem=carriage,
            min_gap=0.001,
            max_gap=0.030,
            name=f"{carriage} clears right track web at rest",
        )
        ctx.expect_gap(
            gate,
            support,
            axis="y",
            positive_elem=carriage,
            negative_elem="track_left_web",
            min_gap=0.001,
            max_gap=0.030,
            name=f"{carriage} clears left track web at rest",
        )
        ctx.expect_gap(
            support,
            gate,
            axis="z",
            positive_elem="track_roof",
            negative_elem=carriage,
            min_gap=0.020,
            max_gap=0.060,
            name=f"{carriage} stays under track roof at rest",
        )
        ctx.expect_overlap(
            support,
            gate,
            axes="x",
            elem_a="track_roof",
            elem_b=carriage,
            min_overlap=0.08,
            name=f"{carriage} remains inside track length at rest",
        )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: slide.motion_limits.upper}):
        for carriage in ("front_carriage", "rear_carriage"):
            ctx.expect_gap(
                support,
                gate,
                axis="y",
                positive_elem="track_right_web",
                negative_elem=carriage,
                min_gap=0.001,
                max_gap=0.030,
                name=f"{carriage} clears right track web when open",
            )
            ctx.expect_gap(
                gate,
                support,
                axis="y",
                positive_elem=carriage,
                negative_elem="track_left_web",
                min_gap=0.001,
                max_gap=0.030,
                name=f"{carriage} clears left track web when open",
            )
            ctx.expect_gap(
                support,
                gate,
                axis="z",
                positive_elem="track_roof",
                negative_elem=carriage,
                min_gap=0.020,
                max_gap=0.060,
                name=f"{carriage} stays under track roof when open",
            )
            ctx.expect_overlap(
                support,
                gate,
                axes="x",
                elem_a="track_roof",
                elem_b=carriage,
                min_overlap=0.08,
                name=f"{carriage} remains inside track length when open",
            )
        open_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate opens by translating to the right",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] > rest_pos[0] + 0.95,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
