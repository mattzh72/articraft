from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    frame_steel = model.material("frame_steel", rgba=(0.18, 0.18, 0.19, 1.0))
    gate_steel = model.material("gate_steel", rgba=(0.31, 0.33, 0.35, 1.0))

    frame = model.part("frame")

    post_height = 2.20
    track_length = 2.90
    track_center_x = 1.50

    frame.visual(
        Box((0.10, 0.10, post_height)),
        origin=Origin(xyz=(0.05, 0.0, post_height / 2.0)),
        material=frame_steel,
        name="left_post",
    )
    frame.visual(
        Box((0.08, 0.08, post_height)),
        origin=Origin(xyz=(1.76, 0.10, post_height / 2.0)),
        material=frame_steel,
        name="receiver_post",
    )
    frame.visual(
        Box((0.10, 0.10, post_height)),
        origin=Origin(xyz=(2.90, 0.0, post_height / 2.0)),
        material=frame_steel,
        name="stack_post",
    )

    frame.visual(
        Box((track_length, 0.14, 0.01)),
        origin=Origin(xyz=(track_center_x, 0.0, 2.12)),
        material=frame_steel,
        name="track_top",
    )
    frame.visual(
        Box((track_length, 0.01, 0.08)),
        origin=Origin(xyz=(track_center_x, -0.065, 2.075)),
        material=frame_steel,
        name="guide_web_neg_y",
    )
    frame.visual(
        Box((track_length, 0.01, 0.08)),
        origin=Origin(xyz=(track_center_x, 0.065, 2.075)),
        material=frame_steel,
        name="guide_web_pos_y",
    )
    frame.visual(
        Box((track_length, 0.04, 0.01)),
        origin=Origin(xyz=(track_center_x, -0.035, 2.035)),
        material=frame_steel,
        name="guide_lip_neg_y",
    )
    frame.visual(
        Box((track_length, 0.04, 0.01)),
        origin=Origin(xyz=(track_center_x, 0.035, 2.035)),
        material=frame_steel,
        name="guide_lip_pos_y",
    )
    frame.inertial = Inertial.from_geometry(
        Box((3.00, 0.30, post_height)),
        mass=85.0,
        origin=Origin(xyz=(1.50, 0.03, post_height / 2.0)),
    )

    gate_leaf = model.part("gate_leaf")

    gate_width = 1.52
    gate_height = 1.78

    gate_leaf.visual(
        Box((0.05, 0.04, gate_height)),
        origin=Origin(xyz=(0.025, 0.0, gate_height / 2.0)),
        material=gate_steel,
        name="outer_left_stile",
    )
    gate_leaf.visual(
        Box((0.05, 0.04, gate_height)),
        origin=Origin(xyz=(gate_width - 0.025, 0.0, gate_height / 2.0)),
        material=gate_steel,
        name="outer_right_stile",
    )
    gate_leaf.visual(
        Box((gate_width, 0.04, 0.05)),
        origin=Origin(xyz=(gate_width / 2.0, 0.0, 0.025)),
        material=gate_steel,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((gate_width, 0.04, 0.05)),
        origin=Origin(xyz=(gate_width / 2.0, 0.0, gate_height - 0.025)),
        material=gate_steel,
        name="top_rail",
    )

    bar_height = gate_height - 0.10
    bar_centers_x = (0.17, 0.33, 0.49, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45)
    for index, x_pos in enumerate(bar_centers_x):
        gate_leaf.visual(
            Box((0.018, 0.014, bar_height)),
            origin=Origin(xyz=(x_pos, 0.0, 0.05 + bar_height / 2.0)),
            material=gate_steel,
            name=f"bar_{index}",
        )

    trolley_z = 2.099
    hanger_z = 1.9315
    trolley_x_positions = (0.24, 1.28)
    for name_prefix, x_pos in zip(("left", "right"), trolley_x_positions):
        gate_leaf.visual(
            Box((0.09, 0.07, 0.032)),
            origin=Origin(xyz=(x_pos, 0.0, trolley_z)),
            material=gate_steel,
            name=f"{name_prefix}_trolley",
        )
        gate_leaf.visual(
            Box((0.022, 0.014, 0.303)),
            origin=Origin(xyz=(x_pos, 0.0, hanger_z)),
            material=gate_steel,
            name=f"{name_prefix}_hanger",
        )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((gate_width, 0.08, 2.10)),
        mass=34.0,
        origin=Origin(xyz=(gate_width / 2.0, 0.0, 1.05)),
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=1.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")
    upper = 1.08
    if slide.motion_limits is not None and slide.motion_limits.upper is not None:
        upper = slide.motion_limits.upper

    ctx.expect_gap(
        frame,
        gate_leaf,
        axis="y",
        positive_elem="guide_web_pos_y",
        negative_elem="left_trolley",
        min_gap=0.02,
        max_gap=0.03,
        name="left trolley clears positive guide wall at rest",
    )
    ctx.expect_gap(
        gate_leaf,
        frame,
        axis="y",
        positive_elem="left_trolley",
        negative_elem="guide_web_neg_y",
        min_gap=0.02,
        max_gap=0.03,
        name="left trolley clears negative guide wall at rest",
    )
    ctx.expect_contact(
        gate_leaf,
        frame,
        elem_a="left_trolley",
        elem_b="track_top",
        name="left trolley bears on the top track at rest",
    )
    ctx.expect_gap(
        gate_leaf,
        frame,
        axis="z",
        positive_elem="left_trolley",
        negative_elem="guide_lip_neg_y",
        min_gap=0.04,
        max_gap=0.05,
        name="left trolley stays above lower retaining lip at rest",
    )
    ctx.expect_overlap(
        gate_leaf,
        frame,
        axes="x",
        elem_a="right_trolley",
        elem_b="track_top",
        min_overlap=0.08,
        name="right trolley remains captured by the fixed track at rest",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: upper}):
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="y",
            positive_elem="guide_web_pos_y",
            negative_elem="right_trolley",
            min_gap=0.02,
            max_gap=0.03,
            name="right trolley clears positive guide wall when open",
        )
        ctx.expect_gap(
            gate_leaf,
            frame,
            axis="y",
            positive_elem="right_trolley",
            negative_elem="guide_web_neg_y",
            min_gap=0.02,
            max_gap=0.03,
            name="right trolley clears negative guide wall when open",
        )
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="x",
            elem_a="left_trolley",
            elem_b="track_top",
            min_overlap=0.08,
            name="left trolley remains captured by the fixed track when open",
        )
        ctx.expect_contact(
            gate_leaf,
            frame,
            elem_a="right_trolley",
            elem_b="track_top",
            name="right trolley bears on the top track when open",
        )
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="x",
            elem_a="right_trolley",
            elem_b="track_top",
            min_overlap=0.08,
            name="right trolley remains captured by the fixed track when open",
        )
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="y",
            positive_elem="receiver_post",
            negative_elem="outer_right_stile",
            min_gap=0.02,
            max_gap=0.06,
            name="moving gate clears the receiver post offset plane when open",
        )
        extended_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf slides rightward along the track",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, open={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
