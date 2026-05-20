from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

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

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.82, 0.70, 0.16, 1.0))

    track_start = -2.30
    track_end = 2.40
    track_length = track_end - track_start
    track_center = 0.5 * (track_start + track_end)

    gate_width = 2.20
    gate_height = 2.00
    gate_depth = 0.030
    gate_travel = 2.00

    top_channel_z = 2.165
    bottom_channel_z = 0.035
    rail_y = 0.030

    fixed_frame = model.part("fixed_frame")
    fixed_frame.inertial = Inertial.from_geometry(
        Box((track_length, 0.22, 2.45)),
        mass=240.0,
        origin=Origin(xyz=(track_center, 0.0, 1.225)),
    )

    fixed_frame.visual(
        Box((track_length, 0.18, 0.14)),
        origin=Origin(xyz=(track_center, -0.12, 2.29)),
        material=powder_black,
        name="header_beam",
    )
    fixed_frame.visual(
        Box((track_length, 0.09, 0.015)),
        origin=Origin(xyz=(track_center, 0.0, 2.218)),
        material=galvanized,
        name="top_channel_cap",
    )
    fixed_frame.visual(
        Box((track_length, 0.010, 0.100)),
        origin=Origin(xyz=(track_center, rail_y, top_channel_z)),
        material=galvanized,
        name="top_channel_front",
    )
    fixed_frame.visual(
        Box((track_length, 0.010, 0.100)),
        origin=Origin(xyz=(track_center, -rail_y, top_channel_z)),
        material=galvanized,
        name="top_channel_rear",
    )
    fixed_frame.visual(
        Box((track_length, 0.09, 0.015)),
        origin=Origin(xyz=(track_center, 0.0, 0.0075)),
        material=galvanized,
        name="bottom_channel_base",
    )
    fixed_frame.visual(
        Box((track_length, 0.010, 0.055)),
        origin=Origin(xyz=(track_center, rail_y, bottom_channel_z)),
        material=galvanized,
        name="bottom_channel_front",
    )
    fixed_frame.visual(
        Box((track_length, 0.010, 0.055)),
        origin=Origin(xyz=(track_center, -rail_y, bottom_channel_z)),
        material=galvanized,
        name="bottom_channel_rear",
    )
    fixed_frame.visual(
        Box((track_length, 0.06, 0.12)),
        origin=Origin(xyz=(track_center, -0.10, 0.060)),
        material=powder_black,
        name="base_beam",
    )
    fixed_frame.visual(
        Box((0.12, 0.14, 2.36)),
        origin=Origin(xyz=(-2.24, 0.0, 1.18)),
        material=powder_black,
        name="support_post",
    )
    fixed_frame.visual(
        Box((0.12, 0.08, 2.18)),
        origin=Origin(xyz=(2.29, 0.0, 1.09)),
        material=powder_black,
        name="receiver_post",
    )
    fixed_frame.visual(
        Box((0.30, 0.10, 0.12)),
        origin=Origin(xyz=(-2.05, 0.0, 2.17)),
        material=powder_black,
        name="left_header_bracket",
    )
    fixed_frame.visual(
        Box((0.10, 0.018, 0.10)),
        origin=Origin(xyz=(2.21, rail_y + 0.010, 2.17)),
        material=powder_black,
        name="right_stop_tab_front",
    )
    fixed_frame.visual(
        Box((0.10, 0.018, 0.10)),
        origin=Origin(xyz=(2.21, -rail_y - 0.010, 2.17)),
        material=powder_black,
        name="right_stop_tab_rear",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.inertial = Inertial.from_geometry(
        Box((gate_width, 0.06, 2.18)),
        mass=155.0,
        origin=Origin(xyz=(0.5 * gate_width, 0.0, 1.09)),
    )

    gate_leaf.visual(
        Box((0.060, gate_depth, gate_height)),
        origin=Origin(xyz=(0.030, 0.0, 1.060)),
        material=powder_black,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.060, gate_depth, gate_height)),
        origin=Origin(xyz=(gate_width - 0.030, 0.0, 1.060)),
        material=powder_black,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((gate_width, gate_depth, 0.060)),
        origin=Origin(xyz=(0.5 * gate_width, 0.0, 2.090)),
        material=powder_black,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((gate_width, gate_depth, 0.060)),
        origin=Origin(xyz=(0.5 * gate_width, 0.0, 0.090)),
        material=powder_black,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((gate_width - 0.24, 0.024, 0.050)),
        origin=Origin(xyz=(0.5 * gate_width, 0.0, 1.10)),
        material=powder_black,
        name="mid_rail",
    )

    for index, x_pos in enumerate((0.28, 0.54, 0.80, 1.06, 1.32, 1.58, 1.84)):
        gate_leaf.visual(
            Box((0.024, 0.024, 1.96)),
            origin=Origin(xyz=(x_pos, 0.0, 1.09)),
            material=galvanized,
            name=f"picket_{index + 1}",
        )

    brace_dx = 1.78
    brace_dz = 1.48
    brace_len = sqrt(brace_dx * brace_dx + brace_dz * brace_dz)
    gate_leaf.visual(
        Box((0.028, 0.022, brace_len)),
        origin=Origin(
            xyz=(1.10, 0.0, 1.05),
            rpy=(0.0, atan2(brace_dx, brace_dz), 0.0),
        ),
        material=galvanized,
        name="diagonal_brace",
    )

    gate_leaf.visual(
        Box((0.090, 0.018, 0.131)),
        origin=Origin(xyz=(0.18, 0.0, 2.145)),
        material=safety_yellow,
        name="hanger_left",
    )
    gate_leaf.visual(
        Box((0.090, 0.018, 0.131)),
        origin=Origin(xyz=(gate_width - 0.18, 0.0, 2.145)),
        material=safety_yellow,
        name="hanger_right",
    )
    gate_leaf.visual(
        Box((0.080, 0.016, 0.060)),
        origin=Origin(xyz=(0.20, 0.0, 0.055)),
        material=galvanized,
        name="guide_fin_left",
    )
    gate_leaf.visual(
        Box((0.080, 0.016, 0.060)),
        origin=Origin(xyz=(gate_width - 0.20, 0.0, 0.055)),
        material=galvanized,
        name="guide_fin_right",
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.50,
            lower=0.0,
            upper=gate_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide_joint = object_model.get_articulation("frame_to_gate")

    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="y",
        positive_elem="top_channel_front",
        negative_elem="hanger_left",
        min_gap=0.012,
        max_gap=0.020,
        name="left hanger clears the front top channel rail",
    )
    ctx.expect_gap(
        gate_leaf,
        fixed_frame,
        axis="y",
        positive_elem="hanger_left",
        negative_elem="top_channel_rear",
        min_gap=0.012,
        max_gap=0.020,
        name="left hanger clears the rear top channel rail",
    )
    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="y",
        positive_elem="bottom_channel_front",
        negative_elem="guide_fin_left",
        min_gap=0.012,
        max_gap=0.020,
        name="bottom guide fin clears the front guide rail",
    )
    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="x",
        positive_elem="receiver_post",
        negative_elem="right_stile",
        min_gap=0.015,
        max_gap=0.045,
        name="closed gate leaf stops just short of the receiver post",
    )
    ctx.expect_gap(
        gate_leaf,
        fixed_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="bottom_channel_base",
        min_gap=0.040,
        max_gap=0.090,
        name="gate bottom rail rides above the threshold guide base",
    )
    ctx.expect_contact(
        fixed_frame,
        gate_leaf,
        elem_a="top_channel_cap",
        elem_b="hanger_left",
        name="left hanger carriage bears against the top track cap",
    )

    closed_pos = ctx.part_world_position(gate_leaf)
    upper = slide_joint.motion_limits.upper if slide_joint.motion_limits is not None else None
    with ctx.pose({slide_joint: upper}):
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="y",
            positive_elem="top_channel_front",
            negative_elem="hanger_right",
            min_gap=0.012,
            max_gap=0.020,
            name="right hanger stays captured at full opening against the front rail",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="y",
            positive_elem="hanger_right",
            negative_elem="top_channel_rear",
            min_gap=0.012,
            max_gap=0.020,
            name="right hanger stays captured at full opening against the rear rail",
        )
        ctx.expect_contact(
            fixed_frame,
            gate_leaf,
            elem_a="top_channel_cap",
            elem_b="hanger_right",
            name="right hanger carriage bears against the top track cap at full opening",
        )
        open_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf translates left along the track",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 1.90
        and abs(open_pos[1] - closed_pos[1]) < 1e-6
        and abs(open_pos[2] - closed_pos[2]) < 1e-6,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
