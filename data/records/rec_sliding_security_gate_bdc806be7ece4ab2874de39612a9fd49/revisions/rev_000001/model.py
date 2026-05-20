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

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    dark_track = model.material("dark_track", rgba=(0.14, 0.15, 0.17, 1.0))
    hardware = model.material("hardware", rgba=(0.50, 0.52, 0.56, 1.0))

    frame_width = 1.24
    frame_height = 2.12
    threshold_height = 0.04

    leaf_width = 0.82
    leaf_height = 1.78
    closed_leaf_x = 0.33
    leaf_base_z = 0.045
    leaf_travel = 0.24

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.08, 0.08, frame_height)),
        origin=Origin(xyz=(0.0, 0.0, frame_height / 2.0)),
        material=painted_steel,
        name="anchor_post",
    )
    support_frame.visual(
        Box((0.06, 0.08, 2.04)),
        origin=Origin(xyz=(1.21, 0.0, 1.02)),
        material=painted_steel,
        name="receiver_post",
    )
    support_frame.visual(
        Box((1.24, 0.05, threshold_height)),
        origin=Origin(xyz=(0.60, 0.0, threshold_height / 2.0)),
        material=painted_steel,
        name="threshold",
    )
    support_frame.visual(
        Box((1.24, 0.10, 0.06)),
        origin=Origin(xyz=(0.60, 0.0, 2.09)),
        material=dark_track,
        name="header_cap",
    )
    support_frame.visual(
        Box((1.24, 0.008, 0.16)),
        origin=Origin(xyz=(0.60, 0.037, 1.98)),
        material=dark_track,
        name="front_flange",
    )
    support_frame.visual(
        Box((1.24, 0.008, 0.16)),
        origin=Origin(xyz=(0.60, -0.037, 1.98)),
        material=dark_track,
        name="rear_flange",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((1.30, 0.10, frame_height)),
        mass=55.0,
        origin=Origin(xyz=(0.60, 0.0, frame_height / 2.0)),
    )

    gate_leaf = model.part("gate_leaf")

    stile_width = 0.05
    frame_depth = 0.028
    rail_height = 0.05

    gate_leaf.visual(
        Box((stile_width, frame_depth, leaf_height)),
        origin=Origin(xyz=(stile_width / 2.0, 0.0, leaf_height / 2.0)),
        material=painted_steel,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((stile_width, frame_depth, leaf_height)),
        origin=Origin(xyz=(leaf_width - stile_width / 2.0, 0.0, leaf_height / 2.0)),
        material=painted_steel,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((leaf_width, frame_depth, rail_height)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, rail_height / 2.0)),
        material=painted_steel,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((leaf_width, frame_depth, rail_height)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, leaf_height - rail_height / 2.0)),
        material=painted_steel,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((leaf_width, 0.022, 0.04)),
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, 0.92)),
        material=painted_steel,
        name="mid_rail",
    )

    bar_positions = (0.14, 0.225, 0.31, 0.395, 0.48, 0.565, 0.65)
    for index, x_pos in enumerate(bar_positions, start=1):
        gate_leaf.visual(
            Box((0.018, 0.012, 1.68)),
            origin=Origin(xyz=(x_pos, 0.0, 0.89)),
            material=painted_steel,
            name=f"bar_{index}",
        )

    gate_leaf.visual(
        Box((0.03, 0.012, 0.176)),
        origin=Origin(xyz=(0.16, 0.0, 1.868)),
        material=hardware,
        name="hanger_strap_a",
    )
    gate_leaf.visual(
        Box((0.07, 0.020, 0.085)),
        origin=Origin(xyz=(0.16, 0.0, 1.9725)),
        material=hardware,
        name="carriage_a",
    )
    gate_leaf.visual(
        Box((0.03, 0.012, 0.176)),
        origin=Origin(xyz=(0.66, 0.0, 1.868)),
        material=hardware,
        name="hanger_strap_b",
    )
    gate_leaf.visual(
        Box((0.07, 0.020, 0.085)),
        origin=Origin(xyz=(0.66, 0.0, 1.9725)),
        material=hardware,
        name="carriage_b",
    )

    gate_leaf.visual(
        Box((0.014, 0.028, 0.022)),
        origin=Origin(xyz=(0.775, 0.028, 0.86)),
        material=hardware,
        name="handle_standoff_low",
    )
    gate_leaf.visual(
        Box((0.014, 0.028, 0.022)),
        origin=Origin(xyz=(0.775, 0.028, 1.08)),
        material=hardware,
        name="handle_standoff_high",
    )
    gate_leaf.visual(
        Box((0.018, 0.018, 0.26)),
        origin=Origin(xyz=(0.775, 0.051, 0.97)),
        material=hardware,
        name="pull_handle",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, 0.06, 1.96)),
        mass=18.0,
        origin=Origin(xyz=(leaf_width / 2.0, 0.0, 0.98)),
    )

    model.articulation(
        "support_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=(closed_leaf_x, 0.0, leaf_base_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=leaf_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("support_to_gate_leaf")
    travel = slide.motion_limits.upper if slide.motion_limits is not None else 0.0

    ctx.expect_gap(
        gate_leaf,
        support_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="threshold",
        min_gap=0.004,
        max_gap=0.03,
        name="bottom rail clears threshold",
    )
    ctx.expect_gap(
        support_frame,
        gate_leaf,
        axis="x",
        positive_elem="receiver_post",
        negative_elem="right_stile",
        min_gap=0.02,
        max_gap=0.06,
        name="closed gate sits near receiver post",
    )
    ctx.expect_contact(
        support_frame,
        gate_leaf,
        elem_a="header_cap",
        elem_b="carriage_a",
        name="front carriage rides on header cap",
    )
    ctx.expect_gap(
        support_frame,
        gate_leaf,
        axis="y",
        positive_elem="front_flange",
        negative_elem="carriage_a",
        min_gap=0.01,
        max_gap=0.04,
        name="front carriage clears front flange",
    )
    ctx.expect_gap(
        gate_leaf,
        support_frame,
        axis="y",
        positive_elem="carriage_a",
        negative_elem="rear_flange",
        min_gap=0.01,
        max_gap=0.04,
        name="front carriage clears rear flange",
    )

    closed_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: travel}):
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="x",
            positive_elem="left_stile",
            negative_elem="anchor_post",
            min_gap=0.04,
            max_gap=0.10,
            name="open gate clears anchor post",
        )
        ctx.expect_contact(
            support_frame,
            gate_leaf,
            elem_a="header_cap",
            elem_b="carriage_b",
            name="rear carriage stays captured in the header when open",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="y",
            positive_elem="front_flange",
            negative_elem="carriage_b",
            min_gap=0.01,
            max_gap=0.04,
            name="rear carriage clears front flange when open",
        )
        ctx.expect_gap(
            gate_leaf,
            support_frame,
            axis="y",
            positive_elem="carriage_b",
            negative_elem="rear_flange",
            min_gap=0.01,
            max_gap=0.04,
            name="rear carriage clears rear flange when open",
        )
        open_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate slides left to open",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 0.15,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
