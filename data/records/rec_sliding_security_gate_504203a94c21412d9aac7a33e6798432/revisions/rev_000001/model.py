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

    steel_dark = model.material("steel_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.33, 0.35, 0.37, 1.0))
    steel_light = model.material("steel_light", rgba=(0.54, 0.56, 0.58, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.78, 0.66, 0.18, 1.0))

    frame = model.part("frame")

    post_size = (0.10, 0.10, 2.22)
    frame.visual(
        Box(post_size),
        origin=Origin(xyz=(0.0, 0.0, post_size[2] / 2.0)),
        material=steel_dark,
        name="latch_post",
    )
    frame.visual(
        Box(post_size),
        origin=Origin(xyz=(3.20, 0.0, post_size[2] / 2.0)),
        material=steel_dark,
        name="support_post",
    )
    frame.visual(
        Box((3.30, 0.12, 0.12)),
        origin=Origin(xyz=(1.60, 0.0, 2.28)),
        material=steel_dark,
        name="head_beam",
    )
    frame.visual(
        Box((3.12, 0.13, 0.012)),
        origin=Origin(xyz=(1.60, 0.0, 2.214)),
        material=steel_mid,
        name="track_roof",
    )
    frame.visual(
        Box((3.12, 0.008, 0.064)),
        origin=Origin(xyz=(1.60, -0.051, 2.176)),
        material=steel_mid,
        name="track_side_back",
    )
    frame.visual(
        Box((3.12, 0.008, 0.064)),
        origin=Origin(xyz=(1.60, 0.051, 2.176)),
        material=steel_mid,
        name="track_side_front",
    )
    frame.visual(
        Box((0.06, 0.11, 0.05)),
        origin=Origin(xyz=(0.06, 0.0, 2.183)),
        material=warning_yellow,
        name="left_track_stop",
    )
    frame.visual(
        Box((0.06, 0.11, 0.05)),
        origin=Origin(xyz=(3.14, 0.0, 2.183)),
        material=warning_yellow,
        name="right_track_stop",
    )
    frame.visual(
        Box((0.06, 0.02, 0.20)),
        origin=Origin(xyz=(0.06, -0.06, 1.22)),
        material=steel_mid,
        name="receiver_plate",
    )
    frame.inertial = Inertial.from_geometry(
        Box((3.30, 0.18, 2.40)),
        mass=85.0,
        origin=Origin(xyz=(1.60, 0.0, 1.20)),
    )

    gate_leaf = model.part("gate_leaf")

    panel_top = -0.117
    panel_bottom = -2.126
    panel_height = panel_top - panel_bottom
    stile_width = 0.05
    rail_height = 0.05
    panel_depth = 0.035
    gate_width = 1.45
    left_edge = -0.08
    right_edge = left_edge + gate_width
    inner_rail_length = gate_width - (2.0 * stile_width)
    frame_center_x = (left_edge + right_edge) / 2.0
    stile_center_z = (panel_top + panel_bottom) / 2.0

    gate_leaf.visual(
        Box((stile_width, panel_depth, panel_height)),
        origin=Origin(xyz=(left_edge + (stile_width / 2.0), 0.0, stile_center_z)),
        material=steel_dark,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((stile_width, panel_depth, panel_height)),
        origin=Origin(xyz=(right_edge - (stile_width / 2.0), 0.0, stile_center_z)),
        material=steel_dark,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((inner_rail_length, panel_depth, rail_height)),
        origin=Origin(
            xyz=(frame_center_x, 0.0, panel_top - (rail_height / 2.0)),
        ),
        material=steel_dark,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((inner_rail_length, panel_depth, rail_height)),
        origin=Origin(
            xyz=(frame_center_x, 0.0, panel_bottom + (rail_height / 2.0)),
        ),
        material=steel_dark,
        name="bottom_rail",
    )

    picket_height = panel_height - (2.0 * rail_height)
    picket_center_z = (panel_top - rail_height + panel_bottom + rail_height) / 2.0
    for index, center_x in enumerate((0.18, 0.39, 0.60, 0.81, 1.02, 1.23)):
        gate_leaf.visual(
            Box((0.022, 0.014, picket_height)),
            origin=Origin(xyz=(center_x, 0.0, picket_center_z)),
            material=steel_light,
            name=f"picket_{index}",
        )

    gate_leaf.visual(
        Box((0.10, 0.05, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_mid,
        name="left_carriage",
    )
    gate_leaf.visual(
        Box((0.10, 0.05, 0.026)),
        origin=Origin(xyz=(1.15, 0.0, 0.0)),
        material=steel_mid,
        name="right_carriage",
    )
    gate_leaf.visual(
        Box((0.03, 0.012, 0.104)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=steel_light,
        name="left_hanger",
    )
    gate_leaf.visual(
        Box((0.03, 0.012, 0.104)),
        origin=Origin(xyz=(1.15, 0.0, -0.065)),
        material=steel_light,
        name="right_hanger",
    )
    gate_leaf.visual(
        Box((0.11, 0.020, 0.030)),
        origin=Origin(xyz=(0.645, 0.0, -2.086)),
        material=steel_mid,
        name="bottom_guide_shoe",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.48, 0.08, 2.16)),
        mass=42.0,
        origin=Origin(xyz=(0.645, 0.0, -1.05)),
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.14, 0.0, 2.176)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.35,
            lower=0.0,
            upper=1.59,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide_joint = object_model.get_articulation("frame_to_gate_leaf")

    ctx.expect_gap(
        gate_leaf,
        frame,
        axis="x",
        positive_elem="left_stile",
        negative_elem="latch_post",
        min_gap=0.008,
        max_gap=0.020,
        name="closed leaf sits just clear of the latch post",
    )
    ctx.expect_gap(
        frame,
        gate_leaf,
        axis="z",
        positive_elem="track_roof",
        negative_elem="left_carriage",
        min_gap=0.017,
        max_gap=0.025,
        name="left carriage hangs beneath the track roof",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="xy",
        inner_elem="left_carriage",
        outer_elem="track_roof",
        margin=0.0,
        name="left carriage stays captured within the top track footprint",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="xy",
        inner_elem="right_carriage",
        outer_elem="track_roof",
        margin=0.0,
        name="right carriage stays captured within the top track footprint",
    )

    closed_position = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        ctx.expect_gap(
            gate_leaf,
            frame,
            axis="x",
            positive_elem="left_stile",
            negative_elem="latch_post",
            min_gap=1.58,
            max_gap=1.62,
            name="open leaf clears the doorway by sliding to the stack side",
        )
        ctx.expect_within(
            gate_leaf,
            frame,
            axes="xy",
            inner_elem="right_carriage",
            outer_elem="track_roof",
            margin=0.0,
            name="right carriage remains within the track at full opening",
        )
        open_position = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf translates in the positive track direction",
        closed_position is not None
        and open_position is not None
        and open_position[0] > closed_position[0] + 1.5,
        details=f"closed={closed_position}, open={open_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
