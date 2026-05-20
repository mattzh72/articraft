from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

FRAME_WIDTH = 0.30
FRAME_HEIGHT = 0.39
FRAME_DEPTH = 0.016
STILE_WIDTH = 0.025
TOP_RAIL_HEIGHT = 0.032
BOTTOM_RAIL_HEIGHT = 0.050

LEDGE_WIDTH = 0.240
LEDGE_DEPTH = 0.030
LEDGE_THICKNESS = 0.010
LEDGE_CENTER_Y = 0.016
LEDGE_CENTER_Z = -0.147

LIP_WIDTH = 0.230
LIP_DEPTH = 0.006
LIP_HEIGHT = 0.014
LIP_CENTER_Y = 0.028
LIP_CENTER_Z = -0.135

PANEL_WIDTH = 0.242
PANEL_HEIGHT = 0.300
PANEL_DEPTH = 0.010
PANEL_HINGE_Y = FRAME_DEPTH / 2.0
PANEL_HINGE_Z = -0.142

LEG_WIDTH = 0.024
LEG_DEPTH = 0.010
LEG_LENGTH = 0.350
LEG_HINGE_Y = -0.012
LEG_HINGE_Z = 0.164


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_sketch_easel")

    beech = model.material("beech", rgba=(0.78, 0.67, 0.50, 1.0))
    birch = model.material("birch", rgba=(0.86, 0.79, 0.65, 1.0))
    walnut = model.material("walnut", rgba=(0.46, 0.33, 0.22, 1.0))
    hardware = model.material("hardware", rgba=(0.43, 0.40, 0.36, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-(FRAME_WIDTH / 2.0 - STILE_WIDTH / 2.0), 0.0, 0.0)),
        material=beech,
        name="left_stile",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=((FRAME_WIDTH / 2.0 - STILE_WIDTH / 2.0), 0.0, 0.0)),
        material=beech,
        name="right_stile",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, TOP_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0 - TOP_RAIL_HEIGHT / 2.0)),
        material=beech,
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, BOTTOM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HEIGHT / 2.0 + BOTTOM_RAIL_HEIGHT / 2.0)),
        material=beech,
        name="bottom_rail",
    )
    frame.visual(
        Box((LEDGE_WIDTH, LEDGE_DEPTH, LEDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, LEDGE_CENTER_Y, LEDGE_CENTER_Z)),
        material=beech,
        name="front_ledge",
    )
    frame.visual(
        Box((LIP_WIDTH, LIP_DEPTH, LIP_HEIGHT)),
        origin=Origin(xyz=(0.0, LIP_CENTER_Y, LIP_CENTER_Z)),
        material=walnut,
        name="ledge_lip",
    )
    frame.visual(
        Box((0.090, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, LEG_HINGE_Z)),
        material=hardware,
        name="rear_hinge_block",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_DEPTH / 2.0, PANEL_HEIGHT / 2.0)),
        material=birch,
        name="board",
    )
    panel.visual(
        Box((PANEL_WIDTH, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, PANEL_HEIGHT - 0.008)),
        material=beech,
        name="top_cap",
    )
    panel.visual(
        Box((0.060, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.009, PANEL_HEIGHT - 0.030)),
        material=hardware,
        name="clip_block",
    )

    back_leg = model.part("back_leg")
    back_leg.visual(
        Box((0.060, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, -0.009)),
        material=hardware,
        name="hinge_head",
    )
    back_leg.visual(
        Box((LEG_WIDTH, LEG_DEPTH, LEG_LENGTH)),
        origin=Origin(xyz=(0.0, -LEG_DEPTH / 2.0, -LEG_LENGTH / 2.0)),
        material=beech,
        name="strut",
    )
    back_leg.visual(
        Box((0.065, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.007, -LEG_LENGTH + 0.007)),
        material=walnut,
        name="foot_pad",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, PANEL_HINGE_Y, PANEL_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "frame_to_back_leg",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_leg,
        origin=Origin(xyz=(0.0, LEG_HINGE_Y, LEG_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    back_leg = object_model.get_part("back_leg")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    leg_hinge = object_model.get_articulation("frame_to_back_leg")

    ctx.expect_within(
        panel,
        frame,
        axes="x",
        inner_elem="board",
        outer_elem="top_rail",
        margin=0.0,
        name="panel stays inside the frame width",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="board",
        negative_elem="front_ledge",
        min_gap=0.0,
        max_gap=0.001,
        name="panel rests just above the front ledge",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="x",
        elem_a="board",
        elem_b="front_ledge",
        min_overlap=0.200,
        name="front ledge spans most of the sketch panel width",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="y",
        positive_elem="board",
        negative_elem="top_rail",
        min_gap=0.0,
        max_gap=0.002,
        name="panel sits flush against the front frame plane",
    )
    ctx.expect_gap(
        frame,
        back_leg,
        axis="y",
        positive_elem="top_rail",
        negative_elem="hinge_head",
        min_gap=0.003,
        max_gap=0.006,
        name="folded back leg clears the rear of the frame",
    )

    rest_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_hinge: panel_hinge.motion_limits.upper}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "panel tilts forward when opened",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > rest_panel_aabb[1][1] + 0.10,
        details=f"rest={rest_panel_aabb!r}, open={open_panel_aabb!r}",
    )

    rest_leg_aabb = ctx.part_world_aabb(back_leg)
    with ctx.pose({leg_hinge: leg_hinge.motion_limits.upper}):
        open_leg_aabb = ctx.part_world_aabb(back_leg)
    ctx.check(
        "back leg swings rearward to form the easel stance",
        rest_leg_aabb is not None
        and open_leg_aabb is not None
        and open_leg_aabb[0][1] < rest_leg_aabb[0][1] - 0.12,
        details=f"rest={rest_leg_aabb!r}, open={open_leg_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
