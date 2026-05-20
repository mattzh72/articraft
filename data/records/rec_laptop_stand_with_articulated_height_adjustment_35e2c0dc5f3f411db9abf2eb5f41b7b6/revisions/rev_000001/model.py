from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_W = 0.280
BASE_D = 0.235
FRAME_H = 0.012
REAR_BEAM_D = 0.028
REAR_BEAM_H = 0.018
SIDE_RAIL_W = 0.020
FRONT_RAIL_D = 0.018

COLUMN_X = 0.106
COLUMN_Y = BASE_D / 2.0 - REAR_BEAM_D / 2.0
COLUMN_SPAN = 2.0 * COLUMN_X

SLEEVE_W = 0.028
SLEEVE_D = 0.024
SLEEVE_H = 0.150
SLEEVE_WALL = 0.003

INNER_W = 0.018
INNER_D = 0.013
INNER_L = 0.340
INNER_CENTER_Z = 0.025
INNER_TOP = INNER_CENTER_Z + INNER_L / 2.0
SLIDE_TRAVEL = 0.080

TRAY_W = 0.300
TRAY_D = 0.240
TRAY_LEFT_OVERHANG = 0.040
TRAY_CENTER_X = TRAY_W / 2.0 - TRAY_LEFT_OVERHANG
TRAY_HINGE_Y = 0.006
TRAY_HINGE_Z = 0.024


def _add_outer_sleeve(part, material, knob_material, *, side_sign: float) -> None:
    wall_x = SLEEVE_W / 2.0 - SLEEVE_WALL / 2.0
    rear_y = SLEEVE_D / 2.0 - SLEEVE_WALL / 2.0

    part.visual(
        Box((SLEEVE_WALL, SLEEVE_D, SLEEVE_H)),
        origin=Origin(xyz=(-wall_x, 0.0, SLEEVE_H / 2.0)),
        material=material,
        name="outboard_wall",
    )
    part.visual(
        Box((SLEEVE_WALL, SLEEVE_D, SLEEVE_H)),
        origin=Origin(xyz=(wall_x, 0.0, SLEEVE_H / 2.0)),
        material=material,
        name="inboard_wall",
    )
    part.visual(
        Box((SLEEVE_W, SLEEVE_WALL, SLEEVE_H)),
        origin=Origin(xyz=(0.0, rear_y, SLEEVE_H / 2.0)),
        material=material,
        name="rear_wall",
    )
    part.visual(
        Box((0.002, 0.012, SLEEVE_H - 0.020)),
        origin=Origin(xyz=(-0.010, -0.002, (SLEEVE_H - 0.020) / 2.0)),
        material=material,
        name="guide_pad_0",
    )
    part.visual(
        Box((0.002, 0.012, SLEEVE_H - 0.020)),
        origin=Origin(xyz=(0.010, -0.002, (SLEEVE_H - 0.020) / 2.0)),
        material=material,
        name="guide_pad_1",
    )
    part.visual(
        Box((SLEEVE_W + 0.008, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -SLEEVE_D / 2.0 - 0.002, SLEEVE_H - 0.006)),
        material=material,
        name="front_bridge",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(
            xyz=(side_sign * (SLEEVE_W / 2.0 + 0.006), 0.0, SLEEVE_H - 0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=knob_material,
        name="clamp_knob",
    )


def _add_inner_column(part, material, cap_material) -> None:
    part.visual(
        Box((INNER_W, INNER_D, INNER_L)),
        origin=Origin(xyz=(0.0, -0.002, INNER_CENTER_Z)),
        material=material,
        name="column_stem",
    )
    part.visual(
        Box((0.024, 0.019, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, INNER_TOP - 0.006)),
        material=cap_material,
        name="top_cap",
    )
    part.visual(
        Box((0.026, 0.021, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, INNER_TOP - 0.020)),
        material=cap_material,
        name="cap_block",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    graphite = model.material("graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    soft_pad = model.material("soft_pad", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_W, FRONT_RAIL_D, FRAME_H)),
        origin=Origin(xyz=(0.0, -BASE_D / 2.0 + FRONT_RAIL_D / 2.0, FRAME_H / 2.0)),
        material=graphite,
        name="front_rail",
    )
    base.visual(
        Box((SIDE_RAIL_W, BASE_D - 0.014, FRAME_H)),
        origin=Origin(xyz=(-BASE_W / 2.0 + SIDE_RAIL_W / 2.0, 0.0, FRAME_H / 2.0)),
        material=graphite,
        name="side_rail_0",
    )
    base.visual(
        Box((SIDE_RAIL_W, BASE_D - 0.014, FRAME_H)),
        origin=Origin(xyz=(BASE_W / 2.0 - SIDE_RAIL_W / 2.0, 0.0, FRAME_H / 2.0)),
        material=graphite,
        name="side_rail_1",
    )
    base.visual(
        Box((BASE_W, REAR_BEAM_D, REAR_BEAM_H)),
        origin=Origin(
            xyz=(
                0.0,
                BASE_D / 2.0 - REAR_BEAM_D / 2.0,
                REAR_BEAM_H / 2.0,
            )
        ),
        material=graphite,
        name="rear_beam",
    )
    base.visual(
        Box((0.240, 0.016, FRAME_H)),
        origin=Origin(xyz=(0.0, -0.008, FRAME_H / 2.0)),
        material=graphite,
        name="center_brace",
    )
    for index, x_pos in enumerate((-0.095, 0.095)):
        base.visual(
            Box((0.040, 0.028, 0.004)),
            origin=Origin(xyz=(x_pos, -0.108, 0.002)),
            material=soft_pad,
            name=f"foot_{index}",
        )

    left_sleeve = model.part("left_sleeve")
    _add_outer_sleeve(left_sleeve, graphite, satin_aluminum, side_sign=-1.0)
    model.articulation(
        "base_to_left_sleeve",
        ArticulationType.FIXED,
        parent=base,
        child=left_sleeve,
        origin=Origin(xyz=(-COLUMN_X, COLUMN_Y, REAR_BEAM_H)),
    )

    right_sleeve = model.part("right_sleeve")
    _add_outer_sleeve(right_sleeve, graphite, satin_aluminum, side_sign=1.0)
    model.articulation(
        "base_to_right_sleeve",
        ArticulationType.FIXED,
        parent=base,
        child=right_sleeve,
        origin=Origin(xyz=(COLUMN_X, COLUMN_Y, REAR_BEAM_H)),
    )

    left_column = model.part("left_column")
    _add_inner_column(left_column, satin_aluminum, graphite)
    model.articulation(
        "left_sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=left_sleeve,
        child=left_column,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.16,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    right_column = model.part("right_column")
    _add_inner_column(right_column, satin_aluminum, graphite)
    model.articulation(
        "right_sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=right_sleeve,
        child=right_column,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.16,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
        mimic=Mimic(joint="left_sleeve_to_column"),
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.034, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="left_saddle",
    )
    bridge.visual(
        Box((0.034, 0.024, 0.016)),
        origin=Origin(xyz=(COLUMN_SPAN, 0.0, 0.008)),
        material=graphite,
        name="right_saddle",
    )
    bridge.visual(
        Box((COLUMN_SPAN + 0.030, 0.016, 0.012)),
        origin=Origin(xyz=(COLUMN_SPAN / 2.0, -0.001, 0.008)),
        material=graphite,
        name="crossbar",
    )
    bridge.visual(
        Cylinder(radius=0.006, length=COLUMN_SPAN + 0.080),
        origin=Origin(
            xyz=(COLUMN_SPAN / 2.0, TRAY_HINGE_Y, TRAY_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="hinge_tube",
    )
    bridge.visual(
        Box((0.054, 0.022, 0.018)),
        origin=Origin(xyz=(COLUMN_SPAN / 2.0, -0.004, 0.005)),
        material=graphite,
        name="center_block",
    )
    bridge.visual(
        Box((0.060, 0.012, 0.008)),
        origin=Origin(xyz=(COLUMN_SPAN / 2.0, 0.003, 0.016)),
        material=graphite,
        name="hinge_support",
    )
    model.articulation(
        "left_column_to_bridge",
        ArticulationType.FIXED,
        parent=left_column,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, INNER_TOP)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, 0.008)),
        origin=Origin(xyz=(TRAY_CENTER_X, -0.128, 0.010)),
        material=charcoal,
        name="deck",
    )
    tray.visual(
        Box((TRAY_W, 0.012, 0.018)),
        origin=Origin(xyz=(TRAY_CENTER_X, -0.242, 0.019)),
        material=charcoal,
        name="front_lip",
    )
    tray.visual(
        Box((0.012, 0.150, 0.014)),
        origin=Origin(xyz=(-0.034, -0.137, 0.017)),
        material=charcoal,
        name="side_rail_0",
    )
    tray.visual(
        Box((0.012, 0.150, 0.014)),
        origin=Origin(xyz=(0.254, -0.137, 0.017)),
        material=charcoal,
        name="side_rail_1",
    )
    tray.visual(
        Box((TRAY_W, 0.014, 0.016)),
        origin=Origin(xyz=(TRAY_CENTER_X, -0.009, 0.002)),
        material=graphite,
        name="rear_leaf",
    )
    tray.visual(
        Box((0.180, 0.026, 0.020)),
        origin=Origin(xyz=(TRAY_CENTER_X, -0.120, -0.004)),
        material=graphite,
        name="center_rib",
    )
    for index, x_pos in enumerate((0.040, 0.180)):
        tray.visual(
            Box((0.050, 0.040, 0.003)),
            origin=Origin(xyz=(x_pos, -0.125, 0.0155)),
            material=soft_pad,
            name=f"pad_{index}",
        )
    model.articulation(
        "bridge_to_tray",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=tray,
        origin=Origin(xyz=(0.0, TRAY_HINGE_Y, TRAY_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_sleeve = object_model.get_part("left_sleeve")
    right_sleeve = object_model.get_part("right_sleeve")
    left_column = object_model.get_part("left_column")
    right_column = object_model.get_part("right_column")
    bridge = object_model.get_part("bridge")
    tray = object_model.get_part("tray")
    left_slide = object_model.get_articulation("left_sleeve_to_column")
    tray_hinge = object_model.get_articulation("bridge_to_tray")

    left_limits = left_slide.motion_limits
    tray_limits = tray_hinge.motion_limits

    ctx.expect_within(
        left_column,
        left_sleeve,
        axes="xy",
        inner_elem="column_stem",
        name="left column stays centered in left sleeve",
    )
    ctx.expect_within(
        right_column,
        right_sleeve,
        axes="xy",
        inner_elem="column_stem",
        name="right column stays centered in right sleeve",
    )
    ctx.expect_overlap(
        left_column,
        left_sleeve,
        axes="z",
        elem_a="column_stem",
        min_overlap=0.120,
        name="left column remains deeply inserted at rest",
    )
    ctx.expect_overlap(
        right_column,
        right_sleeve,
        axes="z",
        elem_a="column_stem",
        min_overlap=0.120,
        name="right column remains deeply inserted at rest",
    )
    ctx.expect_gap(
        bridge,
        right_column,
        axis="z",
        positive_elem="right_saddle",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="right support bridge seats on right column at rest",
    )

    if left_limits is not None and left_limits.upper is not None:
        with ctx.pose({left_slide: left_limits.upper}):
            ctx.expect_within(
                left_column,
                left_sleeve,
                axes="xy",
                inner_elem="column_stem",
                name="left column stays centered when extended",
            )
            ctx.expect_within(
                right_column,
                right_sleeve,
                axes="xy",
                inner_elem="column_stem",
                name="right column stays centered when extended",
            )
            ctx.expect_overlap(
                left_column,
                left_sleeve,
                axes="z",
                elem_a="column_stem",
                min_overlap=0.060,
                name="left column retains insertion at full height",
            )
            ctx.expect_overlap(
                right_column,
                right_sleeve,
                axes="z",
                elem_a="column_stem",
                min_overlap=0.060,
                name="right column retains insertion at full height",
            )
            ctx.expect_gap(
                bridge,
                right_column,
                axis="z",
                positive_elem="right_saddle",
                negative_elem="top_cap",
                max_gap=0.001,
                max_penetration=0.0,
                name="right support bridge remains seated when extended",
            )
            left_pos = ctx.part_world_position(left_column)
            right_pos = ctx.part_world_position(right_column)
            ctx.check(
                "columns extend together",
                left_pos is not None
                and right_pos is not None
                and abs(left_pos[2] - right_pos[2]) <= 1e-6,
                details=f"left={left_pos}, right={right_pos}",
            )

    if tray_limits is not None and tray_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(tray, elem="front_lip")
        with ctx.pose({tray_hinge: tray_limits.upper}):
            opened_aabb = ctx.part_element_world_aabb(tray, elem="front_lip")
        closed_z = None if closed_aabb is None else (closed_aabb[0][2] + closed_aabb[1][2]) / 2.0
        opened_z = None if opened_aabb is None else (opened_aabb[0][2] + opened_aabb[1][2]) / 2.0
        ctx.check(
            "tray front edge rises when tilted",
            closed_z is not None and opened_z is not None and opened_z > closed_z + 0.080,
            details=f"closed_z={closed_z}, opened_z={opened_z}",
        )

    return ctx.report()


object_model = build_object_model()
