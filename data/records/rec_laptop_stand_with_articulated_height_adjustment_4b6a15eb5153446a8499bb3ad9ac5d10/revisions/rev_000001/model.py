from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_laptop_stand")

    graphite = model.material("powder_coated_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=graphite,
        name="weighted_plate",
    )
    base.visual(
        Box((0.22, 0.12, 0.006)),
        origin=Origin(xyz=(-0.025, 0.0, 0.038)),
        material=dark_aluminum,
        name="top_inset",
    )
    for idx, x in enumerate((-0.145, 0.145)):
        for jdx, y in enumerate((-0.095, 0.095)):
            base.visual(
                Cylinder(radius=0.018, length=0.006),
                origin=Origin(xyz=(x, y, -0.002)),
                material=rubber,
                name=f"rubber_foot_{idx}_{jdx}",
            )

    # Forward-offset fixed sleeve.  It is built as a real square tube with a
    # clear central opening so the moving column can slide without colliding.
    sleeve_x = 0.055
    base.visual(
        Box((0.100, 0.090, 0.025)),
        origin=Origin(xyz=(sleeve_x, 0.0, 0.0475)),
        material=dark_aluminum,
        name="socket_block",
    )
    base.visual(
        Box((0.009, 0.068, 0.250)),
        origin=Origin(xyz=(sleeve_x - 0.0295, 0.0, 0.160)),
        material=aluminum,
        name="sleeve_wall_x_neg",
    )
    base.visual(
        Box((0.009, 0.068, 0.250)),
        origin=Origin(xyz=(sleeve_x + 0.0295, 0.0, 0.160)),
        material=aluminum,
        name="sleeve_wall_x_pos",
    )
    base.visual(
        Box((0.068, 0.009, 0.250)),
        origin=Origin(xyz=(sleeve_x, -0.0295, 0.160)),
        material=aluminum,
        name="sleeve_wall_y_neg",
    )
    base.visual(
        Box((0.068, 0.009, 0.250)),
        origin=Origin(xyz=(sleeve_x, 0.0295, 0.160)),
        material=aluminum,
        name="sleeve_wall_y_pos",
    )

    column = model.part("column")
    column.visual(
        Box((0.034, 0.034, 0.380)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=dark_aluminum,
        name="inner_tube",
    )
    column.visual(
        Box((0.070, 0.060, 0.040)),
        origin=Origin(xyz=(0.025, 0.0, 0.390)),
        material=dark_aluminum,
        name="tilt_head_body",
    )
    column.visual(
        Cylinder(radius=0.012, length=0.219),
        origin=Origin(xyz=(0.065, 0.0, 0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="center_hinge_barrel",
    )

    tray = model.part("tray")
    deck = SlotPatternPanelGeometry(
        (0.320, 0.380),
        0.012,
        slot_size=(0.055, 0.008),
        pitch=(0.075, 0.035),
        frame=0.026,
        corner_radius=0.014,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(deck, "slotted_tray_deck"),
        origin=Origin(xyz=(0.170, 0.0, 0.018)),
        material=aluminum,
        name="slotted_deck",
    )
    tray.visual(
        Cylinder(radius=0.014, length=0.065),
        origin=Origin(xyz=(0.0, -0.142, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_barrel_0",
    )
    tray.visual(
        Cylinder(radius=0.014, length=0.065),
        origin=Origin(xyz=(0.0, 0.142, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_barrel_1",
    )
    for idx, y in enumerate((-0.142, 0.142)):
        tray.visual(
            Box((0.038, 0.016, 0.026)),
            origin=Origin(xyz=(0.020, y, 0.010)),
            material=aluminum,
            name=f"hinge_ear_{idx}",
        )
    tray.visual(
        Box((0.275, 0.010, 0.022)),
        origin=Origin(xyz=(0.185, -0.187, 0.032)),
        material=aluminum,
        name="side_flange_0",
    )
    tray.visual(
        Box((0.275, 0.010, 0.022)),
        origin=Origin(xyz=(0.185, 0.187, 0.032)),
        material=aluminum,
        name="side_flange_1",
    )
    for idx, y in enumerate((-0.105, 0.105)):
        tray.visual(
            Box((0.022, 0.085, 0.055)),
            origin=Origin(xyz=(0.326, y, 0.0485)),
            material=aluminum,
            name=f"front_lip_{idx}",
        )
        tray.visual(
            Box((0.018, 0.072, 0.008)),
            origin=Origin(xyz=(0.337, y, 0.079)),
            material=rubber,
            name=f"lip_pad_{idx}",
        )
    for idx, y in enumerate((-0.070, 0.070)):
        tray.visual(
            Box((0.205, 0.026, 0.004)),
            origin=Origin(xyz=(0.185, y, 0.0255)),
            material=rubber,
            name=f"deck_pad_{idx}",
        )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(sleeve_x, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.140),
    )
    model.articulation(
        "column_to_tray",
        ArticulationType.REVOLUTE,
        parent=column,
        child=tray,
        origin=Origin(xyz=(0.065, 0.0, 0.405)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.35, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("base_to_column")
    tilt = object_model.get_articulation("column_to_tray")

    ctx.check(
        "desktop stand has base column and tray",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )
    ctx.check(
        "column height adjustment is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "tray tilt is a horizontal hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (0.0, -1.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    # The inner column is truly clearanced inside the fixed square sleeve.
    ctx.expect_gap(
        column,
        base,
        axis="x",
        positive_elem="inner_tube",
        negative_elem="sleeve_wall_x_neg",
        min_gap=0.004,
        name="inner tube clears negative x sleeve wall",
    )
    ctx.expect_gap(
        base,
        column,
        axis="x",
        positive_elem="sleeve_wall_x_pos",
        negative_elem="inner_tube",
        min_gap=0.004,
        name="inner tube clears positive x sleeve wall",
    )
    ctx.expect_gap(
        column,
        base,
        axis="y",
        positive_elem="inner_tube",
        negative_elem="sleeve_wall_y_neg",
        min_gap=0.004,
        name="inner tube clears negative y sleeve wall",
    )
    ctx.expect_gap(
        base,
        column,
        axis="y",
        positive_elem="sleeve_wall_y_pos",
        negative_elem="inner_tube",
        min_gap=0.004,
        name="inner tube clears positive y sleeve wall",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="sleeve_wall_x_pos",
        min_overlap=0.20,
        name="collapsed column remains engaged in sleeve",
    )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({slide: 0.140}):
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="sleeve_wall_x_pos",
            min_overlap=0.07,
            name="extended column still retained by sleeve",
        )
        extended_column_pos = ctx.part_world_position(column)
    ctx.check(
        "prismatic column raises tray head",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.12,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.14,
        name="tray is carried above the base by the column",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")
    rest_lip_z = None
    if rest_lip_aabb is not None:
        rest_lip_z = (rest_lip_aabb[0][2] + rest_lip_aabb[1][2]) / 2.0
    with ctx.pose({tilt: 0.45}):
        tilted_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")
        tilted_lip_z = None
        if tilted_lip_aabb is not None:
            tilted_lip_z = (tilted_lip_aabb[0][2] + tilted_lip_aabb[1][2]) / 2.0
    ctx.check(
        "positive tray tilt raises the retaining lips",
        rest_lip_z is not None and tilted_lip_z is not None and tilted_lip_z > rest_lip_z + 0.08,
        details=f"rest_lip_z={rest_lip_z}, tilted_lip_z={tilted_lip_z}",
    )

    return ctx.report()


object_model = build_object_model()
