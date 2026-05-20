from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_W = 0.685
CABINET_D = 0.670
DECK_T = 0.022
CABINET_H = 0.922
BODY_H = CABINET_H - DECK_T
BACKSPLASH_H = 0.220
BACKSPLASH_D = 0.085
WALL_T = 0.022

OPENING_W = 0.435
OPENING_D = 0.335
OPENING_Y = -0.030

LID_W = 0.615
LID_D = 0.560
LID_T = 0.026
LID_HINGE_Y = CABINET_D * 0.5 - BACKSPLASH_D - 0.002

HATCH_W = 0.098
HATCH_D = 0.074
HATCH_X = CABINET_W * 0.5 - 0.110
HATCH_HINGE_Y = CABINET_D * 0.5 - 0.004

TUB_OUTER_R = 0.205
TUB_INNER_R = 0.188
TUB_H = 0.430
TUB_BASE_Z = 0.405
TUB_Y = OPENING_Y

KNOB_Y = CABINET_D * 0.5 - BACKSPLASH_D
KNOB_Z = CABINET_H + 0.134
KNOB_X = 0.155
SWITCH_Y = KNOB_Y - 0.004
SWITCH_Z = CABINET_H + 0.138
SWITCH_X = 0.044


def _cabinet_body_mesh():
    outer = cq.Workplane("XY").box(CABINET_W, CABINET_D, BODY_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(0.026)
    inner = (
        cq.Workplane("XY")
        .box(CABINET_W - 2.0 * WALL_T, CABINET_D - 2.0 * WALL_T, BODY_H - WALL_T, centered=(True, True, False))
        .translate((0.0, 0.0, WALL_T))
    )
    return outer.cut(inner)


def _deck_ring_mesh():
    deck = cq.Workplane("XY").box(CABINET_W, CABINET_D, DECK_T, centered=(True, True, False))
    deck = deck.translate((0.0, 0.0, BODY_H))
    opening = (
        cq.Workplane("XY")
        .center(0.0, OPENING_Y)
        .box(OPENING_W, OPENING_D, DECK_T + 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_H - 0.005))
    )
    return deck.cut(opening)


def _backsplash_mesh():
    backsplash = (
        cq.Workplane("XY")
        .box(CABINET_W * 0.94, BACKSPLASH_D, BACKSPLASH_H, centered=(True, True, False))
        .translate((0.0, CABINET_D * 0.5 - BACKSPLASH_D * 0.5, CABINET_H))
    )
    return backsplash.edges("|X").fillet(0.010)


def _lid_mesh(width: float, depth: float, thickness: float):
    lid = cq.Workplane("XY").box(width, depth, thickness, centered=(True, True, False))
    lid = lid.translate((0.0, -depth * 0.5, 0.0))
    lid = lid.edges("|Z").fillet(0.010)
    lid = lid.faces(">Z").edges().fillet(0.006)
    return lid


def _tub_mesh():
    shell = cq.Workplane("XY").circle(TUB_OUTER_R).extrude(TUB_H)
    core = cq.Workplane("XY").circle(TUB_INNER_R).extrude(TUB_H - 0.018).translate((0.0, 0.0, 0.018))
    return shell.cut(core)


def _chrome_knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.032,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.060, 0.007, flare=0.05),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=12.0),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_top_load_washer")

    porcelain = model.material("porcelain_white", rgba=(0.93, 0.94, 0.92, 1.0))
    porcelain_shadow = model.material("porcelain_shadow", rgba=(0.84, 0.85, 0.84, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.88, 1.0))
    basket = model.material("basket_white", rgba=(0.90, 0.91, 0.92, 1.0))
    switch_black = model.material("switch_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.72, 0.74, 0.76, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_body_mesh(), "washer_cabinet_body"),
        material=porcelain,
        name="cabinet_body",
    )
    cabinet.visual(
        mesh_from_cadquery(_deck_ring_mesh(), "washer_deck_ring"),
        material=porcelain,
        name="deck_ring",
    )
    cabinet.visual(
        mesh_from_cadquery(_backsplash_mesh(), "washer_backsplash"),
        material=porcelain_shadow,
        name="backsplash",
    )
    cabinet.visual(
        Cylinder(radius=0.046, length=TUB_BASE_Z),
        origin=Origin(xyz=(0.0, TUB_Y, TUB_BASE_Z * 0.5)),
        material=porcelain_shadow,
        name="drive_pedestal",
    )
    cabinet.visual(
        Box((CABINET_W * 0.80, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, CABINET_D * 0.5 - BACKSPLASH_D + 0.005, CABINET_H + 0.078)),
        material=trim_grey,
        name="control_trim",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_mesh(LID_W, LID_D, LID_T), "washer_lid"),
        material=porcelain,
        name="lid_panel",
    )
    lid.visual(
        Box((0.340, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, -LID_D + 0.016, LID_T - 0.002)),
        material=chrome,
        name="front_handle",
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_tub_mesh(), "washer_tub"),
        material=basket,
        name="tub_shell",
    )
    tub.visual(
        Cylinder(radius=0.048, length=0.101),
        origin=Origin(xyz=(0.0, 0.0, 0.0685)),
        material=basket,
        name="agitator_base",
    )
    tub.visual(
        Cylinder(radius=0.034, length=0.151),
        origin=Origin(xyz=(0.0, 0.0, 0.1695)),
        material=basket,
        name="agitator_mid",
    )
    tub.visual(
        Cylinder(radius=0.044, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.273)),
        material=basket,
        name="agitator_cap",
    )
    tub.visual(
        Box((0.016, 0.060, 0.150)),
        origin=Origin(xyz=(TUB_INNER_R - 0.002, 0.0, 0.180)),
        material=basket,
        name="basket_vane",
    )

    hatch = model.part("bleach_hatch")
    hatch.visual(
        mesh_from_cadquery(_lid_mesh(HATCH_W, HATCH_D, 0.018), "washer_bleach_hatch"),
        material=porcelain,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.042, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -HATCH_D + 0.010, 0.016)),
        material=chrome,
        name="hatch_pull",
    )

    left_knob = model.part("knob_0")
    left_knob.visual(
        _chrome_knob_mesh("washer_knob_left"),
        origin=Origin(xyz=(0.0, 0.0, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_shell",
    )

    right_knob = model.part("knob_1")
    right_knob.visual(
        _chrome_knob_mesh("washer_knob_right"),
        origin=Origin(xyz=(0.0, 0.0, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_shell",
    )

    for index, x_pos in enumerate((-SWITCH_X, SWITCH_X)):
        switch = model.part(f"switch_{index}")
        switch.visual(
            Cylinder(radius=0.004, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=switch_black,
            name="pivot_barrel",
        )
        switch.visual(
            Box((0.020, 0.014, 0.038)),
            origin=Origin(xyz=(0.0, -0.008, 0.0)),
            material=switch_black,
            name="rocker",
        )
        model.articulation(
            f"cabinet_to_switch_{index}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=switch,
            origin=Origin(xyz=(x_pos, SWITCH_Y, SWITCH_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=-0.32, upper=0.32),
        )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, CABINET_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.1, lower=0.0, upper=1.32),
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, TUB_Y, TUB_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=10.0),
    )
    model.articulation(
        "cabinet_to_bleach_hatch",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=hatch,
        origin=Origin(xyz=(HATCH_X, HATCH_HINGE_Y, CABINET_H + BACKSPLASH_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "cabinet_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=left_knob,
        origin=Origin(xyz=(-KNOB_X, KNOB_Y, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )
    model.articulation(
        "cabinet_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=right_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    hatch = object_model.get_part("bleach_hatch")
    left_knob = object_model.get_part("knob_0")
    right_knob = object_model.get_part("knob_1")
    left_switch = object_model.get_part("switch_0")
    right_switch = object_model.get_part("switch_1")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    hatch_hinge = object_model.get_articulation("cabinet_to_bleach_hatch")
    tub_spin = object_model.get_articulation("cabinet_to_tub")

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="deck_ring",
        max_gap=0.004,
        max_penetration=0.0,
        name="lid sits flush on the deck ring",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        elem_a="lid_panel",
        elem_b="deck_ring",
        min_overlap=0.50,
        name="lid covers the washer opening footprint",
    )
    ctx.expect_gap(
        cabinet,
        left_knob,
        axis="y",
        positive_elem="backsplash",
        negative_elem="knob_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="left knob seats on the backsplash",
    )
    ctx.expect_gap(
        cabinet,
        right_knob,
        axis="y",
        positive_elem="backsplash",
        negative_elem="knob_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="right knob seats on the backsplash",
    )
    ctx.expect_gap(
        cabinet,
        left_switch,
        axis="y",
        positive_elem="backsplash",
        negative_elem="rocker",
        max_gap=0.006,
        max_penetration=0.0,
        name="left rocker sits on the control panel",
    )
    ctx.expect_gap(
        cabinet,
        right_switch,
        axis="y",
        positive_elem="backsplash",
        negative_elem="rocker",
        max_gap=0.006,
        max_penetration=0.0,
        name="right rocker sits on the control panel",
    )

    vane_rest = None
    vane_quarter = None
    handle_rest = None
    handle_open = None
    hatch_rest = None
    hatch_open = None

    with ctx.pose({tub_spin: 0.0}):
        vane_rest = ctx.part_element_world_aabb(tub, elem="basket_vane")
    with ctx.pose({tub_spin: math.pi / 2.0}):
        vane_quarter = ctx.part_element_world_aabb(tub, elem="basket_vane")
    with ctx.pose({lid_hinge: 0.0}):
        handle_rest = ctx.part_element_world_aabb(lid, elem="front_handle")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        handle_open = ctx.part_element_world_aabb(lid, elem="front_handle")
    with ctx.pose({hatch_hinge: 0.0}):
        hatch_rest = ctx.part_element_world_aabb(hatch, elem="hatch_pull")
    with ctx.pose({hatch_hinge: hatch_hinge.motion_limits.upper}):
        hatch_open = ctx.part_element_world_aabb(hatch, elem="hatch_pull")

    ctx.check(
        "tub basket vane rotates around the center axis",
        vane_rest is not None
        and vane_quarter is not None
        and abs(((vane_rest[0][0] + vane_rest[1][0]) * 0.5) - ((vane_quarter[0][0] + vane_quarter[1][0]) * 0.5)) > 0.08
        and abs(((vane_rest[0][1] + vane_rest[1][1]) * 0.5) - ((vane_quarter[0][1] + vane_quarter[1][1]) * 0.5)) > 0.08,
        details=f"rest={vane_rest}, quarter_turn={vane_quarter}",
    )
    ctx.check(
        "lid handle rises when the lid opens",
        handle_rest is not None
        and handle_open is not None
        and handle_open[0][2] > handle_rest[0][2] + 0.20,
        details=f"closed={handle_rest}, open={handle_open}",
    )
    ctx.check(
        "bleach hatch swings upward",
        hatch_rest is not None
        and hatch_open is not None
        and hatch_open[0][2] > hatch_rest[0][2] + 0.04,
        details=f"closed={hatch_rest}, open={hatch_open}",
    )

    return ctx.report()


object_model = build_object_model()
