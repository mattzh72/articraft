from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PANEL_WIDTH = 0.395
PANEL_DEPTH = 0.160
PANEL_THICKNESS = 0.026
PANEL_CENTER = (0.225, -0.095, -0.018)

SLOT_LEFT_X = 0.160
SLOT_RIGHT_X = 0.440
SLOT_DEPTH = 0.112

EXTENDER_LENGTH = 0.240
EXTENDER_DEPTH = 0.100
EXTENDER_THICKNESS = 0.014


def _visor_shell_cadquery() -> cq.Workplane:
    """Padded C-shaped visor body with a side-opening sleeve for the extender."""
    body = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH, PANEL_DEPTH, PANEL_THICKNESS)
        .edges("|Z")
        .fillet(0.018)
        .translate(PANEL_CENTER)
    )

    slot_center_x = (SLOT_LEFT_X + SLOT_RIGHT_X) * 0.5
    slot_width = SLOT_RIGHT_X - SLOT_LEFT_X
    cutter = (
        cq.Workplane("XY")
        .box(slot_width, SLOT_DEPTH, PANEL_THICKNESS * 3.0)
        .translate((slot_center_x, PANEL_CENTER[1], PANEL_CENTER[2]))
    )
    return body.cut(cutter)


def _extender_leaf_cadquery() -> cq.Workplane:
    leaf = (
        cq.Workplane("XY")
        .box(EXTENDER_LENGTH, EXTENDER_DEPTH, EXTENDER_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
        .translate((EXTENDER_LENGTH * 0.5, 0.0, 0.0))
    )
    return leaf


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_windshield_sun_visor")

    headliner_fabric = model.material("headliner_fabric", rgba=(0.72, 0.68, 0.60, 1.0))
    visor_fabric = model.material("padded_visor_fabric", rgba=(0.66, 0.63, 0.56, 1.0))
    extender_fabric = model.material("sliding_extender_fabric", rgba=(0.74, 0.71, 0.64, 1.0))
    dark_plastic = model.material("dark_plastic_hardware", rgba=(0.10, 0.10, 0.095, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.56, 0.55, 1.0))

    headliner = model.part("headliner")
    headliner.visual(
        Box((0.620, 0.220, 0.012)),
        origin=Origin(xyz=(0.240, 0.000, 0.020)),
        material=headliner_fabric,
        name="roof_panel",
    )
    headliner.visual(
        Box((0.620, 0.014, 0.016)),
        origin=Origin(xyz=(0.240, -0.117, 0.012)),
        material=headliner_fabric,
        name="windshield_edge_trim",
    )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        Box((0.070, 0.050, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, -0.003)),
        material=dark_plastic,
        name="roof_plate",
    )
    pivot_bracket.visual(
        Box((0.035, 0.035, 0.013)),
        origin=Origin(xyz=(0.000, 0.000, -0.0125)),
        material=dark_plastic,
        name="bracket_pedestal",
    )
    pivot_bracket.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, -0.023)),
        material=dark_plastic,
        name="swivel_collar",
    )
    for index, screw_x in enumerate((-0.022, 0.022)):
        pivot_bracket.visual(
            Cylinder(radius=0.005, length=0.002),
            origin=Origin(xyz=(screw_x, 0.012, -0.007)),
            material=satin_metal,
            name=f"screw_cap_{index}",
        )

    retaining_clip = model.part("retaining_clip")
    retaining_clip.visual(
        Box((0.040, 0.030, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, -0.003)),
        material=dark_plastic,
        name="clip_base",
    )
    retaining_clip.visual(
        Box((0.030, 0.006, 0.038)),
        origin=Origin(xyz=(0.000, -0.012, -0.025)),
        material=dark_plastic,
        name="clip_ear_0",
    )
    retaining_clip.visual(
        Box((0.030, 0.006, 0.038)),
        origin=Origin(xyz=(0.000, 0.012, -0.025)),
        material=dark_plastic,
        name="clip_ear_1",
    )
    retaining_clip.visual(
        Box((0.018, 0.005, 0.006)),
        origin=Origin(xyz=(0.000, -0.009, -0.037)),
        material=dark_plastic,
        name="clip_lip_0",
    )
    retaining_clip.visual(
        Box((0.018, 0.005, 0.006)),
        origin=Origin(xyz=(0.000, 0.009, -0.037)),
        material=dark_plastic,
        name="clip_lip_1",
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=dark_plastic,
        name="swivel_disk",
    )
    swivel_yoke.visual(
        Box((0.046, 0.030, 0.014)),
        origin=Origin(xyz=(0.023, 0.000, -0.010)),
        material=dark_plastic,
        name="rod_socket_block",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.037, 0.000, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="rod_socket_ring",
    )

    hinge_rod = model.part("hinge_rod")
    hinge_rod.visual(
        Cylinder(radius=0.006, length=0.410),
        origin=Origin(xyz=(0.205, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="rod",
    )
    hinge_rod.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.417, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="end_cap",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_cadquery(_visor_shell_cadquery(), "visor_panel_shell", tolerance=0.0008),
        material=visor_fabric,
        name="visor_shell",
    )
    visor_panel.visual(
        Cylinder(radius=0.009, length=0.340),
        origin=Origin(xyz=(0.225, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=visor_fabric,
        name="hinge_sleeve",
    )
    visor_panel.visual(
        Box((0.360, 0.014, 0.010)),
        origin=Origin(xyz=(0.225, -0.0105, -0.013)),
        material=visor_fabric,
        name="sleeve_bridge",
    )
    visor_panel.visual(
        Box((0.360, 0.004, 0.004)),
        origin=Origin(xyz=(0.225, -0.176, -0.006)),
        material=visor_fabric,
        name="padded_lower_seam",
    )

    extender = model.part("extender")
    extender.visual(
        mesh_from_cadquery(_extender_leaf_cadquery(), "visor_extender_leaf", tolerance=0.0008),
        material=extender_fabric,
        name="extender_leaf",
    )
    extender.visual(
        Box((0.026, 0.052, 0.004)),
        origin=Origin(xyz=(0.224, 0.000, -0.009)),
        material=dark_plastic,
        name="finger_pull",
    )

    model.articulation(
        "headliner_to_pivot_bracket",
        ArticulationType.FIXED,
        parent=headliner,
        child=pivot_bracket,
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
    )
    model.articulation(
        "headliner_to_retaining_clip",
        ArticulationType.FIXED,
        parent=headliner,
        child=retaining_clip,
        origin=Origin(xyz=(0.442, 0.000, 0.014)),
    )
    model.articulation(
        "secondary_pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_bracket,
        child=swivel_yoke,
        origin=Origin(xyz=(0.000, 0.000, -0.027)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "yoke_to_hinge_rod",
        ArticulationType.FIXED,
        parent=swivel_yoke,
        child=hinge_rod,
        origin=Origin(xyz=(0.046, 0.000, -0.010)),
    )
    model.articulation(
        "visor_drop_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_rod,
        child=visor_panel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "extender_slide",
        ArticulationType.PRISMATIC,
        parent=visor_panel,
        child=extender,
        origin=Origin(xyz=(SLOT_LEFT_X, PANEL_CENTER[1], PANEL_CENTER[2])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headliner = object_model.get_part("headliner")
    retaining_clip = object_model.get_part("retaining_clip")
    hinge_rod = object_model.get_part("hinge_rod")
    visor_panel = object_model.get_part("visor_panel")
    extender = object_model.get_part("extender")

    swivel = object_model.get_articulation("secondary_pivot")
    drop_hinge = object_model.get_articulation("visor_drop_hinge")
    slide = object_model.get_articulation("extender_slide")

    ctx.allow_overlap(
        hinge_rod,
        visor_panel,
        elem_a="rod",
        elem_b="hinge_sleeve",
        reason=(
            "The metal hinge rod is intentionally captured inside the padded "
            "visor hinge sleeve while remaining a separate visible part at the ends."
        ),
    )
    ctx.expect_within(
        hinge_rod,
        visor_panel,
        axes="yz",
        inner_elem="rod",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="hinge rod is captured inside sleeve bore",
    )
    ctx.expect_overlap(
        hinge_rod,
        visor_panel,
        axes="x",
        elem_a="rod",
        elem_b="hinge_sleeve",
        min_overlap=0.30,
        name="hinge sleeve spans the rod",
    )

    ctx.expect_gap(
        headliner,
        visor_panel,
        axis="z",
        positive_elem="roof_panel",
        negative_elem="visor_shell",
        min_gap=0.015,
        name="stowed visor sits below headliner",
    )
    ctx.expect_within(
        hinge_rod,
        retaining_clip,
        axes="yz",
        inner_elem="rod",
        margin=0.001,
        name="rod rests in retaining clip throat",
    )
    ctx.expect_overlap(
        hinge_rod,
        retaining_clip,
        axes="x",
        elem_a="rod",
        min_overlap=0.020,
        name="retaining clip is located at rod end",
    )

    ctx.expect_within(
        extender,
        visor_panel,
        axes="yz",
        inner_elem="extender_leaf",
        outer_elem="visor_shell",
        margin=0.003,
        name="collapsed extender fits within visor sleeve thickness",
    )
    ctx.expect_overlap(
        extender,
        visor_panel,
        axes="x",
        elem_a="extender_leaf",
        elem_b="visor_shell",
        min_overlap=0.20,
        name="collapsed extender remains nested in visor body",
    )

    rest_extender_aabb = ctx.part_world_aabb(extender)
    with ctx.pose({slide: 0.120}):
        extended_extender_aabb = ctx.part_world_aabb(extender)
        ctx.expect_within(
            extender,
            visor_panel,
            axes="yz",
            inner_elem="extender_leaf",
            outer_elem="visor_shell",
            margin=0.003,
            name="extended extender stays centered in sleeve",
        )
        ctx.expect_overlap(
            extender,
            visor_panel,
            axes="x",
            elem_a="extender_leaf",
            elem_b="visor_shell",
            min_overlap=0.10,
            name="extended extender retains insertion in visor body",
        )
    ctx.check(
        "extender slides outward from panel",
        rest_extender_aabb is not None
        and extended_extender_aabb is not None
        and extended_extender_aabb[1][0] > rest_extender_aabb[1][0] + 0.10,
        details=f"rest={rest_extender_aabb}, extended={extended_extender_aabb}",
    )

    rest_panel_aabb = ctx.part_world_aabb(visor_panel)
    with ctx.pose({drop_hinge: 1.25}):
        dropped_panel_aabb = ctx.part_world_aabb(visor_panel)
    ctx.check(
        "visor rotates downward on hinge rod",
        rest_panel_aabb is not None
        and dropped_panel_aabb is not None
        and dropped_panel_aabb[0][2] < rest_panel_aabb[0][2] - 0.08,
        details=f"rest={rest_panel_aabb}, dropped={dropped_panel_aabb}",
    )

    rest_center_y = None
    swivel_center_y = None
    if rest_panel_aabb is not None:
        rest_center_y = 0.5 * (rest_panel_aabb[0][1] + rest_panel_aabb[1][1])
    with ctx.pose({swivel: 1.0}):
        swiveled_panel_aabb = ctx.part_world_aabb(visor_panel)
        if swiveled_panel_aabb is not None:
            swivel_center_y = 0.5 * (swiveled_panel_aabb[0][1] + swiveled_panel_aabb[1][1])
    ctx.check(
        "secondary pivot swivels visor sideways",
        rest_center_y is not None
        and swivel_center_y is not None
        and swivel_center_y > rest_center_y + 0.12,
        details=f"rest_center_y={rest_center_y}, swivel_center_y={swivel_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
