from __future__ import annotations

from math import pi

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


PANEL_W = 0.304
PANEL_D = 0.158
PANEL_T = 0.018
PANEL_TOP_GAP = 0.0045
PANEL_CORNER_R = 0.016
ROD_R = 0.004
ROD_L = 0.318
ROD_Z = -0.010
HINGE_X = 0.012
PANEL_BOTTOM = -(PANEL_TOP_GAP + PANEL_T)
COVER_W = 0.172
COVER_H = 0.072
COVER_T = 0.003
COVER_X = 0.109
COVER_Y = 0.047
CLIP_X = 0.313


def _panel_shape() -> cq.Workplane:
    panel_center_z = PANEL_BOTTOM + (PANEL_T / 2.0)

    body = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_D, PANEL_T)
        .edges("|Z")
        .fillet(PANEL_CORNER_R)
        .translate((PANEL_W / 2.0, PANEL_D / 2.0, panel_center_z))
    )

    recess = (
        cq.Workplane("XY")
        .box(0.188, 0.088, 0.0038)
        .edges("|Z")
        .fillet(0.007)
        .translate((0.195, 0.083, PANEL_BOTTOM + 0.0019))
    )
    return body.cut(recess)


def _cover_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(COVER_W, COVER_H, COVER_T)
        .edges("|Z")
        .fillet(0.005)
        .translate((COVER_W / 2.0, COVER_H / 2.0, COVER_T / 2.0))
    )


def _clip_shape() -> cq.Workplane:
    pad = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.004)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, 0.030))
    )
    body = (
        cq.Workplane("XY")
        .box(0.022, 0.018, 0.036)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, 0.012))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(0.0054)
        .extrude(0.036)
        .translate((-0.018, 0.0, 0.0))
    )
    lower_slot = cq.Workplane("XY").box(0.034, 0.020, 0.014).translate((0.0, 0.0, -0.008))
    return pad.union(body).cut(bore).cut(lower_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="driver_sun_visor")

    model.material("visor_vinyl", rgba=(0.76, 0.71, 0.62, 1.0))
    model.material("visor_cover", rgba=(0.70, 0.66, 0.58, 1.0))
    model.material("trim_plastic", rgba=(0.62, 0.62, 0.64, 1.0))
    model.material("rod_steel", rgba=(0.30, 0.31, 0.33, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.055, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="trim_plastic",
        name="roof_pad",
    )
    bracket.visual(
        Box((0.300, 0.012, 0.004)),
        origin=Origin(xyz=(0.162, 0.0, 0.020)),
        material="trim_plastic",
        name="roof_strip",
    )
    bracket.visual(
        Box((0.018, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="trim_plastic",
        name="drop_leg",
    )
    bracket.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material="trim_plastic",
        name="pivot_ear_0",
    )
    bracket.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material="trim_plastic",
        name="pivot_ear_1",
    )

    armature = model.part("armature")
    armature.visual(
        Cylinder(radius=0.0045, length=0.022),
        origin=Origin(),
        material="rod_steel",
        name="pivot_pin",
    )
    armature.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, ROD_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="rod_steel",
        name="hinge_collar",
    )
    armature.visual(
        Cylinder(radius=ROD_R, length=ROD_L),
        origin=Origin(xyz=(ROD_L / 2.0, 0.0, ROD_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="rod_steel",
        name="hinge_rod",
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_panel_shape(), "visor_panel"),
        material="visor_vinyl",
        name="panel_body",
    )
    panel.visual(
        Box((COVER_W, 0.010, 0.004)),
        origin=Origin(xyz=(COVER_X + (COVER_W / 2.0), 0.042, PANEL_BOTTOM + 0.002)),
        material="visor_vinyl",
        name="cover_rib",
    )

    clip = model.part("clip")
    clip.visual(
        mesh_from_cadquery(_clip_shape(), "visor_clip"),
        material="trim_plastic",
        name="clip_body",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shape(), "visor_cover"),
        material="visor_cover",
        name="cover_panel",
    )

    model.articulation(
        "bracket_to_armature",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=armature,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=10.0, velocity=1.5),
    )
    model.articulation(
        "armature_to_panel",
        ArticulationType.REVOLUTE,
        parent=armature,
        child=panel,
        origin=Origin(xyz=(HINGE_X, 0.0, ROD_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.50, effort=8.0, velocity=1.6),
    )
    model.articulation(
        "bracket_to_clip",
        ArticulationType.FIXED,
        parent=bracket,
        child=clip,
        origin=Origin(xyz=(CLIP_X, 0.0, ROD_Z)),
    )
    model.articulation(
        "panel_to_cover",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=cover,
        origin=Origin(xyz=(COVER_X, COVER_Y, PANEL_BOTTOM)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    clip = object_model.get_part("clip")
    panel = object_model.get_part("panel")
    cover = object_model.get_part("cover")
    side_joint = object_model.get_articulation("bracket_to_armature")
    main_joint = object_model.get_articulation("armature_to_panel")
    cover_joint = object_model.get_articulation("panel_to_cover")

    panel_rest = None
    panel_swung = None
    panel_dropped = None
    cover_rest = None
    cover_open = None

    with ctx.pose({side_joint: 0.0, main_joint: 0.0}):
        panel_rest = ctx.part_element_world_aabb(panel, elem="panel_body")
        cover_rest = ctx.part_element_world_aabb(cover, elem="cover_panel")
        ctx.expect_origin_gap(
            clip,
            bracket,
            axis="x",
            min_gap=0.29,
            max_gap=0.33,
            name="clip is mounted at the visor free end",
        )
        ctx.expect_within(
            cover,
            panel,
            axes="xy",
            margin=0.003,
            elem_a="cover_panel",
            elem_b="panel_body",
            name="cover sits inset within the visor face",
        )
        ctx.expect_contact(
            cover,
            panel,
            elem_a="cover_panel",
            elem_b="cover_rib",
            name="cover is supported by its hinge rib when closed",
        )

    if main_joint.motion_limits is not None and main_joint.motion_limits.upper is not None:
        with ctx.pose({side_joint: 0.0, main_joint: main_joint.motion_limits.upper}):
            panel_dropped = ctx.part_element_world_aabb(panel, elem="panel_body")

    if side_joint.motion_limits is not None and side_joint.motion_limits.upper is not None:
        with ctx.pose({side_joint: side_joint.motion_limits.upper, main_joint: 0.0}):
            panel_swung = ctx.part_element_world_aabb(panel, elem="panel_body")

    if cover_joint.motion_limits is not None and cover_joint.motion_limits.upper is not None:
        with ctx.pose({cover_joint: cover_joint.motion_limits.upper}):
            cover_open = ctx.part_element_world_aabb(cover, elem="cover_panel")

    panel_rest_aabb = ctx.part_element_world_aabb(panel, elem="panel_body")

    ctx.check(
        "visor swings down from the roof hinge",
        panel_rest_aabb is not None
        and panel_dropped is not None
        and panel_dropped[0][2] < panel_rest_aabb[0][2] - 0.10,
        details=f"rest={panel_rest_aabb}, dropped={panel_dropped}",
    )
    ctx.check(
        "visor pivots sideways at the bracket",
        panel_rest is not None
        and panel_swung is not None
        and ((panel_swung[0][1] + panel_swung[1][1]) / 2.0) > ((panel_rest[0][1] + panel_rest[1][1]) / 2.0) + 0.07
        and ((panel_swung[0][0] + panel_swung[1][0]) / 2.0) < ((panel_rest[0][0] + panel_rest[1][0]) / 2.0) - 0.10,
        details=f"rest={panel_rest}, swung={panel_swung}",
    )
    ctx.check(
        "vanity cover opens away from the visor face",
        cover_rest is not None and cover_open is not None and cover_open[0][2] < cover_rest[0][2] - 0.04,
        details=f"rest={cover_rest}, open={cover_open}",
    )

    return ctx.report()


object_model = build_object_model()
