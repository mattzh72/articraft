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


BASE_L = 0.145
BASE_W = 0.106
BASE_H = 0.020
REAR_HUMP_L = 0.058
REAR_HUMP_W = 0.084
REAR_HUMP_H = 0.018
REAR_HUMP_X = -0.032
DIE_BLOCK_L = 0.030
DIE_BLOCK_W = 0.052
DIE_BLOCK_H = 0.007
DIE_BLOCK_X = 0.009

HANDLE_HINGE_X = -0.053
HANDLE_HINGE_Z = 0.046
GUIDE_SLOT_X = 0.056
GUIDE_SLOT_Z = 0.0145
GUIDE_SLOT_DEPTH = 0.036
GUIDE_SLOT_W = 0.086
GUIDE_SLOT_H = 0.007
GUIDE_JOINT_X = 0.0667
FLAP_HINGE_X = -0.008
FLAP_HINGE_Z = -0.002


def _make_base_shell() -> object:
    main = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
    )
    rear_hump = (
        cq.Workplane("XY")
        .box(REAR_HUMP_L, REAR_HUMP_W, REAR_HUMP_H, centered=(True, True, False))
        .translate((REAR_HUMP_X, 0.0, BASE_H))
        .edges("|Z")
        .fillet(0.010)
    )
    die_block = (
        cq.Workplane("XY")
        .box(DIE_BLOCK_L, DIE_BLOCK_W, DIE_BLOCK_H, centered=(True, True, False))
        .translate((DIE_BLOCK_X, 0.0, BASE_H))
        .edges("|Z")
        .fillet(0.003)
    )

    shell = main.union(rear_hump).union(die_block)

    guide_slot = (
        cq.Workplane("XY")
        .box(GUIDE_SLOT_DEPTH, GUIDE_SLOT_W, GUIDE_SLOT_H, centered=(True, True, False))
        .translate((GUIDE_SLOT_X, 0.0, GUIDE_SLOT_Z - (GUIDE_SLOT_H * 0.5)))
    )
    punch_holes = (
        cq.Workplane("XY")
        .pushPoints([(0.007, -0.016), (0.007, 0.016)])
        .circle(0.0036)
        .extrude(BASE_H + REAR_HUMP_H + 0.010)
    )

    return shell.cut(guide_slot).cut(punch_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_desk_hole_punch")

    painted_steel = model.material("painted_steel", rgba=(0.31, 0.35, 0.40, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    ruler_plastic = model.material("ruler_plastic", rgba=(0.82, 0.82, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shell(), "hole_punch_base"), material=painted_steel, name="base_shell")
    for index, y_pos in enumerate((-0.029, 0.029)):
        base.visual(
            Box((0.016, 0.020, 0.012)),
            origin=Origin(xyz=(HANDLE_HINGE_X, y_pos, 0.041)),
            material=painted_steel,
            name=f"rear_post_{index}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(xyz=(HANDLE_HINGE_X, y_pos, HANDLE_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name=f"rear_knuckle_{index}",
        )

    for index, y_pos in enumerate((-0.017, 0.017)):
        base.visual(
            Box((0.012, 0.012, 0.006)),
            origin=Origin(xyz=(FLAP_HINGE_X, y_pos, -0.002)),
            material=dark_plastic,
            name=f"flap_bracket_{index}",
        )
        base.visual(
            Cylinder(radius=0.0027, length=0.010),
            origin=Origin(xyz=(FLAP_HINGE_X, y_pos, FLAP_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name=f"flap_knuckle_{index}",
        )
    base.visual(
        Box((0.003, 0.042, 0.006)),
        origin=Origin(xyz=(0.068, 0.0, GUIDE_SLOT_Z)),
        material=painted_steel,
        name="guide_track",
    )
    base.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.064, 0.018, 0.009)),
        material=painted_steel,
        name="guide_support",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.022, 0.034, 0.014)),
        origin=Origin(xyz=(0.017, 0.0, 0.000)),
        material=satin_metal,
        name="rear_web",
    )
    handle.visual(
        Box((0.098, 0.078, 0.012)),
        origin=Origin(xyz=(0.057, 0.0, -0.004)),
        material=satin_metal,
        name="lever",
    )
    handle.visual(
        Box((0.022, 0.066, 0.010)),
        origin=Origin(xyz=(0.104, 0.0, -0.006)),
        material=satin_metal,
        name="nose",
    )
    handle.visual(
        Box((0.024, 0.050, 0.006)),
        origin=Origin(xyz=(0.052, 0.0, -0.0035)),
        material=satin_metal,
        name="punch_bridge",
    )
    for index, y_pos in enumerate((-0.016, 0.016)):
        handle.visual(
            Cylinder(radius=0.0028, length=0.021),
            origin=Origin(xyz=(0.060, y_pos, -0.017)),
            material=satin_metal,
            name=f"punch_pin_{index}",
        )

    ruler_guide = model.part("ruler_guide")
    ruler_guide.visual(
        Box((0.004, 0.014, 0.004)),
        origin=Origin(xyz=(0.0048, 0.0, 0.0)),
        material=ruler_plastic,
        name="guide_pad",
    )
    ruler_guide.visual(
        Box((0.016, 0.038, 0.005)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=ruler_plastic,
        name="runner",
    )
    ruler_guide.visual(
        Box((0.010, 0.022, 0.005)),
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        material=ruler_plastic,
        name="guide_bridge",
    )
    ruler_guide.visual(
        Box((0.030, 0.052, 0.006)),
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
        material=ruler_plastic,
        name="guide_strip",
    )
    ruler_guide.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(0.048, 0.019, 0.009)),
        material=dark_plastic,
        name="guide_tab",
    )

    waste_flap = model.part("waste_flap")
    waste_flap.visual(
        Cylinder(radius=0.0025, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="flap_barrel",
    )
    waste_flap.visual(
        Box((0.030, 0.046, 0.003)),
        origin=Origin(xyz=(0.015, 0.0, -0.0035)),
        material=dark_plastic,
        name="flap_panel",
    )
    waste_flap.visual(
        Box((0.004, 0.040, 0.005)),
        origin=Origin(xyz=(0.030, 0.0, -0.003)),
        material=dark_plastic,
        name="flap_lip",
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(HANDLE_HINGE_X, 0.0, HANDLE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "base_to_ruler_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=ruler_guide,
        origin=Origin(xyz=(GUIDE_JOINT_X, 0.0, GUIDE_SLOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "base_to_waste_flap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=waste_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    ruler_guide = object_model.get_part("ruler_guide")
    waste_flap = object_model.get_part("waste_flap")

    handle_joint = object_model.get_articulation("base_to_handle")
    guide_joint = object_model.get_articulation("base_to_ruler_guide")
    flap_joint = object_model.get_articulation("base_to_waste_flap")

    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        elem_a="lever",
        elem_b="base_shell",
        min_overlap=0.055,
        name="handle sits over the punch base",
    )
    ctx.expect_gap(
        ruler_guide,
        base,
        axis="x",
        positive_elem="runner",
        negative_elem="base_shell",
        min_gap=0.0,
        max_gap=0.020,
        name="guide runner sits at the front slot opening",
    )
    ctx.expect_gap(
        ruler_guide,
        base,
        axis="x",
        positive_elem="guide_strip",
        negative_elem="base_shell",
        min_gap=0.0,
        max_gap=0.060,
        name="ruler strip projects from the front edge",
    )
    ctx.expect_overlap(
        ruler_guide,
        base,
        axes="yz",
        elem_a="runner",
        elem_b="base_shell",
        min_overlap=0.004,
        name="guide stays aligned with the front slot band",
    )
    ctx.expect_gap(
        base,
        waste_flap,
        axis="z",
        positive_elem="base_shell",
        negative_elem="flap_panel",
        min_gap=0.001,
        max_gap=0.010,
        name="waste flap sits beneath the die area",
    )

    guide_rest = ctx.part_world_position(ruler_guide)
    with ctx.pose({guide_joint: 0.018}):
        ctx.expect_gap(
            ruler_guide,
            base,
            axis="x",
            positive_elem="runner",
            negative_elem="base_shell",
            min_gap=0.0,
            max_gap=0.020,
            name="extended guide runner stays at the slot opening",
        )
        guide_extended = ctx.part_world_position(ruler_guide)
    ctx.check(
        "ruler guide slides transversely",
        guide_rest is not None and guide_extended is not None and guide_extended[1] > guide_rest[1] + 0.015,
        details=f"rest={guide_rest}, extended={guide_extended}",
    )

    handle_closed = ctx.part_element_world_aabb(handle, elem="lever")
    with ctx.pose({handle_joint: math.radians(55.0)}):
        handle_open = ctx.part_element_world_aabb(handle, elem="lever")
    ctx.check(
        "handle opens upward",
        handle_closed is not None
        and handle_open is not None
        and handle_open[1][2] > handle_closed[1][2] + 0.025,
        details=f"closed={handle_closed}, open={handle_open}",
    )

    flap_closed = ctx.part_element_world_aabb(waste_flap, elem="flap_panel")
    with ctx.pose({flap_joint: math.radians(75.0)}):
        flap_open = ctx.part_element_world_aabb(waste_flap, elem="flap_panel")
    ctx.check(
        "waste flap swings downward",
        flap_closed is not None
        and flap_open is not None
        and flap_open[0][2] < flap_closed[0][2] - 0.015,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    return ctx.report()


object_model = build_object_model()
