from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SAFE_WIDTH = 0.42
SAFE_DEPTH = 0.32
SAFE_HEIGHT = 0.72
SHELL = 0.035
BACK_WALL = 0.03
FRAME_DEPTH = 0.03

OPENING_WIDTH = 0.324
OPENING_HEIGHT = 0.58
FRAME_SIDE = (SAFE_WIDTH - OPENING_WIDTH) / 2.0
FRAME_TOP = (SAFE_HEIGHT - OPENING_HEIGHT) / 2.0

DOOR_WIDTH = 0.35
DOOR_HEIGHT = 0.67
DOOR_THICKNESS = 0.045
DOOR_GAP = 0.002
DOOR_HINGE_REVEAL = 0.012
DOOR_PANEL_WIDTH = DOOR_WIDTH - DOOR_HINGE_REVEAL
DOOR_PANEL_CENTER_X = DOOR_HINGE_REVEAL + DOOR_PANEL_WIDTH / 2.0

TRAY_WIDTH = 0.312
TRAY_DEPTH = 0.18
TRAY_HEIGHT = 0.04
TRAY_TRAVEL = 0.095

TRAY_CLOSED_CENTER = (0.0, -0.03, -0.13)
GUIDE_WIDTH = 0.022
GUIDE_DEPTH = 0.20
GUIDE_HEIGHT = 0.010
GUIDE_CENTER_Z = TRAY_CLOSED_CENTER[2] - TRAY_HEIGHT / 2.0 - GUIDE_HEIGHT / 2.0
HINGE_RADIUS = 0.008
BODY_KNUCKLE_LENGTH = 0.20
DOOR_KNUCKLE_LENGTH = 0.18
HINGE_CENTER_Y = SAFE_DEPTH / 2.0 + DOOR_GAP + 0.011


def _add_box(part, name, size, center, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="document_wall_safe")

    body_finish = model.material("body_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    door_finish = model.material("door_finish", rgba=(0.33, 0.34, 0.36, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.56, 0.57, 0.60, 1.0))

    body = model.part("body")
    _add_box(
        body,
        "back_wall",
        (SAFE_WIDTH, BACK_WALL, SAFE_HEIGHT),
        (0.0, -SAFE_DEPTH / 2.0 + BACK_WALL / 2.0, 0.0),
        body_finish,
    )
    _add_box(
        body,
        "left_wall",
        (SHELL, SAFE_DEPTH - BACK_WALL, SAFE_HEIGHT),
        (-SAFE_WIDTH / 2.0 + SHELL / 2.0, (SAFE_DEPTH - BACK_WALL) / 2.0 - SAFE_DEPTH / 2.0 + BACK_WALL, 0.0),
        body_finish,
    )
    _add_box(
        body,
        "right_wall",
        (SHELL, SAFE_DEPTH - BACK_WALL, SAFE_HEIGHT),
        (SAFE_WIDTH / 2.0 - SHELL / 2.0, (SAFE_DEPTH - BACK_WALL) / 2.0 - SAFE_DEPTH / 2.0 + BACK_WALL, 0.0),
        body_finish,
    )
    _add_box(
        body,
        "floor",
        (SAFE_WIDTH - 2.0 * SHELL, SAFE_DEPTH - BACK_WALL, SHELL),
        (0.0, (SAFE_DEPTH - BACK_WALL) / 2.0 - SAFE_DEPTH / 2.0 + BACK_WALL, -SAFE_HEIGHT / 2.0 + SHELL / 2.0),
        body_finish,
    )
    _add_box(
        body,
        "roof",
        (SAFE_WIDTH - 2.0 * SHELL, SAFE_DEPTH - BACK_WALL, SHELL),
        (0.0, (SAFE_DEPTH - BACK_WALL) / 2.0 - SAFE_DEPTH / 2.0 + BACK_WALL, SAFE_HEIGHT / 2.0 - SHELL / 2.0),
        body_finish,
    )
    _add_box(
        body,
        "left_jamb",
        (FRAME_SIDE, FRAME_DEPTH, OPENING_HEIGHT),
        (-OPENING_WIDTH / 2.0 - FRAME_SIDE / 2.0, SAFE_DEPTH / 2.0 - FRAME_DEPTH / 2.0, 0.0),
        frame_finish,
    )
    _add_box(
        body,
        "right_jamb",
        (FRAME_SIDE, FRAME_DEPTH, OPENING_HEIGHT),
        (OPENING_WIDTH / 2.0 + FRAME_SIDE / 2.0, SAFE_DEPTH / 2.0 - FRAME_DEPTH / 2.0, 0.0),
        frame_finish,
    )
    _add_box(
        body,
        "top_frame",
        (OPENING_WIDTH, FRAME_DEPTH, FRAME_TOP),
        (0.0, SAFE_DEPTH / 2.0 - FRAME_DEPTH / 2.0, OPENING_HEIGHT / 2.0 + FRAME_TOP / 2.0),
        frame_finish,
    )
    _add_box(
        body,
        "bottom_frame",
        (OPENING_WIDTH, FRAME_DEPTH, FRAME_TOP),
        (0.0, SAFE_DEPTH / 2.0 - FRAME_DEPTH / 2.0, -OPENING_HEIGHT / 2.0 - FRAME_TOP / 2.0),
        frame_finish,
    )
    _add_box(
        body,
        "left_guide",
        (GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT),
        (-(SAFE_WIDTH / 2.0 - SHELL) + GUIDE_WIDTH / 2.0, -0.02, GUIDE_CENTER_Z),
        steel_finish,
    )
    _add_box(
        body,
        "right_guide",
        (GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT),
        ((SAFE_WIDTH / 2.0 - SHELL) - GUIDE_WIDTH / 2.0, -0.02, GUIDE_CENTER_Z),
        steel_finish,
    )
    _add_box(
        body,
        "guide_stop",
        (OPENING_WIDTH - 0.02, 0.010, 0.05),
        (0.0, -0.125, TRAY_CLOSED_CENTER[2] - 0.002),
        frame_finish,
    )
    _add_box(
        body,
        "hinge_plate",
        (0.014, 0.010, BODY_KNUCKLE_LENGTH + 0.08),
        (-DOOR_WIDTH / 2.0 + 0.001, SAFE_DEPTH / 2.0, 0.0),
        steel_finish,
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, HINGE_CENTER_Y, 0.0)),
        material=steel_finish,
        name="hinge_knuckle",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_PANEL_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, DOOR_THICKNESS / 2.0, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_PANEL_WIDTH - 0.034, 0.018, DOOR_HEIGHT - 0.08)),
        origin=Origin(xyz=(DOOR_PANEL_CENTER_X, 0.011, 0.0)),
        material=dark_steel,
        name="inner_plate",
    )
    door.visual(
        Box((0.006, 0.016, DOOR_HEIGHT - 0.02)),
        origin=Origin(xyz=(0.009, 0.011, 0.0)),
        material=steel_finish,
        name="hinge_stile",
    )
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.011, BODY_KNUCKLE_LENGTH / 2.0 + DOOR_KNUCKLE_LENGTH / 2.0),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel_finish,
        name="hinge_knuckle_top",
    )
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=DOOR_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.011, -(BODY_KNUCKLE_LENGTH / 2.0 + DOOR_KNUCKLE_LENGTH / 2.0)),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=steel_finish,
        name="hinge_knuckle_bottom",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.037, length=0.012),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="dial_cap",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="spindle",
    )
    handle.visual(
        Box((0.090, 0.014, 0.012)),
        origin=Origin(xyz=(0.045, 0.032, 0.0)),
        material=steel_finish,
        name="lever",
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.089, 0.032, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="grip",
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=tray_finish,
        name="tray_bottom",
    )
    tray.visual(
        Box((0.008, TRAY_DEPTH, 0.032)),
        origin=Origin(xyz=(-0.146, 0.0, -0.004)),
        material=tray_finish,
        name="left_wall",
    )
    tray.visual(
        Box((0.008, TRAY_DEPTH, 0.032)),
        origin=Origin(xyz=(0.146, 0.0, -0.004)),
        material=tray_finish,
        name="right_wall",
    )
    tray.visual(
        Box((TRAY_WIDTH - 0.024, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, -TRAY_DEPTH / 2.0 + 0.004, -0.004)),
        material=tray_finish,
        name="rear_wall",
    )
    tray.visual(
        Box((TRAY_WIDTH - 0.024, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH / 2.0 - 0.005, -0.008)),
        material=tray_finish,
        name="front_lip",
    )

    body_to_door = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, SAFE_DEPTH / 2.0 + DOOR_GAP, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.8,
        ),
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.225, DOOR_THICKNESS, 0.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.225, DOOR_THICKNESS, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.9,
            upper=0.9,
        ),
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=TRAY_CLOSED_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")

    door_hinge = object_model.get_articulation("body_to_door")
    tray_slide = object_model.get_articulation("body_to_tray")
    door_limits = door_hinge.motion_limits
    tray_limits = tray_slide.motion_limits

    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="right_jamb",
        max_gap=0.004,
        max_penetration=0.0,
        name="door panel sits just proud of the front frame",
    )
    ctx.expect_origin_distance(
        dial,
        handle,
        axes="x",
        max_dist=0.002,
        name="dial and handle stay on the same vertical hardware line",
    )
    ctx.expect_origin_gap(
        dial,
        handle,
        axis="z",
        min_gap=0.12,
        name="dial sits above the lever handle",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="tray_bottom",
        elem_b="left_guide",
        name="tray bottom rests on the left guide",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="tray_bottom",
        elem_b="right_guide",
        name="tray bottom rests on the right guide",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="tray_bottom",
        elem_b="left_guide",
        min_overlap=0.14,
        name="closed tray remains engaged on the left guide",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="tray_bottom",
        elem_b="right_guide",
        min_overlap=0.14,
        name="closed tray remains engaged on the right guide",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    closed_tray_pos = ctx.part_world_position(tray)

    if door_limits is not None and tray_limits is not None and door_limits.upper is not None and tray_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper, tray_slide: tray_limits.upper}):
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                elem_a="tray_bottom",
                elem_b="left_guide",
                min_overlap=0.07,
                name="extended tray still stays retained on the left guide",
            )
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                elem_a="tray_bottom",
                elem_b="right_guide",
                min_overlap=0.07,
                name="extended tray still stays retained on the right guide",
            )
            open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
            extended_tray_pos = ctx.part_world_position(tray)

        ctx.check(
            "door opens outward from the front opening",
            closed_panel_aabb is not None
            and open_panel_aabb is not None
            and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.18,
            details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
        )
        ctx.check(
            "tray slides outward when extended",
            closed_tray_pos is not None
            and extended_tray_pos is not None
            and extended_tray_pos[1] > closed_tray_pos[1] + 0.08,
            details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
        )

    return ctx.report()


object_model = build_object_model()
