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


DEPTH = 0.130
WIDTH = 0.380
HEIGHT = 0.340
WALL = 0.018

DOOR_W = 0.340
DOOR_H = 0.300
DOOR_T = 0.008
HINGE_R = 0.004
HINGE_X = DEPTH + 0.010
HINGE_Z = 0.024
PANEL_BOTTOM = HINGE_R + 0.002

SLOT_W = 0.255
SLOT_H = 0.030
SLOT_CENTER_FROM_PANEL_BOTTOM = 0.198
SLOT_CENTER_Z = PANEL_BOTTOM + SLOT_CENTER_FROM_PANEL_BOTTOM
SLOT_TOP_Z = SLOT_CENTER_Z + SLOT_H / 2.0

FLAP_W = 0.292
FLAP_H = 0.074
FLAP_T = 0.004
FLAP_HINGE_Z = SLOT_TOP_Z + 0.018
FLAP_HINGE_X = DOOR_T + 0.007


def _body_shell() -> cq.Workplane:
    """One-piece open-front sheet-metal box with real side/top/bottom walls."""
    outer = cq.Workplane("XY").box(DEPTH, WIDTH, HEIGHT).translate(
        (DEPTH / 2.0, 0.0, HEIGHT / 2.0)
    )
    cutter = cq.Workplane("XY").box(DEPTH + 0.050 - WALL, WIDTH - 2.0 * WALL, HEIGHT - 2.0 * WALL).translate(
        ((WALL + DEPTH + 0.050) / 2.0, 0.0, HEIGHT / 2.0)
    )
    shell = outer.cut(cutter)
    return shell.edges("|X").fillet(0.003)


def _door_panel() -> cq.Workplane:
    """Flat access door as one continuous plate, cut through by the letter slot."""
    panel = cq.Workplane("XY").box(DOOR_T, DOOR_W, DOOR_H)
    slot_z = SLOT_CENTER_FROM_PANEL_BOTTOM - DOOR_H / 2.0
    slot_cutter = cq.Workplane("XY").box(DOOR_T * 3.0, SLOT_W, SLOT_H).translate(
        (0.0, 0.0, slot_z)
    )
    panel = panel.cut(slot_cutter)
    return panel.edges("|X").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_mailbox")

    galvanized = model.material("galvanized_silver", rgba=(0.62, 0.66, 0.67, 1.0))
    darker_metal = model.material("shadowed_grey_metal", rgba=(0.31, 0.34, 0.35, 1.0))
    dark_cavity = model.material("dark_mailbox_interior", rgba=(0.025, 0.027, 0.028, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0008, angular_tolerance=0.08),
        material=galvanized,
        name="body_shell",
    )
    # A dark rear plane, recessed in the hollow box, makes the through-slot read open.
    body.visual(
        Box((0.003, WIDTH - 2.0 * WALL - 0.010, HEIGHT - 2.0 * WALL - 0.020)),
        origin=Origin(xyz=(WALL + 0.0015, 0.0, HEIGHT / 2.0)),
        material=dark_cavity,
        name="interior_shadow",
    )
    # Wall-mounting ears are welded to the back panel, with shallow screw heads.
    for z, name in ((HEIGHT + 0.018, "top_mount"), (-0.018, "bottom_mount")):
        body.visual(
            Box((0.006, 0.170, 0.050)),
            origin=Origin(xyz=(0.003, 0.0, z)),
            material=galvanized,
            name=name,
        )
        body.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.007, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_metal,
            name=f"{name}_screw",
        )
    body.visual(
        Box((0.020, DOOR_W, 0.004)),
        origin=Origin(xyz=(DEPTH + 0.003, 0.0, HINGE_Z - HINGE_R - 0.002)),
        material=galvanized,
        name="lower_hinge_mount",
    )

    front_door = model.part("front_door")
    front_door.visual(
        mesh_from_cadquery(_door_panel(), "front_door_panel", tolerance=0.0006, angular_tolerance=0.08),
        origin=Origin(xyz=(DOOR_T / 2.0, 0.0, PANEL_BOTTOM + DOOR_H / 2.0)),
        material=galvanized,
        name="door_panel",
    )
    # Raised folded seams around the access door keep it visibly stiff and sheet-metal-like.
    front_door.visual(
        Box((0.004, DOOR_W + 0.010, 0.012)),
        origin=Origin(xyz=(DOOR_T + 0.002, 0.0, PANEL_BOTTOM + DOOR_H - 0.006)),
        material=darker_metal,
        name="top_fold",
    )
    front_door.visual(
        Box((0.004, DOOR_W + 0.010, 0.012)),
        origin=Origin(xyz=(DOOR_T + 0.002, 0.0, PANEL_BOTTOM + 0.006)),
        material=darker_metal,
        name="bottom_fold",
    )
    for y, name in ((-(DOOR_W / 2.0 + 0.003), "side_fold_0"), (DOOR_W / 2.0 + 0.003, "side_fold_1")):
        front_door.visual(
            Box((0.004, 0.012, DOOR_H)),
            origin=Origin(xyz=(DOOR_T + 0.002, y, PANEL_BOTTOM + DOOR_H / 2.0)),
            material=darker_metal,
            name=name,
        )
    # The lower hinge barrel is welded to the door by a narrow leaf.
    front_door.visual(
        Cylinder(radius=HINGE_R, length=DOOR_W * 0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_metal,
        name="lower_hinge_barrel",
    )
    front_door.visual(
        Box((0.004, DOOR_W * 0.82, 0.016)),
        origin=Origin(xyz=(DOOR_T / 2.0, 0.0, 0.005)),
        material=darker_metal,
        name="lower_hinge_leaf",
    )
    # Narrow fixed rain hood over the slot, protruding beyond the top-hinged flap.
    front_door.visual(
        Box((0.062, SLOT_W + 0.055, 0.005)),
        origin=Origin(
            xyz=(DOOR_T + 0.028, 0.0, FLAP_HINGE_Z + 0.024),
            rpy=(0.0, -0.26, 0.0),
        ),
        material=galvanized,
        name="rain_hood",
    )
    # Small hinge cheek blocks at the ends of the slot flap pin, tucked under the hood.
    for y, name in ((-(FLAP_W / 2.0 + 0.009), "slot_hinge_bracket_0"), (FLAP_W / 2.0 + 0.009, "slot_hinge_bracket_1")):
        front_door.visual(
            Box((0.012, 0.018, 0.026)),
            origin=Origin(xyz=(FLAP_HINGE_X - 0.002, y, FLAP_HINGE_Z - 0.006)),
            material=darker_metal,
            name=name,
        )
    # Small rubber bumpers receive the flap at the lower corners.
    for y, name in ((-0.118, "slot_bumper_0"), (0.118, "slot_bumper_1")):
        front_door.visual(
            Box((0.004, 0.018, 0.010)),
            origin=Origin(xyz=(DOOR_T + 0.001, y, SLOT_CENTER_Z - 0.046)),
            material=rubber,
            name=name,
        )

    slot_flap = model.part("slot_flap")
    slot_flap.visual(
        Box((FLAP_T, FLAP_W, FLAP_H)),
        origin=Origin(xyz=(0.004, 0.0, -FLAP_H / 2.0)),
        material=galvanized,
        name="flap_panel",
    )
    slot_flap.visual(
        Cylinder(radius=0.0032, length=FLAP_W),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_metal,
        name="flap_hinge_barrel",
    )

    model.articulation(
        "lower_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.40),
    )

    model.articulation(
        "slot_hinge",
        ArticulationType.REVOLUTE,
        parent=front_door,
        child=slot_flap,
        origin=Origin(xyz=(FLAP_HINGE_X, 0.0, FLAP_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_door = object_model.get_part("front_door")
    slot_flap = object_model.get_part("slot_flap")
    lower_hinge = object_model.get_articulation("lower_hinge")
    slot_hinge = object_model.get_articulation("slot_hinge")

    ctx.expect_gap(
        front_door,
        body,
        axis="x",
        min_gap=0.004,
        positive_elem="door_panel",
        negative_elem="body_shell",
        name="closed access door stands proud of body shell",
    )
    ctx.expect_overlap(
        front_door,
        body,
        axes="yz",
        min_overlap=0.24,
        elem_a="door_panel",
        elem_b="body_shell",
        name="flat front door covers the mailbox opening",
    )
    ctx.expect_gap(
        slot_flap,
        front_door,
        axis="x",
        min_gap=0.004,
        positive_elem="flap_panel",
        negative_elem="door_panel",
        name="slot flap is separate and proud of access door",
    )
    ctx.expect_overlap(
        slot_flap,
        front_door,
        axes="yz",
        min_overlap=0.050,
        elem_a="flap_panel",
        elem_b="door_panel",
        name="slot flap covers the letter-slot area",
    )

    closed_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    with ctx.pose({lower_hinge: 1.10}):
        open_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    ctx.check(
        "front door rotates down and outward on lower hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.12
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.08,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(slot_flap, elem="flap_panel")
    with ctx.pose({slot_hinge: 0.90}):
        open_flap_aabb = ctx.part_element_world_aabb(slot_flap, elem="flap_panel")
    ctx.check(
        "slot flap lifts outward from upper hinge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.035,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
