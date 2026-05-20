from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


DEPTH = 0.30
WIDTH = 0.46
BODY_HEIGHT = 0.105
WALL = 0.016
RIM_HEIGHT = 0.006
LID_THICKNESS = 0.018
HINGE_X = -DEPTH / 2.0 - 0.008
HINGE_Z = BODY_HEIGHT + RIM_HEIGHT + LID_THICKNESS / 2.0
LID_DEPTH = 0.310
LID_WIDTH = 0.480
LID_REAR_LOCAL_X = (-DEPTH / 2.0 + 0.005) - HINGE_X
LID_CENTER_LOCAL_X = LID_REAR_LOCAL_X + LID_DEPTH / 2.0


def _rounded_open_body() -> object:
    """One-piece shallow plastic tub with a true open top."""
    return (
        cq.Workplane("XY")
        .box(DEPTH, WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .shell(-WALL)
    )


def _plain_lid_panel() -> object:
    """A single flat panel in the lid frame; the hinge line is local X=0."""
    return (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, LID_THICKNESS)
        .edges("|Z")
        .fillet(0.014)
        .translate((LID_CENTER_LOCAL_X, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    blue_plastic = model.material("deep_blue_plastic", rgba=(0.04, 0.16, 0.32, 1.0))
    lid_plastic = model.material("lid_blue_plastic", rgba=(0.06, 0.27, 0.48, 1.0))
    tray_plastic = model.material("pale_tray_plastic", rgba=(0.68, 0.76, 0.72, 1.0))
    hinge_shadow = model.material("dark_hinge_shadow", rgba=(0.025, 0.030, 0.035, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_open_body(), "rounded_open_body", tolerance=0.001),
        material=blue_plastic,
        name="body_shell",
    )

    # A narrow molded rim supports the closed lid while keeping the body shallow.
    rim_z = BODY_HEIGHT + RIM_HEIGHT / 2.0
    body.visual(
        Box((0.014, WIDTH - 0.010, RIM_HEIGHT)),
        origin=Origin(xyz=(DEPTH / 2.0 - 0.007, 0.0, rim_z)),
        material=blue_plastic,
        name="front_rim",
    )
    body.visual(
        Box((0.014, WIDTH - 0.010, RIM_HEIGHT)),
        origin=Origin(xyz=(-DEPTH / 2.0 + 0.007, 0.0, rim_z)),
        material=blue_plastic,
        name="rear_rim",
    )
    body.visual(
        Box((DEPTH - 0.028, 0.014, RIM_HEIGHT)),
        origin=Origin(xyz=(0.0, WIDTH / 2.0 - 0.007, rim_z)),
        material=blue_plastic,
        name="side_rim_0",
    )
    body.visual(
        Box((DEPTH - 0.028, 0.014, RIM_HEIGHT)),
        origin=Origin(xyz=(0.0, -WIDTH / 2.0 + 0.007, rim_z)),
        material=blue_plastic,
        name="side_rim_1",
    )

    # Fixed internal tackle tray details: low dividers molded into the body.
    tray_z = WALL + 0.002
    body.visual(
        Box((DEPTH - 2.0 * WALL - 0.010, WIDTH - 2.0 * WALL - 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, tray_z)),
        material=tray_plastic,
        name="tray_floor",
    )
    divider_height = 0.048
    divider_z = WALL + divider_height / 2.0 - 0.001
    body.visual(
        Box((DEPTH - 2.0 * WALL - 0.028, 0.006, divider_height)),
        origin=Origin(xyz=(0.012, 0.0, divider_z)),
        material=tray_plastic,
        name="long_divider",
    )
    body.visual(
        Box((0.006, WIDTH / 2.0 - WALL - 0.022, divider_height)),
        origin=Origin(xyz=(-0.055, 0.095, divider_z)),
        material=tray_plastic,
        name="cross_divider_0",
    )
    body.visual(
        Box((0.006, WIDTH / 2.0 - WALL - 0.022, divider_height)),
        origin=Origin(xyz=(0.060, 0.095, divider_z)),
        material=tray_plastic,
        name="cross_divider_1",
    )
    body.visual(
        Box((0.006, WIDTH / 2.0 - WALL - 0.022, divider_height)),
        origin=Origin(xyz=(0.006, -0.095, divider_z)),
        material=tray_plastic,
        name="cross_divider_2",
    )

    # Exposed alternating hinge barrels at the rear edge.  The lid's center
    # knuckle is a child visual, while the two outside knuckles are fixed to the
    # body by small rear leaves.
    hinge_radius = 0.0065
    for index, y in enumerate((-0.150, 0.150)):
        body.visual(
            Box((0.010, 0.110, 0.030)),
            origin=Origin(xyz=(HINGE_X + 0.004, y, HINGE_Z - 0.001)),
            material=hinge_shadow,
            name=f"hinge_leaf_{index}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.110),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=blue_plastic,
            name=f"body_barrel_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_plain_lid_panel(), "plain_lid_panel", tolerance=0.001),
        material=lid_plastic,
        name="lid_panel",
    )
    # Low molded ribs make the plain panel read as a plastic tackle-box lid
    # without adding extra mechanisms.
    rib_z = LID_THICKNESS / 2.0 + 0.0025
    lid.visual(
        Box((LID_DEPTH - 0.060, 0.010, 0.005)),
        origin=Origin(xyz=(LID_CENTER_LOCAL_X + 0.012, 0.0, rib_z)),
        material=lid_plastic,
        name="center_rib",
    )
    lid.visual(
        Box((0.010, LID_WIDTH - 0.070, 0.005)),
        origin=Origin(xyz=(LID_CENTER_LOCAL_X - 0.060, 0.0, rib_z)),
        material=lid_plastic,
        name="cross_rib_0",
    )
    lid.visual(
        Box((0.010, LID_WIDTH - 0.070, 0.005)),
        origin=Origin(xyz=(LID_CENTER_LOCAL_X + 0.075, 0.0, rib_z)),
        material=lid_plastic,
        name="cross_rib_1",
    )
    lid.visual(
        Box((0.046, 0.160, 0.006)),
        origin=Origin(xyz=(0.027, 0.0, -0.003)),
        material=hinge_shadow,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lid_plastic,
        name="lid_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single lid revolute joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim",
            max_gap=0.0015,
            max_penetration=0.0,
            name="closed lid rests on front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="tray_floor",
            min_overlap=0.18,
            name="closed lid covers the shallow tray opening",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim",
            min_gap=0.015,
            name="opened lid separates from the body rim",
        )

    ctx.check(
        "positive hinge angle opens upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
