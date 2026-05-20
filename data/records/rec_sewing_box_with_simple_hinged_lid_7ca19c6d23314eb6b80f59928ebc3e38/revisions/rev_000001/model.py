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


BOX_W = 0.340
BOX_D = 0.240
BOX_H = 0.160
WALL = 0.012
FLOOR = 0.014
LID_T = 0.018
LID_W = 0.356
LID_D = 0.246
HINGE_Y = BOX_D / 2.0 + 0.007
HINGE_Z = BOX_H + LID_T / 2.0
HINGE_R = 0.005


def _body_shell() -> cq.Workplane:
    """Open, thin-walled rectangular sewing-box body with softly eased corners."""
    outer = (
        cq.Workplane("XY")
        .box(BOX_W, BOX_D, BOX_H)
        .translate((0.0, 0.0, BOX_H / 2.0))
        .edges("|Z")
        .fillet(0.006)
    )
    inner = cq.Workplane("XY").box(
        BOX_W - 2.0 * WALL,
        BOX_D - 2.0 * WALL,
        BOX_H + 0.020,
    ).translate((0.0, 0.0, FLOOR + (BOX_H + 0.020) / 2.0))
    return outer.cut(inner)


def _lid_panel() -> cq.Workplane:
    """Plain flat lid panel authored in the hinge frame; it extends forward."""
    rear_y = -0.007
    center_y = rear_y - LID_D / 2.0
    return (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T)
        .translate((0.0, center_y, 0.0))
        .edges("|Z")
        .fillet(0.005)
        .edges(">Z")
        .fillet(0.002)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plain_lid_sewing_box")

    warm_wood = model.material("warm_painted_wood", rgba=(0.72, 0.50, 0.34, 1.0))
    dark_wood = model.material("darker_end_grain", rgba=(0.45, 0.28, 0.16, 1.0))
    interior_cloth = model.material("muted_red_lining", rgba=(0.55, 0.10, 0.13, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.58, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0008),
        material=warm_wood,
        name="body_shell",
    )

    # A shallow fabric-lined floor and low dividers make the box read as a
    # sewing/storage box without adding extra moving controls.
    body.visual(
        Box((BOX_W - 2.0 * WALL - 0.010, BOX_D - 2.0 * WALL - 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR + 0.0015)),
        material=interior_cloth,
        name="floor_lining",
    )
    body.visual(
        Box((BOX_W - 2.0 * WALL, 0.006, 0.050)),
        origin=Origin(xyz=(0.0, -0.020, FLOOR + 0.025)),
        material=dark_wood,
        name="cross_divider",
    )
    body.visual(
        Box((0.006, BOX_D - 2.0 * WALL, 0.050)),
        origin=Origin(xyz=(0.045, 0.0, FLOOR + 0.025)),
        material=dark_wood,
        name="side_divider",
    )

    # Short, tucked hinge hardware: two compact hinges, each with a fixed rear
    # leaf and two outer knuckles on the box side.
    for i, x in enumerate((-0.105, 0.105)):
        body.visual(
            Box((0.054, 0.004, 0.038)),
            origin=Origin(xyz=(x, BOX_D / 2.0 + 0.002, 0.145)),
            material=brass,
            name=f"hinge_leaf_{i}",
        )
        for j, dx in enumerate((-0.0195, 0.0195)):
            body.visual(
                Box((0.014, 0.008, 0.010)),
                origin=Origin(xyz=(x + dx, HINGE_Y - 0.002, HINGE_Z - HINGE_R)),
                material=brass,
                name=f"hinge_bridge_{i}_{j}",
            )
            body.visual(
                Cylinder(radius=HINGE_R, length=0.013),
                origin=Origin(
                    xyz=(x + dx, HINGE_Y, HINGE_Z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=brass,
                name=f"hinge_knuckle_{i}_{j}",
            )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel(), "lid_panel", tolerance=0.0008),
        material=warm_wood,
        name="lid_panel",
    )

    # A simple underside locating lip drops into the body opening with clearance;
    # it keeps the visible top plain while making the closed fit credible.
    lip_w = BOX_W - 2.0 * WALL - 0.014
    lip_d = BOX_D - 2.0 * WALL - 0.014
    lip_t = 0.006
    lip_drop = 0.010
    lip_center_y = -HINGE_Y
    lip_center_z = -LID_T / 2.0 - lip_drop / 2.0
    lid.visual(
        Box((lip_w, lip_t, lip_drop)),
        origin=Origin(xyz=(0.0, lip_center_y + lip_d / 2.0, lip_center_z)),
        material=dark_wood,
        name="rear_lip",
    )
    lid.visual(
        Box((lip_w, lip_t, lip_drop)),
        origin=Origin(xyz=(0.0, lip_center_y - lip_d / 2.0, lip_center_z)),
        material=dark_wood,
        name="front_lip",
    )
    lid.visual(
        Box((lip_t, lip_d, lip_drop)),
        origin=Origin(xyz=(-lip_w / 2.0, lip_center_y, lip_center_z)),
        material=dark_wood,
        name="side_lip_0",
    )
    lid.visual(
        Box((lip_t, lip_d, lip_drop)),
        origin=Origin(xyz=(lip_w / 2.0, lip_center_y, lip_center_z)),
        material=dark_wood,
        name="side_lip_1",
    )

    # Moving hinge leaves and single center knuckles ride with the lid.  The
    # compact knuckles interleave with the fixed outer knuckles without needing
    # a second visible articulation.
    for i, x in enumerate((-0.105, 0.105)):
        lid.visual(
            Box((0.054, 0.026, 0.0025)),
            origin=Origin(xyz=(x, -0.017, LID_T / 2.0 + 0.00125)),
            material=brass,
            name=f"lid_leaf_{i}",
        )
        lid.visual(
            Box((0.022, 0.006, 0.010)),
            origin=Origin(xyz=(x, -0.006, 0.0)),
            material=brass,
            name=f"lid_curl_{i}",
        )
        lid.visual(
            Cylinder(radius=HINGE_R, length=0.020),
            origin=Origin(
                xyz=(x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"lid_knuckle_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="lid_panel",
            negative_elem="body_shell",
            name="closed lid rests on the top rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.18,
            elem_a="lid_panel",
            elem_b="body_shell",
            name="lid covers the rectangular box opening",
        )

    closed_front_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({hinge: 1.2}):
        opened_front_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.010,
            positive_elem="front_lip",
            negative_elem="body_shell",
            name="front lip lifts clear when the lid opens",
        )

    closed_front_lip_z = (
        None
        if closed_front_lip_aabb is None
        else (closed_front_lip_aabb[0][2] + closed_front_lip_aabb[1][2]) / 2.0
    )
    opened_front_lip_z = (
        None
        if opened_front_lip_aabb is None
        else (opened_front_lip_aabb[0][2] + opened_front_lip_aabb[1][2]) / 2.0
    )
    ctx.check(
        "positive hinge angle opens the lid upward",
        closed_front_lip_z is not None
        and opened_front_lip_z is not None
        and opened_front_lip_z > closed_front_lip_z + 0.050,
        details=f"closed_z={closed_front_lip_z}, opened_z={opened_front_lip_z}",
    )

    return ctx.report()


object_model = build_object_model()
