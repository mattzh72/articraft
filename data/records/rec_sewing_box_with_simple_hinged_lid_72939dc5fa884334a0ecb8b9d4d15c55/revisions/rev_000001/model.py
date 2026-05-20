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


BODY_WIDTH = 0.38
BODY_DEPTH = 0.24
BODY_HEIGHT = 0.10
WALL = 0.012
BOTTOM = 0.010

LID_WIDTH = 0.315
LID_DEPTH = 0.242
LID_THICKNESS = 0.014
HINGE_RADIUS = 0.006
HINGE_Y = BODY_DEPTH / 2.0 + 0.012
HINGE_Z = BODY_HEIGHT + LID_THICKNESS / 2.0 + 0.001


def _body_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate(
        (0.0, 0.0, BODY_HEIGHT / 2.0)
    )
    inner_height = BODY_HEIGHT - BOTTOM + 0.020
    inner = cq.Workplane("XY").box(
        BODY_WIDTH - 2.0 * WALL,
        BODY_DEPTH - 2.0 * WALL,
        inner_height,
    ).translate((0.0, 0.0, BOTTOM + inner_height / 2.0))
    return outer.cut(inner).edges("|Z").fillet(0.004)


def _lid_panel_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(LID_WIDTH, LID_DEPTH, LID_THICKNESS).edges("|Z").fillet(0.003)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plain_lid_sewing_box")

    warm_wood = model.material("warm_wood", rgba=(0.64, 0.41, 0.22, 1.0))
    darker_wood = model.material("darker_lid_wood", rgba=(0.48, 0.29, 0.14, 1.0))
    green_felt = model.material("green_felt_liner", rgba=(0.06, 0.28, 0.17, 1.0))
    brass = model.material("brass_hinge", rgba=(0.86, 0.62, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell", tolerance=0.0008),
        material=warm_wood,
        name="body_shell",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * WALL - 0.014, BODY_DEPTH - 2.0 * WALL - 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM + 0.0015)),
        material=green_felt,
        name="felt_liner",
    )

    # Two small stationary hinge knuckles sit on the rear outside corners, leaving
    # a clear center span for the moving lid knuckle.
    for x_sign, leaf_name, barrel_name in (
        (-1.0, "hinge_leaf_0", "hinge_barrel_0"),
        (1.0, "hinge_leaf_1", "hinge_barrel_1"),
    ):
        x_center = x_sign * 0.168
        leaf_center = x_sign * 0.176
        body.visual(
            Box((0.028, 0.007, 0.018)),
            origin=Origin(xyz=(leaf_center, BODY_DEPTH / 2.0 + 0.0035, HINGE_Z - 0.002)),
            material=brass,
            name=leaf_name,
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.044),
            origin=Origin(xyz=(x_center, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=barrel_name,
        )
    body.visual(
        Cylinder(radius=0.0028, length=0.365),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hinge_pin",
    )

    lid = model.part("lid")
    # The lid part frame is the hinge axis.  The plain panel extends forward
    # from that rear axis and spans most of the box width.
    lid_rear_local_y = (BODY_DEPTH / 2.0 + 0.003) - HINGE_Y
    lid.visual(
        mesh_from_cadquery(_lid_panel_shape(), "lid_panel", tolerance=0.0008),
        origin=Origin(xyz=(0.0, lid_rear_local_y - LID_DEPTH / 2.0, 0.0)),
        material=darker_wood,
        name="lid_panel",
    )
    lid.visual(
        Box((0.240, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, -0.006)),
        material=brass,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.265),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check("single rear lid hinge", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("hinge axis is horizontal across width", tuple(hinge.axis) == (-1.0, 0.0, 0.0))

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        min_gap=0.0005,
        max_gap=0.003,
        name="closed lid sits just above the box rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.20,
        name="plain lid panel spans most of the rectangular body",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The fixed brass hinge pin is intentionally captured inside the lid hinge barrel.",
    )
    ctx.expect_within(
        body,
        lid,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.0002,
        name="hinge pin is centered inside the moving lid barrel",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.25,
        name="hinge pin runs through the lid barrel",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "positive hinge motion opens the lid upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.15,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
