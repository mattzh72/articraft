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


BODY_DEPTH = 0.34
BODY_WIDTH = 0.26
BODY_HEIGHT = 0.13
WALL = 0.026
BOTTOM = 0.022

LID_DEPTH = 0.35
LID_WIDTH = 0.272
LID_THICKNESS = 0.024
LID_FRAME_RAIL = 0.038
LID_FRAME_HEIGHT = 0.014
HINGE_Z = BODY_HEIGHT + LID_THICKNESS / 2.0


def _open_wooden_box_shell() -> cq.Workplane:
    """Continuous thick-walled tray, open at the top like a sewing box."""
    outer = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT).translate(
        (BODY_DEPTH / 2.0, 0.0, BODY_HEIGHT / 2.0)
    )
    inner = cq.Workplane("XY").box(
        BODY_DEPTH - 2.0 * WALL,
        BODY_WIDTH - 2.0 * WALL,
        BODY_HEIGHT,
    ).translate((BODY_DEPTH / 2.0, 0.0, BOTTOM + BODY_HEIGHT / 2.0))
    shell = outer.cut(inner)
    return shell


def _framed_lid() -> cq.Workplane:
    """Single moving lid: a plain recessed panel surrounded by a heavy frame."""
    base = cq.Workplane("XY").box(LID_DEPTH, LID_WIDTH, LID_THICKNESS).translate(
        (LID_DEPTH / 2.0, 0.0, 0.0)
    )
    rail_z = LID_THICKNESS / 2.0 + LID_FRAME_HEIGHT / 2.0
    rear_rail = cq.Workplane("XY").box(
        LID_FRAME_RAIL, LID_WIDTH, LID_FRAME_HEIGHT
    ).translate((LID_FRAME_RAIL / 2.0, 0.0, rail_z))
    front_rail = cq.Workplane("XY").box(
        LID_FRAME_RAIL, LID_WIDTH, LID_FRAME_HEIGHT
    ).translate((LID_DEPTH - LID_FRAME_RAIL / 2.0, 0.0, rail_z))
    side_rail_a = cq.Workplane("XY").box(
        LID_DEPTH - 2.0 * LID_FRAME_RAIL,
        LID_FRAME_RAIL,
        LID_FRAME_HEIGHT,
    ).translate(
        (
            LID_DEPTH / 2.0,
            LID_WIDTH / 2.0 - LID_FRAME_RAIL / 2.0,
            rail_z,
        )
    )
    side_rail_b = cq.Workplane("XY").box(
        LID_DEPTH - 2.0 * LID_FRAME_RAIL,
        LID_FRAME_RAIL,
        LID_FRAME_HEIGHT,
    ).translate(
        (
            LID_DEPTH / 2.0,
            -LID_WIDTH / 2.0 + LID_FRAME_RAIL / 2.0,
            rail_z,
        )
    )
    return base.union(rear_rail).union(front_rail).union(side_rail_a).union(side_rail_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    walnut = model.material("warm_walnut", rgba=(0.48, 0.25, 0.11, 1.0))
    end_grain = model.material("dark_end_grain", rgba=(0.30, 0.14, 0.06, 1.0))
    brass = model.material("brass_hinge", rgba=(0.90, 0.68, 0.26, 1.0))
    felt = model.material("green_felt", rgba=(0.05, 0.30, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_open_wooden_box_shell(), "body_shell", tolerance=0.0008),
        material=walnut,
        name="body_shell",
    )
    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL - 0.012, BODY_WIDTH - 2.0 * WALL - 0.012, 0.003)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, 0.0, BOTTOM + 0.0015)),
        material=felt,
        name="felt_liner",
    )
    body.visual(
        Box((BODY_DEPTH, 0.012, 0.020)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, BODY_WIDTH / 2.0 + 0.006, 0.055)),
        material=end_grain,
        name="side_band_0",
    )
    body.visual(
        Box((BODY_DEPTH, 0.012, 0.020)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0, -BODY_WIDTH / 2.0 - 0.006, 0.055)),
        material=end_grain,
        name="side_band_1",
    )

    # Alternating rear hinge knuckles are split between the fixed body and the
    # moving lid, leaving visible gaps so the single revolute joint is credible.
    parent_knuckles = [(-0.096, 0.050), (0.036, 0.050)]
    for index, (y_center, length) in enumerate(parent_knuckles):
        body.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(
                xyz=(-0.007, y_center, HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"body_knuckle_{index}",
        )
        body.visual(
            Box((0.008, length, 0.018)),
            origin=Origin(xyz=(-0.004, y_center, BODY_HEIGHT + 0.004)),
            material=brass,
            name=f"body_hinge_leaf_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_framed_lid(), "lid_shell", tolerance=0.0008),
        material=walnut,
        name="lid_shell",
    )

    child_knuckles = [(-0.030, 0.058), (0.101, 0.042)]
    for index, (y_center, length) in enumerate(child_knuckles):
        lid.visual(
            Cylinder(radius=0.0074, length=length),
            origin=Origin(
                xyz=(-0.007, y_center, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"lid_knuckle_{index}",
        )
        lid.visual(
            Box((0.024, length, 0.012)),
            origin=Origin(xyz=(0.006, y_center, 0.0)),
            material=brass,
            name=f"lid_hinge_leaf_{index}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
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
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid rests on the thick top rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.20,
            name="lid covers the rectangular box body",
        )

    closed_aabb = None
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)

    open_aabb = None
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "rear hinge lifts the front of the lid",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.18,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
