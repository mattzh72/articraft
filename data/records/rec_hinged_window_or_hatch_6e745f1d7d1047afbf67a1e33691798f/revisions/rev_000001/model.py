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


FRAME_W = 0.72
FRAME_D = 0.52
OPEN_W = 0.58
OPEN_D = 0.38
BODY_W = 0.86
BODY_D = 0.66
BODY_TH = 0.018
FRAME_H = 0.030
GASKET_H = 0.006

PANEL_W = 0.76
PANEL_D = 0.54
PANEL_T = 0.025
HINGE_SETBACK = 0.012
HINGE_Y = FRAME_D / 2.0 + HINGE_SETBACK
PANEL_TOP_Z = FRAME_H + GASKET_H + PANEL_T
HINGE_Z_OFFSET = 0.005
HINGE_Z = PANEL_TOP_Z + HINGE_Z_OFFSET


def _rect_ring(width: float, depth: float, inner_width: float, inner_depth: float, height: float, z0: float):
    """A rectangular plate or gasket with a real opening through it."""
    return (
        cq.Workplane("XY")
        .rect(width, depth)
        .rect(inner_width, inner_depth)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _panel_shell():
    """Flush outer panel; its rear edge is clipped forward of the hinge pin."""
    panel_center_y = -(HINGE_SETBACK + PANEL_D / 2.0)
    panel_center_z = -(HINGE_Z_OFFSET + PANEL_T / 2.0)
    return (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_D, PANEL_T)
        .edges("|Z")
        .chamfer(0.003)
        .translate((0.0, panel_center_y, panel_center_z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_body_access_hatch")

    body_paint = model.material("service_body_paint", rgba=(0.18, 0.22, 0.25, 1.0))
    panel_paint = model.material("flush_gray_panel", rgba=(0.44, 0.49, 0.52, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.62, 0.62, 0.58, 1.0))
    gasket_black = model.material("black_rubber_gasket", rgba=(0.015, 0.015, 0.012, 1.0))
    latch_dark = model.material("black_latch_cap", rgba=(0.03, 0.032, 0.030, 1.0))
    latch_mark = model.material("silver_latch_slot", rgba=(0.86, 0.84, 0.76, 1.0))

    frame = model.part("frame")
    body_sheet = _rect_ring(BODY_W, BODY_D, OPEN_W, OPEN_D, BODY_TH, -BODY_TH)
    raised_frame = _rect_ring(FRAME_W, FRAME_D, OPEN_W, OPEN_D, FRAME_H + 0.001, -0.001)
    frame_shell = body_sheet.union(raised_frame)
    frame.visual(
        mesh_from_cadquery(frame_shell, "frame_shell", tolerance=0.0008),
        material=body_paint,
        name="frame_shell",
    )
    gasket = _rect_ring(0.66, 0.46, 0.59, 0.39, GASKET_H + 0.001, FRAME_H - 0.001)
    frame.visual(
        mesh_from_cadquery(gasket, "gasket_ring", tolerance=0.0008),
        material=gasket_black,
        name="gasket_ring",
    )

    # Fixed hinge half: a rear mounting leaf, two raised webs, and the two end
    # barrels. The moving center barrel occupies the gap between them.
    frame.visual(
        Box((PANEL_W, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, FRAME_D / 2.0 + 0.025, FRAME_H + 0.003)),
        material=hinge_metal,
        name="fixed_hinge_leaf",
    )
    for i, x in enumerate((-0.285, 0.285)):
        frame.visual(
            Box((0.17, 0.010, HINGE_Z - FRAME_H)),
            origin=Origin(xyz=(x, HINGE_Y + 0.004, (HINGE_Z + FRAME_H) / 2.0)),
            material=hinge_metal,
            name=f"fixed_hinge_web_{i}",
        )
        frame.visual(
            Cylinder(radius=0.009, length=0.17),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"fixed_hinge_barrel_{i}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_panel_shell(), "panel_shell", tolerance=0.0008),
        material=panel_paint,
        name="panel_shell",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.34),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.34, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -0.012, -0.002)),
        material=hinge_metal,
        name="hinge_leaf",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    latch_y = -(HINGE_SETBACK + PANEL_D - 0.100)
    for index, latch_x in enumerate((-0.275, 0.275)):
        latch = model.part(f"latch_{index}")
        latch.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=latch_dark,
            name="cap",
        )
        latch.visual(
            Box((0.070, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=latch_dark,
            name="turn_bar",
        )
        latch.visual(
            Box((0.048, 0.004, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=latch_mark,
            name="slot_mark",
        )
        latch.visual(
            Cylinder(radius=0.007, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=hinge_metal,
            name="shaft",
        )
        model.articulation(
            f"door_to_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=latch,
            origin=Origin(xyz=(latch_x, latch_y, -HINGE_Z_OFFSET)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("frame_to_door")

    ctx.expect_contact(
        door,
        frame,
        elem_a="panel_shell",
        elem_b="gasket_ring",
        contact_tol=0.0015,
        name="closed door is seated on gasket",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="xy",
        elem_a="panel_shell",
        elem_b="frame_shell",
        min_overlap=0.40,
        name="flush panel overlaps the perimeter frame",
    )

    panel_aabb = ctx.part_element_world_aabb(door, elem="panel_shell")
    hinge_pos = ctx.part_world_position(door)
    ctx.check(
        "panel is clipped behind the hinge pin",
        panel_aabb is not None
        and hinge_pos is not None
        and panel_aabb[1][1] <= hinge_pos[1] - 0.010,
        details=f"panel_aabb={panel_aabb}, hinge={hinge_pos}",
    )

    for index in (0, 1):
        latch = object_model.get_part(f"latch_{index}")
        latch_joint = object_model.get_articulation(f"door_to_latch_{index}")
        ctx.allow_overlap(
            door,
            latch,
            elem_a="panel_shell",
            elem_b="shaft",
            reason="The quarter-turn latch shaft intentionally passes through the hatch panel.",
        )
        ctx.expect_within(
            latch,
            door,
            axes="xy",
            inner_elem="shaft",
            outer_elem="panel_shell",
            margin=0.001,
            name=f"latch_{index} shaft is centered in the panel hole",
        )
        ctx.expect_gap(
            latch,
            door,
            axis="z",
            positive_elem="shaft",
            negative_elem="panel_shell",
            max_penetration=0.031,
            name=f"latch_{index} shaft passes through only the panel thickness",
        )

        closed_aabb = ctx.part_world_aabb(latch)
        with ctx.pose({latch_joint: math.pi / 2.0}):
            turned_aabb = ctx.part_world_aabb(latch)
        ctx.check(
            f"latch_{index} quarter turn rotates the handle",
            closed_aabb is not None
            and turned_aabb is not None
            and (closed_aabb[1][0] - closed_aabb[0][0])
            > (closed_aabb[1][1] - closed_aabb[0][1])
            and (turned_aabb[1][1] - turned_aabb[0][1])
            > (turned_aabb[1][0] - turned_aabb[0][0]),
            details=f"closed={closed_aabb}, turned={turned_aabb}",
        )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(door)
        open_hinge_pos = ctx.part_world_position(door)
    ctx.check(
        "door opens upward about the rear hinge line",
        rest_aabb is not None
        and open_aabb is not None
        and hinge_pos is not None
        and open_hinge_pos is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.20
        and abs(open_hinge_pos[1] - hinge_pos[1]) < 1e-6
        and abs(open_hinge_pos[2] - hinge_pos[2]) < 1e-6,
        details=f"rest={rest_aabb}, open={open_aabb}, hinge={hinge_pos}, open_hinge={open_hinge_pos}",
    )

    return ctx.report()


object_model = build_object_model()
