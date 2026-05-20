from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_swing_service_tailgate")

    painted = model.material("service_white", rgba=(0.86, 0.88, 0.84, 1.0))
    panel_paint = model.material("tailgate_white", rgba=(0.82, 0.84, 0.80, 1.0))
    liner = model.material("black_bed_liner", rgba=(0.04, 0.045, 0.045, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.53, 1.0))
    red = model.material("tail_lamp_red", rgba=(0.75, 0.03, 0.02, 1.0))

    hinge_x = -1.23
    hinge_y = -0.78
    hinge_z = 0.62

    body = model.part("service_body")
    body.visual(
        Box((2.50, 1.82, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=liner,
        name="bed_floor",
    )
    body.visual(
        Box((2.30, 0.25, 0.60)),
        origin=Origin(xyz=(0.0, -0.98, 0.90)),
        material=painted,
        name="side_box_0",
    )
    body.visual(
        Box((2.30, 0.25, 0.60)),
        origin=Origin(xyz=(0.0, 0.98, 0.90)),
        material=painted,
        name="side_box_1",
    )
    body.visual(
        Box((0.14, 1.82, 0.60)),
        origin=Origin(xyz=(1.18, 0.0, 0.90)),
        material=painted,
        name="front_bulkhead",
    )
    body.visual(
        Box((0.18, 0.22, 0.76)),
        origin=Origin(xyz=(-1.20, -0.94, 0.98)),
        material=painted,
        name="hinge_stub",
    )
    body.visual(
        Box((0.18, 0.22, 0.76)),
        origin=Origin(xyz=(-1.20, 0.94, 0.98)),
        material=painted,
        name="latch_stub",
    )
    body.visual(
        Box((0.16, 1.52, 0.08)),
        origin=Origin(xyz=(-1.20, 0.0, 0.54)),
        material=steel,
        name="rear_sill",
    )
    body.visual(
        Box((0.070, 0.100, 0.20)),
        origin=Origin(xyz=(hinge_x, -0.805, hinge_z + 0.36)),
        material=steel,
        name="hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.20),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + 0.36)),
        material=steel,
        name="hinge_barrel",
    )
    body.visual(
        Box((0.070, 0.020, 0.18)),
        origin=Origin(xyz=(-1.23, 0.821, 1.00)),
        material=steel,
        name="latch_receiver",
    )
    body.visual(
        Box((0.012, 0.080, 0.12)),
        origin=Origin(xyz=(-1.295, -0.94, 1.18)),
        material=red,
        name="tail_lamp_0",
    )
    body.visual(
        Box((0.012, 0.080, 0.12)),
        origin=Origin(xyz=(-1.295, 0.94, 1.18)),
        material=red,
        name="tail_lamp_1",
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.070, 1.45, 0.68)),
        origin=Origin(xyz=(0.0, 0.80, 0.36)),
        material=panel_paint,
        name="tailgate_panel",
    )
    tailgate.visual(
        Box((0.012, 1.34, 0.035)),
        origin=Origin(xyz=(-0.041, 0.82, 0.665)),
        material=painted,
        name="top_raised_rib",
    )
    tailgate.visual(
        Box((0.012, 1.34, 0.035)),
        origin=Origin(xyz=(-0.041, 0.82, 0.055)),
        material=painted,
        name="bottom_raised_rib",
    )
    tailgate.visual(
        Box((0.012, 0.035, 0.52)),
        origin=Origin(xyz=(-0.041, 0.20, 0.36)),
        material=painted,
        name="hinge_side_rib",
    )
    tailgate.visual(
        Box((0.012, 0.035, 0.52)),
        origin=Origin(xyz=(-0.041, 1.40, 0.36)),
        material=painted,
        name="free_side_rib",
    )
    tailgate.visual(
        Box((0.080, 0.045, 0.68)),
        origin=Origin(xyz=(0.0, 1.525, 0.36)),
        material=panel_paint,
        name="free_edge_cap",
    )
    tailgate.visual(
        Box((0.080, 1.47, 0.040)),
        origin=Origin(xyz=(0.0, 0.81, 0.705)),
        material=rubber,
        name="top_cap",
    )
    tailgate.visual(
        Box((0.070, 0.090, 0.24)),
        origin=Origin(xyz=(0.0, 0.045, 0.13)),
        material=steel,
        name="lower_hinge_leaf",
    )
    tailgate.visual(
        Box((0.070, 0.090, 0.24)),
        origin=Origin(xyz=(0.0, 0.045, 0.59)),
        material=steel,
        name="upper_hinge_leaf",
    )
    tailgate.visual(
        Cylinder(radius=0.026, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=steel,
        name="lower_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.026, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        material=steel,
        name="upper_hinge_barrel",
    )
    tailgate.visual(
        Box((0.014, 0.18, 0.16)),
        origin=Origin(xyz=(-0.043, 1.37, 0.38)),
        material=steel,
        name="handle_mount_pad",
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="latch_boss",
    )
    handle.visual(
        Box((0.035, 0.045, 0.30)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0)),
        material=rubber,
        name="handle_lever",
    )
    handle.visual(
        Box((0.045, 0.060, 0.060)),
        origin=Origin(xyz=(-0.035, 0.0, 0.18)),
        material=rubber,
        name="handle_tip",
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tailgate,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=handle,
        origin=Origin(xyz=(-0.0675, 1.37, 0.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("service_body")
    tailgate = object_model.get_part("tailgate")
    handle = object_model.get_part("latch_handle")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_overlap(
        tailgate,
        body,
        axes="z",
        elem_a="tailgate_panel",
        elem_b="hinge_stub",
        min_overlap=0.50,
        name="tailgate stands across the rear opening height",
    )
    ctx.expect_gap(
        tailgate,
        body,
        axis="z",
        positive_elem="tailgate_panel",
        negative_elem="bed_floor",
        min_gap=0.015,
        max_gap=0.060,
        name="tailgate bottom clears the bed floor",
    )

    closed_edge = ctx.part_element_world_aabb(tailgate, elem="free_edge_cap")
    with ctx.pose({tailgate_hinge: 1.35}):
        open_edge = ctx.part_element_world_aabb(tailgate, elem="free_edge_cap")
    closed_x = (closed_edge[0][0] + closed_edge[1][0]) / 2.0 if closed_edge else None
    open_x = (open_edge[0][0] + open_edge[1][0]) / 2.0 if open_edge else None
    ctx.check(
        "tailgate swings sideways outward",
        closed_x is not None and open_x is not None and open_x < closed_x - 0.35,
        details=f"closed free-edge x={closed_x}, open free-edge x={open_x}",
    )

    closed_tip = ctx.part_element_world_aabb(handle, elem="handle_tip")
    with ctx.pose({handle_pivot: math.pi / 2.0}):
        turned_tip = ctx.part_element_world_aabb(handle, elem="handle_tip")
    closed_z = (closed_tip[0][2] + closed_tip[1][2]) / 2.0 if closed_tip else None
    turned_z = (turned_tip[0][2] + turned_tip[1][2]) / 2.0 if turned_tip else None
    ctx.check(
        "latch handle turns on its own pivot",
        closed_z is not None and turned_z is not None and turned_z < closed_z - 0.12,
        details=f"closed tip z={closed_z}, turned tip z={turned_z}",
    )

    return ctx.report()


object_model = build_object_model()
