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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    wall_mat = model.material("painted_wall", rgba=(0.73, 0.72, 0.68, 1.0))
    frame_mat = model.material("blackened_steel", rgba=(0.035, 0.038, 0.042, 1.0))
    body_mat = model.material("shadow_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    door_mat = model.material("heavy_blue_steel", rgba=(0.10, 0.13, 0.17, 1.0))
    edge_mat = model.material("worn_dark_edge", rgba=(0.018, 0.020, 0.023, 1.0))
    brass_mat = model.material("aged_brass", rgba=(0.76, 0.56, 0.25, 1.0))
    dial_mat = model.material("brushed_dial_black", rgba=(0.018, 0.018, 0.016, 1.0))
    mark_mat = model.material("painted_white_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    body = model.part("safe_body")

    # Surrounding wall patch made from overlapping masonry/drywall bars, so the
    # safe reads as fixed into a real wall opening rather than freestanding.
    body.visual(
        Box((0.055, 0.38, 1.24)),
        origin=Origin(xyz=(-0.030, -0.585, 0.750)),
        material=wall_mat,
        name="wall_left",
    )
    body.visual(
        Box((0.055, 0.38, 1.24)),
        origin=Origin(xyz=(-0.030, 0.585, 0.750)),
        material=wall_mat,
        name="wall_right",
    )
    body.visual(
        Box((0.055, 1.18, 0.31)),
        origin=Origin(xyz=(-0.030, 0.000, 1.365)),
        material=wall_mat,
        name="wall_top",
    )
    body.visual(
        Box((0.055, 1.18, 0.31)),
        origin=Origin(xyz=(-0.030, 0.000, 0.135)),
        material=wall_mat,
        name="wall_bottom",
    )

    # Recessed steel safe body: a box-shaped shell disappearing into the wall.
    body.visual(
        Box((0.025, 0.64, 0.80)),
        origin=Origin(xyz=(-0.355, 0.000, 0.750)),
        material=body_mat,
        name="back_plate",
    )
    body.visual(
        Box((0.330, 0.040, 0.80)),
        origin=Origin(xyz=(-0.195, -0.330, 0.750)),
        material=body_mat,
        name="side_wall_0",
    )
    body.visual(
        Box((0.330, 0.040, 0.80)),
        origin=Origin(xyz=(-0.195, 0.330, 0.750)),
        material=body_mat,
        name="side_wall_1",
    )
    body.visual(
        Box((0.330, 0.700, 0.040)),
        origin=Origin(xyz=(-0.195, 0.000, 1.155)),
        material=body_mat,
        name="top_wall",
    )
    body.visual(
        Box((0.330, 0.700, 0.040)),
        origin=Origin(xyz=(-0.195, 0.000, 0.345)),
        material=body_mat,
        name="bottom_wall",
    )

    # Thick front escutcheon/frame surrounding the door recess.
    body.visual(
        Box((0.090, 0.080, 0.920)),
        origin=Origin(xyz=(0.000, -0.365, 0.750)),
        material=frame_mat,
        name="front_frame_0",
    )
    body.visual(
        Box((0.090, 0.080, 0.920)),
        origin=Origin(xyz=(0.000, 0.365, 0.750)),
        material=frame_mat,
        name="front_frame_1",
    )
    body.visual(
        Box((0.090, 0.810, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 1.185)),
        material=frame_mat,
        name="front_frame_top",
    )
    body.visual(
        Box((0.090, 0.810, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 0.315)),
        material=frame_mat,
        name="front_frame_bottom",
    )
    body.visual(
        Box((0.085, 0.030, 0.720)),
        origin=Origin(xyz=(0.045, -0.319, 0.750)),
        material=edge_mat,
        name="hinge_support",
    )

    door = model.part("door")
    door.visual(
        Box((0.075, 0.560, 0.720)),
        origin=Origin(xyz=(0.000, 0.290, 0.000)),
        material=door_mat,
        name="door_slab",
    )
    door.visual(
        Box((0.012, 0.460, 0.560)),
        origin=Origin(xyz=(0.042, 0.300, 0.000)),
        material=frame_mat,
        name="door_face_plate",
    )
    door.visual(
        Box((0.014, 0.390, 0.035)),
        origin=Origin(xyz=(0.043, 0.300, 0.300)),
        material=edge_mat,
        name="upper_lock_rail",
    )
    door.visual(
        Box((0.014, 0.390, 0.035)),
        origin=Origin(xyz=(0.043, 0.300, -0.300)),
        material=edge_mat,
        name="lower_lock_rail",
    )
    door.visual(
        Box((0.038, 0.050, 0.620)),
        origin=Origin(xyz=(0.000, 0.018, 0.000)),
        material=edge_mat,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.640),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=brass_mat,
        name="hinge_barrel",
    )

    # Child frame sits on the hinge axis. The door extends in local +Y, so a
    # negative Z axis makes positive motion swing the free edge outward (+X).
    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.065, -0.290, 0.750)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.75),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.061, length=0.030),
        origin=Origin(xyz=(0.015, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_mat,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.067, length=0.010),
        origin=Origin(xyz=(0.005, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass_mat,
        name="dial_outer_rim",
    )
    dial.visual(
        Box((0.006, 0.010, 0.070)),
        origin=Origin(xyz=(0.031, 0.000, 0.024)),
        material=mark_mat,
        name="dial_index_line",
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.048, 0.300, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.013, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass_mat,
        name="handle_hub",
    )
    handle.visual(
        Box((0.034, 0.034, 0.190)),
        origin=Origin(xyz=(0.041, 0.000, -0.095)),
        material=brass_mat,
        name="handle_grip",
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.048, 0.300, -0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("body_to_door")
    handle_joint = object_model.get_articulation("door_to_handle")

    ctx.expect_gap(
        dial,
        door,
        axis="x",
        max_penetration=0.00001,
        max_gap=0.002,
        positive_elem="dial_cap",
        negative_elem="door_face_plate",
        name="dial seats on the outer door face",
    )
    ctx.expect_gap(
        handle,
        door,
        axis="x",
        max_penetration=0.00001,
        max_gap=0.002,
        positive_elem="handle_hub",
        negative_elem="door_face_plate",
        name="handle hub seats on the outer door face",
    )
    ctx.expect_within(
        dial,
        door,
        axes="yz",
        inner_elem="dial_cap",
        outer_elem="door_slab",
        margin=0.005,
        name="combination dial stays on the door panel",
    )
    ctx.expect_within(
        handle,
        door,
        axes="yz",
        inner_elem="handle_grip",
        outer_elem="door_slab",
        margin=0.010,
        name="handle stays on the door panel",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door swings outward from the recessed frame",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.20,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_joint: math.pi / 2.0}):
        turned_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    if closed_handle_aabb is not None and turned_handle_aabb is not None:
        closed_y_span = closed_handle_aabb[1][1] - closed_handle_aabb[0][1]
        turned_y_span = turned_handle_aabb[1][1] - turned_handle_aabb[0][1]
    else:
        closed_y_span = turned_y_span = 0.0
    ctx.check(
        "handle rotates about its spindle",
        turned_y_span > closed_y_span + 0.10,
        details=f"closed={closed_handle_aabb}, turned={turned_handle_aabb}",
    )

    ctx.expect_within(
        door,
        body,
        axes="z",
        inner_elem="door_slab",
        outer_elem="front_frame_0",
        margin=0.015,
        name="heavy door is vertically framed by the safe opening",
    )

    return ctx.report()


object_model = build_object_model()
