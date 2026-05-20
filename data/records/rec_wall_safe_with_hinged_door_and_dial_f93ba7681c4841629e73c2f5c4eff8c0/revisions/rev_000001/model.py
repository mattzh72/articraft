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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_wall_safe")

    dark_steel = Material("dark_steel", color=(0.12, 0.13, 0.14, 1.0))
    black_cavity = Material("black_cavity", color=(0.035, 0.038, 0.040, 1.0))
    brushed_door = Material("brushed_door", color=(0.45, 0.47, 0.48, 1.0))
    satin_edge = Material("satin_edge", color=(0.25, 0.26, 0.27, 1.0))
    nickel = Material("nickel", color=(0.73, 0.70, 0.63, 1.0))
    black_mark = Material("black_mark", color=(0.01, 0.01, 0.012, 1.0))

    # Object frame: +X is out of the wall, Y is horizontal across the square
    # face, and Z is vertical.  The safe body is a hollow box recessed into the
    # wall, with only the front trim proud of the wall plane.
    body_size = 0.64
    body_depth = 0.18
    wall = 0.035
    front_trim_depth = 0.016

    body = model.part("body")
    body.visual(
        Box((0.012, body_size, body_size)),
        origin=Origin(xyz=(-body_depth + 0.006, 0.0, 0.0)),
        material=black_cavity,
        name="back_plate",
    )
    body.visual(
        Box((body_depth, wall, body_size)),
        origin=Origin(xyz=(-body_depth / 2, -body_size / 2 + wall / 2, 0.0)),
        material=dark_steel,
        name="left_wall",
    )
    body.visual(
        Box((body_depth, wall, body_size)),
        origin=Origin(xyz=(-body_depth / 2, body_size / 2 - wall / 2, 0.0)),
        material=dark_steel,
        name="right_wall",
    )
    body.visual(
        Box((body_depth, body_size, wall)),
        origin=Origin(xyz=(-body_depth / 2, 0.0, body_size / 2 - wall / 2)),
        material=dark_steel,
        name="top_wall",
    )
    body.visual(
        Box((body_depth, body_size, wall)),
        origin=Origin(xyz=(-body_depth / 2, 0.0, -body_size / 2 + wall / 2)),
        material=dark_steel,
        name="bottom_wall",
    )

    # Four-piece flush trim around the opening.  Its inner edge clears the door
    # by a few millimeters, so the square face reads as a wall safe rather than
    # a freestanding box.
    body.visual(
        Box((front_trim_depth, 0.040, body_size)),
        origin=Origin(xyz=(0.002, -body_size / 2 + 0.020, 0.0)),
        material=satin_edge,
        name="left_trim",
    )
    body.visual(
        Box((front_trim_depth, 0.040, body_size)),
        origin=Origin(xyz=(0.002, body_size / 2 - 0.020, 0.0)),
        material=satin_edge,
        name="right_trim",
    )
    body.visual(
        Box((front_trim_depth, body_size, 0.040)),
        origin=Origin(xyz=(0.002, 0.0, body_size / 2 - 0.020)),
        material=satin_edge,
        name="top_trim",
    )
    body.visual(
        Box((front_trim_depth, body_size, 0.040)),
        origin=Origin(xyz=(0.002, 0.0, -body_size / 2 + 0.020)),
        material=satin_edge,
        name="bottom_trim",
    )
    body.visual(
        Box((0.120, 0.600, 0.014)),
        origin=Origin(xyz=(-0.095, 0.0, -0.095)),
        material=black_cavity,
        name="shelf",
    )

    hinge_y = -0.315
    hinge_x = 0.030
    hinge_radius = 0.014
    body.visual(
        Cylinder(radius=hinge_radius, length=0.165),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.205)),
        material=nickel,
        name="upper_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.165),
        origin=Origin(xyz=(hinge_x, hinge_y, -0.205)),
        material=nickel,
        name="lower_hinge_barrel",
    )
    body.visual(
        Box((0.030, 0.030, 0.135)),
        origin=Origin(xyz=(0.016, hinge_y + 0.006, 0.205)),
        material=nickel,
        name="upper_hinge_leaf",
    )
    body.visual(
        Box((0.030, 0.030, 0.135)),
        origin=Origin(xyz=(0.016, hinge_y + 0.006, -0.205)),
        material=nickel,
        name="lower_hinge_leaf",
    )

    # The door frame is placed on the hinge axis.  In the closed pose the slab
    # extends from the left edge along local +Y, just like a real left-hinged
    # safe door.
    door = model.part("door")
    door_width = 0.560
    door_height = 0.560
    door_thickness = 0.035
    door_center_y = 0.315
    door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(door_thickness / 2 - hinge_x, door_center_y, 0.0)),
        material=brushed_door,
        name="door_slab",
    )
    door.visual(
        Box((0.008, door_width - 0.070, 0.018)),
        origin=Origin(xyz=(door_thickness + 0.002 - hinge_x, door_center_y, door_height / 2 - 0.042)),
        material=satin_edge,
        name="top_door_bead",
    )
    door.visual(
        Box((0.008, door_width - 0.070, 0.018)),
        origin=Origin(xyz=(door_thickness + 0.002 - hinge_x, door_center_y, -door_height / 2 + 0.042)),
        material=satin_edge,
        name="bottom_door_bead",
    )
    door.visual(
        Box((0.008, 0.018, door_height - 0.070)),
        origin=Origin(xyz=(door_thickness + 0.002 - hinge_x, door_center_y - door_width / 2 + 0.042, 0.0)),
        material=satin_edge,
        name="left_door_bead",
    )
    door.visual(
        Box((0.008, 0.018, door_height - 0.070)),
        origin=Origin(xyz=(door_thickness + 0.002 - hinge_x, door_center_y + door_width / 2 - 0.042, 0.0)),
        material=satin_edge,
        name="right_door_bead",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=nickel,
        name="middle_hinge_barrel",
    )
    door.visual(
        Box((0.020, 0.041, 0.170)),
        origin=Origin(xyz=(-0.006, 0.018, 0.0)),
        material=nickel,
        name="middle_hinge_leaf",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        # With the slab extending along local +Y, -Z makes positive motion swing
        # the free edge outward along +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # Round combination dial above the handle.  The small spindle penetrates the
    # door face like a retained bushing, while the visible disk sits proud.
    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.071, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="dial_spindle",
    )
    dial.visual(
        Box((0.004, 0.010, 0.080)),
        origin=Origin(xyz=(0.010, 0.0, 0.020)),
        material=black_mark,
        name="dial_pointer",
    )
    model.articulation(
        "dial_axis",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.044 - hinge_x, door_center_y, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    # A T-handle on a short horizontal shaft through the face.  Its travel is
    # limited to a half turn, which is typical for a safe latch handle.
    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="handle_shaft",
    )
    handle.visual(
        Cylinder(radius=0.028, length=0.035),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.160),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=nickel,
        name="handle_bar",
    )
    handle.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.040, -0.080, 0.0)),
        material=nickel,
        name="handle_end_0",
    )
    handle.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.040, 0.080, 0.0)),
        material=nickel,
        name="handle_end_1",
    )
    model.articulation(
        "handle_axis",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(0.060 - hinge_x, door_center_y, -0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    hinge = object_model.get_articulation("door_hinge")
    dial_axis = object_model.get_articulation("dial_axis")
    handle_axis = object_model.get_articulation("handle_axis")

    ctx.allow_overlap(
        door,
        dial,
        elem_a="door_slab",
        elem_b="dial_spindle",
        reason="The dial spindle is intentionally captured through the door bushing.",
    )
    ctx.allow_overlap(
        door,
        handle,
        elem_a="door_slab",
        elem_b="handle_shaft",
        reason="The T-handle shaft intentionally passes through the door face.",
    )

    ctx.check(
        "left hinge is vertical revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (0.0, 0.0, -1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.check(
        "dial is continuous",
        dial_axis.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={dial_axis.articulation_type}, axis={dial_axis.axis}",
    )
    ctx.check(
        "handle has half turn limits",
        handle_axis.motion_limits is not None
        and handle_axis.motion_limits.lower is not None
        and handle_axis.motion_limits.upper is not None
        and handle_axis.motion_limits.lower <= -1.55
        and handle_axis.motion_limits.upper >= 1.55,
        details=f"limits={handle_axis.motion_limits}",
    )

    ctx.expect_within(
        door,
        body,
        axes="yz",
        inner_elem="door_slab",
        outer_elem="back_plate",
        margin=0.0,
        name="closed door fits within square body",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_slab",
        negative_elem="shelf",
        min_gap=0.025,
        name="door clears shallow shelf",
    )
    ctx.expect_overlap(
        dial,
        door,
        axes="x",
        elem_a="dial_spindle",
        elem_b="door_slab",
        min_overlap=0.020,
        name="dial spindle is retained through door",
    )
    ctx.expect_within(
        dial,
        door,
        axes="yz",
        inner_elem="dial_spindle",
        outer_elem="door_slab",
        margin=0.0,
        name="dial spindle is centered on door face",
    )
    ctx.expect_overlap(
        handle,
        door,
        axes="x",
        elem_a="handle_shaft",
        elem_b="door_slab",
        min_overlap=0.004,
        name="handle shaft penetrates door face",
    )

    closed_door = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({hinge: 1.30}):
        open_door = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door swings outward from left hinge",
        closed_door is not None and open_door is not None and open_door[1][0] > closed_door[1][0] + 0.25,
        details=f"closed={closed_door}, open={open_door}",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    with ctx.pose({dial_axis: math.pi / 2.0}):
        pointer_turn = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    ctx.check(
        "dial pointer rotates about face normal",
        pointer_rest is not None
        and pointer_turn is not None
        and (pointer_rest[1][2] - pointer_rest[0][2]) > (pointer_rest[1][1] - pointer_rest[0][1]) * 3.0
        and (pointer_turn[1][1] - pointer_turn[0][1]) > (pointer_turn[1][2] - pointer_turn[0][2]) * 3.0,
        details=f"rest={pointer_rest}, turned={pointer_turn}",
    )

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_bar")
    with ctx.pose({handle_axis: math.pi / 2.0}):
        handle_turn = ctx.part_element_world_aabb(handle, elem="handle_bar")
    ctx.check(
        "T handle rotates on horizontal shaft",
        handle_rest is not None
        and handle_turn is not None
        and (handle_rest[1][1] - handle_rest[0][1]) > (handle_rest[1][2] - handle_rest[0][2]) * 3.0
        and (handle_turn[1][2] - handle_turn[0][2]) > (handle_turn[1][1] - handle_turn[0][1]) * 3.0,
        details=f"rest={handle_rest}, turned={handle_turn}",
    )

    return ctx.report()


object_model = build_object_model()
