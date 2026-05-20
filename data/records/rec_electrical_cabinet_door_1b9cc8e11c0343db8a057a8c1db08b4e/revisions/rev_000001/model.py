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
    model = ArticulatedObject(name="industrial_electrical_panel")

    painted_steel = Material("powder_coated_steel", rgba=(0.45, 0.50, 0.52, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    galvanized = Material("galvanized_backplate", rgba=(0.72, 0.74, 0.73, 1.0))
    rubber = Material("black_rubber_gasket", rgba=(0.01, 0.01, 0.01, 1.0))
    black = Material("black_latch", rgba=(0.02, 0.02, 0.018, 1.0))
    warning_yellow = Material("warning_label_yellow", rgba=(1.0, 0.78, 0.08, 1.0))
    red = Material("red_indicator", rgba=(0.75, 0.05, 0.04, 1.0))

    body = model.part("body")

    outer_w = 0.80
    outer_h = 1.20
    depth = 0.32
    wall = 0.025
    flange_w = 0.045
    flange_depth = 0.016

    # A real enclosure is a hollow steel box: five deep walls plus a folded
    # front return frame rather than a solid block.
    body.visual(
        Box((outer_w, wall, outer_h)),
        origin=Origin(xyz=(0.0, depth - wall / 2.0, outer_h / 2.0)),
        material=painted_steel,
        name="back_wall",
    )
    body.visual(
        Box((wall, depth, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall / 2.0, depth / 2.0, outer_h / 2.0)),
        material=painted_steel,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall / 2.0, depth / 2.0, outer_h / 2.0)),
        material=painted_steel,
        name="side_wall_1",
    )
    body.visual(
        Box((outer_w, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, wall / 2.0)),
        material=painted_steel,
        name="bottom_wall",
    )
    body.visual(
        Box((outer_w, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, outer_h - wall / 2.0)),
        material=painted_steel,
        name="top_wall",
    )

    body.visual(
        Box((flange_w, flange_depth, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + flange_w / 2.0, -0.007, outer_h / 2.0)),
        material=painted_steel,
        name="front_frame_left",
    )
    body.visual(
        Box((flange_w, flange_depth, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - flange_w / 2.0, -0.007, outer_h / 2.0)),
        material=painted_steel,
        name="front_frame_right",
    )
    body.visual(
        Box((outer_w, flange_depth, flange_w)),
        origin=Origin(xyz=(0.0, -0.007, flange_w / 2.0)),
        material=painted_steel,
        name="front_frame_bottom",
    )
    body.visual(
        Box((outer_w, flange_depth, flange_w)),
        origin=Origin(xyz=(0.0, -0.007, outer_h - flange_w / 2.0)),
        material=painted_steel,
        name="front_frame_top",
    )

    # Back mounting plate and DIN rails visible when the door is swung open.
    body.visual(
        Box((0.70, 0.010, 1.04)),
        origin=Origin(xyz=(0.0, 0.292, outer_h / 2.0)),
        material=galvanized,
        name="mounting_plate",
    )
    for idx, z in enumerate((0.38, 0.60, 0.82)):
        body.visual(
            Box((0.58, 0.007, 0.022)),
            origin=Origin(xyz=(0.0, 0.284, z)),
            material=dark_steel,
            name=f"din_rail_{idx}",
        )

    hinge_x = -outer_w / 2.0 - 0.020
    hinge_y = -0.032
    hinge_radius = 0.012
    hinge_zs = (0.22, 0.60, 0.98)
    for idx, zc in enumerate(hinge_zs):
        for suffix, zoff in (("lower", -0.040), ("upper", 0.040)):
            body.visual(
                Box((0.050, 0.018, 0.038)),
                origin=Origin(xyz=(hinge_x + 0.020, hinge_y + 0.014, zc + zoff)),
                material=dark_steel,
                name=f"hinge_{idx}_{suffix}_leaf",
            )
            body.visual(
                Cylinder(radius=hinge_radius, length=0.038),
                origin=Origin(xyz=(hinge_x, hinge_y, zc + zoff)),
                material=dark_steel,
                name=f"hinge_{idx}_{suffix}_knuckle",
            )

    body.visual(
        Box((0.025, 0.016, 0.075)),
        origin=Origin(xyz=(outer_w / 2.0 - 0.0025, -0.007, 0.62)),
        material=dark_steel,
        name="latch_striker",
    )

    door = model.part("front_door")
    door_w = 0.840
    door_h = 1.220
    door_x0 = 0.020
    door_center_x = door_x0 + door_w / 2.0
    door_center_z = outer_h / 2.0

    # Shallow pan construction: a front skin with folded perimeter returns and a
    # compressible gasket at the back edge.
    door.visual(
        Box((door_w, 0.018, door_h)),
        origin=Origin(xyz=(door_center_x, -0.042, door_center_z)),
        material=painted_steel,
        name="door_skin",
    )
    door.visual(
        Box((0.030, 0.052, door_h)),
        origin=Origin(xyz=(door_x0 + 0.045, -0.009, door_center_z)),
        material=painted_steel,
        name="door_return_left",
    )
    door.visual(
        Box((0.030, 0.052, door_h)),
        origin=Origin(xyz=(door_x0 + door_w - 0.015, -0.009, door_center_z)),
        material=painted_steel,
        name="door_return_right",
    )
    door.visual(
        Box((door_w, 0.052, 0.030)),
        origin=Origin(xyz=(door_center_x, -0.009, -0.010 + 0.015)),
        material=painted_steel,
        name="door_return_bottom",
    )
    door.visual(
        Box((door_w, 0.052, 0.030)),
        origin=Origin(xyz=(door_center_x, -0.009, -0.010 + door_h - 0.015)),
        material=painted_steel,
        name="door_return_top",
    )

    gasket_y = 0.014
    gasket_t = 0.004
    door.visual(
        Box((0.018, gasket_t, door_h - 0.160)),
        origin=Origin(xyz=(door_x0 + 0.045, gasket_y, door_center_z)),
        material=rubber,
        name="gasket_left",
    )
    door.visual(
        Box((0.018, gasket_t, door_h - 0.160)),
        origin=Origin(xyz=(door_x0 + door_w - 0.015, gasket_y, door_center_z)),
        material=rubber,
        name="gasket_right",
    )
    door.visual(
        Box((door_w - 0.080, gasket_t, 0.018)),
        origin=Origin(xyz=(door_center_x, gasket_y, 0.080)),
        material=rubber,
        name="gasket_bottom",
    )
    door.visual(
        Box((door_w - 0.080, gasket_t, 0.018)),
        origin=Origin(xyz=(door_center_x, gasket_y, outer_h - 0.080)),
        material=rubber,
        name="gasket_top",
    )

    door.visual(
        Box((0.160, 0.004, 0.090)),
        origin=Origin(xyz=(0.600, -0.053, 0.820)),
        material=warning_yellow,
        name="warning_label",
    )
    door.visual(
        Box((0.055, 0.005, 0.055)),
        origin=Origin(xyz=(0.690, -0.052, 0.455)),
        material=red,
        name="power_mark",
    )

    for idx, zc in enumerate(hinge_zs):
        door.visual(
            Box((0.062, 0.010, 0.044)),
            origin=Origin(xyz=(0.022, -0.008, zc)),
            material=dark_steel,
            name=f"hinge_{idx}_door_leaf",
        )
        door.visual(
            Cylinder(radius=hinge_radius, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=dark_steel,
            name=f"hinge_{idx}_door_knuckle",
        )

    door_joint = model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.0, lower=0.0, upper=1.75),
    )

    latch_handle = model.part("latch_handle")
    # Child frame is the handle spindle axis; the visible lever hangs downward in
    # the locked pose and turns a quarter-turn around the door-normal spindle.
    latch_handle.visual(
        Cylinder(radius=0.037, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle_base",
    )
    latch_handle.visual(
        Box((0.038, 0.014, 0.245)),
        origin=Origin(xyz=(0.0, -0.006, -0.105)),
        material=black,
        name="handle_grip",
    )
    latch_handle.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spindle_shaft",
    )
    latch_handle.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.005, 0.091, 0.0)),
        material=dark_steel,
        name="cam_arm",
    )
    latch_handle.visual(
        Box((0.030, 0.014, 0.028)),
        origin=Origin(xyz=(0.015, 0.091, 0.0)),
        material=dark_steel,
        name="cam_tongue",
    )

    model.articulation(
        "door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_handle,
        origin=Origin(xyz=(0.745, -0.066, 0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    door_joint.meta["description"] = "Single modeled DOF for the aligned pins of the three visible left-edge hinges."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("front_door")
    handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("body_to_front_door")
    latch_joint = object_model.get_articulation("door_to_latch_handle")

    ctx.allow_overlap(
        handle,
        door,
        elem_a="spindle_shaft",
        elem_b="door_skin",
        reason="The latch spindle intentionally passes through a hole in the steel door skin.",
    )
    ctx.expect_overlap(
        handle,
        door,
        axes="y",
        elem_a="spindle_shaft",
        elem_b="door_skin",
        min_overlap=0.012,
        name="latch spindle penetrates the door skin",
    )
    ctx.expect_within(
        handle,
        door,
        axes="xz",
        elem_a="spindle_shaft",
        elem_b="door_skin",
        margin=0.0,
        name="latch spindle is inside the door panel footprint",
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_frame_right",
        negative_elem="door_return_right",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed door return seats against front frame",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="door_skin",
        elem_b="back_wall",
        min_overlap=0.65,
        name="door covers the cabinet opening",
    )

    for idx in range(3):
        ctx.expect_gap(
            door,
            body,
            axis="z",
            positive_elem=f"hinge_{idx}_door_knuckle",
            negative_elem=f"hinge_{idx}_lower_knuckle",
            min_gap=0.001,
            max_gap=0.006,
            name=f"hinge {idx} lower knuckle clearance",
        )
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem=f"hinge_{idx}_upper_knuckle",
            negative_elem=f"hinge_{idx}_door_knuckle",
            min_gap=0.001,
            max_gap=0.006,
            name=f"hinge {idx} upper knuckle clearance",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    ctx.check(
        "front door opens outward from the cabinet",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.15,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    locked_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({latch_joint: math.pi / 2.0}):
        turned_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "locking handle turns a quarter turn",
        locked_grip_aabb is not None
        and turned_grip_aabb is not None
        and (locked_grip_aabb[1][2] - locked_grip_aabb[0][2]) > 0.20
        and (turned_grip_aabb[1][0] - turned_grip_aabb[0][0]) > 0.20,
        details=f"locked={locked_grip_aabb}, turned={turned_grip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
