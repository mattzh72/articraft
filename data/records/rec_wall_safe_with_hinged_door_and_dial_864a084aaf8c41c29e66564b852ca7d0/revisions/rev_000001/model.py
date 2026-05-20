from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    wall = model.material("painted_wall", rgba=(0.72, 0.70, 0.66, 1.0))
    frame_steel = model.material("dark_gunmetal", rgba=(0.09, 0.10, 0.11, 1.0))
    door_steel = model.material("blued_steel", rgba=(0.055, 0.065, 0.075, 1.0))
    edge_shadow = model.material("black_shadow", rgba=(0.008, 0.008, 0.008, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.66, 0.64, 0.58, 1.0))
    dial_black = model.material("black_dial", rgba=(0.015, 0.016, 0.018, 1.0))
    white_mark = model.material("white_marking", rgba=(0.92, 0.90, 0.82, 1.0))

    body = model.part("body")

    # Coordinate convention: the wall face is the XZ plane, the safe recess goes
    # into -Y, and the viewer/door swing is toward +Y.
    body.visual(Box((0.160, 0.035, 1.050)), origin=Origin(xyz=(-0.445, -0.025, 0.0)), material=wall, name="wall_left")
    body.visual(Box((0.160, 0.035, 1.050)), origin=Origin(xyz=(0.445, -0.025, 0.0)), material=wall, name="wall_right")
    body.visual(Box((1.050, 0.035, 0.160)), origin=Origin(xyz=(0.0, -0.025, 0.445)), material=wall, name="wall_top")
    body.visual(Box((1.050, 0.035, 0.160)), origin=Origin(xyz=(0.0, -0.025, -0.445)), material=wall, name="wall_bottom")

    # Hollow recessed safe box: four side walls and a back plate, not a solid
    # block.  The front frame overlaps the wall slightly as a plaster trim lip.
    body.visual(Box((0.655, 0.018, 0.655)), origin=Origin(xyz=(0.0, -0.245, 0.0)), material=frame_steel, name="back_plate")
    body.visual(Box((0.035, 0.250, 0.655)), origin=Origin(xyz=(-0.312, -0.120, 0.0)), material=frame_steel, name="body_left_wall")
    body.visual(Box((0.035, 0.250, 0.655)), origin=Origin(xyz=(0.312, -0.120, 0.0)), material=frame_steel, name="body_right_wall")
    body.visual(Box((0.655, 0.250, 0.035)), origin=Origin(xyz=(0.0, -0.120, 0.312)), material=frame_steel, name="body_top_wall")
    body.visual(Box((0.655, 0.250, 0.035)), origin=Origin(xyz=(0.0, -0.120, -0.312)), material=frame_steel, name="body_bottom_wall")

    body.visual(Box((0.070, 0.018, 0.740)), origin=Origin(xyz=(-0.335, -0.003, 0.0)), material=frame_steel, name="front_frame_left")
    body.visual(Box((0.070, 0.018, 0.740)), origin=Origin(xyz=(0.335, -0.003, 0.0)), material=frame_steel, name="front_frame_right")
    body.visual(Box((0.740, 0.018, 0.070)), origin=Origin(xyz=(0.0, -0.003, 0.335)), material=frame_steel, name="front_frame_top")
    body.visual(Box((0.740, 0.018, 0.070)), origin=Origin(xyz=(0.0, -0.003, -0.335)), material=frame_steel, name="front_frame_bottom")

    # Exposed right-side hinge barrel and its welded leaf on the frame edge.
    body.visual(Box((0.026, 0.022, 0.580)), origin=Origin(xyz=(0.302, 0.016, 0.0)), material=frame_steel, name="hinge_leaf")
    body.visual(Cylinder(radius=0.010, length=0.185), origin=Origin(xyz=(0.303, 0.034, 0.205)), material=brushed, name="hinge_barrel_0")
    body.visual(Cylinder(radius=0.010, length=0.185), origin=Origin(xyz=(0.303, 0.034, 0.0)), material=brushed, name="hinge_barrel_1")
    body.visual(Cylinder(radius=0.010, length=0.185), origin=Origin(xyz=(0.303, 0.034, -0.205)), material=brushed, name="hinge_barrel_2")

    door = model.part("door")
    door.visual(Box((0.540, 0.058, 0.580)), origin=Origin(xyz=(-0.288, 0.004, 0.0)), material=door_steel, name="door_slab")
    door.visual(Box((0.510, 0.006, 0.026)), origin=Origin(xyz=(-0.288, 0.036, 0.262)), material=frame_steel, name="door_top_bevel")
    door.visual(Box((0.510, 0.006, 0.026)), origin=Origin(xyz=(-0.288, 0.036, -0.262)), material=frame_steel, name="door_bottom_bevel")
    door.visual(Box((0.026, 0.006, 0.530)), origin=Origin(xyz=(-0.529, 0.036, 0.0)), material=frame_steel, name="door_free_edge")
    door.visual(Box((0.022, 0.020, 0.530)), origin=Origin(xyz=(-0.025, -0.015, 0.0)), material=frame_steel, name="door_hinge_edge")

    # The deposit flap is a separate hinged panel; these narrow dark reveals are
    # on the fixed door face around the panel rather than a stamped outline.
    flap_x = -0.318
    flap_w = 0.205
    flap_h = 0.075
    flap_top = 0.070
    door_front_y = 0.033
    door.visual(Box((0.008, 0.002, flap_h + 0.016)), origin=Origin(xyz=(flap_x - flap_w / 2 - 0.006, door_front_y + 0.001, flap_top - flap_h / 2)), material=edge_shadow, name="flap_gap_left")
    door.visual(Box((0.008, 0.002, flap_h + 0.016)), origin=Origin(xyz=(flap_x + flap_w / 2 + 0.006, door_front_y + 0.001, flap_top - flap_h / 2)), material=edge_shadow, name="flap_gap_right")
    door.visual(Box((flap_w + 0.028, 0.002, 0.008)), origin=Origin(xyz=(flap_x, door_front_y + 0.001, flap_top - flap_h - 0.006)), material=edge_shadow, name="flap_gap_bottom")

    # Fixed index mark above the combination dial.
    door.visual(Box((0.018, 0.002, 0.010)), origin=Origin(xyz=(-0.318, door_front_y + 0.0005, 0.214)), material=white_mark, name="dial_index")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.303, 0.034, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.108,
            0.020,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=56, depth=0.0012, helix_angle_deg=18.0),
            center=False,
        ),
        "combination_dial",
    )
    dial.visual(dial_mesh, origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dial_black, name="dial_body")
    dial.visual(Cylinder(radius=0.020, length=0.005), origin=Origin(xyz=(0.0, 0.0223, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed, name="dial_center_cap")
    for i in range(12):
        theta = i * math.tau / 12.0
        radius = 0.044
        mark_len = 0.013 if i % 3 == 0 else 0.009
        dial.visual(
            Box((0.003, 0.002, mark_len)),
            origin=Origin(
                xyz=(radius * math.cos(theta), 0.021, radius * math.sin(theta)),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=white_mark,
            name=f"dial_tick_{i}",
        )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.318, door_front_y, 0.148)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    handle = model.part("handle")
    handle.visual(Cylinder(radius=0.024, length=0.026), origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed, name="handle_hub")
    handle.visual(Cylinder(radius=0.014, length=0.010), origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=frame_steel, name="hub_button")
    for i, theta in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        radial_mid = 0.038
        radial_end = 0.078
        handle.visual(
            Cylinder(radius=0.0075, length=0.082),
            origin=Origin(
                xyz=(radial_mid * math.cos(theta), 0.025, radial_mid * math.sin(theta)),
                rpy=(0.0, math.pi / 2.0 - theta, 0.0),
            ),
            material=brushed,
            name=f"handle_spoke_{i}",
        )
        handle.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=(radial_end * math.cos(theta), 0.025, radial_end * math.sin(theta))),
            material=brushed,
            name=f"handle_tip_{i}",
        )

    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(-0.318, door_front_y, -0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.75, upper=0.75),
    )

    flap = model.part("flap")
    flap.visual(Box((flap_w, 0.016, flap_h)), origin=Origin(xyz=(0.0, 0.008, -flap_h / 2.0)), material=door_steel, name="flap_panel")
    flap.visual(Cylinder(radius=0.006, length=flap_w * 0.86), origin=Origin(xyz=(0.0, 0.007, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed, name="flap_hinge")
    flap.visual(Box((flap_w * 0.55, 0.004, 0.006)), origin=Origin(xyz=(0.0, 0.018, -flap_h + 0.012)), material=brushed, name="flap_pull_lip")

    model.articulation(
        "door_to_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=flap,
        origin=Origin(xyz=(flap_x, door_front_y, flap_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("handle")
    flap = object_model.get_part("flap")
    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_handle")
    flap_joint = object_model.get_articulation("door_to_flap")

    ctx.check("right hinged door is revolute", door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("combination dial is continuous", dial_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("handle is hub mounted", handle_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("deposit flap is top hinged", flap_joint.articulation_type == ArticulationType.REVOLUTE)

    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_slab",
        negative_elem="front_frame_top",
        min_gap=0.0,
        max_gap=0.010,
        name="closed door sits just proud of frame",
    )
    ctx.expect_gap(
        dial,
        door,
        axis="y",
        positive_elem="dial_body",
        negative_elem="door_slab",
        min_gap=0.0,
        max_gap=0.002,
        name="dial mounts on the door face",
    )
    ctx.expect_gap(
        handle,
        door,
        axis="y",
        positive_elem="handle_hub",
        negative_elem="door_slab",
        min_gap=0.0,
        max_gap=0.002,
        name="handle hub mounts on the door face",
    )
    ctx.expect_gap(
        flap,
        door,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="door_slab",
        min_gap=0.0,
        max_gap=0.002,
        name="flap is a separate panel on the face",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_joint: 1.0}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door swings outward from the right hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.20,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 0.85}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "deposit flap rotates outward on its top edge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.035,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
