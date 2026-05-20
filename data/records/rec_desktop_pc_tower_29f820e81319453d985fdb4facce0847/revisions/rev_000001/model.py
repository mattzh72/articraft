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
    model = ArticulatedObject(name="mid_tower_atx_case")

    steel = model.material("charcoal_powder_coated_steel", rgba=(0.035, 0.038, 0.042, 1.0))
    black = model.material("black_plastic_trim", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_mesh = model.material("dark_perforated_mesh", rgba=(0.010, 0.011, 0.012, 1.0))
    glass = model.material("smoked_tempered_glass", rgba=(0.10, 0.15, 0.18, 0.35))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    accent = model.material("brushed_screw_heads", rgba=(0.55, 0.57, 0.58, 1.0))

    width = 0.220
    depth = 0.460
    height = 0.460
    t = 0.012

    # Root chassis: a thin-wall, rectangular, powder-coated steel ATX tower.
    body = model.part("steel_body")
    body.visual(Box((width, depth, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.007)), material=steel, name="bottom_pan")
    body.visual(Box((width, 0.014, height)), origin=Origin(xyz=(0.0, depth / 2 - 0.007, height / 2)), material=steel, name="rear_wall")
    body.visual(Box((0.012, depth, height)), origin=Origin(xyz=(width / 2 - 0.006, 0.0, height / 2)), material=steel, name="solid_side_wall")

    # Open-side and front structural rails, connected to the bottom, rear wall, and top frame.
    body.visual(Box((0.014, 0.030, height)), origin=Origin(xyz=(-width / 2 + 0.007, depth / 2 - 0.015, height / 2)), material=steel, name="rear_glass_jamb")
    body.visual(Box((0.014, 0.035, height)), origin=Origin(xyz=(-width / 2 + 0.007, -depth / 2 + 0.0175, height / 2)), material=steel, name="front_glass_jamb")
    body.visual(Box((width, 0.032, 0.018)), origin=Origin(xyz=(0.0, -depth / 2 + 0.016, height - 0.009)), material=steel, name="front_top_rail")
    body.visual(Box((width, 0.032, 0.018)), origin=Origin(xyz=(0.0, depth / 2 - 0.016, height - 0.009)), material=steel, name="rear_top_rail")
    body.visual(Box((0.018, depth, 0.018)), origin=Origin(xyz=(-width / 2 + 0.009, 0.0, height - 0.009)), material=steel, name="left_top_rail")
    body.visual(Box((0.018, depth, 0.018)), origin=Origin(xyz=(width / 2 - 0.009, 0.0, height - 0.009)), material=steel, name="right_top_rail")
    body.visual(Box((width, 0.030, 0.020)), origin=Origin(xyz=(0.0, -depth / 2 + 0.015, 0.024)), material=steel, name="front_bottom_rail")
    body.visual(Box((0.018, 0.030, height)), origin=Origin(xyz=(width / 2 - 0.009, -depth / 2 + 0.015, height / 2)), material=steel, name="front_hinge_post")
    body.visual(Box((0.018, 0.030, height)), origin=Origin(xyz=(-width / 2 + 0.009, -depth / 2 + 0.015, height / 2)), material=steel, name="front_latch_post")

    # Rear ATX details mounted on the steel rear wall.
    body.visual(Box((0.092, 0.0025, 0.092)), origin=Origin(xyz=(0.030, depth / 2 + 0.00125, 0.310)), material=dark_mesh, name="rear_fan_grille")
    for i, xoff in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        body.visual(Box((0.003, 0.004, 0.082)), origin=Origin(xyz=(0.030 + xoff, depth / 2 + 0.002, 0.310)), material=steel, name=f"rear_fan_bar_v_{i}")
    for i, zoff in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        body.visual(Box((0.082, 0.004, 0.003)), origin=Origin(xyz=(0.030, depth / 2 + 0.002, 0.310 + zoff)), material=steel, name=f"rear_fan_bar_h_{i}")
    body.visual(Box((0.070, 0.003, 0.060)), origin=Origin(xyz=(-0.055, depth / 2 + 0.0015, 0.320)), material=black, name="rear_io_cutout")
    for i in range(7):
        body.visual(
            Box((0.085, 0.003, 0.006)),
            origin=Origin(xyz=(-0.030, depth / 2 + 0.0015, 0.100 + i * 0.014)),
            material=accent,
            name=f"pci_slot_{i}",
        )

    # Fixed hinge brackets on the chassis.  They are tangent to the moving pins
    # and keep the hinged panels visually mounted without occupying the same space.
    for i, zc in enumerate((0.135, 0.335)):
        body.visual(Box((0.016, 0.032, 0.050)), origin=Origin(xyz=(-width / 2 - 0.002, depth / 2 - 0.030, zc)), material=steel, name=f"glass_body_hinge_{i}")
        body.visual(Box((0.014, 0.026, 0.050)), origin=Origin(xyz=(width / 2 + 0.024, -depth / 2 - 0.010, zc)), material=steel, name=f"front_body_hinge_{i}")
        body.visual(Box((0.032, 0.006, 0.050)), origin=Origin(xyz=(width / 2 + 0.014, -depth / 2 + 0.005, zc)), material=steel, name=f"front_hinge_bridge_{i}")
    for i, xc in enumerate((-0.060, 0.060)):
        body.visual(Box((0.052, 0.018, 0.010)), origin=Origin(xyz=(xc, -depth / 2 + 0.018, height + 0.002)), material=steel, name=f"top_body_hinge_{i}")

    # Small feet make the case sit like a real mid-tower enclosure.
    for i, (x, y) in enumerate(((-0.075, -0.165), (0.075, -0.165), (-0.075, 0.165), (0.075, 0.165))):
        body.visual(Cylinder(radius=0.018, length=0.010), origin=Origin(xyz=(x, y, -0.005)), material=rubber, name=f"foot_{i}")

    # Tempered glass side panel.  Its local origin lies on the rear vertical hinge axis.
    glass_panel = model.part("glass_panel")
    side_len = 0.385
    side_h = 0.390
    side_z0 = 0.035
    frame = 0.014
    glass_panel.visual(Box((0.006, side_len - 2 * frame, side_h - 2 * frame)), origin=Origin(xyz=(0.0, -side_len / 2, side_z0 + side_h / 2)), material=glass, name="glass_sheet")
    glass_panel.visual(Box((0.011, side_len, frame)), origin=Origin(xyz=(0.0, -side_len / 2, side_z0 + frame / 2)), material=black, name="bottom_frame")
    glass_panel.visual(Box((0.011, side_len, frame)), origin=Origin(xyz=(0.0, -side_len / 2, side_z0 + side_h - frame / 2)), material=black, name="top_frame")
    glass_panel.visual(Box((0.011, frame, side_h)), origin=Origin(xyz=(0.0, -frame / 2, side_z0 + side_h / 2)), material=black, name="rear_frame")
    glass_panel.visual(Box((0.011, frame, side_h)), origin=Origin(xyz=(0.0, -side_len + frame / 2, side_z0 + side_h / 2)), material=black, name="front_frame")
    for i, zc in enumerate((0.135, 0.335)):
        glass_panel.visual(Cylinder(radius=0.0055, length=0.070), origin=Origin(xyz=(-0.001, 0.0, zc)), material=accent, name=f"rear_pin_{i}")
        glass_panel.visual(Box((0.006, 0.030, 0.045)), origin=Origin(xyz=(0.0, -0.014, zc)), material=black, name=f"glass_leaf_{i}")
    glass_panel.visual(Box((0.012, 0.010, 0.060)), origin=Origin(xyz=(0.0, -side_len + 0.025, side_z0 + side_h / 2)), material=accent, name="pull_tab")

    model.articulation(
        "body_to_glass_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=glass_panel,
        origin=Origin(xyz=(-width / 2 - 0.015, depth / 2 - 0.030, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.65),
    )

    # Front mesh intake door, hinged along the right edge.
    front_door = model.part("front_mesh_door")
    front_w = 0.205
    front_h = 0.405
    front_z0 = 0.028
    front_door.visual(Box((front_w - 0.034, 0.003, front_h - 0.052)), origin=Origin(xyz=(-front_w / 2, -0.012, front_z0 + front_h / 2)), material=dark_mesh, name="mesh_insert")
    for i in range(9):
        x = -front_w + 0.028 + i * ((front_w - 0.056) / 8)
        front_door.visual(Box((0.0035, 0.006, front_h - 0.048)), origin=Origin(xyz=(x, -0.015, front_z0 + front_h / 2)), material=black, name=f"front_mesh_wire_v_{i}")
    for i in range(13):
        z = front_z0 + 0.032 + i * ((front_h - 0.064) / 12)
        front_door.visual(Box((front_w - 0.040, 0.006, 0.0024)), origin=Origin(xyz=(-front_w / 2, -0.016, z)), material=black, name=f"front_mesh_wire_h_{i}")
    front_door.visual(Box((front_w, 0.014, 0.018)), origin=Origin(xyz=(-front_w / 2, -0.006, front_z0 + 0.009)), material=black, name="bottom_lip")
    front_door.visual(Box((front_w, 0.014, 0.018)), origin=Origin(xyz=(-front_w / 2, -0.006, front_z0 + front_h - 0.009)), material=black, name="top_lip")
    front_door.visual(Box((0.018, 0.014, front_h)), origin=Origin(xyz=(-front_w + 0.009, -0.006, front_z0 + front_h / 2)), material=black, name="latch_stile")
    front_door.visual(Box((0.018, 0.014, front_h)), origin=Origin(xyz=(-0.009, -0.006, front_z0 + front_h / 2)), material=black, name="hinge_stile")
    front_door.visual(Box((0.012, 0.010, 0.070)), origin=Origin(xyz=(-front_w + 0.025, -0.014, front_z0 + front_h / 2)), material=accent, name="front_handle")
    for i, zc in enumerate((0.135, 0.335)):
        front_door.visual(Cylinder(radius=0.0055, length=0.064), origin=Origin(xyz=(0.0, -0.006, zc)), material=accent, name=f"right_pin_{i}")
        front_door.visual(Box((0.026, 0.006, 0.045)), origin=Origin(xyz=(-0.013, -0.006, zc)), material=black, name=f"front_leaf_{i}")

    model.articulation(
        "body_to_front_mesh_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(width / 2 + 0.012, -depth / 2 - 0.012, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.55),
    )

    # Top removable ventilation panel on two front-edge hinge pins.
    top_panel = model.part("top_vent_panel")
    top_w = 0.176
    top_len = 0.365
    top_panel.visual(Box((top_w - 0.032, top_len - 0.045, 0.003)), origin=Origin(xyz=(0.0, top_len / 2, 0.0015)), material=dark_mesh, name="slot_insert")
    for i in range(10):
        y = 0.035 + i * ((top_len - 0.070) / 9)
        top_panel.visual(Box((top_w - 0.042, 0.004, 0.006)), origin=Origin(xyz=(0.0, y, 0.006)), material=black, name=f"top_vent_louver_{i}")
    top_panel.visual(Box((top_w, 0.018, 0.012)), origin=Origin(xyz=(0.0, 0.009, 0.006)), material=black, name="front_lip")
    top_panel.visual(Box((top_w, 0.018, 0.012)), origin=Origin(xyz=(0.0, top_len - 0.009, 0.006)), material=black, name="rear_lip")
    top_panel.visual(Box((0.018, top_len, 0.012)), origin=Origin(xyz=(-top_w / 2 + 0.009, top_len / 2, 0.006)), material=black, name="side_rail_0")
    top_panel.visual(Box((0.018, top_len, 0.012)), origin=Origin(xyz=(top_w / 2 - 0.009, top_len / 2, 0.006)), material=black, name="side_rail_1")
    for i, xc in enumerate((-0.055, 0.055)):
        top_panel.visual(Cylinder(radius=0.005, length=0.050), origin=Origin(xyz=(xc, 0.0, 0.000), rpy=(0.0, math.pi / 2, 0.0)), material=accent, name=f"front_pin_{i}")
        top_panel.visual(Box((0.045, 0.020, 0.006)), origin=Origin(xyz=(xc, 0.014, 0.003)), material=black, name=f"top_leaf_{i}")

    model.articulation(
        "body_to_top_vent_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_panel,
        origin=Origin(xyz=(0.0, -depth / 2 + 0.030, height + 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("steel_body")
    glass_panel = object_model.get_part("glass_panel")
    front_door = object_model.get_part("front_mesh_door")
    top_panel = object_model.get_part("top_vent_panel")
    glass_hinge = object_model.get_articulation("body_to_glass_panel")
    front_hinge = object_model.get_articulation("body_to_front_mesh_door")
    top_hinge = object_model.get_articulation("body_to_top_vent_panel")

    # Closed panels sit just outside their case openings and cover the openings.
    ctx.expect_gap(body, glass_panel, axis="x", min_gap=0.001, max_gap=0.030, positive_elem="rear_glass_jamb", negative_elem="glass_sheet", name="glass panel clears left steel jamb")
    ctx.expect_overlap(glass_panel, body, axes="z", min_overlap=0.300, elem_a="glass_sheet", elem_b="rear_glass_jamb", name="glass panel spans side height")
    ctx.expect_gap(body, front_door, axis="y", min_gap=0.001, max_gap=0.040, positive_elem="front_hinge_post", negative_elem="mesh_insert", name="front mesh door sits ahead of chassis")
    ctx.expect_overlap(front_door, body, axes="z", min_overlap=0.300, elem_a="mesh_insert", elem_b="front_hinge_post", name="front door covers intake height")
    ctx.expect_gap(top_panel, body, axis="z", min_gap=0.001, max_gap=0.030, positive_elem="slot_insert", negative_elem="front_top_rail", name="top vent panel sits above top frame")
    ctx.expect_overlap(top_panel, body, axes="x", min_overlap=0.120, elem_a="slot_insert", elem_b="front_top_rail", name="top vent overlaps top opening width")

    closed_glass_edge = ctx.part_element_world_aabb(glass_panel, elem="front_frame")
    closed_front_edge = ctx.part_element_world_aabb(front_door, elem="latch_stile")
    closed_top_edge = ctx.part_element_world_aabb(top_panel, elem="rear_lip")

    with ctx.pose({glass_hinge: 1.10, front_hinge: 1.10, top_hinge: 0.80}):
        open_glass_edge = ctx.part_element_world_aabb(glass_panel, elem="front_frame")
        open_front_edge = ctx.part_element_world_aabb(front_door, elem="latch_stile")
        open_top_edge = ctx.part_element_world_aabb(top_panel, elem="rear_lip")

    ctx.check(
        "glass side panel opens outward from rear pins",
        closed_glass_edge is not None
        and open_glass_edge is not None
        and open_glass_edge[0][0] < closed_glass_edge[0][0] - 0.080,
        details=f"closed={closed_glass_edge}, open={open_glass_edge}",
    )
    ctx.check(
        "front mesh door swings forward from right hinge",
        closed_front_edge is not None
        and open_front_edge is not None
        and open_front_edge[0][1] < closed_front_edge[0][1] - 0.080,
        details=f"closed={closed_front_edge}, open={open_front_edge}",
    )
    ctx.check(
        "top vent panel lifts upward from front hinges",
        closed_top_edge is not None
        and open_top_edge is not None
        and open_top_edge[1][2] > closed_top_edge[1][2] + 0.080,
        details=f"closed={closed_top_edge}, open={open_top_edge}",
    )

    return ctx.report()


object_model = build_object_model()
