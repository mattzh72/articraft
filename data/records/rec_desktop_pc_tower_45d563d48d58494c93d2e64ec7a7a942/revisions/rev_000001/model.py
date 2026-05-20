from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_glass_gaming_pc_case")

    steel = model.material("powder_coated_black_steel", rgba=(0.015, 0.017, 0.020, 1.0))
    edge_trim = model.material("satin_black_trim", rgba=(0.002, 0.002, 0.003, 1.0))
    dark_glass = model.material("smoked_tempered_glass", rgba=(0.07, 0.12, 0.16, 0.38))
    mesh_black = model.material("black_perforated_mesh", rgba=(0.005, 0.006, 0.006, 1.0))
    magnet = model.material("dark_magnetic_latch", rgba=(0.12, 0.12, 0.13, 1.0))

    width = 0.36
    depth = 0.48
    height = 0.50
    steel_t = 0.015

    body = model.part("body")

    # One connected powder-coated steel chassis: bottom tray, rear wall, top rim,
    # front frame, right wall, and the open side-frame rails for the side glass.
    body.visual(
        Box((width, depth, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, steel_t / 2.0)),
        material=steel,
        name="bottom_tray",
    )
    body.visual(
        Box((width, steel_t, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - steel_t / 2.0, height / 2.0)),
        material=steel,
        name="rear_wall",
    )
    body.visual(
        Box((steel_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - steel_t / 2.0, 0.0, height / 2.0)),
        material=steel,
        name="solid_side_wall",
    )
    body.visual(
        Box((width, steel_t, steel_t)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + steel_t / 2.0, height - steel_t / 2.0)),
        material=steel,
        name="front_top_rail",
    )
    body.visual(
        Box((width, steel_t, steel_t)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + steel_t / 2.0, steel_t / 2.0)),
        material=steel,
        name="front_bottom_rail",
    )
    body.visual(
        Box((steel_t, steel_t, height)),
        origin=Origin(xyz=(-width / 2.0 + steel_t / 2.0, -depth / 2.0 + steel_t / 2.0, height / 2.0)),
        material=steel,
        name="front_frame_stile",
    )
    body.visual(
        Box((steel_t, depth, steel_t)),
        origin=Origin(xyz=(-width / 2.0 + steel_t / 2.0, 0.0, height - steel_t / 2.0)),
        material=steel,
        name="side_top_rail",
    )
    body.visual(
        Box((steel_t, depth, steel_t)),
        origin=Origin(xyz=(-width / 2.0 + steel_t / 2.0, 0.0, steel_t / 2.0)),
        material=steel,
        name="side_bottom_rail",
    )
    body.visual(
        Box((steel_t, steel_t, height)),
        origin=Origin(xyz=(-width / 2.0 + steel_t / 2.0, depth / 2.0 - steel_t / 2.0, height / 2.0)),
        material=steel,
        name="side_rear_stile",
    )
    body.visual(
        Box((steel_t, depth, steel_t)),
        origin=Origin(xyz=(width / 2.0 - steel_t / 2.0, 0.0, height - steel_t / 2.0)),
        material=steel,
        name="top_side_rail",
    )
    body.visual(
        Box((width, steel_t, steel_t)),
        origin=Origin(xyz=(0.0, depth / 2.0 - steel_t / 2.0, height - steel_t / 2.0)),
        material=steel,
        name="top_rear_rail",
    )

    # Dark internal hardware visible behind the glass.
    body.visual(
        Box((0.18, 0.018, 0.28)),
        origin=Origin(xyz=(0.0, depth / 2.0 - steel_t - 0.009, 0.27)),
        material=edge_trim,
        name="motherboard_plate",
    )
    body.visual(
        Box((0.16, 0.11, 0.07)),
        origin=Origin(xyz=(0.0, 0.12, 0.050)),
        material=edge_trim,
        name="power_supply_shroud",
    )
    body.visual(
        Box((0.018, steel_t, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + steel_t / 2.0, height / 2.0)),
        material=edge_trim,
        name="front_fan_spine",
    )
    for i, zc in enumerate((0.18, 0.30, 0.42)):
        body.visual(
            Cylinder(radius=0.052, length=0.006),
            origin=Origin(xyz=(0.0, -depth / 2.0 + 0.012, zc), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=edge_trim,
            name=f"front_fan_ring_{i}",
        )

    # Stationary hinge leaves and magnetic latch catch plates fixed to the chassis.
    body.visual(
        Box((0.014, 0.006, 0.36)),
        origin=Origin(xyz=(width / 2.0, -depth / 2.0 - 0.003, 0.25)),
        material=edge_trim,
        name="front_hinge_leaf",
    )
    body.visual(
        Box((0.007, 0.008, 0.34)),
        origin=Origin(xyz=(-width / 2.0 - 0.0035, depth / 2.0 - 0.003, 0.25)),
        material=edge_trim,
        name="side_hinge_leaf",
    )
    body.visual(
        Box((width, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.173, height + 0.001)),
        material=edge_trim,
        name="top_latch_crossbar",
    )
    for i, x in enumerate((-0.10, 0.10)):
        body.visual(
            Box((0.045, 0.014, 0.002)),
            origin=Origin(xyz=(x, 0.173, height + 0.001)),
            material=magnet,
            name=f"top_latch_catch_{i}",
        )

    # Front tempered glass door: part frame is the vertical hinge pin line.
    front_door = model.part("front_door")
    front_door_w = 0.355
    front_door_h = 0.45
    front_door.visual(
        Box((front_door_w, 0.006, front_door_h)),
        origin=Origin(xyz=(-front_door_w / 2.0, 0.0, front_door_h / 2.0)),
        material=dark_glass,
        name="front_glass",
    )
    front_door.visual(
        Cylinder(radius=0.005, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=edge_trim,
        name="front_hinge_barrel",
    )
    front_door.visual(
        Box((0.012, 0.008, front_door_h)),
        origin=Origin(xyz=(-0.006, 0.0, front_door_h / 2.0)),
        material=edge_trim,
        name="front_hinge_stile",
    )
    front_door.visual(
        Box((0.012, 0.008, front_door_h)),
        origin=Origin(xyz=(-front_door_w + 0.006, 0.0, front_door_h / 2.0)),
        material=edge_trim,
        name="front_latch_stile",
    )
    front_door.visual(
        Box((front_door_w, 0.008, 0.014)),
        origin=Origin(xyz=(-front_door_w / 2.0, 0.0, front_door_h - 0.007)),
        material=edge_trim,
        name="front_top_trim",
    )
    front_door.visual(
        Box((front_door_w, 0.008, 0.014)),
        origin=Origin(xyz=(-front_door_w / 2.0, 0.0, 0.007)),
        material=edge_trim,
        name="front_bottom_trim",
    )
    front_door.visual(
        Box((0.018, 0.012, 0.050)),
        origin=Origin(xyz=(-front_door_w + 0.018, -0.006, 0.25)),
        material=edge_trim,
        name="front_pull_recess",
    )

    # Side tempered glass panel: part frame is the rear vertical hinge line.
    side_panel = model.part("side_panel")
    side_panel_d = 0.425
    side_panel_h = 0.43
    side_panel.visual(
        Box((0.006, side_panel_d, side_panel_h)),
        origin=Origin(xyz=(0.0, -side_panel_d / 2.0, side_panel_h / 2.0)),
        material=dark_glass,
        name="side_glass",
    )
    side_panel.visual(
        Cylinder(radius=0.005, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=edge_trim,
        name="side_hinge_barrel",
    )
    side_panel.visual(
        Box((0.008, 0.012, side_panel_h)),
        origin=Origin(xyz=(0.0, -0.006, side_panel_h / 2.0)),
        material=edge_trim,
        name="side_hinge_stile",
    )
    side_panel.visual(
        Box((0.008, 0.012, side_panel_h)),
        origin=Origin(xyz=(0.0, -side_panel_d + 0.006, side_panel_h / 2.0)),
        material=edge_trim,
        name="side_front_stile",
    )
    side_panel.visual(
        Box((0.008, side_panel_d, 0.014)),
        origin=Origin(xyz=(0.0, -side_panel_d / 2.0, side_panel_h - 0.007)),
        material=edge_trim,
        name="side_top_trim",
    )
    side_panel.visual(
        Box((0.008, side_panel_d, 0.014)),
        origin=Origin(xyz=(0.0, -side_panel_d / 2.0, 0.007)),
        material=edge_trim,
        name="side_bottom_trim",
    )

    # Top magnetic mesh filter: slot-pattern mesh plate, hinge barrel at front,
    # and two underside magnetic blocks that close against the chassis catches.
    top_filter = model.part("top_filter")
    filter_w = 0.320
    filter_d = 0.400
    filter_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (filter_w, filter_d),
            0.004,
            slot_size=(0.040, 0.006),
            pitch=(0.052, 0.018),
            frame=0.014,
            corner_radius=0.006,
            stagger=True,
        ),
        "top_filter_mesh",
    )
    top_filter.visual(
        filter_mesh,
        origin=Origin(xyz=(0.0, filter_d / 2.0, 0.0)),
        material=mesh_black,
        name="top_mesh",
    )
    top_filter.visual(
        Cylinder(radius=0.004, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=edge_trim,
        name="top_hinge_barrel",
    )
    top_filter.visual(
        Box((filter_w, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=edge_trim,
        name="top_hinge_trim",
    )
    top_filter.visual(
        Box((filter_w, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, filter_d - 0.006, 0.0)),
        material=edge_trim,
        name="top_latch_trim",
    )
    for i, x in enumerate((-0.10, 0.10)):
        top_filter.visual(
            Box((0.035, 0.012, 0.006)),
            origin=Origin(xyz=(x, filter_d - 0.012, -0.003)),
            material=magnet,
            name=f"top_magnet_{i}",
        )

    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(width / 2.0 + 0.012, -depth / 2.0 - 0.010, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_panel,
        origin=Origin(xyz=(-width / 2.0 - 0.012, depth / 2.0, 0.035)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "top_filter_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_filter,
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.025, height + 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_door = object_model.get_part("front_door")
    side_panel = object_model.get_part("side_panel")
    top_filter = object_model.get_part("top_filter")
    front_hinge = object_model.get_articulation("front_door_hinge")
    side_hinge = object_model.get_articulation("side_panel_hinge")
    top_hinge = object_model.get_articulation("top_filter_hinge")

    ctx.expect_gap(
        body,
        front_door,
        axis="y",
        min_gap=0.0,
        max_gap=0.015,
        positive_elem="front_top_rail",
        negative_elem="front_glass",
        name="front glass door sits proud of the steel front frame",
    )
    ctx.expect_overlap(
        front_door,
        body,
        axes="xz",
        min_overlap=0.28,
        name="front door covers the rectangular front opening",
    )
    ctx.expect_gap(
        body,
        side_panel,
        axis="x",
        min_gap=0.0,
        max_gap=0.018,
        positive_elem="side_top_rail",
        negative_elem="side_glass",
        name="side glass panel sits proud of the steel side frame",
    )
    ctx.expect_overlap(
        side_panel,
        body,
        axes="yz",
        min_overlap=0.35,
        name="side panel covers the side service opening",
    )
    ctx.expect_gap(
        top_filter,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        name="top filter rests at the magnetic latch plane",
    )
    ctx.expect_overlap(
        top_filter,
        body,
        axes="xy",
        min_overlap=0.25,
        name="top mesh filter spans the roof vent opening",
    )

    closed_front = ctx.part_world_aabb(front_door)
    with ctx.pose({front_hinge: 1.2}):
        open_front = ctx.part_world_aabb(front_door)
    ctx.check(
        "front door opens outward from the right edge",
        closed_front is not None and open_front is not None and open_front[0][1] < closed_front[0][1] - 0.10,
        details=f"closed={closed_front}, open={open_front}",
    )

    closed_side = ctx.part_world_aabb(side_panel)
    with ctx.pose({side_hinge: 1.1}):
        open_side = ctx.part_world_aabb(side_panel)
    ctx.check(
        "side panel opens outward from the rear edge",
        closed_side is not None and open_side is not None and open_side[0][0] < closed_side[0][0] - 0.10,
        details=f"closed={closed_side}, open={open_side}",
    )

    closed_top = ctx.part_world_aabb(top_filter)
    with ctx.pose({top_hinge: 1.0}):
        open_top = ctx.part_world_aabb(top_filter)
    ctx.check(
        "top mesh filter lifts upward from the front edge",
        closed_top is not None and open_top is not None and open_top[1][2] > closed_top[1][2] + 0.12,
        details=f"closed={closed_top}, open={open_top}",
    )

    return ctx.report()


object_model = build_object_model()
