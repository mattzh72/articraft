from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _front_annulus(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """A washer-shaped part in the XZ plane, extruded along the front/back Y axis."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_loading_washing_machine")

    white = model.material("warm_white_enamel", rgba=(0.93, 0.94, 0.91, 1.0))
    panel_white = model.material("slightly_glossy_white_panel", rgba=(0.98, 0.98, 0.95, 1.0))
    dark = model.material("black_rubber_shadow", rgba=(0.02, 0.022, 0.025, 1.0))
    steel = model.material("brushed_stainless_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    rail_steel = model.material("galvanized_rail_steel", rgba=(0.50, 0.52, 0.54, 1.0))
    glass = model.material("blue_tinted_door_glass", rgba=(0.35, 0.62, 0.78, 0.42))
    label_black = model.material("printed_black_labels", rgba=(0.01, 0.01, 0.012, 1.0))

    # The cabinet is a connected panel construction around a real front opening,
    # not a single solid block.  Dimensions are a typical compact front loader.
    cabinet = model.part("cabinet")
    cabinet.visual(Box((0.60, 0.65, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=white, name="bottom_panel")
    cabinet.visual(Box((0.60, 0.65, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.83)), material=white, name="top_panel")
    cabinet.visual(Box((0.04, 0.65, 0.85)), origin=Origin(xyz=(-0.30, 0.0, 0.425)), material=white, name="side_panel_0")
    cabinet.visual(Box((0.04, 0.65, 0.85)), origin=Origin(xyz=(0.30, 0.0, 0.425)), material=white, name="side_panel_1")
    cabinet.visual(Box((0.60, 0.04, 0.85)), origin=Origin(xyz=(0.0, 0.305, 0.425)), material=white, name="rear_panel")

    # Lower front apron and vertical front stiles leave the porthole area open.
    cabinet.visual(Box((0.60, 0.035, 0.14)), origin=Origin(xyz=(0.0, -0.325, 0.12)), material=panel_white, name="front_apron")
    cabinet.visual(Box((0.08, 0.035, 0.55)), origin=Origin(xyz=(-0.26, -0.325, 0.425)), material=panel_white, name="front_stile_0")
    cabinet.visual(Box((0.08, 0.035, 0.55)), origin=Origin(xyz=(0.26, -0.325, 0.425)), material=panel_white, name="front_stile_1")

    # Control fascia is split around the detergent drawer opening.
    cabinet.visual(Box((0.60, 0.035, 0.027)), origin=Origin(xyz=(0.0, -0.325, 0.723)), material=panel_white, name="control_bottom_strip")
    cabinet.visual(Box((0.60, 0.035, 0.027)), origin=Origin(xyz=(0.0, -0.325, 0.817)), material=panel_white, name="control_top_strip")
    cabinet.visual(Box((0.045, 0.035, 0.12)), origin=Origin(xyz=(-0.278, -0.325, 0.77)), material=panel_white, name="drawer_left_jamb")
    cabinet.visual(Box((0.325, 0.035, 0.12)), origin=Origin(xyz=(0.1375, -0.325, 0.77)), material=panel_white, name="control_panel")
    cabinet.visual(Box((0.17, 0.008, 0.030)), origin=Origin(xyz=(0.02, -0.345, 0.785)), material=dark, name="display_window")

    porthole_bezel = mesh_from_cadquery(_front_annulus(0.268, 0.218, 0.028), "cabinet_porthole_bezel")
    cabinet.visual(porthole_bezel, origin=Origin(xyz=(0.0, -0.348, 0.43)), material=panel_white, name="porthole_bezel")
    gasket = mesh_from_cadquery(_front_annulus(0.220, 0.178, 0.010), "door_gasket_shadow")
    cabinet.visual(gasket, origin=Origin(xyz=(0.0, -0.356, 0.43)), material=dark, name="door_gasket_shadow")

    # Fixed drawer rail channels and hinge-side brackets on the cabinet.
    cabinet.visual(Box((0.012, 0.36, 0.026)), origin=Origin(xyz=(-0.235, -0.165, 0.730)), material=rail_steel, name="drawer_channel_0")
    cabinet.visual(Box((0.012, 0.36, 0.026)), origin=Origin(xyz=(-0.105, -0.165, 0.730)), material=rail_steel, name="drawer_channel_1")
    cabinet.visual(Box((0.030, 0.050, 0.090)), origin=Origin(xyz=(-0.300, -0.350, 0.560)), material=steel, name="hinge_leaf_0")
    cabinet.visual(Box((0.030, 0.050, 0.090)), origin=Origin(xyz=(-0.300, -0.350, 0.300)), material=steel, name="hinge_leaf_1")
    cabinet.visual(Cylinder(radius=0.017, length=0.080), origin=Origin(xyz=(-0.300, -0.370, 0.560)), material=steel, name="hinge_knuckle_0")
    cabinet.visual(Cylinder(radius=0.017, length=0.080), origin=Origin(xyz=(-0.300, -0.370, 0.300)), material=steel, name="hinge_knuckle_1")
    knob_boss = mesh_from_cadquery(_front_annulus(0.038, 0.014, 0.012), "knob_panel_boss")
    cabinet.visual(knob_boss, origin=Origin(xyz=(0.18, -0.348, 0.77)), material=panel_white, name="knob_boss")
    cabinet.visual(Cylinder(radius=0.032, length=0.050), origin=Origin(xyz=(0.0, 0.260, 0.43), rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="rear_bearing")

    # The drum is a horizontal cylinder on a continuous axle.  Baffles make its
    # rotation visually apparent through the porthole.
    drum = model.part("drum")
    drum.visual(Cylinder(radius=0.205, length=0.43), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="drum_shell")
    drum.visual(Cylinder(radius=0.174, length=0.014), origin=Origin(xyz=(0.0, -0.222, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=dark, name="drum_mouth_shadow")
    drum.visual(Cylinder(radius=0.024, length=0.140), origin=Origin(xyz=(0.0, 0.140, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="axle_stub")
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radius = 0.125
        x = radius * math.cos(angle)
        z = radius * math.sin(angle)
        drum.visual(
            Box((0.040, 0.330, 0.032)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -angle, 0.0)),
            material=rail_steel,
            name=f"drum_baffle_{index}",
        )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.025, 0.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )

    # Door frame is authored around the hinge line; closed geometry extends in +X.
    door = model.part("porthole_door")
    door_ring = mesh_from_cadquery(_front_annulus(0.248, 0.164, 0.046), "door_outer_ring")
    door.visual(door_ring, origin=Origin(xyz=(0.300, -0.055, 0.0)), material=panel_white, name="door_ring")
    door.visual(
        Cylinder(radius=0.166, length=0.018),
        origin=Origin(xyz=(0.300, -0.060, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=glass,
        name="glass_bowl",
    )
    door.visual(Box((0.038, 0.030, 0.165)), origin=Origin(xyz=(0.523, -0.080, 0.0)), material=dark, name="pull_handle")
    door.visual(Cylinder(radius=0.014, length=0.118), origin=Origin(xyz=(0.0, -0.010, 0.0)), material=steel, name="hinge_barrel")
    door.visual(Box((0.070, 0.055, 0.130)), origin=Origin(xyz=(0.035, -0.030, 0.0)), material=steel, name="hinge_leaf")
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.300, -0.370, 0.43)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    detergent_drawer = model.part("detergent_drawer")
    detergent_drawer.visual(Box((0.205, 0.026, 0.066)), origin=Origin(xyz=(0.0, -0.014, 0.0)), material=panel_white, name="drawer_front")
    detergent_drawer.visual(Box((0.175, 0.362, 0.038)), origin=Origin(xyz=(0.0, 0.180, -0.006)), material=white, name="soap_tray")
    detergent_drawer.visual(Box((0.022, 0.340, 0.012)), origin=Origin(xyz=(-0.035, 0.178, -0.030)), material=rail_steel, name="drawer_runner_0")
    detergent_drawer.visual(Box((0.022, 0.340, 0.012)), origin=Origin(xyz=(0.075, 0.178, -0.030)), material=rail_steel, name="drawer_runner_1")
    detergent_drawer.visual(Box((0.120, 0.006, 0.010)), origin=Origin(xyz=(0.0, -0.030, 0.018)), material=dark, name="drawer_grip_recess")
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=detergent_drawer,
        origin=Origin(xyz=(-0.150, -0.346, 0.772)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.22),
    )

    knob_part = model.part("program_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.036,
            body_style="skirted",
            top_diameter=0.058,
            edge_radius=0.002,
            grip=KnobGrip(style="fluted", count=22, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", angle_deg=90.0, depth=0.0007),
        ),
        "program_selector_knob",
    )
    knob_part.visual(knob_mesh, origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=panel_white, name="knob_cap")
    knob_part.visual(Cylinder(radius=0.010, length=0.018), origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="knob_shaft")
    cabinet.visual(Box((0.075, 0.004, 0.004)), origin=Origin(xyz=(0.18, -0.344, 0.825)), material=label_black, name="knob_index_mark")
    model.articulation(
        "cabinet_to_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob_part,
        origin=Origin(xyz=(0.18, -0.354, 0.77)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("porthole_door")
    drawer = object_model.get_part("detergent_drawer")
    drum = object_model.get_part("drum")
    knob = object_model.get_part("program_knob")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")
    drum_axle = object_model.get_articulation("cabinet_to_drum")
    knob_joint = object_model.get_articulation("cabinet_to_knob")

    ctx.check("door is a left edge revolute hinge", door_hinge.articulation_type == ArticulationType.REVOLUTE and door_hinge.axis == (0.0, 0.0, -1.0))
    ctx.check("drum has continuous horizontal axle", drum_axle.articulation_type == ArticulationType.CONTINUOUS and drum_axle.axis == (0.0, 1.0, 0.0))
    ctx.check("drawer uses forward prismatic rails", drawer_slide.articulation_type == ArticulationType.PRISMATIC and drawer_slide.axis == (0.0, -1.0, 0.0))
    ctx.check("program selector is rotary", knob_joint.articulation_type == ArticulationType.REVOLUTE and knob_joint.axis == (0.0, -1.0, 0.0))

    ctx.expect_overlap(door, cabinet, axes="xz", elem_a="door_ring", elem_b="porthole_bezel", min_overlap=0.18, name="door ring covers the porthole bezel")
    ctx.expect_gap(cabinet, door, axis="y", positive_elem="porthole_bezel", negative_elem="door_ring", min_gap=0.001, max_gap=0.030, name="closed door sits just proud of cabinet front")
    ctx.expect_overlap(drum, cabinet, axes="xz", elem_a="drum_shell", elem_b="porthole_bezel", min_overlap=0.18, name="horizontal drum is centered behind porthole")

    closed_drawer_pos = ctx.part_world_position(drawer)
    ctx.expect_gap(drawer, cabinet, axis="x", positive_elem="drawer_front", negative_elem="drawer_left_jamb", min_gap=0.0, max_gap=0.010, name="drawer front clears the left jamb")
    ctx.expect_gap(cabinet, drawer, axis="x", positive_elem="control_panel", negative_elem="drawer_front", min_gap=0.0, max_gap=0.035, name="drawer front clears the right control panel")
    with ctx.pose({drawer_slide: 0.22}):
        ctx.expect_overlap(drawer, cabinet, axes="y", elem_a="soap_tray", elem_b="drawer_channel_0", min_overlap=0.11, name="extended drawer remains retained on rails")
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides forward from the front",
        closed_drawer_pos is not None and extended_drawer_pos is not None and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.18,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")
    closed_handle_y = (closed_handle_aabb[0][1] + closed_handle_aabb[1][1]) / 2.0 if closed_handle_aabb is not None else None
    with ctx.pose({door_hinge: 1.2}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="pull_handle")
        open_handle_y = (open_handle_aabb[0][1] + open_handle_aabb[1][1]) / 2.0 if open_handle_aabb is not None else None
        ctx.expect_gap(cabinet, door, axis="y", positive_elem="porthole_bezel", negative_elem="door_ring", min_gap=0.010, name="opened porthole door swings clear of the cabinet")
    ctx.check(
        "hinged door opens outward and across the front",
        closed_handle_y is not None and open_handle_y is not None and open_handle_y < closed_handle_y - 0.05,
        details=f"closed_handle_y={closed_handle_y}, open_handle_y={open_handle_y}",
    )

    rest_knob_pos = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: math.pi / 2.0, drum_axle: math.pi / 2.0}):
        turned_knob_pos = ctx.part_world_position(knob)
        ctx.expect_overlap(knob, cabinet, axes="xz", elem_a="knob_cap", elem_b="knob_boss", min_overlap=0.030, name="rotary selector stays on its panel boss")
    ctx.check("knob rotates in place on front panel", rest_knob_pos == turned_knob_pos, details=f"rest={rest_knob_pos}, turned={turned_knob_pos}")

    return ctx.report()


object_model = build_object_model()
