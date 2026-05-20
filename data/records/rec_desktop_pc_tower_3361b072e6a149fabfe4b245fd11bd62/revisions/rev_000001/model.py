from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="mini_itx_cube_case")

    steel = model.material("satin_black_powder_coat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_steel = model.material("shadow_black_metal", rgba=(0.002, 0.002, 0.003, 1.0))
    vent_dark = model.material("black_recessed_vent", rgba=(0.0, 0.0, 0.0, 1.0))
    glass = model.material("smoked_tempered_glass", rgba=(0.08, 0.14, 0.18, 0.38))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    pcb_green = model.material("dark_pcb_green", rgba=(0.02, 0.16, 0.09, 1.0))

    width = 0.280
    depth = 0.300
    height = 0.260
    wall = 0.012
    side_x = width / 2.0
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    chassis = model.part("chassis")

    # Sheet-metal cube frame: a bottom tray, fixed side, front/rear walls and
    # lips around the open side and open top.
    chassis.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=steel,
        name="bottom_tray",
    )
    chassis.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, height / 2.0)),
        material=steel,
        name="front_shell",
    )
    chassis.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, rear_y - wall / 2.0, height / 2.0)),
        material=steel,
        name="rear_shell",
    )
    chassis.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-side_x + wall / 2.0, 0.0, height / 2.0)),
        material=steel,
        name="fixed_side_shell",
    )
    chassis.visual(
        Box((wall, depth, 0.028)),
        origin=Origin(xyz=(side_x - wall / 2.0, 0.0, 0.026)),
        material=steel,
        name="side_sill",
    )
    chassis.visual(
        Box((wall, depth, 0.026)),
        origin=Origin(xyz=(side_x - wall / 2.0, 0.0, height - 0.013)),
        material=steel,
        name="side_top_rail",
    )
    chassis.visual(
        Box((wall, wall, height)),
        origin=Origin(xyz=(side_x - wall / 2.0, front_y + wall / 2.0, height / 2.0)),
        material=steel,
        name="side_front_jamb",
    )
    chassis.visual(
        Box((wall, wall, height)),
        origin=Origin(xyz=(side_x - wall / 2.0, rear_y - wall / 2.0, height / 2.0)),
        material=steel,
        name="side_rear_jamb",
    )

    # A ventilated front face and rear service details make the case read as a
    # compact PC enclosure rather than a plain box.
    front_vent = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.190, 0.145),
            0.004,
            slot_size=(0.030, 0.006),
            pitch=(0.040, 0.017),
            frame=0.014,
            corner_radius=0.006,
            stagger=True,
        ),
        "front_slot_vent",
    )
    chassis.visual(
        front_vent,
        origin=Origin(xyz=(0.0, front_y - 0.001, 0.145), rpy=(pi / 2.0, 0.0, 0.0)),
        material=vent_dark,
        name="front_slot_vent",
    )
    chassis.visual(
        Box((0.080, 0.003, 0.034)),
        origin=Origin(xyz=(-0.070, rear_y + 0.001, 0.185)),
        material=vent_dark,
        name="rear_io_cutout",
    )
    chassis.visual(
        Box((0.070, 0.003, 0.070)),
        origin=Origin(xyz=(0.070, rear_y + 0.001, 0.095)),
        material=vent_dark,
        name="rear_fan_grille",
    )
    chassis.visual(
        Box((0.070, 0.003, 0.036)),
        origin=Origin(xyz=(-0.070, rear_y + 0.001, 0.060)),
        material=vent_dark,
        name="rear_psu_socket",
    )

    # Simple mounted internal cues visible through the hinged glass side.
    chassis.visual(
        Box((0.006, 0.170, 0.150)),
        origin=Origin(xyz=(-side_x + wall + 0.003, -0.020, 0.120)),
        material=dark_steel,
        name="motherboard_tray",
    )
    chassis.visual(
        Box((0.004, 0.130, 0.105)),
        origin=Origin(xyz=(-side_x + wall + 0.008, -0.020, 0.125)),
        material=pcb_green,
        name="motherboard",
    )

    # Rubber feet contact and slightly wrap the bottom tray.
    for x in (-0.095, 0.095):
        for y in (-0.105, 0.105):
            chassis.visual(
                Cylinder(radius=0.016, length=0.010),
                origin=Origin(xyz=(x, y, -0.003)),
                material=rubber,
                name=f"foot_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )

    # Parent-side knuckles for the rear top hinge.
    top_hinge_y = rear_y + 0.014
    top_hinge_z = height + 0.006
    for x in (-0.095, 0.095):
        chassis.visual(
            Cylinder(radius=0.007, length=0.065),
            origin=Origin(xyz=(x, top_hinge_y, top_hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"top_hinge_knuckle_{'rear' if x > 0 else 'front'}",
        )
        chassis.visual(
            Box((0.065, 0.017, 0.004)),
            origin=Origin(xyz=(x, rear_y + 0.006, height + 0.002)),
            material=dark_steel,
            name=f"top_hinge_leaf_{'rear' if x > 0 else 'front'}",
        )

    # Parent-side knuckles for the bottom edge of the glass side panel.
    side_hinge_x = side_x + 0.002 + 0.003
    side_hinge_z = 0.035
    for y in (-0.090, 0.090):
        chassis.visual(
            Cylinder(radius=0.007, length=0.055),
            origin=Origin(xyz=(side_hinge_x, y, side_hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"side_hinge_knuckle_{'rear' if y > 0 else 'front'}",
        )
        chassis.visual(
            Box((0.012, 0.055, 0.012)),
            origin=Origin(xyz=(side_x - 0.001, y, side_hinge_z + 0.002)),
            material=dark_steel,
            name=f"side_hinge_leaf_{'rear' if y > 0 else 'front'}",
        )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((width + 0.010, 0.300, 0.010)),
        origin=Origin(xyz=(0.0, -0.166, 0.0)),
        material=steel,
        name="top_shell",
    )
    top_panel.visual(
        Cylinder(radius=0.007, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="top_hinge_barrel",
    )
    top_panel.visual(
        Box((0.105, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.009, 0.000)),
        material=dark_steel,
        name="top_hinge_leaf",
    )
    top_vent = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.165, 0.140),
            0.002,
            slot_size=(0.026, 0.005),
            pitch=(0.036, 0.015),
            frame=0.012,
            corner_radius=0.005,
        ),
        "top_filter_slots",
    )
    top_panel.visual(
        top_vent,
        origin=Origin(xyz=(0.0, -0.165, 0.006)),
        material=vent_dark,
        name="top_filter_slots",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, top_hinge_y, top_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    side_glass = model.part("side_glass")
    glass_height = 0.178
    glass_depth = 0.286
    glass_bottom = 0.030
    side_glass.visual(
        Box((0.005, glass_depth, glass_height)),
        origin=Origin(xyz=(0.0, 0.0, glass_bottom + glass_height / 2.0)),
        material=glass,
        name="glass_pane",
    )
    side_glass.visual(
        Cylinder(radius=0.007, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="side_hinge_barrel",
    )
    side_glass.visual(
        Box((0.010, 0.085, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_steel,
        name="side_hinge_leaf",
    )
    side_glass.visual(
        Box((0.010, glass_depth, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, glass_bottom - 0.006)),
        material=dark_steel,
        name="glass_bottom_rail",
    )
    side_glass.visual(
        Box((0.010, glass_depth, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, glass_bottom + glass_height + 0.008)),
        material=dark_steel,
        name="glass_top_rail",
    )
    side_glass.visual(
        Box((0.010, 0.016, glass_height)),
        origin=Origin(xyz=(0.0, -glass_depth / 2.0 + 0.008, glass_bottom + glass_height / 2.0)),
        material=dark_steel,
        name="glass_front_rail",
    )
    side_glass.visual(
        Box((0.010, 0.016, glass_height)),
        origin=Origin(xyz=(0.0, glass_depth / 2.0 - 0.008, glass_bottom + glass_height / 2.0)),
        material=dark_steel,
        name="glass_rear_rail",
    )

    model.articulation(
        "side_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_glass,
        origin=Origin(xyz=(side_hinge_x, 0.0, side_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    top_panel = object_model.get_part("top_panel")
    side_glass = object_model.get_part("side_glass")
    top_hinge = object_model.get_articulation("top_hinge")
    side_hinge = object_model.get_articulation("side_hinge")

    ctx.expect_gap(
        top_panel,
        chassis,
        axis="z",
        positive_elem="top_shell",
        negative_elem="side_top_rail",
        min_gap=0.0005,
        max_gap=0.006,
        name="closed top panel sits just above the top rim",
    )
    ctx.expect_overlap(
        top_panel,
        chassis,
        axes="xy",
        elem_a="top_shell",
        elem_b="bottom_tray",
        min_overlap=0.18,
        name="top panel covers the square housing footprint",
    )
    ctx.expect_gap(
        side_glass,
        chassis,
        axis="x",
        positive_elem="glass_pane",
        negative_elem="side_top_rail",
        min_gap=0.001,
        max_gap=0.006,
        name="closed glass panel is proud of the side frame",
    )
    ctx.expect_overlap(
        side_glass,
        chassis,
        axes="yz",
        elem_a="glass_pane",
        elem_b="fixed_side_shell",
        min_overlap=0.16,
        name="glass panel covers the side opening vertically",
    )

    closed_top = ctx.part_world_aabb(top_panel)
    with ctx.pose({top_hinge: 1.20}):
        open_top = ctx.part_world_aabb(top_panel)
    ctx.check(
        "rear top hinge lifts the access panel",
        closed_top is not None
        and open_top is not None
        and open_top[1][2] > closed_top[1][2] + 0.10,
        details=f"closed={closed_top}, open={open_top}",
    )

    closed_glass = ctx.part_world_aabb(side_glass)
    with ctx.pose({side_hinge: 1.20}):
        open_glass = ctx.part_world_aabb(side_glass)
    ctx.check(
        "bottom side hinge swings the glass outward",
        closed_glass is not None
        and open_glass is not None
        and open_glass[1][0] > closed_glass[1][0] + 0.12
        and open_glass[1][2] < closed_glass[1][2] - 0.08,
        details=f"closed={closed_glass}, open={open_glass}",
    )

    return ctx.report()


object_model = build_object_model()
