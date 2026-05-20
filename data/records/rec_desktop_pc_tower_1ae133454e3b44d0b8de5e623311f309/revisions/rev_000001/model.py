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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mid_tower_atx_case")

    steel = Material("satin_black_steel", rgba=(0.015, 0.017, 0.019, 1.0))
    dark_steel = Material("dark_powder_coat", rgba=(0.035, 0.038, 0.042, 1.0))
    edge_trim = Material("black_edge_trim", rgba=(0.0, 0.0, 0.0, 1.0))
    glass = Material("smoked_tempered_glass", rgba=(0.08, 0.14, 0.18, 0.36))
    mesh_black = Material("black_perforated_mesh", rgba=(0.01, 0.012, 0.013, 1.0))
    tray_metal = Material("galvanized_psu_tray", rgba=(0.48, 0.50, 0.52, 1.0))

    width = 0.22
    depth = 0.48
    height = 0.46
    wall = 0.016

    body = model.part("case_body")

    # Main folded sheet-steel chassis: connected panels and lips form a hollow ATX tower.
    body.visual(Box((width, depth, wall)), origin=Origin(xyz=(0.0, 0.0, wall / 2)), material=steel, name="bottom_panel")
    body.visual(Box((width, depth, wall)), origin=Origin(xyz=(0.0, 0.0, height - wall / 2)), material=steel, name="top_panel")
    body.visual(Box((wall, depth, height)), origin=Origin(xyz=(-width / 2 + wall / 2, 0.0, height / 2)), material=steel, name="solid_side_panel")

    # Rear wall is split around a real lower PSU tray opening.
    body.visual(Box((width, wall, height - 0.16)), origin=Origin(xyz=(0.0, depth / 2 - wall / 2, 0.16 + (height - 0.16) / 2)), material=steel, name="rear_upper_panel")
    body.visual(Box((0.028, wall, 0.16)), origin=Origin(xyz=(-width / 2 + 0.014, depth / 2 - wall / 2, 0.08)), material=steel, name="rear_post_0")
    body.visual(Box((0.028, wall, 0.16)), origin=Origin(xyz=(width / 2 - 0.014, depth / 2 - wall / 2, 0.08)), material=steel, name="rear_post_1")
    body.visual(Box((width, wall, 0.026)), origin=Origin(xyz=(0.0, depth / 2 - wall / 2, 0.013)), material=steel, name="rear_bottom_lip")

    # Front rectangular steel aperture behind the hinged mesh door.
    body.visual(Box((0.020, wall, height)), origin=Origin(xyz=(-width / 2 + 0.010, -depth / 2 + wall / 2, height / 2)), material=steel, name="front_left_post")
    body.visual(Box((0.020, wall, height)), origin=Origin(xyz=(width / 2 - 0.010, -depth / 2 + wall / 2, height / 2)), material=steel, name="front_right_post")
    body.visual(Box((width, wall, 0.030)), origin=Origin(xyz=(0.0, -depth / 2 + wall / 2, height - 0.015)), material=steel, name="front_top_lip")
    body.visual(Box((width, wall, 0.040)), origin=Origin(xyz=(0.0, -depth / 2 + wall / 2, 0.020)), material=steel, name="front_bottom_lip")

    # Right-side rails leave a window opening for the glass side panel.
    body.visual(Box((wall, depth, 0.028)), origin=Origin(xyz=(width / 2 - wall / 2, 0.0, height - 0.030)), material=steel, name="side_top_rail")
    body.visual(Box((wall, depth, 0.030)), origin=Origin(xyz=(width / 2 - wall / 2, 0.0, 0.055)), material=steel, name="side_bottom_rail")
    body.visual(Box((wall, 0.038, height)), origin=Origin(xyz=(width / 2 - wall / 2, depth / 2 - 0.019, height / 2)), material=steel, name="side_rear_hinge_post")

    # Fixed PSU tray guide rails inside the base bay.
    body.visual(Box((0.010, 0.285, 0.022)), origin=Origin(xyz=(-0.092, 0.095, 0.092)), material=dark_steel, name="psu_guide_0")
    body.visual(Box((0.010, 0.285, 0.022)), origin=Origin(xyz=(0.092, 0.095, 0.092)), material=dark_steel, name="psu_guide_1")

    # Interleaved hinge knuckles and leaves fixed to the steel body.
    body.visual(Cylinder(radius=0.007, length=0.095), origin=Origin(xyz=(-width / 2 - 0.008, -depth / 2 - 0.014, height / 2 - 0.145)), material=edge_trim, name="front_hinge_knuckle_0")
    body.visual(Cylinder(radius=0.007, length=0.095), origin=Origin(xyz=(-width / 2 - 0.008, -depth / 2 - 0.014, height / 2 + 0.145)), material=edge_trim, name="front_hinge_knuckle_1")
    body.visual(Box((0.012, 0.020, 0.095)), origin=Origin(xyz=(-width / 2 - 0.003, -depth / 2 - 0.006, height / 2 - 0.145)), material=edge_trim, name="front_hinge_leaf_0")
    body.visual(Box((0.012, 0.020, 0.095)), origin=Origin(xyz=(-width / 2 - 0.003, -depth / 2 - 0.006, height / 2 + 0.145)), material=edge_trim, name="front_hinge_leaf_1")
    body.visual(Box((0.012, 0.020, 0.180)), origin=Origin(xyz=(-width / 2 - 0.003, -depth / 2 - 0.006, height / 2)), material=edge_trim, name="front_hinge_leaf_center")

    side_hinge_x = width / 2 + 0.014
    side_hinge_y = depth / 2 - 0.018
    body.visual(Cylinder(radius=0.007, length=0.095), origin=Origin(xyz=(side_hinge_x, side_hinge_y, height / 2 - 0.145)), material=edge_trim, name="side_hinge_knuckle_0")
    body.visual(Cylinder(radius=0.007, length=0.095), origin=Origin(xyz=(side_hinge_x, side_hinge_y, height / 2 + 0.145)), material=edge_trim, name="side_hinge_knuckle_1")
    body.visual(Box((0.024, 0.016, 0.095)), origin=Origin(xyz=(width / 2 + 0.003, side_hinge_y, height / 2 - 0.145)), material=edge_trim, name="side_hinge_leaf_0")
    body.visual(Box((0.024, 0.016, 0.095)), origin=Origin(xyz=(width / 2 + 0.003, side_hinge_y, height / 2 + 0.145)), material=edge_trim, name="side_hinge_leaf_1")
    body.visual(Box((0.024, 0.016, 0.180)), origin=Origin(xyz=(width / 2 + 0.003, side_hinge_y, height / 2)), material=edge_trim, name="side_hinge_leaf_center")

    # Front mesh door, framed by a perforated steel sheet and carried by a left-edge hinge.
    front_door = model.part("front_mesh_door")
    door_width = 0.230
    door_height = 0.430
    mesh_face = mesh_from_geometry(
        PerforatedPanelGeometry(
            (door_width, door_height),
            0.006,
            hole_diameter=0.0065,
            pitch=(0.014, 0.014),
            frame=0.014,
            corner_radius=0.004,
            stagger=True,
        ),
        "front_mesh_face",
    )
    door_hinge_clearance = 0.012
    front_door.visual(
        mesh_face,
        origin=Origin(xyz=(door_hinge_clearance + door_width / 2, -0.006, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=mesh_black,
        name="mesh_face",
    )
    front_door.visual(Box((door_width, 0.008, 0.020)), origin=Origin(xyz=(door_hinge_clearance + door_width / 2, -0.006, door_height / 2 - 0.010)), material=edge_trim, name="top_frame")
    front_door.visual(Box((door_width, 0.008, 0.020)), origin=Origin(xyz=(door_hinge_clearance + door_width / 2, -0.006, -door_height / 2 + 0.010)), material=edge_trim, name="bottom_frame")
    front_door.visual(Box((0.020, 0.008, door_height)), origin=Origin(xyz=(door_hinge_clearance + door_width - 0.010, -0.006, 0.0)), material=edge_trim, name="latch_stile")
    front_door.visual(Box((0.015, 0.008, door_height)), origin=Origin(xyz=(door_hinge_clearance + 0.0075, -0.006, 0.0)), material=edge_trim, name="hinge_stile")
    front_door.visual(Box((0.022, 0.006, 0.180)), origin=Origin(xyz=(0.011, -0.005, 0.0)), material=edge_trim, name="front_hinge_leaf")
    front_door.visual(Cylinder(radius=0.007, length=0.180), origin=Origin(xyz=(0.0, -0.014, 0.0)), material=edge_trim, name="front_hinge_barrel")

    # Smoked tempered-glass side panel with black metal perimeter frame.
    side_panel = model.part("side_glass_panel")
    side_len = 0.420
    side_height = 0.390
    side_panel.visual(Box((side_len - 0.052, 0.005, side_height - 0.060)), origin=Origin(xyz=(0.230, 0.004, 0.0)), material=glass, name="glass_pane")
    side_hinge_clearance = 0.020
    side_panel.visual(Box((side_len - side_hinge_clearance, 0.009, 0.024)), origin=Origin(xyz=(side_hinge_clearance + (side_len - side_hinge_clearance) / 2, 0.004, side_height / 2 - 0.012)), material=edge_trim, name="top_rail")
    side_panel.visual(Box((side_len - side_hinge_clearance, 0.009, 0.024)), origin=Origin(xyz=(side_hinge_clearance + (side_len - side_hinge_clearance) / 2, 0.004, -side_height / 2 + 0.012)), material=edge_trim, name="bottom_rail")
    side_panel.visual(Box((0.025, 0.009, side_height)), origin=Origin(xyz=(side_hinge_clearance + 0.0125, 0.004, 0.0)), material=edge_trim, name="rear_stile")
    side_panel.visual(Box((0.025, 0.009, side_height)), origin=Origin(xyz=(side_len - 0.0125, 0.004, 0.0)), material=edge_trim, name="front_stile")
    side_panel.visual(Box((0.024, 0.006, 0.180)), origin=Origin(xyz=(0.012, 0.004, 0.0)), material=edge_trim, name="side_hinge_leaf")
    side_panel.visual(Cylinder(radius=0.007, length=0.180), origin=Origin(xyz=(0.0, 0.012, 0.0)), material=edge_trim, name="side_hinge_barrel")

    # Sliding power-supply tray: rails and a rear pull flange remain one connected sheet-metal part.
    tray = model.part("psu_tray")
    tray.visual(Box((0.165, 0.300, 0.012)), origin=Origin(xyz=(0.0, -0.150, 0.0)), material=tray_metal, name="tray_shelf")
    tray.visual(Box((0.010, 0.300, 0.026)), origin=Origin(xyz=(-0.070, -0.150, 0.016)), material=tray_metal, name="tray_rail_0")
    tray.visual(Box((0.010, 0.300, 0.026)), origin=Origin(xyz=(0.070, -0.150, 0.016)), material=tray_metal, name="tray_rail_1")
    tray.visual(Box((0.190, 0.010, 0.082)), origin=Origin(xyz=(0.0, 0.005, 0.020)), material=tray_metal, name="rear_pull_flange")
    tray.visual(Box((0.070, 0.006, 0.016)), origin=Origin(xyz=(0.0, 0.012, 0.020)), material=edge_trim, name="pull_grip")

    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(-width / 2 - 0.008, -depth / 2 - 0.014, height / 2)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=0.0, upper=1.75),
    )

    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_panel,
        origin=Origin(xyz=(side_hinge_x, side_hinge_y, height / 2), rpy=(0.0, 0.0, -math.pi / 2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.4, lower=0.0, upper=1.65),
    )

    model.articulation(
        "psu_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, depth / 2 + 0.006, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("case_body")
    front_door = object_model.get_part("front_mesh_door")
    side_panel = object_model.get_part("side_glass_panel")
    tray = object_model.get_part("psu_tray")
    front_hinge = object_model.get_articulation("front_door_hinge")
    side_hinge = object_model.get_articulation("side_panel_hinge")
    tray_slide = object_model.get_articulation("psu_tray_slide")

    ctx.expect_gap(
        body,
        front_door,
        axis="y",
        min_gap=0.004,
        positive_elem="front_left_post",
        negative_elem="mesh_face",
        name="front mesh door sits proud of the steel frame",
    )
    ctx.expect_gap(
        side_panel,
        body,
        axis="x",
        min_gap=0.004,
        positive_elem="glass_pane",
        negative_elem="side_top_rail",
        name="glass panel is outside the right-side steel rail",
    )
    ctx.expect_within(
        tray,
        body,
        axes="x",
        margin=0.010,
        inner_elem="tray_shelf",
        outer_elem="rear_bottom_lip",
        name="psu tray is centered in the rear bay",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        min_overlap=0.12,
        elem_a="tray_shelf",
        elem_b="psu_guide_0",
        name="closed psu tray is retained by the base guide",
    )

    rest_front_aabb = ctx.part_element_world_aabb(front_door, elem="latch_stile")
    rest_side_aabb = ctx.part_element_world_aabb(side_panel, elem="front_stile")
    rest_tray_pos = ctx.part_world_position(tray)

    with ctx.pose({front_hinge: 1.25}):
        opened_front_aabb = ctx.part_element_world_aabb(front_door, elem="latch_stile")
    ctx.check(
        "front mesh door swings outward on the left hinge",
        rest_front_aabb is not None
        and opened_front_aabb is not None
        and opened_front_aabb[0][1] < rest_front_aabb[0][1] - 0.10,
        details=f"rest={rest_front_aabb}, opened={opened_front_aabb}",
    )

    with ctx.pose({side_hinge: 1.20}):
        opened_side_aabb = ctx.part_element_world_aabb(side_panel, elem="front_stile")
    ctx.check(
        "side glass panel swings outward from the rear edge",
        rest_side_aabb is not None
        and opened_side_aabb is not None
        and opened_side_aabb[1][0] > rest_side_aabb[1][0] + 0.10,
        details=f"rest={rest_side_aabb}, opened={opened_side_aabb}",
    )

    with ctx.pose({tray_slide: 0.180}):
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.06,
            elem_a="tray_shelf",
            elem_b="psu_guide_0",
            name="extended psu tray keeps retained insertion",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "psu tray slides rearward from the base",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] > rest_tray_pos[1] + 0.15,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
