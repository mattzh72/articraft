from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = Material("galvanized_steel", rgba=(0.56, 0.60, 0.62, 1.0))
    darker_galv = Material("darker_galvanized_band", rgba=(0.43, 0.46, 0.48, 1.0))
    shadow = Material("dark_interior_shadow", rgba=(0.08, 0.09, 0.09, 1.0))

    width = 0.50
    depth = 0.36
    wall = 0.025
    base_height = 0.040
    top_z = 1.18
    housing_height = top_z - base_height
    front_x = depth / 2.0

    outlet_width = 0.34
    outlet_bottom = 0.72
    outlet_top = 1.08
    outlet_height = outlet_top - outlet_bottom
    lip_depth = 0.014
    lip_width = 0.035

    main_shell = model.part("main_shell")

    # Broad roof flashing/curb plate and the four connected sheet-metal walls.
    main_shell.visual(
        Box((depth + 0.22, width + 0.22, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=galvanized,
        name="roof_flashing",
    )
    main_shell.visual(
        Box((depth, wall, housing_height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, base_height + housing_height / 2.0)),
        material=galvanized,
        name="side_wall_0",
    )
    main_shell.visual(
        Box((depth, wall, housing_height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, base_height + housing_height / 2.0)),
        material=galvanized,
        name="side_wall_1",
    )
    main_shell.visual(
        Box((wall, width, housing_height)),
        origin=Origin(xyz=(-front_x + wall / 2.0, 0.0, base_height + housing_height / 2.0)),
        material=galvanized,
        name="rear_wall",
    )
    main_shell.visual(
        Box((wall, width, outlet_bottom - base_height)),
        origin=Origin(xyz=(front_x - wall / 2.0, 0.0, (base_height + outlet_bottom) / 2.0)),
        material=galvanized,
        name="front_lower_wall",
    )
    main_shell.visual(
        Box((wall, width, top_z - outlet_top)),
        origin=Origin(xyz=(front_x - wall / 2.0, 0.0, (outlet_top + top_z) / 2.0)),
        material=galvanized,
        name="front_upper_wall",
    )

    jamb_width = (width - outlet_width) / 2.0
    for idx, y in enumerate((-outlet_width / 2.0 - jamb_width / 2.0, outlet_width / 2.0 + jamb_width / 2.0)):
        main_shell.visual(
            Box((wall, jamb_width, outlet_height)),
            origin=Origin(xyz=(front_x - wall / 2.0, y, (outlet_bottom + outlet_top) / 2.0)),
            material=galvanized,
            name=f"outlet_jamb_{idx}",
        )

    # Three raised sides of the outlet frame are part of the shell; the top
    # hinge-side band is deliberately separate below.
    frame_front_x = front_x + lip_depth / 2.0
    main_shell.visual(
        Box((lip_depth, lip_width, outlet_height)),
        origin=Origin(xyz=(frame_front_x, -outlet_width / 2.0 - lip_width / 2.0, (outlet_bottom + outlet_top) / 2.0)),
        material=galvanized,
        name="outlet_side_frame_0",
    )
    main_shell.visual(
        Box((lip_depth, lip_width, outlet_height)),
        origin=Origin(xyz=(frame_front_x, outlet_width / 2.0 + lip_width / 2.0, (outlet_bottom + outlet_top) / 2.0)),
        material=galvanized,
        name="outlet_side_frame_1",
    )
    main_shell.visual(
        Box((lip_depth, outlet_width + 2.0 * lip_width, lip_width)),
        origin=Origin(xyz=(frame_front_x, 0.0, outlet_bottom - lip_width / 2.0)),
        material=galvanized,
        name="outlet_sill_frame",
    )

    # Dark return walls just inside the mouth make the vent read as a hollow duct.
    main_shell.visual(
        Box((0.12, 0.018, outlet_height)),
        origin=Origin(xyz=(front_x - 0.060, -outlet_width / 2.0 - 0.009, (outlet_bottom + outlet_top) / 2.0)),
        material=shadow,
        name="inner_return_0",
    )
    main_shell.visual(
        Box((0.12, 0.018, outlet_height)),
        origin=Origin(xyz=(front_x - 0.060, outlet_width / 2.0 + 0.009, (outlet_bottom + outlet_top) / 2.0)),
        material=shadow,
        name="inner_return_1",
    )
    main_shell.visual(
        Box((0.12, outlet_width, 0.018)),
        origin=Origin(xyz=(front_x - 0.060, 0.0, outlet_bottom + 0.009)),
        material=shadow,
        name="inner_sill",
    )
    main_shell.visual(
        Box((depth + 0.02, width + 0.02, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, top_z + 0.0125)),
        material=galvanized,
        name="top_cap",
    )

    reinforcement_band = model.part("reinforcement_band")
    band_thickness = 0.020
    band_height = 0.075
    band_z = 1.1225
    band_front_x = front_x + band_thickness
    reinforcement_band.visual(
        Box((band_thickness, 0.48, band_height)),
        origin=Origin(xyz=(front_x + band_thickness / 2.0, 0.0, band_z)),
        material=darker_galv,
        name="band_plate",
    )
    for idx, y in enumerate((-0.222, 0.222)):
        reinforcement_band.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(band_front_x + 0.003, y, band_z + 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_galv,
            name=f"band_rivet_{idx}",
        )

    flap = model.part("flap")
    hinge_x = band_front_x + 0.013
    hinge_z = 1.105
    flap_width = 0.40
    flap_height = 0.42
    flap_thickness = 0.012
    flap_panel_x = -0.010
    flap_panel_z = -0.025 - flap_height / 2.0
    flap.visual(
        Box((flap_thickness, flap_width, flap_height)),
        origin=Origin(xyz=(flap_panel_x, 0.0, flap_panel_z)),
        material=darker_galv,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.013, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_galv,
        name="rolled_hinge",
    )
    flap.visual(
        Box((0.020, flap_width, 0.026)),
        origin=Origin(xyz=(-0.002, 0.0, -0.018)),
        material=darker_galv,
        name="top_fold",
    )
    flap.visual(
        Box((0.006, 0.030, 0.300)),
        origin=Origin(xyz=(flap_panel_x + flap_thickness / 2.0 + 0.003, 0.0, flap_panel_z - 0.020)),
        material=darker_galv,
        name="pressed_rib",
    )
    flap.visual(
        Box((0.020, flap_width, 0.020)),
        origin=Origin(xyz=(-0.001, 0.0, flap_panel_z - flap_height / 2.0 + 0.006)),
        material=darker_galv,
        name="drip_fold",
    )

    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=main_shell,
        child=reinforcement_band,
        origin=Origin(),
    )
    model.articulation(
        "band_to_flap",
        ArticulationType.REVOLUTE,
        parent=reinforcement_band,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("main_shell")
    band = object_model.get_part("reinforcement_band")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("band_to_flap")

    ctx.expect_gap(
        band,
        shell,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="band_plate",
        negative_elem="front_upper_wall",
        name="reinforcement band is seated on front shell wall",
    )
    ctx.expect_gap(
        flap,
        shell,
        axis="x",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="flap_panel",
        negative_elem="outlet_side_frame_0",
        name="closed weather flap stands just proud of outlet frame",
    )
    ctx.expect_overlap(
        flap,
        shell,
        axes="z",
        min_overlap=0.30,
        elem_a="flap_panel",
        elem_b="outlet_side_frame_0",
        name="flap panel covers the outlet area in closed pose",
    )

    closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "flap swings outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.25
        and open_aabb[0][2] > closed_aabb[0][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
