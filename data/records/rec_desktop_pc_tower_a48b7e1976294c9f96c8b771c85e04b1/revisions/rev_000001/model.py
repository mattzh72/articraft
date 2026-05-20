from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extended_atx_full_tower")

    sheet = model.material("matte_black_powder_coat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_sheet = model.material("charcoal_inner_steel", rgba=(0.045, 0.047, 0.052, 1.0))
    mesh_black = model.material("black_perforated_mesh", rgba=(0.006, 0.006, 0.007, 1.0))
    smoked_glass = model.material("smoked_tempered_glass", rgba=(0.08, 0.11, 0.13, 0.38))
    brushed_metal = model.material("brushed_drive_cage_metal", rgba=(0.46, 0.47, 0.48, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    led_blue = model.material("blue_led_ring", rgba=(0.0, 0.32, 0.95, 1.0))

    width = 0.24
    depth = 0.62
    height = 0.68

    case = model.part("case")
    case.visual(
        Box((width, depth, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=sheet,
        name="bottom_plate",
    )
    case.visual(
        Box((width, 0.026, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.013, height / 2.0)),
        material=sheet,
        name="rear_panel",
    )
    case.visual(
        Box((width, 0.034, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.017, height / 2.0)),
        material=sheet,
        name="front_panel",
    )
    case.visual(
        Box((0.022, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + 0.011, 0.0, height / 2.0)),
        material=sheet,
        name="fixed_side_panel",
    )
    case.visual(
        Box((0.006, 0.43, 0.50)),
        origin=Origin(xyz=(-width / 2.0 + 0.025, -0.015, 0.345)),
        material=dark_sheet,
        name="motherboard_tray",
    )

    for x in (-width / 2.0 + 0.012, width / 2.0 - 0.012):
        case.visual(
            Box((0.024, depth, 0.024)),
            origin=Origin(xyz=(x, 0.0, height - 0.012)),
            material=sheet,
            name=f"top_side_rail_{'neg' if x < 0 else 'pos'}",
        )
    for y, name in ((-depth / 2.0 + 0.018, "top_rear_rail"), (depth / 2.0 - 0.018, "top_front_rail")):
        case.visual(
            Box((width, 0.036, 0.024)),
            origin=Origin(xyz=(0.0, y, height - 0.012)),
            material=sheet,
            name=name,
        )

    for y, y_size, name in (
        (-depth / 2.0 + 0.013, 0.026, "side_rear_post"),
        (depth / 2.0 - 0.017, 0.034, "side_front_post"),
    ):
        case.visual(
            Box((0.024, y_size, height)),
            origin=Origin(xyz=(width / 2.0 - 0.012, y, height / 2.0)),
            material=sheet,
            name=name,
        )

    case.visual(
        Box((0.024, depth, 0.020)),
        origin=Origin(xyz=(width / 2.0 - 0.012, 0.0, 0.050)),
        material=sheet,
        name="side_lower_sill",
    )

    front_mesh = PerforatedPanelGeometry(
        (0.17, 0.42),
        0.006,
        hole_diameter=0.009,
        pitch=(0.016, 0.016),
        frame=0.014,
        corner_radius=0.006,
        stagger=True,
    )
    case.visual(
        mesh_from_geometry(front_mesh, "front_intake_mesh"),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.003, 0.335), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mesh_black,
        name="front_intake_mesh",
    )

    top_vent = SlotPatternPanelGeometry(
        (0.216, 0.43),
        0.004,
        slot_size=(0.052, 0.010),
        pitch=(0.067, 0.033),
        frame=0.018,
        corner_radius=0.010,
        stagger=True,
        center=False,
    )
    case.visual(
        mesh_from_geometry(top_vent, "fixed_top_exhaust_grille"),
        origin=Origin(xyz=(0.0, 0.0, height)),
        material=mesh_black,
        name="fixed_top_exhaust_grille",
    )

    for i, z in enumerate((0.525, 0.570, 0.615)):
        case.visual(
            Box((0.172, 0.008, 0.030)),
            origin=Origin(xyz=(0.0, depth / 2.0 + 0.004, z)),
            material=dark_sheet,
            name=f"front_bay_blank_{i}",
        )

    case.visual(
        Box((0.018, 0.216, 0.020)),
        origin=Origin(xyz=(0.090, 0.168, 0.130)),
        material=sheet,
        name="lower_cage_hinge_rail",
    )
    case.visual(
        Box((0.018, 0.216, 0.020)),
        origin=Origin(xyz=(0.090, 0.168, 0.380)),
        material=sheet,
        name="upper_cage_hinge_rail",
    )
    case.visual(
        Box((0.016, 0.018, 0.250)),
        origin=Origin(xyz=(0.091, 0.066, 0.255)),
        material=sheet,
        name="drive_hinge_mount",
    )

    for i, (x, y) in enumerate(
        (
            (-0.085, -0.230),
            (0.085, -0.230),
            (-0.085, 0.230),
            (0.085, 0.230),
        )
    ):
        case.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    panel_depth = 0.540
    panel_height = 0.600
    panel_thickness = 0.014
    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((panel_thickness, 0.028, panel_height)),
        origin=Origin(xyz=(panel_thickness / 2.0, 0.014, panel_height / 2.0)),
        material=sheet,
        name="rear_frame_strip",
    )
    side_panel.visual(
        Box((panel_thickness, 0.028, panel_height)),
        origin=Origin(xyz=(panel_thickness / 2.0, panel_depth - 0.014, panel_height / 2.0)),
        material=sheet,
        name="front_frame_strip",
    )
    side_panel.visual(
        Box((panel_thickness, panel_depth, 0.026)),
        origin=Origin(xyz=(panel_thickness / 2.0, panel_depth / 2.0, 0.013)),
        material=sheet,
        name="bottom_frame_strip",
    )
    side_panel.visual(
        Box((panel_thickness, panel_depth, 0.026)),
        origin=Origin(xyz=(panel_thickness / 2.0, panel_depth / 2.0, panel_height - 0.013)),
        material=sheet,
        name="top_frame_strip",
    )
    side_panel.visual(
        Box((0.006, panel_depth - 0.040, panel_height - 0.040)),
        origin=Origin(xyz=(panel_thickness / 2.0 + 0.001, panel_depth / 2.0, panel_height / 2.0)),
        material=smoked_glass,
        name="smoked_glass_insert",
    )
    side_panel.visual(
        Box((0.018, 0.030, 0.090)),
        origin=Origin(xyz=(panel_thickness + 0.005, panel_depth - 0.022, panel_height / 2.0)),
        material=dark_sheet,
        name="front_pull_latch",
    )
    for i, z in enumerate((0.115, 0.300, 0.485)):
        side_panel.visual(
            Cylinder(radius=0.007, length=0.110),
            origin=Origin(xyz=(panel_thickness / 2.0, -0.004, z)),
            material=sheet,
            name=f"side_hinge_knuckle_{i}",
        )

    side_hinge = model.articulation(
        "case_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=case,
        child=side_panel,
        origin=Origin(xyz=(width / 2.0, -depth / 2.0 + 0.045, 0.055)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    cage_width = 0.145
    cage_depth = 0.180
    cage_height = 0.250
    drive_cage = model.part("drive_cage")
    drive_cage.visual(
        Cylinder(radius=0.006, length=cage_height),
        origin=Origin(xyz=(0.0, 0.006, cage_height / 2.0)),
        material=brushed_metal,
        name="pivot_barrel",
    )
    drive_cage.visual(
        Box((cage_width, cage_depth, 0.012)),
        origin=Origin(xyz=(-cage_width / 2.0, cage_depth / 2.0, 0.006)),
        material=brushed_metal,
        name="bottom_cage_plate",
    )
    drive_cage.visual(
        Box((cage_width, cage_depth, 0.012)),
        origin=Origin(xyz=(-cage_width / 2.0, cage_depth / 2.0, cage_height - 0.006)),
        material=brushed_metal,
        name="top_cage_plate",
    )
    for x, name in ((-0.006, "hinge_side_wall"), (-cage_width + 0.006, "far_side_wall")):
        drive_cage.visual(
            Box((0.012, cage_depth, cage_height)),
            origin=Origin(xyz=(x, cage_depth / 2.0, cage_height / 2.0)),
            material=brushed_metal,
            name=name,
        )
    for y, name in ((0.006, "rear_cage_bar"), (cage_depth - 0.006, "front_cage_bar")):
        drive_cage.visual(
            Box((cage_width, 0.012, cage_height)),
            origin=Origin(xyz=(-cage_width / 2.0, y, cage_height / 2.0)),
            material=brushed_metal,
            name=name,
        )
    for i, z in enumerate((0.060, 0.128, 0.196)):
        drive_cage.visual(
            Box((cage_width - 0.012, cage_depth - 0.018, 0.008)),
            origin=Origin(xyz=(-cage_width / 2.0, cage_depth / 2.0, z)),
            material=dark_sheet,
            name=f"drive_shelf_{i}",
        )
        drive_cage.visual(
            Box((cage_width - 0.028, 0.008, 0.044)),
            origin=Origin(xyz=(-cage_width / 2.0, cage_depth + 0.004, z + 0.020)),
            material=dark_sheet,
            name=f"drive_face_{i}",
        )

    cage_hinge = model.articulation(
        "case_to_drive_cage",
        ArticulationType.REVOLUTE,
        parent=case,
        child=drive_cage,
        origin=Origin(xyz=(0.075, 0.060, 0.130)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    cover_width = 0.210
    cover_depth = 0.440
    cover_thickness = 0.012
    top_cover = model.part("top_cover")
    cover_panel = SlotPatternPanelGeometry(
        (cover_width, cover_depth),
        cover_thickness,
        slot_size=(0.056, 0.011),
        pitch=(0.070, 0.035),
        frame=0.020,
        corner_radius=0.012,
        stagger=True,
    )
    top_cover.visual(
        mesh_from_geometry(cover_panel, "hinged_top_cover_panel"),
        origin=Origin(xyz=(0.0, cover_depth / 2.0, cover_thickness / 2.0)),
        material=sheet,
        name="vented_cover_panel",
    )
    top_cover.visual(
        Box((cover_width, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, cover_depth - 0.009, cover_thickness + 0.006)),
        material=dark_sheet,
        name="raised_front_lip",
    )
    top_cover.visual(
        Box((0.090, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, cover_depth + 0.003, cover_thickness + 0.005)),
        material=dark_sheet,
        name="finger_tab",
    )
    top_cover.visual(
        Cylinder(radius=0.007, length=cover_width),
        origin=Origin(xyz=(0.0, -0.004, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sheet,
        name="top_hinge_barrel",
    )

    top_hinge = model.articulation(
        "case_to_top_cover",
        ArticulationType.REVOLUTE,
        parent=case,
        child=top_cover,
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.090, height + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=1.20),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=led_blue,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mesh_black,
        name="button_stem",
    )
    button_joint = model.articulation(
        "case_to_power_button",
        ArticulationType.PRISMATIC,
        parent=case,
        child=power_button,
        origin=Origin(xyz=(0.075, depth / 2.0, 0.655)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.006),
    )

    # Keep references alive for linters and readable motion intent in the script.
    _ = (side_hinge, cage_hinge, top_hinge, button_joint)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    side_panel = object_model.get_part("side_panel")
    drive_cage = object_model.get_part("drive_cage")
    top_cover = object_model.get_part("top_cover")
    power_button = object_model.get_part("power_button")

    side_hinge = object_model.get_articulation("case_to_side_panel")
    cage_hinge = object_model.get_articulation("case_to_drive_cage")
    top_hinge = object_model.get_articulation("case_to_top_cover")
    button_joint = object_model.get_articulation("case_to_power_button")

    ctx.expect_gap(
        side_panel,
        case,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="closed side panel sits on the case side seam",
    )
    ctx.expect_overlap(
        side_panel,
        case,
        axes="yz",
        min_overlap=0.50,
        name="side panel covers the tall side opening",
    )
    ctx.expect_within(
        drive_cage,
        case,
        axes="xyz",
        margin=0.002,
        name="drive cage is contained inside the tower when latched",
    )
    ctx.expect_gap(
        top_cover,
        case,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem="vented_cover_panel",
        negative_elem="fixed_top_exhaust_grille",
        name="top exhaust cover rests on the top exhaust grille",
    )
    ctx.expect_gap(
        power_button,
        case,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="button_cap",
        negative_elem="front_panel",
        name="power button cap is mounted on the front face",
    )

    closed_side_aabb = ctx.part_world_aabb(side_panel)
    with ctx.pose({side_hinge: 1.25}):
        open_side_aabb = ctx.part_world_aabb(side_panel)
    ctx.check(
        "side panel swings outward from rear hinges",
        closed_side_aabb is not None
        and open_side_aabb is not None
        and open_side_aabb[1][0] > closed_side_aabb[1][0] + 0.12,
        details=f"closed={closed_side_aabb}, open={open_side_aabb}",
    )

    closed_cage_aabb = ctx.part_world_aabb(drive_cage)
    with ctx.pose({side_hinge: 1.25, cage_hinge: 1.10}):
        open_cage_aabb = ctx.part_world_aabb(drive_cage)
    ctx.check(
        "drive cage pivots out through the side opening",
        closed_cage_aabb is not None
        and open_cage_aabb is not None
        and open_cage_aabb[1][0] > closed_cage_aabb[1][0] + 0.08,
        details=f"closed={closed_cage_aabb}, open={open_cage_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(top_cover)
    with ctx.pose({top_hinge: 1.0}):
        open_cover_aabb = ctx.part_world_aabb(top_cover)
    ctx.check(
        "top exhaust cover hinges upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.15,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    rest_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({button_joint: 0.006}):
        pressed_button_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
