from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_mini_tower_pc")

    steel = model.material("powder_coated_steel", rgba=(0.46, 0.48, 0.49, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.017, 0.020, 1.0))
    tray_plastic = model.material("satin_black_tray", rgba=(0.035, 0.037, 0.040, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.02, 0.30, 0.12, 1.0))
    label_white = model.material("white_label", rgba=(0.82, 0.86, 0.88, 1.0))

    case = model.part("case")

    # Compact mini-tower proportions: 36 cm deep, 20 cm wide, 36 cm tall.
    case.visual(Box((0.360, 0.200, 0.010)), origin=Origin(xyz=(0.000, 0.000, 0.005)), material=steel, name="bottom_panel")
    case.visual(Box((0.360, 0.200, 0.010)), origin=Origin(xyz=(0.000, 0.000, 0.355)), material=steel, name="top_panel")
    case.visual(Box((0.012, 0.200, 0.360)), origin=Origin(xyz=(-0.174, 0.000, 0.180)), material=steel, name="rear_panel")
    case.visual(Box((0.360, 0.006, 0.350)), origin=Origin(xyz=(0.000, -0.103, 0.180)), material=steel, name="fixed_side_panel")

    # Black front bezel, broken into pieces so the optical bay is a real opening.
    case.visual(Box((0.012, 0.025, 0.345)), origin=Origin(xyz=(0.186, -0.0875, 0.180)), material=black, name="front_bezel_side_0")
    case.visual(Box((0.012, 0.025, 0.345)), origin=Origin(xyz=(0.186, 0.0875, 0.180)), material=black, name="front_bezel_side_1")
    case.visual(Box((0.012, 0.150, 0.265)), origin=Origin(xyz=(0.186, 0.000, 0.1375)), material=black, name="front_bezel_lower")
    case.visual(Box((0.012, 0.150, 0.025)), origin=Origin(xyz=(0.186, 0.000, 0.3375)), material=black, name="front_bezel_top")
    case.visual(Box((0.006, 0.152, 0.006)), origin=Origin(xyz=(0.193, 0.000, 0.272)), material=dark_steel, name="bay_lower_lip")
    case.visual(Box((0.006, 0.152, 0.006)), origin=Origin(xyz=(0.193, 0.000, 0.328)), material=dark_steel, name="bay_upper_lip")

    # Drive-bay sleeve and rails, visible through the upper front bay.
    case.visual(Box((0.170, 0.156, 0.004)), origin=Origin(xyz=(0.095, 0.000, 0.326)), material=dark_steel, name="bay_sleeve_ceiling")
    case.visual(Box((0.176, 0.006, 0.006)), origin=Origin(xyz=(0.093, -0.073, 0.292)), material=steel, name="bay_side_rail_0")
    case.visual(Box((0.176, 0.006, 0.006)), origin=Origin(xyz=(0.093, 0.073, 0.292)), material=steel, name="bay_side_rail_1")

    # Subtle lower intake grille and interior components visible when the side is open.
    for i, z in enumerate((0.070, 0.095, 0.120, 0.145, 0.170)):
        case.visual(Box((0.004, 0.100, 0.006)), origin=Origin(xyz=(0.194, 0.000, z)), material=dark_steel, name=f"front_vent_{i}")

    case.visual(Box((0.210, 0.004, 0.220)), origin=Origin(xyz=(-0.010, -0.099, 0.170)), material=pcb_green, name="motherboard")
    case.visual(Box((0.105, 0.082, 0.070)), origin=Origin(xyz=(-0.118, -0.052, 0.305)), material=dark_steel, name="power_supply")
    case.visual(Box((0.115, 0.034, 0.080)), origin=Origin(xyz=(0.094, -0.086, 0.180)), material=steel, name="drive_cage")

    # Fixed hinge knuckles on the chassis side of the rear edge.
    hinge_x = -0.166
    hinge_y = 0.106
    for i, (z, length) in enumerate(((0.045, 0.070), (0.205, 0.075), (0.3475, 0.025))):
        case.visual(Cylinder(radius=0.005, length=length), origin=Origin(xyz=(hinge_x, hinge_y, z)), material=steel, name=f"case_hinge_knuckle_{i}")
        case.visual(Box((0.017, 0.008, length)), origin=Origin(xyz=(hinge_x + 0.006, hinge_y - 0.004, z)), material=steel, name=f"case_hinge_leaf_{i}")

    side_panel = model.part("side_panel")
    side_panel.visual(Box((0.335, 0.004, 0.320)), origin=Origin(xyz=(0.169, -0.003, 0.180)), material=steel, name="panel_sheet")
    side_panel.visual(Box((0.245, 0.002, 0.230)), origin=Origin(xyz=(0.188, -0.0005, 0.180)), material=dark_steel, name="pressed_panel_inset")
    side_panel.visual(Box((0.040, 0.002, 0.075)), origin=Origin(xyz=(0.315, 0.000, 0.205)), material=black, name="finger_pull")
    for i, (x, z) in enumerate(((0.030, 0.045), (0.305, 0.045), (0.030, 0.315), (0.305, 0.315))):
        side_panel.visual(Cylinder(radius=0.004, length=0.0018), origin=Origin(xyz=(x, -0.001, z), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name=f"panel_screw_{i}")

    # Moving hinge knuckles are interleaved with the fixed chassis knuckles.
    for i, (z, length) in enumerate(((0.1225, 0.075), (0.285, 0.080))):
        side_panel.visual(Cylinder(radius=0.0046, length=length), origin=Origin(xyz=(0.000, 0.000, z)), material=steel, name=f"panel_hinge_knuckle_{i}")
        side_panel.visual(Box((0.018, 0.008, length)), origin=Origin(xyz=(0.007, -0.004, z)), material=steel, name=f"panel_hinge_leaf_{i}")

    model.articulation(
        "case_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=case,
        child=side_panel,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    drive_tray = model.part("drive_tray")
    drive_tray.visual(Box((0.150, 0.125, 0.006)), origin=Origin(xyz=(-0.070, 0.000, -0.004)), material=tray_plastic, name="tray_plate")
    drive_tray.visual(Box((0.008, 0.145, 0.036)), origin=Origin(xyz=(0.004, 0.000, 0.000)), material=black, name="front_lip")
    drive_tray.visual(Box((0.142, 0.006, 0.010)), origin=Origin(xyz=(-0.067, -0.067, -0.001)), material=black, name="tray_side_rib_0")
    drive_tray.visual(Box((0.142, 0.006, 0.010)), origin=Origin(xyz=(-0.067, 0.067, -0.001)), material=black, name="tray_side_rib_1")
    drive_tray.visual(Cylinder(radius=0.026, length=0.0015), origin=Origin(xyz=(-0.074, 0.000, -0.001)), material=dark_steel, name="disc_recess")
    drive_tray.visual(Box((0.001, 0.050, 0.006)), origin=Origin(xyz=(0.0080, -0.030, 0.000)), material=label_white, name="drive_logo")

    model.articulation(
        "case_to_drive_tray",
        ArticulationType.PRISMATIC,
        parent=case,
        child=drive_tray,
        origin=Origin(xyz=(0.186, 0.000, 0.300)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.115),
    )

    power_button = model.part("power_button")
    power_button.visual(Cylinder(radius=0.011, length=0.006), origin=Origin(xyz=(0.003, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel, name="button_cap")
    model.articulation(
        "case_to_power_button",
        ArticulationType.PRISMATIC,
        parent=case,
        child=power_button,
        origin=Origin(xyz=(0.192, -0.050, 0.228)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.08, lower=0.0, upper=0.004),
    )

    eject_button = model.part("eject_button")
    eject_button.visual(Box((0.004, 0.018, 0.006)), origin=Origin(xyz=(0.002, 0.000, 0.000)), material=dark_steel, name="button_face")
    model.articulation(
        "tray_to_eject_button",
        ArticulationType.PRISMATIC,
        parent=drive_tray,
        child=eject_button,
        origin=Origin(xyz=(0.008, 0.052, -0.006)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    side_panel = object_model.get_part("side_panel")
    drive_tray = object_model.get_part("drive_tray")
    power_button = object_model.get_part("power_button")
    side_hinge = object_model.get_articulation("case_to_side_panel")
    tray_slide = object_model.get_articulation("case_to_drive_tray")
    power_press = object_model.get_articulation("case_to_power_button")

    ctx.expect_gap(
        side_panel,
        case,
        axis="y",
        min_gap=0.0005,
        max_gap=0.003,
        positive_elem="panel_sheet",
        negative_elem="top_panel",
        name="closed side panel sits just outside steel case",
    )
    ctx.expect_overlap(
        side_panel,
        case,
        axes="xz",
        min_overlap=0.25,
        elem_a="panel_sheet",
        elem_b="fixed_side_panel",
        name="side access panel covers the side opening",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(side_panel, elem="panel_sheet")
    with ctx.pose({side_hinge: 1.20}):
        open_panel_aabb = ctx.part_element_world_aabb(side_panel, elem="panel_sheet")
    ctx.check(
        "side panel swings outward on vertical hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.15,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    ctx.expect_within(
        drive_tray,
        case,
        axes="y",
        inner_elem="tray_plate",
        outer_elem="bay_sleeve_ceiling",
        margin=0.001,
        name="optical tray is centered between bay rails",
    )
    ctx.expect_overlap(
        drive_tray,
        case,
        axes="x",
        min_overlap=0.09,
        elem_a="tray_plate",
        elem_b="bay_side_rail_0",
        name="closed tray remains inserted in optical bay",
    )
    rest_tray_pos = ctx.part_world_position(drive_tray)
    with ctx.pose({tray_slide: 0.115}):
        ctx.expect_overlap(
            drive_tray,
            case,
            axes="x",
            min_overlap=0.020,
            elem_a="tray_plate",
            elem_b="bay_side_rail_0",
            name="extended tray keeps retained insertion",
        )
        extended_tray_pos = ctx.part_world_position(drive_tray)
    ctx.check(
        "optical tray slides forward from top bay",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.10,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    rest_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({power_press: 0.004}):
        pressed_button_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button presses into front panel",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] < rest_button_pos[0] - 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
