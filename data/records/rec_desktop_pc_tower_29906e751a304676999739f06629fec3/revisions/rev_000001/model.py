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
    model = ArticulatedObject(name="mid_tower_atx_pc")

    # Real mid-tower proportions: roughly 460 mm tall, 205 mm wide,
    # and 465 mm deep.  +X is the front, +Y is the removable side panel,
    # and +Z is up from the desk.
    case_w = 0.205
    case_d = 0.465
    case_h = 0.460
    steel_t = 0.006
    front_t = 0.018
    front_x = case_d / 2.0 + front_t

    steel = model.material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.01, 0.011, 0.012, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    port_blue = model.material("port_blue", rgba=(0.0, 0.10, 0.55, 1.0))
    led_green = model.material("led_green", rgba=(0.0, 0.85, 0.24, 1.0))

    case = model.part("case")

    # Sheet-metal chassis and fixed panels.  The +Y side is intentionally open
    # except for flanges because the removable side panel is a separate hinged
    # part.
    case.visual(
        Box((case_d, case_w, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, steel_t / 2.0)),
        material=steel,
        name="bottom_panel",
    )
    case.visual(
        Box((case_d, case_w, steel_t)),
        origin=Origin(xyz=(0.0, 0.0, case_h - steel_t / 2.0)),
        material=steel,
        name="top_panel",
    )
    case.visual(
        Box((steel_t, case_w, case_h)),
        origin=Origin(xyz=(-case_d / 2.0 + steel_t / 2.0, 0.0, case_h / 2.0)),
        material=steel,
        name="rear_panel",
    )
    case.visual(
        Box((case_d, steel_t, case_h)),
        origin=Origin(xyz=(0.0, -case_w / 2.0 + steel_t / 2.0, case_h / 2.0)),
        material=steel,
        name="fixed_side_panel",
    )
    # Rolled front and rear side flanges for the removable panel to close over.
    case.visual(
        Box((0.030, steel_t, case_h - 0.050)),
        origin=Origin(xyz=(case_d / 2.0 - 0.015, case_w / 2.0 - steel_t / 2.0, case_h / 2.0)),
        material=steel,
        name="front_side_flange",
    )
    case.visual(
        Box((0.030, steel_t, case_h - 0.050)),
        origin=Origin(xyz=(-case_d / 2.0 + 0.015, case_w / 2.0 - steel_t / 2.0, case_h / 2.0)),
        material=steel,
        name="rear_side_flange",
    )

    # Black front bezel with a true rectangular optical-drive opening.
    bay_w = 0.166
    bay_h = 0.056
    bay_z = 0.337
    opening_bottom = bay_z - bay_h / 2.0
    opening_top = bay_z + bay_h / 2.0
    side_bar_w = (case_w - bay_w) / 2.0

    case.visual(
        Box((front_t, case_w, opening_bottom)),
        origin=Origin(xyz=(case_d / 2.0 + front_t / 2.0, 0.0, opening_bottom / 2.0)),
        material=black_plastic,
        name="front_lower_bezel",
    )
    case.visual(
        Box((front_t, case_w, case_h - opening_top)),
        origin=Origin(xyz=(case_d / 2.0 + front_t / 2.0, 0.0, (case_h + opening_top) / 2.0)),
        material=black_plastic,
        name="front_upper_bezel",
    )
    for y_sign, name in ((-1.0, "bay_side_0"), (1.0, "bay_side_1")):
        y = y_sign * (bay_w / 2.0 + side_bar_w / 2.0)
        case.visual(
            Box((front_t, side_bar_w, bay_h)),
            origin=Origin(xyz=(case_d / 2.0 + front_t / 2.0, y, bay_z)),
            material=black_plastic,
            name=name,
        )

    # Optical drive sleeve just behind the front aperture.  It is built from
    # separate walls so the tray can slide through the hollow center.
    sleeve_len = 0.190
    sleeve_x = front_x - sleeve_len / 2.0
    sleeve_w = 0.172
    sleeve_h = 0.064
    wall_t = 0.004
    case.visual(
        Box((sleeve_len, sleeve_w, wall_t)),
        origin=Origin(xyz=(sleeve_x, 0.0, bay_z - 0.019)),
        material=dark_steel,
        name="drive_floor",
    )
    case.visual(
        Box((sleeve_len, sleeve_w, wall_t)),
        origin=Origin(xyz=(sleeve_x, 0.0, bay_z + sleeve_h / 2.0 - wall_t / 2.0)),
        material=dark_steel,
        name="drive_ceiling",
    )
    for y_sign, name in ((-1.0, "drive_wall_0"), (1.0, "drive_wall_1")):
        case.visual(
            Box((sleeve_len, wall_t, sleeve_h)),
            origin=Origin(xyz=(sleeve_x, y_sign * (sleeve_w / 2.0 - wall_t / 2.0), bay_z)),
            material=dark_steel,
            name=name,
        )

    # Front ventilation slats and a few rear I/O details make the rectangular
    # tower read as a PC enclosure rather than a plain metal box.
    for i, z in enumerate((0.085, 0.110, 0.135, 0.160, 0.185, 0.210)):
        case.visual(
            Box((0.003, 0.142, 0.007)),
            origin=Origin(xyz=(front_x + 0.0015, 0.0, z)),
            material=dark_steel,
            name=f"front_vent_{i}",
        )
    case.visual(
        Box((0.003, 0.060, 0.095)),
        origin=Origin(xyz=(-case_d / 2.0 - 0.0015, -0.030, 0.300)),
        material=dark_steel,
        name="rear_io_plate",
    )
    case.visual(
        Box((0.004, 0.033, 0.020)),
        origin=Origin(xyz=(-case_d / 2.0 - 0.002, -0.036, 0.322)),
        material=port_blue,
        name="rear_usb_block",
    )
    for i, z in enumerate((0.070, 0.095, 0.120)):
        case.visual(
            Box((0.004, 0.150, 0.006)),
            origin=Origin(xyz=(-case_d / 2.0 - 0.002, 0.010, z)),
            material=dark_steel,
            name=f"rear_slot_{i}",
        )

    # Four small feet under the steel chassis.
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            case.visual(
                Box((0.055, 0.030, 0.012)),
                origin=Origin(
                    xyz=(
                        x_sign * (case_d / 2.0 - 0.060),
                        y_sign * (case_w / 2.0 - 0.035),
                        -0.006,
                    )
                ),
                material=black_plastic,
                name=f"foot_{'front' if x_sign > 0 else 'rear'}_{'side' if y_sign > 0 else 'inner'}",
            )

    # The fixed hinge pin is attached to the rear corner by small top/bottom
    # brackets.  The moving side-panel barrels capture this pin.
    hinge_x = -case_d / 2.0 - 0.010
    hinge_y = case_w / 2.0 + 0.010
    hinge_z_mid = case_h / 2.0
    case.visual(
        Cylinder(radius=0.0032, length=case_h - 0.040),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z_mid)),
        material=dark_steel,
        name="hinge_pin",
    )
    for z, name in ((0.018, "lower_hinge_bracket"), (case_h - 0.018, "upper_hinge_bracket")):
        case.visual(
            Box((0.028, 0.020, 0.024)),
            origin=Origin(xyz=(-case_d / 2.0 - 0.004, case_w / 2.0 + 0.004, z)),
            material=steel,
            name=name,
        )

    # Hinged removable side panel.  Its local frame sits on the rear hinge axis;
    # at q=0 the panel extends forward along local +X and closes over +Y.
    side_panel = model.part("side_panel")
    panel_t = 0.005
    panel_len = case_d + 0.004
    panel_h = case_h - 0.070
    panel_center_y = case_w / 2.0 + 0.0055 - hinge_y
    side_panel.visual(
        Box((panel_len, panel_t, panel_h)),
        origin=Origin(xyz=(panel_len / 2.0 + 0.006, panel_center_y, case_h / 2.0)),
        material=steel,
        name="panel_skin",
    )
    side_panel.visual(
        Box((0.020, 0.006, panel_h)),
        origin=Origin(xyz=(0.016, panel_center_y - 0.001, case_h / 2.0)),
        material=dark_steel,
        name="hinge_leaf",
    )
    for z, length, name in (
        (0.095, 0.075, "hinge_barrel_0"),
        (0.230, 0.085, "hinge_barrel_1"),
        (0.365, 0.075, "hinge_barrel_2"),
    ):
        side_panel.visual(
            Cylinder(radius=0.0085, length=length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=name,
        )
    side_panel.visual(
        Box((0.185, 0.006, 0.020)),
        origin=Origin(xyz=(case_d - 0.080, panel_center_y - 0.001, case_h / 2.0)),
        material=dark_steel,
        name="finger_recess",
    )

    model.articulation(
        "case_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=case,
        child=side_panel,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # Sliding optical-drive tray.  The part frame is at the front of the drive
    # opening; the tray shelf extends backward so it remains engaged after it
    # slides outward.
    tray = model.part("drive_tray")
    tray.visual(
        Box((0.012, 0.148, 0.036)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=tray_plastic,
        name="tray_face",
    )
    tray.visual(
        Box((0.180, 0.132, 0.006)),
        origin=Origin(xyz=(-0.090, 0.0, -0.014)),
        material=tray_plastic,
        name="tray_shelf",
    )
    for y_sign, name in ((-1.0, "tray_lip_0"), (1.0, "tray_lip_1")):
        tray.visual(
            Box((0.170, 0.006, 0.018)),
            origin=Origin(xyz=(-0.085, y_sign * 0.067, -0.006)),
            material=tray_plastic,
            name=name,
        )
    tray.visual(
        Box((0.020, 0.004, 0.004)),
        origin=Origin(xyz=(0.013, 0.052, -0.004)),
        material=dark_steel,
        name="eject_button_mark",
    )

    model.articulation(
        "case_to_drive_tray",
        ArticulationType.PRISMATIC,
        parent=case,
        child=tray,
        origin=Origin(xyz=(front_x, 0.0, bay_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.145),
    )

    # A small separate power button, because it is a visible user-facing control.
    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.004, length=0.0065),
        origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=led_green,
        name="status_led",
    )
    model.articulation(
        "case_to_power_button",
        ArticulationType.PRISMATIC,
        parent=case,
        child=power_button,
        origin=Origin(xyz=(front_x + 0.0030, 0.064, 0.247)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    side_panel = object_model.get_part("side_panel")
    tray = object_model.get_part("drive_tray")
    power_button = object_model.get_part("power_button")
    side_hinge = object_model.get_articulation("case_to_side_panel")
    tray_slide = object_model.get_articulation("case_to_drive_tray")
    button_slide = object_model.get_articulation("case_to_power_button")

    # The hinge pin is intentionally captured inside the moving hinge barrels.
    # These are local, mechanical overlaps rather than unintended broad
    # collisions.
    for barrel in ("hinge_barrel_0", "hinge_barrel_1", "hinge_barrel_2"):
        ctx.allow_overlap(
            case,
            side_panel,
            elem_a="hinge_pin",
            elem_b=barrel,
            reason="The side-panel hinge barrel intentionally surrounds the fixed steel hinge pin.",
        )
        ctx.expect_within(
            case,
            side_panel,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=barrel,
            margin=0.001,
            name=f"{barrel} captures hinge pin radially",
        )
        ctx.expect_overlap(
            case,
            side_panel,
            axes="z",
            elem_a="hinge_pin",
            elem_b=barrel,
            min_overlap=0.060,
            name=f"{barrel} shares vertical hinge length",
        )

    ctx.check(
        "side panel is rear-hinged revolute",
        side_hinge.articulation_type == ArticulationType.REVOLUTE
        and side_hinge.motion_limits is not None
        and side_hinge.motion_limits.upper >= 1.5,
        details=f"type={side_hinge.articulation_type}, limits={side_hinge.motion_limits}",
    )
    ctx.check(
        "optical tray uses a front prismatic slide",
        tray_slide.articulation_type == ArticulationType.PRISMATIC
        and tray_slide.axis == (1.0, 0.0, 0.0)
        and tray_slide.motion_limits is not None
        and tray_slide.motion_limits.upper >= 0.12,
        details=f"type={tray_slide.articulation_type}, axis={tray_slide.axis}, limits={tray_slide.motion_limits}",
    )
    ctx.check(
        "power button is a short inboard prismatic control",
        button_slide.articulation_type == ArticulationType.PRISMATIC
        and button_slide.axis == (-1.0, 0.0, 0.0)
        and button_slide.motion_limits is not None
        and 0.002 <= button_slide.motion_limits.upper <= 0.006,
        details=f"type={button_slide.articulation_type}, axis={button_slide.axis}, limits={button_slide.motion_limits}",
    )

    # At rest and at full extension the optical tray remains guided in the bay.
    ctx.expect_gap(
        tray,
        case,
        axis="z",
        positive_elem="tray_shelf",
        negative_elem="drive_floor",
        min_gap=0.0,
        max_gap=0.001,
        name="closed tray rides on drive floor",
    )
    ctx.expect_overlap(
        tray,
        case,
        axes="x",
        elem_a="tray_shelf",
        elem_b="drive_floor",
        min_overlap=0.150,
        name="closed tray is deeply inserted",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.145}):
        ctx.expect_overlap(
            tray,
            case,
            axes="x",
            elem_a="tray_shelf",
            elem_b="drive_floor",
            min_overlap=0.030,
            name="extended tray keeps retained insertion",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "optical tray extends forward",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > closed_tray_pos[0] + 0.12,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(side_panel, elem="panel_skin")
    with ctx.pose({side_hinge: 1.20}):
        open_panel_aabb = ctx.part_element_world_aabb(side_panel, elem="panel_skin")
    ctx.check(
        "side panel swings outward from the rear hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.25,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    button_rest = ctx.part_world_position(power_button)
    with ctx.pose({button_slide: 0.004}):
        button_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses into the bezel",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] < button_rest[0] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
