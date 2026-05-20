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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_htpc_chassis")

    anodized = Material("satin_black_anodized", rgba=(0.015, 0.016, 0.017, 1.0))
    dark_plastic = Material("dark_front_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    brushed = Material("brushed_graphite", rgba=(0.34, 0.35, 0.36, 1.0))
    bay_black = Material("recessed_bay_black", rgba=(0.005, 0.006, 0.007, 1.0))
    rail_metal = Material("bare_slide_rail", rgba=(0.55, 0.57, 0.58, 1.0))
    smoked = Material("smoked_acrylic", rgba=(0.02, 0.025, 0.03, 0.88))
    blue_led = Material("soft_blue_led", rgba=(0.1, 0.35, 0.9, 1.0))
    rubber = Material("matte_rubber", rgba=(0.008, 0.008, 0.007, 1.0))

    width = 0.430
    depth = 0.330
    height = 0.090
    wall = 0.006

    chassis = model.part("chassis")

    # Structural tray and shell.  The central top is intentionally open; the
    # separate top_panel rides over it and slides rearward for access.
    chassis.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=anodized,
        name="bottom_tray",
    )
    chassis.visual(
        Box((wall, depth, height - wall)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, 0.048)),
        material=anodized,
        name="side_wall_0",
    )
    chassis.visual(
        Box((wall, depth, height - wall)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, 0.048)),
        material=anodized,
        name="side_wall_1",
    )
    chassis.visual(
        Box((width, wall, 0.078)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, 0.045)),
        material=anodized,
        name="rear_wall",
    )

    # Front fascia is modeled as a connected frame with a drive-bay aperture.
    front_y = -depth / 2.0 - 0.001
    chassis.visual(
        Box((width, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, front_y, 0.012)),
        material=dark_plastic,
        name="front_lower_lip",
    )
    chassis.visual(
        Box((width, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, front_y, 0.084)),
        material=dark_plastic,
        name="front_upper_lip",
    )
    chassis.visual(
        Box((0.010, 0.008, height)),
        origin=Origin(xyz=(-width / 2.0 + 0.005, front_y, height / 2.0)),
        material=dark_plastic,
        name="front_side_post",
    )
    chassis.visual(
        Box((0.012, 0.008, height)),
        origin=Origin(xyz=(0.121, front_y, height / 2.0)),
        material=dark_plastic,
        name="hinge_post",
    )
    # Right-side control panel is segmented around a real button clearance so
    # the articulated push button can depress into the fascia instead of
    # passing through a solid plate.
    chassis.visual(
        Box((0.088, 0.008, 0.028)),
        origin=Origin(xyz=(0.170, front_y, 0.020)),
        material=dark_plastic,
        name="control_panel_lower",
    )
    chassis.visual(
        Box((0.088, 0.008, 0.018)),
        origin=Origin(xyz=(0.170, front_y, 0.075)),
        material=dark_plastic,
        name="control_panel_upper",
    )
    chassis.visual(
        Box((0.024, 0.008, 0.078)),
        origin=Origin(xyz=(0.136, front_y, 0.045)),
        material=dark_plastic,
        name="control_panel_side_0",
    )
    chassis.visual(
        Box((0.030, 0.008, 0.078)),
        origin=Origin(xyz=(0.200, front_y, 0.045)),
        material=dark_plastic,
        name="control_panel_side_1",
    )

    # Recessed drive cage and two slim optical-style bay faces behind the door.
    chassis.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(-0.181, -0.156, 0.048)),
        material=bay_black,
        name="bay_side_0",
    )
    chassis.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(0.107, -0.156, 0.048)),
        material=bay_black,
        name="bay_side_1",
    )
    chassis.visual(
        Box((0.288, 0.012, 0.018)),
        origin=Origin(xyz=(-0.037, -0.156, 0.064)),
        material=brushed,
        name="drive_bay_upper",
    )
    chassis.visual(
        Box((0.288, 0.012, 0.018)),
        origin=Origin(xyz=(-0.037, -0.156, 0.032)),
        material=brushed,
        name="drive_bay_lower",
    )
    chassis.visual(
        Box((0.288, 0.013, 0.004)),
        origin=Origin(xyz=(-0.037, -0.156, 0.048)),
        material=bay_black,
        name="bay_divider",
    )

    # Exposed rails which guide the removable top cover.
    chassis.visual(
        Box((0.014, 0.292, 0.006)),
        origin=Origin(xyz=(-0.181, 0.000, 0.09275)),
        material=rail_metal,
        name="slide_rail_0",
    )
    chassis.visual(
        Box((0.036, 0.292, 0.004)),
        origin=Origin(xyz=(-0.197, 0.000, 0.09175)),
        material=anodized,
        name="rail_shelf_0",
    )
    chassis.visual(
        Box((0.014, 0.292, 0.006)),
        origin=Origin(xyz=(0.181, 0.000, 0.09275)),
        material=rail_metal,
        name="slide_rail_1",
    )
    chassis.visual(
        Box((0.036, 0.292, 0.004)),
        origin=Origin(xyz=(0.197, 0.000, 0.09175)),
        material=anodized,
        name="rail_shelf_1",
    )
    chassis.visual(
        Box((0.372, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.151, 0.09325)),
        material=rail_metal,
        name="front_slide_stop",
    )

    # Static hinge leaves and interleaved chassis-side knuckles on the door's
    # right edge.
    hinge_x = 0.115
    hinge_y = -depth / 2.0 - 0.011
    chassis.visual(
        Box((0.012, 0.006, 0.078)),
        origin=Origin(xyz=(hinge_x + 0.009, hinge_y + 0.004, 0.048)),
        material=rail_metal,
        name="hinge_leaf",
    )
    for idx, (z, length) in enumerate(((0.0125, 0.011), (0.048, 0.017), (0.0835, 0.011))):
        chassis.visual(
            Cylinder(radius=0.005, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=rail_metal,
            name=f"hinge_knuckle_{idx}",
        )

    # Small feet make the chassis read as a real shelf component rather than a
    # raw box resting directly on the world plane.
    for idx, (x, y) in enumerate(
        (
            (-0.170, -0.120),
            (0.170, -0.120),
            (-0.170, 0.120),
            (0.170, 0.120),
        )
    ):
        chassis.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber,
            name=f"foot_{idx}",
        )

    # Sliding top panel with side flanges and a perforated ventilation insert.
    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((0.398, 0.310, 0.010)),
        origin=Origin(),
        material=anodized,
        name="lid_sheet",
    )
    top_panel.visual(
        Box((0.010, 0.300, 0.014)),
        origin=Origin(xyz=(-0.204, 0.0, 0.000)),
        material=anodized,
        name="side_flange_0",
    )
    top_panel.visual(
        Box((0.010, 0.300, 0.014)),
        origin=Origin(xyz=(0.204, 0.0, 0.000)),
        material=anodized,
        name="side_flange_1",
    )
    vent_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.220, 0.105),
            0.002,
            slot_size=(0.030, 0.006),
            pitch=(0.040, 0.018),
            frame=0.010,
            corner_radius=0.004,
            stagger=True,
        ),
        "top_vent_slots",
    )
    top_panel.visual(
        vent_mesh,
        origin=Origin(xyz=(-0.030, 0.020, 0.006)),
        material=bay_black,
        name="top_vent",
    )
    top_panel.visual(
        Box((0.090, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.151, 0.0063)),
        material=brushed,
        name="finger_grip",
    )

    model.articulation(
        "chassis_to_top_panel",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, 0.000, 0.10075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    # Front door.  Its local frame is the hinge pin line; the panel extends
    # leftward from the right-edge hinge and swings outward toward the viewer.
    front_door = model.part("front_door")
    front_door.visual(
        Box((0.286, 0.009, 0.068)),
        origin=Origin(xyz=(-0.155, 0.0, 0.0)),
        material=smoked,
        name="door_panel",
    )
    front_door.visual(
        Box((0.011, 0.006, 0.060)),
        origin=Origin(xyz=(-0.293, -0.007, 0.0)),
        material=brushed,
        name="pull_lip",
    )
    for idx, z in enumerate((-0.020, 0.020)):
        front_door.visual(
            Cylinder(radius=0.005, length=0.019),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rail_metal,
            name=f"door_knuckle_{idx}",
        )
        front_door.visual(
            Box((0.014, 0.006, 0.018)),
            origin=Origin(xyz=(-0.007, 0.003, z)),
            material=rail_metal,
            name=f"door_hinge_tab_{idx}",
        )

    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    # A distinct user control: the front power button moves inward on a short
    # prismatic stroke, matching the real tactile switch beneath the fascia.
    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="button_cap",
    )
    power_button.visual(
        Box((0.032, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=bay_black,
        name="button_slider",
    )
    power_button.visual(
        Cylinder(radius=0.004, length=0.0015),
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_led,
        name="button_led",
    )
    model.articulation(
        "chassis_to_power_button",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=power_button,
        origin=Origin(xyz=(0.169, -depth / 2.0 - 0.008, 0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    top_panel = object_model.get_part("top_panel")
    front_door = object_model.get_part("front_door")
    power_button = object_model.get_part("power_button")
    top_slide = object_model.get_articulation("chassis_to_top_panel")
    door_hinge = object_model.get_articulation("chassis_to_front_door")
    button_slide = object_model.get_articulation("chassis_to_power_button")

    ctx.expect_gap(
        top_panel,
        chassis,
        axis="z",
        positive_elem="lid_sheet",
        negative_elem="slide_rail_0",
        min_gap=-0.0001,
        max_gap=0.001,
        name="top lid rides just above left rail",
    )
    ctx.expect_gap(
        top_panel,
        chassis,
        axis="z",
        positive_elem="lid_sheet",
        negative_elem="slide_rail_1",
        min_gap=-0.0001,
        max_gap=0.001,
        name="top lid rides just above right rail",
    )
    ctx.expect_overlap(
        top_panel,
        chassis,
        axes="y",
        elem_a="lid_sheet",
        elem_b="slide_rail_0",
        min_overlap=0.200,
        name="closed top is retained on rails",
    )

    rest_top_pos = ctx.part_world_position(top_panel)
    with ctx.pose({top_slide: 0.160}):
        ctx.expect_overlap(
            top_panel,
            chassis,
            axes="y",
            elem_a="lid_sheet",
            elem_b="slide_rail_0",
            min_overlap=0.070,
            name="slid top keeps rearward rail engagement",
        )
        extended_top_pos = ctx.part_world_position(top_panel)
    ctx.check(
        "top panel slides rearward",
        rest_top_pos is not None
        and extended_top_pos is not None
        and extended_top_pos[1] > rest_top_pos[1] + 0.140,
        details=f"rest={rest_top_pos}, extended={extended_top_pos}",
    )

    ctx.expect_overlap(
        front_door,
        chassis,
        axes="xz",
        elem_a="door_panel",
        elem_b="drive_bay_upper",
        min_overlap=0.014,
        name="front door covers upper drive bay",
    )
    ctx.expect_overlap(
        front_door,
        chassis,
        axes="xz",
        elem_a="door_panel",
        elem_b="drive_bay_lower",
        min_overlap=0.014,
        name="front door covers lower drive bay",
    )
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        positive_elem="drive_bay_upper",
        negative_elem="door_panel",
        min_gap=0.003,
        max_gap=0.025,
        name="closed door stands proud of bay faces",
    )
    rest_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_panel")
    ctx.check(
        "door opens outward from right hinge",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.080,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({button_slide: 0.004}):
        pressed_button_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
