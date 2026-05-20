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


WIDTH = 0.598
DEPTH = 0.620
HEIGHT = 0.860
WALL = 0.024
INNER_WIDTH = WIDTH - 2.0 * WALL
TUB_REAR_Y = -0.282
TUB_FRONT_Y = 0.258
TUB_DEPTH = TUB_FRONT_Y - TUB_REAR_Y
BOTTOM_Z = 0.060
TOP_Z = 0.830
TUB_HEIGHT = TOP_Z - BOTTOM_Z
DOOR_THICKNESS = 0.032
DOOR_HEIGHT = 0.758
HINGE_Y = 0.278
HINGE_Z = 0.055

LOWER_RACK_TRAVEL = 0.205
UPPER_RACK_TRAVEL = 0.190
THIRD_TRAY_TRAVEL = 0.240
BUTTON_TRAVEL = 0.004
GLASS_RACK_DROP = 1.35


def _add_cabinet_shell(model: ArticulatedObject):
    cabinet = model.part("cabinet")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    tub_gray = model.material("tub_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.60, 0.63, 0.67, 1.0))
    spray_gray = model.material("spray_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    button_silver = model.material("button_silver", rgba=(0.80, 0.82, 0.85, 1.0))
    control_black = model.material("control_black", rgba=(0.11, 0.12, 0.13, 1.0))

    shell_center_y = (TUB_REAR_Y + TUB_FRONT_Y) / 2.0
    shell_center_z = (BOTTOM_Z + TOP_Z) / 2.0

    cabinet.visual(
        Box((WALL, TUB_DEPTH, TUB_HEIGHT)),
        origin=Origin(xyz=(-(WIDTH / 2.0) + (WALL / 2.0), shell_center_y, shell_center_z)),
        material=tub_gray,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL, TUB_DEPTH, TUB_HEIGHT)),
        origin=Origin(xyz=((WIDTH / 2.0) - (WALL / 2.0), shell_center_y, shell_center_z)),
        material=tub_gray,
        name="right_wall",
    )
    cabinet.visual(
        Box((WIDTH, TUB_DEPTH, BOTTOM_Z)),
        origin=Origin(xyz=(0.0, shell_center_y, BOTTOM_Z / 2.0)),
        material=tub_gray,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((WIDTH, TUB_DEPTH, HEIGHT - TOP_Z)),
        origin=Origin(
            xyz=(0.0, shell_center_y, TOP_Z + (HEIGHT - TOP_Z) / 2.0)
        ),
        material=tub_gray,
        name="top_pan",
    )
    cabinet.visual(
        Box((WIDTH, WALL, TUB_HEIGHT)),
        origin=Origin(xyz=(0.0, TUB_REAR_Y + WALL / 2.0, shell_center_z)),
        material=tub_gray,
        name="rear_wall",
    )

    jamb_height = HEIGHT - TOP_Z + 0.028
    cabinet.visual(
        Box((WIDTH, 0.018, jamb_height)),
        origin=Origin(
            xyz=(0.0, TUB_FRONT_Y - 0.009, TOP_Z - 0.014 + jamb_height / 2.0)
        ),
        material=graphite,
        name="top_jamb",
    )
    cabinet.visual(
        Box((0.026, 0.020, TUB_HEIGHT - 0.050)),
        origin=Origin(
            xyz=(-(WIDTH / 2.0) + 0.013, TUB_FRONT_Y - 0.010, BOTTOM_Z + (TUB_HEIGHT - 0.050) / 2.0)
        ),
        material=graphite,
        name="left_jamb",
    )
    cabinet.visual(
        Box((0.026, 0.020, TUB_HEIGHT - 0.050)),
        origin=Origin(
            xyz=((WIDTH / 2.0) - 0.013, TUB_FRONT_Y - 0.010, BOTTOM_Z + (TUB_HEIGHT - 0.050) / 2.0)
        ),
        material=graphite,
        name="right_jamb",
    )
    cabinet.visual(
        Box((WIDTH - 0.040, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.013, HINGE_Z)),
        material=graphite,
        name="front_sill",
    )

    rail_specs = (
        ("lower_rail", 0.255, 0.030, 0.460, 0.024),
        ("upper_rail", 0.485, 0.026, 0.455, 0.022),
        ("tray_rail", 0.676, 0.022, 0.470, 0.016),
    )
    for base_name, z_center, rail_width, rail_depth, rail_height in rail_specs:
        x_center = (INNER_WIDTH / 2.0) - rail_width / 2.0
        for side, sign in (("left", -1.0), ("right", 1.0)):
            cabinet.visual(
                Box((rail_width, rail_depth, rail_height)),
                origin=Origin(xyz=(sign * x_center, -0.020, z_center)),
                material=rack_gray,
                name=f"{base_name}_{side}",
            )

    cabinet.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, -0.020, 0.113)),
        material=spray_gray,
        name="lower_arm_mount",
    )
    cabinet.visual(
        Box((0.034, 0.040, 0.056)),
        origin=Origin(xyz=(0.0, -0.020, 0.088)),
        material=spray_gray,
        name="lower_arm_stand",
    )

    return cabinet, stainless, graphite, tub_gray, rack_gray, spray_gray, button_silver, control_black


def _add_door(
    model: ArticulatedObject,
    cabinet,
    stainless,
    graphite,
    tub_gray,
    control_black,
):
    door = model.part("door")

    door.visual(
        Box((WIDTH - 0.006, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=stainless,
        name="outer_panel",
    )
    door.visual(
        Box((WIDTH - 0.040, 0.006, DOOR_HEIGHT - 0.046)),
        origin=Origin(xyz=(0.0, -0.013, (DOOR_HEIGHT - 0.046) / 2.0 + 0.024)),
        material=tub_gray,
        name="inner_liner",
    )
    door.visual(
        Box((0.020, 0.028, DOOR_HEIGHT - 0.070)),
        origin=Origin(
            xyz=(-(WIDTH - 0.040) / 2.0, -0.004, (DOOR_HEIGHT - 0.070) / 2.0 + 0.020)
        ),
        material=tub_gray,
        name="left_return",
    )
    door.visual(
        Box((0.020, 0.028, DOOR_HEIGHT - 0.070)),
        origin=Origin(
            xyz=((WIDTH - 0.040) / 2.0, -0.004, (DOOR_HEIGHT - 0.070) / 2.0 + 0.020)
        ),
        material=tub_gray,
        name="right_return",
    )
    door.visual(
        Box((WIDTH - 0.082, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, -0.004, DOOR_HEIGHT - 0.022)),
        material=tub_gray,
        name="top_return",
    )
    door.visual(
        Box((WIDTH - 0.080, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.020, 0.012)),
        material=graphite,
        name="hinge_strip",
    )
    door.visual(
        Box((WIDTH - 0.090, 0.012, 0.078)),
        origin=Origin(xyz=(0.0, -0.013, DOOR_HEIGHT - 0.060)),
        material=control_black,
        name="control_band",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )

    return door


def _add_rack(
    model: ArticulatedObject,
    *,
    name: str,
    width: float,
    depth: float,
    height: float,
    frame_height: float,
    divider_height: float,
    guide_z: float,
    guide_x: float,
    material,
):
    rack = model.part(name)
    half_width = width / 2.0

    rack.visual(
        Box((width - 0.050, depth, 0.010)),
        origin=Origin(xyz=(0.0, -depth / 2.0, 0.010)),
        material=material,
        name="floor",
    )
    rack.visual(
        Box((0.010, depth, height)),
        origin=Origin(xyz=(-(half_width - 0.012), -depth / 2.0, height / 2.0)),
        material=material,
        name="left_side",
    )
    rack.visual(
        Box((0.010, depth, height)),
        origin=Origin(xyz=((half_width - 0.012), -depth / 2.0, height / 2.0)),
        material=material,
        name="right_side",
    )
    rack.visual(
        Box((width - 0.024, 0.010, height)),
        origin=Origin(xyz=(0.0, -0.005, height / 2.0)),
        material=material,
        name="front_frame",
    )
    rack.visual(
        Box((width - 0.024, 0.010, height)),
        origin=Origin(xyz=(0.0, -(depth - 0.025), height / 2.0)),
        material=material,
        name="rear_frame",
    )
    rack.visual(
        Box((width - 0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, height - 0.005)),
        material=material,
        name="front_top_rail",
    )
    rack.visual(
        Box((width - 0.024, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -(depth - 0.005), height - 0.005)),
        material=material,
        name="rear_top_rail",
    )
    rack.visual(
        Box((0.010, depth, 0.010)),
        origin=Origin(xyz=(-(half_width - 0.005), -depth / 2.0, height - 0.005)),
        material=material,
        name="left_top_rail",
    )
    rack.visual(
        Box((0.010, depth, 0.010)),
        origin=Origin(xyz=((half_width - 0.005), -depth / 2.0, height - 0.005)),
        material=material,
        name="right_top_rail",
    )
    rack.visual(
        Box((width * 0.32, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, height + 0.005)),
        material=material,
        name="handle",
    )
    rack.visual(
        Box((0.014, 0.016, 0.060)),
        origin=Origin(xyz=(-width * 0.12, 0.006, height - 0.020)),
        material=material,
        name="handle_post_left",
    )
    rack.visual(
        Box((0.014, 0.016, 0.060)),
        origin=Origin(xyz=(width * 0.12, 0.006, height - 0.020)),
        material=material,
        name="handle_post_right",
    )
    rack.visual(
        Box((0.008, depth * 0.72, divider_height)),
        origin=Origin(xyz=(-width * 0.22, -(depth * 0.60) / 2.0 - 0.070, divider_height / 2.0)),
        material=material,
        name="divider_left",
    )
    rack.visual(
        Box((0.008, depth * 0.72, divider_height)),
        origin=Origin(xyz=(width * 0.22, -(depth * 0.60) / 2.0 - 0.070, divider_height / 2.0)),
        material=material,
        name="divider_right",
    )
    rack.visual(
        Box((0.026, 0.200, 0.024)),
        origin=Origin(xyz=(-guide_x, -0.200, guide_z)),
        material=material,
        name="guide_tab_left",
    )
    rack.visual(
        Box((0.026, 0.200, 0.024)),
        origin=Origin(xyz=(guide_x, -0.200, guide_z)),
        material=material,
        name="guide_tab_right",
    )
    return rack


def _add_third_tray(model: ArticulatedObject, material):
    tray = model.part("third_tray")
    width = 0.448
    depth = 0.500
    height = 0.038
    half_width = width / 2.0

    tray.visual(
        Box((width - 0.040, depth, 0.008)),
        origin=Origin(xyz=(0.0, -depth / 2.0, 0.006)),
        material=material,
        name="floor",
    )
    tray.visual(
        Box((0.008, depth, height)),
        origin=Origin(xyz=(-(half_width - 0.004), -depth / 2.0, height / 2.0)),
        material=material,
        name="left_side",
    )
    tray.visual(
        Box((0.008, depth, height)),
        origin=Origin(xyz=((half_width - 0.004), -depth / 2.0, height / 2.0)),
        material=material,
        name="right_side",
    )
    tray.visual(
        Box((width, 0.008, height)),
        origin=Origin(xyz=(0.0, -0.004, height / 2.0)),
        material=material,
        name="front_frame",
    )
    tray.visual(
        Box((width, 0.008, height)),
        origin=Origin(xyz=(0.0, -(depth - 0.004), height / 2.0)),
        material=material,
        name="rear_frame",
    )
    for index, x_pos in enumerate((-0.110, 0.0, 0.110)):
        tray.visual(
            Box((0.006, depth - 0.060, height - 0.010)),
            origin=Origin(xyz=(x_pos, -(depth - 0.060) / 2.0 - 0.030, (height - 0.010) / 2.0)),
            material=material,
            name=f"divider_{index}",
        )
    tray.visual(
        Box((0.024, 0.220, 0.018)),
        origin=Origin(xyz=(-(half_width + 0.017), -0.205, 0.012)),
        material=material,
        name="guide_tab_left",
    )
    tray.visual(
        Box((0.024, 0.220, 0.018)),
        origin=Origin(xyz=((half_width + 0.017), -0.205, 0.012)),
        material=material,
        name="guide_tab_right",
    )
    tray.visual(
        Box((0.026, 0.060, 0.018)),
        origin=Origin(xyz=(-(half_width + 0.004), -0.205, 0.012)),
        material=material,
        name="guide_bridge_left",
    )
    tray.visual(
        Box((0.026, 0.060, 0.018)),
        origin=Origin(xyz=((half_width + 0.004), -0.205, 0.012)),
        material=material,
        name="guide_bridge_right",
    )
    tray.visual(
        Box((width * 0.40, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.008, 0.016)),
        material=material,
        name="handle",
    )
    return tray


def _add_spray_arm(model: ArticulatedObject, *, name: str, span: float, material):
    arm = model.part(name)
    arm.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=material,
        name="hub",
    )
    arm.visual(
        Box((span, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=material,
        name="main_arm",
    )
    arm.visual(
        Box((0.120, 0.020, 0.008)),
        origin=Origin(xyz=(0.040, 0.022, 0.013)),
        material=material,
        name="branch_arm",
    )
    arm.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(span / 2.0 - 0.022, 0.0, 0.018)),
        material=material,
        name="tip_left",
    )
    arm.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(-(span / 2.0) + 0.022, 0.0, 0.018)),
        material=material,
        name="tip_right",
    )
    return arm


def _add_controls(model: ArticulatedObject, door, button_silver, control_black):
    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="base",
    )
    dial.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_silver,
        name="dial_cap",
    )
    dial.visual(
        Box((0.003, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.017, 0.013)),
        material=control_black,
        name="indicator",
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.215, -0.019, DOOR_HEIGHT - 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=5.0,
        ),
    )

    button_positions = (-0.120, -0.055, 0.010, 0.075, 0.140)
    for index, x_pos in enumerate(button_positions):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.052, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_silver,
            name="button",
        )
        button.visual(
            Box((0.042, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, -0.009, 0.005)),
            material=control_black,
            name="label_strip",
        )
        model.articulation(
            f"door_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x_pos, -0.019, DOOR_HEIGHT - 0.064)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.050,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_dishwasher")

    (
        cabinet,
        stainless,
        graphite,
        tub_gray,
        rack_gray,
        spray_gray,
        button_silver,
        control_black,
    ) = _add_cabinet_shell(model)

    door = _add_door(model, cabinet, stainless, graphite, tub_gray, control_black)
    _add_controls(model, door, button_silver, control_black)

    lower_rack = _add_rack(
        model,
        name="lower_rack",
        width=0.500,
        depth=0.500,
        height=0.160,
        frame_height=0.110,
        divider_height=0.115,
        guide_z=0.072,
        guide_x=0.232,
        material=rack_gray,
    )
    upper_rack = _add_rack(
        model,
        name="upper_rack",
        width=0.490,
        depth=0.490,
        height=0.108,
        frame_height=0.082,
        divider_height=0.082,
        guide_z=0.052,
        guide_x=0.236,
        material=rack_gray,
    )
    third_tray = _add_third_tray(model, rack_gray)

    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.242, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=LOWER_RACK_TRAVEL,
        ),
    )
    model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.235, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=UPPER_RACK_TRAVEL,
        ),
    )
    model.articulation(
        "cabinet_to_third_tray",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=third_tray,
        origin=Origin(xyz=(0.0, 0.242, 0.664)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.30,
            lower=0.0,
            upper=THIRD_TRAY_TRAVEL,
        ),
    )

    lower_spray_arm = _add_spray_arm(
        model,
        name="lower_spray_arm",
        span=0.360,
        material=spray_gray,
    )
    upper_spray_arm = _add_spray_arm(
        model,
        name="upper_spray_arm",
        span=0.285,
        material=spray_gray,
    )

    model.articulation(
        "cabinet_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, -0.020, 0.138)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=12.0,
        ),
    )

    upper_rack.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.0, -0.245, -0.014)),
        material=spray_gray,
        name="spray_mount",
    )
    upper_rack.visual(
        Box((0.018, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.245, 0.006)),
        material=spray_gray,
        name="spray_stem",
    )
    upper_rack.visual(
        Box((0.024, 0.140, 0.022)),
        origin=Origin(xyz=(0.250, -0.205, 0.080)),
        material=rack_gray,
        name="glass_rack_mount",
    )

    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, -0.245, -0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=12.0,
        ),
    )

    glass_rack = model.part("right_glass_rack")
    glass_rack.visual(
        Box((0.008, 0.140, 0.160)),
        origin=Origin(xyz=(0.004, 0.0, 0.080)),
        material=rack_gray,
        name="fold_panel",
    )
    glass_rack.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rack_gray,
        name="pivot_lower",
    )
    glass_rack.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rack_gray,
        name="pivot_upper",
    )
    model.articulation(
        "upper_rack_to_right_glass_rack",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=glass_rack,
        origin=Origin(xyz=(0.268, -0.205, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=0.0,
            upper=GLASS_RACK_DROP,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    third_tray = object_model.get_part("third_tray")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")
    glass_rack = object_model.get_part("right_glass_rack")
    button = object_model.get_part("preset_button_0")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    lower_slide = object_model.get_articulation("cabinet_to_lower_rack")
    upper_slide = object_model.get_articulation("cabinet_to_upper_rack")
    tray_slide = object_model.get_articulation("cabinet_to_third_tray")
    button_slide = object_model.get_articulation("door_to_preset_button_0")
    glass_hinge = object_model.get_articulation("upper_rack_to_right_glass_rack")

    ctx.expect_contact(
        door,
        cabinet,
        elem_a="outer_panel",
        elem_b="front_sill",
        name="door seats on the lower hinge sill",
    )
    ctx.expect_contact(
        lower_rack,
        cabinet,
        elem_a="guide_tab_left",
        elem_b="lower_rail_left",
        name="lower rack rides on the lower rail",
    )
    ctx.expect_contact(
        upper_rack,
        cabinet,
        elem_a="guide_tab_left",
        elem_b="upper_rail_left",
        name="upper rack rides on the upper rail",
    )
    ctx.expect_contact(
        third_tray,
        cabinet,
        elem_a="guide_tab_left",
        elem_b="tray_rail_left",
        name="third tray rides on the top rail",
    )
    ctx.expect_contact(
        lower_spray_arm,
        cabinet,
        elem_a="hub",
        elem_b="lower_arm_mount",
        name="lower spray arm sits on its center mount",
    )
    ctx.expect_contact(
        upper_spray_arm,
        upper_rack,
        elem_a="hub",
        elem_b="spray_mount",
        name="upper spray arm hangs from the rack mount",
    )
    ctx.expect_contact(
        glass_rack,
        upper_rack,
        elem_a="pivot_lower",
        elem_b="glass_rack_mount",
        name="glass rack is supported by the upper rack side mount",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.45}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings downward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.45
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.60,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_slide: LOWER_RACK_TRAVEL}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(
            lower_rack,
            cabinet,
            axes="y",
            min_overlap=0.030,
            name="lower rack keeps retained insertion at full extension",
        )
    ctx.check(
        "lower rack slides outward",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[1] > lower_rest[1] + LOWER_RACK_TRAVEL * 0.90,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({upper_slide: UPPER_RACK_TRAVEL}):
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(
            upper_rack,
            cabinet,
            axes="y",
            min_overlap=0.035,
            name="upper rack keeps retained insertion at full extension",
        )
    ctx.check(
        "upper rack slides outward",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] > upper_rest[1] + UPPER_RACK_TRAVEL * 0.90,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    tray_rest = ctx.part_world_position(third_tray)
    with ctx.pose({tray_slide: THIRD_TRAY_TRAVEL}):
        tray_extended = ctx.part_world_position(third_tray)
        ctx.expect_overlap(
            third_tray,
            cabinet,
            axes="y",
            min_overlap=0.025,
            name="third tray keeps retained insertion at full extension",
        )
    ctx.check(
        "third tray slides outward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] > tray_rest[1] + THIRD_TRAY_TRAVEL * 0.90,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_slide: BUTTON_TRAVEL}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "preset button presses into the control band",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] < button_rest[1] - BUTTON_TRAVEL * 0.75,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    glass_stowed = ctx.part_world_aabb(glass_rack)
    with ctx.pose({glass_hinge: GLASS_RACK_DROP}):
        glass_open = ctx.part_world_aabb(glass_rack)
    ctx.check(
        "glass rack folds down into a shelf",
        glass_stowed is not None
        and glass_open is not None
        and (glass_open[1][0] - glass_open[0][0]) > (glass_stowed[1][0] - glass_stowed[0][0]) + 0.120,
        details=f"stowed={glass_stowed}, open={glass_open}",
    )

    return ctx.report()


object_model = build_object_model()
