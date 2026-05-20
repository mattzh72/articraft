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


CABINET_WIDTH = 0.60
CABINET_DEPTH = 0.62
CABINET_HEIGHT = 0.85
SIDE_THICKNESS = 0.012
BACK_THICKNESS = 0.012
BOTTOM_THICKNESS = 0.032
TOP_THICKNESS = 0.016
CONTROL_HEIGHT = 0.065
TOE_KICK_HEIGHT = 0.095
DOOR_HEIGHT = CABINET_HEIGHT - CONTROL_HEIGHT - TOE_KICK_HEIGHT


def _add_lower_rack_visuals(part, material) -> None:
    depth = 0.510
    width = 0.500
    top_z = 0.094
    floor_z = 0.022
    side_y = width / 2.0 - 0.006

    part.visual(
        Box((0.018, width, 0.012)),
        origin=Origin(xyz=(0.009, 0.0, top_z)),
        material=material,
        name="front_rail",
    )
    part.visual(
        Box((0.018, width, 0.012)),
        origin=Origin(xyz=(depth - 0.009, 0.0, top_z)),
        material=material,
        name="rear_rail",
    )

    for index, side in enumerate((-1.0, 1.0)):
        part.visual(
            Box((depth, 0.012, 0.012)),
            origin=Origin(xyz=(depth / 2.0, side * side_y, top_z)),
            material=material,
            name=f"side_rail_{index}",
        )
        part.visual(
            Box((depth, 0.010, 0.010)),
            origin=Origin(xyz=(depth / 2.0, side * (side_y - 0.006), floor_z)),
            material=material,
            name=f"bottom_rail_{index}",
        )
        part.visual(
            Box((0.420, 0.024, 0.012)),
            origin=Origin(xyz=(0.235, side * 0.253, 0.026)),
            material=material,
            name=f"runner_bracket_{index}",
        )
        part.visual(
            Box((0.420, 0.006, 0.012)),
            origin=Origin(xyz=(0.235, side * 0.268, 0.026)),
            material=material,
            name=f"runner_{index}",
        )

    for index, rail_x in enumerate((0.090, 0.190, 0.290, 0.390, 0.470)):
        part.visual(
            Box((0.014, width - 0.026, 0.008)),
            origin=Origin(xyz=(rail_x, 0.0, floor_z)),
            material=material,
            name=f"cross_slat_{index}",
        )

    for index, rail_y in enumerate((-0.160, -0.080, 0.0, 0.080, 0.160)):
        part.visual(
            Box((depth - 0.032, 0.010, 0.008)),
            origin=Origin(xyz=(depth / 2.0, rail_y, floor_z)),
            material=material,
            name=f"floor_slat_{index}",
        )

    corner_xs = (0.009, depth - 0.009)
    corner_ys = (-side_y, side_y)
    post_index = 0
    for corner_x in corner_xs:
        for corner_y in corner_ys:
            part.visual(
                Box((0.014, 0.014, 0.072)),
                origin=Origin(xyz=(corner_x, corner_y, 0.058)),
                material=material,
                name=f"post_{post_index}",
            )
            post_index += 1

    part.visual(
        Box((0.012, 0.240, 0.018)),
        origin=Origin(xyz=(-0.004, 0.0, 0.104)),
        material=material,
        name="handle",
    )


def _add_upper_rack_visuals(part, material) -> None:
    depth = 0.480
    width = 0.530
    top_z = 0.062
    floor_z = 0.018
    side_y = width / 2.0 - 0.006

    part.visual(
        Box((0.016, width, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, top_z)),
        material=material,
        name="front_rail",
    )
    part.visual(
        Box((0.016, width, 0.010)),
        origin=Origin(xyz=(depth - 0.008, 0.0, top_z)),
        material=material,
        name="rear_rail",
    )

    for index, side in enumerate((-1.0, 1.0)):
        part.visual(
            Box((depth, 0.012, 0.010)),
            origin=Origin(xyz=(depth / 2.0, side * side_y, top_z)),
            material=material,
            name=f"side_rail_{index}",
        )
        part.visual(
            Box((depth - 0.020, 0.010, 0.008)),
            origin=Origin(xyz=(depth / 2.0, side * (side_y - 0.008), floor_z)),
            material=material,
            name=f"lower_rail_{index}",
        )
        part.visual(
            Box((0.400, 0.018, 0.016)),
            origin=Origin(xyz=(0.220, side * 0.257, 0.015)),
            material=material,
            name=f"runner_bracket_{index}",
        )
        part.visual(
            Box((0.400, 0.006, 0.016)),
            origin=Origin(xyz=(0.220, side * 0.269, 0.015)),
            material=material,
            name=f"runner_{index}",
        )

    for index, rail_x in enumerate((0.080, 0.170, 0.260, 0.350, 0.430)):
        part.visual(
            Box((0.012, width - 0.034, 0.008)),
            origin=Origin(xyz=(rail_x, 0.0, floor_z)),
            material=material,
            name=f"cross_slat_{index}",
        )

    for index, rail_y in enumerate((-0.180, -0.090, 0.0, 0.090, 0.180)):
        part.visual(
            Box((depth - 0.032, 0.008, 0.007)),
            origin=Origin(xyz=(depth / 2.0, rail_y, floor_z)),
            material=material,
            name=f"floor_slat_{index}",
        )

    corner_xs = (0.008, depth - 0.008)
    corner_ys = (-side_y, side_y)
    post_index = 0
    for corner_x in corner_xs:
        for corner_y in corner_ys:
            part.visual(
                Box((0.012, 0.012, 0.050)),
                origin=Origin(xyz=(corner_x, corner_y, 0.040)),
                material=material,
                name=f"post_{post_index}",
            )
            post_index += 1

    part.visual(
        Box((0.010, 0.260, 0.014)),
        origin=Origin(xyz=(-0.004, 0.0, 0.070)),
        material=material,
        name="handle",
    )

def _add_spray_arm_visuals(part, material, *, arm_y: float, arm_x: float) -> None:
    part.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=material,
        name="hub",
    )
    part.visual(
        Box((0.040, arm_y, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=material,
        name="cross_arm",
    )
    part.visual(
        Box((arm_x, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=material,
        name="feed_arm",
    )
    for index, x_pos in enumerate((-arm_x / 2.4, arm_x / 2.4)):
        part.visual(
            Box((0.028, 0.016, 0.012)),
            origin=Origin(xyz=(x_pos, 0.0, 0.006)),
            material=material,
            name=f"nozzle_{index}",
        )


def _add_tine_bank_visuals(part, material) -> None:
    part.visual(
        Box((0.020, 0.360, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=material,
        name="pivot_rod",
    )
    for index, y_pos in enumerate((-0.150, -0.090, -0.030, 0.030, 0.090, 0.150)):
        part.visual(
            Box((0.010, 0.010, 0.120)),
            origin=Origin(xyz=(0.0, y_pos, 0.065)),
            material=material,
            name=f"tine_{index}",
        )


def _add_button_visuals(part, material) -> None:
    part.visual(
        Box((0.010, 0.028, 0.008)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=material,
        name="cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_dishwasher")

    shell = model.material("shell", rgba=(0.92, 0.93, 0.94, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    rack_metal = model.material("rack_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    spray_grey = model.material("spray_grey", rgba=(0.60, 0.63, 0.66, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        Box((CABINET_DEPTH, SIDE_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_DEPTH / 2.0, -(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0), CABINET_HEIGHT / 2.0)),
        material=shell,
        name="left_side",
    )
    body.visual(
        Box((CABINET_DEPTH, SIDE_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_DEPTH / 2.0, CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, CABINET_HEIGHT / 2.0)),
        material=shell,
        name="right_side",
    )
    body.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(CABINET_DEPTH / 2.0, 0.0, BOTTOM_THICKNESS / 2.0)),
        material=stainless,
        name="bottom_pan",
    )
    body.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, TOP_THICKNESS)),
        origin=Origin(xyz=(CABINET_DEPTH / 2.0, 0.0, CABINET_HEIGHT - TOP_THICKNESS / 2.0)),
        material=stainless,
        name="top_pan",
    )
    body.visual(
        Box((BACK_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_HEIGHT - TOP_THICKNESS - 0.020)),
        origin=Origin(xyz=(CABINET_DEPTH - BACK_THICKNESS / 2.0, 0.0, (CABINET_HEIGHT - TOP_THICKNESS - 0.020) / 2.0 + 0.010)),
        material=stainless,
        name="back_pan",
    )
    body.visual(
        Box((0.030, 0.022, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.015, -(CABINET_WIDTH / 2.0 - 0.011), TOE_KICK_HEIGHT + DOOR_HEIGHT / 2.0)),
        material=shell,
        name="left_jamb",
    )
    body.visual(
        Box((0.030, 0.022, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.015, CABINET_WIDTH / 2.0 - 0.011, TOE_KICK_HEIGHT + DOOR_HEIGHT / 2.0)),
        material=shell,
        name="right_jamb",
    )
    body.visual(
        Box((0.030, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, TOE_KICK_HEIGHT + DOOR_HEIGHT - 0.015)),
        material=shell,
        name="top_header",
    )
    body.visual(
        Box((0.030, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, TOE_KICK_HEIGHT)),
        origin=Origin(xyz=(0.015, 0.0, TOE_KICK_HEIGHT / 2.0)),
        material=dark_trim,
        name="toe_kick",
    )
    body.visual(
        Box((0.030, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CONTROL_HEIGHT)),
        origin=Origin(xyz=(0.015, 0.0, CABINET_HEIGHT - CONTROL_HEIGHT / 2.0)),
        material=dark_trim,
        name="control_band",
    )
    for index, side in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.420, 0.018, 0.018)),
            origin=Origin(xyz=(0.255, side * 0.280, 0.185)),
            material=stainless,
            name=f"lower_track_{index}",
        )
        body.visual(
            Box((0.400, 0.016, 0.016)),
            origin=Origin(xyz=(0.245, side * 0.280, 0.565)),
            material=stainless,
            name=f"upper_track_{index}",
        )
    body.visual(
        Cylinder(radius=0.022, length=0.062),
        origin=Origin(xyz=(0.310, 0.0, 0.063)),
        material=stainless,
        name="lower_spray_support",
    )

    door = model.part("door")
    door.visual(
        Box((0.034, 0.592, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.017, 0.0, DOOR_HEIGHT / 2.0)),
        material=shell,
        name="outer_panel",
    )
    door.visual(
        Box((0.020, 0.548, DOOR_HEIGHT - 0.060)),
        origin=Origin(xyz=(-0.012, 0.0, (DOOR_HEIGHT - 0.060) / 2.0 + 0.020)),
        material=stainless,
        name="inner_liner",
    )
    door.visual(
        Box((0.020, 0.592, 0.032)),
        origin=Origin(xyz=(-0.012, 0.0, DOOR_HEIGHT - 0.016)),
        material=dark_trim,
        name="top_trim",
    )

    lower_rack = model.part("lower_rack")
    _add_lower_rack_visuals(lower_rack, rack_metal)

    upper_rack = model.part("upper_rack")
    _add_upper_rack_visuals(upper_rack, rack_metal)

    lower_spray_arm = model.part("lower_spray_arm")
    _add_spray_arm_visuals(lower_spray_arm, spray_grey, arm_y=0.420, arm_x=0.180)

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=spray_grey,
        name="hub",
    )
    upper_spray_arm.visual(
        Box((0.040, 0.360, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=spray_grey,
        name="cross_arm",
    )
    upper_spray_arm.visual(
        Box((0.150, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=spray_grey,
        name="feed_arm",
    )
    for index, x_pos in enumerate((-0.062, 0.062)):
        upper_spray_arm.visual(
            Box((0.028, 0.016, 0.012)),
            origin=Origin(xyz=(x_pos, 0.0, -0.006)),
            material=spray_grey,
            name=f"nozzle_{index}",
        )

    upper_spray_support = model.part("upper_spray_support")
    upper_spray_support.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rack_metal,
        name="bushing",
    )
    upper_spray_support.visual(
        Box((0.024, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=rack_metal,
        name="neck",
    )
    upper_spray_support.visual(
        Box((0.064, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=rack_metal,
        name="bridge",
    )

    front_tine_bank = model.part("front_tine_bank")
    _add_tine_bank_visuals(front_tine_bank, rack_metal)

    rear_tine_bank = model.part("rear_tine_bank")
    _add_tine_bank_visuals(rear_tine_bank, rack_metal)

    cutlery_caddy = model.part("cutlery_caddy")
    cutlery_caddy.visual(
        Box((0.018, 0.086, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rack_metal,
        name="guide_bar",
    )
    cutlery_caddy.visual(
        Box((0.070, 0.078, 0.006)),
        origin=Origin(xyz=(0.053, 0.0, -0.082)),
        material=rack_metal,
        name="basket_bottom",
    )
    cutlery_caddy.visual(
        Box((0.070, 0.006, 0.060)),
        origin=Origin(xyz=(0.053, -0.036, -0.049)),
        material=rack_metal,
        name="left_wall",
    )
    cutlery_caddy.visual(
        Box((0.070, 0.006, 0.060)),
        origin=Origin(xyz=(0.053, 0.036, -0.049)),
        material=rack_metal,
        name="right_wall",
    )
    cutlery_caddy.visual(
        Box((0.006, 0.078, 0.060)),
        origin=Origin(xyz=(0.021, 0.0, -0.049)),
        material=rack_metal,
        name="front_wall",
    )
    cutlery_caddy.visual(
        Box((0.006, 0.078, 0.060)),
        origin=Origin(xyz=(0.085, 0.0, -0.049)),
        material=rack_metal,
        name="back_wall",
    )
    cutlery_caddy.visual(
        Box((0.012, 0.008, 0.056)),
        origin=Origin(xyz=(0.012, -0.030, -0.028)),
        material=rack_metal,
        name="hanger_0",
    )
    cutlery_caddy.visual(
        Box((0.012, 0.008, 0.056)),
        origin=Origin(xyz=(0.012, 0.030, -0.028)),
        material=rack_metal,
        name="hanger_1",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_skirt",
    )
    selector_knob.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.002, 0.003, 0.016)),
        origin=Origin(xyz=(-0.026, 0.0, 0.010)),
        material=shell,
        name="pointer",
    )

    button_positions = (
        ("button_0", -0.185, CABINET_HEIGHT - 0.032),
        ("button_1", -0.105, CABINET_HEIGHT - 0.032),
        ("button_2", 0.000, CABINET_HEIGHT - 0.065),
        ("button_3", 0.105, CABINET_HEIGHT - 0.032),
        ("button_4", 0.185, CABINET_HEIGHT - 0.032),
    )
    for part_name, _, _ in button_positions:
        button_part = model.part(part_name)
        _add_button_visuals(button_part, button_black)

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, TOE_KICK_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.055, 0.0, 0.160)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.280,
        ),
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.075, 0.0, 0.550)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=0.240,
        ),
    )
    model.articulation(
        "body_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.310, 0.0, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "upper_rack_to_upper_spray_support",
        ArticulationType.FIXED,
        parent=upper_rack,
        child=upper_spray_support,
        origin=Origin(xyz=(0.240, 0.0, -0.008)),
    )
    model.articulation(
        "upper_spray_support_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_spray_support,
        child=upper_spray_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=18.0,
        ),
    )
    model.articulation(
        "lower_rack_to_cutlery_caddy",
        ArticulationType.PRISMATIC,
        parent=lower_rack,
        child=cutlery_caddy,
        origin=Origin(xyz=(0.009, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.20,
            lower=-0.150,
            upper=0.150,
        ),
    )
    model.articulation(
        "lower_rack_to_front_tine_bank",
        ArticulationType.REVOLUTE,
        parent=lower_rack,
        child=front_tine_bank,
        origin=Origin(xyz=(0.155, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "lower_rack_to_rear_tine_bank",
        ArticulationType.REVOLUTE,
        parent=lower_rack,
        child=rear_tine_bank,
        origin=Origin(xyz=(0.325, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - 0.032)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=8.0,
        ),
    )

    for part_name, y_pos, z_pos in button_positions:
        button_part = model.get_part(part_name)
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_part,
            origin=Origin(xyz=(0.0, y_pos, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cutlery_caddy = object_model.get_part("cutlery_caddy")
    front_tine_bank = object_model.get_part("front_tine_bank")

    door_hinge = object_model.get_articulation("body_to_door")
    lower_rack_slide = object_model.get_articulation("body_to_lower_rack")
    upper_rack_slide = object_model.get_articulation("body_to_upper_rack")
    caddy_slide = object_model.get_articulation("lower_rack_to_cutlery_caddy")
    front_tine_pivot = object_model.get_articulation("lower_rack_to_front_tine_bank")
    rear_tine_pivot = object_model.get_articulation("lower_rack_to_rear_tine_bank")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    lower_spray_joint = object_model.get_articulation("body_to_lower_spray_arm")
    upper_spray_joint = object_model.get_articulation("upper_spray_support_to_upper_spray_arm")

    ctx.expect_gap(
        body,
        door,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        name="door sits flush with the front frame",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="yz",
        min_overlap=0.560,
        name="door covers the dishwasher opening",
    )
    ctx.expect_gap(
        upper_rack,
        lower_rack,
        axis="z",
        min_gap=0.245,
        name="open chamber space remains between the racks",
    )
    ctx.expect_contact(
        cutlery_caddy,
        lower_rack,
        elem_a="guide_bar",
        elem_b="front_rail",
        name="cutlery caddy rides on the lower rack front rail",
    )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door drops outward on its lower hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][0] < -0.50
            and open_aabb[1][2] < TOE_KICK_HEIGHT + 0.08,
            details=f"closed={closed_aabb!r}, open={open_aabb!r}",
        )

    lower_limits = lower_rack_slide.motion_limits
    if lower_limits is not None and lower_limits.upper is not None:
        lower_rest = ctx.part_world_position(lower_rack)
        with ctx.pose({lower_rack_slide: lower_limits.upper}):
            lower_extended = ctx.part_world_position(lower_rack)
            ctx.expect_within(
                lower_rack,
                body,
                axes="yz",
                margin=0.020,
                name="lower rack stays centered between the tub walls",
            )
            ctx.expect_overlap(
                lower_rack,
                body,
                axes="x",
                min_overlap=0.220,
                name="lower rack remains retained at full extension",
            )
        ctx.check(
            "lower rack slides outward",
            lower_rest is not None
            and lower_extended is not None
            and lower_extended[0] < lower_rest[0] - 0.20,
            details=f"rest={lower_rest!r}, extended={lower_extended!r}",
        )

    upper_limits = upper_rack_slide.motion_limits
    if upper_limits is not None and upper_limits.upper is not None:
        upper_rest = ctx.part_world_position(upper_rack)
        with ctx.pose({upper_rack_slide: upper_limits.upper}):
            upper_extended = ctx.part_world_position(upper_rack)
            ctx.expect_within(
                upper_rack,
                body,
                axes="yz",
                margin=0.020,
                name="upper rack stays centered between the tub walls",
            )
            ctx.expect_overlap(
                upper_rack,
                body,
                axes="x",
                min_overlap=0.200,
                name="upper rack remains retained at full extension",
            )
        ctx.check(
            "upper rack slides outward",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[0] < upper_rest[0] - 0.18,
            details=f"rest={upper_rest!r}, extended={upper_extended!r}",
        )

    caddy_limits = caddy_slide.motion_limits
    lower_partial = 0.160
    if caddy_limits is not None and caddy_limits.lower is not None and caddy_limits.upper is not None:
        with ctx.pose({lower_rack_slide: lower_partial, caddy_slide: caddy_limits.lower}):
            caddy_left = ctx.part_world_position(cutlery_caddy)
        with ctx.pose({lower_rack_slide: lower_partial, caddy_slide: caddy_limits.upper}):
            caddy_right = ctx.part_world_position(cutlery_caddy)
        ctx.check(
            "cutlery caddy slides across the lower rack front rail",
            caddy_left is not None
            and caddy_right is not None
            and abs(caddy_right[1] - caddy_left[1]) > 0.25
            and abs(caddy_right[0] - caddy_left[0]) < 0.01,
            details=f"left={caddy_left!r}, right={caddy_right!r}",
        )

    tine_limits = front_tine_pivot.motion_limits
    if tine_limits is not None and tine_limits.upper is not None:
        upright_aabb = ctx.part_world_aabb(front_tine_bank)
        with ctx.pose({front_tine_pivot: tine_limits.upper, rear_tine_pivot: tine_limits.upper}):
            folded_aabb = ctx.part_world_aabb(front_tine_bank)
        ctx.check(
            "lower rack tine banks fold down",
            upright_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][2] < upright_aabb[1][2] - 0.05,
            details=f"upright={upright_aabb!r}, folded={folded_aabb!r}",
        )

    ctx.check(
        "selector knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )
    ctx.check(
        "spray arms are continuous",
        lower_spray_joint.articulation_type == ArticulationType.CONTINUOUS
        and upper_spray_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"lower={lower_spray_joint.articulation_type!r}, upper={upper_spray_joint.articulation_type!r}",
    )

    for index in range(5):
        button_part = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits
        if button_limits is None or button_limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] > rest_pos[0] + 0.003,
            details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
