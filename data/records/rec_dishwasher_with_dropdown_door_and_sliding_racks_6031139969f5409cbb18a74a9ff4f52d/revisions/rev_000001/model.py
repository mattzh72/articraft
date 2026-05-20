from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

WIDTH = 0.60
DEPTH = 0.63
HEIGHT = 0.86
FASCIA_BOTTOM_Z = 0.74
FASCIA_HEIGHT = HEIGHT - FASCIA_BOTTOM_Z
DOOR_HEIGHT = FASCIA_BOTTOM_Z - 0.04
DOOR_THICKNESS = 0.032


def _rod_origin(
    *,
    center: tuple[float, float, float],
    axis: str,
) -> Origin:
    if axis == "x":
        return Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0))
    if axis == "y":
        return Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0))
    return Origin(xyz=center)


def _add_rod(
    part,
    *,
    name: str,
    center: tuple[float, float, float],
    axis: str,
    length: float,
    radius: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_rod_origin(center=center, axis=axis),
        material=material,
        name=name,
    )


def _add_rack_wirework(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    basket_center_y: float,
    lower_z: float,
    runner_offset_x: float,
    runner_length: float,
    rod_radius: float,
    material,
) -> None:
    x_side = width / 2.0 - rod_radius
    y_front = basket_center_y + depth / 2.0 - rod_radius
    y_back = basket_center_y - depth / 2.0 + rod_radius
    z_top = lower_z + height

    for name, y_value, z_value in (
        ("front_lower", y_front, lower_z),
        ("back_lower", y_back, lower_z),
        ("front_upper", y_front, z_top),
        ("back_upper", y_back, z_top),
    ):
        _add_rod(
            part,
            name=name,
            center=(0.0, y_value, z_value),
            axis="x",
            length=width,
            radius=rod_radius,
            material=material,
        )

    for name, x_value, z_value in (
        ("left_lower", -x_side, lower_z),
        ("right_lower", x_side, lower_z),
        ("left_upper", -x_side, z_top),
        ("right_upper", x_side, z_top),
    ):
        _add_rod(
            part,
            name=name,
            center=(x_value, basket_center_y, z_value),
            axis="y",
            length=depth,
            radius=rod_radius,
            material=material,
        )

    post_positions = (
        (-x_side, y_back),
        (-x_side, y_front),
        (x_side, y_back),
        (x_side, y_front),
        (-x_side, basket_center_y),
        (x_side, basket_center_y),
    )
    for index, (x_value, y_value) in enumerate(post_positions):
        _add_rod(
            part,
            name=f"post_{index}",
            center=(x_value, y_value, (lower_z + z_top) / 2.0),
            axis="z",
            length=height,
            radius=rod_radius,
            material=material,
        )

    cross_rod_y = [
        y_back + 0.05,
        y_back + 0.13,
        basket_center_y,
        y_front - 0.13,
        y_front - 0.05,
    ]
    for index, y_value in enumerate(cross_rod_y):
        _add_rod(
            part,
            name=f"floor_cross_{index}",
            center=(0.0, y_value, lower_z),
            axis="x",
            length=width,
            radius=rod_radius * 0.9,
            material=material,
        )

    for index, x_value in enumerate((-width * 0.22, 0.0, width * 0.22)):
        _add_rod(
            part,
            name=f"floor_longitudinal_{index}",
            center=(x_value, basket_center_y, lower_z),
            axis="y",
            length=depth,
            radius=rod_radius * 0.9,
            material=material,
        )

    part.visual(
        Box((0.012, runner_length, 0.010)),
        origin=Origin(xyz=(-runner_offset_x, -0.195, -0.005)),
        material=material,
        name="left_runner",
    )
    part.visual(
        Box((0.012, runner_length, 0.010)),
        origin=Origin(xyz=(runner_offset_x, -0.195, -0.005)),
        material=material,
        name="right_runner",
    )

    bridge_y_values = (y_back + 0.025, basket_center_y, y_front - 0.045)
    for index, y_value in enumerate(bridge_y_values):
        part.visual(
            Box((0.022, 0.018, 0.010)),
            origin=Origin(xyz=(-(runner_offset_x - x_side) / 2.0 - x_side, y_value, 0.0)),
            material=material,
            name=f"left_runner_bridge_{index}",
        )
        part.visual(
            Box((0.022, 0.018, 0.010)),
            origin=Origin(xyz=((runner_offset_x - x_side) / 2.0 + x_side, y_value, 0.0)),
            material=material,
            name=f"right_runner_bridge_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pro_dishwasher")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    tub_finish = model.material("tub_finish", rgba=(0.70, 0.72, 0.74, 1.0))
    wire_finish = model.material("wire_finish", rgba=(0.82, 0.84, 0.87, 1.0))
    button_black = model.material("button_black", rgba=(0.15, 0.15, 0.16, 1.0))
    accent = model.material("accent", rgba=(0.28, 0.30, 0.32, 1.0))

    body = model.part("body")
    side_thickness = 0.015
    top_thickness = 0.015
    fascia_thickness = 0.025
    tub_width = 0.54
    tub_depth = 0.50
    tub_floor_z = 0.085
    tub_ceiling_z = 0.715
    tub_height = tub_ceiling_z - tub_floor_z

    body.visual(
        Box((side_thickness, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + side_thickness / 2.0, 0.0, HEIGHT / 2.0)),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((side_thickness, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - side_thickness / 2.0, 0.0, HEIGHT / 2.0)),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * side_thickness, DEPTH, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - top_thickness / 2.0)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * side_thickness, side_thickness, HEIGHT - 0.02)),
        origin=Origin(
            xyz=(0.0, -DEPTH / 2.0 + side_thickness / 2.0, (HEIGHT - 0.02) / 2.0 + 0.01)
        ),
        material=stainless,
        name="back_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * side_thickness, 0.56, 0.08)),
        origin=Origin(xyz=(0.0, -0.01, 0.04)),
        material=dark_trim,
        name="base_plinth",
    )
    body.visual(
        Box((WIDTH - 2.0 * side_thickness, fascia_thickness, FASCIA_HEIGHT)),
        origin=Origin(
            xyz=(0.0, DEPTH / 2.0 - fascia_thickness / 2.0, FASCIA_BOTTOM_Z + FASCIA_HEIGHT / 2.0)
        ),
        material=stainless,
        name="fascia",
    )
    body.visual(
        Box((0.50, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.004, 0.805)),
        material=black,
        name="control_band",
    )

    body.visual(
        Box((tub_width, tub_depth, 0.01)),
        origin=Origin(xyz=(0.0, -0.03, tub_floor_z)),
        material=tub_finish,
        name="tub_floor",
    )
    body.visual(
        Box((0.01, tub_depth, tub_height)),
        origin=Origin(xyz=(-tub_width / 2.0 + 0.005, -0.03, tub_floor_z + tub_height / 2.0)),
        material=tub_finish,
        name="tub_left_wall",
    )
    body.visual(
        Box((0.01, tub_depth, tub_height)),
        origin=Origin(xyz=(tub_width / 2.0 - 0.005, -0.03, tub_floor_z + tub_height / 2.0)),
        material=tub_finish,
        name="tub_right_wall",
    )
    body.visual(
        Box((tub_width, 0.01, tub_height)),
        origin=Origin(
            xyz=(0.0, -0.03 - tub_depth / 2.0 + 0.005, tub_floor_z + tub_height / 2.0)
        ),
        material=tub_finish,
        name="tub_back_wall",
    )
    body.visual(
        Box((tub_width, tub_depth, 0.01)),
        origin=Origin(xyz=(0.0, -0.03, tub_ceiling_z)),
        material=tub_finish,
        name="tub_roof",
    )
    rail_length = 0.50
    lower_runner_z = 0.240
    upper_runner_z = 0.505
    rail_y = -0.05
    rail_x = tub_width / 2.0 - 0.019
    for name, x_value, z_value in (
        ("lower_left_rail_lower", -rail_x, lower_runner_z - 0.012),
        ("lower_left_rail_upper", -rail_x, lower_runner_z + 0.012),
        ("lower_right_rail_lower", rail_x, lower_runner_z - 0.012),
        ("lower_right_rail_upper", rail_x, lower_runner_z + 0.012),
        ("upper_left_rail_lower", -rail_x, upper_runner_z - 0.012),
        ("upper_left_rail_upper", -rail_x, upper_runner_z + 0.012),
        ("upper_right_rail_lower", rail_x, upper_runner_z - 0.012),
        ("upper_right_rail_upper", rail_x, upper_runner_z + 0.012),
    ):
        body.visual(
            Box((0.018, rail_length, 0.006)),
            origin=Origin(xyz=(x_value, rail_y, z_value)),
            material=dark_trim,
            name=name,
        )

    door = model.part("door")
    door.visual(
        Box((0.57, 0.028, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.014, DOOR_HEIGHT / 2.0)),
        material=stainless,
        name="outer_panel",
    )
    door.visual(
        Box((0.57, DOOR_THICKNESS, 0.03)),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, 0.015)),
        material=stainless,
        name="bottom_return",
    )
    door.visual(
        Box((0.57, DOOR_THICKNESS, 0.03)),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT - 0.015)),
        material=stainless,
        name="top_return",
    )
    door.visual(
        Box((0.03, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.27, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=stainless,
        name="left_return",
    )
    door.visual(
        Box((0.03, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.27, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=stainless,
        name="right_return",
    )
    door.visual(
        Box((0.51, 0.004, 0.60)),
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS + 0.002, 0.355)),
        material=tub_finish,
        name="inner_liner",
    )

    cup_x = 0.12
    cup_z = 0.34
    cup_w = 0.13
    cup_h = 0.10
    cup_frame_t = 0.012
    cup_depth = 0.018
    door.visual(
        Box((cup_w + 0.02, cup_depth, 0.012)),
        origin=Origin(xyz=(cup_x, -0.023, cup_z + cup_h / 2.0 + 0.006)),
        material=accent,
        name="detergent_frame_top",
    )
    door.visual(
        Box((cup_w + 0.02, cup_depth, 0.012)),
        origin=Origin(xyz=(cup_x, -0.023, cup_z - cup_h / 2.0 - 0.006)),
        material=accent,
        name="detergent_frame_bottom",
    )
    door.visual(
        Box((0.012, cup_depth, cup_h)),
        origin=Origin(xyz=(cup_x - cup_w / 2.0 - 0.006, -0.023, cup_z)),
        material=accent,
        name="detergent_frame_left",
    )
    door.visual(
        Box((0.012, cup_depth, cup_h)),
        origin=Origin(xyz=(cup_x + cup_w / 2.0 + 0.006, -0.023, cup_z)),
        material=accent,
        name="detergent_frame_right",
    )
    door.visual(
        Box((0.026, 0.010, 0.012)),
        origin=Origin(xyz=(cup_x + cup_w / 2.0 - 0.008, -0.022, cup_z)),
        material=dark_trim,
        name="detergent_latch_pocket",
    )

    _add_rod(
        door,
        name="handle_bar",
        center=(0.0, 0.042, 0.47),
        axis="x",
        length=0.38,
        radius=0.010,
        material=stainless,
    )
    _add_rod(
        door,
        name="handle_post_0",
        center=(-0.14, 0.020, 0.47),
        axis="y",
        length=0.044,
        radius=0.008,
        material=stainless,
    )
    _add_rod(
        door,
        name="handle_post_1",
        center=(0.14, 0.020, 0.47),
        axis="y",
        length=0.044,
        radius=0.008,
        material=stainless,
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, DEPTH / 2.0, 0.04)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.032,
                body_style="skirted",
                top_diameter=0.045,
                skirt=KnobSkirt(0.068, 0.007, flare=0.10),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "dishwasher_cycle_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_shell",
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(-0.15, DEPTH / 2.0, 0.802)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.028, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=button_black,
        name="cap",
    )
    model.articulation(
        "start_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(0.115, DEPTH / 2.0, 0.802)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )

    delay_button = model.part("delay_button")
    delay_button.visual(
        Box((0.028, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=button_black,
        name="cap",
    )
    model.articulation(
        "delay_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=delay_button,
        origin=Origin(xyz=(0.185, DEPTH / 2.0, 0.802)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )

    lower_rack = model.part("lower_rack")
    _add_rack_wirework(
        lower_rack,
        width=0.48,
        depth=0.47,
        height=0.16,
        basket_center_y=-0.19,
        lower_z=-0.055,
        runner_offset_x=0.252,
        runner_length=0.54,
        rod_radius=0.004,
        material=wire_finish,
    )
    for index, y_value in enumerate((-0.22, -0.15, -0.08)):
        _add_rod(
            lower_rack,
            name=f"tine_base_{index}",
            center=(0.0, y_value, -0.055),
            axis="x",
            length=0.48,
            radius=0.003,
            material=wire_finish,
        )
    for row_index, y_value in enumerate((-0.22, -0.15, -0.08)):
        for col_index, x_value in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
            _add_rod(
                lower_rack,
                name=f"tine_{row_index}_{col_index}",
                center=(x_value, y_value, -0.005),
                axis="z",
                length=0.10,
                radius=0.0024,
                material=wire_finish,
            )
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.20, lower_runner_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=0.36,
        ),
    )

    upper_rack = model.part("upper_rack")
    _add_rack_wirework(
        upper_rack,
        width=0.48,
        depth=0.45,
        height=0.18,
        basket_center_y=-0.18,
        lower_z=-0.050,
        runner_offset_x=0.252,
        runner_length=0.52,
        rod_radius=0.004,
        material=wire_finish,
    )
    _add_rod(
        upper_rack,
        name="cup_tine_base",
        center=(0.0, -0.14, -0.050),
        axis="x",
        length=0.48,
        radius=0.003,
        material=wire_finish,
    )
    _add_rod(
        upper_rack,
        name="front_guard",
        center=(0.0, 0.04, 0.040),
        axis="x",
        length=0.48,
        radius=0.003,
        material=wire_finish,
    )
    for index, x_value in enumerate((-0.14, -0.07, 0.0, 0.07, 0.14)):
        _add_rod(
            upper_rack,
            name=f"cup_tine_{index}",
            center=(x_value, -0.14, -0.0075),
            axis="z",
            length=0.085,
            radius=0.0022,
            material=wire_finish,
        )
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.19, upper_runner_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=38.0,
            velocity=0.35,
            lower=0.0,
            upper=0.34,
        ),
    )

    lower_spray_arm = model.part("lower_spray_arm")
    _add_rod(
        lower_spray_arm,
        name="stem",
        center=(0.0, 0.0, 0.024),
        axis="z",
        length=0.052,
        radius=0.006,
        material=accent,
    )
    lower_spray_arm.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=accent,
        name="hub",
    )
    lower_spray_arm.visual(
        Box((0.22, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=accent,
        name="main_blade",
    )
    lower_spray_arm.visual(
        Box((0.020, 0.12, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=accent,
        name="cross_blade",
    )
    model.articulation(
        "lower_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, -0.03, 0.092)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    _add_rod(
        upper_spray_arm,
        name="stem",
        center=(0.0, 0.0, -0.025),
        axis="z",
        length=0.060,
        radius=0.006,
        material=accent,
    )
    upper_spray_arm.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=accent,
        name="hub",
    )
    upper_spray_arm.visual(
        Box((0.20, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=accent,
        name="main_blade",
    )
    upper_spray_arm.visual(
        Box((0.020, 0.10, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=accent,
        name="cross_blade",
    )
    model.articulation(
        "upper_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, -0.04, 0.705)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )

    detergent_lid = model.part("detergent_lid")
    detergent_lid.visual(
        Box((cup_w, 0.006, cup_h)),
        origin=Origin(xyz=(cup_w / 2.0, -0.003, 0.0)),
        material=tub_finish,
        name="lid_panel",
    )
    _add_rod(
        detergent_lid,
        name="hinge_barrel",
        center=(0.0, -0.003, 0.0),
        axis="z",
        length=0.075,
        radius=0.004,
        material=accent,
    )
    model.articulation(
        "detergent_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_lid,
        origin=Origin(xyz=(cup_x - cup_w / 2.0, -DOOR_THICKNESS + 0.004, cup_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    detergent_lid = object_model.get_part("detergent_lid")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    start_button = object_model.get_part("start_button")
    delay_button = object_model.get_part("delay_button")
    door_hinge = object_model.get_articulation("door_hinge")
    detergent_hinge = object_model.get_articulation("detergent_lid_hinge")
    knob_joint = object_model.get_articulation("knob_spin")
    start_joint = object_model.get_articulation("start_button_slide")
    delay_joint = object_model.get_articulation("delay_button_slide")
    lower_rack_joint = object_model.get_articulation("lower_rack_slide")
    upper_rack_joint = object_model.get_articulation("upper_rack_slide")
    lower_spray_joint = object_model.get_articulation("lower_spray_spin")
    upper_spray_joint = object_model.get_articulation("upper_spray_spin")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="z",
            positive_elem="fascia",
            negative_elem="outer_panel",
            min_gap=0.0,
            max_gap=0.004,
            name="door closes just under the fascia",
        )

    closed_door = None
    open_door = None
    with ctx.pose({door_hinge: 0.0}):
        closed_door = ctx.part_element_world_aabb(door, elem="outer_panel")
    with ctx.pose({door_hinge: 1.2}):
        open_door = ctx.part_element_world_aabb(door, elem="outer_panel")
    ctx.check(
        "door drops forward when opened",
        closed_door is not None
        and open_door is not None
        and open_door[1][1] > closed_door[1][1] + 0.20
        and open_door[1][2] < closed_door[1][2] - 0.08,
        details=f"closed={closed_door}, open={open_door}",
    )

    closed_lid = None
    open_lid = None
    with ctx.pose({detergent_hinge: 0.0}):
        closed_lid = ctx.part_element_world_aabb(detergent_lid, elem="lid_panel")
    with ctx.pose({detergent_hinge: 1.2}):
        open_lid = ctx.part_element_world_aabb(detergent_lid, elem="lid_panel")
    ctx.check(
        "detergent lid swings into the tub",
        closed_lid is not None
        and open_lid is not None
        and open_lid[0][1] < closed_lid[0][1] - 0.05,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    start_rest = ctx.part_world_position(start_button)
    delay_rest = ctx.part_world_position(delay_button)
    start_pressed = None
    delay_pressed = None
    with ctx.pose({start_joint: 0.004, delay_joint: 0.004}):
        start_pressed = ctx.part_world_position(start_button)
        delay_pressed = ctx.part_world_position(delay_button)
    ctx.check(
        "buttons push inward",
        start_rest is not None
        and delay_rest is not None
        and start_pressed is not None
        and delay_pressed is not None
        and start_pressed[1] < start_rest[1] - 0.003
        and delay_pressed[1] < delay_rest[1] - 0.003,
        details=(
            f"start_rest={start_rest}, start_pressed={start_pressed}, "
            f"delay_rest={delay_rest}, delay_pressed={delay_pressed}"
        ),
    )

    ctx.check(
        "cycle knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "buttons use prismatic travel",
        start_joint.articulation_type == ArticulationType.PRISMATIC
        and delay_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"start_type={start_joint.articulation_type}, "
            f"delay_type={delay_joint.articulation_type}"
        ),
    )

    ctx.allow_overlap(
        lower_rack,
        body,
        elem_a="left_runner",
        elem_b="lower_left_rail_lower",
        reason="The lower rack runner is intentionally represented as a solid member sliding inside the lower rail proxy.",
    )
    ctx.allow_overlap(
        lower_rack,
        body,
        elem_a="right_runner",
        elem_b="lower_right_rail_lower",
        reason="The lower rack runner is intentionally represented as a solid member sliding inside the lower rail proxy.",
    )
    ctx.allow_overlap(
        upper_rack,
        body,
        elem_a="left_runner",
        elem_b="upper_left_rail_lower",
        reason="The upper rack runner is intentionally represented as a solid member sliding inside the lower half of the side rail proxy.",
    )
    ctx.allow_overlap(
        upper_rack,
        body,
        elem_a="right_runner",
        elem_b="upper_right_rail_lower",
        reason="The upper rack runner is intentionally represented as a solid member sliding inside the lower half of the side rail proxy.",
    )

    lower_rack_rest = ctx.part_world_position(lower_rack)
    upper_rack_rest = ctx.part_world_position(upper_rack)
    lower_rack_extended = None
    upper_rack_extended = None
    lower_rack_upper = lower_rack_joint.motion_limits.upper if lower_rack_joint.motion_limits else 0.36
    upper_rack_upper = upper_rack_joint.motion_limits.upper if upper_rack_joint.motion_limits else 0.34
    with ctx.pose({lower_rack_joint: lower_rack_upper, upper_rack_joint: upper_rack_upper}):
        lower_rack_extended = ctx.part_world_position(lower_rack)
        upper_rack_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="lower_left_rail_upper",
            min_overlap=0.10,
            name="lower rack stays retained in its side rail",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="upper_left_rail_upper",
            min_overlap=0.10,
            name="upper rack stays retained in its side rail",
        )
    ctx.check(
        "racks extend forward",
        lower_rack_rest is not None
        and upper_rack_rest is not None
        and lower_rack_extended is not None
        and upper_rack_extended is not None
        and lower_rack_extended[1] > lower_rack_rest[1] + 0.30
        and upper_rack_extended[1] > upper_rack_rest[1] + 0.26,
        details=(
            f"lower_rest={lower_rack_rest}, lower_extended={lower_rack_extended}, "
            f"upper_rest={upper_rack_rest}, upper_extended={upper_rack_extended}"
        ),
    )

    ctx.check(
        "spray arms are continuous",
        lower_spray_joint.articulation_type == ArticulationType.CONTINUOUS
        and upper_spray_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"lower_type={lower_spray_joint.articulation_type}, "
            f"upper_type={upper_spray_joint.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
