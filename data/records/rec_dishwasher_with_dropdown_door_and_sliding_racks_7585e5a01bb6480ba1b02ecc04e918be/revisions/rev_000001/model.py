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
    model = ArticulatedObject(name="panel_ready_dishwasher")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    coated_wire = model.material("coated_wire", rgba=(0.86, 0.87, 0.90, 1.0))
    cabinet_panel = model.material("cabinet_panel", rgba=(0.58, 0.44, 0.28, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))

    body_w = 0.598
    body_d = 0.580
    body_h = 0.865
    shell_t = 0.020
    floor_t = 0.024
    top_t = 0.018

    door_h = 0.760
    door_outer_t = 0.020
    door_depth = 0.046
    inner_liner_w = 0.542
    inner_liner_h = 0.650
    inner_liner_t = 0.012

    body = model.part("body")
    body.visual(
        Box((shell_t, body_d, body_h)),
        origin=Origin(xyz=(-(body_w - shell_t) / 2.0, body_d / 2.0, body_h / 2.0)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, body_d, body_h)),
        origin=Origin(xyz=((body_w - shell_t) / 2.0, body_d / 2.0, body_h / 2.0)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((body_w, shell_t, body_h)),
        origin=Origin(xyz=(0.0, body_d - shell_t / 2.0, body_h / 2.0)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((body_w, body_d, floor_t)),
        origin=Origin(xyz=(0.0, body_d / 2.0, floor_t / 2.0)),
        material=stainless,
        name="floor_pan",
    )
    body.visual(
        Box((body_w, body_d, top_t)),
        origin=Origin(xyz=(0.0, body_d / 2.0, body_h - top_t / 2.0)),
        material=stainless,
        name="roof",
    )
    body.visual(
        Box((body_w, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, 0.0725, 0.045)),
        material=black,
        name="toe_kick",
    )
    body.visual(
        Box((body_w - 0.040, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, 0.020, body_h - 0.0225)),
        material=dark_trim,
        name="top_frame",
    )
    body.visual(
        Box((inner_liner_w + 0.012, 0.002, 0.022)),
        origin=Origin(xyz=(0.0, -0.001, 0.029)),
        material=dark_trim,
        name="hinge_sill",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.300, 0.049)),
        material=dark_trim,
        name="lower_spray_mount",
    )

    rail_len = body_d - 0.090
    rail_y = 0.060 + rail_len / 2.0
    rail_x = body_w / 2.0 - shell_t - 0.004
    rail_size = (0.022, rail_len, 0.016)
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(-rail_x, rail_y, 0.185)),
        material=dark_trim,
        name="lower_rail_left",
    )
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(rail_x, rail_y, 0.185)),
        material=dark_trim,
        name="lower_rail_right",
    )
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(-rail_x, rail_y, 0.455)),
        material=dark_trim,
        name="upper_rail_left",
    )
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(rail_x, rail_y, 0.455)),
        material=dark_trim,
        name="upper_rail_right",
    )
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(-rail_x, rail_y, 0.730)),
        material=dark_trim,
        name="tray_rail_left",
    )
    body.visual(
        Box(rail_size),
        origin=Origin(xyz=(rail_x, rail_y, 0.730)),
        material=dark_trim,
        name="tray_rail_right",
    )

    door = model.part("door")
    door.visual(
        Box((body_w - 0.006, door_outer_t, door_h)),
        origin=Origin(xyz=(0.0, -door_outer_t / 2.0, door_h / 2.0)),
        material=cabinet_panel,
        name="outer_panel",
    )
    door.visual(
        Box((inner_liner_w, inner_liner_t, inner_liner_h)),
        origin=Origin(xyz=(0.0, door_depth / 2.0 - inner_liner_t / 2.0, 0.075 + inner_liner_h / 2.0)),
        material=stainless,
        name="inner_liner",
    )
    door.visual(
        Box((0.028, door_depth, inner_liner_h + 0.030)),
        origin=Origin(xyz=(-0.265, 0.0, 0.060 + (inner_liner_h + 0.030) / 2.0)),
        material=dark_trim,
        name="left_edge",
    )
    door.visual(
        Box((0.028, door_depth, inner_liner_h + 0.030)),
        origin=Origin(xyz=(0.265, 0.0, 0.060 + (inner_liner_h + 0.030) / 2.0)),
        material=dark_trim,
        name="right_edge",
    )
    door.visual(
        Box((inner_liner_w + 0.010, door_depth, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_trim,
        name="bottom_edge",
    )
    door.visual(
        Box((inner_liner_w + 0.010, door_depth, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, door_h - 0.0225)),
        material=dark_trim,
        name="top_strip",
    )
    door.visual(
        Box((0.440, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, -0.036, door_h - 0.088)),
        material=stainless,
        name="handle_rail",
    )
    for x, name in ((-0.180, "handle_mount_left"), (0.180, "handle_mount_right")):
        door.visual(
            Box((0.035, 0.024, 0.060)),
            origin=Origin(xyz=(x, -0.023, door_h - 0.103)),
            material=stainless,
            name=name,
        )
    door.visual(
        Box((0.180, 0.010, 0.090)),
        origin=Origin(xyz=(0.120, door_depth / 2.0 - 0.005, 0.510)),
        material=dark_trim,
        name="dispenser_panel",
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        Box((0.018, 0.440, 0.016)),
        origin=Origin(xyz=(-0.257, 0.220, 0.0)),
        material=dark_trim,
        name="left_runner",
    )
    lower_rack.visual(
        Box((0.018, 0.440, 0.016)),
        origin=Origin(xyz=(0.257, 0.220, 0.0)),
        material=dark_trim,
        name="right_runner",
    )
    lower_rack.visual(
        Box((0.500, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.030, 0.030)),
        material=coated_wire,
        name="front_wall",
    )
    lower_rack.visual(
        Box((0.500, 0.020, 0.090)),
        origin=Origin(xyz=(0.0, 0.490, 0.045)),
        material=coated_wire,
        name="rear_wall",
    )
    lower_rack.visual(
        Box((0.020, 0.450, 0.080)),
        origin=Origin(xyz=(-0.220, 0.255, 0.048)),
        material=coated_wire,
        name="left_frame",
    )
    lower_rack.visual(
        Box((0.020, 0.450, 0.080)),
        origin=Origin(xyz=(0.220, 0.255, 0.048)),
        material=coated_wire,
        name="right_frame",
    )
    for index, x in enumerate((-0.150, -0.075, 0.0, 0.075, 0.150)):
        lower_rack.visual(
            Box((0.012, 0.450, 0.010)),
            origin=Origin(xyz=(x, 0.255, 0.005)),
            material=coated_wire,
            name=f"floor_slat_{index}",
        )
    lower_rack.visual(
        Box((0.010, 0.320, 0.018)),
        origin=Origin(xyz=(-0.214, 0.255, 0.016)),
        material=coated_wire,
        name="left_tine_mount",
    )
    lower_rack.visual(
        Box((0.010, 0.320, 0.018)),
        origin=Origin(xyz=(0.214, 0.255, 0.016)),
        material=coated_wire,
        name="right_tine_mount",
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        Box((0.018, 0.430, 0.016)),
        origin=Origin(xyz=(-0.257, 0.215, 0.0)),
        material=dark_trim,
        name="left_runner",
    )
    upper_rack.visual(
        Box((0.018, 0.430, 0.016)),
        origin=Origin(xyz=(0.257, 0.215, 0.0)),
        material=dark_trim,
        name="right_runner",
    )
    upper_rack.visual(
        Box((0.500, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.028, 0.028)),
        material=coated_wire,
        name="front_wall",
    )
    upper_rack.visual(
        Box((0.500, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, 0.432, 0.040)),
        material=coated_wire,
        name="rear_wall",
    )
    upper_rack.visual(
        Box((0.020, 0.400, 0.070)),
        origin=Origin(xyz=(-0.215, 0.230, 0.043)),
        material=coated_wire,
        name="left_frame",
    )
    upper_rack.visual(
        Box((0.020, 0.400, 0.070)),
        origin=Origin(xyz=(0.215, 0.230, 0.043)),
        material=coated_wire,
        name="right_frame",
    )
    for index, x in enumerate((-0.135, -0.045, 0.045, 0.135)):
        upper_rack.visual(
            Box((0.012, 0.400, 0.010)),
            origin=Origin(xyz=(x, 0.230, 0.005)),
            material=coated_wire,
            name=f"floor_slat_{index}",
        )
    for x, name in ((-0.090, "bottle_rail_left"), (0.090, "bottle_rail_right")):
        upper_rack.visual(
            Box((0.018, 0.400, 0.030)),
            origin=Origin(xyz=(x, 0.230, 0.020)),
            material=coated_wire,
            name=name,
        )
    upper_rack.visual(
        Box((0.012, 0.400, 0.010)),
        origin=Origin(xyz=(0.0, 0.230, 0.005)),
        material=coated_wire,
        name="floor_slat_center",
    )
    upper_rack.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.215, -0.010)),
        material=dark_trim,
        name="spray_mount",
    )

    utensil_tray = model.part("utensil_tray")
    utensil_tray.visual(
        Box((0.016, 0.430, 0.014)),
        origin=Origin(xyz=(-0.257, 0.215, 0.0)),
        material=dark_trim,
        name="left_runner",
    )
    utensil_tray.visual(
        Box((0.016, 0.430, 0.014)),
        origin=Origin(xyz=(0.257, 0.215, 0.0)),
        material=dark_trim,
        name="right_runner",
    )
    utensil_tray.visual(
        Box((0.520, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.028, 0.014)),
        material=coated_wire,
        name="front_lip",
    )
    utensil_tray.visual(
        Box((0.520, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.432, 0.014)),
        material=coated_wire,
        name="rear_lip",
    )
    utensil_tray.visual(
        Box((0.018, 0.404, 0.028)),
        origin=Origin(xyz=(-0.236, 0.230, 0.014)),
        material=coated_wire,
        name="left_lip",
    )
    utensil_tray.visual(
        Box((0.018, 0.404, 0.028)),
        origin=Origin(xyz=(0.236, 0.230, 0.014)),
        material=coated_wire,
        name="right_lip",
    )
    for index, x in enumerate((-0.170, -0.085, 0.0, 0.085, 0.170)):
        utensil_tray.visual(
            Box((0.010, 0.404, 0.008)),
            origin=Origin(xyz=(x, 0.230, 0.004)),
            material=coated_wire,
            name=f"deck_slat_{index}",
        )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_trim,
        name="hub",
    )
    lower_spray_arm.visual(
        Box((0.210, 0.024, 0.012)),
        origin=Origin(xyz=(0.098, 0.0, 0.009)),
        material=dark_trim,
        name="long_arm",
    )
    lower_spray_arm.visual(
        Box((0.148, 0.020, 0.012)),
        origin=Origin(xyz=(-0.070, 0.0, 0.009)),
        material=dark_trim,
        name="short_arm",
    )
    lower_spray_arm.visual(
        Box((0.030, 0.088, 0.010)),
        origin=Origin(xyz=(0.040, 0.044, 0.009)),
        material=dark_trim,
        name="branch",
    )
    lower_spray_arm.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.200, 0.010, 0.009)),
        material=dark_trim,
        name="tip",
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_trim,
        name="hub",
    )
    upper_spray_arm.visual(
        Box((0.170, 0.020, 0.010)),
        origin=Origin(xyz=(0.080, 0.0, -0.009)),
        material=dark_trim,
        name="long_arm",
    )
    upper_spray_arm.visual(
        Box((0.118, 0.018, 0.010)),
        origin=Origin(xyz=(-0.056, 0.0, -0.009)),
        material=dark_trim,
        name="short_arm",
    )
    upper_spray_arm.visual(
        Box((0.022, 0.070, 0.009)),
        origin=Origin(xyz=(-0.010, -0.035, -0.009)),
        material=dark_trim,
        name="branch",
    )

    left_tine_bank = model.part("left_tine_bank")
    left_tine_bank.visual(
        Cylinder(radius=0.004, length=0.320),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_rod",
    )
    left_tine_bank.visual(
        Box((0.090, 0.320, 0.008)),
        origin=Origin(xyz=(0.047, 0.0, 0.004)),
        material=coated_wire,
        name="base_plate",
    )
    for index, y in enumerate((-0.120, -0.060, 0.0, 0.060, 0.120)):
        left_tine_bank.visual(
            Box((0.010, 0.010, 0.070)),
            origin=Origin(xyz=(0.074, y, 0.043)),
            material=coated_wire,
            name=f"tine_{index}",
        )

    right_tine_bank = model.part("right_tine_bank")
    right_tine_bank.visual(
        Cylinder(radius=0.004, length=0.320),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_rod",
    )
    right_tine_bank.visual(
        Box((0.090, 0.320, 0.008)),
        origin=Origin(xyz=(-0.047, 0.0, 0.004)),
        material=coated_wire,
        name="base_plate",
    )
    for index, y in enumerate((-0.120, -0.060, 0.0, 0.060, 0.120)):
        right_tine_bank.visual(
            Box((0.010, 0.010, 0.070)),
            origin=Origin(xyz=(-0.074, y, 0.043)),
            material=coated_wire,
            name=f"tine_{index}",
        )

    rinse_cap = model.part("rinse_cap")
    rinse_cap.visual(
        Cylinder(radius=0.020, length=0.009),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="cap",
    )
    rinse_cap.visual(
        Box((0.010, 0.009, 0.008)),
        origin=Origin(xyz=(0.016, 0.004, 0.0)),
        material=dark_trim,
        name="grip",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.032, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black,
        name="cap",
    )
    start_button.visual(
        Box((0.014, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=black,
        name="stem",
    )

    cancel_button = model.part("cancel_button")
    cancel_button.visual(
        Box((0.032, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black,
        name="cap",
    )
    cancel_button.visual(
        Box((0.014, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=black,
        name="stem",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.025, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.58, effort=45.0, velocity=1.2),
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.060, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.340, effort=20.0, velocity=0.35),
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.060, 0.455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.300, effort=18.0, velocity=0.35),
    )
    model.articulation(
        "body_to_utensil_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=utensil_tray,
        origin=Origin(xyz=(0.0, 0.060, 0.730)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.260, effort=12.0, velocity=0.30),
    )
    model.articulation(
        "body_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.300, 0.074)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.215, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    model.articulation(
        "lower_rack_to_left_tine_bank",
        ArticulationType.REVOLUTE,
        parent=lower_rack,
        child=left_tine_bank,
        origin=Origin(xyz=(-0.205, 0.255, 0.021)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=2.0, velocity=1.2),
    )
    model.articulation(
        "lower_rack_to_right_tine_bank",
        ArticulationType.REVOLUTE,
        parent=lower_rack,
        child=right_tine_bank,
        origin=Origin(xyz=(0.205, 0.255, 0.021)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=2.0, velocity=1.2),
    )
    model.articulation(
        "door_to_rinse_cap",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=rinse_cap,
        origin=Origin(xyz=(0.175, door_depth / 2.0, 0.510)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )
    model.articulation(
        "door_to_start_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=start_button,
        origin=Origin(xyz=(0.075, 0.0, door_h)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.004, effort=0.5, velocity=0.05),
    )
    model.articulation(
        "door_to_cancel_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=cancel_button,
        origin=Origin(xyz=(0.125, 0.0, door_h)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.004, effort=0.5, velocity=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    utensil_tray = object_model.get_part("utensil_tray")
    left_tine_bank = object_model.get_part("left_tine_bank")
    right_tine_bank = object_model.get_part("right_tine_bank")
    start_button = object_model.get_part("start_button")
    cancel_button = object_model.get_part("cancel_button")
    rinse_cap = object_model.get_part("rinse_cap")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")

    door_hinge = object_model.get_articulation("body_to_door")
    lower_slide = object_model.get_articulation("body_to_lower_rack")
    upper_slide = object_model.get_articulation("body_to_upper_rack")
    tray_slide = object_model.get_articulation("body_to_utensil_tray")
    left_tine_joint = object_model.get_articulation("lower_rack_to_left_tine_bank")
    right_tine_joint = object_model.get_articulation("lower_rack_to_right_tine_bank")
    start_joint = object_model.get_articulation("door_to_start_button")
    cancel_joint = object_model.get_articulation("door_to_cancel_button")
    rinse_joint = object_model.get_articulation("door_to_rinse_cap")
    lower_spray_joint = object_model.get_articulation("body_to_lower_spray_arm")
    upper_spray_joint = object_model.get_articulation("upper_rack_to_upper_spray_arm")

    def aabb_center(box):
        if box is None:
            return None
        return tuple((box[0][i] + box[1][i]) * 0.5 for i in range(3))

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="outer_panel",
            min_overlap=0.550,
            name="closed door spans the full front opening width",
        )
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="lower_rail_left",
            min_overlap=0.430,
            name="lower rack sits fully on its guide rail at rest",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="upper_rail_left",
            min_overlap=0.420,
            name="upper rack sits fully on its guide rail at rest",
        )
        ctx.expect_overlap(
            utensil_tray,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="tray_rail_left",
            min_overlap=0.420,
            name="utensil tray sits fully on its guide rail at rest",
        )

    outer_closed = ctx.part_element_world_aabb(door, elem="outer_panel")
    with ctx.pose({door_hinge: 1.50}):
        outer_open = ctx.part_element_world_aabb(door, elem="outer_panel")

    ctx.check(
        "door drops outward on bottom hinge",
        outer_closed is not None
        and outer_open is not None
        and outer_open[0][1] < outer_closed[0][1] - 0.25
        and outer_open[1][2] < outer_closed[1][2] - 0.20,
        details=f"closed={outer_closed}, open={outer_open}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    upper_rest = ctx.part_world_position(upper_rack)
    tray_rest = ctx.part_world_position(utensil_tray)
    with ctx.pose({lower_slide: 0.340, upper_slide: 0.300, tray_slide: 0.260}):
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="lower_rail_left",
            min_overlap=0.095,
            name="lower rack keeps insertion at full extension",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="upper_rail_left",
            min_overlap=0.125,
            name="upper rack keeps insertion at full extension",
        )
        ctx.expect_overlap(
            utensil_tray,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="tray_rail_left",
            min_overlap=0.165,
            name="utensil tray keeps insertion at full extension",
        )
        lower_extended = ctx.part_world_position(lower_rack)
        upper_extended = ctx.part_world_position(upper_rack)
        tray_extended = ctx.part_world_position(utensil_tray)

    ctx.check(
        "all three rack levels slide outward",
        lower_rest is not None
        and lower_extended is not None
        and upper_rest is not None
        and upper_extended is not None
        and tray_rest is not None
        and tray_extended is not None
        and lower_extended[1] < lower_rest[1] - 0.30
        and upper_extended[1] < upper_rest[1] - 0.25
        and tray_extended[1] < tray_rest[1] - 0.20,
        details=(
            f"lower_rest={lower_rest}, lower_extended={lower_extended}, "
            f"upper_rest={upper_rest}, upper_extended={upper_extended}, "
            f"tray_rest={tray_rest}, tray_extended={tray_extended}"
        ),
    )

    left_bank_rest = ctx.part_world_aabb(left_tine_bank)
    right_bank_rest = ctx.part_world_aabb(right_tine_bank)
    with ctx.pose({left_tine_joint: 1.20, right_tine_joint: 1.20}):
        left_bank_folded = ctx.part_world_aabb(left_tine_bank)
        right_bank_folded = ctx.part_world_aabb(right_tine_bank)

    ctx.check(
        "lower rack tine banks fold inward",
        left_bank_rest is not None
        and right_bank_rest is not None
        and left_bank_folded is not None
        and right_bank_folded is not None
        and left_bank_folded[1][2] < left_bank_rest[1][2] - 0.07
        and right_bank_folded[1][2] < right_bank_rest[1][2] - 0.07
        and left_bank_folded[1][0] > left_bank_rest[1][0] + 0.008
        and right_bank_folded[0][0] < right_bank_rest[0][0] - 0.008,
        details=(
            f"left_rest={left_bank_rest}, left_folded={left_bank_folded}, "
            f"right_rest={right_bank_rest}, right_folded={right_bank_folded}"
        ),
    )

    start_rest = ctx.part_world_position(start_button)
    cancel_rest = ctx.part_world_position(cancel_button)
    with ctx.pose({start_joint: 0.004}):
        start_pressed = ctx.part_world_position(start_button)
        cancel_during_start = ctx.part_world_position(cancel_button)
    with ctx.pose({cancel_joint: 0.004}):
        cancel_pressed = ctx.part_world_position(cancel_button)
        start_during_cancel = ctx.part_world_position(start_button)

    ctx.check(
        "hidden start and cancel buttons move independently",
        start_rest is not None
        and cancel_rest is not None
        and start_pressed is not None
        and cancel_pressed is not None
        and cancel_during_start is not None
        and start_during_cancel is not None
        and start_pressed[2] < start_rest[2] - 0.003
        and cancel_pressed[2] < cancel_rest[2] - 0.003
        and abs(cancel_during_start[2] - cancel_rest[2]) < 1e-6
        and abs(start_during_cancel[2] - start_rest[2]) < 1e-6,
        details=(
            f"start_rest={start_rest}, start_pressed={start_pressed}, "
            f"cancel_rest={cancel_rest}, cancel_pressed={cancel_pressed}, "
            f"cancel_during_start={cancel_during_start}, start_during_cancel={start_during_cancel}"
        ),
    )

    rinse_rest = ctx.part_element_world_aabb(rinse_cap, elem="grip")
    lower_arm_rest = ctx.part_element_world_aabb(lower_spray_arm, elem="long_arm")
    upper_arm_rest = ctx.part_element_world_aabb(upper_spray_arm, elem="long_arm")
    with ctx.pose({rinse_joint: 1.00, lower_spray_joint: 1.00, upper_spray_joint: 1.00}):
        rinse_turned = ctx.part_element_world_aabb(rinse_cap, elem="grip")
        lower_arm_turned = ctx.part_element_world_aabb(lower_spray_arm, elem="long_arm")
        upper_arm_turned = ctx.part_element_world_aabb(upper_spray_arm, elem="long_arm")

    rinse_rest_center = aabb_center(rinse_rest)
    rinse_turned_center = aabb_center(rinse_turned)
    lower_arm_rest_center = aabb_center(lower_arm_rest)
    lower_arm_turned_center = aabb_center(lower_arm_turned)
    upper_arm_rest_center = aabb_center(upper_arm_rest)
    upper_arm_turned_center = aabb_center(upper_arm_turned)

    ctx.check(
        "rinse cap and both spray arms rotate on their local axes",
        rinse_rest_center is not None
        and rinse_turned_center is not None
        and lower_arm_rest_center is not None
        and lower_arm_turned_center is not None
        and upper_arm_rest_center is not None
        and upper_arm_turned_center is not None
        and rinse_turned_center[2] < rinse_rest_center[2] - 0.010
        and lower_arm_turned_center[1] > lower_arm_rest_center[1] + 0.07
        and upper_arm_turned_center[1] > upper_arm_rest_center[1] + 0.05,
        details=(
            f"rinse_rest={rinse_rest_center}, rinse_turned={rinse_turned_center}, "
            f"lower_arm_rest={lower_arm_rest_center}, lower_arm_turned={lower_arm_turned_center}, "
            f"upper_arm_rest={upper_arm_rest_center}, upper_arm_turned={upper_arm_turned_center}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
