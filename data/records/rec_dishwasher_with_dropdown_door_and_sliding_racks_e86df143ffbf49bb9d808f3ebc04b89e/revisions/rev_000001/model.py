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
    model = ArticulatedObject(name="black_glass_dishwasher")

    black_glass = model.material("black_glass", rgba=(0.05, 0.05, 0.06, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.17, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    rack_metal = model.material("rack_metal", rgba=(0.78, 0.79, 0.81, 1.0))
    spray_gray = model.material("spray_gray", rgba=(0.54, 0.56, 0.58, 1.0))
    button_black = model.material("button_black", rgba=(0.14, 0.14, 0.15, 1.0))
    indicator_silver = model.material("indicator_silver", rgba=(0.84, 0.85, 0.87, 1.0))

    width = 0.60
    depth = 0.62
    height = 0.86
    case_t = 0.018
    liner_t = 0.010
    fascia_h = 0.11
    fascia_t = 0.028
    toe_kick_h = 0.08
    door_h = height - fascia_h - toe_kick_h
    door_t = 0.046
    inner_w = width - 2.0 * (case_t + 0.014)
    inner_d = depth - case_t - 0.070
    inner_h = height - toe_kick_h - fascia_h - 0.030
    inner_front_y = -depth / 2.0 + 0.030
    inner_back_y = inner_front_y + inner_d
    inner_center_y = (inner_front_y + inner_back_y) / 2.0
    inner_bottom_z = toe_kick_h + 0.016
    inner_top_z = inner_bottom_z + inner_h
    rail_length = inner_d - 0.060
    rack_front_y = inner_front_y + 0.045
    lower_rack_z = inner_bottom_z + 0.035
    upper_rack_z = inner_bottom_z + 0.315
    rack_travel = 0.28
    wire = 0.006
    runner_w = 0.010
    runner_h = 0.010
    lower_rack_w = inner_w - 0.050
    lower_rack_d = 0.445
    lower_rack_h = 0.125
    upper_rack_w = inner_w - 0.085
    upper_rack_d = 0.390
    upper_rack_h = 0.095

    def box_visual(part, size, xyz, material, name):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def rod_x(part, name, length, thickness, xyz, material):
        part.visual(
            Box((length, thickness, thickness)),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    def rod_y(part, name, length, thickness, xyz, material):
        part.visual(
            Box((thickness, length, thickness)),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    def rod_z(part, name, length, thickness, xyz, material):
        part.visual(
            Box((thickness, thickness, length)),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    body = model.part("body")
    box_visual(
        body,
        (case_t, depth, height),
        (-width / 2.0 + case_t / 2.0, 0.0, height / 2.0),
        dark_trim,
        "left_case",
    )
    box_visual(
        body,
        (case_t, depth, height),
        (width / 2.0 - case_t / 2.0, 0.0, height / 2.0),
        dark_trim,
        "right_case",
    )
    box_visual(
        body,
        (width, depth, case_t),
        (0.0, 0.0, case_t / 2.0),
        dark_trim,
        "floor_pan",
    )
    box_visual(
        body,
        (width, depth, case_t),
        (0.0, 0.0, height - case_t / 2.0),
        dark_trim,
        "top_skin",
    )
    box_visual(
        body,
        (width, case_t, height),
        (0.0, depth / 2.0 - case_t / 2.0, height / 2.0),
        dark_trim,
        "rear_case",
    )
    box_visual(
        body,
        (width, fascia_t, fascia_h),
        (0.0, -depth / 2.0 + fascia_t / 2.0, height - fascia_h / 2.0),
        fascia_black,
        "fascia",
    )
    box_visual(
        body,
        (width, fascia_t, toe_kick_h),
        (0.0, -depth / 2.0 + fascia_t / 2.0, toe_kick_h / 2.0),
        fascia_black,
        "toe_kick",
    )
    box_visual(
        body,
        (inner_w + 2.0 * liner_t, inner_d + 0.020, liner_t),
        (0.0, inner_center_y + 0.010, inner_bottom_z - liner_t / 2.0),
        stainless,
        "tub_floor",
    )
    box_visual(
        body,
        (inner_w + 2.0 * liner_t, inner_d + 0.020, liner_t),
        (0.0, inner_center_y + 0.010, inner_top_z + liner_t / 2.0),
        stainless,
        "tub_ceiling",
    )
    box_visual(
        body,
        (liner_t, inner_d + 0.020, inner_h + 2.0 * liner_t),
        (-inner_w / 2.0 - liner_t / 2.0, inner_center_y + 0.010, inner_bottom_z + inner_h / 2.0),
        stainless,
        "tub_left_wall",
    )
    box_visual(
        body,
        (liner_t, inner_d + 0.020, inner_h + 2.0 * liner_t),
        (inner_w / 2.0 + liner_t / 2.0, inner_center_y + 0.010, inner_bottom_z + inner_h / 2.0),
        stainless,
        "tub_right_wall",
    )
    box_visual(
        body,
        (inner_w + 2.0 * liner_t, liner_t, inner_h + 2.0 * liner_t),
        (0.0, inner_back_y + liner_t / 2.0 + 0.010, inner_bottom_z + inner_h / 2.0),
        stainless,
        "tub_back_wall",
    )
    box_visual(
        body,
        (0.022, 0.040, inner_h + 0.020),
        (-inner_w / 2.0 - 0.005, inner_front_y - 0.006, inner_bottom_z + inner_h / 2.0),
        stainless,
        "front_jamb_0",
    )
    box_visual(
        body,
        (0.022, 0.040, inner_h + 0.020),
        (inner_w / 2.0 + 0.005, inner_front_y - 0.006, inner_bottom_z + inner_h / 2.0),
        stainless,
        "front_jamb_1",
    )
    box_visual(
        body,
        (inner_w + 0.050, 0.040, 0.022),
        (0.0, inner_front_y - 0.006, inner_top_z + 0.005),
        stainless,
        "front_header",
    )
    box_visual(
        body,
        (inner_w + 0.050, 0.038, 0.022),
        (0.0, inner_front_y - 0.007, inner_bottom_z - 0.001),
        stainless,
        "front_sill",
    )
    box_visual(
        body,
        (0.020, rail_length, 0.012),
        (-(lower_rack_w / 2.0 - 0.012), rack_front_y + rail_length / 2.0, lower_rack_z + 0.018),
        spray_gray,
        "lower_rail_0",
    )
    box_visual(
        body,
        (0.020, rail_length, 0.012),
        (lower_rack_w / 2.0 - 0.012, rack_front_y + rail_length / 2.0, lower_rack_z + 0.018),
        spray_gray,
        "lower_rail_1",
    )
    box_visual(
        body,
        (0.020, rail_length, 0.012),
        (-(upper_rack_w / 2.0 - 0.012), rack_front_y + rail_length / 2.0, upper_rack_z + 0.030),
        spray_gray,
        "upper_rail_0",
    )
    box_visual(
        body,
        (0.020, rail_length, 0.012),
        (upper_rack_w / 2.0 - 0.012, rack_front_y + rail_length / 2.0, upper_rack_z + 0.030),
        spray_gray,
        "upper_rail_1",
    )
    box_visual(
        body,
        (0.14, 0.10, 0.026),
        (0.0, inner_front_y + 0.12, inner_bottom_z + 0.008),
        spray_gray,
        "sump_cover",
    )
    box_visual(
        body,
        (0.030, 0.030, 0.004),
        (0.0, inner_front_y + 0.215, inner_bottom_z + 0.002),
        spray_gray,
        "spray_pedestal",
    )

    door = model.part("door")
    box_visual(
        door,
        (width - 0.006, door_t * 0.55, door_h),
        (0.0, -door_t * 0.275, door_h / 2.0),
        black_glass,
        "outer_glass",
    )
    box_visual(
        door,
        (width - 0.050, 0.012, door_h - 0.080),
        (0.0, -0.002, door_h / 2.0 + 0.006),
        stainless,
        "inner_panel",
    )
    box_visual(
        door,
        (width - 0.040, 0.014, 0.030),
        (0.0, -0.003, 0.015),
        stainless,
        "bottom_rib",
    )
    box_visual(
        door,
        (0.018, 0.014, door_h - 0.040),
        (-inner_w / 2.0 + 0.012, -0.003, door_h / 2.0),
        stainless,
        "side_rib_0",
    )
    box_visual(
        door,
        (0.018, 0.014, door_h - 0.040),
        (inner_w / 2.0 - 0.012, -0.003, door_h / 2.0),
        stainless,
        "side_rib_1",
    )
    box_visual(
        door,
        (width - 0.040, 0.014, 0.028),
        (0.0, -0.002, door_h - 0.014),
        stainless,
        "top_rib",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.004, toe_kick_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.6, lower=0.0, upper=1.55),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=button_black,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="dial_skirt",
    )
    box_visual(
        dial,
        (0.018, 0.014, 0.018),
        (0.0, 0.006, 0.0),
        dark_trim,
        "dial_mount",
    )
    box_visual(
        dial,
        (0.006, 0.002, 0.020),
        (0.0, -0.025, 0.020),
        indicator_silver,
        "dial_pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(-0.175, -depth / 2.0 - 0.013, height - fascia_h / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    button_x = (0.030, 0.085, 0.140)
    for index, x_pos in enumerate(button_x):
        button = model.part(f"button_{index}")
        box_visual(
            button,
            (0.034, 0.010, 0.014),
            (0.0, -0.005, 0.0),
            button_black,
            "cap",
        )
        box_visual(
            button,
            (0.016, 0.010, 0.010),
            (0.0, 0.000, 0.0),
            dark_trim,
            "stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, -depth / 2.0 - 0.005, height - fascia_h / 2.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=0.005),
        )

    lower_rack = model.part("lower_rack")
    for side, sign in enumerate((-1.0, 1.0)):
        box_visual(
            lower_rack,
            (runner_w, lower_rack_d, runner_h),
            (sign * (lower_rack_w / 2.0 - 0.012), lower_rack_d / 2.0, 0.001),
            rack_metal,
            f"lower_runner_{side}",
        )
        rod_z(
            lower_rack,
            f"lower_post_front_{side}",
            lower_rack_h - 0.010,
            wire,
            (sign * (lower_rack_w / 2.0 - wire / 2.0), 0.012, lower_rack_h / 2.0 - 0.002),
            rack_metal,
        )
        rod_z(
            lower_rack,
            f"lower_post_back_{side}",
            lower_rack_h - 0.010,
            wire,
            (
                sign * (lower_rack_w / 2.0 - wire / 2.0),
                lower_rack_d - 0.012,
                lower_rack_h / 2.0 - 0.002,
            ),
            rack_metal,
        )
        rod_y(
            lower_rack,
            f"lower_side_top_{side}",
            lower_rack_d - 0.012,
            wire,
            (sign * (lower_rack_w / 2.0 - wire / 2.0), lower_rack_d / 2.0, lower_rack_h - 0.008),
            rack_metal,
        )
    rod_x(
        lower_rack,
        "lower_front_base",
        lower_rack_w,
        wire,
        (0.0, 0.012, 0.018),
        rack_metal,
    )
    rod_x(
        lower_rack,
        "lower_back_base",
        lower_rack_w,
        wire,
        (0.0, lower_rack_d - 0.012, 0.018),
        rack_metal,
    )
    rod_x(
        lower_rack,
        "lower_front_top",
        lower_rack_w,
        wire,
        (0.0, 0.012, lower_rack_h - 0.008),
        rack_metal,
    )
    rod_x(
        lower_rack,
        "lower_back_top",
        lower_rack_w,
        wire,
        (0.0, lower_rack_d - 0.012, lower_rack_h - 0.008),
        rack_metal,
    )
    for i, x_pos in enumerate((-0.120, 0.0, 0.120)):
        rod_y(
            lower_rack,
            f"lower_floor_long_{i}",
            lower_rack_d - 0.024,
            wire,
            (x_pos, lower_rack_d / 2.0, 0.018),
            rack_metal,
        )
    for i, y_pos in enumerate((0.080, 0.160, 0.240, 0.320)):
        rod_x(
            lower_rack,
            f"lower_floor_cross_{i}",
            lower_rack_w - 0.020,
            wire,
            (0.0, y_pos, 0.018),
            rack_metal,
        )
    rod_x(
        lower_rack,
        "lower_handle",
        0.180,
        wire,
        (0.0, -0.010, 0.082),
        rack_metal,
    )
    rod_z(
        lower_rack,
        "lower_handle_post_0",
        0.054,
        wire,
        (-0.080, -0.006, 0.053),
        rack_metal,
    )
    rod_z(
        lower_rack,
        "lower_handle_post_1",
        0.054,
        wire,
        (0.080, -0.006, 0.053),
        rack_metal,
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, rack_front_y, lower_rack_z + 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=rack_travel),
    )

    caddy = model.part("caddy")
    box_visual(caddy, (0.082, 0.155, 0.004), (0.0, 0.0, 0.002), stainless, "caddy_base")
    box_visual(caddy, (0.004, 0.155, 0.110), (-0.039, 0.0, 0.055), stainless, "caddy_wall_0")
    box_visual(caddy, (0.004, 0.155, 0.110), (0.039, 0.0, 0.055), stainless, "caddy_wall_1")
    box_visual(caddy, (0.082, 0.004, 0.110), (0.0, -0.0755, 0.055), stainless, "caddy_wall_2")
    box_visual(caddy, (0.082, 0.004, 0.110), (0.0, 0.0755, 0.055), stainless, "caddy_wall_3")
    box_visual(caddy, (0.004, 0.145, 0.092), (0.0, 0.0, 0.050), stainless, "caddy_divider")
    model.articulation(
        "lower_rack_to_caddy",
        ArticulationType.FIXED,
        parent=lower_rack,
        child=caddy,
        origin=Origin(xyz=(lower_rack_w / 2.0 - 0.070, 0.125, 0.018)),
    )

    upper_rack = model.part("upper_rack")
    for side, sign in enumerate((-1.0, 1.0)):
        box_visual(
            upper_rack,
            (runner_w, upper_rack_d, runner_h),
            (sign * (upper_rack_w / 2.0 - 0.012), upper_rack_d / 2.0, 0.012),
            rack_metal,
            f"upper_runner_{side}",
        )
        rod_z(
            upper_rack,
            f"upper_post_front_{side}",
            upper_rack_h - 0.010,
            wire,
            (sign * (upper_rack_w / 2.0 - wire / 2.0), 0.010, upper_rack_h / 2.0 - 0.002),
            rack_metal,
        )
        rod_z(
            upper_rack,
            f"upper_post_back_{side}",
            upper_rack_h - 0.010,
            wire,
            (
                sign * (upper_rack_w / 2.0 - wire / 2.0),
                upper_rack_d - 0.010,
                upper_rack_h / 2.0 - 0.002,
            ),
            rack_metal,
        )
        rod_y(
            upper_rack,
            f"upper_side_top_{side}",
            upper_rack_d - 0.010,
            wire,
            (sign * (upper_rack_w / 2.0 - wire / 2.0), upper_rack_d / 2.0, upper_rack_h - 0.008),
            rack_metal,
        )
    rod_x(upper_rack, "upper_front_base", upper_rack_w - 0.040, wire, (0.0, 0.010, 0.004), rack_metal)
    rod_x(
        upper_rack,
        "upper_back_base",
        upper_rack_w - 0.040,
        wire,
        (0.0, upper_rack_d - 0.010, 0.004),
        rack_metal,
    )
    rod_x(
        upper_rack,
        "upper_front_top",
        upper_rack_w - 0.040,
        wire,
        (0.0, 0.010, upper_rack_h - 0.008),
        rack_metal,
    )
    rod_x(
        upper_rack,
        "upper_back_top",
        upper_rack_w - 0.040,
        wire,
        (0.0, upper_rack_d - 0.010, upper_rack_h - 0.008),
        rack_metal,
    )
    for i, x_pos in enumerate((-0.100, 0.0, 0.100)):
        rod_y(
            upper_rack,
            f"upper_floor_long_{i}",
            upper_rack_d - 0.020,
            wire,
            (x_pos, upper_rack_d / 2.0, 0.004),
            rack_metal,
        )
    for i, y_pos in enumerate((0.075, 0.145, 0.215, 0.285)):
        rod_x(
            upper_rack,
            f"upper_floor_cross_{i}",
            upper_rack_w - 0.018,
            wire,
            (0.0, y_pos, 0.004),
            rack_metal,
        )
    rod_x(upper_rack, "upper_handle", 0.160, wire, (0.0, -0.010, 0.066), rack_metal)
    rod_z(upper_rack, "upper_handle_post_0", 0.040, wire, (-0.070, -0.006, 0.040), rack_metal)
    rod_z(upper_rack, "upper_handle_post_1", 0.040, wire, (0.070, -0.006, 0.040), rack_metal)
    rod_z(
        upper_rack,
        "upper_spray_mount",
        0.016,
        0.016,
        (0.0, upper_rack_d * 0.54, -0.004),
        spray_gray,
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, rack_front_y + 0.008, upper_rack_z + 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.260),
    )

    lower_spray = model.part("lower_spray")
    lower_spray.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=spray_gray,
        name="lower_hub",
    )
    box_visual(lower_spray, (0.030, 0.030, 0.026), (0.0, 0.0, -0.013), spray_gray, "lower_pedestal")
    box_visual(lower_spray, (0.260, 0.020, 0.008), (0.0, 0.0, 0.018), spray_gray, "lower_arm_main")
    box_visual(lower_spray, (0.130, 0.014, 0.008), (0.045, 0.055, 0.018), spray_gray, "lower_arm_aux")
    box_visual(lower_spray, (0.080, 0.012, 0.008), (-0.090, -0.040, 0.018), spray_gray, "lower_nozzle_bar")
    model.articulation(
        "body_to_lower_spray",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray,
        origin=Origin(xyz=(0.0, inner_front_y + 0.215, inner_bottom_z + 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    upper_spray = model.part("upper_spray")
    upper_spray.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=spray_gray,
        name="upper_hub",
    )
    box_visual(upper_spray, (0.220, 0.016, 0.006), (0.0, 0.0, 0.007), spray_gray, "upper_arm_main")
    box_visual(upper_spray, (0.100, 0.012, 0.006), (-0.030, 0.024, 0.007), spray_gray, "upper_arm_aux")
    box_visual(upper_spray, (0.016, 0.016, 0.020), (0.0, 0.0, -0.010), spray_gray, "upper_stem")
    model.articulation(
        "upper_rack_to_upper_spray",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray,
        origin=Origin(xyz=(0.0, upper_rack_d * 0.54, -0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    cup_shelf = model.part("cup_shelf")
    box_visual(cup_shelf, (0.006, 0.075, 0.110), (0.0, 0.0, 0.055), rack_metal, "shelf_panel")
    box_visual(cup_shelf, (0.014, 0.012, 0.012), (0.0, -0.026, 0.032), rack_metal, "pivot_0")
    box_visual(cup_shelf, (0.014, 0.012, 0.012), (0.0, 0.026, 0.032), rack_metal, "pivot_1")
    model.articulation(
        "upper_rack_to_cup_shelf",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=cup_shelf,
        origin=Origin(xyz=(upper_rack_w / 2.0 - 0.006, 0.235, 0.056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    upper_spray = object_model.get_part("upper_spray")
    cup_shelf = object_model.get_part("cup_shelf")
    door_joint = object_model.get_articulation("body_to_door")
    lower_rack_joint = object_model.get_articulation("body_to_lower_rack")
    upper_rack_joint = object_model.get_articulation("body_to_upper_rack")
    cup_shelf_joint = object_model.get_articulation("upper_rack_to_cup_shelf")
    button_0_joint = object_model.get_articulation("body_to_button_0")

    ctx.allow_overlap(
        body,
        lower_rack,
        elem_a="lower_rail_0",
        elem_b="lower_runner_0",
        reason="The lower rack runner is intentionally represented as sliding inside the left rail proxy.",
    )
    ctx.allow_overlap(
        body,
        lower_rack,
        elem_a="lower_rail_1",
        elem_b="lower_runner_1",
        reason="The lower rack runner is intentionally represented as sliding inside the right rail proxy.",
    )
    ctx.allow_overlap(
        body,
        upper_rack,
        elem_a="upper_rail_0",
        elem_b="upper_runner_0",
        reason="The upper rack runner is intentionally represented as sliding inside the left rail proxy.",
    )
    ctx.allow_overlap(
        body,
        upper_rack,
        elem_a="upper_rail_1",
        elem_b="upper_runner_1",
        reason="The upper rack runner is intentionally represented as sliding inside the right rail proxy.",
    )
    ctx.allow_overlap(
        upper_rack,
        upper_spray,
        elem_a="upper_spray_mount",
        elem_b="upper_hub",
        reason="The upper spray arm hub intentionally nests onto the rack-mounted spray feed boss.",
    )
    ctx.allow_overlap(
        upper_rack,
        cup_shelf,
        elem_a="upper_side_top_1",
        elem_b="pivot_0",
        reason="The cup shelf pivot is simplified as a wraparound pivot block riding on the rack side wire.",
    )
    ctx.allow_overlap(
        upper_rack,
        cup_shelf,
        elem_a="upper_side_top_1",
        elem_b="pivot_1",
        reason="The cup shelf uses paired simplified wraparound pivots around the rack side wire.",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="fascia",
            negative_elem="outer_glass",
            min_gap=0.0,
            max_gap=0.010,
            name="closed door sits just ahead of the fascia plane",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="outer_glass",
            elem_b="fascia",
            min_overlap=0.50,
            name="door spans the front opening width",
        )

    door_limits = door_joint.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        door_rest_aabb = ctx.part_world_aabb(door)
        with ctx.pose({door_joint: door_limits.upper}):
            door_open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door lowers when opened",
            door_rest_aabb is not None
            and door_open_aabb is not None
            and door_open_aabb[1][2] < door_rest_aabb[1][2] - 0.20,
            details=f"closed={door_rest_aabb}, open={door_open_aabb}",
        )

    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_mount",
        elem_b="fascia",
        name="cycle dial mounts on the fascia",
    )

    ctx.expect_within(
        lower_rack,
        body,
        axes="xz",
        inner_elem="lower_runner_0",
        outer_elem="lower_rail_0",
        margin=0.006,
        name="lower rack stays aligned on the left rail",
    )
    ctx.expect_within(
        upper_rack,
        body,
        axes="xz",
        inner_elem="upper_runner_0",
        outer_elem="upper_rail_0",
        margin=0.006,
        name="upper rack stays aligned on the left rail",
    )
    ctx.expect_overlap(
        lower_rack,
        body,
        axes="y",
        elem_a="lower_runner_0",
        elem_b="lower_rail_0",
        min_overlap=0.16,
        name="lower rack remains engaged on its rail at rest",
    )
    ctx.expect_overlap(
        upper_rack,
        body,
        axes="y",
        elem_a="upper_runner_0",
        elem_b="upper_rail_0",
        min_overlap=0.14,
        name="upper rack remains engaged on its rail at rest",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({lower_rack_joint: 0.28, upper_rack_joint: 0.26, cup_shelf_joint: 1.45, button_0_joint: 0.005}):
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            elem_a="lower_runner_0",
            elem_b="lower_rail_0",
            min_overlap=0.12,
            name="lower rack retains insertion when extended",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            elem_a="upper_runner_0",
            elem_b="upper_rail_0",
            min_overlap=0.10,
            name="upper rack retains insertion when extended",
        )
        lower_extended = ctx.part_world_position(lower_rack)
        upper_extended = ctx.part_world_position(upper_rack)
        shelf_open_aabb = ctx.part_world_aabb(cup_shelf)
        button_pressed = ctx.part_world_position(object_model.get_part("button_0"))
    shelf_rest_aabb = ctx.part_world_aabb(cup_shelf)
    button_rest = ctx.part_world_position(object_model.get_part("button_0"))

    ctx.check(
        "lower rack extends outward",
        lower_rest is not None and lower_extended is not None and lower_extended[1] < lower_rest[1] - 0.20,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "upper rack extends outward",
        upper_rest is not None and upper_extended is not None and upper_extended[1] < upper_rest[1] - 0.18,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )
    ctx.check(
        "cup shelf folds down",
        shelf_rest_aabb is not None
        and shelf_open_aabb is not None
        and shelf_open_aabb[1][2] < shelf_rest_aabb[1][2] - 0.04,
        details=f"rest={shelf_rest_aabb}, open={shelf_open_aabb}",
    )
    ctx.check(
        "program button pushes inward",
        button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
