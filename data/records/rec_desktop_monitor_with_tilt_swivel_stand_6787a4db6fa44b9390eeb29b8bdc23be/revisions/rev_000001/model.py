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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_monitor_24in")

    charcoal = model.material("charcoal_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    dark = model.material("dark_plastic", rgba=(0.055, 0.058, 0.064, 1.0))
    satin = model.material("satin_black", rgba=(0.09, 0.09, 0.095, 1.0))
    glass = model.material("black_glass", rgba=(0.006, 0.010, 0.016, 1.0))
    metal = model.material("dark_hinge_metal", rgba=(0.18, 0.18, 0.18, 1.0))
    led_green = model.material("green_status_led", rgba=(0.05, 0.75, 0.18, 1.0))

    # Overall display envelope is sized for a 24 inch 16:9 office monitor.
    display_w = 0.560
    display_h = 0.350
    active_w = 0.515
    active_h = 0.292
    case_depth = 0.042
    case_center_y = -0.052
    case_center_z = 0.020
    case_front_y = case_center_y - case_depth / 2.0
    frame_depth = 0.006
    frame_center_y = case_front_y - frame_depth / 2.0
    bottom_bezel_h = 0.040
    top_bezel_h = display_h - active_h - bottom_bezel_h
    side_bezel_w = (display_w - active_w) / 2.0
    active_center_z = case_center_z - display_h / 2.0 + bottom_bezel_h + active_h / 2.0

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.360, 0.240, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=charcoal,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin,
        name="swivel_boss",
    )

    stand = model.part("stand_spine")
    stand.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin,
        name="lower_collar",
    )
    stand.visual(
        Box((0.075, 0.048, 0.360)),
        origin=Origin(xyz=(0.0, 0.028, 0.180)),
        material=dark,
        name="spine_body",
    )
    stand.visual(
        Box((0.160, 0.055, 0.026)),
        origin=Origin(xyz=(0.0, -0.002, 0.365)),
        material=dark,
        name="top_saddle",
    )
    for x in (-0.075, 0.075):
        stand.visual(
            Box((0.022, 0.040, 0.080)),
            origin=Origin(xyz=(x, -0.005, 0.405)),
            material=dark,
            name=f"tilt_yoke_{'neg' if x < 0 else 'pos'}",
        )

    screen = model.part("screen_shell")
    screen.visual(
        Box((display_w, case_depth, display_h)),
        origin=Origin(xyz=(0.0, case_center_y, case_center_z)),
        material=charcoal,
        name="rear_case",
    )
    screen.visual(
        Box((display_w, frame_depth, bottom_bezel_h)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                case_center_z - display_h / 2.0 + bottom_bezel_h / 2.0,
            )
        ),
        material=dark,
        name="lower_bezel",
    )
    screen.visual(
        Box((display_w, frame_depth, top_bezel_h)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                case_center_z + display_h / 2.0 - top_bezel_h / 2.0,
            )
        ),
        material=dark,
        name="upper_bezel",
    )
    for x in (
        -display_w / 2.0 + side_bezel_w / 2.0,
        display_w / 2.0 - side_bezel_w / 2.0,
    ):
        screen.visual(
            Box((side_bezel_w, frame_depth, active_h)),
            origin=Origin(xyz=(x, frame_center_y, active_center_z)),
            material=dark,
            name=f"side_bezel_{'neg' if x < 0 else 'pos'}",
        )
    screen.visual(
        Box((active_w, 0.002, active_h)),
        origin=Origin(xyz=(0.0, case_front_y - 0.001, active_center_z)),
        material=glass,
        name="display_glass",
    )
    screen.visual(
        Box((0.006, 0.0015, 0.006)),
        origin=Origin(xyz=(0.258, frame_center_y - frame_depth / 2.0 - 0.00075, -0.135)),
        material=led_green,
        name="status_led",
    )
    screen.visual(
        Box((0.110, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=satin,
        name="rear_tilt_block",
    )
    screen.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_barrel",
    )

    door_w = 0.065
    door_h = 0.235
    door_t = 0.008
    neck_rear_y = 0.028 + 0.048 / 2.0
    hinge_r = 0.005
    door = model.part("cable_door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, -0.001, 0.0)),
        material=satin,
        name="door_flap",
    )
    door.visual(
        Cylinder(radius=hinge_r, length=door_h),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="door_barrel",
    )
    door.visual(
        Box((0.018, 0.004, 0.040)),
        origin=Origin(xyz=(door_w - 0.014, 0.005, 0.0)),
        material=dark,
        name="finger_pull",
    )

    button_xs = (0.158, 0.181, 0.204, 0.229)
    button_z = case_center_z - display_h / 2.0 + 0.020
    button_y = frame_center_y - frame_depth / 2.0 - 0.003
    for index, x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.015, 0.006, 0.008)),
            origin=Origin(),
            material=satin,
            name="button_cap",
        )
        model.articulation(
            f"screen_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=screen,
            child=button,
            origin=Origin(xyz=(x, button_y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=2.0, velocity=0.05),
        )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "stand_to_screen",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=screen,
        origin=Origin(xyz=(0.0, -0.005, 0.405)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.25, upper=0.35, effort=12.0, velocity=1.2),
    )
    model.articulation(
        "stand_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=door,
        origin=Origin(xyz=(-door_w / 2.0, neck_rear_y + hinge_r, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=1.5, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    screen = object_model.get_part("screen_shell")
    stand = object_model.get_part("stand_spine")
    door = object_model.get_part("cable_door")
    base = object_model.get_part("pedestal")
    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("stand_to_screen")
    door_hinge = object_model.get_articulation("stand_to_cable_door")

    ctx.expect_contact(
        stand,
        base,
        elem_a="lower_collar",
        elem_b="swivel_boss",
        name="swiveling stand sits on base boss",
    )
    ctx.expect_contact(
        door,
        stand,
        elem_a="door_flap",
        elem_b="spine_body",
        name="cable door rests on rear spine face",
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        ctx.expect_contact(
            button,
            screen,
            elem_a="button_cap",
            elem_b="lower_bezel",
            name=f"button_{index} is mounted in lower bezel",
        )
        joint = object_model.get_articulation(f"screen_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.003}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    with ctx.pose({tilt: 0.0}):
        rest_screen_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({tilt: 0.35}):
        tilted_screen_aabb = ctx.part_world_aabb(screen)
    ctx.check(
        "positive screen tilt leans rearward",
        rest_screen_aabb is not None
        and tilted_screen_aabb is not None
        and tilted_screen_aabb[1][1] > rest_screen_aabb[1][1] + 0.015,
        details=f"rest={rest_screen_aabb}, tilted={tilted_screen_aabb}",
    )

    with ctx.pose({door_hinge: 0.0}):
        closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.2}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "cable door swings outward from rear spine",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.035,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({swivel: 0.0}):
        rest_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({swivel: math.pi / 2.0}):
        swivel_aabb = ctx.part_world_aabb(screen)
    if rest_aabb is not None and swivel_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        swivel_center_x = (swivel_aabb[0][0] + swivel_aabb[1][0]) / 2.0
        swivel_ok = swivel_center_x > rest_center_x + 0.030
    else:
        rest_center_x = None
        swivel_center_x = None
        swivel_ok = False
    ctx.check(
        "stand swivels continuously about vertical base axis",
        swivel_ok,
        details=f"rest_center_x={rest_center_x}, swivel_center_x={swivel_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
