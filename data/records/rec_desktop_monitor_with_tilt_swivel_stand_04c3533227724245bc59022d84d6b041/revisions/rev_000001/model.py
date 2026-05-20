from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_desktop_monitor")

    charcoal = model.material("charcoal_plastic", rgba=(0.045, 0.048, 0.052, 1.0))
    satin_black = model.material("satin_black", rgba=(0.010, 0.011, 0.013, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.02, 0.045, 0.070, 0.68))
    indicator_gray = model.material("control_mark_gray", rgba=(0.75, 0.76, 0.74, 1.0))

    # Coordinate convention: X is screen width, Y is depth (front is -Y), Z is up.
    base = model.part("base")
    base_plate = ExtrudeGeometry.centered(rounded_rect_profile(0.320, 0.225, 0.040), 0.028)
    base.visual(
        mesh_from_geometry(base_plate, "rounded_pedestal_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=charcoal,
        name="pedestal_plate",
    )
    base.visual(
        Cylinder(radius=0.063, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=graphite,
        name="swivel_bearing_well",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=graphite,
        name="turntable_disk",
    )
    stand.visual(
        Box((0.046, 0.035, 0.312)),
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=charcoal,
        name="slim_neck",
    )
    stand.visual(
        Box((0.118, 0.017, 0.030)),
        origin=Origin(xyz=(0.0, 0.026, 0.323)),
        material=charcoal,
        name="yoke_bridge",
    )
    stand.visual(
        Box((0.018, 0.046, 0.052)),
        origin=Origin(xyz=(-0.055, 0.0, 0.340)),
        material=charcoal,
        name="yoke_ear_0",
    )
    stand.visual(
        Box((0.018, 0.046, 0.052)),
        origin=Origin(xyz=(0.055, 0.0, 0.340)),
        material=charcoal,
        name="yoke_ear_1",
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0),
    )

    display = model.part("display")
    shell_profile = rounded_rect_profile(0.575, 0.365, 0.018)
    rear_shell = ExtrudeGeometry.centered(shell_profile, 0.046)
    display.visual(
        mesh_from_geometry(rear_shell, "rounded_display_back"),
        origin=Origin(xyz=(0.0, -0.050, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rear_shell",
    )
    front_bezel = BezelGeometry(
        (0.525, 0.295),
        (0.570, 0.365),
        0.014,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.006,
        outer_corner_radius=0.016,
        wall=(0.0225, 0.0225, 0.018, 0.052),
    )
    display.visual(
        mesh_from_geometry(front_bezel, "bottom_heavy_front_bezel"),
        origin=Origin(xyz=(0.0, -0.073, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="front_bezel",
    )
    display.visual(
        Box((0.533, 0.003, 0.303)),
        origin=Origin(xyz=(0.0, -0.0815, 0.020)),
        material=glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.070, 0.030, 0.025)),
        origin=Origin(xyz=(0.0, -0.015, 0.0135)),
        material=charcoal,
        name="hinge_mount",
    )
    display.visual(
        Cylinder(radius=0.017, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )

    model.articulation(
        "stand_to_display",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(18.0),
        ),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.030, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=dark_rubber,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.003, 0.001, 0.010)),
        origin=Origin(xyz=(-0.006, -0.0065, 0.0)),
        material=indicator_gray,
        name="rocker_mark",
    )
    model.articulation(
        "display_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=display,
        child=power_rocker,
        origin=Origin(xyz=(0.145, -0.080, -0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=2.0, lower=-0.24, upper=0.24),
    )

    for index, x_pos in enumerate((0.185, 0.215, 0.245)):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Box((0.017, 0.005, 0.010)),
            origin=Origin(xyz=(0.0, -0.0025, 0.0)),
            material=graphite,
            name="button_cap",
        )
        button.visual(
            Box((0.007, 0.0008, 0.0016)),
            origin=Origin(xyz=(0.0, -0.0054, 0.0)),
            material=indicator_gray,
            name="button_tick",
        )
        model.articulation(
            f"display_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x_pos, -0.080, -0.145)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=0.05, lower=0.0, upper=0.0035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    display = object_model.get_part("display")
    rocker = object_model.get_part("power_rocker")
    buttons = [object_model.get_part(f"menu_button_{i}") for i in range(3)]
    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("stand_to_display")

    ctx.check("24 inch class display part present", display is not None, "display part is missing")
    if display is not None:
        aabb = ctx.part_world_aabb(display)
        if aabb is not None:
            mins, maxs = aabb
            width = float(maxs[0] - mins[0])
            height = float(maxs[2] - mins[2])
            ctx.check("standard office monitor scale", 0.54 <= width <= 0.60 and 0.34 <= height <= 0.39)

    ctx.check("continuous stand swivel present", swivel is not None, "stand swivel joint is missing")
    ctx.check("screen tilt hinge present", tilt is not None, "screen tilt hinge is missing")

    if stand is not None and base is not None:
        ctx.expect_contact(
            stand,
            base,
            elem_a="turntable_disk",
            elem_b="swivel_bearing_well",
            name="stand turntable sits on base bearing",
        )
    if display is not None and stand is not None:
        ctx.expect_contact(
            display,
            stand,
            elem_a="hinge_barrel",
            elem_b="yoke_ear_0",
            contact_tol=0.0015,
            name="display hinge barrel captured by yoke ear",
        )

    if display is not None and rocker is not None:
        ctx.expect_gap(
            display,
            rocker,
            axis="y",
            positive_elem="front_bezel",
            negative_elem="rocker_cap",
            max_gap=0.001,
            max_penetration=0.00002,
            name="power rocker sits on lower front bezel",
        )

    for index, button in enumerate(buttons):
        if display is None or button is None:
            ctx.fail(f"menu button {index} present", "menu button part is missing")
            continue
        ctx.expect_gap(
            display,
            button,
            axis="y",
            positive_elem="front_bezel",
            negative_elem="button_cap",
            max_gap=0.001,
            max_penetration=0.00002,
            name=f"menu button {index} sits on lower front bezel",
        )
        ctx.expect_origin_distance(
            button,
            rocker,
            axes="x",
            min_dist=0.025,
            name=f"menu button {index} remains distinct from rocker",
        )

    button_joint = object_model.get_articulation("display_to_menu_button_0")
    if buttons[0] is not None and button_joint is not None:
        rest_pos = ctx.part_world_position(buttons[0])
        with ctx.pose({button_joint: 0.0035}):
            pressed_pos = ctx.part_world_position(buttons[0])
        ctx.check(
            "menu button depresses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    if display is not None and tilt is not None:
        rest_aabb = ctx.part_world_aabb(display)
        with ctx.pose({tilt: math.radians(18.0)}):
            tilted_aabb = ctx.part_world_aabb(display)
        ctx.check(
            "display shell tilts on horizontal hinge",
            rest_aabb is not None
            and tilted_aabb is not None
            and abs(float(tilted_aabb[0][1] - rest_aabb[0][1])) > 0.020,
            details=f"rest={rest_aabb}, tilted={tilted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
