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


def _aabb_size(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[index] - mins[index] for index in range(3))


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_monitor_24_inch")

    shell_black = model.material("shell_black", rgba=(0.13, 0.14, 0.15, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.07, 0.08, 0.09, 1.0))
    glass_black = model.material("glass_black", rgba=(0.05, 0.06, 0.07, 1.0))
    stand_black = model.material("stand_black", rgba=(0.18, 0.19, 0.20, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.28, 0.29, 0.31, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.240, 0.190, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stand_black,
        name="pedestal_plate",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=accent_grey,
        name="swivel_plinth",
    )
    base.visual(
        Box((0.120, 0.094, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=accent_grey,
        name="top_pad",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_black,
        name="swivel_collar",
    )
    stand.visual(
        Box((0.036, 0.016, 0.232)),
        origin=Origin(xyz=(0.0, 0.010, 0.134)),
        material=stand_black,
        name="neck",
    )
    stand.visual(
        Box((0.052, 0.014, 0.072)),
        origin=Origin(xyz=(0.0, 0.016, 0.218)),
        material=stand_black,
        name="rear_spine",
    )
    stand.visual(
        Box((0.072, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.018, 0.256)),
        material=stand_black,
        name="tilt_yoke",
    )
    stand.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(xyz=(0.0, 0.024, 0.258), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_grey,
        name="tilt_barrel",
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.542, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.021, 0.244)),
        material=bezel_black,
        name="top_bezel",
    )
    housing.visual(
        Box((0.542, 0.016, 0.028)),
        origin=Origin(xyz=(0.0, -0.020, -0.068)),
        material=bezel_black,
        name="lower_bezel",
    )
    housing.visual(
        Box((0.009, 0.008, 0.334)),
        origin=Origin(xyz=(-0.2665, -0.021, 0.085)),
        material=bezel_black,
        name="side_bezel_0",
    )
    housing.visual(
        Box((0.009, 0.008, 0.334)),
        origin=Origin(xyz=(0.2665, -0.021, 0.085)),
        material=bezel_black,
        name="side_bezel_1",
    )
    housing.visual(
        Box((0.526, 0.040, 0.318)),
        origin=Origin(xyz=(0.0, -0.024, 0.085)),
        material=shell_black,
        name="rear_shell",
    )
    housing.visual(
        Box((0.190, 0.016, 0.130)),
        origin=Origin(xyz=(0.0, 0.002, 0.085)),
        material=shell_black,
        name="rear_bulge",
    )
    housing.visual(
        Box((0.094, 0.008, 0.074)),
        origin=Origin(xyz=(0.0, 0.004, 0.060)),
        material=accent_grey,
        name="mount_plate",
    )
    housing.visual(
        Box((0.130, 0.010, 0.008)),
        origin=Origin(xyz=(0.142, -0.015, -0.074)),
        material=shell_black,
        name="control_tray",
    )
    housing.visual(
        Box((0.527, 0.006, 0.296)),
        origin=Origin(xyz=(0.0, -0.018, 0.090)),
        material=glass_black,
        name="screen_panel",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.022, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=accent_grey,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.010, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=accent_grey,
        name="pivot_web",
    )
    power_rocker.visual(
        Box((0.016, 0.004, 0.0015)),
        origin=Origin(xyz=(0.0, -0.006, -0.0012)),
        material=bezel_black,
        name="rocker_ridge",
    )

    menu_button_positions = (0.092, 0.118, 0.144)
    menu_buttons = []
    for index, button_x in enumerate(menu_button_positions):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.0048, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=accent_grey,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0022, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.001)),
            material=accent_grey,
            name="button_stem",
        )
        menu_buttons.append((button, button_x))

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "stand_to_housing",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=housing,
        origin=Origin(xyz=(0.0, 0.010, 0.258)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.30,
            upper=0.45,
        ),
    )
    model.articulation(
        "housing_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=power_rocker,
        origin=Origin(xyz=(0.176, -0.015, -0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=1.2,
            lower=-0.28,
            upper=0.28,
        ),
    )
    for index, (button, button_x) in enumerate(menu_buttons):
        model.articulation(
            f"housing_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(button_x, -0.015, -0.084)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    housing = object_model.get_part("housing")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    menu_button_2 = object_model.get_part("menu_button_2")
    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("stand_to_housing")
    rocker_joint = object_model.get_articulation("housing_to_power_rocker")
    button_joint_0 = object_model.get_articulation("housing_to_menu_button_0")
    button_joint_1 = object_model.get_articulation("housing_to_menu_button_1")
    button_joint_2 = object_model.get_articulation("housing_to_menu_button_2")
    tilt_limits = tilt.motion_limits

    ctx.expect_gap(
        housing,
        base,
        axis="z",
        min_gap=0.16,
        name="display housing clears the pedestal base",
    )
    ctx.expect_overlap(
        stand,
        base,
        axes="xy",
        min_overlap=0.040,
        name="neck stays centered over the pedestal",
    )
    ctx.expect_overlap(
        housing,
        stand,
        axes="x",
        min_overlap=0.060,
        name="display remains centered on the neck",
    )
    ctx.expect_gap(
        housing,
        power_rocker,
        axis="z",
        positive_elem="lower_bezel",
        negative_elem="rocker_cap",
        min_gap=0.001,
        max_gap=0.006,
        name="power rocker hangs visibly beneath the lower bezel",
    )
    ctx.expect_gap(
        housing,
        menu_button_0,
        axis="z",
        positive_elem="lower_bezel",
        negative_elem="button_cap",
        min_gap=0.001,
        max_gap=0.004,
        name="menu buttons sit just below the bezel",
    )
    ctx.expect_contact(
        housing,
        power_rocker,
        elem_a="lower_bezel",
        elem_b="pivot_web",
        name="power rocker remains mounted to the control edge",
    )
    ctx.expect_contact(
        housing,
        menu_button_0,
        elem_a="lower_bezel",
        elem_b="button_stem",
        name="menu buttons remain mounted through their stems",
    )
    ctx.expect_overlap(
        power_rocker,
        housing,
        axes="x",
        min_overlap=0.010,
        name="power rocker sits within the lower control area",
    )
    ctx.expect_gap(
        menu_button_1,
        menu_button_0,
        axis="x",
        min_gap=0.012,
        name="menu button one stays distinct from menu button zero",
    )
    ctx.expect_gap(
        menu_button_2,
        menu_button_1,
        axis="x",
        min_gap=0.012,
        name="menu button two stays distinct from menu button one",
    )

    rest_screen_aabb = ctx.part_element_world_aabb(housing, elem="screen_panel")
    rest_screen_center = _aabb_center(rest_screen_aabb)
    rest_housing_size = _aabb_size(ctx.part_world_aabb(housing))
    rest_button_0_pos = ctx.part_world_position(menu_button_0)
    rest_button_1_pos = ctx.part_world_position(menu_button_1)
    rest_rocker_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")

    if tilt_limits is not None and tilt_limits.upper is not None and rest_screen_center is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            tilted_screen_center = _aabb_center(ctx.part_element_world_aabb(housing, elem="screen_panel"))
            ctx.check(
                "positive tilt moves the screen backward",
                tilted_screen_center is not None and tilted_screen_center[1] > rest_screen_center[1] + 0.02,
                details=f"rest={rest_screen_center}, tilted={tilted_screen_center}",
            )

    if rest_housing_size is not None:
        with ctx.pose({swivel: math.pi / 2.0}):
            swivel_size = _aabb_size(ctx.part_world_aabb(housing))
            ctx.check(
                "swivel rotates the monitor footprint about vertical",
                swivel_size is not None
                and abs(swivel_size[0] - rest_housing_size[1]) < 0.08
                and abs(swivel_size[1] - rest_housing_size[0]) < 0.08,
                details=f"rest={rest_housing_size}, swivel={swivel_size}",
            )

    if rest_button_0_pos is not None and rest_button_1_pos is not None:
        with ctx.pose({button_joint_1: 0.002}):
            pressed_button_0_pos = ctx.part_world_position(menu_button_0)
            pressed_button_1_pos = ctx.part_world_position(menu_button_1)
            ctx.check(
                "menu buttons depress independently",
                pressed_button_0_pos is not None
                and pressed_button_1_pos is not None
                and abs(pressed_button_0_pos[2] - rest_button_0_pos[2]) < 0.0005
                and pressed_button_1_pos[2] > rest_button_1_pos[2] + 0.0015,
                details=(
                    f"button0_rest={rest_button_0_pos}, button0_pressed={pressed_button_0_pos}, "
                    f"button1_rest={rest_button_1_pos}, button1_pressed={pressed_button_1_pos}"
                ),
            )

    if rest_rocker_aabb is not None:
        rest_rocker_size = _aabb_size(rest_rocker_aabb)
        with ctx.pose({rocker_joint: 0.28}):
            tilted_rocker_size = _aabb_size(ctx.part_element_world_aabb(power_rocker, elem="rocker_cap"))
            ctx.check(
                "power rocker rotates on its local pivot",
                rest_rocker_size is not None
                and tilted_rocker_size is not None
                and tilted_rocker_size[2] > rest_rocker_size[2] + 0.002,
                details=f"rest={rest_rocker_size}, rocked={tilted_rocker_size}",
            )

    with ctx.pose({button_joint_0: 0.002, button_joint_1: 0.0, button_joint_2: 0.0}):
        ctx.expect_gap(
            housing,
            menu_button_0,
            axis="z",
            positive_elem="lower_bezel",
            negative_elem="button_cap",
            max_gap=0.0025,
            max_penetration=0.0,
            name="pressed menu button approaches the bezel without penetrating it",
        )

    return ctx.report()


object_model = build_object_model()
