from __future__ import annotations

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


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="restaurant_counter_register")

    housing_dark = model.material("housing_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    housing_mid = model.material("housing_mid", rgba=(0.34, 0.36, 0.39, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.08, 0.09, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.05, 0.07, 1.0))
    menu_key_color = model.material("menu_key_color", rgba=(0.83, 0.73, 0.45, 1.0))
    numeric_key_color = model.material("numeric_key_color", rgba=(0.82, 0.84, 0.86, 1.0))
    key_face_dark = model.material("key_face_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")

    base_len = 0.44
    base_w = 0.34
    base_h = 0.12
    side_t = 0.014
    bottom_t = 0.012
    top_t = 0.014
    back_t = 0.016

    upper_len = 0.18
    upper_w = 0.36
    upper_h = 0.13
    upper_x = -0.13
    upper_center_z = base_h + upper_h / 2.0
    upper_top_z = base_h + upper_h

    deck_len = 0.26
    deck_w = 0.32
    deck_t = 0.016
    deck_x = 0.02
    deck_center_z = upper_top_z + deck_t / 2.0
    deck_top_z = upper_top_z + deck_t

    post_base_h = 0.02
    post_h = 0.09
    post_x = -0.08

    body.visual(
        Box((base_len, side_t, base_h)),
        origin=Origin(xyz=(0.0, -(base_w / 2.0 - side_t / 2.0), base_h / 2.0)),
        material=housing_dark,
        name="base_side_0",
    )
    body.visual(
        Box((base_len, side_t, base_h)),
        origin=Origin(xyz=(0.0, base_w / 2.0 - side_t / 2.0, base_h / 2.0)),
        material=housing_dark,
        name="base_side_1",
    )
    body.visual(
        Box((base_len, base_w - 2.0 * side_t, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=housing_dark,
        name="base_bottom",
    )
    body.visual(
        Box((base_len, base_w - 2.0 * side_t, top_t)),
        origin=Origin(xyz=(0.0, 0.0, base_h - top_t / 2.0)),
        material=housing_dark,
        name="base_top",
    )
    body.visual(
        Box((back_t, base_w - 2.0 * side_t, base_h - bottom_t - top_t)),
        origin=Origin(
            xyz=(
                -(base_len / 2.0 - back_t / 2.0),
                0.0,
                bottom_t + (base_h - bottom_t - top_t) / 2.0,
            )
        ),
        material=housing_dark,
        name="base_back",
    )
    body.visual(
        Box((upper_len, upper_w, upper_h)),
        origin=Origin(xyz=(upper_x, 0.0, upper_center_z)),
        material=housing_mid,
        name="upper_housing",
    )
    body.visual(
        Box((deck_len, deck_w, deck_t)),
        origin=Origin(xyz=(deck_x, 0.0, deck_center_z)),
        material=panel_black,
        name="deck",
    )
    body.visual(
        Box((0.11, 0.026, 0.024)),
        origin=Origin(xyz=(-0.005, -0.11, upper_top_z - 0.012)),
        material=housing_mid,
        name="deck_rib_0",
    )
    body.visual(
        Box((0.11, 0.026, 0.024)),
        origin=Origin(xyz=(-0.005, 0.11, upper_top_z - 0.012)),
        material=housing_mid,
        name="deck_rib_1",
    )
    body.visual(
        Box((0.07, 0.10, post_base_h)),
        origin=Origin(xyz=(post_x, 0.0, deck_top_z + post_base_h / 2.0)),
        material=housing_mid,
        name="post_base",
    )
    body.visual(
        Cylinder(radius=0.024, length=post_h),
        origin=Origin(xyz=(post_x, 0.0, deck_top_z + post_base_h + post_h / 2.0)),
        material=housing_dark,
        name="display_post",
    )

    drawer = model.part("drawer")
    drawer_len = 0.30
    drawer_w = base_w - 2.0 * side_t - 0.010
    drawer_h = base_h - bottom_t - top_t - 0.010
    drawer_front_t = 0.018

    drawer.visual(
        Box((drawer_len, drawer_w, drawer_h)),
        origin=Origin(xyz=(-drawer_len / 2.0, 0.0, 0.0)),
        material=housing_mid,
        name="drawer_bin",
    )
    drawer.visual(
        Box((drawer_front_t, drawer_w + 0.012, drawer_h + 0.012)),
        origin=Origin(xyz=(drawer_front_t / 2.0, 0.0, 0.0)),
        material=housing_mid,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.020, 0.13, 0.018)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=panel_black,
        name="drawer_handle",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(base_len / 2.0, 0.0, bottom_t + 0.005 + drawer_h / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.18),
    )

    display = model.part("display")
    display_t = 0.055
    display_w = 0.22
    display_h = 0.14

    display.visual(
        Box((display_t, display_w, display_h)),
        origin=Origin(xyz=(display_t / 2.0, 0.0, display_h / 2.0)),
        material=panel_black,
        name="screen_housing",
    )
    display.visual(
        Box((0.003, display_w * 0.82, display_h * 0.68)),
        origin=Origin(xyz=(display_t + 0.0015, 0.0, display_h * 0.58)),
        material=screen_black,
        name="screen_glass",
    )

    model.articulation(
        "display_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(post_x, 0.0, deck_top_z + post_base_h + post_h)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.35, upper=0.55),
    )

    def add_key(
        part_name: str,
        joint_name: str,
        *,
        x: float,
        y: float,
        size_x: float,
        size_y: float,
        height: float,
        stroke: float,
        material,
    ) -> None:
        key = model.part(part_name)
        key.visual(
            Box((size_x, size_y, height)),
            origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
            material=material,
            name="cap",
        )
        key.visual(
            Box((size_x * 0.66, size_y * 0.66, 0.0015)),
            origin=Origin(xyz=(0.0, 0.0, height + 0.00075)),
            material=key_face_dark,
            name="legend",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x, y, deck_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=stroke),
        )

    menu_x_positions = (-0.005, 0.060)
    menu_y_positions = (-0.110, -0.050, 0.010)
    menu_index = 0
    for menu_x in menu_x_positions:
        for menu_y in menu_y_positions:
            add_key(
                f"menu_key_{menu_index}",
                f"menu_key_{menu_index}_press",
                x=menu_x,
                y=menu_y,
                size_x=0.050,
                size_y=0.050,
                height=0.016,
                stroke=0.004,
                material=menu_key_color,
            )
            menu_index += 1

    numeric_x_positions = (-0.015, 0.025, 0.065, 0.105)
    numeric_y_positions = (0.070, 0.105, 0.140)
    for row, numeric_x in enumerate(numeric_x_positions):
        for col, numeric_y in enumerate(numeric_y_positions):
            add_key(
                f"numeric_key_{row}_{col}",
                f"numeric_key_{row}_{col}_press",
                x=numeric_x,
                y=numeric_y,
                size_x=0.032,
                size_y=0.032,
                height=0.014,
                stroke=0.003,
                material=numeric_key_color,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    display = object_model.get_part("display")
    menu_key_0 = object_model.get_part("menu_key_0")
    numeric_key_1_1 = object_model.get_part("numeric_key_1_1")

    drawer_slide = object_model.get_articulation("drawer_slide")
    display_tilt = object_model.get_articulation("display_tilt")
    menu_key_0_press = object_model.get_articulation("menu_key_0_press")
    numeric_key_1_1_press = object_model.get_articulation("numeric_key_1_1_press")

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        inner_elem="drawer_bin",
        margin=0.002,
        name="drawer stays centered in the base opening",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="drawer_bin",
        elem_b="base_bottom",
        min_overlap=0.28,
        name="drawer is retained inside the base at rest",
    )

    deck_aabb = ctx.part_element_world_aabb(body, elem="deck")
    upper_aabb = ctx.part_element_world_aabb(body, elem="upper_housing")
    deck_projection_ok = (
        deck_aabb is not None
        and upper_aabb is not None
        and deck_aabb[1][0] > upper_aabb[1][0] + 0.12
    )
    ctx.check(
        "keypad deck projects forward from the upper housing",
        deck_projection_ok,
        details=f"deck={deck_aabb}, upper={upper_aabb}",
    )

    ctx.expect_overlap(
        menu_key_0,
        body,
        axes="xy",
        elem_a="cap",
        elem_b="deck",
        min_overlap=0.030,
        name="menu key sits on the keypad deck",
    )
    ctx.expect_overlap(
        numeric_key_1_1,
        body,
        axes="xy",
        elem_a="cap",
        elem_b="deck",
        min_overlap=0.020,
        name="numeric key sits on the keypad deck",
    )

    drawer_limits = drawer_slide.motion_limits
    rest_drawer_pos = ctx.part_world_position(drawer)
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            extended_drawer_pos = ctx.part_world_position(drawer)
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_bin",
                margin=0.002,
                name="extended drawer stays aligned in the opening",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_bin",
                elem_b="base_bottom",
                min_overlap=0.10,
                name="extended drawer keeps retained insertion",
            )
        ctx.check(
            "drawer slides forward",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.15,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    display_limits = display_tilt.motion_limits
    rest_display_aabb = ctx.part_element_world_aabb(display, elem="screen_housing")
    rest_display_center = _aabb_center(rest_display_aabb)
    if display_limits is not None and display_limits.upper is not None:
        with ctx.pose({display_tilt: display_limits.upper}):
            tilted_display_aabb = ctx.part_element_world_aabb(display, elem="screen_housing")
        tilted_display_center = _aabb_center(tilted_display_aabb)
        ctx.check(
            "display tilts backward on its hinge",
            rest_display_center is not None
            and tilted_display_center is not None
            and tilted_display_center[0] < rest_display_center[0] - 0.01
            and tilted_display_center[2] > rest_display_center[2] - 0.005,
            details=f"rest={rest_display_center}, tilted={tilted_display_center}",
        )

    rest_menu_pos = ctx.part_world_position(menu_key_0)
    rest_numeric_pos = ctx.part_world_position(numeric_key_1_1)

    menu_limits = menu_key_0_press.motion_limits
    if menu_limits is not None and menu_limits.upper is not None:
        with ctx.pose({menu_key_0_press: menu_limits.upper}):
            pressed_menu_pos = ctx.part_world_position(menu_key_0)
            still_numeric_pos = ctx.part_world_position(numeric_key_1_1)
        ctx.check(
            "menu key presses downward independently",
            rest_menu_pos is not None
            and rest_numeric_pos is not None
            and pressed_menu_pos is not None
            and still_numeric_pos is not None
            and pressed_menu_pos[2] < rest_menu_pos[2] - 0.003
            and abs(still_numeric_pos[2] - rest_numeric_pos[2]) < 1e-6,
            details=(
                f"menu_rest={rest_menu_pos}, menu_pressed={pressed_menu_pos}, "
                f"numeric_rest={rest_numeric_pos}, numeric_while_menu={still_numeric_pos}"
            ),
        )

    numeric_limits = numeric_key_1_1_press.motion_limits
    if numeric_limits is not None and numeric_limits.upper is not None:
        with ctx.pose({numeric_key_1_1_press: numeric_limits.upper}):
            pressed_numeric_pos = ctx.part_world_position(numeric_key_1_1)
            still_menu_pos = ctx.part_world_position(menu_key_0)
        ctx.check(
            "numeric key presses downward independently",
            rest_numeric_pos is not None
            and rest_menu_pos is not None
            and pressed_numeric_pos is not None
            and still_menu_pos is not None
            and pressed_numeric_pos[2] < rest_numeric_pos[2] - 0.002
            and abs(still_menu_pos[2] - rest_menu_pos[2]) < 1e-6,
            details=(
                f"numeric_rest={rest_numeric_pos}, numeric_pressed={pressed_numeric_pos}, "
                f"menu_rest={rest_menu_pos}, menu_while_numeric={still_menu_pos}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
