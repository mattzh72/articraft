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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="restaurant_counter_register")

    body_dark = model.material("body_dark", color=(0.18, 0.19, 0.21))
    body_mid = model.material("body_mid", color=(0.29, 0.31, 0.34))
    key_light = model.material("key_light", color=(0.84, 0.84, 0.82))
    key_dark = model.material("key_dark", color=(0.16, 0.17, 0.18))
    drawer_face = model.material("drawer_face", color=(0.33, 0.34, 0.37))
    metal = model.material("metal", color=(0.70, 0.71, 0.73))
    screen_dark = model.material("screen_dark", color=(0.05, 0.07, 0.08))

    base_depth = 0.42
    base_width = 0.38
    floor_thickness = 0.012
    wall_thickness = 0.012
    body_height = 0.112

    drawer_width = 0.352
    drawer_height = 0.078
    drawer_depth = 0.360
    drawer_front_thickness = 0.018
    drawer_travel = 0.165

    deck_depth = 0.300
    deck_width = 0.350
    deck_height = 0.028
    deck_top_z = body_height + deck_height

    body = model.part("body")
    body.visual(
        Box((base_depth, base_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_dark,
        name="floor_pan",
    )
    body.visual(
        Box((base_depth, wall_thickness, body_height - floor_thickness)),
        origin=Origin(
            xyz=(0.0, base_width / 2.0 - wall_thickness / 2.0, floor_thickness + (body_height - floor_thickness) / 2.0)
        ),
        material=body_dark,
        name="right_wall",
    )
    body.visual(
        Box((base_depth, wall_thickness, body_height - floor_thickness)),
        origin=Origin(
            xyz=(0.0, -base_width / 2.0 + wall_thickness / 2.0, floor_thickness + (body_height - floor_thickness) / 2.0)
        ),
        material=body_dark,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, base_width - 2.0 * wall_thickness, body_height - floor_thickness)),
        origin=Origin(
            xyz=(-base_depth / 2.0 + wall_thickness / 2.0, 0.0, floor_thickness + (body_height - floor_thickness) / 2.0)
        ),
        material=body_dark,
        name="rear_wall",
    )
    body.visual(
        Box((base_depth, base_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - floor_thickness / 2.0)),
        material=body_mid,
        name="top_cover",
    )
    body.visual(
        Box((deck_depth, deck_width, deck_height)),
        origin=Origin(xyz=(0.015, 0.0, body_height + deck_height / 2.0)),
        material=body_mid,
        name="console_deck",
    )
    body.visual(
        Box((0.242, 0.228, 0.004)),
        origin=Origin(xyz=(0.042, -0.020, deck_top_z + 0.002)),
        material=key_dark,
        name="keypad_plate",
    )

    key_pitch_x = 0.044
    key_pitch_y = 0.042
    key_size_x = 0.030
    key_size_y = 0.028
    key_height = 0.012
    key_z = deck_top_z + 0.004 + key_height / 2.0
    start_x = -0.010
    start_y = -0.083
    key_materials = [key_light, key_dark, key_light, key_dark]
    for row in range(4):
        for col in range(4):
            body.visual(
                Box((key_size_x, key_size_y, key_height)),
                origin=Origin(
                    xyz=(
                        start_x + col * key_pitch_x,
                        start_y + row * key_pitch_y,
                        key_z,
                    )
                ),
                material=key_materials[(row + col) % len(key_materials)],
                name=f"key_{row}_{col}",
            )

    function_key_x = 0.110
    function_key_y_positions = (-0.088, -0.046, -0.004, 0.038, 0.080)
    for idx, y_pos in enumerate(function_key_y_positions):
        body.visual(
            Box((0.052, 0.028, key_height)),
            origin=Origin(xyz=(function_key_x, y_pos, key_z)),
            material=key_light if idx < 2 else key_dark,
            name=f"function_key_{idx}",
        )
    body.visual(
        Box((0.090, 0.070, 0.020)),
        origin=Origin(xyz=(-0.104, 0.0, deck_top_z + 0.010)),
        material=body_mid,
        name="post_bridge",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.092),
        origin=Origin(xyz=(-0.104, 0.0, deck_top_z + 0.020 + 0.046)),
        material=body_dark,
        name="display_post",
    )
    body.visual(
        Box((0.056, 0.118, 0.012)),
        origin=Origin(xyz=(-0.104, 0.0, deck_top_z + 0.020 + 0.092 + 0.006)),
        material=body_mid,
        name="post_cap",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(-0.038, 0.130, deck_top_z + 0.005)),
        material=body_mid,
        name="selector_boss",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((drawer_front_thickness, drawer_width, drawer_height)),
        origin=Origin(xyz=(-drawer_front_thickness / 2.0, 0.0, drawer_height / 2.0)),
        material=drawer_face,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_depth - drawer_front_thickness, drawer_width - 0.024, drawer_height - 0.010)),
        origin=Origin(
            xyz=(
                -drawer_front_thickness - (drawer_depth - drawer_front_thickness) / 2.0,
                0.0,
                (drawer_height - 0.010) / 2.0,
            )
        ),
        material=body_mid,
        name="drawer_box",
    )
    drawer.visual(
        Box((0.008, 0.110, 0.020)),
        origin=Origin(xyz=(-drawer_front_thickness + 0.004, 0.0, drawer_height / 2.0)),
        material=metal,
        name="drawer_pull",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(base_depth / 2.0, 0.0, floor_thickness)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=drawer_travel,
        ),
    )

    display = model.part("display")
    display.visual(
        Box((0.036, 0.228, 0.118)),
        origin=Origin(xyz=(-0.018, 0.0, 0.059)),
        material=body_mid,
        name="display_housing",
    )
    display.visual(
        Box((0.004, 0.184, 0.082)),
        origin=Origin(xyz=(-0.034, 0.0, 0.062)),
        material=screen_dark,
        name="screen",
    )
    display.visual(
        Box((0.022, 0.110, 0.016)),
        origin=Origin(xyz=(-0.011, 0.0, 0.008)),
        material=body_dark,
        name="hinge_shoe",
    )

    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(-0.104, 0.0, deck_top_z + 0.020 + 0.092 + 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.25,
            upper=0.55,
        ),
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=metal,
        name="selector_bezel",
    )
    selector.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=metal,
        name="selector_shaft",
    )
    selector.visual(
        Box((0.008, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=metal,
        name="key_stem",
    )
    selector.visual(
        Box((0.034, 0.014, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.043)),
        material=metal,
        name="key_bow",
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(-0.038, 0.130, deck_top_z + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    display = object_model.get_part("display")
    selector = object_model.get_part("selector")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    display_tilt = object_model.get_articulation("body_to_display")
    selector_spin = object_model.get_articulation("body_to_selector")
    limits = drawer_slide.motion_limits

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        inner_elem="drawer_box",
        margin=0.018,
        name="drawer stays centered in the base sleeve",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="drawer_box",
        min_overlap=0.20,
        name="closed drawer remains deeply inserted in the base",
    )

    rest_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    if limits is not None and limits.upper is not None:
        with ctx.pose({drawer_slide: limits.upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_box",
                margin=0.018,
                name="drawer stays guided by the base when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_box",
                min_overlap=0.12,
                name="extended drawer retains insertion",
            )
            extended_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
        moved_forward = (
            rest_aabb is not None
            and extended_aabb is not None
            and extended_aabb[1][0] > rest_aabb[1][0] + 0.12
        )
        ctx.check(
            "drawer extends out the front",
            moved_forward,
            details=f"rest={rest_aabb}, extended={extended_aabb}",
        )

    ctx.expect_gap(
        display,
        body,
        axis="z",
        positive_elem="display_housing",
        negative_elem="console_deck",
        min_gap=0.080,
        name="operator display remains visibly raised above the keypad deck",
    )
    ctx.expect_gap(
        body,
        display,
        axis="x",
        positive_elem="keypad_plate",
        negative_elem="display_housing",
        min_gap=0.018,
        name="display post sits behind the key bank",
    )

    display_limits = display_tilt.motion_limits
    display_rest = ctx.part_element_world_aabb(display, elem="display_housing")
    if display_limits is not None and display_limits.upper is not None:
        with ctx.pose({display_tilt: display_limits.upper}):
            ctx.expect_gap(
                display,
                body,
                axis="z",
                positive_elem="display_housing",
                negative_elem="console_deck",
                min_gap=0.060,
                name="tilted display stays above the keypad deck",
            )
            display_tilted = ctx.part_element_world_aabb(display, elem="display_housing")
        tilts_back = (
            display_rest is not None
            and display_tilted is not None
            and display_tilted[0][0] < display_rest[0][0] - 0.025
        )
        ctx.check(
            "display tilts rearward on its hinge",
            tilts_back,
            details=f"rest={display_rest}, tilted={display_tilted}",
        )

    ctx.expect_gap(
        selector,
        body,
        axis="z",
        positive_elem="selector_bezel",
        negative_elem="selector_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="mode selector sits on its shoulder boss",
    )
    ctx.expect_gap(
        selector,
        body,
        axis="y",
        positive_elem="selector_bezel",
        negative_elem="keypad_plate",
        min_gap=0.020,
        name="mode selector stays separate from the key bank",
    )

    selector_rest = ctx.part_element_world_aabb(selector, elem="key_bow")
    with ctx.pose({selector_spin: math.pi / 2.0}):
        selector_quarter = ctx.part_element_world_aabb(selector, elem="key_bow")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[idx] + hi[idx]) / 2.0 for idx in range(3))

    selector_rest_center = aabb_center(selector_rest)
    selector_quarter_center = aabb_center(selector_quarter)
    selector_rotates = (
        selector_rest_center is not None
        and selector_quarter_center is not None
        and selector_quarter_center[0] < selector_rest_center[0] - 0.010
        and selector_quarter_center[1] > selector_rest_center[1] + 0.010
    )
    ctx.check(
        "mode selector rotates around its shaft",
        selector_rotates,
        details=f"rest={selector_rest_center}, quarter={selector_quarter_center}",
    )

    return ctx.report()


object_model = build_object_model()
