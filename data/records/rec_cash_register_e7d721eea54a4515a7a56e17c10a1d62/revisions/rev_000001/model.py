from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DECK_PITCH = math.radians(20.0)


def _deck_mount_origin(local_x: float, local_y: float) -> Origin:
    cos_pitch = math.cos(DECK_PITCH)
    sin_pitch = math.sin(DECK_PITCH)
    deck_center_x = 0.020
    deck_center_z = 0.094
    deck_top_local_z = 0.006
    return Origin(
        xyz=(
            deck_center_x + (local_x * cos_pitch) + (deck_top_local_z * sin_pitch),
            local_y,
            deck_center_z - (local_x * sin_pitch) + (deck_top_local_z * cos_pitch),
        ),
        rpy=(0.0, DECK_PITCH, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_register")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    deck_dark = model.material("deck_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    drawer_dark = model.material("drawer_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.28, 0.29, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.31, 0.36, 0.55))
    menu_key_color = model.material("menu_key_color", rgba=(0.79, 0.80, 0.78, 1.0))
    number_key_color = model.material("number_key_color", rgba=(0.88, 0.89, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.240, 0.240, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=body_dark,
        name="bottom_plate",
    )
    body.visual(
        Box((0.240, 0.014, 0.078)),
        origin=Origin(xyz=(0.000, 0.113, 0.039)),
        material=body_dark,
        name="side_wall_0",
    )
    body.visual(
        Box((0.240, 0.014, 0.078)),
        origin=Origin(xyz=(0.000, -0.113, 0.039)),
        material=body_dark,
        name="side_wall_1",
    )
    body.visual(
        Box((0.014, 0.212, 0.078)),
        origin=Origin(xyz=(-0.113, 0.000, 0.039)),
        material=body_dark,
        name="rear_wall",
    )
    body.visual(
        Box((0.036, 0.212, 0.016)),
        origin=Origin(xyz=(0.102, 0.000, 0.062)),
        material=trim_dark,
        name="front_beam",
    )
    body.visual(
        Box((0.086, 0.212, 0.040)),
        origin=Origin(xyz=(-0.070, 0.000, 0.094)),
        material=body_dark,
        name="rear_riser",
    )
    body.visual(
        Box((0.030, 0.212, 0.020)),
        origin=Origin(xyz=(0.069, 0.000, 0.074)),
        material=trim_dark,
        name="deck_nose",
    )
    body.visual(
        Box((0.148, 0.196, 0.012)),
        origin=Origin(xyz=(0.020, 0.000, 0.094), rpy=(0.000, DECK_PITCH, 0.000)),
        material=deck_dark,
        name="deck_surface",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.206, 0.204, 0.036)),
        origin=Origin(xyz=(-0.103, 0.000, 0.018)),
        material=drawer_dark,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.014, 0.212, 0.040)),
        origin=Origin(xyz=(0.007, 0.000, 0.020)),
        material=drawer_dark,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.010, 0.086, 0.010)),
        origin=Origin(xyz=(0.019, 0.000, 0.022)),
        material=steel,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.006, 0.012, 0.012)),
        origin=Origin(xyz=(0.016, -0.050, 0.022)),
        material=steel,
        name="drawer_lock",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.120, 0.000, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.090,
        ),
    )

    display_stand = model.part("display_stand")
    display_stand.visual(
        Box((0.050, 0.032, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=trim_dark,
        name="stand_base",
    )
    display_stand.visual(
        Box((0.016, 0.026, 0.060)),
        origin=Origin(xyz=(0.000, 0.000, 0.038)),
        material=trim_dark,
        name="stand_stem",
    )
    display_stand.visual(
        Box((0.008, 0.026, 0.010)),
        origin=Origin(xyz=(0.004, 0.000, 0.073)),
        material=trim_dark,
        name="hinge_mount",
    )
    model.articulation(
        "body_to_display_stand",
        ArticulationType.FIXED,
        parent=body,
        child=display_stand,
        origin=Origin(xyz=(-0.080, 0.000, 0.114)),
    )

    rear_display = model.part("rear_display")
    rear_display.visual(
        Box((0.018, 0.102, 0.068)),
        origin=Origin(xyz=(-0.009, 0.000, 0.034)),
        material=trim_dark,
        name="head_shell",
    )
    rear_display.visual(
        Box((0.002, 0.086, 0.052)),
        origin=Origin(xyz=(-0.018, 0.000, 0.034)),
        material=glass,
        name="screen_face",
    )
    rear_display.visual(
        Box((0.010, 0.082, 0.010)),
        origin=Origin(xyz=(-0.005, 0.000, 0.069)),
        material=trim_dark,
        name="top_visor",
    )
    model.articulation(
        "display_stand_to_rear_display",
        ArticulationType.REVOLUTE,
        parent=display_stand,
        child=rear_display,
        origin=Origin(xyz=(0.000, 0.000, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-0.35,
            upper=0.45,
        ),
    )

    def add_deck_key(
        part_name: str,
        joint_name: str,
        *,
        local_x: float,
        local_y: float,
        size: tuple[float, float, float],
        material,
        visual_name: str,
    ) -> None:
        key = model.part(part_name)
        stem_size = (size[0] * 0.40, size[1] * 0.40, 0.002)
        key.visual(
            Box(stem_size),
            origin=Origin(xyz=(0.000, 0.000, stem_size[2] * 0.5)),
            material=trim_dark,
            name=f"{visual_name}_stem",
        )
        key.visual(
            Box(size),
            origin=Origin(xyz=(0.000, 0.000, stem_size[2] + (size[2] * 0.5))),
            material=material,
            name=visual_name,
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=_deck_mount_origin(local_x, local_y),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    menu_rows = (0.018, -0.010)
    menu_cols = (0.024, 0.055, 0.086)
    for row_index, local_x in enumerate(menu_rows):
        for col_index, local_y in enumerate(menu_cols):
            add_deck_key(
                f"menu_key_{row_index}_{col_index}",
                f"body_to_menu_key_{row_index}_{col_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.024, 0.022, 0.006),
                material=menu_key_color,
                visual_name="menu_key",
            )

    number_rows = (0.040, 0.014, -0.012, -0.038)
    number_cols = (-0.064, -0.038, -0.012)
    for row_index, local_x in enumerate(number_rows):
        for col_index, local_y in enumerate(number_cols):
            add_deck_key(
                f"number_key_{row_index}_{col_index}",
                f"body_to_number_key_{row_index}_{col_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.018, 0.018, 0.006),
                material=number_key_color,
                visual_name="number_key",
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    rear_display = object_model.get_part("rear_display")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    display_tilt = object_model.get_articulation("display_stand_to_rear_display")
    menu_key = object_model.get_part("menu_key_0_0")
    menu_key_neighbor = object_model.get_part("menu_key_0_1")
    number_key = object_model.get_part("number_key_0_0")
    number_key_neighbor = object_model.get_part("number_key_0_1")
    menu_key_press = object_model.get_articulation("body_to_menu_key_0_0")
    number_key_press = object_model.get_articulation("body_to_number_key_0_0")

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_gap(
            drawer,
            body,
            axis="x",
            positive_elem="drawer_front",
            negative_elem="front_beam",
            max_gap=0.002,
            max_penetration=0.0,
            name="drawer front sits flush with the register face",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="y",
            inner_elem="drawer_body",
            outer_elem="bottom_plate",
            margin=0.018,
            name="drawer stays centered between the side walls",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.090}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_body",
            elem_b="bottom_plate",
            min_overlap=0.110,
            name="drawer keeps meaningful retained insertion at full extension",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends forward along the service side",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.08,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_head_aabb = ctx.part_element_world_aabb(rear_display, elem="head_shell")
    with ctx.pose({display_tilt: 0.40}):
        tilted_head_aabb = ctx.part_element_world_aabb(rear_display, elem="head_shell")
    ctx.check(
        "rear display head tilts about the stand top hinge",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][0] > rest_head_aabb[1][0] + 0.015,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    rest_menu_pos = ctx.part_world_position(menu_key)
    rest_menu_neighbor_pos = ctx.part_world_position(menu_key_neighbor)
    with ctx.pose({menu_key_press: 0.0025}):
        pressed_menu_pos = ctx.part_world_position(menu_key)
        unchanged_menu_neighbor_pos = ctx.part_world_position(menu_key_neighbor)
    ctx.check(
        "menu key depresses inward on the sloped deck without dragging adjacent keys",
        rest_menu_pos is not None
        and pressed_menu_pos is not None
        and rest_menu_neighbor_pos is not None
        and unchanged_menu_neighbor_pos is not None
        and pressed_menu_pos[0] < rest_menu_pos[0] - 0.0006
        and pressed_menu_pos[2] < rest_menu_pos[2] - 0.0020
        and abs(unchanged_menu_neighbor_pos[0] - rest_menu_neighbor_pos[0]) < 1e-6
        and abs(unchanged_menu_neighbor_pos[1] - rest_menu_neighbor_pos[1]) < 1e-6
        and abs(unchanged_menu_neighbor_pos[2] - rest_menu_neighbor_pos[2]) < 1e-6,
        details=(
            f"rest_menu={rest_menu_pos}, pressed_menu={pressed_menu_pos}, "
            f"rest_neighbor={rest_menu_neighbor_pos}, unchanged_neighbor={unchanged_menu_neighbor_pos}"
        ),
    )

    rest_number_pos = ctx.part_world_position(number_key)
    rest_number_neighbor_pos = ctx.part_world_position(number_key_neighbor)
    with ctx.pose({number_key_press: 0.0025}):
        pressed_number_pos = ctx.part_world_position(number_key)
        unchanged_number_neighbor_pos = ctx.part_world_position(number_key_neighbor)
    ctx.check(
        "number key depresses independently on the numeric pad",
        rest_number_pos is not None
        and pressed_number_pos is not None
        and rest_number_neighbor_pos is not None
        and unchanged_number_neighbor_pos is not None
        and pressed_number_pos[0] < rest_number_pos[0] - 0.0006
        and pressed_number_pos[2] < rest_number_pos[2] - 0.0020
        and abs(unchanged_number_neighbor_pos[0] - rest_number_neighbor_pos[0]) < 1e-6
        and abs(unchanged_number_neighbor_pos[1] - rest_number_neighbor_pos[1]) < 1e-6
        and abs(unchanged_number_neighbor_pos[2] - rest_number_neighbor_pos[2]) < 1e-6,
        details=(
            f"rest_number={rest_number_pos}, pressed_number={pressed_number_pos}, "
            f"rest_neighbor={rest_number_neighbor_pos}, unchanged_neighbor={unchanged_number_neighbor_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
