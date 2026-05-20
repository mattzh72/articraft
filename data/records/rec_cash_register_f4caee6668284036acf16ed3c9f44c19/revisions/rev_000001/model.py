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


BODY_LENGTH = 0.320
BODY_WIDTH = 0.260
BODY_HALF_WIDTH = BODY_WIDTH / 2.0
DRAWER_TRAVEL = 0.095
DECK_PITCH = math.radians(23.5)
DECK_CENTER_X = -0.028
DECK_CENTER_Z = 0.132
DECK_THICKNESS = 0.022


def _deck_surface_z(x: float) -> float:
    return (
        DECK_CENTER_Z
        + (DECK_THICKNESS * 0.5) / math.cos(DECK_PITCH)
        - (x - DECK_CENTER_X) * math.tan(DECK_PITCH)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_register")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    drawer_grey = model.material("drawer_grey", rgba=(0.25, 0.26, 0.28, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    display_dark = model.material("display_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    display_glass = model.material("display_glass", rgba=(0.22, 0.40, 0.46, 0.50))
    metal = model.material("metal", rgba=(0.68, 0.70, 0.72, 1.0))
    key_black = model.material("key_black", rgba=(0.09, 0.09, 0.10, 1.0))
    button_light = model.material("button_light", rgba=(0.78, 0.79, 0.80, 1.0))
    button_green = model.material("button_green", rgba=(0.27, 0.51, 0.29, 1.0))
    button_red = model.material("button_red", rgba=(0.62, 0.20, 0.18, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.320, 0.260, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=body_dark,
        name="base_floor",
    )
    chassis.visual(
        Box((0.302, 0.018, 0.084)),
        origin=Origin(xyz=(0.001, -0.121, 0.049)),
        material=body_dark,
        name="left_side",
    )
    chassis.visual(
        Box((0.302, 0.018, 0.084)),
        origin=Origin(xyz=(0.001, 0.121, 0.049)),
        material=body_dark,
        name="right_side",
    )
    chassis.visual(
        Box((0.018, 0.224, 0.084)),
        origin=Origin(xyz=(-0.151, 0.0, 0.049)),
        material=body_dark,
        name="rear_wall",
    )
    chassis.visual(
        Box((0.024, 0.260, 0.014)),
        origin=Origin(xyz=(0.148, 0.0, 0.085)),
        material=body_dark,
        name="front_lip",
    )
    chassis.visual(
        Box((0.174, 0.260, 0.022)),
        origin=Origin(xyz=(-0.028, 0.0, 0.132), rpy=(0.0, DECK_PITCH, 0.0)),
        material=body_dark,
        name="keypad_deck",
    )
    chassis.visual(
        Box((0.078, 0.260, 0.038)),
        origin=Origin(xyz=(-0.116, 0.0, 0.146)),
        material=body_dark,
        name="rear_cap",
    )
    chassis.visual(
        Box((0.026, 0.022, 0.064)),
        origin=Origin(xyz=(-0.115, 0.0, 0.191)),
        material=body_dark,
        name="stand_post",
    )
    chassis.visual(
        Box((0.020, 0.108, 0.012)),
        origin=Origin(xyz=(-0.115, 0.0, 0.223)),
        material=trim_dark,
        name="stand_cap",
    )
    chassis.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(
            xyz=(-0.018, BODY_HALF_WIDTH + 0.003, 0.132),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="switch_collar",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.214, 0.208, 0.052)),
        material=drawer_grey,
        name="drawer_body",
    )
    for runner_index, runner_y in enumerate((-0.080, 0.080)):
        drawer.visual(
            Box((0.206, 0.014, 0.002)),
            origin=Origin(xyz=(-0.001, runner_y, -0.027)),
            material=trim_dark,
            name=f"runner_{runner_index}",
        )
    drawer.visual(
        Box((0.010, 0.222, 0.064)),
        origin=Origin(xyz=(0.112, 0.0, 0.004)),
        material=drawer_grey,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.110),
        origin=Origin(
            xyz=(0.124, 0.0, 0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_dark,
        name="drawer_handle",
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drawer,
        origin=Origin(xyz=(0.047, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    display_head = model.part("display_head")
    display_head.visual(
        Box((0.020, 0.080, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_dark,
        name="display_mount",
    )
    display_head.visual(
        Box((0.024, 0.126, 0.080)),
        origin=Origin(xyz=(-0.008, 0.0, 0.050)),
        material=display_dark,
        name="display_shell",
    )
    display_head.visual(
        Box((0.002, 0.112, 0.068)),
        origin=Origin(xyz=(-0.020, 0.0, 0.052)),
        material=display_glass,
        name="display_glass",
    )
    model.articulation(
        "display_tilt",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=display_head,
        origin=Origin(xyz=(-0.115, 0.0, 0.229)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.45,
        ),
    )

    button_x_positions = (0.028, -0.004, -0.036)
    button_y_positions = (-0.054, -0.018, 0.018, 0.054)
    for row_index, x_pos in enumerate(button_x_positions):
        for col_index, y_pos in enumerate(button_y_positions):
            if row_index == 0 and col_index == 3:
                button_material = button_green
            elif row_index == 2 and col_index == 3:
                button_material = button_red
            else:
                button_material = button_light

            key_part = model.part(f"key_{row_index}_{col_index}")
            key_part.visual(
                Box((0.024, 0.026, 0.008)),
                origin=Origin(xyz=(0.0, 0.0, 0.004)),
                material=button_material,
                name="key_cap",
            )
            model.articulation(
                f"key_press_{row_index}_{col_index}",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=key_part,
                origin=Origin(
                    xyz=(x_pos, y_pos, _deck_surface_z(x_pos)),
                    rpy=(0.0, DECK_PITCH, 0.0),
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=4.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=0.0025,
                ),
            )

    mode_key = model.part("mode_key")
    mode_key.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="key_shaft",
    )
    mode_key.visual(
        Box((0.016, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.014, 0.004)),
        material=metal,
        name="key_tab",
    )
    mode_key.visual(
        Box((0.030, 0.004, 0.020)),
        origin=Origin(xyz=(0.004, 0.020, 0.015)),
        material=key_black,
        name="key_bow",
    )
    model.articulation(
        "mode_key_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=mode_key,
        origin=Origin(xyz=(-0.018, BODY_HALF_WIDTH + 0.007, 0.132)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    drawer = object_model.get_part("drawer")
    display_head = object_model.get_part("display_head")
    mode_key = object_model.get_part("mode_key")
    sample_key = object_model.get_part("key_1_1")
    shoulder_key = object_model.get_part("key_1_3")
    drawer_slide = object_model.get_articulation("drawer_slide")
    display_tilt = object_model.get_articulation("display_tilt")
    sample_key_press = object_model.get_articulation("key_press_1_1")

    with ctx.pose({drawer_slide: 0.0}):
        closed_drawer_front_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
        closed_chassis_aabb = ctx.part_world_aabb(chassis)
        rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            chassis,
            axes="y",
            min_overlap=0.20,
            name="drawer stays laterally aligned to the cabinet opening",
        )
        ctx.expect_overlap(
            drawer,
            chassis,
            axes="x",
            elem_a="drawer_body",
            min_overlap=0.10,
            name="drawer remains retained in the base at full extension",
        )
        open_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer front sits nearly flush when closed",
        closed_drawer_front_aabb is not None
        and closed_chassis_aabb is not None
        and closed_drawer_front_aabb[1][0] >= closed_chassis_aabb[1][0]
        and closed_drawer_front_aabb[1][0] <= closed_chassis_aabb[1][0] + 0.006,
        details=f"drawer_front={closed_drawer_front_aabb}, chassis_aabb={closed_chassis_aabb}",
    )

    ctx.check(
        "drawer extends forward",
        rest_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[0] > rest_drawer_pos[0] + 0.07,
        details=f"rest={rest_drawer_pos}, open={open_drawer_pos}",
    )

    rest_display_box = ctx.part_element_world_aabb(display_head, elem="display_shell")
    with ctx.pose({display_tilt: 0.45}):
        tilted_display_box = ctx.part_element_world_aabb(display_head, elem="display_shell")
    ctx.check(
        "display head tilts about the stand hinge",
        rest_display_box is not None
        and tilted_display_box is not None
        and tilted_display_box[1][0] > rest_display_box[1][0] + 0.03,
        details=f"rest={rest_display_box}, tilted={tilted_display_box}",
    )

    ctx.expect_gap(
        mode_key,
        shoulder_key,
        axis="y",
        min_gap=0.055,
        name="mode switch stays separate from the button bank",
    )

    rest_key_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_press: 0.0025}):
        pressed_key_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "keypad buttons depress into the angled deck",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - 0.0015,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    return ctx.report()


object_model = build_object_model()
