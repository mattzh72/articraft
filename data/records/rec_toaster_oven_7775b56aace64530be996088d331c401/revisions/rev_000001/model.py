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


BODY_W = 0.58
BODY_D = 0.40
BODY_H = 0.34
SHELL_T = 0.018
FOOT_H = 0.012
FRONT_TRIM_D = 0.020
HALF_PI = 1.5707963267948966

OPEN_LEFT = -0.25
OPEN_RIGHT = 0.10
OPEN_BOTTOM = FOOT_H + 0.065
OPEN_TOP = FOOT_H + 0.263
OPEN_CENTER_Z = 0.5 * (OPEN_BOTTOM + OPEN_TOP)
DOOR_T = 0.018
DOOR_W = 0.173
DOOR_H = 0.192


def _add_door_visuals(part, *, width: float, height: float, thickness: float, sign: float, materials: dict[str, str]) -> None:
    rail_t = 0.020
    stile_t = 0.020
    glass_margin_x = stile_t
    glass_margin_z = rail_t
    latch_x = sign * (width - stile_t * 0.5)
    mid_x = sign * (width * 0.5)
    handle_x = sign * (width - 0.038)
    handle_y = -(thickness * 0.5 + 0.012)

    part.visual(
        Box((stile_t, thickness, height)),
        origin=Origin(xyz=(sign * stile_t * 0.5, 0.0, 0.0)),
        material=materials["trim"],
        name="hinge_stile",
    )
    part.visual(
        Box((stile_t, thickness, height)),
        origin=Origin(xyz=(latch_x, 0.0, 0.0)),
        material=materials["trim"],
        name="latch_stile",
    )
    part.visual(
        Box((width - 2.0 * stile_t, thickness, rail_t)),
        origin=Origin(xyz=(mid_x, 0.0, height * 0.5 - rail_t * 0.5)),
        material=materials["trim"],
        name="top_rail",
    )
    part.visual(
        Box((width - 2.0 * stile_t, thickness, rail_t)),
        origin=Origin(xyz=(mid_x, 0.0, -height * 0.5 + rail_t * 0.5)),
        material=materials["trim"],
        name="bottom_rail",
    )
    part.visual(
        Box((width - 2.0 * glass_margin_x, 0.006, height - 2.0 * glass_margin_z)),
        origin=Origin(xyz=(mid_x, 0.0, 0.0)),
        material=materials["glass"],
        name="glass",
    )
    part.visual(
        Box((0.014, 0.014, height - 0.060)),
        origin=Origin(xyz=(handle_x, handle_y, 0.0)),
        material=materials["handle"],
        name="handle_bar",
    )
    for idx, z in enumerate((-0.040, 0.040)):
        part.visual(
            Box((0.012, 0.030, 0.012)),
            origin=Origin(xyz=(handle_x, -(thickness * 0.5 + 0.007), z)),
            material=materials["handle"],
            name=f"handle_post_{idx}",
        )
    for idx, z in enumerate((-0.060, 0.060)):
        part.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=materials["trim"],
            name=f"hinge_barrel_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_toaster_oven")

    stainless = model.material("stainless", rgba=(0.72, 0.73, 0.75, 1.0))
    trim = model.material("trim", rgba=(0.09, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.22, 0.27, 0.35))
    handle = model.material("handle", rgba=(0.16, 0.16, 0.17, 1.0))
    foot = model.material("foot", rgba=(0.05, 0.05, 0.05, 1.0))
    display = model.material("display", rgba=(0.26, 0.47, 0.70, 0.88))
    control = model.material("control", rgba=(0.16, 0.16, 0.18, 1.0))

    materials = {
        "stainless": stainless.name,
        "trim": trim.name,
        "glass": glass.name,
        "handle": handle.name,
        "foot": foot.name,
        "display": display.name,
        "control": control.name,
    }

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + SHELL_T * 0.5)),
        material=materials["stainless"],
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + BODY_H - SHELL_T * 0.5)),
        material=materials["stainless"],
        name="top_shell",
    )
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W * 0.5 + SHELL_T * 0.5, 0.0, FOOT_H + BODY_H * 0.5)),
        material=materials["stainless"],
        name="left_shell",
    )
    body.visual(
        Box((SHELL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W * 0.5 - SHELL_T * 0.5, 0.0, FOOT_H + BODY_H * 0.5)),
        material=materials["stainless"],
        name="right_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - SHELL_T * 0.5, FOOT_H + BODY_H * 0.5)),
        material=materials["stainless"],
        name="back_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, FRONT_TRIM_D, 0.045)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + FRONT_TRIM_D * 0.5, OPEN_TOP + 0.0225)),
        material=materials["trim"],
        name="front_top_bar",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, FRONT_TRIM_D, OPEN_BOTTOM - FOOT_H - SHELL_T)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_D * 0.5 + FRONT_TRIM_D * 0.5,
                FOOT_H + SHELL_T + (OPEN_BOTTOM - FOOT_H - SHELL_T) * 0.5,
            )
        ),
        material=materials["trim"],
        name="front_bottom_bar",
    )
    body.visual(
        Box((OPEN_LEFT - (-BODY_W * 0.5 + SHELL_T), FRONT_TRIM_D, OPEN_TOP - OPEN_BOTTOM)),
        origin=Origin(
            xyz=(
                (-BODY_W * 0.5 + SHELL_T + OPEN_LEFT) * 0.5,
                -BODY_D * 0.5 + FRONT_TRIM_D * 0.5,
                OPEN_CENTER_Z,
            )
        ),
        material=materials["trim"],
        name="front_left_bar",
    )
    body.visual(
        Box((BODY_W * 0.5 - SHELL_T - OPEN_RIGHT, FRONT_TRIM_D, OPEN_TOP - FOOT_H - SHELL_T)),
        origin=Origin(
            xyz=(
                (OPEN_RIGHT + BODY_W * 0.5 - SHELL_T) * 0.5,
                -BODY_D * 0.5 + FRONT_TRIM_D * 0.5,
                FOOT_H + SHELL_T + (OPEN_TOP - FOOT_H - SHELL_T) * 0.5,
            )
        ),
        material=materials["trim"],
        name="control_panel",
    )
    for idx, (x, y) in enumerate(((-0.22, -0.14), (-0.22, 0.14), (0.22, -0.14), (0.22, 0.14))):
        body.visual(
            Box((0.040, 0.028, FOOT_H)),
            origin=Origin(xyz=(x, y, FOOT_H * 0.5)),
            material=materials["foot"],
            name=f"foot_{idx}",
        )

    left_door = model.part("left_door")
    _add_door_visuals(
        left_door,
        width=DOOR_W,
        height=DOOR_H,
        thickness=DOOR_T,
        sign=1.0,
        materials=materials,
    )

    right_door = model.part("right_door")
    _add_door_visuals(
        right_door,
        width=DOOR_W,
        height=DOOR_H,
        thickness=DOOR_T,
        sign=-1.0,
        materials=materials,
    )

    body.visual(
        Box((0.148, 0.006, 0.070)),
        origin=Origin(xyz=(0.195, -BODY_D * 0.5 - 0.003, FOOT_H + 0.125)),
        material=materials["trim"],
        name="lower_control_bezel",
    )
    body.visual(
        Box((0.050, 0.003, 0.024)),
        origin=Origin(xyz=(0.150, -BODY_D * 0.5 - 0.0075, FOOT_H + 0.147)),
        material=materials["display"],
        name="display_window",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=materials["control"],
        name="dial_collar",
    )
    dial.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=materials["stainless"],
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=materials["control"],
        name="dial_face",
    )
    dial.visual(
        Box((0.006, 0.008, 0.016)),
        origin=Origin(xyz=(0.022, -0.034, 0.010)),
        material=materials["display"],
        name="indicator",
    )

    button_positions = (
        ("button_0", 0.214, FOOT_H + 0.148),
        ("button_1", 0.246, FOOT_H + 0.148),
        ("button_2", 0.214, FOOT_H + 0.120),
        ("button_3", 0.246, FOOT_H + 0.120),
    )
    for button_name, x, z in button_positions:
        button = model.part(button_name)
        button.visual(
            Box((0.020, 0.007, 0.016)),
            origin=Origin(),
            material=materials["control"],
            name="cap",
        )
        model.articulation(
            f"{button_name}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -BODY_D * 0.5 - 0.0075, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.003),
        )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(OPEN_LEFT, -BODY_D * 0.5 - DOOR_T * 0.5, OPEN_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(OPEN_RIGHT, -BODY_D * 0.5 - DOOR_T * 0.5, OPEN_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.190, -BODY_D * 0.5, FOOT_H + 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_3 = object_model.get_part("button_3")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    button_0_slide = object_model.get_articulation("button_0_slide")
    button_1_slide = object_model.get_articulation("button_1_slide")
    button_2_slide = object_model.get_articulation("button_2_slide")
    button_3_slide = object_model.get_articulation("button_3_slide")

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(
            body,
            left_door,
            axis="y",
            positive_elem="front_top_bar",
            max_gap=0.001,
            max_penetration=0.0,
            name="left door sits flush with oven face",
        )
        ctx.expect_gap(
            body,
            right_door,
            axis="y",
            positive_elem="front_top_bar",
            max_gap=0.001,
            max_penetration=0.0,
            name="right door sits flush with oven face",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.002,
            max_gap=0.006,
            name="closed doors keep a narrow center seam",
        )
        left_closed = ctx.part_world_aabb(left_door)
        right_closed = ctx.part_world_aabb(right_door)

    with ctx.pose({left_hinge: 1.35}):
        left_open = ctx.part_world_aabb(left_door)
    with ctx.pose({right_hinge: 1.35}):
        right_open = ctx.part_world_aabb(right_door)

    ctx.check(
        "left door swings outward",
        left_closed is not None
        and left_open is not None
        and left_open[0][1] < left_closed[0][1] - 0.045,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door swings outward",
        right_closed is not None
        and right_open is not None
        and right_open[0][1] < right_closed[0][1] - 0.045,
        details=f"closed={right_closed}, open={right_open}",
    )

    ctx.expect_gap(
        body,
        dial,
        axis="y",
        positive_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial collar stays seated on the control fascia",
    )
    rest_indicator = ctx.part_element_world_aabb(dial, elem="indicator")
    with ctx.pose({dial_spin: 1.2}):
        turned_indicator = ctx.part_element_world_aabb(dial, elem="indicator")
        ctx.expect_gap(
            body,
            dial,
            axis="y",
            positive_elem="control_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="dial remains seated while rotated",
        )
    ctx.check(
        "dial indicator moves when the dial rotates",
        rest_indicator is not None
        and turned_indicator is not None
        and abs(
            0.5 * (turned_indicator[0][0] + turned_indicator[1][0])
            - 0.5 * (rest_indicator[0][0] + rest_indicator[1][0])
        )
        > 0.004,
        details=f"rest={rest_indicator}, turned={turned_indicator}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    rest_button_2 = ctx.part_world_position(button_2)
    rest_button_3 = ctx.part_world_position(button_3)

    with ctx.pose({button_0_slide: 0.003}):
        ctx.expect_gap(
            body,
            button_0,
            axis="y",
            positive_elem="control_panel",
            min_gap=0.0,
            max_gap=0.0015,
            name="button_0 stays proud of the fascia when pressed",
        )
        pressed_button_0 = ctx.part_world_position(button_0)
        held_button_1 = ctx.part_world_position(button_1)

    with ctx.pose({button_3_slide: 0.003}):
        ctx.expect_gap(
            body,
            button_3,
            axis="y",
            positive_elem="control_panel",
            min_gap=0.0,
            max_gap=0.0015,
            name="button_3 stays proud of the fascia when pressed",
        )
        pressed_button_3 = ctx.part_world_position(button_3)
        held_button_2 = ctx.part_world_position(button_2)

    ctx.check(
        "upper button depresses independently",
        rest_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_0 is not None
        and held_button_1 is not None
        and pressed_button_0[1] > rest_button_0[1] + 0.0025
        and abs(held_button_1[1] - rest_button_1[1]) < 1e-6,
        details=(
            f"button_0_rest={rest_button_0}, button_0_pressed={pressed_button_0}, "
            f"button_1_rest={rest_button_1}, button_1_during_press={held_button_1}"
        ),
    )
    ctx.check(
        "lower button depresses independently",
        rest_button_2 is not None
        and rest_button_3 is not None
        and pressed_button_3 is not None
        and held_button_2 is not None
        and pressed_button_3[1] > rest_button_3[1] + 0.0025
        and abs(held_button_2[1] - rest_button_2[1]) < 1e-6,
        details=(
            f"button_3_rest={rest_button_3}, button_3_pressed={pressed_button_3}, "
            f"button_2_rest={rest_button_2}, button_2_during_press={held_button_2}"
        ),
    )
    with ctx.pose({button_0_slide: 0.003, button_1_slide: 0.003, button_2_slide: 0.003, button_3_slide: 0.003}):
        all_pressed = [
            ctx.part_world_position(button_0),
            ctx.part_world_position(button_1),
            ctx.part_world_position(button_2),
            ctx.part_world_position(button_3),
        ]
    ctx.check(
        "button bank stays aligned under full press",
        rest_button_0 is not None
        and rest_button_1 is not None
        and rest_button_2 is not None
        and rest_button_3 is not None
        and all(pos is not None for pos in all_pressed)
        and max(pos[1] for pos in all_pressed) - min(pos[1] for pos in all_pressed) < 1e-6,
        details=f"rest={(rest_button_0, rest_button_1, rest_button_2, rest_button_3)}, pressed={all_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
