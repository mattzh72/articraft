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


DECK_PITCH = 0.18
DECK_CENTER = (-0.045, 0.0, 0.129)


def _deck_origin(local_x: float, local_y: float, local_z: float) -> Origin:
    cos_pitch = math.cos(DECK_PITCH)
    sin_pitch = math.sin(DECK_PITCH)
    x = DECK_CENTER[0] + cos_pitch * local_x + sin_pitch * local_z
    y = DECK_CENTER[1] + local_y
    z = DECK_CENTER[2] - sin_pitch * local_x + cos_pitch * local_z
    return Origin(xyz=(x, y, z), rpy=(0.0, DECK_PITCH, 0.0))


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="restaurant_counter_register")

    housing = model.material("housing", rgba=(0.80, 0.81, 0.79, 1.0))
    trim = model.material("trim", rgba=(0.24, 0.25, 0.27, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.30, 0.31, 0.33, 1.0))
    key_light = model.material("key_light", rgba=(0.90, 0.89, 0.84, 1.0))
    key_mid = model.material("key_mid", rgba=(0.71, 0.73, 0.76, 1.0))
    key_dark = model.material("key_dark", rgba=(0.33, 0.35, 0.38, 1.0))
    display_frame = model.material("display_frame", rgba=(0.16, 0.17, 0.18, 1.0))
    screen = model.material("screen", rgba=(0.18, 0.42, 0.44, 0.55))

    body = model.part("body")
    body.visual(
        Box((0.428, 0.440, 0.012)),
        origin=Origin(xyz=(-0.015, 0.0, 0.006)),
        material=housing,
        name="bottom_pan",
    )
    body.visual(
        Box((0.428, 0.018, 0.110)),
        origin=Origin(xyz=(-0.015, -0.211, 0.055)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((0.428, 0.018, 0.110)),
        origin=Origin(xyz=(-0.015, 0.211, 0.055)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((0.020, 0.440, 0.110)),
        origin=Origin(xyz=(-0.225, 0.0, 0.055)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((0.405, 0.404, 0.012)),
        origin=Origin(xyz=(-0.025, 0.0, 0.094)),
        material=housing,
        name="top_cover",
    )
    body.visual(
        Box((0.030, 0.404, 0.018)),
        origin=Origin(xyz=(0.190, 0.0, 0.101)),
        material=trim,
        name="front_bezel",
    )
    body.visual(
        Box((0.180, 0.404, 0.050)),
        origin=Origin(xyz=(-0.135, 0.0, 0.119)),
        material=housing,
        name="rear_housing",
    )
    body.visual(
        Box((0.060, 0.384, 0.020)),
        origin=Origin(xyz=(0.065, 0.0, 0.108)),
        material=housing,
        name="deck_nose",
    )
    body.visual(
        Box((0.285, 0.400, 0.016)),
        origin=_deck_origin(0.0, 0.0, 0.0),
        material=trim,
        name="control_deck",
    )
    body.visual(
        Box((0.230, 0.212, 0.004)),
        origin=_deck_origin(-0.008, -0.050, 0.010),
        material=key_dark,
        name="main_keypad_seat",
    )
    body.visual(
        Box((0.222, 0.012, 0.010)),
        origin=_deck_origin(-0.008, 0.147, 0.009),
        material=key_dark,
        name="preset_bank",
    )
    body.visual(
        Box((0.222, 0.012, 0.010)),
        origin=_deck_origin(-0.008, 0.119, 0.009),
        material=key_dark,
        name="preset_bank_inner",
    )

    keypad_x_positions = (0.070, 0.025, -0.020, -0.065)
    keypad_y_positions = (-0.120, -0.070, -0.020, 0.030)
    for row_index, local_x in enumerate(keypad_x_positions):
        for column_index, local_y in enumerate(keypad_y_positions):
            body.visual(
                Box((0.034, 0.032, 0.010)),
                origin=_deck_origin(local_x, local_y, 0.017),
                material=key_mid if (row_index + column_index) % 3 else key_light,
                name=f"deck_key_{row_index}_{column_index}",
            )

    body.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(-0.145, 0.0, 0.118)),
        material=trim,
        name="post_collar",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.069),
        origin=Origin(xyz=(-0.145, 0.0, 0.1465)),
        material=trim,
        name="post_shaft",
    )
    body.visual(
        Box((0.016, 0.060, 0.008)),
        origin=Origin(xyz=(-0.145, 0.0, 0.177)),
        material=trim,
        name="post_head",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.016, 0.392, 0.076)),
        origin=Origin(xyz=(-0.008, 0.0, 0.038)),
        material=drawer_finish,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.394, 0.372, 0.004)),
        origin=Origin(xyz=(-0.203, 0.0, 0.004)),
        material=drawer_finish,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.386, 0.008, 0.048)),
        origin=Origin(xyz=(-0.208, -0.186, 0.024)),
        material=drawer_finish,
        name="left_tray_wall",
    )
    drawer.visual(
        Box((0.386, 0.008, 0.048)),
        origin=Origin(xyz=(-0.208, 0.186, 0.024)),
        material=drawer_finish,
        name="right_tray_wall",
    )
    drawer.visual(
        Box((0.008, 0.372, 0.048)),
        origin=Origin(xyz=(-0.396, 0.0, 0.024)),
        material=drawer_finish,
        name="rear_tray_wall",
    )
    drawer.visual(
        Box((0.020, 0.180, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.041)),
        material=trim,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.008, 0.110, 0.032)),
        origin=Origin(xyz=(-0.150, 0.000, 0.018)),
        material=key_mid,
        name="bill_divider",
    )
    drawer.visual(
        Box((0.120, 0.008, 0.022)),
        origin=Origin(xyz=(-0.245, -0.060, 0.013)),
        material=key_mid,
        name="coin_divider_0",
    )
    drawer.visual(
        Box((0.120, 0.008, 0.022)),
        origin=Origin(xyz=(-0.245, 0.060, 0.013)),
        material=key_mid,
        name="coin_divider_1",
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.205, 0.0, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.165,
        ),
    )

    display_part = model.part("display")
    display_part.visual(
        Cylinder(radius=0.007, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    display_part.visual(
        Box((0.014, 0.110, 0.032)),
        origin=Origin(xyz=(-0.008, 0.0, 0.018), rpy=(0.0, -0.22, 0.0)),
        material=display_frame,
        name="hinge_mount",
    )
    display_part.visual(
        Box((0.028, 0.250, 0.132)),
        origin=Origin(xyz=(-0.014, 0.0, 0.069), rpy=(0.0, -0.22, 0.0)),
        material=display_frame,
        name="display_housing",
    )
    display_part.visual(
        Box((0.040, 0.248, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.129), rpy=(0.0, -0.22, 0.0)),
        material=display_frame,
        name="display_top",
    )
    display_part.visual(
        Box((0.003, 0.216, 0.106)),
        origin=Origin(xyz=(-0.028, 0.0, 0.072), rpy=(0.0, -0.22, 0.0)),
        material=screen,
        name="screen_glass",
    )
    model.articulation(
        "display_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display_part,
        origin=Origin(xyz=(-0.145, 0.0, 0.188)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.28,
            upper=0.22,
        ),
    )

    preset_key_positions = (0.082, 0.044, 0.006, -0.032, -0.070, -0.108)
    for index, local_x in enumerate(preset_key_positions):
        preset_key = model.part(f"preset_key_{index}")
        preset_key.visual(
            Box((0.030, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=key_light,
            name="key_cap",
        )
        preset_key.visual(
            Box((0.024, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=key_mid,
            name="key_top",
        )
        model.articulation(
            f"preset_key_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=preset_key,
            origin=_deck_origin(local_x, 0.133, 0.014),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.5,
                velocity=0.10,
                lower=0.0,
                upper=0.006,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    display = object_model.get_part("display")
    drawer_slide = object_model.get_articulation("drawer_slide")
    display_tilt = object_model.get_articulation("display_tilt")
    preset_key_0 = object_model.get_part("preset_key_0")
    preset_key_1 = object_model.get_part("preset_key_1")
    preset_key_0_press = object_model.get_articulation("preset_key_0_press")

    body_aabb = ctx.part_world_aabb(body)
    drawer_face_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_face")
    drawer_tray_aabb = ctx.part_element_world_aabb(drawer, elem="tray_floor")
    drawer_front_flush = (
        body_aabb is not None
        and drawer_face_aabb is not None
        and abs(drawer_face_aabb[1][0] - body_aabb[1][0]) <= 0.003
    )
    ctx.check(
        "drawer front sits flush at rest",
        drawer_front_flush,
        details=f"body={body_aabb}, drawer_face={drawer_face_aabb}",
    )
    ctx.check(
        "drawer remains nested at rest",
        drawer_tray_aabb is not None and drawer_tray_aabb[0][0] < 0.0,
        details=f"tray_aabb={drawer_tray_aabb}",
    )

    rest_drawer_origin = ctx.part_world_position(drawer)
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if drawer_upper is not None:
        with ctx.pose({drawer_slide: drawer_upper}):
            extended_drawer_origin = ctx.part_world_position(drawer)
            extended_tray_aabb = ctx.part_element_world_aabb(drawer, elem="tray_floor")
        ctx.check(
            "drawer slides outward and stays retained",
            rest_drawer_origin is not None
            and extended_drawer_origin is not None
            and extended_drawer_origin[0] > rest_drawer_origin[0] + 0.12
            and extended_tray_aabb is not None
            and extended_tray_aabb[0][0] < 0.0,
            details=(
                f"rest_origin={rest_drawer_origin}, "
                f"extended_origin={extended_drawer_origin}, "
                f"extended_tray={extended_tray_aabb}"
            ),
        )

    tilt_lower = display_tilt.motion_limits.lower if display_tilt.motion_limits is not None else None
    tilt_upper = display_tilt.motion_limits.upper if display_tilt.motion_limits is not None else None
    if tilt_lower is not None and tilt_upper is not None:
        with ctx.pose({display_tilt: tilt_lower}):
            lower_center = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_glass"))
        with ctx.pose({display_tilt: tilt_upper}):
            upper_center = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_glass"))
        ctx.check(
            "display tilts forward through its range",
            lower_center is not None
            and upper_center is not None
            and upper_center[0] > lower_center[0] + 0.015
            and upper_center[2] > lower_center[2] - 0.020,
            details=f"lower_center={lower_center}, upper_center={upper_center}",
        )

    key_0_rest = ctx.part_world_position(preset_key_0)
    key_1_rest = ctx.part_world_position(preset_key_1)
    key_upper = preset_key_0_press.motion_limits.upper if preset_key_0_press.motion_limits is not None else None
    if key_upper is not None:
        with ctx.pose({preset_key_0_press: key_upper}):
            key_0_pressed = ctx.part_world_position(preset_key_0)
            key_1_steady = ctx.part_world_position(preset_key_1)
        ctx.check(
            "preset key depresses independently",
            key_0_rest is not None
            and key_1_rest is not None
            and key_0_pressed is not None
            and key_1_steady is not None
            and key_0_pressed[2] < key_0_rest[2] - 0.004
            and abs(key_1_steady[2] - key_1_rest[2]) <= 1e-6,
            details=(
                f"key_0_rest={key_0_rest}, key_0_pressed={key_0_pressed}, "
                f"key_1_rest={key_1_rest}, key_1_steady={key_1_steady}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
