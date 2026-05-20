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


CONSOLE_PITCH = 0.36
CONSOLE_ANCHOR = (-0.025, 0.095, 0.106)


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x_val, y_val, z_val = point
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        (cos_a * x_val) + (sin_a * z_val),
        y_val,
        (-sin_a * x_val) + (cos_a * z_val),
    )


def _add_vec(
    a_vec: tuple[float, float, float],
    b_vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (a_vec[0] + b_vec[0], a_vec[1] + b_vec[1], a_vec[2] + b_vec[2])


def _console_origin(local_xyz: tuple[float, float, float]) -> Origin:
    return Origin(
        xyz=_add_vec(CONSOLE_ANCHOR, _rotate_y(local_xyz, CONSOLE_PITCH)),
        rpy=(0.0, CONSOLE_PITCH, 0.0),
    )


def _add_console_box(
    part,
    size: tuple[float, float, float],
    *,
    local_xyz: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Box(size),
        origin=_console_origin(local_xyz),
        material=material,
        name=name,
    )


def _add_console_key(
    model: ArticulatedObject,
    body,
    *,
    part_name: str,
    joint_name: str,
    local_xy: tuple[float, float],
    cap_size: tuple[float, float, float],
    cap_material,
    stem_material,
    travel: float,
):
    key = model.part(part_name)
    cap_x, cap_y, cap_z = cap_size
    stem_size = (cap_x * 0.42, cap_y * 0.42, 0.005)
    fin_size = (cap_x * 0.36, 0.004, 0.007)
    fin_center_y = (cap_y * 0.5) - (fin_size[1] * 0.5)
    rib_size = (cap_x * 0.42, 0.004, 0.010)
    rib_center_y = local_xy[1] + (cap_y * 0.5) + (rib_size[1] * 0.5)

    body.visual(
        Box(rib_size),
        origin=_console_origin((local_xy[0], rib_center_y, -0.005)),
        material=stem_material,
        name=f"{part_name}_guide",
    )

    key.visual(
        Box(cap_size),
        origin=Origin(xyz=(0.0, 0.0, cap_z * 0.5)),
        material=cap_material,
        name="cap",
    )
    key.visual(
        Box(stem_size),
        origin=Origin(xyz=(0.0, 0.0, -stem_size[2] * 0.5)),
        material=stem_material,
        name="stem",
    )
    key.visual(
        Box(fin_size),
        origin=Origin(xyz=(0.0, fin_center_y, -fin_size[2] * 0.5)),
        material=stem_material,
        name="guide_fin",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=body,
        child=key,
        origin=_console_origin((local_xy[0], local_xy[1], 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_checkout_register")

    housing = model.material("housing", rgba=(0.29, 0.31, 0.34, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    trim = model.material("trim", rgba=(0.54, 0.57, 0.61, 1.0))
    steel = model.material("steel", rgba=(0.80, 0.82, 0.84, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.24, 0.25, 0.28, 1.0))
    key_light = model.material("key_light", rgba=(0.90, 0.91, 0.92, 1.0))
    key_dark = model.material("key_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    function_key = model.material("function_key", rgba=(0.46, 0.61, 0.76, 1.0))
    glass = model.material("glass", rgba=(0.22, 0.40, 0.43, 0.45))

    body = model.part("body")
    body.visual(
        Box((0.460, 0.340, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=housing,
        name="floor",
    )
    body.visual(
        Box((0.460, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.155, 0.0325)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((0.460, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, 0.155, 0.0325)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((0.060, 0.280, 0.055)),
        origin=Origin(xyz=(-0.200, 0.0, 0.0325)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((0.460, 0.340, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=housing_dark,
        name="top_deck",
    )
    body.visual(
        Box((0.030, 0.038, 0.038)),
        origin=Origin(xyz=(0.205, -0.151, 0.029)),
        material=housing,
        name="drawer_cheek_0",
    )
    body.visual(
        Box((0.030, 0.038, 0.038)),
        origin=Origin(xyz=(0.205, 0.151, 0.029)),
        material=housing,
        name="drawer_cheek_1",
    )
    body.visual(
        Box((0.030, 0.272, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, 0.054)),
        material=housing,
        name="drawer_header",
    )

    body.visual(
        Box((0.210, 0.170, 0.020)),
        origin=Origin(xyz=(-0.015, -0.085, 0.074)),
        material=housing_dark,
        name="scale_base",
    )
    body.visual(
        Box((0.198, 0.158, 0.004)),
        origin=Origin(xyz=(-0.015, -0.085, 0.082)),
        material=trim,
        name="scale_bezel",
    )
    body.visual(
        Box((0.182, 0.142, 0.006)),
        origin=Origin(xyz=(-0.015, -0.085, 0.087)),
        material=steel,
        name="scale_plate",
    )

    body.visual(
        Box((0.120, 0.170, 0.016)),
        origin=Origin(xyz=(0.020, 0.095, 0.072)),
        material=housing,
        name="tower_step",
    )
    body.visual(
        Box((0.110, 0.170, 0.032)),
        origin=Origin(xyz=(-0.070, 0.095, 0.080)),
        material=housing,
        name="tower_block",
    )
    body.visual(
        Box((0.060, 0.170, 0.034)),
        origin=Origin(xyz=(-0.115, 0.095, 0.113)),
        material=housing,
        name="tower_cap",
    )

    _add_console_box(
        body,
        (0.010, 0.164, 0.010),
        local_xyz=(0.071, 0.0, -0.005),
        material=housing_dark,
        name="console_front_rail",
    )
    _add_console_box(
        body,
        (0.010, 0.164, 0.010),
        local_xyz=(-0.071, 0.0, -0.005),
        material=housing_dark,
        name="console_rear_rail",
    )
    _add_console_box(
        body,
        (0.132, 0.024, 0.010),
        local_xyz=(0.0, -0.070, -0.005),
        material=housing_dark,
        name="console_left_field",
    )
    _add_console_box(
        body,
        (0.132, 0.016, 0.010),
        local_xyz=(0.0, -0.016, -0.005),
        material=housing_dark,
        name="console_divider",
    )
    _add_console_box(
        body,
        (0.132, 0.010, 0.010),
        local_xyz=(0.0, 0.077, -0.005),
        material=housing_dark,
        name="console_right_field",
    )

    _add_console_box(
        body,
        (0.010, 0.164, 0.026),
        local_xyz=(0.071, 0.0, -0.023),
        material=housing,
        name="console_front_support",
    )
    _add_console_box(
        body,
        (0.010, 0.164, 0.032),
        local_xyz=(-0.071, 0.0, -0.026),
        material=housing,
        name="console_rear_support",
    )
    _add_console_box(
        body,
        (0.132, 0.024, 0.030),
        local_xyz=(0.0, -0.070, -0.025),
        material=housing,
        name="console_left_support",
    )
    _add_console_box(
        body,
        (0.132, 0.016, 0.030),
        local_xyz=(0.0, -0.016, -0.025),
        material=housing,
        name="console_divider_support",
    )
    _add_console_box(
        body,
        (0.132, 0.010, 0.030),
        local_xyz=(0.0, 0.077, -0.025),
        material=housing,
        name="console_right_support",
    )

    _add_console_box(
        body,
        (0.126, 0.028, 0.003),
        local_xyz=(0.0, -0.041, -0.0095),
        material=trim,
        name="function_floor",
    )
    _add_console_box(
        body,
        (0.126, 0.074, 0.003),
        local_xyz=(0.0, 0.032, -0.0095),
        material=trim,
        name="numpad_floor",
    )

    body.visual(
        Cylinder(radius=0.010, length=0.064),
        origin=Origin(xyz=(-0.200, 0.0, 0.100)),
        material=trim,
        name="display_neck",
    )
    body.visual(
        Box((0.012, 0.082, 0.008)),
        origin=Origin(xyz=(-0.191, 0.0, 0.128)),
        material=trim,
        name="display_head",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.030, 0.272, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=drawer_face,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.355, 0.260, 0.028)),
        origin=Origin(xyz=(-0.1775, 0.0, 0.026)),
        material=housing_dark,
        name="drawer_box",
    )
    drawer.visual(
        Box((0.010, 0.090, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.029)),
        material=trim,
        name="drawer_pull",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=0.140,
        ),
    )

    display = model.part("display")
    display.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    display.visual(
        Box((0.024, 0.145, 0.062)),
        origin=Origin(xyz=(0.012, 0.0, -0.031)),
        material=housing_dark,
        name="display_shell",
    )
    display.visual(
        Box((0.004, 0.112, 0.034)),
        origin=Origin(xyz=(0.023, 0.0, -0.030)),
        material=glass,
        name="glass",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(-0.185, 0.0, 0.138)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.4,
            lower=-0.40,
            upper=0.35,
        ),
    )

    for index, x_pos in enumerate((0.044, 0.022, 0.0, -0.022, -0.044)):
        _add_console_key(
            model,
            body,
            part_name=f"function_key_{index}",
            joint_name=f"body_to_function_key_{index}",
            local_xy=(x_pos, -0.041),
            cap_size=(0.018, 0.019, 0.005),
            cap_material=function_key,
            stem_material=key_dark,
            travel=0.0020,
        )

    for row_index, x_pos in enumerate((0.042, 0.014, -0.014, -0.042)):
        for col_index, y_pos in enumerate((0.006, 0.032, 0.058)):
            _add_console_key(
                model,
                body,
                part_name=f"num_key_{row_index}_{col_index}",
                joint_name=f"body_to_num_key_{row_index}_{col_index}",
                local_xy=(x_pos, y_pos),
                cap_size=(0.020, 0.020, 0.005),
                cap_material=key_light,
                stem_material=key_dark,
                travel=0.0022,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    display = object_model.get_part("display")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    display_joint = object_model.get_articulation("body_to_display")
    num_key = object_model.get_part("num_key_1_1")
    function_key = object_model.get_part("function_key_2")
    num_joint = object_model.get_articulation("body_to_num_key_1_1")
    function_joint = object_model.get_articulation("body_to_function_key_2")

    function_floor = ctx.part_element_world_aabb(body, elem="function_floor")
    numpad_floor = ctx.part_element_world_aabb(body, elem="numpad_floor")
    separated_fields = (
        function_floor is not None
        and numpad_floor is not None
        and (numpad_floor[0][1] - function_floor[1][1]) >= 0.012
    )
    ctx.check(
        "key fields stay visibly separated",
        separated_fields,
        details=f"function_floor={function_floor}, numpad_floor={numpad_floor}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    drawer_upper = drawer_joint.motion_limits.upper if drawer_joint.motion_limits is not None else None
    drawer_extended = None
    if drawer_upper is not None:
        with ctx.pose({drawer_joint: drawer_upper}):
            drawer_extended = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.22,
                name="drawer keeps retained insertion at full extension",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                min_overlap=0.20,
                name="drawer stays guided in the register opening",
            )
    ctx.check(
        "cash drawer slides forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.10,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    display_lower = display_joint.motion_limits.lower if display_joint.motion_limits is not None else None
    display_upper = display_joint.motion_limits.upper if display_joint.motion_limits is not None else None
    glass_low = None
    glass_high = None
    if display_lower is not None:
        with ctx.pose({display_joint: display_lower}):
            glass_low = ctx.part_element_world_aabb(display, elem="glass")
    if display_upper is not None:
        with ctx.pose({display_joint: display_upper}):
            glass_high = ctx.part_element_world_aabb(display, elem="glass")
    ctx.check(
        "rear display tilts on its hinge",
        glass_low is not None
        and glass_high is not None
        and (
            abs(glass_high[0][0] - glass_low[0][0]) > 0.010
            or abs(glass_high[1][2] - glass_low[1][2]) > 0.010
        ),
        details=f"low={glass_low}, high={glass_high}",
    )

    num_rest = ctx.part_world_position(num_key)
    function_rest = ctx.part_world_position(function_key)
    num_pressed = None
    function_static = None
    num_upper = num_joint.motion_limits.upper if num_joint.motion_limits is not None else None
    if num_upper is not None:
        with ctx.pose({num_joint: num_upper}):
            num_pressed = ctx.part_world_position(num_key)
            function_static = ctx.part_world_position(function_key)
    ctx.check(
        "number key depresses independently",
        num_rest is not None
        and function_rest is not None
        and num_pressed is not None
        and function_static is not None
        and num_pressed[2] < num_rest[2] - 0.0015
        and abs(function_static[2] - function_rest[2]) < 0.0002,
        details=(
            f"num_rest={num_rest}, num_pressed={num_pressed}, "
            f"function_rest={function_rest}, function_static={function_static}"
        ),
    )

    function_pressed = None
    num_static = None
    function_upper = function_joint.motion_limits.upper if function_joint.motion_limits is not None else None
    if function_upper is not None:
        with ctx.pose({function_joint: function_upper}):
            function_pressed = ctx.part_world_position(function_key)
            num_static = ctx.part_world_position(num_key)
    ctx.check(
        "function key depresses independently",
        num_rest is not None
        and function_rest is not None
        and function_pressed is not None
        and num_static is not None
        and function_pressed[2] < function_rest[2] - 0.0015
        and abs(num_static[2] - num_rest[2]) < 0.0002,
        details=(
            f"function_rest={function_rest}, function_pressed={function_pressed}, "
            f"num_rest={num_rest}, num_static={num_static}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
