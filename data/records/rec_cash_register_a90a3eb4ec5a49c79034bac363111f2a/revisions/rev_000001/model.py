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


CONSOLE_PITCH = -0.42
CONSOLE_CENTER = (0.055, 0.0, 0.076)
CONSOLE_SIZE = (0.190, 0.300, 0.056)
POD_MOUNT_ORIGIN = (-0.048, 0.0, 0.179)


def _rotate_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (cos_a * x + sin_a * z, y, -sin_a * x + cos_a * z)


def _console_surface_point(local_x: float, local_y: float, proud: float = 0.0) -> tuple[float, float, float]:
    top_z = CONSOLE_SIZE[2] * 0.5 + proud
    px, py, pz = _rotate_y((local_x, local_y, top_z), CONSOLE_PITCH)
    return (CONSOLE_CENTER[0] + px, CONSOLE_CENTER[1] + py, CONSOLE_CENTER[2] + pz)


def _pod_world_point(local_point: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        POD_MOUNT_ORIGIN[0] + local_point[0],
        POD_MOUNT_ORIGIN[1] + local_point[1],
        POD_MOUNT_ORIGIN[2] + local_point[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_register")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    body_mid = model.material("body_mid", rgba=(0.29, 0.31, 0.34, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    trim_mid = model.material("trim_mid", rgba=(0.39, 0.41, 0.44, 1.0))
    menu_key_light = model.material("menu_key_light", rgba=(0.70, 0.72, 0.74, 1.0))
    number_key_dark = model.material("number_key_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    action_key = model.material("action_key", rgba=(0.58, 0.36, 0.18, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.12, 0.25, 0.28, 0.55))

    housing = model.part("housing")
    housing.visual(
        Box((0.300, 0.340, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=body_dark,
        name="bottom",
    )
    housing.visual(
        Box((0.300, 0.012, 0.094)),
        origin=Origin(xyz=(0.000, -0.164, 0.059)),
        material=body_dark,
        name="left_wall",
    )
    housing.visual(
        Box((0.300, 0.012, 0.094)),
        origin=Origin(xyz=(0.000, 0.164, 0.059)),
        material=body_dark,
        name="right_wall",
    )
    housing.visual(
        Box((0.012, 0.316, 0.094)),
        origin=Origin(xyz=(-0.144, 0.000, 0.059)),
        material=body_dark,
        name="rear_wall",
    )
    housing.visual(
        Box((0.288, 0.340, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.112)),
        material=body_mid,
        name="top_cover",
    )
    housing.visual(
        Box((0.220, 0.010, 0.008)),
        origin=Origin(xyz=(-0.006, -0.153, 0.040)),
        material=trim_dark,
        name="left_runner",
    )
    housing.visual(
        Box((0.220, 0.010, 0.008)),
        origin=Origin(xyz=(-0.006, 0.153, 0.040)),
        material=trim_dark,
        name="right_runner",
    )
    housing.visual(
        Box((0.055, 0.034, 0.055)),
        origin=Origin(xyz=(-0.050, -0.060, 0.1395)),
        material=body_mid,
        name="left_pod_post",
    )
    housing.visual(
        Box((0.055, 0.034, 0.055)),
        origin=Origin(xyz=(-0.050, 0.060, 0.1395)),
        material=body_mid,
        name="right_pod_post",
    )
    housing.visual(
        Box((0.050, 0.155, 0.012)),
        origin=Origin(xyz=(-0.048, 0.000, 0.173)),
        material=body_mid,
        name="pod_bridge",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.246, 0.282, 0.058)),
        origin=Origin(xyz=(-0.123, 0.000, 0.041)),
        material=trim_dark,
        name="tray",
    )
    drawer.visual(
        Box((0.014, 0.316, 0.094)),
        origin=Origin(xyz=(0.007, 0.000, 0.059)),
        material=body_mid,
        name="front",
    )
    drawer.visual(
        Box((0.010, 0.120, 0.014)),
        origin=Origin(xyz=(0.015, 0.000, 0.059)),
        material=trim_mid,
        name="handle_bar",
    )
    drawer.visual(
        Box((0.010, 0.014, 0.020)),
        origin=Origin(xyz=(0.011, -0.040, 0.059)),
        material=trim_mid,
        name="handle_post_0",
    )
    drawer.visual(
        Box((0.010, 0.014, 0.020)),
        origin=Origin(xyz=(0.011, 0.040, 0.059)),
        material=trim_mid,
        name="handle_post_1",
    )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0.136, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.30, lower=0.0, upper=0.115),
    )

    pod = model.part("pod")
    pod.visual(
        Box((0.055, 0.150, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=body_mid,
        name="foot",
    )
    pod.visual(
        Box((0.050, 0.180, 0.060)),
        origin=Origin(xyz=(0.000, 0.000, 0.036)),
        material=body_mid,
        name="riser",
    )
    pod.visual(
        Box(CONSOLE_SIZE),
        origin=Origin(xyz=CONSOLE_CENTER, rpy=(0.0, CONSOLE_PITCH, 0.0)),
        material=body_dark,
        name="console_body",
    )
    pod.visual(
        Box((0.014, 0.282, 0.012)),
        origin=Origin(
            xyz=_console_surface_point(0.098, 0.000, proud=0.006),
            rpy=(0.0, CONSOLE_PITCH, 0.0),
        ),
        material=trim_mid,
        name="front_lip",
    )

    model.articulation(
        "housing_to_pod",
        ArticulationType.FIXED,
        parent=housing,
        child=pod,
        origin=Origin(xyz=POD_MOUNT_ORIGIN),
    )

    console_panel = model.part("console_panel")
    console_panel.visual(
        Box((0.162, 0.272, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=trim_dark,
        name="panel",
    )

    model.articulation(
        "pod_to_console_panel",
        ArticulationType.FIXED,
        parent=pod,
        child=console_panel,
        origin=Origin(
            xyz=_console_surface_point(-0.003, 0.000, proud=0.0),
            rpy=(0.0, CONSOLE_PITCH, 0.0),
        ),
    )

    def _add_console_key(
        part_name: str,
        joint_name: str,
        *,
        local_x: float,
        local_y: float,
        size: tuple[float, float, float],
        material,
        travel: float,
    ) -> None:
        key = model.part(part_name)
        key.visual(
            Box(size),
            origin=Origin(xyz=(0.000, 0.000, size[2] * 0.5)),
            material=material,
            name="cap",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=console_panel,
            child=key,
            origin=Origin(xyz=(local_x, local_y, 0.006)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.10, lower=0.0, upper=travel),
        )

    for row_index, local_x in enumerate((-0.045, -0.013)):
        for col_index, local_y in enumerate((-0.112, -0.077, -0.042)):
            _add_console_key(
                f"menu_key_{row_index}_{col_index}",
                f"console_panel_to_menu_key_{row_index}_{col_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.022, 0.026, 0.008),
                material=menu_key_light,
                travel=0.0025,
            )

    for row_index, local_x in enumerate((-0.005, 0.019, 0.043, 0.067)):
        for col_index, local_y in enumerate((0.050, 0.082, 0.114)):
            number_material = action_key if row_index == 3 and col_index == 2 else number_key_dark
            _add_console_key(
                f"number_key_{row_index}_{col_index}",
                f"console_panel_to_number_key_{row_index}_{col_index}",
                local_x=local_x,
                local_y=local_y,
                size=(0.022, 0.022, 0.008),
                material=number_material,
                travel=0.0028,
            )

    screen_stand = model.part("screen_stand")
    screen_stand.visual(
        Box((0.040, 0.080, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
        material=body_mid,
        name="base",
    )
    screen_stand.visual(
        Cylinder(radius=0.012, length=0.120),
        origin=Origin(xyz=(0.000, 0.000, 0.076)),
        material=trim_dark,
        name="column",
    )

    model.articulation(
        "housing_to_screen_stand",
        ArticulationType.FIXED,
        parent=housing,
        child=screen_stand,
        origin=Origin(xyz=(-0.122, 0.000, 0.118)),
    )

    screen_swivel = model.part("screen_swivel")
    screen_swivel.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=trim_dark,
        name="hub",
    )
    screen_swivel.visual(
        Box((0.034, 0.036, 0.016)),
        origin=Origin(xyz=(0.017, 0.000, 0.017)),
        material=trim_dark,
        name="arm",
    )
    screen_swivel.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(xyz=(0.034, 0.000, 0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mid,
        name="tilt_barrel",
    )

    model.articulation(
        "screen_stand_to_screen_swivel",
        ArticulationType.REVOLUTE,
        parent=screen_stand,
        child=screen_swivel,
        origin=Origin(xyz=(0.000, 0.000, 0.136)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-1.05, upper=1.05),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.024, 0.218, 0.152)),
        origin=Origin(xyz=(-0.012, 0.000, 0.086)),
        material=screen_dark,
        name="frame",
    )
    screen.visual(
        Box((0.003, 0.188, 0.122)),
        origin=Origin(xyz=(-0.022, 0.000, 0.088)),
        material=screen_glass,
        name="glass",
    )
    screen.visual(
        Box((0.004, 0.170, 0.012)),
        origin=Origin(xyz=(-0.022, 0.000, 0.010)),
        material=trim_mid,
        name="lower_bezel",
    )

    model.articulation(
        "screen_swivel_to_screen",
        ArticulationType.REVOLUTE,
        parent=screen_swivel,
        child=screen,
        origin=Origin(xyz=(0.034, 0.000, 0.017)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.30, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    pod = object_model.get_part("pod")
    console_panel = object_model.get_part("console_panel")
    screen_stand = object_model.get_part("screen_stand")
    screen = object_model.get_part("screen")
    menu_key = object_model.get_part("menu_key_0_0")
    menu_key_neighbor = object_model.get_part("menu_key_0_1")
    number_key = object_model.get_part("number_key_1_1")
    number_key_neighbor = object_model.get_part("number_key_1_2")

    drawer_slide = object_model.get_articulation("housing_to_drawer")
    swivel = object_model.get_articulation("screen_stand_to_screen_swivel")
    tilt = object_model.get_articulation("screen_swivel_to_screen")
    menu_key_joint = object_model.get_articulation("console_panel_to_menu_key_0_0")
    number_key_joint = object_model.get_articulation("console_panel_to_number_key_1_1")

    ctx.expect_within(
        drawer,
        housing,
        axes="yz",
        inner_elem="tray",
        outer_elem="bottom",
        margin=0.150,
        name="drawer stays centered in the housing",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="y",
        elem_a="front",
        elem_b="top_cover",
        min_overlap=0.200,
        name="drawer front spans the housing width",
    )
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            housing,
            axes="x",
            elem_a="tray",
            elem_b="bottom",
            min_overlap=0.120,
            name="drawer retains insertion at full extension",
        )
        rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.0}):
        closed_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends forward",
        closed_pos is not None
        and rest_pos is not None
        and rest_pos[0] > closed_pos[0] + 0.08,
        details=f"closed={closed_pos}, extended={rest_pos}",
    )

    ctx.expect_contact(
        pod,
        housing,
        elem_a="foot",
        elem_b="pod_bridge",
        name="pod sits on the bridge support",
    )
    ctx.expect_contact(
        console_panel,
        pod,
        elem_a="panel",
        elem_b="console_body",
        name="console panel sits on the sloped pod face",
    )
    ctx.allow_overlap(
        console_panel,
        pod,
        elem_a="panel",
        elem_b="console_body",
        reason="The control fascia is represented as a thin overlay on the pod shell instead of a separately hollow skin.",
    )

    for row in range(2):
        for col in range(3):
            ctx.allow_overlap(
                console_panel,
                object_model.get_part(f"menu_key_{row}_{col}"),
                elem_a="panel",
                elem_b="cap",
                reason="The menu key caps are represented over a simplified solid panel rather than individual cutouts.",
            )

    for row in range(4):
        for col in range(3):
            ctx.allow_overlap(
                console_panel,
                object_model.get_part(f"number_key_{row}_{col}"),
                elem_a="panel",
                elem_b="cap",
                reason="The numeric key caps are represented over a simplified solid keypad panel rather than individual cutouts.",
            )

    ctx.expect_contact(
        screen_stand,
        housing,
        elem_a="base",
        elem_b="top_cover",
        name="screen stand is mounted to the housing",
    )

    rest_aabb = ctx.part_element_world_aabb(screen, elem="frame")
    with ctx.pose({swivel: swivel.motion_limits.upper}):
        swivel_aabb = ctx.part_element_world_aabb(screen, elem="frame")
    ctx.check(
        "screen swivels around the vertical post",
        rest_aabb is not None
        and swivel_aabb is not None
        and (swivel_aabb[1][0] - swivel_aabb[0][0]) > (rest_aabb[1][0] - rest_aabb[0][0]) + 0.100,
        details=f"rest={rest_aabb}, swivel={swivel_aabb}",
    )

    with ctx.pose({tilt: 0.0}):
        neutral_aabb = ctx.part_element_world_aabb(screen, elem="frame")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_aabb = ctx.part_element_world_aabb(screen, elem="frame")
    ctx.check(
        "screen tilts at the head mount",
        neutral_aabb is not None
        and tilted_aabb is not None
        and ((tilted_aabb[0][0] + tilted_aabb[1][0]) * 0.5)
        < ((neutral_aabb[0][0] + neutral_aabb[1][0]) * 0.5) - 0.015,
        details=f"neutral={neutral_aabb}, tilted={tilted_aabb}",
    )

    all_parts = [part.name for part in object_model.parts]
    menu_part_names = [name for name in all_parts if name.startswith("menu_key_")]
    number_part_names = [name for name in all_parts if name.startswith("number_key_")]
    ctx.check(
        "console has a full menu key bank",
        len(menu_part_names) == 6,
        details=f"menu_keys={menu_part_names}",
    )
    ctx.check(
        "console has a twelve-key numeric pad",
        len(number_part_names) == 12,
        details=f"number_keys={number_part_names}",
    )

    menu_rest = ctx.part_world_position(menu_key)
    menu_neighbor_rest = ctx.part_world_position(menu_key_neighbor)
    with ctx.pose({menu_key_joint: menu_key_joint.motion_limits.upper}):
        menu_pressed = ctx.part_world_position(menu_key)
        menu_neighbor_pressed = ctx.part_world_position(menu_key_neighbor)
    ctx.check(
        "menu key depresses independently along the console normal",
        menu_rest is not None
        and menu_pressed is not None
        and menu_neighbor_rest is not None
        and menu_neighbor_pressed is not None
        and menu_pressed[0] > menu_rest[0] + 0.0008
        and menu_pressed[2] < menu_rest[2] - 0.0015
        and abs(menu_neighbor_pressed[0] - menu_neighbor_rest[0]) < 1e-6
        and abs(menu_neighbor_pressed[2] - menu_neighbor_rest[2]) < 1e-6,
        details=(
            f"menu_rest={menu_rest}, menu_pressed={menu_pressed}, "
            f"neighbor_rest={menu_neighbor_rest}, neighbor_pressed={menu_neighbor_pressed}"
        ),
    )

    number_rest = ctx.part_world_position(number_key)
    number_neighbor_rest = ctx.part_world_position(number_key_neighbor)
    with ctx.pose({number_key_joint: number_key_joint.motion_limits.upper}):
        number_pressed = ctx.part_world_position(number_key)
        number_neighbor_pressed = ctx.part_world_position(number_key_neighbor)
    ctx.check(
        "number key depresses independently along the console normal",
        number_rest is not None
        and number_pressed is not None
        and number_neighbor_rest is not None
        and number_neighbor_pressed is not None
        and number_pressed[0] > number_rest[0] + 0.0010
        and number_pressed[2] < number_rest[2] - 0.0018
        and abs(number_neighbor_pressed[0] - number_neighbor_rest[0]) < 1e-6
        and abs(number_neighbor_pressed[2] - number_neighbor_rest[2]) < 1e-6,
        details=(
            f"number_rest={number_rest}, number_pressed={number_pressed}, "
            f"neighbor_rest={number_neighbor_rest}, neighbor_pressed={number_neighbor_pressed}"
        ),
    )

    number_positions = [
        ctx.part_world_position(object_model.get_part(f"number_key_{row}_{col}"))
        for row in range(4)
        for col in range(3)
    ]
    number_rows = sorted({round(pos[2], 3) for pos in number_positions if pos is not None})
    number_cols = sorted({round(pos[1], 3) for pos in number_positions if pos is not None})
    ctx.check(
        "numeric pad sits in clear rows and columns",
        len(number_rows) == 4 and len(number_cols) == 3,
        details=f"rows={number_rows}, cols={number_cols}",
    )

    return ctx.report()


object_model = build_object_model()
