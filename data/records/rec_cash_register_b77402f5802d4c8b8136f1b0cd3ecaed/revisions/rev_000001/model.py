from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.360
BASE_WIDTH = 0.330
BASE_HEIGHT = 0.095
SHELL_THICKNESS = 0.0035
DRAWER_OPENING_WIDTH = 0.286
DRAWER_OPENING_HEIGHT = 0.067
DRAWER_CENTER_Z = 0.044
DRAWER_TRAVEL = 0.130

POD_WIDTH = 0.252
PANEL_LOWER = (0.108, 0.112)
PANEL_UPPER = (0.058, 0.173)
PANEL_CENTER_X = (PANEL_LOWER[0] + PANEL_UPPER[0]) * 0.5
PANEL_CENTER_Z = (PANEL_LOWER[1] + PANEL_UPPER[1]) * 0.5
PANEL_TANGENT_X = PANEL_LOWER[0] - PANEL_UPPER[0]
PANEL_TANGENT_Z = PANEL_LOWER[1] - PANEL_UPPER[1]
PANEL_TANGENT_LEN = math.hypot(PANEL_TANGENT_X, PANEL_TANGENT_Z)
PANEL_DOWN_X = PANEL_TANGENT_X / PANEL_TANGENT_LEN
PANEL_DOWN_Z = PANEL_TANGENT_Z / PANEL_TANGENT_LEN
PANEL_NORMAL_X = -PANEL_DOWN_Z
PANEL_NORMAL_Z = PANEL_DOWN_X
PANEL_PITCH = math.atan2(PANEL_NORMAL_X, PANEL_NORMAL_Z)
PANEL_PITCH_DEG = math.degrees(PANEL_PITCH)

PANEL_RECESS_DEPTH = 0.0010
NUMERIC_RECESS_EXTRA = 0.0010
BUTTON_TRAVEL = 0.0026

MENU_KEYS = [
    ("menu_key_0", -0.026, -0.072),
    ("menu_key_1", -0.026, -0.040),
    ("menu_key_2", 0.004, -0.072),
    ("menu_key_3", 0.004, -0.040),
]
NUMERIC_KEYS = [
    ("numeric_key_1", -0.020, 0.031),
    ("numeric_key_2", -0.020, 0.057),
    ("numeric_key_3", -0.020, 0.083),
    ("numeric_key_4", 0.008, 0.031),
    ("numeric_key_5", 0.008, 0.057),
    ("numeric_key_6", 0.008, 0.083),
    ("numeric_key_7", 0.036, 0.031),
    ("numeric_key_8", 0.036, 0.057),
    ("numeric_key_9", 0.036, 0.083),
    ("numeric_key_clr", 0.064, 0.031),
    ("numeric_key_0", 0.064, 0.057),
    ("numeric_key_ent", 0.064, 0.083),
]


def _panel_point(local_x: float, local_y: float, normal_offset: float = 0.0) -> tuple[float, float, float]:
    return (
        PANEL_CENTER_X + local_x * PANEL_DOWN_X + normal_offset * PANEL_NORMAL_X,
        local_y,
        PANEL_CENTER_Z + local_x * PANEL_DOWN_Z + normal_offset * PANEL_NORMAL_Z,
    )


def _panel_origin(local_x: float, local_y: float, normal_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=_panel_point(local_x, local_y, normal_offset),
        rpy=(0.0, PANEL_PITCH, 0.0),
    )


def _rotated_box(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    center_xyz: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), PANEL_PITCH_DEG)
        .translate(center_xyz)
    )


def _build_base_mesh():
    base = cq.Workplane("XY").box(
        BASE_DEPTH,
        BASE_WIDTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    base = base.edges("|Z").fillet(0.010)

    inner_base = (
        cq.Workplane("XY")
        .box(
            BASE_DEPTH - SHELL_THICKNESS * 2.0,
            BASE_WIDTH - SHELL_THICKNESS * 2.0,
            BASE_HEIGHT - SHELL_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SHELL_THICKNESS))
    )
    drawer_opening = (
        cq.Workplane("XY")
        .box(0.060, DRAWER_OPENING_WIDTH, DRAWER_OPENING_HEIGHT, centered=(True, True, True))
        .translate((BASE_DEPTH * 0.5 - 0.014, 0.0, DRAWER_CENTER_Z))
    )
    return base.cut(inner_base).cut(drawer_opening)


def _build_pod_mesh():
    pod_block = (
        cq.Workplane("XY")
        .box(0.245, POD_WIDTH, 0.086, centered=(True, True, False))
        .translate((0.000, 0.0, BASE_HEIGHT - 0.002))
    )
    front_cutter = _rotated_box(
        0.260,
        0.290,
        0.180,
        center_xyz=_panel_point(0.018, 0.000, 0.090),
    )
    rear_cutter = (
        cq.Workplane("XY")
        .box(0.120, POD_WIDTH + 0.020, 0.090, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28.0)
        .translate((-0.150, 0.0, 0.205))
    )
    pod = pod_block.cut(front_cutter).cut(rear_cutter)

    inner_pod_block = (
        cq.Workplane("XY")
        .box(
            0.237,
            POD_WIDTH - SHELL_THICKNESS * 2.0,
            0.078,
            centered=(True, True, False),
        )
        .translate((0.000, 0.0, BASE_HEIGHT + SHELL_THICKNESS))
    )
    inner_front_cutter = _rotated_box(
        0.240,
        POD_WIDTH + 0.020,
        0.160,
        center_xyz=_panel_point(0.018, 0.000, 0.076),
    )
    inner_rear_cutter = (
        cq.Workplane("XY")
        .box(0.112, POD_WIDTH + 0.010, 0.085, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28.0)
        .translate((-0.147, 0.0, 0.198))
    )
    pod = pod.cut(inner_pod_block.cut(inner_front_cutter).cut(inner_rear_cutter))

    main_recess = _rotated_box(
        0.094,
        0.224,
        PANEL_RECESS_DEPTH,
        center_xyz=_panel_point(0.016, 0.000, -PANEL_RECESS_DEPTH * 0.5),
    )
    numeric_recess = _rotated_box(
        0.098,
        0.090,
        PANEL_RECESS_DEPTH + NUMERIC_RECESS_EXTRA,
        center_xyz=_panel_point(
            0.022,
            0.057,
            -(PANEL_RECESS_DEPTH + NUMERIC_RECESS_EXTRA) * 0.5,
        ),
    )
    pod = pod.cut(main_recess).cut(numeric_recess)

    for _, local_x, local_y in MENU_KEYS:
        pod = pod.cut(
            _rotated_box(
                0.010,
                0.021,
                0.018,
                center_xyz=_panel_point(local_x, local_y, -0.0065),
            )
        )

    for _, local_x, local_y in NUMERIC_KEYS:
        pod = pod.cut(
            _rotated_box(
                0.013,
                0.013,
                0.018,
                center_xyz=_panel_point(local_x, local_y, -0.0070),
            )
        )

    return pod


def _add_button_part(
    model: ArticulatedObject,
    *,
    parent: str,
    name: str,
    local_x: float,
    local_y: float,
    cap_size: tuple[float, float, float],
    stem_size: tuple[float, float, float],
    material: str,
    seat_offset: float,
):
    button = model.part(name)
    button.visual(
        Box(stem_size),
        origin=Origin(xyz=(0.0, 0.0, -stem_size[2] * 0.5)),
        material=material,
        name="stem",
    )
    button.visual(
        Box(cap_size),
        origin=Origin(xyz=(0.0, 0.0, cap_size[2] * 0.5)),
        material=material,
        name="cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((max(cap_size[0], stem_size[0]), max(cap_size[1], stem_size[1]), cap_size[2] + stem_size[2])),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )
    model.articulation(
        f"{parent}_to_{name}",
        ArticulationType.PRISMATIC,
        parent=parent,
        child=button,
        origin=_panel_origin(local_x, local_y, seat_offset),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_register")

    shell = model.material("shell", rgba=(0.24, 0.25, 0.27, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.33, 0.34, 0.36, 1.0))
    trim = model.material("trim", rgba=(0.52, 0.53, 0.56, 1.0))
    key_light = model.material("key_light", rgba=(0.86, 0.87, 0.88, 1.0))
    key_dark = model.material("key_dark", rgba=(0.47, 0.49, 0.51, 1.0))
    glass = model.material("glass", rgba=(0.17, 0.29, 0.35, 0.50))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_base_mesh(), "pos_body"),
        material=shell,
        name="shell",
    )
    body.visual(
        Box((0.220, 0.250, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, 0.010)),
        material=shell_dark,
        name="drawer_floor",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(-0.092, 0.0, BASE_HEIGHT + 0.001)),
        material=shell_dark,
        name="boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT + 0.010)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
    )

    pod = model.part("pod")
    pod.visual(
        mesh_from_cadquery(_build_pod_mesh(), "pos_pod"),
        material=shell,
        name="shell",
    )
    pod.inertial = Inertial.from_geometry(
        Box((0.245, POD_WIDTH, 0.086)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
    )
    model.articulation(
        "body_to_pod",
        ArticulationType.FIXED,
        parent=body,
        child=pod,
        origin=Origin(),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.008, 0.280, 0.062)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=drawer_face,
        name="front",
    )
    drawer.visual(
        Box((0.230, 0.272, 0.054)),
        origin=Origin(xyz=(-0.123, 0.0, 0.0)),
        material=drawer_face,
        name="tray",
    )
    drawer.visual(
        Box((0.006, 0.110, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=trim,
        name="pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.238, 0.280, 0.062)),
        mass=1.1,
        origin=Origin(xyz=(-0.119, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BASE_DEPTH * 0.5 - 0.0025, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=shell_dark,
        name="turntable",
    )
    stem.visual(
        Cylinder(radius=0.015, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=shell_dark,
        name="post",
    )
    stem.visual(
        Box((0.040, 0.024, 0.102)),
        origin=Origin(xyz=(-0.012, 0.0, 0.123)),
        material=shell_dark,
        name="neck",
    )
    stem.visual(
        Box((0.024, 0.170, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, 0.176)),
        material=shell_dark,
        name="yoke_bar",
    )
    stem.visual(
        Box((0.028, 0.012, 0.052)),
        origin=Origin(xyz=(-0.026, -0.079, 0.151)),
        material=shell_dark,
        name="ear_0",
    )
    stem.visual(
        Box((0.028, 0.012, 0.052)),
        origin=Origin(xyz=(-0.026, 0.079, 0.151)),
        material=shell_dark,
        name="ear_1",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.055, 0.170, 0.184)),
        mass=0.65,
        origin=Origin(xyz=(-0.010, 0.0, 0.092)),
    )
    model.articulation(
        "body_to_stem",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stem,
        origin=Origin(xyz=(-0.092, 0.0, BASE_HEIGHT + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.15,
            upper=1.15,
        ),
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.028, 0.182, 0.122)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=shell_dark,
        name="housing",
    )
    screen.visual(
        Box((0.006, 0.170, 0.110)),
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        material=trim,
        name="bezel",
    )
    screen.visual(
        Box((0.002, 0.154, 0.094)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=glass,
        name="glass",
    )
    screen.visual(
        Box((0.026, 0.146, 0.014)),
        origin=Origin(xyz=(-0.027, 0.0, 0.0)),
        material=shell_dark,
        name="axle_bar",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.062, 0.182, 0.122)),
        mass=0.72,
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
    )
    model.articulation(
        "stem_to_screen",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=screen,
        origin=Origin(xyz=(-0.026, 0.0, 0.151)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.3,
            lower=-0.35,
            upper=0.55,
        ),
    )

    for name, local_x, local_y in MENU_KEYS:
        _add_button_part(
            model,
            parent="pod",
            name=name,
            local_x=local_x,
            local_y=local_y,
            cap_size=(0.014, 0.026, 0.0042),
            stem_size=(0.010, 0.020, 0.0080),
            material="key_dark",
            seat_offset=0.0060,
        )

    for name, local_x, local_y in NUMERIC_KEYS:
        _add_button_part(
            model,
            parent="pod",
            name=name,
            local_x=local_x,
            local_y=local_y,
            cap_size=(0.018, 0.018, 0.0044),
            stem_size=(0.013, 0.013, 0.0080),
            material="key_light",
            seat_offset=0.0050,
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pod = object_model.get_part("pod")
    drawer = object_model.get_part("drawer")
    screen = object_model.get_part("screen")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    screen_swivel = object_model.get_articulation("body_to_stem")
    screen_tilt = object_model.get_articulation("stem_to_screen")
    menu_key = object_model.get_part("menu_key_0")
    number_key = object_model.get_part("numeric_key_5")
    menu_joint = object_model.get_articulation("pod_to_menu_key_0")
    number_joint = object_model.get_articulation("pod_to_numeric_key_5")

    for key_name, _, _ in MENU_KEYS + NUMERIC_KEYS:
        ctx.allow_overlap(
            object_model.get_part(key_name),
            pod,
            elem_a="stem",
            elem_b="shell",
            reason="Each push button is represented with an internal plunger stem seated through the simplified pod shell opening.",
        )
    ctx.allow_overlap(
        body,
        pod,
        elem_a="boss",
        elem_b="shell",
        reason="The fixed control pod nests over the concealed rear mounting boss on the base shell.",
    )
    ctx.allow_overlap(
        pod,
        object_model.get_part("stem"),
        elem_a="shell",
        elem_b="post",
        reason="The screen post is intentionally represented as passing through the pod's simplified stem socket.",
    )

    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        min_overlap=0.24,
        name="drawer stays centered laterally in the register bay",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="z",
        min_overlap=0.05,
        name="drawer stays seated vertically in the register bay",
    )

    with ctx.pose({drawer_slide: 0.0}):
        body_box = ctx.part_world_aabb(body)
        drawer_front_box = ctx.part_element_world_aabb(drawer, elem="front")
        drawer_front_max_x = drawer_front_box[1][0] if drawer_front_box is not None else None
        body_front_max_x = body_box[1][0] if body_box is not None else None
        ctx.check(
            "drawer closes nearly flush with the shell",
            drawer_front_max_x is not None
            and body_front_max_x is not None
            and abs(drawer_front_max_x - body_front_max_x) <= 0.0035,
            details=f"drawer_front_max_x={drawer_front_max_x}, body_front_max_x={body_front_max_x}",
        )
        rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.24,
            name="drawer remains laterally aligned when extended",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="z",
            min_overlap=0.05,
            name="drawer remains vertically aligned when extended",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            min_overlap=0.095,
            name="drawer retains insertion at full travel",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.10,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    with ctx.pose({screen_swivel: 0.0, screen_tilt: 0.0}):
        rest_glass_center = _aabb_center(ctx.part_element_world_aabb(screen, elem="glass"))

    with ctx.pose({screen_swivel: 0.95, screen_tilt: 0.0}):
        swiveled_glass_center = _aabb_center(ctx.part_element_world_aabb(screen, elem="glass"))

    ctx.check(
        "screen swivels around the rear mount",
        rest_glass_center is not None
        and swiveled_glass_center is not None
        and abs(swiveled_glass_center[1] - rest_glass_center[1]) > 0.030,
        details=f"rest={rest_glass_center}, swiveled={swiveled_glass_center}",
    )

    with ctx.pose({screen_swivel: 0.0, screen_tilt: -0.30}):
        tilted_down_glass_center = _aabb_center(ctx.part_element_world_aabb(screen, elem="glass"))

    with ctx.pose({screen_swivel: 0.0, screen_tilt: 0.50}):
        tilted_up_glass_center = _aabb_center(ctx.part_element_world_aabb(screen, elem="glass"))

    ctx.check(
        "screen tilt changes viewing angle",
        tilted_down_glass_center is not None
        and tilted_up_glass_center is not None
        and tilted_up_glass_center[2] > tilted_down_glass_center[2] + 0.025,
        details=f"down={tilted_down_glass_center}, up={tilted_up_glass_center}",
    )

    menu_rest = ctx.part_world_position(menu_key)
    with ctx.pose({menu_joint: BUTTON_TRAVEL}):
        menu_pressed = ctx.part_world_position(menu_key)
    ctx.check(
        "menu key has inward travel",
        menu_rest is not None
        and menu_pressed is not None
        and math.dist(menu_rest, menu_pressed) > 0.0015,
        details=f"rest={menu_rest}, pressed={menu_pressed}",
    )

    number_rest = ctx.part_world_position(number_key)
    with ctx.pose({number_joint: BUTTON_TRAVEL}):
        number_pressed = ctx.part_world_position(number_key)
    ctx.check(
        "numeric key has inward travel",
        number_rest is not None
        and number_pressed is not None
        and math.dist(number_rest, number_pressed) > 0.0015,
        details=f"rest={number_rest}, pressed={number_pressed}",
    )

    ctx.expect_overlap(
        pod,
        body,
        axes="xy",
        min_overlap=0.12,
        name="control pod remains seated on the drawer base",
    )

    return ctx.report()


object_model = build_object_model()
