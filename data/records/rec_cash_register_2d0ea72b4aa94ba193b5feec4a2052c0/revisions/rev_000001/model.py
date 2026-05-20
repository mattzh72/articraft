from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.335
BASE_D = 0.365
BASE_H = 0.105
BOTTOM_T = 0.012
TOP_T = 0.012
WALL_T = 0.010
FOOT_H = 0.006

DRAWER_W = 0.286
DRAWER_D = 0.315
DRAWER_H = 0.062
DRAWER_FACE_T = 0.016
DRAWER_TRAVEL = 0.155

POD_W = 0.298
POD_D = 0.205
POD_FRONT_H = 0.028
POD_REAR_H = 0.070
POD_GAP = 0.028
POD_BOTTOM_Z = BASE_H + POD_GAP
POD_CENTER_Y = 0.060
POD_SLOPE = math.atan2(POD_REAR_H - POD_FRONT_H, POD_D)

SCREEN_POST_H = 0.110
SCREEN_W = 0.158
SCREEN_H = 0.108
SCREEN_T = 0.018

DEPT_KEYS = (
    ("dept_key_0", -0.078, -0.012),
    ("dept_key_1", -0.032, -0.012),
    ("dept_key_2", -0.078, 0.030),
    ("dept_key_3", -0.032, 0.030),
)

NUMBER_KEYS = (
    ("number_key_0", 0.028, -0.048),
    ("number_key_1", 0.056, -0.048),
    ("number_key_2", 0.084, -0.048),
    ("number_key_3", 0.028, -0.022),
    ("number_key_4", 0.056, -0.022),
    ("number_key_5", 0.084, -0.022),
    ("number_key_6", 0.028, 0.004),
    ("number_key_7", 0.056, 0.004),
    ("number_key_8", 0.084, 0.004),
    ("number_key_9", 0.028, 0.030),
    ("number_key_10", 0.056, 0.030),
    ("number_key_11", 0.084, 0.030),
)


def _pod_top_z(local_y: float) -> float:
    y_from_front = local_y + POD_D * 0.5
    return POD_FRONT_H + ((POD_REAR_H - POD_FRONT_H) * y_from_front / POD_D)


def _pod_mesh() -> object:
    half_d = POD_D * 0.5
    body = (
        cq.Workplane("YZ")
        .moveTo(-half_d, 0.0)
        .lineTo(-half_d, POD_FRONT_H)
        .lineTo(half_d, POD_REAR_H)
        .lineTo(half_d, 0.0)
        .close()
        .extrude(POD_W)
        .translate((-POD_W * 0.5, 0.0, 0.0))
    )
    body = body.edges("|X").fillet(0.004)

    dept_center = (
        math.sin(POD_SLOPE) * 0.005,
        math.cos(POD_SLOPE) * 0.005,
    )
    number_center = (
        math.sin(POD_SLOPE) * 0.00425,
        math.cos(POD_SLOPE) * 0.00425,
    )
    pocket_shift = (
        math.sin(POD_SLOPE) * 0.002,
        -math.cos(POD_SLOPE) * 0.002,
    )
    pocket_angle = -math.degrees(POD_SLOPE)

    for _, x, y in DEPT_KEYS:
        cutter = (
            cq.Workplane("XY")
            .box(0.044, 0.036, 0.016)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), pocket_angle)
            .translate(
                (
                    x,
                    y + dept_center[0] + pocket_shift[0],
                    _pod_top_z(y) + dept_center[1] + pocket_shift[1],
                )
            )
        )
        body = body.cut(cutter)

    for _, x, y in NUMBER_KEYS:
        cutter = (
            cq.Workplane("XY")
            .box(0.026, 0.024, 0.013)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), pocket_angle)
            .translate(
                (
                    x,
                    y + number_center[0] + pocket_shift[0],
                    _pod_top_z(y) + number_center[1] + pocket_shift[1],
                )
            )
        )
        body = body.cut(cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pos_register")

    housing_dark = model.material("housing_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    housing_mid = model.material("housing_mid", rgba=(0.30, 0.31, 0.33, 1.0))
    drawer_dark = model.material("drawer_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    key_light = model.material("key_light", rgba=(0.90, 0.90, 0.86, 1.0))
    key_dept = model.material("key_dept", rgba=(0.78, 0.78, 0.73, 1.0))
    glass = model.material("glass", rgba=(0.19, 0.29, 0.33, 0.45))

    housing = model.part("housing")
    wall_h = BASE_H - FOOT_H - BOTTOM_T - TOP_T
    wall_center_z = FOOT_H + BOTTOM_T + wall_h * 0.5

    housing.visual(
        Box((BASE_W, BASE_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + BOTTOM_T * 0.5)),
        material=housing_dark,
        name="bottom_plate",
    )
    housing.visual(
        Box((BASE_W, BASE_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H - TOP_T * 0.5)),
        material=housing_dark,
        name="top_cover",
    )
    housing.visual(
        Box((WALL_T, BASE_D, wall_h)),
        origin=Origin(xyz=((BASE_W - WALL_T) * 0.5, 0.0, wall_center_z)),
        material=housing_dark,
        name="right_wall",
    )
    housing.visual(
        Box((WALL_T, BASE_D, wall_h)),
        origin=Origin(xyz=(-(BASE_W - WALL_T) * 0.5, 0.0, wall_center_z)),
        material=housing_dark,
        name="left_wall",
    )
    housing.visual(
        Box((BASE_W - 2.0 * WALL_T, WALL_T, wall_h)),
        origin=Origin(xyz=(0.0, (BASE_D - WALL_T) * 0.5, wall_center_z)),
        material=housing_dark,
        name="rear_wall",
    )
    housing.visual(
        Box((BASE_W - 2.0 * WALL_T, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, -BASE_D * 0.5 + 0.013, BASE_H - 0.006 - 0.004)),
        material=housing_mid,
        name="front_header",
    )
    for idx, x in enumerate((-0.118, 0.118)):
        housing.visual(
            Box((0.036, 0.036, FOOT_H)),
            origin=Origin(xyz=(x, -0.125, FOOT_H * 0.5)),
            material=trim_dark,
            name=f"front_foot_{idx}",
        )
        housing.visual(
            Box((0.036, 0.036, FOOT_H)),
            origin=Origin(xyz=(x, 0.125, FOOT_H * 0.5)),
            material=trim_dark,
            name=f"rear_foot_{idx}",
        )

    support_w = 0.024
    support_d = 0.044
    support_h = POD_BOTTOM_Z - BASE_H
    support_z = BASE_H + support_h * 0.5
    for index, x in enumerate((-0.092, 0.092)):
        housing.visual(
            Box((support_w, support_d, support_h)),
            origin=Origin(xyz=(x, 0.112, support_z)),
            material=housing_mid,
            name=f"pod_post_{index}",
        )
    housing.visual(
        Box((0.208, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.124, BASE_H + 0.005)),
        material=housing_mid,
        name="pod_crossbar",
    )
    for index, x in enumerate((-0.1535, 0.1535)):
        housing.visual(
            Box((0.008, 0.278, 0.010)),
            origin=Origin(xyz=(x, -0.012, FOOT_H + BOTTOM_T + 0.028)),
            material=trim_dark,
            name=f"runner_{index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_W - 0.012, DRAWER_D, DRAWER_H)),
        origin=Origin(xyz=(0.0, DRAWER_D * 0.5, DRAWER_H * 0.5)),
        material=drawer_dark,
        name="drawer_body",
    )
    drawer.visual(
        Box((DRAWER_W, DRAWER_FACE_T, DRAWER_H + 0.016)),
        origin=Origin(xyz=(0.0, -DRAWER_FACE_T * 0.5, (DRAWER_H + 0.016) * 0.5)),
        material=housing_mid,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.094),
        origin=Origin(
            xyz=(0.0, -0.013, DRAWER_H * 0.48),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=trim_dark,
        name="drawer_handle",
    )
    drawer.visual(
        Box((0.116, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.016, DRAWER_H * 0.48)),
        material=trim_dark,
        name="handle_bridge",
    )
    for index, x in enumerate((-0.136, 0.136)):
        drawer.visual(
            Box((0.006, DRAWER_D - 0.050, 0.008)),
            origin=Origin(xyz=(x, 0.165, 0.030)),
            material=trim_dark,
            name=f"slide_{index}",
        )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(
            xyz=(
                0.0,
                -BASE_D * 0.5,
                FOOT_H + BOTTOM_T + 0.008,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.28,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    pod = model.part("pod")
    pod.visual(
        mesh_from_cadquery(_pod_mesh(), "pos_register_pod"),
        material=housing_mid,
        name="pod_shell",
    )
    pod.visual(
        Box((POD_W - 0.020, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -POD_D * 0.5 + 0.007, POD_FRONT_H * 0.5 + 0.006)),
        material=trim_dark,
        name="front_skirt",
    )

    key_axis = (0.0, -math.sin(POD_SLOPE), math.cos(POD_SLOPE))
    key_seat = (
        0.0,
        math.sin(POD_SLOPE) * 0.002,
        -math.cos(POD_SLOPE) * 0.002,
    )
    dept_half_t = 0.005
    dept_mount = (0.0, math.sin(POD_SLOPE) * dept_half_t, math.cos(POD_SLOPE) * dept_half_t)
    number_half_t = 0.00425
    number_mount = (
        0.0,
        math.sin(POD_SLOPE) * number_half_t,
        math.cos(POD_SLOPE) * number_half_t,
    )

    for name, x, y in DEPT_KEYS:
        key_part = model.part(name)
        key_part.visual(
            Box((0.040, 0.032, 0.010)),
            origin=Origin(xyz=dept_mount, rpy=(-POD_SLOPE, 0.0, 0.0)),
            material=key_dept,
            name="keycap",
        )
        key_part.visual(
            Box((0.012, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.0005)),
            material=trim_dark,
            name="stem",
        )
        model.articulation(
            f"pod_to_{name}",
            ArticulationType.PRISMATIC,
            parent=pod,
            child=key_part,
            origin=Origin(xyz=(x + key_seat[0], y + key_seat[1], _pod_top_z(y) + key_seat[2])),
            axis=key_axis,
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=-0.0030,
                upper=0.0,
            ),
        )

    for name, x, y in NUMBER_KEYS:
        key_part = model.part(name)
        key_part.visual(
            Box((0.022, 0.020, 0.0085)),
            origin=Origin(xyz=number_mount, rpy=(-POD_SLOPE, 0.0, 0.0)),
            material=key_light,
            name="keycap",
        )
        key_part.visual(
            Box((0.008, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.0005)),
            material=trim_dark,
            name="stem",
        )
        model.articulation(
            f"pod_to_{name}",
            ArticulationType.PRISMATIC,
            parent=pod,
            child=key_part,
            origin=Origin(xyz=(x + key_seat[0], y + key_seat[1], _pod_top_z(y) + key_seat[2])),
            axis=key_axis,
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=-0.0022,
                upper=0.0,
            ),
        )

    model.articulation(
        "housing_to_pod",
        ArticulationType.FIXED,
        parent=housing,
        child=pod,
        origin=Origin(xyz=(0.0, POD_CENTER_Y, POD_BOTTOM_Z)),
    )

    swivel = model.part("screen_swivel")
    swivel.visual(
        Cylinder(radius=0.012, length=SCREEN_POST_H),
        origin=Origin(xyz=(0.0, 0.0, SCREEN_POST_H * 0.5)),
        material=trim_dark,
        name="post",
    )
    swivel.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=trim_dark,
        name="post_base",
    )
    swivel.visual(
        Box((0.064, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, 0.014, SCREEN_POST_H + 0.007)),
        material=trim_dark,
        name="tilt_head",
    )
    swivel.visual(
        Box((0.014, 0.028, 0.030)),
        origin=Origin(xyz=(-0.039, 0.014, SCREEN_POST_H + 0.008)),
        material=trim_dark,
        name="left_ear",
    )
    swivel.visual(
        Box((0.014, 0.028, 0.030)),
        origin=Origin(xyz=(0.039, 0.014, SCREEN_POST_H + 0.008)),
        material=trim_dark,
        name="right_ear",
    )

    model.articulation(
        "pod_to_screen_swivel",
        ArticulationType.REVOLUTE,
        parent=pod,
        child=swivel,
        origin=Origin(xyz=(0.0, POD_D * 0.5 - 0.030, POD_REAR_H - 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.10,
        ),
    )

    screen = model.part("screen")
    screen.visual(
        Cylinder(radius=0.006, length=SCREEN_W - 0.026),
        origin=Origin(
            xyz=(0.0, -0.006, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="hinge_bar",
    )
    screen.visual(
        Box((SCREEN_W, SCREEN_T, SCREEN_H)),
        origin=Origin(xyz=(0.0, -SCREEN_T * 0.5, SCREEN_H * 0.5)),
        material=housing_dark,
        name="screen_body",
    )
    screen.visual(
        Box((SCREEN_W - 0.018, 0.004, SCREEN_H - 0.020)),
        origin=Origin(xyz=(0.0, -SCREEN_T - 0.001, SCREEN_H * 0.5)),
        material=glass,
        name="display",
    )
    screen.visual(
        Box((0.050, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, SCREEN_H + 0.005)),
        material=trim_dark,
        name="top_bezel",
    )

    model.articulation(
        "swivel_to_screen",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=screen,
        origin=Origin(xyz=(0.0, 0.0, SCREEN_POST_H + 0.008)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.6,
            lower=-0.55,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for key_name, _, _ in DEPT_KEYS + NUMBER_KEYS:
        ctx.allow_overlap(
            key_name,
            "pod",
            elem_a="stem",
            elem_b="pod_shell",
            reason="The hidden button plunger stem is intentionally simplified as nesting inside the pod's key pocket.",
        )

    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    screen = object_model.get_part("screen")

    drawer_slide = object_model.get_articulation("housing_to_drawer")
    swivel = object_model.get_articulation("pod_to_screen_swivel")
    tilt = object_model.get_articulation("swivel_to_screen")
    dept_key = object_model.get_part("dept_key_0")
    number_key = object_model.get_part("number_key_10")
    dept_joint = object_model.get_articulation("pod_to_dept_key_0")
    number_joint = object_model.get_articulation("pod_to_number_key_10")

    drawer_limits = drawer_slide.motion_limits
    swivel_limits = swivel.motion_limits
    tilt_limits = tilt.motion_limits
    dept_limits = dept_joint.motion_limits
    number_limits = number_joint.motion_limits

    ctx.expect_overlap(
        drawer,
        housing,
        axes="x",
        min_overlap=0.22,
        name="drawer stays centered across the housing width",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="z",
        min_overlap=0.05,
        name="drawer stays vertically within the housing bay",
    )

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                housing,
                axes="y",
                min_overlap=0.12,
                name="drawer remains retained at full extension",
            )
        ctx.check(
            "drawer extends toward the customer side",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.10,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    if swivel_limits is not None and swivel_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(screen, elem="screen_body")
        with ctx.pose({swivel: swivel_limits.upper}):
            swiveled_aabb = ctx.part_element_world_aabb(screen, elem="screen_body")
        ctx.check(
            "operator screen swings sideways on the swivel",
            rest_aabb is not None
            and swiveled_aabb is not None
            and (swiveled_aabb[1][1] - swiveled_aabb[0][1]) > (rest_aabb[1][1] - rest_aabb[0][1]) + 0.05,
            details=f"rest={rest_aabb}, swiveled={swiveled_aabb}",
        )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        low_aabb = None
        high_aabb = None
        with ctx.pose({tilt: tilt_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(screen, elem="screen_body")
        with ctx.pose({tilt: tilt_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(screen, elem="screen_body")
        ctx.check(
            "operator screen tilts at the head mount",
            low_aabb is not None
            and high_aabb is not None
            and high_aabb[1][1] > low_aabb[1][1] + 0.015,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    missing_key_joints: list[str] = []
    for key_name, _, _ in DEPT_KEYS + NUMBER_KEYS:
        joint_name = f"pod_to_{key_name}"
        try:
            object_model.get_articulation(joint_name)
        except Exception:
            missing_key_joints.append(joint_name)
    ctx.check(
        "all visible keypad buttons are independently articulated",
        not missing_key_joints and len(DEPT_KEYS) + len(NUMBER_KEYS) == 16,
        details=f"missing={missing_key_joints}",
    )

    if dept_limits is not None and dept_limits.lower is not None:
        dept_rest = ctx.part_world_position(dept_key)
        with ctx.pose({dept_joint: dept_limits.lower}):
            dept_pressed = ctx.part_world_position(dept_key)
        ctx.check(
            "department key presses into the sloped pod",
            dept_rest is not None
            and dept_pressed is not None
            and dept_pressed[2] < dept_rest[2] - 0.002
            and dept_pressed[1] > dept_rest[1] + 0.0004,
            details=f"rest={dept_rest}, pressed={dept_pressed}",
        )

    if number_limits is not None and number_limits.lower is not None:
        num_rest = ctx.part_world_position(number_key)
        with ctx.pose({number_joint: number_limits.lower}):
            num_pressed = ctx.part_world_position(number_key)
        ctx.check(
            "number key also presses independently into the sloped pod",
            num_rest is not None
            and num_pressed is not None
            and num_pressed[2] < num_rest[2] - 0.0015
            and num_pressed[1] > num_rest[1] + 0.0003,
            details=f"rest={num_rest}, pressed={num_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
