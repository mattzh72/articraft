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


BODY_X = 0.280
BODY_Y = 0.215
BODY_FRONT_X = BODY_X / 2.0

POD_FRONT_POINT = (0.055, 0.0, 0.079)
POD_REAR_POINT = (-0.055, 0.0, 0.105)
POD_PITCH = math.atan2(POD_REAR_POINT[2] - POD_FRONT_POINT[2], POD_FRONT_POINT[0] - POD_REAR_POINT[0])
POD_TANGENT_REAR = (-math.cos(POD_PITCH), 0.0, math.sin(POD_PITCH))
POD_NORMAL = (math.sin(POD_PITCH), 0.0, math.cos(POD_PITCH))


def _add_vec(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale_vec(v: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (v[0] * scale, v[1] * scale, v[2] * scale)


def _pod_xyz(s_rear: float, y: float, lift: float = 0.0) -> tuple[float, float, float]:
    point = _add_vec(POD_FRONT_POINT, _scale_vec(POD_TANGENT_REAR, s_rear))
    point = (point[0], y, point[2])
    if lift != 0.0:
        point = _add_vec(point, _scale_vec(POD_NORMAL, lift))
    return point


def _make_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (BODY_FRONT_X, 0.0),
                (-0.136, 0.0),
                (-0.136, 0.095),
                (-0.090, 0.105),
                (-0.055, 0.105),
                (0.055, 0.079),
                (0.122, 0.064),
                (BODY_FRONT_X, 0.050),
            ]
        )
        .close()
        .extrude(BODY_Y)
        .translate((0.0, BODY_Y / 2.0, 0.0))
        .edges("|Y")
        .fillet(0.006)
    )

    drawer_cut = (
        cq.Workplane("XY")
        .box(0.176, 0.194, 0.044)
        .translate((0.052, 0.0, 0.032))
    )
    receipt_tray_cut = (
        cq.Workplane("XY")
        .box(0.062, 0.070, 0.008)
        .translate((0.094, -0.068, 0.060))
    )
    return body.cut(drawer_cut).cut(receipt_tray_cut)


def _make_keycap_shape(length: float, width: float, height: float, edge: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(edge)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boutique_cash_register")

    body_color = model.material("body_cream", rgba=(0.90, 0.88, 0.82, 1.0))
    trim_dark = model.material("trim_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    drawer_dark = model.material("drawer_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    key_light = model.material("key_light", rgba=(0.95, 0.95, 0.93, 1.0))
    key_accent = model.material("key_accent", rgba=(0.88, 0.72, 0.53, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.17, 0.20, 0.21, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.38, 0.68, 0.70, 0.45))
    hinge_dark = model.material("hinge_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "register_body"),
        material=body_color,
        name="housing_shell",
    )
    body.visual(
        Box((0.088, 0.082, 0.0050)),
        origin=Origin(
            xyz=_pod_xyz(0.050, -0.020, -0.0027),
            rpy=(0.0, POD_PITCH, 0.0),
        ),
        material=trim_dark,
        name="numeric_field",
    )
    for mount_index, y_pos in enumerate((-0.067, 0.028)):
        body.visual(
            Box((0.016, 0.014, 0.012)),
            origin=Origin(
                xyz=_pod_xyz(0.050, y_pos, -0.0060),
                rpy=(0.0, POD_PITCH, 0.0),
            ),
            material=trim_dark,
            name=f"numeric_mount_{mount_index}",
        )
    body.visual(
        Box((0.088, 0.022, 0.0050)),
        origin=Origin(
            xyz=_pod_xyz(0.050, 0.042, -0.0027),
            rpy=(0.0, POD_PITCH, 0.0),
        ),
        material=trim_dark,
        name="function_field",
    )
    body.visual(
        Box((0.016, 0.014, 0.012)),
        origin=Origin(
            xyz=_pod_xyz(0.050, 0.059, -0.0060),
            rpy=(0.0, POD_PITCH, 0.0),
        ),
        material=trim_dark,
        name="function_mount",
    )
    for y_pos in (-0.030, 0.030):
        body.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(
                xyz=(-0.097, y_pos, 0.101),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_dark,
            name=f"display_barrel_{0 if y_pos < 0.0 else 1}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.012, 0.206, 0.048)),
        origin=Origin(xyz=(0.006, 0.0, 0.024)),
        material=drawer_dark,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.168, 0.182, 0.003)),
        origin=Origin(xyz=(-0.084, 0.0, 0.0015)),
        material=drawer_dark,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.164, 0.004, 0.030)),
        origin=Origin(xyz=(-0.082, -0.089, 0.0165)),
        material=drawer_dark,
        name="tray_wall_0",
    )
    drawer.visual(
        Box((0.164, 0.004, 0.030)),
        origin=Origin(xyz=(-0.082, 0.089, 0.0165)),
        material=drawer_dark,
        name="tray_wall_1",
    )
    drawer.visual(
        Box((0.004, 0.182, 0.030)),
        origin=Origin(xyz=(-0.166, 0.0, 0.0165)),
        material=drawer_dark,
        name="tray_back",
    )
    drawer.visual(
        Box((0.008, 0.070, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, 0.028)),
        material=trim_dark,
        name="drawer_pull",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.141, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.078,
        ),
    )

    display = model.part("display")
    display.visual(
        Cylinder(radius=0.0034, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="display_barrel",
    )
    display.visual(
        Box((0.060, 0.074, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, 0.006)),
        material=screen_dark,
        name="display_housing",
    )
    display.visual(
        Box((0.050, 0.058, 0.0016)),
        origin=Origin(xyz=(0.031, 0.0, 0.0128)),
        material=screen_glass,
        name="screen_glass",
    )

    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(-0.101, 0.0, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    num_key_mesh = mesh_from_cadquery(_make_keycap_shape(0.020, 0.020, 0.006, 0.002), "num_keycap")
    fn_key_mesh = mesh_from_cadquery(_make_keycap_shape(0.016, 0.019, 0.0052, 0.0016), "fn_keycap")
    num_row_s = (0.018, 0.040, 0.062, 0.084)
    num_col_y = (-0.047, -0.020, 0.007)
    for row_index, s_pos in enumerate(num_row_s):
        for col_index, y_pos in enumerate(num_col_y):
            key = model.part(f"num_key_{row_index}_{col_index}")
            key.visual(
                num_key_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=key_light,
                name="keycap",
            )
            model.articulation(
                f"body_to_num_key_{row_index}_{col_index}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=Origin(
                    xyz=_pod_xyz(s_pos, y_pos),
                    rpy=(0.0, POD_PITCH, 0.0),
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=2.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=0.0028,
                ),
            )

    fn_row_s = (0.018, 0.040, 0.062, 0.084)
    for row_index, s_pos in enumerate(fn_row_s):
        key = model.part(f"fn_key_{row_index}")
        key.visual(
            fn_key_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=key_accent,
            name="keycap",
        )
        model.articulation(
            f"body_to_fn_key_{row_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(
                xyz=_pod_xyz(s_pos, 0.042),
                rpy=(0.0, POD_PITCH, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0024,
            ),
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
    num_neighbor = object_model.get_part("num_key_1_2")
    fn_key = object_model.get_part("fn_key_1")
    fn_neighbor = object_model.get_part("fn_key_2")
    num_joint = object_model.get_articulation("body_to_num_key_1_1")
    fn_joint = object_model.get_articulation("body_to_fn_key_1")

    drawer_limits = drawer_joint.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_drawer_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            extended_drawer_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(drawer, body, axes="yz", min_overlap=0.04, name="drawer stays aligned in opening")
            ctx.expect_overlap(drawer, body, axes="x", min_overlap=0.03, name="drawer retains insertion at full travel")
        ctx.check(
            "drawer extends forward",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.06,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    display_limits = display_joint.motion_limits
    if display_limits is not None and display_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(display, elem="screen_glass")
        with ctx.pose({display_joint: display_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(display, elem="screen_glass")
        ctx.check(
            "display flips upward from rear edge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.040,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    rest_num_pos = ctx.part_world_position(num_key)
    rest_num_neighbor_pos = ctx.part_world_position(num_neighbor)
    with ctx.pose({num_joint: 0.0028}):
        pressed_num_pos = ctx.part_world_position(num_key)
        num_neighbor_pos = ctx.part_world_position(num_neighbor)
    ctx.check(
        "numeric key depresses along console normal",
        rest_num_pos is not None
        and pressed_num_pos is not None
        and pressed_num_pos[2] < rest_num_pos[2] - 0.002
        and pressed_num_pos[0] < rest_num_pos[0] - 0.0004,
        details=f"rest={rest_num_pos}, pressed={pressed_num_pos}",
    )
    ctx.check(
        "numeric key motion stays independent",
        rest_num_neighbor_pos is not None
        and num_neighbor_pos is not None
        and abs(num_neighbor_pos[0] - rest_num_neighbor_pos[0]) < 1e-8
        and abs(num_neighbor_pos[2] - rest_num_neighbor_pos[2]) < 1e-8,
        details=f"rest={rest_num_neighbor_pos}, other={num_neighbor_pos}",
    )

    rest_fn_pos = ctx.part_world_position(fn_key)
    rest_fn_neighbor_pos = ctx.part_world_position(fn_neighbor)
    with ctx.pose({fn_joint: 0.0024}):
        pressed_fn_pos = ctx.part_world_position(fn_key)
        fn_neighbor_pos = ctx.part_world_position(fn_neighbor)
    ctx.check(
        "function key depresses along its own local axis",
        rest_fn_pos is not None
        and pressed_fn_pos is not None
        and pressed_fn_pos[2] < rest_fn_pos[2] - 0.0015
        and pressed_fn_pos[0] < rest_fn_pos[0] - 0.0003,
        details=f"rest={rest_fn_pos}, pressed={pressed_fn_pos}",
    )
    ctx.check(
        "function key strip keys remain independent",
        rest_fn_neighbor_pos is not None
        and fn_neighbor_pos is not None
        and abs(fn_neighbor_pos[0] - rest_fn_neighbor_pos[0]) < 1e-8
        and abs(fn_neighbor_pos[2] - rest_fn_neighbor_pos[2]) < 1e-8,
        details=f"rest={rest_fn_neighbor_pos}, other={fn_neighbor_pos}",
    )

    numeric_field_aabb = ctx.part_element_world_aabb(body, elem="numeric_field")
    function_field_aabb = ctx.part_element_world_aabb(body, elem="function_field")
    ctx.check(
        "numeric pad and function strip remain distinct zones",
        numeric_field_aabb is not None
        and function_field_aabb is not None
        and function_field_aabb[0][1] - numeric_field_aabb[1][1] > 0.007,
        details=f"numeric={numeric_field_aabb}, function={function_field_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
