from __future__ import annotations

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


BASE_DEPTH = 0.245
BASE_WIDTH = 0.280
BASE_HEIGHT = 0.087

DRAWER_TRAVEL = 0.112
DRAWER_JOINT_X = -0.093

POD_OFFSET_X = -0.010
POD_OFFSET_Z = BASE_HEIGHT
POD_REAR_X = -0.070
POD_TOP_Z = 0.072


def _make_pod_shell() -> object:
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (POD_REAR_X, 0.000),
                (0.060, 0.000),
                (0.060, 0.026),
                (0.030, 0.050),
                (-0.008, POD_TOP_Z),
                (POD_REAR_X, POD_TOP_Z),
            ]
        )
        .close()
    )
    pod = profile.extrude(0.104, both=True)
    pod = pod.edges("|Y").fillet(0.010)
    printer_bay = (
        cq.Workplane("XY")
        .box(0.084, 0.148, 0.024, centered=(True, True, False))
        .translate((-0.018, 0.000, 0.054))
    )
    return pod.cut(printer_bay)


def _make_keypad_module() -> object:
    keypad = cq.Workplane("XY").box(0.124, 0.108, 0.003, centered=(True, True, False))
    button_pitch_x = 0.028
    button_pitch_y = 0.024
    x_positions = (-0.042, -0.014, 0.014, 0.044)
    y_positions = (0.032, 0.008, -0.016, -0.040)
    for row, y_pos in enumerate(y_positions):
        for col, x_pos in enumerate(x_positions):
            width = 0.021 if col < 3 else 0.024
            depth = 0.019 if row < 3 else 0.021
            height = 0.0042 if col < 3 else 0.0048
            button = (
                cq.Workplane("XY")
                .box(width, depth, height, centered=(True, True, False))
                .translate((x_pos, y_pos, 0.003))
            )
            keypad = keypad.union(button)
    return keypad.edges("|Z").fillet(0.0024)


def _make_cover_panel() -> object:
    lid = cq.Workplane("XY").box(0.088, 0.154, 0.008, centered=(False, True, False))
    pull = (
        cq.Workplane("XY")
        .box(0.012, 0.062, 0.004, centered=(False, True, False))
        .translate((0.076, 0.000, 0.008))
    )
    return lid.union(pull).edges("|Z").fillet(0.004)


def _make_display_panel() -> object:
    body = (
        cq.Workplane("XY")
        .box(0.056, 0.062, 0.010, centered=(False, True, False))
        .translate((-0.056, 0.000, 0.000))
    )
    body = body.edges("|Z").fillet(0.006)
    collar = (
        cq.Workplane("XY")
        .box(0.014, 0.040, 0.008, centered=(False, True, False))
        .translate((-0.014, 0.000, 0.001))
    )
    return body.union(collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boutique_cash_register")

    body_finish = model.material("body_finish", rgba=(0.80, 0.75, 0.67, 1.0))
    body_dark = model.material("body_dark", rgba=(0.24, 0.23, 0.22, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.69, 0.65, 0.58, 1.0))
    key_finish = model.material("key_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    button_finish = model.material("button_finish", rgba=(0.90, 0.89, 0.85, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.44, 0.39, 0.65))
    trim = model.material("trim", rgba=(0.58, 0.56, 0.52, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((BASE_DEPTH, BASE_WIDTH, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=body_finish,
        name="base_floor",
    )
    cabinet.visual(
        Box((BASE_DEPTH, BASE_WIDTH, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, BASE_HEIGHT - 0.005)),
        material=body_finish,
        name="top_deck",
    )
    cabinet.visual(
        Box((BASE_DEPTH, 0.010, BASE_HEIGHT - 0.010)),
        origin=Origin(xyz=(0.000, -0.135, (BASE_HEIGHT + 0.010) * 0.5)),
        material=body_finish,
        name="left_wall",
    )
    cabinet.visual(
        Box((BASE_DEPTH, 0.010, BASE_HEIGHT - 0.010)),
        origin=Origin(xyz=(0.000, 0.135, (BASE_HEIGHT + 0.010) * 0.5)),
        material=body_finish,
        name="right_wall",
    )
    cabinet.visual(
        Box((0.010, BASE_WIDTH - 0.020, BASE_HEIGHT - 0.010)),
        origin=Origin(xyz=(-0.1175, 0.000, (BASE_HEIGHT + 0.010) * 0.5)),
        material=body_finish,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.018, BASE_WIDTH, 0.016)),
        origin=Origin(xyz=(0.114, 0.000, 0.079)),
        material=body_dark,
        name="front_header",
    )
    cabinet.visual(
        Box((0.080, 0.230, 0.006)),
        origin=Origin(xyz=(-0.016, 0.000, BASE_HEIGHT + 0.003)),
        material=trim,
        name="pod_plinth",
    )
    cabinet.visual(
        mesh_from_cadquery(_make_pod_shell(), "cash_register_pod_shell"),
        origin=Origin(xyz=(POD_OFFSET_X, 0.000, POD_OFFSET_Z)),
        material=body_finish,
        name="pod_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_make_keypad_module(), "cash_register_keypad"),
        origin=Origin(
            xyz=(0.028, 0.000, BASE_HEIGHT + 0.0465),
            rpy=(0.000, 0.38, 0.000),
        ),
        material=button_finish,
        name="keypad",
    )
    cabinet.visual(
        Box((0.122, 0.106, 0.002)),
        origin=Origin(
            xyz=(0.028, 0.000, BASE_HEIGHT + 0.0438),
            rpy=(0.000, 0.38, 0.000),
        ),
        material=key_finish,
        name="keypad_base",
    )
    cabinet.visual(
        Box((0.050, 0.010, 0.006)),
        origin=Origin(xyz=(0.108, 0.000, 0.078)),
        material=trim,
        name="drawer_pull_track",
    )
    cabinet.visual(
        Box((0.020, 0.050, 0.004)),
        origin=Origin(xyz=(POD_OFFSET_X + POD_REAR_X - 0.005, 0.000, POD_OFFSET_Z + POD_TOP_Z)),
        material=trim,
        name="display_saddle",
    )
    cabinet.visual(
        Box((0.190, 0.012, 0.010)),
        origin=Origin(xyz=(0.000, -0.124, 0.024)),
        material=trim,
        name="drawer_runner_0",
    )
    cabinet.visual(
        Box((0.190, 0.012, 0.010)),
        origin=Origin(xyz=(0.000, 0.124, 0.024)),
        material=trim,
        name="drawer_runner_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.194, 0.236, 0.010)),
        origin=Origin(xyz=(0.097, 0.000, 0.005)),
        material=drawer_finish,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.194, 0.004, 0.038)),
        origin=Origin(xyz=(0.097, -0.116, 0.024)),
        material=drawer_finish,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((0.194, 0.004, 0.038)),
        origin=Origin(xyz=(0.097, 0.116, 0.024)),
        material=drawer_finish,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((0.004, 0.236, 0.038)),
        origin=Origin(xyz=(0.002, 0.000, 0.024)),
        material=drawer_finish,
        name="drawer_rear_wall",
    )
    drawer.visual(
        Box((0.016, 0.252, 0.056)),
        origin=Origin(xyz=(0.198, 0.000, 0.028)),
        material=body_dark,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.008, 0.082, 0.010)),
        origin=Origin(xyz=(0.207, 0.000, 0.033)),
        material=trim,
        name="drawer_pull",
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(DRAWER_JOINT_X, 0.000, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    receipt_cover = model.part("receipt_cover")
    receipt_cover.visual(
        mesh_from_cadquery(_make_cover_panel(), "cash_register_receipt_cover"),
        material=body_dark,
        name="cover_panel",
    )
    receipt_cover.visual(
        Cylinder(radius=0.0045, length=0.144),
        origin=Origin(xyz=(0.003, 0.000, 0.0045), rpy=(1.57079632679, 0.000, 0.000)),
        material=trim,
        name="cover_hinge_barrel",
    )
    model.articulation(
        "cabinet_to_receipt_cover",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=receipt_cover,
        origin=Origin(xyz=(POD_OFFSET_X - 0.061, 0.000, POD_OFFSET_Z + 0.0722)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.2,
            lower=0.0,
            upper=1.18,
        ),
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_make_display_panel(), "cash_register_display_panel"),
        material=body_dark,
        name="display_panel",
    )
    display.visual(
        Box((0.046, 0.046, 0.002)),
        origin=Origin(xyz=(-0.040, 0.000, 0.0105)),
        material=glass,
        name="display_window",
    )
    display.visual(
        Cylinder(radius=0.0045, length=0.046),
        origin=Origin(xyz=(-0.004, 0.000, 0.0045), rpy=(1.57079632679, 0.000, 0.000)),
        material=trim,
        name="display_hinge_barrel",
    )
    model.articulation(
        "cabinet_to_display",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=display,
        origin=Origin(xyz=(POD_OFFSET_X + POD_REAR_X + 0.001, 0.000, POD_OFFSET_Z + POD_TOP_Z + 0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.4,
            lower=0.0,
            upper=1.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part("drawer")
    receipt_cover = object_model.get_part("receipt_cover")
    display = object_model.get_part("display")

    drawer_slide = object_model.get_articulation("cabinet_to_drawer")
    cover_hinge = object_model.get_articulation("cabinet_to_receipt_cover")
    display_hinge = object_model.get_articulation("cabinet_to_display")

    drawer_limits = drawer_slide.motion_limits
    cover_limits = cover_hinge.motion_limits
    display_limits = display_hinge.motion_limits

    ctx.expect_within(
        drawer,
        cabinet,
        axes="yz",
        margin=0.006,
        name="drawer stays guided between cabinet walls",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="x",
        min_overlap=0.090,
        name="drawer remains retained inside cabinet at rest",
    )
    ctx.expect_gap(
        receipt_cover,
        cabinet,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="pod_shell",
        max_gap=0.006,
        max_penetration=0.0,
        name="receipt cover sits on the top of the keypad pod",
    )
    ctx.expect_overlap(
        receipt_cover,
        cabinet,
        axes="xy",
        elem_a="cover_panel",
        elem_b="pod_shell",
        min_overlap=0.060,
        name="receipt cover footprint stays centered over pod opening",
    )

    closed_drawer_aabb = ctx.part_world_aabb(drawer)
    extended_drawer_aabb = None
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_within(
                drawer,
                cabinet,
                axes="yz",
                margin=0.006,
                name="drawer stays guided between cabinet walls when open",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="x",
                min_overlap=0.080,
                name="drawer keeps retained insertion when open",
            )
            extended_drawer_aabb = ctx.part_world_aabb(drawer)
    ctx.check(
        "drawer opens forward",
        closed_drawer_aabb is not None
        and extended_drawer_aabb is not None
        and extended_drawer_aabb[1][0] > closed_drawer_aabb[1][0] + 0.090,
        details=f"closed={closed_drawer_aabb!r}, open={extended_drawer_aabb!r}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(receipt_cover, elem="cover_panel")
    open_cover_aabb = None
    if cover_limits is not None and cover_limits.upper is not None:
        with ctx.pose({cover_hinge: cover_limits.upper}):
            open_cover_aabb = ctx.part_element_world_aabb(receipt_cover, elem="cover_panel")
    ctx.check(
        "receipt cover lifts above the pod",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.030,
        details=f"closed={closed_cover_aabb!r}, open={open_cover_aabb!r}",
    )

    closed_display_aabb = ctx.part_element_world_aabb(display, elem="display_panel")
    open_display_aabb = None
    if display_limits is not None and display_limits.upper is not None:
        with ctx.pose({display_hinge: display_limits.upper}):
            open_display_aabb = ctx.part_element_world_aabb(display, elem="display_panel")
    display_stands_up = False
    if open_display_aabb is not None:
        open_dx = open_display_aabb[1][0] - open_display_aabb[0][0]
        open_dz = open_display_aabb[1][2] - open_display_aabb[0][2]
        display_stands_up = open_dz > open_dx * 2.0
    ctx.check(
        "customer display flips upright",
        closed_display_aabb is not None
        and open_display_aabb is not None
        and open_display_aabb[1][2] > closed_display_aabb[1][2] + 0.040
        and display_stands_up,
        details=f"closed={closed_display_aabb!r}, open={open_display_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
