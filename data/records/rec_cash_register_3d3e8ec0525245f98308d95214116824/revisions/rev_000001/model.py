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


BODY_LENGTH = 0.34
BODY_WIDTH = 0.30
BODY_HEIGHT = 0.10

DRAWER_TRAVEL = 0.12
POD_MOUNT_X = -0.015
POD_MOUNT_Z = BODY_HEIGHT
def _pod_shape() -> cq.Workplane:
    pedestal = (
        cq.Workplane("XY")
        .box(0.118, 0.120, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    neck = (
        cq.Workplane("XY")
        .box(0.094, 0.092, 0.016, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.004, 0.0, 0.014))
    )
    shell = (
        cq.Workplane("XY")
        .box(0.192, 0.164, 0.042, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((-0.010, 0.0, 0.026))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.100, 0.140, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
        .translate((0.042, 0.0, 0.018))
    )
    key_deck = (
        cq.Workplane("XY")
        .box(0.152, 0.122, 0.004, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0015)
        .translate((0.020, 0.0, 0.064))
    )
    display_mount = (
        cq.Workplane("XY")
        .box(0.082, 0.018, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0025)
        .translate((-0.070, 0.0, 0.056))
    )
    return pedestal.union(neck).union(shell).union(nose).union(key_deck).union(display_mount)


def _keycap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.022, 0.018, 0.008, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0025)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boutique_cash_register")

    cream = model.material("cream", rgba=(0.89, 0.86, 0.80, 1.0))
    warm_white = model.material("warm_white", rgba=(0.95, 0.94, 0.91, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    cocoa = model.material("cocoa", rgba=(0.30, 0.22, 0.17, 1.0))
    display_glow = model.material("display_glow", rgba=(0.24, 0.48, 0.45, 0.55))
    key_rose = model.material("key_rose", rgba=(0.84, 0.63, 0.61, 1.0))
    key_sage = model.material("key_sage", rgba=(0.69, 0.76, 0.65, 1.0))
    key_honey = model.material("key_honey", rgba=(0.84, 0.73, 0.49, 1.0))
    key_oat = model.material("key_oat", rgba=(0.86, 0.82, 0.71, 1.0))
    key_teal = model.material("key_teal", rgba=(0.60, 0.73, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=cream,
        name="bottom_plate",
    )
    body.visual(
        Box((BODY_LENGTH, 0.010, 0.084)),
        origin=Origin(xyz=(0.0, -0.145, 0.052)),
        material=cream,
        name="side_wall_0",
    )
    body.visual(
        Box((BODY_LENGTH, 0.010, 0.084)),
        origin=Origin(xyz=(0.0, 0.145, 0.052)),
        material=cream,
        name="side_wall_1",
    )
    body.visual(
        Box((0.016, 0.280, 0.084)),
        origin=Origin(xyz=(-0.162, 0.0, 0.052)),
        material=cream,
        name="back_wall",
    )
    body.visual(
        Box((0.186, 0.280, 0.012)),
        origin=Origin(xyz=(-0.077, 0.0, 0.094)),
        material=cream,
        name="top_cover",
    )
    drawer = model.part("drawer")
    drawer.visual(
        Box((0.258, 0.246, 0.006)),
        origin=Origin(xyz=(-0.129, 0.0, 0.003)),
        material=warm_white,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.258, 0.008, 0.036)),
        origin=Origin(xyz=(-0.129, -0.119, 0.021)),
        material=warm_white,
        name="tray_wall_0",
    )
    drawer.visual(
        Box((0.258, 0.008, 0.036)),
        origin=Origin(xyz=(-0.129, 0.119, 0.021)),
        material=warm_white,
        name="tray_wall_1",
    )
    drawer.visual(
        Box((0.008, 0.230, 0.036)),
        origin=Origin(xyz=(-0.254, 0.0, 0.021)),
        material=warm_white,
        name="tray_back",
    )
    drawer.visual(
        Box((0.012, 0.266, 0.058)),
        origin=Origin(xyz=(-0.006, 0.0, 0.029)),
        material=warm_white,
        name="front_panel",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_LENGTH * 0.5, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    keypad_pod = model.part("keypad_pod")
    keypad_pod.visual(
        mesh_from_cadquery(_pod_shape(), "cash_register_keypad_pod"),
        material=cocoa,
        name="pod_shell",
    )
    model.articulation(
        "body_to_keypad_pod",
        ArticulationType.FIXED,
        parent=body,
        child=keypad_pod,
        origin=Origin(xyz=(POD_MOUNT_X, 0.0, POD_MOUNT_Z)),
    )

    customer_display = model.part("customer_display")
    customer_display.visual(
        Cylinder(radius=0.004, length=0.088),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    customer_display.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.004)),
        material=charcoal,
        name="support_arm",
    )
    customer_display.visual(
        Box((0.052, 0.014, 0.036)),
        origin=Origin(xyz=(0.058, 0.0, 0.020)),
        material=charcoal,
        name="display_head",
    )
    customer_display.visual(
        Box((0.040, 0.003, 0.024)),
        origin=Origin(xyz=(0.065, 0.0, 0.021)),
        material=display_glow,
        name="display_glass",
    )
    model.articulation(
        "pod_to_customer_display",
        ArticulationType.REVOLUTE,
        parent=keypad_pod,
        child=customer_display,
        origin=Origin(xyz=(-0.106, 0.0, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )

    keycap_mesh = mesh_from_cadquery(_keycap_shape(), "cash_register_keycap")
    key_rows = (key_rose, key_sage, key_honey, key_oat, key_teal)
    key_x_positions = (0.000, 0.026, 0.052, 0.078)
    key_y_positions = (-0.052, -0.026, 0.000, 0.026, 0.052)

    for row_index, y_pos in enumerate(key_y_positions):
        for col_index, x_pos in enumerate(key_x_positions):
            key_name = f"key_{row_index}_{col_index}"
            key_part = model.part(key_name)
            key_part.visual(
                keycap_mesh,
                material=key_rows[row_index],
                name="keycap",
            )
            model.articulation(
                f"pod_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=keypad_pod,
                child=key_part,
                origin=Origin(xyz=(x_pos, y_pos, 0.068)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.5,
                    velocity=0.08,
                    lower=0.0,
                    upper=0.003,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    keypad_pod = object_model.get_part("keypad_pod")
    customer_display = object_model.get_part("customer_display")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    display_joint = object_model.get_articulation("pod_to_customer_display")
    sample_keys = [
        object_model.get_part("key_0_0"),
        object_model.get_part("key_2_1"),
        object_model.get_part("key_4_3"),
    ]

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        margin=0.02,
        name="drawer stays centered within body width and height",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        min_overlap=0.20,
        name="drawer remains inserted when closed",
    )

    closed_display_box = ctx.part_element_world_aabb(customer_display, elem="display_head")
    drawer_rest = ctx.part_world_position(drawer)

    if drawer_joint.motion_limits is not None and drawer_joint.motion_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.05,
                name="drawer keeps retained insertion when extended",
            )
            drawer_extended = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends forward",
            drawer_rest is not None
            and drawer_extended is not None
            and drawer_extended[0] > drawer_rest[0] + 0.08,
            details=f"rest={drawer_rest}, extended={drawer_extended}",
        )

    if display_joint.motion_limits is not None and display_joint.motion_limits.upper is not None:
        with ctx.pose({display_joint: display_joint.motion_limits.upper}):
            open_display_box = ctx.part_element_world_aabb(customer_display, elem="display_head")
        display_rises = False
        if closed_display_box is not None and open_display_box is not None:
            closed_center = tuple((closed_display_box[0][i] + closed_display_box[1][i]) * 0.5 for i in range(3))
            open_center = tuple((open_display_box[0][i] + open_display_box[1][i]) * 0.5 for i in range(3))
            display_rises = open_center[2] > closed_center[2] + 0.025
        ctx.check(
            "customer display flips upward",
            display_rises,
            details=f"closed={closed_display_box}, open={open_display_box}",
        )

    for key_part in sample_keys:
        ctx.expect_within(
            key_part,
            keypad_pod,
            axes="xy",
            margin=0.005,
            name=f"{key_part.name} stays on keypad deck footprint",
        )
        key_joint = object_model.get_articulation(f"pod_to_{key_part.name}")
        key_rest = ctx.part_world_position(key_part)
        if key_joint.motion_limits is not None and key_joint.motion_limits.upper is not None:
            with ctx.pose({key_joint: key_joint.motion_limits.upper}):
                key_pressed = ctx.part_world_position(key_part)
            ctx.check(
                f"{key_part.name} presses downward",
                key_rest is not None
                and key_pressed is not None
                and key_pressed[2] < key_rest[2] - 0.002,
                details=f"rest={key_rest}, pressed={key_pressed}",
            )

    return ctx.report()


object_model = build_object_model()
