from __future__ import annotations

from math import pi

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


BODY_D = 0.52
BODY_W = 0.43
BODY_H = 0.31

DRAWER_OPENING_D = 0.382
DRAWER_OPENING_W = 0.372
DRAWER_OPENING_H = 0.128
DRAWER_BOTTOM_Z = 0.024

SCANNER_LID_D = 0.32
SCANNER_LID_W = 0.41
SCANNER_LID_T = 0.022

SERVICE_DOOR_D = 0.19
SERVICE_DOOR_H = 0.17
SERVICE_DOOR_T = 0.015

PAD_L = 0.115
PAD_W = 0.108
PAD_T = 0.02
PAD_REST_TILT = 0.38


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _make_body_shell() -> cq.Workplane:
    shell = _box((BODY_D, BODY_W, BODY_H), (0.0, 0.0, BODY_H / 2.0))
    shell = shell.edges("|Z").fillet(0.018)

    drawer_cut = _box(
        (DRAWER_OPENING_D, DRAWER_OPENING_W, DRAWER_OPENING_H),
        (
            BODY_D / 2.0 - DRAWER_OPENING_D / 2.0 + 0.006,
            0.0,
            DRAWER_BOTTOM_Z + DRAWER_OPENING_H / 2.0,
        ),
    )
    shell = shell.cut(drawer_cut.val())

    service_cut = _box(
        (0.205, 0.165, 0.186),
        (-0.025, BODY_W / 2.0 - 0.076, 0.185),
    )
    shell = shell.cut(service_cut.val())

    scanner_recess = _box((0.312, 0.352, 0.012), (-0.05, 0.0, BODY_H - 0.006))
    shell = shell.cut(scanner_recess.val())

    output_slot = _box((0.085, 0.275, 0.028), (BODY_D / 2.0 - 0.016, 0.0, 0.215))
    shell = shell.cut(output_slot.val())

    return shell


def _make_control_base() -> cq.Workplane:
    base = _box((0.084, 0.112, 0.022), (0.184, -0.118, BODY_H + 0.011))
    base = base.edges("|Z").fillet(0.008)

    cheek_a = _box((0.012, 0.014, 0.018), (0.149, -0.162, BODY_H + 0.025))
    cheek_b = _box((0.012, 0.014, 0.018), (0.149, -0.074, BODY_H + 0.025))
    pivot_tube = (
        cq.Workplane("XZ")
        .circle(0.006)
        .extrude(0.088)
        .translate((0.149, -0.162, BODY_H + 0.025))
    )

    return base.union(cheek_a.val()).union(cheek_b.val()).union(pivot_tube.val())


def _make_scanner_lid() -> cq.Workplane:
    outer = _box(
        (SCANNER_LID_D, SCANNER_LID_W, SCANNER_LID_T),
        (SCANNER_LID_D / 2.0, 0.0, SCANNER_LID_T / 2.0),
    )
    outer = outer.edges("|Z").fillet(0.008)

    inner = _box(
        (SCANNER_LID_D - 0.022, SCANNER_LID_W - 0.022, SCANNER_LID_T - 0.007),
        (SCANNER_LID_D / 2.0 + 0.001, 0.0, (SCANNER_LID_T - 0.007) / 2.0),
    )
    finger_notch = _box((0.032, 0.085, 0.012), (SCANNER_LID_D - 0.008, 0.0, 0.006))
    return outer.cut(inner.val()).cut(finger_notch.val())


def _make_drawer() -> cq.Workplane:
    fascia = _box((0.018, 0.36, 0.124), (0.009, 0.0, 0.062))
    tray = _box((0.276, 0.34, 0.105), (-0.129, 0.0, 0.0525))
    body = fascia.union(tray.val())

    tray_void = _box((0.246, 0.316, 0.088), (-0.139, 0.0, 0.064))
    handle_recess = _box((0.01, 0.148, 0.022), (0.013, 0.0, 0.076))
    return body.cut(tray_void.val()).cut(handle_recess.val())


def _make_service_door() -> cq.Workplane:
    outer = _box((SERVICE_DOOR_D, SERVICE_DOOR_T, SERVICE_DOOR_H), (SERVICE_DOOR_D / 2.0, 0.0, 0.0))
    inner = _box((SERVICE_DOOR_D - 0.022, SERVICE_DOOR_T - 0.004, SERVICE_DOOR_H - 0.022), (SERVICE_DOOR_D / 2.0 + 0.002, -0.001, 0.0))
    return outer.cut(inner.val())


def _make_control_pad() -> cq.Workplane:
    outer = _box((PAD_L, PAD_W, PAD_T), (0.061, 0.0, 0.006))
    outer = outer.edges("|Z").fillet(0.005)

    keypad_recess = _box((0.072, 0.082, 0.004), (0.076, 0.0, 0.014))
    screen_recess = _box((0.042, 0.03, 0.0025), (0.031, -0.024, 0.01475))
    wheel_recess = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.0025)
        .translate((0.028, 0.03, 0.01475))
    )
    return outer.cut(keypad_recess.val()).cut(screen_recess.val()).cut(wheel_recess.val())


def _make_selector_wheel() -> cq.Workplane:
    rim = cq.Workplane("XY").circle(0.018).extrude(0.008)
    dish = cq.Workplane("XY").circle(0.011).extrude(0.003).translate((0.0, 0.0, 0.005))
    hub = cq.Workplane("XY").circle(0.006).extrude(0.009)
    return rim.cut(dish.val()).union(hub.val())


def _add_keypad(model: ArticulatedObject, control_pad, button_mat) -> None:
    key_size = (0.014, 0.015, 0.004)
    row_x = (0.051, 0.067, 0.083, 0.099)
    col_y = (-0.024, 0.0, 0.024)

    for row, x_pos in enumerate(row_x):
        for col, y_pos in enumerate(col_y):
            key = model.part(f"key_{row}_{col}")
            key.visual(
                Box(key_size),
                origin=Origin(xyz=(0.0, 0.0, key_size[2] / 2.0)),
                material=button_mat,
                name="key_cap",
            )
            model.articulation(
                f"control_pad_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=control_pad,
                child=key,
                origin=Origin(xyz=(x_pos, y_pos, 0.018)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.0008),
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="duplex_office_printer")

    body_mat = model.material("body_plastic", rgba=(0.86, 0.87, 0.88, 1.0))
    trim_mat = model.material("trim_plastic", rgba=(0.20, 0.22, 0.25, 1.0))
    button_mat = model.material("button_plastic", rgba=(0.31, 0.33, 0.35, 1.0))
    wheel_mat = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_D, BODY_W, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body_mat,
        name="base_pan",
    )
    body.visual(
        Box((BODY_D, 0.018, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + 0.009, BODY_H / 2.0)),
        material=body_mat,
        name="left_wall",
    )
    body.visual(
        Box((0.142, 0.018, BODY_H)),
        origin=Origin(xyz=(-0.189, BODY_W / 2.0 - 0.009, BODY_H / 2.0)),
        material=body_mat,
        name="right_rear_wall",
    )
    body.visual(
        Box((0.192, 0.018, BODY_H)),
        origin=Origin(xyz=(0.164, BODY_W / 2.0 - 0.009, BODY_H / 2.0)),
        material=body_mat,
        name="right_front_wall",
    )
    body.visual(
        Box((0.192, 0.018, 0.104)),
        origin=Origin(xyz=(-0.024, BODY_W / 2.0 - 0.009, 0.052)),
        material=body_mat,
        name="right_lower_rail",
    )
    body.visual(
        Box((0.192, 0.018, 0.036)),
        origin=Origin(xyz=(-0.024, BODY_W / 2.0 - 0.009, 0.292)),
        material=body_mat,
        name="right_upper_rail",
    )
    body.visual(
        Box((0.018, BODY_W, BODY_H)),
        origin=Origin(xyz=(-BODY_D / 2.0 + 0.009, 0.0, BODY_H / 2.0)),
        material=body_mat,
        name="rear_wall",
    )
    body.visual(
        Box((0.324, BODY_W, 0.018)),
        origin=Origin(xyz=(-0.058, 0.0, BODY_H - 0.009)),
        material=body_mat,
        name="body_shell",
    )
    body.visual(
        Box((0.036, BODY_W, 0.146)),
        origin=Origin(xyz=(BODY_D / 2.0 - 0.018, 0.0, 0.237)),
        material=body_mat,
        name="front_upper_wall",
    )
    body.visual(
        Box((0.036, 0.035, 0.124)),
        origin=Origin(xyz=(BODY_D / 2.0 - 0.018, -0.1975, 0.086)),
        material=body_mat,
        name="front_post_0",
    )
    body.visual(
        Box((0.036, 0.035, 0.124)),
        origin=Origin(xyz=(BODY_D / 2.0 - 0.018, 0.1975, 0.086)),
        material=body_mat,
        name="front_post_1",
    )
    body.visual(
        Box((0.086, 0.112, 0.02)),
        origin=Origin(xyz=(0.182, -0.118, BODY_H + 0.01)),
        material=trim_mat,
        name="control_base",
    )
    body.visual(
        Box((0.012, 0.014, 0.018)),
        origin=Origin(xyz=(0.146, -0.162, BODY_H + 0.029)),
        material=trim_mat,
        name="control_cheek_0",
    )
    body.visual(
        Box((0.012, 0.014, 0.018)),
        origin=Origin(xyz=(0.146, -0.074, BODY_H + 0.029)),
        material=trim_mat,
        name="control_cheek_1",
    )
    body.visual(
        Box((0.012, 0.088, 0.012)),
        origin=Origin(xyz=(0.146, -0.118, BODY_H + 0.026)),
        material=trim_mat,
        name="control_pivot",
    )

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(
        Box((SCANNER_LID_D, SCANNER_LID_W, 0.006)),
        origin=Origin(xyz=(SCANNER_LID_D / 2.0, 0.0, 0.019)),
        material=body_mat,
        name="lid_shell",
    )
    scanner_lid.visual(
        Box((SCANNER_LID_D, 0.008, 0.016)),
        origin=Origin(xyz=(SCANNER_LID_D / 2.0, -SCANNER_LID_W / 2.0 + 0.004, 0.008)),
        material=body_mat,
        name="lid_wall_0",
    )
    scanner_lid.visual(
        Box((SCANNER_LID_D, 0.008, 0.016)),
        origin=Origin(xyz=(SCANNER_LID_D / 2.0, SCANNER_LID_W / 2.0 - 0.004, 0.008)),
        material=body_mat,
        name="lid_wall_1",
    )
    scanner_lid.visual(
        Box((0.008, SCANNER_LID_W, 0.016)),
        origin=Origin(xyz=(0.004, 0.0, 0.008)),
        material=body_mat,
        name="lid_wall_2",
    )
    scanner_lid.visual(
        Box((0.008, SCANNER_LID_W, 0.016)),
        origin=Origin(xyz=(SCANNER_LID_D - 0.004, 0.0, 0.008)),
        material=body_mat,
        name="lid_wall_3",
    )

    paper_drawer = model.part("paper_drawer")
    paper_drawer.visual(
        Box((0.27, 0.34, 0.008)),
        origin=Origin(xyz=(-0.135, 0.0, 0.004)),
        material=body_mat,
        name="drawer_floor",
    )
    paper_drawer.visual(
        Box((0.27, 0.008, 0.09)),
        origin=Origin(xyz=(-0.135, -0.166, 0.045)),
        material=body_mat,
        name="drawer_wall_0",
    )
    paper_drawer.visual(
        Box((0.27, 0.008, 0.09)),
        origin=Origin(xyz=(-0.135, 0.166, 0.045)),
        material=body_mat,
        name="drawer_wall_1",
    )
    paper_drawer.visual(
        Box((0.008, 0.34, 0.09)),
        origin=Origin(xyz=(-0.266, 0.0, 0.045)),
        material=body_mat,
        name="drawer_wall_2",
    )
    paper_drawer.visual(
        Box((0.01, 0.34, 0.07)),
        origin=Origin(xyz=(-0.005, 0.0, 0.035)),
        material=body_mat,
        name="drawer_wall_3",
    )
    paper_drawer.visual(
        Box((0.018, 0.36, 0.124)),
        origin=Origin(xyz=(0.009, 0.0, 0.062)),
        material=body_mat,
        name="drawer_shell",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((SERVICE_DOOR_D, SERVICE_DOOR_T, SERVICE_DOOR_H)),
        origin=Origin(xyz=(SERVICE_DOOR_D / 2.0, -SERVICE_DOOR_T / 2.0, 0.0)),
        material=body_mat,
        name="door_shell",
    )

    control_pad = model.part("control_pad")
    control_pad.visual(
        Box((0.012, 0.088, 0.01)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=trim_mat,
        name="pad_pivot",
    )
    control_pad.visual(
        Box((0.11, PAD_W, 0.014)),
        origin=Origin(xyz=(0.06, 0.0, 0.011)),
        material=trim_mat,
        name="pad_shell",
    )

    selector_wheel = model.part("selector_wheel")
    selector_wheel.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=wheel_mat,
        name="wheel_shell",
    )

    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=scanner_lid,
        origin=Origin(xyz=(-0.185, 0.0, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.28),
    )
    model.articulation(
        "body_to_paper_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_drawer,
        origin=Origin(xyz=(0.241, 0.0, DRAWER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.25, lower=0.0, upper=0.18),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(-0.118, BODY_W / 2.0 - 0.003, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_control_pad",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_pad,
        origin=Origin(xyz=(0.146, -0.118, BODY_H + 0.02), rpy=(0.0, -PAD_REST_TILT, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.18, upper=0.35),
    )
    model.articulation(
        "control_pad_to_selector_wheel",
        ArticulationType.CONTINUOUS,
        parent=control_pad,
        child=selector_wheel,
        origin=Origin(xyz=(0.028, 0.03, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )

    _add_keypad(model, control_pad, button_mat)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    scanner_lid = object_model.get_part("scanner_lid")
    paper_drawer = object_model.get_part("paper_drawer")
    service_door = object_model.get_part("service_door")
    control_pad = object_model.get_part("control_pad")
    selector_wheel = object_model.get_part("selector_wheel")
    key_0_0 = object_model.get_part("key_0_0")

    lid_joint = object_model.get_articulation("body_to_scanner_lid")
    drawer_joint = object_model.get_articulation("body_to_paper_drawer")
    door_joint = object_model.get_articulation("body_to_service_door")
    pad_joint = object_model.get_articulation("body_to_control_pad")
    wheel_joint = object_model.get_articulation("control_pad_to_selector_wheel")
    key_joint = object_model.get_articulation("control_pad_to_key_0_0")

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    ctx.check(
        "numeric keypad has twelve separate keys",
        len(key_parts) == 12,
        details=f"found {len(key_parts)} keys",
    )

    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_pivot",
        elem_b="pad_pivot",
        reason="The control-pad hinge is represented with compact overlapping pivot blocks instead of a detailed barrel pair.",
    )
    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_pivot",
        elem_b="pad_shell",
        reason="The control-pad pivot rail slightly intrudes into the simplified pad housing to stand in for a captured hinge cradle.",
    )
    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_base",
        elem_b="pad_pivot",
        reason="The pivot block is slightly embedded into the control-base pedestal to represent a compact tilting hinge mount.",
    )
    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_cheek_0",
        elem_b="pad_shell",
        reason="The front control-pad cheek overlaps the simplified pad housing slightly to stand in for a captured hinge mount.",
    )
    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_cheek_0",
        elem_b="pad_pivot",
        reason="The front control-pad cheek captures the simplified pad pivot block in lieu of a detailed clevis-and-pin hinge.",
    )
    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_cheek_1",
        elem_b="pad_shell",
        reason="The rear control-pad cheek overlaps the simplified pad housing slightly to stand in for a captured hinge mount.",
    )
    ctx.allow_overlap(
        "body",
        "control_pad",
        elem_a="control_cheek_1",
        elem_b="pad_pivot",
        reason="The rear control-pad cheek captures the simplified pad pivot block in lieu of a detailed clevis-and-pin hinge.",
    )
    ctx.allow_overlap(
        control_pad,
        selector_wheel,
        elem_a="pad_shell",
        elem_b="wheel_shell",
        reason="The selector wheel uses a simplified hub that sits partially embedded in the solid control-pad housing.",
    )
    for key_part in key_parts:
        ctx.allow_overlap(
            control_pad,
            key_part,
            elem_a="pad_shell",
            elem_b="key_cap",
            reason="Each keypad button is represented as a slightly embedded cap on the simplified solid control-pad housing.",
        )

    ctx.expect_overlap(
        scanner_lid,
        "body",
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.18,
        name="scanner lid covers the scanner deck footprint",
    )

    drawer_rest = ctx.part_world_position(paper_drawer)
    with ctx.pose({drawer_joint: 0.18}):
        drawer_extended = ctx.part_world_position(paper_drawer)
    ctx.check(
        "paper drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.12,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    lid_closed = ctx.part_element_world_aabb(scanner_lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.28}):
        lid_open = ctx.part_element_world_aabb(scanner_lid, elem="lid_shell")
    ctx.check(
        "scanner lid opens upward",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.18,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    door_closed = ctx.part_element_world_aabb(service_door, elem="door_shell")
    with ctx.pose({door_joint: 1.5}):
        door_open = ctx.part_element_world_aabb(service_door, elem="door_shell")
    ctx.check(
        "service door swings outward",
        door_closed is not None
        and door_open is not None
        and door_open[1][1] > door_closed[1][1] + 0.08,
        details=f"closed={door_closed}, open={door_open}",
    )

    pad_low = None
    pad_high = None
    with ctx.pose({pad_joint: -0.18}):
        pad_low = ctx.part_element_world_aabb(control_pad, elem="pad_shell")
    with ctx.pose({pad_joint: 0.35}):
        pad_high = ctx.part_element_world_aabb(control_pad, elem="pad_shell")
    ctx.check(
        "control pad tilts upward at the top edge",
        pad_low is not None
        and pad_high is not None
        and pad_high[1][2] > pad_low[1][2] + 0.02,
        details=f"low={pad_low}, high={pad_high}",
    )

    ctx.check(
        "selector wheel uses continuous rotation",
        wheel_joint.motion_limits is not None
        and wheel_joint.motion_limits.lower is None
        and wheel_joint.motion_limits.upper is None,
        details=f"limits={wheel_joint.motion_limits}",
    )

    key_rest = ctx.part_world_position(key_0_0)
    with ctx.pose({key_joint: 0.0008}):
        key_pressed = ctx.part_world_position(key_0_0)
    ctx.check(
        "keypad button presses inward",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.0005,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
