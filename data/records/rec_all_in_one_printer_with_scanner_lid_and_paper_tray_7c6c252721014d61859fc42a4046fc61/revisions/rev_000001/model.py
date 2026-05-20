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


BODY_W = 0.48
BODY_D = 0.45
BODY_H = 0.32
WALL = 0.02

SCANNER_CENTER = (-0.09, 0.045)
SCANNER_OPENING = (0.30, 0.23)
SCANNER_LID = (0.32, 0.29, 0.052)

PANEL_W = 0.16
PANEL_D = 0.12
PANEL_H = 0.026
PANEL_CENTER = (0.155, -0.13, BODY_H)

KEY_SIZE = (0.022, 0.020, 0.005)
KEY_TRAVEL = 0.003
KEY_XS = (-0.046, -0.019, 0.008)
KEY_YS = (0.034, 0.008, -0.018, -0.044)


def cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def build_body_shape() -> cq.Workplane:
    outer = cq_box((BODY_W, BODY_D, BODY_H), (0.0, 0.0, BODY_H / 2.0))
    inner = cq_box(
        (BODY_W - 2.0 * WALL, BODY_D - 2.0 * WALL, BODY_H - WALL),
        (0.0, 0.0, WALL + (BODY_H - WALL) / 2.0),
    )
    body = outer.cut(inner)

    drawer_opening = cq_box((0.432, 0.28, 0.172), (0.0, -0.085, 0.106))
    output_cavity = cq_box((0.318, 0.19, 0.06), (0.0, -0.125, 0.234))
    scanner_opening = cq_box(
        (SCANNER_OPENING[0], SCANNER_OPENING[1], 0.03),
        (SCANNER_CENTER[0], SCANNER_CENTER[1], BODY_H - 0.005),
    )
    service_opening = cq_box((0.035, 0.24, 0.18), (BODY_W / 2.0, 0.0, 0.17))

    body = body.cut(drawer_opening)
    body = body.cut(output_cavity)
    body = body.cut(scanner_opening)
    body = body.cut(service_opening)

    output_floor = cq_box((BODY_W - 2.0 * WALL, 0.18, 0.012), (0.0, -0.05, 0.196))
    hinge_block_0 = cq_box((0.034, 0.026, 0.028), (-0.17, 0.205, 0.306))
    hinge_block_1 = cq_box((0.034, 0.026, 0.028), (0.07, 0.205, 0.306))

    body = body.union(output_floor)
    body = body.union(hinge_block_0)
    body = body.union(hinge_block_1)
    return body


def build_scanner_bed_frame() -> cq.Workplane:
    frame = cq_box((0.32, 0.25, 0.008), (0.0, 0.0, 0.004))
    cutout = cq_box((0.264, 0.194, 0.01), (0.0, 0.0, 0.004))
    return frame.cut(cutout)


def build_scanner_lid_shape() -> cq.Workplane:
    lid_w, lid_d, lid_h = SCANNER_LID
    outer = cq_box((lid_w, lid_d, lid_h), (0.0, -lid_d / 2.0, lid_h / 2.0))
    inner = cq_box((lid_w - 0.022, lid_d - 0.022, lid_h - 0.014), (0.0, -lid_d / 2.0, 0.019))
    pull = cq_box((0.11, 0.018, 0.014), (0.0, -lid_d + 0.02, 0.018))
    rear_rib = cq_box((lid_w - 0.03, 0.03, 0.016), (0.0, -0.012, lid_h - 0.008))
    return outer.cut(inner).union(pull).union(rear_rib)


def build_drawer_shape() -> cq.Workplane:
    front = cq_box((0.406, 0.016, 0.154), (0.0, -0.008, 0.0))
    floor = cq_box((0.348, 0.298, 0.010), (0.0, 0.149, -0.055))
    left_wall = cq_box((0.018, 0.298, 0.106), (-0.165, 0.149, -0.007))
    right_wall = cq_box((0.018, 0.298, 0.106), (0.165, 0.149, -0.007))
    rear_wall = cq_box((0.348, 0.016, 0.106), (0.0, 0.290, -0.007))
    handle_recess = cq_box((0.16, 0.012, 0.036), (0.0, -0.010, 0.012))
    paper_lip = cq_box((0.300, 0.016, 0.022), (0.0, 0.287, -0.026))
    tray = front.union(floor).union(left_wall).union(right_wall).union(rear_wall).union(paper_lip)
    return tray.cut(handle_recess)


def build_service_door_shape() -> cq.Workplane:
    panel = cq_box((0.015, 0.24, 0.18), (0.0075, 0.12, 0.0))
    pull = cq_box((0.012, 0.05, 0.018), (0.014, 0.205, 0.0))
    inner_rib = cq_box((0.006, 0.18, 0.10), (0.005, 0.12, 0.0))
    return panel.union(pull).union(inner_rib)


def build_control_panel_shape() -> cq.Workplane:
    panel = cq_box((PANEL_W, PANEL_D, PANEL_H), (0.0, 0.0, PANEL_H / 2.0))
    pocket_w = 0.016
    pocket_d = 0.014
    pocket_h = 0.008
    for x in KEY_XS:
        for y in KEY_YS:
            pocket = cq_box((pocket_w, pocket_d, pocket_h), (x, y, PANEL_H - pocket_h / 2.0))
            panel = panel.cut(pocket)
    start_pocket = cq_box((0.018, 0.018, 0.010), (0.046, -0.034, PANEL_H - 0.005))
    panel = panel.cut(start_pocket)
    return panel


def key_center(col: int, row: int) -> tuple[float, float, float]:
    return (KEY_XS[col], KEY_YS[row], PANEL_H + 0.0005)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_aio_printer")

    dark = model.material("dark", rgba=(0.20, 0.20, 0.22, 1.0))
    mid = model.material("mid", rgba=(0.33, 0.34, 0.36, 1.0))
    light = model.material("light", rgba=(0.58, 0.58, 0.60, 1.0))
    panel_tone = model.material("panel_tone", rgba=(0.26, 0.27, 0.29, 1.0))
    button_tone = model.material("button_tone", rgba=(0.72, 0.72, 0.74, 1.0))
    accent = model.material("accent", rgba=(0.85, 0.85, 0.87, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.20, 0.22, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(build_body_shape(), "printer_body"), material=dark, name="shell")

    scanner_bed = model.part("scanner_bed")
    scanner_bed.visual(
        mesh_from_cadquery(build_scanner_bed_frame(), "scanner_bed_frame"),
        material=mid,
        name="frame",
    )
    scanner_bed.visual(
        Box((0.266, 0.196, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=glass,
        name="glass",
    )

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(
        mesh_from_cadquery(build_scanner_lid_shape(), "scanner_lid"),
        material=mid,
        name="panel",
    )

    paper_drawer = model.part("paper_drawer")
    paper_drawer.visual(
        mesh_from_cadquery(build_drawer_shape(), "paper_drawer"),
        material=mid,
        name="tray",
    )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(build_service_door_shape(), "service_door"),
        material=mid,
        name="door",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(build_control_panel_shape(), "control_panel"),
        material=panel_tone,
        name="deck",
    )

    menu_dial = model.part("menu_dial")
    menu_dial.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=light,
        name="cap",
    )
    menu_dial.visual(
        Box((0.004, 0.016, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.009)),
        material=accent,
        name="indicator",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.028, 0.028, 0.006)),
        material=accent,
        name="cap",
    )
    start_button.visual(
        Box((0.016, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=accent,
        name="stem",
    )

    key_parts: list[str] = []
    for row in range(4):
        for col in range(3):
            key_name = f"key_{row}_{col}"
            key = model.part(key_name)
            key.visual(
                Box(KEY_SIZE),
                material=button_tone,
                name="cap",
            )
            key.visual(
                Box((0.012, 0.010, 0.003)),
                origin=Origin(xyz=(0.0, 0.0, -0.004)),
                material=button_tone,
                name="stem",
            )
            key_parts.append(key_name)

    model.articulation(
        "body_to_scanner_bed",
        ArticulationType.FIXED,
        parent=body,
        child=scanner_bed,
        origin=Origin(xyz=(SCANNER_CENTER[0], SCANNER_CENTER[1], BODY_H)),
    )
    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=scanner_lid,
        origin=Origin(xyz=(SCANNER_CENTER[0], 0.19, BODY_H + 0.008)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.32),
    )
    model.articulation(
        "body_to_paper_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_drawer,
        origin=Origin(xyz=(0.0, -0.219, 0.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.28, lower=0.0, upper=0.18),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(BODY_W / 2.0, -0.12, 0.17)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=1.8),
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=PANEL_CENTER),
    )
    model.articulation(
        "control_panel_to_menu_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=menu_dial,
        origin=Origin(xyz=(0.047, 0.028, PANEL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    model.articulation(
        "control_panel_to_start_button",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=start_button,
        origin=Origin(xyz=(0.046, -0.034, PANEL_H + 0.001)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.05, lower=0.0, upper=KEY_TRAVEL),
    )

    for row in range(4):
        for col in range(3):
            key_name = f"key_{row}_{col}"
            x, y, z = key_center(col, row)
            model.articulation(
                f"control_panel_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=control_panel,
                child=key_name,
                origin=Origin(xyz=(x, y, z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=KEY_TRAVEL),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    scanner_bed = object_model.get_part("scanner_bed")
    scanner_lid = object_model.get_part("scanner_lid")
    paper_drawer = object_model.get_part("paper_drawer")
    service_door = object_model.get_part("service_door")
    control_panel = object_model.get_part("control_panel")
    menu_dial = object_model.get_part("menu_dial")
    start_button = object_model.get_part("start_button")
    key_0_0 = object_model.get_part("key_0_0")

    lid_joint = object_model.get_articulation("body_to_scanner_lid")
    drawer_joint = object_model.get_articulation("body_to_paper_drawer")
    door_joint = object_model.get_articulation("body_to_service_door")
    dial_joint = object_model.get_articulation("control_panel_to_menu_dial")
    start_joint = object_model.get_articulation("control_panel_to_start_button")
    key_joint = object_model.get_articulation("control_panel_to_key_0_0")

    key_names = [f"key_{row}_{col}" for row in range(4) for col in range(3)]
    ctx.check(
        "numeric keypad has twelve separate keys",
        all(object_model.get_part(name) is not None for name in key_names),
        details=str(key_names),
    )
    ctx.allow_isolated_part(
        paper_drawer,
        reason="The paper drawer is supported by hidden slide rails inside the lower cavity, which are intentionally omitted to keep the visible tray opening clear.",
    )

    ctx.expect_overlap(scanner_lid, scanner_bed, axes="xy", min_overlap=0.22, name="scanner lid covers scanner bed")
    ctx.expect_overlap(paper_drawer, body, axes="xz", min_overlap=0.10, name="paper drawer aligns with lower front body")
    ctx.expect_overlap(service_door, body, axes="yz", min_overlap=0.12, name="service door covers side opening")
    ctx.expect_within(key_0_0, control_panel, axes="xy", margin=0.006, name="top keypad key stays on control panel")
    ctx.expect_within(start_button, control_panel, axes="xy", margin=0.01, name="start button stays on control panel")
    ctx.expect_contact(menu_dial, control_panel, contact_tol=0.0005, name="menu dial seats on control panel")
    ctx.check(
        "menu dial is continuous",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=str(dial_joint.motion_limits),
    )

    closed_lid_aabb = ctx.part_world_aabb(scanner_lid)
    closed_drawer_pos = ctx.part_world_position(paper_drawer)
    closed_door_aabb = ctx.part_world_aabb(service_door)
    closed_key_pos = ctx.part_world_position(key_0_0)
    closed_start_pos = ctx.part_world_position(start_button)

    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    drawer_upper = drawer_joint.motion_limits.upper if drawer_joint.motion_limits is not None else None
    door_upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    key_upper = key_joint.motion_limits.upper if key_joint.motion_limits is not None else None
    start_upper = start_joint.motion_limits.upper if start_joint.motion_limits is not None else None

    if lid_upper is not None:
        with ctx.pose({lid_joint: lid_upper}):
            open_lid_aabb = ctx.part_world_aabb(scanner_lid)
        ctx.check(
            "scanner lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    if drawer_upper is not None:
        with ctx.pose({drawer_joint: drawer_upper}):
            open_drawer_pos = ctx.part_world_position(paper_drawer)
            ctx.expect_overlap(
                paper_drawer,
                body,
                axes="y",
                min_overlap=0.08,
                name="paper drawer stays inserted when extended",
            )
        ctx.check(
            "paper drawer extends forward",
            closed_drawer_pos is not None
            and open_drawer_pos is not None
            and open_drawer_pos[1] < closed_drawer_pos[1] - 0.12,
            details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
        )

    if door_upper is not None:
        with ctx.pose({door_joint: door_upper}):
            open_door_aabb = ctx.part_world_aabb(service_door)
        ctx.check(
            "service door swings outward",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.10,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    if key_upper is not None:
        with ctx.pose({key_joint: key_upper}):
            pressed_key_pos = ctx.part_world_position(key_0_0)
        ctx.check(
            "keypad key presses downward",
            closed_key_pos is not None
            and pressed_key_pos is not None
            and pressed_key_pos[2] < closed_key_pos[2] - 0.002,
            details=f"closed={closed_key_pos}, pressed={pressed_key_pos}",
        )

    if start_upper is not None:
        with ctx.pose({start_joint: start_upper}):
            pressed_start_pos = ctx.part_world_position(start_button)
        ctx.check(
            "start button presses downward",
            closed_start_pos is not None
            and pressed_start_pos is not None
            and pressed_start_pos[2] < closed_start_pos[2] - 0.002,
            details=f"closed={closed_start_pos}, pressed={pressed_start_pos}",
        )

    return ctx.report()


object_model = build_object_model()
