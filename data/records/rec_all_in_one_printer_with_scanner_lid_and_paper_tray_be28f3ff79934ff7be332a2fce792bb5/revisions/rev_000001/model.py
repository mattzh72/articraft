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


PRINTER_WIDTH = 0.440
BODY_DEPTH = 0.350
BODY_HEIGHT = 0.118
HINGE_AXIS_Y = BODY_DEPTH * 0.5 - 0.010
HINGE_AXIS_Z = BODY_HEIGHT + 0.006

SCANNER_OPEN_WIDTH = 0.314
SCANNER_OPEN_DEPTH = 0.192
SCANNER_OPEN_CENTER_Y = 0.047
SCANNER_WELL_DEPTH = 0.024

TRAY_WIDTH = 0.286
TRAY_DEPTH = 0.218
TRAY_HEIGHT = 0.044
TRAY_TRAVEL = 0.094
TRAY_OPENING_HEIGHT = 0.050

CONTROL_STRIP_WIDTH = 0.238
CONTROL_STRIP_DEPTH = 0.074
CONTROL_STRIP_THICKNESS = 0.012
CONTROL_STRIP_PITCH = math.radians(22.0)
CONTROL_STRIP_CENTER = (0.018, -0.122, 0.107)

LID_WIDTH = 0.430
LID_DEPTH = 0.232
LID_THICKNESS = 0.016
LID_HINGE_RADIUS = 0.006

SHELF_WIDTH = 0.246
SHELF_DEPTH = 0.066
SHELF_THICKNESS = 0.003


def _rotate_about_x(y_val: float, z_val: float, angle: float) -> tuple[float, float]:
    return (
        y_val * math.cos(angle) - z_val * math.sin(angle),
        y_val * math.sin(angle) + z_val * math.cos(angle),
    )


def _control_strip_surface_point(x_val: float, y_val: float, z_val: float = 0.0) -> tuple[float, float, float]:
    top_center_offset_y, top_center_offset_z = _rotate_about_x(
        0.0,
        CONTROL_STRIP_THICKNESS * 0.5 + z_val,
        CONTROL_STRIP_PITCH,
    )
    local_offset_y, local_offset_z = _rotate_about_x(y_val, 0.0, CONTROL_STRIP_PITCH)
    return (
        CONTROL_STRIP_CENTER[0] + x_val,
        CONTROL_STRIP_CENTER[1] + top_center_offset_y + local_offset_y,
        CONTROL_STRIP_CENTER[2] + top_center_offset_z + local_offset_z,
    )


def _control_strip_mount(x_val: float, y_val: float, z_val: float = 0.0) -> Origin:
    return Origin(
        xyz=_control_strip_surface_point(x_val, y_val, z_val),
        rpy=(CONTROL_STRIP_PITCH, 0.0, 0.0),
    )


def _printer_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(PRINTER_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.010)
    )

    scanner_well = (
        cq.Workplane("XY")
        .box(SCANNER_OPEN_WIDTH, SCANNER_OPEN_DEPTH, SCANNER_WELL_DEPTH + 0.002)
        .translate(
            (
                0.0,
                SCANNER_OPEN_CENTER_Y,
                BODY_HEIGHT - SCANNER_WELL_DEPTH * 0.5 + 0.001,
            )
        )
    )

    tray_cavity = (
        cq.Workplane("XY")
        .box(TRAY_WIDTH + 0.010, 0.150, TRAY_OPENING_HEIGHT)
        .translate((0.0, -BODY_DEPTH * 0.5 + 0.073, 0.034))
    )

    output_slot = (
        cq.Workplane("XY")
        .box(0.236, 0.090, 0.009)
        .translate((0.0, -BODY_DEPTH * 0.5 + 0.040, 0.079))
    )

    return body.cut(scanner_well).cut(tray_cavity).cut(output_slot)


def _scanner_lid_shape() -> cq.Workplane:
    rear_clearance = 0.010
    lid = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_THICKNESS)
        .translate(
            (
                0.0,
                -LID_DEPTH * 0.5 - rear_clearance,
                LID_THICKNESS * 0.5 - LID_HINGE_RADIUS,
            )
        )
        .edges("|Z")
        .fillet(0.005)
    )
    inner_pocket = (
        cq.Workplane("XY")
        .box(LID_WIDTH - 0.026, LID_DEPTH - 0.030, LID_THICKNESS - 0.004)
        .translate((0.0, -LID_DEPTH * 0.5 - rear_clearance - 0.004, -0.002))
    )
    return lid.cut(inner_pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_all_in_one_printer")

    body_dark = model.material("body_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    body_mid = model.material("body_mid", rgba=(0.32, 0.34, 0.36, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    glass = model.material("glass", rgba=(0.53, 0.63, 0.69, 0.28))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_printer_body_shape(), "printer_body"),
        material=body_dark,
        name="shell",
    )
    body.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, CONTROL_STRIP_THICKNESS)),
        origin=Origin(
            xyz=CONTROL_STRIP_CENTER,
            rpy=(CONTROL_STRIP_PITCH, 0.0, 0.0),
        ),
        material=body_mid,
        name="control_panel",
    )
    body.visual(
        Box((0.060, 0.032, 0.003)),
        origin=Origin(
            xyz=(-0.012, -0.120, 0.119),
            rpy=(CONTROL_STRIP_PITCH, 0.0, 0.0),
        ),
        material=trim_dark,
        name="screen_bezel",
    )
    body.visual(
        Box((0.046, 0.020, 0.002)),
        origin=Origin(
            xyz=_control_strip_surface_point(-0.012, 0.004, 0.0022),
            rpy=(CONTROL_STRIP_PITCH, 0.0, 0.0),
        ),
        material=glass,
        name="display_glass",
    )
    body.visual(
        Box((0.250, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + 0.012, 0.083)),
        material=trim_dark,
        name="output_mouth",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(
            xyz=_control_strip_surface_point(0.082, 0.006, 0.002),
            rpy=(CONTROL_STRIP_PITCH, 0.0, 0.0),
        ),
        material=trim_dark,
        name="dial_base",
    )
    for index, x_pos in enumerate((-0.165, 0.165)):
        body.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(
                xyz=(x_pos, HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=trim_dark,
            name=f"hinge_barrel_{index}",
        )

    scanner_platen = model.part("scanner_platen")
    scanner_platen.visual(
        Box((SCANNER_OPEN_WIDTH - 0.028, SCANNER_OPEN_DEPTH - 0.028, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=glass,
        name="glass",
    )
    model.articulation(
        "body_to_scanner_platen",
        ArticulationType.FIXED,
        parent=body,
        child=scanner_platen,
        origin=Origin(
            xyz=(
                0.0,
                SCANNER_OPEN_CENTER_Y,
                BODY_HEIGHT - SCANNER_WELL_DEPTH,
            )
        ),
    )

    lid = model.part("scanner_lid")
    lid.visual(
        mesh_from_cadquery(_scanner_lid_shape(), "scanner_lid"),
        material=body_mid,
        name="lid_shell",
    )
    lid.visual(
        Box((0.150, 0.010, 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                -LID_DEPTH + 0.010,
                0.002,
            )
        ),
        material=trim_dark,
        name="front_grip",
    )
    for index, x_pos in enumerate((-0.118, 0.118)):
        lid.visual(
            Cylinder(radius=0.005, length=0.048),
            origin=Origin(
                xyz=(x_pos, -0.005, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=trim_dark,
            name=f"lid_barrel_{index}",
        )
    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=1.32,
        ),
    )

    tray = model.part("paper_tray")
    tray_wall = 0.003
    floor_thickness = 0.0025
    tray.visual(
        Box((TRAY_WIDTH, TRAY_DEPTH, floor_thickness)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH * 0.5, floor_thickness * 0.5)),
        material=body_mid,
        name="tray_floor",
    )
    tray.visual(
        Box((TRAY_WIDTH, tray_wall, TRAY_HEIGHT)),
        origin=Origin(xyz=(0.0, tray_wall * 0.5, TRAY_HEIGHT * 0.5)),
        material=body_mid,
        name="tray_face",
    )
    tray.visual(
        Box((tray_wall, TRAY_DEPTH, TRAY_HEIGHT)),
        origin=Origin(
            xyz=((TRAY_WIDTH - tray_wall) * 0.5, TRAY_DEPTH * 0.5, TRAY_HEIGHT * 0.5)
        ),
        material=body_mid,
        name="tray_side_0",
    )
    tray.visual(
        Box((tray_wall, TRAY_DEPTH, TRAY_HEIGHT)),
        origin=Origin(
            xyz=(-(TRAY_WIDTH - tray_wall) * 0.5, TRAY_DEPTH * 0.5, TRAY_HEIGHT * 0.5)
        ),
        material=body_mid,
        name="tray_side_1",
    )
    tray.visual(
        Box((TRAY_WIDTH, tray_wall, 0.020)),
        origin=Origin(xyz=(0.0, TRAY_DEPTH - tray_wall * 0.5, 0.010)),
        material=trim_dark,
        name="paper_stop",
    )
    model.articulation(
        "body_to_paper_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    shelf = model.part("output_shelf")
    shelf.visual(
        Box((SHELF_WIDTH, SHELF_DEPTH, SHELF_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -SHELF_DEPTH * 0.5, SHELF_THICKNESS * 0.5),
        ),
        material=body_mid,
        name="shelf_panel",
    )
    shelf.visual(
        Box((SHELF_WIDTH, 0.003, 0.011)),
        origin=Origin(xyz=(0.0, -SHELF_DEPTH + 0.0015, 0.0055)),
        material=body_mid,
        name="front_lip",
    )
    shelf.visual(
        Box((0.003, SHELF_DEPTH - 0.006, 0.011)),
        origin=Origin(
            xyz=((SHELF_WIDTH - 0.003) * 0.5, -SHELF_DEPTH * 0.5 - 0.0015, 0.0055)
        ),
        material=body_mid,
        name="side_rail_0",
    )
    shelf.visual(
        Box((0.003, SHELF_DEPTH - 0.006, 0.011)),
        origin=Origin(
            xyz=(-(SHELF_WIDTH - 0.003) * 0.5, -SHELF_DEPTH * 0.5 - 0.0015, 0.0055)
        ),
        material=body_mid,
        name="side_rail_1",
    )
    model.articulation(
        "body_to_output_shelf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shelf,
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + 0.008, 0.069)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.6,
            lower=0.0,
            upper=1.24,
        ),
    )

    key_specs = (
        ("key_0", -0.088, 0.016),
        ("key_1", -0.088, -0.006),
        ("key_2", 0.034, 0.016),
        ("key_3", 0.034, -0.006),
        ("key_4", -0.034, -0.026),
        ("key_5", 0.010, -0.026),
    )
    for key_name, x_pos, y_pos in key_specs:
        key = model.part(key_name)
        key.visual(
            Box((0.010, 0.006, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0005)),
            material=trim_dark,
            name="key_plunger",
        )
        key.visual(
            Box((0.018, 0.010, 0.0035)),
            origin=Origin(xyz=(0.0, 0.0, 0.00275)),
            material=body_mid,
            name="key_cap",
        )
        model.articulation(
            f"body_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=_control_strip_mount(x_pos, y_pos),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=0.03,
                lower=0.0,
                upper=0.0012,
            ),
        )

    dial = model.part("menu_dial")
    dial.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=trim_dark,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=body_mid,
        name="dial_cap",
    )
    dial.visual(
        Box((0.002, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.006, 0.009)),
        material=body_mid,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_menu_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=_control_strip_mount(0.082, 0.006, 0.004),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=5.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("scanner_lid")
    tray = object_model.get_part("paper_tray")
    shelf = object_model.get_part("output_shelf")
    key_0 = object_model.get_part("key_0")
    dial = object_model.get_part("menu_dial")

    lid_hinge = object_model.get_articulation("body_to_scanner_lid")
    tray_slide = object_model.get_articulation("body_to_paper_tray")
    shelf_hinge = object_model.get_articulation("body_to_output_shelf")
    key_0_slide = object_model.get_articulation("body_to_key_0")
    dial_joint = object_model.get_articulation("body_to_menu_dial")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.180,
        name="scanner lid covers the scan bed footprint when closed",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        min_overlap=0.240,
        name="paper tray stays laterally engaged with the body",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "scanner lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.120,
        details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.240,
            name="extended tray remains guided by the body width",
        )
    ctx.check(
        "paper tray extends forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] < tray_rest[1] - 0.060,
        details=f"rest={tray_rest!r}, extended={tray_extended!r}",
    )

    open_shelf_aabb = ctx.part_world_aabb(shelf)
    with ctx.pose({shelf_hinge: shelf_hinge.motion_limits.upper}):
        closed_shelf_aabb = ctx.part_world_aabb(shelf)
    ctx.check(
        "output shelf folds upward",
        open_shelf_aabb is not None
        and closed_shelf_aabb is not None
        and closed_shelf_aabb[1][2] > open_shelf_aabb[1][2] + 0.045
        and closed_shelf_aabb[0][1] > open_shelf_aabb[0][1] + 0.020,
        details=f"open={open_shelf_aabb!r}, closed={closed_shelf_aabb!r}",
    )

    key_rest = ctx.part_world_position(key_0)
    with ctx.pose({key_0_slide: key_0_slide.motion_limits.upper}):
        key_pressed = ctx.part_world_position(key_0)
    ctx.check(
        "control key depresses into the strip",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.0003,
        details=f"rest={key_rest!r}, pressed={key_pressed!r}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.0}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "menu dial rotates about its center axis",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None
        and dial_rest is not None
        and dial_turned is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(dial_rest, dial_turned)),
        details=f"rest={dial_rest!r}, turned={dial_turned!r}",
    )

    return ctx.report()


object_model = build_object_model()
