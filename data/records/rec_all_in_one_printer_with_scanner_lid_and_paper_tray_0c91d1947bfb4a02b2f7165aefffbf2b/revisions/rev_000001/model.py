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


PRINTER_W = 0.48
PRINTER_D = 0.41
BODY_H = 0.185
SCANNER_X = -0.075
SCANNER_Y = -0.005
SCANNER_W = 0.290
SCANNER_D = 0.225
SCANNER_RECESS = 0.014
LID_W = 0.332
LID_D = 0.300
LID_H = 0.026


def _box_solid(
    width: float,
    depth: float,
    height: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    base_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .translate((center_x, center_y, base_z))
    )


def _open_top_shell(
    width: float,
    depth: float,
    height: float,
    *,
    wall: float,
    bottom: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    base_z: float = 0.0,
) -> cq.Workplane:
    outer = _box_solid(width, depth, height, center_x=center_x, center_y=center_y, base_z=base_z)
    inner = _box_solid(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - bottom + 0.002,
        center_x=center_x,
        center_y=center_y,
        base_z=base_z + bottom,
    )
    return outer.cut(inner)


def _open_bottom_shell(
    width: float,
    depth: float,
    height: float,
    *,
    wall: float,
    top: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    base_z: float = 0.0,
) -> cq.Workplane:
    outer = _box_solid(width, depth, height, center_x=center_x, center_y=center_y, base_z=base_z)
    inner = _box_solid(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - top + 0.002,
        center_x=center_x,
        center_y=center_y,
        base_z=base_z - 0.001,
    )
    return outer.cut(inner)


def _make_body_shape() -> cq.Workplane:
    body = _box_solid(PRINTER_W, PRINTER_D, BODY_H)
    body = body.edges("|Z").fillet(0.010)
    body = body.edges(">Z").fillet(0.006)

    control_pod = _box_solid(0.088, 0.090, 0.020, center_x=0.184, center_y=-0.124, base_z=BODY_H)
    control_pod = control_pod.edges("|Z").fillet(0.004)
    body = body.union(control_pod)

    scanner_recess = _box_solid(
        SCANNER_W,
        SCANNER_D,
        SCANNER_RECESS + 0.002,
        center_x=SCANNER_X,
        center_y=SCANNER_Y,
        base_z=BODY_H - SCANNER_RECESS,
    )
    body = body.cut(scanner_recess)

    output_slot = _box_solid(
        0.248,
        0.070,
        0.012,
        center_x=-0.010,
        center_y=-PRINTER_D * 0.5 + 0.030,
        base_z=0.136,
    )
    body = body.cut(output_slot)

    tray_path = _box_solid(
        0.292,
        0.105,
        0.050,
        center_x=-0.010,
        center_y=-PRINTER_D * 0.5 + 0.0475,
        base_z=0.082,
    )
    body = body.cut(tray_path)

    cassette_cavity = _box_solid(
        0.398,
        0.300,
        0.062,
        center_x=0.000,
        center_y=-PRINTER_D * 0.5 + 0.145,
        base_z=0.010,
    )
    body = body.cut(cassette_cavity)

    handle_notch = _box_solid(
        0.100,
        0.022,
        0.016,
        center_x=0.000,
        center_y=-PRINTER_D * 0.5 + 0.006,
        base_z=0.030,
    )
    body = body.cut(handle_notch)
    return body


def _make_lid_shape() -> cq.Workplane:
    lid = _open_bottom_shell(LID_W, LID_D, LID_H, wall=0.010, top=0.004, center_y=-LID_D * 0.5)
    lower_adf = _open_top_shell(
        0.286,
        0.162,
        0.028,
        wall=0.006,
        bottom=0.004,
        center_x=0.000,
        center_y=-0.116,
        base_z=LID_H,
    )

    throat_front = _box_solid(
        0.210,
        0.024,
        0.013,
        center_x=0.000,
        center_y=-0.186,
        base_z=LID_H + 0.006,
    )
    throat_rear = _box_solid(
        0.188,
        0.018,
        0.010,
        center_x=0.000,
        center_y=-0.046,
        base_z=LID_H + 0.010,
    )
    lower_adf = lower_adf.cut(throat_front).cut(throat_rear)

    return lid.union(lower_adf)


def _make_adf_lid_shape() -> cq.Workplane:
    adf_lid = _open_bottom_shell(
        0.286,
        0.164,
        0.018,
        wall=0.006,
        top=0.0035,
        center_y=-0.082,
        base_z=0.000,
    )
    return adf_lid


def _make_cassette_shape() -> cq.Workplane:
    tray = _open_top_shell(0.386, 0.240, 0.056, wall=0.006, bottom=0.0035, center_y=0.120)
    fascia = _box_solid(0.404, 0.026, 0.066, center_y=0.008, base_z=0.000)
    notch = _box_solid(0.094, 0.014, 0.016, center_y=-0.002, base_z=0.028)
    return tray.union(fascia).cut(notch)


def _make_flap_shape() -> cq.Workplane:
    flap = _box_solid(0.286, 0.004, 0.056, center_y=-0.002, base_z=0.000)
    lip = _box_solid(0.248, 0.016, 0.006, center_y=-0.010, base_z=0.050)
    return flap.union(lip)


def _make_dial_shape() -> cq.Workplane:
    base = cq.Workplane("XY").cylinder(0.008, 0.017)
    cap = cq.Workplane("XY").cylinder(0.005, 0.012).translate((0.0, 0.0, 0.008))
    pointer = _box_solid(0.002, 0.010, 0.0015, center_y=0.008, base_z=0.016)
    return base.union(cap).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_all_in_one_printer")

    body_mat = model.material("body_light", rgba=(0.84, 0.85, 0.86, 1.0))
    panel_mat = model.material("panel_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    trim_mat = model.material("trim_mid", rgba=(0.34, 0.37, 0.40, 1.0))
    glass_mat = model.material("glass_blue", rgba=(0.42, 0.58, 0.70, 0.35))
    button_mat = model.material("button_blue", rgba=(0.22, 0.31, 0.42, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_make_body_shape(), "printer_body"), material=body_mat, name="shell")

    scanner_glass = model.part("scanner_glass")
    scanner_glass.visual(
        Box((SCANNER_W - 0.010, SCANNER_D - 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=glass_mat,
        name="glass",
    )
    model.articulation(
        "body_to_scanner_glass",
        ArticulationType.FIXED,
        parent=body,
        child=scanner_glass,
        origin=Origin(xyz=(SCANNER_X, SCANNER_Y, BODY_H - SCANNER_RECESS)),
    )

    lid_mount = model.part("lid_mount")
    lid_mount.visual(
        Box((0.296, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=trim_mat,
        name="hinge_pad",
    )
    model.articulation(
        "body_to_lid_mount",
        ArticulationType.FIXED,
        parent=body,
        child=lid_mount,
        origin=Origin(xyz=(SCANNER_X, SCANNER_Y + SCANNER_D * 0.5 - 0.009, BODY_H)),
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_make_lid_shape(), "printer_lid"), material=body_mat, name="lid_shell")
    lid_hinge_y = SCANNER_Y + SCANNER_D * 0.5
    lid_joint = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(SCANNER_X, lid_hinge_y, BODY_H + 0.003)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.24),
    )

    adf_lid = model.part("adf_lid")
    adf_lid.visual(mesh_from_cadquery(_make_adf_lid_shape(), "printer_adf_lid"), material=panel_mat, name="adf_shell")
    adf_joint = model.articulation(
        "lid_to_adf_lid",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=adf_lid,
        origin=Origin(xyz=(0.0, -0.035, LID_H + 0.028)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    cassette = model.part("cassette")
    cassette.visual(
        mesh_from_cadquery(_make_cassette_shape(), "printer_cassette"),
        material=panel_mat,
        name="tray_shell",
    )
    cassette_joint = model.articulation(
        "body_to_cassette",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette,
        origin=Origin(xyz=(0.0, -PRINTER_D * 0.5 - 0.020, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.130),
    )

    tray_flap = model.part("tray_flap")
    tray_flap.visual(
        mesh_from_cadquery(_make_flap_shape(), "printer_tray_flap"),
        material=trim_mat,
        name="flap_panel",
    )
    flap_joint = model.articulation(
        "body_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray_flap,
        origin=Origin(xyz=(-0.010, -PRINTER_D * 0.5, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.28),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=panel_mat,
        name="dial_knob",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.184, -0.112, BODY_H + 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0),
    )

    button_positions = (
        (0.156, -0.149),
        (0.184, -0.149),
        (0.212, -0.149),
        (0.170, -0.085),
        (0.198, -0.085),
    )
    for index, (button_x, button_y) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.014, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, button_y, BODY_H + 0.020)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.0025),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    adf_lid = object_model.get_part("adf_lid")
    scanner_glass = object_model.get_part("scanner_glass")
    cassette = object_model.get_part("cassette")
    tray_flap = object_model.get_part("tray_flap")
    dial = object_model.get_part("dial")

    lid_joint = object_model.get_articulation("body_to_lid")
    adf_joint = object_model.get_articulation("lid_to_adf_lid")
    cassette_joint = object_model.get_articulation("body_to_cassette")
    flap_joint = object_model.get_articulation("body_to_tray_flap")

    ctx.allow_overlap(
        body,
        cassette,
        elem_a="shell",
        elem_b="tray_shell",
        reason="The lower body is represented as a simplified monolithic housing proxy around the recessed cassette tunnel.",
    )

    ctx.expect_overlap(
        lid,
        scanner_glass,
        axes="xy",
        min_overlap=0.210,
        name="flatbed lid covers the scanner glass",
    )
    ctx.expect_gap(
        lid,
        scanner_glass,
        axis="z",
        min_gap=0.010,
        max_gap=0.018,
        name="flatbed lid sits just above the scanner bed",
    )
    ctx.expect_gap(
        adf_lid,
        lid,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        name="adf lid rests on the feeder frame",
    )

    lid_rest = _aabb_center(ctx.part_world_aabb(lid))
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_open = _aabb_center(ctx.part_world_aabb(lid))
    ctx.check(
        "flatbed lid opens upward",
        lid_rest is not None and lid_open is not None and lid_open[2] > lid_rest[2] + 0.10,
        details=f"rest={lid_rest}, open={lid_open}",
    )

    adf_rest = _aabb_center(ctx.part_world_aabb(adf_lid))
    with ctx.pose({adf_joint: adf_joint.motion_limits.upper}):
        adf_open = _aabb_center(ctx.part_world_aabb(adf_lid))
    ctx.check(
        "adf lid opens upward",
        adf_rest is not None and adf_open is not None and adf_open[2] > adf_rest[2] + 0.04,
        details=f"rest={adf_rest}, open={adf_open}",
    )

    cassette_rest = ctx.part_world_position(cassette)
    with ctx.pose({cassette_joint: cassette_joint.motion_limits.upper}):
        cassette_open = ctx.part_world_position(cassette)
        ctx.expect_overlap(
            cassette,
            body,
            axes="xz",
            min_overlap=0.050,
            name="extended cassette remains aligned with the lower body",
        )
        ctx.expect_overlap(
            cassette,
            body,
            axes="y",
            min_overlap=0.050,
            name="extended cassette keeps retained insertion",
        )
    ctx.check(
        "cassette slides outward",
        cassette_rest is not None and cassette_open is not None and cassette_open[1] < cassette_rest[1] - 0.10,
        details=f"rest={cassette_rest}, open={cassette_open}",
    )

    flap_rest = _aabb_center(ctx.part_world_aabb(tray_flap))
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        flap_open = _aabb_center(ctx.part_world_aabb(tray_flap))
    ctx.check(
        "multipurpose tray flap folds downward",
        flap_rest is not None
        and flap_open is not None
        and flap_open[1] < flap_rest[1] - 0.015
        and flap_open[2] < flap_rest[2] - 0.020,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    dial_pos = ctx.part_world_position(dial)
    ctx.check(
        "dial sits at the right front control pad",
        dial_pos is not None and dial_pos[0] > 0.10 and dial_pos[1] < -0.07 and dial_pos[2] > BODY_H,
        details=f"dial_pos={dial_pos}",
    )

    return ctx.report()


object_model = build_object_model()
