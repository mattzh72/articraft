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


BODY_W = 0.445
BODY_D = 0.368
BODY_H = 0.142

LID_W = 0.338
LID_D = 0.230
LID_H = 0.026
LID_WALL = 0.004

CASSETTE_W = 0.330
CASSETTE_D = 0.290
CASSETTE_H = 0.052
CASSETTE_TRAVEL = 0.155

FEEDER_W = 0.288
FEEDER_D = 0.150
FEEDER_OPEN = 1.32

CONTROL_ANGLE = 0.0
CONTROL_CENTER_Y = -0.141
CONTROL_W = 0.348
CONTROL_D = 0.066
CONTROL_T = 0.016
CONTROL_CENTER_Z = BODY_H - (CONTROL_T * 0.5)

BUTTON_TRAVEL = 0.0035


def _build_body_mesh():
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H * 0.5))
        .edges("|Z")
        .fillet(0.016)
    )

    body = body.cut(
        cq.Workplane("XY")
        .box(0.397, 0.318, 0.108)
        .translate((0.0, 0.0, 0.064))
    )

    body = body.cut(
        cq.Workplane("XY")
        .box(0.344, 0.060, 0.058)
        .translate((0.0, -0.154, 0.039))
    )

    body = body.cut(
        cq.Workplane("XY")
        .box(0.285, 0.050, 0.036)
        .translate((0.0, -0.159, 0.086))
    )

    body = body.cut(
        cq.Workplane("XY")
        .box(0.252, 0.032, 0.016)
        .translate((0.0, 0.168, 0.124))
    )

    body = body.cut(
        cq.Workplane("XY")
        .box(0.316, 0.224, 0.030)
        .translate((0.0, 0.008, 0.127))
    )

    return body


def _control_mount_origin(local_x: float, local_y: float) -> Origin:
    top_z = CONTROL_T * 0.5
    cos_a = math.cos(CONTROL_ANGLE)
    sin_a = math.sin(CONTROL_ANGLE)
    world_y = CONTROL_CENTER_Y + local_y * cos_a - top_z * sin_a
    world_z = CONTROL_CENTER_Z + local_y * sin_a + top_z * cos_a
    return Origin(
        xyz=(local_x, world_y, world_z),
        rpy=(CONTROL_ANGLE, 0.0, 0.0),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ink_tank_printer")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.47, 0.60, 0.66, 0.32))
    paper = model.material("paper", rgba=(0.87, 0.88, 0.86, 1.0))
    button_dark = model.material("button_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_body_mesh(), "printer_body"),
        material=body_dark,
        name="body_shell",
    )
    base.visual(
        Box((CONTROL_W, CONTROL_D, CONTROL_T)),
        origin=Origin(
            xyz=(0.0, CONTROL_CENTER_Y, CONTROL_CENTER_Z),
            rpy=(CONTROL_ANGLE, 0.0, 0.0),
        ),
        material=panel_dark,
        name="control_strip",
    )
    base.visual(
        Box((0.290, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, -0.166, 0.078)),
        material=trim_dark,
        name="output_lip",
    )

    scanner_bed = model.part("scanner_bed")
    scanner_bed.visual(
        Box((0.326, 0.234, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_dark,
        name="scanner_bezel",
    )
    scanner_bed.visual(
        Box((0.308, 0.216, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=glass,
        name="scanner_glass",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_WALL)),
        origin=Origin(xyz=(0.0, -LID_D * 0.5, LID_H - LID_WALL * 0.5)),
        material=body_dark,
        name="lid_skin",
    )
    lid.visual(
        Box((LID_WALL, LID_D, LID_H)),
        origin=Origin(xyz=(-LID_W * 0.5 + LID_WALL * 0.5, -LID_D * 0.5, LID_H * 0.5)),
        material=body_dark,
        name="lid_side_0",
    )
    lid.visual(
        Box((LID_WALL, LID_D, LID_H)),
        origin=Origin(xyz=(LID_W * 0.5 - LID_WALL * 0.5, -LID_D * 0.5, LID_H * 0.5)),
        material=body_dark,
        name="lid_side_1",
    )
    lid.visual(
        Box((LID_W, LID_WALL, LID_H)),
        origin=Origin(xyz=(0.0, -LID_D + LID_WALL * 0.5, LID_H * 0.5)),
        material=body_dark,
        name="lid_front",
    )
    lid.visual(
        Box((LID_W, LID_WALL, LID_H)),
        origin=Origin(xyz=(0.0, -LID_WALL * 0.5, LID_H * 0.5)),
        material=body_dark,
        name="lid_rear",
    )
    lid.visual(
        Box((LID_W - 2.0 * LID_WALL, LID_D - 2.0 * LID_WALL, 0.003)),
        origin=Origin(xyz=(0.0, -LID_D * 0.5, 0.0045)),
        material=paper,
        name="platen_pad",
    )

    cassette = model.part("cassette")
    cassette.visual(
        Box((CASSETTE_W, CASSETTE_D - 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.154, 0.004)),
        material=trim_dark,
        name="cassette_floor",
    )
    cassette.visual(
        Box((CASSETTE_W, 0.018, CASSETTE_H)),
        origin=Origin(xyz=(0.0, 0.009, CASSETTE_H * 0.5)),
        material=body_dark,
        name="cassette_face",
    )
    cassette.visual(
        Box((0.004, CASSETTE_D - 0.018, CASSETTE_H)),
        origin=Origin(
            xyz=(-CASSETTE_W * 0.5 + 0.002, 0.154, CASSETTE_H * 0.5),
        ),
        material=body_dark,
        name="cassette_side_0",
    )
    cassette.visual(
        Box((0.004, CASSETTE_D - 0.018, CASSETTE_H)),
        origin=Origin(
            xyz=(CASSETTE_W * 0.5 - 0.002, 0.154, CASSETTE_H * 0.5),
        ),
        material=body_dark,
        name="cassette_side_1",
    )
    cassette.visual(
        Box((CASSETTE_W, 0.004, CASSETTE_H)),
        origin=Origin(xyz=(0.0, CASSETTE_D - 0.002, CASSETTE_H * 0.5)),
        material=body_dark,
        name="cassette_back",
    )
    cassette.visual(
        Box((0.120, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.012, 0.026)),
        material=trim_dark,
        name="cassette_handle",
    )

    feeder = model.part("feeder")
    feeder.visual(
        Box((FEEDER_W, FEEDER_D, 0.004)),
        origin=Origin(xyz=(0.0, -FEEDER_D * 0.5, 0.002)),
        material=trim_dark,
        name="feeder_panel",
    )
    feeder.visual(
        Box((0.008, FEEDER_D, 0.020)),
        origin=Origin(xyz=(-FEEDER_W * 0.5 + 0.004, -FEEDER_D * 0.5, 0.010)),
        material=body_dark,
        name="feeder_side_0",
    )
    feeder.visual(
        Box((0.008, FEEDER_D, 0.020)),
        origin=Origin(xyz=(FEEDER_W * 0.5 - 0.004, -FEEDER_D * 0.5, 0.010)),
        material=body_dark,
        name="feeder_side_1",
    )
    feeder.visual(
        Box((0.180, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, -FEEDER_D + 0.004, 0.011)),
        material=body_dark,
        name="feeder_stop",
    )
    feeder.visual(
        Box((0.200, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, 0.005)),
        material=body_dark,
        name="feeder_spine",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.018, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=dial_dark,
        name="dial_knob",
    )
    dial.visual(
        Box((0.003, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, 0.009, 0.012)),
        material=glass,
        name="dial_marker",
    )

    button_positions = (-0.020, 0.020, 0.060, 0.100)
    for index, local_x in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.026, 0.016, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=button_dark,
            name="button_cap",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=_control_mount_origin(local_x, 0.004),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.124, BODY_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=1.34,
        ),
    )
    model.articulation(
        "base_to_cassette",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cassette,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 - 0.018, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=CASSETTE_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_feeder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=feeder,
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.018, BODY_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=FEEDER_OPEN,
        ),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=_control_mount_origin(-0.110, 0.002),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )
    model.articulation(
        "base_to_scanner_bed",
        ArticulationType.FIXED,
        parent=base,
        child=scanner_bed,
        origin=Origin(xyz=(0.0, 0.008, BODY_H - 0.004)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cassette = object_model.get_part("cassette")
    feeder = object_model.get_part("feeder")
    button_0 = object_model.get_part("button_0")

    lid_hinge = object_model.get_articulation("base_to_lid")
    cassette_slide = object_model.get_articulation("base_to_cassette")
    feeder_hinge = object_model.get_articulation("base_to_feeder")
    button_joint = object_model.get_articulation("base_to_button_0")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        name="scanner lid seats on the printer top",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.20,
        name="scanner lid covers the scanner opening",
    )
    ctx.expect_overlap(
        cassette,
        base,
        axes="xz",
        min_overlap=0.04,
        name="cassette stays aligned with the chassis opening",
    )
    ctx.expect_overlap(
        cassette,
        base,
        axes="y",
        min_overlap=0.08,
        name="cassette remains inserted at rest",
    )

    closed_lid_front = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_front"))
    closed_feeder = _aabb_center(ctx.part_element_world_aabb(feeder, elem="feeder_panel"))
    rest_cassette_pos = ctx.part_world_position(cassette)
    rest_button_pos = ctx.part_world_position(button_0)

    with ctx.pose({lid_hinge: 1.10}):
        open_lid_front = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_front"))
    ctx.check(
        "scanner lid front edge lifts upward",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[2] > closed_lid_front[2] + 0.10
        and open_lid_front[1] > closed_lid_front[1] + 0.04,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    with ctx.pose({cassette_slide: CASSETTE_TRAVEL}):
        extended_cassette_pos = ctx.part_world_position(cassette)
        ctx.expect_overlap(
            cassette,
            base,
            axes="y",
            min_overlap=0.06,
            name="cassette keeps retained insertion when extended",
        )
    ctx.check(
        "cassette slides outward from the front",
        rest_cassette_pos is not None
        and extended_cassette_pos is not None
        and extended_cassette_pos[1] < rest_cassette_pos[1] - 0.10,
        details=f"rest={rest_cassette_pos}, extended={extended_cassette_pos}",
    )

    with ctx.pose({feeder_hinge: FEEDER_OPEN}):
        open_feeder = _aabb_center(ctx.part_element_world_aabb(feeder, elem="feeder_panel"))
    ctx.check(
        "rear feeder swings upward behind the scanner bed",
        closed_feeder is not None
        and open_feeder is not None
        and open_feeder[2] > closed_feeder[2] + 0.06
        and open_feeder[1] > closed_feeder[1] + 0.02,
        details=f"closed={closed_feeder}, open={open_feeder}",
    )

    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed_button_pos = ctx.part_world_position(button_0)
    ctx.check(
        "front control button presses into the sloped strip",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
