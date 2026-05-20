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


BODY_WIDTH = 0.360
BODY_DEPTH = 0.290
BODY_BASE_Z = 0.050
BODY_HEAD_Z = 0.670
BODY_OPENING_Z = 0.490
BIN_WIDTH = 0.304
BIN_DEPTH = 0.222
BIN_HEIGHT = 0.305
PANEL_ROLL = math.radians(70.0)
PANEL_CENTER = (0.0, -0.105, 0.548)


def _extrude_yz_profile(profile: list[tuple[float, float]], width: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(width)
        .translate((-width * 0.5, 0.0, 0.0))
    )


def _build_body_shell() -> cq.Workplane:
    outer_profile = [
        (-0.145, BODY_BASE_Z),
        (0.145, BODY_BASE_Z),
        (0.145, 0.585),
        (0.105, BODY_HEAD_Z),
        (-0.040, BODY_HEAD_Z),
        (-0.108, 0.585),
        (-0.130, BODY_OPENING_Z),
        (-0.145, BODY_OPENING_Z),
    ]
    inner_profile = [
        (-0.158, 0.088),
        (0.118, 0.088),
        (0.118, 0.620),
        (0.084, 0.637),
        (-0.014, 0.637),
        (-0.082, 0.555),
        (-0.101, BODY_OPENING_Z),
        (-0.158, BODY_OPENING_Z),
    ]

    shell = _extrude_yz_profile(outer_profile, BODY_WIDTH)
    cavity = _extrude_yz_profile(inner_profile, 0.312)
    body = shell.cut(cavity)

    feed_slot = (
        cq.Workplane("XY")
        .slot2D(0.255, 0.018)
        .extrude(0.085)
        .translate((0.0, -0.010, 0.585))
    )
    body = body.cut(feed_slot)

    caster_reliefs = []
    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.095, 0.095):
            caster_reliefs.append(
                cq.Workplane("XY")
                .circle(0.013)
                .extrude(0.030)
                .translate((x_pos, y_pos, BODY_BASE_Z - 0.002))
            )
    for relief in caster_reliefs:
        body = body.cut(relief)

    return body


def _build_bin_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT)
        .translate((0.0, BIN_DEPTH * 0.5, BIN_HEIGHT * 0.5))
        .faces(">Z")
        .shell(-0.004)
    )


def _build_cutter_drum(length: float = 0.272) -> cq.Workplane:
    drum = (
        cq.Workplane("YZ")
        .circle(0.009)
        .extrude(length)
        .translate((-length * 0.5, 0.0, 0.0))
    )
    disc_width = 0.008
    positions = [
        -0.110,
        -0.088,
        -0.066,
        -0.044,
        -0.022,
        0.000,
        0.022,
        0.044,
        0.066,
        0.088,
        0.110,
    ]
    for x_center in positions:
        disc = (
            cq.Workplane("YZ")
            .circle(0.017)
            .extrude(disc_width)
            .translate((x_center - disc_width * 0.5, 0.0, 0.0))
        )
        drum = drum.union(disc)
    return drum


def _panel_xyz(local_x: float, local_y: float, local_z: float) -> tuple[float, float, float]:
    cos_r = math.cos(PANEL_ROLL)
    sin_r = math.sin(PANEL_ROLL)
    return (
        PANEL_CENTER[0] + local_x,
        PANEL_CENTER[1] + local_y * cos_r - local_z * sin_r,
        PANEL_CENTER[2] + local_y * sin_r + local_z * cos_r,
    )


def _panel_origin(local_x: float, local_z: float, local_y: float = -0.002) -> Origin:
    return Origin(
        xyz=_panel_xyz(local_x, local_y, local_z),
        rpy=(PANEL_ROLL, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    body_charcoal = model.material("body_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    body_black = model.material("body_black", rgba=(0.09, 0.10, 0.11, 1.0))
    bin_black = model.material("bin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.020, BODY_DEPTH, BODY_OPENING_Z - BODY_BASE_Z)),
        origin=Origin(xyz=(-0.170, 0.000, 0.270)),
        material=body_charcoal,
        name="left_wall",
    )
    body.visual(
        Box((0.020, BODY_DEPTH, BODY_OPENING_Z - BODY_BASE_Z)),
        origin=Origin(xyz=(0.170, 0.000, 0.270)),
        material=body_charcoal,
        name="right_wall",
    )
    body.visual(
        Box((0.320, 0.020, BODY_OPENING_Z - BODY_BASE_Z)),
        origin=Origin(xyz=(0.000, 0.135, 0.270)),
        material=body_charcoal,
        name="rear_wall",
    )
    body.visual(
        Box((0.320, 0.250, 0.018)),
        origin=Origin(xyz=(0.000, 0.010, 0.059)),
        material=body_charcoal,
        name="floor_pan",
    )
    body.visual(
        Box((0.320, 0.190, 0.022)),
        origin=Origin(xyz=(0.000, -0.030, 0.501)),
        material=body_charcoal,
        name="cutter_bridge",
    )
    body.visual(
        Box((0.020, 0.215, 0.180)),
        origin=Origin(xyz=(-0.170, 0.020, 0.580)),
        material=body_charcoal,
        name="left_head_wall",
    )
    body.visual(
        Box((0.020, 0.215, 0.180)),
        origin=Origin(xyz=(0.170, 0.020, 0.580)),
        material=body_charcoal,
        name="right_head_wall",
    )
    body.visual(
        Box((0.320, 0.020, 0.180)),
        origin=Origin(xyz=(0.000, 0.135, 0.580)),
        material=body_charcoal,
        name="rear_head_wall",
    )
    body.visual(
        Box((0.320, 0.082, 0.018)),
        origin=Origin(xyz=(0.000, -0.079, 0.661)),
        material=body_charcoal,
        name="front_roof",
    )
    body.visual(
        Box((0.320, 0.093, 0.018)),
        origin=Origin(xyz=(0.000, 0.099, 0.661)),
        material=body_charcoal,
        name="rear_roof",
    )
    body.visual(
        Box((0.320, 0.055, 0.020)),
        origin=Origin(xyz=(0.000, -0.080, 0.505)),
        material=body_charcoal,
        name="front_panel_shell",
    )
    body.visual(
        Box((0.196, 0.004, 0.074)),
        origin=Origin(
            xyz=PANEL_CENTER,
            rpy=(PANEL_ROLL, 0.0, 0.0),
        ),
        material=body_black,
        name="panel_plate",
    )
    for brace_name, x_pos in (("panel_brace_0", -0.130), ("panel_brace_1", 0.130)):
        body.visual(
            Box((0.070, 0.030, 0.040)),
            origin=Origin(
                xyz=(x_pos, -0.092, 0.529),
                rpy=(PANEL_ROLL, 0.0, 0.0),
            ),
            material=body_charcoal,
            name=brace_name,
        )
    body.visual(
        Box((0.282, 0.060, 0.006)),
        origin=Origin(xyz=(0.000, -0.033, 0.661)),
        material=body_black,
        name="slot_bezel",
    )

    for x_pos in (-0.135, 0.135):
        for y_pos in (-0.095, 0.095):
            stem_name = f"caster_stem_{'n' if x_pos < 0 else 'p'}_{'n' if y_pos < 0 else 'p'}"
            top_name = f"caster_top_{'n' if x_pos < 0 else 'p'}_{'n' if y_pos < 0 else 'p'}"
            cheek_a_name = f"caster_cheek_a_{'n' if x_pos < 0 else 'p'}_{'n' if y_pos < 0 else 'p'}"
            cheek_b_name = f"caster_cheek_b_{'n' if x_pos < 0 else 'p'}_{'n' if y_pos < 0 else 'p'}"
            body.visual(
                Box((0.014, 0.014, 0.030)),
                origin=Origin(xyz=(x_pos, y_pos, 0.036)),
                material=body_black,
                name=stem_name,
            )
            body.visual(
                Box((0.034, 0.020, 0.006)),
                origin=Origin(xyz=(x_pos, y_pos, 0.024)),
                material=body_black,
                name=top_name,
            )
            body.visual(
                Box((0.004, 0.020, 0.024)),
                origin=Origin(xyz=(x_pos - 0.012, y_pos, 0.009)),
                material=body_black,
                name=cheek_a_name,
            )
            body.visual(
                Box((0.004, 0.020, 0.024)),
                origin=Origin(xyz=(x_pos + 0.012, y_pos, 0.009)),
                material=body_black,
                name=cheek_b_name,
            )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_build_bin_shell(), "shredder_bin"),
        material=bin_black,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.150, 0.012, 0.012)),
        origin=Origin(xyz=(0.000, -0.016, 0.214)),
        material=handle_black,
        name="handle_bar",
    )
    for x_pos in (-0.055, 0.055):
        bin_part.visual(
            Box((0.018, 0.020, 0.020)),
            origin=Origin(xyz=(x_pos, -0.006, 0.214)),
            material=handle_black,
            name=f"handle_post_{0 if x_pos < 0 else 1}",
        )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, -0.145, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=0.165,
        ),
    )

    drum_mesh = mesh_from_cadquery(_build_cutter_drum(), "shredder_cutter_drum")
    for part_name, y_pos in (("cutter_front", -0.020), ("cutter_rear", 0.020)):
        cutter = model.part(part_name)
        cutter.visual(
            drum_mesh,
            material=body_black,
            name="drum",
        )
        cutter.visual(
            Cylinder(radius=0.006, length=0.308),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=body_black,
            name="shaft",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=cutter,
            origin=Origin(xyz=(0.0, y_pos, 0.525)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    button_specs = (
        ("start_button", -0.056, 0.006, (0.24, 0.52, 0.24, 1.0)),
        ("stop_button", -0.024, 0.006, (0.68, 0.16, 0.16, 1.0)),
        ("reverse_button", 0.008, 0.006, (0.82, 0.68, 0.20, 1.0)),
    )
    for part_name, local_x, local_z, color in button_specs:
        button_material = model.material(f"{part_name}_material", rgba=color)
        button = model.part(part_name)
        button.visual(
            Box((0.022, 0.006, 0.016)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=button_material,
            name="cap",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=_panel_origin(local_x, local_z),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0015,
            ),
        )

    dial = model.part("selector_dial")
    dial.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="dial",
    )
    dial.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="shaft",
    )
    dial.visual(
        Box((0.003, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, 0.010)),
        material=body_charcoal,
        name="indicator",
    )
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=_panel_origin(0.060, 0.002),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    caster_positions = [
        (-0.135, -0.095),
        (-0.135, 0.095),
        (0.135, -0.095),
        (0.135, 0.095),
    ]
    for index, (x_pos, y_pos) in enumerate(caster_positions):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=body_black,
            name="wheel",
        )
        caster.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=handle_black,
            name="hub",
        )
        model.articulation(
            f"body_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, 0.000)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    bin_joint = object_model.get_articulation("body_to_bin")

    ctx.allow_overlap(
        body,
        "selector_dial",
        elem_a="panel_plate",
        elem_b="shaft",
        reason="The selector dial's shaft intentionally passes through the simplified panel plate, which omits a modeled bore.",
    )

    ctx.expect_within(
        bin_part,
        body,
        axes="xz",
        inner_elem="bin_shell",
        margin=0.022,
        name="bin stays centered in the cabinet opening",
    )
    ctx.expect_overlap(
        bin_part,
        body,
        axes="y",
        elem_a="bin_shell",
        min_overlap=0.115,
        name="closed bin remains inserted into the shredder body",
    )

    rest_pos = ctx.part_world_position(bin_part)
    limits = bin_joint.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({bin_joint: limits.upper}):
            ctx.expect_within(
                bin_part,
                body,
                axes="x",
                inner_elem="bin_shell",
                margin=0.022,
                name="extended bin stays laterally aligned with the cabinet opening",
            )
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                elem_a="bin_shell",
                min_overlap=0.055,
                name="extended bin retains visible insertion in the body",
            )
            extended_pos = ctx.part_world_position(bin_part)
        ctx.check(
            "bin pulls forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.12,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
