from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibrated_stick_vacuum")

    graphite = model.material("graphite_polymer", color=(0.055, 0.058, 0.064, 1.0))
    satin = model.material("satin_black", color=(0.010, 0.012, 0.014, 1.0))
    rubber = model.material("black_rubber", color=(0.018, 0.016, 0.014, 1.0))
    datum = model.material("datum_gray", color=(0.62, 0.66, 0.66, 1.0))
    mark = model.material("etched_white", color=(0.92, 0.95, 0.92, 1.0))
    blue = model.material("calibration_blue", color=(0.08, 0.45, 0.82, 1.0))
    steel = model.material("brushed_steel", color=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    body_shell = mesh_from_geometry(CapsuleGeometry(0.055, 0.380, radial_segments=32), "body_shell")
    body.visual(
        body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=graphite,
        name="body_shell",
    )
    body.visual(
        Box((0.070, 0.090, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=graphite,
        name="fold_socket",
    )
    body.visual(
        Box((0.050, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, 0.043, 0.000)),
        material=satin,
        name="fold_cheek_0",
    )
    body.visual(
        Box((0.050, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, -0.043, 0.000)),
        material=satin,
        name="fold_cheek_1",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.104),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin",
    )
    body.visual(
        Box((0.048, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, -0.0575, 0.016)),
        material=satin,
        name="dial_mount_web",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, -0.065, 0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="dial_boss",
    )
    body.visual(
        Box((0.035, 0.050, 0.150)),
        origin=Origin(xyz=(0.082, 0.0, 0.235)),
        material=satin,
        name="handle_grip",
    )
    body.visual(
        Box((0.090, 0.040, 0.034)),
        origin=Origin(xyz=(0.045, 0.0, 0.310)),
        material=satin,
        name="handle_bridge_top",
    )
    body.visual(
        Box((0.095, 0.042, 0.044)),
        origin=Origin(xyz=(0.048, 0.0, 0.150)),
        material=satin,
        name="handle_bridge_low",
    )
    body.visual(
        Box((0.080, 0.060, 0.090)),
        origin=Origin(xyz=(0.058, 0.0, 0.105)),
        material=rubber,
        name="battery_pack",
    )
    for i, x in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        body.visual(
            Box((0.003, 0.0022, 0.020 if i == 2 else 0.013)),
            origin=Origin(xyz=(x, 0.0450, 0.075)),
            material=mark if i == 2 else datum,
            name=f"fold_index_{i}",
        )

    alignment_dial = model.part("alignment_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.014,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=18, depth=0.0007),
            center=False,
        ),
        "alignment_dial_cap",
    )
    alignment_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="dial_cap",
    )
    alignment_dial.visual(
        Box((0.020, 0.002, 0.004)),
        origin=Origin(xyz=(0.005, -0.0144, 0.0)),
        material=blue,
        name="dial_pointer",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.032, 0.034, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=graphite,
        name="upper_socket",
    )
    wand.visual(
        Box((0.038, 0.026, 0.820)),
        origin=Origin(xyz=(0.0, 0.0, -0.450)),
        material=graphite,
        name="wand_tube",
    )
    wand.visual(
        Box((0.004, 0.028, 0.730)),
        origin=Origin(xyz=(-0.0208, 0.0, -0.475)),
        material=datum,
        name="straight_datum_rail",
    )
    for i, z in enumerate((-0.150, -0.172, -0.194, -0.216)):
        wand.visual(
            Box((0.003, 0.029, 0.006)),
            origin=Origin(xyz=(-0.0214, 0.0, z)),
            material=mark if i == 0 else datum,
            name=f"wand_index_{i}",
        )
    wand.visual(
        Box((0.042, 0.034, 0.052)),
        origin=Origin(xyz=(-0.004, 0.0, -0.858)),
        material=graphite,
        name="lower_socket",
    )
    wand.visual(
        Box((0.052, 0.014, 0.086)),
        origin=Origin(xyz=(-0.006, 0.043, -0.910)),
        material=satin,
        name="nozzle_cheek_0",
    )
    wand.visual(
        Box((0.052, 0.014, 0.086)),
        origin=Origin(xyz=(-0.006, -0.043, -0.910)),
        material=satin,
        name="nozzle_cheek_1",
    )
    wand.visual(
        Box((0.042, 0.090, 0.030)),
        origin=Origin(xyz=(-0.012, 0.0, -0.862)),
        material=satin,
        name="nozzle_fork_bridge",
    )
    wand.visual(
        Cylinder(radius=0.0085, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, -0.910), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nozzle_pin",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.016, length=0.064),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="nozzle_trunnion",
    )
    floor_head.visual(
        Box((0.044, 0.050, 0.056)),
        origin=Origin(xyz=(0.008, 0.0, -0.038)),
        material=graphite,
        name="trunnion_neck",
    )
    nozzle_shell = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.280, 0.290, 0.026), 0.050, center=True),
        "floor_head_shell",
    )
    floor_head.visual(
        nozzle_shell,
        origin=Origin(xyz=(0.125, 0.0, -0.085)),
        material=graphite,
        name="floor_head_shell",
    )
    floor_head.visual(
        Box((0.125, 0.050, 0.004)),
        origin=Origin(xyz=(0.145, 0.0, -0.0592)),
        material=datum,
        name="top_datum_pad",
    )
    floor_head.visual(
        Box((0.060, 0.032, 0.004)),
        origin=Origin(xyz=(0.225, 0.0, -0.0592)),
        material=blue,
        name="front_datum_pad",
    )
    floor_head.visual(
        Cylinder(radius=0.032, length=0.003),
        origin=Origin(xyz=(0.125, -0.085, -0.0585)),
        material=satin,
        name="height_dial_pad",
    )
    floor_head.visual(
        Box((0.235, 0.014, 0.012)),
        origin=Origin(xyz=(0.128, 0.119, -0.114)),
        material=rubber,
        name="skid_rail_0",
    )
    floor_head.visual(
        Box((0.235, 0.014, 0.012)),
        origin=Origin(xyz=(0.128, -0.119, -0.114)),
        material=rubber,
        name="skid_rail_1",
    )
    floor_head.visual(
        Box((0.018, 0.235, 0.026)),
        origin=Origin(xyz=(0.260, 0.0, -0.094)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Cylinder(radius=0.015, length=0.235),
        origin=Origin(xyz=(0.184, 0.0, -0.112), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="brush_roll",
    )
    for i, y in enumerate((-0.118, -0.094, -0.070, -0.046)):
        floor_head.visual(
            Box((0.018 if i == 0 else 0.012, 0.003, 0.002)),
            origin=Origin(xyz=(0.125, y, -0.0595)),
            material=mark if i == 0 else datum,
            name=f"height_index_{i}",
        )

    height_dial = model.part("height_dial")
    height_mesh = mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.014,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=20, depth=0.0008),
            center=False,
        ),
        "height_dial_cap",
    )
    height_dial.visual(
        height_mesh,
        material=satin,
        name="height_cap",
    )
    height_dial.visual(
        Box((0.024, 0.004, 0.002)),
        origin=Origin(xyz=(0.007, 0.0, 0.0130)),
        material=blue,
        name="height_pointer",
    )

    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "body_to_alignment_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=alignment_dial,
        origin=Origin(xyz=(0.0, -0.070, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.910)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "floor_head_to_height_dial",
        ArticulationType.REVOLUTE,
        parent=floor_head,
        child=height_dial,
        origin=Origin(xyz=(0.125, -0.085, -0.057)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=1.5, lower=-2.2, upper=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    alignment_dial = object_model.get_part("alignment_dial")
    height_dial = object_model.get_part("height_dial")
    fold = object_model.get_articulation("body_to_wand")
    nozzle = object_model.get_articulation("wand_to_floor_head")

    ctx.allow_overlap(
        body,
        wand,
        elem_a="fold_pin",
        elem_b="fold_barrel",
        reason="The visible hinge pin is intentionally captured inside the wand fold barrel.",
    )
    ctx.expect_within(
        body,
        wand,
        axes="xz",
        inner_elem="fold_pin",
        outer_elem="fold_barrel",
        margin=0.001,
        name="fold pin concentric with barrel",
    )
    ctx.expect_overlap(
        body,
        wand,
        axes="y",
        elem_a="fold_pin",
        elem_b="fold_barrel",
        min_overlap=0.050,
        name="fold pin spans barrel",
    )

    ctx.allow_overlap(
        wand,
        floor_head,
        elem_a="nozzle_pin",
        elem_b="nozzle_trunnion",
        reason="The nozzle pivot pin is intentionally captured inside the trunnion barrel.",
    )
    ctx.expect_within(
        wand,
        floor_head,
        axes="xz",
        inner_elem="nozzle_pin",
        outer_elem="nozzle_trunnion",
        margin=0.001,
        name="nozzle pin concentric with trunnion",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="y",
        elem_a="nozzle_pin",
        elem_b="nozzle_trunnion",
        min_overlap=0.052,
        name="nozzle pin spans trunnion",
    )

    ctx.expect_contact(
        alignment_dial,
        body,
        elem_a="dial_cap",
        elem_b="dial_boss",
        contact_tol=0.001,
        name="alignment dial seats on boss",
    )
    ctx.expect_contact(
        height_dial,
        floor_head,
        elem_a="height_cap",
        elem_b="height_dial_pad",
        contact_tol=0.001,
        name="height dial seats on pad",
    )

    rest_lower = ctx.part_element_world_aabb(wand, elem="lower_socket")
    with ctx.pose({fold: 1.35}):
        folded_lower = ctx.part_element_world_aabb(wand, elem="lower_socket")
    ctx.check(
        "fold joint swings wand clear",
        rest_lower is not None
        and folded_lower is not None
        and folded_lower[0][0] < rest_lower[0][0] - 0.45,
        details=f"rest={rest_lower}, folded={folded_lower}",
    )

    rest_front = ctx.part_element_world_aabb(floor_head, elem="front_datum_pad")
    with ctx.pose({nozzle: 0.45}):
        pitched_front = ctx.part_element_world_aabb(floor_head, elem="front_datum_pad")
    ctx.check(
        "nozzle articulation raises front datum",
        rest_front is not None
        and pitched_front is not None
        and pitched_front[0][2] > rest_front[0][2] + 0.045,
        details=f"rest={rest_front}, pitched={pitched_front}",
    )

    return ctx.report()


object_model = build_object_model()
