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


BODY_W = 0.380
BODY_D = 0.295
BODY_H = 0.245

OPEN_W = 0.202
OPEN_D = 0.154
OPEN_Y = -0.015

CHAMBER_W = 0.182
CHAMBER_D = 0.134
CHAMBER_H = 0.176
CHAMBER_Z = 0.069

POD_W = 0.300
POD_D = 0.110
POD_H = 0.095
POD_Y = 0.085

LID_W = 0.236
LID_D = 0.196
LID_T = 0.022
HINGE_Y = 0.094
HINGE_Z = BODY_H + 0.007

DIAL_Z = 0.062
PROGRAM_BUTTON_POSITIONS = (
    (-0.068, 0.078),
    (-0.068, 0.062),
    (-0.068, 0.046),
    (0.068, 0.078),
    (0.068, 0.062),
    (0.068, 0.046),
)


def _program_button_positions() -> list[tuple[float, float]]:
    return list(PROGRAM_BUTTON_POSITIONS)


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .edges(">Z")
        .fillet(0.010)
    )

    chamber_cut = (
        cq.Workplane("XY")
        .box(OPEN_W, OPEN_D, 0.202, centered=(True, True, False))
        .translate((0.0, OPEN_Y, 0.048))
    )

    release_slot = (
        cq.Workplane("XY")
        .box(0.066, 0.034, 0.016, centered=(True, True, True))
        .translate((0.0, -BODY_D / 2.0 + 0.012, 0.209))
    )

    vent_recess = (
        cq.Workplane("XY")
        .box(0.180, 0.050, 0.012, centered=(True, True, True))
        .translate((0.0, BODY_D / 2.0 - 0.010, 0.155))
    )

    return shell.cut(chamber_cut).cut(release_slot).cut(vent_recess)


def _chamber_shape() -> cq.Workplane:
    wall = 0.003
    corner_r = 0.012
    liner = (
        cq.Workplane("XY")
        .box(CHAMBER_W, CHAMBER_D, CHAMBER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(corner_r)
    )
    inner = (
        cq.Workplane("XY")
        .box(CHAMBER_W - 2.0 * wall, CHAMBER_D - 2.0 * wall, CHAMBER_H, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    liner = liner.cut(inner)

    lip_outer = (
        cq.Workplane("XY")
        .box(0.192, 0.144, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, CHAMBER_H - 0.004))
    )
    lip_inner = (
        cq.Workplane("XY")
        .box(0.172, 0.124, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, CHAMBER_H - 0.004))
    )
    lip = lip_outer.cut(lip_inner)

    side_tabs = (
        cq.Workplane("XY")
        .box(0.010, 0.030, 0.004, centered=(True, True, False))
        .translate((0.096, 0.0, CHAMBER_H - 0.006))
    ).union(
        cq.Workplane("XY")
        .box(0.010, 0.030, 0.004, centered=(True, True, False))
        .translate((-0.096, 0.0, CHAMBER_H - 0.006))
    )

    collar = (
        cq.Workplane("XY")
        .circle(0.014)
        .circle(0.0085)
        .extrude(0.004)
    )
    floor_hole = cq.Workplane("XY").circle(0.0075).extrude(0.010)

    return liner.union(lip).union(side_tabs).union(collar).cut(floor_hole)


def _pod_shape() -> cq.Workplane:
    pod = (
        cq.Workplane("XY")
        .box(POD_W, POD_D, POD_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.008)
    )

    top_recess = (
        cq.Workplane("XY")
        .box(0.190, 0.052, 0.010, centered=(True, True, True))
        .translate((0.0, 0.006, POD_H - 0.004))
    )
    lid_clearance = (
        cq.Workplane("XY")
        .box(0.250, 0.092, 0.040, centered=(True, True, False))
        .translate((0.0, -0.009, 0.0))
    )
    pod = pod.cut(top_recess).cut(lid_clearance)

    dial_hole = cq.Workplane("XZ").circle(0.0115).extrude(POD_D + 0.020, both=True).translate((0.0, 0.0, DIAL_Z))
    pod = pod.cut(dial_hole)

    for button_x, button_z in _program_button_positions():
        button_hole = (
            cq.Workplane("XZ")
            .circle(0.0064)
            .extrude(POD_D + 0.020, both=True)
            .translate((button_x, 0.0, button_z))
        )
        pod = pod.cut(button_hole)

    return pod


def _lid_frame_shape() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, True, True))
        .translate((0.0, -LID_D / 2.0, 0.010))
        .edges("|Z")
        .fillet(0.012)
    )
    window_cut = (
        cq.Workplane("XY")
        .box(0.146, 0.110, 0.040, centered=(True, True, True))
        .translate((0.0, -0.102, 0.011))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(0.132, 0.018, 0.009, centered=(True, True, True))
        .translate((0.0, -LID_D + 0.015, 0.003))
    )
    return frame.cut(window_cut).union(front_lip)


def _dial_ring_shape() -> cq.Workplane:
    shell = cq.Workplane("XZ").circle(0.031).extrude(0.012).translate((0.0, -0.012, 0.0))
    ring_opening = cq.Workplane("XZ").circle(0.018).extrude(0.007).translate((0.0, -0.010, 0.0))
    shaft = cq.Workplane("XZ").circle(0.010).extrude(0.018)
    return shell.cut(ring_opening).union(shaft)


def _round_button_shape(cap_radius: float, cap_depth: float, stem_radius: float, stem_depth: float) -> cq.Workplane:
    cap = cq.Workplane("XZ").circle(cap_radius).extrude(cap_depth).translate((0.0, -cap_depth, 0.0))
    stem = cq.Workplane("XZ").circle(stem_radius).extrude(stem_depth)
    return cap.union(stem)


def _release_button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.062, 0.008, 0.016, centered=(True, True, True))
        .translate((0.0, -0.004, 0.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.050, 0.010, 0.012, centered=(True, True, True))
        .translate((0.0, 0.005, 0.0))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_bread_maker")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.93, 1.0))
    pod_graphite = model.material("pod_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.30, 0.34, 0.38))
    trim_grey = model.material("trim_grey", rgba=(0.62, 0.63, 0.65, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "bread_maker_body"),
        material=body_white,
        name="housing",
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(_chamber_shape(), "bread_maker_chamber"),
        material=steel,
        name="liner",
    )

    pod = model.part("pod")
    pod.visual(
        mesh_from_cadquery(_pod_shape(), "bread_maker_pod"),
        material=pod_graphite,
        name="shell",
    )
    pod.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(
            xyz=(0.0, -POD_D / 2.0 - 0.0015, DIAL_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_plastic,
        name="dial_core",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame_shape(), "bread_maker_lid_frame"),
        material=body_white,
        name="frame",
    )
    lid.visual(
        Box((0.150, 0.114, 0.006)),
        origin=Origin(xyz=(0.0, -0.102, 0.011)),
        material=glass,
        name="window",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_grey,
        name="barrel",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.0085, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=dark_plastic,
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Box((0.018, 0.006, 0.004)),
        origin=Origin(xyz=(0.009, 0.0, 0.024)),
        material=dark_plastic,
        name="paddle",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_ring_shape(), "bread_maker_dial"),
        material=dark_plastic,
        name="ring",
    )
    dial.visual(
        Box((0.010, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.009, 0.025)),
        material=trim_grey,
        name="indicator",
    )

    button_mesh = mesh_from_cadquery(
        _round_button_shape(cap_radius=0.0082, cap_depth=0.0055, stem_radius=0.0064, stem_depth=0.010),
        "bread_maker_program_button",
    )
    button_parts = []
    for index, _ in enumerate(_program_button_positions()):
        program_button = model.part(f"program_button_{index}")
        program_button.visual(
            button_mesh,
            material=trim_grey,
            name="cap",
        )
        button_parts.append(program_button)

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(_release_button_shape(), "bread_maker_release_button"),
        material=trim_grey,
        name="cap",
    )

    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.0, OPEN_Y, CHAMBER_Z)),
    )
    model.articulation(
        "body_to_pod",
        ArticulationType.FIXED,
        parent=body,
        child=pod,
        origin=Origin(xyz=(0.0, POD_Y, BODY_H)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "chamber_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=14.0),
    )
    model.articulation(
        "pod_to_dial",
        ArticulationType.CONTINUOUS,
        parent=pod,
        child=dial,
        origin=Origin(xyz=(0.0, -POD_D / 2.0, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    for index, (button_x, button_z) in enumerate(_program_button_positions()):
        model.articulation(
            f"pod_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=pod,
            child=button_parts[index],
            origin=Origin(xyz=(button_x, -POD_D / 2.0, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0025,
            ),
        )

    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, 0.209)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.005,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    pod = object_model.get_part("pod")
    spindle = object_model.get_part("spindle")
    dial = object_model.get_part("dial")
    release_button = object_model.get_part("release_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    spindle_joint = object_model.get_articulation("chamber_to_spindle")
    dial_joint = object_model.get_articulation("pod_to_dial")
    release_joint = object_model.get_articulation("body_to_release_button")

    ctx.allow_overlap(
        body,
        chamber,
        elem_a="housing",
        elem_b="liner",
        reason="The metal chamber liner is intentionally nested inside the simplified outer housing cavity.",
    )
    ctx.allow_overlap(
        body,
        release_button,
        elem_a="housing",
        elem_b="cap",
        reason="The front release button is intentionally represented as sliding into a simplified housing slot.",
    )

    ctx.expect_origin_distance(
        spindle,
        chamber,
        axes="xy",
        max_dist=0.001,
        name="spindle stays centered in the baking chamber",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="frame",
        name="closed lid sits just above the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.160,
        elem_a="frame",
        name="closed lid covers the top opening footprint",
    )
    ctx.expect_gap(
        pod,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="control pod seats directly on the top deck",
    )

    lid_rest = ctx.part_world_aabb(lid)
    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            lid_open = ctx.part_world_aabb(lid)
        ctx.check(
            "lid rotates upward from the rear hinge",
            lid_rest is not None
            and lid_open is not None
            and (lid_open[1][2] + lid_open[0][2]) / 2.0 > (lid_rest[1][2] + lid_rest[0][2]) / 2.0 + 0.070
            and (lid_open[1][1] + lid_open[0][1]) / 2.0 > (lid_rest[1][1] + lid_rest[0][1]) / 2.0 + 0.045,
            details=f"rest={lid_rest}, open={lid_open}",
        )

    spindle_rest = ctx.part_element_world_aabb(spindle, elem="paddle")
    with ctx.pose({spindle_joint: math.pi / 2.0}):
        spindle_turn = ctx.part_element_world_aabb(spindle, elem="paddle")
    if spindle_rest is not None and spindle_turn is not None:
        spindle_rest_center = tuple((spindle_rest[0][i] + spindle_rest[1][i]) / 2.0 for i in range(3))
        spindle_turn_center = tuple((spindle_turn[0][i] + spindle_turn[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "kneading spindle rotates visibly",
            abs(spindle_turn_center[0] - spindle_rest_center[0]) > 0.006
            and abs(spindle_turn_center[1] - spindle_rest_center[1]) > 0.006,
            details=f"rest={spindle_rest_center}, turned={spindle_turn_center}",
        )

    dial_rest = ctx.part_element_world_aabb(dial, elem="indicator")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_turn = ctx.part_element_world_aabb(dial, elem="indicator")
    if dial_rest is not None and dial_turn is not None:
        dial_rest_center = tuple((dial_rest[0][i] + dial_rest[1][i]) / 2.0 for i in range(3))
        dial_turn_center = tuple((dial_turn[0][i] + dial_turn[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "ring dial rotates about its face axis",
            abs(dial_turn_center[0] - dial_rest_center[0]) > 0.012
            and abs(dial_turn_center[2] - dial_rest_center[2]) > 0.012,
            details=f"rest={dial_rest_center}, turned={dial_turn_center}",
        )

    for index in range(len(_program_button_positions())):
        button = object_model.get_part(f"program_button_{index}")
        joint = object_model.get_articulation(f"pod_to_program_button_{index}")
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"program_button_{index} presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    release_rest = ctx.part_world_position(release_button)
    release_upper = release_joint.motion_limits.upper if release_joint.motion_limits is not None else None
    if release_upper is not None:
        with ctx.pose({release_joint: release_upper}):
            release_pressed = ctx.part_world_position(release_button)
        ctx.check(
            "front release button slides into the body",
            release_rest is not None and release_pressed is not None and release_pressed[1] > release_rest[1] + 0.003,
            details=f"rest={release_rest}, pressed={release_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
