from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.360
BODY_WIDTH = 0.272
BODY_HEIGHT = 0.282
BODY_WALL = 0.009

CHAMBER_OUTER = (0.196, 0.168, 0.198)
CHAMBER_INNER = (0.180, 0.152, 0.194)
CHAMBER_POS = (-0.050, 0.0, 0.068)

LID_DEPTH = 0.208
LID_WIDTH = 0.218
LID_HEIGHT = 0.038
LID_WALL = 0.007
LID_HINGE_X = -0.146
LID_HINGE_Z = BODY_HEIGHT

FLAP_DEPTH = 0.078
FLAP_WIDTH = 0.106
FLAP_HEIGHT = 0.007
FLAP_HINGE_IN_LID = (0.092, 0.0, 0.025)

CONTROL_PAD = (0.096, 0.122, 0.011)
CONTROL_PAD_CENTER = (0.128, 0.0, BODY_HEIGHT + CONTROL_PAD[2] * 0.5)
DIAL_CENTER = (0.138, 0.040, BODY_HEIGHT + CONTROL_PAD[2])
START_CENTER = (0.136, -0.040, BODY_HEIGHT + CONTROL_PAD[2])
MODE_FRONT_CENTER = (0.105, 0.022, BODY_HEIGHT + CONTROL_PAD[2])
MODE_REAR_CENTER = (0.105, -0.020, BODY_HEIGHT + CONTROL_PAD[2])

CHAMBER_FLOOR_Z = CHAMBER_POS[2] + 0.004
SPINDLE_CENTER = (CHAMBER_POS[0], 0.0, CHAMBER_FLOOR_Z)


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def _body_shell_shape() -> cq.Workplane:
    body = _rounded_box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, 0.034)
    body = body.faces(">Z").edges().fillet(0.018)

    control_pad = (
        cq.Workplane("XY")
        .box(*CONTROL_PAD, centered=(True, True, False))
        .translate(
            (
                CONTROL_PAD_CENTER[0],
                CONTROL_PAD_CENTER[1],
                BODY_HEIGHT,
            )
        )
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )
    body = body.union(control_pad)

    lower_cavity = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - 2.0 * BODY_WALL,
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - 0.030,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.012))
    )
    body = body.cut(lower_cavity)

    chamber_opening = (
        cq.Workplane("XY")
        .box(CHAMBER_INNER[0], CHAMBER_INNER[1], 0.070, centered=(True, True, False))
        .translate((CHAMBER_POS[0], CHAMBER_POS[1], BODY_HEIGHT - 0.035))
    )
    body = body.cut(chamber_opening)

    body = body.union(_chamber_liner_shape().translate(CHAMBER_POS))

    dial_bore = (
        cq.Workplane("XY")
        .circle(0.0095)
        .extrude(0.030)
        .translate((DIAL_CENTER[0], DIAL_CENTER[1], BODY_HEIGHT - 0.004))
    )
    body = body.cut(dial_bore)

    for center, radius in (
        (START_CENTER, 0.0105),
        (MODE_FRONT_CENTER, 0.0082),
        (MODE_REAR_CENTER, 0.0082),
    ):
        button_bore = (
            cq.Workplane("XY")
            .circle(radius)
            .extrude(0.028)
            .translate((center[0], center[1], BODY_HEIGHT - 0.003))
        )
        body = body.cut(button_bore)

    foot_offsets = (
        (0.122, 0.090),
        (0.122, -0.090),
        (-0.122, 0.090),
        (-0.122, -0.090),
    )
    for x_pos, y_pos in foot_offsets:
        foot = (
            cq.Workplane("XY")
            .circle(0.012)
            .extrude(0.006)
            .translate((x_pos, y_pos, -0.006))
        )
        body = body.union(foot)

    return body


def _chamber_liner_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(*CHAMBER_OUTER, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    inner = (
        cq.Workplane("XY")
        .box(*CHAMBER_INNER, centered=(True, True, False))
        .translate((0.0, 0.0, 0.004))
        .edges("|Z")
        .fillet(0.010)
    )
    return outer.cut(inner)


def _lid_shape() -> cq.Workplane:
    lid = _rounded_box(LID_DEPTH, LID_WIDTH, LID_HEIGHT, 0.024)
    lid = lid.faces(">Z").edges().fillet(0.010)
    lid = lid.translate((LID_DEPTH * 0.5, 0.0, 0.0))

    underside = (
        cq.Workplane("XY")
        .box(
            LID_DEPTH - 2.0 * LID_WALL,
            LID_WIDTH - 2.0 * LID_WALL,
            LID_HEIGHT - LID_WALL,
            centered=(True, True, False),
        )
        .translate((LID_DEPTH * 0.5, 0.0, 0.0))
        .translate((0.0, 0.0, LID_WALL))
    )
    lid = lid.cut(underside)

    pocket_shell = (
        cq.Workplane("XY")
        .box(0.094, 0.118, 0.018, centered=(True, True, False))
        .translate((0.128, 0.0, -0.006))
        .cut(
            cq.Workplane("XY")
            .box(0.082, 0.106, 0.022, centered=(True, True, False))
            .translate((0.128, 0.0, -0.008))
        )
    )
    lid = lid.cut(
        cq.Workplane("XY")
        .box(0.082, 0.106, 0.024, centered=(True, True, False))
        .translate((0.128, 0.0, 0.014))
    )
    lid = lid.union(pocket_shell)
    return lid


def _flap_shape() -> cq.Workplane:
    flap = (
        cq.Workplane("XY")
        .box(FLAP_DEPTH, FLAP_WIDTH, FLAP_HEIGHT, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .edges()
        .fillet(0.003)
    )
    flap_lip = (
        cq.Workplane("XY")
        .box(FLAP_DEPTH - 0.018, FLAP_WIDTH - 0.018, 0.018, centered=(False, True, False))
        .translate((0.009, 0.0, -0.018))
        .edges("|Z")
        .fillet(0.003)
    )
    hinge_tab = (
        cq.Workplane("XY")
        .box(0.012, 0.060, 0.006, centered=(False, True, False))
        .translate((-0.012, 0.0, -0.003))
        .edges("|Z")
        .fillet(0.002)
    )
    return flap.union(flap_lip).union(hinge_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_bread_maker")

    body_finish = model.material("body_finish", rgba=(0.91, 0.92, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.13, 0.13, 0.14, 1.0))
    control_dark = model.material("control_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    button_light = model.material("button_light", rgba=(0.94, 0.94, 0.93, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "bread_maker_body_shell"),
        material=body_finish,
        name="body_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "bread_maker_lid"),
        material=body_finish,
        name="lid_shell",
    )

    flap = model.part("dispenser_flap")
    flap.visual(
        mesh_from_cadquery(_flap_shape(), "bread_maker_dispenser_flap"),
        material=trim_dark,
        name="flap_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=trim_dark,
        name="spindle_shaft",
    )
    spindle.visual(
        Box((0.046, 0.013, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=trim_dark,
        name="kneading_paddle",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=rubber_dark,
        name="spindle_collar",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.021,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.056, 0.006, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "bread_maker_dial",
        ),
        material=control_dark,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=control_dark,
        name="dial_shaft",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.013, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=button_light,
        name="start_cap",
    )
    start_button.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=button_light,
        name="start_stem",
    )

    mode_button_0 = model.part("mode_button_0")
    mode_button_0.visual(
        Cylinder(radius=0.010, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.00225)),
        material=button_light,
        name="mode_cap",
    )
    mode_button_0.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=button_light,
        name="mode_stem",
    )

    mode_button_1 = model.part("mode_button_1")
    mode_button_1.visual(
        Cylinder(radius=0.010, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.00225)),
        material=button_light,
        name="mode_cap",
    )
    mode_button_1.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=button_light,
        name="mode_stem",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "lid_to_dispenser_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=FLAP_HINGE_IN_LID),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(128.0),
        ),
    )
    model.articulation(
        "body_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=SPINDLE_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=DIAL_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=START_CENTER),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.003),
    )
    model.articulation(
        "body_to_mode_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_0,
        origin=Origin(xyz=MODE_FRONT_CENTER),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0025),
    )
    model.articulation(
        "body_to_mode_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_1,
        origin=Origin(xyz=MODE_REAR_CENTER),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    flap = object_model.get_part("dispenser_flap")
    spindle = object_model.get_part("spindle")
    dial = object_model.get_part("dial")
    start_button = object_model.get_part("start_button")
    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    flap_hinge = object_model.get_articulation("lid_to_dispenser_flap")
    dial_joint = object_model.get_articulation("body_to_dial")
    start_joint = object_model.get_articulation("body_to_start_button")
    mode_joint_0 = object_model.get_articulation("body_to_mode_button_0")
    mode_joint_1 = object_model.get_articulation("body_to_mode_button_1")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="body_shell",
        elem_b="lid_shell",
        reason="The lid is modeled as a nested top cover that seats slightly into the bread-maker rim.",
    )
    ctx.allow_overlap(
        lid,
        flap,
        elem_a="lid_shell",
        elem_b="flap_shell",
        reason="The dispenser flap is represented as a recessed plug seated into the lid pocket.",
    )
    ctx.allow_overlap(
        body,
        spindle,
        elem_a="body_shell",
        elem_b="spindle_collar",
        reason="The kneading spindle collar intentionally passes through the chamber floor opening.",
    )

    def _top_z(part_obj) -> float | None:
        aabb = ctx.part_world_aabb(part_obj)
        if aabb is None:
            return None
        return float(aabb[1][2])

    with ctx.pose({lid_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.160,
            name="lid covers the chamber opening footprint",
        )
        ctx.expect_overlap(
            flap,
            lid,
            axes="xy",
            min_overlap=0.060,
            name="dispenser flap stays seated inside the lid pocket footprint",
        )

    lid_rest_top = _top_z(lid)
    flap_rest_top = _top_z(flap)
    with ctx.pose({lid_hinge: math.radians(100.0)}):
        lid_open_top = _top_z(lid)
    with ctx.pose({flap_hinge: math.radians(110.0)}):
        flap_open_top = _top_z(flap)

    ctx.check(
        "lid opens upward",
        lid_rest_top is not None and lid_open_top is not None and lid_open_top > lid_rest_top + 0.050,
        details=f"rest_top={lid_rest_top}, open_top={lid_open_top}",
    )
    ctx.check(
        "flap opens upward",
        flap_rest_top is not None and flap_open_top is not None and flap_open_top > flap_rest_top + 0.015,
        details=f"rest_top={flap_rest_top}, open_top={flap_open_top}",
    )
    ctx.expect_within(
        spindle,
        body,
        axes="xy",
        margin=0.050,
        name="spindle stays centered within the bread maker footprint",
    )
    ctx.check(
        "dial joint is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )

    start_rest = ctx.part_world_position(start_button)
    mode_0_rest = ctx.part_world_position(mode_button_0)
    mode_1_rest = ctx.part_world_position(mode_button_1)
    with ctx.pose({start_joint: 0.003, mode_joint_0: 0.0025, mode_joint_1: 0.0025, dial_joint: 1.4}):
        start_pressed = ctx.part_world_position(start_button)
        mode_0_pressed = ctx.part_world_position(mode_button_0)
        mode_1_pressed = ctx.part_world_position(mode_button_1)
        dial_pose = ctx.part_world_position(dial)

    dial_rest = ctx.part_world_position(dial)
    ctx.check(
        "start button presses downward",
        start_rest is not None and start_pressed is not None and start_pressed[2] < start_rest[2] - 0.002,
        details=f"rest={start_rest}, pressed={start_pressed}",
    )
    ctx.check(
        "mode button 0 presses downward",
        mode_0_rest is not None and mode_0_pressed is not None and mode_0_pressed[2] < mode_0_rest[2] - 0.0015,
        details=f"rest={mode_0_rest}, pressed={mode_0_pressed}",
    )
    ctx.check(
        "mode button 1 presses downward",
        mode_1_rest is not None and mode_1_pressed is not None and mode_1_pressed[2] < mode_1_rest[2] - 0.0015,
        details=f"rest={mode_1_rest}, pressed={mode_1_pressed}",
    )
    ctx.check(
        "dial rotates in place",
        dial_rest is not None
        and dial_pose is not None
        and abs(dial_pose[0] - dial_rest[0]) < 1e-6
        and abs(dial_pose[1] - dial_rest[1]) < 1e-6
        and abs(dial_pose[2] - dial_rest[2]) < 1e-6,
        details=f"rest={dial_rest}, posed={dial_pose}",
    )

    return ctx.report()


object_model = build_object_model()
