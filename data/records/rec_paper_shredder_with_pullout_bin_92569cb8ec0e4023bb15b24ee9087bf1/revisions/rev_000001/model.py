from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
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


BODY_WIDTH = 0.44
BODY_DEPTH = 0.36
BODY_HEIGHT = 0.94
LOWER_HEIGHT = 0.76
HEAD_HEIGHT = BODY_HEIGHT - LOWER_HEIGHT
BIN_TRAVEL = 0.18


def _make_body_shell() -> cq.Workplane:
    lower_body = cq.Workplane("XY").box(
        BODY_WIDTH,
        BODY_DEPTH,
        LOWER_HEIGHT,
        centered=(True, True, False),
    )
    cutter_head = (
        cq.Workplane("XY")
        .box(0.46, 0.38, HEAD_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, LOWER_HEIGHT))
    )
    shell = lower_body.union(cutter_head)

    inner_cavity = (
        cq.Workplane("XY")
        .box(0.382, 0.306, 0.84, centered=(True, True, False))
        .translate((0.0, 0.015, 0.06))
    )
    bin_opening = (
        cq.Workplane("XY")
        .box(0.344, 0.18, 0.448, centered=(True, True, False))
        .translate((0.0, -0.19, 0.08))
    )
    feed_slot = (
        cq.Workplane("XY")
        .box(0.242, 0.018, 0.055, centered=(True, True, False))
        .translate((0.0, -0.01, 0.885))
    )
    service_opening = (
        cq.Workplane("XY")
        .box(0.10, 0.18, 0.22, centered=(True, True, False))
        .translate((0.22, 0.01, 0.60))
    )
    paper_throat = (
        cq.Workplane("XY")
        .box(0.20, 0.12, 0.05, centered=(True, True, False))
        .translate((0.0, -0.08, 0.82))
    )

    shell = shell.cut(inner_cavity)
    shell = shell.cut(bin_opening)
    shell = shell.cut(feed_slot)
    shell = shell.cut(service_opening)
    shell = shell.cut(paper_throat)

    plinth = (
        cq.Workplane("XY")
        .box(0.40, 0.30, 0.02, centered=(True, True, False))
        .translate((0.0, 0.0, -0.002))
    )
    return shell.union(plinth)


def _make_control_pod() -> cq.Workplane:
    pod = (
        cq.Workplane("XY")
        .box(0.22, 0.105, 0.17)
        .translate((0.0, -0.0525, 0.0))
    )
    lower_cheek = cq.Workplane("XY").box(0.18, 0.04, 0.05).translate((0.0, -0.02, -0.06))
    return pod.union(lower_cheek)


def _make_bin_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.338, 0.276, 0.432, centered=(True, True, False))
        .translate((0.0, 0.138, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.318, 0.252, 0.405, centered=(True, True, False))
        .translate((0.0, 0.142, 0.022))
    )
    front_panel = (
        cq.Workplane("XY")
        .box(0.356, 0.018, 0.454, centered=(True, True, False))
        .translate((0.0, -0.009, 0.0))
    )
    handle_recess = (
        cq.Workplane("XY")
        .box(0.15, 0.032, 0.052, centered=(True, True, True))
        .translate((0.0, -0.003, 0.23))
    )
    finger_relief = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.12)
        .translate((-0.06, -0.019, 0.23))
    )
    return outer.cut(inner).union(front_panel).cut(handle_recess).cut(finger_relief)


def _make_selector_knob():
    return mesh_from_geometry(
        KnobGeometry(
            0.056,
            0.026,
            body_style="skirted",
            top_diameter=0.042,
            skirt=KnobSkirt(0.062, 0.006, flare=0.10),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
        ),
        "selector_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    body_plastic = model.material("body_plastic", rgba=(0.15, 0.16, 0.17, 1.0))
    body_panel = model.material("body_panel", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    pod_finish = model.material("pod_finish", rgba=(0.21, 0.22, 0.24, 1.0))
    bin_finish = model.material("bin_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    cutter_metal = model.material("cutter_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    button_finish = model.material("button_finish", rgba=(0.13, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.40, 0.30, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=trim_dark,
        name="cabinet_floor",
    )
    body.visual(
        Box((0.03, BODY_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(-0.205, 0.0, LOWER_HEIGHT * 0.5)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((0.03, BODY_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.205, 0.0, LOWER_HEIGHT * 0.5)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((0.38, 0.03, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.165, LOWER_HEIGHT * 0.5)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.018, 0.01, 0.48)),
        origin=Origin(xyz=(-0.181, -0.155, 0.27)),
        material=body_panel,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.018, 0.01, 0.48)),
        origin=Origin(xyz=(0.181, -0.155, 0.27)),
        material=body_panel,
        name="front_jamb_1",
    )
    body.visual(
        Box((0.344, 0.01, 0.08)),
        origin=Origin(xyz=(0.0, -0.155, 0.55)),
        material=body_panel,
        name="bin_header",
    )
    body.visual(
        Box((0.18, 0.018, 0.33)),
        origin=Origin(xyz=(0.0, -0.171, 0.75)),
        material=body_panel,
        name="pod_mount",
    )
    body.visual(
        Box((0.04, 0.38, HEAD_HEIGHT)),
        origin=Origin(xyz=(-0.21, 0.0, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=body_plastic,
        name="head_left_wall",
    )
    body.visual(
        Box((0.04, 0.12, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.21, -0.13, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=body_plastic,
        name="head_right_front",
    )
    body.visual(
        Box((0.04, 0.08, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.21, 0.15, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=body_plastic,
        name="head_right_rear",
    )
    body.visual(
        Box((0.04, 0.18, 0.02)),
        origin=Origin(xyz=(0.21, 0.03, 0.77)),
        material=body_plastic,
        name="head_right_lower",
    )
    body.visual(
        Box((0.04, 0.18, 0.02)),
        origin=Origin(xyz=(0.21, 0.03, 0.93)),
        material=body_plastic,
        name="head_right_upper",
    )
    body.visual(
        Box((0.38, 0.04, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.17, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=body_plastic,
        name="head_rear_wall",
    )
    body.visual(
        Box((0.38, 0.14, 0.03)),
        origin=Origin(xyz=(0.0, -0.105, 0.925)),
        material=body_plastic,
        name="roof_front",
    )
    body.visual(
        Box((0.38, 0.18, 0.03)),
        origin=Origin(xyz=(0.0, 0.105, 0.925)),
        material=body_plastic,
        name="roof_rear",
    )
    body.visual(
        Box((0.26, 0.03, 0.01)),
        origin=Origin(xyz=(0.0, -0.03, 0.918)),
        material=trim_dark,
        name="feed_slot_trim",
    )
    body.visual(
        Box((0.40, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=trim_dark,
        name="plinth",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.46, 0.38, BODY_HEIGHT)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.22, 0.105, 0.17)),
        origin=Origin(xyz=(0.0, -0.0525, 0.0)),
        material=pod_finish,
        name="pod_shell",
    )
    control_pod.visual(
        Box((0.18, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.02, -0.06)),
        material=pod_finish,
        name="pod_base",
    )
    control_pod.visual(
        Box((0.17, 0.004, 0.14)),
        origin=Origin(xyz=(0.0, -0.099, 0.0)),
        material=trim_dark,
        name="pod_face",
    )
    control_pod.inertial = Inertial.from_geometry(
        Box((0.22, 0.105, 0.17)),
        mass=1.0,
    )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(0.0, -0.18, 0.75)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        _make_selector_knob(),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="knob_cap",
    )
    selector_knob.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=trim_dark,
        name="knob_hub",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.062, 0.03, 0.062)),
        mass=0.08,
    )
    model.articulation(
        "control_pod_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=selector_knob,
        origin=Origin(xyz=(0.0, -0.111, 0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    button_offsets = (-0.05, 0.0, 0.05)
    for index, x_pos in enumerate(button_offsets):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.034, 0.02, 0.014)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.014, 0.014, 0.022)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=trim_dark,
            name="button_stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.034, 0.026, 0.022)),
            mass=0.03,
        )
        model.articulation(
            f"control_pod_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(x_pos, -0.109, -0.05)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.008,
            ),
        )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.014, 0.178, 0.138)),
        origin=Origin(xyz=(-0.007, -0.089, 0.0)),
        material=body_panel,
        name="door_panel",
    )
    service_door.visual(
        Box((0.012, 0.026, 0.038)),
        origin=Origin(xyz=(-0.006, -0.145, 0.0)),
        material=trim_dark,
        name="door_handle",
    )
    service_door.visual(
        Box((0.016, 0.018, 0.138)),
        origin=Origin(xyz=(-0.008, -0.009, 0.0)),
        material=body_panel,
        name="hinge_barrel",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.02, 0.18, 0.14)),
        mass=0.35,
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(0.223, 0.11, 0.85)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.5,
        ),
    )

    for index, y_pos in enumerate((-0.02, 0.02)):
        drum = model.part(f"drum_{index}")
        drum.visual(
            Box((0.32, 0.038, 0.038)),
            origin=Origin(xyz=(0.19, 0.0, 0.0)),
            material=cutter_metal,
            name="drum_body",
        )
        drum.visual(
            Box((0.04, 0.016, 0.016)),
            origin=Origin(xyz=(0.02, 0.0, 0.0)),
            material=trim_dark,
            name="shaft_inboard",
        )
        drum.visual(
            Box((0.04, 0.016, 0.016)),
            origin=Origin(xyz=(0.36, 0.0, 0.0)),
            material=trim_dark,
            name="shaft_outboard",
        )
        drum.inertial = Inertial.from_geometry(
            Box((0.40, 0.04, 0.04)),
            mass=0.55,
            origin=Origin(xyz=(0.20, 0.0, 0.0)),
        )
        model.articulation(
            f"body_to_drum_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum,
            origin=Origin(xyz=(-0.19, y_pos, 0.845)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=20.0),
        )

    waste_bin = model.part("waste_bin")
    waste_bin.visual(
        Box((0.01, 0.276, 0.432)),
        origin=Origin(xyz=(-0.164, 0.138, 0.216)),
        material=bin_finish,
        name="bin_side_0",
    )
    waste_bin.visual(
        Box((0.01, 0.276, 0.432)),
        origin=Origin(xyz=(0.164, 0.138, 0.216)),
        material=bin_finish,
        name="bin_side_1",
    )
    waste_bin.visual(
        Box((0.318, 0.01, 0.432)),
        origin=Origin(xyz=(0.0, 0.271, 0.216)),
        material=bin_finish,
        name="bin_back",
    )
    waste_bin.visual(
        Box((0.318, 0.254, 0.012)),
        origin=Origin(xyz=(0.0, 0.139, 0.006)),
        material=bin_finish,
        name="bin_floor",
    )
    waste_bin.visual(
        Box((0.356, 0.018, 0.454)),
        origin=Origin(xyz=(0.0, 0.001, 0.227)),
        material=bin_finish,
        name="bin_front",
    )
    waste_bin.visual(
        Box((0.11, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.012, 0.23)),
        material=trim_dark,
        name="bin_pull",
    )
    waste_bin.inertial = Inertial.from_geometry(
        Box((0.356, 0.276, 0.454)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.12, 0.22)),
    )
    model.articulation(
        "body_to_waste_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=waste_bin,
        origin=Origin(xyz=(0.0, -0.18, 0.03)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=BIN_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    control_pod = object_model.get_part("control_pod")
    service_door = object_model.get_part("service_door")
    selector_knob = object_model.get_part("selector_knob")
    waste_bin = object_model.get_part("waste_bin")
    bin_slide = object_model.get_articulation("body_to_waste_bin")
    door_joint = object_model.get_articulation("body_to_service_door")
    selector_joint = object_model.get_articulation("control_pod_to_selector_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_joint_0 = object_model.get_articulation("control_pod_to_button_0")
    button_joint_1 = object_model.get_articulation("control_pod_to_button_1")
    button_joint_2 = object_model.get_articulation("control_pod_to_button_2")
    drum_0 = object_model.get_part("drum_0")
    drum_1 = object_model.get_part("drum_1")

    ctx.allow_overlap(
        control_pod,
        selector_knob,
        elem_a="pod_shell",
        elem_b="knob_hub",
        reason="The selector knob hub is intentionally modeled as passing into the control pod housing.",
    )
    for button in (button_0, button_1, button_2):
        ctx.allow_overlap(
            control_pod,
            button,
            elem_a="pod_shell",
            elem_b="button_stem",
            reason="Each push button stem is intentionally represented as traveling into the control pod body.",
        )

    ctx.expect_overlap(
        waste_bin,
        body,
        axes="x",
        min_overlap=0.30,
        name="bin stays centered in the cabinet width",
    )
    ctx.expect_gap(
        waste_bin,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.015,
        positive_elem="bin_floor",
        negative_elem="cabinet_floor",
        name="bin sits on or just above the cabinet floor plane",
    )

    rest_pos = ctx.part_world_position(waste_bin)
    with ctx.pose({bin_slide: BIN_TRAVEL}):
        extended_pos = ctx.part_world_position(waste_bin)
        ctx.expect_overlap(
            waste_bin,
            body,
            axes="x",
            min_overlap=0.30,
            name="extended bin remains laterally guided by the body opening",
        )
        ctx.expect_overlap(
            waste_bin,
            body,
            axes="y",
            min_overlap=0.08,
            name="extended bin retains insertion into the body",
        )
    ctx.check(
        "bin slides outward from the front",
        rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    door_rest_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    with ctx.pose({door_joint: 1.2}):
        door_open_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    ctx.check(
        "service door swings outward on the side hinge",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and float(door_open_aabb[1][0]) > float(door_rest_aabb[1][0]) + 0.05,
        details=f"rest={door_rest_aabb}, open={door_open_aabb}",
    )

    knob_pos = ctx.part_world_position(selector_knob)
    button_positions = [ctx.part_world_position(button) for button in (button_0, button_1, button_2)]
    ctx.check(
        "selector knob sits above the buttons on the front pod",
        knob_pos is not None
        and all(pos is not None for pos in button_positions)
        and all(knob_pos[2] > pos[2] + 0.05 for pos in button_positions if pos is not None),
        details=f"knob={knob_pos}, buttons={button_positions}",
    )

    button_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_1: 0.008}):
        button_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "center button pushes inward",
        button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.005,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    for articulation in (button_joint_0, button_joint_1, button_joint_2, selector_joint):
        ctx.check(
            f"{articulation.name}_present",
            articulation is not None,
            details="Expected articulation to be present.",
        )

    drum_0_pos = ctx.part_world_position(drum_0)
    drum_1_pos = ctx.part_world_position(drum_1)
    ctx.check(
        "cutter drums are parallel horizontal shafts",
        drum_0_pos is not None
        and drum_1_pos is not None
        and abs(drum_0_pos[2] - drum_1_pos[2]) < 0.002
        and abs(drum_0_pos[1] - drum_1_pos[1]) > 0.03,
        details=f"drum_0={drum_0_pos}, drum_1={drum_1_pos}",
    )

    return ctx.report()


object_model = build_object_model()
