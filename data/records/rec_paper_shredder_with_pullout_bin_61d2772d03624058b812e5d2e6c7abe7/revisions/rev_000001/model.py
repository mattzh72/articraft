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
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_WIDTH = 0.34
BODY_DEPTH = 0.27
BODY_HEIGHT = 0.56
BODY_FRONT_Y = -BODY_DEPTH * 0.5
BODY_REAR_Y = BODY_DEPTH * 0.5

PANEL_FRONT_Y = -0.060
PANEL_REAR_Y = 0.060
PANEL_FRONT_Z = 0.495
PANEL_REAR_Z = 0.555
PANEL_ANGLE = math.atan2(PANEL_REAR_Z - PANEL_FRONT_Z, PANEL_REAR_Y - PANEL_FRONT_Y)

BIN_WIDTH = 0.296
BIN_DEPTH = 0.205
BIN_HEIGHT = 0.248
BIN_BOTTOM_Z = 0.030
BIN_FRONT_Y = BODY_FRONT_Y + 0.001
BIN_CENTER_Y = BIN_FRONT_Y + BIN_DEPTH * 0.5
BIN_CENTER_Z = BIN_BOTTOM_Z + BIN_HEIGHT * 0.5
BIN_TRAVEL = 0.120

BODY_CAVITY_WIDTH = 0.326
RAIL_SIZE = (0.012, 0.240, 0.010)
RAIL_CENTER_Y = -0.010
RAIL_CENTER_Z = 0.198
FLANGE_SIZE = (0.007, 0.210, 0.010)
FLANGE_CENTER_Y = 0.0
FLANGE_CENTER_Z = 0.158
RAIL_X = BIN_WIDTH * 0.5 + FLANGE_SIZE[0] * 0.5 - 0.0005

FLAP_WIDTH = 0.190
FLAP_HEIGHT = 0.058
FLAP_BOTTOM_Z = 0.338
FLAP_CENTER_Z = FLAP_BOTTOM_Z + FLAP_HEIGHT * 0.5

CUTTER_LENGTH = 0.252
CUTTER_RADIUS = 0.014
CUTTER_DISK_RADIUS = 0.015
CUTTER_Z = 0.438
CUTTER_Y_OFFSET = 0.020

KNOB_X = 0.0
KNOB_Y = 0.022
CONTROL_PANEL_CENTER_Y = 0.026
CONTROL_PANEL_SIZE = (0.182, 0.112, 0.008)
CONTROL_PANEL_POCKET_DEPTH = 0.010
BUTTON_LAYOUT = (
    (-0.052, 0.048),
    (0.0, 0.060),
    (0.052, 0.048),
    (-0.047, -0.004),
    (0.047, -0.004),
)


def _panel_z(y: float) -> float:
    return PANEL_FRONT_Z + (y - PANEL_FRONT_Y) * (
        (PANEL_REAR_Z - PANEL_FRONT_Z) / (PANEL_REAR_Y - PANEL_FRONT_Y)
    )


def _panel_origin(x: float, y: float, proud: float = 0.0) -> Origin:
    return Origin(
        xyz=(
            x,
            y - math.sin(PANEL_ANGLE) * proud,
            _panel_z(y) + math.cos(PANEL_ANGLE) * proud,
        ),
        rpy=(PANEL_ANGLE, 0.0, 0.0),
    )


def _button_recess(x: float, y: float):
    return (
        cq.Workplane("XY", origin=(x, y, _panel_z(y)))
        .transformed(rotate=(math.degrees(PANEL_ANGLE), 0.0, 0.0))
        .rect(0.014, 0.009)
        .extrude(-0.025)
    )


def _knob_hole(x: float, y: float):
    return (
        cq.Workplane("XY", origin=(x, y, _panel_z(y)))
        .transformed(rotate=(math.degrees(PANEL_ANGLE), 0.0, 0.0))
        .circle(0.007)
        .extrude(-0.030)
    )


def _panel_pocket():
    return (
        cq.Workplane("XY", origin=(0.0, CONTROL_PANEL_CENTER_Y, _panel_z(CONTROL_PANEL_CENTER_Y)))
        .transformed(rotate=(math.degrees(PANEL_ANGLE), 0.0, 0.0))
        .rect(CONTROL_PANEL_SIZE[0] + 0.008, CONTROL_PANEL_SIZE[1] + 0.008)
        .extrude(-CONTROL_PANEL_POCKET_DEPTH)
    )


def _body_shell_geometry():
    outer = (
        cq.Workplane("YZ")
        .moveTo(BODY_FRONT_Y, 0.0)
        .lineTo(BODY_REAR_Y, 0.0)
        .lineTo(BODY_REAR_Y, 0.455)
        .threePointArc((0.115, 0.535), (PANEL_REAR_Y, PANEL_REAR_Z))
        .lineTo(PANEL_FRONT_Y, PANEL_FRONT_Z)
        .threePointArc((-0.110, 0.475), (BODY_FRONT_Y, 0.410))
        .lineTo(BODY_FRONT_Y, 0.0)
        .close()
        .extrude(BODY_WIDTH * 0.5, both=True)
    )

    outer = outer.edges("|Z").fillet(0.018)

    bin_cavity = (
        cq.Workplane("XY")
        .box(BODY_CAVITY_WIDTH, 0.238, 0.275, centered=(True, True, False))
        .translate((0.0, -0.020, BIN_BOTTOM_Z + 0.1375))
    )
    cutter_chamber = (
        cq.Workplane("XY")
        .box(0.270, 0.112, 0.074, centered=(True, True, True))
        .translate((0.0, -0.004, CUTTER_Z))
    )
    flap_opening = (
        cq.Workplane("XY")
        .box(FLAP_WIDTH + 0.006, 0.050, FLAP_HEIGHT + 0.004, centered=(True, True, True))
        .translate((0.0, BODY_FRONT_Y - 0.010, FLAP_CENTER_Z))
    )
    feed_slot = (
        cq.Workplane("XY", origin=(0.0, -0.018, _panel_z(-0.018)))
        .transformed(rotate=(math.degrees(PANEL_ANGLE), 0.0, 0.0))
        .slot2D(0.225, 0.010)
        .extrude(-0.085)
    )

    shell = outer.cut(bin_cavity).cut(cutter_chamber).cut(flap_opening).cut(feed_slot).cut(_panel_pocket())
    hinge_support = (
        cq.Workplane("XY")
        .box(0.070, 0.004, 0.014, centered=(True, True, True))
        .translate((0.0, BODY_FRONT_Y - 0.002, FLAP_BOTTOM_Z + 0.003))
    )
    shell = shell.union(hinge_support)
    return shell


def _bin_shell_geometry():
    outer = (
        cq.Workplane("XY")
        .box(BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    shell = outer.faces(">Z").shell(-0.0035)
    grip_opening = (
        cq.Workplane("XZ")
        .workplane(offset=-BIN_DEPTH * 0.5 - 0.010)
        .center(0.0, 0.194)
        .slot2D(0.160, 0.030)
        .extrude(0.034)
    )
    return shell.cut(grip_opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    body_finish = model.material("body_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    bin_finish = model.material("bin_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    control_finish = model.material("control_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    button_finish = model.material("button_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.60, 0.62, 0.66, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_geometry(), "shredder_body_shell"),
        material=body_finish,
        name="shell",
    )
    for index, x in enumerate((-RAIL_X, RAIL_X)):
        body.visual(
            Box(RAIL_SIZE),
            origin=Origin(xyz=(x, RAIL_CENTER_Y, RAIL_CENTER_Z)),
            material=control_finish,
            name=f"rail_{index}",
        )
    for cutter_index, y in enumerate((-CUTTER_Y_OFFSET, CUTTER_Y_OFFSET)):
        for side_index, x in enumerate((-0.1305, 0.1305)):
            body.visual(
                Box((0.009, 0.024, 0.024)),
                origin=Origin(xyz=(x, y, CUTTER_Z)),
                material=control_finish,
                name=f"bearing_pad_{cutter_index}_{side_index}",
            )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_bin_shell_geometry(), "shredder_bin_shell"),
        material=bin_finish,
        name="bin_shell",
    )
    flange_x = BIN_WIDTH * 0.5 + FLANGE_SIZE[0] * 0.5
    for index, x in enumerate((-flange_x, flange_x)):
        bin_part.visual(
            Box(FLANGE_SIZE),
            origin=Origin(xyz=(x, FLANGE_CENTER_Y, FLANGE_CENTER_Z)),
            material=bin_finish,
            name=f"side_flange_{index}",
        )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box(CONTROL_PANEL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, CONTROL_PANEL_SIZE[2] * 0.5)),
        material=control_finish,
        name="panel_plate",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.041,
                0.022,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.049, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "selector_knob",
        ),
        material=control_finish,
        name="selector_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=control_finish,
        name="selector_shaft",
    )

    for index, (x, y) in enumerate(BUTTON_LAYOUT):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.018, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=control_finish,
            name="button_stem",
        )

    front_flap = model.part("front_flap")
    front_flap.visual(
        Box((FLAP_WIDTH, 0.005, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0025, FLAP_HEIGHT * 0.5)),
        material=body_finish,
        name="flap_panel",
    )
    front_flap.visual(
        Box((0.070, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.006, FLAP_HEIGHT - 0.012)),
        material=control_finish,
        name="grip_lip",
    )
    front_flap.visual(
        Box((0.070, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.008, 0.004)),
        material=control_finish,
        name="hinge_barrel",
    )
    front_flap.visual(
        Box((0.070, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.004)),
        material=control_finish,
        name="hinge_web",
    )

    for index in range(2):
        cutter = model.part(f"cutter_{index}")
        cutter.visual(
            Box((CUTTER_LENGTH, CUTTER_RADIUS * 2.0, CUTTER_RADIUS * 2.0)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=steel_finish,
            name="shaft",
        )
        for disk_index, x in enumerate((-0.100, -0.075, -0.050, -0.025, 0.0, 0.025, 0.050, 0.075, 0.100)):
            cutter.visual(
                Cylinder(radius=CUTTER_DISK_RADIUS, length=0.008),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
                material=steel_finish,
                name=f"disk_{disk_index}",
            )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, BIN_CENTER_Y, BIN_BOTTOM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=BIN_TRAVEL),
    )

    model.articulation(
        "panel_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y - CONTROL_PANEL_CENTER_Y, CONTROL_PANEL_SIZE[2])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=_panel_origin(0.0, CONTROL_PANEL_CENTER_Y, proud=-CONTROL_PANEL_POCKET_DEPTH),
    )

    for index, (x, y) in enumerate(BUTTON_LAYOUT):
        model.articulation(
            f"panel_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=f"mode_button_{index}",
            origin=Origin(xyz=(x, y - CONTROL_PANEL_CENTER_Y, CONTROL_PANEL_SIZE[2])),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.0025),
        )

    model.articulation(
        "body_to_front_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_flap,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, FLAP_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=0.0, upper=1.05),
    )

    model.articulation(
        "body_to_cutter_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="cutter_0",
        origin=Origin(xyz=(0.0, -CUTTER_Y_OFFSET, CUTTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )
    model.articulation(
        "body_to_cutter_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="cutter_1",
        origin=Origin(xyz=(0.0, CUTTER_Y_OFFSET, CUTTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
        mimic=Mimic("body_to_cutter_0", multiplier=-1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    front_flap = object_model.get_part("front_flap")

    ctx.allow_overlap(
        body,
        bin_part,
        elem_a="shell",
        elem_b="bin_shell",
        reason="The outer housing shell is represented as a simplified enclosure proxy around the pullout bin cavity while the rail checks enforce the intended supported drawer fit.",
    )
    control_panel = object_model.get_part("control_panel")
    selector_knob = object_model.get_part("selector_knob")
    ctx.allow_overlap(
        body,
        control_panel,
        elem_a="shell",
        elem_b="panel_plate",
        reason="The sloped control insert is represented as a solid panel seated into a simplified recessed housing pocket.",
    )
    ctx.allow_overlap(
        body,
        selector_knob,
        elem_a="shell",
        elem_b="selector_shaft",
        reason="The selector shaft intentionally passes through the housing into the control cavity.",
    )
    ctx.allow_overlap(
        control_panel,
        selector_knob,
        elem_a="panel_plate",
        elem_b="selector_shaft",
        reason="The solid panel insert stands in for a drilled panel around the selector shaft.",
    )
    ctx.allow_overlap(
        control_panel,
        selector_knob,
        elem_a="panel_plate",
        elem_b="selector_cap",
        reason="The selector cap is represented as a flush-mounted control seated on a simplified solid panel insert without a modeled surface break.",
    )
    for index in range(5):
        button = object_model.get_part(f"mode_button_{index}")
        ctx.allow_overlap(
            body,
            button,
            elem_a="shell",
            elem_b="button_stem",
            reason="The push-button stem intentionally enters the body through the recessed control cavity.",
        )
        ctx.allow_overlap(
            control_panel,
            button,
            elem_a="panel_plate",
            elem_b="button_stem",
            reason="The solid control-panel insert stands in for pierced button guide holes around each push-button stem.",
        )
        ctx.allow_overlap(
            control_panel,
            button,
            elem_a="panel_plate",
            elem_b="button_cap",
            reason="The button cap is represented as a flush-seated control on a simplified solid panel insert.",
        )

    bin_joint = object_model.get_articulation("body_to_bin")
    knob_joint = object_model.get_articulation("panel_to_selector_knob")
    flap_joint = object_model.get_articulation("body_to_front_flap")
    cutter_0_joint = object_model.get_articulation("body_to_cutter_0")
    cutter_1_joint = object_model.get_articulation("body_to_cutter_1")

    button_ok = True
    for index in range(5):
        part = object_model.get_part(f"mode_button_{index}")
        joint = object_model.get_articulation(f"panel_to_mode_button_{index}")
        button_ok = button_ok and part is not None and joint.articulation_type == ArticulationType.PRISMATIC
    ctx.check("five_separate_mode_buttons", button_ok, "Expected five separate prismatic mode button parts.")

    ctx.check(
        "selector_knob_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )
    ctx.check(
        "cutter_cylinders_are_continuous",
        cutter_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and cutter_1_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={(cutter_0_joint.articulation_type, cutter_1_joint.articulation_type)!r}",
    )
    ctx.check(
        "second_cutter_counter_rotates",
        cutter_1_joint.mimic is not None and cutter_1_joint.mimic.multiplier == -1.0,
        details=f"mimic={cutter_1_joint.mimic!r}",
    )

    bin_limits = bin_joint.motion_limits
    if bin_limits is not None and bin_limits.upper is not None:
        with ctx.pose({bin_joint: 0.0}):
            ctx.expect_contact(
                bin_part,
                body,
                elem_a="side_flange_0",
                elem_b="rail_0",
                name="bin left flange rests on left rail",
            )
            ctx.expect_contact(
                bin_part,
                body,
                elem_a="side_flange_1",
                elem_b="rail_1",
                name="bin right flange rests on right rail",
            )
            closed_aabb = ctx.part_world_aabb(bin_part)

        with ctx.pose({bin_joint: bin_limits.upper}):
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                elem_a="side_flange_0",
                elem_b="rail_0",
                min_overlap=0.038,
                name="left flange stays engaged on extension",
            )
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                elem_a="side_flange_1",
                elem_b="rail_1",
                min_overlap=0.038,
                name="right flange stays engaged on extension",
            )
            open_aabb = ctx.part_world_aabb(bin_part)

        ctx.check(
            "bin_pulls_forward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.08,
            details=f"closed={closed_aabb!r}, open={open_aabb!r}",
        )

    button_joint = object_model.get_articulation("panel_to_mode_button_0")
    button_part = object_model.get_part("mode_button_0")
    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            "mode_button_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.001
            and pressed_pos[1] > rest_pos[1],
            details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    flap_limits = flap_joint.motion_limits
    if flap_limits is not None and flap_limits.upper is not None:
        closed_panel = ctx.part_element_world_aabb(front_flap, elem="flap_panel")
        with ctx.pose({flap_joint: flap_limits.upper}):
            open_panel = ctx.part_element_world_aabb(front_flap, elem="flap_panel")
        ctx.check(
            "front_flap_opens_outward",
            closed_panel is not None
            and open_panel is not None
            and open_panel[0][1] < closed_panel[0][1] - 0.02
            and open_panel[1][2] < closed_panel[1][2] - 0.01,
            details=f"closed={closed_panel!r}, open={open_panel!r}",
        )

    return ctx.report()


object_model = build_object_model()
