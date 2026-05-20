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


BODY_DEPTH = 0.340
BODY_WIDTH = 0.264
BODY_HEIGHT = 0.338

CHAMBER_CENTER_X = 0.026
CHAMBER_DEPTH = 0.216
CHAMBER_WIDTH = 0.154
CHAMBER_FLOOR_Z = 0.048

CONTROL_BANK_WIDTH = 0.176
CONTROL_BANK_HEIGHT = 0.150
CONTROL_BANK_CENTER_Z = 0.190
CONTROL_BANK_THICKNESS = 0.008

DIAL_Y_OFFSET = 0.052
DIAL_CENTER_Z = 0.230

BUTTON_CENTER_Z = 0.154
BUTTON_GUIDE_RADIUS = 0.0105
BUTTON_GUIDE_DEPTH = 0.022
BUTTON_TRAVEL = 0.004

LID_HINGE_X = BODY_DEPTH / 2.0 - 0.014
LID_HINGE_Z = BODY_HEIGHT + 0.011

LID_DEPTH = 0.292
LID_WIDTH = 0.246
LID_THICKNESS = 0.022
LID_REAR_MARGIN = 0.012
LID_WINDOW_DEPTH = 0.214
LID_WINDOW_WIDTH = 0.178
LID_WINDOW_CENTER_X = -0.154
LID_BARREL_RADIUS = 0.0055

TRAP_DOOR_DEPTH = 0.074
TRAP_DOOR_WIDTH = 0.056
TRAP_DOOR_THICKNESS = 0.010
TRAP_DOOR_HINGE_X = -0.132
TRAP_DOOR_CENTER_X = TRAP_DOOR_HINGE_X - (TRAP_DOOR_DEPTH / 2.0) - 0.006

SPINDLE_HEIGHT = 0.040
SPINDLE_RADIUS = 0.006


def _body_shell() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT).translate(
        (0.0, 0.0, BODY_HEIGHT / 2.0)
    )
    body = body.edges("|Z").fillet(0.024)

    chamber_void = (
        cq.Workplane("XY")
        .box(CHAMBER_DEPTH + 0.014, CHAMBER_WIDTH + 0.014, BODY_HEIGHT - CHAMBER_FLOOR_Z + 0.002)
        .edges("|Z")
        .fillet(0.028)
        .translate(
            (
                CHAMBER_CENTER_X,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_HEIGHT - CHAMBER_FLOOR_Z + 0.002) / 2.0,
            )
        )
    )
    body = body.cut(chamber_void)

    control_recess = cq.Workplane("XY").box(0.005, CONTROL_BANK_WIDTH, CONTROL_BANK_HEIGHT).translate(
        (-BODY_DEPTH / 2.0 + 0.0025, 0.0, CONTROL_BANK_CENTER_Z)
    )
    body = body.cut(control_recess)

    button_guide = (
        cq.Workplane("YZ")
        .circle(BUTTON_GUIDE_RADIUS)
        .extrude(BUTTON_GUIDE_DEPTH)
        .translate((-BODY_DEPTH / 2.0, 0.0, BUTTON_CENTER_Z))
    )
    body = body.cut(button_guide)

    spindle_socket = (
        cq.Workplane("XY")
        .circle(0.0055)
        .extrude(0.008, both=True)
        .translate((CHAMBER_CENTER_X, 0.0, CHAMBER_FLOOR_Z + 0.003))
    )
    body = body.cut(spindle_socket)

    for y_center in (-0.074, 0.074):
        hinge_lug = cq.Workplane("XY").box(0.016, 0.030, 0.010).translate(
            (BODY_DEPTH / 2.0 - 0.008, y_center, BODY_HEIGHT - 0.006)
        )
        body = body.union(hinge_lug)

    return body


def _control_bank_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(CONTROL_BANK_THICKNESS, CONTROL_BANK_WIDTH - 0.008, CONTROL_BANK_HEIGHT - 0.008)
    button_pocket = (
        cq.Workplane("YZ")
        .circle(0.0145)
        .extrude(0.0025, both=True)
        .translate((-0.0015, 0.0, BUTTON_CENTER_Z - CONTROL_BANK_CENTER_Z))
    )
    button_opening = (
        cq.Workplane("YZ")
        .circle(BUTTON_GUIDE_RADIUS + 0.0008)
        .extrude(0.010, both=True)
        .translate((0.0, 0.0, BUTTON_CENTER_Z - CONTROL_BANK_CENTER_Z))
    )
    return panel.cut(button_pocket).cut(button_opening)


def _lid_frame() -> cq.Workplane:
    lid = cq.Workplane("XY").box(LID_DEPTH, LID_WIDTH, LID_THICKNESS).translate(
        (-LID_DEPTH / 2.0 - LID_REAR_MARGIN, 0.0, 0.0)
    )
    window_cut = cq.Workplane("XY").box(LID_WINDOW_DEPTH, LID_WINDOW_WIDTH, LID_THICKNESS + 0.004).translate(
        (LID_WINDOW_CENTER_X, 0.0, 0.0)
    )
    lid = lid.cut(window_cut)

    handle = cq.Workplane("XY").box(0.030, 0.118, 0.014).translate(
        (-LID_DEPTH - LID_REAR_MARGIN + 0.016, 0.0, 0.011)
    )
    handle = handle.edges("|Z").fillet(0.004)
    lid = lid.union(handle)

    for y_center in (-0.060, 0.060):
        barrel = cq.Workplane("XZ").circle(LID_BARREL_RADIUS).extrude(0.022, both=True).translate(
            (0.0, y_center, 0.0)
        )
        bracket = cq.Workplane("XY").box(0.014, 0.044, 0.010).translate((-0.007, y_center, -0.001))
        lid = lid.union(barrel).union(bracket)

    return lid


def _lid_glass() -> cq.Workplane:
    pane = cq.Workplane("XY").box(LID_WINDOW_DEPTH + 0.014, LID_WINDOW_WIDTH + 0.014, 0.004).translate(
        (LID_WINDOW_CENTER_X, 0.0, -0.002)
    )
    trap_opening = cq.Workplane("XY").box(TRAP_DOOR_DEPTH + 0.004, TRAP_DOOR_WIDTH + 0.004, 0.010).translate(
        (TRAP_DOOR_CENTER_X, 0.0, -0.002)
    )
    return pane.cut(trap_opening)


def _trap_door_frame() -> cq.Workplane:
    door = cq.Workplane("XY").box(TRAP_DOOR_DEPTH, TRAP_DOOR_WIDTH, TRAP_DOOR_THICKNESS).translate(
        (-(TRAP_DOOR_DEPTH / 2.0) - 0.006, 0.0, 0.0)
    )
    inner_cut = cq.Workplane("XY").box(TRAP_DOOR_DEPTH - 0.018, TRAP_DOOR_WIDTH - 0.014, TRAP_DOOR_THICKNESS + 0.004).translate(
        (-(TRAP_DOOR_DEPTH / 2.0) - 0.006, 0.0, 0.0)
    )
    door = door.cut(inner_cut)

    lip = cq.Workplane("XY").box(0.016, 0.028, 0.008).translate((-(TRAP_DOOR_DEPTH + 0.004), 0.0, 0.003))
    door = door.union(lip)

    for y_center in (-0.014, 0.014):
        barrel = cq.Workplane("XZ").circle(0.0033).extrude(0.008, both=True).translate((0.0, y_center, 0.0))
        bracket = cq.Workplane("XY").box(0.008, 0.016, 0.006).translate((-0.004, y_center, -0.001))
        door = door.union(barrel).union(bracket)

    return door


def _trap_door_glass() -> cq.Workplane:
    return cq.Workplane("XY").box(TRAP_DOOR_DEPTH - 0.010, TRAP_DOOR_WIDTH - 0.006, 0.003).translate(
        (-(TRAP_DOOR_DEPTH / 2.0) - 0.006, 0.0, -0.001)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_bread_maker")

    model.material("stainless", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_trim", rgba=(0.12, 0.12, 0.12, 1.0))
    model.material("glass", rgba=(0.72, 0.84, 0.92, 0.35))
    model.material("spindle_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("button_accent", rgba=(0.24, 0.76, 0.39, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "bread_maker_body"),
        material="stainless",
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_control_bank_panel(), "bread_maker_control_bank"),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 + 0.004, 0.0, CONTROL_BANK_CENTER_Z)),
        material="dark_trim",
        name="control_bank",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame(), "bread_maker_lid_frame"),
        material="stainless",
        name="frame",
    )
    lid.visual(
        mesh_from_cadquery(_lid_glass(), "bread_maker_lid_glass"),
        material="glass",
        name="glass",
    )

    trap_door = model.part("trap_door")
    trap_door.visual(
        mesh_from_cadquery(_trap_door_frame(), "bread_maker_trap_door_frame"),
        material="stainless",
        name="frame",
    )
    trap_door.visual(
        mesh_from_cadquery(_trap_door_glass(), "bread_maker_trap_door_glass"),
        material="glass",
        name="glass",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="spindle_metal",
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material="spindle_metal",
        name="collar",
    )
    spindle.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="spindle_metal",
        name="bearing",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_HEIGHT / 2.0 + 0.003)),
        material="spindle_metal",
        name="shaft",
    )
    spindle.visual(
        Box((0.020, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="spindle_metal",
        name="drive_flat",
    )

    dial_knob = KnobGeometry(
        0.040,
        0.023,
        body_style="skirted",
        top_diameter=0.032,
        skirt=KnobSkirt(0.048, 0.005, flare=0.06),
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,
    )

    dial_0 = model.part("dial_0")
    dial_0.visual(
        mesh_from_geometry(dial_knob, "bread_maker_dial_0"),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material="dark_trim",
        name="knob",
    )

    dial_1 = model.part("dial_1")
    dial_1.visual(
        mesh_from_geometry(dial_knob, "bread_maker_dial_1"),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material="dark_trim",
        name="knob",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material="button_accent",
        name="cap",
    )
    start_button.visual(
        Cylinder(radius=0.0065, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_trim",
        name="stem",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.30),
    )

    model.articulation(
        "lid_to_trap_door",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=trap_door,
        origin=Origin(xyz=(TRAP_DOOR_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5, lower=0.0, upper=1.20),
    )

    model.articulation(
        "body_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(CHAMBER_CENTER_X, 0.0, CHAMBER_FLOOR_Z + 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=16.0),
    )

    model.articulation(
        "body_to_dial_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial_0,
        origin=Origin(xyz=(-BODY_DEPTH / 2.0, DIAL_Y_OFFSET, DIAL_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    model.articulation(
        "body_to_dial_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial_1,
        origin=Origin(xyz=(-BODY_DEPTH / 2.0, -DIAL_Y_OFFSET, DIAL_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(-BODY_DEPTH / 2.0, 0.0, BUTTON_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    trap_door = object_model.get_part("trap_door")
    spindle = object_model.get_part("spindle")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")
    start_button = object_model.get_part("start_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    trap_hinge = object_model.get_articulation("lid_to_trap_door")
    spindle_joint = object_model.get_articulation("body_to_spindle")
    dial_0_joint = object_model.get_articulation("body_to_dial_0")
    dial_1_joint = object_model.get_articulation("body_to_dial_1")
    button_joint = object_model.get_articulation("body_to_start_button")

    ctx.allow_isolated_part(
        start_button,
        reason="The start button is intentionally authored with guide clearance so the plunger can slide inside the front control-bank pocket.",
    )

    ctx.expect_overlap(lid, body, axes="xy", elem_a="glass", elem_b="shell", min_overlap=0.16, name="lid covers chamber footprint")
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="glass",
        negative_elem="shell",
        max_gap=0.020,
        max_penetration=0.0,
        name="closed lid glass stays above shell opening",
    )

    with ctx.pose({lid_hinge: 1.10}):
        opened_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            opened_aabb is not None and opened_aabb[1][2] > BODY_HEIGHT + 0.13,
            details=f"opened lid aabb={opened_aabb}",
        )

    ctx.expect_overlap(trap_door, lid, axes="xy", elem_a="glass", elem_b="glass", min_overlap=0.040, name="trap door sits within lid glass area")
    lid_aabb = ctx.part_world_aabb(lid)
    trap_aabb = ctx.part_world_aabb(trap_door)
    ctx.check(
        "trap door stays near lid top surface",
        lid_aabb is not None and trap_aabb is not None and abs(trap_aabb[1][2] - lid_aabb[1][2]) < 0.015,
        details=f"lid_aabb={lid_aabb}, trap_aabb={trap_aabb}",
    )

    with ctx.pose({trap_hinge: 0.95}):
        trap_open_aabb = ctx.part_world_aabb(trap_door)
        ctx.check(
            "trap door opens upward",
            trap_open_aabb is not None and trap_open_aabb[1][2] > BODY_HEIGHT + 0.06,
            details=f"opened trap door aabb={trap_open_aabb}",
        )

    spindle_pos = ctx.part_world_position(spindle)
    ctx.check(
        "spindle is centered in chamber",
        spindle_pos is not None
        and abs(spindle_pos[0] - CHAMBER_CENTER_X) < 0.001
        and abs(spindle_pos[1]) < 0.001
        and spindle_pos[2] > CHAMBER_FLOOR_Z - 0.001,
        details=f"spindle_pos={spindle_pos}",
    )
    ctx.check(
        "spindle rotates continuously",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spindle_joint.articulation_type}",
    )

    ctx.expect_gap(
        body,
        dial_0,
        axis="x",
        positive_elem="control_bank",
        negative_elem="knob",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial_0 mounts on control bank",
    )
    ctx.expect_gap(
        body,
        dial_1,
        axis="x",
        positive_elem="control_bank",
        negative_elem="knob",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial_1 mounts on control bank",
    )
    ctx.check(
        "both dials rotate continuously",
        dial_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_1_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"dial_0={dial_0_joint.articulation_type}, dial_1={dial_1_joint.articulation_type}",
    )

    button_rest = ctx.part_world_position(start_button)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        button_pressed = ctx.part_world_position(start_button)
    ctx.check(
        "start button presses inward",
        button_rest is not None and button_pressed is not None and button_pressed[0] > button_rest[0] + 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
