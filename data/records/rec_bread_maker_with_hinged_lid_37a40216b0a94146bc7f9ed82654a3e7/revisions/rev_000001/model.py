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


BODY_DEPTH = 0.270
BODY_WIDTH = 0.180
BODY_HEIGHT = 0.340
CHAMBER_DEPTH = 0.145
CHAMBER_WIDTH = 0.104
CHAMBER_FLOOR_Z = 0.045
CHAMBER_X = -0.015

POD_BASE_DEPTH = 0.036
POD_BASE_WIDTH = 0.132
POD_BASE_HEIGHT = 0.086
POD_BASE_Z = 0.185

POD_TOP_DEPTH = 0.020
POD_TOP_WIDTH = 0.102
POD_TOP_HEIGHT = 0.074
POD_TOP_Z = 0.205
POD_FACE_X = BODY_DEPTH * 0.5 + POD_BASE_DEPTH + POD_TOP_DEPTH
CONTROL_PANEL_DEPTH = 0.004
CONTROL_FACE_X = POD_FACE_X - CONTROL_PANEL_DEPTH
KNOB_CENTER_Z = POD_TOP_Z + POD_TOP_HEIGHT * 0.5
BUTTON_TRAVEL = 0.003
BUTTON_DEPTH = 0.010
BUTTON_FACE = 0.016
BUTTON_POSITIONS = (
    ("preset_button_0", -0.031, KNOB_CENTER_Z),
    ("preset_button_1", 0.031, KNOB_CENTER_Z),
    ("preset_button_2", 0.000, KNOB_CENTER_Z + 0.028),
    ("preset_button_3", 0.000, KNOB_CENTER_Z - 0.028),
)
LATCH_FACE_X = BODY_DEPTH * 0.5 - 0.003
LATCH_Z = 0.163
LATCH_TRAVEL = 0.004

LID_DEPTH = CHAMBER_DEPTH + 0.024
LID_WIDTH = CHAMBER_WIDTH + 0.030
LID_THICKNESS = 0.028
LID_HINGE_X = CHAMBER_X - CHAMBER_DEPTH * 0.5 - 0.012


def _body_shell() -> object:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )

    chamber_cut = (
        cq.Workplane("XY")
        .box(
            CHAMBER_DEPTH,
            CHAMBER_WIDTH,
            BODY_HEIGHT - CHAMBER_FLOOR_Z + 0.004,
            centered=(True, True, False),
        )
        .translate((CHAMBER_X, 0.0, CHAMBER_FLOOR_Z))
    )
    shell = shell.cut(chamber_cut)

    pod_base = (
        cq.Workplane("XY")
        .box(POD_BASE_DEPTH, POD_BASE_WIDTH, POD_BASE_HEIGHT, centered=(True, True, False))
        .translate((BODY_DEPTH * 0.5 + POD_BASE_DEPTH * 0.5, 0.0, POD_BASE_Z))
    )
    pod_top = (
        cq.Workplane("XY")
        .box(POD_TOP_DEPTH, POD_TOP_WIDTH, POD_TOP_HEIGHT, centered=(True, True, False))
        .translate(
            (
                BODY_DEPTH * 0.5 + POD_BASE_DEPTH + POD_TOP_DEPTH * 0.5,
                0.0,
                POD_TOP_Z,
            )
        )
    )
    shell = shell.union(pod_base).union(pod_top)

    control_panel_cut = (
        cq.Workplane("XY")
        .box(
            CONTROL_PANEL_DEPTH,
            0.086,
            0.082,
            centered=(True, True, True),
        )
        .translate((POD_FACE_X - CONTROL_PANEL_DEPTH * 0.5, 0.0, KNOB_CENTER_Z))
    )
    latch_cut = (
        cq.Workplane("XY")
        .box(0.005, 0.058, 0.022, centered=(True, True, True))
        .translate((BODY_DEPTH * 0.5 - 0.0025, 0.0, LATCH_Z))
    )
    shell = shell.cut(control_panel_cut).cut(latch_cut)

    return shell


def _lid_shell() -> object:
    shell = cq.Workplane("XY").box(
        LID_DEPTH,
        LID_WIDTH,
        LID_THICKNESS,
        centered=(False, True, False),
    )
    shell = shell.edges("|Z").fillet(0.005)

    pocket = (
        cq.Workplane("XY")
        .box(
            LID_DEPTH - 0.022,
            LID_WIDTH - 0.020,
            LID_THICKNESS - 0.008,
            centered=(False, True, False),
        )
        .translate((0.011, 0.0, 0.0))
    )
    shell = shell.cut(pocket)

    handle_lip = (
        cq.Workplane("XY")
        .box(0.020, 0.056, 0.012, centered=(True, True, False))
        .translate((LID_DEPTH - 0.002, 0.0, 0.006))
    )
    return shell.union(handle_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_loaf_bread_maker")

    shell_white = model.material("shell_white", rgba=(0.88, 0.89, 0.87, 1.0))
    lid_white = model.material("lid_white", rgba=(0.92, 0.93, 0.91, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    button_grey = model.material("button_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    latch_grey = model.material("latch_grey", rgba=(0.62, 0.65, 0.67, 1.0))
    paddle_dark = model.material("paddle_dark", rgba=(0.24, 0.24, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "bread_maker_body"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(CHAMBER_X, 0.0, CHAMBER_FLOOR_Z + 0.003)),
        material=dark_trim,
        name="chamber_boss",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "bread_maker_lid"),
        material=lid_white,
        name="lid_shell",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=paddle_dark,
        name="hub",
    )
    paddle.visual(
        Box((0.058, 0.014, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, 0.015)),
        material=paddle_dark,
        name="blade",
    )
    paddle.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(-0.014, 0.0, 0.020), rpy=(0.0, 0.0, 0.40)),
        material=paddle_dark,
        name="tail_fin",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.024,
                body_style="skirted",
                top_diameter=0.029,
                skirt=KnobSkirt(0.040, 0.006, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "bread_maker_selector_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="knob_shell",
    )

    for button_name, button_y, button_z in BUTTON_POSITIONS:
        preset_button = model.part(button_name)
        preset_button.visual(
            Box((0.004, 0.010, 0.010)),
            origin=Origin(xyz=(0.002, 0.0, 0.0)),
            material=dark_trim,
            name="stem",
        )
        preset_button.visual(
            Box((BUTTON_DEPTH, BUTTON_FACE, BUTTON_FACE)),
            origin=Origin(xyz=(BUTTON_DEPTH * 0.5, 0.0, 0.0)),
            material=button_grey,
            name="cap",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=preset_button,
            origin=Origin(xyz=(CONTROL_FACE_X, button_y, button_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.002, 0.016, 0.008)),
        origin=Origin(xyz=(-0.001, 0.0, 0.0)),
        material=dark_trim,
        name="guide",
    )
    latch_button.visual(
        Box((0.005, 0.036, 0.010)),
        origin=Origin(xyz=(0.0025, 0.0, 0.0)),
        material=dark_trim,
        name="stem",
    )
    latch_button.visual(
        Box((0.012, 0.048, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=latch_grey,
        name="cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "body_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=paddle,
        origin=Origin(xyz=(CHAMBER_X, 0.0, CHAMBER_FLOOR_Z + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(CONTROL_FACE_X, 0.0, KNOB_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(LATCH_FACE_X, 0.0, LATCH_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=LATCH_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    paddle_joint = object_model.get_articulation("body_to_paddle")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    latch_button = object_model.get_part("latch_button")
    latch_joint = object_model.get_articulation("body_to_latch_button")

    closed_aabb = None
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            name="lid seats on the top rim when closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            min_overlap=0.100,
            name="lid spans the loaf chamber width",
        )
        closed_aabb = ctx.part_world_aabb(lid)

    upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    with ctx.pose({lid_hinge: upper if upper is not None else 1.4}):
        opened_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.10
            and opened_aabb[1][0] < closed_aabb[1][0] - 0.02,
            details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
        )

    ctx.check(
        "paddle joint is continuous",
        paddle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={paddle_joint.articulation_type}",
    )
    ctx.check(
        "selector knob joint is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    preset_parts = {
        button_name: object_model.get_part(button_name)
        for button_name, _, _ in BUTTON_POSITIONS
    }
    preset_joints = {
        button_name: object_model.get_articulation(f"body_to_{button_name}")
        for button_name, _, _ in BUTTON_POSITIONS
    }
    rest_positions = {
        button_name: ctx.part_world_position(part)
        for button_name, part in preset_parts.items()
    }
    for button_name, part in preset_parts.items():
        joint = preset_joints[button_name]
        upper = joint.motion_limits.upper if joint.motion_limits is not None else BUTTON_TRAVEL
        with ctx.pose({joint: upper if upper is not None else BUTTON_TRAVEL}):
            pressed_positions = {
                name: ctx.part_world_position(other_part)
                for name, other_part in preset_parts.items()
            }
        target_rest = rest_positions[button_name]
        target_pressed = pressed_positions[button_name]
        others_still = True
        for other_name, other_rest in rest_positions.items():
            if other_name == button_name:
                continue
            other_pressed = pressed_positions[other_name]
            if (
                other_rest is None
                or other_pressed is None
                or abs(other_pressed[0] - other_rest[0]) > 1e-6
            ):
                others_still = False
                break
        ctx.check(
            f"{button_name} presses inward independently",
            target_rest is not None
            and target_pressed is not None
            and target_pressed[0] < target_rest[0] - 0.0015
            and others_still,
            details=f"rest={target_rest}, pressed={target_pressed}, all_pressed={pressed_positions}",
        )

    latch_rest = ctx.part_world_position(latch_button)
    latch_upper = latch_joint.motion_limits.upper if latch_joint.motion_limits is not None else LATCH_TRAVEL
    with ctx.pose({latch_joint: latch_upper if latch_upper is not None else LATCH_TRAVEL}):
        latch_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "front latch button presses into the shell",
        latch_rest is not None
        and latch_pressed is not None
        and latch_pressed[0] < latch_rest[0] - 0.002,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
