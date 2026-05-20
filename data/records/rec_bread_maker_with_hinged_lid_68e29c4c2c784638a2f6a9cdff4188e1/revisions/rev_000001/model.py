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


BODY_WIDTH = 0.39
BODY_DEPTH = 0.30
BODY_HEIGHT = 0.31
BODY_BASE_Z = 0.01
CHAMBER_WIDTH = 0.252
CHAMBER_DEPTH = 0.162
CHAMBER_FLOOR_Z = 0.105
FRONT_RECESS_DEPTH = 0.009
LID_WIDTH = 0.332
LID_DEPTH = 0.228


def _body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT - BODY_BASE_Z, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_BASE_Z))
        .edges("|Z")
        .fillet(0.038)
        .edges(">Z")
        .fillet(0.018)
        .edges("<Z")
        .fillet(0.008)
    )

    for x_pos in (-0.125, 0.125):
        for y_pos in (-0.095, 0.095):
            foot = (
                cq.Workplane("XY")
                .box(0.055, 0.030, BODY_BASE_Z, centered=(True, True, False))
                .translate((x_pos, y_pos, 0.0))
            )
            shell = shell.union(foot)

    chamber_cutter = (
        cq.Workplane("XY")
        .box(CHAMBER_WIDTH, CHAMBER_DEPTH, BODY_HEIGHT - CHAMBER_FLOOR_Z + 0.01, centered=(True, True, False))
        .translate((0.0, 0.006, CHAMBER_FLOOR_Z))
        .edges("|Z")
        .fillet(0.020)
    )
    shell = shell.cut(chamber_cutter)

    spindle_hole = cq.Workplane("XY").circle(0.006).extrude(CHAMBER_FLOOR_Z + 0.004).translate((0.0, 0.006, 0.0))
    shell = shell.cut(spindle_hole)

    recess = (
        cq.Workplane("XY")
        .box(0.122, 0.012, 0.108, centered=(True, True, True))
        .translate((0.0, -BODY_DEPTH / 2.0 + 0.006, 0.136))
    )
    shell = shell.cut(recess)

    for x_pos in (-0.031, 0.031):
        mode_hole = (
            cq.Workplane("XY")
            .box(0.031, 0.022, 0.019, centered=(True, True, True))
            .translate((x_pos, -BODY_DEPTH / 2.0 + 0.011, 0.159))
        )
        shell = shell.cut(mode_hole)

    start_hole = (
        cq.Workplane("XY")
        .box(0.040, 0.022, 0.022, centered=(True, True, True))
        .translate((0.0, -BODY_DEPTH / 2.0 + 0.011, 0.121))
    )
    shell = shell.cut(start_hole)

    dial_recess = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(0.010)
        .translate((BODY_WIDTH / 2.0 - 0.010, -0.040, 0.196))
    )
    dial_hole = (
        cq.Workplane("YZ")
        .circle(0.010)
        .extrude(0.020)
        .translate((BODY_WIDTH / 2.0 - 0.020, -0.040, 0.196))
    )
    shell = shell.cut(dial_recess).cut(dial_hole)

    return shell


def _lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, 0.040, centered=(True, True, False))
        .translate((0.0, -LID_DEPTH / 2.0, 0.004))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.010)
        .faces("<Z")
        .shell(-0.0035)
    )

    hinge_barrel = cq.Workplane("YZ").circle(0.0065).extrude(0.300).translate((-0.150, 0.0, 0.0))
    lid = lid.union(hinge_barrel)

    pocket = (
        cq.Workplane("XY")
        .box(0.108, 0.072, 0.042, centered=(True, True, False))
        .translate((0.086, -0.072, 0.006))
        .edges("|Z")
        .fillet(0.010)
    )
    return lid.cut(pocket)


def _dispenser_flap_shape() -> cq.Workplane:
    flap = (
        cq.Workplane("XY")
        .box(0.104, 0.066, 0.008, centered=(True, True, False))
        .translate((0.0, -0.033, -0.001))
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.003)
    )
    barrel = cq.Workplane("YZ").circle(0.0068).extrude(0.090).translate((-0.045, 0.0, 0.0))
    return flap.union(barrel)


def _button_part(
    model: ArticulatedObject,
    name: str,
    size: tuple[float, float, float],
    material: str,
    visual_name: str,
    stem_name: str,
):
    button = model.part(name)
    button.visual(
        Box(size),
        origin=Origin(xyz=(0.0, size[1] / 2.0 - 0.002, 0.0)),
        material=material,
        name=visual_name,
    )
    button.visual(
        Box((size[0] * 0.72, 0.014, size[2] * 0.72)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=material,
        name=stem_name,
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_bread_maker")

    body_white = model.material("body_white", rgba=(0.92, 0.92, 0.90, 1.0))
    lid_white = model.material("lid_white", rgba=(0.96, 0.96, 0.94, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    mode_gray = model.material("mode_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    start_green = model.material("start_green", rgba=(0.19, 0.50, 0.27, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=body_white,
        name="body_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid_shell"),
        material=lid_white,
        name="lid_shell",
    )

    dispenser_flap = model.part("dispenser_flap")
    dispenser_flap.visual(
        mesh_from_cadquery(_dispenser_flap_shape(), "dispenser_flap"),
        material=trim_gray,
        name="flap_shell",
    )
    dispenser_flap.visual(
        Box((0.094, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        material=trim_gray,
        name="flap_hinge_tab",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="spindle_hub",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="spindle_pin",
    )
    spindle.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Box((0.060, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=steel,
        name="kneader_bar",
    )
    spindle.visual(
        Box((0.010, 0.038, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=steel,
        name="kneader_fin",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.035, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_black,
        name="dial_knob",
    )
    dial.visual(
        Box((0.004, 0.012, 0.006)),
        origin=Origin(xyz=(0.020, 0.017, 0.0)),
        material=steel,
        name="dial_pointer",
    )

    mode_button_0 = _button_part(
        model,
        name="mode_button_0",
        size=(0.026, 0.011, 0.015),
        material=mode_gray,
        visual_name="mode_button_cap_0",
        stem_name="mode_button_cap_0_stem",
    )
    mode_button_1 = _button_part(
        model,
        name="mode_button_1",
        size=(0.026, 0.011, 0.015),
        material=mode_gray,
        visual_name="mode_button_cap_1",
        stem_name="mode_button_cap_1_stem",
    )
    start_button = _button_part(
        model,
        name="start_button",
        size=(0.034, 0.012, 0.018),
        material=start_green,
        visual_name="start_button_cap",
        stem_name="start_button_cap_stem",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.123, 0.316499)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )

    model.articulation(
        "body_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.006, CHAMBER_FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=18.0),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - 0.003, -0.040, 0.196)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    model.articulation(
        "body_to_mode_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_0,
        origin=Origin(xyz=(-0.031, -BODY_DEPTH / 2.0 + FRONT_RECESS_DEPTH + 0.002, 0.159)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.003),
    )

    model.articulation(
        "body_to_mode_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_1,
        origin=Origin(xyz=(0.031, -BODY_DEPTH / 2.0 + FRONT_RECESS_DEPTH + 0.002, 0.159)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.003),
    )

    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + FRONT_RECESS_DEPTH + 0.002, 0.121)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    model.articulation(
        "lid_to_dispenser_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=dispenser_flap,
        origin=Origin(xyz=(0.086, -0.036, 0.0319)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    return model


def _upper_limit(joint, fallback: float) -> float:
    limits = joint.motion_limits
    if limits is None or limits.upper is None:
        return fallback
    return float(limits.upper)


def _y_position(ctx: TestContext, part) -> float | None:
    pos = ctx.part_world_position(part)
    if pos is None:
        return None
    return float(pos[1])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dispenser_flap = object_model.get_part("dispenser_flap")
    spindle = object_model.get_part("spindle")
    dial = object_model.get_part("dial")
    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")
    start_button = object_model.get_part("start_button")

    lid_joint = object_model.get_articulation("body_to_lid")
    flap_joint = object_model.get_articulation("lid_to_dispenser_flap")
    mode_joint_0 = object_model.get_articulation("body_to_mode_button_0")
    mode_joint_1 = object_model.get_articulation("body_to_mode_button_1")
    start_joint = object_model.get_articulation("body_to_start_button")

    ctx.allow_overlap(
        body,
        spindle,
        elem_a="body_shell",
        elem_b="spindle_pin",
        reason="The kneading spindle is intentionally represented with a short drive pin seated inside the chamber-floor bore.",
    )
    ctx.allow_overlap(
        body,
        mode_button_0,
        elem_a="body_shell",
        elem_b="mode_button_cap_0_stem",
        reason="The rear guide stem is intentionally simplified as sliding inside the front control opening proxy.",
    )
    ctx.allow_overlap(
        body,
        mode_button_1,
        elem_a="body_shell",
        elem_b="mode_button_cap_1_stem",
        reason="The rear guide stem is intentionally simplified as sliding inside the front control opening proxy.",
    )
    ctx.allow_overlap(
        body,
        start_button,
        elem_a="body_shell",
        elem_b="start_button_cap_stem",
        reason="The start button uses a simplified rear guide stem that is intentionally represented inside the front control opening proxy.",
    )
    ctx.allow_isolated_part(
        dispenser_flap,
        reason="The ingredient-dispenser flap is carried by a simplified hidden hinge representation rather than fully modeled contacting knuckles.",
    )

    with ctx.pose({lid_joint: 0.0, flap_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.012,
            max_penetration=0.0,
            elem_a="lid_shell",
            elem_b="body_shell",
            name="lid closes without penetrating the shell",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            elem_a="lid_shell",
            elem_b="body_shell",
            name="lid covers the top opening footprint",
        )
        ctx.expect_origin_gap(
            spindle,
            body,
            axis="z",
            min_gap=0.10,
            max_gap=0.11,
            name="spindle sits on the chamber floor height",
        )
        ctx.expect_origin_gap(
            dial,
            body,
            axis="x",
            min_gap=0.19,
            max_gap=0.20,
            name="dial mounts on the right side of the shell",
        )

    lid_rest_aabb = ctx.part_world_aabb(lid)
    lid_open_aabb = None
    with ctx.pose({lid_joint: _upper_limit(lid_joint, math.radians(112.0))}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.12,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    flap_rest_aabb = ctx.part_world_aabb(dispenser_flap)
    flap_open_aabb = None
    with ctx.pose({flap_joint: _upper_limit(flap_joint, math.radians(82.0))}):
        flap_open_aabb = ctx.part_world_aabb(dispenser_flap)
    ctx.check(
        "dispenser flap opens upward",
        flap_rest_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][2] > flap_rest_aabb[1][2] + 0.025,
        details=f"rest={flap_rest_aabb}, open={flap_open_aabb}",
    )

    rest_mode_0 = _y_position(ctx, mode_button_0)
    rest_mode_1 = _y_position(ctx, mode_button_1)
    rest_start = _y_position(ctx, start_button)

    with ctx.pose({mode_joint_0: _upper_limit(mode_joint_0, 0.005)}):
        pressed_mode_0 = _y_position(ctx, mode_button_0)
        unpressed_mode_1 = _y_position(ctx, mode_button_1)
        unpressed_start = _y_position(ctx, start_button)
    ctx.check(
        "mode button 0 presses independently",
        rest_mode_0 is not None
        and pressed_mode_0 is not None
        and pressed_mode_0 > rest_mode_0 + 0.002
        and rest_mode_1 is not None
        and unpressed_mode_1 is not None
        and abs(unpressed_mode_1 - rest_mode_1) < 1e-6
        and rest_start is not None
        and unpressed_start is not None
        and abs(unpressed_start - rest_start) < 1e-6,
        details=(
            f"rest_mode_0={rest_mode_0}, pressed_mode_0={pressed_mode_0}, "
            f"rest_mode_1={rest_mode_1}, unpressed_mode_1={unpressed_mode_1}, "
            f"rest_start={rest_start}, unpressed_start={unpressed_start}"
        ),
    )

    with ctx.pose({mode_joint_1: _upper_limit(mode_joint_1, 0.005)}):
        pressed_mode_1 = _y_position(ctx, mode_button_1)
        unpressed_mode_0 = _y_position(ctx, mode_button_0)
    ctx.check(
        "mode button 1 presses independently",
        rest_mode_1 is not None
        and pressed_mode_1 is not None
        and pressed_mode_1 > rest_mode_1 + 0.002
        and rest_mode_0 is not None
        and unpressed_mode_0 is not None
        and abs(unpressed_mode_0 - rest_mode_0) < 1e-6,
        details=(
            f"rest_mode_1={rest_mode_1}, pressed_mode_1={pressed_mode_1}, "
            f"rest_mode_0={rest_mode_0}, unpressed_mode_0={unpressed_mode_0}"
        ),
    )

    with ctx.pose({start_joint: _upper_limit(start_joint, 0.006)}):
        pressed_start = _y_position(ctx, start_button)
        unpressed_mode_0 = _y_position(ctx, mode_button_0)
        unpressed_mode_1 = _y_position(ctx, mode_button_1)
    ctx.check(
        "start button presses independently",
        rest_start is not None
        and pressed_start is not None
        and pressed_start > rest_start + 0.003
        and rest_mode_0 is not None
        and unpressed_mode_0 is not None
        and abs(unpressed_mode_0 - rest_mode_0) < 1e-6
        and rest_mode_1 is not None
        and unpressed_mode_1 is not None
        and abs(unpressed_mode_1 - rest_mode_1) < 1e-6,
        details=(
            f"rest_start={rest_start}, pressed_start={pressed_start}, "
            f"rest_mode_0={rest_mode_0}, unpressed_mode_0={unpressed_mode_0}, "
            f"rest_mode_1={rest_mode_1}, unpressed_mode_1={unpressed_mode_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
