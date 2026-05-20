from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.235
BODY_D = 0.160
BODY_H = 0.335
BODY_WALL = 0.0045
BODY_CORNER_R = 0.024
BODY_TOP_R = 0.014

BIN_OPEN_W = 0.206
BIN_OPEN_H = 0.148
BIN_TRAVEL = 0.050

BIN_W = 0.202
BIN_H = 0.138
BIN_D = 0.142
BIN_FRONT_OVERHANG = 0.010
BIN_WALL = 0.003


def _rounded_box(width: float, depth: float, height: float, *, corner_r: float, top_r: float):
    solid = cq.Workplane("XY").box(width, depth, height).translate((0.0, 0.0, height * 0.5))
    if corner_r > 0.0:
        solid = solid.edges("|Z").fillet(corner_r)
    if top_r > 0.0:
        solid = solid.faces(">Z").edges().fillet(top_r)
    return solid


def _build_body_shell():
    outer = _rounded_box(
        BODY_W,
        BODY_D,
        BODY_H,
        corner_r=BODY_CORNER_R,
        top_r=BODY_TOP_R,
    )
    inner = _rounded_box(
        BODY_W - 2.0 * BODY_WALL,
        BODY_D - 2.0 * BODY_WALL,
        BODY_H - 2.0 * BODY_WALL,
        corner_r=max(BODY_CORNER_R - BODY_WALL, 0.002),
        top_r=max(BODY_TOP_R - BODY_WALL * 0.5, 0.001),
    ).translate((0.0, 0.0, BODY_WALL))
    shell = outer.cut(inner)

    opening = (
        cq.Workplane("XY")
        .box(BIN_OPEN_W, BODY_WALL * 9.0, BIN_OPEN_H)
        .translate((0.0, -BODY_D * 0.5 + BODY_WALL * 4.0, BIN_OPEN_H * 0.5 + BODY_WALL + 0.001))
    )
    throat = (
        cq.Workplane("XY")
        .box(0.162, 0.024, BODY_WALL * 7.5)
        .translate((0.0, -0.006, BODY_H - BODY_WALL * 2.0))
    )
    return shell.cut(opening).cut(throat)


def _build_bin_shell():
    center_y = (BIN_D - BIN_FRONT_OVERHANG * 2.0) * 0.5
    outer = (
        cq.Workplane("XY")
        .box(BIN_W, BIN_D, BIN_H)
        .translate((0.0, center_y, BIN_H * 0.5))
        .edges("|Z")
        .fillet(0.016)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )
    inner = (
        cq.Workplane("XY")
        .box(BIN_W - 2.0 * BIN_WALL, BIN_D - 2.0 * BIN_WALL, BIN_H - BIN_WALL)
        .translate((0.0, center_y, (BIN_H + BIN_WALL) * 0.5))
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )
    scoop = (
        cq.Workplane("XY")
        .box(0.088, 0.020, 0.028)
        .translate((0.0, -0.006, 0.096))
    )
    return outer.cut(inner).cut(scoop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_shredder")

    body_dark = model.material("body_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    bin_dark = model.material("bin_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    metal = model.material("metal", rgba=(0.63, 0.66, 0.68, 1.0))
    control_black = model.material("control_black", rgba=(0.11, 0.11, 0.12, 1.0))
    button_red = model.material("button_red", rgba=(0.62, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "shredder_body_shell"),
        material=body_dark,
        name="body_shell",
    )
    body.visual(
        Box((0.182, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.006, BODY_H - 0.0045)),
        material=trim_dark,
        name="throat_trim",
    )
    body.visual(
        Box((0.154, 0.058, 0.010)),
        origin=Origin(xyz=(-0.004, 0.040, BODY_H - 0.005)),
        material=trim_dark,
        name="control_pod",
    )
    body.visual(
        Box((0.020, 0.034, 0.004)),
        origin=Origin(xyz=(-0.055, 0.040, BODY_H + 0.0018)),
        material=trim_dark,
        name="rocker_bezel",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.006, 0.040, BODY_H + 0.002), rpy=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_bezel",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.044, 0.040, BODY_H + 0.002), rpy=(0.0, 0.0, 0.0)),
        material=trim_dark,
        name="dial_bezel",
    )
    for support_index, support_y in enumerate((-0.0105, 0.0105)):
        for side_index, support_x in enumerate((-0.108, 0.108)):
            body.visual(
                Box((0.012, 0.014, 0.020)),
                origin=Origin(xyz=(support_x, support_y, BODY_H - 0.0175)),
                material=trim_dark,
                name=f"drum_support_{support_index}_{side_index}",
            )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_build_bin_shell(), "shredder_bin_shell"),
        material=bin_dark,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.120, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, 0.094)),
        material=trim_dark,
        name="bin_grip",
    )
    bin_part.inertial = Inertial.from_geometry(
        Box((BIN_W, BIN_D, BIN_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.060, BIN_H * 0.5)),
    )

    bin_origin_y = -BODY_D * 0.5 + BODY_WALL * 1.3
    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, bin_origin_y, BODY_WALL + 0.001)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.12,
            lower=0.0,
            upper=BIN_TRAVEL,
        ),
    )

    for drum_index, drum_y in enumerate((-0.0105, 0.0105)):
        drum = model.part(f"drum_{drum_index}")
        drum.visual(
            Cylinder(radius=0.0025, length=0.204),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=metal,
            name="drum_journal",
        )
        drum.visual(
            Cylinder(radius=0.0075, length=0.176),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=metal,
            name="drum_core",
        )
        for ring_index, ring_x in enumerate((-0.072, -0.054, -0.036, -0.018, 0.0, 0.018, 0.036, 0.054, 0.072)):
            drum.visual(
                Cylinder(radius=0.0098, length=0.011),
                origin=Origin(xyz=(ring_x, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
                material=metal,
                name=f"cutter_ring_{ring_index}",
            )
        drum.inertial = Inertial.from_geometry(
            Cylinder(radius=0.010, length=0.176),
            mass=0.12,
            origin=Origin(),
        )
        model.articulation(
            f"body_to_drum_{drum_index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=drum,
            origin=Origin(xyz=(0.0, drum_y, BODY_H - 0.0175)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=14.0),
        )

    rocker = model.part("reverse_rocker")
    rocker.visual(
        Box((0.012, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=control_black,
        name="rocker_cap",
    )
    rocker.visual(
        Box((0.012, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=control_black,
        name="rocker_axle",
    )
    rocker.inertial = Inertial.from_geometry(
        Box((0.012, 0.028, 0.010)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    model.articulation(
        "body_to_reverse_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(-0.055, 0.040, BODY_H + 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    pulse_button = model.part("pulse_button")
    pulse_button.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_red,
        name="button_cap",
    )
    pulse_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.007, length=0.006),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "body_to_pulse_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pulse_button,
        origin=Origin(xyz=(-0.006, 0.040, BODY_H + 0.002)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.003,
        ),
    )

    jam_dial = model.part("jam_dial")
    jam_dial.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=control_black,
        name="dial_shaft",
    )
    jam_dial.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=control_black,
        name="dial_cap",
    )
    jam_dial.visual(
        Box((0.003, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0095, 0.012)),
        material=metal,
        name="dial_indicator",
    )
    jam_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.012),
        mass=0.024,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    model.articulation(
        "body_to_jam_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=jam_dial,
        origin=Origin(xyz=(0.044, 0.040, BODY_H + 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    bin_joint = object_model.get_articulation("body_to_bin")
    pulse_button = object_model.get_part("pulse_button")
    pulse_joint = object_model.get_articulation("body_to_pulse_button")
    rocker_joint = object_model.get_articulation("body_to_reverse_rocker")
    jam_joint = object_model.get_articulation("body_to_jam_dial")
    drum_0_joint = object_model.get_articulation("body_to_drum_0")
    drum_1_joint = object_model.get_articulation("body_to_drum_1")
    drum_0 = object_model.get_part("drum_0")
    drum_1 = object_model.get_part("drum_1")

    ctx.allow_overlap(
        body,
        bin_part,
        elem_a="body_shell",
        elem_b="bin_shell",
        reason="The outer housing is represented as a simplified closed shell proxy around the bin cavity, so the hidden nested bin fit is intentionally approximated as embedding inside that proxy.",
    )
    ctx.allow_overlap(
        body,
        drum_0,
        reason="The forward cutter drum journals are intentionally seated into the simplified housing shell, which stands in for internal bearing pockets.",
    )
    ctx.allow_overlap(
        body,
        drum_1,
        reason="The rear cutter drum journals are intentionally seated into the simplified housing shell, which stands in for internal bearing pockets.",
    )

    with ctx.pose({bin_joint: 0.0}):
        ctx.expect_within(
            bin_part,
            body,
            axes="xz",
            inner_elem="bin_shell",
            outer_elem="body_shell",
            margin=0.004,
            name="bin stays centered in the body opening when closed",
        )
        ctx.expect_overlap(
            bin_part,
            body,
            axes="x",
            elem_a="bin_shell",
            elem_b="body_shell",
            min_overlap=0.16,
            name="closed bin spans the shredder width",
        )

    rest_pos = ctx.part_world_position(bin_part)
    upper = bin_joint.motion_limits.upper if bin_joint.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({bin_joint: upper}):
            ctx.expect_within(
                bin_part,
                body,
                axes="xz",
                inner_elem="bin_shell",
                outer_elem="body_shell",
                margin=0.004,
                name="extended bin stays guided by the body opening",
            )
            ctx.expect_overlap(
                bin_part,
                body,
                axes="y",
                elem_a="bin_shell",
                elem_b="body_shell",
                min_overlap=0.060,
                name="extended bin keeps retained insertion",
            )
            extended_pos = ctx.part_world_position(bin_part)
        ctx.check(
            "bin slides forward out of the housing",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.03,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    button_rest = ctx.part_world_position(pulse_button)
    button_upper = pulse_joint.motion_limits.upper if pulse_joint.motion_limits is not None else None
    if button_upper is not None:
        with ctx.pose({pulse_joint: button_upper}):
            button_pressed = ctx.part_world_position(pulse_button)
        ctx.check(
            "pulse button presses downward",
            button_rest is not None and button_pressed is not None and button_pressed[2] < button_rest[2] - 0.0015,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    rocker_limits = rocker_joint.motion_limits
    ctx.check(
        "reverse rocker has a short-axis pivot range",
        rocker_joint.axis == (1.0, 0.0, 0.0)
        and rocker_limits is not None
        and rocker_limits.lower is not None
        and rocker_limits.upper is not None
        and rocker_limits.lower < 0.0 < rocker_limits.upper,
        details=f"axis={rocker_joint.axis}, limits={rocker_limits}",
    )
    ctx.check(
        "jam dial rotates continuously about vertical axis",
        jam_joint.axis == (0.0, 0.0, 1.0)
        and jam_joint.motion_limits is not None
        and jam_joint.motion_limits.lower is None
        and jam_joint.motion_limits.upper is None,
        details=f"axis={jam_joint.axis}, limits={jam_joint.motion_limits}",
    )
    ctx.check(
        "cutter drums rotate on parallel horizontal axes",
        drum_0_joint.axis == (1.0, 0.0, 0.0)
        and drum_1_joint.axis == (1.0, 0.0, 0.0)
        and drum_0_joint.motion_limits is not None
        and drum_1_joint.motion_limits is not None
        and drum_0_joint.motion_limits.lower is None
        and drum_0_joint.motion_limits.upper is None
        and drum_1_joint.motion_limits.lower is None
        and drum_1_joint.motion_limits.upper is None,
        details=(
            f"drum_0 axis={drum_0_joint.axis}, limits={drum_0_joint.motion_limits}; "
            f"drum_1 axis={drum_1_joint.axis}, limits={drum_1_joint.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
