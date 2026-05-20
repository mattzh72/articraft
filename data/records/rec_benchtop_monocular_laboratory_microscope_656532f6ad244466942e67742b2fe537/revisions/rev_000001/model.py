from __future__ import annotations

from math import pi

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

BASE_X = 0.190
BASE_Y = 0.145
BASE_Z = 0.028

COLUMN_X = 0.022
COLUMN_Y = 0.024
COLUMN_H = 0.205
COLUMN_Y_CENTER = 0.022

BODY_HOME_Z = 0.155
BODY_TRAVEL = 0.032
STAGE_Z = 0.086
STAGE_TRAVEL = 0.018
STAGE_Y_OFFSET = -0.040
BODY_COLUMN_FACE_Y = COLUMN_Y_CENTER - (COLUMN_Y / 2.0) - 0.001


def _stand_shape():
    base = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_Z)
        .translate((0.0, 0.0, BASE_Z / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    pedestal = (
        cq.Workplane("XY")
        .box(0.078, 0.060, 0.014)
        .translate((0.0, 0.010, BASE_Z + 0.007))
        .edges("|Z")
        .fillet(0.006)
    )

    column = cq.Workplane("XY").box(COLUMN_X, COLUMN_Y, COLUMN_H).translate(
        (0.0, COLUMN_Y_CENTER, BASE_Z + (COLUMN_H / 2.0))
    )

    arm = (
        cq.Workplane("YZ")
        .moveTo(0.052, BASE_Z)
        .lineTo(0.052, 0.070)
        .threePointArc((0.045, 0.145), (0.034, 0.228))
        .lineTo(0.020, 0.228)
        .threePointArc((0.028, 0.148), (0.036, 0.066))
        .lineTo(0.036, BASE_Z)
        .close()
        .extrude(0.018)
        .translate((0.030, 0.0, 0.0))
    )

    top_bridge = cq.Workplane("XY").box(0.040, 0.016, 0.012).translate(
        (0.020, COLUMN_Y_CENTER + 0.002, 0.227)
    )

    return base.union(pedestal).union(column).union(arm).union(top_bridge)


def _stage_bracket_shape():
    bridge = cq.Workplane("XY").box(0.096, 0.060, 0.010).translate((0.0, 0.0, -0.009))
    left_rail = cq.Workplane("XY").box(0.088, 0.010, 0.008).translate((0.0, -0.029, -0.004))
    right_rail = cq.Workplane("XY").box(0.088, 0.010, 0.008).translate((0.0, 0.029, -0.004))
    mount_neck = cq.Workplane("XY").box(0.022, 0.034, 0.014).translate((0.0, 0.024, -0.012))
    column_mount = cq.Workplane("XY").box(0.028, 0.010, 0.024).translate((0.0, 0.044, -0.012))
    side_housing = cq.Workplane("XY").box(0.018, 0.046, 0.024).translate((0.054, 0.0, -0.012))
    return (
        bridge.union(left_rail)
        .union(right_rail)
        .union(mount_neck)
        .union(column_mount)
        .union(side_housing)
    )


def _stage_shape():
    plate = cq.Workplane("XY").box(0.120, 0.095, 0.006).translate((0.0, 0.0, 0.015))
    window_cut = cq.Workplane("XY").box(0.032, 0.026, 0.010).translate((0.0, 0.0, 0.015))
    connector = cq.Workplane("XY").box(0.040, 0.050, 0.014).translate((0.0, 0.0, 0.007))
    left_shoe = cq.Workplane("XY").box(0.055, 0.012, 0.004).translate((0.0, -0.029, 0.002))
    right_shoe = cq.Workplane("XY").box(0.055, 0.012, 0.004).translate((0.0, 0.029, 0.002))
    return plate.cut(window_cut).union(connector).union(left_shoe).union(right_shoe)


def _body_shape():
    slide_pad = cq.Workplane("XY").box(0.022, 0.008, 0.070).translate((0.0, -0.004, 0.0))
    prism_housing = cq.Workplane("XY").box(0.046, 0.050, 0.026).translate((0.0, -0.028, 0.017))
    shoulder = cq.Workplane("XY").box(0.032, 0.022, 0.018).translate((0.0, -0.040, 0.032))
    nose_mount = cq.Workplane("XY").circle(0.014).extrude(0.012).translate((0.0, -0.030, -0.001))

    return slide_pad.union(prism_housing).union(shoulder).union(nose_mount)


def _turret_shape():
    disk = cq.Workplane("XY").circle(0.020).extrude(0.010).translate((0.0, 0.0, -0.010))
    hub = cq.Workplane("XY").circle(0.010).extrude(0.004).translate((0.0, 0.0, -0.008))
    turret = disk.union(hub)

    for x_pos, y_pos in (
        (0.014, 0.000),
        (-0.007, 0.012),
        (-0.007, -0.012),
    ):
        barrel = cq.Workplane("XY").circle(0.006).extrude(0.024).translate((x_pos, y_pos, -0.034))
        turret = turret.union(barrel)

    return turret


def _ring_knob_shape(outer_radius: float, inner_radius: float, length: float):
    return cq.Workplane("YZ").circle(outer_radius).circle(inner_radius).extrude(length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_monocular_microscope")

    model.material("stand_cream", rgba=(0.92, 0.91, 0.84, 1.0))
    model.material("stage_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("metal_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("metal_light", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("glass_dark", rgba=(0.18, 0.22, 0.24, 0.55))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "microscope_stand"),
        material="stand_cream",
        name="stand_shell",
    )
    stand.visual(
        Box((0.028, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.012, STAGE_Z - 0.012)),
        material="stand_cream",
        name="stage_mount_pad",
    )
    stand.visual(
        Box((0.026, 0.006, 0.110)),
        origin=Origin(xyz=(0.0, 0.012, BODY_HOME_Z + (BODY_TRAVEL / 2.0))),
        material="stand_cream",
        name="body_guide",
    )

    stage_bracket = model.part("stage_bracket")
    stage_bracket.visual(
        mesh_from_cadquery(_stage_bracket_shape(), "microscope_stage_bracket"),
        material="metal_dark",
        name="stage_bracket_shell",
    )
    stage_bracket.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.065, 0.0, -0.012), rpy=(0.0, pi / 2.0, 0.0)),
        material="metal_dark",
        name="knob_boss",
    )

    stage = model.part("stage")
    stage.visual(
        mesh_from_cadquery(_stage_shape(), "microscope_stage"),
        material="stage_black",
        name="stage_plate",
    )
    stage.visual(
        Box((0.006, 0.032, 0.002)),
        origin=Origin(xyz=(-0.034, 0.0, 0.019)),
        material="metal_light",
        name="clip_0",
    )
    stage.visual(
        Box((0.006, 0.032, 0.002)),
        origin=Origin(xyz=(0.034, 0.0, 0.019)),
        material="metal_light",
        name="clip_1",
    )

    body = model.part("body")
    body.visual(
        Box((0.026, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material="stand_cream",
        name="slide_pad",
    )
    body.visual(
        Box((0.046, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, -0.026, 0.018)),
        material="stand_cream",
        name="prism_body",
    )
    body.visual(
        Box((0.032, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, -0.037, 0.032)),
        material="stand_cream",
        name="head_block",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, -0.036, 0.004)),
        material="stand_cream",
        name="nose_mount",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.058),
        origin=Origin(
            xyz=(0.0, -0.030, 0.057),
            rpy=(-0.46, 0.0, 0.0),
        ),
        material="metal_light",
        name="eyepiece_tube",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(
            xyz=(0.0, -0.017, 0.072),
            rpy=(-0.46, 0.0, 0.0),
        ),
        material="glass_dark",
        name="eyecup",
    )

    turret = model.part("turret")
    turret.visual(
        mesh_from_cadquery(_turret_shape(), "microscope_turret"),
        material="metal_light",
        name="nosepiece",
    )

    x_control = model.part("x_control")
    x_control.visual(
        mesh_from_cadquery(_ring_knob_shape(0.018, 0.0085, 0.016), "microscope_x_control"),
        material="knob_black",
        name="outer_knob",
    )

    y_control = model.part("y_control")
    y_control.visual(
        Cylinder(radius=0.0065, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="knob_black",
        name="inner_knob",
    )

    model.articulation(
        "stand_to_stage_bracket",
        ArticulationType.FIXED,
        parent=stand,
        child=stage_bracket,
        origin=Origin(xyz=(0.0, STAGE_Y_OFFSET, STAGE_Z)),
    )
    model.articulation(
        "stage_bracket_to_stage",
        ArticulationType.PRISMATIC,
        parent=stage_bracket,
        child=stage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=-STAGE_TRAVEL,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "stand_to_body",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=body,
        origin=Origin(xyz=(0.0, BODY_COLUMN_FACE_Y, BODY_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.10,
            lower=0.0,
            upper=BODY_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turret,
        origin=Origin(xyz=(0.0, -0.036, -0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    for name, child in (("stage_bracket_to_x_control", x_control), ("stage_bracket_to_y_control", y_control)):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=stage_bracket,
            child=child,
            origin=Origin(xyz=(0.070, 0.0, -0.012)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stage = object_model.get_part("stage")
    stage_bracket = object_model.get_part("stage_bracket")
    body = object_model.get_part("body")
    turret = object_model.get_part("turret")
    x_control = object_model.get_part("x_control")
    y_control = object_model.get_part("y_control")

    stage_slide = object_model.get_articulation("stage_bracket_to_stage")
    body_slide = object_model.get_articulation("stand_to_body")

    stage_limits = stage_slide.motion_limits
    body_limits = body_slide.motion_limits

    if stage_limits is not None and stage_limits.lower is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            left_pos = ctx.part_world_position(stage)
            ctx.expect_overlap(
                stage,
                stage_bracket,
                axes="x",
                min_overlap=0.060,
                name="stage stays engaged on bracket at left travel",
            )
        with ctx.pose({stage_slide: stage_limits.upper}):
            right_pos = ctx.part_world_position(stage)
            ctx.expect_overlap(
                stage,
                stage_bracket,
                axes="x",
                min_overlap=0.060,
                name="stage stays engaged on bracket at right travel",
            )
        ctx.check(
            "stage carriage moves left to right",
            left_pos is not None
            and right_pos is not None
            and right_pos[0] > left_pos[0] + 0.030,
            details=f"left={left_pos}, right={right_pos}",
        )

    if body_limits is not None and body_limits.lower is not None and body_limits.upper is not None:
        with ctx.pose({body_slide: body_limits.lower}):
            ctx.expect_gap(
                turret,
                stage,
                axis="z",
                min_gap=0.006,
                name="objective turret clears the stage at the low focus position",
            )
            ctx.expect_overlap(
                turret,
                stage,
                axes="xy",
                min_overlap=0.020,
                name="objective remains over the specimen stage",
            )

        low_pos = ctx.part_world_position(body)
        with ctx.pose({body_slide: body_limits.upper}):
            high_pos = ctx.part_world_position(body)
        ctx.check(
            "optical body rises on the column",
            low_pos is not None
            and high_pos is not None
            and high_pos[2] > low_pos[2] + 0.020,
            details=f"low={low_pos}, high={high_pos}",
        )

    x_pos = ctx.part_world_position(x_control)
    y_pos = ctx.part_world_position(y_control)
    ctx.check(
        "stage controls are coaxial",
        x_pos is not None
        and y_pos is not None
        and abs(x_pos[0] - y_pos[0]) < 1e-6
        and abs(x_pos[1] - y_pos[1]) < 1e-6
        and abs(x_pos[2] - y_pos[2]) < 1e-6,
        details=f"x_control={x_pos}, y_control={y_pos}",
    )

    return ctx.report()


object_model = build_object_model()
