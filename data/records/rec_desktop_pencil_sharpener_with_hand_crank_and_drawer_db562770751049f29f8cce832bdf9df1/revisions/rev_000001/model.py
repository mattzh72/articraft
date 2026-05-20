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


BODY_W = 0.066
BODY_D = 0.054
BODY_H = 0.080

TRAY_W = 0.051
TRAY_D = 0.042
TRAY_H = 0.020
TRAY_WALL = 0.0018
TRAY_TRAVEL = 0.028

CRANK_AXIS_Y = 0.004
CRANK_AXIS_Z = 0.046


def _body_shell_mesh():
    shell_bottom = 0.026
    shell_height = BODY_H - shell_bottom

    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, shell_height)
        .translate((0.0, 0.0, shell_bottom + shell_height * 0.5))
        .edges()
        .fillet(0.006)
    )

    port_counterbore = (
        cq.Workplane("XZ")
        .center(-0.0030, 0.0550)
        .circle(0.0084)
        .extrude(0.0045)
        .translate((0.0, -BODY_D * 0.5 - 0.0010, 0.0))
    )
    port_bore = (
        cq.Workplane("XZ")
        .center(-0.0030, 0.0550)
        .circle(0.0056)
        .extrude(0.0190)
        .translate((0.0, -BODY_D * 0.5 - 0.0010, 0.0))
    )
    body = body.cut(port_counterbore).cut(port_bore)

    axle_hole = (
        cq.Workplane("YZ")
        .center(CRANK_AXIS_Y, CRANK_AXIS_Z)
        .circle(0.0033)
        .extrude(0.0100)
        .translate((BODY_W * 0.5 - 0.0090, 0.0, 0.0))
    )
    body = body.cut(axle_hole)

    return mesh_from_cadquery(body, "pencil_sharpener_upper_shell")


def _tray_mesh():
    outer = (
        cq.Workplane("XY")
        .box(TRAY_W, TRAY_D, TRAY_H)
        .translate((0.0, 0.0180, TRAY_H * 0.5))
        .edges("|Z")
        .fillet(0.0018)
    )
    inner = (
        cq.Workplane("XY")
        .box(TRAY_W - (2.0 * TRAY_WALL), TRAY_D - (2.0 * TRAY_WALL), TRAY_H)
        .translate((0.0, 0.0180, 0.0118))
    )
    tray = outer.cut(inner)
    return mesh_from_cadquery(tray, "pencil_sharpener_waste_tray")


def _knob_mesh():
    knob = cq.Workplane("YZ").circle(0.006).extrude(0.014)
    bore = cq.Workplane("YZ").circle(0.0032).extrude(0.0105)
    knob = knob.cut(bore)
    return mesh_from_cadquery(knob, "pencil_sharpener_crank_knob")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pencil_sharpener")

    shell_plastic = model.material("shell_plastic", rgba=(0.20, 0.22, 0.25, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.42, 0.43, 0.46, 1.0))
    knob_plastic = model.material("knob_plastic", rgba=(0.14, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=shell_plastic, name="body_shell")
    body.visual(
        Cylinder(radius=0.0072, length=0.0040),
        origin=Origin(
            xyz=(BODY_W * 0.5 - 0.0020, CRANK_AXIS_Y, CRANK_AXIS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_metal,
        name="axle_bushing",
    )
    body.visual(
        Box((BODY_W * 0.72, BODY_D * 0.70, 0.0040)),
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=tray_plastic,
        name="base_plate",
    )
    body.visual(
        Box((0.006, 0.041, 0.026)),
        origin=Origin(xyz=(-0.030, -0.0045, 0.013)),
        material=shell_plastic,
        name="lower_wall_0",
    )
    body.visual(
        Box((0.006, 0.041, 0.026)),
        origin=Origin(xyz=(0.030, -0.0045, 0.013)),
        material=shell_plastic,
        name="lower_wall_1",
    )
    body.visual(
        Box((BODY_W, 0.011, 0.026)),
        origin=Origin(xyz=(0.0, 0.0215, 0.013)),
        material=shell_plastic,
        name="rear_wall",
    )

    waste_tray = model.part("waste_tray")
    waste_tray.visual(_tray_mesh(), material=tray_plastic, name="tray_bin")
    waste_tray.visual(
        Box((0.028, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.0010, 0.0130)),
        material=dark_metal,
        name="tray_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0046, length=0.010),
        origin=Origin(xyz=(0.0050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_metal,
        name="crank_hub",
    )
    crank.visual(
        Box((0.005, 0.005, 0.024)),
        origin=Origin(xyz=(0.0110, -0.0030, 0.0120)),
        material=warm_metal,
        name="crank_arm",
    )
    crank.visual(
        Box((0.005, 0.018, 0.005)),
        origin=Origin(xyz=(0.0110, -0.0085, 0.0215)),
        material=warm_metal,
        name="crank_bridge",
    )
    crank.visual(
        Cylinder(radius=0.0029, length=0.009),
        origin=Origin(xyz=(0.0145, -0.0140, 0.0215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="crank_peg",
    )

    crank_knob = model.part("crank_knob")
    crank_knob.visual(_knob_mesh(), material=knob_plastic, name="knob_body")

    model.articulation(
        "body_to_waste_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=waste_tray,
        origin=Origin(xyz=(0.0, -0.0250, 0.0040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_W * 0.5, CRANK_AXIS_Y, CRANK_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    model.articulation(
        "crank_to_crank_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=crank_knob,
        origin=Origin(xyz=(0.0100, -0.0140, 0.0215)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    waste_tray = object_model.get_part("waste_tray")
    crank_knob = object_model.get_part("crank_knob")

    tray_slide = object_model.get_articulation("body_to_waste_tray")
    crank_spin = object_model.get_articulation("body_to_crank")
    knob_spin = object_model.get_articulation("crank_to_crank_knob")

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_within(
            waste_tray,
            body,
            axes="xz",
            margin=0.004,
            name="closed tray stays centered in the lower opening",
        )
        ctx.expect_overlap(
            waste_tray,
            body,
            axes="y",
            min_overlap=0.035,
            name="closed tray remains nested inside the shell",
        )

    tray_rest = ctx.part_world_position(waste_tray)
    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_within(
            waste_tray,
            body,
            axes="xz",
            margin=0.004,
            name="extended tray remains guided by the body opening",
        )
        ctx.expect_overlap(
            waste_tray,
            body,
            axes="y",
            min_overlap=0.010,
            name="extended tray keeps retained insertion",
        )
        tray_extended = ctx.part_world_position(waste_tray)

    ctx.check(
        "waste tray slides out from the front",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] < tray_rest[1] - 0.020,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    knob_rest = ctx.part_world_position(crank_knob)
    with ctx.pose({crank_spin: math.pi / 2.0}):
        knob_turned = ctx.part_world_position(crank_knob)

    ctx.check(
        "side crank carries the knob through a circular sweep",
        knob_rest is not None
        and knob_turned is not None
        and (
            abs(knob_turned[1] - knob_rest[1]) > 0.010
            or abs(knob_turned[2] - knob_rest[2]) > 0.010
        ),
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    ctx.check(
        "crank knob has its own continuous axle",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS and knob_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
