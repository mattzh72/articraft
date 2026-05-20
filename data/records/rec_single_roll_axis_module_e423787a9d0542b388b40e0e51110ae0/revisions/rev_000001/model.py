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


SHAFT_AXIS_X = 0.16
SHAFT_AXIS_Z = 0.245
SHAFT_RADIUS = 0.026


def _build_body_housing_mesh():
    """One connected grounded body with an integrated bored spindle support."""

    body = cq.Workplane("XY").box(0.42, 0.28, 0.14).translate((0.0, 0.0, 0.07))
    plinth = cq.Workplane("XY").box(0.20, 0.16, 0.06).translate((0.08, 0.0, 0.17))
    bearing_block = (
        cq.Workplane("XY").box(0.11, 0.17, 0.145).translate((SHAFT_AXIS_X, 0.0, SHAFT_AXIS_Z))
    )
    rear_rib = cq.Workplane("XY").box(0.16, 0.045, 0.11).translate((0.055, 0.0, 0.225))
    foot_land = cq.Workplane("XY").box(0.46, 0.31, 0.024).translate((0.0, 0.0, 0.012))

    body = body.union(foot_land).union(plinth).union(rear_rib).union(bearing_block)

    # Through-bore for the rotating shaft.  The hole is intentionally larger
    # than the shaft radius so the spindle is retained by the bearing support
    # without visual/collision interpenetration.
    bore = (
        cq.Workplane("XY")
        .cylinder(0.34, 0.036)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((SHAFT_AXIS_X, 0.0, SHAFT_AXIS_Z))
    )
    body = body.cut(bore)

    # Mild chamfering keeps the compact production module from reading as a
    # placeholder cube while preserving the machined block silhouette.
    try:
        body = body.edges("|Z").chamfer(0.006)
    except Exception:
        pass

    return body


def _build_bearing_lip_mesh():
    outer = (
        cq.Workplane("XY")
        .cylinder(0.014, 0.050)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    bore = (
        cq.Workplane("XY")
        .cylinder(0.030, 0.037)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_production_roll_module")

    powder_coat = model.material("blue_powder_coat", rgba=(0.12, 0.25, 0.36, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.11, 0.12, 0.13, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.03, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.95, 0.70, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_housing_mesh(), "body_housing"),
        material=powder_coat,
        name="body_housing",
    )

    # Dark front/rear bearing lips make the support path around the shaft clear.
    for x, name in ((SHAFT_AXIS_X - 0.0595, "rear_bearing_lip"), (SHAFT_AXIS_X + 0.0595, "front_bearing_lip")):
        body.visual(
            mesh_from_cadquery(_build_bearing_lip_mesh(), name),
            origin=Origin(xyz=(x, 0.0, SHAFT_AXIS_Z)),
            material=dark_steel,
            name=name,
        )

    body.visual(
        Box((0.090, 0.040, 0.018)),
        origin=Origin(xyz=(SHAFT_AXIS_X + 0.018, 0.0, SHAFT_AXIS_Z - SHAFT_RADIUS - 0.009)),
        material=dark_steel,
        name="saddle_pad",
    )

    # Small feet are partially embedded in the grounded body plate.
    for i, (x, y) in enumerate(((-0.17, -0.11), (-0.17, 0.11), (0.17, -0.11), (0.17, 0.11))):
        body.visual(
            Box((0.055, 0.040, 0.018)),
            origin=Origin(xyz=(x, y, 0.009)),
            material=black_rubber,
            name=f"foot_{i}",
        )

    output_stage = model.part("output_stage")
    output_stage.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.330),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="spindle",
    )
    output_stage.visual(
        Cylinder(radius=0.045, length=0.075),
        origin=Origin(xyz=(0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    output_stage.visual(
        Cylinder(radius=0.105, length=0.038),
        origin=Origin(xyz=(0.310, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="output_plate",
    )
    output_stage.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.334, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_cap",
    )

    for i, angle in enumerate((math.radians(30.0), math.radians(150.0), math.radians(270.0))):
        y = 0.070 * math.cos(angle)
        z = 0.070 * math.sin(angle)
        output_stage.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.333, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"bolt_{i}",
        )

    output_stage.visual(
        Box((0.011, 0.032, 0.012)),
        origin=Origin(xyz=(0.333, 0.030, 0.071)),
        material=warning_yellow,
        name="index_mark",
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=output_stage,
        origin=Origin(xyz=(SHAFT_AXIS_X, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    output_stage = object_model.get_part("output_stage")
    shaft_spin = object_model.get_articulation("shaft_spin")

    ctx.check(
        "single spinning output joint",
        len(object_model.articulations) == 1,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_overlap(
        output_stage,
        body,
        axes="x",
        elem_a="spindle",
        elem_b="body_housing",
        min_overlap=0.075,
        name="spindle passes through the supported bearing block",
    )
    ctx.expect_gap(
        output_stage,
        body,
        axis="x",
        positive_elem="output_plate",
        negative_elem="body_housing",
        min_gap=0.010,
        name="output plate clears the fixed housing face",
    )
    ctx.expect_contact(
        body,
        output_stage,
        elem_a="saddle_pad",
        elem_b="spindle",
        contact_tol=0.001,
        name="body saddle visibly carries the rotating spindle",
    )

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "grounded box body sits on the floor plane",
        body_aabb is not None and abs(body_aabb[0][2]) <= 0.002,
        details=f"body_aabb={body_aabb}",
    )

    def _elem_center(elem_name: str):
        aabb = ctx.part_element_world_aabb(output_stage, elem=elem_name)
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_mark = _elem_center("index_mark")
    rest_origin = ctx.part_world_position(output_stage)
    with ctx.pose({shaft_spin: math.pi / 2.0}):
        spun_mark = _elem_center("index_mark")
        spun_origin = ctx.part_world_position(output_stage)

    ctx.check(
        "output stage spins about the shaft axis without translating",
        rest_origin is not None
        and spun_origin is not None
        and max(abs(rest_origin[i] - spun_origin[i]) for i in range(3)) < 1e-6
        and rest_mark is not None
        and spun_mark is not None
        and abs(rest_mark[0] - spun_mark[0]) < 0.003
        and abs(rest_mark[1] - spun_mark[1]) > 0.020
        and abs(rest_mark[2] - spun_mark[2]) > 0.020,
        details=f"rest_origin={rest_origin}, spun_origin={spun_origin}, rest_mark={rest_mark}, spun_mark={spun_mark}",
    )

    return ctx.report()


object_model = build_object_model()
