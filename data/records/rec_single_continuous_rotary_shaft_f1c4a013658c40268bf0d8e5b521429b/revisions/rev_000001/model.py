from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.76
BASE_WIDTH = 0.22
BASE_THICKNESS = 0.020
SHAFT_Z = 0.125

BEARING_X = (-0.18, 0.18)
BEARING_LENGTH = 0.090
BEARING_WIDTH = 0.130
BEARING_FOOT_HEIGHT = 0.058
BEARING_BORE_Z = SHAFT_Z - BASE_THICKNESS
BEARING_CAP_RADIUS = 0.062
BEARING_BORE_RADIUS = 0.018

SHAFT_RADIUS = 0.018
SHAFT_LENGTH = 0.84


def _bearing_block_cad() -> cq.Workplane:
    """Single open pillow-block support with a clear shaft channel along local X."""
    foot = (
        cq.Workplane("XY")
        .box(BEARING_LENGTH, BEARING_WIDTH, BEARING_FOOT_HEIGHT)
        .translate((0.0, 0.0, BEARING_FOOT_HEIGHT / 2.0))
    )

    saddle_top = BEARING_BORE_Z - SHAFT_RADIUS
    saddle = (
        cq.Workplane("XY")
        .box(BEARING_LENGTH, 0.040, saddle_top - BEARING_FOOT_HEIGHT)
        .translate((0.0, 0.0, (BEARING_FOOT_HEIGHT + saddle_top) / 2.0))
    )

    cheek_bottom = BEARING_FOOT_HEIGHT
    cheek_top = BEARING_BORE_Z + SHAFT_RADIUS + 0.010
    cheek_height = cheek_top - cheek_bottom
    block = foot.union(saddle)
    for y in (-0.053, 0.053):
        cheek = (
            cq.Workplane("XY")
            .box(BEARING_LENGTH, 0.024, cheek_height)
            .translate((0.0, y, (cheek_bottom + cheek_top) / 2.0))
        )
        block = block.union(cheek)

    bridge = (
        cq.Workplane("XY")
        .box(BEARING_LENGTH, BEARING_WIDTH, 0.018)
        .translate((0.0, 0.0, cheek_top + 0.009))
    )
    block = block.union(bridge)

    # Four low bolt-head bosses tie the pillow block visually to its base feet.
    for x in (-0.030, 0.030):
        for y in (-0.047, 0.047):
            bolt = (
                cq.Workplane("XY")
                .circle(0.0075)
                .extrude(0.008)
                .translate((x, y, BEARING_FOOT_HEIGHT - 0.001))
            )
            block = block.union(bolt)

    return block


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supported_rotary_shaft_module")

    painted_base = Material("painted_base", rgba=(0.10, 0.12, 0.14, 1.0))
    cast_iron = Material("cast_iron", rgba=(0.45, 0.47, 0.47, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=painted_base,
        name="base_plate",
    )

    bearing_mesh = mesh_from_cadquery(
        _bearing_block_cad(),
        "pillow_bearing_block",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    base.visual(
        bearing_mesh,
        origin=Origin(xyz=(BEARING_X[0], 0.0, BASE_THICKNESS)),
        material=cast_iron,
        name="bearing_block_0",
    )
    base.visual(
        bearing_mesh,
        origin=Origin(xyz=(BEARING_X[1], 0.0, BASE_THICKNESS)),
        material=cast_iron,
        name="bearing_block_1",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft_core",
    )
    shaft.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=(0.392, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hub_neck",
    )
    shaft.visual(
        Cylinder(radius=0.044, length=0.042),
        origin=Origin(xyz=(0.435, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="pulley_groove",
    )
    for idx, x in enumerate((0.410, 0.460)):
        shaft.visual(
            Cylinder(radius=0.052, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"pulley_flange_{idx}",
        )
    shaft.visual(
        Box((0.030, 0.012, 0.008)),
        origin=Origin(xyz=(0.435, 0.0, 0.044)),
        material=dark_steel,
        name="set_screw",
    )

    model.articulation(
        "shaft_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("shaft_rotation")

    ctx.check(
        "single continuous shaft joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_overlap(
        shaft,
        base,
        axes="x",
        min_overlap=BEARING_LENGTH * 0.80,
        elem_a="shaft_core",
        elem_b="bearing_block_0",
        name="shaft runs through first bearing span",
    )
    ctx.expect_overlap(
        shaft,
        base,
        axes="x",
        min_overlap=BEARING_LENGTH * 0.80,
        elem_a="shaft_core",
        elem_b="bearing_block_1",
        name="shaft runs through second bearing span",
    )

    def _element_center(elem_name: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(shaft, elem=elem_name)
        if bounds is None:
            return None
        lo, hi = bounds
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    rest_marker = _element_center("set_screw")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_marker = _element_center("set_screw")

    ctx.check(
        "visible marker rotates with shaft",
        rest_marker is not None
        and turned_marker is not None
        and rest_marker[2] > SHAFT_Z + 0.038
        and turned_marker[1] < -0.036
        and abs(turned_marker[2] - SHAFT_Z) < 0.018,
        details=f"rest={rest_marker}, turned={turned_marker}",
    )

    return ctx.report()


object_model = build_object_model()
