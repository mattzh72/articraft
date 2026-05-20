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


def _cylinder_x(radius: float, length: float) -> Cylinder:
    """Cylinder geometry authored with its local axis later rotated onto +X."""
    return Cylinder(radius=radius, length=length)


def _build_housing_shell():
    """Compact pillow-block housing with a true through-bore along X."""
    foot = cq.Workplane("XY").box(0.200, 0.155, 0.020).translate((0.0, 0.0, 0.010))
    pedestal = cq.Workplane("XY").box(0.112, 0.096, 0.042).translate((0.0, 0.0, 0.041))
    block = cq.Workplane("XY").box(0.118, 0.122, 0.108).translate((0.0, 0.0, 0.102))
    round_boss = (
        cq.Workplane("YZ")
        .center(0.0, 0.104)
        .circle(0.072)
        .extrude(0.063, both=True)
    )

    body = foot.union(pedestal).union(block).union(round_boss)

    # Clearance bore for the moving spindle; centered on the revolute axis.
    bore = (
        cq.Workplane("YZ")
        .center(0.0, 0.104)
        .circle(0.032)
        .extrude(0.130, both=True)
    )
    body = body.cut(bore)

    # Shallow side windows leave the part reading as a machined support rather
    # than a plain block, while preserving bridges around the bearing boss.
    for y in (-0.062, 0.062):
        window = (
            cq.Workplane("XZ")
            .center(0.0, 0.072)
            .rect(0.074, 0.044)
            .extrude(0.040, both=True)
            .translate((0.0, y, 0.0))
        )
        body = body.cut(window)

    return body


def _build_bearing_ring(outer_radius: float, inner_radius: float, thickness: float):
    """Annular bearing cover plate, axis along X."""
    return (
        cq.Workplane("YZ")
        .center(0.0, 0.104)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness * 0.5, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_axis_roll_stage")

    blue_anodized = model.material("blue_anodized", rgba=(0.08, 0.22, 0.45, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.68, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.02, 0.022, 0.024, 1.0))
    index_red = model.material("index_red", rgba=(0.82, 0.08, 0.05, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "roll_stage_housing"),
        material=blue_anodized,
        name="housing_body",
    )
    housing.visual(
        mesh_from_cadquery(_build_bearing_ring(0.057, 0.033, 0.012), "front_bearing_ring"),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=brushed_steel,
        name="front_bearing",
    )
    housing.visual(
        mesh_from_cadquery(_build_bearing_ring(0.049, 0.033, 0.010), "rear_bearing_ring"),
        origin=Origin(xyz=(-0.058, 0.0, 0.0)),
        material=brushed_steel,
        name="rear_bearing",
    )
    # Four cap screws seated in the mounting foot.
    for index, (x, y) in enumerate(
        ((-0.074, -0.055), (-0.074, 0.055), (0.074, -0.055), (0.074, 0.055))
    ):
        housing.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(x, y, 0.022), rpy=(0.0, 0.0, 0.0)),
            material=black_oxide,
            name=f"foot_screw_{index}",
        )
    housing.visual(
        Box((0.010, 0.004, 0.040)),
        origin=Origin(xyz=(0.000, -0.059, 0.132)),
        material=index_red,
        name="housing_index",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.210, 0.165, 0.165)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        _cylinder_x(radius=0.024, length=0.220),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft",
    )
    spindle.visual(
        _cylinder_x(radius=0.036, length=0.030),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_hub",
    )
    spindle.visual(
        _cylinder_x(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="faceplate",
    )
    spindle.visual(
        _cylinder_x(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_boss",
    )
    spindle.visual(
        Box((0.004, 0.009, 0.052)),
        origin=Origin(xyz=(0.109, 0.0, 0.026)),
        material=index_red,
        name="face_index",
    )
    for index, angle in enumerate([i * math.tau / 6.0 for i in range(6)]):
        y = 0.047 * math.cos(angle)
        z = 0.047 * math.sin(angle)
        spindle.visual(
            _cylinder_x(radius=0.0048, length=0.004),
            origin=Origin(xyz=(0.1065, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_oxide,
            name=f"face_screw_{index}",
        )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.160),
        mass=0.62,
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "housing_to_spindle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("housing_to_spindle")

    ctx.check(
        "single revolute roll axis",
        len(object_model.articulations) == 1
        and roll.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"joints={len(object_model.articulations)}, type={roll.articulation_type}, axis={roll.axis}",
    )
    ctx.expect_within(
        spindle,
        housing,
        axes="yz",
        inner_elem="shaft",
        outer_elem="housing_body",
        margin=0.0,
        name="shaft is centered within bearing housing",
    )
    ctx.expect_overlap(
        spindle,
        housing,
        axes="x",
        elem_a="shaft",
        elem_b="housing_body",
        min_overlap=0.100,
        name="shaft passes through bearing span",
    )
    ctx.expect_contact(
        spindle,
        housing,
        elem_a="front_hub",
        elem_b="front_bearing",
        contact_tol=0.001,
        name="spindle shoulder seats against bearing",
    )
    ctx.expect_gap(
        spindle,
        housing,
        axis="x",
        positive_elem="faceplate",
        negative_elem="front_bearing",
        min_gap=0.004,
        max_gap=0.028,
        name="faceplate clears front bearing cover",
    )

    rest_pos = ctx.part_world_position(spindle)
    with ctx.pose({roll: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(spindle)
    ctx.check(
        "roll motion keeps spindle on bearing axis",
        rest_pos is not None
        and rotated_pos is not None
        and all(abs(rest_pos[i] - rotated_pos[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
