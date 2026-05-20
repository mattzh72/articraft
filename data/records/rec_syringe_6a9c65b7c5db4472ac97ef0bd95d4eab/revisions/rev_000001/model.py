from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder_x(outer_radius: float, inner_radius: float, length: float):
    """CadQuery tube centered on the world X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-0.5 * length, 0.0, 0.0))
    )


def _rounded_box(size: tuple[float, float, float], radius: float):
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _build_barrel_shell():
    barrel_length = 0.110
    outer_radius = 0.014
    inner_radius = 0.012

    tube = _annular_cylinder_x(outer_radius, inner_radius, barrel_length)
    rear_rim = _annular_cylinder_x(0.0165, inner_radius, 0.007).translate(
        (-0.5 * barrel_length, 0.0, 0.0)
    )
    front_rim = _annular_cylinder_x(0.0160, inner_radius, 0.007).translate(
        (0.5 * barrel_length, 0.0, 0.0)
    )
    rear_collar = _annular_cylinder_x(0.018, inner_radius, 0.010).translate(
        (-0.5 * barrel_length - 0.002, 0.0, 0.0)
    )

    flange = (
        _rounded_box((0.008, 0.070, 0.012), 0.002)
        .translate((-0.5 * barrel_length - 0.004, 0.0, 0.0))
        .cut(
            cq.Workplane("YZ")
            .circle(0.0128)
            .extrude(0.014)
            .translate((-0.5 * barrel_length - 0.011, 0.0, 0.0))
        )
    )

    return tube.union(rear_rim).union(front_rim).union(rear_collar).union(flange)


def _build_nozzle_tip():
    base_adapter = (
        cq.Workplane("YZ")
        .circle(0.0135)
        .extrude(0.006)
        .translate((0.053, 0.0, 0.0))
    )
    hub = (
        cq.Workplane("YZ")
        .circle(0.0085)
        .extrude(0.011)
        .translate((0.055, 0.0, 0.0))
    )
    short_luer = (
        cq.Workplane("YZ")
        .circle(0.0050)
        .extrude(0.010)
        .translate((0.065, 0.0, 0.0))
    )
    narrow_tip = (
        cq.Workplane("YZ")
        .circle(0.0024)
        .extrude(0.020)
        .translate((0.074, 0.0, 0.0))
    )
    return base_adapter.union(hub).union(short_luer).union(narrow_tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.72, 0.90, 1.00, 0.34))
    pale_tip = model.material("pale_tip", rgba=(0.86, 0.94, 0.98, 0.72))
    black_ink = model.material("black_ink", rgba=(0.03, 0.04, 0.05, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.025, 1.0))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.90, 0.92, 0.94, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel_shell(), "barrel_shell", tolerance=0.00035),
        material=clear_plastic,
        name="barrel_shell",
    )
    barrel.visual(
        mesh_from_cadquery(_build_nozzle_tip(), "nozzle_tip", tolerance=0.00025),
        material=pale_tip,
        name="nozzle_tip",
    )

    for i, x in enumerate((-0.037, -0.025, -0.013, -0.001, 0.011, 0.023, 0.035)):
        width = 0.0012 if i % 2 == 0 else 0.0008
        barrel.visual(
            mesh_from_cadquery(
                _annular_cylinder_x(0.0148, 0.0134, width).translate((x, 0.0, 0.0)),
                f"graduation_{i}",
                tolerance=0.00025,
            ),
            material=black_ink,
            name=f"graduation_{i}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.01215, length=0.012),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0022, length=0.106),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="center_rod",
    )
    plunger.visual(
        Box((0.106, 0.0070, 0.0014)),
        origin=Origin(xyz=(-0.024, 0.0, 0.0)),
        material=plunger_plastic,
        name="flat_web",
    )
    plunger.visual(
        Box((0.106, 0.0014, 0.0070)),
        origin=Origin(xyz=(-0.024, 0.0, 0.0)),
        material=plunger_plastic,
        name="upright_web",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.055),
        motion_properties=MotionProperties(damping=0.2, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="barrel_shell",
        elem_b="plunger_head",
        reason=(
            "The soft rubber plunger head is intentionally modeled with a tiny "
            "radial compression against the inside of the transparent barrel."
        ),
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.0005,
        name="plunger head stays inside barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="plunger head is retained in the barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.055}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0005,
            name="retracted head remains centered in bore",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="retracted head remains inserted",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive slide retracts plunger",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] < rest_pos[0] - 0.050,
        details=f"rest={rest_pos}, retracted={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
