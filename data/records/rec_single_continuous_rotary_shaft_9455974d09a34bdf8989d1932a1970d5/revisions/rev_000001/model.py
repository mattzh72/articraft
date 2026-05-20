from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_min: float):
    outer = _solid_cylinder(outer_radius, height, z_min)
    inner = _solid_cylinder(inner_radius, height + 0.004, z_min - 0.002)
    return outer.cut(inner)


def _solid_cylinder(radius: float, height: float, z_min: float):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _make_stationary_shell():
    body = cq.Workplane("XY").box(0.70, 0.55, 0.30).translate((0.0, 0.0, 0.185))
    try:
        body = body.edges("|Z").fillet(0.035)
    except Exception:
        pass

    pedestal = _solid_cylinder(0.18, 0.16, 0.335)
    pan_floor = _annular_cylinder(0.335, 0.055, 0.025, 0.495)
    outer_rim = _annular_cylinder(0.380, 0.335, 0.090, 0.495)
    bearing_collar = _annular_cylinder(0.095, 0.046, 0.090, 0.495)

    shell = body.union(pedestal).union(pan_floor).union(outer_rim).union(bearing_collar)
    shaft_clearance = _solid_cylinder(0.046, 0.80, 0.0)
    return shell.cut(shaft_clearance)


def _make_top_plate():
    plate = cq.Workplane("XY").circle(0.255).extrude(0.045)
    try:
        plate = plate.edges("|Z").fillet(0.004)
    except Exception:
        pass

    for radius in (0.095, 0.160, 0.220):
        groove = (
            cq.Workplane("XY")
            .circle(radius + 0.0025)
            .circle(radius - 0.0025)
            .extrude(0.007)
            .translate((0.0, 0.0, 0.040))
        )
        plate = plate.cut(groove)

    for x in (-0.145, 0.145):
        pin_hole = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(0.011)
            .extrude(0.070)
            .translate((0.0, 0.0, -0.010))
        )
        plate = plate.cut(pin_hole)

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pottery_wheel")

    enamel = model.material("glazed_teal_enamel", rgba=(0.10, 0.34, 0.38, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.73, 1.0))
    dark_steel = model.material("dark_drive_steel", rgba=(0.10, 0.10, 0.10, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(
            _make_stationary_shell(),
            "stationary_shell",
            tolerance=0.001,
            angular_tolerance=0.06,
        ),
        material=enamel,
        name="base_shell",
    )
    for index, (x, y) in enumerate(
        ((-0.255, -0.190), (-0.255, 0.190), (0.255, -0.190), (0.255, 0.190))
    ):
        base.visual(
            Cylinder(radius=0.045, length=0.040),
            origin=Origin(xyz=(x, y, 0.020)),
            material=rubber,
            name=f"foot_{index}",
        )

    wheelhead = model.part("wheelhead")
    wheelhead.visual(
        Cylinder(radius=0.032, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=dark_steel,
        name="shaft",
    )
    wheelhead.visual(
        Cylinder(radius=0.080, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=dark_steel,
        name="hub",
    )
    wheelhead.visual(
        mesh_from_cadquery(
            _make_top_plate(),
            "top_plate",
            tolerance=0.0007,
            angular_tolerance=0.045,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=aluminum,
        name="top_plate",
    )

    model.articulation(
        "base_to_wheelhead",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheelhead,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wheelhead = object_model.get_part("wheelhead")
    spin = object_model.get_articulation("base_to_wheelhead")

    ctx.check(
        "single continuous vertical drive joint",
        len(object_model.articulations) == 1
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"joint_count={len(object_model.articulations)}, type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        wheelhead,
        base,
        axis="z",
        positive_elem="top_plate",
        negative_elem="base_shell",
        min_gap=0.020,
        max_gap=0.050,
        name="rotating plate clears the stationary splash pan",
    )
    ctx.expect_within(
        wheelhead,
        base,
        axes="xy",
        inner_elem="shaft",
        outer_elem="base_shell",
        margin=0.0,
        name="vertical shaft is centered inside the wheel body",
    )

    with ctx.pose({spin: 1.25}):
        ctx.expect_gap(
            wheelhead,
            base,
            axis="z",
            positive_elem="top_plate",
            negative_elem="base_shell",
            min_gap=0.020,
            max_gap=0.050,
            name="plate clearance is preserved while spinning",
        )

    return ctx.report()


object_model = build_object_model()
