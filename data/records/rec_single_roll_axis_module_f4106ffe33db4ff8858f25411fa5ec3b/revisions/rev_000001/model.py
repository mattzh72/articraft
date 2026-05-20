from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


AXIS_Z = 0.32
FRONT_X = -0.24
REAR_X = 0.24


def _x_tube(outer_radius: float, inner_radius: float, length: float, *, x: float = 0.0, z: float = 0.0):
    """Hollow cylinder whose axis is the world/local X direction."""
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length / 2.0, both=True)
    cutter = cq.Workplane("YZ").circle(inner_radius).extrude(length * 0.7, both=True)
    return outer.cut(cutter).translate((x, 0.0, z))


def _x_cylinder(radius: float, length: float, *, x: float, y: float, z: float):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, y, z))
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _retainer_cap(x_center: float, *, bolt_sign: float):
    cap = _x_tube(0.156, 0.095, 0.012, x=x_center, z=AXIS_Z)
    bolt_radius = 0.007
    bolt_length = 0.006
    bolt_circle = 0.130
    head_x = x_center + bolt_sign * (0.006 + bolt_length / 2.0)
    shapes = [cap]
    for i in range(6):
        angle = i * math.tau / 6.0 + math.radians(30.0)
        y = bolt_circle * math.cos(angle)
        z = bolt_circle * math.sin(angle)
        shapes.append(
            (
                cq.Workplane("YZ")
                .center(y, z)
                .circle(bolt_radius)
                .extrude(bolt_length / 2.0, both=True)
                .translate((head_x, 0.0, AXIS_Z))
            )
        )
    return _union_all(shapes)


def _base_rails():
    shapes = [
        _box((0.74, 0.38, 0.035), (0.0, 0.0, 0.0175)),
        _box((0.70, 0.038, 0.055), (0.0, 0.166, 0.062)),
        _box((0.70, 0.038, 0.055), (0.0, -0.166, 0.062)),
        _box((0.085, 0.34, 0.045), (FRONT_X, 0.0, 0.072)),
        _box((0.085, 0.34, 0.045), (REAR_X, 0.0, 0.072)),
    ]
    return _union_all(shapes)


def _bearing_ribs(x_center: float):
    # Low, broad yoke ribs tie each hoop to the base without entering the carrier
    # sweep envelope between the two bearing stations.
    shapes = [
        _box((0.075, 0.090, 0.120), (x_center, 0.0, 0.105)),
        _box((0.055, 0.034, 0.168), (x_center, 0.122, 0.122)),
        _box((0.055, 0.034, 0.168), (x_center, -0.122, 0.122)),
        _box((0.058, 0.020, 0.112), (x_center, 0.075, 0.152)),
        _box((0.058, 0.020, 0.112), (x_center, -0.075, 0.152)),
    ]
    return _union_all(shapes)


def _carrier_shell():
    shapes = [
        _x_tube(0.078, 0.052, 0.590),
        _x_tube(0.086, 0.052, 0.320),
        _x_tube(0.088, 0.052, 0.018, x=-0.188),
        _x_tube(0.088, 0.052, 0.018, x=0.188),
    ]
    return _union_all(shapes)


def _bearing_roller(x_center: float, angle: float):
    pitch_radius = 0.084
    return _x_cylinder(
        0.0064,
        0.045,
        x=x_center,
        y=pitch_radius * math.cos(angle),
        z=AXIS_Z + pitch_radius * math.sin(angle),
    )


def _carrier_bracket():
    base = _box((0.125, 0.060, 0.018), (0.0, 0.0, 0.091))
    upright = _box((0.082, 0.020, 0.078), (0.0, 0.0, 0.129))
    rib_a = _box((0.070, 0.010, 0.052), (0.0, 0.025, 0.112))
    rib_b = _box((0.070, 0.010, 0.052), (0.0, -0.025, 0.112))
    bracket = _union_all([base, upright, rib_a, rib_b])
    hole = (
        cq.Workplane("XZ")
        .center(0.0, 0.135)
        .circle(0.014)
        .extrude(0.080, both=True)
    )
    return bracket.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tube_rotator_module")

    powdercoat = model.material("dark_powdercoat", rgba=(0.08, 0.09, 0.10, 1.0))
    machined = model.material("machined_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    bronze = model.material("bronze_bearing", rgba=(0.78, 0.50, 0.18, 1.0))
    cap_black = model.material("blackened_cap", rgba=(0.015, 0.016, 0.018, 1.0))
    carrier_metal = model.material("brushed_aluminum", rgba=(0.72, 0.76, 0.78, 1.0))
    roller_steel = model.material("polished_rollers", rgba=(0.82, 0.84, 0.86, 1.0))

    support = model.part("fixed_support")
    support.visual(
        mesh_from_cadquery(_base_rails(), "base_rails", tolerance=0.001),
        material=powdercoat,
        name="base_rails",
    )
    support.visual(
        mesh_from_cadquery(_bearing_ribs(FRONT_X), "front_ribs", tolerance=0.001),
        material=powdercoat,
        name="front_ribs",
    )
    support.visual(
        mesh_from_cadquery(_bearing_ribs(REAR_X), "rear_ribs", tolerance=0.001),
        material=powdercoat,
        name="rear_ribs",
    )
    support.visual(
        mesh_from_cadquery(_x_tube(0.180, 0.114, 0.065, x=FRONT_X, z=AXIS_Z), "front_bearing", tolerance=0.001),
        material=machined,
        name="front_bearing",
    )
    support.visual(
        mesh_from_cadquery(_x_tube(0.180, 0.114, 0.065, x=REAR_X, z=AXIS_Z), "rear_bearing", tolerance=0.001),
        material=machined,
        name="rear_bearing",
    )
    support.visual(
        mesh_from_cadquery(_x_tube(0.115, 0.090, 0.067, x=FRONT_X, z=AXIS_Z), "front_liner", tolerance=0.001),
        material=bronze,
        name="front_liner",
    )
    support.visual(
        mesh_from_cadquery(_x_tube(0.115, 0.090, 0.067, x=REAR_X, z=AXIS_Z), "rear_liner", tolerance=0.001),
        material=bronze,
        name="rear_liner",
    )
    support.visual(
        mesh_from_cadquery(_retainer_cap(FRONT_X - 0.0385, bolt_sign=-1.0), "front_cap", tolerance=0.001),
        material=cap_black,
        name="front_cap",
    )
    support.visual(
        mesh_from_cadquery(_retainer_cap(REAR_X + 0.0385, bolt_sign=1.0), "rear_cap", tolerance=0.001),
        material=cap_black,
        name="rear_cap",
    )
    for station_name, x_center in (("front", FRONT_X), ("rear", REAR_X)):
        for idx, angle in enumerate((math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0, 0.0)):
            roller_name = f"{station_name}_roller_{idx}"
            support.visual(
                mesh_from_cadquery(_bearing_roller(x_center, angle), roller_name, tolerance=0.0005),
                material=roller_steel,
                name=roller_name,
            )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(_carrier_shell(), "carrier_shell", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carrier_metal,
        name="carrier_shell",
    )
    carrier.visual(
        mesh_from_cadquery(_carrier_bracket(), "bracket_body", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carrier_metal,
        name="bracket_body",
    )

    model.articulation(
        "support_to_carrier",
        ArticulationType.REVOLUTE,
        parent=support,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("fixed_support")
    carrier = object_model.get_part("carrier")
    joint = object_model.get_articulation("support_to_carrier")

    ctx.check(
        "carrier rolls about longitudinal x axis",
        tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0)
        and joint.motion_limits.lower <= -math.pi
        and joint.motion_limits.upper >= math.pi,
        details=f"axis={joint.axis}, limits={joint.motion_limits}",
    )

    ctx.expect_overlap(
        carrier,
        support,
        axes="x",
        elem_a="carrier_shell",
        elem_b="front_liner",
        min_overlap=0.055,
        name="carrier journal passes through front bearing liner",
    )
    ctx.expect_overlap(
        carrier,
        support,
        axes="x",
        elem_a="carrier_shell",
        elem_b="rear_liner",
        min_overlap=0.055,
        name="carrier journal passes through rear bearing liner",
    )
    ctx.expect_within(
        carrier,
        support,
        axes="yz",
        inner_elem="carrier_shell",
        outer_elem="front_bearing",
        margin=0.0,
        name="front bearing hoop is coaxial around carrier",
    )
    ctx.expect_within(
        carrier,
        support,
        axes="yz",
        inner_elem="carrier_shell",
        outer_elem="rear_bearing",
        margin=0.0,
        name="rear bearing hoop is coaxial around carrier",
    )

    ctx.expect_gap(
        carrier,
        support,
        axis="x",
        positive_elem="bracket_body",
        negative_elem="front_bearing",
        min_gap=0.080,
        name="bracket stays axially clear of front bearing",
    )
    ctx.expect_gap(
        support,
        carrier,
        axis="x",
        positive_elem="rear_bearing",
        negative_elem="bracket_body",
        min_gap=0.080,
        name="bracket stays axially clear of rear bearing",
    )

    for roller_name in (
        "front_roller_0",
        "front_roller_1",
        "front_roller_2",
        "front_roller_3",
        "rear_roller_0",
        "rear_roller_1",
        "rear_roller_2",
        "rear_roller_3",
    ):
        ctx.allow_overlap(
            carrier,
            support,
            elem_a="carrier_shell",
            elem_b=roller_name,
            reason=(
                "Bearing rollers are intentionally modeled with a tiny preload "
                "against the rotating journal so the carrier is physically grounded."
            ),
        )
        ctx.expect_contact(
            carrier,
            support,
            elem_a="carrier_shell",
            elem_b=roller_name,
            contact_tol=0.001,
            name=f"{roller_name} seats against carrier journal",
        )

    for q, label in (
        (0.0, "upright"),
        (math.pi, "inverted"),
        (math.pi / 2.0, "side_roll"),
        (-math.pi / 2.0, "opposite_side_roll"),
    ):
        with ctx.pose({joint: q}):
            ctx.expect_gap(
                carrier,
                support,
                axis="z",
                positive_elem="bracket_body",
                negative_elem="base_rails",
                min_gap=0.035,
                name=f"bracket clears base rails at {label}",
            )

    return ctx.report()


object_model = build_object_model()
