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


AXIS_Z = 0.190


def _x_axis_ring(outer_radius: float, inner_radius: float, length: float, x: float):
    """CadQuery tube/ring whose centerline is the spindle X axis."""
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length / 2.0, both=True)
    cutter = cq.Workplane("YZ").circle(inner_radius).extrude(length, both=True)
    return outer.cut(cutter).translate((x, 0.0, AXIS_Z))


def _make_body_shell():
    """Stationary compact spindle housing with a through-bore for the shaft."""
    base = cq.Workplane("XY").box(0.500, 0.280, 0.050).translate((-0.020, 0.0, 0.025))
    housing = cq.Workplane("XY").box(0.380, 0.220, 0.222).translate((-0.055, 0.0, 0.160))
    front_flange = _x_axis_ring(0.082, 0.044, 0.046, 0.153)
    rear_cover = _x_axis_ring(0.060, 0.044, 0.028, -0.254)

    body = base.union(housing).union(front_flange).union(rear_cover)
    for i in range(4):
        a = math.pi / 4.0 + i * math.pi / 2.0
        screw = (
            cq.Workplane("YZ")
            .circle(0.0065)
            .extrude(0.009 / 2.0, both=True)
            .translate((0.1795, 0.062 * math.cos(a), AXIS_Z + 0.062 * math.sin(a)))
        )
        body = body.union(screw)

    # Clearanced bore through both bearing supports; the rotating shaft is a
    # separate child link and must not be hidden by body/shaft overlap.
    bore = (
        cq.Workplane("YZ")
        .circle(0.044)
        .extrude(0.780 / 2.0, both=True)
        .translate((0.0, 0.0, AXIS_Z))
    )
    return body.cut(bore)


def _make_spindle():
    """One rigid rotating spindle: shaft, hub, output plate, pilot and bolts."""
    shaft = cq.Workplane("YZ").circle(0.028).extrude(0.540 / 2.0, both=True).translate((-0.005, 0.0, 0.0))
    hub = cq.Workplane("YZ").circle(0.058).extrude(0.058 / 2.0, both=True).translate((0.215, 0.0, 0.0))
    plate = cq.Workplane("YZ").circle(0.095).extrude(0.026 / 2.0, both=True).translate((0.254, 0.0, 0.0))
    pilot = cq.Workplane("YZ").circle(0.040).extrude(0.026 / 2.0, both=True).translate((0.279, 0.0, 0.0))

    spindle = shaft.union(hub).union(plate).union(pilot)
    bolt_radius = 0.006
    bolt_circle = 0.067
    for i in range(6):
        a = i * math.tau / 6.0
        y = bolt_circle * math.cos(a)
        z = bolt_circle * math.sin(a)
        bolt = (
            cq.Workplane("YZ")
            .circle(bolt_radius)
            .extrude(0.009 / 2.0, both=True)
            .translate((0.270, y, z))
        )
        spindle = spindle.union(bolt)
    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_spindle_unit")

    cast_gray = model.material("cast_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    dark_seal = model.material("dark_seal", rgba=(0.035, 0.036, 0.038, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.73, 0.70, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "body_shell", tolerance=0.0008),
        material=cast_gray,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_x_axis_ring(0.052, 0.032, 0.006, 0.179), "front_bearing_seal"),
        material=dark_seal,
        name="front_bearing_seal",
    )
    body.visual(
        mesh_from_cadquery(_x_axis_ring(0.049, 0.032, 0.006, -0.269), "rear_bearing_seal"),
        material=dark_seal,
        name="rear_bearing_seal",
    )
    shoe_height = 0.0172
    shoe_z = AXIS_Z - 0.028 + 0.0002 - shoe_height / 2.0
    for name, x in (("front_bearing_shoe", 0.179), ("rear_bearing_shoe", -0.269)):
        body.visual(
            Box((0.004, 0.004, shoe_height)),
            origin=Origin(xyz=(x, 0.0, shoe_z)),
            material=dark_seal,
            name=name,
        )
    body.inertial = Inertial.from_geometry(
        Box((0.540, 0.280, 0.272)),
        mass=16.0,
        origin=Origin(xyz=(-0.020, 0.0, 0.136)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_make_spindle(), "spindle"),
        material=machined_steel,
        name="spindle_assembly",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.560),
        mass=2.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "spindle_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=90.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spindle = object_model.get_part("spindle")
    rotation = object_model.get_articulation("spindle_rotation")

    for shoe in ("front_bearing_shoe", "rear_bearing_shoe"):
        ctx.allow_overlap(
            body,
            spindle,
            elem_a=shoe,
            elem_b="spindle_assembly",
            reason="A tiny hidden bearing-shoe preload keeps the rotating shaft visibly supported without broad body penetration.",
        )
        ctx.expect_contact(
            body,
            spindle,
            elem_a=shoe,
            elem_b="spindle_assembly",
            contact_tol=0.001,
            name=f"{shoe} supports the spindle",
        )

    ctx.expect_overlap(
        spindle,
        body,
        axes="x",
        min_overlap=0.32,
        name="shaft runs through both bearing supports",
    )

    body_aabb = ctx.part_world_aabb(body)
    spindle_aabb = ctx.part_world_aabb(spindle)
    ctx.check(
        "grounded box body sits on the floor plane",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.002,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "output plate protrudes from the supported body",
        body_aabb is not None
        and spindle_aabb is not None
        and spindle_aabb[1][0] > body_aabb[1][0] + 0.055,
        details=f"body_aabb={body_aabb}, spindle_aabb={spindle_aabb}",
    )

    rest_position = ctx.part_world_position(spindle)
    with ctx.pose({rotation: 1.25}):
        rotated_position = ctx.part_world_position(spindle)
    ctx.check(
        "continuous joint spins about a fixed centerline",
        rest_position is not None
        and rotated_position is not None
        and all(abs(a - b) < 1.0e-6 for a, b in zip(rest_position, rotated_position)),
        details=f"rest={rest_position}, rotated={rotated_position}",
    )

    return ctx.report()


object_model = build_object_model()
