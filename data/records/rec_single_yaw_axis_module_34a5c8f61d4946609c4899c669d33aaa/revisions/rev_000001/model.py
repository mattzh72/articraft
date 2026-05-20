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
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _polar(radius: float, angle: float, z: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _annular_plate(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    bolt_circle: float | None = None,
    bolt_diameter: float = 0.0,
    bolt_count: int = 0,
    name: str,
):
    """CadQuery annular plate with optional through bolt holes, authored in meters."""
    body = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    if bolt_circle is not None and bolt_count:
        points = [
            (bolt_circle * math.cos(2.0 * math.pi * i / bolt_count), bolt_circle * math.sin(2.0 * math.pi * i / bolt_count))
            for i in range(bolt_count)
        ]
        body = body.faces(">Z").workplane(centerOption="CenterOfMass").pushPoints(points).hole(bolt_diameter)
    return mesh_from_cadquery(body, name, tolerance=0.0009, angular_tolerance=0.05)


def _round_plate(
    radius: float,
    height: float,
    *,
    center_hole: float | None = None,
    bolt_circle: float | None = None,
    bolt_diameter: float = 0.0,
    bolt_count: int = 0,
    name: str,
):
    body = cq.Workplane("XY").circle(radius)
    if center_hole is not None and center_hole > 0.0:
        body = body.circle(center_hole)
    body = body.extrude(height)
    if bolt_circle is not None and bolt_count:
        points = [
            (bolt_circle * math.cos(2.0 * math.pi * i / bolt_count), bolt_circle * math.sin(2.0 * math.pi * i / bolt_count))
            for i in range(bolt_count)
        ]
        body = body.faces(">Z").workplane(centerOption="CenterOfMass").pushPoints(points).hole(bolt_diameter)
    return mesh_from_cadquery(body, name, tolerance=0.0009, angular_tolerance=0.05)


def _cover_plate(width: float, depth: float, height: float, hole_dx: float, hole_dia: float, *, name: str):
    body = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-hole_dx, 0.0), (hole_dx, 0.0)])
        .hole(hole_dia)
    )
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.05)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="single_yaw_axis_module",
        meta={
            "description": "Standalone sdk_hybrid mechanical study: broad base yaw turntable, exposed bearing stack, end stops, brackets, covers, and cable pass-through stand-ins.",
        },
    )

    blasted = model.material("blasted_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark = model.material("black_oxide", rgba=(0.035, 0.038, 0.040, 1.0))
    bearing = model.material("bearing_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    cover_mat = model.material("dull_access_cover", rgba=(0.24, 0.27, 0.28, 1.0))
    witness = model.material("engraved_witness_mark", rgba=(0.98, 0.78, 0.16, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.011, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        _annular_plate(
            0.430,
            0.082,
            0.034,
            bolt_circle=0.360,
            bolt_diameter=0.024,
            bolt_count=12,
            name="base_flange_mesh",
        ),
        origin=Origin(),
        material=blasted,
        name="base_flange",
    )
    base.visual(
        _annular_plate(
            0.270,
            0.112,
            0.040,
            bolt_circle=0.222,
            bolt_diameter=0.014,
            bolt_count=16,
            name="lower_race_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=dark,
        name="lower_race",
    )
    base.visual(
        _annular_plate(0.114, 0.074, 0.074, name="cable_tube_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=rubber,
        name="cable_passage_sleeve",
    )
    base.visual(
        Cylinder(radius=0.160, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=blasted,
        name="machined_pedestal",
    )

    # Radial welded/fabricated gussets tying the race pedestal back into the broad flange.
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base.visual(
            Box((0.260, 0.022, 0.088)),
            origin=Origin(xyz=_polar(0.245, angle, 0.078), rpy=(0.0, 0.0, angle)),
            material=blasted,
            name=f"radial_gusset_{i}",
        )
        base.visual(
            Box((0.110, 0.030, 0.022)),
            origin=Origin(xyz=_polar(0.360, angle, 0.045), rpy=(0.0, 0.0, angle)),
            material=dark,
            name=f"outrigger_clamp_{i}",
        )

    # Flange hardware: socket heads and visible shanks seated into the through holes.
    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        x, y, _ = _polar(0.360, angle)
        base.visual(
            Cylinder(radius=0.023, length=0.013),
            origin=Origin(xyz=(x, y, 0.040)),
            material=dark,
            name=f"base_bolt_head_{i}",
        )
        base.visual(
            Cylinder(radius=0.008, length=0.038),
            origin=Origin(xyz=(x, y, 0.018)),
            material=dark,
            name=f"base_bolt_shank_{i}",
        )

    # Exposed slewing bearing balls captured between a lower race and rotating upper race.
    base.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.193, 0.0, 0.109)),
        material=bearing,
        name="bearing_ball_ref",
    )
    for i in range(1, 28):
        angle = 2.0 * math.pi * i / 28.0
        x, y, _ = _polar(0.193, angle)
        base.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(x, y, 0.109)),
            material=bearing,
            name=f"bearing_ball_{i}",
        )
    base.visual(
        Cylinder(radius=0.205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=dark,
        name="bearing_cage_band",
    )

    # Stationary, bolted end-stop brackets on the base perimeter.
    for i, angle in enumerate((2.25, -2.25)):
        x, y, _ = _polar(0.318, angle)
        base.visual(
            Box((0.062, 0.090, 0.086)),
            origin=Origin(xyz=(x, y, 0.077), rpy=(0.0, 0.0, angle)),
            material=dark,
            name=f"end_stop_block_{i}",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(x, y, 0.129)),
            material=bearing,
            name=f"stop_roller_{i}",
        )

    # Two fixed removable inspection covers over the top-side cable trenches.
    cover_mesh = _cover_plate(0.155, 0.080, 0.010, 0.052, 0.010, name="access_cover_mesh")
    for i, angle in enumerate((math.pi / 4.0, -math.pi / 4.0)):
        cover = model.part(f"access_cover_{i}")
        cover.visual(
            cover_mesh,
            origin=Origin(),
            material=cover_mat,
            name="cover_plate",
        )
        for sx in (-0.052, 0.052):
            cover.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(sx, 0.0, 0.012)),
                material=dark,
                name=f"captive_screw_{0 if sx < 0.0 else 1}",
            )
        model.articulation(
            f"base_to_access_cover_{i}",
            ArticulationType.FIXED,
            parent=base,
            child=cover,
            origin=Origin(xyz=_polar(0.285, angle, 0.034), rpy=(0.0, 0.0, angle)),
        )

    upper = model.part("upper_platform")
    upper.visual(
        _annular_plate(
            0.255,
            0.116,
            0.035,
            bolt_circle=0.216,
            bolt_diameter=0.013,
            bolt_count=16,
            name="upper_race_mesh",
        ),
        origin=Origin(),
        material=dark,
        name="upper_race",
    )
    upper.visual(
        _round_plate(
            0.305,
            0.032,
            center_hole=0.064,
            bolt_circle=0.248,
            bolt_diameter=0.016,
            bolt_count=12,
            name="top_plate_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=blasted,
        name="top_plate",
    )
    upper.visual(
        _annular_plate(0.115, 0.064, 0.104, name="rotor_hub_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=blasted,
        name="rotor_hub",
    )
    upper.visual(
        _annular_plate(0.070, 0.050, 0.028, name="rotating_cable_throat_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=rubber,
        name="rotating_cable_throat",
    )

    for i in range(8):
        angle = 2.0 * math.pi * i / 8.0
        x, y, _ = _polar(0.151, angle)
        upper.visual(
            Cylinder(radius=0.013, length=0.021),
            origin=Origin(xyz=(x, y, 0.0455)),
            material=blasted,
            name=f"race_standoff_{i}",
        )

    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        x, y, _ = _polar(0.248, angle)
        upper.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(x, y, 0.093)),
            material=dark,
            name=f"top_socket_bolt_{i}",
        )

    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        upper.visual(
            Box((0.220, 0.020, 0.040)),
            origin=Origin(xyz=_polar(0.168, angle, 0.108), rpy=(0.0, 0.0, angle)),
            material=blasted,
            name=f"top_radial_rib_{i}",
        )
        upper.visual(
            Box((0.062, 0.030, 0.092)),
            origin=Origin(xyz=_polar(0.272, angle, 0.134), rpy=(0.0, 0.0, angle)),
            material=blasted,
            name=f"payload_bracket_{i}",
        )
        upper.visual(
            Cylinder(radius=0.010, length=0.046),
            origin=Origin(xyz=_polar(0.272, angle, 0.161), rpy=(math.pi / 2.0, 0.0, angle)),
            material=dark,
            name=f"bracket_cross_pin_{i}",
        )

    # Rotating end-stop dog that sweeps between the two fixed stop rollers.
    upper.visual(
        Box((0.086, 0.040, 0.072)),
        origin=Origin(xyz=_polar(0.310, 0.0, 0.071), rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name="stop_dog",
    )
    upper.visual(
        Box((0.018, 0.200, 0.006)),
        origin=Origin(xyz=(0.100, 0.0, 0.091)),
        material=witness,
        name="yaw_witness_mark",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.25, lower=-2.05, upper=2.05),
        motion_properties=MotionProperties(damping=2.5, friction=0.8),
        meta={"role": "single yaw axis carried by the visible slewing bearing stack"},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_platform")
    yaw = object_model.get_articulation("yaw_axis")

    ctx.expect_contact(
        upper,
        base,
        elem_a="upper_race",
        elem_b="bearing_ball_ref",
        contact_tol=0.0015,
        name="upper race is physically supported by the exposed bearing row",
    )
    ctx.expect_within(
        upper,
        base,
        axes="xy",
        inner_elem="upper_race",
        outer_elem="lower_race",
        margin=0.025,
        name="slewing races are concentric in plan",
    )
    rest_pos = ctx.part_world_position(upper)
    with ctx.pose({yaw: 1.0}):
        turned_pos = ctx.part_world_position(upper)
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem="upper_race",
            negative_elem="bearing_ball_ref",
            max_penetration=0.0015,
            name="bearing support remains seated after yaw motion",
        )
    ctx.check(
        "upper platform rotates without translating the yaw center",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
