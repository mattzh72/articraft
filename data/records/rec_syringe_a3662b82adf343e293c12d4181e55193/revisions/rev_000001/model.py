from __future__ import annotations

import math

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
import cadquery as cq


def _tube_x(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """CadQuery annular tube running from x=0 to x=length."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .rotate((0, 0, 0), (0, 1, 0), 90)
    )


def _cylinder_x(length: float, radius: float) -> cq.Workplane:
    """CadQuery solid cylinder running from x=0 to x=length."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0, 0, 0), (0, 1, 0), 90)
    )


def _frustum_x(length: float, base_radius: float, tip_radius: float) -> cq.Workplane:
    """CadQuery tapered nozzle section running from x=0 to x=length."""
    return (
        cq.Workplane("XY")
        .circle(base_radius)
        .workplane(offset=length)
        .circle(tip_radius)
        .loft()
        .rotate((0, 0, 0), (0, 1, 0), 90)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = Material("clear_polycarbonate", rgba=(0.72, 0.90, 1.0, 0.36))
    milky_plastic = Material("milky_plastic", rgba=(0.93, 0.96, 0.98, 1.0))
    rubber = Material("black_rubber", rgba=(0.025, 0.023, 0.024, 1.0))
    printed_ink = Material("printed_black", rgba=(0.02, 0.02, 0.025, 1.0))

    barrel_length = 0.130
    barrel_outer_radius = 0.0120
    barrel_inner_radius = 0.0108
    barrel = model.part("barrel")

    barrel.visual(
        mesh_from_cadquery(
            _tube_x(barrel_length, barrel_outer_radius, barrel_inner_radius),
            "barrel_tube",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        material=clear_plastic,
        name="barrel_tube",
    )
    barrel.visual(
        mesh_from_cadquery(
            _tube_x(0.010, 0.0142, 0.0068).translate((-0.006, 0.0, 0.0)),
            "rear_collar",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        material=clear_plastic,
        name="rear_collar",
    )

    # Finger flanges are two side ears bonded to the rear collar, leaving the
    # center open for the plunger rod.
    for suffix, y in (("0", 0.025), ("1", -0.025)):
        barrel.visual(
            Box((0.007, 0.025, 0.010)),
            origin=Origin(xyz=(-0.002, y, 0.0)),
            material=clear_plastic,
            name=f"finger_flange_{suffix}",
        )

    # A cleaner guide layout: only two low molded ribs along the barrel, not a
    # forest of separate exposed guide members.
    for suffix, z in (("top", barrel_outer_radius + 0.00045), ("bottom", -barrel_outer_radius - 0.00045)):
        barrel.visual(
            Box((0.108, 0.0032, 0.0016)),
            origin=Origin(xyz=(0.062, 0.0, z)),
            material=clear_plastic,
            name=f"{suffix}_rail",
        )

    # Luer-style tapered nozzle and a short outlet tip at the front.
    barrel.visual(
        mesh_from_cadquery(
            _frustum_x(0.020, barrel_outer_radius, 0.0026).translate((barrel_length - 0.001, 0.0, 0.0)),
            "nozzle_cone",
            tolerance=0.00025,
            angular_tolerance=0.08,
        ),
        material=clear_plastic,
        name="nozzle_cone",
    )
    barrel.visual(
        mesh_from_cadquery(
            _cylinder_x(0.010, 0.0018).translate((barrel_length + 0.0185, 0.0, 0.0)),
            "nozzle_tip",
            tolerance=0.0002,
            angular_tolerance=0.08,
        ),
        material=clear_plastic,
        name="nozzle_tip",
    )

    # Raised black graduation ticks on the side of the transparent barrel.
    for i, x in enumerate((0.024, 0.042, 0.060, 0.078, 0.096, 0.114)):
        height = 0.010 if i % 2 == 0 else 0.0065
        barrel.visual(
            Box((0.0012, 0.0007, height)),
            origin=Origin(xyz=(x, barrel_outer_radius + 0.0001, 0.0)),
            material=printed_ink,
            name=f"graduation_{i}",
        )

    plunger = model.part("plunger")

    # Two intersecting ribs make a simple cross-section rod that stays visually
    # clean while remaining mechanically plausible.
    plunger.visual(
        Box((0.138, 0.0032, 0.0078)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=milky_plastic,
        name="rod_web_0",
    )
    plunger.visual(
        Box((0.138, 0.0078, 0.0032)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=milky_plastic,
        name="rod_web_1",
    )
    plunger.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=milky_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=milky_plastic,
        name="head_socket",
    )
    plunger.visual(
        Cylinder(radius=0.0090, length=0.010),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="plunger_head",
    )
    for suffix, x in (("front", 0.101), ("rear", 0.089)):
        plunger.visual(
            Cylinder(radius=0.0109, length=0.0025),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"{suffix}_seal",
        )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    for seal in ("front_seal", "rear_seal"):
        ctx.allow_overlap(
            barrel,
            plunger,
            elem_a="barrel_tube",
            elem_b=seal,
            reason="The soft rubber plunger seal is intentionally modeled as lightly compressed against the syringe barrel bore.",
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem=seal,
            outer_elem="barrel_tube",
            margin=0.0001,
            name=f"{seal} seated inside barrel bore",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a=seal,
            elem_b="barrel_tube",
            min_overlap=0.002,
            name=f"{seal} axially retained in barrel",
        )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_tube",
        margin=0.0005,
        name="plunger head centered in barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_tube",
        min_overlap=0.008,
        name="plunger head inserted at rest",
    )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.055}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_tube",
            margin=0.0005,
            name="pulled head remains centered",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_tube",
            min_overlap=0.008,
            name="pulled head remains inserted",
        )
        pulled_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger slides out along barrel axis",
        rest_position is not None
        and pulled_position is not None
        and pulled_position[0] < rest_position[0] - 0.045,
        details=f"rest={rest_position}, pulled={pulled_position}",
    )

    return ctx.report()


object_model = build_object_model()
