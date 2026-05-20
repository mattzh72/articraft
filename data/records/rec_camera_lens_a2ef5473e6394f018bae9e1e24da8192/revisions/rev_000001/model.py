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


BORE_RADIUS = 0.0180
TOTAL_LENGTH = 0.0290
APERTURE_CENTER_Z = 0.0080
APERTURE_WIDTH = 0.0040
FOCUS_CENTER_Z = 0.0220
FOCUS_WIDTH = 0.0080


def _tube(outer_radius: float, inner_radius: float, length: float, *, z_center: float = 0.0):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(length)
    return outer.cut(inner).translate((0.0, 0.0, z_center - (length * 0.5)))


def _grooved_ring(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    groove_depth: float,
    groove_width: float,
    groove_positions: tuple[float, ...],
):
    ring = _tube(outer_radius, inner_radius, length)
    groove_inner_radius = max(inner_radius + 0.0002, outer_radius - groove_depth)
    for position in groove_positions:
        cutter = (
            cq.Workplane("XY")
            .circle(outer_radius + 0.0010)
            .circle(groove_inner_radius)
            .extrude(groove_width)
            .translate((0.0, 0.0, position - (groove_width * 0.5)))
        )
        ring = ring.cut(cutter)
    return ring


def _barrel_shell():
    shell = _tube(0.0265, BORE_RADIUS, 0.0020, z_center=0.0010)
    shell = shell.union(_tube(0.0236, BORE_RADIUS, 0.0100, z_center=0.0070))
    shell = shell.union(_tube(0.0246, BORE_RADIUS, 0.0060, z_center=0.0150))
    shell = shell.union(_tube(0.0204, BORE_RADIUS, 0.0080, z_center=FOCUS_CENTER_Z))
    shell = shell.union(_tube(0.0270, BORE_RADIUS, 0.0030, z_center=0.0275))
    return shell


def _focus_ring_shape():
    return _grooved_ring(
        0.0288,
        0.0210,
        FOCUS_WIDTH,
        groove_depth=0.0010,
        groove_width=0.0008,
        groove_positions=(-0.0026, -0.0009, 0.0009, 0.0026),
    )


def _aperture_ring_shape():
    return _grooved_ring(
        0.0262,
        0.0236,
        APERTURE_WIDTH,
        groove_depth=0.0006,
        groove_width=0.0006,
        groove_positions=(-0.0009, 0.0009),
    )


def _positions_match(a, b, tol: float = 1e-6) -> bool:
    return (
        a is not None
        and b is not None
        and all(abs(coord_a - coord_b) <= tol for coord_a, coord_b in zip(a, b))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pancake_lens")

    barrel_black = model.material("barrel_black", rgba=(0.11, 0.11, 0.12, 1.0))
    focus_rubber = model.material("focus_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    aperture_satin = model.material("aperture_satin", rgba=(0.17, 0.17, 0.18, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.18, 0.28, 0.32, 0.58))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_barrel_shell(), "lens_barrel_shell"),
        material=barrel_black,
        name="shell",
    )
    barrel.visual(
        Cylinder(radius=0.0182, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0228)),
        material=coated_glass,
        name="front_element",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_focus_ring_shape(), "lens_focus_ring"),
        material=focus_rubber,
        name="grip_band",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_cadquery(_aperture_ring_shape(), "lens_aperture_ring"),
        material=aperture_satin,
        name="control_band",
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, FOCUS_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    model.articulation(
        "aperture_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, APERTURE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_rotation = object_model.get_articulation("focus_rotation")
    aperture_rotation = object_model.get_articulation("aperture_rotation")

    ctx.allow_overlap(
        barrel,
        focus_ring,
        elem_a="shell",
        elem_b="grip_band",
        reason="The focus ring is intentionally represented as a close coaxial sleeve riding on the outer barrel band.",
    )

    ctx.expect_origin_distance(
        focus_ring,
        barrel,
        axes="xy",
        max_dist=1e-6,
        name="focus ring stays on the lens axis",
    )
    ctx.expect_origin_distance(
        aperture_ring,
        barrel,
        axes="xy",
        max_dist=1e-6,
        name="aperture ring stays on the lens axis",
    )
    ctx.expect_origin_gap(
        focus_ring,
        aperture_ring,
        axis="z",
        min_gap=0.012,
        max_gap=0.016,
        name="focus ring sits ahead of the aperture ring",
    )
    ctx.expect_gap(
        focus_ring,
        aperture_ring,
        axis="z",
        min_gap=0.007,
        max_gap=0.010,
        name="focus and aperture rings remain distinct bands",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="xy",
        min_overlap=0.048,
        name="focus ring remains coaxial with the barrel shell",
    )
    ctx.expect_overlap(
        aperture_ring,
        barrel,
        axes="xy",
        min_overlap=0.046,
        name="aperture ring remains coaxial with the barrel shell",
    )

    focus_rest = ctx.part_world_position(focus_ring)
    aperture_rest = ctx.part_world_position(aperture_ring)
    with ctx.pose({focus_rotation: 1.35, aperture_rotation: -0.95}):
        focus_turned = ctx.part_world_position(focus_ring)
        aperture_turned = ctx.part_world_position(aperture_ring)

    ctx.check(
        "focus ring rotates in place",
        _positions_match(focus_rest, focus_turned),
        details=f"rest={focus_rest}, turned={focus_turned}",
    )
    ctx.check(
        "aperture ring rotates in place",
        _positions_match(aperture_rest, aperture_turned),
        details=f"rest={aperture_rest}, turned={aperture_turned}",
    )

    return ctx.report()


object_model = build_object_model()
