from __future__ import annotations

import math

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
import cadquery as cq


def _tube(z0: float, z1: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """Annular tube section on the optical Z axis."""
    height = z1 - z0
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate(
        (0.0, 0.0, -0.002)
    )
    return outer.cut(bore).translate((0.0, 0.0, z0))


def _trapezoid_prism_x(length: float, points_yz: list[tuple[float, float]]) -> cq.Workplane:
    """Extrude a Y/Z dovetail section along X and center it on the origin."""
    return (
        cq.Workplane("YZ")
        .polyline(points_yz)
        .close()
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _barrel_body() -> cq.Workplane:
    body = _tube(0.011, 0.023, 0.058, 0.046)
    body = body.union(_tube(0.020, 0.116, 0.052, 0.040))
    body = body.union(_tube(0.108, 0.128, 0.055, 0.035))

    # Raised longitudinal grip ribs on the fixed round barrel.
    for i in range(24):
        angle = 360.0 * i / 24.0
        rib = (
            cq.Workplane("XY")
            .box(0.006, 0.009, 0.040)
            .translate((0.052, 0.0, 0.066))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        body = body.union(rib)
    return body


def _camera_mount() -> cq.Workplane:
    mount = _tube(-0.024, -0.015, 0.060, 0.034)
    for i in range(3):
        angle = 120.0 * i + 18.0
        lug = (
            cq.Workplane("XY")
            .box(0.014, 0.006, 0.005)
            .translate((0.048, 0.0, -0.024))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        mount = mount.union(lug)
    return mount


def _rotation_ring() -> cq.Workplane:
    ring = _tube(-0.008, 0.004, 0.052, 0.034)

    # Two overhanging rails bridge the aperture and form the fixed dovetail guide.
    rail_len = 0.078
    upper = _trapezoid_prism_x(
        rail_len,
        [
            (0.026, 0.004),
            (0.035, 0.004),
            (0.035, 0.011),
            (0.022, 0.011),
        ],
    )
    lower = _trapezoid_prism_x(
        rail_len,
        [
            (-0.035, 0.004),
            (-0.026, 0.004),
            (-0.022, 0.011),
            (-0.035, 0.011),
        ],
    )
    ring = ring.union(upper).union(lower)

    # Small stop blocks at the ends of the guide show the usable shift travel.
    for x in (-0.046, 0.046):
        stop = cq.Workplane("XY").box(0.006, 0.036, 0.007).translate((x, 0.0, 0.0075))
        ring = ring.union(stop)
    return ring


def _optical_carriage() -> cq.Workplane:
    # Matching sliding dovetail tongue, captive under the guide lips.
    tongue = _trapezoid_prism_x(
        0.068,
        [
            (-0.026, 0.011),
            (0.026, 0.011),
            (0.022, 0.0178),
            (-0.022, 0.0178),
        ],
    )

    plate = (
        cq.Workplane("XY")
        .rect(0.058, 0.040)
        .extrude(0.008)
        .faces(">Z")
        .workplane()
        .circle(0.020)
        .cutThruAll()
        .translate((0.0, 0.0, 0.0178))
    )

    lens_cell = _tube(0.025, 0.096, 0.0265, 0.020)
    front_bezel = _tube(0.088, 0.100, 0.028, 0.018)
    rear_retainer = _tube(0.025, 0.034, 0.024, 0.018)
    return tongue.union(plate).union(lens_cell).union(front_bezel).union(rear_retainer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perspective_control_shift_lens")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    graphite = model.material("graphite", rgba=(0.09, 0.085, 0.08, 1.0))
    anodized_gray = model.material("anodized_gray", rgba=(0.42, 0.42, 0.40, 1.0))
    glass_blue = model.material("coated_glass", rgba=(0.18, 0.42, 0.72, 0.42))
    red = model.material("red_index", rgba=(0.9, 0.04, 0.02, 1.0))
    white = model.material("white_engraving", rgba=(0.9, 0.9, 0.82, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_barrel_body(), "barrel_body", tolerance=0.0008),
        material=matte_black,
        name="barrel_body",
    )
    barrel.visual(
        mesh_from_cadquery(_camera_mount(), "camera_mount", tolerance=0.0008),
        material=anodized_gray,
        name="camera_mount",
    )
    for i in range(3):
        angle = 2.0 * math.pi * i / 3.0 + math.pi / 6.0
        barrel.visual(
            Box((0.006, 0.012, 0.027)),
            origin=Origin(
                xyz=(0.057 * math.cos(angle), 0.057 * math.sin(angle), -0.002),
                rpy=(0.0, 0.0, angle),
            ),
            material=matte_black,
            name=f"bridge_pier_{i}",
        )
    barrel.visual(
        Cylinder(radius=0.0022, length=0.0015),
        origin=Origin(xyz=(0.0, -0.0570, 0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="fixed_index_dot",
    )

    rotation_ring = model.part("rotation_ring")
    rotation_ring.visual(
        mesh_from_cadquery(_rotation_ring(), "rotation_ring", tolerance=0.0007),
        material=graphite,
        name="rotating_face",
    )
    rotation_ring.visual(
        Cylinder(radius=0.0016, length=0.0012),
        origin=Origin(xyz=(0.0, -0.0520, 0.0005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="ring_index_dot",
    )

    optical_block = model.part("optical_block")
    optical_block.visual(
        mesh_from_cadquery(_optical_carriage(), "optical_carriage", tolerance=0.0007),
        material=anodized_gray,
        name="optical_carriage",
    )
    optical_block.visual(
        Cylinder(radius=0.0208, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.093), rpy=(0.0, 0.0, 0.0)),
        material=glass_blue,
        name="front_element",
    )

    model.articulation(
        "barrel_to_rotation_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=rotation_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "ring_to_optical_block",
        ArticulationType.PRISMATIC,
        parent=rotation_ring,
        child=optical_block,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.08, lower=-0.012, upper=0.012),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    rotation_ring = object_model.get_part("rotation_ring")
    optical_block = object_model.get_part("optical_block")
    rotate = object_model.get_articulation("barrel_to_rotation_ring")
    shift = object_model.get_articulation("ring_to_optical_block")

    ctx.expect_overlap(
        rotation_ring,
        optical_block,
        axes="x",
        min_overlap=0.05,
        elem_a="rotating_face",
        elem_b="optical_carriage",
        name="dovetail guide retains the sliding block at neutral shift",
    )
    ctx.expect_within(
        optical_block,
        rotation_ring,
        axes="y",
        margin=0.002,
        inner_elem="optical_carriage",
        outer_elem="rotating_face",
        name="sliding carriage stays between the dovetail rails",
    )
    ctx.expect_within(
        optical_block,
        barrel,
        axes="xy",
        margin=0.004,
        inner_elem="front_element",
        outer_elem="barrel_body",
        name="shift lens element remains inside oversized barrel bore",
    )

    neutral_pos = ctx.part_world_position(optical_block)
    with ctx.pose({shift: 0.012}):
        shifted_pos = ctx.part_world_position(optical_block)
        ctx.expect_overlap(
            rotation_ring,
            optical_block,
            axes="x",
            min_overlap=0.04,
            elem_a="rotating_face",
            elem_b="optical_carriage",
            name="dovetail still retains the block at full positive shift",
        )

    ctx.check(
        "prismatic shift moves perpendicular to optical axis",
        neutral_pos is not None
        and shifted_pos is not None
        and shifted_pos[0] > neutral_pos[0] + 0.010
        and abs(shifted_pos[2] - neutral_pos[2]) < 0.001,
        details=f"neutral={neutral_pos}, shifted={shifted_pos}",
    )

    with ctx.pose({rotate: math.pi / 2.0, shift: 0.012}):
        rotated_shift_pos = ctx.part_world_position(optical_block)

    ctx.check(
        "rotation ring redirects the shift axis",
        neutral_pos is not None
        and rotated_shift_pos is not None
        and rotated_shift_pos[1] > neutral_pos[1] + 0.010
        and abs(rotated_shift_pos[0] - neutral_pos[0]) < 0.002,
        details=f"neutral={neutral_pos}, rotated_shift={rotated_shift_pos}",
    )

    return ctx.report()


object_model = build_object_model()
