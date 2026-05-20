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


def _annular_segment(outer_radius: float, inner_radius: float, z_min: float, z_max: float):
    """A vertical annular cylinder in local meters."""
    height = z_max - z_min
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _stepped_annulus(segments: list[tuple[float, float, float, float]]):
    body = None
    for outer_radius, inner_radius, z_min, z_max in segments:
        segment = _annular_segment(outer_radius, inner_radius, z_min, z_max)
        body = segment if body is None else body.union(segment)
    return body


def _radial_hole_cutter(radius: float, hole_radius: float, z_min: float, z_max: float):
    height = z_max - z_min
    return (
        cq.Workplane("XY")
        .center(radius, 0.0)
        .circle(hole_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _add_index_holes(body, bolt_radius: float, hole_radius: float, z_min: float, z_max: float, count: int):
    for index in range(count):
        cutter = _radial_hole_cutter(bolt_radius, hole_radius, z_min, z_max).rotate(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            360.0 * index / count,
        )
        body = body.cut(cutter)
    return body


def _base_body_mesh():
    body = _stepped_annulus(
        [
            (0.360, 0.066, -0.066, -0.036),  # proud lower retainer flange
            (0.322, 0.066, -0.038, 0.026),   # wide drum body
            (0.248, 0.064, 0.024, 0.066),    # raised top bearing land
        ]
    )
    body = _add_index_holes(body, bolt_radius=0.286, hole_radius=0.014, z_min=-0.070, z_max=0.072, count=16)
    # Shallow machined relief grooves at the outside of the lower drum and top land.
    body = body.cut(_annular_segment(0.323, 0.309, -0.010, 0.004))
    body = body.cut(_annular_segment(0.249, 0.235, 0.046, 0.057))
    return body


def _collar_body_mesh():
    body = _stepped_annulus(
        [
            (0.246, 0.060, -0.073, -0.046),  # lower retainer flange
            (0.206, 0.060, -0.048, 0.047),   # raised middle collar
            (0.226, 0.060, 0.045, 0.073),    # upper shoulder flange
        ]
    )
    # Split clamp ears are fused into the collar; a through-slit remains visible.
    for y in (-0.038, 0.038):
        ear = cq.Workplane("XY").box(0.078, 0.030, 0.082).translate((0.232, y, -0.002))
        body = body.union(ear)
    split = cq.Workplane("XY").box(0.118, 0.014, 0.170).translate((0.224, 0.0, 0.0))
    body = body.cut(split)
    body = body.cut(_annular_segment(0.207, 0.193, -0.010, 0.012))
    return body


def _nose_body_mesh():
    body = _stepped_annulus(
        [
            (0.146, 0.046, -0.083, -0.053),  # lower shoulder flange
            (0.106, 0.046, -0.055, 0.022),   # precision bearing land
            (0.076, 0.046, 0.020, 0.066),    # narrow tooling nose
            (0.057, 0.046, 0.064, 0.083),    # small pilot snout
        ]
    )
    body = _add_index_holes(body, bolt_radius=0.107, hole_radius=0.007, z_min=-0.085, z_max=-0.050, count=6)
    body = body.cut(_annular_segment(0.077, 0.068, 0.040, 0.052))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_rotary_indexing_head")

    cast_iron = model.material("dark_cast_iron", rgba=(0.075, 0.080, 0.085, 1.0))
    blue_steel = model.material("blue_gray_steel", rgba=(0.12, 0.15, 0.17, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_oxide", rgba=(0.015, 0.016, 0.017, 1.0))
    bolt_steel = model.material("socket_screw_steel", rgba=(0.24, 0.25, 0.25, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Box((0.30, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="mount_plate",
    )
    shaft.visual(
        Cylinder(radius=0.052, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=brushed,
        name="lower_bearing_post",
    )
    shaft.visual(
        Cylinder(radius=0.034, length=0.545),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=brushed,
        name="vertical_core",
    )
    shaft.visual(
        Cylinder(radius=0.049, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        material=bolt_steel,
        name="shaft_retainer_nut",
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_mesh(), "base_body", tolerance=0.0009, angular_tolerance=0.04),
        material=cast_iron,
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_annular_segment(0.250, 0.067, 0.060, 0.069), "base_bearing_land"),
        material=brushed,
        name="base_bearing_land",
    )
    for index in range(8):
        angle = math.tau * index / 8.0
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(0.292 * math.cos(angle), 0.292 * math.sin(angle), 0.029),
            ),
            material=bolt_steel,
            name=f"base_screw_{index}",
        )

    collar = model.part("collar")
    collar.visual(
        mesh_from_cadquery(_collar_body_mesh(), "collar_body", tolerance=0.0009, angular_tolerance=0.04),
        material=blue_steel,
        name="collar_body",
    )
    collar.visual(
        mesh_from_cadquery(_annular_segment(0.228, 0.061, 0.066, 0.076), "collar_retainer_land"),
        material=brushed,
        name="collar_retainer_land",
    )
    collar.visual(
        Cylinder(radius=0.010, length=0.130),
        origin=Origin(xyz=(0.245, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="clamp_screw",
    )
    for y, suffix in ((-0.069, "0"), (0.069, "1")):
        collar.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(0.245, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt_steel,
            name=f"clamp_screw_head_{suffix}",
        )

    nose = model.part("nose")
    nose.visual(
        mesh_from_cadquery(_nose_body_mesh(), "nose_body", tolerance=0.0009, angular_tolerance=0.04),
        material=brushed,
        name="nose_body",
    )
    nose.visual(
        mesh_from_cadquery(_annular_segment(0.107, 0.047, -0.057, -0.046), "nose_lower_land"),
        material=blue_steel,
        name="nose_lower_land",
    )
    nose.visual(
        mesh_from_cadquery(_annular_segment(0.058, 0.047, 0.079, 0.086), "nose_retainer_lip"),
        material=black,
        name="nose_retainer_lip",
    )
    for index in range(3):
        angle = math.tau * index / 3.0 + math.pi / 6.0
        nose.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(
                xyz=(0.082 * math.cos(angle), 0.082 * math.sin(angle), -0.046),
            ),
            material=bolt_steel,
            name=f"nose_face_screw_{index}",
        )

    # Independent coaxial revolute stages: all axes run along the fixed vertical core.
    model.articulation(
        "shaft_to_base",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=base,
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=2.0, lower=0.0, upper=math.tau),
    )
    model.articulation(
        "shaft_to_collar",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.243)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.5, lower=0.0, upper=math.tau),
    )
    model.articulation(
        "shaft_to_nose",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=nose,
        origin=Origin(xyz=(0.0, 0.0, 0.402)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=3.0, lower=0.0, upper=math.tau),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    base = object_model.get_part("base")
    collar = object_model.get_part("collar")
    nose = object_model.get_part("nose")
    base_joint = object_model.get_articulation("shaft_to_base")
    collar_joint = object_model.get_articulation("shaft_to_collar")
    nose_joint = object_model.get_articulation("shaft_to_nose")

    ctx.check(
        "three coaxial revolute stages",
        all(j.axis == (0.0, 0.0, 1.0) and j.articulation_type == ArticulationType.REVOLUTE for j in (base_joint, collar_joint, nose_joint)),
        details="base, collar, and nose must each rotate about the shared vertical shaft",
    )

    with ctx.pose({base_joint: 0.0, collar_joint: 0.0, nose_joint: 0.0}):
        ctx.expect_gap(collar, base, axis="z", max_gap=0.003, max_penetration=0.001, positive_elem="collar_body", negative_elem="base_bearing_land", name="base to collar thrust land seating")
        ctx.expect_gap(nose, collar, axis="z", max_gap=0.003, max_penetration=0.001, positive_elem="nose_body", negative_elem="collar_retainer_land", name="collar to nose thrust land seating")
        ctx.expect_gap(base, shaft, axis="z", max_gap=0.003, max_penetration=0.001, positive_elem="base_body", negative_elem="mount_plate", name="rotary base seats on fixed mount plate")
        base_aabb = ctx.part_world_aabb(base)
        collar_aabb = ctx.part_world_aabb(collar)
        nose_aabb = ctx.part_world_aabb(nose)

    if base_aabb and collar_aabb and nose_aabb:
        base_diameter = max(base_aabb[1][0] - base_aabb[0][0], base_aabb[1][1] - base_aabb[0][1])
        collar_diameter = max(collar_aabb[1][0] - collar_aabb[0][0], collar_aabb[1][1] - collar_aabb[0][1])
        nose_diameter = max(nose_aabb[1][0] - nose_aabb[0][0], nose_aabb[1][1] - nose_aabb[0][1])
        ctx.check(
            "stepped drum diameters",
            base_diameter > collar_diameter > nose_diameter,
            details=f"diameters base={base_diameter:.3f}, collar={collar_diameter:.3f}, nose={nose_diameter:.3f}",
        )
    else:
        ctx.fail("stepped drum diameters", "could not compute stage AABBs")

    with ctx.pose({base_joint: 1.4, collar_joint: 2.2, nose_joint: 0.9}):
        ctx.expect_gap(collar, base, axis="z", max_penetration=0.001, positive_elem="collar_body", negative_elem="base_bearing_land", name="rotated collar does not clip into base")
        ctx.expect_gap(nose, collar, axis="z", max_penetration=0.001, positive_elem="nose_body", negative_elem="collar_retainer_land", name="rotated nose does not clip into collar")

    return ctx.report()


object_model = build_object_model()
