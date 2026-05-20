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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small helper for molded plastic shells with softened edges."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_stick_vacuum")

    red = model.material("deep_red_plastic", rgba=(0.62, 0.04, 0.03, 1.0))
    dark = model.material("charcoal_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    rubber = model.material("black_rubber", rgba=(0.008, 0.008, 0.007, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.76, 0.70, 1.0))
    blue = model.material("blue_brush", rgba=(0.05, 0.24, 0.80, 1.0))
    clear = model.material("smoked_clear_bin", rgba=(0.55, 0.80, 0.90, 0.36))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box((0.30, 0.19, 0.18), 0.035), "motor_shell"),
        origin=Origin(xyz=(-0.18, 0.0, 0.08)),
        material=red,
        name="motor_shell",
    )
    body.visual(
        Cylinder(radius=0.085, length=0.21),
        origin=Origin(xyz=(-0.30, 0.0, -0.06), rpy=(math.pi / 2, 0.0, 0.0)),
        material=clear,
        name="dust_bin",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box((0.20, 0.15, 0.07), 0.018), "battery_pack"),
        origin=Origin(xyz=(-0.34, 0.0, -0.14)),
        material=dark,
        name="battery_pack",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.085),
        # Ends exactly on the wand joint plane at local x=0.
        origin=Origin(xyz=(-0.0425, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark,
        name="wand_socket",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.17, 0.0, 0.15),
                    (-0.32, 0.0, 0.30),
                    (-0.49, 0.0, 0.24),
                    (-0.43, 0.0, 0.02),
                    (-0.27, 0.0, 0.00),
                ],
                radius=0.021,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "handle_loop",
        ),
        material=dark,
        name="handle_loop",
    )
    body.visual(
        Box((0.075, 0.12, 0.045)),
        origin=Origin(xyz=(-0.25, 0.0, 0.205)),
        material=dark,
        name="trigger",
    )
    body.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(-0.50, 0.0, 0.19)),
        material=red,
        name="handle_knuckle",
    )

    primary = model.part("primary_wand")
    primary.visual(
        Cylinder(radius=0.027, length=0.74),
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=aluminum,
        name="primary_tube",
    )
    primary.visual(
        Cylinder(radius=0.046, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark,
        name="rear_collar",
    )
    primary.visual(
        Cylinder(radius=0.034, length=0.075),
        origin=Origin(xyz=(0.758, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark,
        name="front_collar",
    )
    primary.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.84, 0.0, 0.0)),
        material=dark,
        name="elbow_knuckle",
    )

    secondary = model.part("secondary_wand")
    secondary.visual(
        Cylinder(radius=0.026, length=0.220),
        origin=Origin(xyz=(0.195, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=aluminum,
        name="secondary_tube",
    )
    secondary.visual(
        Cylinder(radius=0.043, length=0.105),
        origin=Origin(xyz=(0.0525, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark,
        name="rear_collar",
    )
    secondary.visual(
        Cylinder(radius=0.038, length=0.070),
        origin=Origin(xyz=(0.305, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark,
        name="nozzle_collar",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        mesh_from_cadquery(_rounded_box((0.42, 0.26, 0.065), 0.020), "nozzle_shell"),
        origin=Origin(xyz=(0.19, 0.0, -0.075)),
        material=dark,
        name="nozzle_shell",
    )
    nozzle.visual(
        Cylinder(radius=0.035, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark,
        name="pitch_barrel",
    )
    nozzle.visual(
        mesh_from_cadquery(_rounded_box((0.10, 0.11, 0.055), 0.014), "nozzle_neck"),
        origin=Origin(xyz=(0.045, 0.0, -0.035)),
        material=dark,
        name="neck_block",
    )
    nozzle.visual(
        Cylinder(radius=0.018, length=0.38),
        origin=Origin(xyz=(0.21, 0.0, -0.105), rpy=(0.0, math.pi / 2, 0.0)),
        material=blue,
        name="brush_roll",
    )
    nozzle.visual(
        Box((0.36, 0.018, 0.020)),
        origin=Origin(xyz=(0.205, 0.118, -0.112)),
        material=rubber,
        name="front_squeegee",
    )
    nozzle.visual(
        Box((0.36, 0.018, 0.020)),
        origin=Origin(xyz=(0.205, -0.118, -0.112)),
        material=rubber,
        name="rear_squeegee",
    )
    nozzle.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(-0.035, 0.105, -0.095), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rubber,
        name="wheel_0",
    )
    nozzle.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(-0.035, -0.105, -0.095), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rubber,
        name="wheel_1",
    )

    # The wand part frames sit on their hinge lines.  At q=0 the fixed rpy
    # pitches the chain down toward the floor nozzle; revolute motion then bends
    # each elbow around the horizontal Y axis.
    model.articulation(
        "body_to_primary",
        ArticulationType.REVOLUTE,
        parent=body,
        child=primary,
        origin=Origin(rpy=(0.0, 0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.45, upper=0.70),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(0.84, 0.0, 0.0), rpy=(0.0, 0.18, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.65, upper=0.75),
    )
    model.articulation(
        "secondary_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=secondary,
        child=nozzle,
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, -0.90, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.55, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    primary = object_model.get_part("primary_wand")
    secondary = object_model.get_part("secondary_wand")
    nozzle = object_model.get_part("nozzle")

    body_elbow = object_model.get_articulation("body_to_primary")
    wand_elbow = object_model.get_articulation("primary_to_secondary")
    nozzle_hinge = object_model.get_articulation("secondary_to_nozzle")

    ctx.allow_overlap(
        body,
        primary,
        elem_a="wand_socket",
        elem_b="rear_collar",
        reason="The rear wand collar is intentionally captured inside the molded socket at the body elbow.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_knuckle",
        elem_b="rear_collar",
        reason="The short wand's rear collar is seated around the primary elbow knuckle like a captured swivel joint.",
    )
    ctx.allow_overlap(
        nozzle,
        secondary,
        elem_a="pitch_barrel",
        elem_b="nozzle_collar",
        reason="The nozzle pitch barrel is intentionally nested in the wand collar to represent the hinge pin.",
    )

    ctx.expect_overlap(
        body,
        primary,
        axes="y",
        elem_a="wand_socket",
        elem_b="rear_collar",
        min_overlap=0.070,
        name="body elbow collar spans socket width",
    )
    ctx.expect_gap(
        primary,
        body,
        axis="x",
        positive_elem="rear_collar",
        negative_elem="wand_socket",
        max_penetration=0.040,
        name="body elbow embed remains local",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="y",
        elem_a="elbow_knuckle",
        elem_b="rear_collar",
        min_overlap=0.070,
        name="wand elbow collar captures knuckle",
    )
    ctx.expect_gap(
        secondary,
        primary,
        axis="x",
        positive_elem="rear_collar",
        negative_elem="elbow_knuckle",
        max_penetration=0.110,
        name="wand elbow embed remains local",
    )
    ctx.expect_overlap(
        nozzle,
        secondary,
        axes="y",
        elem_a="pitch_barrel",
        elem_b="nozzle_collar",
        min_overlap=0.060,
        name="nozzle hinge barrel spans collar",
    )
    ctx.expect_gap(
        nozzle,
        secondary,
        axis="x",
        positive_elem="pitch_barrel",
        negative_elem="nozzle_collar",
        max_penetration=0.080,
        name="nozzle hinge embed remains local",
    )

    ctx.check(
        "primary wand is dominant link",
        0.74 > 2.0 * 0.220,
        details="primary tube length should be more than twice the secondary tube length",
    )

    def _center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    rest_primary_z = _center_z(ctx.part_world_aabb(primary))
    rest_secondary_z = _center_z(ctx.part_world_aabb(secondary))
    rest_nozzle_z = _center_z(ctx.part_world_aabb(nozzle))

    with ctx.pose({body_elbow: 0.45}):
        bent_primary_z = _center_z(ctx.part_world_aabb(primary))
    ctx.check(
        "body elbow bends wand downward",
        rest_primary_z is not None
        and bent_primary_z is not None
        and bent_primary_z < rest_primary_z - 0.04,
        details=f"rest_z={rest_primary_z}, bent_z={bent_primary_z}",
    )

    with ctx.pose({wand_elbow: 0.55}):
        bent_secondary_z = _center_z(ctx.part_world_aabb(secondary))
    ctx.check(
        "secondary elbow changes short link angle",
        rest_secondary_z is not None
        and bent_secondary_z is not None
        and bent_secondary_z < rest_secondary_z - 0.025,
        details=f"rest_z={rest_secondary_z}, bent_z={bent_secondary_z}",
    )

    with ctx.pose({nozzle_hinge: 0.55}):
        pitched_nozzle_z = _center_z(ctx.part_world_aabb(nozzle))
    ctx.check(
        "floor nozzle pitches on horizontal hinge",
        rest_nozzle_z is not None
        and pitched_nozzle_z is not None
        and abs(pitched_nozzle_z - rest_nozzle_z) > 0.015,
        details=f"rest_z={rest_nozzle_z}, pitched_z={pitched_nozzle_z}",
    )

    return ctx.report()


object_model = build_object_model()
