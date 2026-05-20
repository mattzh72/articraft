from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _revolved_shell(profile: list[tuple[float, float]]) -> cq.Workplane:
    """Revolve an X/Z cross-section profile around the vertical Z axis."""
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0, -1.0), axisEnd=(0.0, 0.0, 1.0))
    )


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A centered circular collar with a through-hole along local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _tapered_blade(length: float, root_width: float, tip_width: float, thickness: float) -> cq.Workplane:
    """Wide weatherproof fan blade with rounded root and tip."""
    return (
        cq.Workplane("XY")
        .moveTo(0.0, -root_width / 2.0)
        .lineTo(length * 0.76, -tip_width / 2.0)
        .threePointArc((length + 0.035, 0.0), (length * 0.76, tip_width / 2.0))
        .lineTo(0.0, root_width / 2.0)
        .threePointArc((-0.040, 0.0), (0.0, -root_width / 2.0))
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def _blade_arm(length: float, root_width: float, blade_width: float, thickness: float) -> cq.Workplane:
    """Broad tapered metal blade arm from hub to blade root."""
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (0.0, -root_width / 2.0),
                (length, -blade_width / 2.0),
                (length, blade_width / 2.0),
                (0.0, root_width / 2.0),
            ]
        )
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wet_rated_ceiling_fan")

    powder_coat = model.material("powder_coated_dark_bronze", rgba=(0.08, 0.075, 0.065, 1.0))
    rubber = model.material("black_rubber_gaskets", rgba=(0.01, 0.012, 0.012, 1.0))
    blade_finish = model.material("weathered_composite_blades", rgba=(0.42, 0.36, 0.27, 1.0))
    blade_edge = model.material("sealed_blade_edges", rgba=(0.18, 0.16, 0.13, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=0.23, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=powder_coat,
        name="ceiling_plate",
    )
    canopy_shell = _revolved_shell(
        [
            (0.180, 0.000),
            (0.150, -0.020),
            (0.090, -0.120),
            (0.062, -0.120),
            (0.118, -0.016),
            (0.145, 0.000),
        ]
    )
    canopy.visual(
        mesh_from_cadquery(canopy_shell, "canopy_bell", tolerance=0.0015),
        material=powder_coat,
        name="canopy_bell",
    )
    canopy.visual(
        mesh_from_cadquery(_ring(0.086, 0.056, 0.040), "swivel_socket", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
        material=powder_coat,
        name="swivel_socket",
    )
    canopy.visual(
        mesh_from_cadquery(_ring(0.060, 0.026, 0.012), "socket_gasket", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.164)),
        material=rubber,
        name="socket_gasket",
    )

    downrod = model.part("downrod")
    downrod.visual(
        Sphere(radius=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder_coat,
        name="swivel_ball",
    )
    downrod.visual(
        Cylinder(radius=0.020, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=powder_coat,
        name="rod_tube",
    )
    downrod.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.625)),
        material=powder_coat,
        name="lower_coupler",
    )
    downrod.visual(
        Cylinder(radius=0.027, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=rubber,
        name="ball_boot",
    )

    motor = model.part("motor")
    motor.visual(
        Cylinder(radius=0.050, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=powder_coat,
        name="top_coupler",
    )
    motor_body = _revolved_shell(
        [
            (0.000, -0.315),
            (0.115, -0.315),
            (0.180, -0.270),
            (0.192, -0.165),
            (0.176, -0.075),
            (0.112, -0.045),
            (0.000, -0.045),
        ]
    )
    motor.visual(
        mesh_from_cadquery(motor_body, "sealed_motor_housing", tolerance=0.001),
        material=powder_coat,
        name="sealed_housing",
    )
    motor.visual(
        Cylinder(radius=0.194, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=rubber,
        name="upper_gasket_band",
    )
    motor.visual(
        Cylinder(radius=0.194, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.260)),
        material=rubber,
        name="lower_gasket_band",
    )
    motor.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        material=powder_coat,
        name="bottom_bearing",
    )

    rotor = model.part("blade_assembly")
    rotor.visual(
        Cylinder(radius=0.102, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=powder_coat,
        name="hub",
    )
    rotor.visual(
        Sphere(radius=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=powder_coat,
        name="hub_finial",
    )

    arm_mesh = _blade_arm(0.355, 0.075, 0.130, 0.024)
    blade_mesh = _tapered_blade(0.640, 0.185, 0.155, 0.020)
    for index, (yaw, radial_sign) in enumerate(((0.0, 1.0), (math.pi, -1.0))):
        rotor.visual(
            mesh_from_cadquery(arm_mesh, f"blade_arm_{index}", tolerance=0.001),
            origin=Origin(xyz=(0.070 * radial_sign, 0.0, -0.045), rpy=(0.06, 0.0, yaw)),
            material=powder_coat,
            name=f"blade_arm_{index}",
        )
        rotor.visual(
            mesh_from_cadquery(blade_mesh, f"blade_{index}", tolerance=0.001),
            origin=Origin(xyz=(0.395 * radial_sign, 0.0, -0.064), rpy=(0.11, 0.0, yaw)),
            material=blade_finish,
            name=f"blade_{index}",
        )
        rotor.visual(
            Box((0.520, 0.018, 0.014)),
            origin=Origin(xyz=(0.705 * radial_sign, 0.0, -0.057), rpy=(0.11, 0.0, yaw)),
            material=blade_edge,
            name=f"raised_rib_{index}",
        )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
    )
    model.articulation(
        "downrod_to_motor",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, -0.655)),
    )
    model.articulation(
        "motor_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor = object_model.get_part("motor")
    rotor = object_model.get_part("blade_assembly")
    downrod = object_model.get_part("downrod")
    canopy = object_model.get_part("canopy")
    swivel = object_model.get_articulation("canopy_to_downrod")
    spin = object_model.get_articulation("motor_to_blade_assembly")

    ctx.allow_overlap(
        canopy,
        downrod,
        elem_a="socket_gasket",
        elem_b="swivel_ball",
        reason="The wet-rated rubber gasket intentionally compresses around the swivel ball to seal the downrod opening.",
    )

    ctx.check(
        "canopy uses a fixed swivel mount",
        swivel.articulation_type == ArticulationType.FIXED,
        details=f"joint_type={swivel.articulation_type}",
    )
    ctx.check(
        "blade assembly has continuous rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"joint_type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_overlap(
        "blade_assembly",
        "motor",
        axes="xy",
        elem_a="hub",
        elem_b="bottom_bearing",
        min_overlap=0.060,
        name="rotor hub is centered under motor bearing",
    )
    ctx.expect_gap(
        motor,
        rotor,
        axis="z",
        positive_elem="bottom_bearing",
        negative_elem="hub",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotor hub seats below sealed motor",
    )
    ctx.expect_overlap(
        downrod,
        canopy,
        axes="xy",
        elem_a="swivel_ball",
        elem_b="swivel_socket",
        min_overlap=0.045,
        name="swivel ball is captured in canopy socket footprint",
    )
    ctx.expect_within(
        downrod,
        canopy,
        axes="xy",
        inner_elem="swivel_ball",
        outer_elem="socket_gasket",
        margin=0.002,
        name="rubber gasket surrounds the swivel ball",
    )

    rest_aabb = ctx.part_world_aabb(rotor)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(rotor)
    rest_dx = rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb is not None else 0.0
    rest_dy = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else 0.0
    turned_dx = turned_aabb[1][0] - turned_aabb[0][0] if turned_aabb is not None else 0.0
    turned_dy = turned_aabb[1][1] - turned_aabb[0][1] if turned_aabb is not None else 0.0
    ctx.check(
        "continuous joint rotates the two-blade span",
        rest_dx > 1.75 and rest_dy < 0.45 and turned_dy > 1.75 and turned_dx < 0.45,
        details=f"rest_dx={rest_dx:.3f}, rest_dy={rest_dy:.3f}, turned_dx={turned_dx:.3f}, turned_dy={turned_dy:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
