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
    TorusGeometry,
    ExtrudeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _hollow_tube_x(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """CadQuery annular tube centered on the local X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_windshield_sun_visor")

    tan_headliner = model.material("aged_tan_headliner", rgba=(0.63, 0.54, 0.42, 1.0))
    visor_vinyl = model.material("warm_cream_vinyl", rgba=(0.78, 0.69, 0.52, 1.0))
    hatch_plate = model.material("faded_service_plate", rgba=(0.56, 0.50, 0.40, 1.0))
    black_rubber = model.material("black_rubber_edge", rgba=(0.025, 0.023, 0.020, 1.0))
    oxidized_steel = model.material("oxidized_bolted_steel", rgba=(0.30, 0.31, 0.30, 1.0))
    brushed_pin = model.material("brushed_steel_pin", rgba=(0.62, 0.62, 0.58, 1.0))
    dark_screw = model.material("dark_slot_screws", rgba=(0.08, 0.075, 0.065, 1.0))

    roof_header = model.part("roof_header")
    roof_header.visual(
        Box((0.70, 0.14, 0.035)),
        origin=Origin(xyz=(0.22, 0.0, 0.055)),
        material=tan_headliner,
        name="padded_header",
    )
    roof_header.visual(
        Box((0.18, 0.105, 0.014)),
        origin=Origin(xyz=(0.015, 0.0, 0.031)),
        material=oxidized_steel,
        name="bolted_adapter",
    )
    roof_header.visual(
        mesh_from_geometry(TorusGeometry(0.028, 0.0055, radial_segments=40, tubular_segments=12), "secondary_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=brushed_pin,
        name="secondary_bearing_ring",
    )
    roof_header.visual(
        Box((0.070, 0.011, 0.023)),
        origin=Origin(xyz=(0.024, -0.030, 0.019), rpy=(0.0, -0.55, 0.0)),
        material=oxidized_steel,
        name="front_gusset",
    )
    roof_header.visual(
        Box((0.070, 0.011, 0.023)),
        origin=Origin(xyz=(0.024, 0.030, 0.019), rpy=(0.0, -0.55, 0.0)),
        material=oxidized_steel,
        name="rear_gusset",
    )
    for i, (bx, by) in enumerate(((-0.052, -0.034), (-0.052, 0.034), (0.075, -0.034), (0.075, 0.034))):
        roof_header.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(bx, by, 0.023)),
            material=dark_screw,
            name=f"adapter_bolt_{i}",
        )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.014, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=brushed_pin,
        name="secondary_swivel_pin",
    )
    hinge_arm.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=oxidized_steel,
        name="retaining_washer",
    )
    hinge_arm.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=oxidized_steel,
        name="lower_swivel_collar",
    )
    hinge_arm.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.020, 0.0, -0.066)),
        material=brushed_pin,
        name="drop_link",
    )
    hinge_arm.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.020, 0.0, -0.091)),
        material=brushed_pin,
        name="elbow_boss",
    )
    hinge_arm.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.070, 0.0, -0.091), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_pin,
        name="roof_hinge_arm",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0094, length=0.240),
        origin=Origin(xyz=(0.180, 0.0, -0.091), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_pin,
        name="primary_pin",
    )
    hinge_arm.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.064, 0.0, -0.091), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oxidized_steel,
        name="primary_stop_washer",
    )
    hinge_arm.visual(
        Box((0.050, 0.045, 0.009)),
        origin=Origin(xyz=(0.095, 0.0, -0.075), rpy=(0.0, 0.38, 0.0)),
        material=oxidized_steel,
        name="arm_reinforcement",
    )

    visor = model.part("visor_panel")
    visor_body_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.540, 0.245, 0.036, corner_segments=10), 0.024),
        "rounded_padded_visor",
    )
    visor.visual(
        visor_body_mesh,
        origin=Origin(xyz=(0.290, 0.0, -0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=visor_vinyl,
        name="padded_panel",
    )
    visor.visual(
        mesh_from_cadquery(_hollow_tube_x(0.430, 0.0145, 0.0094), "visor_hinge_sleeve"),
        origin=Origin(xyz=(0.245, 0.0, -0.002)),
        material=oxidized_steel,
        name="hinge_sleeve",
    )
    visor.visual(
        Box((0.500, 0.006, 0.014)),
        origin=Origin(xyz=(0.290, -0.014, -0.252)),
        material=black_rubber,
        name="lower_edge_binding",
    )
    visor.visual(
        Box((0.510, 0.006, 0.012)),
        origin=Origin(xyz=(0.292, -0.014, -0.018)),
        material=oxidized_steel,
        name="upper_steel_binding",
    )
    visor.visual(
        Box((0.160, 0.006, 0.033)),
        origin=Origin(xyz=(0.170, -0.017, -0.035)),
        material=oxidized_steel,
        name="hinge_adapter_strap",
    )
    visor.visual(
        Box((0.090, 0.006, 0.074)),
        origin=Origin(xyz=(0.105, -0.017, -0.092), rpy=(0.0, -0.28, 0.0)),
        material=oxidized_steel,
        name="diagonal_reinforcement",
    )
    visor.visual(
        Box((0.120, 0.004, 0.078)),
        origin=Origin(xyz=(0.350, -0.0135, -0.125)),
        material=hatch_plate,
        name="service_hatch",
    )
    visor.visual(
        Box((0.070, 0.004, 0.045)),
        origin=Origin(xyz=(0.485, -0.0135, -0.118)),
        material=hatch_plate,
        name="wiring_hatch",
    )
    screw_locations = [
        (0.300, -0.0155, -0.092),
        (0.400, -0.0155, -0.092),
        (0.300, -0.0155, -0.158),
        (0.400, -0.0155, -0.158),
        (0.458, -0.0155, -0.098),
        (0.512, -0.0155, -0.098),
        (0.458, -0.0155, -0.138),
        (0.512, -0.0155, -0.138),
    ]
    for i, xyz in enumerate(screw_locations):
        visor.visual(
            Cylinder(radius=0.0055, length=0.0032),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_screw,
            name=f"hatch_screw_{i}",
        )
    for i, x in enumerate((0.128, 0.212)):
        visor.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.019, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_screw,
            name=f"strap_bolt_{i}",
        )

    model.articulation(
        "secondary_swing",
        ArticulationType.REVOLUTE,
        parent=roof_header,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "primary_flip",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor,
        origin=Origin(xyz=(0.095, 0.0, -0.091)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_header = object_model.get_part("roof_header")
    hinge_arm = object_model.get_part("hinge_arm")
    visor = object_model.get_part("visor_panel")
    secondary = object_model.get_articulation("secondary_swing")
    primary = object_model.get_articulation("primary_flip")

    ctx.allow_overlap(
        hinge_arm,
        visor,
        elem_a="primary_pin",
        elem_b="hinge_sleeve",
        reason="The steel primary pin is intentionally captured inside the simplified hinge sleeve barrel.",
    )
    ctx.expect_within(
        hinge_arm,
        roof_header,
        axes="xy",
        inner_elem="secondary_swivel_pin",
        outer_elem="secondary_bearing_ring",
        margin=0.011,
        name="secondary pivot pin sits inside bearing ring",
    )
    ctx.expect_within(
        hinge_arm,
        visor,
        axes="yz",
        inner_elem="primary_pin",
        outer_elem="hinge_sleeve",
        margin=0.003,
        name="primary pin is captured in visor sleeve",
    )
    ctx.expect_overlap(
        hinge_arm,
        visor,
        axes="x",
        elem_a="primary_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.11,
        name="primary hinge has retained pin insertion",
    )

    with ctx.pose({primary: 0.0}):
        lowered_aabb = ctx.part_world_aabb(visor)
    with ctx.pose({primary: 1.20}):
        raised_aabb = ctx.part_world_aabb(visor)
    ctx.check(
        "primary flip raises lower edge toward roof",
        lowered_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > lowered_aabb[0][2] + 0.10,
        details=f"lowered={lowered_aabb}, raised={raised_aabb}",
    )

    with ctx.pose({secondary: 0.80}):
        swung = ctx.part_world_position(visor)
    with ctx.pose({secondary: 0.0}):
        centered = ctx.part_world_position(visor)
    ctx.check(
        "secondary swing moves visor laterally",
        swung is not None
        and centered is not None
        and abs(swung[1] - centered[1]) > 0.025,
        details=f"centered={centered}, swung={swung}",
    )

    return ctx.report()


object_model = build_object_model()
