from __future__ import annotations

import cadquery as cq
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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_min: float):
    """CadQuery tube with an open center, authored in meters."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cold_climate_revolving_door")

    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.06, 0.07, 0.075, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.55, 0.58, 0.58, 1.0))
    warm_insulation = model.material("warm_insulated_panel", rgba=(0.86, 0.84, 0.78, 1.0))
    rubber = model.material("black_weather_seal", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("pale_low_e_glass", rgba=(0.55, 0.78, 0.92, 0.34))

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=1.20, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_aluminum,
        name="floor_plate",
    )
    drum.visual(
        Cylinder(radius=1.22, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        material=dark_aluminum,
        name="roof_cap",
    )
    drum.visual(
        mesh_from_cadquery(_annular_cylinder(1.13, 1.09, 2.26, 0.07), "continuous_glass_drum"),
        material=glass,
        name="outer_glass",
    )
    drum.visual(
        mesh_from_cadquery(_annular_cylinder(1.16, 1.04, 0.06, 0.06), "lower_sill_ring"),
        material=brushed_aluminum,
        name="lower_sill",
    )
    drum.visual(
        mesh_from_cadquery(_annular_cylinder(1.16, 1.04, 0.06, 2.30), "upper_header_ring"),
        material=brushed_aluminum,
        name="upper_header",
    )
    drum.visual(
        mesh_from_cadquery(_annular_cylinder(0.26, 0.17, 0.05, 0.08), "lower_bearing_collar"),
        material=dark_aluminum,
        name="lower_bearing",
    )
    drum.visual(
        mesh_from_cadquery(_annular_cylinder(0.26, 0.17, 0.05, 2.26), "upper_bearing_collar"),
        material=dark_aluminum,
        name="upper_bearing",
    )

    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        drum.visual(
            Box((0.78, 0.045, 0.045)),
            origin=Origin(
                xyz=(0.65 * math.cos(angle), 0.65 * math.sin(angle), 2.285),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"ceiling_spoke_{i}",
        )

    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius = 1.18
        drum.visual(
            Box((0.055, 0.08, 2.28)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 1.20),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"mullion_{i}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.13, length=2.18),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=warm_insulation,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.245, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1475)),
        material=brushed_aluminum,
        name="lower_post_cap",
    )
    rotor.visual(
        Cylinder(radius=0.245, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 2.2425)),
        material=brushed_aluminum,
        name="upper_post_cap",
    )

    wing_length = 0.94
    wing_center = 0.60
    wing_thickness = 0.105
    wing_height = 2.04
    wing_z = 1.20
    wing_names = ("wing_0", "wing_1", "wing_2", "wing_3")
    for i, (angle, wing_name) in enumerate(zip((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0), wing_names)):
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            Box((wing_length, wing_thickness, wing_height)),
            origin=Origin(
                xyz=(wing_center * c, wing_center * s, wing_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=warm_insulation,
            name=wing_name,
        )
        rotor.visual(
            Box((0.035, 0.16, wing_height)),
            origin=Origin(
                xyz=((wing_center + wing_length / 2.0 - 0.008) * c, (wing_center + wing_length / 2.0 - 0.008) * s, wing_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"outer_seal_{i}",
        )
        rotor.visual(
            Box((wing_length, 0.032, 0.055)),
            origin=Origin(
                xyz=(wing_center * c, wing_center * s, wing_z + wing_height / 2.0 - 0.03),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_aluminum,
            name=f"top_rail_{i}",
        )
        rotor.visual(
            Box((wing_length, 0.032, 0.055)),
            origin=Origin(
                xyz=(wing_center * c, wing_center * s, wing_z - wing_height / 2.0 + 0.03),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_aluminum,
            name=f"bottom_rail_{i}",
        )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("drum_to_rotor")

    ctx.check(
        "rotor uses continuous vertical articulation",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        inner_elem="wing_0",
        outer_elem="outer_glass",
        margin=0.0,
        name="wide wing fits inside enclosed drum",
    )
    ctx.expect_gap(
        drum,
        rotor,
        axis="z",
        positive_elem="roof_cap",
        negative_elem="wing_0",
        min_gap=0.08,
        name="insulated wing clears the roof canopy",
    )
    ctx.expect_gap(
        rotor,
        drum,
        axis="z",
        positive_elem="wing_0",
        negative_elem="floor_plate",
        min_gap=0.08,
        name="insulated wing clears the raised sill",
    )

    with ctx.pose({joint: math.pi / 4.0}):
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            inner_elem="wing_0",
            outer_elem="outer_glass",
            margin=0.0,
            name="rotated wing remains inside drum",
        )

    return ctx.report()


object_model = build_object_model()
