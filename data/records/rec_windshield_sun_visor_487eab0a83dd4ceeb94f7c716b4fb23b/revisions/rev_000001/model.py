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


def _rounded_visor_panel(width: float, depth: float, thickness: float) -> cq.Workplane:
    """A soft-cornered, lightly beveled visor board centered on the origin."""

    return (
        cq.Workplane("XY")
        .box(width, depth, thickness)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z or <Z")
        .fillet(0.0025)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_windshield_sun_visor")

    dark_plastic = model.material("charcoal_textured_plastic", rgba=(0.05, 0.055, 0.055, 1.0))
    warm_fabric = model.material("warm_gray_visor_fabric", rgba=(0.58, 0.55, 0.49, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    satin_metal = model.material("satin_pivot_metal", rgba=(0.58, 0.57, 0.53, 1.0))
    glass = model.material("pale_translucent_windshield", rgba=(0.50, 0.72, 0.88, 0.32))
    mirror = model.material("small_vanity_mirror", rgba=(0.72, 0.80, 0.86, 0.85))

    roof_header = model.part("roof_header")
    roof_header.visual(
        Box((0.42, 0.18, 0.012)),
        origin=Origin(xyz=(0.0, -0.01, 0.006)),
        material=dark_plastic,
        name="desktop_base",
    )
    roof_header.visual(
        Box((0.38, 0.055, 0.025)),
        origin=Origin(xyz=(0.0, -0.035, 0.235)),
        material=dark_plastic,
        name="roof_crossbar",
    )
    for x, name in ((-0.195, "pillar_0"), (0.195, "pillar_1")):
        roof_header.visual(
            Box((0.018, 0.040, 0.230)),
            origin=Origin(xyz=(x, -0.035, 0.123)),
            material=dark_plastic,
            name=name,
        )
    roof_header.visual(
        Box((0.34, 0.006, 0.210)),
        origin=Origin(xyz=(0.0, -0.073, 0.116)),
        material=glass,
        name="windshield_pane",
    )
    roof_header.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(-0.150, -0.035, 0.2185)),
        material=dark_plastic,
        name="secondary_socket",
    )
    roof_header.visual(
        Box((0.040, 0.020, 0.016)),
        origin=Origin(xyz=(0.155, -0.035, 0.215)),
        material=dark_plastic,
        name="stow_clip",
    )

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=satin_metal,
        name="secondary_stem",
    )
    swing_arm.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_plastic,
        name="secondary_washer",
    )
    swing_arm.visual(
        Box((0.310, 0.008, 0.008)),
        origin=Origin(xyz=(0.150, -0.022, -0.020)),
        material=dark_plastic,
        name="roof_hinge_arm",
    )
    swing_arm.visual(
        Cylinder(radius=0.005, length=0.300),
        origin=Origin(xyz=(0.150, 0.0, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_rod",
    )
    for x, name in ((0.104, "hinge_web_0"), (0.196, "hinge_web_1")):
        swing_arm.visual(
            Box((0.016, 0.024, 0.020)),
            origin=Origin(xyz=(x, -0.011, -0.024)),
            material=dark_plastic,
            name=name,
        )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        mesh_from_cadquery(_rounded_visor_panel(0.300, 0.110, 0.010), "rounded_visor_panel"),
        origin=Origin(xyz=(0.150, 0.055, -0.006)),
        material=warm_fabric,
        name="rounded_panel",
    )
    for x, name, length in (
        (0.052, "hinge_sleeve_0", 0.070),
        (0.150, "hinge_sleeve_1", 0.060),
        (0.248, "hinge_sleeve_2", 0.070),
    ):
        visor_panel.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_fabric,
            name=name,
        )
    visor_panel.visual(
        Box((0.118, 0.043, 0.003)),
        origin=Origin(xyz=(0.095, 0.057, -0.012)),
        material=mirror,
        name="vanity_mirror",
    )
    for xyz, size, name in (
        ((0.095, 0.084, -0.012), (0.138, 0.006, 0.003), "mirror_bezel_top"),
        ((0.095, 0.030, -0.012), (0.138, 0.006, 0.003), "mirror_bezel_bottom"),
        ((0.164, 0.057, -0.012), (0.006, 0.054, 0.003), "mirror_bezel_side_0"),
        ((0.026, 0.057, -0.012), (0.006, 0.054, 0.003), "mirror_bezel_side_1"),
    ):
        visor_panel.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=dark_plastic,
            name=name,
        )
    for x, name in ((0.038, "bumper_0"), (0.262, "bumper_1")):
        visor_panel.visual(
            Box((0.018, 0.010, 0.004)),
            origin=Origin(xyz=(x, 0.100, 0.001)),
            material=black_rubber,
            name=name,
        )

    model.articulation(
        "secondary_swing",
        ArticulationType.REVOLUTE,
        parent=roof_header,
        child=swing_arm,
        origin=Origin(xyz=(-0.150, -0.035, 0.2145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.45),
    )
    model.articulation(
        "primary_hinge",
        ArticulationType.REVOLUTE,
        parent=swing_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=1.8, lower=0.0, upper=1.58),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_header = object_model.get_part("roof_header")
    swing_arm = object_model.get_part("swing_arm")
    visor_panel = object_model.get_part("visor_panel")
    primary_hinge = object_model.get_articulation("primary_hinge")
    secondary_swing = object_model.get_articulation("secondary_swing")

    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1", "hinge_sleeve_2"):
        ctx.allow_overlap(
            swing_arm,
            visor_panel,
            elem_a="hinge_rod",
            elem_b=sleeve,
            reason="The visible hinge rod is intentionally captured inside the visor sleeve barrel.",
        )
        ctx.expect_overlap(
            swing_arm,
            visor_panel,
            axes="x",
            elem_a="hinge_rod",
            elem_b=sleeve,
            min_overlap=0.045,
            name=f"{sleeve} has retained rod engagement",
        )
        ctx.expect_within(
            swing_arm,
            visor_panel,
            axes="yz",
            inner_elem="hinge_rod",
            outer_elem=sleeve,
            margin=0.001,
            name=f"{sleeve} surrounds the hinge rod centerline",
        )

    ctx.expect_contact(
        roof_header,
        swing_arm,
        elem_a="secondary_socket",
        elem_b="secondary_stem",
        contact_tol=0.001,
        name="secondary swing stem seats in roof socket",
    )
    ctx.expect_gap(
        visor_panel,
        roof_header,
        axis="y",
        min_gap=0.015,
        positive_elem="rounded_panel",
        negative_elem="windshield_pane",
        name="stowed visor clears windshield glass",
    )

    stowed_aabb = ctx.part_world_aabb(visor_panel)
    with ctx.pose({primary_hinge: 1.25}):
        deployed_aabb = ctx.part_world_aabb(visor_panel)
        ctx.expect_gap(
            visor_panel,
            roof_header,
            axis="z",
            max_penetration=0.0,
            positive_elem="rounded_panel",
            negative_elem="desktop_base",
            name="deployed visor stays above desktop base",
        )
    ctx.check(
        "primary hinge folds visor downward from flat stow",
        stowed_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < stowed_aabb[0][2] - 0.035,
        details=f"stowed_aabb={stowed_aabb}, deployed_aabb={deployed_aabb}",
    )

    rest_aabb = ctx.part_element_world_aabb(visor_panel, elem="rounded_panel")
    with ctx.pose({secondary_swing: 1.25}):
        side_aabb = ctx.part_element_world_aabb(visor_panel, elem="rounded_panel")
        ctx.expect_gap(
            visor_panel,
            roof_header,
            axis="y",
            min_gap=0.010,
            positive_elem="rounded_panel",
            negative_elem="windshield_pane",
            name="side-swing pose clears windshield pane",
        )
    ctx.check(
        "secondary pivot swings visor toward side-window orientation",
        rest_aabb is not None
        and side_aabb is not None
        and (side_aabb[0][1] + side_aabb[1][1]) * 0.5 > (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5 + 0.070,
        details=f"rest_aabb={rest_aabb}, side_aabb={side_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
