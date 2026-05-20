from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_transfer_axis")

    dark_cast = model.material("dark_cast_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    blue_carriage = model.material("blue_anodized_carriage", rgba=(0.05, 0.16, 0.30, 1.0))
    orange_tab = model.material("orange_support_tab", rgba=(0.95, 0.36, 0.06, 1.0))
    fastener = model.material("black_oxide_fasteners", rgba=(0.025, 0.025, 0.023, 1.0))

    rail_body = model.part("rail_body")
    rail_body.visual(
        Box((0.780, 0.210, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_cast,
        name="ground_plate",
    )
    rail_body.visual(
        Box((0.700, 0.080, 0.076)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=dark_cast,
        name="guide_spine",
    )
    rail_body.visual(
        Cylinder(radius=0.012, length=0.680),
        origin=Origin(xyz=(0.0, -0.066, 0.118), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="round_way_front",
    )
    rail_body.visual(
        Cylinder(radius=0.012, length=0.680),
        origin=Origin(xyz=(0.0, 0.066, 0.118), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="round_way_rear",
    )
    for x, name in ((-0.360, "rear_stop"), (0.360, "front_stop")):
        rail_body.visual(
            Box((0.042, 0.180, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.074)),
            material=dark_cast,
            name=name,
        )
    for x in (-0.295, 0.295):
        for y in (-0.078, 0.078):
            rail_body.visual(
                Cylinder(radius=0.014, length=0.007),
                origin=Origin(xyz=(x, y, 0.0315)),
                material=fastener,
                name=f"base_bolt_{x:+.3f}_{y:+.3f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.240, 0.182, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=blue_carriage,
        name="carriage_head",
    )
    carriage.visual(
        Box((0.218, 0.092, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=blue_carriage,
        name="bearing_tunnel",
    )
    for x, name in ((-0.127, "rear_wiper"), (0.127, "front_wiper")):
        carriage.visual(
            Box((0.018, 0.070, 0.034)),
            origin=Origin(xyz=(x, 0.0, -0.025)),
            material=black_rubber,
            name=name,
        )
    for y in (-0.058, 0.0, 0.058):
        carriage.visual(
            Box((0.172, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.065)),
            material=blue_carriage,
            name=f"top_rib_{y:+.3f}",
        )
    for y, name in ((-0.099, "front_side_rib"), (0.099, "rear_side_rib")):
        carriage.visual(
            Box((0.196, 0.016, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.004)),
            material=blue_carriage,
            name=name,
        )

    # A front clevis, built into the moving carriage, supports the carried tab.
    carriage.visual(
        Box((0.170, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, -0.097, 0.012)),
        material=blue_carriage,
        name="clevis_back_boss",
    )
    carriage.visual(
        Box((0.026, 0.052, 0.082)),
        origin=Origin(xyz=(-0.068, -0.113, 0.010)),
        material=blue_carriage,
        name="clevis_cheek_0",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.068, -0.141, 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="clevis_cheek_0_bore_lip",
    )
    carriage.visual(
        Box((0.026, 0.052, 0.082)),
        origin=Origin(xyz=(0.068, -0.113, 0.010)),
        material=blue_carriage,
        name="clevis_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.068, -0.141, 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="clevis_cheek_1_bore_lip",
    )

    support_tab = model.part("support_tab")
    support_tab.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_knuckle",
    )
    support_tab.visual(
        Box((0.084, 0.018, 0.132)),
        origin=Origin(xyz=(0.0, -0.006, -0.066)),
        material=orange_tab,
        name="tab_plate",
    )
    support_tab.visual(
        Box((0.092, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, -0.008, -0.018)),
        material=orange_tab,
        name="tab_root_fillet",
    )
    support_tab.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(0.0, -0.016, -0.113)),
        material=fastener,
        name="tab_pad_boss",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_body,
        child=carriage,
        origin=Origin(xyz=(-0.120, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=0.35, lower=0.0, upper=0.240),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=support_tab,
        origin=Origin(xyz=(0.0, -0.126, 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_body = object_model.get_part("rail_body")
    carriage = object_model.get_part("carriage")
    support_tab = object_model.get_part("support_tab")
    slide = object_model.get_articulation("rail_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

    ctx.allow_overlap(
        rail_body,
        carriage,
        elem_a="guide_spine",
        elem_b="bearing_tunnel",
        reason=(
            "The blue bearing tunnel is intentionally modeled as a retained "
            "linear-bearing sleeve riding over the fixed guide spine."
        ),
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_within(
            carriage,
            rail_body,
            axes="y",
            inner_elem="bearing_tunnel",
            outer_elem="guide_spine",
            margin=0.008,
            name="bearing tunnel centered on guide width",
        )
        ctx.expect_overlap(
            carriage,
            rail_body,
            axes="x",
            elem_a="bearing_tunnel",
            elem_b="guide_spine",
            min_overlap=0.210,
            name="carriage retained on rail at home",
        )
        ctx.expect_gap(
            carriage,
            rail_body,
            axis="z",
            positive_elem="bearing_tunnel",
            negative_elem="guide_spine",
            max_penetration=0.014,
            name="bearing tunnel locally seats over guide spine",
        )
        ctx.expect_gap(
            rail_body,
            support_tab,
            axis="y",
            min_gap=0.004,
            name="folded tab stays in front of rail envelope",
        )
        ctx.expect_gap(
            support_tab,
            rail_body,
            axis="z",
            negative_elem="ground_plate",
            min_gap=0.004,
            name="folded tab clears the base plate",
        )
        ctx.expect_overlap(
            support_tab,
            carriage,
            axes="x",
            elem_a="hinge_knuckle",
            elem_b="clevis_back_boss",
            min_overlap=0.080,
            name="tab hinge spans the front clevis",
        )
        ctx.expect_contact(
            support_tab,
            carriage,
            elem_a="hinge_knuckle",
            elem_b="clevis_cheek_0",
            contact_tol=0.001,
            name="hinge knuckle is captured by a clevis cheek",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.240, hinge: 1.10}):
        ctx.expect_overlap(
            carriage,
            rail_body,
            axes="x",
            elem_a="bearing_tunnel",
            elem_b="guide_spine",
            min_overlap=0.210,
            name="carriage retained on rail at full stroke",
        )
        ctx.expect_gap(
            carriage,
            rail_body,
            axis="y",
            positive_elem="bearing_tunnel",
            negative_elem="round_way_front",
            min_gap=0.006,
            name="bearing tunnel clears front round way",
        )
        ctx.expect_gap(
            rail_body,
            support_tab,
            axis="y",
            min_gap=0.004,
            name="raised tab stays in front of rail envelope",
        )
        ctx.expect_gap(
            carriage,
            support_tab,
            axis="y",
            positive_elem="carriage_head",
            negative_elem="tab_plate",
            min_gap=0.020,
            name="raised tab plate clears carriage face",
        )
        ctx.expect_contact(
            support_tab,
            carriage,
            elem_a="hinge_knuckle",
            elem_b="clevis_cheek_1",
            contact_tol=0.001,
            name="raised hinge remains captured in clevis",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage moves along the rail axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.220,
        details=f"home={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
