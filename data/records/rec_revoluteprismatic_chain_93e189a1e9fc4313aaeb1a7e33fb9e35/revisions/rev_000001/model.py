from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="revolute_prismatic_chain_study",
        meta={
            "pipeline": "sdk_hybrid",
            "description": (
                "Standalone mechanical study: a trunnion-mounted revolute "
                "stage feeding a prismatic output carriage on visible guide ways."
            ),
        },
    )

    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.52, 0.54, 0.55, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.73, 0.71, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.015, 0.016, 0.018, 1.0))
    bronze = model.material("bearing_bronze", rgba=(0.72, 0.48, 0.20, 1.0))
    cover_paint = model.material("removable_cover_grey", rgba=(0.36, 0.39, 0.40, 1.0))
    safety_red = model.material("safety_red", rgba=(0.82, 0.05, 0.035, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((0.82, 0.36, 0.035)),
        origin=Origin(xyz=(0.22, 0.0, 0.0175)),
        material=dark_steel,
        name="ground_plate",
    )
    base.visual(
        Box((0.24, 0.25, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=satin_steel,
        name="trunnion_plinth",
    )
    base.visual(
        Box((0.18, 0.035, 0.220)),
        origin=Origin(xyz=(0.0, 0.100, 0.185)),
        material=satin_steel,
        name="trunnion_cheek_pos",
    )
    base.visual(
        Box((0.18, 0.035, 0.220)),
        origin=Origin(xyz=(0.0, -0.100, 0.185)),
        material=satin_steel,
        name="trunnion_cheek_neg",
    )
    base.visual(
        Box((0.035, 0.235, 0.070)),
        origin=Origin(xyz=(-0.0925, 0.0, 0.185)),
        material=satin_steel,
        name="rear_cheek_bridge",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.0, 0.0795, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="bearing_ring_pos",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, 0.075, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="bore_shadow_pos",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.0, -0.0795, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="bearing_ring_neg",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, -0.075, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="bore_shadow_neg",
    )
    for y, suffix in ((0.1325, "pos"), (-0.1325, "neg")):
        base.visual(
            Box((0.055, 0.030, 0.040)),
            origin=Origin(xyz=(0.1175, y, 0.220)),
            material=safety_red,
            name=f"lockout_tab_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(
                xyz=(0.1175, y + math.copysign(0.017, y), 0.220),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_oxide,
            name=f"lockout_hole_{suffix}",
        )
    for x, y, suffix in (
        (-0.12, -0.13, "rear_neg"),
        (0.12, -0.13, "front_neg"),
        (-0.12, 0.13, "rear_pos"),
        (0.12, 0.13, "front_pos"),
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, y, 0.039)),
            material=black_oxide,
            name=f"plinth_bolt_{suffix}",
        )

    rotary = model.part("rotary_stage")
    rotary.visual(
        Cylinder(radius=0.056, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ground_steel,
        name="hub_barrel",
    )
    for y, suffix in ((0.074, "pos"), (-0.074, "neg")):
        rotary.visual(
            Cylinder(radius=0.047, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_oxide,
            name=f"hub_cap_{suffix}",
        )
    for y, suffix in ((0.067, "pos"), (-0.067, "neg")):
        rotary.visual(
            Box((0.280, 0.018, 0.105)),
            origin=Origin(xyz=(0.135, y, 0.0)),
            material=satin_steel,
            name=f"side_link_{suffix}",
        )
    for z, suffix in ((0.0625, "top"), (-0.0625, "bottom")):
        rotary.visual(
            Box((0.036, 0.130, 0.020)),
            origin=Origin(xyz=(0.252, 0.0, z)),
            material=satin_steel,
            name=f"rear_cross_tie_{suffix}",
        )
    rotary.visual(
        Box((0.360, 0.012, 0.084)),
        origin=Origin(xyz=(0.450, 0.052, 0.0)),
        material=dark_steel,
        name="guide_side_pos",
    )
    rotary.visual(
        Box((0.360, 0.012, 0.084)),
        origin=Origin(xyz=(0.450, -0.052, 0.0)),
        material=dark_steel,
        name="guide_side_neg",
    )
    rotary.visual(
        Box((0.360, 0.104, 0.012)),
        origin=Origin(xyz=(0.450, 0.0, 0.046)),
        material=dark_steel,
        name="guide_top",
    )
    rotary.visual(
        Box((0.360, 0.104, 0.012)),
        origin=Origin(xyz=(0.450, 0.0, -0.046)),
        material=dark_steel,
        name="guide_bottom",
    )
    for x, suffix in ((0.295, "rear"), (0.605, "front")):
        rotary.visual(
            Box((0.026, 0.124, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=satin_steel,
            name=f"guide_collar_top_{suffix}",
        )
        rotary.visual(
            Box((0.026, 0.124, 0.016)),
            origin=Origin(xyz=(x, 0.0, -0.060)),
            material=satin_steel,
            name=f"guide_collar_bottom_{suffix}",
        )
        for y, ysuffix in ((0.062, "pos"), (-0.062, "neg")):
            rotary.visual(
                Box((0.026, 0.016, 0.120)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=satin_steel,
                name=f"guide_collar_{suffix}_{ysuffix}",
            )
    rotary.visual(
        Box((0.060, 0.034, 0.026)),
        origin=Origin(xyz=(0.590, -0.075, 0.055)),
        material=safety_red,
        name="guide_lockout_tab",
    )
    rotary.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.590, -0.094, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="guide_lockout_hole",
    )

    carriage = model.part("output_carriage")
    carriage.visual(
        Box((0.540, 0.044, 0.044)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=ground_steel,
        name="ram_core",
    )
    carriage.visual(
        Box((0.320, 0.024, 0.036)),
        origin=Origin(xyz=(0.060, 0.034, 0.0)),
        material=bronze,
        name="side_wear_pos",
    )
    carriage.visual(
        Box((0.320, 0.024, 0.036)),
        origin=Origin(xyz=(0.060, -0.034, 0.0)),
        material=bronze,
        name="side_wear_neg",
    )
    carriage.visual(
        Box((0.320, 0.036, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, 0.031)),
        material=bronze,
        name="top_wear",
    )
    carriage.visual(
        Box((0.320, 0.036, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, -0.031)),
        material=bronze,
        name="bottom_wear",
    )
    carriage.visual(
        Box((0.110, 0.120, 0.100)),
        origin=Origin(xyz=(0.425, 0.0, 0.0)),
        material=satin_steel,
        name="front_guide_block",
    )
    carriage.visual(
        Box((0.026, 0.162, 0.140)),
        origin=Origin(xyz=(0.493, 0.0, 0.0)),
        material=dark_steel,
        name="output_faceplate",
    )
    for y, suffix in ((0.067, "pos"), (-0.067, "neg")):
        carriage.visual(
            Box((0.080, 0.024, 0.070)),
            origin=Origin(xyz=(0.545, y, 0.0)),
            material=satin_steel,
            name=f"output_lug_{suffix}",
        )
    carriage.visual(
        Cylinder(radius=0.012, length=0.152),
        origin=Origin(xyz=(0.560, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="output_pin",
    )
    carriage.visual(
        Box((0.078, 0.082, 0.010)),
        origin=Origin(xyz=(0.425, 0.0, 0.055)),
        material=cover_paint,
        name="carriage_access_cover",
    )
    for x, y, suffix in (
        (0.395, 0.030, "rear_pos"),
        (0.455, 0.030, "front_pos"),
        (0.395, -0.030, "rear_neg"),
        (0.455, -0.030, "front_neg"),
    ):
        carriage.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, y, 0.062), rpy=(0.0, 0.0, 0.0)),
            material=black_oxide,
            name=f"carriage_cover_screw_{suffix}",
        )
    carriage.visual(
        Box((0.060, 0.035, 0.035)),
        origin=Origin(xyz=(0.385, 0.0775, 0.044)),
        material=safety_red,
        name="carriage_lockout_tab",
    )
    carriage.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.385, 0.0965, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="carriage_lockout_hole",
    )

    guide_cover = model.part("guide_cover")
    guide_cover.visual(
        Box((0.280, 0.080, 0.014)),
        origin=Origin(xyz=(0.450, 0.0, 0.059)),
        material=cover_paint,
        name="guide_access_plate",
    )
    for x, y, suffix in (
        (0.335, 0.032, "rear_pos"),
        (0.565, 0.032, "front_pos"),
        (0.335, -0.032, "rear_neg"),
        (0.565, -0.032, "front_neg"),
    ):
        guide_cover.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(x, y, 0.069), rpy=(0.0, 0.0, 0.0)),
            material=black_oxide,
            name=f"guide_cover_screw_{suffix}",
        )

    bearing_cover = model.part("bearing_cover")
    bearing_cover.visual(
        Box((0.130, 0.006, 0.100)),
        origin=Origin(xyz=(0.0, 0.1205, 0.185)),
        material=cover_paint,
        name="outer_bearing_plate",
    )
    for x, z, suffix in (
        (-0.045, 0.150, "lower_rear"),
        (0.045, 0.150, "lower_front"),
        (-0.045, 0.220, "upper_rear"),
        (0.045, 0.220, "upper_front"),
    ):
        bearing_cover.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(x, 0.1265, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_oxide,
            name=f"bearing_cover_screw_{suffix}",
        )

    model.articulation(
        "base_to_rotary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-0.35, upper=0.75),
        motion_properties=MotionProperties(damping=0.7, friction=0.08),
    )
    model.articulation(
        "rotary_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary,
        child=carriage,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.22),
        motion_properties=MotionProperties(damping=1.2, friction=0.15),
    )
    model.articulation(
        "rotary_to_cover",
        ArticulationType.FIXED,
        parent=rotary,
        child=guide_cover,
        origin=Origin(),
    )
    model.articulation(
        "base_to_cover",
        ArticulationType.FIXED,
        parent=base,
        child=bearing_cover,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    rotary = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("output_carriage")
    guide_cover = object_model.get_part("guide_cover")
    bearing_cover = object_model.get_part("bearing_cover")
    revolute = object_model.get_articulation("base_to_rotary")
    slide = object_model.get_articulation("rotary_to_carriage")

    ctx.expect_gap(
        base,
        rotary,
        axis="y",
        positive_elem="bearing_ring_pos",
        negative_elem="hub_barrel",
        min_gap=0.004,
        max_gap=0.012,
        name="positive trunnion bearing clears hub end",
    )
    ctx.expect_gap(
        rotary,
        base,
        axis="y",
        positive_elem="hub_barrel",
        negative_elem="bearing_ring_neg",
        min_gap=0.004,
        max_gap=0.012,
        name="negative trunnion bearing clears hub end",
    )
    ctx.expect_contact(
        guide_cover,
        rotary,
        elem_a="guide_access_plate",
        elem_b="guide_top",
        contact_tol=0.0005,
        name="guide access cover seats on top rail",
    )
    ctx.expect_contact(
        bearing_cover,
        base,
        elem_a="outer_bearing_plate",
        elem_b="trunnion_cheek_pos",
        contact_tol=0.0005,
        name="bearing access cover seats on cheek",
    )

    ctx.expect_contact(
        rotary,
        carriage,
        elem_a="guide_top",
        elem_b="top_wear",
        contact_tol=0.0005,
        name="top wear strip bears on guide rail",
    )
    ctx.expect_contact(
        carriage,
        rotary,
        elem_a="bottom_wear",
        elem_b="guide_bottom",
        contact_tol=0.0005,
        name="bottom wear strip bears on guide rail",
    )
    ctx.expect_contact(
        rotary,
        carriage,
        elem_a="guide_side_pos",
        elem_b="side_wear_pos",
        contact_tol=0.0005,
        name="positive side wear strip bears on guide",
    )
    ctx.expect_contact(
        carriage,
        rotary,
        elem_a="side_wear_neg",
        elem_b="guide_side_neg",
        contact_tol=0.0005,
        name="negative side wear strip bears on guide",
    )
    ctx.expect_overlap(
        carriage,
        rotary,
        axes="x",
        elem_a="ram_core",
        elem_b="guide_side_pos",
        min_overlap=0.25,
        name="ram remains engaged in the guide at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.22}):
        ctx.expect_overlap(
            carriage,
            rotary,
            axes="x",
            elem_a="ram_core",
            elem_b="guide_side_pos",
            min_overlap=0.20,
            name="ram remains engaged at full extension",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic stage extends along output axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    level_pos = ctx.part_world_position(carriage)
    with ctx.pose({revolute: 0.45}):
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "positive revolute motion raises the linear stage",
        level_pos is not None
        and raised_pos is not None
        and raised_pos[2] > level_pos[2] + 0.10,
        details=f"level={level_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
