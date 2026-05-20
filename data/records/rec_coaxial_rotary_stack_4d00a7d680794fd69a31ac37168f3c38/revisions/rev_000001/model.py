from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SEGMENTS = 96


def _lathe_mesh(profile: list[tuple[float, float]], name: str):
    """Build a managed lathed mesh from a radius/z section profile."""
    return mesh_from_geometry(LatheGeometry(profile, segments=SEGMENTS), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_ring_turntable")

    cast_black = model.material("cast_black", rgba=(0.055, 0.060, 0.065, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.015, 0.016, 0.018, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.43, 0.47, 0.51, 1.0))
    fixture_orange = model.material("fixture_orange", rgba=(0.92, 0.39, 0.12, 1.0))

    # Each section below is authored as a single connected body of revolution.
    # The small inner feet are the bearing lands; the overhanging outer skirts
    # leave visible shadow breaks while still giving each rotating stage a real
    # coaxial support path.
    base_profile = [
        (0.000, 0.000),
        (0.245, 0.000),
        (0.245, 0.026),
        (0.180, 0.026),
        (0.180, 0.034),
        (0.124, 0.034),
        (0.124, 0.040),
        (0.000, 0.040),
    ]
    platter_profile = [
        (0.058, 0.000),
        (0.225, 0.000),
        (0.225, 0.014),
        (0.216, 0.020),
        (0.150, 0.020),
        (0.150, 0.028),
        (0.105, 0.028),
        (0.105, 0.044),
        (0.058, 0.044),
    ]
    ring_profile = [
        (0.062, 0.000),
        (0.105, 0.000),
        (0.105, 0.006),
        (0.168, 0.006),
        (0.168, 0.050),
        (0.160, 0.060),
        (0.112, 0.060),
        (0.112, 0.072),
        (0.062, 0.072),
    ]
    cup_profile = [
        (0.000, 0.000),
        (0.072, 0.000),
        (0.072, 0.006),
        (0.083, 0.006),
        (0.083, 0.056),
        (0.078, 0.068),
        (0.058, 0.068),
        (0.058, 0.018),
        (0.000, 0.018),
    ]

    base = model.part("base")
    base.visual(
        _lathe_mesh(base_profile, "base_housing"),
        material=cast_black,
        name="base_housing",
    )
    base.visual(
        _lathe_mesh([(0.226, 0.000), (0.244, 0.000), (0.244, 0.003), (0.226, 0.003)], "base_race_dark"),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=shadow_black,
        name="base_race_dark",
    )

    lower_platter = model.part("lower_platter")
    lower_platter.visual(
        _lathe_mesh(platter_profile, "platter_housing"),
        material=brushed_aluminum,
        name="platter_housing",
    )
    lower_platter.visual(
        _lathe_mesh([(0.106, 0.000), (0.150, 0.000), (0.150, 0.003), (0.106, 0.003)], "platter_shadow_land"),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=shadow_black,
        name="platter_shadow_land",
    )
    lower_platter.visual(
        Box((0.066, 0.018, 0.006)),
        origin=Origin(xyz=(0.184, 0.0, 0.022)),
        material=shadow_black,
        name="platter_index",
    )

    intermediate_ring = model.part("intermediate_ring")
    intermediate_ring.visual(
        _lathe_mesh(ring_profile, "ring_housing"),
        material=bearing_steel,
        name="ring_housing",
    )
    intermediate_ring.visual(
        _lathe_mesh([(0.112, 0.000), (0.160, 0.000), (0.160, 0.003), (0.112, 0.003)], "ring_shadow_land"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=shadow_black,
        name="ring_shadow_land",
    )
    intermediate_ring.visual(
        Box((0.050, 0.016, 0.007)),
        origin=Origin(xyz=(0.136, 0.0, 0.0625)),
        material=shadow_black,
        name="ring_index",
    )

    fixture_cup = model.part("fixture_cup")
    fixture_cup.visual(
        _lathe_mesh(cup_profile, "cup_shell"),
        material=fixture_orange,
        name="cup_shell",
    )
    fixture_cup.visual(
        _lathe_mesh([(0.060, 0.000), (0.077, 0.000), (0.077, 0.003), (0.060, 0.003)], "cup_inner_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=shadow_black,
        name="cup_inner_lip",
    )
    fixture_cup.visual(
        Box((0.020, 0.030, 0.026)),
        origin=Origin(xyz=(0.090, 0.0, 0.041)),
        material=shadow_black,
        name="cup_index_lug",
    )

    base_to_platter = model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_platter,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    platter_to_ring = model.articulation(
        "platter_to_ring",
        ArticulationType.REVOLUTE,
        parent=lower_platter,
        child=intermediate_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=2.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    ring_to_cup = model.articulation(
        "ring_to_cup",
        ArticulationType.REVOLUTE,
        parent=intermediate_ring,
        child=fixture_cup,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    # Names are referenced in run_tests; binding here makes accidental deletion
    # obvious during authoring without changing the model structure.
    assert base_to_platter and platter_to_ring and ring_to_cup
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_platter = object_model.get_part("lower_platter")
    intermediate_ring = object_model.get_part("intermediate_ring")
    fixture_cup = object_model.get_part("fixture_cup")
    base_to_platter = object_model.get_articulation("base_to_platter")
    platter_to_ring = object_model.get_articulation("platter_to_ring")
    ring_to_cup = object_model.get_articulation("ring_to_cup")

    ctx.check(
        "three independent coaxial revolutes",
        len(object_model.articulations) == 3
        and all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis or ()) == (0.0, 0.0, 1.0)
            for joint in (base_to_platter, platter_to_ring, ring_to_cup)
        ),
        details="The platter, intermediate ring, and fixture cup should each have a vertical revolute axis.",
    )
    ctx.expect_origin_distance(
        base,
        lower_platter,
        axes="xy",
        max_dist=0.001,
        name="platter centerline matches base",
    )
    ctx.expect_origin_distance(
        lower_platter,
        intermediate_ring,
        axes="xy",
        max_dist=0.001,
        name="ring centerline matches platter",
    )
    ctx.expect_origin_distance(
        intermediate_ring,
        fixture_cup,
        axes="xy",
        max_dist=0.001,
        name="cup centerline matches ring",
    )
    ctx.expect_contact(
        lower_platter,
        base,
        elem_a="platter_housing",
        elem_b="base_housing",
        contact_tol=0.0015,
        name="platter rides on base bearing land",
    )
    ctx.expect_contact(
        intermediate_ring,
        lower_platter,
        elem_a="ring_housing",
        elem_b="platter_housing",
        contact_tol=0.0015,
        name="intermediate ring rides on platter land",
    )
    ctx.expect_contact(
        fixture_cup,
        intermediate_ring,
        elem_a="cup_shell",
        elem_b="ring_housing",
        contact_tol=0.0015,
        name="fixture cup rides on ring land",
    )

    rest_cup_position = ctx.part_world_position(fixture_cup)
    with ctx.pose({base_to_platter: 0.8, platter_to_ring: -0.6, ring_to_cup: 1.1}):
        turned_cup_position = ctx.part_world_position(fixture_cup)
        ctx.expect_origin_distance(
            base,
            fixture_cup,
            axes="xy",
            max_dist=0.001,
            name="rotated stack stays on common vertical centerline",
        )

    ctx.check(
        "coaxial rotations preserve cup height",
        rest_cup_position is not None
        and turned_cup_position is not None
        and abs(rest_cup_position[2] - turned_cup_position[2]) < 0.001,
        details=f"rest={rest_cup_position}, turned={turned_cup_position}",
    )

    return ctx.report()


object_model = build_object_model()
