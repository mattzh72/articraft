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
    model = ArticulatedObject(name="under_slung_slide_link_slide")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_metal = model.material("brushed_rail_steel", rgba=(0.60, 0.62, 0.63, 1.0))
    carriage_paint = model.material("safety_blue_carriage", rgba=(0.05, 0.18, 0.55, 1.0))
    elbow_paint = model.material("orange_elbow_frame", rgba=(0.95, 0.36, 0.08, 1.0))
    nose_paint = model.material("graphite_nose", rgba=(0.12, 0.12, 0.13, 1.0))
    pin_steel = model.material("polished_pin_steel", rgba=(0.78, 0.77, 0.72, 1.0))
    rubber = model.material("black_rubber_bumper", rgba=(0.02, 0.02, 0.018, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((1.20, 0.24, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=anodized,
        name="main_beam",
    )
    top_support.visual(
        Box((1.15, 0.150, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.4925)),
        material=rail_metal,
        name="under_track",
    )
    for y, suffix in ((0.080, "0"), (-0.080, "1")):
        top_support.visual(
            Box((1.15, 0.045, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.530)),
            material=rail_metal,
            name=f"guide_rib_{suffix}",
        )
    for x, suffix in ((-0.610, "0"), (0.610, "1")):
        top_support.visual(
            Box((0.030, 0.250, 0.150)),
            origin=Origin(xyz=(x, 0.0, 0.545)),
            material=anodized,
            name=f"end_stop_{suffix}",
        )
    top_support.visual(
        Box((0.42, 0.20, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=anodized,
        name="ceiling_mount_plate",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.160, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=carriage_paint,
        name="slider_saddle",
    )
    carriage.visual(
        Box((0.160, 0.085, 0.135)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=carriage_paint,
        name="hanger_web",
    )
    carriage.visual(
        Box((0.165, 0.180, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        material=carriage_paint,
        name="yoke_bridge",
    )
    for y, suffix in ((0.0675, "0"), (-0.0675, "1")):
        carriage.visual(
            Box((0.075, 0.035, 0.120)),
            origin=Origin(xyz=(0.0, y, -0.248)),
            material=carriage_paint,
            name=f"yoke_cheek_{suffix}",
        )
    carriage.visual(
        Cylinder(radius=0.018, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.248), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="hinge_pin",
    )

    elbow_frame = model.part("elbow_frame")
    elbow_frame.visual(
        Box((0.062, 0.072, 0.066)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=elbow_paint,
        name="hinge_boss",
    )
    elbow_frame.visual(
        Cylinder(radius=0.026, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=elbow_paint,
        name="boss_rounding",
    )
    elbow_frame.visual(
        Box((0.055, 0.070, 0.320)),
        origin=Origin(xyz=(0.0, 0.0, -0.193)),
        material=elbow_paint,
        name="drop_leg",
    )
    elbow_frame.visual(
        Box((0.100, 0.110, 0.055)),
        origin=Origin(xyz=(0.030, 0.0, -0.345)),
        material=elbow_paint,
        name="elbow_corner",
    )
    elbow_frame.visual(
        Box((0.480, 0.030, 0.055)),
        origin=Origin(xyz=(0.245, 0.050, -0.345)),
        material=elbow_paint,
        name="side_rail_0",
    )
    elbow_frame.visual(
        Box((0.480, 0.030, 0.055)),
        origin=Origin(xyz=(0.245, -0.050, -0.345)),
        material=elbow_paint,
        name="side_rail_1",
    )
    elbow_frame.visual(
        Box((0.105, 0.116, 0.020)),
        origin=Origin(xyz=(0.235, 0.0, -0.382)),
        material=elbow_paint,
        name="lower_cross_tie",
    )
    elbow_frame.visual(
        Box((0.040, 0.030, 0.065)),
        origin=Origin(xyz=(0.485, 0.052, -0.345)),
        material=elbow_paint,
        name="nose_end_gate",
    )
    elbow_frame.visual(
        Box((0.040, 0.030, 0.065)),
        origin=Origin(xyz=(0.485, -0.052, -0.345)),
        material=elbow_paint,
        name="nose_end_gate_1",
    )

    nose = model.part("nose")
    nose.visual(
        Box((0.420, 0.058, 0.038)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=nose_paint,
        name="sliding_tongue",
    )
    nose.visual(
        Box((0.300, 0.006, 0.032)),
        origin=Origin(xyz=(0.040, 0.032, 0.0)),
        material=rail_metal,
        name="wear_strip_0",
    )
    nose.visual(
        Box((0.300, 0.006, 0.032)),
        origin=Origin(xyz=(0.040, -0.032, 0.0)),
        material=rail_metal,
        name="wear_strip_1",
    )
    nose.visual(
        Box((0.085, 0.078, 0.060)),
        origin=Origin(xyz=(0.3525, 0.0, 0.0)),
        material=nose_paint,
        name="nose_block",
    )
    nose.visual(
        Box((0.030, 0.082, 0.064)),
        origin=Origin(xyz=(0.410, 0.0, 0.0)),
        material=rubber,
        name="nose_bumper",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(-0.250, 0.0, 0.485)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.500),
    )
    model.articulation(
        "carriage_to_elbow",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow_frame,
        origin=Origin(xyz=(0.0, 0.0, -0.248)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.85, upper=0.55),
    )
    model.articulation(
        "elbow_to_nose",
        ArticulationType.PRISMATIC,
        parent=elbow_frame,
        child=nose,
        origin=Origin(xyz=(0.220, 0.0, -0.345)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.25, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    elbow_frame = object_model.get_part("elbow_frame")
    nose = object_model.get_part("nose")

    support_slide = object_model.get_articulation("support_to_carriage")
    elbow_hinge = object_model.get_articulation("carriage_to_elbow")
    nose_slide = object_model.get_articulation("elbow_to_nose")

    ctx.allow_overlap(
        carriage,
        elbow_frame,
        elem_a="hinge_pin",
        elem_b="hinge_boss",
        reason="The steel hinge pin is intentionally captured through the elbow boss bore.",
    )
    ctx.allow_overlap(
        carriage,
        elbow_frame,
        elem_a="hinge_pin",
        elem_b="boss_rounding",
        reason="The hinge pin passes through the rounded boss as the revolute bearing.",
    )

    ctx.expect_gap(
        top_support,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="under_track",
        negative_elem="slider_saddle",
        name="carriage saddle bears on underside track",
    )
    ctx.expect_overlap(
        top_support,
        carriage,
        axes="xy",
        min_overlap=0.10,
        elem_a="under_track",
        elem_b="slider_saddle",
        name="carriage remains captured under the top support",
    )
    ctx.expect_overlap(
        carriage,
        elbow_frame,
        axes="xyz",
        min_overlap=0.010,
        elem_a="hinge_pin",
        elem_b="hinge_boss",
        name="hinge pin passes through elbow boss",
    )
    ctx.expect_gap(
        elbow_frame,
        nose,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="side_rail_0",
        negative_elem="wear_strip_0",
        name="nose guided against positive side rail",
    )
    ctx.expect_overlap(
        nose,
        elbow_frame,
        axes="x",
        min_overlap=0.20,
        elem_a="sliding_tongue",
        elem_b="side_rail_0",
        name="nose tongue retained in distal slide at rest",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({support_slide: 0.50}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            top_support,
            carriage,
            axes="xy",
            min_overlap=0.10,
            elem_a="under_track",
            elem_b="slider_saddle",
            name="carriage still captured at slide limit",
        )
    ctx.check(
        "carriage translates along the top rail",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.45,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_nose_aabb = ctx.part_world_aabb(nose)
    with ctx.pose({elbow_hinge: 0.50}):
        raised_nose_aabb = ctx.part_world_aabb(nose)
    ctx.check(
        "elbow hinge raises distal slide",
        rest_nose_aabb is not None
        and raised_nose_aabb is not None
        and raised_nose_aabb[0][2] > rest_nose_aabb[0][2] + 0.10,
        details=f"rest={rest_nose_aabb}, raised={raised_nose_aabb}",
    )

    rest_nose_pos = ctx.part_world_position(nose)
    with ctx.pose({nose_slide: 0.18}):
        extended_nose_pos = ctx.part_world_position(nose)
        ctx.expect_overlap(
            nose,
            elbow_frame,
            axes="x",
            min_overlap=0.08,
            elem_a="sliding_tongue",
            elem_b="side_rail_0",
            name="nose tongue retained at distal slide limit",
        )
    ctx.check(
        "nose translates outward from elbow frame",
        rest_nose_pos is not None
        and extended_nose_pos is not None
        and extended_nose_pos[0] > rest_nose_pos[0] + 0.16,
        details=f"rest={rest_nose_pos}, extended={extended_nose_pos}",
    )

    return ctx.report()


object_model = build_object_model()
