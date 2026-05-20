from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_slide_two_link_arm")

    steel = model.material("dark_blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_metal = model.material("ground_rail_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    carriage_paint = model.material("blue_carriage_paint", rgba=(0.05, 0.18, 0.42, 1.0))
    arm_anodized = model.material("warm_anodized_arm", rgba=(0.82, 0.48, 0.16, 1.0))
    pivot_dark = model.material("black_pivot_bushings", rgba=(0.02, 0.02, 0.025, 1.0))
    pad_rubber = model.material("matte_rubber_pad", rgba=(0.03, 0.03, 0.03, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        Box((0.72, 0.28, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="base_plate",
    )
    guide.visual(
        Box((0.65, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, 0.102, 0.0625)),
        material=rail_metal,
        name="rail_0",
    )
    guide.visual(
        Box((0.65, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.102, 0.0625)),
        material=rail_metal,
        name="rail_1",
    )
    guide.visual(
        Box((0.64, 0.070, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=rail_metal,
        name="center_way",
    )
    guide.visual(
        Box((0.034, 0.25, 0.075)),
        origin=Origin(xyz=(-0.343, 0.0, 0.0775)),
        material=steel,
        name="end_stop_0",
    )
    guide.visual(
        Box((0.034, 0.25, 0.075)),
        origin=Origin(xyz=(0.343, 0.0, 0.0775)),
        material=steel,
        name="end_stop_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=carriage_paint,
        name="saddle",
    )
    carriage.visual(
        Box((0.100, 0.130, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=carriage_paint,
        name="shoulder_block",
    )
    carriage.visual(
        Box((0.075, 0.020, 0.105)),
        origin=Origin(xyz=(0.0, 0.050, 0.150)),
        material=carriage_paint,
        name="shoulder_cheek_0",
    )
    carriage.visual(
        Box((0.075, 0.020, 0.105)),
        origin=Origin(xyz=(0.0, -0.050, 0.150)),
        material=carriage_paint,
        name="shoulder_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.061, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pivot_dark,
        name="shoulder_washer_0",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, -0.061, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pivot_dark,
        name="shoulder_washer_1",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.030, length=0.076),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_anodized,
        name="shoulder_lug",
    )
    upper_link.visual(
        Box((0.280, 0.044, 0.026)),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=arm_anodized,
        name="upper_bar",
    )
    upper_link.visual(
        Box((0.055, 0.095, 0.035)),
        origin=Origin(xyz=(0.288, 0.0, 0.0)),
        material=arm_anodized,
        name="elbow_yoke_root",
    )
    upper_link.visual(
        Box((0.080, 0.020, 0.060)),
        origin=Origin(xyz=(0.340, 0.050, 0.0)),
        material=arm_anodized,
        name="elbow_cheek_0",
    )
    upper_link.visual(
        Box((0.080, 0.020, 0.060)),
        origin=Origin(xyz=(0.340, -0.050, 0.0)),
        material=arm_anodized,
        name="elbow_cheek_1",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=0.027, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_anodized,
        name="elbow_lug",
    )
    distal_link.visual(
        Box((0.170, 0.040, 0.030)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=arm_anodized,
        name="distal_bar",
    )
    distal_link.visual(
        Box((0.035, 0.100, 0.100)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=pad_rubber,
        name="square_pad",
    )
    for y in (-0.032, 0.032):
        for z in (-0.032, 0.032):
            distal_link.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(0.200, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pivot_dark,
                name=f"pad_rivet_{'p' if y > 0 else 'n'}_{'p' if z > 0 else 'n'}",
            )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(-0.220, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.340, effort=120.0, velocity=0.35),
    )
    model.articulation(
        "carriage_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.25, effort=40.0, velocity=1.4),
    )
    model.articulation(
        "upper_link_to_distal_link",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=distal_link,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=18.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_body")
    carriage = object_model.get_part("carriage")
    upper_link = object_model.get_part("upper_link")
    distal_link = object_model.get_part("distal_link")
    slide = object_model.get_articulation("guide_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_link")
    elbow = object_model.get_articulation("upper_link_to_distal_link")

    ctx.check(
        "bench module has the requested joint chain",
        slide.articulation_type == ArticulationType.PRISMATIC
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
        details=f"types={(slide.articulation_type, shoulder.articulation_type, elbow.articulation_type)}",
    )

    ctx.expect_gap(
        carriage,
        guide,
        axis="z",
        positive_elem="saddle",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage saddle rides on guide rail",
    )
    ctx.expect_within(
        carriage,
        guide,
        axes="x",
        inner_elem="saddle",
        outer_elem="rail_0",
        margin=0.003,
        name="carriage remains on the rail at the retracted stop",
    )

    shoulder_upper = shoulder.motion_limits.upper if shoulder.motion_limits else 0.8
    elbow_upper = elbow.motion_limits.upper if elbow.motion_limits else 0.8
    slide_upper = slide.motion_limits.upper if slide.motion_limits else 0.3

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide_upper}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            guide,
            axes="x",
            inner_elem="saddle",
            outer_elem="rail_0",
            margin=0.003,
            name="carriage remains on the rail at full travel",
        )
        ctx.expect_gap(
            carriage,
            guide,
            axis="z",
            positive_elem="saddle",
            negative_elem="rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage keeps rail contact at full travel",
        )
    ctx.check(
        "slide translates the carriage along the bench guide",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.30,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    ctx.expect_gap(
        carriage,
        upper_link,
        axis="y",
        positive_elem="shoulder_cheek_0",
        negative_elem="shoulder_lug",
        max_gap=0.003,
        max_penetration=0.0,
        name="shoulder lug is captured by positive clevis cheek",
    )
    ctx.expect_gap(
        upper_link,
        carriage,
        axis="y",
        positive_elem="shoulder_lug",
        negative_elem="shoulder_cheek_1",
        max_gap=0.003,
        max_penetration=0.0,
        name="shoulder lug is captured by negative clevis cheek",
    )
    ctx.expect_gap(
        upper_link,
        distal_link,
        axis="y",
        positive_elem="elbow_cheek_0",
        negative_elem="elbow_lug",
        max_gap=0.001,
        max_penetration=0.0,
        name="elbow lug sits between upper link yoke cheeks",
    )

    rest_elbow_pos = ctx.part_world_position(distal_link)
    with ctx.pose({shoulder: min(0.80, shoulder_upper)}):
        raised_elbow_pos = ctx.part_world_position(distal_link)
    ctx.check(
        "positive shoulder rotation raises the elbow end of the arm",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.18,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    def _element_center(part, elem_name):
        bounds = ctx.part_element_world_aabb(part, elem=elem_name)
        if bounds is None:
            return None
        mn, mx = bounds
        return (
            (mn[0] + mx[0]) * 0.5,
            (mn[1] + mx[1]) * 0.5,
            (mn[2] + mx[2]) * 0.5,
        )

    rest_pad_center = _element_center(distal_link, "square_pad")
    with ctx.pose({elbow: min(0.85, elbow_upper)}):
        folded_pad_center = _element_center(distal_link, "square_pad")
    ctx.check(
        "positive elbow rotation lifts the square pad relative to the elbow",
        rest_pad_center is not None
        and folded_pad_center is not None
        and folded_pad_center[2] > rest_pad_center[2] + 0.10,
        details=f"rest={rest_pad_center}, folded={folded_pad_center}",
    )

    return ctx.report()


object_model = build_object_model()
