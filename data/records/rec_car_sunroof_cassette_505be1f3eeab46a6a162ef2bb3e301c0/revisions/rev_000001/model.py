from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_sunroof_cassette")

    aluminum = model.material("anodized_aluminum", rgba=(0.34, 0.36, 0.38, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    glass = model.material("smoked_glass", rgba=(0.09, 0.16, 0.20, 0.42))
    ceramic = model.material("ceramic_frit", rgba=(0.0, 0.0, 0.0, 0.92))

    roof_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(1.85, 1.02, 0.075, corner_segments=10),
            [rounded_rect_profile(1.50, 0.76, 0.055, corner_segments=10)],
            0.055,
            center=True,
        ),
        "sunroof_roof_frame",
    )
    glass_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.49, 0.68, 0.030, corner_segments=8),
            0.018,
            center=True,
        ),
        "front_tilt_glass",
    )
    rear_glass_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.82, 0.68, 0.034, corner_segments=8),
            0.018,
            center=True,
        ),
        "rear_slide_glass",
    )

    cassette = model.part("cassette")
    cassette.visual(
        roof_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=black_plastic,
        name="roof_frame",
    )
    cassette.visual(
        Box((0.050, 0.82, 0.038)),
        origin=Origin(xyz=(-0.180, 0.0, 0.071)),
        material=black_plastic,
        name="center_crossbar",
    )
    cassette.visual(
        Box((0.100, 0.92, 0.040)),
        origin=Origin(xyz=(1.180, 0.0, 0.070)),
        material=black_plastic,
        name="rear_stowage_bridge",
    )
    for side, y in (("0", 0.370), ("1", -0.370)):
        cassette.visual(
            Box((1.62, 0.110, 0.023)),
            origin=Origin(xyz=(0.360, y, 0.0635)),
            material=aluminum,
            name=f"rail_bed_{side}",
        )
        cassette.visual(
            Box((1.62, 0.014, 0.041)),
            origin=Origin(xyz=(0.360, y - math.copysign(0.055, y), 0.0785)),
            material=aluminum,
            name=f"rail_inner_lip_{side}",
        )
        cassette.visual(
            Box((1.62, 0.014, 0.041)),
            origin=Origin(xyz=(0.360, y + math.copysign(0.055, y), 0.0785)),
            material=aluminum,
            name=f"rail_outer_lip_{side}",
        )
        cassette.visual(
            Box((1.58, 0.018, 0.010)),
            origin=Origin(xyz=(0.360, y - math.copysign(0.040, y), 0.083)),
            material=rubber,
            name=f"rail_wear_strip_{side}",
        )
    cassette.visual(
        Cylinder(radius=0.008, length=0.86),
        origin=Origin(xyz=(-0.700, 0.0, 0.086), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_pin",
    )
    for side, y in (("0", 0.425), ("1", -0.425)):
        cassette.visual(
            Cylinder(radius=0.025, length=0.075),
            origin=Origin(xyz=(-0.700, y, 0.086), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"fixed_hinge_knuckle_{side}",
        )
        cassette.visual(
            Box((0.070, 0.050, 0.050)),
            origin=Origin(xyz=(-0.700, y, 0.058)),
            material=black_plastic,
            name=f"hinge_boss_{side}",
        )

    front_tilt = model.part("front_tilt")
    front_tilt.visual(
        Cylinder(radius=0.018, length=0.68),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="front_barrel",
    )
    front_tilt.visual(
        Box((0.050, 0.70, 0.018)),
        origin=Origin(xyz=(0.025, 0.0, 0.023)),
        material=black_plastic,
        name="front_carrier",
    )
    for side, y in (("0", 0.350), ("1", -0.350)):
        front_tilt.visual(
            Box((0.470, 0.026, 0.032)),
            origin=Origin(xyz=(0.255, y, 0.022)),
            material=black_plastic,
            name=f"front_side_arm_{side}",
        )
    front_tilt.visual(
        glass_mesh,
        origin=Origin(xyz=(0.245, 0.0, 0.026)),
        material=glass,
        name="front_glass",
    )
    front_tilt.visual(
        Box((0.455, 0.045, 0.003)),
        origin=Origin(xyz=(0.245, 0.317, 0.036)),
        material=ceramic,
        name="front_frit_side_0",
    )
    front_tilt.visual(
        Box((0.455, 0.045, 0.003)),
        origin=Origin(xyz=(0.245, -0.317, 0.036)),
        material=ceramic,
        name="front_frit_side_1",
    )
    front_tilt.visual(
        Box((0.050, 0.68, 0.003)),
        origin=Origin(xyz=(0.025, 0.0, 0.036)),
        material=ceramic,
        name="front_frit_leading",
    )
    front_tilt.visual(
        Box((0.055, 0.68, 0.003)),
        origin=Origin(xyz=(0.465, 0.0, 0.036)),
        material=ceramic,
        name="front_frit_trailing",
    )

    rear_slide = model.part("rear_slide")
    rear_slide.visual(
        rear_glass_mesh,
        origin=Origin(xyz=(0.410, 0.0, 0.026)),
        material=glass,
        name="rear_glass",
    )
    for side, y in (("0", 0.350), ("1", -0.350)):
        rear_slide.visual(
            Box((0.780, 0.030, 0.038)),
            origin=Origin(xyz=(0.410, y, 0.017)),
            material=black_plastic,
            name=f"rear_side_carrier_{side}",
        )
        rear_slide.visual(
            Box((0.780, 0.045, 0.016)),
            origin=Origin(xyz=(0.410, math.copysign(0.370, y), -0.003)),
            material=aluminum,
            name=f"slider_shoe_{side}",
        )
    rear_slide.visual(
        Box((0.790, 0.045, 0.003)),
        origin=Origin(xyz=(0.410, 0.317, 0.036)),
        material=ceramic,
        name="rear_frit_side_0",
    )
    rear_slide.visual(
        Box((0.790, 0.045, 0.003)),
        origin=Origin(xyz=(0.410, -0.317, 0.036)),
        material=ceramic,
        name="rear_frit_side_1",
    )
    rear_slide.visual(
        Box((0.055, 0.68, 0.003)),
        origin=Origin(xyz=(0.0275, 0.0, 0.036)),
        material=ceramic,
        name="rear_frit_leading",
    )
    rear_slide.visual(
        Box((0.060, 0.68, 0.003)),
        origin=Origin(xyz=(0.790, 0.0, 0.036)),
        material=ceramic,
        name="rear_frit_trailing",
    )

    model.articulation(
        "cassette_to_front_tilt",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=front_tilt,
        origin=Origin(xyz=(-0.700, 0.0, 0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.60, lower=0.0, upper=0.24),
    )
    model.articulation(
        "cassette_to_rear_slide",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=rear_slide,
        origin=Origin(xyz=(-0.145, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    front_tilt = object_model.get_part("front_tilt")
    rear_slide = object_model.get_part("rear_slide")
    tilt_joint = object_model.get_articulation("cassette_to_front_tilt")
    slide_joint = object_model.get_articulation("cassette_to_rear_slide")

    ctx.allow_overlap(
        cassette,
        front_tilt,
        elem_a="hinge_pin",
        elem_b="front_barrel",
        reason="The steel hinge pin is intentionally captured inside the front tilt barrel.",
    )
    ctx.expect_within(
        cassette,
        front_tilt,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="front_barrel",
        margin=0.001,
        name="hinge pin is radially inside the tilt barrel",
    )
    ctx.expect_overlap(
        cassette,
        front_tilt,
        axes="y",
        elem_a="hinge_pin",
        elem_b="front_barrel",
        min_overlap=0.60,
        name="hinge pin spans the front tilt barrel",
    )

    ctx.expect_gap(
        rear_slide,
        cassette,
        axis="z",
        positive_elem="slider_shoe_0",
        negative_elem="rail_bed_0",
        max_gap=0.001,
        max_penetration=0.000001,
        name="slide shoe rides on rail bed",
    )
    ctx.expect_within(
        rear_slide,
        cassette,
        axes="y",
        inner_elem="slider_shoe_0",
        outer_elem="rail_bed_0",
        margin=0.002,
        name="slide shoe is centered in rail channel",
    )
    ctx.expect_overlap(
        rear_slide,
        cassette,
        axes="x",
        elem_a="slider_shoe_0",
        elem_b="rail_bed_0",
        min_overlap=0.70,
        name="closed slide remains carried by the rail",
    )

    closed_front = ctx.part_element_world_aabb(front_tilt, elem="front_glass")
    with ctx.pose({tilt_joint: 0.24}):
        tilted_front = ctx.part_element_world_aabb(front_tilt, elem="front_glass")
    ctx.check(
        "front tilt section raises its rear edge",
        closed_front is not None
        and tilted_front is not None
        and tilted_front[1][2] > closed_front[1][2] + 0.06,
        details=f"closed={closed_front}, tilted={tilted_front}",
    )

    closed_slide = ctx.part_world_position(rear_slide)
    with ctx.pose({slide_joint: 0.45}):
        extended_slide = ctx.part_world_position(rear_slide)
        ctx.expect_overlap(
            rear_slide,
            cassette,
            axes="x",
            elem_a="slider_shoe_0",
            elem_b="rail_bed_0",
            min_overlap=0.28,
            name="extended slide retains rail engagement",
        )
    ctx.check(
        "rear slide travels rearward on rails",
        closed_slide is not None
        and extended_slide is not None
        and extended_slide[0] > closed_slide[0] + 0.40,
        details=f"closed={closed_slide}, extended={extended_slide}",
    )

    return ctx.report()


object_model = build_object_model()
