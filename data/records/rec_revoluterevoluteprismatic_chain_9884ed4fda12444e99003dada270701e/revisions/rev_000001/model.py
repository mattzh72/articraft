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
    model = ArticulatedObject(name="compact_service_elbow_arm")

    base_mat = model.material("mat_cast_dark_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    joint_mat = model.material("mat_bearing_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    link_mat = model.material("mat_service_orange", rgba=(0.95, 0.43, 0.10, 1.0))
    rail_mat = model.material("mat_worn_orange", rgba=(0.82, 0.31, 0.07, 1.0))
    slider_mat = model.material("mat_brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    pad_mat = model.material("mat_rubber_face", rgba=(0.015, 0.016, 0.017, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.36, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=base_mat,
        name="ground_plate",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.23),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=base_mat,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3025)),
        material=base_mat,
        name="top_cap",
    )
    for side, y in enumerate((-0.08, 0.08)):
        base.visual(
            Box((0.12, 0.032, 0.24)),
            origin=Origin(xyz=(0.0, y, 0.39)),
            material=base_mat,
            name=f"shoulder_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.065, length=0.034),
            origin=Origin(xyz=(0.0, y, 0.42), rpy=(pi / 2.0, 0.0, 0.0)),
            material=joint_mat,
            name=f"shoulder_bearing_{side}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.055, length=0.126),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="shoulder_boss",
    )
    upper_link.visual(
        Box((0.070, 0.070, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=link_mat,
        name="upright_spine",
    )
    for side, y in enumerate((-0.065, 0.065)):
        upper_link.visual(
            Box((0.13, 0.026, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.66)),
            material=link_mat,
            name=f"elbow_cheek_{side}",
        )
        upper_link.visual(
            Cylinder(radius=0.055, length=0.024),
            origin=Origin(xyz=(0.0, y, 0.66), rpy=(pi / 2.0, 0.0, 0.0)),
            material=joint_mat,
            name=f"elbow_bearing_{side}",
        )
        upper_link.visual(
            Box((0.070, 0.035, 0.050)),
            origin=Origin(xyz=(0.0, y * 0.69, 0.585)),
            material=link_mat,
            name=f"elbow_web_{side}",
        )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=0.045, length=0.104),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_mat,
        name="elbow_boss",
    )
    for side, y in enumerate((-0.0395, 0.0395)):
        distal_link.visual(
            Box((0.49, 0.026, 0.079)),
            origin=Origin(xyz=(0.28, y, 0.0)),
            material=rail_mat,
            name=f"side_rail_{side}",
        )
    for side, z in enumerate((-0.0395, 0.0395)):
        distal_link.visual(
            Box((0.49, 0.105, 0.026)),
            origin=Origin(xyz=(0.28, 0.0, z)),
            material=rail_mat,
            name=f"cover_rail_{side}",
        )
    for side, y in enumerate((-0.0395, 0.0395)):
        distal_link.visual(
            Box((0.055, 0.026, 0.105)),
            origin=Origin(xyz=(0.535, y, 0.0)),
            material=joint_mat,
            name=f"mouth_side_{side}",
        )
    for side, z in enumerate((-0.0395, 0.0395)):
        distal_link.visual(
            Box((0.055, 0.105, 0.026)),
            origin=Origin(xyz=(0.535, 0.0, z)),
            material=joint_mat,
            name=f"mouth_cover_{side}",
        )

    tip_slider = model.part("tip_slider")
    tip_slider.visual(
        Box((0.382, 0.053, 0.053)),
        origin=Origin(xyz=(-0.109, 0.0, 0.0)),
        material=slider_mat,
        name="slider_tongue",
    )
    tip_slider.visual(
        Box((0.026, 0.085, 0.085)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=slider_mat,
        name="face_plate",
    )
    tip_slider.visual(
        Box((0.006, 0.075, 0.075)),
        origin=Origin(xyz=(0.111, 0.0, 0.0)),
        material=pad_mat,
        name="rubber_face",
    )

    model.articulation(
        "base_to_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.55, upper=1.05),
    )
    model.articulation(
        "upper_to_distal",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=distal_link,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-1.35, upper=0.90),
    )
    model.articulation(
        "distal_to_tip",
        ArticulationType.PRISMATIC,
        parent=distal_link,
        child=tip_slider,
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=0.16),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_link")
    distal = object_model.get_part("distal_link")
    slider = object_model.get_part("tip_slider")
    shoulder = object_model.get_articulation("base_to_upper")
    elbow = object_model.get_articulation("upper_to_distal")
    slide = object_model.get_articulation("distal_to_tip")

    ctx.check(
        "service arm has two revolute joints and one tip slider",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"types={[shoulder.articulation_type, elbow.articulation_type, slide.articulation_type]}",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is grounded",
        base_aabb is not None and abs(base_aabb[0][2]) < 1e-6,
        details=f"base_aabb={base_aabb}",
    )

    upper_spine = ctx.part_element_world_aabb(upper, elem="upright_spine")
    distal_rail = ctx.part_element_world_aabb(distal, elem="side_rail_0")
    ctx.check(
        "distal link is shorter than upper link",
        upper_spine is not None
        and distal_rail is not None
        and (upper_spine[1][2] - upper_spine[0][2]) > (distal_rail[1][0] - distal_rail[0][0]) + 0.06,
        details=f"upper_spine={upper_spine}, distal_rail={distal_rail}",
    )

    slider_tongue = ctx.part_element_world_aabb(slider, elem="slider_tongue")
    distal_outer = ctx.part_world_aabb(distal)
    ctx.check(
        "tip slider stays visibly smaller than link cross section",
        slider_tongue is not None
        and distal_outer is not None
        and (slider_tongue[1][1] - slider_tongue[0][1]) < 0.60 * (distal_outer[1][1] - distal_outer[0][1])
        and (slider_tongue[1][2] - slider_tongue[0][2]) < 0.60 * (distal_outer[1][2] - distal_outer[0][2]),
        details=f"slider_tongue={slider_tongue}, distal_outer={distal_outer}",
    )

    ctx.expect_within(
        slider,
        distal,
        axes="yz",
        inner_elem="slider_tongue",
        margin=0.001,
        name="slider tongue stays centered in the distal link throat",
    )
    ctx.expect_overlap(
        slider,
        distal,
        axes="x",
        elem_a="slider_tongue",
        min_overlap=0.24,
        name="retracted slider keeps long insertion in distal guide",
    )

    rest_aabb = ctx.part_element_world_aabb(slider, elem="rubber_face")
    with ctx.pose({slide: 0.16}):
        extended_aabb = ctx.part_element_world_aabb(slider, elem="rubber_face")
        ctx.expect_within(
            slider,
            distal,
            axes="yz",
            inner_elem="slider_tongue",
            margin=0.001,
            name="extended slider remains centered in distal guide",
        )
        ctx.expect_overlap(
            slider,
            distal,
            axes="x",
            elem_a="slider_tongue",
            min_overlap=0.13,
            name="extended slider remains retained in distal guide",
        )
    ctx.check(
        "tip face extends outward along the distal link",
        rest_aabb is not None and extended_aabb is not None and extended_aabb[1][0] > rest_aabb[1][0] + 0.145,
        details=f"rest={rest_aabb}, extended={extended_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
