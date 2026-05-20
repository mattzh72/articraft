from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_three_stage_slide")

    dark_steel = Material("dark_burnished_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = Material("satin_sliding_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    pale_face = Material("plain_light_front_faces", rgba=(0.78, 0.80, 0.78, 1.0))
    bearing = Material("black_acetal_bearing_strips", rgba=(0.015, 0.015, 0.018, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.50, 0.36, 0.020)),
        origin=Origin(xyz=(0.23, 0.0, 0.010)),
        material=dark_steel,
        name="ground_plinth",
    )
    body.visual(
        Box((0.46, 0.32, 0.040)),
        origin=Origin(xyz=(0.23, 0.0, 0.040)),
        material=dark_steel,
        name="body_floor",
    )
    body.visual(
        Box((0.46, 0.035, 0.140)),
        origin=Origin(xyz=(0.23, 0.1425, 0.130)),
        material=dark_steel,
        name="body_side_0",
    )
    body.visual(
        Box((0.46, 0.035, 0.140)),
        origin=Origin(xyz=(0.23, -0.1425, 0.130)),
        material=dark_steel,
        name="body_side_1",
    )
    body.visual(
        Box((0.46, 0.055, 0.025)),
        origin=Origin(xyz=(0.23, 0.0975, 0.1875)),
        material=dark_steel,
        name="body_top_lip_0",
    )
    body.visual(
        Box((0.46, 0.055, 0.025)),
        origin=Origin(xyz=(0.23, -0.0975, 0.1875)),
        material=dark_steel,
        name="body_top_lip_1",
    )
    body.visual(
        Box((0.035, 0.32, 0.180)),
        origin=Origin(xyz=(0.0175, 0.0, 0.110)),
        material=dark_steel,
        name="rear_stop",
    )
    body.visual(
        Box((0.42, 0.025, 0.013)),
        origin=Origin(xyz=(0.25, 0.050, 0.0665)),
        material=bearing,
        name="body_bearing_0",
    )
    body.visual(
        Box((0.42, 0.025, 0.013)),
        origin=Origin(xyz=(0.25, -0.050, 0.0665)),
        material=bearing,
        name="body_bearing_1",
    )

    outer_stage = model.part("outer_stage")
    outer_stage.visual(
        Box((0.340, 0.180, 0.018)),
        origin=Origin(xyz=(-0.170, 0.0, -0.036)),
        material=satin_steel,
        name="outer_bottom_web",
    )
    outer_stage.visual(
        Box((0.340, 0.018, 0.090)),
        origin=Origin(xyz=(-0.170, 0.081, 0.0)),
        material=satin_steel,
        name="outer_side_0",
    )
    outer_stage.visual(
        Box((0.340, 0.018, 0.090)),
        origin=Origin(xyz=(-0.170, -0.081, 0.0)),
        material=satin_steel,
        name="outer_side_1",
    )
    outer_stage.visual(
        Box((0.340, 0.052, 0.016)),
        origin=Origin(xyz=(-0.170, 0.064, 0.037)),
        material=satin_steel,
        name="outer_top_lip_0",
    )
    outer_stage.visual(
        Box((0.340, 0.052, 0.016)),
        origin=Origin(xyz=(-0.170, -0.064, 0.037)),
        material=satin_steel,
        name="outer_top_lip_1",
    )
    outer_stage.visual(
        Box((0.310, 0.018, 0.003)),
        origin=Origin(xyz=(-0.155, 0.024, -0.0255)),
        material=bearing,
        name="outer_inner_bearing_0",
    )
    outer_stage.visual(
        Box((0.310, 0.018, 0.003)),
        origin=Origin(xyz=(-0.155, -0.024, -0.0255)),
        material=bearing,
        name="outer_inner_bearing_1",
    )
    # The visible front is a plain rectangular service face with a covered
    # center slot for the next nested stage.
    outer_stage.visual(
        Box((0.024, 0.210, 0.028)),
        origin=Origin(xyz=(0.012, 0.0, 0.041)),
        material=pale_face,
        name="outer_face_top",
    )
    outer_stage.visual(
        Box((0.024, 0.210, 0.028)),
        origin=Origin(xyz=(0.012, 0.0, -0.041)),
        material=pale_face,
        name="outer_face_bottom",
    )
    outer_stage.visual(
        Box((0.024, 0.030, 0.110)),
        origin=Origin(xyz=(0.012, 0.090, 0.0)),
        material=pale_face,
        name="outer_face_side_0",
    )
    outer_stage.visual(
        Box((0.024, 0.030, 0.110)),
        origin=Origin(xyz=(0.012, -0.090, 0.0)),
        material=pale_face,
        name="outer_face_side_1",
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        Box((0.340, 0.065, 0.048)),
        origin=Origin(xyz=(-0.170, 0.0, 0.0)),
        material=satin_steel,
        name="inner_rail",
    )
    inner_stage.visual(
        Box((0.026, 0.135, 0.082)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=pale_face,
        name="inner_front_face",
    )

    model.articulation(
        "body_to_outer_stage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=outer_stage,
        origin=Origin(xyz=(0.460, 0.0, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.220),
    )
    model.articulation(
        "outer_stage_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    outer_stage = object_model.get_part("outer_stage")
    inner_stage = object_model.get_part("inner_stage")
    body_to_outer = object_model.get_articulation("body_to_outer_stage")
    outer_to_inner = object_model.get_articulation("outer_stage_to_inner_stage")

    ctx.check(
        "slide uses two serial prismatic joints",
        body_to_outer.articulation_type == ArticulationType.PRISMATIC
        and outer_to_inner.articulation_type == ArticulationType.PRISMATIC
        and body_to_outer.parent == "body"
        and body_to_outer.child == "outer_stage"
        and outer_to_inner.parent == "outer_stage"
        and outer_to_inner.child == "inner_stage",
    )

    ctx.expect_contact(
        outer_stage,
        body,
        elem_a="outer_bottom_web",
        elem_b="body_bearing_0",
        name="outer stage rides on body bearing",
    )
    ctx.expect_contact(
        inner_stage,
        outer_stage,
        elem_a="inner_rail",
        elem_b="outer_inner_bearing_0",
        name="inner stage rides on outer bearing",
    )
    ctx.expect_overlap(
        outer_stage,
        body,
        axes="x",
        elem_a="outer_bottom_web",
        elem_b="body_bearing_0",
        min_overlap=0.30,
        name="outer stage is deeply retained when closed",
    )
    ctx.expect_overlap(
        inner_stage,
        outer_stage,
        axes="x",
        elem_a="inner_rail",
        elem_b="outer_inner_bearing_0",
        min_overlap=0.28,
        name="inner stage is deeply retained when closed",
    )

    rest_outer = ctx.part_world_position(outer_stage)
    rest_inner = ctx.part_world_position(inner_stage)
    with ctx.pose({body_to_outer: 0.220, outer_to_inner: 0.180}):
        extended_outer = ctx.part_world_position(outer_stage)
        extended_inner = ctx.part_world_position(inner_stage)
        ctx.expect_contact(
            outer_stage,
            body,
            elem_a="outer_bottom_web",
            elem_b="body_bearing_0",
            name="outer stage remains bearing-supported when extended",
        )
        ctx.expect_contact(
            inner_stage,
            outer_stage,
            elem_a="inner_rail",
            elem_b="outer_inner_bearing_0",
            name="inner stage remains bearing-supported when extended",
        )
        ctx.expect_overlap(
            outer_stage,
            body,
            axes="x",
            elem_a="outer_bottom_web",
            elem_b="body_bearing_0",
            min_overlap=0.10,
            name="outer stage retains insertion at full travel",
        )
        ctx.expect_overlap(
            inner_stage,
            outer_stage,
            axes="x",
            elem_a="inner_rail",
            elem_b="outer_inner_bearing_0",
            min_overlap=0.10,
            name="inner stage retains insertion at full travel",
        )

    ctx.check(
        "positive travel extends both front faces outward",
        rest_outer is not None
        and rest_inner is not None
        and extended_outer is not None
        and extended_inner is not None
        and extended_outer[0] > rest_outer[0] + 0.20
        and extended_inner[0] > rest_inner[0] + 0.38,
        details=f"rest_outer={rest_outer}, extended_outer={extended_outer}, "
        f"rest_inner={rest_inner}, extended_inner={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
