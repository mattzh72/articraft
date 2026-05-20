from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_three_stage_axis")

    body_mat = model.material("dark_cast_body", color=(0.08, 0.09, 0.10, 1.0))
    pad_mat = model.material("black_slide_pad", color=(0.015, 0.016, 0.018, 1.0))
    stage_0_mat = model.material("blue_gray_stage", color=(0.36, 0.43, 0.50, 1.0))
    stage_1_mat = model.material("warm_steel_stage", color=(0.60, 0.62, 0.62, 1.0))
    stage_2_mat = model.material("light_service_stage", color=(0.76, 0.78, 0.77, 1.0))
    face_mat = model.material("plain_front_faces", color=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.96, 0.36, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=body_mat,
        name="grounded_body",
    )
    body.visual(
        Box((0.78, 0.24, 0.025)),
        origin=Origin(xyz=(-0.02, 0.0, 0.0925)),
        material=pad_mat,
        name="fixed_slide_face",
    )

    stage_0 = model.part("stage_0")
    stage_0.visual(
        Box((0.620, 0.200, 0.045)),
        origin=Origin(xyz=(0.310, 0.0, 0.0225)),
        material=stage_0_mat,
        name="slide_block",
    )
    stage_0.visual(
        Box((0.014, 0.200, 0.045)),
        origin=Origin(xyz=(0.627, 0.0, 0.0225)),
        material=face_mat,
        name="end_face",
    )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        Box((0.500, 0.160, 0.040)),
        origin=Origin(xyz=(0.250, 0.0, 0.020)),
        material=stage_1_mat,
        name="slide_block",
    )
    stage_1.visual(
        Box((0.012, 0.160, 0.040)),
        origin=Origin(xyz=(0.506, 0.0, 0.020)),
        material=face_mat,
        name="end_face",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        Box((0.380, 0.120, 0.035)),
        origin=Origin(xyz=(0.190, 0.0, 0.0175)),
        material=stage_2_mat,
        name="slide_block",
    )
    stage_2.visual(
        Box((0.010, 0.120, 0.035)),
        origin=Origin(xyz=(0.385, 0.0, 0.0175)),
        material=face_mat,
        name="end_face",
    )

    model.articulation(
        "body_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage_0,
        origin=Origin(xyz=(-0.330, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.22, lower=0.0, upper=0.180),
        motion_properties=MotionProperties(damping=4.0, friction=1.0),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.080, 0.0, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.20, lower=0.0, upper=0.150),
        motion_properties=MotionProperties(damping=3.0, friction=0.8),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.080, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.18, lower=0.0, upper=0.120),
        motion_properties=MotionProperties(damping=2.5, friction=0.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    stage_0 = object_model.get_part("stage_0")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    j0 = object_model.get_articulation("body_to_stage_0")
    j1 = object_model.get_articulation("stage_0_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")

    sliders = (stage_0, stage_1, stage_2)
    joints = (j0, j1, j2)
    ctx.check(
        "three serial prismatic joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in joints)
        and [j.parent for j in joints] == ["body", "stage_0", "stage_1"]
        and [j.child for j in joints] == ["stage_0", "stage_1", "stage_2"],
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "three carried sliders",
        all(len(slider.visuals) >= 2 for slider in sliders),
        details="each carried stage has a rectangular slide block and a plain end face",
    )

    ctx.expect_contact(
        body,
        stage_0,
        elem_a="fixed_slide_face",
        elem_b="slide_block",
        name="first stage is seated on the grounded slide face",
    )
    ctx.expect_contact(stage_0, stage_1, name="second stage is carried by first stage")
    ctx.expect_contact(stage_1, stage_2, name="third stage is carried by second stage")
    ctx.expect_overlap(
        stage_0,
        body,
        axes="x",
        min_overlap=0.50,
        elem_a="slide_block",
        elem_b="fixed_slide_face",
        name="first stage visibly overlaps the body at rest",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="x",
        min_overlap=0.42,
        elem_a="slide_block",
        elem_b="slide_block",
        name="second stage visibly overlaps first at rest",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="x",
        min_overlap=0.32,
        elem_a="slide_block",
        elem_b="slide_block",
        name="third stage visibly overlaps second at rest",
    )

    rest_tip = ctx.part_world_position(stage_2)
    with ctx.pose({j0: 0.180, j1: 0.150, j2: 0.120}):
        extended_tip = ctx.part_world_position(stage_2)
        ctx.expect_overlap(
            stage_0,
            body,
            axes="x",
            min_overlap=0.36,
            elem_a="slide_block",
            elem_b="fixed_slide_face",
            name="first stage remains retained at full travel",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="x",
            min_overlap=0.30,
            elem_a="slide_block",
            elem_b="slide_block",
            name="second stage remains retained at full travel",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="x",
            min_overlap=0.22,
            elem_a="slide_block",
            elem_b="slide_block",
            name="third stage remains retained at full travel",
        )

    ctx.check(
        "upper stage extends along the service axis",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.42,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
