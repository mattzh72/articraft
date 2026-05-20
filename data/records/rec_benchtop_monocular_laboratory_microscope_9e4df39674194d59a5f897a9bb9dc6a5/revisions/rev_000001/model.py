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
    model = ArticulatedObject(name="metal_monocular_microscope")

    cast_metal = model.material("warm_gray_cast_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    dark_metal = model.material("dark_burnished_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))
    glass = model.material("pale_slide_glass", rgba=(0.72, 0.90, 0.96, 0.45))
    brass = model.material("brass_detail", rgba=(0.80, 0.62, 0.30, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.280, 0.180, 0.050)),
        origin=Origin(xyz=(0.015, 0.0, 0.025)),
        material=dark_metal,
        name="deep_base",
    )
    frame.visual(
        Box((0.205, 0.125, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, 0.053)),
        material=cast_metal,
        name="base_inset",
    )
    for i, x in enumerate((-0.085, 0.115)):
        for j, y in enumerate((-0.060, 0.060)):
            frame.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, y, 0.001)),
                material=rubber,
                name=f"rubber_foot_{i}_{j}",
            )
    frame.visual(
        Box((0.055, 0.065, 0.340)),
        origin=Origin(xyz=(0.105, 0.0, 0.220)),
        material=cast_metal,
        name="upright_arm",
    )
    frame.visual(
        Box((0.008, 0.074, 0.215)),
        origin=Origin(xyz=(0.075, 0.0, 0.285)),
        material=dark_metal,
        name="focus_rail",
    )
    frame.visual(
        Box((0.113, 0.038, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, 0.170)),
        material=cast_metal,
        name="stage_support",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=dark_metal,
        name="condenser_housing",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(-0.030, 0.0, 0.171), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_metal,
        name="diaphragm_boss",
    )

    body = model.part("body_carriage")
    body.visual(
        Box((0.016, 0.075, 0.160)),
        origin=Origin(xyz=(0.063, 0.0, 0.305)),
        material=dark_metal,
        name="dovetail_slide",
    )
    body.visual(
        Box((0.066, 0.050, 0.040)),
        origin=Origin(xyz=(0.032, 0.0, 0.330)),
        material=cast_metal,
        name="head_bridge",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=cast_metal,
        name="body_tube",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=dark_metal,
        name="lower_tube_collar",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.4925)),
        material=black,
        name="eyepiece",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="turret_disk",
    )
    nosepiece.visual(
        Cylinder(radius=0.018, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=cast_metal,
        name="turret_hub",
    )
    for i, (x, y, radius, length) in enumerate(
        (
            (0.023, 0.000, 0.0075, 0.058),
            (-0.0115, 0.0199, 0.0065, 0.050),
            (-0.0115, -0.0199, 0.0060, 0.044),
        )
    ):
        nosepiece.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, -0.036)),
            material=brass if i == 0 else dark_metal,
            name=f"objective_{i}",
        )
        nosepiece.visual(
            Cylinder(radius=radius * 1.45, length=0.010),
            origin=Origin(xyz=(x, y, -0.015)),
            material=dark_metal,
            name=f"objective_collar_{i}",
        )

    stage = model.part("stage_carriage")
    stage.visual(
        Box((0.135, 0.116, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="stage_plate",
    )
    stage.visual(
        Box((0.046, 0.026, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0058)),
        material=black,
        name="stage_aperture",
    )
    stage.visual(
        Box((0.078, 0.026, 0.002)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0069)),
        material=glass,
        name="glass_slide",
    )
    for i, y in enumerate((-0.036, 0.036)):
        stage.visual(
            Box((0.080, 0.006, 0.003)),
            origin=Origin(xyz=(0.0, y, 0.0068)),
            material=cast_metal,
            name=f"slide_clip_{i}",
        )
    stage.visual(
        Box((0.080, 0.020, 0.020)),
        origin=Origin(xyz=(-0.045, -0.067, -0.010)),
        material=cast_metal,
        name="guide_block",
    )
    stage.visual(
        Box((0.090, 0.012, 0.008)),
        origin=Origin(xyz=(0.000, 0.064, -0.003)),
        material=cast_metal,
        name="stage_slide_rail",
    )
    stage.visual(
        Box((0.036, 0.050, 0.014)),
        origin=Origin(xyz=(0.049, 0.0, -0.013)),
        material=cast_metal,
        name="stage_saddle",
    )

    guide_origins = [(-0.065, -0.077, -0.010), (-0.025, -0.077, -0.010)]
    for i, origin_xyz in enumerate(guide_origins):
        guide = model.part(f"guide_control_{i}")
        guide.visual(
            Cylinder(radius=0.004, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="guide_shaft",
        )
        guide.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="guide_knob",
        )
        guide.visual(
            Box((0.004, 0.002, 0.008)),
            origin=Origin(xyz=(0.0, -0.027, 0.010)),
            material=cast_metal,
            name="guide_tick",
        )
        model.articulation(
            f"stage_to_guide_{i}",
            ArticulationType.CONTINUOUS,
            parent=stage,
            child=guide,
            origin=Origin(xyz=origin_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=5.0),
        )

    lever = model.part("diaphragm_lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="lever_hub",
    )
    lever.visual(
        Box((0.006, 0.062, 0.003)),
        origin=Origin(xyz=(0.0, -0.033, 0.0)),
        material=dark_metal,
        name="lever_blade",
    )
    lever.visual(
        Box((0.014, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.068, 0.0)),
        material=black,
        name="lever_tab",
    )

    model.articulation(
        "frame_to_body",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.08, lower=-0.018, upper=0.026),
    )
    model.articulation(
        "body_to_nosepiece",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0),
    )
    model.articulation(
        "frame_to_stage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.06, lower=-0.030, upper=0.030),
    )
    model.articulation(
        "frame_to_diaphragm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=(-0.0415, 0.0, 0.171)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    body = object_model.get_part("body_carriage")
    nosepiece = object_model.get_part("nosepiece")
    stage = object_model.get_part("stage_carriage")
    lever = object_model.get_part("diaphragm_lever")
    guide_0 = object_model.get_part("guide_control_0")
    guide_1 = object_model.get_part("guide_control_1")

    focus = object_model.get_articulation("frame_to_body")
    stage_slide = object_model.get_articulation("frame_to_stage")
    turret = object_model.get_articulation("body_to_nosepiece")
    diaphragm = object_model.get_articulation("frame_to_diaphragm")

    ctx.allow_overlap(
        frame,
        lever,
        elem_a="diaphragm_boss",
        elem_b="lever_hub",
        reason="The iris lever hub is intentionally captured on the short pivot boss projecting from the condenser housing.",
    )
    ctx.expect_overlap(
        frame,
        lever,
        axes="xy",
        min_overlap=0.006,
        elem_a="diaphragm_boss",
        elem_b="lever_hub",
        name="diaphragm lever hub is seated on condenser boss",
    )
    ctx.expect_contact(
        body,
        nosepiece,
        elem_a="lower_tube_collar",
        elem_b="turret_hub",
        contact_tol=0.001,
        name="nosepiece hub is seated in lower tube collar",
    )
    ctx.expect_gap(
        stage,
        frame,
        axis="z",
        min_gap=0.003,
        max_gap=0.012,
        positive_elem="stage_plate",
        negative_elem="condenser_housing",
        name="stage clears condenser housing",
    )
    ctx.expect_contact(
        stage,
        frame,
        elem_a="stage_saddle",
        elem_b="stage_support",
        contact_tol=0.001,
        name="stage saddle rides on fixed stage support",
    )
    ctx.expect_gap(
        nosepiece,
        stage,
        axis="z",
        min_gap=0.004,
        positive_elem="objective_0",
        negative_elem="glass_slide",
        name="objective clears specimen slide at rest",
    )
    with ctx.pose({focus: -0.018}):
        ctx.expect_gap(
            nosepiece,
            stage,
            axis="z",
            min_gap=0.002,
            positive_elem="objective_0",
            negative_elem="glass_slide",
            name="focused objective still clears slide",
        )

    rest_body = ctx.part_world_position(body)
    with ctx.pose({focus: 0.020}):
        raised_body = ctx.part_world_position(body)
    ctx.check(
        "body carriage focuses upward",
        rest_body is not None and raised_body is not None and raised_body[2] > rest_body[2] + 0.015,
        details=f"rest={rest_body}, raised={raised_body}",
    )

    rest_stage = ctx.part_world_position(stage)
    with ctx.pose({stage_slide: 0.025}):
        shifted_stage = ctx.part_world_position(stage)
    ctx.check(
        "stage carriage slides along its guide axis",
        rest_stage is not None and shifted_stage is not None and shifted_stage[1] > rest_stage[1] + 0.020,
        details=f"rest={rest_stage}, shifted={shifted_stage}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    turret_start = _aabb_center(ctx.part_element_world_aabb(nosepiece, elem="objective_0"))
    with ctx.pose({turret: math.pi / 2.0}):
        turret_quarter = _aabb_center(ctx.part_element_world_aabb(nosepiece, elem="objective_0"))
    ctx.check(
        "nosepiece rotates about optical axis",
        turret_start is not None
        and turret_quarter is not None
        and abs(turret_quarter[1] - turret_start[1]) > 0.015,
        details=f"start={turret_start}, quarter_turn={turret_quarter}",
    )

    lever_start = _aabb_center(ctx.part_element_world_aabb(lever, elem="lever_tab"))
    with ctx.pose({diaphragm: 0.55}):
        lever_swept = _aabb_center(ctx.part_element_world_aabb(lever, elem="lever_tab"))
    ctx.check(
        "diaphragm lever sweeps under stage",
        lever_start is not None
        and lever_swept is not None
        and abs(lever_swept[0] - lever_start[0]) > 0.020,
        details=f"start={lever_start}, swept={lever_swept}",
    )

    ctx.expect_contact(
        stage,
        guide_0,
        elem_a="guide_block",
        elem_b="guide_shaft",
        contact_tol=0.001,
        name="first guide control is mounted on stage block",
    )
    ctx.expect_contact(
        stage,
        guide_1,
        elem_a="guide_block",
        elem_b="guide_shaft",
        contact_tol=0.001,
        name="second guide control is mounted on stage block",
    )

    return ctx.report()


object_model = build_object_model()
