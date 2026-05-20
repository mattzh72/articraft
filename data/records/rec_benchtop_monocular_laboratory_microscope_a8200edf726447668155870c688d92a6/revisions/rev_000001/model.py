from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _build_stage_plate():
    return (
        cq.Workplane("XY")
        .box(0.104, 0.094, 0.006)
        .faces(">Z")
        .workplane()
        .rect(0.028, 0.024)
        .cutThruAll()
        .union(cq.Workplane("XY").box(0.012, 0.094, 0.010).translate((-0.046, 0.0, 0.002)))
    )


def _build_stage_carriage():
    plate = (
        cq.Workplane("XY")
        .box(0.072, 0.034, 0.004)
        .faces(">Z")
        .workplane()
        .rect(0.022, 0.018)
        .cutThruAll()
    )
    clamp = cq.Workplane("XY").box(0.030, 0.010, 0.010).translate((0.014, 0.0, 0.007))
    projection = cq.Workplane("XY").box(0.020, 0.030, 0.008).translate((0.020, 0.030, 0.002))
    finger_tab = cq.Workplane("XY").box(0.010, 0.014, 0.014).translate((0.028, 0.045, 0.005))
    return plate.union(clamp).union(projection).union(finger_tab)


def _objective_barrel(offset_x: float, offset_y: float, length: float) -> tuple[Cylinder, Origin]:
    return (
        Cylinder(radius=0.0055, length=length),
        Origin(xyz=(offset_x, offset_y, -0.006 - length / 2.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_monocular_microscope")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.92, 1.0))
    body_grey = model.material("body_grey", rgba=(0.72, 0.73, 0.75, 1.0))
    stage_black = model.material("stage_black", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.220, 0.160, 0.028)),
        origin=Origin(xyz=(0.020, 0.000, 0.014)),
        material=body_white,
        name="base_shell",
    )
    frame.visual(
        Box((0.110, 0.112, 0.010)),
        origin=Origin(xyz=(0.052, 0.000, 0.028)),
        material=body_grey,
        name="base_top",
    )

    arm_geom = sweep_profile_along_spline(
        [
            (-0.050, 0.000, 0.018),
            (-0.044, 0.000, 0.072),
            (-0.030, 0.000, 0.142),
            (-0.014, 0.000, 0.205),
        ],
        profile=rounded_rect_profile(0.058, 0.020, 0.008),
        samples_per_segment=20,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    frame.visual(
        mesh_from_geometry(arm_geom, "microscope_arm"),
        material=body_white,
        name="arm_shell",
    )
    frame.visual(
        Box((0.034, 0.024, 0.140)),
        origin=Origin(xyz=(-0.010, 0.000, 0.100)),
        material=body_white,
        name="arm_web",
    )
    frame.visual(
        Box((0.040, 0.012, 0.220)),
        origin=Origin(xyz=(-0.008, 0.000, 0.260)),
        material=steel,
        name="guide_rail",
    )
    frame.visual(
        Box((0.092, 0.030, 0.024)),
        origin=Origin(xyz=(0.022, 0.000, 0.171)),
        material=body_white,
        name="stage_bracket",
    )
    frame.visual(
        mesh_from_cadquery(_build_stage_plate(), "microscope_stage_plate"),
        origin=Origin(xyz=(0.050, 0.000, 0.184)),
        material=stage_black,
        name="stage_plate",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.050, 0.000, 0.171)),
        material=dark_steel,
        name="substage_body",
    )
    frame.visual(
        Box((0.024, 0.044, 0.016)),
        origin=Origin(xyz=(0.060, 0.000, 0.154)),
        material=dark_steel,
        name="substage_mount",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.018, 0.006, 0.203), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=body_grey,
        name="fine_knob_boss",
    )

    head = model.part("head")
    head.visual(
        Box((0.052, 0.008, 0.064)),
        origin=Origin(xyz=(0.000, 0.011, 0.032)),
        material=body_grey,
        name="sleeve_front",
    )
    head.visual(
        Box((0.052, 0.008, 0.064)),
        origin=Origin(xyz=(0.000, -0.011, 0.032)),
        material=body_grey,
        name="sleeve_rear",
    )
    head.visual(
        Box((0.008, 0.030, 0.064)),
        origin=Origin(xyz=(0.022, 0.000, 0.032)),
        material=body_grey,
        name="sleeve_side_0",
    )
    head.visual(
        Box((0.008, 0.030, 0.064)),
        origin=Origin(xyz=(-0.022, 0.000, 0.032)),
        material=body_grey,
        name="sleeve_side_1",
    )
    head.visual(
        Box((0.084, 0.046, 0.052)),
        origin=Origin(xyz=(0.066, 0.000, 0.060)),
        material=body_white,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.034),
        origin=Origin(xyz=(0.080, 0.000, 0.022)),
        material=body_white,
        name="nose_block",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.122),
        origin=Origin(xyz=(-0.079, 0.000, 0.122), rpy=(0.000, -math.pi / 3.0, 0.000)),
        material=body_white,
        name="tube_body",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(-0.140, 0.000, 0.158), rpy=(0.000, -math.pi / 3.0, 0.000)),
        material=glass_black,
        name="eyepiece_tube",
    )
    head.visual(
        Box((0.060, 0.010, 0.030)),
        origin=Origin(xyz=(-0.004, 0.017, 0.078)),
        material=body_white,
        name="tube_strap_0",
    )
    head.visual(
        Box((0.060, 0.010, 0.030)),
        origin=Origin(xyz=(-0.004, -0.017, 0.078)),
        material=body_white,
        name="tube_strap_1",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=body_grey,
        name="turret_disk",
    )
    turret.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material=dark_steel,
        name="turret_hub",
    )
    objective_long, objective_long_origin = _objective_barrel(0.013, 0.000, 0.040)
    turret.visual(
        objective_long,
        origin=objective_long_origin,
        material=steel,
        name="objective_long",
    )
    objective_mid, objective_mid_origin = _objective_barrel(-0.006, 0.011, 0.031)
    turret.visual(
        objective_mid,
        origin=objective_mid_origin,
        material=steel,
        name="objective_mid",
    )
    objective_short, objective_short_origin = _objective_barrel(-0.006, -0.011, 0.024)
    turret.visual(
        objective_short,
        origin=objective_short_origin,
        material=steel,
        name="objective_short",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_stage_carriage(), "microscope_stage_carriage"),
        origin=Origin(xyz=(0.000, 0.016, 0.002)),
        material=body_grey,
        name="carriage_body",
    )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=dark_steel,
        name="coarse_shaft",
    )
    coarse_knob.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.000, 0.020, 0.000), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=body_grey,
        name="coarse_body",
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=dark_steel,
        name="fine_shaft",
    )
    fine_knob.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.000, 0.016, 0.000), rpy=(-math.pi / 2.0, 0.000, 0.000)),
        material=body_grey,
        name="fine_body",
    )

    model.articulation(
        "frame_to_head",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head,
        origin=Origin(xyz=(-0.008, 0.000, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "head_to_turret",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=turret,
        origin=Origin(xyz=(0.080, 0.000, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.050, 0.000, 0.187)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.06,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "frame_to_coarse_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=coarse_knob,
        origin=Origin(xyz=(-0.018, 0.010, 0.236)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "frame_to_fine_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=fine_knob,
        origin=Origin(xyz=(0.018, 0.010, 0.203)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    turret = object_model.get_part("turret")
    carriage = object_model.get_part("carriage")
    coarse_knob = object_model.get_part("coarse_knob")
    fine_knob = object_model.get_part("fine_knob")
    head_slide = object_model.get_articulation("frame_to_head")
    stage_slide = object_model.get_articulation("frame_to_carriage")

    ctx.allow_overlap(
        coarse_knob,
        "frame",
        elem_a="coarse_shaft",
        elem_b="guide_rail",
        reason="The coarse focus shaft intentionally passes into the column-side drive housing represented by the guide rail proxy.",
    )
    ctx.allow_overlap(
        fine_knob,
        "frame",
        elem_a="fine_shaft",
        elem_b="fine_knob_boss",
        reason="The fine focus shaft intentionally inserts into the external fine-focus mounting boss, which is modeled as a solid support collar.",
    )

    ctx.expect_gap(
        turret,
        carriage,
        axis="z",
        positive_elem="objective_long",
        negative_elem="carriage_body",
        min_gap=0.003,
        max_gap=0.020,
        name="objective clears stage carriage at rest",
    )
    ctx.expect_overlap(
        turret,
        head,
        axes="xy",
        elem_a="turret_disk",
        elem_b="nose_block",
        min_overlap=0.020,
        name="objective turret stays mounted beneath the head",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        elem_a="carriage_body",
        elem_b="stage_plate",
        min_overlap=0.060,
        name="stage carriage spans the stage opening",
    )
    ctx.expect_origin_distance(
        coarse_knob,
        fine_knob,
        axes="xz",
        min_dist=0.030,
        name="coarse and fine controls are visibly separate knobs",
    )

    rest_head_pos = ctx.part_world_position(head)
    rest_carriage_pos = ctx.part_world_position(carriage)

    head_limits = head_slide.motion_limits
    if head_limits is not None and head_limits.upper is not None:
        with ctx.pose({head_slide: head_limits.upper}):
            upper_head_pos = ctx.part_world_position(head)
            ctx.expect_gap(
                turret,
                carriage,
                axis="z",
                positive_elem="objective_long",
                negative_elem="carriage_body",
                min_gap=0.030,
                name="raising focus lifts the objective clear of the stage",
            )
        ctx.check(
            "head carriage rises on the visible guide",
            rest_head_pos is not None
            and upper_head_pos is not None
            and upper_head_pos[2] > rest_head_pos[2] + 0.060,
            details=f"rest={rest_head_pos}, raised={upper_head_pos}",
        )

    stage_limits = stage_slide.motion_limits
    if stage_limits is not None and stage_limits.lower is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            ctx.expect_overlap(
                carriage,
                frame,
                axes="y",
                elem_a="carriage_body",
                elem_b="stage_plate",
                min_overlap=0.020,
                name="carriage stays retained at lower travel",
            )
        with ctx.pose({stage_slide: stage_limits.upper}):
            upper_carriage_pos = ctx.part_world_position(carriage)
            ctx.expect_overlap(
                carriage,
                frame,
                axes="y",
                elem_a="carriage_body",
                elem_b="stage_plate",
                min_overlap=0.020,
                name="carriage stays retained at upper travel",
            )
        ctx.check(
            "stage carriage slides across the stage",
            rest_carriage_pos is not None
            and upper_carriage_pos is not None
            and upper_carriage_pos[1] > rest_carriage_pos[1] + 0.015,
            details=f"rest={rest_carriage_pos}, moved={upper_carriage_pos}",
        )

    return ctx.report()


object_model = build_object_model()
