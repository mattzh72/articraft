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
)


def _base_casting_shape() -> cq.Workplane:
    sole = cq.Workplane("XY").box(0.225, 0.165, 0.016, centered=(True, True, False))
    front_opening = (
        cq.Workplane("XY")
        .center(0.068, 0.0)
        .box(0.120, 0.084, 0.028, centered=(True, True, False))
    )
    sole = sole.cut(front_opening)

    upper_cast = (
        cq.Workplane("XY")
        .center(-0.028, 0.0)
        .rect(0.144, 0.126)
        .workplane(offset=0.026)
        .rect(0.114, 0.096)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.016))
    )

    pedestal = (
        cq.Workplane("XY")
        .center(-0.048, 0.0)
        .box(0.052, 0.076, 0.040, centered=(True, True, False))
        .translate((0.0, 0.0, 0.016))
    )

    return sole.union(upper_cast).union(pedestal)


def _carriage_sleeve_shape() -> cq.Workplane:
    left_wall = cq.Workplane("XY").center(0.0, 0.0185).box(0.066, 0.009, 0.095, centered=(True, True, False))
    right_wall = cq.Workplane("XY").center(0.0, -0.0185).box(0.066, 0.009, 0.095, centered=(True, True, False))
    rear_wall = cq.Workplane("XY").center(-0.0255, 0.0).box(0.013, 0.032, 0.095, centered=(True, True, False))
    front_wall = cq.Workplane("XY").center(0.0255, 0.0).box(0.013, 0.032, 0.095, centered=(True, True, False))
    front_flange = (
        cq.Workplane("XY")
        .center(0.026, 0.0)
        .box(0.030, 0.050, 0.022, centered=(True, True, False))
        .translate((0.0, 0.0, 0.048))
    )
    return left_wall.union(right_wall).union(rear_wall).union(front_wall).union(front_flange)


def _arm_body_shape() -> cq.Workplane:
    arm = (
        cq.Workplane("XY")
        .center(0.044, 0.0)
        .box(0.034, 0.060, 0.150, centered=(True, True, False))
        .translate((0.0, 0.0, 0.048))
    )
    bridge = (
        cq.Workplane("XY")
        .center(0.080, 0.0)
        .box(0.038, 0.056, 0.024, centered=(True, True, False))
        .translate((0.0, 0.0, 0.196))
    )
    head = (
        cq.Workplane("XY")
        .center(0.122, 0.0)
        .box(0.096, 0.056, 0.044, centered=(True, True, False))
        .translate((0.0, 0.0, 0.190))
    )
    stage_post = (
        cq.Workplane("XY")
        .center(0.094, 0.0)
        .box(0.012, 0.024, 0.058, centered=(True, True, False))
        .translate((0.0, 0.0, 0.086))
    )
    stage_bridge = (
        cq.Workplane("XY")
        .center(0.078, 0.0)
        .box(0.052, 0.024, 0.014, centered=(True, True, False))
        .translate((0.0, 0.0, 0.118))
    )
    nose_hub = (
        cq.Workplane("XY")
        .center(0.136, 0.0)
        .circle(0.019)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.172))
    )
    return arm.union(bridge).union(head).union(stage_post).union(stage_bridge).union(nose_hub)


def _eyepiece_tube_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.014).extrude(0.078)
    inner = cq.Workplane("XY").circle(0.009).extrude(0.072).translate((0.0, 0.0, 0.006))
    collar = cq.Workplane("XY").circle(0.016).extrude(0.010)
    ocular = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.016)
        .translate((0.0, 0.0, 0.062))
    )
    tube = outer.cut(inner).union(collar).union(ocular)
    tube = tube.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -40.0)
    return tube.translate((0.078, 0.0, 0.210))


def _stage_shape() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .box(0.100, 0.090, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.016))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.072, 0.050, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
    )
    left_skirt = (
        cq.Workplane("XY")
        .center(0.0, 0.014)
        .box(0.072, 0.010, 0.024, centered=(True, True, False))
        .translate((0.0, 0.0, -0.010))
    )
    right_skirt = (
        cq.Workplane("XY")
        .center(0.0, -0.014)
        .box(0.072, 0.010, 0.024, centered=(True, True, False))
        .translate((0.0, 0.0, -0.010))
    )
    finger_lip = (
        cq.Workplane("XY")
        .center(0.040, 0.0)
        .box(0.012, 0.090, 0.007, centered=(True, True, False))
        .translate((0.0, 0.0, 0.012))
    )
    body = top_plate.union(bridge).union(left_skirt).union(right_skirt).union(finger_lip)
    aperture = (
        cq.Workplane("XY")
        .rect(0.060, 0.034)
        .extrude(0.040, both=True)
        .translate((0.000, 0.0, 0.010))
    )
    return body.cut(aperture)


def _turret_shape() -> cq.Workplane:
    turret = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.010))
    )
    spindle = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.018)
        .translate((0.0, 0.0, -0.018))
    )
    turret = turret.union(spindle)
    objective_angles = (0.0, 120.0, 240.0)
    body_lengths = (0.020, 0.022, 0.024)
    tip_lengths = (0.012, 0.013, 0.014)
    for angle_deg, body_len, tip_len in zip(objective_angles, body_lengths, tip_lengths):
        angle = math.radians(angle_deg)
        x = 0.017 * math.cos(angle)
        y = 0.017 * math.sin(angle)
        body = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(0.0075)
            .extrude(body_len)
            .translate((0.0, 0.0, -0.010 - body_len))
        )
        tip = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(0.0046)
            .extrude(tip_len)
            .translate((0.0, 0.0, -0.010 - body_len - tip_len))
        )
        turret = turret.union(body).union(tip)
    return turret


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clinical_monocular_microscope")

    ivory = model.material("ivory_enamel", rgba=(0.82, 0.81, 0.76, 1.0))
    stage_black = model.material("stage_black", rgba=(0.11, 0.11, 0.12, 1.0))
    tube_black = model.material("tube_black", rgba=(0.15, 0.15, 0.16, 1.0))
    column_grey = model.material("column_grey", rgba=(0.56, 0.58, 0.61, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    turret_dark = model.material("turret_dark", rgba=(0.17, 0.17, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_casting_shape(), "microscope_base_casting"),
        material=ivory,
        name="base_casting",
    )
    base.visual(
        Box((0.038, 0.028, 0.230)),
        origin=Origin(xyz=(-0.048, 0.0, 0.170)),
        material=column_grey,
        name="column_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.066, 0.009, 0.095)),
        origin=Origin(xyz=(0.0, 0.0185, 0.0475)),
        material=ivory,
        name="sleeve_left",
    )
    carriage.visual(
        Box((0.066, 0.009, 0.095)),
        origin=Origin(xyz=(0.0, -0.0185, 0.0475)),
        material=ivory,
        name="sleeve_right",
    )
    carriage.visual(
        Box((0.014, 0.032, 0.095)),
        origin=Origin(xyz=(-0.0260, 0.0, 0.0475)),
        material=ivory,
        name="sleeve_rear",
    )
    carriage.visual(
        Box((0.014, 0.032, 0.095)),
        origin=Origin(xyz=(0.0260, 0.0, 0.0475)),
        material=ivory,
        name="sleeve_front",
    )
    carriage.visual(
        Box((0.030, 0.050, 0.022)),
        origin=Origin(xyz=(0.038, 0.0, 0.059)),
        material=ivory,
        name="carriage_sleeve",
    )
    carriage.visual(
        mesh_from_cadquery(_arm_body_shape(), "microscope_arm_body"),
        material=ivory,
        name="arm_body",
    )
    carriage.visual(
        mesh_from_cadquery(_eyepiece_tube_shape(), "microscope_eyepiece_tube"),
        material=tube_black,
        name="eyepiece_tube",
    )
    carriage.visual(
        Box((0.080, 0.022, 0.014)),
        origin=Origin(xyz=(0.174, 0.0, 0.086)),
        material=column_grey,
        name="stage_guide",
    )
    carriage.visual(
        Box((0.084, 0.018, 0.014)),
        origin=Origin(xyz=(0.132, 0.0, 0.086)),
        material=ivory,
        name="stage_beam",
    )
    carriage.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.040, 0.028, 0.106), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="focus_mount",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.040, 0.028, 0.136), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="fine_mount",
    )
    carriage.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.174, 0.0, 0.175)),
        material=ivory,
        name="turret_mount",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.038, 0.090, 0.006)),
        origin=Origin(xyz=(0.031, 0.0, 0.019)),
        material=stage_black,
        name="stage_front_strip",
    )
    stage.visual(
        Box((0.038, 0.090, 0.006)),
        origin=Origin(xyz=(-0.031, 0.0, 0.019)),
        material=stage_black,
        name="stage_rear_strip",
    )
    stage.visual(
        Box((0.024, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.030, 0.019)),
        material=stage_black,
        name="stage_upper_strip",
    )
    stage.visual(
        Box((0.024, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.030, 0.019)),
        material=stage_black,
        name="stage_lower_strip",
    )
    stage.visual(
        Box((0.072, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stage_black,
        name="stage_carriage",
    )
    stage.visual(
        Box((0.072, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.002)),
        material=stage_black,
        name="stage_left_skirt",
    )
    stage.visual(
        Box((0.072, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, 0.002)),
        material=stage_black,
        name="stage_right_skirt",
    )
    stage.visual(
        Box((0.012, 0.090, 0.007)),
        origin=Origin(xyz=(0.040, 0.0, 0.0155)),
        material=stage_black,
        name="stage_lip",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=turret_dark,
        name="turret_body",
    )
    turret.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=turret_dark,
        name="turret_spindle",
    )
    for index, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        x = 0.017 * math.cos(angle)
        y = 0.017 * math.sin(angle)
        objective_length = 0.022 + 0.002 * index
        tip_length = 0.010 + 0.0015 * index
        turret.visual(
            Cylinder(radius=0.007, length=objective_length),
            origin=Origin(xyz=(x, y, -0.010 - objective_length / 2.0)),
            material=turret_dark,
            name=f"objective_{index}",
        )
        turret.visual(
            Cylinder(radius=0.0045, length=tip_length),
            origin=Origin(
                xyz=(x, y, -0.010 - objective_length - tip_length / 2.0)
            ),
            material=turret_dark,
            name=f"objective_tip_{index}",
        )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.022, length=0.009),
        origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="coarse_knob",
    )
    coarse_knob.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="coarse_cap",
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.010, length=0.007),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="fine_knob",
    )
    fine_knob.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.0, 0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="fine_cap",
    )

    base_to_carriage = model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.048, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.04,
            lower=0.0,
            upper=0.095,
        ),
    )

    carriage_to_stage = model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(0.174, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.03,
            lower=0.0,
            upper=0.028,
        ),
    )

    model.articulation(
        "carriage_to_turret",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=turret,
        origin=Origin(xyz=(0.174, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )

    coarse_knob_origin = Origin(xyz=(0.040, 0.033, 0.106))
    fine_knob_origin = Origin(xyz=(0.040, 0.032, 0.136))
    knob_limits = MotionLimits(effort=0.35, velocity=8.0)
    model.articulation(
        "carriage_to_coarse_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=coarse_knob,
        origin=coarse_knob_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=knob_limits,
    )
    model.articulation(
        "carriage_to_fine_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=fine_knob,
        origin=fine_knob_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.18, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("stage")
    turret = object_model.get_part("turret")
    coarse_knob = object_model.get_part("coarse_knob")
    fine_knob = object_model.get_part("fine_knob")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    stage_slide = object_model.get_articulation("carriage_to_stage")

    carriage_limits = carriage_slide.motion_limits
    stage_limits = stage_slide.motion_limits

    if carriage_limits is not None and carriage_limits.upper is not None:
        with ctx.pose({carriage_slide: 0.0}):
            ctx.expect_within(
                base,
                carriage,
                axes="xy",
                inner_elem="column_rail",
                margin=0.003,
                name="carriage sleeve stays centered on the column at rest",
            )
            ctx.expect_overlap(
                base,
                carriage,
                axes="z",
                elem_a="column_rail",
                min_overlap=0.090,
                name="carriage sleeve remains engaged on the column at rest",
            )
            ctx.expect_gap(
                turret,
                stage,
                axis="z",
                min_gap=0.003,
                max_gap=0.020,
                name="objective cluster clears the stage in the lowest focus pose",
            )

        with ctx.pose({carriage_slide: carriage_limits.upper}):
            ctx.expect_within(
                base,
                carriage,
                axes="xy",
                inner_elem="column_rail",
                margin=0.003,
                name="carriage sleeve stays centered on the column when raised",
            )
            ctx.expect_overlap(
                base,
                carriage,
                axes="z",
                elem_a="column_rail",
                min_overlap=0.090,
                name="carriage sleeve remains engaged on the column when raised",
            )

        rest_carriage_pos = ctx.part_world_position(carriage)
        with ctx.pose({carriage_slide: carriage_limits.upper}):
            raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.check(
            "focus carriage rises on positive travel",
            rest_carriage_pos is not None
            and raised_carriage_pos is not None
            and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.08,
            details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
        )

    if (
        stage_limits is not None
        and stage_limits.lower is not None
        and stage_limits.upper is not None
    ):
        with ctx.pose({stage_slide: stage_limits.lower}):
            ctx.expect_within(
                carriage,
                stage,
                axes="yz",
                inner_elem="stage_guide",
                margin=0.002,
                name="stage saddle stays wrapped around the guide at rear travel",
            )
            ctx.expect_overlap(
                carriage,
                stage,
                axes="x",
                elem_a="stage_guide",
                min_overlap=0.060,
                name="stage retains guide engagement at rear travel",
            )

        with ctx.pose({stage_slide: stage_limits.upper}):
            ctx.expect_within(
                carriage,
                stage,
                axes="yz",
                inner_elem="stage_guide",
                margin=0.002,
                name="stage saddle stays wrapped around the guide at forward travel",
            )
            ctx.expect_overlap(
                carriage,
                stage,
                axes="x",
                elem_a="stage_guide",
                min_overlap=0.060,
                name="stage retains guide engagement at forward travel",
            )

        with ctx.pose({stage_slide: stage_limits.lower}):
            rear_stage_pos = ctx.part_world_position(stage)
        with ctx.pose({stage_slide: stage_limits.upper}):
            forward_stage_pos = ctx.part_world_position(stage)
        ctx.check(
            "stage moves forward on positive travel",
            rear_stage_pos is not None
            and forward_stage_pos is not None
            and forward_stage_pos[0] > rear_stage_pos[0] + 0.025,
            details=f"rear={rear_stage_pos}, forward={forward_stage_pos}",
        )

    ctx.expect_contact(
        coarse_knob,
        carriage,
        name="coarse focus knob mounts to the arm side",
    )
    ctx.expect_contact(
        fine_knob,
        carriage,
        name="fine focus knob mounts independently on the arm side",
    )

    return ctx.report()


object_model = build_object_model()
