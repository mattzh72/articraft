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


def _stand_shell_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.240, 0.170, 0.022)
        .translate((0.0, 0.0, 0.011))
        .edges("|Z")
        .fillet(0.010)
    )

    outer_profile = [
        (-0.082, 0.022),
        (-0.090, 0.110),
        (-0.082, 0.210),
        (-0.052, 0.312),
        (-0.018, 0.382),
        (0.004, 0.402),
        (0.020, 0.390),
        (0.015, 0.332),
        (0.000, 0.248),
        (0.008, 0.142),
        (0.020, 0.080),
        (0.010, 0.050),
        (-0.005, 0.055),
        (-0.050, 0.056),
    ]
    inner_profile = [
        (-0.050, 0.085),
        (-0.058, 0.160),
        (-0.050, 0.242),
        (-0.028, 0.308),
        (-0.004, 0.348),
        (0.010, 0.352),
        (0.004, 0.292),
        (-0.006, 0.210),
        (0.000, 0.120),
        (0.010, 0.095),
        (0.004, 0.088),
        (-0.008, 0.094),
    ]
    arm_outer = (
        cq.Workplane("XZ").polyline(outer_profile).close().extrude(0.030).translate((0.0, 0.015, -0.004))
    )
    arm_inner = (
        cq.Workplane("XZ").polyline(inner_profile).close().extrude(0.034).translate((0.0, 0.017, -0.004))
    )
    arm = arm_outer.cut(arm_inner)

    return base.union(arm)


def _stage_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.106, 0.124, 0.008)
    opening = cq.Workplane("XY").box(0.026, 0.058, 0.020)
    finger_cut = cq.Workplane("XY").box(0.040, 0.018, 0.020).translate((0.040, 0.000, 0.0))
    return plate.cut(opening).cut(finger_cut)


def _condenser_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").cylinder(0.006, 0.030).translate((0.0, 0.0, -0.006))
    body = cq.Workplane("XY").cylinder(0.028, 0.022).translate((0.0, 0.0, -0.034))
    lower_lens = cq.Workplane("XY").cylinder(0.012, 0.016).translate((0.0, 0.0, -0.038))
    pivot_bridge = cq.Workplane("XY").box(0.020, 0.016, 0.006).translate((0.012, 0.020, -0.024))
    return flange.union(body).union(lower_lens).union(pivot_bridge)


def _head_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(0.052, 0.050, 0.102).translate((-0.035, 0.0, 0.078))
    guide_clearance = cq.Workplane("XY").box(0.024, 0.038, 0.192).translate((-0.035, 0.0, 0.060))
    carriage = carriage.cut(guide_clearance)

    bridge = cq.Workplane("XY").box(0.060, 0.040, 0.052).translate((-0.018, 0.0, 0.024))
    head_body = (
        cq.Workplane("XY")
        .box(0.060, 0.050, 0.030)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -25.0)
        .translate((-0.040, 0.0, 0.114))
    )
    eyepiece_tube = (
        cq.Workplane("XY")
        .cylinder(0.074, 0.014)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -25.0)
        .translate((-0.064, 0.0, 0.142))
    )
    eyepiece_collar = (
        cq.Workplane("XY")
        .cylinder(0.026, 0.016)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -25.0)
        .translate((-0.094, 0.0, 0.157))
    )
    return carriage.union(bridge).union(head_body).union(eyepiece_tube).union(eyepiece_collar)


def _stage_carriage_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.100, 0.088, 0.006).translate((0.0, 0.0, 0.003))
    aperture = cq.Workplane("XY").box(0.024, 0.056, 0.012).translate((0.0, 0.0, 0.003))
    frame = frame.cut(aperture)

    side_block = cq.Workplane("XY").box(0.030, 0.028, 0.012).translate((0.010, 0.056, 0.006))
    pusher = cq.Workplane("XY").box(0.014, 0.014, 0.010).translate((0.022, 0.050, 0.012))
    return frame.union(side_block).union(pusher)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_monocular_microscope")

    model.material("frame_ivory", rgba=(0.84, 0.84, 0.80, 1.0))
    model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("guide_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("metal", rgba=(0.72, 0.74, 0.78, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shell_shape(), "stand_shell"),
        material="frame_ivory",
        name="stand_shell",
    )
    stand.visual(
        Box((0.016, 0.030, 0.150)),
        origin=Origin(xyz=(0.045, 0.000, 0.295)),
        material="guide_gray",
        name="guide_column",
    )
    stand.visual(
        mesh_from_cadquery(_stage_plate_shape(), "stage_plate"),
        origin=Origin(xyz=(0.085, 0.000, 0.190)),
        material="base_black",
        name="stage_plate",
    )
    stand.visual(
        Box((0.024, 0.034, 0.070)),
        origin=Origin(xyz=(0.020, 0.000, 0.151)),
        material="frame_ivory",
        name="stage_bracket",
    )
    stand.visual(
        Box((0.050, 0.028, 0.020)),
        origin=Origin(xyz=(0.025, 0.000, 0.220)),
        material="frame_ivory",
        name="guide_strut",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.085, 0.000, 0.183)),
        material="trim_black",
        name="condenser_flange",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.085, 0.000, 0.171)),
        material="trim_black",
        name="condenser_housing",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.085, 0.000, 0.151)),
        material="trim_black",
        name="condenser_lower",
    )
    stand.visual(
        Box((0.020, 0.016, 0.008)),
        origin=Origin(xyz=(0.097, 0.020, 0.168)),
        material="trim_black",
        name="pivot_bridge",
    )
    stand.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.105, 0.024, 0.164)),
        material="trim_black",
        name="pivot_post",
    )
    stand.visual(
        Box((0.220, 0.150, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material="base_black",
        name="base_pad",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "head_shell"),
        material="frame_ivory",
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material="frame_ivory",
        name="nose_mount",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(
            xyz=(-0.106, 0.000, 0.162),
            rpy=(0.0, math.radians(-25.0), 0.0),
        ),
        material="trim_black",
        name="eyecup",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material="metal",
        name="turret_hub",
    )
    for angle_deg, upper_len, lower_len in (
        (0.0, 0.010, 0.014),
        (120.0, 0.010, 0.012),
        (240.0, 0.010, 0.010),
    ):
        angle = math.radians(angle_deg)
        x_pos = 0.014 * math.cos(angle)
        y_pos = 0.014 * math.sin(angle)
        turret.visual(
            Cylinder(radius=0.007, length=upper_len),
            origin=Origin(xyz=(x_pos, y_pos, -0.021)),
            material="metal",
            name=f"objective_upper_{int(angle_deg)}",
        )
        turret.visual(
            Cylinder(radius=0.0055, length=lower_len),
            origin=Origin(xyz=(x_pos, y_pos, -0.031)),
            material="metal",
            name=f"objective_lower_{int(angle_deg)}",
        )

    stage_carriage = model.part("stage_carriage")
    stage_carriage.visual(
        mesh_from_cadquery(_stage_carriage_shape(), "stage_carriage"),
        material="trim_black",
        name="carriage_frame",
    )
    stage_carriage.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.010, 0.076, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim_black",
        name="carriage_knob",
    )

    diaphragm_lever = model.part("diaphragm_lever")
    diaphragm_lever.visual(
        Cylinder(radius=0.0035, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, -0.003)),
        material="trim_black",
        name="pivot_barrel",
    )
    diaphragm_lever.visual(
        Box((0.028, 0.006, 0.004)),
        origin=Origin(xyz=(0.016, 0.000, -0.004)),
        material="trim_black",
        name="lever_blade",
    )
    diaphragm_lever.visual(
        Box((0.010, 0.012, 0.003)),
        origin=Origin(xyz=(0.032, 0.000, -0.004)),
        material="trim_black",
        name="lever_tip",
    )

    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.085, 0.000, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.050, lower=0.0, upper=0.045),
    )
    model.articulation(
        "turret_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=turret,
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_carriage,
        origin=Origin(xyz=(0.085, 0.000, 0.194)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.040, lower=-0.018, upper=0.018),
    )
    model.articulation(
        "diaphragm_lever_joint",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=diaphragm_lever,
        origin=Origin(xyz=(0.105, 0.024, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=math.radians(-45.0),
            upper=math.radians(20.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    turret = object_model.get_part("turret")
    stage_carriage = object_model.get_part("stage_carriage")
    diaphragm_lever = object_model.get_part("diaphragm_lever")

    head_slide = object_model.get_articulation("head_slide")
    turret_spin = object_model.get_articulation("turret_spin")
    stage_slide = object_model.get_articulation("stage_slide")

    head_limits = head_slide.motion_limits
    stage_limits = stage_slide.motion_limits

    ctx.allow_overlap(
        head,
        stand,
        elem_a="head_shell",
        elem_b="stand_shell",
        reason="The simplified optical-head carriage is modeled as a nested sleeve sliding within the cast arm throat.",
    )
    ctx.allow_overlap(
        head,
        stand,
        elem_a="head_shell",
        elem_b="guide_column",
        reason="The visible guide is simplified as a solid rail running through the carriage sleeve volume.",
    )

    ctx.expect_overlap(
        head,
        stand,
        axes="xy",
        elem_b="guide_column",
        min_overlap=0.016,
        name="head carriage stays aligned to the visible guide",
    )

    rest_head_pos = ctx.part_world_position(head)
    if head_limits is not None and head_limits.upper is not None:
        with ctx.pose({head_slide: head_limits.upper}):
            ctx.expect_overlap(
                head,
                stand,
                axes="xy",
                elem_b="guide_column",
                min_overlap=0.016,
                name="raised head carriage still overlaps the guide column footprint",
            )
            raised_head_pos = ctx.part_world_position(head)
        ctx.check(
            "head slide raises upward",
            rest_head_pos is not None
            and raised_head_pos is not None
            and raised_head_pos[2] > rest_head_pos[2] + 0.030,
            details=f"rest={rest_head_pos}, raised={raised_head_pos}",
        )

    ctx.expect_within(
        stage_carriage,
        stand,
        axes="x",
        inner_elem="carriage_frame",
        outer_elem="stage_plate",
        margin=0.004,
        name="stage carriage remains over the stage opening in X",
    )
    ctx.expect_overlap(
        stage_carriage,
        stand,
        axes="y",
        elem_a="carriage_frame",
        elem_b="stage_plate",
        min_overlap=0.040,
        name="stage carriage retains lateral engagement with the stage",
    )

    rest_stage_pos = ctx.part_world_position(stage_carriage)
    if stage_limits is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.upper}):
            ctx.expect_within(
                stage_carriage,
                stand,
                axes="x",
                inner_elem="carriage_frame",
                outer_elem="stage_plate",
                margin=0.004,
                name="extended stage carriage still stays over the stage in X",
            )
            ctx.expect_overlap(
                stage_carriage,
                stand,
                axes="y",
                elem_a="carriage_frame",
                elem_b="stage_plate",
                min_overlap=0.040,
                name="extended stage carriage retains lateral stage engagement",
            )
            extended_stage_pos = ctx.part_world_position(stage_carriage)
        ctx.check(
            "stage carriage moves toward +Y",
            rest_stage_pos is not None
            and extended_stage_pos is not None
            and extended_stage_pos[1] > rest_stage_pos[1] + 0.010,
            details=f"rest={rest_stage_pos}, extended={extended_stage_pos}",
        )

    ctx.expect_overlap(
        diaphragm_lever,
        stand,
        axes="xy",
        elem_a="pivot_barrel",
        elem_b="pivot_post",
        min_overlap=0.006,
        name="diaphragm lever pivot stays supported by the condenser housing",
    )
    ctx.expect_gap(
        stand,
        diaphragm_lever,
        axis="z",
        positive_elem="pivot_post",
        negative_elem="pivot_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="lever pivot seats directly beneath the condenser housing",
    )

    with ctx.pose({turret_spin: math.radians(45.0)}):
        ctx.expect_gap(
            turret,
            stand,
            axis="z",
            positive_elem="turret_hub",
            negative_elem="stage_plate",
            min_gap=0.001,
            name="objective turret clears the stage while rotating",
        )

    return ctx.report()


object_model = build_object_model()
