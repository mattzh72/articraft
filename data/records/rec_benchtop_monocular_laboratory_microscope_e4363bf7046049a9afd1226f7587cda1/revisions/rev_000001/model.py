from __future__ import annotations

from math import radians

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


BASE_LENGTH = 0.240
BASE_WIDTH = 0.180
BASE_HEIGHT = 0.028

STAGE_JOINT_XYZ = (-0.044, 0.0, 0.136)
BODY_JOINT_XYZ = (-0.060, 0.0, 0.176)
TURRET_JOINT_XYZ = (0.099, 0.0, 0.042)
STAGE_GUIDE_XYZ = (0.090, 0.072, 0.038)
CONDENSER_JOINT_XYZ = (0.058, 0.0, 0.021)
LEVER_PIVOT_XYZ = (0.018, 0.0, -0.025)


def _stand_shell_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.018)
    )
    arm_profile = [
        (-0.104, 0.018),
        (-0.068, 0.018),
        (-0.060, 0.070),
        (-0.050, 0.165),
        (-0.040, 0.255),
        (-0.032, 0.300),
        (-0.024, 0.286),
        (-0.032, 0.220),
        (-0.042, 0.128),
        (-0.056, 0.048),
        (-0.076, 0.018),
    ]
    arm = cq.Workplane("XZ").polyline(arm_profile).close().extrude(0.050).translate((0.0, -0.025, 0.0))
    return base.union(arm)


def _stage_deck_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.112, 0.102, 0.008)
        .translate((0.064, 0.0, 0.025))
    )
    plate = plate.cut(
        cq.Workplane("XY")
        .box(0.038, 0.038, 0.016)
        .translate((0.058, 0.0, 0.025))
    )

    neck = (
        cq.Workplane("XY")
        .box(0.038, 0.048, 0.014)
        .translate((0.024, 0.0, 0.017))
    )
    return plate.union(neck)

def _stage_guide_shape() -> cq.Workplane:
    shelf = cq.Workplane("XY").box(0.094, 0.022, 0.006).translate((0.090, 0.060, 0.024))
    rod = (
        cq.Workplane("XY")
        .circle(0.0053)
        .extrude(0.086)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.047, 0.072, 0.038))
    )

    rear_boss = (
        cq.Workplane("XY")
        .box(0.010, 0.018, 0.010)
        .translate((0.049, 0.072, 0.038))
    )
    front_boss = (
        cq.Workplane("XY")
        .box(0.010, 0.018, 0.010)
        .translate((0.131, 0.072, 0.038))
    )
    rear_post = cq.Workplane("XY").box(0.006, 0.010, 0.018).translate((0.049, 0.068, 0.031))
    front_post = cq.Workplane("XY").box(0.006, 0.010, 0.018).translate((0.131, 0.068, 0.031))

    return shelf.union(rod).union(rear_boss).union(front_boss).union(rear_post).union(front_post)


def _condenser_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.014).extrude(0.003).translate((0.0, 0.0, -0.003))
    housing = cq.Workplane("XY").circle(0.015).extrude(0.024).translate((0.0, 0.0, -0.027))
    lower_collar = cq.Workplane("XY").circle(0.010).extrude(0.010).translate((0.0, 0.0, -0.037))
    lug = cq.Workplane("XY").box(0.014, 0.006, 0.008).translate((0.011, 0.0, -0.023))
    return flange.union(housing).union(lower_collar).union(lug)


def _body_shell_shape() -> cq.Workplane:
    clamp = (
        cq.Workplane("XY")
        .box(0.050, 0.056, 0.038)
        .translate((0.0, 0.0, 0.019))
    )
    clamp = clamp.cut(
        cq.Workplane("XY")
        .circle(0.0167)
        .extrude(0.044)
        .translate((0.0, 0.0, -0.003))
    )
    clamp = clamp.cut(
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.042)
        .translate((-0.017, 0.0, 0.019))
    )

    mast = cq.Workplane("XY").box(0.018, 0.040, 0.070).translate((0.032, 0.0, 0.073))
    connector = cq.Workplane("XY").box(0.030, 0.040, 0.020).translate((0.052, 0.0, 0.088))
    head = cq.Workplane("XY").box(0.052, 0.050, 0.036).translate((0.072, 0.0, 0.108))
    nosepiece_stem = cq.Workplane("XY").circle(0.0088).extrude(0.046).translate((0.099, 0.0, 0.046))
    nosepiece_collar = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.005)
        .cut(cq.Workplane("XY").circle(0.0086).extrude(0.007))
        .translate((0.099, 0.0, 0.042))
    )

    tube = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(0.110)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -32.0)
        .translate((0.072, 0.0, 0.104))
    )
    eyepiece = (
        cq.Workplane("XY")
        .circle(0.0125)
        .extrude(0.028)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -32.0)
        .translate((0.072 - 0.058, 0.0, 0.104 + 0.093))
    )

    return (
        clamp.union(mast)
        .union(connector)
        .union(head)
        .union(nosepiece_stem)
        .union(nosepiece_collar)
        .union(tube)
        .union(eyepiece)
    )


def _turret_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.010))
    )
    body = body.cut(
        cq.Workplane("XY")
        .circle(0.0096)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.012))
    )

    objective_origins = [
        (0.0135, 0.0),
        (-0.00675, 0.01169),
        (-0.00675, -0.01169),
    ]
    lengths = (0.022, 0.027, 0.019)
    for (x_pos, y_pos), length in zip(objective_origins, lengths):
        upper_barrel = (
            cq.Workplane("XY")
            .circle(0.0055)
            .extrude(0.010)
            .translate((x_pos, y_pos, -0.020))
        )
        lower_barrel = (
            cq.Workplane("XY")
            .circle(0.0038)
            .extrude(length)
            .translate((x_pos, y_pos, -0.020 - length))
        )
        body = body.union(upper_barrel).union(lower_barrel)

    return body


def _carriage_shape() -> cq.Workplane:
    clamp = (
        cq.Workplane("XY")
        .box(0.024, 0.020, 0.022)
        .translate((0.0, 0.0, 0.0))
    )
    clamp = clamp.cut(
        cq.Workplane("XY")
        .circle(0.0058)
        .extrude(0.030)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((-0.015, 0.0, 0.0))
    )

    over_stage_arm = cq.Workplane("XY").box(0.040, 0.032, 0.006).translate((0.006, -0.018, 0.010))
    clip_lip = cq.Workplane("XY").box(0.008, 0.016, 0.012).translate((0.018, -0.034, 0.013))
    thumb_tab = cq.Workplane("XY").box(0.010, 0.014, 0.010).translate((-0.014, 0.012, 0.010))

    return clamp.union(over_stage_arm).union(clip_lip).union(thumb_tab)


def _lever_shape() -> cq.Workplane:
    eye = (
        cq.Workplane("XY")
        .circle(0.0056)
        .extrude(0.003)
        .translate((0.0, 0.0, -0.0015))
    )
    eye = eye.cut(
        cq.Workplane("XY")
        .circle(0.0032)
        .extrude(0.005)
        .translate((0.0, 0.0, -0.0025))
    )

    arm = cq.Workplane("XY").box(0.034, 0.006, 0.003).translate((0.019, 0.0, -0.0015))
    handle = cq.Workplane("XY").circle(0.0038).extrude(0.009).translate((0.036, 0.0, -0.006))
    return eye.union(arm).union(handle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_microscope")

    model.material("enamel_ivory", rgba=(0.86, 0.86, 0.82, 1.0))
    model.material("stage_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("optics_silver", rgba=(0.67, 0.69, 0.73, 1.0))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shell_shape(), "microscope_stand_shell"),
        material="enamel_ivory",
        name="stand_shell",
    )
    stand.visual(
        Box((0.036, 0.050, 0.108)),
        origin=Origin(xyz=(-0.044, 0.0, 0.082)),
        material="enamel_ivory",
        name="stage_support",
    )
    stand.visual(
        Cylinder(radius=0.0165, length=0.210),
        origin=Origin(xyz=(-0.060, 0.0, 0.193)),
        material="optics_silver",
        name="focus_column",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.022, 0.038, 0.022)),
        origin=Origin(xyz=(0.010, 0.0, 0.011)),
        material="stage_black",
        name="stage_mount",
    )
    stage.visual(
        mesh_from_cadquery(_stage_deck_shape(), "microscope_stage_plate"),
        material="stage_black",
        name="stage_plate",
    )
    stage.visual(
        mesh_from_cadquery(_stage_guide_shape(), "microscope_stage_guide"),
        material="stage_black",
        name="stage_guide",
    )

    condenser = model.part("condenser")
    condenser.visual(
        mesh_from_cadquery(_condenser_shape(), "microscope_condenser"),
        material="matte_black",
        name="housing",
    )
    condenser.visual(
        Cylinder(radius=0.0028, length=0.006),
        origin=Origin(xyz=(0.018, 0.0, -0.020)),
        material="optics_silver",
        name="pivot_pin",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "microscope_body_shell"),
        material="enamel_ivory",
        name="body_shell",
    )

    turret = model.part("turret")
    turret.visual(
        mesh_from_cadquery(_turret_shape(), "microscope_turret"),
        material="optics_silver",
        name="turret",
    )

    side_carriage = model.part("side_carriage")
    side_carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "microscope_side_carriage"),
        material="stage_black",
        name="carriage",
    )

    diaphragm_lever = model.part("diaphragm_lever")
    diaphragm_lever.visual(
        mesh_from_cadquery(_lever_shape(), "microscope_diaphragm_lever"),
        material="knob_black",
        name="lever",
    )

    model.articulation(
        "stand_to_stage",
        ArticulationType.FIXED,
        parent=stand,
        child=stage,
        origin=Origin(xyz=STAGE_JOINT_XYZ),
    )
    model.articulation(
        "stage_to_condenser",
        ArticulationType.FIXED,
        parent=stage,
        child=condenser,
        origin=Origin(xyz=CONDENSER_JOINT_XYZ),
    )
    model.articulation(
        "stand_to_body",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=body,
        origin=Origin(xyz=BODY_JOINT_XYZ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.10,
            lower=0.0,
            upper=0.070,
        ),
    )
    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turret,
        origin=Origin(xyz=TURRET_JOINT_XYZ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=10.0,
        ),
    )
    model.articulation(
        "stage_to_side_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=side_carriage,
        origin=Origin(xyz=STAGE_GUIDE_XYZ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "condenser_to_diaphragm_lever",
        ArticulationType.REVOLUTE,
        parent=condenser,
        child=diaphragm_lever,
        origin=Origin(xyz=LEVER_PIVOT_XYZ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-radians(28.0),
            upper=radians(28.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage")
    condenser = object_model.get_part("condenser")
    body = object_model.get_part("body")
    turret = object_model.get_part("turret")
    side_carriage = object_model.get_part("side_carriage")
    diaphragm_lever = object_model.get_part("diaphragm_lever")

    focus_slide = object_model.get_articulation("stand_to_body")
    turret_joint = object_model.get_articulation("body_to_turret")
    carriage_slide = object_model.get_articulation("stage_to_side_carriage")
    lever_pivot = object_model.get_articulation("condenser_to_diaphragm_lever")

    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="stage_support",
        min_gap=0.009,
        name="stage plate stays clearly above the stand support",
    )

    ctx.expect_gap(
        stage,
        diaphragm_lever,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="lever",
        min_gap=0.008,
        name="diaphragm lever remains below the stage plate",
    )

    ctx.check(
        "objective turret uses continuous rotation",
        turret_joint.articulation_type == ArticulationType.CONTINUOUS
        and turret_joint.motion_limits is not None
        and turret_joint.motion_limits.lower is None
        and turret_joint.motion_limits.upper is None,
        details=str(turret_joint.motion_limits),
    )

    lower_focus = focus_slide.motion_limits.lower if focus_slide.motion_limits is not None else None
    upper_focus = focus_slide.motion_limits.upper if focus_slide.motion_limits is not None else None
    if lower_focus is not None and upper_focus is not None:
        with ctx.pose({focus_slide: lower_focus}):
            ctx.expect_gap(
                turret,
                stage,
                axis="z",
                negative_elem="stage_plate",
                min_gap=0.0055,
                name="lowest objective position still clears the stage",
            )
            lower_body_pos = ctx.part_world_position(body)

        with ctx.pose({focus_slide: upper_focus}):
            upper_body_pos = ctx.part_world_position(body)

        ctx.check(
            "body slides upward on the focus column",
            lower_body_pos is not None
            and upper_body_pos is not None
            and upper_body_pos[2] > lower_body_pos[2] + 0.05,
            details=f"lower={lower_body_pos}, upper={upper_body_pos}",
        )

    lower_carriage = carriage_slide.motion_limits.lower if carriage_slide.motion_limits is not None else None
    upper_carriage = carriage_slide.motion_limits.upper if carriage_slide.motion_limits is not None else None
    if lower_carriage is not None and upper_carriage is not None:
        with ctx.pose({carriage_slide: lower_carriage}):
            lower_carriage_pos = ctx.part_world_position(side_carriage)
            ctx.expect_overlap(
                side_carriage,
                stage,
                axes="x",
                elem_b="stage_guide",
                min_overlap=0.024,
                name="side carriage remains captured over the stage travel",
            )
        with ctx.pose({carriage_slide: upper_carriage}):
            upper_carriage_pos = ctx.part_world_position(side_carriage)

        ctx.check(
            "side carriage moves along the stage guide",
            lower_carriage_pos is not None
            and upper_carriage_pos is not None
            and upper_carriage_pos[0] > lower_carriage_pos[0] + 0.03,
            details=f"lower={lower_carriage_pos}, upper={upper_carriage_pos}",
        )

    lower_lever = lever_pivot.motion_limits.lower if lever_pivot.motion_limits is not None else None
    upper_lever = lever_pivot.motion_limits.upper if lever_pivot.motion_limits is not None else None
    if lower_lever is not None and upper_lever is not None:
        with ctx.pose({lever_pivot: lower_lever}):
            lower_lever_aabb = ctx.part_world_aabb(diaphragm_lever)
        with ctx.pose({lever_pivot: upper_lever}):
            upper_lever_aabb = ctx.part_world_aabb(diaphragm_lever)

        ctx.check(
            "diaphragm lever swings under the condenser housing",
            lower_lever_aabb is not None
            and upper_lever_aabb is not None
            and upper_lever_aabb[1][1] > lower_lever_aabb[1][1] + 0.01,
            details=f"lower={lower_lever_aabb}, upper={upper_lever_aabb}",
        )

    ctx.check(
        "diaphragm lever is mounted on the condenser",
        lever_pivot.parent == condenser.name,
        details=f"parent={lever_pivot.parent}",
    )
    ctx.allow_overlap(
        condenser,
        diaphragm_lever,
        elem_a="housing",
        elem_b="lever",
        reason="The diaphragm lever eye is intentionally modeled as a close-fitting pivot around the condenser-side support lug.",
    )
    ctx.allow_overlap(
        body,
        turret,
        elem_a="body_shell",
        elem_b="turret",
        reason="The objective turret is intentionally represented as a closely captured nosepiece hub under the body shell.",
    )

    return ctx.report()


object_model = build_object_model()
