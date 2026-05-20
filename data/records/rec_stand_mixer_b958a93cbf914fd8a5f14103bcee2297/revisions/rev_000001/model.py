from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_COLOR = (0.86, 0.79, 0.67, 1.0)
STEEL_COLOR = (0.86, 0.87, 0.90, 1.0)
DARK_COLOR = (0.12, 0.11, 0.10, 1.0)
CLEAR_COLOR = (0.82, 0.90, 0.92, 0.35)

BODY_HINGE_X = -0.020
BODY_HINGE_Z = 0.246
BOWL_CENTER_X = 0.125
BOWL_BASE_Z = 0.102
GUARD_HINGE_X = 0.000
GUARD_HINGE_Z = 0.227


def _body_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(0.300, 0.190, 0.045)
        .translate((0.020, 0.000, 0.0225))
        .edges("|Z")
        .fillet(0.020)
        .edges(">Z")
        .fillet(0.010)
    )

    bowl_pedestal = (
        cq.Workplane("XY")
        .circle(0.048)
        .extrude(0.045)
        .translate((BOWL_CENTER_X, 0.000, 0.045))
        .faces(">Z")
        .edges()
        .fillet(0.008)
    )

    bowl_plate = (
        cq.Workplane("XY")
        .circle(0.044)
        .extrude(0.012)
        .translate((BOWL_CENTER_X, 0.000, 0.090))
    )

    rear_column = (
        cq.Workplane("XY")
        .box(0.112, 0.112, 0.170)
        .translate((-0.066, 0.000, 0.130))
        .edges("|Z")
        .fillet(0.024)
    )

    neck = (
        cq.Workplane("XY")
        .box(0.100, 0.086, 0.048)
        .translate((-0.020, 0.000, 0.206))
        .edges("|Z")
        .fillet(0.015)
    )

    guard_post = (
        cq.Workplane("XY")
        .box(0.016, 0.032, 0.074)
        .translate((-0.022, 0.000, 0.144))
    )

    guard_cap = (
        cq.Workplane("XY")
        .box(0.012, 0.016, 0.024)
        .translate((-0.016, 0.000, 0.186))
    )

    left_ear = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.050)
        .translate((BODY_HINGE_X, 0.042, BODY_HINGE_Z))
        .edges("|Z")
        .fillet(0.004)
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.050)
        .translate((BODY_HINGE_X, -0.042, BODY_HINGE_Z))
        .edges("|Z")
        .fillet(0.004)
    )

    return (
        foot.union(bowl_pedestal)
        .union(bowl_plate)
        .union(rear_column)
        .union(neck)
        .union(guard_post)
        .union(guard_cap)
        .union(left_ear)
        .union(right_ear)
    )


def _head_shape() -> cq.Workplane:
    hinge_knuckle = cq.Workplane("XZ").circle(0.012).extrude(0.060, both=True)

    main_shell = (
        cq.Workplane("XY")
        .box(0.205, 0.135, 0.096, centered=(False, True, True))
        .translate((0.015, 0.000, 0.058))
        .edges("|X")
        .fillet(0.024)
        .edges("|Y")
        .fillet(0.022)
    )

    rear_spine = (
        cq.Workplane("XY")
        .box(0.040, 0.050, 0.030, centered=(False, True, True))
        .translate((0.004, 0.000, 0.024))
    )

    top_hump = (
        cq.Workplane("XY")
        .box(0.124, 0.110, 0.042, centered=(False, True, True))
        .translate((0.048, 0.000, 0.110))
        .edges("|Y")
        .fillet(0.018)
        .edges("|X")
        .fillet(0.016)
    )

    nose_cap = (
        cq.Workplane("YZ")
        .circle(0.036)
        .extrude(0.040)
        .translate((0.220, 0.000, 0.034))
    )

    beater_boss = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.038)
        .translate((0.145, 0.000, -0.028))
    )

    return hinge_knuckle.union(rear_spine).union(main_shell).union(top_hump).union(nose_cap).union(beater_boss)


def _bowl_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.028)
        .workplane(offset=0.010)
        .circle(0.036)
        .workplane(offset=0.012)
        .circle(0.052)
        .workplane(offset=0.040)
        .circle(0.082)
        .workplane(offset=0.036)
        .circle(0.103)
        .workplane(offset=0.015)
        .circle(0.114)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(0.011)
        .workplane(offset=0.009)
        .circle(0.024)
        .workplane(offset=0.012)
        .circle(0.045)
        .workplane(offset=0.039)
        .circle(0.074)
        .workplane(offset=0.035)
        .circle(0.096)
        .workplane(offset=0.013)
        .circle(0.106)
        .loft(combine=True)
    )

    rim = cq.Workplane("XY").circle(0.118).circle(0.110).extrude(0.004).translate((0.000, 0.000, 0.111))
    return outer.cut(inner).union(rim)


def _guard_shape() -> cq.Workplane:
    guard_panel = (
        cq.Workplane("XY")
        .center(0.120, 0.000)
        .circle(0.110)
        .circle(0.055)
        .extrude(0.003, both=True)
        .intersect(
            cq.Workplane("XY")
            .box(0.255, 0.240, 0.008, centered=(False, True, True))
        )
    )

    hinge_barrel = cq.Workplane("XZ").circle(0.006).extrude(0.058, both=True)
    left_arm = cq.Workplane("XY").box(0.064, 0.012, 0.006).translate((0.032, 0.025, 0.000))
    right_arm = cq.Workplane("XY").box(0.064, 0.012, 0.006).translate((0.032, -0.025, 0.000))

    return guard_panel.union(hinge_barrel).union(left_arm).union(right_arm)


def _knob_shape() -> cq.Workplane:
    skirt = cq.Workplane("XZ").circle(0.022).extrude(0.006)
    cap = cq.Workplane("XZ").circle(0.018).extrude(0.018)
    return skirt.union(cap).translate((0.000, 0.018, 0.000))


def _attachment_shape() -> cq.Workplane:
    top_collar = cq.Workplane("XY").circle(0.009).extrude(0.010).translate((0.000, 0.000, -0.010))
    shaft = cq.Workplane("XY").circle(0.0035).extrude(0.030).translate((0.000, 0.000, -0.040))
    beater_arm = cq.Workplane("XY").box(0.016, 0.006, 0.010).translate((0.006, 0.000, -0.040))
    frame_outer = cq.Workplane("XZ").rect(0.042, 0.072).extrude(0.006, both=True).translate((0.010, 0.000, -0.074))
    frame_inner = cq.Workplane("XZ").rect(0.026, 0.050).extrude(0.010, both=True).translate((0.012, 0.000, -0.074))
    frame = frame_outer.cut(frame_inner)
    lower_blade = cq.Workplane("XY").box(0.028, 0.006, 0.010).translate((0.012, 0.000, -0.097))
    return top_collar.union(shaft).union(beater_arm).union(frame).union(lower_blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_tilt_head_stand_mixer")

    cream = model.material("cream", rgba=BODY_COLOR)
    steel = model.material("steel", rgba=STEEL_COLOR)
    dark = model.material("dark", rgba=DARK_COLOR)
    clear = model.material("clear_guard", rgba=CLEAR_COLOR)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "mixer_body"),
        material=cream,
        name="body_shell",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shape(), "mixing_bowl"),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=steel,
        name="bowl_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "mixer_head"),
        material=cream,
        name="head_shell",
    )

    knob = model.part("speed_knob")
    knob.visual(
        mesh_from_cadquery(_knob_shape(), "speed_knob"),
        material=dark,
        name="knob_cap",
    )

    splash_guard = model.part("splash_guard")
    splash_guard.visual(
        mesh_from_cadquery(_guard_shape(), "splash_guard"),
        material=clear,
        name="guard_panel",
    )

    attachment = model.part("attachment")
    attachment.visual(
        mesh_from_cadquery(_attachment_shape(), "beater_attachment"),
        material=steel,
        name="beater",
    )

    model.articulation(
        "body_to_bowl",
        ArticulationType.FIXED,
        parent=body,
        child=bowl,
        origin=Origin(xyz=(BOWL_CENTER_X, 0.000, BOWL_BASE_Z)),
    )
    model.articulation(
        "body_to_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(BODY_HINGE_X, 0.000, BODY_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=12.0, velocity=1.6),
    )
    model.articulation(
        "head_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=knob,
        origin=Origin(xyz=(0.088, 0.0675, 0.066)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "body_to_splash_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=splash_guard,
        origin=Origin(xyz=(GUARD_HINGE_X, 0.000, GUARD_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=1.2, velocity=2.5),
    )
    model.articulation(
        "head_to_attachment",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=attachment,
        origin=Origin(xyz=(0.145, 0.000, -0.028)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    knob = object_model.get_part("speed_knob")
    splash_guard = object_model.get_part("splash_guard")
    attachment = object_model.get_part("attachment")

    head_hinge = object_model.get_articulation("body_to_head")
    guard_hinge = object_model.get_articulation("body_to_splash_guard")
    knob_joint = object_model.get_articulation("head_to_speed_knob")
    attachment_joint = object_model.get_articulation("head_to_attachment")

    ctx.allow_overlap(
        body,
        bowl,
        elem_a="body_shell",
        elem_b="bowl_shell",
        reason="The bowl sits on a simplified support hub beneath a watertight bowl shell proxy.",
    )
    ctx.allow_overlap(
        attachment,
        bowl,
        elem_a="beater",
        elem_b="bowl_shell",
        reason="The beater is intentionally authored inside the bowl cavity, and the exact overlap pass treats the closed bowl shell as solid.",
    )
    ctx.allow_overlap(
        body,
        head,
        elem_a="body_shell",
        elem_b="head_shell",
        reason="The rear trunnion and cheek plates are represented with simplified solid hinge geometry at the tilt pivot.",
    )
    ctx.allow_overlap(
        body,
        splash_guard,
        elem_a="body_shell",
        elem_b="guard_panel",
        reason="The splash guard hinge uses a simplified closed-barrel pivot against the rear bracket.",
    )

    with ctx.pose({head_hinge: 0.0, guard_hinge: 0.0}):
        ctx.expect_overlap(
            head,
            bowl,
            axes="xy",
            min_overlap=0.090,
            name="closed head spans over the bowl",
        )
        ctx.expect_gap(
            splash_guard,
            bowl,
            axis="z",
            max_penetration=0.001,
            max_gap=0.030,
            name="closed splash guard sits just above the bowl rim",
        )
        ctx.expect_overlap(
            splash_guard,
            bowl,
            axes="xy",
            min_overlap=0.100,
            name="closed splash guard covers the bowl opening",
        )

    rest_attachment = ctx.part_world_position(attachment)
    with ctx.pose({head_hinge: 1.00}):
        open_attachment = ctx.part_world_position(attachment)

    rest_guard_center = _aabb_center(ctx.part_element_world_aabb(splash_guard, elem="guard_panel"))
    with ctx.pose({guard_hinge: 1.20}):
        open_guard_center = _aabb_center(ctx.part_element_world_aabb(splash_guard, elem="guard_panel"))

    ctx.check(
        "tilt head raises the beater axis",
        rest_attachment is not None
        and open_attachment is not None
        and open_attachment[2] > rest_attachment[2] + 0.100,
        details=f"rest={rest_attachment}, open={open_attachment}",
    )
    ctx.check(
        "splash guard flips upward above the bowl",
        rest_guard_center is not None
        and open_guard_center is not None
        and open_guard_center[2] > rest_guard_center[2] + 0.070,
        details=f"rest={rest_guard_center}, open={open_guard_center}",
    )
    ctx.check(
        "speed knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )
    ctx.check(
        "attachment spins continuously",
        attachment_joint.articulation_type == ArticulationType.CONTINUOUS
        and attachment_joint.motion_limits is not None
        and attachment_joint.motion_limits.lower is None
        and attachment_joint.motion_limits.upper is None,
        details=f"type={attachment_joint.articulation_type}, limits={attachment_joint.motion_limits}",
    )
    ctx.expect_origin_gap(
        knob,
        head,
        axis="y",
        min_gap=0.020,
        max_gap=0.120,
        name="speed knob sits outboard on the mixer side",
    )

    return ctx.report()


object_model = build_object_model()
