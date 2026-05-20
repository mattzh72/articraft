from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _build_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").rect(0.37, 0.29).extrude(0.105)
    rear_column = cq.Workplane("XY").center(-0.11, 0.0).rect(0.11, 0.17).extrude(0.315)
    head_shell = (
        cq.Workplane("XZ")
        .moveTo(-0.12, 0.305)
        .lineTo(0.18, 0.315)
        .lineTo(0.13, 0.42)
        .lineTo(-0.03, 0.42)
        .lineTo(-0.10, 0.35)
        .close()
        .extrude(0.085, both=True)
    )
    drive_hub = cq.Workplane("XY").center(0.11, 0.0).circle(0.028).extrude(0.024).translate((0.0, 0.0, 0.321))
    return base.union(rear_column).union(head_shell).union(drive_hub)


def _build_lift_cradle_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(0.29, 0.28)
        .extrude(0.012)
        .faces(">Z")
        .workplane()
        .circle(0.07)
        .cutThruAll()
        .union(cq.Workplane("XY").center(-0.14, 0.0).rect(0.03, 0.22).extrude(0.145))
        .union(cq.Workplane("XY").center(0.125, 0.0).rect(0.028, 0.17).extrude(0.022))
        .union(cq.Workplane("XY").center(0.0, -0.136).rect(0.034, 0.022).extrude(0.17))
        .union(cq.Workplane("XY").center(0.0, 0.136).rect(0.034, 0.022).extrude(0.17))
    )


def _build_speed_dial_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.006)
        .faces(">Z")
        .workplane()
        .circle(0.015)
        .extrude(0.010)
    )


def _build_bowl_geometry():
    return LatheGeometry.from_shell_profiles(
        [
            (0.048, 0.000),
            (0.060, 0.010),
            (0.087, 0.035),
            (0.106, 0.090),
            (0.118, 0.145),
            (0.112, 0.155),
        ],
        [
            (0.032, 0.012),
            (0.050, 0.024),
            (0.078, 0.048),
            (0.099, 0.094),
            (0.109, 0.142),
            (0.103, 0.149),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def _build_dough_hook_geometry():
    blade = sweep_profile_along_spline(
        [
            (0.000, 0.000, -0.024),
            (0.000, 0.000, -0.066),
            (0.014, 0.000, -0.100),
            (0.034, 0.000, -0.126),
            (0.022, 0.000, -0.145),
            (-0.004, 0.000, -0.138),
        ],
        profile=rounded_rect_profile(0.018, 0.007, radius=0.002),
        samples_per_segment=16,
        up_hint=(0.0, 1.0, 0.0),
    )
    tip = CylinderGeometry(0.0055, 0.026, radial_segments=18).rotate_y(-0.65).translate(0.010, 0.0, -0.139)
    blade.merge(tip)
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_bowl_lift_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.92, 0.90, 0.85, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.80, 0.81, 0.83, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.74, 0.75, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=body_finish,
        name="shell",
    )

    lift_cradle = model.part("lift_cradle")
    lift_cradle.visual(
        mesh_from_cadquery(_build_lift_cradle_shape(), "lift_cradle"),
        material=dark_trim,
        name="frame",
    )
    lift_cradle.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_trim,
        name="platform_pad",
    )
    lift_cradle.visual(
        Box((0.07, 0.018, 0.004)),
        origin=Origin(xyz=(-0.015, -0.126, 0.0)),
        material=dark_trim,
        name="guide_0",
    )
    lift_cradle.visual(
        Box((0.07, 0.018, 0.004)),
        origin=Origin(xyz=(-0.015, 0.126, 0.0)),
        material=dark_trim,
        name="guide_1",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(_build_bowl_geometry(), "mixing_bowl"),
        material=steel,
        name="shell",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_cadquery(_build_speed_dial_shape(), "speed_dial"),
        material=dark_trim,
        name="dial",
    )
    speed_dial.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_trim,
        name="shaft",
    )

    dough_hook = model.part("dough_hook")
    dough_hook.visual(
        Cylinder(radius=0.009, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=polished_steel,
        name="stem",
    )
    dough_hook.visual(
        mesh_from_geometry(_build_dough_hook_geometry(), "dough_hook"),
        material=polished_steel,
        name="blade",
    )

    model.articulation(
        "body_to_lift_cradle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lift_cradle,
        origin=Origin(xyz=(0.11, 0.0, 0.107)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.03, effort=180.0, velocity=0.18),
    )
    model.articulation(
        "lift_cradle_to_bowl",
        ArticulationType.FIXED,
        parent=lift_cradle,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=(-0.11, 0.093, 0.294), rpy=(-pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_dough_hook",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dough_hook,
        origin=Origin(xyz=(0.11, 0.0, 0.321)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lift_cradle = object_model.get_part("lift_cradle")
    bowl = object_model.get_part("bowl")
    speed_dial = object_model.get_part("speed_dial")
    dough_hook = object_model.get_part("dough_hook")

    lift_joint = object_model.get_articulation("body_to_lift_cradle")
    dial_joint = object_model.get_articulation("body_to_speed_dial")
    hook_joint = object_model.get_articulation("body_to_dough_hook")

    ctx.allow_overlap(
        body,
        lift_cradle,
        elem_a="shell",
        elem_b="guide_0",
        reason="The lower guide shoe is intentionally simplified as riding directly on the body guide surface.",
    )
    ctx.allow_overlap(
        body,
        lift_cradle,
        elem_a="shell",
        elem_b="guide_1",
        reason="The upper guide shoe is intentionally simplified as riding directly on the body guide surface.",
    )
    ctx.allow_overlap(
        body,
        speed_dial,
        elem_a="shell",
        elem_b="shaft",
        reason="The speed dial uses a short side shaft that intentionally seats into the body sidewall.",
    )
    ctx.allow_overlap(
        body,
        dough_hook,
        elem_a="shell",
        elem_b="stem",
        reason="The dough hook stem intentionally seats into the mixer drive hub.",
    )

    ctx.expect_origin_distance(
        bowl,
        dough_hook,
        axes="xy",
        max_dist=0.008,
        name="dough hook stays centered over the bowl",
    )
    ctx.expect_origin_distance(
        bowl,
        lift_cradle,
        axes="xy",
        max_dist=0.001,
        name="bowl stays centered on the lift cradle",
    )
    ctx.expect_overlap(
        bowl,
        lift_cradle,
        axes="xy",
        min_overlap=0.10,
        name="bowl stays within the lift cradle footprint",
    )
    ctx.expect_origin_gap(
        speed_dial,
        body,
        axis="y",
        min_gap=0.075,
        max_gap=0.095,
        name="speed dial sits on the right flank",
    )

    rest_bowl = ctx.part_world_position(bowl)
    rest_lift = ctx.part_world_position(lift_cradle)
    upper = 0.0
    if lift_joint.motion_limits is not None and lift_joint.motion_limits.upper is not None:
        upper = lift_joint.motion_limits.upper

    with ctx.pose({lift_joint: upper}):
        raised_bowl = ctx.part_world_position(bowl)
        raised_lift = ctx.part_world_position(lift_cradle)
        ctx.expect_origin_distance(
            bowl,
            dough_hook,
            axes="xy",
            max_dist=0.008,
            name="raised bowl stays aligned with the dough hook",
        )
        ctx.expect_overlap(
            bowl,
            lift_cradle,
            axes="xy",
            min_overlap=0.10,
            name="raised bowl remains carried by the lift cradle",
        )

    bowl_rise = None
    lift_rise = None
    if rest_bowl is not None and raised_bowl is not None:
        bowl_rise = raised_bowl[2] - rest_bowl[2]
    if rest_lift is not None and raised_lift is not None:
        lift_rise = raised_lift[2] - rest_lift[2]

    ctx.check(
        "lift cradle raises the bowl vertically",
        bowl_rise is not None and bowl_rise > 0.03,
        details=f"bowl_rise={bowl_rise}",
    )
    ctx.check(
        "lift cradle moves as one supported assembly",
        bowl_rise is not None and lift_rise is not None and abs(bowl_rise - lift_rise) < 1e-4,
        details=f"bowl_rise={bowl_rise}, lift_rise={lift_rise}",
    )
    ctx.check(
        "speed dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"type={dial_joint.articulation_type}, limits={dial_joint.motion_limits}",
    )
    ctx.check(
        "dough hook is continuous",
        hook_joint.articulation_type == ArticulationType.CONTINUOUS
        and hook_joint.motion_limits is not None
        and hook_joint.motion_limits.lower is None
        and hook_joint.motion_limits.upper is None,
        details=f"type={hook_joint.articulation_type}, limits={hook_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
