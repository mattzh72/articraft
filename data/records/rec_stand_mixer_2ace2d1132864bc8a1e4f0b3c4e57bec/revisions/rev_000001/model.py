from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    place_on_surface,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def _make_body_shell() -> cq.Workplane:
    base = _rounded_box((0.34, 0.42, 0.06), 0.026).translate((0.0, 0.0, 0.03))
    column = _rounded_box((0.13, 0.20, 0.29), 0.020).translate((-0.085, 0.0, 0.190))
    shoulder = _rounded_box((0.21, 0.18, 0.15), 0.030).translate((0.015, 0.0, 0.350))
    head = _rounded_box((0.18, 0.16, 0.12), 0.024).translate((0.155, 0.0, 0.365))
    nose = _rounded_box((0.07, 0.13, 0.09), 0.018).translate((0.235, 0.0, 0.335))
    drive_collar = (
        cq.Workplane("XY")
        .circle(0.036)
        .extrude(0.032)
        .translate((0.180, 0.0, 0.266))
    )

    return base.union(column).union(shoulder).union(head).union(nose).union(drive_collar)


def _make_bowl_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.085))
        .circle(0.105)
        .extrude(0.170)
    )
    bowl = outer.faces(">Z").shell(-0.003)
    foot = (
        cq.Workplane("XY")
        .circle(0.042)
        .extrude(0.006)
        .translate((0.0, 0.0, -0.091))
    )
    rim = (
        cq.Workplane("XY")
        .circle(0.111)
        .circle(0.104)
        .extrude(0.005)
        .translate((0.0, 0.0, 0.085))
    )
    return bowl.union(foot).union(rim)


def _make_bowl_lift() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.240, 0.265, 0.018)
        .translate((0.0, 0.0, -0.096))
        .cut(cq.Workplane("XY").box(0.160, 0.195, 0.028).translate((0.020, 0.0, -0.096)))
        .cut(cq.Workplane("XY").box(0.125, 0.285, 0.040).translate((-0.095, 0.0, -0.096)))
        .union(cq.Workplane("XY").box(0.050, 0.110, 0.032).translate((0.098, 0.0, -0.088)))
        .union(cq.Workplane("XY").box(0.028, 0.040, 0.028).translate((0.030, -0.078, -0.091)))
        .union(cq.Workplane("XY").box(0.028, 0.040, 0.028).translate((0.030, 0.078, -0.091)))
    )


def _make_lift_arm() -> cq.Workplane:
    bar = cq.Workplane("XY").box(0.018, 0.018, 0.210)
    shoe = cq.Workplane("XY").box(0.022, 0.030, 0.044).translate((0.018, 0.0, 0.090))
    foot = cq.Workplane("XY").box(0.032, 0.024, 0.040).translate((-0.006, 0.0, -0.085))
    return bar.union(shoe).union(foot)


def _make_beater() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.008).extrude(0.036).translate((0.0, 0.0, -0.036))
    hub = cq.Workplane("XY").circle(0.014).extrude(0.016).translate((0.0, 0.0, -0.052))
    frame = (
        cq.Workplane("XY")
        .box(0.065, 0.012, 0.066)
        .translate((0.0, 0.0, -0.085))
        .cut(cq.Workplane("XY").box(0.038, 0.018, 0.034).translate((0.0, 0.0, -0.085)))
    )
    crossbar = cq.Workplane("XY").box(0.045, 0.012, 0.012).translate((0.0, 0.0, -0.055))
    return shaft.union(hub).union(frame).union(crossbar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bowl_lift_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.86, 0.83, 0.75, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    beater_finish = model.material("beater_finish", rgba=(0.72, 0.72, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_body_shell(), "body_shell"),
        material=body_finish,
        name="body_shell",
    )
    base.visual(
        Box((0.160, 0.120, 0.004)),
        origin=Origin(xyz=(0.015, 0.0, 0.062)),
        material=dark_trim,
        name="base_trim",
    )

    bowl_lift = model.part("bowl_lift")
    bowl_lift.visual(
        mesh_from_cadquery(_make_bowl_lift(), "bowl_lift"),
        material=steel,
        name="lift_frame",
    )
    bowl_lift.visual(
        Box((0.018, 0.018, 0.225)),
        origin=Origin(xyz=(-0.010, -0.122, 0.025)),
        material=steel,
        name="arm_bar_0",
    )
    bowl_lift.visual(
        Box((0.038, 0.030, 0.050)),
        origin=Origin(xyz=(0.018, -0.122, 0.119)),
        material=steel,
        name="arm_shoe_0",
    )
    bowl_lift.visual(
        Box((0.018, 0.018, 0.225)),
        origin=Origin(xyz=(-0.010, 0.122, 0.025)),
        material=steel,
        name="arm_bar_1",
    )
    bowl_lift.visual(
        Box((0.038, 0.030, 0.050)),
        origin=Origin(xyz=(0.018, 0.122, 0.119)),
        material=steel,
        name="arm_shoe_1",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shell(), "bowl"),
        material=steel,
        name="bowl_shell",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.0040, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_trim,
        name="dial_shaft",
    )
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.016,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.045, 0.004, flare=0.050),
                grip=KnobGrip(style="fluted", count=18, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_dial",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_trim,
        name="dial_knob",
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_cadquery(_make_beater(), "beater"),
        material=beater_finish,
        name="beater_frame",
    )

    model.articulation(
        "base_to_bowl_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_lift,
        origin=Origin(xyz=(0.170, 0.0, 0.165)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=0.0, upper=0.085),
    )

    model.articulation(
        "bowl_lift_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_lift,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "base_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_dial,
        origin=place_on_surface(
            timer_dial,
            base,
            point_hint=(0.165, 0.100, 0.340),
            child_axis="+z",
            clearance=0.0,
            spin=math.pi / 2.0,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=12.0),
    )

    model.articulation(
        "base_to_beater",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=beater,
        origin=Origin(xyz=(0.180, 0.0, 0.266)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl_lift = object_model.get_part("bowl_lift")
    bowl = object_model.get_part("bowl")
    timer_dial = object_model.get_part("timer_dial")
    beater = object_model.get_part("beater")

    lift_joint = object_model.get_articulation("base_to_bowl_lift")
    dial_joint = object_model.get_articulation("base_to_timer_dial")
    beater_joint = object_model.get_articulation("base_to_beater")

    ctx.allow_overlap(
        base,
        timer_dial,
        elem_a="body_shell",
        elem_b="dial_shaft",
        reason="The timer dial shaft is intentionally represented as seated in the solid housing wall proxy.",
    )
    ctx.allow_overlap(
        base,
        beater,
        elem_a="body_shell",
        elem_b="beater_frame",
        reason="The beater shaft is intentionally represented as inserting into the solid drive-collar proxy.",
    )

    lift_upper = 0.0
    if lift_joint.motion_limits is not None and lift_joint.motion_limits.upper is not None:
        lift_upper = lift_joint.motion_limits.upper

    ctx.check(
        "bowl lift uses a vertical prismatic joint",
        lift_joint.articulation_type == ArticulationType.PRISMATIC and tuple(lift_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift_joint.articulation_type}, axis={lift_joint.axis}",
    )
    ctx.check(
        "timer dial rotates continuously",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.check(
        "beater spins continuously",
        beater_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(beater_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={beater_joint.articulation_type}, axis={beater_joint.axis}",
    )

    ctx.expect_origin_gap(
        timer_dial,
        base,
        axis="y",
        min_gap=0.070,
        name="timer dial sits on the housing side",
    )
    ctx.expect_overlap(
        bowl,
        beater,
        axes="xy",
        min_overlap=0.020,
        name="beater stays centered over the bowl opening",
    )

    rest_bowl_pos = ctx.part_world_position(bowl)
    rest_lift_pos = ctx.part_world_position(bowl_lift)

    with ctx.pose({lift_joint: lift_upper}):
        raised_bowl_pos = ctx.part_world_position(bowl)
        raised_lift_pos = ctx.part_world_position(bowl_lift)
        ctx.expect_overlap(
            bowl,
            beater,
            axes="xy",
            min_overlap=0.020,
            name="raised bowl remains centered under the beater",
        )

    ctx.check(
        "bowl cradle rises noticeably",
        rest_bowl_pos is not None
        and raised_bowl_pos is not None
        and raised_bowl_pos[2] > rest_bowl_pos[2] + 0.060,
        details=f"rest={rest_bowl_pos}, raised={raised_bowl_pos}",
    )
    ctx.check(
        "lift frame carries the bowl upward",
        rest_lift_pos is not None
        and raised_lift_pos is not None
        and raised_lift_pos[2] > rest_lift_pos[2] + 0.060,
        details=f"rest={rest_lift_pos}, raised={raised_lift_pos}",
    )

    return ctx.report()


object_model = build_object_model()
