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


def _make_body_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.34, 0.24, 0.05)
        .edges("|Z")
        .fillet(0.026)
    )

    column = (
        cq.Workplane("XY")
        .box(0.13, 0.11, 0.28)
        .edges("|Z")
        .fillet(0.028)
        .translate((-0.10, 0.0, 0.165))
    )

    head = (
        cq.Workplane("XY")
        .box(0.32, 0.18, 0.14)
        .edges("|X")
        .fillet(0.040)
        .translate((0.03, 0.0, 0.395))
    )

    bridge = (
        cq.Workplane("XY")
        .box(0.12, 0.22, 0.05)
        .edges("|X")
        .fillet(0.016)
        .translate((-0.035, 0.0, 0.305))
    )

    nose_outer = cq.Workplane("XY").circle(0.050).extrude(0.045).translate((0.115, 0.0, 0.255))
    nose_inner = cq.Workplane("XY").circle(0.022).extrude(0.049).translate((0.115, 0.0, 0.253))
    throat_outer = cq.Workplane("XY").circle(0.036).extrude(0.030).translate((0.115, 0.0, 0.295))
    throat_inner = cq.Workplane("XY").circle(0.022).extrude(0.034).translate((0.115, 0.0, 0.293))
    nose = nose_outer.cut(nose_inner)
    throat = throat_outer.cut(throat_inner)
    rod_blank = cq.Workplane("XY").circle(0.012).extrude(0.260)
    rod_0 = rod_blank.translate((-0.045, 0.089, 0.020))
    rod_1 = rod_blank.translate((-0.045, -0.089, 0.020))

    body = base.union(column).union(head).union(bridge).union(rod_0).union(rod_1).union(nose).union(throat)

    speed_slot = cq.Workplane("XY").box(0.085, 0.050, 0.014).translate((0.035, 0.107, 0.335))
    return body.cut(speed_slot)


def _make_carriage_shape() -> cq.Workplane:
    def sleeve(y_pos: float) -> cq.Workplane:
        outer = cq.Workplane("XY").circle(0.022).extrude(0.080).translate((0.000, y_pos, -0.040))
        inner = cq.Workplane("XY").circle(0.0135).extrude(0.084).translate((0.000, y_pos, -0.042))
        return outer.cut(inner)

    carriage = sleeve(0.089).union(sleeve(-0.089))

    carriage = carriage.union(
        cq.Workplane("XY").box(0.030, 0.178, 0.026).translate((-0.032, 0.000, -0.026))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.140, 0.016, 0.018).translate((0.073, 0.160, 0.048))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.140, 0.016, 0.018).translate((0.073, -0.160, 0.048))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.022, 0.016, 0.050).translate((0.145, 0.160, 0.062))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.022, 0.016, 0.050).translate((0.145, -0.160, 0.062))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.020, 0.074, 0.026).translate((0.004, 0.126, 0.034))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.020, 0.074, 0.026).translate((0.004, -0.126, 0.034))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.024, 0.010, 0.010).translate((0.156, 0.154, 0.070))
    )
    carriage = carriage.union(
        cq.Workplane("XY").box(0.024, 0.010, 0.010).translate((0.156, -0.154, 0.070))
    )
    return carriage


def _make_bowl_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.136).extrude(0.145)
    outer = outer.edges("<Z").fillet(0.024)
    return outer.faces(">Z").shell(-0.005)


def _make_beater_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.008).extrude(0.050).translate((0.000, 0.000, -0.050))
    collar = cq.Workplane("XY").circle(0.014).extrude(0.018).translate((0.000, 0.000, -0.068))
    neck = cq.Workplane("XY").box(0.028, 0.012, 0.042).translate((0.000, 0.000, -0.089))
    blade_outer = cq.Workplane("XY").box(0.088, 0.012, 0.088).translate((0.000, 0.000, -0.126))
    blade_inner = cq.Workplane("XY").box(0.050, 0.020, 0.054).translate((0.000, 0.000, -0.126))
    lower_bar = cq.Workplane("XY").box(0.068, 0.014, 0.014).translate((0.000, 0.000, -0.163))
    return shaft.union(collar).union(neck).union(blade_outer.cut(blade_inner)).union(lower_bar)


def _make_splash_guard_shape() -> cq.Workplane:
    disc = cq.Workplane("XY").circle(0.148).extrude(0.008)
    disc = disc.cut(cq.Workplane("XY").circle(0.052).extrude(0.012).translate((0.000, 0.000, -0.002)))

    collar = cq.Workplane("XY").box(0.060, 0.072, 0.020).translate((0.102, 0.000, 0.010))
    guard = disc.union(collar)
    feed_opening = cq.Workplane("XY").box(0.042, 0.042, 0.022).translate((0.102, 0.000, 0.012))
    front_slot = cq.Workplane("XY").box(0.072, 0.048, 0.012).translate((0.110, 0.000, 0.006))
    return guard.cut(feed_opening).cut(front_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bowl_lift_stand_mixer")

    enamel = model.material("enamel", rgba=(0.94, 0.94, 0.91, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.87, 1.0))
    dark = model.material("dark", rgba=(0.15, 0.15, 0.16, 1.0))
    beater_finish = model.material("beater_finish", rgba=(0.76, 0.77, 0.74, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.82, 0.88, 0.92, 0.45))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "body_shell"),
        material=enamel,
        name="body_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage_frame"),
        material=steel,
        name="carriage_frame",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shape(), "mixing_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.012, 0.138, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lug_0",
    )
    bowl.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.012, -0.138, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lug_1",
    )

    attachment = model.part("attachment")
    attachment.visual(
        mesh_from_cadquery(_make_beater_shape(), "flat_beater"),
        material=beater_finish,
        name="beater",
    )

    splash_guard = model.part("splash_guard")
    splash_guard.visual(
        mesh_from_cadquery(_make_splash_guard_shape(), "splash_guard"),
        material=clear_guard,
        name="guard_shell",
    )
    splash_guard.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(0.074, 0.000, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clear_guard,
        name="hinge_barrel",
    )

    ingredient_flap = model.part("ingredient_flap")
    ingredient_flap.visual(
        Box((0.050, 0.040, 0.004)),
        origin=Origin(xyz=(0.025, 0.000, 0.000)),
        material=clear_guard,
        name="flap_panel",
    )
    ingredient_flap.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.006, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clear_guard,
        name="flap_hinge",
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Box((0.022, 0.010, 0.010)),
        origin=Origin(xyz=(0.011, 0.000, 0.000)),
        material=dark,
        name="lever_slider",
    )
    speed_lever.visual(
        Cylinder(radius=0.0035, length=0.024),
        origin=Origin(xyz=(0.012, 0.010, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="lever_stem",
    )
    speed_lever.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.020, 0.016, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="lever_grip",
    )

    lift = model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(-0.045, 0.000, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.080, lower=0.0, upper=0.035),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.160, 0.000, -0.025)),
    )

    model.articulation(
        "bowl_to_splash_guard",
        ArticulationType.FIXED,
        parent=bowl,
        child=splash_guard,
        origin=Origin(xyz=(0.000, 0.000, 0.146)),
    )

    model.articulation(
        "splash_guard_to_ingredient_flap",
        ArticulationType.REVOLUTE,
        parent=splash_guard,
        child=ingredient_flap,
        origin=Origin(xyz=(0.074, 0.000, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    drive = model.articulation(
        "body_to_attachment",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=attachment,
        origin=Origin(xyz=(0.115, 0.000, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    drive.meta["qc_samples"] = [0.0, math.pi / 3.0]

    model.articulation(
        "body_to_speed_lever",
        ArticulationType.PRISMATIC,
        parent=body,
        child=speed_lever,
        origin=Origin(xyz=(-0.006, 0.101, 0.335)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "body",
        "attachment",
        elem_a="body_shell",
        elem_b="beater",
        reason="The rotating beater shank is intentionally represented as retained within the fixed drive-hub socket.",
    )
    ctx.allow_overlap(
        "body",
        "carriage",
        elem_a="body_shell",
        elem_b="carriage_frame",
        reason="The bowl-lift carriage sleeves and guide frame are simplified around the lift posts and close guide casting.",
    )
    ctx.allow_overlap(
        "bowl",
        "carriage",
        elem_a="lug_0",
        elem_b="carriage_frame",
        reason="The bowl trunnion is intentionally simplified as seated into the carriage hook on the lift arm.",
    )
    ctx.allow_overlap(
        "bowl",
        "carriage",
        elem_a="lug_1",
        elem_b="carriage_frame",
        reason="The bowl trunnion is intentionally simplified as seated into the carriage hook on the lift arm.",
    )
    ctx.allow_overlap(
        "body",
        "speed_lever",
        elem_a="body_shell",
        elem_b="lever_slider",
        reason="The speed control is represented as a slider captured inside the side slot proxy.",
    )
    ctx.allow_isolated_part(
        "speed_lever",
        reason="The speed lever is modeled with a small running clearance inside the side slot proxy rather than exact sliding contact surfaces.",
    )
    ctx.allow_isolated_part(
        "splash_guard",
        reason="The removable splash guard is represented as a close-seated cover hovering slightly above the bowl rim for visual clarity.",
    )
    ctx.allow_isolated_part(
        "ingredient_flap",
        reason="The hinged ingredient flap belongs to the same removable splash-guard cover group with a tiny seating clearance over the bowl.",
    )

    bowl = object_model.get_part("bowl")
    carriage = object_model.get_part("carriage")
    attachment = object_model.get_part("attachment")
    lift = object_model.get_articulation("body_to_carriage")
    ingredient_flap = object_model.get_part("ingredient_flap")
    flap_joint = object_model.get_articulation("splash_guard_to_ingredient_flap")
    speed_lever = object_model.get_part("speed_lever")
    speed_joint = object_model.get_articulation("body_to_speed_lever")

    lift_upper = 0.035
    if lift.motion_limits is not None and lift.motion_limits.upper is not None:
        lift_upper = lift.motion_limits.upper

    with ctx.pose({lift: 0.0}):
        ctx.expect_origin_distance(
            attachment,
            bowl,
            axes="xy",
            max_dist=0.008,
            name="attachment stays centered over bowl at rest",
        )
        rest_bowl_pos = ctx.part_world_position(bowl)
        rest_carriage_pos = ctx.part_world_position(carriage)
        rest_bowl_aabb = ctx.part_world_aabb(bowl)
        rest_beater_aabb = ctx.part_world_aabb(attachment)

    with ctx.pose({lift: lift_upper}):
        ctx.expect_origin_distance(
            attachment,
            bowl,
            axes="xy",
            max_dist=0.008,
            name="attachment stays centered over bowl when raised",
        )
        raised_bowl_pos = ctx.part_world_position(bowl)
        raised_carriage_pos = ctx.part_world_position(carriage)
        raised_bowl_aabb = ctx.part_world_aabb(bowl)
        raised_beater_aabb = ctx.part_world_aabb(attachment)

    bowl_moves_up = (
        rest_bowl_pos is not None
        and raised_bowl_pos is not None
        and raised_bowl_pos[2] > rest_bowl_pos[2] + 0.025
    )
    carriage_stays_vertical = (
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and abs(raised_carriage_pos[0] - rest_carriage_pos[0]) < 1e-6
        and abs(raised_carriage_pos[1] - rest_carriage_pos[1]) < 1e-6
    )
    ctx.check(
        "bowl carriage raises vertically",
        bowl_moves_up and carriage_stays_vertical,
        details=(
            f"rest_bowl={rest_bowl_pos}, raised_bowl={raised_bowl_pos}, "
            f"rest_carriage={rest_carriage_pos}, raised_carriage={raised_carriage_pos}"
        ),
    )

    beater_sits_inside_raised_bowl = (
        raised_bowl_aabb is not None
        and raised_beater_aabb is not None
        and raised_beater_aabb[0][2] > raised_bowl_aabb[0][2] + 0.010
        and raised_beater_aabb[0][2] < raised_bowl_aabb[1][2] - 0.040
    )
    ctx.check(
        "beater remains above bowl floor at mix position",
        beater_sits_inside_raised_bowl,
        details=f"raised_bowl={raised_bowl_aabb}, raised_beater={raised_beater_aabb}",
    )

    beater_clears_lowered_bowl = (
        rest_bowl_aabb is not None
        and rest_beater_aabb is not None
        and rest_beater_aabb[0][2] > rest_bowl_aabb[0][2] + 0.030
    )
    ctx.check(
        "lowered bowl sits below the beater",
        beater_clears_lowered_bowl,
        details=f"rest_bowl={rest_bowl_aabb}, rest_beater={rest_beater_aabb}",
    )

    with ctx.pose({flap_joint: 0.0}):
        closed_flap_aabb = ctx.part_world_aabb(ingredient_flap)
    with ctx.pose({flap_joint: 1.0}):
        open_flap_aabb = ctx.part_world_aabb(ingredient_flap)
    flap_opens_up = (
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.015
    )
    ctx.check(
        "ingredient flap rotates upward",
        flap_opens_up,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    speed_upper = 0.055
    if speed_joint.motion_limits is not None and speed_joint.motion_limits.upper is not None:
        speed_upper = speed_joint.motion_limits.upper

    with ctx.pose({speed_joint: 0.0}):
        speed_rest = ctx.part_world_position(speed_lever)
    with ctx.pose({speed_joint: speed_upper}):
        speed_fast = ctx.part_world_position(speed_lever)
    speed_slides_forward = (
        speed_rest is not None
        and speed_fast is not None
        and speed_fast[0] > speed_rest[0] + 0.040
        and abs(speed_fast[1] - speed_rest[1]) < 1e-6
        and abs(speed_fast[2] - speed_rest[2]) < 1e-6
    )
    ctx.check(
        "speed lever slides along the side slot",
        speed_slides_forward,
        details=f"rest={speed_rest}, fast={speed_fast}",
    )

    return ctx.report()


object_model = build_object_model()
