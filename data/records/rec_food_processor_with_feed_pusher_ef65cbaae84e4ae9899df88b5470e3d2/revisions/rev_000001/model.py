from __future__ import annotations

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


BASE_W = 0.182
BASE_D = 0.148
BASE_H = 0.092
BASE_CORNER_R = 0.016
COLLAR_R = 0.058
COLLAR_H = 0.014

BOWL_H = 0.108
BOWL_OR = 0.074
BOWL_WALL = 0.0032
BOWL_BOTTOM = 0.0045

LID_TH = 0.008
LID_OR = 0.081
TUBE_OR = 0.027
TUBE_IR = 0.0215
TUBE_H = 0.060
TUBE_X = 0.036
TUBE_Y = 0.016

PUSHER_R = 0.0192
PUSHER_L = 0.086
PUSHER_CAP_R = 0.029
PUSHER_CAP_H = 0.012
PUSHER_TRAVEL = 0.032

KNOB_X = -0.026
KNOB_Z = 0.046
BUTTON_X = 0.040
BUTTON_ZS = (0.057, 0.035)


def _make_base_shape():
    body = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BASE_CORNER_R)
        .edges(">Z")
        .fillet(0.010)
    )

    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .extrude(COLLAR_H)
        .edges(">Z")
        .fillet(0.004)
        .translate((0.0, 0.0, BASE_H))
    )

    knob_hole = (
        cq.Workplane("XY")
        .circle(0.0062)
        .extrude(0.020)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((KNOB_X, -BASE_D / 2.0 + 0.020, KNOB_Z))
    )

    button_slots = []
    for z in BUTTON_ZS:
        face_pocket = cq.Workplane("XY").box(0.024, 0.008, 0.014).translate(
            (BUTTON_X, -BASE_D / 2.0 + 0.004, z)
        )
        stem_pocket = cq.Workplane("XY").box(0.014, 0.024, 0.014).translate(
            (BUTTON_X, -BASE_D / 2.0 + 0.012, z)
        )
        button_slots.append(face_pocket.union(stem_pocket))

    base = body.union(collar).cut(knob_hole)
    for slot in button_slots:
        base = base.cut(slot)
    return base


def _make_bowl_shape():
    shell = cq.Workplane("XY").circle(BOWL_OR).extrude(BOWL_H)
    lip = cq.Workplane("XY").circle(BOWL_OR + 0.004).extrude(0.009).translate(
        (0.0, 0.0, BOWL_H - 0.009)
    )
    seat = cq.Workplane("XY").circle(0.060).extrude(0.012)
    center_sleeve = cq.Workplane("XY").circle(0.015).extrude(0.020)
    cavity = cq.Workplane("XY").circle(BOWL_OR - BOWL_WALL).extrude(BOWL_H - BOWL_BOTTOM).translate(
        (0.0, 0.0, BOWL_BOTTOM)
    )
    center_bore = cq.Workplane("XY").circle(0.0105).extrude(0.022).translate((0.0, 0.0, -0.001))

    handle = (
        cq.Workplane("XY")
        .box(0.024, 0.032, 0.082, centered=(True, True, False))
        .translate((BOWL_OR + 0.008, 0.0, 0.018))
        .edges("|Z")
        .fillet(0.006)
    )

    bowl = shell.union(lip).union(seat).union(center_sleeve).cut(cavity).cut(center_bore)
    return bowl.union(handle)


def _make_lid_plate_shape():
    plate = cq.Workplane("XY").circle(LID_OR).extrude(LID_TH)
    feed_opening = cq.Workplane("XY").circle(TUBE_IR).extrude(LID_TH + 0.002).translate(
        (TUBE_X, TUBE_Y, -0.001)
    )
    return plate.cut(feed_opening)


def _make_feed_tube_shape():
    outer = cq.Workplane("XY").circle(TUBE_OR).extrude(TUBE_H)
    inner = cq.Workplane("XY").circle(TUBE_IR).extrude(TUBE_H + 0.002).translate((0.0, 0.0, -0.001))
    return outer.cut(inner).edges(">Z").fillet(0.002)


def _make_blade_shape():
    spindle = cq.Workplane("XY").circle(0.0065).extrude(0.026)
    hub = cq.Workplane("XY").circle(0.012).extrude(0.006).translate((0.0, 0.0, 0.018))

    blade_arm = (
        cq.Workplane("XY")
        .box(0.056, 0.014, 0.0024)
        .translate((0.028, 0.0, 0.027))
        .edges("|Z")
        .fillet(0.003)
    )
    blade_a = blade_arm.rotate((0.0, 0.0, 0.027), (1.0, 0.0, 0.027), 8.0)
    blade_b = blade_arm.rotate((0.0, 0.0, 0.027), (0.0, 1.0, 0.027), 180.0).rotate(
        (0.0, 0.0, 0.027), (1.0, 0.0, 0.027), -8.0
    )
    return hub.union(spindle).union(blade_a).union(blade_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_food_processor")

    base_plastic = model.material("base_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    clear_plastic = model.material("clear_plastic", rgba=(0.84, 0.88, 0.92, 0.30))
    clear_lid = model.material("clear_lid", rgba=(0.88, 0.91, 0.95, 0.28))
    blade_metal = model.material("blade_metal", rgba=(0.83, 0.84, 0.86, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    button_finish = model.material("button_finish", rgba=(0.90, 0.91, 0.93, 1.0))
    pusher_finish = model.material("pusher_finish", rgba=(0.93, 0.94, 0.95, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "processor_base"),
        material=base_plastic,
        name="base_shell",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shape(), "processor_bowl"),
        material=clear_plastic,
        name="bowl_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_plate_shape(), "processor_lid_plate"),
        material=clear_lid,
        name="lid_plate",
    )
    lid.visual(
        mesh_from_cadquery(_make_feed_tube_shape(), "processor_feed_tube"),
        origin=Origin(xyz=(TUBE_X, TUBE_Y, LID_TH)),
        material=clear_lid,
        name="feed_tube",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=PUSHER_R, length=PUSHER_L),
        origin=Origin(xyz=(0.0, 0.0, -PUSHER_L / 2.0)),
        material=pusher_finish,
        name="plunger",
    )
    pusher.visual(
        Cylinder(radius=PUSHER_CAP_R, length=PUSHER_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, PUSHER_CAP_H / 2.0)),
        material=pusher_finish,
        name="cap",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_make_blade_shape(), "processor_blade"),
        material=blade_metal,
        name="blade_shell",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(1.5708, 0.0, 0.0)),
        material=knob_finish,
        name="knob_body",
    )
    timer_knob.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(1.5708, 0.0, 0.0)),
        material=knob_finish,
        name="knob_shaft",
    )
    timer_knob.visual(
        Box((0.003, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, 0.010)),
        material=button_finish,
        name="indicator",
    )

    button_parts = []
    for index, z in enumerate(BUTTON_ZS):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.022, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=button_finish,
            name="button_stem",
        )
        button.visual(
            Box((0.014, 0.012, 0.014)),
            origin=Origin(xyz=(0.0, 0.014, 0.0)),
            material=button_finish,
            name="button_guide",
        )
        button_parts.append((button, z))

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLLAR_H)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BOWL_H)),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(TUBE_X, TUBE_Y, LID_TH + TUBE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.15,
            lower=0.0,
            upper=PUSHER_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLLAR_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )
    model.articulation(
        "base_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_knob,
        origin=Origin(xyz=(KNOB_X, -BASE_D / 2.0, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    for index, (button, z) in enumerate(button_parts):
        model.articulation(
            f"base_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(BUTTON_X, -BASE_D / 2.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.10,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    blade = object_model.get_part("blade")
    timer_knob = object_model.get_part("timer_knob")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")

    pusher_joint = object_model.get_articulation("lid_to_pusher")
    button_joint_0 = object_model.get_articulation("base_to_mode_button_0")
    button_joint_1 = object_model.get_articulation("base_to_mode_button_1")

    ctx.allow_overlap(
        base,
        button_0,
        elem_a="base_shell",
        elem_b="button_guide",
        reason="The mode button rides in a simplified internal guide pocket inside the motor base.",
    )
    ctx.allow_overlap(
        base,
        button_1,
        elem_a="base_shell",
        elem_b="button_guide",
        reason="The mode button rides in a simplified internal guide pocket inside the motor base.",
    )

    ctx.expect_origin_gap(
        bowl,
        base,
        axis="z",
        min_gap=BASE_H + COLLAR_H - 0.001,
        max_gap=BASE_H + COLLAR_H + 0.001,
        name="bowl is seated on the motor base",
    )
    ctx.expect_within(
        blade,
        bowl,
        axes="xy",
        elem_a="blade_shell",
        elem_b="bowl_shell",
        margin=0.0,
        name="blade footprint stays inside the bowl",
    )
    ctx.expect_gap(
        lid,
        blade,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="blade_shell",
        min_gap=0.060,
        name="lid clears the chopping blade",
    )
    ctx.expect_origin_gap(
        base,
        timer_knob,
        axis="y",
        min_gap=0.070,
        max_gap=0.078,
        name="timer knob sits on the front face",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="plunger",
        outer_elem="feed_tube",
        margin=0.0,
        name="pusher stays centered in the feed tube",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="plunger",
        elem_b="feed_tube",
        min_overlap=0.055,
        name="resting pusher remains inserted in the chute",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    if pusher_joint.motion_limits is not None and pusher_joint.motion_limits.upper is not None:
        with ctx.pose({pusher_joint: pusher_joint.motion_limits.upper}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="plunger",
                outer_elem="feed_tube",
                margin=0.0,
                name="raised pusher stays centered in the feed tube",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="plunger",
                elem_b="feed_tube",
                min_overlap=0.028,
                name="raised pusher keeps retained insertion",
            )
            raised_pusher_pos = ctx.part_world_position(pusher)
        ctx.check(
            "pusher lifts upward",
            rest_pusher_pos is not None
            and raised_pusher_pos is not None
            and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.025,
            details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
        )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    if button_joint_0.motion_limits is not None and button_joint_0.motion_limits.upper is not None:
        with ctx.pose({button_joint_0: button_joint_0.motion_limits.upper}):
            pressed_button_0 = ctx.part_world_position(button_0)
            unchanged_button_1 = ctx.part_world_position(button_1)
        ctx.check(
            "mode button 0 presses inward",
            rest_button_0 is not None
            and pressed_button_0 is not None
            and pressed_button_0[1] > rest_button_0[1] + 0.002,
            details=f"rest={rest_button_0}, pressed={pressed_button_0}",
        )
        ctx.check(
            "mode button 1 stays independent when button 0 is pressed",
            rest_button_1 is not None
            and unchanged_button_1 is not None
            and abs(unchanged_button_1[1] - rest_button_1[1]) < 1e-6,
            details=f"rest={rest_button_1}, during_button_0_press={unchanged_button_1}",
        )

    if button_joint_1.motion_limits is not None and button_joint_1.motion_limits.upper is not None:
        with ctx.pose({button_joint_1: button_joint_1.motion_limits.upper}):
            pressed_button_1 = ctx.part_world_position(button_1)
        ctx.check(
            "mode button 1 presses inward",
            rest_button_1 is not None
            and pressed_button_1 is not None
            and pressed_button_1[1] > rest_button_1[1] + 0.002,
            details=f"rest={rest_button_1}, pressed={pressed_button_1}",
        )

    return ctx.report()


object_model = build_object_model()
