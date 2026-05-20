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


BASE_RADIUS = 0.108
BASE_FRONT_Y = -0.104
SEAT_TOP_Z = 0.202
BOWL_HEIGHT = 0.151
CHIMNEY_HEIGHT = 0.154
CHIMNEY_OUTER_RADIUS = 0.0405
CHIMNEY_INNER_RADIUS = 0.0327


def _annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _make_bowl_shell() -> cq.Workplane:
    shell = _annulus(0.086, 0.074, 0.018)
    shell = shell.union(cq.Workplane("XY").workplane(offset=0.018).circle(0.089).extrude(0.133))
    shell = shell.cut(cq.Workplane("XY").workplane(offset=0.024).circle(0.082).extrude(0.121))
    shell = shell.cut(cq.Workplane("XY").workplane(offset=0.145).circle(CHIMNEY_INNER_RADIUS).extrude(0.006))
    shell = shell.cut(cq.Workplane("XY").circle(0.0125).extrude(0.030))

    lug = (
        cq.Workplane("XY")
        .box(0.022, 0.010, 0.008)
        .translate((0.074, 0.0, 0.009))
    )
    shell = shell.union(lug)
    shell = shell.union(lug.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 120.0))
    shell = shell.union(lug.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 240.0))
    return shell


def _make_feed_chimney() -> cq.Workplane:
    chimney = _annulus(CHIMNEY_OUTER_RADIUS, CHIMNEY_INNER_RADIUS, CHIMNEY_HEIGHT)
    chimney = chimney.union(
        _annulus(CHIMNEY_OUTER_RADIUS + 0.003, CHIMNEY_INNER_RADIUS, 0.006).translate((0.0, 0.0, CHIMNEY_HEIGHT))
    )
    return chimney.translate((0.0, 0.0, BOWL_HEIGHT))


def _make_blade() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.018)
        .translate((0.0, 0.0, -0.009))
    )
    upper_blade = (
        cq.Workplane("XY")
        .box(0.096, 0.016, 0.004)
        .translate((0.028, 0.0, 0.001))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 12.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 20.0)
    )
    lower_blade = (
        cq.Workplane("XY")
        .box(0.096, 0.016, 0.004)
        .translate((-0.028, 0.0, -0.001))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 200.0)
    )
    return hub.union(upper_blade).union(lower_blade)


def _make_main_pusher_body() -> cq.Workplane:
    return _annulus(0.0312, 0.0248, 0.210).translate((0.0, 0.0, -0.210))


def _make_main_pusher_cap() -> cq.Workplane:
    return _annulus(0.0365, 0.0158, 0.022)


def _make_mini_guide() -> cq.Workplane:
    return _annulus(0.0158, 0.0115, 0.208).translate((0.0, 0.0, -0.198))


def _make_mini_pusher_body() -> cq.Workplane:
    return _annulus(0.0109, 0.0078, 0.202).translate((0.0, 0.0, -0.202))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_food_processor")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.70, 0.78, 0.82, 0.36))
    frosted_clear = model.material("frosted_clear", rgba=(0.90, 0.92, 0.94, 0.72))
    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=charcoal,
        name="foot_ring",
    )
    base.visual(
        Cylinder(radius=0.102, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=matte_black,
        name="body_shell",
    )
    base.visual(
        Cylinder(radius=0.093, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=matte_black,
        name="shoulder",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        material=charcoal,
        name="deck",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, SEAT_TOP_Z - 0.009)),
        material=charcoal,
        name="seat_top",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, SEAT_TOP_Z + 0.022)),
        material=charcoal,
        name="drive_stub",
    )
    base.visual(
        Box((0.126, 0.032, 0.086)),
        origin=Origin(xyz=(0.0, -0.088, 0.090)),
        material=charcoal,
        name="control_pod",
    )
    base.visual(
        Box((0.126, 0.010, 0.072)),
        origin=Origin(xyz=(0.0, BASE_FRONT_Y + 0.005, 0.092)),
        material=charcoal,
        name="control_fascia",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shell(), "processor_bowl_shell"),
        material=clear_smoke,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(_make_feed_chimney(), "processor_feed_chimney"),
        material=clear_smoke,
        name="feed_chimney",
    )

    blade_hub = model.part("blade_hub")
    blade_hub.visual(
        mesh_from_cadquery(_make_blade(), "processor_blade_hub"),
        material=stainless,
        name="blade_assembly",
    )

    main_pusher = model.part("main_pusher")
    main_pusher.visual(
        mesh_from_cadquery(_make_main_pusher_body(), "processor_main_pusher_body"),
        material=frosted_clear,
        name="main_pusher_body",
    )
    main_pusher.visual(
        mesh_from_cadquery(_make_main_pusher_cap(), "processor_main_pusher_cap"),
        material=frosted_clear,
        name="main_top_cap",
    )
    main_pusher.visual(
        mesh_from_cadquery(_make_mini_guide(), "processor_mini_guide"),
        material=frosted_clear,
        name="mini_guide",
    )

    mini_pusher = model.part("mini_pusher")
    mini_pusher.visual(
        mesh_from_cadquery(_make_mini_pusher_body(), "processor_mini_pusher_body"),
        material=frosted_clear,
        name="mini_pusher_body",
    )
    mini_pusher.visual(
        Cylinder(radius=0.0148, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=frosted_clear,
        name="mini_pusher_cap",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="dial_skirt",
    )
    speed_dial.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="dial_cap",
    )
    speed_dial.visual(
        Box((0.003, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, 0.011)),
        material=stainless,
        name="dial_pointer",
    )

    pulse_lever = model.part("pulse_lever")
    pulse_lever.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="pivot_barrel",
    )
    pulse_lever.visual(
        Box((0.018, 0.014, 0.056)),
        origin=Origin(xyz=(0.0, -0.008, 0.031)),
        material=charcoal,
        name="lever_arm",
    )
    pulse_lever.visual(
        Box((0.030, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.014, 0.058)),
        material=matte_black,
        name="thumb_pad",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, SEAT_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0),
    )
    model.articulation(
        "base_to_blade_hub",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade_hub,
        origin=Origin(xyz=(0.0, 0.0, SEAT_TOP_Z + 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=22.0),
    )
    model.articulation(
        "bowl_to_main_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=main_pusher,
        origin=Origin(xyz=(0.0, 0.0, BOWL_HEIGHT + CHIMNEY_HEIGHT + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.18, lower=0.0, upper=0.118),
    )
    model.articulation(
        "main_pusher_to_mini_pusher",
        ArticulationType.PRISMATIC,
        parent=main_pusher,
        child=mini_pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.105),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.036, BASE_FRONT_Y, 0.098)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "base_to_pulse_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pulse_lever,
        origin=Origin(xyz=(-0.038, BASE_FRONT_Y - 0.005, 0.072)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-0.18, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    blade_hub = object_model.get_part("blade_hub")
    main_pusher = object_model.get_part("main_pusher")
    mini_pusher = object_model.get_part("mini_pusher")
    speed_dial = object_model.get_part("speed_dial")
    pulse_lever = object_model.get_part("pulse_lever")
    bowl_lock = object_model.get_articulation("base_to_bowl")
    main_slide = object_model.get_articulation("bowl_to_main_pusher")
    mini_slide = object_model.get_articulation("main_pusher_to_mini_pusher")
    pulse_joint = object_model.get_articulation("base_to_pulse_lever")

    ctx.expect_origin_distance(
        bowl,
        base,
        axes="xy",
        max_dist=0.001,
        name="bowl stays centered on the base axis",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        negative_elem="seat_top",
        max_gap=0.003,
        max_penetration=0.0,
        name="bowl seats cleanly on the locking collar",
    )
    ctx.expect_origin_distance(
        blade_hub,
        bowl,
        axes="xy",
        max_dist=0.001,
        name="blade hub shares the bowl center axis",
    )

    with ctx.pose({bowl_lock: 0.55}):
        ctx.expect_origin_distance(
            bowl,
            base,
            axes="xy",
            max_dist=0.001,
            name="twisted bowl remains coaxial with the base",
        )
        ctx.expect_origin_distance(
            blade_hub,
            bowl,
            axes="xy",
            max_dist=0.001,
            name="twisted bowl remains coaxial with the blade hub",
        )

    ctx.expect_within(
        main_pusher,
        bowl,
        axes="xy",
        inner_elem="main_pusher_body",
        outer_elem="feed_chimney",
        margin=0.0015,
        name="main pusher stays centered in the feed chimney",
    )
    ctx.expect_overlap(
        main_pusher,
        bowl,
        axes="z",
        elem_a="main_pusher_body",
        elem_b="feed_chimney",
        min_overlap=0.090,
        name="main pusher remains inserted at rest",
    )

    main_rest = ctx.part_world_position(main_pusher)
    with ctx.pose({main_slide: 0.118}):
        ctx.expect_within(
            main_pusher,
            bowl,
            axes="xy",
            inner_elem="main_pusher_body",
            outer_elem="feed_chimney",
            margin=0.0015,
            name="main pusher stays centered when raised",
        )
        ctx.expect_overlap(
            main_pusher,
            bowl,
            axes="z",
            elem_a="main_pusher_body",
            elem_b="feed_chimney",
            min_overlap=0.028,
            name="main pusher retains insertion when raised",
        )
        main_raised = ctx.part_world_position(main_pusher)

    ctx.check(
        "main pusher lifts upward",
        main_rest is not None and main_raised is not None and main_raised[2] > main_rest[2] + 0.05,
        details=f"rest={main_rest}, raised={main_raised}",
    )

    ctx.expect_within(
        mini_pusher,
        main_pusher,
        axes="xy",
        inner_elem="mini_pusher_body",
        outer_elem="mini_guide",
        margin=0.0012,
        name="mini pusher stays centered in the inner guide",
    )
    ctx.expect_overlap(
        mini_pusher,
        main_pusher,
        axes="z",
        elem_a="mini_pusher_body",
        elem_b="mini_guide",
        min_overlap=0.090,
        name="mini pusher remains inserted at rest",
    )

    mini_rest = ctx.part_world_position(mini_pusher)
    with ctx.pose({mini_slide: 0.105}):
        ctx.expect_within(
            mini_pusher,
            main_pusher,
            axes="xy",
            inner_elem="mini_pusher_body",
            outer_elem="mini_guide",
            margin=0.0012,
            name="mini pusher stays centered when raised",
        )
        ctx.expect_overlap(
            mini_pusher,
            main_pusher,
            axes="z",
            elem_a="mini_pusher_body",
            elem_b="mini_guide",
            min_overlap=0.030,
            name="mini pusher retains insertion when raised",
        )
        mini_raised = ctx.part_world_position(mini_pusher)

    ctx.check(
        "mini pusher lifts upward",
        mini_rest is not None and mini_raised is not None and mini_raised[2] > mini_rest[2] + 0.04,
        details=f"rest={mini_rest}, raised={mini_raised}",
    )

    ctx.expect_origin_distance(
        speed_dial,
        pulse_lever,
        axes="x",
        min_dist=0.050,
        max_dist=0.090,
        name="front controls sit beside each other",
    )

    pulse_rest = ctx.part_element_world_aabb(pulse_lever, elem="thumb_pad")
    with ctx.pose({pulse_joint: 0.32}):
        pulse_pressed = ctx.part_element_world_aabb(pulse_lever, elem="thumb_pad")

    pulse_moved_inward = (
        pulse_rest is not None
        and pulse_pressed is not None
        and pulse_pressed[1][1] > pulse_rest[1][1] + 0.010
    )
    ctx.check(
        "pulse lever pivots inward on its front hinge",
        pulse_moved_inward,
        details=f"rest={pulse_rest}, pressed={pulse_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
