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


def _rounded_rect_tube(
    *,
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
):
    outer = (
        cq.Workplane("XY")
        .box(outer_x, outer_y, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(outer_radius)
    )
    inner = (
        cq.Workplane("XY")
        .box(inner_x, inner_y, height + 0.006, centered=(True, True, False))
        .edges("|Z")
        .fillet(inner_radius)
        .translate((0.0, 0.0, -0.003))
    )
    return outer.cut(inner)


def _cylindrical_shell(*, outer_radius: float, inner_radius: float, height: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.006).translate((0.0, 0.0, -0.003))
    return outer.cut(inner)


def _build_body_shape():
    base = (
        cq.Workplane("XY")
        .box(0.240, 0.188, 0.064)
        .edges("|Z")
        .fillet(0.016)
        .translate((0.0, 0.0, 0.032))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.082, 0.040, 0.040)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.105, 0.084))
    )
    tower = (
        cq.Workplane("XY")
        .box(0.156, 0.102, 0.180)
        .edges("|Z")
        .fillet(0.024)
        .translate((0.0, 0.140, 0.154))
    )
    bowl_mount = cq.Workplane("XY").circle(0.034).extrude(0.058).translate((0.0, -0.040, 0.064))
    rear_cap = cq.Workplane("XY").circle(0.050).extrude(0.016).translate((0.0, 0.145, 0.236))
    bowl_clearance = cq.Workplane("XY").circle(0.112).extrude(0.170).translate((0.0, -0.040, 0.122))
    body = base.union(bridge).union(tower).union(rear_cap).cut(bowl_clearance)
    return body.union(bowl_mount)


def _build_bowl_shape():
    shell = cq.Workplane("XY").circle(0.110).extrude(0.175).faces(">Z").shell(-0.007)
    lip = cq.Workplane("XY").circle(0.114).circle(0.103).extrude(0.010).translate((0.0, 0.0, 0.165))
    shaft_bore = cq.Workplane("XY").circle(0.011).extrude(0.024).translate((0.0, 0.0, -0.001))
    rear_flat = cq.Workplane("XY").box(0.260, 0.130, 0.190).translate((0.0, 0.125, 0.095))
    handle_block = cq.Workplane("XY").box(0.050, 0.026, 0.110).translate((0.130, 0.0, 0.095))
    handle_cut = cq.Workplane("XY").box(0.028, 0.032, 0.070).translate((0.136, 0.0, 0.095))
    handle = handle_block.cut(handle_cut)
    return shell.cut(shaft_bore).cut(rear_flat).union(lip).union(handle)


def _build_lid_shape():
    top_plate = cq.Workplane("XY").circle(0.115).extrude(0.012)
    main_opening = (
        cq.Workplane("XY")
        .box(0.076, 0.056, 0.030, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.011)
        .translate((0.015, 0.000, -0.009))
    )
    small_opening = cq.Workplane("XY").circle(0.0165).extrude(0.032).translate((0.082, 0.040, -0.010))
    return top_plate.cut(main_opening).cut(small_opening)


def _build_disc_shape():
    plate = cq.Workplane("XY").circle(0.085).circle(0.0046).extrude(0.004).translate((0.0, 0.0, -0.002))
    hub = cq.Workplane("XY").circle(0.020).circle(0.0046).extrude(0.016).translate((0.0, 0.0, -0.004))
    blade_0 = cq.Workplane("XY").box(0.040, 0.010, 0.003).translate((0.044, 0.0, 0.002))
    blade_1 = cq.Workplane("XY").box(0.040, 0.010, 0.003).translate((-0.044, 0.0, 0.002))
    blade_2 = cq.Workplane("XY").box(0.010, 0.036, 0.003).translate((0.018, 0.046, 0.002))
    return plate.union(hub).union(blade_0).union(blade_1).union(blade_2)


def _build_main_pusher_shape():
    plunger = (
        cq.Workplane("XY")
        .box(0.074, 0.054, 0.122)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, -0.061))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.088, 0.068, 0.018)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, 0.009))
    )
    grip = cq.Workplane("XY").box(0.028, 0.018, 0.014).translate((0.0, 0.0, 0.025))
    return plunger.union(cap).union(grip)


def _build_small_pusher_shape():
    plunger = cq.Workplane("XY").circle(0.015).extrude(0.114).translate((0.0, 0.0, -0.114))
    cap = cq.Workplane("XY").circle(0.020).extrude(0.016)
    knob = cq.Workplane("XY").circle(0.010).extrude(0.010).translate((0.0, 0.0, 0.016))
    return plunger.union(cap).union(knob)


def _build_control_strip_shape():
    return (
        cq.Workplane("XY")
        .box(0.092, 0.008, 0.090)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, 0.045))
    )


def _build_ring_dial_shape():
    ring = cq.Workplane("XY").circle(0.028).circle(0.017).extrude(0.008)
    finger_tab = cq.Workplane("XY").box(0.008, 0.014, 0.010).translate((0.022, 0.0, 0.004))
    return ring.union(finger_tab).translate((0.0, 0.0, -0.004))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_food_processor")

    body_white = model.material("body_white", rgba=(0.90, 0.90, 0.88, 1.0))
    control_black = model.material("control_black", rgba=(0.13, 0.14, 0.15, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    start_green = model.material("start_green", rgba=(0.26, 0.52, 0.28, 1.0))
    stop_red = model.material("stop_red", rgba=(0.58, 0.19, 0.18, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.86, 0.90, 0.34))
    clear_frost = model.material("clear_frost", rgba=(0.88, 0.92, 0.95, 0.52))
    steel = model.material("steel", rgba=(0.79, 0.81, 0.83, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.32, 0.33, 0.35, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shape(), "body_shell"), material=body_white, name="body_shell")

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(_build_bowl_shape(), "bowl_shell"), material=clear_smoke, name="bowl_shell")

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid_shape(), "lid_shell"), material=clear_smoke, name="lid_shell")

    main_tube = model.part("main_tube")
    main_tube.visual(
        mesh_from_cadquery(
            _rounded_rect_tube(
                outer_x=0.084,
                outer_y=0.064,
                inner_x=0.076,
                inner_y=0.056,
                height=0.118,
                outer_radius=0.013,
                inner_radius=0.009,
            ),
            "main_tube_shell",
        ),
        material=clear_smoke,
        name="main_tube_shell",
    )

    small_tube = model.part("small_tube")
    small_tube.visual(
        mesh_from_cadquery(
            _cylindrical_shell(outer_radius=0.020, inner_radius=0.016, height=0.105),
            "small_tube_shell",
        ),
        material=clear_smoke,
        name="small_tube_shell",
    )

    main_pusher = model.part("main_pusher")
    main_pusher.visual(
        Box((0.070, 0.050, 0.122)),
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
        material=clear_frost,
        name="main_plunger",
    )
    main_pusher.visual(
        Box((0.086, 0.066, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=clear_frost,
        name="main_cap",
    )
    main_pusher.visual(
        Box((0.028, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=clear_frost,
        name="main_grip",
    )

    small_pusher = model.part("small_pusher")
    small_pusher.visual(
        Cylinder(radius=0.0145, length=0.114),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=clear_frost,
        name="small_plunger",
    )
    small_pusher.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=clear_frost,
        name="small_cap",
    )
    small_pusher.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=clear_frost,
        name="small_knob",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.0047, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_grey,
        name="shaft_post",
    )

    disc = model.part("disc")
    disc.visual(mesh_from_cadquery(_build_disc_shape(), "cutter_disc"), material=steel, name="disc_plate")

    control_strip = model.part("control_strip")
    control_strip.visual(
        mesh_from_cadquery(_build_control_strip_shape(), "control_strip"),
        material=control_black,
        name="control_strip_shell",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_ring_dial_shape(), "ring_dial"),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=dial_dark,
        name="ring_dial",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.024, 0.004, 0.015)),
        origin=Origin(xyz=(0.0, -0.002, 0.0075)),
        material=start_green,
        name="start_cap",
    )
    start_button.visual(
        Box((0.010, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.001, 0.0075)),
        material=start_green,
        name="start_stem",
    )

    stop_button = model.part("stop_button")
    stop_button.visual(
        Box((0.024, 0.004, 0.015)),
        origin=Origin(xyz=(0.0, -0.002, 0.0075)),
        material=stop_red,
        name="stop_cap",
    )
    stop_button.visual(
        Box((0.010, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.001, 0.0075)),
        material=stop_red,
        name="stop_stem",
    )

    model.articulation(
        "body_to_bowl",
        ArticulationType.FIXED,
        parent=body,
        child=bowl,
        origin=Origin(xyz=(0.0, -0.040, 0.122)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )
    model.articulation(
        "lid_to_main_tube",
        ArticulationType.FIXED,
        parent=lid,
        child=main_tube,
        origin=Origin(xyz=(0.015, 0.000, 0.012)),
    )
    model.articulation(
        "lid_to_small_tube",
        ArticulationType.FIXED,
        parent=lid,
        child=small_tube,
        origin=Origin(xyz=(0.082, 0.040, 0.012)),
    )
    model.articulation(
        "main_tube_to_main_pusher",
        ArticulationType.PRISMATIC,
        parent=main_tube,
        child=main_pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.070),
    )
    model.articulation(
        "small_tube_to_small_pusher",
        ArticulationType.PRISMATIC,
        parent=small_tube,
        child=small_pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=0.0, upper=0.055),
    )
    model.articulation(
        "body_to_shaft",
        ArticulationType.FIXED,
        parent=body,
        child=shaft,
        origin=Origin(xyz=(0.0, -0.040, 0.122)),
    )
    model.articulation(
        "shaft_to_disc",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )
    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(0.0, -0.098, 0.012)),
    )
    model.articulation(
        "control_strip_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_strip,
        child=dial,
        origin=Origin(xyz=(0.0, -0.010, 0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )
    model.articulation(
        "control_strip_to_start_button",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=start_button,
        origin=Origin(xyz=(0.0, -0.007, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.0010),
    )
    model.articulation(
        "control_strip_to_stop_button",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=stop_button,
        origin=Origin(xyz=(0.0, -0.007, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.0010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    main_tube = object_model.get_part("main_tube")
    small_tube = object_model.get_part("small_tube")
    main_pusher = object_model.get_part("main_pusher")
    small_pusher = object_model.get_part("small_pusher")
    disc = object_model.get_part("disc")
    shaft = object_model.get_part("shaft")
    start_button = object_model.get_part("start_button")
    stop_button = object_model.get_part("stop_button")

    main_slide = object_model.get_articulation("main_tube_to_main_pusher")
    small_slide = object_model.get_articulation("small_tube_to_small_pusher")
    start_press = object_model.get_articulation("control_strip_to_start_button")
    stop_press = object_model.get_articulation("control_strip_to_stop_button")
    disc_spin = object_model.get_articulation("shaft_to_disc")
    dial_spin = object_model.get_articulation("control_strip_to_dial")

    ctx.allow_overlap(
        "body",
        "shaft",
        elem_a="body_shell",
        elem_b="shaft_post",
        reason="The drive shaft intentionally emerges through the motor housing socket.",
    )
    ctx.allow_overlap(
        "disc",
        "shaft",
        elem_a="disc_plate",
        elem_b="shaft_post",
        reason="The cutter disc hub is intentionally represented as a close rotational fit on the drive shaft.",
    )
    ctx.allow_overlap(
        "body",
        "bowl",
        elem_a="body_shell",
        elem_b="bowl_shell",
        reason="The bowl is intentionally simplified as a thin clear shell nesting over the motor collar and rear locking interface.",
    )

    ctx.expect_contact(lid, bowl, elem_a="lid_shell", elem_b="bowl_shell", name="lid seats on bowl lip")
    ctx.expect_contact(main_tube, lid, elem_a="main_tube_shell", elem_b="lid_shell", name="main tube mounts on lid")
    ctx.expect_contact(small_tube, lid, elem_a="small_tube_shell", elem_b="lid_shell", name="small tube mounts on lid")

    ctx.expect_within(
        main_pusher,
        main_tube,
        axes="xy",
        inner_elem="main_plunger",
        outer_elem="main_tube_shell",
        margin=0.004,
        name="main pusher stays centered in main tube",
    )
    ctx.expect_within(
        small_pusher,
        small_tube,
        axes="xy",
        inner_elem="small_plunger",
        outer_elem="small_tube_shell",
        margin=0.004,
        name="small pusher stays centered in small tube",
    )
    ctx.expect_overlap(
        main_pusher,
        main_tube,
        axes="z",
        elem_a="main_plunger",
        elem_b="main_tube_shell",
        min_overlap=0.110,
        name="main pusher remains deeply inserted at rest",
    )
    ctx.expect_overlap(
        small_pusher,
        small_tube,
        axes="z",
        elem_a="small_plunger",
        elem_b="small_tube_shell",
        min_overlap=0.100,
        name="small pusher remains deeply inserted at rest",
    )

    main_limits = main_slide.motion_limits
    if main_limits is not None and main_limits.upper is not None:
        main_rest = ctx.part_world_position(main_pusher)
        with ctx.pose({main_slide: main_limits.upper}):
            ctx.expect_within(
                main_pusher,
                main_tube,
                axes="xy",
                inner_elem="main_plunger",
                outer_elem="main_tube_shell",
                margin=0.004,
                name="main pusher stays centered when pushed",
            )
            ctx.expect_overlap(
                main_pusher,
                main_tube,
                axes="z",
                elem_a="main_plunger",
                elem_b="main_tube_shell",
                min_overlap=0.040,
                name="main pusher retains insertion at full press",
            )
            main_pressed = ctx.part_world_position(main_pusher)
        ctx.check(
            "main pusher moves downward",
            main_rest is not None and main_pressed is not None and main_pressed[2] < main_rest[2] - 0.050,
            details=f"rest={main_rest}, pressed={main_pressed}",
        )

    small_limits = small_slide.motion_limits
    if small_limits is not None and small_limits.upper is not None:
        small_rest = ctx.part_world_position(small_pusher)
        with ctx.pose({small_slide: small_limits.upper}):
            ctx.expect_within(
                small_pusher,
                small_tube,
                axes="xy",
                inner_elem="small_plunger",
                outer_elem="small_tube_shell",
                margin=0.004,
                name="small pusher stays centered when pushed",
            )
            ctx.expect_overlap(
                small_pusher,
                small_tube,
                axes="z",
                elem_a="small_plunger",
                elem_b="small_tube_shell",
                min_overlap=0.040,
                name="small pusher retains insertion at full press",
            )
            small_pressed = ctx.part_world_position(small_pusher)
        ctx.check(
            "small pusher moves downward",
            small_rest is not None and small_pressed is not None and small_pressed[2] < small_rest[2] - 0.035,
            details=f"rest={small_rest}, pressed={small_pressed}",
        )

    start_limits = start_press.motion_limits
    if start_limits is not None and start_limits.upper is not None:
        start_rest = ctx.part_world_position(start_button)
        with ctx.pose({start_press: start_limits.upper}):
            start_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start button presses inward",
            start_rest is not None and start_pressed is not None and start_pressed[1] > start_rest[1] + 0.0006,
            details=f"rest={start_rest}, pressed={start_pressed}",
        )

    stop_limits = stop_press.motion_limits
    if stop_limits is not None and stop_limits.upper is not None:
        stop_rest = ctx.part_world_position(stop_button)
        with ctx.pose({stop_press: stop_limits.upper}):
            stop_pressed = ctx.part_world_position(stop_button)
        ctx.check(
            "stop button presses inward",
            stop_rest is not None and stop_pressed is not None and stop_pressed[1] > stop_rest[1] + 0.0006,
            details=f"rest={stop_rest}, pressed={stop_pressed}",
        )

    ctx.check(
        "cutter disc uses continuous rotation",
        disc_spin.articulation_type == ArticulationType.CONTINUOUS
        and disc_spin.motion_limits is not None
        and disc_spin.motion_limits.lower is None
        and disc_spin.motion_limits.upper is None,
        details=f"type={disc_spin.articulation_type}, limits={disc_spin.motion_limits}",
    )
    ctx.check(
        "ring dial uses continuous rotation",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin.motion_limits is not None
        and dial_spin.motion_limits.lower is None
        and dial_spin.motion_limits.upper is None,
        details=f"type={dial_spin.articulation_type}, limits={dial_spin.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
