from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
)


def _make_body_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(0.245, 0.208, 0.040, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )

    upper = (
        cq.Workplane("XY")
        .workplane(offset=0.040)
        .rect(0.220, 0.162)
        .workplane(offset=0.050)
        .center(0.000, 0.018)
        .rect(0.152, 0.118)
        .loft(combine=True)
    )

    control_panel = (
        cq.Workplane("XY")
        .box(0.150, 0.014, 0.060, centered=(True, True, False))
        .translate((0.000, -0.111, 0.022))
    )

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=0.090)
        .center(0.000, 0.020)
        .circle(0.060)
        .extrude(0.014)
    )

    rear_cap = (
        cq.Workplane("XY")
        .box(0.165, 0.085, 0.020, centered=(True, True, False))
        .translate((0.000, 0.038, 0.084))
    )

    return lower.union(upper).union(control_panel).union(pedestal).union(rear_cap)


def _make_bowl_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.044)
        .workplane(offset=0.028)
        .circle(0.074)
        .workplane(offset=0.064)
        .circle(0.103)
        .workplane(offset=0.046)
        .circle(0.108)
        .loft(combine=True)
    )

    rim = (
        cq.Workplane("XY")
        .workplane(offset=0.138)
        .circle(0.112)
        .circle(0.100)
        .extrude(0.008)
    )

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .circle(0.030)
        .workplane(offset=0.022)
        .circle(0.066)
        .workplane(offset=0.070)
        .circle(0.095)
        .workplane(offset=0.035)
        .circle(0.099)
        .loft(combine=True)
    )

    handle_left = (
        cq.Workplane("XY")
        .box(0.008, 0.048, 0.096, centered=(True, True, False))
        .translate((-0.010, -0.132, 0.026))
    )
    handle_right = (
        cq.Workplane("XY")
        .box(0.008, 0.048, 0.096, centered=(True, True, False))
        .translate((0.010, -0.132, 0.026))
    )
    handle_top = (
        cq.Workplane("XY")
        .box(0.028, 0.048, 0.010, centered=(True, True, False))
        .translate((0.000, -0.132, 0.112))
    )
    handle = handle_left.union(handle_right).union(handle_top)

    spindle = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(0.010)
        .extrude(0.046)
    )

    left_hinge_support = (
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.014, centered=(True, True, False))
        .translate((-0.052, 0.108, 0.138))
    )
    right_hinge_support = (
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.014, centered=(True, True, False))
        .translate((0.052, 0.108, 0.138))
    )
    left_knuckle = (
        cq.Workplane("YZ")
        .circle(0.007)
        .extrude(0.034)
        .translate((-0.069, 0.116, 0.151))
    )
    right_knuckle = (
        cq.Workplane("YZ")
        .circle(0.007)
        .extrude(0.034)
        .translate((0.035, 0.116, 0.151))
    )

    bowl = outer.union(rim).cut(cavity)
    bowl = bowl.union(handle).union(spindle)
    bowl = bowl.union(left_hinge_support).union(right_hinge_support)
    bowl = bowl.union(left_knuckle).union(right_knuckle)
    return bowl


def _make_lid_shape() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .center(0.000, -0.116)
        .circle(0.112)
        .extrude(0.008)
    )

    rim_lip = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .center(0.000, -0.116)
        .circle(0.112)
        .circle(0.103)
        .extrude(0.004)
    )

    left_lock_tab = (
        cq.Workplane("XY")
        .box(0.018, 0.012, 0.009, centered=(True, True, False))
        .translate((-0.050, -0.028, 0.007))
    )
    right_lock_tab = (
        cq.Workplane("XY")
        .box(0.018, 0.012, 0.009, centered=(True, True, False))
        .translate((0.050, -0.028, 0.007))
    )

    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.064, 0.018, 0.010, centered=(True, True, False))
        .translate((0.000, 0.000, 0.004))
    )

    chute_outer = (
        cq.Workplane("XY")
        .box(0.070, 0.056, 0.102, centered=(True, True, False))
        .translate((0.030, -0.068, 0.018))
    )
    chute_inner = (
        cq.Workplane("XY")
        .box(0.062, 0.048, 0.116, centered=(True, True, False))
        .translate((0.030, -0.068, 0.014))
    )

    flap_collar_outer = (
        cq.Workplane("XY")
        .box(0.050, 0.038, 0.010, centered=(True, True, False))
        .translate((-0.050, -0.068, 0.020))
    )
    flap_collar_inner = (
        cq.Workplane("XY")
        .box(0.040, 0.028, 0.014, centered=(True, True, False))
        .translate((-0.050, -0.068, 0.018))
    )

    chute = chute_outer.cut(chute_inner)
    flap_collar = flap_collar_outer.cut(flap_collar_inner)

    lid = cover.union(rim_lip).union(hinge_bridge).union(left_lock_tab).union(right_lock_tab)
    lid = lid.union(chute).union(flap_collar)
    lid = lid.cut(chute_inner).cut(flap_collar_inner)
    return lid


def _make_pusher_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.066, 0.052, 0.020, centered=(True, True, True))
        .translate((0.000, 0.000, 0.000))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.058, 0.044, 0.148, centered=(True, True, False))
        .translate((0.000, 0.000, -0.150))
    )
    return cap.union(stem)


def _make_flap_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("YZ")
        .circle(0.0030)
        .extrude(0.034, both=True)
    )
    cover = (
        cq.Workplane("XY")
        .box(0.044, 0.034, 0.0030, centered=(True, True, False))
        .translate((0.000, -0.017, -0.003))
    )
    tab = (
        cq.Workplane("XY")
        .box(0.012, 0.010, 0.006, centered=(True, True, False))
        .translate((0.000, -0.030, -0.001))
    )
    return barrel.union(cover).union(tab)


def _make_blade_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.018)
        .circle(0.010)
        .extrude(0.018)
    )
    lower_blade = (
        cq.Workplane("XY")
        .box(0.060, 0.014, 0.003, centered=(True, True, False))
        .translate((0.000, 0.000, 0.007))
        .rotate((0.000, 0.000, 0.000), (0.000, 1.000, 0.000), 10.0)
    )
    upper_blade = (
        cq.Workplane("XY")
        .box(0.052, 0.014, 0.003, centered=(True, True, False))
        .translate((0.000, 0.000, 0.012))
        .rotate((0.000, 0.000, 0.000), (0.000, 1.000, 0.000), -12.0)
    )
    return hub.union(lower_blade).union(upper_blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_base_food_processor")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.73, 0.74, 0.75, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.82, 0.88, 0.90, 0.35))
    pusher_grey = model.material("pusher_grey", rgba=(0.83, 0.84, 0.85, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    button_light = model.material("button_light", rgba=(0.95, 0.95, 0.94, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "food_processor_body"),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        Box((0.150, 0.004, 0.060)),
        origin=Origin(xyz=(0.000, -0.119, 0.052)),
        material=panel_grey,
        name="panel_trim",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shape(), "food_processor_bowl"),
        material=clear_smoke,
        name="bowl_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shape(), "food_processor_lid"),
        material=clear_smoke,
        name="lid_shell",
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_make_pusher_shape(), "food_processor_pusher"),
        material=pusher_grey,
        name="pusher_shell",
    )

    ingredient_flap = model.part("ingredient_flap")
    ingredient_flap.visual(
        mesh_from_cadquery(_make_flap_shape(), "food_processor_ingredient_flap"),
        material=clear_smoke,
        name="flap_shell",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_make_blade_shape(), "food_processor_blade"),
        material=blade_steel,
        name="blade_shell",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.026,
                body_style="skirted",
                top_diameter=0.030,
                skirt=KnobSkirt(0.044, 0.006, flare=0.10),
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            ),
            "food_processor_speed_knob",
        ),
        origin=Origin(xyz=(0.000, -0.020, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_shell",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Box((0.020, 0.010, 0.015)),
        origin=Origin(xyz=(0.000, -0.006, 0.000)),
        material=button_light,
        name="button_cap",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Box((0.020, 0.010, 0.015)),
        origin=Origin(xyz=(0.000, -0.006, 0.000)),
        material=button_light,
        name="button_cap",
    )

    model.articulation(
        "body_to_bowl",
        ArticulationType.FIXED,
        parent=body,
        child=bowl,
        origin=Origin(xyz=(0.000, 0.020, 0.104)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.000, 0.116, 0.148)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.030, -0.068, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.30, lower=0.0, upper=0.110),
    )
    model.articulation(
        "lid_to_ingredient_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=ingredient_flap,
        origin=Origin(xyz=(-0.050, -0.049, 0.033)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "bowl_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade,
        origin=Origin(xyz=(0.000, 0.000, 0.0465)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "body_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_knob,
        origin=Origin(xyz=(-0.028, -0.120, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "body_to_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(0.028, -0.120, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.0035),
    )
    model.articulation(
        "body_to_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.060, -0.120, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.0035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    ingredient_flap = object_model.get_part("ingredient_flap")
    blade = object_model.get_part("blade")
    speed_knob = object_model.get_part("speed_knob")
    button_0 = object_model.get_part("button_0")

    lid_hinge = object_model.get_articulation("bowl_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    flap_hinge = object_model.get_articulation("lid_to_ingredient_flap")
    blade_spin = object_model.get_articulation("bowl_to_blade")
    knob_spin = object_model.get_articulation("body_to_speed_knob")
    button_press = object_model.get_articulation("body_to_button_0")

    ctx.allow_overlap(
        blade,
        bowl,
        reason="The removable blade hub is intentionally represented as a close spindle fit inside the bowl's central drive post.",
        elem_a="blade_shell",
        elem_b="bowl_shell",
    )
    ctx.allow_isolated_part(
        lid,
        reason="The rear lid assembly is carried by its articulated hinge line with a small modeled clearance to the bowl shell in the closed pose.",
    )
    ctx.allow_isolated_part(
        pusher,
        reason="The pusher is part of the hinged lid assembly and is supported by the chute representation rather than separate closed-pose contact to the grounded bowl.",
    )
    ctx.allow_isolated_part(
        ingredient_flap,
        reason="The ingredient flap rides on the lid assembly and shares the hinge-carried clearance of the closed lid group.",
    )

    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        min_overlap=0.18,
        name="closed lid spans the bowl opening",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    lid_limits = lid_hinge.motion_limits
    if closed_lid_aabb is not None and lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward from the rear hinge",
            open_lid_aabb is not None and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    pusher_limits = pusher_slide.motion_limits
    if pusher_limits is not None and pusher_limits.upper is not None:
        rest_pusher_pos = ctx.part_world_position(pusher)
        with ctx.pose({pusher_slide: pusher_limits.upper}):
            extended_pusher_pos = ctx.part_world_position(pusher)
        ctx.check(
            "pusher lifts out of the chute",
            rest_pusher_pos is not None
            and extended_pusher_pos is not None
            and extended_pusher_pos[2] > rest_pusher_pos[2] + 0.08,
            details=f"rest={rest_pusher_pos}, extended={extended_pusher_pos}",
        )

    closed_flap_aabb = ctx.part_element_world_aabb(ingredient_flap, elem="flap_shell")
    flap_limits = flap_hinge.motion_limits
    if closed_flap_aabb is not None and flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({flap_hinge: flap_limits.upper}):
            open_flap_aabb = ctx.part_element_world_aabb(ingredient_flap, elem="flap_shell")
        ctx.check(
            "ingredient flap rotates upward",
            open_flap_aabb is not None and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.015,
            details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
        )

    button_limits = button_press.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_button_pos = ctx.part_world_position(button_0)
        with ctx.pose({button_press: button_limits.upper}):
            pressed_button_pos = ctx.part_world_position(button_0)
        ctx.check(
            "front button presses into the control panel",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[1] > rest_button_pos[1] + 0.0015,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    ctx.check(
        "blade uses continuous spindle rotation",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_spin.articulation_type}",
    )
    ctx.check(
        "speed knob uses continuous rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )

    ctx.expect_origin_distance(
        speed_knob,
        button_0,
        axes="x",
        min_dist=0.04,
        max_dist=0.08,
        name="buttons sit close to the speed knob",
    )

    ctx.expect_overlap(
        blade,
        bowl,
        axes="xy",
        min_overlap=0.03,
        name="blade remains centered inside the bowl footprint",
    )

    return ctx.report()


object_model = build_object_model()
