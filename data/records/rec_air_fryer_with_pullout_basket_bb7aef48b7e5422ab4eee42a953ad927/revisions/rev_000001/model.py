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
)


def _body_shell(
    body_depth: float,
    body_width: float,
    body_height: float,
    corner_radius: float,
    cavity_cut_depth: float,
    cavity_cut_width: float,
    cavity_cut_height: float,
    cavity_cut_x_min: float,
    cavity_center_y: float,
    cavity_cut_z0: float,
):
    outer = (
        cq.Workplane("XY")
        .box(body_depth, body_width, body_height, centered=(True, True, False))
        .edges("|Z")
        .fillet(corner_radius)
    )
    cavity_cut = (
        cq.Workplane("XY")
        .box(cavity_cut_depth, cavity_cut_width, cavity_cut_height, centered=(False, True, False))
        .translate((cavity_cut_x_min, cavity_center_y, cavity_cut_z0))
    )
    return outer.cut(cavity_cut)


def _front_panel(
    panel_t: float,
    panel_w: float,
    panel_h: float,
    opening_w: float,
    opening_h: float,
    opening_center_y: float,
    opening_z0: float,
):
    panel = cq.Workplane("XY").box(panel_t, panel_w, panel_h, centered=(False, True, False))
    opening = (
        cq.Workplane("XY")
        .box(panel_t + 0.012, opening_w, opening_h, centered=(False, True, False))
        .translate((-0.004, opening_center_y, opening_z0))
    )
    return panel.cut(opening)


def _cavity_liner(depth: float, width: float, height: float, wall: float):
    outer = (
        cq.Workplane("XY")
        .box(depth, width, height, centered=(False, True, False))
        .translate((-depth, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall + 0.020, width - 2.0 * wall, height - wall, centered=(False, True, False))
        .translate((-depth + wall, 0.0, wall))
    )
    return outer.cut(inner)


def _drawer_shell():
    face_t = 0.044
    face_w = 0.252
    face_h = 0.182
    shell_depth = 0.286
    shell_w = 0.220
    shell_h = 0.128
    shell_z0 = 0.020
    wall = 0.005
    handle_depth = 0.052
    handle_w = 0.172
    handle_h = 0.030
    handle_z0 = 0.074

    face = cq.Workplane("XY").box(face_t, face_w, face_h, centered=(False, True, False))

    bucket_outer = (
        cq.Workplane("XY")
        .box(shell_depth, shell_w, shell_h, centered=(False, True, False))
        .translate((-0.278, 0.0, shell_z0))
    )
    bucket_inner = (
        cq.Workplane("XY")
        .box(shell_depth - 0.018, shell_w - 2.0 * wall, shell_h, centered=(False, True, False))
        .translate((-0.269, 0.0, shell_z0 + wall))
    )
    shell = face.union(bucket_outer.cut(bucket_inner))

    handle_outer = (
        cq.Workplane("XY")
        .box(handle_depth, handle_w, handle_h, centered=(False, True, False))
        .translate((face_t, 0.0, handle_z0))
    )
    grip_slot = (
        cq.Workplane("XY")
        .box(0.038, 0.136, 0.020, centered=(False, True, False))
        .translate((face_t + 0.010, 0.0, handle_z0))
    )
    button_slot = (
        cq.Workplane("XY")
        .box(0.044, 0.036, 0.022, centered=(True, True, False))
        .translate((0.066, 0.0, 0.084))
    )

    return shell.union(handle_outer.cut(grip_slot).cut(button_slot))


def _basket():
    depth = 0.228
    width = 0.186
    height = 0.076
    wall = 0.004

    outer = (
        cq.Workplane("XY")
        .box(depth, width, height, centered=(False, True, False))
        .translate((-depth + 0.004, 0.0, 0.010))
    )
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall + 0.018, width - 2.0 * wall, height - wall, centered=(False, True, False))
        .translate((-depth + wall + 0.013, 0.0, 0.014))
    )
    basket = outer.cut(inner)

    for y in (-0.060, -0.020, 0.020, 0.060):
        slot = (
            cq.Workplane("XY")
            .box(0.160, 0.012, 0.0045, centered=(False, True, False))
            .translate((-0.188, y, 0.012))
        )
        basket = basket.cut(slot)

    for x, y in ((-0.206, -0.070), (-0.206, 0.070), (-0.034, -0.070), (-0.034, 0.070)):
        foot = (
            cq.Workplane("XY")
            .box(0.012, 0.012, 0.010, centered=(False, True, False))
            .translate((x, y, 0.0))
        )
        basket = basket.union(foot)

    return basket


def _rocker():
    return cq.Workplane("XY").box(0.014, 0.032, 0.019, centered=(False, True, True))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_air_fryer")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.32, 0.33, 0.35, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.63, 0.66, 1.0))
    glass_black = model.material("glass_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body_depth = 0.410
    body_width = 0.340
    body_height = 0.390
    body_front_x = body_depth / 2.0
    drawer_center_y = -0.048
    cavity_floor_z = 0.074

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            _body_shell(
                body_depth=body_depth,
                body_width=body_width,
                body_height=body_height,
                corner_radius=0.026,
                cavity_cut_depth=0.345,
                cavity_cut_width=0.252,
                cavity_cut_height=0.180,
                cavity_cut_x_min=-0.128,
                cavity_center_y=drawer_center_y,
                cavity_cut_z0=0.072,
            ),
            "air_fryer_body_shell",
        ),
        material=matte_black,
        name="body_shell",
    )
    cavity_liner = model.part("cavity_liner")
    cavity_liner.visual(
        Box((0.314, 0.244, 0.004)),
        origin=Origin(xyz=(-0.157, 0.0, -0.002)),
        material=satin_metal,
        name="liner_floor",
    )
    cavity_liner.visual(
        Box((0.314, 0.244, 0.004)),
        origin=Origin(xyz=(-0.157, 0.0, 0.172)),
        material=satin_metal,
        name="liner_roof",
    )
    cavity_liner.visual(
        Box((0.314, 0.004, 0.170)),
        origin=Origin(xyz=(-0.157, -0.120, 0.085)),
        material=satin_metal,
        name="liner_wall_0",
    )
    cavity_liner.visual(
        Box((0.314, 0.004, 0.170)),
        origin=Origin(xyz=(-0.157, 0.120, 0.085)),
        material=satin_metal,
        name="liner_wall_1",
    )
    cavity_liner.visual(
        Box((0.004, 0.244, 0.170)),
        origin=Origin(xyz=(-0.312, 0.0, 0.085)),
        material=satin_metal,
        name="liner_back",
    )
    cavity_liner.visual(
        Box((0.260, 0.016, 0.020)),
        origin=Origin(xyz=(-0.140, -0.114, -0.006)),
        material=dark_metal,
        name="liner_rail_0",
    )
    cavity_liner.visual(
        Box((0.260, 0.016, 0.020)),
        origin=Origin(xyz=(-0.140, 0.114, -0.006)),
        material=dark_metal,
        name="liner_rail_1",
    )
    model.articulation(
        "body_to_cavity_liner",
        ArticulationType.FIXED,
        parent=body,
        child=cavity_liner,
        origin=Origin(xyz=(0.193, drawer_center_y, 0.076)),
    )

    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_cadquery(
            _front_panel(
                panel_t=0.003,
                panel_w=0.316,
                panel_h=0.332,
                opening_w=0.242,
                opening_h=0.160,
                opening_center_y=drawer_center_y,
                opening_z0=0.052,
            ),
            "air_fryer_front_panel",
        ),
        material=brushed_steel,
        name="front_panel",
    )
    model.articulation(
        "body_to_front_panel",
        ArticulationType.FIXED,
        parent=body,
        child=front_panel,
        origin=Origin(xyz=(body_front_x, 0.0, 0.028)),
    )

    display_window = model.part("display_window")
    display_window.visual(
        Box((0.006, 0.124, 0.050)),
        origin=Origin(xyz=(0.003, 0.0, 0.025)),
        material=glass_black,
        name="display_window",
    )
    model.articulation(
        "front_panel_to_display_window",
        ArticulationType.FIXED,
        parent=front_panel,
        child=display_window,
        origin=Origin(xyz=(0.003, -0.012, 0.248)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shell(), "air_fryer_drawer"),
        material=charcoal,
        name="drawer_shell",
    )
    drawer.visual(
        Box((0.280, 0.010, 0.010)),
        origin=Origin(xyz=(-0.140, -0.114, 0.012)),
        material=dark_metal,
        name="drawer_rail_0",
    )
    drawer.visual(
        Box((0.280, 0.010, 0.010)),
        origin=Origin(xyz=(-0.140, 0.114, 0.012)),
        material=dark_metal,
        name="drawer_rail_1",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.217, drawer_center_y, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=-0.030, upper=0.150),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket(), "air_fryer_basket"),
        material=dark_metal,
        name="basket_shell",
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(-0.006, 0.0, 0.025)),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.022,
                body_style="skirted",
                top_diameter=0.044,
                skirt=KnobSkirt(0.066, 0.004, flare=0.08),
                grip=KnobGrip(style="fluted", count=18, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "air_fryer_selector_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="dial_cap",
    )
    selector_dial.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="dial_shaft",
    )
    model.articulation(
        "front_panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=front_panel,
        child=selector_dial,
        origin=Origin(xyz=(0.003, 0.109, 0.255)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    rocker_0 = model.part("rocker_0")
    rocker_0.visual(
        mesh_from_cadquery(_rocker(), "air_fryer_rocker_0"),
        material=charcoal,
        name="rocker_cap",
    )
    rocker_0.visual(
        Cylinder(radius=0.003, length=0.032),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rocker_pivot",
    )
    model.articulation(
        "front_panel_to_rocker_0",
        ArticulationType.REVOLUTE,
        parent=front_panel,
        child=rocker_0,
        origin=Origin(xyz=(0.005, 0.109, 0.192)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-0.18, upper=0.18),
    )

    rocker_1 = model.part("rocker_1")
    rocker_1.visual(
        mesh_from_cadquery(_rocker(), "air_fryer_rocker_1"),
        material=charcoal,
        name="rocker_cap",
    )
    rocker_1.visual(
        Cylinder(radius=0.003, length=0.032),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rocker_pivot",
    )
    model.articulation(
        "front_panel_to_rocker_1",
        ArticulationType.REVOLUTE,
        parent=front_panel,
        child=rocker_1,
        origin=Origin(xyz=(0.005, 0.109, 0.147)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-0.18, upper=0.18),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.044, 0.036, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_steel,
        name="latch_pad",
    )
    latch_button.visual(
        Box((0.012, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_metal,
        name="latch_stem",
    )
    model.articulation(
        "drawer_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=latch_button,
        origin=Origin(xyz=(0.066, 0.0, 0.093)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cavity_liner = object_model.get_part("cavity_liner")
    front_panel = object_model.get_part("front_panel")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    selector_dial = object_model.get_articulation("front_panel_to_selector_dial")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    rocker_joint_0 = object_model.get_articulation("front_panel_to_rocker_0")
    rocker_joint_1 = object_model.get_articulation("front_panel_to_rocker_1")
    latch_joint = object_model.get_articulation("drawer_to_latch_button")

    ctx.allow_overlap(
        basket,
        drawer,
        elem_a="basket_shell",
        elem_b="drawer_shell",
        reason="The removable fry basket is intentionally represented as nesting inside the simplified drawer carrier shell.",
    )
    ctx.allow_overlap(
        body,
        cavity_liner,
        elem_a="body_shell",
        elem_b="liner_rail_0",
        reason="The fixed side rail is intentionally modeled as mounted through the simplified outer housing shell into the cavity sidewall.",
    )
    ctx.allow_overlap(
        body,
        cavity_liner,
        elem_a="body_shell",
        elem_b="liner_rail_1",
        reason="The fixed side rail is intentionally modeled as mounted through the simplified outer housing shell into the cavity sidewall.",
    )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="body_shell",
        elem_b="drawer_rail_0",
        reason="The sliding rail member is intentionally represented as running within the body side channel simplified by the shell volume.",
    )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="body_shell",
        elem_b="drawer_rail_1",
        reason="The sliding rail member is intentionally represented as running within the body side channel simplified by the shell volume.",
    )
    ctx.allow_overlap(
        cavity_liner,
        drawer,
        elem_a="liner_rail_0",
        elem_b="drawer_rail_0",
        reason="The telescoping rail members are simplified as interpenetrating box runners while representing a nested slide assembly.",
    )
    ctx.allow_overlap(
        cavity_liner,
        drawer,
        elem_a="liner_rail_1",
        elem_b="drawer_rail_1",
        reason="The telescoping rail members are simplified as interpenetrating box runners while representing a nested slide assembly.",
    )
    ctx.allow_overlap(
        cavity_liner,
        drawer,
        elem_a="liner_rail_0",
        elem_b="drawer_shell",
        reason="The drawer shell is simplified around the telescoping rail track, so the carrier rail is intentionally allowed to occupy that local side-channel volume.",
    )
    ctx.allow_overlap(
        cavity_liner,
        drawer,
        elem_a="liner_rail_1",
        elem_b="drawer_shell",
        reason="The drawer shell is simplified around the telescoping rail track, so the carrier rail is intentionally allowed to occupy that local side-channel volume.",
    )
    ctx.allow_overlap(
        drawer,
        latch_joint.child,
        elem_a="drawer_shell",
        elem_b="latch_pad",
        reason="The latch cap intentionally sits captured inside the recessed handle cutout at the drawer front.",
    )

    ctx.check(
        "selector dial uses continuous rotation",
        selector_dial.articulation_type == ArticulationType.CONTINUOUS
        and selector_dial.motion_limits is not None
        and selector_dial.motion_limits.lower is None
        and selector_dial.motion_limits.upper is None,
        details=f"type={selector_dial.articulation_type}, limits={selector_dial.motion_limits}",
    )

    slide_limits = drawer_slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        closed_pos = None
        open_pos = None
        with ctx.pose({drawer_slide: slide_limits.lower}):
            ctx.expect_overlap(
                drawer,
                front_panel,
                axes="yz",
                elem_a="drawer_shell",
                elem_b="front_panel",
                min_overlap=0.150,
                name="drawer face still covers the front opening when closed",
            )
            ctx.expect_within(
                basket,
                drawer,
                axes="yz",
                inner_elem="basket_shell",
                outer_elem="drawer_shell",
                margin=0.004,
                name="basket stays centered within the drawer shell",
            )
            closed_pos = ctx.part_world_position(drawer)

        with ctx.pose({drawer_slide: slide_limits.upper}):
            ctx.expect_overlap(
                cavity_liner,
                drawer,
                axes="x",
                elem_a="liner_rail_0",
                elem_b="drawer_rail_0",
                min_overlap=0.080,
                name="left drawer rail retains insertion at full extension",
            )
            ctx.expect_overlap(
                cavity_liner,
                drawer,
                axes="x",
                elem_a="liner_rail_1",
                elem_b="drawer_rail_1",
                min_overlap=0.080,
                name="right drawer rail retains insertion at full extension",
            )
            ctx.expect_within(
                drawer,
                cavity_liner,
                axes="yz",
                inner_elem="drawer_rail_0",
                outer_elem="liner_rail_0",
                margin=0.004,
                name="left drawer rail stays aligned on the support rail",
            )
            ctx.expect_within(
                drawer,
                cavity_liner,
                axes="yz",
                inner_elem="drawer_rail_1",
                outer_elem="liner_rail_1",
                margin=0.004,
                name="right drawer rail stays aligned on the support rail",
            )
            ctx.expect_within(
                basket,
                cavity_liner,
                axes="yz",
                inner_elem="basket_shell",
                margin=0.030,
                name="basket remains centered in the heating cavity path",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            "drawer extends outward",
            closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 0.170,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    latch_limits = latch_joint.motion_limits
    if latch_limits is not None and latch_limits.upper is not None:
        released = None
        pressed = None
        with ctx.pose({latch_joint: 0.0}):
            released = ctx.part_element_world_aabb("latch_button", elem="latch_pad")
        with ctx.pose({latch_joint: latch_limits.upper}):
            pressed = ctx.part_element_world_aabb("latch_button", elem="latch_pad")
        ctx.check(
            "handle latch button presses downward",
            released is not None and pressed is not None and pressed[1][2] < released[1][2] - 0.007,
            details=f"released={released}, pressed={pressed}",
        )

    for joint_name, joint in (("upper rocker", rocker_joint_0), ("lower rocker", rocker_joint_1)):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        low_aabb = None
        high_aabb = None
        with ctx.pose({joint: limits.lower}):
            low_aabb = ctx.part_element_world_aabb(joint.child, elem="rocker_cap")
        with ctx.pose({joint: limits.upper}):
            high_aabb = ctx.part_element_world_aabb(joint.child, elem="rocker_cap")
        ctx.check(
            f"{joint_name} pivots through a visible stroke",
            low_aabb is not None
            and high_aabb is not None
            and abs(high_aabb[0][2] - low_aabb[0][2]) > 0.002,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
