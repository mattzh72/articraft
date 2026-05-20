from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def rounded_box(dx: float, dy: float, dz: float, *, fillet_axis: str, radius: float) -> cq.Workplane:
    selector = {"x": "|X", "y": "|Y", "z": "|Z"}[fillet_axis]
    solid = cq.Workplane("XY").box(dx, dy, dz)
    if radius > 0.0:
        solid = solid.edges(selector).fillet(radius)
    return solid


def make_button_mesh(
    *,
    cap_dx: float,
    cap_dy: float,
    cap_dz: float,
    stem_dx: float,
    stem_dy: float,
    stem_dz: float,
) -> cq.Workplane:
    cap_radius = min(cap_dy, cap_dz) * 0.42

    button = rounded_box(cap_dx, cap_dy, cap_dz, fillet_axis="x", radius=cap_radius).translate(
        (cap_dx / 2.0, 0.0, 0.0)
    )
    if stem_dx > 0.0 and stem_dy > 0.0 and stem_dz > 0.0:
        stem = cq.Workplane("XY").box(stem_dx, stem_dy, stem_dz).translate((-stem_dx / 2.0, 0.0, 0.0))
        button = button.union(stem)
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_rice_cooker")

    body_color = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_color = model.material("trim_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    button_color = model.material("button_charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    display_color = model.material("display_black", rgba=(0.09, 0.10, 0.11, 1.0))

    body_h = 0.188
    body_rx_bottom = 0.148
    body_ry_bottom = 0.136
    body_rx_mid = 0.146
    body_ry_mid = 0.134
    body_rx_top = 0.142
    body_ry_top = 0.128

    cavity_rx = 0.116
    cavity_ry = 0.102
    cavity_floor = 0.018

    body_shell = (
        cq.Workplane("XY")
        .ellipse(body_rx_bottom, body_ry_bottom)
        .workplane(offset=0.110)
        .ellipse(body_rx_mid, body_ry_mid)
        .workplane(offset=body_h - 0.110)
        .ellipse(body_rx_top, body_ry_top)
        .loft(combine=True)
    )

    body_cavity = (
        cq.Workplane("XY")
        .ellipse(cavity_rx, cavity_ry)
        .extrude(body_h)
        .translate((0.0, 0.0, cavity_floor))
    )
    body_shell = body_shell.cut(body_cavity)

    control_panel = rounded_box(0.016, 0.208, 0.116, fillet_axis="x", radius=0.026).translate(
        (0.148, 0.0, 0.087)
    )
    body_shell = body_shell.union(control_panel)

    display_recess = rounded_box(0.008, 0.102, 0.028, fillet_axis="x", radius=0.010).translate(
        (0.152, 0.0, 0.115)
    )
    body_shell = body_shell.cut(display_recess)

    menu_opening = rounded_box(0.045, 0.034, 0.020, fillet_axis="x", radius=0.008)
    body_shell = body_shell.cut(menu_opening.translate((0.145, -0.043, 0.076)))
    body_shell = body_shell.cut(menu_opening.translate((0.145, 0.043, 0.076)))

    latch_opening = rounded_box(0.045, 0.072, 0.028, fillet_axis="x", radius=0.011).translate(
        (0.145, 0.0, 0.047)
    )
    body_shell = body_shell.cut(latch_opening)

    panel_cover = rounded_box(0.010, 0.204, 0.112, fillet_axis="x", radius=0.024).translate(
        (0.149, 0.0, 0.087)
    )
    panel_cover = panel_cover.cut(
        rounded_box(0.012, 0.096, 0.026, fillet_axis="x", radius=0.009).translate((0.150, 0.0, 0.115))
    )
    panel_cover = panel_cover.cut(
        rounded_box(0.014, 0.038, 0.022, fillet_axis="x", radius=0.008).translate((0.149, -0.043, 0.076))
    )
    panel_cover = panel_cover.cut(
        rounded_box(0.014, 0.038, 0.022, fillet_axis="x", radius=0.008).translate((0.149, 0.043, 0.076))
    )
    panel_cover = panel_cover.cut(
        rounded_box(0.014, 0.074, 0.030, fillet_axis="x", radius=0.010).translate((0.149, 0.0, 0.047))
    )

    lid_rx = 0.137
    lid_ry = 0.124
    lid_shell = (
        cq.Workplane("XY")
        .ellipse(lid_rx, lid_ry)
        .workplane(offset=0.028)
        .ellipse(0.133, 0.120)
        .workplane(offset=0.024)
        .ellipse(0.125, 0.113)
        .loft(combine=True)
        .translate((lid_rx, 0.0, 0.0))
    )

    underside_pocket = (
        cq.Workplane("XY")
        .ellipse(0.116, 0.102)
        .extrude(0.040)
        .translate((lid_rx, 0.0, 0.004))
    )
    lid_shell = lid_shell.cut(underside_pocket)

    handle_recess = rounded_box(0.078, 0.088, 0.016, fillet_axis="x", radius=0.007).translate(
        (0.198, 0.0, 0.044)
    )
    lid_shell = lid_shell.cut(handle_recess)

    steam_vent = rounded_box(0.046, 0.028, 0.010, fillet_axis="z", radius=0.009).translate(
        (0.114, 0.0, 0.051)
    )
    lid_shell = lid_shell.union(steam_vent)
    vent_slot = cq.Workplane("XY").box(0.022, 0.003, 0.004)
    lid_shell = lid_shell.cut(vent_slot.translate((0.114, -0.006, 0.054)))
    lid_shell = lid_shell.cut(vent_slot.translate((0.114, 0.006, 0.054)))

    latch_mesh = make_button_mesh(
        cap_dx=0.007,
        cap_dy=0.076,
        cap_dz=0.032,
        stem_dx=0.0,
        stem_dy=0.0,
        stem_dz=0.0,
    )
    menu_mesh = make_button_mesh(
        cap_dx=0.006,
        cap_dy=0.038,
        cap_dz=0.024,
        stem_dx=0.0,
        stem_dy=0.0,
        stem_dz=0.0,
    )

    body = model.part("body")
    body.visual(mesh_from_cadquery(body_shell, "body_shell"), material=body_color, name="body_shell")
    body.visual(mesh_from_cadquery(panel_cover, "front_panel"), material=trim_color, name="front_panel")
    body.visual(
        Box((0.010, 0.088, 0.020)),
        origin=Origin(xyz=(0.151, 0.0, 0.115)),
        material=display_color,
        name="display_window",
    )
    body.visual(
        Box((0.022, 0.088, 0.014)),
        origin=Origin(xyz=(-0.145, 0.0, 0.176)),
        material=trim_color,
        name="hinge_bridge",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(lid_shell, "lid_shell"), material=body_color, name="lid_shell")

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(latch_mesh, "latch_button"),
        material=button_color,
        name="latch_button",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        mesh_from_cadquery(menu_mesh, "menu_button_0"),
        material=button_color,
        name="menu_button_0",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        mesh_from_cadquery(menu_mesh, "menu_button_1"),
        material=button_color,
        name="menu_button_1",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.137, 0.0, body_h)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.42),
    )

    button_limits = MotionLimits(effort=12.0, velocity=0.06, lower=0.0, upper=0.004)

    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.156, 0.0, 0.047)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=button_limits,
    )
    model.articulation(
        "body_to_menu_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_0,
        origin=Origin(xyz=(0.156, -0.043, 0.076)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=button_limits,
    )
    model.articulation(
        "body_to_menu_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_1,
        origin=Origin(xyz=(0.156, 0.043, 0.076)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=button_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("body_to_latch_button")
    menu_slide_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_slide_1 = object_model.get_articulation("body_to_menu_button_1")

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0}):
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                elem_a="lid_shell",
                elem_b="body_shell",
                min_overlap=0.22,
                name="closed lid covers the cooker body",
            )
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="lid_shell",
                negative_elem="body_shell",
                min_gap=0.0,
                max_gap=0.010,
                name="closed lid sits on the body rim without sinking in",
            )
            closed_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

        ctx.check(
            "lid opens upward on the rear hinge",
            closed_shell_aabb is not None
            and open_shell_aabb is not None
            and open_shell_aabb[1][2] > closed_shell_aabb[1][2] + 0.10,
            details=f"closed={closed_shell_aabb}, open={open_shell_aabb}",
        )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_slide: 0.004}):
        latch_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "front latch button depresses into the housing",
        latch_rest is not None and latch_pressed is not None and latch_pressed[0] < latch_rest[0] - 0.003,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    menu_0_rest = ctx.part_world_position(menu_button_0)
    menu_1_rest = ctx.part_world_position(menu_button_1)
    with ctx.pose({menu_slide_0: 0.004}):
        menu_0_pressed = ctx.part_world_position(menu_button_0)
        menu_1_when_menu_0_pressed = ctx.part_world_position(menu_button_1)
    with ctx.pose({menu_slide_1: 0.004}):
        menu_1_pressed = ctx.part_world_position(menu_button_1)

    ctx.check(
        "menu button 0 depresses independently",
        menu_0_rest is not None and menu_0_pressed is not None and menu_0_pressed[0] < menu_0_rest[0] - 0.003,
        details=f"rest={menu_0_rest}, pressed={menu_0_pressed}",
    )
    ctx.check(
        "menu button 1 depresses independently",
        menu_1_rest is not None and menu_1_pressed is not None and menu_1_pressed[0] < menu_1_rest[0] - 0.003,
        details=f"rest={menu_1_rest}, pressed={menu_1_pressed}",
    )
    ctx.check(
        "menu buttons remain independent controls",
        menu_1_rest is not None
        and menu_1_when_menu_0_pressed is not None
        and abs(menu_1_when_menu_0_pressed[0] - menu_1_rest[0]) < 1e-6,
        details=f"menu_1_rest={menu_1_rest}, menu_1_with_menu_0_pressed={menu_1_when_menu_0_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
