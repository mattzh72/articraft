from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_oval_rice_cooker")

    porcelain = model.material("warm_white_shell", rgba=(0.88, 0.86, 0.80, 1.0))
    pearl = model.material("pearl_lid", rgba=(0.96, 0.95, 0.90, 1.0))
    dark_plastic = model.material("gloss_black_plastic", rgba=(0.02, 0.023, 0.025, 1.0))
    smoked_glass = model.material("smoked_display_glass", rgba=(0.05, 0.09, 0.12, 1.0))
    soft_gray = model.material("soft_gray_trim", rgba=(0.44, 0.45, 0.44, 1.0))
    satin_metal = model.material("satin_pivot_metal", rgba=(0.58, 0.60, 0.58, 1.0))
    red_button = model.material("red_power_button", rgba=(0.68, 0.06, 0.045, 1.0))
    blue_lcd = model.material("muted_blue_lcd", rgba=(0.42, 0.68, 0.78, 1.0))

    shell = model.part("shell")

    body_sections = [
        (-0.185, 0.028, 0.220, 0.270),
        (-0.135, 0.004, 0.243, 0.335),
        (-0.045, 0.000, 0.252, 0.395),
        (0.070, 0.000, 0.250, 0.405),
        (0.165, 0.018, 0.230, 0.335),
    ]
    body_mesh = superellipse_side_loft(body_sections, exponents=2.35, segments=72, cap=True)
    shell.visual(
        mesh_from_geometry(body_mesh, "oval_body_shell"),
        material=porcelain,
        name="oval_body_shell",
    )

    base_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.330, 0.260, 0.045, corner_segments=12),
        0.030,
        center=True,
    )
    shell.visual(
        mesh_from_geometry(base_mesh, "black_oval_base"),
        origin=Origin(xyz=(0.0, -0.006, 0.018)),
        material=dark_plastic,
        name="black_oval_base",
    )

    rim_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.300, 0.230, 0.055, corner_segments=14),
        0.010,
        center=True,
    )
    shell.visual(
        mesh_from_geometry(rim_mesh, "inner_pot_rim"),
        origin=Origin(xyz=(0.0, -0.020, 0.247)),
        material=dark_plastic,
        name="inner_pot_rim",
    )
    bowl_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.245, 0.180, 0.040, corner_segments=12),
        0.018,
        center=True,
    )
    shell.visual(
        mesh_from_geometry(bowl_mesh, "dark_inner_bowl"),
        origin=Origin(xyz=(0.0, -0.020, 0.235)),
        material=smoked_glass,
        name="dark_inner_bowl",
    )

    front_panel_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.230, 0.150, 0.024, corner_segments=10),
        0.012,
        center=True,
    )
    shell.visual(
        mesh_from_geometry(front_panel_mesh, "front_control_panel"),
        origin=Origin(xyz=(0.0, -0.188, 0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_gray,
        name="front_control_panel",
    )

    display_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.138, 0.047, 0.010, corner_segments=8),
        0.005,
        center=True,
    )
    shell.visual(
        mesh_from_geometry(display_mesh, "display_window"),
        origin=Origin(xyz=(0.0, -0.196, 0.166), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="display_window",
    )
    lcd_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.104, 0.026, 0.005, corner_segments=5),
        0.002,
        center=True,
    )
    shell.visual(
        mesh_from_geometry(lcd_mesh, "display_lcd"),
        origin=Origin(xyz=(0.0, -0.199, 0.167), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_lcd,
        name="display_lcd",
    )

    for i, x in enumerate((-0.074, -0.038, 0.038, 0.074)):
        shell.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, -0.1955, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name=f"touch_icon_{i}",
        )

    for i, x in enumerate((-0.218, 0.218)):
        shell.visual(
            Box((0.050, 0.070, 0.080)),
            origin=Origin(xyz=(x * 0.87, 0.000, 0.220)),
            material=porcelain,
            name=f"pivot_shoulder_pad_{i}",
        )
        shell.visual(
            Cylinder(radius=0.023, length=0.024),
            origin=Origin(xyz=(x, 0.000, 0.236), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"pivot_socket_{i}",
        )

    for i, x in enumerate((-0.158, 0.158)):
        shell.visual(
            Cylinder(radius=0.013, length=0.055),
            origin=Origin(xyz=(x, 0.168, 0.258), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=soft_gray,
            name=f"rear_hinge_knuckle_{i}",
        )
        shell.visual(
            Box((0.052, 0.060, 0.040)),
            origin=Origin(xyz=(x, 0.174, 0.225)),
            material=soft_gray,
            name=f"rear_hinge_bracket_{i}",
        )

    lid = model.part("lid")
    lid_sections = [
        (-0.305, 0.000, 0.030, 0.250),
        (-0.238, 0.000, 0.052, 0.330),
        (-0.128, 0.000, 0.066, 0.382),
        (-0.040, 0.000, 0.055, 0.355),
        (-0.025, 0.000, 0.030, 0.250),
    ]
    lid_mesh = superellipse_side_loft(lid_sections, exponents=2.55, segments=72, cap=True)
    lid.visual(
        mesh_from_geometry(lid_mesh, "domed_lid_crown"),
        material=pearl,
        name="domed_lid_crown",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.115, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.014, 0.010)),
        material=pearl,
        name="rear_hinge_tab",
    )
    recess_mesh = ExtrudeGeometry(
        rounded_rect_profile(0.150, 0.052, 0.018, corner_segments=10),
        0.004,
        center=True,
    )
    lid.visual(
        mesh_from_geometry(recess_mesh, "lid_handle_recess"),
        origin=Origin(xyz=(0.0, -0.180, 0.060)),
        material=soft_gray,
        name="lid_handle_recess",
    )
    lid.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.090, -0.078, 0.060)),
        material=dark_plastic,
        name="steam_vent_cap",
    )
    for i, y in enumerate((-0.008, 0.000, 0.008)):
        lid.visual(
            Box((0.026, 0.003, 0.002)),
            origin=Origin(xyz=(0.090, -0.078 + y, 0.065)),
            material=soft_gray,
            name=f"steam_slot_{i}",
        )

    carry_handle = model.part("carry_handle")
    handle_mesh = tube_from_spline_points(
        [
            (-0.236, 0.000, 0.000),
            (-0.202, -0.020, 0.070),
            (-0.090, -0.052, 0.135),
            (0.000, -0.058, 0.150),
            (0.090, -0.052, 0.135),
            (0.202, -0.020, 0.070),
            (0.236, 0.000, 0.000),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    carry_handle.visual(
        mesh_from_geometry(handle_mesh, "carry_handle_tube"),
        material=dark_plastic,
        name="carry_handle_tube",
    )
    for i, x in enumerate((-0.236, 0.236)):
        carry_handle.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=f"handle_pivot_cap_{i}",
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_button,
        name="button_cap",
    )
    power_button.visual(
        Box((0.004, 0.0018, 0.014)),
        origin=Origin(xyz=(0.0, -0.011, 0.004)),
        material=dark_plastic,
        name="power_mark",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, 0.168, 0.258)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.28),
    )
    model.articulation(
        "carry_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.000, 0.236)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "power_button_slide",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=power_button,
        origin=Origin(xyz=(0.0, -0.194, 0.083)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("carry_handle")
    button = object_model.get_part("power_button")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivot = object_model.get_articulation("carry_handle_pivot")
    button_slide = object_model.get_articulation("power_button_slide")

    for socket_name in ("pivot_socket_0", "pivot_socket_1"):
        ctx.allow_overlap(
            handle,
            shell,
            elem_a="carry_handle_tube",
            elem_b=socket_name,
            reason="The folding bail handle tube is intentionally captured in the side pivot socket.",
        )
        ctx.expect_contact(
            handle,
            shell,
            elem_a="carry_handle_tube",
            elem_b=socket_name,
            contact_tol=0.0,
            name=f"{socket_name} captures the carry handle",
        )
        ctx.expect_overlap(
            handle,
            shell,
            axes="x",
            elem_a="carry_handle_tube",
            elem_b=socket_name,
            min_overlap=0.005,
            name=f"{socket_name} retains handle axle width",
        )

    for knuckle_name in ("rear_hinge_knuckle_0", "rear_hinge_knuckle_1"):
        ctx.allow_overlap(
            lid,
            shell,
            elem_a="lid_hinge_barrel",
            elem_b=knuckle_name,
            reason="The lid hinge barrel is intentionally captured through the fixed rear hinge knuckle.",
        )
        ctx.expect_contact(
            lid,
            shell,
            elem_a="lid_hinge_barrel",
            elem_b=knuckle_name,
            contact_tol=0.0,
            name=f"{knuckle_name} captures lid hinge barrel",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="x",
            elem_a="lid_hinge_barrel",
            elem_b=knuckle_name,
            min_overlap=0.020,
            name=f"{knuckle_name} shares hinge pin span",
        )

    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="domed_lid_crown",
        negative_elem="inner_pot_rim",
        name="closed lid rests just above oval cooker rim",
    )
    ctx.expect_contact(
        button,
        shell,
        elem_a="button_cap",
        elem_b="front_control_panel",
        contact_tol=0.004,
        name="power button cap is seated in front panel",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.070,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    upright_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivot: 1.00}):
        folded_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "carry handle rotates about side pivots",
        upright_handle_aabb is not None
        and folded_handle_aabb is not None
        and abs(folded_handle_aabb[0][1] - upright_handle_aabb[0][1]) > 0.030,
        details=f"upright={upright_handle_aabb}, folded={folded_handle_aabb}",
    )

    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.006}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "power button presses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
