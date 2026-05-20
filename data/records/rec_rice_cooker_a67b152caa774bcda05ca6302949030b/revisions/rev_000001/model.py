from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)
import cadquery as cq


def _rounded_rect_slab(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """CadQuery slab with rounded vertical corners, authored in meters."""
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def _rim_mesh() -> cq.Workplane:
    outer = _rounded_rect_slab(0.330, 0.365, 0.006, 0.075)
    inner = (
        cq.Workplane("XY")
        .box(0.250, 0.285, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.055)
        .translate((0.0, 0.0, -0.003))
    )
    return outer.cut(inner)


def _lid_shell_mesh() -> cq.Workplane:
    base = _rounded_rect_slab(0.315, 0.345, 0.018, 0.075)
    dome = _rounded_rect_slab(0.238, 0.268, 0.036, 0.060).translate((0.0, 0.0, 0.016))
    dome = dome.edges(">Z").fillet(0.010)
    return base.union(dome).translate((0.175, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    shell_mat = model.material("warm_white_shell", rgba=(0.92, 0.89, 0.82, 1.0))
    lid_mat = model.material("slightly_lighter_lid", rgba=(0.96, 0.94, 0.88, 1.0))
    seam_mat = model.material("dark_lid_seam", rgba=(0.08, 0.08, 0.075, 1.0))
    panel_mat = model.material("gloss_black_panel", rgba=(0.015, 0.017, 0.020, 1.0))
    display_mat = model.material("smoked_display", rgba=(0.05, 0.12, 0.15, 1.0))
    rubber_mat = model.material("dark_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    handle_mat = model.material("satin_grey_handle", rgba=(0.48, 0.49, 0.47, 1.0))
    button_mat = model.material("soft_power_button", rgba=(0.95, 0.34, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_rect_slab(0.315, 0.355, 0.210, 0.072), "rounded_body_shell"),
        material=shell_mat,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_rim_mesh(), "dark_top_seam_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.211)),
        material=seam_mat,
        name="top_seam",
    )
    body.visual(
        Box((0.012, 0.155, 0.105)),
        origin=Origin(xyz=(0.161, 0.0, 0.118)),
        material=panel_mat,
        name="control_panel",
    )
    body.visual(
        Box((0.004, 0.105, 0.034)),
        origin=Origin(xyz=(0.169, 0.0, 0.150)),
        material=display_mat,
        name="display_window",
    )
    body.visual(
        Box((0.008, 0.085, 0.020)),
        origin=Origin(xyz=(0.168, 0.0, 0.202)),
        material=seam_mat,
        name="latch_catch",
    )
    body.visual(
        Box((0.040, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, -0.184, 0.224)),
        material=handle_mat,
        name="pivot_bracket_0",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, -0.184, 0.242), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_socket_0",
    )
    body.visual(
        Box((0.040, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, 0.184, 0.224)),
        material=handle_mat,
        name="pivot_bracket_1",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, 0.184, 0.242), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_socket_1",
    )
    for idx, (x, y) in enumerate(((-0.095, -0.110), (-0.095, 0.110), (0.105, -0.110), (0.105, 0.110))):
        body.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(x, y, 0.005)),
            material=rubber_mat,
            name=f"foot_{idx}",
        )
    body.visual(
        Cylinder(radius=0.011, length=0.054),
        origin=Origin(xyz=(-0.158, -0.105, 0.226), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam_mat,
        name="rear_hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.054),
        origin=Origin(xyz=(-0.158, 0.105, 0.226), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam_mat,
        name="rear_hinge_barrel_1",
    )
    body.visual(
        Box((0.018, 0.052, 0.018)),
        origin=Origin(xyz=(-0.150, -0.105, 0.216)),
        material=seam_mat,
        name="hinge_mount_0",
    )
    body.visual(
        Box((0.018, 0.052, 0.018)),
        origin=Origin(xyz=(-0.150, 0.105, 0.216)),
        material=seam_mat,
        name="hinge_mount_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_mesh(), "rounded_lid_shell"),
        material=lid_mat,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.230, 0.050, 0.053)),
        material=seam_mat,
        name="steam_vent",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam_mat,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.030, 0.070, 0.016)),
        origin=Origin(xyz=(0.022, 0.0, 0.006)),
        material=seam_mat,
        name="hinge_tab",
    )
    lid.visual(
        Box((0.022, 0.082, 0.018)),
        origin=Origin(xyz=(0.304, 0.0, 0.006)),
        material=seam_mat,
        name="front_latch",
    )

    handle = model.part("carry_handle")
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.000, -0.142, 0.070),
                    (0.000, -0.070, 0.112),
                    (0.000, 0.000, 0.124),
                    (0.000, 0.070, 0.112),
                    (0.000, 0.142, 0.070),
                ],
                radius=0.007,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "carry_handle_arc",
        ),
        material=handle_mat,
        name="carry_arc",
    )
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.000, -0.170, 0.010),
                    (0.000, -0.160, 0.040),
                    (0.000, -0.142, 0.070),
                ],
                radius=0.007,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
            "carry_handle_side_0",
        ),
        material=handle_mat,
        name="side_arm_0",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, -0.184, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_pin_0",
    )
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.000, 0.170, 0.010),
                    (0.000, 0.160, 0.040),
                    (0.000, 0.142, 0.070),
                ],
                radius=0.007,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
            "carry_handle_side_1",
        ),
        material=handle_mat,
        name="side_arm_1",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.184, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_pin_1",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_mat,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_mat,
        name="button_stem",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.158, 0.0, 0.226)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.171, 0.0, 0.086)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("carry_handle")
    power_button = object_model.get_part("power_button")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")
    button_slide = object_model.get_articulation("button_slide")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="pivot_socket_0",
        elem_b="pivot_pin_0",
        reason="The folding carry handle pin is intentionally captured inside the side shoulder socket.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="pivot_socket_1",
        elem_b="pivot_pin_1",
        reason="The folding carry handle pin is intentionally captured inside the side shoulder socket.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="pivot_bracket_0",
        elem_b="pivot_pin_0",
        reason="The shoulder bracket is a solid proxy for a bored boss around the captured handle pin.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="pivot_bracket_1",
        elem_b="pivot_pin_1",
        reason="The shoulder bracket is a solid proxy for a bored boss around the captured handle pin.",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xz",
        elem_a="pivot_pin_0",
        elem_b="pivot_socket_0",
        min_overlap=0.010,
        name="carry handle pin 0 is coaxial with its shoulder socket",
    )
    ctx.expect_gap(
        body,
        handle,
        axis="y",
        positive_elem="pivot_socket_0",
        negative_elem="pivot_pin_0",
        max_penetration=0.026,
        name="carry handle pin 0 has only local captured-socket penetration",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xz",
        elem_a="pivot_pin_1",
        elem_b="pivot_socket_1",
        min_overlap=0.010,
        name="carry handle pin 1 is coaxial with its shoulder socket",
    )
    ctx.expect_gap(
        body,
        handle,
        axis="y",
        positive_elem="pivot_socket_1",
        negative_elem="pivot_pin_1",
        max_penetration=0.026,
        name="carry handle pin 1 has only local captured-socket penetration",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xz",
        elem_a="pivot_pin_0",
        elem_b="pivot_bracket_0",
        min_overlap=0.010,
        name="carry handle pin 0 passes through the shoulder bracket boss",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xz",
        elem_a="pivot_pin_1",
        elem_b="pivot_bracket_1",
        min_overlap=0.010,
        name="carry handle pin 1 passes through the shoulder bracket boss",
    )

    ctx.allow_overlap(
        body,
        power_button,
        elem_a="control_panel",
        elem_b="button_stem",
        reason="The push button stem intentionally enters the control-panel pocket while the cap remains proud.",
    )
    ctx.expect_overlap(
        power_button,
        body,
        axes="yz",
        elem_a="button_stem",
        elem_b="control_panel",
        min_overlap=0.014,
        name="power button stem is retained in the control panel",
    )
    ctx.expect_gap(
        power_button,
        body,
        axis="x",
        positive_elem="button_stem",
        negative_elem="control_panel",
        max_penetration=0.008,
        name="power button stem insertion is shallow and local",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="top_seam",
        min_gap=0.004,
        max_gap=0.014,
        name="closed lid leaves a clear dark seam above the body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="top_seam",
        min_overlap=0.20,
        name="lid footprint covers the cooker rim",
    )
    ctx.expect_gap(
        handle,
        lid,
        axis="z",
        positive_elem="carry_arc",
        negative_elem="lid_shell",
        min_gap=0.018,
        name="carry handle arc stays visibly separate above the lid shell",
    )
    ctx.expect_gap(
        body,
        power_button,
        axis="z",
        positive_elem="display_window",
        negative_elem="button_cap",
        min_gap=0.020,
        max_gap=0.060,
        name="power button sits below the display",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(lid, elem="front_latch")
    with ctx.pose({lid_hinge: 1.10}):
        open_latch_aabb = ctx.part_element_world_aabb(lid, elem="front_latch")
    ctx.check(
        "rear hinge lifts the front latch upward",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.18,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    rest_arc_aabb = ctx.part_element_world_aabb(handle, elem="carry_arc")
    with ctx.pose({handle_pivot: 1.00}):
        folded_arc_aabb = ctx.part_element_world_aabb(handle, elem="carry_arc")
    ctx.check(
        "carry handle rotates forward on the shoulder pivots",
        rest_arc_aabb is not None
        and folded_arc_aabb is not None
        and folded_arc_aabb[1][0] > rest_arc_aabb[1][0] + 0.070,
        details=f"rest={rest_arc_aabb}, folded={folded_arc_aabb}",
    )

    rest_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({button_slide: 0.006}):
        pressed_button_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button slides inward when pressed",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] < rest_button_pos[0] - 0.004,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
