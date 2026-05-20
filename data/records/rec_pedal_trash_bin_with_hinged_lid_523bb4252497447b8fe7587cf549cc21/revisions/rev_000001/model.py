from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_bathroom_step_bin")

    body_depth = 0.320
    body_width = 0.180
    body_height = 0.500
    wall = 0.008
    hinge_radius = 0.012
    hinge_x = body_depth / 2.0 + hinge_radius
    hinge_z = body_height + 0.018

    shell_mat = model.material("brushed_warm_white_steel", rgba=(0.86, 0.85, 0.80, 1.0))
    rim_mat = model.material("soft_shadow_rim", rgba=(0.13, 0.14, 0.14, 1.0))
    lid_mat = model.material("flat_white_lid", rgba=(0.95, 0.95, 0.91, 1.0))
    hinge_mat = model.material("satin_hinge_pin", rgba=(0.58, 0.57, 0.54, 1.0))
    pedal_mat = model.material("dark_rubber_pedal", rgba=(0.04, 0.045, 0.045, 1.0))

    # A tall, narrow, open-topped shell: the negative shell operation keeps the
    # body genuinely hollow under the lid instead of making it a solid block.
    body_shell = (
        cq.Workplane("XY")
        .box(body_depth, body_width, body_height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .shell(-wall)
    )

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(body_shell, "hollow_shell", tolerance=0.0008, angular_tolerance=0.08),
        material=shell_mat,
        name="body_shell",
    )

    # Rubber-like base band and small foot step recess detail are fused to the
    # root shell so the lower body reads as one supported manufactured piece.
    shell.visual(
        Box((body_depth + 0.012, body_width + 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=rim_mat,
        name="base_band",
    )
    top_shadow_rim = (
        cq.Workplane("XY")
        .box(body_depth + 0.002, body_width + 0.006, 0.010, centered=(True, True, False))
        .cut(
            cq.Workplane("XY")
            .box(body_depth - 2.8 * wall, body_width - 2.8 * wall, 0.030, centered=(True, True, False))
            .translate((0.0, 0.0, -0.010))
        )
    )
    shell.visual(
        mesh_from_cadquery(top_shadow_rim, "top_shadow_rim", tolerance=0.0008, angular_tolerance=0.08),
        origin=Origin(xyz=(0.0, 0.0, body_height - 0.010)),
        material=rim_mat,
        name="top_shadow_rim",
    )

    # Rear hinge: two fixed knuckles on the shell with a central moving knuckle
    # on the lid.  Side support webs touch the rear upper shell.
    for i, y in enumerate((-0.070, 0.070)):
        shell.visual(
            Cylinder(radius=hinge_radius, length=0.048),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"rear_hinge_knuckle_{i}",
        )
        shell.visual(
            Box((0.012, 0.052, 0.026)),
            origin=Origin(xyz=(body_depth / 2.0 + 0.006, y, body_height + 0.006)),
            material=hinge_mat,
            name=f"rear_hinge_web_{i}",
        )

    pedal_y = -0.047
    pedal_pivot_x = -body_depth / 2.0 - 0.014
    pedal_pivot_z = 0.067
    pedal_width = 0.064
    pedal_bracket_span = 0.086

    shell.visual(
        Cylinder(radius=0.0055, length=pedal_bracket_span),
        origin=Origin(xyz=(pedal_pivot_x, pedal_y, pedal_pivot_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="pedal_axle",
    )
    for i, y in enumerate((pedal_y - pedal_bracket_span / 2.0, pedal_y + pedal_bracket_span / 2.0)):
        shell.visual(
            Box((0.030, 0.007, 0.045)),
            origin=Origin(xyz=(-body_depth / 2.0 - 0.006, y, pedal_pivot_z)),
            material=hinge_mat,
            name=f"pedal_support_{i}",
        )

    lid = model.part("lid")
    lid_depth = body_depth + 0.012
    lid_width = body_width + 0.022
    lid_thickness = 0.018
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        # The panel extends forward from the rear hinge axis.  Its rear edge
        # sits just in front of the hinge barrel so the stationary hinge knuckles
        # can occupy the rear edge without colliding with the flat lid.
        origin=Origin(xyz=(-(lid_depth / 2.0 + hinge_radius), 0.0, -0.003)),
        material=lid_mat,
        name="lid_panel",
    )
    lid.visual(
        Box((0.018, lid_width - 0.020, 0.010)),
        origin=Origin(xyz=(-(lid_depth + hinge_radius - 0.012), 0.0, -0.012)),
        material=lid_mat,
        name="front_lip",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="center_hinge_knuckle",
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.012, length=pedal_width),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pedal_mat,
        name="pedal_hub",
    )
    pedal.visual(
        Box((0.095, pedal_width, 0.014)),
        origin=Origin(xyz=(-0.058, 0.0, -0.019)),
        material=pedal_mat,
        name="pedal_plate",
    )
    pedal.visual(
        Box((0.046, pedal_width * 0.84, 0.010)),
        origin=Origin(xyz=(-0.031, 0.0, -0.011)),
        material=pedal_mat,
        name="pedal_neck",
    )
    for i, x in enumerate((-0.082, -0.058, -0.034)):
        pedal.visual(
            Box((0.006, pedal_width * 0.78, 0.003)),
            origin=Origin(xyz=(x, 0.0, -0.0105)),
            material=rim_mat,
            name=f"grip_rib_{i}",
        )

    model.articulation(
        "shell_to_pedal",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(pedal_pivot_x, pedal_y, pedal_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.45, upper=0.16),
        motion_properties=MotionProperties(damping=0.04, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    pedal_hinge = object_model.get_articulation("shell_to_pedal")

    ctx.allow_overlap(
        shell,
        pedal,
        elem_a="pedal_axle",
        elem_b="pedal_hub",
        reason="The fixed axle is intentionally captured inside the pedal hub.",
    )

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            min_gap=0.003,
            max_gap=0.010,
            name="closed lid sits just above the shell rim",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.160,
            name="flat lid covers the narrow top opening",
        )
        ctx.expect_overlap(
            shell,
            pedal,
            axes="y",
            elem_a="pedal_axle",
            elem_b="pedal_hub",
            min_overlap=0.055,
            name="pedal hub spans the transverse axle",
        )
        ctx.expect_within(
            shell,
            pedal,
            axes="xz",
            inner_elem="pedal_axle",
            outer_elem="pedal_hub",
            margin=0.001,
            name="axle is centered inside the pedal hub",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "lid hinge raises the front lip",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[0][2] > closed_lid_aabb[0][2] + 0.16,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    with ctx.pose({pedal_hinge: -0.35}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    ctx.check(
        "pedal pivots downward when pressed",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.015,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
