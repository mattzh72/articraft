from __future__ import annotations

import math

import cadquery as cq

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_home_video_camcorder")

    graphite = model.material("satin_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_glass = model.material("dark_blue_glass", rgba=(0.02, 0.045, 0.075, 0.86))
    button_mat = model.material("charcoal_buttons", rgba=(0.025, 0.026, 0.028, 1.0))
    mark_mat = model.material("white_index_mark", rgba=(0.84, 0.84, 0.78, 1.0))
    strap_mat = model.material("woven_black_strap", rgba=(0.018, 0.017, 0.015, 1.0))

    body = model.part("body")

    body_shell = (
        cq.Workplane("XY")
        .box(0.160, 0.055, 0.075)
        .edges()
        .fillet(0.006)
    )
    body.visual(
        mesh_from_cadquery(body_shell, "rounded_camcorder_body", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=graphite,
        name="body_shell",
    )

    # Dark side recess left behind when the flip-out LCD is opened.
    body.visual(
        Box((0.098, 0.0012, 0.041)),
        origin=Origin(xyz=(-0.002, 0.0276, 0.047)),
        material=black,
        name="screen_recess",
    )

    # Fixed front zoom-lens base and a small, non-rotating front optic.
    body.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.089, 0.0, 0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="fixed_lens_barrel",
    )
    # A visible, two-knuckle vertical hinge supported by brackets from the body sidewall.
    for suffix, zc in (("lower", 0.020), ("upper", 0.064)):
        body.visual(
            Box((0.016, 0.011, 0.008)),
            origin=Origin(xyz=(-0.060, 0.0292, zc)),
            material=graphite,
            name=f"{suffix}_hinge_bracket",
        )
        body.visual(
            Cylinder(radius=0.0048, length=0.014),
            origin=Origin(xyz=(-0.060, 0.0335, zc)),
            material=black,
            name=f"{suffix}_hinge_knuckle",
        )

    # Hand strap on the opposite side: two mounted tabs plus a raised fabric band.
    body.visual(
        Box((0.108, 0.006, 0.020)),
        origin=Origin(xyz=(-0.006, -0.0475, 0.041)),
        material=strap_mat,
        name="strap_band",
    )
    for suffix, x in (("rear", -0.061), ("front", 0.053)):
        body.visual(
            Box((0.013, 0.025, 0.030)),
            origin=Origin(xyz=(x, -0.0365, 0.041)),
            material=black,
            name=f"{suffix}_strap_anchor",
        )

    lens_ring = model.part("lens_ring")
    lens_ring_shape = (
        cq.Workplane("YZ")
        .circle(0.032)
        .circle(0.023)
        .extrude(0.018)
    )
    lens_ring.visual(
        mesh_from_cadquery(lens_ring_shape, "hollow_zoom_lens_ring", tolerance=0.00055),
        material=rubber,
        name="zoom_ring",
    )
    lens_ring.visual(
        Cylinder(radius=0.024, length=0.002),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_glass,
        name="front_glass",
    )
    lens_ring.visual(
        Box((0.010, 0.006, 0.003)),
        origin=Origin(xyz=(0.009, 0.0, 0.033)),
        material=mark_mat,
        name="index_mark",
    )

    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.098, 0.0, 0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=5.0),
    )

    screen = model.part("side_screen")
    screen.visual(
        Cylinder(radius=0.0037, length=0.030),
        origin=Origin(),
        material=black,
        name="screen_hinge_barrel",
    )
    screen.visual(
        Box((0.012, 0.004, 0.031)),
        origin=Origin(xyz=(0.006, 0.0038, 0.0)),
        material=black,
        name="hinge_leaf",
    )
    screen.visual(
        Box((0.090, 0.006, 0.055)),
        origin=Origin(xyz=(0.052, 0.0065, 0.0)),
        material=black,
        name="screen_panel",
    )
    screen.visual(
        Box((0.070, 0.001, 0.039)),
        origin=Origin(xyz=(0.056, 0.0100, 0.0)),
        material=dark_glass,
        name="lcd_glass",
    )

    model.articulation(
        "body_to_side_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(-0.060, 0.0335, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    # Three independent push buttons in the body wall below the screen opening.
    for index, x in enumerate((-0.027, 0.0, 0.027)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.013, 0.005, 0.007)),
            origin=Origin(xyz=(0.0, 0.0022, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.0272, 0.017)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=0.08, lower=0.0, upper=0.0035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_ring = object_model.get_part("lens_ring")
    screen = object_model.get_part("side_screen")
    screen_hinge = object_model.get_articulation("body_to_side_screen")
    lens_joint = object_model.get_articulation("body_to_lens_ring")

    # The button caps are slightly seated into their wall openings at rest.
    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        ctx.allow_overlap(
            body,
            button,
            elem_a="body_shell",
            elem_b="button_cap",
            reason="The push button cap is intentionally seated a fraction of a millimeter into its body-wall opening.",
        )
        ctx.expect_gap(
            button,
            body,
            axis="y",
            max_penetration=0.0012,
            positive_elem="button_cap",
            negative_elem="body_shell",
            name=f"button_{index} is only shallowly seated",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            elem_a="button_cap",
            elem_b="body_shell",
            min_overlap=0.006,
            name=f"button_{index} lies in the side wall footprint",
        )

        rest_y = ctx.part_world_position(button)[1]
        with ctx.pose({object_model.get_articulation(f"body_to_button_{index}"): 0.0035}):
            depressed_y = ctx.part_world_position(button)[1]
        ctx.check(
            f"button_{index} depresses inward",
            depressed_y < rest_y - 0.003,
            details=f"rest_y={rest_y}, depressed_y={depressed_y}",
        )

    ctx.expect_gap(
        screen,
        body,
        axis="y",
        min_gap=0.001,
        elem_a="screen_panel",
        elem_b="body_shell",
        name="closed screen stands just outside the sidewall",
    )
    ctx.expect_overlap(
        screen,
        body,
        axes="z",
        elem_a="screen_hinge_barrel",
        elem_b="screen_recess",
        min_overlap=0.020,
        name="screen hinge spans the side opening height",
    )
    closed_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
    closed_panel_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    with ctx.pose({screen_hinge: 1.45}):
        open_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
        open_panel_y = (open_aabb[0][1] + open_aabb[1][1]) / 2.0
    ctx.check(
        "screen hinge opens outward",
        open_panel_y > closed_panel_y + 0.030,
        details=f"closed_panel_y={closed_panel_y}, open_panel_y={open_panel_y}",
    )

    ctx.expect_gap(
        lens_ring,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        elem_a="zoom_ring",
        elem_b="fixed_lens_barrel",
        name="zoom ring sits just ahead of fixed lens barrel",
    )
    ctx.check(
        "lens ring joint is continuous around optical axis",
        lens_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(lens_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={lens_joint.articulation_type}, axis={lens_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
