from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_camcorder")

    graphite = model.material("graphite_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    charcoal = model.material("charcoal_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    satin = model.material("satin_dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    screen_black = model.material("screen_black", rgba=(0.004, 0.006, 0.008, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.02, 0.05, 0.075, 0.88))
    metal = model.material("brushed_hinge_metal", rgba=(0.45, 0.46, 0.43, 1.0))
    white = model.material("white_print", rgba=(0.86, 0.88, 0.84, 1.0))

    body = model.part("body")

    # Consumer travel camcorder scale: a slim 160 mm long housing with softened
    # corners, a side LCD bay, and a prominent lens mounted on the front.
    body_shell = ExtrudeGeometry(
        rounded_rect_profile(0.160, 0.056, 0.010, corner_segments=8),
        0.082,
        cap=True,
        center=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "camcorder_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=graphite,
        name="body_shell",
    )

    body.visual(
        Box((0.106, 0.0016, 0.059)),
        origin=Origin(xyz=(-0.008, 0.0281, 0.059)),
        material=screen_black,
        name="screen_opening",
    )
    body.visual(
        Box((0.118, 0.0020, 0.004)),
        origin=Origin(xyz=(-0.008, 0.0288, 0.0905)),
        material=satin,
        name="screen_top_lip",
    )
    body.visual(
        Box((0.118, 0.0020, 0.004)),
        origin=Origin(xyz=(-0.008, 0.0288, 0.0275)),
        material=satin,
        name="screen_bottom_lip",
    )
    body.visual(
        Box((0.004, 0.0020, 0.064)),
        origin=Origin(xyz=(-0.065, 0.0288, 0.059)),
        material=satin,
        name="screen_rear_lip",
    )
    body.visual(
        Box((0.004, 0.0020, 0.064)),
        origin=Origin(xyz=(0.049, 0.0288, 0.059)),
        material=satin,
        name="screen_front_lip",
    )

    # Fixed portions of the vertical side hinge: top and bottom knuckles plus
    # short leaves tying the hinge line into the body wall.
    for suffix, zc in (("lower", 0.025), ("upper", 0.091)):
        body.visual(
            Cylinder(radius=0.0044, length=0.012),
            origin=Origin(xyz=(-0.060, 0.0330, zc)),
            material=metal,
            name=f"hinge_{suffix}_knuckle",
        )
        body.visual(
            Box((0.015, 0.0070, 0.011)),
            origin=Origin(xyz=(-0.060, 0.0308, zc)),
            material=metal,
            name=f"hinge_{suffix}_leaf",
        )

    # The front optical stack is attached to the body.  The rotating focus ring
    # starts at the end of this fixed barrel.
    body.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.0855, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="lens_shoulder",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.1050, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="fixed_barrel",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.0025),
        origin=Origin(xyz=(0.0790, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="rear_lens_glass",
    )

    screen = model.part("screen")
    # Child frame is the vertical hinge axis; in the closed pose the panel
    # extends along local +X and sits just proud of the left body wall.
    screen.visual(
        Cylinder(radius=0.0041, length=0.050),
        origin=Origin(),
        material=metal,
        name="hinge_spine",
    )
    screen.visual(
        Box((0.012, 0.0042, 0.044)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=metal,
        name="hinge_leaf",
    )
    screen.visual(
        Box((0.100, 0.0046, 0.058)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=satin,
        name="screen_panel",
    )
    screen.visual(
        Box((0.083, 0.0010, 0.044)),
        origin=Origin(xyz=(0.060, 0.00255, 0.0)),
        material=screen_black,
        name="lcd_glass",
    )

    model.articulation(
        "body_to_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(-0.060, 0.0330, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=0.0, upper=1.75),
    )

    focus_ring = model.part("focus_ring")
    ring_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0265, -0.0135),
            (0.0285, -0.0110),
            (0.0285, 0.0110),
            (0.0265, 0.0135),
        ],
        [
            (0.0190, -0.0135),
            (0.0190, 0.0135),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    focus_ring.visual(
        mesh_from_geometry(ring_shell, "focus_ring_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="ring_shell",
    )
    focus_ring.visual(
        Box((0.013, 0.0012, 0.0032)),
        origin=Origin(xyz=(0.000, 0.0, 0.0282)),
        material=white,
        name="focus_mark",
    )
    focus_ring.visual(
        Cylinder(radius=0.0195, length=0.0022),
        origin=Origin(xyz=(0.0124, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.1335, 0.0, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )

    button_xs = (-0.038, -0.014, 0.010)
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.014, 0.0040, 0.0070)),
            origin=Origin(xyz=(0.0, 0.0020, 0.0)),
            material=satin,
            name="button_cap",
        )
        button.visual(
            Box((0.009, 0.0007, 0.0020)),
            origin=Origin(xyz=(0.0, 0.00435, 0.0)),
            material=white,
            name="button_highlight",
        )
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.0278, 0.0215)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.003),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    screen = object_model.get_part("screen")
    focus_ring = object_model.get_part("focus_ring")
    screen_joint = object_model.get_articulation("body_to_screen")
    focus_joint = object_model.get_articulation("body_to_focus_ring")

    ctx.expect_overlap(
        screen,
        body,
        axes="xz",
        elem_a="screen_panel",
        elem_b="screen_opening",
        min_overlap=0.040,
        name="closed flip screen covers the side LCD opening",
    )
    ctx.expect_gap(
        screen,
        body,
        axis="y",
        positive_elem="screen_panel",
        negative_elem="screen_opening",
        min_gap=0.001,
        max_gap=0.006,
        name="closed screen panel sits just outside the body wall",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
    with ctx.pose({screen_joint: 1.45}):
        open_panel_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
    ctx.check(
        "flip screen swings outward on vertical hinge",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > rest_panel_aabb[1][1] + 0.055,
        details=f"rest={rest_panel_aabb}, open={open_panel_aabb}",
    )

    rest_mark_aabb = ctx.part_element_world_aabb(focus_ring, elem="focus_mark")
    with ctx.pose({focus_joint: math.pi / 2.0}):
        turned_mark_aabb = ctx.part_element_world_aabb(focus_ring, elem="focus_mark")
    ctx.check(
        "focus ring rotates continuously about the lens barrel",
        rest_mark_aabb is not None
        and turned_mark_aabb is not None
        and turned_mark_aabb[0][1] < rest_mark_aabb[0][1] - 0.015,
        details=f"rest={rest_mark_aabb}, turned={turned_mark_aabb}",
    )

    for i in range(3):
        button = object_model.get_part(f"button_{i}")
        button_joint = object_model.get_articulation(f"body_to_button_{i}")
        ctx.allow_overlap(
            button,
            body,
            elem_a="button_cap",
            elem_b="body_shell",
            reason="The push button cap is intentionally flush-seated a fraction into the molded side-wall opening for retention.",
        )
        ctx.expect_gap(
            body,
            button,
            axis="z",
            positive_elem="screen_opening",
            negative_elem="button_cap",
            min_gap=0.003,
            name=f"button_{i} is below the side screen opening",
        )
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="body_shell",
            max_gap=0.0006,
            max_penetration=0.0008,
            name=f"button_{i} cap is seated in the wall without deep penetration",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            elem_a="button_cap",
            elem_b="body_shell",
            min_overlap=0.006,
            name=f"button_{i} is set into the screen-side body wall",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.003}):
            depressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} depresses inward independently",
            rest_pos is not None
            and depressed_pos is not None
            and depressed_pos[1] < rest_pos[1] - 0.0025,
            details=f"rest={rest_pos}, depressed={depressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
