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


def _hollow_body_shell(width: float, depth: float, height: float, wall: float, base: float):
    """One-piece open presentation-box body: base slab plus four connected walls."""
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, base))
    )
    return outer.cut(inner)


def _lid_wood_frame(width: float, depth: float, thickness: float, rail: float):
    """A shallow lid frame whose local origin is the rear hinge line."""
    hinge_clearance = 0.008
    outer = (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, False))
        .translate((0.0, -0.5 * depth - hinge_clearance, 0.0))
    )
    window = (
        cq.Workplane("XY")
        .box(width - 2.0 * rail, depth - 2.0 * rail, thickness + 0.006, centered=(True, True, False))
        .translate((0.0, -0.5 * depth - hinge_clearance, -0.003))
    )
    return outer.cut(window)


def _rounded_cushion(width: float, depth: float, height: float, radius: float):
    """Soft rectangular watch cushion, centered on its own local origin."""
    return cq.Workplane("XY").box(width, depth, height).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("dark_walnut", rgba=(0.20, 0.105, 0.045, 1.0))
    darker_walnut = model.material("hinge_side_walnut", rgba=(0.13, 0.065, 0.030, 1.0))
    velvet = model.material("black_velvet", rgba=(0.012, 0.011, 0.014, 1.0))
    cushion_mat = model.material("cream_suede", rgba=(0.78, 0.67, 0.50, 1.0))
    brass = model.material("brushed_brass", rgba=(0.84, 0.63, 0.25, 1.0))
    glass = model.material("smoked_glass", rgba=(0.45, 0.58, 0.66, 0.35))

    body_w = 0.340
    body_d = 0.235
    body_h = 0.095
    wall = 0.012
    base = 0.010

    main_shell = model.part("main_shell")
    main_shell.visual(
        mesh_from_cadquery(_hollow_body_shell(body_w, body_d, body_h, wall, base), "main_shell"),
        material=walnut,
        name="wood_shell",
    )
    # Velvet inset is slightly proud of the inner floor and reads as the soft presentation lining.
    main_shell.visual(
        Box((body_w - 2.0 * wall - 0.010, body_d - 2.0 * wall - 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, base + 0.001)),
        material=velvet,
        name="velvet_floor",
    )

    hinge_band = model.part("hinge_band")
    hinge_band.visual(
        Box((body_w + 0.020, 0.015, 0.070)),
        origin=Origin(xyz=(0.0, 0.5 * body_d + 0.0075, 0.056)),
        material=darker_walnut,
        name="rear_band",
    )
    # Two fixed hinge knuckles and their short leaves are mounted to the reinforcement band.
    for idx, x in enumerate((-0.106, 0.106)):
        hinge_band.visual(
            Box((0.090, 0.004, 0.010)),
            origin=Origin(xyz=(x, 0.5 * body_d + 0.002, body_h - 0.001)),
            material=brass,
            name=f"band_leaf_{idx}",
        )
        hinge_band.visual(
            Cylinder(radius=0.0055, length=0.090),
            origin=Origin(
                xyz=(x, 0.5 * body_d + 0.007, body_h + 0.005),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"band_knuckle_{idx}",
        )
    hinge_band.visual(
        Cylinder(radius=0.0025, length=0.310),
        origin=Origin(
            xyz=(0.0, 0.5 * body_d + 0.007, body_h + 0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid_w = 0.355
    lid_d = 0.250
    lid_t = 0.014
    lid.visual(
        mesh_from_cadquery(_lid_wood_frame(lid_w, lid_d, lid_t, 0.045), "lid_frame"),
        material=walnut,
        name="lid_frame",
    )
    lid.visual(
        Box((lid_w - 0.080, lid_d - 0.080, 0.003)),
        origin=Origin(xyz=(0.0, -0.5 * lid_d - 0.008, 0.0075)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.110, 0.016, 0.003)),
        origin=Origin(xyz=(0.0, -0.009, 0.004)),
        material=brass,
        name="lid_leaf",
    )
    lid.visual(
        Cylinder(radius=0.0048, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lid_knuckle",
    )

    spindle = model.part("spindle")
    pedestal_h = 0.054
    spindle_axis = (0.0, -0.020, 0.054)
    spindle.visual(
        Box((0.082, 0.036, pedestal_h)),
        origin=Origin(xyz=(0.0, 0.018, base + 0.002 + 0.5 * pedestal_h)),
        material=velvet,
        name="pedestal",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.000, spindle_axis[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.022, spindle_axis[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="motor_boss",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=velvet,
        name="back_plate",
    )
    cradle.visual(
        mesh_from_cadquery(_rounded_cushion(0.066, 0.034, 0.043, 0.006), "watch_cushion"),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=cushion_mat,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.012, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
        material=velvet,
        name="elastic_strap",
    )
    cradle.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="center_cap",
    )

    model.articulation(
        "shell_to_band",
        ArticulationType.FIXED,
        parent=main_shell,
        child=hinge_band,
        origin=Origin(),
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=main_shell,
        child=lid,
        origin=Origin(xyz=(0.0, 0.5 * body_d + 0.007, body_h + 0.005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "shell_to_spindle",
        ArticulationType.FIXED,
        parent=main_shell,
        child=spindle,
        origin=Origin(),
    )
    model.articulation(
        "spindle_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=spindle,
        child=cradle,
        origin=Origin(xyz=spindle_axis),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_shell = object_model.get_part("main_shell")
    hinge_band = object_model.get_part("hinge_band")
    lid = object_model.get_part("lid")
    spindle = object_model.get_part("spindle")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    cradle_spin = object_model.get_articulation("spindle_to_cradle")

    ctx.allow_overlap(
        hinge_band,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_knuckle",
        reason="The hinge pin is intentionally captured through the rotating lid knuckle.",
    )
    ctx.expect_overlap(
        lid,
        hinge_band,
        axes="x",
        elem_a="lid_knuckle",
        elem_b="hinge_pin",
        min_overlap=0.070,
        name="lid knuckle remains captured on the hinge pin",
    )
    ctx.expect_contact(
        main_shell,
        hinge_band,
        elem_a="wood_shell",
        elem_b="rear_band",
        contact_tol=0.0005,
        name="reinforcement band is seated on rear wall",
    )
    ctx.expect_contact(
        spindle,
        main_shell,
        elem_a="pedestal",
        elem_b="velvet_floor",
        contact_tol=0.0005,
        name="spindle pedestal sits on the lined floor",
    )
    ctx.expect_contact(
        cradle,
        spindle,
        elem_a="back_plate",
        elem_b="shaft",
        contact_tol=0.0005,
        name="cradle bears against the spindle tip",
    )
    ctx.expect_gap(
        lid,
        main_shell,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed lid is seated just above the body",
    )
    ctx.expect_overlap(
        lid,
        main_shell,
        axes="xy",
        min_overlap=0.20,
        name="lid covers the box footprint when closed",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    with ctx.pose({lid_hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    ctx.check(
        "lid opens upward about the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    strap_closed = ctx.part_element_world_aabb(cradle, elem="elastic_strap")
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        strap_rotated = ctx.part_element_world_aabb(cradle, elem="elastic_strap")
    ctx.check(
        "cradle spin visibly rotates the watch strap",
        strap_closed is not None
        and strap_rotated is not None
        and (strap_rotated[1][0] - strap_rotated[0][0]) > (strap_closed[1][0] - strap_closed[0][0]) + 0.025,
        details=f"closed={strap_closed}, rotated={strap_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
