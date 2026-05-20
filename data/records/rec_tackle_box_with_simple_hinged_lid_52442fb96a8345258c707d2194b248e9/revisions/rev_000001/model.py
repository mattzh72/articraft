from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WIDTH = 0.460
DEPTH = 0.245
BASE_HEIGHT = 0.130
LID_HEIGHT = 0.072
WALL = 0.004
CORNER_R = 0.022
HINGE_PIN_R = 0.006
HINGE_BACKSET = 0.007
SEAM_GAP = 0.0015
LID_BOTTOM_Z = -HINGE_PIN_R + SEAM_GAP
HINGE_Z = BASE_HEIGHT + HINGE_PIN_R
HINGE_Y = DEPTH / 2.0 + HINGE_BACKSET
LID_CENTER_Y = -DEPTH / 2.0 - HINGE_BACKSET
LID_CENTER_Z = LID_BOTTOM_Z + LID_HEIGHT / 2.0
LATCH_XS = (-0.128, 0.128)


def _vertical_fillet(shape: cq.Workplane, radius: float) -> cq.Workplane:
    return shape.edges("|Z").fillet(radius)


def _base_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(WIDTH, DEPTH, BASE_HEIGHT)
    outer = _vertical_fillet(outer, CORNER_R)
    outer = outer.translate((0.0, 0.0, BASE_HEIGHT / 2.0))

    inner = cq.Workplane("XY").box(WIDTH - 2.0 * WALL, DEPTH - 2.0 * WALL, BASE_HEIGHT)
    inner = _vertical_fillet(inner, max(CORNER_R - WALL, 0.001))
    inner = inner.translate((0.0, 0.0, WALL + BASE_HEIGHT / 2.0))

    return outer.cut(inner)


def _lid_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(WIDTH, DEPTH, LID_HEIGHT)
    outer = _vertical_fillet(outer, CORNER_R)
    outer = outer.translate((0.0, LID_CENTER_Y, LID_CENTER_Z))

    inner = cq.Workplane("XY").box(WIDTH - 2.0 * WALL, DEPTH - 2.0 * WALL, LID_HEIGHT)
    inner = _vertical_fillet(inner, max(CORNER_R - WALL, 0.001))
    inner = inner.translate((0.0, LID_CENTER_Y, LID_BOTTOM_Z - WALL + LID_HEIGHT / 2.0))

    return outer.cut(inner)


def _rounded_slab(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    slab = cq.Workplane("XY").box(width, depth, height)
    safe_radius = min(radius, width / 2.0 - 0.0005, depth / 2.0 - 0.0005)
    if safe_radius <= 0.0002:
        return slab
    return _vertical_fillet(slab, safe_radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tackle_box")

    painted_metal = model.material("painted_metal_deep_blue", color=(0.05, 0.10, 0.16, 1.0))
    satin_metal = model.material("satin_stainless", color=(0.68, 0.70, 0.70, 1.0))
    graphite_polymer = model.material("graphite_polymer", color=(0.025, 0.027, 0.030, 1.0))
    soft_elastomer = model.material("soft_black_elastomer", color=(0.006, 0.006, 0.005, 1.0))
    shadow = model.material("shadow_gap", color=(0.0, 0.0, 0.0, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell_shape(), "base_shell", tolerance=0.0006),
        origin=Origin(),
        material=painted_metal,
        name="base_shell",
    )

    # A dark recessed seam makes the lid/base break read intentional and tight.
    base.visual(
        Box((WIDTH - 0.050, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 - 0.0015, BASE_HEIGHT - 0.006)),
        material=shadow,
        name="front_shadow_line",
    )

    # Fixed latch keepers are set into the painted front wall.
    for i, x in enumerate(LATCH_XS):
        base.visual(
            Box((0.060, 0.007, 0.016)),
            origin=Origin(xyz=(x, -DEPTH / 2.0 - 0.0010, BASE_HEIGHT - 0.039)),
            material=satin_metal,
            name=f"keeper_{i}",
        )
        base.visual(
            Cylinder(radius=0.0035, length=0.048),
            origin=Origin(
                xyz=(x, -DEPTH / 2.0 + 0.0010, BASE_HEIGHT - 0.032),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_metal,
            name=f"keeper_bar_{i}",
        )

    # Exposed rear hinge leaf, alternating knuckles, and a continuous pin.
    base.visual(
        Box((WIDTH - 0.070, 0.005, 0.032)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 + 0.002, BASE_HEIGHT - 0.014)),
        material=satin_metal,
        name="base_hinge_leaf",
    )
    for i, (x, length) in enumerate(((-0.172, 0.086), (0.0, 0.090), (0.172, 0.086))):
        base.visual(
            Cylinder(radius=0.0062, length=length),
            origin=Origin(
                xyz=(x, HINGE_Y, HINGE_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_metal,
            name=f"base_knuckle_{i}",
        )
    base.visual(
        Cylinder(radius=0.0026, length=WIDTH - 0.055),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_pin",
    )

    # Low elastomer feet are slightly sunk into the bottom shell, not floating pads.
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        base.visual(
            Cylinder(radius=0.021, length=0.006),
            origin=Origin(xyz=(sx * 0.170, sy * 0.082, -0.0015)),
            material=soft_elastomer,
            name=f"foot_{i}",
        )

    base.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, BASE_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "lid_shell", tolerance=0.0006),
        origin=Origin(),
        material=painted_metal,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_slab(0.305, 0.100, 0.003, 0.018), "top_grip_pad", tolerance=0.0005),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_BOTTOM_Z + LID_HEIGHT + 0.0005)),
        material=soft_elastomer,
        name="top_grip_pad",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_slab(0.365, 0.150, 0.0022, 0.026), "lid_recess_panel", tolerance=0.0005),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_BOTTOM_Z + LID_HEIGHT - 0.0007)),
        material=shadow,
        name="lid_recess_panel",
    )
    lid.visual(
        Box((WIDTH - 0.090, 0.0075, 0.024)),
        origin=Origin(xyz=(0.0, -0.01025, 0.005)),
        material=satin_metal,
        name="lid_hinge_leaf",
    )
    for i, x in enumerate((-0.094, 0.094)):
        lid.visual(
            Box((0.062, 0.009, 0.005)),
            origin=Origin(xyz=(x, -0.006, 0.0)),
            material=satin_metal,
            name=f"knuckle_web_{i}",
        )
        lid.visual(
            Cylinder(radius=0.0060, length=0.076),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"lid_knuckle_{i}",
        )

    for i, x in enumerate(LATCH_XS):
        # Clevis cheeks on the lid constrain each latch pivot while leaving a small
        # clearanced gap around the moving latch barrel.
        for side, dx in enumerate((-0.029, 0.029)):
            lid.visual(
                Box((0.006, 0.013, 0.020)),
                origin=Origin(xyz=(x + dx, -DEPTH - HINGE_BACKSET - 0.004, 0.032)),
                material=satin_metal,
                name=f"latch_clevis_{i}_{side}",
            )
        lid.visual(
            Cylinder(radius=0.003, length=0.066),
            origin=Origin(
                xyz=(x, -DEPTH - HINGE_BACKSET - 0.004, 0.032),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_metal,
            name=f"latch_pivot_pin_{i}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, LID_HEIGHT)),
        mass=0.75,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_CENTER_Z)),
    )

    lid_joint = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    latch_parts = []
    for i, x in enumerate(LATCH_XS):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.0051, length=0.040),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.044, 0.0055, 0.068)),
            origin=Origin(xyz=(0.0, -0.0040, -0.037)),
            material=satin_metal,
            name="lever_plate",
        )
        latch.visual(
            mesh_from_cadquery(_rounded_slab(0.034, 0.006, 0.022, 0.004), f"latch_thumb_{i}", tolerance=0.0005),
            origin=Origin(xyz=(0.0, -0.0075, -0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite_polymer,
            name="thumb_pad",
        )
        latch.visual(
            Box((0.052, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, -0.0040, -0.073)),
            material=satin_metal,
            name="hook",
        )
        latch.visual(
            Cylinder(radius=0.0065, length=0.0025),
            origin=Origin(xyz=(0.0208, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name="rivet_cap",
        )
        latch.inertial = Inertial.from_geometry(
            Box((0.052, 0.010, 0.086)),
            mass=0.05,
            origin=Origin(xyz=(0.0, -0.004, -0.040)),
        )
        latch_parts.append(latch)
        model.articulation(
            f"lid_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x, -DEPTH - HINGE_BACKSET - 0.004, 0.032)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.05),
        )

    lid_joint.meta["qc_samples"] = [0.0, 0.65, 1.20]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("base_to_lid")

    # The continuous steel hinge pin is intentionally captured inside the lid's
    # two rotating hinge knuckle proxies.
    for i in range(2):
        ctx.allow_overlap(
            base,
            lid,
            elem_a="hinge_pin",
            elem_b=f"lid_knuckle_{i}",
            reason="The fixed hinge pin is intentionally captured through the rotating lid knuckle.",
        )
        ctx.expect_overlap(
            base,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b=f"lid_knuckle_{i}",
            min_overlap=0.050,
            name=f"hinge_pin_captures_knuckle_{i}",
        )
        ctx.expect_overlap(
            base,
            lid,
            axes="yz",
            elem_a="hinge_pin",
            elem_b=f"lid_knuckle_{i}",
            min_overlap=0.003,
            name=f"hinge_axis_centered_in_knuckle_{i}",
        )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="base_shell",
            min_gap=0.0005,
            max_gap=0.0030,
            name="closed_lid_has_tight_even_seam",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="base_shell",
            min_overlap=0.190,
            name="lid_footprint_registers_on_base",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.10}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid_opens_upward_about_rear_hinge",
        closed_aabb is not None
        and open_aabb is not None
        and float(open_aabb[1][2] - closed_aabb[1][2]) > 0.110,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for i in range(2):
        latch = object_model.get_part(f"latch_{i}")
        joint = object_model.get_articulation(f"lid_to_latch_{i}")
        ctx.allow_overlap(
            lid,
            latch,
            elem_a=f"latch_pivot_pin_{i}",
            elem_b="pivot_barrel",
            reason="The lid-mounted pivot pin is intentionally captured through the latch barrel proxy.",
        )
        ctx.expect_overlap(
            lid,
            latch,
            axes="x",
            elem_a=f"latch_pivot_pin_{i}",
            elem_b="pivot_barrel",
            min_overlap=0.035,
            name=f"latch_{i}_pin_spans_barrel",
        )
        ctx.expect_overlap(
            lid,
            latch,
            axes="yz",
            elem_a=f"latch_pivot_pin_{i}",
            elem_b="pivot_barrel",
            min_overlap=0.004,
            name=f"latch_{i}_pin_centered_in_barrel",
        )
        with ctx.pose({joint: 0.0}):
            ctx.expect_gap(
                base,
                latch,
                axis="y",
                positive_elem=f"keeper_{i}",
                negative_elem="hook",
                min_gap=0.0002,
                max_gap=0.0040,
                name=f"latch_{i}_hook_sits_just_proud_of_keeper",
            )
            ctx.expect_overlap(
                latch,
                base,
                axes="xz",
                elem_a="hook",
                elem_b=f"keeper_{i}",
                min_overlap=0.008,
                name=f"latch_{i}_hook_registers_on_keeper",
            )

        locked_hook = ctx.part_element_world_aabb(latch, elem="hook")
        with ctx.pose({joint: 0.95}):
            released_hook = ctx.part_element_world_aabb(latch, elem="hook")
        ctx.check(
            f"latch_{i}_rotates_outward_to_release",
            locked_hook is not None
            and released_hook is not None
            and float(locked_hook[0][1] - released_hook[0][1]) > 0.020,
            details=f"locked={locked_hook}, released={released_hook}",
        )

    return ctx.report()


object_model = build_object_model()
