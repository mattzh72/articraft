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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_drying_rack")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.58, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.92, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))

    base = model.part("base_frame")

    # Central welded trestle: two longitudinal hinge rails, cross members,
    # four straight legs and wide rubber feet sized for an industrial rack.
    for side in (-1.0, 1.0):
        side_i = 0 if side < 0.0 else 1
        base.visual(
            Box((0.055, 1.36, 0.055)),
            origin=Origin(xyz=(side * 0.32, 0.0, 0.92)),
            material=galvanized,
            name="top_rail_0" if side_i == 0 else "top_rail_1",
        )
        base.visual(
            Box((0.06, 0.06, 0.91)),
            origin=Origin(xyz=(side * 0.27, -0.58, 0.455)),
            material=galvanized,
            name=f"front_leg_{int((side + 1) / 2)}",
        )
        base.visual(
            Box((0.06, 0.06, 0.91)),
            origin=Origin(xyz=(side * 0.27, 0.58, 0.455)),
            material=galvanized,
            name=f"rear_leg_{int((side + 1) / 2)}",
        )
        base.visual(
            Box((0.30, 0.09, 0.045)),
            origin=Origin(xyz=(side * 0.27, -0.58, 0.0225)),
            material=rubber,
            name=f"front_foot_{int((side + 1) / 2)}",
        )
        base.visual(
            Box((0.30, 0.09, 0.045)),
            origin=Origin(xyz=(side * 0.27, 0.58, 0.0225)),
            material=rubber,
            name=f"rear_foot_{int((side + 1) / 2)}",
        )
        base.visual(
            Box((0.045, 0.090, 0.095)),
            origin=Origin(xyz=(side * 0.315, -0.58, 0.55)),
            material=galvanized,
            name=f"stay_bracket_{side_i}",
        )
        # Continuous yellow hinge guard fixed to the frame, not to the wing.
        base.visual(
            Box((0.035, 1.36, 0.13)),
            origin=Origin(xyz=(side * 0.355, 0.0, 1.005)),
            material=safety_yellow,
            name=f"hinge_guard_{int((side + 1) / 2)}",
        )
        # Horizontal deployed-position stops and a taller folded stop tower.
        for yi, y in enumerate((-0.48, 0.48)):
            base.visual(
                Box((0.18, 0.055, 0.035)),
                origin=Origin(xyz=(side * 0.405, y, 0.925)),
                material=safety_yellow,
                name=(
                    "flat_stop_0_0"
                    if side_i == 0 and yi == 0
                    else "flat_stop_0_1"
                    if side_i == 0
                    else "flat_stop_1_0"
                    if yi == 0
                    else "flat_stop_1_1"
                ),
            )
            base.visual(
                Box((0.065, 0.07, 0.20)),
                origin=Origin(xyz=(side * 0.335, y, 1.0425)),
                material=safety_yellow,
                name=f"fold_stop_{int((side + 1) / 2)}_{yi}",
            )
        # Full-length hinge pin captured by alternating wing barrels.  The
        # solid pin/barrel nesting is allowed and tested below.
        base.visual(
            Cylinder(radius=0.012, length=1.34),
            origin=Origin(
                xyz=(side * 0.40, 0.0, 0.965), rpy=(-math.pi / 2.0, 0.0, 0.0)
            ),
            material=dark_steel,
            name=f"hinge_pin_{int((side + 1) / 2)}",
        )
        for yi, y in enumerate((-0.185, 0.185)):
            base.visual(
                Box((0.085, 0.045, 0.052)),
                origin=Origin(xyz=(side * 0.365, y, 0.945)),
                material=galvanized,
                name=f"pin_clevis_{int((side + 1) / 2)}_{yi}",
            )

    for yi, y in enumerate((-0.65, 0.0, 0.65)):
        base.visual(
            Box((0.69, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.92)),
            material=galvanized,
            name=f"cross_tube_{yi}",
        )

    # Repeating bolt heads make the load path readable: feet, hinge guards,
    # stop plates, and clevis blocks are visibly fastened to the welded frame.
    bolt_sites = []
    for side in (-1.0, 1.0):
        for y in (-0.56, -0.30, 0.30, 0.56):
            bolt_sites.append((side * 0.32, y, 0.9535))
        for y in (-0.48, 0.48):
            bolt_sites.append((side * 0.465, y, 0.9485))
    for i, xyz in enumerate(bolt_sites):
        base.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=xyz),
            material=dark_steel,
            name=f"bolt_{i}",
        )

    def add_wing(name: str, side: float) -> None:
        wing = model.part(name)

        # Alternating hinge barrels around the base pin.
        for bi, (y, length) in enumerate(((-0.435, 0.37), (0.0, 0.24), (0.435, 0.37))):
            wing.visual(
                Cylinder(radius=0.026, length=length),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=galvanized,
                name=f"barrel_{bi}",
            )
            # Short welded webs tie the barrel segments to the first square tube.
            wing.visual(
                Box((0.060, 0.030, 0.030)),
                origin=Origin(xyz=(side * 0.056, y, 0.0)),
                material=galvanized,
                name=f"barrel_web_{bi}",
            )

        # Rectangular wing frame and seven hanging rails.  The inner tube is set
        # outboard of the hinge barrels so the hinge pin only overlaps barrels.
        wing.visual(
            Box((0.045, 1.25, 0.045)),
            origin=Origin(xyz=(side * 0.105, 0.0, 0.0)),
            material=galvanized,
            name="inner_tube",
        )
        wing.visual(
            Box((0.055, 1.25, 0.055)),
            origin=Origin(xyz=(side * 0.69, 0.0, 0.0)),
            material=galvanized,
            name="outer_tube",
        )
        for yi, y in enumerate((-0.625, 0.625)):
            wing.visual(
                Box((0.585, 0.045, 0.045)),
                origin=Origin(xyz=(side * 0.3975, y, 0.0)),
                material=galvanized,
                name="end_tube_0" if yi == 0 else "end_tube_1",
            )
        for ri, x in enumerate((0.19, 0.27, 0.35, 0.43, 0.51, 0.59)):
            wing.visual(
                Box((0.020, 1.25, 0.020)),
                origin=Origin(xyz=(side * x, 0.0, 0.026)),
                material=dark_steel,
                name=f"hanging_rail_{ri}",
            )

        # Upturned outer guard rail with welded standoffs; yellow for pinch-point
        # visibility and to keep hands clear of swinging loads.
        for yi, y in enumerate((-0.54, 0.0, 0.54)):
            wing.visual(
                Box((0.030, 0.030, 0.135)),
                origin=Origin(xyz=(side * 0.69, y, 0.068)),
                material=safety_yellow,
                name=f"guard_post_{yi}",
            )
        wing.visual(
            Box((0.040, 1.25, 0.035)),
            origin=Origin(xyz=(side * 0.69, 0.0, 0.147)),
            material=safety_yellow,
            name="guard_rail",
        )

        # Hinge reinforcement and a lock tab at the high-stress root of the wing.
        for yi, y in enumerate((-0.53, 0.53)):
            wing.visual(
                Box((0.16, 0.075, 0.026)),
                origin=Origin(xyz=(side * 0.185, y, 0.031)),
                material=galvanized,
                name=f"root_plate_{yi}",
            )
            wing.visual(
                Cylinder(radius=0.011, length=0.010),
                origin=Origin(xyz=(side * 0.185, y, 0.049)),
                material=dark_steel,
                name=f"root_bolt_{yi}",
            )
        wing.visual(
            Box((0.075, 0.070, 0.050)),
            origin=Origin(xyz=(side * 0.135, -0.56, 0.069)),
            material=safety_yellow,
            name="lock_tab",
        )

    add_wing("wing_0", 1.0)
    add_wing("wing_1", -1.0)

    def add_link(name: str, side: float) -> None:
        link = model.part(name)
        dx = 0.39
        dz = 0.36
        length = math.hypot(dx, dz)
        angle = math.atan2(dz, dx)
        if side > 0:
            center_x = dx / 2.0
            yaw_pitch = -angle
        else:
            center_x = -dx / 2.0
            yaw_pitch = math.pi + angle
        link.visual(
            Cylinder(radius=0.023, length=0.060),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_boss",
        )
        link.visual(
            Box((length, 0.035, 0.040)),
            origin=Origin(xyz=(center_x, 0.0, dz / 2.0), rpy=(0.0, yaw_pitch, 0.0)),
            material=galvanized,
            name="diagonal_stay",
        )
        link.visual(
            Cylinder(radius=0.020, length=0.070),
            origin=Origin(xyz=(side * dx, 0.0, dz + 0.0125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="end_roller",
        )
        link.visual(
            Box((0.075, 0.045, 0.035)),
            origin=Origin(xyz=(side * (dx - 0.025), 0.0, dz - 0.018)),
            material=safety_yellow,
            name="stay_stop_pad",
        )

    add_link("hinge_link_0", 1.0)
    add_link("hinge_link_1", -1.0)

    def add_lockout(name: str, side: float) -> None:
        lockout = model.part(name)
        lockout.visual(
            Cylinder(radius=0.027, length=0.020),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="pivot_hub",
        )
        lockout.visual(
            Box((0.19, 0.035, 0.030)),
            origin=Origin(xyz=(side * 0.095, 0.0, 0.0)),
            material=lockout_red,
            name="lock_handle",
        )
        lockout.visual(
            Box((0.050, 0.055, 0.025)),
            origin=Origin(xyz=(side * 0.190, 0.0, 0.0)),
            material=lockout_red,
            name="grip_pad",
        )

    add_lockout("lockout_0", 1.0)
    add_lockout("lockout_1", -1.0)

    model.articulation(
        "base_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child="wing_0",
        origin=Origin(xyz=(0.40, 0.0, 0.965)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=0.0, upper=1.38),
    )
    model.articulation(
        "base_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child="wing_1",
        origin=Origin(xyz=(-0.40, 0.0, 0.965)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.7, lower=0.0, upper=1.38),
    )
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child="hinge_link_0",
        origin=Origin(xyz=(0.33, -0.58, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.05, upper=0.95),
    )
    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child="hinge_link_1",
        origin=Origin(xyz=(-0.33, -0.58, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.05, upper=0.95),
    )
    model.articulation(
        "base_to_lockout_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child="lockout_0",
        origin=Origin(xyz=(0.43, -0.60, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "base_to_lockout_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child="lockout_1",
        origin=Origin(xyz=(-0.43, -0.60, 1.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-1.05, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    link_0 = object_model.get_part("hinge_link_0")
    link_1 = object_model.get_part("hinge_link_1")
    lockout_part_0 = object_model.get_part("lockout_0")
    lockout_part_1 = object_model.get_part("lockout_1")
    hinge_0 = object_model.get_articulation("base_to_wing_0")
    hinge_1 = object_model.get_articulation("base_to_wing_1")
    lockout_0 = object_model.get_articulation("base_to_lockout_0")

    # The rack uses a simplified solid hinge pin captured by solid barrel
    # segments.  This is a deliberately local proxy overlap, proven by
    # containment in the pin cross-section and retained barrel overlap.
    for side_i, wing in ((1, wing_0), (0, wing_1)):
        for barrel_i in range(3):
            ctx.allow_overlap(
                base,
                wing,
                elem_a=f"hinge_pin_{side_i}",
                elem_b=f"barrel_{barrel_i}",
                reason="The solid hinge pin is intentionally nested inside the wing hinge barrel segment.",
            )
            ctx.expect_within(
                base,
                wing,
                axes="xz",
                inner_elem=f"hinge_pin_{side_i}",
                outer_elem=f"barrel_{barrel_i}",
                margin=0.001,
                name=f"hinge pin {side_i} fits barrel {barrel_i}",
            )
            ctx.expect_overlap(
                base,
                wing,
                axes="y",
                elem_a=f"hinge_pin_{side_i}",
                elem_b=f"barrel_{barrel_i}",
                min_overlap=0.18,
                name=f"hinge barrel {side_i}-{barrel_i} retains pin",
            )

    for side_i, link in ((1, link_0), (0, link_1)):
        ctx.allow_overlap(
            base,
            link,
            elem_a=f"stay_bracket_{side_i}",
            elem_b="pivot_boss",
            reason="The hinge-link pivot boss is captured inside the welded base stay bracket.",
        )
        ctx.allow_overlap(
            base,
            link,
            elem_a=f"stay_bracket_{side_i}",
            elem_b="diagonal_stay",
            reason="The stay bar root is represented as passing through the same captured pivot fork at the base bracket.",
        )
        ctx.expect_overlap(
            base,
            link,
            axes="yz",
            elem_a=f"stay_bracket_{side_i}",
            elem_b="pivot_boss",
            min_overlap=0.040,
            name=f"stay bracket {side_i} captures link pivot",
        )
        ctx.expect_overlap(
            base,
            link,
            axes="yz",
            elem_a=f"stay_bracket_{side_i}",
            elem_b="diagonal_stay",
            min_overlap=0.020,
            name=f"stay bracket {side_i} captures stay root",
        )

    for lock_part, wing in ((lockout_part_0, wing_0), (lockout_part_1, wing_1)):
        ctx.allow_overlap(
            lock_part,
            wing,
            elem_a="lock_handle",
            elem_b="lock_tab",
            reason="The red lockout handle is modeled as seated into the wing lock tab pocket in its locked position.",
        )
        ctx.expect_overlap(
            lock_part,
            wing,
            axes="xy",
            elem_a="lock_handle",
            elem_b="lock_tab",
            min_overlap=0.010,
            name=f"{lock_part.name} engages wing lock tab",
        )

    ctx.expect_contact(
        link_0,
        wing_0,
        elem_a="end_roller",
        elem_b="end_tube_0",
        contact_tol=0.001,
        name="right stay roller supports deployed wing",
    )
    ctx.expect_contact(
        link_1,
        wing_1,
        elem_a="end_roller",
        elem_b="end_tube_0",
        contact_tol=0.001,
        name="left stay roller supports deployed wing",
    )

    # Horizontal deployed pose: wing frames sit just over the stop pads and have
    # a broad useful hanging span in the long direction.
    ctx.expect_gap(
        wing_0,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="inner_tube",
        negative_elem="flat_stop_1_0",
        name="right wing rests on deployed stop",
    )
    ctx.expect_gap(
        wing_1,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="inner_tube",
        negative_elem="flat_stop_0_0",
        name="left wing rests on deployed stop",
    )
    ctx.expect_overlap(
        wing_0,
        base,
        axes="y",
        elem_a="outer_tube",
        elem_b="top_rail_1",
        min_overlap=1.0,
        name="right wing has long drying span",
    )
    ctx.expect_overlap(
        wing_1,
        base,
        axes="y",
        elem_a="outer_tube",
        elem_b="top_rail_0",
        min_overlap=1.0,
        name="left wing has long drying span",
    )

    rest_aabb_0 = ctx.part_element_world_aabb(wing_0, elem="outer_tube")
    rest_aabb_1 = ctx.part_element_world_aabb(wing_1, elem="outer_tube")
    with ctx.pose({hinge_0: 1.30, hinge_1: 1.30}):
        folded_aabb_0 = ctx.part_element_world_aabb(wing_0, elem="outer_tube")
        folded_aabb_1 = ctx.part_element_world_aabb(wing_1, elem="outer_tube")
    ctx.check(
        "wings fold upward about hinge lines",
        rest_aabb_0 is not None
        and rest_aabb_1 is not None
        and folded_aabb_0 is not None
        and folded_aabb_1 is not None
        and folded_aabb_0[1][2] > rest_aabb_0[1][2] + 0.45
        and folded_aabb_1[1][2] > rest_aabb_1[1][2] + 0.45,
        details=f"rest={rest_aabb_0},{rest_aabb_1}; folded={folded_aabb_0},{folded_aabb_1}",
    )

    lock_rest = ctx.part_element_world_aabb("lockout_0", elem="grip_pad")
    with ctx.pose({lockout_0: 0.85}):
        lock_swung = ctx.part_element_world_aabb("lockout_0", elem="grip_pad")
    ctx.check(
        "red lockout lever swings clear",
        lock_rest is not None
        and lock_swung is not None
        and abs(lock_swung[0][1] - lock_rest[0][1]) > 0.05,
        details=f"locked={lock_rest}, swung={lock_swung}",
    )

    return ctx.report()


object_model = build_object_model()
