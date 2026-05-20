from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cost_optimized_watch_winder_box",
        meta={
            "manufacturing_note": (
                "Three main molded assemblies: base with integral hinge lugs, "
                "lid with a single molded hinge knuckle and clipped window, and "
                "one rotating cradle carried on two molded saddle bearings."
            )
        },
    )

    abs_black = model.material("matte_black_abs", rgba=(0.015, 0.015, 0.014, 1.0))
    abs_warm = model.material("warm_gray_abs", rgba=(0.34, 0.32, 0.28, 1.0))
    felt = model.material("black_felt_liner", rgba=(0.02, 0.018, 0.016, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    brass = model.material("brushed_brass_bushing", rgba=(0.82, 0.64, 0.32, 1.0))
    clear_smoke = model.material("smoked_clear_window", rgba=(0.18, 0.22, 0.25, 0.34))

    base = model.part(
        "base",
        meta={
            "process": "single injection-molded tray with ribs, bosses, hinge lugs, and snap catch molded in"
        },
    )

    # Open-top presentation tray: straight pulls, modest wall thickness, and a
    # separate-looking soft liner bonded to the bottom.
    base.visual(
        Box((0.280, 0.200, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=abs_black,
        name="bottom_pan",
    )
    base.visual(
        Box((0.012, 0.200, 0.106)),
        origin=Origin(xyz=(-0.134, 0.0, 0.059)),
        material=abs_black,
        name="side_wall_0",
    )
    base.visual(
        Box((0.012, 0.200, 0.106)),
        origin=Origin(xyz=(0.134, 0.0, 0.059)),
        material=abs_black,
        name="side_wall_1",
    )
    base.visual(
        Box((0.280, 0.012, 0.106)),
        origin=Origin(xyz=(0.0, -0.094, 0.059)),
        material=abs_black,
        name="front_wall",
    )
    base.visual(
        Box((0.280, 0.012, 0.106)),
        origin=Origin(xyz=(0.0, 0.094, 0.059)),
        material=abs_black,
        name="rear_wall",
    )
    base.visual(
        Box((0.235, 0.145, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.014)),
        material=felt,
        name="felt_floor",
    )

    # Integrated screw bosses and ribs show the low-cost motor/deck fastening
    # path without introducing extra loose components.
    for i, x in enumerate((-0.082, 0.082)):
        base.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(x, 0.018, 0.027)),
            material=abs_black,
            name=f"screw_boss_{i}",
        )
        base.visual(
            Cylinder(radius=0.004, length=0.0025),
            origin=Origin(xyz=(x, 0.018, 0.043)),
            material=brass,
            name=f"bolt_head_{i}",
        )
        base.visual(
            Box((0.056, 0.006, 0.014)),
            origin=Origin(xyz=(x * 0.55, 0.018, 0.022)),
            material=abs_black,
            name=f"boss_rib_{i}",
        )

    # Front snap receiver: molded catch shelf tied directly into the front wall.
    base.visual(
        Box((0.068, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.084, 0.087)),
        material=abs_black,
        name="snap_catch",
    )

    # Lid hinge supports: two outer barrel lugs on the base, with reinforcing
    # pads that bridge back into the rear wall. The center gap is left for the
    # lid knuckle.
    base.visual(
        Box((0.072, 0.018, 0.026)),
        origin=Origin(xyz=(-0.081, 0.104, 0.101)),
        material=abs_black,
        name="hinge_lug_0",
    )
    base.visual(
        Cylinder(radius=0.0075, length=0.076),
        origin=Origin(xyz=(-0.081, 0.112, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=abs_black,
        name="hinge_bushing_0",
    )
    base.visual(
        Box((0.072, 0.018, 0.026)),
        origin=Origin(xyz=(0.081, 0.104, 0.101)),
        material=abs_black,
        name="hinge_lug_1",
    )
    base.visual(
        Cylinder(radius=0.0075, length=0.076),
        origin=Origin(xyz=(0.081, 0.112, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=abs_black,
        name="hinge_bushing_1",
    )

    # Two molded saddle bearings visibly support the rotating cradle shaft at
    # front and rear. They are simple straight-pull blocks, not costly inserts.
    base.visual(
        Box((0.026, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.050, 0.037)),
        material=abs_black,
        name="cradle_saddle_0",
    )
    base.visual(
        Box((0.054, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.050, 0.022)),
        material=abs_black,
        name="saddle_foot_0",
    )
    base.visual(
        Box((0.026, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.050, 0.037)),
        material=abs_black,
        name="cradle_saddle_1",
    )
    base.visual(
        Box((0.054, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.050, 0.022)),
        material=abs_black,
        name="saddle_foot_1",
    )

    # A low rear motor blister sits behind the rotating cup and is molded into
    # the tray floor, making the cradle support/drive path obvious.
    base.visual(
        Box((0.090, 0.034, 0.036)),
        origin=Origin(xyz=(0.0, 0.066, 0.034)),
        material=abs_black,
        name="motor_blister",
    )

    lid = model.part(
        "lid",
        meta={"process": "single molded frame with clipped smoked window and integral latch tongue"},
    )
    # The lid part frame is the hinge axis. In the closed pose the lid extends
    # along local -Y from the hinge line and sits just above the base rim.
    lid.visual(
        Box((0.280, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.202, 0.021)),
        material=abs_warm,
        name="front_lid_rail",
    )
    lid.visual(
        Box((0.280, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.010, 0.021)),
        material=abs_warm,
        name="rear_lid_rail",
    )
    lid.visual(
        Box((0.026, 0.198, 0.024)),
        origin=Origin(xyz=(-0.127, -0.106, 0.021)),
        material=abs_warm,
        name="side_lid_rail_0",
    )
    lid.visual(
        Box((0.026, 0.198, 0.024)),
        origin=Origin(xyz=(0.127, -0.106, 0.021)),
        material=abs_warm,
        name="side_lid_rail_1",
    )
    lid.visual(
        Box((0.228, 0.174, 0.005)),
        origin=Origin(xyz=(0.0, -0.106, 0.024)),
        material=clear_smoke,
        name="window_panel",
    )
    lid.visual(
        Box((0.076, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, -0.006, 0.006)),
        material=abs_warm,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=abs_warm,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((0.050, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.194, -0.003)),
        material=abs_warm,
        name="snap_tongue",
    )

    cradle = model.part(
        "cradle",
        meta={"process": "one-piece rotor with overmolded cushion and shaft journals"},
    )
    # The cradle frame is the spin axis. The shaft lies in the two base saddles,
    # and the cup/pillow rotate with it.
    cradle.visual(
        Cylinder(radius=0.006, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shaft",
    )
    cradle.visual(
        Cylinder(radius=0.044, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rotor_cup",
    )
    cradle.visual(
        Box((0.074, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=felt,
        name="watch_pillow",
    )
    cradle.visual(
        Box((0.085, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.026, 0.012)),
        material=rubber,
        name="retainer_band_0",
    )
    cradle.visual(
        Box((0.085, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.026, -0.012)),
        material=rubber,
        name="retainer_band_1",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.112, 0.112)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.92),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.002, 0.063)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    hinge = object_model.get_articulation("lid_hinge")
    spin = object_model.get_articulation("cradle_spin")

    ctx.check(
        "three main manufacturable assemblies",
        len(object_model.parts) == 3,
        details=f"parts={[part.name for part in object_model.parts]}",
    )

    with ctx.pose({hinge: 0.0, spin: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.001,
            max_gap=0.025,
            positive_elem="window_panel",
            negative_elem="rear_wall",
            name="closed lid clears the base rim",
        )
        ctx.expect_contact(
            lid,
            base,
            elem_a="hinge_knuckle",
            elem_b="hinge_bushing_0",
            contact_tol=0.0005,
            name="left hinge barrel is supported by base lug",
        )
        ctx.expect_contact(
            lid,
            base,
            elem_a="hinge_knuckle",
            elem_b="hinge_bushing_1",
            contact_tol=0.0005,
            name="right hinge barrel is supported by base lug",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a="shaft",
            elem_b="cradle_saddle_0",
            contact_tol=0.001,
            name="front saddle visibly supports rotating shaft",
        )
        ctx.expect_contact(
            cradle,
            base,
            elem_a="shaft",
            elem_b="cradle_saddle_1",
            contact_tol=0.001,
            name="rear saddle visibly supports rotating shaft",
        )

    closed_aabb = None
    open_aabb = None
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.60}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(
            lid,
            base,
            axis="x",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="hinge_knuckle",
            negative_elem="hinge_lug_0",
            name="open hinge clears left molded lug",
        )
        ctx.expect_gap(
            base,
            lid,
            axis="x",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="hinge_lug_1",
            negative_elem="hinge_knuckle",
            name="open hinge clears right molded lug",
        )
    ctx.check(
        "positive hinge motion opens lid upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.045,
        details=f"closed={closed_aabb}, opened={open_aabb}",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            cradle,
            base,
            axis="z",
            min_gap=0.001,
            positive_elem="rotor_cup",
            negative_elem="felt_floor",
            name="rotating cup clears liner through spin",
        )

    return ctx.report()


object_model = build_object_model()
