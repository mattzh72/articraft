from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_watch_winder_box")

    black_leather = Material("matte_black_leather", rgba=(0.015, 0.014, 0.013, 1.0))
    charcoal_velvet = Material("charcoal_velvet", rgba=(0.02, 0.025, 0.030, 1.0))
    smoked_acrylic = Material("smoked_acrylic", rgba=(0.20, 0.24, 0.28, 0.42))
    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.70, 0.65, 1.0))
    satin_black = Material("satin_black", rgba=(0.03, 0.032, 0.035, 1.0))
    warm_gold = Material("warm_gold", rgba=(0.92, 0.70, 0.32, 1.0))

    # Coordinate frame: +X is the front of the presentation box, +Y is across
    # its width, and +Z is upward.  The footprint is deliberately desktop sized.
    base = model.part("base")

    # Open tray: floor plus four thin walls, leaving the visible interior hollow.
    base.visual(
        Box((0.240, 0.160, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black_leather,
        name="floor",
    )
    base.visual(
        Box((0.240, 0.008, 0.081)),
        origin=Origin(xyz=(0.0, 0.076, 0.050)),
        material=black_leather,
        name="side_wall_0",
    )
    base.visual(
        Box((0.240, 0.008, 0.081)),
        origin=Origin(xyz=(0.0, -0.076, 0.050)),
        material=black_leather,
        name="side_wall_1",
    )
    base.visual(
        Box((0.008, 0.160, 0.081)),
        origin=Origin(xyz=(-0.116, 0.0, 0.050)),
        material=black_leather,
        name="rear_wall",
    )
    base.visual(
        Box((0.008, 0.160, 0.081)),
        origin=Origin(xyz=(0.116, 0.0, 0.050)),
        material=black_leather,
        name="front_wall",
    )
    base.visual(
        Box((0.206, 0.126, 0.003)),
        origin=Origin(xyz=(0.006, 0.0, 0.012)),
        material=charcoal_velvet,
        name="velvet_liner",
    )

    # Lid hinge support: three fixed knuckles on the base, with small straps
    # bridging each knuckle down to the rear wall.
    hinge_z = 0.098
    for i, y in enumerate((-0.058, 0.0, 0.058)):
        base.visual(
            Box((0.018, 0.032, 0.010)),
            origin=Origin(xyz=(-0.120, y, 0.093)),
            material=brushed_steel,
            name=f"hinge_strap_{i}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.032),
            origin=Origin(xyz=(-0.120, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"hinge_knuckle_{i}",
        )

    # Two compact bearing towers show the actual cradle support points.  The
    # rotating axle is intentionally captured inside these bushings.
    cradle_axis = (0.014, 0.0, 0.052)
    for i, y in enumerate((-0.058, 0.058)):
        base.visual(
            Box((0.024, 0.014, 0.035)),
            origin=Origin(xyz=(cradle_axis[0], y, 0.0265)),
            material=satin_black,
            name=f"bearing_post_{i}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(
                xyz=(cradle_axis[0], y, cradle_axis[2]),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"bearing_boss_{i}",
        )

    # Hinged transparent lid.  Its part frame is exactly on the hinge axis so it
    # folds back cleanly for stow-friendly access.
    lid = model.part("lid")
    lid.visual(
        Box((0.224, 0.146, 0.008)),
        origin=Origin(xyz=(0.118, 0.0, 0.006)),
        material=smoked_acrylic,
        name="window",
    )
    lid.visual(
        Box((0.224, 0.010, 0.012)),
        origin=Origin(xyz=(0.118, 0.070, 0.006)),
        material=black_leather,
        name="side_frame_0",
    )
    lid.visual(
        Box((0.224, 0.010, 0.012)),
        origin=Origin(xyz=(0.118, -0.070, 0.006)),
        material=black_leather,
        name="side_frame_1",
    )
    lid.visual(
        Box((0.014, 0.146, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.006)),
        material=black_leather,
        name="rear_frame",
    )
    lid.visual(
        Box((0.014, 0.146, 0.012)),
        origin=Origin(xyz=(0.228, 0.0, 0.006)),
        material=black_leather,
        name="front_frame",
    )
    for i, y in enumerate((-0.029, 0.029)):
        lid.visual(
            Box((0.018, 0.022, 0.008)),
            origin=Origin(xyz=(0.004, y, 0.004)),
            material=brushed_steel,
            name=f"hinge_leaf_{i}",
        )
        lid.visual(
            Cylinder(radius=0.006, length=0.022),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"hinge_barrel_{i}",
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.120, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=2.55),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    # Rotating cradle: a padded watch pillow carried on a single visible axle.
    # It is small enough to spin with the lid closed and still clear the floor.
    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0048, length=0.136),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="axle",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal_velvet,
        name="pillow",
    )
    for i, y in enumerate((-0.046, 0.046)):
        cradle.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"hub_collar_{i}",
        )
    cradle.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gold,
        name="watch_bezel",
    )
    cradle.visual(
        Cylinder(radius=0.010, length=0.005),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="watch_dial",
    )
    cradle.visual(
        Box((0.008, 0.086, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.023)),
        material=satin_black,
        name="watch_strap_top",
    )
    cradle.visual(
        Box((0.008, 0.086, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, -0.023)),
        material=satin_black,
        name="watch_strap_bottom",
    )

    model.articulation(
        "base_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=cradle_axis),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.005),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("base_to_lid")
    cradle_spin = object_model.get_articulation("base_to_cradle")

    # The rotating shaft is intentionally captured in two fixed bushings.  This
    # local hidden overlap represents a realistic axle-in-bearing fit.
    for boss in ("bearing_boss_0", "bearing_boss_1"):
        ctx.allow_overlap(
            base,
            cradle,
            elem_a=boss,
            elem_b="axle",
            reason="The cradle axle is intentionally seated inside the fixed bearing bushing.",
        )
        ctx.expect_within(
            cradle,
            base,
            axes="xz",
            inner_elem="axle",
            outer_elem=boss,
            margin=0.001,
            name=f"axle centered in {boss}",
        )
        ctx.expect_overlap(
            cradle,
            base,
            axes="y",
            elem_a="axle",
            elem_b=boss,
            min_overlap=0.010,
            name=f"axle retained by {boss}",
        )

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="front_frame",
        negative_elem="front_wall",
        min_gap=0.002,
        max_gap=0.016,
        name="closed lid clears front wall",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="window",
        elem_b="velvet_liner",
        min_overlap=0.10,
        name="lid covers tray footprint",
    )
    ctx.expect_gap(
        cradle,
        base,
        axis="z",
        positive_elem="pillow",
        negative_elem="floor",
        min_gap=0.010,
        name="cradle clears tray floor",
    )
    ctx.expect_gap(
        lid,
        cradle,
        axis="z",
        positive_elem="window",
        negative_elem="pillow",
        min_gap=0.010,
        name="closed lid clears spinning pillow envelope",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "desktop footprint remains compact",
        base_aabb is not None
        and (base_aabb[1][0] - base_aabb[0][0]) <= 0.250
        and (base_aabb[1][1] - base_aabb[0][1]) <= 0.170,
        details=f"base_aabb={base_aabb}",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({lid_hinge: 1.85}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    ctx.check(
        "lid opens upward from hinge",
        closed_front is not None
        and opened_front is not None
        and opened_front[0][2] > closed_front[0][2] + 0.120,
        details=f"closed={closed_front}, opened={opened_front}",
    )

    rest_dial = ctx.part_element_world_aabb(cradle, elem="watch_dial")
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        raised_dial = ctx.part_element_world_aabb(cradle, elem="watch_dial")
        ctx.expect_gap(
            lid,
            cradle,
            axis="z",
            positive_elem="window",
            negative_elem="watch_dial",
            min_gap=0.003,
            name="dial clears lid while spinning",
        )
    with ctx.pose({cradle_spin: -math.pi / 2.0}):
        ctx.expect_gap(
            cradle,
            base,
            axis="z",
            positive_elem="watch_dial",
            negative_elem="floor",
            min_gap=0.002,
            name="dial clears floor while spinning",
        )
    ctx.check(
        "cradle spin visibly carries watch face",
        rest_dial is not None
        and raised_dial is not None
        and raised_dial[0][2] > rest_dial[0][2] + 0.018,
        details=f"rest={rest_dial}, raised={raised_dial}",
    )

    return ctx.report()


object_model = build_object_model()
