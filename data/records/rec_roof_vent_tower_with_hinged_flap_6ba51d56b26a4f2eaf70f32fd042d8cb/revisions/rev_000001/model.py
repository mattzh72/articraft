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
    model = ArticulatedObject(name="desktop_roof_vent_tower")

    galvanized = Material("galvanized_metal", rgba=(0.62, 0.68, 0.70, 1.0))
    darker = Material("darker_folded_metal", rgba=(0.43, 0.48, 0.50, 1.0))
    shadow = Material("dark_hollow_duct", rgba=(0.035, 0.040, 0.042, 1.0))
    rubber = Material("black_rubber_stops", rgba=(0.01, 0.01, 0.01, 1.0))
    brass = Material("brushed_pin_caps", rgba=(0.82, 0.66, 0.34, 1.0))

    tower = model.part("tower")

    # Compact desktop/apartment footprint: a shallow plinth, a hollow vent tower,
    # framed side openings, and a front weather opening covered by the flap.
    tower.visual(
        Box((0.178, 0.138, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=darker,
        name="low_base",
    )
    tower.visual(
        Box((0.126, 0.104, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=galvanized,
        name="duct_sill",
    )
    tower.visual(
        Box((0.010, 0.108, 0.176)),
        origin=Origin(xyz=(0.052, 0.0, 0.116)),
        material=galvanized,
        name="rear_wall",
    )

    # Front framed weather opening.  The middle is intentionally open, not a
    # solid placeholder.
    tower.visual(
        Box((0.012, 0.012, 0.168)),
        origin=Origin(xyz=(-0.054, -0.052, 0.116)),
        material=galvanized,
        name="front_stile_0",
    )
    tower.visual(
        Box((0.012, 0.012, 0.168)),
        origin=Origin(xyz=(-0.054, 0.052, 0.116)),
        material=galvanized,
        name="front_stile_1",
    )
    tower.visual(
        Box((0.012, 0.112, 0.014)),
        origin=Origin(xyz=(-0.054, 0.0, 0.035)),
        material=galvanized,
        name="front_bottom_rail",
    )
    tower.visual(
        Box((0.012, 0.112, 0.014)),
        origin=Origin(xyz=(-0.054, 0.0, 0.197)),
        material=galvanized,
        name="front_top_rail",
    )
    tower.visual(
        Box((0.006, 0.100, 0.150)),
        origin=Origin(xyz=(-0.046, 0.0, 0.116)),
        material=shadow,
        name="setback_dark_duct",
    )

    # Open side frames with slanted rain louvres, tied into the front/rear
    # structure so the root part remains a single coherent shell.
    for side_index, y in enumerate((-0.056, 0.056)):
        suffix = str(side_index)
        tower.visual(
            Box((0.096, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.066)),
            material=galvanized,
            name=f"side_bottom_rail_{suffix}",
        )
        tower.visual(
            Box((0.096, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.164)),
            material=galvanized,
            name=f"side_top_rail_{suffix}",
        )
        tower.visual(
            Box((0.010, 0.008, 0.108)),
            origin=Origin(xyz=(-0.043, y, 0.115)),
            material=galvanized,
            name=f"side_post_front_{suffix}",
        )
        tower.visual(
            Box((0.010, 0.008, 0.108)),
            origin=Origin(xyz=(0.043, y, 0.115)),
            material=galvanized,
            name=f"side_post_rear_{suffix}",
        )
        for slat_index, z in enumerate((0.088, 0.116, 0.144)):
            tower.visual(
                Box((0.088, 0.006, 0.006)),
                origin=Origin(xyz=(0.0, y, z), rpy=(0.0, 0.22, 0.0)),
                material=darker,
                name=f"side_louver_{suffix}_{slat_index}",
            )

    # Thin cap/roof lip keeps the envelope short while making the miniature
    # tower read as weather-shedding sheet metal.
    tower.visual(
        Box((0.132, 0.124, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, 0.210)),
        material=galvanized,
        name="flat_roof_cap",
    )
    tower.visual(
        Box((0.024, 0.118, 0.006)),
        origin=Origin(xyz=(-0.043, 0.0, 0.221)),
        material=darker,
        name="front_drip_lip",
    )

    # Fixed hinge leaf, outer barrels, pin caps, and visible stop hardware.
    hinge_x = -0.068
    hinge_z = 0.220
    tower.visual(
        Box((0.005, 0.128, 0.024)),
        origin=Origin(xyz=(-0.0615, 0.0, 0.209)),
        material=darker,
        name="fixed_hinge_plate",
    )
    for name, y in (("fixed_knuckle_0", -0.044), ("fixed_knuckle_1", 0.044)):
        tower.visual(
            Cylinder(radius=0.006, length=0.032),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker,
            name=name,
        )
    for name, y in (("pin_cap_0", -0.063), ("pin_cap_1", 0.063)):
        tower.visual(
            Cylinder(radius=0.0032, length=0.006),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=name,
        )
    for name, y in (("closed_stop_0", -0.045), ("closed_stop_1", 0.045)):
        tower.visual(
            Box((0.010, 0.016, 0.014)),
            origin=Origin(xyz=(-0.065, y, 0.058)),
            material=rubber,
            name=name,
        )
    tower.visual(
        Box((0.014, 0.122, 0.006)),
        origin=Origin(xyz=(-0.075, 0.0, 0.232)),
        material=rubber,
        name="open_stop_bar",
    )
    for name, y in (("open_stop_ear_0", -0.064), ("open_stop_ear_1", 0.064)):
        tower.visual(
            Box((0.018, 0.008, 0.030)),
            origin=Origin(xyz=(-0.071, y, 0.222)),
            material=darker,
            name=name,
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.008, 0.116, 0.168)),
        origin=Origin(xyz=(-0.006, 0.0, -0.086)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Box((0.006, 0.006, 0.160)),
        origin=Origin(xyz=(-0.011, -0.058, -0.086)),
        material=darker,
        name="side_fold_0",
    )
    flap.visual(
        Box((0.006, 0.006, 0.160)),
        origin=Origin(xyz=(-0.011, 0.058, -0.086)),
        material=darker,
        name="side_fold_1",
    )
    flap.visual(
        Box((0.014, 0.114, 0.008)),
        origin=Origin(xyz=(-0.011, 0.0, -0.171)),
        material=darker,
        name="bottom_drip_fold",
    )
    flap.visual(
        Box((0.004, 0.106, 0.022)),
        origin=Origin(xyz=(-0.004, 0.0, -0.009)),
        material=darker,
        name="moving_hinge_plate",
    )
    flap.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=darker,
        name="moving_knuckle",
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")

    # Closed/stowed pose: the weather flap folds flat against the tiny tower and
    # seats against two visible rubber stops without penetrating the frame.
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            tower,
            flap,
            axis="x",
            positive_elem="closed_stop_0",
            negative_elem="flap_panel",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="flap seats on lower stop",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="yz",
            elem_a="flap_panel",
            elem_b="setback_dark_duct",
            min_overlap=0.070,
            name="closed flap covers framed opening",
        )

    # Fully deployed pose: the hinge swings the flap outward/upward and leaves
    # clearance above the front rail, matching the visible open stop hardware.
    with ctx.pose({hinge: 1.75}):
        ctx.expect_gap(
            flap,
            tower,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="front_top_rail",
            min_gap=0.010,
            name="open flap clears front rail",
        )
        opened = ctx.part_element_world_aabb(flap, elem="flap_panel")
        ctx.check(
            "open flap moves outward",
            opened is not None and opened[1][0] < -0.068,
            details=f"opened_aabb={opened}",
        )

    return ctx.report()


object_model = build_object_model()
