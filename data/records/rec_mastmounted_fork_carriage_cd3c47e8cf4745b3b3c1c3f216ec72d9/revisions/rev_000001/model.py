from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_stacker_mast")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black_steel = Material("black_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.68, 0.05, 1.0))
    fork_steel = Material("fork_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    worn_steel = Material("worn_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    bronze = Material("bronze_wear_strip", rgba=(0.62, 0.42, 0.18, 1.0))
    rubber = Material("rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    stop_red = Material("red_stop", rgba=(0.72, 0.05, 0.03, 1.0))

    mast = model.part("mast")

    def box(part, size, xyz, name, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(part, radius, length, xyz, name, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Fixed mast: a low, heavy base with twin upright guide rails and stops.
    box(mast, (0.82, 0.58, 0.08), (0.0, 0.0, 0.04), "base_plate", dark_steel)
    box(mast, (0.20, 0.38, 0.08), (-0.26, -0.43, 0.08), "base_outrigger_0", dark_steel)
    box(mast, (0.20, 0.38, 0.08), (0.26, -0.43, 0.08), "base_outrigger_1", dark_steel)
    box(mast, (0.78, 0.12, 0.18), (0.0, 0.27, 0.09), "rear_counterweight", black_steel)

    mast.visual(Box((0.06, 0.09, 1.65)), origin=Origin(xyz=(-0.24, 0.0, 0.90)), material=dark_steel, name="rail_0")
    mast.visual(Box((0.020, 0.012, 1.50)), origin=Origin(xyz=(-0.24, -0.050, 0.88)), material=bronze, name="rail_wear_0")
    box(mast, (0.082, 0.025, 0.07), (-0.24, -0.055, 1.69), "top_stop_0", stop_red)
    box(mast, (0.082, 0.025, 0.07), (-0.24, -0.055, 0.18), "bottom_stop_0", stop_red)
    box(mast, (0.06, 0.09, 1.65), (0.24, 0.0, 0.90), "rail_1", dark_steel)
    box(mast, (0.020, 0.012, 1.50), (0.24, -0.050, 0.88), "rail_wear_1", bronze)
    box(mast, (0.082, 0.025, 0.07), (0.24, -0.055, 1.69), "top_stop_1", stop_red)
    box(mast, (0.082, 0.025, 0.07), (0.24, -0.055, 0.18), "bottom_stop_1", stop_red)

    mast.visual(Box((0.62, 0.12, 0.10)), origin=Origin(xyz=(0.0, 0.0, 1.775)), material=dark_steel, name="top_crosshead")
    box(mast, (0.60, 0.08, 0.08), (0.0, 0.025, 0.29), "lower_cross_tie", dark_steel)
    box(mast, (0.10, 0.08, 1.45), (0.0, 0.06, 0.805), "back_spine", black_steel)
    box(mast, (0.34, 0.045, 0.045), (0.0, 0.055, 1.39), "upper_cross_tie", dark_steel)
    box(mast, (0.34, 0.045, 0.045), (0.0, 0.055, 0.58), "middle_cross_tie", dark_steel)

    carriage = model.part("carriage")

    # Moving carriage: a single welded lift stage carrying the backrest mesh,
    # guide-capture hardware, and two fork tines tied into the lower beam.
    carriage.visual(Box((0.62, 0.08, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=safety_yellow, name="lower_beam")
    carriage.visual(Box((0.62, 0.07, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.72)), material=safety_yellow, name="upper_beam")
    carriage.visual(Box((0.07, 0.07, 0.73)), origin=Origin(xyz=(-0.31, 0.0, 0.36)), material=safety_yellow, name="side_stile_0")
    box(carriage, (0.07, 0.07, 0.73), (0.31, 0.0, 0.36), "side_stile_1", safety_yellow)
    box(carriage, (0.55, 0.055, 0.045), (0.0, 0.0, 0.38), "middle_beam", safety_yellow)

    # Welded wire-like backrest panel, attached to the carriage frame.
    for j, z in enumerate((0.18, 0.30, 0.50, 0.62)):
        box(carriage, (0.57, 0.014, 0.018), (0.0, -0.043, z), f"mesh_bar_{j}", black_steel)
    for i, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        box(carriage, (0.014, 0.014, 0.54), (x, -0.043, 0.43), f"mesh_upright_{i}", black_steel)

    # Guide pockets sit in front of the fixed rails with small running clearance.
    for i, x in enumerate((-0.24, 0.24)):
        if i == 0:
            carriage.visual(Box((0.14, 0.070, 0.10)), origin=Origin(xyz=(x, 0.010, 0.16)), material=safety_yellow, name="guide_pocket_0_lower")
        else:
            box(carriage, (0.14, 0.070, 0.10), (x, 0.010, 0.16), "guide_pocket_1_lower", safety_yellow)
        box(carriage, (0.14, 0.070, 0.10), (x, 0.010, 0.58), f"guide_pocket_{i}_upper", safety_yellow)
        box(carriage, (0.035, 0.050, 0.18), (x - 0.055, -0.020, 0.16), f"guide_cheek_{i}_lower_a", safety_yellow)
        box(carriage, (0.035, 0.050, 0.18), (x + 0.055, -0.020, 0.16), f"guide_cheek_{i}_lower_b", safety_yellow)
        box(carriage, (0.035, 0.050, 0.18), (x - 0.055, -0.020, 0.58), f"guide_cheek_{i}_upper_a", safety_yellow)
        box(carriage, (0.035, 0.050, 0.18), (x + 0.055, -0.020, 0.58), f"guide_cheek_{i}_upper_b", safety_yellow)
        if i == 0:
            carriage.visual(
                Cylinder(radius=0.024, length=0.055),
                origin=Origin(xyz=(x, 0.040, 0.16), rpy=(0.0, pi / 2.0, 0.0)),
                material=rubber,
                name="guide_roller_0_lower",
            )
        else:
            cyl(carriage, 0.024, 0.055, (x, 0.035, 0.16), "guide_roller_1_lower", rubber, rpy=(0.0, pi / 2.0, 0.0))
        cyl(carriage, 0.024, 0.055, (x, 0.035, 0.58), f"guide_roller_{i}_upper", rubber, rpy=(0.0, pi / 2.0, 0.0))
        cyl(carriage, 0.010, 0.080, (x, 0.040 if i == 0 else 0.035, 0.16), f"roller_axle_{i}_lower", worn_steel, rpy=(0.0, pi / 2.0, 0.0))
        cyl(carriage, 0.010, 0.080, (x, 0.035, 0.58), f"roller_axle_{i}_upper", worn_steel, rpy=(0.0, pi / 2.0, 0.0))
        box(carriage, (0.036, 0.030, 0.64), (x, -0.040, 0.37), f"rear_wear_shoe_{i}", bronze)

    for i, x in enumerate((-0.17, 0.17)):
        if i == 0:
            carriage.visual(Box((0.14, 0.19, 0.11)), origin=Origin(xyz=(x, -0.095, -0.025)), material=fork_steel, name="fork_root_0")
        else:
            box(carriage, (0.14, 0.19, 0.11), (x, -0.095, -0.025), "fork_root_1", fork_steel)
        box(carriage, (0.13, 0.23, 0.035), (x, -0.115, 0.035), f"fork_root_cap_{i}", worn_steel)
        box(carriage, (0.085, 0.74, 0.055), (x, -0.52, -0.065), f"fork_tine_{i}", fork_steel)
        box(carriage, (0.075, 0.075, 0.035), (x, -0.88, -0.054), f"fork_tip_{i}", worn_steel)

    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.118, 0.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.45, lower=0.0, upper=0.72),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("lift")

    ctx.allow_overlap(
        mast,
        carriage,
        elem_a="rail_wear_0",
        elem_b="guide_roller_0_lower",
        reason="A tiny rubber roller compression against the fixed wear strip grounds the stowed guide capture.",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_overlap(
            carriage,
            mast,
            axes="xz",
            min_overlap=0.035,
            elem_a="guide_pocket_0_lower",
            elem_b="rail_0",
            name="stowed guide pocket wraps rail projection",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            max_penetration=0.004,
            max_gap=0.001,
            positive_elem="rail_wear_0",
            negative_elem="guide_roller_0_lower",
            name="lower guide roller compresses wear strip locally",
        )
        ctx.expect_contact(
            carriage,
            carriage,
            elem_a="fork_root_0",
            elem_b="lower_beam",
            name="fork roots are tied into lower carriage beam",
        )
        rest_pos = ctx.part_world_position(carriage)

    with ctx.pose({lift: 0.72}):
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            min_overlap=0.18,
            elem_a="side_stile_0",
            elem_b="rail_0",
            name="raised carriage remains guided by mast",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            min_gap=0.010,
            max_gap=0.040,
            positive_elem="top_crosshead",
            negative_elem="upper_beam",
            name="raised carriage stays clear of top crosshead",
        )
        lifted_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage lift is vertical",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.70
        and abs(lifted_pos[0] - rest_pos[0]) < 1e-6
        and abs(lifted_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    return ctx.report()


object_model = build_object_model()
