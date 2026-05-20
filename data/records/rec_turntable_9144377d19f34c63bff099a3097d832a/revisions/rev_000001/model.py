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
    model = ArticulatedObject(name="retrofit_legacy_turntable")

    walnut = Material("oiled_walnut", rgba=(0.30, 0.15, 0.07, 1.0))
    black = Material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    deck_metal = Material("warm_brushed_aluminum", rgba=(0.62, 0.60, 0.54, 1.0))
    dark_steel = Material("dark_blued_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    bright_steel = Material("polished_spindle_steel", rgba=(0.83, 0.84, 0.82, 1.0))
    rubber = Material("aged_black_rubber", rgba=(0.02, 0.022, 0.020, 1.0))
    hatch_paint = Material("service_hatch_black", rgba=(0.04, 0.045, 0.045, 1.0))
    brass = Material("dull_brass", rgba=(0.72, 0.56, 0.25, 1.0))
    needle_red = Material("cartridge_red", rgba=(0.65, 0.04, 0.025, 1.0))

    plinth = model.part("plinth")

    # A heavy, old-school plinth: walnut cheeks wrapped around a modern metal deck.
    plinth.visual(
        Box((0.62, 0.42, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=black,
        name="core_box",
    )
    plinth.visual(
        Box((0.65, 0.034, 0.086)),
        origin=Origin(xyz=(0.0, 0.224, 0.068)),
        material=walnut,
        name="rear_wood_rail",
    )
    plinth.visual(
        Box((0.65, 0.034, 0.086)),
        origin=Origin(xyz=(0.0, -0.224, 0.068)),
        material=walnut,
        name="front_wood_rail",
    )
    plinth.visual(
        Box((0.034, 0.42, 0.078)),
        origin=Origin(xyz=(-0.318, 0.0, 0.064)),
        material=walnut,
        name="end_wood_rail_0",
    )
    plinth.visual(
        Box((0.034, 0.42, 0.078)),
        origin=Origin(xyz=(0.318, 0.0, 0.064)),
        material=walnut,
        name="end_wood_rail_1",
    )
    plinth.visual(
        Box((0.540, 0.335, 0.012)),
        origin=Origin(xyz=(-0.015, 0.000, 0.101)),
        material=deck_metal,
        name="top_deck",
    )

    # Rubber isolation feet are let into the plinth rather than floating below it.
    for idx, (x, y) in enumerate(
        ((-0.245, -0.155), (0.245, -0.155), (-0.245, 0.155), (0.245, 0.155))
    ):
        plinth.visual(
            Cylinder(radius=0.034, length=0.026),
            origin=Origin(xyz=(x, y, 0.013)),
            material=rubber,
            name=f"isolation_foot_{idx}",
        )

    # Visible service hatches and retrofit access plates, seated into the deck/rails.
    plinth.visual(
        Box((0.180, 0.085, 0.006)),
        origin=Origin(xyz=(0.115, 0.090, 0.108)),
        material=hatch_paint,
        name="motor_service_hatch",
    )
    plinth.visual(
        Box((0.145, 0.004, 0.055)),
        origin=Origin(xyz=(-0.030, 0.242, 0.070)),
        material=hatch_paint,
        name="rear_service_hatch",
    )
    plinth.visual(
        Box((0.115, 0.004, 0.045)),
        origin=Origin(xyz=(0.205, -0.242, 0.066)),
        material=hatch_paint,
        name="front_cable_hatch",
    )

    # Platter bearing system: bolted adapter plate, reinforcements, and a fixed bearing cap.
    platter_x, platter_y = -0.130, 0.030
    plinth.visual(
        Box((0.225, 0.175, 0.010)),
        origin=Origin(xyz=(platter_x, platter_y, 0.112)),
        material=dark_steel,
        name="platter_adapter",
    )
    plinth.visual(
        Box((0.275, 0.020, 0.018)),
        origin=Origin(xyz=(platter_x, platter_y - 0.098, 0.112)),
        material=dark_steel,
        name="front_bearing_strap",
    )
    plinth.visual(
        Box((0.275, 0.020, 0.018)),
        origin=Origin(xyz=(platter_x, platter_y + 0.098, 0.112)),
        material=dark_steel,
        name="rear_bearing_strap",
    )
    plinth.visual(
        Cylinder(radius=0.076, length=0.008),
        origin=Origin(xyz=(platter_x, platter_y, 0.118)),
        material=deck_metal,
        name="bearing_flange",
    )
    plinth.visual(
        Cylinder(radius=0.055, length=0.021),
        origin=Origin(xyz=(platter_x, platter_y, 0.1255)),
        material=bright_steel,
        name="bearing_cap",
    )

    for idx, (dx, dy) in enumerate(
        ((-0.085, -0.062), (0.085, -0.062), (-0.085, 0.062), (0.085, 0.062))
    ):
        plinth.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(platter_x + dx, platter_y + dy, 0.118)),
            material=brass,
            name=f"platter_bolt_{idx}",
        )

    # Tonearm bearing system: separate bolted adapter, tall pivot cup, and cross braces.
    pivot_x, pivot_y = 0.220, -0.130
    plinth.visual(
        Box((0.130, 0.105, 0.010)),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.112)),
        material=dark_steel,
        name="tonearm_adapter",
    )
    plinth.visual(
        Box((0.150, 0.016, 0.024)),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.122)),
        material=dark_steel,
        name="pivot_brace_x",
    )
    plinth.visual(
        Box((0.016, 0.120, 0.024)),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.122)),
        material=dark_steel,
        name="pivot_brace_y",
    )
    plinth.visual(
        Cylinder(radius=0.033, length=0.050),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.140)),
        material=bright_steel,
        name="tonearm_bearing_cup",
    )
    for idx, (dx, dy) in enumerate(
        ((-0.050, -0.038), (0.050, -0.038), (-0.050, 0.038), (0.050, 0.038))
    ):
        plinth.visual(
            Cylinder(radius=0.0075, length=0.005),
            origin=Origin(xyz=(pivot_x + dx, pivot_y + dy, 0.1175)),
            material=brass,
            name=f"tonearm_bolt_{idx}",
        )

    # Park bracket and cable clamp are fixed to the plinth, providing practical service cues.
    plinth.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(0.145, -0.188, 0.140)),
        material=dark_steel,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.145, -0.188, 0.175)),
        material=rubber,
        name="arm_rest_cradle",
    )
    plinth.visual(
        Box((0.085, 0.016, 0.018)),
        origin=Origin(xyz=(0.230, 0.168, 0.113)),
        material=dark_steel,
        name="cable_strain_clamp",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.180, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=deck_metal,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.181, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_steel,
        name="strobe_rim",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.045, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0445)),
        material=bright_steel,
        name="center_boss",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=bright_steel,
        name="record_spindle",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.025, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bright_steel,
        name="pivot_turret",
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_steel,
        name="upper_bearing",
    )
    tonearm.visual(
        Cylinder(radius=0.0055, length=0.330),
        origin=Origin(xyz=(0.145, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.060),
        origin=Origin(xyz=(-0.028, 0.0, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counterweight_stem",
    )
    tonearm.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(-0.058, 0.0, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.056, 0.026, 0.008)),
        origin=Origin(xyz=(0.310, 0.0, 0.048)),
        material=dark_steel,
        name="headshell",
    )
    tonearm.visual(
        Box((0.022, 0.018, 0.012)),
        origin=Origin(xyz=(0.334, 0.0, 0.038)),
        material=needle_red,
        name="cartridge",
    )
    tonearm.visual(
        Box((0.004, 0.003, 0.020)),
        origin=Origin(xyz=(0.343, 0.0, 0.026)),
        material=bright_steel,
        name="stylus",
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_x, platter_y, 0.136)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=9.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.002),
    )

    # The local +X direction of the tonearm frame points from its pivot toward the record.
    arm_yaw = math.atan2(0.030 - pivot_y, platter_x - pivot_x)
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.165), rpy=(0.0, 0.0, arm_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=0.7, lower=-0.55, upper=0.28),
        motion_properties=MotionProperties(damping=0.025, friction=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    ctx.check(
        "primary mechanisms are present",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tonearm_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"platter={platter_joint.articulation_type}, tonearm={tonearm_joint.articulation_type}",
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter rides on bearing cap",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="bearing_cap",
        min_overlap=0.090,
        name="platter bearing is centered under platter",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_turret",
        negative_elem="tonearm_bearing_cup",
        max_gap=0.001,
        max_penetration=0.0,
        name="tonearm turret rides on bearing cup",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="xy",
        elem_a="pivot_turret",
        elem_b="tonearm_bearing_cup",
        min_overlap=0.040,
        name="tonearm pivot is centered in cup",
    )

    lower = tonearm_joint.motion_limits.lower
    upper = tonearm_joint.motion_limits.upper
    ctx.check(
        "tonearm has realistic sweep limits",
        lower is not None and upper is not None and lower < 0.0 < upper and (upper - lower) < 1.2,
        details=f"lower={lower}, upper={upper}",
    )

    return ctx.report()


object_model = build_object_model()
