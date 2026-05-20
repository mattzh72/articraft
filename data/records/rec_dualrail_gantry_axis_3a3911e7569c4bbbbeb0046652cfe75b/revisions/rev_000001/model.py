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
    model = ArticulatedObject(name="wide_gantry_positioning_axis")

    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    blue_anodized = Material("blue_anodized_carriage", rgba=(0.05, 0.20, 0.55, 1.0))
    black = Material("black_tooling", rgba=(0.01, 0.01, 0.012, 1.0))
    warning = Material("travel_stop_red", rgba=(0.75, 0.05, 0.03, 1.0))

    # Fixed base frame: X is the long gantry travel axis, Y is the wide span,
    # and Z is vertical.  The two long rails are bolted to a continuous base
    # plate so the root part is one supported assembly.
    base = model.part("base")
    base.visual(
        Box((0.62, 0.46, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=aluminum,
        name="base_plate",
    )
    for rail_name, stop_a_name, stop_b_name, y in (
        ("rail_0", "travel_stop_0_0", "travel_stop_0_1", -0.17),
        ("rail_1", "travel_stop_1_0", "travel_stop_1_1", 0.17),
    ):
        base.visual(
            Box((0.54, 0.026, 0.025)),
            # Sink the rail 0.5 mm into the base plate to read as bolted down.
            origin=Origin(xyz=(0.0, y, 0.0470)),
            material=dark_steel,
            name=rail_name,
        )
        base.visual(
            Box((0.040, 0.070, 0.045)),
            origin=Origin(xyz=(-0.285, y, 0.0570)),
            material=warning,
            name=stop_a_name,
        )
        base.visual(
            Box((0.040, 0.070, 0.045)),
            origin=Origin(xyz=(0.285, y, 0.0570)),
            material=warning,
            name=stop_b_name,
        )

    # First moving carriage: a stiff rectangular crossbeam with bearing saddles
    # sitting on both base rails.  The beam is broad across Y and travels along X.
    gantry = model.part("gantry")
    for saddle_name, y in (("saddle_0", -0.17), ("saddle_1", 0.17)):
        gantry.visual(
            Box((0.112, 0.070, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=blue_anodized,
            name=saddle_name,
        )
    gantry.visual(
        Box((0.125, 0.430, 0.060)),
        # Its lower face slightly keys into the saddle tops as a welded/bolted
        # connection, avoiding a floating-looking beam.
        origin=Origin(xyz=(0.0, 0.0, 0.0470)),
        material=aluminum,
        name="crossbeam",
    )
    gantry.visual(
        Box((0.010, 0.330, 0.014)),
        origin=Origin(xyz=(0.0675, 0.0, 0.060)),
        material=dark_steel,
        name="front_guide",
    )
    gantry.visual(
        Box((0.010, 0.330, 0.014)),
        origin=Origin(xyz=(0.0675, 0.0, 0.026)),
        material=dark_steel,
        name="lower_guide",
    )
    for index, y in enumerate((-0.145, 0.145)):
        gantry.visual(
            Box((0.030, 0.028, 0.045)),
            origin=Origin(xyz=(0.073, y, 0.043)),
            material=warning,
            name=f"truck_stop_{index}",
        )

    model.articulation(
        "base_to_gantry",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gantry,
        # At q=0 the saddles are near the negative-X end of the rails.  Positive
        # travel moves the whole gantry 250 mm along the base.
        origin=Origin(xyz=(-0.125, 0.0, 0.0770)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.250),
    )

    # Second moving axis: a short tool truck riding on the front guide strip of
    # the crossbeam.  It travels across the beam in Y by 120 mm.
    truck = model.part("tool_truck")
    truck.visual(
        Box((0.036, 0.095, 0.080)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=blue_anodized,
        name="truck_plate",
    )
    for index, z in enumerate((-0.025, 0.025)):
        truck.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(0.040, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"roller_{index}",
        )
    truck.visual(
        Box((0.030, 0.045, 0.052)),
        origin=Origin(xyz=(0.047, 0.0, -0.055)),
        material=black,
        name="tool_holder",
    )
    truck.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.047, 0.0, -0.073), rpy=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="tool_bit",
    )

    model.articulation(
        "gantry_to_truck",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=truck,
        # The child frame sits on the front guide face at the negative-Y end
        # of the usable stroke.  Positive q moves the truck across the beam.
        origin=Origin(xyz=(0.0725, -0.060, 0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    gantry = object_model.get_part("gantry")
    truck = object_model.get_part("tool_truck")
    base_slide = object_model.get_articulation("base_to_gantry")
    truck_slide = object_model.get_articulation("gantry_to_truck")

    ctx.check(
        "gantry carriage travel is 250 mm",
        abs((base_slide.motion_limits.upper or 0.0) - 0.250) < 1e-9
        and (base_slide.motion_limits.lower or 0.0) == 0.0,
        details=f"limits={base_slide.motion_limits}",
    )
    ctx.check(
        "tool truck travel is 120 mm",
        abs((truck_slide.motion_limits.upper or 0.0) - 0.120) < 1e-9
        and (truck_slide.motion_limits.lower or 0.0) == 0.0,
        details=f"limits={truck_slide.motion_limits}",
    )

    ctx.expect_gap(
        gantry,
        base,
        axis="z",
        positive_elem="saddle_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="gantry saddle rests on first base rail",
    )
    ctx.expect_overlap(
        gantry,
        base,
        axes="xy",
        elem_a="saddle_0",
        elem_b="rail_0",
        min_overlap=0.020,
        name="first saddle footprint remains on rail",
    )
    ctx.expect_gap(
        truck,
        gantry,
        axis="x",
        positive_elem="truck_plate",
        negative_elem="front_guide",
        max_gap=0.001,
        max_penetration=1e-6,
        name="tool truck rides on front guide face",
    )
    ctx.expect_overlap(
        truck,
        gantry,
        axes="yz",
        elem_a="truck_plate",
        elem_b="front_guide",
        min_overlap=0.010,
        name="tool truck overlaps the guide in its sliding plane",
    )

    gantry_rest = ctx.part_world_position(gantry)
    with ctx.pose({base_slide: 0.250}):
        gantry_extended = ctx.part_world_position(gantry)
        ctx.expect_overlap(
            gantry,
            base,
            axes="xy",
            elem_a="saddle_0",
            elem_b="rail_0",
            min_overlap=0.020,
            name="gantry saddle stays captured at end of travel",
        )
    ctx.check(
        "gantry moves in positive X by 250 mm",
        gantry_rest is not None
        and gantry_extended is not None
        and abs((gantry_extended[0] - gantry_rest[0]) - 0.250) < 1e-6,
        details=f"rest={gantry_rest}, extended={gantry_extended}",
    )

    truck_rest = ctx.part_world_position(truck)
    with ctx.pose({truck_slide: 0.120}):
        truck_extended = ctx.part_world_position(truck)
        ctx.expect_overlap(
            truck,
            gantry,
            axes="yz",
            elem_a="truck_plate",
            elem_b="front_guide",
            min_overlap=0.010,
            name="tool truck remains on guide at end of travel",
        )
    ctx.check(
        "tool truck moves across beam by 120 mm",
        truck_rest is not None
        and truck_extended is not None
        and abs((truck_extended[1] - truck_rest[1]) - 0.120) < 1e-6,
        details=f"rest={truck_rest}, extended={truck_extended}",
    )

    return ctx.report()


object_model = build_object_model()
