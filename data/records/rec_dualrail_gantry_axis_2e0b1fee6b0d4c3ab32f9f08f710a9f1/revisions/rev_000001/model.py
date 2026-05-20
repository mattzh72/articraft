from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bridge_rider_dual_rail_gantry")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    blue = model.material("machine_blue", rgba=(0.05, 0.22, 0.48, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.05, 1.0))
    polished = model.material("polished_rail", rgba=(0.72, 0.76, 0.78, 1.0))
    black = model.material("black_hardware", rgba=(0.01, 0.012, 0.014, 1.0))

    bed = model.part("bed")
    # Grounded bed: cross ties make the two rail lines read as one fixed frame.
    for x in (-0.66, 0.0, 0.66):
        bed.visual(
            Box((0.09, 0.82, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=dark_steel,
            name=f"cross_tie_{x:+.2f}",
        )
    for i, y in enumerate((-0.32, 0.32)):
        bed.visual(
            Box((1.48, 0.10, 0.05)),
            origin=Origin(xyz=(0.0, y, 0.025)),
            material=dark_steel,
            name=f"base_beam_{i}",
        )
        bed.visual(
            Box((1.42, 0.06, 0.05)),
            origin=Origin(xyz=(0.0, y, 0.075)),
            material=blue,
            name=f"rail_pedestal_{i}",
        )
        bed.visual(
            Cylinder(radius=0.020, length=1.42),
            origin=Origin(xyz=(0.0, y, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
            material=polished,
            name=f"rail_{i}",
        )
        for x in (-0.73, 0.73):
            bed.visual(
                Box((0.04, 0.15, 0.13)),
                origin=Origin(xyz=(x, y, 0.115)),
                material=orange,
                name=f"end_stop_{i}_{'neg' if x < 0 else 'pos'}",
            )

    bridge = model.part("bridge")
    # Two riding shoes sit on the grounded rails, then carry the raised cross bridge.
    for i, y in enumerate((-0.32, 0.32)):
        bridge.visual(
            Box((0.22, 0.15, 0.07)),
            origin=Origin(xyz=(0.0, y, 0.175)),
            material=black,
            name=f"rail_shoe_{i}",
        )
        for j, side in enumerate((-1.0, 1.0)):
            bridge.visual(
                Box((0.20, 0.018, 0.05)),
                origin=Origin(xyz=(0.0, y + side * 0.047, 0.120)),
                material=black,
                name=f"shoe_cheek_{i}_{j}",
            )
        bridge.visual(
            Box((0.12, 0.12, 0.345)),
            origin=Origin(xyz=(0.0, y, 0.3825)),
            material=orange,
            name=f"upright_{i}",
        )

    bridge.visual(
        Box((0.22, 0.78, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=orange,
        name="cross_beam",
    )
    # The bridge's own crosswise guide path for the compact truck.
    for i, x in enumerate((-0.055, 0.055)):
        bridge.visual(
            Box((0.030, 0.66, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.690)),
            material=polished,
            name=f"cross_guide_{i}",
        )
    for y, label in ((-0.365, "neg"), (0.365, "pos")):
        bridge.visual(
            Box((0.17, 0.03, 0.06)),
            origin=Origin(xyz=(0.0, y, 0.705)),
            material=black,
            name=f"truck_stop_{label}",
        )

    truck = model.part("truck")
    truck.visual(
        Box((0.20, 0.16, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=blue,
        name="carriage",
    )
    for i, x in enumerate((-0.055, 0.055)):
        truck.visual(
            Box((0.040, 0.14, 0.025)),
            origin=Origin(xyz=(x, 0.0, -0.0525)),
            material=black,
            name=f"bearing_pad_{i}",
        )
    truck.visual(
        Box((0.055, 0.12, 0.05)),
        origin=Origin(xyz=(0.095, 0.0, -0.025)),
        material=blue,
        name="tool_neck",
    )
    truck.visual(
        Box((0.04, 0.11, 0.22)),
        origin=Origin(xyz=(0.13, 0.0, -0.110)),
        material=blue,
        name="tool_plate",
    )
    truck.visual(
        Cylinder(radius=0.024, length=0.08),
        origin=Origin(xyz=(0.13, 0.0, -0.255)),
        material=black,
        name="tool_socket",
    )

    model.articulation(
        "bed_to_bridge",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=bridge,
        origin=Origin(xyz=(-0.50, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.55, lower=0.0, upper=1.0),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.0, -0.25, 0.770)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bed = object_model.get_part("bed")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")
    bridge_slide = object_model.get_articulation("bed_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

    ctx.check(
        "stacked perpendicular prismatic joints",
        bridge_slide.articulation_type == ArticulationType.PRISMATIC
        and truck_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(bridge_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(truck_slide.axis) == (0.0, 1.0, 0.0),
        details=f"bridge_axis={bridge_slide.axis}, truck_axis={truck_slide.axis}",
    )

    with ctx.pose({bridge_slide: 0.0, truck_slide: 0.0}):
        ctx.expect_gap(
            bridge,
            bed,
            axis="z",
            positive_elem="rail_shoe_0",
            negative_elem="rail_0",
            max_gap=0.001,
            max_penetration=0.00001,
            name="bridge shoe rests on grounded rail",
        )
        ctx.expect_overlap(
            bridge,
            bed,
            axes="xy",
            elem_a="rail_shoe_0",
            elem_b="rail_0",
            min_overlap=0.04,
            name="bridge shoe footprint stays over rail",
        )
        ctx.expect_gap(
            truck,
            bridge,
            axis="z",
            positive_elem="bearing_pad_0",
            negative_elem="cross_guide_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="truck bearing pad rides on cross guide",
        )
        ctx.expect_overlap(
            truck,
            bridge,
            axes="xy",
            elem_a="bearing_pad_0",
            elem_b="cross_guide_0",
            min_overlap=0.02,
            name="truck bearing pad remains over guide",
        )
        bridge_rest = ctx.part_world_position(bridge)
        truck_rest = ctx.part_world_position(truck)

    with ctx.pose({bridge_slide: 1.0, truck_slide: 0.50}):
        ctx.expect_gap(
            bridge,
            bed,
            axis="z",
            positive_elem="rail_shoe_0",
            negative_elem="rail_0",
            max_gap=0.001,
            max_penetration=0.00001,
            name="bridge remains rail-supported at travel end",
        )
        ctx.expect_overlap(
            bridge,
            bed,
            axes="xy",
            elem_a="rail_shoe_0",
            elem_b="rail_0",
            min_overlap=0.04,
            name="bridge remains over rail at travel end",
        )
        ctx.expect_gap(
            truck,
            bridge,
            axis="z",
            positive_elem="bearing_pad_0",
            negative_elem="cross_guide_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="truck remains guide-supported at travel end",
        )
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            inner_elem="bearing_pad_0",
            outer_elem="cross_guide_0",
            margin=0.002,
            name="truck bearing pad stays within guide length",
        )
        bridge_end = ctx.part_world_position(bridge)
        truck_end = ctx.part_world_position(truck)

    ctx.check(
        "bridge advances along rail axis",
        bridge_rest is not None
        and bridge_end is not None
        and bridge_end[0] > bridge_rest[0] + 0.95
        and abs(bridge_end[1] - bridge_rest[1]) < 1e-6,
        details=f"rest={bridge_rest}, end={bridge_end}",
    )
    ctx.check(
        "truck advances crosswise on bridge",
        truck_rest is not None
        and truck_end is not None
        and truck_end[1] > truck_rest[1] + 0.45
        and truck_end[0] > truck_rest[0] + 0.95,
        details=f"rest={truck_rest}, end={truck_end}",
    )

    return ctx.report()


object_model = build_object_model()
