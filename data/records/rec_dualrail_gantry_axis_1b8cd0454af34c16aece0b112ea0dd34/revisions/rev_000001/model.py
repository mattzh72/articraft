from __future__ import annotations

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
    model = ArticulatedObject(name="wide_machine_gantry")

    cast_iron = Material("painted_cast_iron", rgba=(0.16, 0.18, 0.20, 1.0))
    rail_steel = Material("ground_rail_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = Material("darkened_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_blue = Material("safety_blue_covers", rgba=(0.07, 0.22, 0.42, 1.0))
    carriage_gray = Material("carriage_gray", rgba=(0.34, 0.36, 0.37, 1.0))
    warning_red = Material("red_stop_blocks", rgba=(0.58, 0.07, 0.05, 1.0))

    base = model.part("base")
    # Two long, parallel base rails connected by bolted cross ties.  The steel
    # strips are intentionally narrow machined wear pads on top of heavier
    # painted castings.
    for idx, y in enumerate((-0.66, 0.66)):
        base.visual(
            Box((3.55, 0.22, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.06)),
            material=cast_iron,
            name=f"base_rail_{idx}",
        )
        base.visual(
            Box((3.24, 0.075, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.1375)),
            material=rail_steel,
            name=f"rail_pad_{idx}",
        )
        # Small bolt heads visually tie each machined pad to the casting.
        for bolt_idx, x in enumerate((-1.34, -0.67, 0.0, 0.67, 1.34)):
            base.visual(
                Cylinder(radius=0.022, length=0.010),
                origin=Origin(xyz=(x, y, 0.160)),
                material=dark_steel,
                name=f"pad_bolt_{idx}_{bolt_idx}",
            )

    for idx, x in enumerate((-1.32, 0.0, 1.32)):
        base.visual(
            Box((0.20, 1.56, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.045)),
            material=cast_iron,
            name=f"cross_tie_{idx}",
        )

    for idx, x in enumerate((-1.60, 1.60)):
        for side, y in enumerate((-0.66, 0.66)):
            base.visual(
                Box((0.10, 0.18, 0.12)),
                origin=Origin(xyz=(x, y, 0.21)),
                material=warning_red,
                name=f"travel_stop_{idx}_{side}",
            )

    portal = model.part("portal")
    # The portal frame rides on both rails: stout trucks, upright cheeks, and a
    # broad box beam form a connected moving bridge.
    for idx, y in enumerate((-0.66, 0.66)):
        portal.visual(
            Box((0.52, 0.34, 0.20)),
            origin=Origin(xyz=(0.0, y, 0.250)),
            material=dark_steel,
            name=f"truck_{idx}",
        )
        portal.visual(
            Box((0.42, 0.24, 0.075)),
            origin=Origin(xyz=(0.0, y, 0.1875)),
            material=rail_steel,
            name=f"bearing_shoe_{idx}",
        )
        portal.visual(
            Box((0.36, 0.30, 0.98)),
            origin=Origin(xyz=(0.0, y, 0.820)),
            material=safety_blue,
            name=f"upright_{idx}",
        )

    portal.visual(
        Box((0.44, 1.72, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 1.315)),
        material=safety_blue,
        name="portal_beam",
    )
    portal.visual(
        Box((0.12, 1.62, 0.055)),
        origin=Origin(xyz=(-0.250, 0.0, 1.5025)),
        material=safety_blue,
        name="top_cover_flange",
    )
    portal.visual(
        Box((0.12, 1.62, 0.055)),
        origin=Origin(xyz=(-0.250, 0.0, 1.1275)),
        material=safety_blue,
        name="bottom_cover_flange",
    )
    for idx, z in enumerate((1.205, 1.425)):
        portal.visual(
            Box((0.14, 1.34, 0.050)),
            origin=Origin(xyz=(-0.290, 0.0, z)),
            material=rail_steel,
            name=f"front_guide_{idx}",
        )
    for idx, y in enumerate((-0.755, 0.755)):
        portal.visual(
            Box((0.16, 0.070, 0.30)),
            origin=Origin(xyz=(-0.285, y, 1.315)),
            material=warning_red,
            name=f"carriage_stop_{idx}",
        )

    carriage = model.part("carriage")
    # The carriage is compact and nested against the front of the portal beam.
    # Its rear bearing pads contact the two front guide rails, while the central
    # block and front service cover remain clear of the flanges and stops.
    carriage.visual(
        Box((0.160, 0.300, 0.260)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=carriage_gray,
        name="carriage_housing",
    )
    for idx, z in enumerate((-0.110, 0.110)):
        carriage.visual(
            Box((0.045, 0.330, 0.060)),
            origin=Origin(xyz=(-0.0225, 0.0, z)),
            material=dark_steel,
            name=f"bearing_pad_{idx}",
        )
    carriage.visual(
        Box((0.045, 0.240, 0.300)),
        origin=Origin(xyz=(-0.1825, 0.0, 0.0)),
        material=dark_steel,
        name="front_tool_plate",
    )
    carriage.visual(
        Box((0.060, 0.120, 0.120)),
        origin=Origin(xyz=(-0.235, 0.0, -0.010)),
        material=dark_steel,
        name="tool_mount_boss",
    )

    model.articulation(
        "portal_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=portal,
        origin=Origin(xyz=(-1.00, 0.0, 0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.55, lower=0.0, upper=2.00),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=portal,
        child=carriage,
        origin=Origin(xyz=(-0.360, -0.450, 1.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.45, lower=0.0, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    portal = object_model.get_part("portal")
    carriage = object_model.get_part("carriage")
    portal_slide = object_model.get_articulation("portal_slide")
    carriage_slide = object_model.get_articulation("carriage_slide")

    with ctx.pose({portal_slide: 0.0, carriage_slide: 0.0}):
        for idx in (0, 1):
            ctx.expect_gap(
                portal,
                base,
                axis="z",
                positive_elem=f"truck_{idx}",
                negative_elem=f"rail_pad_{idx}",
                max_gap=0.001,
                max_penetration=1e-6,
                name=f"portal truck {idx} rides on rail pad",
            )
            ctx.expect_overlap(
                portal,
                base,
                axes="xy",
                elem_a=f"truck_{idx}",
                elem_b=f"rail_pad_{idx}",
                min_overlap=0.060,
                name=f"portal truck {idx} is supported by rail footprint",
            )
        ctx.expect_gap(
            portal,
            base,
            axis="x",
            positive_elem="truck_0",
            negative_elem="travel_stop_0_0",
            min_gap=0.05,
            name="portal clears the rear travel stop at the lower limit",
        )
        ctx.expect_gap(
            portal,
            carriage,
            axis="x",
            positive_elem="front_guide_0",
            negative_elem="bearing_pad_0",
            max_gap=0.001,
            max_penetration=1e-6,
            name="carriage lower bearing is seated on the front guide",
        )
        ctx.expect_gap(
            portal,
            carriage,
            axis="x",
            positive_elem="front_guide_1",
            negative_elem="bearing_pad_1",
            max_gap=0.001,
            max_penetration=1e-6,
            name="carriage upper bearing is seated on the front guide",
        )
        ctx.expect_within(
            carriage,
            portal,
            axes="y",
            inner_elem="carriage_housing",
            outer_elem="front_guide_0",
            margin=0.0,
            name="carriage starts inside the front guide span",
        )
        ctx.expect_gap(
            carriage,
            portal,
            axis="y",
            positive_elem="carriage_housing",
            negative_elem="carriage_stop_0",
            min_gap=0.08,
            name="carriage clears the negative beam stop",
        )

    rest_portal = ctx.part_world_position(portal)
    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({portal_slide: 2.0, carriage_slide: 0.90}):
        for idx in (0, 1):
            ctx.expect_gap(
                portal,
                base,
                axis="z",
                positive_elem=f"truck_{idx}",
                negative_elem=f"rail_pad_{idx}",
                max_gap=0.001,
                max_penetration=1e-6,
                name=f"extended portal truck {idx} remains on its rail pad",
            )
            ctx.expect_overlap(
                portal,
                base,
                axes="xy",
                elem_a=f"truck_{idx}",
                elem_b=f"rail_pad_{idx}",
                min_overlap=0.060,
                name=f"extended portal truck {idx} retains rail support",
            )
        ctx.expect_gap(
            base,
            portal,
            axis="x",
            positive_elem="travel_stop_1_0",
            negative_elem="truck_0",
            min_gap=0.05,
            name="portal clears the forward travel stop at the upper limit",
        )
        ctx.expect_within(
            carriage,
            portal,
            axes="y",
            inner_elem="carriage_housing",
            outer_elem="front_guide_0",
            margin=0.0,
            name="carriage stays inside the front guide span at max travel",
        )
        ctx.expect_gap(
            portal,
            carriage,
            axis="x",
            positive_elem="front_guide_0",
            negative_elem="bearing_pad_0",
            max_gap=0.001,
            max_penetration=1e-6,
            name="extended carriage lower bearing stays seated",
        )
        ctx.expect_gap(
            portal,
            carriage,
            axis="x",
            positive_elem="front_guide_1",
            negative_elem="bearing_pad_1",
            max_gap=0.001,
            max_penetration=1e-6,
            name="extended carriage upper bearing stays seated",
        )
        ctx.expect_gap(
            portal,
            carriage,
            axis="y",
            positive_elem="carriage_stop_1",
            negative_elem="carriage_housing",
            min_gap=0.08,
            name="carriage clears the positive beam stop",
        )
        extended_portal = ctx.part_world_position(portal)
        extended_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "portal slide travels along the base rails",
        rest_portal is not None
        and extended_portal is not None
        and extended_portal[0] > rest_portal[0] + 1.8,
        details=f"rest={rest_portal}, extended={extended_portal}",
    )
    ctx.check(
        "nested carriage travels across the portal beam",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[1] > rest_carriage[1] + 0.8,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    return ctx.report()


object_model = build_object_model()
