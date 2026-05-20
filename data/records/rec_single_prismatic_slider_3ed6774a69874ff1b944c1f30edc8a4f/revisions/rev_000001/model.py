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
    model = ArticulatedObject(name="low_profile_shuttle_axis")

    anodized_black = Material("anodized_black", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_plate = Material("dark_plate", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.74, 0.76, 0.72, 1.0))
    blue_carriage = Material("blue_carriage", rgba=(0.05, 0.22, 0.72, 1.0))
    bearing_grey = Material("bearing_grey", rgba=(0.30, 0.32, 0.34, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.01, 0.01, 0.01, 1.0))
    fastener = Material("socket_head", rgba=(0.02, 0.02, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.78, 0.30, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_plate,
        name="base_plate",
    )

    # Low rail pads and two shallow hardened rails define the travel centerline.
    for y in (-0.070, 0.070):
        suffix = "neg" if y < 0.0 else "pos"
        base.visual(
            Box((0.660, 0.052, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.023)),
            material=anodized_black,
            name=f"rail_pad_{suffix}",
        )
        base.visual(
            Box((0.640, 0.030, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.034)),
            material=brushed_steel,
            name="linear_rail_neg" if y < 0.0 else "linear_rail_pos",
        )
        for x in (-0.255, -0.085, 0.085, 0.255):
            base.visual(
                Cylinder(radius=0.0075, length=0.0035),
                origin=Origin(xyz=(x, y, 0.0442)),
                material=fastener,
                name=f"rail_screw_{suffix}_{x:+.3f}",
            )

    # Raised stop blocks, rubber bumpers, and cap screws at both travel ends.
    for sign, end_name in ((-1.0, "neg"), (1.0, "pos")):
        x_center = sign * 0.295
        base.visual(
            Box((0.042, 0.190, 0.060)),
            origin=Origin(xyz=(x_center, 0.0, 0.050)),
            material=anodized_black,
            name=f"end_stop_{end_name}",
        )
        bumper_center_x = sign * 0.262
        base.visual(
            Cylinder(radius=0.017, length=0.024),
            origin=Origin(
                xyz=(bumper_center_x, 0.0, 0.060),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber_black,
            name="rubber_bumper_neg" if sign < 0.0 else "rubber_bumper_pos",
        )
        for y in (-0.065, 0.065):
            base.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(x_center, y, 0.082)),
                material=fastener,
                name=f"stop_screw_{end_name}_{'neg' if y < 0.0 else 'pos'}",
            )

    # Four base mounting holes are represented by dark recessed socket heads.
    for x in (-0.330, 0.330):
        for y in (-0.120, 0.120):
            base.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(x, y, 0.0215)),
                material=fastener,
                name=f"base_screw_{'neg' if x < 0 else 'pos'}_{'neg' if y < 0 else 'pos'}",
            )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((0.210, 0.180, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_carriage,
        name="shuttle_body",
    )
    shuttle.visual(
        Box((0.145, 0.105, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=anodized_black,
        name="payload_mount",
    )

    # Long bearing housings are carried by the moving shuttle and straddle the rails.
    for y in (-0.070, 0.070):
        suffix = "neg" if y < 0.0 else "pos"
        shuttle.visual(
            Box((0.200, 0.070, 0.014)),
            origin=Origin(xyz=(0.0, y, -0.0295)),
            material=bearing_grey,
            name="bearing_bridge_neg" if y < 0.0 else "bearing_bridge_pos",
        )
        for side in (-1.0, 1.0):
            side_name = "inner" if side * y < 0.0 else "outer"
            shuttle.visual(
                Box((0.200, 0.010, 0.028)),
                origin=Origin(xyz=(0.0, y + side * 0.020, -0.0505)),
                material=bearing_grey,
                name=f"bearing_cheek_{suffix}_{side_name}",
            )
        for x in (-0.072, 0.072):
            shuttle.visual(
                Cylinder(radius=0.0065, length=0.0035),
                origin=Origin(xyz=(x, y, 0.0234)),
                material=fastener,
                name=f"carriage_screw_{suffix}_{'neg' if x < 0.0 else 'pos'}",
            )

    model.articulation(
        "base_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=shuttle,
        # The child frame sits on the center of the boxy shuttle, directly above
        # the rail centerline.  Positive travel moves toward +X.
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=-0.120, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shuttle = object_model.get_part("shuttle")
    slide = object_model.get_articulation("base_to_shuttle")

    ctx.check(
        "single prismatic shuttle joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"joint_count={len(object_model.articulations)}, axis={slide.axis}",
    )

    ctx.expect_within(
        shuttle,
        base,
        axes="y",
        inner_elem="shuttle_body",
        outer_elem="base_plate",
        margin=0.0,
        name="shuttle stays inside base width",
    )
    ctx.expect_gap(
        shuttle,
        base,
        axis="z",
        positive_elem="bearing_bridge_pos",
        negative_elem="linear_rail_pos",
        min_gap=0.012,
        max_gap=0.030,
        name="bearing bridge clears positive rail",
    )
    ctx.expect_gap(
        shuttle,
        base,
        axis="z",
        positive_elem="bearing_bridge_neg",
        negative_elem="linear_rail_neg",
        min_gap=0.012,
        max_gap=0.030,
        name="bearing bridge clears negative rail",
    )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: 0.120}):
        ctx.expect_gap(
            base,
            shuttle,
            axis="x",
            positive_elem="rubber_bumper_pos",
            negative_elem="shuttle_body",
            min_gap=0.020,
            max_gap=0.050,
            name="positive end stop is near the shuttle",
        )
        upper_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: -0.120}):
        ctx.expect_gap(
            shuttle,
            base,
            axis="x",
            positive_elem="shuttle_body",
            negative_elem="rubber_bumper_neg",
            min_gap=0.020,
            max_gap=0.050,
            name="negative end stop is near the shuttle",
        )
        lower_pos = ctx.part_world_position(shuttle)

    ctx.check(
        "prismatic joint translates along rail centerline",
        rest_pos is not None
        and upper_pos is not None
        and lower_pos is not None
        and upper_pos[0] > rest_pos[0] + 0.10
        and lower_pos[0] < rest_pos[0] - 0.10
        and abs(upper_pos[1] - rest_pos[1]) < 1e-6
        and abs(lower_pos[1] - rest_pos[1]) < 1e-6,
        details=f"lower={lower_pos}, rest={rest_pos}, upper={upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
