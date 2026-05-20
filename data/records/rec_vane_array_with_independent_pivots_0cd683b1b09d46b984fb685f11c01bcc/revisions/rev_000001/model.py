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
    model = ArticulatedObject(name="multi_flap_sheet_metal_frame")

    galvanized = Material("galvanized_frame", rgba=(0.50, 0.54, 0.56, 1.0))
    shadow = Material("dark_hinge_shadow", rgba=(0.05, 0.055, 0.06, 1.0))
    painted_sheet = Material("painted_flap_sheet", rgba=(0.64, 0.72, 0.76, 1.0))
    raw_edge = Material("folded_raw_edge", rgba=(0.42, 0.46, 0.48, 1.0))

    outer_w = 1.20
    outer_h = 0.72
    frame_depth = 0.080
    rail = 0.060
    mullion_w = 0.018
    flap_count = 4

    opening_w = outer_w - 2.0 * rail
    opening_h = outer_h - 2.0 * rail
    bay_w = (opening_w - (flap_count - 1) * mullion_w) / flap_count
    flap_w = bay_w - 0.026
    flap_h = opening_h - 0.058
    hinge_y = frame_depth / 2.0 + 0.015
    hinge_z = outer_h / 2.0 - rail + 0.005

    frame = model.part("frame")
    # The frame is made from overlapping sheet-metal channel members.  Each
    # visual touches or intersects a neighboring member so the root part reads
    # as one welded rectangular carrier rather than separate floating bars.
    frame.visual(
        Box((outer_w, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - rail / 2.0)),
        material=galvanized,
        name="top_rail",
    )
    frame.visual(
        Box((outer_w, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + rail / 2.0)),
        material=galvanized,
        name="bottom_rail",
    )
    for side, x in (("side_0", -outer_w / 2.0 + rail / 2.0), ("side_1", outer_w / 2.0 - rail / 2.0)):
        frame.visual(
            Box((rail, frame_depth, outer_h)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=galvanized,
            name=side,
        )

    # Vertical dividers define separate bays for the independently hinged flaps.
    x0 = -opening_w / 2.0 + bay_w
    for i in range(flap_count - 1):
        x = x0 + i * (bay_w + mullion_w) + mullion_w / 2.0
        frame.visual(
            Box((mullion_w, frame_depth, opening_h + 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=galvanized,
            name=f"mullion_{i}",
        )

    # Folded front lips make the rectangular carrier read as pressed sheet metal.
    frame.visual(
        Box((outer_w, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, frame_depth / 2.0 + 0.008, outer_h / 2.0 - 0.009)),
        material=raw_edge,
        name="top_front_fold",
    )
    frame.visual(
        Box((outer_w, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, frame_depth / 2.0 + 0.008, -outer_h / 2.0 + 0.009)),
        material=raw_edge,
        name="bottom_front_fold",
    )
    frame.visual(
        Box((0.018, 0.016, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + 0.009, frame_depth / 2.0 + 0.008, 0.0)),
        material=raw_edge,
        name="side_front_fold_0",
    )
    frame.visual(
        Box((0.018, 0.016, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - 0.009, frame_depth / 2.0 + 0.008, 0.0)),
        material=raw_edge,
        name="side_front_fold_1",
    )

    # Stationary hinge eyes and leaves at each bay.  The moving flap barrels sit
    # between these short knuckles, leaving real axial gaps instead of a broad
    # visual overlap.
    hinge_radius = 0.010
    knuckle_len = 0.034
    for i in range(flap_count):
        bay_center = -opening_w / 2.0 + bay_w / 2.0 + i * (bay_w + mullion_w)
        for suffix, x in (
            ("a", bay_center - flap_w / 2.0 - knuckle_len / 2.0 - 0.006),
            ("b", bay_center + flap_w / 2.0 + knuckle_len / 2.0 + 0.006),
        ):
            frame.visual(
                Box((knuckle_len + 0.010, 0.012, 0.050)),
                origin=Origin(xyz=(x, frame_depth / 2.0 + 0.006, hinge_z - 0.020)),
                material=galvanized,
                name=f"hinge_leaf_{i}_{suffix}",
            )
            frame.visual(
                Cylinder(radius=hinge_radius, length=knuckle_len),
                origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
                material=shadow,
                name=f"fixed_knuckle_{i}_{suffix}",
            )

    for i in range(flap_count):
        bay_center = -opening_w / 2.0 + bay_w / 2.0 + i * (bay_w + mullion_w)
        flap = model.part(f"flap_{i}")
        flap.visual(
            Box((flap_w, 0.006, flap_h)),
            origin=Origin(xyz=(0.0, 0.0, -0.018 - flap_h / 2.0)),
            material=painted_sheet,
            name="sheet",
        )
        flap.visual(
            Box((flap_w, 0.004, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=raw_edge,
            name="hinge_leaf",
        )
        flap.visual(
            Cylinder(radius=hinge_radius, length=flap_w - 0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=shadow,
            name="moving_knuckle",
        )
        # Shallow folded hems along the sides and bottom stiffen the thin sheet.
        flap.visual(
            Box((0.010, 0.018, flap_h)),
            origin=Origin(xyz=(-flap_w / 2.0 + 0.005, -0.006, -0.018 - flap_h / 2.0)),
            material=raw_edge,
            name="side_hem_0",
        )
        flap.visual(
            Box((0.010, 0.018, flap_h)),
            origin=Origin(xyz=(flap_w / 2.0 - 0.005, -0.006, -0.018 - flap_h / 2.0)),
            material=raw_edge,
            name="side_hem_1",
        )
        flap.visual(
            Box((flap_w, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, -0.006, -0.018 - flap_h + 0.006)),
            material=raw_edge,
            name="bottom_hem",
        )
        flap.visual(
            Box((flap_w * 0.62, 0.008, 0.020)),
            origin=Origin(xyz=(0.0, 0.006, -0.018 - flap_h * 0.56)),
            material=raw_edge,
            name="stiffener",
        )

        model.articulation(
            f"frame_to_flap_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=flap,
            origin=Origin(xyz=(bay_center, hinge_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=7.0, velocity=2.0, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    ctx.check(
        "four independent flap parts",
        all(object_model.get_part(f"flap_{i}") is not None for i in range(4)),
        details="Expected flap_0 through flap_3.",
    )
    ctx.check(
        "four repeated revolute joints",
        all(
            object_model.get_articulation(f"frame_to_flap_{i}") is not None
            and object_model.get_articulation(f"frame_to_flap_{i}").articulation_type == ArticulationType.REVOLUTE
            for i in range(4)
        ),
        details="Each flap should have its own revolute hinge.",
    )

    for i in range(4):
        flap = object_model.get_part(f"flap_{i}")
        hinge = object_model.get_articulation(f"frame_to_flap_{i}")
        ctx.expect_within(
            flap,
            frame,
            axes="xz",
            inner_elem="sheet",
            margin=0.002,
            name=f"flap_{i} sheet stays inside rectangular frame",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="y",
            min_gap=0.0005,
            max_gap=0.030,
            positive_elem="sheet",
            negative_elem="top_rail",
            name=f"flap_{i} sheet sits just proud of frame face",
        )
        with ctx.pose({hinge: 1.0}):
            rest_pos = ctx.part_world_position(flap)
            aabb = ctx.part_element_world_aabb(flap, elem="sheet")
            ctx.check(
                f"flap_{i} opens forward on its hinge",
                rest_pos is not None and aabb is not None and aabb[1][1] > 0.25,
                details=f"rest_pos={rest_pos}, sheet_aabb={aabb}",
            )

    return ctx.report()


object_model = build_object_model()
