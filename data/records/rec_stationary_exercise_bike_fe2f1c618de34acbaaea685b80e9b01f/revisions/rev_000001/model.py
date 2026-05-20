from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _circle_profile(radius: float, segments: int = 72, *, reverse: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if reverse else points


def _offset_circle_profile(
    radius: float,
    center: tuple[float, float],
    segments: int = 72,
    *,
    reverse: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points = [(x + cx, y + cy) for x, y in _circle_profile(radius, segments)]
    return list(reversed(points)) if reverse else points


def _add_tube(part, name: str, points, radius: float, material, *, corner_radius: float = 0.0) -> None:
    part.visual(
        mesh_from_geometry(
            wire_from_points(
                points,
                radius=radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=corner_radius,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stationary_bike")

    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    charcoal = model.material("charcoal", rgba=(0.09, 0.095, 0.10, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.015, 0.015, 0.016, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    red_accent = model.material("red_accent", rgba=(0.80, 0.05, 0.035, 1.0))

    frame = model.part("frame")

    # Compact floor stance: short wheelbase with wide stabilizer feet.
    _add_tube(frame, "base_rail", [(-0.34, 0.0, 0.065), (0.67, 0.0, 0.065)], 0.026, satin_black)
    _add_tube(frame, "rear_stabilizer", [(-0.32, -0.30, 0.045), (-0.32, 0.30, 0.045)], 0.026, satin_black)
    _add_tube(frame, "front_stabilizer", [(0.63, -0.30, 0.045), (0.63, 0.30, 0.045)], 0.026, satin_black)
    for x, y, name in (
        (-0.32, -0.32, "rear_foot_0"),
        (-0.32, 0.32, "rear_foot_1"),
        (0.63, -0.32, "front_foot_0"),
        (0.63, 0.32, "front_foot_1"),
    ):
        frame.visual(
            Box((0.13, 0.075, 0.035)),
            origin=Origin(xyz=(x, y, 0.023)),
            material=rubber,
            name=name,
        )

    # Seat column, bottom bracket, and handlebar mast are tied into a welded tube frame.
    frame.visual(
        Cylinder(radius=0.037, length=0.34),
        origin=Origin(xyz=(-0.12, 0.0, 0.49)),
        material=satin_black,
        name="seat_sleeve",
    )
    _add_tube(frame, "seat_stay", [(-0.23, 0.0, 0.08), (-0.12, 0.0, 0.33)], 0.020, satin_black)
    _add_tube(frame, "down_tube", [(-0.12, 0.0, 0.34), (0.04, 0.0, 0.43)], 0.022, satin_black)
    _add_tube(frame, "top_tube", [(-0.12, 0.041, 0.62), (0.10, 0.075, 0.76), (0.47, 0.0, 0.99)], 0.021, satin_black, corner_radius=0.045)
    _add_tube(frame, "front_stay_0", [(0.63, -0.02, 0.07), (0.54, -0.09, 0.28), (0.39, -0.09, 0.61)], 0.020, satin_black, corner_radius=0.035)
    _add_tube(frame, "front_stay_1", [(0.63, 0.02, 0.07), (0.54, 0.09, 0.28), (0.39, 0.09, 0.61)], 0.020, satin_black, corner_radius=0.035)
    _add_tube(frame, "fork_stay", [(0.11, 0.065, 0.37), (0.31, 0.09, 0.34)], 0.020, satin_black)

    frame.visual(
        Cylinder(radius=0.037, length=0.20),
        origin=Origin(xyz=(0.08, 0.0, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="bottom_bracket",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.035),
        origin=Origin(xyz=(-0.12, -0.052, 0.61), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="seat_lock_knob",
    )

    # Protective belt cover on the drive side, clear of the rotating crank.
    frame.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.34, 0.16, 0.065), 0.014, center=True),
            "belt_cover",
        ),
        origin=Origin(xyz=(0.340, -0.130, 0.385), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="belt_cover",
    )

    # Open flywheel housing: paired annular guards leave the rotor visible while
    # keeping clearance around it.
    flywheel_center = (0.39, 0.0, 0.37)
    housing_ring = ExtrudeWithHolesGeometry(
        _circle_profile(0.275, 96),
        [_circle_profile(0.225, 96)],
        0.012,
        center=True,
    )
    for y, name in ((0.045, "housing_ring_0"), (-0.045, "housing_ring_1")):
        frame.visual(
            mesh_from_geometry(housing_ring, name),
            origin=Origin(xyz=(flywheel_center[0], y, flywheel_center[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=flywheel_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="flywheel_bearing",
    )
    _add_tube(frame, "housing_bridge_top", [(0.39, -0.055, 0.645), (0.39, 0.055, 0.645)], 0.012, charcoal)
    _add_tube(frame, "housing_bridge_low", [(0.39, -0.055, 0.095), (0.39, 0.055, 0.095)], 0.012, charcoal)

    # Handlebar loop and rubber grips.
    _add_tube(
        frame,
        "handlebar",
        [(0.50, -0.28, 1.05), (0.44, -0.19, 1.08), (0.44, 0.0, 1.08), (0.44, 0.19, 1.08), (0.50, 0.28, 1.05)],
        0.017,
        brushed_steel,
        corner_radius=0.055,
    )
    _add_tube(frame, "handlebar_stem", [(0.47, 0.0, 0.99), (0.44, 0.0, 1.08)], 0.017, brushed_steel)
    _add_tube(frame, "grip_0", [(0.505, -0.33, 1.045), (0.50, -0.25, 1.05)], 0.021, rubber)
    _add_tube(frame, "grip_1", [(0.50, 0.25, 1.05), (0.505, 0.33, 1.045)], 0.021, rubber)

    # Sliding seat post with a tapered bicycle saddle and metal rails.
    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.018, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=brushed_steel,
        name="inner_post",
    )
    seat_post.visual(
        Cylinder(radius=0.026, length=0.11),
        origin=Origin(xyz=(-0.02, 0.0, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="seat_clamp",
    )
    for y, name in ((-0.036, "seat_rail_0"), (0.036, "seat_rail_1")):
        seat_post.visual(
            Box((0.19, 0.009, 0.009)),
            origin=Origin(xyz=(-0.035, y, 0.232)),
            material=brushed_steel,
            name=name,
        )
    saddle_profile = [
        (0.155, 0.0),
        (0.115, 0.032),
        (0.018, 0.060),
        (-0.125, 0.090),
        (-0.175, 0.070),
        (-0.182, 0.0),
        (-0.175, -0.070),
        (-0.125, -0.090),
        (0.018, -0.060),
        (0.115, -0.032),
    ]
    seat_post.visual(
        mesh_from_geometry(ExtrudeGeometry(saddle_profile, 0.055, center=True), "saddle"),
        origin=Origin(xyz=(-0.020, 0.0, 0.262)),
        material=dark_plastic,
        name="saddle",
    )

    # Rotating crankset around the bottom-bracket axis.
    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.016, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="crank_axle",
    )
    crank.visual(
        Cylinder(radius=0.125, length=0.012),
        origin=Origin(xyz=(0.0, -0.114, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_accent,
        name="chainring",
    )
    crank.visual(
        Box((0.026, 0.090, 0.235)),
        origin=Origin(xyz=(0.0, 0.145, -0.120)),
        material=brushed_steel,
        name="crank_arm_0",
    )
    crank.visual(
        Box((0.026, 0.090, 0.235)),
        origin=Origin(xyz=(0.0, -0.145, 0.120)),
        material=brushed_steel,
        name="crank_arm_1",
    )
    crank.visual(
        Box((0.085, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, 0.195, -0.245)),
        material=dark_plastic,
        name="pedal_0",
    )
    crank.visual(
        Box((0.085, 0.045, 0.026)),
        origin=Origin(xyz=(0.0, -0.195, 0.245)),
        material=dark_plastic,
        name="pedal_1",
    )

    # Rotating flywheel inside the open guard.
    flywheel = model.part("flywheel")
    flywheel.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.205, 96),
                [_circle_profile(0.043, 72)],
                0.038,
                center=True,
            ),
            "flywheel_rotor",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="flywheel_rotor",
    )
    flywheel.visual(
        Cylinder(radius=0.035, length=0.055),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="flywheel_hub",
    )
    flywheel.visual(
        Box((0.140, 0.007, 0.020)),
        origin=Origin(xyz=(0.055, 0.019, 0.118)),
        material=red_accent,
        name="flywheel_mark",
    )

    model.articulation(
        "seat_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.12, 0.0, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.14),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.08, 0.0, 0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=9.0),
    )
    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=flywheel_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=14.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    crank = object_model.get_part("crank")
    flywheel = object_model.get_part("flywheel")
    seat_slide = object_model.get_articulation("seat_slide")
    crank_spin = object_model.get_articulation("crank_spin")
    flywheel_spin = object_model.get_articulation("flywheel_spin")

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason="The inner seat post is intentionally represented as a sliding member inside the sleeve.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bottom_bracket",
        elem_b="crank_axle",
        reason="The crank axle is captured inside the bottom-bracket bearing proxy.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_bearing",
        elem_b="flywheel_hub",
        reason="The flywheel hub is intentionally seated around the fixed bearing/axle proxy.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_bearing",
        elem_b="flywheel_rotor",
        reason="The fixed flywheel bearing passes through the rotor's central hub region in this compact proxy.",
    )

    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.002,
        name="seat post remains centered in sleeve",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.16,
        name="seat post retained at low setting",
    )
    rest_seat = ctx.part_world_position(seat_post)
    with ctx.pose({seat_slide: 0.14}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.002,
            name="raised seat post remains centered",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.06,
            name="raised seat post keeps insertion",
        )
        raised_seat = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post translates upward",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.12,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="crank_axle",
        outer_elem="bottom_bracket",
        margin=0.002,
        name="crank axle centered in bottom bracket",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="crank_axle",
        elem_b="bottom_bracket",
        min_overlap=0.18,
        name="crank axle spans bottom bracket",
    )
    ctx.expect_within(
        flywheel,
        frame,
        axes="xz",
        inner_elem="flywheel_hub",
        outer_elem="flywheel_bearing",
        margin=0.020,
        name="flywheel hub centered on bearing",
    )
    ctx.expect_overlap(
        frame,
        flywheel,
        axes="y",
        elem_a="flywheel_bearing",
        elem_b="flywheel_rotor",
        min_overlap=0.030,
        name="bearing passes through flywheel plane",
    )
    ctx.expect_gap(
        frame,
        flywheel,
        axis="y",
        positive_elem="housing_ring_0",
        negative_elem="flywheel_rotor",
        min_gap=0.010,
        name="flywheel clears side guard",
    )

    def _aabb_center(bounds):
        return tuple((bounds[0][i] + bounds[1][i]) * 0.5 for i in range(3))

    pedal_rest = ctx.part_element_world_aabb(crank, elem="pedal_0")
    with ctx.pose({crank_spin: math.pi / 2.0}):
        pedal_turned = ctx.part_element_world_aabb(crank, elem="pedal_0")
    if pedal_rest is not None and pedal_turned is not None:
        p0 = _aabb_center(pedal_rest)
        p1 = _aabb_center(pedal_turned)
        crank_moves = abs(p1[0] - p0[0]) > 0.12 and abs(p1[2] - p0[2]) > 0.12
    else:
        p0 = p1 = None
        crank_moves = False
    ctx.check("crank visibly rotates about bottom bracket", crank_moves, details=f"rest={p0}, turned={p1}")

    mark_rest = ctx.part_element_world_aabb(flywheel, elem="flywheel_mark")
    with ctx.pose({flywheel_spin: math.pi / 2.0}):
        mark_turned = ctx.part_element_world_aabb(flywheel, elem="flywheel_mark")
    if mark_rest is not None and mark_turned is not None:
        m0 = _aabb_center(mark_rest)
        m1 = _aabb_center(mark_turned)
        flywheel_moves = abs(m1[0] - m0[0]) > 0.06 and abs(m1[2] - m0[2]) > 0.06
    else:
        m0 = m1 = None
        flywheel_moves = False
    ctx.check("flywheel marker rotates about axle", flywheel_moves, details=f"rest={m0}, turned={m1}")

    return ctx.report()


object_model = build_object_model()
