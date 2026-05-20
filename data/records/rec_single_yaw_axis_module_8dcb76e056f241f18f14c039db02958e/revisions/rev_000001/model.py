from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_disc(outer_radius: float, inner_radius: float, height: float):
    """CadQuery annulus centered on the local XY origin."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _rounded_plate(size: tuple[float, float, float], radius: float):
    plate = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        plate = plate.edges("|Z").fillet(radius)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_scanning_platform")

    frame_paint = model.material("charcoal_powder_coat", rgba=(0.06, 0.07, 0.08, 1.0))
    deck_paint = model.material("safety_orange", rgba=(0.95, 0.43, 0.08, 1.0))
    rail_blue = model.material("blue_linear_rail", rgba=(0.06, 0.16, 0.32, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    bearing_metal = model.material("polished_bearing_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    brass = model.material("oiled_bronze", rgba=(0.85, 0.58, 0.22, 1.0))

    frame = model.part("mounting_frame")

    # Welded industrial base: four pads, rectangular skid rails, upright posts,
    # and a thick machinery plate carrying the pan bearing.
    for idx, y in enumerate((-0.30, 0.30)):
        frame.visual(
            Box((0.92, 0.140, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=frame_paint,
            name=f"side_rail_{idx}",
        )
    for idx, x in enumerate((-0.43, 0.43)):
        frame.visual(
            Box((0.065, 0.62, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=frame_paint,
            name=f"end_rail_{idx}",
        )

    pad_positions = [(-0.39, -0.26), (-0.39, 0.26), (0.39, -0.26), (0.39, 0.26)]
    for idx, (x, y) in enumerate(pad_positions):
        frame.visual(
            Box((0.125, 0.125, 0.026)),
            origin=Origin(xyz=(x, y, 0.013)),
            material=frame_paint,
            name=f"floor_pad_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.032)),
            material=steel,
            name=f"anchor_bolt_{idx}",
        )

    for idx, (x, y) in enumerate([(-0.31, -0.20), (-0.31, 0.20), (0.31, -0.20), (0.31, 0.20)]):
        frame.visual(
            Box((0.060, 0.060, 0.118)),
            origin=Origin(xyz=(x, y, 0.111)),
            material=frame_paint,
            name=f"upright_post_{idx}",
        )

    frame.visual(
        mesh_from_cadquery(_rounded_plate((0.64, 0.46, 0.036), 0.025), "machinery_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=frame_paint,
        name="machinery_plate",
    )
    frame.visual(
        Cylinder(radius=0.145, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=frame_paint,
        name="bearing_pedestal",
    )

    # Exposed stationary race stack with visible rolling elements.
    frame.visual(
        mesh_from_cadquery(_annular_disc(0.245, 0.118, 0.020), "outer_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.246)),
        material=bearing_metal,
        name="outer_race",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.182, 0.011, radial_segments=24, tubular_segments=72), "lower_race_groove"),
        origin=Origin(xyz=(0.0, 0.0, 0.259)),
        material=bearing_metal,
        name="lower_race_groove",
    )
    frame.visual(
        mesh_from_cadquery(_annular_disc(0.196, 0.168, 0.010), "bearing_cage"),
        origin=Origin(xyz=(0.0, 0.0, 0.266)),
        material=steel,
        name="bearing_cage",
    )
    frame.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.182, 0.0, 0.274)),
        material=brass,
        name="bearing_ball_0",
    )
    for idx in range(1, 16):
        angle = 2.0 * math.pi * idx / 16.0
        frame.visual(
            Sphere(radius=0.011),
            origin=Origin(xyz=(0.182 * math.cos(angle), 0.182 * math.sin(angle), 0.274)),
            material=brass,
            name=f"bearing_ball_{idx}",
        )

    # A fixed index mark on the frame makes the pan motion legible.
    frame.visual(
        Box((0.060, 0.012, 0.018)),
        origin=Origin(xyz=(0.230, 0.0, 0.265)),
        material=dark_rubber,
        name="fixed_index",
    )

    platform = model.part("pan_platform")

    platform.visual(
        mesh_from_cadquery(_annular_disc(0.220, 0.120, 0.016), "upper_race"),
        origin=Origin(),
        material=bearing_metal,
        name="upper_race",
    )
    platform.visual(
        Box((0.405, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=bearing_metal,
        name="race_spoke_x",
    )
    platform.visual(
        Box((0.026, 0.405, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=bearing_metal,
        name="race_spoke_y",
    )
    platform.visual(
        Cylinder(radius=0.100, length=0.114),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=steel,
        name="rotary_hub",
    )
    platform.visual(
        mesh_from_cadquery(_rounded_plate((0.720, 0.480, 0.034), 0.030), "deck_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=deck_paint,
        name="deck_plate",
    )

    # Raised scanner fixture rails and black recessed T-slot strips on the deck.
    for idx, y in enumerate((-0.155, 0.155)):
        platform.visual(
            Box((0.620, 0.044, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.1595)),
            material=rail_blue,
            name=f"fixture_rail_{idx}",
        )
        platform.visual(
            Box((0.560, 0.018, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.174)),
            material=dark_rubber,
            name=f"rail_slot_{idx}",
        )

    for idx, x in enumerate((-0.315, 0.315)):
        platform.visual(
            Box((0.030, 0.370, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.159)),
            material=deck_paint,
            name=f"end_stop_{idx}",
        )

    platform.visual(
        Box((0.080, 0.018, 0.020)),
        origin=Origin(xyz=(0.345, 0.0, 0.160)),
        material=dark_rubber,
        name="scan_pointer",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.293)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("mounting_frame")
    platform = object_model.get_part("pan_platform")
    pan = object_model.get_articulation("pan_axis")

    ctx.check(
        "single vertical pan joint",
        len(object_model.articulations) == 1
        and pan.articulation_type == ArticulationType.REVOLUTE
        and tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={len(object_model.articulations)}, axis={pan.axis}",
    )

    ctx.expect_contact(
        platform,
        frame,
        contact_tol=0.0005,
        elem_a="upper_race",
        elem_b="bearing_ball_0",
        name="upper race contacts bearing balls",
    )
    ctx.expect_overlap(
        platform,
        frame,
        axes="xy",
        min_overlap=0.30,
        elem_a="deck_plate",
        elem_b="machinery_plate",
        name="turntable remains centered on frame",
    )

    rest_pointer = ctx.part_element_world_aabb(platform, elem="scan_pointer")
    with ctx.pose({pan: math.pi / 2.0}):
        turned_pointer = ctx.part_element_world_aabb(platform, elem="scan_pointer")

    def _center_xy(aabb):
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_xy = _center_xy(rest_pointer) if rest_pointer is not None else None
    turned_xy = _center_xy(turned_pointer) if turned_pointer is not None else None
    ctx.check(
        "pan rotation carries pointer around vertical axis",
        rest_xy is not None
        and turned_xy is not None
        and rest_xy[0] > 0.32
        and abs(rest_xy[1]) < 0.03
        and turned_xy[1] > 0.32
        and abs(turned_xy[0]) < 0.03,
        details=f"rest={rest_xy}, turned={turned_xy}",
    )

    return ctx.report()


object_model = build_object_model()
