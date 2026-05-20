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
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    """CadQuery annulus extruded upward from z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_twin_rail_missile_launcher")

    deck_gray = model.material("non_skid_deck_gray", rgba=(0.24, 0.27, 0.28, 1.0))
    haze_gray = model.material("naval_haze_gray", rgba=(0.48, 0.54, 0.56, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    worn_metal = model.material("worn_rail_metal", rgba=(0.64, 0.66, 0.64, 1.0))
    warning_red = model.material("safety_red_caps", rgba=(0.65, 0.05, 0.03, 1.0))

    deck = model.part("deck_base")
    deck.visual(
        Box((2.70, 1.90, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=deck_gray,
        name="deck_plate",
    )
    deck.visual(
        mesh_from_cadquery(_annular_cylinder(0.78, 0.42, 0.14), "fixed_foundation_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=haze_gray,
        name="foundation_ring",
    )
    deck.visual(
        Cylinder(radius=0.42, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_steel,
        name="bearing_shadow",
    )
    # Deck fasteners on the fixed foundation ring.
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        r = 0.73
        deck.visual(
            Cylinder(radius=0.035, length=0.030),
            origin=Origin(xyz=(r * math.cos(a), r * math.sin(a), 0.255)),
            material=dark_steel,
            name=f"deck_bolt_{i}",
        )

    pedestal = model.part("yaw_pedestal")
    pedestal.visual(
        mesh_from_cadquery(_annular_cylinder(0.67, 0.34, 0.14), "rotating_yaw_ring"),
        origin=Origin(),
        material=haze_gray,
        name="yaw_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.345, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=haze_gray,
        name="yaw_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.31, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=haze_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.42, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=haze_gray,
        name="upper_turntable",
    )
    pedestal.visual(
        Box((0.86, 0.68, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=haze_gray,
        name="saddle_block",
    )
    yoke = TrunnionYokeGeometry(
        (1.16, 0.72, 0.76),
        span_width=0.78,
        trunnion_diameter=0.28,
        trunnion_center_z=0.44,
        base_thickness=0.16,
        corner_radius=0.035,
        center=False,
    )
    pedestal.visual(
        mesh_from_geometry(yoke, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.70), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=haze_gray,
        name="trunnion_yoke",
    )
    pedestal.visual(
        Box((0.18, 0.92, 0.12)),
        origin=Origin(xyz=(-0.34, 0.0, 0.95)),
        material=haze_gray,
        name="rear_yoke_rib",
    )
    pedestal.visual(
        Box((0.18, 0.92, 0.12)),
        origin=Origin(xyz=(0.34, 0.0, 0.95)),
        material=haze_gray,
        name="front_yoke_rib",
    )

    frame = model.part("elevation_frame")
    # The frame part origin is exactly on the pitch/trunnion axis.
    frame.visual(
        Cylinder(radius=0.080, length=1.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.62, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_cap_0",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, -0.62, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_cap_1",
    )
    frame.visual(
        Box((2.48, 0.12, 0.12)),
        origin=Origin(xyz=(1.24, 0.31, -0.06)),
        material=haze_gray,
        name="frame_arm_0",
    )
    frame.visual(
        Box((2.48, 0.12, 0.12)),
        origin=Origin(xyz=(1.24, -0.31, -0.06)),
        material=haze_gray,
        name="frame_arm_1",
    )
    frame.visual(
        Box((0.20, 0.78, 0.12)),
        origin=Origin(xyz=(0.48, 0.0, -0.06)),
        material=haze_gray,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.18, 0.78, 0.12)),
        origin=Origin(xyz=(2.38, 0.0, -0.06)),
        material=haze_gray,
        name="front_crossmember",
    )
    frame.visual(
        Box((1.95, 0.08, 0.08)),
        origin=Origin(xyz=(1.42, 0.0, -0.12)),
        material=haze_gray,
        name="lower_keel",
    )
    for idx, y in enumerate((0.31, -0.31)):
        frame.visual(
            Box((2.62, 0.105, 0.075)),
            origin=Origin(xyz=(1.34, y, 0.055)),
            material=worn_metal,
            name=f"launch_rail_{idx}",
        )
        frame.visual(
            Box((2.50, 0.045, 0.090)),
            origin=Origin(xyz=(1.39, y, 0.135)),
            material=worn_metal,
            name=f"rail_guide_{idx}",
        )
        frame.visual(
            Box((0.12, 0.16, 0.16)),
            origin=Origin(xyz=(2.68, y, 0.075)),
            material=warning_red,
            name=f"muzzle_stop_{idx}",
        )
        frame.visual(
            Box((0.16, 0.14, 0.12)),
            origin=Origin(xyz=(0.28, y, 0.06)),
            material=dark_steel,
            name=f"rear_latch_{idx}",
        )

    model.articulation(
        "deck_to_pedestal",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22000.0, velocity=0.45, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pedestal_to_frame",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.35, lower=-0.08, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck_base")
    pedestal = object_model.get_part("yaw_pedestal")
    frame = object_model.get_part("elevation_frame")
    yaw = object_model.get_articulation("deck_to_pedestal")
    pitch = object_model.get_articulation("pedestal_to_frame")

    ctx.check(
        "pedestal uses vertical yaw axis",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "frame uses horizontal pitch axis",
        abs(pitch.axis[1]) > 0.99 and abs(pitch.axis[0]) < 1e-6 and abs(pitch.axis[2]) < 1e-6,
        details=f"axis={pitch.axis}",
    )
    ctx.expect_gap(
        pedestal,
        deck,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="yaw_ring",
        negative_elem="foundation_ring",
        name="rotating ring sits on fixed foundation ring",
    )
    ctx.expect_overlap(
        pedestal,
        deck,
        axes="xy",
        min_overlap=0.50,
        elem_a="yaw_ring",
        elem_b="foundation_ring",
        name="yaw bearing rings share deck footprint",
    )
    ctx.expect_within(
        frame,
        pedestal,
        axes="y",
        margin=0.04,
        inner_elem="trunnion_shaft",
        outer_elem="trunnion_yoke",
        name="trunnion shaft is captured between yoke cheeks",
    )

    rest_aabb = ctx.part_element_world_aabb(frame, elem="launch_rail_0")
    with ctx.pose({pitch: 0.85}):
        raised_aabb = ctx.part_element_world_aabb(frame, elem="launch_rail_0")
    ctx.check(
        "pitch joint raises launch rails",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.55,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    with ctx.pose({yaw: 0.65}):
        yawed_aabb = ctx.part_element_world_aabb(frame, elem="launch_rail_0")
    ctx.check(
        "yaw joint slews rail frame around deck pedestal",
        rest_aabb is not None
        and yawed_aabb is not None
        and abs(((yawed_aabb[0][1] + yawed_aabb[1][1]) * 0.5) - ((rest_aabb[0][1] + rest_aabb[1][1]) * 0.5)) > 0.45,
        details=f"rest={rest_aabb}, yawed={yawed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
