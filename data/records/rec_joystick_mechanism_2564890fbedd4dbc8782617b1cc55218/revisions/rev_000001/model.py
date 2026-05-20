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
    mesh_from_cadquery,
)
import cadquery as cq


LIMIT = math.radians(18.0)


def annular_plate(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A simple flat annulus, centered on local XY and extruded upward from z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_console_joystick")

    dark_metal = Material("mat_dark_metal", color=(0.08, 0.085, 0.09, 1.0))
    satin_black = Material("mat_satin_black", color=(0.005, 0.006, 0.007, 1.0))
    oiled_steel = Material("mat_oiled_steel", color=(0.42, 0.43, 0.42, 1.0))
    worn_edge = Material("mat_worn_edge", color=(0.62, 0.61, 0.56, 1.0))

    base = model.part("base_cup")
    base.visual(
        Cylinder(radius=0.115, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_metal,
        name="console_flange",
    )
    base.visual(
        mesh_from_cadquery(annular_plate(0.102, 0.079, 0.032), "base_cup_wall"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_black,
        name="cup_wall",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=Material("mat_shadow_well", color=(0.0, 0.0, 0.0, 1.0)),
        name="recess_floor",
    )
    for sign, name in ((-1.0, "bearing_post_0"), (1.0, "bearing_post_1")):
        base.visual(
            Box((0.016, 0.030, 0.054)),
            origin=Origin(xyz=(sign * 0.088, 0.0, 0.048)),
            material=dark_metal,
            name=name,
        )
        base.visual(
            Cylinder(radius=0.012, length=0.009),
            origin=Origin(xyz=(sign * 0.0805, 0.0, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=oiled_steel,
            name=f"pitch_bushing_{0 if sign < 0 else 1}",
        )

    outer = model.part("outer_ring")
    outer.visual(
        mesh_from_cadquery(annular_plate(0.066, 0.052, 0.008), "outer_pitch_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=oiled_steel,
        name="pitch_ring",
    )
    for sign, name in ((-1.0, "pitch_pin_0"), (1.0, "pitch_pin_1")):
        outer.visual(
            Cylinder(radius=0.0052, length=0.022),
            origin=Origin(xyz=(sign * 0.069, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_edge,
            name=name,
        )
    for sign, name in ((-1.0, "roll_bushing_0"), (1.0, "roll_bushing_1")):
        outer.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(0.0, sign * 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_edge,
            name=name,
        )

    inner = model.part("inner_ring")
    inner.visual(
        mesh_from_cadquery(annular_plate(0.044, 0.032, 0.007), "inner_roll_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.0035)),
        material=oiled_steel,
        name="roll_ring",
    )
    inner.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=oiled_steel,
        name="central_hub",
    )
    inner.visual(
        Box((0.064, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=oiled_steel,
        name="spoke_x",
    )
    inner.visual(
        Box((0.008, 0.064, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=oiled_steel,
        name="spoke_y",
    )
    for sign, name in ((-1.0, "roll_pin_0"), (1.0, "roll_pin_1")):
        inner.visual(
            Cylinder(radius=0.0032, length=0.0143),
            origin=Origin(xyz=(0.0, sign * 0.0449, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_edge,
            name=name,
        )
    inner.visual(
        Cylinder(radius=0.008, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark_metal,
        name="lever_stem",
    )
    inner.visual(
        Box((0.036, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=dark_metal,
        name="clamp_block",
    )
    inner.visual(
        Cylinder(radius=0.0032, length=0.026),
        origin=Origin(xyz=(-0.010, 0.0, 0.091), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="clamp_screw_0",
    )
    inner.visual(
        Cylinder(radius=0.0032, length=0.026),
        origin=Origin(xyz=(0.010, 0.0, 0.091), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="clamp_screw_1",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-LIMIT, upper=LIMIT),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=inner,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=-LIMIT, upper=LIMIT),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_ring")
    inner = object_model.get_part("inner_ring")
    pitch = object_model.get_articulation("base_to_outer")
    roll = object_model.get_articulation("outer_to_inner")

    for pin_name in ("roll_pin_0", "roll_pin_1"):
        ctx.allow_overlap(
            inner,
            outer,
            elem_a=pin_name,
            elem_b="pitch_ring",
            reason="The small roll trunnion pin is intentionally captured just inside the outer gimbal ring bearing.",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="y",
            elem_a=pin_name,
            elem_b="pitch_ring",
            min_overlap=0.00005,
            name=f"{pin_name} remains seated in the outer ring bearing",
        )

    ctx.check(
        "pitch range is eighteen degrees each way",
        abs(pitch.motion_limits.lower + LIMIT) < 1e-6 and abs(pitch.motion_limits.upper - LIMIT) < 1e-6,
        details=str(pitch.motion_limits),
    )
    ctx.check(
        "roll range is eighteen degrees each way",
        abs(roll.motion_limits.lower + LIMIT) < 1e-6 and abs(roll.motion_limits.upper - LIMIT) < 1e-6,
        details=str(roll.motion_limits),
    )
    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="roll_ring",
        outer_elem="pitch_ring",
        margin=0.002,
        name="inner ring nests inside outer pitch ring",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="roll_ring",
        elem_b="pitch_ring",
        min_overlap=0.005,
        name="cardan rings share the same pivot plane",
    )

    rest_aabb = ctx.part_element_world_aabb(inner, elem="clamp_block")
    with ctx.pose({pitch: LIMIT}):
        pitched_aabb = ctx.part_element_world_aabb(inner, elem="clamp_block")
    with ctx.pose({roll: LIMIT}):
        rolled_aabb = ctx.part_element_world_aabb(inner, elem="clamp_block")

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb else None

    rest_center = aabb_center(rest_aabb)
    pitch_center = aabb_center(pitched_aabb)
    roll_center = aabb_center(rolled_aabb)
    ctx.check(
        "pitch motion tilts the upright lever",
        rest_center is not None and pitch_center is not None and abs(pitch_center[1] - rest_center[1]) > 0.015,
        details=f"rest={rest_center}, pitched={pitch_center}",
    )
    ctx.check(
        "roll motion tilts the upright lever",
        rest_center is not None and roll_center is not None and abs(roll_center[0] - rest_center[0]) > 0.015,
        details=f"rest={rest_center}, rolled={roll_center}",
    )

    return ctx.report()


object_model = build_object_model()
