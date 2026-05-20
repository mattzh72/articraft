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
    mesh_from_cadquery,
)


VANE_COUNT = 6
VANE_SPACING = 0.105
VANE_Z0 = -0.5 * VANE_SPACING * (VANE_COUNT - 1)


def _vane_z(index: int) -> float:
    return VANE_Z0 + index * VANE_SPACING


def _airfoil_vane(length: float, chord: float, thickness: float) -> cq.Workplane:
    """A lightly cambered louver slat, extruded along the shaft direction."""
    half_chord = chord / 2.0
    half_thick = thickness / 2.0
    profile = [
        (-half_chord, -0.001),
        (-0.37 * chord, half_thick * 0.85),
        (-0.06 * chord, half_thick),
        (0.34 * chord, half_thick * 0.55),
        (half_chord, 0.001),
        (0.34 * chord, -half_thick * 0.55),
        (-0.06 * chord, -half_thick),
        (-0.37 * chord, -half_thick * 0.85),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _bearing_ring(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """A short annular bearing collar aligned with the louver shaft axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_louver_vanes")

    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.73, 0.72, 1.0))
    warm_bearing = model.material("warm_bearing", rgba=(0.78, 0.58, 0.28, 1.0))
    blue_handles = model.material("blue_adjuster_grips", rgba=(0.08, 0.26, 0.55, 1.0))

    frame_width = 0.96
    frame_height = 0.72
    frame_depth = 0.14
    rail = 0.06
    side_x = frame_width / 2.0 - rail / 2.0
    rail_z = frame_height / 2.0 - rail / 2.0

    frame = model.part("frame")
    frame.visual(
        Box((rail, frame_depth, frame_height)),
        origin=Origin(xyz=(-side_x, 0.0, 0.0)),
        material=dark_anodized,
        name="side_rail_0",
    )
    frame.visual(
        Box((rail, frame_depth, frame_height)),
        origin=Origin(xyz=(side_x, 0.0, 0.0)),
        material=dark_anodized,
        name="side_rail_1",
    )
    frame.visual(
        Box((frame_width, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, rail_z)),
        material=dark_anodized,
        name="top_rail",
    )
    frame.visual(
        Box((frame_width, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, -rail_z)),
        material=dark_anodized,
        name="bottom_rail",
    )
    # A slim rear tie bar keeps the side plates visibly square without blocking
    # the front view of the adjustable vanes.
    frame.visual(
        Box((0.84, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.078, 0.0)),
        material=dark_anodized,
        name="rear_tie_bar",
    )

    bearing_mesh = mesh_from_cadquery(_bearing_ring(0.026, 0.023, 0.0075), "bearing_ring")
    for i in range(VANE_COUNT):
        z = _vane_z(i)
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(-0.419, 0.0, z)),
            material=warm_bearing,
            name=f"bearing_0_{i}",
        )
        frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.419, 0.0, z), rpy=(0.0, 0.0, math.pi)),
            material=warm_bearing,
            name=f"bearing_1_{i}",
        )

    blade_mesh = mesh_from_cadquery(_airfoil_vane(0.78, 0.105, 0.018), "cambered_vane")
    for i in range(VANE_COUNT):
        vane = model.part(f"vane_{i}")
        vane.visual(
            blade_mesh,
            origin=Origin(),
            material=satin_aluminum,
            name="airfoil_blade",
        )
        vane.visual(
            Cylinder(radius=0.0075, length=0.82),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_bearing,
            name="shaft",
        )
        vane.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(xyz=(0.370, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_bearing,
            name="adjuster_hub",
        )
        vane.visual(
            Box((0.052, 0.078, 0.012)),
            origin=Origin(xyz=(0.370, 0.040, 0.0)),
            material=blue_handles,
            name="adjuster_tab",
        )

        model.articulation(
            f"frame_to_vane_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, _vane_z(i))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.6,
                lower=-0.60,
                upper=0.60,
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    vanes = [object_model.get_part(f"vane_{i}") for i in range(VANE_COUNT)]
    joints = [object_model.get_articulation(f"frame_to_vane_{i}") for i in range(VANE_COUNT)]

    ctx.check(
        "six independent pivoting vanes",
        len(vanes) == VANE_COUNT and len(set(joints)) == VANE_COUNT,
        details=f"vanes={len(vanes)}, joints={len(joints)}",
    )
    ctx.check(
        "all shafts use parallel horizontal axes",
        all(tuple(j.axis or ()) == (1.0, 0.0, 0.0) for j in joints),
        details=[j.axis for j in joints],
    )
    for i in range(VANE_COUNT - 1):
        ctx.expect_origin_gap(
            vanes[i + 1],
            vanes[i],
            axis="z",
            min_gap=VANE_SPACING - 0.001,
            max_gap=VANE_SPACING + 0.001,
            name=f"shaft spacing {i}",
        )
        ctx.expect_within(
            vanes[i],
            frame,
            axes="x",
            margin=0.002,
            inner_elem="shaft",
            name=f"shaft {i} fits between side bearings",
        )

    vane_2_rest = ctx.part_world_position(vanes[2])
    vane_3_rest = ctx.part_world_position(vanes[3])
    with ctx.pose({joints[2]: 0.50}):
        vane_2_moved = ctx.part_world_position(vanes[2])
        vane_3_moved = ctx.part_world_position(vanes[3])
        ctx.check(
            "vane pivots about its own fixed shaft",
            vane_2_rest is not None
            and vane_2_moved is not None
            and all(abs(a - b) < 1.0e-6 for a, b in zip(vane_2_rest, vane_2_moved)),
            details=f"rest={vane_2_rest}, moved={vane_2_moved}",
        )
        ctx.check(
            "neighbor remains independent",
            vane_3_rest is not None
            and vane_3_moved is not None
            and all(abs(a - b) < 1.0e-6 for a, b in zip(vane_3_rest, vane_3_moved)),
            details=f"rest={vane_3_rest}, moved={vane_3_moved}",
        )

    with ctx.pose({joints[i]: (-0.60 if i % 2 == 0 else 0.60) for i in range(VANE_COUNT)}):
        for i in range(VANE_COUNT - 1):
            ctx.expect_gap(
                vanes[i + 1],
                vanes[i],
                axis="z",
                min_gap=0.004,
                name=f"opposed vanes clear {i}",
            )

    return ctx.report()


object_model = build_object_model()
