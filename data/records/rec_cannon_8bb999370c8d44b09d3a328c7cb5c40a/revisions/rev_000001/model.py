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


def _bombard_barrel_mesh():
    """Short wrought-iron bombard barrel with a visible blind bore."""
    # CadQuery profile is drawn in the XZ half-section and revolved about X.
    # The polygon includes a thick closed breech and an open muzzle bore.
    half_section = [
        (-0.58, 0.00),
        (-0.58, 0.22),
        (-0.53, 0.30),
        (-0.42, 0.36),
        (-0.30, 0.31),
        (-0.08, 0.31),
        (-0.05, 0.34),
        (0.05, 0.34),
        (0.08, 0.31),
        (0.33, 0.30),
        (0.39, 0.34),
        (0.50, 0.34),
        (0.56, 0.31),
        (0.70, 0.34),
        (0.82, 0.39),
        (0.86, 0.18),
        (0.52, 0.18),
        (0.20, 0.16),
        (-0.12, 0.14),
        (-0.22, 0.05),
        (-0.25, 0.00),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(half_section)
        .close()
        .revolve(360.0, axisStart=(-1.0, 0.0, 0.0), axisEnd=(1.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_siege_bombard")

    oak = model.material("weathered_oak", rgba=(0.46, 0.28, 0.14, 1.0))
    endgrain = model.material("dark_endgrain", rgba=(0.30, 0.18, 0.09, 1.0))
    wrought_iron = model.material("blackened_wrought_iron", rgba=(0.055, 0.058, 0.060, 1.0))
    polished_wear = model.material("rubbed_iron_wear", rgba=(0.18, 0.17, 0.15, 1.0))
    bore_black = model.material("sooty_bore", rgba=(0.005, 0.004, 0.003, 1.0))

    cradle = model.part("cradle")
    # Low timber trough bed.
    cradle.visual(
        Box((2.10, 1.02, 0.18)),
        origin=Origin(xyz=(0.04, 0.0, -0.70)),
        material=oak,
        name="trough_floor",
    )
    cradle.visual(
        Box((0.88, 0.16, 0.62)),
        origin=Origin(xyz=(-0.62, 0.58, -0.31)),
        material=oak,
        name="rear_cheek_0",
    )
    cradle.visual(
        Box((0.88, 0.16, 0.62)),
        origin=Origin(xyz=(-0.62, -0.58, -0.31)),
        material=oak,
        name="rear_cheek_1",
    )
    cradle.visual(
        Box((0.88, 0.16, 0.62)),
        origin=Origin(xyz=(0.62, 0.58, -0.31)),
        material=oak,
        name="front_cheek_0",
    )
    cradle.visual(
        Box((0.88, 0.16, 0.62)),
        origin=Origin(xyz=(0.62, -0.58, -0.31)),
        material=oak,
        name="front_cheek_1",
    )
    # Sloped inner timbers make the bed read as a trough rather than a flat cart.
    cradle.visual(
        Box((1.72, 0.11, 0.13)),
        origin=Origin(xyz=(0.10, 0.25, -0.51), rpy=(0.22, 0.0, 0.0)),
        material=oak,
        name="inner_rail_0",
    )
    cradle.visual(
        Box((1.72, 0.11, 0.13)),
        origin=Origin(xyz=(0.10, -0.25, -0.51), rpy=(-0.22, 0.0, 0.0)),
        material=oak,
        name="inner_rail_1",
    )
    cradle.visual(
        Box((0.18, 1.16, 0.30)),
        origin=Origin(xyz=(-1.06, 0.0, -0.46)),
        material=endgrain,
        name="rear_tie",
    )
    cradle.visual(
        Box((0.18, 1.16, 0.30)),
        origin=Origin(xyz=(1.04, 0.0, -0.46)),
        material=endgrain,
        name="front_tie",
    )
    cradle.visual(
        Box((0.18, 1.02, 0.42)),
        origin=Origin(xyz=(-0.73, 0.0, -0.31)),
        material=oak,
        name="breech_stop",
    )

    # Timber bearing forks around the two trunnions, split so the barrel can clear.
    for side, y in enumerate((0.68, -0.68)):
        cradle.visual(
            Box((0.10, 0.15, 0.34)),
            origin=Origin(xyz=(-0.14, y, -0.01)),
            material=oak,
            name=f"bearing_post_{side}_0",
        )
        cradle.visual(
            Box((0.10, 0.15, 0.34)),
            origin=Origin(xyz=(0.14, y, -0.01)),
            material=oak,
            name=f"bearing_post_{side}_1",
        )
        cradle.visual(
            Box((0.34, 0.15, 0.10)),
            origin=Origin(xyz=(0.0, y, -0.15)),
            material=oak,
            name=f"bearing_sill_{side}",
        )
    cradle.visual(
        Box((0.24, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, 0.68, -0.094)),
        material=polished_wear,
        name="bearing_saddle_0",
    )
    cradle.visual(
        Box((0.24, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, -0.68, -0.094)),
        material=polished_wear,
        name="bearing_saddle_1",
    )

    # Iron straps over the timber cheeks and across the tie blocks.
    for idx, x in enumerate((-0.92, -0.22, 0.34, 0.86)):
        cradle.visual(
            Box((0.035, 0.025, 0.66)),
            origin=Origin(xyz=(x, 0.672, -0.29)),
            material=wrought_iron,
            name=f"side_strap_{idx}_0",
        )
        cradle.visual(
            Box((0.035, 0.025, 0.66)),
            origin=Origin(xyz=(x, -0.672, -0.29)),
            material=wrought_iron,
            name=f"side_strap_{idx}_1",
        )
    for idx, x in enumerate((-1.06, 1.04)):
        cradle.visual(
            Box((0.045, 1.22, 0.050)),
            origin=Origin(xyz=(x, 0.0, -0.28)),
            material=wrought_iron,
            name=f"cross_strap_{idx}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_bombard_barrel_mesh(), "squat_bombard_barrel", tolerance=0.0015),
        material=wrought_iron,
        name="barrel_shell",
    )
    # Two separate trunnion stubs are integral to the barrel but visibly sit in the cradle sides.
    barrel.visual(
        Cylinder(radius=0.085, length=0.34),
        origin=Origin(xyz=(0.0, 0.465, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_wear,
        name="trunnion_0",
    )
    barrel.visual(
        Cylinder(radius=0.085, length=0.34),
        origin=Origin(xyz=(0.0, -0.465, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_wear,
        name="trunnion_1",
    )
    barrel.visual(
        Cylinder(radius=0.128, length=0.052),
        origin=Origin(xyz=(0.0, 0.323, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="trunnion_collar_0",
    )
    barrel.visual(
        Cylinder(radius=0.128, length=0.052),
        origin=Origin(xyz=(0.0, -0.323, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="trunnion_collar_1",
    )
    # A dark blind-bore surface makes the open muzzle read as hollow.
    barrel.visual(
        Cylinder(radius=0.181, length=0.50),
        origin=Origin(xyz=(0.62, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bore_black,
        name="muzzle_bore",
    )

    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.35, lower=-0.04, upper=0.34),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cradle = object_model.get_part("cradle")
    barrel = object_model.get_part("barrel")
    elevation = object_model.get_articulation("elevation_axis")
    limits = elevation.motion_limits

    ctx.check(
        "barrel pivots on a revolute trunnion axis",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in elevation.axis) == (0.0, -1.0, 0.0),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    ctx.check(
        "bombard elevation has restrained period limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= 0.0
        and 0.25 <= limits.upper <= 0.45,
        details=f"limits={limits}",
    )
    ctx.expect_contact(
        barrel,
        cradle,
        elem_a="trunnion_0",
        elem_b="bearing_saddle_0",
        contact_tol=0.003,
        name="upper trunnion rests in its saddle",
    )
    ctx.expect_contact(
        barrel,
        cradle,
        elem_a="trunnion_1",
        elem_b="bearing_saddle_1",
        contact_tol=0.003,
        name="lower trunnion rests in its saddle",
    )
    ctx.expect_overlap(
        barrel,
        cradle,
        axes="xy",
        min_overlap=0.55,
        name="squat barrel sits within timber trough footprint",
    )

    rest_aabb = ctx.part_world_aabb(barrel)
    with ctx.pose({elevation: limits.upper if limits is not None else 0.0}):
        raised_aabb = ctx.part_world_aabb(barrel)
    ctx.check(
        "positive elevation raises the muzzle end",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )
    return ctx.report()


object_model = build_object_model()
