from __future__ import annotations

import math

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
    model = ArticulatedObject(name="low_table_pitch_stage")

    dark_machined = model.material("dark_machined", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.63, 0.66, 0.67, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    blue_gray = model.material("blue_gray_anodized", rgba=(0.20, 0.27, 0.34, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.75, 0.73, 1.0))

    base = model.part("base")

    # A wide, shallow ground plate establishes the low table silhouette.
    base.visual(
        Box((0.68, 0.46, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_machined,
        name="ground_plate",
    )

    # A low bolted yoke: a shallow saddle base with two short side cheeks.  The
    # moving table fits in the open span and its trunnion bears on the cheek
    # inner faces at the horizontal pitch axis.
    base.visual(
        Box((0.46, 0.36, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=satin_aluminum,
        name="yoke_foot",
    )
    base.visual(
        Box((0.060, 0.36, 0.127)),
        origin=Origin(xyz=(-0.200, 0.0, 0.1165)),
        material=satin_aluminum,
        name="cheek_neg",
    )
    base.visual(
        Box((0.060, 0.36, 0.127)),
        origin=Origin(xyz=(0.200, 0.0, 0.1165)),
        material=satin_aluminum,
        name="cheek_pos",
    )
    for x, suffix in ((-0.234, "neg"), (0.234, "pos")):
        base.visual(
            Cylinder(radius=0.036, length=0.008),
            origin=Origin(xyz=(x, 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"outer_bearing_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.020, length=0.009),
            origin=Origin(xyz=(x + (0.001 if x < 0 else -0.001), 0.0, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_machined,
            name=f"bore_shadow_{suffix}",
        )

    # Recessed dark rails on the grounded plate make the assembly read as a
    # precision pitch stage instead of a plain bracket.
    for y in (-0.155, 0.155):
        base.visual(
            Box((0.58, 0.018, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.037)),
            material=blue_gray,
            name=f"side_rail_{'neg' if y < 0 else 'pos'}",
        )

    # Visible fasteners and rubber feet are mounted into the base, not separate
    # articulated controls.
    for i, (x, y) in enumerate(
        ((-0.270, -0.175), (0.270, -0.175), (-0.270, 0.175), (0.270, 0.175))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.038)),
            material=brushed_steel,
            name=f"base_bolt_{i}",
        )
        base.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(x, y, 0.005)),
            material=black_rubber,
            name=f"foot_{i}",
        )

    platform = model.part("platform")
    # The child frame is exactly on the trunnion axis.  At q=0 the table is
    # level and the tabletop sits just above the axis for a flat, compact stage.
    platform.visual(
        Box((0.305, 0.335, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=blue_gray,
        name="tabletop",
    )
    platform.visual(
        Box((0.250, 0.070, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=blue_gray,
        name="center_boss",
    )
    platform.visual(
        Cylinder(radius=0.018, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion",
    )
    for x in (-0.150, 0.150):
        platform.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"bearing_collar_{'neg' if x < 0 else 'pos'}",
        )

    model.articulation(
        "base_to_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    hinge = object_model.get_articulation("base_to_platform")

    ctx.check(
        "single revolute pitch joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "horizontal pitch axis",
        tuple(round(v, 6) for v in hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "limited low-angle travel",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == -0.35
        and hinge.motion_limits.upper == 0.35,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            base,
            platform,
            axis="x",
            positive_elem="cheek_pos",
            negative_elem="tabletop",
            min_gap=0.010,
            max_gap=0.030,
            name="positive cheek clears tabletop",
        )
        ctx.expect_gap(
            platform,
            base,
            axis="x",
            positive_elem="tabletop",
            negative_elem="cheek_neg",
            min_gap=0.010,
            max_gap=0.030,
            name="negative cheek clears tabletop",
        )
        ctx.expect_contact(
            platform,
            base,
            elem_a="trunnion",
            elem_b="cheek_neg",
            name="trunnion bears on negative cheek",
        )
        ctx.expect_contact(
            platform,
            base,
            elem_a="trunnion",
            elem_b="cheek_pos",
            name="trunnion bears on positive cheek",
        )
        rest_table = ctx.part_element_world_aabb(platform, elem="tabletop")

    with ctx.pose({hinge: 0.35}):
        raised_table = ctx.part_element_world_aabb(platform, elem="tabletop")

    ctx.check(
        "positive pitch visibly raises a table edge",
        rest_table is not None
        and raised_table is not None
        and raised_table[1][2] > rest_table[1][2] + 0.035,
        details=f"rest={rest_table}, raised={raised_table}",
    )

    return ctx.report()


object_model = build_object_model()
