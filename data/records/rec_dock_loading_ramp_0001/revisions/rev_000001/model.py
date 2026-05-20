from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
BASE_FRAME = "base_frame"
DECK = "deck"
LIP = "lip"
DECK_HINGE = "deck_hinge"
LIP_HINGE = "lip_hinge"


def _add_box(part, size, xyz, material, rpy=(0.0, 0.0, 0.0), name=None) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius,
    length,
    xyz,
    material,
    rpy=(0.0, 0.0, 0.0),
    name=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dock_loading_ramp", assets=ASSETS)

    painted_steel = Material(
        name="painted_steel",
        rgba=(0.27, 0.29, 0.31, 1.0),
    )
    safety_yellow = Material(
        name="safety_yellow",
        rgba=(0.89, 0.74, 0.16, 1.0),
    )
    brushed_steel = Material(
        name="brushed_steel",
        rgba=(0.76, 0.77, 0.79, 1.0),
    )
    rubber_black = Material(
        name="rubber_black",
        rgba=(0.08, 0.08, 0.09, 1.0),
    )
    model.materials.extend([painted_steel, safety_yellow, brushed_steel, rubber_black])

    base_frame = model.part(BASE_FRAME)
    _add_box(base_frame, (2.50, 0.56, 0.10), (0.0, -0.28, 0.05), painted_steel)
    _add_box(base_frame, (2.44, 0.08, 0.12), (0.0, -0.02, 0.12), painted_steel)
    _add_box(base_frame, (0.14, 0.18, 0.24), (1.16, -0.08, 0.12), painted_steel)
    _add_box(base_frame, (0.14, 0.18, 0.24), (-1.16, -0.08, 0.12), painted_steel)
    _add_box(base_frame, (0.20, 0.10, 0.12), (0.84, -0.02, 0.20), painted_steel)
    _add_box(base_frame, (0.20, 0.10, 0.12), (-0.84, -0.02, 0.20), painted_steel)
    _add_box(base_frame, (1.86, 0.16, 0.16), (0.0, -0.22, 0.14), painted_steel)
    _add_box(base_frame, (0.18, 0.22, 0.11), (0.82, -0.12, 0.055), painted_steel)
    _add_box(base_frame, (0.18, 0.22, 0.11), (-0.82, -0.12, 0.055), painted_steel)
    _add_box(base_frame, (0.42, 0.24, 0.18), (-0.72, -0.30, 0.19), painted_steel)
    _add_cylinder(
        base_frame,
        radius=0.055,
        length=0.30,
        xyz=(-0.48, -0.24, 0.20),
        material=brushed_steel,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _add_cylinder(
        base_frame,
        radius=0.06,
        length=0.28,
        xyz=(0.0, -0.10, 0.14),
        material=painted_steel,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _add_box(base_frame, (0.18, 0.10, 0.16), (0.0, -0.22, 0.13), painted_steel)
    _add_box(base_frame, (0.18, 0.12, 0.28), (0.80, -0.52, 0.14), rubber_black)
    _add_box(base_frame, (0.18, 0.12, 0.28), (-0.80, -0.52, 0.14), rubber_black)
    base_frame.inertial = Inertial.from_geometry(
        Box((2.50, 0.56, 0.10)),
        mass=820.0,
        origin=Origin(xyz=(0.0, -0.28, 0.05)),
    )

    deck = model.part(DECK)
    _add_box(deck, (2.18, 0.08, 0.06), (0.0, 0.04, 0.03), painted_steel)
    _add_box(deck, (2.30, 2.02, 0.05), (0.0, 1.05, 0.025), painted_steel)
    _add_box(deck, (0.08, 2.02, 0.14), (1.11, 1.05, 0.07), safety_yellow)
    _add_box(deck, (0.08, 2.02, 0.14), (-1.11, 1.05, 0.07), safety_yellow)
    _add_box(deck, (0.18, 1.86, 0.10), (0.62, 1.01, -0.03), painted_steel)
    _add_box(deck, (0.18, 1.86, 0.10), (-0.62, 1.01, -0.03), painted_steel)
    _add_box(deck, (0.22, 1.68, 0.12), (0.0, 0.93, -0.04), painted_steel)
    _add_box(deck, (2.18, 0.10, 0.06), (0.0, 2.01, 0.03), painted_steel)
    _add_box(deck, (2.18, 0.06, 0.03), (0.0, 2.04, 0.055), safety_yellow)
    for strip_x in (-0.72, -0.36, 0.0, 0.36, 0.72):
        _add_box(deck, (0.16, 1.70, 0.008), (strip_x, 1.10, 0.052), brushed_steel)
    _add_cylinder(
        deck,
        radius=0.035,
        length=0.34,
        xyz=(0.0, 0.22, -0.06),
        material=brushed_steel,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    deck.inertial = Inertial.from_geometry(
        Box((2.30, 2.02, 0.05)),
        mass=1350.0,
        origin=Origin(xyz=(0.0, 1.05, 0.025)),
    )

    lip = model.part(LIP)
    _add_box(lip, (2.10, 0.08, 0.03), (0.0, 0.02, -0.005), painted_steel)
    _add_box(lip, (2.18, 0.42, 0.035), (0.0, 0.21, -0.0175), safety_yellow)
    _add_box(lip, (0.10, 0.18, 0.10), (1.00, 0.09, -0.03), painted_steel)
    _add_box(lip, (0.10, 0.18, 0.10), (-1.00, 0.09, -0.03), painted_steel)
    _add_box(lip, (2.18, 0.06, 0.015), (0.0, 0.39, -0.0375), brushed_steel)
    _add_box(lip, (0.26, 0.22, 0.01), (-0.60, 0.23, 0.004), brushed_steel)
    _add_box(lip, (0.26, 0.22, 0.01), (0.0, 0.23, 0.004), brushed_steel)
    _add_box(lip, (0.26, 0.22, 0.01), (0.60, 0.23, 0.004), brushed_steel)
    lip.inertial = Inertial.from_geometry(
        Box((2.18, 0.42, 0.035)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.21, -0.0175)),
    )

    model.articulation(
        DECK_HINGE,
        ArticulationType.REVOLUTE,
        parent=BASE_FRAME,
        child=DECK,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=0.5,
            lower=0.0,
            upper=0.52,
        ),
    )
    model.articulation(
        LIP_HINGE,
        ArticulationType.REVOLUTE,
        parent=DECK,
        child=LIP,
        origin=Origin(xyz=(0.0, 2.04, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.7,
            lower=-0.85,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        BASE_FRAME,
        DECK,
        reason="rear hinge beam and dock-frame pivot bracket intentionally share conservative collision envelopes at the hinge line",
    )
    ctx.allow_overlap(
        DECK,
        LIP,
        reason="folding lip nests into the deck front edge and contacts along the industrial hinge seam",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        DECK_HINGE,
        DECK,
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        LIP_HINGE,
        LIP,
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )

    with ctx.pose(deck_hinge=0.0, lip_hinge=0.0):
        ctx.expect_origin_distance(DECK, BASE_FRAME, axes="xy", max_dist=1.45)
        ctx.expect_aabb_overlap(LIP, DECK, axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap(DECK, BASE_FRAME, axes="xy", min_overlap=0.03)

    with ctx.pose(deck_hinge=0.45, lip_hinge=0.0):
        ctx.expect_origin_distance(DECK, BASE_FRAME, axes="xy", max_dist=1.35)
        ctx.expect_aabb_overlap(LIP, DECK, axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap(DECK, BASE_FRAME, axes="xy", min_overlap=0.05)

    with ctx.pose(deck_hinge=0.0, lip_hinge=-0.85):
        ctx.expect_aabb_overlap(LIP, DECK, axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap(DECK, BASE_FRAME, axes="xy", min_overlap=0.03)

    with ctx.pose(deck_hinge=0.0, lip_hinge=0.10):
        ctx.expect_aabb_overlap(LIP, DECK, axes="xy", min_overlap=0.015)

    with ctx.pose(deck_hinge=0.45, lip_hinge=-0.85):
        ctx.expect_aabb_overlap(LIP, DECK, axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap(DECK, BASE_FRAME, axes="xy", min_overlap=0.05)

    with ctx.pose(deck_hinge=0.45, lip_hinge=0.10):
        ctx.expect_aabb_overlap(LIP, DECK, axes="xy", min_overlap=0.015)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
