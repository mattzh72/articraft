from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
HALF_PI = math.pi / 2.0


def _box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy))


def _cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy))


def _add_guardrail(
    part,
    x0: float,
    x1: float,
    y: float,
    deck_top_z: float,
    post_count: int,
) -> None:
    span = x1 - x0
    rail_center_x = 0.5 * (x0 + x1)
    for rail_z in (deck_top_z + 0.055, deck_top_z + 0.115):
        _cylinder(
            part,
            radius=0.005,
            length=span,
            xyz=(rail_center_x, y, rail_z),
            rpy=(0.0, HALF_PI, 0.0),
        )
    for index in range(post_count):
        if post_count == 1:
            x = rail_center_x
        else:
            x = x0 + span * index / (post_count - 1)
        _cylinder(part, radius=0.0045, length=0.13, xyz=(x, y, deck_top_z + 0.065))


def _add_leaf_truss_web(part, y: float) -> None:
    panel_x = (0.16, 0.36, 0.56, 0.76, 0.96, 1.16)
    for x in panel_x:
        _box(part, (0.016, 0.016, 0.18), (x, y, 0.05))
    for index, x in enumerate((0.21, 0.41, 0.61, 0.81, 1.01)):
        angle = 0.78 if index % 2 == 0 else -0.78
        _box(part, (0.27, 0.014, 0.014), (x, y, 0.05), rpy=(0.0, angle, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bascule_bridge", assets=ASSETS)

    pier_base = model.part("pier_base")
    _box(pier_base, (0.72, 0.60, 0.36), (-0.22, 0.0, 0.18))
    _box(pier_base, (0.32, 0.10, 0.30), (-0.16, -0.19, 0.51))
    _box(pier_base, (0.32, 0.10, 0.30), (-0.16, 0.19, 0.51))
    _box(pier_base, (0.18, 0.30, 0.10), (-0.08, 0.0, 0.60))
    _box(pier_base, (0.82, 0.46, 0.06), (-0.47, 0.0, 0.66))
    _box(pier_base, (0.76, 0.32, 0.008), (-0.49, 0.0, 0.694))
    _box(pier_base, (0.82, 0.04, 0.18), (-0.47, -0.21, 0.71))
    _box(pier_base, (0.82, 0.04, 0.18), (-0.47, 0.21, 0.71))
    _box(pier_base, (0.08, 0.44, 0.10), (-0.04, 0.0, 0.64))
    _cylinder(pier_base, radius=0.05, length=0.36, xyz=(0.08, -0.18, 0.18))
    _cylinder(pier_base, radius=0.05, length=0.36, xyz=(0.08, 0.18, 0.18))
    _add_guardrail(pier_base, -0.90, -0.08, -0.16, 0.69, post_count=6)
    _add_guardrail(pier_base, -0.90, -0.08, 0.16, 0.69, post_count=6)
    pier_base.inertial = Inertial.from_geometry(
        Box((0.92, 0.60, 0.80)),
        mass=4500.0,
        origin=Origin(xyz=(-0.30, 0.0, 0.40)),
    )

    tower_frame = model.part("tower_frame")
    _box(tower_frame, (0.18, 0.08, 0.70), (-0.08, -0.27, 0.04))
    _box(tower_frame, (0.18, 0.08, 0.70), (-0.08, 0.27, 0.04))
    _box(tower_frame, (0.16, 0.14, 0.18), (-0.04, -0.24, 0.0))
    _box(tower_frame, (0.16, 0.14, 0.18), (-0.04, 0.24, 0.0))
    _cylinder(tower_frame, radius=0.055, length=0.40, xyz=(0.0, 0.0, 0.0), rpy=(HALF_PI, 0.0, 0.0))
    _box(tower_frame, (0.30, 0.62, 0.08), (-0.08, 0.0, 0.41))
    _box(tower_frame, (0.24, 0.48, 0.05), (-0.10, 0.0, 0.16))
    brace_angle = -math.atan2(0.34, 0.18)
    _box(tower_frame, (0.40, 0.03, 0.03), (-0.13, -0.27, 0.05), rpy=(0.0, brace_angle, 0.0))
    _box(tower_frame, (0.40, 0.03, 0.03), (-0.13, 0.27, 0.05), rpy=(0.0, brace_angle, 0.0))
    _cylinder(
        tower_frame,
        radius=0.006,
        length=0.50,
        xyz=(-0.08, 0.0, 0.45),
        rpy=(HALF_PI, 0.0, 0.0),
    )
    tower_frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.66, 0.82)),
        mass=1800.0,
        origin=Origin(xyz=(-0.08, 0.0, 0.12)),
    )

    leaf_span = model.part("leaf_span")
    _cylinder(leaf_span, radius=0.07, length=0.36, xyz=(0.0, 0.0, 0.0), rpy=(HALF_PI, 0.0, 0.0))
    _box(leaf_span, (0.10, 0.44, 0.08), (0.10, 0.0, 0.0))
    _box(leaf_span, (1.22, 0.44, 0.06), (0.71, 0.0, -0.01))
    _box(leaf_span, (1.10, 0.32, 0.008), (0.73, 0.0, 0.024))
    _box(leaf_span, (1.22, 0.04, 0.18), (0.71, -0.21, 0.05))
    _box(leaf_span, (1.22, 0.04, 0.18), (0.71, 0.21, 0.05))
    _box(leaf_span, (0.04, 0.44, 0.10), (1.32, 0.0, 0.02))
    for x in (0.14, 0.32, 0.50, 0.68, 0.86, 1.04, 1.22):
        _box(leaf_span, (0.03, 0.36, 0.10), (x, 0.0, -0.01))
    _add_leaf_truss_web(leaf_span, -0.205)
    _add_leaf_truss_web(leaf_span, 0.205)
    _add_guardrail(leaf_span, 0.12, 1.26, -0.16, 0.02, post_count=7)
    _add_guardrail(leaf_span, 0.12, 1.26, 0.16, 0.02, post_count=7)
    arm_angle = 2.60
    _box(leaf_span, (0.38, 0.03, 0.03), (-0.18, -0.14, -0.11), rpy=(0.0, arm_angle, 0.0))
    _box(leaf_span, (0.38, 0.03, 0.03), (-0.18, 0.14, -0.11), rpy=(0.0, arm_angle, 0.0))
    _box(leaf_span, (0.22, 0.22, 0.03), (-0.22, 0.0, -0.12))
    _box(leaf_span, (0.36, 0.32, 0.18), (-0.34, 0.0, -0.21))
    _box(leaf_span, (0.24, 0.02, 0.02), (-0.22, 0.0, -0.04), rpy=(0.0, 2.18, 0.0))
    leaf_span.inertial = Inertial.from_geometry(
        Box((1.82, 0.44, 0.46)),
        mass=2200.0,
        origin=Origin(xyz=(0.38, 0.0, -0.05)),
    )

    tip_signal = model.part("tip_signal")
    _box(tip_signal, (0.08, 0.30, 0.02), (0.0, 0.0, 0.01))
    _cylinder(tip_signal, radius=0.012, length=0.18, xyz=(0.0, 0.0, 0.11))
    _box(tip_signal, (0.06, 0.24, 0.04), (0.0, 0.0, 0.20))
    _cylinder(
        tip_signal, radius=0.018, length=0.03, xyz=(0.0, -0.08, 0.20), rpy=(0.0, HALF_PI, 0.0)
    )
    _cylinder(tip_signal, radius=0.018, length=0.03, xyz=(0.0, 0.08, 0.20), rpy=(0.0, HALF_PI, 0.0))
    tip_signal.inertial = Inertial.from_geometry(
        Box((0.10, 0.30, 0.24)),
        mass=25.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    model.articulation(
        "pier_to_tower",
        ArticulationType.FIXED,
        parent="pier_base",
        child="tower_frame",
        origin=Origin(xyz=(-0.06, 0.0, 0.67)),
    )
    model.articulation(
        "leaf_lift",
        ArticulationType.REVOLUTE,
        parent="tower_frame",
        child="leaf_span",
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.7,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "leaf_to_tip_signal",
        ArticulationType.FIXED,
        parent="leaf_span",
        child="tip_signal",
        origin=Origin(xyz=(1.32, 0.0, 0.05)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "tower_frame",
        "leaf_span",
        reason="The trunnion drum sits inside the bascule bearing assembly and conservative generated collision hulls exaggerate this hinge contact.",
    )
    ctx.allow_overlap(
        "leaf_span",
        "pier_base",
        reason="The bascule heel nests into the pier-side bearing seat at the trunnion, and generated collision hulls are conservative around that pocketed hinge region.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)
    ctx.expect_joint_motion_axis(
        "leaf_lift", "leaf_span", world_axis="z", direction="positive", min_delta=0.30
    )
    ctx.expect_joint_motion_axis(
        "leaf_lift", "leaf_span", world_axis="x", direction="negative", min_delta=0.15
    )
    with ctx.pose(leaf_lift=0.0):
        ctx.expect_aabb_overlap("leaf_span", "pier_base", axes="xy", min_overlap=0.10)
        tip_closed = ctx.part_world_position("tip_signal")
    with ctx.pose(leaf_lift=1.15):
        ctx.expect_aabb_overlap("leaf_span", "tower_frame", axes="xy", min_overlap=0.08)
        tip_open = ctx.part_world_position("tip_signal")

    if tip_closed[0] < 1.10:
        raise AssertionError("Closed bridge tip should project well beyond the support pier.")
    if not 0.70 <= tip_closed[2] <= 0.85:
        raise AssertionError("Closed bridge tip should sit near the roadway elevation.")
    if tip_open[2] < tip_closed[2] + 0.75:
        raise AssertionError("Raised bascule span should lift the tip dramatically upward.")
    if tip_open[0] > tip_closed[0] - 0.55:
        raise AssertionError("Raised bascule span should pull the tip back toward the pivot.")
    if abs(tip_open[1] - tip_closed[1]) > 0.02:
        raise AssertionError("Lift motion should remain in a vertical plane without lateral drift.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
