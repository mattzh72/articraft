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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _helical_points(
    *,
    radius: float,
    z_start: float,
    rise: float,
    turns: float,
    count: int,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(count):
        s = i / (count - 1)
        angle = phase + turns * math.tau * s
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_start + rise * s,
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle", assets=ASSETS)

    bottle = model.part("bottle")
    bottle.visual(
        Cylinder(radius=0.0335, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    bottle.visual(
        Cylinder(radius=0.0340, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )
    bottle.visual(
        Cylinder(radius=0.0305, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
    )
    bottle.visual(
        Cylinder(radius=0.0350, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )
    bottle.visual(
        Cylinder(radius=0.0335, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
    )
    bottle.visual(
        Cylinder(radius=0.0280, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
    )
    bottle.visual(
        Cylinder(radius=0.0220, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.1835)),
    )
    bottle.visual(
        Cylinder(radius=0.0180, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.1915)),
    )
    bottle.visual(
        Cylinder(radius=0.0145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.203)),
    )
    bottle.visual(
        Cylinder(radius=0.0150, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
    )
    bottle.visual(
        Cylinder(radius=0.0130, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.221)),
    )

    for z in (0.061, 0.069, 0.077, 0.085):
        bottle.visual(
            Cylinder(radius=0.0317, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, z)),
        )

    for z in (0.091, 0.145):
        bottle.visual(
            Cylinder(radius=0.0358, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, z)),
        )

    bottle.visual(
        Cylinder(radius=0.0358, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )
    bottle.visual(
        Cylinder(radius=0.0164, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.1985)),
    )
    bottle.visual(
        Cylinder(radius=0.0160, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.2125)),
    )

    thread_geom = tube_from_spline_points(
        _helical_points(
            radius=0.0151,
            z_start=0.1990,
            rise=0.0120,
            turns=1.45,
            count=14,
        ),
        radius=0.0010,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    bottle.visual(
        mesh_from_geometry(thread_geom, ASSETS.mesh_path("bottle_neck_thread.obj")),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.222),
        mass=0.070,
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.0195, length=0.0195),
        origin=Origin(xyz=(0.0, 0.0, 0.01625)),
    )
    cap.visual(
        Cylinder(radius=0.0188, length=0.0070),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
    )
    cap.visual(
        Cylinder(radius=0.0176, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0245)),
    )

    for i in range(24):
        angle = math.tau * i / 24.0
        cap.visual(
            Box((0.0018, 0.0036, 0.0140)),
            origin=Origin(
                xyz=(0.0187 * math.cos(angle), 0.0187 * math.sin(angle), 0.0150),
                rpy=(0.0, 0.0, angle),
            ),
        )

    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.026),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    model.articulation(
        "cap_thread",
        ArticulationType.REVOLUTE,
        parent="bottle",
        child="cap",
        origin=Origin(xyz=(0.0, 0.0, 0.196)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=10.0,
            lower=0.0,
            upper=9.42477796076938,
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
        "bottle",
        "cap",
        reason=(
            "The cap is authored as a realistic closure shell around the threaded neck, "
            "so conservative generated collision hulls treat the enclosure as overlap."
        ),
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    for angle in (0.0, math.pi, 2.0 * math.pi, 3.0 * math.pi):
        with ctx.pose(cap_thread=angle):
            ctx.expect_aabb_overlap("cap", "bottle", axes="xy", min_overlap=0.032)
            ctx.expect_origin_distance("cap", "bottle", axes="xy", max_dist=0.006)
            ctx.expect_aabb_gap("cap", "bottle", axis="z", max_gap=0.004, max_penetration=0.032)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
