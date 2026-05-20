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
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged: MeshGeometry | None = None
    for geometry in geometries:
        merged = geometry if merged is None else merged.merge(geometry)
    if merged is None:
        raise ValueError("expected at least one geometry")
    return merged


def _straight_tube(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    radial_segments: int,
) -> MeshGeometry:
    return wire_from_points(
        [start, end],
        radius=radius,
        radial_segments=radial_segments,
        cap_ends=True,
        corner_mode="miter",
    )


def _build_tower_frame_mesh():
    geoms: list[MeshGeometry] = []

    base_corners = [
        (-0.34, -0.34, 0.20),
        (-0.34, 0.34, 0.20),
        (0.34, 0.34, 0.20),
        (0.34, -0.34, 0.20),
    ]
    top_corners = [
        (-0.15, -0.15, 2.22),
        (-0.15, 0.15, 2.22),
        (0.15, 0.15, 2.22),
        (0.15, -0.15, 2.22),
    ]

    for base_pt, top_pt in zip(base_corners, top_corners):
        mid_pt = (
            0.52 * base_pt[0] + 0.48 * top_pt[0],
            0.52 * base_pt[1] + 0.48 * top_pt[1],
            1.20,
        )
        geoms.append(
            tube_from_spline_points(
                [base_pt, mid_pt, top_pt],
                radius=0.018,
                samples_per_segment=18,
                radial_segments=16,
                cap_ends=True,
            )
        )

    for extent, z in ((0.28, 0.68), (0.22, 1.28), (0.18, 1.86)):
        corners = [
            (-extent, -extent, z),
            (-extent, extent, z),
            (extent, extent, z),
            (extent, -extent, z),
        ]
        for start, end in zip(corners, corners[1:] + corners[:1]):
            geoms.append(
                _straight_tube(
                    start,
                    end,
                    radius=0.011,
                    radial_segments=12,
                )
            )

    brace_radius = 0.010
    brace_specs = [
        [(-0.30, 0.30, 0.34), (0.21, 0.21, 1.28)],
        [(0.30, 0.30, 0.34), (-0.21, 0.21, 1.28)],
        [(-0.30, -0.30, 0.34), (0.21, -0.21, 1.28)],
        [(0.30, -0.30, 0.34), (-0.21, -0.21, 1.28)],
        [(-0.30, -0.30, 1.02), (-0.18, -0.18, 1.88)],
        [(-0.30, 0.30, 1.02), (-0.18, 0.18, 1.88)],
        [(0.30, -0.30, 1.02), (0.18, -0.18, 1.88)],
        [(0.30, 0.30, 1.02), (0.18, 0.18, 1.88)],
    ]
    for points in brace_specs:
        geoms.append(
            _straight_tube(
                points[0],
                points[1],
                radius=brace_radius,
                radial_segments=12,
            )
        )

    for x in (-0.045, 0.045):
        geoms.append(
            _straight_tube(
                (x, -0.095, 0.46),
                (x, -0.095, 1.94),
                radius=0.009,
                radial_segments=10,
            )
        )
    for z in (0.62, 0.82, 1.02, 1.22, 1.42, 1.62, 1.82):
        geoms.append(
            _straight_tube(
                (-0.045, -0.095, z),
                (0.045, -0.095, z),
                radius=0.006,
                radial_segments=8,
            )
        )

    geoms.append(
        tube_from_spline_points(
            [
                (0.40, -0.10, 0.82),
                (0.33, -0.12, 1.10),
                (0.24, -0.15, 1.56),
                (0.11, -0.11, 2.22),
            ],
            radius=0.012,
            samples_per_segment=14,
            radial_segments=12,
            cap_ends=True,
        )
    )

    return mesh_from_geometry(
        _merge_geometries(geoms),
        ASSETS.mesh_path("searchlight_tower_frame.obj"),
    )


def _build_lamp_shell_mesh():
    profile = [
        (0.030, -0.220),
        (0.080, -0.205),
        (0.145, -0.175),
        (0.195, -0.120),
        (0.220, -0.045),
        (0.230, 0.055),
        (0.225, 0.145),
        (0.190, 0.215),
        (0.105, 0.248),
        (0.030, 0.258),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=44),
        ASSETS.mesh_path("searchlight_head_shell.obj"),
    )


def _build_head_handle_mesh():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.010, -0.395, 0.060),
                (-0.105, -0.250, 0.175),
                (-0.010, -0.105, 0.060),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=12,
            cap_ends=True,
        ),
        ASSETS.mesh_path("searchlight_head_handle.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower", assets=ASSETS)

    materials = {
        "concrete": Material(name="concrete", rgba=(0.58, 0.58, 0.57, 1.0)),
        "painted_steel": Material(name="painted_steel", rgba=(0.28, 0.35, 0.40, 1.0)),
        "dark_steel": Material(name="dark_steel", rgba=(0.18, 0.20, 0.22, 1.0)),
        "galvanized_steel": Material(name="galvanized_steel", rgba=(0.66, 0.68, 0.70, 1.0)),
        "stainless_steel": Material(name="stainless_steel", rgba=(0.80, 0.81, 0.83, 1.0)),
        "glass_lens": Material(name="glass_lens", rgba=(0.84, 0.93, 0.98, 0.35)),
    }
    model.materials.extend(materials.values())

    tower_frame_mesh = _build_tower_frame_mesh()
    lamp_shell_mesh = _build_lamp_shell_mesh()
    lamp_handle_mesh = _build_head_handle_mesh()

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((1.20, 1.20, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=materials["concrete"],
    )
    tower_base.visual(
        Cylinder(radius=0.20, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=materials["painted_steel"],
    )
    tower_base.visual(
        Cylinder(radius=0.10, length=1.58),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=materials["painted_steel"],
    )
    tower_base.visual(
        Cylinder(radius=0.14, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 2.12)),
        material=materials["dark_steel"],
    )
    tower_base.visual(
        Cylinder(radius=0.23, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.28)),
        material=materials["dark_steel"],
    )
    tower_base.visual(
        Box((0.34, 0.26, 0.62)),
        origin=Origin(xyz=(0.40, 0.0, 0.51)),
        material=materials["painted_steel"],
    )
    tower_base.visual(
        Box((0.014, 0.22, 0.52)),
        origin=Origin(xyz=(0.577, 0.0, 0.51)),
        material=materials["dark_steel"],
    )
    tower_base.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.587, 0.0, 0.53)),
        material=materials["stainless_steel"],
    )
    tower_base.visual(tower_frame_mesh, material=materials["galvanized_steel"])
    tower_base.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 2.40)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.19, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=materials["dark_steel"],
    )
    turret.visual(
        Cylinder(radius=0.15, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=materials["painted_steel"],
    )
    turret.visual(
        Box((0.18, 0.18, 0.10)),
        origin=Origin(xyz=(-0.02, 0.0, 0.22)),
        material=materials["painted_steel"],
    )
    turret.visual(
        Box((0.14, 0.14, 0.34)),
        origin=Origin(xyz=(-0.04, 0.0, 0.35)),
        material=materials["painted_steel"],
    )
    turret.visual(
        Box((0.22, 0.05, 0.62)),
        origin=Origin(xyz=(-0.05, 0.275, 0.40)),
        material=materials["painted_steel"],
    )
    turret.visual(
        Box((0.22, 0.05, 0.62)),
        origin=Origin(xyz=(-0.05, -0.275, 0.40)),
        material=materials["painted_steel"],
    )
    turret.visual(
        Box((0.08, 0.56, 0.08)),
        origin=Origin(xyz=(-0.16, 0.0, 0.48)),
        material=materials["dark_steel"],
    )
    turret.visual(
        Cylinder(radius=0.045, length=0.56),
        origin=Origin(xyz=(0.03, 0.0, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["dark_steel"],
    )
    turret.visual(
        Box((0.06, 0.56, 0.06)),
        origin=Origin(xyz=(-0.15, 0.0, 0.68)),
        material=materials["dark_steel"],
    )
    turret.visual(
        Box((0.10, 0.12, 0.18)),
        origin=Origin(xyz=(-0.06, 0.345, 0.33)),
        material=materials["dark_steel"],
    )
    turret.visual(
        Box((0.08, 0.10, 0.14)),
        origin=Origin(xyz=(-0.05, -0.34, 0.30)),
        material=materials["dark_steel"],
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.42, 0.72, 0.80)),
        mass=140.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.40)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        lamp_shell_mesh,
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["painted_steel"],
    )
    lamp_head.visual(
        Cylinder(radius=0.245, length=0.050),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["stainless_steel"],
    )
    lamp_head.visual(
        Cylinder(radius=0.218, length=0.008),
        origin=Origin(xyz=(0.368, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["glass_lens"],
    )
    lamp_head.visual(
        Cylinder(radius=0.132, length=0.080),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Cylinder(radius=0.052, length=0.044),
        origin=Origin(xyz=(0.03, 0.226, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Cylinder(radius=0.052, length=0.044),
        origin=Origin(xyz=(0.03, -0.226, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.05, 0.10, 0.13)),
        origin=Origin(xyz=(0.03, 0.175, 0.0)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.05, 0.10, 0.13)),
        origin=Origin(xyz=(0.03, -0.175, 0.0)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.20, 0.18, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, 0.18)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.18, 0.16, 0.09)),
        origin=Origin(xyz=(0.04, 0.0, -0.17)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.15, 0.46, 0.018)),
        origin=Origin(xyz=(0.30, 0.0, 0.18)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.05, 0.018, 0.10)),
        origin=Origin(xyz=(0.31, -0.22, 0.13)),
        material=materials["dark_steel"],
    )
    lamp_head.visual(
        Box((0.05, 0.018, 0.10)),
        origin=Origin(xyz=(0.31, 0.22, 0.13)),
        material=materials["dark_steel"],
    )
    for fin_x in (-0.065, -0.040, -0.015, 0.010):
        lamp_head.visual(
            Box((0.008, 0.28, 0.19)),
            origin=Origin(xyz=(fin_x, 0.0, 0.0)),
            material=materials["dark_steel"],
        )
    lamp_head.visual(
        lamp_handle_mesh,
        origin=Origin(xyz=(0.0, 0.25, 0.0)),
        material=materials["galvanized_steel"],
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.56, 0.54, 0.50)),
        mass=82.0,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    model.articulation(
        "searchlight_yaw",
        ArticulationType.REVOLUTE,
        parent="tower_base",
        child="turret",
        origin=Origin(xyz=(0.0, 0.0, 2.322)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.2,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "searchlight_pitch",
        ArticulationType.REVOLUTE,
        parent="turret",
        child="lamp_head",
        origin=Origin(xyz=(0.03, 0.0, 0.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.9,
            lower=-0.35,
            upper=0.95,
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
        "tower_base",
        "turret",
        reason="slewing bearing races sit with tight real-world running clearance",
    )
    ctx.allow_overlap(
        "turret",
        "lamp_head",
        reason="trunnion clevis and lamp bosses are intentionally modeled with tight industrial clearance",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("turret", "tower_base", axes="xy", max_dist=0.08)
    ctx.expect_aabb_overlap("turret", "tower_base", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_gap("turret", "tower_base", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_origin_distance("lamp_head", "turret", axes="xy", max_dist=0.28)
    ctx.expect_origin_distance("lamp_head", "tower_base", axes="xy", max_dist=0.30)
    ctx.expect_aabb_overlap("lamp_head", "tower_base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_overlap("lamp_head", "turret", axes="xy", min_overlap=0.10)
    ctx.expect_joint_motion_axis(
        "searchlight_yaw",
        "lamp_head",
        world_axis="y",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "searchlight_pitch",
        "lamp_head",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(searchlight_pitch=0.9):
        ctx.expect_origin_distance("lamp_head", "tower_base", axes="xy", max_dist=0.34)
        ctx.expect_aabb_overlap("lamp_head", "tower_base", axes="xy", min_overlap=0.10)
        ctx.expect_aabb_overlap("lamp_head", "turret", axes="xy", min_overlap=0.08)

    with ctx.pose(searchlight_pitch=-0.30):
        ctx.expect_origin_distance("lamp_head", "tower_base", axes="xy", max_dist=0.35)
        ctx.expect_aabb_overlap("lamp_head", "tower_base", axes="xy", min_overlap=0.12)
        ctx.expect_aabb_overlap("lamp_head", "turret", axes="xy", min_overlap=0.10)

    with ctx.pose(searchlight_yaw=1.5):
        ctx.expect_origin_distance("turret", "tower_base", axes="xy", max_dist=0.08)
        ctx.expect_origin_distance("lamp_head", "tower_base", axes="xy", max_dist=0.32)
        ctx.expect_aabb_overlap("lamp_head", "tower_base", axes="xy", min_overlap=0.10)

    with ctx.pose(searchlight_yaw=-1.2, searchlight_pitch=0.85):
        ctx.expect_origin_distance("lamp_head", "tower_base", axes="xy", max_dist=0.35)
        ctx.expect_aabb_overlap("lamp_head", "tower_base", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("lamp_head", "turret", axes="xy", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
