from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _armored_housing_shell():
    sections = [
        (-0.18, 0.035, 0.165, 0.17),
        (-0.08, 0.025, 0.210, 0.24),
        (0.02, 0.020, 0.250, 0.28),
        (0.12, 0.030, 0.255, 0.27),
        (0.22, 0.060, 0.205, 0.18),
    ]
    shell = superellipse_side_loft(
        sections,
        exponents=(2.2, 2.6, 3.0, 2.8, 2.3),
        segments=56,
        cap=True,
        closed=True,
    )
    shell.rotate_z(-math.pi / 2.0)
    return _save_mesh(shell, "rws_armored_housing.obj")


def _yoke_plate_mesh():
    profile = [
        (-0.050, -0.080),
        (-0.015, -0.110),
        (0.055, -0.105),
        (0.130, -0.065),
        (0.175, -0.010),
        (0.180, 0.060),
        (0.100, 0.095),
        (0.010, 0.090),
        (-0.045, 0.040),
    ]
    yoke = ExtrudeGeometry(profile, height=0.018, cap=True, center=True, closed=True)
    yoke.rotate_x(math.pi / 2.0)
    return _save_mesh(yoke, "rws_cradle_yoke.obj")


def _feed_conduit_mesh():
    conduit = tube_from_spline_points(
        [
            (-0.005, -0.110, -0.020),
            (0.035, -0.085, 0.020),
            (0.085, -0.060, 0.040),
            (0.140, -0.035, 0.020),
        ],
        radius=0.007,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )
    return _save_mesh(conduit, "rws_feed_conduit.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station", assets=ASSETS)

    olive_paint = model.material("olive_paint", rgba=(0.34, 0.38, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.25, 1.0))
    parkerized = model.material("parkerized", rgba=(0.16, 0.17, 0.15, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.11, 0.11, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.30, 0.42, 0.46, 0.45))
    fastener_steel = model.material("fastener_steel", rgba=(0.52, 0.54, 0.56, 1.0))

    housing_shell = _armored_housing_shell()
    yoke_mesh = _yoke_plate_mesh()
    conduit_mesh = _feed_conduit_mesh()

    mount_base = model.part("mount_base")
    mount_base.visual(
        Cylinder(radius=0.24, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
    )
    mount_base.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=olive_paint,
    )
    mount_base.visual(
        Cylinder(radius=0.115, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=olive_paint,
    )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        mount_base.visual(
            Box((0.04, 0.10, 0.16)),
            origin=Origin(
                xyz=(0.10 * math.cos(angle), 0.10 * math.sin(angle), 0.12),
                rpy=(0.0, 0.55, angle),
            ),
            material=dark_steel,
        )
    mount_base.visual(
        Box((0.12, 0.08, 0.10)),
        origin=Origin(xyz=(-0.13, 0.0, 0.11)),
        material=olive_paint,
    )
    mount_base.visual(
        Box((0.10, 0.06, 0.055)),
        origin=Origin(xyz=(-0.13, 0.0, 0.1775)),
        material=dark_steel,
    )
    mount_base.visual(
        Cylinder(radius=0.18, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=dark_steel,
    )
    for idx in range(8):
        angle = idx * (math.tau / 8.0)
        mount_base.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.15 * math.cos(angle), 0.15 * math.sin(angle), 0.276)),
            material=fastener_steel,
        )
    mount_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.28),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    turret_housing = model.part("turret_housing")
    turret_housing.visual(
        Cylinder(radius=0.17, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
    )
    turret_housing.visual(housing_shell, origin=Origin(), material=olive_paint)
    turret_housing.visual(
        Box((0.075, 0.18, 0.06)),
        origin=Origin(xyz=(0.08, 0.0, 0.155)),
        material=dark_steel,
    )
    turret_housing.visual(
        Box((0.14, 0.03, 0.11)),
        origin=Origin(xyz=(0.02, 0.135, 0.155)),
        material=olive_paint,
    )
    turret_housing.visual(
        Box((0.14, 0.03, 0.11)),
        origin=Origin(xyz=(0.02, -0.135, 0.155)),
        material=olive_paint,
    )
    turret_housing.visual(
        Cylinder(radius=0.033, length=0.036),
        origin=Origin(xyz=(0.08, 0.134, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )
    turret_housing.visual(
        Cylinder(radius=0.033, length=0.036),
        origin=Origin(xyz=(0.08, -0.134, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )
    turret_housing.visual(
        Box((0.10, 0.24, 0.045)),
        origin=Origin(xyz=(0.175, 0.0, 0.115), rpy=(0.0, 0.32, 0.0)),
        material=olive_paint,
    )
    turret_housing.visual(
        Box((0.16, 0.18, 0.02)),
        origin=Origin(xyz=(0.02, 0.0, 0.252), rpy=(0.0, -0.10, 0.0)),
        material=dark_steel,
    )
    turret_housing.visual(
        Box((0.08, 0.14, 0.055)),
        origin=Origin(xyz=(-0.15, 0.0, 0.145)),
        material=dark_steel,
    )
    turret_housing.visual(
        Box((0.11, 0.07, 0.09)),
        origin=Origin(xyz=(0.11, 0.105, 0.175)),
        material=olive_paint,
    )
    turret_housing.visual(
        Box((0.015, 0.045, 0.055)),
        origin=Origin(xyz=(0.165, 0.105, 0.185)),
        material=sensor_glass,
    )
    turret_housing.visual(
        Box((0.050, 0.015, 0.045)),
        origin=Origin(xyz=(0.110, 0.142, 0.175)),
        material=sensor_glass,
    )
    turret_housing.visual(
        Box((0.06, 0.03, 0.18)),
        origin=Origin(xyz=(0.10, -0.133, 0.16)),
        material=dark_steel,
    )
    for z in (0.11, 0.145, 0.18):
        turret_housing.visual(
            Cylinder(radius=0.018, length=0.09),
            origin=Origin(
                xyz=(0.15, -0.145, z),
                rpy=(0.0, math.pi / 2.0 - 0.35, 0.0),
            ),
            material=dark_steel,
        )
    turret_housing.inertial = Inertial.from_geometry(
        Box((0.32, 0.28, 0.26)),
        mass=58.0,
        origin=Origin(xyz=(0.02, 0.0, 0.13)),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.03, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.111, 0.0)),
        material=olive_paint,
    )
    weapon_cradle.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, -0.111, 0.0)),
        material=olive_paint,
    )
    weapon_cradle.visual(
        Box((0.06, 0.18, 0.06)),
        origin=Origin(xyz=(-0.03, 0.0, -0.02)),
        material=olive_paint,
    )
    weapon_cradle.visual(
        Box((0.09, 0.21, 0.05)),
        origin=Origin(xyz=(0.055, 0.0, -0.055)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Box((0.23, 0.095, 0.09)),
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Box((0.18, 0.13, 0.05)),
        origin=Origin(xyz=(0.13, 0.0, 0.075), rpy=(0.0, -0.12, 0.0)),
        material=olive_paint,
    )
    weapon_cradle.visual(
        Box((0.12, 0.06, 0.07)),
        origin=Origin(xyz=(0.10, 0.072, -0.015)),
        material=black_polymer,
    )
    weapon_cradle.visual(
        Box((0.16, 0.11, 0.14)),
        origin=Origin(xyz=(0.04, -0.12, -0.03)),
        material=black_polymer,
    )
    weapon_cradle.visual(
        Box((0.11, 0.055, 0.06)),
        origin=Origin(xyz=(0.11, -0.065, -0.002)),
        material=black_polymer,
    )
    weapon_cradle.visual(conduit_mesh, origin=Origin(), material=dark_steel)
    weapon_cradle.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.028, length=0.42),
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.016, length=0.52),
        origin=Origin(xyz=(0.47, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.024, length=0.085),
        origin=Origin(xyz=(0.71, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((0.78, 0.26, 0.22)),
        mass=42.0,
        origin=Origin(xyz=(0.27, 0.0, -0.01)),
    )

    model.articulation(
        "turret_pan",
        ArticulationType.CONTINUOUS,
        parent="mount_base",
        child="turret_housing",
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=1.0),
    )
    model.articulation(
        "cradle_elevation",
        ArticulationType.REVOLUTE,
        parent="turret_housing",
        child="weapon_cradle",
        origin=Origin(xyz=(0.08, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("turret_housing", "mount_base", axes="xy", min_overlap=0.22)
    ctx.expect_origin_distance("turret_housing", "mount_base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_gap(
        "turret_housing",
        "mount_base",
        axis="z",
        max_gap=0.003,
        max_penetration=0.003,
    )
    ctx.expect_aabb_contact("weapon_cradle", "turret_housing")
    ctx.expect_aabb_overlap("weapon_cradle", "turret_housing", axes="yz", min_overlap=0.08)
    ctx.expect_joint_motion_axis(
        "cradle_elevation",
        "weapon_cradle",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    for angle in (0.0, math.pi / 2.0, math.pi):
        with ctx.pose(turret_pan=angle):
            ctx.expect_aabb_overlap("turret_housing", "mount_base", axes="xy", min_overlap=0.22)
            ctx.expect_origin_distance("turret_housing", "mount_base", axes="xy", max_dist=0.01)
            ctx.expect_aabb_gap(
                "turret_housing",
                "mount_base",
                axis="z",
                max_gap=0.003,
                max_penetration=0.003,
            )
            ctx.expect_aabb_contact("weapon_cradle", "turret_housing")

    for elevation in (-0.35, 0.0, 1.05):
        with ctx.pose(cradle_elevation=elevation):
            ctx.expect_aabb_contact("weapon_cradle", "turret_housing")
            ctx.expect_aabb_overlap(
                "weapon_cradle",
                "turret_housing",
                axes="yz",
                min_overlap=0.08,
            )

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
