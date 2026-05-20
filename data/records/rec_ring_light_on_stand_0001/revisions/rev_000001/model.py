from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba)


def _mirrored_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="creator_ring_light", assets=ASSETS)

    powder_black = _material("powder_black", (0.12, 0.12, 0.13, 1.0))
    satin_black = _material("satin_black", (0.18, 0.18, 0.19, 1.0))
    dark_plastic = _material("dark_plastic", (0.16, 0.16, 0.17, 1.0))
    brushed_steel = _material("brushed_steel", (0.58, 0.60, 0.63, 1.0))
    rubber = _material("rubber", (0.06, 0.06, 0.07, 1.0))
    diffuser_white = _material("diffuser_white", (0.97, 0.96, 0.92, 0.82))

    ring_outer_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.195, tube=0.032, radial_segments=22, tubular_segments=60),
        ASSETS.mesh_path("ring_outer_housing.obj"),
    )
    ring_diffuser_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.195, tube=0.019, radial_segments=18, tubular_segments=60),
        ASSETS.mesh_path("ring_diffuser.obj"),
    )
    ring_heat_sink_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.186, tube=0.024, radial_segments=20, tubular_segments=56),
        ASSETS.mesh_path("ring_heat_sink.obj"),
    )

    tripod_leg_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.046, 0.0, 0.082),
                (0.120, 0.0, 0.072),
                (0.250, 0.0, 0.030),
                (0.360, 0.0, 0.015),
            ],
            radius=0.010,
            samples_per_segment=14,
            radial_segments=16,
        ),
        ASSETS.mesh_path("tripod_leg.obj"),
    )
    tripod_brace_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.038, 0.0, 0.120),
                (0.082, 0.0, 0.098),
                (0.140, 0.0, 0.072),
            ],
            radius=0.006,
            samples_per_segment=10,
            radial_segments=14,
        ),
        ASSETS.mesh_path("tripod_brace.obj"),
    )

    fork_arm_path = [
        (0.000, 0.000, 0.015),
        (0.045, -0.012, 0.032),
        (0.110, -0.016, 0.078),
        (0.165, -0.016, 0.118),
        (0.188, -0.015, 0.145),
    ]
    fork_right_mesh = mesh_from_geometry(
        tube_from_spline_points(
            fork_arm_path,
            radius=0.008,
            samples_per_segment=16,
            radial_segments=16,
        ),
        ASSETS.mesh_path("fork_arm_right.obj"),
    )
    fork_left_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _mirrored_x(fork_arm_path),
            radius=0.008,
            samples_per_segment=16,
            radial_segments=16,
        ),
        ASSETS.mesh_path("fork_arm_left.obj"),
    )

    stand = model.part("stand_base")
    stand.visual(
        Cylinder(radius=0.050, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=powder_black,
    )
    stand.visual(
        Cylinder(radius=0.038, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=satin_black,
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.840),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=powder_black,
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=brushed_steel,
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
        material=powder_black,
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.030, 0.0, 0.690), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.048, 0.0, 0.690), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
    )
    for yaw in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0):
        stand.visual(tripod_leg_mesh, origin=Origin(rpy=(0.0, 0.0, yaw)), material=powder_black)
        stand.visual(tripod_brace_mesh, origin=Origin(rpy=(0.0, 0.0, yaw)), material=brushed_steel)
        stand.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.360, 0.0, 0.015), rpy=(0.0, 0.0, yaw)),
            material=rubber,
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.760, 0.760, 0.940)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.015, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=powder_black,
    )
    upper_column.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_steel,
    )
    upper_column.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=powder_black,
    )
    upper_column.visual(
        Box((0.034, 0.050, 0.140)),
        origin=Origin(xyz=(0.0, -0.028, 0.285)),
        material=powder_black,
    )
    upper_column.visual(
        Box((0.072, 0.046, 0.068)),
        origin=Origin(xyz=(0.0, -0.020, 0.360)),
        material=powder_black,
    )
    upper_column.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.360), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    upper_column.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(-0.048, 0.0, 0.360), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
    )
    upper_column.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.048, 0.0, 0.360), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
    )
    upper_column.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.028, -0.045, 0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
    )
    upper_column.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.028, -0.060, 0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
    )
    upper_column.inertial = Inertial.from_geometry(
        Box((0.120, 0.090, 0.460)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.010, 0.230)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        ring_outer_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.210), rpy=(pi / 2.0, 0.0, 0.0)),
        material=powder_black,
    )
    ring_head.visual(
        ring_heat_sink_mesh,
        origin=Origin(xyz=(0.0, -0.014, 0.210), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
    )
    ring_head.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.0, 0.012, 0.210), rpy=(pi / 2.0, 0.0, 0.0)),
        material=diffuser_white,
    )
    ring_head.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    ring_head.visual(
        Box((0.060, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=powder_black,
    )
    ring_head.visual(fork_right_mesh, material=powder_black)
    ring_head.visual(fork_left_mesh, material=powder_black)
    ring_head.visual(
        Box((0.028, 0.024, 0.230)),
        origin=Origin(xyz=(0.0, -0.006, 0.115)),
        material=powder_black,
    )
    ring_head.visual(
        Box((0.020, 0.100, 0.016)),
        origin=Origin(xyz=(0.0, 0.050, 0.098)),
        material=powder_black,
    )
    ring_head.visual(
        Box((0.016, 0.028, 0.070)),
        origin=Origin(xyz=(0.0, 0.074, 0.072)),
        material=powder_black,
    )
    ring_head.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.086, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
    )
    ring_head.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.078, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
    )
    ring_head.visual(
        Box((0.085, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, -0.035, 0.070)),
        material=dark_plastic,
    )
    ring_head.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-0.198, -0.015, 0.210), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    ring_head.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.198, -0.015, 0.210), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.480, 0.090, 0.500)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
    )

    device_mount = model.part("device_mount")
    device_mount.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
    )
    device_mount.visual(
        Box((0.020, 0.012, 0.118)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=powder_black,
    )
    device_mount.visual(
        Box((0.042, 0.022, 0.048)),
        origin=Origin(xyz=(0.0, 0.002, 0.106)),
        material=dark_plastic,
    )
    device_mount.visual(
        Box((0.090, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.006, 0.146)),
        material=dark_plastic,
    )
    device_mount.visual(
        Box((0.090, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.006, 0.066)),
        material=dark_plastic,
    )
    device_mount.visual(
        Box((0.070, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.015, 0.146)),
        material=rubber,
    )
    device_mount.visual(
        Box((0.070, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.015, 0.066)),
        material=rubber,
    )
    device_mount.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, 0.118)),
        material=brushed_steel,
    )
    device_mount.inertial = Inertial.from_geometry(
        Box((0.100, 0.030, 0.150)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.004, 0.090)),
    )

    model.articulation(
        "stand_height",
        ArticulationType.PRISMATIC,
        parent="stand_base",
        child="upper_column",
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.20),
    )
    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent="upper_column",
        child="ring_head",
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.55, upper=0.40),
    )
    model.articulation(
        "mount_fix",
        ArticulationType.FIXED,
        parent="ring_head",
        child="device_mount",
        origin=Origin(xyz=(0.0, 0.078, 0.045)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("upper_column", "stand_base", axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance("ring_head", "stand_base", axes="xy", max_dist=0.08)
    ctx.expect_aabb_overlap("ring_head", "upper_column", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("device_mount", "ring_head", axes="xy", min_overlap=0.01)
    ctx.expect_joint_motion_axis(
        "stand_height",
        "upper_column",
        world_axis="z",
        direction="positive",
        min_delta=0.12,
    )
    ctx.expect_joint_motion_axis(
        "stand_height",
        "ring_head",
        world_axis="z",
        direction="positive",
        min_delta=0.12,
    )

    with ctx.pose(stand_height=0.20):
        ctx.expect_aabb_overlap("upper_column", "stand_base", axes="xy", min_overlap=0.02)
        ctx.expect_origin_distance("ring_head", "stand_base", axes="xy", max_dist=0.08)
        ctx.expect_aabb_overlap("ring_head", "upper_column", axes="xy", min_overlap=0.03)

    with ctx.pose(ring_tilt=-0.55):
        ctx.expect_origin_distance("ring_head", "upper_column", axes="xy", max_dist=0.16)
        ctx.expect_aabb_overlap("ring_head", "upper_column", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_overlap("device_mount", "ring_head", axes="xy", min_overlap=0.01)

    with ctx.pose(ring_tilt=0.40):
        ctx.expect_origin_distance("ring_head", "upper_column", axes="xy", max_dist=0.16)
        ctx.expect_aabb_overlap("ring_head", "upper_column", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_overlap("device_mount", "ring_head", axes="xy", min_overlap=0.01)

    with ctx.pose(stand_height=0.20, ring_tilt=-0.55):
        ctx.expect_origin_distance("ring_head", "stand_base", axes="xy", max_dist=0.18)
        ctx.expect_aabb_overlap("device_mount", "ring_head", axes="xy", min_overlap=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
