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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _dish_shell_mesh():
    profile = [
        (0.0, -0.035),
        (0.08, -0.030),
        (0.22, -0.010),
        (0.46, 0.060),
        (0.70, 0.172),
        (0.79, 0.226),
        (0.79, 0.242),
        (0.62, 0.190),
        (0.38, 0.092),
        (0.16, 0.022),
        (0.05, -0.006),
        (0.0, 0.0),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72), ASSETS.mesh_path("dish_shell.obj")
    )


def _feed_arm_mesh(filename: str, points: list[tuple[float, float, float]], radius: float = 0.018):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radar_dish", assets=ASSETS)

    pedestal = model.part("pedestal")
    pedestal.visual(Cylinder(radius=0.62, length=0.14), origin=Origin(xyz=(0.0, 0.0, 0.07)))
    pedestal.visual(Cylinder(radius=0.48, length=0.18), origin=Origin(xyz=(0.0, 0.0, 0.23)))
    pedestal.visual(Cylinder(radius=0.22, length=0.78), origin=Origin(xyz=(0.0, 0.0, 0.70)))
    pedestal.visual(Cylinder(radius=0.30, length=0.12), origin=Origin(xyz=(0.0, 0.0, 1.15)))
    pedestal.visual(Box((0.44, 0.34, 0.52)), origin=Origin(xyz=(-0.34, 0.0, 0.42)))
    pedestal.visual(Box((0.16, 0.20, 0.18)), origin=Origin(xyz=(-0.20, 0.0, 1.04)))
    pedestal.inertial = Inertial.from_geometry(
        Box((1.24, 1.24, 1.25)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(Cylinder(radius=0.28, length=0.08), origin=Origin(xyz=(0.0, 0.0, 0.04)))
    yaw_head.visual(Box((0.70, 0.56, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.14)))
    yaw_head.visual(Box((0.48, 0.38, 0.28)), origin=Origin(xyz=(-0.04, 0.0, 0.34)))
    yaw_head.visual(Box((0.22, 0.28, 0.18)), origin=Origin(xyz=(-0.34, 0.0, 0.34)))
    yaw_head.visual(Box((0.30, 0.10, 0.26)), origin=Origin(xyz=(0.04, 0.23, 0.41)))
    yaw_head.visual(Box((0.30, 0.10, 0.26)), origin=Origin(xyz=(0.04, -0.23, 0.41)))
    yaw_head.visual(Box((0.16, 0.10, 0.62)), origin=Origin(xyz=(0.14, 0.23, 0.66)))
    yaw_head.visual(Box((0.16, 0.10, 0.62)), origin=Origin(xyz=(0.14, -0.23, 0.66)))
    yaw_head.visual(Box((0.28, 0.40, 0.08)), origin=Origin(xyz=(0.00, 0.0, 0.63)))
    yaw_head.visual(
        Cylinder(radius=0.08, length=0.10),
        origin=Origin(xyz=(0.14, 0.23, 0.84), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    yaw_head.visual(
        Cylinder(radius=0.08, length=0.10),
        origin=Origin(xyz=(0.14, -0.23, 0.84), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    yaw_head.visual(Cylinder(radius=0.03, length=0.22), origin=Origin(xyz=(-0.08, 0.0, 0.59)))
    yaw_head.visual(Sphere(radius=0.055), origin=Origin(xyz=(-0.08, 0.0, 0.74)))
    yaw_head.visual(Box((0.14, 0.16, 0.10)), origin=Origin(xyz=(-0.20, -0.12, 0.53)))
    yaw_head.inertial = Inertial.from_geometry(
        Box((0.78, 0.60, 1.08)),
        mass=210.0,
        origin=Origin(xyz=(0.00, 0.0, 0.54)),
    )

    dish_shell = _dish_shell_mesh()
    feed_arm_top = _feed_arm_mesh(
        "feed_arm_top.obj",
        [
            (0.08, -0.23, 0.07),
            (0.20, -0.23, 0.16),
            (0.45, -0.23, 0.12),
            (0.69, -0.23, 0.03),
        ],
    )
    feed_arm_left = _feed_arm_mesh(
        "feed_arm_left.obj",
        [
            (0.08, -0.31, -0.05),
            (0.20, -0.31, -0.10),
            (0.44, -0.28, -0.08),
            (0.69, -0.27, -0.03),
        ],
    )
    feed_arm_right = _feed_arm_mesh(
        "feed_arm_right.obj",
        [
            (0.08, -0.15, -0.05),
            (0.20, -0.15, -0.10),
            (0.44, -0.18, -0.08),
            (0.69, -0.19, -0.03),
        ],
    )

    dish = model.part("dish_assembly")
    dish.visual(
        Cylinder(radius=0.03, length=0.46),
        origin=Origin(xyz=(0.0, -0.23, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    dish.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(0.06, -0.23, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
    )
    dish.visual(
        dish_shell,
        origin=Origin(xyz=(0.09, -0.23, 0.12), rpy=(0.0, pi / 2.0, 0.0)),
    )
    dish.visual(feed_arm_top, origin=Origin(xyz=(0.0, 0.0, 0.12)))
    dish.visual(feed_arm_left, origin=Origin(xyz=(0.0, 0.0, 0.12)))
    dish.visual(feed_arm_right, origin=Origin(xyz=(0.0, 0.0, 0.12)))
    dish.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(xyz=(0.74, -0.23, 0.12), rpy=(0.0, pi / 2.0, 0.0)),
    )
    dish.visual(
        Cylinder(radius=0.032, length=0.16),
        origin=Origin(xyz=(0.86, -0.23, 0.12), rpy=(0.0, pi / 2.0, 0.0)),
    )
    dish.visual(Sphere(radius=0.05), origin=Origin(xyz=(0.96, -0.23, 0.12)))
    dish.visual(Box((0.08, 0.05, 0.10)), origin=Origin(xyz=(0.70, -0.23, 0.05)))
    dish.inertial = Inertial.from_geometry(
        Box((1.05, 0.48, 1.62)),
        mass=95.0,
        origin=Origin(xyz=(0.50, -0.23, 0.12)),
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent="pedestal",
        child="yaw_head",
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.6, lower=-pi, upper=pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent="yaw_head",
        child="dish_assembly",
        origin=Origin(xyz=(0.14, 0.23, 0.84)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.8, lower=-0.25, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "yaw_head",
        "dish_assembly",
        reason="dish trunnion axle is intentionally captured inside the cradle bearing housings",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("yaw_head", "pedestal", axes="xy", max_dist=0.12)
    ctx.expect_aabb_overlap("yaw_head", "pedestal", axes="xy", min_overlap=0.40)
    ctx.expect_aabb_gap("yaw_head", "pedestal", axis="z", max_gap=0.008, max_penetration=0.0)
    ctx.expect_origin_distance("dish_assembly", "pedestal", axes="xy", max_dist=0.36)
    ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.44)
    ctx.expect_origin_distance("dish_assembly", "yaw_head", axes="xy", max_dist=0.30)
    ctx.expect_aabb_overlap("dish_assembly", "yaw_head", axes="xy", min_overlap=0.22)
    ctx.expect_joint_motion_axis(
        "elevation", "dish_assembly", world_axis="z", direction="positive", min_delta=0.08
    )

    with ctx.pose(elevation=-0.2):
        ctx.expect_origin_distance("dish_assembly", "pedestal", axes="xy", max_dist=0.38)
        ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.36)
        ctx.expect_origin_distance("dish_assembly", "yaw_head", axes="xy", max_dist=0.34)

    with ctx.pose(elevation=1.0):
        ctx.expect_origin_distance("dish_assembly", "pedestal", axes="xy", max_dist=0.60)
        ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.18)
        ctx.expect_origin_distance("dish_assembly", "yaw_head", axes="xy", max_dist=0.44)

    with ctx.pose(azimuth=pi / 2.0):
        ctx.expect_origin_distance("yaw_head", "pedestal", axes="xy", max_dist=0.12)
        ctx.expect_aabb_overlap("yaw_head", "pedestal", axes="xy", min_overlap=0.40)
        ctx.expect_origin_distance("dish_assembly", "pedestal", axes="xy", max_dist=0.36)
        ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.44)

    with ctx.pose(azimuth=pi / 2.0, elevation=1.0):
        ctx.expect_origin_distance("dish_assembly", "pedestal", axes="xy", max_dist=0.60)
        ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.18)
        ctx.expect_origin_distance("dish_assembly", "yaw_head", axes="xy", max_dist=0.44)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
