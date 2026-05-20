from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
HALF_PI = math.pi / 2.0


def _build_handle_mesh():
    handle_geom = tube_from_spline_points(
        [
            (-0.065, 0.106, 0.154),
            (-0.060, 0.136, 0.198),
            (0.000, 0.158, 0.226),
            (0.060, 0.136, 0.198),
            (0.065, 0.106, 0.154),
        ],
        radius=0.0075,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(handle_geom, ASSETS.mesh_path("lamp_handle.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight", assets=ASSETS)

    stand = model.part("stand")
    stand.visual(
        Box((0.420, 0.300, 0.028)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        name="base_plate",
    )
    stand.visual(
        Box((0.240, 0.160, 0.042)),
        origin=Origin(xyz=(0.000, 0.000, 0.049)),
        name="base_plinth",
    )
    stand.visual(
        Box((0.180, 0.100, 0.090)),
        origin=Origin(xyz=(0.000, -0.070, 0.115)),
        name="ballast_box",
    )
    stand.visual(
        Cylinder(radius=0.032, length=0.620),
        origin=Origin(xyz=(0.000, 0.000, 0.380)),
        name="lower_riser",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.710)),
        name="lock_collar",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.130),
        origin=Origin(xyz=(0.000, 0.000, 0.795)),
        name="upper_riser",
    )
    stand.visual(
        Box((0.100, 0.080, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.871)),
        name="pan_head",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            stand.visual(
                Box((0.045, 0.045, 0.008)),
                origin=Origin(xyz=(0.155 * x_sign, 0.105 * y_sign, -0.004)),
                name=f"foot_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'b'}",
            )
    stand.inertial = Inertial.from_geometry(
        Box((0.420, 0.300, 0.920)),
        mass=18.0,
        origin=Origin(xyz=(0.000, 0.000, 0.460)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.360, 0.080, 0.012)),
        origin=Origin(xyz=(0.000, 0.050, -0.006)),
        name="top_bridge",
    )
    yoke.visual(
        Box((0.120, 0.100, 0.022)),
        origin=Origin(xyz=(0.000, 0.018, -0.011)),
        name="pan_crown",
    )
    yoke.visual(
        Box((0.090, 0.060, 0.052)),
        origin=Origin(xyz=(0.000, 0.082, -0.132)),
        name="center_trunnion_block",
    )
    for x_sign in (-1.0, 1.0):
        yoke.visual(
            Box((0.024, 0.050, 0.260)),
            origin=Origin(xyz=(0.170 * x_sign, 0.082, -0.140)),
            name=f"side_arm_{'r' if x_sign > 0 else 'l'}",
        )
        yoke.visual(
            Cylinder(radius=0.032, length=0.016),
            origin=Origin(
                xyz=(0.154 * x_sign, 0.110, -0.125),
                rpy=(0.000, HALF_PI, 0.000),
            ),
            name=f"tilt_collar_{'r' if x_sign > 0 else 'l'}",
        )
    yoke.visual(
        Box((0.342, 0.028, 0.028)),
        origin=Origin(xyz=(0.000, 0.050, -0.145)),
        name="tilt_backbrace",
    )
    yoke.visual(
        Box((0.342, 0.018, 0.022)),
        origin=Origin(xyz=(0.000, 0.068, -0.235)),
        name="lower_stiffener",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.360, 0.100, 0.270)),
        mass=3.4,
        origin=Origin(xyz=(0.000, 0.050, -0.135)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.015, length=0.230),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, HALF_PI, 0.000)),
        name="tilt_axle",
    )
    for x_sign in (-1.0, 1.0):
        lamp.visual(
            Cylinder(radius=0.036, length=0.024),
            origin=Origin(
                xyz=(0.127 * x_sign, 0.000, 0.000),
                rpy=(0.000, HALF_PI, 0.000),
            ),
            name=f"tilt_knob_{'r' if x_sign > 0 else 'l'}",
        )
    lamp.visual(
        Box((0.200, 0.080, 0.180)),
        origin=Origin(xyz=(0.000, 0.100, 0.060)),
        name="rear_body",
    )
    lamp.visual(
        Box((0.150, 0.090, 0.120)),
        origin=Origin(xyz=(0.000, 0.050, 0.045)),
        name="trunnion_core",
    )
    lamp.visual(
        Cylinder(radius=0.130, length=0.200),
        origin=Origin(xyz=(0.000, 0.190, 0.080), rpy=(HALF_PI, 0.000, 0.000)),
        name="main_drum",
    )
    lamp.visual(
        Cylinder(radius=0.145, length=0.030),
        origin=Origin(xyz=(0.000, 0.300, 0.080), rpy=(HALF_PI, 0.000, 0.000)),
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.102, length=0.060),
        origin=Origin(xyz=(0.000, 0.255, 0.080), rpy=(HALF_PI, 0.000, 0.000)),
        name="lens_tube",
    )
    lamp.visual(
        Cylinder(radius=0.122, length=0.008),
        origin=Origin(xyz=(0.000, 0.287, 0.080), rpy=(HALF_PI, 0.000, 0.000)),
        name="front_lens",
    )
    lamp.visual(
        Box((0.180, 0.110, 0.080)),
        origin=Origin(xyz=(0.000, 0.165, 0.165)),
        name="driver_housing",
    )
    lamp.visual(
        Box((0.180, 0.090, 0.050)),
        origin=Origin(xyz=(0.000, 0.155, -0.025)),
        name="lower_chin",
    )
    for idx, y_pos in enumerate((0.105, 0.130, 0.155, 0.180, 0.205), start=1):
        lamp.visual(
            Box((0.150, 0.014, 0.060)),
            origin=Origin(xyz=(0.000, y_pos, 0.222)),
            name=f"cooling_fin_{idx}",
        )
    for x_sign in (-1.0, 1.0):
        lamp.visual(
            Box((0.022, 0.140, 0.130)),
            origin=Origin(xyz=(0.100 * x_sign, 0.090, 0.075)),
            name=f"side_cheek_{'r' if x_sign > 0 else 'l'}",
        )
    for idx, z_pos in enumerate((-0.050, 0.176), start=1):
        lamp.visual(
            Box((0.055, 0.012, 0.028)),
            origin=Origin(xyz=(0.000, 0.305, z_pos)),
            name=f"accessory_tab_{idx}",
        )
    lamp.visual(_build_handle_mesh(), origin=Origin(), name="carry_handle")
    lamp.inertial = Inertial.from_geometry(
        Box((0.300, 0.340, 0.320)),
        mass=6.8,
        origin=Origin(xyz=(0.000, 0.180, 0.085)),
    )

    model.articulation(
        "stand_pan",
        ArticulationType.REVOLUTE,
        parent="stand",
        child="yoke",
        origin=Origin(xyz=(0.000, 0.000, 0.890)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-1.90,
            upper=1.90,
        ),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent="yoke",
        child="lamp",
        origin=Origin(xyz=(0.000, 0.110, -0.125)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.8,
            lower=-1.05,
            upper=0.65,
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
        "stand",
        "yoke",
        reason="Pan bearing surfaces sit in a tight turntable stack and generated collision hulls can conservatively touch.",
    )
    ctx.allow_overlap(
        "yoke",
        "lamp",
        reason="The yoke collars and tilt handwheels are intentionally close-fitting around the trunnion axis.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("yoke", "stand", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
    ctx.expect_origin_distance("lamp", "yoke", axes="xy", max_dist=0.20)
    ctx.expect_aabb_overlap("yoke", "stand", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_overlap("lamp", "yoke", axes="xy", min_overlap=0.03)
    ctx.expect_joint_motion_axis(
        "yoke_tilt",
        "lamp",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )

    with ctx.pose(stand_pan=1.60):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_overlap("yoke", "stand", axes="xy", min_overlap=0.06)

    with ctx.pose(stand_pan=-1.60):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_overlap("yoke", "stand", axes="xy", min_overlap=0.06)

    with ctx.pose({"stand_pan": 1.85, "yoke_tilt": 0.55}):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("lamp", "yoke", axes="xy", min_overlap=0.03)

    with ctx.pose({"stand_pan": -1.85, "yoke_tilt": 0.55}):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("lamp", "yoke", axes="xy", min_overlap=0.03)

    with ctx.pose(yoke_tilt=0.60):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
        ctx.expect_origin_distance("lamp", "yoke", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("lamp", "yoke", axes="xy", min_overlap=0.03)

    with ctx.pose(yoke_tilt=-0.95):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.24)
        ctx.expect_origin_distance("lamp", "yoke", axes="xy", max_dist=0.24)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("lamp", "yoke", axes="xy", min_overlap=0.03)

    with ctx.pose({"stand_pan": 1.20, "yoke_tilt": 0.55}):
        ctx.expect_origin_distance("lamp", "stand", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("lamp", "stand", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("lamp", "yoke", axes="xy", min_overlap=0.03)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
