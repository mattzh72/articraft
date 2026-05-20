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
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

WHEEL_RADIUS = 0.447
AXLE_HEIGHT = 0.82
WHEEL_WIDTH = 0.36
LEFT_JOURNAL_WORLD_Y = -0.209
RIGHT_JOURNAL_WORLD_Y = 0.209
WHEEL_JOINT_Y = -0.212
WHEEL_CENTER_LOCAL_Y = -WHEEL_JOINT_Y
WHEEL_JOINT_NAME = "wheel_rotation"
FLUME_JOINT_NAME = "frame_to_flume"


def _add_box(part, size, xyz, *, rpy=(0.0, 0.0, 0.0), name=None):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), name=name)


def _add_cylinder(part, radius, length, xyz, *, rpy=(0.0, 0.0, 0.0), name=None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel", assets=ASSETS)

    frame = model.part("frame")

    _add_box(frame, (0.82, 0.82, 0.16), (0.0, 0.0, 0.08), name="foundation")
    _add_box(frame, (0.24, 0.14, 0.30), (0.0, -0.28, 0.31), name="left_pier")
    _add_box(frame, (0.24, 0.14, 0.30), (0.0, 0.28, 0.31), name="right_pier")

    _add_box(frame, (0.10, 0.66, 0.10), (-0.20, 0.0, 0.21), name="front_sill")
    _add_box(frame, (0.10, 0.66, 0.10), (0.20, 0.0, 0.21), name="rear_sill")

    for side_y in (-0.28, 0.28):
        for post_x in (-0.20, 0.20):
            _add_box(
                frame,
                (0.06, 0.05, 0.72),
                (post_x, side_y, 0.52),
                name=f"post_{'l' if side_y < 0 else 'r'}_{'f' if post_x < 0 else 'b'}",
            )

        _add_box(frame, (0.50, 0.06, 0.06), (0.0, side_y, 0.88), name=f"cap_{side_y:.2f}")
        _add_box(frame, (0.12, 0.08, 0.10), (0.0, -0.254 if side_y < 0 else 0.254, 0.82))

        _add_box(
            frame,
            (0.05, 0.05, 0.58),
            (-0.11, side_y, 0.58),
            rpy=(0.0, -0.32 if side_y < 0 else -0.32, 0.0),
        )
        _add_box(
            frame,
            (0.05, 0.05, 0.58),
            (0.11, side_y, 0.58),
            rpy=(0.0, 0.32 if side_y < 0 else 0.32, 0.0),
        )

        _add_box(frame, (0.05, 0.05, 0.42), (-0.20, -0.26 if side_y < 0 else 0.26, 1.09))

    _add_box(frame, (0.08, 0.56, 0.06), (-0.20, 0.0, 1.30), name="flume_support_beam")

    frame.inertial = Inertial.from_geometry(
        Box((0.90, 0.86, 1.40)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
    )

    flume = model.part("flume")
    _add_box(flume, (0.26, 0.56, 0.04), (-0.09, 0.0, 0.022), name="floor")
    _add_box(flume, (0.26, 0.02, 0.10), (-0.09, -0.27, 0.092), name="left_wall")
    _add_box(flume, (0.26, 0.02, 0.10), (-0.09, 0.27, 0.092), name="right_wall")
    _add_box(flume, (0.04, 0.56, 0.10), (-0.20, 0.0, 0.092), name="headboard")
    _add_box(flume, (0.05, 0.56, 0.05), (0.03, 0.0, 0.055), name="spill_lip")
    flume.inertial = Inertial.from_geometry(
        Box((0.32, 0.60, 0.14)),
        mass=28.0,
        origin=Origin(xyz=(-0.09, 0.0, 0.07)),
    )

    wheel = model.part("wheel")

    outer_ring = TorusGeometry(
        radius=0.43,
        tube=0.017,
        radial_segments=18,
        tubular_segments=56,
    )
    outer_ring.rotate_x(math.pi / 2.0)
    outer_ring_mesh = mesh_from_geometry(outer_ring, ASSETS.mesh_dir / "waterwheel_outer_ring.obj")

    inner_ring = TorusGeometry(
        radius=0.245,
        tube=0.010,
        radial_segments=16,
        tubular_segments=48,
    )
    inner_ring.rotate_x(math.pi / 2.0)
    inner_ring_mesh = mesh_from_geometry(inner_ring, ASSETS.mesh_dir / "waterwheel_inner_ring.obj")

    left_shroud_y = WHEEL_CENTER_LOCAL_Y - (WHEEL_WIDTH / 2.0)
    right_shroud_y = WHEEL_CENTER_LOCAL_Y + (WHEEL_WIDTH / 2.0)

    wheel.visual(
        outer_ring_mesh, origin=Origin(xyz=(0.0, left_shroud_y, 0.0)), name="left_outer_ring"
    )
    wheel.visual(
        outer_ring_mesh, origin=Origin(xyz=(0.0, right_shroud_y, 0.0)), name="right_outer_ring"
    )
    wheel.visual(
        inner_ring_mesh, origin=Origin(xyz=(0.0, left_shroud_y, 0.0)), name="left_inner_ring"
    )
    wheel.visual(
        inner_ring_mesh, origin=Origin(xyz=(0.0, right_shroud_y, 0.0)), name="right_inner_ring"
    )

    _add_box(wheel, (0.10, 0.362, 0.10), (0.0, WHEEL_CENTER_LOCAL_Y, 0.0), name="timber_axle")
    _add_cylinder(
        wheel,
        0.028,
        0.032,
        (0.0, 0.016, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        name="left_journal",
    )
    _add_cylinder(
        wheel,
        0.028,
        0.032,
        (0.0, 0.408, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        name="right_journal",
    )
    _add_cylinder(
        wheel,
        0.11,
        0.16,
        (0.0, WHEEL_CENTER_LOCAL_Y, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        name="hub_drum",
    )
    _add_cylinder(
        wheel,
        0.13,
        0.024,
        (0.0, WHEEL_CENTER_LOCAL_Y - 0.09, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        name="left_hub_collar",
    )
    _add_cylinder(
        wheel,
        0.13,
        0.024,
        (0.0, WHEEL_CENTER_LOCAL_Y + 0.09, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        name="right_hub_collar",
    )

    spoke_count = 16
    spoke_inner_radius = 0.105
    spoke_outer_radius = 0.413
    spoke_length = spoke_outer_radius - spoke_inner_radius
    spoke_mid_radius = (spoke_outer_radius + spoke_inner_radius) / 2.0

    bucket_count = 16
    bucket_floor_radius = 0.385
    bucket_divider_radius = 0.335
    bucket_width = 0.362
    bucket_span = (2.0 * math.pi) / bucket_count

    for idx in range(spoke_count):
        angle = (2.0 * math.pi * idx) / spoke_count
        x = spoke_mid_radius * math.sin(angle)
        z = spoke_mid_radius * math.cos(angle)
        _add_cylinder(
            wheel,
            0.010,
            spoke_length,
            (x, left_shroud_y, z),
            rpy=(0.0, angle, 0.0),
            name=f"left_spoke_{idx}",
        )
        _add_cylinder(
            wheel,
            0.010,
            spoke_length,
            (x, right_shroud_y, z),
            rpy=(0.0, angle, 0.0),
            name=f"right_spoke_{idx}",
        )

    for idx in range(bucket_count):
        angle = idx * bucket_span
        floor_x = bucket_floor_radius * math.sin(angle)
        floor_z = bucket_floor_radius * math.cos(angle)
        divider_x = bucket_divider_radius * math.sin(angle - 0.18)
        divider_z = bucket_divider_radius * math.cos(angle - 0.18)

        _add_box(
            wheel,
            (0.052, bucket_width, 0.135),
            (floor_x, WHEEL_CENTER_LOCAL_Y, floor_z),
            rpy=(0.0, angle + (math.pi / 2.0), 0.0),
            name=f"bucket_floor_{idx}",
        )
        _add_box(
            wheel,
            (0.020, bucket_width, 0.125),
            (divider_x, WHEEL_CENTER_LOCAL_Y, divider_z),
            rpy=(0.0, angle - 0.18, 0.0),
            name=f"bucket_divider_{idx}",
        )

    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=0.43),
        mass=72.0,
        origin=Origin(
            xyz=(0.0, WHEEL_CENTER_LOCAL_Y, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        FLUME_JOINT_NAME,
        ArticulationType.FIXED,
        parent="frame",
        child="flume",
        origin=Origin(xyz=(-0.20, 0.0, 1.332)),
        axis=(0.0, 0.0, 1.0),
    )
    model.articulation(
        WHEEL_JOINT_NAME,
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="wheel",
        origin=Origin(xyz=(0.0, WHEEL_JOINT_Y, AXLE_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.8),
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
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_gap("flume", "frame", axis="z", max_gap=0.02, max_penetration=0.0)
    ctx.expect_aabb_overlap("flume", "frame", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_overlap("flume", "wheel", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_gap("flume", "wheel", axis="z", max_gap=0.18, max_penetration=0.07)

    for angle in (0.0, math.pi / 4.0, math.pi / 2.0, math.pi, 1.5 * math.pi):
        with ctx.pose(**{WHEEL_JOINT_NAME: angle}):
            ctx.expect_aabb_overlap("wheel", "frame", axes="xy", min_overlap=0.22)
            ctx.expect_aabb_overlap("flume", "wheel", axes="xy", min_overlap=0.10)
            ctx.expect_aabb_gap("flume", "wheel", axis="z", max_gap=0.18, max_penetration=0.07)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
