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
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
PI = math.pi

WHEEL_RADIUS = 0.085
WHEEL_WIDTH = 0.024
WHEEL_HUB_WIDTH = 0.034
STEERING_X = 0.275
STEERING_Z = 0.110
FRONT_AXLE_LOCAL = (0.155, 0.0, -0.025)
REAR_AXLE = (-0.315, 0.0, WHEEL_RADIUS)


def _wheel_visuals(part) -> None:
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(PI / 2.0, 0.0, 0.0)),
    )
    part.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(rpy=(PI / 2.0, 0.0, 0.0)),
    )
    part.visual(
        Cylinder(radius=0.018, length=WHEEL_HUB_WIDTH),
        origin=Origin(rpy=(PI / 2.0, 0.0, 0.0)),
    )
    for angle in (0.0, PI / 3.0, 2.0 * PI / 3.0):
        part.visual(
            Box((0.006, 0.008, 0.146)),
            origin=Origin(rpy=(0.0, angle, 0.0)),
        )
        part.visual(
            Box((0.006, 0.008, 0.128)),
            origin=Origin(rpy=(0.0, angle + PI / 6.0, 0.0)),
        )


def _fork_blade_mesh(filename: str, lateral_offset: float):
    blade = tube_from_spline_points(
        [
            (0.028, lateral_offset, 0.046),
            (0.060, lateral_offset, 0.020),
            (0.110, lateral_offset, 0.000),
            (FRONT_AXLE_LOCAL[0], lateral_offset, FRONT_AXLE_LOCAL[2]),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=16,
    )
    return mesh_from_geometry(blade, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kick_scooter", assets=ASSETS)

    frame = model.part("frame")
    frame.visual(
        Box((0.410, 0.112, 0.022)),
        origin=Origin(xyz=(-0.035, 0.0, 0.066)),
    )
    frame.visual(
        Box((0.080, 0.048, 0.018)),
        origin=Origin(xyz=(0.170, 0.0, 0.069)),
    )
    frame.visual(
        Box((0.105, 0.036, 0.046)),
        origin=Origin(xyz=(0.210, 0.0, 0.108), rpy=(0.0, -0.68, 0.0)),
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.094),
        origin=Origin(xyz=(STEERING_X, 0.0, 0.155)),
    )
    frame.visual(
        Box((0.320, 0.010, 0.014)),
        origin=Origin(xyz=(-0.030, 0.049, 0.076)),
    )
    frame.visual(
        Box((0.320, 0.010, 0.014)),
        origin=Origin(xyz=(-0.030, -0.049, 0.076)),
    )
    frame.visual(
        Box((0.160, 0.012, 0.100)),
        origin=Origin(xyz=(-0.255, 0.032, 0.098), rpy=(0.0, 0.24, 0.0)),
    )
    frame.visual(
        Box((0.160, 0.012, 0.100)),
        origin=Origin(xyz=(-0.255, -0.032, 0.098), rpy=(0.0, 0.24, 0.0)),
    )
    frame.visual(
        Box((0.085, 0.085, 0.012)),
        origin=Origin(xyz=(-0.382, 0.0, 0.145)),
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.076),
        origin=Origin(xyz=REAR_AXLE, rpy=(PI / 2.0, 0.0, 0.0)),
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.620, 0.120, 0.130)),
        mass=3.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.090)),
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
    )
    front_assembly.visual(
        Cylinder(radius=0.022, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
    )
    front_assembly.visual(
        Box((0.052, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
    )
    front_assembly.visual(
        Cylinder(radius=0.013, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.665), rpy=(PI / 2.0, 0.0, 0.0)),
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, 0.176, 0.665), rpy=(PI / 2.0, 0.0, 0.0)),
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, -0.176, 0.665), rpy=(PI / 2.0, 0.0, 0.0)),
    )
    front_assembly.visual(
        Box((0.055, 0.060, 0.016)),
        origin=Origin(xyz=(0.050, 0.0, 0.034)),
    )
    front_assembly.visual(_fork_blade_mesh("front_fork_left.obj", 0.022))
    front_assembly.visual(_fork_blade_mesh("front_fork_right.obj", -0.022))
    front_assembly.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=FRONT_AXLE_LOCAL, rpy=(PI / 2.0, 0.0, 0.0)),
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.500, 0.460, 0.760)),
        mass=1.6,
        origin=Origin(xyz=(0.010, 0.0, 0.345)),
    )

    front_wheel = model.part("front_wheel")
    _wheel_visuals(front_wheel)
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.7,
        origin=Origin(rpy=(PI / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    _wheel_visuals(rear_wheel)
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.7,
        origin=Origin(rpy=(PI / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "steering_head",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="front_assembly",
        origin=Origin(xyz=(STEERING_X, 0.0, STEERING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=4.0,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "front_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent="front_assembly",
        child="front_wheel",
        origin=Origin(xyz=FRONT_AXLE_LOCAL),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=40.0,
        ),
    )
    model.articulation(
        "rear_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="rear_wheel",
        origin=Origin(xyz=REAR_AXLE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=40.0,
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
        "front_assembly",
        "front_wheel",
        reason="front axle passes through the wheel hub and bearings",
    )
    ctx.allow_overlap(
        "frame",
        "front_assembly",
        reason="the steering steerer passes through the headset bearings inside the head tube",
    )
    ctx.allow_overlap(
        "frame",
        "rear_wheel",
        reason="rear axle passes through the wheel hub and bearings",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("front_wheel", "front_assembly", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("rear_wheel", "frame", axes="xy", min_overlap=0.020)
    ctx.expect_origin_distance("front_wheel", "front_assembly", axes="xy", max_dist=0.170)
    ctx.expect_joint_motion_axis(
        "steering_head",
        "front_wheel",
        world_axis="y",
        direction="positive",
        min_delta=0.040,
    )

    rear_rest = ctx.part_world_position("rear_wheel")
    front_rest = ctx.part_world_position("front_wheel")
    stem_rest = ctx.part_world_position("front_assembly")

    assert rear_rest[0] < -0.290, "rear wheel should sit well behind the deck"
    assert abs(rear_rest[1]) < 1e-6, "rear wheel should be centered on the scooter"
    assert 0.080 <= rear_rest[2] <= 0.090, "rear axle height should match the wheel radius"
    assert front_rest[0] > 0.340, "front wheel should lead the steering axis and deck"
    assert abs(front_rest[1]) < 1e-6, "front wheel should be centered at neutral steering"
    assert abs(front_rest[2] - rear_rest[2]) < 1e-6, "both wheels should share axle height"
    assert front_rest[0] - rear_rest[0] > 0.640, (
        "wheelbase should read as a practical urban scooter"
    )
    assert 0.255 <= stem_rest[0] <= 0.290, "steering column should mount near the deck nose"
    assert abs(stem_rest[1]) < 1e-6, "steering column should be centered over the frame"
    assert 0.100 <= stem_rest[2] <= 0.120, "steering origin should sit at the headset height"

    with ctx.pose(steering_head=0.60):
        ctx.expect_aabb_overlap("front_wheel", "front_assembly", axes="xy", min_overlap=0.020)
        left_turn = ctx.part_world_position("front_wheel")
        assert left_turn[1] > 0.055, "positive steering should swing the front wheel to rider-left"
        assert left_turn[0] > 0.330, "the fork rake should keep the front wheel ahead of the deck"

    with ctx.pose(steering_head=-0.60):
        ctx.expect_aabb_overlap("front_wheel", "front_assembly", axes="xy", min_overlap=0.020)
        right_turn = ctx.part_world_position("front_wheel")
        assert right_turn[1] < -0.055, (
            "negative steering should swing the front wheel to rider-right"
        )
        assert right_turn[0] > 0.330, (
            "the front wheel should stay ahead of the frame at full opposite steer"
        )

    with ctx.pose(front_wheel_roll=1.70, rear_wheel_roll=-2.30):
        spun_front = ctx.part_world_position("front_wheel")
        spun_rear = ctx.part_world_position("rear_wheel")
        assert all(abs(a - b) < 1e-6 for a, b in zip(front_rest, spun_front)), (
            "front wheel rolling should rotate in place without changing axle position"
        )
        assert all(abs(a - b) < 1e-6 for a, b in zip(rear_rest, spun_rear)), (
            "rear wheel rolling should rotate in place without changing axle position"
        )
        ctx.expect_aabb_overlap("front_wheel", "front_assembly", axes="xy", min_overlap=0.020)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
