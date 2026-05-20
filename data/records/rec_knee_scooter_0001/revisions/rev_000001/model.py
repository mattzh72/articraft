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

WHEEL_RADIUS = 0.10
WHEEL_WIDTH = 0.016


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def _max_delta(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return max(abs(a[i] - b[i]) for i in range(3))


def _tube(points: list[tuple[float, float, float]], radius: float, radial_segments: int = 18):
    if len(points) > 2:
        return tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=radial_segments,
            cap_ends=True,
        )
    return wire_from_points(
        points,
        radius=radius,
        radial_segments=radial_segments,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
        corner_radius=0.0,
    )


def _build_frame_mesh():
    paths = [
        (
            [
                (0.10, -0.12, 0.10),
                (0.18, -0.12, 0.12),
                (0.28, -0.10, 0.18),
                (0.38, -0.08, 0.28),
                (0.46, -0.05, 0.40),
            ],
            0.018,
        ),
        (
            [
                (0.10, 0.12, 0.10),
                (0.18, 0.12, 0.12),
                (0.28, 0.10, 0.18),
                (0.38, 0.08, 0.28),
                (0.46, 0.05, 0.40),
            ],
            0.018,
        ),
        ([(0.10, -0.12, 0.10), (0.10, 0.12, 0.10)], 0.016),
        ([(0.24, -0.10, 0.18), (0.24, 0.10, 0.18)], 0.014),
        ([(0.48, -0.05, 0.40), (0.48, 0.05, 0.40)], 0.014),
        ([(0.24, 0.00, 0.18), (0.24, 0.00, 0.50)], 0.018),
        ([(0.10, -0.12, 0.10), (0.10, -0.188, 0.10)], 0.010),
        ([(0.10, 0.12, 0.10), (0.10, 0.188, 0.10)], 0.010),
        ([(0.24, 0.00, 0.42), (0.36, 0.00, 0.42), (0.48, 0.00, 0.40)], 0.012),
        ([(0.20, -0.08, 0.20), (0.23, -0.06, 0.34), (0.29, -0.05, 0.50)], 0.012),
        ([(0.20, 0.08, 0.20), (0.23, 0.06, 0.34), (0.29, 0.05, 0.50)], 0.012),
    ]
    geometry = _tube(paths[0][0], paths[0][1], radial_segments=20)
    for points, radius in paths[1:]:
        geometry.merge(_tube(points, radius))
    return mesh_from_geometry(geometry, ASSETS.mesh_path("knee_scooter_frame.obj"))


def _build_fork_mesh():
    paths = [
        (
            [
                (0.01, -0.05, 0.03),
                (0.06, -0.08, -0.10),
                (0.12, -0.10, -0.24),
                (0.18, -0.111, -0.34),
            ],
            0.010,
        ),
        (
            [
                (0.01, 0.05, 0.03),
                (0.06, 0.08, -0.10),
                (0.12, 0.10, -0.24),
                (0.18, 0.111, -0.34),
            ],
            0.010,
        ),
        (
            [
                (0.00, 0.00, 0.02),
                (0.05, 0.00, -0.10),
                (0.12, 0.00, -0.24),
                (0.18, 0.00, -0.34),
            ],
            0.011,
        ),
    ]
    geometry = _tube(paths[0][0], paths[0][1])
    for points, radius in paths[1:]:
        geometry.merge(_tube(points, radius))
    return mesh_from_geometry(geometry, ASSETS.mesh_path("knee_scooter_fork.obj"))


def _add_wheel_visuals(part) -> None:
    wheel_pose = Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0))
    side_offset = 0.006

    part.visual(Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH), origin=wheel_pose)
    part.visual(Cylinder(radius=0.080, length=0.015), origin=wheel_pose)
    part.visual(Cylinder(radius=0.030, length=0.012), origin=wheel_pose)
    part.visual(
        Cylinder(radius=0.086, length=0.003),
        origin=Origin(xyz=(0.0, -side_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    part.visual(
        Cylinder(radius=0.086, length=0.003),
        origin=Origin(xyz=(0.0, side_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    part.visual(Cylinder(radius=0.018, length=0.007), origin=wheel_pose)
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.75,
        origin=wheel_pose,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter", assets=ASSETS)

    base_frame = model.part("base_frame")
    base_frame.visual(_build_frame_mesh())
    base_frame.visual(Cylinder(radius=0.028, length=0.14), origin=Origin(xyz=(0.48, 0.0, 0.44)))
    base_frame.visual(Box((0.055, 0.095, 0.050)), origin=Origin(xyz=(0.24, 0.0, 0.476)))
    base_frame.visual(Box((0.26, 0.15, 0.008)), origin=Origin(xyz=(0.27, 0.0, 0.507)))
    base_frame.visual(Box((0.22, 0.12, 0.042)), origin=Origin(xyz=(0.27, 0.0, 0.531)))
    base_frame.visual(Box((0.040, 0.110, 0.014)), origin=Origin(xyz=(0.37, 0.0, 0.546)))
    base_frame.visual(
        Cylinder(radius=0.012, length=0.110),
        origin=Origin(xyz=(0.38, 0.0, 0.535), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    base_frame.visual(
        Box((0.040, 0.016, 0.012)),
        origin=Origin(xyz=(0.105, 0.095, 0.118), rpy=(0.0, -0.30, 0.0)),
    )
    base_frame.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.10, -0.198, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
    )
    base_frame.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.10, 0.198, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.44, 0.64)),
        mass=6.8,
        origin=Origin(xyz=(0.29, 0.0, 0.32)),
    )

    steering = model.part("steering_assembly")
    steering.visual(Cylinder(radius=0.022, length=0.036), origin=Origin(xyz=(0.0, 0.0, 0.012)))
    steering.visual(Cylinder(radius=0.020, length=0.56), origin=Origin(xyz=(0.0, 0.0, 0.30)))
    steering.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.01, 0.0, 0.03), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    steering.visual(
        Cylinder(radius=0.009, length=0.16),
        origin=Origin(xyz=(0.10, 0.0, -0.18), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    steering.visual(
        Cylinder(radius=0.010, length=0.264),
        origin=Origin(xyz=(0.18, 0.0, -0.34), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    steering.visual(_build_fork_mesh())
    steering.visual(Box((0.035, 0.070, 0.040)), origin=Origin(xyz=(0.005, 0.0, 0.525)))
    steering.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(xyz=(0.035, 0.0, 0.53), rpy=(0.0, pi / 2.0, 0.0)),
    )
    steering.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(0.07, 0.0, 0.53), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    steering.visual(
        Cylinder(radius=0.017, length=0.10),
        origin=Origin(xyz=(0.07, -0.17, 0.53), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    steering.visual(
        Cylinder(radius=0.017, length=0.10),
        origin=Origin(xyz=(0.07, 0.17, 0.53), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    steering.visual(
        Box((0.10, 0.010, 0.012)),
        origin=Origin(xyz=(0.10, -0.12, 0.495), rpy=(0.0, -0.58, -0.10)),
    )
    steering.visual(
        Box((0.10, 0.010, 0.012)),
        origin=Origin(xyz=(0.10, 0.12, 0.495), rpy=(0.0, -0.58, 0.10)),
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.30, 0.42, 0.96)),
        mass=2.6,
        origin=Origin(xyz=(0.08, 0.0, 0.14)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(rear_left_wheel)

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(rear_right_wheel)

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel)

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel)

    model.articulation(
        "steering_joint",
        ArticulationType.REVOLUTE,
        parent="base_frame",
        child="steering_assembly",
        origin=Origin(xyz=(0.48, 0.0, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "rear_left_roll",
        ArticulationType.CONTINUOUS,
        parent="base_frame",
        child="rear_left_wheel",
        origin=Origin(xyz=(0.10, -0.214, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_roll",
        ArticulationType.CONTINUOUS,
        parent="base_frame",
        child="rear_right_wheel",
        origin=Origin(xyz=(0.10, 0.214, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "front_left_roll",
        ArticulationType.CONTINUOUS,
        parent="steering_assembly",
        child="front_left_wheel",
        origin=Origin(xyz=(0.18, -0.132, -0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "front_right_roll",
        ArticulationType.CONTINUOUS,
        parent="steering_assembly",
        child="front_right_wheel",
        origin=Origin(xyz=(0.18, 0.132, -0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "base_frame",
        "steering_assembly",
        reason="Steering stem intentionally nests into the headset tube.",
    )
    ctx.allow_overlap(
        "base_frame",
        "rear_left_wheel",
        reason="The rear wheel hub sits tightly beside the tubular rear outrigger and generated hulls overestimate contact.",
    )
    ctx.allow_overlap(
        "base_frame",
        "rear_right_wheel",
        reason="The rear wheel hub sits tightly beside the tubular rear outrigger and generated hulls overestimate contact.",
    )
    ctx.allow_overlap(
        "front_left_wheel",
        "steering_assembly",
        reason="The open fork and axle are represented by tubular visuals whose generated convex hulls conservatively span the wheel pocket.",
    )
    ctx.allow_overlap(
        "front_right_wheel",
        "steering_assembly",
        reason="The open fork and axle are represented by tubular visuals whose generated convex hulls conservatively span the wheel pocket.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    _require(len(object_model.parts) == 6, "Expected six parts for the scooter assembly.")
    _require(
        len(object_model.articulations) == 5, "Expected one steering joint and four rolling joints."
    )

    steering_joint = object_model.get_articulation("steering_joint")
    _require(
        steering_joint.articulation_type == ArticulationType.REVOLUTE,
        "The steering column must use a revolute articulation.",
    )
    _require(steering_joint.motion_limits is not None, "The steering joint needs explicit limits.")
    _require(
        steering_joint.motion_limits.lower <= -0.70,
        "Steering should turn meaningfully to the right.",
    )
    _require(
        steering_joint.motion_limits.upper >= 0.70, "Steering should turn meaningfully to the left."
    )

    for wheel_joint_name in (
        "rear_left_roll",
        "rear_right_roll",
        "front_left_roll",
        "front_right_roll",
    ):
        _require(
            object_model.get_articulation(wheel_joint_name).articulation_type
            == ArticulationType.CONTINUOUS,
            f"{wheel_joint_name} should be a continuous rolling joint.",
        )

    ctx.expect_aabb_overlap("steering_assembly", "base_frame", axes="xy", min_overlap=0.01)
    ctx.expect_joint_motion_axis(
        "steering_joint",
        "front_left_wheel",
        world_axis="y",
        direction="positive",
        min_delta=0.12,
    )
    ctx.expect_joint_motion_axis(
        "steering_joint",
        "front_right_wheel",
        world_axis="y",
        direction="positive",
        min_delta=0.12,
    )
    ctx.expect_origin_distance("front_left_wheel", "front_right_wheel", axes="xy", max_dist=0.30)
    ctx.expect_origin_distance("rear_left_wheel", "rear_right_wheel", axes="xy", max_dist=0.45)

    wheel_names = (
        "rear_left_wheel",
        "rear_right_wheel",
        "front_left_wheel",
        "front_right_wheel",
    )
    rest_positions = {name: ctx.part_world_position(name) for name in wheel_names}

    rear_track = rest_positions["rear_right_wheel"][1] - rest_positions["rear_left_wheel"][1]
    front_track = rest_positions["front_right_wheel"][1] - rest_positions["front_left_wheel"][1]

    _require(
        rest_positions["rear_left_wheel"][1] < -0.18,
        "Rear left wheel should sit outboard of the frame.",
    )
    _require(
        rest_positions["rear_right_wheel"][1] > 0.18,
        "Rear right wheel should sit outboard of the frame.",
    )
    _require(
        rest_positions["front_left_wheel"][1] < -0.10,
        "Front left wheel should be on the left side at rest.",
    )
    _require(
        rest_positions["front_right_wheel"][1] > 0.10,
        "Front right wheel should be on the right side at rest.",
    )
    _require(
        abs(rest_positions["rear_left_wheel"][1] + rest_positions["rear_right_wheel"][1]) < 1e-6,
        "Rear wheels should be symmetric about the centerline.",
    )
    _require(
        abs(rest_positions["front_left_wheel"][1] + rest_positions["front_right_wheel"][1]) < 1e-6,
        "Front wheels should be symmetric about the centerline.",
    )
    _require(0.42 < rear_track < 0.44, "Rear wheel track should be broad for stability.")
    _require(
        0.25 < front_track < 0.28,
        "Front wheel track should be narrower than the rear for believable steering.",
    )
    _require(
        rear_track > front_track + 0.14,
        "Rear wheelbase should be wider than the steered front axle.",
    )
    _require(
        rest_positions["front_left_wheel"][0] > rest_positions["rear_left_wheel"][0] + 0.50,
        "Front axle should sit meaningfully ahead of the rear axle.",
    )
    _require(
        rest_positions["front_right_wheel"][0] > rest_positions["rear_right_wheel"][0] + 0.50,
        "Front axle should sit meaningfully ahead of the rear axle.",
    )
    _require(
        abs(rest_positions["front_left_wheel"][0] - rest_positions["front_right_wheel"][0]) < 1e-6,
        "Front wheel centers should align fore-aft at rest.",
    )
    _require(
        abs(rest_positions["rear_left_wheel"][0] - rest_positions["rear_right_wheel"][0]) < 1e-6,
        "Rear wheel centers should align fore-aft at rest.",
    )
    for wheel_name in wheel_names:
        _require(
            abs(rest_positions[wheel_name][2] - WHEEL_RADIUS) < 1e-6,
            f"{wheel_name} should sit at the correct axle height for ground contact.",
        )

    with ctx.pose(steering_joint=0.70):
        left_turn_positions = {name: ctx.part_world_position(name) for name in wheel_names}
        _require(
            left_turn_positions["front_left_wheel"][1]
            > rest_positions["front_left_wheel"][1] + 0.10,
            "Positive steering should move the left front wheel toward the left-turn path.",
        )
        _require(
            left_turn_positions["front_right_wheel"][1]
            > rest_positions["front_right_wheel"][1] + 0.08,
            "Positive steering should move the right front wheel toward the left-turn path.",
        )
        _require(
            left_turn_positions["front_left_wheel"][0]
            > rest_positions["rear_left_wheel"][0] + 0.42,
            "Even at steering lock, the left front wheel should remain ahead of the rear axle.",
        )
        _require(
            left_turn_positions["front_right_wheel"][0]
            > rest_positions["rear_right_wheel"][0] + 0.42,
            "Even at steering lock, the right front wheel should remain ahead of the rear axle.",
        )
        _require(
            _max_delta(left_turn_positions["rear_left_wheel"], rest_positions["rear_left_wheel"])
            < 1e-6,
            "Steering should not drag the rear left wheel sideways.",
        )
        _require(
            _max_delta(left_turn_positions["rear_right_wheel"], rest_positions["rear_right_wheel"])
            < 1e-6,
            "Steering should not drag the rear right wheel sideways.",
        )

    with ctx.pose(steering_joint=-0.70):
        right_turn_positions = {name: ctx.part_world_position(name) for name in wheel_names}
        _require(
            right_turn_positions["front_left_wheel"][1]
            < rest_positions["front_left_wheel"][1] - 0.08,
            "Negative steering should move the left front wheel toward the right-turn path.",
        )
        _require(
            right_turn_positions["front_right_wheel"][1]
            < rest_positions["front_right_wheel"][1] - 0.10,
            "Negative steering should move the right front wheel toward the right-turn path.",
        )
        _require(
            right_turn_positions["front_left_wheel"][0]
            > rest_positions["rear_left_wheel"][0] + 0.42,
            "The left front wheel should stay in front of the chassis during a right turn.",
        )
        _require(
            right_turn_positions["front_right_wheel"][0]
            > rest_positions["rear_right_wheel"][0] + 0.42,
            "The right front wheel should stay in front of the chassis during a right turn.",
        )
        _require(
            _max_delta(right_turn_positions["rear_left_wheel"], rest_positions["rear_left_wheel"])
            < 1e-6,
            "Rear wheel placement should remain fixed during steering.",
        )
        _require(
            _max_delta(right_turn_positions["rear_right_wheel"], rest_positions["rear_right_wheel"])
            < 1e-6,
            "Rear wheel placement should remain fixed during steering.",
        )

    with ctx.pose(
        front_left_roll=1.35, front_right_roll=-0.85, rear_left_roll=0.60, rear_right_roll=-1.20
    ):
        rolled_positions = {name: ctx.part_world_position(name) for name in wheel_names}
        for wheel_name in wheel_names:
            _require(
                _max_delta(rolled_positions[wheel_name], rest_positions[wheel_name]) < 1e-6,
                f"{wheel_name} should spin in place without translating.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
