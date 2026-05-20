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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _tilted_box_center_from_top(
    top_y: float,
    top_z: float,
    length: float,
    angle: float,
) -> tuple[float, float]:
    return (
        top_y + 0.5 * length * math.sin(angle),
        top_z - 0.5 * length * math.cos(angle),
    )


def _point_along_tilt(
    top_y: float,
    top_z: float,
    distance_from_top: float,
    angle: float,
) -> tuple[float, float]:
    return (
        top_y + distance_from_top * math.sin(angle),
        top_z - distance_from_top * math.cos(angle),
    )


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a_frame_step_ladder", assets=ASSETS)

    front = model.part("front_frame")

    front_angle = -0.33
    front_top_y = -0.010
    front_top_z = -0.038
    front_rail_length = 1.48
    front_rail_x = 0.195
    front_rail_y, front_rail_z = _tilted_box_center_from_top(
        front_top_y,
        front_top_z,
        front_rail_length,
        front_angle,
    )

    for x in (-front_rail_x, front_rail_x):
        _add_box(
            front,
            (0.045, 0.020, front_rail_length),
            (x, front_rail_y, front_rail_z),
            (front_angle, 0.0, 0.0),
        )

    _add_box(front, (0.400, 0.124, 0.058), (0.0, -0.062, -0.046))
    _add_box(front, (0.300, 0.042, 0.014), (0.0, -0.114, -0.018))
    _add_box(front, (0.340, 0.052, 0.022), (0.0, -0.020, -0.120))
    _add_box(front, (0.100, 0.012, 0.030), (0.0, -0.006, -0.012))

    for x in (-0.165, 0.165):
        _add_box(front, (0.054, 0.012, 0.082), (x, -0.006, -0.024))

    for x in (-0.212, 0.212):
        _add_box(front, (0.018, 0.012, 0.180), (x, -0.012, -0.116))

    for step_distance in (0.34, 0.61, 0.88, 1.15):
        step_y, step_z = _point_along_tilt(
            front_top_y,
            front_top_z,
            step_distance,
            front_angle,
        )
        step_y -= 0.008
        _add_box(front, (0.356, 0.126, 0.034), (0.0, step_y, step_z))
        for tread_x in (-0.100, 0.0, 0.100):
            _add_box(front, (0.078, 0.112, 0.005), (tread_x, step_y, step_z + 0.015))

    for bracket_distance in (0.74, 1.00):
        bracket_y, bracket_z = _point_along_tilt(
            front_top_y,
            front_top_z,
            bracket_distance,
            front_angle,
        )
        for x in (-0.168, 0.168):
            _add_box(front, (0.016, 0.014, 0.115), (x, bracket_y, bracket_z))

    foot_y, foot_z = _point_along_tilt(
        front_top_y,
        front_top_z,
        front_rail_length - 0.012,
        front_angle,
    )
    for x in (-front_rail_x, front_rail_x):
        _add_box(
            front,
            (0.062, 0.055, 0.040),
            (x, foot_y, foot_z),
            (front_angle, 0.0, 0.0),
        )

    front.inertial = Inertial.from_geometry(
        Box((0.460, 0.580, 1.480)),
        mass=6.5,
        origin=Origin(xyz=(0.0, -0.235, -0.760)),
    )

    rear = model.part("rear_frame")

    rear_nominal_angle = 0.35
    rear_top_y = 0.016
    rear_top_z = -0.036
    rear_rail_length = 1.54
    rear_rail_x = 0.175
    rear_rail_y, rear_rail_z = _tilted_box_center_from_top(
        rear_top_y,
        rear_top_z,
        rear_rail_length,
        rear_nominal_angle,
    )

    _add_box(rear, (0.112, 0.018, 0.034), (0.0, 0.012, -0.010))
    _add_box(rear, (0.372, 0.064, 0.050), (0.0, 0.040, -0.038))

    for x in (-0.165, 0.165):
        _add_box(rear, (0.070, 0.030, 0.110), (x, 0.026, -0.046))

    for x in (-rear_rail_x, rear_rail_x):
        _add_box(
            rear,
            (0.042, 0.022, rear_rail_length),
            (x, rear_rail_y, rear_rail_z),
            (rear_nominal_angle, 0.0, 0.0),
        )

    spine_y, spine_z = _point_along_tilt(
        rear_top_y,
        rear_top_z,
        0.72,
        rear_nominal_angle,
    )
    _add_box(
        rear,
        (0.138, 0.028, 1.02),
        (0.0, spine_y, spine_z),
        (rear_nominal_angle, 0.0, 0.0),
    )

    for brace_distance in (0.34, 0.58, 0.82, 1.06, 1.28):
        brace_y, brace_z = _point_along_tilt(
            rear_top_y,
            rear_top_z,
            brace_distance,
            rear_nominal_angle,
        )
        _add_box(
            rear,
            (0.396, 0.058, 0.024),
            (0.0, brace_y, brace_z),
            (rear_nominal_angle, 0.0, 0.0),
        )

    for stiffener_distance in (0.70, 0.96):
        stiffener_y, stiffener_z = _point_along_tilt(
            rear_top_y,
            rear_top_z,
            stiffener_distance,
            rear_nominal_angle,
        )
        for x in (-0.168, 0.168):
            _add_box(
                rear,
                (0.028, 0.026, 0.180),
                (x, stiffener_y, stiffener_z),
                (rear_nominal_angle, 0.0, 0.0),
            )

    rear_foot_y, rear_foot_z = _point_along_tilt(
        rear_top_y,
        rear_top_z,
        rear_rail_length - 0.012,
        rear_nominal_angle,
    )
    for x in (-rear_rail_x, rear_rail_x):
        _add_box(
            rear,
            (0.062, 0.060, 0.040),
            (x, rear_foot_y, rear_foot_z),
            (rear_nominal_angle, 0.0, 0.0),
        )

    rear.inertial = Inertial.from_geometry(
        Box((0.420, 0.620, 1.540)),
        mass=3.9,
        origin=Origin(xyz=(0.0, 0.300, -0.800)),
    )

    model.articulation(
        "front_to_rear_hinge",
        ArticulationType.REVOLUTE,
        parent="front_frame",
        child="rear_frame",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-0.10,
            upper=0.16,
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
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.80)
    ctx.expect_joint_motion_axis(
        "front_to_rear_hinge",
        "rear_frame",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )

    with ctx.pose(front_to_rear_hinge=-0.10):
        ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.72)

    with ctx.pose(front_to_rear_hinge=0.16):
        ctx.expect_origin_distance("front_frame", "rear_frame", axes="xy", max_dist=0.92)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
