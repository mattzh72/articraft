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
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

HERE = Path(__file__).resolve().parent
DOOR_PLANE_Y = 0.685


def _box(part, size, xyz, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy))


def _cyl(part, radius, length, xyz, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_elevator")

    shaft_width = 2.25
    shaft_depth = 2.05
    shaft_height = 4.35

    cab_width = 1.55
    cab_depth = 1.30
    cab_height = 2.30
    cab_origin = (0.0, 0.10, 0.12)
    door_panel_width = 0.54
    door_panel_height = 2.10
    door_travel = 0.31
    door_plane_y = DOOR_PLANE_Y
    door_top_z = 2.22

    shaft = model.part("shaft_frame")
    _box(shaft, (shaft_width, shaft_depth, 0.12), (0.0, 0.0, 0.06))
    _box(shaft, (shaft_width, 0.10, shaft_height), (0.0, -0.975, shaft_height / 2.0))
    _box(shaft, (0.10, shaft_depth, shaft_height), (-1.075, 0.0, shaft_height / 2.0))
    _box(shaft, (0.10, shaft_depth, shaft_height), (1.075, 0.0, shaft_height / 2.0))

    _box(shaft, (1.95, 0.14, 0.04), (0.0, 0.955, 0.14))
    _box(shaft, (0.25, 0.14, 2.55), (-0.90, 0.955, 1.395))
    _box(shaft, (0.25, 0.14, 2.55), (0.90, 0.955, 1.395))
    _box(shaft, (1.95, 0.14, 0.16), (0.0, 0.955, 2.75))
    _box(shaft, (1.95, 0.06, 1.44), (0.0, 0.915, 3.55))

    for x in (-0.50, 0.50):
        _box(shaft, (0.04, 0.03, 4.00), (x, -0.93, 2.12))
        _box(shaft, (0.012, 0.08, 4.00), (x, -0.90, 2.12))
        for z in (0.32, 1.38, 2.44, 3.50):
            _box(shaft, (0.08, 0.10, 0.06), (x, -0.90, z))
    for z in (0.32, 1.38, 2.44, 3.50, 4.10):
        _box(shaft, (1.12, 0.06, 0.08), (0.0, -0.90, z))

    _box(shaft, (0.20, 0.20, 0.06), (0.0, 0.10, 0.09))
    _box(shaft, (0.04, 0.05, 0.28), (0.81, 0.905, 1.05))
    _box(shaft, (0.28, 0.05, 0.10), (0.0, 0.915, 2.93))

    shaft.inertial = Inertial.from_geometry(
        Box((shaft_width, shaft_depth, shaft_height)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, shaft_height / 2.0)),
    )

    cab = model.part("cab")
    _box(cab, (0.18, 0.10, 0.04), (0.0, 0.0, 0.02))
    _box(cab, (cab_width, cab_depth, 0.06), (0.0, 0.0, 0.06))
    _box(cab, (cab_width, 0.04, 2.18), (0.0, -0.63, 1.18))
    _box(cab, (0.04, 1.26, 2.18), (-0.755, 0.0, 1.18))
    _box(cab, (0.04, 1.26, 2.18), (0.755, 0.0, 1.18))
    _box(cab, (cab_width, cab_depth, 0.04), (0.0, 0.0, 2.34))

    _box(cab, (cab_width, 0.08, 0.02), (0.0, 0.62, 0.07))
    _box(cab, (0.08, 0.08, 2.18), (-0.72, 0.62, 1.18))
    _box(cab, (0.08, 0.08, 2.18), (0.72, 0.62, 1.18))
    _box(cab, (cab_width, 0.08, 0.10), (0.0, 0.62, 2.31))
    _box(cab, (1.12, 0.05, 0.03), (0.0, 0.675, 2.245))
    _box(cab, (0.12, 0.02, 0.02), (-0.275, 0.66, 2.22))
    _box(cab, (0.12, 0.02, 0.02), (0.275, 0.66, 2.22))

    _box(cab, (1.18, 0.08, 0.10), (0.0, -0.60, 0.16))
    _box(cab, (1.18, 0.08, 0.10), (0.0, -0.60, 2.26))
    _box(cab, (0.08, 0.08, 2.18), (-0.50, -0.60, 1.18))
    _box(cab, (0.08, 0.08, 2.18), (0.50, -0.60, 1.18))
    for x in (-0.50, 0.50):
        _box(cab, (0.08, 0.14, 0.10), (x, -0.64, 0.18))
        _box(cab, (0.08, 0.14, 0.10), (x, -0.64, 2.24))

    _box(cab, (0.04, 0.22, 1.10), (0.736, 0.24, 1.24))
    _box(cab, (0.90, 0.50, 0.03), (0.0, 0.0, 2.315))
    _cyl(cab, 0.014, 0.90, (0.0, -0.61, 1.08), rpy=(0.0, math.pi / 2.0, 0.0))
    _cyl(cab, 0.012, 0.60, (-0.742, 0.10, 1.06), rpy=(math.pi / 2.0, 0.0, 0.0))
    _cyl(cab, 0.012, 0.60, (0.742, 0.10, 1.06), rpy=(math.pi / 2.0, 0.0, 0.0))
    _box(cab, (0.10, 0.04, 0.08), (-0.32, -0.625, 1.08))
    _box(cab, (0.10, 0.04, 0.08), (0.32, -0.625, 1.08))
    _box(cab, (0.024, 0.68, 0.08), (-0.742, 0.10, 1.06))
    _box(cab, (0.024, 0.68, 0.08), (0.742, 0.10, 1.06))

    cab.inertial = Inertial.from_geometry(
        Box((1.65, 1.40, cab_height)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, cab_height / 2.0 + 0.03)),
    )

    left_door = model.part("left_door")
    _box(
        left_door,
        (door_panel_width, 0.025, door_panel_height),
        (0.0, 0.0, -door_panel_height / 2.0),
    )
    _box(left_door, (0.03, 0.010, door_panel_height), (-0.255, 0.0075, -door_panel_height / 2.0))
    _box(left_door, (0.03, 0.010, door_panel_height), (0.255, 0.0075, -door_panel_height / 2.0))
    _box(left_door, (0.44, 0.010, 0.04), (0.0, 0.0075, -0.48))
    _box(left_door, (0.44, 0.010, 0.04), (0.0, 0.0075, -1.34))
    left_door.inertial = Inertial.from_geometry(
        Box((door_panel_width, 0.025, door_panel_height)),
        mass=75.0,
        origin=Origin(xyz=(0.0, 0.0, -door_panel_height / 2.0)),
    )

    right_door = model.part("right_door")
    _box(
        right_door,
        (door_panel_width, 0.025, door_panel_height),
        (0.0, 0.0, -door_panel_height / 2.0),
    )
    _box(right_door, (0.03, 0.010, door_panel_height), (-0.255, 0.0075, -door_panel_height / 2.0))
    _box(right_door, (0.03, 0.010, door_panel_height), (0.255, 0.0075, -door_panel_height / 2.0))
    _box(right_door, (0.44, 0.010, 0.04), (0.0, 0.0075, -0.48))
    _box(right_door, (0.44, 0.010, 0.04), (0.0, 0.0075, -1.34))
    right_door.inertial = Inertial.from_geometry(
        Box((door_panel_width, 0.025, door_panel_height)),
        mass=75.0,
        origin=Origin(xyz=(0.0, 0.0, -door_panel_height / 2.0)),
    )

    model.articulation(
        "shaft_to_cab",
        ArticulationType.PRISMATIC,
        parent="shaft_frame",
        child="cab",
        origin=Origin(xyz=cab_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "cab_to_left_door",
        ArticulationType.PRISMATIC,
        parent="cab",
        child="left_door",
        origin=Origin(xyz=(-0.275, door_plane_y, door_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.2,
            lower=0.0,
            upper=door_travel,
        ),
    )
    model.articulation(
        "cab_to_right_door",
        ArticulationType.PRISMATIC,
        parent="cab",
        child="right_door",
        origin=Origin(xyz=(0.275, door_plane_y, door_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.2,
            lower=0.0,
            upper=door_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.02,
        reason="Recessed sliding-door hanger origins sit slightly behind the visible fascia.",
    )
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "cab",
        "left_door",
        reason="Door leaves ride in a tight recessed front channel; primitive collision hulls are slightly conservative at open poses.",
    )
    ctx.allow_overlap(
        "cab",
        "right_door",
        reason="Door leaves ride in a tight recessed front channel; primitive collision hulls are slightly conservative at open poses.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("cab", "shaft_frame", axes="xy", min_overlap=1.20)
    ctx.expect_joint_motion_axis(
        "shaft_to_cab",
        "cab",
        world_axis="z",
        direction="positive",
        min_delta=0.75,
    )
    ctx.expect_joint_motion_axis(
        "cab_to_left_door",
        "left_door",
        world_axis="x",
        direction="negative",
        min_delta=0.15,
    )
    ctx.expect_joint_motion_axis(
        "cab_to_right_door",
        "right_door",
        world_axis="x",
        direction="positive",
        min_delta=0.15,
    )

    cab_rest = ctx.part_world_position("cab")
    left_closed = ctx.part_world_position("left_door")
    right_closed = ctx.part_world_position("right_door")

    with ctx.pose(shaft_to_cab=1.20):
        cab_high = ctx.part_world_position("cab")
        ctx.expect_aabb_overlap("cab", "shaft_frame", axes="xy", min_overlap=1.20)

    cab_rise = cab_high[2] - cab_rest[2]
    if cab_rise < 1.0:
        raise AssertionError(f"Cab rise too small for an elevator travel: {cab_rise:.3f} m")
    if abs(cab_high[0] - cab_rest[0]) > 1e-6 or abs(cab_high[1] - cab_rest[1]) > 1e-6:
        raise AssertionError("Cab should translate vertically without lateral drift.")

    with ctx.pose(cab_to_left_door=0.31, cab_to_right_door=0.31):
        left_open = ctx.part_world_position("left_door")
        right_open = ctx.part_world_position("right_door")

    if abs(left_closed[1] - (cab_rest[1] + DOOR_PLANE_Y)) > 0.02:
        raise AssertionError("Left door should sit just ahead of the cab opening plane.")
    if abs(right_closed[1] - (cab_rest[1] + DOOR_PLANE_Y)) > 0.02:
        raise AssertionError("Right door should sit just ahead of the cab opening plane.")

    if left_open[0] > left_closed[0] - 0.20:
        raise AssertionError("Left door panel does not slide far enough toward the left pocket.")
    if right_open[0] < right_closed[0] + 0.20:
        raise AssertionError("Right door panel does not slide far enough toward the right pocket.")
    if right_open[0] - left_open[0] < 0.95:
        raise AssertionError(
            "Door opening is too narrow for a believable center-opening elevator door."
        )
    if abs(left_open[2] - left_closed[2]) > 1e-6 or abs(right_open[2] - right_closed[2]) > 1e-6:
        raise AssertionError("Door panels should slide horizontally without changing elevation.")
    if abs(left_open[1] - left_closed[1]) > 1e-6 or abs(right_open[1] - right_closed[1]) > 1e-6:
        raise AssertionError("Door panels should remain on their front track plane while sliding.")

    with ctx.pose(shaft_to_cab=1.20, cab_to_left_door=0.31, cab_to_right_door=0.31):
        left_open_high = ctx.part_world_position("left_door")
        right_open_high = ctx.part_world_position("right_door")
        ctx.expect_aabb_overlap("cab", "shaft_frame", axes="xy", min_overlap=1.20)

    if abs((left_open_high[2] - left_open[2]) - cab_rise) > 0.02:
        raise AssertionError("Left door should travel upward together with the cab.")
    if abs((right_open_high[2] - right_open[2]) - cab_rise) > 0.02:
        raise AssertionError("Right door should travel upward together with the cab.")
    if abs((right_open_high[0] - left_open_high[0]) - (right_open[0] - left_open[0])) > 0.01:
        raise AssertionError("Door opening width should stay consistent as the cab changes floors.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
