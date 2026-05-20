from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FOOT_RADIUS = 0.080
BASE_FOOT_THICKNESS = 0.022
BASE_COLUMN_RADIUS = 0.038
BASE_COLUMN_HEIGHT = 0.046
SHOULDER_AXIS_Z = 0.110

SHOULDER_HUB_RADIUS = 0.032
SHOULDER_HUB_LENGTH = 0.060
BASE_CHEEK_THICKNESS = 0.018
BASE_CHEEK_LENGTH = 0.056
BASE_CHEEK_HEIGHT = 0.076

UPPER_ARM_LENGTH = 0.195
UPPER_ARM_BEAM_WIDTH = 0.054
UPPER_ARM_BEAM_HEIGHT = 0.062
ELBOW_HUB_RADIUS = 0.024
ELBOW_HUB_LENGTH = 0.044
ELBOW_CHEEK_THICKNESS = 0.010
ELBOW_CHEEK_LENGTH = 0.028
ELBOW_CHEEK_HEIGHT = 0.058

FOREARM_LENGTH = 0.155
FOREARM_BEAM_WIDTH = 0.038
FOREARM_BEAM_HEIGHT = 0.044
WRIST_HUB_RADIUS = 0.020
WRIST_HUB_LENGTH = 0.036
WRIST_CHEEK_THICKNESS = 0.012
WRIST_CHEEK_LENGTH = 0.024
WRIST_CHEEK_HEIGHT = 0.046

OUTPUT_PLATE_SIZE = 0.058
OUTPUT_PLATE_THICKNESS = 0.012
OUTPUT_PLATE_CENTER_X = 0.091


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((cx, cy, cz - length / 2.0))


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _half_hub_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    side: str,
) -> cq.Workplane:
    cx, cy, cz = center
    hub = _cylinder_y(radius, length, center)
    if side == "positive":
        cutter_center = (cx + radius, cy, cz)
    elif side == "negative":
        cutter_center = (cx - radius, cy, cz)
    else:
        raise ValueError(f"Unsupported half-hub side: {side}")
    cutter = _box((2.0 * radius, length + 0.004, 2.0 * radius + 0.004), cutter_center)
    return hub.intersect(cutter)


def _make_base_shape() -> cq.Workplane:
    foot = _cylinder_z(
        BASE_FOOT_RADIUS,
        BASE_FOOT_THICKNESS,
        (0.0, 0.0, BASE_FOOT_THICKNESS / 2.0),
    )
    column = _cylinder_z(
        BASE_COLUMN_RADIUS,
        BASE_COLUMN_HEIGHT,
        (0.0, 0.0, BASE_FOOT_THICKNESS + BASE_COLUMN_HEIGHT / 2.0),
    )
    support_block = _box((0.030, 0.036, 0.050), (-0.038, 0.0, 0.091))
    yoke_blank = _box((0.064, 0.080, 0.078), (-0.036, 0.0, SHOULDER_AXIS_Z))
    yoke_slot = _box((0.060, 0.056, 0.090), (-0.018, 0.0, SHOULDER_AXIS_Z))
    yoke = yoke_blank.cut(yoke_slot)

    return foot.union(column).union(support_block).union(yoke)


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _half_hub_y(0.024, 0.056, (0.0, 0.0, 0.0), "positive")
    shoulder_motor = _box((0.060, 0.044, 0.072), (0.048, 0.0, 0.0))
    beam = _box((0.104, 0.040, 0.050), (0.124, 0.0, 0.0))
    elbow_case = _box((0.036, 0.052, 0.060), (UPPER_ARM_LENGTH - 0.018, 0.0, 0.0))
    elbow_half_hub = _half_hub_y(
        ELBOW_HUB_RADIUS,
        ELBOW_HUB_LENGTH,
        (UPPER_ARM_LENGTH, 0.0, 0.0),
        "negative",
    )

    return shoulder_hub.union(shoulder_motor).union(beam).union(elbow_case).union(elbow_half_hub)


def _make_forearm_shape() -> cq.Workplane:
    elbow_half_hub = _half_hub_y(
        ELBOW_HUB_RADIUS,
        ELBOW_HUB_LENGTH,
        (0.0, 0.0, 0.0),
        "positive",
    )
    proximal_block = _box((0.036, 0.050, 0.050), (0.030, 0.0, 0.0))
    beam = _box((0.082, 0.038, 0.042), (0.089, 0.0, 0.0))
    wrist_case = _box((0.028, 0.046, 0.050), (FOREARM_LENGTH - 0.014, 0.0, 0.0))
    wrist_half_hub = _half_hub_y(
        WRIST_HUB_RADIUS,
        WRIST_HUB_LENGTH,
        (FOREARM_LENGTH, 0.0, 0.0),
        "negative",
    )

    return elbow_half_hub.union(proximal_block).union(beam).union(wrist_case).union(wrist_half_hub)


def _make_wrist_yoke_shape() -> cq.Workplane:
    hub = _half_hub_y(WRIST_HUB_RADIUS, WRIST_HUB_LENGTH, (0.0, 0.0, 0.0), "positive")
    rear_block = _box((0.032, 0.032, 0.052), (0.018, 0.0, 0.0))
    upper_arm = _box((0.052, 0.028, 0.012), (0.056, 0.0, 0.018))
    lower_arm = _box((0.052, 0.028, 0.012), (0.056, 0.0, -0.018))
    plate_mount = _box((0.010, 0.028, 0.032), (0.080, 0.0, 0.0))

    return hub.union(rear_block).union(upper_arm).union(lower_arm).union(plate_mount)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_robot_arm")

    model.material("base_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("arm_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("forearm_gray", rgba=(0.52, 0.56, 0.61, 1.0))
    model.material("wrist_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("plate_silver", rgba=(0.76, 0.79, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_THICKNESS / 2.0)),
        material="base_black",
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=BASE_COLUMN_RADIUS, length=BASE_COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_THICKNESS + BASE_COLUMN_HEIGHT / 2.0)),
        material="base_black",
        name="base_column",
    )
    base.visual(
        Box((0.030, 0.036, 0.050)),
        origin=Origin(xyz=(-0.038, 0.0, 0.091)),
        material="base_black",
        name="base_support",
    )
    base.visual(
        Box((0.014, 0.080, 0.074)),
        origin=Origin(xyz=(-0.050, 0.0, SHOULDER_AXIS_Z)),
        material="base_black",
        name="rear_bridge",
    )
    base.visual(
        Box((0.050, 0.012, 0.074)),
        origin=Origin(xyz=(-0.025, 0.034, SHOULDER_AXIS_Z)),
        material="base_black",
        name="left_cheek",
    )
    base.visual(
        Box((0.050, 0.012, 0.074)),
        origin=Origin(xyz=(-0.025, -0.034, SHOULDER_AXIS_Z)),
        material="base_black",
        name="right_cheek",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=0.024, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="arm_gray",
        name="shoulder_hub",
    )
    shoulder_link.visual(
        Box((0.060, 0.044, 0.072)),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        material="arm_gray",
        name="shoulder_motor",
    )
    shoulder_link.visual(
        Box((0.104, 0.040, 0.050)),
        origin=Origin(xyz=(0.124, 0.0, 0.0)),
        material="arm_gray",
        name="upper_beam",
    )
    shoulder_link.visual(
        mesh_from_cadquery(_half_hub_y(ELBOW_HUB_RADIUS, ELBOW_HUB_LENGTH, (UPPER_ARM_LENGTH, 0.0, 0.0), "negative").union(
            _box((0.036, 0.052, 0.060), (UPPER_ARM_LENGTH - 0.018, 0.0, 0.0))
        ), "shoulder_elbow_end"),
        material="arm_gray",
        name="elbow_end",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_make_forearm_shape(), "forearm"),
        material="forearm_gray",
        name="forearm_body",
    )

    wrist_yoke = model.part("wrist_yoke")
    wrist_yoke.visual(
        mesh_from_cadquery(_make_wrist_yoke_shape(), "wrist_yoke"),
        material="wrist_dark",
        name="wrist_body",
    )
    wrist_yoke.visual(
        Box((OUTPUT_PLATE_THICKNESS, OUTPUT_PLATE_SIZE, OUTPUT_PLATE_SIZE)),
        origin=Origin(xyz=(OUTPUT_PLATE_CENTER_X, 0.0, 0.0)),
        material="plate_silver",
        name="output_plate",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.25, effort=35.0, velocity=1.8),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.95, effort=25.0, velocity=2.0),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_yoke,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.10, effort=10.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shoulder_link = object_model.get_part("shoulder_link")
    forearm = object_model.get_part("forearm")
    wrist_yoke = object_model.get_part("wrist_yoke")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    wrist_joint = object_model.get_articulation("wrist_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(base, shoulder_link, name="base touches shoulder hub")
    ctx.expect_contact(shoulder_link, forearm, name="upper arm touches forearm hub")
    ctx.expect_contact(forearm, wrist_yoke, name="forearm touches wrist hub")

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.0}):
        rest_plate_center = _aabb_center(ctx.part_element_world_aabb(wrist_yoke, elem="output_plate"))

    with ctx.pose({shoulder_joint: 0.90, elbow_joint: 0.0, wrist_joint: 0.0}):
        shoulder_up_center = _aabb_center(ctx.part_element_world_aabb(wrist_yoke, elem="output_plate"))

    ctx.check(
        "shoulder positive angle raises the end plate",
        shoulder_up_center[2] > rest_plate_center[2] + 0.10,
        details=f"rest_z={rest_plate_center[2]:.4f}, raised_z={shoulder_up_center[2]:.4f}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 1.10, wrist_joint: 0.0}):
        elbow_up_center = _aabb_center(ctx.part_element_world_aabb(wrist_yoke, elem="output_plate"))

    ctx.check(
        "elbow positive angle folds the forearm upward",
        elbow_up_center[2] > rest_plate_center[2] + 0.06 and elbow_up_center[0] < rest_plate_center[0] - 0.04,
        details=(
            f"rest_center={rest_plate_center}, elbow_center={elbow_up_center}"
        ),
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.85}):
        wrist_up_center = _aabb_center(ctx.part_element_world_aabb(wrist_yoke, elem="output_plate"))
    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: -0.85}):
        wrist_down_center = _aabb_center(ctx.part_element_world_aabb(wrist_yoke, elem="output_plate"))

    ctx.check(
        "wrist joint pitches the square output plate",
        wrist_up_center[2] > wrist_down_center[2] + 0.05,
        details=f"wrist_up={wrist_up_center}, wrist_down={wrist_down_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
