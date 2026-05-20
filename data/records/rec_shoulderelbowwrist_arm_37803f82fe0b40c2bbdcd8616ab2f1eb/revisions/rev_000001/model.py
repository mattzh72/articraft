from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLATE_WIDTH = 0.18
PLATE_HEIGHT = 0.34
PLATE_THICKNESS = 0.012
SHOULDER_HEIGHT = 0.225
SHOULDER_OFFSET_Y = 0.030

UPPER_ARM_LENGTH = 0.22
FOREARM_LENGTH = 0.18
WRIST_LENGTH = 0.082

def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][2] + aabb[1][2]) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_arm_module")

    model.material("plate_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("housing_gray", rgba=(0.47, 0.50, 0.54, 1.0))
    model.material("arm_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("forearm_silver", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("wrist_black", rgba=(0.14, 0.15, 0.17, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_WIDTH, PLATE_THICKNESS, PLATE_HEIGHT)),
        material="plate_steel",
        name="plate_body",
        origin=Origin(xyz=(0.0, 0.0, PLATE_HEIGHT / 2.0)),
    )
    side_plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, 0.020, PLATE_HEIGHT)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.004, PLATE_HEIGHT / 2.0)),
    )

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Box((0.060, 0.024, 0.100)),
        material="housing_gray",
        name="housing_body",
        origin=Origin(xyz=(-0.030, -0.012, 0.0)),
    )
    shoulder_housing.visual(
        Box((0.030, 0.024, 0.022)),
        material="housing_gray",
        name="housing_top_cap",
        origin=Origin(xyz=(-0.060, -0.012, 0.030)),
    )
    shoulder_housing.visual(
        Box((0.030, 0.024, 0.022)),
        material="housing_gray",
        name="housing_bottom_cap",
        origin=Origin(xyz=(-0.060, -0.012, -0.030)),
    )
    shoulder_housing.inertial = Inertial.from_geometry(
        Box((0.080, 0.050, 0.100)),
        mass=1.5,
        origin=Origin(),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((UPPER_ARM_LENGTH, 0.018, 0.038)),
        material="arm_silver",
        name="upper_arm_body",
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )
    upper_arm.visual(
        Box((0.110, 0.014, 0.012)),
        material="arm_silver",
        name="upper_arm_cover",
        origin=Origin(xyz=(0.085, 0.0, 0.024)),
    )
    upper_arm.visual(
        Box((0.040, 0.018, 0.046)),
        material="arm_silver",
        name="upper_arm_shoulder_pad",
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.032, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((FOREARM_LENGTH, 0.016, 0.032)),
        material="forearm_silver",
        name="forearm_body",
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )
    forearm.visual(
        Box((0.100, 0.014, 0.010)),
        material="forearm_silver",
        name="forearm_cover",
        origin=Origin(xyz=(0.075, 0.0, 0.021)),
    )
    forearm.visual(
        Box((0.032, 0.016, 0.042)),
        material="forearm_silver",
        name="forearm_wrist_block",
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, 0.028, 0.042)),
        mass=0.9,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    wrist_member = model.part("wrist_member")
    wrist_member.visual(
        Box((WRIST_LENGTH, 0.016, 0.026)),
        material="wrist_black",
        name="wrist_body",
        origin=Origin(xyz=(WRIST_LENGTH / 2.0, 0.0, 0.0)),
    )
    wrist_member.visual(
        Box((0.030, 0.018, 0.012)),
        material="wrist_black",
        name="wrist_top_cap",
        origin=Origin(xyz=(0.030, 0.0, 0.018)),
    )
    wrist_member.visual(
        Box((0.018, 0.026, 0.036)),
        material="wrist_black",
        name="tool_flange",
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
    )
    wrist_member.inertial = Inertial.from_geometry(
        Box((WRIST_LENGTH, 0.035, 0.045)),
        mass=0.35,
        origin=Origin(xyz=(WRIST_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "plate_to_shoulder_housing",
        ArticulationType.FIXED,
        parent=side_plate,
        child=shoulder_housing,
        origin=Origin(xyz=(0.0, SHOULDER_OFFSET_Y, SHOULDER_HEIGHT)),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_housing,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.10,
            upper=1.45,
            effort=85.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.15,
            effort=55.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_member,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.90,
            upper=1.00,
            effort=20.0,
            velocity=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_member = object_model.get_part("wrist_member")

    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")

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

    ctx.check(
        "all_pitch_axes_are_parallel_to_side_plate_normal",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (0.0, -1.0, 0.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        ctx.expect_contact(shoulder_housing, side_plate, contact_tol=0.001, name="housing_mounts_to_plate")
        ctx.expect_contact(upper_arm, shoulder_housing, contact_tol=0.001, name="upper_arm_supported_in_shoulder")
        ctx.expect_contact(forearm, upper_arm, contact_tol=0.001, name="forearm_supported_in_elbow")
        ctx.expect_contact(wrist_member, forearm, contact_tol=0.001, name="wrist_supported_in_forearm")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        upper_rest = _center_z(ctx.part_world_aabb(upper_arm))
        forearm_rest = _center_z(ctx.part_world_aabb(forearm))
        wrist_rest = _center_z(ctx.part_world_aabb(wrist_member))

    with ctx.pose({shoulder: 0.80, elbow: 0.0, wrist: 0.0}):
        upper_raised = _center_z(ctx.part_world_aabb(upper_arm))
    ctx.check(
        "positive_shoulder_rotation_raises_upper_arm",
        upper_rest is not None and upper_raised is not None and upper_raised > upper_rest + 0.05,
        details=f"rest_z={upper_rest}, raised_z={upper_raised}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 1.00, wrist: 0.0}):
        forearm_raised = _center_z(ctx.part_world_aabb(forearm))
    ctx.check(
        "positive_elbow_rotation_raises_forearm",
        forearm_rest is not None and forearm_raised is not None and forearm_raised > forearm_rest + 0.05,
        details=f"rest_z={forearm_rest}, raised_z={forearm_raised}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.60}):
        wrist_raised = _center_z(ctx.part_world_aabb(wrist_member))
    ctx.check(
        "positive_wrist_rotation_raises_wrist_member",
        wrist_rest is not None and wrist_raised is not None and wrist_raised > wrist_rest + 0.015,
        details=f"rest_z={wrist_rest}, raised_z={wrist_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
