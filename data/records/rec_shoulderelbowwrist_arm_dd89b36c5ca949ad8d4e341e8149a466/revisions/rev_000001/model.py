from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


SHOULDER_HEIGHT = 0.255
UPPER_ARM_LENGTH = 0.300
FOREARM_LENGTH = 0.220


def _box(x: float, y: float, z: float, *, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z).translate((cx, cy, cz))


def _cylinder_y(length: float, radius: float, *, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _cylinder_x(length: float, radius: float, *, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _build_base_fork() -> cq.Workplane:
    fork = _box(0.24, 0.18, 0.02, cz=0.01)
    fork = fork.union(_box(0.10, 0.09, 0.18, cx=-0.075, cz=0.11))
    fork = fork.union(_box(0.07, 0.05, 0.04, cx=-0.055, cz=0.20))
    fork = fork.union(_box(0.07, 0.008, 0.09, cx=0.005, cy=0.023, cz=SHOULDER_HEIGHT))
    fork = fork.union(_box(0.07, 0.008, 0.09, cx=0.005, cy=-0.023, cz=SHOULDER_HEIGHT))
    return fork


def _build_upper_arm() -> cq.Workplane:
    arm = _cylinder_y(0.032, 0.015)
    arm = arm.union(_box(0.24, 0.028, 0.038, cx=0.12))
    arm = arm.union(_box(0.06, 0.042, 0.046, cx=0.27))
    arm = arm.cut(_cylinder_y(0.045, 0.006))
    arm = arm.cut(_box(0.042, 0.026, 0.026, cx=0.282))
    arm = arm.cut(_cylinder_y(0.045, 0.0055, cx=UPPER_ARM_LENGTH))
    return arm


def _build_forearm() -> cq.Workplane:
    forearm = _cylinder_y(0.024, 0.011)
    forearm = forearm.union(_box(0.18, 0.022, 0.032, cx=0.09))
    forearm = forearm.union(_box(0.06, 0.036, 0.04, cx=0.19))
    forearm = forearm.cut(_cylinder_y(0.036, 0.0055))
    forearm = forearm.cut(_box(0.040, 0.022, 0.022, cx=0.202))
    forearm = forearm.cut(_cylinder_y(0.032, 0.0045, cx=FOREARM_LENGTH))
    return forearm


def _build_wrist_flange() -> cq.Workplane:
    wrist = _cylinder_y(0.018, 0.009)
    wrist = wrist.union(_box(0.040, 0.014, 0.016, cx=0.020))
    wrist = wrist.union(_box(0.020, 0.018, 0.018, cx=0.045))
    wrist = wrist.union(_cylinder_x(0.010, 0.032, cx=0.055))
    wrist = wrist.cut(_cylinder_y(0.028, 0.0045))
    wrist = wrist.cut(_cylinder_x(0.016, 0.009, cx=0.055))
    for y_off in (-0.018, 0.018):
        for z_off in (-0.018, 0.018):
            wrist = wrist.cut(_cylinder_x(0.016, 0.003, cx=0.055, cy=y_off, cz=z_off))
    return wrist


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_three_joint_arm")

    dark_base = model.material("dark_base", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.63, 0.68, 0.73, 1.0))
    forearm_paint = model.material("forearm_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.70, 1.0))

    base_fork = model.part("base_fork")
    base_fork.visual(
        Box((0.24, 0.18, 0.02)),
        material=dark_base,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    base_fork.visual(
        Box((0.10, 0.09, 0.18)),
        material=dark_base,
        origin=Origin(xyz=(-0.075, 0.0, 0.11)),
        name="pedestal",
    )
    base_fork.visual(
        Box((0.07, 0.05, 0.04)),
        material=dark_base,
        origin=Origin(xyz=(-0.045, 0.0, 0.20)),
        name="fork_bridge",
    )
    base_fork.visual(
        Box((0.07, 0.008, 0.09)),
        material=dark_base,
        origin=Origin(xyz=(0.005, 0.026, SHOULDER_HEIGHT)),
        name="left_cheek",
    )
    base_fork.visual(
        Box((0.07, 0.008, 0.09)),
        material=dark_base,
        origin=Origin(xyz=(0.005, -0.026, SHOULDER_HEIGHT)),
        name="right_cheek",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.015, length=0.044),
        material=arm_paint,
        origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Box((0.288, 0.028, 0.038)),
        material=arm_paint,
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.05, 0.042, 0.046)),
        material=arm_paint,
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        name="elbow_block",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.220, 0.022, 0.032)),
        material=forearm_paint,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.06, 0.036, 0.040)),
        material=forearm_paint,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        name="wrist_mount",
    )

    wrist_flange = model.part("wrist_flange")
    wrist_flange.visual(
        Box((0.050, 0.016, 0.016)),
        material=steel,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        name="wrist_body",
    )
    wrist_flange.visual(
        Cylinder(radius=0.032, length=0.010),
        material=steel,
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        name="flange_plate",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_fork,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=-0.45, upper=1.35),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.6, lower=-0.10, upper=2.10),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_flange,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_fork = object_model.get_part("base_fork")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_flange = object_model.get_part("wrist_flange")
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

    ctx.expect_contact(base_fork, upper_arm, name="shoulder_joint_is_supported")
    ctx.expect_contact(upper_arm, forearm, name="elbow_joint_is_supported")
    ctx.expect_contact(forearm, wrist_flange, name="wrist_joint_is_supported")

    ctx.expect_origin_distance(
        upper_arm,
        forearm,
        axes="x",
        min_dist=0.295,
        max_dist=0.305,
        name="elbow_origin_is_at_upper_arm_tip",
    )
    ctx.expect_origin_distance(
        forearm,
        wrist_flange,
        axes="x",
        min_dist=0.215,
        max_dist=0.225,
        name="wrist_origin_is_at_forearm_tip",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.0}):
        elbow_at_rest = ctx.part_world_position(forearm)
        wrist_at_rest = ctx.part_world_position(wrist_flange)
        plate_aabb_rest = ctx.part_element_world_aabb(wrist_flange, elem="flange_plate")

    with ctx.pose({shoulder_joint: 0.80, elbow_joint: 0.0, wrist_joint: 0.0}):
        elbow_raised = ctx.part_world_position(forearm)

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.95, wrist_joint: 0.0}):
        wrist_raised = ctx.part_world_position(wrist_flange)

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, wrist_joint: 0.80}):
        plate_aabb_raised = ctx.part_element_world_aabb(wrist_flange, elem="flange_plate")

    def z_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return 0.5 * (mins[2] + maxs[2])

    shoulder_ok = elbow_at_rest is not None and elbow_raised is not None and elbow_raised[2] > elbow_at_rest[2] + 0.18
    ctx.check(
        "shoulder_positive_rotation_lifts_elbow",
        shoulder_ok,
        details=f"rest={elbow_at_rest}, raised={elbow_raised}",
    )

    elbow_ok = wrist_at_rest is not None and wrist_raised is not None and wrist_raised[2] > wrist_at_rest[2] + 0.14
    ctx.check(
        "elbow_positive_rotation_lifts_wrist",
        elbow_ok,
        details=f"rest={wrist_at_rest}, raised={wrist_raised}",
    )

    plate_rest_z = z_center(plate_aabb_rest)
    plate_raised_z = z_center(plate_aabb_raised)
    wrist_ok = (
        plate_rest_z is not None
        and plate_raised_z is not None
        and plate_raised_z > plate_rest_z + 0.015
    )
    ctx.check(
        "wrist_positive_rotation_lifts_flange_face",
        wrist_ok,
        details=f"rest_z={plate_rest_z}, raised_z={plate_raised_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
