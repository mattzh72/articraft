from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
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
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.090
BASE_HEIGHT = 0.022
COLUMN_SIZE = (0.060, 0.060, 0.205)
SHOULDER_X = 0.220
SHOULDER_Z = 0.270
SHOULDER_HUB_RADIUS = 0.026
SHOULDER_HUB_LENGTH = 0.050
SHOULDER_CHEEK_THICKNESS = 0.008
SHOULDER_CHEEK_SIZE = (0.034, SHOULDER_CHEEK_THICKNESS, 0.056)
SHOULDER_CHEEK_OFFSET_Y = SHOULDER_HUB_LENGTH / 2.0 + SHOULDER_CHEEK_THICKNESS / 2.0
SHOULDER_BRIDGE_SIZE = (0.130, 0.072, 0.024)
SHOULDER_BRIDGE_CENTER_X = 0.065
SHOULDER_BRIDGE_CENTER_Z = 0.223
SHOULDER_STRUT_SIZE = (0.090, 0.008, 0.040)
SHOULDER_STRUT_CENTER_X = 0.175
SHOULDER_STRUT_OFFSET_Y = 0.031
SHOULDER_STRUT_CENTER_Z = 0.240

UPPER_ARM_LENGTH = 0.180
UPPER_ARM_BODY_LENGTH = 0.136
UPPER_ARM_BODY_SIZE = (UPPER_ARM_BODY_LENGTH, 0.030, 0.028)
UPPER_ARM_BODY_CENTER_X = 0.022 + UPPER_ARM_BODY_LENGTH / 2.0
UPPER_ARM_FORK_GAP = 0.034
UPPER_ARM_FORK_THICKNESS = 0.008
UPPER_ARM_FORK_SIZE = (0.030, UPPER_ARM_FORK_THICKNESS, 0.046)
UPPER_ARM_FORK_CENTER_X = UPPER_ARM_LENGTH - UPPER_ARM_FORK_SIZE[0] / 2.0
UPPER_ARM_FORK_OFFSET_Y = UPPER_ARM_FORK_GAP / 2.0 + UPPER_ARM_FORK_THICKNESS / 2.0
UPPER_ARM_SIDE_RIB_SIZE = (0.024, 0.004, 0.034)
UPPER_ARM_SIDE_RIB_CENTER_X = 0.146
UPPER_ARM_SIDE_RIB_OFFSET_Y = 0.015 + UPPER_ARM_SIDE_RIB_SIZE[1] / 2.0

ELBOW_HUB_RADIUS = 0.022
ELBOW_HUB_LENGTH = UPPER_ARM_FORK_GAP
FOREARM_LENGTH = 0.170
FOREARM_BODY_LENGTH = 0.127
FOREARM_BODY_SIZE = (FOREARM_BODY_LENGTH, 0.026, 0.024)
FOREARM_BODY_CENTER_X = 0.018 + FOREARM_BODY_LENGTH / 2.0
FOREARM_WRIST_BLOCK_SIZE = (0.026, 0.032, 0.032)
FOREARM_WRIST_BLOCK_CENTER_X = FOREARM_LENGTH - FOREARM_WRIST_BLOCK_SIZE[0] / 2.0

END_PLATE_MOUNT_SIZE = (0.016, 0.030, 0.030)
END_PLATE_FACE_SIZE = (0.006, 0.075, 0.060)
END_PLATE_FACE_CENTER_X = END_PLATE_MOUNT_SIZE[0] + END_PLATE_FACE_SIZE[0] / 2.0

JOINT_LOWER = math.radians(-120.0)
JOINT_UPPER = math.radians(135.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elbow_arm_module", assets=ASSETS)

    dark = model.material("dark", rgba=(0.20, 0.22, 0.24, 1.0))
    base_gray = model.material("base_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.62, 0.64, 0.68, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=dark,
        name="base_disk",
    )
    pedestal.visual(
        Box(COLUMN_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + COLUMN_SIZE[2] / 2.0)),
        material=base_gray,
        name="column",
    )
    pedestal.visual(
        Box(SHOULDER_BRIDGE_SIZE),
        origin=Origin(xyz=(SHOULDER_BRIDGE_CENTER_X, 0.0, SHOULDER_BRIDGE_CENTER_Z)),
        material=base_gray,
        name="shoulder_bridge",
    )
    pedestal.visual(
        Box(SHOULDER_STRUT_SIZE),
        origin=Origin(xyz=(SHOULDER_STRUT_CENTER_X, SHOULDER_STRUT_OFFSET_Y, SHOULDER_STRUT_CENTER_Z)),
        material=base_gray,
        name="shoulder_strut_left",
    )
    pedestal.visual(
        Box(SHOULDER_STRUT_SIZE),
        origin=Origin(xyz=(SHOULDER_STRUT_CENTER_X, -SHOULDER_STRUT_OFFSET_Y, SHOULDER_STRUT_CENTER_Z)),
        material=base_gray,
        name="shoulder_strut_right",
    )
    pedestal.visual(
        Box(SHOULDER_CHEEK_SIZE),
        origin=Origin(xyz=(SHOULDER_X, SHOULDER_CHEEK_OFFSET_Y, SHOULDER_Z)),
        material=base_gray,
        name="shoulder_cheek_left",
    )
    pedestal.visual(
        Box(SHOULDER_CHEEK_SIZE),
        origin=Origin(xyz=(SHOULDER_X, -SHOULDER_CHEEK_OFFSET_Y, SHOULDER_Z)),
        material=base_gray,
        name="shoulder_cheek_right",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.360, 0.180, SHOULDER_Z + 0.030)),
        mass=4.0,
        origin=Origin(xyz=(0.100, 0.0, (SHOULDER_Z + 0.030) / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box(UPPER_ARM_BODY_SIZE),
        origin=Origin(xyz=(UPPER_ARM_BODY_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="upper_arm_body",
    )
    upper_arm.visual(
        Box(UPPER_ARM_SIDE_RIB_SIZE),
        origin=Origin(xyz=(UPPER_ARM_SIDE_RIB_CENTER_X, UPPER_ARM_SIDE_RIB_OFFSET_Y, 0.0)),
        material=aluminum,
        name="elbow_rib_left",
    )
    upper_arm.visual(
        Box(UPPER_ARM_SIDE_RIB_SIZE),
        origin=Origin(xyz=(UPPER_ARM_SIDE_RIB_CENTER_X, -UPPER_ARM_SIDE_RIB_OFFSET_Y, 0.0)),
        material=aluminum,
        name="elbow_rib_right",
    )
    upper_arm.visual(
        Box(UPPER_ARM_FORK_SIZE),
        origin=Origin(xyz=(UPPER_ARM_FORK_CENTER_X, UPPER_ARM_FORK_OFFSET_Y, 0.0)),
        material=aluminum,
        name="elbow_fork_left",
    )
    upper_arm.visual(
        Box(UPPER_ARM_FORK_SIZE),
        origin=Origin(xyz=(UPPER_ARM_FORK_CENTER_X, -UPPER_ARM_FORK_OFFSET_Y, 0.0)),
        material=aluminum,
        name="elbow_fork_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.050, 0.055)),
        mass=1.6,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=ELBOW_HUB_RADIUS, length=ELBOW_HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="elbow_hub",
    )
    forearm.visual(
        Box(FOREARM_BODY_SIZE),
        origin=Origin(xyz=(FOREARM_BODY_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="forearm_body",
    )
    forearm.visual(
        Box(FOREARM_WRIST_BLOCK_SIZE),
        origin=Origin(xyz=(FOREARM_WRIST_BLOCK_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="wrist_block",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, 0.040, 0.040)),
        mass=1.1,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Box(END_PLATE_MOUNT_SIZE),
        origin=Origin(xyz=(END_PLATE_MOUNT_SIZE[0] / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="mount_block",
    )
    end_plate.visual(
        Box(END_PLATE_FACE_SIZE),
        origin=Origin(xyz=(END_PLATE_FACE_CENTER_X, 0.0, 0.0)),
        material=plate_gray,
        name="plate_face",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((0.026, 0.080, 0.065)),
        mass=0.35,
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.0,
            lower=JOINT_LOWER,
            upper=JOINT_UPPER,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.4,
            lower=JOINT_LOWER,
            upper=JOINT_UPPER,
        ),
    )
    model.articulation(
        "forearm_to_end_plate",
        ArticulationType.FIXED,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    plate_mount = object_model.get_articulation("forearm_to_end_plate")

    shoulder_hub = upper_arm.get_visual("shoulder_hub")
    shoulder_cheek_left = pedestal.get_visual("shoulder_cheek_left")
    shoulder_cheek_right = pedestal.get_visual("shoulder_cheek_right")
    elbow_hub = forearm.get_visual("elbow_hub")
    elbow_fork_left = upper_arm.get_visual("elbow_fork_left")
    elbow_fork_right = upper_arm.get_visual("elbow_fork_right")
    wrist_block = forearm.get_visual("wrist_block")
    mount_block = end_plate.get_visual("mount_block")
    plate_face = end_plate.get_visual("plate_face")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.expect_contact(
        upper_arm,
        pedestal,
        elem_a=shoulder_hub,
        elem_b=shoulder_cheek_left,
        name="shoulder_hub_contacts_left_cheek",
    )
    ctx.expect_contact(
        upper_arm,
        pedestal,
        elem_a=shoulder_hub,
        elem_b=shoulder_cheek_right,
        name="shoulder_hub_contacts_right_cheek",
    )
    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="xz",
        elem_a=shoulder_hub,
        elem_b=shoulder_cheek_left,
        min_overlap=0.025,
        name="shoulder_hub_seats_in_cheek_window",
    )

    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=elbow_hub,
        elem_b=elbow_fork_left,
        name="elbow_hub_contacts_left_fork",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=elbow_hub,
        elem_b=elbow_fork_right,
        name="elbow_hub_contacts_right_fork",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        elem_a=elbow_hub,
        elem_b=elbow_fork_left,
        min_overlap=0.020,
        name="elbow_hub_seats_in_fork_window",
    )

    ctx.expect_contact(
        end_plate,
        forearm,
        elem_a=mount_block,
        elem_b=wrist_block,
        name="end_plate_mount_contacts_forearm",
    )
    ctx.expect_overlap(
        end_plate,
        forearm,
        axes="yz",
        elem_a=mount_block,
        elem_b=wrist_block,
        min_overlap=0.028,
        name="end_plate_mount_footprint_matches_forearm",
    )
    ctx.expect_within(
        end_plate,
        forearm,
        axes="z",
        inner_elem=mount_block,
        outer_elem=wrist_block,
        margin=0.002,
        name="end_plate_mount_stays_within_forearm_height",
    )

    ctx.check(
        "shoulder_axis_is_horizontal_y",
        tuple(shoulder.axis) == (0.0, 1.0, 0.0),
        f"shoulder axis was {shoulder.axis}",
    )
    ctx.check(
        "elbow_axis_is_horizontal_y",
        tuple(elbow.axis) == (0.0, 1.0, 0.0),
        f"elbow axis was {elbow.axis}",
    )
    ctx.check(
        "serial_arm_joint_chain",
        getattr(shoulder.child, "name", shoulder.child) == upper_arm.name
        and getattr(elbow.parent, "name", elbow.parent) == upper_arm.name
        and getattr(elbow.child, "name", elbow.child) == forearm.name,
        "pedestal -> upper_arm -> forearm chain is incorrect",
    )
    ctx.check(
        "end_plate_is_fixed_to_forearm",
        getattr(plate_mount.parent, "name", plate_mount.parent) == forearm.name
        and getattr(plate_mount.child, "name", plate_mount.child) == end_plate.name,
        "end plate is not rigidly mounted to the forearm",
    )
    ctx.check(
        "shoulder_limits_match_requested_range",
        shoulder.motion_limits.lower <= math.radians(-115.0)
        and shoulder.motion_limits.upper >= math.radians(130.0),
        (
            "shoulder limits should provide about -120 deg rearward and +135 deg forward travel; "
            f"got {shoulder.motion_limits.lower:.3f} to {shoulder.motion_limits.upper:.3f}"
        ),
    )
    ctx.check(
        "elbow_limits_match_requested_range",
        elbow.motion_limits.lower <= math.radians(-115.0)
        and elbow.motion_limits.upper >= math.radians(130.0),
        (
            "elbow limits should provide about -120 deg rearward and +135 deg forward travel; "
            f"got {elbow.motion_limits.lower:.3f} to {elbow.motion_limits.upper:.3f}"
        ),
    )

    upper_body_aabb = ctx.part_element_world_aabb(upper_arm, elem="upper_arm_body")
    forearm_body_aabb = ctx.part_element_world_aabb(forearm, elem="forearm_body")
    plate_face_aabb = ctx.part_element_world_aabb(end_plate, elem="plate_face")
    if upper_body_aabb is not None and forearm_body_aabb is not None and plate_face_aabb is not None:
        upper_span_x = upper_body_aabb[1][0] - upper_body_aabb[0][0]
        forearm_span_x = forearm_body_aabb[1][0] - forearm_body_aabb[0][0]
        plate_span_y = plate_face_aabb[1][1] - plate_face_aabb[0][1]
        plate_span_z = plate_face_aabb[1][2] - plate_face_aabb[0][2]
        ctx.check(
            "link_proportions_read_as_arm_module",
            upper_span_x > 0.12 and forearm_span_x > 0.11 and plate_span_y > 0.06 and plate_span_z > 0.05,
            (
                "upper arm, forearm, or end plate proportions are too small to read clearly: "
                f"upper={upper_span_x:.3f}, forearm={forearm_span_x:.3f}, "
                f"plate_y={plate_span_y:.3f}, plate_z={plate_span_z:.3f}"
            ),
        )
    else:
        ctx.fail("visual_aabb_available", "could not measure one or more key visuals")

    test_poses = (
        {shoulder: math.radians(45.0), elbow: math.radians(30.0)},
        {shoulder: math.radians(-90.0), elbow: math.radians(105.0)},
        {shoulder: math.radians(120.0), elbow: math.radians(-60.0)},
    )
    for index, pose in enumerate(test_poses, start=1):
        with ctx.pose(pose):
            ctx.expect_contact(
                upper_arm,
                pedestal,
                elem_a=shoulder_hub,
                elem_b=shoulder_cheek_left,
                name=f"shoulder_left_contact_pose_{index}",
            )
            ctx.expect_contact(
                forearm,
                upper_arm,
                elem_a=elbow_hub,
                elem_b=elbow_fork_right,
                name=f"elbow_right_contact_pose_{index}",
            )
            ctx.expect_contact(
                end_plate,
                forearm,
                elem_a=mount_block,
                elem_b=wrist_block,
                name=f"end_plate_mount_contact_pose_{index}",
            )
            ctx.expect_origin_distance(
                end_plate,
                forearm,
                axes="xyz",
                min_dist=0.165,
                max_dist=0.175,
                name=f"end_plate_rigid_offset_pose_{index}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"no_overlaps_pose_{index}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
