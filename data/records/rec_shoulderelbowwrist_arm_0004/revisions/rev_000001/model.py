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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

SHOULDER_Z = 0.245
UPPER_ARM_LENGTH = 0.250
FOREARM_LENGTH = 0.200

SHOULDER_HUB_RADIUS = 0.028
SHOULDER_HUB_LENGTH = 0.038
SHOULDER_PLATE_THICKNESS = 0.015
SHOULDER_PLATE_CENTER_Y = (SHOULDER_HUB_LENGTH + SHOULDER_PLATE_THICKNESS) / 2.0

ELBOW_HUB_RADIUS = 0.022
ELBOW_HUB_LENGTH = 0.030
ELBOW_PLATE_THICKNESS = 0.012
ELBOW_PLATE_CENTER_Y = (ELBOW_HUB_LENGTH + ELBOW_PLATE_THICKNESS) / 2.0

WRIST_HUB_RADIUS = 0.015
WRIST_HUB_LENGTH = 0.022
WRIST_PLATE_THICKNESS = 0.010
WRIST_PLATE_CENTER_Y = (WRIST_HUB_LENGTH + WRIST_PLATE_THICKNESS) / 2.0

SHOULDER_LIMITS = MotionLimits(
    effort=120.0,
    velocity=1.5,
    lower=math.radians(-120.0),
    upper=math.radians(135.0),
)
ELBOW_LIMITS = MotionLimits(
    effort=90.0,
    velocity=1.8,
    lower=math.radians(-120.0),
    upper=math.radians(135.0),
)
WRIST_LIMITS = MotionLimits(
    effort=35.0,
    velocity=2.5,
    lower=math.radians(-90.0),
    upper=math.radians(90.0),
)


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(low, high))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_elbow_wrist_arm", assets=ASSETS)

    dark_base = model.material("dark_base", rgba=(0.20, 0.22, 0.25, 1.0))
    painted_arm = model.material("painted_arm", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_joint = model.material("dark_joint", rgba=(0.16, 0.17, 0.19, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_base,
        name="base_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=dark_base,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.090, 0.060, 0.047)),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z - SHOULDER_HUB_RADIUS - 0.0235)),
        material=dark_base,
        name="shoulder_carrier",
    )
    pedestal.visual(
        Box((0.055, SHOULDER_PLATE_THICKNESS, 0.090)),
        origin=Origin(xyz=(0.0, SHOULDER_PLATE_CENTER_Y, SHOULDER_Z)),
        material=dark_joint,
        name="shoulder_plate_left",
    )
    pedestal.visual(
        Box((0.055, SHOULDER_PLATE_THICKNESS, 0.090)),
        origin=Origin(xyz=(0.0, -SHOULDER_PLATE_CENTER_Y, SHOULDER_Z)),
        material=dark_joint,
        name="shoulder_plate_right",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=dark_joint,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.070, 0.036, 0.055)),
        origin=Origin(xyz=(0.063, 0.0, 0.0)),
        material=painted_arm,
        name="shoulder_cheek",
    )
    upper_arm.visual(
        Box((0.160, 0.050, 0.045)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=painted_arm,
        name="arm_beam",
    )
    upper_arm.visual(
        Box((0.040, ELBOW_PLATE_THICKNESS, 0.065)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, ELBOW_PLATE_CENTER_Y, 0.0)),
        material=dark_joint,
        name="elbow_plate_left",
    )
    upper_arm.visual(
        Box((0.040, ELBOW_PLATE_THICKNESS, 0.065)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, -ELBOW_PLATE_CENTER_Y, 0.0)),
        material=dark_joint,
        name="elbow_plate_right",
    )
    upper_arm.visual(
        Box((0.045, 0.042, 0.016)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.005, 0.0, -0.030)),
        material=dark_joint,
        name="elbow_bridge",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=ELBOW_HUB_RADIUS, length=ELBOW_HUB_LENGTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=dark_joint,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.050, 0.028, 0.044)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=painted_arm,
        name="elbow_cheek",
    )
    forearm.visual(
        Box((0.135, 0.040, 0.038)),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=painted_arm,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.030, WRIST_PLATE_THICKNESS, 0.050)),
        origin=Origin(xyz=(FOREARM_LENGTH, WRIST_PLATE_CENTER_Y, 0.0)),
        material=dark_joint,
        name="wrist_plate_left",
    )
    forearm.visual(
        Box((0.030, WRIST_PLATE_THICKNESS, 0.050)),
        origin=Origin(xyz=(FOREARM_LENGTH, -WRIST_PLATE_CENTER_Y, 0.0)),
        material=dark_joint,
        name="wrist_plate_right",
    )
    forearm.visual(
        Box((0.032, 0.032, 0.012)),
        origin=Origin(xyz=(FOREARM_LENGTH - 0.006, 0.0, -0.021)),
        material=dark_joint,
        name="wrist_bridge",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=WRIST_HUB_RADIUS, length=WRIST_HUB_LENGTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=dark_joint,
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.056, 0.028, 0.028)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=painted_arm,
        name="wrist_beam",
    )
    wrist.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=_x_cylinder_origin((0.080, 0.0, 0.0)),
        material=tool_steel,
        name="tool_flange",
    )
    wrist.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=_x_cylinder_origin((0.096, 0.0, 0.0)),
        material=tool_steel,
        name="tool_spigot",
    )

    model.articulation(
        "pedestal_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=SHOULDER_LIMITS,
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=ELBOW_LIMITS,
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=WRIST_LIMITS,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder = object_model.get_articulation("pedestal_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    shoulder_carrier = pedestal.get_visual("shoulder_carrier")
    shoulder_plate_left = pedestal.get_visual("shoulder_plate_left")
    shoulder_plate_right = pedestal.get_visual("shoulder_plate_right")
    shoulder_hub = upper_arm.get_visual("shoulder_hub")

    elbow_plate_left = upper_arm.get_visual("elbow_plate_left")
    elbow_plate_right = upper_arm.get_visual("elbow_plate_right")
    elbow_bridge = upper_arm.get_visual("elbow_bridge")
    elbow_hub = forearm.get_visual("elbow_hub")

    wrist_plate_left = forearm.get_visual("wrist_plate_left")
    wrist_plate_right = forearm.get_visual("wrist_plate_right")
    wrist_bridge = forearm.get_visual("wrist_bridge")
    wrist_hub = wrist.get_visual("wrist_hub")
    tool_flange = wrist.get_visual("tool_flange")

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
        elem_b=shoulder_carrier,
        name="shoulder_hub_seated_on_carrier",
    )
    ctx.expect_gap(
        pedestal,
        upper_arm,
        axis="y",
        positive_elem=shoulder_plate_left,
        negative_elem=shoulder_hub,
        min_gap=0.0,
        max_gap=0.0,
        name="shoulder_left_plate_clamps_hub",
    )
    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="y",
        positive_elem=shoulder_hub,
        negative_elem=shoulder_plate_right,
        min_gap=0.0,
        max_gap=0.0,
        name="shoulder_right_plate_clamps_hub",
    )

    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=elbow_hub,
        elem_b=elbow_bridge,
        name="elbow_hub_seated_on_bridge",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem=elbow_plate_left,
        negative_elem=elbow_hub,
        min_gap=-1.0e-6,
        max_gap=1.0e-6,
        name="elbow_left_plate_clamps_hub",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem=elbow_hub,
        negative_elem=elbow_plate_right,
        min_gap=-1.0e-6,
        max_gap=1.0e-6,
        name="elbow_right_plate_clamps_hub",
    )

    ctx.expect_contact(
        wrist,
        forearm,
        elem_a=wrist_hub,
        elem_b=wrist_bridge,
        name="wrist_hub_seated_on_bridge",
    )
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        positive_elem=wrist_plate_left,
        negative_elem=wrist_hub,
        min_gap=0.0,
        max_gap=0.0,
        name="wrist_left_plate_clamps_hub",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        positive_elem=wrist_hub,
        negative_elem=wrist_plate_right,
        min_gap=0.0,
        max_gap=0.0,
        name="wrist_right_plate_clamps_hub",
    )

    ctx.expect_origin_gap(
        upper_arm,
        pedestal,
        axis="z",
        min_gap=SHOULDER_Z - 0.001,
        max_gap=SHOULDER_Z + 0.001,
        name="shoulder_axis_height",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_joint: 0.0}):
        ctx.expect_origin_gap(
            forearm,
            upper_arm,
            axis="x",
            min_gap=UPPER_ARM_LENGTH - 0.001,
            max_gap=UPPER_ARM_LENGTH + 0.001,
            name="elbow_axis_at_upper_arm_tip",
        )
        ctx.expect_origin_gap(
            wrist,
            forearm,
            axis="x",
            min_gap=FOREARM_LENGTH - 0.001,
            max_gap=FOREARM_LENGTH + 0.001,
            name="wrist_axis_at_forearm_tip",
        )
        ctx.expect_overlap(
            wrist,
            forearm,
            axes="yz",
            min_overlap=0.020,
            name="wrist_hub_aligned_with_forearm_yoke",
        )

    rest_forearm_pos = ctx.part_world_position(forearm)
    rest_wrist_pos = ctx.part_world_position(wrist)

    with ctx.pose({shoulder: math.radians(-60.0), elbow: 0.0, wrist_joint: 0.0}):
        raised_forearm_pos = ctx.part_world_position(forearm)
        ctx.check(
            "shoulder_rotation_lifts_forearm",
            rest_forearm_pos is not None
            and raised_forearm_pos is not None
            and raised_forearm_pos[2] > rest_forearm_pos[2] + 0.20,
            details=f"rest={rest_forearm_pos}, raised={raised_forearm_pos}",
        )

    with ctx.pose({shoulder: 0.0, elbow: math.radians(-60.0), wrist_joint: 0.0}):
        bent_wrist_pos = ctx.part_world_position(wrist)
        ctx.check(
            "elbow_rotation_lifts_wrist_origin",
            rest_wrist_pos is not None
            and bent_wrist_pos is not None
            and bent_wrist_pos[2] > rest_wrist_pos[2] + 0.15,
            details=f"rest={rest_wrist_pos}, bent={bent_wrist_pos}",
        )

    rest_flange_aabb = ctx.part_element_world_aabb(wrist, elem=tool_flange)
    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist_joint: math.radians(-45.0)}):
        pitched_flange_aabb = ctx.part_element_world_aabb(wrist, elem=tool_flange)
        rest_flange_center = _aabb_center(rest_flange_aabb)
        pitched_flange_center = _aabb_center(pitched_flange_aabb)
        ctx.check(
            "wrist_rotation_moves_tool_flange",
            rest_flange_center is not None
            and pitched_flange_center is not None
            and pitched_flange_center[2] > rest_flange_center[2] + 0.035,
            details=f"rest={rest_flange_center}, pitched={pitched_flange_center}",
        )

    axes_ok = (
        tuple(shoulder.axis) == (0.0, 1.0, 0.0)
        and tuple(elbow.axis) == (0.0, 1.0, 0.0)
        and tuple(wrist_joint.axis) == (0.0, 1.0, 0.0)
    )
    ctx.check("joint_axes_parallel_and_horizontal", axes_ok, details="expected all joint axes to be +Y")

    limits_ok = (
        shoulder.motion_limits.lower <= math.radians(-119.0)
        and shoulder.motion_limits.upper >= math.radians(134.0)
        and elbow.motion_limits.lower <= math.radians(-119.0)
        and elbow.motion_limits.upper >= math.radians(134.0)
        and wrist_joint.motion_limits.lower <= math.radians(-89.0)
        and wrist_joint.motion_limits.upper >= math.radians(89.0)
    )
    ctx.check("joint_ranges_match_brief", limits_ok, details="joint limits do not match requested travel")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
