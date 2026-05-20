from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_SIZE = 0.26
BASE_THICKNESS = 0.024
BASE_COLLAR_RADIUS = 0.042
BASE_COLLAR_HEIGHT = 0.05
MAST_RADIUS = 0.024
MAST_HEIGHT = 0.92
TOP_CAP_RADIUS = 0.032
TOP_CAP_HEIGHT = 0.014

HUB_HEIGHT = 0.058
HUB_BODY_DEPTH = 0.055
HUB_BODY_WIDTH = 0.07
HUB_BODY_CENTER_X = HUB_BODY_DEPTH / 2.0
HUB_NECK_LENGTH = 0.022
HUB_NECK_CENTER_X = HUB_BODY_DEPTH + HUB_NECK_LENGTH / 2.0
HUB_NECK_WIDTH = 0.036
HUB_EAR_LENGTH = 0.018
PIVOT_X = HUB_BODY_DEPTH + HUB_NECK_LENGTH + HUB_EAR_LENGTH / 2.0
HUB_EAR_CENTER_X = PIVOT_X
HUB_EAR_WIDTH = 0.012
HUB_EAR_HEIGHT = 0.048
HUB_EAR_OFFSET_Y = 0.016

ARM_ROOT_LENGTH = 0.018
ARM_ROOT_WIDTH = 0.02
ARM_ROOT_HEIGHT = 0.036
ARM_ROOT_CENTER_X = HUB_EAR_LENGTH / 2.0 + ARM_ROOT_LENGTH / 2.0
ARM_BAR_LENGTH = 0.148
ARM_BAR_WIDTH = 0.014
ARM_BAR_THICKNESS = 0.012
ARM_TIP_RADIUS = 0.012
ARM_BAR_CENTER_X = ARM_ROOT_CENTER_X + ARM_ROOT_LENGTH / 2.0 + ARM_BAR_LENGTH / 2.0
ARM_TIP_X = ARM_ROOT_CENTER_X + ARM_ROOT_LENGTH / 2.0 + ARM_BAR_LENGTH

HUB_ZS = (0.25, 0.47, 0.69)
HUB_YAWS = (0.0, 2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_motion_rig")

    mast_finish = model.material("mast_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.32, 0.34, 0.37, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.80, 0.47, 0.16, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((BASE_SIZE, BASE_SIZE, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=mast_finish,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=BASE_COLLAR_RADIUS, length=BASE_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_COLLAR_HEIGHT / 2.0)),
        material=mast_finish,
        name="base_collar",
    )
    mast.visual(
        Cylinder(radius=MAST_RADIUS, length=MAST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + MAST_HEIGHT / 2.0)),
        material=mast_finish,
        name="mast_shaft",
    )
    mast.visual(
        Cylinder(radius=TOP_CAP_RADIUS, length=TOP_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + MAST_HEIGHT - TOP_CAP_HEIGHT / 2.0)),
        material=mast_finish,
        name="top_cap",
    )

    hub_names = ("hub_lower", "hub_middle", "hub_upper")
    arm_names = ("arm_lower", "arm_middle", "arm_upper")

    for index, (hub_name, arm_name, hub_z, yaw) in enumerate(
        zip(hub_names, arm_names, HUB_ZS, HUB_YAWS)
    ):
        hub = model.part(hub_name)
        hub.visual(
            Box((HUB_BODY_DEPTH, HUB_BODY_WIDTH, HUB_HEIGHT)),
            origin=Origin(xyz=(HUB_BODY_CENTER_X, 0.0, 0.0)),
            material=hub_finish,
            name="hub_body",
        )
        hub.visual(
            Box((HUB_NECK_LENGTH, HUB_NECK_WIDTH, HUB_HEIGHT)),
            origin=Origin(xyz=(HUB_NECK_CENTER_X, 0.0, 0.0)),
            material=hub_finish,
            name="hub_neck",
        )
        hub.visual(
            Box((HUB_EAR_LENGTH, HUB_EAR_WIDTH, HUB_EAR_HEIGHT)),
            origin=Origin(xyz=(HUB_EAR_CENTER_X, HUB_EAR_OFFSET_Y, 0.0)),
            material=hub_finish,
            name="upper_ear",
        )
        hub.visual(
            Box((HUB_EAR_LENGTH, HUB_EAR_WIDTH, HUB_EAR_HEIGHT)),
            origin=Origin(xyz=(HUB_EAR_CENTER_X, -HUB_EAR_OFFSET_Y, 0.0)),
            material=hub_finish,
            name="lower_ear",
        )
        model.articulation(
            f"mast_to_{hub_name}",
            ArticulationType.FIXED,
            parent=mast,
            child=hub,
            origin=Origin(
                xyz=(MAST_RADIUS * math.cos(yaw), MAST_RADIUS * math.sin(yaw), hub_z),
                rpy=(0.0, 0.0, yaw),
            ),
        )

        arm = model.part(arm_name)
        arm.visual(
            Box((ARM_ROOT_LENGTH, ARM_ROOT_WIDTH, ARM_ROOT_HEIGHT)),
            origin=Origin(xyz=(ARM_ROOT_CENTER_X, 0.0, 0.0)),
            material=arm_finish,
            name="arm_root",
        )
        arm.visual(
            Box((ARM_BAR_LENGTH, ARM_BAR_WIDTH, ARM_BAR_THICKNESS)),
            origin=Origin(xyz=(ARM_BAR_CENTER_X, 0.0, 0.0)),
            material=arm_finish,
            name="arm_bar",
        )
        arm.visual(
            Cylinder(radius=ARM_TIP_RADIUS, length=ARM_BAR_WIDTH),
            origin=Origin(xyz=(ARM_TIP_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=arm_finish,
            name="arm_tip",
        )
        model.articulation(
            f"{hub_name}_to_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=arm,
            origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=2.0,
                lower=-0.55,
                upper=1.1,
            ),
            meta={"branch_index": index},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    hub_lower = object_model.get_part("hub_lower")
    hub_middle = object_model.get_part("hub_middle")
    hub_upper = object_model.get_part("hub_upper")
    arm_lower = object_model.get_part("arm_lower")
    arm_middle = object_model.get_part("arm_middle")
    arm_upper = object_model.get_part("arm_upper")
    lower_joint = object_model.get_articulation("hub_lower_to_arm_lower")
    middle_joint = object_model.get_articulation("hub_middle_to_arm_middle")
    upper_joint = object_model.get_articulation("hub_upper_to_arm_upper")

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

    ctx.expect_contact(hub_lower, mast, name="lower_hub_is_mounted_to_mast")
    ctx.expect_contact(hub_middle, mast, name="middle_hub_is_mounted_to_mast")
    ctx.expect_contact(hub_upper, mast, name="upper_hub_is_mounted_to_mast")

    ctx.expect_contact(arm_lower, hub_lower, name="lower_arm_is_supported_by_hub")
    ctx.expect_contact(arm_middle, hub_middle, name="middle_arm_is_supported_by_hub")
    ctx.expect_contact(arm_upper, hub_upper, name="upper_arm_is_supported_by_hub")

    def _aabb_center(part_obj):
        bounds = ctx.part_world_aabb(part_obj)
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((a + b) / 2.0 for a, b in zip(lower, upper))

    with ctx.pose({lower_joint: 0.0}):
        lower_rest = _aabb_center(arm_lower)
        middle_rest = _aabb_center(arm_middle)
    with ctx.pose({lower_joint: 0.9}):
        lower_lifted = _aabb_center(arm_lower)
        middle_unchanged = _aabb_center(arm_middle)

    ctx.check(
        "lower_arm_positive_motion_lifts_outward",
        lower_rest is not None
        and lower_lifted is not None
        and lower_lifted[2] > lower_rest[2] + 0.05,
        details=f"rest={lower_rest}, lifted={lower_lifted}",
    )
    ctx.check(
        "lower_joint_motion_is_independent",
        middle_rest is not None
        and middle_unchanged is not None
        and max(abs(a - b) for a, b in zip(middle_rest, middle_unchanged)) <= 1e-6,
        details=f"middle_rest={middle_rest}, middle_with_lower_motion={middle_unchanged}",
    )

    def _lift_check(name: str, arm, joint) -> None:
        with ctx.pose({joint: 0.0}):
            rest = _aabb_center(arm)
        with ctx.pose({joint: 0.9}):
            lifted = _aabb_center(arm)
        ctx.check(
            name,
            rest is not None and lifted is not None and lifted[2] > rest[2] + 0.05,
            details=f"rest={rest}, lifted={lifted}",
        )

    _lift_check("middle_arm_positive_motion_lifts_outward", arm_middle, middle_joint)
    _lift_check("upper_arm_positive_motion_lifts_outward", arm_upper, upper_joint)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
