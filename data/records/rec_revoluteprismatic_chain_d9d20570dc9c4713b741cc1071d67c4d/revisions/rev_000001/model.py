from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.180
BASE_WIDTH = 0.090
BASE_THICKNESS = 0.012

SUPPORT_WALL_THICKNESS = 0.016
SUPPORT_WALL_WIDTH = 0.068
SUPPORT_WALL_HEIGHT = 0.110
SUPPORT_WALL_CENTER_X = -0.068

BRIDGE_LENGTH = 0.060
BRIDGE_WIDTH = 0.060
BRIDGE_HEIGHT = 0.020
BRIDGE_CENTER_X = -0.030
BRIDGE_CENTER_Z = 0.122

HINGE_Z = 0.092
YOKE_LENGTH = 0.020
YOKE_WIDTH = 0.060
YOKE_HEIGHT = 0.046
YOKE_CENTER_X = 0.004
YOKE_CENTER_Z = 0.092
YOKE_SLOT_LENGTH = 0.022
YOKE_SLOT_WIDTH = 0.018
YOKE_SLOT_HEIGHT = 0.052
YOKE_SLOT_CENTER_X = 0.004
YOKE_SLOT_CENTER_Z = 0.092

STRUT_LENGTH = 0.048
STRUT_WIDTH = 0.012
STRUT_HEIGHT = 0.092
STRUT_CENTER_X = -0.042
STRUT_CENTER_Y = 0.024
STRUT_CENTER_Z = 0.072

ARM_LENGTH = 0.205
ARM_WIDTH = 0.034
ARM_HEIGHT = 0.030
ARM_LUG_LENGTH = 0.012
ARM_LUG_WIDTH = 0.018
ARM_LUG_HEIGHT = 0.022
ARM_LUG_CENTER_X = 0.006
ARM_NECK_LENGTH = 0.020
ARM_NECK_WIDTH = 0.018
ARM_NECK_HEIGHT = 0.014
ARM_NECK_CENTER_X = 0.018
ARM_RAIL_START_X = 0.022
ARM_RAIL_LENGTH = 0.183
ARM_RAIL_WIDTH = 0.008
ARM_RAIL_HEIGHT = 0.028
ARM_RAIL_CENTER_X = 0.1135
ARM_RAIL_CENTER_Y = 0.013
ARM_TOP_WEB_LENGTH = 0.150
ARM_TOP_WEB_THICKNESS = 0.008
ARM_TOP_WEB_CENTER_X = 0.115
ARM_TOP_WEB_CENTER_Z = 0.010
ARM_REAR_BRIDGE_LENGTH = 0.022
ARM_REAR_BRIDGE_CENTER_X = 0.031

TIP_RAIL_LENGTH = 0.145
TIP_RAIL_WIDTH = 0.018
TIP_THICKNESS = 0.010
TIP_RAIL_BACK_X = -0.095
TIP_FRONT_SHOULDER_X = 0.020
TIP_NOSE_TIP_X = 0.058
TIP_Z_CENTER = 0.0

ARM_HINGE_ORIGIN = (0.012, 0.0, HINGE_Z)
TIP_SLIDE_ORIGIN = (0.190, 0.0, 0.0)
ARM_MAX_ANGLE = 1.10
TIP_MAX_TRAVEL = 0.075


def _box(center_x: float, center_y: float, center_z: float, size_x: float, size_y: float, size_z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((center_x, center_y, center_z))


def _rear_support_shape() -> cq.Workplane:
    base = _box(0.0, 0.0, BASE_THICKNESS / 2.0, BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
    rear_wall = _box(
        SUPPORT_WALL_CENTER_X,
        0.0,
        BASE_THICKNESS + SUPPORT_WALL_HEIGHT / 2.0,
        SUPPORT_WALL_THICKNESS,
        SUPPORT_WALL_WIDTH,
        SUPPORT_WALL_HEIGHT,
    )
    bridge = _box(
        BRIDGE_CENTER_X,
        0.0,
        BRIDGE_CENTER_Z,
        BRIDGE_LENGTH,
        BRIDGE_WIDTH,
        BRIDGE_HEIGHT,
    )
    yoke_block = _box(
        YOKE_CENTER_X,
        0.0,
        YOKE_CENTER_Z,
        YOKE_LENGTH,
        YOKE_WIDTH,
        YOKE_HEIGHT,
    )

    left_strut = _box(
        STRUT_CENTER_X,
        STRUT_CENTER_Y,
        STRUT_CENTER_Z,
        STRUT_LENGTH,
        STRUT_WIDTH,
        STRUT_HEIGHT,
    )
    right_strut = _box(
        STRUT_CENTER_X,
        -STRUT_CENTER_Y,
        STRUT_CENTER_Z,
        STRUT_LENGTH,
        STRUT_WIDTH,
        STRUT_HEIGHT,
    )

    support = (
        base.union(rear_wall)
        .union(bridge)
        .union(yoke_block)
        .union(left_strut)
        .union(right_strut)
    )
    yoke_slot = _box(
        YOKE_SLOT_CENTER_X,
        0.0,
        YOKE_SLOT_CENTER_Z,
        YOKE_SLOT_LENGTH,
        YOKE_SLOT_WIDTH,
        YOKE_SLOT_HEIGHT,
    )
    return support.cut(yoke_slot)


def _pivot_arm_shape() -> cq.Workplane:
    hinge_lug = _box(ARM_LUG_CENTER_X, 0.0, 0.0, ARM_LUG_LENGTH, ARM_LUG_WIDTH, ARM_LUG_HEIGHT)
    neck = _box(ARM_NECK_CENTER_X, 0.0, 0.0, ARM_NECK_LENGTH, ARM_NECK_WIDTH, ARM_NECK_HEIGHT)
    left_rail = _box(
        ARM_RAIL_CENTER_X,
        ARM_RAIL_CENTER_Y,
        0.0,
        ARM_RAIL_LENGTH,
        ARM_RAIL_WIDTH,
        ARM_RAIL_HEIGHT,
    )
    right_rail = _box(
        ARM_RAIL_CENTER_X,
        -ARM_RAIL_CENTER_Y,
        0.0,
        ARM_RAIL_LENGTH,
        ARM_RAIL_WIDTH,
        ARM_RAIL_HEIGHT,
    )
    top_web = _box(
        ARM_TOP_WEB_CENTER_X,
        0.0,
        ARM_TOP_WEB_CENTER_Z,
        ARM_TOP_WEB_LENGTH,
        ARM_WIDTH,
        ARM_TOP_WEB_THICKNESS,
    )
    rear_bridge = _box(
        ARM_REAR_BRIDGE_CENTER_X,
        0.0,
        ARM_TOP_WEB_CENTER_Z,
        ARM_REAR_BRIDGE_LENGTH,
        ARM_WIDTH,
        ARM_TOP_WEB_THICKNESS,
    )
    return hinge_lug.union(neck).union(left_rail).union(right_rail).union(top_web).union(rear_bridge)


def _extension_nose_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (TIP_RAIL_BACK_X, TIP_Z_CENTER - TIP_THICKNESS / 2.0),
                (TIP_FRONT_SHOULDER_X, TIP_Z_CENTER - TIP_THICKNESS / 2.0),
                (TIP_NOSE_TIP_X - 0.014, TIP_Z_CENTER - 0.003),
                (TIP_NOSE_TIP_X, TIP_Z_CENTER),
                (TIP_NOSE_TIP_X - 0.014, TIP_Z_CENTER + 0.003),
                (TIP_FRONT_SHOULDER_X, TIP_Z_CENTER + TIP_THICKNESS / 2.0),
                (TIP_RAIL_BACK_X, TIP_Z_CENTER + TIP_THICKNESS / 2.0),
            ]
        )
        .close()
        .extrude(TIP_RAIL_WIDTH / 2.0, both=True)
    )


def _set_box_inertial(part, size: tuple[float, float, float], center: tuple[float, float, float], mass: float) -> None:
    part.inertial = Inertial.from_geometry(Box(size), mass=mass, origin=Origin(xyz=center))


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_pivot_arm")

    model.material("support_gray", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("arm_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("nose_black", rgba=(0.14, 0.15, 0.17, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.180, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="support_gray",
        name="support_base",
    )
    rear_support.visual(
        Box((0.016, 0.068, 0.110)),
        origin=Origin(xyz=(-0.068, 0.0, 0.067)),
        material="support_gray",
        name="support_upright",
    )
    rear_support.visual(
        Box((0.060, 0.060, 0.020)),
        origin=Origin(xyz=(-0.030, 0.0, 0.122)),
        material="support_gray",
        name="support_bridge",
    )
    rear_support.visual(
        Box((0.048, 0.012, 0.092)),
        origin=Origin(xyz=(-0.042, 0.024, 0.072)),
        material="support_gray",
        name="left_strut",
    )
    rear_support.visual(
        Box((0.048, 0.012, 0.092)),
        origin=Origin(xyz=(-0.042, -0.024, 0.072)),
        material="support_gray",
        name="right_strut",
    )
    rear_support.visual(
        Box((0.028, 0.010, 0.046)),
        origin=Origin(xyz=(0.006, 0.014, HINGE_Z)),
        material="support_gray",
        name="left_yoke",
    )
    rear_support.visual(
        Box((0.028, 0.010, 0.046)),
        origin=Origin(xyz=(0.006, -0.014, HINGE_Z)),
        material="support_gray",
        name="right_yoke",
    )
    _set_box_inertial(rear_support, (BASE_LENGTH, BASE_WIDTH, 0.140), (0.0, 0.0, 0.070), mass=2.8)

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Box((0.016, 0.018, 0.022)),
        origin=Origin(xyz=(0.001, 0.0, 0.0)),
        material="arm_aluminum",
        name="hinge_lug",
    )
    pivot_arm.visual(
        Box((0.018, 0.018, 0.014)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material="arm_aluminum",
        name="arm_neck",
    )
    pivot_arm.visual(
        Box((0.020, 0.034, 0.010)),
        origin=Origin(xyz=(0.027, 0.0, 0.006)),
        material="arm_aluminum",
        name="rear_bridge",
    )
    pivot_arm.visual(
        Box((0.174, 0.006, 0.030)),
        origin=Origin(xyz=(0.118, 0.014, 0.0)),
        material="arm_aluminum",
        name="left_rail",
    )
    pivot_arm.visual(
        Box((0.174, 0.006, 0.030)),
        origin=Origin(xyz=(0.118, -0.014, 0.0)),
        material="arm_aluminum",
        name="right_rail",
    )
    pivot_arm.visual(
        Box((0.174, 0.034, 0.006)),
        origin=Origin(xyz=(0.118, 0.0, 0.012)),
        material="arm_aluminum",
        name="top_web",
    )
    _set_box_inertial(pivot_arm, (0.205, 0.034, 0.030), (0.103, 0.0, 0.0), mass=1.1)

    extension_nose = model.part("extension_nose")
    extension_nose.visual(
        Box((0.120, 0.022, 0.010)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material="nose_black",
        name="tip_rail",
    )
    extension_nose.visual(
        Box((0.014, 0.022, 0.010)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material="nose_black",
        name="tip_shoulder",
    )
    extension_nose.visual(
        Box((0.040, 0.022, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material="nose_black",
        name="tip_nose",
    )
    _set_box_inertial(
        extension_nose,
        (0.174, 0.022, 0.010),
        (-0.033, 0.0, 0.0),
        mass=0.32,
    )

    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.6, lower=0.0, upper=ARM_MAX_ANGLE),
    )
    model.articulation(
        "arm_to_tip",
        ArticulationType.PRISMATIC,
        parent=pivot_arm,
        child=extension_nose,
        origin=Origin(xyz=(0.191, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.18, lower=0.0, upper=TIP_MAX_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    pivot_arm = object_model.get_part("pivot_arm")
    extension_nose = object_model.get_part("extension_nose")
    arm_hinge = object_model.get_articulation("support_to_arm")
    tip_slide = object_model.get_articulation("arm_to_tip")

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

    arm_limits = arm_hinge.motion_limits
    tip_limits = tip_slide.motion_limits
    ctx.check(
        "arm hinge configuration",
        arm_hinge.articulation_type == ArticulationType.REVOLUTE
        and arm_hinge.axis == (0.0, -1.0, 0.0)
        and arm_limits is not None
        and arm_limits.lower == 0.0
        and arm_limits.upper == ARM_MAX_ANGLE,
        details="The primary arm should be an upward-pitch revolute joint on the rear support.",
    )
    ctx.check(
        "tip slide configuration",
        tip_slide.articulation_type == ArticulationType.PRISMATIC
        and tip_slide.axis == (1.0, 0.0, 0.0)
        and tip_limits is not None
        and tip_limits.lower == 0.0
        and tip_limits.upper == TIP_MAX_TRAVEL,
        details="The short extension nose should telescope forward along the arm axis.",
    )

    ctx.expect_contact(
        pivot_arm,
        rear_support,
        contact_tol=0.001,
        name="arm barrel seats against support cheeks",
    )

    with ctx.pose({tip_slide: 0.0}):
        ctx.expect_contact(
            extension_nose,
            pivot_arm,
            contact_tol=0.001,
            name="tip shoulder seats against arm face at home",
        )
        ctx.expect_within(
            extension_nose,
            pivot_arm,
            axes="yz",
            margin=0.003,
            name="tip stays laterally guided by the arm",
        )

        arm_aabb = ctx.part_world_aabb(pivot_arm)
        tip_aabb = ctx.part_world_aabb(extension_nose)
        protrusion_ok = False
        if arm_aabb is not None and tip_aabb is not None:
            arm_max_x = arm_aabb[1][0]
            tip_min_x = tip_aabb[0][0]
            tip_max_x = tip_aabb[1][0]
            protrusion = tip_max_x - arm_max_x
            inserted_length = arm_max_x - tip_min_x
            protrusion_ok = 0.035 <= protrusion <= 0.060 and inserted_length >= 0.090
        ctx.check(
            "tip is short and telescopically nested at home",
            protrusion_ok,
            details="The extension nose should protrude slightly while still remaining substantially inserted in the arm.",
        )

    with ctx.pose({arm_hinge: 0.0, tip_slide: 0.0}):
        closed_tip_pos = ctx.part_world_position(extension_nose)
    with ctx.pose({arm_hinge: 0.85, tip_slide: 0.0}):
        raised_tip_pos = ctx.part_world_position(extension_nose)

    lift_ok = False
    if closed_tip_pos is not None and raised_tip_pos is not None:
        lift_ok = raised_tip_pos[2] > closed_tip_pos[2] + 0.120 and raised_tip_pos[0] < closed_tip_pos[0] - 0.050
    ctx.check(
        "arm hinge raises the nose",
        lift_ok,
        details="Positive arm motion should pitch the forward arm section upward from the rear support.",
    )

    with ctx.pose({arm_hinge: 0.45, tip_slide: 0.0}):
        retracted_tip_pos = ctx.part_world_position(extension_nose)
    with ctx.pose({arm_hinge: 0.45, tip_slide: TIP_MAX_TRAVEL}):
        extended_tip_pos = ctx.part_world_position(extension_nose)

    slide_ok = False
    if retracted_tip_pos is not None and extended_tip_pos is not None:
        dx = extended_tip_pos[0] - retracted_tip_pos[0]
        dy = extended_tip_pos[1] - retracted_tip_pos[1]
        dz = extended_tip_pos[2] - retracted_tip_pos[2]
        travel = _distance(extended_tip_pos, retracted_tip_pos)
        slide_ok = 0.073 <= travel <= 0.077 and dx > 0.060 and abs(dy) < 1e-4 and dz > 0.025
    ctx.check(
        "tip slide extends along the pitched arm axis",
        slide_ok,
        details="At a raised arm pose the prismatic stage should move the nose forward along the arm, not sideways.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
