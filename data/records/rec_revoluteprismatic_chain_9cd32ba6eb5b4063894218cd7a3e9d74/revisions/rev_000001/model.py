from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SUPPORT_PLATE_LEN = 0.34
SUPPORT_PLATE_W = 0.12
SUPPORT_PLATE_T = 0.018
SUPPORT_PLATE_CENTER_Z = 0.037

CHEEK_DEPTH = 0.040
CHEEK_T = 0.014
CHEEK_GAP = 0.030
CHEEK_CENTER_Y = CHEEK_GAP / 2.0 + CHEEK_T / 2.0
CHEEK_H = 0.052
CHEEK_CENTER_Z = 0.008

HINGE_BARREL_R = 0.012
HINGE_BARREL_LEN = CHEEK_GAP

ARM_BAR_X = 0.010
ARM_RAIL_W = 0.004
ARM_RAIL_Y = 0.011
ARM_BAR_X_OFFSET = 0.013
ARM_HUB_X = 0.024
ARM_HUB_Z = 0.024
ARM_HUB_CENTER_Z = -0.018
ARM_TIP_BOTTOM_Z = -0.255
ARM_BAR_Z = 0.225
ARM_BAR_CENTER_Z = (ARM_TIP_BOTTOM_Z - 0.030) / 2.0
ARM_BRACE_X = 0.036
ARM_BRACE_Z = 0.010
ARM_BRACE_CENTER_Z = -0.200

SLIDER_STEM_X = 0.012
SLIDER_STEM_Y = 0.018
SLIDER_STEM_Z = 0.045
SLIDER_COLLAR_X = 0.036
SLIDER_COLLAR_Y = 0.022
SLIDER_COLLAR_Z = 0.008
SLIDER_ROD_R = 0.008
SLIDER_ROD_Z = 0.022
SLIDER_FOOT_X = 0.030
SLIDER_FOOT_Y = 0.022
SLIDER_FOOT_Z = 0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_hinge_slide_unit")

    dark_coat = model.material("dark_coat", color=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", color=(0.68, 0.70, 0.73, 1.0))
    blackened = model.material("blackened", color=(0.12, 0.12, 0.13, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((SUPPORT_PLATE_LEN, SUPPORT_PLATE_W, SUPPORT_PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_PLATE_CENTER_Z)),
        material=dark_coat,
        name="plate",
    )
    top_support.visual(
        Box((CHEEK_DEPTH, CHEEK_T, CHEEK_H)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, CHEEK_CENTER_Z)),
        material=dark_coat,
        name="left_cheek",
    )
    top_support.visual(
        Box((CHEEK_DEPTH, CHEEK_T, CHEEK_H)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, CHEEK_CENTER_Z)),
        material=dark_coat,
        name="right_cheek",
    )
    top_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_PLATE_LEN, SUPPORT_PLATE_W, 0.08)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    hanging_arm = model.part("hanging_arm")
    hanging_arm.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    hanging_arm.visual(
        Box((ARM_BAR_X, ARM_RAIL_W, ARM_BAR_Z)),
        origin=Origin(xyz=(ARM_BAR_X_OFFSET, ARM_RAIL_Y, ARM_BAR_CENTER_Z)),
        material=steel,
        name="left_rail",
    )
    hanging_arm.visual(
        Box((ARM_BAR_X, ARM_RAIL_W, ARM_BAR_Z)),
        origin=Origin(xyz=(ARM_BAR_X_OFFSET, -ARM_RAIL_Y, ARM_BAR_CENTER_Z)),
        material=steel,
        name="right_rail",
    )
    hanging_arm.visual(
        Box((ARM_HUB_X, 0.026, ARM_HUB_Z)),
        origin=Origin(xyz=(ARM_BAR_X_OFFSET / 2.0, 0.0, ARM_HUB_CENTER_Z)),
        material=steel,
        name="shoulder",
    )
    hanging_arm.visual(
        Box((0.040, 0.028, 0.020)),
        origin=Origin(xyz=(ARM_BAR_X_OFFSET, 0.0, -0.215)),
        material=steel,
        name="guide_block",
    )
    hanging_arm.inertial = Inertial.from_geometry(
        Box((0.05, 0.03, 0.27)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
    )

    tip_slider = model.part("tip_slider")
    tip_slider.visual(
        Box((SLIDER_STEM_X, SLIDER_STEM_Y, SLIDER_STEM_Z)),
        origin=Origin(xyz=(0.0, 0.0, -SLIDER_STEM_Z / 2.0)),
        material=blackened,
        name="stem",
    )
    tip_slider.visual(
        Box((SLIDER_COLLAR_X, SLIDER_COLLAR_Y, SLIDER_COLLAR_Z)),
        origin=Origin(xyz=(0.0, 0.0, -SLIDER_COLLAR_Z / 2.0)),
        material=blackened,
        name="collar",
    )
    tip_slider.visual(
        Cylinder(radius=SLIDER_ROD_R, length=SLIDER_ROD_Z),
        origin=Origin(xyz=(0.0, 0.0, -(SLIDER_STEM_Z + SLIDER_ROD_Z / 2.0))),
        material=blackened,
        name="lower_rod",
    )
    tip_slider.visual(
        Box((SLIDER_FOOT_X, SLIDER_FOOT_Y, SLIDER_FOOT_Z)),
        origin=Origin(
            xyz=(0.0, 0.0, -(SLIDER_STEM_Z + SLIDER_ROD_Z + SLIDER_FOOT_Z / 2.0))
        ),
        material=blackened,
        name="foot",
    )
    tip_slider.inertial = Inertial.from_geometry(
        Box((SLIDER_FOOT_X, SLIDER_FOOT_Y, 0.09)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=hanging_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.15,
        ),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=hanging_arm,
        child=tip_slider,
        origin=Origin(xyz=(ARM_BAR_X_OFFSET, 0.0, -0.225)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.10,
            lower=0.0,
            upper=0.04,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    hanging_arm = object_model.get_part("hanging_arm")
    tip_slider = object_model.get_part("tip_slider")
    arm_joint = object_model.get_articulation("support_to_arm")
    slider_joint = object_model.get_articulation("arm_to_slider")

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
        "hinge articulation axis is transverse",
        arm_joint.axis == (0.0, -1.0, 0.0),
        f"expected transverse hinge axis (0,-1,0), got {arm_joint.axis}",
    )
    ctx.check(
        "tip articulation axis is a downward slider",
        slider_joint.axis == (0.0, 0.0, -1.0),
        f"expected slider axis (0,0,-1), got {slider_joint.axis}",
    )
    ctx.expect_contact(
        hanging_arm,
        top_support,
        elem_a="hub",
        elem_b="left_cheek",
        name="hanging arm is physically carried by the top support",
    )
    ctx.expect_contact(
        tip_slider,
        hanging_arm,
        elem_a="collar",
        elem_b="guide_block",
        name="tip slider seats against the arm guide in the retracted pose",
    )

    rest_slider_pos = ctx.part_world_position(tip_slider)
    with ctx.pose({arm_joint: 0.75}):
        swung_slider_pos = ctx.part_world_position(tip_slider)
    ctx.check(
        "positive hinge motion swings the arm forward",
        rest_slider_pos is not None
        and swung_slider_pos is not None
        and swung_slider_pos[0] > rest_slider_pos[0] + 0.08,
        (
            "expected positive hinge motion to move the slider tip forward in +X; "
            f"rest={rest_slider_pos}, swung={swung_slider_pos}"
        ),
    )

    with ctx.pose({slider_joint: 0.03}):
        extended_slider_pos = ctx.part_world_position(tip_slider)
    ctx.check(
        "positive prismatic motion extends the tip downward",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[2] < rest_slider_pos[2] - 0.02,
        (
            "expected positive slider motion to lower the tip along -Z; "
            f"rest={rest_slider_pos}, extended={extended_slider_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
