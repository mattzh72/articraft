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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_W = 0.32
BASE_D = 0.25
BASE_H = 0.03

PEDESTAL_W = 0.20
PEDESTAL_D = 0.16
PEDESTAL_H = 0.16

MAST_W = 0.14
MAST_D = 0.10
MAST_H = 1.18
MAST_WALL = 0.018
MAST_SLOT_BOTTOM = 0.20
MAST_SLOT_TOP = 1.08

FRONT_RAIL_W = 0.076
FRONT_RAIL_D = 0.014
FRONT_RAIL_H = MAST_SLOT_TOP - MAST_SLOT_BOTTOM
FRONT_RAIL_Y = MAST_D / 2.0 + FRONT_RAIL_D / 2.0
FRONT_RAIL_Z = (MAST_SLOT_BOTTOM + MAST_SLOT_TOP) / 2.0
FRONT_RAIL_FOOT_H = 0.022
FRONT_RAIL_FOOT_Z = MAST_SLOT_BOTTOM - FRONT_RAIL_FOOT_H / 2.0 + 0.001

CARR_BODY_W = 0.16
CARR_BODY_D = 0.04
CARR_BODY_Y = 0.108
CARR_H = 0.16

GUIDE_T = 0.012
GUIDE_D = 0.03
GUIDE_H = 0.14
GUIDE_X = FRONT_RAIL_W / 2.0 + GUIDE_T / 2.0
GUIDE_Y = FRONT_RAIL_Y

PAD_W = 0.076
PAD_D = 0.024
PAD_H = 0.11
PAD_Y = FRONT_RAIL_Y + FRONT_RAIL_D / 2.0 + PAD_D / 2.0

EAR_T = 0.016
EAR_D = 0.016
EAR_H = 0.074
EAR_X = 0.05
POST_D = 0.012
POST_Y = CARR_BODY_Y + CARR_BODY_D / 2.0 + POST_D / 2.0
EAR_Y = POST_Y + POST_D / 2.0 + EAR_D / 2.0
EAR_Z = 0.014

HUB_R = 0.01
DISC_T = 0.004
DISC_X = EAR_X - EAR_T / 2.0 - DISC_T / 2.0
HUB_L = 2.0 * (DISC_X - DISC_T / 2.0)
NOSE_W = 0.052
NOSE_ARM_D = 0.10
NOSE_ARM_H = 0.028
NOSE_ARM_Y = 0.064
NOSE_SPINE_W = 0.038
NOSE_SPINE_D = 0.014
NOSE_SPINE_H = 0.024
NOSE_SPINE_Y = NOSE_SPINE_D / 2.0
NOSE_TIP_W = 0.038
NOSE_TIP_D = 0.028
NOSE_TIP_H = 0.05
NOSE_TIP_Y = 0.112
NOSE_TIP_Z = -0.015
NOSE_RIB_D = 0.016
NOSE_RIB_H = 0.018
NOSE_RIB_Y = 0.04
NOSE_RIB_Z = 0.012

LIFT_ZERO_Z = 0.30
LIFT_TRAVEL = 0.72
WRIST_ZERO = 0.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_lift_column")

    mast_gray = model.material("mast_gray", rgba=(0.27, 0.29, 0.32, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.54, 0.56, 0.58, 1.0))
    nose_orange = model.material("nose_orange", rgba=(0.88, 0.53, 0.18, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((BASE_W, BASE_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=mast_gray,
        name="base_plate",
    )
    mast.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + PEDESTAL_H / 2.0)),
        material=mast_gray,
        name="pedestal",
    )
    mast.visual(
        Box((MAST_WALL, MAST_D, MAST_H)),
        origin=Origin(
            xyz=(-(MAST_W / 2.0 - MAST_WALL / 2.0), 0.0, BASE_H + MAST_H / 2.0)
        ),
        material=mast_gray,
        name="left_upright",
    )
    mast.visual(
        Box((MAST_WALL, MAST_D, MAST_H)),
        origin=Origin(
            xyz=((MAST_W / 2.0 - MAST_WALL / 2.0), 0.0, BASE_H + MAST_H / 2.0)
        ),
        material=mast_gray,
        name="right_upright",
    )
    mast.visual(
        Box((MAST_W - 2.0 * MAST_WALL, MAST_WALL, MAST_H)),
        origin=Origin(
            xyz=(0.0, -(MAST_D / 2.0 - MAST_WALL / 2.0), BASE_H + MAST_H / 2.0)
        ),
        material=mast_gray,
        name="back_web",
    )
    mast.visual(
        Box((FRONT_RAIL_W, FRONT_RAIL_D, FRONT_RAIL_H)),
        origin=Origin(xyz=(0.0, FRONT_RAIL_Y, FRONT_RAIL_Z)),
        material=mast_gray,
        name="front_rail",
    )
    mast.visual(
        Box((FRONT_RAIL_W, FRONT_RAIL_D, FRONT_RAIL_FOOT_H)),
        origin=Origin(xyz=(0.0, FRONT_RAIL_Y, FRONT_RAIL_FOOT_Z)),
        material=mast_gray,
        name="front_rail_foot",
    )
    mast.visual(
        Box((MAST_W * 0.92, MAST_D * 0.92, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + MAST_H + 0.009)),
        material=mast_gray,
        name="top_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((GUIDE_T, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, 0.0)),
        material=carriage_gray,
        name="left_guide",
    )
    carriage.visual(
        Box((GUIDE_T, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, 0.0)),
        material=carriage_gray,
        name="right_guide",
    )
    carriage.visual(
        Box((PAD_W, PAD_D, PAD_H)),
        origin=Origin(xyz=(0.0, PAD_Y, 0.0)),
        material=carriage_gray,
        name="center_pad",
    )
    carriage.visual(
        Box((CARR_BODY_W, CARR_BODY_D, CARR_H)),
        origin=Origin(xyz=(0.0, CARR_BODY_Y, 0.0)),
        material=carriage_gray,
        name="carriage_body",
    )
    carriage.visual(
        Box((EAR_T, POST_D, 0.092)),
        origin=Origin(xyz=(-EAR_X, POST_Y, 0.0)),
        material=carriage_gray,
        name="left_post",
    )
    carriage.visual(
        Box((EAR_T, POST_D, 0.092)),
        origin=Origin(xyz=(EAR_X, POST_Y, 0.0)),
        material=carriage_gray,
        name="right_post",
    )
    carriage.visual(
        Box((EAR_T, EAR_D, EAR_H)),
        origin=Origin(xyz=(-EAR_X, EAR_Y, EAR_Z)),
        material=carriage_gray,
        name="left_ear",
    )
    carriage.visual(
        Box((EAR_T, EAR_D, EAR_H)),
        origin=Origin(xyz=(EAR_X, EAR_Y, EAR_Z)),
        material=carriage_gray,
        name="right_ear",
    )

    nose_bracket = model.part("nose_bracket")
    nose_bracket.visual(
        Cylinder(radius=HUB_R, length=HUB_L),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_orange,
        name="hinge_shaft",
    )
    nose_bracket.visual(
        Cylinder(radius=HUB_R * 1.15, length=DISC_T),
        origin=Origin(xyz=(-DISC_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_orange,
        name="left_disc",
    )
    nose_bracket.visual(
        Cylinder(radius=HUB_R * 1.15, length=DISC_T),
        origin=Origin(xyz=(DISC_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_orange,
        name="right_disc",
    )
    nose_bracket.visual(
        Box((NOSE_SPINE_W, NOSE_SPINE_D, NOSE_SPINE_H)),
        origin=Origin(xyz=(0.0, NOSE_SPINE_Y, 0.0)),
        material=nose_orange,
        name="nose_spine",
    )
    nose_bracket.visual(
        Box((NOSE_W, NOSE_ARM_D, NOSE_ARM_H)),
        origin=Origin(xyz=(0.0, NOSE_ARM_Y, 0.0)),
        material=nose_orange,
        name="nose_arm",
    )
    nose_bracket.visual(
        Box((NOSE_TIP_W, NOSE_TIP_D, NOSE_TIP_H)),
        origin=Origin(xyz=(0.0, NOSE_TIP_Y, NOSE_TIP_Z)),
        material=nose_orange,
        name="nose_tip",
    )
    nose_bracket.visual(
        Box((NOSE_W * 0.75, NOSE_RIB_D, NOSE_RIB_H)),
        origin=Origin(xyz=(0.0, NOSE_RIB_Y, NOSE_RIB_Z)),
        material=nose_orange,
        name="nose_rib",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_ZERO_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.45,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_nose_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose_bracket,
        origin=Origin(xyz=(0.0, EAR_Y, EAR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-0.55,
            upper=0.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    nose_bracket = object_model.get_part("nose_bracket")
    lift = object_model.get_articulation("mast_to_carriage")
    wrist = object_model.get_articulation("carriage_to_nose_bracket")

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
        "lift uses vertical prismatic axis",
        lift.axis == (0.0, 0.0, 1.0),
        f"expected vertical axis, got {lift.axis}",
    )
    ctx.check(
        "wrist uses lateral hinge axis",
        wrist.axis == (1.0, 0.0, 0.0),
        f"expected x-axis hinge, got {wrist.axis}",
    )
    ctx.check(
        "lift limits are realistic",
        (
            lift.motion_limits is not None
            and lift.motion_limits.lower == 0.0
            and lift.motion_limits.upper is not None
            and 0.6 <= lift.motion_limits.upper <= 0.9
        ),
        f"unexpected lift limits {lift.motion_limits}",
    )
    ctx.check(
        "wrist limits are realistic",
        (
            wrist.motion_limits is not None
            and wrist.motion_limits.lower is not None
            and wrist.motion_limits.upper is not None
            and wrist.motion_limits.lower < 0.0 < wrist.motion_limits.upper
            and wrist.motion_limits.upper - wrist.motion_limits.lower < 2.0
        ),
        f"unexpected wrist limits {wrist.motion_limits}",
    )

    with ctx.pose({lift: 0.0, wrist: WRIST_ZERO}):
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=0.0015,
            name="carriage is guided directly on the mast",
        )
        ctx.expect_contact(
            nose_bracket,
            carriage,
            contact_tol=0.0015,
            name="nose bracket is seated between side supports",
        )
        ctx.expect_overlap(
            nose_bracket,
            carriage,
            axes="xz",
            min_overlap=0.045,
            name="nose bracket stays centered in the carriage cheeks",
        )

    with ctx.pose({lift: 0.0}):
        lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        upper_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=0.0015,
            name="carriage remains guided when raised",
        )
    lift_delta = None
    if lower_pos is not None and upper_pos is not None:
        lift_delta = upper_pos[2] - lower_pos[2]
    ctx.check(
        "carriage actually lifts",
        lift_delta is not None and lift_delta > 0.65,
        f"expected upward travel above 0.65 m, got {lift_delta}",
    )

    with ctx.pose({lift: 0.36, wrist: 0.55}):
        ctx.expect_contact(
            nose_bracket,
            carriage,
            contact_tol=0.0015,
            name="wrist stays supported through pitch motion",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
