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


PIN_RADIUS = 0.0055
EYE_OUTER_RADIUS = 0.018

EYE_THICKNESS = 0.012
EAR_THICKNESS = 0.010
FORK_WIDTH = EYE_THICKNESS + 2.0 * EAR_THICKNESS

LINK_CENTER_DISTANCE = 0.115
LINK_BODY_WIDTH = 0.022

BRACKET_TOP_WIDTH = 0.085
BRACKET_TOP_DEPTH = 0.050
BRACKET_TOP_THICKNESS = 0.008
BRACKET_TOP_CENTER_Z = 0.052
BRACKET_BODY_DEPTH = 0.032
BRACKET_BODY_HEIGHT = 0.088
BRACKET_BODY_CENTER_Z = 0.012
BRACKET_SLOT_HEIGHT = 0.074
BRACKET_SLOT_CENTER_Z = -0.007

LOWER_FORK_TOP_Z = -LINK_CENTER_DISTANCE + 0.047
LOWER_FORK_BOTTOM_Z = -LINK_CENTER_DISTANCE - EYE_OUTER_RADIUS
LOWER_FORK_HEIGHT = LOWER_FORK_TOP_Z - LOWER_FORK_BOTTOM_Z
LOWER_FORK_CENTER_Z = 0.5 * (LOWER_FORK_TOP_Z + LOWER_FORK_BOTTOM_Z)
LOWER_SLOT_TOP_Z = -LINK_CENTER_DISTANCE + 0.033
LOWER_SLOT_BOTTOM_Z = -LINK_CENTER_DISTANCE - EYE_OUTER_RADIUS - 0.004
LOWER_SLOT_HEIGHT = LOWER_SLOT_TOP_Z - LOWER_SLOT_BOTTOM_Z
LOWER_SLOT_CENTER_Z = 0.5 * (LOWER_SLOT_TOP_Z + LOWER_SLOT_BOTTOM_Z)

END_TAB_WIDTH = 0.030
END_TAB_BODY_HEIGHT = 0.050
END_TAB_BODY_CENTER_Z = -0.032
END_TAB_BOTTOM_RADIUS = 0.013
END_TAB_BOTTOM_CENTER_Z = -0.056

BRACKET_YOKE_BLOCK_HEIGHT = 0.030
BRACKET_YOKE_BLOCK_CENTER_Z = 0.033
BRACKET_EAR_HEIGHT = 0.046
BRACKET_EAR_CENTER_Z = 0.006

LINK_STRAP_HEIGHT = 0.068
LINK_STRAP_CENTER_Z = -0.041
LINK_SHOULDER_HEIGHT = 0.018
LINK_SHOULDER_CENTER_Z = -0.072
LINK_EAR_HEIGHT = 0.040
LINK_EAR_CENTER_Z = -0.096

EAR_OFFSET_X = EYE_THICKNESS / 2.0 + EAR_THICKNESS / 2.0


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _x_cylinder_origin(center: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_revolute_chain")

    model.material("powder_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("zinc_link", rgba=(0.73, 0.76, 0.79, 1.0))
    model.material("black_oxide", rgba=(0.15, 0.16, 0.17, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        Box((BRACKET_TOP_WIDTH, BRACKET_TOP_DEPTH, BRACKET_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_TOP_CENTER_Z)),
        material="powder_steel",
        name="bracket_top",
    )
    support_bracket.visual(
        Box((FORK_WIDTH, BRACKET_BODY_DEPTH, BRACKET_YOKE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_YOKE_BLOCK_CENTER_Z)),
        material="powder_steel",
        name="bracket_yoke",
    )
    support_bracket.visual(
        Box((EAR_THICKNESS, 0.024, BRACKET_EAR_HEIGHT)),
        origin=Origin(xyz=(-EAR_OFFSET_X, 0.0, BRACKET_EAR_CENTER_Z)),
        material="powder_steel",
        name="left_ear",
    )
    support_bracket.visual(
        Box((EAR_THICKNESS, 0.024, BRACKET_EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_OFFSET_X, 0.0, BRACKET_EAR_CENTER_Z)),
        material="powder_steel",
        name="right_ear",
    )
    support_bracket.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EAR_THICKNESS),
        origin=_x_cylinder_origin((-EAR_OFFSET_X, 0.0, 0.0)),
        material="powder_steel",
        name="left_hinge_barrel",
    )
    support_bracket.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EAR_THICKNESS),
        origin=_x_cylinder_origin((EAR_OFFSET_X, 0.0, 0.0)),
        material="powder_steel",
        name="right_hinge_barrel",
    )
    support_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_TOP_WIDTH, BRACKET_TOP_DEPTH, 0.092)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EYE_THICKNESS),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material="zinc_link",
        name="top_eye",
    )
    link_1.visual(
        Box((EYE_THICKNESS, LINK_BODY_WIDTH, LINK_STRAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LINK_STRAP_CENTER_Z)),
        material="zinc_link",
        name="center_strap",
    )
    link_1.visual(
        Box((FORK_WIDTH, LINK_BODY_WIDTH, LINK_SHOULDER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LINK_SHOULDER_CENTER_Z)),
        material="zinc_link",
        name="lower_shoulder",
    )
    link_1.visual(
        Box((EAR_THICKNESS, LINK_BODY_WIDTH, LINK_EAR_HEIGHT)),
        origin=Origin(xyz=(-EAR_OFFSET_X, 0.0, LINK_EAR_CENTER_Z)),
        material="zinc_link",
        name="left_clevis_ear",
    )
    link_1.visual(
        Box((EAR_THICKNESS, LINK_BODY_WIDTH, LINK_EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_OFFSET_X, 0.0, LINK_EAR_CENTER_Z)),
        material="zinc_link",
        name="right_clevis_ear",
    )
    link_1.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EAR_THICKNESS),
        origin=_x_cylinder_origin((-EAR_OFFSET_X, 0.0, -LINK_CENTER_DISTANCE)),
        material="zinc_link",
        name="left_lower_barrel",
    )
    link_1.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EAR_THICKNESS),
        origin=_x_cylinder_origin((EAR_OFFSET_X, 0.0, -LINK_CENTER_DISTANCE)),
        material="zinc_link",
        name="right_lower_barrel",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((FORK_WIDTH, LINK_BODY_WIDTH, LINK_CENTER_DISTANCE + 0.018)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * LINK_CENTER_DISTANCE)),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EYE_THICKNESS),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material="zinc_link",
        name="top_eye",
    )
    link_2.visual(
        Box((EYE_THICKNESS, LINK_BODY_WIDTH, LINK_STRAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LINK_STRAP_CENTER_Z)),
        material="zinc_link",
        name="center_strap",
    )
    link_2.visual(
        Box((FORK_WIDTH, LINK_BODY_WIDTH, LINK_SHOULDER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LINK_SHOULDER_CENTER_Z)),
        material="zinc_link",
        name="lower_shoulder",
    )
    link_2.visual(
        Box((EAR_THICKNESS, LINK_BODY_WIDTH, LINK_EAR_HEIGHT)),
        origin=Origin(xyz=(-EAR_OFFSET_X, 0.0, LINK_EAR_CENTER_Z)),
        material="zinc_link",
        name="left_clevis_ear",
    )
    link_2.visual(
        Box((EAR_THICKNESS, LINK_BODY_WIDTH, LINK_EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_OFFSET_X, 0.0, LINK_EAR_CENTER_Z)),
        material="zinc_link",
        name="right_clevis_ear",
    )
    link_2.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EAR_THICKNESS),
        origin=_x_cylinder_origin((-EAR_OFFSET_X, 0.0, -LINK_CENTER_DISTANCE)),
        material="zinc_link",
        name="left_lower_barrel",
    )
    link_2.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EAR_THICKNESS),
        origin=_x_cylinder_origin((EAR_OFFSET_X, 0.0, -LINK_CENTER_DISTANCE)),
        material="zinc_link",
        name="right_lower_barrel",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((FORK_WIDTH, LINK_BODY_WIDTH, LINK_CENTER_DISTANCE + 0.018)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * LINK_CENTER_DISTANCE)),
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        Cylinder(radius=EYE_OUTER_RADIUS, length=EYE_THICKNESS),
        origin=_x_cylinder_origin((0.0, 0.0, 0.0)),
        material="black_oxide",
        name="top_eye",
    )
    end_tab.visual(
        Box((EYE_THICKNESS, END_TAB_WIDTH, END_TAB_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, END_TAB_BODY_CENTER_Z)),
        material="black_oxide",
        name="tab_body",
    )
    end_tab.visual(
        Cylinder(radius=END_TAB_BOTTOM_RADIUS, length=EYE_THICKNESS),
        origin=_x_cylinder_origin((0.0, 0.0, END_TAB_BOTTOM_CENTER_Z)),
        material="black_oxide",
        name="tip_pad",
    )
    end_tab.inertial = Inertial.from_geometry(
        Box((EYE_THICKNESS, END_TAB_WIDTH, 0.074)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    model.articulation(
        "bracket_to_link_1",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=link_1,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK_CENTER_DISTANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "link_2_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -LINK_CENTER_DISTANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")

    bracket_to_link_1 = object_model.get_articulation("bracket_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_end_tab = object_model.get_articulation("link_2_to_end_tab")

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
        "three_revolute_joints_present",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (bracket_to_link_1, link_1_to_link_2, link_2_to_end_tab)
        ),
        "expected three revolute articulations in the hanging chain",
    )
    ctx.check(
        "parallel_hinge_axes",
        all(
            tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0)
            for joint in (bracket_to_link_1, link_1_to_link_2, link_2_to_end_tab)
        ),
        "all three hinges should share the same horizontal x-axis",
    )

    ctx.expect_contact(
        support_bracket,
        link_1,
        name="support_bracket_physically_carries_link_1",
    )
    ctx.expect_contact(
        link_1,
        link_2,
        name="link_1_physically_carries_link_2",
    )
    ctx.expect_contact(
        link_2,
        end_tab,
        name="link_2_physically_carries_end_tab",
    )

    ctx.expect_origin_gap(
        link_1,
        link_2,
        axis="z",
        min_gap=LINK_CENTER_DISTANCE - 1e-4,
        max_gap=LINK_CENTER_DISTANCE + 1e-4,
        name="link_2_hangs_one_pitch_below_link_1",
    )
    ctx.expect_origin_gap(
        link_2,
        end_tab,
        axis="z",
        min_gap=LINK_CENTER_DISTANCE - 1e-4,
        max_gap=LINK_CENTER_DISTANCE + 1e-4,
        name="end_tab_hangs_one_pitch_below_link_2",
    )

    rest_tab_center = _aabb_center(ctx.part_world_aabb(end_tab))
    rest_tip_pad_center = _aabb_center(
        ctx.part_element_world_aabb(end_tab, elem="tip_pad")
    )

    with ctx.pose({bracket_to_link_1: 0.55}):
        joint_1_tab_center = _aabb_center(ctx.part_world_aabb(end_tab))
    ctx.check(
        "joint_1_swings_chain_forward",
        joint_1_tab_center[1] > rest_tab_center[1] + 0.09
        and joint_1_tab_center[2] > rest_tab_center[2] + 0.02,
        (
            "positive bracket_to_link_1 rotation should move the suspended chain "
            "forward in +y and slightly upward"
        ),
    )

    with ctx.pose({link_1_to_link_2: 0.55}):
        joint_2_tab_center = _aabb_center(ctx.part_world_aabb(end_tab))
    ctx.check(
        "joint_2_swings_lower_chain_forward",
        joint_2_tab_center[1] > rest_tab_center[1] + 0.05
        and joint_2_tab_center[2] > rest_tab_center[2] + 0.01,
        (
            "positive link_1_to_link_2 rotation should carry the lower link and "
            "end tab forward"
        ),
    )

    with ctx.pose({link_2_to_end_tab: 0.55}):
        joint_3_tip_pad_center = _aabb_center(
            ctx.part_element_world_aabb(end_tab, elem="tip_pad")
        )
    ctx.check(
        "joint_3_swings_end_tab_forward",
        joint_3_tip_pad_center[1] > rest_tip_pad_center[1] + 0.02,
        "positive link_2_to_end_tab rotation should swing the compact tab in +y",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
