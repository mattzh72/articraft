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


TOP_PLATE_WIDTH = 0.110
TOP_PLATE_DEPTH = 0.052
TOP_PLATE_THICKNESS = 0.008
BRACKET_DROP = 0.032
BRACKET_LUG_WIDTH = 0.022
BRACKET_LUG_THICKNESS = 0.008
BRACKET_HINGE_RADIUS = 0.015

LINK_1_THICKNESS = 0.010
LINK_1_BODY_WIDTH = 0.020
LINK_1_CENTER_SPAN = 0.115
LINK_1_TOP_EYE_RADIUS = 0.015
LINK_1_STRAP_LENGTH = 0.082
LINK_1_STRAP_CENTER_Z = -0.055
LINK_1_LOWER_BRIDGE_WIDTH = 0.028
LINK_1_LOWER_BRIDGE_HEIGHT = 0.012
LINK_1_LOWER_BRIDGE_CENTER_Z = -0.102
LINK_1_FORK_ARM_THICKNESS = 0.005
LINK_1_FORK_ARM_WIDTH = 0.010
LINK_1_FORK_ARM_LENGTH = 0.019
LINK_1_FORK_ARM_CENTER_Z = -0.104

LINK_2_THICKNESS = 0.010
LINK_2_BODY_WIDTH = 0.018
LINK_2_TOP_EYE_RADIUS = 0.012
LINK_2_BODY_LENGTH = 0.078
LINK_2_BODY_CENTER_Z = -0.051
LINK_2_TAB_RADIUS = 0.013
LINK_2_TAB_CENTER_Z = -0.090
LINK_2_TAIL_LENGTH = 0.014

UPPER_HINGE_PIN_RADIUS = 0.0055
LOWER_HINGE_PIN_RADIUS = 0.005

LINK_1_LUG_CENTER_Y = (BRACKET_LUG_THICKNESS + LINK_1_THICKNESS) / 2.0
LINK_2_WITHIN_FORK_CENTER_Y = (
    LINK_1_FORK_ARM_THICKNESS + LINK_2_THICKNESS
) / 2.0


def _hinge_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_revolute_chain")

    bracket_finish = model.material("bracket_finish", rgba=(0.18, 0.19, 0.22, 1.0))
    link_finish = model.material("link_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        Box((TOP_PLATE_WIDTH, TOP_PLATE_DEPTH, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_DROP + TOP_PLATE_THICKNESS / 2.0)),
        material=bracket_finish,
        name="top_plate",
    )
    bracket.visual(
        Box((BRACKET_LUG_WIDTH, BRACKET_LUG_THICKNESS, BRACKET_DROP)),
        origin=Origin(xyz=(0.0, LINK_1_LUG_CENTER_Y, BRACKET_DROP / 2.0)),
        material=bracket_finish,
        name="left_lug_web",
    )
    bracket.visual(
        Box((BRACKET_LUG_WIDTH, BRACKET_LUG_THICKNESS, BRACKET_DROP)),
        origin=Origin(xyz=(0.0, -LINK_1_LUG_CENTER_Y, BRACKET_DROP / 2.0)),
        material=bracket_finish,
        name="right_lug_web",
    )
    bracket.visual(
        Cylinder(radius=BRACKET_HINGE_RADIUS, length=BRACKET_LUG_THICKNESS),
        origin=_hinge_origin((0.0, LINK_1_LUG_CENTER_Y, 0.0)),
        material=bracket_finish,
        name="left_hinge_lug",
    )
    bracket.visual(
        Cylinder(radius=BRACKET_HINGE_RADIUS, length=BRACKET_LUG_THICKNESS),
        origin=_hinge_origin((0.0, -LINK_1_LUG_CENTER_Y, 0.0)),
        material=bracket_finish,
        name="right_hinge_lug",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=LINK_1_TOP_EYE_RADIUS, length=LINK_1_THICKNESS),
        origin=_hinge_origin((0.0, 0.0, 0.0)),
        material=link_finish,
        name="upper_barrel",
    )
    link_1.visual(
        Box((LINK_1_BODY_WIDTH, LINK_1_THICKNESS, LINK_1_STRAP_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, LINK_1_STRAP_CENTER_Z)),
        material=link_finish,
        name="strap",
    )
    link_1.visual(
        Box((LINK_1_LOWER_BRIDGE_WIDTH, LINK_1_THICKNESS, LINK_1_LOWER_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LINK_1_LOWER_BRIDGE_CENTER_Z)),
        material=link_finish,
        name="lower_bridge",
    )
    link_1.visual(
        Box((LINK_1_FORK_ARM_WIDTH, LINK_1_FORK_ARM_THICKNESS, LINK_1_FORK_ARM_LENGTH)),
        origin=Origin(
            xyz=(0.0, LINK_2_WITHIN_FORK_CENTER_Y, LINK_1_FORK_ARM_CENTER_Z)
        ),
        material=link_finish,
        name="left_fork_arm",
    )
    link_1.visual(
        Box((LINK_1_FORK_ARM_WIDTH, LINK_1_FORK_ARM_THICKNESS, LINK_1_FORK_ARM_LENGTH)),
        origin=Origin(
            xyz=(0.0, -LINK_2_WITHIN_FORK_CENTER_Y, LINK_1_FORK_ARM_CENTER_Z)
        ),
        material=link_finish,
        name="right_fork_arm",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        Cylinder(radius=LINK_2_TOP_EYE_RADIUS, length=LINK_2_THICKNESS),
        origin=_hinge_origin((0.0, 0.0, 0.0)),
        material=link_finish,
        name="upper_eye",
    )
    link_2.visual(
        Box((LINK_2_BODY_WIDTH, LINK_2_THICKNESS, LINK_2_BODY_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, LINK_2_BODY_CENTER_Z)),
        material=link_finish,
        name="strap",
    )
    link_2.visual(
        Cylinder(radius=LINK_2_TAB_RADIUS, length=LINK_2_THICKNESS),
        origin=_hinge_origin((0.0, 0.0, LINK_2_TAB_CENTER_Z)),
        material=link_finish,
        name="end_tab",
    )
    link_2.visual(
        Box((LINK_2_BODY_WIDTH * 0.8, LINK_2_THICKNESS, LINK_2_TAIL_LENGTH)),
        origin=Origin(
            xyz=(0.0, 0.0, LINK_2_TAB_CENTER_Z - LINK_2_TAIL_LENGTH / 2.0)
        ),
        material=link_finish,
        name="tail",
    )

    model.articulation(
        "bracket_to_link_1",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=-1.30,
            upper=1.30,
        ),
    )

    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK_1_CENTER_SPAN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.8,
            lower=-1.55,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    shoulder = object_model.get_articulation("bracket_to_link_1")
    elbow = object_model.get_articulation("link_1_to_link_2")

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
        "parallel hinge axes",
        shoulder.axis == elbow.axis == (0.0, -1.0, 0.0),
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )
    ctx.check(
        "rest pose lies within both joint ranges",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and shoulder.motion_limits.lower is not None
        and shoulder.motion_limits.upper is not None
        and elbow.motion_limits.lower is not None
        and elbow.motion_limits.upper is not None
        and shoulder.motion_limits.lower < 0.0 < shoulder.motion_limits.upper
        and elbow.motion_limits.lower < 0.0 < elbow.motion_limits.upper,
        details="Both revolute joints should be able to hang through the neutral pose.",
    )

    ctx.expect_contact(
        bracket,
        link_1,
        name="bracket cheeks physically carry first link",
    )
    ctx.expect_contact(
        link_1,
        link_2,
        name="lower clevis physically carries second link",
    )
    ctx.expect_origin_gap(
        bracket,
        link_2,
        axis="z",
        min_gap=0.10,
        max_gap=0.13,
        name="chain hangs below the top bracket in rest pose",
    )

    closed_link_2_pos = ctx.part_world_position(link_2)
    with ctx.pose({shoulder: 0.70}):
        swung_link_2_pos = ctx.part_world_position(link_2)

    ctx.check(
        "positive shoulder angle swings the lower chain forward",
        closed_link_2_pos is not None
        and swung_link_2_pos is not None
        and swung_link_2_pos[0] > closed_link_2_pos[0] + 0.06,
        details=f"closed={closed_link_2_pos}, swung={swung_link_2_pos}",
    )

    def _aabb_center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    closed_link_2_aabb = ctx.part_world_aabb(link_2)
    with ctx.pose({elbow: 0.95}):
        bent_link_2_aabb = ctx.part_world_aabb(link_2)

    closed_center_x = _aabb_center_x(closed_link_2_aabb)
    bent_center_x = _aabb_center_x(bent_link_2_aabb)
    ctx.check(
        "positive elbow angle folds the end tab forward",
        closed_center_x is not None
        and bent_center_x is not None
        and bent_center_x > closed_center_x + 0.02,
        details=f"closed_center_x={closed_center_x}, bent_center_x={bent_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
