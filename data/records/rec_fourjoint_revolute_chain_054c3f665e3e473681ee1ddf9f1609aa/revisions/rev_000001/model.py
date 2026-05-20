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


FOOT_X = 0.100
FOOT_Y = 0.070
FOOT_Z = 0.012

SPINE_X = 0.016
SPINE_Y = 0.020
SPINE_Z = 0.180
SPINE_X_CENTER = -0.008

JOINT_AXIS_X = 0.014
SHOULDER_Z = 0.184

HUB_RADIUS = 0.011
HUB_WIDTH = 0.010
BODY_WIDTH = 0.018
BODY_THICKNESS = 0.008
CHEEK_THICKNESS = 0.004
CHEEK_HEIGHT = 0.026
SIDE_RAIL_LEN = 0.022
CHEEK_LEN = 0.016
PROXIMAL_NECK_LEN = 0.020
FORK_Y = HUB_WIDTH / 2.0 + CHEEK_THICKNESS / 2.0

LINK_1_LENGTH = 0.170
LINK_2_LENGTH = 0.150
LINK_3_LENGTH = 0.130
LINK_4_LENGTH = 0.100


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_hub(part, *, name: str = "hub", material: str = "joint_dark") -> None:
    part.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_root_support(model: ArticulatedObject):
    support = model.part("ground_spine")

    _add_box(
        support,
        name="foot",
        size=(FOOT_X, FOOT_Y, FOOT_Z),
        xyz=(0.010, 0.0, FOOT_Z / 2.0),
        material="base_charcoal",
    )
    _add_box(
        support,
        name="spine",
        size=(SPINE_X, SPINE_Y, SPINE_Z),
        xyz=(SPINE_X_CENTER, 0.0, FOOT_Z + SPINE_Z / 2.0),
        material="base_charcoal",
    )
    for suffix, y_pos in (("left", FORK_Y), ("right", -FORK_Y)):
        _add_box(
            support,
            name=f"shoulder_rail_{suffix}",
            size=(0.020, CHEEK_THICKNESS, BODY_THICKNESS),
            xyz=(0.010, y_pos, SHOULDER_Z - 0.017),
            material="base_charcoal",
        )
        _add_box(
            support,
            name=f"shoulder_cheek_{suffix}",
            size=(CHEEK_LEN, CHEEK_THICKNESS, CHEEK_HEIGHT),
            xyz=(JOINT_AXIS_X, y_pos, SHOULDER_Z),
            material="joint_dark",
        )

    support.inertial = Inertial.from_geometry(
        Box((FOOT_X, FOOT_Y, SPINE_Z + FOOT_Z)),
        mass=2.8,
        origin=Origin(xyz=(0.010, 0.0, (SPINE_Z + FOOT_Z) / 2.0)),
    )
    return support


def _build_chain_link(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    end_tab: bool = False,
):
    link = model.part(name)

    _add_hub(link, material="joint_dark")
    _add_box(
        link,
        name="proximal_neck",
        size=(PROXIMAL_NECK_LEN, HUB_WIDTH, BODY_THICKNESS),
        xyz=(PROXIMAL_NECK_LEN / 2.0, 0.0, 0.0),
        material="link_silver",
    )

    if end_tab:
        main_body_len = length - PROXIMAL_NECK_LEN
        _add_box(
            link,
            name="main_body",
            size=(main_body_len, BODY_WIDTH, BODY_THICKNESS),
            xyz=(PROXIMAL_NECK_LEN + main_body_len / 2.0, 0.0, 0.0),
            material="link_silver",
        )
        _add_box(
            link,
            name="end_tab",
            size=(0.030, 0.022, 0.005),
            xyz=(length + 0.015, 0.0, 0.0),
            material="tab_black",
        )
        _add_box(
            link,
            name="tip_pad",
            size=(0.010, 0.012, 0.010),
            xyz=(length + 0.033, 0.0, 0.0),
            material="tab_black",
        )
    else:
        main_body_len = length - PROXIMAL_NECK_LEN - SIDE_RAIL_LEN
        _add_box(
            link,
            name="main_body",
            size=(main_body_len, BODY_WIDTH, BODY_THICKNESS),
            xyz=(PROXIMAL_NECK_LEN + main_body_len / 2.0, 0.0, 0.0),
            material="link_silver",
        )
        for suffix, y_pos in (("left", FORK_Y), ("right", -FORK_Y)):
            _add_box(
                link,
                name=f"side_rail_{suffix}",
                size=(SIDE_RAIL_LEN, CHEEK_THICKNESS, BODY_THICKNESS),
                xyz=(length - SIDE_RAIL_LEN / 2.0, y_pos, 0.0),
                material="link_silver",
            )
            _add_box(
                link,
                name=f"distal_cheek_{suffix}",
                size=(CHEEK_LEN, CHEEK_THICKNESS, CHEEK_HEIGHT),
                xyz=(length, y_pos, 0.0),
                material="joint_dark",
            )

    inertial_length = length + (0.040 if end_tab else 0.0)
    link.inertial = Inertial.from_geometry(
        Box((inertial_length, BODY_WIDTH, CHEEK_HEIGHT)),
        mass=0.34 if not end_tab else 0.28,
        origin=Origin(xyz=(inertial_length / 2.0, 0.0, 0.0)),
    )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_service_four_joint_chain")

    model.material("base_charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("joint_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("link_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("tab_black", rgba=(0.15, 0.16, 0.18, 1.0))

    support = _build_root_support(model)
    link_1 = _build_chain_link(model, name="link_1", length=LINK_1_LENGTH)
    link_2 = _build_chain_link(model, name="link_2", length=LINK_2_LENGTH)
    link_3 = _build_chain_link(model, name="link_3", length=LINK_3_LENGTH)
    link_4 = _build_chain_link(model, name="link_4", length=LINK_4_LENGTH, end_tab=True)

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_1,
        origin=Origin(xyz=(JOINT_AXIS_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.20, effort=22.0, velocity=1.4),
    )
    model.articulation(
        "mid_joint_1",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.15, effort=18.0, velocity=1.8),
    )
    model.articulation(
        "mid_joint_2",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.15, effort=14.0, velocity=2.0),
    )
    model.articulation(
        "end_joint",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.20, effort=10.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("ground_spine")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")

    shoulder = object_model.get_articulation("shoulder_joint")
    mid_1 = object_model.get_articulation("mid_joint_1")
    mid_2 = object_model.get_articulation("mid_joint_2")
    end = object_model.get_articulation("end_joint")

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

    for joint in (shoulder, mid_1, mid_2, end):
        ctx.check(
            f"{joint.name} uses a parallel pitch axis",
            joint.axis == (0.0, -1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_contact(
        support,
        link_1,
        elem_a="shoulder_cheek_left",
        elem_b="hub",
        name="shoulder hub bears on left support cheek",
    )
    ctx.expect_contact(
        support,
        link_1,
        elem_a="shoulder_cheek_right",
        elem_b="hub",
        name="shoulder hub bears on right support cheek",
    )
    ctx.expect_contact(
        link_1,
        link_2,
        elem_a="distal_cheek_left",
        elem_b="hub",
        name="link 1 supports link 2 at left cheek",
    )
    ctx.expect_contact(
        link_1,
        link_2,
        elem_a="distal_cheek_right",
        elem_b="hub",
        name="link 1 supports link 2 at right cheek",
    )
    ctx.expect_contact(
        link_2,
        link_3,
        elem_a="distal_cheek_left",
        elem_b="hub",
        name="link 2 supports link 3 at left cheek",
    )
    ctx.expect_contact(
        link_2,
        link_3,
        elem_a="distal_cheek_right",
        elem_b="hub",
        name="link 2 supports link 3 at right cheek",
    )
    ctx.expect_contact(
        link_3,
        link_4,
        elem_a="distal_cheek_left",
        elem_b="hub",
        name="link 3 supports link 4 at left cheek",
    )
    ctx.expect_contact(
        link_3,
        link_4,
        elem_a="distal_cheek_right",
        elem_b="hub",
        name="link 3 supports link 4 at right cheek",
    )

    rest_tip_pos = ctx.part_world_position(link_4)
    with ctx.pose(
        {
            shoulder: 0.55,
            mid_1: -0.35,
            mid_2: 0.60,
            end: -0.20,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="chain stays clear in a bent service pose")
        posed_tip_pos = ctx.part_world_position(link_4)
        ctx.check(
            "positive chain pose lifts the end tab assembly",
            rest_tip_pos is not None
            and posed_tip_pos is not None
            and posed_tip_pos[2] > rest_tip_pos[2] + 0.05,
            details=f"rest={rest_tip_pos}, posed={posed_tip_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
