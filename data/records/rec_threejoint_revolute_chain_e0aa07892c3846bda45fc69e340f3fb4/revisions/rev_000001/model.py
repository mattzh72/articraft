from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_FOOT_LENGTH = 0.20
BASE_FOOT_WIDTH = 0.09
BASE_FOOT_THICKNESS = 0.018
SHOULDER_X = -0.050
SHOULDER_Z = 0.083

CLEVIS_GAP = 0.018
PLATE_THICKNESS = 0.006
CLEVIS_OUTER_WIDTH = CLEVIS_GAP + 2.0 * PLATE_THICKNESS
CLEVIS_HEIGHT = 0.034
CLEVIS_PLATE_LENGTH = 0.022
CLEVIS_PLATE_CENTER_X = -0.003
CLEVIS_BRIDGE_LENGTH = 0.010
CLEVIS_BRIDGE_CENTER_X = -0.013
CLEVIS_SIDE_Y = CLEVIS_GAP / 2.0 + PLATE_THICKNESS / 2.0

JOINT_LUG_LENGTH = 0.014
JOINT_LUG_WIDTH = CLEVIS_GAP
JOINT_LUG_HEIGHT = 0.024
JOINT_LUG_CENTER_X = 0.006

ROOT_BODY_LENGTH = 0.022
ROOT_BODY_WIDTH = 0.016
ROOT_BODY_HEIGHT = 0.020
ROOT_BODY_START = 0.012

BEAM_WIDTH = 0.014
BEAM_HEIGHT = 0.012
BEAM_START = 0.034

LINK_1_SPAN = 0.112
LINK_2_SPAN = 0.092

END_TAB_LENGTH = 0.020
END_TAB_WIDTH = 0.018
END_TAB_HEIGHT = 0.020
END_TAB_CENTER_X = 0.068


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_clevis(part, joint_x: float, joint_z: float, *, material: str, prefix: str) -> None:
    for side, y_pos in (("left", CLEVIS_SIDE_Y), ("right", -CLEVIS_SIDE_Y)):
        _add_box(
            part,
            (CLEVIS_PLATE_LENGTH, PLATE_THICKNESS, CLEVIS_HEIGHT),
            (joint_x + CLEVIS_PLATE_CENTER_X, y_pos, joint_z),
            material=material,
            name=f"{prefix}_{side}_plate",
        )
    _add_box(
        part,
        (CLEVIS_BRIDGE_LENGTH, CLEVIS_OUTER_WIDTH, CLEVIS_HEIGHT),
        (joint_x + CLEVIS_BRIDGE_CENTER_X, 0.0, joint_z),
        material=material,
        name=f"{prefix}_bridge",
    )


def _add_link_body(part, span: float, *, material: str, prefix: str) -> None:
    _add_box(
        part,
        (JOINT_LUG_LENGTH, JOINT_LUG_WIDTH, JOINT_LUG_HEIGHT),
        (JOINT_LUG_CENTER_X, 0.0, 0.0),
        material=material,
        name=f"{prefix}_hinge_block",
    )
    _add_box(
        part,
        (ROOT_BODY_LENGTH, ROOT_BODY_WIDTH, ROOT_BODY_HEIGHT),
        (ROOT_BODY_START + ROOT_BODY_LENGTH / 2.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_root_body",
    )
    beam_end = span - 0.010
    beam_length = beam_end - BEAM_START
    _add_box(
        part,
        (beam_length, BEAM_WIDTH, BEAM_HEIGHT),
        ((BEAM_START + beam_end) / 2.0, 0.0, 0.0),
        material=material,
        name=f"{prefix}_beam",
    )


def _add_distal_tab(part, *, material: str) -> None:
    _add_box(
        part,
        (0.012, 0.012, 0.012),
        (0.054, 0.0, 0.0),
        material=material,
        name="tab_neck",
    )
    _add_box(
        part,
        (END_TAB_LENGTH, END_TAB_WIDTH, END_TAB_HEIGHT),
        (END_TAB_CENTER_X, 0.0, 0.0),
        material=material,
        name="end_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_three_joint_chain")

    model.material("powder_coat", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("dark_anodized", rgba=(0.30, 0.32, 0.35, 1.0))

    base = model.part("base_foot")
    _add_box(
        base,
        (BASE_FOOT_LENGTH, BASE_FOOT_WIDTH, BASE_FOOT_THICKNESS),
        (0.0, 0.0, BASE_FOOT_THICKNESS / 2.0),
        material="powder_coat",
        name="foot_pad",
    )
    tower_height = SHOULDER_Z - CLEVIS_HEIGHT / 2.0 - BASE_FOOT_THICKNESS
    _add_box(
        base,
        (0.036, 0.040, tower_height),
        (SHOULDER_X - 0.020, 0.0, BASE_FOOT_THICKNESS + tower_height / 2.0),
        material="powder_coat",
        name="rear_tower",
    )
    _add_box(
        base,
        (0.020, 0.040, 0.020),
        (SHOULDER_X - 0.018, 0.0, SHOULDER_Z - CLEVIS_HEIGHT / 2.0 - 0.010),
        material="powder_coat",
        name="shoulder_head",
    )
    _add_clevis(base, SHOULDER_X, SHOULDER_Z, material="powder_coat", prefix="shoulder")

    link_1 = model.part("link_1")
    _add_link_body(link_1, LINK_1_SPAN, material="brushed_aluminum", prefix="link_1")
    _add_clevis(link_1, LINK_1_SPAN, 0.0, material="brushed_aluminum", prefix="elbow")

    link_2 = model.part("link_2")
    _add_link_body(link_2, LINK_2_SPAN, material="brushed_aluminum", prefix="link_2")
    _add_clevis(link_2, LINK_2_SPAN, 0.0, material="brushed_aluminum", prefix="wrist")

    link_3 = model.part("link_3")
    _add_link_body(link_3, END_TAB_CENTER_X - END_TAB_LENGTH / 2.0, material="dark_anodized", prefix="link_3")
    _add_distal_tab(link_3, material="dark_anodized")

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.45, upper=1.20),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_SPAN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.3, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_SPAN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.8, lower=-1.00, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_foot")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")

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
        len(object_model.articulations) == 3
        and all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder, elbow, wrist)
        ),
        details="expected a three-joint revolute chain",
    )
    ctx.check(
        "joint_axes_share_motion_plane",
        all(joint.axis == (0.0, -1.0, 0.0) for joint in (shoulder, elbow, wrist)),
        details="all revolute axes should align along the same lateral hinge axis",
    )

    ctx.expect_contact(base, link_1, name="shoulder_joint_supported")
    ctx.expect_contact(link_1, link_2, name="elbow_joint_supported")
    ctx.expect_contact(link_2, link_3, name="wrist_joint_supported")

    link_1_pos = ctx.part_world_position(link_1)
    link_2_pos = ctx.part_world_position(link_2)
    link_3_pos = ctx.part_world_position(link_3)
    ctx.check(
        "rest_pose_joint_origins_step_forward",
        link_1_pos is not None
        and link_2_pos is not None
        and link_3_pos is not None
        and link_1_pos[0] < link_2_pos[0] < link_3_pos[0],
        details="neutral pose should read as a forward-reaching stacked chain",
    )
    ctx.check(
        "rest_pose_joint_origins_share_height",
        link_1_pos is not None
        and link_2_pos is not None
        and link_3_pos is not None
        and abs(link_1_pos[2] - link_2_pos[2]) < 1e-6
        and abs(link_2_pos[2] - link_3_pos[2]) < 1e-6,
        details="neutral pose should keep all three hinge axes in one motion plane",
    )

    with ctx.pose(shoulder_pitch=0.60, elbow_pitch=0.55, wrist_pitch=-0.20):
        raised_link_3_pos = ctx.part_world_position(link_3)
    ctx.check(
        "positive_pitch_raises_distal_chain",
        link_3_pos is not None
        and raised_link_3_pos is not None
        and raised_link_3_pos[2] > link_3_pos[2] + 0.040,
        details="positive shoulder and elbow motion should lift the distal link upward",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
