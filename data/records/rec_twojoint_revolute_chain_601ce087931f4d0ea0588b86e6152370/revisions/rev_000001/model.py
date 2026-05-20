from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_PLATE_L = 0.230
BASE_PLATE_W = 0.160
BASE_PLATE_T = 0.022
PEDESTAL_L = 0.090
PEDESTAL_W = 0.082
PEDESTAL_H = 0.068
CHEEK_T = 0.016
BASE_CHEEK_GAP = 0.052
CHEEK_L = 0.050
CHEEK_H = 0.096
CHEEK_BOTTOM_Z = 0.076
SHOULDER_X = 0.0
SHOULDER_Z = 0.145
BASE_PIN_R = 0.0125
BASE_BOSS_R = 0.021
BASE_BOSS_T = 0.010

LINK1_LENGTH = 0.255
LINK1_W = 0.050
LINK1_H = 0.082
LINK1_NECK_L = 0.060
LINK1_NECK_W = 0.026
LINK1_NECK_H = 0.060
LINK1_BEAM_START = 0.058
LINK1_BEAM_LEN = 0.120
LINK1_ROOT_SPINDLE_R = BASE_PIN_R
LINK1_ROOT_SPINDLE_LEN = BASE_CHEEK_GAP
LINK1_FORK_START = 0.178
ELBOW_GAP = 0.042
ELBOW_CHEEK_T = 0.014
ELBOW_HOLE_R = 0.0115
ELBOW_YOKE_L = LINK1_LENGTH - LINK1_FORK_START
ELBOW_YOKE_H = 0.074
ELBOW_PAD_R = 0.021

LINK2_W = 0.044
LINK2_H = 0.068
LINK2_BEAM_START = 0.045
LINK2_BEAM_LEN = 0.122
LINK2_ROOT_SPINDLE_R = ELBOW_HOLE_R
LINK2_ROOT_SPINDLE_LEN = ELBOW_GAP
LINK2_ROOT_NECK_L = 0.058
LINK2_ROOT_NECK_W = 0.022
LINK2_ROOT_NECK_H = 0.052
END_TAB_L = 0.054
END_TAB_W = 0.028
END_TAB_T = 0.016
END_TAB_CENTER_X = 0.190
END_TAB_HOLE_R = 0.0065

SHOULDER_LIMITS = (0.0, 1.20)
ELBOW_LIMITS = (0.0, 1.45)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_two_joint_revolute_chain")

    model.material("base_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("link_amber", rgba=(0.86, 0.58, 0.16, 1.0))

    base = model.part("base")
    cheek_center_y = BASE_CHEEK_GAP / 2.0 + CHEEK_T / 2.0
    boss_center_y = BASE_CHEEK_GAP / 2.0 + CHEEK_T + BASE_BOSS_T / 2.0

    base.visual(
        Box((BASE_PLATE_L, BASE_PLATE_W, BASE_PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T / 2.0)),
        material="base_gray",
        name="base_plate",
    )
    base.visual(
        Box((PEDESTAL_L, PEDESTAL_W, PEDESTAL_H)),
        origin=Origin(xyz=(-0.030, 0.0, BASE_PLATE_T + PEDESTAL_H / 2.0)),
        material="base_gray",
        name="pedestal_block",
    )
    base.visual(
        Box((0.050, 0.012, CHEEK_BOTTOM_Z - BASE_PLATE_T)),
        origin=Origin(xyz=(-0.012, 0.028, BASE_PLATE_T + (CHEEK_BOTTOM_Z - BASE_PLATE_T) / 2.0)),
        material="base_gray",
        name="left_rib",
    )
    base.visual(
        Box((0.050, 0.012, CHEEK_BOTTOM_Z - BASE_PLATE_T)),
        origin=Origin(xyz=(-0.012, -0.028, BASE_PLATE_T + (CHEEK_BOTTOM_Z - BASE_PLATE_T) / 2.0)),
        material="base_gray",
        name="right_rib",
    )
    base.visual(
        Box((CHEEK_L, CHEEK_T, CHEEK_H)),
        origin=Origin(xyz=(SHOULDER_X, cheek_center_y, CHEEK_BOTTOM_Z + CHEEK_H / 2.0)),
        material="base_gray",
        name="left_cheek",
    )
    base.visual(
        Box((CHEEK_L, CHEEK_T, CHEEK_H)),
        origin=Origin(xyz=(SHOULDER_X, -cheek_center_y, CHEEK_BOTTOM_Z + CHEEK_H / 2.0)),
        material="base_gray",
        name="right_cheek",
    )
    base.visual(
        Cylinder(radius=BASE_BOSS_R, length=BASE_BOSS_T),
        origin=Origin(xyz=(SHOULDER_X, boss_center_y, SHOULDER_Z), rpy=(1.57079632679, 0.0, 0.0)),
        material="base_gray",
        name="left_boss",
    )
    base.visual(
        Cylinder(radius=BASE_BOSS_R, length=BASE_BOSS_T),
        origin=Origin(xyz=(SHOULDER_X, -boss_center_y, SHOULDER_Z), rpy=(1.57079632679, 0.0, 0.0)),
        material="base_gray",
        name="right_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_L, BASE_PLATE_W, SHOULDER_Z + 0.035)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_Z + 0.035) / 2.0)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=LINK1_ROOT_SPINDLE_R, length=LINK1_ROOT_SPINDLE_LEN),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material="link_amber",
        name="root_spindle",
    )
    primary_link.visual(
        Box((LINK1_NECK_L, LINK1_NECK_W, LINK1_NECK_H)),
        origin=Origin(xyz=(LINK1_NECK_L / 2.0, 0.0, 0.0)),
        material="link_amber",
        name="root_neck",
    )
    primary_link.visual(
        Box((LINK1_BEAM_LEN, LINK1_W, LINK1_H)),
        origin=Origin(xyz=(LINK1_BEAM_START + LINK1_BEAM_LEN / 2.0, 0.0, 0.0)),
        material="link_amber",
        name="main_beam",
    )
    primary_link.visual(
        Box((ELBOW_YOKE_L, ELBOW_CHEEK_T, ELBOW_YOKE_H)),
        origin=Origin(xyz=(LINK1_FORK_START + ELBOW_YOKE_L / 2.0, ELBOW_GAP / 2.0 + ELBOW_CHEEK_T / 2.0, 0.0)),
        material="link_amber",
        name="fork_left",
    )
    primary_link.visual(
        Box((ELBOW_YOKE_L, ELBOW_CHEEK_T, ELBOW_YOKE_H)),
        origin=Origin(xyz=(LINK1_FORK_START + ELBOW_YOKE_L / 2.0, -ELBOW_GAP / 2.0 - ELBOW_CHEEK_T / 2.0, 0.0)),
        material="link_amber",
        name="fork_right",
    )
    primary_link.visual(
        Cylinder(radius=ELBOW_PAD_R, length=ELBOW_CHEEK_T),
        origin=Origin(
            xyz=(LINK1_LENGTH, ELBOW_GAP / 2.0 + ELBOW_CHEEK_T / 2.0, 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="link_amber",
        name="elbow_left_boss",
    )
    primary_link.visual(
        Cylinder(radius=ELBOW_PAD_R, length=ELBOW_CHEEK_T),
        origin=Origin(
            xyz=(LINK1_LENGTH, -ELBOW_GAP / 2.0 - ELBOW_CHEEK_T / 2.0, 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="link_amber",
        name="elbow_right_boss",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((LINK1_LENGTH, LINK1_W, LINK1_H)),
        mass=1.6,
        origin=Origin(xyz=(LINK1_LENGTH / 2.0, 0.0, 0.0)),
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        Cylinder(radius=LINK2_ROOT_SPINDLE_R, length=LINK2_ROOT_SPINDLE_LEN),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material="link_amber",
        name="elbow_spindle",
    )
    terminal_link.visual(
        Box((LINK2_ROOT_NECK_L, LINK2_ROOT_NECK_W, LINK2_ROOT_NECK_H)),
        origin=Origin(xyz=(LINK2_ROOT_NECK_L / 2.0, 0.0, 0.0)),
        material="link_amber",
        name="terminal_neck",
    )
    terminal_link.visual(
        Box((LINK2_BEAM_LEN, LINK2_W, LINK2_H)),
        origin=Origin(xyz=(LINK2_BEAM_START + LINK2_BEAM_LEN / 2.0, 0.0, 0.0)),
        material="link_amber",
        name="terminal_beam",
    )
    terminal_link.visual(
        Box((END_TAB_L, END_TAB_W, END_TAB_T)),
        origin=Origin(xyz=(END_TAB_CENTER_X, 0.0, 0.0)),
        material="link_amber",
        name="end_tab",
    )
    terminal_link.inertial = Inertial.from_geometry(
        Box((0.240, LINK2_W, LINK2_H)),
        mass=1.0,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_primary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
        ),
    )
    model.articulation(
        "primary_to_terminal",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=terminal_link,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.4,
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    expected_parts = {"base", "primary_link", "terminal_link"}
    expected_joints = {"base_to_primary", "primary_to_terminal"}
    present_parts = {part.name for part in object_model.parts}
    present_joints = {joint.name for joint in object_model.articulations}

    ctx.check(
        "expected_parts_present",
        expected_parts.issubset(present_parts),
        details=f"present={sorted(present_parts)}",
    )
    ctx.check(
        "expected_joints_present",
        expected_joints.issubset(present_joints),
        details=f"present={sorted(present_joints)}",
    )
    if not expected_parts.issubset(present_parts) or not expected_joints.issubset(present_joints):
        return ctx.report()

    base = object_model.get_part("base")
    primary_link = object_model.get_part("primary_link")
    terminal_link = object_model.get_part("terminal_link")
    shoulder = object_model.get_articulation("base_to_primary")
    elbow = object_model.get_articulation("primary_to_terminal")

    axes_parallel = all(abs(a - b) < 1e-9 for a, b in zip(shoulder.axis, elbow.axis))
    axes_are_pitch = (
        abs(abs(shoulder.axis[1]) - 1.0) < 1e-9
        and abs(shoulder.axis[0]) < 1e-9
        and abs(shoulder.axis[2]) < 1e-9
    )
    ctx.check(
        "parallel_hinge_axes",
        axes_parallel and axes_are_pitch,
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )

    ctx.expect_contact(base, primary_link, contact_tol=5e-4, name="shoulder_joint_contact")
    ctx.expect_contact(primary_link, terminal_link, contact_tol=5e-4, name="elbow_joint_contact")
    ctx.expect_origin_distance(
        primary_link,
        terminal_link,
        axes="x",
        min_dist=LINK1_LENGTH - 1e-3,
        max_dist=LINK1_LENGTH + 1e-3,
        name="link1_center_spacing",
    )
    ctx.expect_origin_gap(
        primary_link,
        base,
        axis="z",
        min_gap=SHOULDER_Z - 1e-3,
        max_gap=SHOULDER_Z + 1e-3,
        name="shoulder_height",
    )

    rest_primary_aabb = ctx.part_world_aabb(primary_link)
    rest_terminal_aabb = ctx.part_world_aabb(terminal_link)
    if rest_primary_aabb is not None:
        with ctx.pose({shoulder: 0.85}):
            raised_primary_aabb = ctx.part_world_aabb(primary_link)
        primary_raised = (
            raised_primary_aabb is not None and raised_primary_aabb[1][2] > rest_primary_aabb[1][2] + 0.045
        )
        ctx.check(
            "shoulder_positive_raises_primary",
            primary_raised,
            details=f"rest_max_z={rest_primary_aabb[1][2]:.4f}, raised_max_z={None if raised_primary_aabb is None else round(raised_primary_aabb[1][2], 4)}",
        )
    if rest_terminal_aabb is not None:
        with ctx.pose({elbow: 0.95}):
            raised_terminal_aabb = ctx.part_world_aabb(terminal_link)
        terminal_raised = (
            raised_terminal_aabb is not None and raised_terminal_aabb[1][2] > rest_terminal_aabb[1][2] + 0.040
        )
        ctx.check(
            "elbow_positive_raises_terminal",
            terminal_raised,
            details=f"rest_max_z={rest_terminal_aabb[1][2]:.4f}, raised_max_z={None if raised_terminal_aabb is None else round(raised_terminal_aabb[1][2], 4)}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
