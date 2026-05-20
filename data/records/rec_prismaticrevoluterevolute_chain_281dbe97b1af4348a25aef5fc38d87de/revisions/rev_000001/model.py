from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_W = 0.34
BASE_D = 0.26
BASE_H = 0.04

COLUMN_W = 0.10
COLUMN_D = 0.12
COLUMN_H = 0.94

RAIL_D = 0.03
RAIL_W = 0.068
RAIL_H = 0.68
RAIL_Z0 = 0.16

CARRIAGE_W = 0.12
CARRIAGE_D = 0.11
CARRIAGE_H = 0.18
CARRIAGE_REAR_W = 0.09
CARRIAGE_X = COLUMN_W / 2 + RAIL_D + CARRIAGE_REAR_W
CARRIAGE_Z0 = 0.31
CARRIAGE_TRAVEL = 0.42
CARRIAGE_BLOCK_FRONT_CLEAR = 0.016
CARRIAGE_BLOCK_W = CARRIAGE_REAR_W - CARRIAGE_BLOCK_FRONT_CLEAR

LINK_T = 0.016
CLEVIS_EAR_T = 0.010
CLEVIS_GAP = LINK_T
CLEVIS_CENTER_Z = CLEVIS_GAP / 2 + CLEVIS_EAR_T / 2

ROOT_FORK_L = 0.034
ROOT_FORK_W = 0.072
ELBOW_FORK_L = 0.034

UPPER_LEN = 0.20
UPPER_ROOT_R = 0.042
UPPER_BEAM_W = 0.056
UPPER_ELBOW_R = 0.028

FOREARM_LEN = 0.17
FOREARM_ROOT_R = 0.026
FOREARM_BEAM_W = 0.046
FOREARM_TIP_R = 0.022


def make_mast_frame() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))

    column = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BASE_H))
        .box(COLUMN_W, COLUMN_D, COLUMN_H, centered=(True, True, False))
    )

    rail = (
        cq.Workplane("XY")
        .transformed(offset=(COLUMN_W / 2 + RAIL_D / 2, 0.0, BASE_H + RAIL_Z0))
        .box(RAIL_D, RAIL_W, RAIL_H, centered=(True, True, False))
    )

    top_cap = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BASE_H + COLUMN_H - 0.04))
        .box(0.13, 0.14, 0.04, centered=(True, True, False))
    )

    return base.union(column).union(rail).union(top_cap)


def make_carriage() -> cq.Workplane:
    rear_block = (
        cq.Workplane("XY")
        .transformed(offset=(-CARRIAGE_REAR_W / 2, 0.0, 0.0))
        .box(CARRIAGE_REAR_W, CARRIAGE_D, CARRIAGE_H)
    )

    top_ear = (
        cq.Workplane("XY")
        .transformed(offset=(ROOT_FORK_L / 2 - 0.001, 0.0, CLEVIS_CENTER_Z))
        .box(ROOT_FORK_L + 0.002, ROOT_FORK_W, CLEVIS_EAR_T)
    )
    bottom_ear = (
        cq.Workplane("XY")
        .transformed(offset=(ROOT_FORK_L / 2 - 0.001, 0.0, -CLEVIS_CENTER_Z))
        .box(ROOT_FORK_L + 0.002, ROOT_FORK_W, CLEVIS_EAR_T)
    )

    return rear_block.union(top_ear).union(bottom_ear)


def make_upper_link() -> cq.Workplane:
    shoulder_disc = cq.Workplane("XY").circle(UPPER_ROOT_R).extrude(LINK_T / 2, both=True)
    shoulder_cut = (
        cq.Workplane("XY")
        .transformed(offset=(-2 * UPPER_ROOT_R, 0.0, 0.0))
        .box(2 * UPPER_ROOT_R, 2.5 * UPPER_ROOT_R, 4 * LINK_T, centered=(False, True, True))
    )
    shoulder_disc = shoulder_disc.cut(shoulder_cut)

    beam = cq.Workplane("XY").transformed(offset=((UPPER_LEN - 0.030) / 2, 0.0, 0.0)).box(
        UPPER_LEN - 0.030, UPPER_BEAM_W, LINK_T
    )

    top_fork = (
        cq.Workplane("XY")
        .transformed(offset=(UPPER_LEN + 0.015, 0.0, CLEVIS_CENTER_Z))
        .box(0.032, 0.052, CLEVIS_EAR_T)
    )
    bottom_fork = (
        cq.Workplane("XY")
        .transformed(offset=(UPPER_LEN + 0.015, 0.0, -CLEVIS_CENTER_Z))
        .box(0.032, 0.052, CLEVIS_EAR_T)
    )

    return shoulder_disc.union(beam).union(top_fork).union(bottom_fork)


def make_forearm() -> cq.Workplane:
    root_disc = cq.Workplane("XY").circle(FOREARM_ROOT_R).extrude(LINK_T / 2, both=True)
    root_cut = (
        cq.Workplane("XY")
        .transformed(offset=(-2 * FOREARM_ROOT_R, 0.0, 0.0))
        .box(2 * FOREARM_ROOT_R, 2.5 * FOREARM_ROOT_R, 4 * LINK_T, centered=(False, True, True))
    )
    root_disc = root_disc.cut(root_cut)

    beam = (
        cq.Workplane("XY")
        .transformed(offset=(FOREARM_LEN / 2, 0.0, 0.0))
        .box(FOREARM_LEN, FOREARM_BEAM_W, LINK_T)
    )
    tip_disc = (
        cq.Workplane("XY")
        .transformed(offset=(FOREARM_LEN, 0.0, 0.0))
        .circle(FOREARM_TIP_R)
        .extrude(LINK_T / 2, both=True)
    )
    tip_pad = (
        cq.Workplane("XY")
        .transformed(offset=(FOREARM_LEN + 0.018, 0.0, 0.0))
        .box(0.036, 0.040, 0.022)
    )
    return root_disc.union(beam).union(tip_disc).union(tip_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_slide_side_arm")

    model.material("mast_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("carriage_gray", rgba=(0.52, 0.56, 0.60, 1.0))
    model.material("arm_orange", rgba=(0.92, 0.56, 0.17, 1.0))
    model.material("tool_dark", rgba=(0.15, 0.15, 0.16, 1.0))

    mast_frame = model.part("mast_frame")
    mast_frame.visual(
        Box((BASE_W, BASE_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2)),
        name="base_plate",
        material="mast_dark",
    )
    mast_frame.visual(
        Box((COLUMN_W, COLUMN_D, COLUMN_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLUMN_H / 2)),
        name="column_shell",
        material="mast_dark",
    )
    mast_frame.visual(
        Box((RAIL_D, RAIL_W, RAIL_H)),
        origin=Origin(
            xyz=(COLUMN_W / 2 + RAIL_D / 2, 0.0, BASE_H + RAIL_Z0 + RAIL_H / 2),
        ),
        name="guide_rail",
        material="carriage_gray",
    )
    mast_frame.visual(
        Box((0.13, 0.14, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + COLUMN_H - 0.02)),
        name="top_cap",
        material="mast_dark",
    )
    mast_frame.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_H + COLUMN_H)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + COLUMN_H) / 2)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_BLOCK_W, 0.086, CARRIAGE_H)),
        origin=Origin(
            xyz=(-CARRIAGE_REAR_W + CARRIAGE_BLOCK_W / 2, 0.0, 0.0),
        ),
        name="slide_block",
        material="carriage_gray",
    )
    carriage.visual(
        Box((ROOT_FORK_L, ROOT_FORK_W, CLEVIS_EAR_T)),
        origin=Origin(xyz=(-ROOT_FORK_L / 2, 0.0, CLEVIS_CENTER_Z)),
        name="upper_ear",
        material="carriage_gray",
    )
    carriage.visual(
        Box((ROOT_FORK_L, ROOT_FORK_W, CLEVIS_EAR_T)),
        origin=Origin(xyz=(-ROOT_FORK_L / 2, 0.0, -CLEVIS_CENTER_Z)),
        name="lower_ear",
        material="carriage_gray",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_REAR_W + ROOT_FORK_L, CARRIAGE_D, CARRIAGE_H)),
        mass=4.0,
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=UPPER_ROOT_R, length=LINK_T),
        origin=Origin(xyz=(UPPER_ROOT_R, 0.0, 0.0)),
        name="shoulder_hub",
        material="arm_orange",
    )
    upper_link.visual(
        Box((UPPER_LEN - UPPER_ROOT_R - 0.018, UPPER_BEAM_W, LINK_T)),
        origin=Origin(xyz=((UPPER_LEN + UPPER_ROOT_R - 0.018) / 2, 0.0, 0.0)),
        name="upper_beam",
        material="arm_orange",
    )
    upper_link.visual(
        Box((ELBOW_FORK_L, 0.052, CLEVIS_EAR_T)),
        origin=Origin(xyz=(UPPER_LEN - ELBOW_FORK_L / 2, 0.0, CLEVIS_CENTER_Z)),
        name="elbow_upper_ear",
        material="arm_orange",
    )
    upper_link.visual(
        Box((ELBOW_FORK_L, 0.052, CLEVIS_EAR_T)),
        origin=Origin(xyz=(UPPER_LEN - ELBOW_FORK_L / 2, 0.0, -CLEVIS_CENTER_Z)),
        name="elbow_lower_ear",
        material="arm_orange",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((UPPER_LEN + UPPER_ROOT_R, UPPER_BEAM_W, 0.040)),
        mass=2.0,
        origin=Origin(xyz=((UPPER_LEN + UPPER_ROOT_R) / 2, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=FOREARM_ROOT_R, length=LINK_T),
        origin=Origin(xyz=(FOREARM_ROOT_R, 0.0, 0.0)),
        name="elbow_hub",
        material="tool_dark",
    )
    forearm.visual(
        Box((FOREARM_LEN - FOREARM_ROOT_R, FOREARM_BEAM_W, LINK_T)),
        origin=Origin(xyz=(FOREARM_ROOT_R + (FOREARM_LEN - FOREARM_ROOT_R) / 2, 0.0, 0.0)),
        name="forearm_beam",
        material="tool_dark",
    )
    forearm.visual(
        Cylinder(radius=FOREARM_TIP_R, length=0.022),
        origin=Origin(xyz=(FOREARM_LEN + 0.012, 0.0, 0.0)),
        name="tool_tip",
        material="tool_dark",
    )
    forearm.visual(
        Box((0.028, 0.038, 0.022)),
        origin=Origin(xyz=(FOREARM_LEN + 0.030, 0.0, 0.0)),
        name="tip_pad",
        material="tool_dark",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LEN + 0.036, FOREARM_BEAM_W, 0.032)),
        mass=1.5,
        origin=Origin(xyz=((FOREARM_LEN + 0.036) / 2, 0.0, 0.0)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_X, 0.0, CARRIAGE_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.6,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "upper_link_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.9,
            lower=-2.0,
            upper=0.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast_frame = object_model.get_part("mast_frame")
    carriage = object_model.get_part("carriage")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")

    slide = object_model.get_articulation("mast_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_link")
    elbow = object_model.get_articulation("upper_link_to_forearm")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=5e-06)
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
        "parts_present",
        all(part is not None for part in (mast_frame, carriage, upper_link, forearm)),
        "Expected mast_frame, carriage, upper_link, and forearm parts.",
    )
    ctx.check(
        "joint_axes_match_mechanism",
        slide.axis == (0.0, 0.0, 1.0)
        and shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0),
        f"Unexpected joint axes: slide={slide.axis}, shoulder={shoulder.axis}, elbow={elbow.axis}",
    )
    ctx.check(
        "joint_limits_are_realistic",
        slide.motion_limits is not None
        and shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.35
        and shoulder.motion_limits.lower is not None
        and shoulder.motion_limits.upper is not None
        and shoulder.motion_limits.lower <= -1.0
        and shoulder.motion_limits.upper >= 1.0
        and elbow.motion_limits.lower is not None
        and elbow.motion_limits.lower <= -1.8
        and elbow.motion_limits.upper is not None
        and elbow.motion_limits.upper >= 0.1,
        "Prismatic travel or arm swing limits are too small for the requested mechanism.",
    )

    ctx.expect_contact(mast_frame, carriage, name="carriage_contacts_mast")
    ctx.expect_contact(carriage, upper_link, name="shoulder_mount_contacts")
    ctx.expect_contact(upper_link, forearm, name="elbow_mount_contacts")

    carriage_z0 = ctx.part_world_position(carriage)[2]
    with ctx.pose({slide: 0.34}):
        carriage_z1 = ctx.part_world_position(carriage)[2]
        ctx.check(
            "carriage_moves_prismatically_upward",
            carriage_z1 > carriage_z0 + 0.30,
            f"Expected carriage to move upward substantially, got z0={carriage_z0}, z1={carriage_z1}",
        )

    with ctx.pose({shoulder: 0.85, elbow: -1.1}):
        forearm_pos = ctx.part_world_position(forearm)
        upper_pos = ctx.part_world_position(upper_link)
        ctx.check(
            "side_arm_swings_away_from_mast",
            forearm_pos[1] > upper_pos[1] + 0.10,
            f"Expected forearm origin to swing to +Y side, got upper={upper_pos}, forearm={forearm_pos}",
        )

    straight_forearm_aabb = ctx.part_world_aabb(forearm)
    with ctx.pose({elbow: -1.3}):
        folded_forearm_aabb = ctx.part_world_aabb(forearm)
        ctx.check(
            "elbow_folds_forearm_back",
            folded_forearm_aabb[1][0] < straight_forearm_aabb[1][0] - 0.05,
            (
                "Expected folded forearm to reduce reach in +X, "
                f"got straight_max_x={straight_forearm_aabb[1][0]}, "
                f"folded_max_x={folded_forearm_aabb[1][0]}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
