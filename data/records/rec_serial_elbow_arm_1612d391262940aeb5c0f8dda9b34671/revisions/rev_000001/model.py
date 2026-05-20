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


BASE_L = 0.36
BASE_W = 0.24
BASE_T = 0.028

SHOULDER_AXIS_X = 0.03
SHOULDER_AXIS_Z = 0.34
STAND_PLATE_T = 0.012
SHOULDER_GAP = 0.052
STAND_SIDE_Y = SHOULDER_GAP / 2.0 + STAND_PLATE_T / 2.0
STAND_TOTAL_W = SHOULDER_GAP + 2.0 * STAND_PLATE_T

UPPER_LEN = 0.27
ELBOW_GAP = 0.046
ELBOW_PLATE_T = 0.010
ELBOW_SIDE_Y = ELBOW_GAP / 2.0 + ELBOW_PLATE_T / 2.0

FOREARM_PLATE_X = 0.232
FOREARM_PLATE_T = 0.012
FOREARM_PLATE_W = 0.10
FOREARM_PLATE_H = 0.08

PIN_HOLE_R = 0.007
STAND_BOSS_R = 0.023
LINK_BOSS_R = 0.019
BOSS_LEN = 0.007
SHOULDER_BARREL_LEN = SHOULDER_GAP
ELBOW_BARREL_LEN = ELBOW_GAP


CYLINDER_Y = Origin(rpy=(1.5707963267948966, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_elbow_arm_fixture")

    stand_material = model.material("stand_steel", color=(0.26, 0.28, 0.30))
    link_material = model.material("painted_link", color=(0.45, 0.50, 0.57))

    stand = model.part("stand")
    stand.visual(Box((BASE_L, BASE_W, BASE_T)), origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)), material=stand_material, name="base_plate")
    stand.visual(Box((0.08, STAND_TOTAL_W, 0.224)), origin=Origin(xyz=(-0.055, 0.0, 0.138)), material=stand_material, name="mast")
    stand.visual(Box((0.09, STAND_TOTAL_W, 0.042)), origin=Origin(xyz=(-0.02, 0.0, 0.269)), material=stand_material, name="upper_crossbrace")
    stand.visual(Box((0.13, STAND_PLATE_T, 0.12)), origin=Origin(xyz=(0.0, STAND_SIDE_Y, 0.33)), material=stand_material, name="left_cheek")
    stand.visual(Box((0.13, STAND_PLATE_T, 0.12)), origin=Origin(xyz=(0.0, -STAND_SIDE_Y, 0.33)), material=stand_material, name="right_cheek")
    stand.visual(Box((0.05, STAND_TOTAL_W, 0.024)), origin=Origin(xyz=(-0.055, 0.0, 0.305)), material=stand_material, name="top_tie")
    stand.visual(
        Cylinder(radius=STAND_BOSS_R, length=BOSS_LEN),
        origin=Origin(xyz=(SHOULDER_AXIS_X, STAND_SIDE_Y + STAND_PLATE_T / 2.0 + BOSS_LEN / 2.0, SHOULDER_AXIS_Z), rpy=CYLINDER_Y.rpy),
        material=stand_material,
        name="left_shoulder_boss",
    )
    stand.visual(
        Cylinder(radius=STAND_BOSS_R, length=BOSS_LEN),
        origin=Origin(xyz=(SHOULDER_AXIS_X, -STAND_SIDE_Y - STAND_PLATE_T / 2.0 - BOSS_LEN / 2.0, SHOULDER_AXIS_Z), rpy=CYLINDER_Y.rpy),
        material=stand_material,
        name="right_shoulder_boss",
    )
    stand.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.40)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.017, length=SHOULDER_BARREL_LEN),
        origin=Origin(rpy=CYLINDER_Y.rpy),
        material=link_material,
        name="shoulder_barrel",
    )
    upper_link.visual(
        Box((0.075, 0.020, 0.038)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=link_material,
        name="root_bridge",
    )
    upper_link.visual(
        Box((0.16, 0.020, 0.040)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=link_material,
        name="main_beam",
    )
    upper_link.visual(
        Box((0.03, 0.020, 0.032)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=link_material,
        name="elbow_bridge",
    )
    upper_link.visual(
        Box((0.050, ELBOW_GAP + 2.0 * ELBOW_PLATE_T, 0.010)),
        origin=Origin(xyz=(0.247, 0.0, 0.024)),
        material=link_material,
        name="top_bridge",
    )
    upper_link.visual(
        Box((0.050, ELBOW_GAP + 2.0 * ELBOW_PLATE_T, 0.010)),
        origin=Origin(xyz=(0.247, 0.0, -0.024)),
        material=link_material,
        name="bottom_bridge",
    )
    upper_link.visual(
        Box((0.06, ELBOW_PLATE_T, 0.052)),
        origin=Origin(xyz=(UPPER_LEN, ELBOW_SIDE_Y, 0.0)),
        material=link_material,
        name="left_fork",
    )
    upper_link.visual(
        Box((0.06, ELBOW_PLATE_T, 0.052)),
        origin=Origin(xyz=(UPPER_LEN, -ELBOW_SIDE_Y, 0.0)),
        material=link_material,
        name="right_fork",
    )
    upper_link.visual(
        Cylinder(radius=LINK_BOSS_R, length=BOSS_LEN),
        origin=Origin(xyz=(UPPER_LEN, ELBOW_SIDE_Y + ELBOW_PLATE_T / 2.0 + BOSS_LEN / 2.0 - 0.001, 0.0), rpy=CYLINDER_Y.rpy),
        material=link_material,
        name="left_elbow_boss",
    )
    upper_link.visual(
        Cylinder(radius=LINK_BOSS_R, length=BOSS_LEN),
        origin=Origin(xyz=(UPPER_LEN, -ELBOW_SIDE_Y - ELBOW_PLATE_T / 2.0 - BOSS_LEN / 2.0 + 0.001, 0.0), rpy=CYLINDER_Y.rpy),
        material=link_material,
        name="right_elbow_boss",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.30, 0.08, 0.07)),
        mass=3.2,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.0155, length=ELBOW_BARREL_LEN),
        origin=Origin(rpy=CYLINDER_Y.rpy),
        material=link_material,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.045, 0.018, 0.028)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=link_material,
        name="forearm_root",
    )
    forearm.visual(
        Box((0.150, 0.018, 0.034)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=link_material,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.028, 0.018, 0.024)),
        origin=Origin(xyz=(0.214, 0.0, 0.0)),
        material=link_material,
        name="wrist_stem",
    )
    forearm.visual(
        Box((0.016, 0.036, 0.056)),
        origin=Origin(xyz=(0.228, 0.0, 0.0)),
        material=link_material,
        name="plate_web",
    )
    forearm.visual(
        Box((FOREARM_PLATE_T, FOREARM_PLATE_W, FOREARM_PLATE_H)),
        origin=Origin(xyz=(0.242, 0.0, 0.0)),
        material=link_material,
        name="end_plate",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.25, 0.11, 0.08)),
        mass=2.3,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-0.7, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=1.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

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
        "shoulder_axis_is_pitch",
        tuple(round(v, 3) for v in shoulder.axis) == (0.0, -1.0, 0.0),
        f"expected shoulder axis (0, -1, 0), got {shoulder.axis}",
    )
    ctx.check(
        "elbow_axis_is_pitch",
        tuple(round(v, 3) for v in elbow.axis) == (0.0, -1.0, 0.0),
        f"expected elbow axis (0, -1, 0), got {elbow.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_contact(stand, upper_link, name="shoulder_joint_contact")
        ctx.expect_contact(upper_link, forearm, name="elbow_joint_contact")
        rest_forearm_origin = ctx.part_world_position(forearm)
        rest_forearm_aabb = ctx.part_world_aabb(forearm)

    with ctx.pose({shoulder: 0.9, elbow: 0.0}):
        raised_forearm_origin = ctx.part_world_position(forearm)

    with ctx.pose({shoulder: 0.0, elbow: 1.05}):
        flexed_forearm_aabb = ctx.part_world_aabb(forearm)

    shoulder_ok = (
        rest_forearm_origin is not None
        and raised_forearm_origin is not None
        and raised_forearm_origin[2] > rest_forearm_origin[2] + 0.12
    )
    ctx.check(
        "shoulder_positive_lifts_elbow",
        shoulder_ok,
        f"rest forearm origin={rest_forearm_origin}, raised origin={raised_forearm_origin}",
    )

    elbow_ok = (
        rest_forearm_aabb is not None
        and flexed_forearm_aabb is not None
        and flexed_forearm_aabb[1][2] > rest_forearm_aabb[1][2] + 0.05
    )
    ctx.check(
        "elbow_positive_lifts_end_plate",
        elbow_ok,
        f"rest forearm aabb={rest_forearm_aabb}, flexed forearm aabb={flexed_forearm_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
