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
    Sphere,
    TestContext,
    TestReport,
)


ROOT_BACK = 0.032
ROOT_FRONT = 0.008
ROOT_OUTER_WIDTH = 0.028
ROOT_GAP_WIDTH = 0.014
ROOT_HEIGHT = 0.022
ROOT_BOSS_RADIUS = 0.008
ROOT_CHEEK_THICKNESS = (ROOT_OUTER_WIDTH - ROOT_GAP_WIDTH) / 2.0
ROOT_CHEEK_HEIGHT = 0.018
ROOT_CHEEK_X0 = -0.010
ROOT_CHEEK_X1 = 0.008

PROXIMAL_LENGTH = 0.052
PROXIMAL_BODY_WIDTH = 0.012
PROXIMAL_BODY_HEIGHT = 0.014
PROXIMAL_EAR_WIDTH = 0.020
PROXIMAL_ROOT_BARREL_RADIUS = 0.006

MIDDLE_LENGTH = 0.040
MIDDLE_BODY_WIDTH = 0.010
MIDDLE_BODY_HEIGHT = 0.012
MIDDLE_BARREL_WIDTH = 0.012
MIDDLE_EAR_WIDTH = 0.018
MIDDLE_BARREL_RADIUS = 0.0052

DISTAL_LENGTH = 0.030
DISTAL_BODY_WIDTH = 0.009
DISTAL_BODY_HEIGHT = 0.011
DISTAL_BARREL_WIDTH = 0.010
DISTAL_BARREL_RADIUS = 0.0045
TIP_PAD_LENGTH = 0.017
TIP_PAD_WIDTH = 0.013
TIP_PAD_HEIGHT = 0.012

PROXIMAL_EAR_THICKNESS = (PROXIMAL_EAR_WIDTH - MIDDLE_BARREL_WIDTH) / 2.0
MIDDLE_EAR_THICKNESS = (MIDDLE_EAR_WIDTH - DISTAL_BARREL_WIDTH) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_digit_assembly")

    model.material("root_finish", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("link_finish", rgba=(0.74, 0.77, 0.80, 1.0))
    model.material("pad_finish", rgba=(0.16, 0.17, 0.18, 1.0))

    fork_root = model.part("fork_root")
    fork_root.visual(
        Box((ROOT_BACK - 0.010, ROOT_OUTER_WIDTH, ROOT_HEIGHT)),
        origin=Origin(xyz=(-(ROOT_BACK + 0.010) / 2.0, 0.0, 0.0)),
        material="root_finish",
        name="root_backbone",
    )
    for y_sign in (-1.0, 1.0):
        cheek_y = y_sign * (ROOT_GAP_WIDTH / 2.0 + ROOT_CHEEK_THICKNESS / 2.0)
        fork_root.visual(
            Box((ROOT_CHEEK_X1 - ROOT_CHEEK_X0, ROOT_CHEEK_THICKNESS, ROOT_CHEEK_HEIGHT)),
            origin=Origin(xyz=((ROOT_CHEEK_X0 + ROOT_CHEEK_X1) / 2.0, cheek_y, 0.0)),
            material="root_finish",
            name=f"root_cheek_{'right' if y_sign < 0.0 else 'left'}",
        )
        fork_root.visual(
            Cylinder(radius=ROOT_BOSS_RADIUS, length=ROOT_CHEEK_THICKNESS),
            origin=Origin(xyz=(0.0, cheek_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="root_finish",
            name=f"root_boss_{'right' if y_sign < 0.0 else 'left'}",
        )
    fork_root.visual(
        Box((0.010, ROOT_OUTER_WIDTH, ROOT_HEIGHT * 0.45)),
        origin=Origin(xyz=(-0.017, 0.0, -ROOT_HEIGHT * 0.26)),
        material="root_finish",
        name="root_mount_rib",
    )
    fork_root.inertial = Inertial.from_geometry(
        Box((ROOT_BACK + ROOT_FRONT, ROOT_OUTER_WIDTH, ROOT_HEIGHT)),
        mass=0.11,
        origin=Origin(xyz=((ROOT_FRONT - ROOT_BACK) / 2.0, 0.0, 0.0)),
    )

    proximal = model.part("proximal_link")
    proximal.visual(
        Cylinder(radius=PROXIMAL_ROOT_BARREL_RADIUS, length=ROOT_GAP_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_finish",
        name="proximal_root_barrel",
    )
    proximal.visual(
        Box((0.040, PROXIMAL_BODY_WIDTH, PROXIMAL_BODY_HEIGHT)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material="link_finish",
        name="proximal_body",
    )
    for y_sign in (-1.0, 1.0):
        ear_y = y_sign * (MIDDLE_BARREL_WIDTH / 2.0 + PROXIMAL_EAR_THICKNESS / 2.0)
        proximal.visual(
            Box((0.013, PROXIMAL_EAR_THICKNESS, PROXIMAL_BODY_HEIGHT * 0.90)),
            origin=Origin(xyz=(PROXIMAL_LENGTH - 0.0065, ear_y, 0.0)),
            material="link_finish",
            name=f"proximal_ear_{'right' if y_sign < 0.0 else 'left'}",
        )
        proximal.visual(
            Cylinder(radius=MIDDLE_BARREL_RADIUS, length=PROXIMAL_EAR_THICKNESS),
            origin=Origin(
                xyz=(PROXIMAL_LENGTH, ear_y, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="link_finish",
            name=f"proximal_knuckle_{'right' if y_sign < 0.0 else 'left'}",
        )
    proximal.visual(
        Box((0.010, PROXIMAL_EAR_WIDTH, PROXIMAL_BODY_HEIGHT * 0.55)),
        origin=Origin(xyz=(PROXIMAL_LENGTH - 0.011, 0.0, -0.0015)),
        material="link_finish",
        name="proximal_yoke_bridge",
    )
    proximal.inertial = Inertial.from_geometry(
        Box((PROXIMAL_LENGTH + 0.012, PROXIMAL_EAR_WIDTH, PROXIMAL_BODY_HEIGHT)),
        mass=0.045,
        origin=Origin(xyz=((PROXIMAL_LENGTH + 0.012) / 2.0, 0.0, 0.0)),
    )

    middle = model.part("middle_link")
    middle.visual(
        Cylinder(radius=MIDDLE_BARREL_RADIUS, length=MIDDLE_BARREL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_finish",
        name="middle_root_barrel",
    )
    middle.visual(
        Box((0.031, MIDDLE_BODY_WIDTH, MIDDLE_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0205, 0.0, 0.0)),
        material="link_finish",
        name="middle_body",
    )
    for y_sign in (-1.0, 1.0):
        ear_y = y_sign * (DISTAL_BARREL_WIDTH / 2.0 + MIDDLE_EAR_THICKNESS / 2.0)
        middle.visual(
            Box((0.012, MIDDLE_EAR_THICKNESS, MIDDLE_BODY_HEIGHT * 0.90)),
            origin=Origin(xyz=(MIDDLE_LENGTH - 0.006, ear_y, 0.0)),
            material="link_finish",
            name=f"middle_ear_{'right' if y_sign < 0.0 else 'left'}",
        )
        middle.visual(
            Cylinder(radius=DISTAL_BARREL_RADIUS, length=MIDDLE_EAR_THICKNESS),
            origin=Origin(
                xyz=(MIDDLE_LENGTH, ear_y, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material="link_finish",
            name=f"middle_knuckle_{'right' if y_sign < 0.0 else 'left'}",
        )
    middle.visual(
        Box((0.009, MIDDLE_EAR_WIDTH, MIDDLE_BODY_HEIGHT * 0.52)),
        origin=Origin(xyz=(MIDDLE_LENGTH - 0.010, 0.0, -0.0012)),
        material="link_finish",
        name="middle_yoke_bridge",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH + 0.010, MIDDLE_EAR_WIDTH, MIDDLE_BODY_HEIGHT)),
        mass=0.031,
        origin=Origin(xyz=((MIDDLE_LENGTH + 0.010) / 2.0, 0.0, 0.0)),
    )

    distal = model.part("distal_link")
    distal.visual(
        Cylinder(radius=DISTAL_BARREL_RADIUS, length=DISTAL_BARREL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="link_finish",
        name="distal_root_barrel",
    )
    distal.visual(
        Box((0.024, DISTAL_BODY_WIDTH, DISTAL_BODY_HEIGHT)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material="link_finish",
        name="distal_body",
    )
    distal.visual(
        Box((TIP_PAD_LENGTH, TIP_PAD_WIDTH, TIP_PAD_HEIGHT)),
        origin=Origin(xyz=(DISTAL_LENGTH + TIP_PAD_LENGTH / 2.0 - 0.002, 0.0, -0.0004)),
        material="pad_finish",
        name="tip_pad_body",
    )
    distal.visual(
        Sphere(radius=TIP_PAD_HEIGHT * 0.50),
        origin=Origin(xyz=(DISTAL_LENGTH + TIP_PAD_LENGTH - 0.002, 0.0, -0.0004)),
        material="pad_finish",
        name="tip_pad_nose",
    )
    distal.inertial = Inertial.from_geometry(
        Box((DISTAL_LENGTH + TIP_PAD_LENGTH + 0.012, TIP_PAD_WIDTH, TIP_PAD_HEIGHT)),
        mass=0.022,
        origin=Origin(xyz=((DISTAL_LENGTH + TIP_PAD_LENGTH + 0.012) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "fork_to_proximal",
        ArticulationType.REVOLUTE,
        parent=fork_root,
        child=proximal,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=2.5, velocity=3.0),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=1.8, velocity=3.4),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=1.4, velocity=3.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork_root = object_model.get_part("fork_root")
    proximal = object_model.get_part("proximal_link")
    middle = object_model.get_part("middle_link")
    distal = object_model.get_part("distal_link")

    root_joint = object_model.get_articulation("fork_to_proximal")
    proximal_joint = object_model.get_articulation("proximal_to_middle")
    distal_joint = object_model.get_articulation("middle_to_distal")

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

    ctx.expect_contact(fork_root, proximal, name="fork root captures the proximal barrel")
    ctx.expect_contact(proximal, middle, name="proximal yoke captures the middle barrel")
    ctx.expect_contact(middle, distal, name="middle yoke captures the distal barrel")

    ctx.expect_origin_gap(
        middle,
        proximal,
        axis="x",
        min_gap=PROXIMAL_LENGTH - 0.001,
        max_gap=PROXIMAL_LENGTH + 0.001,
        name="middle joint sits at the end of the proximal link",
    )
    ctx.expect_origin_gap(
        distal,
        middle,
        axis="x",
        min_gap=MIDDLE_LENGTH - 0.001,
        max_gap=MIDDLE_LENGTH + 0.001,
        name="distal joint sits at the end of the middle link",
    )

    rest_aabb = ctx.part_world_aabb(distal)
    with ctx.pose(
        {
            root_joint: 0.65,
            proximal_joint: 0.85,
            distal_joint: 0.70,
        }
    ):
        flex_aabb = ctx.part_world_aabb(distal)
        ctx.check(
            "positive flexion curls the fingertip upward",
            rest_aabb is not None
            and flex_aabb is not None
            and flex_aabb[1][2] > rest_aabb[1][2] + 0.020
            and flex_aabb[1][0] < rest_aabb[1][0] - 0.010,
            details=f"rest_aabb={rest_aabb}, flex_aabb={flex_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
