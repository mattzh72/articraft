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


FOOT_X = 0.34
FOOT_Y = 0.26
FOOT_H = 0.055

PEDESTAL_LOWER_X = 0.14
PEDESTAL_LOWER_Y = 0.11
PEDESTAL_LOWER_H = 0.10

PEDESTAL_UPPER_X = 0.11
PEDESTAL_UPPER_Y = 0.085
PEDESTAL_UPPER_H = 0.06

SADDLE_X = 0.18
SADDLE_Y = 0.10
SADDLE_H = 0.04

CHEEK_T = 0.032
CHEEK_D = 0.09
CHEEK_H = 0.18
CHEEK_INNER_X = 0.065

REAR_BRIDGE_X = 0.19
REAR_BRIDGE_Y = 0.03
REAR_BRIDGE_H = 0.14
REAR_BRIDGE_Y_CENTER = -0.05

AXIS_Z = FOOT_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H + 0.02 + 0.10

BOSS_LEN = 0.010
BOSS_R = 0.024

PIN_LEN = 0.010
PIN_R = 0.020
PIN_X_CENTER = 0.050

ARM_X = 0.018
ARM_Y = 0.050
ARM_Z = 0.060
ARM_Y_CENTER = 0.026
ARM_X_CENTER = 0.036

CROSSHEAD_X = 0.086
CROSSHEAD_Y = 0.024
CROSSHEAD_Z = 0.055
CROSSHEAD_Y_CENTER = 0.062

FACE_X = 0.16
FACE_Y = 0.018
FACE_Z = 0.12
FACE_Y_CENTER = 0.083


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_trunnion_module")

    support_finish = model.material("support_finish", rgba=(0.25, 0.28, 0.31, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.31, 0.34, 0.37, 1.0))
    carrier_finish = model.material("carrier_finish", rgba=(0.56, 0.58, 0.60, 1.0))
    face_finish = model.material("face_finish", rgba=(0.84, 0.86, 0.88, 1.0))

    support = model.part("tower_support")
    support.visual(
        Box((FOOT_X, FOOT_Y, FOOT_H)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H / 2.0)),
        material=support_finish,
        name="tower_foot",
    )
    support.visual(
        Box((PEDESTAL_LOWER_X, PEDESTAL_LOWER_Y, PEDESTAL_LOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + PEDESTAL_LOWER_H / 2.0)),
        material=support_finish,
        name="pedestal_lower",
    )
    support.visual(
        Box((PEDESTAL_UPPER_X, PEDESTAL_UPPER_Y, PEDESTAL_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H / 2.0)),
        material=support_finish,
        name="pedestal_upper",
    )
    support.visual(
        Box((SADDLE_X, SADDLE_Y, SADDLE_H)),
        origin=Origin(
            xyz=(0.0, 0.0, FOOT_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H + SADDLE_H / 2.0)
        ),
        material=yoke_finish,
        name="saddle_cap",
    )
    support.visual(
        Box((REAR_BRIDGE_X, REAR_BRIDGE_Y, REAR_BRIDGE_H)),
        origin=Origin(
            xyz=(
                0.0,
                REAR_BRIDGE_Y_CENTER,
                FOOT_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H + REAR_BRIDGE_H / 2.0,
            )
        ),
        material=yoke_finish,
        name="rear_bridge",
    )
    support.visual(
        Box((CHEEK_T, CHEEK_D, CHEEK_H)),
        origin=Origin(
            xyz=(
                -(CHEEK_INNER_X + CHEEK_T / 2.0),
                0.0,
                FOOT_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H + CHEEK_H / 2.0,
            )
        ),
        material=yoke_finish,
        name="left_cheek",
    )
    support.visual(
        Box((CHEEK_T, CHEEK_D, CHEEK_H)),
        origin=Origin(
            xyz=(
                CHEEK_INNER_X + CHEEK_T / 2.0,
                0.0,
                FOOT_H + PEDESTAL_LOWER_H + PEDESTAL_UPPER_H + CHEEK_H / 2.0,
            )
        ),
        material=yoke_finish,
        name="right_cheek",
    )
    support.visual(
        Cylinder(radius=BOSS_R, length=BOSS_LEN),
        origin=Origin(xyz=(-CHEEK_INNER_X + BOSS_LEN / 2.0, 0.0, AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=yoke_finish,
        name="left_boss",
    )
    support.visual(
        Cylinder(radius=BOSS_R, length=BOSS_LEN),
        origin=Origin(xyz=(CHEEK_INNER_X - BOSS_LEN / 2.0, 0.0, AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=yoke_finish,
        name="right_boss",
    )
    support.inertial = Inertial.from_geometry(
        Box((FOOT_X, FOOT_Y, AXIS_Z + CHEEK_H / 2.0)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, (AXIS_Z + CHEEK_H / 2.0) / 2.0)),
    )

    carrier = model.part("tilt_carrier")
    carrier.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(-PIN_X_CENTER, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=carrier_finish,
        name="left_pin",
    )
    carrier.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(PIN_X_CENTER, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=carrier_finish,
        name="right_pin",
    )
    carrier.visual(
        Box((ARM_X, ARM_Y, ARM_Z)),
        origin=Origin(xyz=(-ARM_X_CENTER, ARM_Y_CENTER, 0.0)),
        material=carrier_finish,
        name="left_arm",
    )
    carrier.visual(
        Box((ARM_X, ARM_Y, ARM_Z)),
        origin=Origin(xyz=(ARM_X_CENTER, ARM_Y_CENTER, 0.0)),
        material=carrier_finish,
        name="right_arm",
    )
    carrier.visual(
        Box((CROSSHEAD_X, CROSSHEAD_Y, CROSSHEAD_Z)),
        origin=Origin(xyz=(0.0, CROSSHEAD_Y_CENTER, 0.0)),
        material=carrier_finish,
        name="crosshead",
    )
    carrier.visual(
        Box((FACE_X, FACE_Y, FACE_Z)),
        origin=Origin(xyz=(0.0, FACE_Y_CENTER, 0.0)),
        material=face_finish,
        name="carried_face",
    )
    carrier.inertial = Inertial.from_geometry(
        Box((FACE_X, FACE_Y_CENTER + FACE_Y / 2.0, FACE_Z)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    model.articulation(
        "pitch_trunnion",
        ArticulationType.REVOLUTE,
        parent=support,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.30, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("tower_support")
    carrier = object_model.get_part("tilt_carrier")
    pitch = object_model.get_articulation("pitch_trunnion")

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

    ctx.expect_contact(
        support,
        carrier,
        elem_a="left_boss",
        elem_b="left_pin",
        name="left_trunnion_pin_contacts_left_boss",
    )
    ctx.expect_contact(
        support,
        carrier,
        elem_a="right_boss",
        elem_b="right_pin",
        name="right_trunnion_pin_contacts_right_boss",
    )
    ctx.expect_gap(
        carrier,
        support,
        axis="y",
        min_gap=0.020,
        positive_elem="carried_face",
        negative_elem="left_cheek",
        name="carried_face_sits_proud_of_yoke",
    )
    ctx.expect_within(
        carrier,
        support,
        axes="x",
        margin=0.005,
        inner_elem="carried_face",
        outer_elem="rear_bridge",
        name="carried_face_is_narrower_than_support_package",
    )

    closed_face = ctx.part_element_world_aabb(carrier, elem="carried_face")
    with ctx.pose({pitch: 0.90}):
        raised_face = ctx.part_element_world_aabb(carrier, elem="carried_face")

    closed_center_z = 0.5 * (closed_face[0][2] + closed_face[1][2])
    raised_center_z = 0.5 * (raised_face[0][2] + raised_face[1][2])
    ctx.check(
        "positive_pitch_raises_the_carried_face",
        raised_center_z > closed_center_z + 0.04,
        (
            f"expected positive pitch to lift the carried face; "
            f"closed_center_z={closed_center_z:.4f}, raised_center_z={raised_center_z:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
