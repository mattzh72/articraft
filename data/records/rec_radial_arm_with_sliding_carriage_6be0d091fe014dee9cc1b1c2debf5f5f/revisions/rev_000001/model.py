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


PLATE_T = 0.016
PLATE_W = 0.240
PLATE_H = 0.460

SIDE_CHEEK_L = 0.090
SIDE_CHEEK_T = 0.012
SIDE_CHEEK_H = 0.180
SIDE_CHEEK_Y = 0.052
SIDE_CHEEK_Z = -0.050

PEDESTAL_L = 0.070
PEDESTAL_W = 0.090
PEDESTAL_H = 0.074
PEDESTAL_Z = -0.053

BOLT_HEAD_R = 0.012
BOLT_HEAD_T = 0.004
BOLT_Y = 0.080
BOLT_Z = 0.145

PIVOT_X = 0.094
LOWER_DISK_R = 0.048
LOWER_DISK_T = 0.016
UPPER_DISK_R = 0.046
UPPER_DISK_T = 0.016

HUB_BLOCK_L = 0.094
HUB_BLOCK_W = 0.102
HUB_BLOCK_H = 0.046
HUB_BLOCK_X = 0.030
HUB_BLOCK_Z = 0.024

ARM_LEN = 0.620
ARM_W = 0.080
ARM_H = 0.048
ARM_X = 0.340
ARM_Z = 0.039

ARM_NOSE_L = 0.052
ARM_NOSE_W = 0.062
ARM_NOSE_H = 0.040
ARM_NOSE_X = 0.646
ARM_NOSE_Z = 0.040

GUIDE_L = 0.360
GUIDE_W = 0.038
GUIDE_H = 0.016
GUIDE_X = 0.400
GUIDE_Z = 0.070
GUIDE_TOP_Z = GUIDE_Z + (GUIDE_H / 2.0)

CARRIAGE_HOME_X = 0.390
CARRIAGE_TRAVEL = 0.100

CARR_LEN = 0.110
CARR_OUT_W = 0.118
CARR_ROOF_T = 0.018
CARR_SIDE_T = 0.018
CARR_SIDE_H = 0.050
CARR_SIDE_Z = -0.007
CARR_PAD_L = 0.060
CARR_PAD_W = 0.070
CARR_PAD_H = 0.028
CARR_PAD_Z = 0.032
CARR_STRAP_L = 0.034
CARR_STRAP_W = 0.050
CARR_STRAP_H = 0.018
CARR_STRAP_X = 0.030
CARR_STRAP_Z = 0.055


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_radial_arm_fixture")

    model.material("support_dark", rgba=(0.20, 0.23, 0.27, 1.0))
    model.material("arm_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("guide_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("fastener_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("carriage_gray", rgba=(0.48, 0.50, 0.54, 1.0))

    back_support = model.part("back_support")
    back_support.visual(
        Box((PLATE_T, PLATE_W, PLATE_H)),
        origin=Origin(xyz=(PLATE_T / 2.0, 0.0, 0.0)),
        material="support_dark",
        name="wall_plate",
    )
    back_support.visual(
        Box((SIDE_CHEEK_L, SIDE_CHEEK_T, SIDE_CHEEK_H)),
        origin=Origin(xyz=(0.057, SIDE_CHEEK_Y, SIDE_CHEEK_Z)),
        material="support_dark",
        name="left_cheek",
    )
    back_support.visual(
        Box((SIDE_CHEEK_L, SIDE_CHEEK_T, SIDE_CHEEK_H)),
        origin=Origin(xyz=(0.057, -SIDE_CHEEK_Y, SIDE_CHEEK_Z)),
        material="support_dark",
        name="right_cheek",
    )
    back_support.visual(
        Box((PEDESTAL_L, PEDESTAL_W, PEDESTAL_H)),
        origin=Origin(xyz=(PLATE_T + (PEDESTAL_L / 2.0), 0.0, PEDESTAL_Z)),
        material="support_dark",
        name="pedestal",
    )
    back_support.visual(
        Cylinder(radius=LOWER_DISK_R, length=LOWER_DISK_T),
        origin=Origin(xyz=(PIVOT_X, 0.0, -(LOWER_DISK_T / 2.0))),
        material="guide_steel",
        name="lower_disk",
    )
    for bolt_name, bolt_y, bolt_z in (
        ("upper_left_bolt", BOLT_Y, BOLT_Z),
        ("upper_right_bolt", -BOLT_Y, BOLT_Z),
        ("lower_left_bolt", BOLT_Y, -BOLT_Z),
        ("lower_right_bolt", -BOLT_Y, -BOLT_Z),
    ):
        back_support.visual(
            Cylinder(radius=BOLT_HEAD_R, length=BOLT_HEAD_T),
            origin=Origin(
                xyz=(PLATE_T + (BOLT_HEAD_T / 2.0), bolt_y, bolt_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="fastener_steel",
            name=bolt_name,
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=UPPER_DISK_R, length=UPPER_DISK_T),
        origin=Origin(xyz=(0.0, 0.0, UPPER_DISK_T / 2.0)),
        material="guide_steel",
        name="upper_disk",
    )
    arm.visual(
        Box((HUB_BLOCK_L, HUB_BLOCK_W, HUB_BLOCK_H)),
        origin=Origin(xyz=(HUB_BLOCK_X, 0.0, HUB_BLOCK_Z)),
        material="arm_gray",
        name="hub_block",
    )
    arm.visual(
        Box((ARM_LEN, ARM_W, ARM_H)),
        origin=Origin(xyz=(ARM_X, 0.0, ARM_Z)),
        material="arm_gray",
        name="beam",
    )
    arm.visual(
        Box((ARM_NOSE_L, ARM_NOSE_W, ARM_NOSE_H)),
        origin=Origin(xyz=(ARM_NOSE_X, 0.0, ARM_NOSE_Z)),
        material="arm_gray",
        name="nose",
    )
    arm.visual(
        Box((GUIDE_L, GUIDE_W, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X, 0.0, GUIDE_Z)),
        material="guide_steel",
        name="guide_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARR_LEN, CARR_OUT_W, CARR_ROOF_T)),
        origin=Origin(xyz=(0.0, 0.0, CARR_ROOF_T / 2.0)),
        material="carriage_gray",
        name="top_bridge",
    )
    carriage.visual(
        Box((CARR_LEN, CARR_SIDE_T, CARR_SIDE_H)),
        origin=Origin(
            xyz=(0.0, (CARR_OUT_W / 2.0) - (CARR_SIDE_T / 2.0), CARR_SIDE_Z)
        ),
        material="carriage_gray",
        name="left_wall",
    )
    carriage.visual(
        Box((CARR_LEN, CARR_SIDE_T, CARR_SIDE_H)),
        origin=Origin(
            xyz=(0.0, -((CARR_OUT_W / 2.0) - (CARR_SIDE_T / 2.0)), CARR_SIDE_Z)
        ),
        material="carriage_gray",
        name="right_wall",
    )
    carriage.visual(
        Box((CARR_PAD_L, CARR_PAD_W, CARR_PAD_H)),
        origin=Origin(xyz=(0.0, 0.0, CARR_PAD_Z)),
        material="carriage_gray",
        name="top_pad",
    )
    carriage.visual(
        Box((CARR_STRAP_L, CARR_STRAP_W, CARR_STRAP_H)),
        origin=Origin(xyz=(CARR_STRAP_X, 0.0, CARR_STRAP_Z)),
        material="carriage_gray",
        name="front_strap",
    )

    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=back_support,
        child=arm,
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_support = object_model.get_part("back_support")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    rotary = object_model.get_articulation("support_to_arm")
    slider = object_model.get_articulation("arm_to_carriage")

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
        "rotary boss uses a vertical axis",
        rotary.axis == (0.0, 0.0, 1.0),
        f"axis was {rotary.axis}",
    )
    ctx.check(
        "carriage slides along the arm axis",
        slider.axis == (1.0, 0.0, 0.0),
        f"axis was {slider.axis}",
    )

    ctx.expect_contact(
        arm,
        back_support,
        elem_a="upper_disk",
        elem_b="lower_disk",
        name="arm turntable seats on the support disk",
    )
    ctx.expect_contact(
        carriage,
        arm,
        elem_a="top_bridge",
        elem_b="guide_rail",
        name="carriage roof bears on the guide rail",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="xy",
        elem_a="top_bridge",
        elem_b="guide_rail",
        min_overlap=0.03,
        name="carriage remains centered over the guide at home",
    )

    home_carriage_pos = ctx.part_world_position(carriage)
    extended_carriage_pos = None
    with ctx.pose({slider: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            arm,
            elem_a="top_bridge",
            elem_b="guide_rail",
            name="carriage stays supported at full extension",
        )
        ctx.expect_overlap(
            carriage,
            arm,
            axes="xy",
            elem_a="top_bridge",
            elem_b="guide_rail",
            min_overlap=0.03,
            name="carriage still spans the guide at full extension",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    if home_carriage_pos is not None and extended_carriage_pos is not None:
        ctx.check(
            "positive prismatic motion extends the carriage forward",
            extended_carriage_pos[0] > home_carriage_pos[0] + 0.08,
            f"home={home_carriage_pos}, extended={extended_carriage_pos}",
        )
        ctx.check(
            "carriage homes near the arm front half",
            home_carriage_pos[0] > 0.35,
            f"home carriage position was {home_carriage_pos}",
        )
    else:
        ctx.fail(
            "carriage world positions are available",
            f"home={home_carriage_pos}, extended={extended_carriage_pos}",
        )

    home_beam_aabb = ctx.part_element_world_aabb(arm, elem="beam")
    swung_beam_aabb = None
    with ctx.pose({rotary: 0.75}):
        ctx.expect_contact(
            arm,
            back_support,
            elem_a="upper_disk",
            elem_b="lower_disk",
            name="rotary disk stays seated while swung",
        )
        swung_beam_aabb = ctx.part_element_world_aabb(arm, elem="beam")

    if home_beam_aabb is not None and swung_beam_aabb is not None:
        home_beam_center_y = 0.5 * (home_beam_aabb[0][1] + home_beam_aabb[1][1])
        swung_beam_center_y = 0.5 * (swung_beam_aabb[0][1] + swung_beam_aabb[1][1])
        ctx.check(
            "positive rotary motion swings the arm toward +Y",
            swung_beam_center_y > home_beam_center_y + 0.16,
            f"home_y={home_beam_center_y}, swung_y={swung_beam_center_y}",
        )
    else:
        ctx.fail(
            "beam aabbs are available for rotary test",
            f"home={home_beam_aabb}, swung={swung_beam_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
