from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_W = 0.24
BASE_D = 0.20
BASE_T = 0.02

PEDESTAL_W = 0.13
PEDESTAL_D = 0.11
PEDESTAL_H = 0.06

COLUMN_W = 0.10
COLUMN_D = 0.04
COLUMN_H = 0.56
COLUMN_CENTER_Z = BASE_T + PEDESTAL_H + (COLUMN_H / 2.0)

RAIL_W = 0.06
RAIL_D = 0.02
RAIL_H = 0.56
RAIL_Y = 0.03
RAIL_CENTER_Z = COLUMN_CENTER_Z

CAP_W = 0.09
CAP_D = 0.05
CAP_H = 0.03
CAP_CENTER_Y = 0.01
CAP_CENTER_Z = BASE_T + PEDESTAL_H + COLUMN_H + (CAP_H / 2.0)

CARRIAGE_W = 0.15
CARRIAGE_D = 0.11
CARRIAGE_H = 0.17
CARRIAGE_SHOE_W = 0.06
CARRIAGE_SHOE_D = 0.03
CARRIAGE_SHOE_H = 0.15
CARRIAGE_STANDOFF_W = 0.045
CARRIAGE_STANDOFF_D = 0.02
CARRIAGE_STANDOFF_H = 0.13
CARRIAGE_BRIDGE_W = 0.09
CARRIAGE_BRIDGE_D = 0.02
CARRIAGE_BRIDGE_H = 0.13
CARRIAGE_BODY_Y = 0.07
CARRIAGE_STANDOFF_Y = 0.025
CARRIAGE_BRIDGE_Y = 0.04
CARRIAGE_HOME_Z = 0.19
SLIDE_TRAVEL = 0.28

YOKE_GAP = 0.070
EAR_T = 0.022
EAR_D = 0.020
EAR_H = 0.074
EAR_CENTER_X = (YOKE_GAP / 2.0) + (EAR_T / 2.0)
YOKE_CENTER_Y = 0.105
EAR_CENTER_Y = YOKE_CENTER_Y - (EAR_D / 2.0)

FACE_W = 0.098
FACE_H = 0.098
FACE_T = 0.010
HINGE_BLOCK_W = 0.07
HINGE_BLOCK_D = 0.01
HINGE_BLOCK_H = 0.036
HINGE_BLOCK_CENTER_Y = 0.005
FACE_NECK_W = 0.036
FACE_NECK_D = 0.02
FACE_NECK_H = 0.044
FACE_NECK_CENTER_Y = 0.02
FACE_PLATE_CENTER_Y = 0.035
FACE_RIB_W = 0.018
FACE_RIB_D = 0.018
FACE_RIB_H = 0.06
FACE_RIB_CENTER_Y = 0.025
FACE_RIB_OFFSET_X = 0.026
FACE_BOSS_W = 0.038
FACE_BOSS_D = 0.012
FACE_BOSS_H = 0.04
FACE_BOSS_CENTER_Y = 0.033

SLIDE_ORIGIN_Y = RAIL_Y + (RAIL_D / 2.0) + (CARRIAGE_SHOE_D / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_wrist_slide")

    model.material("powder_coat_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.48, 0.14, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        name="mast_shell",
        material="powder_coat_dark",
    )
    mast.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + (PEDESTAL_H / 2.0))),
        material="powder_coat_dark",
        name="pedestal_shell",
    )
    mast.visual(
        Box((COLUMN_W, COLUMN_D, COLUMN_H)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_CENTER_Z)),
        material="powder_coat_dark",
        name="column_shell",
    )
    mast.visual(
        Box((RAIL_W, RAIL_D, RAIL_H)),
        origin=Origin(xyz=(0.0, RAIL_Y, RAIL_CENTER_Z)),
        material="machined_aluminum",
        name="guide_rail",
    )
    mast.visual(
        Box((CAP_W, CAP_D, CAP_H)),
        origin=Origin(xyz=(0.0, CAP_CENTER_Y, CAP_CENTER_Z)),
        material="powder_coat_dark",
        name="top_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_T + PEDESTAL_H + COLUMN_H + CAP_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + PEDESTAL_H + COLUMN_H + CAP_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_SHOE_W, CARRIAGE_SHOE_D, CARRIAGE_SHOE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="carriage_shell",
        material="carriage_orange",
    )
    carriage.visual(
        Box((CARRIAGE_STANDOFF_W, CARRIAGE_STANDOFF_D, CARRIAGE_STANDOFF_H)),
        origin=Origin(xyz=(0.0, CARRIAGE_STANDOFF_Y, 0.0)),
        material="carriage_orange",
        name="carriage_standoff",
    )
    carriage.visual(
        Box((CARRIAGE_BRIDGE_W, CARRIAGE_BRIDGE_D, CARRIAGE_BRIDGE_H)),
        origin=Origin(xyz=(0.0, CARRIAGE_BRIDGE_Y, 0.0)),
        material="carriage_orange",
        name="carriage_bridge",
    )
    carriage.visual(
        Box((CARRIAGE_W, 0.06, CARRIAGE_H)),
        origin=Origin(xyz=(0.0, CARRIAGE_BODY_Y, 0.0)),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.visual(
        Box((EAR_T, EAR_D, EAR_H)),
        origin=Origin(xyz=(-EAR_CENTER_X, EAR_CENTER_Y, 0.0)),
        material="carriage_orange",
        name="left_ear",
    )
    carriage.visual(
        Box((EAR_T, EAR_D, EAR_H)),
        origin=Origin(xyz=(EAR_CENTER_X, EAR_CENTER_Y, 0.0)),
        material="carriage_orange",
        name="right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)),
        mass=4.5,
        origin=Origin(xyz=(0.0, CARRIAGE_BODY_Y, 0.0)),
    )

    output_face = model.part("output_face")
    output_face.visual(
        Box((HINGE_BLOCK_W, HINGE_BLOCK_D, HINGE_BLOCK_H)),
        origin=Origin(xyz=(0.0, HINGE_BLOCK_CENTER_Y, 0.0)),
        material="machined_aluminum",
        name="hinge_block",
    )
    output_face.visual(
        Box((FACE_NECK_W, FACE_NECK_D, FACE_NECK_H)),
        origin=Origin(xyz=(0.0, FACE_NECK_CENTER_Y, 0.0)),
        material="machined_aluminum",
        name="face_neck",
    )
    output_face.visual(
        Box((FACE_W, FACE_T, FACE_H)),
        origin=Origin(xyz=(0.0, FACE_PLATE_CENTER_Y, 0.0)),
        material="machined_aluminum",
        name="output_face_shell",
    )
    output_face.visual(
        Box((FACE_RIB_W, FACE_RIB_D, FACE_RIB_H)),
        origin=Origin(xyz=(-FACE_RIB_OFFSET_X, FACE_RIB_CENTER_Y, 0.0)),
        material="machined_aluminum",
        name="left_rib",
    )
    output_face.visual(
        Box((FACE_RIB_W, FACE_RIB_D, FACE_RIB_H)),
        origin=Origin(xyz=(FACE_RIB_OFFSET_X, FACE_RIB_CENTER_Y, 0.0)),
        material="machined_aluminum",
        name="right_rib",
    )
    output_face.visual(
        Box((FACE_BOSS_W, FACE_BOSS_D, FACE_BOSS_H)),
        origin=Origin(xyz=(0.0, FACE_BOSS_CENTER_Y, 0.0)),
        material="machined_aluminum",
        name="center_boss",
    )
    output_face.inertial = Inertial.from_geometry(
        Box((FACE_W, 0.04, FACE_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, FACE_PLATE_CENTER_Y, 0.0)),
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, SLIDE_ORIGIN_Y, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_face,
        origin=Origin(xyz=(0.0, YOKE_CENTER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-0.65,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    output_face = object_model.get_part("output_face")
    slide = object_model.get_articulation("mast_slide")
    wrist = object_model.get_articulation("carriage_wrist")

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
        "slide_axis_is_vertical",
        slide.axis == (0.0, 0.0, 1.0),
        f"expected vertical slide axis, got {slide.axis}",
    )
    ctx.check(
        "wrist_axis_is_horizontal",
        wrist.axis == (1.0, 0.0, 0.0),
        f"expected wrist pitch axis along +X, got {wrist.axis}",
    )
    ctx.expect_origin_gap(
        output_face,
        carriage,
        axis="y",
        min_gap=0.09,
        max_gap=0.12,
        name="output_face_is_carried_on_carriage_front",
    )

    with ctx.pose({slide: slide.motion_limits.lower}):
        lower_z = ctx.part_world_position(carriage)[2]
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=0.002,
            name="carriage_contacts_mast_at_lower_slide_pose",
        )

    with ctx.pose({slide: slide.motion_limits.upper}):
        upper_z = ctx.part_world_position(carriage)[2]
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=0.002,
            name="carriage_contacts_mast_at_upper_slide_pose",
        )

    ctx.check(
        "positive_slide_moves_carriage_upward",
        upper_z > lower_z + 0.20,
        f"expected upward slide motion larger than 0.20 m, got lower_z={lower_z:.4f}, upper_z={upper_z:.4f}",
    )

    with ctx.pose({wrist: 0.0}):
        rest_face_aabb = ctx.part_element_world_aabb(output_face, elem="output_face_shell")
        ctx.expect_contact(
            output_face,
            carriage,
            contact_tol=0.002,
            name="wrist_is_supported_at_rest",
        )

    with ctx.pose({wrist: wrist.motion_limits.upper}):
        pitched_face_aabb = ctx.part_element_world_aabb(output_face, elem="output_face_shell")
        ctx.expect_contact(
            output_face,
            carriage,
            contact_tol=0.002,
            name="wrist_remains_supported_when_pitched",
        )

    rest_face_center_z = (rest_face_aabb[0][2] + rest_face_aabb[1][2]) / 2.0
    pitched_face_center_z = (pitched_face_aabb[0][2] + pitched_face_aabb[1][2]) / 2.0
    ctx.check(
        "positive_wrist_rotation_lifts_output_face",
        pitched_face_center_z > rest_face_center_z + 0.015,
        (
            "expected positive wrist rotation to raise the output face center, "
            f"got rest_z={rest_face_center_z:.4f}, pitched_z={pitched_face_center_z:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
