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


BASE_X = 0.125
BASE_Y = 0.052
BASE_Z = 0.036
BASE_CAP_Z = 0.012
ROOT_X_OFFSET = 0.046
ROOT_AXIS_Z = 0.060

PLATE_T = 0.004
CLEVIS_GAP = 0.006
FINGER_THICKNESS = CLEVIS_GAP + 2.0 * PLATE_T
HINGE_BLOCK_Z = 0.014
BODY_Y = 0.008

PROX_LEN = 0.078
MID_LEN = 0.064
DIST_LEN = 0.054

PROX_WIDTH = 0.022
MID_WIDTH = 0.019
DIST_WIDTH = 0.016


def _add_base_visuals(base) -> None:
    plate_center_y = CLEVIS_GAP / 2.0 + PLATE_T / 2.0
    cheek_x = PROX_WIDTH + 0.006
    cheek_z = 0.028

    base.visual(
        Box((BASE_X, BASE_Y, BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z / 2.0)),
        material="base_dark",
        name="base_body",
    )
    base.visual(
        Box((BASE_X * 0.72, BASE_Y * 0.84, BASE_CAP_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z + BASE_CAP_Z / 2.0 - 0.002)),
        material="base_dark",
        name="base_cap",
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        x0 = sign * ROOT_X_OFFSET
        base.visual(
            Box((cheek_x, PLATE_T, cheek_z)),
            origin=Origin(xyz=(x0, -plate_center_y, ROOT_AXIS_Z)),
            material="base_dark",
            name=f"{side_name}_inner_plate",
        )
        base.visual(
            Box((cheek_x, PLATE_T, cheek_z)),
            origin=Origin(xyz=(x0, plate_center_y, ROOT_AXIS_Z)),
            material="base_dark",
            name=f"{side_name}_outer_plate",
        )


def _add_intermediate_link_visuals(part, *, prefix: str, length: float, root_width: float, distal_width: float) -> None:
    plate_center_y = CLEVIS_GAP / 2.0 + PLATE_T / 2.0
    spine_top = length - 0.033
    spine_bottom = HINGE_BLOCK_Z
    spine_height = spine_top - spine_bottom
    bridge_center_z = length - 0.028
    plate_center_z = length - 0.010
    rib_center_z = length - 0.018

    part.visual(
        Box((root_width * 0.76, CLEVIS_GAP, HINGE_BLOCK_Z)),
        origin=Origin(xyz=(0.0, 0.0, HINGE_BLOCK_Z / 2.0)),
        material="finger_metal",
        name=f"{prefix}_root_lug",
    )
    part.visual(
        Box((root_width * 0.42, BODY_Y, spine_height)),
        origin=Origin(xyz=(0.0, 0.0, (spine_bottom + spine_top) / 2.0)),
        material="finger_metal",
        name=f"{prefix}_spine",
    )
    part.visual(
        Box((distal_width * 0.60, FINGER_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, bridge_center_z)),
        material="finger_metal",
        name=f"{prefix}_bridge",
    )
    part.visual(
        Box((distal_width * 0.56, PLATE_T, 0.030)),
        origin=Origin(xyz=(0.0, -plate_center_y, rib_center_z)),
        material="finger_metal",
        name=f"{prefix}_left_rib",
    )
    part.visual(
        Box((distal_width * 0.56, PLATE_T, 0.030)),
        origin=Origin(xyz=(0.0, plate_center_y, rib_center_z)),
        material="finger_metal",
        name=f"{prefix}_right_rib",
    )
    part.visual(
        Box((distal_width, PLATE_T, 0.020)),
        origin=Origin(xyz=(0.0, -plate_center_y, plate_center_z)),
        material="finger_metal",
        name=f"{prefix}_left_plate",
    )
    part.visual(
        Box((distal_width, PLATE_T, 0.020)),
        origin=Origin(xyz=(0.0, plate_center_y, plate_center_z)),
        material="finger_metal",
        name=f"{prefix}_right_plate",
    )


def _add_distal_link_visuals(part, *, prefix: str, length: float, root_width: float, tip_width: float) -> None:
    part.visual(
        Box((root_width * 0.76, CLEVIS_GAP, HINGE_BLOCK_Z)),
        origin=Origin(xyz=(0.0, 0.0, HINGE_BLOCK_Z / 2.0)),
        material="finger_metal",
        name=f"{prefix}_root_lug",
    )
    spine_height = length - 0.020 - HINGE_BLOCK_Z
    part.visual(
        Box((root_width * 0.40, BODY_Y, spine_height)),
        origin=Origin(xyz=(0.0, 0.0, HINGE_BLOCK_Z + spine_height / 2.0)),
        material="finger_metal",
        name=f"{prefix}_spine",
    )
    part.visual(
        Box((tip_width * 0.92, BODY_Y * 1.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, length - 0.015)),
        material="finger_metal",
        name=f"{prefix}_tip_pad",
    )
    part.visual(
        Box((tip_width * 0.50, BODY_Y * 0.85, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, length - 0.006)),
        material="finger_metal",
        name=f"{prefix}_tip_nose",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tool_finger_pair")

    model.material("base_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("finger_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    _add_base_visuals(base)

    left_prox = model.part("left_proximal")
    _add_intermediate_link_visuals(
        left_prox,
        prefix="left_proximal",
        length=PROX_LEN,
        root_width=PROX_WIDTH,
        distal_width=MID_WIDTH,
    )

    left_mid = model.part("left_middle")
    _add_intermediate_link_visuals(
        left_mid,
        prefix="left_middle",
        length=MID_LEN,
        root_width=MID_WIDTH,
        distal_width=DIST_WIDTH,
    )

    left_dist = model.part("left_distal")
    _add_distal_link_visuals(
        left_dist,
        prefix="left_distal",
        length=DIST_LEN,
        root_width=DIST_WIDTH,
        tip_width=DIST_WIDTH * 0.90,
    )

    right_prox = model.part("right_proximal")
    _add_intermediate_link_visuals(
        right_prox,
        prefix="right_proximal",
        length=PROX_LEN,
        root_width=PROX_WIDTH,
        distal_width=MID_WIDTH,
    )

    right_mid = model.part("right_middle")
    _add_intermediate_link_visuals(
        right_mid,
        prefix="right_middle",
        length=MID_LEN,
        root_width=MID_WIDTH,
        distal_width=DIST_WIDTH,
    )

    right_dist = model.part("right_distal")
    _add_distal_link_visuals(
        right_dist,
        prefix="right_distal",
        length=DIST_LEN,
        root_width=DIST_WIDTH,
        tip_width=DIST_WIDTH * 0.90,
    )

    model.articulation(
        "base_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_prox,
        origin=Origin(xyz=(-ROOT_X_OFFSET, 0.0, ROOT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.5,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_prox,
        child=left_mid,
        origin=Origin(xyz=(0.0, 0.0, PROX_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_mid,
        child=left_dist,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    model.articulation(
        "base_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_prox,
        origin=Origin(xyz=(ROOT_X_OFFSET, 0.0, ROOT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=7.0,
            velocity=2.5,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_prox,
        child=right_mid,
        origin=Origin(xyz=(0.0, 0.0, PROX_LEN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_mid,
        child=right_dist,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    left_prox = object_model.get_part("left_proximal")
    left_mid = object_model.get_part("left_middle")
    left_dist = object_model.get_part("left_distal")
    right_prox = object_model.get_part("right_proximal")
    right_mid = object_model.get_part("right_middle")
    right_dist = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("base_to_left_proximal")
    left_mid_joint = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip_joint = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("base_to_right_proximal")
    right_mid_joint = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip_joint = object_model.get_articulation("right_middle_to_right_distal")

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

    ctx.expect_contact(base, left_prox, contact_tol=5e-4, name="left root finger is mounted to base")
    ctx.expect_contact(
        left_prox,
        left_mid,
        contact_tol=5e-4,
        name="left proximal and middle links stay in hinge contact",
    )
    ctx.expect_contact(
        left_mid,
        left_dist,
        contact_tol=5e-4,
        name="left middle and distal links stay in hinge contact",
    )
    ctx.expect_contact(base, right_prox, contact_tol=5e-4, name="right root finger is mounted to base")
    ctx.expect_contact(
        right_prox,
        right_mid,
        contact_tol=5e-4,
        name="right proximal and middle links stay in hinge contact",
    )
    ctx.expect_contact(
        right_mid,
        right_dist,
        contact_tol=5e-4,
        name="right middle and distal links stay in hinge contact",
    )
    ctx.expect_gap(
        right_dist,
        left_dist,
        axis="x",
        min_gap=0.03,
        name="open fingertips have a clear service gap",
    )

    rest_left_tip_x = ctx.part_world_position(left_dist)[0]
    rest_right_tip_x = ctx.part_world_position(right_dist)[0]
    service_pose = {
        left_root: 0.24,
        left_mid_joint: 0.46,
        left_tip_joint: 0.38,
        right_root: 0.24,
        right_mid_joint: 0.46,
        right_tip_joint: 0.38,
    }
    with ctx.pose(service_pose):
        curled_left_tip_x = ctx.part_world_position(left_dist)[0]
        curled_right_tip_x = ctx.part_world_position(right_dist)[0]
        ctx.check(
            "distal links curl inward from the open pose",
            curled_left_tip_x > rest_left_tip_x + 0.012
            and curled_right_tip_x < rest_right_tip_x - 0.012,
            details=(
                f"rest x=({rest_left_tip_x:.4f}, {rest_right_tip_x:.4f}), "
                f"curled x=({curled_left_tip_x:.4f}, {curled_right_tip_x:.4f})"
            ),
        )
        ctx.expect_contact(
            base,
            left_prox,
            contact_tol=5e-4,
            name="left root hinge stays physically seated while curled",
        )
        ctx.expect_contact(
            left_prox,
            left_mid,
            contact_tol=5e-4,
            name="left middle hinge stays seated while curled",
        )
        ctx.expect_contact(
            left_mid,
            left_dist,
            contact_tol=5e-4,
            name="left distal hinge stays seated while curled",
        )
        ctx.check(
            "service pose brings both fingertips inward toward the centerline",
            abs(curled_left_tip_x) < abs(rest_left_tip_x) - 0.010
            and abs(curled_right_tip_x) < abs(rest_right_tip_x) - 0.010,
            details=(
                f"rest_abs=({abs(rest_left_tip_x):.4f}, {abs(rest_right_tip_x):.4f}), "
                f"service_abs=({abs(curled_left_tip_x):.4f}, {abs(curled_right_tip_x):.4f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
