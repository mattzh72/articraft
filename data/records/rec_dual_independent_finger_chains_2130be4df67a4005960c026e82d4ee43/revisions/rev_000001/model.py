from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.096
BODY_D = 0.058
BODY_H = 0.058

LUG_T = 0.010
CHEEK_T = 0.004
ROOT_LUG_T = 0.006
BOSS_CAP_T = 0.003
SLOT_W = 0.010
OUTER_W = SLOT_W + 2.0 * CHEEK_T
PIN_R = 0.0032
ROOT_R = 0.0080
BOSS_R = 0.0088
ARM_W = 0.015
CHEEK_W = 0.018
BAR_T = 0.008
CLEVIS_LEN = 0.018
CHEEK_OFFSET = SLOT_W / 2.0 + CHEEK_T / 2.0

PROX_LEN = 0.052
MID_LEN = 0.040
DIST_LEN = 0.036
TIP_PAD_LEN = 0.014
TIP_PAD_W = 0.022

SHOULDER_X = 0.039
SHOULDER_Z = BODY_H / 2.0 + 0.020
SHOULDER_W = 0.020


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _ring_y(outer_radius: float, inner_radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return _cyl_y(outer_radius, length, center).cut(_cyl_y(inner_radius, length + 0.004, center))


def _make_link_shape(length: float, *, distal: bool) -> cq.Workplane:
    shape = _ring_y(BOSS_R, PIN_R, ROOT_LUG_T, (0.0, 0.0, 0.0))
    shape = shape.union(_box((ARM_W * 0.94, BAR_T, 0.010), (0.0, 0.0, 0.005)))

    if distal:
        bar_start = 0.006
        bar_end = length - TIP_PAD_LEN + 0.001
        bar_len = max(0.010, bar_end - bar_start)
        shape = shape.union(_box((ARM_W, BAR_T, bar_len), (0.0, 0.0, bar_start + bar_len / 2.0)))
        pad_center_z = length - TIP_PAD_LEN / 2.0 + 0.001
        shape = shape.union(_box((TIP_PAD_W, BAR_T + 0.004, TIP_PAD_LEN), (0.0, 0.0, pad_center_z)))
        shape = shape.union(_cyl_y(ROOT_R * 0.90, BAR_T + 0.004, (0.0, 0.0, length - 0.006)))
        return shape

    split_z = length - CLEVIS_LEN - 0.006
    beam_start = 0.006
    beam_len = max(0.012, split_z - beam_start)
    shape = shape.union(_box((ARM_W, BAR_T, beam_len), (0.0, 0.0, beam_start + beam_len / 2.0)))

    rail_start = split_z - 0.004
    rail_len = length - rail_start
    cheek_box_len = CLEVIS_LEN * 0.72
    cheek_box_center_z = length - cheek_box_len / 2.0 - 0.001
    connector_center_z = split_z + 0.002

    for side in (-1.0, 1.0):
        shape = shape.union(
            _box(
                (ARM_W * 0.78, CHEEK_T, rail_len),
                (0.0, side * CHEEK_OFFSET, rail_start + rail_len / 2.0),
            )
        )
        shape = shape.union(
            _box(
                (ARM_W * 0.74, CHEEK_OFFSET + BAR_T / 2.0, 0.008),
                (0.0, side * (CHEEK_OFFSET / 2.0), connector_center_z),
            )
        )
        shape = shape.union(_box((CHEEK_W, CHEEK_T, cheek_box_len), (0.0, side * CHEEK_OFFSET, cheek_box_center_z)))
        shape = shape.union(_cyl_y(BOSS_R, BOSS_CAP_T, (0.0, side * (SLOT_W / 2.0 + BOSS_CAP_T / 2.0), length)))
    return shape


def _make_body_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).edges("|Z").fillet(0.004)
    housing = housing.union(
        _box(
            (BODY_W * 0.76, BODY_D * 0.52, 0.018),
            (0.0, 0.0, 0.010),
        )
    )
    housing = housing.union(
        _box(
            (BODY_W * 0.66, BODY_D * 0.34, 0.012),
            (0.0, 0.0, -BODY_H / 2.0 - 0.006),
        )
    )

    clevis_start = SHOULDER_Z - CLEVIS_LEN
    post_start = BODY_H / 2.0 - 0.001
    post_len = clevis_start - post_start + 0.004
    post_center_z = post_start + post_len / 2.0
    cheek_box_len = CLEVIS_LEN * 0.78
    cheek_box_center_z = SHOULDER_Z - cheek_box_len / 2.0 - 0.001
    connector_center_z = clevis_start + 0.004

    for x_pos in (-SHOULDER_X, SHOULDER_X):
        for side in (-1.0, 1.0):
            housing = housing.union(_box((SHOULDER_W * 0.80, CHEEK_T, post_len), (x_pos, side * CHEEK_OFFSET, post_center_z)))
            housing = housing.union(
                _box(
                    (SHOULDER_W * 0.74, CHEEK_OFFSET + BAR_T / 2.0, 0.010),
                    (x_pos, side * (CHEEK_OFFSET / 2.0), connector_center_z),
                )
            )
            housing = housing.union(_box((SHOULDER_W, CHEEK_T, cheek_box_len), (x_pos, side * CHEEK_OFFSET, cheek_box_center_z)))
            housing = housing.union(_cyl_y(BOSS_R, BOSS_CAP_T, (x_pos, side * (SLOT_W / 2.0 + BOSS_CAP_T / 2.0), SHOULDER_Z)))

    return housing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_gripper")

    body_color = model.material("body_gray", rgba=(0.23, 0.25, 0.29, 1.0))
    finger_color = model.material("finger_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "body_shell"),
        origin=Origin(),
        material=body_color,
        name="body_shell",
    )

    left_proximal = model.part("left_proximal")
    left_proximal.visual(
        mesh_from_cadquery(_make_link_shape(PROX_LEN, distal=False), "left_proximal_shell"),
        origin=Origin(),
        material=finger_color,
        name="left_proximal_shell",
    )

    left_middle = model.part("left_middle")
    left_middle.visual(
        mesh_from_cadquery(_make_link_shape(MID_LEN, distal=False), "left_middle_shell"),
        origin=Origin(),
        material=finger_color,
        name="left_middle_shell",
    )

    left_distal = model.part("left_distal")
    left_distal.visual(
        mesh_from_cadquery(_make_link_shape(DIST_LEN, distal=True), "left_distal_shell"),
        origin=Origin(),
        material=finger_color,
        name="left_distal_shell",
    )

    right_proximal = model.part("right_proximal")
    right_proximal.visual(
        mesh_from_cadquery(_make_link_shape(PROX_LEN, distal=False), "right_proximal_shell"),
        origin=Origin(),
        material=finger_color,
        name="right_proximal_shell",
    )

    right_middle = model.part("right_middle")
    right_middle.visual(
        mesh_from_cadquery(_make_link_shape(MID_LEN, distal=False), "right_middle_shell"),
        origin=Origin(),
        material=finger_color,
        name="right_middle_shell",
    )

    right_distal = model.part("right_distal")
    right_distal.visual(
        mesh_from_cadquery(_make_link_shape(DIST_LEN, distal=True), "right_distal_shell"),
        origin=Origin(),
        material=finger_color,
        name="right_distal_shell",
    )

    model.articulation(
        "body_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_proximal,
        origin=Origin(xyz=(-SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, 0.0, PROX_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.10, upper=1.15),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=4.0, lower=-0.10, upper=1.00),
    )

    model.articulation(
        "body_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_proximal,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, 0.0, PROX_LEN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.10, upper=1.15),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, 0.0, MID_LEN)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=4.0, lower=-0.10, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    body_to_left = object_model.get_articulation("body_to_left_proximal")
    left_to_mid = object_model.get_articulation("left_proximal_to_left_middle")
    left_to_distal = object_model.get_articulation("left_middle_to_left_distal")
    body_to_right = object_model.get_articulation("body_to_right_proximal")
    right_to_mid = object_model.get_articulation("right_proximal_to_right_middle")
    right_to_distal = object_model.get_articulation("right_middle_to_right_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0011)
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

    for parent_part, child_part, check_name in (
        (body, left_proximal, "body_supports_left_chain"),
        (left_proximal, left_middle, "left_knuckle_contacts_middle"),
        (left_middle, left_distal, "left_middle_contacts_distal"),
        (body, right_proximal, "body_supports_right_chain"),
        (right_proximal, right_middle, "right_knuckle_contacts_middle"),
        (right_middle, right_distal, "right_middle_contacts_distal"),
    ):
        ctx.expect_contact(parent_part, child_part, contact_tol=0.0011, name=check_name)

    axis_ok = (
        body_to_left.axis == (0.0, 1.0, 0.0)
        and left_to_mid.axis == (0.0, 1.0, 0.0)
        and left_to_distal.axis == (0.0, 1.0, 0.0)
        and body_to_right.axis == (0.0, -1.0, 0.0)
        and right_to_mid.axis == (0.0, -1.0, 0.0)
        and right_to_distal.axis == (0.0, -1.0, 0.0)
    )
    ctx.check(
        "mirrored_joint_axes",
        axis_ok,
        details="Left finger should curl around +Y and right finger should curl around -Y.",
    )

    open_left = ctx.part_world_position(left_distal)
    open_right = ctx.part_world_position(right_distal)
    open_span = None if open_left is None or open_right is None else open_right[0] - open_left[0]

    service_curl = {
        body_to_left: 0.10,
        left_to_mid: 0.12,
        left_to_distal: 0.10,
        body_to_right: 0.10,
        right_to_mid: 0.12,
        right_to_distal: 0.10,
    }
    with ctx.pose(service_curl):
        curled_left = ctx.part_world_position(left_distal)
        curled_right = ctx.part_world_position(right_distal)
        curled_span = None if curled_left is None or curled_right is None else curled_right[0] - curled_left[0]
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_service_curl")
        ctx.expect_origin_distance(
            left_distal,
            right_distal,
            axes="x",
            min_dist=0.030,
            max_dist=0.120,
            name="service_curl_tip_clearance",
        )

    closing_ok = (
        open_span is not None
        and curled_span is not None
        and curled_span < open_span
        and curled_left is not None
        and curled_right is not None
        and curled_left[0] > open_left[0]
        and curled_right[0] < open_right[0]
    )
    ctx.check(
        "fingers_close_toward_center",
        closing_ok,
        details=(
            f"Open span={open_span}, curled span={curled_span}, "
            f"left x {None if open_left is None else open_left[0]}->{None if curled_left is None else curled_left[0]}, "
            f"right x {None if open_right is None else open_right[0]}->{None if curled_right is None else curled_right[0]}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
