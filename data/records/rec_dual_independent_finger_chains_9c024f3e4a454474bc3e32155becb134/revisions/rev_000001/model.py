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


PALM_BEAM_X = 0.038
PALM_BEAM_Y = 0.078
PALM_BEAM_Z = 0.030
PALM_MOUNT_X = 0.020
PALM_MOUNT_Y = 0.034
PALM_MOUNT_Z = 0.040
PALM_MOUNT_CENTER_X = -(PALM_BEAM_X / 2.0 + PALM_MOUNT_X / 2.0) + 0.002

ROOT_CENTER_Y = 0.020
ROOT_JOINT_X = 0.024

JOINT_BARREL_RADIUS = 0.0055
JOINT_BARREL_LENGTH = 0.010
FORK_EAR_THICKNESS = 0.004
FORK_EAR_HEIGHT = 0.016
FORK_EAR_LENGTH = 0.010
EAR_CENTER_OFFSET_Y = JOINT_BARREL_LENGTH / 2.0 + FORK_EAR_THICKNESS / 2.0
EAR_CENTER_BACK_X = 0.004

BODY_START_X = 0.0045
FORK_BACK_X = 0.0085
ROOT_SLOT_X = 0.018
ROOT_SLOT_Y = JOINT_BARREL_LENGTH
ROOT_SLOT_Z = 0.018
FORK_SLOT_X = 0.013
FORK_SLOT_Y = JOINT_BARREL_LENGTH
FORK_SLOT_Z = 0.018

PROXIMAL_JOINT_X = 0.050
MIDDLE_JOINT_X = 0.038
DISTAL_TIP_X = 0.031

PROXIMAL_BODY_W = 0.010
PROXIMAL_BODY_H = 0.015
MIDDLE_BODY_W = 0.009
MIDDLE_BODY_H = 0.013
DISTAL_BODY_W = 0.0085
DISTAL_BODY_H = 0.011

POSE_FLEX = {
    "palm_to_left_proximal": 0.80,
    "left_proximal_to_left_middle": 0.90,
    "left_middle_to_left_distal": 0.72,
}
POSE_FLEX_RIGHT = {
    "palm_to_right_proximal": 0.80,
    "right_proximal_to_right_middle": 0.90,
    "right_middle_to_right_distal": 0.72,
}


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _centered_y_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, -length / 2.0, 0.0))
    )


def _make_palm_shape() -> cq.Workplane:
    beam = _box((0.050, PALM_BEAM_Y, PALM_BEAM_Z), (-0.006, 0.0, 0.0))
    mount = _box((PALM_MOUNT_X, PALM_MOUNT_Y, PALM_MOUNT_Z), (PALM_MOUNT_CENTER_X, 0.0, 0.0))
    palm = beam.union(mount)

    root_web_x = 0.010
    root_web_len = 0.016
    root_web_y = 0.016
    clevis_len = ROOT_JOINT_X - 0.006
    clevis_y = 0.0045
    clevis_z = 0.018
    clevis_offset_y = 0.007

    for y_center in (ROOT_CENTER_Y, -ROOT_CENTER_Y):
        palm = palm.union(
            _box(
                (root_web_len, root_web_y, 0.022),
                (root_web_x, y_center, 0.0),
            )
        )
        for side in (-1.0, 1.0):
            palm = palm.union(
                _box(
                    (clevis_len, clevis_y, clevis_z),
                    (0.006 + clevis_len / 2.0, y_center + side * clevis_offset_y, 0.0),
                )
            )

    return palm


def _make_link_with_fork(
    *,
    next_joint_x: float,
    body_width: float,
    body_height: float,
) -> cq.Workplane:
    body = _box((next_joint_x, body_width, body_height), (next_joint_x / 2.0, 0.0, 0.0))
    root_block = _box((0.008, body_width * 1.10, body_height * 0.92), (0.004, 0.0, 0.0))
    tendon_ridge = _box(
        (next_joint_x * 0.58, body_width * 0.56, body_height * 0.28),
        (next_joint_x * 0.40, 0.0, body_height * 0.24),
    )
    knuckle_pad = _box(
        (0.008, body_width * 0.92, body_height * 0.84),
        (next_joint_x - 0.004, 0.0, 0.0),
    )
    return body.union(root_block).union(tendon_ridge).union(knuckle_pad)


def _make_distal_link(*, tip_x: float, body_width: float, body_height: float) -> cq.Workplane:
    tip_radius = body_height * 0.42
    body_length = tip_x - tip_radius * 1.2
    body = _box((body_length, body_width, body_height), (body_length / 2.0, 0.0, 0.0))
    root_block = _box((0.007, body_width * 1.08, body_height * 0.90), (0.0035, 0.0, 0.0))
    tendon_ridge = _box(
        (body_length * 0.55, body_width * 0.54, body_height * 0.25),
        (body_length * 0.42, 0.0, body_height * 0.22),
    )
    fingertip = cq.Workplane("XY").sphere(tip_radius).translate((tip_x - tip_radius, 0.0, 0.0))
    return body.union(root_block).union(tendon_ridge).union(fingertip)


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    material_name: str,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(shape, name, tolerance=0.0005, angular_tolerance=0.08),
        material=material_name,
        name="shell",
    )
    return part


def _position_delta(
    a: tuple[float, float, float] | None,
    b: tuple[float, float, float] | None,
) -> float | None:
    if a is None or b is None:
        return None
    return max(abs(a[i] - b[i]) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="forked_palm_two_finger_chain")

    model.material("palm_graphite", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("finger_alloy", rgba=(0.76, 0.78, 0.81, 1.0))

    palm = _add_mesh_part(model, name="palm", shape=_make_palm_shape(), material_name="palm_graphite")
    left_proximal = _add_mesh_part(
        model,
        name="left_proximal",
        shape=_make_link_with_fork(
            next_joint_x=PROXIMAL_JOINT_X,
            body_width=PROXIMAL_BODY_W,
            body_height=PROXIMAL_BODY_H,
        ),
        material_name="finger_alloy",
    )
    left_middle = _add_mesh_part(
        model,
        name="left_middle",
        shape=_make_link_with_fork(
            next_joint_x=MIDDLE_JOINT_X,
            body_width=MIDDLE_BODY_W,
            body_height=MIDDLE_BODY_H,
        ),
        material_name="finger_alloy",
    )
    left_distal = _add_mesh_part(
        model,
        name="left_distal",
        shape=_make_distal_link(
            tip_x=DISTAL_TIP_X,
            body_width=DISTAL_BODY_W,
            body_height=DISTAL_BODY_H,
        ),
        material_name="finger_alloy",
    )
    right_proximal = _add_mesh_part(
        model,
        name="right_proximal",
        shape=_make_link_with_fork(
            next_joint_x=PROXIMAL_JOINT_X,
            body_width=PROXIMAL_BODY_W,
            body_height=PROXIMAL_BODY_H,
        ),
        material_name="finger_alloy",
    )
    right_middle = _add_mesh_part(
        model,
        name="right_middle",
        shape=_make_link_with_fork(
            next_joint_x=MIDDLE_JOINT_X,
            body_width=MIDDLE_BODY_W,
            body_height=MIDDLE_BODY_H,
        ),
        material_name="finger_alloy",
    )
    right_distal = _add_mesh_part(
        model,
        name="right_distal",
        shape=_make_distal_link(
            tip_x=DISTAL_TIP_X,
            body_width=DISTAL_BODY_W,
            body_height=DISTAL_BODY_H,
        ),
        material_name="finger_alloy",
    )

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(ROOT_JOINT_X, ROOT_CENTER_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(PROXIMAL_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(MIDDLE_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.20),
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(ROOT_JOINT_X, -ROOT_CENTER_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(PROXIMAL_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(MIDDLE_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.20),
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

    expected_parts = {
        "palm",
        "left_proximal",
        "left_middle",
        "left_distal",
        "right_proximal",
        "right_middle",
        "right_distal",
    }
    expected_joints = {
        "palm_to_left_proximal",
        "left_proximal_to_left_middle",
        "left_middle_to_left_distal",
        "palm_to_right_proximal",
        "right_proximal_to_right_middle",
        "right_middle_to_right_distal",
    }
    actual_parts = {part.name for part in object_model.parts}
    actual_joints = {joint.name for joint in object_model.articulations}
    chain_topology = {joint.child: joint.parent for joint in object_model.articulations}

    ctx.check(
        "expected_part_set_present",
        actual_parts == expected_parts,
        details=f"actual_parts={sorted(actual_parts)}",
    )
    ctx.check(
        "expected_joint_set_present",
        actual_joints == expected_joints,
        details=f"actual_joints={sorted(actual_joints)}",
    )
    ctx.check(
        "two_uncoupled_three_joint_chains",
        chain_topology
        == {
            "left_proximal": "palm",
            "left_middle": "left_proximal",
            "left_distal": "left_middle",
            "right_proximal": "palm",
            "right_middle": "right_proximal",
            "right_distal": "right_middle",
        },
        details=f"topology={chain_topology}",
    )
    ctx.check(
        "all_revolute_axes_pitch_upward",
        all(tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in object_model.articulations),
        details=str({joint.name: tuple(joint.axis) for joint in object_model.articulations}),
    )

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid = object_model.get_articulation("left_proximal_to_left_middle")
    left_tip = object_model.get_articulation("left_middle_to_left_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid = object_model.get_articulation("right_proximal_to_right_middle")
    right_tip = object_model.get_articulation("right_middle_to_right_distal")

    ctx.expect_contact(palm, left_proximal, contact_tol=0.001, name="left_root_knuckle_supported")
    ctx.expect_contact(left_proximal, left_middle, contact_tol=0.001, name="left_middle_knuckle_supported")
    ctx.expect_contact(left_middle, left_distal, contact_tol=0.001, name="left_distal_knuckle_supported")
    ctx.expect_contact(palm, right_proximal, contact_tol=0.001, name="right_root_knuckle_supported")
    ctx.expect_contact(right_proximal, right_middle, contact_tol=0.001, name="right_middle_knuckle_supported")
    ctx.expect_contact(right_middle, right_distal, contact_tol=0.001, name="right_distal_knuckle_supported")

    ctx.expect_gap(
        left_proximal,
        right_proximal,
        axis="y",
        min_gap=0.018,
        name="root_fingers_remain_side_by_side_separated",
    )

    left_tip_rest = ctx.part_world_position(left_distal)
    right_tip_rest = ctx.part_world_position(right_distal)
    with ctx.pose({left_root: 0.80, left_mid: 0.90, left_tip: 0.72}):
        left_tip_flex = ctx.part_world_position(left_distal)
        right_tip_still = ctx.part_world_position(right_distal)
    left_move_ok = (
        left_tip_rest is not None
        and left_tip_flex is not None
        and right_tip_rest is not None
        and right_tip_still is not None
        and left_tip_flex[2] > left_tip_rest[2] + 0.020
        and (_position_delta(right_tip_rest, right_tip_still) or 0.0) < 1e-6
    )
    ctx.check(
        "left_chain_moves_without_dragging_right_chain",
        left_move_ok,
        details=(
            f"left_rest={left_tip_rest}, left_flex={left_tip_flex}, "
            f"right_rest={right_tip_rest}, right_after_left_pose={right_tip_still}"
        ),
    )

    with ctx.pose({right_root: 0.80, right_mid: 0.90, right_tip: 0.72}):
        right_tip_flex = ctx.part_world_position(right_distal)
        left_tip_still = ctx.part_world_position(left_distal)
    right_move_ok = (
        right_tip_rest is not None
        and right_tip_flex is not None
        and left_tip_rest is not None
        and left_tip_still is not None
        and right_tip_flex[2] > right_tip_rest[2] + 0.020
        and (_position_delta(left_tip_rest, left_tip_still) or 0.0) < 1e-6
    )
    ctx.check(
        "right_chain_moves_without_dragging_left_chain",
        right_move_ok,
        details=(
            f"right_rest={right_tip_rest}, right_flex={right_tip_flex}, "
            f"left_rest={left_tip_rest}, left_after_right_pose={left_tip_still}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
