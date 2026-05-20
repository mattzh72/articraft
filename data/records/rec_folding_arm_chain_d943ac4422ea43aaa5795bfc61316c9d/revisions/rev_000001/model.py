from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.086
FOOT_DEPTH = 0.072
FOOT_HEIGHT = 0.012

SPINE_WIDTH = 0.024
SPINE_DEPTH = 0.018
SPINE_HEIGHT = 0.225
SHOULDER_X = 0.004
SHOULDER_Z = FOOT_HEIGHT + SPINE_HEIGHT

JOINT_CLEARANCE = 0.002
ROOT_EYE_FRONT = 0.018
ROOT_EYE_LENGTH = ROOT_EYE_FRONT
ROOT_EYE_WIDTH = 0.010
ROOT_EYE_HEIGHT = 0.024

LINK_WIDTH = 0.012
LINK_HEIGHT = 0.010
LINK_BEAM_Z = -0.014

CHEEK_LENGTH = 0.028
CHEEK_OUTER_WIDTH = 0.024
CHEEK_HEIGHT = 0.028
CHEEK_SIDE_WIDTH = 0.006
CHEEK_GAP = CHEEK_OUTER_WIDTH - 2.0 * CHEEK_SIDE_WIDTH
CHEEK_Y_OFFSET = CHEEK_GAP / 2.0 + CHEEK_SIDE_WIDTH / 2.0
BRIDGE_HEIGHT = 0.010
BRIDGE_Z = -0.016

LINK_1_LENGTH = 0.165
LINK_2_LENGTH = 0.150
LINK_3_LENGTH = 0.125

PAD_LENGTH = 0.030
PAD_WIDTH = 0.024
PAD_HEIGHT = 0.010


def _joint_lug() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(ROOT_EYE_LENGTH, ROOT_EYE_WIDTH, ROOT_EYE_HEIGHT)
        .translate((JOINT_CLEARANCE / 2.0 + ROOT_EYE_LENGTH / 2.0, 0.0, 0.0))
    )


def _side_cheeks(x_center: float, *, z_center: float = 0.0) -> cq.Workplane:
    cheek = cq.Workplane("XY").box(CHEEK_LENGTH, CHEEK_SIDE_WIDTH, CHEEK_HEIGHT)
    left = cheek.translate((x_center, CHEEK_Y_OFFSET, z_center))
    right = cheek.translate((x_center, -CHEEK_Y_OFFSET, z_center))
    return left.union(right)


def _under_bridge(length: float, x_center: float, *, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, CHEEK_OUTER_WIDTH, BRIDGE_HEIGHT)
        .translate((x_center, 0.0, z_center))
    )


def _make_link_shape(length: float, *, distal_fork: bool) -> cq.Workplane:
    beam_start = ROOT_EYE_FRONT - 0.002
    beam_end = length - CHEEK_LENGTH - JOINT_CLEARANCE / 2.0 + 0.004 if distal_fork else length - 0.010
    beam_length = max(beam_end - beam_start, 0.050)
    beam = (
        cq.Workplane("XY")
        .box(beam_length, LINK_WIDTH, LINK_HEIGHT)
        .translate((beam_start + beam_length / 2.0, 0.0, LINK_BEAM_Z))
    )
    root_web = (
        cq.Workplane("XY")
        .box(0.022, 0.016, 0.012)
        .translate((0.015, 0.0, -0.010))
    )

    body = _joint_lug().union(beam).union(root_web)

    if distal_fork:
        x_center = length - JOINT_CLEARANCE / 2.0 - CHEEK_LENGTH / 2.0
        fork = _side_cheeks(x_center)
        bridge = _under_bridge(CHEEK_LENGTH, x_center, z_center=BRIDGE_Z)
        body = body.union(fork).union(bridge)
    else:
        distal_bridge = _under_bridge(0.022, length - 0.016, z_center=BRIDGE_Z)
        tip_block = (
            cq.Workplane("XY")
            .box(0.020, 0.016, 0.020)
            .translate((length - 0.010, 0.0, -0.005))
        )
        body = body.union(distal_bridge).union(tip_block)

    return body


def _make_spine_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(FOOT_LENGTH, FOOT_DEPTH, FOOT_HEIGHT).translate((0.0, 0.0, FOOT_HEIGHT / 2.0))
    column_height = SPINE_HEIGHT - 0.055
    upright = (
        cq.Workplane("XY")
        .box(SPINE_WIDTH, SPINE_DEPTH, column_height)
        .translate((0.0, 0.0, FOOT_HEIGHT + column_height / 2.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.016, 0.014, 0.028)
        .translate((-0.010, 0.0, 0.193))
    )
    head = (
        cq.Workplane("XY")
        .box(0.022, 0.014, 0.034)
        .translate((-0.011, 0.0, SHOULDER_Z - 0.016))
    )
    shoulder_x_center = SHOULDER_X - CHEEK_LENGTH / 2.0
    shoulder_cheeks = _side_cheeks(
        SHOULDER_X - JOINT_CLEARANCE / 2.0 - CHEEK_LENGTH / 2.0,
        z_center=SHOULDER_Z,
    )
    shoulder_bridge = _under_bridge(
        CHEEK_LENGTH,
        SHOULDER_X - JOINT_CLEARANCE / 2.0 - CHEEK_LENGTH / 2.0,
        z_center=SHOULDER_Z + BRIDGE_Z,
    )

    return foot.union(upright).union(neck).union(head).union(shoulder_bridge).union(shoulder_cheeks)


def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def _position_z(position: tuple[float, float, float] | None) -> float | None:
    if position is None:
        return None
    return position[2]


def _is_parallel_axis(axis: tuple[float, float, float], ref: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(axis, ref))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_folding_arm_chain")

    model.material("graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("rubber", rgba=(0.10, 0.11, 0.12, 1.0))

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_make_spine_shape(), "spine"),
        material="graphite",
        name="spine_body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_make_link_shape(LINK_1_LENGTH, distal_fork=True), "link_1"),
        material="alloy",
        name="link_1_body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_make_link_shape(LINK_2_LENGTH, distal_fork=True), "link_2"),
        material="alloy",
        name="link_2_body",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_make_link_shape(LINK_3_LENGTH, distal_fork=False), "link_3"),
        material="alloy",
        name="link_3_body",
    )

    end_pad = model.part("end_pad")
    end_pad.visual(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT)),
        origin=Origin(xyz=(PAD_LENGTH / 2.0, 0.0, 0.0)),
        material="rubber",
        name="pad_block",
    )

    model.articulation(
        "spine_to_link_1",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=link_1,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.45, effort=18.0, velocity=1.4),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.8, upper=1.8, effort=14.0, velocity=1.7),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.9, upper=1.9, effort=10.0, velocity=1.9),
    )
    model.articulation(
        "link_3_to_end_pad",
        ArticulationType.FIXED,
        parent=link_3,
        child=end_pad,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_pad = object_model.get_part("end_pad")

    shoulder = object_model.get_articulation("spine_to_link_1")
    elbow = object_model.get_articulation("link_1_to_link_2")
    wrist = object_model.get_articulation("link_2_to_link_3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.003)
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
        "three serial joints use parallel axes",
        _is_parallel_axis(shoulder.axis, (0.0, -1.0, 0.0))
        and _is_parallel_axis(elbow.axis, shoulder.axis)
        and _is_parallel_axis(wrist.axis, shoulder.axis),
        details=f"axes were {shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )

    ctx.expect_contact(link_1, spine, contact_tol=0.003, name="link_1 seated in spine joint cheeks")
    ctx.expect_overlap(link_1, spine, axes="yz", min_overlap=0.02, name="link_1 hinge stays aligned with spine cheeks")
    ctx.expect_contact(link_2, link_1, contact_tol=0.003, name="link_2 seated in link_1 joint cheeks")
    ctx.expect_contact(link_3, link_2, contact_tol=0.003, name="link_3 seated in link_2 joint cheeks")
    ctx.expect_contact(end_pad, link_3, contact_tol=0.001, name="end pad mounted to distal link")

    ctx.expect_origin_gap(link_2, spine, axis="x", min_gap=0.15, max_gap=0.18, name="rest reach from spine to second joint")
    ctx.expect_origin_gap(link_3, link_2, axis="x", min_gap=0.14, max_gap=0.16, name="rest reach from second to third joint")
    ctx.expect_origin_gap(end_pad, link_3, axis="x", min_gap=0.12, max_gap=0.13, name="rest reach from third joint to end pad")

    rest_link_2_z = _position_z(ctx.part_world_position(link_2))
    with ctx.pose({shoulder: 0.8}):
        raised_link_2_z = _position_z(ctx.part_world_position(link_2))
    ctx.check(
        "shoulder raises the next joint upward",
        rest_link_2_z is not None and raised_link_2_z is not None and raised_link_2_z > rest_link_2_z + 0.10,
        details=f"rest_z={rest_link_2_z}, raised_z={raised_link_2_z}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.0}):
        elbow_rest_link_3_z = _position_z(ctx.part_world_position(link_3))
    with ctx.pose({shoulder: 0.35, elbow: 0.75}):
        elbow_raised_link_3_z = _position_z(ctx.part_world_position(link_3))
    ctx.check(
        "elbow raises the distal joint upward",
        elbow_rest_link_3_z is not None
        and elbow_raised_link_3_z is not None
        and elbow_raised_link_3_z > elbow_rest_link_3_z + 0.07,
        details=f"rest_z={elbow_rest_link_3_z}, raised_z={elbow_raised_link_3_z}",
    )

    with ctx.pose({shoulder: 0.25, elbow: 0.1, wrist: 0.0}):
        pad_neutral_aabb = ctx.part_world_aabb(end_pad)
    with ctx.pose({shoulder: 0.25, elbow: 0.1, wrist: 0.9}):
        pad_raised_aabb = ctx.part_world_aabb(end_pad)

    neutral_z = _center_z(pad_neutral_aabb)
    raised_z = _center_z(pad_raised_aabb)
    ctx.check(
        "wrist moves the end pad upward",
        neutral_z is not None and raised_z is not None and raised_z > neutral_z + 0.03,
        details=f"neutral_z={neutral_z}, raised_z={raised_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
