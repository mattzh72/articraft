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


PALM_WIDTH = 0.044
PALM_FRONT_X = 0.022
UPPER_SUPPORT_Z = 0.018
LOWER_SUPPORT_Z = -0.018

UPPER_SUPPORT_LEN = 0.016
LOWER_SUPPORT_LEN = 0.013

LONG_PROX_LEN = 0.032
LONG_MID_LEN = 0.028
LONG_DIST_LEN = 0.024

SHORT_PROX_LEN = 0.026
SHORT_MID_LEN = 0.020


def _extrude_xz_profile(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(width)
        .translate((0.0, width / 2.0, 0.0))
    )


def _make_palm() -> cq.Workplane:
    profile = [
        (-0.027, -0.026),
        (0.016, -0.026),
        (0.022, -0.022),
        (0.022, -0.010),
        (0.012, 0.000),
        (0.022, 0.010),
        (0.022, 0.022),
        (0.016, 0.026),
        (-0.027, 0.026),
    ]
    return _extrude_xz_profile(profile, PALM_WIDTH)


def _make_support_block(length: float, height: float, nose_height: float, width: float) -> cq.Workplane:
    profile = [
        (0.000, -height / 2.0),
        (length * 0.62, -height / 2.0),
        (length, -nose_height / 2.0),
        (length, nose_height / 2.0),
        (length * 0.62, height / 2.0),
        (0.000, height / 2.0),
    ]
    return _extrude_xz_profile(profile, width)


def _make_link_with_distal_pod(
    length: float,
    width: float,
    pod_height: float,
    beam_height: float,
    pod_len: float,
    distal_pod_height: float | None = None,
) -> cq.Workplane:
    distal_pod_height = pod_height if distal_pod_height is None else distal_pod_height
    shoulder = min(0.0022, max(0.0014, length * 0.08))
    x1 = pod_len + shoulder
    x2 = length - pod_len - shoulder
    profile = [
        (0.000, -pod_height / 2.0),
        (pod_len, -pod_height / 2.0),
        (x1, -beam_height / 2.0),
        (x2, -beam_height / 2.0),
        (length - pod_len, -distal_pod_height / 2.0),
        (length, -distal_pod_height / 2.0),
        (length, distal_pod_height / 2.0),
        (length - pod_len, distal_pod_height / 2.0),
        (x2, beam_height / 2.0),
        (x1, beam_height / 2.0),
        (pod_len, pod_height / 2.0),
        (0.000, pod_height / 2.0),
    ]
    return _extrude_xz_profile(profile, width)


def _make_long_distal() -> cq.Workplane:
    profile = [
        (0.000, -0.0065),
        (0.0070, -0.0065),
        (0.0100, -0.0040),
        (0.0180, -0.0040),
        (0.0240, -0.0028),
        (0.0240, 0.0028),
        (0.0180, 0.0040),
        (0.0100, 0.0040),
        (0.0070, 0.0065),
        (0.000, 0.0065),
    ]
    return _extrude_xz_profile(profile, 0.0105)


def _make_hooked_distal() -> cq.Workplane:
    profile = [
        (0.000, -0.0060),
        (0.0070, -0.0060),
        (0.0100, -0.0038),
        (0.0140, -0.0038),
        (0.0180, 0.0015),
        (0.0215, 0.0085),
        (0.0205, 0.0120),
        (0.0155, 0.0110),
        (0.0100, 0.0075),
        (0.0070, 0.0060),
        (0.000, 0.0060),
    ]
    return _extrude_xz_profile(profile, 0.0100)


def _add_mesh(part, shape: cq.Workplane, mesh_name: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=f"{mesh_name}_visual",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_robot_pinch_hand")

    model.material("palm_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("support_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("finger_alloy", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("finger_dark_alloy", rgba=(0.58, 0.60, 0.64, 1.0))

    palm = model.part("palm")
    upper_support = model.part("upper_support")
    lower_support = model.part("lower_support")
    long_proximal = model.part("long_proximal")
    long_middle = model.part("long_middle")
    long_distal = model.part("long_distal")
    short_proximal = model.part("short_proximal")
    short_middle = model.part("short_middle")
    short_distal = model.part("short_distal")

    _add_mesh(palm, _make_palm(), "palm_shell", "palm_dark")
    _add_mesh(
        upper_support,
        _make_support_block(length=UPPER_SUPPORT_LEN, height=0.014, nose_height=0.010, width=0.018),
        "upper_support_shell",
        "support_black",
    )
    _add_mesh(
        lower_support,
        _make_support_block(length=LOWER_SUPPORT_LEN, height=0.014, nose_height=0.009, width=0.017),
        "lower_support_shell",
        "support_black",
    )
    _add_mesh(
        long_proximal,
        _make_link_with_distal_pod(
            length=LONG_PROX_LEN,
            width=0.011,
            pod_height=0.014,
            beam_height=0.008,
            pod_len=0.008,
            distal_pod_height=0.013,
        ),
        "long_proximal_shell",
        "finger_alloy",
    )
    _add_mesh(
        long_middle,
        _make_link_with_distal_pod(
            length=LONG_MID_LEN,
            width=0.0105,
            pod_height=0.013,
            beam_height=0.0075,
            pod_len=0.0075,
            distal_pod_height=0.0125,
        ),
        "long_middle_shell",
        "finger_alloy",
    )
    _add_mesh(long_distal, _make_long_distal(), "long_distal_shell", "finger_alloy")
    _add_mesh(
        short_proximal,
        _make_link_with_distal_pod(
            length=SHORT_PROX_LEN,
            width=0.0105,
            pod_height=0.013,
            beam_height=0.0074,
            pod_len=0.007,
            distal_pod_height=0.012,
        ),
        "short_proximal_shell",
        "finger_dark_alloy",
    )
    _add_mesh(
        short_middle,
        _make_link_with_distal_pod(
            length=SHORT_MID_LEN,
            width=0.0100,
            pod_height=0.012,
            beam_height=0.0070,
            pod_len=0.0065,
            distal_pod_height=0.011,
        ),
        "short_middle_shell",
        "finger_dark_alloy",
    )
    _add_mesh(short_distal, _make_hooked_distal(), "short_distal_shell", "finger_dark_alloy")

    model.articulation(
        "palm_to_upper_support",
        ArticulationType.FIXED,
        parent=palm,
        child=upper_support,
        origin=Origin(xyz=(PALM_FRONT_X, 0.0, UPPER_SUPPORT_Z)),
    )
    model.articulation(
        "palm_to_lower_support",
        ArticulationType.FIXED,
        parent=palm,
        child=lower_support,
        origin=Origin(xyz=(PALM_FRONT_X, 0.0, LOWER_SUPPORT_Z)),
    )

    model.articulation(
        "upper_support_to_long_proximal",
        ArticulationType.REVOLUTE,
        parent=upper_support,
        child=long_proximal,
        origin=Origin(xyz=(UPPER_SUPPORT_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.8, lower=0.0, upper=0.90),
    )
    model.articulation(
        "long_proximal_to_long_middle",
        ArticulationType.REVOLUTE,
        parent=long_proximal,
        child=long_middle,
        origin=Origin(xyz=(LONG_PROX_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "long_middle_to_long_distal",
        ArticulationType.REVOLUTE,
        parent=long_middle,
        child=long_distal,
        origin=Origin(xyz=(LONG_MID_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5, lower=0.0, upper=0.95),
    )

    model.articulation(
        "lower_support_to_short_proximal",
        ArticulationType.REVOLUTE,
        parent=lower_support,
        child=short_proximal,
        origin=Origin(xyz=(LOWER_SUPPORT_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.8, lower=0.0, upper=0.95),
    )
    model.articulation(
        "short_proximal_to_short_middle",
        ArticulationType.REVOLUTE,
        parent=short_proximal,
        child=short_middle,
        origin=Origin(xyz=(SHORT_PROX_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.2, lower=0.0, upper=1.10),
    )
    model.articulation(
        "short_middle_to_short_distal",
        ArticulationType.REVOLUTE,
        parent=short_middle,
        child=short_distal,
        origin=Origin(xyz=(SHORT_MID_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5, lower=0.0, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    upper_support = object_model.get_part("upper_support")
    lower_support = object_model.get_part("lower_support")
    long_proximal = object_model.get_part("long_proximal")
    long_middle = object_model.get_part("long_middle")
    long_distal = object_model.get_part("long_distal")
    short_proximal = object_model.get_part("short_proximal")
    short_middle = object_model.get_part("short_middle")
    short_distal = object_model.get_part("short_distal")

    upper_mount = object_model.get_articulation("palm_to_upper_support")
    lower_mount = object_model.get_articulation("palm_to_lower_support")
    upper_base = object_model.get_articulation("upper_support_to_long_proximal")
    upper_mid = object_model.get_articulation("long_proximal_to_long_middle")
    upper_tip = object_model.get_articulation("long_middle_to_long_distal")
    lower_base = object_model.get_articulation("lower_support_to_short_proximal")
    lower_mid = object_model.get_articulation("short_proximal_to_short_middle")
    lower_tip = object_model.get_articulation("short_middle_to_short_distal")

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

    for part_name in (
        "palm",
        "upper_support",
        "lower_support",
        "long_proximal",
        "long_middle",
        "long_distal",
        "short_proximal",
        "short_middle",
        "short_distal",
    ):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None)

    ctx.expect_contact(upper_support, palm, name="upper_support_attached_to_palm")
    ctx.expect_contact(lower_support, palm, name="lower_support_attached_to_palm")
    ctx.expect_contact(long_proximal, upper_support, name="long_base_joint_seated")
    ctx.expect_contact(long_middle, long_proximal, name="long_middle_joint_seated")
    ctx.expect_contact(long_distal, long_middle, name="long_tip_joint_seated")
    ctx.expect_contact(short_proximal, lower_support, name="short_base_joint_seated")
    ctx.expect_contact(short_middle, short_proximal, name="short_middle_joint_seated")
    ctx.expect_contact(short_distal, short_middle, name="short_tip_joint_seated")

    ctx.expect_origin_gap(
        long_proximal,
        short_proximal,
        axis="z",
        min_gap=0.030,
        name="finger_roots_are_vertically_separated",
    )
    ctx.expect_origin_gap(
        long_proximal,
        short_proximal,
        axis="x",
        min_gap=0.002,
        name="root_knuckles_are_x_offset",
    )

    ctx.check(
        "support_mounts_are_fixed",
        upper_mount.articulation_type == ArticulationType.FIXED
        and lower_mount.articulation_type == ArticulationType.FIXED,
        details="Both support blocks should be rigidly mounted to the palm.",
    )

    expected_axes = {
        upper_base: (0.0, 1.0, 0.0),
        upper_mid: (0.0, 1.0, 0.0),
        upper_tip: (0.0, 1.0, 0.0),
        lower_base: (0.0, -1.0, 0.0),
        lower_mid: (0.0, -1.0, 0.0),
        lower_tip: (0.0, -1.0, 0.0),
    }
    for joint, expected_axis in expected_axes.items():
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_and_limits",
            tuple(joint.axis) == expected_axis
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.90,
            details=f"{joint.name} should be a one-plane curl joint with a usable positive closing range.",
        )

    long_tip_aabb = ctx.part_world_aabb(long_distal)
    short_tip_aabb = ctx.part_world_aabb(short_distal)
    short_tip_origin = ctx.part_world_position(short_distal)
    if long_tip_aabb is not None and short_tip_aabb is not None and short_tip_origin is not None:
        ctx.check(
            "long_finger_reaches_farther_than_short_finger",
            long_tip_aabb[1][0] > short_tip_aabb[1][0] + 0.010,
            details="The longer finger should project noticeably farther forward than the hooked finger.",
        )
        ctx.check(
            "short_distal_has_hooked_tip",
            short_tip_aabb[1][2] > short_tip_origin[2] + 0.008,
            details="The short distal link should curl upward into a visible hook.",
        )

    with ctx.pose(
        upper_support_to_long_proximal=0.40,
        long_proximal_to_long_middle=0.30,
        long_middle_to_long_distal=0.20,
        lower_support_to_short_proximal=0.40,
        short_proximal_to_short_middle=0.30,
        short_middle_to_short_distal=0.20,
    ):
        ctx.expect_gap(
            short_distal,
            long_distal,
            axis="z",
            min_gap=0.006,
            max_gap=0.014,
            name="pinch_pose_tip_clearance",
        )
        ctx.expect_overlap(
            long_distal,
            short_distal,
            axes="x",
            min_overlap=0.0015,
            name="pinch_pose_tip_reach_alignment",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
