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


SHOULDER_GAP = 0.046
ELBOW_GAP = 0.028
UPPER_LINK_LENGTH = 0.250
FOREARM_PLATE_CENTER_X = 0.207


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_elbow_arm")

    model.material("frame_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("link_alloy", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("joint_dark", rgba=(0.28, 0.30, 0.33, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        "base_block",
        (0.120, 0.160, 0.030),
        (-0.085, 0.0, -0.125),
        material="frame_steel",
    )
    _add_box(
        frame,
        "rear_web",
        (0.040, 0.100, 0.118),
        (-0.075, 0.0, -0.052),
        material="frame_steel",
    )
    _add_box(
        frame,
        "bridge_block",
        (0.080, 0.100, 0.026),
        (-0.020, 0.0, 0.030),
        material="frame_steel",
    )
    _add_box(
        frame,
        "left_backer",
        (0.050, 0.018, 0.048),
        (-0.040, 0.030, -0.006),
        material="frame_steel",
    )
    _add_box(
        frame,
        "right_backer",
        (0.050, 0.018, 0.048),
        (-0.040, -0.030, -0.006),
        material="frame_steel",
    )
    _add_box(
        frame,
        "left_saddle",
        (0.036, 0.020, 0.080),
        (0.000, 0.033, 0.000),
        material="frame_steel",
    )
    _add_box(
        frame,
        "right_saddle",
        (0.036, 0.020, 0.080),
        (0.000, -0.033, 0.000),
        material="frame_steel",
    )

    upper_link = model.part("upper_link")
    _add_y_cylinder(
        upper_link,
        "shoulder_boss",
        radius=0.021,
        length=SHOULDER_GAP,
        xyz=(0.0, 0.0, 0.0),
        material="joint_dark",
    )
    _add_box(
        upper_link,
        "beam_main",
        (0.190, 0.030, 0.040),
        (0.110, 0.0, 0.0),
        material="link_alloy",
    )
    _add_box(
        upper_link,
        "beam_nose",
        (0.030, 0.028, 0.036),
        (0.210, 0.0, 0.0),
        material="link_alloy",
    )
    _add_box(
        upper_link,
        "yoke_web",
        (0.012, 0.056, 0.046),
        (0.219, 0.0, 0.0),
        material="link_alloy",
    )
    _add_box(
        upper_link,
        "left_yoke",
        (0.042, 0.014, 0.066),
        (0.237, 0.021, 0.0),
        material="link_alloy",
    )
    _add_box(
        upper_link,
        "right_yoke",
        (0.042, 0.014, 0.066),
        (0.237, -0.021, 0.0),
        material="link_alloy",
    )

    forearm_link = model.part("forearm_link")
    _add_y_cylinder(
        forearm_link,
        "elbow_boss",
        radius=0.018,
        length=ELBOW_GAP,
        xyz=(0.0, 0.0, 0.0),
        material="joint_dark",
    )
    _add_box(
        forearm_link,
        "beam_main",
        (0.160, 0.028, 0.034),
        (0.092, 0.0, 0.0),
        material="link_alloy",
    )
    _add_box(
        forearm_link,
        "beam_nose",
        (0.034, 0.036, 0.030),
        (0.183, 0.0, 0.0),
        material="link_alloy",
    )
    _add_box(
        forearm_link,
        "end_plate",
        (0.018, 0.082, 0.082),
        (FOREARM_PLATE_CENTER_X, 0.0, 0.0),
        material="link_alloy",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=-0.85,
            upper=1.20,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm_link,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=32.0,
            velocity=1.8,
            lower=0.0,
            upper=2.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _center_z(aabb):
        return 0.5 * (aabb[0][2] + aabb[1][2])

    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    upper_link = object_model.get_part("upper_link")
    forearm_link = object_model.get_part("forearm_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

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
        frame,
        upper_link,
        elem_a="left_saddle",
        elem_b="shoulder_boss",
        contact_tol=0.001,
        name="left_shoulder_saddle_contacts_boss",
    )
    ctx.expect_contact(
        frame,
        upper_link,
        elem_a="right_saddle",
        elem_b="shoulder_boss",
        contact_tol=0.001,
        name="right_shoulder_saddle_contacts_boss",
    )
    ctx.expect_contact(
        upper_link,
        forearm_link,
        elem_a="left_yoke",
        elem_b="elbow_boss",
        contact_tol=0.001,
        name="left_elbow_yoke_contacts_boss",
    )
    ctx.expect_contact(
        upper_link,
        forearm_link,
        elem_a="right_yoke",
        elem_b="elbow_boss",
        contact_tol=0.001,
        name="right_elbow_yoke_contacts_boss",
    )
    ctx.check(
        "joint_axes_are_shared_pitch_axes",
        shoulder_joint.axis == (0.0, -1.0, 0.0) and elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"shoulder axis={shoulder_joint.axis}, elbow axis={elbow_joint.axis}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0}):
        straight_elbow_pos = ctx.part_world_position(forearm_link)
        straight_forearm_aabb = ctx.part_world_aabb(forearm_link)

    with ctx.pose({shoulder_joint: 0.85, elbow_joint: 0.0}):
        raised_elbow_pos = ctx.part_world_position(forearm_link)

    ctx.check(
        "positive_shoulder_motion_raises_elbow",
        straight_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > straight_elbow_pos[2] + 0.12,
        details=f"straight elbow={straight_elbow_pos}, raised elbow={raised_elbow_pos}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 1.20}):
        flexed_forearm_aabb = ctx.part_world_aabb(forearm_link)

    ctx.check(
        "positive_elbow_motion_lifts_forearm_mass",
        straight_forearm_aabb is not None
        and flexed_forearm_aabb is not None
        and _center_z(flexed_forearm_aabb) > _center_z(straight_forearm_aabb) + 0.05,
        details=f"straight aabb={straight_forearm_aabb}, flexed aabb={flexed_forearm_aabb}",
    )

    if straight_forearm_aabb is None:
        ctx.fail("forearm_plate_readable", "forearm AABB unavailable in straight pose")
    else:
        plate_y = straight_forearm_aabb[1][1] - straight_forearm_aabb[0][1]
        plate_z = straight_forearm_aabb[1][2] - straight_forearm_aabb[0][2]
        ctx.check(
            "forearm_reads_with_square_end_plate",
            plate_y >= 0.080 and plate_z >= 0.080 and abs(plate_y - plate_z) <= 0.006,
            details=f"forearm yz extents=({plate_y:.4f}, {plate_z:.4f})",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
