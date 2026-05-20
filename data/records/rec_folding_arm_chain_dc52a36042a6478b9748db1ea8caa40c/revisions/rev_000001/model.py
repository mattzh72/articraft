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


PLATE_THICKNESS = 0.008
PLATE_WIDTH = 0.120
PLATE_HEIGHT = 0.175
PLATE_CENTER_X = -0.020

ROOT_HINGE_X = 0.014
BARREL_RADIUS = 0.007
CENTER_BARREL_LEN = 0.010
OUTER_BARREL_LEN = 0.010
BARREL_OFFSET_Y = 0.5 * (CENTER_BARREL_LEN + OUTER_BARREL_LEN)
BARREL_RPY = (pi / 2.0, 0.0, 0.0)

LINK_HEIGHT = 0.028
TONGUE_WIDTH = 0.010
FORK_ARM_WIDTH = 0.010
FORK_ARM_HEIGHT = 0.024

LINK_1_LENGTH = 0.158
LINK_2_LENGTH = 0.132

ROOT_NECK_LEN = 0.022
BODY_START_X = 0.018
DISTAL_BRIDGE_LEN = 0.024
DISTAL_ARM_LEN = 0.022
DISTAL_BRIDGE_BACKOFF = 0.030
DISTAL_ARM_BACKOFF = 0.011

PAD_NECK_LEN = 0.024
PAD_HEAD_LEN = 0.032
PAD_WIDTH = 0.030
PAD_HEIGHT = 0.036


def _add_y_barrel(part, name: str, x: float, y: float, z: float = 0.0, *, length: float) -> None:
    part.visual(
        Cylinder(radius=BARREL_RADIUS, length=length),
        origin=Origin(xyz=(x, y, z), rpy=BARREL_RPY),
        name=name,
    )


def _add_mount_plate(part) -> None:
    part.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(xyz=(PLATE_CENTER_X, 0.0, 0.0)),
        name="plate",
    )
    for side, y in (("left", -BARREL_OFFSET_Y), ("right", BARREL_OFFSET_Y)):
        part.visual(
            Box((0.030, FORK_ARM_WIDTH, FORK_ARM_HEIGHT)),
            origin=Origin(xyz=(-0.002, y, 0.0)),
            name=f"{side}_arm",
        )
        _add_y_barrel(part, f"{side}_barrel", ROOT_HINGE_X, y, length=OUTER_BARREL_LEN)


def _add_link(part, *, length: float) -> None:
    body_end_x = length - (DISTAL_BRIDGE_BACKOFF + 0.012)
    body_length = body_end_x - BODY_START_X
    body_center_x = 0.5 * (BODY_START_X + body_end_x)

    _add_y_barrel(part, "root_barrel", 0.0, 0.0, length=CENTER_BARREL_LEN)
    part.visual(
        Box((ROOT_NECK_LEN, TONGUE_WIDTH, LINK_HEIGHT * 0.90)),
        origin=Origin(xyz=(ROOT_NECK_LEN / 2.0, 0.0, 0.0)),
        name="root_neck",
    )
    part.visual(
        Box((body_length, TONGUE_WIDTH, LINK_HEIGHT)),
        origin=Origin(xyz=(body_center_x, 0.0, 0.0)),
        name="spine",
    )
    part.visual(
        Box((DISTAL_BRIDGE_LEN, 0.026, LINK_HEIGHT * 0.62)),
        origin=Origin(xyz=(length - DISTAL_BRIDGE_BACKOFF, 0.0, 0.0)),
        name="distal_bridge",
    )
    for side, y in (("left", -BARREL_OFFSET_Y), ("right", BARREL_OFFSET_Y)):
        part.visual(
            Box((DISTAL_ARM_LEN, FORK_ARM_WIDTH, FORK_ARM_HEIGHT)),
            origin=Origin(xyz=(length - DISTAL_ARM_BACKOFF, y, 0.0)),
            name=f"{side}_arm",
        )
        _add_y_barrel(part, f"{side}_barrel", length, y, length=OUTER_BARREL_LEN)


def _add_end_pad(part) -> None:
    _add_y_barrel(part, "root_barrel", 0.0, 0.0, length=CENTER_BARREL_LEN)
    part.visual(
        Box((PAD_NECK_LEN, TONGUE_WIDTH, 0.024)),
        origin=Origin(xyz=(PAD_NECK_LEN / 2.0, 0.0, 0.0)),
        name="pad_neck",
    )
    part.visual(
        Box((PAD_HEAD_LEN, PAD_WIDTH, PAD_HEIGHT)),
        origin=Origin(xyz=(PAD_NECK_LEN + PAD_HEAD_LEN / 2.0, 0.0, 0.0)),
        name="pad_head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain")

    model.material("mount_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("link_alloy", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("pad_dark", rgba=(0.13, 0.14, 0.16, 1.0))

    mount_plate = model.part("mount_plate")
    _add_mount_plate(mount_plate)
    for visual in mount_plate.visuals:
        visual.material = "mount_steel"

    primary_link = model.part("primary_link")
    _add_link(primary_link, length=LINK_1_LENGTH)
    for visual in primary_link.visuals:
        visual.material = "link_alloy"

    secondary_link = model.part("secondary_link")
    _add_link(secondary_link, length=LINK_2_LENGTH)
    for visual in secondary_link.visuals:
        visual.material = "link_alloy"

    end_pad = model.part("end_pad")
    _add_end_pad(end_pad)
    for visual in end_pad.visuals:
        visual.material = "pad_dark"

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=mount_plate,
        child=primary_link,
        origin=Origin(xyz=(ROOT_HINGE_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.20, upper=2.35, effort=24.0, velocity=2.2),
    )
    model.articulation(
        "mid_hinge",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.45, upper=2.45, effort=18.0, velocity=2.6),
    )
    model.articulation(
        "pad_hinge",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=end_pad,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.30, upper=2.10, effort=10.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_plate = object_model.get_part("mount_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    end_pad = object_model.get_part("end_pad")

    root_hinge = object_model.get_articulation("root_hinge")
    mid_hinge = object_model.get_articulation("mid_hinge")
    pad_hinge = object_model.get_articulation("pad_hinge")

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

    ctx.expect_contact(mount_plate, primary_link, name="mount_plate_contacts_primary_link")
    ctx.expect_contact(primary_link, secondary_link, name="primary_link_contacts_secondary_link")
    ctx.expect_contact(secondary_link, end_pad, name="secondary_link_contacts_end_pad")

    ctx.expect_origin_gap(
        end_pad,
        mount_plate,
        axis="x",
        min_gap=0.28,
        max_gap=0.38,
        name="chain_reaches_forward_in_closed_pose",
    )
    ctx.expect_overlap(primary_link, secondary_link, axes="z", min_overlap=0.02, name="links_share_bend_plane")
    ctx.expect_overlap(secondary_link, end_pad, axes="z", min_overlap=0.02, name="pad_shares_bend_plane")

    for hinge in (root_hinge, mid_hinge, pad_hinge):
        ctx.check(
            f"{hinge.name}_is_revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={hinge.articulation_type}",
        )
        ctx.check(
            f"{hinge.name}_uses_common_bend_axis",
            tuple(round(value, 6) for value in hinge.axis) == (0.0, -1.0, 0.0),
            details=f"axis={hinge.axis}",
        )

    closed_pad_pos = ctx.part_world_position(end_pad)
    with ctx.pose(root_hinge=0.95, mid_hinge=0.70, pad_hinge=0.45):
        raised_pad_pos = ctx.part_world_position(end_pad)
        if closed_pad_pos is not None and raised_pad_pos is not None:
            ctx.check(
                "folded_pose_lifts_end_pad",
                raised_pad_pos[2] > closed_pad_pos[2] + 0.12,
                details=f"closed_z={closed_pad_pos[2]:.4f}, raised_z={raised_pad_pos[2]:.4f}",
            )
            ctx.check(
                "folded_pose_stays_in_single_bend_plane",
                abs(raised_pad_pos[1]) < 1e-4,
                details=f"raised_y={raised_pad_pos[1]:.6f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
