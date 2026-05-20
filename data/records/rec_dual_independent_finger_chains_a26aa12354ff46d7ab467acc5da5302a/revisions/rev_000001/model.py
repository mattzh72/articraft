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


PALM_WIDTH = 0.160
PALM_DEPTH = 0.074
PALM_HEIGHT = 0.030
KNUCKLE_BEAM_HEIGHT = 0.008
KNUCKLE_Y = 0.022
ROOT_JOINT_Z = 0.049
LEFT_BASE_X = -0.034
RIGHT_BASE_X = 0.034
FUSE_OVERLAP = 0.001

ROOT_SLOT_WIDTH = 0.016
ROOT_MOUNT_WIDTH = 0.028
ROOT_BARREL_RADIUS = 0.007
ROOT_EAR_DEPTH = 0.017

MID_SLOT_WIDTH = 0.012
MID_MOUNT_WIDTH = 0.020
MID_BARREL_RADIUS = 0.006

TIP_SLOT_WIDTH = 0.010
TIP_MOUNT_WIDTH = 0.016
TIP_BARREL_RADIUS = 0.005


def _add_finger_link(
    part,
    *,
    prefix: str,
    material,
    link_length: float,
    body_width: float,
    base_thickness: float,
    tip_thickness: float,
    root_barrel_radius: float,
    root_slot_width: float,
    distal_slot_width: float | None = None,
    distal_mount_width: float | None = None,
    distal_barrel_radius: float | None = None,
) -> None:
    has_child_joint = (
        distal_slot_width is not None
        and distal_mount_width is not None
        and distal_barrel_radius is not None
    )
    next_radius = distal_barrel_radius if has_child_joint else root_barrel_radius * 0.82

    root_web_depth = root_barrel_radius + base_thickness * 0.60
    body1_len = link_length * 0.68
    body2_len = link_length * 0.20
    body1_depth = base_thickness
    body2_depth = max(tip_thickness, next_radius * 1.25)

    part.visual(
        Cylinder(radius=root_barrel_radius, length=root_slot_width),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_root_barrel",
    )
    part.visual(
        Box((min(root_slot_width * 0.72, body_width * 0.86), root_web_depth, root_barrel_radius * 0.90)),
        origin=Origin(
            xyz=(
                0.0,
                root_web_depth / 2.0 - 0.001,
                root_barrel_radius * 0.34,
            )
        ),
        material=material,
        name=f"{prefix}_root_web",
    )
    part.visual(
        Box((body_width, body1_depth, body1_len)),
        origin=Origin(
            xyz=(
                0.0,
                root_barrel_radius + body1_depth / 2.0 + 0.002,
                body1_len / 2.0 + root_barrel_radius * 0.32,
            )
        ),
        material=material,
        name=f"{prefix}_body_main",
    )
    part.visual(
        Box((body_width * 0.82, body2_depth, body2_len)),
        origin=Origin(
            xyz=(
                0.0,
                root_barrel_radius + body2_depth / 2.0 + 0.0015,
                link_length - next_radius - 0.0005 - body2_len / 2.0,
            )
        ),
        material=material,
        name=f"{prefix}_body_tip",
    )

    if has_child_joint:
        ear_width = (distal_mount_width - distal_slot_width) / 2.0
        ear_depth = max(tip_thickness * 0.95, next_radius * 2.2)
        ear_height = next_radius * 2.0 + 0.004
        ear_center_y = max(0.0018, next_radius * 0.45)
        tip_block_len = max(0.0032, next_radius * 0.75)
        tip_block_depth = max(tip_thickness * 0.92, next_radius * 1.28)

        part.visual(
            Box((distal_mount_width * 0.96, tip_block_depth, tip_block_len)),
            origin=Origin(
                xyz=(
                    0.0,
                    max(0.0012, next_radius * 0.55) + tip_block_depth / 2.0 - 0.001,
                    link_length - next_radius - 0.0002 - tip_block_len / 2.0,
                )
            ),
            material=material,
            name=f"{prefix}_distal_mount",
        )
        for side, side_name in ((-1.0, "left"), (1.0, "right")):
            part.visual(
                Box((ear_width, ear_depth, ear_height)),
                origin=Origin(
                    xyz=(
                        side * (distal_slot_width / 2.0 + ear_width / 2.0),
                        ear_center_y,
                        link_length,
                    )
                ),
                material=material,
                name=f"{prefix}_{side_name}_ear",
            )
    else:
        fingertip_len = max(0.0045, next_radius * 1.30)
        fingertip_depth = max(tip_thickness * 0.92, next_radius * 1.18)
        part.visual(
            Box((body_width * 0.70, fingertip_depth, fingertip_len)),
            origin=Origin(
                xyz=(
                    0.0,
                    max(0.0012, next_radius * 0.70) + fingertip_depth / 2.0 - 0.001,
                    link_length - fingertip_len / 2.0,
                )
            ),
            material=material,
            name=f"{prefix}_fingertip",
        )


def _add_palm_mount(palm, *, prefix: str, material, base_x: float) -> None:
    pedestal_height = ROOT_JOINT_Z - ROOT_BARREL_RADIUS - PALM_HEIGHT
    ear_width = (ROOT_MOUNT_WIDTH - ROOT_SLOT_WIDTH) / 2.0
    ear_height = ROOT_BARREL_RADIUS * 2.0 + 0.004
    ear_center_y = KNUCKLE_Y + max(0.0018, ROOT_BARREL_RADIUS * 0.45)

    palm.visual(
        Box((ROOT_MOUNT_WIDTH, 0.018, pedestal_height)),
        origin=Origin(xyz=(base_x, KNUCKLE_Y, PALM_HEIGHT + pedestal_height / 2.0)),
        material=material,
        name=f"{prefix}_pedestal",
    )
    palm.visual(
        Box((ROOT_MOUNT_WIDTH * 1.10, 0.010, pedestal_height * 0.70)),
        origin=Origin(
            xyz=(
                base_x,
                KNUCKLE_Y - 0.008,
                PALM_HEIGHT + pedestal_height * 0.35,
            )
        ),
        material=material,
        name=f"{prefix}_rear_rib",
    )
    for side, side_name in ((-1.0, "left"), (1.0, "right")):
        palm.visual(
            Box((ear_width, ROOT_EAR_DEPTH, ear_height)),
            origin=Origin(
                xyz=(
                    base_x + side * (ROOT_SLOT_WIDTH / 2.0 + ear_width / 2.0),
                    ear_center_y,
                    ROOT_JOINT_Z,
                )
            ),
            material=material,
            name=f"{prefix}_{side_name}_ear",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_palm_dual_finger_module")

    palm_material = model.material("palm_plastic", color=(0.19, 0.22, 0.25))
    left_material = model.material("left_finger_metal", color=(0.70, 0.73, 0.76))
    right_material = model.material("right_finger_metal", color=(0.62, 0.66, 0.70))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_DEPTH, PALM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PALM_HEIGHT / 2.0)),
        material=palm_material,
        name="palm_body",
    )
    palm.visual(
        Box((PALM_WIDTH * 0.88, 0.016, KNUCKLE_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, KNUCKLE_Y - 0.004, PALM_HEIGHT + KNUCKLE_BEAM_HEIGHT / 2.0)),
        material=palm_material,
        name="palm_knuckle_beam",
    )
    _add_palm_mount(palm, prefix="left_root", material=palm_material, base_x=LEFT_BASE_X)
    _add_palm_mount(palm, prefix="right_root", material=palm_material, base_x=RIGHT_BASE_X)

    left_proximal = model.part("left_proximal")
    _add_finger_link(
        left_proximal,
        prefix="left_proximal",
        material=left_material,
        link_length=0.060,
        body_width=0.014,
        base_thickness=0.020,
        tip_thickness=0.017,
        root_barrel_radius=ROOT_BARREL_RADIUS,
        root_slot_width=ROOT_SLOT_WIDTH,
        distal_slot_width=MID_SLOT_WIDTH,
        distal_mount_width=MID_MOUNT_WIDTH,
        distal_barrel_radius=MID_BARREL_RADIUS,
    )

    left_middle = model.part("left_middle")
    _add_finger_link(
        left_middle,
        prefix="left_middle",
        material=left_material,
        link_length=0.042,
        body_width=0.0115,
        base_thickness=0.0165,
        tip_thickness=0.0138,
        root_barrel_radius=MID_BARREL_RADIUS,
        root_slot_width=MID_SLOT_WIDTH,
        distal_slot_width=TIP_SLOT_WIDTH,
        distal_mount_width=TIP_MOUNT_WIDTH,
        distal_barrel_radius=TIP_BARREL_RADIUS,
    )

    left_distal = model.part("left_distal")
    _add_finger_link(
        left_distal,
        prefix="left_distal",
        material=left_material,
        link_length=0.032,
        body_width=0.0095,
        base_thickness=0.0130,
        tip_thickness=0.0100,
        root_barrel_radius=TIP_BARREL_RADIUS,
        root_slot_width=TIP_SLOT_WIDTH,
    )

    right_proximal = model.part("right_proximal")
    _add_finger_link(
        right_proximal,
        prefix="right_proximal",
        material=right_material,
        link_length=0.055,
        body_width=0.0135,
        base_thickness=0.0192,
        tip_thickness=0.0162,
        root_barrel_radius=ROOT_BARREL_RADIUS,
        root_slot_width=ROOT_SLOT_WIDTH,
        distal_slot_width=MID_SLOT_WIDTH,
        distal_mount_width=MID_MOUNT_WIDTH,
        distal_barrel_radius=MID_BARREL_RADIUS,
    )

    right_middle = model.part("right_middle")
    _add_finger_link(
        right_middle,
        prefix="right_middle",
        material=right_material,
        link_length=0.039,
        body_width=0.0110,
        base_thickness=0.0158,
        tip_thickness=0.0130,
        root_barrel_radius=MID_BARREL_RADIUS,
        root_slot_width=MID_SLOT_WIDTH,
        distal_slot_width=TIP_SLOT_WIDTH,
        distal_mount_width=TIP_MOUNT_WIDTH,
        distal_barrel_radius=TIP_BARREL_RADIUS,
    )

    right_distal = model.part("right_distal")
    _add_finger_link(
        right_distal,
        prefix="right_distal",
        material=right_material,
        link_length=0.030,
        body_width=0.0090,
        base_thickness=0.0125,
        tip_thickness=0.0098,
        root_barrel_radius=TIP_BARREL_RADIUS,
        root_slot_width=TIP_SLOT_WIDTH,
    )

    revolute_limits = MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35)

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(LEFT_BASE_X, KNUCKLE_Y, ROOT_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "left_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "left_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.2, lower=0.0, upper=1.35),
    )

    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(RIGHT_BASE_X, KNUCKLE_Y, ROOT_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "right_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.2, lower=0.0, upper=1.35),
    )

    return model


def _axis_value(vec: tuple[float, float, float] | list[float], axis: str) -> float:
    return {"x": vec[0], "y": vec[1], "z": vec[2]}[axis]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    palm_to_left = object_model.get_articulation("palm_to_left_proximal")
    left_to_mid = object_model.get_articulation("left_proximal_to_middle")
    mid_to_tip_left = object_model.get_articulation("left_middle_to_distal")
    palm_to_right = object_model.get_articulation("palm_to_right_proximal")
    right_to_mid = object_model.get_articulation("right_proximal_to_middle")
    mid_to_tip_right = object_model.get_articulation("right_middle_to_distal")

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

    for joint in (
        palm_to_left,
        left_to_mid,
        mid_to_tip_left,
        palm_to_right,
        right_to_mid,
        mid_to_tip_right,
    ):
        ctx.check(
            f"{joint.name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"{joint.name} should be a revolute hinge.",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has flexion limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower == 0.0
            and limits.upper > 1.2,
            f"{joint.name} should have forward flexion limits with a useful travel range.",
        )

    ctx.expect_contact(palm, left_proximal, name="left root hinge is physically seated")
    ctx.expect_contact(
        left_proximal,
        left_middle,
        name="left middle hinge stays mounted to proximal link",
    )
    ctx.expect_contact(
        left_middle,
        left_distal,
        name="left distal hinge stays mounted to middle link",
    )
    ctx.expect_contact(palm, right_proximal, name="right root hinge is physically seated")
    ctx.expect_contact(
        right_proximal,
        right_middle,
        name="right middle hinge stays mounted to proximal link",
    )
    ctx.expect_contact(
        right_middle,
        right_distal,
        name="right distal hinge stays mounted to middle link",
    )

    left_middle_pos = ctx.part_world_position(left_middle)
    right_middle_pos = ctx.part_world_position(right_middle)
    left_distal_pos = ctx.part_world_position(left_distal)
    right_distal_pos = ctx.part_world_position(right_distal)
    ctx.check(
        "finger lengths differ across the pair",
        left_middle_pos is not None
        and right_middle_pos is not None
        and left_distal_pos is not None
        and right_distal_pos is not None
        and _axis_value(left_middle_pos, "z") - _axis_value(right_middle_pos, "z") > 0.004
        and _axis_value(left_distal_pos, "z") - _axis_value(right_distal_pos, "z") > 0.007,
        "The two fingers should have slightly different link lengths rather than perfect mirror symmetry.",
    )

    left_rest = ctx.part_world_position(left_distal)
    right_rest = ctx.part_world_position(right_distal)
    with ctx.pose(
        {
            palm_to_left: 0.70,
            left_to_mid: 0.92,
            mid_to_tip_left: 0.65,
            palm_to_right: 0.62,
            right_to_mid: 0.80,
            mid_to_tip_right: 0.60,
        }
    ):
        left_flex = ctx.part_world_position(left_distal)
        right_flex = ctx.part_world_position(right_distal)
        ctx.expect_contact(
            palm,
            left_proximal,
            name="left root hinge keeps contact while flexed",
        )
        ctx.expect_contact(
            left_proximal,
            left_middle,
            name="left second hinge keeps contact while flexed",
        )
        ctx.expect_contact(
            left_middle,
            left_distal,
            name="left third hinge keeps contact while flexed",
        )
        ctx.expect_contact(
            palm,
            right_proximal,
            name="right root hinge keeps contact while flexed",
        )
        ctx.check(
            "left finger curls back over the palm",
            left_rest is not None
            and left_flex is not None
            and _axis_value(left_flex, "y") < _axis_value(left_rest, "y") - 0.025
            and _axis_value(left_flex, "z") < _axis_value(left_rest, "z") - 0.020,
            "Positive left finger motion should carry the distal link rearward and downward over the palm.",
        )
        ctx.check(
            "right finger curls back over the palm",
            right_rest is not None
            and right_flex is not None
            and _axis_value(right_flex, "y") < _axis_value(right_rest, "y") - 0.020
            and _axis_value(right_flex, "z") < _axis_value(right_rest, "z") - 0.018,
            "Positive right finger motion should carry the distal link rearward and downward over the palm.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
