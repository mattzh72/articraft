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


PALM_BASE_SIZE = (0.18, 0.08, 0.012)
PALM_STANCHION_SIZE = (0.022, 0.024, 0.046)
PALM_BRIDGE_SIZE = (0.128, 0.036, 0.012)
PALM_STANCHION_X = 0.043
PALM_STANCHION_Y = -0.008

ROOT_JOINT_Y = 0.018
ROOT_JOINT_Z = 0.056
FINGER_X_OFFSET = 0.038

INNER_GAP = 0.014
CHEEK_THICKNESS = 0.003
CHEEK_DEPTH = 0.012
CHEEK_HEIGHT = 0.018
CHEEK_CENTER_OFFSET_X = (INNER_GAP / 2.0) + (CHEEK_THICKNESS / 2.0)

BARREL_RADIUS = 0.0046
BARREL_LENGTH = INNER_GAP
BEAM_WIDTH = 0.009
BEAM_HEIGHT = 0.009
BEAM_CENTER_Z = -0.005

RIB_WIDTH = 0.006
RIB_DEPTH = 0.008
RIB_HEIGHT = 0.006
RIB_CENTER_Z = -0.006
RIB_CENTER_OFFSET_X = 0.006
RIB_CENTER_OFFSET_Y = 0.010

PROXIMAL_PITCH = 0.060
MIDDLE_PITCH = 0.050
DISTAL_LENGTH = 0.042


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def _aabb_size(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[i] - mins[i] for i in range(3))


def _add_cheeks(part, joint_y: float, joint_z: float, x_offset: float = 0.0) -> None:
    for cheek_sign in (-1.0, 1.0):
        part.visual(
            Box((CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_offset + (cheek_sign * CHEEK_CENTER_OFFSET_X),
                    joint_y,
                    joint_z,
                )
            ),
        )


def _add_support_ribs(part, anchor_y: float, anchor_z: float, x_offset: float = 0.0) -> None:
    for rib_sign in (-1.0, 1.0):
        part.visual(
            Box((RIB_WIDTH, RIB_DEPTH, RIB_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_offset + (rib_sign * RIB_CENTER_OFFSET_X),
                    anchor_y - RIB_CENTER_OFFSET_Y,
                    anchor_z + RIB_CENTER_Z,
                )
            ),
        )


def _add_knuckle_barrel(part) -> None:
    part.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )


def _add_finger_link(part, *, pitch: float) -> None:
    beam_start = 0.001
    beam_end = pitch - 0.008
    beam_length = beam_end - beam_start
    beam_center_y = (beam_start + beam_end) / 2.0

    _add_knuckle_barrel(part)
    part.visual(
        Box((BEAM_WIDTH, beam_length, BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, beam_center_y, BEAM_CENTER_Z)),
    )
    _add_support_ribs(part, pitch, 0.0)
    _add_cheeks(part, pitch, 0.0)


def _add_distal_link(part) -> None:
    beam_start = 0.001
    beam_end = DISTAL_LENGTH - 0.002
    beam_length = beam_end - beam_start
    beam_center_y = (beam_start + beam_end) / 2.0

    _add_knuckle_barrel(part)
    part.visual(
        Box((BEAM_WIDTH, beam_length, BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, beam_center_y, BEAM_CENTER_Z)),
    )
    part.visual(
        Box((BEAM_WIDTH * 1.15, 0.010, BEAM_HEIGHT * 0.8)),
        origin=Origin(xyz=(0.0, DISTAL_LENGTH - 0.003, BEAM_CENTER_Z - 0.0008)),
    )
    part.visual(
        Box((BEAM_WIDTH * 0.95, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, DISTAL_LENGTH - 0.005, BEAM_CENTER_Z - 0.0036)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_palm_twin_finger")

    palm_color = model.material("palm_graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    finger_color = model.material("finger_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    palm = model.part("palm_bridge")
    palm.visual(
        Box(PALM_BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, PALM_BASE_SIZE[2] / 2.0)),
        material=palm_color,
        name="palm_bridge_body",
    )
    stanchion_center_z = PALM_BASE_SIZE[2] + (PALM_STANCHION_SIZE[2] / 2.0)
    for x_pos in (-PALM_STANCHION_X, PALM_STANCHION_X):
        palm.visual(
            Box(PALM_STANCHION_SIZE),
            origin=Origin(xyz=(x_pos, PALM_STANCHION_Y, stanchion_center_z)),
            material=palm_color,
        )
    palm.visual(
        Box(PALM_BRIDGE_SIZE),
        origin=Origin(xyz=(0.0, -0.006, ROOT_JOINT_Z - 0.006)),
        material=palm_color,
    )
    for x_pos in (-FINGER_X_OFFSET, FINGER_X_OFFSET):
        _add_support_ribs(palm, ROOT_JOINT_Y, ROOT_JOINT_Z, x_offset=x_pos)
        _add_cheeks(palm, ROOT_JOINT_Y, ROOT_JOINT_Z, x_offset=x_pos)

    left_proximal = model.part("left_proximal")
    _add_finger_link(left_proximal, pitch=PROXIMAL_PITCH)
    for visual in left_proximal.visuals:
        visual.material = finger_color

    left_middle = model.part("left_middle")
    _add_finger_link(left_middle, pitch=MIDDLE_PITCH)
    for visual in left_middle.visuals:
        visual.material = finger_color

    left_distal = model.part("left_distal")
    _add_distal_link(left_distal)
    for visual in left_distal.visuals:
        visual.material = finger_color

    right_proximal = model.part("right_proximal")
    _add_finger_link(right_proximal, pitch=PROXIMAL_PITCH)
    for visual in right_proximal.visuals:
        visual.material = finger_color

    right_middle = model.part("right_middle")
    _add_finger_link(right_middle, pitch=MIDDLE_PITCH)
    for visual in right_middle.visuals:
        visual.material = finger_color

    right_distal = model.part("right_distal")
    _add_distal_link(right_distal)
    for visual in right_distal.visuals:
        visual.material = finger_color

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(-FINGER_X_OFFSET, ROOT_JOINT_Y, ROOT_JOINT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.0, effort=8.0, velocity=2.2),
    )
    model.articulation(
        "left_proximal_to_left_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, PROXIMAL_PITCH, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "left_middle_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, MIDDLE_PITCH, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=4.0, velocity=2.8),
    )
    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(FINGER_X_OFFSET, ROOT_JOINT_Y, ROOT_JOINT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.0, effort=8.0, velocity=2.2),
    )
    model.articulation(
        "right_proximal_to_right_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, PROXIMAL_PITCH, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "right_middle_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, MIDDLE_PITCH, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=4.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    palm = object_model.get_part("palm_bridge")
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

    for part in (
        palm,
        left_proximal,
        left_middle,
        left_distal,
        right_proximal,
        right_middle,
        right_distal,
    ):
        ctx.check(f"{part.name} present", part is not None)

    for joint in (left_root, left_mid, left_tip, right_root, right_mid, right_tip):
        ctx.check(
            f"{joint.name} axis along finger knuckle pin",
            joint.axis == (-1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_contact(
        palm,
        left_proximal,
        contact_tol=0.0008,
        name="left proximal link is supported by the palm fork",
    )
    ctx.expect_contact(
        left_proximal,
        left_middle,
        contact_tol=0.0008,
        name="left middle link is captured in the proximal fork",
    )
    ctx.expect_contact(
        left_middle,
        left_distal,
        contact_tol=0.0008,
        name="left distal link is captured in the middle fork",
    )
    ctx.expect_contact(
        palm,
        right_proximal,
        contact_tol=0.0008,
        name="right proximal link is supported by the palm fork",
    )
    ctx.expect_contact(
        right_proximal,
        right_middle,
        contact_tol=0.0008,
        name="right middle link is captured in the proximal fork",
    )
    ctx.expect_contact(
        right_middle,
        right_distal,
        contact_tol=0.0008,
        name="right distal link is captured in the middle fork",
    )

    left_prox_size = _aabb_size(ctx.part_world_aabb(left_proximal))
    left_dist_size = _aabb_size(ctx.part_world_aabb(left_distal))
    ctx.check(
        "proximal link reads longer than distal link",
        left_prox_size is not None
        and left_dist_size is not None
        and left_prox_size[1] > left_dist_size[1] + 0.012,
        details=f"left_prox_size={left_prox_size}, left_dist_size={left_dist_size}",
    )

    right_prox_rest = _aabb_center(ctx.part_world_aabb(right_proximal))
    with ctx.pose({right_root: 0.55}):
        right_prox_curled = _aabb_center(ctx.part_world_aabb(right_proximal))
    ctx.check(
        "positive right root rotation curls the finger downward",
        right_prox_rest is not None
        and right_prox_curled is not None
        and right_prox_curled[2] < right_prox_rest[2] - 0.01,
        details=f"rest={right_prox_rest}, curled={right_prox_curled}",
    )

    left_distal_rest = _aabb_center(ctx.part_world_aabb(left_distal))
    right_distal_rest = _aabb_center(ctx.part_world_aabb(right_distal))
    with ctx.pose({left_root: 0.55, left_mid: 0.72, left_tip: 0.58}):
        left_distal_curled = _aabb_center(ctx.part_world_aabb(left_distal))
        right_distal_steady = _aabb_center(ctx.part_world_aabb(right_distal))
    ctx.check(
        "left finger independently curls while the right finger stays put",
        left_distal_rest is not None
        and left_distal_curled is not None
        and right_distal_rest is not None
        and right_distal_steady is not None
        and left_distal_curled[2] < left_distal_rest[2] - 0.03
        and abs(right_distal_steady[1] - right_distal_rest[1]) < 0.00005
        and abs(right_distal_steady[2] - right_distal_rest[2]) < 0.00005,
        details=(
            f"left_rest={left_distal_rest}, left_curled={left_distal_curled}, "
            f"right_rest={right_distal_rest}, right_steady={right_distal_steady}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
