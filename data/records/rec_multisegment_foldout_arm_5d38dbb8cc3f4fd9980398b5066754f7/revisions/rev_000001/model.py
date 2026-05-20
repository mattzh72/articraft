from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_T = 0.010
BASE_PLATE_W = 0.082
BASE_PLATE_H = 0.124
BASE_BLOCK_LEN = 0.020
ROOT_AXIS_X = 0.0
LINK_PROX_LEN = 0.012
LINK_DIST_LEN = 0.012
CHEEK_T = 0.005
LUG_W = 0.020
TOTAL_W = LUG_W + 2.0 * CHEEK_T
CHEEK_Y = 0.5 * LUG_W + 0.5 * CHEEK_T
BODY_T = 0.010
STACK_OFFSET = 0.014
BODY_LANE_OFFSET = 0.018
TRANSITION_LEN = 0.014
JOINT_PAD_H = 0.012
LINK_LENGTHS = (0.092, 0.086, 0.080, 0.074)
LINK_STACK_SIGNS = (1.0, -1.0, 1.0, -1.0)
FOLD_POSE = {
    "base_to_link_1": 1.0,
    "link_1_to_link_2": -2.25,
    "link_2_to_link_3": 0.45,
    "link_3_to_link_4": -2.25,
}


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse_all(solids: list[cq.Workplane]) -> cq.Workplane:
    shape = solids[0]
    for solid in solids[1:]:
        shape = shape.union(solid)
    return shape


def _link_shape(length: float, stack_sign: float) -> cq.Workplane:
    lane_y = stack_sign * BODY_LANE_OFFSET
    lane_z = stack_sign * STACK_OFFSET
    main_len = length - LINK_PROX_LEN - LINK_DIST_LEN - 2.0 * TRANSITION_LEN
    lateral_span = BODY_LANE_OFFSET + BODY_T
    vertical_span = STACK_OFFSET + BODY_T
    proximal_transition_x = LINK_PROX_LEN + 0.5 * TRANSITION_LEN
    distal_transition_x = length - LINK_DIST_LEN - 0.5 * TRANSITION_LEN

    solids = [
        _box((LINK_PROX_LEN, TOTAL_W, JOINT_PAD_H), (0.5 * LINK_PROX_LEN, 0.0, 0.0)),
        _box(
            (TRANSITION_LEN, lateral_span, BODY_T),
            (proximal_transition_x, 0.5 * lane_y, lane_z),
        ),
        _box(
            (TRANSITION_LEN, BODY_T, vertical_span),
            (proximal_transition_x, lane_y, 0.5 * lane_z),
        ),
        _box(
            (main_len, BODY_T, BODY_T),
            (LINK_PROX_LEN + TRANSITION_LEN + 0.5 * main_len, lane_y, lane_z),
        ),
        _box(
            (TRANSITION_LEN, lateral_span, BODY_T),
            (distal_transition_x, 0.5 * lane_y, lane_z),
        ),
        _box(
            (TRANSITION_LEN, BODY_T, vertical_span),
            (distal_transition_x, lane_y, 0.5 * lane_z),
        ),
        _box((LINK_DIST_LEN, TOTAL_W, JOINT_PAD_H), (length - 0.5 * LINK_DIST_LEN, 0.0, 0.0)),
    ]
    return _fuse_all(solids)


def _base_shape() -> cq.Workplane:
    plate = _box(
        (BASE_PLATE_T, BASE_PLATE_W, BASE_PLATE_H),
        (ROOT_AXIS_X - BASE_BLOCK_LEN - 0.5 * BASE_PLATE_T, 0.0, 0.0),
    )
    block = _box((BASE_BLOCK_LEN, TOTAL_W, 0.040), (ROOT_AXIS_X - 0.5 * BASE_BLOCK_LEN, 0.0, 0.0))
    upper_rib = _box((BASE_BLOCK_LEN, 0.016, 0.018), (ROOT_AXIS_X - 0.5 * BASE_BLOCK_LEN, 0.0, 0.028))
    lower_rib = _box((BASE_BLOCK_LEN, 0.016, 0.018), (ROOT_AXIS_X - 0.5 * BASE_BLOCK_LEN, 0.0, -0.028))
    hinge_pad = _box((0.006, TOTAL_W, JOINT_PAD_H), (ROOT_AXIS_X - 0.003, 0.0, 0.0))
    return _fuse_all([plate, block, upper_rib, lower_rib, hinge_pad])


def _platform_bracket_shape() -> cq.Workplane:
    mount_len = 0.012
    solids = [
        _box((mount_len, TOTAL_W, JOINT_PAD_H), (0.5 * mount_len, 0.0, 0.0)),
        _box((0.010, 0.020, 0.020), (mount_len + 0.005, 0.0, 0.010)),
        _box((0.028, 0.028, 0.005), (mount_len + 0.014, 0.0, 0.0225)),
        _box((0.012, 0.016, 0.020), (mount_len + 0.006, 0.0, 0.002)),
    ]
    return _fuse_all(solids)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_segment_fold_out_arm")

    model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("satin_aluminum", rgba=(0.71, 0.74, 0.77, 1.0))
    model.material("bracket_gray", rgba=(0.34, 0.37, 0.41, 1.0))

    base = model.part("base_plate")
    base.visual(mesh_from_cadquery(_base_shape(), "base_plate"), material="powder_black", name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((BASE_BLOCK_LEN + BASE_PLATE_T, BASE_PLATE_W, BASE_PLATE_H)),
        mass=1.4,
        origin=Origin(xyz=(ROOT_AXIS_X - 0.5 * (BASE_BLOCK_LEN + BASE_PLATE_T), 0.0, 0.0)),
    )

    link_parts = []
    for index, (length, stack_sign) in enumerate(zip(LINK_LENGTHS, LINK_STACK_SIGNS), start=1):
        link = model.part(f"link_{index}")
        link.visual(
            mesh_from_cadquery(_link_shape(length, stack_sign), f"link_{index}"),
            material="satin_aluminum",
            name=f"link_{index}_shell",
        )
        link.inertial = Inertial.from_geometry(
            Box((length, TOTAL_W, BODY_T + STACK_OFFSET)),
            mass=0.20 - 0.02 * (index - 1),
            origin=Origin(xyz=(0.5 * length, 0.0, 0.5 * stack_sign * STACK_OFFSET)),
        )
        link_parts.append(link)

    platform = model.part("platform_bracket")
    platform.visual(
        mesh_from_cadquery(_platform_bracket_shape(), "platform_bracket"),
        material="bracket_gray",
        name="platform_shell",
    )
    platform.inertial = Inertial.from_geometry(
        Box((0.052, 0.032, 0.049)),
        mass=0.18,
        origin=Origin(xyz=(0.022, 0.0, 0.024)),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_parts[0],
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=1.60, effort=20.0, velocity=1.5),
    )
    for index in range(3):
        model.articulation(
            f"link_{index + 1}_to_link_{index + 2}",
            ArticulationType.REVOLUTE,
            parent=link_parts[index],
            child=link_parts[index + 1],
            origin=Origin(xyz=(LINK_LENGTHS[index], 0.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=-2.80, upper=2.80, effort=14.0, velocity=2.0),
        )

    model.articulation(
        "link_4_to_platform",
        ArticulationType.FIXED,
        parent=link_parts[-1],
        child=platform,
        origin=Origin(xyz=(LINK_LENGTHS[-1], 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    platform = object_model.get_part("platform_bracket")

    joint_1 = object_model.get_articulation("base_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")
    joint_4 = object_model.get_articulation("link_3_to_link_4")

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

    serial_joints = (joint_1, joint_2, joint_3, joint_4)
    ctx.check(
        "four_parallel_revolute_joints",
        len(serial_joints) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in serial_joints)
        and all(tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in serial_joints),
        details="The arm should use four serial revolute joints with parallel pitch axes.",
    )

    for first, second, name in (
        (base, link_1, "root_hinge_has_physical_contact"),
        (link_1, link_2, "joint_1_has_physical_contact"),
        (link_2, link_3, "joint_2_has_physical_contact"),
        (link_3, link_4, "joint_3_has_physical_contact"),
        (link_4, platform, "platform_bracket_is_mounted_to_last_link"),
    ):
        ctx.expect_contact(first, second, name=name)

    deployed_platform_pos = ctx.part_world_position(platform)
    ctx.check(
        "deployed_pose_reaches_outward",
        deployed_platform_pos is not None and deployed_platform_pos[0] > 0.30,
        details=f"Expected the deployed arm to project forward, got platform position {deployed_platform_pos}.",
    )

    with ctx.pose(FOLD_POSE):
        folded_platform_pos = ctx.part_world_position(platform)
        folded_base_pos = ctx.part_world_position(base)
        folded_radius = None
        if folded_platform_pos is not None and folded_base_pos is not None:
            folded_radius = hypot(
                folded_platform_pos[0] - folded_base_pos[0],
                folded_platform_pos[2] - folded_base_pos[2],
            )
        ctx.check(
            "compact_fold_returns_platform_near_root",
            folded_radius is not None and folded_radius < 0.10,
            details=(
                "The folded configuration should collapse near the base; "
                f"measured root-to-platform radius={folded_radius}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
