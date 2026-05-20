from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot

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


PLATE_THICKNESS = 0.006
PLATE_WIDTH = 0.050
PLATE_HEIGHT = 0.090
BASE_STANDOFF = 0.018

LINK_WIDTH = 0.010
LINK_PITCH = 0.055
HINGE_PAD_LENGTH = 0.016
HINGE_PAD_HEIGHT = 0.002
LINK_BODY_HEIGHT = 0.014

BASE_BRACKET_LENGTH = 0.018
BASE_BRACKET_HEIGHT = 0.022
BASE_HINGE_PAD_LENGTH = 0.006
BASE_BRACKET_WIDTH = 0.018

PLATFORM_FLANGE_LENGTH = 0.006
PLATFORM_WIDTH = 0.028
PLATFORM_DEPTH = 0.038
PLATFORM_THICKNESS = 0.004
PLATFORM_LIP_HEIGHT = 0.010
BRACKET_FLANGE_HEIGHT = 0.022
BRACKET_FLANGE_THICKNESS = 0.004

REVOLUTE_LIMITS = MotionLimits(
    effort=8.0,
    velocity=2.5,
    lower=-1.05,
    upper=1.05,
)


def _box_segment(x_start: float, x_end: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(
        x_end - x_start,
        width,
        height,
    ).translate(((x_start + x_end) / 2.0, 0.0, 0.0))


def _make_link_shape(*, terminal: bool = False) -> cq.Workplane:
    start_pad = _box_segment(0.0, HINGE_PAD_LENGTH, LINK_WIDTH, HINGE_PAD_HEIGHT)

    if terminal:
        body = _box_segment(HINGE_PAD_LENGTH, LINK_PITCH, LINK_WIDTH, LINK_BODY_HEIGHT)
        return start_pad.union(body)

    center_body = _box_segment(
        HINGE_PAD_LENGTH,
        LINK_PITCH - HINGE_PAD_LENGTH,
        LINK_WIDTH,
        LINK_BODY_HEIGHT,
    )
    end_pad = _box_segment(
        LINK_PITCH - HINGE_PAD_LENGTH,
        LINK_PITCH,
        LINK_WIDTH,
        HINGE_PAD_HEIGHT,
    )
    return start_pad.union(center_body).union(end_pad)


def _make_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT).translate(
        (-BASE_STANDOFF - PLATE_THICKNESS / 2.0, 0.0, 0.0)
    )
    bracket_body = _box_segment(
        -BASE_BRACKET_LENGTH,
        -BASE_HINGE_PAD_LENGTH,
        BASE_BRACKET_WIDTH,
        BASE_BRACKET_HEIGHT,
    )
    hinge_pad = _box_segment(
        -BASE_HINGE_PAD_LENGTH,
        0.0,
        LINK_WIDTH,
        HINGE_PAD_HEIGHT,
    )
    return plate.union(bracket_body).union(hinge_pad)


def _make_platform_bracket_shape() -> cq.Workplane:
    flange = _box_segment(0.0, PLATFORM_FLANGE_LENGTH, PLATFORM_WIDTH, BRACKET_FLANGE_HEIGHT)
    shelf = cq.Workplane("XY").box(
        PLATFORM_DEPTH,
        PLATFORM_WIDTH,
        PLATFORM_THICKNESS,
    ).translate(
        (
            PLATFORM_FLANGE_LENGTH + PLATFORM_DEPTH / 2.0,
            0.0,
            BRACKET_FLANGE_HEIGHT / 2.0 - PLATFORM_THICKNESS / 2.0,
        )
    )
    lip = cq.Workplane("XY").box(
        BRACKET_FLANGE_THICKNESS,
        PLATFORM_WIDTH,
        PLATFORM_LIP_HEIGHT,
    ).translate(
        (
            PLATFORM_FLANGE_LENGTH + PLATFORM_DEPTH,
            0.0,
            BRACKET_FLANGE_HEIGHT / 2.0 + PLATFORM_LIP_HEIGHT / 2.0 - PLATFORM_THICKNESS,
        )
    )
    gusset = cq.Workplane("XY").box(0.018, 0.006, 0.014).translate(
        (0.010, 0.0, BRACKET_FLANGE_HEIGHT / 2.0 - 0.009)
    )
    return flange.union(shelf).union(lip).union(gusset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_support_arm")

    base_color = model.material("base_plate_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    link_color = model.material("link_finish", rgba=(0.65, 0.67, 0.70, 1.0))
    bracket_color = model.material("bracket_finish", rgba=(0.74, 0.75, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_plate"),
        origin=Origin(),
        material=base_color,
        name="base_body",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_make_link_shape(), "link_1"),
        origin=Origin(),
        material=link_color,
        name="link_1_body",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_make_link_shape(), "link_2"),
        origin=Origin(),
        material=link_color,
        name="link_2_body",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_make_link_shape(), "link_3"),
        origin=Origin(),
        material=link_color,
        name="link_3_body",
    )

    link_4 = model.part("link_4")
    link_4.visual(
        mesh_from_cadquery(_make_link_shape(terminal=True), "link_4"),
        origin=Origin(),
        material=link_color,
        name="link_4_body",
    )

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        mesh_from_cadquery(_make_platform_bracket_shape(), "platform_bracket"),
        origin=Origin(),
        material=bracket_color,
        name="platform_bracket_body",
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=REVOLUTE_LIMITS,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=REVOLUTE_LIMITS,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=REVOLUTE_LIMITS,
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=REVOLUTE_LIMITS,
    )
    model.articulation(
        "link_4_to_platform_bracket",
        ArticulationType.FIXED,
        parent=link_4,
        child=platform_bracket,
        origin=Origin(
            xyz=(LINK_PITCH, 0.0, LINK_BODY_HEIGHT / 2.0 + BRACKET_FLANGE_HEIGHT / 2.0)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    platform_bracket = object_model.get_part("platform_bracket")

    revolute_joints = [
        object_model.get_articulation("base_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
        object_model.get_articulation("link_3_to_link_4"),
    ]
    platform_joint = object_model.get_articulation("link_4_to_platform_bracket")

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
    ctx.allow_overlap(
        base,
        link_1,
        reason="Simplified hinge knuckle envelopes intentionally share the first pivot volume.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        reason="Adjacent boxed links use simplified coaxial hinge-end solids that intentionally interpenetrate at the pivot envelope.",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        reason="Adjacent boxed links use simplified coaxial hinge-end solids that intentionally interpenetrate at the pivot envelope.",
    )
    ctx.allow_overlap(
        link_3,
        link_4,
        reason="Adjacent boxed links use simplified coaxial hinge-end solids that intentionally interpenetrate at the pivot envelope.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_revolute_axes_parallel_y",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in revolute_joints),
        details="Every folding joint should rotate about the shared world Y axis.",
    )
    ctx.check(
        "platform_joint_fixed",
        platform_joint.joint_type == ArticulationType.FIXED,
        details="The end platform bracket should be rigidly mounted to the last link.",
    )

    ctx.expect_origin_distance(base, link_1, axes="xz", max_dist=1e-6, name="base_link_1_origin_coincident")
    ctx.expect_origin_gap(
        link_2,
        link_1,
        axis="x",
        min_gap=LINK_PITCH - 1e-6,
        max_gap=LINK_PITCH + 1e-6,
        name="link_1_to_link_2_pitch",
    )
    ctx.expect_origin_gap(
        link_3,
        link_2,
        axis="x",
        min_gap=LINK_PITCH - 1e-6,
        max_gap=LINK_PITCH + 1e-6,
        name="link_2_to_link_3_pitch",
    )
    ctx.expect_origin_gap(
        link_4,
        link_3,
        axis="x",
        min_gap=LINK_PITCH - 1e-6,
        max_gap=LINK_PITCH + 1e-6,
        name="link_3_to_link_4_pitch",
    )
    ctx.expect_origin_gap(
        platform_bracket,
        link_4,
        axis="x",
        min_gap=LINK_PITCH - 1e-6,
        max_gap=LINK_PITCH + 1e-6,
        name="link_4_to_platform_mount_offset",
    )

    adjacent_pairs = [
        ("base_to_link_1_contact", base, link_1),
        ("link_1_to_link_2_contact", link_1, link_2),
        ("link_2_to_link_3_contact", link_2, link_3),
        ("link_3_to_link_4_contact", link_3, link_4),
        ("link_4_to_platform_contact", link_4, platform_bracket),
    ]
    for name, part_a, part_b in adjacent_pairs:
        ctx.expect_contact(part_a, part_b, name=name)

    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is not None:
        (base_min, base_max) = base_aabb
        ctx.check(
            "base_plate_envelopes_hinge",
            base_min[0] < -0.020 and base_max[2] > 0.040 and base_min[2] < -0.040,
            details="The fixed plate should sit behind the first hinge and be tall enough to read as a mounting plate.",
        )
    else:
        ctx.fail("base_plate_has_geometry", "Base plate AABB was unavailable.")

    for joint in revolute_joints:
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
                parent_part = object_model.get_part(joint.parent)
                child_part = object_model.get_part(joint.child)
                ctx.expect_contact(parent_part, child_part, name=f"{joint.name}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                parent_part = object_model.get_part(joint.parent)
                child_part = object_model.get_part(joint.child)
                ctx.expect_contact(parent_part, child_part, name=f"{joint.name}_upper_contact")

    folded_pose = {
        revolute_joints[0]: 1.05,
        revolute_joints[1]: -1.05,
        revolute_joints[2]: -1.05,
        revolute_joints[3]: 1.05,
    }
    with ctx.pose(folded_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        for name, part_a, part_b in adjacent_pairs:
            ctx.expect_contact(part_a, part_b, name=f"folded_{name}")

    base_pos = ctx.part_world_position(base)
    with ctx.pose({joint: 0.0 for joint in revolute_joints}):
        extended_pos = ctx.part_world_position(platform_bracket)
    with ctx.pose(folded_pose):
        folded_pos = ctx.part_world_position(platform_bracket)

    if base_pos is None or extended_pos is None or folded_pos is None:
        ctx.fail("reach_positions_available", "Could not evaluate one or more reach positions.")
    else:
        extended_reach = hypot(extended_pos[0] - base_pos[0], extended_pos[2] - base_pos[2])
        folded_reach = hypot(folded_pos[0] - base_pos[0], folded_pos[2] - base_pos[2])
        ctx.check(
            "arm_extends_beyond_folded_pack",
            extended_reach > folded_reach + 0.050,
            details=(
                f"Extended reach {extended_reach:.3f} m should exceed folded reach "
                f"{folded_reach:.3f} m by a clear margin."
            ),
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
