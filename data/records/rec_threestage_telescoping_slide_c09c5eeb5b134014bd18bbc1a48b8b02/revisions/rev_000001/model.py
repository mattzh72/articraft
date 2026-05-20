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


SUPPORT_LENGTH = 0.54
SUPPORT_WIDTH = 0.064
SUPPORT_TOP_THICKNESS = 0.004
SUPPORT_TOP_Z = 0.0076
SUPPORT_SADDLE_LENGTH = 0.40
SUPPORT_SADDLE_X = 0.04
SUPPORT_SADDLE_WIDTH = 0.016
SUPPORT_SADDLE_THICKNESS = 0.002
SUPPORT_WEB_WIDTH = 0.0035
SUPPORT_WEB_Y = 0.017
SUPPORT_BRIDGE_LENGTH = 0.018
SUPPORT_BRIDGE_WIDTH = 0.039

OUTER_LENGTH = 0.46
OUTER_WIDTH = 0.048
OUTER_ROOF_THICKNESS = 0.003
OUTER_DEPTH = 0.020
OUTER_WALL_THICKNESS = 0.0024
OUTER_LIP_WIDTH = 0.0085
OUTER_LIP_THICKNESS = 0.0024

MIDDLE_LENGTH = 0.34
MIDDLE_UPPER_WIDTH = 0.033
MIDDLE_UPPER_THICKNESS = 0.0022
MIDDLE_LOWER_WIDTH = 0.030
MIDDLE_LOWER_ROOF_TOP = -0.0144
MIDDLE_LOWER_ROOF_THICKNESS = 0.0024
MIDDLE_LOWER_BOTTOM = -0.0306
MIDDLE_LOWER_WALL_THICKNESS = 0.0022
MIDDLE_LOWER_LIP_WIDTH = 0.0057
MIDDLE_LOWER_LIP_THICKNESS = 0.0024
MIDDLE_NECK_WIDTH = 0.018

INNER_LENGTH = 0.24
INNER_UPPER_WIDTH = 0.021
INNER_UPPER_THICKNESS = 0.0022
INNER_NECK_WIDTH = 0.0115
INNER_BODY_WIDTH = 0.018
INNER_BODY_TOP = -0.0145
INNER_BODY_BOTTOM = -0.0245
INNER_MOUNT_LENGTH = 0.07
INNER_MOUNT_WIDTH = 0.036
INNER_MOUNT_BOTTOM = -0.028
INNER_MOUNT_THICKNESS = 0.0045

OUTER_TO_MIDDLE_HOME = 0.065
OUTER_TO_MIDDLE_Z = -0.0154
OUTER_TO_MIDDLE_TRAVEL = 0.19

MIDDLE_TO_INNER_HOME = 0.05
MIDDLE_TO_INNER_Z = -0.026
MIDDLE_TO_INNER_TRAVEL = 0.17

HOLE_RADIUS = 0.0034
HOLE_XS = (0.075, 0.175, 0.365, 0.465)
FUSE_EPS = 0.0004


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x, y, z))
    )


def _cylinder(radius: float, height: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z))


def _support_shape() -> cq.Workplane:
    plate = _box(
        SUPPORT_LENGTH,
        SUPPORT_WIDTH,
        SUPPORT_TOP_THICKNESS,
        z=SUPPORT_TOP_Z,
    )
    for hole_x in HOLE_XS:
        plate = plate.cut(
            _cylinder(
                HOLE_RADIUS,
                SUPPORT_TOP_THICKNESS + 0.002,
                x=hole_x,
                y=0.0,
                z=SUPPORT_TOP_Z - 0.001,
            )
        )

    saddle = _box(
        SUPPORT_SADDLE_LENGTH,
        SUPPORT_SADDLE_WIDTH,
        SUPPORT_SADDLE_THICKNESS,
        x=SUPPORT_SADDLE_X,
        z=0.0,
    )
    web_left = _box(
        SUPPORT_SADDLE_LENGTH,
        SUPPORT_WEB_WIDTH,
        SUPPORT_TOP_Z + FUSE_EPS,
        x=SUPPORT_SADDLE_X,
        y=SUPPORT_WEB_Y,
        z=0.0,
    )
    web_right = _box(
        SUPPORT_SADDLE_LENGTH,
        SUPPORT_WEB_WIDTH,
        SUPPORT_TOP_Z + FUSE_EPS,
        x=SUPPORT_SADDLE_X,
        y=-SUPPORT_WEB_Y,
        z=0.0,
    )
    bridge_front = _box(
        SUPPORT_BRIDGE_LENGTH,
        SUPPORT_BRIDGE_WIDTH,
        SUPPORT_SADDLE_THICKNESS,
        x=SUPPORT_SADDLE_X + 0.028,
        z=0.0,
    )
    bridge_rear = _box(
        SUPPORT_BRIDGE_LENGTH,
        SUPPORT_BRIDGE_WIDTH,
        SUPPORT_SADDLE_THICKNESS,
        x=SUPPORT_SADDLE_X + SUPPORT_SADDLE_LENGTH - 0.028 - SUPPORT_BRIDGE_LENGTH,
        z=0.0,
    )

    return (
        plate.union(saddle)
        .union(web_left)
        .union(web_right)
        .union(bridge_front)
        .union(bridge_rear)
    )


def _outer_sleeve_shape() -> cq.Workplane:
    roof = _box(
        OUTER_LENGTH,
        OUTER_WIDTH,
        OUTER_ROOF_THICKNESS,
        z=-OUTER_ROOF_THICKNESS,
    )
    wall_height = OUTER_DEPTH - OUTER_ROOF_THICKNESS + FUSE_EPS
    wall_z = -OUTER_DEPTH
    wall_y = OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS / 2.0
    left_wall = _box(
        OUTER_LENGTH,
        OUTER_WALL_THICKNESS,
        wall_height,
        y=wall_y,
        z=wall_z,
    )
    right_wall = _box(
        OUTER_LENGTH,
        OUTER_WALL_THICKNESS,
        wall_height,
        y=-wall_y,
        z=wall_z,
    )
    lip_y = OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS - OUTER_LIP_WIDTH / 2.0
    left_lip = _box(
        OUTER_LENGTH,
        OUTER_LIP_WIDTH + FUSE_EPS,
        OUTER_LIP_THICKNESS,
        y=lip_y,
        z=-OUTER_DEPTH,
    )
    right_lip = _box(
        OUTER_LENGTH,
        OUTER_LIP_WIDTH + FUSE_EPS,
        OUTER_LIP_THICKNESS,
        y=-lip_y,
        z=-OUTER_DEPTH,
    )

    return roof.union(left_wall).union(right_wall).union(left_lip).union(right_lip)


def _middle_runner_shape() -> cq.Workplane:
    upper_flange = _box(
        MIDDLE_LENGTH,
        MIDDLE_UPPER_WIDTH,
        MIDDLE_UPPER_THICKNESS,
        z=-MIDDLE_UPPER_THICKNESS,
    )
    neck = _box(
        MIDDLE_LENGTH,
        MIDDLE_NECK_WIDTH,
        abs(MIDDLE_LOWER_ROOF_TOP) - MIDDLE_UPPER_THICKNESS + FUSE_EPS,
        z=MIDDLE_LOWER_ROOF_TOP,
    )
    lower_roof = _box(
        MIDDLE_LENGTH,
        MIDDLE_LOWER_WIDTH,
        MIDDLE_LOWER_ROOF_THICKNESS,
        z=MIDDLE_LOWER_ROOF_TOP - MIDDLE_LOWER_ROOF_THICKNESS,
    )
    lower_wall_height = (
        (MIDDLE_LOWER_ROOF_TOP - MIDDLE_LOWER_ROOF_THICKNESS) - MIDDLE_LOWER_BOTTOM
        + FUSE_EPS
    )
    lower_wall_y = MIDDLE_LOWER_WIDTH / 2.0 - MIDDLE_LOWER_WALL_THICKNESS / 2.0
    left_wall = _box(
        MIDDLE_LENGTH,
        MIDDLE_LOWER_WALL_THICKNESS,
        lower_wall_height,
        y=lower_wall_y,
        z=MIDDLE_LOWER_BOTTOM,
    )
    right_wall = _box(
        MIDDLE_LENGTH,
        MIDDLE_LOWER_WALL_THICKNESS,
        lower_wall_height,
        y=-lower_wall_y,
        z=MIDDLE_LOWER_BOTTOM,
    )
    lip_y = (
        MIDDLE_LOWER_WIDTH / 2.0
        - MIDDLE_LOWER_WALL_THICKNESS
        - MIDDLE_LOWER_LIP_WIDTH / 2.0
    )
    left_lip = _box(
        MIDDLE_LENGTH,
        MIDDLE_LOWER_LIP_WIDTH + FUSE_EPS,
        MIDDLE_LOWER_LIP_THICKNESS,
        y=lip_y,
        z=MIDDLE_LOWER_BOTTOM,
    )
    right_lip = _box(
        MIDDLE_LENGTH,
        MIDDLE_LOWER_LIP_WIDTH + FUSE_EPS,
        MIDDLE_LOWER_LIP_THICKNESS,
        y=-lip_y,
        z=MIDDLE_LOWER_BOTTOM,
    )

    return (
        upper_flange
        .union(neck)
        .union(lower_roof)
        .union(left_wall)
        .union(right_wall)
        .union(left_lip)
        .union(right_lip)
    )


def _inner_runner_shape() -> cq.Workplane:
    upper_flange = _box(
        INNER_LENGTH,
        INNER_UPPER_WIDTH,
        INNER_UPPER_THICKNESS,
        z=-INNER_UPPER_THICKNESS,
    )
    neck = _box(
        INNER_LENGTH,
        INNER_NECK_WIDTH,
        abs(INNER_BODY_TOP) - INNER_UPPER_THICKNESS + FUSE_EPS,
        z=INNER_BODY_TOP,
    )
    lower_body = _box(
        INNER_LENGTH,
        INNER_BODY_WIDTH,
        INNER_BODY_TOP - INNER_BODY_BOTTOM,
        z=INNER_BODY_BOTTOM,
    )
    mount_pad = _box(
        INNER_MOUNT_LENGTH,
        INNER_MOUNT_WIDTH,
        INNER_MOUNT_THICKNESS,
        x=INNER_LENGTH - INNER_MOUNT_LENGTH,
        z=INNER_MOUNT_BOTTOM,
    )

    return upper_flange.union(neck).union(lower_body).union(mount_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_three_stage_runner")

    model.material("support_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("outer_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("middle_steel", rgba=(0.54, 0.56, 0.59, 1.0))
    model.material("inner_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    support = model.part("top_support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "top_support"),
        material="support_aluminum",
        name="support_body",
    )

    outer = model.part("outer_sleeve")
    outer.visual(
        mesh_from_cadquery(_outer_sleeve_shape(), "outer_sleeve"),
        material="outer_steel",
        name="outer_body",
    )

    middle = model.part("middle_runner")
    middle.visual(
        mesh_from_cadquery(_middle_runner_shape(), "middle_runner"),
        material="middle_steel",
        name="middle_body",
    )

    inner = model.part("inner_runner")
    inner.visual(
        mesh_from_cadquery(_inner_runner_shape(), "inner_runner"),
        material="inner_steel",
        name="inner_body",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support,
        child=outer,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=160.0,
            velocity=0.55,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=120.0,
            velocity=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    outer = object_model.get_part("outer_sleeve")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    support_to_outer = object_model.get_articulation("support_to_outer")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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

    ctx.check(
        "support_and_stage_joints_present",
        (
            support_to_outer.articulation_type == ArticulationType.FIXED
            and outer_to_middle.articulation_type == ArticulationType.PRISMATIC
            and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        ),
        details="Expected one fixed support joint and two serial prismatic runner joints.",
    )
    ctx.check(
        "prismatic_axes_point_along_runner_length",
        outer_to_middle.axis == (1.0, 0.0, 0.0)
        and middle_to_inner.axis == (1.0, 0.0, 0.0)
        and outer_to_middle.motion_limits is not None
        and middle_to_inner.motion_limits is not None
        and outer_to_middle.motion_limits.lower == 0.0
        and middle_to_inner.motion_limits.lower == 0.0
        and outer_to_middle.motion_limits.upper == OUTER_TO_MIDDLE_TRAVEL
        and middle_to_inner.motion_limits.upper == MIDDLE_TO_INNER_TRAVEL,
        details="Runner stages should telescope in +X from a closed pose at q=0.",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(support, outer, name="top_support_carries_outer_sleeve")
        ctx.expect_contact(outer, middle, name="outer_sleeve_supports_middle_runner_closed")
        ctx.expect_contact(middle, inner, name="middle_runner_supports_inner_runner_closed")

        outer_pos = ctx.part_world_position(outer)
        middle_pos = ctx.part_world_position(middle)
        inner_pos = ctx.part_world_position(inner)
        ctx.check(
            "stages_hang_progressively_lower",
            (
                outer_pos is not None
                and middle_pos is not None
                and inner_pos is not None
                and middle_pos[2] < outer_pos[2] - 0.010
                and inner_pos[2] < middle_pos[2] - 0.010
            ),
            details="The middle runner should hang below the outer sleeve, and the inner below the middle.",
        )

        middle_closed_x = None if middle_pos is None else middle_pos[0]
        inner_closed_x = None if inner_pos is None else inner_pos[0]

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL, middle_to_inner: 0.0}):
        middle_extended = ctx.part_world_position(middle)
        ctx.check(
            "middle_runner_extends_forward",
            (
                middle_extended is not None
                and middle_closed_x is not None
                and middle_extended[0] > middle_closed_x + 0.15
            ),
            details="The middle stage did not translate forward along +X at its upper travel limit.",
        )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        inner_extended = ctx.part_world_position(inner)
        ctx.check(
            "inner_runner_extends_forward",
            (
                inner_extended is not None
                and inner_closed_x is not None
                and inner_extended[0] > inner_closed_x + 0.14
            ),
            details="The inner stage did not translate forward relative to the middle stage.",
        )
        ctx.expect_contact(outer, middle, name="outer_sleeve_still_carries_middle_at_full_extension")
        ctx.expect_contact(middle, inner, name="middle_runner_still_carries_inner_at_full_extension")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlaps_at_full_extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
