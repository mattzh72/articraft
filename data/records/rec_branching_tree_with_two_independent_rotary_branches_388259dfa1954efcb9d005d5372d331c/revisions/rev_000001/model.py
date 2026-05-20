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


SPINE_WIDTH = 0.085
SPINE_DEPTH = 0.065
SPINE_HALF_WIDTH = SPINE_WIDTH / 2.0
BRACKET_PIVOT_X = SPINE_HALF_WIDTH + 0.029
LOWER_BRANCH_LENGTH = 0.285
UPPER_BRANCH_LENGTH = 0.215
BARREL_LENGTH = 0.036


def _y_cylinder_origin(center: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0))


def _add_spine_visuals(model: ArticulatedObject):
    spine = model.part("spine")
    spine.visual(
        Box((0.190, 0.150, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="powder_steel",
        name="base_plate",
    )
    spine.visual(
        Box((0.112, 0.094, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material="powder_steel",
        name="plinth",
    )
    spine.visual(
        Box((SPINE_WIDTH, SPINE_DEPTH, 0.446)),
        origin=Origin(xyz=(0.0, 0.0, 0.257)),
        material="powder_steel",
        name="column",
    )
    spine.visual(
        Box((0.112, 0.080, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.507)),
        material="powder_steel",
        name="top_box",
    )
    spine.visual(
        Box((0.050, 0.004, 0.160)),
        origin=Origin(xyz=(0.0, 0.034, 0.310)),
        material="bracket_black",
        name="front_panel",
    )
    return spine


def _add_bracket_visuals(model: ArticulatedObject, name: str, side_sign: float):
    bracket = model.part(name)
    ear_thickness = 0.008
    ear_center_y = (BARREL_LENGTH + ear_thickness) / 2.0

    bracket.visual(
        Box((0.014, 0.062, 0.092)),
        origin=Origin(xyz=(side_sign * 0.022, 0.0, 0.0)),
        material="bracket_black",
        name="back_plate",
    )
    bracket.visual(
        Box((0.024, 0.046, 0.012)),
        origin=Origin(xyz=(side_sign * 0.016, 0.0, 0.025)),
        material="bracket_black",
        name="upper_gusset",
    )
    bracket.visual(
        Box((0.024, 0.046, 0.012)),
        origin=Origin(xyz=(side_sign * 0.016, 0.0, -0.025)),
        material="bracket_black",
        name="lower_gusset",
    )
    bracket.visual(
        Box((0.028, ear_thickness, 0.060)),
        origin=Origin(xyz=(side_sign * 0.003, ear_center_y, 0.0)),
        material="bracket_black",
        name="upper_ear",
    )
    bracket.visual(
        Box((0.028, ear_thickness, 0.060)),
        origin=Origin(xyz=(side_sign * 0.003, -ear_center_y, 0.0)),
        material="bracket_black",
        name="lower_ear",
    )
    bracket.visual(
        Cylinder(radius=0.017, length=ear_thickness),
        origin=_y_cylinder_origin((0.0, ear_center_y, 0.0)),
        material="bracket_black",
        name="upper_boss",
    )
    bracket.visual(
        Cylinder(radius=0.017, length=ear_thickness),
        origin=_y_cylinder_origin((0.0, -ear_center_y, 0.0)),
        material="bracket_black",
        name="lower_boss",
    )
    return bracket


def _add_branch_visuals(
    model: ArticulatedObject,
    name: str,
    *,
    beam_length: float,
    beam_width: float,
    beam_height: float,
    head_length: float,
    head_width: float,
    head_height: float,
    head_center_x: float,
    head_center_z: float,
    barrel_radius: float,
):
    branch = model.part(name)
    beam_start_x = 0.032
    beam_end_x = beam_start_x + beam_length
    head_start_x = head_center_x - (head_length / 2.0)
    neck_overlap = 0.003
    neck_start_x = beam_end_x - neck_overlap
    neck_end_x = head_start_x + neck_overlap
    neck_length = max(0.016, neck_end_x - neck_start_x)
    neck_center_x = (neck_start_x + neck_end_x) / 2.0

    branch.visual(
        Cylinder(radius=barrel_radius, length=BARREL_LENGTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material="arm_aluminum",
        name="pivot_barrel",
    )
    branch.visual(
        Box((0.026, beam_width * 0.84, beam_height * 1.22)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material="arm_aluminum",
        name="root_collar",
    )
    branch.visual(
        Box((beam_length, beam_width, beam_height)),
        origin=Origin(xyz=(beam_start_x + beam_length / 2.0, 0.0, 0.0)),
        material="arm_aluminum",
        name="beam",
    )
    branch.visual(
        Box((neck_length, beam_width * 0.82, beam_height * 1.05)),
        origin=Origin(xyz=(neck_center_x, 0.0, 0.0)),
        material="arm_aluminum",
        name="neck_block",
    )
    branch.visual(
        Box((head_length, head_width, head_height)),
        origin=Origin(xyz=(head_center_x, 0.0, head_center_z)),
        material="arm_aluminum",
        name="head",
    )
    branch.visual(
        Box((0.016, head_width * 0.72, head_height * 0.34)),
        origin=Origin(xyz=(head_center_x + head_length / 2.0 + 0.007, 0.0, head_center_z)),
        material="arm_aluminum",
        name="nose_cap",
    )
    branch.visual(
        Box((0.026, head_width * 0.76, 0.010)),
        origin=Origin(xyz=(head_center_x + 0.010, 0.0, head_center_z - head_height * 0.36)),
        material="arm_aluminum",
        name="chin_plate",
    )
    branch.visual(
        Box((0.020, head_width * 0.90, 0.008)),
        origin=Origin(xyz=(head_center_x + 0.004, 0.0, head_center_z + head_height * 0.38)),
        material="arm_aluminum",
        name="crown",
    )
    return branch


def _aabb_size(aabb):
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def _aabb_max_delta(aabb_a, aabb_b):
    return max(abs(aabb_a[j][i] - aabb_b[j][i]) for j in range(2) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_tree")

    model.material("powder_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("bracket_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("arm_aluminum", rgba=(0.67, 0.70, 0.74, 1.0))

    spine = _add_spine_visuals(model)
    lower_bracket = _add_bracket_visuals(model, "lower_bracket", 1.0)
    upper_bracket = _add_bracket_visuals(model, "upper_bracket", -1.0)
    lower_branch = _add_branch_visuals(
        model,
        "lower_branch",
        beam_length=0.176,
        beam_width=0.024,
        beam_height=0.022,
        head_length=0.062,
        head_width=0.042,
        head_height=0.050,
        head_center_x=LOWER_BRANCH_LENGTH - 0.031,
        head_center_z=0.004,
        barrel_radius=0.014,
    )
    upper_branch = _add_branch_visuals(
        model,
        "upper_branch",
        beam_length=0.128,
        beam_width=0.020,
        beam_height=0.018,
        head_length=0.050,
        head_width=0.036,
        head_height=0.042,
        head_center_x=UPPER_BRANCH_LENGTH - 0.025,
        head_center_z=0.008,
        barrel_radius=0.013,
    )

    model.articulation(
        "spine_to_lower_bracket",
        ArticulationType.FIXED,
        parent=spine,
        child=lower_bracket,
        origin=Origin(xyz=(-BRACKET_PIVOT_X, 0.0, 0.258)),
    )
    model.articulation(
        "spine_to_upper_bracket",
        ArticulationType.FIXED,
        parent=spine,
        child=upper_bracket,
        origin=Origin(xyz=(BRACKET_PIVOT_X, 0.0, 0.408)),
    )
    model.articulation(
        "lower_branch_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_bracket,
        child=lower_branch,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, pi)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.45, upper=1.05),
    )
    model.articulation(
        "upper_branch_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_bracket,
        child=upper_branch,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.5, lower=-0.30, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    lower_bracket = object_model.get_part("lower_bracket")
    upper_bracket = object_model.get_part("upper_bracket")
    lower_branch = object_model.get_part("lower_branch")
    upper_branch = object_model.get_part("upper_branch")
    lower_joint = object_model.get_articulation("lower_branch_pitch")
    upper_joint = object_model.get_articulation("upper_branch_pitch")

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

    ctx.expect_contact(lower_bracket, spine, name="lower bracket is mounted to spine")
    ctx.expect_contact(upper_bracket, spine, name="upper bracket is mounted to spine")
    ctx.expect_contact(lower_branch, lower_bracket, name="lower branch is supported by its bracket")
    ctx.expect_contact(upper_branch, upper_bracket, name="upper branch is supported by its bracket")

    lower_bracket_pos = ctx.part_world_position(lower_bracket)
    upper_bracket_pos = ctx.part_world_position(upper_bracket)
    ctx.check(
        "branch brackets are staggered in height",
        abs(upper_bracket_pos[2] - lower_bracket_pos[2]) >= 0.12,
        details=f"lower_z={lower_bracket_pos[2]:.4f}, upper_z={upper_bracket_pos[2]:.4f}",
    )

    ctx.check(
        "lower branch joint uses upward pitching revolute axis",
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_joint.axis) == (0.0, -1.0, 0.0)
        and lower_joint.motion_limits is not None
        and lower_joint.motion_limits.lower is not None
        and lower_joint.motion_limits.upper is not None
        and lower_joint.motion_limits.lower < 0.0 < lower_joint.motion_limits.upper,
        details=f"axis={lower_joint.axis}, limits={lower_joint.motion_limits}",
    )
    ctx.check(
        "upper branch joint uses upward pitching revolute axis",
        upper_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(upper_joint.axis) == (0.0, -1.0, 0.0)
        and upper_joint.motion_limits is not None
        and upper_joint.motion_limits.lower is not None
        and upper_joint.motion_limits.upper is not None
        and upper_joint.motion_limits.lower < 0.0 < upper_joint.motion_limits.upper,
        details=f"axis={upper_joint.axis}, limits={upper_joint.motion_limits}",
    )

    with ctx.pose({lower_joint: 0.0, upper_joint: 0.0}):
        lower_branch_closed = ctx.part_world_aabb(lower_branch)
        upper_branch_closed = ctx.part_world_aabb(upper_branch)

    lower_branch_x = _aabb_size(lower_branch_closed)[0]
    upper_branch_x = _aabb_size(upper_branch_closed)[0]
    ctx.check(
        "branch lengths are intentionally varied",
        abs(lower_branch_x - upper_branch_x) >= 0.045,
        details=f"lower_dx={lower_branch_x:.4f}, upper_dx={upper_branch_x:.4f}",
    )

    with ctx.pose({lower_joint: 0.82, upper_joint: 0.0}):
        lower_branch_open = ctx.part_world_aabb(lower_branch)
        upper_branch_while_lower_moves = ctx.part_world_aabb(upper_branch)

    ctx.check(
        "lower branch pitches upward when actuated",
        lower_branch_open[1][2] > lower_branch_closed[1][2] + 0.05,
        details=(
            f"closed_max_z={lower_branch_closed[1][2]:.4f}, "
            f"open_max_z={lower_branch_open[1][2]:.4f}"
        ),
    )
    ctx.check(
        "upper branch remains independent when lower branch moves",
        _aabb_max_delta(upper_branch_closed, upper_branch_while_lower_moves) <= 1e-6,
        details=(
            f"delta={_aabb_max_delta(upper_branch_closed, upper_branch_while_lower_moves):.8f}"
        ),
    )

    with ctx.pose({lower_joint: 0.0, upper_joint: 0.70}):
        upper_branch_open = ctx.part_world_aabb(upper_branch)
        lower_branch_while_upper_moves = ctx.part_world_aabb(lower_branch)

    ctx.check(
        "upper branch pitches upward when actuated",
        upper_branch_open[1][2] > upper_branch_closed[1][2] + 0.04,
        details=(
            f"closed_max_z={upper_branch_closed[1][2]:.4f}, "
            f"open_max_z={upper_branch_open[1][2]:.4f}"
        ),
    )
    ctx.check(
        "lower branch remains independent when upper branch moves",
        _aabb_max_delta(lower_branch_closed, lower_branch_while_upper_moves) <= 1e-6,
        details=(
            f"delta={_aabb_max_delta(lower_branch_closed, lower_branch_while_upper_moves):.8f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
