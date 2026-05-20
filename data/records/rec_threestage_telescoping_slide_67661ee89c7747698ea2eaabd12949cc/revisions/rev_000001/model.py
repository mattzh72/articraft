from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.340
MIDDLE_LENGTH = 0.255
INNER_LENGTH = 0.185

OUTER_WIDTH = 0.038
OUTER_HEIGHT = 0.016
OUTER_WALL = 0.0016
OUTER_FLOOR = 0.0016
OUTER_LIP_THICKNESS = 0.0012
OUTER_LIP_WIDTH = 0.0030

MIDDLE_BASE_WIDTH = 0.0230
MIDDLE_BASE_THICKNESS = 0.0014
MIDDLE_WALL = 0.0014
MIDDLE_SIDE_HEIGHT = 0.0081
MIDDLE_CAP_THICKNESS = 0.0010
MIDDLE_CAP_WIDTH = 0.0060
MIDDLE_HEIGHT = MIDDLE_BASE_THICKNESS + MIDDLE_SIDE_HEIGHT + MIDDLE_CAP_THICKNESS
MIDDLE_WIDTH = MIDDLE_BASE_WIDTH + 2.0 * (MIDDLE_CAP_WIDTH - MIDDLE_WALL) / 2.0 + 2.0 * MIDDLE_WALL

INNER_BASE_WIDTH = 0.0136
INNER_BASE_THICKNESS = 0.0012
INNER_WALL = 0.0012
INNER_SIDE_HEIGHT = 0.0044
INNER_CAP_THICKNESS = 0.0012
INNER_CAP_WIDTH = 0.0044
INNER_HEIGHT = INNER_BASE_THICKNESS + INNER_SIDE_HEIGHT + INNER_CAP_THICKNESS
INNER_WIDTH = INNER_BASE_WIDTH + 2.0 * (INNER_CAP_WIDTH - INNER_WALL) / 2.0 + 2.0 * INNER_WALL

OUTER_TRAVEL = 0.120
INNER_TRAVEL = 0.085
OUTER_TO_MIDDLE_Z = OUTER_HEIGHT - OUTER_LIP_THICKNESS - MIDDLE_HEIGHT
MIDDLE_TO_INNER_Z = MIDDLE_HEIGHT - MIDDLE_CAP_THICKNESS - INNER_HEIGHT


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_drawer_slide")

    model.material("outer_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("middle_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("inner_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    outer_channel = model.part("outer_channel")
    _add_box(
        outer_channel,
        size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_FLOOR),
        xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_FLOOR / 2.0),
        material="outer_steel",
        name="outer_floor",
    )
    outer_side_height = OUTER_HEIGHT - OUTER_FLOOR
    outer_side_z = OUTER_FLOOR + outer_side_height / 2.0
    _add_box(
        outer_channel,
        size=(OUTER_LENGTH, OUTER_WALL, outer_side_height),
        xyz=(OUTER_LENGTH / 2.0, OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0, outer_side_z),
        material="outer_steel",
        name="outer_left_web",
    )
    _add_box(
        outer_channel,
        size=(OUTER_LENGTH, OUTER_WALL, outer_side_height),
        xyz=(OUTER_LENGTH / 2.0, -OUTER_WIDTH / 2.0 + OUTER_WALL / 2.0, outer_side_z),
        material="outer_steel",
        name="outer_right_web",
    )
    outer_lip_y = OUTER_WIDTH / 2.0 - OUTER_WALL - OUTER_LIP_WIDTH / 2.0
    outer_lip_z = OUTER_HEIGHT - OUTER_LIP_THICKNESS / 2.0
    _add_box(
        outer_channel,
        size=(OUTER_LENGTH, OUTER_LIP_WIDTH, OUTER_LIP_THICKNESS),
        xyz=(OUTER_LENGTH / 2.0, outer_lip_y, outer_lip_z),
        material="outer_steel",
        name="outer_left_lip",
    )
    _add_box(
        outer_channel,
        size=(OUTER_LENGTH, OUTER_LIP_WIDTH, OUTER_LIP_THICKNESS),
        xyz=(OUTER_LENGTH / 2.0, -outer_lip_y, outer_lip_z),
        material="outer_steel",
        name="outer_right_lip",
    )

    middle_runner = model.part("middle_runner")
    _add_box(
        middle_runner,
        size=(MIDDLE_LENGTH, MIDDLE_BASE_WIDTH, MIDDLE_BASE_THICKNESS),
        xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_BASE_THICKNESS / 2.0),
        material="middle_steel",
        name="middle_base",
    )
    middle_wall_y = MIDDLE_BASE_WIDTH / 2.0 + MIDDLE_WALL / 2.0
    middle_wall_z = MIDDLE_BASE_THICKNESS + MIDDLE_SIDE_HEIGHT / 2.0
    _add_box(
        middle_runner,
        size=(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT),
        xyz=(MIDDLE_LENGTH / 2.0, middle_wall_y, middle_wall_z),
        material="middle_steel",
        name="middle_left_web",
    )
    _add_box(
        middle_runner,
        size=(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT),
        xyz=(MIDDLE_LENGTH / 2.0, -middle_wall_y, middle_wall_z),
        material="middle_steel",
        name="middle_right_web",
    )
    middle_cap_z = MIDDLE_HEIGHT - MIDDLE_CAP_THICKNESS / 2.0
    _add_box(
        middle_runner,
        size=(MIDDLE_LENGTH, MIDDLE_CAP_WIDTH, MIDDLE_CAP_THICKNESS),
        xyz=(MIDDLE_LENGTH / 2.0, middle_wall_y, middle_cap_z),
        material="middle_steel",
        name="middle_left_cap",
    )
    _add_box(
        middle_runner,
        size=(MIDDLE_LENGTH, MIDDLE_CAP_WIDTH, MIDDLE_CAP_THICKNESS),
        xyz=(MIDDLE_LENGTH / 2.0, -middle_wall_y, middle_cap_z),
        material="middle_steel",
        name="middle_right_cap",
    )

    inner_runner = model.part("inner_runner")
    _add_box(
        inner_runner,
        size=(INNER_LENGTH, INNER_BASE_WIDTH, INNER_BASE_THICKNESS),
        xyz=(INNER_LENGTH / 2.0, 0.0, INNER_BASE_THICKNESS / 2.0),
        material="inner_steel",
        name="inner_base",
    )
    inner_wall_y = INNER_BASE_WIDTH / 2.0 + INNER_WALL / 2.0
    inner_wall_z = INNER_BASE_THICKNESS + INNER_SIDE_HEIGHT / 2.0
    _add_box(
        inner_runner,
        size=(INNER_LENGTH, INNER_WALL, INNER_SIDE_HEIGHT),
        xyz=(INNER_LENGTH / 2.0, inner_wall_y, inner_wall_z),
        material="inner_steel",
        name="inner_left_web",
    )
    _add_box(
        inner_runner,
        size=(INNER_LENGTH, INNER_WALL, INNER_SIDE_HEIGHT),
        xyz=(INNER_LENGTH / 2.0, -inner_wall_y, inner_wall_z),
        material="inner_steel",
        name="inner_right_web",
    )
    inner_cap_z = INNER_HEIGHT - INNER_CAP_THICKNESS / 2.0
    _add_box(
        inner_runner,
        size=(INNER_LENGTH, INNER_CAP_WIDTH, INNER_CAP_THICKNESS),
        xyz=(INNER_LENGTH / 2.0, inner_wall_y, inner_cap_z),
        material="inner_steel",
        name="inner_left_cap",
    )
    _add_box(
        inner_runner,
        size=(INNER_LENGTH, INNER_CAP_WIDTH, INNER_CAP_THICKNESS),
        xyz=(INNER_LENGTH / 2.0, -inner_wall_y, inner_cap_z),
        material="inner_steel",
        name="inner_right_cap",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=middle_runner,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
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

    part_names = {part.name for part in object_model.parts}
    ctx.check(
        "slide_parts_present",
        {"outer_channel", "middle_runner", "inner_runner"}.issubset(part_names),
        f"found parts: {sorted(part_names)}",
    )

    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "slide_joints_present",
        {"outer_to_middle", "middle_to_inner"}.issubset(joint_names),
        f"found joints: {sorted(joint_names)}",
    )

    ctx.check(
        "prismatic_axes_point_forward",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        (
            f"outer_to_middle={outer_to_middle.articulation_type}/{outer_to_middle.axis}, "
            f"middle_to_inner={middle_to_inner.articulation_type}/{middle_to_inner.axis}"
        ),
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(
            middle,
            outer,
            contact_tol=1e-6,
            name="middle_runner_retained_by_outer_channel",
        )
        ctx.expect_contact(
            inner,
            middle,
            contact_tol=1e-6,
            name="inner_runner_retained_by_middle_runner",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0008,
            name="middle_runner_nested_inside_outer_channel",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0008,
            name="inner_runner_nested_inside_middle_runner",
        )
        ctx.expect_overlap(
            outer,
            middle,
            axes="x",
            min_overlap=0.240,
            name="outer_and_middle_have_closed_overlap",
        )
        ctx.expect_overlap(
            middle,
            inner,
            axes="x",
            min_overlap=0.175,
            name="middle_and_inner_have_closed_overlap",
        )

    with ctx.pose({outer_to_middle: OUTER_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_interference_at_full_extension")
        ctx.expect_overlap(
            outer,
            middle,
            axes="x",
            min_overlap=0.200,
            name="outer_and_middle_retain_visible_overlap",
        )
        ctx.expect_overlap(
            middle,
            inner,
            axes="x",
            min_overlap=0.160,
            name="middle_and_inner_retain_visible_overlap",
        )
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=OUTER_TRAVEL - 0.002,
            max_gap=OUTER_TRAVEL + 0.002,
            name="middle_runner_extends_forward_on_outer_joint",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=INNER_TRAVEL - 0.002,
            max_gap=INNER_TRAVEL + 0.002,
            name="inner_runner_extends_forward_on_middle_joint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
