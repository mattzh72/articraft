from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.320
OUTER_WIDTH = 0.084
OUTER_HEIGHT = 0.060
OUTER_WALL = 0.004
OUTER_FLOOR = 0.004
OUTER_BACK_WALL = 0.018

MIDDLE_LENGTH = 0.235
MIDDLE_WIDTH = 0.072
MIDDLE_HEIGHT = 0.046
MIDDLE_WALL = 0.003
MIDDLE_FLOOR = 0.003

INNER_LENGTH = 0.165
INNER_WIDTH = 0.056
INNER_HEIGHT = 0.035
INNER_WALL = 0.0025
INNER_FLOOR = 0.0025

TERMINAL_LENGTH = 0.092
TERMINAL_WIDTH = 0.045
TERMINAL_HEIGHT = 0.018

OUTER_TO_MIDDLE_INSERT = 0.026
MIDDLE_TO_INNER_INSERT = 0.019
INNER_TO_TERMINAL_INSERT = 0.016

OUTER_TO_MIDDLE_TRAVEL = 0.105
MIDDLE_TO_INNER_TRAVEL = 0.085
INNER_TO_TERMINAL_TRAVEL = 0.070
def _add_box_visual(
    part: object,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_channel_part(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    floor: float,
    material: str,
    box_size: tuple[float, float, float],
    mass: float,
    back_wall: float = 0.0,
    front_stop: float = 0.0,
) -> object:
    part = model.part(name)
    side_wall_height = height - floor

    _add_box_visual(
        part,
        size=(length, width, floor),
        center=(length / 2.0, 0.0, floor / 2.0),
        material=material,
        name=f"{name}_floor",
    )
    _add_box_visual(
        part,
        size=(length, wall, side_wall_height),
        center=(length / 2.0, (width / 2.0) - (wall / 2.0), floor + side_wall_height / 2.0),
        material=material,
        name=f"{name}_left_wall",
    )
    _add_box_visual(
        part,
        size=(length, wall, side_wall_height),
        center=(length / 2.0, -(width / 2.0) + (wall / 2.0), floor + side_wall_height / 2.0),
        material=material,
        name=f"{name}_right_wall",
    )

    if back_wall > 0.0:
        _add_box_visual(
            part,
            size=(back_wall, width - 2.0 * wall, side_wall_height),
            center=(back_wall / 2.0, 0.0, floor + side_wall_height / 2.0),
            material=material,
            name=f"{name}_back_wall",
        )

    if front_stop > 0.0:
        _add_box_visual(
            part,
            size=(front_stop, width - 2.0 * wall, floor + side_wall_height * 0.28),
            center=(
                length - front_stop / 2.0,
                0.0,
                (floor + side_wall_height * 0.28) / 2.0,
            ),
            material=material,
            name=f"{name}_front_stop",
        )

    part.inertial = Inertial.from_geometry(
        Box(box_size),
        mass=mass,
        origin=Origin(xyz=(box_size[0] / 2.0, 0.0, box_size[2] / 2.0)),
    )
    return part


def _is_prismatic_axis_x(joint_obj: object) -> bool:
    axis = getattr(joint_obj, "axis", None)
    if axis is None or len(axis) != 3:
        return False
    return (
        math.isclose(axis[0], 1.0, abs_tol=1e-9)
        and math.isclose(axis[1], 0.0, abs_tol=1e-9)
        and math.isclose(axis[2], 0.0, abs_tol=1e-9)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_linear_stack")

    model.material("outer_body_finish", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("middle_carriage_finish", rgba=(0.54, 0.57, 0.60, 1.0))
    model.material("inner_carriage_finish", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("terminal_slide_finish", rgba=(0.18, 0.19, 0.21, 1.0))

    outer_body = _add_channel_part(
        model,
        name="outer_body",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        floor=OUTER_FLOOR,
        material="outer_body_finish",
        box_size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT),
        mass=1.15,
        back_wall=OUTER_BACK_WALL,
        front_stop=0.016,
    )
    middle_carriage = _add_channel_part(
        model,
        name="middle_carriage",
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        floor=MIDDLE_FLOOR,
        material="middle_carriage_finish",
        box_size=(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT),
        mass=0.58,
        front_stop=0.014,
    )
    inner_carriage = _add_channel_part(
        model,
        name="inner_carriage",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        floor=INNER_FLOOR,
        material="inner_carriage_finish",
        box_size=(INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT),
        mass=0.33,
        front_stop=0.012,
    )
    terminal_slide = model.part("terminal_slide")
    _add_box_visual(
        terminal_slide,
        size=(TERMINAL_LENGTH, TERMINAL_WIDTH, TERMINAL_HEIGHT),
        center=(TERMINAL_LENGTH / 2.0, 0.0, TERMINAL_HEIGHT / 2.0),
        material="terminal_slide_finish",
        name="terminal_slide_body",
    )
    _add_box_visual(
        terminal_slide,
        size=(0.020, TERMINAL_WIDTH * 0.86, 0.010),
        center=(TERMINAL_LENGTH - 0.010, 0.0, TERMINAL_HEIGHT + 0.005),
        material="terminal_slide_finish",
        name="terminal_slide_head",
    )
    terminal_slide.inertial = Inertial.from_geometry(
        Box((TERMINAL_LENGTH, TERMINAL_WIDTH, TERMINAL_HEIGHT + 0.010)),
        mass=0.16,
        origin=Origin(xyz=(TERMINAL_LENGTH / 2.0, 0.0, (TERMINAL_HEIGHT + 0.010) / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=middle_carriage,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_INSERT, 0.0, OUTER_FLOOR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.40,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_carriage,
        child=inner_carriage,
        origin=Origin(xyz=(MIDDLE_TO_INNER_INSERT, 0.0, MIDDLE_FLOOR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_terminal",
        ArticulationType.PRISMATIC,
        parent=inner_carriage,
        child=terminal_slide,
        origin=Origin(xyz=(INNER_TO_TERMINAL_INSERT, 0.0, INNER_FLOOR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.55,
            lower=0.0,
            upper=INNER_TO_TERMINAL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    middle_carriage = object_model.get_part("middle_carriage")
    inner_carriage = object_model.get_part("inner_carriage")
    terminal_slide = object_model.get_part("terminal_slide")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_terminal = object_model.get_articulation("inner_to_terminal")

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

    for joint_obj, min_upper in (
        (outer_to_middle, 0.09),
        (middle_to_inner, 0.07),
        (inner_to_terminal, 0.05),
    ):
        limits = joint_obj.motion_limits
        ctx.check(
            f"{joint_obj.name}_configuration",
            (
                joint_obj.articulation_type == ArticulationType.PRISMATIC
                and _is_prismatic_axis_x(joint_obj)
                and limits is not None
                and limits.lower is not None
                and limits.upper is not None
                and math.isclose(limits.lower, 0.0, abs_tol=1e-9)
                and limits.upper >= min_upper
            ),
            details=f"{joint_obj.name} should be an +X prismatic stage with a positive travel limit.",
        )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0, inner_to_terminal: 0.0}):
        ctx.expect_contact(middle_carriage, outer_body, name="middle_carriage_is_supported_by_outer_body")
        ctx.expect_contact(inner_carriage, middle_carriage, name="inner_carriage_is_supported_by_middle_carriage")
        ctx.expect_contact(terminal_slide, inner_carriage, name="terminal_slide_is_supported_by_inner_carriage")

        ctx.expect_within(middle_carriage, outer_body, axes="yz", margin=0.0015, name="middle_carriage_nested_in_outer_body")
        ctx.expect_within(inner_carriage, middle_carriage, axes="yz", margin=0.0015, name="inner_carriage_nested_in_middle_carriage")
        ctx.expect_within(terminal_slide, inner_carriage, axes="yz", margin=0.0015, name="terminal_slide_nested_in_inner_carriage")

        ctx.expect_overlap(middle_carriage, outer_body, axes="x", min_overlap=0.18, name="middle_carriage_has_rest_pose_engagement")
        ctx.expect_overlap(inner_carriage, middle_carriage, axes="x", min_overlap=0.12, name="inner_carriage_has_rest_pose_engagement")
        ctx.expect_overlap(terminal_slide, inner_carriage, axes="x", min_overlap=0.08, name="terminal_slide_has_rest_pose_engagement")

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
            inner_to_terminal: INNER_TO_TERMINAL_TRAVEL,
        }
    ):
        ctx.expect_within(middle_carriage, outer_body, axes="yz", margin=0.0015, name="middle_carriage_stays_guided_when_extended")
        ctx.expect_within(inner_carriage, middle_carriage, axes="yz", margin=0.0015, name="inner_carriage_stays_guided_when_extended")
        ctx.expect_within(terminal_slide, inner_carriage, axes="yz", margin=0.0015, name="terminal_slide_stays_guided_when_extended")

        ctx.expect_overlap(middle_carriage, outer_body, axes="x", min_overlap=0.15, name="middle_carriage_retains_overlap_when_extended")
        ctx.expect_overlap(inner_carriage, middle_carriage, axes="x", min_overlap=0.10, name="inner_carriage_retains_overlap_when_extended")
        ctx.expect_overlap(terminal_slide, inner_carriage, axes="x", min_overlap=0.065, name="terminal_slide_retains_overlap_when_extended")
        ctx.expect_gap(terminal_slide, outer_body, axis="x", min_gap=0.0005, name="terminal_slide_projects_past_outer_body_at_full_extension")

        extended_terminal_x = ctx.part_world_position(terminal_slide)

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0, inner_to_terminal: 0.0}):
        rest_terminal_x = ctx.part_world_position(terminal_slide)

    ctx.check(
        "serial_prismatic_stack_extends_forward",
        (
            extended_terminal_x is not None
            and rest_terminal_x is not None
            and extended_terminal_x[0] > rest_terminal_x[0] + 0.22
        ),
        details="The terminal slide should advance substantially along +X when all three stages extend.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
