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


PLATE_LENGTH = 0.40
PLATE_THICKNESS = 0.012
PLATE_HEIGHT = 0.24

OUTER_RAIL_LENGTH = 0.30
OUTER_RAIL_DEPTH = 0.018
OUTER_RAIL_HEIGHT = 0.020
OUTER_RAIL_Z = 0.065
OUTER_RAIL_START_X = 0.05

STAGE1_HOME_X = 0.060
STAGE1_TRAVEL = 0.170
STAGE1_BODY_LENGTH = 0.085
STAGE1_BODY_DEPTH = 0.040
STAGE1_BODY_HEIGHT = 0.168
STAGE1_PAD_DEPTH = 0.014
STAGE1_PAD_HEIGHT = 0.030
STAGE1_BEAM_LENGTH = 0.198
STAGE1_BEAM_DEPTH = 0.028
STAGE1_BEAM_HEIGHT = 0.076
STAGE1_SECONDARY_STRIP_LENGTH = 0.190
STAGE1_SECONDARY_STRIP_DEPTH = 0.010
STAGE1_SECONDARY_STRIP_HEIGHT = 0.016
STAGE1_SECONDARY_STRIP_Z = 0.026

STAGE2_HOME_X = 0.090
STAGE2_TRAVEL = 0.130
STAGE2_BODY_LENGTH = 0.105
STAGE2_BODY_DEPTH = 0.030
STAGE2_BODY_HEIGHT = 0.125
STAGE2_PAD_DEPTH = 0.014
STAGE2_PAD_HEIGHT = 0.022
STAGE2_PAD_Z = 0.028
STAGE2_FRONT_PLATE_THICKNESS = 0.016
STAGE2_FRONT_PLATE_DEPTH = 0.090
STAGE2_FRONT_PLATE_HEIGHT = 0.170


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_double_carriage_module")

    model.material("plate_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    model.material("slider_light", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("fastener_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("accent_blue", rgba=(0.20, 0.41, 0.74, 1.0))

    side_plate = model.part("side_plate")
    _add_box(
        side_plate,
        (PLATE_LENGTH, PLATE_THICKNESS, PLATE_HEIGHT),
        (PLATE_LENGTH / 2.0, 0.0, 0.0),
        "plate_gray",
        name="wall_plate",
    )
    _add_box(
        side_plate,
        (OUTER_RAIL_LENGTH, OUTER_RAIL_DEPTH, OUTER_RAIL_HEIGHT),
        (OUTER_RAIL_START_X + OUTER_RAIL_LENGTH / 2.0, 0.015, OUTER_RAIL_Z),
        "rail_steel",
        name="outer_upper_rail",
    )
    _add_box(
        side_plate,
        (OUTER_RAIL_LENGTH, OUTER_RAIL_DEPTH, OUTER_RAIL_HEIGHT),
        (OUTER_RAIL_START_X + OUTER_RAIL_LENGTH / 2.0, 0.015, -OUTER_RAIL_Z),
        "rail_steel",
        name="outer_lower_rail",
    )
    _add_box(
        side_plate,
        (0.018, OUTER_RAIL_DEPTH, 0.155),
        (OUTER_RAIL_START_X - 0.001, 0.015, 0.0),
        "plate_gray",
        name="rear_travel_stop",
    )
    _add_box(
        side_plate,
        (0.018, OUTER_RAIL_DEPTH, 0.155),
        (OUTER_RAIL_START_X + OUTER_RAIL_LENGTH + 0.001, 0.015, 0.0),
        "plate_gray",
        name="front_travel_stop",
    )
    for index, x_pos in enumerate((0.060, 0.340), start=1):
        for suffix, z_pos in (("upper", 0.095), ("lower", -0.095)):
            _add_cylinder(
                side_plate,
                radius=0.012,
                length=0.006,
                xyz=(x_pos, -0.009, z_pos),
                rpy=(-pi / 2.0, 0.0, 0.0),
                material="fastener_black",
                name=f"mount_boss_{index}_{suffix}",
            )

    first_carriage = model.part("first_carriage")
    _add_box(
        first_carriage,
        (STAGE1_BODY_LENGTH, STAGE1_BODY_DEPTH, STAGE1_BODY_HEIGHT),
        (STAGE1_BODY_LENGTH / 2.0, 0.045, 0.0),
        "carriage_dark",
        name="first_stage_body",
    )
    _add_box(
        first_carriage,
        (STAGE1_BODY_LENGTH, STAGE1_PAD_DEPTH, STAGE1_PAD_HEIGHT),
        (STAGE1_BODY_LENGTH / 2.0, 0.031, OUTER_RAIL_Z),
        "accent_blue",
        name="first_stage_upper_pad",
    )
    _add_box(
        first_carriage,
        (STAGE1_BODY_LENGTH, STAGE1_PAD_DEPTH, STAGE1_PAD_HEIGHT),
        (STAGE1_BODY_LENGTH / 2.0, 0.031, -OUTER_RAIL_Z),
        "accent_blue",
        name="first_stage_lower_pad",
    )
    _add_box(
        first_carriage,
        (STAGE1_BEAM_LENGTH, STAGE1_BEAM_DEPTH, STAGE1_BEAM_HEIGHT),
        (0.181, 0.061, 0.0),
        "slider_light",
        name="second_stage_beam",
    )
    _add_box(
        first_carriage,
        (
            STAGE1_SECONDARY_STRIP_LENGTH,
            STAGE1_SECONDARY_STRIP_DEPTH,
            STAGE1_SECONDARY_STRIP_HEIGHT,
        ),
        (0.183, 0.080, STAGE1_SECONDARY_STRIP_Z),
        "rail_steel",
        name="second_stage_upper_strip",
    )
    _add_box(
        first_carriage,
        (
            STAGE1_SECONDARY_STRIP_LENGTH,
            STAGE1_SECONDARY_STRIP_DEPTH,
            STAGE1_SECONDARY_STRIP_HEIGHT,
        ),
        (0.183, 0.080, -STAGE1_SECONDARY_STRIP_Z),
        "rail_steel",
        name="second_stage_lower_strip",
    )

    second_slider = model.part("second_slider")
    _add_box(
        second_slider,
        (STAGE2_BODY_LENGTH, STAGE2_BODY_DEPTH, STAGE2_BODY_HEIGHT),
        (STAGE2_BODY_LENGTH / 2.0, 0.113, 0.0),
        "slider_light",
        name="second_stage_body",
    )
    _add_box(
        second_slider,
        (STAGE2_BODY_LENGTH, STAGE2_PAD_DEPTH, STAGE2_PAD_HEIGHT),
        (STAGE2_BODY_LENGTH / 2.0, 0.092, STAGE2_PAD_Z),
        "accent_blue",
        name="second_stage_upper_pad",
    )
    _add_box(
        second_slider,
        (STAGE2_BODY_LENGTH, STAGE2_PAD_DEPTH, STAGE2_PAD_HEIGHT),
        (STAGE2_BODY_LENGTH / 2.0, 0.092, -STAGE2_PAD_Z),
        "accent_blue",
        name="second_stage_lower_pad",
    )
    _add_box(
        second_slider,
        (
            STAGE2_FRONT_PLATE_THICKNESS,
            STAGE2_FRONT_PLATE_DEPTH,
            STAGE2_FRONT_PLATE_HEIGHT,
        ),
        (0.113, 0.142, 0.0),
        "plate_gray",
        name="tool_plate",
    )
    for z_pos, name in ((0.040, "tool_mount_upper"), (-0.040, "tool_mount_lower")):
        _add_cylinder(
            second_slider,
            radius=0.009,
            length=0.010,
            xyz=(0.113, 0.192, z_pos),
            rpy=(-pi / 2.0, 0.0, 0.0),
            material="fastener_black",
            name=name,
        )

    model.articulation(
        "side_plate_to_first_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=first_carriage,
        origin=Origin(xyz=(STAGE1_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE1_TRAVEL,
            effort=900.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "first_carriage_to_second_slider",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_slider,
        origin=Origin(xyz=(STAGE2_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE2_TRAVEL,
            effort=450.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    first_carriage = object_model.get_part("first_carriage")
    second_slider = object_model.get_part("second_slider")
    stage1 = object_model.get_articulation("side_plate_to_first_carriage")
    stage2 = object_model.get_articulation("first_carriage_to_second_slider")

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
        first_carriage,
        side_plate,
        name="first_stage_is_supported_by_side_plate_rails",
    )
    ctx.expect_contact(
        second_slider,
        first_carriage,
        name="second_stage_is_supported_by_carried_rail",
    )
    ctx.expect_overlap(
        first_carriage,
        side_plate,
        axes="xz",
        min_overlap=0.08,
        name="first_carriage_tracks_within_side_module_envelope",
    )
    ctx.expect_overlap(
        second_slider,
        first_carriage,
        axes="xz",
        min_overlap=0.06,
        name="second_slider_tracks_within_first_carriage_envelope",
    )

    rest_first_pos = ctx.part_world_position(first_carriage)
    rest_second_pos = ctx.part_world_position(second_slider)

    stage1_upper = (
        stage1.motion_limits.upper
        if stage1.motion_limits is not None and stage1.motion_limits.upper is not None
        else 0.0
    )
    stage2_upper = (
        stage2.motion_limits.upper
        if stage2.motion_limits is not None and stage2.motion_limits.upper is not None
        else 0.0
    )

    with ctx.pose({stage1: stage1_upper}):
        stage1_first_pos = ctx.part_world_position(first_carriage)
        stage1_second_pos = ctx.part_world_position(second_slider)
        ctx.expect_contact(
            first_carriage,
            side_plate,
            name="first_stage_stays_supported_at_full_extension",
        )

    with ctx.pose({stage2: stage2_upper}):
        stage2_second_pos = ctx.part_world_position(second_slider)
        ctx.expect_contact(
            second_slider,
            first_carriage,
            name="second_stage_stays_supported_at_full_extension",
        )

    rest_relative_x = (
        None
        if rest_first_pos is None or rest_second_pos is None
        else rest_second_pos[0] - rest_first_pos[0]
    )
    carried_relative_x = (
        None
        if stage1_first_pos is None or stage1_second_pos is None
        else stage1_second_pos[0] - stage1_first_pos[0]
    )

    ctx.check(
        "first_stage_moves_outward_along_positive_x",
        rest_first_pos is not None
        and stage1_first_pos is not None
        and stage1_first_pos[0] > rest_first_pos[0] + 0.12,
        details=(
            f"expected first carriage to move outward by > 0.12 m; "
            f"rest={None if rest_first_pos is None else rest_first_pos[0]:.6f}, "
            f"extended={None if stage1_first_pos is None else stage1_first_pos[0]:.6f}"
        ),
    )
    ctx.check(
        "second_slider_is_carried_when_stage1_moves",
        rest_relative_x is not None
        and carried_relative_x is not None
        and abs(carried_relative_x - rest_relative_x) <= 1e-4,
        details=(
            f"expected retracted second slider to keep its carriage-relative x offset; "
            f"rest={rest_relative_x}, carried={carried_relative_x}"
        ),
    )
    ctx.check(
        "second_stage_extends_from_first_carriage",
        rest_second_pos is not None
        and stage2_second_pos is not None
        and stage2_second_pos[0] > rest_second_pos[0] + 0.09,
        details=(
            f"expected second slider to extend by > 0.09 m; "
            f"rest={None if rest_second_pos is None else rest_second_pos[0]:.6f}, "
            f"extended={None if stage2_second_pos is None else stage2_second_pos[0]:.6f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
