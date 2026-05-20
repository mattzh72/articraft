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


OUTER_LENGTH = 0.320
OUTER_FLANGE_LENGTH = 0.018
OUTER_STAGE_ORIGIN = 0.296
OUTER_RAIL_RADIAL = 0.040
OUTER_RAIL_WIDTH = 0.018
OUTER_RAIL_THICK = 0.010
OUTER_FRONT_STOP_START = 0.286
OUTER_FRONT_STOP_RADIAL = 0.048
OUTER_FRONT_STOP_WIDTH = 0.022
OUTER_FRONT_STOP_THICK = 0.010

INNER_REAR_INSERT = 0.210
INNER_FORWARD_REACH = 0.120
INNER_BODY_LENGTH = INNER_REAR_INSERT + INNER_FORWARD_REACH
INNER_RAIL_RADIAL = 0.027
INNER_RAIL_WIDTH = 0.014
INNER_RAIL_THICK = 0.008
INNER_TRANSITION_START = -0.020
INNER_TRANSITION_LENGTH = 0.042
INNER_TRANSITION_RADIAL = 0.033
INNER_TRANSITION_WIDTH = 0.015
INNER_TRANSITION_THICK = 0.010
INNER_STOP_LENGTH = 0.024
INNER_STOP_RADIAL = 0.039
INNER_STOP_WIDTH = 0.016
INNER_STOP_THICK = 0.008
INNER_FRONT_GUIDE_START = 0.090
INNER_FRONT_GUIDE_LENGTH = 0.030
INNER_FRONT_GUIDE_RADIAL = 0.026
INNER_FRONT_GUIDE_WIDTH = 0.012
INNER_FRONT_GUIDE_THICK = 0.008

ROD_REAR_INSERT = 0.165
ROD_REAR_RADIUS = 0.020
ROD_SHOULDER_LENGTH = 0.016
ROD_SHOULDER_RADIUS = 0.028
ROD_FRONT_SECTION_LENGTH = 0.150
ROD_FRONT_RADIUS = 0.014
ROD_TIP_LENGTH = 0.030
ROD_TIP_RADIUS = 0.010

OUTER_STAGE_TRAVEL = 0.130
ROD_STAGE_TRAVEL = 0.120


def add_box_visual(
    part,
    *,
    name: str,
    x_start: float,
    length: float,
    size_y: float,
    size_z: float,
    material,
    y: float = 0.0,
    z: float = 0.0,
) -> None:
    part.visual(
        Box((length, size_y, size_z)),
        origin=Origin(xyz=(x_start + 0.5 * length, y, z)),
        material=material,
        name=name,
    )


def add_x_cylinder_visual(
    part,
    *,
    name: str,
    x_start: float,
    length: float,
    radius: float,
    material,
    y: float = 0.0,
    z: float = 0.0,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x_start + 0.5 * length, y, z), rpy=(0.0, pi * 0.5, 0.0)),
        material=material,
        name=name,
    )


def add_cage_visuals(
    part,
    *,
    prefix: str,
    x_start: float,
    length: float,
    radial: float,
    width: float,
    thickness: float,
    material,
) -> None:
    add_box_visual(
        part,
        name=f"{prefix}_top",
        x_start=x_start,
        length=length,
        size_y=width,
        size_z=thickness,
        material=material,
        z=radial,
    )
    add_box_visual(
        part,
        name=f"{prefix}_bottom",
        x_start=x_start,
        length=length,
        size_y=width,
        size_z=thickness,
        material=material,
        z=-radial,
    )
    add_box_visual(
        part,
        name=f"{prefix}_left",
        x_start=x_start,
        length=length,
        size_y=thickness,
        size_z=width,
        material=material,
        y=radial,
    )
    add_box_visual(
        part,
        name=f"{prefix}_right",
        x_start=x_start,
        length=length,
        size_y=thickness,
        size_z=width,
        material=material,
        y=-radial,
    )


def add_window_frame_visuals(
    part,
    *,
    prefix: str,
    x_start: float,
    length: float,
    radial: float,
    thickness: float,
    material,
) -> None:
    span = 2.0 * radial + thickness
    add_box_visual(
        part,
        name=f"{prefix}_top",
        x_start=x_start,
        length=length,
        size_y=span,
        size_z=thickness,
        material=material,
        z=radial,
    )
    add_box_visual(
        part,
        name=f"{prefix}_bottom",
        x_start=x_start,
        length=length,
        size_y=span,
        size_z=thickness,
        material=material,
        z=-radial,
    )
    add_box_visual(
        part,
        name=f"{prefix}_left",
        x_start=x_start,
        length=length,
        size_y=thickness,
        size_z=span,
        material=material,
        y=radial,
    )
    add_box_visual(
        part,
        name=f"{prefix}_right",
        x_start=x_start,
        length=length,
        size_y=thickness,
        size_z=span,
        material=material,
        y=-radial,
    )


def author_rear_bracket(part, material) -> None:
    add_box_visual(
        part,
        name="base_plate",
        x_start=-0.060,
        length=0.060,
        size_y=0.180,
        size_z=0.022,
        material=material,
        z=-0.074,
    )
    add_box_visual(
        part,
        name="left_foot",
        x_start=-0.044,
        length=0.044,
        size_y=0.050,
        size_z=0.016,
        material=material,
        y=0.055,
        z=-0.088,
    )
    add_box_visual(
        part,
        name="right_foot",
        x_start=-0.044,
        length=0.044,
        size_y=0.050,
        size_z=0.016,
        material=material,
        y=-0.055,
        z=-0.088,
    )
    add_box_visual(
        part,
        name="center_pedestal",
        x_start=-0.024,
        length=0.024,
        size_y=0.094,
        size_z=0.094,
        material=material,
        z=-0.016,
    )
    add_box_visual(
        part,
        name="left_rib",
        x_start=-0.034,
        length=0.034,
        size_y=0.018,
        size_z=0.058,
        material=material,
        y=0.042,
        z=-0.020,
    )
    add_box_visual(
        part,
        name="right_rib",
        x_start=-0.034,
        length=0.034,
        size_y=0.018,
        size_z=0.058,
        material=material,
        y=-0.042,
        z=-0.020,
    )


def author_outer_sleeve(part, material) -> None:
    add_x_cylinder_visual(
        part,
        name="rear_flange",
        x_start=0.0,
        length=OUTER_FLANGE_LENGTH,
        radius=0.050,
        material=material,
    )
    add_cage_visuals(
        part,
        prefix="main_guide",
        x_start=OUTER_FLANGE_LENGTH,
        length=OUTER_STAGE_ORIGIN - OUTER_FLANGE_LENGTH,
        radial=OUTER_RAIL_RADIAL,
        width=OUTER_RAIL_WIDTH,
        thickness=OUTER_RAIL_THICK,
        material=material,
    )
    add_cage_visuals(
        part,
        prefix="front_stop",
        x_start=OUTER_FRONT_STOP_START,
        length=OUTER_LENGTH - OUTER_FRONT_STOP_START,
        radial=OUTER_FRONT_STOP_RADIAL,
        width=OUTER_FRONT_STOP_WIDTH,
        thickness=OUTER_FRONT_STOP_THICK,
        material=material,
    )


def author_inner_sleeve(part, material) -> None:
    add_cage_visuals(
        part,
        prefix="body",
        x_start=-INNER_REAR_INSERT,
        length=INNER_BODY_LENGTH,
        radial=INNER_RAIL_RADIAL,
        width=INNER_RAIL_WIDTH,
        thickness=INNER_RAIL_THICK,
        material=material,
    )
    add_cage_visuals(
        part,
        prefix="transition",
        x_start=INNER_TRANSITION_START,
        length=INNER_TRANSITION_LENGTH,
        radial=INNER_TRANSITION_RADIAL,
        width=INNER_TRANSITION_WIDTH,
        thickness=INNER_TRANSITION_THICK,
        material=material,
    )
    add_cage_visuals(
        part,
        prefix="retaining_stop",
        x_start=0.0,
        length=INNER_STOP_LENGTH,
        radial=INNER_STOP_RADIAL,
        width=INNER_STOP_WIDTH,
        thickness=INNER_STOP_THICK,
        material=material,
    )
    add_cage_visuals(
        part,
        prefix="front_guide",
        x_start=INNER_FRONT_GUIDE_START,
        length=INNER_FRONT_GUIDE_LENGTH,
        radial=INNER_FRONT_GUIDE_RADIAL,
        width=INNER_FRONT_GUIDE_WIDTH,
        thickness=INNER_FRONT_GUIDE_THICK,
        material=material,
    )
    add_box_visual(
        part,
        name="transition_top_bridge",
        x_start=INNER_TRANSITION_START,
        length=INNER_TRANSITION_LENGTH,
        size_y=0.014,
        size_z=0.012,
        material=material,
        z=0.031,
    )
    add_box_visual(
        part,
        name="transition_bottom_bridge",
        x_start=INNER_TRANSITION_START,
        length=INNER_TRANSITION_LENGTH,
        size_y=0.014,
        size_z=0.012,
        material=material,
        z=-0.031,
    )
    add_box_visual(
        part,
        name="transition_left_bridge",
        x_start=INNER_TRANSITION_START,
        length=INNER_TRANSITION_LENGTH,
        size_y=0.012,
        size_z=0.014,
        material=material,
        y=0.031,
    )
    add_box_visual(
        part,
        name="transition_right_bridge",
        x_start=INNER_TRANSITION_START,
        length=INNER_TRANSITION_LENGTH,
        size_y=0.012,
        size_z=0.014,
        material=material,
        y=-0.031,
    )
    add_window_frame_visuals(
        part,
        prefix="retaining_frame",
        x_start=0.008,
        length=0.008,
        radial=INNER_STOP_RADIAL,
        thickness=INNER_STOP_THICK,
        material=material,
    )


def author_output_rod(part, material) -> None:
    add_x_cylinder_visual(
        part,
        name="rear_guide",
        x_start=-ROD_REAR_INSERT,
        length=ROD_REAR_INSERT,
        radius=ROD_REAR_RADIUS,
        material=material,
    )
    add_x_cylinder_visual(
        part,
        name="shoulder",
        x_start=0.0,
        length=ROD_SHOULDER_LENGTH,
        radius=ROD_SHOULDER_RADIUS,
        material=material,
    )
    add_x_cylinder_visual(
        part,
        name="front_section",
        x_start=ROD_SHOULDER_LENGTH,
        length=ROD_FRONT_SECTION_LENGTH,
        radius=ROD_FRONT_RADIUS,
        material=material,
    )
    add_x_cylinder_visual(
        part,
        name="tip",
        x_start=ROD_SHOULDER_LENGTH + ROD_FRONT_SECTION_LENGTH,
        length=ROD_TIP_LENGTH,
        radius=ROD_TIP_RADIUS,
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_stage_plunger_actuator")

    bracket_material = model.material("bracket_black", rgba=(0.14, 0.15, 0.17, 1.0))
    outer_material = model.material("outer_sleeve_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    inner_material = model.material("inner_sleeve_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    rod_material = model.material("rod_steel", rgba=(0.83, 0.84, 0.86, 1.0))

    rear_bracket = model.part("rear_bracket")
    author_rear_bracket(rear_bracket, bracket_material)

    outer_sleeve = model.part("outer_sleeve")
    author_outer_sleeve(outer_sleeve, outer_material)

    inner_sleeve = model.part("inner_sleeve")
    author_inner_sleeve(inner_sleeve, inner_material)

    output_rod = model.part("output_rod")
    author_output_rod(output_rod, rod_material)

    model.articulation(
        "bracket_to_outer",
        ArticulationType.FIXED,
        parent=rear_bracket,
        child=outer_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_sleeve,
        origin=Origin(xyz=(OUTER_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_rod",
        ArticulationType.PRISMATIC,
        parent=inner_sleeve,
        child=output_rod,
        origin=Origin(xyz=(INNER_FORWARD_REACH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.45,
            lower=0.0,
            upper=ROD_STAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_bracket = object_model.get_part("rear_bracket")
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_sleeve = object_model.get_part("inner_sleeve")
    output_rod = object_model.get_part("output_rod")
    outer_to_inner = object_model.get_articulation("outer_to_inner")
    inner_to_rod = object_model.get_articulation("inner_to_rod")

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
        outer_sleeve,
        rear_bracket,
        contact_tol=5e-4,
        name="outer_sleeve_seats_on_rear_bracket",
    )
    ctx.expect_overlap(
        inner_sleeve,
        outer_sleeve,
        axes="yz",
        min_overlap=0.050,
        name="inner_sleeve_tracks_inside_outer_guide_at_rest",
    )
    ctx.expect_overlap(
        output_rod,
        inner_sleeve,
        axes="yz",
        min_overlap=0.035,
        name="output_rod_tracks_inside_inner_guide_at_rest",
    )

    ctx.expect_origin_distance(
        outer_sleeve,
        rear_bracket,
        axes="yz",
        max_dist=1e-6,
        name="outer_sleeve_bracket_share_centerline",
    )
    ctx.expect_origin_distance(
        inner_sleeve,
        outer_sleeve,
        axes="yz",
        max_dist=1e-6,
        name="inner_sleeve_shares_outer_centerline",
    )
    ctx.expect_origin_distance(
        output_rod,
        inner_sleeve,
        axes="yz",
        max_dist=1e-6,
        name="output_rod_shares_inner_centerline",
    )

    ctx.check(
        "prismatic_stages_slide_along_x",
        outer_to_inner.axis == (1.0, 0.0, 0.0) and inner_to_rod.axis == (1.0, 0.0, 0.0),
        details=f"outer axis={outer_to_inner.axis}, rod axis={inner_to_rod.axis}",
    )

    outer_limits = outer_to_inner.motion_limits
    rod_limits = inner_to_rod.motion_limits
    ctx.check(
        "stage_travel_limits_match_telescoping_layout",
        outer_limits is not None
        and rod_limits is not None
        and outer_limits.lower == 0.0
        and rod_limits.lower == 0.0
        and abs(outer_limits.upper - OUTER_STAGE_TRAVEL) < 1e-9
        and abs(rod_limits.upper - ROD_STAGE_TRAVEL) < 1e-9,
        details=(
            f"outer_limits={outer_limits}, rod_limits={rod_limits}, "
            f"expected=({OUTER_STAGE_TRAVEL}, {ROD_STAGE_TRAVEL})"
        ),
    )

    with ctx.pose({outer_to_inner: 0.105, inner_to_rod: 0.095}):
        ctx.expect_origin_distance(
            inner_sleeve,
            outer_sleeve,
            axes="yz",
            max_dist=1e-6,
            name="inner_sleeve_remains_coaxial_when_extended",
        )
        ctx.expect_origin_distance(
            output_rod,
            inner_sleeve,
            axes="yz",
            max_dist=1e-6,
            name="output_rod_remains_coaxial_when_extended",
        )
        ctx.expect_overlap(
            inner_sleeve,
            outer_sleeve,
            axes="yz",
            min_overlap=0.060,
            name="inner_stage_tracks_within_outer_footprint",
        )
        ctx.expect_overlap(
            output_rod,
            inner_sleeve,
            axes="yz",
            min_overlap=0.040,
            name="rod_tracks_within_inner_footprint",
        )

        outer_pos = ctx.part_world_position(outer_sleeve)
        inner_pos = ctx.part_world_position(inner_sleeve)
        rod_pos = ctx.part_world_position(output_rod)
        if outer_pos is None or inner_pos is None or rod_pos is None:
            ctx.fail("stage_positions_available", "could not resolve articulated part positions")
        else:
            ctx.check(
                "stages_extend_forward_in_order",
                outer_pos[0] < inner_pos[0] < rod_pos[0],
                details=f"outer={outer_pos}, inner={inner_pos}, rod={rod_pos}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
