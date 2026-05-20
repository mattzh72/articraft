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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SECTION_LENGTH = 2.70
BASE_RAIL_CENTER_X = 0.211
MIDDLE_RAIL_CENTER_X = 0.181
TOP_RAIL_CENTER_X = 0.157
BASE_RAIL_WIDTH = 0.040
MIDDLE_RAIL_WIDTH = 0.034
TOP_RAIL_WIDTH = 0.030
BASE_RAIL_DEPTH = 0.088
MIDDLE_RAIL_DEPTH = 0.074
TOP_RAIL_DEPTH = 0.062
RAIL_WALL = 0.0036
RUNG_COUNT = 9
RUNG_DEPTH = 0.030
RUNG_HEIGHT = 0.010
RUNG_RADIUS = 0.010
MIDDLE_RETRACTED_Z = 0.82
TOP_RETRACTED_Z = 0.82
MIDDLE_EXTENSION = 1.52
TOP_EXTENSION = 1.46
SECTION_Y_STEP = 0.060


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _channel_profile(width: float, depth: float, wall: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    inner_x = -half_width + wall
    inner_depth = half_depth - wall
    return [
        (-half_width, -half_depth),
        (half_width, -half_depth),
        (half_width, -half_depth + wall),
        (inner_x, -half_depth + wall),
        (inner_x, inner_depth),
        (half_width, inner_depth),
        (half_width, half_depth),
        (-half_width, half_depth),
    ]


def _add_d_rung(part, *, length: float, z: float, aluminum) -> None:
    part.visual(
        Box((length, RUNG_DEPTH, RUNG_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.004, z + 0.001)),
        material=aluminum,
    )
    part.visual(
        Cylinder(radius=RUNG_RADIUS, length=length),
        origin=Origin(xyz=(0.0, 0.004, z), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
    )


def _add_section(
    part,
    *,
    prefix: str,
    rail_center_x: float,
    rail_width: float,
    rail_depth: float,
    aluminum,
    accent,
) -> None:
    rail_z = SECTION_LENGTH * 0.5
    half_depth = rail_depth * 0.5
    flange_width = rail_width - RAIL_WALL
    for side_sign, visual_name in ((-1.0, "left_rail"), (1.0, "right_rail")):
        x = side_sign * rail_center_x
        outer_wall_x = x + side_sign * (rail_width * 0.5 - RAIL_WALL * 0.5)
        flange_x = x - side_sign * (RAIL_WALL * 0.5)
        part.visual(
            Box((RAIL_WALL, rail_depth, SECTION_LENGTH)),
            origin=Origin(xyz=(outer_wall_x, 0.0, rail_z)),
            material=aluminum,
            name=visual_name,
        )
        part.visual(
            Box((flange_width, RAIL_WALL, SECTION_LENGTH)),
            origin=Origin(xyz=(flange_x, -half_depth + RAIL_WALL * 0.5, rail_z)),
            material=aluminum,
        )
        part.visual(
            Box((flange_width, RAIL_WALL, SECTION_LENGTH)),
            origin=Origin(xyz=(flange_x, half_depth - RAIL_WALL * 0.5, rail_z)),
            material=aluminum,
        )
        part.visual(
            Box((RAIL_WALL * 1.5, rail_depth * 0.86, 0.22)),
            origin=Origin(xyz=(outer_wall_x, 0.0, 0.20)),
            material=accent,
        )
        part.visual(
            Box((RAIL_WALL * 1.5, rail_depth * 0.86, 0.22)),
            origin=Origin(xyz=(outer_wall_x, 0.0, SECTION_LENGTH - 0.20)),
            material=accent,
        )

    rung_length = 2.0 * (rail_center_x + rail_width * 0.5 - RAIL_WALL)
    rung_start = 0.34
    rung_span = SECTION_LENGTH - 0.68
    for rung_index in range(RUNG_COUNT):
        z = rung_start + rung_span * rung_index / (RUNG_COUNT - 1)
        _add_d_rung(part, length=rung_length, z=z, aluminum=aluminum)


def _add_guide_brackets(
    part,
    *,
    prefix: str,
    parent_rail_center_x: float,
    parent_rail_width: float,
    child_rail_center_x: float,
    child_rail_width: float,
    child_rail_depth: float,
    child_y: float,
    z_center: float,
    bracket_material,
) -> None:
    guide_length = 0.18
    saddle_thickness = 0.006
    bridge_depth = 0.016
    pad_clearance = 0.003
    bridge_length = abs(
        (parent_rail_center_x - parent_rail_width * 0.5)
        - (child_rail_center_x + child_rail_width * 0.5 + saddle_thickness + pad_clearance)
    ) + 0.006
    back_y = child_y - child_rail_depth * 0.5 - bridge_depth * 0.5 - 0.004
    for side_sign in (-1.0, 1.0):
        bracket_x = side_sign * child_rail_center_x
        saddle_x = bracket_x + side_sign * (
            child_rail_width * 0.5 + saddle_thickness * 0.5 + pad_clearance
        )
        part.visual(
            Box((saddle_thickness, child_rail_depth + 0.010, guide_length)),
            origin=Origin(xyz=(saddle_x, child_y, z_center)),
            material=bracket_material,
        )
        part.visual(
            Box((bridge_length, bridge_depth, 0.044)),
            origin=Origin(
                xyz=((side_sign * (parent_rail_center_x - parent_rail_width * 0.5) + saddle_x) * 0.5, back_y, z_center - 0.048)
            ),
            material=bracket_material,
        )
        part.visual(
            Box((bridge_length, bridge_depth, 0.044)),
            origin=Origin(
                xyz=((side_sign * (parent_rail_center_x - parent_rail_width * 0.5) + saddle_x) * 0.5, back_y, z_center + 0.048)
            ),
            material=bracket_material,
        )
        part.visual(
            Box((saddle_thickness * 0.6, child_rail_depth * 0.66, guide_length * 0.72)),
            origin=Origin(
                xyz=(bracket_x + side_sign * (child_rail_width * 0.5 + 0.0015), child_y - 0.003, z_center)
            ),
            material=bracket_material,
        )


def _add_base_foot_mounts(part, *, rail_center_x: float, rail_width: float, aluminum) -> None:
    for side_sign in (-1.0, 1.0):
        x = side_sign * rail_center_x
        for y in (-BASE_RAIL_DEPTH * 0.5 + 0.006, BASE_RAIL_DEPTH * 0.5 - 0.006):
            part.visual(
                Box((rail_width * 0.72, 0.012, 0.054)),
                origin=Origin(xyz=(x, y, -0.001)),
                material=aluminum,
            )
        part.visual(
            Box((rail_width * 0.72, BASE_RAIL_DEPTH * 0.93, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.001)),
            material=aluminum,
        )


def _build_swivel_foot(part, *, steel, rubber) -> None:
    part.visual(
        Box((0.028, 0.060, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="hinge_lug",
    )
    part.visual(
        Box((0.036, 0.060, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=steel,
    )
    part.visual(
        Box((0.102, 0.130, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=rubber,
        name="pad",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.0, -0.026, -0.003), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.0, 0.026, -0.003), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.014, length=0.102),
        origin=Origin(xyz=(0.0, 0.040, -0.078), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_section_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.81, 0.83, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base_section = model.part("base_section")
    _add_section(
        base_section,
        prefix="base",
        rail_center_x=BASE_RAIL_CENTER_X,
        rail_width=BASE_RAIL_WIDTH,
        rail_depth=BASE_RAIL_DEPTH,
        aluminum=aluminum,
        accent=dark_grey,
    )
    _add_guide_brackets(
        base_section,
        prefix="base",
        parent_rail_center_x=BASE_RAIL_CENTER_X,
        parent_rail_width=BASE_RAIL_WIDTH,
        child_rail_center_x=MIDDLE_RAIL_CENTER_X,
        child_rail_width=MIDDLE_RAIL_WIDTH,
        child_rail_depth=MIDDLE_RAIL_DEPTH,
        child_y=SECTION_Y_STEP,
        z_center=SECTION_LENGTH - 0.14,
        bracket_material=brushed,
    )
    _add_base_foot_mounts(
        base_section,
        rail_center_x=BASE_RAIL_CENTER_X,
        rail_width=BASE_RAIL_WIDTH,
        aluminum=brushed,
    )
    base_section.inertial = Inertial.from_geometry(
        Box((0.49, 0.15, SECTION_LENGTH)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.02, SECTION_LENGTH * 0.5)),
    )

    middle_section = model.part("middle_section")
    _add_section(
        middle_section,
        prefix="middle",
        rail_center_x=MIDDLE_RAIL_CENTER_X,
        rail_width=MIDDLE_RAIL_WIDTH,
        rail_depth=MIDDLE_RAIL_DEPTH,
        aluminum=aluminum,
        accent=dark_grey,
    )
    _add_guide_brackets(
        middle_section,
        prefix="middle",
        parent_rail_center_x=MIDDLE_RAIL_CENTER_X,
        parent_rail_width=MIDDLE_RAIL_WIDTH,
        child_rail_center_x=TOP_RAIL_CENTER_X,
        child_rail_width=TOP_RAIL_WIDTH,
        child_rail_depth=TOP_RAIL_DEPTH,
        child_y=SECTION_Y_STEP,
        z_center=SECTION_LENGTH - 0.14,
        bracket_material=brushed,
    )
    middle_section.inertial = Inertial.from_geometry(
        Box((0.40, 0.13, SECTION_LENGTH)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.02, SECTION_LENGTH * 0.5)),
    )

    top_section = model.part("top_section")
    _add_section(
        top_section,
        prefix="top",
        rail_center_x=TOP_RAIL_CENTER_X,
        rail_width=TOP_RAIL_WIDTH,
        rail_depth=TOP_RAIL_DEPTH,
        aluminum=aluminum,
        accent=dark_grey,
    )
    top_section.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(-TOP_RAIL_CENTER_X, 0.0, SECTION_LENGTH - 0.025)),
        material=rubber,
    )
    top_section.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(TOP_RAIL_CENTER_X, 0.0, SECTION_LENGTH - 0.025)),
        material=rubber,
    )
    top_section.inertial = Inertial.from_geometry(
        Box((0.35, 0.11, SECTION_LENGTH)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.03, SECTION_LENGTH * 0.5)),
    )

    left_foot = model.part("left_foot")
    _build_swivel_foot(left_foot, steel=dark_grey, rubber=rubber)
    left_foot.inertial = Inertial.from_geometry(
        Box((0.11, 0.13, 0.10)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    right_foot = model.part("right_foot")
    _build_swivel_foot(right_foot, steel=dark_grey, rubber=rubber)
    right_foot.inertial = Inertial.from_geometry(
        Box((0.11, 0.13, 0.10)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=middle_section,
        origin=Origin(xyz=(0.0, SECTION_Y_STEP, MIDDLE_RETRACTED_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.55,
            lower=0.0,
            upper=MIDDLE_EXTENSION,
        ),
    )
    model.articulation(
        "middle_to_top",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=top_section,
        origin=Origin(xyz=(0.0, SECTION_Y_STEP, TOP_RETRACTED_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.55,
            lower=0.0,
            upper=TOP_EXTENSION,
        ),
    )
    model.articulation(
        "left_foot_pivot",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=left_foot,
        origin=Origin(xyz=(-BASE_RAIL_CENTER_X, 0.0, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.85, upper=0.55),
    )
    model.articulation(
        "right_foot_pivot",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=right_foot,
        origin=Origin(xyz=(BASE_RAIL_CENTER_X, 0.0, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.85, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    middle = object_model.get_part("middle_section")
    top = object_model.get_part("top_section")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    middle_slide = object_model.get_articulation("base_to_middle")
    top_slide = object_model.get_articulation("middle_to_top")
    left_pivot = object_model.get_articulation("left_foot_pivot")
    right_pivot = object_model.get_articulation("right_foot_pivot")

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
        "slide_axes_are_vertical",
        middle_slide.axis == (0.0, 0.0, 1.0) and top_slide.axis == (0.0, 0.0, 1.0),
        details=f"{middle_slide.axis=}, {top_slide.axis=}",
    )
    ctx.check(
        "foot_axes_are_transverse",
        left_pivot.axis == (1.0, 0.0, 0.0) and right_pivot.axis == (1.0, 0.0, 0.0),
        details=f"{left_pivot.axis=}, {right_pivot.axis=}",
    )
    ctx.check(
        "foot_limits_allow_swivel",
        (
            left_pivot.motion_limits is not None
            and right_pivot.motion_limits is not None
            and left_pivot.motion_limits.lower is not None
            and left_pivot.motion_limits.upper is not None
            and right_pivot.motion_limits.lower is not None
            and right_pivot.motion_limits.upper is not None
            and left_pivot.motion_limits.lower < 0.0 < left_pivot.motion_limits.upper
            and right_pivot.motion_limits.lower < 0.0 < right_pivot.motion_limits.upper
        ),
        details="swivel feet should rotate both above and below the rail line",
    )

    ctx.expect_origin_gap(
        middle,
        base,
        axis="z",
        min_gap=0.81,
        max_gap=0.83,
        name="middle_section_retracted_height",
    )
    ctx.expect_origin_gap(
        top,
        middle,
        axis="z",
        min_gap=0.81,
        max_gap=0.83,
        name="top_section_retracted_height",
    )
    ctx.expect_within(
        middle,
        base,
        axes="x",
        margin=0.0,
        name="middle_section_nested_between_base_rails",
    )
    ctx.expect_within(
        top,
        middle,
        axes="x",
        margin=0.0,
        name="top_section_nested_between_middle_rails",
    )
    ctx.expect_contact(
        left_foot,
        base,
        contact_tol=0.0015,
        name="left_swivel_foot_attached",
    )
    ctx.expect_contact(
        right_foot,
        base,
        contact_tol=0.0015,
        name="right_swivel_foot_attached",
    )

    with ctx.pose({middle_slide: MIDDLE_EXTENSION, top_slide: TOP_EXTENSION}):
        ctx.expect_origin_gap(
            middle,
            base,
            axis="z",
            min_gap=2.33,
            max_gap=2.35,
            name="middle_section_extended_height",
        )
        ctx.expect_origin_gap(
            top,
            middle,
            axis="z",
            min_gap=2.27,
            max_gap=2.29,
            name="top_section_extended_height",
        )
        ctx.expect_within(
            top,
            middle,
            axes="x",
            margin=0.0,
            name="top_section_remains_tracked_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
