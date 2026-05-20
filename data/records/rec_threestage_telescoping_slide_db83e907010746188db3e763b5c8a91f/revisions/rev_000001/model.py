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


SLIDE_AXIS = (1.0, 0.0, 0.0)

OUTER_LENGTH = 0.72
OUTER_WIDTH = 0.086
OUTER_HEIGHT = 0.060
OUTER_WALL = 0.006
OUTER_RAIL_HEIGHT = 0.004
OUTER_RAIL_WIDTH = 0.010

MIDDLE_LENGTH = 0.60
MIDDLE_WIDTH = 0.068
MIDDLE_HEIGHT = 0.046
MIDDLE_WALL = 0.0055
MIDDLE_RAIL_HEIGHT = 0.003
MIDDLE_RAIL_WIDTH = 0.009

INNER_LENGTH = 0.47
INNER_WIDTH = 0.051
INNER_HEIGHT = 0.034
INNER_WALL = 0.0045

SHOE_LENGTH = 0.074
SHOE_WIDTH = 0.040
SHOE_HEIGHT = 0.022

OUTER_TO_MIDDLE_TRAVEL = 0.300
MIDDLE_TO_INNER_TRAVEL = 0.225

OUTER_TO_MIDDLE_Z = OUTER_WALL + OUTER_RAIL_HEIGHT
MIDDLE_TO_INNER_Z = MIDDLE_WALL + MIDDLE_RAIL_HEIGHT
INNER_TO_SHOE_Z = 0.006

OUTER_STOP_FACE = 0.685
MIDDLE_STOP_FACE = 0.560

BUFFER_LEN = 0.012
STOP_TAB_LEN = 0.015
STOP_TAB_DEPTH = 0.0032

MIDDLE_STOP_TAB_X = OUTER_STOP_FACE - BUFFER_LEN - OUTER_TO_MIDDLE_TRAVEL - STOP_TAB_LEN
INNER_STOP_TAB_X = MIDDLE_STOP_FACE - BUFFER_LEN - MIDDLE_TO_INNER_TRAVEL - STOP_TAB_LEN


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x, y, z))
    )


def _apply_counterbores(
    body,
    *,
    x_positions: tuple[float, ...],
    y_positions: tuple[float, ...],
    through_radius: float,
    counterbore_radius: float,
    floor_thickness: float,
    counterbore_depth: float,
):
    points = [(x, y) for x in x_positions for y in y_positions]
    through = cq.Workplane("XY").pushPoints(points).circle(through_radius).extrude(floor_thickness + 0.003)
    counter = (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(counterbore_radius)
        .extrude(counterbore_depth + 0.001)
        .translate((0.0, 0.0, floor_thickness - counterbore_depth))
    )
    return body.cut(through).cut(counter)


def _channel_member(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    rail_length: float,
    rail_width: float,
    rail_height: float,
    rail_x: float,
    rail_side_gap: float,
    window_spans: tuple[tuple[float, float], ...],
    window_z: float,
    window_height: float,
    hole_x_positions: tuple[float, ...],
    hole_y_positions: tuple[float, ...],
    stop_face: float | None = None,
    stop_lane_y: float | None = None,
    stop_tab_x: float | None = None,
):
    floor = _box(length, width, wall)
    left_wall = _box(length, wall, height - wall, y=width * 0.5 - wall * 0.5, z=wall)
    right_wall = _box(length, wall, height - wall, y=-width * 0.5 + wall * 0.5, z=wall)
    body = floor.union(left_wall).union(right_wall)

    inner_half_width = (width - 2.0 * wall) * 0.5
    rail_y = inner_half_width - rail_side_gap - rail_width * 0.5
    rail_left = _box(rail_length, rail_width, rail_height, x=rail_x, y=rail_y, z=wall)
    rail_right = _box(rail_length, rail_width, rail_height, x=rail_x, y=-rail_y, z=wall)
    rail_left = rail_left.edges("|X and >Z").chamfer(min(rail_height * 0.45, rail_width * 0.22))
    rail_right = rail_right.edges("|X and >Z").chamfer(min(rail_height * 0.45, rail_width * 0.22))
    body = body.union(rail_left).union(rail_right)

    for span_start, span_end in window_spans:
        cutter = _box(
            span_end - span_start,
            width + 0.010,
            window_height,
            x=span_start,
            z=window_z,
        )
        body = body.cut(cutter)

    body = _apply_counterbores(
        body,
        x_positions=hole_x_positions,
        y_positions=hole_y_positions,
        through_radius=0.0022,
        counterbore_radius=0.0042,
        floor_thickness=wall,
        counterbore_depth=min(0.0025, wall * 0.6),
    )

    if stop_face is not None and stop_lane_y is not None:
        buffer_left = _box(
            BUFFER_LEN,
            max(wall * 0.95, 0.004),
            rail_height,
            x=stop_face - BUFFER_LEN,
            y=stop_lane_y,
            z=wall,
        )
        buffer_right = _box(
            BUFFER_LEN,
            max(wall * 0.95, 0.004),
            rail_height,
            x=stop_face - BUFFER_LEN,
            y=-stop_lane_y,
            z=wall,
        )
        body = body.union(buffer_left).union(buffer_right)

    if stop_tab_x is not None and stop_lane_y is not None:
        tab_left = _box(
            STOP_TAB_LEN,
            max(wall * 0.95, 0.004),
            STOP_TAB_DEPTH,
            x=stop_tab_x,
            y=stop_lane_y,
            z=-STOP_TAB_DEPTH,
        )
        tab_right = _box(
            STOP_TAB_LEN,
            max(wall * 0.95, 0.004),
            STOP_TAB_DEPTH,
            x=stop_tab_x,
            y=-stop_lane_y,
            z=-STOP_TAB_DEPTH,
        )
        body = body.union(tab_left).union(tab_right)

    return body


def _outer_beam_shape():
    body = _channel_member(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        rail_length=0.60,
        rail_width=OUTER_RAIL_WIDTH,
        rail_height=OUTER_RAIL_HEIGHT,
        rail_x=0.050,
        rail_side_gap=0.006,
        window_spans=((0.120, 0.285), (0.375, 0.565)),
        window_z=0.020,
        window_height=0.020,
        hole_x_positions=(0.080, 0.220, 0.500, 0.640),
        hole_y_positions=(-0.017, 0.017),
        stop_face=OUTER_STOP_FACE,
        stop_lane_y=MIDDLE_WIDTH * 0.5 - MIDDLE_WALL * 0.5,
    )

    foot_rows = []
    for x_center in (0.095, 0.255, 0.465, 0.625):
        foot_rows.append(
            _box(0.030, OUTER_WIDTH * 0.70, 0.004, x=x_center - 0.015, z=-0.004)
        )
    for foot in foot_rows:
        body = body.union(foot)

    return body


def _middle_stage_shape():
    return _channel_member(
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        rail_length=0.48,
        rail_width=MIDDLE_RAIL_WIDTH,
        rail_height=MIDDLE_RAIL_HEIGHT,
        rail_x=0.055,
        rail_side_gap=0.0045,
        window_spans=((0.090, 0.230), (0.470, 0.525)),
        window_z=0.016,
        window_height=0.016,
        hole_x_positions=(0.070, 0.180, 0.415, 0.525),
        hole_y_positions=(-0.013, 0.013),
        stop_face=MIDDLE_STOP_FACE,
        stop_lane_y=INNER_WIDTH * 0.5 - INNER_WALL * 0.5,
        stop_tab_x=MIDDLE_STOP_TAB_X,
    )


def _inner_stage_shape():
    body = _channel_member(
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        rail_length=0.34,
        rail_width=0.007,
        rail_height=0.0026,
        rail_x=0.060,
        rail_side_gap=0.004,
        window_spans=((0.085, 0.200),),
        window_z=0.012,
        window_height=0.012,
        hole_x_positions=(0.080, 0.190, 0.310, 0.405),
        hole_y_positions=(-0.010, 0.010),
        stop_tab_x=INNER_STOP_TAB_X,
        stop_lane_y=INNER_WIDTH * 0.5 - INNER_WALL * 0.5,
    )

    nose_plate = _box(0.020, INNER_WIDTH * 0.88, 0.006, x=INNER_LENGTH - 0.020, z=INNER_HEIGHT - 0.006)
    return body.union(nose_plate)


def _carriage_shoe_shape():
    body = _box(SHOE_LENGTH, SHOE_WIDTH, SHOE_HEIGHT)
    body = body.edges("|Z").fillet(0.0015)

    underside_relief = _box(
        SHOE_LENGTH * 0.56,
        SHOE_WIDTH * 0.48,
        SHOE_HEIGHT * 0.44,
        x=0.014,
        z=0.0,
    )
    body = body.cut(underside_relief)

    top_holes = (
        cq.Workplane("XY")
        .pushPoints([(0.022, 0.0), (0.052, 0.0)])
        .circle(0.0021)
        .extrude(SHOE_HEIGHT + 0.003)
    )
    top_cbore = (
        cq.Workplane("XY")
        .pushPoints([(0.022, 0.0), (0.052, 0.0)])
        .circle(0.0040)
        .extrude(0.0045)
        .translate((0.0, 0.0, SHOE_HEIGHT - 0.0045))
    )
    body = body.cut(top_holes).cut(top_cbore)

    nose = _box(0.012, SHOE_WIDTH * 0.78, 0.010, x=SHOE_LENGTH - 0.004, z=0.006)
    return body.union(nose)


def _add_mesh_visual(part, shape, mesh_name: str, material: str, visual_name: str):
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=visual_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_linear_extension")

    model.material("outer_steel", rgba=(0.26, 0.29, 0.32, 1.0))
    model.material("middle_steel", rgba=(0.48, 0.52, 0.57, 1.0))
    model.material("inner_steel", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("shoe_steel", rgba=(0.56, 0.47, 0.34, 1.0))

    outer_beam = model.part("outer_beam")
    middle_stage = model.part("middle_stage")
    inner_stage = model.part("inner_stage")
    carriage_shoe = model.part("carriage_shoe")

    _add_mesh_visual(
        outer_beam,
        _outer_beam_shape(),
        "outer_beam_body",
        "outer_steel",
        "outer_beam_body",
    )
    _add_mesh_visual(
        middle_stage,
        _middle_stage_shape(),
        "middle_stage_body",
        "middle_steel",
        "middle_stage_body",
    )
    _add_mesh_visual(
        inner_stage,
        _inner_stage_shape(),
        "inner_stage_body",
        "inner_steel",
        "inner_stage_body",
    )
    _add_mesh_visual(
        carriage_shoe,
        _carriage_shoe_shape(),
        "carriage_shoe_body",
        "shoe_steel",
        "carriage_shoe_body",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_beam,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=900.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_INNER_Z)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=650.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "inner_to_shoe",
        ArticulationType.FIXED,
        parent=inner_stage,
        child=carriage_shoe,
        origin=Origin(xyz=(INNER_LENGTH, 0.0, INNER_TO_SHOE_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_beam = object_model.get_part("outer_beam")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    carriage_shoe = object_model.get_part("carriage_shoe")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.allow_overlap(
        outer_beam,
        middle_stage,
        reason="Nested channel fit uses nominal zero-clearance guide contact between outer beam raceways and middle stage floor lands.",
    )
    ctx.allow_overlap(
        middle_stage,
        inner_stage,
        reason="Nested channel fit uses nominal zero-clearance guide contact between middle stage raceways and inner stage floor lands.",
    )

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
        "outer_to_middle axis",
        tuple(outer_to_middle.axis) == SLIDE_AXIS,
        details=f"axis={outer_to_middle.axis}",
    )
    ctx.check(
        "middle_to_inner axis",
        tuple(middle_to_inner.axis) == SLIDE_AXIS,
        details=f"axis={middle_to_inner.axis}",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(middle_stage, outer_beam, name="middle supported on outer guides at home")
        ctx.expect_contact(inner_stage, middle_stage, name="inner supported on middle guides at home")
        ctx.expect_contact(carriage_shoe, inner_stage, name="shoe mounted to inner stage")
        ctx.expect_within(middle_stage, outer_beam, axes="yz", margin=0.0, name="middle within outer section")
        ctx.expect_within(inner_stage, middle_stage, axes="yz", margin=0.0, name="inner within middle section")
        ctx.expect_within(inner_stage, outer_beam, axes="yz", margin=0.0, name="inner stays nested inside outer section")
        ctx.expect_within(carriage_shoe, inner_stage, axes="yz", margin=0.0, name="shoe centered within inner section")
        ctx.expect_overlap(middle_stage, outer_beam, axes="x", min_overlap=0.58, name="home outer-middle engagement")
        ctx.expect_overlap(inner_stage, middle_stage, axes="x", min_overlap=0.44, name="home middle-inner engagement")
        ctx.expect_gap(
            carriage_shoe,
            inner_stage,
            axis="x",
            min_gap=0.0,
            max_gap=0.0,
            name="shoe flush to inner face",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_home_pose")

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL, middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        ctx.expect_contact(middle_stage, outer_beam, name="middle supported on outer guides at full extension")
        ctx.expect_contact(inner_stage, middle_stage, name="inner supported on middle guides at full extension")
        ctx.expect_contact(carriage_shoe, inner_stage, name="shoe mounted at full extension")
        ctx.expect_within(middle_stage, outer_beam, axes="yz", margin=0.0, name="middle stays in outer section at full extension")
        ctx.expect_within(inner_stage, middle_stage, axes="yz", margin=0.0, name="inner stays in middle section at full extension")
        ctx.expect_within(inner_stage, outer_beam, axes="yz", margin=0.0, name="inner stays laterally captured by outer section at full extension")
        ctx.expect_overlap(middle_stage, outer_beam, axes="x", min_overlap=0.29, name="full-extension outer-middle engagement")
        ctx.expect_overlap(inner_stage, middle_stage, axes="x", min_overlap=0.24, name="full-extension middle-inner engagement")
        ctx.expect_gap(
            carriage_shoe,
            inner_stage,
            axis="x",
            min_gap=0.0,
            max_gap=0.0,
            name="shoe stays flush at full extension",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_full_extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
