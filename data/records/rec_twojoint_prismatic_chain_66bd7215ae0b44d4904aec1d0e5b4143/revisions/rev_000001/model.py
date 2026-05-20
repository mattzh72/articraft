from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.160
PLATE_HEIGHT = 0.200

BASE_LENGTH = 0.520
BASE_OUTER_WIDTH = 0.120
BASE_OUTER_HEIGHT = 0.050
BASE_GUIDE_WIDTH = 0.028
BASE_GUIDE_HEIGHT = 0.012
BASE_GUIDE_OFFSET = 0.026
BASE_GUIDE_START = 0.020
BASE_REAR_BLOCK_LENGTH = 0.085

CARRIAGE_LENGTH = 0.300
CARRIAGE_OUTER_WIDTH = 0.110
CARRIAGE_OUTER_HEIGHT = 0.024
CARRIAGE_PLATE_THICKNESS = 0.012
CARRIAGE_SECONDARY_RAIL_WIDTH = 0.068
CARRIAGE_SECONDARY_RAIL_HEIGHT = 0.016
CARRIAGE_SKIRT_WIDTH = 0.012

SLIDER_BAR_LENGTH = 0.200
SLIDER_WIDTH = 0.088
SLIDER_HEIGHT = 0.012
FRONT_PLATE_THICKNESS = 0.012
FRONT_PLATE_WIDTH = 0.088
FRONT_PLATE_HEIGHT = 0.048

OUTER_HOME = 0.030
OUTER_TRAVEL = 0.180
INNER_HOME = 0.100
INNER_TRAVEL = 0.090
_CUT_EPS = 0.002


def _box_from_zero(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate((length / 2.0, 0.0, 0.0))


def _tube_open_front(
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    *,
    rear_wall: float,
) -> cq.Workplane:
    outer = _box_from_zero(length, outer_width, outer_height)
    inner_width = outer_width - 2.0 * wall
    inner_height = outer_height - 2.0 * wall
    inner_start = rear_wall
    inner_end = length + _CUT_EPS
    inner_length = inner_end - inner_start
    inner = _box_from_zero(inner_length, inner_width, inner_height).translate((inner_start, 0.0, 0.0))
    return outer.cut(inner)


def _tube_open_both(length: float, outer_width: float, outer_height: float, wall: float) -> cq.Workplane:
    outer = _box_from_zero(length, outer_width, outer_height)
    inner_width = outer_width - 2.0 * wall
    inner_height = outer_height - 2.0 * wall
    inner_length = length + 2.0 * _CUT_EPS
    inner = _box_from_zero(inner_length, inner_width, inner_height).translate((-_CUT_EPS, 0.0, 0.0))
    return outer.cut(inner)


def _make_wall_hole_cutter(x_start: float, depth: float, points: list[tuple[float, float]], radius: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .pushPoints(points)
        .circle(radius)
        .extrude(depth)
        .translate((x_start, 0.0, 0.0))
    )


def _make_base_support_shape() -> cq.Workplane:
    wall_plate = cq.Workplane("XY").box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT).translate(
        (-PLATE_THICKNESS / 2.0, 0.0, 0.0)
    )
    support_beam = _box_from_zero(BASE_LENGTH, BASE_OUTER_WIDTH, BASE_OUTER_HEIGHT)
    rear_block = _box_from_zero(BASE_REAR_BLOCK_LENGTH, BASE_OUTER_WIDTH + 0.020, BASE_OUTER_HEIGHT + 0.016).translate(
        (0.0, 0.0, 0.008)
    )
    left_guide = _box_from_zero(
        BASE_LENGTH - BASE_GUIDE_START - 0.020,
        BASE_GUIDE_WIDTH,
        BASE_GUIDE_HEIGHT,
    ).translate((BASE_GUIDE_START, -BASE_GUIDE_OFFSET, BASE_OUTER_HEIGHT / 2.0 + BASE_GUIDE_HEIGHT / 2.0))
    right_guide = _box_from_zero(
        BASE_LENGTH - BASE_GUIDE_START - 0.020,
        BASE_GUIDE_WIDTH,
        BASE_GUIDE_HEIGHT,
    ).translate((BASE_GUIDE_START, BASE_GUIDE_OFFSET, BASE_OUTER_HEIGHT / 2.0 + BASE_GUIDE_HEIGHT / 2.0))

    upper_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, BASE_OUTER_HEIGHT / 2.0),
                (0.075, BASE_OUTER_HEIGHT / 2.0 + 0.004),
                (0.0, PLATE_HEIGHT / 2.0 - 0.020),
            ]
        )
        .close()
        .extrude(0.056, both=True)
    )
    lower_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -BASE_OUTER_HEIGHT / 2.0),
                (0.075, -BASE_OUTER_HEIGHT / 2.0 - 0.004),
                (0.0, -PLATE_HEIGHT / 2.0 + 0.020),
            ]
        )
        .close()
        .extrude(0.056, both=True)
    )

    wall_holes = _make_wall_hole_cutter(
        -PLATE_THICKNESS - 0.001,
        PLATE_THICKNESS + 0.002,
        [
            (-0.048, -0.065),
            (0.048, -0.065),
            (-0.048, 0.065),
            (0.048, 0.065),
        ],
        radius=0.0065,
    )

    shape = (
        wall_plate.union(support_beam)
        .union(rear_block)
        .union(left_guide)
        .union(right_guide)
        .union(upper_rib)
        .union(lower_rib)
        .cut(wall_holes)
    )
    return shape


def _make_carriage_shape() -> cq.Workplane:
    plate_z = BASE_OUTER_HEIGHT / 2.0 + BASE_GUIDE_HEIGHT + CARRIAGE_PLATE_THICKNESS / 2.0
    skirt_z = BASE_OUTER_HEIGHT / 2.0 + CARRIAGE_OUTER_HEIGHT / 2.0
    secondary_rail_z = (
        BASE_OUTER_HEIGHT / 2.0
        + BASE_GUIDE_HEIGHT
        + CARRIAGE_PLATE_THICKNESS
        + CARRIAGE_SECONDARY_RAIL_HEIGHT / 2.0
    )

    saddle_plate = _box_from_zero(CARRIAGE_LENGTH, CARRIAGE_OUTER_WIDTH, CARRIAGE_PLATE_THICKNESS).translate(
        (0.0, 0.0, plate_z)
    )
    left_skirt = _box_from_zero(
        CARRIAGE_LENGTH - 0.024,
        CARRIAGE_SKIRT_WIDTH,
        CARRIAGE_OUTER_HEIGHT,
    ).translate((0.012, -(CARRIAGE_OUTER_WIDTH / 2.0 - CARRIAGE_SKIRT_WIDTH / 2.0), skirt_z))
    right_skirt = _box_from_zero(
        CARRIAGE_LENGTH - 0.024,
        CARRIAGE_SKIRT_WIDTH,
        CARRIAGE_OUTER_HEIGHT,
    ).translate((0.012, CARRIAGE_OUTER_WIDTH / 2.0 - CARRIAGE_SKIRT_WIDTH / 2.0, skirt_z))
    secondary_rail = _box_from_zero(
        CARRIAGE_LENGTH - 0.060,
        CARRIAGE_SECONDARY_RAIL_WIDTH,
        CARRIAGE_SECONDARY_RAIL_HEIGHT,
    ).translate((0.030, 0.0, secondary_rail_z))
    return saddle_plate.union(left_skirt).union(right_skirt).union(secondary_rail)


def _make_slider_shape() -> cq.Workplane:
    slider_z = (
        BASE_OUTER_HEIGHT / 2.0
        + BASE_GUIDE_HEIGHT
        + CARRIAGE_PLATE_THICKNESS
        + CARRIAGE_SECONDARY_RAIL_HEIGHT
        + SLIDER_HEIGHT / 2.0
    )
    bar = _box_from_zero(SLIDER_BAR_LENGTH, SLIDER_WIDTH, SLIDER_HEIGHT).translate((0.0, 0.0, slider_z))
    front_plate = cq.Workplane("XY").box(FRONT_PLATE_THICKNESS, FRONT_PLATE_WIDTH, FRONT_PLATE_HEIGHT).translate(
        (SLIDER_BAR_LENGTH + FRONT_PLATE_THICKNESS / 2.0, 0.0, slider_z)
    )
    top_pad = _box_from_zero(0.060, 0.060, 0.010).translate((0.062, 0.0, slider_z + SLIDER_HEIGHT / 2.0 + 0.005))

    face_holes = _make_wall_hole_cutter(
        SLIDER_BAR_LENGTH - 0.001,
        FRONT_PLATE_THICKNESS + 0.002,
        [(-0.026, 0.0), (0.026, 0.0)],
        radius=0.005,
    )

    return bar.union(front_plate).union(top_pad).cut(face_holes)


def _aabb_length(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
    if aabb is None:
        return None
    return float(aabb[1][axis] - aabb[0][axis])


def _x_position(ctx: TestContext, part_name: str) -> float | None:
    pos = ctx.part_world_position(part_name)
    if pos is None:
        return None
    return float(pos[0])


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_double_slide_module")

    model.material("support_gray", rgba=(0.36, 0.39, 0.43, 1.0))
    model.material("stage_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("inner_stage", rgba=(0.80, 0.82, 0.85, 1.0))

    base_support = model.part("base_support")
    _add_box_visual(
        base_support,
        (PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT),
        (-PLATE_THICKNESS / 2.0, 0.0, 0.0),
        "support_gray",
        "wall_plate",
    )
    _add_box_visual(
        base_support,
        (BASE_LENGTH, 0.100, 0.028),
        (BASE_LENGTH / 2.0, 0.0, 0.014),
        "support_gray",
        "base_beam",
    )
    _add_box_visual(
        base_support,
        (0.085, 0.122, 0.034),
        (0.0425, 0.0, 0.017),
        "support_gray",
        "rear_block",
    )
    _add_box_visual(
        base_support,
        (0.440, 0.020, 0.010),
        (0.240, -0.026, 0.033),
        "stage_steel",
        "left_guide",
    )
    _add_box_visual(
        base_support,
        (0.440, 0.020, 0.010),
        (0.240, 0.026, 0.033),
        "stage_steel",
        "right_guide",
    )

    carriage_stage = model.part("carriage_stage")
    _add_box_visual(
        carriage_stage,
        (CARRIAGE_LENGTH, 0.018, 0.010),
        (CARRIAGE_LENGTH / 2.0, -0.026, 0.043),
        "stage_steel",
        "left_runner",
    )
    _add_box_visual(
        carriage_stage,
        (CARRIAGE_LENGTH, 0.018, 0.010),
        (CARRIAGE_LENGTH / 2.0, 0.026, 0.043),
        "stage_steel",
        "right_runner",
    )
    _add_box_visual(
        carriage_stage,
        (0.280, 0.094, 0.008),
        (0.140, 0.0, 0.052),
        "stage_steel",
        "bridge_plate",
    )
    _add_box_visual(
        carriage_stage,
        (0.230, 0.052, 0.012),
        (0.135, 0.0, 0.062),
        "stage_steel",
        "secondary_rail",
    )

    inner_slider = model.part("inner_slider")
    _add_box_visual(
        inner_slider,
        (SLIDER_BAR_LENGTH, 0.046, 0.010),
        (SLIDER_BAR_LENGTH / 2.0, 0.0, 0.073),
        "inner_stage",
        "slider_bar",
    )
    _add_box_visual(
        inner_slider,
        (FRONT_PLATE_THICKNESS, 0.078, 0.030),
        (SLIDER_BAR_LENGTH + FRONT_PLATE_THICKNESS / 2.0, 0.0, 0.083),
        "inner_stage",
        "front_plate",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_support,
        child=carriage_stage,
        origin=Origin(xyz=(OUTER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_slider",
        ArticulationType.PRISMATIC,
        parent=carriage_stage,
        child=inner_slider,
        origin=Origin(xyz=(INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.55,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    carriage_stage = object_model.get_part("carriage_stage")
    inner_slider = object_model.get_part("inner_slider")
    support_to_carriage = object_model.get_articulation("support_to_carriage")
    carriage_to_slider = object_model.get_articulation("carriage_to_slider")
    left_guide = base_support.get_visual("left_guide")
    right_guide = base_support.get_visual("right_guide")
    left_runner = carriage_stage.get_visual("left_runner")
    right_runner = carriage_stage.get_visual("right_runner")
    secondary_rail = carriage_stage.get_visual("secondary_rail")
    slider_bar = inner_slider.get_visual("slider_bar")

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
        "support_to_carriage_is_prismatic_x",
        support_to_carriage.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(v) for v in support_to_carriage.axis) == (1.0, 0.0, 0.0),
        f"expected +X prismatic joint, got type={support_to_carriage.articulation_type} axis={support_to_carriage.axis}",
    )
    ctx.check(
        "carriage_to_slider_is_prismatic_x",
        carriage_to_slider.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(v) for v in carriage_to_slider.axis) == (1.0, 0.0, 0.0),
        f"expected +X prismatic joint, got type={carriage_to_slider.articulation_type} axis={carriage_to_slider.axis}",
    )

    base_length = _aabb_length(ctx.part_world_aabb(base_support), 0)
    slider_length = _aabb_length(ctx.part_world_aabb(inner_slider), 0)
    ctx.check(
        "root_stage_longer_than_inner_stage",
        base_length is not None and slider_length is not None and base_length > slider_length + 0.250,
        f"expected root stage to read longer than inner stage, got base_length={base_length} slider_length={slider_length}",
    )

    with ctx.pose({support_to_carriage: 0.0, carriage_to_slider: 0.0}):
        ctx.expect_contact(
            carriage_stage,
            base_support,
            elem_a=left_runner,
            elem_b=left_guide,
            name="left_runner_supported_by_left_guide",
        )
        ctx.expect_contact(
            carriage_stage,
            base_support,
            elem_a=right_runner,
            elem_b=right_guide,
            name="right_runner_supported_by_right_guide",
        )
        ctx.expect_within(
            inner_slider,
            carriage_stage,
            axes="y",
            inner_elem="slider_bar",
            outer_elem="bridge_plate",
            margin=0.004,
            name="slider_bar_stays_laterally_within_carriage_bridge",
        )
        ctx.expect_contact(
            inner_slider,
            carriage_stage,
            elem_a=slider_bar,
            elem_b=secondary_rail,
            name="slider_bar_seats_on_secondary_rail",
        )

    rest_carriage_x = _x_position(ctx, "carriage_stage")
    rest_slider_x = _x_position(ctx, "inner_slider")
    with ctx.pose({support_to_carriage: OUTER_TRAVEL, carriage_to_slider: 0.0}):
        extended_carriage_x = _x_position(ctx, "carriage_stage")
        carried_slider_x = _x_position(ctx, "inner_slider")
        ctx.expect_within(
            carriage_stage,
            base_support,
            axes="y",
            inner_elem="left_runner",
            outer_elem="left_guide",
            margin=0.003,
            name="left_runner_stays_aligned_at_outer_extension",
        )
        ctx.expect_within(
            carriage_stage,
            base_support,
            axes="y",
            inner_elem="right_runner",
            outer_elem="right_guide",
            margin=0.003,
            name="right_runner_stays_aligned_at_outer_extension",
        )
        ctx.expect_within(
            inner_slider,
            carriage_stage,
            axes="y",
            inner_elem="slider_bar",
            outer_elem="bridge_plate",
            margin=0.004,
            name="slider_bar_stays_laterally_guided_when_carriage_extends",
        )
        ctx.expect_contact(
            inner_slider,
            carriage_stage,
            elem_a=slider_bar,
            elem_b=secondary_rail,
            name="slider_bar_remains_seated_when_carriage_extends",
        )

    ctx.check(
        "outer_stage_translates_by_joint_travel",
        rest_carriage_x is not None
        and extended_carriage_x is not None
        and abs((extended_carriage_x - rest_carriage_x) - OUTER_TRAVEL) < 1e-6,
        f"expected carriage translation {OUTER_TRAVEL}, got rest={rest_carriage_x} extended={extended_carriage_x}",
    )
    ctx.check(
        "inner_stage_is_carried_by_outer_stage",
        rest_slider_x is not None
        and carried_slider_x is not None
        and abs((carried_slider_x - rest_slider_x) - OUTER_TRAVEL) < 1e-6,
        f"expected slider to inherit carriage translation {OUTER_TRAVEL}, got rest={rest_slider_x} carried={carried_slider_x}",
    )

    with ctx.pose({support_to_carriage: OUTER_TRAVEL, carriage_to_slider: INNER_TRAVEL}):
        fully_extended_slider_x = _x_position(ctx, "inner_slider")
        ctx.expect_within(
            inner_slider,
            carriage_stage,
            axes="y",
            inner_elem="slider_bar",
            outer_elem="bridge_plate",
            margin=0.004,
            name="slider_bar_stays_laterally_guided_at_full_extension",
        )
        ctx.expect_contact(
            inner_slider,
            carriage_stage,
            elem_a=slider_bar,
            elem_b=secondary_rail,
            name="slider_bar_stays_seated_at_full_extension",
        )

    ctx.check(
        "inner_stage_adds_its_own_travel_after_being_carried",
        carried_slider_x is not None
        and fully_extended_slider_x is not None
        and abs((fully_extended_slider_x - carried_slider_x) - INNER_TRAVEL) < 1e-6,
        f"expected slider local travel {INNER_TRAVEL}, got carried={carried_slider_x} full={fully_extended_slider_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
