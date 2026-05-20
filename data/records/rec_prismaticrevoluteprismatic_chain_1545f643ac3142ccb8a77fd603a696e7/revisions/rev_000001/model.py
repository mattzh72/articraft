from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.24
BASE_HEIGHT = 0.05
DECK_LENGTH = 0.36
DECK_WIDTH = 0.18
DECK_HEIGHT = 0.015
RAIL_LENGTH = 0.30
RAIL_WIDTH = 0.03
RAIL_HEIGHT = 0.014
RAIL_CENTER_Y = 0.055
RAIL_TOP_Z = BASE_HEIGHT + DECK_HEIGHT + RAIL_HEIGHT

STOP_WIDTH_X = 0.018
STOP_WIDTH_Y = 0.10
STOP_HEIGHT = 0.022
STOP_CENTER_X = 0.135

CARRIAGE_ORIGIN_X = -0.05
CARRIAGE_ORIGIN_Z = 0.112
CARRIAGE_TRAVEL = 0.10
RUNNER_LENGTH = 0.16
RUNNER_WIDTH = 0.032
RUNNER_HEIGHT = 0.012
RUNNER_CENTER_Z = -0.027
CARRIAGE_BODY_LENGTH = 0.15
CARRIAGE_BODY_WIDTH = 0.15
CARRIAGE_BODY_HEIGHT = 0.024
CARRIAGE_BODY_CENTER_Z = -0.012
PIVOT_PAD_SIZE = 0.09
PIVOT_PAD_HEIGHT = 0.008
PIVOT_PAD_CENTER_Z = -0.004
REAR_BLOCK_LENGTH = 0.04
REAR_BLOCK_WIDTH = 0.10
REAR_BLOCK_HEIGHT = 0.028
REAR_BLOCK_CENTER_X = -0.04
REAR_BLOCK_CENTER_Z = -0.014

PIVOT_BASE_SIZE = 0.09
PIVOT_BASE_HEIGHT = 0.014
FRAME_PLATE_LENGTH = 0.16
FRAME_PLATE_THICKNESS = 0.014
FRAME_PLATE_HEIGHT = 0.09
FRAME_PLATE_CENTER_X = 0.08
FRAME_PLATE_CENTER_Y = 0.038
FRAME_PLATE_CENTER_Z = 0.059
REAR_BRACE_LENGTH = 0.022
REAR_BRACE_WIDTH = 0.09
REAR_BRACE_HEIGHT = 0.056
REAR_BRACE_CENTER_X = 0.011
REAR_BRACE_CENTER_Z = 0.042
TOP_TIE_LENGTH = 0.06
TOP_TIE_WIDTH = 0.09
TOP_TIE_HEIGHT = 0.014
TOP_TIE_CENTER_X = 0.04
TOP_TIE_CENTER_Z = 0.097
LOWER_BRACE_LENGTH = 0.07
LOWER_BRACE_THICKNESS = 0.014
LOWER_BRACE_HEIGHT = 0.018
LOWER_BRACE_CENTER_X = 0.055
LOWER_BRACE_CENTER_Y = 0.038
LOWER_BRACE_CENTER_Z = 0.023

GUIDE_REAR_X = 0.08
GUIDE_LENGTH = 0.10
GUIDE_CENTER_X = GUIDE_REAR_X + (GUIDE_LENGTH / 2.0)
GUIDE_OUTER_WIDTH = 0.062
GUIDE_OUTER_HEIGHT = 0.052
GUIDE_WALL = 0.008
GUIDE_CENTER_Z = 0.052
GUIDE_SIDE_CENTER_Y = 0.027
GUIDE_TOP_CENTER_Z = 0.074
GUIDE_BOTTOM_CENTER_Z = 0.030
GUIDE_JOINT_Z = GUIDE_CENTER_Z

PIVOT_SWING = 1.0

SLIDER_JOINT_X = GUIDE_REAR_X
SLIDER_TRAVEL = 0.05
SLIDER_BAR_LENGTH = 0.11
SLIDER_BAR_WIDTH = 0.042
SLIDER_BAR_HEIGHT = 0.032
SLIDER_BAR_CENTER_X = SLIDER_BAR_LENGTH / 2.0
WEAR_PAD_LENGTH = 0.10
WEAR_PAD_WIDTH = 0.024
WEAR_PAD_HEIGHT = 0.002
WEAR_PAD_CENTER_X = WEAR_PAD_LENGTH / 2.0
WEAR_PAD_CENTER_Z = -0.017
TERMINAL_HEAD_LENGTH = 0.022
TERMINAL_HEAD_WIDTH = 0.056
TERMINAL_HEAD_HEIGHT = 0.042
TERMINAL_HEAD_CENTER_X = SLIDER_BAR_LENGTH + (TERMINAL_HEAD_LENGTH / 2.0)


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_slide_link_slide_module")

    model.material("body_paint", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("body_detail", rgba=(0.31, 0.33, 0.36, 1.0))
    model.material("carriage_paint", rgba=(0.53, 0.57, 0.61, 1.0))
    model.material("pivot_frame_paint", rgba=(0.36, 0.43, 0.49, 1.0))
    model.material("slider_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("stop_color", rgba=(0.63, 0.34, 0.10, 1.0))

    body = model.part("body")
    _add_box(
        body,
        "base_block",
        (BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT),
        (0.0, 0.0, BASE_HEIGHT / 2.0),
        "body_paint",
    )
    _add_box(
        body,
        "upper_deck",
        (DECK_LENGTH, DECK_WIDTH, DECK_HEIGHT),
        (0.0, 0.0, BASE_HEIGHT + (DECK_HEIGHT / 2.0)),
        "body_detail",
    )
    _add_box(
        body,
        "rail_pos_y",
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, RAIL_CENTER_Y, BASE_HEIGHT + DECK_HEIGHT + (RAIL_HEIGHT / 2.0)),
        "body_detail",
    )
    _add_box(
        body,
        "rail_neg_y",
        (RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT),
        (0.0, -RAIL_CENTER_Y, BASE_HEIGHT + DECK_HEIGHT + (RAIL_HEIGHT / 2.0)),
        "body_detail",
    )
    _add_box(
        body,
        "front_stop",
        (STOP_WIDTH_X, STOP_WIDTH_Y, STOP_HEIGHT),
        (STOP_CENTER_X, 0.0, BASE_HEIGHT + DECK_HEIGHT + (STOP_HEIGHT / 2.0)),
        "stop_color",
    )
    _add_box(
        body,
        "rear_stop",
        (STOP_WIDTH_X, STOP_WIDTH_Y, STOP_HEIGHT),
        (-STOP_CENTER_X, 0.0, BASE_HEIGHT + DECK_HEIGHT + (STOP_HEIGHT / 2.0)),
        "stop_color",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.08)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        "runner_pos_y",
        (RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT),
        (0.0, RAIL_CENTER_Y, RUNNER_CENTER_Z),
        "carriage_paint",
    )
    _add_box(
        carriage,
        "runner_neg_y",
        (RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT),
        (0.0, -RAIL_CENTER_Y, RUNNER_CENTER_Z),
        "carriage_paint",
    )
    _add_box(
        carriage,
        "carriage_body",
        (CARRIAGE_BODY_LENGTH, CARRIAGE_BODY_WIDTH, CARRIAGE_BODY_HEIGHT),
        (0.0, 0.0, CARRIAGE_BODY_CENTER_Z),
        "carriage_paint",
    )
    _add_box(
        carriage,
        "pivot_pad",
        (PIVOT_PAD_SIZE, PIVOT_PAD_SIZE, PIVOT_PAD_HEIGHT),
        (0.0, 0.0, PIVOT_PAD_CENTER_Z),
        "carriage_paint",
    )
    _add_box(
        carriage,
        "rear_block",
        (REAR_BLOCK_LENGTH, REAR_BLOCK_WIDTH, REAR_BLOCK_HEIGHT),
        (REAR_BLOCK_CENTER_X, 0.0, REAR_BLOCK_CENTER_Z),
        "carriage_paint",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.17, 0.04)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    pivot_frame = model.part("pivot_frame")
    _add_box(
        pivot_frame,
        "pivot_base",
        (PIVOT_BASE_SIZE, PIVOT_BASE_SIZE, PIVOT_BASE_HEIGHT),
        (0.0, 0.0, PIVOT_BASE_HEIGHT / 2.0),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "frame_plate_pos_y",
        (FRAME_PLATE_LENGTH, FRAME_PLATE_THICKNESS, FRAME_PLATE_HEIGHT),
        (FRAME_PLATE_CENTER_X, FRAME_PLATE_CENTER_Y, FRAME_PLATE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "frame_plate_neg_y",
        (FRAME_PLATE_LENGTH, FRAME_PLATE_THICKNESS, FRAME_PLATE_HEIGHT),
        (FRAME_PLATE_CENTER_X, -FRAME_PLATE_CENTER_Y, FRAME_PLATE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "rear_brace",
        (REAR_BRACE_LENGTH, REAR_BRACE_WIDTH, REAR_BRACE_HEIGHT),
        (REAR_BRACE_CENTER_X, 0.0, REAR_BRACE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "top_tie",
        (TOP_TIE_LENGTH, TOP_TIE_WIDTH, TOP_TIE_HEIGHT),
        (TOP_TIE_CENTER_X, 0.0, TOP_TIE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "lower_brace_pos_y",
        (LOWER_BRACE_LENGTH, LOWER_BRACE_THICKNESS, LOWER_BRACE_HEIGHT),
        (LOWER_BRACE_CENTER_X, LOWER_BRACE_CENTER_Y, LOWER_BRACE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "lower_brace_neg_y",
        (LOWER_BRACE_LENGTH, LOWER_BRACE_THICKNESS, LOWER_BRACE_HEIGHT),
        (LOWER_BRACE_CENTER_X, -LOWER_BRACE_CENTER_Y, LOWER_BRACE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "guide_wall_pos_y",
        (GUIDE_LENGTH, GUIDE_WALL, GUIDE_OUTER_HEIGHT),
        (GUIDE_CENTER_X, GUIDE_SIDE_CENTER_Y, GUIDE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "guide_wall_neg_y",
        (GUIDE_LENGTH, GUIDE_WALL, GUIDE_OUTER_HEIGHT),
        (GUIDE_CENTER_X, -GUIDE_SIDE_CENTER_Y, GUIDE_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "guide_wall_top",
        (GUIDE_LENGTH, GUIDE_OUTER_WIDTH, GUIDE_WALL),
        (GUIDE_CENTER_X, 0.0, GUIDE_TOP_CENTER_Z),
        "pivot_frame_paint",
    )
    _add_box(
        pivot_frame,
        "guide_wall_bottom",
        (GUIDE_LENGTH, GUIDE_OUTER_WIDTH, GUIDE_WALL),
        (GUIDE_CENTER_X, 0.0, GUIDE_BOTTOM_CENTER_Z),
        "pivot_frame_paint",
    )
    pivot_frame.inertial = Inertial.from_geometry(
        Box((0.19, 0.10, 0.11)),
        mass=2.4,
        origin=Origin(xyz=(0.095, 0.0, 0.055)),
    )

    terminal_slider = model.part("terminal_slider")
    _add_box(
        terminal_slider,
        "slider_bar",
        (SLIDER_BAR_LENGTH, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT),
        (SLIDER_BAR_CENTER_X, 0.0, 0.0),
        "slider_metal",
    )
    _add_box(
        terminal_slider,
        "wear_pad",
        (WEAR_PAD_LENGTH, WEAR_PAD_WIDTH, WEAR_PAD_HEIGHT),
        (WEAR_PAD_CENTER_X, 0.0, WEAR_PAD_CENTER_Z),
        "slider_metal",
    )
    _add_box(
        terminal_slider,
        "terminal_head",
        (TERMINAL_HEAD_LENGTH, TERMINAL_HEAD_WIDTH, TERMINAL_HEAD_HEIGHT),
        (TERMINAL_HEAD_CENTER_X, 0.0, 0.0),
        "slider_metal",
    )
    terminal_slider.inertial = Inertial.from_geometry(
        Box((SLIDER_BAR_LENGTH + TERMINAL_HEAD_LENGTH, TERMINAL_HEAD_WIDTH, TERMINAL_HEAD_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=((SLIDER_BAR_LENGTH + TERMINAL_HEAD_LENGTH) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_ORIGIN_X, 0.0, CARRIAGE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=180.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_to_pivot_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_frame,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PIVOT_SWING,
            upper=PIVOT_SWING,
            effort=35.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "pivot_to_terminal_slide",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=terminal_slider,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, GUIDE_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDER_TRAVEL,
            effort=50.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    pivot_frame = object_model.get_part("pivot_frame")
    terminal_slider = object_model.get_part("terminal_slider")
    carriage_slide = object_model.get_articulation("body_to_carriage_slide")
    pivot_joint = object_model.get_articulation("carriage_to_pivot_frame")
    terminal_slide = object_model.get_articulation("pivot_to_terminal_slide")

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

    ctx.expect_gap(
        carriage,
        body,
        axis="z",
        positive_elem="runner_pos_y",
        negative_elem="rail_pos_y",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="positive-side carriage runner sits on the positive-side rail",
    )
    ctx.expect_gap(
        carriage,
        body,
        axis="z",
        positive_elem="runner_neg_y",
        negative_elem="rail_neg_y",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="negative-side carriage runner sits on the negative-side rail",
    )
    ctx.expect_overlap(
        carriage,
        body,
        axes="x",
        elem_a="runner_pos_y",
        elem_b="rail_pos_y",
        min_overlap=0.15,
        name="positive-side carriage runner retains long rail engagement",
    )
    ctx.expect_overlap(
        carriage,
        body,
        axes="x",
        elem_a="runner_neg_y",
        elem_b="rail_neg_y",
        min_overlap=0.15,
        name="negative-side carriage runner retains long rail engagement",
    )

    ctx.expect_gap(
        pivot_frame,
        carriage,
        axis="z",
        positive_elem="pivot_base",
        negative_elem="pivot_pad",
        max_gap=0.0005,
        max_penetration=0.0,
        name="pivot frame base seats on the carriage pivot pad",
    )
    ctx.expect_overlap(
        pivot_frame,
        carriage,
        axes="xy",
        elem_a="pivot_base",
        elem_b="pivot_pad",
        min_overlap=0.085,
        name="pivot frame base stays centered on the carriage pivot pad",
    )

    ctx.expect_gap(
        terminal_slider,
        pivot_frame,
        axis="y",
        positive_elem="slider_bar",
        negative_elem="guide_wall_neg_y",
        min_gap=0.0015,
        max_gap=0.0025,
        name="terminal slider clears the negative Y guide wall",
    )
    ctx.expect_gap(
        pivot_frame,
        terminal_slider,
        axis="y",
        positive_elem="guide_wall_pos_y",
        negative_elem="slider_bar",
        min_gap=0.0015,
        max_gap=0.0025,
        name="terminal slider clears the positive Y guide wall",
    )
    ctx.expect_gap(
        terminal_slider,
        pivot_frame,
        axis="z",
        positive_elem="wear_pad",
        negative_elem="guide_wall_bottom",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="terminal slider wear pad rides on the lower guide wall",
    )
    ctx.expect_gap(
        pivot_frame,
        terminal_slider,
        axis="z",
        positive_elem="guide_wall_top",
        negative_elem="slider_bar",
        min_gap=0.0015,
        max_gap=0.0025,
        name="terminal slider clears the upper guide wall",
    )
    ctx.expect_overlap(
        terminal_slider,
        pivot_frame,
        axes="x",
        elem_a="slider_bar",
        elem_b="guide_wall_top",
        min_overlap=0.10,
        name="terminal slider stays deeply inserted at rest",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: CARRIAGE_TRAVEL}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            body,
            axes="x",
            elem_a="runner_pos_y",
            elem_b="rail_pos_y",
            min_overlap=0.15,
            name="positive-side carriage runner stays engaged at full travel",
        )
        ctx.expect_overlap(
            carriage,
            body,
            axes="x",
            elem_a="runner_neg_y",
            elem_b="rail_neg_y",
            min_overlap=0.15,
            name="negative-side carriage runner stays engaged at full travel",
        )
    ctx.check(
        "carriage prismatic joint moves the carriage along +X",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.09,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    with ctx.pose({pivot_joint: 0.0}):
        rest_guide_center = _aabb_center(
            ctx.part_element_world_aabb(pivot_frame, elem="guide_wall_top")
        )
    with ctx.pose({pivot_joint: 0.75}):
        swung_guide_center = _aabb_center(
            ctx.part_element_world_aabb(pivot_frame, elem="guide_wall_top")
        )
    ctx.check(
        "pivot frame revolute joint swings the guide toward +Y for positive motion",
        rest_guide_center is not None
        and swung_guide_center is not None
        and swung_guide_center[1] > rest_guide_center[1] + 0.08,
        details=f"rest={rest_guide_center}, swung={swung_guide_center}",
    )

    rest_slider_pos = ctx.part_world_position(terminal_slider)
    with ctx.pose({terminal_slide: SLIDER_TRAVEL}):
        extended_slider_pos = ctx.part_world_position(terminal_slider)
        ctx.expect_overlap(
            terminal_slider,
            pivot_frame,
            axes="x",
            elem_a="slider_bar",
            elem_b="guide_wall_top",
            min_overlap=0.049,
            name="terminal slider retains insertion at full extension",
        )
    ctx.check(
        "terminal slider prismatic joint extends outward along the frame",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.045,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
