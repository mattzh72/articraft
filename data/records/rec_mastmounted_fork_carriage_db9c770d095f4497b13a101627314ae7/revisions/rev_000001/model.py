from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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


POST_X = -0.07
POST_HALF_SPACING = 0.24
POST_RADIUS = 0.035
POST_BASE_Z = 0.085
POST_HEIGHT = 1.37

BASE_REAR_DEPTH = 0.30
BASE_REAR_WIDTH = 0.62
BASE_REAR_HEIGHT = 0.11
BASE_REAR_CENTER_X = -0.09

BASE_LEG_LENGTH = 0.68
BASE_LEG_WIDTH = 0.12
BASE_LEG_HEIGHT = 0.08
BASE_LEG_CENTER_X = 0.36
BASE_LEG_CENTER_Y = 0.22
FRONT_TIE_DEPTH = 0.06
FRONT_TIE_WIDTH = 0.56
FRONT_TIE_CENTER_X = 0.67

LIFT_HOME_Z = 0.18
LIFT_TRAVEL = 0.74

GUIDE_OUTER_RADIUS = 0.057
GUIDE_INNER_RADIUS = 0.041
GUIDE_LENGTH = 0.44

SLIDE_RAIL_CENTER_X = -0.05
SLIDE_RAIL_THICKNESS = 0.04
SLIDE_RAIL_WIDTH = 0.08
SLIDE_RAIL_HEIGHT = 1.05

SLIDER_SHOE_CENTER_X = 0.04
SLIDER_SHOE_LENGTH = 0.08
SLIDER_SHOE_WIDTH = 0.02
SLIDER_SHOE_HEIGHT = 0.34
SLIDER_SHOE_CENTER_Y = 0.05
SLIDER_SHOE_BASE_Z = 0.04

PLATE_THICKNESS = 0.03
PLATE_BACK_X = 0.046
PLATE_WIDTH = 0.52
PLATE_HEIGHT = 0.42

BACKREST_HEIGHT = 0.52
BACKREST_TOP_Z = 0.94
BACKREST_UPRIGHT_WIDTH = 0.03
BACKREST_UPRIGHT_DEPTH = 0.04
BACKREST_HALF_SPACING = 0.215

LOWER_RAIL_LENGTH = 0.09
LOWER_RAIL_HEIGHT = 0.08
LOWER_RAIL_CENTER_X = 0.11

FORK_CENTER_Y = 0.16
FORK_LENGTH = 0.62
FORK_WIDTH = 0.09
FORK_THICKNESS = 0.04
FORK_START_X = 0.10
FORK_HEEL_LENGTH = 0.10
FORK_HEEL_WIDTH = 0.03
FORK_HEEL_HEIGHT = 0.14


def _box(length: float, width: float, height: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(True, True, False),
    ).translate(center_xyz)


def _make_base_frame() -> cq.Workplane:
    rear_block = _box(
        BASE_REAR_DEPTH,
        BASE_REAR_WIDTH,
        BASE_REAR_HEIGHT,
        (BASE_REAR_CENTER_X, 0.0, 0.0),
    )
    left_leg = _box(
        BASE_LEG_LENGTH,
        BASE_LEG_WIDTH,
        BASE_LEG_HEIGHT,
        (BASE_LEG_CENTER_X, BASE_LEG_CENTER_Y, 0.0),
    )
    right_leg = _box(
        BASE_LEG_LENGTH,
        BASE_LEG_WIDTH,
        BASE_LEG_HEIGHT,
        (BASE_LEG_CENTER_X, -BASE_LEG_CENTER_Y, 0.0),
    )
    front_tie = _box(
        FRONT_TIE_DEPTH,
        FRONT_TIE_WIDTH,
        BASE_LEG_HEIGHT,
        (FRONT_TIE_CENTER_X, 0.0, 0.0),
    )
    deck = _box(
        0.22,
        0.34,
        0.03,
        (-0.01, 0.0, BASE_REAR_HEIGHT),
    )
    return rear_block.union(left_leg).union(right_leg).union(front_tie).union(deck)


def _make_guide_posts() -> cq.Workplane:
    posts = None
    for y_center in (-POST_HALF_SPACING, POST_HALF_SPACING):
        collar = (
            cq.Workplane("XY")
            .center(POST_X, y_center)
            .circle(POST_RADIUS + 0.010)
            .extrude(0.035)
            .translate((0.0, 0.0, POST_BASE_Z - 0.020))
        )
        post = (
            cq.Workplane("XY")
            .center(POST_X, y_center)
            .circle(POST_RADIUS)
            .extrude(POST_HEIGHT)
            .translate((0.0, 0.0, POST_BASE_Z))
        )
        post_set = collar.union(post)
        posts = post_set if posts is None else posts.union(post_set)

    top_tie = _box(
        0.10,
        0.58,
        0.05,
        (POST_X - 0.02, 0.0, POST_BASE_Z + POST_HEIGHT - 0.025),
    )
    return posts.union(top_tie)


def _make_lift_rail() -> cq.Workplane:
    return _box(
        SLIDE_RAIL_THICKNESS,
        SLIDE_RAIL_WIDTH,
        SLIDE_RAIL_HEIGHT,
        (SLIDE_RAIL_CENTER_X, 0.0, BASE_REAR_HEIGHT),
    )


def _make_carriage_guides() -> cq.Workplane:
    guides = None
    for y_center in (-POST_HALF_SPACING, POST_HALF_SPACING):
        outer = cq.Workplane("XY").center(0.0, y_center).circle(GUIDE_OUTER_RADIUS).extrude(GUIDE_LENGTH)
        inner = cq.Workplane("XY").center(0.0, y_center).circle(GUIDE_INNER_RADIUS).extrude(GUIDE_LENGTH)
        sleeve = outer.cut(inner)
        guides = sleeve if guides is None else guides.union(sleeve)
    return guides


def _make_carriage_body() -> cq.Workplane:
    plate_center_x = PLATE_BACK_X + (PLATE_THICKNESS / 2.0)
    lower_rail = _box(
        LOWER_RAIL_LENGTH,
        PLATE_WIDTH,
        LOWER_RAIL_HEIGHT,
        (LOWER_RAIL_CENTER_X, 0.0, 0.0),
    )
    plate = _box(
        PLATE_THICKNESS,
        PLATE_WIDTH,
        PLATE_HEIGHT,
        (plate_center_x, 0.0, 0.0),
    )
    left_upright = _box(
        BACKREST_UPRIGHT_DEPTH,
        BACKREST_UPRIGHT_WIDTH,
        BACKREST_HEIGHT,
        (plate_center_x, -BACKREST_HALF_SPACING, PLATE_HEIGHT - 0.02),
    )
    right_upright = _box(
        BACKREST_UPRIGHT_DEPTH,
        BACKREST_UPRIGHT_WIDTH,
        BACKREST_HEIGHT,
        (plate_center_x, BACKREST_HALF_SPACING, PLATE_HEIGHT - 0.02),
    )
    top_bar = _box(
        BACKREST_UPRIGHT_DEPTH,
        0.48,
        0.04,
        (plate_center_x, 0.0, BACKREST_TOP_Z - 0.04),
    )
    mid_bar = _box(
        BACKREST_UPRIGHT_DEPTH,
        0.44,
        0.03,
        (plate_center_x, 0.0, 0.68),
    )
    return plate.union(lower_rail).union(left_upright).union(right_upright).union(top_bar).union(mid_bar)


def _make_slider_shoes() -> cq.Workplane:
    left_shoe = _box(
        SLIDER_SHOE_LENGTH,
        SLIDER_SHOE_WIDTH,
        SLIDER_SHOE_HEIGHT,
        (SLIDER_SHOE_CENTER_X, -SLIDER_SHOE_CENTER_Y, SLIDER_SHOE_BASE_Z),
    )
    right_shoe = _box(
        SLIDER_SHOE_LENGTH,
        SLIDER_SHOE_WIDTH,
        SLIDER_SHOE_HEIGHT,
        (SLIDER_SHOE_CENTER_X, SLIDER_SHOE_CENTER_Y, SLIDER_SHOE_BASE_Z),
    )
    return left_shoe.union(right_shoe)


def _make_forks() -> cq.Workplane:
    forks = None
    tine_center_x = FORK_START_X + (FORK_LENGTH / 2.0)
    heel_center_x = FORK_START_X + 0.02
    for y_center in (-FORK_CENTER_Y, FORK_CENTER_Y):
        tine = _box(
            FORK_LENGTH,
            FORK_WIDTH,
            FORK_THICKNESS,
            (tine_center_x, y_center, 0.0),
        )
        heel = _box(
            FORK_HEEL_LENGTH,
            FORK_HEEL_WIDTH,
            FORK_HEEL_HEIGHT,
            (heel_center_x, y_center, 0.0),
        )
        fork = tine.union(heel)
        forks = fork if forks is None else forks.union(fork)
    return forks


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_post_fork_carriage_lift")

    model.material("base_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("steel_post", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_orange", rgba=(0.93, 0.49, 0.11, 1.0))
    model.material("fork_steel", rgba=(0.17, 0.18, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_frame(), "base_frame"),
        material="base_gray",
        name="base_frame",
    )
    base.visual(
        mesh_from_cadquery(_make_guide_posts(), "guide_posts"),
        material="steel_post",
        name="guide_posts",
    )
    base.visual(
        mesh_from_cadquery(_make_lift_rail(), "lift_rail"),
        material="base_gray",
        name="lift_rail",
    )
    base.inertial = None

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_guides(), "carriage_guides"),
        material="steel_post",
        name="carriage_guides",
    )
    carriage.visual(
        mesh_from_cadquery(_make_carriage_body(), "carriage_body"),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(_make_slider_shoes(), "slider_shoes"),
        material="base_gray",
        name="slider_shoes",
    )
    carriage.visual(
        mesh_from_cadquery(_make_forks(), "forks"),
        material="fork_steel",
        name="forks",
    )
    carriage.inertial = None

    model.articulation(
        "base_to_carriage_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(POST_X, 0.0, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=1800.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("base_to_carriage_lift")

    limits = lift.motion_limits
    upper = 0.0 if limits is None or limits.upper is None else limits.upper

    ctx.check(
        "parts and lift joint exist",
        base is not None and carriage is not None and lift is not None,
        details=f"base={base}, carriage={carriage}, lift={lift}",
    )
    ctx.check(
        "lift joint uses vertical prismatic motion",
        lift.articulation_type == ArticulationType.PRISMATIC and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="forks",
        negative_elem="base_frame",
        min_gap=0.038,
        max_gap=0.090,
        name="forks sit just above the grounded base at rest",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="carriage_guides",
        elem_b="guide_posts",
        min_overlap=0.10,
        name="guide sleeves stay centered on the twin posts at rest",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="z",
        elem_a="carriage_guides",
        elem_b="guide_posts",
        min_overlap=0.30,
        name="guide sleeves have substantial post engagement at rest",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="slider_shoes",
        elem_b="lift_rail",
        name="hidden slider shoes bear on the lift rail at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: upper}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="slider_shoes",
            elem_b="lift_rail",
            name="hidden slider shoes stay on the lift rail at full lift",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            elem_a="carriage_guides",
            elem_b="guide_posts",
            min_overlap=0.10,
            name="guide sleeves remain aligned on the posts at full lift",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="z",
            elem_a="carriage_guides",
            elem_b="guide_posts",
            min_overlap=0.12,
            name="guide sleeves retain insertion on the posts at full lift",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage raises upward along the posts",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + (0.90 * upper),
        details=f"rest={rest_pos}, raised={raised_pos}, upper={upper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
