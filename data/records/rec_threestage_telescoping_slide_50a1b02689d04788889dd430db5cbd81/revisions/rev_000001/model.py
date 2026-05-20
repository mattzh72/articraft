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


OUTER_LENGTH = 0.72
MIDDLE_LENGTH = 0.52
INNER_LENGTH = 0.33

OUTER_WIDTH = 0.090
MIDDLE_WIDTH = 0.058
INNER_WIDTH = 0.036

OUTER_HEIGHT = 0.040
MIDDLE_HEIGHT = 0.030
INNER_HEIGHT = 0.020

OUTER_FLOOR = 0.0030
MIDDLE_FLOOR = 0.0025
INNER_FLOOR = 0.0020

OUTER_WALL = 0.0030
MIDDLE_WALL = 0.0025
INNER_WALL = 0.0020

OUTER_LIP_W = 0.012
MIDDLE_LIP_W = 0.009
INNER_LIP_W = 0.006
LIP_T = 0.0016

MIDDLE_RUNNER_W = 0.010
INNER_RUNNER_W = 0.007
MIDDLE_RUNNER_H = 0.0020
INNER_RUNNER_H = 0.0016

MIDDLE_TRAVEL = 0.24
INNER_TRAVEL = 0.22

PLATE_X_OFFSET = 0.075
PLATE_LENGTH = 0.225
PLATE_DEPTH = 0.060
PLATE_WEB = 0.006
PLATE_WEB_DEPTH = 0.020
PLATE_THICKNESS = 0.003
PLATE_WEB_HEIGHT = 0.032
PLATE_FOOT_LENGTH = 0.055
PLATE_FOOT_WIDTH = 0.012
PLATE_FOOT_THICKNESS = 0.0025
PLATE_WEB_X = 0.040
PLATE_FOOT_X = 0.040
PLATE_OUTBOARD_Y = 0.050


def add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_boxed_channel(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall_t: float,
    floor_t: float,
    lip_w: float,
    material,
    runner_w: float = 0.0,
    runner_h: float = 0.0,
):
    add_box(
        part,
        name=f"{prefix}_floor",
        size=(length, width, floor_t),
        xyz=(length / 2.0, 0.0, runner_h + floor_t / 2.0),
        material=material,
    )
    wall_height = height - runner_h
    add_box(
        part,
        name=f"{prefix}_left_wall",
        size=(length, wall_t, wall_height),
        xyz=(length / 2.0, -(width / 2.0 - wall_t / 2.0), runner_h + wall_height / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_wall",
        size=(length, wall_t, wall_height),
        xyz=(length / 2.0, width / 2.0 - wall_t / 2.0, runner_h + wall_height / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_left_lip",
        size=(length, lip_w, LIP_T),
        xyz=(
            length / 2.0,
            -(width / 2.0 - wall_t - lip_w / 2.0),
            height - LIP_T / 2.0,
        ),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_lip",
        size=(length, lip_w, LIP_T),
        xyz=(
            length / 2.0,
            width / 2.0 - wall_t - lip_w / 2.0,
            height - LIP_T / 2.0,
        ),
        material=material,
    )
    if runner_w > 0.0 and runner_h > 0.0:
        runner_y = width / 2.0 - wall_t - runner_w / 2.0
        add_box(
            part,
            name=f"{prefix}_left_runner",
            size=(length, runner_w, runner_h),
            xyz=(length / 2.0, -runner_y, runner_h / 2.0),
            material=material,
        )
        add_box(
            part,
            name=f"{prefix}_right_runner",
            size=(length, runner_w, runner_h),
            xyz=(length / 2.0, runner_y, runner_h / 2.0),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_drawer_slide_module")

    rail_finish = model.material("rail_finish", rgba=(0.68, 0.71, 0.74, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.82, 0.82, 0.80, 1.0))

    outer = model.part("outer_member")
    add_boxed_channel(
        outer,
        prefix="outer",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall_t=OUTER_WALL,
        floor_t=OUTER_FLOOR,
        lip_w=OUTER_LIP_W,
        material=rail_finish,
    )

    middle = model.part("middle_member")
    add_boxed_channel(
        middle,
        prefix="middle",
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall_t=MIDDLE_WALL,
        floor_t=MIDDLE_FLOOR,
        lip_w=MIDDLE_LIP_W,
        material=rail_finish,
        runner_w=MIDDLE_RUNNER_W,
        runner_h=MIDDLE_RUNNER_H,
    )

    inner = model.part("inner_member")
    add_boxed_channel(
        inner,
        prefix="inner",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall_t=INNER_WALL,
        floor_t=INNER_FLOOR,
        lip_w=INNER_LIP_W,
        material=rail_finish,
        runner_w=INNER_RUNNER_W,
        runner_h=INNER_RUNNER_H,
    )

    plate = model.part("mounting_plate")
    add_box(
        plate,
        name="plate_foot",
        size=(PLATE_FOOT_LENGTH, PLATE_FOOT_WIDTH, PLATE_FOOT_THICKNESS),
        xyz=(
            PLATE_FOOT_X,
            INNER_WIDTH / 2.0 - PLATE_FOOT_WIDTH / 2.0,
            PLATE_FOOT_THICKNESS / 2.0,
        ),
        material=plate_finish,
    )
    add_box(
        plate,
        name="plate_web",
        size=(PLATE_WEB, PLATE_WEB_DEPTH, PLATE_WEB_HEIGHT),
        xyz=(
            PLATE_WEB_X,
            INNER_WIDTH / 2.0 - PLATE_FOOT_WIDTH + PLATE_WEB_DEPTH / 2.0,
            PLATE_FOOT_THICKNESS + PLATE_WEB_HEIGHT / 2.0,
        ),
        material=plate_finish,
    )
    add_box(
        plate,
        name="plate_top",
        size=(PLATE_LENGTH, PLATE_DEPTH, PLATE_THICKNESS),
        xyz=(
            PLATE_LENGTH / 2.0,
            PLATE_OUTBOARD_Y,
            PLATE_FOOT_THICKNESS + PLATE_WEB_HEIGHT + PLATE_THICKNESS / 2.0,
        ),
        material=plate_finish,
    )

    outer_to_middle = model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, OUTER_FLOOR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    middle_to_inner = model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_FLOOR)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.45,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_plate",
        ArticulationType.FIXED,
        parent=inner,
        child=plate,
        origin=Origin(xyz=(PLATE_X_OFFSET, 0.0, INNER_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    plate = object_model.get_part("mounting_plate")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_plate = object_model.get_articulation("inner_to_plate")

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
        "parts_present",
        all(part is not None for part in (outer, middle, inner, plate)),
        "Expected outer_member, middle_member, inner_member, and mounting_plate.",
    )
    ctx.check(
        "outer_to_middle_prismatic_axis",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0),
        "outer_to_middle must be a prismatic slide along +X.",
    )
    ctx.check(
        "middle_to_inner_prismatic_axis",
        middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        "middle_to_inner must be a prismatic slide along +X.",
    )
    ctx.check(
        "plate_fixed_to_inner",
        inner_to_plate.articulation_type == ArticulationType.FIXED,
        "mounting_plate should be rigidly mounted to inner_member.",
    )

    ctx.expect_contact(
        middle,
        outer,
        name="middle_supported_by_outer",
    )
    ctx.expect_contact(
        inner,
        middle,
        name="inner_supported_by_middle",
    )
    ctx.expect_contact(
        plate,
        inner,
        elem_a="plate_foot",
        name="plate_foot_attached_to_inner",
    )
    ctx.expect_gap(
        plate,
        inner,
        axis="y",
        positive_elem="plate_top",
        min_gap=0.001,
        name="plate_top_cantilevered_outboard",
    )
    ctx.expect_overlap(
        plate,
        inner,
        axes="x",
        elem_a="plate_top",
        min_overlap=0.20,
        name="plate_top_runs_along_inner_stage",
    )
    ctx.expect_gap(
        plate,
        outer,
        axis="z",
        positive_elem="plate_top",
        min_gap=0.015,
        name="plate_top_clears_outer_rail",
    )

    with ctx.pose({outer_to_middle: 0.18, middle_to_inner: 0.22}):
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=0.179,
            max_gap=0.181,
            name="middle_stage_extends_from_outer",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=0.219,
            max_gap=0.221,
            name="inner_stage_extends_from_middle",
        )
        ctx.expect_contact(
            middle,
            outer,
            name="middle_remains_supported_when_extended",
        )
        ctx.expect_contact(
            inner,
            middle,
            name="inner_remains_supported_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
