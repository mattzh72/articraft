from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RUNNER_LENGTH = 0.74
RUNNER_WIDTH = 0.085
RUNNER_HEIGHT = 0.045
RUNNER_Y_OFFSET = 0.33
RUNNER_Z = 0.11

CROSS_TUBE_LENGTH = 0.58
CROSS_TUBE_DEPTH = 0.18
CROSS_TUBE_HEIGHT = 0.05
CROSS_TUBE_X = -0.08

COLUMN_SPACING = 0.28
COLUMN_X = -0.08
OUTER_COLUMN_SIZE = 0.09
OUTER_COLUMN_HEIGHT = 0.46
INNER_POST_SIZE = 0.056
INNER_POST_LENGTH = 0.46
INNER_POST_TRAVEL = 0.22
INNER_POST_VISIBLE_TOP = 0.35

CARRIAGE_WIDTH = 0.42
CARRIAGE_DEPTH = 0.16
CARRIAGE_HEIGHT = 0.05
HINGE_BARREL_RADIUS = 0.014

TRAY_DEPTH = 0.50
TRAY_WIDTH = 0.88
TRAY_THICKNESS = 0.028

CASTER_RADIUS = 0.04
CASTER_WIDTH = 0.024
CASTER_CHEEK = 0.008
CASTER_X = 0.28

WING_HINGE_X = 0.18
WING_HINGE_Y = (TRAY_WIDTH / 2.0) + 0.06
WING_HINGE_RADIUS = 0.01


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bariatric_overbed_table")

    model.material("powder_steel", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("chrome", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("laminate", rgba=(0.77, 0.70, 0.58, 1.0))
    model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")
    for y_pos in (-RUNNER_Y_OFFSET, RUNNER_Y_OFFSET):
        base.visual(
            Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(0.0, y_pos, RUNNER_Z)),
            material="powder_steel",
            name=f"runner_{0 if y_pos < 0.0 else 1}",
        )
    base.visual(
        Box((CROSS_TUBE_DEPTH, CROSS_TUBE_LENGTH, CROSS_TUBE_HEIGHT)),
        origin=Origin(xyz=(CROSS_TUBE_X, 0.0, RUNNER_Z)),
        material="powder_steel",
        name="cross_tube",
    )
    base.visual(
        Box((0.22, 0.40, 0.025)),
        origin=Origin(
            xyz=(COLUMN_X, 0.0, RUNNER_Z + (RUNNER_HEIGHT + 0.025) / 2.0),
        ),
        material="powder_steel",
        name="column_plate",
    )
    outer_column_center_z = RUNNER_Z + (RUNNER_HEIGHT / 2.0) + (OUTER_COLUMN_HEIGHT / 2.0)
    sleeve_wall = 0.014
    sleeve_inner = OUTER_COLUMN_SIZE - (2.0 * sleeve_wall)
    for index, y_pos in enumerate((-COLUMN_SPACING / 2.0, COLUMN_SPACING / 2.0)):
        base.visual(
            Box((OUTER_COLUMN_SIZE, sleeve_wall, OUTER_COLUMN_HEIGHT)),
            origin=Origin(
                xyz=(COLUMN_X, y_pos - ((OUTER_COLUMN_SIZE - sleeve_wall) / 2.0), outer_column_center_z),
            ),
            material="chrome",
            name=f"outer_column_{index}_front",
        )
        base.visual(
            Box((OUTER_COLUMN_SIZE, sleeve_wall, OUTER_COLUMN_HEIGHT)),
            origin=Origin(
                xyz=(COLUMN_X, y_pos + ((OUTER_COLUMN_SIZE - sleeve_wall) / 2.0), outer_column_center_z),
            ),
            material="chrome",
            name=f"outer_column_{index}_rear",
        )
        base.visual(
            Box((sleeve_wall, sleeve_inner, OUTER_COLUMN_HEIGHT)),
            origin=Origin(
                xyz=(COLUMN_X - ((OUTER_COLUMN_SIZE - sleeve_wall) / 2.0), y_pos, outer_column_center_z),
            ),
            material="chrome",
            name=f"outer_column_{index}_side_0",
        )
        base.visual(
            Box((sleeve_wall, sleeve_inner, OUTER_COLUMN_HEIGHT)),
            origin=Origin(
                xyz=(COLUMN_X + ((OUTER_COLUMN_SIZE - sleeve_wall) / 2.0), y_pos, outer_column_center_z),
            ),
            material="chrome",
            name=f"outer_column_{index}_side_1",
        )
    for x_pos in (-CASTER_X, CASTER_X):
        for y_pos in (-RUNNER_Y_OFFSET, RUNNER_Y_OFFSET):
            suffix = f"{0 if x_pos < 0.0 else 1}_{0 if y_pos < 0.0 else 1}"
            base.visual(
                Box((0.06, 0.04, 0.025)),
                origin=Origin(xyz=(x_pos, y_pos, 0.09625)),
                material="powder_steel",
                name=f"caster_crown_{suffix}",
            )
            base.visual(
                Box((0.06, CASTER_CHEEK, 0.08)),
                origin=Origin(
                    xyz=(x_pos, y_pos - ((CASTER_WIDTH + CASTER_CHEEK) / 2.0), 0.045),
                ),
                material="powder_steel",
                name=f"caster_cheek_{suffix}_0",
            )
            base.visual(
                Box((0.06, CASTER_CHEEK, 0.08)),
                origin=Origin(
                    xyz=(x_pos, y_pos + ((CASTER_WIDTH + CASTER_CHEEK) / 2.0), 0.045),
                ),
                material="powder_steel",
                name=f"caster_cheek_{suffix}_1",
            )

    inner_post_0 = model.part("inner_post_0")
    inner_post_0.visual(
        Box((INNER_POST_SIZE, INNER_POST_SIZE, INNER_POST_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material="chrome",
        name="post_tube",
    )
    inner_post_0.visual(
        Box((0.003, 0.04, 0.11)),
        origin=Origin(xyz=((INNER_POST_SIZE / 2.0) + 0.0015, 0.0, -0.055)),
        material="charcoal",
        name="guide_pad_0",
    )
    inner_post_0.visual(
        Box((0.003, 0.04, 0.11)),
        origin=Origin(xyz=(-(INNER_POST_SIZE / 2.0) - 0.0015, 0.0, -0.055)),
        material="charcoal",
        name="guide_pad_1",
    )
    inner_post_0.visual(
        Box((0.092, 0.11, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material="powder_steel",
        name="post_head",
    )

    inner_post_1 = model.part("inner_post_1")
    inner_post_1.visual(
        Box((INNER_POST_SIZE, INNER_POST_SIZE, INNER_POST_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material="chrome",
        name="post_tube",
    )
    inner_post_1.visual(
        Box((0.003, 0.04, 0.11)),
        origin=Origin(xyz=((INNER_POST_SIZE / 2.0) + 0.0015, 0.0, -0.055)),
        material="charcoal",
        name="guide_pad_0",
    )
    inner_post_1.visual(
        Box((0.003, 0.04, 0.11)),
        origin=Origin(xyz=(-(INNER_POST_SIZE / 2.0) - 0.0015, 0.0, -0.055)),
        material="charcoal",
        name="guide_pad_1",
    )
    inner_post_1.visual(
        Box((0.092, 0.11, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material="powder_steel",
        name="post_head",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_DEPTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
        material="powder_steel",
        name="bridge",
    )
    carriage.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.08),
        origin=Origin(
            xyz=(0.0, -0.08, CARRIAGE_HEIGHT + HINGE_BARREL_RADIUS),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="charcoal",
        name="hinge_barrel_0",
    )
    carriage.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.08),
        origin=Origin(
            xyz=(0.0, 0.08, CARRIAGE_HEIGHT + HINGE_BARREL_RADIUS),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="charcoal",
        name="hinge_barrel_1",
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_DEPTH, TRAY_WIDTH, TRAY_THICKNESS)),
        origin=Origin(xyz=((TRAY_DEPTH / 2.0) + 0.03, 0.0, TRAY_THICKNESS / 2.0)),
        material="laminate",
        name="main_top",
    )
    tray.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="charcoal",
        name="hinge_knuckle",
    )
    tray.visual(
        Box((0.04, 0.08, 0.012)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material="powder_steel",
        name="hinge_strap",
    )
    tray.visual(
        Box((0.08, 0.38, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, -0.02)),
        material="powder_steel",
        name="underframe",
    )
    tray.visual(
        Box((0.02, TRAY_WIDTH, 0.012)),
        origin=Origin(xyz=(TRAY_DEPTH + 0.02, 0.0, TRAY_THICKNESS + 0.006)),
        material="laminate",
        name="front_rim",
    )
    tray.visual(
        Box((TRAY_DEPTH - 0.02, 0.02, 0.012)),
        origin=Origin(
            xyz=((TRAY_DEPTH / 2.0) + 0.03, (TRAY_WIDTH / 2.0) - 0.01, TRAY_THICKNESS + 0.006),
        ),
        material="laminate",
        name="side_rim_0",
    )
    tray.visual(
        Box((TRAY_DEPTH - 0.02, 0.02, 0.012)),
        origin=Origin(
            xyz=((TRAY_DEPTH / 2.0) + 0.03, -(TRAY_WIDTH / 2.0) + 0.01, TRAY_THICKNESS + 0.006),
        ),
        material="laminate",
        name="side_rim_1",
    )
    tray.visual(
        Box((0.18, 0.05, 0.016)),
        origin=Origin(xyz=(WING_HINGE_X, (TRAY_WIDTH / 2.0) + 0.025, -0.008)),
        material="powder_steel",
        name="wing_bracket",
    )
    tray.visual(
        Box((0.10, 0.03, 0.05)),
        origin=Origin(xyz=(WING_HINGE_X, (TRAY_WIDTH / 2.0) + 0.035, -0.025)),
        material="powder_steel",
        name="wing_drop",
    )
    tray.visual(
        Cylinder(radius=WING_HINGE_RADIUS, length=0.05),
        origin=Origin(
            xyz=(WING_HINGE_X - 0.07, WING_HINGE_Y, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="charcoal",
        name="wing_barrel_0",
    )
    tray.visual(
        Cylinder(radius=WING_HINGE_RADIUS, length=0.05),
        origin=Origin(
            xyz=(WING_HINGE_X + 0.07, WING_HINGE_Y, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="charcoal",
        name="wing_barrel_1",
    )
    tray.visual(
        Box((0.08, 0.16, 0.02)),
        origin=Origin(xyz=(0.39, 0.0, -0.01)),
        material="charcoal",
        name="paddle_mount",
    )
    tray.visual(
        Cylinder(radius=0.008, length=0.05),
        origin=Origin(xyz=(0.39, -0.055, -0.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material="charcoal",
        name="paddle_barrel_0",
    )
    tray.visual(
        Cylinder(radius=0.008, length=0.05),
        origin=Origin(xyz=(0.39, 0.055, -0.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material="charcoal",
        name="paddle_barrel_1",
    )
    tray.visual(
        Box((0.02, 0.02, 0.02)),
        origin=Origin(xyz=(0.39, -0.055, -0.02)),
        material="charcoal",
        name="paddle_lug_0",
    )
    tray.visual(
        Box((0.02, 0.02, 0.02)),
        origin=Origin(xyz=(0.39, 0.055, -0.02)),
        material="charcoal",
        name="paddle_lug_1",
    )

    reading_wing = model.part("reading_wing")
    reading_wing.visual(
        Box((0.28, 0.24, 0.022)),
        origin=Origin(xyz=(0.0, 0.13, 0.011)),
        material="laminate",
        name="wing_panel",
    )
    reading_wing.visual(
        Cylinder(radius=WING_HINGE_RADIUS, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="charcoal",
        name="wing_knuckle",
    )
    reading_wing.visual(
        Box((0.09, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
        material="powder_steel",
        name="wing_strap",
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.008, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="charcoal",
        name="paddle_knuckle",
    )
    release_paddle.visual(
        Box((0.028, 0.06, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, -0.012)),
        material="charcoal",
        name="paddle_stem",
    )
    release_paddle.visual(
        Box((0.11, 0.055, 0.09)),
        origin=Origin(xyz=(0.055, 0.0, -0.055)),
        material="charcoal",
        name="paddle_blade",
    )

    caster_points = [
        (-CASTER_X, -RUNNER_Y_OFFSET),
        (CASTER_X, -RUNNER_Y_OFFSET),
        (-CASTER_X, RUNNER_Y_OFFSET),
        (CASTER_X, RUNNER_Y_OFFSET),
    ]
    caster_parts = []
    for index, (x_pos, y_pos) in enumerate(caster_points):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=CASTER_RADIUS, length=CASTER_WIDTH),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="charcoal",
            name="wheel_tire",
        )
        caster.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="powder_steel",
            name="wheel_hub",
        )
        caster_parts.append((caster, x_pos, y_pos))

    column_top_z = RUNNER_Z + (RUNNER_HEIGHT / 2.0) + OUTER_COLUMN_HEIGHT
    lift_limits = MotionLimits(
        effort=600.0,
        velocity=0.18,
        lower=0.0,
        upper=INNER_POST_TRAVEL,
    )
    model.articulation(
        "base_to_inner_post_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_post_0,
        origin=Origin(xyz=(COLUMN_X, -COLUMN_SPACING / 2.0, column_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_inner_post_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_post_1,
        origin=Origin(xyz=(COLUMN_X, COLUMN_SPACING / 2.0, column_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.18,
            lower=0.0,
            upper=INNER_POST_TRAVEL,
        ),
        mimic=Mimic("base_to_inner_post_0"),
    )
    model.articulation(
        "inner_post_0_to_carriage",
        ArticulationType.FIXED,
        parent=inner_post_0,
        child=carriage,
        origin=Origin(xyz=(0.0, COLUMN_SPACING / 2.0, INNER_POST_VISIBLE_TOP)),
    )
    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT + HINGE_BARREL_RADIUS)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.1,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "tray_to_reading_wing",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=reading_wing,
        origin=Origin(xyz=(WING_HINGE_X, WING_HINGE_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "tray_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release_paddle,
        origin=Origin(xyz=(0.39, 0.0, -0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=0.7,
        ),
    )
    for index, (caster, x_pos, y_pos) in enumerate(caster_parts):
        model.articulation(
            f"base_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, CASTER_RADIUS)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=20.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    inner_post_0 = object_model.get_part("inner_post_0")
    inner_post_1 = object_model.get_part("inner_post_1")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    reading_wing = object_model.get_part("reading_wing")
    release_paddle = object_model.get_part("release_paddle")

    lift = object_model.get_articulation("base_to_inner_post_0")
    tray_tilt = object_model.get_articulation("carriage_to_tray")
    wing_hinge = object_model.get_articulation("tray_to_reading_wing")

    ctx.expect_contact(
        inner_post_1,
        carriage,
        elem_a="post_head",
        elem_b="bridge",
        name="right lift post supports the carriage at rest",
    )
    ctx.expect_contact(
        tray,
        carriage,
        elem_a="hinge_knuckle",
        elem_b="hinge_barrel_0",
        name="tray hinge knuckle bears on the carriage barrel",
    )
    ctx.expect_gap(
        tray,
        release_paddle,
        axis="z",
        positive_elem="main_top",
        negative_elem="paddle_blade",
        min_gap=0.03,
        name="release paddle sits below the work surface",
    )

    rest_left = ctx.part_world_position(inner_post_0)
    rest_right = ctx.part_world_position(inner_post_1)
    lift_upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    if lift_upper is not None:
        with ctx.pose({lift: lift_upper}):
            ctx.expect_contact(
                inner_post_1,
                carriage,
                elem_a="post_head",
                elem_b="bridge",
                name="matched lift posts stay aligned at full raise",
            )
            raised_left = ctx.part_world_position(inner_post_0)
            raised_right = ctx.part_world_position(inner_post_1)
        ctx.check(
            "twin posts rise together",
            rest_left is not None
            and rest_right is not None
            and raised_left is not None
            and raised_right is not None
            and raised_left[2] > rest_left[2] + 0.15
            and raised_right[2] > rest_right[2] + 0.15
            and abs((raised_left[2] - rest_left[2]) - (raised_right[2] - rest_right[2])) < 1e-6,
            details=(
                f"rest_left={rest_left}, rest_right={rest_right}, "
                f"raised_left={raised_left}, raised_right={raised_right}"
            ),
        )

    tilt_upper = tray_tilt.motion_limits.upper if tray_tilt.motion_limits is not None else None
    if tilt_upper is not None:
        with ctx.pose({tray_tilt: tilt_upper}):
            ctx.expect_gap(
                tray,
                carriage,
                axis="z",
                positive_elem="main_top",
                negative_elem="bridge",
                min_gap=0.03,
                name="meal tray lifts well above the carriage when tilted",
            )

    wing_upper = wing_hinge.motion_limits.upper if wing_hinge.motion_limits is not None else None
    if wing_upper is not None:
        wing_rest = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
        with ctx.pose({wing_hinge: wing_upper}):
            wing_open = ctx.part_element_world_aabb(reading_wing, elem="wing_panel")
        ctx.check(
            "reading wing rotates upward beside the tray",
            wing_rest is not None
            and wing_open is not None
            and wing_open[1][2] > wing_rest[1][2] + 0.12,
            details=f"wing_rest={wing_rest}, wing_open={wing_open}",
        )

    caster_checks = []
    for index in range(4):
        joint = object_model.get_articulation(f"base_to_caster_{index}")
        caster = object_model.get_part(f"caster_{index}")
        joint_ok = (
            str(joint.articulation_type).endswith("CONTINUOUS")
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
        )
        caster_checks.append(joint_ok)
        base_aabb = ctx.part_world_aabb(base)
        caster_aabb = ctx.part_world_aabb(caster)
        runner_bottom = RUNNER_Z - (RUNNER_HEIGHT / 2.0)
        ctx.check(
            f"caster_{index} hangs below the H base",
            base_aabb is not None
            and caster_aabb is not None
            and caster_aabb[1][2] < runner_bottom - 0.002,
            details=(
                f"runner_bottom={runner_bottom}, base_aabb={base_aabb}, "
                f"caster_aabb={caster_aabb}"
            ),
        )
    ctx.check(
        "all four casters are continuous wheel joints",
        all(caster_checks),
        details=f"caster_checks={caster_checks}",
    )

    return ctx.report()


object_model = build_object_model()
