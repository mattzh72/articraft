from __future__ import annotations

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


BASE_RAIL_X = 0.34
BASE_RAIL_LENGTH = 0.54
BASE_RAIL_WIDTH = 0.08
BASE_MEMBER_HEIGHT = 0.045
BASE_MEMBER_Z = 0.1305
BASE_TOP_Z = BASE_MEMBER_Z + (BASE_MEMBER_HEIGHT / 2.0)
CASTER_Y = 0.225
WHEEL_RADIUS = 0.047
WHEEL_WIDTH = 0.040

COLUMN_Y = -0.205
POST_HALF_SPACING = 0.23
POST_SPACING = POST_HALF_SPACING * 2.0
OUTER_COLUMN_OUTER = 0.070
OUTER_COLUMN_WALL = 0.008
OUTER_COLUMN_HEIGHT = 0.450
COLUMN_TOP_Z = BASE_TOP_Z + OUTER_COLUMN_HEIGHT

INNER_POST_SIZE = 0.046
INNER_POST_LENGTH = 0.560
INNER_POST_CENTER_Z = -0.020
INNER_POST_TOP_Z = INNER_POST_CENTER_Z + (INNER_POST_LENGTH / 2.0)
LIFT_TRAVEL = 0.220
POST_GUIDE_THICKNESS = 0.004
POST_GUIDE_DEPTH = 0.020
POST_GUIDE_LENGTH = 0.300
POST_GUIDE_CENTER_Z = -0.150

TRAY_WIDTH = 1.080
TRAY_DEPTH = 0.560
TRAY_THICKNESS = 0.026
TRAY_LIP_THICKNESS = 0.020
TRAY_FRONT_RIM_DEPTH = 0.030
TRAY_FRONT_RIM_HEIGHT = 0.030


def _box(
    part,
    size,
    xyz,
    *,
    material: str,
    name: str | None = None,
    rpy=(0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder(
    part,
    radius,
    length,
    xyz,
    *,
    material: str,
    name: str | None = None,
    rpy=(0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_caster_mount(base, x_pos: float, y_pos: float, prefix: str) -> None:
    _box(
        base,
        (0.074, 0.060, 0.010),
        (x_pos, y_pos, 0.103),
        material="frame_gray",
        name=f"{prefix}_fork_cap",
    )
    side_offset = (WHEEL_WIDTH / 2.0) + 0.008
    for idx, sign in enumerate((-1.0, 1.0)):
        _box(
            base,
            (0.012, 0.060, 0.095),
            (x_pos + (sign * side_offset), y_pos, 0.0515),
            material="frame_gray",
            name=f"{prefix}_fork_side_{idx}",
        )


def _add_outer_column_shell(base, x_pos: float, prefix: str) -> None:
    shell_z = BASE_TOP_Z + (OUTER_COLUMN_HEIGHT / 2.0)
    side_x = (OUTER_COLUMN_OUTER / 2.0) - (OUTER_COLUMN_WALL / 2.0)
    side_y = (OUTER_COLUMN_OUTER / 2.0) - (OUTER_COLUMN_WALL / 2.0)
    inner_span = OUTER_COLUMN_OUTER - (2.0 * OUTER_COLUMN_WALL)

    for idx, sign in enumerate((-1.0, 1.0)):
        _box(
            base,
            (OUTER_COLUMN_WALL, OUTER_COLUMN_OUTER, OUTER_COLUMN_HEIGHT),
            (x_pos + (sign * side_x), COLUMN_Y, shell_z),
            material="frame_gray",
            name=f"{prefix}_side_{idx}",
        )
        _box(
            base,
            (inner_span, OUTER_COLUMN_WALL, OUTER_COLUMN_HEIGHT),
            (x_pos, COLUMN_Y + (sign * side_y), shell_z),
            material="frame_gray",
            name=f"{prefix}_face_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bariatric_overbed_table")

    model.material("frame_gray", rgba=(0.41, 0.44, 0.47, 1.0))
    model.material("lift_silver", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("tray_beige", rgba=(0.86, 0.84, 0.78, 1.0))
    model.material("clip_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("control_black", rgba=(0.17, 0.17, 0.18, 1.0))

    base = model.part("base")
    _box(
        base,
        (BASE_RAIL_WIDTH, BASE_RAIL_LENGTH, BASE_MEMBER_HEIGHT),
        (-BASE_RAIL_X, 0.0, BASE_MEMBER_Z),
        material="frame_gray",
        name="side_rail_0",
    )
    _box(
        base,
        (BASE_RAIL_WIDTH, BASE_RAIL_LENGTH, BASE_MEMBER_HEIGHT),
        (BASE_RAIL_X, 0.0, BASE_MEMBER_Z),
        material="frame_gray",
        name="side_rail_1",
    )
    _box(
        base,
        (0.760, 0.080, BASE_MEMBER_HEIGHT),
        (0.0, 0.0, BASE_MEMBER_Z),
        material="frame_gray",
        name="center_bridge",
    )
    _box(
        base,
        (0.120, 0.380, BASE_MEMBER_HEIGHT),
        (0.0, -0.110, BASE_MEMBER_Z),
        material="frame_gray",
        name="center_spine",
    )
    _box(
        base,
        (0.560, 0.110, BASE_MEMBER_HEIGHT),
        (0.0, -0.245, BASE_MEMBER_Z),
        material="frame_gray",
        name="column_beam",
    )
    _box(
        base,
        (0.600, 0.110, 0.022),
        (0.0, COLUMN_Y, BASE_TOP_Z + 0.011),
        material="frame_gray",
        name="column_plate",
    )

    for caster_idx, (x_pos, y_pos) in enumerate(
        (
            (-BASE_RAIL_X, -CASTER_Y),
            (-BASE_RAIL_X, CASTER_Y),
            (BASE_RAIL_X, -CASTER_Y),
            (BASE_RAIL_X, CASTER_Y),
        )
    ):
        _add_caster_mount(base, x_pos, y_pos, f"caster_mount_{caster_idx}")

    _add_outer_column_shell(base, -POST_HALF_SPACING, "outer_column_0")
    _add_outer_column_shell(base, POST_HALF_SPACING, "outer_column_1")

    lift_post_0 = model.part("lift_post_0")
    _box(
        lift_post_0,
        (INNER_POST_SIZE, INNER_POST_SIZE, INNER_POST_LENGTH),
        (0.0, 0.0, INNER_POST_CENTER_Z),
        material="lift_silver",
        name="mast",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        _box(
            lift_post_0,
            (POST_GUIDE_THICKNESS, POST_GUIDE_DEPTH, POST_GUIDE_LENGTH),
            (sign * ((INNER_POST_SIZE / 2.0) + (POST_GUIDE_THICKNESS / 2.0)), 0.0, POST_GUIDE_CENTER_Z),
            material="lift_silver",
            name=f"guide_{idx}",
        )

    lift_post_1 = model.part("lift_post_1")
    _box(
        lift_post_1,
        (INNER_POST_SIZE, INNER_POST_SIZE, INNER_POST_LENGTH),
        (0.0, 0.0, INNER_POST_CENTER_Z),
        material="lift_silver",
        name="mast",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        _box(
            lift_post_1,
            (POST_GUIDE_THICKNESS, POST_GUIDE_DEPTH, POST_GUIDE_LENGTH),
            (sign * ((INNER_POST_SIZE / 2.0) + (POST_GUIDE_THICKNESS / 2.0)), 0.0, POST_GUIDE_CENTER_Z),
            material="lift_silver",
            name=f"guide_{idx}",
        )

    carriage = model.part("carriage")
    _box(
        carriage,
        (POST_SPACING + 0.100, 0.080, 0.032),
        (POST_SPACING / 2.0, 0.0, 0.016),
        material="lift_silver",
        name="crossbeam",
    )
    _box(
        carriage,
        (0.084, 0.092, 0.030),
        (0.0, 0.0, 0.015),
        material="lift_silver",
        name="saddle_0",
    )
    _box(
        carriage,
        (0.084, 0.092, 0.030),
        (POST_SPACING, 0.0, 0.015),
        material="lift_silver",
        name="saddle_1",
    )
    _box(
        carriage,
        (POST_SPACING + 0.140, 0.180, 0.018),
        (POST_SPACING / 2.0, 0.060, 0.041),
        material="lift_silver",
        name="top_carriage_plate",
    )
    _box(
        carriage,
        (0.100, 0.050, 0.028),
        (POST_SPACING * 0.76, 0.118, 0.018),
        material="frame_gray",
        name="release_box",
    )

    release_paddle = model.part("release_paddle")
    _cylinder(
        release_paddle,
        0.008,
        0.070,
        (0.0, 0.0, 0.0),
        material="control_black",
        name="pivot",
        rpy=(0.0, 1.5707963267948966, 0.0),
    )
    _box(
        release_paddle,
        (0.110, 0.018, 0.058),
        (0.0, 0.013, -0.029),
        material="control_black",
        name="paddle",
    )

    tray = model.part("tray")
    _box(
        tray,
        (TRAY_WIDTH, TRAY_DEPTH, TRAY_THICKNESS),
        (0.0, TRAY_DEPTH / 2.0, TRAY_THICKNESS / 2.0),
        material="tray_beige",
        name="deck",
    )
    _box(
        tray,
        (TRAY_LIP_THICKNESS, TRAY_DEPTH, TRAY_LIP_THICKNESS),
        (-(TRAY_WIDTH / 2.0) + (TRAY_LIP_THICKNESS / 2.0), TRAY_DEPTH / 2.0, 0.036),
        material="tray_beige",
        name="side_rim_0",
    )
    _box(
        tray,
        (TRAY_LIP_THICKNESS, TRAY_DEPTH, TRAY_LIP_THICKNESS),
        ((TRAY_WIDTH / 2.0) - (TRAY_LIP_THICKNESS / 2.0), TRAY_DEPTH / 2.0, 0.036),
        material="tray_beige",
        name="side_rim_1",
    )
    _box(
        tray,
        (TRAY_WIDTH - (2.0 * TRAY_LIP_THICKNESS), TRAY_LIP_THICKNESS, TRAY_LIP_THICKNESS),
        (0.0, TRAY_LIP_THICKNESS / 2.0, 0.036),
        material="tray_beige",
        name="rear_rim",
    )
    _box(
        tray,
        (TRAY_WIDTH, TRAY_FRONT_RIM_DEPTH, TRAY_FRONT_RIM_HEIGHT),
        (0.0, TRAY_DEPTH - (TRAY_FRONT_RIM_DEPTH / 2.0), TRAY_FRONT_RIM_HEIGHT / 2.0),
        material="tray_beige",
        name="front_rim",
    )

    clip_bar = model.part("clip_bar")
    _box(
        clip_bar,
        (0.820, 0.010, 0.010),
        (0.0, 0.0, 0.005),
        material="clip_silver",
        name="hinge_rail",
    )
    for idx, x_pos in enumerate((-0.300, 0.300)):
        _box(
            clip_bar,
            (0.030, 0.030, 0.050),
            (x_pos, 0.015, 0.030),
            material="clip_silver",
            name=f"support_{idx}",
        )
    _box(
        clip_bar,
        (0.900, 0.020, 0.012),
        (0.0, 0.032, 0.058),
        material="clip_silver",
        name="bar",
    )

    for caster_idx in range(4):
        caster = model.part(f"caster_{caster_idx}")
        _cylinder(
            caster,
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            (0.0, 0.0, 0.0),
            material="rubber_black",
            name="wheel",
            rpy=(0.0, 1.5707963267948966, 0.0),
        )
        _cylinder(
            caster,
            0.016,
            0.044,
            (0.0, 0.0, 0.0),
            material="frame_gray",
            name="hub",
            rpy=(0.0, 1.5707963267948966, 0.0),
        )

    lift_joint_0 = model.articulation(
        "base_to_lift_post_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_post_0,
        origin=Origin(xyz=(-POST_HALF_SPACING, COLUMN_Y, COLUMN_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=250.0, velocity=0.25),
    )
    model.articulation(
        "base_to_lift_post_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_post_1,
        origin=Origin(xyz=(POST_HALF_SPACING, COLUMN_Y, COLUMN_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=250.0, velocity=0.25),
        mimic=Mimic(joint=lift_joint_0.name),
    )
    model.articulation(
        "lift_post_0_to_carriage",
        ArticulationType.FIXED,
        parent=lift_post_0,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, INNER_POST_TOP_Z)),
    )
    model.articulation(
        "carriage_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=release_paddle,
        origin=Origin(xyz=(POST_SPACING * 0.76, 0.118, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=0.35, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(POST_SPACING / 2.0, -0.010, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.10, upper=0.95, effort=18.0, velocity=1.2),
    )
    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(0.0, TRAY_DEPTH - 0.018, 0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=4.0, velocity=2.2),
    )

    for caster_idx, (x_pos, y_pos) in enumerate(
        (
            (-BASE_RAIL_X, -CASTER_Y),
            (-BASE_RAIL_X, CASTER_Y),
            (BASE_RAIL_X, -CASTER_Y),
            (BASE_RAIL_X, CASTER_Y),
        )
    ):
        model.articulation(
            f"base_to_caster_{caster_idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=f"caster_{caster_idx}",
            origin=Origin(xyz=(x_pos, y_pos, WHEEL_RADIUS)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    clip_bar = object_model.get_part("clip_bar")
    lift_post_0 = object_model.get_part("lift_post_0")
    lift_post_1 = object_model.get_part("lift_post_1")
    release_paddle = object_model.get_part("release_paddle")

    lift_joint_0 = object_model.get_articulation("base_to_lift_post_0")
    lift_joint_1 = object_model.get_articulation("base_to_lift_post_1")
    tray_hinge = object_model.get_articulation("carriage_to_tray")
    clip_hinge = object_model.get_articulation("tray_to_clip_bar")
    paddle_hinge = object_model.get_articulation("carriage_to_release_paddle")

    ctx.expect_contact(
        carriage,
        lift_post_1,
        elem_a="saddle_1",
        elem_b="mast",
        name="right lift post supports the shared carriage",
    )
    ctx.expect_gap(
        tray,
        carriage,
        axis="z",
        positive_elem="deck",
        negative_elem="top_carriage_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="tray sits on the top carriage when level",
    )
    ctx.expect_gap(
        clip_bar,
        tray,
        axis="y",
        positive_elem="bar",
        negative_elem="front_rim",
        max_gap=0.035,
        max_penetration=0.0,
        name="chart clip bar stays at the tray edge",
    )
    ctx.expect_gap(
        carriage,
        release_paddle,
        axis="z",
        positive_elem="release_box",
        negative_elem="paddle",
        min_gap=0.0,
        max_gap=0.045,
        name="release paddle hangs below its housing",
    )

    rest_left = ctx.part_world_position(lift_post_0)
    rest_right = ctx.part_world_position(lift_post_1)
    rest_tray_aabb = ctx.part_world_aabb(tray)
    rest_clip_aabb = ctx.part_world_aabb(clip_bar)
    rest_paddle_aabb = ctx.part_world_aabb(release_paddle)

    lift_upper = lift_joint_0.motion_limits.upper if lift_joint_0.motion_limits is not None else None
    tray_upper = tray_hinge.motion_limits.upper if tray_hinge.motion_limits is not None else None
    clip_upper = clip_hinge.motion_limits.upper if clip_hinge.motion_limits is not None else None
    paddle_lower = paddle_hinge.motion_limits.lower if paddle_hinge.motion_limits is not None else None

    if lift_upper is not None:
        with ctx.pose({lift_joint_0: lift_upper}):
            extended_left = ctx.part_world_position(lift_post_0)
            extended_right = ctx.part_world_position(lift_post_1)
            ctx.check(
                "twin lift posts rise together",
                rest_left is not None
                and rest_right is not None
                and extended_left is not None
                and extended_right is not None
                and extended_left[2] > rest_left[2] + 0.15
                and extended_right[2] > rest_right[2] + 0.15
                and abs(extended_left[2] - extended_right[2]) < 1e-6,
                details=(
                    f"rest_left={rest_left}, rest_right={rest_right}, "
                    f"extended_left={extended_left}, extended_right={extended_right}"
                ),
            )

    if tray_upper is not None:
        with ctx.pose({tray_hinge: tray_upper}):
            tilted_tray_aabb = ctx.part_world_aabb(tray)
            ctx.check(
                "tray tilts upward at its rear hinge",
                rest_tray_aabb is not None
                and tilted_tray_aabb is not None
                and tilted_tray_aabb[1][2] > rest_tray_aabb[1][2] + 0.18,
                details=f"rest={rest_tray_aabb}, tilted={tilted_tray_aabb}",
            )

    if clip_upper is not None:
        with ctx.pose({clip_hinge: clip_upper}):
            raised_clip_aabb = ctx.part_world_aabb(clip_bar)
            ctx.check(
                "clip bar swings out from the tray rim hinge",
                rest_clip_aabb is not None
                and raised_clip_aabb is not None
                and raised_clip_aabb[1][1] > rest_clip_aabb[1][1] + 0.025,
                details=f"rest={rest_clip_aabb}, raised={raised_clip_aabb}",
            )

    if paddle_lower is not None:
        with ctx.pose({paddle_hinge: paddle_lower}):
            dropped_paddle_aabb = ctx.part_world_aabb(release_paddle)
            ctx.check(
                "release paddle can swing below the carriage",
                rest_paddle_aabb is not None
                and dropped_paddle_aabb is not None
                and dropped_paddle_aabb[0][1] < rest_paddle_aabb[0][1] - 0.02,
                details=f"rest={rest_paddle_aabb}, dropped={dropped_paddle_aabb}",
            )

    caster_checks_ok = True
    caster_details: list[str] = []
    for caster_idx in range(4):
        caster_joint = object_model.get_articulation(f"base_to_caster_{caster_idx}")
        limits = caster_joint.motion_limits
        is_continuous = caster_joint.articulation_type == ArticulationType.CONTINUOUS
        no_bounds = limits is not None and limits.lower is None and limits.upper is None
        caster_checks_ok = caster_checks_ok and is_continuous and no_bounds
        caster_details.append(
            f"{caster_joint.name}: type={caster_joint.articulation_type}, lower={None if limits is None else limits.lower}, upper={None if limits is None else limits.upper}"
        )
    ctx.check(
        "all four casters spin continuously on axles",
        caster_checks_ok,
        details="; ".join(caster_details),
    )

    column_opening_margin = (OUTER_COLUMN_OUTER / 2.0) - OUTER_COLUMN_WALL - (INNER_POST_SIZE / 2.0)
    ctx.check(
        "inner lift posts fit within the outer columns",
        column_opening_margin > 0.003,
        details=f"column_opening_margin={column_opening_margin}",
    )

    _ = base  # Keeps the named root part explicit in tests.
    _ = lift_joint_1
    return ctx.report()


object_model = build_object_model()
