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


BASE_LENGTH = 0.22
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.014
COLUMN_RADIUS = 0.032
COLUMN_HEIGHT = 0.112
TOP_FLANGE_RADIUS = 0.055
TOP_FLANGE_THICKNESS = 0.020

PLATFORM_RADIUS = 0.075
PLATFORM_THICKNESS = 0.014
PLATFORM_HUB_RADIUS = 0.028
PLATFORM_HUB_HEIGHT = 0.016
ARM_THICKNESS = 0.016
ARM_DEPTH = 0.018
ARM_HEIGHT = 0.060
FORK_INNER_GAP = 0.052
FORK_OUTER_SPAN = FORK_INNER_GAP + 2.0 * ARM_THICKNESS
ARM_CENTER_Y = FORK_INNER_GAP / 2.0 + ARM_THICKNESS / 2.0
SUPPORT_DEPTH = 0.060
SUPPORT_WIDTH = FORK_OUTER_SPAN
SUPPORT_HEIGHT = 0.028
SUPPORT_CENTER_X = 0.020
PITCH_AXIS_X = 0.048
PITCH_AXIS_Z = 0.060
FORK_HOLE_RADIUS = 0.0060
ARM_BASE_Z = PLATFORM_THICKNESS + SUPPORT_HEIGHT

TRUNNION_RADIUS = FORK_HOLE_RADIUS
COLLAR_RADIUS = 0.009
COLLAR_THICKNESS = 0.002
CRADLE_LENGTH = 0.044
CRADLE_WIDTH = 0.032
CRADLE_HEIGHT = 0.020
CRADLE_X_START = 0.010
CRADLE_Z_START = -0.010
CRADLE_INNER_LENGTH = 0.030
CRADLE_INNER_WIDTH = 0.022
CRADLE_INNER_HEIGHT = 0.012
CRADLE_INNER_X_START = 0.018
CRADLE_INNER_Z_START = -0.002
CRADLE_BRACKET_LENGTH = 0.018
CRADLE_BRACKET_WIDTH = 0.018
CRADLE_BRACKET_HEIGHT = 0.018
CRADLE_BRACKET_X_START = -0.006
CRADLE_BRACKET_Z_START = -0.009

YAW_LIMIT = 2.6
PITCH_LOWER = -0.45
PITCH_UPPER = 1.05


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=0.0006,
        angular_tolerance=0.08,
    )


def _y_axis_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(center[0], center[1] - length / 2.0, center[2]),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )


def _make_pedestal() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .edges()
        .fillet(0.003)
    )
    column = (
        cq.Workplane("XY")
        .circle(COLUMN_RADIUS)
        .extrude(COLUMN_HEIGHT)
        .faces(">Z")
        .edges()
        .fillet(0.004)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    flange = (
        cq.Workplane("XY")
        .circle(TOP_FLANGE_RADIUS)
        .extrude(TOP_FLANGE_THICKNESS)
        .faces(">Z")
        .edges()
        .fillet(0.003)
        .translate((0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT))
    )
    return base.union(column).union(flange)


def _make_lower_platform() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .circle(PLATFORM_RADIUS)
        .extrude(PLATFORM_THICKNESS)
        .faces(">Z")
        .edges()
        .fillet(0.0025)
    )
    hub = (
        cq.Workplane("XY")
        .circle(PLATFORM_HUB_RADIUS)
        .extrude(PLATFORM_HUB_HEIGHT)
        .translate((0.0, 0.0, PLATFORM_THICKNESS))
    )
    support_block = cq.Workplane("XY").box(
        SUPPORT_DEPTH,
        SUPPORT_WIDTH,
        SUPPORT_HEIGHT,
        centered=(True, True, False),
    ).translate((SUPPORT_CENTER_X, 0.0, PLATFORM_THICKNESS))
    return turntable.union(hub).union(support_block)


def _make_fork_arm(side_sign: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(ARM_DEPTH, ARM_THICKNESS, ARM_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
        .translate((PITCH_AXIS_X, side_sign * ARM_CENTER_Y, ARM_BASE_Z))
    )


def _make_cradle_body() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CRADLE_LENGTH, CRADLE_WIDTH, CRADLE_HEIGHT, centered=(False, True, False))
        .translate((CRADLE_X_START, 0.0, CRADLE_Z_START))
    )
    pocket = (
        cq.Workplane("XY")
        .box(
            CRADLE_INNER_LENGTH,
            CRADLE_INNER_WIDTH,
            CRADLE_INNER_HEIGHT,
            centered=(False, True, False),
        )
        .translate((CRADLE_INNER_X_START, 0.0, CRADLE_INNER_Z_START))
    )
    tray = outer.cut(pocket)
    neck = (
        cq.Workplane("XY")
        .box(
            CRADLE_BRACKET_LENGTH,
            CRADLE_BRACKET_WIDTH,
            CRADLE_BRACKET_HEIGHT,
            centered=(False, True, False),
        )
        .translate((CRADLE_BRACKET_X_START, 0.0, CRADLE_BRACKET_Z_START))
    )
    return tray.union(neck)


def _make_trunnion_axle() -> cq.Workplane:
    axle = _y_axis_cylinder(TRUNNION_RADIUS, FORK_OUTER_SPAN, (0.0, 0.0, 0.0))
    left_collar = _y_axis_cylinder(
        COLLAR_RADIUS,
        COLLAR_THICKNESS,
        (0.0, FORK_OUTER_SPAN / 2.0 + COLLAR_THICKNESS / 2.0, 0.0),
    )
    right_collar = _y_axis_cylinder(
        COLLAR_RADIUS,
        COLLAR_THICKNESS,
        (0.0, -FORK_OUTER_SPAN / 2.0 - COLLAR_THICKNESS / 2.0, 0.0),
    )
    return axle.union(left_collar).union(right_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_yaw_pitch_fixture")

    pedestal_mat = model.material("pedestal_gray", color=(0.28, 0.30, 0.33, 1.0))
    platform_mat = model.material("platform_gray", color=(0.44, 0.46, 0.49, 1.0))
    cradle_mat = model.material("cradle_black", color=(0.12, 0.13, 0.14, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        _mesh(_make_pedestal(), "pedestal_shell"),
        origin=Origin(),
        material=pedestal_mat,
        name="pedestal_shell",
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        _mesh(_make_lower_platform(), "lower_platform_shell"),
        origin=Origin(),
        material=platform_mat,
        name="platform_base",
    )
    lower_platform.visual(
        _mesh(_make_fork_arm(1.0), "left_fork_arm"),
        origin=Origin(),
        material=platform_mat,
        name="left_arm",
    )
    lower_platform.visual(
        _mesh(_make_fork_arm(-1.0), "right_fork_arm"),
        origin=Origin(),
        material=platform_mat,
        name="right_arm",
    )

    cradle = model.part("cradle")
    cradle.visual(
        _mesh(_make_cradle_body(), "cradle_body_mesh"),
        origin=Origin(),
        material=cradle_mat,
        name="cradle_body",
    )
    cradle.visual(
        _mesh(_make_trunnion_axle(), "trunnion_axle_mesh"),
        origin=Origin(),
        material=cradle_mat,
        name="trunnion_axle",
    )

    model.articulation(
        "pedestal_to_platform",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=lower_platform,
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + TOP_FLANGE_THICKNESS)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )
    model.articulation(
        "platform_to_cradle",
        ArticulationType.REVOLUTE,
        parent=lower_platform,
        child=cradle,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    lower_platform = object_model.get_part("lower_platform")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("pedestal_to_platform")
    pitch = object_model.get_articulation("platform_to_cradle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        cradle,
        lower_platform,
        elem_a="trunnion_axle",
        elem_b="left_arm",
        reason="pitch trunnion passes through the left fork bearing bore",
    )
    ctx.allow_overlap(
        cradle,
        lower_platform,
        elem_a="trunnion_axle",
        elem_b="right_arm",
        reason="pitch trunnion passes through the right fork bearing bore",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "yaw_axis_is_vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {yaw.axis}",
    )
    ctx.check(
        "pitch_axis_is_horizontal",
        pitch.axis == (0.0, -1.0, 0.0),
        details=f"expected (0, -1, 0), got {pitch.axis}",
    )
    ctx.check(
        "yaw_has_bi_directional_range",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower < 0.0 < yaw.motion_limits.upper,
        details="yaw joint should sweep to both sides of center",
    )
    ctx.check(
        "pitch_has_upward_bias",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper
        and pitch.motion_limits.upper > abs(pitch.motion_limits.lower),
        details="pitch should allow a modest downward tip and a larger upward tip",
    )

    ctx.expect_gap(
        lower_platform,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="platform_seats_on_pedestal",
    )
    ctx.expect_overlap(
        lower_platform,
        pedestal,
        axes="xy",
        min_overlap=0.10,
        name="platform_overlaps_pedestal_in_plan",
    )
    ctx.expect_contact(
        cradle,
        lower_platform,
        elem_a="trunnion_axle",
        elem_b="left_arm",
        contact_tol=0.0015,
        name="left_trunnion_supported_by_fork",
    )
    ctx.expect_contact(
        cradle,
        lower_platform,
        elem_a="trunnion_axle",
        elem_b="right_arm",
        contact_tol=0.0015,
        name="right_trunnion_supported_by_fork",
    )
    ctx.expect_gap(
        lower_platform,
        cradle,
        axis="y",
        positive_elem="left_arm",
        negative_elem="cradle_body",
        min_gap=0.001,
        name="cradle_body_clears_left_arm",
    )
    ctx.expect_gap(
        cradle,
        lower_platform,
        axis="y",
        positive_elem="cradle_body",
        negative_elem="right_arm",
        min_gap=0.001,
        name="cradle_body_clears_right_arm",
    )
    ctx.expect_contact(
        cradle,
        lower_platform,
        elem_a="trunnion_axle",
        elem_b="left_arm",
        contact_tol=0.0015,
        name="cradle_supported_by_fork",
    )

    closed_cradle_box = ctx.part_world_aabb(cradle)
    with ctx.pose({yaw: 1.2}):
        yawed_cradle_box = ctx.part_world_aabb(cradle)
        ctx.expect_gap(
            lower_platform,
            pedestal,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="platform_stays_seated_when_yawed",
        )
        ctx.expect_contact(
            cradle,
            lower_platform,
            elem_a="trunnion_axle",
            elem_b="left_arm",
            contact_tol=0.0015,
            name="left_trunnion_stays_supported_when_yawed",
        )
        ctx.expect_contact(
            cradle,
            lower_platform,
            elem_a="trunnion_axle",
            elem_b="right_arm",
            contact_tol=0.0015,
            name="right_trunnion_stays_supported_when_yawed",
        )
        ctx.expect_contact(
            cradle,
            lower_platform,
            elem_a="trunnion_axle",
            elem_b="left_arm",
            contact_tol=0.0015,
            name="cradle_stays_supported_when_yawed",
        )

    with ctx.pose({pitch: PITCH_UPPER}):
        pitched_cradle_box = ctx.part_world_aabb(cradle)
        ctx.expect_contact(
            cradle,
            lower_platform,
            elem_a="trunnion_axle",
            elem_b="left_arm",
            contact_tol=0.0015,
            name="left_trunnion_stays_supported_when_pitched",
        )
        ctx.expect_contact(
            cradle,
            lower_platform,
            elem_a="trunnion_axle",
            elem_b="right_arm",
            contact_tol=0.0015,
            name="right_trunnion_stays_supported_when_pitched",
        )
        ctx.expect_contact(
            cradle,
            lower_platform,
            elem_a="trunnion_axle",
            elem_b="left_arm",
            contact_tol=0.0015,
            name="cradle_stays_supported_when_pitched",
        )

    if closed_cradle_box is not None and yawed_cradle_box is not None:
        ctx.check(
            "positive_yaw_swings_cradle_toward_positive_y",
            yawed_cradle_box[1][1] > closed_cradle_box[1][1] + 0.015,
            details=(
                f"closed max y={closed_cradle_box[1][1]:.4f}, "
                f"yawed max y={yawed_cradle_box[1][1]:.4f}"
            ),
        )

    if closed_cradle_box is not None and pitched_cradle_box is not None:
        ctx.check(
            "positive_pitch_raises_cradle",
            pitched_cradle_box[1][2] > closed_cradle_box[1][2] + 0.02,
            details=(
                f"closed max z={closed_cradle_box[1][2]:.4f}, "
                f"pitched max z={pitched_cradle_box[1][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
