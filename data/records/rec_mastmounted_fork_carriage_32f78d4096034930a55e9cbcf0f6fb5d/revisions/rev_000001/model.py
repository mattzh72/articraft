from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.40
BODY_L = 0.32
BODY_H = 0.30
DECK_W = 0.24
DECK_L = 0.18
DECK_H = 0.18
LEG_X = 0.125
LEG_W = 0.08
LEG_L = 0.58
LEG_H = 0.06

RAIL_X = 0.11
RAIL_W = 0.056
RAIL_D = 0.052
RAIL_H = 1.46

GUIDE_W = 0.072
GUIDE_D = 0.020
GUIDE_H = 0.26
PLATE_W = 0.32
PLATE_D = 0.028
PLATE_H = 0.36
FORK_W = 0.075
FORK_X = 0.11
FORK_L = 0.48
FORK_T = 0.028
FORK_HEEL_H = 0.10

LIFT_HOME_Z = 0.075
LIFT_TRAVEL = 0.50


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(*size, centered=(True, True, False))
        .translate(xyz)
    )


def _combine(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _make_body_shell() -> cq.Workplane:
    rear_body = _box((BODY_W, BODY_L, BODY_H), (0.0, -0.23, 0.0))
    service_deck = _box((DECK_W, 0.16, DECK_H), (0.0, -0.12, 0.06))
    mast_plinth = _box((0.22, 0.08, 0.10), (0.0, -0.08, 0.20))
    return _combine(rear_body, service_deck, mast_plinth)


def _make_outrigger_shell() -> cq.Workplane:
    left_leg = _box((LEG_W, LEG_L, LEG_H), (LEG_X, 0.11, 0.0))
    right_leg = _box((LEG_W, LEG_L, LEG_H), (-LEG_X, 0.11, 0.0))
    inner_bridge = _box((0.18, 0.10, LEG_H), (0.0, -0.10, 0.0))
    return _combine(left_leg, right_leg, inner_bridge)


def _make_mast_shell() -> cq.Workplane:
    left_rail = _box((RAIL_W, RAIL_D, RAIL_H), (-RAIL_X, -RAIL_D / 2.0, 0.06))
    right_rail = _box((RAIL_W, RAIL_D, RAIL_H), (RAIL_X, -RAIL_D / 2.0, 0.06))
    lower_cross = _box((0.30, RAIL_D, 0.12), (0.0, -RAIL_D / 2.0, 0.06))
    upper_cross = _box((0.34, 0.060, 0.08), (0.0, -0.030, 1.44))
    center_spine = _box((0.11, 0.028, 1.10), (0.0, -0.060, 0.18))
    cylinder_cover = _box((0.09, 0.018, 1.04), (0.0, -0.014, 0.18))
    return _combine(
        left_rail,
        right_rail,
        lower_cross,
        upper_cross,
        center_spine,
        cylinder_cover,
    )


def _make_carriage_shell() -> cq.Workplane:
    plate = _box((PLATE_W, PLATE_D, PLATE_H), (0.0, 0.024, 0.10))
    slot_left = _box((0.050, 0.060, 0.18), (-0.085, 0.024, 0.14))
    slot_center = _box((0.050, 0.060, 0.18), (0.0, 0.024, 0.14))
    slot_right = _box((0.050, 0.060, 0.18), (0.085, 0.024, 0.14))
    plate = plate.cut(slot_left).cut(slot_center).cut(slot_right)

    left_guide = _box((GUIDE_W, GUIDE_D, GUIDE_H), (-RAIL_X, GUIDE_D / 2.0, 0.105))
    right_guide = _box((GUIDE_W, GUIDE_D, GUIDE_H), (RAIL_X, GUIDE_D / 2.0, 0.105))
    upper_beam = _box((0.28, 0.024, 0.045), (0.0, 0.016, 0.42))

    left_guard = _box((0.028, 0.018, 0.36), (-0.108, 0.014, 0.465))
    right_guard = _box((0.028, 0.018, 0.36), (0.108, 0.014, 0.465))
    guard_mid = _box((0.22, 0.014, 0.028), (0.0, 0.014, 0.61))
    guard_top = _box((0.26, 0.020, 0.036), (0.0, 0.014, 0.80))
    guard_inner_left = _box((0.016, 0.012, 0.29), (-0.036, 0.014, 0.53))
    guard_inner_center = _box((0.016, 0.012, 0.29), (0.0, 0.014, 0.53))
    guard_inner_right = _box((0.016, 0.012, 0.29), (0.036, 0.014, 0.53))

    return _combine(
        plate,
        left_guide,
        right_guide,
        upper_beam,
        left_guard,
        right_guard,
        guard_mid,
        guard_top,
        guard_inner_left,
        guard_inner_center,
        guard_inner_right,
    )


def _make_fork(x_offset: float) -> cq.Workplane:
    profile = [
        (0.0, 0.0),
        (FORK_L, 0.0),
        (FORK_L, FORK_T),
        (0.10, FORK_T),
        (0.055, FORK_HEEL_H),
        (0.0, FORK_HEEL_H),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(FORK_W, both=True)
        .translate((x_offset, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_service_fork_carriage")

    model.material("body_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("mast_steel", rgba=(0.65, 0.67, 0.70, 1.0))
    model.material("carriage_orange", rgba=(0.92, 0.48, 0.12, 1.0))
    model.material("fork_steel", rgba=(0.40, 0.42, 0.45, 1.0))

    mast_frame = model.part("mast_frame")
    mast_frame.visual(
        mesh_from_cadquery(_make_body_shell(), "body_shell"),
        material="body_graphite",
        name="body_shell",
    )
    mast_frame.visual(
        mesh_from_cadquery(_make_outrigger_shell(), "outrigger_shell"),
        material="body_graphite",
        name="outrigger_shell",
    )
    mast_frame.visual(
        mesh_from_cadquery(_make_mast_shell(), "mast_shell"),
        material="mast_steel",
        name="mast_shell",
    )
    mast_frame.inertial = Inertial.from_geometry(
        Box((0.44, 0.74, 1.54)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.03, 0.77)),
    )

    moving_platen = model.part("moving_platen")
    moving_platen.visual(
        mesh_from_cadquery(_make_carriage_shell(), "carriage_shell"),
        material="carriage_orange",
        name="carriage_shell",
    )
    moving_platen.visual(
        mesh_from_cadquery(_make_fork(FORK_X), "left_fork"),
        material="fork_steel",
        name="left_fork",
    )
    moving_platen.visual(
        mesh_from_cadquery(_make_fork(-FORK_X), "right_fork"),
        material="fork_steel",
        name="right_fork",
    )
    moving_platen.inertial = Inertial.from_geometry(
        Box((0.34, 0.50, 0.90)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.25, 0.45)),
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=moving_platen,
        origin=Origin(xyz=(0.0, 0.0, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=2500.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast_frame = object_model.get_part("mast_frame")
    moving_platen = object_model.get_part("moving_platen")
    mast_lift = object_model.get_articulation("mast_lift")

    body_shell = mast_frame.get_visual("body_shell")
    outrigger_shell = mast_frame.get_visual("outrigger_shell")
    mast_shell = mast_frame.get_visual("mast_shell")
    carriage_shell = moving_platen.get_visual("carriage_shell")
    left_fork = moving_platen.get_visual("left_fork")
    right_fork = moving_platen.get_visual("right_fork")

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

    limits = mast_lift.motion_limits
    axis_ok = tuple(round(value, 6) for value in mast_lift.axis) == (0.0, 0.0, 1.0)
    limit_ok = (
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - LIFT_TRAVEL) < 1e-9
    )
    ctx.check(
        "mast_lift_is_vertical",
        axis_ok,
        f"expected vertical lift axis, got {mast_lift.axis}",
    )
    ctx.check(
        "mast_lift_travel_is_compact",
        limit_ok,
        f"unexpected lift limits: {limits}",
    )

    with ctx.pose({mast_lift: 0.0}):
        ctx.expect_contact(
            moving_platen,
            mast_frame,
            elem_a=carriage_shell,
            elem_b=mast_shell,
            name="lowered_carriage_guides_touch_mast",
        )
        ctx.expect_overlap(
            moving_platen,
            mast_frame,
            axes="x",
            elem_a=carriage_shell,
            elem_b=mast_shell,
            min_overlap=0.20,
            name="lowered_carriage_stays_between_mast_rails",
        )
        ctx.expect_gap(
            moving_platen,
            mast_frame,
            axis="z",
            positive_elem=left_fork,
            negative_elem=outrigger_shell,
            min_gap=0.010,
            max_gap=0.025,
            name="left_fork_clears_outriggers_when_lowered",
        )
        ctx.expect_gap(
            moving_platen,
            mast_frame,
            axis="z",
            positive_elem=right_fork,
            negative_elem=outrigger_shell,
            min_gap=0.010,
            max_gap=0.025,
            name="right_fork_clears_outriggers_when_lowered",
        )

    if limits is not None and limits.upper is not None:
        with ctx.pose({mast_lift: limits.upper}):
            ctx.expect_contact(
                moving_platen,
                mast_frame,
                elem_a=carriage_shell,
                elem_b=mast_shell,
                name="raised_carriage_guides_still_touch_mast",
            )
            ctx.expect_overlap(
                moving_platen,
                mast_frame,
                axes="x",
                elem_a=carriage_shell,
                elem_b=mast_shell,
                min_overlap=0.20,
                name="raised_carriage_stays_between_mast_rails",
            )

        with ctx.pose({mast_lift: 0.0}):
            lowered_pos = ctx.part_world_position(moving_platen)
        with ctx.pose({mast_lift: limits.upper}):
            raised_pos = ctx.part_world_position(moving_platen)
            raised_aabb = ctx.part_world_aabb(moving_platen)
            mast_aabb = ctx.part_world_aabb(mast_frame)

        straight_lift_ok = (
            lowered_pos is not None
            and raised_pos is not None
            and abs(raised_pos[0] - lowered_pos[0]) < 1e-6
            and abs(raised_pos[1] - lowered_pos[1]) < 1e-6
            and abs((raised_pos[2] - lowered_pos[2]) - limits.upper) < 1e-6
        )
        ctx.check(
            "mast_lift_moves_platen_straight_up",
            straight_lift_ok,
            f"lowered={lowered_pos}, raised={raised_pos}, expected dz={limits.upper}",
        )

        raised_within_mast_height = (
            raised_aabb is not None
            and mast_aabb is not None
            and raised_aabb[1][2] <= mast_aabb[1][2] + 0.02
        )
        ctx.check(
            "raised_platen_stays_below_mast_head",
            raised_within_mast_height,
            f"raised platen aabb={raised_aabb}, mast aabb={mast_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
