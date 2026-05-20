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


BASE_L = 0.34
BASE_W = 0.12
PLATE_T = 0.012
RAIL_L = 0.28
RAIL_W = 0.016
RAIL_H = 0.017
RAIL_Y = 0.032
STOP_T = 0.014
STOP_W = 0.072
STOP_H = 0.022
STOP_X = 0.149
RAIL_HOLE_XS = (-0.10, 0.0, 0.10)

CAR_L = 0.125
CAR_W = 0.076
CAR_H = 0.036
CAR_WALL = 0.0045
CAR_FLOOR = 0.005
SHOE_L = 0.086
SHOE_W = 0.014
SHOE_T = 0.010
SKIRT_L = 0.096
SKIRT_T = 0.004
SKIRT_H = 0.024
SKIRT_Y = 0.043
SKIRT_Z = -0.016

PIVOT_BLOCK_L = 0.028
PIVOT_BLOCK_T = 0.028
PIVOT_BLOCK_H = 0.046
PIVOT_BLOCK_X = 0.022
PIVOT_BLOCK_Y = 0.052
PIVOT_BLOCK_Z = 0.014
PIVOT_X = 0.024
PIVOT_Z = 0.024

EYE_T = 0.008
EYE_R = 0.0105
EYE_HOLE_R = 0.0048
PIVOT_Y = 0.079
EAR_T = 0.006
EAR_L = 0.020
EAR_H = 0.030
EAR_Y = 0.068
SUPPORT_BRIDGE_Y = 0.064
SUPPORT_BRIDGE_T = 0.010

BOLT_R = 0.004
SHAFT_LEN = 0.017
SHAFT_Y = 0.0795
WASHER_R = 0.007
WASHER_T = 0.0015
INNER_WASHER_Y = 0.07425
OUTER_WASHER_Y = 0.08375
HEAD_R = 0.0085
HEAD_T = 0.004
HEAD_Y = 0.0865

LINK_ARM_L = 0.050
LINK_ARM_H = 0.010
PADDLE_MOUNT_L = 0.012
PADDLE_MOUNT_H = 0.016
PADDLE_T = 0.004
PADDLE_W = 0.010
PADDLE_H = 0.026
PADDLE_X = 0.072

SLIDE_LOWER_X = -0.07
SHUTTLE_ORIGIN_Z = PLATE_T + RAIL_H + CAR_H / 2.0 + SHOE_T
SLIDE_TRAVEL = 0.14
PIVOT_LOWER = -0.35
PIVOT_UPPER = 1.10


def _translated_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _base_bed_shape() -> cq.Workplane:
    bed = _translated_box((BASE_L, BASE_W, PLATE_T), (0.0, 0.0, PLATE_T / 2.0))

    for sign in (-1.0, 1.0):
        stop = _translated_box(
            (STOP_T, STOP_W, STOP_H),
            (sign * STOP_X, 0.0, STOP_H / 2.0),
        )
        foot = _translated_box(
            (0.028, 0.040, 0.010),
            (sign * (STOP_X - sign * 0.008), 0.0, PLATE_T + 0.005),
        )
        bed = bed.union(stop).union(foot)

    return bed


def _rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(RAIL_L, RAIL_W, RAIL_H).translate((0.0, 0.0, PLATE_T + RAIL_H / 2.0))
    for x_pos in RAIL_HOLE_XS:
        hole = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .circle(0.0025)
            .extrude(RAIL_H + 0.004)
            .translate((0.0, 0.0, PLATE_T - 0.002))
        )
        counterbore = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .circle(0.0045)
            .extrude(0.0032)
            .translate((0.0, 0.0, PLATE_T + RAIL_H - 0.0032))
        )
        rail = rail.cut(hole).cut(counterbore)
    return rail


def _shuttle_body_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(CAR_L, CAR_W, CAR_H)
    inner = cq.Workplane("XY").box(
        CAR_L - 2.0 * CAR_WALL,
        CAR_W - 2.0 * CAR_WALL,
        CAR_H + 0.012,
    ).translate((0.0, 0.0, CAR_FLOOR + 0.005))
    body = outer.cut(inner)

    for sign in (-1.0, 1.0):
        shoe = _translated_box((SHOE_L, SHOE_W, SHOE_T), (0.0, sign * RAIL_Y, -(CAR_H / 2.0 + SHOE_T / 2.0)))
        skirt = _translated_box((SKIRT_L, SKIRT_T, SKIRT_H), (0.0, sign * SKIRT_Y, SKIRT_Z))
        body = body.union(shoe).union(skirt)

    front_flange = _translated_box((0.008, CAR_W - 0.010, 0.018), (CAR_L / 2.0 + 0.004, 0.0, 0.0))
    rear_flange = _translated_box((0.008, CAR_W - 0.018, 0.014), (-(CAR_L / 2.0 + 0.004), 0.0, -0.002))

    return body.union(front_flange).union(rear_flange)


def _pivot_support_shape() -> cq.Workplane:
    block = _translated_box(
        (PIVOT_BLOCK_L, PIVOT_BLOCK_T, PIVOT_BLOCK_H),
        (PIVOT_BLOCK_X, PIVOT_BLOCK_Y, PIVOT_BLOCK_Z),
    )
    rib = _translated_box((0.020, 0.010, 0.020), (0.010, CAR_W / 2.0 + 0.005, -0.002))
    bridge = _translated_box(
        (0.014, SUPPORT_BRIDGE_T, 0.020),
        (PIVOT_X, SUPPORT_BRIDGE_Y, PIVOT_Z),
    )
    ear = _translated_box((EAR_L, EAR_T, EAR_H), (PIVOT_X, EAR_Y, PIVOT_Z))
    ear_clear = (
        cq.Workplane("XZ")
        .center(PIVOT_X, PIVOT_Z)
        .circle(BOLT_R + 0.0006)
        .extrude(EAR_T + 0.002)
        .translate((0.0, EAR_Y - (EAR_T / 2.0) - 0.001, 0.0))
    )
    return block.union(rib).union(bridge).union(ear).cut(ear_clear)


def _shoulder_bolt_shape() -> cq.Workplane:
    inner_washer = (
        cq.Workplane("XZ")
        .center(PIVOT_X, PIVOT_Z)
        .circle(WASHER_R)
        .extrude(WASHER_T)
        .translate((0.0, INNER_WASHER_Y - (WASHER_T / 2.0), 0.0))
    )
    outer_washer = (
        cq.Workplane("XZ")
        .center(PIVOT_X, PIVOT_Z)
        .circle(WASHER_R)
        .extrude(WASHER_T)
        .translate((0.0, OUTER_WASHER_Y - (WASHER_T / 2.0), 0.0))
    )
    head = (
        cq.Workplane("XZ")
        .center(PIVOT_X, PIVOT_Z)
        .circle(HEAD_R)
        .extrude(HEAD_T)
        .translate((0.0, HEAD_Y - (HEAD_T / 2.0), 0.0))
    )
    return inner_washer.union(outer_washer).union(head)


def _inspection_link_shape() -> cq.Workplane:
    eye = (
        cq.Workplane("XZ")
        .circle(EYE_R)
        .extrude(EYE_T)
        .translate((0.0, -(EYE_T / 2.0), 0.0))
    )
    arm = _translated_box((LINK_ARM_L, EYE_T, LINK_ARM_H), (0.034, 0.0, 0.0))
    mount = _translated_box((PADDLE_MOUNT_L, EYE_T, PADDLE_MOUNT_H), (0.060, 0.0, 0.0))
    paddle = _translated_box((PADDLE_T, PADDLE_W, PADDLE_H), (PADDLE_X, 0.0, 0.0))
    link = eye.union(arm).union(mount).union(paddle)
    eye_hole = (
        cq.Workplane("XZ")
        .circle(EYE_HOLE_R)
        .extrude(EYE_T + 0.002)
        .translate((0.0, -(EYE_T / 2.0) - 0.001, 0.0))
    )
    return link.cut(eye_hole)


def _axis_matches(axis: tuple[float, float, float] | None, expected: tuple[float, float, float]) -> bool:
    if axis is None:
        return False
    return tuple(round(v, 6) for v in axis) == expected


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drawer_shuttle_inspection_paddle")

    model.material("base_steel", rgba=(0.54, 0.57, 0.60, 1.0))
    model.material("rail_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("carriage_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("black_oxide", rgba=(0.13, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_bed_shape(), "base_bed"),
        material="base_steel",
        name="bed",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape(), "left_rail"),
        material="rail_steel",
        origin=Origin(xyz=(0.0, RAIL_Y, 0.0)),
        name="left_rail",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape(), "right_rail"),
        material="rail_steel",
        origin=Origin(xyz=(0.0, -RAIL_Y, 0.0)),
        name="right_rail",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.04)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        mesh_from_cadquery(_shuttle_body_shape(), "shuttle_body"),
        material="carriage_gray",
        name="carriage",
    )
    shuttle.visual(
        mesh_from_cadquery(_pivot_support_shape(), "pivot_support"),
        material="rail_steel",
        name="pivot_support",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((0.14, 0.09, 0.07)),
        mass=1.7,
        origin=Origin(),
    )

    inspection_link = model.part("inspection_link")
    inspection_link.visual(
        mesh_from_cadquery(_inspection_link_shape(), "inspection_link"),
        material="black_oxide",
        name="link",
    )
    inspection_link.inertial = Inertial.from_geometry(
        Box((0.09, 0.02, 0.03)),
        mass=0.25,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=shuttle,
        origin=Origin(xyz=(SLIDE_LOWER_X, 0.0, SHUTTLE_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=220.0,
            velocity=0.24,
        ),
    )
    model.articulation(
        "shuttle_to_link",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=inspection_link,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=PIVOT_LOWER,
            upper=PIVOT_UPPER,
            effort=18.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    shuttle = object_model.get_part("shuttle")
    inspection_link = object_model.get_part("inspection_link")
    slide = object_model.get_articulation("base_to_shuttle")
    pivot = object_model.get_articulation("shuttle_to_link")
    left_rail = base.get_visual("left_rail")
    right_rail = base.get_visual("right_rail")
    pivot_support = shuttle.get_visual("pivot_support")

    ctx.check(
        "articulation_axes_match_layout",
        _axis_matches(slide.axis, (1.0, 0.0, 0.0)) and _axis_matches(pivot.axis, (0.0, 1.0, 0.0)),
        details=f"slide axis={slide.axis}, pivot axis={pivot.axis}",
    )
    ctx.check(
        "motion_limits_match_mechanism",
        (
            slide.motion_limits is not None
            and slide.motion_limits.lower == 0.0
            and slide.motion_limits.upper == SLIDE_TRAVEL
            and pivot.motion_limits is not None
            and pivot.motion_limits.lower == PIVOT_LOWER
            and pivot.motion_limits.upper == PIVOT_UPPER
        ),
        details="unexpected prismatic or revolute motion limits",
    )

    with ctx.pose({slide: 0.0, pivot: 0.0}):
        ctx.expect_contact(
            shuttle,
            base,
            elem_b=left_rail,
            name="shuttle_is_supported_on_left_rail_at_home",
        )
        ctx.expect_contact(
            shuttle,
            base,
            elem_b=right_rail,
            name="shuttle_is_supported_on_right_rail_at_home",
        )
        ctx.expect_contact(
            inspection_link,
            shuttle,
            elem_b=pivot_support,
            name="inspection_link_is_supported_by_pivot_block",
        )
        ctx.expect_gap(
            inspection_link,
            base,
            axis="y",
            min_gap=0.004,
            name="inspection_link_stays_outboard_of_base_at_home",
        )

    with ctx.pose({slide: SLIDE_TRAVEL, pivot: 0.0}):
        ctx.expect_contact(
            shuttle,
            base,
            elem_b=left_rail,
            name="shuttle_remains_supported_on_left_rail_fully_extended",
        )
        ctx.expect_contact(
            shuttle,
            base,
            elem_b=right_rail,
            name="shuttle_remains_supported_on_right_rail_fully_extended",
        )

    with ctx.pose({slide: SLIDE_TRAVEL, pivot: PIVOT_LOWER}):
        ctx.expect_gap(
            inspection_link,
            base,
            axis="z",
            min_gap=0.006,
            name="lowered_paddle_clears_base_top_at_full_extension",
        )
        ctx.expect_gap(
            inspection_link,
            base,
            axis="y",
            min_gap=0.004,
            name="lowered_paddle_stays_outboard_of_base",
        )

    with ctx.pose({slide: SLIDE_TRAVEL, pivot: PIVOT_UPPER}):
        ctx.expect_gap(
            inspection_link,
            base,
            axis="y",
            min_gap=0.004,
            name="raised_paddle_stays_outboard_of_base",
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
