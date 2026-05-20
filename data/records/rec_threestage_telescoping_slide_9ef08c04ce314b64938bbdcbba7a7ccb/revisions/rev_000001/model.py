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


BASE_T = 0.012
WALL = 0.005
FACE_T = 0.007

OUTER_LEN = 0.72
OUTER_W = 0.20
OUTER_H = 0.105

MIDDLE_LEN = 0.54
MIDDLE_W = 0.162
MIDDLE_H = 0.076

INNER_LEN = 0.38
INNER_W = 0.128
INNER_H = 0.054

HOME_SETBACK = 0.03
OUTER_TRAVEL = 0.27
INNER_TRAVEL = 0.20

BASE_FRONT_MARGIN = 0.015
BASE_REAR_MARGIN = 0.03
BASE_SIDE_MARGIN = 0.02


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    x_min: float,
    z_min: float,
    material: str,
    y_center: float = 0.0,
):
    sx, sy, sz = size
    part.visual(
        Box(size),
        origin=Origin(xyz=(x_min + 0.5 * sx, y_center, z_min + 0.5 * sz)),
        material=material,
        name=name,
    )


def _box(length: float, width: float, height: float, *, x_min: float, z_min: float, y_center: float = 0.0):
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, False))
        .translate((x_min, y_center, z_min))
    )


def _outer_guide_shape():
    base = _box(
        OUTER_LEN + BASE_FRONT_MARGIN + BASE_REAR_MARGIN,
        OUTER_W + 2.0 * BASE_SIDE_MARGIN,
        BASE_T,
        x_min=-OUTER_LEN - BASE_REAR_MARGIN,
        z_min=0.0,
    )
    floor = _box(OUTER_LEN, OUTER_W, WALL, x_min=-OUTER_LEN, z_min=BASE_T)
    left_wall = _box(
        OUTER_LEN,
        WALL,
        OUTER_H - WALL,
        x_min=-OUTER_LEN,
        z_min=BASE_T + WALL,
        y_center=0.5 * (OUTER_W - WALL),
    )
    right_wall = _box(
        OUTER_LEN,
        WALL,
        OUTER_H - WALL,
        x_min=-OUTER_LEN,
        z_min=BASE_T + WALL,
        y_center=-0.5 * (OUTER_W - WALL),
    )
    rear_wall = _box(
        WALL,
        OUTER_W,
        OUTER_H - WALL,
        x_min=-OUTER_LEN,
        z_min=BASE_T + WALL,
    )
    front_frame = _box(FACE_T, OUTER_W, OUTER_H, x_min=-FACE_T, z_min=BASE_T)

    outer = base.union(floor).union(left_wall).union(right_wall).union(rear_wall).union(front_frame)

    front_opening = _box(
        FACE_T + 0.002,
        MIDDLE_W + 0.008,
        MIDDLE_H + 0.003,
        x_min=-FACE_T - 0.001,
        z_min=BASE_T + WALL - 0.0005,
    )
    return outer.cut(front_opening)


def _middle_runner_shape():
    floor = _box(MIDDLE_LEN, MIDDLE_W, WALL, x_min=-MIDDLE_LEN, z_min=0.0)
    left_wall = _box(
        MIDDLE_LEN,
        WALL,
        MIDDLE_H - WALL,
        x_min=-MIDDLE_LEN,
        z_min=WALL,
        y_center=0.5 * (MIDDLE_W - WALL),
    )
    right_wall = _box(
        MIDDLE_LEN,
        WALL,
        MIDDLE_H - WALL,
        x_min=-MIDDLE_LEN,
        z_min=WALL,
        y_center=-0.5 * (MIDDLE_W - WALL),
    )
    rear_wall = _box(WALL, MIDDLE_W, MIDDLE_H - WALL, x_min=-MIDDLE_LEN, z_min=WALL)
    front_frame = _box(FACE_T, MIDDLE_W, MIDDLE_H, x_min=-FACE_T, z_min=0.0)

    middle = floor.union(left_wall).union(right_wall).union(rear_wall).union(front_frame)

    front_opening = _box(
        FACE_T + 0.002,
        INNER_W + 0.008,
        INNER_H + 0.003,
        x_min=-FACE_T - 0.001,
        z_min=WALL - 0.0005,
    )
    return middle.cut(front_opening)


def _inner_runner_shape():
    floor = _box(INNER_LEN, INNER_W, WALL, x_min=-INNER_LEN, z_min=0.0)
    left_wall = _box(
        INNER_LEN,
        WALL,
        INNER_H - WALL,
        x_min=-INNER_LEN,
        z_min=WALL,
        y_center=0.5 * (INNER_W - WALL),
    )
    right_wall = _box(
        INNER_LEN,
        WALL,
        INNER_H - WALL,
        x_min=-INNER_LEN,
        z_min=WALL,
        y_center=-0.5 * (INNER_W - WALL),
    )
    rear_wall = _box(WALL, INNER_W, INNER_H - WALL, x_min=-INNER_LEN, z_min=WALL)
    front_face = _box(FACE_T, INNER_W, INNER_H, x_min=-FACE_T, z_min=0.0)

    return floor.union(left_wall).union(right_wall).union(rear_wall).union(front_face)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_service_slide")

    model.material("fixture_dark", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("runner_mid", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("runner_light", rgba=(0.75, 0.77, 0.79, 1.0))

    outer = model.part("outer_guide")
    _add_box_visual(
        outer,
        name="base_plate",
        size=(
            OUTER_LEN + BASE_FRONT_MARGIN + BASE_REAR_MARGIN,
            OUTER_W + 2.0 * BASE_SIDE_MARGIN,
            BASE_T,
        ),
        x_min=-OUTER_LEN - BASE_REAR_MARGIN,
        z_min=0.0,
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="guide_floor",
        size=(OUTER_LEN, OUTER_W, WALL),
        x_min=-OUTER_LEN,
        z_min=BASE_T,
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="left_wall",
        size=(OUTER_LEN, WALL, OUTER_H - WALL),
        x_min=-OUTER_LEN,
        z_min=BASE_T + WALL,
        y_center=0.5 * (OUTER_W - WALL),
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="right_wall",
        size=(OUTER_LEN, WALL, OUTER_H - WALL),
        x_min=-OUTER_LEN,
        z_min=BASE_T + WALL,
        y_center=-0.5 * (OUTER_W - WALL),
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="rear_stop",
        size=(WALL, OUTER_W, OUTER_H - WALL),
        x_min=-OUTER_LEN,
        z_min=BASE_T + WALL,
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="front_left_jamb",
        size=(FACE_T, WALL, OUTER_H),
        x_min=-FACE_T,
        z_min=BASE_T,
        y_center=0.5 * (OUTER_W - WALL),
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="front_right_jamb",
        size=(FACE_T, WALL, OUTER_H),
        x_min=-FACE_T,
        z_min=BASE_T,
        y_center=-0.5 * (OUTER_W - WALL),
        material="fixture_dark",
    )
    _add_box_visual(
        outer,
        name="front_lintel",
        size=(FACE_T, OUTER_W, OUTER_H - MIDDLE_H - WALL - 0.004),
        x_min=-FACE_T,
        z_min=BASE_T + WALL + MIDDLE_H + 0.004,
        material="fixture_dark",
    )

    middle = model.part("middle_runner")
    _add_box_visual(
        middle,
        name="runner_floor",
        size=(MIDDLE_LEN, MIDDLE_W, WALL),
        x_min=-MIDDLE_LEN,
        z_min=0.0,
        material="runner_mid",
    )
    _add_box_visual(
        middle,
        name="left_wall",
        size=(MIDDLE_LEN, WALL, MIDDLE_H - WALL),
        x_min=-MIDDLE_LEN,
        z_min=WALL,
        y_center=0.5 * (MIDDLE_W - WALL),
        material="runner_mid",
    )
    _add_box_visual(
        middle,
        name="right_wall",
        size=(MIDDLE_LEN, WALL, MIDDLE_H - WALL),
        x_min=-MIDDLE_LEN,
        z_min=WALL,
        y_center=-0.5 * (MIDDLE_W - WALL),
        material="runner_mid",
    )
    _add_box_visual(
        middle,
        name="rear_face",
        size=(WALL, MIDDLE_W, MIDDLE_H - WALL),
        x_min=-MIDDLE_LEN,
        z_min=WALL,
        material="runner_mid",
    )
    _add_box_visual(
        middle,
        name="front_left_jamb",
        size=(FACE_T, WALL, MIDDLE_H),
        x_min=-FACE_T,
        z_min=0.0,
        y_center=0.5 * (MIDDLE_W - WALL),
        material="runner_mid",
    )
    _add_box_visual(
        middle,
        name="front_right_jamb",
        size=(FACE_T, WALL, MIDDLE_H),
        x_min=-FACE_T,
        z_min=0.0,
        y_center=-0.5 * (MIDDLE_W - WALL),
        material="runner_mid",
    )
    _add_box_visual(
        middle,
        name="front_lintel",
        size=(FACE_T, MIDDLE_W, MIDDLE_H - INNER_H - 0.002),
        x_min=-FACE_T,
        z_min=INNER_H + 0.002,
        material="runner_mid",
    )

    inner = model.part("inner_runner")
    _add_box_visual(
        inner,
        name="runner_floor",
        size=(INNER_LEN, INNER_W, WALL),
        x_min=-INNER_LEN,
        z_min=0.0,
        material="runner_light",
    )
    _add_box_visual(
        inner,
        name="left_wall",
        size=(INNER_LEN, WALL, INNER_H - WALL),
        x_min=-INNER_LEN,
        z_min=WALL,
        y_center=0.5 * (INNER_W - WALL),
        material="runner_light",
    )
    _add_box_visual(
        inner,
        name="right_wall",
        size=(INNER_LEN, WALL, INNER_H - WALL),
        x_min=-INNER_LEN,
        z_min=WALL,
        y_center=-0.5 * (INNER_W - WALL),
        material="runner_light",
    )
    _add_box_visual(
        inner,
        name="rear_face",
        size=(WALL, INNER_W, INNER_H - WALL),
        x_min=-INNER_LEN,
        z_min=WALL,
        material="runner_light",
    )
    _add_box_visual(
        inner,
        name="front_face",
        size=(FACE_T, INNER_W, INNER_H),
        x_min=-FACE_T,
        z_min=0.0,
        material="runner_light",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(-HOME_SETBACK, 0.0, BASE_T + WALL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(-HOME_SETBACK, 0.0, WALL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "outer_to_middle_is_prismatic_x",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and outer_to_middle.motion_limits is not None
        and outer_to_middle.motion_limits.lower == 0.0
        and outer_to_middle.motion_limits.upper == OUTER_TRAVEL,
        details="outer_to_middle should be an +X prismatic stage with the planned travel.",
    )
    ctx.check(
        "middle_to_inner_is_prismatic_x",
        middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0)
        and middle_to_inner.motion_limits is not None
        and middle_to_inner.motion_limits.lower == 0.0
        and middle_to_inner.motion_limits.upper == INNER_TRAVEL,
        details="middle_to_inner should be an +X prismatic stage with the planned travel.",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(middle, outer, name="middle_supported_by_outer_closed")
        ctx.expect_contact(inner, middle, name="inner_supported_by_middle_closed")
        ctx.expect_within(middle, outer, axes="yz", margin=0.0, name="middle_nested_in_outer_closed")
        ctx.expect_within(inner, middle, axes="yz", margin=0.0, name="inner_nested_in_middle_closed")

    with ctx.pose({outer_to_middle: OUTER_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_contact(middle, outer, name="middle_supported_by_outer_extended")
        ctx.expect_contact(inner, middle, name="inner_supported_by_middle_extended")
        ctx.expect_within(middle, outer, axes="yz", margin=0.0, name="middle_guided_in_outer_extended")
        ctx.expect_within(inner, middle, axes="yz", margin=0.0, name="inner_guided_in_middle_extended")
        ctx.expect_overlap(middle, outer, axes="x", min_overlap=0.29, name="middle_keeps_practical_overlap")
        ctx.expect_overlap(inner, middle, axes="x", min_overlap=0.20, name="inner_keeps_practical_overlap")
        ctx.expect_origin_gap(middle, outer, axis="x", min_gap=0.20, name="middle_extends_forward")
        ctx.expect_origin_gap(inner, middle, axis="x", min_gap=0.13, name="inner_extends_forward_of_middle")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
