from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


UPRIGHT_X = 0.27
UPRIGHT_SIZE = (0.08, 0.05, 1.82)
FOOT_SIZE = (0.16, 0.22, 0.06)
CROSSBAR_SIZE = (0.46, 0.045, 0.06)
LOWER_CROSSBAR_Z = 0.31
UPPER_CROSSBAR_Z = 1.58
CHANNEL_RAIL_SIZE = (0.012, 0.012, 1.22)
CHANNEL_Z = 0.88
SHELF_ORIGIN_Z = 0.38
SHELF_TRAVEL = 0.80

BRACE_ANGLE = 0.40
BRACE_AXIS = (0.0, -math.sin(BRACE_ANGLE), -math.cos(BRACE_ANGLE))
BRACE_RPY = (math.pi - BRACE_ANGLE, 0.0, 0.0)
BRACE_START = (0.0, 0.0, -0.03)
BRACE_OUTER_LENGTH = 0.72
BRACE_COUPLER_LENGTH = 0.09
BRACE_CHANNEL_LENGTH = 0.63
BRACE_INNER_INSERTION = 0.34
BRACE_INNER_LENGTH = 1.27
BRACE_EXTENSION = 0.14


def _v_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _v_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _rot_x(v: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = v
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _brace_offset(v: tuple[float, float, float]) -> tuple[float, float, float]:
    return _rot_x(v, BRACE_RPY[0])


def _close(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(a - b) <= tol


def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(_close(x, y, tol) for x, y in zip(a, b))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="h_frame_easel")

    oak = model.material("oak", color=(0.67, 0.51, 0.34))
    dark_steel = model.material("dark_steel", color=(0.20, 0.22, 0.24))
    graphite = model.material("graphite", color=(0.14, 0.15, 0.16))
    rubber = model.material("rubber", color=(0.08, 0.08, 0.08))

    def add_box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        *,
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        return part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    front_frame = model.part("front_frame")
    add_box(front_frame, "left_foot", FOOT_SIZE, (-UPRIGHT_X, 0.03, FOOT_SIZE[2] / 2.0), material=oak)
    add_box(front_frame, "right_foot", FOOT_SIZE, (UPRIGHT_X, 0.03, FOOT_SIZE[2] / 2.0), material=oak)
    add_box(
        front_frame,
        "left_upright",
        UPRIGHT_SIZE,
        (-UPRIGHT_X, 0.0, FOOT_SIZE[2] + UPRIGHT_SIZE[2] / 2.0),
        material=oak,
    )
    add_box(
        front_frame,
        "right_upright",
        UPRIGHT_SIZE,
        (UPRIGHT_X, 0.0, FOOT_SIZE[2] + UPRIGHT_SIZE[2] / 2.0),
        material=oak,
    )
    add_box(front_frame, "lower_crossbar", CROSSBAR_SIZE, (0.0, 0.0, LOWER_CROSSBAR_Z), material=oak)
    add_box(front_frame, "upper_crossbar", CROSSBAR_SIZE, (0.0, 0.0, UPPER_CROSSBAR_Z), material=oak)

    rail_offsets = (-0.022, 0.022)
    for side_name, side_x in (("left", -UPRIGHT_X), ("right", UPRIGHT_X)):
        for rail_name, rail_offset in (("outer", rail_offsets[0]), ("inner", rail_offsets[1])):
            add_box(
                front_frame,
                f"{side_name}_channel_{rail_name}_rail",
                CHANNEL_RAIL_SIZE,
                (side_x + rail_offset, 0.031, CHANNEL_Z),
                material=dark_steel,
            )

    canvas_shelf = model.part("canvas_shelf")
    slider_size = (0.028, 0.011, 0.22)
    carriage_size = (0.05, 0.03, 0.06)
    board_size = (0.56, 0.092, 0.028)
    lip_size = (0.56, 0.018, 0.022)

    add_box(
        canvas_shelf,
        "left_slider",
        slider_size,
        (-UPRIGHT_X, 0.0305, slider_size[2] / 2.0),
        material=graphite,
    )
    add_box(
        canvas_shelf,
        "right_slider",
        slider_size,
        (UPRIGHT_X, 0.0305, slider_size[2] / 2.0),
        material=graphite,
    )
    add_box(
        canvas_shelf,
        "left_carriage_block",
        carriage_size,
        (-UPRIGHT_X, 0.051, carriage_size[2] / 2.0),
        material=dark_steel,
    )
    add_box(
        canvas_shelf,
        "right_carriage_block",
        carriage_size,
        (UPRIGHT_X, 0.051, carriage_size[2] / 2.0),
        material=dark_steel,
    )
    add_box(
        canvas_shelf,
        "shelf_board",
        board_size,
        (0.0, 0.082, board_size[2] / 2.0),
        material=oak,
    )
    add_box(
        canvas_shelf,
        "shelf_lip",
        lip_size,
        (0.0, 0.137, board_size[2] + lip_size[2] / 2.0),
        material=oak,
    )

    rear_brace_outer = model.part("rear_brace_outer")
    add_box(
        rear_brace_outer,
        "brace_mount_block",
        (0.09, 0.025, 0.08),
        (0.0, 0.0, 0.0),
        material=dark_steel,
    )
    add_box(
        rear_brace_outer,
        "brace_coupler",
        (0.055, 0.025, BRACE_COUPLER_LENGTH),
        _brace_offset((0.0, 0.0, BRACE_COUPLER_LENGTH / 2.0)),
        material=dark_steel,
        rpy=BRACE_RPY,
    )
    for name, size, offset in (
        (
            "brace_back_web",
            (0.048, 0.004, BRACE_CHANNEL_LENGTH),
            (0.0, 0.010, BRACE_COUPLER_LENGTH + BRACE_CHANNEL_LENGTH / 2.0),
        ),
        (
            "brace_left_flange",
            (0.004, 0.024, BRACE_CHANNEL_LENGTH),
            (-0.024, 0.0, BRACE_COUPLER_LENGTH + BRACE_CHANNEL_LENGTH / 2.0),
        ),
        (
            "brace_right_flange",
            (0.004, 0.024, BRACE_CHANNEL_LENGTH),
            (0.024, 0.0, BRACE_COUPLER_LENGTH + BRACE_CHANNEL_LENGTH / 2.0),
        ),
    ):
        add_box(
            rear_brace_outer,
            name,
            size,
            _brace_offset(offset),
            material=graphite,
            rpy=BRACE_RPY,
        )
    for name, x_center in (("brace_stop_left", -0.019), ("brace_stop_right", 0.019)):
        add_box(
            rear_brace_outer,
            name,
            (0.006, 0.018, 0.008),
            _brace_offset((x_center, -0.002, BRACE_OUTER_LENGTH - 0.042)),
            material=dark_steel,
            rpy=BRACE_RPY,
        )

    rear_brace_inner = model.part("rear_brace_inner")
    add_box(
        rear_brace_inner,
        "brace_stop_collar",
        (0.042, 0.018, 0.016),
        _brace_offset((0.0, -0.002, -0.030)),
        material=dark_steel,
        rpy=BRACE_RPY,
    )
    add_box(
        rear_brace_inner,
        "brace_inner_tube",
        (0.032, 0.014, BRACE_INNER_LENGTH),
        _brace_offset((0.0, -0.004, BRACE_INNER_LENGTH / 2.0 - BRACE_INNER_INSERTION)),
        material=dark_steel,
        rpy=BRACE_RPY,
    )
    add_box(
        rear_brace_inner,
        "brace_foot_bracket",
        (0.046, 0.018, 0.18),
        _brace_offset((0.0, -0.006, 0.86)),
        material=dark_steel,
        rpy=BRACE_RPY,
    )
    foot_center = _v_add(_brace_offset((0.0, 0.0, 0.95)), (0.0, 0.0, -0.010))
    add_box(
        rear_brace_inner,
        "brace_foot",
        (0.10, 0.08, 0.032),
        foot_center,
        material=rubber,
    )

    model.articulation(
        "frame_to_canvas_shelf",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=canvas_shelf,
        origin=Origin(xyz=(0.0, 0.0, SHELF_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.5,
            lower=0.0,
            upper=SHELF_TRAVEL,
        ),
    )
    model.articulation(
        "frame_to_rear_brace_outer",
        ArticulationType.FIXED,
        parent=front_frame,
        child=rear_brace_outer,
        origin=Origin(xyz=(0.0, -0.035, UPPER_CROSSBAR_Z)),
    )
    model.articulation(
        "rear_brace_outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=rear_brace_outer,
        child=rear_brace_inner,
        origin=Origin(xyz=_v_add(BRACE_START, _v_scale(BRACE_AXIS, BRACE_OUTER_LENGTH))),
        axis=BRACE_AXIS,
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=BRACE_EXTENSION,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    canvas_shelf = object_model.get_part("canvas_shelf")
    rear_brace_outer = object_model.get_part("rear_brace_outer")
    rear_brace_inner = object_model.get_part("rear_brace_inner")

    frame_to_canvas_shelf = object_model.get_articulation("frame_to_canvas_shelf")
    frame_to_rear_brace_outer = object_model.get_articulation("frame_to_rear_brace_outer")
    rear_brace_outer_to_inner = object_model.get_articulation("rear_brace_outer_to_inner")

    left_upright = front_frame.get_visual("left_upright")
    right_upright = front_frame.get_visual("right_upright")
    upper_crossbar = front_frame.get_visual("upper_crossbar")
    left_slider = canvas_shelf.get_visual("left_slider")
    right_slider = canvas_shelf.get_visual("right_slider")
    brace_mount_block = rear_brace_outer.get_visual("brace_mount_block")
    brace_back_web = rear_brace_outer.get_visual("brace_back_web")
    brace_stop_collar = rear_brace_inner.get_visual("brace_stop_collar")
    brace_inner_tube = rear_brace_inner.get_visual("brace_inner_tube")

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

    frame_aabb = ctx.part_world_aabb(front_frame)
    if frame_aabb is None:
        ctx.fail("front_frame_has_geometry", "front_frame AABB was unavailable")
    else:
        width = frame_aabb[1][0] - frame_aabb[0][0]
        height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "front_frame_reads_as_heavy_duty_studio_easel",
            0.60 <= width <= 0.72 and 1.84 <= height <= 1.92,
            f"expected a broad, tall H-frame; got width={width:.3f} m height={height:.3f} m",
        )

    ctx.expect_gap(
        canvas_shelf,
        front_frame,
        axis="y",
        positive_elem=left_slider,
        negative_elem=left_upright,
        max_gap=0.0005,
        max_penetration=0.0,
        name="left_slider_bears_on_left_upright_channel",
    )
    ctx.expect_gap(
        canvas_shelf,
        front_frame,
        axis="y",
        positive_elem=right_slider,
        negative_elem=right_upright,
        max_gap=0.0005,
        max_penetration=0.0,
        name="right_slider_bears_on_right_upright_channel",
    )
    ctx.expect_overlap(
        canvas_shelf,
        front_frame,
        axes="xz",
        elem_a=left_slider,
        elem_b=left_upright,
        min_overlap=0.02,
        name="left_slider_stays_registered_to_left_upright",
    )
    ctx.expect_overlap(
        canvas_shelf,
        front_frame,
        axes="xz",
        elem_a=right_slider,
        elem_b=right_upright,
        min_overlap=0.02,
        name="right_slider_stays_registered_to_right_upright",
    )
    ctx.expect_contact(
        rear_brace_outer,
        front_frame,
        elem_a=brace_mount_block,
        elem_b=upper_crossbar,
        name="rear_brace_mount_contacts_upper_crossbar",
    )
    ctx.expect_contact(
        rear_brace_inner,
        rear_brace_outer,
        name="rear_brace_inner_remains_supported_by_outer_channel",
    )
    ctx.expect_overlap(
        rear_brace_inner,
        rear_brace_outer,
        axes="x",
        elem_a=brace_inner_tube,
        elem_b=brace_back_web,
        min_overlap=0.03,
        name="rear_brace_inner_tube_stays_centered_in_outer_channel",
    )

    shelf_limits = frame_to_canvas_shelf.motion_limits
    brace_limits = rear_brace_outer_to_inner.motion_limits
    ctx.check(
        "articulations_match_requested_mechanisms",
        frame_to_canvas_shelf.articulation_type == ArticulationType.PRISMATIC
        and _vec_close(tuple(frame_to_canvas_shelf.axis), (0.0, 0.0, 1.0))
        and shelf_limits is not None
        and _close(shelf_limits.lower or 0.0, 0.0)
        and _close(shelf_limits.upper or 0.0, SHELF_TRAVEL)
        and frame_to_rear_brace_outer.articulation_type == ArticulationType.FIXED
        and rear_brace_outer_to_inner.articulation_type == ArticulationType.PRISMATIC
        and _vec_close(tuple(rear_brace_outer_to_inner.axis), BRACE_AXIS)
        and brace_limits is not None
        and _close(brace_limits.lower or 0.0, 0.0)
        and _close(brace_limits.upper or 0.0, BRACE_EXTENSION),
        "expected a vertical sliding shelf, fixed outer brace, and telescoping rear brace strut",
    )

    shelf_rest = ctx.part_world_position(canvas_shelf)
    with ctx.pose({frame_to_canvas_shelf: 0.52}):
        shelf_raised = ctx.part_world_position(canvas_shelf)
    if shelf_rest is None or shelf_raised is None:
        ctx.fail("canvas_shelf_moves_vertically", "shelf world positions were unavailable")
    else:
        shelf_delta = _v_sub(shelf_raised, shelf_rest)
        ctx.check(
            "canvas_shelf_moves_straight_up_on_prismatic_channels",
            abs(shelf_delta[2] - 0.52) <= 1e-5
            and abs(shelf_delta[0]) <= 1e-6
            and abs(shelf_delta[1]) <= 1e-6,
            f"expected +0.52 m vertical motion only; got delta={shelf_delta}",
        )

    brace_rest = ctx.part_world_position(rear_brace_inner)
    with ctx.pose({rear_brace_outer_to_inner: BRACE_EXTENSION / 2.0}):
        brace_extended = ctx.part_world_position(rear_brace_inner)
    if brace_rest is None or brace_extended is None:
        ctx.fail("rear_brace_extends_along_telescoping_axis", "brace world positions were unavailable")
    else:
        brace_delta = _v_sub(brace_extended, brace_rest)
        expected_delta = _v_scale(BRACE_AXIS, BRACE_EXTENSION / 2.0)
        ctx.check(
            "rear_brace_extends_down_and_back",
            _vec_close(brace_delta, expected_delta, tol=5e-4)
            and brace_delta[1] < 0.0
            and brace_delta[2] < 0.0,
            f"expected brace delta {expected_delta}; got {brace_delta}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
