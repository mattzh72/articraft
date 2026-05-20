from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_W = 0.170
BASE_D = 0.190
BASE_H = 0.030
BASE_POCKET_W = 0.104
BASE_POCKET_D = 0.132
BASE_POCKET_H = 0.010

INNER_SPAN = 0.120
CHEEK_T = 0.012
CHEEK_CENTER_X = INNER_SPAN / 2.0 + CHEEK_T / 2.0
FOOT_W = 0.020
FOOT_D = 0.112
FOOT_H = 0.012
CHEEK_PLATE_D = 0.160
CHEEK_PLATE_H = 0.093
CHEEK_TOTAL_H = FOOT_H + CHEEK_PLATE_H
PITCH_LOCAL_Z = 0.066
PITCH_GLOBAL_Z = BASE_H + PITCH_LOCAL_Z
PIVOT_BOSS_R = 0.018
PIVOT_HOLE_R = 0.0075
PIVOT_BOSS_EXTRA = 0.004
TAB_PROJ = 0.003

TRAY_OUTER_W = 0.098
TRAY_LEN = 0.136
TRAY_DEPTH = 0.018
TRAY_WALL = 0.004
TRAY_PAN_Z = -0.017
TRAY_FLOOR_T = 0.003
TRAY_RIM_T = 0.003
TRAY_RIM_W = 0.008
TRAY_SIDE_PLATE_T = 0.004
TRAY_SIDE_PLATE_W = 0.122
TRAY_SIDE_PLATE_H = 0.066
TRAY_SIDE_PLATE_Z = -0.003
TRAY_SIDE_X = 0.048
TRAY_BOSS_R = 0.016
TRAY_BOSS_L = 0.016
TRAY_PIVOT_HOLE_R = 0.0085
TRAY_STOP_T = 0.003
TRAY_STOP_Y = 0.014
TRAY_STOP_Z = 0.010
TRAY_STOP_X = 0.049

SHAFT_R = 0.006
SHAFT_LEN = INNER_SPAN + 2.0 * CHEEK_T
WASHER_R_OUT = 0.018
WASHER_R_IN = 0.0085
WASHER_T = 0.002
END_HEAD_R = 0.0105
END_HEAD_T = 0.005

LOWER_LIMIT = -0.50
UPPER_LIMIT = 0.65


def _box(size_x: float, size_y: float, size_z: float, center_xyz: tuple[float, float, float]):
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center_xyz)


def _cyl_x(radius: float, length: float, center_xyz: tuple[float, float, float]):
    cx, cy, cz = center_xyz
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - length / 2.0, cy, cz))
    )


def _washer_x(center_x: float):
    outer = _cyl_x(WASHER_R_OUT, WASHER_T, (center_x, 0.0, 0.0))
    inner = _cyl_x(WASHER_R_IN, WASHER_T + 0.002, (center_x, 0.0, 0.0))
    return outer.cut(inner)


def _cap_screw_z(
    head_r: float,
    head_h: float,
    shank_r: float,
    shank_l: float,
    socket_r: float,
    socket_d: float,
):
    head = cq.Workplane("XY").circle(head_r).extrude(head_h)
    shank = cq.Workplane("XY").circle(shank_r).extrude(shank_l).translate((0.0, 0.0, -shank_l))
    socket = cq.Workplane("XY").circle(socket_r).extrude(socket_d).translate((0.0, 0.0, head_h - socket_d))
    return head.union(shank).cut(socket)


def _end_head_x(center_x: float):
    return _cyl_x(END_HEAD_R, END_HEAD_T, (center_x, 0.0, 0.0))


def _make_base():
    base = _box(BASE_W, BASE_D, BASE_H, (0.0, 0.0, BASE_H / 2.0))
    pocket = _box(
        BASE_POCKET_W,
        BASE_POCKET_D,
        BASE_POCKET_H,
        (0.0, 0.0, BASE_H - BASE_POCKET_H / 2.0),
    )
    return base.cut(pocket)


def _make_cheek_body(inward_sign: float):
    plate = _box(
        CHEEK_T,
        CHEEK_PLATE_D,
        CHEEK_PLATE_H,
        (0.0, 0.0, FOOT_H + CHEEK_PLATE_H / 2.0),
    )
    foot = _box(FOOT_W, FOOT_D, FOOT_H, (0.0, 0.0, FOOT_H / 2.0))
    slot = _box(CHEEK_T + 0.010, 0.104, 0.030, (0.0, 0.0, 0.046))
    boss = _cyl_x(
        PIVOT_BOSS_R,
        CHEEK_T + 2.0 * PIVOT_BOSS_EXTRA,
        (0.0, 0.0, PITCH_LOCAL_Z),
    )
    pivot_hole = _cyl_x(
        PIVOT_HOLE_R,
        CHEEK_T + 2.0 * PIVOT_BOSS_EXTRA + 0.008,
        (0.0, 0.0, PITCH_LOCAL_Z),
    )
    gusset = _box(
        0.006,
        0.078,
        0.038,
        (
            inward_sign * (CHEEK_T / 2.0 + 0.003),
            0.0,
            FOOT_H + 0.019,
        ),
    )
    front_tab = _box(
        TAB_PROJ,
        0.018,
        0.010,
        (
            inward_sign * (CHEEK_T / 2.0 + TAB_PROJ / 2.0),
            0.034,
            0.044,
        ),
    )
    rear_tab = _box(
        TAB_PROJ,
        0.018,
        0.010,
        (
            inward_sign * (CHEEK_T / 2.0 + TAB_PROJ / 2.0),
            -0.034,
            0.044,
        ),
    )
    return plate.union(foot).union(gusset).union(front_tab).union(rear_tab).union(boss).cut(slot).cut(pivot_hole)


def _make_cheek_fasteners():
    screw_a = _cap_screw_z(
        head_r=0.006,
        head_h=0.0045,
        shank_r=0.0022,
        shank_l=0.012,
        socket_r=0.0025,
        socket_d=0.0025,
    ).translate((0.0, 0.030, FOOT_H))
    screw_b = _cap_screw_z(
        head_r=0.006,
        head_h=0.0045,
        shank_r=0.0022,
        shank_l=0.012,
        socket_r=0.0025,
        socket_d=0.0025,
    ).translate((0.0, -0.030, FOOT_H))
    return screw_a.union(screw_b)


def _make_pivot_hardware(outward_sign: float):
    washer_center_x = outward_sign * (CHEEK_T / 2.0 + PIVOT_BOSS_EXTRA + WASHER_T / 2.0)
    head_center_x = outward_sign * (
        CHEEK_T / 2.0 + PIVOT_BOSS_EXTRA + WASHER_T + END_HEAD_T / 2.0
    )
    washer = _washer_x(washer_center_x).translate((0.0, 0.0, PITCH_LOCAL_Z))
    head = _end_head_x(head_center_x).translate((0.0, 0.0, PITCH_LOCAL_Z))
    return washer.union(head)


def _make_tray_pan():
    side_wall_h = TRAY_DEPTH
    floor = _box(
        TRAY_OUTER_W - 2.0 * TRAY_WALL,
        TRAY_LEN - 2.0 * TRAY_WALL,
        TRAY_FLOOR_T,
        (0.0, 0.0, TRAY_PAN_Z - TRAY_DEPTH / 2.0 + TRAY_FLOOR_T / 2.0),
    )
    left_wall = _box(
        TRAY_WALL,
        TRAY_LEN - 2.0 * TRAY_WALL,
        side_wall_h,
        (-TRAY_OUTER_W / 2.0 + TRAY_WALL / 2.0, 0.0, TRAY_PAN_Z),
    )
    right_wall = _box(
        TRAY_WALL,
        TRAY_LEN - 2.0 * TRAY_WALL,
        side_wall_h,
        (TRAY_OUTER_W / 2.0 - TRAY_WALL / 2.0, 0.0, TRAY_PAN_Z),
    )
    front_wall = _box(
        TRAY_OUTER_W - 2.0 * TRAY_WALL,
        TRAY_WALL,
        side_wall_h,
        (0.0, TRAY_LEN / 2.0 - TRAY_WALL / 2.0, TRAY_PAN_Z),
    )
    rear_wall = _box(
        TRAY_OUTER_W - 2.0 * TRAY_WALL,
        TRAY_WALL,
        side_wall_h,
        (0.0, -TRAY_LEN / 2.0 + TRAY_WALL / 2.0, TRAY_PAN_Z),
    )
    left_rim = _box(
        TRAY_RIM_W,
        TRAY_LEN,
        TRAY_RIM_T,
        (-TRAY_OUTER_W / 2.0 + TRAY_RIM_W / 2.0, 0.0, TRAY_PAN_Z + TRAY_DEPTH / 2.0 - TRAY_RIM_T / 2.0),
    )
    right_rim = _box(
        TRAY_RIM_W,
        TRAY_LEN,
        TRAY_RIM_T,
        (TRAY_OUTER_W / 2.0 - TRAY_RIM_W / 2.0, 0.0, TRAY_PAN_Z + TRAY_DEPTH / 2.0 - TRAY_RIM_T / 2.0),
    )
    front_rim = _box(
        TRAY_OUTER_W - 2.0 * TRAY_RIM_W,
        TRAY_RIM_W,
        TRAY_RIM_T,
        (0.0, TRAY_LEN / 2.0 - TRAY_RIM_W / 2.0, TRAY_PAN_Z + TRAY_DEPTH / 2.0 - TRAY_RIM_T / 2.0),
    )
    rear_rim = _box(
        TRAY_OUTER_W - 2.0 * TRAY_RIM_W,
        TRAY_RIM_W,
        TRAY_RIM_T,
        (0.0, -TRAY_LEN / 2.0 + TRAY_RIM_W / 2.0, TRAY_PAN_Z + TRAY_DEPTH / 2.0 - TRAY_RIM_T / 2.0),
    )
    return (
        floor.union(left_wall)
        .union(right_wall)
        .union(front_wall)
        .union(rear_wall)
        .union(left_rim)
        .union(right_rim)
        .union(front_rim)
        .union(rear_rim)
    )


def _make_tray_supports():
    left_plate = _box(
        TRAY_SIDE_PLATE_T,
        TRAY_SIDE_PLATE_W,
        TRAY_SIDE_PLATE_H,
        (-TRAY_SIDE_X, 0.0, TRAY_SIDE_PLATE_Z),
    )
    right_plate = _box(
        TRAY_SIDE_PLATE_T,
        TRAY_SIDE_PLATE_W,
        TRAY_SIDE_PLATE_H,
        (TRAY_SIDE_X, 0.0, TRAY_SIDE_PLATE_Z),
    )
    left_boss = _cyl_x(TRAY_BOSS_R, TRAY_BOSS_L, (-TRAY_SIDE_X, 0.0, 0.0))
    right_boss = _cyl_x(TRAY_BOSS_R, TRAY_BOSS_L, (TRAY_SIDE_X, 0.0, 0.0))
    pivot_hole_left = _cyl_x(TRAY_PIVOT_HOLE_R, TRAY_SIDE_PLATE_T + TRAY_BOSS_L + 0.004, (-TRAY_SIDE_X, 0.0, 0.0))
    pivot_hole_right = _cyl_x(TRAY_PIVOT_HOLE_R, TRAY_SIDE_PLATE_T + TRAY_BOSS_L + 0.004, (TRAY_SIDE_X, 0.0, 0.0))

    stop_centers = (
        (-TRAY_STOP_X, 0.040, 0.000),
        (-TRAY_STOP_X, -0.040, 0.000),
        (TRAY_STOP_X, 0.040, 0.000),
        (TRAY_STOP_X, -0.040, 0.000),
    )
    stop_blocks = None
    for center in stop_centers:
        block = _box(TRAY_STOP_T, TRAY_STOP_Y, TRAY_STOP_Z, center)
        stop_blocks = block if stop_blocks is None else stop_blocks.union(block)

    body = left_plate.union(right_plate).union(left_boss).union(right_boss).union(stop_blocks)
    return body.cut(pivot_hole_left).cut(pivot_hole_right)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_equipment_cradle")

    base_mat = model.material("base_paint", rgba=(0.18, 0.18, 0.20, 1.0))
    support_mat = model.material("support_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    tray_mat = model.material("tray_paint", rgba=(0.23, 0.31, 0.40, 1.0))
    hardware_mat = model.material("hardware", rgba=(0.79, 0.81, 0.84, 1.0))
    fastener_mat = model.material("fastener_black", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base_block")
    base.visual(
        mesh_from_cadquery(_make_base(), "base_block_body"),
        material=base_mat,
        name="base_body",
    )

    left_cheek = model.part("left_cheek")
    left_cheek.visual(
        mesh_from_cadquery(_make_cheek_body(inward_sign=1.0), "left_cheek_body"),
        material=support_mat,
        name="cheek_body",
    )
    left_cheek.visual(
        mesh_from_cadquery(_make_cheek_fasteners(), "left_cheek_fasteners"),
        material=fastener_mat,
        name="cheek_fasteners",
    )
    left_cheek.visual(
        mesh_from_cadquery(_make_pivot_hardware(outward_sign=-1.0), "left_pivot_hardware"),
        material=hardware_mat,
        name="pivot_hardware",
    )

    right_cheek = model.part("right_cheek")
    right_cheek.visual(
        mesh_from_cadquery(_make_cheek_body(inward_sign=-1.0), "right_cheek_body"),
        material=support_mat,
        name="cheek_body",
    )
    right_cheek.visual(
        mesh_from_cadquery(_make_cheek_fasteners(), "right_cheek_fasteners"),
        material=fastener_mat,
        name="cheek_fasteners",
    )
    right_cheek.visual(
        mesh_from_cadquery(_make_pivot_hardware(outward_sign=1.0), "right_pivot_hardware"),
        material=hardware_mat,
        name="pivot_hardware",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_make_tray_pan(), "tray_pan_mesh"),
        material=tray_mat,
        name="tray_pan",
    )
    tray.visual(
        mesh_from_cadquery(_make_tray_supports(), "tray_supports_mesh"),
        material=tray_mat,
        name="tray_supports",
    )

    model.articulation(
        "base_to_left_cheek",
        ArticulationType.FIXED,
        parent=base,
        child=left_cheek,
        origin=Origin(xyz=(-CHEEK_CENTER_X, 0.0, BASE_H)),
    )
    model.articulation(
        "base_to_right_cheek",
        ArticulationType.FIXED,
        parent=base,
        child=right_cheek,
        origin=Origin(xyz=(CHEEK_CENTER_X, 0.0, BASE_H)),
    )
    model.articulation(
        "base_to_tray",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, PITCH_GLOBAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=LOWER_LIMIT,
            upper=UPPER_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_block")
    left_cheek = object_model.get_part("left_cheek")
    right_cheek = object_model.get_part("right_cheek")
    tray = object_model.get_part("tray")
    pitch = object_model.get_articulation("base_to_tray")

    base_body = base.get_visual("base_body")
    left_body = left_cheek.get_visual("cheek_body")
    right_body = right_cheek.get_visual("cheek_body")
    tray_pan = tray.get_visual("tray_pan")
    tray_supports = tray.get_visual("tray_supports")

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
        "pitch_axis_is_lateral",
        tuple(round(v, 6) for v in pitch.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis pitch joint, got {pitch.axis}",
    )
    ctx.check(
        "pitch_limits_match_cradle_stops",
        pitch.motion_limits is not None
        and isclose(pitch.motion_limits.lower or 0.0, LOWER_LIMIT, abs_tol=1e-6)
        and isclose(pitch.motion_limits.upper or 0.0, UPPER_LIMIT, abs_tol=1e-6),
        f"limits={pitch.motion_limits}",
    )

    with ctx.pose({pitch: 0.0}):
        ctx.expect_contact(
            left_cheek,
            base,
            elem_a=left_body,
            elem_b=base_body,
            name="left_cheek_is_seated_on_base",
        )
        ctx.expect_contact(
            right_cheek,
            base,
            elem_a=right_body,
            elem_b=base_body,
            name="right_cheek_is_seated_on_base",
        )
        ctx.expect_overlap(
            tray,
            left_cheek,
            axes="yz",
            min_overlap=0.060,
            elem_a=tray_supports,
            elem_b=left_body,
            name="left_cheek_captures_tray_projection",
        )
        ctx.expect_overlap(
            tray,
            right_cheek,
            axes="yz",
            min_overlap=0.060,
            elem_a=tray_supports,
            elem_b=right_body,
            name="right_cheek_captures_tray_projection",
        )
        ctx.expect_contact(
            tray,
            left_cheek,
            elem_a=tray_supports,
            elem_b=left_body,
            name="left_trunnion_bears_on_left_cheek",
        )
        ctx.expect_contact(
            tray,
            right_cheek,
            elem_a=tray_supports,
            elem_b=right_body,
            name="right_trunnion_bears_on_right_cheek",
        )
        ctx.expect_gap(
            tray,
            left_cheek,
            axis="x",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem=tray_pan,
            negative_elem=left_body,
            name="left_side_running_clearance",
        )
        ctx.expect_gap(
            right_cheek,
            tray,
            axis="x",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem=right_body,
            negative_elem=tray_pan,
            name="right_side_running_clearance",
        )
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.020,
            positive_elem=tray_pan,
            negative_elem=base_body,
            name="tray_clears_base_at_level_pose",
        )

    with ctx.pose({pitch: LOWER_LIMIT}):
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.004,
            positive_elem=tray_pan,
            negative_elem=base_body,
            name="tray_clears_base_at_lower_stop",
        )

    with ctx.pose({pitch: UPPER_LIMIT}):
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.002,
            positive_elem=tray_pan,
            negative_elem=base_body,
            name="tray_clears_base_at_upper_stop",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=9)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
