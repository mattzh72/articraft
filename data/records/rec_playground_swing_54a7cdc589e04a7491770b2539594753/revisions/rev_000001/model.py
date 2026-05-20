from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


BEAM_Z = 2.10
UPPER_PIVOT_Z = 2.00
LOWER_PIVOT_Z = 1.04
LINK_LENGTH = UPPER_PIVOT_Z - LOWER_PIVOT_Z
PIVOT_X = 0.58
PIVOT_Y = 0.34
BENCH_WIDTH = PIVOT_X * 2.0
BENCH_DEPTH = PIVOT_Y * 2.0

BEAM_SUPPORT_X = 0.82
FRAME_FOOT_X = 0.98
FRAME_FOOT_Y = 0.68

TOP_BEAM_RADIUS = 0.045
LEG_RADIUS = 0.036
BRACE_RADIUS = 0.028
HANGER_ARM_RADIUS = 0.024
EYE_RADIUS = 0.022
EYE_LENGTH = 0.044
LINK_BAR_SIZE = (0.032, 0.018, LINK_LENGTH - 0.044)

CLIP_PLATE_THICKNESS = 0.010
CLIP_PLATE_OFFSET = 0.027
CLIP_PLATE_SIZE = (CLIP_PLATE_THICKNESS, 0.040, 0.080)
CLIP_BRIDGE_SIZE = (0.068, 0.040, 0.012)
CLIP_BRIDGE_Z = -0.055
HANGER_POST_SIZE = (0.050, 0.040, 0.356)
HANGER_POST_Z = -0.230

SWING_LIMIT = 0.55


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_mesh(name: str, start, end, *, radius: float):
    return _save_mesh(
        name,
        wire_from_points(
            [start, end],
            radius=radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode="miter",
        ),
    )


def _add_link_geometry(part, *, upward: bool, bar_material, eye_material) -> None:
    sign = 1.0 if upward else -1.0
    part.visual(
        Cylinder(radius=EYE_RADIUS, length=EYE_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=eye_material,
        name="lower_eye" if upward else "upper_eye",
    )
    part.visual(
        Cylinder(radius=EYE_RADIUS, length=EYE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, sign * LINK_LENGTH), rpy=(0.0, pi / 2.0, 0.0)),
        material=eye_material,
        name="upper_eye" if upward else "lower_eye",
    )
    part.visual(
        Box(LINK_BAR_SIZE),
        origin=Origin(xyz=(0.0, 0.0, sign * LINK_LENGTH * 0.5)),
        material=bar_material,
        name="link_bar",
    )
    part.visual(
        Box((0.040, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, sign * 0.070)),
        material=bar_material,
    )
    part.visual(
        Box((0.040, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, sign * (LINK_LENGTH - 0.070))),
        material=bar_material,
    )


def _add_clip_set(part, x: float, y: float, *, steel, prefix: str) -> None:
    part.visual(
        Box(CLIP_PLATE_SIZE),
        origin=Origin(xyz=(x - CLIP_PLATE_OFFSET, y, -0.030)),
        material=steel,
        name=f"{prefix}_outer_cheek",
    )
    part.visual(
        Box(CLIP_PLATE_SIZE),
        origin=Origin(xyz=(x + CLIP_PLATE_OFFSET, y, -0.030)),
        material=steel,
        name=f"{prefix}_inner_cheek",
    )
    part.visual(
        Box(CLIP_BRIDGE_SIZE),
        origin=Origin(xyz=(x, y, CLIP_BRIDGE_Z)),
        material=steel,
        name=f"{prefix}_bridge",
    )
    part.visual(
        Box(HANGER_POST_SIZE),
        origin=Origin(xyz=(x, y, HANGER_POST_Z)),
        material=steel,
        name=f"{prefix}_post",
    )


def _center_of_aabb(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((a + b) * 0.5 for a, b in zip(minimum, maximum))


def _point_within_tol(actual, expected, tol: float) -> bool:
    return actual is not None and all(abs(a - e) <= tol for a, e in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_face_to_face_glider")

    frame_steel = model.material("frame_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    link_steel = model.material("link_steel", rgba=(0.15, 0.17, 0.18, 1.0))
    pivot_steel = model.material("pivot_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    seat_slat = model.material("seat_slat", rgba=(0.61, 0.43, 0.21, 1.0))
    pad_black = model.material("pad_black", rgba=(0.09, 0.09, 0.10, 1.0))

    top_frame = model.part("top_frame")
    top_frame.inertial = Inertial.from_geometry(
        Box((2.20, 1.60, 2.20)),
        mass=150.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )
    top_frame.visual(
        _tube_mesh("glider_top_beam", (-BEAM_SUPPORT_X, 0.0, BEAM_Z), (BEAM_SUPPORT_X, 0.0, BEAM_Z), radius=TOP_BEAM_RADIUS),
        material=frame_steel,
        name="top_beam",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        top_point = (side_sign * BEAM_SUPPORT_X, 0.0, BEAM_Z)
        front_foot = (side_sign * FRAME_FOOT_X, -FRAME_FOOT_Y, 0.040)
        rear_foot = (side_sign * FRAME_FOOT_X, FRAME_FOOT_Y, 0.040)
        front_mid = tuple((a + b) * 0.5 for a, b in zip(front_foot, top_point))
        rear_mid = tuple((a + b) * 0.5 for a, b in zip(rear_foot, top_point))
        top_frame.visual(
            _tube_mesh(f"{side_name}_front_leg", front_foot, top_point, radius=LEG_RADIUS),
            material=frame_steel,
            name=f"{side_name}_front_leg",
        )
        top_frame.visual(
            _tube_mesh(f"{side_name}_rear_leg", rear_foot, top_point, radius=LEG_RADIUS),
            material=frame_steel,
            name=f"{side_name}_rear_leg",
        )
        top_frame.visual(
            _tube_mesh(f"{side_name}_side_brace", front_mid, rear_mid, radius=BRACE_RADIUS),
            material=frame_steel,
            name=f"{side_name}_side_brace",
        )
        top_frame.visual(
            Box((0.120, 0.220, 0.040)),
            origin=Origin(xyz=(front_foot[0], front_foot[1], 0.020)),
            material=pad_black,
            name=f"{side_name}_front_pad",
        )
        top_frame.visual(
            Box((0.120, 0.220, 0.040)),
            origin=Origin(xyz=(rear_foot[0], rear_foot[1], 0.020)),
            material=pad_black,
            name=f"{side_name}_rear_pad",
        )

    for prefix, x, y in (
        ("lf_upper", -PIVOT_X, -PIVOT_Y),
        ("lr_upper", -PIVOT_X, PIVOT_Y),
        ("rf_upper", PIVOT_X, -PIVOT_Y),
        ("rr_upper", PIVOT_X, PIVOT_Y),
    ):
        top_frame.visual(
            Box(CLIP_PLATE_SIZE),
            origin=Origin(xyz=(x - CLIP_PLATE_OFFSET, y, UPPER_PIVOT_Z + 0.010)),
            material=frame_steel,
            name=f"{prefix}_outer_cheek",
        )
        top_frame.visual(
            Box(CLIP_PLATE_SIZE),
            origin=Origin(xyz=(x + CLIP_PLATE_OFFSET, y, UPPER_PIVOT_Z + 0.010)),
            material=frame_steel,
            name=f"{prefix}_inner_cheek",
        )
        top_frame.visual(
            Box(CLIP_BRIDGE_SIZE),
            origin=Origin(xyz=(x, y, UPPER_PIVOT_Z + 0.049)),
            material=frame_steel,
            name=f"{prefix}_bridge",
        )
        top_frame.visual(
            _tube_mesh(
                f"{prefix}_hanger_arm",
                (x, 0.0, BEAM_Z - 0.010),
                (x, y, UPPER_PIVOT_Z + 0.049),
                radius=HANGER_ARM_RADIUS,
            ),
            material=frame_steel,
            name=f"{prefix}_hanger_arm",
        )

    left_front_link = model.part("left_front_link")
    left_front_link.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, LINK_LENGTH)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )
    _add_link_geometry(left_front_link, upward=False, bar_material=link_steel, eye_material=pivot_steel)

    bench_frame = model.part("bench_frame")
    bench_frame.inertial = Inertial.from_geometry(
        Box((1.30, 0.90, 0.70)),
        mass=48.0,
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, BENCH_DEPTH * 0.5, -0.220)),
    )

    for prefix, x, y in (
        ("lf_clip", 0.0, 0.0),
        ("lr_clip", 0.0, BENCH_DEPTH),
        ("rf_clip", BENCH_WIDTH, 0.0),
        ("rr_clip", BENCH_WIDTH, BENCH_DEPTH),
    ):
        _add_clip_set(bench_frame, x, y, steel=frame_steel, prefix=prefix)

    bench_frame.visual(
        Box((0.040, BENCH_DEPTH, 0.040)),
        origin=Origin(xyz=(0.0, BENCH_DEPTH * 0.5, -0.410)),
        material=frame_steel,
        name="left_side_rail",
    )
    bench_frame.visual(
        Box((0.040, BENCH_DEPTH, 0.040)),
        origin=Origin(xyz=(BENCH_WIDTH, BENCH_DEPTH * 0.5, -0.410)),
        material=frame_steel,
        name="right_side_rail",
    )
    for name, y in (
        ("front_outer_rail", 0.030),
        ("front_footwell_rail", 0.270),
        ("rear_footwell_rail", 0.410),
        ("rear_outer_rail", 0.650),
    ):
        bench_frame.visual(
            Box((BENCH_WIDTH, 0.040, 0.040)),
            origin=Origin(xyz=(BENCH_WIDTH * 0.5, y, -0.410)),
            material=frame_steel,
            name=name,
        )

    for name, y in (
        ("front_seat_slat_1", 0.110),
        ("front_seat_slat_2", 0.175),
        ("front_seat_slat_3", 0.240),
        ("rear_seat_slat_1", 0.440),
        ("rear_seat_slat_2", 0.505),
        ("rear_seat_slat_3", 0.570),
    ):
        bench_frame.visual(
            Box((1.160, 0.045, 0.024)),
            origin=Origin(xyz=(BENCH_WIDTH * 0.5, y, -0.395)),
            material=seat_slat,
            name=name,
        )

    for x in (0.080, BENCH_WIDTH - 0.080):
        bench_frame.visual(
            Box((0.040, 0.030, 0.430)),
            origin=Origin(xyz=(x, 0.030, -0.215)),
            material=frame_steel,
            name=f"front_backrest_post_{'left' if x < BENCH_WIDTH * 0.5 else 'right'}",
        )
        bench_frame.visual(
            Box((0.040, 0.030, 0.430)),
            origin=Origin(xyz=(x, 0.650, -0.215)),
            material=frame_steel,
            name=f"rear_backrest_post_{'left' if x < BENCH_WIDTH * 0.5 else 'right'}",
        )

    for z, index in ((-0.260, 1), (-0.180, 2), (-0.100, 3)):
        bench_frame.visual(
            Box((1.000, 0.028, 0.024)),
            origin=Origin(xyz=(BENCH_WIDTH * 0.5, 0.030, z)),
            material=seat_slat,
            name=f"front_backrest_slat_{index}",
        )
        bench_frame.visual(
            Box((1.000, 0.028, 0.024)),
            origin=Origin(xyz=(BENCH_WIDTH * 0.5, 0.650, z)),
            material=seat_slat,
            name=f"rear_backrest_slat_{index}",
        )

    bench_frame.visual(
        Box((1.040, 0.030, 0.030)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, 0.030, -0.020)),
        material=frame_steel,
        name="front_backrest_top_rail",
    )
    bench_frame.visual(
        Box((1.040, 0.030, 0.030)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, 0.650, -0.020)),
        material=frame_steel,
        name="rear_backrest_top_rail",
    )

    left_rear_link = model.part("left_rear_link")
    left_rear_link.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, LINK_LENGTH)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, LINK_LENGTH * 0.5)),
    )
    _add_link_geometry(left_rear_link, upward=True, bar_material=link_steel, eye_material=pivot_steel)

    right_front_link = model.part("right_front_link")
    right_front_link.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, LINK_LENGTH)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, LINK_LENGTH * 0.5)),
    )
    _add_link_geometry(right_front_link, upward=True, bar_material=link_steel, eye_material=pivot_steel)

    right_rear_link = model.part("right_rear_link")
    right_rear_link.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, LINK_LENGTH)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, LINK_LENGTH * 0.5)),
    )
    _add_link_geometry(right_rear_link, upward=True, bar_material=link_steel, eye_material=pivot_steel)

    model.articulation(
        "left_front_upper_swing",
        ArticulationType.REVOLUTE,
        parent=top_frame,
        child=left_front_link,
        origin=Origin(xyz=(-PIVOT_X, -PIVOT_Y, UPPER_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT),
    )
    model.articulation(
        "bench_left_front_counter",
        ArticulationType.REVOLUTE,
        parent=left_front_link,
        child=bench_frame,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "left_rear_lower_pivot",
        ArticulationType.REVOLUTE,
        parent=bench_frame,
        child=left_rear_link,
        origin=Origin(xyz=(0.0, BENCH_DEPTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT),
    )
    model.articulation(
        "right_front_lower_pivot",
        ArticulationType.REVOLUTE,
        parent=bench_frame,
        child=right_front_link,
        origin=Origin(xyz=(BENCH_WIDTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT),
    )
    model.articulation(
        "right_rear_lower_pivot",
        ArticulationType.REVOLUTE,
        parent=bench_frame,
        child=right_rear_link,
        origin=Origin(xyz=(BENCH_WIDTH, BENCH_DEPTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_frame = object_model.get_part("top_frame")
    left_front_link = object_model.get_part("left_front_link")
    bench_frame = object_model.get_part("bench_frame")
    left_rear_link = object_model.get_part("left_rear_link")
    right_front_link = object_model.get_part("right_front_link")
    right_rear_link = object_model.get_part("right_rear_link")

    left_front_upper = object_model.get_articulation("left_front_upper_swing")
    bench_counter = object_model.get_articulation("bench_left_front_counter")
    left_rear_lower = object_model.get_articulation("left_rear_lower_pivot")
    right_front_lower = object_model.get_articulation("right_front_lower_pivot")
    right_rear_lower = object_model.get_articulation("right_rear_lower_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_glider_joints_rotate_about_beam_axis",
        all(
            joint.axis == (1.0, 0.0, 0.0)
            for joint in (
                left_front_upper,
                bench_counter,
                left_rear_lower,
                right_front_lower,
                right_rear_lower,
            )
        ),
        details="Each glider pivot should rotate about the overhead beam's X axis.",
    )

    ctx.expect_gap(
        top_frame,
        bench_frame,
        axis="z",
        positive_elem="top_beam",
        min_gap=0.85,
        max_gap=1.15,
        name="bench_hangs_well_below_top_beam",
    )

    swing_pose = {
        left_front_upper: 0.38,
        bench_counter: -0.38,
        left_rear_lower: 0.38,
        right_front_lower: 0.38,
        right_rear_lower: 0.38,
    }
    with ctx.pose(swing_pose):
        upper_targets = (
            (left_front_link, "upper_eye", (-PIVOT_X, -PIVOT_Y, UPPER_PIVOT_Z)),
            (left_rear_link, "upper_eye", (-PIVOT_X, PIVOT_Y, UPPER_PIVOT_Z)),
            (right_front_link, "upper_eye", (PIVOT_X, -PIVOT_Y, UPPER_PIVOT_Z)),
            (right_rear_link, "upper_eye", (PIVOT_X, PIVOT_Y, UPPER_PIVOT_Z)),
        )
        upper_ok = True
        upper_details = []
        for part, elem, expected in upper_targets:
            actual = _center_of_aabb(ctx.part_element_world_aabb(part, elem=elem))
            if not _point_within_tol(actual, expected, tol=0.012):
                upper_ok = False
                upper_details.append(f"{part.name}:{elem} actual={actual} expected={expected}")
        ctx.check(
            "parallel_links_stay_seated_in_upper_beam_pivots",
            upper_ok,
            details="; ".join(upper_details),
        )

        bench_origin = ctx.part_world_position(bench_frame)
        lower_targets = (
            (left_front_link, "lower_eye", bench_origin),
            (
                left_rear_link,
                "lower_eye",
                None if bench_origin is None else (bench_origin[0], bench_origin[1] + BENCH_DEPTH, bench_origin[2]),
            ),
            (
                right_front_link,
                "lower_eye",
                None if bench_origin is None else (bench_origin[0] + BENCH_WIDTH, bench_origin[1], bench_origin[2]),
            ),
            (
                right_rear_link,
                "lower_eye",
                None if bench_origin is None else (bench_origin[0] + BENCH_WIDTH, bench_origin[1] + BENCH_DEPTH, bench_origin[2]),
            ),
        )
        lower_ok = True
        lower_details = []
        for part, elem, expected in lower_targets:
            actual = _center_of_aabb(ctx.part_element_world_aabb(part, elem=elem))
            if not _point_within_tol(actual, expected, tol=0.012):
                lower_ok = False
                lower_details.append(f"{part.name}:{elem} actual={actual} expected={expected}")
        ctx.check(
            "bench_frame_stays_clipped_to_lower_link_pivots",
            lower_ok,
            details="; ".join(lower_details),
        )

        front_slat = _center_of_aabb(ctx.part_element_world_aabb(bench_frame, elem="front_seat_slat_2"))
        rear_slat = _center_of_aabb(ctx.part_element_world_aabb(bench_frame, elem="rear_seat_slat_2"))
        seat_level_ok = (
            front_slat is not None
            and rear_slat is not None
            and abs(front_slat[2] - rear_slat[2]) <= 0.010
        )
        ctx.check(
            "face_to_face_bench_stays_level_in_swing_pose",
            seat_level_ok,
            details=f"front_slat={front_slat}, rear_slat={rear_slat}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
