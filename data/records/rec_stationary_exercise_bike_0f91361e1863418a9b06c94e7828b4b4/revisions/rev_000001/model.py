from __future__ import annotations

from math import pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _unit(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _scale(vector: tuple[float, float, float], amount: float) -> tuple[float, float, float]:
    return (vector[0] * amount, vector[1] * amount, vector[2] * amount)


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _tube(name: str, points, *, radius: float, segments: int = 16):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=segments,
            cap_ends=True,
        ),
        name,
    )


def _saddle_mesh():
    # A low, broad, rounded saddle with a tapered forward nose.
    return superellipse_side_loft(
        [
            (-0.13, 0.24, 0.026, 0.052),
            (-0.03, 0.25, 0.030, 0.058),
            (0.10, 0.16, 0.026, 0.050),
            (0.23, 0.08, 0.020, 0.038),
        ],
        exponents=2.2,
        segments=44,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.025, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    flywheel_steel = model.material("flywheel_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.05, 0.26, 0.78, 1.0))
    vinyl = model.material("black_vinyl", rgba=(0.025, 0.023, 0.022, 1.0))

    frame = model.part("frame")

    # Bike frame coordinates: +X is forward toward the flywheel and handlebars,
    # +Y is rider-left/right across the bike, and +Z is upward.
    bottom_bracket = (-0.08, 0.0, 0.50)
    flywheel_axle = (0.43, 0.0, 0.43)
    seat_base = (-0.33, 0.0, 0.39)
    seat_sleeve_top = (-0.43, 0.0, 0.80)
    seat_axis = _unit(
        (
            seat_sleeve_top[0] - seat_base[0],
            seat_sleeve_top[1] - seat_base[1],
            seat_sleeve_top[2] - seat_base[2],
        )
    )

    # Floor contact rails and stabilizer feet.
    frame.visual(
        Cylinder(radius=0.035, length=0.66),
        origin=Origin(xyz=(0.55, 0.0, 0.04), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="front_foot",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.68),
        origin=Origin(xyz=(-0.48, 0.0, 0.04), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="rear_foot",
    )
    for y, name in ((0.22, "left_floor_rail"), (-0.22, "right_floor_rail")):
        frame.visual(
            _tube(
                name,
                [(-0.50, y, 0.055), (-0.10, y, 0.060), (0.57, y, 0.055)],
                radius=0.020,
            ),
            material=dark_frame,
            name=name,
        )
    for x, y, name in (
        (0.55, 0.32, "front_left_leveler"),
        (0.55, -0.32, "front_right_leveler"),
        (-0.48, 0.32, "rear_left_leveler"),
        (-0.48, -0.32, "rear_right_leveler"),
    ):
        frame.visual(
            Box((0.14, 0.07, 0.026)),
            origin=Origin(xyz=(x, y, 0.017)),
            material=rubber,
            name=name,
        )

    # Symmetric paired support triangles on either side of the centerline.
    left_points = [
        (-0.48, 0.27, 0.075),
        (-0.30, 0.28, 0.24),
        (bottom_bracket[0], 0.285, bottom_bracket[2]),
        (0.18, 0.28, 0.36),
        (flywheel_axle[0], 0.27, flywheel_axle[2]),
    ]
    right_points = [(x, -y, z) for (x, y, z) in left_points]
    frame.visual(
        _tube("left_support", left_points, radius=0.024, segments=18),
        material=dark_frame,
        name="left_support",
    )
    frame.visual(
        _tube("right_support", right_points, radius=0.024, segments=18),
        material=dark_frame,
        name="right_support",
    )
    frame.visual(
        _tube(
            "center_spine",
            [(-0.48, 0.0, 0.08), (-0.30, 0.0, 0.25), (-0.08, 0.0, 0.42)],
            radius=0.028,
        ),
        material=dark_frame,
        name="center_spine",
    )

    # Seat sleeve and handlebar mast are supported by the same floor frame.
    frame.visual(
        _tube("outer_seat_sleeve", [seat_base, seat_sleeve_top], radius=0.033, segments=20),
        material=graphite,
        name="outer_seat_sleeve",
    )
    frame.visual(
        _tube("seat_stay_left", [(-0.46, 0.20, 0.07), (-0.40, 0.11, 0.38), (-0.40, 0.050, 0.73)], radius=0.019),
        material=dark_frame,
        name="seat_stay_left",
    )
    frame.visual(
        _tube("seat_stay_right", [(-0.46, -0.20, 0.07), (-0.40, -0.11, 0.38), (-0.40, -0.050, 0.73)], radius=0.019),
        material=dark_frame,
        name="seat_stay_right",
    )
    handle_top = (0.31, 0.0, 1.06)
    frame.visual(
        _tube("handlebar_mast", [(0.10, 0.0, 0.35), (0.20, 0.0, 0.72), handle_top], radius=0.030),
        material=graphite,
        name="handlebar_mast",
    )
    frame.visual(
        _tube("front_mast_stay", [(-0.05, 0.0, 0.55), (0.02, 0.0, 0.45), (0.10, 0.0, 0.35)], radius=0.020),
        material=dark_frame,
        name="front_mast_stay",
    )
    frame.visual(
        _tube(
            "handlebar",
            [
                (0.08, 0.0, 1.03),
                (0.21, 0.0, 1.11),
                (0.31, 0.0, 1.13),
                (0.41, 0.0, 1.11),
                (0.54, 0.0, 1.03),
            ],
            radius=0.018,
            segments=18,
        ),
        material=dark_frame,
        name="handlebar",
    )
    frame.visual(
        _tube("handlebar_stem", [handle_top, (0.31, 0.0, 1.13)], radius=0.018),
        material=dark_frame,
        name="handlebar_stem",
    )
    for y, name in ((0.23, "left_grip"), (-0.23, "right_grip")):
        frame.visual(
            Cylinder(radius=0.023, length=0.20),
            origin=Origin(xyz=(0.50, y, 1.035), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=name,
        )
        frame.visual(
            _tube(name + "_stub", [(0.45, 0.0, 1.05), (0.49, y * 0.55, 1.04), (0.50, y, 1.035)], radius=0.016),
            material=dark_frame,
            name=name + "_stub",
        )
    frame.visual(
        Box((0.18, 0.035, 0.10)),
        origin=Origin(xyz=(0.33, 0.0, 1.13), rpy=(0.20, 0.0, 0.0)),
        material=satin_black,
        name="display_console",
    )
    frame.visual(
        Box((0.07, 0.035, 0.055)),
        origin=Origin(xyz=(0.32, 0.0, 1.10)),
        material=graphite,
        name="console_mount",
    )

    # Flywheel guard: a round shroud ring with side collars and fork braces.
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.305, 0.024, radial_segments=64, tubular_segments=16).rotate_x(pi / 2.0), "flywheel_guard_ring"),
        origin=Origin(xyz=flywheel_axle),
        material=satin_black,
        name="flywheel_guard_ring",
    )
    frame.visual(
        Cylinder(radius=0.047, length=0.055),
        origin=Origin(xyz=(flywheel_axle[0], 0.135, flywheel_axle[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="left_axle_collar",
    )
    frame.visual(
        _tube(
            "left_axle_collar_fork",
            [(0.08, 0.22, 0.075), (0.22, 0.17, 0.29), (flywheel_axle[0], 0.18, flywheel_axle[2] + 0.045), (0.59, 0.22, 0.075)],
            radius=0.019,
        ),
        material=dark_frame,
        name="left_axle_collar_fork",
    )
    frame.visual(
        Cylinder(radius=0.047, length=0.055),
        origin=Origin(xyz=(flywheel_axle[0], -0.135, flywheel_axle[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="right_axle_collar",
    )
    frame.visual(
        _tube(
            "right_axle_collar_fork",
            [(0.08, -0.22, 0.075), (0.22, -0.17, 0.29), (flywheel_axle[0], -0.18, flywheel_axle[2] + 0.045), (0.59, -0.22, 0.075)],
            radius=0.019,
        ),
        material=dark_frame,
        name="right_axle_collar_fork",
    )
    frame.visual(
        Box((0.10, 0.14, 0.07)),
        origin=Origin(xyz=(flywheel_axle[0], 0.0, flywheel_axle[2] - 0.31)),
        material=satin_black,
        name="guard_lower_bridge",
    )

    # Bottom-bracket bearing shell that captures the crank axle.
    frame.visual(
        Cylinder(radius=0.058, length=0.14),
        origin=Origin(xyz=bottom_bracket, rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="bottom_bracket_shell",
    )
    frame.visual(
        _tube(
            "bb_to_seat",
            [(bottom_bracket[0] - 0.02, 0.065, bottom_bracket[2] + 0.055), (-0.23, 0.075, 0.61), (-0.40, 0.070, 0.74)],
            radius=0.020,
        ),
        material=dark_frame,
        name="bb_to_seat",
    )

    # Continuously spinning flywheel.
    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.255, length=0.045),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=flywheel_steel,
        name="flywheel_disk",
    )
    flywheel.visual(
        Cylinder(radius=0.072, length=0.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="flywheel_hub",
    )
    flywheel.visual(
        Cylinder(radius=0.022, length=0.33),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="flywheel_axle",
    )
    flywheel.visual(
        Box((0.045, 0.014, 0.09)),
        origin=Origin(xyz=(0.0, -0.028, 0.15)),
        material=accent_blue,
        name="flywheel_marker",
    )

    # Continuously rotating crank set with opposed crank arms and pedals.
    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.020, length=0.43),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="crank_axle",
    )
    crank.visual(
        Cylinder(radius=0.135, length=0.016),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="chainring",
    )
    crank.visual(
        Box((0.035, 0.026, 0.34)),
        origin=Origin(xyz=(0.0, -0.215, -0.17)),
        material=brushed_steel,
        name="right_crank_arm",
    )
    crank.visual(
        Box((0.035, 0.026, 0.34)),
        origin=Origin(xyz=(0.0, 0.215, 0.17)),
        material=brushed_steel,
        name="left_crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.0, -0.275, -0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="right_pedal_spindle",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.0, 0.275, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="left_pedal_spindle",
    )
    crank.visual(
        Box((0.15, 0.065, 0.032)),
        origin=Origin(xyz=(0.0, -0.335, -0.34)),
        material=rubber,
        name="right_pedal",
    )
    crank.visual(
        Box((0.15, 0.065, 0.032)),
        origin=Origin(xyz=(0.0, 0.335, 0.34)),
        material=rubber,
        name="left_pedal",
    )

    # Prismatic seat post: the inner post remains hidden in the sleeve at both
    # ends of travel, while the saddle rides along the tilted seat column.
    seat_post = model.part("seat_post")
    inner_start = _scale(seat_axis, -0.30)
    inner_end = _scale(seat_axis, 0.26)
    saddle_center = _add(_scale(seat_axis, 0.31), (0.05, 0.0, 0.025))
    seat_post.visual(
        _tube("inner_post", [inner_start, inner_end], radius=0.021, segments=20),
        material=brushed_steel,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.18, 0.13, 0.035)),
        origin=Origin(xyz=_add(_scale(seat_axis, 0.24), (0.02, 0.0, 0.025))),
        material=graphite,
        name="saddle_clamp",
    )
    for y, name in ((0.045, "left_saddle_rail"), (-0.045, "right_saddle_rail")):
        seat_post.visual(
            _tube(name, [(-0.085, y, 0.280), (0.03, y, 0.318), (0.125, y, 0.352)], radius=0.010, segments=12),
            material=brushed_steel,
            name=name,
        )
    seat_post.visual(
        Box((0.055, 0.10, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=graphite,
        name="saddle_support",
    )
    seat_post.visual(
        mesh_from_geometry(_saddle_mesh(), "saddle_pad"),
        origin=Origin(xyz=saddle_center),
        material=vinyl,
        name="saddle_pad",
    )

    model.articulation(
        "frame_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=flywheel_axle),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=35.0),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=bottom_bracket),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=seat_sleeve_top),
        axis=seat_axis,
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    flywheel = object_model.get_part("flywheel")
    crank = object_model.get_part("crank")
    seat_post = object_model.get_part("seat_post")
    flywheel_joint = object_model.get_articulation("frame_to_flywheel")
    crank_joint = object_model.get_articulation("frame_to_crank")
    seat_joint = object_model.get_articulation("frame_to_seat_post")

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bottom_bracket_shell",
        elem_b="crank_axle",
        reason="The crank axle is intentionally captured inside the bottom-bracket bearing shell.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="left_axle_collar",
        elem_b="flywheel_axle",
        reason="The flywheel axle is intentionally seated through the left frame bearing collar.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="right_axle_collar",
        elem_b="flywheel_axle",
        reason="The flywheel axle is intentionally seated through the right frame bearing collar.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="outer_seat_sleeve",
        elem_b="inner_post",
        reason="The adjustable inner seat post telescopes inside the outer seat sleeve.",
    )

    ctx.check(
        "specified joint types",
        flywheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and seat_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"flywheel={flywheel_joint.articulation_type}, crank={crank_joint.articulation_type}, seat={seat_joint.articulation_type}",
    )

    ctx.expect_overlap(
        frame,
        crank,
        axes="y",
        elem_a="bottom_bracket_shell",
        elem_b="crank_axle",
        min_overlap=0.10,
        name="crank axle retained in bottom bracket",
    )
    ctx.expect_overlap(
        frame,
        flywheel,
        axes="y",
        elem_a="left_axle_collar",
        elem_b="flywheel_axle",
        min_overlap=0.02,
        name="left flywheel axle collar retained",
    )
    ctx.expect_overlap(
        frame,
        flywheel,
        axes="y",
        elem_a="right_axle_collar",
        elem_b="flywheel_axle",
        min_overlap=0.02,
        name="right flywheel axle collar retained",
    )
    ctx.expect_overlap(
        frame,
        seat_post,
        axes="z",
        elem_a="outer_seat_sleeve",
        elem_b="inner_post",
        min_overlap=0.20,
        name="collapsed seat post remains inserted",
    )

    rest_seat = ctx.part_world_position(seat_post)
    with ctx.pose({seat_joint: 0.16}):
        raised_seat = ctx.part_world_position(seat_post)
        ctx.expect_overlap(
            frame,
            seat_post,
            axes="z",
            elem_a="outer_seat_sleeve",
            elem_b="inner_post",
            min_overlap=0.05,
            name="raised seat post remains inserted",
        )
    ctx.check(
        "seat post raises along tilted column",
        rest_seat is not None
        and raised_seat is not None
        and raised_seat[2] > rest_seat[2] + 0.14
        and raised_seat[0] < rest_seat[0] - 0.02,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    def _elem_center_x(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    crank_x0 = _elem_center_x(crank, "right_pedal")
    with ctx.pose({crank_joint: pi / 2.0}):
        crank_x90 = _elem_center_x(crank, "right_pedal")
    ctx.check(
        "crank visibly rotates about bottom bracket axis",
        crank_x0 is not None and crank_x90 is not None and abs(crank_x90 - crank_x0) > 0.20,
        details=f"right_pedal_x_rest={crank_x0}, right_pedal_x_quarter_turn={crank_x90}",
    )

    marker_x0 = _elem_center_x(flywheel, "flywheel_marker")
    with ctx.pose({flywheel_joint: pi / 2.0}):
        marker_x90 = _elem_center_x(flywheel, "flywheel_marker")
    ctx.check(
        "flywheel marker rotates about axle",
        marker_x0 is not None and marker_x90 is not None and abs(marker_x90 - marker_x0) > 0.09,
        details=f"marker_x_rest={marker_x0}, marker_x_quarter_turn={marker_x90}",
    )

    left_aabb = ctx.part_element_world_aabb(frame, elem="left_support")
    right_aabb = ctx.part_element_world_aabb(frame, elem="right_support")
    if left_aabb is not None and right_aabb is not None:
        left_center_y = (left_aabb[0][1] + left_aabb[1][1]) * 0.5
        right_center_y = (right_aabb[0][1] + right_aabb[1][1]) * 0.5
        left_width = left_aabb[1][1] - left_aabb[0][1]
        right_width = right_aabb[1][1] - right_aabb[0][1]
        symmetric = abs(left_center_y + right_center_y) < 0.012 and abs(left_width - right_width) < 0.012
    else:
        symmetric = False
        left_center_y = right_center_y = left_width = right_width = None
    ctx.check(
        "left and right support tubes mirror about centerline",
        symmetric,
        details=f"left_center_y={left_center_y}, right_center_y={right_center_y}, left_width={left_width}, right_width={right_width}",
    )

    return ctx.report()


object_model = build_object_model()
