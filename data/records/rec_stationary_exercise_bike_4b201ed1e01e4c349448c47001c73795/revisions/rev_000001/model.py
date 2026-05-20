from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _ring_shell(
    *,
    name: str,
    inner_radius: float,
    outer_radius: float,
    length: float,
    axis: str = "z",
):
    half = length * 0.5
    shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half),
            (outer_radius, half),
        ],
        [
            (inner_radius, -half),
            (inner_radius, half),
        ],
        segments=56,
    )
    if axis == "x":
        shell.rotate_y(math.pi / 2.0)
    elif axis == "y":
        shell.rotate_x(math.pi / 2.0)
    return _save_mesh(name, shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.17, 0.18, 0.20, 1.0))
    charcoal = model.material("charcoal", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Cylinder(radius=0.032, length=0.62),
        origin=Origin(xyz=(0.0, 0.54, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="front_stabilizer",
    )
    base_frame.visual(
        Cylinder(radius=0.032, length=0.60),
        origin=Origin(xyz=(0.0, -0.48, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_stabilizer",
    )
    for side_name, points in (
        (
            "left_support_rail",
            [
                (-0.22, -0.48, 0.032),
                (-0.24, -0.28, 0.10),
                (-0.26, -0.03, 0.19),
                (-0.20, 0.25, 0.24),
                (-0.13, 0.46, 0.17),
                (-0.10, 0.54, 0.032),
            ],
        ),
        (
            "right_support_rail",
            _mirror_x(
                [
                    (-0.22, -0.48, 0.032),
                    (-0.24, -0.28, 0.10),
                    (-0.26, -0.03, 0.19),
                    (-0.20, 0.25, 0.24),
                    (-0.13, 0.46, 0.17),
                    (-0.10, 0.54, 0.032),
                ]
            ),
        ),
    ):
        base_frame.visual(
            _save_mesh(
                side_name,
                tube_from_spline_points(
                    points,
                    radius=0.024,
                    samples_per_segment=16,
                    radial_segments=18,
                    cap_ends=True,
                ),
            ),
            material=frame_black,
            name=side_name,
        )

    base_frame.visual(
        _ring_shell(
            name="bottom_bracket_shell",
            inner_radius=0.024,
            outer_radius=0.050,
            length=0.34,
            axis="x",
        ),
        origin=Origin(xyz=(0.0, 0.03, 0.22)),
        material=charcoal,
        name="bottom_bracket_shell",
    )
    base_frame.visual(
        _save_mesh(
            "main_spine",
            tube_from_spline_points(
                [
                    (0.0, -0.02, 0.27),
                    (0.0, -0.08, 0.38),
                    (0.0, -0.15, 0.56),
                    (0.0, -0.21, 0.73),
                ],
                radius=0.038,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="main_spine",
    )
    base_frame.visual(
        _ring_shell(
            name="seat_sleeve",
            inner_radius=0.028,
            outer_radius=0.040,
            length=0.16,
        ),
        origin=Origin(xyz=(0.0, -0.27, 0.75)),
        material=charcoal,
        name="seat_sleeve",
    )
    base_frame.visual(
        _save_mesh(
            "handlebar_mast",
            tube_from_spline_points(
                [
                    (0.0, -0.02, 0.26),
                    (0.0, 0.08, 0.42),
                    (0.0, 0.20, 0.69),
                    (0.0, 0.35, 0.92),
                ],
                radius=0.034,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="handlebar_mast",
    )
    for side_name, points in (
        (
            "left_front_upright",
            [
                (-0.11, 0.54, 0.032),
                (-0.11, 0.49, 0.17),
                (-0.11, 0.44, 0.33),
                (-0.11, 0.40, 0.46),
            ],
        ),
        (
            "right_front_upright",
            _mirror_x(
                [
                    (-0.11, 0.54, 0.032),
                    (-0.11, 0.49, 0.17),
                    (-0.11, 0.44, 0.33),
                    (-0.11, 0.40, 0.46),
                ]
            ),
        ),
    ):
        base_frame.visual(
            _save_mesh(
                side_name,
                tube_from_spline_points(
                    points,
                    radius=0.026,
                    samples_per_segment=14,
                    radial_segments=18,
                    cap_ends=True,
                ),
            ),
            material=frame_black,
            name=side_name,
        )
    base_frame.visual(
        Cylinder(radius=0.030, length=0.16),
        origin=Origin(xyz=(0.0, 0.36, 0.98)),
        material=charcoal,
        name="handlebar_stem",
    )
    base_frame.visual(
        _save_mesh(
            "handlebar_bar",
            tube_from_spline_points(
                [
                    (-0.26, 0.35, 1.00),
                    (-0.18, 0.38, 1.03),
                    (-0.05, 0.39, 1.05),
                    (0.05, 0.39, 1.05),
                    (0.18, 0.38, 1.03),
                    (0.26, 0.35, 1.00),
                ],
                radius=0.016,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=rubber,
        name="handlebar_bar",
    )
    base_frame.visual(
        _save_mesh(
            "front_drive_strut",
            tube_from_spline_points(
                [
                    (0.0, 0.04, 0.25),
                    (0.0, 0.12, 0.29),
                    (0.0, 0.22, 0.33),
                    (0.0, 0.30, 0.36),
                ],
                radius=0.024,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="front_drive_strut",
    )
    base_frame.visual(
        _ring_shell(
            name="housing_rim_shell",
            inner_radius=0.218,
            outer_radius=0.255,
            length=0.22,
            axis="x",
        ),
        origin=Origin(xyz=(0.0, 0.50, 0.44)),
        material=charcoal,
        name="housing_rim_shell",
    )
    base_frame.visual(
        Cylinder(radius=0.242, length=0.010),
        origin=Origin(xyz=(-0.105, 0.50, 0.44), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="left_cover",
    )
    base_frame.visual(
        Cylinder(radius=0.242, length=0.010),
        origin=Origin(xyz=(0.105, 0.50, 0.44), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="right_cover",
    )
    base_frame.visual(
        _ring_shell(
            name="left_flywheel_bearing",
            inner_radius=0.038,
            outer_radius=0.058,
            length=0.028,
            axis="x",
        ),
        origin=Origin(xyz=(-0.108, 0.50, 0.44)),
        material=charcoal,
        name="left_flywheel_bearing",
    )
    base_frame.visual(
        _ring_shell(
            name="right_flywheel_bearing",
            inner_radius=0.038,
            outer_radius=0.058,
            length=0.028,
            axis="x",
        ),
        origin=Origin(xyz=(0.108, 0.50, 0.44)),
        material=charcoal,
        name="right_flywheel_bearing",
    )
    base_frame.visual(
        _save_mesh(
            "left_housing_brace",
            tube_from_spline_points(
                [
                    (-0.11, 0.40, 0.46),
                    (-0.108, 0.44, 0.455),
                    (-0.108, 0.50, 0.44),
                ],
                radius=0.018,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=charcoal,
        name="left_housing_brace",
    )
    base_frame.visual(
        _save_mesh(
            "right_housing_brace",
            tube_from_spline_points(
                [
                    (0.11, 0.40, 0.46),
                    (0.108, 0.44, 0.455),
                    (0.108, 0.50, 0.44),
                ],
                radius=0.018,
                samples_per_segment=12,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=charcoal,
        name="right_housing_brace",
    )
    base_frame.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.0, 0.50, 0.735)),
        material=steel,
        name="resistance_stem",
    )
    base_frame.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.0, 0.50, 0.784)),
        material=rubber,
        name="resistance_knob",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.70, 1.16, 1.12)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.03, 0.56)),
    )

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.215, length=0.074),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rim_mass",
    )
    flywheel.visual(
        Cylinder(radius=0.038, length=0.188),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hub",
    )
    flywheel.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="face_cap",
    )
    flywheel.visual(
        Box((0.024, 0.040, 0.016)),
        origin=Origin(xyz=(0.034, 0.0, 0.184)),
        material=frame_black,
        name="rotation_marker",
    )
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.215, length=0.074),
        mass=12.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    crank_set = model.part("crank_set")
    crank_set.visual(
        Cylinder(radius=0.018, length=0.410),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )
    crank_set.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(-0.179, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="left_bb_cup",
    )
    crank_set.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.179, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="right_bb_cup",
    )
    crank_set.visual(
        Cylinder(radius=0.095, length=0.010),
        origin=Origin(xyz=(0.178, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="chainring",
    )
    crank_set.visual(
        Box((0.020, 0.032, 0.200)),
        origin=Origin(xyz=(0.195, 0.030, -0.100)),
        material=charcoal,
        name="right_crank_arm",
    )
    crank_set.visual(
        Box((0.020, 0.032, 0.200)),
        origin=Origin(xyz=(-0.195, -0.030, 0.100)),
        material=charcoal,
        name="left_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.225, 0.055, -0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(-0.225, -0.055, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Box((0.090, 0.030, 0.015)),
        origin=Origin(xyz=(0.225, 0.055, -0.190)),
        material=rubber,
        name="right_pedal",
    )
    crank_set.visual(
        Box((0.090, 0.030, 0.015)),
        origin=Origin(xyz=(-0.225, -0.055, 0.190)),
        material=rubber,
        name="left_pedal",
    )
    crank_set.inertial = Inertial.from_geometry(
        Box((0.34, 0.20, 0.46)),
        mass=4.0,
        origin=Origin(),
    )

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.022, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="post_shaft",
    )
    seat_post.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=charcoal,
        name="seat_clamp",
    )
    seat_post.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="height_stop",
    )
    seat_post.visual(
        Cylinder(radius=0.006, length=0.190),
        origin=Origin(xyz=(-0.028, 0.0, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_seat_rail",
    )
    seat_post.visual(
        Cylinder(radius=0.006, length=0.190),
        origin=Origin(xyz=(0.028, 0.0, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_seat_rail",
    )
    seat_post.visual(
        Box((0.090, 0.100, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=charcoal,
        name="seat_mount",
    )
    seat_post.visual(
        Box((0.185, 0.190, 0.050)),
        origin=Origin(xyz=(0.0, -0.050, 0.390), rpy=(0.10, 0.0, 0.0)),
        material=rubber,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.085, 0.150, 0.040)),
        origin=Origin(xyz=(0.0, 0.105, 0.378), rpy=(0.10, 0.0, 0.0)),
        material=rubber,
        name="saddle_nose",
    )
    seat_post.inertial = Inertial.from_geometry(
        Box((0.24, 0.36, 0.66)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=flywheel,
        origin=Origin(xyz=(0.0, 0.50, 0.44)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=25.0),
    )
    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=crank_set,
        origin=Origin(xyz=(0.0, 0.03, 0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=16.0),
    )
    model.articulation(
        "seat_post_adjust",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=seat_post,
        origin=Origin(xyz=(0.0, -0.27, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    flywheel = object_model.get_part("flywheel")
    crank_set = object_model.get_part("crank_set")
    seat_post = object_model.get_part("seat_post")

    flywheel_joint = object_model.get_articulation("flywheel_spin")
    crank_joint = object_model.get_articulation("crank_rotation")
    seat_joint = object_model.get_articulation("seat_post_adjust")

    def _aabb_center(elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(base_frame, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    def _aabb_dims(elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(base_frame, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(maxs[i] - mins[i] for i in range(3))

    ctx.expect_overlap(
        flywheel,
        base_frame,
        axes="yz",
        elem_a="rim_mass",
        elem_b="housing_rim_shell",
        min_overlap=0.42,
        name="flywheel remains centered within housing footprint",
    )
    ctx.expect_gap(
        flywheel,
        base_frame,
        axis="x",
        positive_elem="hub",
        negative_elem="left_flywheel_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="left flywheel hub shoulder meets bearing",
    )
    ctx.expect_gap(
        base_frame,
        flywheel,
        axis="x",
        positive_elem="right_flywheel_bearing",
        negative_elem="hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="right flywheel hub shoulder meets bearing",
    )
    ctx.expect_gap(
        seat_post,
        base_frame,
        axis="z",
        positive_elem="height_stop",
        negative_elem="seat_sleeve",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat post stop seats on sleeve in lowest position",
    )

    left_support_center = _aabb_center("left_support_rail")
    right_support_center = _aabb_center("right_support_rail")
    left_support_dims = _aabb_dims("left_support_rail")
    right_support_dims = _aabb_dims("right_support_rail")
    ctx.check(
        "left and right support rails stay nearly mirrored",
        left_support_center is not None
        and right_support_center is not None
        and left_support_dims is not None
        and right_support_dims is not None
        and abs(left_support_center[0] + right_support_center[0]) <= 0.01
        and abs(left_support_center[1] - right_support_center[1]) <= 0.005
        and abs(left_support_center[2] - right_support_center[2]) <= 0.005
        and max(abs(a - b) for a, b in zip(left_support_dims, right_support_dims)) <= 0.01,
        details=(
            f"left_center={left_support_center}, right_center={right_support_center}, "
            f"left_dims={left_support_dims}, right_dims={right_support_dims}"
        ),
    )

    left_upright_center = _aabb_center("left_front_upright")
    right_upright_center = _aabb_center("right_front_upright")
    ctx.check(
        "front uprights stay nearly mirrored",
        left_upright_center is not None
        and right_upright_center is not None
        and abs(left_upright_center[0] + right_upright_center[0]) <= 0.01
        and abs(left_upright_center[1] - right_upright_center[1]) <= 0.005
        and abs(left_upright_center[2] - right_upright_center[2]) <= 0.005,
        details=f"left={left_upright_center}, right={right_upright_center}",
    )

    rest_seat_position = ctx.part_world_position(seat_post)
    rest_right_pedal_position = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
    rest_marker_aabb = ctx.part_element_world_aabb(flywheel, elem="rotation_marker")

    with ctx.pose({seat_joint: 0.16}):
        raised_seat_position = ctx.part_world_position(seat_post)
        ctx.check(
            "seat post raises upward along its column",
            rest_seat_position is not None
            and raised_seat_position is not None
            and raised_seat_position[2] >= rest_seat_position[2] + 0.12,
            details=f"rest={rest_seat_position}, raised={raised_seat_position}",
        )

    with ctx.pose({crank_joint: math.pi / 2.0}):
        quarter_turn_right_pedal = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
        pedal_raised = (
            rest_right_pedal_position is not None
            and quarter_turn_right_pedal is not None
            and quarter_turn_right_pedal[0][2] > rest_right_pedal_position[1][2] + 0.02
        )
        ctx.check(
            "crank rotation lifts the right pedal through a quarter turn",
            pedal_raised,
            details=f"rest={rest_right_pedal_position}, quarter_turn={quarter_turn_right_pedal}",
        )

    with ctx.pose({flywheel_joint: math.pi / 2.0}):
        quarter_turn_marker_aabb = ctx.part_element_world_aabb(flywheel, elem="rotation_marker")
        marker_moved = (
            rest_marker_aabb is not None
            and quarter_turn_marker_aabb is not None
            and abs(
                ((quarter_turn_marker_aabb[0][1] + quarter_turn_marker_aabb[1][1]) * 0.5)
                - ((rest_marker_aabb[0][1] + rest_marker_aabb[1][1]) * 0.5)
            )
            > 0.10
        )
        ctx.check(
            "flywheel spin moves the visual rotation marker around the axle",
            marker_moved,
            details=f"rest={rest_marker_aabb}, quarter_turn={quarter_turn_marker_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
