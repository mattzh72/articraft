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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.13, 0.14, 0.15, 1.0))
    shroud_black = model.material("shroud_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.61, 0.64, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.16, 0.16, 0.17, 1.0))
    accent_red = model.material("accent_red", rgba=(0.76, 0.10, 0.10, 1.0))
    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.10, 0.56, 1.25)),
        mass=42.0,
        origin=Origin(xyz=(0.02, 0.0, 0.62)),
    )

    frame.visual(
        Box((0.11, 0.42, 0.055)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0275)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.11, 0.42, 0.055)),
        origin=Origin(xyz=(0.42, 0.0, 0.0275)),
        material=frame_black,
        name="front_stabilizer",
    )
    for x_bar in (-0.32, 0.42):
        for y_foot in (-0.18, 0.18):
            frame.visual(
                Box((0.05, 0.04, 0.014)),
                origin=Origin(xyz=(x_bar, y_foot, 0.007)),
                material=rubber,
            )

    frame.visual(
        _save_mesh(
            "bike_rear_support",
            tube_from_spline_points(
                [
                    (-0.32, 0.0, 0.055),
                    (-0.29, 0.0, 0.20),
                    (-0.22, 0.0, 0.34),
                    (-0.16, 0.0, 0.43),
                ],
                radius=0.034,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="rear_support",
    )
    frame.visual(
        _save_mesh(
            "bike_main_beam",
            tube_from_spline_points(
                [
                    (-0.16, 0.0, 0.43),
                    (-0.06, 0.0, 0.39),
                    (0.03, 0.0, 0.36),
                    (0.10, 0.0, 0.34),
                ],
                radius=0.034,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="main_beam",
    )
    frame.visual(
        _save_mesh(
            "bike_front_mast",
            tube_from_spline_points(
                [
                    (0.10, 0.0, 0.38),
                    (0.22, 0.0, 0.69),
                    (0.32, 0.0, 1.03),
                ],
                radius=0.032,
                samples_per_segment=20,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="front_mast",
    )
    frame.visual(
        _save_mesh(
            "bike_front_brace",
            tube_from_spline_points(
                [
                    (0.42, 0.0, 0.055),
                    (0.33, 0.0, 0.21),
                    (0.23, 0.0, 0.37),
                ],
                radius=0.029,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=frame_black,
        name="front_brace",
    )
    frame.visual(
        Box((0.006, 0.060, 0.40)),
        origin=Origin(xyz=(-0.1875, 0.0, 0.62)),
        material=frame_black,
        name="seat_left_guide",
    )
    frame.visual(
        Box((0.006, 0.060, 0.40)),
        origin=Origin(xyz=(-0.1325, 0.0, 0.62)),
        material=frame_black,
        name="seat_right_guide",
    )
    frame.visual(
        Box((0.061, 0.006, 0.40)),
        origin=Origin(xyz=(-0.16, -0.024, 0.62)),
        material=frame_black,
        name="seat_rear_guide",
    )
    frame.visual(
        Box((0.072, 0.008, 0.030)),
        origin=Origin(xyz=(-0.16, -0.032, 0.805)),
        material=frame_black,
        name="seat_mast_collar",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.19),
        origin=Origin(xyz=(0.02, 0.0, 0.30), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="bottom_bracket_shell",
    )
    frame.visual(
        Box((0.31, 0.09, 0.12)),
        origin=Origin(xyz=(0.11, 0.0, 0.40), rpy=(0.0, 0.72, 0.0)),
        material=shroud_black,
        name="drive_shroud",
    )
    frame.visual(
        _save_mesh(
            "bike_flywheel_rim_cover",
            TorusGeometry(radius=0.235, tube=0.035).rotate_x(pi / 2.0).translate(0.18, 0.0, 0.53),
        ),
        material=shroud_black,
        name="flywheel_rim_cover",
    )
    frame.visual(
        Cylinder(radius=0.245, length=0.008),
        origin=Origin(xyz=(0.18, 0.044, 0.53), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="left_housing_cover",
    )
    frame.visual(
        Cylinder(radius=0.078, length=0.05),
        origin=Origin(xyz=(0.18, 0.022, 0.53), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="housing_hub",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.034),
        origin=Origin(xyz=(0.18, -0.035, 0.53), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="flywheel_bearing_boss",
    )
    frame.visual(
        Box((0.18, 0.10, 0.055)),
        origin=Origin(xyz=(0.29, 0.0, 0.25), rpy=(0.0, 0.25, 0.0)),
        material=frame_black,
        name="mast_socket",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.18),
        origin=Origin(xyz=(0.32, 0.0, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_black,
        name="handlebar_stem",
    )
    frame.visual(
        _save_mesh(
            "bike_handlebar",
            tube_from_spline_points(
                [
                    (0.32, -0.20, 1.02),
                    (0.38, -0.12, 1.10),
                    (0.40, 0.0, 1.14),
                    (0.38, 0.12, 1.10),
                    (0.32, 0.20, 1.02),
                ],
                radius=0.018,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
        ),
        material=frame_black,
        name="handlebar",
    )
    frame.visual(
        Box((0.05, 0.05, 0.12)),
        origin=Origin(xyz=(0.40, 0.0, 1.08)),
        material=frame_black,
        name="bar_riser",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.10),
        origin=Origin(xyz=(0.32, -0.19, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.10),
        origin=Origin(xyz=(0.32, 0.19, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.34, 0.20, 0.72)),
        mass=5.8,
        origin=Origin(xyz=(0.05, 0.0, 0.32)),
    )
    seat_post.visual(
        Box((0.045, 0.036, 0.33)),
        origin=Origin(xyz=(0.0, -0.003, -0.145)),
        material=steel,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.072, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=frame_black,
        name="seat_clamp",
    )
    seat_post.visual(
        Box((0.07, 0.09, 0.050)),
        origin=Origin(xyz=(0.02, 0.0, 0.055)),
        material=frame_black,
        name="seat_head",
    )
    seat_post.visual(
        Cylinder(radius=0.005, length=0.18),
        origin=Origin(xyz=(0.02, -0.038, 0.085), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_rail",
    )
    seat_post.visual(
        Cylinder(radius=0.005, length=0.18),
        origin=Origin(xyz=(0.02, 0.038, 0.085), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_rail",
    )
    seat_post.visual(
        Box((0.20, 0.12, 0.020)),
        origin=Origin(xyz=(0.02, 0.0, 0.100)),
        material=frame_black,
        name="saddle_base",
    )
    seat_post.visual(
        Box((0.22, 0.18, 0.045)),
        origin=Origin(xyz=(-0.01, 0.0, 0.123), rpy=(0.0, -0.06, 0.0)),
        material=saddle_black,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.18, 0.085, 0.032)),
        origin=Origin(xyz=(0.14, 0.0, 0.118), rpy=(0.0, -0.10, 0.0)),
        material=saddle_black,
        name="saddle_nose",
    )

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Box((0.28, 0.32, 0.40)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.234),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_axle",
    )
    crankset.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, -0.104, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="chainring_spider",
    )
    crankset.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.104, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_crank_cap",
    )
    crankset.visual(
        Box((0.026, 0.018, 0.176)),
        origin=Origin(xyz=(0.0, 0.123, -0.088)),
        material=steel,
        name="left_crank_arm",
    )
    crankset.visual(
        Box((0.026, 0.018, 0.176)),
        origin=Origin(xyz=(0.0, -0.123, 0.088)),
        material=steel,
        name="right_crank_arm",
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.153, -0.178), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_pedal_spindle",
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, -0.153, 0.178), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_pedal_spindle",
    )
    crankset.visual(
        Box((0.022, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.138, -0.170)),
        material=steel,
        name="left_pedal_mount",
    )
    crankset.visual(
        Box((0.022, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.138, 0.170)),
        material=steel,
        name="right_pedal_mount",
    )
    crankset.visual(
        Box((0.100, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.153, -0.178)),
        material=rubber,
        name="left_pedal",
    )
    crankset.visual(
        Box((0.100, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.153, 0.178)),
        material=rubber,
        name="right_pedal",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.018),
        mass=11.5,
        origin=Origin(),
    )
    flywheel.visual(
        Cylinder(radius=0.190, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="flywheel_hub",
    )
    flywheel.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="flywheel_axle_cap",
    )
    flywheel.visual(
        Box((0.028, 0.010, 0.020)),
        origin=Origin(xyz=(0.135, 0.0, 0.112)),
        material=accent_red,
        name="flywheel_counterweight",
    )

    model.articulation(
        "seat_height_adjust",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.16, 0.0, 0.82)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.14),
    )
    model.articulation(
        "bottom_bracket_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(0.02, 0.0, 0.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )
    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.18, -0.060, 0.53)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    crankset = object_model.get_part("crankset")
    flywheel = object_model.get_part("flywheel")
    seat_height = object_model.get_articulation("seat_height_adjust")
    crank_joint = object_model.get_articulation("bottom_bracket_crank")
    flywheel_joint = object_model.get_articulation("flywheel_spin")

    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="bottom_bracket_shell",
        elem_b="crank_axle",
        reason="The crank axle intentionally passes through the simplified bottom-bracket shell proxy.",
    )

    ctx.expect_within(
        seat_post,
        frame,
        axes="x",
        inner_elem="inner_post",
        outer_elem="seat_mast_collar",
        margin=0.0,
        name="seat post stays laterally centered in the mast at rest",
    )
    ctx.expect_gap(
        frame,
        seat_post,
        axis="x",
        positive_elem="seat_right_guide",
        negative_elem="inner_post",
        min_gap=0.0,
        max_gap=0.003,
        name="seat post runs close to the right guide at rest",
    )
    ctx.expect_gap(
        seat_post,
        frame,
        axis="x",
        positive_elem="inner_post",
        negative_elem="seat_left_guide",
        min_gap=0.0,
        max_gap=0.003,
        name="seat post runs close to the left guide at rest",
    )
    ctx.expect_gap(
        seat_post,
        frame,
        axis="y",
        positive_elem="inner_post",
        negative_elem="seat_rear_guide",
        min_gap=0.0,
        max_gap=0.001,
        name="seat post bears against the rear guide at rest",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_left_guide",
        min_overlap=0.17,
        name="seat post has deep retained insertion at rest",
    )

    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.14}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="x",
            inner_elem="inner_post",
            outer_elem="seat_mast_collar",
            margin=0.0,
            name="seat post stays laterally centered in the mast when raised",
        )
        ctx.expect_gap(
            frame,
            seat_post,
            axis="x",
            positive_elem="seat_right_guide",
            negative_elem="inner_post",
            min_gap=0.0,
            max_gap=0.003,
            name="seat post runs close to the right guide when raised",
        )
        ctx.expect_gap(
            seat_post,
            frame,
            axis="x",
            positive_elem="inner_post",
            negative_elem="seat_left_guide",
            min_gap=0.0,
            max_gap=0.003,
            name="seat post runs close to the left guide when raised",
        )
        ctx.expect_gap(
            seat_post,
            frame,
            axis="y",
            positive_elem="inner_post",
            negative_elem="seat_rear_guide",
            min_gap=0.0,
            max_gap=0.001,
            name="seat post bears against the rear guide when raised",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_left_guide",
            min_overlap=0.06,
            name="seat post keeps retained insertion when raised",
        )
        raised_pos = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    left_pedal_rest = ctx.part_element_world_aabb(crankset, elem="left_pedal")
    with ctx.pose({crank_joint: pi / 2.0}):
        left_pedal_quarter = ctx.part_element_world_aabb(crankset, elem="left_pedal")

    pedal_motion_ok = False
    if left_pedal_rest is not None and left_pedal_quarter is not None:
        pedal_rest_center = tuple(
            (left_pedal_rest[0][i] + left_pedal_rest[1][i]) / 2.0 for i in range(3)
        )
        pedal_quarter_center = tuple(
            (left_pedal_quarter[0][i] + left_pedal_quarter[1][i]) / 2.0 for i in range(3)
        )
        pedal_motion_ok = (
            pedal_quarter_center[0] < pedal_rest_center[0] - 0.10
            and pedal_quarter_center[2] > pedal_rest_center[2] + 0.10
        )
    else:
        pedal_rest_center = None
        pedal_quarter_center = None
    ctx.check(
        "crank rotates around the bottom bracket axis",
        pedal_motion_ok,
        details=f"rest={pedal_rest_center}, quarter_turn={pedal_quarter_center}",
    )

    counterweight_rest = ctx.part_element_world_aabb(flywheel, elem="flywheel_counterweight")
    with ctx.pose({flywheel_joint: pi / 2.0}):
        counterweight_quarter = ctx.part_element_world_aabb(flywheel, elem="flywheel_counterweight")

    flywheel_motion_ok = False
    if counterweight_rest is not None and counterweight_quarter is not None:
        counterweight_rest_center = tuple(
            (counterweight_rest[0][i] + counterweight_rest[1][i]) / 2.0 for i in range(3)
        )
        counterweight_quarter_center = tuple(
            (counterweight_quarter[0][i] + counterweight_quarter[1][i]) / 2.0 for i in range(3)
        )
        flywheel_motion_ok = (
            counterweight_quarter_center[0] < counterweight_rest_center[0] - 0.015
            and counterweight_quarter_center[2] < counterweight_rest_center[2] - 0.20
        )
    else:
        counterweight_rest_center = None
        counterweight_quarter_center = None
    ctx.check(
        "flywheel rotates about its axle",
        flywheel_motion_ok,
        details=f"rest={counterweight_rest_center}, quarter_turn={counterweight_quarter_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
