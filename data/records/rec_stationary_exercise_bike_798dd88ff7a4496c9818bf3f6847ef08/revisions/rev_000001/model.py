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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.15, 0.16, 0.17, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    def tube(name: str, points: list[tuple[float, float, float]], radius: float):
        return mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=14,
                radial_segments=18,
            ),
            name,
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.15, 0.52, 1.35)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.02, 0.64)),
    )

    frame.visual(
        Box((0.46, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, -0.38, 0.025)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.42, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.46, 0.025)),
        material=frame_black,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.10, 0.08, 0.018)),
        origin=Origin(xyz=(0.0, -0.38, -0.004)),
        material=rubber,
        name="rear_pad_strip",
    )
    frame.visual(
        Box((0.10, 0.08, 0.018)),
        origin=Origin(xyz=(0.0, 0.46, -0.004)),
        material=rubber,
        name="front_pad_strip",
    )

    frame.visual(
        tube(
            "main_spine",
            [
                (0.0, -0.30, 0.08),
                (0.0, -0.18, 0.09),
                (0.0, 0.02, 0.10),
                (0.0, 0.18, 0.11),
            ],
            radius=0.030,
        ),
        material=frame_black,
        name="main_spine",
    )
    frame.visual(
        tube(
            "front_mast",
            [
                (0.0, 0.20, 0.13),
                (0.0, 0.25, 0.42),
                (0.0, 0.29, 0.84),
                (0.0, 0.28, 1.12),
            ],
            radius=0.030,
        ),
        material=frame_black,
        name="front_mast",
    )
    frame.visual(
        tube(
            "seat_brace",
            [
                (0.0, -0.24, 0.08),
                (0.0, -0.20, 0.21),
                (0.0, -0.17, 0.34),
            ],
            radius=0.026,
        ),
        material=frame_black,
        name="seat_brace",
    )
    frame.visual(
        tube(
            "top_link",
            [
                (0.0, -0.045, 0.67),
                (0.0, 0.03, 0.66),
                (0.0, 0.17, 0.61),
            ],
            radius=0.022,
        ),
        material=frame_black,
        name="top_link",
    )

    frame.visual(
        Box((0.012, 0.31, 0.23)),
        origin=Origin(xyz=(-0.075, 0.105, 0.385)),
        material=housing_gray,
        name="housing_left_upper_cover",
    )
    frame.visual(
        Box((0.012, 0.12, 0.11)),
        origin=Origin(xyz=(-0.075, 0.17, 0.165)),
        material=housing_gray,
        name="housing_left_front_cheek",
    )
    frame.visual(
        Box((0.012, 0.07, 0.09)),
        origin=Origin(xyz=(-0.075, -0.06, 0.135)),
        material=housing_gray,
        name="housing_left_rear_cheek",
    )
    frame.visual(
        Box((0.012, 0.31, 0.23)),
        origin=Origin(xyz=(0.075, 0.105, 0.385)),
        material=housing_gray,
        name="housing_right_upper_cover",
    )
    frame.visual(
        Box((0.012, 0.12, 0.11)),
        origin=Origin(xyz=(0.075, 0.17, 0.165)),
        material=housing_gray,
        name="housing_right_front_cheek",
    )
    frame.visual(
        Box((0.012, 0.07, 0.09)),
        origin=Origin(xyz=(0.075, -0.06, 0.135)),
        material=housing_gray,
        name="housing_right_rear_cheek",
    )
    frame.visual(
        Box((0.146, 0.25, 0.04)),
        origin=Origin(xyz=(0.0, 0.10, 0.505)),
        material=housing_gray,
        name="housing_top_cover",
    )
    frame.visual(
        Box((0.146, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.09, 0.080)),
        material=housing_gray,
        name="housing_bottom_cover",
    )
    frame.visual(
        Box((0.146, 0.05, 0.19)),
        origin=Origin(xyz=(0.0, 0.255, 0.28)),
        material=housing_gray,
        name="housing_front_cover",
    )
    frame.visual(
        Box((0.146, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, -0.095, 0.130)),
        material=housing_gray,
        name="housing_rear_cover",
    )
    frame.visual(
        Box((0.026, 0.10, 0.12)),
        origin=Origin(xyz=(-0.062, 0.02, 0.130)),
        material=frame_black,
        name="bottom_bracket_left_web",
    )
    frame.visual(
        Box((0.026, 0.10, 0.12)),
        origin=Origin(xyz=(0.062, 0.02, 0.130)),
        material=frame_black,
        name="bottom_bracket_right_web",
    )
    frame.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.02, 0.150)),
        material=frame_black,
        name="bottom_bracket_bridge",
    )
    frame.visual(
        Box((0.10, 0.16, 0.11)),
        origin=Origin(xyz=(0.0, 0.23, 0.075)),
        material=frame_black,
        name="front_support_pedestal",
    )
    frame.visual(
        Box((0.08, 0.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.36, 0.075)),
        material=frame_black,
        name="front_leg_fairing",
    )
    frame.visual(
        Box((0.10, 0.14, 0.09)),
        origin=Origin(xyz=(0.0, -0.30, 0.065)),
        material=frame_black,
        name="rear_support_pedestal",
    )
    frame.visual(
        Box((0.08, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.19, 0.60)),
        material=dark_trim,
        name="head_cluster",
    )
    frame.visual(
        Box((0.12, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.18, 0.40)),
        material=frame_black,
        name="seat_cluster",
    )
    frame.visual(
        Box((0.08, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -0.055, 0.66)),
        material=frame_black,
        name="seat_top_junction",
    )

    sleeve_center = (0.0, -0.10, 0.60)
    sleeve_outer_x = 0.060
    sleeve_outer_y = 0.082
    sleeve_height = 0.36
    wall = 0.008
    frame.visual(
        Box((wall, sleeve_outer_y, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0] - sleeve_outer_x / 2.0 + wall / 2.0, sleeve_center[1], sleeve_center[2])),
        material=steel,
        name="seat_sleeve_left",
    )
    frame.visual(
        Box((wall, sleeve_outer_y, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0] + sleeve_outer_x / 2.0 - wall / 2.0, sleeve_center[1], sleeve_center[2])),
        material=steel,
        name="seat_sleeve_right",
    )
    frame.visual(
        Box((sleeve_outer_x - 2.0 * wall, wall, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0], sleeve_center[1] - sleeve_outer_y / 2.0 + wall / 2.0, sleeve_center[2])),
        material=steel,
        name="seat_sleeve_back",
    )
    frame.visual(
        Box((sleeve_outer_x - 2.0 * wall, wall, sleeve_height)),
        origin=Origin(xyz=(sleeve_center[0], sleeve_center[1] + sleeve_outer_y / 2.0 - wall / 2.0, sleeve_center[2])),
        material=steel,
        name="seat_sleeve_front",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.46),
        origin=Origin(xyz=(0.0, 0.28, 1.10), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="handlebar_bar",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.12),
        origin=Origin(xyz=(0.0, 0.28, 1.04)),
        material=dark_trim,
        name="handlebar_stem",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.17, 0.28, 1.10), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(-0.17, 0.28, 1.10), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame.visual(
        Box((0.058, 0.024, 0.080)),
        origin=Origin(xyz=(0.091, -0.018, 0.230)),
        material=frame_black,
        name="right_crank_bearing_block",
    )
    frame.visual(
        Box((0.058, 0.024, 0.080)),
        origin=Origin(xyz=(-0.091, -0.018, 0.230)),
        material=frame_black,
        name="left_crank_bearing_block",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Box((0.18, 0.36, 0.36)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.110, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.014, length=0.138),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="flywheel_axle",
    )
    flywheel.visual(
        Box((0.018, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.062, 0.058)),
        material=steel,
        name="flywheel_counterweight",
    )

    crank_set = model.part("crank_set")
    crank_set.inertial = Inertial.from_geometry(
        Box((0.42, 0.18, 0.42)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crank_set.visual(
        Cylinder(radius=0.015, length=0.140),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="right_crank_hub",
    )
    crank_set.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(-0.086, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_trim,
        name="left_crank_hub",
    )
    crank_set.visual(
        Box((0.024, 0.040, 0.17)),
        origin=Origin(xyz=(0.102, 0.0, -0.085)),
        material=steel,
        name="right_crank_arm",
    )
    crank_set.visual(
        Box((0.024, 0.040, 0.17)),
        origin=Origin(xyz=(-0.102, 0.0, 0.085)),
        material=steel,
        name="left_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(0.141, 0.0, -0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(-0.141, 0.0, 0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Box((0.095, 0.045, 0.020)),
        origin=Origin(xyz=(0.185, 0.0, -0.170)),
        material=rubber,
        name="right_pedal",
    )
    crank_set.visual(
        Box((0.095, 0.045, 0.020)),
        origin=Origin(xyz=(-0.185, 0.0, 0.170)),
        material=rubber,
        name="left_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 0.66)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.01, 0.34)),
    )
    seat_post.visual(
        Box((0.044, 0.048, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=steel,
        name="post",
    )
    seat_post.visual(
        Cylinder(radius=0.015, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.515), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="seat_rail",
    )
    seat_post.visual(
        Box((0.23, 0.19, 0.05)),
        origin=Origin(xyz=(0.0, -0.02, 0.555)),
        material=seat_vinyl,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.10, 0.17, 0.04)),
        origin=Origin(xyz=(0.0, 0.12, 0.55)),
        material=seat_vinyl,
        name="saddle_nose",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.0, 0.08, 0.35)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=25.0,
        ),
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_set,
        origin=Origin(xyz=(0.0, 0.02, 0.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=18.0,
        ),
    )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.0, -0.10, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank_set = object_model.get_part("crank_set")
    flywheel = object_model.get_part("flywheel")
    seat_post = object_model.get_part("seat_post")
    crank_spin = object_model.get_articulation("crank_spin")
    flywheel_spin = object_model.get_articulation("flywheel_spin")
    seat_height = object_model.get_articulation("seat_height")

    ctx.expect_gap(
        seat_post,
        frame,
        axis="z",
        positive_elem="saddle_rear",
        negative_elem="rear_stabilizer",
        min_gap=0.72,
        name="seat sits well above the floor frame",
    )

    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.14}):
        raised_pos = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post raises upward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    pedal_aabb = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
    ctx.check(
        "lowest pedal clears the floor frame",
        frame_aabb is not None
        and pedal_aabb is not None
        and pedal_aabb[0][2] >= frame_aabb[0][2] + 0.04,
        details=f"frame_aabb={frame_aabb}, pedal_aabb={pedal_aabb}",
    )

    with ctx.pose({crank_spin: pi / 2.0}):
        quarter_turn_pedal_aabb = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
    ctx.check(
        "crank rotation lifts the right pedal",
        pedal_aabb is not None
        and quarter_turn_pedal_aabb is not None
        and (quarter_turn_pedal_aabb[0][2] + quarter_turn_pedal_aabb[1][2]) / 2.0
        > (pedal_aabb[0][2] + pedal_aabb[1][2]) / 2.0 + 0.12,
        details=f"rest={pedal_aabb}, quarter_turn={quarter_turn_pedal_aabb}",
    )

    counterweight_aabb = ctx.part_element_world_aabb(flywheel, elem="flywheel_counterweight")
    with ctx.pose({flywheel_spin: pi / 2.0}):
        rotated_counterweight_aabb = ctx.part_element_world_aabb(flywheel, elem="flywheel_counterweight")
    ctx.check(
        "flywheel counterweight advances around the axle",
        counterweight_aabb is not None
        and rotated_counterweight_aabb is not None
        and (rotated_counterweight_aabb[0][1] + rotated_counterweight_aabb[1][1]) / 2.0
        < (counterweight_aabb[0][1] + counterweight_aabb[1][1]) / 2.0 - 0.12,
        details=f"rest={counterweight_aabb}, quarter_turn={rotated_counterweight_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
