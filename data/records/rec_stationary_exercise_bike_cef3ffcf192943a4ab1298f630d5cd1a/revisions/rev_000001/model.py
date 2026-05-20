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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.12, 0.13, 0.14, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.14, 0.12, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.16, 0.60, 1.18)),
        mass=34.0,
        origin=Origin(xyz=(0.06, 0.0, 0.59)),
    )

    frame.visual(
        Box((0.12, 0.56, 0.05)),
        origin=Origin(xyz=(-0.28, 0.0, 0.025)),
        material=dark_gray,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.12, 0.52, 0.05)),
        origin=Origin(xyz=(0.38, 0.0, 0.025)),
        material=dark_gray,
        name="front_stabilizer",
    )

    left_rail = tube_from_spline_points(
        [
            (-0.28, 0.16, 0.05),
            (-0.16, 0.16, 0.17),
            (-0.16, 0.16, 0.28),
            (0.18, 0.16, 0.13),
            (0.38, 0.16, 0.05),
        ],
        radius=0.028,
        samples_per_segment=14,
        radial_segments=18,
    )
    right_rail = tube_from_spline_points(
        [
            (-0.28, -0.16, 0.05),
            (-0.16, -0.16, 0.17),
            (-0.16, -0.16, 0.28),
            (0.18, -0.16, 0.13),
            (0.38, -0.16, 0.05),
        ],
        radius=0.028,
        samples_per_segment=14,
        radial_segments=18,
    )
    frame.visual(_mesh("bike_left_base_rail", left_rail), material=frame_black, name="left_base_rail")
    frame.visual(_mesh("bike_right_base_rail", right_rail), material=frame_black, name="right_base_rail")

    left_rear_stay = tube_from_spline_points(
        [
            (-0.28, 0.18, 0.05),
            (-0.22, 0.18, 0.19),
            (-0.08, 0.14, 0.38),
            (0.06, 0.10, 0.56),
        ],
        radius=0.024,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_rear_stay = tube_from_spline_points(
        [
            (-0.28, -0.18, 0.05),
            (-0.22, -0.18, 0.19),
            (-0.08, -0.14, 0.38),
            (0.06, -0.10, 0.56),
        ],
        radius=0.024,
        samples_per_segment=16,
        radial_segments=18,
    )
    handlebar_mast = tube_from_spline_points(
        [
            (0.24, 0.0, 0.66),
            (0.31, 0.0, 0.85),
            (0.37, 0.0, 1.03),
        ],
        radius=0.040,
        samples_per_segment=18,
        radial_segments=18,
    )
    frame.visual(_mesh("bike_left_rear_stay", left_rear_stay), material=frame_black, name="left_rear_stay")
    frame.visual(_mesh("bike_right_rear_stay", right_rear_stay), material=frame_black, name="right_rear_stay")
    frame.visual(_mesh("bike_handlebar_mast", handlebar_mast), material=frame_black, name="handlebar_mast")

    frame.visual(
        Box((0.04, 0.14, 0.10)),
        origin=Origin(xyz=(0.00, 0.0, 0.19)),
        material=frame_black,
        name="bottom_bracket_spine",
    )
    frame.visual(
        Box((0.12, 0.25, 0.11)),
        origin=Origin(xyz=(0.06, 0.0, 0.215)),
        material=frame_black,
        name="bottom_bracket_junction",
    )
    frame.visual(
        Box((0.055, 0.040, 0.10)),
        origin=Origin(xyz=(0.12, 0.14, 0.31)),
        material=frame_black,
        name="left_bottom_bracket_bearing",
    )
    frame.visual(
        Box((0.055, 0.040, 0.10)),
        origin=Origin(xyz=(0.12, -0.14, 0.31)),
        material=frame_black,
        name="right_bottom_bracket_bearing",
    )
    frame.visual(
        Box((0.28, 0.24, 0.14)),
        origin=Origin(xyz=(-0.16, 0.0, 0.18)),
        material=frame_black,
        name="lower_housing",
    )
    frame.visual(
        Box((0.024, 0.16, 0.20)),
        origin=Origin(xyz=(0.002, 0.0, 0.56)),
        material=frame_black,
        name="seat_cluster",
    )
    frame.visual(
        Box((0.08, 0.10, 0.40)),
        origin=Origin(xyz=(0.04, 0.0, 0.46)),
        material=frame_black,
        name="outer_sleeve",
    )
    frame.visual(
        Box((0.21, 0.08, 0.06)),
        origin=Origin(xyz=(0.175, 0.0, 0.69)),
        material=frame_black,
        name="top_bridge",
    )
    frame.visual(
        Box((0.09, 0.10, 0.06)),
        origin=Origin(xyz=(-0.28, 0.17, 0.03)),
        material=dark_gray,
        name="left_rear_foot_node",
    )
    frame.visual(
        Box((0.09, 0.10, 0.06)),
        origin=Origin(xyz=(-0.28, -0.17, 0.03)),
        material=dark_gray,
        name="right_rear_foot_node",
    )
    frame.visual(
        Box((0.09, 0.09, 0.06)),
        origin=Origin(xyz=(0.38, 0.16, 0.03)),
        material=dark_gray,
        name="left_front_foot_node",
    )
    frame.visual(
        Box((0.09, 0.09, 0.06)),
        origin=Origin(xyz=(0.38, -0.16, 0.03)),
        material=dark_gray,
        name="right_front_foot_node",
    )
    frame.visual(
        Box((0.09, 0.05, 0.12)),
        origin=Origin(xyz=(-0.16, 0.135, 0.18)),
        material=frame_black,
        name="left_lower_rail_gusset",
    )
    frame.visual(
        Box((0.09, 0.05, 0.12)),
        origin=Origin(xyz=(-0.16, -0.135, 0.18)),
        material=frame_black,
        name="right_lower_rail_gusset",
    )
    frame.visual(
        Box((0.10, 0.05, 0.12)),
        origin=Origin(xyz=(0.05, 0.075, 0.56)),
        material=frame_black,
        name="left_seat_gusset",
    )
    frame.visual(
        Box((0.10, 0.05, 0.12)),
        origin=Origin(xyz=(0.05, -0.075, 0.56)),
        material=frame_black,
        name="right_seat_gusset",
    )
    frame.visual(
        Box((0.10, 0.08, 0.12)),
        origin=Origin(xyz=(0.37, 0.0, 1.04)),
        material=dark_gray,
        name="console_block",
    )
    frame.visual(
        Box((0.06, 0.08, 0.12)),
        origin=Origin(xyz=(0.37, 0.0, 0.98)),
        material=dark_gray,
        name="bar_stem",
    )
    frame.visual(
        Box((0.08, 0.12, 0.05)),
        origin=Origin(xyz=(0.37, 0.0, 1.05)),
        material=dark_gray,
        name="bar_clamp",
    )

    frame.visual(
        Box((0.035, 0.030, 0.46)),
        origin=Origin(xyz=(-0.22, 0.105, 0.34)),
        material=housing_gray,
        name="left_flywheel_fork",
    )
    frame.visual(
        Box((0.035, 0.030, 0.46)),
        origin=Origin(xyz=(-0.22, -0.105, 0.34)),
        material=housing_gray,
        name="right_flywheel_fork",
    )

    handlebars = tube_from_spline_points(
        [
            (0.35, -0.24, 1.00),
            (0.37, -0.14, 1.05),
            (0.38, 0.0, 1.07),
            (0.37, 0.14, 1.05),
            (0.35, 0.24, 1.00),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
    )
    frame.visual(_mesh("bike_handlebars", handlebars), material=rubber, name="handlebars")

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.16),
        mass=12.0,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.21, length=0.16),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="flywheel_hub",
    )
    flywheel.visual(
        Cylinder(radius=0.11, length=0.020),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="flywheel_left_cap",
    )
    flywheel.visual(
        Cylinder(radius=0.11, length=0.020),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="flywheel_right_cap",
    )

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Box((0.24, 0.56, 0.40)),
        mass=6.0,
        origin=Origin(),
    )
    crankset.visual(
        Cylinder(radius=0.022, length=0.42),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    crankset.visual(
        Box((0.12, 0.040, 0.020)),
        origin=Origin(xyz=(0.05, -0.200, 0.0)),
        material=dark_gray,
        name="chainring_spider",
    )
    crankset.visual(
        Cylinder(radius=0.060, length=0.024),
        origin=Origin(xyz=(0.10, -0.190, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="chainring",
    )
    crankset.visual(
        Box((0.030, 0.018, 0.170)),
        origin=Origin(xyz=(0.0, 0.21, -0.085)),
        material=dark_gray,
        name="left_crank_arm",
    )
    crankset.visual(
        Box((0.030, 0.018, 0.170)),
        origin=Origin(xyz=(0.0, -0.21, 0.085)),
        material=dark_gray,
        name="right_crank_arm",
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.0, 0.255, -0.17), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_spindle",
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.0, -0.255, 0.17), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_spindle",
    )
    crankset.visual(
        Box((0.11, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.295, -0.17)),
        material=dark_gray,
        name="left_pedal",
    )
    crankset.visual(
        Box((0.11, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -0.295, 0.17)),
        material=dark_gray,
        name="right_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.30, 0.22, 0.50)),
        mass=5.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.18)),
    )
    seat_post.visual(
        Box((0.050, 0.070, 0.480)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.080, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=dark_gray,
        name="seat_head",
    )
    seat_post.visual(
        Box((0.14, 0.10, 0.03)),
        origin=Origin(xyz=(-0.02, 0.0, 0.30)),
        material=dark_gray,
        name="seat_yoke",
    )
    seat_post.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(-0.02, 0.040, 0.32), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_seat_rail",
    )
    seat_post.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(-0.02, -0.040, 0.32), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_seat_rail",
    )
    seat_post.visual(
        Box((0.28, 0.18, 0.050)),
        origin=Origin(xyz=(-0.03, 0.0, 0.33)),
        material=saddle_black,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.18, 0.10, 0.040)),
        origin=Origin(xyz=(0.10, 0.0, 0.32)),
        material=saddle_black,
        name="saddle_nose",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(-0.22, 0.0, 0.48)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=25.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(0.12, 0.0, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=18.0),
    )
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.04, 0.0, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.15, lower=0.0, upper=0.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flywheel = object_model.get_part("flywheel")
    crankset = object_model.get_part("crankset")
    seat_post = object_model.get_part("seat_post")
    seat_height = object_model.get_articulation("seat_height")
    crank_spin = object_model.get_articulation("crank_spin")

    seat_upper = seat_height.motion_limits.upper if seat_height.motion_limits is not None else None

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="outer_sleeve",
        elem_b="inner_post",
        reason="The outer sleeve is a simplified solid proxy for the seat tube around the telescoping inner post.",
    )
    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="left_bottom_bracket_bearing",
        elem_b="axle",
        reason="The left bottom-bracket bearing block is a simplified solid proxy around the crank axle.",
    )
    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="right_bottom_bracket_bearing",
        elem_b="axle",
        reason="The right bottom-bracket bearing block is a simplified solid proxy around the crank axle.",
    )

    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="outer_sleeve",
        margin=0.003,
        name="seat post stays centered in outer sleeve at rest",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="outer_sleeve",
        min_overlap=0.17,
        name="seat post retains deep insertion at rest",
    )

    rest_seat_pos = ctx.part_world_position(seat_post)
    if seat_upper is not None:
        with ctx.pose({seat_height: seat_upper}):
            ctx.expect_within(
                seat_post,
                frame,
                axes="xy",
                inner_elem="inner_post",
                outer_elem="outer_sleeve",
                margin=0.003,
                name="seat post stays centered when raised",
            )
            ctx.expect_overlap(
                seat_post,
                frame,
                axes="z",
                elem_a="inner_post",
                elem_b="outer_sleeve",
                min_overlap=0.04,
                name="seat post keeps retained insertion when raised",
            )
            raised_seat_pos = ctx.part_world_position(seat_post)
        ctx.check(
            "seat height adjustment moves upward",
            rest_seat_pos is not None
            and raised_seat_pos is not None
            and raised_seat_pos[2] > rest_seat_pos[2] + 0.10,
            details=f"rest={rest_seat_pos}, raised={raised_seat_pos}",
        )

    def _center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def _center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    sleeve_aabb = ctx.part_element_world_aabb(frame, elem="outer_sleeve")
    mast_aabb = ctx.part_element_world_aabb(frame, elem="handlebar_mast")
    flywheel_pos = ctx.part_world_position(flywheel)
    sleeve_x = _center_x(sleeve_aabb)
    mast_x = _center_x(mast_aabb)
    flywheel_x = flywheel_pos[0] if flywheel_pos is not None else None
    ctx.check(
        "upper support structure sits forward of flywheel center",
        sleeve_x is not None
        and mast_x is not None
        and flywheel_x is not None
        and sleeve_x > flywheel_x + 0.05
        and mast_x > flywheel_x + 0.18,
        details=f"sleeve_x={sleeve_x}, mast_x={mast_x}, flywheel_x={flywheel_x}",
    )

    left_pedal_rest = ctx.part_element_world_aabb(crankset, elem="left_pedal")
    with ctx.pose({crank_spin: pi / 2.0}):
        left_pedal_quarter = ctx.part_element_world_aabb(crankset, elem="left_pedal")
    pedal_rest_z = _center_z(left_pedal_rest)
    pedal_quarter_z = _center_z(left_pedal_quarter)
    ctx.check(
        "crank quarter turn lifts left pedal",
        pedal_rest_z is not None
        and pedal_quarter_z is not None
        and pedal_quarter_z > pedal_rest_z + 0.08,
        details=f"rest_z={pedal_rest_z}, quarter_z={pedal_quarter_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
