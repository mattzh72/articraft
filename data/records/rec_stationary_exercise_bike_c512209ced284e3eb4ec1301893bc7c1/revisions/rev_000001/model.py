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

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.24, 0.26, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    flywheel_red = model.material("flywheel_red", rgba=(0.68, 0.12, 0.10, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 1.12, 1.30)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.02, 0.65)),
    )

    frame.visual(
        Cylinder(radius=0.030, length=0.64),
        origin=Origin(xyz=(0.0, -0.28, 0.05), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_stabilizer",
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.30),
        origin=Origin(xyz=(0.0, 0.34, 0.06), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_stabilizer",
    )

    lower_rail = tube_from_spline_points(
        [
            (0.0, -0.28, 0.05),
            (0.0, -0.16, 0.08),
            (0.0, -0.05, 0.16),
            (0.0, -0.02, 0.27),
        ],
        radius=0.035,
        samples_per_segment=16,
        radial_segments=20,
    )
    frame.visual(_mesh("bike_lower_rail", lower_rail), material=frame_paint, name="lower_rail")

    drive_stay = tube_from_spline_points(
        [
            (0.090, -0.005, 0.205),
            (0.090, 0.085, 0.225),
            (0.086, 0.20, 0.22),
        ],
        radius=0.034,
        samples_per_segment=14,
        radial_segments=20,
    )
    frame.visual(_mesh("bike_drive_stay", drive_stay), material=frame_paint, name="drive_stay")

    left_seat_brace = tube_from_spline_points(
        [
            (-0.070, -0.145, 0.245),
            (-0.064, -0.162, 0.360),
            (-0.058, -0.178, 0.520),
            (-0.052, -0.188, 0.680),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.visual(
        _mesh("bike_left_seat_brace", left_seat_brace),
        material=frame_paint,
        name="seat_column",
    )
    right_seat_brace = tube_from_spline_points(
        [
            (0.070, -0.145, 0.245),
            (0.064, -0.162, 0.360),
            (0.058, -0.178, 0.520),
            (0.052, -0.188, 0.680),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.visual(
        _mesh("bike_right_seat_brace", right_seat_brace),
        material=frame_paint,
        name="seat_column_right",
    )
    frame.visual(
        Box((0.016, 0.034, 0.16)),
        origin=Origin(xyz=(-0.029, -0.181, 0.738), rpy=(0.34, 0.0, 0.0)),
        material=dark_metal,
        name="seat_clamp_left",
    )
    frame.visual(
        Box((0.016, 0.034, 0.16)),
        origin=Origin(xyz=(0.029, -0.181, 0.738), rpy=(0.34, 0.0, 0.0)),
        material=dark_metal,
        name="seat_clamp_right",
    )

    front_mast = tube_from_spline_points(
        [
            (0.0, 0.34, 0.06),
            (0.12, 0.37, 0.22),
            (0.17, 0.35, 0.56),
            (0.11, 0.26, 0.92),
            (0.0, 0.17, 1.16),
        ],
        radius=0.031,
        samples_per_segment=18,
        radial_segments=18,
    )
    frame.visual(_mesh("bike_front_mast", front_mast), material=frame_paint, name="front_mast")

    frame.visual(
        Cylinder(radius=0.255, length=0.016),
        origin=Origin(xyz=(-0.094, 0.34, 0.35), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_black,
        name="left_housing_shroud",
    )
    frame.visual(
        Cylinder(radius=0.255, length=0.016),
        origin=Origin(xyz=(0.094, 0.34, 0.35), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_black,
        name="right_housing_shroud",
    )
    frame.visual(
        Box((0.19, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.61, 0.35)),
        material=trim_black,
        name="housing_front_bridge",
    )
    frame.visual(
        Box((0.19, 0.07, 0.06)),
        origin=Origin(xyz=(0.0, 0.055, 0.22)),
        material=trim_black,
        name="housing_rear_bridge",
    )
    frame.visual(
        Box((0.19, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, 0.34, 0.61)),
        material=trim_black,
        name="housing_top_bridge",
    )
    frame.visual(
        Box((0.19, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, 0.34, 0.075)),
        material=trim_black,
        name="housing_bottom_bridge",
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(-0.066, 0.34, 0.35), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_flywheel_bearing",
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(0.066, 0.34, 0.35), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_flywheel_bearing",
    )
    frame.visual(
        Box((0.08, 0.22, 0.34)),
        origin=Origin(xyz=(0.085, 0.20, 0.19)),
        material=trim_black,
        name="drive_case",
    )
    frame.visual(
        Box((0.18, 0.070, 0.034)),
        origin=Origin(xyz=(0.0, -0.10, 0.220)),
        material=dark_metal,
        name="bottom_bracket_shell",
    )
    frame.visual(
        Box((0.050, 0.20, 0.05)),
        origin=Origin(xyz=(0.075, 0.015, 0.220)),
        material=trim_black,
        name="drive_mount_bridge",
    )
    frame.visual(
        Box((0.016, 0.052, 0.080)),
        origin=Origin(xyz=(-0.082, -0.158, 0.245)),
        material=dark_metal,
        name="bb_bearing_left",
    )
    frame.visual(
        Box((0.016, 0.052, 0.080)),
        origin=Origin(xyz=(0.082, -0.158, 0.245)),
        material=dark_metal,
        name="bb_bearing_right",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.08),
        origin=Origin(xyz=(0.0, 0.17, 1.16)),
        material=dark_metal,
        name="handlebar_clamp",
    )
    handlebar = tube_from_spline_points(
        [
            (-0.23, 0.10, 1.08),
            (-0.16, 0.12, 1.13),
            (-0.07, 0.15, 1.17),
            (0.0, 0.17, 1.18),
            (0.07, 0.15, 1.17),
            (0.16, 0.12, 1.13),
            (0.23, 0.10, 1.08),
        ],
        radius=0.016,
        samples_per_segment=14,
        radial_segments=18,
    )
    frame.visual(_mesh("bike_handlebar", handlebar), material=dark_metal, name="handlebar")
    frame.visual(
        Cylinder(radius=0.020, length=0.11),
        origin=Origin(xyz=(-0.255, 0.095, 1.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.11),
        origin=Origin(xyz=(0.255, 0.095, 1.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    crank_set = model.part("crank_set")
    crank_set.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.32),
        mass=4.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    crank_set.visual(
        Cylinder(radius=0.032, length=0.240),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle",
    )
    crank_set.visual(
        Box((0.024, 0.17, 0.042)),
        origin=Origin(xyz=(0.120, -0.085, 0.0)),
        material=satin_silver,
        name="left_crank_arm",
    )
    crank_set.visual(
        Box((0.024, 0.17, 0.042)),
        origin=Origin(xyz=(-0.120, 0.085, 0.0)),
        material=satin_silver,
        name="right_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.12),
        origin=Origin(xyz=(0.120, -0.168, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.12),
        origin=Origin(xyz=(-0.120, 0.168, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_spindle",
    )
    crank_set.visual(
        Box((0.10, 0.040, 0.020)),
        origin=Origin(xyz=(0.180, -0.168, 0.0)),
        material=trim_black,
        name="left_pedal",
    )
    crank_set.visual(
        Box((0.10, 0.040, 0.020)),
        origin=Origin(xyz=(-0.180, 0.168, 0.0)),
        material=trim_black,
        name="right_pedal",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.05),
        mass=10.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.20, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=flywheel_red,
        name="wheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.23, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="wheel_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.055, length=0.076),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_silver,
        name="wheel_hub",
    )
    flywheel.visual(
        Box((0.016, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=satin_silver,
        name="timing_marker",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.30, 0.32, 0.58)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.01, 0.28)),
    )
    seat_post.visual(
        Box((0.042, 0.056, 0.38)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=satin_silver,
        name="seat_post_shaft",
    )
    seat_post.visual(
        Box((0.06, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=dark_metal,
        name="seat_mount",
    )
    seat_post.visual(
        Box((0.17, 0.21, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.32), rpy=(-0.28, 0.0, 0.0)),
        material=saddle_black,
        name="saddle_main",
    )
    seat_post.visual(
        Box((0.10, 0.11, 0.04)),
        origin=Origin(xyz=(0.0, 0.12, 0.305), rpy=(-0.34, 0.0, 0.0)),
        material=saddle_black,
        name="saddle_nose",
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_set,
        origin=Origin(xyz=(0.0, -0.10, 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=18.0),
    )
    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.0, 0.34, 0.35)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=24.0),
    )
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.0, -0.16, 0.74), rpy=(0.34, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.18),
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

    def span(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][index] - aabb[0][index]

    rear_support = ctx.part_element_world_aabb(frame, elem="rear_stabilizer")
    front_support = ctx.part_element_world_aabb(frame, elem="front_stabilizer")
    rear_width = span(rear_support, "x")
    front_width = span(front_support, "x")
    ctx.check(
        "rear support is broader than front support",
        rear_width is not None and front_width is not None and rear_width > front_width + 0.20,
        details=f"rear_width={rear_width}, front_width={front_width}",
    )

    ctx.expect_contact(
        flywheel,
        frame,
        elem_a="wheel_hub",
        elem_b="left_flywheel_bearing",
        name="flywheel hub seats against left bearing",
    )
    ctx.expect_contact(
        flywheel,
        frame,
        elem_a="wheel_hub",
        elem_b="right_flywheel_bearing",
        name="flywheel hub seats against right bearing",
    )
    ctx.expect_contact(
        crank_set,
        frame,
        elem_a="axle",
        elem_b="bb_bearing_left",
        name="crank axle is carried by left bottom bracket support",
    )
    ctx.expect_contact(
        crank_set,
        frame,
        elem_a="axle",
        elem_b="bb_bearing_right",
        name="crank axle is carried by right bottom bracket support",
    )

    left_pedal_rest = ctx.part_element_world_aabb(crank_set, elem="left_pedal")
    timing_marker_rest = ctx.part_element_world_aabb(flywheel, elem="timing_marker")
    seat_rest = ctx.part_world_position(seat_post)
    with ctx.pose({crank_spin: pi / 2.0, flywheel_spin: pi / 2.0, seat_height: 0.18}):
        left_pedal_turned = ctx.part_element_world_aabb(crank_set, elem="left_pedal")
        timing_marker_turned = ctx.part_element_world_aabb(flywheel, elem="timing_marker")
        seat_raised = ctx.part_world_position(seat_post)

        ctx.check(
            "crank rotation moves the pedal through its cycle",
            left_pedal_rest is not None
            and left_pedal_turned is not None
            and abs(left_pedal_turned[0][2] - left_pedal_rest[0][2]) > 0.08,
            details=f"rest={left_pedal_rest}, turned={left_pedal_turned}",
        )
        ctx.check(
            "flywheel rotation moves the timing marker",
            timing_marker_rest is not None
            and timing_marker_turned is not None
            and abs(timing_marker_turned[0][1] - timing_marker_rest[0][1]) > 0.12,
            details=f"rest={timing_marker_rest}, turned={timing_marker_turned}",
        )
        ctx.check(
            "seat post raises along its column",
            seat_rest is not None and seat_raised is not None and seat_raised[2] > seat_rest[2] + 0.12,
            details=f"rest={seat_rest}, raised={seat_raised}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
