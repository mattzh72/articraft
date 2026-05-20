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
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (v[0] / length, v[1] / length, v[2] / length)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _hollow_cylinder_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    half = length * 0.5
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.13, 0.14, 0.15, 1.0))
    housing_black = model.material("housing_black", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.16, 0.56, 1.28)),
        mass=38.0,
        origin=Origin(xyz=(0.02, 0.0, 0.58)),
    )

    frame.visual(
        Box((0.16, 0.54, 0.08)),
        origin=Origin(xyz=(-0.34, 0.0, 0.04)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.16, 0.58, 0.08)),
        origin=Origin(xyz=(0.38, 0.0, 0.04)),
        material=frame_black,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.78, 0.05, 0.06)),
        origin=Origin(xyz=(0.02, 0.12, 0.07)),
        material=frame_black,
        name="right_base_rail",
    )
    frame.visual(
        Box((0.78, 0.05, 0.06)),
        origin=Origin(xyz=(0.02, -0.12, 0.07)),
        material=frame_black,
        name="left_base_rail",
    )

    bottom_bracket = (0.02, 0.0, 0.34)
    bottom_lug_center = (0.02, 0.0, 0.285)
    head_node = (0.22, 0.0, 0.58)
    handlebar_top = (0.32, 0.0, 1.01)
    seat_entry = (-0.18, 0.0, 0.78)
    seat_axis = _unit(
        (
            seat_entry[0] - bottom_bracket[0],
            seat_entry[1] - bottom_bracket[1],
            seat_entry[2] - bottom_bracket[2],
        )
    )

    frame.visual(
        _hollow_cylinder_mesh(
            "bottom_bracket_shell_mesh",
            outer_radius=0.050,
            inner_radius=0.022,
            length=0.16,
        ),
        origin=Origin(xyz=bottom_bracket, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="bottom_bracket_shell",
    )
    frame.visual(
        Box((0.14, 0.13, 0.05)),
        origin=Origin(xyz=bottom_lug_center),
        material=frame_black,
        name="bottom_bracket_lug",
    )
    _add_member(
        frame,
        (0.08, 0.0, 0.35),
        head_node,
        radius=0.030,
        material=frame_black,
        name="main_downtube",
    )
    frame.visual(
        Cylinder(radius=0.048, length=0.11),
        origin=Origin(xyz=head_node, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="head_tube",
    )
    _add_member(frame, head_node, handlebar_top, radius=0.032, material=frame_black, name="handlebar_mast")
    seat_tube_base = (-0.05, 0.0, 0.36)
    seat_tube_top = tuple(seat_entry[i] - seat_axis[i] * 0.055 for i in range(3))
    frame.visual(
        _hollow_cylinder_mesh(
            "seat_tube_mesh",
            outer_radius=0.032,
            inner_radius=0.0265,
            length=_distance(seat_tube_base, seat_tube_top),
        ),
        origin=Origin(
            xyz=_midpoint(seat_tube_base, seat_tube_top),
            rpy=_rpy_for_segment(seat_tube_base, seat_tube_top),
        ),
        material=frame_black,
        name="seat_tube",
    )
    _add_member(
        frame,
        (0.05, 0.0, 0.27),
        (0.30, 0.0, 0.07),
        radius=0.028,
        material=frame_black,
        name="lower_spine",
    )

    frame.visual(
        _hollow_cylinder_mesh(
            "seat_sleeve_mesh",
            outer_radius=0.036,
            inner_radius=0.0265,
            length=0.12,
        ),
        origin=Origin(xyz=seat_entry, rpy=_rpy_for_segment(bottom_bracket, seat_entry)),
        material=trim_silver,
        name="seat_sleeve",
    )

    housing_profile_outer = [
        (0.065, -0.080),
        (0.175, -0.080),
        (0.230, -0.040),
        (0.250, 0.0),
        (0.230, 0.040),
        (0.175, 0.080),
        (0.065, 0.080),
    ]
    housing_profile_inner = [
        (0.058, -0.068),
        (0.156, -0.068),
        (0.206, -0.034),
        (0.224, 0.0),
        (0.206, 0.034),
        (0.156, 0.068),
        (0.058, 0.068),
    ]
    frame.visual(
        _save_mesh(
            "bike_housing_shell",
            LatheGeometry.from_shell_profiles(
                housing_profile_outer,
                housing_profile_inner,
                segments=72,
                start_cap="round",
                end_cap="round",
                lip_samples=8,
            ),
        ),
        origin=Origin(xyz=(0.40, 0.0, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="flywheel_housing",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.40, 0.086, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="right_flywheel_bearing",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.40, -0.086, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="left_flywheel_bearing",
    )
    _add_member(
        frame,
        (0.34, 0.110, 0.090),
        (0.392, 0.086, 0.286),
        radius=0.014,
        material=frame_black,
        name="right_flywheel_stay",
    )
    _add_member(
        frame,
        (0.34, -0.110, 0.090),
        (0.392, -0.086, 0.286),
        radius=0.014,
        material=frame_black,
        name="left_flywheel_stay",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.11),
        origin=Origin(xyz=(0.32, 0.0, 1.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="handlebar_clamp",
    )

    frame.visual(
        Cylinder(radius=0.017, length=0.42),
        origin=Origin(xyz=(0.32, 0.0, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="handlebar",
    )
    _add_member(frame, (0.32, 0.13, 1.05), (0.22, 0.21, 1.16), radius=0.014, material=frame_black)
    _add_member(frame, (0.32, -0.13, 1.05), (0.22, -0.21, 1.16), radius=0.014, material=frame_black)
    frame.visual(
        Cylinder(radius=0.021, length=0.10),
        origin=Origin(xyz=(0.19, 0.23, 1.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.10),
        origin=Origin(xyz=(0.19, -0.23, 1.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.038),
        mass=14.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.190, length=0.038),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="flywheel_hub",
    )
    flywheel.visual(
        Cylinder(radius=0.015, length=0.148),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="flywheel_axle",
    )
    flywheel.visual(
        Box((0.040, 0.014, 0.028)),
        origin=Origin(xyz=(0.145, 0.0, -0.070)),
        material=trim_silver,
        name="flywheel_counterweight",
    )
    model.articulation(
        "frame_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.40, 0.0, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=30.0),
    )

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.22),
        mass=4.8,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    crankset.visual(
        Cylinder(radius=0.017, length=0.18),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="crank_axle",
    )
    crankset.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="right_crank_boss",
    )
    crankset.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="left_crank_boss",
    )
    crankset.visual(
        Cylinder(radius=0.105, length=0.012),
        origin=Origin(xyz=(0.0, 0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="drive_ring",
    )
    crankset.visual(
        Box((0.028, 0.020, 0.180)),
        origin=Origin(xyz=(0.0, 0.100, -0.110)),
        material=trim_silver,
        name="right_crank_arm",
    )
    crankset.visual(
        Box((0.028, 0.020, 0.180)),
        origin=Origin(xyz=(0.0, -0.100, 0.110)),
        material=trim_silver,
        name="left_crank_arm",
    )
    crankset.visual(
        Box((0.105, 0.030, 0.018)),
        origin=Origin(xyz=(0.050, 0.100, -0.205)),
        material=rubber,
        name="right_pedal",
    )
    crankset.visual(
        Box((0.105, 0.030, 0.018)),
        origin=Origin(xyz=(0.050, -0.100, 0.205)),
        material=rubber,
        name="left_pedal",
    )
    model.articulation(
        "frame_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=bottom_bracket),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=20.0),
    )

    seat_post = model.part("seat_post")
    post_lower = tuple(-axis * 0.045 for axis in seat_axis)
    post_upper = tuple(axis * 0.34 for axis in seat_axis)
    seat_post.inertial = Inertial.from_geometry(
        Box((0.28, 0.20, 0.44)),
        mass=4.0,
        origin=Origin(xyz=(-0.08, 0.0, 0.21)),
    )
    _add_member(
        seat_post,
        post_lower,
        post_upper,
        radius=0.0265,
        material=trim_silver,
        name="seat_post_shaft",
    )
    seat_post.visual(
        Cylinder(radius=0.014, length=0.16),
        origin=Origin(
            xyz=(post_upper[0] - 0.010, 0.0, post_upper[2] + 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_silver,
        name="seat_rail_bridge",
    )
    _add_member(
        seat_post,
        post_upper,
        (post_upper[0] - 0.010, 0.0, post_upper[2] + 0.035),
        radius=0.012,
        material=trim_silver,
        name="seat_clamp_stem",
    )
    seat_post.visual(
        Box((0.18, 0.19, 0.040)),
        origin=Origin(xyz=(post_upper[0] - 0.010, 0.0, post_upper[2] + 0.052)),
        material=saddle_black,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.13, 0.11, 0.032)),
        origin=Origin(xyz=(post_upper[0] + 0.085, 0.0, post_upper[2] + 0.045)),
        material=saddle_black,
        name="saddle_nose",
    )
    model.articulation(
        "frame_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=seat_entry),
        axis=seat_axis,
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.10,
        ),
    )

    brace = model.part("base_brace")
    brace_start = (-0.10, -0.095, 0.07)
    brace_end = (-0.10, 0.095, 0.07)
    brace.inertial = Inertial.from_geometry(
        Cylinder(radius=0.009, length=_distance(brace_start, brace_end)),
        mass=1.2,
        origin=Origin(
            xyz=_midpoint(
                (0.0, 0.0, 0.0),
                (
                    brace_end[0] - brace_start[0],
                    brace_end[1] - brace_start[1],
                    brace_end[2] - brace_start[2],
                ),
            ),
            rpy=_rpy_for_segment(
                (0.0, 0.0, 0.0),
                (
                    brace_end[0] - brace_start[0],
                    brace_end[1] - brace_start[1],
                    brace_end[2] - brace_start[2],
                ),
            ),
        ),
    )
    _add_member(
        brace,
        (0.0, 0.0, 0.0),
        (
            brace_end[0] - brace_start[0],
            brace_end[1] - brace_start[1],
            brace_end[2] - brace_start[2],
        ),
        radius=0.009,
        material=trim_silver,
        name="brace_tube",
    )

    model.articulation(
        "frame_to_brace",
        ArticulationType.FIXED,
        parent=frame,
        child=brace,
        origin=Origin(xyz=brace_start),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    flywheel = object_model.get_part("flywheel")
    crankset = object_model.get_part("crankset")
    seat_joint = object_model.get_articulation("frame_to_seat_post")
    crank_joint = object_model.get_articulation("frame_to_crankset")

    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="bottom_bracket_shell",
        elem_b="crank_axle",
        reason="The crank spindle is intentionally represented as rotating inside a simplified bottom-bracket shell proxy.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="seat_post_shaft",
        reason="The adjustable seat post is intentionally represented as sliding inside the seat sleeve proxy.",
    )

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    ctx.expect_within(
        flywheel,
        frame,
        axes="yz",
        inner_elem="flywheel_disc",
        outer_elem="flywheel_housing",
        margin=0.01,
        name="flywheel stays within housing envelope",
    )

    seat_rest = ctx.part_world_position(seat_post)
    with ctx.pose({seat_joint: 0.10}):
        seat_extended = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post raises along the column",
        seat_rest is not None
        and seat_extended is not None
        and seat_extended[2] > seat_rest[2] + 0.08
        and seat_extended[0] < seat_rest[0] - 0.03,
        details=f"rest={seat_rest}, extended={seat_extended}",
    )

    pedal_rest_aabb = ctx.part_element_world_aabb(crankset, elem="right_pedal")
    with ctx.pose({crank_joint: math.pi / 2.0}):
        pedal_quarter_aabb = ctx.part_element_world_aabb(crankset, elem="right_pedal")
    pedal_rest = _aabb_center(pedal_rest_aabb) if pedal_rest_aabb is not None else None
    pedal_quarter = _aabb_center(pedal_quarter_aabb) if pedal_quarter_aabb is not None else None
    ctx.check(
        "crank rotation carries the pedal through a circular path",
        pedal_rest is not None
        and pedal_quarter is not None
        and pedal_quarter[2] > pedal_rest[2] + 0.12
        and pedal_quarter[0] < pedal_rest[0] - 0.10,
        details=f"rest={pedal_rest}, quarter_turn={pedal_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
