from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _annulus_mesh(outer_diameter: float, inner_diameter: float, thickness: float, name: str):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            superellipse_profile(outer_diameter, outer_diameter, exponent=2.0, segments=72),
            [superellipse_profile(inner_diameter, inner_diameter, exponent=2.0, segments=56)],
            height=thickness,
            center=True,
        ).rotate_x(pi / 2.0),
    )


def _build_saddle_mesh():
    saddle = superellipse_side_loft(
        [
            (-0.12, 0.0, 0.022, 0.040),
            (-0.03, 0.0, 0.054, 0.145),
            (0.06, 0.0, 0.064, 0.178),
            (0.14, 0.0, 0.050, 0.130),
        ],
        exponents=2.0,
        segments=44,
    )
    saddle.rotate_z(-pi / 2.0)
    return _save_mesh("saddle_shell", saddle)


def _build_flywheel_spoke_positions(radius: float) -> list[tuple[float, float, float, float]]:
    return [
        (cos(angle) * radius, 0.0, sin(angle) * radius, angle)
        for angle in (0.0, pi / 3.0, 2.0 * pi / 3.0, pi, 4.0 * pi / 3.0, 5.0 * pi / 3.0)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.60, 0.62, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    red_accent = model.material("red_accent", rgba=(0.72, 0.10, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.12, 0.12, 0.13, 1.0))

    left_cover_mesh = _annulus_mesh(0.46, 0.20, 0.008, "flywheel_guard_left_mesh")
    right_cover_mesh = _annulus_mesh(0.46, 0.20, 0.008, "flywheel_guard_right_mesh")
    chainring_mesh = _annulus_mesh(0.210, 0.140, 0.010, "chainring_mesh")
    saddle_mesh = _build_saddle_mesh()

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.20, 0.56, 1.25)),
        mass=46.0,
        origin=Origin(xyz=(0.02, 0.0, 0.62)),
    )

    frame.visual(
        Box((0.54, 0.08, 0.05)),
        origin=Origin(xyz=(0.38, 0.0, 0.04)),
        material=frame_black,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.62, 0.08, 0.05)),
        origin=Origin(xyz=(-0.33, 0.0, 0.04)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(Box((0.12, 0.10, 0.02)), origin=Origin(xyz=(0.38, 0.0, 0.01)), material=rubber)
    frame.visual(Box((0.14, 0.10, 0.02)), origin=Origin(xyz=(-0.33, 0.0, 0.01)), material=rubber)

    _add_tube(frame, (-0.23, 0.0, 0.08), (0.06, 0.0, 0.21), 0.037, frame_black, name="lower_beam")
    _add_tube(frame, (-0.23, 0.0, 0.08), (-0.10, 0.0, 0.50), 0.031, frame_black, name="seat_brace")
    _add_tube(frame, (-0.10, 0.0, 0.51), (0.00, 0.0, 0.40), 0.028, frame_black, name="seat_mast_strut")
    _add_tube(frame, (-0.10, 0.065, 0.82), (0.48, 0.065, 0.80), 0.024, frame_black, name="top_rail_left")
    _add_tube(frame, (-0.10, -0.065, 0.82), (0.48, -0.065, 0.80), 0.024, frame_black, name="top_rail_right")
    _add_tube(frame, (0.26, 0.0, 0.20), (0.52, 0.0, 0.055), 0.028, frame_black, name="front_leg")
    _add_tube(frame, (0.52, 0.0, 0.055), (0.58, 0.0, 0.84), 0.032, frame_black, name="handlebar_mast")
    _add_tube(frame, (0.06, 0.0, 0.21), (0.24, 0.0, 0.20), 0.035, frame_black, name="flywheel_down_tube")
    _add_tube(frame, (0.10, -0.073, 0.36), (0.28, -0.073, 0.42), 0.013, dark_steel, name="belt_guard_upper")
    _add_tube(frame, (0.08, -0.073, 0.31), (0.25, -0.073, 0.38), 0.013, dark_steel, name="belt_guard_lower")

    frame.visual(
        Box((0.06, 0.18, 0.028)),
        origin=Origin(xyz=(0.02, 0.0, 0.393)),
        material=dark_steel,
        name="bottom_bracket_upper_bridge",
    )
    frame.visual(
        Box((0.06, 0.18, 0.028)),
        origin=Origin(xyz=(0.02, 0.0, 0.287)),
        material=dark_steel,
        name="bottom_bracket_lower_bridge",
    )
    frame.visual(
        Box((0.080, 0.026, 0.038)),
        origin=Origin(xyz=(0.02, 0.058, 0.390)),
        material=dark_steel,
        name="bottom_bracket_shell_left_upper",
    )
    frame.visual(
        Box((0.080, 0.026, 0.038)),
        origin=Origin(xyz=(0.02, 0.058, 0.290)),
        material=dark_steel,
        name="bottom_bracket_shell_left_lower",
    )
    frame.visual(
        Box((0.080, 0.026, 0.038)),
        origin=Origin(xyz=(0.02, -0.058, 0.390)),
        material=dark_steel,
        name="bottom_bracket_shell_right_upper",
    )
    frame.visual(
        Box((0.080, 0.026, 0.038)),
        origin=Origin(xyz=(0.02, -0.058, 0.290)),
        material=dark_steel,
        name="bottom_bracket_shell_right_lower",
    )
    frame.visual(
        Box((0.055, 0.050, 0.070)),
        origin=Origin(xyz=(-0.008, 0.0, 0.402)),
        material=dark_steel,
        name="bottom_bracket_seat_gusset",
    )
    frame.visual(
        Box((0.055, 0.050, 0.070)),
        origin=Origin(xyz=(0.048, 0.0, 0.278)),
        material=dark_steel,
        name="bottom_bracket_front_gusset",
    )
    _add_tube(
        frame,
        (-0.018, 0.0, 0.390),
        (-0.10, 0.0, 0.51),
        0.020,
        frame_black,
        name="bottom_bracket_seat_strut",
    )
    _add_tube(
        frame,
        (0.055, 0.0, 0.295),
        (0.06, 0.0, 0.21),
        0.020,
        frame_black,
        name="bottom_bracket_front_strut",
    )
    frame.visual(
        Box((0.012, 0.140, 0.30)),
        origin=Origin(xyz=(-0.141, 0.0, 0.69)),
        material=dark_steel,
        name="seat_guide_left",
    )
    frame.visual(
        Box((0.012, 0.140, 0.30)),
        origin=Origin(xyz=(-0.059, 0.0, 0.69)),
        material=dark_steel,
        name="seat_guide_right",
    )
    frame.visual(
        Box((0.094, 0.012, 0.30)),
        origin=Origin(xyz=(-0.10, 0.041, 0.69)),
        material=dark_steel,
        name="seat_guide_front",
    )
    frame.visual(
        Box((0.094, 0.012, 0.30)),
        origin=Origin(xyz=(-0.10, -0.041, 0.69)),
        material=dark_steel,
        name="seat_guide_rear",
    )
    frame.visual(
        Box((0.12, 0.12, 0.06)),
        origin=Origin(xyz=(-0.10, 0.0, 0.51)),
        material=dark_steel,
        name="seat_slider_block",
    )
    frame.visual(
        Box((0.080, 0.014, 0.050)),
        origin=Origin(xyz=(-0.10, 0.047, 0.845)),
        material=satin_steel,
        name="seat_clamp_front",
    )
    frame.visual(
        Box((0.080, 0.014, 0.050)),
        origin=Origin(xyz=(-0.10, -0.047, 0.845)),
        material=satin_steel,
        name="seat_clamp_rear",
    )
    frame.visual(
        Box((0.18, 0.12, 0.04)),
        origin=Origin(xyz=(0.31, 0.0, 0.71)),
        material=dark_steel,
        name="flywheel_bridge",
    )
    frame.visual(
        Box((0.18, 0.14, 0.04)),
        origin=Origin(xyz=(0.24, 0.0, 0.20)),
        material=dark_steel,
    )
    _add_tube(frame, (0.24, 0.060, 0.20), (0.31, 0.060, 0.69), 0.022, dark_steel, name="flywheel_upright_left")
    _add_tube(frame, (0.24, -0.060, 0.20), (0.31, -0.060, 0.69), 0.022, dark_steel, name="flywheel_upright_right")
    frame.visual(
        left_cover_mesh,
        origin=Origin(xyz=(0.28, 0.044, 0.46)),
        material=frame_black,
        name="flywheel_guard_left",
    )
    frame.visual(
        right_cover_mesh,
        origin=Origin(xyz=(0.28, -0.044, 0.46)),
        material=frame_black,
        name="flywheel_guard_right",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.046),
        origin=Origin(xyz=(0.28, 0.059, 0.46), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="flywheel_bearing_left",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.046),
        origin=Origin(xyz=(0.28, -0.059, 0.46), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="flywheel_bearing_right",
    )
    frame.visual(
        Box((0.10, 0.15, 0.10)),
        origin=Origin(xyz=(0.58, 0.0, 0.88)),
        material=dark_steel,
        name="bar_stem_block",
    )
    frame.visual(
        Box((0.08, 0.12, 0.06)),
        origin=Origin(xyz=(0.57, 0.0, 0.95)),
        material=dark_steel,
    )
    _add_tube(frame, (0.58, 0.0, 0.94), (0.56, 0.0, 1.12), 0.018, dark_steel, name="handlebar_stem")
    frame.visual(
        Box((0.050, 0.060, 0.030)),
        origin=Origin(xyz=(0.585, 0.0, 1.095)),
        material=dark_steel,
        name="handlebar_clamp",
    )
    handlebar_loop = tube_from_spline_points(
        [
            (0.46, -0.18, 1.09),
            (0.55, -0.11, 1.11),
            (0.60, 0.0, 1.09),
            (0.55, 0.11, 1.11),
            (0.46, 0.18, 1.09),
        ],
        radius=0.016,
        samples_per_segment=18,
        radial_segments=18,
    )
    frame.visual(_save_mesh("handlebar_loop", handlebar_loop), material=dark_steel, name="handlebar_loop")
    _add_tube(frame, (0.46, -0.18, 1.09), (0.47, -0.18, 1.015), 0.015, dark_steel, name="left_grip_stub")
    _add_tube(frame, (0.46, 0.18, 1.09), (0.47, 0.18, 1.015), 0.015, dark_steel, name="right_grip_stub")
    frame.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(0.53, -0.18, 1.015), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(0.53, 0.18, 1.015), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.23, 0.070, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_transport_tire",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.23, 0.070, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="left_transport_hub",
    )
    frame.visual(
        Box((0.030, 0.050, 0.060)),
        origin=Origin(xyz=(0.23, 0.055, 0.095)),
        material=dark_steel,
        name="left_transport_bracket",
    )
    frame.visual(
        Box((0.030, 0.050, 0.045)),
        origin=Origin(xyz=(0.23, 0.055, 0.1475)),
        material=dark_steel,
        name="left_transport_arm",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.23, -0.070, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_transport_tire",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.23, -0.070, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="right_transport_hub",
    )
    frame.visual(
        Box((0.030, 0.050, 0.060)),
        origin=Origin(xyz=(0.23, -0.055, 0.095)),
        material=dark_steel,
        name="right_transport_bracket",
    )
    frame.visual(
        Box((0.030, 0.050, 0.045)),
        origin=Origin(xyz=(0.23, -0.055, 0.1475)),
        material=dark_steel,
        name="right_transport_arm",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.05),
        mass=11.5,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.225, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=red_accent,
        name="flywheel_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.205, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="flywheel_core",
    )
    flywheel.visual(
        Cylinder(radius=0.058, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="flywheel_hub",
    )
    for spoke_index, (x, y, z, angle) in enumerate(_build_flywheel_spoke_positions(0.105)):
        flywheel.visual(
            Box((0.165, 0.018, 0.026)),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, angle, 0.0)),
            material=satin_steel,
            name=f"spoke_{spoke_index}",
        )
    flywheel.visual(
        Box((0.045, 0.020, 0.030)),
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
        material=red_accent,
        name="balance_weight",
    )

    crank_set = model.part("crank_set")
    crank_set.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.20)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crank_set.visual(
        Cylinder(radius=0.030, length=0.34),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="crank_axle",
    )
    crank_set.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.0, 0.100, 0.0)),
        material=red_accent,
        name="chainring",
    )
    crank_set.visual(
        Box((0.17, 0.012, 0.034)),
        origin=Origin(xyz=(0.085, 0.165, 0.0)),
        material=satin_steel,
        name="right_crank_arm",
    )
    crank_set.visual(
        Box((0.17, 0.012, 0.034)),
        origin=Origin(xyz=(-0.085, -0.165, 0.0)),
        material=satin_steel,
        name="left_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.17, 0.190, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(-0.17, -0.190, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Box((0.11, 0.050, 0.022)),
        origin=Origin(xyz=(0.17, 0.227, 0.0)),
        material=rubber,
        name="right_pedal",
    )
    crank_set.visual(
        Box((0.11, 0.050, 0.022)),
        origin=Origin(xyz=(-0.17, -0.227, 0.0)),
        material=rubber,
        name="left_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.26, 0.24, 0.58)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )
    seat_post.visual(
        Cylinder(radius=0.035, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=satin_steel,
        name="seat_slider",
    )
    seat_post.visual(
        Box((0.06, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=dark_steel,
        name="seat_head",
    )
    _add_tube(seat_post, (0.00, -0.030, 0.38), (0.035, -0.030, 0.425), 0.006, satin_steel)
    _add_tube(seat_post, (0.00, 0.030, 0.38), (0.035, 0.030, 0.425), 0.006, satin_steel)
    seat_post.visual(
        saddle_mesh,
        origin=Origin(xyz=(-0.01, 0.0, 0.43)),
        material=saddle_vinyl,
        name="saddle",
    )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.10, 0.0, 0.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.18),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_set,
        origin=Origin(xyz=(0.02, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )
    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.28, 0.0, 0.46)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    crank_set = object_model.get_part("crank_set")
    flywheel = object_model.get_part("flywheel")
    seat_height = object_model.get_articulation("seat_height")
    crank_spin = object_model.get_articulation("crank_spin")
    flywheel_spin = object_model.get_articulation("flywheel_spin")

    ctx.expect_gap(
        seat_post,
        frame,
        axis="x",
        positive_elem="seat_slider",
        negative_elem="seat_guide_left",
        max_gap=0.001,
        max_penetration=1e-6,
        name="seat post rides snug against left guide rail",
    )
    ctx.expect_gap(
        frame,
        seat_post,
        axis="x",
        positive_elem="seat_guide_right",
        negative_elem="seat_slider",
        max_gap=0.001,
        max_penetration=1e-6,
        name="seat post rides snug against right guide rail",
    )
    ctx.expect_gap(
        frame,
        seat_post,
        axis="y",
        positive_elem="seat_guide_front",
        negative_elem="seat_slider",
        max_gap=0.001,
        max_penetration=1e-6,
        name="seat post rides snug against front guide rail",
    )
    ctx.expect_gap(
        seat_post,
        frame,
        axis="y",
        positive_elem="seat_slider",
        negative_elem="seat_guide_rear",
        max_gap=0.001,
        max_penetration=1e-6,
        name="seat post rides snug against rear guide rail",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="seat_slider",
        elem_b="seat_guide_left",
        min_overlap=0.16,
        name="seat post remains inserted at lowest setting",
    )
    ctx.expect_gap(
        frame,
        flywheel,
        axis="y",
        positive_elem="flywheel_guard_left",
        negative_elem="flywheel_rim",
        min_gap=0.030,
        name="left flywheel cover clears rotating rim",
    )
    ctx.expect_gap(
        flywheel,
        frame,
        axis="y",
        positive_elem="flywheel_rim",
        negative_elem="flywheel_guard_right",
        min_gap=0.030,
        name="right flywheel cover clears rotating rim",
    )
    left_tire = ctx.part_element_world_aabb(frame, elem="left_transport_tire")
    right_tire = ctx.part_element_world_aabb(frame, elem="right_transport_tire")
    front_stabilizer = ctx.part_element_world_aabb(frame, elem="front_stabilizer")
    left_side_gap = None
    right_side_gap = None
    if left_tire is not None and front_stabilizer is not None:
        left_side_gap = left_tire[0][1] - front_stabilizer[1][1]
    if right_tire is not None and front_stabilizer is not None:
        right_side_gap = front_stabilizer[0][1] - right_tire[1][1]
    ctx.check(
        "left transport wheel sits close to frame",
        left_side_gap is not None and 0.0 <= left_side_gap <= 0.03,
        details=f"gap={left_side_gap}",
    )
    ctx.check(
        "right transport wheel sits close to frame",
        right_side_gap is not None and 0.0 <= right_side_gap <= 0.03,
        details=f"gap={right_side_gap}",
    )

    seat_rest = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.18}):
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="seat_slider",
            elem_b="seat_guide_left",
            min_overlap=0.10,
            name="seat post retains insertion at max height",
        )
        seat_high = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post raises upward",
        seat_rest is not None and seat_high is not None and seat_high[2] > seat_rest[2] + 0.12,
        details=f"rest={seat_rest}, high={seat_high}",
    )

    crank_rest = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
    with ctx.pose({crank_spin: pi / 2.0}):
        crank_quarter = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
    crank_dx = None
    crank_dz = None
    if crank_rest is not None and crank_quarter is not None:
        crank_rest_center = tuple((a + b) * 0.5 for a, b in zip(crank_rest[0], crank_rest[1]))
        crank_quarter_center = tuple((a + b) * 0.5 for a, b in zip(crank_quarter[0], crank_quarter[1]))
        crank_dx = abs(crank_quarter_center[0] - crank_rest_center[0])
        crank_dz = abs(crank_quarter_center[2] - crank_rest_center[2])
    ctx.check(
        "crank rotation moves pedal through the stroke",
        crank_dx is not None and crank_dz is not None and crank_dx > 0.10 and crank_dz > 0.10,
        details=f"rest={crank_rest}, quarter={crank_quarter}, dx={crank_dx}, dz={crank_dz}",
    )

    flywheel_rest = ctx.part_element_world_aabb(flywheel, elem="balance_weight")
    with ctx.pose({flywheel_spin: pi / 2.0}):
        flywheel_quarter = ctx.part_element_world_aabb(flywheel, elem="balance_weight")
    fly_dx = None
    fly_dz = None
    if flywheel_rest is not None and flywheel_quarter is not None:
        fly_rest_center = tuple((a + b) * 0.5 for a, b in zip(flywheel_rest[0], flywheel_rest[1]))
        fly_quarter_center = tuple((a + b) * 0.5 for a, b in zip(flywheel_quarter[0], flywheel_quarter[1]))
        fly_dx = abs(fly_quarter_center[0] - fly_rest_center[0])
        fly_dz = abs(fly_quarter_center[2] - fly_rest_center[2])
    ctx.check(
        "flywheel rotates about its axle",
        fly_dx is not None and fly_dz is not None and fly_dx > 0.14 and fly_dz > 0.14,
        details=f"rest={flywheel_rest}, quarter={flywheel_quarter}, dx={fly_dx}, dz={fly_dz}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
