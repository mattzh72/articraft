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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_profile_2d(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    return [
        (
            center[0] + radius * cos(2.0 * pi * index / segments),
            center[1] + radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _housing_side_cover_profile() -> list[tuple[float, float]]:
    return [
        (-0.205, -0.090),
        (-0.220, -0.020),
        (-0.205, 0.085),
        (-0.145, 0.185),
        (-0.040, 0.245),
        (0.090, 0.225),
        (0.190, 0.140),
        (0.235, 0.035),
        (0.228, -0.045),
        (0.188, -0.095),
        (0.120, -0.118),
        (0.020, -0.108),
        (-0.090, -0.092),
        (-0.170, -0.102),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.13, 0.14, 0.15, 1.0))
    housing_grey = model.material("housing_grey", rgba=(0.20, 0.21, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.11, 0.11, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.86, 1.16, 1.12)),
        mass=38.0,
        origin=Origin(xyz=(0.0, -0.02, 0.56)),
    )

    frame.visual(
        Box((0.78, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.52, 0.03)),
        material=frame_black,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.82, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, -0.54, 0.03)),
        material=frame_black,
        name="rear_stabilizer",
    )
    for side, x in (("left", 0.36), ("right", -0.36)):
        frame.visual(
            Cylinder(radius=0.04, length=0.036),
            origin=Origin(xyz=(x, 0.52, 0.04), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=f"front_{side}_transport_wheel",
        )
        frame.visual(
            Box((0.05, 0.03, 0.06)),
            origin=Origin(xyz=(x, 0.51, 0.08)),
            material=frame_black,
            name=f"front_{side}_wheel_bracket",
        )

    frame.visual(
        Box((0.12, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.075, 0.289)),
        material=dark_steel,
        name="bottom_bracket_pod",
    )

    housing_cover_mesh = _save_mesh(
        "housing_side_cover",
        ExtrudeGeometry(
            _housing_side_cover_profile(),
            height=0.008,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    frame.visual(
        housing_cover_mesh,
        origin=Origin(xyz=(0.102, 0.22, 0.50)),
        material=housing_grey,
        name="housing_left_cover",
    )
    frame.visual(
        housing_cover_mesh,
        origin=Origin(xyz=(-0.102, 0.22, 0.50)),
        material=housing_grey,
        name="housing_right_cover",
    )
    frame.visual(
        Box((0.22, 0.07, 0.03)),
        origin=Origin(xyz=(0.0, 0.22, 0.72)),
        material=housing_grey,
        name="housing_top_bridge",
    )
    frame.visual(
        Box((0.22, 0.03, 0.12)),
        origin=Origin(xyz=(0.0, 0.45, 0.50)),
        material=housing_grey,
        name="housing_front_bridge",
    )
    frame.visual(
        Box((0.22, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.14, 0.31)),
        material=housing_grey,
        name="housing_lower_bridge",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.044),
        origin=Origin(xyz=(0.102, 0.22, 0.50), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="housing_left_axle_boss",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.044),
        origin=Origin(xyz=(-0.102, 0.22, 0.50), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="housing_right_axle_boss",
    )

    _add_member(
        frame,
        (0.0, 0.18, 0.70),
        (0.0, -0.18, 0.63),
        0.032,
        frame_black,
        name="main_beam",
    )
    _add_member(
        frame,
        (0.11, 0.52, 0.03),
        (0.09, 0.20, 0.70),
        0.028,
        frame_black,
        name="left_down_tube",
    )
    _add_member(
        frame,
        (-0.11, 0.52, 0.03),
        (-0.09, 0.20, 0.70),
        0.028,
        frame_black,
        name="right_down_tube",
    )
    frame.visual(
        Box((0.20, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.20, 0.70)),
        material=frame_black,
        name="front_upright_bridge",
    )
    _add_member(
        frame,
        (0.05, 0.14, 0.31),
        (0.21, -0.54, 0.03),
        0.024,
        frame_black,
        name="left_lower_rear_stay",
    )
    _add_member(
        frame,
        (-0.05, 0.14, 0.31),
        (-0.21, -0.54, 0.03),
        0.024,
        frame_black,
        name="right_lower_rear_stay",
    )
    _add_member(
        frame,
        (0.055, -0.285, 0.49),
        (0.21, -0.54, 0.03),
        0.017,
        frame_black,
        name="left_seat_stay",
    )
    _add_member(
        frame,
        (-0.055, -0.285, 0.49),
        (-0.21, -0.54, 0.03),
        0.017,
        frame_black,
        name="right_seat_stay",
    )
    frame.visual(
        Box((0.050, 0.085, 0.200)),
        origin=Origin(xyz=(0.063, -0.258, 0.545)),
        material=frame_black,
        name="left_seat_cluster_gusset",
    )
    frame.visual(
        Box((0.050, 0.085, 0.200)),
        origin=Origin(xyz=(-0.063, -0.258, 0.545)),
        material=frame_black,
        name="right_seat_cluster_gusset",
    )
    _add_member(
        frame,
        (0.0, 0.20, 0.72),
        (0.0, 0.30, 0.95),
        0.037,
        frame_black,
        name="handlebar_mast",
    )
    _add_member(
        frame,
        (0.0, 0.30, 0.95),
        (0.0, 0.30, 1.03),
        0.026,
        dark_steel,
        name="handlebar_stem",
    )

    frame.visual(
        Box((0.016, 0.055, 0.22)),
        origin=Origin(xyz=(0.037, -0.25, 0.59)),
        material=frame_black,
        name="seat_sleeve_left",
    )
    frame.visual(
        Box((0.016, 0.055, 0.22)),
        origin=Origin(xyz=(-0.037, -0.25, 0.59)),
        material=frame_black,
        name="seat_sleeve_right",
    )
    frame.visual(
        Box((0.058, 0.0105, 0.22)),
        origin=Origin(xyz=(0.0, -0.22725, 0.59)),
        material=frame_black,
        name="seat_sleeve_front",
    )
    frame.visual(
        Box((0.058, 0.0105, 0.22)),
        origin=Origin(xyz=(0.0, -0.27275, 0.59)),
        material=frame_black,
        name="seat_sleeve_rear",
    )
    frame.visual(
        Box((0.02, 0.07, 0.03)),
        origin=Origin(xyz=(0.044, -0.25, 0.71)),
        material=dark_steel,
        name="seat_clamp_left",
    )
    frame.visual(
        Box((0.02, 0.07, 0.03)),
        origin=Origin(xyz=(-0.044, -0.25, 0.71)),
        material=dark_steel,
        name="seat_clamp_right",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.11),
        origin=Origin(xyz=(0.0, -0.282, 0.712), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="seat_clamp_bolt",
    )

    handlebar_geom = tube_from_spline_points(
        [
            (-0.24, 0.30, 0.98),
            (-0.18, 0.31, 1.02),
            (-0.07, 0.30, 1.05),
            (0.07, 0.30, 1.05),
            (0.18, 0.31, 1.02),
            (0.24, 0.30, 0.98),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
    )
    frame.visual(
        _save_mesh("handlebar_bar", handlebar_geom),
        material=steel,
        name="handlebar_bar",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.11),
        origin=Origin(xyz=(0.24, 0.30, 0.98), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.11),
        origin=Origin(xyz=(-0.24, 0.30, 0.98), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame.visual(
        Box((0.10, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.31, 1.03)),
        material=dark_steel,
        name="console_block",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.28, 0.34, 0.42)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.02, 0.21)),
    )
    seat_post.visual(
        Box((0.052, 0.028, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=steel,
        name="seat_post_inner",
    )
    seat_post.visual(
        Box((0.003, 0.018, 0.18)),
        origin=Origin(xyz=(0.0275, 0.0, -0.08)),
        material=rubber,
        name="left_seat_guide_pad",
    )
    seat_post.visual(
        Box((0.003, 0.018, 0.18)),
        origin=Origin(xyz=(-0.0275, 0.0, -0.08)),
        material=rubber,
        name="right_seat_guide_pad",
    )
    seat_post.visual(
        Box((0.040, 0.0035, 0.18)),
        origin=Origin(xyz=(0.0, 0.01575, -0.08)),
        material=rubber,
        name="front_seat_guide_pad",
    )
    seat_post.visual(
        Box((0.040, 0.0035, 0.18)),
        origin=Origin(xyz=(0.0, -0.01575, -0.08)),
        material=rubber,
        name="rear_seat_guide_pad",
    )
    seat_post.visual(
        Box((0.10, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=dark_steel,
        name="seat_carriage",
    )
    saddle_geom = superellipse_side_loft(
        [
            (-0.11, 0.24, 0.285, 0.16),
            (-0.02, 0.24, 0.315, 0.21),
            (0.08, 0.25, 0.295, 0.12),
        ],
        exponents=2.1,
        segments=42,
    )
    seat_post.visual(
        _save_mesh("saddle_shell", saddle_geom),
        material=saddle_black,
        name="saddle_shell",
    )

    crankset = model.part("crankset")
    crankset.inertial = Inertial.from_geometry(
        Box((0.44, 0.22, 0.42)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.172),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="crank_spindle",
    )
    crankset.visual(
        Cylinder(radius=0.105, length=0.012),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="chainring",
    )
    crankset.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_crank_boss",
    )
    crankset.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(-0.098, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_crank_boss",
    )
    _add_member(
        crankset,
        (0.109, 0.0, -0.020),
        (0.215, 0.0, -0.176),
        0.014,
        steel,
        name="right_crank_arm",
    )
    _add_member(
        crankset,
        (-0.109, 0.0, 0.020),
        (-0.215, 0.0, 0.176),
        0.014,
        steel,
        name="left_crank_arm",
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.06),
        origin=Origin(xyz=(0.239, 0.0, -0.176), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_pedal_axle",
    )
    crankset.visual(
        Cylinder(radius=0.008, length=0.06),
        origin=Origin(xyz=(-0.239, 0.0, 0.176), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_pedal_axle",
    )
    crankset.visual(
        Box((0.034, 0.095, 0.022)),
        origin=Origin(xyz=(0.256, 0.0, -0.176)),
        material=rubber,
        name="right_pedal",
    )
    crankset.visual(
        Box((0.034, 0.095, 0.022)),
        origin=Origin(xyz=(-0.256, 0.0, 0.176)),
        material=rubber,
        name="left_pedal",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.04),
        mass=9.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.17, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="flywheel_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.145, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="flywheel_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.028, length=0.136),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="flywheel_hub",
    )
    flywheel.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="flywheel_left_bearing_cap",
    )
    flywheel.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(-0.074, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="flywheel_right_bearing_cap",
    )
    for index, angle in enumerate((0.0, pi / 4.0, pi / 2.0, 3.0 * pi / 4.0)):
        flywheel.visual(
            Box((0.008, 0.13, 0.018)),
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=steel,
            name=f"flywheel_spoke_{index}",
        )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.0, -0.25, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.12),
    )
    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(0.0, 0.05, 0.34)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=10.0),
    )
    model.articulation(
        "flywheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.0, 0.22, 0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    seat_height = object_model.get_articulation("seat_height")
    crank_rotation = object_model.get_articulation("crank_rotation")
    flywheel_rotation = object_model.get_articulation("flywheel_rotation")
    crankset = object_model.get_part("crankset")

    ctx.expect_gap(
        frame,
        seat_post,
        axis="x",
        positive_elem="seat_sleeve_left",
        negative_elem="seat_post_inner",
        min_gap=0.002,
        max_gap=0.0045,
        name="seat post clears the left sleeve wall at rest",
    )
    ctx.expect_gap(
        seat_post,
        frame,
        axis="x",
        positive_elem="seat_post_inner",
        negative_elem="seat_sleeve_right",
        min_gap=0.002,
        max_gap=0.0045,
        name="seat post clears the right sleeve wall at rest",
    )
    ctx.expect_gap(
        frame,
        seat_post,
        axis="y",
        positive_elem="seat_sleeve_front",
        negative_elem="seat_post_inner",
        min_gap=0.0025,
        max_gap=0.005,
        name="seat post clears the front sleeve wall at rest",
    )
    ctx.expect_gap(
        seat_post,
        frame,
        axis="y",
        positive_elem="seat_post_inner",
        negative_elem="seat_sleeve_rear",
        min_gap=0.0025,
        max_gap=0.005,
        name="seat post clears the rear sleeve wall at rest",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="seat_post_inner",
        elem_b="seat_sleeve_left",
        min_overlap=0.05,
        name="seat post retains insertion at rest",
    )

    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.12}):
        ctx.expect_gap(
            frame,
            seat_post,
            axis="x",
            positive_elem="seat_sleeve_left",
            negative_elem="seat_post_inner",
            min_gap=0.002,
            max_gap=0.0045,
            name="seat post clears the left sleeve wall when raised",
        )
        ctx.expect_gap(
            seat_post,
            frame,
            axis="x",
            positive_elem="seat_post_inner",
            negative_elem="seat_sleeve_right",
            min_gap=0.002,
            max_gap=0.0045,
            name="seat post clears the right sleeve wall when raised",
        )
        ctx.expect_gap(
            frame,
            seat_post,
            axis="y",
            positive_elem="seat_sleeve_front",
            negative_elem="seat_post_inner",
            min_gap=0.0025,
            max_gap=0.005,
            name="seat post clears the front sleeve wall when raised",
        )
        ctx.expect_gap(
            seat_post,
            frame,
            axis="y",
            positive_elem="seat_post_inner",
            negative_elem="seat_sleeve_rear",
            min_gap=0.0025,
            max_gap=0.005,
            name="seat post clears the rear sleeve wall when raised",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="seat_post_inner",
            elem_b="seat_sleeve_left",
            min_overlap=0.03,
            name="seat post keeps retained insertion when raised",
        )
        raised_pos = ctx.part_world_position(seat_post)

    def _center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    right_pedal_rest = _center(ctx.part_element_world_aabb(crankset, elem="right_pedal"))
    with ctx.pose({crank_rotation: pi / 2.0}):
        right_pedal_quarter = _center(ctx.part_element_world_aabb(crankset, elem="right_pedal"))

    frame_part = object_model.get_part("frame")
    left_wheel = frame_part.get_visual("front_left_transport_wheel")
    right_wheel = frame_part.get_visual("front_right_transport_wheel")
    left_x = left_wheel.origin.xyz[0]
    right_x = right_wheel.origin.xyz[0]
    ctx.check(
        "front transport wheels sit far outboard for a wide stance",
        left_x > 0.27 and right_x < -0.27,
        details=f"left_x={left_x}, right_x={right_x}",
    )
    ctx.check(
        "seat post raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.08,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.check(
        "crank and flywheel use continuous rotation joints",
        crank_rotation.articulation_type == ArticulationType.CONTINUOUS
        and flywheel_rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"crank_type={crank_rotation.articulation_type}, "
            f"flywheel_type={flywheel_rotation.articulation_type}"
        ),
    )
    ctx.check(
        "right pedal moves around the crank axis",
        right_pedal_rest is not None
        and right_pedal_quarter is not None
        and abs(right_pedal_quarter[2] - right_pedal_rest[2]) > 0.12
        and abs(right_pedal_quarter[1] - right_pedal_rest[1]) > 0.12,
        details=f"rest={right_pedal_rest}, quarter_turn={right_pedal_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
