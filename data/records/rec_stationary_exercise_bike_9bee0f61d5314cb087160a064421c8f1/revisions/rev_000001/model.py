from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.11, 0.12, 0.13, 1.0))
    housing_graphite = model.material("housing_graphite", rgba=(0.20, 0.21, 0.24, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.69, 0.70, 0.73, 1.0))
    seat_black = model.material("seat_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.08, 0.08, 0.08, 1.0))
    accent_red = model.material("accent_red", rgba=(0.68, 0.11, 0.09, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.76, 1.10)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.03, 0.55)),
    )

    frame.visual(
        Box((0.42, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, -0.29, 0.025)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.38, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.36, 0.025)),
        material=frame_black,
        name="front_stabilizer",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(0.0, 0.01, 0.28), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="bottom_bracket_shell",
    )

    _add_member(frame, (0.0, -0.24, 0.03), (0.0, -0.05, 0.28), 0.028, frame_black)
    _add_member(
        frame,
        (0.0, 0.40, 0.03),
        (0.0, 0.405, 0.56),
        0.024,
        frame_black,
        name="front_upright",
    )
    _add_member(
        frame,
        (-0.095, 0.31, 0.03),
        (-0.092, 0.21, 0.36),
        0.020,
        frame_black,
        name="left_flywheel_stay",
    )
    _add_member(
        frame,
        (0.095, 0.31, 0.03),
        (0.092, 0.21, 0.36),
        0.020,
        frame_black,
        name="right_flywheel_stay",
    )
    _add_member(frame, (0.0, -0.015, 0.30), (0.0, -0.125, 0.70), 0.029, frame_black, name="main_spine")
    _add_member(
        frame,
        (-0.068, -0.03, 0.31),
        (-0.068, -0.145, 0.67),
        0.013,
        frame_black,
        name="left_seat_support",
    )
    _add_member(
        frame,
        (0.068, -0.03, 0.31),
        (0.068, -0.145, 0.67),
        0.013,
        frame_black,
        name="right_seat_support",
    )
    _add_member(
        frame,
        (0.0, 0.405, 0.56),
        (0.0, 0.22, 0.94),
        0.029,
        frame_black,
        name="handlebar_mast",
    )

    frame.visual(
        Box((0.024, 0.075, 0.10)),
        origin=Origin(xyz=(-0.066, -0.125, 0.69)),
        material=frame_black,
        name="left_seat_clamp_cheek",
    )
    frame.visual(
        Box((0.024, 0.075, 0.10)),
        origin=Origin(xyz=(0.066, -0.125, 0.69)),
        material=frame_black,
        name="right_seat_clamp_cheek",
    )
    frame.visual(
        Box((0.17, 0.025, 0.05)),
        origin=Origin(xyz=(0.0, -0.165, 0.695)),
        material=frame_black,
        name="seat_clamp_block",
    )
    frame.visual(
        Cylinder(radius=0.160, length=0.012),
        origin=Origin(xyz=(-0.088, 0.21, 0.38), rpy=(0.0, pi / 2.0, 0.0)),
        material=housing_graphite,
        name="housing_left_cover",
    )
    frame.visual(
        Cylinder(radius=0.160, length=0.012),
        origin=Origin(xyz=(0.088, 0.21, 0.38), rpy=(0.0, pi / 2.0, 0.0)),
        material=housing_graphite,
        name="housing_right_cover",
    )
    frame.visual(
        Box((0.050, 0.10, 0.08)),
        origin=Origin(xyz=(-0.060, 0.20, 0.53)),
        material=housing_graphite,
        name="housing_left_upper_shroud",
    )
    frame.visual(
        Box((0.050, 0.10, 0.08)),
        origin=Origin(xyz=(0.060, 0.20, 0.53)),
        material=housing_graphite,
        name="housing_right_upper_shroud",
    )
    frame.visual(
        Box((0.020, 0.34, 0.11)),
        origin=Origin(xyz=(-0.078, 0.065, 0.185)),
        material=housing_graphite,
        name="housing_left_lower_fairing",
    )
    frame.visual(
        Box((0.020, 0.34, 0.11)),
        origin=Origin(xyz=(0.078, 0.065, 0.185)),
        material=housing_graphite,
        name="housing_right_lower_fairing",
    )
    frame.visual(
        Box((0.16, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -0.13, 0.30)),
        material=housing_graphite,
        name="chaincase_rear",
    )
    frame.visual(
        Box((0.024, 0.06, 0.10)),
        origin=Origin(xyz=(-0.092, 0.21, 0.38)),
        material=frame_black,
        name="left_axle_bracket",
    )
    frame.visual(
        Box((0.024, 0.06, 0.10)),
        origin=Origin(xyz=(0.092, 0.21, 0.38)),
        material=frame_black,
        name="right_axle_bracket",
    )

    frame.visual(
        Cylinder(radius=0.024, length=0.14),
        origin=Origin(xyz=(0.0, 0.22, 0.94)),
        material=satin_steel,
        name="bar_stem",
    )
    frame.visual(
        _save_mesh(
            "exercise_bike_handlebar",
            tube_from_spline_points(
                [
                    (-0.18, -0.03, 0.00),
                    (-0.15, 0.00, 0.03),
                    (-0.09, 0.02, 0.06),
                    (0.09, 0.02, 0.06),
                    (0.15, 0.00, 0.03),
                    (0.18, -0.03, 0.00),
                ],
                radius=0.017,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.22, 0.95)),
        material=frame_black,
        name="handlebar",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.11),
        origin=Origin(xyz=(-0.17, 0.19, 0.95), rpy=(0.0, pi / 2.0, 0.0)),
        material=pedal_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.11),
        origin=Origin(xyz=(0.17, 0.19, 0.95), rpy=(0.0, pi / 2.0, 0.0)),
        material=pedal_black,
        name="right_grip",
    )
    frame.visual(
        Box((0.08, 0.03, 0.06)),
        origin=Origin(xyz=(0.0, 0.22, 0.98)),
        material=housing_graphite,
        name="console_block",
    )
    frame.visual(
        Box((0.03, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.31, 0.07)),
        material=accent_red,
        name="front_transport_wheel_cover",
    )

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.145, length=0.045),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_red,
        name="flywheel_disk",
    )
    flywheel.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="flywheel_axle",
    )
    flywheel.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=housing_graphite,
        name="flywheel_hub",
    )
    flywheel.visual(
        Box((0.008, 0.028, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.126)),
        material=satin_steel,
        name="flywheel_counterweight",
    )
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.045),
        mass=8.0,
        origin=Origin(),
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.015, length=0.24),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="crank_axle",
    )
    crankset.visual(
        Box((0.022, 0.030, 0.19)),
        origin=Origin(xyz=(-0.11, 0.0, 0.10)),
        material=satin_steel,
        name="left_crank_arm",
    )
    crankset.visual(
        Box((0.10, 0.040, 0.018)),
        origin=Origin(xyz=(-0.115, 0.031, 0.198)),
        material=pedal_black,
        name="left_pedal",
    )
    crankset.visual(
        Box((0.022, 0.030, 0.19)),
        origin=Origin(xyz=(0.11, 0.0, -0.10)),
        material=satin_steel,
        name="right_crank_arm",
    )
    crankset.visual(
        Box((0.10, 0.040, 0.018)),
        origin=Origin(xyz=(0.115, -0.031, -0.198)),
        material=pedal_black,
        name="right_pedal",
    )
    crankset.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.42)),
        mass=3.2,
        origin=Origin(),
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.24, 0.34, 0.50)),
        mass=4.5,
        origin=Origin(xyz=(0.0, -0.08, 0.25)),
    )

    seat_axis_raw = (0.0, -0.26, 0.966)
    seat_axis_len = sqrt(sum(component * component for component in seat_axis_raw))
    seat_axis = tuple(component / seat_axis_len for component in seat_axis_raw)
    post_lower = tuple(-0.14 * component for component in seat_axis)
    post_upper = tuple(0.30 * component for component in seat_axis)
    _add_member(seat_post, post_lower, post_upper, 0.018, satin_steel, name="seat_post_tube")
    seat_post.visual(
        Box((0.060, 0.07, 0.06)),
        origin=Origin(xyz=(0.0, -0.090, 0.290)),
        material=frame_black,
        name="seat_slider_head",
    )
    seat_post.visual(
        Cylinder(radius=0.006, length=0.12),
        origin=Origin(xyz=(-0.03, -0.102, 0.314), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="left_seat_rail",
    )
    seat_post.visual(
        Cylinder(radius=0.006, length=0.12),
        origin=Origin(xyz=(0.03, -0.102, 0.314), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="right_seat_rail",
    )
    seat_post.visual(
        Box((0.12, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, -0.074, 0.319)),
        material=frame_black,
        name="seat_base_pan",
    )
    seat_post.visual(
        Box((0.16, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, -0.112, 0.354)),
        material=seat_black,
        name="seat_rear_pad",
    )
    seat_post.visual(
        Box((0.09, 0.16, 0.036)),
        origin=Origin(xyz=(0.0, -0.024, 0.352)),
        material=seat_black,
        name="seat_nose",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.0, 0.21, 0.38)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=20.0),
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(0.0, 0.01, 0.28)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=12.0),
    )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(0.0, -0.10, 0.68)),
        axis=seat_axis,
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    frame = object_model.get_part("frame")
    crankset = object_model.get_part("crankset")
    flywheel = object_model.get_part("flywheel")
    seat_post = object_model.get_part("seat_post")
    crank_spin = object_model.get_articulation("crank_spin")
    flywheel_spin = object_model.get_articulation("flywheel_spin")
    seat_height = object_model.get_articulation("seat_height")

    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="bottom_bracket_shell",
        elem_b="crank_axle",
        reason="The crank axle is intentionally represented as passing through the bottom-bracket shell.",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="main_spine",
        elem_b="seat_post_tube",
        reason="The seat post is intentionally represented as telescoping inside the simplified seat-tube sleeve.",
    )

    ctx.check(
        "exercise-bike articulations are configured",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS
        and flywheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and seat_height.articulation_type == ArticulationType.PRISMATIC
        and crank_spin.axis == (-1.0, 0.0, 0.0)
        and flywheel_spin.axis == (-1.0, 0.0, 0.0),
        details=(
            f"crank={crank_spin.articulation_type}/{crank_spin.axis}, "
            f"flywheel={flywheel_spin.articulation_type}/{flywheel_spin.axis}, "
            f"seat={seat_height.articulation_type}/{seat_height.axis}"
        ),
    )

    ctx.expect_gap(
        seat_post,
        frame,
        axis="z",
        min_gap=0.02,
        positive_elem="seat_rear_pad",
        negative_elem="seat_clamp_block",
        name="saddle sits above the frame clamp",
    )

    rest_left_pedal = ctx.part_element_world_aabb(crankset, elem="left_pedal")
    with ctx.pose({crank_spin: pi / 2.0}):
        quarter_turn_left_pedal = ctx.part_element_world_aabb(crankset, elem="left_pedal")

    rest_counterweight = ctx.part_element_world_aabb(flywheel, elem="flywheel_counterweight")
    with ctx.pose({flywheel_spin: pi / 2.0}):
        quarter_turn_counterweight = ctx.part_element_world_aabb(flywheel, elem="flywheel_counterweight")

    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_height: 0.16}):
        raised_pos = ctx.part_world_position(seat_post)

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    ctx.check(
        "left pedal advances through a quarter turn",
        rest_left_pedal is not None
        and quarter_turn_left_pedal is not None
        and _center_z(rest_left_pedal) is not None
        and _center_z(quarter_turn_left_pedal) is not None
        and _center_y(rest_left_pedal) is not None
        and _center_y(quarter_turn_left_pedal) is not None
        and _center_z(quarter_turn_left_pedal) < _center_z(rest_left_pedal) - 0.12
        and _center_y(quarter_turn_left_pedal) > _center_y(rest_left_pedal) + 0.12,
        details=f"rest={rest_left_pedal}, quarter={quarter_turn_left_pedal}",
    )

    ctx.check(
        "flywheel rotates about its axle",
        rest_counterweight is not None
        and quarter_turn_counterweight is not None
        and _center_z(rest_counterweight) is not None
        and _center_z(quarter_turn_counterweight) is not None
        and _center_y(rest_counterweight) is not None
        and _center_y(quarter_turn_counterweight) is not None
        and _center_z(quarter_turn_counterweight) < _center_z(rest_counterweight) - 0.12
        and _center_y(quarter_turn_counterweight) > _center_y(rest_counterweight) + 0.12,
        details=f"rest={rest_counterweight}, quarter={quarter_turn_counterweight}",
    )
    ctx.expect_contact(
        flywheel,
        frame,
        elem_a="flywheel_axle",
        elem_b="left_axle_bracket",
        contact_tol=1e-5,
        name="left flywheel bearing support contacts the axle",
    )
    ctx.expect_contact(
        flywheel,
        frame,
        elem_a="flywheel_axle",
        elem_b="right_axle_bracket",
        contact_tol=1e-5,
        name="right flywheel bearing support contacts the axle",
    )

    ctx.check(
        "seat post raises upward when extended",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.14
        and raised_pos[1] < rest_pos[1] - 0.02,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
