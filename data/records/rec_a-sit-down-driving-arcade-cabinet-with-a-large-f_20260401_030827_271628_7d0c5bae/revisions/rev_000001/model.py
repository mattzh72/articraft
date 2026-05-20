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
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sit_down_driving_arcade_cabinet")

    cabinet_blue = model.material("cabinet_blue", rgba=(0.10, 0.23, 0.52, 1.0))
    cabinet_navy = model.material("cabinet_navy", rgba=(0.07, 0.11, 0.23, 1.0))
    black_trim = model.material("black_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    textured_black = model.material("textured_black", rgba=(0.12, 0.12, 0.13, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.15, 0.15, 0.16, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.22, 0.35, 0.48, 0.50))
    wheel_metal = model.material("wheel_metal", rgba=(0.60, 0.62, 0.66, 1.0))
    column_gray = model.material("column_gray", rgba=(0.30, 0.32, 0.35, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((1.72, 0.96, 1.68)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
    )

    cabinet.visual(
        Box((1.72, 0.96, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=cabinet_navy,
        name="base_plinth",
    )
    cabinet.visual(
        Box((1.60, 0.08, 0.44)),
        origin=Origin(xyz=(0.0, 0.44, 0.22)),
        material=cabinet_blue,
        name="left_lower_side",
    )
    cabinet.visual(
        Box((1.60, 0.08, 0.44)),
        origin=Origin(xyz=(0.0, -0.44, 0.22)),
        material=cabinet_blue,
        name="right_lower_side",
    )
    cabinet.visual(
        Box((0.22, 0.88, 0.42)),
        origin=Origin(xyz=(-0.73, 0.0, 0.21)),
        material=cabinet_blue,
        name="rear_bulkhead",
    )
    cabinet.visual(
        Box((0.86, 0.18, 0.08)),
        origin=Origin(xyz=(-0.22, 0.33, 0.48)),
        material=cabinet_blue,
        name="left_armrest_deck",
    )
    cabinet.visual(
        Box((0.86, 0.18, 0.08)),
        origin=Origin(xyz=(-0.22, -0.33, 0.48)),
        material=cabinet_blue,
        name="right_armrest_deck",
    )
    cabinet.visual(
        Box((0.56, 0.64, 0.30)),
        origin=Origin(xyz=(-0.48, 0.0, 0.20)),
        material=cabinet_navy,
        name="seat_pedestal",
    )
    cabinet.visual(
        Box((0.92, 0.30, 0.42)),
        origin=Origin(xyz=(-0.05, 0.0, 0.21)),
        material=cabinet_navy,
        name="center_console",
    )
    cabinet.visual(
        Box((0.26, 0.74, 0.28)),
        origin=Origin(xyz=(0.24, 0.0, 0.57)),
        material=cabinet_blue,
        name="dashboard_support",
    )
    cabinet.visual(
        Box((0.34, 0.78, 0.10)),
        origin=Origin(xyz=(0.34, 0.0, 0.73), rpy=(0.0, -0.34, 0.0)),
        material=black_trim,
        name="dashboard_top",
    )
    cabinet.visual(
        Box((0.24, 0.58, 0.05)),
        origin=Origin(xyz=(0.20, 0.0, 0.76), rpy=(0.0, -0.34, 0.0)),
        material=textured_black,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.56, 0.88, 0.76)),
        origin=Origin(xyz=(0.57, 0.0, 0.43)),
        material=cabinet_blue,
        name="front_housing",
    )
    cabinet.visual(
        Box((0.22, 0.82, 0.86)),
        origin=Origin(xyz=(0.58, 0.0, 1.19)),
        material=cabinet_blue,
        name="display_surround",
    )
    cabinet.visual(
        Box((0.44, 0.88, 0.12)),
        origin=Origin(xyz=(0.36, 0.0, 1.40), rpy=(0.0, -0.28, 0.0)),
        material=cabinet_navy,
        name="display_hood",
    )
    cabinet.visual(
        Box((0.06, 0.76, 0.66)),
        origin=Origin(xyz=(0.69, 0.0, 1.18)),
        material=black_trim,
        name="bezel_frame",
    )
    cabinet.visual(
        Box((0.012, 0.60, 0.44)),
        origin=Origin(xyz=(0.681, 0.0, 1.18)),
        material=smoked_glass,
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.14, 0.22, 0.16)),
        origin=Origin(xyz=(0.12, 0.0, 0.80)),
        material=column_gray,
        name="column_shroud",
    )
    cabinet.visual(
        Cylinder(radius=0.032, length=0.16),
        origin=Origin(xyz=(0.12, 0.0, 0.80), rpy=(0.0, pi / 2.0, 0.0)),
        material=column_gray,
        name="wheel_spindle",
    )
    cabinet.visual(
        Box((0.12, 0.22, 0.07)),
        origin=Origin(xyz=(0.25, 0.0, 0.88), rpy=(0.0, -0.18, 0.0)),
        material=black_trim,
        name="instrument_cluster",
    )
    cabinet.visual(
        Box((0.52, 0.46, 0.12)),
        origin=Origin(xyz=(-0.46, 0.0, 0.41)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    cabinet.visual(
        Box((0.38, 0.08, 0.18)),
        origin=Origin(xyz=(-0.48, 0.19, 0.44)),
        material=seat_vinyl,
        name="left_seat_bolster",
    )
    cabinet.visual(
        Box((0.38, 0.08, 0.18)),
        origin=Origin(xyz=(-0.48, -0.19, 0.44)),
        material=seat_vinyl,
        name="right_seat_bolster",
    )
    cabinet.visual(
        Box((0.20, 0.56, 0.10)),
        origin=Origin(xyz=(-0.24, 0.0, 0.36)),
        material=seat_vinyl,
        name="seat_front_bolster",
    )
    cabinet.visual(
        Box((0.18, 0.54, 0.64)),
        origin=Origin(xyz=(-0.70, 0.0, 0.73), rpy=(0.0, -0.22, 0.0)),
        material=seat_vinyl,
        name="seat_backrest",
    )
    cabinet.visual(
        Box((0.12, 0.30, 0.12)),
        origin=Origin(xyz=(-0.78, 0.0, 1.05), rpy=(0.0, -0.22, 0.0)),
        material=seat_vinyl,
        name="headrest",
    )
    cabinet.visual(
        Box((0.12, 0.34, 0.10)),
        origin=Origin(xyz=(0.48, 0.0, 0.17)),
        material=black_trim,
        name="pedal_box",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(0.46, 0.0, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=column_gray,
        name="pedal_pivot_bar",
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Box((0.03, 0.08, 0.16)),
        origin=Origin(xyz=(0.04, 0.0, 0.12), rpy=(0.0, -0.48, 0.0)),
        material=wheel_metal,
        name="pedal_face",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.05, 0.08, 0.20)),
        mass=0.18,
        origin=Origin(xyz=(0.04, 0.0, 0.12)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Box((0.03, 0.08, 0.16)),
        origin=Origin(xyz=(0.04, 0.0, 0.12), rpy=(0.0, -0.38, 0.0)),
        material=wheel_metal,
        name="pedal_face",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.05, 0.08, 0.20)),
        mass=0.18,
        origin=Origin(xyz=(0.04, 0.0, 0.12)),
    )

    steering_wheel = model.part("steering_wheel")
    steering_wheel.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.12)),
        mass=1.8,
    )

    steering_ring = _mesh(
        "steering_ring",
        TorusGeometry(radius=0.205, tube=0.022, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0),
    )
    steering_wheel.visual(
        steering_ring,
        material=textured_black,
        name="wheel_ring",
    )
    steering_wheel.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(0.01, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=column_gray,
        name="hub_barrel",
    )
    steering_wheel.visual(
        Box((0.06, 0.13, 0.11)),
        origin=Origin(xyz=(-0.02, 0.0, -0.01)),
        material=black_trim,
        name="center_pad",
    )
    steering_wheel.visual(
        Box((0.024, 0.04, 0.24)),
        origin=Origin(xyz=(-0.005, 0.0, 0.12)),
        material=wheel_metal,
        name="top_spoke",
    )
    steering_wheel.visual(
        Box((0.024, 0.04, 0.18)),
        origin=Origin(xyz=(-0.005, 0.075, -0.055), rpy=(0.0, 0.0, 0.95)),
        material=wheel_metal,
        name="lower_left_spoke",
    )
    steering_wheel.visual(
        Box((0.024, 0.04, 0.18)),
        origin=Origin(xyz=(-0.005, -0.075, -0.055), rpy=(0.0, 0.0, -0.95)),
        material=wheel_metal,
        name="lower_right_spoke",
    )
    steering_wheel.visual(
        Box((0.018, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        material=wheel_metal,
        name="wheel_marker",
    )

    model.articulation(
        "steering_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=steering_wheel,
        origin=Origin(xyz=(-0.02, 0.0, 0.80)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    model.articulation(
        "left_pedal_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_pedal,
        origin=Origin(xyz=(0.46, 0.09, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "right_pedal_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_pedal,
        origin=Origin(xyz=(0.46, -0.09, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    steering_wheel = object_model.get_part("steering_wheel")
    left_pedal_hinge = object_model.get_articulation("left_pedal_hinge")
    right_pedal_hinge = object_model.get_articulation("right_pedal_hinge")
    steering_spin = object_model.get_articulation("steering_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        steering_wheel,
        cabinet,
        elem_a="hub_barrel",
        elem_b="wheel_spindle",
        name="steering wheel is mounted on the steering spindle",
    )
    ctx.check(
        "pedals hinge on transverse y axes",
        left_pedal_hinge.axis == (0.0, 1.0, 0.0) and right_pedal_hinge.axis == (0.0, 1.0, 0.0),
        details=f"left={left_pedal_hinge.axis}, right={right_pedal_hinge.axis}",
    )

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    if cabinet_aabb is not None:
        overall_dims = tuple(cabinet_aabb[1][axis] - cabinet_aabb[0][axis] for axis in range(3))
        ctx.check(
            "cabinet stays at realistic sit-down arcade scale",
            1.55 <= overall_dims[0] <= 1.90
            and 0.82 <= overall_dims[1] <= 1.05
            and 1.45 <= overall_dims[2] <= 1.80,
            details=f"dims={overall_dims}",
        )
    else:
        ctx.fail("cabinet AABB available", "cabinet world AABB was unavailable")

    seat_aabb = ctx.part_element_world_aabb(cabinet, elem="seat_cushion")
    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen_glass")
    wheel_rest_pos = ctx.part_world_position(steering_wheel)

    if seat_aabb is not None:
        seat_top = seat_aabb[1][2]
        ctx.check(
            "seat cushion height is plausible for a sit-down cabinet",
            0.43 <= seat_top <= 0.50,
            details=f"seat_top={seat_top}",
        )
    else:
        ctx.fail("seat cushion measurable", "seat cushion AABB was unavailable")

    if seat_aabb is not None and screen_aabb is not None and wheel_rest_pos is not None:
        seat_center = _aabb_center(seat_aabb)
        screen_center = _aabb_center(screen_aabb)
        ctx.check(
            "steering wheel sits between seat and screen",
            seat_center[0] < wheel_rest_pos[0] < screen_center[0]
            and screen_center[2] > wheel_rest_pos[2] + 0.25,
            details=f"seat_center={seat_center}, wheel={wheel_rest_pos}, screen_center={screen_center}",
        )
    else:
        ctx.fail(
            "driving layout measurable",
            f"seat_aabb={seat_aabb}, screen_aabb={screen_aabb}, wheel_rest_pos={wheel_rest_pos}",
        )

    left_pedal_rest = ctx.part_element_world_aabb(left_pedal, elem="pedal_face")
    right_pedal_rest = ctx.part_element_world_aabb(right_pedal, elem="pedal_face")
    with ctx.pose(
        {
            left_pedal_hinge: left_pedal_hinge.motion_limits.upper,
            right_pedal_hinge: right_pedal_hinge.motion_limits.upper,
        }
    ):
        left_pedal_pressed = ctx.part_element_world_aabb(left_pedal, elem="pedal_face")
        right_pedal_pressed = ctx.part_element_world_aabb(right_pedal, elem="pedal_face")

    if (
        left_pedal_rest is not None
        and left_pedal_pressed is not None
        and right_pedal_rest is not None
        and right_pedal_pressed is not None
    ):
        left_rest_center = _aabb_center(left_pedal_rest)
        left_pressed_center = _aabb_center(left_pedal_pressed)
        right_rest_center = _aabb_center(right_pedal_rest)
        right_pressed_center = _aabb_center(right_pedal_pressed)
        ctx.check(
            "left pedal rotates downward when pressed",
            left_pressed_center[0] > left_rest_center[0] + 0.01
            and left_pressed_center[2] < left_rest_center[2] - 0.01,
            details=f"rest={left_rest_center}, pressed={left_pressed_center}",
        )
        ctx.check(
            "right pedal rotates downward when pressed",
            right_pressed_center[0] > right_rest_center[0] + 0.01
            and right_pressed_center[2] < right_rest_center[2] - 0.01,
            details=f"rest={right_rest_center}, pressed={right_pressed_center}",
        )
    else:
        ctx.fail(
            "pedal travel measurable",
            (
                f"left_rest={left_pedal_rest}, left_pressed={left_pedal_pressed}, "
                f"right_rest={right_pedal_rest}, right_pressed={right_pedal_pressed}"
            ),
        )

    marker_rest_aabb = ctx.part_element_world_aabb(steering_wheel, elem="wheel_marker")
    with ctx.pose({steering_spin: pi / 2.0}):
        marker_turn_aabb = ctx.part_element_world_aabb(steering_wheel, elem="wheel_marker")
        wheel_turn_pos = ctx.part_world_position(steering_wheel)

    if (
        marker_rest_aabb is not None
        and marker_turn_aabb is not None
        and wheel_rest_pos is not None
        and wheel_turn_pos is not None
    ):
        marker_rest_center = _aabb_center(marker_rest_aabb)
        marker_turn_center = _aabb_center(marker_turn_aabb)
        ctx.check(
            "steering wheel spins around its fore-aft axis",
            abs(wheel_turn_pos[0] - wheel_rest_pos[0]) < 1e-6
            and abs(wheel_turn_pos[1] - wheel_rest_pos[1]) < 1e-6
            and abs(wheel_turn_pos[2] - wheel_rest_pos[2]) < 1e-6
            and abs(marker_turn_center[0] - marker_rest_center[0]) < 0.03
            and marker_turn_center[1] < marker_rest_center[1] - 0.14
            and marker_turn_center[2] < marker_rest_center[2] - 0.14,
            details=f"rest={marker_rest_center}, turned={marker_turn_center}",
        )
    else:
        ctx.fail(
            "steering marker measurable",
            (
                f"marker_rest_aabb={marker_rest_aabb}, marker_turn_aabb={marker_turn_aabb}, "
                f"wheel_rest_pos={wheel_rest_pos}, wheel_turn_pos={wheel_turn_pos}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
