from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sin, cos

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


AXLE_HEIGHT = 1.45
WHEEL_RADIUS = 1.00
WHEEL_HALF_THICKNESS = 0.14
SEAT_PIVOT_RADIUS = 0.86
SEAT_COUNT = 6


def _ring_point(radius: float, angle: float) -> tuple[float, float]:
    return radius * sin(angle), radius * cos(angle)


def _aabb_dims(aabb) -> tuple[float, float, float]:
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _add_frame_visuals(frame, *, frame_paint, axle_metal) -> None:
    front_frame = wire_from_points(
        [
            (0.30, -0.88, 0.04),
            (0.30, -0.68, 0.04),
            (0.30, 0.0, AXLE_HEIGHT),
            (0.30, 0.68, 0.04),
            (0.30, 0.88, 0.04),
        ],
        radius=0.035,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.08,
        corner_segments=10,
    )
    rear_frame = wire_from_points(
        [
            (-0.30, -0.88, 0.04),
            (-0.30, -0.68, 0.04),
            (-0.30, 0.0, AXLE_HEIGHT),
            (-0.30, 0.68, 0.04),
            (-0.30, 0.88, 0.04),
        ],
        radius=0.035,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.08,
        corner_segments=10,
    )
    front_tie = wire_from_points(
        [(-0.30, -0.88, 0.04), (0.30, -0.88, 0.04)],
        radius=0.03,
        radial_segments=18,
        cap_ends=True,
        corner_mode="miter",
    )
    rear_tie = wire_from_points(
        [(-0.30, 0.88, 0.04), (0.30, 0.88, 0.04)],
        radius=0.03,
        radial_segments=18,
        cap_ends=True,
        corner_mode="miter",
    )
    frame.visual(
        mesh_from_geometry(front_frame, "observation_wheel_front_frame_v2"),
        material=frame_paint,
        name="front_frame",
    )
    frame.visual(
        mesh_from_geometry(rear_frame, "observation_wheel_rear_frame_v2"),
        material=frame_paint,
        name="rear_frame",
    )
    frame.visual(
        mesh_from_geometry(front_tie, "observation_wheel_front_tie_v2"),
        material=frame_paint,
        name="front_tie",
    )
    frame.visual(
        mesh_from_geometry(rear_tie, "observation_wheel_rear_tie_v2"),
        material=frame_paint,
        name="rear_tie",
    )
    frame.visual(
        Box((0.72, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, -0.88, 0.045)),
        material=frame_paint,
        name="front_base_foot",
    )
    frame.visual(
        Box((0.72, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.88, 0.045)),
        material=frame_paint,
        name="rear_base_foot",
    )
    frame.visual(
        Box((0.08, 0.16, 0.22)),
        origin=Origin(xyz=(0.28, 0.0, AXLE_HEIGHT)),
        material=frame_paint,
        name="front_bearing_block",
    )
    frame.visual(
        Box((0.08, 0.16, 0.22)),
        origin=Origin(xyz=(-0.28, 0.0, AXLE_HEIGHT)),
        material=frame_paint,
        name="rear_bearing_block",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.05),
        origin=Origin(xyz=(0.215, 0.0, AXLE_HEIGHT), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_metal,
        name="front_axle_cap",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.05),
        origin=Origin(xyz=(-0.215, 0.0, AXLE_HEIGHT), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_metal,
        name="rear_axle_cap",
    )


def _add_wheel_visuals(wheel, *, rim_color, arm_color) -> None:
    rim_mesh = mesh_from_geometry(
        TorusGeometry(WHEEL_RADIUS, 0.03, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0),
        "observation_wheel_rim_v3",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(WHEEL_HALF_THICKNESS, 0.0, 0.0)),
        material=rim_color,
        name="front_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-WHEEL_HALF_THICKNESS, 0.0, 0.0)),
        material=rim_color,
        name="rear_rim",
    )
    wheel.visual(
        Cylinder(radius=0.12, length=0.38),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=arm_color,
        name="hub_shell",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.02),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_color,
        name="front_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.02),
        origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_color,
        name="rear_hub_flange",
    )

    arm_radius = 0.97
    pivot_radius = SEAT_PIVOT_RADIUS
    spoke_center_radius = (0.12 + arm_radius) * 0.5
    spoke_length = arm_radius - 0.12
    bracket_radius = (arm_radius + pivot_radius) * 0.5
    bracket_length = arm_radius - pivot_radius

    for index in range(SEAT_COUNT):
        angle = 2.0 * pi * index / SEAT_COUNT
        spoke_y, spoke_z = _ring_point(spoke_center_radius, angle)
        arm_y, arm_z = _ring_point(arm_radius, angle)
        bracket_y, bracket_z = _ring_point(bracket_radius, angle)
        pivot_y, pivot_z = _ring_point(pivot_radius, angle)

        for side_name, side_x in (("front", WHEEL_HALF_THICKNESS), ("rear", -WHEEL_HALF_THICKNESS)):
            wheel.visual(
                Cylinder(radius=0.013, length=spoke_length),
                origin=Origin(
                    xyz=(side_x, spoke_y, spoke_z),
                    rpy=(-angle, 0.0, 0.0),
                ),
                material=arm_color,
                name=f"{side_name}_spoke_{index}",
            )
            wheel.visual(
                Box((0.09, 0.014, bracket_length)),
                origin=Origin(
                    xyz=(side_x * 0.68, bracket_y, bracket_z),
                    rpy=(-angle, 0.0, 0.0),
                ),
                material=arm_color,
                name=f"{side_name}_hanger_strut_{index}",
            )

        wheel.visual(
            Box((0.016, 0.016, 0.05)),
            origin=Origin(xyz=(0.053, pivot_y, pivot_z)),
            material=arm_color,
            name=f"front_hanger_lug_{index}",
        )
        wheel.visual(
            Box((0.016, 0.016, 0.05)),
            origin=Origin(xyz=(-0.053, pivot_y, pivot_z)),
            material=arm_color,
            name=f"rear_hanger_lug_{index}",
        )


def _seat_color(index: int) -> str:
    return ("seat_red", "seat_yellow", "seat_blue")[index % 3]


def _add_seat_part(model: ArticulatedObject, index: int):
    seat = model.part(f"seat_{index}")
    seat.visual(
        Cylinder(radius=0.010, length=0.09),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="hanger_metal",
        name="hanger_pivot",
    )
    seat.visual(
        Box((0.014, 0.024, 0.16)),
        origin=Origin(xyz=(0.03, 0.0, -0.08)),
        material="hanger_metal",
        name="right_hanger",
    )
    seat.visual(
        Box((0.014, 0.024, 0.16)),
        origin=Origin(xyz=(-0.03, 0.0, -0.08)),
        material="hanger_metal",
        name="left_hanger",
    )
    seat.visual(
        Box((0.17, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.17)),
        material="seat_frame",
        name="seat_support",
    )
    seat.visual(
        Box((0.19, 0.12, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=_seat_color(index),
        name="seat_pan",
    )
    seat.visual(
        Box((0.18, 0.018, 0.12)),
        origin=Origin(xyz=(0.0, -0.051, -0.13)),
        material=_seat_color(index),
        name="seat_back",
    )
    seat.visual(
        Box((0.018, 0.10, 0.11)),
        origin=Origin(xyz=(0.087, 0.0, -0.185)),
        material=_seat_color(index),
        name="right_side_panel",
    )
    seat.visual(
        Box((0.018, 0.10, 0.11)),
        origin=Origin(xyz=(-0.087, 0.0, -0.185)),
        material=_seat_color(index),
        name="left_side_panel",
    )
    seat.visual(
        Cylinder(radius=0.006, length=0.156),
        origin=Origin(xyz=(0.0, 0.040, -0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material="hanger_metal",
        name="front_guard_bar",
    )
    seat.visual(
        Cylinder(radius=0.005, length=0.182),
        origin=Origin(xyz=(0.0, 0.040, -0.235), rpy=(0.0, pi / 2.0, 0.0)),
        material="hanger_metal",
        name="foot_bar",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.26, 0.18, 0.28)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
    )
    return seat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_observation_wheel")

    model.material("frame_paint", rgba=(0.80, 0.82, 0.86, 1.0))
    model.material("rim_white", rgba=(0.96, 0.97, 0.98, 1.0))
    model.material("arm_grey", rgba=(0.62, 0.65, 0.70, 1.0))
    model.material("axle_metal", rgba=(0.35, 0.37, 0.40, 1.0))
    model.material("hanger_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("seat_frame", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("seat_red", rgba=(0.82, 0.20, 0.19, 1.0))
    model.material("seat_yellow", rgba=(0.92, 0.76, 0.16, 1.0))
    model.material("seat_blue", rgba=(0.20, 0.47, 0.82, 1.0))

    frame = model.part("frame")
    _add_frame_visuals(frame, frame_paint="frame_paint", axle_metal="axle_metal")
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 1.80, 1.55)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    wheel = model.part("wheel")
    _add_wheel_visuals(wheel, rim_color="rim_white", arm_color="arm_grey")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.03, length=0.42),
        mass=120.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4),
    )

    for index in range(SEAT_COUNT):
        seat = _add_seat_part(model, index)
        angle = 2.0 * pi * index / SEAT_COUNT
        seat_y, seat_z = _ring_point(SEAT_PIVOT_RADIUS, angle)
        model.articulation(
            f"wheel_to_seat_{index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=seat,
            origin=Origin(xyz=(0.0, seat_y, seat_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=4.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("frame_to_wheel")
    seats = [object_model.get_part(f"seat_{index}") for index in range(SEAT_COUNT)]
    seat_joints = [object_model.get_articulation(f"wheel_to_seat_{index}") for index in range(SEAT_COUNT)]

    ctx.expect_contact(wheel, frame, name="wheel_hub_contacts_supports")
    ctx.check(
        "wheel_axis_is_horizontal",
        tuple(wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"Expected wheel axis (1,0,0), got {wheel_joint.axis}",
    )

    for index, (seat, seat_joint) in enumerate(zip(seats, seat_joints)):
        ctx.expect_contact(seat, wheel, name=f"seat_{index}_hung_from_wheel")
        ctx.expect_origin_distance(
            seat,
            wheel,
            axes="x",
            min_dist=0.0,
            max_dist=0.001,
            name=f"seat_{index}_centered_between_rims",
        )
        ctx.check(
            f"seat_{index}_pivot_axis_is_horizontal",
            tuple(seat_joint.axis) == (1.0, 0.0, 0.0),
            details=f"Expected seat axis (1,0,0), got {seat_joint.axis}",
        )

    seat_0 = seats[0]
    rest_pivot = ctx.part_world_position(seat_0)
    rest_aabb = ctx.part_world_aabb(seat_0)
    assert rest_pivot is not None
    assert rest_aabb is not None
    rest_dims = _aabb_dims(rest_aabb)

    with ctx.pose({wheel_joint: -pi / 3.0}):
        rotated_pivot = ctx.part_world_position(seat_0)
        assert rotated_pivot is not None
        ctx.check(
            "wheel_rotation_moves_top_seat_around_rim",
            rotated_pivot[1] > rest_pivot[1] + 0.70 and rotated_pivot[2] < rest_pivot[2] - 0.35,
            details=f"Seat pivot moved to {rotated_pivot} from {rest_pivot}",
        )

    with ctx.pose({wheel_joint: -pi / 3.0, seat_joints[0]: pi / 3.0}):
        corrected_aabb = ctx.part_world_aabb(seat_0)
        corrected_pos = ctx.part_world_position(seat_0)
        assert corrected_aabb is not None
        assert corrected_pos is not None
        corrected_dims = _aabb_dims(corrected_aabb)
        ctx.check(
            "seat_can_counter_rotate_to_stay_upright",
            abs(corrected_dims[1] - rest_dims[1]) < 0.03 and abs(corrected_dims[2] - rest_dims[2]) < 0.03,
            details=f"Rest dims {rest_dims}, corrected dims {corrected_dims}",
        )
        ctx.check(
            "seat_0_remains_hung_in_counter_rotated_pose",
            abs((corrected_pos[1] ** 2 + (corrected_pos[2] - AXLE_HEIGHT) ** 2) ** 0.5 - SEAT_PIVOT_RADIUS) < 0.01,
            details=f"Seat pivot radius drifted at counter-rotated pose: pos={corrected_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
