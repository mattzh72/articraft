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
    tube_from_spline_points,
    wire_from_points,
)


FRAME_HALF_WIDTH = 0.19
FRAME_MOUNT_X = 0.24
CLUSTER_RADIUS = 0.12
WHEEL_RADIUS = 0.09
WHEEL_WIDTH = 0.05
CLUSTER_HUB_Z = 0.21
CLUSTER_HUB_Y = -0.01


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _wheel_offsets() -> list[tuple[float, float]]:
    side = CLUSTER_RADIUS * 0.8660254
    return [
        (0.0, -CLUSTER_RADIUS),
        (side, CLUSTER_RADIUS * 0.5),
        (-side, CLUSTER_RADIUS * 0.5),
    ]


def _tire_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (WHEEL_RADIUS * 0.54, -half_width * 0.98),
        (WHEEL_RADIUS * 0.74, -half_width),
        (WHEEL_RADIUS * 0.90, -half_width * 0.84),
        (WHEEL_RADIUS * 0.98, -half_width * 0.46),
        (WHEEL_RADIUS, -half_width * 0.14),
        (WHEEL_RADIUS, half_width * 0.14),
        (WHEEL_RADIUS * 0.98, half_width * 0.46),
        (WHEEL_RADIUS * 0.90, half_width * 0.84),
        (WHEEL_RADIUS * 0.74, half_width),
        (WHEEL_RADIUS * 0.54, half_width * 0.98),
        (WHEEL_RADIUS * 0.44, half_width * 0.28),
        (WHEEL_RADIUS * 0.40, 0.0),
        (WHEEL_RADIUS * 0.44, -half_width * 0.28),
        (WHEEL_RADIUS * 0.54, -half_width * 0.98),
    ]
    return _mesh(
        "stair_truck_tire",
        LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0),
    )


def _add_wheel_visuals(part, *, side_sign: float, tire_mesh, rubber, wheel_steel, dark_steel) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        tire_mesh,
        origin=Origin(xyz=(side_sign * (WHEEL_WIDTH * 0.5), 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(side_sign * 0.018, 0.0, 0.0), rpy=spin_origin.rpy),
        material=wheel_steel,
        name="rim_inner",
    )
    part.visual(
        Cylinder(radius=0.044, length=0.020),
        origin=Origin(xyz=(side_sign * 0.027, 0.0, 0.0), rpy=spin_origin.rpy),
        material=wheel_steel,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(side_sign * 0.008, 0.0, 0.0), rpy=spin_origin.rpy),
        material=dark_steel,
        name="hub_cap",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(side_sign * 0.038, 0.0, 0.0), rpy=spin_origin.rpy),
        material=dark_steel,
        name="outer_cap",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=2.2,
        origin=Origin(xyz=(side_sign * (WHEEL_WIDTH * 0.5), 0.0, 0.0), rpy=spin_origin.rpy),
    )


def _add_cluster_visuals(part, *, mesh_prefix: str, side_sign: float, wheel_steel, dark_steel) -> None:
    offsets = _wheel_offsets()
    for index, (wheel_y, wheel_z) in enumerate(offsets):
        arm = tube_from_spline_points(
            [
                (side_sign * 0.015, 0.0, 0.0),
                (side_sign * 0.028, wheel_y * 0.68, wheel_z * 0.68),
                (side_sign * 0.032, wheel_y, wheel_z),
            ],
            radius=0.010,
            samples_per_segment=4,
            radial_segments=14,
        )
        part.visual(
            _mesh(f"{mesh_prefix}_arm_{index}", arm),
            material=wheel_steel,
            name=f"arm_{index}",
        )
        part.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(
                xyz=(side_sign * 0.038, wheel_y, wheel_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=wheel_steel,
            name=f"axle_pad_{index}",
        )

    triangle = wire_from_points(
        [
            (side_sign * 0.028, wheel_y * 0.68, wheel_z * 0.68)
            for wheel_y, wheel_z in offsets
        ],
        radius=0.008,
        radial_segments=14,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.018,
        cap_ends=False,
    )
    part.visual(
        _mesh(f"{mesh_prefix}_triangle", triangle),
        material=wheel_steel,
        name="triangle_brace",
    )
    part.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(side_sign * 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="inner_washer",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(side_sign * 0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_steel,
        name="hub_drum",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.08, 0.28, 0.28)),
        mass=3.0,
        origin=Origin(xyz=(side_sign * 0.020, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stair_climber_hand_truck")

    frame_red = model.material("frame_red", rgba=(0.72, 0.10, 0.09, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    grip_black = model.material("grip_black", rgba=(0.12, 0.12, 0.13, 1.0))

    tire_mesh = _tire_mesh()

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.50, 0.34, 1.26)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.01, 0.63)),
    )

    left_rail_points = [
        (FRAME_HALF_WIDTH, -0.01, 0.17),
        (FRAME_HALF_WIDTH, -0.02, 0.56),
        (0.18, -0.08, 1.02),
        (0.14, -0.12, 1.21),
    ]
    right_rail_points = _mirror_x(left_rail_points)

    frame.visual(
        _mesh(
            "frame_left_rail",
            tube_from_spline_points(left_rail_points, radius=0.018, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_red,
        name="left_rail",
    )
    frame.visual(
        _mesh(
            "frame_right_rail",
            tube_from_spline_points(right_rail_points, radius=0.018, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_red,
        name="right_rail",
    )
    frame.visual(
        _mesh(
            "frame_top_handle",
            tube_from_spline_points(
                [
                    (-0.14, -0.12, 1.21),
                    (-0.07, -0.13, 1.24),
                    (0.07, -0.13, 1.24),
                    (0.14, -0.12, 1.21),
                ],
                radius=0.018,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_red,
        name="top_handle",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(0.09, -0.13, 1.24), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(-0.09, -0.13, 1.24), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.38),
        origin=Origin(xyz=(0.0, -0.07, 0.94), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_red,
        name="upper_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.38),
        origin=Origin(xyz=(0.0, -0.03, 0.63), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_red,
        name="mid_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.36),
        origin=Origin(xyz=(0.0, -0.02, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_red,
        name="lower_crossbar",
    )
    frame.visual(
        _mesh(
            "frame_left_toe_support",
            tube_from_spline_points(
                [(0.18, -0.01, 0.17), (0.17, 0.00, 0.11), (0.15, 0.02, 0.031)],
                radius=0.015,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_red,
        name="left_toe_support",
    )
    frame.visual(
        _mesh(
            "frame_right_toe_support",
            tube_from_spline_points(
                [(-0.18, -0.01, 0.17), (-0.17, 0.00, 0.11), (-0.15, 0.02, 0.031)],
                radius=0.015,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_red,
        name="right_toe_support",
    )
    frame.visual(
        Box((0.36, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.11, 0.031)),
        material=frame_red,
        name="toe_plate",
    )
    frame.visual(
        Box((0.36, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.006, 0.051)),
        material=frame_red,
        name="toe_plate_lip",
    )
    frame.visual(
        Box((0.042, 0.080, 0.120)),
        origin=Origin(xyz=(0.219, CLUSTER_HUB_Y, CLUSTER_HUB_Z)),
        material=dark_steel,
        name="left_mount",
    )
    frame.visual(
        Box((0.042, 0.080, 0.120)),
        origin=Origin(xyz=(-0.219, CLUSTER_HUB_Y, CLUSTER_HUB_Z)),
        material=dark_steel,
        name="right_mount",
    )

    left_cluster = model.part("left_cluster_carrier")
    _add_cluster_visuals(
        left_cluster,
        mesh_prefix="left_cluster",
        side_sign=1.0,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
    )

    right_cluster = model.part("right_cluster_carrier")
    _add_cluster_visuals(
        right_cluster,
        mesh_prefix="right_cluster",
        side_sign=-1.0,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
    )

    left_wheels = []
    right_wheels = []
    for side_name, side_sign, collector in (
        ("left", 1.0, left_wheels),
        ("right", -1.0, right_wheels),
    ):
        for index in range(3):
            wheel = model.part(f"{side_name}_cluster_wheel_{index}")
            _add_wheel_visuals(
                wheel,
                side_sign=side_sign,
                tire_mesh=tire_mesh,
                rubber=rubber,
                wheel_steel=wheel_steel,
                dark_steel=dark_steel,
            )
            collector.append(wheel)

    model.articulation(
        "left_cluster_rotate",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_cluster,
        origin=Origin(xyz=(FRAME_MOUNT_X, CLUSTER_HUB_Y, CLUSTER_HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=10.0),
    )
    model.articulation(
        "right_cluster_rotate",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_cluster,
        origin=Origin(xyz=(-FRAME_MOUNT_X, CLUSTER_HUB_Y, CLUSTER_HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=10.0),
    )

    for side_name, side_sign, cluster_part, wheels in (
        ("left", 1.0, left_cluster, left_wheels),
        ("right", -1.0, right_cluster, right_wheels),
    ):
        for index, ((wheel_y, wheel_z), wheel_part) in enumerate(zip(_wheel_offsets(), wheels)):
            model.articulation(
                f"{side_name}_cluster_wheel_{index}_spin",
                ArticulationType.CONTINUOUS,
                parent=cluster_part,
                child=wheel_part,
                origin=Origin(xyz=(side_sign * 0.044, wheel_y, wheel_z)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=20.0, velocity=25.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_cluster = object_model.get_part("left_cluster_carrier")
    right_cluster = object_model.get_part("right_cluster_carrier")
    left_wheels = [object_model.get_part(f"left_cluster_wheel_{index}") for index in range(3)]
    right_wheels = [object_model.get_part(f"right_cluster_wheel_{index}") for index in range(3)]

    left_cluster_rotate = object_model.get_articulation("left_cluster_rotate")
    right_cluster_rotate = object_model.get_articulation("right_cluster_rotate")
    left_wheel_0_spin = object_model.get_articulation("left_cluster_wheel_0_spin")
    right_wheel_1_spin = object_model.get_articulation("right_cluster_wheel_1_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.expect_contact(left_cluster, frame, name="left_cluster_contacts_frame")
    ctx.expect_contact(right_cluster, frame, name="right_cluster_contacts_frame")
    ctx.expect_gap(left_cluster, frame, axis="x", max_gap=0.001, max_penetration=0.0, name="left_cluster_seats_on_mount")
    ctx.expect_gap(frame, right_cluster, axis="x", max_gap=0.001, max_penetration=0.0, name="right_cluster_seats_on_mount")
    ctx.expect_origin_distance(
        left_cluster,
        right_cluster,
        axes="x",
        min_dist=0.46,
        max_dist=0.50,
        name="cluster_spacing_matches_frame_width",
    )

    for side_name, cluster_part, wheels in (
        ("left", left_cluster, left_wheels),
        ("right", right_cluster, right_wheels),
    ):
        for index, wheel in enumerate(wheels):
            ctx.expect_contact(
                wheel,
                cluster_part,
                name=f"{side_name}_wheel_{index}_contacts_cluster",
            )

    frame_aabb = ctx.part_world_aabb(frame)
    toe_aabb = ctx.part_element_world_aabb(frame, elem="toe_plate")
    left_rail_aabb = ctx.part_element_world_aabb(frame, elem="left_rail")

    frame_height_ok = False
    frame_width_ok = False
    toe_plate_ok = False
    height_detail = "frame AABB unavailable"
    width_detail = "frame AABB unavailable"
    toe_detail = "toe plate or rail AABB unavailable"
    if frame_aabb is not None:
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        frame_height_ok = 1.18 <= frame_height <= 1.30
        frame_width_ok = 0.38 <= frame_width <= 0.50
        height_detail = f"frame height={frame_height:.3f} m"
        width_detail = f"frame width={frame_width:.3f} m"
    if toe_aabb is not None and left_rail_aabb is not None:
        toe_plate_ok = (
            toe_aabb[1][1] >= 0.21
            and toe_aabb[0][2] <= 0.03
            and toe_aabb[1][2] <= 0.05
            and toe_aabb[0][1] > left_rail_aabb[0][1]
        )
        toe_detail = (
            f"toe max_y={toe_aabb[1][1]:.3f}, toe z=({toe_aabb[0][2]:.3f},{toe_aabb[1][2]:.3f}), "
            f"rail min_y={left_rail_aabb[0][1]:.3f}"
        )
    ctx.check("frame_realistic_height", frame_height_ok, height_detail)
    ctx.check("frame_realistic_width", frame_width_ok, width_detail)
    ctx.check("toe_plate_low_and_forward", toe_plate_ok, toe_detail)

    left_rest_positions = [ctx.part_world_position(wheel) for wheel in left_wheels]
    right_rest_positions = [ctx.part_world_position(wheel) for wheel in right_wheels]
    left_rest_ok = all(pos is not None for pos in left_rest_positions)
    right_rest_ok = all(pos is not None for pos in right_rest_positions)
    ctx.check(
        "left_cluster_has_bottom_wheel_at_rest",
        left_rest_ok
        and left_rest_positions[0][2] < left_rest_positions[1][2] - 0.08
        and left_rest_positions[0][2] < left_rest_positions[2][2] - 0.08,
        "left wheel cluster should present one lowest wheel in the default pose",
    )
    ctx.check(
        "right_cluster_has_bottom_wheel_at_rest",
        right_rest_ok
        and right_rest_positions[0][2] < right_rest_positions[1][2] - 0.08
        and right_rest_positions[0][2] < right_rest_positions[2][2] - 0.08,
        "right wheel cluster should present one lowest wheel in the default pose",
    )

    with ctx.pose({left_cluster_rotate: 2.0 * math.pi / 3.0}):
        ctx.expect_contact(left_cluster, frame, name="left_cluster_rotated_stays_mounted")
        for index, wheel in enumerate(left_wheels):
            ctx.expect_contact(wheel, left_cluster, name=f"left_rotated_wheel_{index}_still_contacts_cluster")
        ctx.fail_if_parts_overlap_in_current_pose(name="left_cluster_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="left_cluster_rotated_no_floating")

        rotated_positions = [ctx.part_world_position(wheel) for wheel in left_wheels]
        rotated_ok = all(pos is not None for pos in rotated_positions) and left_rest_positions[0] is not None
        ctx.check(
            "left_cluster_rotation_reindexes_lowest_wheel",
            rotated_ok
            and rotated_positions[2][2] < rotated_positions[0][2] - 0.06
            and rotated_positions[2][2] < rotated_positions[1][2] - 0.06
            and abs(rotated_positions[0][0] - left_rest_positions[0][0]) <= 0.002
            and abs(rotated_positions[0][2] - left_rest_positions[0][2]) >= 0.10,
            "cluster rotation should move the wheel triangle in the YZ plane while staying on the same axle line",
        )

    with ctx.pose({left_wheel_0_spin: 1.4}):
        spun_position = ctx.part_world_position(left_wheels[0])
        spin_ok = left_rest_positions[0] is not None and spun_position is not None and all(
            abs(spun_position[axis] - left_rest_positions[0][axis]) <= 1e-6
            for axis in range(3)
        )
        ctx.check(
            "wheel_spin_keeps_axle_center_fixed",
            spin_ok,
            "a wheel should spin in place about its axle without translating",
        )
        ctx.expect_contact(left_wheels[0], left_cluster, name="spun_left_wheel_stays_on_cluster")

    with ctx.pose(
        {
            left_cluster_rotate: 2.0 * math.pi / 3.0,
            right_cluster_rotate: -2.0 * math.pi / 3.0,
            left_wheel_0_spin: 1.8,
            right_wheel_1_spin: -1.3,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.expect_contact(left_cluster, frame, name="combined_pose_left_cluster_contact")
        ctx.expect_contact(right_cluster, frame, name="combined_pose_right_cluster_contact")
        ctx.expect_contact(right_wheels[1], right_cluster, name="combined_pose_right_wheel_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
