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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _wheel_visuals(part, *, rubber, steel, dark_steel) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    side_disc = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    part.visual(
        Cylinder(radius=0.085, length=0.042),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.034),
        origin=spin_origin,
        material=steel,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.038),
        origin=spin_origin,
        material=dark_steel,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=side_disc.rpy),
        material=steel,
        name="outer_flange",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=side_disc.rpy),
        material=steel,
        name="inner_flange",
    )


def _build_bogie(part, *, side_sign: float, steel, dark_steel) -> None:
    hub_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.048, length=0.050),
        origin=hub_axis,
        material=dark_steel,
        name="hub_drum",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.030 * side_sign, 0.0), rpy=hub_axis.rpy),
        material=steel,
        name="hub_cap",
    )

    arm_radius = 0.110
    axle_y = 0.010 * side_sign
    boss_y = 0.032 * side_sign
    boss_axis = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    for angle_deg, stem_name, boss_name in (
        (-90.0, "lower_arm", "lower_boss"),
        (30.0, "front_arm", "front_boss"),
        (150.0, "rear_arm", "rear_boss"),
    ):
        angle = math.radians(angle_deg)
        axle_point = (arm_radius * math.cos(angle), axle_y, arm_radius * math.sin(angle))
        _add_member(part, (0.0, 0.0, 0.0), axle_point, radius=0.016, material=steel, name=stem_name)
        part.visual(
            Cylinder(radius=0.024, length=0.016),
            origin=Origin(xyz=(axle_point[0], boss_y, axle_point[2]), rpy=boss_axis.rpy),
            material=dark_steel,
            name=boss_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stair_climbing_hand_truck")

    frame_blue = model.material("frame_blue", rgba=(0.14, 0.28, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.40, 1.20)),
        mass=13.0,
        origin=Origin(xyz=(0.05, 0.0, 0.60)),
    )

    rail_height = 0.95
    rail_center_z = 0.635
    rail_center_x = -0.015
    rail_offset_y = 0.170

    frame.visual(
        Box((0.045, 0.032, rail_height)),
        origin=Origin(xyz=(rail_center_x, rail_offset_y, rail_center_z)),
        material=frame_blue,
        name="left_rail",
    )
    frame.visual(
        Box((0.045, 0.032, rail_height)),
        origin=Origin(xyz=(rail_center_x, -rail_offset_y, rail_center_z)),
        material=frame_blue,
        name="right_rail",
    )
    frame.visual(
        Box((0.050, 0.340, 0.050)),
        origin=Origin(xyz=(-0.005, 0.0, 0.135)),
        material=frame_blue,
        name="lower_crossmember",
    )
    frame.visual(
        Box((0.038, 0.340, 0.045)),
        origin=Origin(xyz=(-0.010, 0.0, 0.520)),
        material=frame_blue,
        name="mid_crossmember",
    )
    frame.visual(
        Box((0.030, 0.280, 0.110)),
        origin=Origin(xyz=(0.015, 0.0, 0.055)),
        material=frame_blue,
        name="toe_web",
    )
    frame.visual(
        Box((0.200, 0.280, 0.012)),
        origin=Origin(xyz=(0.130, 0.0, 0.011)),
        material=frame_blue,
        name="toe_plate",
    )
    frame.visual(
        Box((0.008, 0.250, 0.420)),
        origin=Origin(xyz=(0.004, 0.0, 0.360)),
        material=frame_blue,
        name="back_panel",
    )
    frame.visual(
        Box((0.035, 0.050, 0.150)),
        origin=Origin(xyz=(-0.015, 0.165, 0.185)),
        material=frame_blue,
        name="left_bogie_mount",
    )
    frame.visual(
        Box((0.035, 0.050, 0.150)),
        origin=Origin(xyz=(-0.015, -0.165, 0.185)),
        material=frame_blue,
        name="right_bogie_mount",
    )
    frame.visual(
        Box((0.022, 0.260, 0.026)),
        origin=Origin(xyz=(-0.010, 0.0, 0.255)),
        material=frame_blue,
        name="brace_bar",
    )
    _add_member(
        frame,
        (0.005, 0.145, 0.255),
        (0.010, 0.085, 0.135),
        radius=0.010,
        material=dark_steel,
        name="left_lower_brace",
    )
    _add_member(
        frame,
        (0.005, -0.145, 0.255),
        (0.010, -0.085, 0.135),
        radius=0.010,
        material=dark_steel,
        name="right_lower_brace",
    )

    handle_geom = tube_from_spline_points(
        [
            (-0.015, -0.170, 1.110),
            (-0.035, -0.165, 1.155),
            (-0.055, 0.000, 1.205),
            (-0.035, 0.165, 1.155),
            (-0.015, 0.170, 1.110),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(
        mesh_from_geometry(handle_geom, "hand_truck_handle"),
        material=frame_blue,
        name="handle_loop",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(-0.040, 0.125, 1.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(-0.040, -0.125, 1.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    left_bogie = model.part("left_bogie")
    left_bogie.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.28)),
        mass=3.4,
        origin=Origin(),
    )
    _build_bogie(left_bogie, side_sign=1.0, steel=steel, dark_steel=dark_steel)

    right_bogie = model.part("right_bogie")
    right_bogie.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.28)),
        mass=3.4,
        origin=Origin(),
    )
    _build_bogie(right_bogie, side_sign=-1.0, steel=steel, dark_steel=dark_steel)

    wheel_names = (
        "left_lower_wheel",
        "left_front_wheel",
        "left_rear_wheel",
        "right_lower_wheel",
        "right_front_wheel",
        "right_rear_wheel",
    )
    for wheel_name in wheel_names:
        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.085, length=0.042),
            mass=1.1,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        _wheel_visuals(wheel, rubber=rubber, steel=steel, dark_steel=dark_steel)

    model.articulation(
        "frame_to_left_bogie",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_bogie,
        origin=Origin(xyz=(-0.015, 0.215, 0.195)),
        # Negative Y makes positive motion rotate the lower wheel forward/upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=4.0),
    )
    model.articulation(
        "frame_to_right_bogie",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_bogie,
        origin=Origin(xyz=(-0.015, -0.215, 0.195)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=4.0),
    )

    left_wheel_y = 0.059
    right_wheel_y = -0.059
    arm_radius = 0.110
    front_x = arm_radius * math.cos(math.radians(30.0))
    front_z = arm_radius * math.sin(math.radians(30.0))
    rear_x = arm_radius * math.cos(math.radians(150.0))
    rear_z = arm_radius * math.sin(math.radians(150.0))

    for bogie_name, side_y, lower_name, front_name, rear_name in (
        ("left_bogie", left_wheel_y, "left_lower_wheel", "left_front_wheel", "left_rear_wheel"),
        ("right_bogie", right_wheel_y, "right_lower_wheel", "right_front_wheel", "right_rear_wheel"),
    ):
        bogie = model.get_part(bogie_name)
        model.articulation(
            f"{bogie_name}_to_lower_wheel",
            ArticulationType.CONTINUOUS,
            parent=bogie,
            child=lower_name,
            origin=Origin(xyz=(0.0, side_y, -arm_radius)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=25.0),
        )
        model.articulation(
            f"{bogie_name}_to_front_wheel",
            ArticulationType.CONTINUOUS,
            parent=bogie,
            child=front_name,
            origin=Origin(xyz=(front_x, side_y, front_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=25.0),
        )
        model.articulation(
            f"{bogie_name}_to_rear_wheel",
            ArticulationType.CONTINUOUS,
            parent=bogie,
            child=rear_name,
            origin=Origin(xyz=(rear_x, side_y, rear_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_bogie = object_model.get_part("left_bogie")
    right_bogie = object_model.get_part("right_bogie")
    left_lower_wheel = object_model.get_part("left_lower_wheel")
    right_lower_wheel = object_model.get_part("right_lower_wheel")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")

    left_bogie_joint = object_model.get_articulation("frame_to_left_bogie")
    right_bogie_joint = object_model.get_articulation("frame_to_right_bogie")
    wheel_joints = [
        object_model.get_articulation("left_bogie_to_lower_wheel"),
        object_model.get_articulation("left_bogie_to_front_wheel"),
        object_model.get_articulation("left_bogie_to_rear_wheel"),
        object_model.get_articulation("right_bogie_to_lower_wheel"),
        object_model.get_articulation("right_bogie_to_front_wheel"),
        object_model.get_articulation("right_bogie_to_rear_wheel"),
    ]

    ctx.expect_contact(
        left_bogie,
        frame,
        contact_tol=0.002,
        name="left bogie hub seats against frame mount",
    )
    ctx.expect_contact(
        right_bogie,
        frame,
        contact_tol=0.002,
        name="right bogie hub seats against frame mount",
    )
    ctx.expect_contact(
        left_lower_wheel,
        left_bogie,
        contact_tol=0.002,
        name="left lower wheel seats on its axle boss",
    )
    ctx.expect_contact(
        right_lower_wheel,
        right_bogie,
        contact_tol=0.002,
        name="right lower wheel seats on its axle boss",
    )
    ctx.expect_origin_gap(
        left_bogie,
        right_bogie,
        axis="y",
        min_gap=0.40,
        max_gap=0.46,
        name="bogie hubs are mounted on opposite sides of the frame",
    )

    toe_aabb = ctx.part_element_world_aabb(frame, elem="toe_plate")
    left_hub_pos = ctx.part_world_position(left_bogie)
    right_hub_pos = ctx.part_world_position(right_bogie)
    ctx.check(
        "toe plate projects ahead of both bogie hubs",
        toe_aabb is not None
        and left_hub_pos is not None
        and right_hub_pos is not None
        and toe_aabb[1][0] > left_hub_pos[0] + 0.20
        and toe_aabb[1][0] > right_hub_pos[0] + 0.20,
        details=f"toe_aabb={toe_aabb}, left_hub={left_hub_pos}, right_hub={right_hub_pos}",
    )

    ctx.check(
        "bogie joints rotate about lateral axle axis",
        left_bogie_joint.axis == (0.0, -1.0, 0.0) and right_bogie_joint.axis == (0.0, -1.0, 0.0),
        details=f"left_axis={left_bogie_joint.axis}, right_axis={right_bogie_joint.axis}",
    )
    ctx.check(
        "all wheel joints spin on axle axis",
        all(joint.axis == (0.0, 1.0, 0.0) for joint in wheel_joints),
        details=", ".join(f"{joint.name}:{joint.axis}" for joint in wheel_joints),
    )

    rest_lower_pos = ctx.part_world_position(left_lower_wheel)
    rest_front_pos = ctx.part_world_position(left_front_wheel)
    with ctx.pose({left_bogie_joint: math.pi / 3.0, right_bogie_joint: math.pi / 3.0}):
        rotated_lower_pos = ctx.part_world_position(left_lower_wheel)
        rotated_front_pos = ctx.part_world_position(left_front_wheel)
        ctx.check(
            "bogie rotation advances the formerly low wheel forward",
            rest_lower_pos is not None
            and rotated_lower_pos is not None
            and rotated_lower_pos[0] > rest_lower_pos[0] + 0.08
            and rotated_lower_pos[2] > rest_lower_pos[2] + 0.04,
            details=f"rest_lower={rest_lower_pos}, rotated_lower={rotated_lower_pos}",
        )
        ctx.check(
            "bogie rotation moves the front wheel upward and rearward around the hub",
            rest_front_pos is not None
            and rotated_front_pos is not None
            and rotated_front_pos[0] < rest_front_pos[0] - 0.03
            and rotated_front_pos[2] > rest_front_pos[2] + 0.03,
            details=f"rest_front={rest_front_pos}, rotated_front={rotated_front_pos}",
        )
        ctx.expect_origin_gap(
            left_front_wheel,
            right_front_wheel,
            axis="y",
            min_gap=0.45,
            name="front wheels remain separated across the frame when bogies rotate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
