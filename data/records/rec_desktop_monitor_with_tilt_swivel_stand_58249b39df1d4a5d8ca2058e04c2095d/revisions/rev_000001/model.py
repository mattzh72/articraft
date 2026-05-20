from __future__ import annotations

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
    rounded_rect_profile,
    section_loft,
)


def _rpy_for_z_axis(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-9:
        raise ValueError("Vector must be non-zero.")
    nx, ny, nz = x / length, y / length, z / length
    pitch = math.acos(max(-1.0, min(1.0, nz)))
    yaw = math.atan2(ny, nx)
    return (0.0, pitch, yaw)


def _add_cylinder_segment(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    center = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=_rpy_for_z_axis((vx, vy, vz))),
        material=material,
        name=name,
    )


def _screen_section(
    y: float,
    *,
    half_width: float,
    height: float,
    front_edge: float,
    front_center: float,
    depth_edge: float,
    depth_center: float,
    inset: float = 0.0,
) -> list[tuple[float, float, float]]:
    t = abs(y) / half_width if half_width > 1e-9 else 0.0
    curve = max(0.0, 1.0 - t * t)
    front_x = -(front_edge + (front_center - front_edge) * curve) + inset
    depth = depth_edge + (depth_center - depth_edge) * curve - 2.0 * inset
    section_height = height - 2.0 * inset
    radius = min(0.018, 0.4 * depth, 0.18 * section_height)
    profile = rounded_rect_profile(depth, section_height, radius, corner_segments=8)
    center_x = front_x + 0.5 * depth
    return [(center_x + x, y, z) for x, z in profile]


def _build_screen_shell():
    half_width = 0.41
    sample_ys = (-half_width, -0.28, -0.14, 0.0, 0.14, 0.28, half_width)
    return section_loft(
        [
            _screen_section(
                y,
                half_width=half_width,
                height=0.372,
                front_edge=0.055,
                front_center=0.084,
                depth_edge=0.050,
                depth_center=0.068,
            )
            for y in sample_ys
        ]
    )


def _build_glass_panel():
    half_width = 0.385
    sample_ys = (-half_width, -0.255, -0.127, 0.0, 0.127, 0.255, half_width)
    return section_loft(
        [
            _screen_section(
                y,
                half_width=half_width,
                height=0.334,
                front_edge=0.050,
                front_center=0.077,
                depth_edge=0.0054,
                depth_center=0.0060,
                inset=0.0,
            )
            for y in sample_ys
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.03, 0.04, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.060, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=graphite,
        name="hub_disk",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=matte_black,
        name="swivel_pedestal",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=graphite,
        name="top_collar",
    )

    leg_specs = (
        ("front_leg", math.pi, 0.342),
        ("rear_leg_0", math.radians(52.0), 0.328),
        ("rear_leg_1", math.radians(-52.0), 0.328),
    )
    for leg_name, angle, radius in leg_specs:
        c = math.cos(angle)
        s = math.sin(angle)
        start = (0.040 * c, 0.040 * s, 0.044)
        end = (radius * c, radius * s, 0.014)
        _add_cylinder_segment(
            base,
            start=start,
            end=end,
            radius=0.0125,
            material=graphite,
            name=leg_name,
        )
        base.visual(
            Box((0.048, 0.034, 0.010)),
            origin=Origin(
                xyz=(radius * c, radius * s, 0.005),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber,
            name=f"{leg_name}_foot",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 0.09)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=graphite,
        name="swivel_socket",
    )
    yoke.visual(
        Box((0.050, 0.066, 0.230)),
        origin=Origin(xyz=(0.010, 0.0, 0.167)),
        material=matte_black,
        name="spine",
    )
    yoke.visual(
        Box((0.042, 0.220, 0.040)),
        origin=Origin(xyz=(-0.004, 0.0, 0.236)),
        material=matte_black,
        name="shoulder_block",
    )
    yoke.visual(
        Cylinder(radius=0.010, length=0.660),
        origin=Origin(
            xyz=(-0.048, 0.0, 0.300),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="tilt_crossbar",
    )
    yoke.visual(
        Box((0.072, 0.052, 0.030)),
        origin=Origin(xyz=(-0.036, 0.0, 0.270)),
        material=graphite,
        name="tilt_web",
    )

    arm_start_x = -0.008
    arm_start_z = 0.240
    arm_end_x = -0.044
    arm_end_z = 0.300
    arm_start_y = 0.088
    arm_end_y = 0.330
    _add_cylinder_segment(
        yoke,
        start=(arm_start_x, arm_start_y, arm_start_z),
        end=(arm_end_x, arm_end_y, arm_end_z),
        radius=0.013,
        material=graphite,
        name="arm_0",
    )
    _add_cylinder_segment(
        yoke,
        start=(arm_start_x, -arm_start_y, arm_start_z),
        end=(arm_end_x, -arm_end_y, arm_end_z),
        radius=0.013,
        material=graphite,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(
            xyz=(-0.046, arm_end_y, 0.300),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="pivot_hub_0",
    )
    yoke.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(
            xyz=(-0.046, -arm_end_y, 0.300),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="pivot_hub_1",
    )
    hinge_axis_x = 0.0365
    hinge_axis_y = 0.019
    for index, z in enumerate((0.094, 0.155, 0.216)):
        yoke.visual(
            Cylinder(radius=0.0042, length=0.020),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, z)),
            material=dark_gray,
            name=f"hinge_barrel_{index}",
        )

    yoke.inertial = Inertial.from_geometry(
        Box((0.20, 0.66, 0.34)),
        mass=2.4,
        origin=Origin(xyz=(-0.020, 0.0, 0.175)),
    )

    screen = model.part("screen")
    screen.visual(
        mesh_from_geometry(_build_screen_shell(), "monitor_shell"),
        material=matte_black,
        name="shell",
    )
    screen.visual(
        mesh_from_geometry(_build_glass_panel(), "monitor_glass"),
        material=screen_black,
        name="glass",
    )
    screen.visual(
        Box((0.060, 0.220, 0.132)),
        origin=Origin(xyz=(-0.052, 0.0, 0.004)),
        material=graphite,
        name="rear_core",
    )
    screen.visual(
        Box((0.024, 0.760, 0.044)),
        origin=Origin(xyz=(-0.022, 0.0, -0.012)),
        material=graphite,
        name="mount_web",
    )
    screen.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(
            xyz=(0.000, 0.368, -0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="trunnion_0",
    )
    screen.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(
            xyz=(0.000, -0.368, -0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="trunnion_1",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.09, 0.84, 0.39)),
        mass=4.9,
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        Box((0.0045, 0.038, 0.148)),
        origin=Origin(xyz=(0.00225, -0.019, 0.0)),
        material=graphite,
        name="panel",
    )
    cable_door.visual(
        Box((0.004, 0.006, 0.026)),
        origin=Origin(xyz=(0.0035, -0.036, 0.0)),
        material=dark_gray,
        name="pull_tab",
    )
    for index, z in enumerate((-0.026, 0.026)):
        cable_door.visual(
            Cylinder(radius=0.0040, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_gray,
            name=f"knuckle_{index}",
        )
    cable_door.inertial = Inertial.from_geometry(
        Box((0.014, 0.042, 0.150)),
        mass=0.12,
        origin=Origin(xyz=(0.004, -0.019, 0.0)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "yoke_to_screen",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=screen,
        origin=Origin(xyz=(-0.062, 0.0, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.30,
            upper=0.42,
        ),
    )
    model.articulation(
        "yoke_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=cable_door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    screen = object_model.get_part("screen")
    cable_door = object_model.get_part("cable_door")

    swivel = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_screen")
    door_hinge = object_model.get_articulation("yoke_to_cable_door")

    ctx.expect_gap(
        screen,
        base,
        axis="z",
        min_gap=0.100,
        name="display clears the tripod base",
    )
    ctx.expect_gap(
        cable_door,
        yoke,
        axis="x",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="panel",
        negative_elem="spine",
        name="cable door sits on the spine rear face",
    )
    ctx.expect_overlap(
        cable_door,
        yoke,
        axes="yz",
        min_overlap=0.035,
        elem_a="panel",
        elem_b="spine",
        name="cable door covers the stand spine channel",
    )

    rest_screen_pos = ctx.part_world_position(screen)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "swivel rotates the display around the stand axis",
        rest_screen_pos is not None
        and turned_screen_pos is not None
        and abs(turned_screen_pos[1]) > 0.045
        and abs(turned_screen_pos[0]) < abs(rest_screen_pos[0]),
        details=f"rest={rest_screen_pos}, turned={turned_screen_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(screen, elem="shell")
    with ctx.pose({tilt: 0.36}):
        tilted_shell_aabb = ctx.part_element_world_aabb(screen, elem="shell")
    ctx.check(
        "positive tilt moves the top of the screen backward",
        rest_shell_aabb is not None
        and tilted_shell_aabb is not None
        and tilted_shell_aabb[1][0] > rest_shell_aabb[1][0] + 0.025,
        details=f"rest={rest_shell_aabb}, tilted={tilted_shell_aabb}",
    )

    rest_door_aabb = ctx.part_element_world_aabb(cable_door, elem="panel")
    with ctx.pose({door_hinge: 1.10}):
        open_door_aabb = ctx.part_element_world_aabb(cable_door, elem="panel")
    ctx.check(
        "cable door opens away from the spine",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > rest_door_aabb[1][0] + 0.012,
        details=f"closed={rest_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
