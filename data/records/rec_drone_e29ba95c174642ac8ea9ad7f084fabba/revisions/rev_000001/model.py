from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, radians, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _thruster_orientation(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    if abs(axis[0]) > 0.5:
        return (0.0, pi / 2.0, 0.0)
    return (pi / 2.0, 0.0, 0.0)


def _axis_basis(
    axis: tuple[float, float, float]
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    if abs(axis[0]) > 0.5:
        sign = 1.0 if axis[0] >= 0.0 else -1.0
        return (sign, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    sign = 1.0 if axis[1] >= 0.0 else -1.0
    return (0.0, sign, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)


def _point_from_basis(
    center: tuple[float, float, float],
    axial: tuple[float, float, float],
    radial_u: tuple[float, float, float],
    radial_v: tuple[float, float, float],
    along: float = 0.0,
    u: float = 0.0,
    v: float = 0.0,
) -> tuple[float, float, float]:
    return (
        center[0] + axial[0] * along + radial_u[0] * u + radial_v[0] * v,
        center[1] + axial[1] * along + radial_u[1] * u + radial_v[1] * v,
        center[2] + axial[2] * along + radial_u[2] * u + radial_v[2] * v,
    )


def _thruster_duct_mesh(name: str):
    outer_profile = [
        (0.055, -0.036),
        (0.061, -0.028),
        (0.064, -0.018),
        (0.064, 0.018),
        (0.061, 0.028),
        (0.055, 0.036),
    ]
    inner_profile = [
        (0.051, -0.030),
        (0.053, -0.018),
        (0.053, 0.018),
        (0.051, 0.030),
    ]
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )


def _add_thruster_assembly(
    part,
    *,
    duct_mesh_name: str,
    duct_name: str,
    motor_name: str,
    mount_name: str,
    center: tuple[float, float, float],
    axis: tuple[float, float, float],
    frame_anchor: tuple[float, float, float],
    frame_material,
    duct_material,
    motor_material,
) -> None:
    axial, radial_u, radial_v = _axis_basis(axis)
    orientation = _thruster_orientation(axis)

    part.visual(
        _thruster_duct_mesh(duct_mesh_name),
        origin=Origin(xyz=center, rpy=orientation),
        material=duct_material,
        name=duct_name,
    )

    motor_center = _point_from_basis(center, axial, radial_u, radial_v, along=-0.024)
    part.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=motor_center, rpy=orientation),
        material=motor_material,
        name=motor_name,
    )

    _add_member(part, frame_anchor, motor_center, 0.010, frame_material, name=mount_name)

    for angle_deg in (30.0, 150.0, 270.0):
        angle = radians(angle_deg)
        ring_point = _point_from_basis(
            center,
            axial,
            radial_u,
            radial_v,
            along=-0.020,
            u=cos(angle) * 0.053,
            v=sin(angle) * 0.053,
        )
        _add_member(part, motor_center, ring_point, 0.0045, motor_material)


def _add_propeller_visuals(part, *, axis_name: str, prop_material, hub_material) -> None:
    if axis_name == "x":
        spin_rpy = (0.0, pi / 2.0, 0.0)
        part.visual(
            Cylinder(radius=0.014, length=0.016),
            origin=Origin(rpy=spin_rpy),
            material=hub_material,
            name="rotor_hub",
        )
        part.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(rpy=spin_rpy),
            material=hub_material,
            name="hub_spindle",
        )
        for blade_index in range(3):
            angle = 2.0 * pi * blade_index / 3.0
            part.visual(
                Box((0.012, 0.040, 0.004)),
                origin=Origin(
                    xyz=(0.002, cos(angle) * 0.016, sin(angle) * 0.016),
                    rpy=(angle, 0.16, 0.0),
                ),
                material=prop_material,
            )
    else:
        spin_rpy = (pi / 2.0, 0.0, 0.0)
        part.visual(
            Cylinder(radius=0.014, length=0.016),
            origin=Origin(rpy=spin_rpy),
            material=hub_material,
            name="rotor_hub",
        )
        part.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(rpy=spin_rpy),
            material=hub_material,
            name="hub_spindle",
        )
        for blade_index in range(3):
            angle = 2.0 * pi * blade_index / 3.0
            part.visual(
                Box((0.040, 0.012, 0.004)),
                origin=Origin(
                    xyz=(cos(angle) * 0.016, 0.002, sin(angle) * 0.016),
                    rpy=(0.08, angle, 0.0),
                ),
                material=prop_material,
            )


def _glass_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underwater_rov_drone")

    frame_yellow = model.material("frame_yellow", rgba=(0.86, 0.74, 0.13, 1.0))
    foam_yellow = model.material("foam_yellow", rgba=(0.95, 0.86, 0.21, 1.0))
    hull_teal = model.material("hull_teal", rgba=(0.12, 0.28, 0.31, 1.0))
    duct_grey = model.material("duct_grey", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.28, 1.0))
    prop_black = model.material("prop_black", rgba=(0.05, 0.05, 0.06, 1.0))
    camera_glass = model.material("camera_glass", rgba=(0.45, 0.80, 0.92, 0.45))
    silver = model.material("silver", rgba=(0.70, 0.72, 0.74, 1.0))

    frame = model.part("frame_body")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.56, 0.42)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    top_z = 0.110
    bottom_z = -0.090
    half_x = 0.230
    half_y = 0.160
    tube_radius = 0.012

    top_corners = [
        (half_x, half_y, top_z),
        (half_x, -half_y, top_z),
        (-half_x, -half_y, top_z),
        (-half_x, half_y, top_z),
    ]
    bottom_corners = [
        (half_x, half_y, bottom_z),
        (half_x, -half_y, bottom_z),
        (-half_x, -half_y, bottom_z),
        (-half_x, half_y, bottom_z),
    ]

    for i in range(4):
        _add_member(
            frame,
            top_corners[i],
            top_corners[(i + 1) % 4],
            tube_radius,
            frame_yellow,
        )
        _add_member(
            frame,
            bottom_corners[i],
            bottom_corners[(i + 1) % 4],
            tube_radius,
            frame_yellow,
        )
        _add_member(frame, top_corners[i], bottom_corners[i], tube_radius, frame_yellow)

    _add_member(frame, (-0.175, 0.0, top_z), (0.175, 0.0, top_z), tube_radius, frame_yellow)
    _add_member(frame, (-0.175, 0.0, bottom_z), (0.175, 0.0, bottom_z), tube_radius, frame_yellow)
    _add_member(frame, (-0.080, -half_y, top_z), (-0.080, half_y, top_z), tube_radius, frame_yellow)
    _add_member(frame, (0.080, -half_y, top_z), (0.080, half_y, top_z), tube_radius, frame_yellow)
    _add_member(frame, (-0.080, -half_y, bottom_z), (-0.080, half_y, bottom_z), tube_radius, frame_yellow)
    _add_member(frame, (0.080, -half_y, bottom_z), (0.080, half_y, bottom_z), tube_radius, frame_yellow)
    _add_member(frame, (half_x, 0.0, bottom_z), (half_x, 0.0, top_z), tube_radius, frame_yellow)
    _add_member(frame, (-half_x, 0.0, bottom_z), (-half_x, 0.0, top_z), tube_radius, frame_yellow)
    _add_member(frame, (0.0, half_y, bottom_z), (0.0, half_y, top_z), tube_radius, frame_yellow)
    _add_member(frame, (0.0, -half_y, bottom_z), (0.0, -half_y, top_z), tube_radius, frame_yellow)

    frame.visual(
        Cylinder(radius=0.055, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material=hull_teal,
        name="pressure_hull",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.155, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(-0.155, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
    )

    for x in (-0.082, 0.082):
        for y in (-0.055, 0.055):
            frame.visual(
                Box((0.024, 0.016, 0.106)),
                origin=Origin(xyz=(x, y, 0.057)),
                material=silver,
            )
            frame.visual(
                Box((0.024, 0.016, 0.086)),
                origin=Origin(xyz=(x, y, -0.047)),
                material=silver,
            )

    frame.visual(
        Box((0.300, 0.140, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.1525)),
        material=foam_yellow,
        name="buoyancy_pack",
    )
    frame.visual(
        Box((0.260, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=dark_metal,
    )
    frame.visual(
        Box((0.240, 0.160, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=dark_metal,
        name="lower_tray",
    )

    skid_left_a = (-0.200, 0.145, -0.145)
    skid_left_b = (0.200, 0.145, -0.145)
    skid_right_a = (-0.200, -0.145, -0.145)
    skid_right_b = (0.200, -0.145, -0.145)
    _add_member(frame, skid_left_a, skid_left_b, 0.010, dark_metal)
    _add_member(frame, skid_right_a, skid_right_b, 0.010, dark_metal)
    for corner, skid in (
        (bottom_corners[0], (0.185, 0.145, -0.145)),
        (bottom_corners[3], (-0.185, 0.145, -0.145)),
        (bottom_corners[1], (0.185, -0.145, -0.145)),
        (bottom_corners[2], (-0.185, -0.145, -0.145)),
    ):
        _add_member(frame, corner, skid, 0.009, dark_metal)

    _add_member(frame, (half_x, -0.080, 0.035), (half_x, 0.080, 0.035), 0.010, frame_yellow)
    _add_member(frame, (-half_x, -0.080, 0.035), (-half_x, 0.080, 0.035), 0.010, frame_yellow)
    _add_member(frame, (-0.080, half_y, 0.020), (0.080, half_y, 0.020), 0.010, frame_yellow)
    _add_member(frame, (-0.080, -half_y, 0.020), (0.080, -half_y, 0.020), 0.010, frame_yellow)

    _add_thruster_assembly(
        frame,
        duct_mesh_name="front_duct_mesh",
        duct_name="front_duct",
        motor_name="front_motor",
        mount_name="front_mount",
        center=(0.295, 0.0, 0.035),
        axis=(1.0, 0.0, 0.0),
        frame_anchor=(0.228, 0.0, 0.035),
        frame_material=frame_yellow,
        duct_material=duct_grey,
        motor_material=dark_metal,
    )
    _add_thruster_assembly(
        frame,
        duct_mesh_name="rear_duct_mesh",
        duct_name="rear_duct",
        motor_name="rear_motor",
        mount_name="rear_mount",
        center=(-0.295, 0.0, 0.035),
        axis=(-1.0, 0.0, 0.0),
        frame_anchor=(-0.228, 0.0, 0.035),
        frame_material=frame_yellow,
        duct_material=duct_grey,
        motor_material=dark_metal,
    )
    _add_thruster_assembly(
        frame,
        duct_mesh_name="left_duct_mesh",
        duct_name="left_duct",
        motor_name="left_motor",
        mount_name="left_mount",
        center=(0.0, 0.225, 0.020),
        axis=(0.0, 1.0, 0.0),
        frame_anchor=(0.0, 0.158, 0.020),
        frame_material=frame_yellow,
        duct_material=duct_grey,
        motor_material=dark_metal,
    )
    _add_thruster_assembly(
        frame,
        duct_mesh_name="right_duct_mesh",
        duct_name="right_duct",
        motor_name="right_motor",
        mount_name="right_mount",
        center=(0.0, -0.225, 0.020),
        axis=(0.0, -1.0, 0.0),
        frame_anchor=(0.0, -0.158, 0.020),
        frame_material=frame_yellow,
        duct_material=duct_grey,
        motor_material=dark_metal,
    )

    frame.visual(
        Box((0.070, 0.130, 0.032)),
        origin=Origin(xyz=(0.245, 0.0, -0.018)),
        material=dark_metal,
        name="nose_bracket_bridge",
    )
    frame.visual(
        Box((0.180, 0.014, 0.068)),
        origin=Origin(xyz=(0.290, 0.070, 0.002)),
        material=dark_metal,
        name="nose_bracket_left_arm",
    )
    frame.visual(
        Box((0.180, 0.014, 0.068)),
        origin=Origin(xyz=(0.290, -0.070, 0.002)),
        material=dark_metal,
        name="nose_bracket_right_arm",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.370, 0.070, 0.002), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.370, -0.070, 0.002), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
    )

    front_propeller = model.part("front_propeller")
    front_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.022),
        mass=0.10,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_propeller_visuals(
        front_propeller,
        axis_name="x",
        prop_material=prop_black,
        hub_material=silver,
    )

    rear_propeller = model.part("rear_propeller")
    rear_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.022),
        mass=0.10,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_propeller_visuals(
        rear_propeller,
        axis_name="x",
        prop_material=prop_black,
        hub_material=silver,
    )

    left_propeller = model.part("left_propeller")
    left_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.022),
        mass=0.10,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_propeller_visuals(
        left_propeller,
        axis_name="y",
        prop_material=prop_black,
        hub_material=silver,
    )

    right_propeller = model.part("right_propeller")
    right_propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.022),
        mass=0.10,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_propeller_visuals(
        right_propeller,
        axis_name="y",
        prop_material=prop_black,
        hub_material=silver,
    )

    camera_pod = model.part("camera_pod")
    camera_pod.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.090)),
        mass=0.55,
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
    )
    camera_pod.visual(
        Cylinder(radius=0.037, length=0.072),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="camera_shell",
    )
    camera_pod.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=dark_metal,
    )
    camera_pod.visual(
        Sphere(radius=0.031),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=camera_glass,
        name="front_glass",
    )
    camera_pod.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver,
    )
    camera_pod.visual(
        Cylinder(radius=0.008, length=0.126),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="camera_trunnion",
    )

    model.articulation(
        "front_thruster_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_propeller,
        origin=Origin(xyz=(0.295, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=42.0),
    )
    model.articulation(
        "rear_thruster_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_propeller,
        origin=Origin(xyz=(-0.295, 0.0, 0.035)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=42.0),
    )
    model.articulation(
        "left_thruster_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_propeller,
        origin=Origin(xyz=(0.0, 0.225, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=42.0),
    )
    model.articulation(
        "right_thruster_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_propeller,
        origin=Origin(xyz=(0.0, -0.225, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=42.0),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=camera_pod,
        origin=Origin(xyz=(0.370, 0.0, 0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.6,
            lower=-0.70,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame_body")
    front_propeller = object_model.get_part("front_propeller")
    rear_propeller = object_model.get_part("rear_propeller")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    camera_pod = object_model.get_part("camera_pod")

    front_joint = object_model.get_articulation("front_thruster_spin")
    rear_joint = object_model.get_articulation("rear_thruster_spin")
    left_joint = object_model.get_articulation("left_thruster_spin")
    right_joint = object_model.get_articulation("right_thruster_spin")
    camera_joint = object_model.get_articulation("camera_tilt")
    front_duct = frame.get_visual("front_duct")
    rear_duct = frame.get_visual("rear_duct")
    left_duct = frame.get_visual("left_duct")
    right_duct = frame.get_visual("right_duct")

    for joint, expected_axis in (
        (front_joint, (1.0, 0.0, 0.0)),
        (rear_joint, (-1.0, 0.0, 0.0)),
        (left_joint, (0.0, 1.0, 0.0)),
        (right_joint, (0.0, -1.0, 0.0)),
    ):
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name} axis is correct",
            tuple(joint.axis) == expected_axis,
            details=f"axis={joint.axis}, expected={expected_axis}",
        )

    ctx.check(
        "camera tilt is revolute about lateral axis",
        camera_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(camera_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={camera_joint.articulation_type}, axis={camera_joint.axis}",
    )

    ctx.expect_within(
        front_propeller,
        frame,
        axes="yz",
        outer_elem=front_duct,
        margin=0.004,
        name="front propeller stays inside front duct opening",
    )
    ctx.expect_within(
        rear_propeller,
        frame,
        axes="yz",
        outer_elem=rear_duct,
        margin=0.004,
        name="rear propeller stays inside rear duct opening",
    )
    ctx.expect_within(
        left_propeller,
        frame,
        axes="xz",
        outer_elem=left_duct,
        margin=0.004,
        name="left propeller stays inside left duct opening",
    )
    ctx.expect_within(
        right_propeller,
        frame,
        axes="xz",
        outer_elem=right_duct,
        margin=0.004,
        name="right propeller stays inside right duct opening",
    )

    with ctx.pose({camera_joint: -0.55}):
        down_glass = _glass_center(ctx.part_element_world_aabb(camera_pod, elem="front_glass"))
    with ctx.pose({camera_joint: 0.55}):
        up_glass = _glass_center(ctx.part_element_world_aabb(camera_pod, elem="front_glass"))

    ctx.check(
        "camera pod raises its nose at positive tilt",
        down_glass is not None
        and up_glass is not None
        and up_glass[2] > down_glass[2] + 0.045,
        details=f"down_glass={down_glass}, up_glass={up_glass}",
    )
    ctx.check(
        "camera pod stays near the nose bracket while tilting",
        down_glass is not None
        and up_glass is not None
        and abs(up_glass[0] - down_glass[0]) < 0.040,
        details=f"down_glass={down_glass}, up_glass={up_glass}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
