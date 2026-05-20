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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolt(
    part,
    *,
    center: tuple[float, float, float],
    axis: str,
    radius: float,
    length: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_vacuum")

    body_gray = model.material("body_gray", rgba=(0.47, 0.50, 0.53, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.92, 0.79, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    hazard_red = model.material("hazard_red", rgba=(0.75, 0.16, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.44, 0.32, 0.23)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=body_gray,
        name="lower_tank",
    )
    body.visual(
        Box((0.24, 0.26, 0.18)),
        origin=Origin(xyz=(-0.10, 0.0, 0.31)),
        material=body_gray,
        name="power_housing",
    )
    body.visual(
        Box((0.18, 0.18, 0.03)),
        origin=Origin(xyz=(-0.10, 0.0, 0.405)),
        material=dark_steel,
        name="top_motor_guard",
    )
    body.visual(
        Box((0.14, 0.22, 0.16)),
        origin=Origin(xyz=(0.10, 0.0, 0.22)),
        material=guard_yellow,
        name="front_intake_tower",
    )
    body.visual(
        Box((0.18, 0.12, 0.016)),
        origin=Origin(xyz=(0.14, 0.0, 0.288)),
        material=dark_steel,
        name="wand_mount_plate",
    )
    body.visual(
        Box((0.34, 0.010, 0.24)),
        origin=Origin(xyz=(-0.02, 0.165, 0.20)),
        material=dark_steel,
        name="left_side_plate",
    )
    body.visual(
        Box((0.34, 0.010, 0.24)),
        origin=Origin(xyz=(-0.02, -0.165, 0.20)),
        material=dark_steel,
        name="right_side_plate",
    )
    body.visual(
        Box((0.05, 0.12, 0.24)),
        origin=Origin(xyz=(0.145, 0.0, 0.255)),
        material=dark_steel,
        name="shoulder_spine",
    )
    body.visual(
        Box((0.026, 0.008, 0.09)),
        origin=Origin(xyz=(0.205, 0.049, 0.33)),
        material=dark_steel,
        name="shoulder_left_plate",
    )
    body.visual(
        Box((0.026, 0.008, 0.09)),
        origin=Origin(xyz=(0.205, -0.049, 0.33)),
        material=dark_steel,
        name="shoulder_right_plate",
    )
    body.visual(
        Box((0.052, 0.090, 0.030)),
        origin=Origin(xyz=(0.166, 0.0, 0.390)),
        material=dark_steel,
        name="shoulder_cap",
    )
    body.visual(
        Box((0.034, 0.016, 0.020)),
        origin=Origin(xyz=(0.176, 0.0, 0.292)),
        material=hazard_red,
        name="shoulder_left_stop",
    )
    body.visual(
        Box((0.020, 0.048, 0.026)),
        origin=Origin(xyz=(0.166, 0.0, 0.272)),
        material=dark_steel,
        name="shoulder_right_stop",
    )
    body.visual(
        Box((0.08, 0.18, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, 0.055)),
        material=dark_steel,
        name="front_roller_cradle",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.16),
        origin=Origin(xyz=(0.18, 0.0, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="front_roller",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.40),
        origin=Origin(xyz=(-0.10, 0.0, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.05),
        origin=Origin(xyz=(-0.10, 0.225, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="left_rear_wheel",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.05),
        origin=Origin(xyz=(-0.10, -0.225, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="right_rear_wheel",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(-0.10, 0.195, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="left_wheel_hub",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.026),
        origin=Origin(xyz=(-0.10, -0.195, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="right_wheel_hub",
    )
    body.visual(
        Box((0.11, 0.25, 0.02)),
        origin=Origin(xyz=(-0.22, 0.0, 0.105)),
        material=hazard_red,
        name="rear_bumper_crossbar",
    )
    body.visual(
        Box((0.045, 0.04, 0.08)),
        origin=Origin(xyz=(-0.17, 0.125, 0.14)),
        material=dark_steel,
        name="left_handle_mount",
    )
    body.visual(
        Box((0.045, 0.04, 0.08)),
        origin=Origin(xyz=(-0.17, -0.125, 0.14)),
        material=dark_steel,
        name="right_handle_mount",
    )
    body.visual(
        Box((0.16, 0.12, 0.04)),
        origin=Origin(xyz=(-0.12, 0.0, 0.19)),
        material=dark_steel,
        name="service_frame_bridge",
    )
    _add_member(
        body,
        (-0.19, 0.125, 0.18),
        (-0.24, 0.125, 0.52),
        radius=0.014,
        material=dark_steel,
        name="left_handle_upright",
    )
    _add_member(
        body,
        (-0.19, -0.125, 0.18),
        (-0.24, -0.125, 0.52),
        radius=0.014,
        material=dark_steel,
        name="right_handle_upright",
    )
    _add_member(
        body,
        (-0.24, -0.125, 0.52),
        (-0.24, 0.125, 0.52),
        radius=0.014,
        material=dark_steel,
        name="handle_crossbar",
    )
    body.visual(
        Box((0.11, 0.018, 0.085)),
        origin=Origin(xyz=(0.122, 0.025, 0.278)),
        material=dark_steel,
        name="left_shoulder_brace",
    )
    body.visual(
        Box((0.11, 0.018, 0.085)),
        origin=Origin(xyz=(0.122, -0.025, 0.278)),
        material=dark_steel,
        name="right_shoulder_brace",
    )
    _add_member(
        body,
        (-0.02, 0.09, 0.33),
        (0.13, 0.045, 0.31),
        radius=0.007,
        material=dark_steel,
        name="left_rear_mount_brace",
    )
    _add_member(
        body,
        (-0.02, -0.09, 0.33),
        (0.13, -0.045, 0.31),
        radius=0.007,
        material=dark_steel,
        name="right_rear_mount_brace",
    )
    for index, z in enumerate((0.12, 0.18, 0.24, 0.30)):
        _add_bolt(
            body,
            center=(-0.08, 0.169, z),
            axis="y",
            radius=0.006,
            length=0.008,
            material=fastener_steel,
            name=f"left_panel_bolt_{index}",
        )
        _add_bolt(
            body,
            center=(-0.08, -0.169, z),
            axis="y",
            radius=0.006,
            length=0.008,
            material=fastener_steel,
            name=f"right_panel_bolt_{index}",
        )
    for index, z in enumerate((0.304, 0.348)):
        _add_bolt(
            body,
            center=(0.218, 0.053, z),
            axis="y",
            radius=0.005,
            length=0.010,
            material=fastener_steel,
            name=f"left_shoulder_bolt_{index}",
        )
        _add_bolt(
            body,
            center=(0.218, -0.053, z),
            axis="y",
            radius=0.005,
            length=0.010,
            material=fastener_steel,
            name=f"right_shoulder_bolt_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.58, 0.50, 0.58)),
        mass=28.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.23)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.024, length=0.082),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_barrel",
    )
    lower_wand.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="shoulder_left_washer",
    )
    lower_wand.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="shoulder_right_washer",
    )
    lower_wand.visual(
        Box((0.060, 0.060, 0.056)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=guard_yellow,
        name="shoulder_lockout_collar",
    )
    lower_wand.visual(
        Box((0.34, 0.050, 0.034)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=body_gray,
        name="lower_main_tube",
    )
    lower_wand.visual(
        Box((0.18, 0.064, 0.010)),
        origin=Origin(xyz=(0.16, 0.0, 0.022)),
        material=dark_steel,
        name="lower_top_reinforcement",
    )
    lower_wand.visual(
        Box((0.10, 0.070, 0.010)),
        origin=Origin(xyz=(0.09, 0.0, -0.022)),
        material=dark_steel,
        name="lower_bottom_wear_plate",
    )
    lower_wand.visual(
        Box((0.046, 0.008, 0.060)),
        origin=Origin(xyz=(0.376, 0.040, 0.0)),
        material=dark_steel,
        name="elbow_left_plate",
    )
    lower_wand.visual(
        Box((0.046, 0.008, 0.060)),
        origin=Origin(xyz=(0.376, -0.040, 0.0)),
        material=dark_steel,
        name="elbow_right_plate",
    )
    lower_wand.visual(
        Box((0.032, 0.076, 0.028)),
        origin=Origin(xyz=(0.338, 0.0, 0.002)),
        material=dark_steel,
        name="elbow_bridge",
    )
    lower_wand.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(0.360, 0.0, 0.026)),
        material=hazard_red,
        name="left_elbow_stop",
    )
    lower_wand.visual(
        Box((0.018, 0.050, 0.018)),
        origin=Origin(xyz=(0.351, 0.0, 0.013)),
        material=hazard_red,
        name="right_elbow_stop",
    )
    _add_member(
        lower_wand,
        (0.28, 0.018, 0.015),
        (0.360, 0.028, 0.0),
        radius=0.006,
        material=dark_steel,
        name="left_elbow_brace",
    )
    _add_member(
        lower_wand,
        (0.28, -0.018, 0.015),
        (0.360, -0.028, 0.0),
        radius=0.006,
        material=dark_steel,
        name="right_elbow_brace",
    )
    for index, x in enumerate((0.020, 0.055)):
        _add_bolt(
            lower_wand,
            center=(x, 0.034, 0.0),
            axis="y",
            radius=0.005,
            length=0.010,
            material=fastener_steel,
            name=f"left_shoulder_collar_bolt_{index}",
        )
        _add_bolt(
            lower_wand,
            center=(x, -0.034, 0.0),
            axis="y",
            radius=0.005,
            length=0.010,
            material=fastener_steel,
            name=f"right_shoulder_collar_bolt_{index}",
        )
    for index, z in enumerate((-0.016, 0.016)):
        _add_bolt(
            lower_wand,
            center=(0.396, 0.040, z),
            axis="y",
            radius=0.0048,
            length=0.010,
            material=fastener_steel,
            name=f"left_elbow_plate_bolt_{index}",
        )
        _add_bolt(
            lower_wand,
            center=(0.396, -0.040, z),
            axis="y",
            radius=0.0048,
            length=0.010,
            material=fastener_steel,
            name=f"right_elbow_plate_bolt_{index}",
        )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.42, 0.10, 0.10)),
        mass=4.5,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.022, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_barrel",
    )
    upper_wand.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="elbow_left_washer",
    )
    upper_wand.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="elbow_right_washer",
    )
    upper_wand.visual(
        Box((0.055, 0.056, 0.052)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=guard_yellow,
        name="elbow_lockout_collar",
    )
    upper_wand.visual(
        Box((0.25, 0.044, 0.030)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=body_gray,
        name="upper_main_tube",
    )
    upper_wand.visual(
        Box((0.16, 0.058, 0.010)),
        origin=Origin(xyz=(0.13, 0.0, 0.020)),
        material=dark_steel,
        name="upper_top_reinforcement",
    )
    upper_wand.visual(
        Box((0.12, 0.060, 0.010)),
        origin=Origin(xyz=(0.11, 0.0, -0.020)),
        material=dark_steel,
        name="upper_bottom_wear_plate",
    )
    upper_wand.visual(
        Box((0.042, 0.008, 0.055)),
        origin=Origin(xyz=(0.292, 0.038, 0.0)),
        material=dark_steel,
        name="neck_left_plate",
    )
    upper_wand.visual(
        Box((0.042, 0.008, 0.055)),
        origin=Origin(xyz=(0.292, -0.038, 0.0)),
        material=dark_steel,
        name="neck_right_plate",
    )
    upper_wand.visual(
        Box((0.050, 0.076, 0.028)),
        origin=Origin(xyz=(0.246, 0.0, 0.000)),
        material=dark_steel,
        name="neck_bridge",
    )
    upper_wand.visual(
        Box((0.028, 0.018, 0.020)),
        origin=Origin(xyz=(0.278, 0.0, 0.024)),
        material=hazard_red,
        name="left_neck_stop_host",
    )
    upper_wand.visual(
        Box((0.018, 0.050, 0.018)),
        origin=Origin(xyz=(0.265, 0.0, 0.013)),
        material=hazard_red,
        name="right_neck_stop_host",
    )
    _add_member(
        upper_wand,
        (0.21, 0.008, 0.010),
        (0.272, 0.018, -0.004),
        radius=0.0055,
        material=dark_steel,
        name="left_neck_brace",
    )
    _add_member(
        upper_wand,
        (0.21, -0.008, 0.010),
        (0.272, -0.018, -0.004),
        radius=0.0055,
        material=dark_steel,
        name="right_neck_brace",
    )
    for index, x in enumerate((0.018, 0.048)):
        _add_bolt(
            upper_wand,
            center=(x, 0.028, 0.0),
            axis="y",
            radius=0.0048,
            length=0.010,
            material=fastener_steel,
            name=f"left_elbow_collar_bolt_{index}",
        )
        _add_bolt(
            upper_wand,
            center=(x, -0.028, 0.0),
            axis="y",
            radius=0.0048,
            length=0.010,
            material=fastener_steel,
            name=f"right_elbow_collar_bolt_{index}",
        )
    for index, z in enumerate((-0.014, 0.014)):
        _add_bolt(
            upper_wand,
            center=(0.308, 0.038, z),
            axis="y",
            radius=0.0045,
            length=0.010,
            material=fastener_steel,
            name=f"left_neck_plate_bolt_{index}",
        )
        _add_bolt(
            upper_wand,
            center=(0.308, -0.038, z),
            axis="y",
            radius=0.0045,
            length=0.010,
            material=fastener_steel,
            name=f"right_neck_plate_bolt_{index}",
        )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.34, 0.09, 0.09)),
        mass=3.2,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.022, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="neck_barrel",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="neck_left_washer",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="neck_right_washer",
    )
    floor_nozzle.visual(
        Box((0.080, 0.055, 0.050)),
        origin=Origin(xyz=(0.032, 0.0, -0.006)),
        material=guard_yellow,
        name="neck_block",
    )
    floor_nozzle.visual(
        Box((0.26, 0.32, 0.030)),
        origin=Origin(xyz=(0.14, 0.0, -0.055)),
        material=body_gray,
        name="nozzle_body",
    )
    floor_nozzle.visual(
        Box((0.15, 0.22, 0.024)),
        origin=Origin(xyz=(0.11, 0.0, -0.030)),
        material=guard_yellow,
        name="top_guard",
    )
    floor_nozzle.visual(
        Box((0.20, 0.010, 0.046)),
        origin=Origin(xyz=(0.14, 0.155, -0.043)),
        material=dark_steel,
        name="left_side_skid",
    )
    floor_nozzle.visual(
        Box((0.20, 0.010, 0.046)),
        origin=Origin(xyz=(0.14, -0.155, -0.043)),
        material=dark_steel,
        name="right_side_skid",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.012, length=0.28),
        origin=Origin(xyz=(0.25, 0.0, -0.051), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hazard_red,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.16, 0.25, 0.008)),
        origin=Origin(xyz=(0.11, 0.0, -0.072)),
        material=rubber_black,
        name="brush_strip",
    )
    floor_nozzle.visual(
        Box((0.018, 0.024, 0.020)),
        origin=Origin(xyz=(0.020, 0.016, 0.028)),
        material=hazard_red,
        name="left_neck_stop",
    )
    floor_nozzle.visual(
        Box((0.018, 0.024, 0.020)),
        origin=Origin(xyz=(0.020, -0.016, 0.028)),
        material=hazard_red,
        name="right_neck_stop",
    )
    _add_member(
        floor_nozzle,
        (0.055, 0.11, -0.028),
        (0.17, 0.145, -0.050),
        radius=0.005,
        material=dark_steel,
        name="left_nozzle_brace",
    )
    _add_member(
        floor_nozzle,
        (0.055, -0.11, -0.028),
        (0.17, -0.145, -0.050),
        radius=0.005,
        material=dark_steel,
        name="right_nozzle_brace",
    )
    for index, x in enumerate((0.018, 0.048)):
        _add_bolt(
            floor_nozzle,
            center=(x, 0.028, 0.0),
            axis="y",
            radius=0.0048,
            length=0.010,
            material=fastener_steel,
            name=f"left_neck_collar_bolt_{index}",
        )
        _add_bolt(
            floor_nozzle,
            center=(x, -0.028, 0.0),
            axis="y",
            radius=0.0048,
            length=0.010,
            material=fastener_steel,
            name=f"right_neck_collar_bolt_{index}",
        )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.28, 0.34, 0.10)),
        mass=2.8,
        origin=Origin(xyz=(0.14, 0.0, -0.04)),
    )

    model.articulation(
        "body_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_wand,
        origin=Origin(xyz=(0.205, 0.0, 0.33), rpy=(0.0, -0.23, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=-0.30,
            upper=0.75,
        ),
    )
    model.articulation(
        "lower_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.392, 0.0, 0.0), rpy=(0.0, -0.28, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.0,
            lower=-0.22,
            upper=0.95,
        ),
    )
    model.articulation(
        "upper_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.304, 0.0, 0.0), rpy=(0.0, 0.51, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-0.40,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lower_wand = object_model.get_part("lower_wand")
    upper_wand = object_model.get_part("upper_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    shoulder = object_model.get_articulation("body_to_lower_wand")
    elbow = object_model.get_articulation("lower_to_upper_wand")
    neck = object_model.get_articulation("upper_to_nozzle")

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
        body,
        lower_wand,
        elem_a="shoulder_left_plate",
        elem_b="shoulder_left_washer",
        contact_tol=1e-5,
        name="left_shoulder_washer_seated",
    )
    ctx.expect_contact(
        body,
        lower_wand,
        elem_a="shoulder_right_plate",
        elem_b="shoulder_right_washer",
        contact_tol=1e-5,
        name="right_shoulder_washer_seated",
    )
    ctx.expect_contact(
        lower_wand,
        upper_wand,
        elem_a="elbow_left_plate",
        elem_b="elbow_left_washer",
        contact_tol=1e-5,
        name="left_elbow_washer_seated",
    )
    ctx.expect_contact(
        lower_wand,
        upper_wand,
        elem_a="elbow_right_plate",
        elem_b="elbow_right_washer",
        contact_tol=1e-5,
        name="right_elbow_washer_seated",
    )
    ctx.expect_contact(
        upper_wand,
        floor_nozzle,
        elem_a="neck_left_plate",
        elem_b="neck_left_washer",
        contact_tol=1e-5,
        name="left_neck_washer_seated",
    )
    ctx.expect_contact(
        upper_wand,
        floor_nozzle,
        elem_a="neck_right_plate",
        elem_b="neck_right_washer",
        contact_tol=1e-5,
        name="right_neck_washer_seated",
    )

    rest_nozzle_pos = ctx.part_world_position(floor_nozzle)
    with ctx.pose({shoulder: 0.55}):
        shoulder_raised_nozzle_pos = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "shoulder_positive_motion_raises_wand",
        rest_nozzle_pos is not None
        and shoulder_raised_nozzle_pos is not None
        and shoulder_raised_nozzle_pos[2] > rest_nozzle_pos[2] + 0.09,
        details=(
            f"rest={rest_nozzle_pos}, raised={shoulder_raised_nozzle_pos}; "
            "positive shoulder motion should lift the wand assembly."
        ),
    )

    with ctx.pose({elbow: 0.70}):
        elbow_raised_nozzle_pos = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "elbow_positive_motion_raises_nozzle",
        rest_nozzle_pos is not None
        and elbow_raised_nozzle_pos is not None
        and elbow_raised_nozzle_pos[2] > rest_nozzle_pos[2] + 0.05,
        details=(
            f"rest={rest_nozzle_pos}, elbow_pose={elbow_raised_nozzle_pos}; "
            "positive elbow motion should fold the upper wand upward."
        ),
    )

    rest_front_bumper = ctx.part_element_world_aabb(floor_nozzle, elem="front_bumper")
    with ctx.pose({neck: 0.24}):
        pitched_front_bumper = ctx.part_element_world_aabb(floor_nozzle, elem="front_bumper")
    ctx.check(
        "neck_positive_motion_lifts_nozzle_front",
        rest_front_bumper is not None
        and pitched_front_bumper is not None
        and pitched_front_bumper[0][2] > rest_front_bumper[0][2] + 0.015,
        details=(
            f"rest={rest_front_bumper}, pitched={pitched_front_bumper}; "
            "positive nozzle pitch should lift the front bumper for obstacle clearance."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
