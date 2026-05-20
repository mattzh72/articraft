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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_side_bolt(
    part,
    *,
    x: float,
    y: float,
    z: float,
    material,
    radius: float = 0.008,
    length: float = 0.010,
    name: str | None = None,
) -> None:
    sign = 1.0 if y >= 0.0 else -1.0
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(x, y + sign * (length * 0.5 - 0.002), z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_top_bolt(
    part,
    *,
    x: float,
    y: float,
    z: float,
    material,
    radius: float = 0.008,
    length: float = 0.010,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z + length * 0.5 - 0.002)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_robotic_leg")

    shell_yellow = model.material("shell_yellow", rgba=(0.82, 0.68, 0.12, 1.0))
    structure_gray = model.material("structure_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    service_black = model.material("service_black", rgba=(0.09, 0.10, 0.11, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.72, 0.12, 0.08, 1.0))

    hip_carrier = model.part("hip_carrier")
    hip_carrier.visual(
        Box((0.22, 0.22, 0.025)),
        origin=Origin(xyz=(-0.04, 0.0, 0.108)),
        material=structure_gray,
        name="mount_plate",
    )
    hip_carrier.visual(
        Box((0.07, 0.18, 0.24)),
        origin=Origin(xyz=(-0.095, 0.0, -0.015)),
        material=service_black,
        name="rear_service_spine",
    )
    hip_carrier.visual(
        Box((0.07, 0.22, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, -0.108)),
        material=structure_gray,
        name="rear_crossbeam",
    )
    hip_carrier.visual(
        Box((0.08, 0.02, 0.19)),
        origin=Origin(xyz=(-0.005, 0.115, -0.05)),
        material=shell_yellow,
        name="left_yoke_plate",
    )
    hip_carrier.visual(
        Box((0.08, 0.02, 0.19)),
        origin=Origin(xyz=(-0.005, -0.115, -0.05)),
        material=shell_yellow,
        name="right_yoke_plate",
    )
    hip_carrier.visual(
        Box((0.045, 0.02, 0.08)),
        origin=Origin(xyz=(-0.055, 0.115, -0.075)),
        material=structure_gray,
        name="left_rear_gusset",
    )
    hip_carrier.visual(
        Box((0.045, 0.02, 0.08)),
        origin=Origin(xyz=(-0.055, -0.115, -0.075)),
        material=structure_gray,
        name="right_rear_gusset",
    )
    hip_carrier.visual(
        Box((0.04, 0.02, 0.07)),
        origin=Origin(xyz=(0.018, 0.115, -0.085)),
        material=structure_gray,
        name="left_lower_strap",
    )
    hip_carrier.visual(
        Box((0.04, 0.02, 0.07)),
        origin=Origin(xyz=(0.018, -0.115, -0.085)),
        material=structure_gray,
        name="right_lower_strap",
    )
    hip_carrier.visual(
        Box((0.045, 0.02, 0.05)),
        origin=Origin(xyz=(-0.03, 0.115, 0.055)),
        material=structure_gray,
        name="left_upper_cap_rail",
    )
    hip_carrier.visual(
        Box((0.045, 0.02, 0.05)),
        origin=Origin(xyz=(-0.03, -0.115, 0.055)),
        material=structure_gray,
        name="right_upper_cap_rail",
    )
    hip_carrier.visual(
        Box((0.03, 0.22, 0.05)),
        origin=Origin(xyz=(-0.06, 0.0, 0.055)),
        material=structure_gray,
        name="upper_tie_block",
    )
    hip_carrier.visual(
        Cylinder(radius=0.042, length=0.024),
        origin=Origin(xyz=(0.0, 0.127, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_hip_bearing_cap",
    )
    hip_carrier.visual(
        Cylinder(radius=0.042, length=0.024),
        origin=Origin(xyz=(0.0, -0.127, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_hip_bearing_cap",
    )
    hip_carrier.visual(
        Box((0.018, 0.012, 0.05)),
        origin=Origin(xyz=(0.04, 0.131, -0.08)),
        material=structure_gray,
        name="left_forward_stop_block",
    )
    hip_carrier.visual(
        Box((0.018, 0.012, 0.05)),
        origin=Origin(xyz=(0.04, -0.131, -0.08)),
        material=structure_gray,
        name="right_forward_stop_block",
    )
    hip_carrier.visual(
        Box((0.026, 0.012, 0.045)),
        origin=Origin(xyz=(-0.015, 0.131, -0.075)),
        material=lockout_red,
        name="hip_lockout_tab",
    )
    for index, z in enumerate((0.015, -0.03, -0.075)):
        _add_side_bolt(
            hip_carrier,
            x=0.0,
            y=0.124,
            z=z,
            material=dark_steel,
            radius=0.006,
            length=0.008,
            name=f"left_yoke_bolt_{index}",
        )
        _add_side_bolt(
            hip_carrier,
            x=0.0,
            y=-0.124,
            z=z,
            material=dark_steel,
            radius=0.006,
            length=0.008,
            name=f"right_yoke_bolt_{index}",
        )
    hip_carrier.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.28)),
        mass=48.0,
        origin=Origin(xyz=(-0.045, 0.0, -0.01)),
    )

    thigh_module = model.part("thigh_module")
    thigh_module.visual(
        Cylinder(radius=0.038, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hip_barrel",
    )
    thigh_module.visual(
        Box((0.08, 0.024, 0.14)),
        origin=Origin(xyz=(-0.01, 0.088, -0.07)),
        material=structure_gray,
        name="left_hip_cheek",
    )
    thigh_module.visual(
        Box((0.08, 0.024, 0.14)),
        origin=Origin(xyz=(-0.01, -0.088, -0.07)),
        material=structure_gray,
        name="right_hip_cheek",
    )
    thigh_module.visual(
        Box((0.13, 0.02, 0.26)),
        origin=Origin(xyz=(0.015, 0.066, -0.18)),
        material=shell_yellow,
        name="left_side_shell",
    )
    thigh_module.visual(
        Box((0.13, 0.02, 0.26)),
        origin=Origin(xyz=(0.015, -0.066, -0.18)),
        material=shell_yellow,
        name="right_side_shell",
    )
    thigh_module.visual(
        Box((0.03, 0.14, 0.34)),
        origin=Origin(xyz=(0.072, 0.0, -0.17)),
        material=shell_yellow,
        name="front_spine_plate",
    )
    thigh_module.visual(
        Cylinder(radius=0.026, length=0.24),
        origin=Origin(xyz=(-0.035, 0.0, -0.22)),
        material=service_black,
        name="actuator_bay",
    )
    thigh_module.visual(
        Box((0.08, 0.12, 0.04)),
        origin=Origin(xyz=(-0.025, 0.0, -0.155)),
        material=structure_gray,
        name="upper_actuator_cradle",
    )
    thigh_module.visual(
        Box((0.10, 0.13, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=structure_gray,
        name="upper_crossmember",
    )
    thigh_module.visual(
        Box((0.05, 0.13, 0.04)),
        origin=Origin(xyz=(0.055, 0.0, -0.24)),
        material=shell_yellow,
        name="bay_guard_cover",
    )
    thigh_module.visual(
        Box((0.13, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.37)),
        material=structure_gray,
        name="lower_crossmember",
    )
    thigh_module.visual(
        Box((0.06, 0.022, 0.08)),
        origin=Origin(xyz=(0.0, 0.111, -0.41)),
        material=structure_gray,
        name="left_knee_support",
    )
    thigh_module.visual(
        Box((0.06, 0.022, 0.08)),
        origin=Origin(xyz=(0.0, -0.111, -0.41)),
        material=structure_gray,
        name="right_knee_support",
    )
    thigh_module.visual(
        Box((0.11, 0.022, 0.12)),
        origin=Origin(xyz=(0.0, 0.111, -0.47)),
        material=structure_gray,
        name="left_knee_yoke",
    )
    thigh_module.visual(
        Box((0.11, 0.022, 0.12)),
        origin=Origin(xyz=(0.0, -0.111, -0.47)),
        material=structure_gray,
        name="right_knee_yoke",
    )
    thigh_module.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, 0.123, -0.47), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_knee_bearing_cap",
    )
    thigh_module.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, -0.123, -0.47), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_knee_bearing_cap",
    )
    thigh_module.visual(
        Box((0.026, 0.018, 0.055)),
        origin=Origin(xyz=(0.052, 0.129, -0.42)),
        material=structure_gray,
        name="knee_forward_stop",
    )
    thigh_module.visual(
        Box((0.026, 0.018, 0.055)),
        origin=Origin(xyz=(0.052, -0.129, -0.42)),
        material=structure_gray,
        name="knee_forward_stop_mirror",
    )
    thigh_module.visual(
        Box((0.03, 0.012, 0.05)),
        origin=Origin(xyz=(0.04, -0.124, -0.42)),
        material=lockout_red,
        name="knee_lockout_tab",
    )
    _add_member(
        thigh_module,
        (0.06, 0.07, -0.08),
        (0.02, 0.095, -0.39),
        radius=0.012,
        material=structure_gray,
        name="left_front_load_brace",
    )
    _add_member(
        thigh_module,
        (0.06, -0.07, -0.08),
        (0.02, -0.095, -0.39),
        radius=0.012,
        material=structure_gray,
        name="right_front_load_brace",
    )
    _add_member(
        thigh_module,
        (-0.04, 0.07, -0.10),
        (-0.03, 0.095, -0.37),
        radius=0.011,
        material=structure_gray,
        name="left_rear_load_brace",
    )
    _add_member(
        thigh_module,
        (-0.04, -0.07, -0.10),
        (-0.03, -0.095, -0.37),
        radius=0.011,
        material=structure_gray,
        name="right_rear_load_brace",
    )
    for index, z in enumerate((-0.44, -0.49)):
        _add_side_bolt(
            thigh_module,
            x=0.025,
            y=0.119,
            z=z,
            material=dark_steel,
            name=f"left_knee_yoke_bolt_{index}",
        )
        _add_side_bolt(
            thigh_module,
            x=0.025,
            y=-0.119,
            z=z,
            material=dark_steel,
            name=f"right_knee_yoke_bolt_{index}",
        )
    thigh_module.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.54)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -0.25)),
    )

    shank_module = model.part("shank_module")
    shank_module.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knee_barrel",
    )
    shank_module.visual(
        Box((0.11, 0.03, 0.16)),
        origin=Origin(xyz=(0.0, 0.085, -0.05)),
        material=structure_gray,
        name="left_knee_cheek",
    )
    shank_module.visual(
        Box((0.11, 0.03, 0.16)),
        origin=Origin(xyz=(0.0, -0.085, -0.05)),
        material=structure_gray,
        name="right_knee_cheek",
    )
    shank_module.visual(
        Box((0.11, 0.022, 0.36)),
        origin=Origin(xyz=(0.01, 0.065, -0.22)),
        material=shell_yellow,
        name="left_shank_shell",
    )
    shank_module.visual(
        Box((0.11, 0.022, 0.36)),
        origin=Origin(xyz=(0.01, -0.065, -0.22)),
        material=shell_yellow,
        name="right_shank_shell",
    )
    shank_module.visual(
        Box((0.035, 0.12, 0.32)),
        origin=Origin(xyz=(-0.05, 0.0, -0.22)),
        material=structure_gray,
        name="rear_spine_plate",
    )
    shank_module.visual(
        Box((0.05, 0.12, 0.18)),
        origin=Origin(xyz=(0.06, 0.0, -0.14)),
        material=shell_yellow,
        name="upper_front_guard",
    )
    shank_module.visual(
        Box((0.04, 0.10, 0.10)),
        origin=Origin(xyz=(0.07, 0.0, -0.34)),
        material=shell_yellow,
        name="lower_front_guard",
    )
    shank_module.visual(
        Cylinder(radius=0.025, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=service_black,
        name="shank_actuator_core",
    )
    shank_module.visual(
        Box((0.07, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=structure_gray,
        name="upper_service_bridge",
    )
    shank_module.visual(
        Box((0.12, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        material=structure_gray,
        name="lower_service_bridge",
    )
    shank_module.visual(
        Box((0.055, 0.02, 0.085)),
        origin=Origin(xyz=(0.0, 0.10, -0.405)),
        material=structure_gray,
        name="left_ankle_support",
    )
    shank_module.visual(
        Box((0.055, 0.02, 0.085)),
        origin=Origin(xyz=(0.0, -0.10, -0.405)),
        material=structure_gray,
        name="right_ankle_support",
    )
    shank_module.visual(
        Box((0.09, 0.02, 0.10)),
        origin=Origin(xyz=(0.0, 0.10, -0.46)),
        material=structure_gray,
        name="left_ankle_yoke",
    )
    shank_module.visual(
        Box((0.09, 0.02, 0.10)),
        origin=Origin(xyz=(0.0, -0.10, -0.46)),
        material=structure_gray,
        name="right_ankle_yoke",
    )
    shank_module.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.111, -0.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_ankle_bearing_cap",
    )
    shank_module.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, -0.111, -0.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_ankle_bearing_cap",
    )
    shank_module.visual(
        Box((0.03, 0.012, 0.05)),
        origin=Origin(xyz=(-0.03, 0.116, -0.43)),
        material=lockout_red,
        name="ankle_lockout_tab",
    )
    shank_module.visual(
        Box((0.03, 0.12, 0.04)),
        origin=Origin(xyz=(-0.06, 0.0, -0.02)),
        material=structure_gray,
        name="knee_reverse_stop",
    )
    _add_member(
        shank_module,
        (0.03, 0.07, -0.03),
        (0.01, 0.088, -0.36),
        radius=0.011,
        material=structure_gray,
        name="left_front_shank_brace",
    )
    _add_member(
        shank_module,
        (0.03, -0.07, -0.03),
        (0.01, -0.088, -0.36),
        radius=0.011,
        material=structure_gray,
        name="right_front_shank_brace",
    )
    _add_member(
        shank_module,
        (-0.04, 0.07, -0.06),
        (-0.01, 0.09, -0.36),
        radius=0.011,
        material=structure_gray,
        name="left_rear_shank_brace",
    )
    _add_member(
        shank_module,
        (-0.04, -0.07, -0.06),
        (-0.01, -0.09, -0.36),
        radius=0.011,
        material=structure_gray,
        name="right_rear_shank_brace",
    )
    for index, z in enumerate((-0.01, -0.07)):
        _add_side_bolt(
            shank_module,
            x=0.03,
            y=0.086,
            z=z,
            material=dark_steel,
            radius=0.006,
            length=0.006,
            name=f"left_knee_cheek_bolt_{index}",
        )
        _add_side_bolt(
            shank_module,
            x=0.03,
            y=-0.086,
            z=z,
            material=dark_steel,
            radius=0.006,
            length=0.006,
            name=f"right_knee_cheek_bolt_{index}",
        )
    for index, z in enumerate((-0.43, -0.48)):
        _add_side_bolt(
            shank_module,
            x=0.02,
            y=0.108,
            z=z,
            material=dark_steel,
            name=f"left_ankle_yoke_bolt_{index}",
        )
        _add_side_bolt(
            shank_module,
            x=0.02,
            y=-0.108,
            z=z,
            material=dark_steel,
            name=f"right_ankle_yoke_bolt_{index}",
        )
    shank_module.inertial = Inertial.from_geometry(
        Box((0.18, 0.22, 0.50)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
    )

    foot_module = model.part("foot_module")
    foot_module.visual(
        Cylinder(radius=0.03, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="ankle_barrel",
    )
    foot_module.visual(
        Box((0.08, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, 0.075, -0.02)),
        material=structure_gray,
        name="left_ankle_cheek",
    )
    foot_module.visual(
        Box((0.08, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, -0.075, -0.02)),
        material=structure_gray,
        name="right_ankle_cheek",
    )
    foot_module.visual(
        Box((0.10, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=structure_gray,
        name="ankle_block",
    )
    foot_module.visual(
        Box((0.30, 0.16, 0.032)),
        origin=Origin(xyz=(0.08, 0.0, -0.10)),
        material=shell_yellow,
        name="sole_plate",
    )
    foot_module.visual(
        Box((0.08, 0.14, 0.05)),
        origin=Origin(xyz=(-0.08, 0.0, -0.085)),
        material=structure_gray,
        name="heel_block",
    )
    foot_module.visual(
        Box((0.05, 0.16, 0.05)),
        origin=Origin(xyz=(0.205, 0.0, -0.08)),
        material=structure_gray,
        name="toe_bumper",
    )
    foot_module.visual(
        Box((0.20, 0.015, 0.06)),
        origin=Origin(xyz=(0.09, 0.072, -0.065)),
        material=shell_yellow,
        name="left_side_guard",
    )
    foot_module.visual(
        Box((0.20, 0.015, 0.06)),
        origin=Origin(xyz=(0.09, -0.072, -0.065)),
        material=shell_yellow,
        name="right_side_guard",
    )
    foot_module.visual(
        Box((0.04, 0.12, 0.03)),
        origin=Origin(xyz=(-0.045, 0.0, 0.015)),
        material=structure_gray,
        name="ankle_stop_block",
    )
    foot_module.visual(
        Box((0.028, 0.012, 0.045)),
        origin=Origin(xyz=(0.13, -0.078, -0.065)),
        material=lockout_red,
        name="foot_lockout_tab",
    )
    _add_member(
        foot_module,
        (0.02, 0.05, -0.04),
        (0.13, 0.05, -0.09),
        radius=0.012,
        material=structure_gray,
        name="left_toe_load_brace",
    )
    _add_member(
        foot_module,
        (0.02, -0.05, -0.04),
        (0.13, -0.05, -0.09),
        radius=0.012,
        material=structure_gray,
        name="right_toe_load_brace",
    )
    _add_member(
        foot_module,
        (-0.02, 0.05, -0.03),
        (-0.06, 0.05, -0.08),
        radius=0.010,
        material=structure_gray,
        name="left_heel_brace",
    )
    _add_member(
        foot_module,
        (-0.02, -0.05, -0.03),
        (-0.06, -0.05, -0.08),
        radius=0.010,
        material=structure_gray,
        name="right_heel_brace",
    )
    for index, x in enumerate((-0.02, 0.04)):
        _add_side_bolt(
            foot_module,
            x=x,
            y=0.082,
            z=-0.02,
            material=dark_steel,
            radius=0.006,
            length=0.006,
            name=f"left_ankle_cheek_bolt_{index}",
        )
        _add_side_bolt(
            foot_module,
            x=x,
            y=-0.082,
            z=-0.02,
            material=dark_steel,
            radius=0.006,
            length=0.006,
            name=f"right_ankle_cheek_bolt_{index}",
        )
    foot_module.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.16)),
        mass=18.0,
        origin=Origin(xyz=(0.06, 0.0, -0.07)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_carrier,
        child=thigh_module,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.9,
            lower=-0.35,
            upper=0.85,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_module,
        child=shank_module,
        origin=Origin(xyz=(0.0, 0.0, -0.47)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2600.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_module,
        child=foot_module,
        origin=Origin(xyz=(0.0, 0.0, -0.46)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=1.1,
            lower=-0.30,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip_carrier = object_model.get_part("hip_carrier")
    thigh_module = object_model.get_part("thigh_module")
    shank_module = object_model.get_part("shank_module")
    foot_module = object_model.get_part("foot_module")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

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

    for part_name in ("hip_carrier", "thigh_module", "shank_module", "foot_module"):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None)

    ctx.expect_contact(hip_carrier, thigh_module, name="hip_joint_has_real_mount_contact")
    ctx.expect_contact(thigh_module, shank_module, name="knee_joint_has_real_mount_contact")
    ctx.expect_contact(shank_module, foot_module, name="ankle_joint_has_real_mount_contact")

    ctx.expect_origin_gap(
        hip_carrier,
        shank_module,
        axis="z",
        min_gap=0.42,
        max_gap=0.60,
        name="knee_axis_sits_below_hip_axis",
    )
    ctx.expect_origin_gap(
        shank_module,
        foot_module,
        axis="z",
        min_gap=0.40,
        max_gap=0.52,
        name="ankle_axis_sits_below_knee_axis",
    )

    hip_limits = hip_pitch.motion_limits
    knee_limits = knee_pitch.motion_limits
    ankle_limits = ankle_pitch.motion_limits
    ctx.check(
        "joint_axes_match_pitch_chain",
        hip_pitch.axis == (0.0, -1.0, 0.0)
        and knee_pitch.axis == (0.0, -1.0, 0.0)
        and ankle_pitch.axis == (0.0, -1.0, 0.0),
    )
    ctx.check(
        "range_limits_are_safety_first_and_plausible",
        hip_limits is not None
        and knee_limits is not None
        and ankle_limits is not None
        and hip_limits.lower is not None
        and hip_limits.upper is not None
        and knee_limits.lower == 0.0
        and knee_limits.upper is not None
        and ankle_limits.lower is not None
        and ankle_limits.upper is not None
        and hip_limits.lower < 0.0 < hip_limits.upper
        and 1.0 <= knee_limits.upper <= 1.45
        and ankle_limits.lower >= -0.4
        and ankle_limits.upper <= 0.45,
    )

    with ctx.pose({hip_pitch: 0.0, knee_pitch: 0.0, ankle_pitch: 0.0}):
        rest_foot = ctx.part_world_aabb(foot_module)
        rest_shank = ctx.part_world_aabb(shank_module)
        ctx.check(
            "rest_pose_has_groundward_chain",
            rest_foot is not None
            and rest_shank is not None
            and rest_foot[0][2] < rest_shank[0][2] - 0.06,
        )

    with ctx.pose(
        {
            hip_pitch: math.radians(25.0),
            knee_pitch: math.radians(55.0),
            ankle_pitch: math.radians(8.0),
        }
    ):
        crouch_foot = ctx.part_world_aabb(foot_module)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_crouched_pose")
        ctx.check(
            "serial_kinematics_raise_the_foot_in_crouch",
            rest_foot is not None
            and crouch_foot is not None
            and crouch_foot[1][2] > rest_foot[1][2] + 0.18
            and crouch_foot[1][0] > rest_foot[1][0] + 0.10,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
