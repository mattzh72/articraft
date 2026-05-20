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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robot_arm")

    base_paint = model.material("base_paint", rgba=(0.28, 0.30, 0.32, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.83, 0.85, 0.87, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.69, 0.72, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=base_paint,
        name="foot_plate",
    )
    base.visual(
        Cylinder(radius=0.10, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=base_paint,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.135, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=graphite,
        name="yaw_bearing_seat",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.229)),
        material=steel,
        name="yaw_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.26),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        _mesh(
            "shoulder_housing_shell",
            section_loft(
                [
                    _yz_section(-0.075, 0.155, 0.120, 0.028),
                    _yz_section(0.000, 0.175, 0.138, 0.032),
                    _yz_section(0.080, 0.148, 0.112, 0.026),
                ]
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=arm_paint,
        name="housing_shell",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=graphite,
        name="yaw_cartridge",
    )
    shoulder_housing.visual(
        Box((0.078, 0.024, 0.086)),
        origin=Origin(xyz=(0.096, 0.044, 0.054)),
        material=graphite,
        name="left_cheek_plate",
    )
    shoulder_housing.visual(
        Box((0.078, 0.024, 0.086)),
        origin=Origin(xyz=(0.096, -0.044, 0.054)),
        material=graphite,
        name="right_cheek_plate",
    )
    shoulder_housing.visual(
        Box((0.06, 0.11, 0.05)),
        origin=Origin(xyz=(0.09, 0.0, 0.03)),
        material=graphite,
        name="front_mount_block",
    )
    shoulder_housing.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.18)),
        mass=7.5,
        origin=Origin(xyz=(0.01, 0.0, 0.04)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _mesh(
            "upper_arm_shell",
            section_loft(
                [
                    _yz_section(0.000, 0.110, 0.090, 0.024),
                    _yz_section(0.130, 0.108, 0.098, 0.024),
                    _yz_section(0.265, 0.094, 0.084, 0.022),
                    _yz_section(0.340, 0.086, 0.078, 0.020),
                ]
            ),
        ),
        material=arm_paint,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="shoulder_hinge_barrel",
    )
    upper_arm.visual(
        Box((0.11, 0.038, 0.028)),
        origin=Origin(xyz=(0.055, 0.0, 0.040)),
        material=sensor_black,
        name="cable_cover",
    )
    upper_arm.visual(
        Box((0.046, 0.022, 0.092)),
        origin=Origin(xyz=(0.319, 0.044, 0.0)),
        material=graphite,
        name="left_elbow_cheek",
    )
    upper_arm.visual(
        Box((0.046, 0.022, 0.092)),
        origin=Origin(xyz=(0.319, -0.044, 0.0)),
        material=graphite,
        name="right_elbow_cheek",
    )
    upper_arm.visual(
        Box((0.040, 0.068, 0.038)),
        origin=Origin(xyz=(0.308, 0.0, 0.0)),
        material=graphite,
        name="elbow_nose_block",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.34, 0.11, 0.10)),
        mass=6.2,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.032, length=0.084),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_barrel",
    )
    forearm.visual(
        _mesh(
            "forearm_shell",
            section_loft(
                [
                    _yz_section(0.026, 0.082, 0.074, 0.018),
                    _yz_section(0.120, 0.080, 0.076, 0.018),
                    _yz_section(0.220, 0.074, 0.070, 0.017),
                    _yz_section(0.292, 0.068, 0.064, 0.015),
                ]
            ),
        ),
        material=arm_paint,
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.116, 0.030, 0.020)),
        origin=Origin(xyz=(0.092, 0.0, 0.034)),
        material=sensor_black,
        name="forearm_cable_channel",
    )
    forearm.visual(
        Box((0.090, 0.060, 0.048)),
        origin=Origin(xyz=(0.247, 0.0, 0.0)),
        material=graphite,
        name="wrist_mount_block",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.30, 0.09, 0.08)),
        mass=4.8,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="roll_shaft",
    )
    wrist_head.visual(
        _mesh(
            "wrist_head_shell",
            section_loft(
                [
                    _yz_section(0.040, 0.074, 0.078, 0.018),
                    _yz_section(0.086, 0.076, 0.082, 0.020),
                    _yz_section(0.126, 0.056, 0.062, 0.014),
                ]
            ),
        ),
        material=arm_paint,
        name="wrist_shell",
    )
    wrist_head.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.133, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.034, 0.028, 0.018)),
        origin=Origin(xyz=(0.084, 0.0, 0.044)),
        material=sensor_black,
        name="sensor_pod",
    )
    wrist_head.visual(
        Box((0.016, 0.042, 0.030)),
        origin=Origin(xyz=(0.136, 0.0, 0.0)),
        material=steel,
        name="face_plate",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.15, 0.08, 0.09)),
        mass=2.2,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.4, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "shoulder_to_upper_arm_mount",
        ArticulationType.FIXED,
        parent=shoulder_housing,
        child=upper_arm,
        origin=Origin(xyz=(0.172, 0.0, 0.054)),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.342, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-1.30, upper=1.15),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.292, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.4, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder = object_model.get_articulation("base_to_shoulder")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist = object_model.get_articulation("forearm_to_wrist")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.expect_gap(
        shoulder_housing,
        base,
        axis="z",
        max_gap=0.012,
        max_penetration=0.0,
        name="shoulder housing seats tightly on pedestal",
    )
    ctx.expect_overlap(
        shoulder_housing,
        base,
        axes="xy",
        min_overlap=0.10,
        name="shoulder housing remains centered over pedestal",
    )
    ctx.expect_overlap(
        shoulder_housing,
        upper_arm,
        axes="yz",
        min_overlap=0.10,
        name="shoulder package stays densely nested",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="yz",
        min_overlap=0.08,
        name="elbow barrel stays tucked within the upper arm yoke",
    )
    ctx.expect_overlap(
        forearm,
        wrist_head,
        axes="yz",
        min_overlap=0.07,
        name="wrist head remains compact on the forearm axis",
    )

    rest_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder: 0.6}):
        turned_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings arm around vertical axis",
        rest_pos is not None and turned_pos is not None and turned_pos[1] > rest_pos[1] + 0.35,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow: 1.0}):
        lifted_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch lifts the forearm and wrist",
        rest_wrist_pos is not None
        and lifted_wrist_pos is not None
        and lifted_wrist_pos[2] > rest_wrist_pos[2] + 0.20
        and lifted_wrist_pos[0] < rest_wrist_pos[0] - 0.10,
        details=f"rest={rest_wrist_pos}, lifted={lifted_wrist_pos}",
    )

    rest_sensor_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="sensor_pod"))
    with ctx.pose({wrist: pi / 2.0}):
        rolled_sensor_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="sensor_pod"))
    ctx.check(
        "wrist roll spins the compact head around the forearm axis",
        rest_sensor_center is not None
        and rolled_sensor_center is not None
        and abs(rolled_sensor_center[1]) > abs(rest_sensor_center[1]) + 0.03
        and rolled_sensor_center[2] < rest_sensor_center[2] - 0.03,
        details=f"rest_sensor={rest_sensor_center}, rolled_sensor={rolled_sensor_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
