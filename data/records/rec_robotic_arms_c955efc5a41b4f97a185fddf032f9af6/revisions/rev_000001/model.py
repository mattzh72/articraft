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
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, py, pz + z_center) for py, pz in rounded_rect_profile(width, height, radius)]


def _arm_shell_mesh(
    name: str,
    sections: list[tuple[float, float, float, float, float]],
):
    loft_sections = [
        _yz_section(
            x,
            width=width,
            height=height,
            radius=radius,
            z_center=z_center,
        )
        for x, width, height, radius, z_center in sections
    ]
    return mesh_from_geometry(section_loft(loft_sections), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robotic_arm")

    base_gray = model.material("base_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    arm_white = model.material("arm_white", rgba=(0.90, 0.91, 0.92, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    joint_black = model.material("joint_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.12, 0.35, 0.70, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((0.34, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=base_gray,
        name="floor_plinth",
    )
    pedestal_base.visual(
        Cylinder(radius=0.105, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=base_gray,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Box((0.22, 0.24, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=base_gray,
        name="shoulder_housing",
    )
    pedestal_base.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=joint_black,
        name="shoulder_bearing_ring",
    )
    pedestal_base.visual(
        Box((0.18, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, -0.08, 0.43)),
        material=dark_metal,
        name="rear_service_box",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.58)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.053, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=joint_black,
        name="shoulder_post",
    )
    upper_arm.visual(
        Cylinder(radius=0.092, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=joint_black,
        name="shoulder_turntable",
    )
    upper_arm.visual(
        _arm_shell_mesh(
            "upper_arm_shell",
            [
                (0.015, 0.15, 0.18, 0.030, 0.095),
                (0.16, 0.13, 0.16, 0.028, 0.115),
                (0.30, 0.105, 0.13, 0.024, 0.140),
            ],
        ),
        material=arm_white,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.055, 0.024, 0.105)),
        origin=Origin(xyz=(0.338, 0.050, 0.143)),
        material=arm_white,
        name="elbow_ear_left",
    )
    upper_arm.visual(
        Box((0.055, 0.024, 0.105)),
        origin=Origin(xyz=(0.338, -0.050, 0.143)),
        material=arm_white,
        name="elbow_ear_right",
    )
    upper_arm.visual(
        Cylinder(radius=0.020, length=0.082),
        origin=Origin(xyz=(0.318, 0.0, 0.195), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_blue,
        name="status_beacon",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.16, 0.24)),
        mass=24.0,
        origin=Origin(xyz=(0.18, 0.0, 0.12)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.036, length=0.076),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_black,
        name="elbow_hub",
    )
    forearm.visual(
        _arm_shell_mesh(
            "forearm_shell",
            [
                (0.036, 0.104, 0.118, 0.022, 0.000),
                (0.155, 0.092, 0.102, 0.020, 0.000),
                (0.285, 0.082, 0.092, 0.018, -0.004),
            ],
        ),
        material=arm_white,
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.11, 0.044, 0.020)),
        origin=Origin(xyz=(0.115, 0.0, 0.048)),
        material=accent_blue,
        name="cable_cover",
    )
    forearm.visual(
        Cylinder(radius=0.046, length=0.044),
        origin=Origin(xyz=(0.296, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_black,
        name="wrist_collar",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.34, 0.12, 0.14)),
        mass=16.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.040, length=0.036),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_black,
        name="roll_stub",
    )
    wrist_head.visual(
        Box((0.094, 0.080, 0.074)),
        origin=Origin(xyz=(0.067, 0.0, 0.0)),
        material=arm_white,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.032, 0.028, 0.050)),
        origin=Origin(xyz=(0.066, 0.040, 0.0)),
        material=dark_metal,
        name="side_service_pod",
    )
    wrist_head.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_black,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.026, 0.034, 0.022)),
        origin=Origin(xyz=(0.132, 0.0, -0.020)),
        material=dark_metal,
        name="pickup_pad_mount",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.15, 0.10, 0.09)),
        mass=6.5,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.338, 0.0, 0.143)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.318, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=3.5,
            lower=-3.0,
            upper=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal_base = object_model.get_part("pedestal_base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.expect_contact(
        upper_arm,
        pedestal_base,
        elem_a="shoulder_post",
        elem_b="shoulder_bearing_ring",
        name="shoulder post seats on bearing ring",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a="elbow_hub",
        elem_b="elbow_ear_left",
        name="forearm hub bears on left elbow ear",
    )
    ctx.expect_contact(
        wrist_head,
        forearm,
        elem_a="roll_stub",
        elem_b="wrist_collar",
        name="wrist stub seats in forearm collar",
    )

    rest_wrist = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder_yaw: 0.8}):
        swung_wrist = ctx.part_world_position(wrist_head)
    ctx.check(
        "positive shoulder yaw swings arm around pedestal",
        rest_wrist is not None
        and swung_wrist is not None
        and swung_wrist[1] > rest_wrist[1] + 0.32,
        details=f"rest={rest_wrist}, swung={swung_wrist}",
    )

    with ctx.pose({elbow_pitch: 0.8}):
        lifted_wrist = ctx.part_world_position(wrist_head)
    ctx.check(
        "positive elbow pitch lifts wrist head",
        rest_wrist is not None
        and lifted_wrist is not None
        and lifted_wrist[2] > rest_wrist[2] + 0.16,
        details=f"rest={rest_wrist}, lifted={lifted_wrist}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))

    rest_pod_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="side_service_pod"))
    with ctx.pose({wrist_roll: 1.2}):
        rolled_pod_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="side_service_pod"))
    ctx.check(
        "wrist roll rotates asymmetric service pod about forearm axis",
        rest_pod_center is not None
        and rolled_pod_center is not None
        and abs(rolled_pod_center[1] - rest_pod_center[1]) > 0.02
        and abs(rolled_pod_center[2] - rest_pod_center[2]) > 0.02,
        details=f"rest={rest_pod_center}, rolled={rolled_pod_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
