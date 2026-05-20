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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_and_place_robot_arm")

    def yz_section(
        x: float,
        *,
        z_center: float,
        width_y: float,
        height_z: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z_center + z)
            for z, y in rounded_rect_profile(height_z, width_y, radius, corner_segments=8)
        ]

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    base_gray = model.material("base_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.86, 0.42, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.08, 0.09, 0.10, 1.0))
    sensor_blue = model.material("sensor_blue", rgba=(0.16, 0.42, 0.54, 1.0))

    upper_arm_shell = save_mesh(
        "upper_arm_shell",
        section_loft(
            [
                yz_section(0.02, z_center=0.06, width_y=0.12, height_z=0.12, radius=0.026),
                yz_section(0.12, z_center=0.10, width_y=0.14, height_z=0.18, radius=0.030),
                yz_section(0.28, z_center=0.16, width_y=0.12, height_z=0.15, radius=0.028),
                yz_section(0.35, z_center=0.14, width_y=0.11, height_z=0.09, radius=0.020),
            ]
        ),
    )
    forearm_shell = save_mesh(
        "forearm_shell",
        section_loft(
            [
                yz_section(0.04, z_center=0.00, width_y=0.072, height_z=0.090, radius=0.018),
                yz_section(0.14, z_center=0.02, width_y=0.095, height_z=0.115, radius=0.024),
                yz_section(0.27, z_center=0.01, width_y=0.09, height_z=0.10, radius=0.022),
                yz_section(0.32, z_center=0.00, width_y=0.08, height_z=0.09, radius=0.020),
            ]
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((0.52, 0.46, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_gray,
        name="floor_plinth",
    )
    pedestal_base.visual(
        Box((0.30, 0.26, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_metal,
        name="drive_cabinet",
    )
    pedestal_base.visual(
        Cylinder(radius=0.10, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=base_gray,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Box((0.24, 0.18, 0.12)),
        origin=Origin(xyz=(0.10, 0.0, 0.56)),
        material=base_gray,
        name="shoulder_bridge",
    )
    pedestal_base.visual(
        Box((0.14, 0.18, 0.18)),
        origin=Origin(xyz=(0.04, 0.0, 0.47)),
        material=base_gray,
        name="shoulder_web",
    )
    pedestal_base.visual(
        Cylinder(radius=0.085, length=0.14),
        origin=Origin(xyz=(0.16, 0.0, 0.63)),
        material=machined_steel,
        name="shoulder_bearing_housing",
    )
    pedestal_base.visual(
        Cylinder(radius=0.055, length=0.07),
        origin=Origin(xyz=(0.06, 0.0, 0.67)),
        material=dark_metal,
        name="bearing_cap",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.52, 0.46, 0.79)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.07, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machined_steel,
        name="shoulder_rotor",
    )
    upper_arm.visual(
        upper_arm_shell,
        material=arm_orange,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.12, 0.04, 0.11)),
        origin=Origin(xyz=(0.40, 0.065, 0.14)),
        material=arm_orange,
        name="elbow_clevis_right",
    )
    upper_arm.visual(
        Box((0.12, 0.04, 0.11)),
        origin=Origin(xyz=(0.40, -0.065, 0.14)),
        material=arm_orange,
        name="elbow_clevis_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.022, length=0.16),
        origin=Origin(xyz=(0.12, 0.0, 0.17), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="cable_raceway",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.48, 0.18, 0.23)),
        mass=32.0,
        origin=Origin(xyz=(0.24, 0.0, 0.12)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.045, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_knuckle",
    )
    forearm.visual(
        forearm_shell,
        material=arm_orange,
        name="forearm_shell",
    )
    forearm.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="wrist_drive_housing",
    )
    forearm.visual(
        Box((0.13, 0.05, 0.04)),
        origin=Origin(xyz=(0.20, 0.0, 0.055)),
        material=dark_metal,
        name="tool_cable_cover",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.40, 0.12, 0.15)),
        mass=18.0,
        origin=Origin(xyz=(0.20, 0.0, 0.02)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.040, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="roll_motor",
    )
    wrist_head.visual(
        Box((0.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=dark_metal,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.04, 0.035, 0.03)),
        origin=Origin(xyz=(0.11, 0.045, 0.020)),
        material=sensor_blue,
        name="sensor_pod",
    )
    wrist_head.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.162, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_polymer,
        name="tool_flange",
    )
    wrist_head.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.195, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_polymer,
        name="pickup_nozzle_mount",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.22, 0.12, 0.10)),
        mass=6.0,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=upper_arm,
        origin=Origin(xyz=(0.16, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=1.6,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.43, 0.0, 0.14)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-0.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal_base = object_model.get_part("pedestal_base")
    upper_arm = object_model.get_part("upper_arm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.expect_origin_gap(
        upper_arm,
        pedestal_base,
        axis="x",
        min_gap=0.12,
        max_gap=0.20,
        name="moving chain starts offset to one side of the fixed pedestal",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 1.15}):
        bent_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow joint raises the forearm and wrist",
        rest_wrist_pos is not None
        and bent_wrist_pos is not None
        and bent_wrist_pos[2] > rest_wrist_pos[2] + 0.18,
        details=f"rest={rest_wrist_pos}, bent={bent_wrist_pos}",
    )

    with ctx.pose({shoulder_yaw: 1.0}):
        swung_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder joint sweeps the arm around the vertical axis",
        rest_wrist_pos is not None
        and swung_wrist_pos is not None
        and abs(swung_wrist_pos[1] - rest_wrist_pos[1]) > 0.30,
        details=f"rest={rest_wrist_pos}, swung={swung_wrist_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    rest_sensor = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="sensor_pod"))
    with ctx.pose({wrist_roll: math.pi / 2.0}):
        rolled_sensor = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="sensor_pod"))
    ctx.check(
        "wrist joint rolls the asymmetric wrist head about the forearm axis",
        rest_sensor is not None
        and rolled_sensor is not None
        and abs(rolled_sensor[2] - rest_sensor[2]) > 0.025,
        details=f"rest_sensor={rest_sensor}, rolled_sensor={rolled_sensor}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
