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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robot_arm")

    base_grey = model.material("base_grey", rgba=(0.34, 0.36, 0.39, 1.0))
    arm_white = model.material("arm_white", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.65, 0.68, 0.71, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.14, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=base_grey,
        name="pedestal_skirt",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=base_grey,
        name="pedestal_drum",
    )
    base.visual(
        Box((0.185, 0.185, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=dark_trim,
        name="shoulder_plinth",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=steel,
        name="turret_seat",
    )
    base.visual(
        Box((0.09, 0.065, 0.06)),
        origin=Origin(xyz=(-0.065, 0.0, 0.16)),
        material=dark_trim,
        name="rear_service_box",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.24)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.055, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=steel,
        name="shoulder_turntable",
    )

    upper_arm_shell = section_loft(
        [
            _yz_section(0.104, 0.114, 0.028, 0.028, z_center=0.058),
            _yz_section(0.096, 0.106, 0.026, 0.150, z_center=0.058),
            _yz_section(0.082, 0.090, 0.022, 0.295, z_center=0.058),
        ]
    )
    upper_arm.visual(
        mesh_from_geometry(upper_arm_shell, "upper_arm_shell"),
        material=arm_white,
        name="arm_shell",
    )
    upper_arm.visual(
        Cylinder(radius=0.03, length=0.018),
        origin=Origin(xyz=(0.325, 0.045, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_knuckle_right",
    )
    upper_arm.visual(
        Cylinder(radius=0.03, length=0.018),
        origin=Origin(xyz=(0.325, -0.045, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_knuckle_left",
    )
    upper_arm.visual(
        Box((0.09, 0.075, 0.028)),
        origin=Origin(xyz=(0.05, 0.0, 0.085)),
        material=dark_trim,
        name="shoulder_motor_cover",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.38, 0.16, 0.14)),
        mass=6.0,
        origin=Origin(xyz=(0.19, 0.0, 0.07)),
    )

    model.articulation(
        "shoulder_rotate",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=-2.2,
            upper=2.2,
        ),
    )

    guard = model.part("guard")
    guard.visual(
        mesh_from_geometry(TorusGeometry(0.115, 0.01), "shoulder_guard_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=steel,
        name="guard_ring",
    )
    for index, (px, py) in enumerate(((0.115, 0.0), (-0.115, 0.0), (0.0, 0.115), (0.0, -0.115))):
        guard.visual(
            Cylinder(radius=0.009, length=0.06),
            origin=Origin(xyz=(px, py, 0.195)),
            material=steel,
            name=f"guard_post_{index}",
        )
    guard.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.07)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )

    model.articulation(
        "base_to_guard",
        ArticulationType.FIXED,
        parent=base,
        child=guard,
        origin=Origin(),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm_shell = section_loft(
        [
            _yz_section(0.076, 0.082, 0.020, 0.0),
            _yz_section(0.070, 0.075, 0.018, 0.140),
            _yz_section(0.062, 0.064, 0.016, 0.245),
        ]
    )
    forearm.visual(
        mesh_from_geometry(forearm_shell, "forearm_shell"),
        material=arm_white,
        name="forearm_shell",
    )
    forearm.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(xyz=(0.145, 0.0, -0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="forearm_actuator",
    )
    forearm.visual(
        Cylinder(radius=0.028, length=0.04),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wrist_collar",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.31, 0.12, 0.12)),
        mass=4.0,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.325, 0.0, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.8,
            lower=-1.15,
            upper=1.30,
        ),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.026, length=0.066),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wrist_barrel",
    )
    wrist.visual(
        Box((0.052, 0.074, 0.070)),
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        material=arm_white,
        name="wrist_head",
    )
    wrist.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.118, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.024, 0.038, 0.020)),
        origin=Origin(xyz=(0.090, 0.0, -0.040)),
        material=dark_trim,
        name="tool_mount_block",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.14, 0.08, 0.09)),
        mass=1.2,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )

    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.0,
            lower=-3.0,
            upper=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    guard = object_model.get_part("guard")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder_rotate")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.expect_gap(
        upper_arm,
        base,
        axis="z",
        positive_elem="shoulder_turntable",
        negative_elem="turret_seat",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper arm seats on shoulder turret",
    )

    rest_aabb = ctx.part_element_world_aabb(upper_arm, elem="arm_shell")
    with ctx.pose({shoulder: 1.0}):
        swung_aabb = ctx.part_element_world_aabb(upper_arm, elem="arm_shell")
    ctx.check(
        "shoulder positive rotation sweeps arm laterally",
        rest_aabb is not None and swung_aabb is not None and swung_aabb[1][1] > rest_aabb[1][1] + 0.15,
        details=f"rest={rest_aabb}, swung={swung_aabb}",
    )

    ctx.expect_gap(
        upper_arm,
        guard,
        axis="z",
        positive_elem="shoulder_turntable",
        negative_elem="guard_ring",
        min_gap=0.004,
        max_gap=0.03,
        name="guard ring sits just below the first moving stage",
    )

    rest_wrist = ctx.part_world_aabb(wrist)
    with ctx.pose({elbow: 0.9}):
        raised_wrist = ctx.part_world_aabb(wrist)
    ctx.check(
        "positive elbow raises the wrist head",
        rest_wrist is not None and raised_wrist is not None and raised_wrist[0][2] > rest_wrist[0][2] + 0.12,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )

    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="x",
        positive_elem="forearm_shell",
        negative_elem="arm_shell",
        min_gap=0.01,
        max_gap=0.06,
        name="forearm begins ahead of the upper arm nose",
    )

    wrist_origin_rest = ctx.part_world_position(wrist)
    with ctx.pose({wrist_roll: 1.8}):
        wrist_origin_rolled = ctx.part_world_position(wrist)
    ctx.check(
        "wrist roll keeps the wrist joint origin in place",
        wrist_origin_rest is not None
        and wrist_origin_rolled is not None
        and abs(wrist_origin_rolled[0] - wrist_origin_rest[0]) < 1e-6
        and abs(wrist_origin_rolled[1] - wrist_origin_rest[1]) < 1e-6
        and abs(wrist_origin_rolled[2] - wrist_origin_rest[2]) < 1e-6,
        details=f"rest={wrist_origin_rest}, rolled={wrist_origin_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
