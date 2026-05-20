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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robotic_arm")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.88, 0.45, 0.12, 1.0))
    joint_dark = model.material("joint_dark", rgba=(0.26, 0.28, 0.30, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    cover_black = model.material("cover_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.19, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_dark,
        name="base_disk",
    )
    pedestal_base.visual(
        Cylinder(radius=0.13, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=joint_dark,
        name="base_shoulder",
    )
    pedestal_base.visual(
        Cylinder(radius=0.105, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=base_dark,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.125, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=joint_dark,
        name="top_cap",
    )
    pedestal_base.visual(
        Box((0.10, 0.18, 0.16)),
        origin=Origin(xyz=(-0.015, 0.0, 0.12)),
        material=cover_black,
        name="service_cover",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 0.36)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    shoulder_assembly = model.part("shoulder_assembly")
    shoulder_assembly.visual(
        Cylinder(radius=0.12, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=joint_dark,
        name="turntable_ring",
    )
    shoulder_assembly.visual(
        Box((0.16, 0.22, 0.18)),
        origin=Origin(xyz=(0.08, 0.0, 0.13)),
        material=joint_dark,
        name="shoulder_housing",
    )
    shoulder_assembly.visual(
        Box((0.10, 0.18, 0.10)),
        origin=Origin(xyz=(0.18, 0.0, 0.17)),
        material=cover_black,
        name="motor_cover",
    )
    upper_arm_shell = section_loft(
        [
            _yz_section(0.08, width=0.18, height=0.16, radius=0.030, z_center=0.17),
            _yz_section(0.24, width=0.15, height=0.13, radius=0.026, z_center=0.16),
            _yz_section(0.37, width=0.11, height=0.10, radius=0.020, z_center=0.145),
        ]
    )
    shoulder_assembly.visual(
        _mesh("upper_arm_shell", upper_arm_shell),
        material=arm_orange,
        name="upper_arm_shell",
    )
    shoulder_assembly.visual(
        Box((0.06, 0.04, 0.11)),
        origin=Origin(xyz=(0.40, 0.055, 0.145)),
        material=joint_dark,
        name="elbow_fork_left",
    )
    shoulder_assembly.visual(
        Box((0.06, 0.04, 0.11)),
        origin=Origin(xyz=(0.40, -0.055, 0.145)),
        material=joint_dark,
        name="elbow_fork_right",
    )
    shoulder_assembly.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(xyz=(0.43, 0.055, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_cap_left",
    )
    shoulder_assembly.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(xyz=(0.43, -0.055, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_cap_right",
    )
    shoulder_assembly.inertial = Inertial.from_geometry(
        Box((0.54, 0.24, 0.24)),
        mass=48.0,
        origin=Origin(xyz=(0.22, 0.0, 0.14)),
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        Cylinder(radius=0.036, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_knuckle",
    )
    forearm_link.visual(
        Box((0.10, 0.09, 0.10)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=joint_dark,
        name="forearm_shoulder",
    )
    forearm_shell = section_loft(
        [
            _yz_section(0.04, width=0.11, height=0.11, radius=0.022, z_center=0.0),
            _yz_section(0.24, width=0.10, height=0.09, radius=0.018, z_center=-0.004),
            _yz_section(0.40, width=0.08, height=0.08, radius=0.016, z_center=0.0),
        ]
    )
    forearm_link.visual(
        _mesh("forearm_shell", forearm_shell),
        material=arm_orange,
        name="forearm_shell",
    )
    forearm_link.visual(
        Box((0.18, 0.05, 0.06)),
        origin=Origin(xyz=(0.14, 0.0, -0.038)),
        material=joint_dark,
        name="forearm_rib",
    )
    forearm_link.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.36, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_dark,
        name="wrist_sleeve",
    )
    forearm_link.inertial = Inertial.from_geometry(
        Box((0.44, 0.12, 0.14)),
        mass=31.0,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.034, length=0.05),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="roll_collar",
    )
    wrist_head.visual(
        Box((0.10, 0.12, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=joint_dark,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.05, 0.05, 0.022)),
        origin=Origin(xyz=(0.07, 0.0, 0.056)),
        material=cover_black,
        name="roll_key",
    )
    wrist_head.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.134, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.012, 0.070, 0.070)),
        origin=Origin(xyz=(0.144, 0.0, 0.0)),
        material=machined_steel,
        name="face_plate",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.14)),
        mass=9.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=shoulder_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=1.5,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_assembly,
        child=forearm_link,
        origin=Origin(xyz=(0.43, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=wrist_head,
        origin=Origin(xyz=(0.40, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.0,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    shoulder_assembly = object_model.get_part("shoulder_assembly")
    wrist_head = object_model.get_part("wrist_head")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    with ctx.pose({shoulder_yaw: 0.0, elbow_pitch: 0.0, wrist_roll: 0.0}):
        ctx.expect_gap(
            shoulder_assembly,
            pedestal_base,
            axis="z",
            positive_elem="turntable_ring",
            negative_elem="top_cap",
            max_gap=0.001,
            max_penetration=1e-6,
            name="shoulder turntable seats on the pedestal cap",
        )
        ctx.expect_origin_gap(
            wrist_head,
            pedestal_base,
            axis="x",
            min_gap=0.78,
            name="rest pose projects the wrist well forward of the pedestal",
        )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder_yaw: math.radians(38.0), elbow_pitch: 0.0, wrist_roll: 0.0}):
        yawed_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder joint yaws the arm about the vertical axis",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and yawed_wrist_pos[1] > rest_wrist_pos[1] + 0.35
        and abs(yawed_wrist_pos[2] - rest_wrist_pos[2]) < 0.02,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({shoulder_yaw: 0.0, elbow_pitch: 1.0, wrist_roll: 0.0}):
        lifted_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow joint raises the forearm in positive motion",
        rest_wrist_pos is not None
        and lifted_wrist_pos is not None
        and lifted_wrist_pos[2] > rest_wrist_pos[2] + 0.25
        and lifted_wrist_pos[0] < rest_wrist_pos[0] - 0.10,
        details=f"rest={rest_wrist_pos}, lifted={lifted_wrist_pos}",
    )

    with ctx.pose({wrist_roll: 0.0}):
        key_rest = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="roll_key"))
    with ctx.pose({wrist_roll: math.pi / 2.0}):
        key_rolled = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="roll_key"))
    ctx.check(
        "wrist joint rolls the head around the forearm axis",
        key_rest is not None
        and key_rolled is not None
        and abs(key_rolled[0] - key_rest[0]) < 0.005
        and key_rolled[1] < key_rest[1] - 0.035
        and key_rolled[2] < key_rest[2] - 0.020,
        details=f"rest={key_rest}, rolled={key_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
