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
    width_y: float,
    height_z: float,
    radius: float,
    *,
    z_center: float = 0.0,
):
    safe_radius = min(radius, 0.49 * width_y, 0.49 * height_z)
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(
            width_y,
            height_z,
            safe_radius,
            corner_segments=8,
        )
    ]


def _xy_section(z: float, width_x: float, depth_y: float, radius: float):
    safe_radius = min(radius, 0.49 * width_x, 0.49 * depth_y)
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width_x,
            depth_y,
            safe_radius,
            corner_segments=8,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_robotic_arm")

    concrete = model.material("concrete", rgba=(0.64, 0.65, 0.67, 1.0))
    coated_shell = model.material("coated_shell", rgba=(0.79, 0.81, 0.83, 1.0))
    cartridge_dark = model.material("cartridge_dark", rgba=(0.29, 0.32, 0.35, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))

    pedestal_shell = _mesh(
        "pedestal_shell",
        section_loft(
            [
                _xy_section(0.14, 0.58, 0.52, 0.06),
                _xy_section(0.48, 0.52, 0.46, 0.055),
                _xy_section(0.92, 0.44, 0.40, 0.050),
                _xy_section(1.20, 0.38, 0.34, 0.040),
            ]
        ),
    )
    upper_arm_shell = _mesh(
        "upper_arm_shell",
        section_loft(
            [
                _yz_section(0.04, 0.17, 0.18, 0.032, z_center=-0.020),
                _yz_section(0.22, 0.19, 0.20, 0.034, z_center=-0.015),
                _yz_section(0.46, 0.17, 0.18, 0.030, z_center=-0.018),
                _yz_section(0.64, 0.14, 0.16, 0.026, z_center=-0.012),
            ]
        ),
    )
    forearm_shell = _mesh(
        "forearm_shell",
        section_loft(
            [
                _yz_section(0.10, 0.13, 0.14, 0.024, z_center=-0.014),
                _yz_section(0.24, 0.15, 0.16, 0.028, z_center=-0.012),
                _yz_section(0.40, 0.14, 0.15, 0.026, z_center=-0.010),
                _yz_section(0.56, 0.12, 0.13, 0.022, z_center=-0.006),
            ]
        ),
    )
    wrist_shell = _mesh(
        "wrist_shell",
        section_loft(
            [
                _yz_section(0.03, 0.13, 0.13, 0.024, z_center=-0.010),
                _yz_section(0.11, 0.13, 0.13, 0.024, z_center=-0.006),
                _yz_section(0.18, 0.11, 0.11, 0.020, z_center=0.000),
            ]
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((1.05, 1.05, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=concrete,
        name="foundation_slab",
    )
    pedestal.visual(
        pedestal_shell,
        material=coated_shell,
        name="pedestal_shell",
    )
    pedestal.visual(
        Cylinder(radius=0.28, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.215)),
        material=cartridge_dark,
        name="drip_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.22, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.245)),
        material=cartridge_dark,
        name="yaw_bearing_cap",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            pedestal.visual(
                Cylinder(radius=0.018, length=0.018),
                origin=Origin(xyz=(0.22 * x_sign, 0.18 * y_sign, 0.149)),
                material=stainless,
                name=f"anchor_cap_{int((x_sign + 1.0) * 0.5)}_{int((y_sign + 1.0) * 0.5)}",
            )
    pedestal.inertial = Inertial.from_geometry(
        Box((1.05, 1.05, 1.30)),
        mass=640.0,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    shoulder_carriage = model.part("shoulder_carriage")
    shoulder_carriage.visual(
        Cylinder(radius=0.24, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=cartridge_dark,
        name="turntable_flange",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.20, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=gasket_black,
        name="yaw_seal_band",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.19, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=coated_shell,
        name="yaw_drum",
    )
    shoulder_carriage.visual(
        Box((0.18, 0.18, 0.42)),
        origin=Origin(xyz=(-0.04, 0.0, 0.46)),
        material=coated_shell,
        name="shoulder_backbone",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.14, length=0.07),
        origin=Origin(xyz=(0.16, 0.145, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="shoulder_cartridge_left",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.14, length=0.07),
        origin=Origin(xyz=(0.16, -0.145, 0.56), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="shoulder_cartridge_right",
    )
    shoulder_carriage.visual(
        Box((0.18, 0.22, 0.08)),
        origin=Origin(xyz=(0.10, 0.0, 0.705)),
        material=coated_shell,
        name="shoulder_bridge",
    )
    shoulder_carriage.visual(
        Box((0.30, 0.26, 0.03)),
        origin=Origin(xyz=(0.09, 0.0, 0.76)),
        material=coated_shell,
        name="shoulder_rain_visor",
    )
    shoulder_carriage.inertial = Inertial.from_geometry(
        Box((0.58, 0.34, 0.80)),
        mass=180.0,
        origin=Origin(xyz=(0.08, 0.0, 0.40)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.095, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Cylinder(radius=0.097, length=0.03),
        origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="shoulder_seal_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.097, length=0.03),
        origin=Origin(xyz=(0.0, -0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="shoulder_seal_right",
    )
    upper_arm.visual(
        Box((0.60, 0.22, 0.16)),
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        material=coated_shell,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.46, 0.20, 0.018)),
        origin=Origin(xyz=(0.34, 0.0, 0.089)),
        material=coated_shell,
        name="upper_arm_drip_cap",
    )
    upper_arm.visual(
        Box((0.12, 0.22, 0.10)),
        origin=Origin(xyz=(0.64, 0.0, 0.0)),
        material=coated_shell,
        name="elbow_riser",
    )
    upper_arm.visual(
        Cylinder(radius=0.11, length=0.07),
        origin=Origin(xyz=(0.82, 0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="elbow_cartridge_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.11, length=0.07),
        origin=Origin(xyz=(0.82, -0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="elbow_cartridge_right",
    )
    upper_arm.visual(
        Box((0.12, 0.07, 0.11)),
        origin=Origin(xyz=(0.70, 0.145, 0.0)),
        material=coated_shell,
        name="elbow_fork_left",
    )
    upper_arm.visual(
        Box((0.12, 0.07, 0.11)),
        origin=Origin(xyz=(0.70, -0.145, 0.0)),
        material=coated_shell,
        name="elbow_fork_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.86, 0.28, 0.24)),
        mass=120.0,
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.085, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="elbow_barrel",
    )
    forearm.visual(
        Cylinder(radius=0.087, length=0.03),
        origin=Origin(xyz=(0.0, 0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="elbow_seal_left",
    )
    forearm.visual(
        Cylinder(radius=0.087, length=0.03),
        origin=Origin(xyz=(0.0, -0.075, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="elbow_seal_right",
    )
    forearm.visual(
        Box((0.56, 0.20, 0.14)),
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        material=coated_shell,
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.40, 0.18, 0.018)),
        origin=Origin(xyz=(0.31, 0.0, 0.079)),
        material=coated_shell,
        name="forearm_drip_cap",
    )
    forearm.visual(
        Box((0.08, 0.08, 0.08)),
        origin=Origin(xyz=(0.56, 0.0, 0.09)),
        material=coated_shell,
        name="wrist_riser",
    )
    forearm.visual(
        Cylinder(radius=0.095, length=0.06),
        origin=Origin(xyz=(0.70, 0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="wrist_cartridge_left",
    )
    forearm.visual(
        Cylinder(radius=0.095, length=0.06),
        origin=Origin(xyz=(0.70, -0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="wrist_cartridge_right",
    )
    forearm.visual(
        Box((0.10, 0.06, 0.10)),
        origin=Origin(xyz=(0.65, 0.12, 0.0)),
        material=coated_shell,
        name="wrist_fork_left",
    )
    forearm.visual(
        Box((0.10, 0.06, 0.10)),
        origin=Origin(xyz=(0.65, -0.12, 0.0)),
        material=coated_shell,
        name="wrist_fork_right",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.74, 0.22, 0.20)),
        mass=82.0,
        origin=Origin(xyz=(0.37, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.07, length=0.18),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge_dark,
        name="wrist_barrel",
    )
    wrist_head.visual(
        Cylinder(radius=0.071, length=0.025),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="wrist_seal_left",
    )
    wrist_head.visual(
        Cylinder(radius=0.071, length=0.025),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_black,
        name="wrist_seal_right",
    )
    wrist_head.visual(
        Box((0.16, 0.12, 0.12)),
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
        material=coated_shell,
        name="wrist_shell",
    )
    wrist_head.visual(
        Box((0.12, 0.12, 0.016)),
        origin=Origin(xyz=(0.14, 0.0, 0.068)),
        material=coated_shell,
        name="wrist_drip_cap",
    )
    wrist_head.visual(
        Box((0.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        material=coated_shell,
        name="tool_adapter",
    )
    wrist_head.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cartridge_dark,
        name="tool_spindle",
    )
    wrist_head.visual(
        Box((0.02, 0.12, 0.12)),
        origin=Origin(xyz=(0.37, 0.0, 0.0)),
        material=stainless,
        name="tool_face_plate",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.18)),
        mass=36.0,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=shoulder_carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.275)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2800.0, velocity=0.70),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.16, 0.0, 0.56)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.85,
            lower=math.radians(-25.0),
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.82, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.95,
            lower=math.radians(-10.0),
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.70, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.20,
            lower=math.radians(-95.0),
            upper=math.radians(90.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    shoulder_carriage = object_model.get_part("shoulder_carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    pedestal_yaw = object_model.get_articulation("pedestal_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

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

    ctx.expect_contact(pedestal, shoulder_carriage, name="yaw_turntable_is_seated")
    ctx.expect_contact(shoulder_carriage, upper_arm, name="shoulder_cartridge_is_supported")
    ctx.expect_contact(upper_arm, forearm, name="elbow_cartridge_is_supported")
    ctx.expect_contact(forearm, wrist_head, name="wrist_cartridge_is_supported")

    ctx.check(
        "joint_axes_and_offsets_are_readable",
        pedestal_yaw.axis == (0.0, 0.0, 1.0)
        and shoulder_pitch.axis == (0.0, -1.0, 0.0)
        and elbow_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_pitch.axis == (0.0, -1.0, 0.0)
        and shoulder_pitch.origin.xyz[0] > 0.10
        and shoulder_pitch.origin.xyz[2] > 0.45
        and elbow_pitch.origin.xyz[0] > 0.75
        and wrist_pitch.origin.xyz[0] > 0.65,
        details="Expected a vertical pedestal yaw followed by shoulder/elbow/wrist pitch joints with forward joint offsets.",
    )

    rest_wrist = ctx.part_world_position(wrist_head)
    with ctx.pose(pedestal_yaw=0.45):
        yawed_wrist = ctx.part_world_position(wrist_head)
    with ctx.pose(shoulder_pitch=0.55):
        raised_wrist = ctx.part_world_position(wrist_head)
    with ctx.pose(elbow_pitch=0.90):
        folded_wrist = ctx.part_world_position(wrist_head)

    ctx.check(
        "positive_yaw_swings_arm_around_pedestal",
        rest_wrist is not None
        and yawed_wrist is not None
        and yawed_wrist[1] > rest_wrist[1] + 0.45,
        details="Positive pedestal yaw should sweep the chain toward +Y.",
    )
    ctx.check(
        "positive_shoulder_raises_reach",
        rest_wrist is not None
        and raised_wrist is not None
        and raised_wrist[2] > rest_wrist[2] + 0.45,
        details="Positive shoulder pitch should lift the distal chain upward.",
    )
    ctx.check(
        "positive_elbow_folds_chain_upward",
        rest_wrist is not None
        and folded_wrist is not None
        and folded_wrist[2] > rest_wrist[2] + 0.12
        and folded_wrist[0] < rest_wrist[0] - 0.10,
        details="Positive elbow pitch should fold the forearm upward and back toward the shoulder.",
    )

    rest_tool_aabb = ctx.part_element_world_aabb(wrist_head, elem="tool_face_plate")
    with ctx.pose(wrist_pitch=0.70):
        raised_tool_aabb = ctx.part_element_world_aabb(wrist_head, elem="tool_face_plate")
    ctx.check(
        "positive_wrist_tips_tool_face_upward",
        rest_tool_aabb is not None
        and raised_tool_aabb is not None
        and 0.5 * (raised_tool_aabb[0][2] + raised_tool_aabb[1][2])
        > 0.5 * (rest_tool_aabb[0][2] + rest_tool_aabb[1][2]) + 0.03,
        details="Positive wrist pitch should raise the weather-sealed tool face.",
    )

    with ctx.pose(shoulder_pitch=0.45, elbow_pitch=0.95, wrist_pitch=-0.35):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_compact_work_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
