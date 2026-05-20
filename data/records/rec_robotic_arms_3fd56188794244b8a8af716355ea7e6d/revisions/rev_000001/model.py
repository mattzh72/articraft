from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, radians, sin

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


def _xy_section(width_x: float, depth_y: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width_x, depth_y, radius, corner_segments=8)]


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    *,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_offset) for z, y in rounded_rect_profile(height_z, width_y, radius, corner_segments=8)]


def _add_x_bolt_grid(
    part,
    *,
    prefix: str,
    x: float,
    ys: tuple[float, ...],
    zs: tuple[float, ...],
    radius: float,
    length: float,
    material,
) -> None:
    for y_index, y in enumerate(ys):
        for z_index, z in enumerate(zs):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=material,
                name=f"{prefix}_{y_index}_{z_index}",
            )


def _add_y_bolt_grid(
    part,
    *,
    prefix: str,
    y: float,
    xs: tuple[float, ...],
    zs: tuple[float, ...],
    radius: float,
    length: float,
    material,
) -> None:
    for x_index, x in enumerate(xs):
        for z_index, z in enumerate(zs):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{prefix}_{x_index}_{z_index}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_robotic_arm")

    enamel_green = model.material("enamel_green", rgba=(0.38, 0.49, 0.42, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    panel_cream = model.material("panel_cream", rgba=(0.79, 0.77, 0.71, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    warning_red = model.material("warning_red", rgba=(0.62, 0.19, 0.16, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.84, 0.74, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_iron,
        name="base_plinth",
    )
    pedestal_shell = section_loft(
        [
            _xy_section(0.56, 0.50, 0.06, 0.10),
            _xy_section(0.50, 0.46, 0.06, 0.46),
            _xy_section(0.42, 0.40, 0.05, 0.84),
        ]
    )
    pedestal.visual(_mesh("pedestal_shell", pedestal_shell), material=enamel_green, name="cabinet_shell")
    pedestal.visual(
        Cylinder(radius=0.24, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=machinery_gray,
        name="slew_ring",
    )
    pedestal.visual(
        Box((0.018, 0.30, 0.42)),
        origin=Origin(xyz=(0.275, 0.0, 0.42)),
        material=panel_cream,
        name="front_service_hatch",
    )
    pedestal.visual(
        Box((0.22, 0.018, 0.32)),
        origin=Origin(xyz=(-0.05, 0.245, 0.49)),
        material=panel_cream,
        name="side_service_hatch",
    )
    pedestal.visual(
        Box((0.08, 0.18, 0.22)),
        origin=Origin(xyz=(0.16, 0.0, 0.73)),
        material=machinery_gray,
        name="front_reinforcement",
    )
    pedestal.visual(
        Box((0.08, 0.18, 0.22)),
        origin=Origin(xyz=(-0.16, 0.0, 0.73)),
        material=machinery_gray,
        name="rear_reinforcement",
    )
    pedestal.visual(
        Box((0.18, 0.08, 0.22)),
        origin=Origin(xyz=(0.0, 0.16, 0.73)),
        material=machinery_gray,
        name="left_reinforcement",
    )
    pedestal.visual(
        Box((0.18, 0.08, 0.22)),
        origin=Origin(xyz=(0.0, -0.16, 0.73)),
        material=machinery_gray,
        name="right_reinforcement",
    )
    _add_x_bolt_grid(
        pedestal,
        prefix="front_hatch_bolt",
        x=0.282,
        ys=(-0.11, 0.11),
        zs=(0.29, 0.55),
        radius=0.008,
        length=0.012,
        material=steel,
    )
    _add_y_bolt_grid(
        pedestal,
        prefix="side_hatch_bolt",
        y=0.252,
        xs=(-0.11, 0.01),
        zs=(0.39, 0.59),
        radius=0.008,
        length=0.012,
        material=steel,
    )
    for index in range(8):
        angle = 2.0 * pi * index / 8.0
        pedestal.visual(
            Cylinder(radius=0.011, length=0.018),
            origin=Origin(xyz=(0.225 * cos(angle), 0.225 * sin(angle), 0.949)),
            material=steel,
            name=f"slew_bolt_{index}",
        )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.84, 0.74, 0.94)),
        mass=440.0,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_iron,
        name="turntable_drum",
    )
    turret.visual(
        Box((0.28, 0.26, 0.14)),
        origin=Origin(xyz=(-0.02, 0.0, 0.20)),
        material=enamel_green,
        name="shoulder_gearbox",
    )
    turret.visual(
        Box((0.22, 0.24, 0.16)),
        origin=Origin(xyz=(-0.15, 0.0, 0.24)),
        material=machinery_gray,
        name="rear_drive_pack",
    )
    turret.visual(
        Box((0.20, 0.06, 0.34)),
        origin=Origin(xyz=(0.12, 0.17, 0.38)),
        material=enamel_green,
        name="left_shoulder_cheek",
    )
    turret.visual(
        Box((0.20, 0.06, 0.34)),
        origin=Origin(xyz=(0.12, -0.17, 0.38)),
        material=enamel_green,
        name="right_shoulder_cheek",
    )
    turret.visual(
        Box((0.16, 0.28, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 0.54)),
        material=machinery_gray,
        name="shoulder_bridge",
    )
    turret.visual(
        Box((0.18, 0.04, 0.22)),
        origin=Origin(xyz=(0.06, 0.135, 0.31)),
        material=machinery_gray,
        name="left_shoulder_web",
    )
    turret.visual(
        Box((0.18, 0.04, 0.22)),
        origin=Origin(xyz=(0.06, -0.135, 0.31)),
        material=machinery_gray,
        name="right_shoulder_web",
    )
    turret.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(xyz=(0.12, 0.12, 0.38), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="left_shoulder_cap",
    )
    turret.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(xyz=(0.12, -0.12, 0.38), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="right_shoulder_cap",
    )
    turret.visual(
        Box((0.20, 0.10, 0.18)),
        origin=Origin(xyz=(-0.02, -0.24, 0.31)),
        material=warning_red,
        name="slew_motor_box",
    )
    turret.visual(
        Box((0.12, 0.012, 0.12)),
        origin=Origin(xyz=(-0.03, -0.292, 0.31)),
        material=panel_cream,
        name="motor_service_cover",
    )
    _add_y_bolt_grid(
        turret,
        prefix="motor_cover_bolt",
        y=-0.298,
        xs=(-0.07, 0.01),
        zs=(0.26, 0.36),
        radius=0.007,
        length=0.012,
        material=steel,
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.56, 0.60, 0.70)),
        mass=170.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.09, length=0.18),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="shoulder_cartridge",
    )
    upper_arm_body = section_loft(
        [
            _yz_section(0.16, 0.22, 0.04, 0.02, z_offset=0.00),
            _yz_section(0.18, 0.20, 0.04, 0.28, z_offset=0.02),
            _yz_section(0.16, 0.16, 0.03, 0.56, z_offset=-0.02),
        ]
    )
    upper_arm.visual(_mesh("upper_arm_body", upper_arm_body), material=enamel_green, name="arm_body")
    upper_arm.visual(
        Box((0.22, 0.12, 0.06)),
        origin=Origin(xyz=(0.22, 0.0, -0.10)),
        material=machinery_gray,
        name="underslung_rib",
    )
    upper_arm.visual(
        Box((0.10, 0.18, 0.04)),
        origin=Origin(xyz=(0.60, 0.0, 0.105)),
        material=machinery_gray,
        name="elbow_adapter_block",
    )
    upper_arm.visual(
        Cylinder(radius=0.08, length=0.06),
        origin=Origin(xyz=(0.64, 0.11, -0.02), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="left_elbow_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.08, length=0.06),
        origin=Origin(xyz=(0.64, -0.11, -0.02), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="right_elbow_cap",
    )
    upper_arm.visual(
        Box((0.10, 0.05, 0.12)),
        origin=Origin(xyz=(0.60, 0.11, 0.055)),
        material=machinery_gray,
        name="left_elbow_web",
    )
    upper_arm.visual(
        Box((0.10, 0.05, 0.12)),
        origin=Origin(xyz=(0.60, -0.11, 0.055)),
        material=machinery_gray,
        name="right_elbow_web",
    )
    upper_arm.visual(
        Box((0.26, 0.012, 0.12)),
        origin=Origin(xyz=(0.26, 0.089, 0.02)),
        material=panel_cream,
        name="upper_arm_service_hatch",
    )
    _add_y_bolt_grid(
        upper_arm,
        prefix="upper_arm_hatch_bolt",
        y=0.096,
        xs=(0.17, 0.35),
        zs=(-0.02, 0.06),
        radius=0.006,
        length=0.014,
        material=steel,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.72, 0.20, 0.28)),
        mass=95.0,
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.075, length=0.16),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="elbow_cartridge",
    )
    forearm_body = section_loft(
        [
            _yz_section(0.14, 0.16, 0.03, 0.03, z_offset=-0.01),
            _yz_section(0.15, 0.15, 0.03, 0.24, z_offset=-0.03),
            _yz_section(0.12, 0.12, 0.025, 0.52, z_offset=0.01),
        ]
    )
    forearm.visual(_mesh("forearm_body", forearm_body), material=enamel_green, name="forearm_body")
    forearm.visual(
        Box((0.16, 0.10, 0.05)),
        origin=Origin(xyz=(0.12, 0.0, -0.08)),
        material=machinery_gray,
        name="forearm_rib",
    )
    forearm.visual(
        Box((0.26, 0.12, 0.12)),
        origin=Origin(xyz=(0.45, 0.0, 0.01)),
        material=machinery_gray,
        name="wrist_yoke_base",
    )
    forearm.visual(
        Box((0.16, 0.035, 0.18)),
        origin=Origin(xyz=(0.50, 0.0925, 0.0)),
        material=machinery_gray,
        name="left_wrist_cheek",
    )
    forearm.visual(
        Box((0.16, 0.035, 0.18)),
        origin=Origin(xyz=(0.50, -0.0925, 0.0)),
        material=machinery_gray,
        name="right_wrist_cheek",
    )
    forearm.visual(
        Box((0.10, 0.035, 0.10)),
        origin=Origin(xyz=(0.41, 0.075, -0.02)),
        material=machinery_gray,
        name="left_wrist_web",
    )
    forearm.visual(
        Box((0.10, 0.035, 0.10)),
        origin=Origin(xyz=(0.41, -0.075, -0.02)),
        material=machinery_gray,
        name="right_wrist_web",
    )
    forearm.visual(
        Box((0.20, 0.012, 0.10)),
        origin=Origin(xyz=(0.24, -0.075, -0.01)),
        material=panel_cream,
        name="forearm_service_hatch",
    )
    _add_y_bolt_grid(
        forearm,
        prefix="forearm_hatch_bolt",
        y=-0.081,
        xs=(0.17, 0.31),
        zs=(-0.04, 0.02),
        radius=0.0055,
        length=0.012,
        material=steel,
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.64, 0.18, 0.22)),
        mass=62.0,
        origin=Origin(xyz=(0.28, 0.0, -0.01)),
    )

    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(
        Cylinder(radius=0.055, length=0.11),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="wrist_pitch_cartridge",
    )
    wrist_pitch.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=enamel_green,
        name="wrist_body",
    )
    wrist_pitch.visual(
        Box((0.10, 0.08, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, -0.05)),
        material=machinery_gray,
        name="wrist_rib",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(xyz=(0.17, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machinery_gray,
        name="roll_adapter_nose",
    )
    wrist_pitch.visual(
        Box((0.09, 0.06, 0.010)),
        origin=Origin(xyz=(0.11, 0.0, 0.055)),
        material=panel_cream,
        name="wrist_access_cover",
    )
    for x in (0.08, 0.13):
        for y in (-0.018, 0.018):
            wrist_pitch.visual(
                Cylinder(radius=0.0045, length=0.012),
                origin=Origin(xyz=(x, y, 0.061)),
                material=steel,
                name=f"wrist_cover_bolt_{int(round(x * 1000))}_{int(round((y + 0.05) * 1000))}",
            )
    wrist_pitch.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.18)),
        mass=24.0,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.04, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="roll_spindle",
    )
    tool_flange.visual(
        Cylinder(radius=0.06, length=0.04),
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machinery_gray,
        name="adapter_ring",
    )
    tool_flange.visual(
        Cylinder(radius=0.095, length=0.04),
        origin=Origin(xyz=(0.13, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="tool_mount_flange",
    )
    for index in range(6):
        angle = 2.0 * pi * index / 6.0
        tool_flange.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(
                xyz=(0.148, 0.060 * cos(angle), 0.060 * sin(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=warning_red if index == 0 else steel,
            name=f"flange_bolt_{index}",
        )
    tool_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.16),
        mass=11.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    base_yaw = model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2600.0,
            velocity=0.8,
            lower=-radians(170.0),
            upper=radians(170.0),
        ),
    )
    shoulder_pitch = model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.12, 0.0, 0.38)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.0,
            lower=-radians(95.0),
            upper=radians(80.0),
        ),
    )
    elbow_pitch = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.64, 0.0, -0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=1.2,
            lower=-radians(120.0),
            upper=radians(130.0),
        ),
    )
    wrist_pitch_joint = model.articulation(
        "wrist_pitch_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch,
        origin=Origin(xyz=(0.58, 0.0, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=520.0,
            velocity=1.8,
            lower=-radians(120.0),
            upper=radians(120.0),
        ),
    )
    wrist_roll = model.articulation(
        "wrist_roll",
        ArticulationType.CONTINUOUS,
        parent=wrist_pitch,
        child=tool_flange,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=4.0),
    )

    model.meta["primary_joints"] = [
        base_yaw.name,
        shoulder_pitch.name,
        elbow_pitch.name,
        wrist_pitch_joint.name,
        wrist_roll.name,
    ]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_pitch = object_model.get_part("wrist_pitch")
    tool_flange = object_model.get_part("tool_flange")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch_joint = object_model.get_articulation("wrist_pitch_joint")

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

    ctx.expect_contact(turret, pedestal, contact_tol=1e-4, name="turntable_is_supported")
    ctx.expect_contact(upper_arm, turret, contact_tol=1e-4, name="shoulder_cartridge_is_supported")
    ctx.expect_contact(forearm, upper_arm, contact_tol=1e-4, name="elbow_cartridge_is_supported")
    ctx.expect_contact(wrist_pitch, forearm, contact_tol=1e-4, name="wrist_pitch_cartridge_is_supported")
    ctx.expect_contact(tool_flange, wrist_pitch, contact_tol=1e-4, name="wrist_roll_cartridge_is_supported")

    ctx.expect_origin_gap(upper_arm, turret, axis="x", min_gap=0.10, max_gap=0.14, name="shoulder_offset_reads_forward")
    ctx.expect_origin_gap(forearm, upper_arm, axis="x", min_gap=0.60, max_gap=0.68, name="elbow_offset_is_readable")
    ctx.expect_origin_gap(wrist_pitch, forearm, axis="x", min_gap=0.54, max_gap=0.62, name="wrist_offset_is_readable")
    ctx.expect_origin_gap(tool_flange, wrist_pitch, axis="x", min_gap=0.19, max_gap=0.21, name="tool_roll_offset_is_readable")

    with ctx.pose({shoulder_pitch: radians(45.0)}):
        ctx.expect_origin_gap(forearm, upper_arm, axis="z", min_gap=0.40, name="positive_shoulder_lifts_elbow")

    with ctx.pose({elbow_pitch: radians(55.0)}):
        ctx.expect_origin_gap(wrist_pitch, forearm, axis="z", min_gap=0.32, name="positive_elbow_lifts_wrist")

    with ctx.pose({wrist_pitch_joint: radians(45.0)}):
        ctx.expect_origin_gap(tool_flange, wrist_pitch, axis="z", min_gap=0.10, name="positive_wrist_lifts_tool")

    with ctx.pose({base_yaw: radians(45.0)}):
        ctx.expect_origin_gap(upper_arm, turret, axis="y", min_gap=0.07, name="base_yaw_swings_shoulder_laterally")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
