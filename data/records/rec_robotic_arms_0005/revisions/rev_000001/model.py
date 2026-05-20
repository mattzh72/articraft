from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_robotic_arm", assets=ASSETS)

    utility_yellow = model.material("utility_yellow", rgba=(0.79, 0.67, 0.16, 1.0))
    machine_charcoal = model.material("machine_charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    cover_black = model.material("cover_black", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]

    def yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x: float,
        *,
        z_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_shift) for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=8)]

    def axis_rpy(axis: str) -> tuple[float, float, float]:
        if axis == "x":
            return (0.0, math.pi / 2.0, 0.0)
        if axis == "y":
            return (-math.pi / 2.0, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    def add_bolt_cluster(
        part,
        points: list[tuple[float, float, float]],
        *,
        axis: str,
        radius: float,
        length: float,
        material,
        prefix: str,
    ) -> None:
        rpy = axis_rpy(axis)
        for index, (x, y, z) in enumerate(points):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=rpy),
                material=material,
                name=f"{prefix}_{index}",
            )

    base = model.part("base")
    base.visual(Box((0.48, 0.34, 0.03)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=machine_charcoal, name="base_plate")
    base.visual(Box((0.42, 0.08, 0.05)), origin=Origin(xyz=(0.0, 0.12, 0.025)), material=machine_charcoal, name="front_rail")
    base.visual(Box((0.42, 0.08, 0.05)), origin=Origin(xyz=(0.0, -0.12, 0.025)), material=machine_charcoal, name="rear_rail")
    base.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(0.19, 0.13, 0.006)), material=rubber)
    base.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(-0.19, 0.13, 0.006)), material=rubber)
    base.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(0.19, -0.13, 0.006)), material=rubber)
    base.visual(Cylinder(radius=0.028, length=0.012), origin=Origin(xyz=(-0.19, -0.13, 0.006)), material=rubber)
    base.visual(
        save_mesh(
            "robotic_arm_base_pedestal.obj",
            section_loft(
                [
                    xy_section(0.26, 0.22, 0.040, 0.030),
                    xy_section(0.22, 0.19, 0.035, 0.110),
                    xy_section(0.18, 0.15, 0.030, 0.145),
                ]
            ),
        ),
        material=utility_yellow,
        name="pedestal_shell",
    )
    base.visual(Box((0.16, 0.018, 0.08)), origin=Origin(xyz=(0.0, -0.084, 0.088)), material=cover_black, name="service_panel")
    base.visual(Cylinder(radius=0.122, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.180)), material=dark_steel, name="slew_base_ring")
    base.visual(Cylinder(radius=0.070, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.172)), material=machine_charcoal, name="slew_center_hub")
    add_bolt_cluster(
        base,
        [(0.18, 0.12, 0.034), (-0.18, 0.12, 0.034), (0.18, -0.12, 0.034), (-0.18, -0.12, 0.034)],
        axis="z",
        radius=0.009,
        length=0.008,
        material=steel,
        prefix="anchor_bolt",
    )
    add_bolt_cluster(
        base,
        [(0.090, 0.090, 0.204), (-0.090, 0.090, 0.204), (0.090, -0.090, 0.204), (-0.090, -0.090, 0.204)],
        axis="z",
        radius=0.007,
        length=0.008,
        material=steel,
        prefix="slew_ring_bolt",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.48, 0.34, 0.24)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    shoulder_carriage = model.part("shoulder_carriage")
    shoulder_carriage.visual(
        save_mesh(
            "robotic_arm_yaw_turntable.obj",
            boolean_difference(
                CylinderGeometry(radius=0.108, height=0.050, radial_segments=56),
                CylinderGeometry(radius=0.078, height=0.056, radial_segments=56),
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machine_charcoal,
        name="yaw_turntable",
    )
    shoulder_carriage.visual(Box((0.15, 0.14, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.050)), material=utility_yellow, name="lower_carriage")
    shoulder_carriage.visual(Box((0.09, 0.12, 0.11)), origin=Origin(xyz=(-0.040, 0.0, 0.110)), material=utility_yellow, name="upper_carriage")
    shoulder_carriage.visual(Box((0.045, 0.070, 0.125)), origin=Origin(xyz=(-0.010, 0.0, 0.2275)), material=utility_yellow, name="rear_upright")
    shoulder_carriage.visual(Box((0.11, 0.16, 0.030)), origin=Origin(xyz=(0.040, 0.0, 0.305)), material=utility_yellow, name="shoulder_bridge")
    shoulder_carriage.visual(Box((0.045, 0.028, 0.150)), origin=Origin(xyz=(0.082, 0.079, 0.225)), material=utility_yellow, name="shoulder_left_ear")
    shoulder_carriage.visual(Box((0.045, 0.028, 0.150)), origin=Origin(xyz=(0.082, -0.079, 0.225)), material=utility_yellow, name="shoulder_right_ear")
    shoulder_carriage.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.080, 0.055, 0.220), rpy=axis_rpy("y")),
        material=dark_steel,
        name="shoulder_left_sleeve",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.080, -0.055, 0.220), rpy=axis_rpy("y")),
        material=dark_steel,
        name="shoulder_right_sleeve",
    )
    shoulder_carriage.visual(Box((0.10, 0.12, 0.10)), origin=Origin(xyz=(-0.050, 0.0, 0.130)), material=cover_black, name="base_drive_housing")
    shoulder_carriage.visual(Cylinder(radius=0.034, length=0.080), origin=Origin(xyz=(-0.100, 0.0, 0.145), rpy=axis_rpy("x")), material=machine_charcoal, name="base_motor_cap")
    shoulder_carriage.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.34)),
        mass=28.0,
        origin=Origin(xyz=(0.010, 0.0, 0.150)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.055, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=axis_rpy("y")),
        material=dark_steel,
        name="shoulder_hub",
    )
    upper_arm.visual(Box((0.070, 0.050, 0.080)), origin=Origin(xyz=(0.085, 0.0, 0.000)), material=utility_yellow, name="root_web")
    upper_arm.visual(
        save_mesh(
            "robotic_arm_upper_arm_shell.obj",
            section_loft(
                [
                    yz_section(0.064, 0.100, 0.016, 0.095, z_shift=0.010),
                    yz_section(0.106, 0.128, 0.022, 0.195, z_shift=0.016),
                    yz_section(0.086, 0.102, 0.018, 0.292, z_shift=0.012),
                ]
            ),
        ),
        material=utility_yellow,
        name="upper_arm_shell",
    )
    upper_arm.visual(Box((0.120, 0.075, 0.035)), origin=Origin(xyz=(0.190, 0.0, 0.0925)), material=cover_black, name="upper_actuator_cover")
    upper_arm.visual(Cylinder(radius=0.026, length=0.120), origin=Origin(xyz=(0.190, 0.0, 0.060), rpy=axis_rpy("x")), material=machine_charcoal, name="upper_actuator_can")
    upper_arm.visual(Box((0.180, 0.045, 0.030)), origin=Origin(xyz=(0.205, 0.0, -0.050)), material=machine_charcoal, name="lower_reinforcement")
    upper_arm.visual(Box((0.070, 0.140, 0.035)), origin=Origin(xyz=(0.305, 0.0, 0.0725)), material=utility_yellow, name="elbow_bridge")
    upper_arm.visual(Box((0.045, 0.030, 0.110)), origin=Origin(xyz=(0.338, 0.070, 0.000)), material=utility_yellow, name="elbow_left_ear")
    upper_arm.visual(Box((0.045, 0.030, 0.110)), origin=Origin(xyz=(0.338, -0.070, 0.000)), material=utility_yellow, name="elbow_right_ear")
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=Origin(xyz=(0.360, 0.055, 0.000), rpy=axis_rpy("y")),
        material=dark_steel,
        name="elbow_left_sleeve",
    )
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=Origin(xyz=(0.360, -0.055, 0.000), rpy=axis_rpy("y")),
        material=dark_steel,
        name="elbow_right_sleeve",
    )
    add_bolt_cluster(
        upper_arm,
        [(0.150, 0.025, 0.113), (0.230, 0.025, 0.113), (0.150, -0.025, 0.113), (0.230, -0.025, 0.113)],
        axis="z",
        radius=0.0055,
        length=0.008,
        material=steel,
        prefix="upper_cover_bolt",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.18, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.180, 0.0, 0.010)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.048, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=axis_rpy("y")),
        material=dark_steel,
        name="elbow_hub",
    )
    forearm.visual(Box((0.065, 0.050, 0.075)), origin=Origin(xyz=(0.075, 0.0, 0.000)), material=utility_yellow, name="root_web")
    forearm.visual(
        save_mesh(
            "robotic_arm_forearm_shell.obj",
            section_loft(
                [
                    yz_section(0.056, 0.086, 0.014, 0.085, z_shift=0.004),
                    yz_section(0.094, 0.112, 0.020, 0.182, z_shift=0.010),
                    yz_section(0.080, 0.092, 0.016, 0.270, z_shift=0.014),
                ]
            ),
        ),
        material=utility_yellow,
        name="forearm_shell",
    )
    forearm.visual(Box((0.120, 0.065, 0.032)), origin=Origin(xyz=(0.175, 0.0, 0.076)), material=cover_black, name="forearm_actuator_cover")
    forearm.visual(Cylinder(radius=0.024, length=0.110), origin=Origin(xyz=(0.175, 0.0, 0.050), rpy=axis_rpy("x")), material=machine_charcoal, name="forearm_actuator_can")
    forearm.visual(Box((0.180, 0.035, 0.028)), origin=Origin(xyz=(0.185, 0.0, -0.054)), material=machine_charcoal, name="forearm_keel")
    forearm.visual(Box((0.060, 0.130, 0.032)), origin=Origin(xyz=(0.260, 0.0, 0.056)), material=utility_yellow, name="wrist_bridge")
    forearm.visual(Box((0.040, 0.026, 0.095)), origin=Origin(xyz=(0.290, 0.068, 0.000)), material=utility_yellow, name="wrist_left_ear")
    forearm.visual(Box((0.040, 0.026, 0.095)), origin=Origin(xyz=(0.290, -0.068, 0.000)), material=utility_yellow, name="wrist_right_ear")
    forearm.visual(
        Cylinder(radius=0.042, length=0.042),
        origin=Origin(xyz=(0.310, 0.050, 0.000), rpy=axis_rpy("y")),
        material=dark_steel,
        name="wrist_left_sleeve",
    )
    forearm.visual(
        Cylinder(radius=0.042, length=0.042),
        origin=Origin(xyz=(0.310, -0.050, 0.000), rpy=axis_rpy("y")),
        material=dark_steel,
        name="wrist_right_sleeve",
    )
    add_bolt_cluster(
        forearm,
        [(0.145, 0.022, 0.092), (0.205, 0.022, 0.092), (0.145, -0.022, 0.092), (0.205, -0.022, 0.092)],
        axis="z",
        radius=0.005,
        length=0.008,
        material=steel,
        prefix="forearm_cover_bolt",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.16)),
        mass=12.0,
        origin=Origin(xyz=(0.155, 0.0, 0.005)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.038, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=axis_rpy("y")),
        material=dark_steel,
        name="wrist_hub",
    )
    wrist_head.visual(Box((0.050, 0.045, 0.070)), origin=Origin(xyz=(0.055, 0.0, 0.000)), material=utility_yellow, name="root_web")
    wrist_head.visual(
        save_mesh(
            "robotic_arm_wrist_head_shell.obj",
            section_loft(
                [
                    yz_section(0.044, 0.060, 0.012, 0.058, z_shift=0.002),
                    yz_section(0.070, 0.080, 0.016, 0.084, z_shift=0.006),
                    yz_section(0.056, 0.064, 0.014, 0.106, z_shift=0.002),
                ]
            ),
        ),
        material=utility_yellow,
        name="wrist_head_shell",
    )
    wrist_head.visual(Box((0.055, 0.060, 0.030)), origin=Origin(xyz=(0.084, 0.0, 0.050)), material=cover_black, name="wrist_access_cover")
    wrist_head.visual(Cylinder(radius=0.038, length=0.040), origin=Origin(xyz=(0.125, 0.0, 0.000), rpy=axis_rpy("x")), material=machine_charcoal, name="roll_bearing")
    wrist_head.visual(Cylinder(radius=0.018, length=0.050), origin=Origin(xyz=(0.070, 0.028, 0.010)), material=machine_charcoal, name="wrist_drive_can")
    add_bolt_cluster(
        wrist_head,
        [(0.068, 0.020, 0.0685), (0.068, -0.020, 0.0685), (0.094, 0.020, 0.0685), (0.094, -0.020, 0.0685)],
        axis="z",
        radius=0.0045,
        length=0.007,
        material=steel,
        prefix="wrist_cover_bolt",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.12)),
        mass=6.0,
        origin=Origin(xyz=(0.080, 0.0, 0.000)),
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=axis_rpy("x")),
        material=dark_steel,
        name="roll_spindle",
    )
    tool_flange.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=axis_rpy("x")),
        material=steel,
        name="tool_flange_disc",
    )
    tool_flange.visual(Box((0.028, 0.060, 0.024)), origin=Origin(xyz=(0.045, 0.0, -0.018)), material=machine_charcoal, name="flange_mount_block")
    tool_flange.visual(Box((0.055, 0.080, 0.055)), origin=Origin(xyz=(0.075, 0.0, 0.0)), material=machine_charcoal, name="tool_plate")
    tool_flange.visual(Box((0.030, 0.022, 0.090)), origin=Origin(xyz=(0.090, 0.0, 0.018)), material=cover_black, name="tool_bracket")
    add_bolt_cluster(
        tool_flange,
        [(0.048, 0.023, 0.022), (0.048, -0.023, 0.022), (0.048, 0.023, -0.022), (0.048, -0.023, -0.022)],
        axis="x",
        radius=0.0045,
        length=0.008,
        material=steel,
        prefix="tool_flange_bolt",
    )
    tool_flange.inertial = Inertial.from_geometry(
        Box((0.14, 0.10, 0.12)),
        mass=2.5,
        origin=Origin(xyz=(0.070, 0.0, 0.010)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=1.2, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.080, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=1.4, lower=-1.2, upper=1.35),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=1.7, lower=-1.5, upper=1.45),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=2.1, lower=-1.9, upper=1.9),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.CONTINUOUS,
        parent=wrist_head,
        child=tool_flange,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    shoulder_carriage = object_model.get_part("shoulder_carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    tool_flange = object_model.get_part("tool_flange")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    slew_base_ring = base.get_visual("slew_base_ring")
    yaw_turntable = shoulder_carriage.get_visual("yaw_turntable")
    shoulder_left_sleeve = shoulder_carriage.get_visual("shoulder_left_sleeve")
    shoulder_right_sleeve = shoulder_carriage.get_visual("shoulder_right_sleeve")
    shoulder_hub = upper_arm.get_visual("shoulder_hub")
    elbow_left_sleeve = upper_arm.get_visual("elbow_left_sleeve")
    elbow_right_sleeve = upper_arm.get_visual("elbow_right_sleeve")
    elbow_hub = forearm.get_visual("elbow_hub")
    wrist_left_sleeve = forearm.get_visual("wrist_left_sleeve")
    wrist_right_sleeve = forearm.get_visual("wrist_right_sleeve")
    wrist_hub = wrist_head.get_visual("wrist_hub")
    roll_bearing = wrist_head.get_visual("roll_bearing")
    roll_spindle = tool_flange.get_visual("roll_spindle")
    tool_bracket = tool_flange.get_visual("tool_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        base,
        shoulder_carriage,
        reason="Nested slewing bearing between pedestal ring and rotating carriage turntable.",
        elem_a=slew_base_ring,
        elem_b=yaw_turntable,
    )
    ctx.allow_overlap(
        shoulder_carriage,
        upper_arm,
        reason="Upper-arm shoulder hub sits inside the carriage bearing sleeves.",
        elem_a=shoulder_left_sleeve,
        elem_b=shoulder_hub,
    )
    ctx.allow_overlap(
        shoulder_carriage,
        upper_arm,
        reason="Upper-arm shoulder hub sits inside the carriage bearing sleeves.",
        elem_a=shoulder_right_sleeve,
        elem_b=shoulder_hub,
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        reason="Forearm elbow hub nests in reinforced elbow collars.",
        elem_a=elbow_left_sleeve,
        elem_b=elbow_hub,
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        reason="Forearm elbow hub nests in reinforced elbow collars.",
        elem_a=elbow_right_sleeve,
        elem_b=elbow_hub,
    )
    ctx.allow_overlap(
        forearm,
        wrist_head,
        reason="Wrist pitch hub nests inside the forearm wrist collars.",
        elem_a=wrist_left_sleeve,
        elem_b=wrist_hub,
    )
    ctx.allow_overlap(
        forearm,
        wrist_head,
        reason="Wrist pitch hub nests inside the forearm wrist collars.",
        elem_a=wrist_right_sleeve,
        elem_b=wrist_hub,
    )
    ctx.allow_overlap(
        wrist_head,
        tool_flange,
        reason="Tool spindle is captured by the wrist roll bearing.",
        elem_a=roll_bearing,
        elem_b=roll_spindle,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(base, shoulder_carriage, elem_a=slew_base_ring, elem_b=yaw_turntable)
    ctx.expect_overlap(base, shoulder_carriage, axes="xy", elem_a=slew_base_ring, elem_b=yaw_turntable, min_overlap=0.19)
    ctx.expect_contact(shoulder_carriage, upper_arm, elem_a=shoulder_left_sleeve, elem_b=shoulder_hub)
    ctx.expect_contact(shoulder_carriage, upper_arm, elem_a=shoulder_right_sleeve, elem_b=shoulder_hub)
    ctx.expect_overlap(shoulder_carriage, upper_arm, axes="xz", elem_a=shoulder_left_sleeve, elem_b=shoulder_hub, min_overlap=0.09)
    ctx.expect_overlap(shoulder_carriage, upper_arm, axes="xz", elem_a=shoulder_right_sleeve, elem_b=shoulder_hub, min_overlap=0.09)
    ctx.expect_contact(upper_arm, forearm, elem_a=elbow_left_sleeve, elem_b=elbow_hub)
    ctx.expect_contact(upper_arm, forearm, elem_a=elbow_right_sleeve, elem_b=elbow_hub)
    ctx.expect_overlap(upper_arm, forearm, axes="xz", elem_a=elbow_left_sleeve, elem_b=elbow_hub, min_overlap=0.08)
    ctx.expect_overlap(upper_arm, forearm, axes="xz", elem_a=elbow_right_sleeve, elem_b=elbow_hub, min_overlap=0.08)
    ctx.expect_contact(forearm, wrist_head, elem_a=wrist_left_sleeve, elem_b=wrist_hub)
    ctx.expect_contact(forearm, wrist_head, elem_a=wrist_right_sleeve, elem_b=wrist_hub)
    ctx.expect_overlap(forearm, wrist_head, axes="xz", elem_a=wrist_left_sleeve, elem_b=wrist_hub, min_overlap=0.06)
    ctx.expect_overlap(forearm, wrist_head, axes="xz", elem_a=wrist_right_sleeve, elem_b=wrist_hub, min_overlap=0.06)
    ctx.expect_contact(wrist_head, tool_flange, elem_a=roll_bearing, elem_b=roll_spindle)
    ctx.expect_overlap(wrist_head, tool_flange, axes="yz", elem_a=roll_bearing, elem_b=roll_spindle, min_overlap=0.05)

    ctx.expect_origin_distance(shoulder_carriage, base, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(forearm, upper_arm, axes="x", min_dist=0.32, max_dist=0.40)
    ctx.expect_origin_distance(wrist_head, forearm, axes="x", min_dist=0.27, max_dist=0.34)
    ctx.expect_origin_distance(tool_flange, wrist_head, axes="x", min_dist=0.13, max_dist=0.17)

    forearm_rest = ctx.part_world_position(forearm)
    wrist_rest = ctx.part_world_position(wrist_head)
    tool_rest = ctx.part_world_position(tool_flange)
    bracket_rest = ctx.part_element_world_aabb(tool_flange, elem=tool_bracket)

    if forearm_rest is None or wrist_rest is None or tool_rest is None or bracket_rest is None:
        ctx.fail("pose_probe_availability", "Missing world-space measurements for articulated arm pose checks.")
        return ctx.report()

    with ctx.pose({base_yaw: math.radians(60.0)}):
        forearm_yawed = ctx.part_world_position(forearm)
        if forearm_yawed is None:
            ctx.fail("base_yaw_motion", "Forearm position unavailable in yawed pose.")
        else:
            ctx.check(
                "base_yaw_motion",
                forearm_yawed[1] > forearm_rest[1] + 0.26 and forearm_yawed[0] < forearm_rest[0] - 0.12,
                f"Expected yaw to swing forearm around base; rest={forearm_rest}, yawed={forearm_yawed}",
            )
        ctx.expect_contact(base, shoulder_carriage, elem_a=slew_base_ring, elem_b=yaw_turntable)

    with ctx.pose({shoulder_pitch: math.radians(45.0)}):
        wrist_lifted = ctx.part_world_position(wrist_head)
        if wrist_lifted is None:
            ctx.fail("shoulder_pitch_motion", "Wrist position unavailable in lifted shoulder pose.")
        else:
            ctx.check(
                "shoulder_pitch_motion",
                wrist_lifted[2] > wrist_rest[2] + 0.18 and wrist_lifted[0] < wrist_rest[0] - 0.10,
                f"Expected shoulder lift to raise wrist; rest={wrist_rest}, lifted={wrist_lifted}",
            )
        ctx.expect_contact(shoulder_carriage, upper_arm, elem_a=shoulder_left_sleeve, elem_b=shoulder_hub)

    with ctx.pose({elbow_pitch: math.radians(70.0)}):
        tool_folded = ctx.part_world_position(tool_flange)
        if tool_folded is None:
            ctx.fail("elbow_pitch_motion", "Tool position unavailable in elbow-fold pose.")
        else:
            ctx.check(
                "elbow_pitch_motion",
                tool_folded[2] > tool_rest[2] + 0.16 and tool_folded[0] < tool_rest[0] - 0.16,
                f"Expected elbow fold to bring tool upward and back; rest={tool_rest}, folded={tool_folded}",
            )
        ctx.expect_contact(upper_arm, forearm, elem_a=elbow_left_sleeve, elem_b=elbow_hub)

    with ctx.pose({wrist_pitch: math.radians(65.0)}):
        tool_tipped = ctx.part_world_position(tool_flange)
        if tool_tipped is None:
            ctx.fail("wrist_pitch_motion", "Tool position unavailable in wrist-tip pose.")
        else:
            ctx.check(
                "wrist_pitch_motion",
                tool_tipped[2] > tool_rest[2] + 0.05 and tool_tipped[0] < tool_rest[0],
                f"Expected wrist pitch to tip tool upward; rest={tool_rest}, tipped={tool_tipped}",
            )
        ctx.expect_contact(forearm, wrist_head, elem_a=wrist_left_sleeve, elem_b=wrist_hub)

    with ctx.pose({wrist_roll: math.pi / 2.0}):
        bracket_rolled = ctx.part_element_world_aabb(tool_flange, elem=tool_bracket)
        if bracket_rolled is None:
            ctx.fail("wrist_roll_motion", "Tool bracket AABB unavailable in rolled pose.")
        else:
            rest_dims = (
                bracket_rest[1][0] - bracket_rest[0][0],
                bracket_rest[1][1] - bracket_rest[0][1],
                bracket_rest[1][2] - bracket_rest[0][2],
            )
            rolled_dims = (
                bracket_rolled[1][0] - bracket_rolled[0][0],
                bracket_rolled[1][1] - bracket_rolled[0][1],
                bracket_rolled[1][2] - bracket_rolled[0][2],
            )
            ctx.check(
                "wrist_roll_motion",
                rolled_dims[1] > rest_dims[1] + 0.05 and rolled_dims[2] < rest_dims[2] - 0.05,
                f"Expected tool bracket y/z extents to swap under roll; rest={rest_dims}, rolled={rolled_dims}",
            )
        ctx.expect_contact(wrist_head, tool_flange, elem_a=roll_bearing, elem_b=roll_spindle)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
