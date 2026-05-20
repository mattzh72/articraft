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
    width: float,
    height: float,
    radius: float,
    *,
    y_off: float = 0.0,
    z_off: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_off + y, z_off + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _xy_section(
    z: float,
    width: float,
    depth: float,
    radius: float,
    *,
    x_off: float = 0.0,
    y_off: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x_off + x, y_off + y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_arm")

    shell_paint = model.material("shell_paint", rgba=(0.84, 0.85, 0.87, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.20, 0.21, 0.23, 1.0))
    dark_elastomer = model.material("dark_elastomer", rgba=(0.09, 0.09, 0.10, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    pedestal = model.part("pedestal")
    pedestal_shell = section_loft(
        [
            _xy_section(0.000, 0.280, 0.240, 0.055),
            _xy_section(0.032, 0.272, 0.232, 0.053),
            _xy_section(0.108, 0.205, 0.190, 0.045),
            _xy_section(0.145, 0.185, 0.175, 0.040),
        ]
    )
    pedestal.visual(_mesh("pedestal_shell", pedestal_shell), material=shell_paint, name="pedestal_shell")
    pedestal.visual(
        Cylinder(radius=0.140, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_elastomer,
        name="foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.092, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        material=graphite_polymer,
        name="yaw_bearing",
    )

    turret = model.part("turret")
    turret_shell = section_loft(
        [
            _xy_section(0.000, 0.156, 0.146, 0.040),
            _xy_section(0.072, 0.136, 0.126, 0.034, x_off=0.008),
            _xy_section(0.128, 0.116, 0.110, 0.030, x_off=0.015),
            _xy_section(0.155, 0.096, 0.100, 0.028, x_off=0.020),
        ]
    )
    turret.visual(
        Cylinder(radius=0.098, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite_polymer,
        name="yaw_cartridge",
    )
    turret.visual(
        Cylinder(radius=0.038, length=0.088),
        origin=Origin(xyz=(0.030, 0.0, 0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="shoulder_cartridge",
    )
    turret.visual(_mesh("turret_shell", turret_shell), material=shell_paint, name="turret_shell")
    turret.visual(
        Box((0.052, 0.015, 0.092)),
        origin=Origin(xyz=(0.030, 0.0515, 0.190)),
        material=graphite_polymer,
        name="shoulder_left_ear",
    )
    turret.visual(
        Box((0.052, 0.015, 0.092)),
        origin=Origin(xyz=(0.030, -0.0515, 0.190)),
        material=graphite_polymer,
        name="shoulder_right_ear",
    )
    turret.visual(
        Box((0.055, 0.118, 0.022)),
        origin=Origin(xyz=(0.028, 0.0, 0.145)),
        material=graphite_polymer,
        name="shoulder_bridge",
    )

    upper_arm = model.part("upper_arm")
    upper_arm_shell = section_loft(
        [
            _yz_section(0.072, 0.052, 0.062, 0.015, z_off=0.004),
            _yz_section(0.150, 0.094, 0.112, 0.026, z_off=0.022),
            _yz_section(0.245, 0.090, 0.106, 0.024, z_off=0.020),
            _yz_section(0.285, 0.080, 0.096, 0.022, z_off=0.010),
        ]
    )
    upper_arm.visual(
        Box((0.040, 0.040, 0.024)),
        origin=Origin(xyz=(0.058, 0.0, 0.000)),
        material=graphite_polymer,
        name="shoulder_tongue",
    )
    upper_arm.visual(_mesh("upper_arm_shell", upper_arm_shell), material=shell_paint, name="upper_arm_shell")
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.068),
        origin=Origin(xyz=(0.330, 0.0, -0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="elbow_cartridge",
    )
    upper_arm.visual(
        Box((0.044, 0.014, 0.082)),
        origin=Origin(xyz=(0.308, 0.041, -0.008)),
        material=graphite_polymer,
        name="elbow_left_ear",
    )
    upper_arm.visual(
        Box((0.044, 0.014, 0.082)),
        origin=Origin(xyz=(0.308, -0.041, -0.008)),
        material=graphite_polymer,
        name="elbow_right_ear",
    )
    upper_arm.visual(
        Box((0.060, 0.110, 0.022)),
        origin=Origin(xyz=(0.292, 0.0, -0.037)),
        material=graphite_polymer,
        name="elbow_bridge",
    )

    forearm = model.part("forearm")
    forearm_shell = section_loft(
        [
            _yz_section(0.074, 0.050, 0.056, 0.014, z_off=0.002),
            _yz_section(0.145, 0.076, 0.086, 0.020, z_off=0.016),
            _yz_section(0.215, 0.070, 0.080, 0.018, z_off=0.018),
            _yz_section(0.240, 0.062, 0.074, 0.017, z_off=0.012),
        ]
    )
    forearm.visual(
        Box((0.042, 0.036, 0.020)),
        origin=Origin(xyz=(0.053, 0.0, 0.000)),
        material=graphite_polymer,
        name="elbow_tongue",
    )
    forearm.visual(_mesh("forearm_shell", forearm_shell), material=shell_paint, name="forearm_shell")
    forearm.visual(
        Cylinder(radius=0.026, length=0.056),
        origin=Origin(xyz=(0.290, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="wrist_pitch_cartridge",
    )
    forearm.visual(
        Box((0.038, 0.012, 0.064)),
        origin=Origin(xyz=(0.272, 0.034, 0.012)),
        material=graphite_polymer,
        name="wrist_left_ear",
    )
    forearm.visual(
        Box((0.038, 0.012, 0.064)),
        origin=Origin(xyz=(0.272, -0.034, 0.012)),
        material=graphite_polymer,
        name="wrist_right_ear",
    )
    forearm.visual(
        Box((0.055, 0.082, 0.018)),
        origin=Origin(xyz=(0.255, 0.0, -0.012)),
        material=graphite_polymer,
        name="wrist_bridge",
    )

    wrist_housing = model.part("wrist_housing")
    wrist_shell = section_loft(
        [
            _yz_section(0.062, 0.040, 0.045, 0.012, z_off=0.002),
            _yz_section(0.088, 0.056, 0.064, 0.016, z_off=0.008),
            _yz_section(0.118, 0.048, 0.058, 0.014, z_off=0.004),
        ]
    )
    wrist_housing.visual(
        Box((0.036, 0.030, 0.018)),
        origin=Origin(xyz=(0.044, 0.0, 0.004)),
        material=graphite_polymer,
        name="wrist_tongue",
    )
    wrist_housing.visual(_mesh("wrist_shell", wrist_shell), material=shell_paint, name="wrist_shell")
    wrist_housing.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.105, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_polymer,
        name="roll_collar",
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite_polymer,
        name="roll_spindle",
    )
    tool_flange.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_aluminum,
        name="flange_plate",
    )
    tool_flange.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_elastomer,
        name="tool_face_pad",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.2),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.030, 0.0, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.8,
            lower=-0.55,
            upper=1.25,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.330, 0.0, -0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.9,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_housing,
        origin=Origin(xyz=(0.290, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.4,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.CONTINUOUS,
        parent=wrist_housing,
        child=tool_flange,
        origin=Origin(xyz=(0.125, 0.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_housing = object_model.get_part("wrist_housing")
    tool_flange = object_model.get_part("tool_flange")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

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

    ctx.expect_contact(pedestal, turret, name="pedestal_supports_turret")
    ctx.expect_contact(turret, upper_arm, name="turret_supports_upper_arm")
    ctx.expect_contact(upper_arm, forearm, name="upper_arm_supports_forearm")
    ctx.expect_contact(forearm, wrist_housing, name="forearm_supports_wrist")
    ctx.expect_contact(wrist_housing, tool_flange, name="wrist_supports_tool")

    ctx.check("base_yaw_axis_is_vertical", base_yaw.axis == (0.0, 0.0, 1.0), str(base_yaw.axis))
    ctx.check("shoulder_axis_reads_as_pitch", shoulder_pitch.axis == (0.0, -1.0, 0.0), str(shoulder_pitch.axis))
    ctx.check("elbow_axis_reads_as_pitch", elbow_pitch.axis == (0.0, -1.0, 0.0), str(elbow_pitch.axis))
    ctx.check("wrist_pitch_axis_reads_as_pitch", wrist_pitch.axis == (0.0, -1.0, 0.0), str(wrist_pitch.axis))
    ctx.check("wrist_roll_axis_reads_as_axial_roll", wrist_roll.axis == (1.0, 0.0, 0.0), str(wrist_roll.axis))

    rest_tip = ctx.part_world_position(tool_flange)
    with ctx.pose({shoulder_pitch: 0.65}):
        shoulder_tip = ctx.part_world_position(tool_flange)
    with ctx.pose({elbow_pitch: 1.00}):
        elbow_tip = ctx.part_world_position(tool_flange)
    with ctx.pose({wrist_pitch: 0.75}):
        wrist_tip = ctx.part_world_position(tool_flange)
    with ctx.pose({wrist_roll: 1.10}):
        roll_tip = ctx.part_world_position(tool_flange)

    if rest_tip is not None and shoulder_tip is not None:
        ctx.check(
            "positive_shoulder_lifts_tool",
            shoulder_tip[2] > rest_tip[2] + 0.12,
            f"rest_z={rest_tip[2]:.4f}, posed_z={shoulder_tip[2]:.4f}",
        )
    else:
        ctx.fail("positive_shoulder_lifts_tool", "tool flange world position unavailable")

    if rest_tip is not None and elbow_tip is not None:
        ctx.check(
            "positive_elbow_folds_reach_inward",
            elbow_tip[0] < rest_tip[0] - 0.10 and elbow_tip[2] > rest_tip[2] + 0.06,
            f"rest={rest_tip}, posed={elbow_tip}",
        )
    else:
        ctx.fail("positive_elbow_folds_reach_inward", "tool flange world position unavailable")

    if rest_tip is not None and wrist_tip is not None:
        ctx.check(
            "positive_wrist_pitch_lifts_tool_face",
            wrist_tip[2] > rest_tip[2] + 0.025,
            f"rest_z={rest_tip[2]:.4f}, posed_z={wrist_tip[2]:.4f}",
        )
    else:
        ctx.fail("positive_wrist_pitch_lifts_tool_face", "tool flange world position unavailable")

    if rest_tip is not None and roll_tip is not None:
        same_point = (
            abs(roll_tip[0] - rest_tip[0]) < 1e-6
            and abs(roll_tip[1] - rest_tip[1]) < 1e-6
            and abs(roll_tip[2] - rest_tip[2]) < 1e-6
        )
        ctx.check(
            "wrist_roll_spins_in_place",
            same_point,
            f"rest={rest_tip}, posed={roll_tip}",
        )
    else:
        ctx.fail("wrist_roll_spins_in_place", "tool flange world position unavailable")

    with ctx.pose(
        {
            base_yaw: 0.55,
            shoulder_pitch: 0.70,
            elbow_pitch: 1.15,
            wrist_pitch: -0.45,
            wrist_roll: 0.60,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_work_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
