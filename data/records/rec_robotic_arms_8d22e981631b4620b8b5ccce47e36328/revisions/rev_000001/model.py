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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _beam_mesh(name: str, *, length: float, width_y: float, height_z: float, corner: float):
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(height_z, width_y, corner),
        length,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_robot_arm")

    cast_gray = model.material("cast_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    machined_silver = model.material("machined_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.20, 0.22, 0.24, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.92, 0.47, 0.12, 1.0))

    upper_beam = _beam_mesh(
        "upper_arm_beam",
        length=0.336,
        width_y=0.082,
        height_z=0.092,
        corner=0.012,
    )
    forearm_beam = _beam_mesh(
        "forearm_beam",
        length=0.272,
        width_y=0.068,
        height_z=0.074,
        corner=0.010,
    )
    pedestal_column = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.220, 0.180, 0.026),
            0.285,
        ),
        "pedestal_column",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.440, 0.440, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=graphite,
        name="base_plate",
    )
    pedestal.visual(
        pedestal_column,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=dark_anodized,
        name="bearing_cap",
    )
    pedestal.visual(
        Box((0.100, 0.012, 0.140)),
        origin=Origin(xyz=(0.0, -0.096, 0.145)),
        material=machined_silver,
        name="front_datum_pad",
    )
    pedestal.visual(
        Box((0.016, 0.080, 0.140)),
        origin=Origin(xyz=(0.112, 0.0, 0.145)),
        material=machined_silver,
        name="side_datum_pad",
    )
    pedestal.visual(
        Box((0.010, 0.022, 0.003)),
        origin=Origin(xyz=(0.090, 0.0, 0.3495)),
        material=safety_orange,
        name="pedestal_index_mark",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.440, 0.440, 0.360)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    shoulder = model.part("shoulder_carriage")
    shoulder.visual(
        Cylinder(radius=0.112, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_anodized,
        name="rotor_flange",
    )
    shoulder.visual(
        Cylinder(radius=0.086, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=cast_gray,
        name="azimuth_cartridge",
    )
    shoulder.visual(
        Box((0.096, 0.170, 0.080)),
        origin=Origin(xyz=(-0.056, 0.0, 0.160)),
        material=cast_gray,
        name="actuator_block",
    )
    shoulder.visual(
        Box((0.060, 0.180, 0.024)),
        origin=Origin(xyz=(0.012, 0.0, 0.132)),
        material=cast_gray,
        name="shoulder_bridge",
    )
    shoulder.visual(
        Box((0.040, 0.024, 0.160)),
        origin=Origin(xyz=(0.028, 0.066, 0.220)),
        material=machined_silver,
        name="left_yoke",
    )
    shoulder.visual(
        Box((0.040, 0.024, 0.160)),
        origin=Origin(xyz=(0.028, -0.066, 0.220)),
        material=machined_silver,
        name="right_yoke",
    )
    shoulder.visual(
        Box((0.032, 0.180, 0.024)),
        origin=Origin(xyz=(-0.002, 0.0, 0.286)),
        material=machined_silver,
        name="axis_cap_bridge",
    )
    shoulder.visual(
        Box((0.010, 0.020, 0.003)),
        origin=Origin(xyz=(0.090, 0.0, 0.0255)),
        material=safety_orange,
        name="azimuth_index_mark",
    )
    shoulder.visual(
        Box((0.012, 0.004, 0.038)),
        origin=Origin(xyz=(0.048, 0.080, 0.220)),
        material=safety_orange,
        name="shoulder_index_mark",
    )
    shoulder.inertial = Inertial.from_geometry(
        Box((0.220, 0.190, 0.300)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="shoulder_trunnion",
    )
    upper_arm.visual(
        Box((0.036, 0.078, 0.080)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=cast_gray,
        name="shoulder_root_block",
    )
    upper_arm.visual(
        upper_beam,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=cast_gray,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.140, 0.040, 0.010)),
        origin=Origin(xyz=(0.170, 0.0, 0.048)),
        material=machined_silver,
        name="upper_datum_flat",
    )
    upper_arm.visual(
        Box((0.050, 0.060, 0.026)),
        origin=Origin(xyz=(0.220, 0.0, -0.059)),
        material=dark_anodized,
        name="upper_adjuster_block",
    )
    upper_arm.visual(
        Box((0.070, 0.138, 0.036)),
        origin=Origin(xyz=(0.344, 0.0, -0.058)),
        material=cast_gray,
        name="elbow_clevis_bridge",
    )
    upper_arm.visual(
        Box((0.032, 0.022, 0.108)),
        origin=Origin(xyz=(0.379, 0.056, 0.0)),
        material=machined_silver,
        name="left_elbow_clevis",
    )
    upper_arm.visual(
        Box((0.032, 0.022, 0.108)),
        origin=Origin(xyz=(0.379, -0.056, 0.0)),
        material=machined_silver,
        name="right_elbow_clevis",
    )
    upper_arm.visual(
        Box((0.010, 0.004, 0.032)),
        origin=Origin(xyz=(0.391, 0.069, 0.0)),
        material=safety_orange,
        name="elbow_index_mark",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.420, 0.120, 0.130)),
        mass=16.0,
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="elbow_trunnion",
    )
    forearm.visual(
        Box((0.030, 0.068, 0.072)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=cast_gray,
        name="elbow_root_block",
    )
    forearm.visual(
        forearm_beam,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=cast_gray,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.120, 0.034, 0.008)),
        origin=Origin(xyz=(0.150, 0.0, 0.038)),
        material=machined_silver,
        name="forearm_datum_flat",
    )
    forearm.visual(
        Box((0.046, 0.050, 0.020)),
        origin=Origin(xyz=(0.190, 0.0, -0.047)),
        material=dark_anodized,
        name="forearm_adjuster_block",
    )
    forearm.visual(
        Box((0.054, 0.118, 0.040)),
        origin=Origin(xyz=(0.280, 0.0, -0.052)),
        material=cast_gray,
        name="wrist_clevis_bridge",
    )
    forearm.visual(
        Box((0.026, 0.018, 0.082)),
        origin=Origin(xyz=(0.307, 0.045, 0.0)),
        material=machined_silver,
        name="left_wrist_clevis",
    )
    forearm.visual(
        Box((0.026, 0.018, 0.082)),
        origin=Origin(xyz=(0.307, -0.045, 0.0)),
        material=machined_silver,
        name="right_wrist_clevis",
    )
    forearm.visual(
        Box((0.008, 0.004, 0.026)),
        origin=Origin(xyz=(0.314, 0.056, 0.0)),
        material=safety_orange,
        name="wrist_index_mark",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.340, 0.100, 0.105)),
        mass=9.0,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.032, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="wrist_trunnion",
    )
    wrist.visual(
        Box((0.028, 0.058, 0.058)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=cast_gray,
        name="wrist_root_block",
    )
    wrist.visual(
        Box((0.100, 0.060, 0.062)),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=cast_gray,
        name="wrist_head",
    )
    wrist.visual(
        Box((0.050, 0.040, 0.008)),
        origin=Origin(xyz=(0.082, 0.0, 0.035)),
        material=machined_silver,
        name="wrist_datum_flat",
    )
    wrist.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anodized,
        name="tool_spindle",
    )
    wrist.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_silver,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.012, 0.012, 0.004)),
        origin=Origin(xyz=(0.166, 0.0, 0.022)),
        material=safety_orange,
        name="flange_alignment_key",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.190, 0.080, 0.090)),
        mass=4.0,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.348)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.048, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=1.0,
            lower=-0.55,
            upper=1.45,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.395, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.1,
            lower=-0.35,
            upper=2.0,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    shoulder = object_model.get_part("shoulder_carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    bearing_cap = pedestal.get_visual("bearing_cap")
    rotor_flange = shoulder.get_visual("rotor_flange")
    left_yoke = shoulder.get_visual("left_yoke")
    right_yoke = shoulder.get_visual("right_yoke")
    shoulder_trunnion = upper_arm.get_visual("shoulder_trunnion")
    left_elbow_clevis = upper_arm.get_visual("left_elbow_clevis")
    right_elbow_clevis = upper_arm.get_visual("right_elbow_clevis")
    elbow_trunnion = forearm.get_visual("elbow_trunnion")
    left_wrist_clevis = forearm.get_visual("left_wrist_clevis")
    right_wrist_clevis = forearm.get_visual("right_wrist_clevis")
    wrist_trunnion = wrist.get_visual("wrist_trunnion")
    tool_flange = wrist.get_visual("tool_flange")

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

    ctx.expect_contact(
        shoulder,
        pedestal,
        elem_a=rotor_flange,
        elem_b=bearing_cap,
        name="azimuth_rotor_seats_on_bearing_cap",
    )
    ctx.expect_contact(
        upper_arm,
        shoulder,
        elem_a=shoulder_trunnion,
        elem_b=left_yoke,
        name="upper_arm_left_trunnion_supported",
    )
    ctx.expect_contact(
        upper_arm,
        shoulder,
        elem_a=shoulder_trunnion,
        elem_b=right_yoke,
        name="upper_arm_right_trunnion_supported",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=elbow_trunnion,
        elem_b=left_elbow_clevis,
        name="forearm_left_trunnion_supported",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=elbow_trunnion,
        elem_b=right_elbow_clevis,
        name="forearm_right_trunnion_supported",
    )
    ctx.expect_contact(
        wrist,
        forearm,
        elem_a=wrist_trunnion,
        elem_b=left_wrist_clevis,
        name="wrist_left_trunnion_supported",
    )
    ctx.expect_contact(
        wrist,
        forearm,
        elem_a=wrist_trunnion,
        elem_b=right_wrist_clevis,
        name="wrist_right_trunnion_supported",
    )

    rest_wrist_pos = ctx.part_world_position(wrist)
    rest_tool_center = _aabb_center(ctx.part_element_world_aabb(wrist, elem=tool_flange))

    with ctx.pose({base_yaw: math.radians(35.0)}):
        yawed_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "base_yaw_swings_chain_toward_positive_y",
        rest_wrist_pos is not None
        and yawed_wrist_pos is not None
        and yawed_wrist_pos[1] > rest_wrist_pos[1] + 0.18,
        details=f"rest={rest_wrist_pos}, yawed={yawed_wrist_pos}",
    )

    with ctx.pose({shoulder_pitch: 0.75}):
        shoulder_raised_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "shoulder_pitch_raises_wrist_center",
        rest_wrist_pos is not None
        and shoulder_raised_wrist is not None
        and shoulder_raised_wrist[2] > rest_wrist_pos[2] + 0.22,
        details=f"rest={rest_wrist_pos}, raised={shoulder_raised_wrist}",
    )

    with ctx.pose({elbow_pitch: 1.0}):
        elbow_raised_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "elbow_pitch_lifts_distal_chain",
        rest_wrist_pos is not None
        and elbow_raised_wrist is not None
        and elbow_raised_wrist[2] > rest_wrist_pos[2] + 0.10,
        details=f"rest={rest_wrist_pos}, elbow_pose={elbow_raised_wrist}",
    )

    with ctx.pose({wrist_pitch: 0.80}):
        raised_tool_center = _aabb_center(ctx.part_element_world_aabb(wrist, elem=tool_flange))
    ctx.check(
        "wrist_pitch_lifts_tool_flange",
        rest_tool_center is not None
        and raised_tool_center is not None
        and raised_tool_center[2] > rest_tool_center[2] + 0.05,
        details=f"rest={rest_tool_center}, wrist_pose={raised_tool_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
