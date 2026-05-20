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


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    width_x: float,
    depth_y: float,
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(center_x + x, center_y + y, z) for x, y in rounded_rect_profile(width_x, depth_y, radius)]


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    *,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, center_y + y, center_z + z) for z, y in rounded_rect_profile(height_z, width_y, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_robotic_arm")

    body_paint = model.material("body_paint", rgba=(0.73, 0.64, 0.20, 1.0))
    service_gray = model.material("service_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Box((0.84, 0.76, 0.62)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )
    pedestal.visual(
        Box((0.84, 0.76, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_steel,
        name="base_plate",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            pedestal.visual(
                Cylinder(radius=0.065, length=0.03),
                origin=Origin(xyz=(0.29 * x_sign, 0.25 * y_sign, 0.015)),
                material=seal_black,
            )
    pedestal_shell = section_loft(
        [
            _xy_section(0.44, 0.36, 0.05, 0.08),
            _xy_section(0.40, 0.33, 0.05, 0.22),
            _xy_section(0.34, 0.28, 0.045, 0.38),
            _xy_section(0.28, 0.24, 0.04, 0.50),
        ]
    )
    pedestal.visual(
        _mesh(pedestal_shell, "pedestal_shell"),
        material=body_paint,
        name="pedestal_shell",
    )
    pedestal.visual(
        Box((0.20, 0.04, 0.23)),
        origin=Origin(xyz=(0.0, -0.155, 0.25)),
        material=service_gray,
        name="pedestal_access_panel",
    )
    pedestal.visual(
        Box((0.04, 0.07, 0.23)),
        origin=Origin(xyz=(0.0, -0.135, 0.25)),
        material=dark_steel,
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=dark_steel,
        name="pedestal_deck",
    )
    pedestal.visual(
        Cylinder(radius=0.19, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=seal_black,
    )

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((0.38, 0.28, 0.66)),
        mass=95.0,
        origin=Origin(xyz=(0.08, 0.0, 0.33)),
    )
    carriage.visual(
        Cylinder(radius=0.22, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="slew_cartridge",
    )
    carriage.visual(
        Cylinder(radius=0.245, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=seal_black,
    )
    carriage.visual(
        Box((0.30, 0.23, 0.38)),
        origin=Origin(xyz=(-0.04, 0.0, 0.26)),
        material=body_paint,
        name="carriage_column",
    )
    carriage.visual(
        Box((0.08, 0.16, 0.12)),
        origin=Origin(xyz=(0.02, 0.0, 0.38)),
        material=body_paint,
        name="shoulder_riser",
    )
    carriage.visual(
        Box((0.18, 0.03, 0.24)),
        origin=Origin(xyz=(0.18, 0.105, 0.52)),
        material=dark_steel,
        name="shoulder_cheek_left",
    )
    carriage.visual(
        Box((0.18, 0.03, 0.24)),
        origin=Origin(xyz=(0.18, -0.105, 0.52)),
        material=dark_steel,
        name="shoulder_cheek_right",
    )
    carriage.visual(
        Box((0.18, 0.22, 0.04)),
        origin=Origin(xyz=(0.18, 0.0, 0.64)),
        material=body_paint,
        name="shoulder_saddle",
    )
    carriage.visual(
        Box((0.12, 0.04, 0.20)),
        origin=Origin(xyz=(-0.12, 0.0, 0.28)),
        material=service_gray,
        name="carriage_service_hatch",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.82, 0.26, 0.30)),
        mass=72.0,
        origin=Origin(xyz=(0.36, 0.0, -0.03)),
    )
    upper_arm.visual(
        Cylinder(radius=0.065, length=0.18),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_cartridge",
    )
    upper_arm.visual(
        Box((0.14, 0.14, 0.18)),
        origin=Origin(xyz=(0.08, 0.0, -0.01)),
        material=dark_steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.18, 0.16, 0.18)),
        origin=Origin(xyz=(0.18, 0.0, -0.03)),
        material=body_paint,
        name="upper_arm_root_block",
    )
    upper_arm.visual(
        Box((0.38, 0.16, 0.16)),
        origin=Origin(xyz=(0.44, 0.0, -0.03)),
        material=body_paint,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.24, 0.14, 0.06)),
        origin=Origin(xyz=(0.36, 0.0, 0.06)),
        material=service_gray,
        name="upper_arm_access_cover",
    )
    upper_arm.visual(
        Box((0.40, 0.10, 0.03)),
        origin=Origin(xyz=(0.40, 0.0, -0.125)),
        material=dark_steel,
        name="upper_arm_wear_strip",
    )
    upper_arm.visual(
        Box((0.12, 0.04, 0.18)),
        origin=Origin(xyz=(0.70, 0.10, -0.05)),
        material=dark_steel,
        name="elbow_cheek_left",
    )
    upper_arm.visual(
        Box((0.12, 0.04, 0.18)),
        origin=Origin(xyz=(0.70, -0.10, -0.05)),
        material=dark_steel,
        name="elbow_cheek_right",
    )
    upper_arm.visual(
        Box((0.12, 0.18, 0.05)),
        origin=Origin(xyz=(0.66, 0.0, 0.05)),
        material=dark_steel,
        name="elbow_top_bridge",
    )
    upper_arm.visual(
        Box((0.08, 0.12, 0.04)),
        origin=Origin(xyz=(0.58, 0.0, -0.11)),
        material=dark_steel,
        name="elbow_bridge",
    )

    forearm = model.part("forearm")
    forearm.inertial = Inertial.from_geometry(
        Box((0.68, 0.22, 0.24)),
        mass=44.0,
        origin=Origin(xyz=(0.30, 0.0, 0.01)),
    )
    forearm.visual(
        Cylinder(radius=0.065, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_cartridge",
    )
    forearm.visual(
        Box((0.10, 0.13, 0.16)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_hub",
    )
    forearm_shell = section_loft(
        [
            _yz_section(0.20, 0.22, 0.035, 0.05, center_z=0.00),
            _yz_section(0.18, 0.20, 0.03, 0.24, center_z=0.01),
            _yz_section(0.16, 0.18, 0.028, 0.44, center_z=0.02),
            _yz_section(0.14, 0.16, 0.025, 0.50, center_z=0.03),
        ]
    )
    forearm.visual(
        _mesh(forearm_shell, "forearm_shell"),
        material=body_paint,
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.22, 0.018, 0.12)),
        origin=Origin(xyz=(0.28, 0.084, 0.01)),
        material=service_gray,
        name="forearm_left_cover",
    )
    forearm.visual(
        Box((0.22, 0.018, 0.12)),
        origin=Origin(xyz=(0.28, -0.084, 0.01)),
        material=service_gray,
        name="forearm_right_cover",
    )
    forearm.visual(
        Box((0.26, 0.07, 0.024)),
        origin=Origin(xyz=(0.34, 0.0, -0.095)),
        material=dark_steel,
        name="forearm_wear_strip",
    )
    forearm.visual(
        Box((0.08, 0.04, 0.16)),
        origin=Origin(xyz=(0.58, 0.08, 0.03)),
        material=dark_steel,
        name="wrist_cheek_left",
    )
    forearm.visual(
        Box((0.08, 0.04, 0.16)),
        origin=Origin(xyz=(0.58, -0.08, 0.03)),
        material=dark_steel,
        name="wrist_cheek_right",
    )
    forearm.visual(
        Box((0.08, 0.16, 0.05)),
        origin=Origin(xyz=(0.50, 0.0, -0.075)),
        material=dark_steel,
        name="wrist_bridge",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.22, 0.14, 0.14)),
        mass=18.0,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )
    wrist_head.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_cartridge",
    )
    wrist_head.visual(
        Box((0.14, 0.12, 0.12)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=body_paint,
        name="wrist_housing",
    )
    wrist_head.visual(
        Box((0.09, 0.10, 0.02)),
        origin=Origin(xyz=(0.10, 0.0, 0.07)),
        material=service_gray,
        name="wrist_service_cover",
    )
    wrist_head.visual(
        Cylinder(radius=0.06, length=0.08),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_support_collar",
    )

    tool_flange = model.part("tool_flange")
    tool_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.14),
        mass=9.0,
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    tool_flange.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_shaft",
    )
    tool_flange.visual(
        Cylinder(radius=0.095, length=0.028),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=service_gray,
        name="flange_plate",
    )
    tool_flange.visual(
        Box((0.024, 0.18, 0.18)),
        origin=Origin(xyz=(0.111, 0.0, 0.0)),
        material=dark_steel,
        name="tool_face_pad",
    )
    tool_flange.visual(
        Box((0.018, 0.07, 0.07)),
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        material=seal_black,
        name="wear_block",
    )

    model.articulation(
        "pedestal_to_carriage_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=1.0,
            lower=-math.radians(175.0),
            upper=math.radians(175.0),
        ),
    )
    model.articulation(
        "carriage_to_upper_arm_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.18, 0.0, 0.52)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-1.15,
            upper=1.35,
        ),
    )
    model.articulation(
        "upper_arm_to_forearm_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.74, 0.0, -0.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.5,
            lower=-1.55,
            upper=1.95,
        ),
    )
    model.articulation(
        "forearm_to_wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.60, 0.0, 0.03)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=75.0,
            velocity=2.2,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "wrist_to_tool_roll",
        ArticulationType.CONTINUOUS,
        parent=wrist_head,
        child=tool_flange,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    tool_flange = object_model.get_part("tool_flange")

    base_yaw = object_model.get_articulation("pedestal_to_carriage_yaw")
    shoulder = object_model.get_articulation("carriage_to_upper_arm_shoulder")
    elbow = object_model.get_articulation("upper_arm_to_forearm_elbow")
    wrist_pitch = object_model.get_articulation("forearm_to_wrist_pitch")
    wrist_roll = object_model.get_articulation("wrist_to_tool_roll")

    pedestal_deck = pedestal.get_visual("pedestal_deck")
    slew_cartridge = carriage.get_visual("slew_cartridge")
    roll_support_collar = wrist_head.get_visual("roll_support_collar")
    roll_shaft = tool_flange.get_visual("roll_shaft")

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
        pedestal,
        carriage,
        elem_a=pedestal_deck,
        elem_b=slew_cartridge,
        name="slew_bearing_is_seated_on_pedestal",
    )
    ctx.expect_contact(carriage, upper_arm, name="shoulder_cartridge_captured_in_yoke")
    ctx.expect_contact(upper_arm, forearm, name="elbow_cartridge_captured_in_yoke")
    ctx.expect_contact(forearm, wrist_head, name="wrist_pitch_cartridge_captured_in_yoke")
    ctx.expect_contact(
        wrist_head,
        tool_flange,
        elem_a=roll_support_collar,
        elem_b=roll_shaft,
        name="tool_roll_shaft_supported_by_collar",
    )

    ctx.expect_origin_gap(
        upper_arm,
        carriage,
        axis="x",
        min_gap=0.17,
        max_gap=0.19,
        name="shoulder_axis_offset_is_readable",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=0.73,
        max_gap=0.75,
        name="elbow_offset_is_readable",
    )
    ctx.expect_origin_gap(
        wrist_head,
        forearm,
        axis="x",
        min_gap=0.59,
        max_gap=0.61,
        name="wrist_offset_is_readable",
    )

    ctx.check(
        "joint_axis_order_matches_service_arm_layout",
        base_yaw.axis == (0.0, 0.0, 1.0)
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_roll.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes were yaw={base_yaw.axis}, shoulder={shoulder.axis}, "
            f"elbow={elbow.axis}, wrist_pitch={wrist_pitch.axis}, wrist_roll={wrist_roll.axis}"
        ),
    )

    rest_tool = ctx.part_world_position(tool_flange)
    if rest_tool is None:
        ctx.fail("tool_flange_position_available", "tool flange position was unavailable in rest pose")
    else:
        with ctx.pose({base_yaw: math.pi / 2.0}):
            yaw_tool = ctx.part_world_position(tool_flange)
        ctx.check(
            "base_yaw_swings_reach_around_pedestal",
            yaw_tool is not None and yaw_tool[1] > 1.3 and abs(yaw_tool[0]) < 0.25,
            details=f"rest={rest_tool}, yaw_pose={yaw_tool}",
        )

        with ctx.pose({shoulder: 0.70}):
            shoulder_tool = ctx.part_world_position(tool_flange)
        ctx.check(
            "positive_shoulder_pitch_raises_arm",
            shoulder_tool is not None and shoulder_tool[2] > rest_tool[2] + 0.75,
            details=f"rest={rest_tool}, shoulder_pose={shoulder_tool}",
        )

        with ctx.pose({shoulder: 0.25, elbow: 0.0}):
            elbow_rest = ctx.part_world_position(tool_flange)
        with ctx.pose({shoulder: 0.25, elbow: 1.00}):
            elbow_lift = ctx.part_world_position(tool_flange)
        ctx.check(
            "positive_elbow_pitch_lifts_forearm",
            elbow_rest is not None and elbow_lift is not None and elbow_lift[2] > elbow_rest[2] + 0.30,
            details=f"elbow_rest={elbow_rest}, elbow_lift={elbow_lift}",
        )

        with ctx.pose({shoulder: 0.35, elbow: 0.55, wrist_pitch: 0.0}):
            wrist_rest = ctx.part_world_position(tool_flange)
        with ctx.pose({shoulder: 0.35, elbow: 0.55, wrist_pitch: 0.85}):
            wrist_tip = ctx.part_world_position(tool_flange)
        ctx.check(
            "positive_wrist_pitch_changes_tool_approach",
            wrist_rest is not None
            and wrist_tip is not None
            and wrist_tip[2] > wrist_rest[2] + 0.02
            and wrist_tip[0] < wrist_rest[0] - 0.10,
            details=f"wrist_rest={wrist_rest}, wrist_tip={wrist_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
