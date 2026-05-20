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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _yz_section(
    x: float,
    span_y: float,
    span_z: float,
    radius: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for z, y in rounded_rect_profile(
            span_z,
            span_y,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _xy_section(
    z: float,
    span_x: float,
    span_y: float,
    radius: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            span_x,
            span_y,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _loft_yz(name: str, sections: list[tuple[float, float, float, float]]):
    return _save_mesh(
        name,
        section_loft([_yz_section(x, span_y, span_z, radius) for x, span_y, span_z, radius in sections]),
    )


def _loft_xy(name: str, sections: list[tuple[float, float, float, float]]):
    return _save_mesh(
        name,
        section_loft([_xy_section(z, span_x, span_y, radius) for z, span_x, span_y, radius in sections]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_arm", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_shell = model.material("satin_shell", rgba=(0.70, 0.72, 0.75, 1.0))
    joint_titanium = model.material("joint_titanium", rgba=(0.56, 0.59, 0.63, 1.0))
    deep_black = model.material("deep_black", rgba=(0.08, 0.09, 0.10, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.33, 0.35, 0.38, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh(
            "robotic_arm_base_plate.obj",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.36, 0.30, 0.055, corner_segments=12),
                0.034,
            ),
        ),
        material=matte_graphite,
        name="foot_plate",
    )
    base.visual(
        _loft_xy(
            "robotic_arm_base_pedestal.obj",
            [
                (0.034, 0.248, 0.202, 0.040),
                (0.070, 0.194, 0.162, 0.032),
                (0.104, 0.162, 0.138, 0.028),
            ],
        ),
        material=deep_black,
        name="pedestal_shell",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=soft_gray,
        name="core_column",
    )
    base.visual(
        Cylinder(radius=0.076, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=joint_titanium,
        name="bearing_skirt",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=joint_titanium,
        name="upper_bearing",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.30, 0.132)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.056, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=joint_titanium,
        name="turntable_ring",
    )
    carriage.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=matte_graphite,
        name="waist_core",
    )
    carriage.visual(
        Box((0.120, 0.086, 0.028)),
        origin=Origin(xyz=(0.010, 0.0, 0.016)),
        material=matte_graphite,
        name="shoulder_mount_block",
    )
    carriage.visual(
        Box((0.032, 0.058, 0.048)),
        origin=Origin(xyz=(-0.062, 0.0, 0.050)),
        material=matte_graphite,
        name="shoulder_riser",
    )
    carriage.visual(
        Box((0.050, 0.044, 0.060)),
        origin=Origin(xyz=(-0.090, 0.0, 0.076)),
        material=soft_gray,
        name="rear_actuator_housing",
    )
    carriage.visual(
        Box((0.050, 0.018, 0.116)),
        origin=Origin(xyz=(0.000, -0.042, 0.070)),
        material=satin_shell,
        name="shoulder_left_cheek",
    )
    carriage.visual(
        Box((0.050, 0.018, 0.116)),
        origin=Origin(xyz=(0.000, 0.042, 0.070)),
        material=satin_shell,
        name="shoulder_right_cheek",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.000, -0.046, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="shoulder_left_boss",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.000, 0.046, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="shoulder_right_boss",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.15, 0.10, 0.13)),
        mass=8.0,
        origin=Origin(xyz=(0.005, 0.0, 0.065)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.026, length=0.066),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="shoulder_drive_collar",
    )
    upper_arm.visual(
        Box((0.040, 0.040, 0.042)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=matte_graphite,
        name="shoulder_root_block",
    )
    upper_arm.visual(
        _loft_yz(
            "robotic_arm_upper_arm_shell.obj",
            [
                (0.052, 0.084, 0.076, 0.018),
                (0.112, 0.082, 0.074, 0.018),
                (0.182, 0.072, 0.066, 0.016),
                (0.238, 0.058, 0.058, 0.013),
            ],
        ),
        material=satin_shell,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Cylinder(radius=0.018, length=0.132),
        origin=Origin(xyz=(0.128, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_graphite,
        name="upper_actuator_pod",
    )
    upper_arm.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.068, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_titanium,
        name="upper_actuator_front_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.188, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_titanium,
        name="upper_actuator_rear_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.106, -0.014, 0.028)),
        material=soft_gray,
        name="upper_fastener_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.150, 0.014, 0.028)),
        material=soft_gray,
        name="upper_fastener_right",
    )
    upper_arm.visual(
        Box((0.064, 0.008, 0.050)),
        origin=Origin(xyz=(0.232, -0.030, 0.0)),
        material=matte_graphite,
        name="elbow_left_stem",
    )
    upper_arm.visual(
        Box((0.064, 0.008, 0.050)),
        origin=Origin(xyz=(0.232, 0.030, 0.0)),
        material=matte_graphite,
        name="elbow_right_stem",
    )
    upper_arm.visual(
        Box((0.032, 0.018, 0.082)),
        origin=Origin(xyz=(0.272, -0.035, 0.0)),
        material=satin_shell,
        name="elbow_left_cheek",
    )
    upper_arm.visual(
        Box((0.032, 0.018, 0.082)),
        origin=Origin(xyz=(0.272, 0.035, 0.0)),
        material=satin_shell,
        name="elbow_right_cheek",
    )
    upper_arm.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.274, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="elbow_left_collar",
    )
    upper_arm.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.274, 0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="elbow_right_collar",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.31, 0.12, 0.10)),
        mass=9.0,
        origin=Origin(xyz=(0.155, 0.0, 0.006)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.024, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="elbow_drive_collar",
    )
    forearm.visual(
        _loft_yz(
            "robotic_arm_forearm_shell.obj",
            [
                (0.032, 0.068, 0.058, 0.017),
                (0.104, 0.064, 0.054, 0.015),
                (0.182, 0.058, 0.048, 0.013),
                (0.222, 0.038, 0.040, 0.011),
            ],
        ),
        material=satin_shell,
        name="forearm_shell",
    )
    forearm.visual(
        Box((0.116, 0.048, 0.032)),
        origin=Origin(xyz=(0.122, 0.0, -0.022)),
        material=matte_graphite,
        name="forearm_actuator_housing",
    )
    forearm.visual(
        Box((0.014, 0.050, 0.036)),
        origin=Origin(xyz=(0.064, 0.0, -0.022)),
        material=joint_titanium,
        name="forearm_actuator_front_cap",
    )
    forearm.visual(
        Box((0.014, 0.050, 0.036)),
        origin=Origin(xyz=(0.176, 0.0, -0.022)),
        material=joint_titanium,
        name="forearm_actuator_rear_cap",
    )
    forearm.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.106, -0.020, -0.004)),
        material=soft_gray,
        name="forearm_fastener_left",
    )
    forearm.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.140, 0.020, -0.004)),
        material=soft_gray,
        name="forearm_fastener_right",
    )
    forearm.visual(
        Box((0.024, 0.018, 0.040)),
        origin=Origin(xyz=(0.206, -0.021, 0.0)),
        material=matte_graphite,
        name="wrist_left_stem",
    )
    forearm.visual(
        Box((0.024, 0.018, 0.040)),
        origin=Origin(xyz=(0.206, 0.021, 0.0)),
        material=matte_graphite,
        name="wrist_right_stem",
    )
    forearm.visual(
        Box((0.028, 0.016, 0.060)),
        origin=Origin(xyz=(0.238, -0.029, 0.0)),
        material=satin_shell,
        name="wrist_left_cheek",
    )
    forearm.visual(
        Box((0.028, 0.016, 0.060)),
        origin=Origin(xyz=(0.238, 0.029, 0.0)),
        material=satin_shell,
        name="wrist_right_cheek",
    )
    forearm.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.238, -0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="wrist_left_collar",
    )
    forearm.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.238, 0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="wrist_right_collar",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.26, 0.10, 0.08)),
        mass=6.0,
        origin=Origin(xyz=(0.130, 0.0, -0.004)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.020, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_titanium,
        name="wrist_hub",
    )
    wrist.visual(
        _loft_yz(
            "robotic_arm_wrist_shell.obj",
            [
                (0.014, 0.044, 0.048, 0.012),
                (0.046, 0.042, 0.044, 0.010),
                (0.082, 0.036, 0.034, 0.009),
            ],
        ),
        material=satin_shell,
        name="wrist_shell",
    )
    wrist.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.050, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_graphite,
        name="wrist_actuator",
    )
    wrist.visual(
        Cylinder(radius=0.021, length=0.030),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_graphite,
        name="wrist_barrel",
    )
    wrist.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_gray,
        name="wrist_output_shaft",
    )
    wrist.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_titanium,
        name="tool_flange",
    )
    wrist.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.126, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deep_black,
        name="tool_face",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.14, 0.07, 0.07)),
        mass=2.5,
        origin=Origin(xyz=(0.070, 0.0, 0.005)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-2.80, upper=2.80),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=-1.05, upper=0.30),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.274, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.7, lower=-1.45, upper=0.0),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.238, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.2, lower=-1.20, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    upper_bearing = base.get_visual("upper_bearing")
    turntable_ring = carriage.get_visual("turntable_ring")
    shoulder_left_cheek = carriage.get_visual("shoulder_left_cheek")
    shoulder_right_cheek = carriage.get_visual("shoulder_right_cheek")
    shoulder_hub = upper_arm.get_visual("shoulder_hub")
    elbow_left_cheek = upper_arm.get_visual("elbow_left_cheek")
    elbow_right_cheek = upper_arm.get_visual("elbow_right_cheek")
    elbow_hub = forearm.get_visual("elbow_hub")
    wrist_left_cheek = forearm.get_visual("wrist_left_cheek")
    wrist_right_cheek = forearm.get_visual("wrist_right_cheek")
    wrist_hub = wrist.get_visual("wrist_hub")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        carriage,
        base,
        elem_a=turntable_ring,
        elem_b=upper_bearing,
        name="carriage_turntable_contacts_base_bearing",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        min_overlap=0.10,
        elem_a=turntable_ring,
        elem_b=upper_bearing,
        name="carriage_turntable_has_broad_bearing_footprint",
    )

    ctx.expect_gap(
        upper_arm,
        carriage,
        axis="y",
        positive_elem=shoulder_hub,
        negative_elem=shoulder_left_cheek,
        max_gap=0.0005,
        max_penetration=0.0,
        name="shoulder_left_joint_seat",
    )
    ctx.expect_gap(
        carriage,
        upper_arm,
        axis="y",
        positive_elem=shoulder_right_cheek,
        negative_elem=shoulder_hub,
        max_gap=0.0005,
        max_penetration=0.0,
        name="shoulder_right_joint_seat",
    )
    ctx.expect_within(
        upper_arm,
        carriage,
        axes="z",
        inner_elem=shoulder_hub,
        outer_elem=shoulder_left_cheek,
        name="shoulder_axis_stays_within_cheek_height",
    )

    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem=elbow_hub,
        negative_elem=elbow_left_cheek,
        max_gap=0.0005,
        max_penetration=0.0,
        name="elbow_left_joint_seat",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem=elbow_right_cheek,
        negative_elem=elbow_hub,
        max_gap=0.0005,
        max_penetration=0.0,
        name="elbow_right_joint_seat",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="z",
        inner_elem=elbow_hub,
        outer_elem=elbow_left_cheek,
        name="elbow_axis_stays_within_cheek_height",
    )

    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        positive_elem=wrist_hub,
        negative_elem=wrist_left_cheek,
        max_gap=0.0005,
        max_penetration=0.0,
        name="wrist_left_joint_seat",
    )
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        positive_elem=wrist_right_cheek,
        negative_elem=wrist_hub,
        max_gap=0.0005,
        max_penetration=0.0,
        name="wrist_right_joint_seat",
    )
    ctx.expect_within(
        wrist,
        forearm,
        axes="z",
        inner_elem=wrist_hub,
        outer_elem=wrist_left_cheek,
        name="wrist_axis_stays_within_cheek_height",
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    base_aabb = ctx.part_world_aabb(base)
    if base_aabb is None:
        ctx.fail("base_has_measurable_bounds", "Base AABB unavailable in rest pose.")
    else:
        base_size = tuple(base_aabb[1][axis] - base_aabb[0][axis] for axis in range(3))
        ctx.check(
            "base_footprint_is_stable",
            base_size[0] >= 0.34 and base_size[1] >= 0.28 and base_size[2] >= 0.13,
            f"Base extents were {base_size}.",
        )

    tool_face_aabb = ctx.part_element_world_aabb(wrist, elem="tool_face")
    if tool_face_aabb is None:
        ctx.fail("tool_face_is_measurable", "Tool face AABB unavailable in rest pose.")
        return ctx.report()

    rest_tool_center = _aabb_center(tool_face_aabb)
    ctx.check(
        "rest_pose_has_clear_forward_reach",
        rest_tool_center[0] > 0.55 and rest_tool_center[2] > 0.18,
        f"Tool center was {rest_tool_center}.",
    )

    with ctx.pose({base_yaw: 0.80}):
        yaw_tool_aabb = ctx.part_element_world_aabb(wrist, elem="tool_face")
        if yaw_tool_aabb is None:
            ctx.fail("tool_face_measurable_during_base_yaw", "Tool face AABB missing during base yaw.")
        else:
            yaw_tool_center = _aabb_center(yaw_tool_aabb)
            ctx.check(
                "base_yaw_swings_the_reach_sideways",
                yaw_tool_center[1] > rest_tool_center[1] + 0.35
                and yaw_tool_center[0] < rest_tool_center[0] - 0.12,
                f"Rest tool center {rest_tool_center}, yawed center {yaw_tool_center}.",
            )

    with ctx.pose({shoulder_pitch: -0.65}):
        shoulder_tool_aabb = ctx.part_element_world_aabb(wrist, elem="tool_face")
        if shoulder_tool_aabb is None:
            ctx.fail("tool_face_measurable_during_shoulder_pose", "Tool face AABB missing during shoulder pose.")
        else:
            shoulder_tool_center = _aabb_center(shoulder_tool_aabb)
            ctx.check(
                "shoulder_joint_lifts_the_chain",
                shoulder_tool_center[2] > rest_tool_center[2] + 0.18
                and shoulder_tool_center[0] < rest_tool_center[0] - 0.10,
                f"Rest tool center {rest_tool_center}, shoulder-lifted center {shoulder_tool_center}.",
            )

    with ctx.pose({elbow_pitch: -1.05}):
        elbow_tool_aabb = ctx.part_element_world_aabb(wrist, elem="tool_face")
        if elbow_tool_aabb is None:
            ctx.fail("tool_face_measurable_during_elbow_pose", "Tool face AABB missing during elbow pose.")
        else:
            elbow_tool_center = _aabb_center(elbow_tool_aabb)
            ctx.check(
                "elbow_joint_compacts_the_reach",
                elbow_tool_center[2] > rest_tool_center[2] + 0.14
                and elbow_tool_center[0] < rest_tool_center[0] - 0.10,
                f"Rest tool center {rest_tool_center}, elbow-folded center {elbow_tool_center}.",
            )

    with ctx.pose({wrist_pitch: 0.85}):
        wrist_tool_aabb = ctx.part_element_world_aabb(wrist, elem="tool_face")
        if wrist_tool_aabb is None:
            ctx.fail("tool_face_measurable_during_wrist_pose", "Tool face AABB missing during wrist pose.")
        else:
            wrist_tool_center = _aabb_center(wrist_tool_aabb)
            ctx.check(
                "wrist_joint_reorients_the_tool_face",
                wrist_tool_center[2] < rest_tool_center[2] - 0.05,
                f"Rest tool center {rest_tool_center}, wrist-pitched center {wrist_tool_center}.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
