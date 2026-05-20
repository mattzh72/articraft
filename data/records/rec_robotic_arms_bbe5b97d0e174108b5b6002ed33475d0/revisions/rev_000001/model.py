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
)


def _add_top_bolts(
    part,
    *,
    xs: tuple[float, ...],
    ys: tuple[float, ...],
    top_z: float,
    material,
    prefix: str,
    radius: float = 0.010,
    length: float = 0.012,
    embed: float = 0.002,
) -> None:
    for ix, x in enumerate(xs):
        for iy, y in enumerate(ys):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, top_z + 0.5 * length - embed)),
                material=material,
                name=f"{prefix}_{ix}_{iy}",
            )


def _add_face_bolts_x(
    part,
    *,
    x_face: float,
    yz_positions: tuple[tuple[float, float], ...],
    material,
    prefix: str,
    radius: float = 0.007,
    length: float = 0.010,
    embed: float = 0.002,
) -> None:
    for index, (y, z) in enumerate(yz_positions):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(x_face + 0.5 * length - embed, y, z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_first_industrial_robot_arm")

    structure = model.material("structure", rgba=(0.28, 0.30, 0.33, 1.0))
    cartridge = model.material("cartridge", rgba=(0.12, 0.13, 0.15, 1.0))
    guard = model.material("guard", rgba=(0.90, 0.74, 0.10, 1.0))
    fastener = model.material("fastener", rgba=(0.72, 0.74, 0.77, 1.0))
    lockout = model.material("lockout", rgba=(0.78, 0.14, 0.12, 1.0))
    panel = model.material("panel", rgba=(0.44, 0.47, 0.50, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.92, 0.74, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=structure,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.34, 0.28, 0.57)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=structure,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.48, 0.36, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=structure,
        name="pedestal_cap",
    )
    pedestal.visual(
        Cylinder(radius=0.23, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=cartridge,
        name="yaw_ring",
    )
    pedestal.visual(
        Box((0.16, 0.06, 0.36)),
        origin=Origin(xyz=(0.20, 0.0, 0.230)),
        material=structure,
        name="front_rib",
    )
    pedestal.visual(
        Box((0.16, 0.06, 0.36)),
        origin=Origin(xyz=(-0.20, 0.0, 0.230)),
        material=structure,
        name="rear_rib",
    )
    pedestal.visual(
        Box((0.06, 0.16, 0.36)),
        origin=Origin(xyz=(0.0, 0.20, 0.230)),
        material=structure,
        name="left_rib",
    )
    pedestal.visual(
        Box((0.06, 0.16, 0.36)),
        origin=Origin(xyz=(0.0, -0.20, 0.230)),
        material=structure,
        name="right_rib",
    )
    pedestal.visual(
        Box((0.18, 0.008, 0.22)),
        origin=Origin(xyz=(0.10, 0.144, 0.280)),
        material=panel,
        name="service_panel",
    )
    pedestal.visual(
        Box((0.08, 0.10, 0.24)),
        origin=Origin(xyz=(-0.16, -0.09, 0.170)),
        material=panel,
        name="rear_conduit_guard",
    )
    _add_top_bolts(
        pedestal,
        xs=(-0.34, 0.34),
        ys=(-0.25, 0.25),
        top_z=0.05,
        material=fastener,
        prefix="anchor_bolt",
        radius=0.014,
        length=0.016,
        embed=0.003,
    )
    _add_top_bolts(
        pedestal,
        xs=(-0.18, 0.18),
        ys=(-0.12, 0.12),
        top_z=0.67,
        material=fastener,
        prefix="yaw_ring_bolt",
        radius=0.010,
        length=0.012,
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.92, 0.74, 0.67)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.205, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cartridge,
        name="yaw_ring",
    )
    turret.visual(
        Box((0.48, 0.36, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=structure,
        name="rotary_table",
    )
    turret.visual(
        Box((0.18, 0.26, 0.08)),
        origin=Origin(xyz=(-0.14, 0.0, 0.160)),
        material=structure,
        name="table_skirt",
    )
    turret.visual(
        Box((0.12, 0.20, 0.18)),
        origin=Origin(xyz=(-0.19, 0.0, 0.280)),
        material=guard,
        name="rear_joint_guard",
    )
    turret.visual(
        Box((0.10, 0.06, 0.30)),
        origin=Origin(xyz=(-0.06, 0.160, 0.350)),
        material=structure,
        name="shoulder_left_cheek",
    )
    turret.visual(
        Box((0.10, 0.06, 0.30)),
        origin=Origin(xyz=(-0.06, -0.160, 0.350)),
        material=structure,
        name="shoulder_right_cheek",
    )
    turret.visual(
        Box((0.10, 0.38, 0.03)),
        origin=Origin(xyz=(-0.06, 0.0, 0.455)),
        material=guard,
        name="shoulder_bridge",
    )
    turret.visual(
        Box((0.10, 0.30, 0.14)),
        origin=Origin(xyz=(-0.10, 0.0, 0.390)),
        material=structure,
        name="shoulder_backplate",
    )
    turret.visual(
        Box((0.08, 0.10, 0.05)),
        origin=Origin(xyz=(-0.08, 0.0, 0.210)),
        material=lockout,
        name="shoulder_overtravel_stop",
    )
    turret.visual(
        Box((0.05, 0.02, 0.14)),
        origin=Origin(xyz=(-0.08, 0.185, 0.360)),
        material=lockout,
        name="shoulder_lockout_tab",
    )
    _add_top_bolts(
        turret,
        xs=(-0.14, 0.14),
        ys=(-0.10, 0.10),
        top_z=0.18,
        material=fastener,
        prefix="table_bolt",
        radius=0.009,
        length=0.012,
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.50, 0.40, 0.56)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.100, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.18, 0.22, 0.14)),
        origin=Origin(xyz=(0.08, 0.0, 0.000)),
        material=guard,
        name="shoulder_guard_shell",
    )
    upper_arm.visual(
        Box((0.52, 0.16, 0.16)),
        origin=Origin(xyz=(0.34, 0.0, -0.010)),
        material=structure,
        name="main_beam",
    )
    upper_arm.visual(
        Box((0.30, 0.18, 0.03)),
        origin=Origin(xyz=(0.35, 0.0, 0.085)),
        material=guard,
        name="top_cover",
    )
    upper_arm.visual(
        Box((0.30, 0.15, 0.035)),
        origin=Origin(xyz=(0.27, 0.0, -0.1075)),
        material=structure,
        name="lower_tie_plate",
    )
    upper_arm.visual(
        Box((0.18, 0.02, 0.22)),
        origin=Origin(xyz=(0.11, 0.090, -0.010)),
        material=structure,
        name="left_shoulder_web",
    )
    upper_arm.visual(
        Box((0.18, 0.02, 0.22)),
        origin=Origin(xyz=(0.11, -0.090, -0.010)),
        material=structure,
        name="right_shoulder_web",
    )
    upper_arm.visual(
        Box((0.16, 0.20, 0.18)),
        origin=Origin(xyz=(0.59, 0.0, -0.020)),
        material=structure,
        name="elbow_mount_block",
    )
    upper_arm.visual(
        Box((0.12, 0.22, 0.03)),
        origin=Origin(xyz=(0.60, 0.0, 0.085)),
        material=guard,
        name="elbow_guard_cap",
    )
    upper_arm.visual(
        Box((0.12, 0.18, 0.03)),
        origin=Origin(xyz=(0.60, 0.0, -0.125)),
        material=structure,
        name="elbow_base_plate",
    )
    upper_arm.visual(
        Box((0.07, 0.12, 0.04)),
        origin=Origin(xyz=(0.180, 0.0, -0.130)),
        material=lockout,
        name="shoulder_stop_lug",
    )
    upper_arm.visual(
        Box((0.05, 0.02, 0.08)),
        origin=Origin(xyz=(0.57, -0.100, 0.050)),
        material=lockout,
        name="elbow_lockout_tab",
    )
    _add_top_bolts(
        upper_arm,
        xs=(0.26, 0.34, 0.42),
        ys=(-0.055, 0.055),
        top_z=0.10,
        material=fastener,
        prefix="upper_cover_bolt",
        radius=0.008,
        length=0.010,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.78, 0.30, 0.32)),
        mass=165.0,
        origin=Origin(xyz=(0.34, 0.0, -0.010)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.070, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.14, 0.18, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, -0.060)),
        material=guard,
        name="elbow_guard_shell",
    )
    forearm.visual(
        Box((0.44, 0.12, 0.12)),
        origin=Origin(xyz=(0.29, 0.0, -0.040)),
        material=structure,
        name="main_beam",
    )
    forearm.visual(
        Box((0.22, 0.14, 0.025)),
        origin=Origin(xyz=(0.30, 0.0, 0.0325)),
        material=guard,
        name="top_rib",
    )
    forearm.visual(
        Box((0.24, 0.14, 0.03)),
        origin=Origin(xyz=(0.24, 0.0, -0.105)),
        material=structure,
        name="bottom_plate",
    )
    forearm.visual(
        Box((0.18, 0.02, 0.18)),
        origin=Origin(xyz=(0.12, 0.060, -0.040)),
        material=structure,
        name="left_elbow_web",
    )
    forearm.visual(
        Box((0.18, 0.02, 0.18)),
        origin=Origin(xyz=(0.12, -0.060, -0.040)),
        material=structure,
        name="right_elbow_web",
    )
    forearm.visual(
        Box((0.14, 0.18, 0.16)),
        origin=Origin(xyz=(0.47, 0.0, -0.030)),
        material=structure,
        name="wrist_mount_block",
    )
    forearm.visual(
        Box((0.04, 0.16, 0.12)),
        origin=Origin(xyz=(0.54, 0.0, -0.030)),
        material=structure,
        name="wrist_seat_pad",
    )
    forearm.visual(
        Box((0.10, 0.20, 0.02)),
        origin=Origin(xyz=(0.50, 0.0, 0.075)),
        material=guard,
        name="wrist_guard_cap",
    )
    forearm.visual(
        Box((0.10, 0.18, 0.02)),
        origin=Origin(xyz=(0.50, 0.0, -0.120)),
        material=structure,
        name="wrist_base_plate",
    )
    forearm.visual(
        Box((0.07, 0.10, 0.04)),
        origin=Origin(xyz=(0.12, 0.0, -0.125)),
        material=lockout,
        name="elbow_stop_lug",
    )
    forearm.visual(
        Box((0.05, 0.02, 0.08)),
        origin=Origin(xyz=(0.46, 0.100, 0.040)),
        material=lockout,
        name="wrist_lockout_tab",
    )
    _add_top_bolts(
        forearm,
        xs=(0.22, 0.30, 0.38),
        ys=(-0.05, 0.05),
        top_z=0.045,
        material=fastener,
        prefix="forearm_rib_bolt",
        radius=0.0075,
        length=0.010,
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.66, 0.24, 0.26)),
        mass=112.0,
        origin=Origin(xyz=(0.28, 0.0, -0.040)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.060, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cartridge,
        name="wrist_hub",
    )
    wrist_head.visual(
        Box((0.16, 0.10, 0.10)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=structure,
        name="wrist_housing",
    )
    wrist_head.visual(
        Box((0.10, 0.12, 0.025)),
        origin=Origin(xyz=(0.11, 0.0, 0.0625)),
        material=guard,
        name="service_cover",
    )
    wrist_head.visual(
        Box((0.12, 0.12, 0.025)),
        origin=Origin(xyz=(0.12, 0.0, -0.0625)),
        material=structure,
        name="underside_guard_plate",
    )
    wrist_head.visual(
        Box((0.08, 0.08, 0.08)),
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
        material=structure,
        name="tool_neck",
    )
    wrist_head.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.265, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cartridge,
        name="tool_flange_body",
    )
    wrist_head.visual(
        Cylinder(radius=0.065, length=0.008),
        origin=Origin(xyz=(0.284, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cartridge,
        name="tool_flange_face",
    )
    wrist_head.visual(
        Box((0.14, 0.02, 0.12)),
        origin=Origin(xyz=(0.18, 0.070, 0.0)),
        material=guard,
        name="left_guard_rail",
    )
    wrist_head.visual(
        Box((0.14, 0.02, 0.12)),
        origin=Origin(xyz=(0.18, -0.070, 0.0)),
        material=guard,
        name="right_guard_rail",
    )
    wrist_head.visual(
        Box((0.02, 0.16, 0.12)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=guard,
        name="front_guard_bar",
    )
    wrist_head.visual(
        Box((0.06, 0.02, 0.12)),
        origin=Origin(xyz=(0.09, 0.070, 0.0)),
        material=guard,
        name="left_guard_standoff",
    )
    wrist_head.visual(
        Box((0.06, 0.02, 0.12)),
        origin=Origin(xyz=(0.09, -0.070, 0.0)),
        material=guard,
        name="right_guard_standoff",
    )
    wrist_head.visual(
        Box((0.05, 0.07, 0.025)),
        origin=Origin(xyz=(0.05, 0.0, 0.0725)),
        material=lockout,
        name="wrist_overtravel_stop",
    )
    _add_top_bolts(
        wrist_head,
        xs=(0.08, 0.14),
        ys=(-0.04, 0.04),
        top_z=0.075,
        material=fastener,
        prefix="service_cover_bolt",
        radius=0.0065,
        length=0.009,
    )
    _add_face_bolts_x(
        wrist_head,
        x_face=0.288,
        yz_positions=((-0.026, -0.026), (-0.026, 0.026), (0.026, -0.026), (0.026, 0.026)),
        material=fastener,
        prefix="tool_flange_bolt",
        radius=0.006,
        length=0.010,
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.31, 0.18, 0.16)),
        mass=46.0,
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
    )

    base_yaw = model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.7,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )

    shoulder = model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.090, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.8,
            lower=-math.radians(55.0),
            upper=math.radians(80.0),
        ),
    )

    elbow = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.740, 0.0, -0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.0,
            lower=-math.radians(120.0),
            upper=math.radians(70.0),
        ),
    )

    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.620, 0.0, -0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=420.0,
            velocity=1.5,
            lower=-math.radians(85.0),
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")

    ctx.expect_contact(turret, pedestal, name="turret_supported_on_pedestal")
    ctx.expect_contact(upper_arm, turret, name="shoulder_cartridge_seated")
    ctx.expect_contact(forearm, upper_arm, name="elbow_cartridge_seated")
    ctx.expect_contact(wrist_head, forearm, name="wrist_cartridge_seated")

    ctx.check(
        "axis_order_readable",
        base_yaw.axis == (0.0, 0.0, 1.0)
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (0.0, -1.0, 0.0),
        details=(
            f"axes were yaw={base_yaw.axis}, shoulder={shoulder.axis}, "
            f"elbow={elbow.axis}, wrist={wrist.axis}"
        ),
    )

    with ctx.pose({base_yaw: 0.0, shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        rest_wrist_center = _aabb_center(ctx.part_world_aabb(wrist_head))
    with ctx.pose({base_yaw: 0.0, shoulder: 0.65, elbow: 0.0, wrist: 0.0}):
        shoulder_lift_center = _aabb_center(ctx.part_world_aabb(wrist_head))
    ctx.check(
        "positive_shoulder_lifts_wrist",
        rest_wrist_center is not None
        and shoulder_lift_center is not None
        and shoulder_lift_center[2] > rest_wrist_center[2] + 0.18,
        details=f"rest={rest_wrist_center}, lifted={shoulder_lift_center}",
    )

    with ctx.pose({base_yaw: 0.0, shoulder: 0.30, elbow: 0.0, wrist: 0.0}):
        elbow_rest_center = _aabb_center(ctx.part_world_aabb(wrist_head))
    with ctx.pose({base_yaw: 0.0, shoulder: 0.30, elbow: 0.70, wrist: 0.0}):
        elbow_lift_center = _aabb_center(ctx.part_world_aabb(wrist_head))
    ctx.check(
        "positive_elbow_lifts_tooling",
        elbow_rest_center is not None
        and elbow_lift_center is not None
        and elbow_lift_center[2] > elbow_rest_center[2] + 0.10,
        details=f"rest={elbow_rest_center}, lifted={elbow_lift_center}",
    )

    with ctx.pose({base_yaw: 0.0, shoulder: 0.25, elbow: 0.35, wrist: 0.0}):
        yaw_zero_center = _aabb_center(ctx.part_world_aabb(wrist_head))
    with ctx.pose({base_yaw: 0.60, shoulder: 0.25, elbow: 0.35, wrist: 0.0}):
        yaw_swept_center = _aabb_center(ctx.part_world_aabb(wrist_head))
    ctx.check(
        "base_yaw_moves_arm_laterally",
        yaw_zero_center is not None
        and yaw_swept_center is not None
        and abs(yaw_swept_center[1] - yaw_zero_center[1]) > 0.35,
        details=f"yaw0={yaw_zero_center}, yaw0.6={yaw_swept_center}",
    )

    with ctx.pose({base_yaw: 0.0, shoulder: 0.35, elbow: 0.25, wrist: 0.0}):
        flange_flat = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_flange_face"))
    with ctx.pose({base_yaw: 0.0, shoulder: 0.35, elbow: 0.25, wrist: 0.65}):
        flange_tilted = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_flange_face"))
    ctx.check(
        "positive_wrist_lifts_flange_face",
        flange_flat is not None
        and flange_tilted is not None
        and flange_tilted[2] > flange_flat[2] + 0.03,
        details=f"flat={flange_flat}, tilted={flange_tilted}",
    )

    with ctx.pose({base_yaw: 0.35, shoulder: 0.55, elbow: 0.55, wrist: -0.25}):
        ctx.fail_if_parts_overlap_in_current_pose(name="working_pose_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
