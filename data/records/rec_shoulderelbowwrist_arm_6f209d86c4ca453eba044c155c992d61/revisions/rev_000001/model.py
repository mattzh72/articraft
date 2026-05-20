from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_PLATE_LENGTH = 0.260
BASE_PLATE_WIDTH = 0.220
BASE_PLATE_THICKNESS = 0.028
BASE_PLINTH_HEIGHT = 0.075
BASE_BEARING_HEIGHT = 0.016
SHOULDER_JOINT_Z = BASE_PLATE_THICKNESS + BASE_PLINTH_HEIGHT + BASE_BEARING_HEIGHT

PEDESTAL_DRUM_RADIUS = 0.051
PEDESTAL_DRUM_HEIGHT = 0.040
PEDESTAL_MOUNT_X = 0.082
PEDESTAL_MOUNT_Z = 0.092

UPPER_ARM_LENGTH = 0.205
UPPER_ARM_CLEVIS_WIDTH = 0.072
UPPER_ARM_CLEVIS_SLOT = 0.052

FOREARM_LENGTH = 0.175
FOREARM_BARREL_WIDTH = 0.052
FOREARM_BARREL_RADIUS = 0.029
WRIST_BODY_LENGTH = 0.072
WRIST_FLANGE_START = 0.070


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _bench_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            BASE_PLATE_LENGTH,
            BASE_PLATE_WIDTH,
            BASE_PLATE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.014)
    )

    plinth = (
        cq.Workplane("XY")
        .workplane(offset=BASE_PLATE_THICKNESS)
        .box(0.168, 0.148, BASE_PLINTH_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .chamfer(0.016)
    )

    bearing_ring = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_JOINT_Z - BASE_BEARING_HEIGHT)
        .circle(0.061)
        .extrude(BASE_BEARING_HEIGHT)
        .cut(
            cq.Workplane("XY")
            .workplane(offset=SHOULDER_JOINT_Z - BASE_BEARING_HEIGHT + 0.004)
            .circle(0.041)
            .extrude(BASE_BEARING_HEIGHT)
        )
    )

    access_panel = (
        cq.Workplane("XZ")
        .workplane(offset=BASE_PLATE_WIDTH / 2.0 - 0.003)
        .center(0.0, 0.050)
        .rect(0.072, 0.034)
        .extrude(0.006)
    )

    return plate.union(plinth).union(bearing_ring).union(access_panel)


def _shoulder_pedestal_shape() -> cq.Workplane:
    drum = cq.Workplane("XY").circle(PEDESTAL_DRUM_RADIUS).extrude(PEDESTAL_DRUM_HEIGHT)

    lower_casting = (
        cq.Workplane("XY")
        .workplane(offset=0.026)
        .box(0.108, 0.102, 0.080, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.017)
    )

    upper_cap = (
        cq.Workplane("XY")
        .workplane(offset=0.094)
        .box(0.088, 0.084, 0.048, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.013)
    )

    arm_boss = (
        cq.Workplane("YZ")
        .workplane(offset=-0.010)
        .center(0.0, PEDESTAL_MOUNT_Z)
        .circle(0.039)
        .extrude(PEDESTAL_MOUNT_X + 0.010)
    )

    return drum.union(lower_casting).union(upper_cap).union(arm_boss)


def _upper_arm_shape() -> cq.Workplane:
    mount_flange = (
        cq.Workplane("XY")
        .box(0.012, 0.066, 0.072, centered=(False, True, True))
        .edges("|X")
        .fillet(0.006)
    )

    shoulder_block = (
        cq.Workplane("XY")
        .box(0.052, 0.074, 0.080, centered=(False, True, True))
        .edges("|X")
        .fillet(0.010)
        .translate((0.018, 0.0, 0.0))
    )

    beam = (
        cq.Workplane("XY")
        .box(0.096, 0.046, 0.040, centered=(False, True, True))
        .edges("|X")
        .fillet(0.007)
        .translate((0.078, 0.0, 0.0))
    )

    elbow_plate = (
        cq.Workplane("XY")
        .box(0.014, 0.050, 0.050, centered=(False, True, True))
        .edges("|X")
        .fillet(0.004)
        .translate((UPPER_ARM_LENGTH - 0.020, 0.0, 0.0))
    )

    elbow_barrel = (
        cq.Workplane("XZ")
        .center(UPPER_ARM_LENGTH - 0.028, 0.0)
        .circle(0.022)
        .extrude(0.044, both=True)
    )

    service_boss = (
        cq.Workplane("XY")
        .box(0.024, 0.028, 0.010, centered=(False, True, True))
        .translate((0.112, 0.0, 0.024))
    )

    return mount_flange.union(shoulder_block).union(beam).union(elbow_plate).union(elbow_barrel).union(service_boss)


def _forearm_shape() -> cq.Workplane:
    elbow_plate = (
        cq.Workplane("XY")
        .box(0.012, 0.046, 0.046, centered=(False, True, True))
        .edges("|X")
        .fillet(0.004)
    )

    elbow_barrel = (
        cq.Workplane("XZ")
        .center(0.018, 0.0)
        .circle(0.022)
        .extrude(0.040, both=True)
    )

    beam = (
        cq.Workplane("XY")
        .box(0.102, 0.034, 0.030, centered=(False, True, True))
        .edges("|X")
        .fillet(0.005)
        .translate((0.042, 0.0, 0.0))
    )

    wrist_plate = (
        cq.Workplane("XY")
        .box(0.014, 0.042, 0.042, centered=(False, True, True))
        .edges("|X")
        .fillet(0.004)
        .translate((FOREARM_LENGTH - 0.020, 0.0, 0.0))
    )

    wrist_barrel = (
        cq.Workplane("XZ")
        .center(FOREARM_LENGTH - 0.022, 0.0)
        .circle(0.018)
        .extrude(0.036, both=True)
    )

    return elbow_plate.union(elbow_barrel).union(beam).union(wrist_plate).union(wrist_barrel)


def _wrist_body_shape() -> cq.Workplane:
    rear_collar = cq.Workplane("YZ").workplane(offset=0.0).circle(0.018).extrude(0.014)
    mid_body = cq.Workplane("YZ").workplane(offset=0.012).circle(0.021).extrude(0.034)
    nose = cq.Workplane("YZ").workplane(offset=0.046).circle(0.018).extrude(0.026)
    face_pad = cq.Workplane("YZ").workplane(offset=0.0).circle(0.010).extrude(0.004)
    return rear_collar.union(mid_body).union(nose).union(face_pad)


def _tool_flange_shape() -> cq.Workplane:
    flange = cq.Workplane("YZ").workplane(offset=WRIST_FLANGE_START).circle(0.032).extrude(0.010)
    pilot = cq.Workplane("YZ").workplane(offset=WRIST_FLANGE_START - 0.001).circle(0.006).extrude(0.012)
    bolt_pattern = (
        cq.Workplane("YZ")
        .workplane(offset=WRIST_FLANGE_START - 0.001)
        .pushPoints([(0.020, 0.0), (-0.020, 0.0), (0.0, 0.020), (0.0, -0.020)])
        .circle(0.003)
        .extrude(0.012)
    )
    nose_register = cq.Workplane("YZ").workplane(offset=WRIST_FLANGE_START + 0.010).circle(0.015).extrude(0.004)
    return flange.cut(pilot).cut(bolt_pattern).union(nose_register)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_robot_arm")

    model.material("machine_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("cast_orange", rgba=(0.91, 0.48, 0.12, 1.0))
    model.material("cap_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("machined_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    bench_base = model.part("bench_base")
    bench_base.visual(_mesh(_bench_base_shape(), "bench_base_shell"), material="machine_gray", name="bench_shell")
    bench_base.inertial = Inertial.from_geometry(
        Box((0.260, 0.220, SHOULDER_JOINT_Z)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_JOINT_Z / 2.0)),
    )

    shoulder_pedestal = model.part("shoulder_pedestal")
    shoulder_pedestal.visual(
        _mesh(_shoulder_pedestal_shape(), "shoulder_pedestal_shell"),
        material="cast_orange",
        name="pedestal_shell",
    )
    shoulder_pedestal.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.015, 0.048, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cap_black",
        name="pedestal_cap_right",
    )
    shoulder_pedestal.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.015, -0.048, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cap_black",
        name="pedestal_cap_left",
    )
    shoulder_pedestal.inertial = Inertial.from_geometry(
        Box((0.120, 0.110, 0.145)),
        mass=4.0,
        origin=Origin(xyz=(0.010, 0.0, 0.072)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(_mesh(_upper_arm_shape(), "upper_arm_shell"), material="cast_orange", name="upper_arm_shell")
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.072, 0.075)),
        mass=2.3,
        origin=Origin(xyz=(UPPER_ARM_LENGTH * 0.48, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(_mesh(_forearm_shape(), "forearm_shell"), material="cast_orange", name="forearm_shell")
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, 0.052, 0.050)),
        mass=1.5,
        origin=Origin(xyz=(FOREARM_LENGTH * 0.50, 0.0, 0.0)),
    )

    wrist_cartridge = model.part("wrist_cartridge")
    wrist_cartridge.visual(
        _mesh(_wrist_body_shape(), "wrist_cartridge_body"),
        material="machine_gray",
        name="wrist_body",
    )
    wrist_cartridge.visual(
        _mesh(_tool_flange_shape(), "wrist_tool_flange"),
        material="machined_steel",
        name="tool_flange",
    )
    wrist_cartridge.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=WRIST_BODY_LENGTH),
        mass=0.8,
        origin=Origin(xyz=(WRIST_BODY_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=bench_base,
        child=shoulder_pedestal,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=140.0, velocity=1.4),
    )
    model.articulation(
        "pedestal_to_upper_arm",
        ArticulationType.FIXED,
        parent=shoulder_pedestal,
        child=upper_arm,
        origin=Origin(xyz=(PEDESTAL_MOUNT_X, 0.0, PEDESTAL_MOUNT_Z)),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.90, effort=95.0, velocity=1.8),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_cartridge,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.80, upper=2.80, effort=35.0, velocity=3.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bench_base = object_model.get_part("bench_base")
    shoulder_pedestal = object_model.get_part("shoulder_pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_cartridge = object_model.get_part("wrist_cartridge")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    tool_flange = wrist_cartridge.get_visual("tool_flange")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        shoulder_pedestal,
        upper_arm,
        reason="upper arm shoulder mount is intentionally nested into the yaw-head casting as a simplified enclosed bearing stack",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        reason="elbow joint is represented as an enclosed nested bearing cartridge rather than fully hollowed mating shells",
    )
    ctx.allow_overlap(
        forearm,
        wrist_cartridge,
        reason="wrist roll cartridge is represented as a compact nested bearing module at the forearm nose",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.005)
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

    ctx.check(
        "shoulder_axis_vertical",
        tuple(shoulder_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {shoulder_yaw.axis}",
    )
    ctx.check(
        "elbow_axis_horizontal",
        tuple(elbow_pitch.axis) == (0.0, 1.0, 0.0),
        details=f"expected horizontal elbow axis, got {elbow_pitch.axis}",
    )
    ctx.check(
        "wrist_axis_forearm_aligned",
        tuple(wrist_roll.axis) == (1.0, 0.0, 0.0),
        details=f"expected wrist roll on forearm axis, got {wrist_roll.axis}",
    )

    ctx.expect_contact(bench_base, shoulder_pedestal, name="pedestal_bears_on_base")
    ctx.expect_overlap(bench_base, shoulder_pedestal, axes="xy", min_overlap=0.090, name="pedestal_seated_over_base")
    ctx.expect_contact(
        shoulder_pedestal,
        upper_arm,
        contact_tol=0.002,
        name="upper_arm_mounted_to_pedestal",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        contact_tol=0.0025,
        name="forearm_supported_by_elbow_housing",
    )
    ctx.expect_contact(
        forearm,
        wrist_cartridge,
        contact_tol=0.005,
        name="wrist_cartridge_supported_by_forearm",
    )
    ctx.expect_gap(
        wrist_cartridge,
        forearm,
        axis="x",
        min_gap=0.050,
        positive_elem=tool_flange,
        name="tool_flange_projects_ahead_of_forearm",
    )

    with ctx.pose(shoulder_yaw=1.40):
        ctx.expect_contact(bench_base, shoulder_pedestal, name="yaw_bearing_stays_seated")

    with ctx.pose(elbow_pitch=-0.30):
        ctx.fail_if_parts_overlap_in_current_pose(name="downreach_pose_clear")
        ctx.expect_gap(
            wrist_cartridge,
            bench_base,
            axis="z",
            min_gap=0.004,
            name="wrist_clears_bench_in_downreach",
        )

    with ctx.pose(shoulder_yaw=0.80, elbow_pitch=0.75, wrist_roll=2.00):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
