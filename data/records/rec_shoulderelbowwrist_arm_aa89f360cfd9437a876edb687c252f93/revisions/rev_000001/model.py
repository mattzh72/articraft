from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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


BASE_RADIUS = 0.190
BASE_THICKNESS = 0.040
PEDESTAL_RADIUS = 0.112
PEDESTAL_HEIGHT = 0.255
TOP_COLLAR_RADIUS = 0.122
TOP_COLLAR_HEIGHT = 0.035
SHOULDER_Z = BASE_THICKNESS + PEDESTAL_HEIGHT + TOP_COLLAR_HEIGHT

UPPER_ARM_LENGTH = 0.340
UPPER_ARM_PIVOT_Z = 0.065
FOREARM_LENGTH = 0.300
WRIST_LENGTH = 0.085
def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _cylinder_x(radius: float, length: float, *, start_x: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((start_x, 0.0, 0.0))
    )


def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _cylinder_z(radius: float, length: float, *, start_z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, start_z))


def _pedestal_shape() -> cq.Workplane:
    base_plate = _cylinder_z(BASE_RADIUS, BASE_THICKNESS)
    base_plate = base_plate.edges(">Z").fillet(0.008)

    pedestal = _cylinder_z(PEDESTAL_RADIUS, PEDESTAL_HEIGHT, start_z=BASE_THICKNESS)
    collar = _cylinder_z(TOP_COLLAR_RADIUS, TOP_COLLAR_HEIGHT, start_z=BASE_THICKNESS + PEDESTAL_HEIGHT)

    service_band = _cylinder_z(0.090, 0.020, start_z=0.130)
    return base_plate.union(pedestal).union(collar).union(service_band)


def _upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _cylinder_z(0.076, 0.130)
    shoulder_fairing = _box(0.140, 0.114, 0.102, center=(0.090, 0.0, UPPER_ARM_PIVOT_Z)).edges(
        "|X"
    ).fillet(0.018)
    main_beam = _box(0.210, 0.084, 0.068, center=(0.185, 0.0, UPPER_ARM_PIVOT_Z)).edges("|X").fillet(
        0.014
    )
    elbow_mount = _box(0.050, 0.054, 0.054, center=(0.315, 0.0, UPPER_ARM_PIVOT_Z))
    elbow_cheek_left = _box(0.060, 0.014, 0.076, center=(0.300, 0.032, UPPER_ARM_PIVOT_Z))
    elbow_cheek_right = _box(0.060, 0.014, 0.076, center=(0.300, -0.032, UPPER_ARM_PIVOT_Z))

    return (
        shoulder_hub.union(shoulder_fairing)
        .union(main_beam)
        .union(elbow_mount)
        .union(elbow_cheek_left)
        .union(elbow_cheek_right)
    )


def _forearm_shape() -> cq.Workplane:
    root_block = _box(0.060, 0.054, 0.058, center=(0.030, 0.0, 0.0)).edges("|X").fillet(0.010)
    main_beam = _box(0.190, 0.062, 0.054, center=(0.155, 0.0, 0.0)).edges("|X").fillet(0.010)
    wrist_mount = _box(0.045, 0.050, 0.050, center=(0.2775, 0.0, 0.0))
    wrist_clevis_left = _box(0.052, 0.010, 0.050, center=(0.274, 0.026, 0.0))
    wrist_clevis_right = _box(0.052, 0.010, 0.050, center=(0.274, -0.026, 0.0))

    return (
        root_block.union(main_beam)
        .union(wrist_mount)
        .union(wrist_clevis_left)
        .union(wrist_clevis_right)
    )


def _wrist_shape() -> cq.Workplane:
    wrist_root = _box(0.040, 0.044, 0.042, center=(0.020, 0.0, 0.0)).edges("|X").fillet(0.007)
    wrist_body = _box(0.040, 0.050, 0.046, center=(0.050, 0.0, 0.0)).edges("|X").fillet(0.009)
    tool_nose = _cylinder_x(0.021, 0.018, start_x=0.067)
    return wrist_root.union(wrist_body).union(tool_nose)


def _tool_flange_shape() -> cq.Workplane:
    flange_plate = _cylinder_x(0.040, 0.016, start_x=0.0)
    pilot = _cylinder_x(0.017, 0.020, start_x=0.016)
    flange = flange_plate.union(pilot)

    for y_pos, z_pos in ((0.024, 0.0), (-0.024, 0.0), (0.0, 0.024), (0.0, -0.024)):
        flange = flange.cut(
            _cylinder_x(0.0035, 0.020, start_x=-0.002).translate((0.0, y_pos, z_pos))
        )

    return flange


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_shoulder_wristed_arm")

    model.material("pedestal_gray", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("arm_orange", rgba=(0.86, 0.39, 0.11, 1.0))
    model.material("wrist_graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("tool_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    pedestal = model.part("pedestal")
    _add_mesh_visual(pedestal, _pedestal_shape(), "pedestal_shell", "pedestal_gray", "pedestal_shell")
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=SHOULDER_Z),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    _add_mesh_visual(upper_arm, _upper_arm_shape(), "upper_arm_shell", "arm_orange", "upper_arm_shell")
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.360, 0.120, 0.140)),
        mass=18.0,
        origin=Origin(xyz=(0.180, 0.0, 0.070)),
    )

    forearm = model.part("forearm")
    _add_mesh_visual(forearm, _forearm_shape(), "forearm_shell", "arm_orange", "forearm_shell")
    forearm.inertial = Inertial.from_geometry(
        Box((0.310, 0.090, 0.090)),
        mass=11.0,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    wrist_link = model.part("wrist_link")
    _add_mesh_visual(wrist_link, _wrist_shape(), "wrist_shell", "wrist_graphite", "wrist_shell")
    wrist_link.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.060)),
        mass=3.2,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )

    tool_flange = model.part("tool_flange")
    _add_mesh_visual(tool_flange, _tool_flange_shape(), "tool_flange_shell", "tool_steel", "tool_flange_shell")
    tool_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.036),
        mass=1.4,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, radians(90.0), 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.2,
            upper=2.2,
            effort=120.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, UPPER_ARM_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.20,
            upper=2.05,
            effort=90.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_link,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.30,
            upper=1.45,
            effort=35.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "wrist_to_tool_flange",
        ArticulationType.FIXED,
        parent=wrist_link,
        child=tool_flange,
        origin=Origin(xyz=(WRIST_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_link = object_model.get_part("wrist_link")
    tool_flange = object_model.get_part("tool_flange")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="upper_arm_shell",
        negative_elem="pedestal_shell",
        name="upper arm seats on pedestal collar",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        contact_tol=0.0005,
        elem_a="upper_arm_shell",
        elem_b="forearm_shell",
        name="forearm is carried in the elbow clevis",
    )
    ctx.expect_contact(
        forearm,
        wrist_link,
        contact_tol=0.0005,
        elem_a="forearm_shell",
        elem_b="wrist_shell",
        name="wrist link is carried at the forearm tip",
    )
    ctx.expect_contact(
        wrist_link,
        tool_flange,
        contact_tol=0.0005,
        elem_a="wrist_shell",
        elem_b="tool_flange_shell",
        name="tool flange mounts to the wrist nose",
    )
    ctx.expect_gap(
        tool_flange,
        pedestal,
        axis="z",
        min_gap=0.020,
        positive_elem="tool_flange_shell",
        negative_elem="pedestal_shell",
        name="rest pose keeps the tool flange above the pedestal",
    )

    rest_pos = ctx.part_world_position(tool_flange)

    with ctx.pose({shoulder_yaw: 0.80}):
        yawed_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "positive shoulder yaw sweeps the arm toward +Y",
        rest_pos is not None
        and yawed_pos is not None
        and yawed_pos[1] > rest_pos[1] + 0.18
        and yawed_pos[0] < rest_pos[0] - 0.10,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    with ctx.pose({elbow_pitch: 0.95}):
        raised_elbow_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "positive elbow pitch raises the distal arm",
        rest_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_pos[2] + 0.22,
        details=f"rest={rest_pos}, elbow={raised_elbow_pos}",
    )

    with ctx.pose({wrist_pitch: 0.85}):
        raised_wrist_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "positive wrist pitch lifts the tool flange",
        rest_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_pos[2] + 0.04,
        details=f"rest={rest_pos}, wrist={raised_wrist_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
