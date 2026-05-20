from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(
    start: tuple[float, float],
    end: tuple[float, float],
    radius: float,
    *,
    arc_segments: int = 20,
) -> list[tuple[float, float]]:
    sx, sy = start
    ex, ey = end
    angle = math.atan2(ey - sy, ex - sx)

    points: list[tuple[float, float]] = []
    for i in range(arc_segments + 1):
        a = angle - math.pi / 2.0 + math.pi * i / arc_segments
        points.append((ex + radius * math.cos(a), ey + radius * math.sin(a)))
    for i in range(arc_segments + 1):
        a = angle + math.pi / 2.0 + math.pi * i / arc_segments
        points.append((sx + radius * math.cos(a), sy + radius * math.sin(a)))
    return points


def _machined_link_mesh(
    name: str,
    end_x: float,
    end_z: float,
    *,
    thickness: float,
    outer_radius: float,
    slot_radius: float,
):
    length = math.hypot(end_x, end_z)
    ux = end_x / length
    uz = end_z / length
    slot_start = (ux * outer_radius * 1.45, uz * outer_radius * 1.45)
    slot_end = (end_x - ux * outer_radius * 1.45, end_z - uz * outer_radius * 1.45)
    geometry = ExtrudeWithHolesGeometry(
        _capsule_profile((0.0, 0.0), (end_x, end_z), outer_radius),
        [
            _capsule_profile(slot_start, slot_end, slot_radius, arc_segments=14),
            _circle_profile(slot_radius * 0.72, center=(end_x * 0.50, end_z * 0.50), segments=28),
        ],
        thickness,
        center=True,
    )
    return mesh_from_geometry(geometry, name)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="staggered_rotary_assembly")

    model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("dark_anodized", rgba=(0.08, 0.09, 0.11, 1.0))
    model.material("motor_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("bearing_steel", rgba=(0.46, 0.47, 0.48, 1.0))
    model.material("accent_blue", rgba=(0.10, 0.28, 0.58, 1.0))
    model.material("fastener_dark", rgba=(0.02, 0.02, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.32, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="dark_anodized",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material="brushed_aluminum",
        name="fixed_pedestal",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.135),
        origin=Origin(xyz=(0.145, 0.0, 0.092), rpy=_axis_rpy("x")),
        material="motor_black",
        name="base_motor",
    )
    base.visual(
        Box((0.050, 0.105, 0.070)),
        origin=Origin(xyz=(0.085, 0.0, 0.090)),
        material="motor_black",
        name="base_gearbox",
    )
    for i, (x, y) in enumerate(
        ((-0.165, -0.105), (-0.165, 0.105), (0.165, -0.105), (0.165, 0.105))
    ):
        base.visual(
            Cylinder(radius=0.015, length=0.007),
            origin=Origin(xyz=(x, y, 0.043)),
            material="fastener_dark",
            name=f"base_bolt_{i}",
        )

    yaw_carrier = model.part("yaw_carrier")
    yaw_carrier.visual(
        Cylinder(radius=0.086, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="bearing_steel",
        name="yaw_platter",
    )
    yaw_carrier.visual(
        Box((0.175, 0.135, 0.030)),
        origin=Origin(xyz=(0.092, 0.0, 0.049)),
        material="brushed_aluminum",
        name="yoke_foot",
    )
    yaw_carrier.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.145, 0.132, 0.155),
                span_width=0.072,
                trunnion_diameter=0.026,
                trunnion_center_z=0.105,
                base_thickness=0.030,
                corner_radius=0.006,
                center=False,
            ),
            "shoulder_yoke",
        ),
        origin=Origin(xyz=(0.160, 0.0, 0.035), rpy=(0.0, 0.0, math.pi / 2.0)),
        material="brushed_aluminum",
        name="shoulder_yoke",
    )
    yaw_carrier.visual(
        Box((0.070, 0.090, 0.075)),
        origin=Origin(xyz=(0.045, 0.0, 0.103)),
        material="motor_black",
        name="shoulder_actuator",
    )
    yaw_carrier.visual(
        Cylinder(radius=0.030, length=0.058),
        origin=Origin(xyz=(0.160, -0.095, 0.140), rpy=_axis_rpy("y")),
        material="accent_blue",
        name="shoulder_output_collar",
    )
    yaw_carrier.visual(
        Box((0.028, 0.090, 0.055)),
        origin=Origin(xyz=(0.087, 0.0, 0.103)),
        material="motor_black",
        name="shoulder_drive_bridge",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        _machined_link_mesh(
            "upper_machined_link",
            0.231,
            0.020,
            thickness=0.046,
            outer_radius=0.032,
            slot_radius=0.010,
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_aluminum",
        name="upper_machined_link",
    )
    shoulder_link.visual(
        Cylinder(radius=0.045, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_rpy("y")),
        material="bearing_steel",
        name="shoulder_hub",
    )
    shoulder_link.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.095, 0.076, 0.118),
                span_width=0.072,
                trunnion_diameter=0.022,
                trunnion_center_z=0.064,
                base_thickness=0.024,
                corner_radius=0.005,
                center=False,
            ),
            "elbow_yoke",
        ),
        origin=Origin(xyz=(0.300, 0.0, 0.000), rpy=(0.0, 0.0, math.pi / 2.0)),
        material="brushed_aluminum",
        name="elbow_yoke",
    )
    shoulder_link.visual(
        Box((0.092, 0.046, 0.070)),
        origin=Origin(xyz=(0.210, -0.045, 0.058)),
        material="motor_black",
        name="elbow_actuator",
    )
    shoulder_link.visual(
        Cylinder(radius=0.032, length=0.052),
        origin=Origin(xyz=(0.300, -0.0625, 0.064), rpy=_axis_rpy("y")),
        material="accent_blue",
        name="elbow_drive_collar",
    )
    shoulder_link.visual(
        Box((0.018, 0.046, 0.046)),
        origin=Origin(xyz=(0.247, -0.045, 0.064)),
        material="motor_black",
        name="elbow_drive_bridge",
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        _machined_link_mesh(
            "forearm_machined_link",
            0.240,
            -0.040,
            thickness=0.042,
            outer_radius=0.036,
            slot_radius=0.010,
        ),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_aluminum",
        name="forearm_machined_link",
    )
    forearm_link.visual(
        Cylinder(radius=0.039, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_rpy("y")),
        material="bearing_steel",
        name="elbow_hub",
    )
    forearm_link.visual(
        Cylinder(radius=0.056, length=0.020),
        origin=Origin(xyz=(0.240, 0.0, 0.018)),
        material="bearing_steel",
        name="wrist_mount_face",
    )
    forearm_link.visual(
        Box((0.100, 0.044, 0.064)),
        origin=Origin(xyz=(0.155, 0.042, -0.012)),
        material="motor_black",
        name="wrist_actuator",
    )
    forearm_link.visual(
        Cylinder(radius=0.026, length=0.046),
        origin=Origin(xyz=(0.210, 0.024, -0.012), rpy=_axis_rpy("x")),
        material="accent_blue",
        name="wrist_drive_collar",
    )

    wrist_rotor = model.part("wrist_rotor")
    wrist_rotor.visual(
        Cylinder(radius=0.050, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="bearing_steel",
        name="wrist_flange",
    )
    wrist_rotor.visual(
        Box((0.105, 0.050, 0.030)),
        origin=Origin(xyz=(0.060, 0.0, 0.038)),
        material="dark_anodized",
        name="output_saddle",
    )
    wrist_rotor.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.025, 0.030, 0.026)),
        material="fastener_dark",
        name="flange_bolt_0",
    )
    wrist_rotor.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.025, -0.030, 0.026)),
        material="fastener_dark",
        name="flange_bolt_1",
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "yaw_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=yaw_carrier,
        child=shoulder_link,
        origin=Origin(xyz=(0.160, 0.0, 0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=-1.15, upper=1.20),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forearm_link,
        origin=Origin(xyz=(0.300, 0.0, 0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=34.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=wrist_rotor,
        origin=Origin(xyz=(0.240, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-2.7, upper=2.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw = object_model.get_part("yaw_carrier")
    shoulder = object_model.get_part("shoulder_link")
    forearm = object_model.get_part("forearm_link")
    wrist = object_model.get_part("wrist_rotor")

    yaw_joint = object_model.get_articulation("base_to_yaw")
    shoulder_joint = object_model.get_articulation("yaw_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    revolute_count = sum(
        1 for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check(
        "four rotary stages are articulated",
        revolute_count == 4,
        details=f"revolute_count={revolute_count}",
    )

    ctx.expect_gap(
        yaw,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="yaw_platter",
        negative_elem="fixed_pedestal",
        name="yaw platter seats on fixed pedestal",
    )
    ctx.expect_overlap(
        yaw,
        base,
        axes="xy",
        min_overlap=0.070,
        elem_a="yaw_platter",
        elem_b="fixed_pedestal",
        name="yaw platter overlaps pedestal footprint",
    )
    ctx.allow_overlap(
        shoulder,
        yaw,
        elem_a="shoulder_hub",
        elem_b="shoulder_yoke",
        reason=(
            "The shoulder hub is modeled as a captured trunnion seated through the yoke cheeks; "
            "the tiny bearing interference keeps the rotary stage mechanically supported."
        ),
    )
    ctx.expect_within(
        shoulder,
        yaw,
        axes="y",
        inner_elem="shoulder_hub",
        outer_elem="shoulder_yoke",
        margin=0.004,
        name="shoulder hub captured by yoke span",
    )
    ctx.expect_overlap(
        shoulder,
        yaw,
        axes="xz",
        elem_a="shoulder_hub",
        elem_b="shoulder_yoke",
        min_overlap=0.045,
        name="shoulder hub remains carried in yoke bore",
    )
    ctx.allow_overlap(
        forearm,
        shoulder,
        elem_a="elbow_hub",
        elem_b="elbow_yoke",
        reason=(
            "The elbow hub is a captured rotary trunnion inside the forked yoke; "
            "a small proxy interference represents the pressed bearing fit."
        ),
    )
    ctx.expect_within(
        forearm,
        shoulder,
        axes="y",
        inner_elem="elbow_hub",
        outer_elem="elbow_yoke",
        margin=0.004,
        name="elbow hub captured by fork span",
    )
    ctx.expect_overlap(
        forearm,
        shoulder,
        axes="xz",
        elem_a="elbow_hub",
        elem_b="elbow_yoke",
        min_overlap=0.040,
        name="elbow hub remains carried in yoke bore",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="wrist_flange",
        negative_elem="wrist_mount_face",
        name="wrist flange seats on machined face",
    )

    rest_tool = ctx.part_world_position(wrist)
    with ctx.pose({shoulder_joint: 0.55, elbow_joint: -0.65, wrist_joint: 1.0}):
        moved_tool = ctx.part_world_position(wrist)
    ctx.check(
        "offset rotary chain moves distal wrist",
        rest_tool is not None
        and moved_tool is not None
        and math.dist(rest_tool, moved_tool) > 0.080
        and abs(moved_tool[2] - rest_tool[2]) > 0.040,
        details=f"rest={rest_tool}, moved={moved_tool}",
    )

    ctx.check(
        "stage axes are deliberately staggered",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and shoulder_joint.axis == (0.0, 1.0, 0.0)
        and elbow_joint.axis == (0.0, 1.0, 0.0)
        and wrist_joint.axis == (0.0, 0.0, 1.0),
        details=(
            f"axes={yaw_joint.axis}, {shoulder_joint.axis}, "
            f"{elbow_joint.axis}, {wrist_joint.axis}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
