from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.220
BASE_WIDTH = 0.210
BASE_THICKNESS = 0.006
BASE_PIVOT_X = -0.070
BASE_PIVOT_Z = 0.022

LOWER_LENGTH = 0.145
LOWER_ANGLE = 1.02
UPPER_LENGTH = 0.128
UPPER_ANGLE = 2.20
TRAY_TILT = -0.17

ARM_RAIL_CENTER_Y = 0.068
ARM_RAIL_THICKNESS = 0.008
ARM_RAIL_HEIGHT = 0.016
ARM_BOSS_RADIUS = 0.012
ARM_CROSS_RADIUS = 0.006
ARM_BRIDGE_WIDTH = 0.136

TRAY_WIDTH = 0.230
TRAY_PANEL_START = 0.018
TRAY_PANEL_LENGTH = 0.250
TRAY_PANEL_THICKNESS = 0.004


def endpoint(length: float, angle: float) -> tuple[float, float, float]:
    return (length * math.cos(angle), 0.0, length * math.sin(angle))


def linkage_frame(
    *,
    length: float,
    cross_positions: tuple[float, ...],
    bridge_position: float | None = None,
    bridge_length: float = 0.016,
) -> cq.Workplane:
    rail_span = ARM_RAIL_CENTER_Y * 2.0 - ARM_RAIL_THICKNESS + 0.004
    rail_core_length = length - 2.0 * ARM_BOSS_RADIUS

    frame = None
    for side in (-ARM_RAIL_CENTER_Y, ARM_RAIL_CENTER_Y):
        rail = (
            cq.Workplane("XZ")
            .center(length * 0.5, 0.0)
            .rect(rail_core_length, ARM_RAIL_HEIGHT)
            .extrude(ARM_RAIL_THICKNESS * 0.5, both=True)
            .translate((0.0, side, 0.0))
        )
        start_boss = (
            cq.Workplane("XZ")
            .center(0.0, 0.0)
            .circle(ARM_BOSS_RADIUS)
            .extrude(ARM_RAIL_THICKNESS * 0.5, both=True)
            .translate((0.0, side, 0.0))
        )
        end_boss = (
            cq.Workplane("XZ")
            .center(length, 0.0)
            .circle(ARM_BOSS_RADIUS)
            .extrude(ARM_RAIL_THICKNESS * 0.5, both=True)
            .translate((0.0, side, 0.0))
        )

        if frame is None:
            frame = rail.union(start_boss).union(end_boss)
        else:
            frame = frame.union(rail).union(start_boss).union(end_boss)

    for x_pos in cross_positions:
        frame = frame.union(
            cq.Workplane("XZ")
            .center(x_pos, 0.0)
            .circle(ARM_CROSS_RADIUS)
            .extrude(rail_span * 0.5, both=True)
        )

    if bridge_position is not None:
        frame = frame.union(
            cq.Workplane("XY")
            .box(bridge_length, ARM_BRIDGE_WIDTH, ARM_RAIL_HEIGHT)
            .translate((bridge_position, 0.0, 0.0))
        )

    return frame


def tray_shape() -> cq.Workplane:
    panel_center_x = TRAY_PANEL_START + TRAY_PANEL_LENGTH * 0.5
    panel_center_z = 0.007

    tray = (
        cq.Workplane("XY")
        .box(TRAY_PANEL_LENGTH, TRAY_WIDTH, TRAY_PANEL_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
        .translate((panel_center_x, 0.0, panel_center_z))
    )

    tray = tray.union(
        cq.Workplane("XY")
        .box(0.030, 0.094, 0.010)
        .translate((0.015, 0.0, 0.005))
    )
    tray = tray.union(
        cq.Workplane("XZ").circle(0.009).extrude(0.043, both=True)
    )

    slot_centers: list[tuple[float, float]] = []
    for x_pos in (0.102, 0.164, 0.226):
        for y_pos in (-0.058, -0.019, 0.019, 0.058):
            slot_centers.append((x_pos, y_pos))

    slot_cutters = None
    for x_pos, y_pos in slot_centers:
        cutter = (
            cq.Workplane("XY")
            .box(0.040, 0.012, 0.018)
            .translate((x_pos, y_pos, panel_center_z))
        )
        slot_cutters = cutter if slot_cutters is None else slot_cutters.union(cutter)

    if slot_cutters is not None:
        tray = tray.cut(slot_cutters)

    side_rail_x = TRAY_PANEL_START + 0.125
    for side in (-0.109, 0.109):
        tray = tray.union(
            cq.Workplane("XY")
            .box(0.210, 0.008, 0.010)
            .edges("|Z")
            .fillet(0.0025)
            .translate((side_rail_x, side, 0.012))
        )

    front_lip_x = TRAY_PANEL_START + TRAY_PANEL_LENGTH - 0.010
    for lip_y in (-0.062, 0.062):
        tray = tray.union(
            cq.Workplane("XY")
            .box(0.020, 0.050, 0.014)
            .edges("|Z")
            .fillet(0.004)
            .faces(">X")
            .edges("|Y")
            .chamfer(0.003)
            .translate((front_lip_x, lip_y, 0.016))
        )

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_laptop_stand")

    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.37, 0.40, 1.0))
    satin = model.material("satin", rgba=(0.70, 0.72, 0.74, 1.0))

    base = model.part("base")
    lower_arm = model.part("lower_arm")
    upper_arm = model.part("upper_arm")
    tray = model.part("tray")

    base_shape = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.016)
        .translate((0.0, 0.0, BASE_THICKNESS * 0.5))
        .union(
            cq.Workplane("XY")
            .box(0.024, 0.012, 0.028)
            .edges("|Z")
            .fillet(0.003)
            .translate((BASE_PIVOT_X, -0.078, BASE_THICKNESS + 0.014))
        )
        .union(
            cq.Workplane("XY")
            .box(0.024, 0.012, 0.028)
            .edges("|Z")
            .fillet(0.003)
            .translate((BASE_PIVOT_X, 0.078, BASE_THICKNESS + 0.014))
        )
        .union(
            cq.Workplane("XY")
            .box(0.014, 0.146, 0.012)
            .edges("|Z")
            .fillet(0.003)
            .translate((BASE_PIVOT_X - 0.010, 0.0, BASE_THICKNESS + 0.006))
        )
    )
    base.visual(
        mesh_from_cadquery(base_shape, "base_plate"),
        material=graphite,
        name="base_plate",
    )

    lower_shape = linkage_frame(
        length=LOWER_LENGTH,
        cross_positions=(0.056, 0.098),
    )
    lower_arm.visual(
        mesh_from_cadquery(lower_shape, "lower_arm"),
        origin=Origin(rpy=(0.0, -LOWER_ANGLE, 0.0)),
        material=dark_metal,
        name="lower_frame",
    )

    upper_shape = linkage_frame(
        length=UPPER_LENGTH,
        cross_positions=(0.044, 0.086),
        bridge_position=UPPER_LENGTH - 0.016,
        bridge_length=0.018,
    )
    upper_arm.visual(
        mesh_from_cadquery(upper_shape, "upper_arm"),
        origin=Origin(rpy=(0.0, -UPPER_ANGLE, 0.0)),
        material=dark_metal,
        name="upper_frame",
    )

    tray.visual(
        mesh_from_cadquery(tray_shape(), "tray"),
        origin=Origin(rpy=(0.0, TRAY_TILT, 0.0)),
        material=satin,
        name="tray_panel",
    )

    lower_limits = MotionLimits(
        effort=12.0,
        velocity=2.0,
        lower=-0.45,
        upper=0.30,
    )
    upper_limits = MotionLimits(
        effort=12.0,
        velocity=2.0,
        lower=-0.35,
        upper=0.42,
    )
    tray_limits = MotionLimits(
        effort=8.0,
        velocity=2.5,
        lower=-0.22,
        upper=0.24,
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(BASE_PIVOT_X, 0.0, BASE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=lower_limits,
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=endpoint(LOWER_LENGTH, LOWER_ANGLE)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=upper_limits,
    )
    model.articulation(
        "upper_arm_to_tray",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tray,
        origin=Origin(xyz=endpoint(UPPER_LENGTH, UPPER_ANGLE)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=tray_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    tray = object_model.get_part("tray")

    base_to_lower = object_model.get_articulation("base_to_lower_arm")
    lower_to_upper = object_model.get_articulation("lower_to_upper_arm")
    upper_to_tray = object_model.get_articulation("upper_arm_to_tray")

    ctx.allow_overlap(
        base,
        lower_arm,
        reason="The lower linkage pivot is simplified as a nested clevis-and-boss hinge around the shared base axle.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        reason="The Z-link elbow uses simplified solid pivot bosses that stand in for the interleaved hinge barrels.",
    )
    ctx.allow_overlap(
        upper_arm,
        tray,
        reason="The tray tilt hinge is represented by a simplified hinge block nested into the upper yoke around the shared pivot line.",
    )

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.155,
        name="tray sits well above the desk base",
    )

    ctx.expect_gap(
        upper_arm,
        base,
        axis="z",
        min_gap=0.060,
        name="upper linkage clears the base plate",
    )

    rest_upper = ctx.part_world_position(upper_arm)
    deployed_upper = None
    if base_to_lower.motion_limits is not None and base_to_lower.motion_limits.upper is not None:
        with ctx.pose({base_to_lower: base_to_lower.motion_limits.upper}):
            deployed_upper = ctx.part_world_position(upper_arm)

    ctx.check(
        "lower arm lifts the linkage upward",
        rest_upper is not None
        and deployed_upper is not None
        and deployed_upper[2] > rest_upper[2] + 0.010,
        details=f"rest={rest_upper}, deployed={deployed_upper}",
    )

    rest_tray_origin = ctx.part_world_position(tray)
    raised_tray_origin = None
    if lower_to_upper.motion_limits is not None and lower_to_upper.motion_limits.upper is not None:
        with ctx.pose({lower_to_upper: lower_to_upper.motion_limits.upper}):
            raised_tray_origin = ctx.part_world_position(tray)

    ctx.check(
        "upper arm rotates the tray hinge upward",
        rest_tray_origin is not None
        and raised_tray_origin is not None
        and raised_tray_origin[2] > rest_tray_origin[2] + 0.020,
        details=f"rest={rest_tray_origin}, raised={raised_tray_origin}",
    )

    rest_tray_aabb = ctx.part_world_aabb(tray)
    tilted_tray_aabb = None
    if upper_to_tray.motion_limits is not None and upper_to_tray.motion_limits.upper is not None:
        with ctx.pose({upper_to_tray: upper_to_tray.motion_limits.upper}):
            tilted_tray_aabb = ctx.part_world_aabb(tray)

    ctx.check(
        "tray hinge lifts the tray front edge",
        rest_tray_aabb is not None
        and tilted_tray_aabb is not None
        and tilted_tray_aabb[1][2] > rest_tray_aabb[1][2] + 0.040,
        details=f"rest={rest_tray_aabb}, tilted={tilted_tray_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
