from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_RADIUS = 0.006
SHAFT_LENGTH = 0.11

BASE_LENGTH = 0.30
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.012

SIDE_WALL_THICKNESS = 0.014
FRAME_TOP_Z = 0.10
SIDE_WALL_HEIGHT = FRAME_TOP_Z - BASE_THICKNESS

BRIDGE_THICKNESS = 0.012
BRIDGE_BOTTOM_Z = FRAME_TOP_Z - BRIDGE_THICKNESS
BRIDGE_WIDTH_X = 0.036

LOWER_BOSS_HEIGHT = 0.010
LOWER_BOSS_RADIUS = 0.017
LOWER_BOSS_TOP_Z = BASE_THICKNESS + LOWER_BOSS_HEIGHT

UPPER_BOSS_HEIGHT = 0.010
UPPER_BOSS_RADIUS = 0.016
UPPER_BOSS_BOTTOM_Z = BRIDGE_BOTTOM_Z - UPPER_BOSS_HEIGHT

GEAR_THICKNESS = 0.012
GEAR_BOTTOM_Z = 0.041

LOWER_COLLAR_THICKNESS = 0.004
LOWER_COLLAR_RADIUS = 0.012
LOWER_COLLAR_CENTER_Z = LOWER_BOSS_TOP_Z + (LOWER_COLLAR_THICKNESS * 0.5)

UPPER_COLLAR_THICKNESS = 0.004
UPPER_COLLAR_RADIUS = 0.0115
UPPER_COLLAR_CENTER_Z = UPPER_BOSS_BOTTOM_Z - (UPPER_COLLAR_THICKNESS * 0.5)

GEAR_GAP = 0.0012

INPUT_GEAR_RADIUS = 0.036
IDLER_GEAR_RADIUS = 0.028
OUTPUT_GEAR_RADIUS = 0.046

INPUT_TEETH = 18
IDLER_TEETH = 14
OUTPUT_TEETH = 24

INPUT_X = -(INPUT_GEAR_RADIUS + IDLER_GEAR_RADIUS + GEAR_GAP)
IDLER_X = 0.0
OUTPUT_X = IDLER_GEAR_RADIUS + OUTPUT_GEAR_RADIUS + GEAR_GAP


def polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * cos(angle), radius * sin(angle))


def make_gear_body(
    *,
    tip_radius: float,
    teeth: int,
    thickness: float,
    shaft_radius: float,
    hub_radius: float,
    hub_thickness: float,
    window_count: int = 4,
) -> cq.Workplane:
    tooth_depth = min(tip_radius * 0.18, 0.0048)
    root_radius = tip_radius - tooth_depth
    pitch = (2.0 * pi) / teeth

    profile_points: list[tuple[float, float]] = []
    for tooth_index in range(teeth):
        theta = tooth_index * pitch
        a0 = theta - (0.50 * pitch)
        a1 = theta - (0.18 * pitch)
        a2 = theta + (0.18 * pitch)
        a3 = theta + (0.50 * pitch)
        if tooth_index == 0:
            profile_points.append(polar_xy(root_radius, a0))
        profile_points.extend(
            [
                polar_xy(tip_radius, a1),
                polar_xy(tip_radius, a2),
                polar_xy(root_radius, a3),
            ]
        )

    gear = cq.Workplane("XY").polyline(profile_points).close().extrude(thickness)

    hub_z0 = 0.5 * (thickness - hub_thickness)
    hub = (
        cq.Workplane("XY")
        .circle(hub_radius)
        .extrude(hub_thickness)
        .translate((0.0, 0.0, hub_z0))
    )
    body = gear.union(hub)

    if window_count > 0 and root_radius - hub_radius > 0.016:
        window_radius = min((root_radius - hub_radius) * 0.24, 0.009)
        window_ring = 0.5 * (root_radius + hub_radius)
        window_points = [
            polar_xy(window_ring, (2.0 * pi * i / window_count) + (pi / window_count))
            for i in range(window_count)
        ]
        windows = (
            cq.Workplane("XY")
            .pushPoints(window_points)
            .circle(window_radius)
            .extrude(hub_thickness + 0.04)
            .translate((0.0, 0.0, hub_z0 - 0.02))
        )
        body = body.cut(windows)

    bore = (
        cq.Workplane("XY")
        .circle(shaft_radius)
        .extrude(hub_thickness + 0.05)
        .translate((0.0, 0.0, hub_z0 - 0.025))
    )
    return body.cut(bore)


def make_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    wall_y = 0.5 * (BASE_WIDTH - SIDE_WALL_THICKNESS)
    for y_sign in (-1.0, 1.0):
        side_wall = (
            cq.Workplane("XY")
            .box(
                BASE_LENGTH,
                SIDE_WALL_THICKNESS,
                SIDE_WALL_HEIGHT,
                centered=(True, True, False),
            )
            .translate((0.0, y_sign * wall_y, BASE_THICKNESS))
        )
        frame = frame.union(side_wall)

    for shaft_x in (INPUT_X, IDLER_X, OUTPUT_X):
        bridge = (
            cq.Workplane("XY")
            .box(
                BRIDGE_WIDTH_X,
                BASE_WIDTH - (2.0 * SIDE_WALL_THICKNESS),
                BRIDGE_THICKNESS,
                centered=(True, True, False),
            )
            .translate((shaft_x, 0.0, BRIDGE_BOTTOM_Z))
        )
        lower_boss = (
            cq.Workplane("XY")
            .circle(LOWER_BOSS_RADIUS)
            .extrude(LOWER_BOSS_HEIGHT)
            .translate((shaft_x, 0.0, BASE_THICKNESS))
        )
        upper_boss = (
            cq.Workplane("XY")
            .circle(UPPER_BOSS_RADIUS)
            .extrude(UPPER_BOSS_HEIGHT)
            .translate((shaft_x, 0.0, UPPER_BOSS_BOTTOM_Z))
        )
        frame = frame.union(bridge).union(lower_boss).union(upper_boss)

    hole_cutter = (
        cq.Workplane("XY")
        .pushPoints([(INPUT_X, 0.0), (IDLER_X, 0.0), (OUTPUT_X, 0.0)])
        .circle(SHAFT_RADIUS)
        .extrude(FRAME_TOP_Z + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return frame.cut(hole_cutter)


def add_shaft_part(
    model: ArticulatedObject,
    *,
    name: str,
    shaft_x: float,
    gear_radius: float,
    teeth: int,
) -> None:
    part = model.part(name)

    part.visual(
        Cylinder(radius=SHAFT_RADIUS, length=GEAR_BOTTOM_Z - (LOWER_COLLAR_CENTER_Z + (LOWER_COLLAR_THICKNESS * 0.5))),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5
                * (
                    GEAR_BOTTOM_Z
                    + LOWER_COLLAR_CENTER_Z
                    + (LOWER_COLLAR_THICKNESS * 0.5)
                ),
            )
        ),
        material="steel",
        name="lower_journal",
    )
    part.visual(
        Cylinder(radius=LOWER_COLLAR_RADIUS, length=LOWER_COLLAR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_CENTER_Z)),
        material="steel",
        name="lower_collar",
    )
    part.visual(
        Cylinder(radius=SHAFT_RADIUS, length=(UPPER_COLLAR_CENTER_Z - (UPPER_COLLAR_THICKNESS * 0.5)) - (GEAR_BOTTOM_Z + GEAR_THICKNESS + 0.005)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                0.5
                * (
                    UPPER_COLLAR_CENTER_Z
                    - (UPPER_COLLAR_THICKNESS * 0.5)
                    + GEAR_BOTTOM_Z
                    + GEAR_THICKNESS
                    + 0.005
                ),
            )
        ),
        material="steel",
        name="upper_journal",
    )
    part.visual(
        Cylinder(radius=UPPER_COLLAR_RADIUS, length=UPPER_COLLAR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, UPPER_COLLAR_CENTER_Z)),
        material="steel",
        name="upper_collar",
    )

    gear_mesh = make_gear_body(
        tip_radius=gear_radius,
        teeth=teeth,
        thickness=GEAR_THICKNESS,
        shaft_radius=SHAFT_RADIUS,
        hub_radius=max(gear_radius * 0.34, 0.012),
        hub_thickness=GEAR_THICKNESS + 0.010,
        window_count=4,
    )
    part.visual(
        mesh_from_cadquery(gear_mesh, f"{name}_gear"),
        origin=Origin(xyz=(0.0, 0.0, GEAR_BOTTOM_Z)),
        material="brass",
        name="gear",
    )

    model.articulation(
        f"frame_to_{name}",
        ArticulationType.REVOLUTE,
        parent="frame",
        child=part,
        origin=Origin(xyz=(shaft_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=10.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_spur_train")

    model.material("frame_paint", rgba=(0.19, 0.22, 0.26, 1.0))
    model.material("steel", rgba=(0.75, 0.77, 0.81, 1.0))
    model.material("brass", rgba=(0.79, 0.67, 0.28, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame(), "spur_train_frame"),
        material="frame_paint",
        name="frame_structure",
    )

    add_shaft_part(
        model,
        name="input_shaft",
        shaft_x=INPUT_X,
        gear_radius=INPUT_GEAR_RADIUS,
        teeth=INPUT_TEETH,
    )
    add_shaft_part(
        model,
        name="idler_shaft",
        shaft_x=IDLER_X,
        gear_radius=IDLER_GEAR_RADIUS,
        teeth=IDLER_TEETH,
    )
    add_shaft_part(
        model,
        name="output_shaft",
        shaft_x=OUTPUT_X,
        gear_radius=OUTPUT_GEAR_RADIUS,
        teeth=OUTPUT_TEETH,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")

    input_joint = object_model.get_articulation("frame_to_input_shaft")
    idler_joint = object_model.get_articulation("frame_to_idler_shaft")
    output_joint = object_model.get_articulation("frame_to_output_shaft")

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

    for joint in (input_joint, idler_joint, output_joint):
        ctx.check(
            f"{joint.name}_vertical_axis",
            tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"expected +Z shaft axis, got {joint.axis}",
        )

    for shaft in (input_shaft, idler_shaft, output_shaft):
        ctx.expect_contact(
            shaft,
            frame,
            elem_a="lower_collar",
            name=f"{shaft.name}_lower_collar_supported",
        )
        ctx.expect_contact(
            shaft,
            frame,
            elem_a="upper_collar",
            name=f"{shaft.name}_upper_collar_supported",
        )

    ctx.expect_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        positive_elem="gear",
        negative_elem="gear",
        min_gap=0.0004,
        max_gap=0.0022,
        name="input_to_idler_gear_clearance",
    )
    ctx.expect_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        positive_elem="gear",
        negative_elem="gear",
        min_gap=0.0004,
        max_gap=0.0022,
        name="idler_to_output_gear_clearance",
    )
    ctx.expect_overlap(
        input_shaft,
        idler_shaft,
        axes="z",
        elem_a="gear",
        elem_b="gear",
        min_overlap=0.011,
        name="input_and_idler_share_gear_plane",
    )
    ctx.expect_overlap(
        idler_shaft,
        output_shaft,
        axes="z",
        elem_a="gear",
        elem_b="gear",
        min_overlap=0.011,
        name="idler_and_output_share_gear_plane",
    )

    with ctx.pose(
        {
            input_joint: 0.8,
            idler_joint: -1.15,
            output_joint: 0.55,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_pose_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
