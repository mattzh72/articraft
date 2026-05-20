from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


BASE_LENGTH = 0.48
BASE_WIDTH = 0.36
BASE_THICKNESS = 0.018

LOWER_AXIS_Z = 0.064
LOWER_STAGE_Z_SHIFT = 0.044

DRUM_OUTER_RADIUS = 0.165
DRUM_INNER_RADIUS = 0.118
DRUM_BOTTOM_Z = -0.040 + LOWER_STAGE_Z_SHIFT
DRUM_TOP_Z = 0.060 + LOWER_STAGE_Z_SHIFT
DRUM_TOP_THICKNESS = 0.012

MAIN_SPINDLE_RADIUS = 0.027
MAIN_SPINDLE_BOTTOM_Z = -0.040
MAIN_THRUST_RADIUS = 0.060

BRIDGE_X0 = 0.082
BRIDGE_X1 = 0.220
BRIDGE_Y0 = -0.055
BRIDGE_Y1 = 0.035
BRIDGE_Z0 = 0.048 + LOWER_STAGE_Z_SHIFT
BRIDGE_Z1 = 0.072 + LOWER_STAGE_Z_SHIFT

UPPER_AXIS_X = 0.198
UPPER_AXIS_Y = 0.028
UPPER_AXIS_Z = 0.096 + LOWER_STAGE_Z_SHIFT
UPPER_BARREL_RADIUS = 0.048
UPPER_BORE_RADIUS = 0.022

UPPER_FLANGE_RADIUS = 0.075
UPPER_SPINDLE_RADIUS = 0.018
UPPER_SPINDLE_BOTTOM_Z = -0.042


def polar_points(
    radius: float,
    angles_deg: list[float],
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(angle_deg * pi / 180.0),
            cy + radius * sin(angle_deg * pi / 180.0),
        )
        for angle_deg in angles_deg
    ]


def make_cap_screw(
    *,
    head_radius: float,
    head_height: float,
    shank_radius: float,
    shank_length: float,
    socket_radius: float,
    socket_depth: float,
) -> cq.Workplane:
    screw = cq.Workplane("XY").circle(head_radius).extrude(head_height)
    screw = (
        screw.faces("<Z")
        .workplane()
        .circle(shank_radius)
        .extrude(-shank_length)
    )
    screw = (
        screw.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polygon(6, socket_radius * 2.0)
        .cutBlind(-socket_depth)
    )
    return screw


def add_vertical_screws(
    body: cq.Workplane,
    *,
    points: list[tuple[float, float]],
    z: float,
    head_radius: float,
    head_height: float,
    shank_radius: float,
    shank_length: float,
    socket_radius: float,
    socket_depth: float,
) -> cq.Workplane:
    screw = make_cap_screw(
        head_radius=head_radius,
        head_height=head_height,
        shank_radius=shank_radius,
        shank_length=shank_length,
        socket_radius=socket_radius,
        socket_depth=socket_depth,
    )
    for x, y in points:
        body = body.union(screw.translate((x, y, z)))
    return body


def make_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").rect(BASE_LENGTH, BASE_WIDTH).extrude(BASE_THICKNESS)
    base = base.edges("|Z").fillet(0.010)

    lower_flange = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .circle(0.138)
        .extrude(0.012)
    )
    lower_barrel = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + 0.012)
        .circle(0.102)
        .extrude(0.024)
    )
    top_support = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_AXIS_Z - 0.010)
        .circle(MAIN_THRUST_RADIUS)
        .extrude(0.010)
    )
    base = base.union(lower_flange).union(lower_barrel).union(top_support)

    base = base.cut(
        cq.Workplane("XY")
        .workplane(offset=0.020)
        .circle(0.030)
        .extrude(LOWER_AXIS_Z - 0.020)
    )

    base = add_vertical_screws(
        base,
        points=[
            (-0.180, -0.120),
            (-0.180, 0.120),
            (0.180, -0.120),
            (0.180, 0.120),
        ],
        z=BASE_THICKNESS,
        head_radius=0.007,
        head_height=0.006,
        shank_radius=0.0028,
        shank_length=0.010,
        socket_radius=0.0030,
        socket_depth=0.0025,
    )
    return base


def make_lower_stage_shape() -> cq.Workplane:
    thrust_pad = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(0.062)
        .circle(0.031)
        .extrude(0.008)
    )
    drum_shell = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(DRUM_OUTER_RADIUS)
        .circle(0.121)
        .extrude(0.078)
    )
    drum_top_ring = (
        cq.Workplane("XY")
        .workplane(offset=0.084)
        .circle(0.147)
        .circle(0.094)
        .extrude(0.010)
    )
    lower = thrust_pad.union(drum_shell).union(drum_top_ring)

    bridge = (
        cq.Workplane("XY")
        .workplane(offset=0.088)
        .center((BRIDGE_X0 + BRIDGE_X1) / 2.0, (BRIDGE_Y0 + BRIDGE_Y1) / 2.0)
        .rect(BRIDGE_X1 - BRIDGE_X0, BRIDGE_Y1 - BRIDGE_Y0)
        .extrude(0.024)
    )
    outer_cheek = (
        cq.Workplane("XY")
        .workplane(offset=0.088)
        .center(0.176, 0.038)
        .rect(0.096, 0.014)
        .extrude(0.054)
    )
    housing_base_flange = (
        cq.Workplane("XY")
        .workplane(offset=0.114)
        .center(UPPER_AXIS_X, UPPER_AXIS_Y)
        .circle(0.066)
        .extrude(0.012)
    )
    housing_barrel = (
        cq.Workplane("XY")
        .workplane(offset=0.094)
        .center(UPPER_AXIS_X, UPPER_AXIS_Y)
        .circle(UPPER_BARREL_RADIUS)
        .extrude(0.040)
    )
    split_collar = (
        cq.Workplane("XY")
        .workplane(offset=0.134)
        .center(UPPER_AXIS_X, UPPER_AXIS_Y)
        .circle(0.058)
        .circle(0.029)
        .extrude(0.006)
    )
    split_collar = split_collar.cut(
        cq.Workplane("XY")
        .workplane(offset=0.133)
        .center(UPPER_AXIS_X, UPPER_AXIS_Y + 0.054)
        .rect(0.026, 0.012)
        .extrude(0.009)
    )
    clamp_ear_left = (
        cq.Workplane("XY")
        .workplane(offset=0.128)
        .center(UPPER_AXIS_X - 0.016, UPPER_AXIS_Y + 0.056)
        .rect(0.020, 0.018)
        .extrude(0.012)
    )
    clamp_ear_right = (
        cq.Workplane("XY")
        .workplane(offset=0.128)
        .center(UPPER_AXIS_X + 0.016, UPPER_AXIS_Y + 0.056)
        .rect(0.020, 0.018)
        .extrude(0.012)
    )

    primary_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.055, 0.088),
                (0.118, 0.088),
                (0.214, 0.126),
                (0.214, 0.112),
                (0.055, 0.112),
            ]
        )
        .close()
        .extrude(0.024)
        .translate((0.0, 0.004, 0.0))
    )
    inner_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.028, 0.082),
                (0.084, 0.082),
                (0.160, 0.104),
                (0.160, 0.096),
                (0.028, 0.096),
            ]
        )
        .close()
        .extrude(0.020)
        .translate((0.0, -0.032, 0.0))
    )

    lower = (
        lower.union(bridge)
        .union(outer_cheek)
        .union(housing_base_flange)
        .union(housing_barrel)
        .union(split_collar)
        .union(clamp_ear_left)
        .union(clamp_ear_right)
        .union(primary_rib)
        .union(inner_rib)
    )

    lower = lower.cut(
        cq.Workplane("XY")
        .workplane(offset=0.094)
        .center(UPPER_AXIS_X, UPPER_AXIS_Y)
        .circle(0.025)
        .extrude(0.046)
    )

    lower = add_vertical_screws(
        lower,
        points=polar_points(0.131, [70, 130, 190, 250, 310]),
        z=0.094,
        head_radius=0.0055,
        head_height=0.006,
        shank_radius=0.0025,
        shank_length=0.008,
        socket_radius=0.0025,
        socket_depth=0.0020,
    )
    lower = add_vertical_screws(
        lower,
        points=polar_points(
            0.055,
            [45, 135, 225, 315],
            center=(UPPER_AXIS_X, UPPER_AXIS_Y),
        ),
        z=0.126,
        head_radius=0.0045,
        head_height=0.005,
        shank_radius=0.0020,
        shank_length=0.008,
        socket_radius=0.0020,
        socket_depth=0.0018,
    )
    lower = add_vertical_screws(
        lower,
        points=[
            (UPPER_AXIS_X - 0.016, UPPER_AXIS_Y + 0.056),
            (UPPER_AXIS_X + 0.016, UPPER_AXIS_Y + 0.056),
        ],
        z=0.134,
        head_radius=0.0045,
        head_height=0.005,
        shank_radius=0.0020,
        shank_length=0.008,
        socket_radius=0.0020,
        socket_depth=0.0018,
    )

    return lower


def make_upper_flange_shape() -> cq.Workplane:
    thrust_disc = cq.Workplane("XY").circle(0.041).extrude(0.006)
    hub_barrel = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(0.030)
        .extrude(0.018)
    )
    flange_disc = (
        cq.Workplane("XY")
        .workplane(offset=0.020)
        .circle(UPPER_FLANGE_RADIUS)
        .extrude(0.014)
    )
    pilot_boss = (
        cq.Workplane("XY")
        .workplane(offset=0.034)
        .circle(0.022)
        .extrude(0.008)
    )
    upper = thrust_disc.union(hub_barrel).union(flange_disc).union(pilot_boss)

    upper = add_vertical_screws(
        upper,
        points=polar_points(0.051, [0, 60, 120, 180, 240, 300]),
        z=0.034,
        head_radius=0.0045,
        head_height=0.005,
        shank_radius=0.0020,
        shank_length=0.008,
        socket_radius=0.0020,
        socket_depth=0.0018,
    )

    return upper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_bridge_rotary_stack")

    base_material = model.material("base_cast", rgba=(0.19, 0.20, 0.22, 1.0))
    rotor_material = model.material("rotor_steel", rgba=(0.46, 0.48, 0.51, 1.0))
    upper_material = model.material("upper_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base"),
        material=base_material,
        name="base_shell",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(make_lower_stage_shape(), "lower_stage"),
        material=rotor_material,
        name="lower_stage_shell",
    )

    upper_flange = model.part("upper_flange")
    upper_flange.visual(
        mesh_from_cadquery(make_upper_flange_shape(), "upper_flange"),
        material=upper_material,
        name="upper_flange_shell",
    )

    model.articulation(
        "lower_drum_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.4,
            lower=-3.1,
            upper=3.1,
        ),
    )

    model.articulation(
        "upper_flange_joint",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_flange,
        origin=Origin(xyz=(UPPER_AXIS_X, UPPER_AXIS_Y, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=-2.7,
            upper=2.7,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_flange = object_model.get_part("upper_flange")
    lower_joint = object_model.get_articulation("lower_drum_joint")
    upper_joint = object_model.get_articulation("upper_flange_joint")
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
        lower_stage,
        base,
        name="lower drum is supported on the stationary base housing",
    )
    ctx.expect_contact(
        upper_flange,
        lower_stage,
        name="upper flange is supported by the bridge housing",
    )
    ctx.expect_origin_distance(
        upper_flange,
        lower_stage,
        axes="xy",
        min_dist=0.19,
        max_dist=0.22,
        name="upper rotary axis is deliberately offset from the main axis",
    )
    ctx.expect_origin_gap(
        upper_flange,
        lower_stage,
        axis="z",
        min_gap=0.13,
        max_gap=0.15,
        name="upper stage rides above the bridge rather than sitting on the drum",
    )
    ctx.check(
        "both rotary joints stay vertical and parallel",
        lower_joint.axis == (0.0, 0.0, 1.0) and upper_joint.axis == (0.0, 0.0, 1.0),
        details=f"lower axis={lower_joint.axis}, upper axis={upper_joint.axis}",
    )
    ctx.check(
        "upper joint origin is edge-carried and offset",
        upper_joint.origin.xyz[0] > 0.18 and upper_joint.origin.xyz[1] > 0.02,
        details=f"upper joint origin={upper_joint.origin.xyz}",
    )

    with ctx.pose({lower_joint: 1.8, upper_joint: -1.9}):
        ctx.expect_contact(
            lower_stage,
            base,
            name="lower support contact survives an off-axis drum pose",
        )
        ctx.expect_contact(
            upper_flange,
            lower_stage,
            name="upper support contact survives compound rotation",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
