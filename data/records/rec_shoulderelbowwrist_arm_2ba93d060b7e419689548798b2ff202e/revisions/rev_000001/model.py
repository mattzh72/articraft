from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_SIZE = 0.28
BASE_PLATE_THICKNESS = 0.025
COLUMN_SIZE = 0.16
COLUMN_HEIGHT = 0.20
SHOULDER_HOUSING_SIZE = 0.22
SHOULDER_HOUSING_HEIGHT = 0.038
SHOULDER_DRUM_RADIUS = 0.072
SHOULDER_DRUM_HEIGHT = 0.022
SHOULDER_Z = (
    BASE_PLATE_THICKNESS
    + COLUMN_HEIGHT
    + SHOULDER_HOUSING_HEIGHT
    + SHOULDER_DRUM_HEIGHT
)

ELBOW_X = 0.255
ELBOW_Z = 0.105
UPPER_CHEEK_THICKNESS = 0.014
UPPER_CHEEK_INNER_Y = 0.027
UPPER_CHEEK_OUTER_Y = UPPER_CHEEK_INNER_Y + UPPER_CHEEK_THICKNESS

FOREARM_LENGTH = 0.18
FOREARM_ROOT_DISC_RADIUS = 0.034
FOREARM_ROOT_DISC_LENGTH = 0.012
FOREARM_ROOT_DISC_CENTER_Y = UPPER_CHEEK_OUTER_Y + FOREARM_ROOT_DISC_LENGTH / 2.0
FOREARM_CLEVIS_THICKNESS = 0.012
FOREARM_CLEVIS_INNER_Y = 0.018
FOREARM_CLEVIS_OUTER_Y = FOREARM_CLEVIS_INNER_Y + FOREARM_CLEVIS_THICKNESS

WRIST_DISC_RADIUS = 0.026
WRIST_DISC_LENGTH = 0.011
WRIST_DISC_CENTER_Y = FOREARM_CLEVIS_OUTER_Y + WRIST_DISC_LENGTH / 2.0


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _regular_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(phase + index * 2.0 * pi / count),
            radius * sin(phase + index * 2.0 * pi / count),
        )
        for index in range(count)
    ]


def _base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_PLATE_SIZE,
        BASE_PLATE_SIZE,
        BASE_PLATE_THICKNESS,
        centered=(True, True, False),
    )
    column = (
        cq.Workplane("XY")
        .box(COLUMN_SIZE, COLUMN_SIZE, COLUMN_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS))
    )
    shoulder_box = (
        cq.Workplane("XY")
        .box(
            SHOULDER_HOUSING_SIZE,
            SHOULDER_HOUSING_SIZE,
            SHOULDER_HOUSING_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS + COLUMN_HEIGHT))
    )
    shoulder_drum = (
        cq.Workplane("XY")
        .circle(SHOULDER_DRUM_RADIUS)
        .extrude(SHOULDER_DRUM_HEIGHT)
        .translate((0.0, 0.0, SHOULDER_Z - SHOULDER_DRUM_HEIGHT))
    )
    mount_bolts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.098, -0.098),
                (-0.098, 0.098),
                (0.098, -0.098),
                (0.098, 0.098),
            ]
        )
        .polygon(6, 0.018)
        .extrude(0.007)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS - 0.001))
    )

    return plate.union(column).union(shoulder_box).union(shoulder_drum).union(mount_bolts)


def _upper_link_shape() -> cq.Workplane:
    rotary_flange = cq.Workplane("XY").circle(0.082).extrude(0.018)
    shoulder_collar = (
        cq.Workplane("XY").circle(0.046).extrude(0.050).translate((0.0, 0.0, 0.009))
    )
    body_core = cq.Workplane("XY").box(0.150, 0.082, 0.054).translate((0.080, 0.0, 0.038))
    top_spine = cq.Workplane("XY").box(0.072, 0.036, 0.020).translate((0.145, 0.0, 0.078))
    brace_pos = cq.Workplane("XY").box(0.040, UPPER_CHEEK_THICKNESS, 0.018).translate(
        (0.188, UPPER_CHEEK_INNER_Y + UPPER_CHEEK_THICKNESS / 2.0, 0.082)
    )
    brace_neg = cq.Workplane("XY").box(0.040, UPPER_CHEEK_THICKNESS, 0.018).translate(
        (0.188, -UPPER_CHEEK_INNER_Y - UPPER_CHEEK_THICKNESS / 2.0, 0.082)
    )
    cheek_pos = cq.Workplane("XY").box(0.024, UPPER_CHEEK_THICKNESS, 0.074).translate(
        (0.225, UPPER_CHEEK_INNER_Y + UPPER_CHEEK_THICKNESS / 2.0, ELBOW_Z)
    )
    cheek_neg = cq.Workplane("XY").box(0.024, UPPER_CHEEK_THICKNESS, 0.074).translate(
        (0.225, -UPPER_CHEEK_INNER_Y - UPPER_CHEEK_THICKNESS / 2.0, ELBOW_Z)
    )
    trunnion_cap_pos = _cylinder_y(0.012, 0.006, (0.227, 0.045, ELBOW_Z))
    trunnion_cap_neg = _cylinder_y(0.012, 0.006, (0.227, -0.045, ELBOW_Z))
    flange_bolts = (
        cq.Workplane("XY")
        .pushPoints(_regular_points(0.058, 6, phase=pi / 6.0))
        .polygon(6, 0.013)
        .extrude(0.007)
        .translate((0.0, 0.0, 0.019))
    )

    return (
        rotary_flange.union(shoulder_collar)
        .union(body_core)
        .union(top_spine)
        .union(brace_pos)
        .union(brace_neg)
        .union(cheek_pos)
        .union(cheek_neg)
        .union(trunnion_cap_pos)
        .union(trunnion_cap_neg)
        .union(flange_bolts)
    )


def _forearm_shape() -> cq.Workplane:
    root_hub = _cylinder_y(0.018, 2.0 * UPPER_CHEEK_INNER_Y, (0.0, 0.0, 0.0))
    root_block = cq.Workplane("XY").box(0.032, 0.028, 0.026).translate((0.024, 0.0, 0.0))
    main_beam = cq.Workplane("XY").box(0.078, 0.040, 0.034).translate((0.090, 0.0, 0.0))
    beam_window = cq.Workplane("XY").box(0.050, 0.024, 0.014).translate((0.092, 0.0, 0.0))
    main_beam = main_beam.cut(beam_window)
    top_rib = cq.Workplane("XY").box(0.056, 0.014, 0.012).translate((0.098, 0.0, 0.022))
    lower_rib = cq.Workplane("XY").box(0.026, 0.016, 0.012).translate((0.124, 0.0, -0.017))
    nose_bridge = cq.Workplane("XY").box(0.022, 0.022, 0.018).translate((0.144, 0.0, 0.0))
    cheek_pos = cq.Workplane("XY").box(0.020, FOREARM_CLEVIS_THICKNESS, 0.042).translate(
        (0.170, FOREARM_CLEVIS_INNER_Y + FOREARM_CLEVIS_THICKNESS / 2.0, 0.0)
    )
    cheek_neg = cq.Workplane("XY").box(0.020, FOREARM_CLEVIS_THICKNESS, 0.042).translate(
        (0.170, -FOREARM_CLEVIS_INNER_Y - FOREARM_CLEVIS_THICKNESS / 2.0, 0.0)
    )

    return (
        root_hub.union(root_block)
        .union(main_beam)
        .union(top_rib)
        .union(lower_rib)
        .union(nose_bridge)
        .union(cheek_pos)
        .union(cheek_neg)
    )


def _wrist_shape() -> cq.Workplane:
    pivot_hub = _cylinder_y(0.009, 2.0 * FOREARM_CLEVIS_INNER_Y, (0.007, 0.0, 0.0))
    pivot_lug = cq.Workplane("XY").box(0.010, 0.016, 0.012).translate((0.005, 0.0, 0.0))
    neck = cq.Workplane("XY").box(0.016, 0.020, 0.014).translate((0.026, 0.0, 0.0))
    wrist_body = cq.Workplane("XY").box(0.040, 0.038, 0.046).translate((0.062, 0.0, 0.0))
    top_cap = cq.Workplane("XY").box(0.024, 0.028, 0.010).translate((0.064, 0.0, 0.028))
    nose_flange = cq.Workplane("XY").box(0.010, 0.068, 0.068).translate((0.086, 0.0, 0.0))
    flange_bolts = (
        cq.Workplane("YZ")
        .pushPoints([(-0.020, -0.020), (-0.020, 0.020), (0.020, -0.020), (0.020, 0.020)])
        .polygon(6, 0.010)
        .extrude(0.007)
        .translate((0.091, 0.0, 0.0))
    )

    return (
        pivot_hub.union(pivot_lug)
        .union(neck)
        .union(wrist_body)
        .union(top_cap)
        .union(nose_flange)
        .union(flange_bolts)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_manipulator")

    model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machine_gray", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("satin_alloy", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("safety_orange", rgba=(0.84, 0.47, 0.15, 1.0))

    base_column = model.part("base_column")
    base_column.visual(
        mesh_from_cadquery(_base_shape(), "base_column"),
        material="graphite",
        name="base_shell",
    )
    base_column.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_SIZE, BASE_PLATE_SIZE, SHOULDER_Z)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z / 2.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_upper_link_shape(), "upper_link"),
        material="machine_gray",
        name="upper_shell",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.29, 0.10, 0.15)),
        mass=7.0,
        origin=Origin(xyz=(0.13, 0.0, 0.07)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm"),
        material="satin_alloy",
        name="forearm_shell",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.21, 0.09, 0.08)),
        mass=4.2,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    wrist_block = model.part("wrist_block")
    wrist_block.visual(
        mesh_from_cadquery(_wrist_shape(), "wrist_block"),
        material="safety_orange",
        name="wrist_shell",
    )
    wrist_block.inertial = Inertial.from_geometry(
        Box((0.11, 0.09, 0.09)),
        mass=1.8,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base_column,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=180.0, velocity=1.6),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(ELBOW_X, 0.0, ELBOW_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.95, upper=1.15, effort=95.0, velocity=1.9),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_block,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.60, effort=32.0, velocity=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_block")

    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist_joint = object_model.get_articulation("wrist_pitch")

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

    ctx.check(
        "joint_axes_match_prompt",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 1.0, 0.0)
        and wrist_joint.axis == (0.0, 1.0, 0.0),
        (
            f"expected axes ((0,0,1), (0,1,0), (0,1,0)); "
            f"got {shoulder.axis}, {elbow.axis}, {wrist_joint.axis}"
        ),
    )
    ctx.expect_origin_gap(
        upper,
        base,
        axis="z",
        min_gap=0.28,
        max_gap=0.29,
        name="shoulder_axis_height",
    )
    ctx.expect_origin_gap(
        forearm,
        upper,
        axis="x",
        min_gap=0.24,
        max_gap=0.27,
        name="elbow_reach",
    )
    ctx.expect_origin_gap(
        wrist,
        forearm,
        axis="x",
        min_gap=0.17,
        max_gap=0.19,
        name="wrist_reach",
    )
    ctx.expect_contact(upper, base, name="shoulder_mount_contact")
    ctx.expect_contact(forearm, upper, name="elbow_trunnion_contact")
    ctx.expect_contact(wrist, forearm, name="wrist_trunnion_contact")

    with ctx.pose({elbow: 0.95, wrist_joint: -0.45}):
        ctx.expect_contact(forearm, upper, name="elbow_contact_in_flexed_pose")
        ctx.expect_contact(wrist, forearm, name="wrist_contact_in_flexed_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
