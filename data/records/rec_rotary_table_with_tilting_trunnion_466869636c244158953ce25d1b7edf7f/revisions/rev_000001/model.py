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


BASE_LENGTH = 0.82
BASE_WIDTH = 0.62
BASE_PLINTH_H = 0.10
PEDESTAL_BASE_LENGTH = 0.50
PEDESTAL_BASE_WIDTH = 0.38
PEDESTAL_TOP_LENGTH = 0.36
PEDESTAL_TOP_WIDTH = 0.28
PEDESTAL_H = 0.23
ROTARY_SEAT_RADIUS = 0.22
ROTARY_SEAT_H = 0.03
BODY_TOP_Z = BASE_PLINTH_H + PEDESTAL_H + ROTARY_SEAT_H

ROTARY_DRUM_RADIUS = 0.21
ROTARY_DRUM_H = 0.085
SADDLE_LENGTH = 0.38
SADDLE_WIDTH = 0.30
SADDLE_THICKNESS = 0.035
CHEEK_LENGTH = 0.34
CHEEK_THICKNESS = 0.035
CHEEK_HEIGHT = 0.20
CHEEK_CENTER_Y = 0.1425
YOKE_OUTER_WIDTH = 2.0 * (CHEEK_CENTER_Y + (CHEEK_THICKNESS / 2.0))
TRUNNION_RADIUS = 0.028
TRUNNION_AXIS_Z = 0.215

TABLE_LENGTH = 0.34
TABLE_WIDTH = 0.20
TABLE_THICKNESS = 0.035
TABLE_TOP_BOTTOM_Z = 0.03
TABLE_BLOCK_LENGTH = 0.18
TABLE_BLOCK_WIDTH = 0.16
TABLE_BLOCK_HEIGHT = 0.08
TABLE_BLOCK_CENTER_Z = -0.01
JOURNAL_CENTER_Y = 0.1025
JOURNAL_LENGTH = 0.045

LOWER_ROTARY_LIMIT = pi
UPPER_TILT_LIMIT = 0.6


def _build_body_shape() -> cq.Workplane:
    plinth = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_PLINTH_H,
        centered=(True, True, False),
    )

    pedestal = (
        cq.Workplane("XY", origin=(0.0, 0.0, BASE_PLINTH_H))
        .rect(PEDESTAL_BASE_LENGTH, PEDESTAL_BASE_WIDTH)
        .workplane(offset=PEDESTAL_H)
        .rect(PEDESTAL_TOP_LENGTH, PEDESTAL_TOP_WIDTH)
        .loft(combine=False)
    )

    rotary_seat = (
        cq.Workplane("XY", origin=(0.0, 0.0, BASE_PLINTH_H + PEDESTAL_H))
        .circle(ROTARY_SEAT_RADIUS)
        .extrude(ROTARY_SEAT_H)
    )

    body = plinth.union(pedestal).union(rotary_seat)

    front_relief = (
        cq.Workplane("YZ", origin=(0.12, BASE_WIDTH / 2.0, 0.13))
        .rect(0.18, 0.13)
        .extrude(0.055)
    )
    side_relief = (
        cq.Workplane("XZ", origin=(BASE_LENGTH / 2.0, 0.0, 0.16))
        .rect(0.22, 0.11)
        .extrude(0.045)
    )
    body = body.cut(front_relief).cut(side_relief).cut(side_relief.mirror("YZ"))

    return body


def _build_rotary_yoke_shape() -> cq.Workplane:
    drum = cq.Workplane("XY").circle(ROTARY_DRUM_RADIUS).extrude(ROTARY_DRUM_H)

    saddle = cq.Workplane("XY", origin=(0.0, 0.0, ROTARY_DRUM_H)).box(
        0.34,
        0.26,
        SADDLE_THICKNESS,
        centered=(True, True, False),
    )

    left_cheek = cq.Workplane(
        "XY",
        origin=(0.0, CHEEK_CENTER_Y, ROTARY_DRUM_H + SADDLE_THICKNESS),
    ).box(
        0.12,
        CHEEK_THICKNESS,
        0.19,
        centered=(True, True, False),
    )
    right_cheek = cq.Workplane(
        "XY",
        origin=(0.0, -CHEEK_CENTER_Y, ROTARY_DRUM_H + SADDLE_THICKNESS),
    ).box(
        0.12,
        CHEEK_THICKNESS,
        0.19,
        centered=(True, True, False),
    )

    yoke = drum.union(saddle).union(left_cheek).union(right_cheek)

    return yoke


def _build_upper_table_shape() -> cq.Workplane:
    table_plate = cq.Workplane("XY", origin=(0.0, 0.0, TABLE_TOP_BOTTOM_Z)).box(
        TABLE_LENGTH,
        TABLE_WIDTH,
        TABLE_THICKNESS,
        centered=(True, True, False),
    )
    table_block = cq.Workplane(
        "XY",
        origin=(0.0, 0.0, TABLE_BLOCK_CENTER_Z),
    ).box(
        TABLE_BLOCK_LENGTH,
        TABLE_BLOCK_WIDTH,
        TABLE_BLOCK_HEIGHT,
        centered=(True, True, True),
    )
    left_journal = (
        cq.Workplane("XZ", origin=(0.0, JOURNAL_CENTER_Y, 0.0))
        .circle(TRUNNION_RADIUS)
        .extrude(JOURNAL_LENGTH / 2.0, both=True)
    )
    right_journal = (
        cq.Workplane("XZ", origin=(0.0, -JOURNAL_CENTER_Y, 0.0))
        .circle(TRUNNION_RADIUS)
        .extrude(JOURNAL_LENGTH / 2.0, both=True)
    )
    front_rib = cq.Workplane(
        "XY",
        origin=(0.10, 0.0, 0.0),
    ).box(
        0.06,
        0.14,
        0.04,
        centered=(True, True, False),
    )

    table = table_plate.union(table_block).union(left_journal).union(right_journal).union(front_rib)
    table = (
        table.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, -0.07), (0.0, 0.0), (0.0, 0.07)])
        .slot2D(TABLE_LENGTH - 0.07, 0.012, angle=0.0)
        .cutBlind(-0.008)
    )
    table = (
        table.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.035)
        .cutBlind(-0.005)
    )

    return table


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_trunnion_table")

    base_paint = model.material("base_paint", rgba=(0.34, 0.37, 0.40, 1.0))
    stage_paint = model.material("stage_paint", rgba=(0.60, 0.63, 0.67, 1.0))
    machined_face = model.material("machined_face", rgba=(0.73, 0.76, 0.79, 1.0))
    dark_cover = model.material("dark_cover", rgba=(0.14, 0.15, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=base_paint,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BODY_TOP_Z)),
        mass=130.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z / 2.0)),
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=ROTARY_DRUM_RADIUS, length=ROTARY_DRUM_H),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DRUM_H / 2.0)),
        material=stage_paint,
        name="rotary_drum",
    )
    rotary_stage.visual(
        Box((0.34, 0.26, SADDLE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DRUM_H + (SADDLE_THICKNESS / 2.0))),
        material=stage_paint,
        name="rotary_saddle",
    )
    rotary_stage.visual(
        Box((0.12, CHEEK_THICKNESS, 0.19)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, ROTARY_DRUM_H + SADDLE_THICKNESS + 0.095)),
        material=stage_paint,
        name="left_cheek",
    )
    rotary_stage.visual(
        Box((0.12, CHEEK_THICKNESS, 0.19)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, ROTARY_DRUM_H + SADDLE_THICKNESS + 0.095)),
        material=stage_paint,
        name="right_cheek",
    )
    rotary_stage.visual(
        Box((0.11, 0.085, 0.075)),
        origin=Origin(xyz=(ROTARY_DRUM_RADIUS + 0.03, 0.0, 0.055)),
        material=dark_cover,
        name="drive_pod",
    )
    rotary_stage.inertial = Inertial.from_geometry(
        Box((0.48, 0.34, 0.29)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    upper_table = model.part("upper_table")
    upper_table.visual(
        mesh_from_cadquery(_build_upper_table_shape(), "upper_table_shell"),
        material=machined_face,
        name="upper_table_shell",
    )
    upper_table.visual(
        Box((0.05, 0.18, 0.03)),
        origin=Origin(xyz=(TABLE_LENGTH / 2.0 - 0.02, 0.0, 0.045)),
        material=dark_cover,
        name="front_probe",
    )
    upper_table.inertial = Inertial.from_geometry(
        Box((TABLE_LENGTH, 0.24, 0.14)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    model.articulation(
        "base_rotation",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.0,
            lower=-LOWER_ROTARY_LIMIT,
            upper=LOWER_ROTARY_LIMIT,
        ),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=rotary_stage,
        child=upper_table,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-UPPER_TILT_LIMIT,
            upper=UPPER_TILT_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    rotary_stage = object_model.get_part("rotary_stage")
    upper_table = object_model.get_part("upper_table")
    base_rotation = object_model.get_articulation("base_rotation")
    table_tilt = object_model.get_articulation("table_tilt")

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
    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        rotary_stage,
        body,
        name="rotary stage seats on the grounded body",
    )
    ctx.expect_contact(
        upper_table,
        rotary_stage,
        name="upper table is carried by the trunnion yoke",
    )
    ctx.expect_within(
        upper_table,
        rotary_stage,
        axes="y",
        margin=0.0,
        name="upper table stays captured between the side cheeks",
    )
    ctx.expect_gap(
        upper_table,
        body,
        axis="z",
        min_gap=0.02,
        name="tilt table clears the grounded body",
    )

    pod_rest = _aabb_center(ctx.part_element_world_aabb(rotary_stage, elem="drive_pod"))
    with ctx.pose({base_rotation: pi / 2.0}):
        pod_turned = _aabb_center(ctx.part_element_world_aabb(rotary_stage, elem="drive_pod"))
    ctx.check(
        "lower rotary base turns about vertical axis",
        pod_rest is not None
        and pod_turned is not None
        and pod_turned[1] > pod_rest[1] + 0.15
        and abs(pod_turned[0]) < pod_rest[0],
        details=f"rest={pod_rest}, turned={pod_turned}",
    )

    front_rest = _aabb_center(ctx.part_element_world_aabb(upper_table, elem="front_probe"))
    with ctx.pose({table_tilt: 0.6}):
        front_tilted = _aabb_center(ctx.part_element_world_aabb(upper_table, elem="front_probe"))
    ctx.check(
        "upper work face tilts upward about the trunnion axis",
        front_rest is not None
        and front_tilted is not None
        and front_tilted[2] > front_rest[2] + 0.05,
        details=f"rest={front_rest}, tilted={front_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
