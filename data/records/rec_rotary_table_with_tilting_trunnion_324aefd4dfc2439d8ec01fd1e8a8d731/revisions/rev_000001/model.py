from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.26
BASE_HEIGHT = 0.045
SPINDLE_PAD_DIAMETER = 0.16
SPINDLE_PAD_HEIGHT = 0.012

ROTARY_DISC_DIAMETER = 0.21
ROTARY_DISC_HEIGHT = 0.028

CHEEK_THICKNESS = 0.024
CHEEK_LENGTH = 0.15
CHEEK_HEIGHT = 0.18
CHEEK_GAP = 0.164
CHEEK_CENTER_X = CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0
TRUNNION_AXIS_HEIGHT = 0.125
TRUNNION_HOLE_RADIUS = 0.018

TABLE_WIDTH = 0.14
TABLE_LENGTH = 0.18
TABLE_THICKNESS = 0.03
TABLE_SLOT_WIDTH = 0.012
TABLE_SLOT_DEPTH = 0.005
TABLE_SLOT_OFFSET = 0.04

TRUNNION_BOSS_RADIUS = 0.028
TRUNNION_BOSS_LENGTH = 0.012
JOURNAL_RADIUS = 0.016
JOURNAL_LENGTH = CHEEK_GAP / 2.0 - TABLE_WIDTH / 2.0 + CHEEK_THICKNESS - TRUNNION_BOSS_LENGTH

BASE_ROTATION_EFFORT = 25.0
BASE_ROTATION_VELOCITY = 1.5
TABLE_TILT_EFFORT = 35.0
TABLE_TILT_VELOCITY = 1.0
TABLE_TILT_LOWER = -0.35
TABLE_TILT_UPPER = 1.92


def make_base_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    body = body.edges("|Z").fillet(0.01)
    body = body.faces(">Z").edges().chamfer(0.003)
    return body


def make_cheek_plate() -> cq.Workplane:
    cheek = cq.Workplane("XY").box(
        CHEEK_THICKNESS,
        CHEEK_LENGTH,
        CHEEK_HEIGHT,
        centered=(True, True, False),
    )
    cheek = cheek.translate((0.0, 0.0, ROTARY_DISC_HEIGHT))

    lightening_window = cq.Workplane("XY").box(
        CHEEK_THICKNESS + 0.008,
        0.082,
        0.05,
        centered=(True, True, False),
    )
    lightening_window = lightening_window.translate((0.0, 0.0, ROTARY_DISC_HEIGHT + 0.032))

    trunnion_bore = cq.Workplane("YZ").circle(TRUNNION_HOLE_RADIUS).extrude(CHEEK_THICKNESS + 0.008)
    trunnion_bore = trunnion_bore.translate(
        (-CHEEK_THICKNESS / 2.0 - 0.004, 0.0, TRUNNION_AXIS_HEIGHT)
    )

    cheek = cheek.cut(lightening_window).cut(trunnion_bore)
    cheek = cheek.faces(">Z").edges().fillet(0.006)
    return cheek


def make_table_core() -> cq.Workplane:
    plate = cq.Workplane("XY").box(TABLE_WIDTH, TABLE_LENGTH, TABLE_THICKNESS)
    plate = plate.edges("|Z").fillet(0.004)
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-TABLE_SLOT_OFFSET, 0.0),
                (0.0, 0.0),
                (TABLE_SLOT_OFFSET, 0.0),
            ]
        )
        .rect(TABLE_SLOT_WIDTH, TABLE_LENGTH - 0.03)
        .cutBlind(-TABLE_SLOT_DEPTH)
    )

    left_boss = cq.Workplane("YZ").circle(TRUNNION_BOSS_RADIUS).extrude(TRUNNION_BOSS_LENGTH)
    left_boss = left_boss.translate((-TABLE_WIDTH / 2.0 - TRUNNION_BOSS_LENGTH, 0.0, 0.0))

    right_boss = cq.Workplane("YZ").circle(TRUNNION_BOSS_RADIUS).extrude(TRUNNION_BOSS_LENGTH)
    right_boss = right_boss.translate((TABLE_WIDTH / 2.0, 0.0, 0.0))

    return plate.union(left_boss).union(right_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_table_fixture")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.26, 0.28, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.33, 0.37, 0.43, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.71, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_body(), "base_body"),
        material=cast_iron,
        name="base_body",
    )
    base.visual(
        Cylinder(SPINDLE_PAD_DIAMETER / 2.0, SPINDLE_PAD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + SPINDLE_PAD_HEIGHT / 2.0)),
        material=blued_steel,
        name="spindle_pad",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(ROTARY_DISC_DIAMETER / 2.0, ROTARY_DISC_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_DISC_HEIGHT / 2.0)),
        material=blued_steel,
        name="rotary_disc",
    )
    cradle.visual(
        mesh_from_cadquery(
            make_cheek_plate().translate((-CHEEK_CENTER_X, 0.0, 0.0)),
            "left_cheek",
        ),
        material=cast_iron,
        name="left_cheek",
    )
    cradle.visual(
        mesh_from_cadquery(
            make_cheek_plate().translate((CHEEK_CENTER_X, 0.0, 0.0)),
            "right_cheek",
        ),
        material=cast_iron,
        name="right_cheek",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(make_table_core(), "table_core"),
        material=machined_steel,
        name="table_plate",
    )
    table.visual(
        Cylinder(JOURNAL_RADIUS, JOURNAL_LENGTH),
        origin=Origin(
            xyz=(
                -TABLE_WIDTH / 2.0 - TRUNNION_BOSS_LENGTH - JOURNAL_LENGTH / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=blued_steel,
        name="left_journal",
    )
    table.visual(
        Cylinder(JOURNAL_RADIUS, JOURNAL_LENGTH),
        origin=Origin(
            xyz=(
                TABLE_WIDTH / 2.0 + TRUNNION_BOSS_LENGTH + JOURNAL_LENGTH / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=blued_steel,
        name="right_journal",
    )

    model.articulation(
        "base_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + SPINDLE_PAD_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=BASE_ROTATION_EFFORT,
            velocity=BASE_ROTATION_VELOCITY,
        ),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=TABLE_TILT_EFFORT,
            velocity=TABLE_TILT_VELOCITY,
            lower=TABLE_TILT_LOWER,
            upper=TABLE_TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cradle = object_model.get_part("cradle")
    table = object_model.get_part("table")
    base_spin = object_model.get_articulation("base_spin")
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

    ctx.check(
        "base joint is a vertical continuous rotary axis",
        base_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(base_spin.axis) == (0.0, 0.0, 1.0)
        and base_spin.motion_limits is not None
        and base_spin.motion_limits.lower is None
        and base_spin.motion_limits.upper is None,
        f"got type={base_spin.articulation_type}, axis={base_spin.axis}, limits={base_spin.motion_limits}",
    )
    ctx.check(
        "table joint is a horizontal trunnion tilt axis",
        table_tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(table_tilt.axis) == (1.0, 0.0, 0.0)
        and table_tilt.motion_limits is not None
        and table_tilt.motion_limits.lower == TABLE_TILT_LOWER
        and table_tilt.motion_limits.upper == TABLE_TILT_UPPER,
        f"got type={table_tilt.articulation_type}, axis={table_tilt.axis}, limits={table_tilt.motion_limits}",
    )
    ctx.expect_contact(
        cradle,
        base,
        elem_a="rotary_disc",
        elem_b="spindle_pad",
        contact_tol=0.001,
        name="rotary disc is seated on the stationary base pad",
    )
    ctx.expect_overlap(
        table,
        cradle,
        axes="yz",
        min_overlap=0.028,
        elem_a="left_journal",
        elem_b="left_cheek",
        name="left trunnion journal aligns with the left cheek",
    )
    ctx.expect_overlap(
        table,
        cradle,
        axes="yz",
        min_overlap=0.028,
        elem_a="right_journal",
        elem_b="right_cheek",
        name="right trunnion journal aligns with the right cheek",
    )
    ctx.expect_gap(
        table,
        cradle,
        axis="z",
        min_gap=0.06,
        max_gap=0.09,
        positive_elem="table_plate",
        negative_elem="rotary_disc",
        name="table rides above the rotary base instead of floating far above it",
    )

    with ctx.pose({table_tilt: 1.2}):
        ctx.expect_gap(
            table,
            cradle,
            axis="z",
            min_gap=0.006,
            positive_elem="table_plate",
            negative_elem="rotary_disc",
            name="tilted table still clears the rotary disc",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
