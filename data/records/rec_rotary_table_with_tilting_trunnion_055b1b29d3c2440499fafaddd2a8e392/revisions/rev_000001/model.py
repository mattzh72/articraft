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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.400
BASE_WIDTH = 0.280
BASE_HEIGHT = 0.055
HOUSING_LOWER_RADIUS = 0.095
HOUSING_LOWER_HEIGHT = 0.016
HOUSING_UPPER_RADIUS = 0.072
HOUSING_UPPER_HEIGHT = 0.024
BASE_ROTARY_Z = BASE_HEIGHT + HOUSING_LOWER_HEIGHT + HOUSING_UPPER_HEIGHT

PLATTER_RADIUS = 0.140
PLATTER_THICKNESS = 0.028
PLATTER_RECESS_RADIUS = 0.116
PLATTER_RECESS_DEPTH = 0.004

SUPPORT_CENTER_X = 0.115
SUPPORT_THICKNESS = 0.024
SUPPORT_DEPTH = 0.156
TRUNNION_AXIS_HEIGHT = 0.112
SUPPORT_TOP_HEIGHT = 0.150

SUPPORT_BOSS_OUTER_RADIUS = 0.030
SHAFT_RADIUS = 0.010
BORE_RADIUS = SHAFT_RADIUS
SHAFT_HALF_LENGTH = SUPPORT_CENTER_X - SUPPORT_THICKNESS / 2.0

TABLE_LENGTH = 0.190
TABLE_WIDTH = 0.160
TABLE_THICKNESS = 0.022
TABLE_BOTTOM_Z = 0.058
TABLE_SLOT_WIDTH = 0.012
TABLE_SLOT_LENGTH = 0.126
TABLE_SLOT_DEPTH = 0.004

FRAME_BLOCK_LENGTH = 0.060
FRAME_BLOCK_WIDTH = 0.038
FRAME_BLOCK_HEIGHT = 0.026
COLUMN_LENGTH = 0.050
COLUMN_WIDTH = 0.038
COLUMN_BASE_Z = FRAME_BLOCK_HEIGHT - SHAFT_RADIUS
COLUMN_HEIGHT = 0.030
SADDLE_LENGTH = 0.120
SADDLE_WIDTH = 0.050
SADDLE_THICKNESS = 0.014
SADDLE_BASE_Z = TABLE_BOTTOM_Z - SADDLE_THICKNESS
GUSSET_Y_OFFSET = 0.019
GUSSET_HALF_THICKNESS = 0.006


def _make_base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.015)
    )
    body = body.edges(">Z").fillet(0.008)

    lower_housing = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(HOUSING_LOWER_RADIUS)
        .extrude(HOUSING_LOWER_HEIGHT)
    )
    upper_housing = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT + HOUSING_LOWER_HEIGHT)
        .circle(HOUSING_UPPER_RADIUS)
        .extrude(HOUSING_UPPER_HEIGHT)
    )

    front_recess = (
        cq.Workplane("XZ")
        .workplane(offset=BASE_WIDTH / 2.0)
        .center(0.0, BASE_HEIGHT * 0.46)
        .rect(0.170, 0.022)
        .extrude(-0.018)
    )
    rear_recess = (
        cq.Workplane("XZ")
        .workplane(offset=-BASE_WIDTH / 2.0)
        .center(0.0, BASE_HEIGHT * 0.46)
        .rect(0.170, 0.022)
        .extrude(0.018)
    )

    return body.union(lower_housing).union(upper_housing).cut(front_recess).cut(rear_recess)


def _support_cheek(center_x: float) -> cq.Workplane:
    profile_points = [
        (-0.055, 0.0),
        (-0.055, 0.018),
        (-0.040, 0.056),
        (-0.030, 0.096),
        (-0.030, 0.128),
        (0.030, 0.128),
        (0.030, 0.096),
        (0.040, 0.056),
        (0.055, 0.018),
        (0.055, 0.0),
    ]

    cheek = (
        cq.Workplane("YZ", origin=(center_x, 0.0, PLATTER_THICKNESS - 0.004))
        .polyline(profile_points)
        .close()
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
    )
    boss = (
        cq.Workplane(
            "YZ",
            origin=(center_x, 0.0, PLATTER_THICKNESS + TRUNNION_AXIS_HEIGHT),
        )
        .circle(SUPPORT_BOSS_OUTER_RADIUS)
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
    )
    boss_hole = (
        cq.Workplane(
            "YZ",
            origin=(center_x, 0.0, PLATTER_THICKNESS + TRUNNION_AXIS_HEIGHT),
        )
        .circle(BORE_RADIUS)
        .extrude(SUPPORT_THICKNESS, both=True)
    )
    return cheek.union(boss).cut(boss_hole)


def _make_rotary_platter_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(PLATTER_RADIUS).extrude(PLATTER_THICKNESS)
    platter = platter.edges("%CIRCLE").fillet(0.004)
    platter = (
        platter.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(PLATTER_RECESS_RADIUS)
        .cutBlind(-PLATTER_RECESS_DEPTH)
    )

    left_support = _support_cheek(-SUPPORT_CENTER_X)
    right_support = _support_cheek(SUPPORT_CENTER_X)

    return platter.union(left_support).union(right_support)


def _make_work_table_top_shape() -> cq.Workplane:
    top = (
        cq.Workplane("XY")
        .box(TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, TABLE_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.007)
    )
    top = top.edges(">Z").fillet(0.003)
    return (
        top.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.050, 0.0), (0.0, 0.0), (0.050, 0.0)])
        .rect(TABLE_SLOT_WIDTH, TABLE_SLOT_LENGTH)
        .cutBlind(-TABLE_SLOT_DEPTH)
    )


def _make_trunnion_shaft_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_HALF_LENGTH, both=True)
    )
    return shaft


def _make_work_table_frame_shape() -> cq.Workplane:
    frame_block = (
        cq.Workplane("XY")
        .box(FRAME_BLOCK_LENGTH, FRAME_BLOCK_WIDTH, FRAME_BLOCK_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, -SHAFT_RADIUS))
    )
    column = (
        cq.Workplane("XY")
        .box(COLUMN_LENGTH, COLUMN_WIDTH, COLUMN_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, COLUMN_BASE_Z))
    )
    saddle = (
        cq.Workplane("XY")
        .box(SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, SADDLE_BASE_Z))
    )
    front_gusset = (
        cq.Workplane("XZ", origin=(0.0, GUSSET_Y_OFFSET, 0.006))
        .polyline(
            [
                (-0.054, 0.0),
                (-0.040, 0.020),
                (-0.026, SADDLE_BASE_Z + 0.010),
                (0.026, SADDLE_BASE_Z + 0.010),
                (0.040, 0.020),
                (0.054, 0.0),
            ]
        )
        .close()
        .extrude(GUSSET_HALF_THICKNESS, both=True)
    )
    rear_gusset = (
        cq.Workplane("XZ", origin=(0.0, -GUSSET_Y_OFFSET, 0.006))
        .polyline(
            [
                (-0.054, 0.0),
                (-0.040, 0.020),
                (-0.026, SADDLE_BASE_Z + 0.010),
                (0.026, SADDLE_BASE_Z + 0.010),
                (0.040, 0.020),
                (0.054, 0.0),
            ]
        )
        .close()
        .extrude(GUSSET_HALF_THICKNESS, both=True)
    )

    return frame_block.union(column).union(saddle).union(front_gusset).union(rear_gusset)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platter_and_trunnion_table")

    base_finish = model.material("base_finish", rgba=(0.28, 0.30, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    table_finish = model.material("table_finish", rgba=(0.64, 0.66, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_casting"),
        material=base_finish,
        name="base_casting",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_ROTARY_Z)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_ROTARY_Z / 2.0)),
    )

    rotary_platter = model.part("rotary_platter")
    rotary_platter.visual(
        mesh_from_cadquery(_make_rotary_platter_shape(), "rotary_platter_assembly"),
        material=machined_steel,
        name="rotary_platter_assembly",
    )
    rotary_platter.inertial = Inertial.from_geometry(
        Box((0.300, SUPPORT_DEPTH, 0.190)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    work_table = model.part("work_table")
    work_table.visual(
        mesh_from_cadquery(_make_work_table_top_shape(), "work_table_top"),
        material=table_finish,
        name="table_top",
    )
    work_table.visual(
        mesh_from_cadquery(_make_work_table_frame_shape(), "work_table_frame"),
        material=machined_steel,
        name="table_frame",
    )
    work_table.inertial = Inertial.from_geometry(
        Box((TABLE_LENGTH, TABLE_WIDTH, 0.090)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    trunnion_shaft = model.part("trunnion_shaft")
    trunnion_shaft.visual(
        mesh_from_cadquery(_make_trunnion_shaft_shape(), "work_table_trunnion_shaft"),
        material=machined_steel,
        name="trunnion_shaft",
    )
    trunnion_shaft.inertial = Inertial.from_geometry(
        Box((SHAFT_HALF_LENGTH * 2.0, SHAFT_RADIUS * 2.0, SHAFT_RADIUS * 2.0)),
        mass=0.7,
        origin=Origin(),
    )

    model.articulation(
        "base_to_rotary_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_platter,
        origin=Origin(xyz=(0.0, 0.0, BASE_ROTARY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=45.0,
            velocity=1.25,
        ),
    )
    model.articulation(
        "rotary_platter_to_work_table",
        ArticulationType.REVOLUTE,
        parent=rotary_platter,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_THICKNESS + TRUNNION_AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=1.25,
            effort=35.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "work_table_to_trunnion_shaft",
        ArticulationType.FIXED,
        parent=work_table,
        child=trunnion_shaft,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotary_platter = object_model.get_part("rotary_platter")
    work_table = object_model.get_part("work_table")
    trunnion_shaft = object_model.get_part("trunnion_shaft")
    platter_joint = object_model.get_articulation("base_to_rotary_platter")
    trunnion_joint = object_model.get_articulation("rotary_platter_to_work_table")
    table_top = work_table.get_visual("table_top")
    table_frame = work_table.get_visual("table_frame")
    shaft_visual = trunnion_shaft.get_visual("trunnion_shaft")

    ctx.allow_overlap(
        work_table,
        trunnion_shaft,
        reason="The trunnion shaft is intentionally press-fit through the work-table frame block.",
    )
    ctx.allow_overlap(
        rotary_platter,
        trunnion_shaft,
        reason="The trunnion shaft is intentionally captured inside the platter side-support bearing bores.",
    )

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

    ctx.check("base_present", base is not None, "Missing base part.")
    ctx.check("rotary_platter_present", rotary_platter is not None, "Missing rotary platter part.")
    ctx.check("work_table_present", work_table is not None, "Missing trunnion work table part.")
    ctx.check("trunnion_shaft_present", trunnion_shaft is not None, "Missing transverse trunnion shaft part.")

    ctx.check(
        "platter_joint_is_vertical_revolute",
        platter_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical revolute platter joint, got type={platter_joint.articulation_type}, axis={platter_joint.axis}.",
    )
    ctx.check(
        "trunnion_joint_is_horizontal_revolute",
        trunnion_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(trunnion_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected a horizontal X-axis trunnion joint, got type={trunnion_joint.articulation_type}, axis={trunnion_joint.axis}.",
    )

    with ctx.pose({platter_joint: 0.0, trunnion_joint: 0.0}):
        ctx.expect_contact(
            rotary_platter,
            base,
            name="rotary_platter_is_supported_by_base",
        )
        ctx.expect_contact(
            trunnion_shaft,
            rotary_platter,
            name="work_table_is_supported_by_trunnion_shaft",
        )
        ctx.expect_contact(
            work_table,
            trunnion_shaft,
            elem_b=shaft_visual,
            name="trunnion_shaft_is_captured_by_work_table_frame",
        )
        ctx.expect_gap(
            work_table,
            rotary_platter,
            axis="z",
            positive_elem=table_top,
            min_gap=0.018,
            name="tabletop_clears_rotary_platter_when_level",
        )
        ctx.expect_overlap(
            work_table,
            rotary_platter,
            axes="xy",
            elem_a=table_top,
            min_overlap=0.150,
            name="tabletop_is_centered_over_rotary_platter",
        )

    with ctx.pose({platter_joint: 0.80, trunnion_joint: 1.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_rotated_and_tilted_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
