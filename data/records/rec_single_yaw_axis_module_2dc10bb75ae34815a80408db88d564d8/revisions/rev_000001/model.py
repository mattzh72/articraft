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


BASE_LENGTH = 0.220
BASE_WIDTH = 0.180
BASE_THICKNESS = 0.022
BASE_CORNER_RADIUS = 0.012
BASE_MOUNT_HOLE_DIAMETER = 0.010
BASE_MOUNT_SPACING_X = 0.150
BASE_MOUNT_SPACING_Y = 0.110

COLUMN_RADIUS = 0.032
COLUMN_HEIGHT = 0.070

COLLAR_RADIUS = 0.038
COLLAR_HEIGHT = 0.010

PIVOT_PIN_RADIUS = 0.010
PIVOT_PIN_HEIGHT = 0.006

TABLE_RADIUS = 0.090
TABLE_THICKNESS = 0.018
TABLE_EDGE_CHAMFER = 0.003
TABLE_SKIRT_OUTER_RADIUS = 0.052
TABLE_SKIRT_INNER_RADIUS = 0.041
TABLE_SKIRT_DEPTH = 0.012
TABLE_POCKET_RADIUS = 0.013
TABLE_POCKET_DEPTH = 0.008

GRIP_PEG_RADIUS = 0.008
GRIP_PEG_HEIGHT = 0.022
GRIP_PEG_OFFSET = 0.056

SUPPORT_PLANE_Z = BASE_THICKNESS + COLUMN_HEIGHT + COLLAR_HEIGHT
BASE_TOTAL_HEIGHT = SUPPORT_PLANE_Z + PIVOT_PIN_HEIGHT


def _build_base_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_THICKNESS)
        .edges("|Z")
        .fillet(BASE_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-BASE_MOUNT_SPACING_X * 0.5, -BASE_MOUNT_SPACING_Y * 0.5),
                (-BASE_MOUNT_SPACING_X * 0.5, BASE_MOUNT_SPACING_Y * 0.5),
                (BASE_MOUNT_SPACING_X * 0.5, -BASE_MOUNT_SPACING_Y * 0.5),
                (BASE_MOUNT_SPACING_X * 0.5, BASE_MOUNT_SPACING_Y * 0.5),
            ]
        )
        .hole(BASE_MOUNT_HOLE_DIAMETER)
    )


def _build_table_body() -> cq.Workplane:
    disk = (
        cq.Workplane("XY")
        .circle(TABLE_RADIUS)
        .extrude(TABLE_THICKNESS)
        .edges("%CIRCLE")
        .chamfer(TABLE_EDGE_CHAMFER)
    )
    pocket = cq.Workplane("XY").circle(TABLE_POCKET_RADIUS).extrude(TABLE_POCKET_DEPTH)
    return disk.cut(pocket)


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def _aabb_size(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple(high[i] - low[i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_column_yaw_fixture")

    model.material("powder_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machined_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("satin_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_plate(), "fixture_base_plate"),
        material="powder_gray",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT * 0.5)),
        material="machined_steel",
        name="column_shell",
    )
    base.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + COLLAR_HEIGHT * 0.5)),
        material="machined_steel",
        name="top_collar",
    )
    base.visual(
        Cylinder(radius=PIVOT_PIN_RADIUS, length=PIVOT_PIN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_PLANE_Z + PIVOT_PIN_HEIGHT * 0.5)),
        material="black_oxide",
        name="pivot_pin",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_TOTAL_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT * 0.5)),
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_build_table_body(), "fixture_rotary_table"),
        material="satin_aluminum",
        name="table_body",
    )
    table.visual(
        Cylinder(radius=GRIP_PEG_RADIUS, length=GRIP_PEG_HEIGHT),
        origin=Origin(xyz=(GRIP_PEG_OFFSET, 0.0, TABLE_THICKNESS + GRIP_PEG_HEIGHT * 0.5)),
        material="black_oxide",
        name="grip_peg",
    )
    table.inertial = Inertial.from_geometry(
        Cylinder(radius=TABLE_RADIUS, length=TABLE_THICKNESS),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, TABLE_THICKNESS * 0.5)),
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=8.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    table = object_model.get_part("table")
    yaw = object_model.get_articulation("base_to_table")

    limits = yaw.motion_limits
    ctx.check(
        "fixture uses a vertical revolute yaw joint",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -3.0
        and limits.upper >= 3.0,
        details=f"type={yaw.articulation_type}, axis={yaw.axis}, limits={limits}",
    )

    ctx.expect_origin_distance(
        table,
        base,
        axes="xy",
        max_dist=0.001,
        name="table stays centered on the column axis",
    )

    ctx.expect_contact(
        table,
        base,
        elem_a="table_body",
        elem_b="top_collar",
        contact_tol=1e-4,
        name="table body seats on the top collar",
    )

    column_size = _aabb_size(ctx.part_element_world_aabb(base, elem="column_shell"))
    table_size = _aabb_size(ctx.part_element_world_aabb(table, elem="table_body"))
    table_diameter = None if table_size is None else min(table_size[0], table_size[1])
    column_diameter = None if column_size is None else max(column_size[0], column_size[1])
    ctx.check(
        "table diameter is clearly larger than the stub column",
        table_diameter is not None
        and column_diameter is not None
        and table_diameter > 2.2 * column_diameter,
        details=f"table_diameter={table_diameter}, column_diameter={column_diameter}",
    )

    peg_rest = _aabb_center(ctx.part_element_world_aabb(table, elem="grip_peg"))
    with ctx.pose({yaw: pi / 2.0}):
        peg_turned = _aabb_center(ctx.part_element_world_aabb(table, elem="grip_peg"))
    ctx.check(
        "positive yaw rotates the grip peg toward +Y",
        peg_rest is not None
        and peg_turned is not None
        and peg_rest[0] > 0.045
        and abs(peg_rest[1]) < 0.010
        and peg_turned[1] > 0.045
        and abs(peg_turned[0]) < 0.010,
        details=f"rest={peg_rest}, turned={peg_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
