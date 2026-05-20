from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.00
BASE_WIDTH = 0.46
BASE_HEIGHT = 0.10
BASE_RECESS_DEPTH = 0.028
X_RAIL_LENGTH = 0.88
X_RAIL_WIDTH = 0.05
X_RAIL_HEIGHT = 0.022
X_RAIL_Y = 0.15

X_BLOCK_LENGTH = 0.09
X_BLOCK_WIDTH = 0.07
X_BLOCK_HEIGHT = 0.026
X_BLOCK_X = 0.11
X_BLOCK_Y = X_RAIL_Y
X_TOP_LENGTH = 0.38
X_TOP_WIDTH = 0.42
X_TOP_THICKNESS = 0.038
Y_RAIL_LENGTH = 0.34
Y_RAIL_WIDTH = 0.04
Y_RAIL_HEIGHT = 0.018
Y_RAIL_X = 0.11

Y_BLOCK_WIDTH = 0.08
Y_BLOCK_LENGTH = 0.07
Y_BLOCK_HEIGHT = 0.024
Y_BLOCK_X = Y_RAIL_X
Y_BLOCK_Y = 0.05
Y_TABLE_WIDTH = 0.32
Y_TABLE_LENGTH = 0.28
Y_TABLE_THICKNESS = 0.040
COLUMN_WIDTH = 0.22
COLUMN_DEPTH = 0.12
COLUMN_HEIGHT = 0.66
COLUMN_CENTER_Y = -0.08
Z_RAIL_WIDTH = 0.028
Z_RAIL_DEPTH = 0.022
Z_RAIL_HEIGHT = 0.54
Z_RAIL_X = 0.065
Z_RAIL_BOTTOM_Z = 0.09

Z_STAGE_WIDTH = 0.24
Z_STAGE_HEIGHT = 0.30
Z_STAGE_BODY_DEPTH = 0.040
Z_STAGE_FRONT_WIDTH = 0.16
Z_STAGE_FRONT_HEIGHT = 0.20
Z_STAGE_FRONT_DEPTH = 0.034
Z_STAGE_NOSE_WIDTH = 0.07
Z_STAGE_NOSE_HEIGHT = 0.10
Z_STAGE_NOSE_DEPTH = 0.022
Z_BLOCK_WIDTH = 0.055
Z_BLOCK_HEIGHT = 0.060
Z_BLOCK_DEPTH = 0.026
Z_BLOCK_Z = 0.09

X_TRAVEL = 0.20
Y_TRAVEL = 0.07
Z_TRAVEL = 0.18


def _box(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False))


def _make_base_shape() -> cq.Workplane:
    bed = _box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
    bed = bed.cut(
        _box(BASE_LENGTH - 0.18, BASE_WIDTH - 0.18, BASE_RECESS_DEPTH).translate((0.0, 0.0, 0.0))
    )
    bed = bed.edges("|Z").fillet(0.010)

    rail = _box(X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT).edges("|Z").fillet(0.003)
    left_rail = rail.translate((0.0, -X_RAIL_Y, BASE_HEIGHT))
    right_rail = rail.translate((0.0, X_RAIL_Y, BASE_HEIGHT))

    return bed.union(left_rail).union(right_rail)


def _make_x_carriage_shape() -> cq.Workplane:
    top = _box(X_TOP_LENGTH, X_TOP_WIDTH, X_TOP_THICKNESS).translate((0.0, 0.0, X_BLOCK_HEIGHT))
    top = top.edges("|Z").fillet(0.005)

    block = _box(X_BLOCK_LENGTH, X_BLOCK_WIDTH, X_BLOCK_HEIGHT).edges("|Z").fillet(0.002)
    body = top
    for x_pos in (-X_BLOCK_X, X_BLOCK_X):
        for y_pos in (-X_BLOCK_Y, X_BLOCK_Y):
            body = body.union(block.translate((x_pos, y_pos, 0.0)))

    rail = _box(Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT).edges("|Z").fillet(0.002)
    left_rail = rail.translate((-Y_RAIL_X, 0.0, X_BLOCK_HEIGHT + X_TOP_THICKNESS))
    right_rail = rail.translate((Y_RAIL_X, 0.0, X_BLOCK_HEIGHT + X_TOP_THICKNESS))

    return body.union(left_rail).union(right_rail)


def _make_y_slide_shape() -> cq.Workplane:
    table = _box(Y_TABLE_WIDTH, Y_TABLE_LENGTH, Y_TABLE_THICKNESS).translate((0.0, 0.0, Y_BLOCK_HEIGHT))
    table = table.edges("|Z").fillet(0.005)

    block = _box(Y_BLOCK_WIDTH, Y_BLOCK_LENGTH, Y_BLOCK_HEIGHT).edges("|Z").fillet(0.002)
    body = table
    for x_pos in (-Y_BLOCK_X, Y_BLOCK_X):
        for y_pos in (-Y_BLOCK_Y, Y_BLOCK_Y):
            body = body.union(block.translate((x_pos, y_pos, 0.0)))

    column = _box(COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT).translate(
        (0.0, COLUMN_CENTER_Y, Y_BLOCK_HEIGHT + Y_TABLE_THICKNESS)
    )
    column = column.edges("|Z").fillet(0.004)
    body = body.union(column)

    column_front_y = COLUMN_CENTER_Y + COLUMN_DEPTH / 2.0
    rail = _box(Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT).edges("|Z").fillet(0.0015)
    left_rail = rail.translate(
        (-Z_RAIL_X, column_front_y + Z_RAIL_DEPTH / 2.0, Z_RAIL_BOTTOM_Z)
    )
    right_rail = rail.translate(
        (Z_RAIL_X, column_front_y + Z_RAIL_DEPTH / 2.0, Z_RAIL_BOTTOM_Z)
    )

    return body.union(left_rail).union(right_rail)


def _make_z_stage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        Z_STAGE_WIDTH,
        Z_STAGE_BODY_DEPTH,
        Z_STAGE_HEIGHT,
        centered=(True, True, True),
    ).translate((0.0, Z_BLOCK_DEPTH + Z_STAGE_BODY_DEPTH / 2.0, 0.0))
    carriage = carriage.edges("|Y").fillet(0.004)

    front_plate = cq.Workplane("XY").box(
        Z_STAGE_FRONT_WIDTH,
        Z_STAGE_FRONT_DEPTH,
        Z_STAGE_FRONT_HEIGHT,
        centered=(True, True, True),
    ).translate((0.0, Z_BLOCK_DEPTH + Z_STAGE_BODY_DEPTH + Z_STAGE_FRONT_DEPTH / 2.0, 0.0))
    front_plate = front_plate.edges("|Y").fillet(0.003)

    nose = cq.Workplane("XY").box(
        Z_STAGE_NOSE_WIDTH,
        Z_STAGE_NOSE_DEPTH,
        Z_STAGE_NOSE_HEIGHT,
        centered=(True, True, True),
    ).translate(
        (
            0.0,
            Z_BLOCK_DEPTH
            + Z_STAGE_BODY_DEPTH
            + Z_STAGE_FRONT_DEPTH
            + Z_STAGE_NOSE_DEPTH / 2.0,
            0.0,
        )
    )
    nose = nose.edges("|Y").fillet(0.002)

    block = cq.Workplane("XY").box(
        Z_BLOCK_WIDTH,
        Z_BLOCK_DEPTH,
        Z_BLOCK_HEIGHT,
        centered=(True, True, True),
    )
    body = carriage.union(front_plate).union(nose)
    for x_pos in (-Z_RAIL_X, Z_RAIL_X):
        for z_pos in (-Z_BLOCK_Z, Z_BLOCK_Z):
            body = body.union(block.translate((x_pos, Z_BLOCK_DEPTH / 2.0, z_pos)))

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_table_xyz_slide_stack")

    model.material("base_cast", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("stage_blue", rgba=(0.34, 0.44, 0.68, 1.0))
    model.material("head_steel", rgba=(0.67, 0.70, 0.73, 1.0))

    x_base = model.part("x_base")
    x_base.visual(
        mesh_from_cadquery(_make_base_shape(), "x_base"),
        material="base_cast",
        name="x_base_body",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_make_x_carriage_shape(), "x_carriage"),
        material="machined_aluminum",
        name="x_carriage_body",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        Box((Y_TABLE_WIDTH, Y_TABLE_LENGTH, Y_TABLE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, Y_BLOCK_HEIGHT + Y_TABLE_THICKNESS / 2.0)),
        material="stage_blue",
        name="y_table",
    )
    for x_pos in (-Y_BLOCK_X, Y_BLOCK_X):
        for y_pos in (-Y_BLOCK_Y, Y_BLOCK_Y):
            y_slide.visual(
                Box((Y_BLOCK_WIDTH, Y_BLOCK_LENGTH, Y_BLOCK_HEIGHT)),
                origin=Origin(xyz=(x_pos, y_pos, Y_BLOCK_HEIGHT / 2.0)),
                material="machined_aluminum",
                name=f"y_block_{'l' if x_pos < 0.0 else 'r'}_{'b' if y_pos < 0.0 else 'f'}",
            )
    y_slide.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_CENTER_Y,
                Y_BLOCK_HEIGHT + Y_TABLE_THICKNESS + COLUMN_HEIGHT / 2.0,
            )
        ),
        material="stage_blue",
        name="z_column",
    )
    column_front_y = COLUMN_CENTER_Y + COLUMN_DEPTH / 2.0
    for x_pos, name in ((-Z_RAIL_X, "left_z_rail"), (Z_RAIL_X, "right_z_rail")):
        y_slide.visual(
            Box((Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_pos,
                    column_front_y + Z_RAIL_DEPTH / 2.0,
                    Z_RAIL_BOTTOM_Z + Z_RAIL_HEIGHT / 2.0,
                )
            ),
            material="machined_aluminum",
            name=name,
        )

    z_stage = model.part("z_stage")
    z_stage.visual(
        Box((0.18, 0.028, 0.28)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material="head_steel",
        name="z_carriage",
    )
    z_stage.visual(
        Box((Z_STAGE_FRONT_WIDTH, 0.032, Z_STAGE_FRONT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        material="head_steel",
        name="z_front_plate",
    )
    z_stage.visual(
        Box((Z_STAGE_NOSE_WIDTH, Z_STAGE_NOSE_DEPTH, Z_STAGE_NOSE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.071, 0.0)),
        material="head_steel",
        name="z_nose",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=x_base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + X_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=450.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, X_BLOCK_HEIGHT + X_TOP_THICKNESS + Y_RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=320.0,
            velocity=0.24,
        ),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=z_stage,
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_CENTER_Y + COLUMN_DEPTH / 2.0 + Z_RAIL_DEPTH,
                0.31,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=260.0,
            velocity=0.18,
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
    x_base = object_model.get_part("x_base")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    z_stage = object_model.get_part("z_stage")
    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")
    left_z_rail = y_slide.get_visual("left_z_rail")
    right_z_rail = y_slide.get_visual("right_z_rail")
    z_carriage = z_stage.get_visual("z_carriage")

    ctx.expect_contact(x_base, x_carriage, name="x carriage sits on the lower X guide")
    ctx.expect_contact(x_carriage, y_slide, name="y slide sits on the carriage-mounted guide")
    ctx.expect_within(
        x_carriage,
        x_base,
        axes="y",
        margin=0.0,
        name="x carriage stays within the base width",
    )
    ctx.expect_within(
        y_slide,
        x_carriage,
        axes="x",
        margin=0.0,
        name="y slide stays centered over the carriage",
    )
    ctx.expect_within(
        z_stage,
        y_slide,
        axes="x",
        margin=0.0,
        name="z stage stays centered on the column",
    )
    ctx.expect_overlap(
        z_stage,
        y_slide,
        axes="z",
        min_overlap=0.25,
        name="z stage remains engaged with the column height",
    )
    ctx.expect_contact(
        z_stage,
        y_slide,
        elem_a=z_carriage,
        elem_b=left_z_rail,
        name="z carriage bears on the left rail",
    )
    ctx.expect_contact(
        z_stage,
        y_slide,
        elem_a=z_carriage,
        elem_b=right_z_rail,
        name="z carriage bears on the right rail",
    )

    x_rest = ctx.part_world_position(x_carriage)
    with ctx.pose({x_axis: X_TRAVEL}):
        x_extended = ctx.part_world_position(x_carriage)
        ctx.expect_contact(x_base, x_carriage, name="x carriage remains supported at positive travel")
    ctx.check(
        "x carriage moves along +X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.15,
        details=f"rest={x_rest}, extended={x_extended}",
    )

    y_rest = ctx.part_world_position(y_slide)
    with ctx.pose({y_axis: Y_TRAVEL}):
        y_extended = ctx.part_world_position(y_slide)
        ctx.expect_contact(
            x_carriage,
            y_slide,
            name="y slide remains supported at positive travel",
        )
    ctx.check(
        "y slide moves along +Y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.05,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    z_rest = ctx.part_world_position(z_stage)
    with ctx.pose({z_axis: Z_TRAVEL}):
        z_extended = ctx.part_world_position(z_stage)
        ctx.expect_overlap(
            z_stage,
            y_slide,
            axes="z",
            min_overlap=0.12,
            name="z stage remains engaged at positive travel",
        )
        ctx.expect_overlap(
            z_stage,
            y_slide,
            axes="z",
            elem_a=z_carriage,
            elem_b=left_z_rail,
            min_overlap=0.08,
            name="left rail keeps the z carriage captured at positive travel",
        )
    ctx.check(
        "z stage moves upward along +Z",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.12,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
