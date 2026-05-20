from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.26
BASE_THICKNESS = 0.024
X_RAIL_LENGTH = 0.34
X_RAIL_WIDTH = 0.034
X_RAIL_HEIGHT = 0.016
X_RAIL_OFFSET_Y = 0.086
BASE_SPINE_LENGTH = 0.30
BASE_SPINE_WIDTH = 0.044
BASE_SPINE_HEIGHT = 0.010

X_SADDLE_LENGTH = 0.24
X_SADDLE_WIDTH = 0.22
X_SADDLE_BODY_HEIGHT = 0.040
X_SADDLE_POCKET_LENGTH = 0.18
X_SADDLE_POCKET_WIDTH = 0.10
X_SADDLE_POCKET_HEIGHT = 0.028
Y_RAIL_LENGTH = 0.16
Y_RAIL_WIDTH = 0.032
Y_RAIL_HEIGHT = 0.012
Y_RAIL_OFFSET_X = 0.070

Y_CARRIAGE_WIDTH_X = 0.20
Y_CARRIAGE_LENGTH_Y = 0.18
Y_CARRIAGE_BASE_HEIGHT = 0.036
Y_CARRIAGE_POCKET_X = 0.11
Y_CARRIAGE_POCKET_Y = 0.14
Y_CARRIAGE_POCKET_HEIGHT = 0.024
Z_COLUMN_WIDTH = 0.14
Z_COLUMN_DEPTH = 0.050
Z_COLUMN_HEIGHT = 0.220
Z_COLUMN_CENTER_Y = 0.045
Z_GUSSET_WIDTH = 0.030
Z_GUSSET_DEPTH = 0.030
Z_GUSSET_HEIGHT = 0.095
Z_GUSSET_CENTER_Y = 0.040
Z_GUSSET_OFFSET_X = 0.048

Z_SLIDE_WIDTH = 0.12
Z_SLIDE_DEPTH = 0.045
Z_SLIDE_BODY_HEIGHT = 0.160
TOP_TABLE_DEPTH = 0.100
TOP_TABLE_THICKNESS = 0.016
TOP_TABLE_OFFSET_Y = 0.0

X_TRAVEL = 0.080
Y_TRAVEL = 0.050
Z_TRAVEL = 0.120


def _box(
    length: float,
    width: float,
    height: float,
    *,
    centered: tuple[bool, bool, bool] = (True, True, False),
    fillet_radius: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, width, height, centered=centered)
    if fillet_radius > 0.0:
        shape = shape.edges("|Z").fillet(fillet_radius)
    return shape


def _x_base_shape() -> cq.Workplane:
    body = _box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, fillet_radius=0.010)

    left_rail = _box(X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT, fillet_radius=0.003).translate(
        (0.0, X_RAIL_OFFSET_Y, BASE_THICKNESS)
    )
    right_rail = _box(
        X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT, fillet_radius=0.003
    ).translate((0.0, -X_RAIL_OFFSET_Y, BASE_THICKNESS))
    spine = _box(BASE_SPINE_LENGTH, BASE_SPINE_WIDTH, BASE_SPINE_HEIGHT, fillet_radius=0.003).translate(
        (0.0, 0.0, BASE_THICKNESS)
    )

    return body.union(left_rail).union(right_rail).union(spine)


def _x_saddle_shape() -> cq.Workplane:
    saddle = _box(
        X_SADDLE_LENGTH,
        X_SADDLE_WIDTH,
        X_SADDLE_BODY_HEIGHT,
        fillet_radius=0.007,
    )
    pocket = _box(
        X_SADDLE_POCKET_LENGTH,
        X_SADDLE_POCKET_WIDTH,
        X_SADDLE_POCKET_HEIGHT,
    )
    saddle = saddle.cut(pocket)

    left_y_rail = _box(Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT, fillet_radius=0.0025).translate(
        (Y_RAIL_OFFSET_X, 0.0, X_SADDLE_BODY_HEIGHT)
    )
    right_y_rail = _box(
        Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT, fillet_radius=0.0025
    ).translate((-Y_RAIL_OFFSET_X, 0.0, X_SADDLE_BODY_HEIGHT))

    return saddle.union(left_y_rail).union(right_y_rail)


def _y_carriage_shape() -> cq.Workplane:
    base = _box(
        Y_CARRIAGE_WIDTH_X,
        Y_CARRIAGE_LENGTH_Y,
        Y_CARRIAGE_BASE_HEIGHT,
        fillet_radius=0.006,
    )
    pocket = _box(
        Y_CARRIAGE_POCKET_X,
        Y_CARRIAGE_POCKET_Y,
        Y_CARRIAGE_POCKET_HEIGHT,
    )
    base = base.cut(pocket)

    column = _box(
        Z_COLUMN_WIDTH,
        Z_COLUMN_DEPTH,
        Z_COLUMN_HEIGHT,
        fillet_radius=0.005,
    ).translate((0.0, Z_COLUMN_CENTER_Y, Y_CARRIAGE_BASE_HEIGHT))

    left_gusset = _box(
        Z_GUSSET_WIDTH,
        Z_GUSSET_DEPTH,
        Z_GUSSET_HEIGHT,
        fillet_radius=0.003,
    ).translate((Z_GUSSET_OFFSET_X, Z_GUSSET_CENTER_Y, Y_CARRIAGE_BASE_HEIGHT))
    right_gusset = _box(
        Z_GUSSET_WIDTH,
        Z_GUSSET_DEPTH,
        Z_GUSSET_HEIGHT,
        fillet_radius=0.003,
    ).translate((-Z_GUSSET_OFFSET_X, Z_GUSSET_CENTER_Y, Y_CARRIAGE_BASE_HEIGHT))

    return base.union(column).union(left_gusset).union(right_gusset)


def _z_slide_shape() -> cq.Workplane:
    body = _box(
        Z_SLIDE_WIDTH,
        Z_SLIDE_DEPTH,
        Z_SLIDE_BODY_HEIGHT,
        centered=(True, False, False),
        fillet_radius=0.004,
    ).translate((0.0, -Z_SLIDE_DEPTH, 0.0))

    table = _box(
        Z_SLIDE_WIDTH,
        TOP_TABLE_DEPTH,
        TOP_TABLE_THICKNESS,
        centered=(True, False, False),
        fillet_radius=0.004,
    ).translate((0.0, -TOP_TABLE_DEPTH + TOP_TABLE_OFFSET_Y, Z_SLIDE_BODY_HEIGHT))
    table = (
        table.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.030, 0.050), (0.030, 0.050)])
        .slot2D(0.030, 0.008, angle=90)
        .cutBlind(-TOP_TABLE_THICKNESS * 0.70)
    )

    return body.union(table)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_xyz_positioning_stage")

    model.material("base_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("stage_aluminum", rgba=(0.73, 0.76, 0.79, 1.0))
    model.material("carriage_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))

    x_base = model.part("x_base")
    x_base.visual(
        mesh_from_cadquery(_x_base_shape(), "x_base_body"),
        material="base_gray",
        name="x_base_body",
    )

    x_saddle = model.part("x_saddle")
    x_saddle.visual(
        mesh_from_cadquery(_x_saddle_shape(), "x_saddle_body"),
        material="stage_aluminum",
        name="x_saddle_body",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_y_carriage_shape(), "y_carriage_body"),
        material="carriage_aluminum",
        name="y_carriage_body",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        mesh_from_cadquery(_z_slide_shape(), "z_slide_body"),
        material="stage_aluminum",
        name="z_slide_body",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=x_base,
        child=x_saddle,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + X_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=180.0,
            velocity=0.18,
        ),
    )

    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_SADDLE_BODY_HEIGHT + Y_RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=120.0,
            velocity=0.16,
        ),
    )

    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, Z_COLUMN_CENTER_Y - Z_COLUMN_DEPTH / 2.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=140.0,
            velocity=0.14,
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
    x_saddle = object_model.get_part("x_saddle")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")

    x_axis = object_model.get_articulation("x_axis")
    y_axis = object_model.get_articulation("y_axis")
    z_axis = object_model.get_articulation("z_axis")

    ctx.check("x axis is prismatic", x_axis.joint_type == ArticulationType.PRISMATIC)
    ctx.check("y axis is prismatic", y_axis.joint_type == ArticulationType.PRISMATIC)
    ctx.check("z axis is prismatic", z_axis.joint_type == ArticulationType.PRISMATIC)
    ctx.check("x axis points along +X", tuple(x_axis.axis) == (1.0, 0.0, 0.0), details=str(x_axis.axis))
    ctx.check("y axis points along +Y", tuple(y_axis.axis) == (0.0, 1.0, 0.0), details=str(y_axis.axis))
    ctx.check("z axis points along +Z", tuple(z_axis.axis) == (0.0, 0.0, 1.0), details=str(z_axis.axis))

    ctx.expect_contact(x_saddle, x_base, name="x saddle sits on the grounded X base rails")
    ctx.expect_contact(y_carriage, x_saddle, name="y carriage sits on the X saddle rails")
    ctx.expect_contact(z_slide, y_carriage, name="z slide bears against the carriage column")

    ctx.expect_within(
        x_saddle,
        x_base,
        axes="y",
        margin=0.0,
        name="x saddle stays centered laterally on the base",
    )
    ctx.expect_overlap(
        x_saddle,
        x_base,
        axes="x",
        min_overlap=0.22,
        name="x saddle remains inserted on the base at rest",
    )

    ctx.expect_within(
        y_carriage,
        x_saddle,
        axes="x",
        margin=0.0,
        name="y carriage stays centered across the X saddle",
    )
    ctx.expect_overlap(
        y_carriage,
        x_saddle,
        axes="y",
        min_overlap=0.16,
        name="y carriage remains inserted on the saddle at rest",
    )

    ctx.expect_within(
        z_slide,
        y_carriage,
        axes="x",
        margin=0.0,
        name="z slide stays centered on the carriage column",
    )
    ctx.expect_overlap(
        z_slide,
        y_carriage,
        axes="z",
        min_overlap=0.17,
        name="lowered z slide retains strong column engagement",
    )

    x_rest = ctx.part_world_position(x_saddle)
    with ctx.pose({x_axis: X_TRAVEL}):
        ctx.expect_within(
            x_saddle,
            x_base,
            axes="y",
            margin=0.0,
            name="x saddle stays laterally centered at full X travel",
        )
        ctx.expect_overlap(
            x_saddle,
            x_base,
            axes="x",
            min_overlap=0.14,
            name="x saddle still overlaps the base at full X travel",
        )
        x_extended = ctx.part_world_position(x_saddle)

    y_rest = ctx.part_world_position(y_carriage)
    with ctx.pose({y_axis: Y_TRAVEL}):
        ctx.expect_within(
            y_carriage,
            x_saddle,
            axes="x",
            margin=0.0,
            name="y carriage stays centered across the saddle at full Y travel",
        )
        ctx.expect_overlap(
            y_carriage,
            x_saddle,
            axes="y",
            min_overlap=0.09,
            name="y carriage still overlaps the saddle at full Y travel",
        )
        y_extended = ctx.part_world_position(y_carriage)

    z_rest = ctx.part_world_position(z_slide)
    with ctx.pose({z_axis: Z_TRAVEL}):
        ctx.expect_contact(
            z_slide,
            y_carriage,
            name="z slide remains guided by the column when raised",
        )
        ctx.expect_within(
            z_slide,
            y_carriage,
            axes="x",
            margin=0.0,
            name="raised z slide stays centered on the column",
        )
        ctx.expect_overlap(
            z_slide,
            y_carriage,
            axes="z",
            min_overlap=0.085,
            name="raised z slide still retains column engagement",
        )
        z_extended = ctx.part_world_position(z_slide)

    ctx.check(
        "positive x joint moves the saddle along +X",
        x_rest is not None
        and x_extended is not None
        and x_extended[0] > x_rest[0] + 0.05
        and abs(x_extended[1] - x_rest[1]) < 1e-6,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "positive y joint moves the carriage along +Y",
        y_rest is not None
        and y_extended is not None
        and y_extended[1] > y_rest[1] + 0.03
        and abs(y_extended[0] - y_rest[0]) < 1e-6,
        details=f"rest={y_rest}, extended={y_extended}",
    )
    ctx.check(
        "positive z joint raises the slide along +Z",
        z_rest is not None
        and z_extended is not None
        and z_extended[2] > z_rest[2] + 0.08,
        details=f"rest={z_rest}, raised={z_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
