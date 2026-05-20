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
BASE_WIDTH = 0.23
BASE_FLOOR_HEIGHT = 0.018
BASE_RAIL_LENGTH = 0.34
BASE_RAIL_WIDTH = 0.040
BASE_RAIL_HEIGHT = 0.016
BASE_RAIL_OFFSET_Y = 0.074
BASE_END_BLOCK_LENGTH = 0.045
BASE_COVER_WIDTH = 0.046
BASE_COVER_HEIGHT = 0.010

X_RUNNER_LENGTH = 0.14
X_RUNNER_WIDTH = 0.032
X_RUNNER_HEIGHT = 0.012
X_RUNNER_OFFSET_Y = BASE_RAIL_OFFSET_Y
X_PLATE_LENGTH = 0.17
X_PLATE_WIDTH = 0.20
X_PLATE_HEIGHT = 0.022
X_PLATE_Z0 = 0.010
Y_BASE_X = 0.14
Y_BASE_Y = 0.19
Y_BASE_HEIGHT = 0.016
Y_BASE_Z0 = 0.030
Y_RAIL_X = 0.020
Y_RAIL_Y = 0.155
Y_RAIL_HEIGHT = 0.012
Y_RAIL_Z0 = 0.044
Y_RAIL_OFFSET_X = 0.040

Y_RUNNER_X = 0.022
Y_RUNNER_Y = 0.10
Y_RUNNER_HEIGHT = 0.010
Y_RUNNER_OFFSET_X = Y_RAIL_OFFSET_X
Y_PLATE_X = 0.13
Y_PLATE_Y = 0.14
Y_PLATE_HEIGHT = 0.020
Y_PLATE_Z0 = 0.008
COLUMN_BASE_X = 0.044
COLUMN_BASE_Y = 0.110
COLUMN_BASE_HEIGHT = 0.020
COLUMN_BASE_Z0 = 0.026
COLUMN_X = 0.036
COLUMN_Y = 0.100
COLUMN_HEIGHT = 0.112
COLUMN_Z0 = 0.044
GUIDE_RIB_X = 0.008
GUIDE_RIB_Y = 0.020
GUIDE_RIB_HEIGHT = 0.104
GUIDE_RIB_OFFSET_Y = 0.032
GUIDE_RIB_CENTER_X = (COLUMN_X / 2.0) + (GUIDE_RIB_X / 2.0)

Z_BACKPLATE_X = 0.008
Z_BACKPLATE_Y = 0.104
Z_BACKPLATE_HEIGHT = 0.106
Z_BACKPLATE_START_X = -0.002
Z_CARRIAGE_X = 0.022
Z_CARRIAGE_Y = 0.024
Z_CARRIAGE_HEIGHT = 0.128
Z_STAGE_WEB_X = 0.020
Z_STAGE_WEB_Y = 0.056
Z_STAGE_WEB_HEIGHT = 0.094
Z_STAGE_WEB_START_X = 0.006
FACEPLATE_X = 0.010
FACEPLATE_Y = 0.140
FACEPLATE_HEIGHT = 0.132
FACEPLATE_START_X = 0.012
Z_CARRIAGE_OFFSET_Y = GUIDE_RIB_OFFSET_Y

X_TRAVEL = 0.075
Y_TRAVEL = 0.055
Z_TRAVEL = 0.050

X_STAGE_CONTACT_Z = BASE_FLOOR_HEIGHT + BASE_RAIL_HEIGHT
Y_STAGE_CONTACT_Z = Y_RAIL_Z0 + Y_RAIL_HEIGHT
Z_STAGE_GUIDE_X = 0.028
Z_STAGE_GUIDE_Z = COLUMN_Z0 + (GUIDE_RIB_HEIGHT / 2.0)


def _box_from_bottom(
    x_size: float,
    y_size: float,
    z_size: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        x_size,
        y_size,
        z_size,
        centered=(True, True, False),
    ).translate((x, y, z0))


def _box_from_back(
    x_size: float,
    y_size: float,
    z_size: float,
    *,
    x0: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        x_size,
        y_size,
        z_size,
        centered=(False, True, True),
    ).translate((x0, y, z))


def _base_shape() -> cq.Workplane:
    base = _box_from_bottom(BASE_LENGTH, BASE_WIDTH, BASE_FLOOR_HEIGHT)
    for y in (-BASE_RAIL_OFFSET_Y, BASE_RAIL_OFFSET_Y):
        base = base.union(
            _box_from_bottom(
                BASE_RAIL_LENGTH,
                BASE_RAIL_WIDTH,
                BASE_RAIL_HEIGHT,
                y=y,
                z0=BASE_FLOOR_HEIGHT,
            )
        )
    for x in (
        -(BASE_LENGTH / 2.0 - BASE_END_BLOCK_LENGTH / 2.0 - 0.01),
        BASE_LENGTH / 2.0 - BASE_END_BLOCK_LENGTH / 2.0 - 0.01,
    ):
        base = base.union(
            _box_from_bottom(
                BASE_END_BLOCK_LENGTH,
                BASE_WIDTH - 0.04,
                BASE_RAIL_HEIGHT,
                x=x,
                z0=BASE_FLOOR_HEIGHT,
            )
        )
    base = base.union(
        _box_from_bottom(
            BASE_RAIL_LENGTH - 0.04,
            BASE_COVER_WIDTH,
            BASE_COVER_HEIGHT,
            z0=BASE_FLOOR_HEIGHT,
        )
    )
    return base


def _x_stage_shape() -> cq.Workplane:
    stage = _box_from_bottom(
        X_PLATE_LENGTH,
        X_PLATE_WIDTH,
        X_PLATE_HEIGHT,
        z0=X_PLATE_Z0,
    )
    for y in (-X_RUNNER_OFFSET_Y, X_RUNNER_OFFSET_Y):
        stage = stage.union(
            _box_from_bottom(
                X_RUNNER_LENGTH,
                X_RUNNER_WIDTH,
                X_RUNNER_HEIGHT,
                y=y,
                z0=0.0,
            )
        )
    stage = stage.union(
        _box_from_bottom(
            Y_BASE_X,
            Y_BASE_Y,
            Y_BASE_HEIGHT,
            z0=Y_BASE_Z0,
        )
    )
    for x in (-Y_RAIL_OFFSET_X, Y_RAIL_OFFSET_X):
        stage = stage.union(
            _box_from_bottom(
                Y_RAIL_X,
                Y_RAIL_Y,
                Y_RAIL_HEIGHT,
                x=x,
                z0=Y_RAIL_Z0,
            )
        )
    return stage


def _y_stage_shape() -> cq.Workplane:
    stage = _box_from_bottom(
        Y_PLATE_X,
        Y_PLATE_Y,
        Y_PLATE_HEIGHT,
        z0=Y_PLATE_Z0,
    )
    for x in (-Y_RUNNER_OFFSET_X, Y_RUNNER_OFFSET_X):
        stage = stage.union(
            _box_from_bottom(
                Y_RUNNER_X,
                Y_RUNNER_Y,
                Y_RUNNER_HEIGHT,
                x=x,
                z0=0.0,
            )
        )
    stage = stage.union(
        _box_from_bottom(
            COLUMN_BASE_X,
            COLUMN_BASE_Y,
            COLUMN_BASE_HEIGHT,
            z0=COLUMN_BASE_Z0,
        )
    )
    stage = stage.union(
        _box_from_bottom(
            COLUMN_X,
            COLUMN_Y,
            COLUMN_HEIGHT,
            z0=COLUMN_Z0,
        )
    )
    for y in (-GUIDE_RIB_OFFSET_Y, GUIDE_RIB_OFFSET_Y):
        stage = stage.union(
            _box_from_bottom(
                GUIDE_RIB_X,
                GUIDE_RIB_Y,
                GUIDE_RIB_HEIGHT,
                x=GUIDE_RIB_CENTER_X,
                y=y,
                z0=COLUMN_Z0,
            )
        )
    return stage


def _z_stage_shape() -> cq.Workplane:
    stage = _box_from_back(
        Z_BACKPLATE_X,
        Z_BACKPLATE_Y,
        Z_BACKPLATE_HEIGHT,
        x0=Z_BACKPLATE_START_X,
        z=0.0,
    )
    for y in (-Z_CARRIAGE_OFFSET_Y, Z_CARRIAGE_OFFSET_Y):
        stage = stage.union(
            _box_from_back(
                Z_CARRIAGE_X,
                Z_CARRIAGE_Y,
                Z_CARRIAGE_HEIGHT,
                y=y,
                z=0.0,
            )
        )
    stage = stage.union(
        _box_from_back(
            Z_STAGE_WEB_X,
            Z_STAGE_WEB_Y,
            Z_STAGE_WEB_HEIGHT,
            x0=Z_STAGE_WEB_START_X,
            z=0.0,
        )
    )
    stage = stage.union(
        _box_from_back(
            Z_CARRIAGE_X,
            0.040,
            0.028,
            x0=0.0,
            z=0.0,
        )
    )
    stage = stage.union(
        _box_from_back(
            FACEPLATE_X,
            FACEPLATE_Y,
            FACEPLATE_HEIGHT,
            x0=FACEPLATE_START_X,
            z=0.0,
        )
    )
    return stage


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_xyz_module")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("faceplate_gray", rgba=(0.68, 0.70, 0.73, 1.0))

    base = model.part("base")
    x_stage = model.part("x_stage")
    y_stage = model.part("y_stage")
    z_stage = model.part("z_stage")

    _add_mesh_visual(base, _base_shape(), "base_body", "base_charcoal", "base_body")
    _add_mesh_visual(
        x_stage,
        _x_stage_shape(),
        "x_stage_body",
        "machined_aluminum",
        "x_stage_body",
    )
    _add_mesh_visual(
        y_stage,
        _y_stage_shape(),
        "y_stage_body",
        "machined_aluminum",
        "y_stage_body",
    )
    _add_mesh_visual(
        z_stage,
        _z_stage_shape(),
        "z_stage_body",
        "faceplate_gray",
        "z_stage_body",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_CONTACT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=220.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_CONTACT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=160.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(Z_STAGE_GUIDE_X, 0.0, Z_STAGE_GUIDE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=120.0,
            velocity=0.16,
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
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    z_stage = object_model.get_part("z_stage")
    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    y_to_z = object_model.get_articulation("y_to_z")

    ctx.expect_contact(
        x_stage,
        base,
        contact_tol=0.001,
        name="X carriage rides on the grounded lower slide",
    )
    ctx.expect_contact(
        y_stage,
        x_stage,
        contact_tol=0.001,
        name="Y cross-slide rides on the X saddle",
    )
    ctx.expect_overlap(
        x_stage,
        base,
        axes="y",
        min_overlap=0.12,
        name="X stage stays supported across the base width",
    )
    ctx.expect_overlap(
        y_stage,
        x_stage,
        axes="x",
        min_overlap=0.09,
        name="Y stage stays supported across the cross-slide body",
    )
    ctx.expect_overlap(
        z_stage,
        y_stage,
        axes="y",
        min_overlap=0.07,
        name="Z stage stays centered on the upright guide",
    )
    ctx.expect_origin_distance(
        z_stage,
        y_stage,
        axes="x",
        min_dist=0.02,
        max_dist=0.04,
        name="Z stage sits just forward of the upright guide",
    )

    x_rest = ctx.part_world_position(x_stage)
    with ctx.pose({base_to_x: X_TRAVEL}):
        x_extended = ctx.part_world_position(x_stage)
        ctx.expect_contact(
            x_stage,
            base,
            contact_tol=0.001,
            name="X carriage remains supported at +X travel",
        )
    ctx.check(
        "X axis extends along +X",
        x_rest is not None
        and x_extended is not None
        and x_extended[0] > x_rest[0] + 0.05,
        details=f"rest={x_rest}, extended={x_extended}",
    )

    y_rest = ctx.part_world_position(y_stage)
    with ctx.pose({x_to_y: Y_TRAVEL}):
        y_extended = ctx.part_world_position(y_stage)
        ctx.expect_contact(
            y_stage,
            x_stage,
            contact_tol=0.001,
            name="Y carriage remains supported at +Y travel",
        )
    ctx.check(
        "Y axis extends along +Y",
        y_rest is not None
        and y_extended is not None
        and y_extended[1] > y_rest[1] + 0.03,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    z_rest = ctx.part_world_position(z_stage)
    with ctx.pose({y_to_z: Z_TRAVEL}):
        z_extended = ctx.part_world_position(z_stage)
        ctx.expect_overlap(
            z_stage,
            y_stage,
            axes="y",
            min_overlap=0.07,
            name="Z carriage remains laterally aligned at full lift",
        )
        ctx.expect_origin_distance(
            z_stage,
            y_stage,
            axes="x",
            min_dist=0.02,
            max_dist=0.04,
            name="Z carriage stays just forward of the guide at full lift",
        )
    ctx.check(
        "Z axis extends upward",
        z_rest is not None
        and z_extended is not None
        and z_extended[2] > z_rest[2] + 0.03,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
