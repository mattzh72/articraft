from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.16
BASE_THICK = 0.012
BASE_HOLE_RADIUS = 0.006
LOWER_RAIL_LENGTH = 0.28
LOWER_RAIL_WIDTH = 0.022
LOWER_RAIL_HEIGHT = 0.014
LOWER_RAIL_OFFSET = 0.043

X_CARRIAGE_LENGTH = 0.128
X_CARRIAGE_WIDTH = 0.132
X_SHOE_WIDTH = 0.034
X_SHOE_HEIGHT = 0.020
X_SHOE_CENTER_Z = -0.002
X_BRIDGE_BOTTOM = 0.004
X_BRIDGE_THICK = 0.014
X_BRIDGE_TOP = X_BRIDGE_BOTTOM + X_BRIDGE_THICK
LOWER_CHANNEL_WIDTH = 0.025
LOWER_CHANNEL_HEIGHT = 0.013
LOWER_CHANNEL_CENTER_Z = -0.0055
X_TRAVEL = 0.085

UPPER_RAIL_X_OFFSET = 0.022
UPPER_RAIL_LENGTH = 0.11
UPPER_RAIL_WIDTH = 0.016
UPPER_RAIL_HEIGHT = 0.008
UPPER_RAIL_TOP = X_BRIDGE_TOP + UPPER_RAIL_HEIGHT

Y_STAGE_LENGTH = 0.096
Y_SHOE_WIDTH = 0.03
Y_SHOE_HEIGHT = 0.014
Y_SHOE_CENTER_Z = -0.001
Y_CHANNEL_WIDTH = 0.019
Y_CHANNEL_HEIGHT = 0.009
Y_CHANNEL_CENTER_Z = -0.0035
Y_SADDLE_WIDTH = 0.074
Y_SADDLE_THICK = 0.012
Y_BOSS_WIDTH = 0.052
Y_BOSS_LENGTH = 0.06
Y_BOSS_THICK = 0.006
Y_BOSS_BOTTOM = Y_SADDLE_THICK
TOP_PLATE_LENGTH = 0.092
TOP_PLATE_WIDTH = 0.07
TOP_PLATE_THICK = 0.008
Y_TRAVEL = 0.04


def _add_mesh_visual(part, shape, mesh_name: str, material: str, visual_name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=visual_name,
    )


def _lower_guideway_body_shape():
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICK).translate(
        (0.0, 0.0, BASE_THICK / 2.0)
    )
    holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.12, -0.05), (-0.12, 0.05), (0.12, -0.05), (0.12, 0.05)])
        .circle(BASE_HOLE_RADIUS)
        .extrude(BASE_THICK + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    front_bumper = cq.Workplane("XY").box(0.014, 0.11, 0.006).translate((0.16, 0.0, 0.015))
    rear_bumper = cq.Workplane("XY").box(0.014, 0.11, 0.006).translate((-0.16, 0.0, 0.015))
    return base.cut(holes).union(front_bumper).union(rear_bumper)


def _x_carriage_body_shape():
    bridge = cq.Workplane("XY").box(
        X_CARRIAGE_LENGTH,
        X_CARRIAGE_WIDTH,
        X_BRIDGE_THICK,
    ).translate((0.0, 0.0, X_BRIDGE_BOTTOM + X_BRIDGE_THICK / 2.0))
    left_shoe = cq.Workplane("XY").box(
        X_CARRIAGE_LENGTH - 0.008,
        X_SHOE_WIDTH,
        X_SHOE_HEIGHT,
    ).translate((0.0, -LOWER_RAIL_OFFSET, X_SHOE_CENTER_Z))
    right_shoe = cq.Workplane("XY").box(
        X_CARRIAGE_LENGTH - 0.008,
        X_SHOE_WIDTH,
        X_SHOE_HEIGHT,
    ).translate((0.0, LOWER_RAIL_OFFSET, X_SHOE_CENTER_Z))
    body = bridge.union(left_shoe).union(right_shoe)
    relief = cq.Workplane("XY").box(0.06, 0.044, 0.004).translate((0.0, 0.0, 0.016))
    left_channel = cq.Workplane("XY").box(
        X_CARRIAGE_LENGTH + 0.01,
        LOWER_CHANNEL_WIDTH,
        LOWER_CHANNEL_HEIGHT,
    ).translate((0.0, -LOWER_RAIL_OFFSET, LOWER_CHANNEL_CENTER_Z))
    right_channel = cq.Workplane("XY").box(
        X_CARRIAGE_LENGTH + 0.01,
        LOWER_CHANNEL_WIDTH,
        LOWER_CHANNEL_HEIGHT,
    ).translate((0.0, LOWER_RAIL_OFFSET, LOWER_CHANNEL_CENTER_Z))
    return body.cut(relief).cut(left_channel).cut(right_channel)


def _y_stage_body_shape():
    left_shoe = cq.Workplane("XY").box(
        Y_SHOE_WIDTH,
        Y_STAGE_LENGTH,
        Y_SHOE_HEIGHT,
    ).translate((UPPER_RAIL_X_OFFSET, 0.0, Y_SHOE_CENTER_Z))
    right_shoe = cq.Workplane("XY").box(
        Y_SHOE_WIDTH,
        Y_STAGE_LENGTH,
        Y_SHOE_HEIGHT,
    ).translate((-UPPER_RAIL_X_OFFSET, 0.0, Y_SHOE_CENTER_Z))
    saddle = cq.Workplane("XY").box(
        Y_SADDLE_WIDTH,
        0.086,
        Y_SADDLE_THICK,
    ).translate((0.0, 0.0, Y_SADDLE_THICK / 2.0))
    boss = cq.Workplane("XY").box(
        Y_BOSS_WIDTH,
        Y_BOSS_LENGTH,
        Y_BOSS_THICK,
    ).translate((0.0, 0.0, Y_BOSS_BOTTOM + Y_BOSS_THICK / 2.0))
    body = left_shoe.union(right_shoe).union(saddle).union(boss)
    left_channel = cq.Workplane("XY").box(
        Y_CHANNEL_WIDTH,
        Y_STAGE_LENGTH + 0.014,
        Y_CHANNEL_HEIGHT,
    ).translate((UPPER_RAIL_X_OFFSET, 0.0, Y_CHANNEL_CENTER_Z))
    right_channel = cq.Workplane("XY").box(
        Y_CHANNEL_WIDTH,
        Y_STAGE_LENGTH + 0.014,
        Y_CHANNEL_HEIGHT,
    ).translate((-UPPER_RAIL_X_OFFSET, 0.0, Y_CHANNEL_CENTER_Z))
    return body.cut(left_channel).cut(right_channel)


def _top_plate_shape():
    plate = cq.Workplane("XY").box(
        TOP_PLATE_LENGTH,
        TOP_PLATE_WIDTH,
        TOP_PLATE_THICK,
    ).translate((0.0, 0.0, TOP_PLATE_THICK / 2.0))
    holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.026, -0.02), (-0.026, 0.02), (0.026, -0.02), (0.026, 0.02)])
        .circle(0.0035)
        .extrude(TOP_PLATE_THICK + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return plate.cut(holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_plate_xy_stage")

    model.material("base_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("machined_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("plate_blue", rgba=(0.20, 0.38, 0.71, 1.0))

    lower_guideway = model.part("lower_guideway")
    _add_mesh_visual(
        lower_guideway,
        _lower_guideway_body_shape(),
        "lower_guideway_body",
        "base_black",
        "guideway_body",
    )
    lower_guideway.visual(
        Box((LOWER_RAIL_LENGTH, LOWER_RAIL_WIDTH, LOWER_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -LOWER_RAIL_OFFSET, BASE_THICK + LOWER_RAIL_HEIGHT / 2.0)),
        material="rail_steel",
        name="lower_rail_left",
    )
    lower_guideway.visual(
        Box((LOWER_RAIL_LENGTH, LOWER_RAIL_WIDTH, LOWER_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, LOWER_RAIL_OFFSET, BASE_THICK + LOWER_RAIL_HEIGHT / 2.0)),
        material="rail_steel",
        name="lower_rail_right",
    )
    lower_guideway.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICK + LOWER_RAIL_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICK + LOWER_RAIL_HEIGHT) / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    _add_mesh_visual(
        x_carriage,
        _x_carriage_body_shape(),
        "x_carriage_body",
        "machined_aluminum",
        "x_carriage_body",
    )
    x_carriage.visual(
        Box((UPPER_RAIL_WIDTH, UPPER_RAIL_LENGTH, UPPER_RAIL_HEIGHT)),
        origin=Origin(xyz=(UPPER_RAIL_X_OFFSET, 0.0, X_BRIDGE_TOP + UPPER_RAIL_HEIGHT / 2.0)),
        material="rail_steel",
        name="upper_rail_left",
    )
    x_carriage.visual(
        Box((UPPER_RAIL_WIDTH, UPPER_RAIL_LENGTH, UPPER_RAIL_HEIGHT)),
        origin=Origin(xyz=(-UPPER_RAIL_X_OFFSET, 0.0, X_BRIDGE_TOP + UPPER_RAIL_HEIGHT / 2.0)),
        material="rail_steel",
        name="upper_rail_right",
    )
    x_carriage.visual(
        Box((0.09, LOWER_RAIL_WIDTH - 0.002, 0.002)),
        origin=Origin(xyz=(0.0, -LOWER_RAIL_OFFSET, 0.001)),
        material="base_black",
        name="lower_wiper_left",
    )
    x_carriage.visual(
        Box((0.09, LOWER_RAIL_WIDTH - 0.002, 0.002)),
        origin=Origin(xyz=(0.0, LOWER_RAIL_OFFSET, 0.001)),
        material="base_black",
        name="lower_wiper_right",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, UPPER_RAIL_TOP + 0.012)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, (UPPER_RAIL_TOP - 0.012) / 2.0)),
    )

    y_stage = model.part("y_stage")
    _add_mesh_visual(
        y_stage,
        _y_stage_body_shape(),
        "y_stage_body",
        "machined_aluminum",
        "y_stage_body",
    )
    y_stage.visual(
        mesh_from_cadquery(_top_plate_shape(), "top_plate"),
        origin=Origin(xyz=(0.0, 0.0, Y_BOSS_BOTTOM + Y_BOSS_THICK)),
        material="plate_blue",
        name="top_plate",
    )
    y_stage.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_LENGTH, Y_STAGE_LENGTH, TOP_PLATE_THICK + 0.026)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "lower_to_x",
        ArticulationType.PRISMATIC,
        parent=lower_guideway,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK + LOWER_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=180.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, UPPER_RAIL_TOP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=120.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_guideway = object_model.get_part("lower_guideway")
    x_carriage = object_model.get_part("x_carriage")
    y_stage = object_model.get_part("y_stage")
    lower_to_x = object_model.get_articulation("lower_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    lower_rail_left = lower_guideway.get_visual("lower_rail_left")
    upper_rail_left = x_carriage.get_visual("upper_rail_left")

    ctx.check(
        "all xy stage parts resolve",
        all(part is not None for part in (lower_guideway, x_carriage, y_stage)),
        details="lower_guideway, x_carriage, and y_stage should all exist.",
    )
    ctx.check(
        "prismatic axes are orthogonal",
        lower_to_x.axis == (1.0, 0.0, 0.0) and x_to_y.axis == (0.0, 1.0, 0.0),
        details=f"lower_to_x.axis={lower_to_x.axis}, x_to_y.axis={x_to_y.axis}",
    )

    ctx.expect_within(
        x_carriage,
        lower_guideway,
        axes="y",
        margin=0.0,
        name="x carriage stays within lower guideway width",
    )
    ctx.expect_overlap(
        x_carriage,
        lower_guideway,
        axes="x",
        elem_b=lower_rail_left,
        min_overlap=0.11,
        name="x carriage is captured on the lower rail set",
    )
    ctx.expect_within(
        y_stage,
        x_carriage,
        axes="x",
        margin=0.0,
        name="y stage stays within x carriage bridge width",
    )
    ctx.expect_overlap(
        y_stage,
        x_carriage,
        axes="y",
        elem_b=upper_rail_left,
        min_overlap=0.055,
        name="y stage is captured on the turned upper slide",
    )

    x_rest = ctx.part_world_position(x_carriage)
    y_rest = ctx.part_world_position(y_stage)

    with ctx.pose({lower_to_x: X_TRAVEL}):
        ctx.expect_within(
            x_carriage,
            lower_guideway,
            axes="y",
            margin=0.0,
            name="extended x carriage stays within lower guideway width",
        )
        ctx.expect_overlap(
            x_carriage,
            lower_guideway,
            axes="x",
            elem_b=lower_rail_left,
            min_overlap=0.11,
            name="extended x carriage retains lower rail engagement",
        )
        x_extended = ctx.part_world_position(x_carriage)

    with ctx.pose({x_to_y: Y_TRAVEL}):
        ctx.expect_within(
            y_stage,
            x_carriage,
            axes="x",
            margin=0.0,
            name="extended y stage stays within x carriage bridge width",
        )
        ctx.expect_overlap(
            y_stage,
            x_carriage,
            axes="y",
            elem_b=upper_rail_left,
            min_overlap=0.055,
            name="extended y stage retains upper rail engagement",
        )
        y_extended = ctx.part_world_position(y_stage)

    ctx.check(
        "x carriage moves along +X",
        x_rest is not None
        and x_extended is not None
        and x_extended[0] > x_rest[0] + 0.05
        and abs(x_extended[1] - x_rest[1]) < 1e-6,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "y stage moves along +Y",
        y_rest is not None
        and y_extended is not None
        and y_extended[1] > y_rest[1] + 0.03
        and abs(y_extended[0] - y_rest[0]) < 1e-6,
        details=f"rest={y_rest}, extended={y_extended}",
    )
    ctx.check(
        "upper stage sits above the carriage",
        x_rest is not None and y_rest is not None and y_rest[2] > x_rest[2] + 0.02,
        details=f"x_rest={x_rest}, y_rest={y_rest}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
