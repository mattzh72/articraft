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


BASE_LENGTH = 0.62
BASE_WIDTH = 0.28
BASE_FOOT_HEIGHT = 0.018
BASE_DECK_THICKNESS = 0.026
BASE_TOP_Z = BASE_FOOT_HEIGHT + BASE_DECK_THICKNESS
X_RAIL_LENGTH = 0.54
X_RAIL_WIDTH = 0.022
X_RAIL_HEIGHT = 0.016
X_RAIL_Y = 0.078
X_STAGE_Z = BASE_TOP_Z + X_RAIL_HEIGHT

X_CARRIAGE_LENGTH = 0.22
X_CARRIAGE_WIDTH = 0.23
X_SHOE_LENGTH = 0.16
X_SHOE_WIDTH = 0.048
X_SHOE_HEIGHT = 0.020
X_BRIDGE_HEIGHT = 0.014
X_TOP_DECK_HEIGHT = 0.010
Y_RAIL_LENGTH = 0.24
Y_RAIL_WIDTH = 0.018
Y_RAIL_HEIGHT = 0.012
Y_RAIL_X = 0.046
Y_STAGE_Z = X_SHOE_HEIGHT + X_BRIDGE_HEIGHT + X_TOP_DECK_HEIGHT + Y_RAIL_HEIGHT

Y_SADDLE_LENGTH = 0.125
Y_SADDLE_WIDTH = 0.115
Y_SADDLE_HEIGHT = 0.018
Y_UPPER_DECK_LENGTH = 0.10
Y_UPPER_DECK_WIDTH = 0.16
Y_UPPER_DECK_HEIGHT = 0.012
Y_COLUMN_THICKNESS = 0.040
Y_COLUMN_WIDTH = 0.090
Y_COLUMN_HEIGHT = 0.185
Y_COLUMN_CENTER_X = 0.042
Z_GUIDE_RAIL_THICKNESS = 0.010
Z_GUIDE_RAIL_WIDTH = 0.016
Z_GUIDE_RAIL_HEIGHT = 0.150
Z_GUIDE_RAIL_Y = 0.025
Y_STAGE_MAX_X = (
    Y_COLUMN_CENTER_X
    + Y_COLUMN_THICKNESS / 2.0
    + Z_GUIDE_RAIL_THICKNESS
)
Z_JOINT_Z = 0.110

Z_CARRIAGE_DEPTH = 0.026
Z_CARRIAGE_WIDTH = 0.076
Z_CARRIAGE_HEIGHT = 0.086
Z_FACEPLATE_DEPTH = 0.018
Z_FACEPLATE_WIDTH = 0.110
Z_FACEPLATE_HEIGHT = 0.062
Z_NOSE_DEPTH = 0.020
Z_NOSE_WIDTH = 0.045
Z_NOSE_HEIGHT = 0.035
Z_SPINDLE_RADIUS = 0.007
Z_SPINDLE_LENGTH = 0.040

X_TRAVEL = 0.18
Y_TRAVEL = 0.10
Z_TRAVEL = 0.08


def _raised_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _base_shape() -> cq.Workplane:
    deck = _raised_box(
        (BASE_LENGTH, BASE_WIDTH, BASE_DECK_THICKNESS),
        (0.0, 0.0, BASE_FOOT_HEIGHT + BASE_DECK_THICKNESS / 2.0),
    )
    foot_length = BASE_LENGTH * 0.92
    foot_width = 0.034
    foot_offset_y = BASE_WIDTH / 2.0 - foot_width / 2.0 - 0.028
    left_foot = _raised_box(
        (foot_length, foot_width, BASE_FOOT_HEIGHT),
        (0.0, -foot_offset_y, BASE_FOOT_HEIGHT / 2.0),
    )
    right_foot = _raised_box(
        (foot_length, foot_width, BASE_FOOT_HEIGHT),
        (0.0, foot_offset_y, BASE_FOOT_HEIGHT / 2.0),
    )
    center_spine = _raised_box(
        (BASE_LENGTH * 0.76, 0.042, 0.012),
        (0.0, 0.0, BASE_TOP_Z - 0.006),
    )
    left_rail = _raised_box(
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (0.0, -X_RAIL_Y, BASE_TOP_Z + X_RAIL_HEIGHT / 2.0),
    )
    right_rail = _raised_box(
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (0.0, X_RAIL_Y, BASE_TOP_Z + X_RAIL_HEIGHT / 2.0),
    )
    end_blocks = _raised_box(
        (0.040, BASE_WIDTH * 0.76, 0.020),
        (-BASE_LENGTH / 2.0 + 0.020, 0.0, BASE_TOP_Z - 0.010),
    ).union(
        _raised_box(
            (0.040, BASE_WIDTH * 0.76, 0.020),
            (BASE_LENGTH / 2.0 - 0.020, 0.0, BASE_TOP_Z - 0.010),
        )
    )

    return deck.union(left_foot).union(right_foot).union(center_spine).union(left_rail).union(right_rail).union(end_blocks)


def _x_carriage_shape() -> cq.Workplane:
    left_shoe = _raised_box(
        (X_SHOE_LENGTH, X_SHOE_WIDTH, X_SHOE_HEIGHT),
        (0.0, -X_RAIL_Y, X_SHOE_HEIGHT / 2.0),
    )
    right_shoe = _raised_box(
        (X_SHOE_LENGTH, X_SHOE_WIDTH, X_SHOE_HEIGHT),
        (0.0, X_RAIL_Y, X_SHOE_HEIGHT / 2.0),
    )
    bridge = _raised_box(
        (X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, X_BRIDGE_HEIGHT),
        (0.0, 0.0, X_SHOE_HEIGHT + X_BRIDGE_HEIGHT / 2.0),
    )
    top_deck = _raised_box(
        (X_CARRIAGE_LENGTH * 0.86, X_CARRIAGE_WIDTH * 0.92, X_TOP_DECK_HEIGHT),
        (
            0.0,
            0.0,
            X_SHOE_HEIGHT + X_BRIDGE_HEIGHT + X_TOP_DECK_HEIGHT / 2.0,
        ),
    )
    left_y_rail = _raised_box(
        (Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT),
        (
            -Y_RAIL_X,
            0.0,
            X_SHOE_HEIGHT + X_BRIDGE_HEIGHT + X_TOP_DECK_HEIGHT + Y_RAIL_HEIGHT / 2.0,
        ),
    )
    right_y_rail = _raised_box(
        (Y_RAIL_WIDTH, Y_RAIL_LENGTH, Y_RAIL_HEIGHT),
        (
            Y_RAIL_X,
            0.0,
            X_SHOE_HEIGHT + X_BRIDGE_HEIGHT + X_TOP_DECK_HEIGHT + Y_RAIL_HEIGHT / 2.0,
        ),
    )
    cable_plate = _raised_box(
        (0.060, 0.090, 0.012),
        (-X_CARRIAGE_LENGTH * 0.28, 0.0, X_SHOE_HEIGHT + X_BRIDGE_HEIGHT + 0.006),
    )

    return (
        left_shoe.union(right_shoe)
        .union(bridge)
        .union(top_deck)
        .union(left_y_rail)
        .union(right_y_rail)
        .union(cable_plate)
    )


def _y_slide_shape() -> cq.Workplane:
    saddle = _raised_box(
        (Y_SADDLE_LENGTH, Y_SADDLE_WIDTH, Y_SADDLE_HEIGHT),
        (0.0, 0.0, Y_SADDLE_HEIGHT / 2.0),
    )
    upper_deck = _raised_box(
        (Y_UPPER_DECK_LENGTH, Y_UPPER_DECK_WIDTH, Y_UPPER_DECK_HEIGHT),
        (
            -0.004,
            0.0,
            Y_SADDLE_HEIGHT + Y_UPPER_DECK_HEIGHT / 2.0,
        ),
    )
    column = _raised_box(
        (Y_COLUMN_THICKNESS, Y_COLUMN_WIDTH, Y_COLUMN_HEIGHT),
        (
            Y_COLUMN_CENTER_X,
            0.0,
            Y_SADDLE_HEIGHT + Y_COLUMN_HEIGHT / 2.0,
        ),
    )
    left_guide = _raised_box(
        (Z_GUIDE_RAIL_THICKNESS, Z_GUIDE_RAIL_WIDTH, Z_GUIDE_RAIL_HEIGHT),
        (
            Y_COLUMN_CENTER_X + Y_COLUMN_THICKNESS / 2.0 + Z_GUIDE_RAIL_THICKNESS / 2.0,
            -Z_GUIDE_RAIL_Y,
            Y_SADDLE_HEIGHT + Z_GUIDE_RAIL_HEIGHT / 2.0 + 0.012,
        ),
    )
    right_guide = _raised_box(
        (Z_GUIDE_RAIL_THICKNESS, Z_GUIDE_RAIL_WIDTH, Z_GUIDE_RAIL_HEIGHT),
        (
            Y_COLUMN_CENTER_X + Y_COLUMN_THICKNESS / 2.0 + Z_GUIDE_RAIL_THICKNESS / 2.0,
            Z_GUIDE_RAIL_Y,
            Y_SADDLE_HEIGHT + Z_GUIDE_RAIL_HEIGHT / 2.0 + 0.012,
        ),
    )
    left_gusset = _raised_box(
        (0.030, 0.020, 0.082),
        (0.016, -0.035, Y_SADDLE_HEIGHT + 0.041),
    )
    right_gusset = _raised_box(
        (0.030, 0.020, 0.082),
        (0.016, 0.035, Y_SADDLE_HEIGHT + 0.041),
    )
    top_cap = _raised_box(
        (0.052, 0.082, 0.014),
        (
            Y_COLUMN_CENTER_X + 0.004,
            0.0,
            Y_SADDLE_HEIGHT + Y_COLUMN_HEIGHT - 0.007,
        ),
    )

    return (
        saddle.union(upper_deck)
        .union(column)
        .union(left_guide)
        .union(right_guide)
        .union(left_gusset)
        .union(right_gusset)
        .union(top_cap)
    )


def _z_head_shape() -> cq.Workplane:
    carriage = _raised_box(
        (Z_CARRIAGE_DEPTH, Z_CARRIAGE_WIDTH, Z_CARRIAGE_HEIGHT),
        (Z_CARRIAGE_DEPTH / 2.0, 0.0, 0.0),
    )
    faceplate = _raised_box(
        (Z_FACEPLATE_DEPTH, Z_FACEPLATE_WIDTH, Z_FACEPLATE_HEIGHT),
        (Z_CARRIAGE_DEPTH + Z_FACEPLATE_DEPTH / 2.0, 0.0, -0.010),
    )
    nose = _raised_box(
        (Z_NOSE_DEPTH, Z_NOSE_WIDTH, Z_NOSE_HEIGHT),
        (
            Z_CARRIAGE_DEPTH + Z_FACEPLATE_DEPTH + Z_NOSE_DEPTH / 2.0,
            0.0,
            -0.052,
        ),
    )
    spindle = (
        cq.Workplane("XY")
        .circle(Z_SPINDLE_RADIUS)
        .extrude(Z_SPINDLE_LENGTH)
        .translate(
            (
                Z_CARRIAGE_DEPTH + Z_FACEPLATE_DEPTH + Z_NOSE_DEPTH * 0.55,
                0.0,
                -0.092,
            )
        )
    )
    top_bridge = _raised_box(
        (0.020, 0.060, 0.018),
        (Z_CARRIAGE_DEPTH + 0.010, 0.0, 0.038),
    )

    return carriage.union(faceplate).union(nose).union(spindle).union(top_bridge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cartesian_transfer_stage")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("rail_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("stage_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("head_blue", rgba=(0.24, 0.38, 0.64, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "transfer_stage_base"),
        material="base_charcoal",
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, X_STAGE_Z)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_Z / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_shape(), "transfer_stage_x_carriage"),
        material="stage_aluminum",
        name="x_carriage_body",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, Y_STAGE_Z)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_Z / 2.0)),
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_y_slide_shape(), "transfer_stage_y_slide"),
        material="stage_aluminum",
        name="y_slide_body",
    )
    y_slide.inertial = Inertial.from_geometry(
        Box((Y_STAGE_MAX_X + Y_SADDLE_LENGTH / 2.0, Y_UPPER_DECK_WIDTH, Y_SADDLE_HEIGHT + Y_COLUMN_HEIGHT)),
        mass=2.3,
        origin=Origin(xyz=(0.020, 0.0, (Y_SADDLE_HEIGHT + Y_COLUMN_HEIGHT) / 2.0)),
    )

    z_head = model.part("z_head")
    z_head.visual(
        mesh_from_cadquery(_z_head_shape(), "transfer_stage_z_head"),
        material="head_blue",
        name="z_head_body",
    )
    z_head.inertial = Inertial.from_geometry(
        Box((0.070, Z_FACEPLATE_WIDTH, 0.15)),
        mass=1.0,
        origin=Origin(xyz=(0.035, 0.0, -0.020)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.28,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=z_head,
        origin=Origin(xyz=(Y_STAGE_MAX_X, 0.0, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.22,
            lower=-0.020,
            upper=Z_TRAVEL,
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
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    z_head = object_model.get_part("z_head")
    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    y_to_z = object_model.get_articulation("y_to_z")

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="x carriage rides directly on the base rails",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="y",
        min_overlap=0.18,
        name="x carriage spans both base rails",
    )
    ctx.expect_gap(
        y_slide,
        x_carriage,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="y slide sits on top of the cross rails",
    )
    ctx.expect_overlap(
        y_slide,
        x_carriage,
        axes="x",
        min_overlap=0.08,
        name="y slide stays centered between the y rails",
    )
    ctx.expect_gap(
        z_head,
        y_slide,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        name="z head is mounted on the front guide face",
    )
    ctx.expect_overlap(
        z_head,
        y_slide,
        axes="z",
        min_overlap=0.06,
        name="z head remains engaged with the upright guide",
    )

    x_rest = ctx.part_world_position(x_carriage)
    y_rest = ctx.part_world_position(y_slide)
    z_rest = ctx.part_world_position(z_head)
    with ctx.pose({base_to_x: X_TRAVEL, x_to_y: Y_TRAVEL, y_to_z: Z_TRAVEL}):
        ctx.expect_overlap(
            y_slide,
            x_carriage,
            axes="y",
            min_overlap=0.04,
            name="y slide retains rail engagement at full y travel",
        )
        ctx.expect_overlap(
            z_head,
            y_slide,
            axes="z",
            min_overlap=0.04,
            name="z head retains guide engagement at full z travel",
        )
        x_extended = ctx.part_world_position(x_carriage)
        y_extended = ctx.part_world_position(y_slide)
        z_extended = ctx.part_world_position(z_head)

    ctx.check(
        "x carriage moves along +X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.10,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "y slide moves along +Y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.06,
        details=f"rest={y_rest}, extended={y_extended}",
    )
    ctx.check(
        "z head moves upward",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.05,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
