from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_L = 0.42
BASE_W = 0.24
BASE_T = 0.018

X_RAIL_L = 0.34
X_RAIL_W = 0.026
X_RAIL_H = 0.016
X_RAIL_Y = 0.075

X_FOOT_L = 0.12
X_FOOT_W = 0.038
X_FOOT_H = 0.012
X_BRIDGE_L = 0.18
X_BRIDGE_W = 0.17
X_BRIDGE_T = 0.018

Y_RAIL_L = 0.32
Y_RAIL_W = 0.022
Y_RAIL_H = 0.014
Y_RAIL_X = 0.05

Y_FOOT_L = 0.038
Y_FOOT_W = 0.09
Y_FOOT_H = 0.012
Y_SADDLE_L = 0.15
Y_SADDLE_W = 0.17
Y_SADDLE_T = 0.014
Y_COLUMN_L = 0.06
Y_COLUMN_W = 0.06
Y_COLUMN_H = 0.022
TABLE_L = 0.20
TABLE_W = 0.16
TABLE_T = 0.012

TRAVEL = 0.10


def _base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    x_rail_pos = cq.Workplane("XY").box(
        X_RAIL_L, X_RAIL_W, X_RAIL_H, centered=(True, True, False)
    ).translate((0.0, X_RAIL_Y, BASE_T))
    x_rail_neg = cq.Workplane("XY").box(
        X_RAIL_L, X_RAIL_W, X_RAIL_H, centered=(True, True, False)
    ).translate((0.0, -X_RAIL_Y, BASE_T))
    end_pad_pos = cq.Workplane("XY").box(
        0.03, BASE_W * 0.72, 0.008, centered=(True, True, False)
    ).translate((BASE_L * 0.5 - 0.03, 0.0, BASE_T))
    end_pad_neg = cq.Workplane("XY").box(
        0.03, BASE_W * 0.72, 0.008, centered=(True, True, False)
    ).translate((-BASE_L * 0.5 + 0.03, 0.0, BASE_T))
    return plate.union(x_rail_pos).union(x_rail_neg).union(end_pad_pos).union(end_pad_neg)


def _x_carriage_shape() -> cq.Workplane:
    foot_pos = cq.Workplane("XY").box(
        X_FOOT_L, X_FOOT_W, X_FOOT_H, centered=(True, True, False)
    ).translate((0.0, X_RAIL_Y, 0.0))
    foot_neg = cq.Workplane("XY").box(
        X_FOOT_L, X_FOOT_W, X_FOOT_H, centered=(True, True, False)
    ).translate((0.0, -X_RAIL_Y, 0.0))
    bridge = (
        cq.Workplane("XY")
        .box(X_BRIDGE_L, X_BRIDGE_W, X_BRIDGE_T, centered=(True, True, False))
        .translate((0.0, 0.0, X_FOOT_H))
    )
    bridge = (
        bridge.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.11, 0.09)
        .cutBlind(-0.005)
    )
    y_rail_pos = cq.Workplane("XY").box(
        Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, centered=(True, True, False)
    ).translate((Y_RAIL_X, 0.0, X_FOOT_H + X_BRIDGE_T))
    y_rail_neg = cq.Workplane("XY").box(
        Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, centered=(True, True, False)
    ).translate((-Y_RAIL_X, 0.0, X_FOOT_H + X_BRIDGE_T))
    return foot_pos.union(foot_neg).union(bridge).union(y_rail_pos).union(y_rail_neg)


def _y_table_shape() -> cq.Workplane:
    foot_pos = cq.Workplane("XY").box(
        Y_FOOT_L, Y_FOOT_W, Y_FOOT_H, centered=(True, True, False)
    ).translate((Y_RAIL_X, 0.0, 0.0))
    foot_neg = cq.Workplane("XY").box(
        Y_FOOT_L, Y_FOOT_W, Y_FOOT_H, centered=(True, True, False)
    ).translate((-Y_RAIL_X, 0.0, 0.0))
    saddle = (
        cq.Workplane("XY")
        .box(Y_SADDLE_L, Y_SADDLE_W, Y_SADDLE_T, centered=(True, True, False))
        .translate((0.0, 0.0, Y_FOOT_H))
    )
    saddle = (
        saddle.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.08, 0.10)
        .cutBlind(-0.004)
    )
    column = cq.Workplane("XY").box(
        Y_COLUMN_L, Y_COLUMN_W, Y_COLUMN_H, centered=(True, True, False)
    ).translate((0.0, 0.0, Y_FOOT_H + Y_SADDLE_T))
    table = cq.Workplane("XY").box(TABLE_L, TABLE_W, TABLE_T, centered=(True, True, False)).translate(
        (0.0, 0.0, Y_FOOT_H + Y_SADDLE_T + Y_COLUMN_H)
    )
    return foot_pos.union(foot_neg).union(saddle).union(column).union(table)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xy_positioning_stage", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    table_finish = model.material("table_finish", rgba=(0.68, 0.70, 0.73, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base.obj", assets=ASSETS),
        material=painted_steel,
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + X_RAIL_H)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + X_RAIL_H) * 0.5)),
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_shape(), "x_carriage.obj", assets=ASSETS),
        material=ground_steel,
        name="x_stage_body",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_BRIDGE_L, X_BRIDGE_W, X_FOOT_H + X_BRIDGE_T + Y_RAIL_H)),
        mass=2.8,
        origin=Origin(
            xyz=(0.0, 0.0, (X_FOOT_H + X_BRIDGE_T + Y_RAIL_H) * 0.5),
        ),
    )

    y_table = model.part("y_table")
    y_table.visual(
        mesh_from_cadquery(_y_table_shape(), "y_table.obj", assets=ASSETS),
        material=table_finish,
        name="y_stage_body",
    )
    y_table.inertial = Inertial.from_geometry(
        Box((TABLE_L, TABLE_W, Y_FOOT_H + Y_SADDLE_T + Y_COLUMN_H + TABLE_T)),
        mass=2.1,
        origin=Origin(
            xyz=(0.0, 0.0, (Y_FOOT_H + Y_SADDLE_T + Y_COLUMN_H + TABLE_T) * 0.5),
        ),
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + X_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=-TRAVEL,
            upper=TRAVEL,
        ),
    )

    model.articulation(
        "x_carriage_to_y_table",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, X_FOOT_H + X_BRIDGE_T + Y_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.30,
            lower=-TRAVEL,
            upper=TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_table = object_model.get_part("y_table")
    x_stage = object_model.get_articulation("base_to_x_carriage")
    y_stage = object_model.get_articulation("x_carriage_to_y_table")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "parts_present",
        all(part is not None for part in (base, x_carriage, y_table)),
        "Expected base, x_carriage, and y_table parts.",
    )
    ctx.check(
        "prismatic_axes_are_orthogonal",
        x_stage.axis == (1.0, 0.0, 0.0) and y_stage.axis == (0.0, 1.0, 0.0),
        f"Unexpected joint axes: x={x_stage.axis}, y={y_stage.axis}",
    )
    ctx.check(
        "travel_limits_are_symmetric",
        x_stage.motion_limits.lower == -TRAVEL
        and x_stage.motion_limits.upper == TRAVEL
        and y_stage.motion_limits.lower == -TRAVEL
        and y_stage.motion_limits.upper == TRAVEL,
        "Expected ±100 mm travel on both axes.",
    )

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="x_carriage_seats_on_base_rails",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="xy",
        min_overlap=0.10,
        name="x_carriage_overlaps_base_footprint",
    )
    ctx.expect_gap(
        y_table,
        x_carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="y_table_seats_on_y_rails",
    )
    ctx.expect_overlap(
        y_table,
        x_carriage,
        axes="xy",
        min_overlap=0.08,
        name="y_stage_overlaps_x_carriage_footprint",
    )
    ctx.expect_overlap(
        y_table,
        base,
        axes="xy",
        min_overlap=0.12,
        name="top_table_stays_above_base_envelope_at_center",
    )

    with ctx.pose({x_stage: TRAVEL}):
        ctx.expect_origin_gap(
            x_carriage,
            base,
            axis="x",
            min_gap=TRAVEL - 1e-6,
            max_gap=TRAVEL + 1e-6,
            name="x_stage_reaches_positive_travel",
        )
        ctx.expect_gap(
            x_carriage,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="x_carriage_remains_supported_at_positive_travel",
        )

    with ctx.pose({x_stage: -TRAVEL}):
        ctx.expect_origin_gap(
            base,
            x_carriage,
            axis="x",
            min_gap=TRAVEL - 1e-6,
            max_gap=TRAVEL + 1e-6,
            name="x_stage_reaches_negative_travel",
        )

    with ctx.pose({y_stage: TRAVEL}):
        ctx.expect_origin_gap(
            y_table,
            x_carriage,
            axis="y",
            min_gap=TRAVEL - 1e-6,
            max_gap=TRAVEL + 1e-6,
            name="y_stage_reaches_positive_travel",
        )
        ctx.expect_gap(
            y_table,
            x_carriage,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="y_table_remains_supported_at_positive_travel",
        )

    with ctx.pose({y_stage: -TRAVEL, x_stage: TRAVEL}):
        ctx.expect_origin_gap(
            x_carriage,
            base,
            axis="x",
            min_gap=TRAVEL - 1e-6,
            max_gap=TRAVEL + 1e-6,
            name="x_stage_still_positioned_when_y_moves",
        )
        ctx.expect_origin_gap(
            x_carriage,
            y_table,
            axis="y",
            min_gap=TRAVEL - 1e-6,
            max_gap=TRAVEL + 1e-6,
            name="y_stage_reaches_negative_travel",
        )
        ctx.expect_overlap(
            y_table,
            x_carriage,
            axes="x",
            min_overlap=0.07,
            name="y_table_keeps_crosswise_support_at_corner_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
