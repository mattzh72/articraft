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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COLUMN_X = 0.128

FOOT_T = 0.010
FOOT_W = 0.038
FOOT_D = 0.190
REAR_TIE_D = 0.030
REAR_TIE_Y = -0.065
REAR_TIE_W = (2.0 * COLUMN_X) + 0.054

SLEEVE_OUT_X = 0.034
SLEEVE_OUT_Y = 0.028
SLEEVE_WALL = 0.0025
SLEEVE_H = 0.160
SLEEVE_Z = FOOT_T + (SLEEVE_H / 2.0) - 0.0005

SLIDE_ORIGIN_Z = 0.145
SLIDE_TRAVEL = 0.120

INNER_COL_X = 0.024
INNER_COL_Y = 0.018
INNER_COL_BOTTOM_Z = -0.110
INNER_COL_TOP_Z = 0.185
INNER_COL_H = INNER_COL_TOP_Z - INNER_COL_BOTTOM_Z
INNER_COL_CENTER_Z = (INNER_COL_TOP_Z + INNER_COL_BOTTOM_Z) / 2.0

HEAD_X = 0.036
HEAD_Y = 0.026
HEAD_Z = 0.028
HEAD_CENTER_Z = 0.174

BAR_R = 0.008
BAR_LEN = 0.286
BAR_Z = 0.190

TRAY_W = 0.300
TRAY_D = 0.230
TRAY_T = 0.004
TRAY_REAR_Y = 0.004
TRAY_CENTER_Y = TRAY_REAR_Y + (TRAY_D / 2.0)
TRAY_CENTER_Z = 0.012
TRAY_TAB_W = 0.036
TRAY_TAB_D = 0.012
TRAY_TAB_H = 0.018
TRAY_TAB_X = 0.103
TRAY_FRONT_Y = TRAY_REAR_Y + TRAY_D
TRAY_KNUCKLE_R = 0.013
TRAY_KNUCKLE_CLEAR_R = BAR_R
TRAY_KNUCKLE_LEN = 0.028
TRAY_KNUCKLE_XS = (-0.094, 0.0, 0.094)
TRAY_TILT_UPPER = 0.90


def _sleeve_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(SLEEVE_OUT_X, SLEEVE_OUT_Y, SLEEVE_H)
    inner = cq.Workplane("XY").box(
        SLEEVE_OUT_X - (2.0 * SLEEVE_WALL),
        SLEEVE_OUT_Y - (2.0 * SLEEVE_WALL),
        SLEEVE_H + 0.004,
    )
    return outer.cut(inner)


def _tray_shape() -> cq.Workplane:
    tray = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_T).translate(
        (0.0, TRAY_CENTER_Y, TRAY_CENTER_Z)
    )

    slot_points = [
        (x_pos, y_pos)
        for x_pos in (-0.082, 0.0, 0.082)
        for y_pos in (0.060, 0.110, 0.160)
    ]
    slot_cutter = (
        cq.Workplane("XY")
        .pushPoints(slot_points)
        .slot2D(0.056, 0.010, angle=90.0)
        .extrude(0.040)
        .translate((0.0, 0.0, -0.010))
    )
    tray = tray.cut(slot_cutter)

    for x_pos in (-TRAY_TAB_X, TRAY_TAB_X):
        tray = tray.union(
            cq.Workplane("XY")
            .box(TRAY_TAB_W, TRAY_TAB_D, TRAY_TAB_H)
            .translate(
                (
                    x_pos,
                    TRAY_FRONT_Y - (TRAY_TAB_D / 2.0),
                    TRAY_CENTER_Z + (TRAY_TAB_H / 2.0) - 0.001,
                )
            )
        )

    for x_pos in TRAY_KNUCKLE_XS:
        knuckle = (
            cq.Workplane("YZ")
            .circle(TRAY_KNUCKLE_R)
            .extrude(TRAY_KNUCKLE_LEN / 2.0, both=True)
            .translate((x_pos, 0.0, 0.0))
        )
        bore = (
            cq.Workplane("YZ")
            .circle(TRAY_KNUCKLE_CLEAR_R)
            .extrude((TRAY_KNUCKLE_LEN / 2.0) + 0.004, both=True)
            .translate((x_pos, 0.0, 0.0))
        )
        tray = tray.union(knuckle).cut(bore)

    side_rails = (
        cq.Workplane("XY")
        .box(0.008, 0.155, 0.010)
        .translate((-(TRAY_W / 2.0) + 0.003, 0.112, 0.015))
        .union(
            cq.Workplane("XY")
            .box(0.008, 0.155, 0.010)
            .translate(((TRAY_W / 2.0) - 0.003, 0.112, 0.015))
        )
    )
    tray = tray.union(side_rails)

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_laptop_stand")

    model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("satin_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("matte_black", rgba=(0.11, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        Box((FOOT_W, FOOT_D, FOOT_T)),
        origin=Origin(xyz=(-COLUMN_X, 0.030, FOOT_T / 2.0)),
        material="graphite",
        name="left_foot",
    )
    base.visual(
        Box((FOOT_W, FOOT_D, FOOT_T)),
        origin=Origin(xyz=(COLUMN_X, 0.030, FOOT_T / 2.0)),
        material="graphite",
        name="right_foot",
    )
    base.visual(
        Box((REAR_TIE_W, REAR_TIE_D, FOOT_T)),
        origin=Origin(xyz=(0.0, REAR_TIE_Y, FOOT_T / 2.0)),
        material="graphite",
        name="rear_tie",
    )
    base.visual(
        Box((REAR_TIE_W - 0.048, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, 0.040)),
        material="graphite",
        name="rear_spine",
    )
    base.visual(
        mesh_from_cadquery(_sleeve_shape(), "left_sleeve"),
        origin=Origin(xyz=(-COLUMN_X, 0.0, SLEEVE_Z)),
        material="matte_black",
        name="left_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_sleeve_shape(), "right_sleeve"),
        origin=Origin(xyz=(COLUMN_X, 0.0, SLEEVE_Z)),
        material="matte_black",
        name="right_sleeve",
    )

    lift_frame = model.part("lift_frame")
    lift_frame.visual(
        Box((INNER_COL_X, INNER_COL_Y, INNER_COL_H)),
        origin=Origin(xyz=(-COLUMN_X, 0.0, INNER_COL_CENTER_Z)),
        material="satin_silver",
        name="left_column",
    )
    lift_frame.visual(
        Box((INNER_COL_X, INNER_COL_Y, INNER_COL_H)),
        origin=Origin(xyz=(COLUMN_X, 0.0, INNER_COL_CENTER_Z)),
        material="satin_silver",
        name="right_column",
    )
    lift_frame.visual(
        Box((HEAD_X, HEAD_Y, HEAD_Z)),
        origin=Origin(xyz=(-COLUMN_X, 0.0, HEAD_CENTER_Z)),
        material="graphite",
        name="left_head",
    )
    lift_frame.visual(
        Box((HEAD_X, HEAD_Y, HEAD_Z)),
        origin=Origin(xyz=(COLUMN_X, 0.0, HEAD_CENTER_Z)),
        material="graphite",
        name="right_head",
    )
    lift_frame.visual(
        Cylinder(radius=BAR_R, length=BAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, BAR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="graphite",
        name="rear_bar",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray"),
        material="graphite",
        name="tray",
    )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_frame,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=120.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=lift_frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, BAR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TRAY_TILT_UPPER,
            effort=24.0,
            velocity=1.8,
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
    lift_frame = object_model.get_part("lift_frame")
    tray = object_model.get_part("tray")
    column_slide = object_model.get_articulation("column_slide")
    tray_tilt = object_model.get_articulation("tray_tilt")

    ctx.allow_overlap(
        lift_frame,
        tray,
        elem_a="rear_bar",
        elem_b="tray",
        reason="The common rear hinge bar intentionally passes through the tray knuckles.",
    )

    ctx.expect_within(
        lift_frame,
        base,
        axes="xy",
        inner_elem="left_column",
        outer_elem="left_sleeve",
        margin=0.0015,
        name="left column stays centered in left sleeve at rest",
    )
    ctx.expect_within(
        lift_frame,
        base,
        axes="xy",
        inner_elem="right_column",
        outer_elem="right_sleeve",
        margin=0.0015,
        name="right column stays centered in right sleeve at rest",
    )
    ctx.expect_overlap(
        lift_frame,
        base,
        axes="z",
        elem_a="left_column",
        elem_b="left_sleeve",
        min_overlap=0.120,
        name="left column has deep retained insertion at rest",
    )
    ctx.expect_overlap(
        lift_frame,
        base,
        axes="z",
        elem_a="right_column",
        elem_b="right_sleeve",
        min_overlap=0.120,
        name="right column has deep retained insertion at rest",
    )

    left_rest = ctx.part_element_world_aabb(lift_frame, elem="left_column")
    right_rest = ctx.part_element_world_aabb(lift_frame, elem="right_column")
    ctx.check(
        "columns are matched counterparts at rest",
        left_rest is not None
        and right_rest is not None
        and abs(left_rest[0][2] - right_rest[0][2]) < 1e-6
        and abs(left_rest[1][2] - right_rest[1][2]) < 1e-6
        and abs((left_rest[1][0] - left_rest[0][0]) - (right_rest[1][0] - right_rest[0][0])) < 1e-6
        and abs((left_rest[1][1] - left_rest[0][1]) - (right_rest[1][1] - right_rest[0][1])) < 1e-6,
        details=f"left={left_rest}, right={right_rest}",
    )

    tray_rest = ctx.part_element_world_aabb(tray, elem="tray")
    lift_rest_pos = ctx.part_world_position(lift_frame)
    with ctx.pose({column_slide: SLIDE_TRAVEL, tray_tilt: 0.72}):
        ctx.expect_within(
            lift_frame,
            base,
            axes="xy",
            inner_elem="left_column",
            outer_elem="left_sleeve",
            margin=0.0015,
            name="left column stays centered in left sleeve when raised",
        )
        ctx.expect_within(
            lift_frame,
            base,
            axes="xy",
            inner_elem="right_column",
            outer_elem="right_sleeve",
            margin=0.0015,
            name="right column stays centered in right sleeve when raised",
        )
        ctx.expect_overlap(
            lift_frame,
            base,
            axes="z",
            elem_a="left_column",
            elem_b="left_sleeve",
            min_overlap=0.012,
            name="left column keeps insertion at max height",
        )
        ctx.expect_overlap(
            lift_frame,
            base,
            axes="z",
            elem_a="right_column",
            elem_b="right_sleeve",
            min_overlap=0.012,
            name="right column keeps insertion at max height",
        )

        left_raised = ctx.part_element_world_aabb(lift_frame, elem="left_column")
        right_raised = ctx.part_element_world_aabb(lift_frame, elem="right_column")
        tray_raised = ctx.part_element_world_aabb(tray, elem="tray")
        lift_raised_pos = ctx.part_world_position(lift_frame)

        ctx.check(
            "columns stay level when raised",
            left_raised is not None
            and right_raised is not None
            and abs(left_raised[0][2] - right_raised[0][2]) < 1e-6
            and abs(left_raised[1][2] - right_raised[1][2]) < 1e-6,
            details=f"left={left_raised}, right={right_raised}",
        )
        ctx.check(
            "lift frame moves upward",
            lift_rest_pos is not None
            and lift_raised_pos is not None
            and lift_raised_pos[2] > lift_rest_pos[2] + 0.10,
            details=f"rest={lift_rest_pos}, raised={lift_raised_pos}",
        )
        ctx.check(
            "tray front lifts under tilt",
            tray_rest is not None
            and tray_raised is not None
            and tray_raised[1][2] > tray_rest[1][2] + 0.11,
            details=f"rest={tray_rest}, raised={tray_raised}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
