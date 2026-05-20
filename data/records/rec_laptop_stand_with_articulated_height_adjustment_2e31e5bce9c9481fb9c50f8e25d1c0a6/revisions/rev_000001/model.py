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


FRAME_W = 0.300
FRAME_D = 0.230
FRAME_H = 0.018
RAIL_W = 0.022

POST_SPACING = 0.252
SLEEVE_W = 0.030
SLEEVE_D = 0.024
SLEEVE_H = 0.090
SLEEVE_WALL = 0.0025

POST_W = 0.022
POST_D = 0.016
POST_HIDDEN = 0.090
POST_VISIBLE = 0.105
LIFT_TRAVEL = 0.065
CAP_W = 0.034
CAP_D = 0.026
CAP_T = 0.004

PAD_W = 0.040
PAD_D = 0.030
PAD_H = 0.010
BEAM_D = 0.032
BEAM_H = 0.012
TRAY_W = 0.320
TRAY_D = 0.245
TRAY_T = 0.004
SIDE_RAIL_T = 0.006
SIDE_RAIL_H = 0.012
FRONT_LIP_T = 0.006
FRONT_LIP_H = 0.016

REAR_Y = -(FRAME_D / 2.0) + (RAIL_W / 2.0)
FRONT_Y = (FRAME_D / 2.0) - (RAIL_W / 2.0)
LEFT_X = -(POST_SPACING / 2.0)
RIGHT_X = POST_SPACING / 2.0
POST_TOP_Z = POST_VISIBLE + CAP_T


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _sleeve_shape(x_center: float) -> cq.Workplane:
    outer = _box_shape(
        (SLEEVE_W, SLEEVE_D, SLEEVE_H),
        (x_center, REAR_Y, FRAME_H + (SLEEVE_H / 2.0)),
    )
    inner = _box_shape(
        (SLEEVE_W - (2.0 * SLEEVE_WALL), SLEEVE_D - (2.0 * SLEEVE_WALL), SLEEVE_H + 0.004),
        (x_center, REAR_Y, FRAME_H + (SLEEVE_H / 2.0)),
    )
    return outer.cut(inner)


def _frame_shape() -> cq.Workplane:
    front_rail = _box_shape((FRAME_W, RAIL_W, FRAME_H), (0.0, FRONT_Y, FRAME_H / 2.0))
    rear_rail = _box_shape((FRAME_W, RAIL_W, FRAME_H), (0.0, REAR_Y, FRAME_H / 2.0))
    side_length = FRAME_D - RAIL_W
    left_rail = _box_shape((RAIL_W, side_length, FRAME_H), (-(FRAME_W / 2.0) + (RAIL_W / 2.0), 0.0, FRAME_H / 2.0))
    right_rail = _box_shape((RAIL_W, side_length, FRAME_H), ((FRAME_W / 2.0) - (RAIL_W / 2.0), 0.0, FRAME_H / 2.0))
    front_pad_left = _box_shape((0.045, 0.030, 0.004), (-0.100, FRONT_Y + 0.010, 0.002))
    front_pad_right = _box_shape((0.045, 0.030, 0.004), (0.100, FRONT_Y + 0.010, 0.002))
    return front_rail.union(rear_rail).union(left_rail).union(right_rail).union(front_pad_left).union(front_pad_right)


def _post_shape() -> cq.Workplane:
    total_height = POST_HIDDEN + POST_VISIBLE
    return cq.Workplane("XY").box(POST_W, POST_D, total_height).translate(
        (0.0, 0.0, (POST_VISIBLE - POST_HIDDEN) / 2.0)
    )


def _tray_body_shape() -> cq.Workplane:
    beam = _box_shape(
        (POST_SPACING + PAD_W, BEAM_D, BEAM_H),
        (POST_SPACING / 2.0, 0.0, PAD_H + (BEAM_H / 2.0)),
    )

    panel_z = PAD_H + BEAM_H
    panel = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_T).translate(
        (POST_SPACING / 2.0, (TRAY_D / 2.0) - 0.010, panel_z + (TRAY_T / 2.0))
    )
    slot_cutters = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (POST_SPACING / 2.0 - 0.070, 0.120),
                (POST_SPACING / 2.0, 0.120),
                (POST_SPACING / 2.0 + 0.070, 0.120),
            ]
        )
        .slot2D(0.150, 0.012, angle=90)
        .extrude(TRAY_T + 0.004)
        .translate((0.0, -0.010, panel_z - 0.002))
    )
    panel = panel.cut(slot_cutters)

    left_side = _box_shape(
        (SIDE_RAIL_T, TRAY_D, SIDE_RAIL_H),
        (
            (POST_SPACING / 2.0) - (TRAY_W / 2.0) + (SIDE_RAIL_T / 2.0),
            (TRAY_D / 2.0) - 0.010,
            panel_z + TRAY_T + (SIDE_RAIL_H / 2.0),
        ),
    )
    right_side = _box_shape(
        (SIDE_RAIL_T, TRAY_D, SIDE_RAIL_H),
        (
            (POST_SPACING / 2.0) + (TRAY_W / 2.0) - (SIDE_RAIL_T / 2.0),
            (TRAY_D / 2.0) - 0.010,
            panel_z + TRAY_T + (SIDE_RAIL_H / 2.0),
        ),
    )
    front_lip = _box_shape(
        (TRAY_W, FRONT_LIP_T, FRONT_LIP_H),
        (
            POST_SPACING / 2.0,
            TRAY_D - 0.010 - (FRONT_LIP_T / 2.0),
            panel_z + TRAY_T + (FRONT_LIP_H / 2.0),
        ),
    )
    return beam.union(panel).union(left_side).union(right_side).union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    frame_mat = model.material("frame", rgba=(0.16, 0.17, 0.18, 1.0))
    sleeve_mat = model.material("sleeve", rgba=(0.20, 0.21, 0.23, 1.0))
    post_mat = model.material("post", rgba=(0.55, 0.57, 0.60, 1.0))
    tray_mat = model.material("tray", rgba=(0.78, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_frame_shape(), "base_frame"), material=frame_mat, name="frame")
    base.visual(mesh_from_cadquery(_sleeve_shape(LEFT_X), "left_sleeve"), material=sleeve_mat, name="left_sleeve")
    base.visual(mesh_from_cadquery(_sleeve_shape(RIGHT_X), "right_sleeve"), material=sleeve_mat, name="right_sleeve")

    left_post = model.part("left_post")
    left_post.visual(mesh_from_cadquery(_post_shape(), "left_post_body"), material=post_mat, name="post")
    left_post.visual(
        Box((CAP_W, CAP_D, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, POST_VISIBLE + (CAP_T / 2.0))),
        material=post_mat,
        name="cap",
    )

    right_post = model.part("right_post")
    right_post.visual(mesh_from_cadquery(_post_shape(), "right_post_body"), material=post_mat, name="post")
    right_post.visual(
        Box((CAP_W, CAP_D, CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, POST_VISIBLE + (CAP_T / 2.0))),
        material=post_mat,
        name="cap",
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_body_shape(), "tray_body"), material=tray_mat, name="body")
    tray.visual(
        Box((PAD_W, PAD_D, PAD_H)),
        origin=Origin(xyz=(0.0, 0.0, PAD_H / 2.0)),
        material=tray_mat,
        name="left_pad",
    )
    tray.visual(
        Box((PAD_W, PAD_D, PAD_H)),
        origin=Origin(xyz=(POST_SPACING, 0.0, PAD_H / 2.0)),
        material=tray_mat,
        name="right_pad",
    )

    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_post,
        origin=Origin(xyz=(LEFT_X, REAR_Y, FRAME_H + SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=LIFT_TRAVEL),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_post,
        origin=Origin(xyz=(RIGHT_X, REAR_Y, FRAME_H + SLEEVE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=LIFT_TRAVEL),
    )
    model.articulation(
        "left_post_to_tray",
        ArticulationType.FIXED,
        parent=left_post,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    left_post = object_model.get_part("left_post")
    right_post = object_model.get_part("right_post")
    tray = object_model.get_part("tray")
    left_lift = object_model.get_articulation("left_lift")
    right_lift = object_model.get_articulation("right_lift")

    ctx.expect_within(
        left_post,
        base,
        axes="xy",
        inner_elem="post",
        outer_elem="left_sleeve",
        margin=0.0005,
        name="left post stays centered inside left sleeve",
    )
    ctx.expect_within(
        right_post,
        base,
        axes="xy",
        inner_elem="post",
        outer_elem="right_sleeve",
        margin=0.0005,
        name="right post stays centered inside right sleeve",
    )
    ctx.expect_overlap(
        left_post,
        base,
        axes="z",
        elem_a="post",
        elem_b="left_sleeve",
        min_overlap=0.080,
        name="left post retains deep insertion when collapsed",
    )
    ctx.expect_overlap(
        right_post,
        base,
        axes="z",
        elem_a="post",
        elem_b="right_sleeve",
        min_overlap=0.080,
        name="right post retains deep insertion when collapsed",
    )
    ctx.expect_contact(
        left_post,
        tray,
        elem_a="cap",
        elem_b="left_pad",
        name="left column cap supports the tray at rest",
    )
    ctx.expect_contact(
        right_post,
        tray,
        elem_a="cap",
        elem_b="right_pad",
        name="right column cap supports the tray at rest",
    )

    rest_left = ctx.part_world_position(left_post)
    rest_right = ctx.part_world_position(right_post)
    rest_tray = ctx.part_world_position(tray)

    with ctx.pose({left_lift: LIFT_TRAVEL, right_lift: LIFT_TRAVEL}):
        ctx.expect_within(
            left_post,
            base,
            axes="xy",
            inner_elem="post",
            outer_elem="left_sleeve",
            margin=0.0005,
            name="left post stays centered inside left sleeve when extended",
        )
        ctx.expect_within(
            right_post,
            base,
            axes="xy",
            inner_elem="post",
            outer_elem="right_sleeve",
            margin=0.0005,
            name="right post stays centered inside right sleeve when extended",
        )
        ctx.expect_overlap(
            left_post,
            base,
            axes="z",
            elem_a="post",
            elem_b="left_sleeve",
            min_overlap=0.020,
            name="left post still retains insertion at max height",
        )
        ctx.expect_overlap(
            right_post,
            base,
            axes="z",
            elem_a="post",
            elem_b="right_sleeve",
            min_overlap=0.020,
            name="right post still retains insertion at max height",
        )
        ctx.expect_contact(
            left_post,
            tray,
            elem_a="cap",
            elem_b="left_pad",
            name="left column cap stays seated under the tray when extended",
        )
        ctx.expect_contact(
            right_post,
            tray,
            elem_a="cap",
            elem_b="right_pad",
            name="right column cap stays seated under the tray when extended",
        )

        extended_left = ctx.part_world_position(left_post)
        extended_right = ctx.part_world_position(right_post)
        extended_tray = ctx.part_world_position(tray)

    ctx.check(
        "matched columns rise together",
        rest_left is not None
        and rest_right is not None
        and extended_left is not None
        and extended_right is not None
        and abs(extended_left[2] - extended_right[2]) <= 1e-6
        and abs((extended_left[0] - extended_right[0]) - (rest_left[0] - rest_right[0])) <= 1e-6,
        details=f"rest_left={rest_left}, rest_right={rest_right}, extended_left={extended_left}, extended_right={extended_right}",
    )
    ctx.check(
        "tray rises with the telescoping columns",
        rest_tray is not None and extended_tray is not None and extended_tray[2] > rest_tray[2] + 0.050,
        details=f"rest_tray={rest_tray}, extended_tray={extended_tray}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
