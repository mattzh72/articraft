from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.42
BODY_D = 0.24
BODY_H = 0.17
WALL_T = 0.006
FLOOR_T = 0.007
INNER_W = BODY_W - 2.0 * WALL_T
INNER_D = BODY_D - 2.0 * WALL_T
INTERLOCK = 0.001

FRAME_LIP_W = 0.016
FRAME_LIP_T = 0.004
FRAME_LIP_Z = BODY_H - 0.010

TRAY_BOTTOM_Z = 0.075
TRAY_FLOOR_T = 0.004
TRAY_W = INNER_W - 0.024
TRAY_D = INNER_D - 0.032
TRAY_DIVIDER_T = 0.004
TRAY_DIVIDER_H = 0.022

LID_OVERHANG = 0.0015
LID_W = BODY_W + 2.0 * LID_OVERHANG
LID_D = BODY_D + 2.0 * LID_OVERHANG
LID_TOP_T = 0.005
LID_SKIRT_T = 0.0035
LID_SKIRT_DROP = 0.020
LID_OPEN_ANGLE = 1.95


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_mat = model.material("body_plastic", rgba=(0.30, 0.34, 0.22, 1.0))
    lid_mat = model.material("lid_plastic", rgba=(0.34, 0.39, 0.26, 1.0))
    tray_mat = model.material("tray_insert", rgba=(0.76, 0.72, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T / 2.0)),
        material=body_mat,
        name="floor",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-(BODY_W - WALL_T) / 2.0, 0.0, BODY_H / 2.0)),
        material=body_mat,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=((BODY_W - WALL_T) / 2.0, 0.0, BODY_H / 2.0)),
        material=body_mat,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W, WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, (BODY_D - WALL_T) / 2.0, BODY_H / 2.0)),
        material=body_mat,
        name="front_wall",
    )
    body.visual(
        Box((BODY_W, WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, -(BODY_D - WALL_T) / 2.0, BODY_H / 2.0)),
        material=body_mat,
        name="rear_wall",
    )
    body.visual(
        Box((FRAME_LIP_W + INTERLOCK, INNER_D - 0.020, FRAME_LIP_T)),
        origin=Origin(
            xyz=(
                -INNER_W / 2.0 + FRAME_LIP_W / 2.0 - INTERLOCK / 2.0,
                0.010,
                FRAME_LIP_Z,
            )
        ),
        material=body_mat,
        name="left_stop_lip",
    )
    body.visual(
        Box((FRAME_LIP_W + INTERLOCK, INNER_D - 0.020, FRAME_LIP_T)),
        origin=Origin(
            xyz=(
                INNER_W / 2.0 - FRAME_LIP_W / 2.0 + INTERLOCK / 2.0,
                0.010,
                FRAME_LIP_Z,
            )
        ),
        material=body_mat,
        name="right_stop_lip",
    )
    body.visual(
        Box((INNER_W - 2.0 * FRAME_LIP_W, FRAME_LIP_W + INTERLOCK, FRAME_LIP_T)),
        origin=Origin(
            xyz=(
                0.0,
                INNER_D / 2.0 - FRAME_LIP_W / 2.0 + INTERLOCK / 2.0,
                FRAME_LIP_Z,
            )
        ),
        material=body_mat,
        name="front_stop_lip",
    )
    body.visual(
        Box((0.012 + INTERLOCK, TRAY_D, 0.006)),
        origin=Origin(
            xyz=(
                -INNER_W / 2.0 + 0.006 - INTERLOCK / 2.0,
                0.0,
                TRAY_BOTTOM_Z - 0.003,
            )
        ),
        material=body_mat,
        name="left_tray_rail",
    )
    body.visual(
        Box((0.012 + INTERLOCK, TRAY_D, 0.006)),
        origin=Origin(
            xyz=(
                INNER_W / 2.0 - 0.006 + INTERLOCK / 2.0,
                0.0,
                TRAY_BOTTOM_Z - 0.003,
            )
        ),
        material=body_mat,
        name="right_tray_rail",
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_FLOOR_T / 2.0)),
        material=tray_mat,
        name="tray_floor",
    )
    tray.visual(
        Box((TRAY_DIVIDER_T, TRAY_D, TRAY_DIVIDER_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=tray_mat,
        name="center_divider",
    )
    tray.visual(
        Box((TRAY_W / 2.0, TRAY_DIVIDER_T, TRAY_DIVIDER_H)),
        origin=Origin(xyz=(TRAY_W / 4.0, -TRAY_D / 4.0, 0.014)),
        material=tray_mat,
        name="front_bin_divider",
    )
    tray.visual(
        Box((TRAY_W / 2.0, TRAY_DIVIDER_T, TRAY_DIVIDER_H)),
        origin=Origin(xyz=(TRAY_W / 4.0, TRAY_D / 4.0, 0.014)),
        material=tray_mat,
        name="rear_bin_divider",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_TOP_T)),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_TOP_T / 2.0)),
        material=lid_mat,
        name="lid_top",
    )
    lid.visual(
        Box((LID_SKIRT_T, LID_D - 0.006, LID_SKIRT_DROP + 0.0005)),
        origin=Origin(
            xyz=(
                -(LID_W - LID_SKIRT_T) / 2.0,
                LID_D / 2.0,
                -(LID_SKIRT_DROP / 2.0) + 0.00025,
            )
        ),
        material=lid_mat,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((LID_SKIRT_T, LID_D - 0.006, LID_SKIRT_DROP + 0.0005)),
        origin=Origin(
            xyz=(
                (LID_W - LID_SKIRT_T) / 2.0,
                LID_D / 2.0,
                -(LID_SKIRT_DROP / 2.0) + 0.00025,
            )
        ),
        material=lid_mat,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((LID_W, LID_SKIRT_T, LID_SKIRT_DROP + 0.0005)),
        origin=Origin(
            xyz=(
                0.0,
                LID_D - LID_SKIRT_T / 2.0,
                -(LID_SKIRT_DROP / 2.0) + 0.00025,
            )
        ),
        material=lid_mat,
        name="lid_front_skirt",
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.FIXED,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_BOTTOM_Z)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -LID_D / 2.0, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=2.05,
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
    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_within(
        tray,
        body,
        axes="xy",
        margin=0.0,
        name="tray stays within the body footprint",
    )
    ctx.expect_gap(
        tray,
        body,
        axis="z",
        min_gap=0.060,
        max_gap=0.075,
        positive_elem="tray_floor",
        negative_elem="floor",
        name="tray sits above the box floor on support rails",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lid_top",
        name="lid closes flush on the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.220,
        elem_a="lid_top",
        name="lid covers the body opening when closed",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: LID_OPEN_ANGLE}):
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.060,
            positive_elem="lid_front_skirt",
            name="opened lid front edge rises above the body",
        )

    ctx.check(
        "lid swings upward about the rear hinge",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.100,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
