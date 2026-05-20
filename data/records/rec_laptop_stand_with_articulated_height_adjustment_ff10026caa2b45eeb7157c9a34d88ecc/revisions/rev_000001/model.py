from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_laptop_stand")

    powder_black = model.material("powder_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    silver = model.material("silver", rgba=(0.68, 0.70, 0.73, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.330, 0.230, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=powder_black,
        name="ballast",
    )
    base.visual(
        Box((0.250, 0.170, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.021)),
        material=graphite,
        name="top_skin",
    )

    column = model.part("column")
    column.visual(
        Box((0.090, 0.110, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=powder_black,
        name="pedestal",
    )
    column.visual(
        Box((0.006, 0.044, 0.190)),
        origin=Origin(xyz=(-0.020, 0.0, 0.125)),
        material=graphite,
        name="rear_wall",
    )
    column.visual(
        Box((0.006, 0.044, 0.190)),
        origin=Origin(xyz=(0.020, 0.0, 0.125)),
        material=graphite,
        name="front_wall",
    )
    column.visual(
        Box((0.040, 0.006, 0.190)),
        origin=Origin(xyz=(0.0, -0.017, 0.125)),
        material=graphite,
        name="side_wall_0",
    )
    column.visual(
        Box((0.040, 0.006, 0.190)),
        origin=Origin(xyz=(0.0, 0.017, 0.125)),
        material=graphite,
        name="side_wall_1",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.034, 0.028, 0.510)),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=silver,
        name="inner_mast",
    )
    mast.visual(
        Box((0.120, 0.024, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, 0.304)),
        material=powder_black,
        name="forward_arm",
    )
    mast.visual(
        Box((0.028, 0.090, 0.020)),
        origin=Origin(xyz=(0.116, 0.0, 0.286)),
        material=powder_black,
        name="head",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.290, 0.300, 0.006)),
        origin=Origin(xyz=(0.145, 0.0, 0.004)),
        material=silver,
        name="deck",
    )
    tray.visual(
        Box((0.250, 0.010, 0.016)),
        origin=Origin(xyz=(0.145, -0.145, 0.015)),
        material=graphite,
        name="side_rail_0",
    )
    tray.visual(
        Box((0.250, 0.010, 0.016)),
        origin=Origin(xyz=(0.145, 0.145, 0.015)),
        material=graphite,
        name="side_rail_1",
    )
    tray.visual(
        Box((0.022, 0.050, 0.016)),
        origin=Origin(xyz=(0.279, -0.090, 0.015)),
        material=powder_black,
        name="lip_0",
    )
    tray.visual(
        Box((0.022, 0.050, 0.016)),
        origin=Origin(xyz=(0.279, 0.090, 0.015)),
        material=powder_black,
        name="lip_1",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(-0.090, 0.0, 0.026)),
    )
    model.articulation(
        "column_to_mast",
        ArticulationType.PRISMATIC,
        parent=column,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "mast_to_tray",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=tray,
        origin=Origin(xyz=(0.116, 0.0, 0.297)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    mast = object_model.get_part("mast")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("column_to_mast")
    tilt = object_model.get_articulation("mast_to_tray")

    ctx.expect_contact(
        column,
        base,
        elem_a="pedestal",
        elem_b="top_skin",
        name="column pedestal seats on the weighted base",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.250,
        name="tray is carried above the base rather than resting on it",
    )
    ctx.expect_origin_distance(
        mast,
        column,
        axes="xy",
        max_dist=0.001,
        name="mast stays centered in the telescoping sleeve at rest",
    )
    ctx.expect_overlap(
        mast,
        column,
        axes="z",
        min_overlap=0.180,
        name="collapsed mast remains deeply inserted in the sleeve",
    )
    ctx.expect_gap(
        tray,
        mast,
        axis="z",
        positive_elem="deck",
        negative_elem="head",
        min_gap=0.0005,
        max_gap=0.003,
        name="tray deck clears the tilt head with a small realistic gap",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    rest_lip_aabb = ctx.part_element_world_aabb(tray, elem="lip_0")

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_origin_distance(
                mast,
                column,
                axes="xy",
                max_dist=0.001,
                name="extended mast stays centered in the sleeve",
            )
            ctx.expect_overlap(
                mast,
                column,
                axes="z",
                min_overlap=0.025,
                name="extended mast still retains insertion in the sleeve",
            )
            extended_mast_pos = ctx.part_world_position(mast)
        ctx.check(
            "mast extends upward",
            rest_mast_pos is not None
            and extended_mast_pos is not None
            and extended_mast_pos[2] > rest_mast_pos[2] + 0.12,
            details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.upper}):
            raised_lip_aabb = ctx.part_element_world_aabb(tray, elem="lip_0")
        ctx.check(
            "tray tilts upward from the head",
            rest_lip_aabb is not None
            and raised_lip_aabb is not None
            and raised_lip_aabb[0][2] > rest_lip_aabb[0][2] + 0.08,
            details=f"rest={rest_lip_aabb}, raised={raised_lip_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
