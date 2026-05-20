from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.42
BODY_D = 0.34
BODY_H = 0.78
WALL_T = 0.018
FLOOR_T = 0.014
RIM_T = 0.014
RIM_H = 0.028

PEDAL_W = 0.30
PEDAL_D = 0.095
PEDAL_T = 0.012
PEDAL_TEXTURE_T = 0.003
PEDAL_REST_ANGLE = math.radians(32.0)
PEDAL_TRAVEL = math.radians(26.0)

LID_W = BODY_W + 0.028
LID_D = BODY_D + 0.010
LID_T = 0.006
LID_SKIRT_T = 0.006
LID_SKIRT_H = 0.028
LID_FRONT_LIP_D = 0.016
LID_FRONT_LIP_H = 0.032
LID_TRAVEL = math.radians(66.0)

COVER_W = 0.18
COVER_D = 0.086
COVER_T = 0.005
COVER_SKIRT_H = 0.026


def _rotate_y(x: float, z: float, angle: float) -> tuple[float, float]:
    return (
        x * math.cos(angle) + z * math.sin(angle),
        -x * math.sin(angle) + z * math.cos(angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_waste_bin")

    shell_white = model.material("shell_white", rgba=(0.93, 0.95, 0.94, 1.0))
    lid_red = model.material("lid_red", rgba=(0.78, 0.16, 0.16, 1.0))
    pedal_gray = model.material("pedal_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    cover_black = model.material("cover_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    wall_center_z = FLOOR_T + (BODY_H - FLOOR_T) / 2.0

    body.visual(
        Box((BODY_D - (2.0 * WALL_T), BODY_W - (2.0 * WALL_T), FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T / 2.0)),
        material=shell_white,
        name="floor",
    )
    body.visual(
        Box((BODY_D, WALL_T, BODY_H - FLOOR_T)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - WALL_T / 2.0, wall_center_z)),
        material=shell_white,
        name="side_wall_0",
    )
    body.visual(
        Box((BODY_D, WALL_T, BODY_H - FLOOR_T)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + WALL_T / 2.0, wall_center_z)),
        material=shell_white,
        name="side_wall_1",
    )
    body.visual(
        Box((WALL_T, BODY_W - (2.0 * WALL_T), BODY_H - FLOOR_T)),
        origin=Origin(xyz=(BODY_D / 2.0 - WALL_T / 2.0, 0.0, wall_center_z)),
        material=shell_white,
        name="front_wall",
    )
    body.visual(
        Box((WALL_T, BODY_W - (2.0 * WALL_T), BODY_H - FLOOR_T)),
        origin=Origin(xyz=(-BODY_D / 2.0 + WALL_T / 2.0, 0.0, wall_center_z)),
        material=shell_white,
        name="rear_wall",
    )

    rim_center_z = BODY_H - RIM_H / 2.0
    body.visual(
        Box((BODY_D - 0.020, RIM_T, RIM_H)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - RIM_T / 2.0, rim_center_z)),
        material=shell_white,
        name="side_rim_0",
    )
    body.visual(
        Box((BODY_D - 0.020, RIM_T, RIM_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + RIM_T / 2.0, rim_center_z)),
        material=shell_white,
        name="side_rim_1",
    )
    body.visual(
        Box((RIM_T, BODY_W - 0.028, RIM_H)),
        origin=Origin(xyz=(BODY_D / 2.0 - RIM_T / 2.0, 0.0, rim_center_z)),
        material=shell_white,
        name="front_rim",
    )
    body.visual(
        Box((0.020, PEDAL_W - 0.030, 0.020)),
        origin=Origin(xyz=(BODY_D / 2.0 - 0.009, 0.0, 0.105)),
        material=trim_dark,
        name="pedal_mount",
    )
    body.visual(
        Box((0.040, 0.160, 0.042)),
        origin=Origin(xyz=(-BODY_D / 2.0 - 0.020, 0.0, BODY_H + 0.012)),
        material=trim_dark,
        name="damper_base",
    )
    body.visual(
        Box((0.030, 0.140, 0.026)),
        origin=Origin(xyz=(-BODY_D / 2.0 - 0.054, 0.0, BODY_H + 0.024)),
        material=trim_dark,
        name="cover_mount",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_D, LID_W, LID_T)),
        origin=Origin(xyz=(0.178, 0.0, 0.005)),
        material=lid_red,
        name="lid_panel",
    )
    lid.visual(
        Box((LID_D - 0.020, LID_SKIRT_T, LID_SKIRT_H)),
        origin=Origin(
            xyz=(0.182, LID_W / 2.0 - LID_SKIRT_T / 2.0, -0.010),
        ),
        material=lid_red,
        name="lid_skirt_0",
    )
    lid.visual(
        Box((LID_D - 0.020, LID_SKIRT_T, LID_SKIRT_H)),
        origin=Origin(
            xyz=(0.182, -LID_W / 2.0 + LID_SKIRT_T / 2.0, -0.010),
        ),
        material=lid_red,
        name="lid_skirt_1",
    )
    lid.visual(
        Box((LID_FRONT_LIP_D, LID_W - 0.012, LID_FRONT_LIP_H)),
        origin=Origin(xyz=(0.353, 0.0, -0.011)),
        material=lid_red,
        name="lid_front_lip",
    )
    lid.visual(
        Box((0.040, LID_W - 0.060, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, -0.004)),
        material=lid_red,
        name="lid_rear_rib",
    )
    for index, y_pos in enumerate((-0.135, 0.135)):
        lid.visual(
            Cylinder(radius=0.008, length=0.085),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim_dark,
            name=f"lid_barrel_{index}",
        )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.009, length=PEDAL_W - 0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="pedal_pivot",
    )
    pad_x, pad_z = _rotate_y(PEDAL_D / 2.0, -PEDAL_T / 2.0, PEDAL_REST_ANGLE)
    pedal.visual(
        Box((PEDAL_D, PEDAL_W, PEDAL_T)),
        origin=Origin(xyz=(pad_x, 0.0, pad_z), rpy=(0.0, PEDAL_REST_ANGLE, 0.0)),
        material=pedal_gray,
        name="pedal_pad",
    )
    web_x, web_z = _rotate_y(0.020, -0.017, PEDAL_REST_ANGLE)
    pedal.visual(
        Box((0.032, PEDAL_W - 0.020, 0.014)),
        origin=Origin(xyz=(web_x, 0.0, web_z), rpy=(0.0, PEDAL_REST_ANGLE, 0.0)),
        material=pedal_gray,
        name="pedal_web",
    )
    for index, local_x in enumerate((0.018, 0.036, 0.054, 0.072)):
        rib_x, rib_z = _rotate_y(local_x, PEDAL_TEXTURE_T / 2.0, PEDAL_REST_ANGLE)
        pedal.visual(
            Box((0.010, PEDAL_W - 0.050, PEDAL_TEXTURE_T)),
            origin=Origin(
                xyz=(rib_x, 0.0, rib_z),
                rpy=(0.0, PEDAL_REST_ANGLE, 0.0),
            ),
            material=trim_dark,
            name=f"pedal_rib_{index}",
        )

    damper_cover = model.part("damper_cover")
    damper_cover.visual(
        Box((COVER_D, COVER_W, COVER_T)),
        origin=Origin(xyz=(0.043, 0.0, 0.008)),
        material=cover_black,
        name="cover_panel",
    )
    damper_cover.visual(
        Box((0.028, COVER_W - 0.020, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.001)),
        material=cover_black,
        name="cover_rear_bridge",
    )
    damper_cover.visual(
        Box((COVER_D - 0.016, 0.006, COVER_SKIRT_H)),
        origin=Origin(xyz=(0.045, COVER_W / 2.0 - 0.003, -0.006)),
        material=cover_black,
        name="cover_skirt_0",
    )
    damper_cover.visual(
        Box((COVER_D - 0.016, 0.006, COVER_SKIRT_H)),
        origin=Origin(xyz=(0.045, -COVER_W / 2.0 + 0.003, -0.006)),
        material=cover_black,
        name="cover_skirt_1",
    )
    damper_cover.visual(
        Box((0.012, COVER_W - 0.008, 0.028)),
        origin=Origin(xyz=(0.080, 0.0, -0.008)),
        material=cover_black,
        name="cover_front",
    )
    damper_cover.visual(
        Cylinder(radius=0.006, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="cover_barrel",
    )

    pedal_joint = model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(BODY_D / 2.0 + 0.010, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=0.0,
            upper=PEDAL_TRAVEL,
        ),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-BODY_D / 2.0 - 0.006, 0.0, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
        mimic=Mimic(joint=pedal_joint.name, multiplier=LID_TRAVEL / PEDAL_TRAVEL),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(-BODY_D / 2.0 - 0.064, 0.0, BODY_H + 0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    damper_cover = object_model.get_part("damper_cover")

    pedal_joint = object_model.get_articulation("pedal_pivot")
    cover_joint = object_model.get_articulation("cover_hinge")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_rim",
        max_gap=0.012,
        max_penetration=0.0,
        name="lid closes near the front rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.30,
        name="lid covers the bin opening footprint",
    )
    ctx.expect_overlap(
        pedal,
        body,
        axes="y",
        elem_a="pedal_pad",
        min_overlap=0.28,
        name="pedal spans most of the bin width",
    )
    ctx.expect_origin_gap(
        pedal,
        body,
        axis="x",
        min_gap=0.16,
        name="pedal sits out at the front of the bin",
    )
    ctx.expect_overlap(
        damper_cover,
        body,
        axes="y",
        elem_a="cover_panel",
        elem_b="damper_base",
        min_overlap=0.15,
        name="damper cover caps the rear mechanism",
    )
    ctx.expect_gap(
        damper_cover,
        body,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="damper_base",
        max_gap=0.020,
        max_penetration=0.0,
        name="damper cover sits close over its base",
    )

    pedal_upper = pedal_joint.motion_limits.upper if pedal_joint.motion_limits is not None else None
    cover_upper = cover_joint.motion_limits.upper if cover_joint.motion_limits is not None else None

    lid_front_closed = ctx.part_element_world_aabb(lid, elem="lid_front_lip")
    pedal_closed = ctx.part_element_world_aabb(pedal, elem="pedal_pad")
    if pedal_upper is not None:
        with ctx.pose({pedal_joint: pedal_upper}):
            lid_front_open = ctx.part_element_world_aabb(lid, elem="lid_front_lip")
            pedal_open = ctx.part_element_world_aabb(pedal, elem="pedal_pad")

        ctx.check(
            "pedal press raises the front of the lid",
            lid_front_closed is not None
            and lid_front_open is not None
            and lid_front_open[0][2] > lid_front_closed[0][2] + 0.14,
            details=f"closed={lid_front_closed}, open={lid_front_open}",
        )
        ctx.check(
            "pedal press drops the toe edge",
            pedal_closed is not None
            and pedal_open is not None
            and pedal_open[0][2] < pedal_closed[0][2] - 0.015,
            details=f"closed={pedal_closed}, open={pedal_open}",
        )

    cover_closed = ctx.part_element_world_aabb(damper_cover, elem="cover_front")
    if cover_upper is not None:
        with ctx.pose({cover_joint: cover_upper}):
            cover_open = ctx.part_element_world_aabb(damper_cover, elem="cover_front")

        ctx.check(
            "damper cover flips upward from the rear hinge",
            cover_closed is not None
            and cover_open is not None
            and cover_open[0][2] > cover_closed[0][2] + 0.025,
            details=f"closed={cover_closed}, open={cover_open}",
        )

    return ctx.report()


object_model = build_object_model()
