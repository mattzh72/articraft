from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_W = 0.236
OUTER_H = 0.305
FRAME_W = 0.012
FRAME_D = 0.014
BOARD_T = 0.004

CLAP_W = 0.240
CLAP_D = 0.012
CLAP_H = 0.014
CLAP_OPEN = 1.18
HINGE_R = 0.003

CLIP_T = 0.003
CLIP_W = 0.018
CLIP_H = 0.075
CLIP_OUTREACH = 0.016
CLIP_FINGER_T = 0.0025
CLIP_BRIDGE_H = 0.010
CLIP_Z = 0.155


def add_stripe_series(part, *, width: float, depth: float, height: float, z: float, material) -> None:
    stripe_angle = math.radians(34.0)
    stripe_size = (0.018, 0.026, height)
    centers = (-0.086, -0.043, 0.0, 0.043, 0.086)
    for index, x_center in enumerate(centers):
        part.visual(
            Box(stripe_size),
            origin=Origin(
                xyz=(x_center, depth * 0.54, z),
                rpy=(0.0, 0.0, stripe_angle),
            ),
            material=material,
            name=f"stripe_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lineup_clapperboard")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.08, 1.0))
    board_white = model.material("board_white", rgba=(0.95, 0.96, 0.94, 1.0))
    paint_white = model.material("paint_white", rgba=(0.99, 0.99, 0.99, 1.0))
    metal = model.material("clip_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber_strip", rgba=(0.14, 0.15, 0.16, 1.0))

    slate = model.part("slate")
    slate.visual(
        Box((FRAME_W, FRAME_D, OUTER_H)),
        origin=Origin(xyz=(-OUTER_W / 2 + FRAME_W / 2, 0.0, OUTER_H / 2)),
        material=matte_black,
        name="left_rail",
    )
    slate.visual(
        Box((FRAME_W, FRAME_D, OUTER_H)),
        origin=Origin(xyz=(OUTER_W / 2 - FRAME_W / 2, 0.0, OUTER_H / 2)),
        material=matte_black,
        name="right_rail",
    )
    slate.visual(
        Box((OUTER_W, FRAME_D, FRAME_W)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_W / 2)),
        material=matte_black,
        name="bottom_rail",
    )
    slate.visual(
        Box((OUTER_W, FRAME_D, FRAME_W)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_H - FRAME_W / 2)),
        material=matte_black,
        name="top_rail",
    )
    slate.visual(
        Box((OUTER_W - 2 * (FRAME_W - 0.004), BOARD_T, OUTER_H - 2 * (FRAME_W - 0.004))),
        origin=Origin(xyz=(0.0, -0.001, OUTER_H / 2)),
        material=board_white,
        name="writing_board",
    )
    slate.visual(
        Box((OUTER_W - 2 * FRAME_W + 0.010, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0045, OUTER_H - FRAME_W - 0.014)),
        material=rubber,
        name="title_strip",
    )
    for index, x_center in enumerate((-0.080, 0.0, 0.080)):
        slate.visual(
            Box((0.014, 0.004, 0.006)),
            origin=Origin(xyz=(x_center, -0.003, OUTER_H + HINGE_R)),
            material=matte_black,
            name=f"hinge_tab_{index}",
        )
        slate.visual(
            Cylinder(radius=HINGE_R, length=0.040),
            origin=Origin(
                xyz=(x_center, 0.0, OUTER_H + HINGE_R),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=matte_black,
            name=f"hinge_knuckle_{index}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((CLAP_W, CLAP_D, CLAP_H)),
        origin=Origin(xyz=(0.0, CLAP_D / 2, CLAP_H / 2 - HINGE_R)),
        material=matte_black,
        name="body",
    )
    clapstick.visual(
        Box((CLAP_W - 0.006, CLAP_D * 0.55, 0.0022)),
        origin=Origin(xyz=(0.0, CLAP_D * 0.46, 0.0011)),
        material=rubber,
        name="strike_pad",
    )
    clapstick.visual(
        Box((CLAP_W - 0.008, 0.0024, 0.004)),
        origin=Origin(xyz=(0.0, CLAP_D - 0.0012, 0.002)),
        material=matte_black,
        name="front_edge",
    )
    for index, x_center in enumerate((-0.040, 0.040)):
        clapstick.visual(
            Cylinder(radius=HINGE_R, length=0.032),
            origin=Origin(
                xyz=(x_center, 0.0, 0.0),
                rpy=(0.0, math.pi / 2, 0.0),
            ),
            material=matte_black,
            name=f"hinge_sleeve_{index}",
        )
    add_stripe_series(
        clapstick,
        width=CLAP_W,
        depth=CLAP_D,
        height=0.002,
        z=CLAP_H - HINGE_R - 0.001,
        material=paint_white,
    )

    marker_clip = model.part("marker_clip")
    marker_clip.visual(
        Box((CLIP_T, CLIP_W, CLIP_H)),
        origin=Origin(xyz=(CLIP_T / 2, 0.0, 0.0)),
        material=metal,
        name="spine",
    )
    marker_clip.visual(
        Box((CLIP_OUTREACH, CLIP_W, CLIP_BRIDGE_H)),
        origin=Origin(
            xyz=(CLIP_T + CLIP_OUTREACH / 2, 0.0, CLIP_H / 2 - CLIP_BRIDGE_H / 2),
        ),
        material=metal,
        name="upper_bridge",
    )
    marker_clip.visual(
        Box((CLIP_OUTREACH, CLIP_W, CLIP_BRIDGE_H)),
        origin=Origin(
            xyz=(CLIP_T + CLIP_OUTREACH / 2, 0.0, -CLIP_H / 2 + CLIP_BRIDGE_H / 2),
        ),
        material=metal,
        name="lower_bridge",
    )
    marker_clip.visual(
        Box((CLIP_FINGER_T, CLIP_W * 0.44, CLIP_H - 0.018)),
        origin=Origin(xyz=(CLIP_T + CLIP_OUTREACH, 0.0, 0.0)),
        material=metal,
        name="outer_finger",
    )

    model.articulation(
        "slate_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.0, OUTER_H + HINGE_R)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=CLAP_OPEN),
    )
    model.articulation(
        "slate_to_marker_clip",
        ArticulationType.FIXED,
        parent=slate,
        child=marker_clip,
        origin=Origin(xyz=(OUTER_W / 2, 0.0, CLIP_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slate = object_model.get_part("slate")
    clapstick = object_model.get_part("clapstick")
    marker_clip = object_model.get_part("marker_clip")
    hinge = object_model.get_articulation("slate_to_clapstick")
    limits = hinge.motion_limits

    ctx.expect_gap(
        clapstick,
        slate,
        axis="z",
        positive_elem="body",
        negative_elem="top_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="clapstick seats on the top rail when closed",
    )
    ctx.expect_overlap(
        clapstick,
        slate,
        axes="x",
        elem_a="body",
        elem_b="top_rail",
        min_overlap=0.225,
        name="clapstick spans the slate width",
    )
    ctx.expect_contact(
        marker_clip,
        slate,
        elem_a="spine",
        elem_b="right_rail",
        name="marker clip is mounted to the side frame",
    )
    ctx.expect_gap(
        marker_clip,
        slate,
        axis="x",
        positive_elem="spine",
        negative_elem="right_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="marker clip sits on the right side edge",
    )

    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="front_edge")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapstick, elem="front_edge")
            ctx.expect_overlap(
                clapstick,
                slate,
                axes="x",
                elem_a="body",
                elem_b="top_rail",
                min_overlap=0.225,
                name="opened clapstick stays aligned across the top edge",
            )
        ctx.check(
            "clapstick opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.008,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
