from __future__ import annotations

import math

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


FRAME_WIDTH = 0.22
FRAME_THICKNESS = 0.014
FRAME_HEIGHT = 0.30
FRAME_OPENING_WIDTH = 0.13
FRAME_OPENING_HEIGHT = 0.236
FRAME_OPENING_BOTTOM = 0.036
SLOT_X = 0.081
SLOT_WIDTH = 0.011
SLOT_HEIGHT = 0.150
SLOT_BOTTOM = 0.070
HINGE_Z = 0.292
HINGE_Y = -0.017
LEG_OPEN_ANGLE = 0.48
LEDGE_REST_Z = 0.110
LEDGE_TRAVEL = 0.082


def _frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(
        FRAME_WIDTH,
        FRAME_THICKNESS,
        FRAME_HEIGHT,
        centered=(True, True, False),
    )

    opening = cq.Workplane("XY").box(
        FRAME_OPENING_WIDTH,
        FRAME_THICKNESS + 0.004,
        FRAME_OPENING_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, FRAME_OPENING_BOTTOM))
    frame = frame.cut(opening)

    for x_pos in (-SLOT_X, SLOT_X):
        slot = cq.Workplane("XY").box(
            SLOT_WIDTH,
            FRAME_THICKNESS + 0.006,
            SLOT_HEIGHT,
            centered=(True, True, False),
        ).translate((x_pos, 0.0, SLOT_BOTTOM))
        frame = frame.cut(slot)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_easel")

    beech = model.material("beech", rgba=(0.77, 0.66, 0.49, 1.0))
    walnut = model.material("walnut", rgba=(0.42, 0.28, 0.18, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.61, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    main_frame = model.part("main_frame")
    main_frame.visual(
        mesh_from_cadquery(_frame_shape(), "easel_frame"),
        material=beech,
        name="frame_panel",
    )
    main_frame.visual(
        Box((0.028, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.011, HINGE_Z - 0.001)),
        material=beech,
        name="hinge_boss",
    )
    main_frame.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(
            xyz=(0.0, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="hinge_barrel",
    )
    for index, x_pos in enumerate((-0.082, 0.082)):
        main_frame.visual(
            Box((0.026, 0.010, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, 0.002)),
            material=rubber,
            name=f"foot_{index}",
        )

    rear_leg = model.part("rear_leg")
    for index, x_pos in enumerate((-0.026, 0.026)):
        rear_leg.visual(
            Cylinder(radius=0.0055, length=0.018),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"leg_barrel_{index}",
        )
    rear_leg.visual(
        Box((0.062, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, -0.011, -0.010)),
        material=walnut,
        name="upper_link",
    )
    rear_leg.visual(
        Box((0.020, 0.008, 0.308)),
        origin=Origin(xyz=(0.0, -0.012, -0.166)),
        material=walnut,
        name="leg_bar",
    )
    rear_leg.visual(
        Box((0.034, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.015, -0.322)),
        material=rubber,
        name="leg_foot",
    )

    model.articulation(
        "frame_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=LEG_OPEN_ANGLE,
        ),
    )

    front_ledge = model.part("front_ledge")
    for index, x_pos in enumerate((-SLOT_X, SLOT_X)):
        front_ledge.visual(
            Cylinder(radius=0.0036, length=0.026),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"guide_post_{index}",
        )
        front_ledge.visual(
            Box((0.020, 0.010, 0.032)),
            origin=Origin(xyz=(x_pos, 0.012, 0.0)),
            material=beech,
            name=f"front_pad_{index}",
        )
        front_ledge.visual(
            Box((0.020, 0.010, 0.032)),
            origin=Origin(xyz=(x_pos, -0.012, 0.0)),
            material=beech,
            name=f"rear_pad_{index}",
        )
        front_ledge.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(x_pos, 0.019, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"thumbscrew_{index}",
        )

    for index, x_pos in enumerate((-0.068, 0.068)):
        front_ledge.visual(
            Box((0.016, 0.010, 0.040)),
            origin=Origin(xyz=(x_pos, 0.016, -0.018)),
            material=beech,
            name=f"support_arm_{index}",
        )

    front_ledge.visual(
        Box((0.172, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.021, -0.034)),
        material=beech,
        name="tray_back",
    )
    front_ledge.visual(
        Box((0.172, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.031, -0.041)),
        material=beech,
        name="shelf",
    )
    front_ledge.visual(
        Box((0.172, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.046, -0.035)),
        material=beech,
        name="shelf_lip",
    )

    model.articulation(
        "frame_to_ledge",
        ArticulationType.PRISMATIC,
        parent=main_frame,
        child=front_ledge,
        origin=Origin(xyz=(0.0, 0.0, LEDGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=LEDGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("main_frame")
    rear_leg = object_model.get_part("rear_leg")
    ledge = object_model.get_part("front_ledge")
    leg_joint = object_model.get_articulation("frame_to_rear_leg")
    ledge_joint = object_model.get_articulation("frame_to_ledge")

    for pose_name, ledge_q in (("lower", 0.0), ("raised", LEDGE_TRAVEL)):
        with ctx.pose({ledge_joint: ledge_q}):
            for elem_name in ("front_pad_0", "front_pad_1", "rear_pad_0", "rear_pad_1"):
                ctx.expect_within(
                    ledge,
                    frame,
                    axes="x",
                    inner_elem=elem_name,
                    margin=0.0,
                    name=f"{pose_name} {elem_name} stays within frame width",
                )
                ctx.expect_overlap(
                    ledge,
                    frame,
                    axes="z",
                    elem_a=elem_name,
                    min_overlap=0.02,
                    name=f"{pose_name} {elem_name} stays within slot height zone",
                )

    lower_ledge_pos = ctx.part_world_position(ledge)
    with ctx.pose({ledge_joint: LEDGE_TRAVEL}):
        raised_ledge_pos = ctx.part_world_position(ledge)
    ctx.check(
        "ledge rises along the frame",
        lower_ledge_pos is not None
        and raised_ledge_pos is not None
        and raised_ledge_pos[2] > lower_ledge_pos[2] + 0.07,
        details=f"lower={lower_ledge_pos}, raised={raised_ledge_pos}",
    )

    ctx.expect_gap(
        frame,
        rear_leg,
        axis="y",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem="frame_panel",
        negative_elem="leg_bar",
        name="folded rear leg nests close behind frame",
    )

    folded_leg_aabb = ctx.part_world_aabb(rear_leg)
    with ctx.pose({leg_joint: LEG_OPEN_ANGLE}):
        deployed_leg_aabb = ctx.part_world_aabb(rear_leg)

    ctx.check(
        "rear leg swings farther back when opened",
        deployed_leg_aabb is not None
        and folded_leg_aabb is not None
        and deployed_leg_aabb[0][1] < folded_leg_aabb[0][1] - 0.045,
        details=f"deployed={deployed_leg_aabb}, folded={folded_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
