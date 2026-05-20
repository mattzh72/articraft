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


BASE_LENGTH = 0.325
BASE_WIDTH = 0.215
BASE_BOTTOM = 0.010
BASE_WALL = 0.008
BASE_CAVITY = 0.040
DECK_THICKNESS = 0.006
DECK_TOP = BASE_BOTTOM + BASE_CAVITY + DECK_THICKNESS

HINGE_X = 0.130
HINGE_Z = 0.108
GUIDE_REST_X = -0.132
GUIDE_Z = 0.078
DOOR_HINGE_X = -(BASE_LENGTH / 2.0) + BASE_WALL
DOOR_HINGE_Z = BASE_BOTTOM


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_paper_punch")

    body = model.material("body", rgba=(0.28, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    door_finish = model.material("door_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_BOTTOM)),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM / 2.0)),
        material=body,
        name="base_floor",
    )
    base.visual(
        Box((BASE_LENGTH, BASE_WALL, BASE_CAVITY)),
        origin=Origin(
            xyz=(0.0, -(BASE_WIDTH / 2.0) + (BASE_WALL / 2.0), BASE_BOTTOM + (BASE_CAVITY / 2.0))
        ),
        material=body,
        name="side_wall_0",
    )
    base.visual(
        Box((BASE_LENGTH, BASE_WALL, BASE_CAVITY)),
        origin=Origin(
            xyz=(0.0, (BASE_WIDTH / 2.0) - (BASE_WALL / 2.0), BASE_BOTTOM + (BASE_CAVITY / 2.0))
        ),
        material=body,
        name="side_wall_1",
    )
    base.visual(
        Box((BASE_WALL, BASE_WIDTH, BASE_CAVITY)),
        origin=Origin(xyz=((BASE_LENGTH / 2.0) - (BASE_WALL / 2.0), 0.0, BASE_BOTTOM + (BASE_CAVITY / 2.0))),
        material=body,
        name="rear_wall",
    )
    base.visual(
        Box((BASE_WALL, 0.034, BASE_CAVITY)),
        origin=Origin(
            xyz=(
                -(BASE_LENGTH / 2.0) + (BASE_WALL / 2.0),
                -0.084,
                BASE_BOTTOM + (BASE_CAVITY / 2.0),
            )
        ),
        material=body,
        name="door_jamb_0",
    )
    base.visual(
        Box((BASE_WALL, 0.034, BASE_CAVITY)),
        origin=Origin(
            xyz=(
                -(BASE_LENGTH / 2.0) + (BASE_WALL / 2.0),
                0.084,
                BASE_BOTTOM + (BASE_CAVITY / 2.0),
            )
        ),
        material=body,
        name="door_jamb_1",
    )
    base.visual(
        Box((BASE_WALL, 0.147, 0.010)),
        origin=Origin(xyz=(-(BASE_LENGTH / 2.0) + (BASE_WALL / 2.0), 0.0, 0.045)),
        material=body,
        name="door_header",
    )
    base.visual(
        Box((BASE_LENGTH - (2.0 * BASE_WALL), BASE_WIDTH - (2.0 * BASE_WALL), DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM + BASE_CAVITY + (DECK_THICKNESS / 2.0))),
        material=body,
        name="top_deck",
    )
    base.visual(
        Box((0.020, BASE_WIDTH, 0.018)),
        origin=Origin(xyz=(-0.148, 0.0, 0.065)),
        material=body,
        name="front_fence",
    )
    base.visual(
        Box((0.035, 0.070, 0.020)),
        origin=Origin(xyz=(0.055, 0.0, 0.066)),
        material=body,
        name="punch_nose",
    )
    base.visual(
        Box((0.095, 0.105, 0.032)),
        origin=Origin(xyz=(0.095, 0.0, 0.072)),
        material=body,
        name="punch_head",
    )
    base.visual(
        Box((0.055, 0.032, 0.046)),
        origin=Origin(xyz=(0.124, -0.086, 0.079)),
        material=body,
        name="hinge_cheek_0",
    )
    base.visual(
        Box((0.055, 0.032, 0.046)),
        origin=Origin(xyz=(0.124, 0.086, 0.079)),
        material=body,
        name="hinge_cheek_1",
    )
    base.visual(
        Box((0.040, 0.140, 0.016)),
        origin=Origin(xyz=(0.120, 0.0, 0.094)),
        material=body,
        name="hinge_bridge",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, -0.086, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_finish,
        name="handle_knuckle_0",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.086, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_finish,
        name="handle_knuckle_1",
    )
    handle.visual(
        Box((0.146, 0.018, 0.016)),
        origin=Origin(xyz=(-0.073, -0.086, 0.0)),
        material=handle_finish,
        name="handle_leg_0",
    )
    handle.visual(
        Box((0.146, 0.018, 0.016)),
        origin=Origin(xyz=(-0.073, 0.086, 0.0)),
        material=handle_finish,
        name="handle_leg_1",
    )
    handle.visual(
        Box((0.022, 0.194, 0.028)),
        origin=Origin(xyz=(-0.146, 0.0, 0.0)),
        material=handle_finish,
        name="front_grip",
    )
    handle.visual(
        Box((0.018, 0.160, 0.012)),
        origin=Origin(xyz=(-0.146, 0.0, 0.014)),
        material=rubber,
        name="grip_pad",
    )

    guide_bar = model.part("guide_bar")
    guide_bar.visual(
        Cylinder(radius=0.005, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="guide_bar",
    )
    guide_bar.visual(
        Box((0.012, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.082, -0.010)),
        material=steel,
        name="guide_stem_0",
    )
    guide_bar.visual(
        Box((0.012, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.082, -0.010)),
        material=steel,
        name="guide_stem_1",
    )
    guide_bar.visual(
        Box((0.045, 0.018, 0.004)),
        origin=Origin(xyz=(0.018, -0.082, -0.020)),
        material=steel,
        name="guide_runner_0",
    )
    guide_bar.visual(
        Box((0.045, 0.018, 0.004)),
        origin=Origin(xyz=(0.018, 0.082, -0.020)),
        material=steel,
        name="guide_runner_1",
    )

    waste_door = model.part("waste_door")
    waste_door.visual(
        Box((0.004, 0.138, 0.030)),
        origin=Origin(xyz=(-0.002, 0.0, 0.015)),
        material=door_finish,
        name="door_panel",
    )
    waste_door.visual(
        Box((0.010, 0.050, 0.008)),
        origin=Origin(xyz=(-0.009, 0.0, 0.024)),
        material=door_finish,
        name="door_pull",
    )
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-28.0),
            upper=math.radians(42.0),
            effort=180.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide_bar,
        origin=Origin(xyz=(GUIDE_REST_X, 0.0, GUIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.045, effort=25.0, velocity=0.08),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=waste_door,
        origin=Origin(xyz=(DOOR_HINGE_X, 0.0, DOOR_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(115.0),
            effort=8.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    guide_bar = object_model.get_part("guide_bar")
    waste_door = object_model.get_part("waste_door")

    handle_hinge = object_model.get_articulation("handle_hinge")
    guide_slide = object_model.get_articulation("guide_slide")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.expect_gap(
        guide_bar,
        base,
        axis="x",
        positive_elem="guide_bar",
        negative_elem="front_fence",
        min_gap=0.0005,
        max_gap=0.012,
        name="guide bar rests just behind the front fence",
    )
    ctx.expect_gap(
        guide_bar,
        base,
        axis="z",
        positive_elem="guide_runner_0",
        negative_elem="top_deck",
        max_penetration=1e-5,
        max_gap=0.003,
        name="guide runners sit on the top deck",
    )
    ctx.expect_overlap(
        guide_bar,
        base,
        axes="y",
        elem_a="guide_bar",
        elem_b="front_fence",
        min_overlap=0.19,
        name="guide bar spans the front fence width",
    )

    guide_rest = ctx.part_world_position(guide_bar)
    with ctx.pose({guide_slide: 0.045}):
        ctx.expect_gap(
            guide_bar,
            base,
            axis="z",
            positive_elem="guide_runner_1",
            negative_elem="top_deck",
            max_penetration=1e-5,
            max_gap=0.003,
            name="guide runners stay supported when slid inward",
        )
        guide_extended = ctx.part_world_position(guide_bar)
    ctx.check(
        "guide bar slides rearward",
        guide_rest is not None
        and guide_extended is not None
        and guide_extended[0] > guide_rest[0] + 0.035,
        details=f"rest={guide_rest}, extended={guide_extended}",
    )

    grip_rest = ctx.part_element_world_aabb(handle, elem="front_grip")
    with ctx.pose({handle_hinge: math.radians(35.0)}):
        grip_raised = ctx.part_element_world_aabb(handle, elem="front_grip")
    ctx.check(
        "handle lifts upward on positive rotation",
        grip_rest is not None
        and grip_raised is not None
        and grip_raised[1][2] > grip_rest[1][2] + 0.045,
        details=f"rest={grip_rest}, raised={grip_raised}",
    )

    door_rest = ctx.part_element_world_aabb(waste_door, elem="door_panel")
    with ctx.pose({door_hinge: math.radians(100.0)}):
        door_open = ctx.part_element_world_aabb(waste_door, elem="door_panel")
    ctx.check(
        "waste door opens downward and outward",
        door_rest is not None
        and door_open is not None
        and door_open[1][2] < door_rest[1][2] - 0.010
        and door_open[0][0] < door_rest[0][0] - 0.015,
        details=f"rest={door_rest}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
