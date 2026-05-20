from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

WALL_HINGE_X = 0.018
WALL_HINGE_Z = 0.020
BASE_LINK_LENGTH = 0.118
OUTER_ARM_LENGTH = 0.218


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_x(radius: float, length: float, start_x: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def _cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    x_pos, y_pos, z_pos = center
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x_pos, y_pos - (length / 2.0), z_pos))
    )


def _wall_plate_shape() -> cq.Workplane:
    shape = _box((0.008, 0.065, 0.110), (0.0, 0.0, 0.0))
    shape = shape.union(_box((0.008, 0.028, 0.010), (0.007, 0.0, 0.002)))
    for y_pos in (-0.010, 0.010):
        shape = shape.union(_box((0.018, 0.006, 0.028), (0.015, y_pos, WALL_HINGE_Z)))
    return shape


def _base_link_shape() -> cq.Workplane:
    shape = _box((0.012, 0.014, 0.020), (0.0, 0.0, 0.0))
    shape = shape.union(_box((0.104, 0.012, 0.014), (0.056, 0.0, -0.004)))
    shape = shape.union(_box((0.014, 0.024, 0.010), (0.101, 0.0, -0.015)))
    for y_pos in (-0.008, 0.008):
        shape = shape.union(_box((0.024, 0.006, 0.024), (0.113, y_pos, 0.0)))
    return shape


def _outer_arm_shape() -> cq.Workplane:
    shape = _box((0.012, 0.010, 0.018), (0.0, 0.0, 0.0))
    shape = shape.union(_box((0.184, 0.010, 0.012), (0.092, 0.0, -0.004)))
    shape = shape.union(_box((0.022, 0.024, 0.028), (0.187, 0.0, -0.020)))
    shape = shape.union(_box((0.020, 0.068, 0.010), (0.192, 0.0, -0.035)))
    for y_pos in (-0.031, 0.031):
        shape = shape.union(_box((0.034, 0.006, 0.044), (0.203, y_pos, -0.008)))
    return shape


def _head_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("YZ").circle(0.028).extrude(0.084)
    shell = shell.faces(">X").workplane().circle(0.0225).cutBlind(-0.070)
    shell = shell.translate((-0.010, 0.0, 0.0))
    shell = shell.union(_cylinder_x(0.021, 0.020, -0.018))
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_arm_task_lamp")

    model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("arm_black", rgba=(0.17, 0.17, 0.18, 1.0))
    model.material("charcoal", rgba=(0.10, 0.10, 0.11, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material="powder_black",
        name="wall_plate_shell",
    )

    base_link = model.part("base_link")
    base_link.visual(
        mesh_from_cadquery(_base_link_shape(), "base_link"),
        material="arm_black",
        name="base_link_body",
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        mesh_from_cadquery(_outer_arm_shape(), "outer_arm"),
        material="arm_black",
        name="outer_arm_body",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "head_shell"),
        material="charcoal",
        name="head_shell",
    )

    model.articulation(
        "wall_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=base_link,
        origin=Origin(xyz=(WALL_HINGE_X, 0.0, WALL_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-30.0),
            upper=math.radians(80.0),
            effort=18.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=base_link,
        child=outer_arm,
        origin=Origin(xyz=(BASE_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-80.0),
            upper=math.radians(85.0),
            effort=14.0,
            velocity=1.7,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=head,
        origin=Origin(xyz=(OUTER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-55.0),
            upper=math.radians(70.0),
            effort=8.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    base_link = object_model.get_part("base_link")
    outer_arm = object_model.get_part("outer_arm")
    head = object_model.get_part("head")

    wall_hinge = object_model.get_articulation("wall_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.allow_overlap(
        wall_plate,
        base_link,
        elem_a="wall_plate_shell",
        elem_b="base_link_body",
        reason="The wall bracket and rear knuckle are intentionally simplified as nested hinge members without separate pin bores.",
    )
    ctx.allow_overlap(
        base_link,
        outer_arm,
        elem_a="base_link_body",
        elem_b="outer_arm_body",
        reason="The elbow hinge uses simplified nested knuckle solids instead of a fully bored clevis and pin assembly.",
    )
    ctx.allow_overlap(
        outer_arm,
        head,
        elem_a="outer_arm_body",
        elem_b="head_shell",
        reason="The yoke and spotlight head intentionally share a simplified trunnion seat volume at the tilt hinge.",
    )

    ctx.expect_gap(
        head,
        wall_plate,
        axis="x",
        positive_elem="head_shell",
        negative_elem="wall_plate_shell",
        min_gap=0.28,
        name="spotlight head projects well clear of the wall plate",
    )
    ctx.expect_origin_gap(
        head,
        wall_plate,
        axis="x",
        min_gap=0.33,
        name="task light reaches out from the wall",
    )

    rest_elbow = ctx.part_world_position(outer_arm)
    with ctx.pose({wall_hinge: math.radians(40.0)}):
        raised_elbow = ctx.part_world_position(outer_arm)
    ctx.check(
        "wall hinge raises the elbow outward and upward",
        rest_elbow is not None
        and raised_elbow is not None
        and raised_elbow[2] > rest_elbow[2] + 0.06
        and raised_elbow[0] < rest_elbow[0] - 0.02,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({elbow_hinge: math.radians(45.0)}):
        raised_head = ctx.part_world_position(head)
    ctx.check(
        "elbow hinge lifts the spotlight assembly",
        rest_head is not None
        and raised_head is not None
        and raised_head[2] > rest_head[2] + 0.12,
        details=f"rest={rest_head}, raised={raised_head}",
    )

    rest_head_shell = ctx.part_element_world_aabb(head, elem="head_shell")
    with ctx.pose({head_tilt: math.radians(35.0)}):
        tilted_head_shell = ctx.part_element_world_aabb(head, elem="head_shell")
    ctx.check(
        "head tilt pitches the spotlight upward",
        rest_head_shell is not None
        and tilted_head_shell is not None
        and tilted_head_shell[1][2] > rest_head_shell[1][2] + 0.03,
        details=f"rest={rest_head_shell}, tilted={tilted_head_shell}",
    )

    return ctx.report()


object_model = build_object_model()
