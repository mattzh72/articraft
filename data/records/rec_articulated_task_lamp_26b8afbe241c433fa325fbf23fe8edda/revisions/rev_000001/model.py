from __future__ import annotations

from math import radians

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


PLATE_RADIUS = 0.055
PLATE_THICKNESS = 0.010
WALL_HUB_RADIUS = 0.018
WALL_HUB_LENGTH = 0.022
SHOULDER_X = 0.028

LOWER_ARM_LENGTH = 0.140
UPPER_ARM_LENGTH = 0.205

LOWER_TONGUE_LENGTH = 0.018
LOWER_TONGUE_WIDTH = 0.007
LOWER_TONGUE_HEIGHT = 0.018
LOWER_BODY_START = 0.012
LOWER_BODY_WIDTH = 0.016
LOWER_BODY_HEIGHT = 0.012
LOWER_YOKE_LENGTH = 0.018
LOWER_YOKE_GAP = 0.010
LOWER_YOKE_EAR = 0.005
LOWER_YOKE_HEIGHT = 0.024

UPPER_TONGUE_LENGTH = 0.018
UPPER_TONGUE_WIDTH = 0.007
UPPER_TONGUE_HEIGHT = 0.016
UPPER_BODY_START = 0.012
UPPER_BODY_WIDTH = 0.014
UPPER_BODY_HEIGHT = 0.010
UPPER_YOKE_LENGTH = 0.016
UPPER_YOKE_GAP = 0.008
UPPER_YOKE_EAR = 0.005
UPPER_YOKE_HEIGHT = 0.020

SHADE_TAB_LENGTH = 0.015
SHADE_TAB_WIDTH = 0.006
SHADE_TAB_HEIGHT = 0.016
SHADE_DROP = -0.018
SHADE_BRACKET_START = 0.012
SHADE_BRACKET_LENGTH = 0.030
SHADE_BRACKET_WIDTH = 0.012
SHADE_BRACKET_HEIGHT = 0.014
SHADE_COLLAR_START = 0.016
SHADE_COLLAR_LENGTH = 0.024
SHADE_COLLAR_RADIUS = 0.019
SHADE_SHELL_START = 0.028
SHADE_SHELL_LENGTH = 0.098
SHADE_REAR_RADIUS = 0.025
SHADE_FRONT_RADIUS = 0.055
SHADE_WALL = 0.0022


def _y_axis_knuckle(*, x_center: float, radius: float, width: float, z_center: float = 0.0):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(width)
        .translate((x_center, -width / 2.0, z_center))
    )


def _wall_plate_shape():
    shape = cq.Workplane("YZ").circle(PLATE_RADIUS).extrude(PLATE_THICKNESS)
    shape = shape.union(cq.Workplane("YZ").circle(WALL_HUB_RADIUS).extrude(WALL_HUB_LENGTH))

    return shape


def _yoke_ear(*, x_center: float, y_center: float, length: float, thickness: float, height: float):
    return (
        cq.Workplane("XY")
        .box(length, thickness, height, centered=(True, True, True))
        .translate((x_center, y_center, 0.0))
    )


def _wall_plate_ears():
    yoke_offset = (LOWER_YOKE_GAP / 2.0) + (LOWER_YOKE_EAR / 2.0)
    return [
        _yoke_ear(
            x_center=SHOULDER_X,
            y_center=sign * yoke_offset,
            length=LOWER_YOKE_LENGTH,
            thickness=LOWER_YOKE_EAR,
            height=LOWER_YOKE_HEIGHT,
        )
        for sign in (-1.0, 1.0)
    ]


def _lower_arm_shape():

    tongue = cq.Workplane("XY").box(
        LOWER_TONGUE_LENGTH,
        LOWER_TONGUE_WIDTH,
        LOWER_TONGUE_HEIGHT,
        centered=(False, True, True),
    )
    body = (
        cq.Workplane("XY")
        .box(
            LOWER_ARM_LENGTH - LOWER_BODY_START,
            LOWER_BODY_WIDTH,
            LOWER_BODY_HEIGHT,
            centered=(False, True, True),
        )
        .translate((LOWER_BODY_START, 0.0, 0.0))
    )
    knuckle = _y_axis_knuckle(
        x_center=0.007,
        radius=0.009,
        width=LOWER_TONGUE_WIDTH,
    )

    return tongue.union(body).union(knuckle)


def _lower_arm_ears():
    yoke_offset = (LOWER_YOKE_GAP / 2.0) + (LOWER_YOKE_EAR / 2.0)
    return [
        _yoke_ear(
            x_center=LOWER_ARM_LENGTH + 0.002,
            y_center=sign * yoke_offset,
            length=LOWER_YOKE_LENGTH,
            thickness=LOWER_YOKE_EAR,
            height=LOWER_YOKE_HEIGHT,
        )
        for sign in (-1.0, 1.0)
    ]


def _upper_arm_shape():
    tongue = cq.Workplane("XY").box(
        UPPER_TONGUE_LENGTH,
        UPPER_TONGUE_WIDTH,
        UPPER_TONGUE_HEIGHT,
        centered=(False, True, True),
    )
    body = (
        cq.Workplane("XY")
        .box(
            UPPER_ARM_LENGTH - UPPER_BODY_START,
            UPPER_BODY_WIDTH,
            UPPER_BODY_HEIGHT,
            centered=(False, True, True),
        )
        .translate((UPPER_BODY_START, 0.0, 0.0))
    )
    knuckle = _y_axis_knuckle(
        x_center=0.007,
        radius=0.008,
        width=UPPER_TONGUE_WIDTH,
    )

    return tongue.union(body).union(knuckle)


def _upper_arm_ears():
    yoke_offset = (UPPER_YOKE_GAP / 2.0) + (UPPER_YOKE_EAR / 2.0)
    return [
        _yoke_ear(
            x_center=UPPER_ARM_LENGTH + 0.002,
            y_center=sign * yoke_offset,
            length=UPPER_YOKE_LENGTH,
            thickness=UPPER_YOKE_EAR,
            height=UPPER_YOKE_HEIGHT,
        )
        for sign in (-1.0, 1.0)
    ]


def _shade_bracket_shape():
    tab = cq.Workplane("XY").box(
        SHADE_TAB_LENGTH,
        SHADE_TAB_WIDTH,
        SHADE_TAB_HEIGHT,
        centered=(False, True, True),
    )
    knuckle = _y_axis_knuckle(
        x_center=0.006,
        radius=0.008,
        width=SHADE_TAB_WIDTH,
    )
    bridge = (
        cq.Workplane("XY")
        .box(
            SHADE_BRACKET_LENGTH,
            SHADE_BRACKET_WIDTH,
            SHADE_BRACKET_HEIGHT,
            centered=(False, True, True),
        )
        .translate((SHADE_BRACKET_START, 0.0, -0.010))
    )
    collar = (
        cq.Workplane("YZ")
        .circle(SHADE_COLLAR_RADIUS)
        .extrude(SHADE_COLLAR_LENGTH)
        .translate((SHADE_COLLAR_START, 0.0, SHADE_DROP))
    )

    return tab.union(knuckle).union(bridge).union(collar)


def _shade_shell_shape():
    outer = (
        cq.Workplane("YZ")
        .circle(SHADE_REAR_RADIUS)
        .workplane(offset=SHADE_SHELL_LENGTH)
        .circle(SHADE_FRONT_RADIUS)
        .loft()
        .translate((SHADE_SHELL_START, 0.0, SHADE_DROP))
    )
    inner = (
        cq.Workplane("YZ")
        .circle(SHADE_REAR_RADIUS - (SHADE_WALL * 1.6))
        .workplane(offset=SHADE_SHELL_LENGTH + 0.003)
        .circle(SHADE_FRONT_RADIUS - SHADE_WALL)
        .loft()
        .translate((SHADE_SHELL_START + 0.012, 0.0, SHADE_DROP))
    )
    lip = (
        cq.Workplane("YZ")
        .circle(SHADE_FRONT_RADIUS + 0.002)
        .circle(SHADE_FRONT_RADIUS - 0.001)
        .extrude(0.004)
        .translate((SHADE_SHELL_START + SHADE_SHELL_LENGTH - 0.004, 0.0, SHADE_DROP))
    )

    return outer.cut(inner).union(lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_task_lamp")

    model.material("graphite", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("shade_cream", rgba=(0.91, 0.89, 0.82, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material="graphite",
        name="plate_body",
    )
    for index, ear in enumerate(_wall_plate_ears()):
        wall_plate.visual(
            mesh_from_cadquery(ear, f"wall_plate_ear_{index}"),
            material="graphite",
            name=f"plate_ear_{index}",
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_lower_arm_shape(), "lower_arm"),
        material="graphite",
        name="lower_link",
    )
    for index, ear in enumerate(_lower_arm_ears()):
        lower_arm.visual(
            mesh_from_cadquery(ear, f"lower_arm_ear_{index}"),
            material="graphite",
            name=f"lower_ear_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm"),
        material="graphite",
        name="upper_link",
    )
    for index, ear in enumerate(_upper_arm_ears()):
        upper_arm.visual(
            mesh_from_cadquery(ear, f"upper_arm_ear_{index}"),
            material="graphite",
            name=f"upper_ear_{index}",
        )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_shade_bracket_shape(), "shade_bracket"),
        material="graphite",
        name="shade_bracket",
    )
    shade.visual(
        mesh_from_cadquery(_shade_shell_shape(), "bell_shade"),
        material="shade_cream",
        name="bell_shell",
    )

    model.articulation(
        "wall_plate_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=lower_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=radians(-45.0),
            upper=radians(80.0),
            effort=16.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=radians(-75.0),
            upper=radians(95.0),
            effort=12.0,
            velocity=1.7,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=radians(-55.0),
            upper=radians(40.0),
            effort=8.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("wall_plate_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_tilt = object_model.get_articulation("upper_arm_to_shade")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, shade_tilt: 0.0}):
        ctx.expect_gap(
            shade,
            wall_plate,
            axis="x",
            positive_elem="bell_shell",
            negative_elem="plate_body",
            min_gap=0.30,
            name="bell shade projects clearly away from the wall plate",
        )
        ctx.expect_origin_gap(
            upper_arm,
            wall_plate,
            axis="x",
            min_gap=0.15,
            name="elbow hinge sits forward of the wall mount",
        )

        rest_shade_origin = ctx.part_world_position(shade)
        rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="bell_shell")

    with ctx.pose({shoulder: radians(55.0), elbow: radians(32.0), shade_tilt: 0.0}):
        raised_shade_origin = ctx.part_world_position(shade)

    ctx.check(
        "arm chain lifts the lamp head upward",
        rest_shade_origin is not None
        and raised_shade_origin is not None
        and raised_shade_origin[2] > rest_shade_origin[2] + 0.18,
        details=f"rest={rest_shade_origin}, raised={raised_shade_origin}",
    )

    with ctx.pose({shade_tilt: radians(30.0)}):
        tilted_shell_aabb = ctx.part_element_world_aabb(shade, elem="bell_shell")

    ctx.check(
        "positive shade tilt raises the bell nose",
        rest_shell_aabb is not None
        and tilted_shell_aabb is not None
        and tilted_shell_aabb[1][2] > rest_shell_aabb[1][2] + 0.03,
        details=f"rest={rest_shell_aabb}, tilted={tilted_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
