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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_PLATE_W = 0.110
TOP_PLATE_D = 0.085
TOP_PLATE_T = 0.008
TOP_PLATE_Z = 0.028

BRACKET_INNER_SPAN_X = 0.072
BRACKET_CHEEK_T = 0.010
BRACKET_CHEEK_D = 0.048
BRACKET_CHEEK_H = 0.034
BRACKET_CHEEK_Z = 0.007
BRACKET_CHEEK_X = BRACKET_INNER_SPAN_X / 2.0 + BRACKET_CHEEK_T / 2.0
BRACKET_PIVOT_HOLE_R = 0.0059

OUTER_PIVOT_AXLE_R = BRACKET_PIVOT_HOLE_R
OUTER_PIVOT_AXLE_L = BRACKET_INNER_SPAN_X + 2.0 * BRACKET_CHEEK_T
OUTER_BODY_T = 0.010
OUTER_BODY_W = 0.054
OUTER_BODY_H = 0.050
OUTER_BODY_CENTER_Z = -0.034
OUTER_BODY_OPEN_W = 0.036
OUTER_BODY_OPEN_H = 0.032
OUTER_BODY_OPEN_Z = -0.039
OUTER_TO_INNER_DROP = 0.017
OUTER_INNER_HOLE_R = 0.0054

INNER_BODY_T = 0.010
INNER_BODY_W = 0.044
INNER_BODY_H = 0.048
INNER_BODY_CENTER_Z = -0.034
INNER_BODY_OPEN_W = 0.028
INNER_BODY_OPEN_H = 0.028
INNER_BODY_OPEN_Z = -0.039
INNER_PIVOT_AXLE_R = OUTER_INNER_HOLE_R
INNER_PIVOT_AXLE_L = OUTER_BODY_T + 0.006
INNER_TO_STICK_DROP = 0.061

STICK_LENGTH = 0.118


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .translate(center)
    )


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate(center)
    )


def _support_bracket_shape() -> cq.Workplane:
    bracket = _box((TOP_PLATE_W, TOP_PLATE_D, TOP_PLATE_T), (0.0, 0.0, TOP_PLATE_Z))
    for x_sign in (-1.0, 1.0):
        cheek = _box(
            (BRACKET_CHEEK_T, BRACKET_CHEEK_D, BRACKET_CHEEK_H),
            (x_sign * BRACKET_CHEEK_X, 0.0, BRACKET_CHEEK_Z),
        )
        boss = _x_cylinder(
            0.013,
            BRACKET_CHEEK_T,
            (x_sign * BRACKET_CHEEK_X, 0.0, 0.0),
        )
        bracket = bracket.union(cheek).union(boss)

    for x_pos in (-0.036, 0.036):
        for y_pos in (-0.024, 0.024):
            bracket = bracket.cut(_z_cylinder(0.0032, TOP_PLATE_T + 0.004, (x_pos, y_pos, TOP_PLATE_Z)))

    for x_sign in (-1.0, 1.0):
        bracket = bracket.cut(
            _x_cylinder(
                BRACKET_PIVOT_HOLE_R,
                BRACKET_CHEEK_T + 0.008,
                (x_sign * BRACKET_CHEEK_X, 0.0, 0.0),
            )
        )

    return bracket


def _outer_yoke_shape() -> cq.Workplane:
    return _outer_yoke_pivot_shape().union(_outer_yoke_body_shape())


def _outer_yoke_pivot_shape() -> cq.Workplane:
    return _x_cylinder(OUTER_PIVOT_AXLE_R, OUTER_PIVOT_AXLE_L, (0.0, 0.0, 0.0))


def _outer_yoke_body_shape() -> cq.Workplane:
    body = _box((OUTER_BODY_W, OUTER_BODY_T, OUTER_BODY_H), (0.0, 0.0, OUTER_BODY_CENTER_Z))
    body = body.cut(
        _box(
            (OUTER_BODY_OPEN_W, OUTER_BODY_T + 0.002, OUTER_BODY_OPEN_H),
            (0.0, 0.0, OUTER_BODY_OPEN_Z),
        )
    )
    body = body.cut(
        _y_cylinder(
            OUTER_INNER_HOLE_R,
            OUTER_BODY_T + 0.004,
            (0.0, 0.0, -OUTER_TO_INNER_DROP),
        )
    )
    web = _box((0.018, OUTER_BODY_T, 0.014), (0.0, 0.0, -0.007))
    return web.union(body)


def _inner_yoke_shape() -> cq.Workplane:
    return _inner_yoke_pivot_shape().union(_inner_yoke_body_shape())


def _inner_yoke_pivot_shape() -> cq.Workplane:
    return _y_cylinder(INNER_PIVOT_AXLE_R, INNER_PIVOT_AXLE_L, (0.0, 0.0, 0.0))


def _inner_yoke_body_shape() -> cq.Workplane:
    lower_yoke = _box((INNER_BODY_T, 0.034, 0.032), (0.0, 0.0, -0.043))
    lower_yoke = lower_yoke.cut(_box((INNER_BODY_T + 0.002, 0.020, 0.020), (0.0, 0.0, -0.046)))
    stem = _z_cylinder(0.0058, 0.036, (0.0, 0.0, -0.021))
    collar = _z_cylinder(0.011, 0.010, (0.0, 0.0, -0.058))
    return lower_yoke.union(stem).union(collar)


def _stick_shape() -> cq.Workplane:
    shaft = _z_cylinder(0.007, 0.056, (0.0, 0.0, 0.028))
    grip = (
        cq.Workplane("XY")
        .circle(0.010)
        .workplane(offset=0.026)
        .circle(0.015)
        .workplane(offset=0.022)
        .circle(0.017)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.056))
    )
    base_cap = _z_cylinder(0.017, 0.014, (0.0, 0.0, 0.111))
    shape = shaft.union(grip).union(base_cap)
    shape = shape.union(_z_cylinder(0.010, 0.004, (0.0, 0.0, 0.002)))
    return shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 180.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_joystick")

    bracket_finish = model.material("bracket_finish", rgba=(0.24, 0.26, 0.29, 1.0))
    outer_finish = model.material("outer_finish", rgba=(0.34, 0.36, 0.39, 1.0))
    inner_finish = model.material("inner_finish", rgba=(0.48, 0.50, 0.54, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.11, 0.12, 0.13, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        mesh_from_cadquery(_support_bracket_shape(), "support_bracket"),
        material=bracket_finish,
        name="bracket_frame",
    )
    support_bracket.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_W, TOP_PLATE_D, TOP_PLATE_Z + TOP_PLATE_T / 2.0 - (-0.010))),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        mesh_from_cadquery(_outer_yoke_body_shape(), "outer_yoke_body"),
        material=outer_finish,
        name="outer_yoke_body",
    )
    outer_yoke.visual(
        mesh_from_cadquery(_outer_yoke_pivot_shape(), "outer_x_pivot"),
        material=inner_finish,
        name="outer_x_pivot",
    )
    outer_yoke.inertial = Inertial.from_geometry(
        Box((OUTER_PIVOT_AXLE_L, OUTER_BODY_T, 0.060)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
    )

    inner_yoke = model.part("inner_yoke")
    inner_yoke.visual(
        mesh_from_cadquery(_inner_yoke_body_shape(), "inner_yoke_body"),
        material=inner_finish,
        name="inner_yoke_body",
    )
    inner_yoke.visual(
        mesh_from_cadquery(_inner_yoke_pivot_shape(), "inner_y_pivot"),
        material=outer_finish,
        name="inner_y_pivot",
    )
    inner_yoke.inertial = Inertial.from_geometry(
        Box((INNER_BODY_T, INNER_BODY_W + INNER_PIVOT_AXLE_L, 0.062)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    stick = model.part("stick")
    stick.visual(
        mesh_from_cadquery(_stick_shape(), "stick"),
        material=grip_finish,
        name="stick_body",
    )
    stick.inertial = Inertial.from_geometry(
        Box((0.034, 0.034, STICK_LENGTH)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -STICK_LENGTH / 2.0)),
    )

    model.articulation(
        "bracket_to_outer_yoke",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "outer_to_inner_yoke",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_yoke,
        origin=Origin(xyz=(0.0, 0.0, -OUTER_TO_INNER_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "inner_to_stick",
        ArticulationType.FIXED,
        parent=inner_yoke,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, -INNER_TO_STICK_DROP)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_yoke = object_model.get_part("inner_yoke")
    stick = object_model.get_part("stick")

    outer_joint = object_model.get_articulation("bracket_to_outer_yoke")
    inner_joint = object_model.get_articulation("outer_to_inner_yoke")

    ctx.check("support bracket present", support_bracket is not None)
    ctx.check("outer yoke present", outer_yoke is not None)
    ctx.check("inner yoke present", inner_yoke is not None)
    ctx.check("stick present", stick is not None)

    ctx.allow_overlap(
        support_bracket,
        outer_yoke,
        elem_a="bracket_frame",
        elem_b="outer_x_pivot",
        reason="The outer x-axis journal is represented as a close-fit trunnion through the bracket bearing bores.",
    )
    ctx.allow_overlap(
        outer_yoke,
        inner_yoke,
        elem_a="outer_yoke_body",
        elem_b="inner_y_pivot",
        reason="The inner y-axis journal is represented as a close-fit trunnion through the outer yoke bearing bore.",
    )
    ctx.allow_overlap(
        outer_yoke,
        inner_yoke,
        elem_a="outer_yoke_body",
        elem_b="inner_yoke_body",
        reason="The nested yoke meshes use a zero-clearance bearing-pocket representation around the inner hanger stem at the neutral pose.",
    )

    ctx.expect_origin_distance(
        support_bracket,
        outer_yoke,
        axes="xyz",
        max_dist=0.001,
        name="outer yoke pivots about the bracket centerline",
    )
    ctx.expect_origin_gap(
        outer_yoke,
        inner_yoke,
        axis="z",
        min_gap=0.016,
        max_gap=0.018,
        name="inner yoke hangs below the outer yoke pivot",
    )
    ctx.expect_origin_gap(
        inner_yoke,
        stick,
        axis="z",
        min_gap=0.060,
        max_gap=0.062,
        name="stick mount sits below the inner yoke pivot",
    )
    ctx.expect_origin_gap(
        support_bracket,
        stick,
        axis="z",
        min_gap=0.075,
        name="stick hangs below the support bracket",
    )

    rest_pos = ctx.part_world_position(stick)

    with ctx.pose({outer_joint: 0.45}):
        outer_pos = ctx.part_world_position(stick)
        ctx.expect_origin_distance(
            support_bracket,
            outer_yoke,
            axes="xyz",
            max_dist=0.001,
            name="outer pivot stays concentric while rolled",
        )

    ctx.check(
        "outer yoke swings the stick sideways",
        rest_pos is not None
        and outer_pos is not None
        and outer_pos[1] > rest_pos[1] + 0.03,
        details=f"rest={rest_pos}, rolled={outer_pos}",
    )

    with ctx.pose({inner_joint: 0.50}):
        inner_pos = ctx.part_world_position(stick)
        ctx.expect_origin_gap(
            outer_yoke,
            inner_yoke,
            axis="z",
            min_gap=0.016,
            max_gap=0.018,
            name="inner pivot stays at the dropped gimbal center while pitched",
        )

    ctx.check(
        "inner yoke swings the stick fore-aft",
        rest_pos is not None
        and inner_pos is not None
        and inner_pos[0] > rest_pos[0] + 0.025,
        details=f"rest={rest_pos}, pitched={inner_pos}",
    )

    with ctx.pose({outer_joint: 0.35, inner_joint: 0.45}):
        ctx.expect_origin_gap(
            support_bracket,
            stick,
            axis="z",
            min_gap=0.045,
            name="stick remains under-slung in a compound pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
