from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.22
BASE_THICKNESS = 0.045
LOWER_SLEEVE_RADIUS = 0.028
LOWER_SLEEVE_INNER_RADIUS = INNER_POLE_RADIUS = 0.017
LOWER_SLEEVE_BOTTOM = BASE_THICKNESS - 0.002
LOWER_SLEEVE_TOP = 1.00
COLLAR_HEIGHT = 0.080
COLLAR_CENTER_Z = 1.005
SLIDE_ORIGIN_Z = 1.020

INNER_POLE_LENGTH = 1.150
INNER_POLE_LOCAL_MIN_Z = -0.450
INNER_POLE_LOCAL_MAX_Z = INNER_POLE_LOCAL_MIN_Z + INNER_POLE_LENGTH
ARM_HEIGHT = INNER_POLE_LOCAL_MAX_Z
ARM_LENGTH = 0.320
SLIDE_TRAVEL = 0.350

SHADE_AXIS_X = 0.090
SHADE_RIM_Z = -0.165
SHADE_CROWN_Z = -0.055
SHADE_RADIUS = 0.155


def _beveled_disc() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(BASE_THICKNESS, BASE_RADIUS)
        .edges()
        .fillet(0.004)
    )


def _hollow_tube(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    outer = cq.Workplane("XY").cylinder(length, outer_radius)
    bore = cq.Workplane("XY").cylinder(length + 0.020, inner_radius)
    return outer.cut(bore)


def _shade_shell() -> cq.Workplane:
    # Revolve a thin cross-section about local +Z, then move the shade axis
    # forward of the hinge.  The lower side is open like a real pharmacy shade.
    shell = (
        cq.Workplane("XZ")
        .moveTo(SHADE_RADIUS, 0.000)
        .spline(
            [
                (0.145, 0.030),
                (0.105, 0.078),
                (0.036, 0.108),
            ]
        )
        .lineTo(0.026, 0.089)
        .spline(
            [
                (0.090, 0.067),
                (0.126, 0.026),
                (0.137, 0.010),
            ]
        )
        .lineTo(SHADE_RADIUS, 0.000)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )
    return shell.translate((SHADE_AXIS_X, 0.0, SHADE_RIM_Z))


def _shade_inner_liner() -> cq.Workplane:
    # A very thin off-white inner liner sits just inside the metal shade, leaving
    # the rim visibly open.
    liner = (
        cq.Workplane("XZ")
        .moveTo(0.132, 0.014)
        .spline(
            [
                (0.119, 0.032),
                (0.083, 0.067),
                (0.031, 0.086),
            ]
        )
        .lineTo(0.028, 0.078)
        .spline(
            [
                (0.079, 0.060),
                (0.111, 0.029),
                (0.124, 0.018),
            ]
        )
        .lineTo(0.132, 0.014)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )
    return liner.translate((SHADE_AXIS_X, 0.0, SHADE_RIM_Z))


def _rim_band() -> cq.Workplane:
    band = (
        cq.Workplane("XZ")
        .moveTo(SHADE_RADIUS - 0.004, -0.004)
        .lineTo(SHADE_RADIUS + 0.004, -0.004)
        .lineTo(SHADE_RADIUS + 0.004, 0.006)
        .lineTo(SHADE_RADIUS - 0.004, 0.006)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )
    return band.translate((SHADE_AXIS_X, 0.0, SHADE_RIM_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_floor_lamp")

    model.material("aged_brass", rgba=(0.70, 0.56, 0.30, 1.0))
    model.material("dark_felt", rgba=(0.025, 0.024, 0.022, 1.0))
    model.material("green_enamel", rgba=(0.02, 0.24, 0.13, 1.0))
    model.material("warm_cream", rgba=(0.98, 0.91, 0.72, 1.0))
    model.material("dark_bakelite", rgba=(0.04, 0.035, 0.03, 1.0))
    model.material("warm_glow", rgba=(1.0, 0.82, 0.42, 0.75))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_beveled_disc(), "weighted_disc"),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="aged_brass",
        name="weighted_disc",
    )
    base.visual(
        Cylinder(radius=BASE_RADIUS * 0.92, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material="dark_felt",
        name="felt_pad",
    )
    base.visual(
        mesh_from_cadquery(
            _hollow_tube(
                LOWER_SLEEVE_TOP - LOWER_SLEEVE_BOTTOM,
                LOWER_SLEEVE_RADIUS,
                LOWER_SLEEVE_INNER_RADIUS,
            ),
            "lower_sleeve",
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (LOWER_SLEEVE_TOP + LOWER_SLEEVE_BOTTOM) / 2.0,
            )
        ),
        material="aged_brass",
        name="lower_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_hollow_tube(COLLAR_HEIGHT, 0.043, LOWER_SLEEVE_INNER_RADIUS), "height_collar"),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_CENTER_Z)),
        material="aged_brass",
        name="height_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.058),
        origin=Origin(
            xyz=(0.0, -0.064, COLLAR_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="aged_brass",
        name="clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.014),
        origin=Origin(
            xyz=(0.0, -0.099, COLLAR_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="dark_bakelite",
        name="lock_knob",
    )

    upper_pole = model.part("upper_pole")
    upper_pole.visual(
        Cylinder(radius=INNER_POLE_RADIUS, length=INNER_POLE_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, (INNER_POLE_LOCAL_MIN_Z + INNER_POLE_LOCAL_MAX_Z) / 2.0)
        ),
        material="aged_brass",
        name="inner_tube",
    )
    upper_pole.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(
            xyz=(0.150, 0.0, ARM_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="aged_brass",
        name="arm_tube",
    )
    upper_pole.visual(
        Sphere(radius=0.027),
        origin=Origin(xyz=(0.0, 0.0, ARM_HEIGHT)),
        material="aged_brass",
        name="elbow_ball",
    )
    upper_pole.visual(
        Box((0.026, 0.076, 0.030)),
        origin=Origin(xyz=(0.297, 0.0, ARM_HEIGHT)),
        material="aged_brass",
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.041, 0.041)):
        upper_pole.visual(
            Box((0.070, 0.010, 0.060)),
            origin=Origin(xyz=(ARM_LENGTH, y, ARM_HEIGHT)),
            material="aged_brass",
            name=f"yoke_plate_{index}",
        )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_shade_shell(), "dome_shell"),
        material="green_enamel",
        name="dome_shell",
    )
    shade.visual(
        mesh_from_cadquery(_shade_inner_liner(), "inner_liner"),
        material="warm_cream",
        name="inner_liner",
    )
    shade.visual(
        mesh_from_cadquery(_rim_band(), "rolled_rim"),
        material="aged_brass",
        name="rolled_rim",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aged_brass",
        name="hinge_barrel",
    )
    shade.visual(
        Box((0.043, 0.026, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, -0.007)),
        material="aged_brass",
        name="hinge_lug",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(
            xyz=(0.055, 0.0, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="aged_brass",
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(SHADE_AXIS_X, 0.0, -0.030)),
        material="aged_brass",
        name="socket_stem",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(SHADE_AXIS_X, 0.0, -0.086)),
        material="aged_brass",
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(SHADE_AXIS_X, 0.0, -0.124)),
        material="warm_glow",
        name="bulb",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_pole,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=80.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_pole,
        child=shade,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, ARM_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.75,
            upper=0.75,
            effort=5.0,
            velocity=1.2,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_pole = object_model.get_part("upper_pole")
    shade = object_model.get_part("shade")
    height_slide = object_model.get_articulation("height_slide")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.allow_overlap(
        base,
        upper_pole,
        elem_a="lower_sleeve",
        elem_b="inner_tube",
        reason=(
            "The telescoping inner pole is intentionally represented as a tight "
            "sliding fit captured inside the lower sleeve proxy."
        ),
    )
    ctx.allow_overlap(
        base,
        upper_pole,
        elem_a="height_collar",
        elem_b="inner_tube",
        reason=(
            "The collar is a tight guide bushing around the telescoping pole, "
            "so the simplified collar proxy intentionally captures the tube."
        ),
    )
    ctx.expect_within(
        upper_pole,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_sleeve",
        margin=0.002,
        name="sliding tube is centered in the lower sleeve",
    )
    ctx.expect_overlap(
        upper_pole,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_sleeve",
        min_overlap=0.35,
        name="collapsed pole has deep retained insertion",
    )
    ctx.expect_within(
        upper_pole,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="height_collar",
        margin=0.002,
        name="sliding tube is centered in the lock collar",
    )
    ctx.expect_overlap(
        upper_pole,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="height_collar",
        min_overlap=0.070,
        name="collar surrounds the moving tube at rest",
    )

    rest_position = ctx.part_world_position(upper_pole)
    rest_shade_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({height_slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            upper_pole,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_sleeve",
            margin=0.002,
            name="extended tube stays centered in the sleeve",
        )
        ctx.expect_overlap(
            upper_pole,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_sleeve",
            min_overlap=0.060,
            name="extended pole remains captured in the lower sleeve",
        )
        extended_position = ctx.part_world_position(upper_pole)

    ctx.check(
        "prismatic joint raises the upper stage",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + SLIDE_TRAVEL * 0.95,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    with ctx.pose({shade_tilt: 0.65}):
        tilted_shade_aabb = ctx.part_world_aabb(shade)

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "revolute hinge visibly pitches the dome shade",
        rest_shade_aabb is not None
        and tilted_shade_aabb is not None
        and abs(_aabb_center_x(tilted_shade_aabb) - _aabb_center_x(rest_shade_aabb)) > 0.035,
        details=f"rest_aabb={rest_shade_aabb}, tilted_aabb={tilted_shade_aabb}",
    )
    ctx.check(
        "shade tilt limit is a practical pharmacy-lamp pitch range",
        shade_tilt.motion_limits is not None
        and shade_tilt.motion_limits.lower <= -0.70
        and shade_tilt.motion_limits.upper >= 0.70,
        details=f"limits={shade_tilt.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
