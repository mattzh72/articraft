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

BAR_LENGTH = 1.10
BAR_RADIUS = 0.014
BAR_CENTER_Z = -0.060
CANOPY_RADIUS = 0.060
CANOPY_DEPTH = 0.024
PAN_PAD_RADIUS = 0.020
PAN_PAD_HEIGHT = 0.010
PAN_Z = -0.080
YOKE_TILT_Z = -0.052
MOUNT_XS = (-0.36, 0.0, 0.36)


def _bar_shape() -> cq.Workplane:
    bar = (
        cq.Workplane("YZ")
        .circle(BAR_RADIUS)
        .extrude(BAR_LENGTH)
        .translate((-BAR_LENGTH / 2.0, 0.0, BAR_CENTER_Z))
    )

    canopy = (
        cq.Workplane("XY")
        .circle(CANOPY_RADIUS)
        .extrude(CANOPY_DEPTH)
        .translate((0.0, 0.0, -CANOPY_DEPTH))
    )
    support_length = (-CANOPY_DEPTH) - (BAR_CENTER_Z + BAR_RADIUS)
    support = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(support_length)
        .translate((0.0, 0.0, BAR_CENTER_Z + BAR_RADIUS))
    )

    assembly = bar.union(canopy).union(support)
    for x_pos in MOUNT_XS:
        pad = (
            cq.Workplane("XY")
            .circle(PAN_PAD_RADIUS)
            .extrude(PAN_PAD_HEIGHT)
            .translate((x_pos, 0.0, PAN_Z))
        )
        assembly = assembly.union(pad)

    return assembly


def _yoke_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.010))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.014, 0.028, 0.012)
        .translate((0.0, 0.0, -0.016))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.016, 0.082, 0.008)
        .translate((0.0, 0.0, -0.024))
    )
    arm_positive = (
        cq.Workplane("XY")
        .box(0.006, 0.006, 0.050)
        .translate((0.0, 0.041, -0.049))
    )
    arm_negative = (
        cq.Workplane("XY")
        .box(0.006, 0.006, 0.050)
        .translate((0.0, -0.041, -0.049))
    )
    pin_positive = (
        cq.Workplane("YZ")
        .circle(0.0025)
        .extrude(0.010)
        .translate((-0.005, 0.038, YOKE_TILT_Z))
    )
    pin_negative = (
        cq.Workplane("YZ")
        .circle(0.0025)
        .extrude(0.010)
        .translate((-0.005, -0.038, YOKE_TILT_Z))
    )

    return (
        collar.union(stem)
        .union(bridge)
        .union(arm_positive)
        .union(arm_negative)
        .union(pin_positive)
        .union(pin_negative)
    )


def _head_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.072))
    )
    inner = (
        cq.Workplane("XY")
        .circle(0.021)
        .extrude(0.052)
        .translate((0.0, 0.0, -0.072))
    )
    bezel_outer = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.006)
        .translate((0.0, 0.0, -0.078))
    )
    bezel_inner = (
        cq.Workplane("XY")
        .circle(0.021)
        .extrude(0.006)
        .translate((0.0, 0.0, -0.078))
    )
    knuckle = (
        cq.Workplane("XY")
        .box(0.012, 0.058, 0.014)
        .translate((0.0, 0.0, -0.008))
    )
    trunnion_positive = (
        cq.Workplane("YZ")
        .circle(0.0035)
        .extrude(0.010)
        .translate((-0.005, 0.032, 0.0))
    )
    trunnion_negative = (
        cq.Workplane("YZ")
        .circle(0.0035)
        .extrude(0.010)
        .translate((-0.005, -0.032, 0.0))
    )

    shell = outer.cut(inner)
    bezel = bezel_outer.cut(bezel_inner)
    return shell.union(bezel).union(knuckle).union(trunnion_positive).union(trunnion_negative)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_track_light")

    brushed_nickel = model.material("brushed_nickel", rgba=(0.82, 0.83, 0.84, 1.0))
    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))

    bar = model.part("bar")
    bar.visual(
        mesh_from_cadquery(_bar_shape(), "bar"),
        material=brushed_nickel,
        name="bar_shell",
    )

    pan_limits = MotionLimits(
        effort=8.0,
        velocity=2.0,
        lower=-math.radians(100.0),
        upper=math.radians(100.0),
    )
    tilt_limits = MotionLimits(
        effort=5.0,
        velocity=2.0,
        lower=-math.radians(65.0),
        upper=math.radians(65.0),
    )

    for index, x_pos in enumerate(MOUNT_XS):
        yoke = model.part(f"yoke_{index}")
        yoke.visual(
            mesh_from_cadquery(_yoke_shape(), f"yoke_{index}"),
            material=matte_black,
            name="yoke_shell",
        )

        head = model.part(f"head_{index}")
        head.visual(
            mesh_from_cadquery(_head_shape(), f"head_{index}"),
            material=matte_black,
            name="head_shell",
        )

        model.articulation(
            f"bar_to_yoke_{index}",
            ArticulationType.REVOLUTE,
            parent=bar,
            child=yoke,
            origin=Origin(xyz=(x_pos, 0.0, PAN_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=pan_limits,
        )
        model.articulation(
            f"yoke_{index}_to_head_{index}",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=head,
            origin=Origin(xyz=(0.0, 0.0, YOKE_TILT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=tilt_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bar = object_model.get_part("bar")

    outer_yokes = (object_model.get_part("yoke_0"), object_model.get_part("yoke_2"))
    ctx.expect_origin_distance(
        outer_yokes[0],
        outer_yokes[1],
        axes="x",
        min_dist=0.70,
        max_dist=0.74,
        name="outer yokes are spread across the straight bar",
    )

    for index in range(3):
        yoke = object_model.get_part(f"yoke_{index}")
        head = object_model.get_part(f"head_{index}")
        pan = object_model.get_articulation(f"bar_to_yoke_{index}")
        tilt = object_model.get_articulation(f"yoke_{index}_to_head_{index}")

        pan_limits = pan.motion_limits
        tilt_limits = tilt.motion_limits

        ctx.check(
            f"pan_{index} has realistic pan range",
            pan_limits is not None
            and pan_limits.lower is not None
            and pan_limits.upper is not None
            and pan_limits.lower <= -1.6
            and pan_limits.upper >= 1.6,
            details=f"limits={pan_limits}",
        )
        ctx.check(
            f"tilt_{index} has realistic tilt range",
            tilt_limits is not None
            and tilt_limits.lower is not None
            and tilt_limits.upper is not None
            and tilt_limits.lower <= -1.0
            and tilt_limits.upper >= 1.0,
            details=f"limits={tilt_limits}",
        )

        ctx.allow_overlap(
            head,
            yoke,
            elem_a="head_shell",
            elem_b="yoke_shell",
            reason="The spotlight tilt hinge uses solid trunnion and bushing proxies instead of modeling through-bores for the concealed pivot pin.",
        )

        ctx.expect_gap(
            bar,
            yoke,
            axis="z",
            min_gap=-0.001,
            max_gap=0.001,
            name=f"yoke_{index} seats directly under the bar",
        )
        ctx.expect_gap(
            bar,
            head,
            axis="z",
            min_gap=0.020,
            name=f"head_{index} hangs clearly below the bar",
        )
        ctx.expect_overlap(
            head,
            yoke,
            axes="xy",
            min_overlap=0.012,
            name=f"head_{index} stays centered within its yoke footprint",
        )

        with ctx.pose({tilt: tilt_limits.upper if tilt_limits is not None else 0.0}):
            ctx.expect_gap(
                bar,
                head,
                axis="z",
                min_gap=0.006,
                name=f"head_{index} clears the bar at maximum upward tilt",
            )

    return ctx.report()


object_model = build_object_model()
