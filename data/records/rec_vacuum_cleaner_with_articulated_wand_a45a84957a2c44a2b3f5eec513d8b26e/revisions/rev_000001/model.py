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


def _rounded_box(size: tuple[float, float, float], radius: float) -> object:
    """CadQuery rounded rectangular housing, authored directly in meters."""
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_articulated_vacuum")

    satin_graphite = model.material("satin_graphite", rgba=(0.20, 0.22, 0.23, 1.0))
    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.016, 1.0))
    soft_black = model.material("soft_black", rgba=(0.04, 0.045, 0.045, 1.0))
    warm_metal = model.material("warm_satin_metal", rgba=(0.62, 0.59, 0.52, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    smoky_clear = model.material("smoky_clear_polycarbonate", rgba=(0.55, 0.72, 0.82, 0.34))
    amber_accent = model.material("quiet_amber_accent", rgba=(0.95, 0.58, 0.18, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.011, 0.012, 1.0))

    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_rounded_box((0.18, 0.19, 0.31), 0.040), "body_shell"),
        origin=Origin(xyz=(0.0, -0.040, 1.205)),
        material=satin_graphite,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.062, length=0.285),
        origin=Origin(xyz=(0.0, 0.070, 1.145)),
        material=smoky_clear,
        name="dust_bin",
    )
    for z, visual_name in ((1.000, "bin_lower_gasket"), (1.290, "bin_upper_gasket")):
        body.visual(
            Cylinder(radius=0.066, length=0.018),
            origin=Origin(xyz=(0.0, 0.070, z)),
            material=matte_black,
            name=visual_name,
        )
    body.visual(
        Cylinder(radius=0.081, length=0.145),
        origin=Origin(xyz=(0.0, -0.155, 1.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.087, length=0.022),
        origin=Origin(xyz=(0.0, -0.235, 1.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="filter_cap",
    )
    for index, dz in enumerate((-0.042, -0.026, -0.010, 0.006, 0.022, 0.038)):
        body.visual(
            Box((0.102, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, -0.247, 1.265 + dz)),
            material=dark_metal,
            name=f"filter_slit_{index}",
        )
    body.visual(
        Box((0.145, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.132, 1.142)),
        material=matte_black,
        name="body_seam",
    )
    body.visual(
        Box((0.162, 0.052, 0.044)),
        origin=Origin(xyz=(0.0, -0.205, 1.405)),
        material=soft_black,
        name="handle_grip",
    )
    body.visual(
        Box((0.052, 0.046, 0.250)),
        origin=Origin(xyz=(0.0, -0.226, 1.275)),
        material=soft_black,
        name="rear_handle_post",
    )
    body.visual(
        Box((0.132, 0.090, 0.040)),
        origin=Origin(xyz=(0.0, -0.166, 1.128)),
        material=soft_black,
        name="lower_handle_bridge",
    )
    body.visual(
        Box((0.050, 0.040, 0.205)),
        origin=Origin(xyz=(0.0, -0.118, 1.252)),
        material=soft_black,
        name="front_handle_post",
    )
    body.visual(
        Box((0.110, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, -0.052, 1.354)),
        material=matte_black,
        name="top_control_recess",
    )
    body.visual(
        Box((0.180, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, 0.058, 1.030)),
        material=matte_black,
        name="upper_yoke_crossbar",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.225),
        origin=Origin(xyz=(0.0, 0.188, 1.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="upper_pivot_pin",
    )
    for side, x in (("neg", -0.078), ("pos", 0.078)):
        body.visual(
            Box((0.026, 0.170, 0.034)),
            origin=Origin(xyz=(x, 0.128, 1.030)),
            material=matte_black,
            name=f"upper_yoke_arm_{side}",
        )
        body.visual(
            Box((0.026, 0.045, 0.096)),
            origin=Origin(xyz=(x, 0.188, 1.050)),
            material=matte_black,
            name=f"upper_yoke_cheek_{side}",
        )
        body.visual(
            Cylinder(radius=0.027, length=0.020),
            origin=Origin(xyz=(x, 0.188, 1.050), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"upper_yoke_boss_{side}",
        )
    body.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(0.0, -0.288, 1.350), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="trigger_pivot_pin",
    )
    for side, x in (("neg", -0.040), ("pos", 0.040)):
        body.visual(
            Box((0.018, 0.060, 0.060)),
            origin=Origin(xyz=(x, -0.260, 1.372)),
            material=soft_black,
            name=f"trigger_hanger_{side}",
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=amber_accent,
        name="button_cap",
    )
    power_button.visual(
        Box((0.048, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=matte_black,
        name="button_skirt",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.038, 0.020, 0.078)),
        origin=Origin(xyz=(0.0, 0.012, -0.058), rpy=(0.18, 0.0, 0.0)),
        material=amber_accent,
        name="trigger_blade",
    )
    trigger.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="trigger_pin",
    )
    trigger.visual(
        Box((0.036, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.008, -0.022)),
        material=amber_accent,
        name="trigger_root",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.038, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="upper_socket",
    )
    upper_wand.visual(
        Cylinder(radius=0.024, length=0.410),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=warm_metal,
        name="upper_tube",
    )
    for z, visual_name in ((-0.058, "upper_collar"), (-0.392, "elbow_collar")):
        upper_wand.visual(
            Cylinder(radius=0.030, length=0.038),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=matte_black,
            name=visual_name,
        )
    upper_wand.visual(
        Box((0.145, 0.038, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, -0.390)),
        material=matte_black,
        name="elbow_bridge",
    )
    upper_wand.visual(
        Cylinder(radius=0.009, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, -0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="elbow_pivot_pin",
    )
    for side, x in (("neg", -0.066), ("pos", 0.066)):
        upper_wand.visual(
            Box((0.018, 0.038, 0.050)),
            origin=Origin(xyz=(x, 0.0, -0.405)),
            material=matte_black,
            name=f"elbow_side_web_{side}",
        )
        upper_wand.visual(
            Box((0.023, 0.040, 0.083)),
            origin=Origin(xyz=(x, 0.0, -0.460)),
            material=matte_black,
            name=f"elbow_yoke_cheek_{side}",
        )
        upper_wand.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x, 0.0, -0.460), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"elbow_boss_{side}",
        )
    upper_wand.visual(
        Box((0.030, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.025, -0.260)),
        material=amber_accent,
        name="upper_release_tab",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.032, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lower_socket",
    )
    lower_wand.visual(
        Cylinder(radius=0.022, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=warm_metal,
        name="lower_tube",
    )
    for z, visual_name in ((-0.060, "lower_upper_collar"), (-0.428, "nozzle_collar")):
        lower_wand.visual(
            Cylinder(radius=0.028, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=matte_black,
            name=visual_name,
        )
    lower_wand.visual(
        Box((0.160, 0.040, 0.064)),
        origin=Origin(xyz=(0.0, 0.0, -0.405)),
        material=matte_black,
        name="nozzle_bridge",
    )
    lower_wand.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.480), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="nozzle_pivot_pin",
    )
    for side, x in (("neg", -0.071), ("pos", 0.071)):
        lower_wand.visual(
            Box((0.024, 0.043, 0.088)),
            origin=Origin(xyz=(x, 0.0, -0.480)),
            material=matte_black,
            name=f"nozzle_yoke_cheek_{side}",
        )
        lower_wand.visual(
            Cylinder(radius=0.023, length=0.020),
            origin=Origin(xyz=(x, 0.0, -0.480), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"nozzle_boss_{side}",
        )
    lower_wand.visual(
        Box((0.026, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.024, -0.250)),
        material=amber_accent,
        name="lower_release_tab",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Cylinder(radius=0.035, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="nozzle_socket",
    )
    nozzle.visual(
        Box((0.082, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, 0.030, -0.055)),
        material=matte_black,
        name="neck_drop",
    )
    nozzle.visual(
        Box((0.112, 0.100, 0.046)),
        origin=Origin(xyz=(0.0, 0.082, -0.071)),
        material=matte_black,
        name="neck_fairing",
    )
    nozzle.visual(
        mesh_from_cadquery(_rounded_box((0.520, 0.180, 0.070), 0.028), "nozzle_head"),
        origin=Origin(xyz=(0.0, 0.168, -0.075)),
        material=satin_graphite,
        name="nozzle_head",
    )
    nozzle.visual(
        Box((0.490, 0.016, 0.025)),
        origin=Origin(xyz=(0.0, 0.264, -0.066)),
        material=rubber,
        name="front_bumper",
    )
    nozzle.visual(
        Box((0.400, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.197, -0.035)),
        material=smoky_clear,
        name="brush_window",
    )
    nozzle.visual(
        Cylinder(radius=0.024, length=0.392),
        origin=Origin(xyz=(0.0, 0.218, -0.102), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="brush_roller",
    )
    for side, x in (("neg", -0.225), ("pos", 0.225)):
        nozzle.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(xyz=(x, 0.070, -0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"rear_wheel_{side}",
        )
        nozzle.visual(
            Box((0.030, 0.030, 0.028)),
            origin=Origin(xyz=(x, 0.086, -0.088)),
            material=matte_black,
            name=f"wheel_fork_{side}",
        )
    nozzle.visual(
        Box((0.180, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.126, -0.050)),
        material=matte_black,
        name="nozzle_top_seam",
    )

    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.048, -0.052, 1.361)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, -0.288, 1.350)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.42),
    )
    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.188, 1.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=-0.22, upper=0.58),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, -0.460)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-0.42, upper=0.72),
    )
    model.articulation(
        "lower_wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-0.55, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    trigger = object_model.get_part("trigger")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("nozzle")
    power_button = object_model.get_part("power_button")

    upper_joint = object_model.get_articulation("body_to_upper_wand")
    lower_joint = object_model.get_articulation("upper_to_lower_wand")
    nozzle_joint = object_model.get_articulation("lower_wand_to_nozzle")

    ctx.allow_overlap(
        body,
        trigger,
        elem_a="trigger_pivot_pin",
        elem_b="trigger_pin",
        reason="The handle trigger pivots around a small captured metal pin in the grip.",
    )
    ctx.allow_overlap(
        body,
        upper_wand,
        elem_a="upper_pivot_pin",
        elem_b="upper_socket",
        reason="The polished cross pin is intentionally captured inside the upper wand pivot socket.",
    )
    ctx.allow_overlap(
        upper_wand,
        lower_wand,
        elem_a="elbow_pivot_pin",
        elem_b="lower_socket",
        reason="The elbow pin is intentionally nested through the lower wand hinge barrel.",
    )
    ctx.allow_overlap(
        lower_wand,
        nozzle,
        elem_a="nozzle_pivot_pin",
        elem_b="nozzle_socket",
        reason="The nozzle axle is intentionally captured through the floor-head pivot barrel.",
    )

    ctx.expect_within(
        body,
        trigger,
        axes="yz",
        inner_elem="trigger_pivot_pin",
        outer_elem="trigger_pin",
        margin=0.002,
        name="trigger pin is centered in trigger barrel",
    )
    ctx.expect_overlap(
        body,
        trigger,
        axes="x",
        elem_a="trigger_pivot_pin",
        elem_b="trigger_pin",
        min_overlap=0.045,
        name="trigger pin spans trigger width",
    )

    ctx.expect_within(
        body,
        upper_wand,
        axes="yz",
        inner_elem="upper_pivot_pin",
        outer_elem="upper_socket",
        margin=0.002,
        name="upper pivot pin is centered in socket",
    )
    ctx.expect_overlap(
        body,
        upper_wand,
        axes="x",
        elem_a="upper_pivot_pin",
        elem_b="upper_socket",
        min_overlap=0.10,
        name="upper pivot pin spans socket width",
    )
    ctx.expect_within(
        upper_wand,
        lower_wand,
        axes="yz",
        inner_elem="elbow_pivot_pin",
        outer_elem="lower_socket",
        margin=0.002,
        name="elbow pin is centered in socket",
    )
    ctx.expect_overlap(
        upper_wand,
        lower_wand,
        axes="x",
        elem_a="elbow_pivot_pin",
        elem_b="lower_socket",
        min_overlap=0.09,
        name="elbow pin spans hinge barrel",
    )
    ctx.expect_within(
        lower_wand,
        nozzle,
        axes="yz",
        inner_elem="nozzle_pivot_pin",
        outer_elem="nozzle_socket",
        margin=0.002,
        name="nozzle axle is centered in socket",
    )
    ctx.expect_overlap(
        lower_wand,
        nozzle,
        axes="x",
        elem_a="nozzle_pivot_pin",
        elem_b="nozzle_socket",
        min_overlap=0.10,
        name="nozzle axle spans pivot barrel",
    )

    ctx.expect_gap(
        power_button,
        body,
        axis="z",
        positive_elem="button_skirt",
        negative_elem="top_control_recess",
        min_gap=0.0,
        max_gap=0.006,
        name="power button sits proud in its recess",
    )

    rest_nozzle = ctx.part_world_position(nozzle)
    with ctx.pose({upper_joint: 0.45, lower_joint: 0.38, nozzle_joint: -0.30}):
        posed_nozzle = ctx.part_world_position(nozzle)
        ctx.expect_origin_gap(
            nozzle,
            body,
            axis="y",
            min_gap=0.18,
            name="articulated wand carries nozzle forward",
        )

    ctx.check(
        "wand chain changes nozzle pose",
        rest_nozzle is not None
        and posed_nozzle is not None
        and posed_nozzle[1] > rest_nozzle[1] + 0.20,
        details=f"rest={rest_nozzle}, posed={posed_nozzle}",
    )

    return ctx.report()


object_model = build_object_model()
