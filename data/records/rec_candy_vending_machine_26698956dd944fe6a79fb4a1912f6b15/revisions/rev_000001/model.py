from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_gumball_machine")

    cast_red = model.material("cast_red", rgba=(0.70, 0.10, 0.11, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.85, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.63, 0.23, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.83, 0.92, 0.99, 0.26))
    slot_black = model.material("slot_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.086, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=cast_red,
        name="foot",
    )
    base.visual(
        Cylinder(radius=0.074, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=cast_red,
        name="lower_body",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=cast_red,
        name="neck",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=cast_red,
        name="shoulder",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=cast_red,
        name="seat",
    )
    base.visual(
        Box((0.094, 0.014, 0.092)),
        origin=Origin(xyz=(0.0, 0.075, 0.098)),
        material=chrome,
        name="coin_mech",
    )
    base.visual(
        Cylinder(radius=0.033, length=0.014),
        origin=Origin(xyz=(0.0, 0.082, 0.104), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_boss",
    )
    base.visual(
        Box((0.034, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, 0.0835, 0.136)),
        material=slot_black,
        name="coin_slot",
    )
    base.visual(
        Box((0.052, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.091, 0.054), rpy=(-0.36, 0.0, 0.0)),
        material=chrome,
        name="chute_floor",
    )
    base.visual(
        Box((0.004, 0.028, 0.020)),
        origin=Origin(xyz=(-0.024, 0.091, 0.060), rpy=(-0.36, 0.0, 0.0)),
        material=chrome,
        name="chute_wall_0",
    )
    base.visual(
        Box((0.004, 0.028, 0.020)),
        origin=Origin(xyz=(0.024, 0.091, 0.060), rpy=(-0.36, 0.0, 0.0)),
        material=chrome,
        name="chute_wall_1",
    )
    base.visual(
        Box((0.044, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.103, 0.046), rpy=(-0.36, 0.0, 0.0)),
        material=chrome,
        name="chute_lip",
    )
    base.visual(
        Box((0.018, 0.006, 0.036)),
        origin=Origin(xyz=(0.050, 0.083, 0.084)),
        material=chrome,
        name="coin_return_leaf",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.058, 0.086, 0.084)),
        material=chrome,
        name="coin_return_barrel",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=0.112),
        material=clear_glass,
        name="shell",
    )
    globe.visual(
        Sphere(radius=0.106),
        material=clear_glass,
        name="inner_shell",
    )
    globe.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=chrome,
        name="collar",
    )
    globe.visual(
        Box((0.014, 0.014, 0.014)),
        origin=Origin(xyz=(-0.024, -0.060, 0.100)),
        material=chrome,
        name="cap_ear_0",
    )
    globe.visual(
        Box((0.014, 0.014, 0.014)),
        origin=Origin(xyz=(0.024, -0.060, 0.100)),
        material=chrome,
        name="cap_ear_1",
    )
    globe.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(-0.024, -0.066, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="cap_barrel_0",
    )
    globe.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.024, -0.066, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="cap_barrel_1",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="knob_grip",
    )

    coin_return = model.part("coin_return")
    coin_return.visual(
        Box((0.030, 0.006, 0.036)),
        origin=Origin(xyz=(-0.015, 0.003, 0.0)),
        material=chrome,
        name="flap",
    )
    coin_return.visual(
        Box((0.004, 0.006, 0.036)),
        origin=Origin(xyz=(-0.002, 0.003, 0.0)),
        material=chrome,
        name="hinge_leaf",
    )
    coin_return.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.003, -0.013)),
        material=chrome,
        name="hinge_barrel_0",
    )
    coin_return.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.003, 0.013)),
        material=chrome,
        name="hinge_barrel_1",
    )
    coin_return.visual(
        Box((0.014, 0.010, 0.006)),
        origin=Origin(xyz=(-0.018, 0.008, -0.012)),
        material=chrome,
        name="pull_lip",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.064, length=0.008),
        origin=Origin(xyz=(0.0, 0.057, 0.004)),
        material=cast_red,
        name="cap_shell",
    )
    cap.visual(
        Box((0.030, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, 0.003)),
        material=cast_red,
        name="hinge_leaf",
    )
    cap.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    cap.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.057, 0.014)),
        material=chrome,
        name="handle",
    )

    model.articulation(
        "base_to_globe",
        ArticulationType.FIXED,
        parent=base,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.0, 0.089, 0.104)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "base_to_coin_return",
        ArticulationType.REVOLUTE,
        parent=base,
        child=coin_return,
        origin=Origin(xyz=(0.058, 0.083, 0.084)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.0, effort=1.0, velocity=3.0),
    )
    model.articulation(
        "globe_to_cap",
        ArticulationType.REVOLUTE,
        parent=globe,
        child=cap,
        origin=Origin(xyz=(0.0, -0.066, 0.106)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=1.5, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    globe = object_model.get_part("globe")
    knob = object_model.get_part("knob")
    coin_return = object_model.get_part("coin_return")
    cap = object_model.get_part("cap")

    knob_joint = object_model.get_articulation("base_to_knob")
    coin_return_joint = object_model.get_articulation("base_to_coin_return")
    cap_joint = object_model.get_articulation("globe_to_cap")

    ctx.check(
        "knob_joint_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )
    ctx.check(
        "coin_return_joint_is_revolute",
        coin_return_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={coin_return_joint.articulation_type!r}",
    )
    ctx.check(
        "cap_joint_is_revolute",
        cap_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={cap_joint.articulation_type!r}",
    )

    ctx.allow_overlap(
        cap,
        globe,
        elem_a="cap_shell",
        elem_b="shell",
        reason="The clear globe is approximated as a solid sphere proxy, so the closed refill cap lightly occupies the implied opening volume.",
    )

    ctx.expect_gap(
        globe,
        base,
        axis="z",
        positive_elem="shell",
        negative_elem="seat",
        min_gap=0.0,
        max_gap=0.001,
        name="globe sits just above the cast base seat",
    )
    ctx.expect_overlap(
        globe,
        base,
        axes="xy",
        elem_a="shell",
        elem_b="seat",
        min_overlap=0.12,
        name="globe is centered over the base",
    )
    ctx.expect_gap(
        knob,
        base,
        axis="y",
        positive_elem="knob_body",
        negative_elem="coin_mech",
        min_gap=0.0,
        max_gap=0.040,
        name="dispense knob stands proud of the coin mechanism",
    )
    ctx.expect_overlap(
        cap,
        globe,
        axes="xy",
        elem_a="cap_shell",
        elem_b="collar",
        min_overlap=0.10,
        name="refill cap covers the globe opening when closed",
    )
    ctx.expect_gap(
        cap,
        globe,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="collar",
        min_gap=0.0,
        max_gap=0.010,
        name="closed refill cap sits just above the collar",
    )

    cap_closed = ctx.part_world_aabb(cap)
    with ctx.pose({cap_joint: 1.25}):
        cap_open = ctx.part_world_aabb(cap)
    ctx.check(
        "cap opens upward",
        cap_closed is not None
        and cap_open is not None
        and float(cap_open[1][2]) > float(cap_closed[1][2]) + 0.040,
        details=f"closed={cap_closed!r}, open={cap_open!r}",
    )

    flap_closed = ctx.part_world_aabb(coin_return)
    with ctx.pose({coin_return_joint: 1.0}):
        flap_open = ctx.part_world_aabb(coin_return)
    ctx.check(
        "coin_return_swings_forward",
        flap_closed is not None
        and flap_open is not None
        and float(flap_open[1][1]) > float(flap_closed[1][1]) + 0.015,
        details=f"closed={flap_closed!r}, open={flap_open!r}",
    )

    return ctx.report()


object_model = build_object_model()
