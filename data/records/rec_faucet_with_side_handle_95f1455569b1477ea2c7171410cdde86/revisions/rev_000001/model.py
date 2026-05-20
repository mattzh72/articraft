from __future__ import annotations

from math import pi

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


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_faucet")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stainless,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=stainless,
        name="valve_body",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=stainless,
        name="swivel_collar",
    )
    body.visual(
        Box((0.034, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.021, 0.052)),
        material=stainless,
        name="handle_mount",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.011, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=stainless,
        name="swivel_stem",
    )
    spout.visual(
        Cylinder(radius=0.009, length=0.214),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=stainless,
        name="upright_tube",
    )
    spout.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=stainless,
        name="elbow_hub",
    )
    spout.visual(
        Cylinder(radius=0.009, length=0.176),
        origin=Origin(xyz=(0.088, 0.0, 0.214), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="spout_tube",
    )
    spout.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.176, 0.0, 0.214), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="tip_collar",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="pivot_hub",
    )
    handle.visual(
        Box((0.014, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=stainless,
        name="pivot_neck",
    )
    handle.visual(
        Box((0.010, 0.070, 0.044)),
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        material=graphite,
        name="paddle",
    )
    handle.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.093, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="paddle_tip",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Box((0.008, 0.016, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=stainless,
        name="hinge_block",
    )
    nozzle.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="nozzle_body",
    )
    nozzle.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.030, 0.0, -0.007)),
        material=stainless,
        name="outlet_collar",
    )
    nozzle.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(0.030, 0.0, -0.023)),
        material=dark_rubber,
        name="outlet_tube",
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.034, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.55, upper=1.05),
    )
    model.articulation(
        "spout_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=nozzle,
        origin=Origin(xyz=(0.190, 0.0, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=-0.45, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    nozzle = object_model.get_part("nozzle")

    spout_joint = object_model.get_articulation("body_to_spout")
    handle_joint = object_model.get_articulation("body_to_handle")
    nozzle_joint = object_model.get_articulation("spout_to_nozzle")

    ctx.expect_gap(
        spout,
        body,
        axis="z",
        positive_elem="swivel_stem",
        negative_elem="swivel_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="spout stem seats on the swivel collar",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xy",
        elem_a="swivel_stem",
        elem_b="swivel_collar",
        min_overlap=0.020,
        name="spout remains centered over the base swivel",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="y",
        positive_elem="pivot_hub",
        negative_elem="handle_mount",
        max_gap=0.001,
        max_penetration=0.0,
        name="handle hub meets the side pivot mount",
    )
    ctx.expect_gap(
        nozzle,
        spout,
        axis="x",
        positive_elem="hinge_block",
        negative_elem="tip_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="nozzle hinges directly from the spout tip",
    )

    rest_nozzle_pos = ctx.part_world_position(nozzle)
    with ctx.pose({spout_joint: pi / 2.0}):
        quarter_turn_nozzle_pos = ctx.part_world_position(nozzle)
    ctx.check(
        "spout quarter turn swings the tip sideways",
        rest_nozzle_pos is not None
        and quarter_turn_nozzle_pos is not None
        and rest_nozzle_pos[0] > 0.16
        and abs(quarter_turn_nozzle_pos[0]) < 0.03
        and quarter_turn_nozzle_pos[1] > 0.16,
        details=f"rest={rest_nozzle_pos}, quarter_turn={quarter_turn_nozzle_pos}",
    )

    rest_handle_tip = ctx.part_element_world_aabb(handle, elem="paddle_tip")
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        raised_handle_tip = ctx.part_element_world_aabb(handle, elem="paddle_tip")
    ctx.check(
        "paddle handle lifts at the open limit",
        _aabb_center_z(rest_handle_tip) is not None
        and _aabb_center_z(raised_handle_tip) is not None
        and _aabb_center_z(raised_handle_tip) > _aabb_center_z(rest_handle_tip) + 0.035,
        details=f"rest={rest_handle_tip}, raised={raised_handle_tip}",
    )

    rest_outlet = ctx.part_element_world_aabb(nozzle, elem="outlet_tube")
    with ctx.pose({nozzle_joint: nozzle_joint.motion_limits.upper}):
        dropped_outlet = ctx.part_element_world_aabb(nozzle, elem="outlet_tube")
    ctx.check(
        "nozzle tilts the outlet downward",
        _aabb_center_z(rest_outlet) is not None
        and _aabb_center_z(dropped_outlet) is not None
        and _aabb_center_z(dropped_outlet) < _aabb_center_z(rest_outlet) - 0.010,
        details=f"rest={rest_outlet}, dropped={dropped_outlet}",
    )

    return ctx.report()


object_model = build_object_model()
