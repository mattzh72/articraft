from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Origin,
    MotionLimits,
    Sphere,
    TestContext,
    TestReport,
)


def _x_cylinder(radius: float, length: float, *, xyz: tuple[float, float, float], name: str | None = None):
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)), name


def _y_cylinder(radius: float, length: float, *, xyz: tuple[float, float, float], name: str | None = None):
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)), name


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_pot_filler_faucet")

    brass = model.material("brass", rgba=(0.78, 0.67, 0.42, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.60, 0.50, 0.30, 1.0))
    seal_black = model.material("seal_black", rgba=(0.10, 0.10, 0.11, 1.0))

    valve_body = model.part("valve_body")
    for geometry, origin, name in (
        _x_cylinder(0.056, 0.008, xyz=(0.004, 0.0, 0.0), name="wall_plate"),
        _x_cylinder(0.018, 0.022, xyz=(0.019, 0.0, 0.0), name="mount_neck"),
        _x_cylinder(0.028, 0.068, xyz=(0.056, 0.0, 0.0), name="body_barrel"),
        _x_cylinder(0.022, 0.013, xyz=(0.0865, 0.0, 0.0), name="front_sleeve"),
    ):
        valve_body.visual(geometry, origin=origin, material=brass, name=name)
    valve_body.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.110, 0.0, -0.012)),
        material=brass_dark,
        name="wall_knuckle_lower",
    )
    valve_body.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.052, 0.018, 0.0)),
        material=brass_dark,
        name="handle_hub",
    )
    handle_boss_geom, handle_boss_origin, _ = _y_cylinder(0.010, 0.026, xyz=(0.052, 0.031, 0.0))
    valve_body.visual(handle_boss_geom, origin=handle_boss_origin, material=brass_dark, name="handle_boss")

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brass_dark,
        name="wall_knuckle_upper",
    )
    inner_tube_geom, inner_tube_origin, _ = _x_cylinder(0.011, 0.160, xyz=(0.080, 0.0, 0.012))
    inner_arm.visual(inner_tube_geom, origin=inner_tube_origin, material=brass, name="inner_tube")
    elbow_housing_geom, elbow_housing_origin, _ = _x_cylinder(0.013, 0.020, xyz=(0.158, 0.0, 0.012))
    inner_arm.visual(elbow_housing_geom, origin=elbow_housing_origin, material=brass_dark, name="elbow_housing")
    inner_arm.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
        material=brass_dark,
        name="elbow_knuckle_lower",
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brass_dark,
        name="elbow_knuckle_upper",
    )
    outer_tube_geom, outer_tube_origin, _ = _x_cylinder(0.011, 0.235, xyz=(0.1175, 0.0, 0.012))
    outer_arm.visual(outer_tube_geom, origin=outer_tube_origin, material=brass, name="outer_tube")
    spout_collar_geom, spout_collar_origin, _ = _x_cylinder(0.014, 0.024, xyz=(0.223, 0.0, 0.012))
    outer_arm.visual(spout_collar_geom, origin=spout_collar_origin, material=brass_dark, name="spout_collar")
    outer_arm.visual(
        Cylinder(radius=0.010, length=0.086),
        origin=Origin(xyz=(0.235, 0.0, -0.031)),
        material=brass,
        name="spout_drop",
    )
    outer_arm.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.235, 0.0, -0.083)),
        material=brass_dark,
        name="nozzle_tip",
    )
    outer_arm.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.235, 0.0, -0.095)),
        material=seal_black,
        name="aerator",
    )

    handle = model.part("handle")
    handle_stem_geom, handle_stem_origin, _ = _y_cylinder(0.007, 0.014, xyz=(0.0, 0.007, 0.0))
    handle.visual(handle_stem_geom, origin=handle_stem_origin, material=brass_dark, name="handle_stem")
    handle.visual(
        Box((0.046, 0.010, 0.012)),
        origin=Origin(xyz=(0.023, 0.010, -0.008)),
        material=brass,
        name="handle_paddle",
    )
    lever_tip_geom, lever_tip_origin, _ = _x_cylinder(0.006, 0.022, xyz=(0.051, 0.010, -0.008))
    handle.visual(lever_tip_geom, origin=lever_tip_origin, material=brass_dark, name="lever_tip")

    model.articulation(
        "inner_swing",
        ArticulationType.REVOLUTE,
        parent=valve_body,
        child=inner_arm,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-1.9, upper=1.9),
    )
    model.articulation(
        "outer_swing",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(0.185, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=valve_body,
        child=handle,
        origin=Origin(xyz=(0.052, 0.044, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.45, upper=0.75),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    valve_body = object_model.get_part("valve_body")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    handle = object_model.get_part("handle")
    inner_swing = object_model.get_articulation("inner_swing")
    outer_swing = object_model.get_articulation("outer_swing")
    handle_turn = object_model.get_articulation("handle_turn")

    ctx.expect_contact(
        inner_arm,
        valve_body,
        elem_a="wall_knuckle_upper",
        elem_b="wall_knuckle_lower",
        name="wall swivel knuckle halves meet cleanly",
    )
    ctx.expect_contact(
        inner_arm,
        outer_arm,
        elem_a="elbow_knuckle_lower",
        elem_b="elbow_knuckle_upper",
        name="elbow swivel knuckle halves meet cleanly",
    )

    ctx.expect_gap(
        outer_arm,
        valve_body,
        axis="x",
        positive_elem="nozzle_tip",
        negative_elem="wall_plate",
        min_gap=0.49,
        name="extended spout reaches well clear of the wall plate",
    )

    with ctx.pose({inner_swing: pi / 2.0}):
        ctx.expect_gap(
            outer_arm,
            valve_body,
            axis="y",
            positive_elem="nozzle_tip",
            negative_elem="body_barrel",
            min_gap=0.34,
            name="wall hinge swings the nozzle far to the side",
        )

    with ctx.pose({outer_swing: pi / 2.0}):
        ctx.expect_gap(
            outer_arm,
            inner_arm,
            axis="y",
            positive_elem="nozzle_tip",
            negative_elem="elbow_knuckle_lower",
            min_gap=0.20,
            name="elbow hinge folds the outer arm sideways",
        )

    rest_tip = _aabb_center(ctx.part_element_world_aabb(handle, elem="lever_tip"))
    with ctx.pose({handle_turn: 0.65}):
        open_tip = _aabb_center(ctx.part_element_world_aabb(handle, elem="lever_tip"))
    ctx.check(
        "side handle lifts on its short pivot",
        rest_tip is not None and open_tip is not None and open_tip[2] > rest_tip[2] + 0.025,
        details=f"rest_tip={rest_tip}, open_tip={open_tip}",
    )

    return ctx.report()


object_model = build_object_model()
