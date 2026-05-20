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


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_sensor")

    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    anodized = model.material("anodized", rgba=(0.62, 0.65, 0.69, 1.0))
    housing = model.material("housing", rgba=(0.28, 0.30, 0.33, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.30, 0.40, 0.95))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    crown_radius = 0.055
    crown_height = 0.018
    hub_radius = 0.030
    hub_height = 0.012
    hinge_radius = 0.046
    hinge_z = 0.011

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=crown_radius, length=crown_height),
        origin=Origin(xyz=(0.0, 0.0, crown_height * 0.5)),
        material=charcoal,
        name="crown_shell",
    )
    crown.visual(
        Cylinder(radius=hub_radius, length=hub_height),
        origin=Origin(xyz=(0.0, 0.0, crown_height + (hub_height * 0.5))),
        material=anodized,
        name="top_hub",
    )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        crown.visual(
            Box((0.032, 0.018, 0.020)),
            origin=Origin(
                xyz=(0.046 * math.cos(angle), 0.046 * math.sin(angle), 0.009),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"hinge_tab_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=anodized,
        name="pan_plate",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=anodized,
        name="pan_pedestal",
    )
    yoke.visual(
        Box((0.034, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=anodized,
        name="saddle_bridge",
    )
    for index, side in enumerate((-1.0, 1.0)):
        yoke.visual(
            Box((0.022, 0.012, 0.054)),
            origin=Origin(xyz=(0.0, side * 0.048, 0.071)),
            material=anodized,
            name=f"arm_{index}",
        )
        yoke.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(
                xyz=(0.0, side * 0.055, 0.064),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=charcoal,
            name=f"pivot_cap_{index}",
        )

    sensor = model.part("sensor")
    sensor.visual(
        Box((0.110, 0.062, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=housing,
        name="main_shell",
    )
    sensor.visual(
        Box((0.078, 0.054, 0.016)),
        origin=Origin(xyz=(-0.002, 0.0, 0.056)),
        material=housing,
        name="top_cowl",
    )
    sensor.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=charcoal,
        name="lens_shroud",
    )
    sensor.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=lens_glass,
        name="lens",
    )
    sensor.visual(
        Box((0.034, 0.048, 0.042)),
        origin=Origin(xyz=(-0.038, 0.0, 0.014)),
        material=charcoal,
        name="rear_pack",
    )
    for index, side in enumerate((-1.0, 1.0)):
        sensor.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(
                xyz=(0.0, side * 0.036, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=anodized,
            name=f"trunnion_{index}",
        )

    leg_pitch = math.radians(65.0)
    for index in range(3):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.036, 0.020, 0.016)),
            origin=Origin(xyz=(0.031, 0.0, 0.0)),
            material=charcoal,
            name="hinge_block",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.290),
            origin=Origin(xyz=(0.192, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=charcoal,
            name="upper_tube",
        )
        leg.visual(
            Box((0.024, 0.022, 0.030)),
            origin=Origin(xyz=(0.325, 0.0, 0.0)),
            material=charcoal,
            name="leg_clamp",
        )
        leg.visual(
            Cylinder(radius=0.008, length=0.250),
            origin=Origin(xyz=(0.445, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=anodized,
            name="lower_tube",
        )
        leg.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.578, 0.0, 0.0)),
            material=rubber,
            name="foot",
        )

        angle = (2.0 * math.pi * index) / 3.0
        model.articulation(
            f"leg_fold_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_z),
                rpy=(0.0, leg_pitch, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.5,
                lower=0.0,
                upper=1.05,
            ),
        )

    model.articulation(
        "pan",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, crown_height + hub_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=sensor,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=-0.65,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crown = object_model.get_part("crown")
    yoke = object_model.get_part("yoke")
    sensor = object_model.get_part("sensor")
    leg_0 = object_model.get_part("leg_0")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")
    leg_fold_0 = object_model.get_articulation("leg_fold_0")

    ctx.expect_gap(
        yoke,
        crown,
        axis="z",
        positive_elem="pan_plate",
        negative_elem="top_hub",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan plate seats on crown hub",
    )
    ctx.expect_origin_gap(
        sensor,
        crown,
        axis="z",
        min_gap=0.085,
        name="sensor axis sits above the low tripod crown",
    )
    ctx.expect_within(
        sensor,
        yoke,
        axes="y",
        elem_a="main_shell",
        margin=0.0,
        name="sensor body stays between yoke cheeks",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(sensor, elem="lens"))
    with ctx.pose({pan: 0.7}):
        panned_lens_center = _aabb_center(ctx.part_element_world_aabb(sensor, elem="lens"))
    pan_ok = (
        rest_lens_center is not None
        and panned_lens_center is not None
        and math.hypot(
            panned_lens_center[0] - rest_lens_center[0],
            panned_lens_center[1] - rest_lens_center[1],
        )
        > 0.045
        and abs(panned_lens_center[2] - rest_lens_center[2]) < 0.005
    )
    ctx.check(
        "pan swings the lens around the vertical axis",
        pan_ok,
        details=f"rest={rest_lens_center}, panned={panned_lens_center}",
    )

    with ctx.pose({tilt: 0.75}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(sensor, elem="lens"))
    tilt_ok = (
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.035
    )
    ctx.check(
        "positive tilt raises the front lens",
        tilt_ok,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    rest_foot_center = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    with ctx.pose({leg_fold_0: 0.95}):
        folded_foot_center = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    leg_ok = (
        rest_foot_center is not None
        and folded_foot_center is not None
        and folded_foot_center[2] > rest_foot_center[2] + 0.22
    )
    ctx.check(
        "front leg folds upward from the deployed stance",
        leg_ok,
        details=f"rest={rest_foot_center}, folded={folded_foot_center}",
    )

    return ctx.report()


object_model = build_object_model()
