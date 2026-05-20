from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_serial_elbow_arm")

    housing_mat = Material("satin_charcoal_housing", color=(0.12, 0.13, 0.14, 1.0))
    bolt_mat = Material("dark_bolts", color=(0.03, 0.03, 0.035, 1.0))
    upper_mat = Material("blue_upper_link", color=(0.05, 0.22, 0.70, 1.0))
    elbow_mat = Material("orange_elbow_hub", color=(0.95, 0.38, 0.08, 1.0))
    forearm_mat = Material("warm_gray_forearm", color=(0.45, 0.47, 0.50, 1.0))
    pad_mat = Material("black_rubber_pad", color=(0.02, 0.022, 0.02, 1.0))

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Box((0.46, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=housing_mat,
        name="ground_foot",
    )
    shoulder_housing.visual(
        Box((0.22, 0.22, 0.27)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=housing_mat,
        name="upright_block",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.11, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.38), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_mat,
        name="shoulder_barrel",
    )
    for ix, x in enumerate((-0.16, 0.16)):
        for iy, y in enumerate((-0.11, 0.11)):
            shoulder_housing.visual(
                Cylinder(radius=0.024, length=0.012),
                origin=Origin(xyz=(x, y, 0.064)),
                material=bolt_mat,
                name=f"foot_bolt_{ix}_{iy}",
            )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Box((0.345, 0.080, 0.060)),
        origin=Origin(xyz=(0.2875, 0.0, 0.0)),
        material=upper_mat,
        name="straight_beam",
    )
    upper_link.visual(
        Box((0.025, 0.300, 0.055)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=upper_mat,
        name="shoulder_yoke_bridge",
    )
    for suffix, y in (("0", -0.135), ("1", 0.135)):
        upper_link.visual(
            Box((0.170, 0.030, 0.180)),
            origin=Origin(xyz=(0.050, y, 0.0)),
            material=upper_mat,
            name=f"shoulder_yoke_ear_{suffix}",
        )
    upper_link.visual(
        Cylinder(radius=0.045, length=0.320),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_mat,
        name="shoulder_pin",
    )
    upper_link.visual(
        Cylinder(radius=0.130, length=0.220),
        origin=Origin(xyz=(0.52, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=elbow_mat,
        name="elbow_hub",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.200, 0.140, 0.050)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=forearm_mat,
        name="short_beam",
    )
    forearm.visual(
        Box((0.030, 0.320, 0.055)),
        origin=Origin(xyz=(0.149, 0.0, 0.0)),
        material=forearm_mat,
        name="elbow_yoke_bridge",
    )
    for suffix, y in (("0", -0.142), ("1", 0.142)):
        forearm.visual(
            Box((0.230, 0.035, 0.220)),
            origin=Origin(xyz=(0.035, y, 0.0)),
            material=forearm_mat,
            name=f"elbow_yoke_ear_{suffix}",
        )
    forearm.visual(
        Cylinder(radius=0.050, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_mat,
        name="elbow_pin",
    )
    forearm.visual(
        Box((0.170, 0.220, 0.070)),
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        material=forearm_mat,
        name="rectangular_pad_body",
    )
    forearm.visual(
        Box((0.014, 0.240, 0.086)),
        origin=Origin(xyz=(0.510, 0.0, 0.0)),
        material=pad_mat,
        name="rubber_pad_face",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=shoulder_housing,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-0.70, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.2, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        shoulder_housing,
        upper_link,
        elem_a="shoulder_barrel",
        elem_b="shoulder_pin",
        reason="The upper link has a captured shoulder pin running through the shoulder bearing barrel.",
    )
    ctx.expect_within(
        upper_link,
        shoulder_housing,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_barrel",
        margin=0.0,
        name="shoulder pin sits inside shoulder barrel",
    )
    ctx.expect_overlap(
        upper_link,
        shoulder_housing,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        min_overlap=0.20,
        name="shoulder pin passes through the barrel",
    )

    ctx.allow_overlap(
        upper_link,
        forearm,
        elem_a="elbow_hub",
        elem_b="elbow_pin",
        reason="The forearm elbow pin is intentionally captured by the broad elbow hub.",
    )
    ctx.expect_within(
        forearm,
        upper_link,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.0,
        name="elbow pin sits inside elbow hub",
    )
    ctx.expect_overlap(
        forearm,
        upper_link,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.20,
        name="elbow pin passes through the hub",
    )

    ctx.check(
        "shoulder and elbow are revolute joints",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
        details=f"shoulder={shoulder.articulation_type}, elbow={elbow.articulation_type}",
    )
    ctx.check(
        "joint axes are parallel horizontal",
        abs(shoulder.axis[0]) < 1e-9
        and abs(elbow.axis[0]) < 1e-9
        and abs(abs(shoulder.axis[1]) - 1.0) < 1e-9
        and abs(abs(elbow.axis[1]) - 1.0) < 1e-9
        and abs(shoulder.axis[2]) < 1e-9
        and abs(elbow.axis[2]) < 1e-9
        and shoulder.axis[1] * elbow.axis[1] > 0.0,
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )

    housing_aabb = ctx.part_world_aabb(shoulder_housing)
    ctx.check(
        "shoulder housing is grounded",
        housing_aabb is not None and abs(housing_aabb[0][2]) < 1e-6,
        details=f"housing_aabb={housing_aabb}",
    )

    upper_beam = upper_link.get_visual("straight_beam").geometry
    forearm_beam = forearm.get_visual("short_beam").geometry
    ctx.check(
        "links have different proportions",
        upper_beam.size[0] > forearm_beam.size[0] * 1.5
        and upper_beam.size[1] < forearm_beam.size[1]
        and upper_beam.size[2] > forearm_beam.size[2],
        details=f"upper={upper_beam.size}, forearm={forearm_beam.size}",
    )

    rest_upper_hub = ctx.part_element_world_aabb(upper_link, elem="elbow_hub")
    rest_pad = ctx.part_element_world_aabb(forearm, elem="rectangular_pad_body")
    with ctx.pose({shoulder: 0.45, elbow: 0.75}):
        raised_upper_hub = ctx.part_element_world_aabb(upper_link, elem="elbow_hub")
        raised_pad = ctx.part_element_world_aabb(forearm, elem="rectangular_pad_body")
    ctx.check(
        "positive joint motion lifts the arm in a vertical plane",
        rest_upper_hub is not None
        and rest_pad is not None
        and raised_upper_hub is not None
        and raised_pad is not None
        and (raised_upper_hub[0][2] + raised_upper_hub[1][2]) / 2.0
        > (rest_upper_hub[0][2] + rest_upper_hub[1][2]) / 2.0 + 0.08
        and (raised_pad[0][2] + raised_pad[1][2]) / 2.0
        > (rest_pad[0][2] + rest_pad[1][2]) / 2.0 + 0.18,
        details=f"rest_hub={rest_upper_hub}, raised_hub={raised_upper_hub}, rest_pad={rest_pad}, raised_pad={raised_pad}",
    )

    return ctx.report()


object_model = build_object_model()
