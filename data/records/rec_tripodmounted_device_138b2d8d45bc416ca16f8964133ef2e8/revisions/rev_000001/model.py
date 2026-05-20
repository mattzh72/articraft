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
    Sphere,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rpy_align_z_to(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an Origin.rpy that aims a local +Z cylinder along *vector*."""
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 0.0:
        return (0.0, 0.0, 0.0)
    x, y, z = x / length, y / length, z / length
    pitch = math.acos(max(-1.0, min(1.0, z)))
    yaw = math.atan2(y, x)
    return (0.0, pitch, yaw)


def _rounded_sensor_shell() -> cq.Workplane:
    """A compact rounded instrument housing, centered at its local origin."""
    shell = cq.Workplane("XY").box(0.106, 0.088, 0.074)
    return shell.edges().fillet(0.010)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sensor_tripod")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.08, 0.085, 0.09, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.23, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.56, 0.55, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.02, 0.05, 0.075, 0.92))
    indicator = model.material("soft_green_indicator", rgba=(0.15, 0.85, 0.42, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.054, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.389)),
        material=graphite,
        name="low_crown",
    )
    crown.visual(
        Cylinder(radius=0.040, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.431)),
        material=graphite,
        name="top_bearing",
    )

    hinge_radius = 0.072
    hinge_z = 0.382
    lug_offset = 0.016
    lug_length = 0.014
    lug_radius = 0.010
    leg_radial_reach = 0.180
    leg_drop = -0.342

    legs = []
    for index in range(3):
        theta = 2.0 * math.pi * index / 3.0 + math.pi / 2.0
        er = (math.cos(theta), math.sin(theta), 0.0)
        et = (-math.sin(theta), math.cos(theta), 0.0)
        hinge_xyz = (hinge_radius * er[0], hinge_radius * er[1], hinge_z)
        lug_rpy = _rpy_align_z_to(et)

        for side in (-1.0, 1.0):
            lug_center = (
                hinge_xyz[0] + side * lug_offset * et[0],
                hinge_xyz[1] + side * lug_offset * et[1],
                hinge_xyz[2],
            )
            block_center = (
                0.061 * er[0] + side * lug_offset * et[0],
                0.061 * er[1] + side * lug_offset * et[1],
                hinge_z - 0.003,
            )
            crown.visual(
                Box((0.026, 0.012, 0.018)),
                origin=Origin(xyz=block_center, rpy=(0.0, 0.0, theta)),
                material=graphite,
                name=f"hinge_web_{index}_{0 if side < 0.0 else 1}",
            )
            crown.visual(
                Cylinder(radius=lug_radius, length=lug_length),
                origin=Origin(xyz=lug_center, rpy=lug_rpy),
                material=satin_metal,
                name=f"hinge_lug_{index}_{0 if side < 0.0 else 1}",
            )

        leg = model.part(f"leg_{index}")
        legs.append(leg)
        leg.visual(
            Cylinder(radius=0.0085, length=0.018),
            origin=Origin(rpy=lug_rpy),
            material=satin_metal,
            name="hinge_barrel",
        )

        leg_vector = (
            leg_radial_reach * er[0],
            leg_radial_reach * er[1],
            leg_drop,
        )
        leg_length = math.sqrt(
            leg_vector[0] * leg_vector[0]
            + leg_vector[1] * leg_vector[1]
            + leg_vector[2] * leg_vector[2]
        )
        leg.visual(
            Cylinder(radius=0.0075, length=leg_length),
            origin=Origin(
                xyz=(leg_vector[0] * 0.5, leg_vector[1] * 0.5, leg_vector[2] * 0.5),
                rpy=_rpy_align_z_to(leg_vector),
            ),
            material=matte_black,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=leg_vector),
            material=rubber,
            name="rubber_foot",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=hinge_xyz),
            axis=(-et[0], -et[1], -et[2]),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.15, upper=1.05),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.043, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_metal,
        name="turntable",
    )
    pan_head.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=graphite,
        name="pan_neck",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.180, 0.066, 0.120),
            span_width=0.126,
            trunnion_diameter=0.028,
            trunnion_center_z=0.083,
            base_thickness=0.018,
            corner_radius=0.004,
            center=False,
        ),
        "trunnion_yoke",
    )
    pan_head.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=graphite,
        name="yoke",
    )

    pan_joint = model.articulation(
        "crown_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    sensor = model.part("sensor_body")
    sensor.visual(
        mesh_from_cadquery(_rounded_sensor_shell(), "sensor_shell", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.012, -0.004)),
        material=dark_polymer,
        name="sensor_shell",
    )
    sensor.visual(
        Cylinder(radius=0.0105, length=0.186),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="trunnion_pin",
    )
    for side in (-1.0, 1.0):
        sensor.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(side * 0.093, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"trunnion_cap_{0 if side < 0.0 else 1}",
        )
    sensor.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.059, -0.005), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="front_bezel",
    )
    sensor.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.064, -0.005), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    sensor.visual(
        Box((0.030, 0.004, 0.009)),
        origin=Origin(xyz=(0.0, 0.058, 0.027)),
        material=indicator,
        name="status_window",
    )

    model.articulation(
        "pan_head_to_sensor_body",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=sensor,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.70, upper=0.80),
    )

    # Keep these locals referenced during authoring; the returned model owns them.
    _ = (legs, pan_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    sensor = object_model.get_part("sensor_body")
    pan = object_model.get_articulation("crown_to_pan_head")
    tilt = object_model.get_articulation("pan_head_to_sensor_body")
    leg_0 = object_model.get_part("leg_0")
    leg_hinge = object_model.get_articulation("crown_to_leg_0")

    ctx.expect_gap(
        pan_head,
        crown,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan turntable seats on crown bearing",
    )
    ctx.expect_within(
        sensor,
        pan_head,
        axes="x",
        inner_elem="sensor_shell",
        outer_elem="yoke",
        margin=0.002,
        name="sensor shell fits between yoke cheeks",
    )
    ctx.expect_overlap(
        sensor,
        pan_head,
        axes="x",
        elem_a="trunnion_pin",
        elem_b="yoke",
        min_overlap=0.080,
        name="tilt pin spans the yoke cheeks",
    )

    rest_foot = ctx.part_element_world_aabb(leg_0, elem="rubber_foot")
    with ctx.pose({leg_hinge: 0.85}):
        folded_foot = ctx.part_element_world_aabb(leg_0, elem="rubber_foot")
    ctx.check(
        "leg hinge folds foot upward",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[0][2] > rest_foot[0][2] + 0.10,
        details=f"rest={rest_foot}, folded={folded_foot}",
    )

    rest_lens = ctx.part_element_world_aabb(sensor, elem="front_lens")
    with ctx.pose({tilt: 0.55}):
        tilted_lens = ctx.part_element_world_aabb(sensor, elem="front_lens")
    ctx.check(
        "tilt hinge raises sensor front",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[1][2] > rest_lens[1][2] + 0.020,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    with ctx.pose({pan: 1.0}):
        panned_lens = ctx.part_element_world_aabb(sensor, elem="front_lens")
    ctx.check(
        "pan joint rotates sensor around tripod axis",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1][0] < rest_lens[0][0] - 0.010,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
