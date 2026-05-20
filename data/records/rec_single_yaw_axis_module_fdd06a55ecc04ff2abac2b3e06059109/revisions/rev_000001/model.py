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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_yaw_carriage")

    painted_steel = Material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_bearing = Material("dark_bearing", rgba=(0.05, 0.055, 0.06, 1.0))
    machined_metal = Material("machined_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    carriage_blue = Material("carriage_blue", rgba=(0.05, 0.22, 0.55, 1.0))
    lower_mass = Material("lower_mass", rgba=(0.11, 0.12, 0.13, 1.0))
    bolt_black = Material("black_fasteners", rgba=(0.015, 0.015, 0.018, 1.0))

    hub_z = 0.47
    ring_height = 0.065
    bearing_ring = (
        cq.Workplane("XY")
        .circle(0.110)
        .circle(0.058)
        .extrude(ring_height)
        .translate((0.0, 0.0, -ring_height / 2.0))
    )

    frame = model.part("frame")
    frame.visual(
        Box((0.62, 0.36, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=painted_steel,
        name="floor_base",
    )
    for index, x in enumerate((-0.255, 0.255)):
        frame.visual(
            Box((0.055, 0.30, 0.520)),
            origin=Origin(xyz=(x, 0.0, 0.310)),
            material=painted_steel,
            name=f"side_post_{index}",
        )
    frame.visual(
        Box((0.62, 0.32, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=painted_steel,
        name="top_bridge",
    )
    frame.visual(
        mesh_from_cadquery(bearing_ring, "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, hub_z)),
        material=dark_bearing,
        name="bearing_ring",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.080, 0.125, 0.012)),
            origin=Origin(xyz=(0.0, y_sign * 0.110, 0.506)),
            material=dark_bearing,
            name=f"saddle_flange_{index}",
        )
        frame.visual(
            Box((0.080, 0.030, 0.048)),
            origin=Origin(xyz=(0.0, y_sign * 0.155, 0.532)),
            material=painted_steel,
            name=f"hanger_tab_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.200, 0.090, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=carriage_blue,
        name="top_plate",
    )
    carriage.visual(
        Cylinder(radius=0.032, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=machined_metal,
        name="hub_shaft",
    )
    carriage.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=machined_metal,
        name="rotor_hub",
    )
    carriage.visual(
        Cylinder(radius=0.075, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=lower_mass,
        name="support_mass",
    )
    carriage.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=machined_metal,
        name="lower_collar",
    )
    for ix, x in enumerate((-0.075, 0.075)):
        for iy, y in enumerate((-0.027, 0.027)):
            carriage.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, 0.0705)),
                material=bolt_black,
                name=f"plate_bolt_{ix}_{iy}",
            )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, hub_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("frame_to_carriage")

    ctx.check(
        "single vertical revolute yaw joint",
        len(object_model.articulations) == 1
        and yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="hub_shaft",
        elem_b="bearing_ring",
        min_overlap=0.050,
        name="shaft passes through bearing height",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="top_plate",
        negative_elem="bearing_ring",
        min_gap=0.006,
        max_gap=0.020,
        name="carried top plate clears fixed bearing",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="bearing_ring",
        negative_elem="support_mass",
        min_gap=0.030,
        max_gap=0.060,
        name="support mass hangs below hub",
    )

    rest_aabb = ctx.part_element_world_aabb(carriage, elem="top_plate")
    with ctx.pose({yaw: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(carriage, elem="top_plate")

    def _span(aabb, axis_index: int) -> float:
        if aabb is None:
            return 0.0
        return float(aabb[1][axis_index] - aabb[0][axis_index])

    ctx.check(
        "top plate yaws about hub axis",
        rest_aabb is not None
        and turned_aabb is not None
        and _span(rest_aabb, 0) > _span(rest_aabb, 1) + 0.08
        and _span(turned_aabb, 1) > _span(turned_aabb, 0) + 0.08,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
