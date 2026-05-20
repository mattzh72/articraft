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


BASE_RADIUS = 0.145
BASE_PLATE_THICKNESS = 0.0055
BASE_HOUSING_RADIUS = 0.108
BASE_HOUSING_DEPTH = 0.024
BASE_SKIRT_DEPTH = 0.016

DIFFUSER_OUTER_RADIUS = 0.134
DIFFUSER_INNER_RADIUS = 0.126
DIFFUSER_DEPTH = 0.080
DIFFUSER_WALL = 0.0028

HINGE_X = 0.132
HINGE_Z = -0.018
HINGE_RADIUS = 0.0038
BASE_KNUCKLE_LENGTH = 0.026
DIFFUSER_KNUCKLE_LENGTH = 0.042


def _build_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(BASE_RADIUS).extrude(-BASE_PLATE_THICKNESS)

    housing = (
        cq.Workplane("XY")
        .workplane(offset=-BASE_PLATE_THICKNESS)
        .circle(BASE_HOUSING_RADIUS)
        .extrude(-(BASE_HOUSING_DEPTH - BASE_PLATE_THICKNESS))
    )

    skirt_outer = (
        cq.Workplane("XY")
        .workplane(offset=-BASE_PLATE_THICKNESS)
        .circle(BASE_RADIUS)
        .extrude(-(BASE_SKIRT_DEPTH - BASE_PLATE_THICKNESS))
    )
    skirt_inner = (
        cq.Workplane("XY")
        .workplane(offset=-BASE_PLATE_THICKNESS)
        .circle(DIFFUSER_OUTER_RADIUS + 0.004)
        .extrude(-(BASE_SKIRT_DEPTH - BASE_PLATE_THICKNESS))
    )
    skirt = skirt_outer.cut(skirt_inner)

    service_cover = (
        cq.Workplane("XY")
        .workplane(offset=-BASE_PLATE_THICKNESS)
        .circle(0.050)
        .extrude(-0.006)
    )

    return plate.union(housing).union(skirt).union(service_cover)


def _build_diffuser_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(DIFFUSER_OUTER_RADIUS)
        .workplane(offset=-0.010)
        .circle(DIFFUSER_OUTER_RADIUS * 0.985)
        .workplane(offset=-0.014)
        .circle(DIFFUSER_OUTER_RADIUS * 0.93)
        .workplane(offset=-0.020)
        .circle(DIFFUSER_OUTER_RADIUS * 0.76)
        .workplane(offset=-0.022)
        .circle(DIFFUSER_OUTER_RADIUS * 0.44)
        .workplane(offset=-0.014)
        .circle(0.013)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY")
        .circle(DIFFUSER_INNER_RADIUS)
        .workplane(offset=-0.010)
        .circle(DIFFUSER_OUTER_RADIUS * 0.955)
        .workplane(offset=-0.014)
        .circle(DIFFUSER_OUTER_RADIUS * 0.90)
        .workplane(offset=-0.020)
        .circle(DIFFUSER_OUTER_RADIUS * 0.72)
        .workplane(offset=-0.020)
        .circle(DIFFUSER_OUTER_RADIUS * 0.40)
        .workplane(offset=-0.011)
        .circle(0.0105)
        .loft(combine=True)
    )

    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_dome_ceiling_light")

    model.material("painted_steel", rgba=(0.95, 0.95, 0.93, 1.0))
    model.material("opal_diffuser", rgba=(0.95, 0.96, 0.97, 0.62))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_shell"),
        material="painted_steel",
        name="base_shell",
    )
    for index, y in enumerate((-0.034, 0.034)):
        base.visual(
            Box((0.016, 0.026, 0.010)),
            origin=Origin(xyz=(HINGE_X - 0.008, y, -0.011)),
            material="painted_steel",
            name=f"hinge_ear_{index}",
        )
        base.visual(
            Cylinder(radius=HINGE_RADIUS, length=BASE_KNUCKLE_LENGTH),
            origin=Origin(
                xyz=(HINGE_X, y, HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material="painted_steel",
            name=f"hinge_knuckle_{index}",
        )

    diffuser = model.part("diffuser")
    diffuser.visual(
        mesh_from_cadquery(_build_diffuser_shape(), "lens_shell"),
        origin=Origin(xyz=(-HINGE_X, 0.0, 0.0)),
        material="opal_diffuser",
        name="lens_shell",
    )
    diffuser.visual(
        Box((0.012, DIFFUSER_KNUCKLE_LENGTH, 0.006)),
        origin=Origin(xyz=(-0.006, 0.0, -0.003)),
        material="opal_diffuser",
        name="hinge_bridge",
    )
    diffuser.visual(
        Cylinder(radius=HINGE_RADIUS - 0.0003, length=DIFFUSER_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="opal_diffuser",
        name="hinge_knuckle",
    )
    diffuser.visual(
        Box((0.018, 0.036, 0.004)),
        origin=Origin(xyz=(-0.254, 0.0, HINGE_Z + 0.0015)),
        material="opal_diffuser",
        name="rear_tab",
    )

    model.articulation(
        "base_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=base,
        child=diffuser,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.35,
            effort=3.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    diffuser = object_model.get_part("diffuser")
    hinge = object_model.get_articulation("base_to_diffuser")

    ctx.expect_origin_gap(
        diffuser,
        base,
        axis="x",
        min_gap=0.126,
        max_gap=0.138,
        name="diffuser hinge sits on the front rim",
    )
    ctx.expect_origin_gap(
        base,
        diffuser,
        axis="z",
        min_gap=0.016,
        max_gap=0.020,
        name="diffuser hinge hangs below the ceiling plate",
    )
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            diffuser,
            base,
            axes="xy",
            min_overlap=0.24,
            name="closed diffuser stays centered under the base",
        )

    closed_aabb = ctx.part_world_aabb(diffuser)
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_world_aabb(diffuser)

    opens_downward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.030
        and open_aabb[1][0] > closed_aabb[1][0] + 0.045
    )
    ctx.check(
        "diffuser rotates downward from the front hinge",
        opens_downward,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
