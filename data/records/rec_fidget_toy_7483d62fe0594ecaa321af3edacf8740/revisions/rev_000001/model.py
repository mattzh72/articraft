from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RIB_COUNT = 8
RIB_LENGTH = 0.108
RIB_THICKNESS = 0.0020
RIB_PITCH = 0.0036
RIB_BASE_WIDTH = 0.010
RIB_TIP_WIDTH = 0.0055
PIVOT_OUTER_RADIUS = 0.012
PIVOT_HOLE_RADIUS = 0.0044
PIN_RADIUS = PIVOT_HOLE_RADIUS
CAP_RADIUS = 0.011
CAP_THICKNESS = 0.0026

OPEN_ANGLES = tuple(math.radians(a) for a in (-56, -40, -24, -8, 8, 24, 40, 56))


def _rib_geometry() -> cq.Workplane:
    """Flat fidget-fan rib with a round pivot eye and clear rivet hole."""

    strip = (
        cq.Workplane("XY")
        .polyline(
            [
                (0.0, -RIB_BASE_WIDTH / 2.0),
                (RIB_LENGTH, -RIB_TIP_WIDTH / 2.0),
                (RIB_LENGTH, RIB_TIP_WIDTH / 2.0),
                (0.0, RIB_BASE_WIDTH / 2.0),
            ]
        )
        .close()
        .extrude(RIB_THICKNESS)
    )
    hub = cq.Workplane("XY").circle(PIVOT_OUTER_RADIUS).extrude(RIB_THICKNESS)
    tip = (
        cq.Workplane("XY")
        .center(RIB_LENGTH, 0.0)
        .circle(RIB_TIP_WIDTH / 2.0)
        .extrude(RIB_THICKNESS)
    )
    hole = cq.Workplane("XY").circle(PIVOT_HOLE_RADIUS).extrude(RIB_THICKNESS * 3.0)

    rib = strip.union(hub).union(tip).cut(hole)
    rib = rib.edges("|Z").fillet(0.0008)
    return rib.translate((0.0, 0.0, -RIB_THICKNESS / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_fan_fidget")

    brushed_steel = Material("brushed_steel", rgba=(0.74, 0.70, 0.64, 1.0))
    rib_materials = [
        Material("rib_red", rgba=(0.90, 0.18, 0.16, 1.0)),
        Material("rib_orange", rgba=(0.95, 0.48, 0.13, 1.0)),
        Material("rib_yellow", rgba=(0.96, 0.74, 0.16, 1.0)),
        Material("rib_green", rgba=(0.25, 0.70, 0.25, 1.0)),
        Material("rib_teal", rgba=(0.12, 0.65, 0.70, 1.0)),
        Material("rib_blue", rgba=(0.14, 0.34, 0.90, 1.0)),
        Material("rib_indigo", rgba=(0.34, 0.25, 0.78, 1.0)),
        Material("rib_violet", rgba=(0.62, 0.25, 0.74, 1.0)),
    ]

    stack_half_height = ((RIB_COUNT - 1) * RIB_PITCH + RIB_THICKNESS) / 2.0

    rivet = model.part("rivet")
    rivet.visual(
        Cylinder(PIN_RADIUS, 2.0 * (stack_half_height + CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pin",
    )
    rivet.visual(
        Cylinder(CAP_RADIUS, CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, stack_half_height + CAP_THICKNESS / 2.0)),
        material=brushed_steel,
        name="top_cap",
    )
    rivet.visual(
        Cylinder(CAP_RADIUS, CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -stack_half_height - CAP_THICKNESS / 2.0)),
        material=brushed_steel,
        name="bottom_cap",
    )

    rib_mesh = mesh_from_cadquery(_rib_geometry(), "flat_rib")
    for i, open_angle in enumerate(OPEN_ANGLES):
        z = (i - (RIB_COUNT - 1) / 2.0) * RIB_PITCH
        rib = model.part(f"rib_{i}")
        rib.visual(
            rib_mesh,
            origin=Origin(),
            material=rib_materials[i],
            name="rib_body",
        )

        lower = min(0.0, open_angle)
        upper = max(0.0, open_angle)
        model.articulation(
            f"pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=rivet,
            child=rib,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=8.0, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rivet = object_model.get_part("rivet")
    ribs = [object_model.get_part(f"rib_{i}") for i in range(RIB_COUNT)]
    pivots = [object_model.get_articulation(f"pivot_{i}") for i in range(RIB_COUNT)]

    ctx.check(
        "eight articulated ribs",
        len(ribs) == RIB_COUNT and len(pivots) == RIB_COUNT,
        details=f"ribs={len(ribs)} pivots={len(pivots)}",
    )

    for i, pivot in enumerate(pivots):
        limits = pivot.motion_limits
        target = OPEN_ANGLES[i]
        ctx.check(
            f"pivot_{i} has realistic revolute travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= min(0.0, target) + 1e-6
            and limits.upper >= max(0.0, target) - 1e-6,
            details=f"limits={limits}, target={target}",
        )
        ctx.expect_origin_distance(
            ribs[i],
            rivet,
            axes="xy",
            max_dist=0.0005,
            name=f"rib_{i} pivots on shared rivet axis",
        )
        if i > 0:
            ctx.expect_gap(
                ribs[i],
                ribs[i - 1],
                axis="z",
                min_gap=0.0008,
                max_gap=0.0022,
                name=f"rib_{i} is a separate stacked leaf",
            )

    closed_aabbs = [ctx.part_world_aabb(rib) for rib in ribs]
    closed_min_y = min(aabb[0][1] for aabb in closed_aabbs if aabb is not None)
    closed_max_y = max(aabb[1][1] for aabb in closed_aabbs if aabb is not None)
    ctx.check(
        "closed ribs nest into a compact handle-width stack",
        closed_max_y - closed_min_y < 0.032,
        details=f"closed_y_span={closed_max_y - closed_min_y:.4f}",
    )

    with ctx.pose({pivots[i]: OPEN_ANGLES[i] for i in range(RIB_COUNT)}):
        open_aabbs = [ctx.part_world_aabb(rib) for rib in ribs]
        open_min_y = min(aabb[0][1] for aabb in open_aabbs if aabb is not None)
        open_max_y = max(aabb[1][1] for aabb in open_aabbs if aabb is not None)
        open_max_x = max(aabb[1][0] for aabb in open_aabbs if aabb is not None)
        ctx.check(
            "open pose spreads ribs into a fan",
            open_max_y - open_min_y > 0.14 and open_max_x > 0.060,
            details=f"open_y_span={open_max_y - open_min_y:.4f}, open_max_x={open_max_x:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
