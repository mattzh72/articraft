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


SHAFT_LENGTH = 0.74
SHAFT_RADIUS = 0.035
BEARING_X = (-0.22, 0.22)
BEARING_WIDTH = 0.076


def _ring_along_x(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Return a hollow cylinder with its bore on the world X axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        # CadQuery's ``both=True`` extrudes the requested distance to each side.
        .extrude(width / 2.0, both=True)
        .rotate((0, 0, 0), (0, 1, 0), 90)
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _grounded_support_body() -> cq.Workplane:
    """Painted top bracket, hanger straps, and open bearing housings."""
    body = _box((0.64, 0.18, 0.040), (0.0, 0.0, 0.285))

    for x in BEARING_X:
        # Two short straps visually suspend each bearing from the overhead rail.
        for y in (-0.064, 0.064):
            body = body.union(_box((0.070, 0.026, 0.190), (x, y, 0.180)))

        # A round bearing carrier with a real through bore, not a solid proxy.
        housing = _ring_along_x(0.102, 0.064, BEARING_WIDTH).translate((x, 0.0, 0.0))
        body = body.union(housing)

        # Small bridge cap makes the hanger-to-housing load path unmistakable.
        body = body.union(_box((0.084, 0.124, 0.030), (x, 0.0, 0.105)))

    return body


def _bearing_liner() -> cq.Workplane:
    # The bore is sized as a snug bearing fit so the rotating shaft is visibly
    # supported by the hanging bearings instead of hovering with clearance.
    return _ring_along_x(0.066, SHAFT_RADIUS, BEARING_WIDTH + 0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_rotary_shaft_unit")

    painted_steel = Material("painted_steel", color=(0.10, 0.20, 0.32, 1.0))
    dark_bearing = Material("dark_bearing_race", color=(0.015, 0.017, 0.018, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.72, 0.70, 0.66, 1.0))
    key_mark = Material("blackened_key_mark", color=(0.03, 0.03, 0.028, 1.0))
    bolt_finish = Material("bolt_finish", color=(0.08, 0.08, 0.075, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_grounded_support_body(), "support_body"),
        material=painted_steel,
        name="support_body",
    )
    for i, x in enumerate(BEARING_X):
        support.visual(
            mesh_from_cadquery(_bearing_liner().translate((x, 0.0, 0.0)), f"bearing_liner_{i}"),
            material=dark_bearing,
            name=f"bearing_liner_{i}",
        )

    for i, x in enumerate((-0.27, -0.09, 0.09, 0.27)):
        for j, y in enumerate((-0.055, 0.055)):
            support.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(x, y, 0.310)),
                material=bolt_finish,
                name=f"top_bolt_{i}_{j}",
            )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft_core",
    )
    for i, x in enumerate((-0.165, 0.165, -0.292, 0.292)):
        shaft.visual(
            Cylinder(radius=0.046, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"shaft_collar_{i}",
        )
    shaft.visual(
        Box((0.250, 0.011, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_RADIUS + 0.005)),
        material=key_mark,
        name="index_mark",
    )

    model.articulation(
        "support_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=shaft,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    support = object_model.get_part("support")
    shaft = object_model.get_part("shaft")
    joint = object_model.get_articulation("support_to_shaft")

    ctx.check(
        "shaft uses one continuous revolute joint",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={joint.articulation_type}",
    )
    ctx.expect_origin_distance(
        support,
        shaft,
        axes="xyz",
        max_dist=0.0005,
        name="shaft frame stays on supported centerline",
    )
    for i in range(2):
        ctx.allow_overlap(
            shaft,
            support,
            elem_a="shaft_core",
            elem_b=f"bearing_liner_{i}",
            reason=(
                "The shaft core is intentionally a snug captured fit inside the "
                "bearing liner so the rotary joint has a physical support path."
            ),
        )
        ctx.expect_contact(
            shaft,
            support,
            elem_a="shaft_core",
            elem_b=f"bearing_liner_{i}",
            name=f"shaft bears on liner {i}",
        )
        ctx.expect_overlap(
            shaft,
            support,
            axes="x",
            elem_a="shaft_core",
            elem_b=f"bearing_liner_{i}",
            min_overlap=0.060,
            name=f"shaft spans bearing {i}",
        )
        ctx.expect_within(
            shaft,
            support,
            axes="yz",
            inner_elem="shaft_core",
            outer_elem=f"bearing_liner_{i}",
            margin=0.0,
            name=f"shaft is centered inside bearing {i}",
        )

    rest_aabb = ctx.part_element_world_aabb(shaft, elem="index_mark")
    with ctx.pose({joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(shaft, elem="index_mark")

    def _aabb_center(aabb, axis: int) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[axis] + hi[axis]) / 2.0

    rest_z = _aabb_center(rest_aabb, 2)
    turned_y = _aabb_center(turned_aabb, 1)
    turned_z = _aabb_center(turned_aabb, 2)
    ctx.check(
        "index mark follows shaft rotation",
        rest_z is not None
        and turned_y is not None
        and turned_z is not None
        and rest_z > 0.035
        and turned_y < -0.030
        and abs(turned_z) < 0.015,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
