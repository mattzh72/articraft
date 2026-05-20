from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.18
FOOT_WIDTH = 0.12
FOOT_HEIGHT = 0.022
FOOT_CORNER_RADIUS = 0.012

PEDESTAL_HEIGHT = 0.085
PEDESTAL_BASE_X = 0.060
PEDESTAL_BASE_Y = 0.046
PEDESTAL_TOP_X = 0.050
PEDESTAL_TOP_Y = 0.038

HEAD_BODY_X = 0.074
HEAD_BODY_Y = 0.060
HEAD_BODY_HEIGHT = 0.022
HEAD_BODY_BORE_DEPTH = 0.018

BEARING_RING_RADIUS = 0.033
BEARING_RING_HEIGHT = 0.011
BEARING_RING_OVERLAP = 0.001
BORE_RADIUS = 0.0126

SHAFT_RADIUS = 0.0105
SHAFT_INSERT_DEPTH = 0.022
SHAFT_HUB_OVERLAP = 0.002

FLANGE_HUB_RADIUS = 0.015
FLANGE_HUB_HEIGHT = 0.007
FLANGE_RADIUS = 0.022
FLANGE_THICKNESS = 0.007
BOLT_CIRCLE_RADIUS = 0.013
BOLT_HOLE_DIAMETER = 0.0042

ROLL_LIMIT = pi
ROLL_AXIS_Z = (
    FOOT_HEIGHT
    + PEDESTAL_HEIGHT
    + HEAD_BODY_HEIGHT
    - BEARING_RING_OVERLAP
    + BEARING_RING_HEIGHT
)


def _bolt_circle_points(count: int, radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * i) / count),
            radius * sin((2.0 * pi * i) / count),
        )
        for i in range(count)
    ]


def _build_tower_body() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(FOOT_CORNER_RADIUS)
    )
    foot = (
        foot.faces(">Z")
        .workplane()
        .pushPoints([(-0.052, 0.0), (0.052, 0.0)])
        .slot2D(0.028, 0.010, 0.0)
        .cutThruAll()
    )

    pedestal = (
        cq.Workplane("XY")
        .ellipse(PEDESTAL_BASE_X / 2.0, PEDESTAL_BASE_Y / 2.0)
        .workplane(offset=PEDESTAL_HEIGHT)
        .ellipse(PEDESTAL_TOP_X / 2.0, PEDESTAL_TOP_Y / 2.0)
        .loft(combine=True)
        .translate((0.0, 0.0, FOOT_HEIGHT))
    )

    head = (
        cq.Workplane("XY")
        .rect(HEAD_BODY_X, HEAD_BODY_Y)
        .extrude(HEAD_BODY_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT + PEDESTAL_HEIGHT))
        .edges("|Z")
        .fillet(0.006)
    )

    body = foot.union(pedestal).union(head)

    head_bore = (
        cq.Workplane("XY")
        .circle(BORE_RADIUS)
        .extrude(HEAD_BODY_BORE_DEPTH)
        .translate(
            (
                0.0,
                0.0,
                FOOT_HEIGHT
                + PEDESTAL_HEIGHT
                + HEAD_BODY_HEIGHT
                - HEAD_BODY_BORE_DEPTH,
            )
        )
    )
    return body.cut(head_bore)


def _build_bearing_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(BEARING_RING_RADIUS)
        .circle(BORE_RADIUS)
        .extrude(BEARING_RING_HEIGHT)
        .translate(
            (
                0.0,
                0.0,
                FOOT_HEIGHT + PEDESTAL_HEIGHT + HEAD_BODY_HEIGHT - BEARING_RING_OVERLAP,
            )
        )
    )


def _build_flange_plate() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_HUB_RADIUS)
        .extrude(FLANGE_HUB_HEIGHT)
        .faces(">Z")
        .workplane()
        .circle(FLANGE_RADIUS)
        .extrude(FLANGE_THICKNESS)
    )
    return (
        flange.faces(">Z")
        .workplane()
        .pushPoints(_bolt_circle_points(4, BOLT_CIRCLE_RADIUS))
        .hole(BOLT_HOLE_DIAMETER, depth=FLANGE_THICKNESS)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_roll_stage")

    model.material("base_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("bearing_gray", rgba=(0.54, 0.57, 0.61, 1.0))
    model.material("machined_flange", rgba=(0.77, 0.79, 0.82, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_build_tower_body(), "tower_body"),
        material="base_coat",
        name="tower_body",
    )
    tower.visual(
        mesh_from_cadquery(_build_bearing_ring(), "bearing_ring"),
        material="bearing_gray",
        name="bearing_ring",
    )

    flange = model.part("flange")
    flange.visual(
        Cylinder(
            radius=SHAFT_RADIUS,
            length=SHAFT_INSERT_DEPTH + SHAFT_HUB_OVERLAP,
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -(SHAFT_INSERT_DEPTH / 2.0) + (SHAFT_HUB_OVERLAP / 2.0),
            )
        ),
        material="bearing_gray",
        name="shaft_journal",
    )
    flange.visual(
        mesh_from_cadquery(_build_flange_plate(), "flange_plate"),
        material="machined_flange",
        name="flange_plate",
    )

    model.articulation(
        "tower_to_flange",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flange,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=6.0,
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
        ),
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

    tower = object_model.get_part("tower")
    flange = object_model.get_part("flange")
    roll_joint = object_model.get_articulation("tower_to_flange")

    limits = roll_joint.motion_limits
    ctx.check(
        "roll joint is vertical and centered",
        tuple(round(v, 6) for v in roll_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in roll_joint.origin.xyz) == (0.0, 0.0, round(ROLL_AXIS_Z, 6)),
        details=f"axis={roll_joint.axis}, origin={roll_joint.origin.xyz}",
    )
    ctx.check(
        "roll joint has finite bidirectional travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and limits.upper - limits.lower >= 2.0 * pi - 0.05,
        details=f"limits={limits}",
    )

    with ctx.pose({roll_joint: 0.0}):
        ctx.expect_gap(
            flange,
            tower,
            axis="z",
            positive_elem="flange_plate",
            negative_elem="bearing_ring",
            max_gap=0.0005,
            max_penetration=0.0,
            name="flange seats on the bearing ring",
        )
        ctx.expect_within(
            flange,
            tower,
            axes="xy",
            inner_elem="flange_plate",
            outer_elem="bearing_ring",
            margin=0.0,
            name="spinning flange stays inside support footprint",
        )
        rest_pos = ctx.part_world_position(flange)

    with ctx.pose({roll_joint: 1.2}):
        ctx.expect_gap(
            flange,
            tower,
            axis="z",
            positive_elem="flange_plate",
            negative_elem="bearing_ring",
            max_gap=0.0005,
            max_penetration=0.0,
            name="flange stays seated while rolled",
        )
        turned_pos = ctx.part_world_position(flange)

    stationary = (
        rest_pos is not None
        and turned_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, turned_pos)) <= 1e-6
    )
    ctx.check(
        "spin is pure rotation about the shaft axis",
        stationary,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
