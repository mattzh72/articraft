from __future__ import annotations

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
import cadquery as cq


POST_X = 0.22
POST_RADIUS = 0.032
POST_HEIGHT = 0.92
BASE_HEIGHT = 0.10
PLATEN_Z0 = 0.34
PLATEN_TRAVEL = 0.38


def _platen_body_shape() -> cq.Workplane:
    """Compact carriage plate with two true vertical guide holes."""
    post_points = [(-POST_X, 0.0), (POST_X, 0.0)]
    body = cq.Workplane("XY").box(0.62, 0.24, 0.12)

    # Large guide bosses around both posts make the moving platen read as one
    # rigid wrap-around carriage rather than a plain floating block.
    for x, y in post_points:
        boss = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(0.078)
            .extrude(0.22)
            .translate((0.0, 0.0, -0.11))
        )
        body = body.union(boss)

    # Cut the post clearances after the bosses are fused so the holes continue
    # cleanly through the full carriage and leave no collision with the posts.
    for x, y in post_points:
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(0.052)
            .extrude(0.40)
            .translate((0.0, 0.0, -0.20))
        )
        body = body.cut(cutter)

    return body


def _guide_liner_shape(x: float) -> cq.Workplane:
    """Bronze bushing sleeve seated in one platen guide hole."""
    sleeve = (
        cq.Workplane("XY")
        .center(x, 0.0)
        .circle(0.054)
        .extrude(0.18)
        .translate((0.0, 0.0, -0.09))
    )
    bore = (
        cq.Workplane("XY")
        .center(x, 0.0)
        .circle(POST_RADIUS)
        .extrude(0.24)
        .translate((0.0, 0.0, -0.12))
    )
    return sleeve.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_post_platen_lift")

    cast_iron = model.material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.75, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.05, 0.22, 0.55, 1.0))
    bronze = model.material("bronze_bushings", rgba=(0.70, 0.46, 0.20, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    bolt = model.material("black_oxide_bolts", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.40, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=cast_iron,
        name="base_casting",
    )
    base.visual(
        Box((0.62, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.0175)),
        material=cast_iron,
        name="raised_plinth",
    )

    for x, post_name, socket_name, stop_name in (
        (-POST_X, "post_0", "post_socket_0", "top_stop_0"),
        (POST_X, "post_1", "post_socket_1", "top_stop_1"),
    ):
        base.visual(
            Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
            origin=Origin(xyz=(x, 0.0, BASE_HEIGHT + POST_HEIGHT / 2.0)),
            material=steel,
            name=post_name,
        )
        base.visual(
            Cylinder(radius=0.070, length=0.034),
            origin=Origin(xyz=(x, 0.0, BASE_HEIGHT + 0.017)),
            material=cast_iron,
            name=socket_name,
        )
        base.visual(
            Cylinder(radius=0.046, length=0.026),
            origin=Origin(xyz=(x, 0.0, BASE_HEIGHT + POST_HEIGHT + 0.013)),
            material=bolt,
            name=stop_name,
        )

    # Four anchor ears and bolt heads reinforce the heavy machine-base reading.
    for index, (x, y) in enumerate(
        ((-0.31, -0.15), (-0.31, 0.15), (0.31, -0.15), (0.31, 0.15))
    ):
        base.visual(
            Cylinder(radius=0.033, length=0.018),
            origin=Origin(xyz=(x, y, BASE_HEIGHT + 0.009)),
            material=bolt,
            name=f"anchor_bolt_{index}",
        )

    platen = model.part("platen")
    platen.visual(
        Box((0.20, 0.24, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=safety_blue,
        name="platen_body",
    )
    platen.visual(
        Box((0.62, 0.060, 0.12)),
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        material=safety_blue,
        name="front_rail",
    )
    platen.visual(
        Box((0.62, 0.060, 0.12)),
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
        material=safety_blue,
        name="rear_rail",
    )
    platen.visual(
        Box((0.080, 0.24, 0.12)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0)),
        material=safety_blue,
        name="outer_web_0",
    )
    platen.visual(
        Box((0.080, 0.24, 0.12)),
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
        material=safety_blue,
        name="outer_web_1",
    )
    for x, front_name, rear_name in (
        (-POST_X, "front_bearing_0", "rear_bearing_0"),
        (POST_X, "front_bearing_1", "rear_bearing_1"),
    ):
        platen.visual(
            Box((0.050, 0.028, 0.14)),
            origin=Origin(xyz=(x, POST_RADIUS + 0.014, 0.0)),
            material=bronze,
            name=front_name,
        )
        platen.visual(
            Box((0.050, 0.028, 0.14)),
            origin=Origin(xyz=(x, -(POST_RADIUS + 0.014), 0.0)),
            material=bronze,
            name=rear_name,
        )
    platen.visual(
        Box((0.028, 0.050, 0.14)),
        origin=Origin(xyz=(-POST_X - (POST_RADIUS + 0.014), 0.0, 0.0)),
        material=bronze,
        name="guide_liner_0",
    )
    platen.visual(
        Box((0.028, 0.050, 0.14)),
        origin=Origin(xyz=(POST_X + (POST_RADIUS + 0.014), 0.0, 0.0)),
        material=bronze,
        name="guide_liner_1",
    )
    platen.visual(
        Box((0.30, 0.18, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material=rubber,
        name="press_pad",
    )

    model.articulation(
        "base_to_platen",
        ArticulationType.PRISMATIC,
        parent=base,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, PLATEN_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLATEN_TRAVEL,
            effort=1500.0,
            velocity=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platen = object_model.get_part("platen")
    lift = object_model.get_articulation("base_to_platen")

    ctx.expect_gap(
        platen,
        base,
        axis="z",
        positive_elem="press_pad",
        negative_elem="raised_plinth",
        min_gap=0.09,
        name="platen starts above the lower base",
    )
    ctx.expect_contact(
        base,
        platen,
        elem_a="post_0",
        elem_b="front_bearing_0",
        contact_tol=0.001,
        name="first post bears against the platen guide",
    )
    ctx.expect_contact(
        base,
        platen,
        elem_a="post_1",
        elem_b="front_bearing_1",
        contact_tol=0.001,
        name="second post bears against the platen guide",
    )
    ctx.expect_overlap(
        platen,
        base,
        axes="xy",
        elem_b="post_0",
        min_overlap=0.05,
        name="platen frame surrounds the first post footprint",
    )
    ctx.expect_overlap(
        platen,
        base,
        axes="xy",
        elem_b="post_1",
        min_overlap=0.05,
        name="platen frame surrounds the second post footprint",
    )

    rest_pos = ctx.part_world_position(platen)
    with ctx.pose({lift: PLATEN_TRAVEL}):
        ctx.expect_gap(
            base,
            platen,
            axis="z",
            positive_elem="top_stop_0",
            negative_elem="platen_body",
            min_gap=0.11,
            name="raised platen remains below post stops",
        )
        ctx.expect_contact(
            base,
            platen,
            elem_a="post_0",
            elem_b="front_bearing_0",
            contact_tol=0.001,
            name="raised first guide still tracks its post",
        )
        extended_pos = ctx.part_world_position(platen)

    ctx.check(
        "single prismatic joint moves platen upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + PLATEN_TRAVEL - 0.005,
        details=f"rest={rest_pos}, raised={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
