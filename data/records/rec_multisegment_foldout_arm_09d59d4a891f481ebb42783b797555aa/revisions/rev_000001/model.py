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
import cadquery as cq

def make_link(hole_dist: float, width: float, thickness: float, hole_radius: float, second_hole: bool = True):
    length = hole_dist + width
    wp = (
        cq.Workplane("XY")
        .center(hole_dist / 2, 0)
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(width / 2 - 0.001)
        .faces(">Z")
        .workplane()
        .center(-hole_dist / 2, 0)
        .hole(hole_radius * 2)
    )
    if second_hole:
        wp = wp.center(hole_dist, 0).hole(hole_radius * 2)
    return wp

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_arm")

    # Dimensions
    hole_dist = 0.16
    width = 0.03
    thickness = 0.02
    z_step = 0.02
    hole_radius = 0.008
    pin_radius = 0.0075

    geom_1hole = make_link(hole_dist, width, thickness, hole_radius, second_hole=False)

    # Base
    base = model.part("base")
    base.visual(
        Box((0.15, 0.15, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=pin_radius, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        name="base_pin",
    )

    # Link 1
    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(geom_1hole, "link_geom_1hole"),
        origin=Origin(xyz=(0.0, 0.0, thickness / 2)),
        name="link_1_body",
    )
    link_1.visual(
        Cylinder(radius=pin_radius, length=0.038),
        origin=Origin(xyz=(hole_dist, 0.0, 0.019)),
        name="link_1_pin",
    )

    # Link 2
    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(geom_1hole, "link_geom_1hole"),
        origin=Origin(xyz=(0.0, 0.0, thickness / 2)),
        name="link_2_body",
    )
    link_2.visual(
        Cylinder(radius=pin_radius, length=0.038),
        origin=Origin(xyz=(hole_dist, 0.0, 0.019)),
        name="link_2_pin",
    )

    # Link 3
    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(geom_1hole, "link_geom_1hole"),
        origin=Origin(xyz=(0.0, 0.0, thickness / 2)),
        name="link_3_body",
    )
    link_3.visual(
        Cylinder(radius=pin_radius, length=0.038),
        origin=Origin(xyz=(hole_dist, 0.0, 0.019)),
        name="link_3_pin",
    )

    # Link 4
    link_4 = model.part("link_4")
    link_4.visual(
        mesh_from_cadquery(geom_1hole, "link_geom_1hole"),
        origin=Origin(xyz=(0.0, 0.0, thickness / 2)),
        name="link_4_body",
    )

    # Platform
    platform = model.part("platform")
    platform.visual(
        Box((0.06, 0.06, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="platform_bracket",
    )

    # Articulations
    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, z_step)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=10.0, velocity=1.0),
    )

    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(hole_dist, 0.0, z_step)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.14, effort=10.0, velocity=1.0),
    )

    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(hole_dist, 0.0, z_step)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=0.0, effort=10.0, velocity=1.0),
    )

    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(hole_dist, 0.0, z_step)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.14, effort=10.0, velocity=1.0),
    )

    model.articulation(
        "link_4_to_platform",
        ArticulationType.FIXED,
        parent=link_4,
        child=platform,
        origin=Origin(xyz=(hole_dist, 0.0, z_step)),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    platform = object_model.get_part("platform")

    # Check Z clearances
    ctx.expect_gap(link_1, base, axis="z", positive_elem="link_1_body", negative_elem="base_plate", min_gap=-0.0001, max_gap=0.002)
    ctx.expect_gap(link_2, link_1, axis="z", positive_elem="link_2_body", negative_elem="link_1_body", min_gap=-0.0001, max_gap=0.002)
    ctx.expect_gap(link_3, link_2, axis="z", positive_elem="link_3_body", negative_elem="link_2_body", min_gap=-0.0001, max_gap=0.002)
    ctx.expect_gap(link_4, link_3, axis="z", positive_elem="link_4_body", negative_elem="link_3_body", min_gap=-0.0001, max_gap=0.002)
    ctx.expect_gap(platform, link_4, axis="z", positive_elem="platform_bracket", negative_elem="link_4_body", min_gap=-0.0001, max_gap=0.002)

    # Check extended reach
    ctx.expect_origin_distance(base, platform, min_dist=0.6, name="extended reach")

    # Check folded state
    with ctx.pose(
        base_to_link_1=1.57,
        link_1_to_link_2=3.14,
        link_2_to_link_3=-3.14,
        link_3_to_link_4=3.14,
    ):
        ctx.expect_origin_distance(base, platform, max_dist=0.15, name="folded compactly")

    return ctx.report()

object_model = build_object_model()
