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


POST_TRAVEL = 0.82
SKIRT_UP_ANGLE = math.pi / 2.0


def _annular_disc(
    *,
    inner_radius: float,
    outer_radius: float,
    thickness: float,
    center_x: float = 0.0,
    center_z: float = 0.0,
):
    """A flat annular solid in the local XY plane."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((center_x, 0.0, center_z - thickness / 2.0))
    )


def _housing_sleeve_geometry():
    """Underground cylindrical sleeve plus flush annular rim flange."""
    sleeve = (
        cq.Workplane("XY")
        .circle(0.220)
        .circle(0.132)
        .extrude(1.10)
        .translate((0.0, 0.0, -1.10))
    )
    rim = (
        cq.Workplane("XY")
        .circle(0.360)
        .circle(0.132)
        .extrude(0.055)
        .translate((0.0, 0.0, -0.055))
    )
    return sleeve.union(rim)


def _ground_pad_geometry():
    pad = cq.Workplane("XY").box(1.10, 1.10, 0.080).translate((0.0, 0.0, -0.040))
    cutter = cq.Workplane("XY").circle(0.358).extrude(0.120).translate((0.0, 0.0, -0.090))
    return pad.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rising_bollard_barrier")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    asphalt = model.material("dark_asphalt", rgba=(0.10, 0.095, 0.085, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.04, 1.0))
    reflector_red = model.material("red_reflector", rgba=(0.95, 0.05, 0.025, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_ground_pad_geometry(), "ground_pad", tolerance=0.0015),
        material=asphalt,
        name="ground_pad",
    )
    housing.visual(
        mesh_from_cadquery(_housing_sleeve_geometry(), "rim_and_guide_sleeve", tolerance=0.001),
        material=brushed_steel,
        name="rim_flange",
    )
    housing.visual(
        mesh_from_cadquery(
            _annular_disc(inner_radius=0.132, outer_radius=0.164, thickness=0.012, center_z=0.006),
            "wiper_seal",
            tolerance=0.001,
        ),
        material=rubber,
        name="wiper_seal",
    )
    for index in range(8):
        if index == 4:
            # The rim-side hinge occupies this bolt station on the real flange.
            continue
        angle = index * math.tau / 8.0
        housing.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(0.290 * math.cos(angle), 0.290 * math.sin(angle), 0.005)),
            material=dark_steel,
            name=f"rim_bolt_{index}",
        )

    # Two fixed hinge leaves and alternating knuckles sit on the rim-side bracket.
    for y in (-0.125, 0.125):
        housing.visual(
            Box((0.070, 0.075, 0.020)),
            origin=Origin(xyz=(-0.350, y, 0.009)),
            material=dark_steel,
            name=f"fixed_leaf_{'neg' if y < 0 else 'pos'}",
        )
        housing.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(-0.310, y, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"fixed_knuckle_{'neg' if y < 0 else 'pos'}",
        )
    for axis_name, sx, sy, x, y in (
        ("x_pos", 0.027, 0.050, 0.1185, 0.0),
        ("x_neg", 0.027, 0.050, -0.1185, 0.0),
        ("y_pos", 0.050, 0.027, 0.0, 0.1185),
        ("y_neg", 0.050, 0.027, 0.0, -0.1185),
    ):
        housing.visual(
            Box((sx, sy, 0.100)),
            origin=Origin(xyz=(x, y, -0.180)),
            material=rubber,
            name=f"guide_pad_{axis_name}",
        )

    post = model.part("steel_post")
    post.visual(
        Cylinder(radius=0.105, length=1.080),
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        material=brushed_steel,
        name="main_post",
    )
    post.visual(
        Cylinder(radius=0.118, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=brushed_steel,
        name="top_cap",
    )
    post.visual(
        Cylinder(radius=0.122, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -1.035)),
        material=dark_steel,
        name="guide_shoe",
    )
    for z, mat, band_name in (
        (-0.145, safety_yellow, "upper_band"),
        (-0.310, reflector_red, "lower_band"),
    ):
        post.visual(
            Cylinder(radius=0.108, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=mat,
            name=band_name,
        )

    skirt = model.part("skirt_ring")
    skirt.visual(
        mesh_from_cadquery(
            _annular_disc(
                inner_radius=0.175,
                outer_radius=0.310,
                thickness=0.012,
                center_x=0.310,
                center_z=-0.023,
            ),
            "skirt_ring_plate",
            tolerance=0.001,
        ),
        material=safety_yellow,
        name="ring_plate",
    )
    skirt.visual(
        Box((0.070, 0.105, 0.024)),
        origin=Origin(xyz=(0.045, 0.0, -0.021)),
        material=dark_steel,
        name="moving_leaf",
    )
    skirt.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="moving_knuckle",
    )

    model.articulation(
        "housing_to_post",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35, lower=0.0, upper=POST_TRAVEL),
    )

    model.articulation(
        "housing_to_skirt",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=skirt,
        origin=Origin(xyz=(-0.310, 0.0, 0.035)),
        # Positive revolute motion raises the ring from its flat pose.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=SKIRT_UP_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    post = object_model.get_part("steel_post")
    skirt = object_model.get_part("skirt_ring")
    post_slide = object_model.get_articulation("housing_to_post")
    skirt_hinge = object_model.get_articulation("housing_to_skirt")

    with ctx.pose({post_slide: 0.0, skirt_hinge: SKIRT_UP_ANGLE}):
        lowered_origin = ctx.part_world_position(post)
        upright_ring = ctx.part_element_world_aabb(skirt, elem="ring_plate")
        ctx.expect_overlap(
            post,
            housing,
            axes="z",
            elem_a="main_post",
            elem_b="rim_flange",
            min_overlap=0.80,
            name="lowered post is housed in the underground sleeve",
        )

    with ctx.pose({post_slide: POST_TRAVEL, skirt_hinge: 0.0}):
        raised_origin = ctx.part_world_position(post)
        flat_ring = ctx.part_element_world_aabb(skirt, elem="ring_plate")
        ctx.expect_overlap(
            post,
            housing,
            axes="z",
            elem_a="main_post",
            elem_b="rim_flange",
            min_overlap=0.20,
            name="raised post keeps retained insertion in the guide sleeve",
        )
        ctx.expect_gap(
            skirt,
            housing,
            axis="z",
            positive_elem="ring_plate",
            negative_elem="rim_flange",
            min_gap=0.003,
            max_gap=0.020,
            name="folded skirt ring sits just above the flush rim",
        )

    ctx.check(
        "post rises upward on the prismatic joint",
        lowered_origin is not None
        and raised_origin is not None
        and raised_origin[2] > lowered_origin[2] + 0.75,
        details=f"lowered={lowered_origin}, raised={raised_origin}",
    )

    if upright_ring is not None and flat_ring is not None:
        upright_z = upright_ring[1][2] - upright_ring[0][2]
        flat_z = flat_ring[1][2] - flat_ring[0][2]
        flat_top = flat_ring[1][2]
        ctx.check(
            "skirt hinge folds the ring flat as the bollard rises",
            upright_z > 0.45 and flat_z < 0.030 and flat_top < 0.030,
            details=f"upright_z={upright_z:.3f}, flat_z={flat_z:.3f}, flat_top={flat_top:.3f}",
        )
    else:
        ctx.fail("skirt hinge folds the ring flat as the bollard rises", "missing ring AABB")

    return ctx.report()


object_model = build_object_model()
