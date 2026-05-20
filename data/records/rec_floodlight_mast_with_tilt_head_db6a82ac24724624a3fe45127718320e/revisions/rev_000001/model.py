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


def _base_ring_shape() -> cq.Workplane:
    """Connected hollow base ring, hub, spokes, and lower telescoping sleeve."""
    ring = cq.Workplane("XY").circle(0.32).circle(0.22).extrude(0.035)
    hub = cq.Workplane("XY").circle(0.085).extrude(0.055)

    spoke_x = cq.Workplane("XY").box(0.54, 0.055, 0.035).translate((0.0, 0.0, 0.0175))
    spoke_y = cq.Workplane("XY").box(0.055, 0.54, 0.035).translate((0.0, 0.0, 0.0175))

    sleeve = cq.Workplane("XY").workplane(offset=0.035).circle(0.045).extrude(0.515)
    top_collar = cq.Workplane("XY").workplane(offset=0.49).circle(0.065).extrude(0.060)

    bore = cq.Workplane("XY").circle(0.0305).extrude(0.68).translate((0.0, 0.0, -0.03))

    return ring.union(hub).union(spoke_x).union(spoke_y).union(sleeve).union(top_collar).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_led_floodlight_stand")

    black = model.material("powder_coated_black", rgba=(0.02, 0.022, 0.024, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    warm_lens = model.material("warm_led_lens", rgba=(1.0, 0.86, 0.34, 1.0))
    led_chip = model.material("led_chip_yellow", rgba=(1.0, 0.97, 0.68, 1.0))
    steel = model.material("brushed_steel", rgba=(0.65, 0.67, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_ring_shape(), "base_ring"),
        material=black,
        name="base_ring",
    )
    # Rubber pads sit under the ring/spoke ends so the portable stand reads as a floor object.
    for idx, (x, y, yaw) in enumerate(
        (
            (0.265, 0.0, 0.0),
            (-0.265, 0.0, 0.0),
            (0.0, 0.265, math.pi / 2.0),
            (0.0, -0.265, math.pi / 2.0),
        )
    ):
        base.visual(
            Box((0.095, 0.036, 0.014)),
            origin=Origin(xyz=(x, y, -0.002), rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )
    # A side thumb screw visually locks the telescoping tube at the collar.
    base.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.065, 0.505), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lock_screw",
    )
    base.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.0, 0.107, 0.505), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="lock_knob",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.024, length=1.200),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=steel,
        name="inner_post",
    )
    post.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3675)),
        material=graphite,
        name="height_collar",
    )
    post.visual(
        Cylinder(radius=0.036, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=black,
        name="top_clamp",
    )
    post.visual(
        Box((0.070, 0.062, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        material=black,
        name="bracket_stem",
    )
    post.visual(
        Box((0.150, 0.052, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.035)),
        material=black,
        name="yoke_bridge",
    )
    for x, plate_name, washer_name in (
        (-0.065, "yoke_plate_0", "hinge_washer_0"),
        (0.065, "yoke_plate_1", "hinge_washer_1"),
    ):
        post.visual(
            Box((0.018, 0.070, 0.150)),
            origin=Origin(xyz=(x, 0.0, 1.130)),
            material=black,
            name=plate_name,
        )
        post.visual(
            Cylinder(radius=0.026, length=0.008),
            origin=Origin(
                xyz=(x + (-0.013 if x < 0.0 else 0.013), 0.0, 1.130),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=graphite,
            name=washer_name,
        )

    lamp_bar = model.part("lamp_bar")
    lamp_bar.visual(
        Cylinder(radius=0.022, length=0.112),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    lamp_bar.visual(
        Box((0.080, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, -0.032, -0.012)),
        material=graphite,
        name="hinge_saddle",
    )
    lamp_bar.visual(
        Box((0.760, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.085, -0.047)),
        material=graphite,
        name="crossbar",
    )
    lamp_bar.visual(
        Box((0.090, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, -0.061, -0.040)),
        material=graphite,
        name="saddle_neck",
    )

    lamp_centers = (-0.210, 0.210)
    for lamp_idx, (x, housing_name, face_name) in enumerate(
        (
            (lamp_centers[0], "lamp_housing_0", "lamp_face_0"),
            (lamp_centers[1], "lamp_housing_1", "lamp_face_1"),
        )
    ):
        lamp_bar.visual(
            Box((0.285, 0.065, 0.185)),
            origin=Origin(xyz=(x, -0.075, -0.140)),
            material=black,
            name=housing_name,
        )
        lamp_bar.visual(
            Box((0.235, 0.008, 0.132)),
            origin=Origin(xyz=(x, -0.109, -0.140)),
            material=warm_lens,
            name=face_name,
        )
        # Raised heat-sink ribs on the back of each LED head.
        for rib_idx, dx in enumerate((-0.084, -0.042, 0.0, 0.042, 0.084)):
            lamp_bar.visual(
                Box((0.011, 0.025, 0.140)),
                origin=Origin(xyz=(x + dx, -0.034, -0.140)),
                material=graphite,
                name=f"cooling_rib_{lamp_idx}_{rib_idx}",
            )
        # Six visible LED emitters behind each warm lens.
        for row, z in enumerate((-0.172, -0.108)):
            for col, dx in enumerate((-0.066, 0.0, 0.066)):
                lamp_bar.visual(
                    Cylinder(radius=0.012, length=0.004),
                    origin=Origin(
                        xyz=(x + dx, -0.1135, z),
                        rpy=(-math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=led_chip,
                    name=f"led_{lamp_idx}_{row}_{col}",
                )

    model.articulation(
        "base_to_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.50),
    )
    model.articulation(
        "post_to_lamp_bar",
        ArticulationType.REVOLUTE,
        parent=post,
        child=lamp_bar,
        origin=Origin(xyz=(0.0, 0.0, 1.130)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.60, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    lamp_bar = object_model.get_part("lamp_bar")
    lift = object_model.get_articulation("base_to_post")
    tilt = object_model.get_articulation("post_to_lamp_bar")

    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="base_ring",
        min_overlap=0.45,
        name="collapsed post is deeply retained in the sleeve",
    )
    ctx.expect_within(
        post,
        base,
        axes="xy",
        elem_a="inner_post",
        elem_b="base_ring",
        margin=0.0,
        name="round center post remains inside base footprint",
    )

    rest_post_pos = ctx.part_world_position(post)
    with ctx.pose({lift: 0.50}):
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="base_ring",
            min_overlap=0.02,
            name="extended post keeps retained insertion",
        )
        extended_post_pos = ctx.part_world_position(post)

    ctx.check(
        "post extends upward on prismatic joint",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.45,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    rest_face_aabb = ctx.part_element_world_aabb(lamp_bar, elem="lamp_face_0")
    with ctx.pose({tilt: 0.55}):
        tilted_face_aabb = ctx.part_element_world_aabb(lamp_bar, elem="lamp_face_0")

    rest_face_z = None if rest_face_aabb is None else (rest_face_aabb[0][2] + rest_face_aabb[1][2]) / 2.0
    tilted_face_z = (
        None if tilted_face_aabb is None else (tilted_face_aabb[0][2] + tilted_face_aabb[1][2]) / 2.0
    )
    ctx.check(
        "lamp bar tilts upward on bracket hinge",
        rest_face_z is not None and tilted_face_z is not None and tilted_face_z > rest_face_z + 0.04,
        details=f"rest_face_z={rest_face_z}, tilted_face_z={tilted_face_z}",
    )
    ctx.expect_overlap(
        lamp_bar,
        post,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="yoke_plate_0",
        min_overlap=0.015,
        name="hinge barrel is aligned with yoke bearing",
    )

    return ctx.report()


object_model = build_object_model()
