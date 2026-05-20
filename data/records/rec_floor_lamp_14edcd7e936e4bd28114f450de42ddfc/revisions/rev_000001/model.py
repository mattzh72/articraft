from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_arc_reading_lamp")

    matte_black = model.material("matte_black", rgba=(0.01, 0.011, 0.012, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.07, 0.075, 0.08, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.60, 0.54, 1.0))
    warm_light = model.material("warm_light", rgba=(1.0, 0.82, 0.40, 1.0))

    base_thickness = 0.055
    post_top_z = 0.610
    boom_hinge_z = 0.640

    base = model.part("base")

    base_plate_shape = (
        cq.Workplane("XY")
        .box(0.300, 0.300, base_thickness)
        .edges("|Z")
        .fillet(0.020)
        .edges(">Z")
        .fillet(0.004)
    )
    base.visual(
        mesh_from_cadquery(base_plate_shape, "weighted_square_base"),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=matte_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.002)),
        material=brushed_steel,
        name="post_collar",
    )
    base.visual(
        Cylinder(radius=0.018, length=post_top_z - base_thickness + 0.010),
        origin=Origin(xyz=(0.0, 0.0, (post_top_z + base_thickness) / 2.0 - 0.005)),
        material=brushed_steel,
        name="upright_post",
    )
    base.visual(
        Box((0.060, 0.104, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, boom_hinge_z - 0.041)),
        material=brushed_steel,
        name="base_yoke_block",
    )
    for y, name in ((-0.046, "base_yoke_0"), (0.046, "base_yoke_1")):
        base.visual(
            Box((0.058, 0.008, 0.078)),
            origin=Origin(xyz=(0.0, y, boom_hinge_z - 0.002)),
            material=brushed_steel,
            name=name,
        )
        base.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(
                xyz=(0.0, y + (0.006 if y > 0 else -0.006), boom_hinge_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_graphite,
            name=f"pivot_cap_{0 if y < 0 else 1}",
        )

    boom = model.part("boom_arm")
    boom_end = (0.695, 0.0, -0.015)
    boom_tube = tube_from_spline_points(
        [
            (0.000, 0.0, 0.000),
            (0.170, 0.0, 0.070),
            (0.460, 0.0, 0.055),
            boom_end,
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=24,
        cap_ends=True,
    )
    boom.visual(
        mesh_from_geometry(boom_tube, "arched_boom_tube"),
        material=brushed_steel,
        name="arched_tube",
    )
    boom.visual(
        Cylinder(radius=0.024, length=0.084),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_hub",
    )
    boom.visual(
        Box((0.030, 0.110, 0.035)),
        origin=Origin(xyz=(0.695, 0.0, -0.015)),
        material=brushed_steel,
        name="end_yoke_web",
    )
    for y, name in ((-0.046, "end_yoke_0"), (0.046, "end_yoke_1")):
        boom.visual(
            Box((0.070, 0.008, 0.060)),
            origin=Origin(xyz=(0.725, y, -0.015)),
            material=brushed_steel,
            name=name,
        )
        boom.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(0.750, y + (0.006 if y > 0 else -0.006), -0.015),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_graphite,
            name=f"shade_cap_{0 if y < 0 else 1}",
        )

    shade = model.part("shade")
    top_z = -0.060
    bottom_z = -0.180
    shade_shell = LatheGeometry(
        [
            (0.046, top_z),
            (0.088, bottom_z),
            (0.081, bottom_z + 0.005),
            (0.018, top_z - 0.006),
        ],
        segments=64,
        closed=True,
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "open_tapered_shade"),
        material=dark_graphite,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.016, length=0.084),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="shade_hub",
    )
    shade.visual(
        Box((0.026, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=brushed_steel,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, top_z + 0.004)),
        material=brushed_steel,
        name="top_collar",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=brushed_steel,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=warm_light,
        name="glowing_bulb",
    )

    model.articulation(
        "base_to_boom",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, boom_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.35, upper=0.65),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(0.750, 0.0, -0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    boom = object_model.get_part("boom_arm")
    shade = object_model.get_part("shade")
    boom_joint = object_model.get_articulation("base_to_boom")
    shade_joint = object_model.get_articulation("boom_to_shade")

    ctx.expect_within(
        boom,
        base,
        axes="y",
        inner_elem="pivot_hub",
        outer_elem="base_yoke_block",
        margin=0.002,
        name="boom hub sits between base yoke cheeks",
    )
    ctx.expect_gap(
        boom,
        base,
        axis="z",
        positive_elem="pivot_hub",
        negative_elem="base_yoke_block",
        min_gap=0.002,
        max_gap=0.020,
        name="boom hub clears the yoke saddle",
    )
    ctx.expect_within(
        shade,
        boom,
        axes="y",
        inner_elem="shade_hub",
        outer_elem="end_yoke_web",
        margin=0.002,
        name="shade hub sits between end bracket cheeks",
    )
    ctx.expect_overlap(
        shade,
        boom,
        axes="xz",
        elem_a="shade_hub",
        elem_b="end_yoke_0",
        min_overlap=0.010,
        name="shade hinge axis aligns with bracket cheeks",
    )

    rest_shade_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({boom_joint: 0.55}):
        raised_shade_aabb = ctx.part_world_aabb(shade)
    ctx.check(
        "boom joint raises the reading head",
        rest_shade_aabb is not None
        and raised_shade_aabb is not None
        and (raised_shade_aabb[0][2] + raised_shade_aabb[1][2])
        > (rest_shade_aabb[0][2] + rest_shade_aabb[1][2])
        + 0.20,
        details=f"rest={rest_shade_aabb}, raised={raised_shade_aabb}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_joint: 0.60}):
        tilted_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade hinge tilts the shade",
        rest_shell_aabb is not None
        and tilted_shell_aabb is not None
        and (tilted_shell_aabb[0][0] + tilted_shell_aabb[1][0])
        < (rest_shell_aabb[0][0] + rest_shell_aabb[1][0])
        - 0.05,
        details=f"rest={rest_shell_aabb}, tilted={tilted_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
