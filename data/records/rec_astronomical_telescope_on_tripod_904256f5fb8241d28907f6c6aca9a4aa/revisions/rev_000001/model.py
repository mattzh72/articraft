from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_spotting_scope_alt_az")

    satin_black = Material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber_black = Material("ribbed_rubber_black", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_graphite = Material("dark_graphite", rgba=(0.12, 0.12, 0.13, 1.0))
    warm_metal = Material("anodized_graphite", rgba=(0.23, 0.23, 0.25, 1.0))
    glass_green = Material("coated_glass_green", rgba=(0.12, 0.45, 0.38, 0.45))
    foot_rubber = Material("matte_rubber_feet", rgba=(0.025, 0.024, 0.022, 1.0))

    cylinder_x = (0.0, math.pi / 2.0, 0.0)
    cylinder_y = (-math.pi / 2.0, 0.0, 0.0)

    # Root: a compact field tripod with a central column and three splayed legs.
    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.024, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=warm_metal,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        material=dark_graphite,
        name="top_casting",
    )
    tripod.visual(
        Cylinder(radius=0.036, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=dark_graphite,
        name="lower_spider",
    )
    leg_top_z = 0.545
    foot_z = 0.060
    foot_radius = 0.355
    leg_length = math.sqrt(foot_radius * foot_radius + (foot_z - leg_top_z) ** 2)
    leg_pitch = math.acos((foot_z - leg_top_z) / leg_length)
    for index, yaw in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        foot_x = foot_radius * math.cos(yaw)
        foot_y = foot_radius * math.sin(yaw)
        center_x = 0.5 * foot_x
        center_y = 0.5 * foot_y
        center_z = 0.5 * (leg_top_z + foot_z)
        tripod.visual(
            Cylinder(radius=0.014, length=leg_length),
            origin=Origin(xyz=(center_x, center_y, center_z), rpy=(0.0, leg_pitch, yaw)),
            material=warm_metal,
            name=f"leg_{index}",
        )
        tripod.visual(
            Sphere(radius=0.026),
            origin=Origin(xyz=(foot_x, foot_y, foot_z)),
            material=foot_rubber,
            name=f"foot_{index}",
        )

    # Azimuth head: a rotating bearing base carrying a compact fork/yoke.
    head = model.part("head")
    head.visual(
        Cylinder(radius=0.074, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_graphite,
        name="azimuth_bearing",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=warm_metal,
        name="pedestal",
    )
    head.visual(
        Box((0.105, 0.205, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=dark_graphite,
        name="fork_bridge",
    )
    for side, y in enumerate((-0.089, 0.089)):
        head.visual(
            Box((0.052, 0.026, 0.205)),
            origin=Origin(xyz=(0.0, y, 0.168)),
            material=dark_graphite,
            name=f"fork_arm_{side}",
        )
        head.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(0.0, 0.074 if y > 0.0 else -0.074, 0.220), rpy=cylinder_y),
            material=warm_metal,
            name=f"altitude_bushing_{side}",
        )

    # Main optical tube: a short objective housing with glass, rear socket, and
    # side trunnions for the altitude axis.
    scope = model.part("scope_body")
    scope.visual(
        Cylinder(radius=0.052, length=0.235),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=cylinder_x),
        material=satin_black,
        name="objective_housing",
    )
    scope.visual(
        Cylinder(radius=0.062, length=0.040),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=cylinder_x),
        material=rubber_black,
        name="objective_rim",
    )
    scope.visual(
        Cylinder(radius=0.047, length=0.007),
        origin=Origin(xyz=(0.2285, 0.0, 0.0), rpy=cylinder_x),
        material=glass_green,
        name="objective_glass",
    )
    scope.visual(
        Cylinder(radius=0.038, length=0.105),
        origin=Origin(xyz=(-0.081, 0.0, 0.0), rpy=cylinder_x),
        material=satin_black,
        name="rear_socket",
    )
    scope.visual(
        Box((0.170, 0.017, 0.012)),
        origin=Origin(xyz=(0.064, 0.0, 0.057)),
        material=warm_metal,
        name="top_sight_rail",
    )
    scope.visual(
        Box((0.080, 0.040, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, -0.060)),
        material=dark_graphite,
        name="mount_foot",
    )
    for side, y in enumerate((-0.055, 0.055)):
        scope.visual(
            Cylinder(radius=0.017, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cylinder_y),
            material=warm_metal,
            name=f"trunnion_{side}",
        )

    # The eyepiece and draw-tube move together on the focus slide.
    drawtube = model.part("drawtube")
    drawtube.visual(
        Cylinder(radius=0.027, length=0.190),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=cylinder_x),
        material=dark_graphite,
        name="drawtube_barrel",
    )
    drawtube.visual(
        Cylinder(radius=0.034, length=0.062),
        origin=Origin(xyz=(-0.164, 0.0, 0.0), rpy=cylinder_x),
        material=rubber_black,
        name="eyepiece_body",
    )
    drawtube.visual(
        Cylinder(radius=0.020, length=0.007),
        origin=Origin(xyz=(-0.1985, 0.0, 0.0), rpy=cylinder_x),
        material=glass_green,
        name="eyepiece_glass",
    )
    drawtube.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(xyz=(-0.098, 0.0, 0.0), rpy=cylinder_x),
        material=warm_metal,
        name="focus_collar",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "altitude",
        ArticulationType.REVOLUTE,
        parent=head,
        child=scope,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.45, upper=1.05),
    )
    model.articulation(
        "focus",
        ArticulationType.PRISMATIC,
        parent=scope,
        child=drawtube,
        origin=Origin(xyz=(-0.100, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.050),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    scope = object_model.get_part("scope_body")
    drawtube = object_model.get_part("drawtube")
    tripod = object_model.get_part("tripod")
    head = object_model.get_part("head")
    azimuth = object_model.get_articulation("azimuth")
    altitude = object_model.get_articulation("altitude")
    focus = object_model.get_articulation("focus")

    ctx.allow_overlap(
        scope,
        drawtube,
        elem_a="rear_socket",
        elem_b="drawtube_barrel",
        reason=(
            "The focus tube is intentionally retained inside the rear socket; "
            "the simple socket proxy represents the hidden sliding sleeve."
        ),
    )

    ctx.expect_within(
        drawtube,
        scope,
        axes="yz",
        inner_elem="drawtube_barrel",
        outer_elem="rear_socket",
        margin=0.002,
        name="focus drawtube is centered in rear socket",
    )
    ctx.expect_overlap(
        drawtube,
        scope,
        axes="x",
        elem_a="drawtube_barrel",
        elem_b="rear_socket",
        min_overlap=0.070,
        name="collapsed focus tube remains inserted",
    )

    rest_drawtube_pos = ctx.part_world_position(drawtube)
    with ctx.pose({focus: 0.050}):
        ctx.expect_within(
            drawtube,
            scope,
            axes="yz",
            inner_elem="drawtube_barrel",
            outer_elem="rear_socket",
            margin=0.002,
            name="extended focus drawtube stays centered",
        )
        ctx.expect_overlap(
            drawtube,
            scope,
            axes="x",
            elem_a="drawtube_barrel",
            elem_b="rear_socket",
            min_overlap=0.025,
            name="extended focus tube retains insertion",
        )
        extended_drawtube_pos = ctx.part_world_position(drawtube)
    ctx.check(
        "focus extension moves eyepiece rearward",
        rest_drawtube_pos is not None
        and extended_drawtube_pos is not None
        and extended_drawtube_pos[0] < rest_drawtube_pos[0] - 0.040,
        details=f"rest={rest_drawtube_pos}, extended={extended_drawtube_pos}",
    )

    rest_objective_aabb = ctx.part_element_world_aabb(scope, elem="objective_glass")
    rest_objective_z = (
        None
        if rest_objective_aabb is None
        else 0.5 * (rest_objective_aabb[0][2] + rest_objective_aabb[1][2])
    )
    with ctx.pose({altitude: 0.60}):
        raised_objective_aabb = ctx.part_element_world_aabb(scope, elem="objective_glass")
        raised_objective_z = (
            None
            if raised_objective_aabb is None
            else 0.5 * (raised_objective_aabb[0][2] + raised_objective_aabb[1][2])
        )
        ctx.expect_origin_gap(scope, head, axis="z", min_gap=0.0, name="scope remains above fork head")
    ctx.check(
        "altitude joint lifts objective end",
        rest_objective_z is not None
        and raised_objective_z is not None
        and raised_objective_z > rest_objective_z + 0.08,
        details=f"rest_z={rest_objective_z}, raised_z={raised_objective_z}",
    )

    with ctx.pose({azimuth: math.pi / 2.0}):
        ctx.expect_contact(
            tripod,
            head,
            elem_a="top_casting",
            elem_b="azimuth_bearing",
            contact_tol=0.001,
            name="azimuth bearing stays seated on tripod",
        )

    return ctx.report()


object_model = build_object_model()
