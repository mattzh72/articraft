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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_between(part, name, radius, p0, p1, material):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _annular_x(length, outer_radius, inner_radius, x0, z0):
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x0, 0.0, z0))
    )


def _annular_y(length, outer_radius, inner_radius, y_center):
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, y_center + length * 0.5, 0.0))
    )


def _launcher_tube():
    tube_length = 1.25
    rear_x = -0.45
    tube_z = 0.34
    shell = _annular_x(tube_length, 0.075, 0.055, rear_x, tube_z)
    front_collar = _annular_x(0.060, 0.088, 0.056, rear_x + tube_length - 0.060, tube_z)
    rear_collar = _annular_x(0.055, 0.084, 0.056, rear_x, tube_z)
    return shell.union(front_collar).union(rear_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_missile_launcher")

    olive = model.material("olive_drab", rgba=(0.30, 0.36, 0.22, 1.0))
    dark = model.material("parkerized_dark_steel", rgba=(0.08, 0.085, 0.075, 1.0))
    bare = model.material("worn_pin_steel", rgba=(0.55, 0.56, 0.52, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.130, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.880)),
        material=dark,
        name="yaw_base_plate",
    )
    tripod.visual(
        Cylinder(radius=0.026, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=dark,
        name="center_column",
    )
    tripod.visual(
        Sphere(radius=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=dark,
        name="apex_hub",
    )
    leg_top = (0.0, 0.0, 0.700)
    for i, angle in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        foot = (0.56 * math.cos(angle), 0.56 * math.sin(angle), 0.024)
        _cylinder_between(tripod, f"leg_{i}", 0.022, leg_top, foot, dark)
        tripod.visual(
            Cylinder(radius=0.070, length=0.026),
            origin=Origin(xyz=(foot[0], foot[1], 0.013)),
            material=rubber,
            name=f"foot_{i}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.112, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark,
        name="yaw_disc",
    )
    head.visual(
        Cylinder(radius=0.042, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark,
        name="pedestal",
    )
    head.visual(
        Box((0.120, 0.225, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=dark,
        name="yoke_bridge",
    )
    for side, y in enumerate((-0.105, 0.105)):
        head.visual(
            Box((0.105, 0.035, 0.205)),
            origin=Origin(xyz=(0.0, y, 0.160)),
            material=dark,
            name=f"side_cheek_{side}",
        )
    head.visual(
        Cylinder(radius=0.032, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare,
        name="elevation_axle",
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_launcher_tube(), "launcher_tube", tolerance=0.001, angular_tolerance=0.08),
        material=olive,
        name="tube_shell",
    )
    for i, x in enumerate((-0.120, 0.120)):
        cradle.visual(
            mesh_from_cadquery(
                _annular_x(0.045, 0.091, 0.073, x - 0.0225, 0.340),
                f"clamp_band_{i}",
                tolerance=0.001,
                angular_tolerance=0.08,
            ),
            material=dark,
            name=f"clamp_band_{i}",
        )
    cradle.visual(
        Box((0.330, 0.130, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.255)),
        material=dark,
        name="tube_saddle",
    )
    for side, y in enumerate((-0.155, 0.155)):
        cradle.visual(
            mesh_from_cadquery(
                _annular_y(0.046, 0.054, 0.032, y),
                f"clip_sleeve_{side}",
                tolerance=0.001,
                angular_tolerance=0.08,
            ),
            material=dark,
            name=f"clip_sleeve_{side}",
        )
        cradle.visual(
            Box((0.265, 0.026, 0.250)),
            origin=Origin(xyz=(0.0, y, 0.177)),
            material=dark,
            name=f"side_plate_{side}",
        )
        cradle.visual(
            Box((0.265, 0.104, 0.044)),
            origin=Origin(xyz=(0.0, y * 0.67, 0.255)),
            material=dark,
            name=f"side_web_{side}",
        )
        cradle.visual(
            Box((0.265, 0.058, 0.050)),
            origin=Origin(xyz=(0.0, y * 0.82, 0.302)),
            material=dark,
            name=f"upper_clip_bridge_{side}",
        )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=head,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.8, lower=-0.15, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    head = object_model.get_part("head")
    cradle = object_model.get_part("cradle")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.expect_gap(
        head,
        tripod,
        axis="z",
        positive_elem="yaw_disc",
        negative_elem="yaw_base_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating head sits on yaw plate",
    )
    for sleeve in ("clip_sleeve_0", "clip_sleeve_1"):
        ctx.allow_overlap(
            cradle,
            head,
            elem_a=sleeve,
            elem_b="elevation_axle",
            reason="Each cradle clip sleeve intentionally captures the side trunnion axle as a simplified bearing fit.",
        )
        ctx.expect_within(
            head,
            cradle,
            axes="xz",
            inner_elem="elevation_axle",
            outer_elem=sleeve,
            margin=0.003,
            name=f"axle centered inside {sleeve}",
        )
        ctx.expect_overlap(
            head,
            cradle,
            axes="y",
            elem_a="elevation_axle",
            elem_b=sleeve,
            min_overlap=0.035,
            name=f"{sleeve} remains axially captured on trunnion",
        )

    rest_tube = ctx.part_element_world_aabb(cradle, elem="tube_shell")
    with ctx.pose({elevation: 0.70}):
        raised_tube = ctx.part_element_world_aabb(cradle, elem="tube_shell")
        ctx.expect_within(
            head,
            cradle,
            axes="xz",
            inner_elem="elevation_axle",
            outer_elem="clip_sleeve_0",
            margin=0.003,
            name="left clip stays coaxial while elevating",
        )
    ctx.check(
        "elevation raises launcher muzzle",
        rest_tube is not None
        and raised_tube is not None
        and raised_tube[1][2] > rest_tube[1][2] + 0.20,
        details=f"rest={rest_tube}, raised={raised_tube}",
    )

    rest_head = ctx.part_element_world_aabb(cradle, elem="tube_shell")
    with ctx.pose({azimuth: 0.65}):
        yawed_head = ctx.part_element_world_aabb(cradle, elem="tube_shell")
    ctx.check(
        "azimuth yaws tube about vertical axis",
        rest_head is not None
        and yawed_head is not None
        and abs((yawed_head[0][1] + yawed_head[1][1]) * 0.5) > 0.10,
        details=f"rest={rest_head}, yawed={yawed_head}",
    )

    return ctx.report()


object_model = build_object_model()
