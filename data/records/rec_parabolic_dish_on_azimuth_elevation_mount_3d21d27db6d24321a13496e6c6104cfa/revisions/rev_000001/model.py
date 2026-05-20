from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _reflector_shell(radius: float, depth: float, thickness: float) -> MeshGeometry:
    """Thin parabolic reflector with a real rim edge, opening along local +Y."""

    radial_steps = 12
    segments = 96
    geom = MeshGeometry()

    def inner_y(r: float) -> float:
        # The vertex is recessed and the lip sits farther forward.
        return 0.025 + depth * (r / radius) ** 2

    inner_center = geom.add_vertex(0.0, inner_y(0.0), 0.0)
    outer_center = geom.add_vertex(0.0, inner_y(0.0) - thickness, 0.0)
    inner_rings: list[list[int]] = []
    outer_rings: list[list[int]] = []

    for i in range(1, radial_steps + 1):
        r = radius * i / radial_steps
        yi = inner_y(r)
        yo = yi - thickness
        inner_ring: list[int] = []
        outer_ring: list[int] = []
        for j in range(segments):
            a = 2.0 * math.pi * j / segments
            x = r * math.cos(a)
            z = r * math.sin(a)
            inner_ring.append(geom.add_vertex(x, yi, z))
            outer_ring.append(geom.add_vertex(x, yo, z))
        inner_rings.append(inner_ring)
        outer_rings.append(outer_ring)

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for j in range(segments):
        jn = (j + 1) % segments
        geom.add_face(inner_center, inner_rings[0][j], inner_rings[0][jn])
        geom.add_face(outer_center, outer_rings[0][jn], outer_rings[0][j])

    for i in range(radial_steps - 1):
        for j in range(segments):
            jn = (j + 1) % segments
            add_quad(inner_rings[i][j], inner_rings[i + 1][j], inner_rings[i + 1][jn], inner_rings[i][jn])
            add_quad(outer_rings[i][jn], outer_rings[i + 1][jn], outer_rings[i + 1][j], outer_rings[i][j])

    # Close the physical lip between the inner and outer skins.
    for j in range(segments):
        jn = (j + 1) % segments
        add_quad(inner_rings[-1][j], outer_rings[-1][j], outer_rings[-1][jn], inner_rings[-1][jn])

    return geom


def _cheek_plate(x_offset: float) -> cq.Workplane:
    """Side cheek plate in the YZ plane, with a real clearance hole for the axle."""

    outer_profile = [
        (-0.18, 0.060),
        (0.18, 0.060),
        (0.18, 0.430),
        (0.105, 0.685),
        (-0.105, 0.685),
        (-0.18, 0.430),
    ]
    hole_radius = 0.062
    thickness = 0.040

    plate = cq.Workplane("YZ").polyline(outer_profile).close().extrude(thickness, both=True)
    hole = cq.Workplane("YZ").center(0.0, 0.580).circle(hole_radius).extrude(thickness * 4.0, both=True)
    boss = cq.Workplane("YZ").center(0.0, 0.580).circle(0.105).extrude(0.058, both=True).cut(hole)
    plate = plate.cut(hole).union(boss)
    return plate.translate((x_offset, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_dish")

    dark = model.material("matte_dark_graphite", rgba=(0.035, 0.040, 0.045, 1.0))
    black = model.material("black_bearing", rgba=(0.005, 0.006, 0.007, 1.0))
    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    reflector_finish = model.material("pale_reflector", rgba=(0.84, 0.86, 0.82, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.50, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black,
        name="lower_disk",
    )

    base_ring = model.part("base_ring")
    ring = TorusGeometry(radius=0.440, tube=0.028, radial_segments=32, tubular_segments=96)
    base_ring.visual(
        mesh_from_geometry(ring, "azimuth_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark,
        name="azimuth_ring",
    )
    base_ring.visual(
        Cylinder(radius=0.095, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark,
        name="center_hub",
    )
    base_ring.visual(
        Box((1.10, 0.205, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark,
        name="yoke_foot",
    )
    base_ring.visual(
        mesh_from_cadquery(_cheek_plate(0.520), "cheek_0"),
        material=dark,
        name="cheek_0",
    )
    base_ring.visual(
        mesh_from_cadquery(_cheek_plate(-0.520), "cheek_1"),
        material=dark,
        name="cheek_1",
    )

    reflector = model.part("reflector")
    dish_radius = 0.455
    dish_depth = 0.115
    reflector.visual(
        mesh_from_geometry(_reflector_shell(dish_radius, dish_depth, 0.012), "dish_shell"),
        material=reflector_finish,
        name="dish_shell",
    )
    rim = TorusGeometry(radius=dish_radius, tube=0.014, radial_segments=20, tubular_segments=96)
    rim.rotate_x(math.pi / 2.0).translate(0.0, 0.025 + dish_depth, 0.0)
    reflector.visual(
        mesh_from_geometry(rim, "rolled_rim"),
        material=aluminum,
        name="rolled_rim",
    )
    reflector.visual(
        Cylinder(radius=0.034, length=1.160),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="elevation_axle",
    )
    reflector.visual(
        Box((0.080, 0.045, 0.620)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=dark,
        name="vertical_frame",
    )
    reflector.visual(
        Box((0.650, 0.045, 0.070)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=dark,
        name="cross_frame",
    )
    reflector.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="center_boss",
    )
    reflector.visual(
        Cylinder(radius=0.078, length=0.030),
        origin=Origin(xyz=(0.593, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="axle_collar_0",
    )
    reflector.visual(
        Cylinder(radius=0.078, length=0.030),
        origin=Origin(xyz=(-0.593, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="axle_collar_1",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=base_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=base_ring,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.30, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    base_ring = object_model.get_part("base_ring")
    reflector = object_model.get_part("reflector")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.expect_gap(
        base_ring,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating ring sits just above fixed lower bearing",
    )
    ctx.expect_overlap(
        reflector,
        base_ring,
        axes="yz",
        elem_a="elevation_axle",
        elem_b="cheek_0",
        min_overlap=0.025,
        name="axle passes through first cheek bearing",
    )
    ctx.expect_overlap(
        reflector,
        base_ring,
        axes="yz",
        elem_a="elevation_axle",
        elem_b="cheek_1",
        min_overlap=0.025,
        name="axle passes through second cheek bearing",
    )

    rest_dish = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    with ctx.pose({elevation: 0.55}):
        raised_dish = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    ctx.check(
        "positive elevation tips reflector upward",
        rest_dish is not None
        and raised_dish is not None
        and raised_dish[0][2] > rest_dish[0][2] + 0.045,
        details=f"rest={rest_dish}, raised={raised_dish}",
    )

    rest_aabb = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(reflector, elem="dish_shell")
    rest_center_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    turned_center_x = None if turned_aabb is None else (turned_aabb[0][0] + turned_aabb[1][0]) * 0.5
    ctx.check(
        "azimuth rotates dish around vertical axis",
        rest_center_y is not None and turned_center_x is not None and rest_center_y > 0.05 and turned_center_x < -0.05,
        details=f"rest_center_y={rest_center_y}, turned_center_x={turned_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
