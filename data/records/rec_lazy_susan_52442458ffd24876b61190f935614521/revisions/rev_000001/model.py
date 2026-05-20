from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _oval_plate(width: float, depth: float, height: float, exponent: float = 2.35):
    return ExtrudeGeometry.from_z0(
        superellipse_profile(width, depth, exponent=exponent, segments=96),
        height,
        cap=True,
        closed=True,
    )


def _annular_disk(outer_radius: float, inner_radius: float, height: float):
    geom = MeshGeometry()
    segments = 96
    z0 = -height / 2.0
    z1 = height / 2.0
    bottom_outer: list[int] = []
    bottom_inner: list[int] = []
    top_outer: list[int] = []
    top_inner: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        co = math.cos(angle)
        si = math.sin(angle)
        bottom_outer.append(geom.add_vertex(outer_radius * co, outer_radius * si, z0))
        bottom_inner.append(geom.add_vertex(inner_radius * co, inner_radius * si, z0))
        top_outer.append(geom.add_vertex(outer_radius * co, outer_radius * si, z1))
        top_inner.append(geom.add_vertex(inner_radius * co, inner_radius * si, z1))
    for i in range(segments):
        j = (i + 1) % segments
        # Outer cylindrical wall.
        geom.add_face(bottom_outer[i], bottom_outer[j], top_outer[j])
        geom.add_face(bottom_outer[i], top_outer[j], top_outer[i])
        # Inner cylindrical wall around the clearance hole.
        geom.add_face(bottom_inner[i], top_inner[j], bottom_inner[j])
        geom.add_face(bottom_inner[i], top_inner[i], top_inner[j])
        # Top annular face.
        geom.add_face(top_outer[i], top_outer[j], top_inner[j])
        geom.add_face(top_outer[i], top_inner[j], top_inner[i])
        # Bottom annular face.
        geom.add_face(bottom_outer[i], bottom_inner[j], bottom_outer[j])
        geom.add_face(bottom_outer[i], bottom_inner[i], bottom_inner[j])
    return geom


def _tray_body_cadquery() -> cq.Workplane:
    """One continuous shallow oval tray with a raised ring-like outer rim."""
    floor = (
        cq.Workplane("XY")
        .ellipse(0.215, 0.135)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.018))
    )
    rim_outer = cq.Workplane("XY").ellipse(0.235, 0.155).extrude(0.036)
    rim_cut = (
        cq.Workplane("XY")
        .ellipse(0.198, 0.118)
        .extrude(0.046)
        .translate((0.0, 0.0, -0.005))
    )
    rim = (
        rim_outer.cut(rim_cut)
        .translate((0.0, 0.0, 0.020))
    )
    return floor.union(rim)


def _handle_mesh(x_sign: float):
    x_attach = x_sign * 0.206
    x_out = x_sign * 0.236
    return tube_from_spline_points(
        [
            (x_attach, -0.070, 0.049),
            (x_out, -0.048, 0.066),
            (x_out, 0.000, 0.074),
            (x_out, 0.048, 0.066),
            (x_attach, 0.070, 0.049),
        ],
        radius=0.006,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_condiment_lazy_susan")

    bamboo = model.material("warm_bamboo", rgba=(0.78, 0.56, 0.32, 1.0))
    dark_bamboo = model.material("darker_bamboo_edge", rgba=(0.55, 0.34, 0.16, 1.0))
    satin_metal = model.material("satin_bearing_metal", rgba=(0.66, 0.66, 0.62, 1.0))
    shadow = model.material("soft_shadow_rubber", rgba=(0.035, 0.032, 0.030, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_oval_plate(0.330, 0.225, 0.022), "lower_oval_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_bamboo,
        name="oval_foot",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=bamboo,
        name="pedestal_plinth",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=bamboo,
        name="short_pedestal",
    )
    base.visual(
        Cylinder(radius=0.083, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=satin_metal,
        name="lower_turntable",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=satin_metal,
        name="center_boss",
    )
    base.visual(
        mesh_from_geometry(_oval_plate(0.300, 0.195, 0.004), "rubber_foot_pad"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=shadow,
        name="rubber_pad",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_geometry(_annular_disk(0.061, 0.048, 0.014).translate(0.0, 0.0, 0.007), "retaining_clip"),
        origin=Origin(),
        material=satin_metal,
        name="retaining_clip",
    )
    tray.visual(
        mesh_from_geometry(_annular_disk(0.077, 0.048, 0.010).translate(0.0, 0.0, 0.005), "upper_turntable_disk"),
        origin=Origin(),
        material=satin_metal,
        name="upper_turntable",
    )
    tray.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=bamboo,
        name="central_hub",
    )
    tray.visual(
        mesh_from_cadquery(_tray_body_cadquery(), "oval_tray_body", tolerance=0.0007, angular_tolerance=0.08),
        origin=Origin(),
        material=bamboo,
        name="tray_bowl",
    )
    tray.visual(
        mesh_from_geometry(_handle_mesh(1.0), "front_handle_mesh"),
        origin=Origin(),
        material=dark_bamboo,
        name="front_handle",
    )
    tray.visual(
        mesh_from_geometry(_handle_mesh(-1.0), "rear_handle_mesh"),
        origin=Origin(),
        material=dark_bamboo,
        name="rear_handle",
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=3.5),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    spin = object_model.get_articulation("base_to_tray")

    ctx.check(
        "tray uses continuous vertical turntable joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_within(
        tray,
        base,
        axes="xy",
        inner_elem="upper_turntable",
        outer_elem="lower_turntable",
        margin=0.001,
        name="upper turntable stays centered on lower bearing",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        positive_elem="upper_turntable",
        negative_elem="lower_turntable",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper turntable sits on fixed bearing",
    )

    rest_position = ctx.part_world_position(tray)
    with ctx.pose({spin: 1.75}):
        turned_position = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            base,
            axes="xy",
            inner_elem="upper_turntable",
            outer_elem="lower_turntable",
            margin=0.001,
            name="rotated tray remains centered on bearing",
        )
    ctx.check(
        "tray origin remains clipped to pedestal axis during rotation",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6,
        details=f"rest={rest_position}, rotated={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
