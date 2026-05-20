from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIS_TO_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _tube_geometry(
    z0: float,
    z1: float,
    outer_radius: float,
    inner_radius: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    """A hollow, capped tube authored along local Z."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _ribbed_tube_geometry(
    length: float,
    inner_radius: float,
    valley_outer_radius: float,
    rib_depth: float,
    rib_count: int,
    *,
    segments: int = 144,
) -> MeshGeometry:
    """A hollow rubber grip sleeve with longitudinal raised ribs."""
    geom = MeshGeometry()
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []

    for z in (-length / 2.0, length / 2.0):
        outer_loop: list[int] = []
        inner_loop: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            # Smooth scalloped ribs: valleys sit at valley_outer_radius, crests
            # add rib_depth. The exponent flattens the valleys like molded rubber.
            wave = 0.5 + 0.5 * math.cos(rib_count * theta)
            outer_radius = valley_outer_radius + rib_depth * (wave**2.2)
            x = math.cos(theta)
            y = math.sin(theta)
            outer_loop.append(geom.add_vertex(outer_radius * x, outer_radius * y, z))
            inner_loop.append(geom.add_vertex(inner_radius * x, inner_radius * y, z))
        outer_loops.append(outer_loop)
        inner_loops.append(inner_loop)

    for i in range(segments):
        j = (i + 1) % segments
        o0_i, o0_j = outer_loops[0][i], outer_loops[0][j]
        o1_i, o1_j = outer_loops[1][i], outer_loops[1][j]
        n0_i, n0_j = inner_loops[0][i], inner_loops[0][j]
        n1_i, n1_j = inner_loops[1][i], inner_loops[1][j]

        # Outer and inner cylindrical walls.
        geom.add_face(o0_i, o0_j, o1_j)
        geom.add_face(o0_i, o1_j, o1_i)
        geom.add_face(n0_j, n0_i, n1_i)
        geom.add_face(n0_j, n1_i, n1_j)

        # Flat annular end faces, leaving the center open.
        geom.add_face(o0_i, n0_j, o0_j)
        geom.add_face(o0_i, n0_i, n0_j)
        geom.add_face(o1_i, o1_j, n1_j)
        geom.add_face(o1_i, n1_j, n1_i)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_zoom_lens")

    matte_black = Material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_black = Material("satin_black", rgba=(0.015, 0.015, 0.018, 1.0))
    rubber = Material("ribbed_black_rubber", rgba=(0.001, 0.001, 0.001, 1.0))
    gunmetal = Material("dark_gunmetal", rgba=(0.16, 0.15, 0.14, 1.0))
    glass = Material("coated_glass", rgba=(0.06, 0.16, 0.20, 0.55))
    white = Material("engraved_white_paint", rgba=(0.92, 0.92, 0.86, 1.0))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        mesh_from_geometry(_tube_geometry(-0.065, 0.065, 0.043, 0.036), "outer_barrel_shell"),
        origin=AXIS_TO_X,
        material=matte_black,
        name="main_shell",
    )
    outer_barrel.visual(
        mesh_from_geometry(_tube_geometry(-0.065, -0.036, 0.046, 0.036), "rear_fixed_collar"),
        origin=AXIS_TO_X,
        material=satin_black,
        name="rear_collar",
    )
    outer_barrel.visual(
        mesh_from_geometry(_tube_geometry(0.033, 0.065, 0.046, 0.036), "front_fixed_collar"),
        origin=AXIS_TO_X,
        material=satin_black,
        name="front_collar",
    )
    outer_barrel.visual(
        mesh_from_geometry(_tube_geometry(-0.073, -0.065, 0.041, 0.024), "bayonet_mount_plate"),
        origin=AXIS_TO_X,
        material=gunmetal,
        name="bayonet_plate",
    )
    for index, (y, z, sy, sz) in enumerate(
        (
            (0.000, 0.040, 0.022, 0.009),
            (0.033, -0.020, 0.014, 0.020),
            (-0.033, -0.020, 0.014, 0.020),
        )
    ):
        outer_barrel.visual(
            Box((0.006, sy, sz)),
            origin=Origin(xyz=(-0.075, y, z)),
            material=gunmetal,
            name=f"bayonet_lug_{index}",
        )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_geometry(
            _ribbed_tube_geometry(0.056, 0.0430, 0.0478, 0.0026, 32),
            "ribbed_zoom_ring",
        ),
        origin=AXIS_TO_X,
        material=rubber,
        name="ribbed_grip",
    )
    zoom_ring.visual(
        Box((0.026, 0.0035, 0.0010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0506)),
        material=white,
        name="zoom_mark",
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        mesh_from_geometry(_tube_geometry(-0.035, 0.060, 0.036, 0.0275), "inner_extending_barrel"),
        origin=AXIS_TO_X,
        material=satin_black,
        name="inner_sleeve",
    )
    inner_barrel.visual(
        mesh_from_geometry(_tube_geometry(0.052, 0.061, 0.036, 0.026), "front_retaining_ring"),
        origin=AXIS_TO_X,
        material=matte_black,
        name="front_retainer",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0268, length=0.005),
        origin=Origin(xyz=(0.0595, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            _ribbed_tube_geometry(0.030, 0.0360, 0.0398, 0.0018, 40),
            "ribbed_focus_ring",
        ),
        origin=AXIS_TO_X,
        material=rubber,
        name="ribbed_grip",
    )
    focus_ring.visual(
        Box((0.014, 0.0028, 0.0009)),
        origin=Origin(xyz=(0.000, 0.0, 0.0418)),
        material=white,
        name="focus_mark",
    )

    zoom_joint = model.articulation(
        "outer_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=zoom_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=math.radians(110.0)),
    )
    model.articulation(
        "outer_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.035),
        meta={
            "driven_by": zoom_joint.name,
            "motion_ratio_m_per_rad": 0.035 / math.radians(110.0),
        },
    )
    model.articulation(
        "inner_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=-math.radians(95.0), upper=math.radians(95.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_barrel")
    zoom = object_model.get_part("zoom_ring")
    inner = object_model.get_part("inner_barrel")
    focus = object_model.get_part("focus_ring")
    zoom_joint = object_model.get_articulation("outer_to_zoom_ring")
    inner_slide = object_model.get_articulation("outer_to_inner_barrel")

    ctx.allow_overlap(
        inner,
        outer,
        elem_a="inner_sleeve",
        elem_b="main_shell",
        reason=(
            "The extending barrel is intentionally nested inside the outer lens barrel; "
            "the annular sleeve is represented as a close sliding fit."
        ),
    )
    ctx.allow_overlap(
        inner,
        outer,
        elem_a="inner_sleeve",
        elem_b="front_collar",
        reason=(
            "The inner extension passes through the fixed front guide collar; "
            "this hidden sliding bearing is intentionally nested."
        ),
    )
    ctx.allow_overlap(
        outer,
        zoom,
        elem_a="main_shell",
        elem_b="ribbed_grip",
        reason=(
            "The zoom ring is a rotating sleeve captured around the outer barrel, "
            "with the ring wall represented as a close annular fit."
        ),
    )
    ctx.allow_overlap(
        focus,
        inner,
        elem_a="ribbed_grip",
        elem_b="inner_sleeve",
        reason=(
            "The manual focus ring is a rotating sleeve captured around the extending barrel."
        ),
    )

    ctx.expect_overlap(
        zoom,
        outer,
        axes="x",
        elem_a="ribbed_grip",
        elem_b="main_shell",
        min_overlap=0.050,
        name="zoom ring wraps the middle of the outer barrel",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_sleeve",
        elem_b="main_shell",
        min_overlap=0.050,
        name="collapsed inner barrel remains guided inside outer barrel",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        inner_elem="inner_sleeve",
        outer_elem="main_shell",
        margin=0.001,
        name="inner barrel stays centered in the outer sleeve",
    )
    ctx.expect_overlap(
        focus,
        inner,
        axes="x",
        elem_a="ribbed_grip",
        elem_b="inner_sleeve",
        min_overlap=0.028,
        name="focus ring is carried on the extending front barrel",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_sleeve",
        elem_b="front_collar",
        min_overlap=0.030,
        name="front collar captures the sliding inner barrel",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({zoom_joint: math.radians(110.0), inner_slide: 0.035}):
        extended_pos = ctx.part_world_position(inner)
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="inner_sleeve",
            elem_b="main_shell",
            min_overlap=0.018,
            name="extended inner barrel keeps retained insertion",
        )

    ctx.check(
        "zoom ring declares mechanical drive for barrel extension",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.030
        and inner_slide.meta.get("driven_by") == zoom_joint.name,
        details=f"rest={rest_pos}, extended={extended_pos}, meta={inner_slide.meta}",
    )

    return ctx.report()


object_model = build_object_model()
