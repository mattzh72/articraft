from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_slab(width: float, height: float, depth: float, corner_radius: float):
    """Rounded-rectangle slab with its rear face on local z=0."""
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, height, corner_radius, corner_segments=10),
        depth,
        cap=True,
    )


def _annulus(outer_radius: float, inner_radius: float, depth: float):
    """Flat annular mesh with its rear face on local z=0."""
    segments = 96
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, depth))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, depth))

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for index in range(segments):
        nxt = (index + 1) % segments
        quad(outer_bottom[index], outer_bottom[nxt], outer_top[nxt], outer_top[index])
        quad(inner_bottom[nxt], inner_bottom[index], inner_top[index], inner_top[nxt])
        quad(outer_top[index], outer_top[nxt], inner_top[nxt], inner_top[index])
        quad(outer_bottom[nxt], outer_bottom[index], inner_bottom[index], inner_bottom[nxt])
    return geom


def _scale_track():
    """Connected minute/major tick track for the fixed thermostat dial face."""
    return _annulus(0.0445, 0.0432, 0.00045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wall_thermostat_rotary_dial")

    matte_porcelain = model.material("matte_porcelain", rgba=(0.86, 0.84, 0.79, 1.0))
    satin_face = model.material("satin_warm_face", rgba=(0.91, 0.89, 0.84, 1.0))
    shadow = model.material("controlled_shadow", rgba=(0.035, 0.036, 0.034, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.68, 0.66, 0.61, 1.0))
    graphite = model.material("satin_graphite", rgba=(0.11, 0.115, 0.12, 1.0))
    pale_mark = model.material("warm_indicator", rgba=(0.82, 0.80, 0.73, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_rounded_slab(0.122, 0.122, 0.0036, 0.018), "wall_plate"),
        material=matte_porcelain,
        name="wall_plate",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.111, 0.111),
                (0.119, 0.119),
                0.0012,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.016,
                outer_corner_radius=0.019,
                center=False,
            ),
            "perimeter_shadow",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.00325)),
        material=shadow,
        name="perimeter_shadow",
    )
    body.visual(
        mesh_from_geometry(_rounded_slab(0.109, 0.109, 0.0140, 0.0155), "front_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=matte_porcelain,
        name="front_shell",
    )
    body.visual(
        mesh_from_geometry(_rounded_slab(0.102, 0.102, 0.0014, 0.0135), "satin_front_face"),
        origin=Origin(xyz=(0.0, 0.0, 0.0171)),
        material=satin_face,
        name="satin_front_face",
    )
    body.visual(
        mesh_from_geometry(_annulus(0.0490, 0.0467, 0.00065), "outer_dial_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.01815)),
        material=shadow,
        name="outer_dial_gasket",
    )
    body.visual(
        mesh_from_geometry(_annulus(0.0464, 0.0335, 0.00235), "dial_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.01835)),
        material=satin_aluminum,
        name="dial_ring",
    )
    body.visual(
        mesh_from_geometry(_scale_track(), "scale_track"),
        origin=Origin(xyz=(0.0, 0.0, 0.02055)),
        material=graphite,
        name="scale_track",
    )
    for index in range(48):
        major = index % 6 == 0
        length = 0.0072 if major else 0.0046
        width = 0.00105 if major else 0.00055
        radius = 0.0405 if major else 0.0418
        angle = index * (2.0 * math.pi / 48.0)
        body.visual(
            Box((length, width, 0.00072)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.02095),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"scale_tick_{index}",
        )
    body.visual(
        mesh_from_geometry(_annulus(0.0343, 0.0319, 0.00075), "inner_dial_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.02062)),
        material=shadow,
        name="inner_dial_gasket",
    )
    body.visual(
        mesh_from_geometry(_annulus(0.0155, 0.0088, 0.0062), "bearing_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.01835)),
        material=satin_aluminum,
        name="bearing_sleeve",
    )
    for index, y in enumerate((-0.052, 0.052)):
        body.visual(
            Cylinder(radius=0.0023, length=0.0007),
            origin=Origin(xyz=(0.0, y, 0.01835)),
            material=satin_aluminum,
            name=f"screw_cap_{index}",
        )
        body.visual(
            Cylinder(radius=0.0013, length=0.0008),
            origin=Origin(xyz=(0.0, y, 0.01878)),
            material=shadow,
            name=f"screw_recess_{index}",
        )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.0145,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=72, depth=0.00045, width=0.0009),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.00035,
                    angle_deg=90.0,
                ),
                center=False,
            ),
            "rotary_knob_shell",
        ),
        material=graphite,
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.0072, length=0.0061),
        origin=Origin(xyz=(0.0, 0.0, -0.00305)),
        material=graphite,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.0118, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.01425)),
        material=shadow,
        name="center_medallion",
    )
    knob.visual(
        Box((0.0039, 0.0185, 0.00072)),
        origin=Origin(xyz=(0.0, 0.0158, 0.01505)),
        material=pale_mark,
        name="indicator_bar",
    )

    model.articulation(
        "body_to_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, 0.02455)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.22, velocity=2.4, lower=-2.35, upper=2.35),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    knob = object_model.get_part("knob")
    dial = object_model.get_articulation("body_to_knob")

    ctx.check("rotary_dial_present", knob is not None and dial is not None)
    if body is None or knob is None or dial is None:
        return ctx.report()

    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        elem_a="shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.012,
        name="shaft is centered inside bearing sleeve footprint",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="z",
        elem_a="shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.0045,
        name="shaft has practical bearing engagement depth",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        positive_elem="knob_shell",
        negative_elem="bearing_sleeve",
        max_gap=0.00015,
        max_penetration=0.00002,
        name="knob seats on stationary bearing collar without penetration",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        positive_elem="knob_shell",
        negative_elem="dial_ring",
        min_gap=0.0030,
        max_gap=0.0058,
        name="dial cap floats cleanly above face ring",
    )

    rest_aabb = ctx.part_element_world_aabb(knob, elem="indicator_bar")
    rest_pos = ctx.part_world_position(knob)
    with ctx.pose({dial: 1.2}):
        turned_aabb = ctx.part_element_world_aabb(knob, elem="indicator_bar")
        turned_pos = ctx.part_world_position(knob)

    def _center(aabb):
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    if rest_aabb is not None and turned_aabb is not None:
        rest_center = _center(rest_aabb)
        turned_center = _center(turned_aabb)
        moved_laterally = math.hypot(
            turned_center[0] - rest_center[0],
            turned_center[1] - rest_center[1],
        )
    else:
        rest_center = None
        turned_center = None
        moved_laterally = 0.0
    ctx.check(
        "indicator rotates about fixed center axis",
        rest_center is not None
        and turned_center is not None
        and moved_laterally > 0.014
        and rest_pos is not None
        and turned_pos is not None
        and max(abs(rest_pos[i] - turned_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_center}, turned={turned_center}, rest_pos={rest_pos}, turned_pos={turned_pos}",
    )
    return ctx.report()


object_model = build_object_model()
