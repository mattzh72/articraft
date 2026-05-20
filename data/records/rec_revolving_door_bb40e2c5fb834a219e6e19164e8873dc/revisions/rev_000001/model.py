from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _annular_sector(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    segments: int,
    close_ends: bool = True,
) -> MeshGeometry:
    """Closed, vertical annular sector mesh in world/local coordinates."""
    geom = MeshGeometry()
    angles = [
        start_angle + (end_angle - start_angle) * i / segments
        for i in range(segments + 1)
    ]
    verts: list[tuple[int, int, int, int]] = []
    for angle in angles:
        c = math.cos(angle)
        s = math.sin(angle)
        verts.append(
            (
                geom.add_vertex(inner_radius * c, inner_radius * s, z_min),
                geom.add_vertex(outer_radius * c, outer_radius * s, z_min),
                geom.add_vertex(outer_radius * c, outer_radius * s, z_max),
                geom.add_vertex(inner_radius * c, inner_radius * s, z_max),
            )
        )

    for i in range(segments):
        i0, o0, ot0, it0 = verts[i]
        i1, o1, ot1, it1 = verts[i + 1]
        _add_quad(geom, o0, o1, ot1, ot0)  # outside curve
        _add_quad(geom, i1, i0, it0, it1)  # inside curve
        _add_quad(geom, it0, ot0, ot1, it1)  # top
        _add_quad(geom, i0, i1, o1, o0)  # bottom

    if close_ends:
        _add_quad(geom, verts[0][0], verts[0][1], verts[0][2], verts[0][3])
        _add_quad(geom, verts[-1][1], verts[-1][0], verts[-1][3], verts[-1][2])
    return geom


def _annular_ring(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 96,
) -> MeshGeometry:
    geom = MeshGeometry()
    verts: list[tuple[int, int, int, int]] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        verts.append(
            (
                geom.add_vertex(inner_radius * c, inner_radius * s, z_min),
                geom.add_vertex(outer_radius * c, outer_radius * s, z_min),
                geom.add_vertex(outer_radius * c, outer_radius * s, z_max),
                geom.add_vertex(inner_radius * c, inner_radius * s, z_max),
            )
        )
    for i in range(segments):
        i0, o0, ot0, it0 = verts[i]
        i1, o1, ot1, it1 = verts[(i + 1) % segments]
        _add_quad(geom, o0, o1, ot1, ot0)
        _add_quad(geom, i1, i0, it0, it1)
        _add_quad(geom, it0, ot0, ot1, it1)
        _add_quad(geom, i0, i1, o1, o0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_airport_revolving_door")

    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    safety_glass = model.material("pale_blue_transparent_glass", rgba=(0.62, 0.82, 0.96, 0.34))
    gasket = model.material("gray_edge_gasket", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("drum_frame")
    # Extra-large terminal drum: nearly four meters across, with opposite openings.
    frame.visual(
        mesh_from_geometry(
            _annular_ring(inner_radius=1.72, outer_radius=1.95, z_min=0.00, z_max=0.075),
            "floor_track_ring",
        ),
        material=dark_aluminum,
        name="floor_track_ring",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_ring(inner_radius=1.72, outer_radius=1.95, z_min=2.36, z_max=2.50),
            "ceiling_header_ring",
        ),
        material=dark_aluminum,
        name="ceiling_header_ring",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_sector(
                inner_radius=1.82,
                outer_radius=1.86,
                z_min=0.075,
                z_max=2.36,
                start_angle=math.radians(32.0),
                end_angle=math.radians(148.0),
                segments=36,
            ),
            "curved_glass_wall_0",
        ),
        material=safety_glass,
        name="curved_glass_wall_0",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_sector(
                inner_radius=1.82,
                outer_radius=1.86,
                z_min=0.075,
                z_max=2.36,
                start_angle=math.radians(212.0),
                end_angle=math.radians(328.0),
                segments=36,
            ),
            "curved_glass_wall_1",
        ),
        material=safety_glass,
        name="curved_glass_wall_1",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_ring(inner_radius=0.215, outer_radius=0.36, z_min=0.00, z_max=0.17),
            "floor_bearing_housing",
        ),
        material=brushed_aluminum,
        name="floor_bearing_housing",
    )
    frame.visual(
        mesh_from_geometry(
            _annular_ring(inner_radius=0.215, outer_radius=0.36, z_min=2.28, z_max=2.50),
            "ceiling_bearing_housing",
        ),
        material=brushed_aluminum,
        name="ceiling_bearing_housing",
    )

    # Low radial tracks and overhead braces visibly tie the central bearing to the big drum.
    for idx, angle in enumerate((0.0, math.pi, math.pi / 2.0, -math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        length = 1.36
        center_radius = 1.04
        yaw = angle
        frame.visual(
            Box((length, 0.075, 0.055)),
            origin=Origin(xyz=(center_radius * c, center_radius * s, 0.055), rpy=(0.0, 0.0, yaw)),
            material=dark_aluminum,
            name=f"floor_spoke_{idx}",
        )
        frame.visual(
            Box((length, 0.065, 0.070)),
            origin=Origin(xyz=(center_radius * c, center_radius * s, 2.405), rpy=(0.0, 0.0, yaw)),
            material=dark_aluminum,
            name=f"ceiling_spoke_{idx}",
        )

    for idx, angle in enumerate(
        (
            math.radians(32.0),
            math.radians(148.0),
            math.radians(212.0),
            math.radians(328.0),
        )
    ):
        frame.visual(
            Cylinder(radius=0.045, length=2.36),
            origin=Origin(xyz=(1.84 * math.cos(angle), 1.84 * math.sin(angle), 1.215)),
            material=dark_aluminum,
            name=f"entry_mullion_{idx}",
        )

    rotor = model.part("wing_assembly")
    rotor.visual(
        Cylinder(radius=0.165, length=2.36),
        origin=Origin(xyz=(0.0, 0.0, 1.235)),
        material=brushed_aluminum,
        name="central_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.245, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=brushed_aluminum,
        name="lower_rotor_collar",
    )
    rotor.visual(
        Cylinder(radius=0.245, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.23)),
        material=brushed_aluminum,
        name="upper_rotor_collar",
    )

    # Four broad luggage-friendly wings, each with glass infill and metal edge rails.
    panel_length = 1.46
    panel_inner = 0.11
    panel_center = panel_inner + panel_length / 2.0
    panel_height = 2.04
    panel_z = 1.22
    rail_height = 2.16
    wing_names = (
        ("wing_glass_0", "wing_bottom_rail_0", "wing_top_rail_0", "outer_stile_0", "shaft_clamp_0", "lower_sweep_0"),
        ("wing_glass_1", "wing_bottom_rail_1", "wing_top_rail_1", "outer_stile_1", "shaft_clamp_1", "lower_sweep_1"),
        ("wing_glass_2", "wing_bottom_rail_2", "wing_top_rail_2", "outer_stile_2", "shaft_clamp_2", "lower_sweep_2"),
        ("wing_glass_3", "wing_bottom_rail_3", "wing_top_rail_3", "outer_stile_3", "shaft_clamp_3", "lower_sweep_3"),
    )
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        glass_name, bottom_name, top_name, stile_name, clamp_name, sweep_name = wing_names[idx]
        c = math.cos(angle)
        s = math.sin(angle)
        yaw = angle
        rotor.visual(
            Box((panel_length, 0.052, panel_height)),
            origin=Origin(xyz=(panel_center * c, panel_center * s, panel_z), rpy=(0.0, 0.0, yaw)),
            material=safety_glass,
            name=glass_name,
        )
        rotor.visual(
            Box((panel_length + 0.05, 0.070, 0.055)),
            origin=Origin(xyz=(panel_center * c, panel_center * s, 0.235), rpy=(0.0, 0.0, yaw)),
            material=gasket,
            name=bottom_name,
        )
        rotor.visual(
            Box((panel_length + 0.05, 0.070, 0.055)),
            origin=Origin(xyz=(panel_center * c, panel_center * s, 2.200), rpy=(0.0, 0.0, yaw)),
            material=gasket,
            name=top_name,
        )
        rotor.visual(
            Box((0.060, 0.085, rail_height)),
            origin=Origin(
                xyz=((panel_inner + panel_length) * c, (panel_inner + panel_length) * s, panel_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=brushed_aluminum,
            name=stile_name,
        )
        rotor.visual(
            Box((0.085, 0.085, 1.88)),
            origin=Origin(xyz=(0.17 * c, 0.17 * s, panel_z), rpy=(0.0, 0.0, yaw)),
            material=brushed_aluminum,
            name=clamp_name,
        )
        rotor.visual(
            Box((0.055, 0.095, 0.60)),
            origin=Origin(
                xyz=((panel_inner + panel_length - 0.06) * c, (panel_inner + panel_length - 0.06) * s, 0.64),
                rpy=(0.0, 0.0, yaw),
            ),
            material=rubber,
            name=sweep_name,
        )

    model.articulation(
        "drum_to_wings",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.9),
        motion_properties=MotionProperties(damping=0.15, friction=0.04),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("drum_frame")
    rotor = object_model.get_part("wing_assembly")
    joint = object_model.get_articulation("drum_to_wings")

    ctx.check(
        "wing assembly uses a continuous vertical rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.02,
        name="four-wing rotor fits inside the extra-large drum footprint",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="wing_bottom_rail_0",
        negative_elem="floor_track_ring",
        min_gap=0.10,
        name="bottom wing rail clears the low floor track",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="ceiling_header_ring",
        negative_elem="wing_top_rail_0",
        min_gap=0.10,
        name="top wing rail clears the overhead header",
    )

    with ctx.pose({joint: math.pi / 4.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.02,
            name="rotor remains inside the drum while continuously turning",
        )

    return ctx.report()


object_model = build_object_model()
