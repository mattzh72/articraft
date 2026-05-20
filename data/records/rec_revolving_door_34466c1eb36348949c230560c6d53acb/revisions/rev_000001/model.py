from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ellipse_ring_slab(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    """Build a closed extruded elliptical annulus for the floor/head rings."""
    geom = MeshGeometry()
    rings = []
    for z in (z_min, z_max):
        outer = []
        inner = []
        for i in range(segments):
            t = 2.0 * math.pi * i / segments
            outer.append(geom.add_vertex(outer_x * math.cos(t), outer_y * math.sin(t), z))
            inner.append(geom.add_vertex(inner_x * math.cos(t), inner_y * math.sin(t), z))
        rings.append((outer, inner))

    (outer_b, inner_b), (outer_t, inner_t) = rings
    for i in range(segments):
        j = (i + 1) % segments
        # Outer vertical wall.
        geom.add_face(outer_b[i], outer_b[j], outer_t[j])
        geom.add_face(outer_b[i], outer_t[j], outer_t[i])
        # Inner vertical wall.
        geom.add_face(inner_b[j], inner_b[i], inner_t[i])
        geom.add_face(inner_b[j], inner_t[i], inner_t[j])
        # Top annulus.
        geom.add_face(outer_t[i], outer_t[j], inner_t[j])
        geom.add_face(outer_t[i], inner_t[j], inner_t[i])
        # Bottom annulus.
        geom.add_face(outer_b[j], outer_b[i], inner_b[i])
        geom.add_face(outer_b[j], inner_b[i], inner_b[j])
    return geom


def _ellipse_arc_wall(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 36,
) -> MeshGeometry:
    """Build one transparent curved wall segment on an elliptical footprint."""
    geom = MeshGeometry()
    outer_b = []
    inner_b = []
    outer_t = []
    inner_t = []
    for i in range(segments + 1):
        u = i / segments
        t = start_angle + (end_angle - start_angle) * u
        c = math.cos(t)
        s = math.sin(t)
        outer_b.append(geom.add_vertex(outer_x * c, outer_y * s, z_min))
        inner_b.append(geom.add_vertex(inner_x * c, inner_y * s, z_min))
        outer_t.append(geom.add_vertex(outer_x * c, outer_y * s, z_max))
        inner_t.append(geom.add_vertex(inner_x * c, inner_y * s, z_max))

    for i in range(segments):
        j = i + 1
        # Outer glass face.
        geom.add_face(outer_b[i], outer_b[j], outer_t[j])
        geom.add_face(outer_b[i], outer_t[j], outer_t[i])
        # Inner glass face.
        geom.add_face(inner_b[j], inner_b[i], inner_t[i])
        geom.add_face(inner_b[j], inner_t[i], inner_t[j])
        # Thin top and bottom edges.
        geom.add_face(outer_t[i], outer_t[j], inner_t[j])
        geom.add_face(outer_t[i], inner_t[j], inner_t[i])
        geom.add_face(outer_b[j], outer_b[i], inner_b[i])
        geom.add_face(outer_b[j], inner_b[i], inner_b[j])

    # Close the two vertical ends so each curved pane is a single solid sheet.
    for idx in (0, segments):
        geom.add_face(outer_b[idx], inner_b[idx], inner_t[idx])
        geom.add_face(outer_b[idx], inner_t[idx], outer_t[idx])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_four_wing_revolving_door")

    dark_aluminum = model.material("dark_aluminum", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    clear_glass = model.material("slightly_blue_glass", rgba=(0.55, 0.78, 0.95, 0.34))
    wing_glass = model.material("clear_door_glass", rgba=(0.70, 0.90, 1.0, 0.42))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    # Dimensions are in meters and sized like a small office vestibule.
    outer_x = 1.36
    outer_y = 1.20
    inner_x = 1.24
    inner_y = 1.08
    bottom_top = 0.12
    glass_bottom = bottom_top
    glass_top = 2.34
    canopy_top = 2.46

    drum = model.part("oval_drum")
    drum.visual(
        mesh_from_geometry(
            _ellipse_ring_slab(outer_x, outer_y, inner_x, inner_y, 0.0, bottom_top),
            "oval_floor_ring",
        ),
        material=dark_aluminum,
        name="floor_ring",
    )
    drum.visual(
        mesh_from_geometry(
            _ellipse_ring_slab(outer_x, outer_y, inner_x, inner_y, glass_top, canopy_top),
            "oval_head_ring",
        ),
        material=dark_aluminum,
        name="head_ring",
    )

    # Two fixed curved glass walls form the oval drum while leaving entry
    # openings at the front and rear.
    arc = math.radians(58.0)
    for side, center_angle in (("side_wall_0", 0.0), ("side_wall_1", math.pi)):
        drum.visual(
            mesh_from_geometry(
                _ellipse_arc_wall(
                    1.325,
                    1.165,
                    1.275,
                    1.115,
                    glass_bottom,
                    glass_top,
                    center_angle - arc,
                    center_angle + arc,
                ),
                side,
            ),
            material=clear_glass,
            name=side,
        )

    # Low floor and overhead spokes tie the central pivot bearings into the oval
    # housing so the hub reads as physically supported, not floating.
    drum.visual(
        Cylinder(radius=0.115, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=brushed_metal,
        name="floor_bearing",
    )
    drum.visual(
        Cylinder(radius=0.105, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.405)),
        material=brushed_metal,
        name="head_bearing",
    )
    for level, z in (("floor", 0.065), ("head", 2.395)):
        height = 0.035 if level == "floor" else 0.045
        drum.visual(
            Box((2.48, 0.060, height)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_aluminum,
            name=f"{level}_spoke_x",
        )
        drum.visual(
            Box((2.16, 0.060, height)),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=dark_aluminum,
            name=f"{level}_spoke_y",
        )

    # Four slim mullions mark the open jambs of the drum.
    for idx, angle in enumerate((-arc, arc, math.pi - arc, math.pi + arc)):
        x = 1.30 * math.cos(angle)
        y = 1.14 * math.sin(angle)
        drum.visual(
            Cylinder(radius=0.026, length=glass_top - glass_bottom),
            origin=Origin(xyz=(x, y, (glass_top + glass_bottom) / 2.0)),
            material=dark_aluminum,
            name=f"jamb_post_{idx}",
        )

    assembly = model.part("wing_assembly")
    assembly.visual(
        Cylinder(radius=0.055, length=2.18),
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
        material=brushed_metal,
        name="hub_post",
    )
    assembly.visual(
        Cylinder(radius=0.085, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=brushed_metal,
        name="lower_hub_cap",
    )
    assembly.visual(
        Cylinder(radius=0.075, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 2.330)),
        material=brushed_metal,
        name="upper_hub_cap",
    )

    wing_length = 0.98
    hub_overlap = 0.025
    radial_center = 0.055 - hub_overlap + wing_length / 2.0
    wing_height = 2.06
    wing_z = 1.22
    wing_thickness = 0.035
    rail_thickness = 0.035
    stile_width = 0.040
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        origin = Origin(
            xyz=(radial_center * c, radial_center * s, wing_z),
            rpy=(0.0, 0.0, angle),
        )
        assembly.visual(
            Box((wing_length, wing_thickness, wing_height)),
            origin=origin,
            material=wing_glass,
            name=f"wing_glass_{idx}",
        )
        assembly.visual(
            Box((wing_length, 0.050, rail_thickness)),
            origin=Origin(
                xyz=(radial_center * c, radial_center * s, 0.195),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"lower_rail_{idx}",
        )
        assembly.visual(
            Box((wing_length, 0.050, rail_thickness)),
            origin=Origin(
                xyz=(radial_center * c, radial_center * s, 2.245),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"upper_rail_{idx}",
        )
        outer_center = 0.055 - hub_overlap + wing_length - stile_width / 2.0
        assembly.visual(
            Box((stile_width, 0.055, wing_height)),
            origin=Origin(
                xyz=(outer_center * c, outer_center * s, wing_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"outer_stile_{idx}",
        )
        inner_center = 0.055 - hub_overlap + stile_width / 2.0
        assembly.visual(
            Box((stile_width, 0.055, wing_height)),
            origin=Origin(
                xyz=(inner_center * c, inner_center * s, wing_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_aluminum,
            name=f"inner_stile_{idx}",
        )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("oval_drum")
    assembly = object_model.get_part("wing_assembly")
    spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "wing assembly has continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        assembly,
        drum,
        axes="xy",
        margin=0.005,
        name="four wings fit within the oval drum footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(assembly, elem="wing_glass_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(assembly, elem="wing_glass_0")
        ctx.expect_within(
            assembly,
            drum,
            axes="xy",
            margin=0.005,
            name="rotated wings remain inside the drum footprint",
        )

    if rest_aabb is not None and turned_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        turned_center_y = (turned_aabb[0][1] + turned_aabb[1][1]) / 2.0
        ctx.check(
            "named wing rotates around hub",
            rest_center_x > 0.35 and turned_center_y > 0.35,
            details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
        )
    else:
        ctx.fail("named wing rotates around hub", "wing_glass_0 AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
