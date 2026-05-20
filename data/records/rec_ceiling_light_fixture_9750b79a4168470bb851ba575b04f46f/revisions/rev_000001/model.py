from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BRASS = "aged_brass"
BRONZE = "dark_bronze"
GLASS = "warm_frosted_glass"


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _sunburst_body() -> MeshGeometry:
    """A single connected extrusion for the round plate plus twelve ray arms."""
    ray_count = 12
    disk_radius = 0.300
    ray_tip_radius = 0.720
    base_half_angle = math.radians(5.6)
    tip_half_angle = math.radians(1.8)

    profile: list[tuple[float, float]] = []
    for i in range(ray_count):
        center = 2.0 * math.pi * i / ray_count
        next_center = 2.0 * math.pi * (i + 1) / ray_count

        profile.append(_polar(disk_radius, center - base_half_angle))
        profile.append(_polar(ray_tip_radius, center - tip_half_angle))
        profile.append(_polar(ray_tip_radius, center + tip_half_angle))
        profile.append(_polar(disk_radius, center + base_half_angle))

        # Preserve a visibly round backplate edge between the fixed ray roots.
        start = center + base_half_angle
        end = next_center - base_half_angle
        if end <= start:
            end += 2.0 * math.pi
        for step in range(1, 5):
            a = start + (end - start) * step / 5.0
            profile.append(_polar(disk_radius, a))

    return ExtrudeGeometry.from_z0(profile, 0.024, cap=True, closed=True)


def _dome_glass() -> MeshGeometry:
    """A shallow fluted diffuser dome with twelve art-deco facets."""
    radial_segments = 64
    rings = 10
    base_radius = 0.155
    height = 0.092
    base_z = 0.007
    flute_depth = 0.018

    geom = MeshGeometry()
    ring_indices: list[list[int]] = []
    for r in range(rings + 1):
        fraction = r / rings
        ring: list[int] = []
        if r == 0:
            ring = [geom.add_vertex(0.0, 0.0, base_z + height) for _ in range(radial_segments)]
        else:
            dome_z = base_z + height * math.sqrt(max(0.0, 1.0 - fraction * fraction))
            for s in range(radial_segments):
                theta = 2.0 * math.pi * s / radial_segments
                # Subtle lobing becomes strongest near the rim, where pressed glass
                # fixtures often show flutes.
                lobe = 1.0 + flute_depth * math.cos(12.0 * theta) * (fraction**1.8)
                radius = base_radius * fraction * lobe
                ring.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), dome_z))
        ring_indices.append(ring)

    for r in range(rings):
        current = ring_indices[r]
        outer = ring_indices[r + 1]
        for s in range(radial_segments):
            n = (s + 1) % radial_segments
            if r == 0:
                geom.add_face(current[s], outer[s], outer[n])
            else:
                geom.add_face(current[s], outer[s], outer[n])
                geom.add_face(current[s], outer[n], current[n])

    base_center = geom.add_vertex(0.0, 0.0, base_z)
    base_ring = ring_indices[-1]
    for s in range(radial_segments):
        geom.add_face(base_center, base_ring[(s + 1) % radial_segments], base_ring[s])

    return geom


def _orientation_rib() -> MeshGeometry:
    """An off-center raised brass rib so dome rotation is visually legible."""
    base_radius = 0.155
    height = 0.092
    base_z = 0.007
    half_width = 0.006
    bottom_offset = -0.001
    top_offset = 0.004
    samples = 9

    geom = MeshGeometry()
    lower_a: list[int] = []
    lower_b: list[int] = []
    upper_a: list[int] = []
    upper_b: list[int] = []
    for i in range(samples):
        fraction = 0.20 + 0.68 * i / (samples - 1)
        x = base_radius * fraction
        z_surface = base_z + height * math.sqrt(max(0.0, 1.0 - fraction * fraction))
        lower_a.append(geom.add_vertex(x, -half_width, z_surface + bottom_offset))
        lower_b.append(geom.add_vertex(x, half_width, z_surface + bottom_offset))
        upper_a.append(geom.add_vertex(x, -half_width, z_surface + top_offset))
        upper_b.append(geom.add_vertex(x, half_width, z_surface + top_offset))

    for i in range(samples - 1):
        # top face
        geom.add_face(upper_a[i], upper_a[i + 1], upper_b[i + 1])
        geom.add_face(upper_a[i], upper_b[i + 1], upper_b[i])
        # sides
        geom.add_face(lower_a[i], lower_a[i + 1], upper_a[i + 1])
        geom.add_face(lower_a[i], upper_a[i + 1], upper_a[i])
        geom.add_face(lower_b[i], upper_b[i], upper_b[i + 1])
        geom.add_face(lower_b[i], upper_b[i + 1], lower_b[i + 1])

    # Close the two ends.
    geom.add_face(lower_a[0], upper_a[0], upper_b[0])
    geom.add_face(lower_a[0], upper_b[0], lower_b[0])
    geom.add_face(lower_a[-1], lower_b[-1], upper_b[-1])
    geom.add_face(lower_a[-1], upper_b[-1], upper_a[-1])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="art_deco_sun_ray_ceiling_fixture")
    model.material(BRASS, rgba=(0.86, 0.63, 0.27, 1.0))
    model.material(BRONZE, rgba=(0.28, 0.17, 0.08, 1.0))
    model.material(GLASS, rgba=(1.0, 0.88, 0.62, 0.58))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_geometry(_sunburst_body(), "sunburst_backplate"),
        material=BRASS,
        name="sunburst_body",
    )
    backplate.visual(
        Cylinder(radius=0.320, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=BRASS,
        name="round_backplate",
    )
    backplate.visual(
        Cylinder(radius=0.168, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=BRASS,
        name="center_bearing_pad",
    )
    backplate.visual(
        mesh_from_geometry(TorusGeometry(radius=0.286, tube=0.005, radial_segments=18, tubular_segments=72), "outer_raised_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=BRONZE,
        name="outer_raised_ring",
    )
    backplate.visual(
        mesh_from_geometry(TorusGeometry(radius=0.205, tube=0.0035, radial_segments=14, tubular_segments=72), "inner_raised_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=BRONZE,
        name="inner_raised_ring",
    )

    for i in range(12):
        angle = 2.0 * math.pi * i / 12
        backplate.visual(
            Cylinder(radius=0.006, length=0.355),
            origin=Origin(
                xyz=(0.492 * math.cos(angle), 0.492 * math.sin(angle), 0.029),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=BRONZE,
            name=f"ray_inlay_{i}",
        )

    dome = model.part("dome")
    dome.visual(
        Cylinder(radius=0.170, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=BRASS,
        name="dome_flange",
    )
    dome.visual(
        mesh_from_geometry(TorusGeometry(radius=0.155, tube=0.006, radial_segments=18, tubular_segments=72), "dome_bezel"),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=BRASS,
        name="dome_bezel",
    )
    dome.visual(
        mesh_from_geometry(_dome_glass(), "fluted_glass_dome"),
        material=GLASS,
        name="fluted_glass",
    )
    dome.visual(
        mesh_from_geometry(_orientation_rib(), "orientation_rib"),
        material=BRASS,
        name="orientation_rib",
    )
    dome.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=BRASS,
        name="top_finial",
    )

    model.articulation(
        "backplate_to_dome",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=dome,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    dome = object_model.get_part("dome")
    joint = object_model.get_articulation("backplate_to_dome")

    ctx.check(
        "only dome is articulated",
        len(object_model.parts) == 2 and len(object_model.articulations) == 1,
        details="The twelve ray arms are modeled as fixed geometry on the backplate.",
    )
    ctx.check(
        "twelve ray inlays mark fixed arms",
        len([v for v in backplate.visuals if v.name and v.name.startswith("ray_inlay_")]) == 12,
        details="Expected one visible inlay on each of the twelve fixed ray arms.",
    )
    ctx.expect_origin_distance(
        backplate,
        dome,
        axes="xy",
        max_dist=0.001,
        name="dome joint is centered on the backplate",
    )
    ctx.expect_gap(
        dome,
        backplate,
        axis="z",
        positive_elem="dome_flange",
        negative_elem="center_bearing_pad",
        max_gap=0.001,
        max_penetration=0.00001,
        name="dome flange seats on the bearing pad",
    )

    at_zero = ctx.part_element_world_aabb(dome, elem="orientation_rib")
    with ctx.pose({joint: math.pi / 2.0}):
        at_quarter = ctx.part_element_world_aabb(dome, elem="orientation_rib")

    def _center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    zero_xy = _center_xy(at_zero)
    quarter_xy = _center_xy(at_quarter)
    ctx.check(
        "dome orientation rib rotates with joint",
        zero_xy is not None
        and quarter_xy is not None
        and zero_xy[0] > 0.060
        and abs(zero_xy[1]) < 0.020
        and quarter_xy[1] > 0.060
        and abs(quarter_xy[0]) < 0.020,
        details=f"rest={zero_xy}, quarter_turn={quarter_xy}",
    )

    return ctx.report()


object_model = build_object_model()
