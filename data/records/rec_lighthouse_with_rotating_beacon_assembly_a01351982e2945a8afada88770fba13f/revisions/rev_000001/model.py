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
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _frustum_mesh(
    bottom_radius: float,
    top_radius: float,
    height: float,
    *,
    segments: int = 32,
    z0: float = 0.0,
) -> MeshGeometry:
    """Closed faceted frustum/cone centered on the vertical lighthouse axis."""
    geom = MeshGeometry()
    bottom: list[int] = []
    top: list[int] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        bottom.append(geom.add_vertex(bottom_radius * ca, bottom_radius * sa, z0))
        top.append(geom.add_vertex(top_radius * ca, top_radius * sa, z0 + height))

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])

    bottom_center = geom.add_vertex(0.0, 0.0, z0)
    top_center = geom.add_vertex(0.0, 0.0, z0 + height)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return geom


def _radius_at(z: float, *, z0: float, z1: float, r0: float, r1: float) -> float:
    t = (z - z0) / (z1 - z0)
    return r0 + t * (r1 - r0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_harbor_lighthouse")

    whitewash = model.material("warm_whitewash", rgba=(0.86, 0.82, 0.72, 1.0))
    harbor_red = model.material("weathered_red", rgba=(0.62, 0.08, 0.06, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.23, 0.22, 0.20, 1.0))
    concrete = model.material("salt_stained_concrete", rgba=(0.56, 0.55, 0.50, 1.0))
    black_metal = model.material("blackened_metal", rgba=(0.03, 0.035, 0.035, 1.0))
    roof_green = model.material("aged_copper_roof", rgba=(0.10, 0.34, 0.29, 1.0))
    glass = model.material("pale_lantern_glass", rgba=(0.65, 0.86, 0.95, 0.38))
    brass = model.material("brass", rgba=(0.85, 0.62, 0.22, 1.0))
    lit_glass = model.material("warm_lit_lens", rgba=(1.0, 0.78, 0.25, 0.82))
    dark_glass = model.material("smoky_dark_glass", rgba=(0.02, 0.035, 0.045, 1.0))

    lighthouse = model.part("lighthouse")

    # Squat masonry base and tapered banded tower.
    lighthouse.visual(
        Cylinder(radius=1.90, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=dark_stone,
        name="foundation",
    )
    lighthouse.visual(
        Cylinder(radius=1.62, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=concrete,
        name="foot_ring",
    )

    tower_z0, tower_z1 = 0.50, 3.45
    tower_r0, tower_r1 = 1.36, 1.02
    band_count = 4
    band_height = (tower_z1 - tower_z0) / band_count
    for i in range(band_count):
        z_start = tower_z0 + i * band_height - (0.004 if i > 0 else 0.0)
        z_end = tower_z0 + (i + 1) * band_height + (0.004 if i < band_count - 1 else 0.0)
        r_start = _radius_at(z_start, z0=tower_z0, z1=tower_z1, r0=tower_r0, r1=tower_r1)
        r_end = _radius_at(z_end, z0=tower_z0, z1=tower_z1, r0=tower_r0, r1=tower_r1)
        mat = whitewash if i % 2 == 0 else harbor_red
        lighthouse.visual(
            mesh_from_geometry(
                _frustum_mesh(r_start, r_end, z_end - z_start, segments=48, z0=z_start),
                f"tower_band_{i}",
            ),
            material=mat,
            name=f"tower_band_{i}",
        )

    # Surface-mounted door and small porthole windows on the harbor side.
    lighthouse.visual(
        Box((0.50, 0.055, 0.96)),
        origin=Origin(xyz=(0.0, -1.31, 1.00)),
        material=dark_glass,
        name="front_door",
    )
    lighthouse.visual(
        Box((0.38, 0.045, 0.32)),
        origin=Origin(xyz=(0.0, -1.18, 2.08)),
        material=dark_glass,
        name="front_window",
    )
    lighthouse.visual(
        Box((0.34, 0.045, 0.28)),
        origin=Origin(xyz=(1.09, 0.0, 2.75), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_glass,
        name="side_window",
    )

    # Gallery deck, rails, and the polygonal lantern enclosure.
    lighthouse.visual(
        Cylinder(radius=1.55, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 3.54)),
        material=concrete,
        name="gallery_deck",
    )

    rail_segments = 8
    rail_radius = 1.43
    post_bottom, post_top = 3.62, 4.30
    rail_vertices: list[tuple[float, float]] = []
    for i in range(rail_segments):
        a = 2.0 * math.pi * i / rail_segments + math.pi / rail_segments
        x, y = rail_radius * math.cos(a), rail_radius * math.sin(a)
        rail_vertices.append((x, y))
        lighthouse.visual(
            Cylinder(radius=0.035, length=post_top - post_bottom),
            origin=Origin(xyz=(x, y, (post_bottom + post_top) / 2.0)),
            material=black_metal,
            name=f"rail_post_{i}",
        )

    for level, z in (("lower", 4.00), ("top", 4.28)):
        for i, (x0, y0) in enumerate(rail_vertices):
            x1, y1 = rail_vertices[(i + 1) % rail_segments]
            mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            length = math.hypot(x1 - x0, y1 - y0) + 0.07
            yaw = math.atan2(y1 - y0, x1 - x0)
            lighthouse.visual(
                Box((length, 0.052, 0.052)),
                origin=Origin(xyz=(mx, my, z), rpy=(0.0, 0.0, yaw)),
                material=black_metal,
                name=f"{level}_rail_{i}",
            )

    lantern_segments = 8
    lantern_radius = 1.08
    lighthouse.visual(
        mesh_from_geometry(
            _frustum_mesh(lantern_radius, lantern_radius, 0.22, segments=lantern_segments, z0=3.63),
            "lantern_sill",
        ),
        material=black_metal,
        name="lantern_sill",
    )

    # Static central pedestal visible through the glass.  The rotating head sits on it.
    lighthouse.visual(
        Cylinder(radius=0.22, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 4.025)),
        material=brass,
        name="pedestal",
    )

    panel_height = 1.04
    panel_z = 4.36
    panel_apothem = lantern_radius * math.cos(math.pi / lantern_segments)
    panel_width = 2.0 * lantern_radius * math.sin(math.pi / lantern_segments) - 0.15
    for i in range(lantern_segments):
        theta = 2.0 * math.pi * (i + 0.5) / lantern_segments
        x, y = panel_apothem * math.cos(theta), panel_apothem * math.sin(theta)
        lighthouse.visual(
            Box((panel_width, 0.030, panel_height)),
            origin=Origin(xyz=(x, y, panel_z), rpy=(0.0, 0.0, theta - math.pi / 2.0)),
            material=glass,
            name=f"glass_panel_{i}",
        )

    for i in range(lantern_segments):
        theta = 2.0 * math.pi * i / lantern_segments
        x, y = lantern_radius * math.cos(theta), lantern_radius * math.sin(theta)
        lighthouse.visual(
            Cylinder(radius=0.040, length=1.16),
            origin=Origin(xyz=(x, y, 4.39)),
            material=black_metal,
            name=f"mullion_{i}",
        )

    lighthouse.visual(
        mesh_from_geometry(
            _frustum_mesh(lantern_radius, lantern_radius, 0.12, segments=lantern_segments, z0=4.85),
            "lantern_top_ring",
        ),
        material=black_metal,
        name="lantern_top_ring",
    )
    lighthouse.visual(
        mesh_from_geometry(
            _frustum_mesh(1.20, 0.18, 0.55, segments=lantern_segments, z0=4.97),
            "roof_cap",
        ),
        material=roof_green,
        name="roof_cap",
    )
    lighthouse.visual(
        Cylinder(radius=0.085, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 5.61)),
        material=roof_green,
        name="roof_finial",
    )
    lighthouse.visual(
        Cylinder(radius=0.018, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 5.94)),
        material=black_metal,
        name="lightning_rod",
    )

    # The articulated rotating light head.  Its part frame is exactly on the vertical
    # bearing axis at the top of the fixed central pedestal.
    light_head = model.part("light_head")
    light_head.visual(
        Cylinder(radius=0.20, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=brass,
        name="bearing_disk",
    )
    light_head.visual(
        Cylinder(radius=0.055, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=brass,
        name="central_spindle",
    )
    light_head.visual(
        Cylinder(radius=0.16, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=brass,
        name="center_hub",
    )

    for i in range(4):
        theta = 0.5 * math.pi * i
        nx, ny = math.cos(theta), math.sin(theta)
        yaw_radial = theta
        yaw_panel = theta - math.pi / 2.0
        light_head.visual(
            Box((0.46, 0.060, 0.060)),
            origin=Origin(xyz=(0.23 * nx, 0.23 * ny, 0.36), rpy=(0.0, 0.0, yaw_radial)),
            material=brass,
            name=f"support_arm_{i}",
        )
        light_head.visual(
            Box((0.46, 0.055, 0.30)),
            origin=Origin(xyz=(0.43 * nx, 0.43 * ny, 0.43), rpy=(0.0, 0.0, yaw_panel)),
            material=lit_glass,
            name=f"lens_panel_{i}",
        )
        light_head.visual(
            Box((0.50, 0.035, 0.035)),
            origin=Origin(xyz=(0.43 * nx, 0.43 * ny, 0.597), rpy=(0.0, 0.0, yaw_panel)),
            material=brass,
            name=f"panel_cap_{i}",
        )
        light_head.visual(
            Box((0.50, 0.035, 0.035)),
            origin=Origin(xyz=(0.43 * nx, 0.43 * ny, 0.263), rpy=(0.0, 0.0, yaw_panel)),
            material=brass,
            name=f"panel_foot_{i}",
        )

    model.articulation(
        "light_spin",
        ArticulationType.CONTINUOUS,
        parent=lighthouse,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 4.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lighthouse = object_model.get_part("lighthouse")
    light_head = object_model.get_part("light_head")
    spin = object_model.get_articulation("light_spin")

    ctx.check(
        "light head has continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "polygonal lantern has eight glass panels",
        sum(1 for v in lighthouse.visuals if v.name and v.name.startswith("glass_panel_")) == 8,
    )
    ctx.check(
        "rotating head has four lit lens panels",
        sum(1 for v in light_head.visuals if v.name and v.name.startswith("lens_panel_")) == 4,
    )
    ctx.expect_gap(
        light_head,
        lighthouse,
        axis="z",
        positive_elem="bearing_disk",
        negative_elem="pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating bearing sits on fixed pedestal",
    )
    ctx.expect_within(
        light_head,
        lighthouse,
        axes="xy",
        outer_elem="lantern_sill",
        margin=0.0,
        name="light head fits within lantern footprint",
    )
    ctx.expect_overlap(
        light_head,
        lighthouse,
        axes="z",
        elem_a="lens_panel_0",
        elem_b="glass_panel_0",
        min_overlap=0.25,
        name="light panels sit at lantern-room height",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_within(
            light_head,
            lighthouse,
            axes="xy",
            outer_elem="lantern_sill",
            margin=0.0,
            name="rotated head remains inside lantern footprint",
        )

    return ctx.report()


object_model = build_object_model()
