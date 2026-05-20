from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TAU = 2.0 * math.pi


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _smooth_oval_band_geometry(
    *,
    x_radius: float,
    y_radius: float,
    radial_radius: float,
    axial_radius: float,
    path_segments: int = 96,
    tube_segments: int = 20,
) -> MeshGeometry:
    """Elliptical finger band with a smooth oval tube cross-section."""
    mesh = MeshGeometry()
    grid: list[list[int]] = []

    for i in range(path_segments):
        theta = TAU * i / path_segments
        ct = math.cos(theta)
        st = math.sin(theta)

        cx = x_radius * ct
        cy = y_radius * st

        # Outward normal of the ellipse x^2/a^2 + y^2/b^2 = 1.
        nx = ct / max(x_radius, 1e-9)
        ny = st / max(y_radius, 1e-9)
        n_len = math.hypot(nx, ny)
        nx /= n_len
        ny /= n_len

        ring: list[int] = []
        for j in range(tube_segments):
            phi = TAU * j / tube_segments
            cp = math.cos(phi)
            sp = math.sin(phi)
            x = cx + radial_radius * cp * nx
            y = cy + radial_radius * cp * ny
            z = axial_radius * sp
            ring.append(mesh.add_vertex(x, y, z))
        grid.append(ring)

    for i in range(path_segments):
        ni = (i + 1) % path_segments
        for j in range(tube_segments):
            nj = (j + 1) % tube_segments
            _add_quad(mesh, grid[i][j], grid[ni][j], grid[ni][nj], grid[i][nj])

    return mesh


def _spiked_roller_geometry(
    *,
    major_radius: float,
    tube_radial: float,
    tube_axial: float,
    spike_height: float,
    bearing_pad_height: float = 0.00131,
    spike_count: int = 30,
    radial_segments: int = 180,
    tube_segments: int = 28,
) -> MeshGeometry:
    """Continuous corrugated torus: integral rounded teeth on the outer equator."""
    mesh = MeshGeometry()
    grid: list[list[int]] = []

    for i in range(radial_segments):
        theta = TAU * i / radial_segments
        ct = math.cos(theta)
        st = math.sin(theta)
        phase = (theta * spike_count / TAU) % 1.0
        triangular_peak = max(0.0, 1.0 - abs(phase - 0.5) * 2.0)
        peak = triangular_peak * triangular_peak
        pad_phase = max(0.0, (abs(math.cos(theta)) - 0.90) / 0.10)
        bearing_pad = pad_phase * pad_phase

        ring: list[int] = []
        for j in range(tube_segments):
            phi = TAU * j / tube_segments
            cp = math.cos(phi)
            sp = math.sin(phi)

            # Teeth grow only from the outward-facing half of the torus.  The
            # steep taper gives each circumferential peak a spiked acupressure
            # profile while keeping the mesh one continuous roller body.
            outward_mask = max(0.0, cp) ** 3.0
            inward_mask = max(0.0, -cp) ** 5.0
            fine_texture = 0.00012 * max(0.0, cp) * (0.5 + 0.5 * math.cos(6.0 * phi))
            radial_offset = (
                tube_radial * cp
                + spike_height * peak * outward_mask
                + fine_texture
                - bearing_pad_height * bearing_pad * inward_mask
            )

            radius = major_radius + radial_offset
            x = radius * ct
            y = radius * st
            z = tube_axial * sp
            ring.append(mesh.add_vertex(x, y, z))
        grid.append(ring)

    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        for j in range(tube_segments):
            nj = (j + 1) % tube_segments
            _add_quad(mesh, grid[i][j], grid[ni][j], grid[ni][nj], grid[i][nj])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="acupressure_finger_massage_ring")

    band_material = model.material(
        "smooth_satin_steel",
        rgba=(0.78, 0.80, 0.78, 1.0),
    )
    roller_material = model.material(
        "warm_anodized_spikes",
        rgba=(0.95, 0.44, 0.18, 1.0),
    )

    inner_band = model.part("inner_band")
    inner_band.visual(
        mesh_from_geometry(
            _smooth_oval_band_geometry(
                x_radius=0.0100,
                y_radius=0.0100,
                radial_radius=0.00105,
                axial_radius=0.00275,
            ),
            "smooth_oval_inner_band",
        ),
        material=band_material,
        name="smooth_band",
    )

    outer_roller = model.part("outer_roller")
    outer_roller.visual(
        mesh_from_geometry(
            _spiked_roller_geometry(
                major_radius=0.0140,
                tube_radial=0.00165,
                tube_axial=0.00345,
                spike_height=0.00135,
            ),
            "spiked_outer_roller",
        ),
        material=roller_material,
        name="spiked_roller",
    )

    model.articulation(
        "band_to_roller",
        ArticulationType.CONTINUOUS,
        parent=inner_band,
        child=outer_roller,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.003),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    inner_band = object_model.get_part("inner_band")
    outer_roller = object_model.get_part("outer_roller")
    spin = object_model.get_articulation("band_to_roller")

    ctx.allow_overlap(
        inner_band,
        outer_roller,
        elem_a="smooth_band",
        elem_b="spiked_roller",
        reason=(
            "The roller has tiny integral inner bearing pads with slight hidden "
            "interference on the smooth band so the free-spinning ring is visibly "
            "captured rather than floating."
        ),
    )
    ctx.expect_contact(
        inner_band,
        outer_roller,
        elem_a="smooth_band",
        elem_b="spiked_roller",
        contact_tol=0.0002,
        name="bearing pads seat on smooth inner band",
    )
    ctx.check(
        "outer roller has continuous central spin",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={spin.articulation_type}",
    )
    ctx.expect_origin_distance(
        inner_band,
        outer_roller,
        axes="xy",
        max_dist=0.0005,
        name="inner band and roller share central axis",
    )
    ctx.expect_within(
        inner_band,
        outer_roller,
        axes="xyz",
        margin=0.0,
        name="spiked roller fully encircles smooth band envelope",
    )
    ctx.expect_overlap(
        inner_band,
        outer_roller,
        axes="xy",
        min_overlap=0.018,
        name="roller surrounds finger-band footprint",
    )

    band_aabb = ctx.part_world_aabb(inner_band)
    roller_aabb = ctx.part_world_aabb(outer_roller)
    if band_aabb is not None and roller_aabb is not None:
        band_width = band_aabb[1][0] - band_aabb[0][0]
        roller_width = roller_aabb[1][0] - roller_aabb[0][0]
        roller_depth = roller_aabb[1][1] - roller_aabb[0][1]
        ctx.check(
            "wearable ring-sized scale",
            0.028 <= roller_width <= 0.038 and 0.028 <= roller_depth <= 0.038,
            details=f"roller_width={roller_width:.4f}, roller_depth={roller_depth:.4f}",
        )
        ctx.check(
            "outer roller is visibly larger than inner band",
            roller_width > band_width + 0.006,
            details=f"band_width={band_width:.4f}, roller_width={roller_width:.4f}",
        )

    rest_position = ctx.part_world_position(outer_roller)
    with ctx.pose({spin: math.pi / 7.0}):
        spun_position = ctx.part_world_position(outer_roller)
        ctx.expect_origin_distance(
            inner_band,
            outer_roller,
            axes="xy",
            max_dist=0.0005,
            name="roller stays coaxial while spinning",
        )
        ctx.expect_contact(
            inner_band,
            outer_roller,
            elem_a="smooth_band",
            elem_b="spiked_roller",
            contact_tol=0.0002,
            name="bearing pads remain seated during off-index spin",
        )
    ctx.check(
        "continuous spin does not translate roller",
        rest_position is not None
        and spun_position is not None
        and max(abs(rest_position[i] - spun_position[i]) for i in range(3)) < 0.0005,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
