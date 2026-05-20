from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _arched_lid_shell(
    *,
    x_rear: float,
    x_front: float,
    width: float,
    arch_height: float,
    thickness: float,
    segments: int = 28,
) -> MeshGeometry:
    """Single-piece vaulted wooden lid shell, open on the underside."""
    geom = MeshGeometry()
    half_span = (x_front - x_rear) * 0.5
    center_x = (x_front + x_rear) * 0.5
    y0 = -width * 0.5
    y1 = width * 0.5

    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for i in range(segments + 1):
        u = -1.0 + 2.0 * i / segments
        x = center_x + half_span * u
        # Semi-elliptical crown: low at both lips, tallest at the center.
        z = arch_height * math.sqrt(max(0.0, 1.0 - u * u))
        outer.append((x, z))

        inner_u = max(-0.98, min(0.98, u))
        ix = center_x + (half_span - thickness * 0.95) * inner_u
        iz = max(thickness * 0.65, z - thickness)
        inner.append((ix, iz))

    # Outer and inner arc vertices at the two extrusion sides.
    outer_lo: list[int] = []
    outer_hi: list[int] = []
    inner_lo: list[int] = []
    inner_hi: list[int] = []
    for x, z in outer:
        outer_lo.append(geom.add_vertex(x, y0, z))
        outer_hi.append(geom.add_vertex(x, y1, z))
    for x, z in inner:
        inner_lo.append(geom.add_vertex(x, y0, z))
        inner_hi.append(geom.add_vertex(x, y1, z))

    for i in range(segments):
        # Exterior curved wood surface.
        _add_quad(geom, outer_lo[i], outer_lo[i + 1], outer_hi[i + 1], outer_hi[i])
        # Concave underside visible when the lid is open.
        _add_quad(geom, inner_hi[i], inner_hi[i + 1], inner_lo[i + 1], inner_lo[i])
        # End cap faces at each side of the chest width.
        _add_quad(geom, outer_lo[i], inner_lo[i], inner_lo[i + 1], outer_lo[i + 1])
        _add_quad(geom, outer_hi[i + 1], inner_hi[i + 1], inner_hi[i], outer_hi[i])

    # Front and rear lip thickness faces.
    _add_quad(geom, outer_lo[0], outer_hi[0], inner_hi[0], inner_lo[0])
    _add_quad(geom, outer_lo[-1], inner_lo[-1], inner_hi[-1], outer_hi[-1])
    return geom


def _arch_band(
    *,
    x_rear: float,
    x_front: float,
    y_center: float,
    band_width: float,
    arch_height: float,
    lift: float = 0.004,
    segments: int = 28,
) -> MeshGeometry:
    """Thin iron strap following the arched lid crown."""
    geom = MeshGeometry()
    half_span = (x_front - x_rear) * 0.5
    center_x = (x_front + x_rear) * 0.5
    y0 = y_center - band_width * 0.5
    y1 = y_center + band_width * 0.5
    lo: list[int] = []
    hi: list[int] = []
    for i in range(segments + 1):
        u = -1.0 + 2.0 * i / segments
        x = center_x + half_span * u
        z = arch_height * math.sqrt(max(0.0, 1.0 - u * u)) + lift
        lo.append(geom.add_vertex(x, y0, z))
        hi.append(geom.add_vertex(x, y1, z))
    for i in range(segments):
        _add_quad(geom, lo[i], lo[i + 1], hi[i + 1], hi[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_treasure_chest")

    wood = model.material("warm_wood", color=(0.55, 0.28, 0.10, 1.0))
    dark_wood = model.material("dark_wood_grooves", color=(0.20, 0.10, 0.035, 1.0))
    iron = model.material("darkened_iron", color=(0.08, 0.075, 0.07, 1.0))
    brass = model.material("aged_brass", color=(0.82, 0.58, 0.22, 1.0))
    shadow = model.material("shadowed_interior", color=(0.025, 0.018, 0.012, 1.0))

    width = 0.64
    depth = 0.36
    base_height = 0.25
    wall = 0.025
    hinge_x = -depth * 0.5 - 0.015
    x_rear = 0.015
    x_front = x_rear + depth
    arch_height = 0.16

    base = model.part("base")
    base.visual(
        Box((depth, width, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=wood,
        name="bottom",
    )
    base.visual(
        Box((wall, width, base_height)),
        origin=Origin(xyz=(depth * 0.5 - wall * 0.5, 0.0, base_height * 0.5)),
        material=wood,
        name="front_wall",
    )
    base.visual(
        Box((wall, width, base_height)),
        origin=Origin(xyz=(-depth * 0.5 + wall * 0.5, 0.0, base_height * 0.5)),
        material=wood,
        name="rear_wall",
    )
    for y, name in ((width * 0.5 - wall * 0.5, "side_wall_0"), (-width * 0.5 + wall * 0.5, "side_wall_1")):
        base.visual(
            Box((depth, wall, base_height)),
            origin=Origin(xyz=(0.0, y, base_height * 0.5)),
            material=wood,
            name=name,
        )
    base.visual(
        Box((depth - 2.0 * wall, width - 2.0 * wall, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=shadow,
        name="inner_shadow",
    )

    # Front planks and iron reinforcing straps.
    for y in (-0.18, 0.0, 0.18):
        base.visual(
            Box((0.003, 0.004, base_height * 0.74)),
            origin=Origin(xyz=(depth * 0.5 + 0.0015, y, base_height * 0.50)),
            material=dark_wood,
            name=f"front_plank_groove_{len(base.visuals)}",
        )
    for y, name in ((-0.22, "front_strap_0"), (0.22, "front_strap_1")):
        base.visual(
            Box((0.006, 0.030, base_height)),
            origin=Origin(xyz=(depth * 0.5 + 0.003, y, base_height * 0.5)),
            material=iron,
            name=name,
        )
    base.visual(
        Box((0.006, width, 0.020)),
        origin=Origin(xyz=(depth * 0.5 + 0.003, 0.0, 0.065)),
        material=iron,
        name="front_lower_band",
    )

    # Latch receiver on the fixed front wall.
    base.visual(
        Box((0.004, 0.13, 0.085)),
        origin=Origin(xyz=(depth * 0.5 + 0.002, 0.0, 0.145)),
        material=brass,
        name="latch_plate",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(depth * 0.5 + 0.006, 0.0, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="latch_ring",
    )

    # Fixed half of two exposed barrel hinges.
    for y0, suffix in ((-0.19, "0"), (0.19, "1")):
        base.visual(
            Box((0.004, 0.110, 0.088)),
            origin=Origin(xyz=(hinge_x + 0.013, y0, base_height - 0.048)),
            material=iron,
            name=f"hinge_leaf_{suffix}",
        )
        for dy, knuckle_name in ((-0.037, "lower"), (0.037, "upper")):
            base.visual(
                Cylinder(radius=0.011, length=0.024),
                origin=Origin(
                    xyz=(hinge_x, y0 + dy, base_height),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=iron,
                name=f"hinge_{suffix}_{knuckle_name}_knuckle",
            )
            base.visual(
                Box((0.012, 0.020, 0.012)),
                origin=Origin(xyz=(hinge_x + 0.006, y0 + dy, base_height - 0.006)),
                material=iron,
                name=f"hinge_{suffix}_{knuckle_name}_web",
            )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            _arched_lid_shell(
                x_rear=x_rear,
                x_front=x_front,
                width=width,
                arch_height=arch_height,
                thickness=0.026,
            ),
            "arched_lid_shell",
        ),
        material=wood,
        name="lid_shell",
    )
    lid.visual(
        Box((0.035, width, 0.060)),
        origin=Origin(xyz=(0.035, 0.0, 0.030)),
        material=wood,
        name="rear_rail",
    )
    lid.visual(
        Box((0.040, width, 0.050)),
        origin=Origin(xyz=(x_front - 0.020, 0.0, 0.025)),
        material=wood,
        name="front_rail",
    )

    for y0, suffix in ((-0.19, "0"), (0.19, "1")):
        lid.visual(
            Cylinder(radius=0.0105, length=0.036),
            origin=Origin(xyz=(0.0, y0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"hinge_{suffix}_middle_knuckle",
        )
        lid.visual(
            Box((0.006, 0.038, 0.070)),
            origin=Origin(xyz=(0.014, y0, 0.035)),
            material=iron,
            name=f"hinge_leaf_{suffix}",
        )
        lid.visual(
            Box((0.010, 0.026, 0.014)),
            origin=Origin(xyz=(0.009, y0, 0.004)),
            material=iron,
            name=f"hinge_web_{suffix}",
        )
        lid.visual(
            mesh_from_geometry(
                _arch_band(
                    x_rear=x_rear,
                    x_front=x_front,
                    y_center=y0,
                    band_width=0.028,
                    arch_height=arch_height,
                ),
                f"arched_iron_band_{suffix}",
            ),
            material=iron,
            name=f"arched_band_{suffix}",
        )

    # Moving hasp hardware mounted on the lid; it hangs just proud of the
    # fixed receiver plate when the chest is closed.
    lid.visual(
        Box((0.014, 0.120, 0.018)),
        origin=Origin(xyz=(x_front + 0.007, 0.0, 0.022)),
        material=brass,
        name="hasp_hinge_bar",
    )
    lid.visual(
        Box((0.010, 0.082, 0.130)),
        origin=Origin(xyz=(x_front + 0.017, 0.0, -0.045)),
        material=brass,
        name="hasp_plate",
    )
    lid.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(x_front + 0.023, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hasp_keyhole_ring",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, base_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="front_wall",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed lid rests on front rim",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="rear_wall",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed lid rests on rear rim",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="front_wall",
        min_overlap=0.02,
        name="closed lid spans chest footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.expect_origin_gap(lid, base, axis="z", min_gap=-0.001, name="hinged lid remains attached while open")

    ctx.check(
        "lid opens upward on rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.08,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    ctx.check(
        "visible hinge and latch hardware",
        all(
            lid.get_visual(name) is not None
            for name in ("hinge_0_middle_knuckle", "hinge_1_middle_knuckle", "hasp_plate")
        )
        and base.get_visual("latch_plate") is not None,
        details="expected metal hinge knuckles, hasp, and receiver plate",
    )

    return ctx.report()


object_model = build_object_model()
