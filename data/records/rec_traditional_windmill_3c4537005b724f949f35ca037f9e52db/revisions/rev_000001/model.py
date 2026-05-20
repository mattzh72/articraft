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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _gable_roof_mesh(
    *,
    length: float,
    width: float,
    eave_z: float,
    ridge_z: float,
    x_center: float = 0.0,
) -> MeshGeometry:
    """Closed triangular-prism roof with the ridge running along local X."""
    x0 = x_center - length / 2.0
    x1 = x_center + length / 2.0
    y0 = -width / 2.0
    y1 = width / 2.0
    geom = MeshGeometry()
    verts = [
        (x0, y0, eave_z),
        (x0, y1, eave_z),
        (x0, 0.0, ridge_z),
        (x1, y0, eave_z),
        (x1, y1, eave_z),
        (x1, 0.0, ridge_z),
    ]
    for v in verts:
        geom.add_vertex(*v)
    for face in [
        (0, 3, 4),
        (0, 4, 1),
        (0, 2, 5),
        (0, 5, 3),
        (1, 4, 5),
        (1, 5, 2),
        (0, 1, 2),
        (3, 5, 4),
    ]:
        geom.add_face(*face)
    return geom


def _add_beam(
    part,
    *,
    name: str,
    x: float,
    p0: tuple[float, float],
    p1: tuple[float, float],
    depth: float,
    thickness: float,
    material: Material,
) -> None:
    """Add one rectangular wooden member between two YZ-plane points."""
    dy = p1[0] - p0[0]
    dz = p1[1] - p0[1]
    length = math.hypot(dy, dz)
    if length <= 1.0e-6:
        return
    phi = math.atan2(dy, dz)
    part.visual(
        Box((depth, thickness, length)),
        origin=Origin(
            xyz=(x, (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0),
            rpy=(-phi, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _blade_point(theta: float, radius: float, half_width: float) -> tuple[float, float]:
    radial_y = math.sin(theta)
    radial_z = math.cos(theta)
    tangent_y = math.cos(theta)
    tangent_z = -math.sin(theta)
    return (
        radial_y * radius + tangent_y * half_width,
        radial_z * radius + tangent_z * half_width,
    )


def _blade_half_width(radius: float) -> float:
    root_r = 0.18
    tip_r = 1.18
    root_w = 0.08
    tip_w = 0.19
    t = max(0.0, min(1.0, (radius - root_r) / (tip_r - root_r)))
    return root_w + t * (tip_w - root_w)


def _add_lattice_blade(
    part,
    *,
    index: int,
    theta: float,
    x: float,
    material: Material,
) -> None:
    """A tapered, open wooden sail built from connected lattice members."""
    depth = 0.035
    thick_main = 0.045
    thick_lattice = 0.026
    root_r = 0.18
    tip_r = 1.18

    _add_beam(
        part,
        name=f"blade_spar_{index}",
        x=x,
        p0=_blade_point(theta, 0.02, 0.0),
        p1=_blade_point(theta, tip_r + 0.02, 0.0),
        depth=depth,
        thickness=thick_main,
        material=material,
    )

    for side, label in [(-1.0, "a"), (1.0, "b")]:
        _add_beam(
            part,
            name=f"blade_edge_{index}_{label}",
            x=x,
            p0=_blade_point(theta, root_r, side * _blade_half_width(root_r)),
            p1=_blade_point(theta, tip_r, side * _blade_half_width(tip_r)),
            depth=depth,
            thickness=thick_lattice,
            material=material,
        )

    for rail_i, radius in enumerate([root_r, 0.46, 0.76, 1.03, tip_r]):
        width = _blade_half_width(radius)
        _add_beam(
            part,
            name=f"blade_rail_{index}_{rail_i}",
            x=x,
            p0=_blade_point(theta, radius, -width),
            p1=_blade_point(theta, radius, width),
            depth=depth,
            thickness=thick_lattice,
            material=material,
        )

    brace_spans = [(root_r, 0.46), (0.46, 0.76), (0.76, 1.03), (1.03, tip_r)]
    for brace_i, (r0, r1) in enumerate(brace_spans):
        w0 = _blade_half_width(r0)
        w1 = _blade_half_width(r1)
        if brace_i % 2 == 0:
            p0 = _blade_point(theta, r0, -w0)
            p1 = _blade_point(theta, r1, w1)
        else:
            p0 = _blade_point(theta, r0, w0)
            p1 = _blade_point(theta, r1, -w1)
        _add_beam(
            part,
            name=f"blade_brace_{index}_{brace_i}",
            x=x,
            p0=p0,
            p1=p1,
            depth=depth * 0.9,
            thickness=0.020,
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = Material("warm_limestone", rgba=(0.74, 0.68, 0.56, 1.0))
    stone_shadow = Material("mortar_shadow", rgba=(0.54, 0.50, 0.43, 1.0))
    dark_wood = Material("weathered_dark_wood", rgba=(0.20, 0.14, 0.09, 1.0))
    sail_wood = Material("pale_sail_wood", rgba=(0.78, 0.60, 0.38, 1.0))
    roof_mat = Material("aged_black_roof", rgba=(0.08, 0.075, 0.065, 1.0))
    iron = Material("dark_iron", rgba=(0.05, 0.055, 0.055, 1.0))
    glass = Material("dark_window_glass", rgba=(0.04, 0.07, 0.09, 1.0))

    tower = model.part("tower")
    tower_height = 5.20
    tower.visual(
        mesh_from_geometry(
            LatheGeometry(
                [
                    (0.0, 0.0),
                    (0.72, 0.0),
                    (0.67, 0.35),
                    (0.55, 3.10),
                    (0.46, tower_height),
                    (0.0, tower_height),
                ],
                segments=64,
                closed=True,
            ),
            "tapered_tower",
        ),
        material=stone,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(0.78, 0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=stone_shadow,
        name="base_plinth",
    )
    for i, z in enumerate([0.85, 1.70, 2.55, 3.40, 4.25]):
        radius = 0.72 - (0.72 - 0.46) * (z / tower_height)
        tower.visual(
            Cylinder(radius + 0.018, 0.035),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stone_shadow,
            name=f"stone_band_{i}",
        )
    tower.visual(
        Box((0.045, 0.38, 0.78)),
        origin=Origin(xyz=(0.695, 0.0, 0.49)),
        material=dark_wood,
        name="front_door",
    )
    for i, (z, width, height) in enumerate([(2.05, 0.20, 0.30), (3.25, 0.18, 0.27)]):
        radius = 0.72 - (0.72 - 0.46) * (z / tower_height)
        tower.visual(
            Box((0.065, width, height)),
            origin=Origin(xyz=(radius - 0.005, 0.0, z)),
            material=glass,
            name=f"front_window_{i}",
        )
        tower.visual(
            Box((0.070, width + 0.10, 0.035)),
            origin=Origin(xyz=(radius - 0.002, 0.0, z + height / 2.0 + 0.035)),
            material=stone_shadow,
            name=f"window_lintel_{i}",
        )

    cap = model.part("cap")
    cap.visual(
        Cylinder(0.50, 0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_wood,
        name="turntable_collar",
    )
    cap.visual(
        Box((0.92, 0.60, 0.38)),
        origin=Origin(xyz=(0.15, 0.0, 0.31)),
        material=dark_wood,
        name="cap_house",
    )
    cap.visual(
        mesh_from_geometry(
            _gable_roof_mesh(
                length=1.08,
                width=0.78,
                eave_z=0.48,
                ridge_z=0.78,
                x_center=0.12,
            ),
            "cap_gable_roof",
        ),
        material=roof_mat,
        name="gable_roof",
    )
    cap.visual(
        Box((0.08, 0.70, 0.10)),
        origin=Origin(xyz=(-0.22, 0.0, 0.47)),
        material=roof_mat,
        name="rear_eave",
    )
    cap.visual(
        Cylinder(0.070, 0.56),
        origin=Origin(xyz=(0.73, 0.0, 0.36), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="wind_shaft",
    )
    cap.visual(
        Box((0.13, 0.24, 0.22)),
        origin=Origin(xyz=(0.50, 0.0, 0.36)),
        material=iron,
        name="front_bearing",
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, tower_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(0.135, 0.14),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_barrel",
    )
    hub.visual(
        Sphere(0.09),
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        material=iron,
        name="nose_bolt",
    )
    for i, theta in enumerate([0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]):
        _add_lattice_blade(hub, index=i, theta=theta, x=0.155, material=sail_wood)

    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(1.01, 0.0, 0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("hub")
    cap_yaw = object_model.get_articulation("tower_to_cap")
    hub_spin = object_model.get_articulation("cap_to_hub")

    ctx.expect_contact(
        tower,
        cap,
        elem_a="tapered_tower",
        elem_b="turntable_collar",
        contact_tol=0.002,
        name="rotating cap is seated on the tower top",
    )
    ctx.expect_contact(
        cap,
        hub,
        elem_a="wind_shaft",
        elem_b="hub_barrel",
        contact_tol=0.003,
        name="hub is carried by the wind shaft",
    )
    ctx.check(
        "tower is much taller than the head assembly",
        ctx.part_world_aabb(tower) is not None
        and ctx.part_world_aabb(cap) is not None
        and (ctx.part_world_aabb(tower)[1][2] - ctx.part_world_aabb(tower)[0][2])
        > 5.0
        * (ctx.part_world_aabb(cap)[1][2] - ctx.part_world_aabb(cap)[0][2]),
        details="The tower should dominate the silhouette with a small rotating head.",
    )

    rest_hub_position = ctx.part_world_position(hub)
    with ctx.pose({cap_yaw: math.pi / 2.0}):
        turned_hub_position = ctx.part_world_position(hub)
    ctx.check(
        "cap yaw carries the hub around the vertical tower axis",
        rest_hub_position is not None
        and turned_hub_position is not None
        and turned_hub_position[1] > rest_hub_position[1] + 0.75,
        details=f"rest={rest_hub_position}, turned={turned_hub_position}",
    )

    rest_blade_aabb = ctx.part_element_world_aabb(hub, elem="blade_spar_0")
    with ctx.pose({hub_spin: 0.45}):
        spun_blade_aabb = ctx.part_element_world_aabb(hub, elem="blade_spar_0")
    ctx.check(
        "sail hub spin rotates individual blade members",
        rest_blade_aabb is not None
        and spun_blade_aabb is not None
        and abs(spun_blade_aabb[0][1] - rest_blade_aabb[0][1]) > 0.05,
        details=f"rest={rest_blade_aabb}, spun={spun_blade_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
