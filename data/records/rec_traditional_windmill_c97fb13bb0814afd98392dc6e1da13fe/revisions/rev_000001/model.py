from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _octagon_section(rx: float, ry: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (
            rx * math.cos(2.0 * math.pi * i / 8.0 + math.pi / 8.0),
            ry * math.sin(2.0 * math.pi * i / 8.0 + math.pi / 8.0),
            z,
        )
        for i in range(8)
    ]


def _gabled_roof_mesh(length: float, width: float, height: float) -> MeshGeometry:
    """A closed triangular-prism roof, centered on X/Y with its eaves at z=0."""
    lx = length / 2.0
    wy = width / 2.0
    geom = MeshGeometry()
    verts = [
        (-lx, -wy, 0.0),
        (-lx, wy, 0.0),
        (-lx, 0.0, height),
        (lx, -wy, 0.0),
        (lx, wy, 0.0),
        (lx, 0.0, height),
    ]
    for v in verts:
        geom.add_vertex(*v)
    # End gables.
    geom.add_face(0, 2, 1)
    geom.add_face(3, 4, 5)
    # Sloped shingle faces.
    geom.add_face(0, 3, 5)
    geom.add_face(0, 5, 2)
    geom.add_face(1, 2, 5)
    geom.add_face(1, 5, 4)
    # Underside, so the roof reads as a single closed cap piece.
    geom.add_face(0, 1, 4)
    geom.add_face(0, 4, 3)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    whitewash = model.material("warm_whitewash", rgba=(0.78, 0.73, 0.62, 1.0))
    stone = model.material("grey_stone", rgba=(0.42, 0.40, 0.36, 1.0))
    wood = model.material("weathered_wood", rgba=(0.50, 0.30, 0.15, 1.0))
    dark_wood = model.material("dark_oak", rgba=(0.20, 0.12, 0.07, 1.0))
    shingle = model.material("black_shingles", rgba=(0.05, 0.055, 0.06, 1.0))
    glass = model.material("dark_glass", rgba=(0.03, 0.06, 0.09, 1.0))
    iron = model.material("dark_iron", rgba=(0.02, 0.02, 0.018, 1.0))

    tower = model.part("tower")
    tower_body = LoftGeometry(
        [
            _octagon_section(0.55, 0.55, 0.10),
            _octagon_section(0.48, 0.48, 0.90),
            _octagon_section(0.40, 0.40, 1.65),
            _octagon_section(0.34, 0.34, 2.20),
        ],
        cap=True,
        closed=True,
    )
    tower.visual(
        mesh_from_geometry(tower_body, "tower_body"),
        material=whitewash,
        name="tower_body",
    )
    tower.visual(
        Cylinder(radius=0.66, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="stone_plinth",
    )
    tower.visual(
        Cylinder(radius=0.38, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=stone,
        name="tower_curb",
    )
    tower.visual(
        Box((0.030, 0.24, 0.45)),
        origin=Origin(xyz=(0.515, 0.0, 0.34)),
        material=dark_wood,
        name="front_door",
    )
    tower.visual(
        Box((0.055, 0.18, 0.15)),
        origin=Origin(xyz=(0.425, 0.0, 1.13)),
        material=glass,
        name="front_window",
    )
    tower.visual(
        Box((0.035, 0.14, 0.13)),
        origin=Origin(xyz=(0.365, 0.0, 1.74)),
        material=glass,
        name="upper_window",
    )
    tower.visual(
        Box((0.30, 0.42, 0.07)),
        origin=Origin(xyz=(0.68, 0.0, 0.035)),
        material=stone,
        name="front_step",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.40, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_wood,
        name="turntable_ring",
    )
    cap.visual(
        Box((0.78, 0.62, 0.30)),
        origin=Origin(xyz=(0.13, 0.0, 0.22)),
        material=wood,
        name="cap_house",
    )
    roof_mesh = _gabled_roof_mesh(0.88, 0.74, 0.30)
    roof_mesh.translate(0.13, 0.0, 0.37)
    cap.visual(
        mesh_from_geometry(roof_mesh, "cap_roof"),
        material=shingle,
        name="cap_roof",
    )
    cap.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(xyz=(0.52, 0.0, 0.28), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="windshaft",
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.135, length=0.10),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood,
        name="hub_barrel",
    )
    sail_hub.visual(
        Cylinder(radius=0.075, length=0.055),
        origin=Origin(xyz=(0.127, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_bolt",
    )

    blade_length = 0.92
    blade_root = 0.10
    blade_x = 0.105
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radial_y = -math.sin(angle)
        radial_z = math.cos(angle)
        tangent_y = math.cos(angle)
        tangent_z = math.sin(angle)
        center_r = blade_root + blade_length / 2.0
        sail_hub.visual(
            Box((0.045, 0.040, blade_length)),
            origin=Origin(
                xyz=(blade_x, radial_y * center_r, radial_z * center_r),
                rpy=(angle, 0.0, 0.0),
            ),
            material=wood,
            name=f"blade_{i}_spar",
        )
        for j, (dist, width) in enumerate(((0.28, 0.18), (0.45, 0.24), (0.62, 0.30), (0.80, 0.36), (0.97, 0.40))):
            sail_hub.visual(
                Box((0.034, width, 0.024)),
                origin=Origin(
                    xyz=(blade_x, radial_y * dist, radial_z * dist),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=wood,
                name=f"blade_{i}_slat_{j}",
            )
        side_center = 0.63
        for side, tangent_offset in enumerate((-0.12, 0.12)):
            sail_hub.visual(
                Box((0.030, 0.020, 0.66)),
                origin=Origin(
                    xyz=(
                        blade_x,
                        radial_y * side_center + tangent_y * tangent_offset,
                        radial_z * side_center + tangent_z * tangent_offset,
                    ),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=wood,
                name=f"blade_{i}_rail_{side}",
            )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(0.64, 0.0, 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    cap_yaw = object_model.get_articulation("tower_to_cap")
    hub_spin = object_model.get_articulation("cap_to_sail_hub")

    ctx.check(
        "cap yaws about the vertical tower axis",
        tuple(cap_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={cap_yaw.axis}",
    )
    ctx.check(
        "sail hub spins about the windshaft axis",
        tuple(hub_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hub_spin.axis}",
    )
    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="cap turntable rests on tower curb",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        min_overlap=0.30,
        name="cap turntable is centered over the tower",
    )
    ctx.expect_gap(
        sail_hub,
        cap,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        name="sail hub seats against the windshaft nose",
    )

    hub_origin = ctx.part_world_position(sail_hub)
    ctx.check(
        "rotating sail hub is offset from the tower centerline",
        hub_origin is not None and hub_origin[0] > 0.55 and hub_origin[2] > 2.35,
        details=f"hub_origin={hub_origin}",
    )

    with ctx.pose({cap_yaw: math.pi / 2.0}):
        yawed_origin = ctx.part_world_position(sail_hub)
    ctx.check(
        "cap yaw carries the offset hub around the tower",
        hub_origin is not None
        and yawed_origin is not None
        and abs(yawed_origin[0]) < 0.02
        and yawed_origin[1] > 0.55,
        details=f"rest={hub_origin}, yawed={yawed_origin}",
    )

    with ctx.pose({hub_spin: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(sail_hub, elem="blade_0_spar")
    with ctx.pose({hub_spin: math.pi / 2.0}):
        spun_aabb = ctx.part_element_world_aabb(sail_hub, elem="blade_0_spar")

    def _aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    rest_z = _aabb_center(rest_aabb, 2)
    spun_y = _aabb_center(spun_aabb, 1)
    ctx.check(
        "hub spin rotates a lattice blade about the shaft",
        rest_z is not None
        and spun_y is not None
        and rest_z > 2.85
        and spun_y < -0.35,
        details=f"rest_aabb={rest_aabb}, spun_aabb={spun_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
