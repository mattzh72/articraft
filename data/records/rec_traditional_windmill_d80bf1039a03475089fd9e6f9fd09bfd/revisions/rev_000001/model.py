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


def _triangular_prism_roof(
    *,
    half_width: float,
    half_depth: float,
    eave_z: float,
    ridge_z: float,
) -> MeshGeometry:
    """Closed gabled roof mesh in local coordinates."""
    geom = MeshGeometry()
    verts = [
        (-half_width, -half_depth, eave_z),
        (half_width, -half_depth, eave_z),
        (0.0, -half_depth, ridge_z),
        (-half_width, half_depth, eave_z),
        (half_width, half_depth, eave_z),
        (0.0, half_depth, ridge_z),
    ]
    for vx, vy, vz in verts:
        geom.add_vertex(vx, vy, vz)
    # Front and rear triangular ends.
    geom.add_face(0, 1, 2)
    geom.add_face(3, 5, 4)
    # Roof slopes and underside, split into triangles.
    geom.add_face(0, 2, 5)
    geom.add_face(0, 5, 3)
    geom.add_face(1, 4, 5)
    geom.add_face(1, 5, 2)
    geom.add_face(0, 3, 4)
    geom.add_face(0, 4, 1)
    return geom


def _add_blade_members(part, blade_index: int, angle: float, material: Material) -> None:
    """Add one ladder-like sail blade in the hub local XZ plane."""
    u = (math.cos(angle), 0.0, math.sin(angle))
    v = (-math.sin(angle), 0.0, math.cos(angle))
    pitch = -angle

    def xyz(radial: float, tangential: float, y: float = -0.23) -> tuple[float, float, float]:
        return (
            u[0] * radial + v[0] * tangential,
            y,
            u[2] * radial + v[2] * tangential,
        )

    # Strong central sail arm from the hub to the tip.
    main_length = 3.05
    part.visual(
        Box((main_length, 0.070, 0.085)),
        origin=Origin(xyz=xyz(main_length * 0.5, 0.0), rpy=(0.0, pitch, 0.0)),
        material=material,
        name=f"blade_{blade_index}_spar",
    )

    # Parallel rails and rungs make the blade read as traditional open lattice.
    rail_start = 0.52
    rail_length = 2.45
    rail_offset = 0.245
    for side, offset in enumerate((-rail_offset, rail_offset)):
        part.visual(
            Box((rail_length, 0.052, 0.045)),
            origin=Origin(
                xyz=xyz(rail_start + rail_length * 0.5, offset),
                rpy=(0.0, pitch, 0.0),
            ),
            material=material,
            name=f"blade_{blade_index}_rail_{side}",
        )

    for rung, radial in enumerate((0.62, 1.02, 1.42, 1.82, 2.22, 2.62, 2.94)):
        # Slightly longer outer rungs suggest the widening cloth frame of a real sail.
        width = 0.42 + 0.08 * (radial - rail_start) / rail_length
        part.visual(
            Box((0.070, 0.055, width)),
            origin=Origin(xyz=xyz(radial, 0.0), rpy=(0.0, pitch, 0.0)),
            material=material,
            name=f"blade_{blade_index}_rung_{rung}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("warm_stone", rgba=(0.63, 0.58, 0.49, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.34, 0.32, 0.29, 1.0))
    timber = model.material("weathered_timber", rgba=(0.43, 0.28, 0.16, 1.0))
    shingle = model.material("dark_shingles", rgba=(0.12, 0.11, 0.10, 1.0))
    sail_wood = model.material("pale_sail_wood", rgba=(0.83, 0.76, 0.62, 1.0))
    iron = model.material("blackened_iron", rgba=(0.06, 0.065, 0.07, 1.0))
    door_mat = model.material("painted_door", rgba=(0.16, 0.11, 0.07, 1.0))
    glass = model.material("dark_glass", rgba=(0.08, 0.12, 0.16, 1.0))

    tower = model.part("tower")
    tower_shell = LatheGeometry(
        [
            (1.62, 0.00),
            (1.54, 0.22),
            (1.34, 2.00),
            (1.16, 4.20),
            (1.04, 5.72),
            (1.10, 5.92),
            (1.10, 6.00),
            (0.0, 6.00),
            (0.0, 0.00),
        ],
        segments=56,
        closed=True,
    )
    tower.visual(
        mesh_from_geometry(tower_shell, "tapered_stone_tower"),
        material=stone,
        name="stone_tower",
    )
    tower.visual(
        Box((0.74, 0.075, 1.32)),
        origin=Origin(xyz=(0.0, -1.56, 0.78)),
        material=door_mat,
        name="arched_door",
    )
    tower.visual(
        Box((0.42, 0.060, 0.55)),
        origin=Origin(xyz=(0.0, -1.18, 3.18)),
        material=glass,
        name="front_window",
    )
    tower.visual(
        Cylinder(radius=1.13, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 5.92)),
        material=dark_stone,
        name="stone_curb",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=1.13, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_stone,
        name="cap_base_ring",
    )
    cap.visual(
        Box((2.45, 1.72, 0.78)),
        origin=Origin(xyz=(0.0, -0.15, 0.49)),
        material=timber,
        name="cap_body",
    )
    cap.visual(
        mesh_from_geometry(
            _triangular_prism_roof(
                half_width=1.36,
                half_depth=1.05,
                eave_z=0.82,
                ridge_z=1.45,
            ),
            "shingled_cap_roof",
        ),
        material=shingle,
        name="cap_roof",
    )
    cap.visual(
        Cylinder(radius=0.22, length=0.50),
        origin=Origin(xyz=(0.0, -1.14, 0.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="shaft_bearing",
    )
    cap.visual(
        Box((0.42, 0.95, 0.24)),
        origin=Origin(xyz=(0.0, 0.98, 0.45)),
        material=timber,
        name="tail_beam",
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(0.0, -0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=timber,
        name="hub_disk",
    )
    sail_hub.visual(
        Cylinder(radius=0.105, length=0.42),
        origin=Origin(xyz=(0.0, 0.09, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle_stub",
    )
    sail_hub.visual(
        Cylinder(radius=0.16, length=0.07),
        origin=Origin(xyz=(0.0, -0.245, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_cap",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_blade_members(sail_hub, i, angle, sail_wood)

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 6.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(0.0, -1.45, 0.70)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    tower_to_cap = object_model.get_articulation("tower_to_cap")
    cap_to_sail_hub = object_model.get_articulation("cap_to_sail_hub")

    ctx.allow_overlap(
        cap,
        sail_hub,
        elem_a="shaft_bearing",
        elem_b="axle_stub",
        reason=(
            "The windshaft is intentionally captured inside the cap's bearing "
            "sleeve while the sail hub rotates."
        ),
    )
    ctx.expect_within(
        sail_hub,
        cap,
        axes="xz",
        inner_elem="axle_stub",
        outer_elem="shaft_bearing",
        margin=0.01,
        name="windshaft is centered in the bearing sleeve",
    )
    ctx.expect_overlap(
        sail_hub,
        cap,
        axes="y",
        elem_a="axle_stub",
        elem_b="shaft_bearing",
        min_overlap=0.12,
        name="windshaft remains inserted in the bearing sleeve",
    )

    ctx.expect_contact(
        tower,
        cap,
        elem_a="stone_curb",
        elem_b="cap_base_ring",
        contact_tol=0.003,
        name="rotating cap rests on the tower curb",
    )
    ctx.expect_gap(
        cap,
        sail_hub,
        axis="y",
        positive_elem="shaft_bearing",
        negative_elem="hub_disk",
        min_gap=0.035,
        max_gap=0.09,
        name="small hub disk sits in front of the broad bearing support",
    )

    rest_cap_pos = ctx.part_world_position(cap)
    rest_hub_pos = ctx.part_world_position(sail_hub)
    with ctx.pose({tower_to_cap: math.pi / 2.0}):
        yawed_cap_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_to_sail_hub: math.pi / 2.0}):
        spun_hub_pos = ctx.part_world_position(sail_hub)
        ctx.expect_within(
            sail_hub,
            cap,
            axes="xz",
            inner_elem="axle_stub",
            outer_elem="shaft_bearing",
            margin=0.01,
            name="windshaft stays centered after yaw and spin",
        )

    ctx.check(
        "cap yaws about fixed tower axis",
        rest_cap_pos is not None
        and yawed_cap_pos is not None
        and abs(rest_cap_pos[0] - yawed_cap_pos[0]) < 1e-6
        and abs(rest_cap_pos[1] - yawed_cap_pos[1]) < 1e-6,
        details=f"rest={rest_cap_pos}, yawed={yawed_cap_pos}",
    )
    ctx.check(
        "sail hub stays on shaft while spinning",
        rest_hub_pos is not None
        and spun_hub_pos is not None
        and abs(rest_hub_pos[2] - spun_hub_pos[2]) < 1e-6,
        details=f"rest={rest_hub_pos}, spun={spun_hub_pos}",
    )

    return ctx.report()


object_model = build_object_model()
