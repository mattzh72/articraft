from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _cap_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    *,
    base_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + base_z + (0.5 * height))
        for y_pos, z_pos in rounded_rect_profile(
            width,
            height,
            radius,
            corner_segments=8,
        )
    ]


def _build_tower_geometry() -> MeshGeometry:
    foundation = CylinderGeometry(
        radius=1.90,
        height=0.24,
        radial_segments=64,
    ).translate(0.0, 0.0, 0.12)

    tower_body = LatheGeometry(
        [
            (0.0, 0.00),
            (1.62, 0.00),
            (1.58, 0.10),
            (1.48, 0.24),
            (1.34, 0.68),
            (1.20, 1.48),
            (1.08, 2.22),
            (0.98, 2.66),
            (0.92, 2.76),
            (0.0, 2.76),
        ],
        segments=72,
    )

    cap_ring = CylinderGeometry(
        radius=1.02,
        height=0.12,
        radial_segments=56,
    ).translate(0.0, 0.0, 2.82)

    return _merge_geometries(foundation, tower_body, cap_ring)


def _build_cap_geometry() -> MeshGeometry:
    roof = section_loft(
        [
            _cap_section(-0.46, 0.82, 0.42, 0.11, base_z=0.04),
            _cap_section(-0.10, 1.12, 0.68, 0.18, base_z=0.04),
            _cap_section(0.36, 1.34, 0.86, 0.22, base_z=0.02),
            _cap_section(0.88, 1.02, 0.58, 0.16, base_z=0.03),
            _cap_section(1.14, 0.52, 0.28, 0.10, base_z=0.10),
        ]
    )

    base_ring = CylinderGeometry(
        radius=0.98,
        height=0.12,
        radial_segments=56,
    ).translate(0.0, 0.0, 0.06)

    nose_bearing = CylinderGeometry(
        radius=0.14,
        height=1.12,
        radial_segments=40,
    ).rotate_y(math.pi / 2.0).translate(1.08, 0.0, 0.43)

    nose_cheek = BoxGeometry((0.50, 0.42, 0.22)).translate(0.92, 0.0, 0.36)

    front_collar = CylinderGeometry(
        radius=0.18,
        height=0.04,
        radial_segments=40,
    ).rotate_y(math.pi / 2.0).translate(1.64, 0.0, 0.43)

    return _merge_geometries(base_ring, roof, nose_bearing, nose_cheek, front_collar)


def _build_sail_geometry() -> MeshGeometry:
    sail = MeshGeometry()
    sail.merge(BoxGeometry((0.09, 0.14, 2.45)).translate(0.0, 0.0, 1.225))
    sail.merge(BoxGeometry((0.05, 0.92, 0.05)).translate(0.0, 0.0, 0.72))
    sail.merge(BoxGeometry((0.05, 0.92, 0.05)).translate(0.0, 0.0, 2.46))
    sail.merge(BoxGeometry((0.045, 0.045, 1.82)).translate(0.0, -0.44, 1.59))
    sail.merge(BoxGeometry((0.045, 0.045, 1.82)).translate(0.0, 0.44, 1.59))
    sail.merge(BoxGeometry((0.03, 0.03, 1.72)).translate(0.0, -0.22, 1.59))
    sail.merge(BoxGeometry((0.03, 0.03, 1.72)).translate(0.0, 0.22, 1.59))

    for z_pos in (0.95, 1.20, 1.45, 1.70, 1.95, 2.20, 2.38):
        sail.merge(BoxGeometry((0.03, 0.84, 0.024)).translate(0.0, 0.0, z_pos))

    patterned = MeshGeometry()
    for index in range(4):
        patterned.merge(sail.copy().rotate_x(index * (math.pi / 2.0)))
    return patterned


def _build_hub_metal_geometry() -> MeshGeometry:
    drum = CylinderGeometry(
        radius=0.20,
        height=0.30,
        radial_segments=56,
    ).rotate_y(math.pi / 2.0)

    rear_flange = CylinderGeometry(
        radius=0.28,
        height=0.08,
        radial_segments=56,
    ).rotate_y(math.pi / 2.0).translate(-0.12, 0.0, 0.0)

    nose = ConeGeometry(
        radius=0.18,
        height=0.36,
        radial_segments=48,
    ).rotate_y(math.pi / 2.0).translate(0.33, 0.0, 0.0)

    return _merge_geometries(drum, rear_flange, nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.74, 0.70, 0.62, 1.0))
    roof_red = model.material("roof_red", rgba=(0.46, 0.18, 0.12, 1.0))
    iron = model.material("iron", rgba=(0.18, 0.19, 0.21, 1.0))
    sail_wood = model.material("sail_wood", rgba=(0.84, 0.82, 0.74, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_build_tower_geometry(), "tower_body"),
        material=stone,
        name="tower_body",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=1.55, length=2.94),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.47)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_build_cap_geometry(), "cap_shell"),
        material=roof_red,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Box((1.55, 1.45, 0.95)),
        mass=120.0,
        origin=Origin(xyz=(0.22, 0.0, 0.42)),
    )

    hub = model.part("hub")
    hub.visual(
        mesh_from_geometry(_build_hub_metal_geometry(), "hub_metal"),
        material=iron,
        name="hub_metal",
    )
    hub.visual(
        mesh_from_geometry(_build_sail_geometry(), "sail_lattice"),
        material=sail_wood,
        name="sail_lattice",
    )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=1.28, length=5.20),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 2.88)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.8),
    )

    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(1.82, 0.0, 0.43)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    hub = object_model.get_part("hub")
    tower_to_cap = object_model.get_articulation("tower_to_cap")
    cap_to_hub = object_model.get_articulation("cap_to_hub")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        min_gap=0.0,
        max_gap=0.03,
        name="cap sits directly on tower crown",
    )
    ctx.expect_origin_gap(
        hub,
        cap,
        axis="x",
        min_gap=0.9,
        name="hub projects forward from cap",
    )
    ctx.expect_overlap(
        hub,
        cap,
        axes="yz",
        min_overlap=0.18,
        name="hub remains aligned with cap nose",
    )

    rest_hub_pos = ctx.part_world_position(hub)
    rest_hub_aabb = ctx.part_world_aabb(hub)

    with ctx.pose({tower_to_cap: math.pi / 2.0}):
        turned_hub_pos = ctx.part_world_position(hub)

    ctx.check(
        "cap rotation swings hub around tower axis",
        rest_hub_pos is not None
        and turned_hub_pos is not None
        and rest_hub_pos[0] > 1.0
        and abs(turned_hub_pos[1]) > 1.0
        and abs(turned_hub_pos[0]) < 0.2,
        details=f"rest={rest_hub_pos}, turned={turned_hub_pos}",
    )

    with ctx.pose({cap_to_hub: math.pi / 4.0}):
        turned_hub_aabb = ctx.part_world_aabb(hub)

    ctx.check(
        "hub rotation changes blade envelope",
        rest_hub_aabb is not None
        and turned_hub_aabb is not None
        and abs(
            (turned_hub_aabb[1][1] - turned_hub_aabb[0][1])
            - (rest_hub_aabb[1][1] - rest_hub_aabb[0][1])
        )
        > 1.0,
        details=f"rest={rest_hub_aabb}, turned={turned_hub_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
