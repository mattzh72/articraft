from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _add_tube(
    geom: MeshGeometry,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    radial_segments: int = 12,
) -> None:
    geom.merge(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=radial_segments,
            cap_ends=True,
            corner_mode="miter",
        )
    )


def _axle_cylinder(radius: float, length: float) -> MeshGeometry:
    """Cylinder aligned along the local Y axis."""
    return CylinderGeometry(radius, length, radial_segments=24).rotate_x(math.pi / 2.0)


def _wheel_structure() -> MeshGeometry:
    radius = 4.0
    side_y = 0.65
    arm_radius = 3.28
    geom = MeshGeometry()

    # Two narrow side rims keep the observation-wheel form open.
    for y in (-side_y, side_y):
        geom.merge(TorusGeometry(radius, 0.070, radial_segments=24, tubular_segments=96).rotate_x(math.pi / 2.0).translate(0.0, y, 0.0))
        geom.merge(TorusGeometry(radius - 0.34, 0.032, radial_segments=16, tubular_segments=96).rotate_x(math.pi / 2.0).translate(0.0, y, 0.0))

    # Central hub sleeve and the straight radial arms.
    geom.merge(_axle_cylinder(0.30, 1.16))
    spoke_count = 8
    for i in range(spoke_count):
        theta = 2.0 * math.pi * i / spoke_count
        x = arm_radius * math.cos(theta)
        z = arm_radius * math.sin(theta)
        for y in (-side_y, side_y):
            _add_tube(geom, [(0.0, y, 0.0), (x, y, z)], radius=0.055, radial_segments=14)

    # Light polygonal bracing between adjacent arm tips on each side of the wheel.
    for y in (-side_y, side_y):
        for i in range(spoke_count):
            theta_a = 2.0 * math.pi * i / spoke_count
            theta_b = 2.0 * math.pi * (i + 1) / spoke_count
            a = ((radius - 0.34) * math.cos(theta_a), y, (radius - 0.34) * math.sin(theta_a))
            b = ((radius - 0.34) * math.cos(theta_b), y, (radius - 0.34) * math.sin(theta_b))
            _add_tube(geom, [a, b], radius=0.025, radial_segments=10)

    return geom


def _tower_frame() -> MeshGeometry:
    axle_z = 5.0
    geom = MeshGeometry()

    # Four A-frame legs, outside the rotating wheel plane.
    for y in (-1.18, 1.18):
        for x in (-2.85, 2.85):
            _add_tube(geom, [(x, y, 0.22), (0.0, y * 0.78, axle_z)], radius=0.085, radial_segments=16)
        _add_tube(geom, [(-2.85, y, 0.34), (2.85, y, 0.34)], radius=0.070, radial_segments=14)
        _add_tube(geom, [(-1.55, y, 2.65), (1.55, y, 2.65)], radius=0.045, radial_segments=12)

    # Cross ties between the front and rear A-frames.
    for x in (-2.85, 2.85):
        _add_tube(geom, [(x, -1.18, 0.32), (x, 1.18, 0.32)], radius=0.060, radial_segments=12)
    # Outboard axle/bearing caps.  These stop short of the rotating hub, so the
    # wheel reads as mounted on a horizontal axle without hidden interpenetration.
    geom.merge(_axle_cylinder(0.22, 0.36).translate(0.0, -1.03, axle_z))
    geom.merge(_axle_cylinder(0.22, 0.36).translate(0.0, 1.03, axle_z))
    return geom


def _gondola_frame() -> MeshGeometry:
    geom = MeshGeometry()
    half_width = 0.38
    pivot_z = 0.0
    floor_z = -0.94
    rail_z = -0.62
    front_x = 0.34
    rear_x = -0.34

    # Pivot pin and twin triangular hangers.  The pin remains visibly separate
    # from the wheel rim and radial arm bosses.
    geom.merge(_axle_cylinder(0.045, 1.16))
    for y in (-half_width, half_width):
        _add_tube(geom, [(0.0, y, pivot_z), (rear_x, y, floor_z + 0.12)], radius=0.025, radial_segments=10)
        _add_tube(geom, [(0.0, y, pivot_z), (front_x, y, floor_z + 0.12)], radius=0.025, radial_segments=10)
        _add_tube(geom, [(rear_x, y, floor_z + 0.12), (front_x, y, floor_z + 0.12)], radius=0.023, radial_segments=10)
        _add_tube(geom, [(front_x, y, floor_z + 0.12), (front_x, y, rail_z)], radius=0.022, radial_segments=10)
        _add_tube(geom, [(rear_x, y, floor_z + 0.12), (rear_x, y, rail_z - 0.08)], radius=0.022, radial_segments=10)

    # Open front and side guard rails around the bench.
    _add_tube(geom, [(front_x, -half_width, rail_z), (front_x, half_width, rail_z)], radius=0.023, radial_segments=10)
    _add_tube(geom, [(rear_x, -half_width, rail_z - 0.08), (rear_x, half_width, rail_z - 0.08)], radius=0.023, radial_segments=10)
    for y in (-half_width, half_width):
        _add_tube(geom, [(rear_x, y, rail_z - 0.08), (front_x, y, rail_z)], radius=0.022, radial_segments=10)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_gondola_observation_wheel")

    steel = model.material("painted_steel", rgba=(0.82, 0.05, 0.04, 1.0))
    yellow = model.material("fairground_yellow", rgba=(1.0, 0.78, 0.08, 1.0))
    blue = model.material("seat_blue", rgba=(0.06, 0.22, 0.72, 1.0))
    dark = model.material("dark_rubber", rgba=(0.03, 0.035, 0.04, 1.0))
    concrete = model.material("concrete", rgba=(0.52, 0.52, 0.48, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((7.2, 3.4, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="base_slab",
    )
    tower.visual(
        mesh_from_geometry(_tower_frame(), "tower_frame"),
        material=steel,
        name="tower_frame",
    )
    tower.visual(
        mesh_from_geometry(_axle_cylinder(0.16, 1.95).translate(0.0, 0.0, 5.0), "axle_shaft"),
        material=dark,
        name="axle_shaft",
    )
    for y in (-1.03, 1.03):
        tower.visual(
            Box((0.52, 0.20, 0.46)),
            origin=Origin(xyz=(0.0, y, 5.0)),
            material=dark,
            name=f"bearing_{'rear' if y < 0.0 else 'front'}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(_wheel_structure(), "wheel_structure"),
        material=steel,
        name="rim_spokes",
    )
    gondola_radius = 3.28
    for i in range(8):
        theta = 2.0 * math.pi * i / 8
        pivot_x = gondola_radius * math.cos(theta)
        pivot_z = gondola_radius * math.sin(theta)
        for lug_index, y in enumerate((-0.62, 0.62)):
            wheel.visual(
                Box((0.22, 0.08, 0.22)),
                origin=Origin(xyz=(pivot_x, y, pivot_z)),
                material=steel,
                name=f"pivot_lug_{i}_{lug_index}",
            )
    wheel_joint = model.articulation(
        "tower_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 5.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35),
    )

    gondola_mesh = _gondola_frame()
    gondola_count = 8
    for i in range(gondola_count):
        theta = 2.0 * math.pi * i / gondola_count
        pivot_x = gondola_radius * math.cos(theta)
        pivot_z = gondola_radius * math.sin(theta)
        gondola = model.part(f"gondola_{i}")
        gondola.visual(
            mesh_from_geometry(gondola_mesh.copy(), f"gondola_frame_{i}"),
            material=yellow,
            name="hanger_frame",
        )
        gondola.visual(
            Box((0.72, 0.82, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -0.94)),
            material=blue,
            name="seat_floor",
        )
        gondola.visual(
            Box((0.10, 0.82, 0.36)),
            origin=Origin(xyz=(-0.34, 0.0, -0.78)),
            material=blue,
            name="seat_back",
        )
        gondola.visual(
            Box((0.07, 0.82, 0.16)),
            origin=Origin(xyz=(0.18, 0.0, -0.82)),
            material=blue,
            name="bench_cushion",
        )
        model.articulation(
            f"wheel_to_gondola_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=1.0),
            mimic=Mimic(joint=wheel_joint.name, multiplier=-1.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("tower_to_wheel")
    gondola = object_model.get_part("gondola_0")
    gondola_joint = object_model.get_articulation("wheel_to_gondola_0")

    ctx.check(
        "eight open gondolas are present",
        all(object_model.get_part(f"gondola_{i}") is not None for i in range(8)),
        details="Expected gondola_0 through gondola_7.",
    )
    ctx.allow_overlap(
        tower,
        wheel,
        elem_a="axle_shaft",
        elem_b="rim_spokes",
        reason="The fixed axle shaft intentionally passes through the rotating wheel hub/bearing sleeve.",
    )
    ctx.expect_overlap(
        tower,
        wheel,
        axes="y",
        min_overlap=1.0,
        elem_a="axle_shaft",
        elem_b="rim_spokes",
        name="axle shaft spans the rotating hub",
    )
    ctx.expect_overlap(
        tower,
        wheel,
        axes="xz",
        min_overlap=0.25,
        elem_a="axle_shaft",
        elem_b="rim_spokes",
        name="axle shaft is centered inside the hub",
    )

    ctx.expect_origin_gap(wheel, tower, axis="z", min_gap=4.9, max_gap=5.1, name="wheel axle is at fairground height")
    ctx.expect_overlap(wheel, tower, axes="y", min_overlap=0.8, elem_a="rim_spokes", elem_b="tower_frame", name="wheel is centered between tower frames")
    ctx.check(
        "main wheel uses continuous rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(wheel_joint.articulation_type),
    )
    ctx.check(
        "gondola pivot counter-rotates with wheel",
        gondola_joint.mimic is not None
        and gondola_joint.mimic.joint == wheel_joint.name
        and abs(gondola_joint.mimic.multiplier + 1.0) < 1e-9,
        details=str(gondola_joint.mimic),
    )

    def _seat_center_z() -> float | None:
        aabb = ctx.part_element_world_aabb(gondola, elem="seat_floor")
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def _pivot_z() -> float | None:
        pos = ctx.part_world_position(gondola)
        if pos is None:
            return None
        return pos[2]

    rest_pivot_z = _pivot_z()
    rest_seat_z = _seat_center_z()
    with ctx.pose({wheel_joint: 1.1}):
        turned_pivot_z = _pivot_z()
        turned_seat_z = _seat_center_z()

    ctx.check(
        "gondola hangs below its pivot at rest",
        rest_pivot_z is not None and rest_seat_z is not None and rest_seat_z < rest_pivot_z - 0.75,
        details=f"pivot_z={rest_pivot_z}, seat_z={rest_seat_z}",
    )
    ctx.check(
        "gondola remains level while wheel turns",
        turned_pivot_z is not None and turned_seat_z is not None and turned_seat_z < turned_pivot_z - 0.75,
        details=f"pivot_z={turned_pivot_z}, seat_z={turned_seat_z}",
    )

    return ctx.report()


object_model = build_object_model()
