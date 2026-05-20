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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arched_profile(width: float, bottom_z: float, spring_z: float, *, segments: int = 28):
    """Convex arched doorway/profile loop in local XZ, ordered counter-clockwise."""
    radius = width / 2.0
    pts = [(-radius, bottom_z), (radius, bottom_z)]
    for i in range(segments + 1):
        theta = math.pi * i / segments
        pts.append((radius * math.cos(theta), spring_z + radius * math.sin(theta)))
    return pts


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _add_fan_cap(mesh: MeshGeometry, loop: list[int], center: int, *, reverse: bool = False) -> None:
    count = len(loop)
    for i in range(count):
        a = loop[i]
        b = loop[(i + 1) % count]
        if reverse:
            mesh.add_face(center, b, a)
        else:
            mesh.add_face(center, a, b)


def _extruded_arch_slab(
    width: float,
    bottom_z: float,
    spring_z: float,
    *,
    y_min: float,
    y_max: float,
    segments: int = 28,
) -> MeshGeometry:
    """Solid arched plate, with its thickness along local Y."""
    profile = _arched_profile(width, bottom_z, spring_z, segments=segments)
    mesh = MeshGeometry()

    front = [mesh.add_vertex(x, y_min, z) for x, z in profile]
    back = [mesh.add_vertex(x, y_max, z) for x, z in profile]

    for i in range(len(profile)):
        j = (i + 1) % len(profile)
        _add_quad(mesh, front[i], front[j], back[j], back[i])

    cx = sum(p[0] for p in profile) / len(profile)
    cz = sum(p[1] for p in profile) / len(profile)
    front_center = mesh.add_vertex(cx, y_min, cz)
    back_center = mesh.add_vertex(cx, y_max, cz)
    _add_fan_cap(mesh, front, front_center, reverse=True)
    _add_fan_cap(mesh, back, back_center, reverse=False)
    return mesh


def _extruded_arch_ring(
    *,
    outer_width: float,
    outer_bottom_z: float,
    outer_spring_z: float,
    inner_width: float,
    inner_bottom_z: float,
    inner_spring_z: float,
    y_front: float,
    y_back: float,
    rear_cap: bool = False,
    segments: int = 28,
) -> MeshGeometry:
    """Arched masonry/steel ring with a hollow throat along Y."""
    outer = _arched_profile(outer_width, outer_bottom_z, outer_spring_z, segments=segments)
    inner = _arched_profile(inner_width, inner_bottom_z, inner_spring_z, segments=segments)
    mesh = MeshGeometry()

    outer_front = [mesh.add_vertex(x, y_front, z) for x, z in outer]
    inner_front = [mesh.add_vertex(x, y_front, z) for x, z in inner]
    outer_back = [mesh.add_vertex(x, y_back, z) for x, z in outer]
    inner_back = [mesh.add_vertex(x, y_back, z) for x, z in inner]

    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        # Exterior skin, interior throat, and the front/back annular faces.
        _add_quad(mesh, outer_front[i], outer_back[i], outer_back[j], outer_front[j])
        _add_quad(mesh, inner_front[j], inner_back[j], inner_back[i], inner_front[i])
        _add_quad(mesh, outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        _add_quad(mesh, outer_back[j], outer_back[i], inner_back[i], inner_back[j])

    if rear_cap:
        cx = sum(p[0] for p in inner) / len(inner)
        cz = sum(p[1] for p in inner) / len(inner)
        center = mesh.add_vertex(cx, y_back, cz)
        _add_fan_cap(mesh, inner_back, center, reverse=True)

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wood_fired_oven")

    firebrick = Material("warm_firebrick", rgba=(0.56, 0.34, 0.20, 1.0))
    blackened_steel = Material("blackened_steel", rgba=(0.05, 0.052, 0.050, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.33, 0.34, 0.33, 1.0))

    housing = model.part("oven_housing")

    housing.visual(
        mesh_from_geometry(
            _extruded_arch_ring(
                outer_width=1.65,
                outer_bottom_z=0.0,
                outer_spring_z=0.43,
                inner_width=1.08,
                inner_bottom_z=0.18,
                inner_spring_z=0.45,
                y_front=-0.58,
                y_back=0.58,
                rear_cap=True,
            ),
            "masonry_vault",
        ),
        material=firebrick,
        name="masonry_vault",
    )

    housing.visual(
        mesh_from_geometry(
            _extruded_arch_ring(
                outer_width=1.30,
                outer_bottom_z=0.145,
                outer_spring_z=0.45,
                inner_width=1.08,
                inner_bottom_z=0.18,
                inner_spring_z=0.45,
                y_front=-0.660,
                y_back=-0.580,
            ),
            "mouth_surround",
        ),
        material=brushed_steel,
        name="mouth_surround",
    )

    # A blackened rear soot patch makes the open arch read as a hollow fired chamber.
    housing.visual(
        mesh_from_geometry(
            _extruded_arch_slab(1.00, 0.20, 0.46, y_min=-0.003, y_max=0.003, segments=28),
            "soot_back",
        ),
        origin=Origin(xyz=(0.0, 0.581, 0.0)),
        material=blackened_steel,
        name="soot_back",
    )

    # Short flue projecting from the hot crown of the vault.
    housing.visual(
        Cylinder(radius=0.105, length=0.38),
        origin=Origin(xyz=(0.0, -0.12, 1.36)),
        material=blackened_steel,
        name="flue_stack",
    )

    axle_y = -0.672
    axle_z = 0.100
    housing.visual(
        Cylinder(radius=0.020, length=1.22),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_steel,
        name="axle_pin",
    )
    for x in (-0.56, 0.56):
        housing.visual(
            Box((0.085, 0.090, 0.120)),
            origin=Origin(xyz=(x, -0.657, 0.105)),
            material=brushed_steel,
            name=f"axle_boss_{'neg' if x < 0 else 'pos'}",
        )

    # Low dark hearth shelf visible below the door and bonded into the masonry mouth.
    housing.visual(
        Box((1.22, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, -0.43, 0.115)),
        material=blackened_steel,
        name="hearth_sill",
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            _extruded_arch_slab(1.00, 0.090, 0.350, y_min=-0.0375, y_max=0.0175, segments=28),
            "door_slab",
        ),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=blackened_steel,
        name="door_slab",
    )

    for idx, x in enumerate((-0.36, 0.0, 0.36)):
        door.visual(
            Cylinder(radius=0.033, length=0.18),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"hinge_knuckle_{idx}",
        )
        door.visual(
            Box((0.135, 0.025, 0.130)),
            origin=Origin(xyz=(x, -0.038, 0.065)),
            material=brushed_steel,
            name=f"hinge_strap_{idx}",
        )

    # Forged horizontal pull bar with two welded stand-offs and knobbed ends.
    door.visual(
        Cylinder(radius=0.023, length=0.72),
        origin=Origin(xyz=(0.0, -0.096, 0.440), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="handle_bar",
    )
    for idx, x in enumerate((-0.29, 0.29)):
        door.visual(
            Cylinder(radius=0.016, length=0.070),
            origin=Origin(xyz=(x, -0.062, 0.440), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"handle_post_{idx}",
        )
    for idx, x in enumerate((-0.39, 0.39)):
        door.visual(
            Sphere(radius=0.032),
            origin=Origin(xyz=(x, -0.096, 0.440)),
            material=brushed_steel,
            name=f"handle_finial_{idx}",
        )

    # Small raised rivets around the front plate emphasize the heavy forged door.
    rivet_points = [
        (-0.38, -0.046, 0.220),
        (0.38, -0.046, 0.220),
        (-0.42, -0.046, 0.390),
        (0.42, -0.046, 0.390),
        (-0.30, -0.046, 0.665),
        (0.30, -0.046, 0.665),
    ]
    for idx, xyz in enumerate(rivet_points):
        door.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"rivet_{idx}",
        )

    model.articulation(
        "door_axle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("oven_housing")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_axle")

    for idx in range(3):
        knuckle = f"hinge_knuckle_{idx}"
        ctx.allow_overlap(
            housing,
            door,
            elem_a="axle_pin",
            elem_b=knuckle,
            reason="The bottom axle pin is intentionally captured inside the steel hinge knuckle.",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="x",
            elem_a="axle_pin",
            elem_b=knuckle,
            min_overlap=0.14,
            name=f"{knuckle} retained on axle length",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="yz",
            elem_a="axle_pin",
            elem_b=knuckle,
            min_overlap=0.035,
            name=f"{knuckle} coaxial with axle",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            positive_elem="mouth_surround",
            negative_elem="door_slab",
            min_gap=0.0,
            max_gap=0.006,
            name="closed door seats just proud of surround",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            elem_a="door_slab",
            elem_b="mouth_surround",
            min_overlap=0.70,
            name="door covers the arched steel mouth",
        )
        rest_aabb = ctx.part_world_aabb(door)

    with ctx.pose({hinge: 1.75}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door hinges downward and forward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < rest_aabb[1][2] - 0.45
        and open_aabb[0][1] < rest_aabb[0][1] - 0.55,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
