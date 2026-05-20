from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


LEG_SPECS: tuple[tuple[str, str, float], ...] = (
    ("front_leg", "hub_to_front_leg", 0.0),
    ("left_rear_leg", "hub_to_left_rear_leg", 2.0 * math.pi / 3.0),
    ("right_rear_leg", "hub_to_right_rear_leg", 4.0 * math.pi / 3.0),
)


def _world_from_local(
    base_xyz: tuple[float, float, float],
    yaw: float,
    local_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    lx, ly, lz = local_xyz
    bx, by, bz = base_xyz
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        bx + (lx * c) - (ly * s),
        by + (lx * s) + (ly * c),
        bz + lz,
    )


def _add_quad(
    geom: MeshGeometry,
    a: int,
    b: int,
    c: int,
    d: int,
) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _make_revolved_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
) -> MeshGeometry:
    geom = MeshGeometry()

    def add_ring(radius: float, z_pos: float) -> list[int]:
        ring: list[int] = []
        for segment in range(segments):
            angle = (2.0 * math.pi * segment) / segments
            ring.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z_pos))
        return ring

    outer_rings = [add_ring(radius, z_pos) for radius, z_pos in outer_profile]
    inner_rings = [add_ring(radius, z_pos) for radius, z_pos in inner_profile]

    for ring_index in range(len(outer_rings) - 1):
        lower_ring = outer_rings[ring_index]
        upper_ring = outer_rings[ring_index + 1]
        for segment in range(segments):
            next_segment = (segment + 1) % segments
            _add_quad(
                geom,
                lower_ring[segment],
                lower_ring[next_segment],
                upper_ring[next_segment],
                upper_ring[segment],
            )

    for ring_index in range(len(inner_rings) - 1):
        lower_ring = inner_rings[ring_index]
        upper_ring = inner_rings[ring_index + 1]
        for segment in range(segments):
            next_segment = (segment + 1) % segments
            _add_quad(
                geom,
                lower_ring[segment],
                upper_ring[segment],
                upper_ring[next_segment],
                lower_ring[next_segment],
            )

    bottom_outer = outer_rings[0]
    bottom_inner = inner_rings[0]
    top_outer = outer_rings[-1]
    top_inner = inner_rings[-1]

    for segment in range(segments):
        next_segment = (segment + 1) % segments
        _add_quad(
            geom,
            bottom_outer[segment],
            bottom_inner[segment],
            bottom_inner[next_segment],
            bottom_outer[next_segment],
        )
        _add_quad(
            geom,
            top_outer[segment],
            top_outer[next_segment],
            top_inner[next_segment],
            top_inner[segment],
        )

    return geom


def _build_shade_bowl_mesh() -> MeshGeometry:
    outer_profile = [
        (0.034, 0.036),
        (0.058, 0.046),
        (0.110, 0.074),
        (0.162, 0.108),
        (0.191, 0.134),
        (0.205, 0.154),
    ]
    inner_profile = [
        (0.028, 0.040),
        (0.052, 0.049),
        (0.103, 0.075),
        (0.154, 0.107),
        (0.183, 0.131),
        (0.197, 0.149),
    ]
    return _make_revolved_shell(outer_profile, inner_profile, segments=84)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_floor_lamp")

    painted_steel = model.material("painted_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.62, 0.52, 0.34, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.92, 0.90, 0.84, 0.92))

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.056, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=painted_steel,
        name="hub_body",
    )
    hub.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.303)),
        material=satin_brass,
        name="hub_socket",
    )

    hinge_radius = 0.072
    hinge_height = 0.238
    leg_drop_angle = 0.37
    leg_axis = (
        math.cos(leg_drop_angle),
        0.0,
        -math.sin(leg_drop_angle),
    )

    for leg_name, _, yaw in LEG_SPECS:
        hinge_origin = (
            hinge_radius * math.cos(yaw),
            hinge_radius * math.sin(yaw),
            hinge_height,
        )
        spine_center = _world_from_local(hinge_origin, yaw, (-0.020, 0.0, 0.0))
        lug_upper_center = _world_from_local(hinge_origin, yaw, (-0.017, 0.019, 0.0))
        lug_lower_center = _world_from_local(hinge_origin, yaw, (-0.017, -0.019, 0.0))

        hub.visual(
            Box((0.016, 0.046, 0.020)),
            origin=Origin(xyz=spine_center, rpy=(0.0, 0.0, yaw)),
            material=painted_steel,
            name=f"{leg_name}_mount_spine",
        )
        hub.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=lug_upper_center, rpy=(math.pi / 2.0, 0.0, yaw)),
            material=painted_steel,
            name=f"{leg_name}_mount_lug_upper",
        )
        hub.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=lug_lower_center, rpy=(math.pi / 2.0, 0.0, yaw)),
            material=painted_steel,
            name=f"{leg_name}_mount_lug_lower",
        )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_brass,
        name="column_ferrule",
    )
    column.visual(
        Cylinder(radius=0.014, length=1.240),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=painted_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.2675)),
        material=satin_brass,
        name="column_tip",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.032, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_brass,
        name="shade_socket",
    )
    shade.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_brass,
        name="shade_flange",
    )
    shade.visual(
        mesh_from_geometry(_build_shade_bowl_mesh(), "shade_bowl"),
        material=frosted_glass,
        name="shade_bowl",
    )

    model.articulation(
        "hub_to_column",
        ArticulationType.FIXED,
        parent=hub,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
    )
    model.articulation(
        "column_to_shade",
        ArticulationType.FIXED,
        parent=column,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 1.295)),
    )

    spar_pitch = (math.pi / 2.0) + leg_drop_angle
    spar_center_distance = 0.302
    foot_center_distance = 0.590

    for leg_name, joint_name, yaw in LEG_SPECS:
        leg = model.part(leg_name)
        leg.visual(
            Cylinder(radius=0.011, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name="hinge_barrel",
        )
        leg.visual(
            Sphere(radius=0.015),
            origin=Origin(
                xyz=(
                    leg_axis[0] * 0.020,
                    0.0,
                    leg_axis[2] * 0.020,
                )
            ),
            material=painted_steel,
            name="hinge_knuckle",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.550),
            origin=Origin(
                xyz=(
                    leg_axis[0] * spar_center_distance,
                    0.0,
                    leg_axis[2] * spar_center_distance,
                ),
                rpy=(0.0, spar_pitch, 0.0),
            ),
            material=painted_steel,
            name="spar_tube",
        )
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(
                xyz=(
                    leg_axis[0] * foot_center_distance,
                    0.0,
                    leg_axis[2] * foot_center_distance,
                )
            ),
            material=foot_rubber,
            name="foot_cap",
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=hub,
            child=leg,
            origin=Origin(
                xyz=(
                    hinge_radius * math.cos(yaw),
                    hinge_radius * math.sin(yaw),
                    hinge_height,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=1.2,
                lower=0.0,
                upper=1.40,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub")
    column = object_model.get_part("column")
    shade = object_model.get_part("shade")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        column,
        hub,
        elem_a="column_ferrule",
        elem_b="hub_socket",
        name="column ferrule seats on hub socket",
    )
    ctx.expect_contact(
        shade,
        column,
        elem_a="shade_socket",
        elem_b="column_tip",
        name="shade socket seats on column tip",
    )

    for leg_name, joint_name, _ in LEG_SPECS:
        leg = object_model.get_part(leg_name)
        hinge = object_model.get_articulation(joint_name)
        upper_limit = hinge.motion_limits.upper if hinge.motion_limits is not None else None

        ctx.expect_contact(
            leg,
            hub,
            elem_a="hinge_barrel",
            name=f"{leg_name} hinge barrel stays mounted in the hub clevis",
        )

        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_cap")
        foot_center = _aabb_center(foot_aabb)
        foot_min_z = None if foot_aabb is None else foot_aabb[0][2]
        foot_radius = None
        if foot_center is not None:
            foot_radius = math.hypot(foot_center[0], foot_center[1])

        ctx.check(
            f"{leg_name} foot sits near the floor plane when deployed",
            foot_min_z is not None and abs(foot_min_z) <= 0.01,
            details=f"foot_min_z={foot_min_z}",
        )
        ctx.check(
            f"{leg_name} is splayed well out from the centerline",
            foot_radius is not None and foot_radius >= 0.50,
            details=f"foot_radius={foot_radius}",
        )

        folded_foot_aabb = None
        if upper_limit is not None:
            with ctx.pose({hinge: upper_limit}):
                folded_foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_cap")
        folded_center = _aabb_center(folded_foot_aabb)

        ctx.check(
            f"{leg_name} folds upward on its hinge",
            foot_center is not None
            and folded_center is not None
            and folded_center[2] > foot_center[2] + 0.45,
            details=f"deployed={foot_center}, folded={folded_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
