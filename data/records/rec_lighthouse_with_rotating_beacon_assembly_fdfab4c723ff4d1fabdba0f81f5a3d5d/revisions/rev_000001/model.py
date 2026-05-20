from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _frustum_geometry(
    bottom_radius: float,
    top_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Closed truncated cone aligned on the world Z axis."""
    geom = MeshGeometry()
    bottom = []
    top = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        bottom.append(geom.add_vertex(bottom_radius * ca, bottom_radius * sa, z_min))
        top.append(geom.add_vertex(top_radius * ca, top_radius * sa, z_max))

    bottom_center = geom.add_vertex(0.0, 0.0, z_min)
    top_center = geom.add_vertex(0.0, 0.0, z_max)

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return geom


def _annular_cylinder_geometry(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Flat ring/deck with a central opening."""
    geom = MeshGeometry()
    ob, ot, ib, it = [], [], [], []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        ob.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_min))
        ot.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, z_max))
        ib.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_min))
        it.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, z_max))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer and inner cylindrical walls.
        geom.add_face(ob[i], ob[j], ot[j])
        geom.add_face(ob[i], ot[j], ot[i])
        geom.add_face(ib[j], ib[i], it[i])
        geom.add_face(ib[j], it[i], it[j])
        # Top and bottom annular faces.
        geom.add_face(ot[i], ot[j], it[j])
        geom.add_face(ot[i], it[j], it[i])
        geom.add_face(ob[j], ob[i], ib[i])
        geom.add_face(ob[j], ib[i], ib[j])
    return geom


def _arc_points(radius: float, z: float, start: float, end: float, count: int) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(start + (end - start) * i / (count - 1)),
            radius * math.sin(start + (end - start) * i / (count - 1)),
            z,
        )
        for i in range(count)
    ]


def _surface_box_origin(radius: float, theta: float, z: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(theta), radius * math.sin(theta), z),
        rpy=(0.0, 0.0, theta - math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="striped_lighthouse")

    red = model.material("oxide_red", rgba=(0.72, 0.07, 0.045, 1.0))
    white = model.material("warm_white", rgba=(0.93, 0.90, 0.80, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.42, 0.42, 0.39, 1.0))
    dark = model.material("blackened_iron", rgba=(0.035, 0.038, 0.040, 1.0))
    cap_red = model.material("dark_cap_red", rgba=(0.42, 0.035, 0.03, 1.0))
    brass = model.material("aged_brass", rgba=(0.75, 0.58, 0.27, 1.0))
    glass = model.material("slightly_green_glass", rgba=(0.62, 0.88, 0.92, 0.34))
    yellow_glass = model.material("warm_fresnel_glass", rgba=(1.0, 0.84, 0.26, 0.55))
    light = model.material("lit_lamp", rgba=(1.0, 0.92, 0.45, 1.0))
    shadow = model.material("dark_opening", rgba=(0.005, 0.006, 0.008, 1.0))

    tower = model.part("tower")

    # Heavy foundation and the tall tapered striped masonry body.
    tower.visual(
        Cylinder(radius=1.55, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=concrete,
        name="foundation",
    )

    tower_z0, tower_z1 = 0.35, 8.75
    tower_r0, tower_r1 = 1.05, 0.58

    def tower_radius(z: float) -> float:
        return tower_r0 + (tower_r1 - tower_r0) * ((z - tower_z0) / (tower_z1 - tower_z0))

    stripe_count = 8
    for i in range(stripe_count):
        z0 = tower_z0 + (tower_z1 - tower_z0) * i / stripe_count
        z1 = tower_z0 + (tower_z1 - tower_z0) * (i + 1) / stripe_count
        band = _frustum_geometry(tower_radius(z0), tower_radius(z1), z0, z1)
        tower.visual(
            mesh_from_geometry(band, f"stripe_band_{i}"),
            material=red if i % 2 == 0 else white,
            name=f"stripe_band_{i}",
        )

    # Door and small stair windows are seated into the masonry surface.
    tower.visual(
        Box((0.56, 0.045, 1.35)),
        origin=Origin(xyz=(0.0, -tower_radius(1.05) - 0.006, 1.05)),
        material=shadow,
        name="base_door",
    )
    tower.visual(
        Box((0.68, 0.032, 0.08)),
        origin=Origin(xyz=(0.0, -tower_radius(1.76) - 0.018, 1.76)),
        material=brass,
        name="door_lintel",
    )
    for idx, (theta, z) in enumerate(((math.radians(18), 3.05), (math.radians(165), 5.25), (math.radians(-34), 7.05))):
        r = tower_radius(z) + 0.010
        tower.visual(
            Box((0.30, 0.035, 0.42)),
            origin=_surface_box_origin(r, theta, z),
            material=shadow,
            name=f"stair_window_{idx}",
        )

    # Gallery deck and guard rail with a gap for the hinged trap door.
    tower.visual(
        mesh_from_geometry(_annular_cylinder_geometry(0.52, 1.35, 8.75, 8.96), "gallery_deck"),
        material=concrete,
        name="gallery_deck",
    )
    tower.visual(
        Cylinder(radius=0.57, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 9.10)),
        material=white,
        name="lantern_curb",
    )

    rail_radius = 1.27
    gate_width = 0.64
    gate_half = gate_width / 2.0
    gate_y = -math.sqrt(rail_radius * rail_radius - gate_half * gate_half)
    hinge_x = -gate_half
    latch_x = gate_half
    hinge_theta = math.atan2(gate_y, hinge_x)
    latch_theta = math.atan2(gate_y, latch_x)
    rail_start = latch_theta + 0.055
    rail_end = hinge_theta + 2.0 * math.pi - 0.055
    for z, name in ((9.32, "mid_rail"), (9.66, "top_rail")):
        arc = wire_from_points(
            _arc_points(rail_radius, z, rail_start, rail_end, 76),
            radius=0.026,
            radial_segments=14,
            closed_path=False,
            cap_ends=True,
            corner_mode="miter",
        )
        tower.visual(mesh_from_geometry(arc, name), material=dark, name=name)

    # Railing posts include a named hinge post for the trap door sleeve.
    post_angles = [hinge_theta, latch_theta]
    for i in range(14):
        a = 2.0 * math.pi * i / 14.0
        if not (hinge_theta < a - 2.0 * math.pi < latch_theta):
            # Skip only the gallery opening centered at -Y; all other posts remain.
            if not (hinge_theta < a < latch_theta):
                post_angles.append(a)
    seen = set()
    for idx, theta in enumerate(post_angles):
        key = round(theta, 4)
        if key in seen:
            continue
        seen.add(key)
        name = "hinge_post" if abs(theta - hinge_theta) < 1e-6 else ("latch_post" if abs(theta - latch_theta) < 1e-6 else f"rail_post_{idx}")
        tower.visual(
            Cylinder(radius=0.032, length=0.78),
            origin=Origin(xyz=(rail_radius * math.cos(theta), rail_radius * math.sin(theta), 9.35)),
            material=dark,
            name=name,
        )

    # Narrow lantern room: metal rings, mullions, and transparent glazing.
    tower.visual(
        mesh_from_geometry(_annular_cylinder_geometry(0.38, 0.73, 9.24, 9.42), "lower_lantern_ring"),
        material=dark,
        name="lower_lantern_ring",
    )
    tower.visual(
        mesh_from_geometry(_annular_cylinder_geometry(0.40, 0.70, 10.84, 11.00), "upper_lantern_ring"),
        material=dark,
        name="upper_lantern_ring",
    )
    tower.visual(
        Cylinder(radius=0.405, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 9.39)),
        material=dark,
        name="lantern_floor",
    )
    pane_radius = 0.61
    pane_height = 1.50
    for i in range(8):
        theta = 2.0 * math.pi * (i + 0.5) / 8.0
        tower.visual(
            Box((0.40, 0.026, pane_height)),
            origin=_surface_box_origin(pane_radius, theta, 10.12),
            material=glass,
            name=f"glass_pane_{i}",
        )
    for i in range(8):
        theta = 2.0 * math.pi * i / 8.0
        tower.visual(
            Box((0.060, 0.090, 1.58)),
            origin=_surface_box_origin(pane_radius, theta, 10.12),
            material=dark,
            name=f"mullion_{i}",
        )

    # The fixed central pedestal is the physical axis for the rotating beacon.
    tower.visual(
        Cylinder(radius=0.16, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 9.81)),
        material=brass,
        name="pedestal",
    )

    # Vented cap: louver slots around a narrow metal band, conical roof, finial.
    tower.visual(
        mesh_from_geometry(_frustum_geometry(0.70, 0.64, 11.00, 11.35), "vent_band"),
        material=dark,
        name="vent_band",
    )
    for i in range(8):
        theta = 2.0 * math.pi * (i + 0.5) / 8.0
        for row, z in enumerate((11.10, 11.20, 11.30)):
            tower.visual(
                Box((0.28, 0.030, 0.055)),
                origin=_surface_box_origin(0.665, theta, z),
                material=shadow,
                name=f"cap_vent_{i}_{row}",
            )
    tower.visual(
        mesh_from_geometry(_frustum_geometry(0.78, 0.16, 11.35, 11.95), "conical_cap"),
        material=cap_red,
        name="conical_cap",
    )
    tower.visual(
        Sphere(radius=0.10),
        origin=Origin(xyz=(0.0, 0.0, 12.05)),
        material=brass,
        name="finial_ball",
    )
    tower.visual(
        Cylinder(radius=0.022, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 12.32)),
        material=dark,
        name="lightning_rod",
    )

    # Continuously rotating beacon mounted exactly on the pedestal axis.
    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.22, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark,
        name="turntable",
    )
    beacon.visual(
        Cylinder(radius=0.24, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.37), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yellow_glass,
        name="lens",
    )
    beacon.visual(
        Sphere(radius=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=light,
        name="lamp",
    )
    for x, name in ((-0.47, "lens_ring_0"), (0.47, "lens_ring_1")):
        beacon.visual(
            Cylinder(radius=0.255, length=0.070),
            origin=Origin(xyz=(x * 0.936, 0.0, 0.37), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=name,
        )
    beacon.visual(
        Box((0.16, 0.54, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=brass,
        name="yoke",
    )

    model.articulation(
        "pedestal_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 10.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    # Hinged trap door/gate in the gallery railing.
    trap = model.part("trap_door")
    trap.visual(
        Cylinder(radius=0.048, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=dark,
        name="hinge_sleeve",
    )
    for z, name in ((0.35, "mid_bar"), (0.69, "top_bar")):
        trap.visual(
            Cylinder(radius=0.028, length=0.50),
            origin=Origin(xyz=(0.315, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=name,
        )
        trap.visual(
            Box((0.052, 0.070, 0.065)),
            origin=Origin(xyz=(0.062, 0.0, z)),
            material=dark,
            name=f"{name}_weld",
        )
    trap.visual(
        Cylinder(radius=0.026, length=0.58),
        origin=Origin(xyz=(0.565, 0.0, 0.50)),
        material=dark,
        name="latch_stile",
    )
    trap.visual(
        Box((0.10, 0.025, 0.10)),
        origin=Origin(xyz=(0.48, 0.0, 0.66)),
        material=brass,
        name="latch_plate",
    )
    model.articulation(
        "rail_to_trap_door",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=trap,
        origin=Origin(xyz=(hinge_x, gate_y, 8.97)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    trap = object_model.get_part("trap_door")
    spin = object_model.get_articulation("pedestal_to_beacon")
    hinge = object_model.get_articulation("rail_to_trap_door")

    ctx.allow_overlap(
        tower,
        trap,
        elem_a="hinge_post",
        elem_b="hinge_sleeve",
        reason="The trap door hinge sleeve is intentionally captured around the fixed gallery hinge post.",
    )
    ctx.expect_overlap(
        tower,
        trap,
        axes="z",
        elem_a="hinge_post",
        elem_b="hinge_sleeve",
        min_overlap=0.55,
        name="trap door hinge sleeve captures the post height",
    )

    ctx.check(
        "beacon uses continuous rotation",
        "CONTINUOUS" in str(spin.articulation_type),
        details=f"type={spin.articulation_type}",
    )
    ctx.check(
        "beacon joint is on pedestal vertical axis",
        abs(spin.origin.xyz[0]) < 1e-6 and abs(spin.origin.xyz[1]) < 1e-6 and spin.axis == (0.0, 0.0, 1.0),
        details=f"origin={spin.origin.xyz}, axis={spin.axis}",
    )
    ctx.expect_origin_distance(
        beacon,
        tower,
        axes="xy",
        max_dist=0.001,
        name="beacon part origin remains centered over tower axis",
    )
    ctx.expect_contact(
        beacon,
        tower,
        elem_a="turntable",
        elem_b="pedestal",
        contact_tol=0.002,
        name="beacon turntable stands on the pedestal",
    )
    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="lens",
        outer_elem="upper_lantern_ring",
        margin=0.0,
        name="beacon lens stays within the lantern glazing ring",
    )

    with ctx.pose({spin: 1.7}):
        ctx.expect_origin_distance(
            beacon,
            tower,
            axes="xy",
            max_dist=0.001,
            name="rotated beacon stays clipped to vertical pedestal axis",
        )
        ctx.expect_within(
            beacon,
            tower,
            axes="xy",
            inner_elem="lens",
            outer_elem="upper_lantern_ring",
            margin=0.0,
            name="rotated beacon remains inside the lantern room",
        )

    closed_aabb = ctx.part_world_aabb(trap)
    with ctx.pose({hinge: 1.1}):
        opened_aabb = ctx.part_world_aabb(trap)
    ctx.check(
        "trap door swings outward from gallery rail",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.20,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
