from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _blade_airfoil_mesh(name_hint: str) -> MeshGeometry:
    """Small tapered turbine blade with a twisted airfoil-like section.

    The blade frame has its root hinge at the origin and the working blade
    extends along local +Y.  Local +Z is roughly the tangential chord direction
    and local X is axial thickness, matching the rotor spin axis.
    """

    geom = MeshGeometry()
    section_specs = [
        # y, chord, thickness, twist radians, camber
        (0.010, 0.030, 0.0080, math.radians(13.0), 0.0018),
        (0.030, 0.028, 0.0070, math.radians(9.0), 0.0012),
        (0.060, 0.022, 0.0056, math.radians(4.0), 0.0008),
        (0.090, 0.016, 0.0040, math.radians(-1.0), 0.0004),
        (0.112, 0.008, 0.0024, math.radians(-5.0), 0.0000),
    ]
    points_per_section = 14
    rings: list[list[int]] = []

    for y, chord, thickness, twist, camber in section_specs:
        ring: list[int] = []
        for i in range(points_per_section):
            a = 2.0 * math.pi * i / points_per_section
            # An asymmetric oval is enough to read as a miniature airfoil.
            raw_x = 0.5 * thickness * math.sin(a) + camber * (1.0 + math.cos(a))
            raw_z = 0.5 * chord * math.cos(a)
            x = raw_x * math.cos(twist) + raw_z * math.sin(twist)
            z = -raw_x * math.sin(twist) + raw_z * math.cos(twist)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for ring_a, ring_b in zip(rings[:-1], rings[1:]):
        for i in range(points_per_section):
            j = (i + 1) % points_per_section
            geom.add_face(ring_a[i], ring_a[j], ring_b[j])
            geom.add_face(ring_a[i], ring_b[j], ring_b[i])

    root_center = geom.add_vertex(0.0, section_specs[0][0], 0.0)
    tip_center = geom.add_vertex(0.0, section_specs[-1][0], 0.0)
    root_ring = rings[0]
    tip_ring = rings[-1]
    for i in range(points_per_section):
        j = (i + 1) % points_per_section
        geom.add_face(root_center, root_ring[j], root_ring[i])
        geom.add_face(tip_center, tip_ring[i], tip_ring[j])

    # Give each exported asset a stable but semantically distinct mesh name.
    geom.meta = {"name_hint": name_hint}
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_foldable_wind_turbine")

    white = model.material("satin_white", rgba=(0.92, 0.94, 0.92, 1.0))
    dark = model.material("graphite", rgba=(0.08, 0.09, 0.10, 1.0))
    metal = model.material("brushed_metal", rgba=(0.58, 0.61, 0.62, 1.0))
    blue = model.material("muted_blue", rgba=(0.18, 0.36, 0.62, 1.0))
    safety = model.material("translucent_guard", rgba=(0.72, 0.86, 0.95, 0.42))

    # Root desktop base: long enough for the tower to fold over it without
    # increasing the working rotor diameter.
    base = model.part("base")
    base.visual(
        Box((0.50, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark,
        name="weighted_plate",
    )
    base.visual(
        Box((0.050, 0.012, 0.070)),
        origin=Origin(xyz=(0.130, 0.041, 0.057)),
        material=metal,
        name="hinge_cheek_0",
    )
    base.visual(
        Box((0.050, 0.012, 0.070)),
        origin=Origin(xyz=(0.130, -0.041, 0.057)),
        material=metal,
        name="hinge_cheek_1",
    )
    base.visual(
        Box((0.070, 0.090, 0.014)),
        origin=Origin(xyz=(0.130, 0.0, 0.027)),
        material=metal,
        name="hinge_foot",
    )
    base.visual(
        Box((0.070, 0.078, 0.012)),
        origin=Origin(xyz=(-0.118, 0.0, 0.035)),
        material=blue,
        name="stow_cradle_pad",
    )
    base.visual(
        Box((0.018, 0.018, 0.060)),
        origin=Origin(xyz=(-0.150, 0.043, 0.055)),
        material=blue,
        name="cradle_post_0",
    )
    base.visual(
        Box((0.018, 0.018, 0.060)),
        origin=Origin(xyz=(-0.150, -0.043, 0.055)),
        material=blue,
        name="cradle_post_1",
    )

    # Folding tower.  The frame origin is the hinge axis, so q=0 is upright and
    # positive q folds the turbine rearward over the base.
    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    tower.visual(
        Cylinder(radius=0.013, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=white,
        name="mast",
    )
    tower.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.341)),
        material=metal,
        name="top_bearing",
    )
    tower.visual(
        Box((0.034, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=metal,
        name="lower_clamp",
    )
    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.130, 0.0, 0.065)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.7, lower=0.0, upper=1.32),
    )

    # Nacelle with yaw bearing, compact generator body, front bearing, and a
    # stationary safety hoop sized to clear the folding/pitching blades.
    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=metal,
        name="yaw_collar",
    )
    body_mesh = CapsuleGeometry(radius=0.028, length=0.066, radial_segments=32)
    body_mesh.rotate_y(math.pi / 2.0).translate(0.045, 0.0, 0.037)
    nacelle.visual(
        mesh_from_geometry(body_mesh, "nacelle_body"),
        material=white,
        name="generator_body",
    )
    nacelle.visual(
        Box((0.050, 0.006, 0.006)),
        origin=Origin(xyz=(0.112, 0.017, 0.037)),
        material=metal,
        name="bearing_arm_0",
    )
    nacelle.visual(
        Box((0.050, 0.006, 0.006)),
        origin=Origin(xyz=(0.112, -0.017, 0.037)),
        material=metal,
        name="bearing_arm_1",
    )
    bearing_ring = TorusGeometry(radius=0.0155, tube=0.0035, radial_segments=24, tubular_segments=36)
    bearing_ring.rotate_y(math.pi / 2.0).translate(0.120, 0.0, 0.037)
    nacelle.visual(
        mesh_from_geometry(bearing_ring, "nose_bearing_ring"),
        material=metal,
        name="nose_bearing",
    )
    guard_ring = TorusGeometry(radius=0.198, tube=0.004, radial_segments=32, tubular_segments=72)
    guard_ring.rotate_y(math.pi / 2.0).translate(0.130, 0.0, 0.037)
    nacelle.visual(
        mesh_from_geometry(guard_ring, "rotor_guard_ring"),
        material=safety,
        name="guard_hoop",
    )
    nacelle.visual(
        Cylinder(radius=0.0045, length=0.178),
        origin=Origin(xyz=(0.130, 0.109, 0.037), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety,
        name="guard_spoke_0",
    )
    nacelle.visual(
        Cylinder(radius=0.0045, length=0.178),
        origin=Origin(xyz=(0.130, -0.109, 0.037), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety,
        name="guard_spoke_1",
    )
    nacelle.visual(
        Box((0.006, 0.026, 0.060)),
        origin=Origin(xyz=(-0.006, 0.0, 0.066)),
        material=blue,
        name="tail_fin",
    )
    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )

    # Rotor hub: a real load path from the main hub through three root lugs to
    # blade hinge axes.  It spins as one continuous assembly about local +X.
    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.029, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub_drum",
    )
    hub.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="drive_shaft",
    )
    hub.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="spinner_cap",
    )

    root_radius = 0.055
    lug_mid = 0.038
    lug_length = 0.018
    blade_angles = [math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0]
    for i, phi in enumerate(blade_angles):
        y = lug_mid * math.cos(phi)
        z = lug_mid * math.sin(phi)
        hub.visual(
            Cylinder(radius=0.0095, length=lug_length),
            origin=Origin(xyz=(0.0, y, z), rpy=(phi - math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"root_lug_{i}",
        )

    model.articulation(
        "nacelle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(0.148, 0.0, 0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    # Three identical folding/pitching blade root parts.  The joint frame is
    # rotated around hub X so each blade's local +Y is radial at q=0.  Positive
    # blade-root motion folds the blade forward along the hub axis for flat stow.
    for i, phi in enumerate(blade_angles):
        blade = model.part(f"blade_{i}")
        blade.visual(
            Cylinder(radius=0.0085, length=0.024),
            origin=Origin(),
            material=metal,
            name="root_barrel",
        )
        blade.visual(
            Cylinder(radius=0.0065, length=0.026),
            origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="root_plug",
        )
        blade.visual(
            mesh_from_geometry(_blade_airfoil_mesh(f"blade_{i}"), f"blade_airfoil_{i}"),
            material=white,
            name="airfoil",
        )
        blade.visual(
            Box((0.004, 0.012, 0.020)),
            origin=Origin(xyz=(0.0, 0.019, 0.0)),
            material=blue,
            name="pitch_mark",
        )

        model.articulation(
            f"hub_to_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=blade,
            origin=Origin(
                xyz=(0.0, root_radius * math.cos(phi), root_radius * math.sin(phi)),
                rpy=(phi, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.6, velocity=1.4, lower=0.0, upper=1.38),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("hub")
    blades = [object_model.get_part(f"blade_{i}") for i in range(3)]
    tower_hinge = object_model.get_articulation("base_to_tower")
    rotor_spin = object_model.get_articulation("nacelle_to_hub")
    blade_roots = [object_model.get_articulation(f"hub_to_blade_{i}") for i in range(3)]

    for i, blade in enumerate(blades):
        ctx.allow_overlap(
            blade,
            hub,
            elem_a="root_barrel",
            elem_b=f"root_lug_{i}",
            reason=(
                "The blade-root hinge barrel is intentionally nested in the "
                "hub root lug to show a captured load path at the folding hinge."
            ),
        )
        ctx.expect_overlap(
            blade,
            hub,
            axes="x",
            elem_a="root_barrel",
            elem_b=f"root_lug_{i}",
            min_overlap=0.010,
            name=f"blade_{i} root barrel is captured by hub lug",
        )

    ctx.expect_gap(
        hub,
        tower,
        axis="x",
        min_gap=0.07,
        name="operating rotor is forward of the tower",
    )
    ctx.expect_gap(
        hub,
        base,
        axis="z",
        min_gap=0.20,
        name="operating rotor has desktop clearance",
    )
    ctx.expect_within(
        hub,
        nacelle,
        axes="z",
        margin=0.17,
        inner_elem="hub_drum",
        outer_elem="guard_hoop",
        name="hub center sits inside safety hoop height",
    )

    # Prove that the fold-flat/stow mode keeps the rotor above the base when
    # blades are folded forward along the hub axis.
    stow_pose = {tower_hinge: 1.20}
    for blade_joint in blade_roots:
        stow_pose[blade_joint] = 1.30
    with ctx.pose(stow_pose):
        ctx.expect_gap(
            hub,
            base,
            axis="z",
            min_gap=0.10,
            name="folded tower keeps hub above base",
        )
        for i, blade in enumerate(blades):
            ctx.expect_gap(
                blade,
                base,
                axis="z",
                min_gap=0.035,
                name=f"folded blade_{i} clears desktop base",
            )

    with ctx.pose({rotor_spin: 1.1}):
        ctx.expect_gap(
            hub,
            base,
            axis="z",
            min_gap=0.19,
            name="spun hub still clears base",
        )

    return ctx.report()


object_model = build_object_model()
