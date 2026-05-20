from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _elliptical_x_loft(sections, segments: int = 40) -> MeshGeometry:
    """Closed smooth fuselage skin lofted along the airplane nose/tail X axis."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, width, height, z_center in sections:
        ring = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            y = 0.5 * width * math.cos(theta)
            z = z_center + 0.5 * height * math.sin(theta)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    for ring, x, width, height, z_center, reverse in (
        (rings[0], *sections[0], True),
        (rings[-1], *sections[-1], False),
    ):
        center = geom.add_vertex(x, 0.0, z_center)
        for i in range(segments):
            j = (i + 1) % segments
            if reverse:
                geom.add_face(center, ring[j], ring[i])
            else:
                geom.add_face(center, ring[i], ring[j])
    return geom


def _thin_planform(profile_xy, z_center: float, root_thickness: float, tip_thickness: float, span: float) -> MeshGeometry:
    """Thin tapered flying surface from a closed XY planform."""
    geom = MeshGeometry()
    top: list[int] = []
    bottom: list[int] = []
    for x, y in profile_xy:
        blend = min(1.0, abs(y) / span)
        thickness = root_thickness * (1.0 - blend) + tip_thickness * blend
        top.append(geom.add_vertex(x, y, z_center + 0.5 * thickness))
        bottom.append(geom.add_vertex(x, y, z_center - 0.5 * thickness))

    n = len(profile_xy)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])

    # Fan triangulation is adequate for these convex/tapered control surfaces.
    for i in range(1, n - 1):
        geom.add_face(top[0], top[i], top[i + 1])
        geom.add_face(bottom[0], bottom[i + 1], bottom[i])
    return geom


def _vertical_fin(profile_xz, half_width: float) -> MeshGeometry:
    """Extrude an XZ fin profile a small amount in Y."""
    geom = MeshGeometry()
    left: list[int] = []
    right: list[int] = []
    for x, z in profile_xz:
        left.append(geom.add_vertex(x, -half_width, z))
        right.append(geom.add_vertex(x, half_width, z))

    n = len(profile_xz)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(left[i], left[j], right[j])
        geom.add_face(left[i], right[j], right[i])
    for i in range(1, n - 1):
        geom.add_face(right[0], right[i], right[i + 1])
        geom.add_face(left[0], left[i + 1], left[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_propeller_model_airplane")

    body_blue = model.material("painted_blue", rgba=(0.10, 0.22, 0.72, 1.0))
    wing_white = model.material("painted_white", rgba=(0.92, 0.94, 0.90, 1.0))
    tail_red = model.material("tail_red", rgba=(0.80, 0.08, 0.05, 1.0))
    smoke = model.material("smoked_canopy", rgba=(0.04, 0.08, 0.12, 0.72))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.65, 0.68, 0.70, 1.0))

    airframe = model.part("airframe")

    fuselage = _elliptical_x_loft(
        [
            (-0.315, 0.018, 0.020, 0.000),
            (-0.255, 0.050, 0.056, 0.004),
            (-0.120, 0.082, 0.092, 0.006),
            (0.080, 0.094, 0.104, 0.006),
            (0.220, 0.066, 0.078, 0.003),
            (0.292, 0.030, 0.036, 0.000),
        ],
        segments=48,
    )
    airframe.visual(mesh_from_geometry(fuselage, "fuselage_shell"), material=body_blue, name="fuselage_shell")

    main_wing = _thin_planform(
        [
            (0.112, -0.070),
            (0.045, -0.370),
            (-0.070, -0.370),
            (-0.132, -0.070),
            (-0.138, 0.000),
            (-0.132, 0.070),
            (-0.070, 0.370),
            (0.045, 0.370),
            (0.112, 0.070),
            (0.122, 0.000),
        ],
        z_center=0.000,
        root_thickness=0.020,
        tip_thickness=0.009,
        span=0.370,
    )
    airframe.visual(mesh_from_geometry(main_wing, "main_wing"), material=wing_white, name="main_wing")

    stabilizer = _thin_planform(
        [
            (-0.205, -0.030),
            (-0.258, -0.165),
            (-0.322, -0.165),
            (-0.334, 0.000),
            (-0.322, 0.165),
            (-0.258, 0.165),
            (-0.205, 0.030),
        ],
        z_center=0.055,
        root_thickness=0.010,
        tip_thickness=0.006,
        span=0.165,
    )
    airframe.visual(mesh_from_geometry(stabilizer, "tailplane"), material=tail_red, name="tailplane")

    fin = _vertical_fin(
        [
            (-0.296, 0.036),
            (-0.188, 0.040),
            (-0.205, 0.145),
            (-0.272, 0.116),
        ],
        half_width=0.010,
    )
    airframe.visual(mesh_from_geometry(fin, "tail_fin"), material=tail_red, name="tail_fin")

    canopy = _elliptical_x_loft(
        [
            (-0.030, 0.018, 0.010, 0.057),
            (0.000, 0.045, 0.026, 0.064),
            (0.060, 0.052, 0.034, 0.071),
            (0.118, 0.034, 0.020, 0.064),
        ],
        segments=32,
    )
    airframe.visual(mesh_from_geometry(canopy, "cockpit_canopy"), material=smoke, name="cockpit_canopy")

    # A small stationary nose bearing stops before the propeller collar, making
    # the continuous spin axis visually obvious without colliding at rest.
    airframe.visual(
        Cylinder(radius=0.022, length=0.048),
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="nose_bearing",
    )

    # Simple display peg under the belly keeps the model airplane visibly
    # supported, like a small desktop model.
    airframe.visual(
        Cylinder(radius=0.011, length=0.185),
        origin=Origin(xyz=(-0.015, 0.0, -0.130)),
        material=metal,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(xyz=(-0.015, 0.0, -0.223)),
        material=rubber,
        name="display_base",
    )

    propeller = model.part("propeller")
    propeller.visual(
        Cylinder(radius=0.031, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rotating_collar",
    )
    rotor = FanRotorGeometry(
        outer_radius=0.125,
        hub_radius=0.026,
        blade_count=2,
        thickness=0.016,
        blade_pitch_deg=30.0,
        blade_sweep_deg=8.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=14.0, camber=0.08, tip_clearance=0.002),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.012, rear_collar_radius=0.024, bore_diameter=0.004),
    )
    propeller.visual(
        mesh_from_geometry(rotor, "two_blade_propeller"),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="two_blade_propeller",
    )

    model.articulation(
        "nose_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.324, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.20, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    prop_joint = object_model.get_articulation("nose_to_propeller")

    ctx.check(
        "propeller joint is continuous",
        prop_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={prop_joint.articulation_type}",
    )
    ctx.check(
        "propeller spins on nose axis",
        tuple(round(v, 6) for v in prop_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={prop_joint.axis}",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="rotating_collar",
        negative_elem="nose_bearing",
        name="rotating collar seats on nose bearing",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        elem_a="rotating_collar",
        elem_b="nose_bearing",
        min_overlap=0.035,
        name="propeller collar is coaxial with nose bearing",
    )

    rest_position = ctx.part_world_position(propeller)
    with ctx.pose({prop_joint: math.pi / 2.0}):
        spun_position = ctx.part_world_position(propeller)
        ctx.expect_gap(
            propeller,
            airframe,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0005,
            positive_elem="rotating_collar",
            negative_elem="nose_bearing",
            name="spinning propeller keeps bearing clearance",
        )

    ctx.check(
        "continuous spin keeps propeller centered",
        rest_position is not None
        and spun_position is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, spun_position)),
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
