from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


TRUNNION_X = 0.0
TRUNNION_Z = 0.82
WHEEL_X = -0.08
WHEEL_Z = 0.50
WHEEL_Y = 0.62


def _barrel_geometry(segments: int = 56) -> MeshGeometry:
    """A lathed smooth-bore cannon barrel, authored along local +X."""
    # (x, radius) profile with breech knob, reinforces, tapered chase, and muzzle swell.
    profile = [
        (-0.82, 0.018),
        (-0.79, 0.052),
        (-0.75, 0.074),
        (-0.71, 0.058),
        (-0.67, 0.070),
        (-0.63, 0.125),
        (-0.58, 0.168),
        (-0.43, 0.176),
        (-0.39, 0.158),
        (-0.12, 0.158),
        (-0.08, 0.174),
        (0.03, 0.168),
        (0.10, 0.148),
        (0.42, 0.126),
        (0.47, 0.142),
        (0.53, 0.132),
        (0.78, 0.112),
        (0.84, 0.136),
        (0.92, 0.142),
    ]
    bore_radius = 0.055
    bore_depth_x = 0.48

    geom = MeshGeometry()

    def add_ring(x: float, radius: float) -> list[int]:
        return [
            geom.add_vertex(x, radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
            for i in range(segments)
        ]

    outer_rings = [add_ring(x, r) for x, r in profile]
    for ring_a, ring_b in zip(outer_rings, outer_rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(ring_a[i], ring_b[i], ring_b[j])
            geom.add_face(ring_a[i], ring_b[j], ring_a[j])

    # Solid breech cap.
    breech_center = geom.add_vertex(profile[0][0], 0.0, 0.0)
    first = outer_rings[0]
    for i in range(segments):
        geom.add_face(breech_center, first[(i + 1) % segments], first[i])

    # Open muzzle annulus and a short dark bore cavity.
    inner_front = add_ring(profile[-1][0], bore_radius)
    inner_back = add_ring(bore_depth_x, bore_radius)
    muzzle_outer = outer_rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(muzzle_outer[i], muzzle_outer[j], inner_front[j])
        geom.add_face(muzzle_outer[i], inner_front[j], inner_front[i])
        geom.add_face(inner_front[i], inner_front[j], inner_back[j])
        geom.add_face(inner_front[i], inner_back[j], inner_back[i])

    bore_back_center = geom.add_vertex(bore_depth_x, 0.0, 0.0)
    for i in range(segments):
        geom.add_face(bore_back_center, inner_back[i], inner_back[(i + 1) % segments])

    return geom


def _wedge_geometry() -> MeshGeometry:
    """Triangular-prism quoin wedge with a sloped upper face."""
    length = 0.38
    width = 0.22
    low = 0.050
    high = 0.112
    hx = length / 2.0
    hy = width / 2.0
    geom = MeshGeometry()
    verts = [
        (-hx, -hy, 0.0),
        (hx, -hy, 0.0),
        (hx, hy, 0.0),
        (-hx, hy, 0.0),
        (-hx, -hy, high),
        (hx, -hy, low),
        (hx, hy, low),
        (-hx, hy, high),
    ]
    ids = [geom.add_vertex(*v) for v in verts]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
        (4, 7, 6),
        (4, 6, 5),
    ]
    for a, b, c in faces:
        geom.add_face(ids[a], ids[b], ids[c])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="napoleonic_field_cannon")

    wood = model.material("oiled_oak", rgba=(0.47, 0.27, 0.12, 1.0))
    dark_wood = model.material("dark_endgrain", rgba=(0.30, 0.16, 0.07, 1.0))
    iron = model.material("blackened_iron", rgba=(0.045, 0.045, 0.043, 1.0))
    barrel_bronze = model.material("aged_cast_bronze", rgba=(0.40, 0.30, 0.17, 1.0))
    bore_black = model.material("shadowed_bore", rgba=(0.005, 0.004, 0.003, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.16, 0.28, 0.08)),
        origin=Origin(xyz=(-0.22, 0.0, 0.51)),
        material=wood,
        name="bed_plank",
    )
    for idx, y in enumerate((-0.30, 0.30)):
        carriage.visual(
            Box((1.34, 0.105, 0.18)),
            origin=Origin(xyz=(-0.10, y, 0.665)),
            material=wood,
            name=f"cheek_{idx}",
        )
        carriage.visual(
            Box((0.22, 0.105, 0.105)),
            origin=Origin(xyz=(0.00, y, 0.792)),
            material=wood,
            name=f"trunnion_block_{idx}",
        )
        carriage.visual(
            Box((0.20, 0.100, 0.020)),
            origin=Origin(xyz=(0.00, y, 0.854)),
            material=iron,
            name=f"trunnion_cap_{idx}",
        )
        carriage.visual(
            Box((1.14, 0.080, 0.090)),
            origin=Origin(xyz=(-1.05, y * 0.62, 0.38), rpy=(0.0, -0.30, 0.0)),
            material=wood,
            name=f"trail_beam_{idx}",
        )

    for idx, x in enumerate((-0.82, 0.34)):
        carriage.visual(
            Box((0.12, 0.72, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.565)),
            material=wood,
            name=f"transom_{idx}",
        )
    carriage.visual(
        Box((0.20, 0.74, 0.10)),
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        material=wood,
        name="axle_block",
    )
    carriage.visual(
        Cylinder(radius=0.042, length=1.25),
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    for idx, y in enumerate((-0.52237, 0.545)):
        carriage.visual(
            Cylinder(radius=0.096, length=0.025),
            origin=Origin(xyz=(WHEEL_X, y, WHEEL_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"axle_collar_{idx}",
        )
    carriage.visual(
        Box((0.18, 0.48, 0.11)),
        origin=Origin(xyz=(-1.58, 0.0, 0.205)),
        material=dark_wood,
        name="trail_spade",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(-1.675, 0.0, 0.245), rpy=(0.0, pi / 2.0, 0.0)),
        material=iron,
        name="tow_ring",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_barrel_geometry(), "barrel_shell"),
        material=barrel_bronze,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.056, length=0.50),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=barrel_bronze,
        name="trunnions",
    )
    barrel.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.486, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bore_black,
        name="bore_shadow",
    )

    quoin = model.part("quoin")
    quoin.visual(
        mesh_from_geometry(_wedge_geometry(), "quoin_wedge"),
        origin=Origin(),
        material=dark_wood,
        name="wedge_body",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.430,
            0.100,
            rim=WheelRim(inner_radius=0.320, flange_height=0.018, flange_thickness=0.012),
            hub=WheelHub(radius=0.085, width=0.125, cap_style="domed"),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.022, window_radius=0.040),
            bore=WheelBore(style="round", diameter=0.110),
        ),
        "wooden_spoked_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.500,
            0.108,
            inner_radius=0.432,
            tread=TireTread(style="circumferential", depth=0.002, count=1),
            sidewall=TireSidewall(style="square", bulge=0.01),
            shoulder=TireShoulder(width=0.010, radius=0.002),
        ),
        "iron_wheel_tire",
    )
    for idx, y in enumerate((-WHEEL_Y, WHEEL_Y)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=wood,
            name="spoked_wheel",
        )
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=iron,
            name="iron_tire",
        )

        model.articulation(
            f"carriage_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(WHEEL_X, y, WHEEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=180.0, velocity=8.0),
        )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.7, lower=-0.08, upper=0.34),
    )
    model.articulation(
        "carriage_to_quoin",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=quoin,
        origin=Origin(xyz=(-0.48, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=-0.10, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    quoin = object_model.get_part("quoin")
    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    quoin_slide = object_model.get_articulation("carriage_to_quoin")

    ctx.check("barrel_elevates_on_revolute_trunnions", barrel_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("quoin_is_prismatic_wedge", quoin_slide.articulation_type == ArticulationType.PRISMATIC)
    for idx in (0, 1):
        wheel_joint = object_model.get_articulation(f"carriage_to_wheel_{idx}")
        ctx.check(
            f"wheel_{idx}_rotates_continuously",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        )

    ctx.expect_gap(
        quoin,
        carriage,
        axis="z",
        positive_elem="wedge_body",
        negative_elem="bed_plank",
        max_gap=0.002,
        max_penetration=0.001,
        name="quoin_wedge_sits_on_bed",
    )
    ctx.expect_within(
        quoin,
        carriage,
        axes="y",
        inner_elem="wedge_body",
        outer_elem="bed_plank",
        margin=0.002,
        name="quoin_wedge_stays_between_cheeks",
    )

    rest_quoin_pos = ctx.part_world_position(quoin)
    with ctx.pose({quoin_slide: quoin_slide.motion_limits.upper}):
        extended_quoin_pos = ctx.part_world_position(quoin)
    ctx.check(
        "quoin_slides_forward_along_bed",
        rest_quoin_pos is not None
        and extended_quoin_pos is not None
        and extended_quoin_pos[0] > rest_quoin_pos[0] + 0.15,
        details=f"rest={rest_quoin_pos}, extended={extended_quoin_pos}",
    )

    with ctx.pose({barrel_joint: 0.0}):
        rest_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    with ctx.pose({barrel_joint: barrel_joint.motion_limits.upper}):
        raised_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    ctx.check(
        "barrel_muzzle_raises_at_upper_elevation",
        rest_barrel_aabb is not None
        and raised_barrel_aabb is not None
        and raised_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.12,
        details=f"rest={rest_barrel_aabb}, raised={raised_barrel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
