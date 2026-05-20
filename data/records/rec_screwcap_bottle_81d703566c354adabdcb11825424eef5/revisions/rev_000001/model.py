from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _bottle_shell_geometry() -> MeshGeometry:
    """Thin PET bottle with rounded shoulders, lip, and raised neck thread bands."""

    outer_profile = [
        (0.0000, 0.000),
        (0.0200, 0.000),
        (0.0305, 0.004),
        (0.0330, 0.010),
        (0.0340, 0.028),
        (0.0335, 0.070),
        (0.0340, 0.120),
        (0.0320, 0.145),
        (0.0260, 0.162),
        (0.0190, 0.174),
        (0.0120, 0.181),
        (0.0120, 0.184),
        # Thread-like raised bands on the neck.  The cap covers the upper turns
        # while the lower thread remains visible below the skirt.
        (0.0137, 0.185),
        (0.0137, 0.188),
        (0.0118, 0.190),
        (0.0118, 0.193),
        (0.0137, 0.194),
        (0.0137, 0.197),
        (0.0118, 0.199),
        (0.0118, 0.202),
        (0.0136, 0.203),
        (0.0136, 0.206),
        (0.0118, 0.208),
        (0.0118, 0.219),
        (0.0142, 0.221),
        (0.0142, 0.224),
    ]
    inner_profile = [
        (0.0091, 0.224),
        (0.0091, 0.191),
        (0.0145, 0.181),
        (0.0220, 0.170),
        (0.0285, 0.148),
        (0.0300, 0.118),
        (0.0295, 0.040),
        (0.0270, 0.014),
        (0.0060, 0.009),
        (0.0000, 0.009),
    ]
    return LatheGeometry(outer_profile + inner_profile, segments=96, closed=True)


def _ribbed_cap_geometry(
    *,
    outer_radius: float = 0.0180,
    inner_radius: float = 0.0150,
    height: float = 0.0370,
    top_thickness: float = 0.0060,
    rib_count: int = 36,
    radial_segments: int = 144,
) -> MeshGeometry:
    """Open-bottom screw closure with a closed top and grippy vertical ribs."""

    geom = MeshGeometry()
    z_levels = (0.0000, 0.0030, height - 0.0045, height)
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []
    underside_z = height - top_thickness

    def ribbed_radius(theta: float, z: float) -> float:
        # Squared-sine rib profile creates narrow proud ridges with smooth valleys.
        ridge = math.sin(0.5 * rib_count * theta) ** 2
        edge_softening = 0.78 + 0.22 * min(1.0, max(0.0, z / 0.004, (height - z) / 0.004))
        return outer_radius + 0.00115 * ridge * edge_softening

    for z in z_levels:
        ring = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            r = ribbed_radius(theta, z)
            ring.append(geom.add_vertex(r * math.cos(theta), r * math.sin(theta), z))
        outer_rings.append(ring)

    for z in (0.0000, underside_z):
        ring = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            ring.append(geom.add_vertex(inner_radius * math.cos(theta), inner_radius * math.sin(theta), z))
        inner_rings.append(ring)

    top_center = geom.add_vertex(0.0, 0.0, height)
    underside_center = geom.add_vertex(0.0, 0.0, underside_z)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    # Outer ribbed sidewall.
    for ring_a, ring_b in zip(outer_rings[:-1], outer_rings[1:]):
        for i in range(radial_segments):
            j = (i + 1) % radial_segments
            quad(ring_a[i], ring_a[j], ring_b[j], ring_b[i])

    # Inner smooth bore wall, open to the bottom.
    inner_bottom, inner_top = inner_rings
    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        quad(inner_top[i], inner_top[j], inner_bottom[j], inner_bottom[i])

    # Bottom annular rim surface.
    outer_bottom = outer_rings[0]
    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        quad(inner_bottom[i], inner_bottom[j], outer_bottom[j], outer_bottom[i])

    # Flat top disk and the underside of the closure top.
    outer_top = outer_rings[-1]
    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        geom.add_face(top_center, outer_top[i], outer_top[j])
        geom.add_face(underside_center, inner_top[j], inner_top[i])

    # Small internal thread lands inside the closure.  They are shallow enough to
    # clear the bottle neck but make the cap read as a screw closure.
    for zc in (0.010, 0.017, 0.024):
        lower = []
        crest = []
        upper = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            lower.append(
                geom.add_vertex(inner_radius * math.cos(theta), inner_radius * math.sin(theta), zc - 0.0015)
            )
            crest_r = inner_radius - 0.0009
            crest.append(geom.add_vertex(crest_r * math.cos(theta), crest_r * math.sin(theta), zc))
            upper.append(
                geom.add_vertex(inner_radius * math.cos(theta), inner_radius * math.sin(theta), zc + 0.0015)
            )
        for i in range(radial_segments):
            j = (i + 1) % radial_segments
            quad(lower[i], lower[j], crest[j], crest[i])
            quad(crest[i], crest[j], upper[j], upper[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    pet_clear = model.material("clear_pet", rgba=(0.62, 0.86, 1.0, 0.46))
    pet_edge = model.material("pet_edge_blue", rgba=(0.42, 0.70, 0.95, 0.62))
    white_plastic = model.material("white_plastic", rgba=(0.93, 0.94, 0.92, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        _mesh("bottle_shell", _bottle_shell_geometry()),
        material=pet_clear,
        name="bottle_shell",
    )
    # Subtle blue-tinted thickened rings at the base and mouth reinforce that the
    # bottle is a hollow transparent shell rather than a solid cylinder.
    bottle.visual(
        _mesh("base_foot_ring", LatheGeometry([(0.0290, 0.003), (0.0345, 0.004), (0.0345, 0.007), (0.0290, 0.007)], segments=96)),
        material=pet_edge,
        name="base_foot_ring",
    )
    bottle.visual(
        _mesh("mouth_lip", LatheGeometry([(0.0090, 0.221), (0.0147, 0.221), (0.0147, 0.225), (0.0090, 0.225)], segments=96)),
        material=pet_edge,
        name="mouth_lip",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.072, 0.072, 0.225)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    cap = model.part("cap")
    cap.visual(
        _mesh("cap_shell", _ribbed_cap_geometry()),
        material=white_plastic,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.037)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("cap_spin")

    ctx.check(
        "cap uses a continuous screw rotation joint",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={joint.articulation_type}",
    )
    ctx.check(
        "cap rotation axis is the bottle neck centerline",
        tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.0005,
        name="cap origin stays on the bottle midline",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        min_overlap=0.020,
        elem_a="cap_shell",
        elem_b="bottle_shell",
        name="cap skirt covers the threaded neck height",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({joint: 1.75}):
        spun_pos = ctx.part_world_position(cap)
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=0.0005,
            name="rotated cap remains centered on body midline",
        )

    ctx.check(
        "continuous spin does not translate the cap",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
