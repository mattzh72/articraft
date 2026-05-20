from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


THREAD_PITCH = 0.009
THREAD_TURNS = 1.85
THREAD_TRAVEL = THREAD_PITCH * 2.25
CAP_SEAT_Z = 0.248


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _revolved_profile(
    profile: list[tuple[float, float]],
    *,
    segments: int = 96,
    radial_mod=None,
) -> MeshGeometry:
    """Revolve an R/Z polyline into one connected visual surface."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for radius, z in profile:
        ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * pi * i / segments
            r = radial_mod(radius, z, theta) if radial_mod else radius
            ring.append(geom.add_vertex(r * cos(theta), r * sin(theta), z))
        rings.append(ring)

    for lower, upper in zip(rings, rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            _add_quad(geom, lower[i], lower[j], upper[j], upper[i])
    return geom


def _bottle_shell() -> MeshGeometry:
    # One thin-walled blow-molded PET shell: stiff base, simple label waist,
    # sloped shoulders, support ledge, and an open drinking mouth.
    outer = [
        (0.0010, 0.0020),
        (0.0180, 0.0006),
        (0.0300, 0.0035),
        (0.0340, 0.0130),
        (0.0340, 0.0580),
        (0.0323, 0.0660),
        (0.0340, 0.0740),
        (0.0340, 0.1220),
        (0.0324, 0.1310),
        (0.0340, 0.1390),
        (0.0340, 0.1530),
        (0.0310, 0.1720),
        (0.0250, 0.1900),
        (0.0175, 0.2070),
        (0.0148, 0.2140),
        (0.0148, 0.2180),
        (0.0120, 0.2190),
        (0.0120, 0.2460),
        (0.0132, 0.2485),
        (0.0132, 0.2510),
    ]
    inner = [
        (0.0010, 0.0060),
        (0.0120, 0.0060),
        (0.0265, 0.0080),
        (0.0310, 0.0160),
        (0.0310, 0.1510),
        (0.0280, 0.1690),
        (0.0220, 0.1870),
        (0.0150, 0.2060),
        (0.0100, 0.2190),
        (0.0095, 0.2510),
    ]

    geom = MeshGeometry()
    outer_geom = _revolved_profile(outer, segments=112)
    inner_geom = _revolved_profile(inner, segments=112)
    geom.merge(outer_geom)

    offset = len(geom.vertices)
    geom.merge(inner_geom)
    segments = 112

    # Close the top lip and the heavy base by connecting the shell surfaces.
    outer_first = list(range(0, segments))
    outer_last = list(range((len(outer) - 1) * segments, len(outer) * segments))
    inner_first = list(range(offset, offset + segments))
    inner_last = list(
        range(offset + (len(inner) - 1) * segments, offset + len(inner) * segments)
    )
    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(geom, outer_last[i], outer_last[j], inner_last[j], inner_last[i])
        _add_quad(geom, inner_first[i], inner_first[j], outer_first[j], outer_first[i])
    return geom


def _cap_shell() -> MeshGeometry:
    # Single molded cap section: top disk, skirt, and inner plug seal are one
    # piece. A sinusoidal outer radius creates molded grip ribs without adding
    # separate glued-on or floating detail pieces.
    profile = [
        (0.0010, 0.0120),
        (0.0185, 0.0120),
        (0.0185, -0.0310),
        (0.0154, -0.0310),
        (0.0154, 0.0050),
        (0.0085, 0.0050),
        (0.0085, -0.0060),
        (0.0070, -0.0060),
        (0.0070, 0.0050),
        (0.0010, 0.0050),
        (0.0010, 0.0120),
    ]

    def ribbed_radius(radius: float, z: float, theta: float) -> float:
        if radius < 0.0180 or z > 0.010:
            return radius
        wave = max(0.0, cos(36.0 * theta)) ** 4
        return radius + 0.00085 * wave

    return _revolved_profile(profile, segments=144, radial_mod=ribbed_radius)


def _liner_seal() -> MeshGeometry:
    # Small compliant annular liner under the cap top. It is the only intentional
    # compressed interface; the helical threads themselves keep a visible radial
    # clearance.
    profile = [
        (0.0090, 0.0026),
        (0.0138, 0.0026),
        (0.0138, 0.0052),
        (0.0090, 0.0052),
        (0.0090, 0.0026),
    ]
    return _revolved_profile(profile, segments=96)


def _helical_thread(
    *,
    r_inner: float,
    r_outer: float,
    z_start: float,
    pitch: float = THREAD_PITCH,
    turns: float = THREAD_TURNS,
    width: float = 0.0027,
    phase: float = 0.0,
    segments_per_turn: int = 36,
) -> MeshGeometry:
    geom = MeshGeometry()
    count = max(2, int(turns * segments_per_turn) + 1)
    sections: list[tuple[int, int, int, int]] = []
    for i in range(count):
        t = i / (count - 1)
        theta = phase + 2.0 * pi * turns * t
        z = z_start + pitch * turns * t
        verts = []
        for r, dz in (
            (r_inner, -0.5 * width),
            (r_outer, -0.5 * width),
            (r_outer, 0.5 * width),
            (r_inner, 0.5 * width),
        ):
            verts.append(geom.add_vertex(r * cos(theta), r * sin(theta), z + dz))
        sections.append(tuple(verts))  # type: ignore[arg-type]

    for a, b in zip(sections, sections[1:]):
        for k in range(4):
            _add_quad(geom, a[k], b[k], b[(k + 1) % 4], a[(k + 1) % 4])

    # End faces make the ridge read as a molded solid instead of a surface strip.
    a = sections[0]
    b = sections[-1]
    _add_quad(geom, a[0], a[1], a[2], a[3])
    _add_quad(geom, b[3], b[2], b[1], b[0])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_screwcap_bottle")

    pet = model.material("clear_pet", rgba=(0.72, 0.88, 1.0, 0.38))
    pet_edge = model.material("thick_clear_pet", rgba=(0.58, 0.78, 1.0, 0.55))
    cap_blue = model.material("molded_blue_pp", rgba=(0.05, 0.28, 0.82, 0.82))
    cap_thread = model.material("cap_thread_highlight", rgba=(0.70, 0.86, 1.0, 0.90))
    liner = model.material("soft_white_liner", rgba=(0.92, 0.96, 1.0, 0.95))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_shell(), "bottle_shell"),
        material=pet,
        name="bottle_shell",
    )
    bottle.visual(
        mesh_from_geometry(
            _helical_thread(
                r_inner=0.0116,
                r_outer=0.0142,
                z_start=0.2250,
                width=0.0029,
            ),
            "neck_thread",
        ),
        material=pet_edge,
        name="neck_thread",
    )

    thread_drive = model.part("thread_drive")

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_cap_shell(), "cap_shell"),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_geometry(_liner_seal(), "liner_seal"),
        material=liner,
        name="liner_seal",
    )
    cap.visual(
        mesh_from_geometry(
            _helical_thread(
                r_inner=0.0148,
                r_outer=0.0159,
                z_start=0.2250 - CAP_SEAT_Z,
                width=0.0026,
            ),
            "female_thread",
        ),
        material=cap_thread,
        name="female_thread",
    )

    turn = model.articulation(
        "neck_to_cap",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=thread_drive,
        origin=Origin(xyz=(0.0, 0.0, CAP_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5, velocity=4.0, lower=0.0, upper=2.25 * 2.0 * pi
        ),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=thread_drive,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.06, lower=0.0, upper=THREAD_TRAVEL
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    turn = object_model.get_articulation("neck_to_cap")
    lift = object_model.get_articulation("cap_lift")

    ctx.allow_overlap(
        cap,
        bottle,
        elem_a="liner_seal",
        elem_b="bottle_shell",
        reason=(
            "The soft cap liner is intentionally shown slightly compressed "
            "against the bottle mouth; the molded helical threads remain clear."
        ),
    )

    ctx.expect_origin_distance(
        bottle,
        cap,
        axes="xy",
        max_dist=0.001,
        name="cap stays coaxial with molded neck",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="female_thread",
        elem_b="neck_thread",
        min_overlap=0.012,
        name="female and male helical threads are axially engaged",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="liner_seal",
        negative_elem="bottle_shell",
        max_penetration=0.0008,
        max_gap=0.0002,
        name="liner is lightly compressed on the mouth rim",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({turn: 2.0 * pi, lift: THREAD_PITCH}):
        one_turn_pos = ctx.part_world_position(cap)
        ctx.expect_origin_distance(
            bottle,
            cap,
            axes="xy",
            max_dist=0.001,
            name="one turn remains coaxial",
        )

    ctx.check(
        "cap rises by one thread pitch per turn",
        rest_pos is not None
        and one_turn_pos is not None
        and abs((one_turn_pos[2] - rest_pos[2]) - THREAD_PITCH) < 0.0015,
        details=f"rest={rest_pos}, one_turn={one_turn_pos}, pitch={THREAD_PITCH}",
    )

    return ctx.report()


object_model = build_object_model()
