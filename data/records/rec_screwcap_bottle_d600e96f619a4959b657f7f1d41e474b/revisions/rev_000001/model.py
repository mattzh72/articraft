from __future__ import annotations

import math

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


def _lerp_radius(profile: list[tuple[float, float]], z: float) -> float:
    """Piecewise-linear radius lookup for a revolved bottle profile."""
    if z <= profile[0][0]:
        return profile[0][1]
    for (z0, r0), (z1, r1) in zip(profile, profile[1:]):
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return profile[-1][1]


def _threaded_radius(base_radius: float, z: float, theta: float) -> float:
    """Add a shallow helical raised thread on the neck finish."""
    z0 = 0.126
    z1 = 0.150
    if z < z0 or z > z1:
        return base_radius
    pitch = 0.008
    # One raised ridge winds around the neck roughly three times.
    phase = ((z - z0) / pitch - theta / (2.0 * math.pi)) % 1.0
    distance = abs(phase - 0.5)
    ridge = max(0.0, 1.0 - distance / 0.18) ** 2
    return base_radius + 0.0048 * ridge


def _add_closed_ring_faces(mesh: MeshGeometry, a: list[int], b: list[int]) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        mesh.add_face(a[i], a[j], b[j])
        mesh.add_face(a[i], b[j], b[i])


def _build_bottle_mesh(segments: int = 128) -> MeshGeometry:
    """Squat hollow bottle body with a raised threaded neck."""
    mesh = MeshGeometry()

    # z, radius.  The body is intentionally short and wide with a heavy shoulder.
    profile = [
        (0.000, 0.000),
        (0.000, 0.070),
        (0.006, 0.092),
        (0.018, 0.104),
        (0.074, 0.105),
        (0.090, 0.101),
        (0.105, 0.086),
        (0.116, 0.064),
        (0.124, 0.050),
        (0.150, 0.047),
        (0.160, 0.044),
    ]
    z_values = {
        0.000,
        0.006,
        0.018,
        0.074,
        0.090,
        0.105,
        0.116,
        0.124,
        0.150,
        0.160,
    }
    for i in range(31):
        z_values.add(0.124 + (0.030 * i / 30.0))
    z_sorted = sorted(z_values)

    rings: list[list[int]] = []
    for z in z_sorted:
        ring: list[int] = []
        base_radius = _lerp_radius(profile, z)
        for s in range(segments):
            theta = 2.0 * math.pi * s / segments
            radius = _threaded_radius(base_radius, z, theta)
            ring.append(mesh.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        rings.append(ring)

    for lower, upper in zip(rings, rings[1:]):
        _add_closed_ring_faces(mesh, lower, upper)

    # Visible mouth and inner bore make the bottle read as hollow, even though the
    # screw cap hides most of the opening.
    inner_radius = 0.028
    inner_top: list[int] = []
    inner_low: list[int] = []
    for s in range(segments):
        theta = 2.0 * math.pi * s / segments
        x = inner_radius * math.cos(theta)
        y = inner_radius * math.sin(theta)
        inner_top.append(mesh.add_vertex(x, y, 0.160))
        inner_low.append(mesh.add_vertex(x, y, 0.126))

    outer_top = rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        # Annular top lip.
        mesh.add_face(outer_top[i], inner_top[i], inner_top[j])
        mesh.add_face(outer_top[i], inner_top[j], outer_top[j])
        # Inner bore wall.
        mesh.add_face(inner_top[i], inner_low[j], inner_low[i])
        mesh.add_face(inner_top[i], inner_top[j], inner_low[j])

    center = mesh.add_vertex(0.0, 0.0, 0.126)
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(center, inner_low[i], inner_low[j])

    return mesh


def _cap_outer_radius(theta: float) -> float:
    ridges = 36
    rib = max(0.0, math.cos(ridges * theta)) ** 3
    return 0.083 + 0.0050 * rib


def _build_cap_mesh(segments: int = 144) -> MeshGeometry:
    """One-piece hollow screw cap with a ribbed outside skirt."""
    mesh = MeshGeometry()

    outer_zs = [0.000, 0.004, 0.044, 0.052]
    outer_scale = [0.985, 1.000, 1.000, 0.955]
    outer_rings: list[list[int]] = []
    for z, scale in zip(outer_zs, outer_scale):
        ring: list[int] = []
        for s in range(segments):
            theta = 2.0 * math.pi * s / segments
            radius = _cap_outer_radius(theta) * scale
            ring.append(mesh.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        outer_rings.append(ring)

    for lower, upper in zip(outer_rings, outer_rings[1:]):
        _add_closed_ring_faces(mesh, lower, upper)

    # The inner wall is slightly smaller than the tallest bottle thread ridge;
    # that tiny local interference represents engaged screw threads and gives
    # the cap a physical support path rather than floating above the finish.
    inner_radius = 0.0505
    inner_bottom: list[int] = []
    inner_roof: list[int] = []
    top_inner: list[int] = []
    for s in range(segments):
        theta = 2.0 * math.pi * s / segments
        inner_bottom.append(
            mesh.add_vertex(inner_radius * math.cos(theta), inner_radius * math.sin(theta), 0.000)
        )
        inner_roof.append(
            mesh.add_vertex(inner_radius * math.cos(theta), inner_radius * math.sin(theta), 0.044)
        )
        top_inner.append(
            mesh.add_vertex(0.055 * math.cos(theta), 0.055 * math.sin(theta), 0.052)
        )

    # Bottom annular rim connects the hollow inner wall to the outside skirt.
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(outer_rings[0][i], outer_rings[0][j], inner_bottom[j])
        mesh.add_face(outer_rings[0][i], inner_bottom[j], inner_bottom[i])
        mesh.add_face(inner_bottom[i], inner_roof[j], inner_bottom[j])
        mesh.add_face(inner_bottom[i], inner_roof[i], inner_roof[j])

    # Top face: a broad disk with a slightly softened edge, visually dominating
    # the front/top of the squat bottle.
    top_center = mesh.add_vertex(0.0, 0.0, 0.052)
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(outer_rings[-1][i], top_inner[i], top_inner[j])
        mesh.add_face(outer_rings[-1][i], top_inner[j], outer_rings[-1][j])
        mesh.add_face(top_center, top_inner[j], top_inner[i])

    # Hidden underside of the cap roof.
    roof_center = mesh.add_vertex(0.0, 0.0, 0.044)
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(roof_center, inner_roof[i], inner_roof[j])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_wide_screw_cap_bottle")

    translucent_glass = model.material("smoky_clear_plastic", rgba=(0.62, 0.80, 0.86, 0.48))
    cap_red = model.material("matte_red_cap", rgba=(0.78, 0.08, 0.05, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_mesh(), "squat_threaded_bottle"),
        material=translucent_glass,
        name="threaded_body",
    )
    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_build_cap_mesh(), "wide_ribbed_screw_cap"),
        material=cap_red,
        name="ribbed_closure",
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        # The child frame is the bottom center of the cap skirt, coaxial with the
        # threaded neck.  The cap mesh extends upward from this frame.
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("cap_spin")

    ctx.allow_overlap(
        cap,
        bottle,
        elem_a="ribbed_closure",
        elem_b="threaded_body",
        reason=(
            "The cap's simplified inner thread intentionally bites slightly into "
            "the raised bottle-neck thread to represent screw engagement."
        ),
    )

    ctx.check(
        "cap uses a continuous screw rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={joint.articulation_type}",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.10,
        elem_a="ribbed_closure",
        elem_b="threaded_body",
        name="wide cap is coaxial over the neck",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        min_overlap=0.008,
        elem_a="ribbed_closure",
        elem_b="threaded_body",
        name="cap skirt wraps the threaded neck height",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        max_penetration=0.014,
        positive_elem="ribbed_closure",
        negative_elem="threaded_body",
        name="thread engagement remains shallow in height",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({joint: math.pi * 1.25}):
        spun_pos = ctx.part_world_position(cap)
        ctx.expect_overlap(
            cap,
            bottle,
            axes="xy",
            min_overlap=0.10,
            elem_a="ribbed_closure",
            elem_b="threaded_body",
            name="rotated cap remains on bottle axis",
        )

    ctx.check(
        "continuous spin does not translate the cap",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(rest_pos[i] - spun_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
