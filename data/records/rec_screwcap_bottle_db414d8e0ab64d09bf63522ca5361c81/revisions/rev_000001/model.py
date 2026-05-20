from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SEGMENTS = 96
NECK_X = 0.028
CAP_BOTTOM_Z = 0.261


def _superellipse_loop(
    width: float,
    depth: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    exponent: float = 2.6,
    segments: int = SEGMENTS,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    power = 2.0 / exponent
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = cx + 0.5 * width * math.copysign(abs(c) ** power, c)
        y = cy + 0.5 * depth * math.copysign(abs(s) ** power, s)
        points.append((x, y, z))
    return points


def _circle_loop(
    radius: float,
    z: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = SEGMENTS,
) -> list[tuple[float, float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
            z,
        )
        for i in range(segments)
    ]


def _add_loop(geom: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in loop]


def _bridge(geom: MeshGeometry, a: list[int], b: list[int], *, reverse: bool = False) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        if reverse:
            geom.add_face(a[i], b[j], a[j])
            geom.add_face(a[i], b[i], b[j])
        else:
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], *, upward: bool) -> None:
    cx = sum(geom.vertices[i][0] for i in loop) / len(loop)
    cy = sum(geom.vertices[i][1] for i in loop) / len(loop)
    cz = sum(geom.vertices[i][2] for i in loop) / len(loop)
    center = geom.add_vertex(cx, cy, cz)
    n = len(loop)
    for i in range(n):
        j = (i + 1) % n
        if upward:
            geom.add_face(center, loop[i], loop[j])
        else:
            geom.add_face(center, loop[j], loop[i])


def _bottle_shell_geometry() -> MeshGeometry:
    """Thin-walled, off-center bottle body with an open mouth at the neck."""
    geom = MeshGeometry()

    outer_specs = [
        (0.000, 0.136, 0.084, -0.018, 2.7),
        (0.014, 0.156, 0.096, -0.018, 3.0),
        (0.060, 0.162, 0.098, -0.016, 3.0),
        (0.145, 0.146, 0.088, -0.011, 2.8),
        (0.186, 0.118, 0.073, -0.002, 2.6),
        (0.215, 0.075, 0.055, 0.015, 2.2),
        (0.228, 0.038, 0.038, NECK_X, 2.0),
    ]
    inner_specs = [
        (0.008, 0.112, 0.062, -0.018, 2.7),
        (0.024, 0.130, 0.071, -0.018, 3.0),
        (0.132, 0.124, 0.068, -0.012, 2.9),
        (0.178, 0.096, 0.052, -0.003, 2.5),
        (0.208, 0.051, 0.033, 0.016, 2.2),
        (0.228, 0.018, 0.018, NECK_X, 2.0),
    ]

    outer = [
        _add_loop(
            geom,
            _superellipse_loop(width, depth, z, cx=cx, exponent=exp),
        )
        for z, width, depth, cx, exp in outer_specs
    ]
    inner = [
        _add_loop(
            geom,
            _superellipse_loop(width, depth, z, cx=cx, exponent=exp),
        )
        for z, width, depth, cx, exp in inner_specs
    ]

    for low, high in zip(outer, outer[1:]):
        _bridge(geom, low, high)
    for low, high in zip(inner, inner[1:]):
        _bridge(geom, low, high, reverse=True)

    _cap_loop(geom, outer[0], upward=False)
    _cap_loop(geom, inner[0], upward=True)
    _bridge(geom, outer[-1], inner[-1], reverse=True)
    return geom


def _hollow_cylinder_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    cx: float = 0.0,
    segments: int = SEGMENTS,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_bottom = _add_loop(geom, _circle_loop(outer_radius, z_min, cx=cx, segments=segments))
    outer_top = _add_loop(geom, _circle_loop(outer_radius, z_max, cx=cx, segments=segments))
    inner_bottom = _add_loop(geom, _circle_loop(inner_radius, z_min, cx=cx, segments=segments))
    inner_top = _add_loop(geom, _circle_loop(inner_radius, z_max, cx=cx, segments=segments))
    _bridge(geom, outer_bottom, outer_top)
    _bridge(geom, inner_bottom, inner_top, reverse=True)
    _bridge(geom, outer_top, inner_top, reverse=True)
    _bridge(geom, outer_bottom, inner_bottom)
    return geom


def _thread_geometry(
    *,
    cx: float,
    z_start: float,
    z_end: float,
    root_radius: float,
    crest_radius: float,
    turns: float,
    band_height: float,
    samples: int = 150,
) -> MeshGeometry:
    geom = MeshGeometry()
    sections: list[list[int]] = []
    for i in range(samples):
        u = i / (samples - 1)
        angle = 2.0 * math.pi * turns * u
        z = z_start + (z_end - z_start) * u
        ca = math.cos(angle)
        sa = math.sin(angle)
        verts = [
            (cx + root_radius * ca, root_radius * sa, z - band_height * 0.5),
            (cx + crest_radius * ca, crest_radius * sa, z - band_height * 0.5),
            (cx + crest_radius * ca, crest_radius * sa, z + band_height * 0.5),
            (cx + root_radius * ca, root_radius * sa, z + band_height * 0.5),
        ]
        sections.append([geom.add_vertex(x, y, zz) for x, y, zz in verts])

    for a, b in zip(sections, sections[1:]):
        for i in range(4):
            j = (i + 1) % 4
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])

    # Close the two cut ends of the helical land.
    first = sections[0]
    last = sections[-1]
    geom.add_face(first[0], first[2], first[1])
    geom.add_face(first[0], first[3], first[2])
    geom.add_face(last[0], last[1], last[2])
    geom.add_face(last[0], last[2], last[3])
    return geom


def _ribbed_cap_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    segments = SEGMENTS
    rib_count = 40
    base_radius = 0.0248
    rib_depth = 0.0014
    inner_radius = 0.0205
    height = 0.056
    underside_z = 0.044

    def outer_loop(z: float) -> list[tuple[float, float, float]]:
        loop: list[tuple[float, float, float]] = []
        for i in range(segments):
            t = 2.0 * math.pi * i / segments
            # Fine vertical flutes/knurls, rounded by the cosine rather than sharp teeth.
            r = base_radius + rib_depth * (0.5 + 0.5 * math.cos(rib_count * t))
            loop.append((r * math.cos(t), r * math.sin(t), z))
        return loop

    outer_bottom = _add_loop(geom, outer_loop(0.000))
    outer_lower = _add_loop(geom, outer_loop(0.006))
    outer_underside = _add_loop(geom, outer_loop(underside_z))
    outer_top = _add_loop(geom, outer_loop(height))
    inner_bottom = _add_loop(geom, _circle_loop(inner_radius, 0.000, segments=segments))
    inner_top = _add_loop(geom, _circle_loop(inner_radius, underside_z, segments=segments))

    _bridge(geom, outer_bottom, outer_lower)
    _bridge(geom, outer_lower, outer_underside)
    _bridge(geom, outer_underside, outer_top)
    _bridge(geom, inner_bottom, inner_top, reverse=True)
    _bridge(geom, outer_bottom, inner_bottom)
    _bridge(geom, outer_underside, inner_top, reverse=True)
    _cap_loop(geom, outer_top, upward=True)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_screw_cap_bottle")

    bottle_plastic = Material("frosted_bottle_plastic", rgba=(0.63, 0.86, 0.96, 0.58))
    neck_plastic = Material("clear_threaded_plastic", rgba=(0.78, 0.93, 1.0, 0.72))
    cap_plastic = Material("matte_white_cap", rgba=(0.95, 0.93, 0.88, 1.0))
    mark_material = Material("blue_orientation_mark", rgba=(0.08, 0.16, 0.42, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_shell_geometry(), "asymmetric_hollow_bottle_shell"),
        material=bottle_plastic,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.0152,
                inner_radius=0.0088,
                z_min=0.222,
                z_max=0.286,
                cx=NECK_X,
            ),
            "hollow_bottle_neck",
        ),
        material=neck_plastic,
        name="neck_wall",
    )
    bottle.visual(
        mesh_from_geometry(
            _thread_geometry(
                cx=NECK_X,
                z_start=0.234,
                z_end=0.257,
                root_radius=0.0146,
                crest_radius=0.0183,
                turns=1.85,
                band_height=0.0026,
            ),
            "raised_neck_thread",
        ),
        material=neck_plastic,
        name="neck_thread",
    )
    bottle.visual(
        mesh_from_geometry(
            _hollow_cylinder_geometry(
                outer_radius=0.0235,
                inner_radius=0.0150,
                z_min=0.258,
                z_max=CAP_BOTTOM_Z,
                cx=NECK_X,
            ),
            "neck_support_bead",
        ),
        material=neck_plastic,
        name="neck_bead",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_ribbed_cap_geometry(), "hollow_knurled_screw_cap"),
        material=cap_plastic,
        name="cap_shell",
    )
    cap.visual(
        mesh_from_geometry(
            _thread_geometry(
                cx=0.0,
                z_start=0.007,
                z_end=0.030,
                root_radius=0.0208,
                crest_radius=0.0189,
                turns=1.65,
                band_height=0.0020,
                samples=132,
            ),
            "cap_inner_thread",
        ),
        material=cap_plastic,
        name="inner_thread",
    )
    cap.visual(
        Box((0.020, 0.004, 0.0014)),
        origin=Origin(xyz=(0.006, 0.0, 0.0567)),
        material=mark_material,
        name="top_mark",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(NECK_X, 0.0, CAP_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    spin = object_model.get_articulation("neck_to_cap")

    ctx.check(
        "cap has continuous screw rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "cap spins about the vertical neck axis",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_wall",
        outer_elem="cap_shell",
        margin=0.001,
        name="neck sits inside the hollow cap footprint",
    )
    ctx.expect_overlap(
        bottle,
        cap,
        axes="z",
        elem_a="neck_wall",
        elem_b="cap_shell",
        min_overlap=0.018,
        name="cap skirt overlaps the neck height",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="neck_thread",
        min_gap=0.001,
        max_gap=0.006,
        name="cap lower edge clears the exposed external thread",
    )
    ctx.expect_contact(
        cap,
        bottle,
        elem_a="cap_shell",
        elem_b="neck_bead",
        contact_tol=0.0002,
        name="cap skirt seats on the neck support bead",
    )

    rest_position = ctx.part_world_position(cap)
    with ctx.pose({spin: 1.75}):
        turned_position = ctx.part_world_position(cap)
        ctx.expect_within(
            bottle,
            cap,
            axes="xy",
            inner_elem="neck_wall",
            outer_elem="cap_shell",
            margin=0.001,
            name="rotated cap remains concentric around neck",
        )
    ctx.check(
        "continuous rotation does not translate the cap",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6
        and abs(rest_position[2] - turned_position[2]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
