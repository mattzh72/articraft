from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SEGMENTS = 96


def _add_ring(mesh: MeshGeometry, radius: float, z: float, *, segments: int = SEGMENTS) -> list[int]:
    return [
        mesh.add_vertex(radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments), z)
        for i in range(segments)
    ]


def _quad_strip(mesh: MeshGeometry, lower: list[int], upper: list[int], *, reverse: bool = False) -> None:
    n = len(lower)
    for i in range(n):
        a = lower[i]
        b = lower[(i + 1) % n]
        c = upper[(i + 1) % n]
        d = upper[i]
        if reverse:
            mesh.add_face(a, c, b)
            mesh.add_face(a, d, c)
        else:
            mesh.add_face(a, b, c)
            mesh.add_face(a, c, d)


def _disk(mesh: MeshGeometry, ring: list[int], z: float, *, upward: bool) -> None:
    center = mesh.add_vertex(0.0, 0.0, z)
    n = len(ring)
    for i in range(n):
        a = ring[i]
        b = ring[(i + 1) % n]
        if upward:
            mesh.add_face(center, a, b)
        else:
            mesh.add_face(center, b, a)


def _lathe_surface(mesh: MeshGeometry, profile: list[tuple[float, float]], *, reverse: bool = False) -> list[list[int]]:
    rings = [_add_ring(mesh, radius, z) for radius, z in profile]
    for a, b in zip(rings, rings[1:]):
        _quad_strip(mesh, a, b, reverse=reverse)
    return rings


def _add_helical_ridge(
    mesh: MeshGeometry,
    *,
    base_radius: float,
    crest_radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    width: float,
    inward: bool = False,
    samples_per_turn: int = 56,
) -> None:
    """Triangular screw-thread ridge following a helix around the Z axis."""

    samples = max(8, int(turns * samples_per_turn))
    rows: list[list[int]] = []
    for i in range(samples + 1):
        theta = 2.0 * math.pi * turns * i / samples
        z_mid = z_start + pitch * theta / (2.0 * math.pi)
        radii = (base_radius, crest_radius, base_radius)
        offsets = (-width * 0.5, 0.0, width * 0.5)
        row = []
        for radius, dz in zip(radii, offsets):
            row.append(mesh.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z_mid + dz))
        rows.append(row)

    for row_a, row_b in zip(rows, rows[1:]):
        for j in range(2):
            a = row_a[j]
            b = row_a[j + 1]
            c = row_b[j + 1]
            d = row_b[j]
            if inward:
                mesh.add_face(a, c, b)
                mesh.add_face(a, d, c)
            else:
                mesh.add_face(a, b, c)
                mesh.add_face(a, c, d)


def _bottle_mesh() -> MeshGeometry:
    mesh = MeshGeometry()

    outer_profile = [
        (0.021, 0.000),
        (0.031, 0.004),
        (0.037, 0.018),
        (0.039, 0.055),
        (0.039, 0.125),
        (0.037, 0.154),
        (0.031, 0.174),
        (0.022, 0.188),
        (0.0125, 0.198),
        (0.0125, 0.234),
    ]
    inner_profile = [
        (0.0070, 0.234),
        (0.0070, 0.201),
        (0.0140, 0.192),
        (0.0260, 0.174),
        (0.0340, 0.145),
        (0.0340, 0.026),
        (0.0250, 0.010),
    ]

    outer_rings = _lathe_surface(mesh, outer_profile)
    inner_rings = _lathe_surface(mesh, inner_profile, reverse=True)

    # Top lip and base floor make the bottle read as a thin-walled hollow container.
    _quad_strip(mesh, inner_rings[0], outer_rings[-1])
    _disk(mesh, outer_rings[0], outer_profile[0][1], upward=False)
    _disk(mesh, inner_rings[-1], inner_profile[-1][1], upward=True)

    # Finish rings and one raised helical screw thread on the neck.
    bead_outer = _lathe_surface(
        mesh,
        [
            (0.0126, 0.190),
            (0.0150, 0.192),
            (0.0150, 0.197),
            (0.0126, 0.199),
        ],
    )
    _quad_strip(mesh, bead_outer[0], bead_outer[-1])
    _add_helical_ridge(
        mesh,
        base_radius=0.0125,
        crest_radius=0.0146,
        z_start=0.202,
        pitch=0.0074,
        turns=3.0,
        width=0.0026,
    )
    return mesh


def _label_mesh() -> MeshGeometry:
    mesh = MeshGeometry()
    z0, z1 = 0.060, 0.113
    inner_r, outer_r = 0.0380, 0.0400
    outer = _lathe_surface(mesh, [(outer_r, z0), (outer_r, z1)])
    inner = _lathe_surface(mesh, [(inner_r, z1), (inner_r, z0)], reverse=True)
    _quad_strip(mesh, outer[0], inner[-1])
    _quad_strip(mesh, inner[0], outer[-1])
    return mesh


def _cap_mesh() -> MeshGeometry:
    mesh = MeshGeometry()

    outer_r = 0.0180
    rib_r = 0.0210
    inner_r = 0.0156
    plug_r = 0.0062
    gasket_r = 0.0132
    bottom_z = -0.022
    under_z = 0.000
    top_z = 0.014

    outer_bottom = _add_ring(mesh, outer_r, bottom_z)
    outer_top = _add_ring(mesh, outer_r, top_z)
    inner_bottom = _add_ring(mesh, inner_r, bottom_z)
    plug_bottom = _add_ring(mesh, plug_r, under_z)
    plug_top = _add_ring(mesh, plug_r, top_z)
    gasket_outer = _add_ring(mesh, gasket_r, under_z)

    # The visible closure is an open-bottom shell so the bottle neck can sit
    # inside the cap without the collision proxy becoming a solid plug.
    _quad_strip(mesh, outer_bottom, outer_top)
    _quad_strip(mesh, inner_bottom, outer_bottom, reverse=True)
    _quad_strip(mesh, plug_bottom, plug_top)
    _quad_strip(mesh, plug_top, outer_top)
    _disk(mesh, plug_top, top_z, upward=True)
    _quad_strip(mesh, plug_bottom, gasket_outer, reverse=True)

    # Tall grip flutes around the skirt are modeled as raised, connected pads.
    rib_count = 28
    rib_width = 0.075
    z_low = bottom_z + 0.003
    z_high = top_z - 0.003
    for k in range(rib_count):
        theta = 2.0 * math.pi * k / rib_count
        angles = (theta - rib_width * 0.5, theta + rib_width * 0.5)
        v = {}
        for label_r, radius in (("base", outer_r * 0.997), ("crest", rib_r)):
            for label_z, z in (("low", z_low), ("high", z_high)):
                for label_a, ang in (("a", angles[0]), ("b", angles[1])):
                    v[(label_r, label_z, label_a)] = mesh.add_vertex(radius * math.cos(ang), radius * math.sin(ang), z)
        # Outer face of rib.
        mesh.add_face(v[("crest", "low", "a")], v[("crest", "low", "b")], v[("crest", "high", "b")])
        mesh.add_face(v[("crest", "low", "a")], v[("crest", "high", "b")], v[("crest", "high", "a")])
        # Two side faces and the cap face where the rib blends into the skirt.
        for label_a in ("a", "b"):
            mesh.add_face(v[("base", "low", label_a)], v[("crest", "low", label_a)], v[("crest", "high", label_a)])
            mesh.add_face(v[("base", "low", label_a)], v[("crest", "high", label_a)], v[("base", "high", label_a)])
        mesh.add_face(v[("base", "low", "a")], v[("base", "high", "b")], v[("base", "low", "b")])
        mesh.add_face(v[("base", "low", "a")], v[("base", "high", "a")], v[("base", "high", "b")])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    glass = model.material("clear_aqua_plastic", rgba=(0.64, 0.92, 1.0, 0.38))
    label_mat = model.material("paper_label", rgba=(0.94, 0.95, 0.90, 1.0))
    cap_mat = model.material("blue_polypropylene", rgba=(0.05, 0.22, 0.72, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_mesh(), "hollow_bottle_shell"),
        material=glass,
        name="bottle_shell",
    )
    bottle.visual(
        mesh_from_geometry(_label_mesh(), "paper_label_sleeve"),
        material=label_mat,
        name="label_sleeve",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_cap_mesh(), "ribbed_screw_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cap_mat,
        name="cap_shell",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.234)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
        meta={"description": "Continuous screw-cap rotation axis lies inside the bottle neck, with no exposed axle."},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("neck_to_cap")

    ctx.check(
        "cap uses continuous neck-axis rotation",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_origin_gap(cap, bottle, axis="z", min_gap=0.230, max_gap=0.238, name="cap frame is tucked on the neck axis")
    ctx.expect_origin_distance(cap, bottle, axes="xy", max_dist=0.0005, name="cap is centered on bottle neck")
    ctx.expect_contact(cap, bottle, elem_a="cap_shell", elem_b="bottle_shell", contact_tol=0.0015, name="cap seals against the bottle lip")

    rest_position = ctx.part_world_position(cap)
    with ctx.pose({joint: math.pi * 1.5}):
        turned_position = ctx.part_world_position(cap)
        ctx.expect_origin_distance(cap, bottle, axes="xy", max_dist=0.0005, name="turned cap remains coaxial")
        ctx.expect_contact(cap, bottle, elem_a="cap_shell", elem_b="bottle_shell", contact_tol=0.0015, name="turned cap still seats on lip")

    ctx.check(
        "continuous joint rotates without translating the cap frame",
        rest_position is not None
        and turned_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, turned_position)) < 0.0005,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
