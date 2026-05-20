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


def _add_ring(
    mesh: MeshGeometry,
    radii: float | list[float],
    z: float,
    *,
    segments: int = SEGMENTS,
) -> list[int]:
    if isinstance(radii, (int, float)):
        radius_values = [float(radii)] * segments
    else:
        radius_values = radii
    ring: list[int] = []
    for i, radius in enumerate(radius_values):
        theta = 2.0 * math.pi * i / segments
        ring.append(
            mesh.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z)
        )
    return ring


def _connect_rings(mesh: MeshGeometry, lower: list[int], upper: list[int]) -> None:
    count = len(lower)
    for i in range(count):
        j = (i + 1) % count
        mesh.add_face(lower[i], upper[i], upper[j])
        mesh.add_face(lower[i], upper[j], lower[j])


def _add_disk(mesh: MeshGeometry, ring: list[int], z: float, *, flip: bool = False) -> None:
    center = mesh.add_vertex(0.0, 0.0, z)
    count = len(ring)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(center, ring[j], ring[i])
        else:
            mesh.add_face(center, ring[i], ring[j])


def _lathe_surface(profile: list[tuple[float, float]]) -> tuple[MeshGeometry, list[list[int]]]:
    mesh = MeshGeometry()
    rings = [_add_ring(mesh, radius, z) for radius, z in profile]
    for lower, upper in zip(rings, rings[1:]):
        _connect_rings(mesh, lower, upper)
    return mesh, rings


def _make_bottle_shell() -> MeshGeometry:
    # A thin-walled compact bottle body with a rounded base, cylindrical belly,
    # sloping shoulder, and an open throat that receives the threaded neck piece.
    outer_profile = [
        (0.038, 0.000),
        (0.047, 0.004),
        (0.053, 0.012),
        (0.055, 0.026),
        (0.055, 0.088),
        (0.052, 0.103),
        (0.044, 0.115),
        (0.031, 0.124),
        (0.025, 0.128),
    ]
    mesh, outer = _lathe_surface(outer_profile)
    _add_disk(mesh, outer[0], 0.000, flip=True)

    inner_profile = [
        (0.018, 0.128),
        (0.023, 0.117),
        (0.039, 0.101),
        (0.050, 0.083),
        (0.050, 0.022),
        (0.030, 0.008),
    ]
    inner_rings = [_add_ring(mesh, radius, z) for radius, z in inner_profile]
    _connect_rings(mesh, outer[-1], inner_rings[0])
    for upper, lower in zip(inner_rings, inner_rings[1:]):
        _connect_rings(mesh, upper, lower)
    _add_disk(mesh, inner_rings[-1], 0.008)
    return mesh


def _make_threaded_neck() -> MeshGeometry:
    # Circumferential crests give the neck an unmistakable screw-thread profile
    # while keeping the visible fixed part compact under the cap.
    outer_profile = [
        (0.023, 0.120),
        (0.026, 0.124),
        (0.024, 0.128),
        (0.0185, 0.132),
        (0.0185, 0.135),
        (0.0210, 0.1365),
        (0.0210, 0.1385),
        (0.0185, 0.140),
        (0.0185, 0.144),
        (0.0210, 0.1455),
        (0.0210, 0.1475),
        (0.0185, 0.149),
        (0.0185, 0.153),
        (0.0210, 0.1545),
        (0.0210, 0.1565),
        (0.0185, 0.158),
        (0.0185, 0.164),
        (0.0205, 0.166),
        (0.0205, 0.170),
        (0.0185, 0.172),
        (0.0185, 0.174),
    ]
    mesh, outer = _lathe_surface(outer_profile)

    inner_profile = [
        (0.0120, 0.174),
        (0.0120, 0.121),
    ]
    inner_rings = [_add_ring(mesh, radius, z) for radius, z in inner_profile]
    _connect_rings(mesh, outer[-1], inner_rings[0])
    _connect_rings(mesh, inner_rings[0], inner_rings[1])
    _connect_rings(mesh, inner_rings[1], outer[0])
    return mesh


def _make_label_sleeve() -> MeshGeometry:
    mesh = MeshGeometry()
    outer_bottom = _add_ring(mesh, 0.0565, 0.047)
    outer_top = _add_ring(mesh, 0.0565, 0.083)
    inner_bottom = _add_ring(mesh, 0.0540, 0.047)
    inner_top = _add_ring(mesh, 0.0540, 0.083)
    _connect_rings(mesh, outer_bottom, outer_top)
    _connect_rings(mesh, inner_top, inner_bottom)
    _connect_rings(mesh, outer_top, inner_top)
    _connect_rings(mesh, inner_bottom, outer_bottom)
    return mesh


def _make_cap_shell() -> MeshGeometry:
    mesh = MeshGeometry()
    rib_count = 40

    def ribbed_radii(base: float, amplitude: float) -> list[float]:
        values: list[float] = []
        for i in range(SEGMENTS):
            theta = 2.0 * math.pi * i / SEGMENTS
            # Rounded vertical flutes: high spots catch light, low spots act as grip grooves.
            wave = 0.5 + 0.5 * math.cos(rib_count * theta)
            values.append(base + amplitude * (wave**1.8))
        return values

    outer_bottom = _add_ring(mesh, ribbed_radii(0.0284, 0.0008), 0.000)
    outer_lower = _add_ring(mesh, ribbed_radii(0.0296, 0.0015), 0.004)
    outer_upper = _add_ring(mesh, ribbed_radii(0.0296, 0.0015), 0.037)
    outer_top_bevel = _add_ring(mesh, ribbed_radii(0.0288, 0.0007), 0.042)
    outer_top = _add_ring(mesh, 0.0278, 0.044)
    for lower, upper in (
        (outer_bottom, outer_lower),
        (outer_lower, outer_upper),
        (outer_upper, outer_top_bevel),
        (outer_top_bevel, outer_top),
    ):
        _connect_rings(mesh, lower, upper)
    _add_disk(mesh, outer_top, 0.044)

    # The cap's inner screw surface is just under the 21 mm thread crests, so
    # the simplified meshes form a shallow retained thread engagement instead
    # of appearing to float around the bottle finish.
    inner_bottom = _add_ring(mesh, 0.0206, 0.001)
    inner_top = _add_ring(mesh, 0.0206, 0.036)
    _connect_rings(mesh, outer_bottom, inner_bottom)
    _connect_rings(mesh, inner_bottom, inner_top)
    _add_disk(mesh, inner_top, 0.036, flip=True)
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_plastic = Material("pale_clear_plastic", rgba=(0.75, 0.92, 1.0, 0.58))
    white_label = Material("satin_white_label", rgba=(0.96, 0.96, 0.90, 1.0))
    blue_cap = Material("ribbed_blue_plastic", rgba=(0.03, 0.18, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_bottle_shell(), "bottle_shell"),
        material=clear_plastic,
        name="bottle_shell",
    )
    body.visual(
        mesh_from_geometry(_make_threaded_neck(), "threaded_neck"),
        material=clear_plastic,
        name="threaded_neck",
    )
    body.visual(
        mesh_from_geometry(_make_label_sleeve(), "label_sleeve"),
        material=white_label,
        name="label_sleeve",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_make_cap_shell(), "cap_shell"),
        material=blue_cap,
        name="cap_shell",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    spin = object_model.get_articulation("neck_to_cap")

    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="threaded_neck",
        reason="The cap's simplified inner screw surface is intentionally seated into the neck thread crests.",
    )

    ctx.check(
        "cap uses a continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.expect_within(
        body,
        cap,
        axes="xy",
        inner_elem="threaded_neck",
        outer_elem="cap_shell",
        margin=0.0,
        name="cap diameter wraps threaded neck",
    )
    ctx.expect_overlap(
        cap,
        body,
        axes="z",
        elem_a="cap_shell",
        elem_b="threaded_neck",
        min_overlap=0.030,
        name="cap skirt covers threaded neck length",
    )
    ctx.expect_contact(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="threaded_neck",
        contact_tol=0.001,
        name="cap threads are seated on neck threads",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="bottle_shell",
        min_gap=0.004,
        max_gap=0.018,
        name="cap sits closely above bottle shoulder",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({spin: math.pi * 1.25}):
        turned_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap rotation stays on neck axis",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) < 1.0e-9 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
