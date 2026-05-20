from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathed_profile_mesh(
    profile: list[tuple[float, float]],
    *,
    segments: int = 96,
    radius_offset=None,
) -> MeshGeometry:
    """Revolve a closed radius/z cross-section into a connected triangular mesh."""

    geom = MeshGeometry()
    rings: list[list[int]] = []
    two_pi = 2.0 * math.pi

    for radius, z in profile:
        ring: list[int] = []
        for index in range(segments):
            theta = two_pi * index / segments
            local_radius = max(0.00005, radius)
            if radius_offset is not None:
                local_radius = max(0.00005, local_radius + radius_offset(radius, z, theta))
            ring.append(
                geom.add_vertex(
                    local_radius * math.cos(theta),
                    local_radius * math.sin(theta),
                    z,
                )
            )
        rings.append(ring)

    for i in range(len(profile)):
        next_i = (i + 1) % len(profile)
        for j in range(segments):
            next_j = (j + 1) % segments
            a = rings[i][j]
            b = rings[i][next_j]
            c = rings[next_i][next_j]
            d = rings[next_i][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    return geom


def _bottle_body_mesh() -> MeshGeometry:
    # Closed-bottom, open-mouth thin shell.  The profile walks around the
    # material cross-section: bottom center -> outside -> mouth lip -> inside.
    profile = [
        (0.0005, 0.000),
        (0.031, 0.000),
        (0.039, 0.006),
        (0.043, 0.017),
        (0.045, 0.045),
        (0.044, 0.112),
        (0.040, 0.142),
        (0.031, 0.156),
        (0.019, 0.166),
        (0.015, 0.173),
        (0.011, 0.173),
        (0.010, 0.166),
        (0.026, 0.151),
        (0.037, 0.134),
        (0.039, 0.045),
        (0.034, 0.014),
        (0.020, 0.008),
        (0.0005, 0.008),
    ]
    return _lathed_profile_mesh(profile, segments=112)


def _threaded_neck_mesh() -> MeshGeometry:
    # The raised circumferential ridges are modeled directly in the neck
    # cross-section so the threads are one continuous shell, not floating bands.
    profile = [
        (0.0105, 0.166),
        (0.0140, 0.166),
        (0.0140, 0.170),
        (0.0156, 0.1715),
        (0.0156, 0.1735),
        (0.0142, 0.1750),
        (0.0142, 0.1775),
        (0.0157, 0.1790),
        (0.0157, 0.1810),
        (0.0142, 0.1825),
        (0.0142, 0.1850),
        (0.0157, 0.1865),
        (0.0157, 0.1885),
        (0.0142, 0.1900),
        (0.0142, 0.198),
        (0.0140, 0.209),
        (0.0127, 0.212),
        (0.0100, 0.212),
    ]
    return _lathed_profile_mesh(profile, segments=112)


def _ribbed_cap_mesh() -> MeshGeometry:
    outer_radius = 0.0220
    inner_radius = 0.0175
    segments = 144
    geom = MeshGeometry()
    two_pi = 2.0 * math.pi

    def ribbed_radius(base: float, theta: float, amount: float = 0.0014) -> float:
        rib_wave = max(0.0, math.cos(36.0 * theta))
        return base + amount * rib_wave**4

    def add_ring(base_radius: float, z: float, *, ribbed: bool = False) -> list[int]:
        ring: list[int] = []
        for index in range(segments):
            theta = two_pi * index / segments
            radius = ribbed_radius(base_radius, theta) if ribbed else base_radius
            ring.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        return ring

    def connect(a: list[int], b: list[int]) -> None:
        for j in range(segments):
            next_j = (j + 1) % segments
            geom.add_face(a[j], a[next_j], b[next_j])
            geom.add_face(a[j], b[next_j], b[j])

    bottom_outer = add_ring(outer_radius, 0.000, ribbed=True)
    bottom_inner = add_ring(inner_radius, 0.000, ribbed=False)
    mid_outer = add_ring(outer_radius, 0.037, ribbed=True)
    top_outer = add_ring(0.0200, 0.044, ribbed=False)
    underside_inner = add_ring(inner_radius, 0.039, ribbed=False)

    # Open lower lip and single-wall ribbed skirt.  Leaving the skirt open keeps
    # the cap's interior as a real clearance cavity around the threaded neck.
    connect(bottom_inner, bottom_outer)
    connect(bottom_outer, mid_outer)
    connect(mid_outer, top_outer)
    connect(underside_inner, top_outer)

    top_center = geom.add_vertex(0.0, 0.0, 0.044)
    underside_center = geom.add_vertex(0.0, 0.0, 0.039)
    for j in range(segments):
        next_j = (j + 1) % segments
        geom.add_face(top_center, top_outer[j], top_outer[next_j])
        geom.add_face(underside_center, underside_inner[next_j], underside_inner[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("translucent_bottle_plastic", rgba=(0.58, 0.86, 1.0, 0.42))
    cap_plastic = model.material("ribbed_blue_cap", rgba=(0.02, 0.12, 0.62, 1.0))
    foot_metal = model.material("brushed_steel_foot", rgba=(0.55, 0.57, 0.58, 1.0))
    dark_recess = model.material("dark_screw_recesses", rgba=(0.02, 0.02, 0.025, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_body_mesh(), "bottle_body_shell"),
        material=bottle_plastic,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_geometry(_threaded_neck_mesh(), "threaded_neck_shell"),
        material=bottle_plastic,
        name="neck_threads",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_ribbed_cap_mesh(), "ribbed_screw_cap"),
        material=cap_plastic,
        name="cap_shell",
    )

    foot = model.part("foot")
    foot.visual(
        Box((0.140, 0.082, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=foot_metal,
        name="foot_plate",
    )
    for x in (-0.053, 0.053):
        foot.visual(
            Cylinder(radius=0.0065, length=0.0012),
            origin=Origin(xyz=(x, 0.0, 0.0006)),
            material=dark_recess,
            name=f"screw_recess_{0 if x < 0 else 1}",
        )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        # The cap frame is at the lower skirt edge.  Its internal top land sits
        # on the bottle mouth as the axial seal, while the skirt surrounds the
        # threaded neck with radial clearance.
        origin=Origin(xyz=(0.0, 0.0, 0.173)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "bottle_to_foot",
        ArticulationType.FIXED,
        parent=bottle,
        child=foot,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    foot = object_model.get_part("foot")
    cap_joint = object_model.get_articulation("neck_to_cap")
    foot_joint = object_model.get_articulation("bottle_to_foot")

    ctx.check(
        "cap rotates continuously about the neck axis",
        cap_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cap_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={cap_joint.articulation_type}, axis={cap_joint.axis}",
    )
    ctx.check(
        "support foot is fixed to the bottle",
        foot_joint.articulation_type == ArticulationType.FIXED,
        details=f"type={foot_joint.articulation_type}",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_threads",
        outer_elem="cap_shell",
        margin=0.002,
        name="threaded neck is centered inside the cap skirt",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_shell",
        elem_b="neck_threads",
        min_overlap=0.025,
        name="cap surrounds the threaded neck vertically",
    )
    ctx.expect_gap(
        bottle,
        foot,
        axis="z",
        positive_elem="body_shell",
        negative_elem="foot_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="bottle bottom sits on the fixed foot plate",
    )
    ctx.expect_overlap(
        foot,
        bottle,
        axes="xy",
        elem_a="foot_plate",
        elem_b="body_shell",
        min_overlap=0.040,
        name="foot plate supports the bottle footprint",
    )

    return ctx.report()


object_model = build_object_model()
