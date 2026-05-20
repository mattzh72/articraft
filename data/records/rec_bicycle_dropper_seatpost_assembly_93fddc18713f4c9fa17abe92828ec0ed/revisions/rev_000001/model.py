from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


OUTER_HEIGHT = 0.365
TRAVEL = 0.150
INNER_TOP = 0.360
INNER_LENGTH = 0.560


def _hollow_ring_mesh(
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=2,
    )


def _outer_body_mesh() -> MeshGeometry:
    # A continuous stepped hollow extrusion: lower insertion skirt, two machined
    # reinforcing bands, and a wider wiper-seat collar at the top.
    outer_profile = [
        (0.0185, 0.000),
        (0.0185, 0.022),
        (0.0162, 0.030),
        (0.0162, 0.095),
        (0.0174, 0.100),
        (0.0174, 0.111),
        (0.0162, 0.116),
        (0.0162, 0.212),
        (0.0172, 0.218),
        (0.0172, 0.230),
        (0.0162, 0.236),
        (0.0162, 0.318),
        (0.0206, 0.329),
        (0.0206, OUTER_HEIGHT),
    ]
    inner_profile = [(0.0138, 0.000), (0.0138, OUTER_HEIGHT)]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def _saddle_body_mesh() -> MeshGeometry:
    """Closed superellipse-section loft for a slim-front trail saddle."""
    geom = MeshGeometry()
    sections = [
        (-0.150, 0.042, 0.030, 0.017),
        (-0.108, 0.074, 0.040, 0.022),
        (-0.040, 0.068, 0.039, 0.021),
        (0.040, 0.050, 0.034, 0.018),
        (0.112, 0.030, 0.028, 0.014),
        (0.168, 0.014, 0.024, 0.010),
    ]
    ring_count = 40
    rings: list[list[int]] = []
    exponent = 2.45
    for x, half_width, z_center, half_height in sections:
        ring: list[int] = []
        for i in range(ring_count):
            angle = 2.0 * math.pi * i / ring_count
            c = math.cos(angle)
            s = math.sin(angle)
            y = half_width * math.copysign(abs(c) ** (2.0 / exponent), c)
            z = z_center + half_height * math.copysign(abs(s) ** (2.0 / exponent), s)
            # Subtle lengthwise crown: rear platform stays thick, the slim nose
            # drops slightly, and the middle has a shallow center relief.
            if abs(y) < half_width * 0.20 and x < 0.075 and z > z_center:
                z -= 0.0035 * (1.0 - abs(y) / (half_width * 0.20))
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for a, b in zip(rings, rings[1:]):
        for i in range(ring_count):
            j = (i + 1) % ring_count
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])

    for ring in (rings[0], rings[-1]):
        cx = sum(geom.vertices[i][0] for i in ring) / len(ring)
        cy = sum(geom.vertices[i][1] for i in ring) / len(ring)
        cz = sum(geom.vertices[i][2] for i in ring) / len(ring)
        center = geom.add_vertex(cx, cy, cz)
        for i in range(ring_count):
            j = (i + 1) % ring_count
            geom.add_face(center, ring[i], ring[j])
    return geom


def _center_channel_mesh() -> MeshGeometry:
    # A low, dark, recessed-looking strip that follows the saddle crown.
    profile = [
        (-0.116, -0.010),
        (0.060, -0.006),
        (0.087, 0.000),
        (0.060, 0.006),
        (-0.116, 0.010),
    ]
    geom = MeshGeometry()
    top: list[int] = []
    bottom: list[int] = []
    for x, y in profile:
        z = 0.060 if x < -0.060 else 0.050 - 0.040 * max(x, 0.0)
        top.append(geom.add_vertex(x, y, z))
        bottom.append(geom.add_vertex(x, y, z - 0.0012))
    for i in range(len(profile)):
        j = (i + 1) % len(profile)
        geom.add_face(top[i], top[j], bottom[j])
        geom.add_face(top[i], bottom[j], bottom[i])
    center_top = geom.add_vertex(
        sum(geom.vertices[i][0] for i in top) / len(top),
        0.0,
        sum(geom.vertices[i][2] for i in top) / len(top),
    )
    for i in range(len(profile)):
        geom.add_face(center_top, top[i], top[(i + 1) % len(profile)])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_dropper_post")

    hard_anodized = model.material("hard_anodized_alloy", rgba=(0.10, 0.105, 0.11, 1.0))
    satin_alloy = model.material("satin_machined_alloy", rgba=(0.58, 0.60, 0.62, 1.0))
    black_alloy = model.material("black_machined_alloy", rgba=(0.045, 0.048, 0.052, 1.0))
    wiper_rubber = model.material("wiper_rubber", rgba=(0.010, 0.010, 0.011, 1.0))
    cable_black = model.material("black_cable_housing", rgba=(0.004, 0.004, 0.005, 1.0))
    saddle_cover = model.material("matte_saddle_cover", rgba=(0.025, 0.026, 0.028, 1.0))
    rear_pad = model.material("reinforced_rear_pad", rgba=(0.055, 0.057, 0.060, 1.0))
    rail_steel = model.material("brushed_steel_rails", rgba=(0.72, 0.74, 0.75, 1.0))

    outer_body = model.part("outer_body")
    outer_body.visual(
        mesh_from_geometry(_outer_body_mesh(), "machined_outer_body"),
        material=hard_anodized,
        name="outer_body_shell",
    )
    outer_body.visual(
        mesh_from_geometry(
            _hollow_ring_mesh(0.0136, 0.0198, 0.349, 0.365, segments=96),
            "rubber_wiper_ring",
        ),
        material=wiper_rubber,
        name="wiper_seal",
    )
    # Machined anti-rotation flats, seated into the alloy body so they read as
    # milled surface treatment rather than separate floating badges.
    for index, angle in enumerate((0.0, math.pi)):
        outer_body.visual(
            Box((0.004, 0.0016, 0.210)),
            origin=Origin(
                xyz=(0.0, 0.01645 * math.cos(angle), 0.178),
                rpy=(0.0, 0.0, angle),
            ),
            material=black_alloy,
            name=f"machined_flat_{index}",
        )

    inner_shaft = model.part("inner_shaft")
    inner_shaft.visual(
        Cylinder(radius=0.0121, length=INNER_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_TOP - INNER_LENGTH / 2.0)),
        material=satin_alloy,
        name="stanchion",
    )
    inner_shaft.visual(
        Cylinder(radius=0.0134, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, INNER_TOP - 0.010)),
        material=satin_alloy,
        name="top_shoulder",
    )
    # Low-friction guide pads lightly preload against the bore of the outer
    # body, giving the retained telescoping stage real sliding support.
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        inner_shaft.visual(
            Box((0.0040, 0.0025, 0.035)),
            origin=Origin(
                xyz=(0.0129 * math.cos(angle), 0.0129 * math.sin(angle), -0.165),
                rpy=(0.0, 0.0, angle - math.pi / 2.0),
            ),
            material=wiper_rubber,
            name=f"guide_pad_{index}",
        )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=inner_shaft,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=TRAVEL),
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Cylinder(radius=0.0178, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=black_alloy,
        name="stanchion_socket",
    )
    clamp_head.visual(
        Box((0.066, 0.052, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=black_alloy,
        name="head_block",
    )
    for index, y in enumerate((-0.034, 0.034)):
        clamp_head.visual(
            Box((0.066, 0.014, 0.008)),
            origin=Origin(xyz=(0.0, 0.032 if y > 0.0 else -0.032, 0.073)),
            material=satin_alloy,
            name=f"rail_cradle_{index}",
        )
        clamp_head.visual(
            Cylinder(radius=0.0042, length=0.070),
            origin=Origin(xyz=(0.0, 0.030 if y > 0.0 else -0.030, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_alloy,
            name=f"clamp_bolt_{index}",
        )
    clamp_head.visual(
        Cylinder(radius=0.0080, length=0.035),
        origin=Origin(xyz=(-0.023, 0.041, 0.047), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_alloy,
        name="cable_port_boss",
    )
    clamp_head.visual(
        Cylinder(radius=0.0043, length=0.040),
        origin=Origin(xyz=(-0.023, 0.050, 0.047), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cable_black,
        name="cable_port_liner",
    )
    clamp_head.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.023, 0.065, 0.047),
                    (-0.010, 0.078, 0.010),
                    (0.004, 0.081, -0.050),
                    (0.006, 0.077, -0.105),
                ],
                radius=0.0035,
                samples_per_segment=16,
                radial_segments=14,
                cap_ends=True,
            ),
            "short_cable_housing",
        ),
        material=cable_black,
        name="cable_housing",
    )

    model.articulation(
        "inner_to_clamp",
        ArticulationType.FIXED,
        parent=inner_shaft,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, INNER_TOP)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_geometry(_saddle_body_mesh(), "contoured_saddle_body"),
        material=saddle_cover,
        name="saddle_shell",
    )
    saddle.visual(
        mesh_from_geometry(_center_channel_mesh(), "saddle_center_channel"),
        material=wiper_rubber,
        name="center_channel",
    )
    saddle.visual(
        Box((0.118, 0.118, 0.003)),
        origin=Origin(xyz=(-0.088, 0.0, 0.060)),
        material=rear_pad,
        name="padded_rear_platform",
    )
    rail_path = [
        (-0.118, -0.042, 0.006),
        (-0.070, -0.039, 0.001),
        (0.015, -0.035, 0.001),
        (0.092, -0.024, 0.004),
        (0.122, 0.000, 0.006),
        (0.092, 0.024, 0.004),
        (0.015, 0.035, 0.001),
        (-0.070, 0.039, 0.001),
        (-0.118, 0.042, 0.006),
    ]
    saddle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rail_path,
                radius=0.0032,
                samples_per_segment=18,
                radial_segments=16,
                cap_ends=True,
            ),
            "saddle_rail_loop",
        ),
        material=rail_steel,
        name="rail_loop",
    )
    for index, (x, y) in enumerate(
        ((-0.112, -0.042), (-0.112, 0.042), (0.078, -0.026), (0.078, 0.026))
    ):
        saddle.visual(
            Box((0.026, 0.014, 0.016)),
            origin=Origin(xyz=(x, y, 0.016)),
            material=black_alloy,
            name=f"rail_anchor_{index}",
        )

    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent=clamp_head,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.0797)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    inner_shaft = object_model.get_part("inner_shaft")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")
    slide = object_model.get_articulation("outer_to_inner")

    ctx.expect_within(
        inner_shaft,
        outer_body,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_body_shell",
        margin=0.001,
        name="stanchion centered in hollow outer body",
    )
    ctx.expect_overlap(
        inner_shaft,
        outer_body,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_body_shell",
        min_overlap=0.145,
        name="collapsed stanchion retained in outer sleeve",
    )
    rest_pos = ctx.part_world_position(inner_shaft)
    with ctx.pose({slide: TRAVEL}):
        ctx.expect_within(
            inner_shaft,
            outer_body,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_body_shell",
            margin=0.001,
            name="extended stanchion remains centered in sleeve",
        )
        ctx.expect_overlap(
            inner_shaft,
            outer_body,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_body_shell",
            min_overlap=0.040,
            name="extended stanchion preserves insertion",
        )
        extended_pos = ctx.part_world_position(inner_shaft)

    ctx.check(
        "dropper shaft extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.expect_overlap(
        saddle,
        clamp_head,
        axes="xy",
        elem_a="rail_loop",
        elem_b="head_block",
        min_overlap=0.035,
        name="saddle rails span the clamp head",
    )

    return ctx.report()


object_model = build_object_model()
