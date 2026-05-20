from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylindrical_shell_segment(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 64,
) -> MeshGeometry:
    """Closed solid mesh for a partial cylindrical sleeve/collar."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for i in range(segments + 1):
        theta = start_angle + (end_angle - start_angle) * i / segments
        c, s = math.cos(theta), math.sin(theta)
        rings.append(
            [
                geom.add_vertex(outer_radius * c, outer_radius * s, z_min),
                geom.add_vertex(outer_radius * c, outer_radius * s, z_max),
                geom.add_vertex(inner_radius * c, inner_radius * s, z_max),
                geom.add_vertex(inner_radius * c, inner_radius * s, z_min),
            ]
        )

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        a, b = rings[i], rings[i + 1]
        quad(a[0], b[0], b[1], a[1])  # outer wall
        quad(a[2], b[2], b[3], a[3])  # inner wall
        quad(a[1], b[1], b[2], a[2])  # top lip
        quad(a[3], b[3], b[0], a[0])  # lower lip

    # End caps make the cut-away collar read as thick molded material.
    quad(rings[0][0], rings[0][1], rings[0][2], rings[0][3])
    quad(rings[-1][3], rings[-1][2], rings[-1][1], rings[-1][0])
    return geom


def _helical_ridge(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    pitch: float,
    width: float,
    *,
    phase: float = 0.0,
    segments_per_turn: int = 28,
) -> MeshGeometry:
    """Raised rectangular helical strip used for external and internal threads."""
    turns = max((z_max - z_min) / pitch, 0.25)
    steps = max(int(turns * segments_per_turn), 12)
    geom = MeshGeometry()
    profiles: list[list[int]] = []

    for i in range(steps + 1):
        frac = i / steps
        theta = phase + turns * 2.0 * math.pi * frac
        z = z_min + (z_max - z_min) * frac
        c, s = math.cos(theta), math.sin(theta)
        z0 = z - width * 0.5
        z1 = z + width * 0.5
        profiles.append(
            [
                geom.add_vertex(inner_radius * c, inner_radius * s, z0),
                geom.add_vertex(outer_radius * c, outer_radius * s, z0),
                geom.add_vertex(outer_radius * c, outer_radius * s, z1),
                geom.add_vertex(inner_radius * c, inner_radius * s, z1),
            ]
        )

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(steps):
        a, b = profiles[i], profiles[i + 1]
        quad(a[0], b[0], b[1], a[1])  # lower flank
        quad(a[1], b[1], b[2], a[2])  # outer crest
        quad(a[2], b[2], b[3], a[3])  # upper flank
        quad(a[3], b[3], b[0], a[0])  # root face

    quad(profiles[0][0], profiles[0][1], profiles[0][2], profiles[0][3])
    quad(profiles[-1][3], profiles[-1][2], profiles[-1][1], profiles[-1][0])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_screw_in_bulb_socket")

    painted = model.material("dark_olive_powdercoat", rgba=(0.11, 0.13, 0.10, 1.0))
    black = model.material("black_molded_rubber", rgba=(0.015, 0.016, 0.014, 1.0))
    brass = model.material("brushed_brass", rgba=(0.82, 0.62, 0.28, 1.0))
    steel = model.material("scratched_stainless", rgba=(0.62, 0.63, 0.60, 1.0))
    ceramic = model.material("off_white_ceramic", rgba=(0.88, 0.84, 0.73, 1.0))
    glass = model.material("warm_translucent_glass", rgba=(0.95, 0.86, 0.55, 0.36))
    amber = model.material("warm_filament", rgba=(1.0, 0.62, 0.18, 1.0))
    witness = model.material("black_index_paint", rgba=(0.01, 0.01, 0.008, 1.0))

    socket = model.part("socket")
    socket.visual(
        Box((0.112, 0.080, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=painted,
        name="mounting_plate",
    )
    socket.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[(0.037, 0.000), (0.041, 0.010), (0.038, 0.020), (0.034, 0.028)],
                inner_profile=[(0.0172, 0.000), (0.0172, 0.028)],
                segments=72,
            ),
            "socket_lower_housing",
        ),
        material=painted,
        name="lower_housing",
    )
    socket.visual(
        mesh_from_geometry(
            _cylindrical_shell_segment(
                0.0166,
                0.0325,
                0.018,
                0.062,
                math.radians(52.0),
                math.radians(308.0),
                segments=72,
            ),
            "socket_cutaway_collar",
        ),
        material=painted,
        name="cutaway_collar",
    )
    socket.visual(
        mesh_from_geometry(
            _helical_ridge(0.0147, 0.0167, 0.026, 0.058, 0.0041, 0.00125, phase=math.pi * 0.72),
            "socket_female_thread",
        ),
        material=brass,
        name="female_thread",
    )
    socket.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[(0.041, 0.008), (0.041, 0.012)],
                inner_profile=[(0.0170, 0.008), (0.0170, 0.012)],
                segments=72,
            ),
            "socket_black_gasket",
        ),
        material=black,
        name="gasket_ring",
    )
    socket.visual(
        Cylinder(radius=0.0176, length=0.0065),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=ceramic,
        name="ceramic_floor",
    )
    socket.visual(
        Cylinder(radius=0.0062, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.0244)),
        material=brass,
        name="spring_contact",
    )

    # Overbuilt vertical ribs and side lips make the collar read as a utility socket,
    # not a delicate bare lampholder.
    for idx, (x, y, sx, sy) in enumerate(
        (
            (0.031, 0.000, 0.010, 0.050),
            (-0.031, 0.000, 0.010, 0.050),
            (0.000, -0.032, 0.050, 0.010),
        )
    ):
        socket.visual(
            Box((sx, sy, 0.047)),
            origin=Origin(xyz=(x, y, 0.038)),
            material=painted,
            name=f"reinforcing_rib_{idx}",
        )

    # Armored cable strain relief with compression ribs at the rear of the socket.
    socket.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.0, -0.051, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="strain_relief",
    )
    for idx, y in enumerate((-0.041, -0.053, -0.065)):
        socket.visual(
            Cylinder(radius=0.014, length=0.0045),
            origin=Origin(xyz=(0.0, y, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"relief_rib_{idx}",
        )

    # Exposed fasteners clamp the mounting plate. Slots are dark painted insets.
    for row, y in enumerate((-0.026, 0.026)):
        for col, x in enumerate((-0.041, 0.041)):
            suffix = f"{row}_{col}"
            socket.visual(
                Cylinder(radius=0.0065, length=0.0032),
                origin=Origin(xyz=(x, y, 0.0113)),
                material=steel,
                name=f"fastener_{suffix}",
            )
            socket.visual(
                Box((0.0095, 0.0017, 0.0008)),
                origin=Origin(xyz=(x, y, 0.0132)),
                material=witness,
                name=f"fastener_slot_{suffix}",
            )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0120, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=brass,
        name="screw_core",
    )
    bulb.visual(
        mesh_from_geometry(
            _helical_ridge(0.0120, 0.0138, -0.030, -0.004, 0.0041, 0.00135, phase=0.0),
            "bulb_external_thread",
        ),
        material=brass,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0070, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, -0.0318)),
        material=steel,
        name="bottom_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0134, length=0.0075),
        origin=Origin(xyz=(0.0, 0.0, 0.0028)),
        material=brass,
        name="crimp_ring",
    )
    bulb.visual(
        Box((0.0020, 0.0060, 0.0100)),
        origin=Origin(xyz=(0.0142, 0.0, -0.0040)),
        material=witness,
        name="index_mark",
    )
    bulb.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[
                    (0.0100, 0.004),
                    (0.0145, 0.014),
                    (0.0245, 0.030),
                    (0.0300, 0.052),
                    (0.0285, 0.076),
                    (0.0200, 0.096),
                    (0.0060, 0.110),
                ],
                inner_profile=[
                    (0.0078, 0.006),
                    (0.0122, 0.016),
                    (0.0220, 0.031),
                    (0.0274, 0.052),
                    (0.0260, 0.075),
                    (0.0180, 0.093),
                    (0.0042, 0.106),
                ],
                segments=96,
                lip_samples=8,
            ),
            "bulb_glass_envelope",
        ),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0058, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=glass,
        name="glass_stem",
    )
    for idx, x in enumerate((-0.0048, 0.0048)):
        bulb.visual(
            Cylinder(radius=0.00065, length=0.055),
            origin=Origin(xyz=(x, 0.0, 0.0375)),
            material=steel,
            name=f"lead_wire_{idx}",
        )
    bulb.visual(
        Cylinder(radius=0.0012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.063), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="filament_bar",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=4.0 * math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    screw_joint = object_model.get_articulation("socket_to_bulb")

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_core",
        outer_elem="lower_housing",
        margin=0.001,
        name="screw base centered inside rugged socket",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="external_thread",
        elem_b="female_thread",
        min_overlap=0.024,
        name="threaded base has practical engagement depth",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="bottom_contact",
        negative_elem="spring_contact",
        min_gap=0.001,
        max_gap=0.010,
        name="bottom contact is seated near spring contact without collision",
    )

    rest_aabb = ctx.part_element_world_aabb(bulb, elem="index_mark")
    with ctx.pose({screw_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(bulb, elem="index_mark")

    def _center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    rest_xy = _center_xy(rest_aabb)
    turned_xy = _center_xy(turned_aabb)
    ctx.check(
        "bulb rotation is coaxial and visible",
        rest_xy is not None
        and turned_xy is not None
        and rest_xy[0] > 0.012
        and abs(rest_xy[1]) < 0.004
        and abs(turned_xy[0]) < 0.004
        and turned_xy[1] > 0.012,
        details=f"rest_index={rest_xy}, quarter_turn_index={turned_xy}",
    )

    return ctx.report()


object_model = build_object_model()
