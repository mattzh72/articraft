from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_disc_y(
    inner_radius: float,
    outer_radius: float,
    thickness: float,
    *,
    segments: int = 72,
    center_y: float = 0.0,
    tooth_count: int | None = None,
    tooth_height: float = 0.0,
) -> MeshGeometry:
    """Annular cylinder in the XZ plane, extruded along Y."""
    geom = MeshGeometry()
    half = thickness / 2.0
    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []

    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        tooth = 0.0
        if tooth_count and tooth_height > 0.0:
            # A small alternating crest makes the ring read as a chainring
            # without pretending that a chain is present.
            phase = (i * tooth_count) % segments
            tooth = tooth_height if phase < segments / (tooth_count * 2.0) else 0.0
        ro = outer_radius + tooth
        x_o, z_o = ro * math.cos(theta), ro * math.sin(theta)
        x_i, z_i = inner_radius * math.cos(theta), inner_radius * math.sin(theta)
        outer_front.append(geom.add_vertex(x_o, center_y + half, z_o))
        outer_back.append(geom.add_vertex(x_o, center_y - half, z_o))
        inner_front.append(geom.add_vertex(x_i, center_y + half, z_i))
        inner_back.append(geom.add_vertex(x_i, center_y - half, z_i))

    for i in range(segments):
        j = (i + 1) % segments
        # Front annulus face.
        geom.add_face(outer_front[i], outer_front[j], inner_front[j])
        geom.add_face(outer_front[i], inner_front[j], inner_front[i])
        # Back annulus face.
        geom.add_face(outer_back[j], outer_back[i], inner_back[i])
        geom.add_face(outer_back[j], inner_back[i], inner_back[j])
        # Outer cylindrical wall.
        geom.add_face(outer_front[j], outer_front[i], outer_back[i])
        geom.add_face(outer_front[j], outer_back[i], outer_back[j])
        # Inner bore wall.
        geom.add_face(inner_front[i], inner_front[j], inner_back[j])
        geom.add_face(inner_front[i], inner_back[j], inner_back[i])

    return geom


def _bar_between_xz(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    width: float,
    thickness_y: float,
    center_y: float,
) -> MeshGeometry:
    """Rectangular machined strut between two XZ points, extruded along Y."""
    x0, z0 = start
    x1, z1 = end
    dx, dz = x1 - x0, z1 - z0
    length = math.hypot(dx, dz)
    if length <= 0.0:
        raise ValueError("bar endpoints must differ")
    nx, nz = -dz / length, dx / length
    half_w = width / 2.0
    half_t = thickness_y / 2.0

    section = [
        (x0 + nx * half_w, z0 + nz * half_w),
        (x1 + nx * half_w, z1 + nz * half_w),
        (x1 - nx * half_w, z1 - nz * half_w),
        (x0 - nx * half_w, z0 - nz * half_w),
    ]

    geom = MeshGeometry()
    front = [geom.add_vertex(x, center_y + half_t, z) for x, z in section]
    back = [geom.add_vertex(x, center_y - half_t, z) for x, z in section]
    geom.add_face(front[0], front[1], front[2])
    geom.add_face(front[0], front[2], front[3])
    geom.add_face(back[2], back[1], back[0])
    geom.add_face(back[3], back[2], back[0])
    for i in range(4):
        j = (i + 1) % 4
        geom.add_face(front[i], front[j], back[j])
        geom.add_face(front[i], back[j], back[i])
    return geom


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    segments: int = 24,
) -> MeshGeometry:
    """Capped cylinder mesh between two arbitrary points."""
    sx, sy, sz = start
    ex, ey, ez = end
    ax, ay, az = ex - sx, ey - sy, ez - sz
    length = math.sqrt(ax * ax + ay * ay + az * az)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must differ")
    wx, wy, wz = ax / length, ay / length, az / length

    # Choose a stable perpendicular basis around the tube axis.
    if abs(wz) < 0.9:
        ux, uy, uz = -wy, wx, 0.0
    else:
        ux, uy, uz = 0.0, -wz, wy
    u_len = math.sqrt(ux * ux + uy * uy + uz * uz)
    ux, uy, uz = ux / u_len, uy / u_len, uz / u_len
    vx = wy * uz - wz * uy
    vy = wz * ux - wx * uz
    vz = wx * uy - wy * ux

    geom = MeshGeometry()
    ring0: list[int] = []
    ring1: list[int] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        cx = math.cos(theta) * ux + math.sin(theta) * vx
        cy = math.cos(theta) * uy + math.sin(theta) * vy
        cz = math.cos(theta) * uz + math.sin(theta) * vz
        ring0.append(geom.add_vertex(sx + radius * cx, sy + radius * cy, sz + radius * cz))
        ring1.append(geom.add_vertex(ex + radius * cx, ey + radius * cy, ez + radius * cz))
    c0 = geom.add_vertex(sx, sy, sz)
    c1 = geom.add_vertex(ex, ey, ez)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(ring0[i], ring0[j], ring1[j])
        geom.add_face(ring0[i], ring1[j], ring1[i])
        geom.add_face(c0, ring0[i], ring0[j])
        geom.add_face(c1, ring1[j], ring1[i])
    return geom


def _make_pedal(model: ArticulatedObject, name: str, side: float, material: Material) -> object:
    """Flat pedal body with a real-looking bearing sleeve and open cage."""
    pedal = model.part(name)
    body_y = side * 0.046
    rail_y = 0.076
    rail_x = 0.105

    # Bearing sleeve centered on the pedal spindle axis. The spindle itself is
    # fixed to the crankset part and intentionally passes through this sleeve.
    pedal.visual(
        Cylinder(radius=0.0105, length=0.082),
        origin=Origin(xyz=(0.0, body_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="bearing_sleeve",
    )

    for z in (-0.020, 0.020):
        pedal.visual(
            Box((0.012, rail_y, 0.006)),
            origin=Origin(xyz=(-rail_x / 2.0, body_y, z)),
            material=material,
            name=f"side_rail_{'top' if z > 0 else 'bottom'}_0",
        )
        pedal.visual(
            Box((0.012, rail_y, 0.006)),
            origin=Origin(xyz=(rail_x / 2.0, body_y, z)),
            material=material,
            name=f"side_rail_{'top' if z > 0 else 'bottom'}_1",
        )
        pedal.visual(
            Box((rail_x, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, body_y + side * rail_y / 2.0, z)),
            material=material,
            name=f"outer_rail_{'top' if z > 0 else 'bottom'}",
        )
        pedal.visual(
            Box((rail_x, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, body_y - side * rail_y / 2.0, z)),
            material=material,
            name=f"inner_rail_{'top' if z > 0 else 'bottom'}",
        )
        # A shallow bridge kisses the sleeve, tying the cage to the bearing
        # without intersecting the visible spindle.
        pedal.visual(
            Box((0.096, 0.016, 0.008)),
            origin=Origin(xyz=(0.0, body_y, z * 0.70)),
            material=material,
            name=f"center_bridge_{'top' if z > 0 else 'bottom'}",
        )

    # Four corner posts keep the upper and lower cages visibly connected.
    for x in (-rail_x / 2.0, rail_x / 2.0):
        for y in (body_y - side * rail_y / 2.0, body_y + side * rail_y / 2.0):
            pedal.visual(
                Box((0.010, 0.010, 0.042)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=material,
                name=f"corner_post_{len(pedal.visuals)}",
            )

    # Small traction pins on both faces, proud enough to read but still mounted.
    for z in (-0.025, 0.025):
        for x in (-0.035, 0.0, 0.035):
            for y in (body_y - side * 0.033, body_y + side * 0.033):
                pedal.visual(
                    Box((0.007, 0.007, 0.006)),
                    origin=Origin(xyz=(x, y, z)),
                    material=material,
                    name=f"grip_pin_{len(pedal.visuals)}",
                )
    return pedal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_crankset_pedals")

    polished = model.material("brushed_aluminum", rgba=(0.78, 0.76, 0.70, 1.0))
    dark_metal = model.material("anodized_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_composite", rgba=(0.015, 0.015, 0.017, 1.0))
    bearing = model.material("dark_bearing_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_geometry(
            _annular_disc_y(0.0185, 0.034, 0.078, segments=80),
            "bottom_bracket_shell",
        ),
        material=polished,
        name="shell",
    )
    for side in (-1.0, 1.0):
        bottom_bracket.visual(
            mesh_from_geometry(
                _annular_disc_y(0.0145, 0.030, 0.008, segments=72, center_y=side * 0.043),
                f"bearing_cup_{side:+.0f}",
            ),
            material=bearing,
            name=f"bearing_cup_{0 if side < 0 else 1}",
        )

    # Short welded frame-tube stubs make the subassembly feel mounted and
    # grounded, while leaving the drivetrain side unobstructed.
    bottom_bracket.visual(
        mesh_from_geometry(_cylinder_between((-0.004, 0.0, 0.023), (-0.030, 0.0, 0.165), 0.018), "seat_tube_stub"),
        material=dark_metal,
        name="seat_tube_stub",
    )
    bottom_bracket.visual(
        mesh_from_geometry(_cylinder_between((0.026, 0.0, 0.015), (0.145, 0.0, 0.095), 0.021), "down_tube_stub"),
        material=dark_metal,
        name="down_tube_stub",
    )
    for side in (-1.0, 1.0):
        bottom_bracket.visual(
            mesh_from_geometry(
                _cylinder_between((-0.022, side * 0.031, -0.012), (-0.124, side * 0.055, -0.038), 0.010),
                f"chainstay_lug_{side:+.0f}",
            ),
            material=dark_metal,
            name=f"chainstay_lug_{0 if side < 0 else 1}",
        )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="spindle",
    )
    for y, side_name in ((-0.050, "drive"), (0.050, "outer")):
        crankset.visual(
            Cylinder(radius=0.026, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=f"{side_name}_hub_cap",
        )

    # Drive-side chainring and five-arm spider.
    crankset.visual(
        mesh_from_geometry(
            _annular_disc_y(0.074, 0.106, 0.0045, segments=144, center_y=-0.065, tooth_count=48, tooth_height=0.004),
            "toothed_chainring",
        ),
        material=dark_metal,
        name="chainring",
    )
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0 + math.radians(18.0)
        start = (0.031 * math.cos(angle), 0.031 * math.sin(angle))
        end = (0.081 * math.cos(angle), 0.081 * math.sin(angle))
        crankset.visual(
            mesh_from_geometry(
                _bar_between_xz(start, end, width=0.018, thickness_y=0.007, center_y=-0.060),
                f"spider_arm_{i}",
            ),
            material=polished,
            name=f"spider_arm_{i}",
        )
        crankset.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(end[0], -0.066, end[1]), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=f"chainring_bolt_{i}",
        )

    # Opposed crank arms: drive-side arm down, non-drive arm up at zero pose.
    crankset.visual(
        mesh_from_geometry(
            _bar_between_xz((0.0, -0.018), (0.0, -0.167), width=0.024, thickness_y=0.014, center_y=-0.055),
            "drive_crank_arm",
        ),
        material=polished,
        name="drive_arm",
    )
    crankset.visual(
        mesh_from_geometry(
            _bar_between_xz((0.0, 0.018), (0.0, 0.167), width=0.024, thickness_y=0.014, center_y=0.055),
            "outer_crank_arm",
        ),
        material=polished,
        name="outer_arm",
    )
    for y, z, name in ((-0.055, -0.170, "drive_pedal_eye"), (0.055, 0.170, "outer_pedal_eye")):
        crankset.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.0, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=name,
        )
    for side, y, z, name in ((-1.0, -0.106, -0.170, "pedal_axle_0"), (1.0, 0.106, 0.170, "pedal_axle_1")):
        crankset.visual(
            Cylinder(radius=0.006, length=0.088),
            origin=Origin(xyz=(0.0, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=name,
        )

    pedal_0 = _make_pedal(model, "pedal_0", -1.0, black)
    pedal_1 = _make_pedal(model, "pedal_1", 1.0, black)

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.03, friction=0.01),
    )
    model.articulation(
        "pedal_0_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=pedal_0,
        origin=Origin(xyz=(0.0, -0.072, -0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.003),
    )
    model.articulation(
        "pedal_1_spin",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=pedal_1,
        origin=Origin(xyz=(0.0, 0.072, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.003),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    pedal_0 = object_model.get_part("pedal_0")
    pedal_1 = object_model.get_part("pedal_1")
    crank_spin = object_model.get_articulation("crank_spin")

    ctx.allow_overlap(
        crankset,
        pedal_0,
        elem_a="pedal_axle_0",
        elem_b="bearing_sleeve",
        reason="The pedal spindle is intentionally captured inside the pedal bearing sleeve.",
    )
    ctx.allow_overlap(
        crankset,
        pedal_1,
        elem_a="pedal_axle_1",
        elem_b="bearing_sleeve",
        reason="The pedal spindle is intentionally captured inside the pedal bearing sleeve.",
    )

    ctx.expect_within(
        crankset,
        bottom_bracket,
        axes="xz",
        inner_elem="spindle",
        outer_elem="shell",
        margin=0.0,
        name="spindle is centered in bottom bracket bore footprint",
    )
    ctx.expect_overlap(
        crankset,
        bottom_bracket,
        axes="y",
        elem_a="spindle",
        elem_b="shell",
        min_overlap=0.070,
        name="spindle passes through bottom bracket shell",
    )
    ctx.expect_gap(
        bottom_bracket,
        crankset,
        axis="y",
        positive_elem="shell",
        negative_elem="chainring",
        min_gap=0.015,
        name="chainring is clear of shell on drive side",
    )

    for pedal, axle in ((pedal_0, "pedal_axle_0"), (pedal_1, "pedal_axle_1")):
        ctx.expect_within(
            crankset,
            pedal,
            axes="xz",
            inner_elem=axle,
            outer_elem="bearing_sleeve",
            margin=0.0,
            name=f"{axle} is centered in pedal bearing",
        )
        ctx.expect_overlap(
            crankset,
            pedal,
            axes="y",
            elem_a=axle,
            elem_b="bearing_sleeve",
            min_overlap=0.060,
            name=f"{axle} remains captured along its sleeve",
        )

    rest_pedal = ctx.part_world_position(pedal_0)
    with ctx.pose({crank_spin: math.pi / 2.0}):
        turned_pedal = ctx.part_world_position(pedal_0)
        ctx.expect_overlap(
            crankset,
            bottom_bracket,
            axes="y",
            elem_a="spindle",
            elem_b="shell",
            min_overlap=0.070,
            name="spindle remains coaxial after quarter crank rotation",
        )
    ctx.check(
        "crank rotation carries pedal around bottom bracket",
        rest_pedal is not None
        and turned_pedal is not None
        and rest_pedal[2] < -0.15
        and turned_pedal[0] < -0.15,
        details=f"rest={rest_pedal}, turned={turned_pedal}",
    )

    return ctx.report()


object_model = build_object_model()
