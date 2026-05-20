from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _cap_loop(geom: MeshGeometry, loop: list[int], *, reverse: bool = False) -> None:
    center = geom.add_vertex(
        sum(geom.vertices[i][0] for i in loop) / len(loop),
        sum(geom.vertices[i][1] for i in loop) / len(loop),
        sum(geom.vertices[i][2] for i in loop) / len(loop),
    )
    count = len(loop)
    for i in range(count):
        a = loop[i]
        b = loop[(i + 1) % count]
        if reverse:
            geom.add_face(center, b, a)
        else:
            geom.add_face(center, a, b)


def _loft_between_loops(geom: MeshGeometry, loops: list[list[int]]) -> None:
    count = len(loops[0])
    for lower, upper in zip(loops, loops[1:]):
        for i in range(count):
            a = lower[i]
            b = lower[(i + 1) % count]
            c = upper[(i + 1) % count]
            d = upper[i]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)


def _fuselage_mesh() -> MeshGeometry:
    """Smooth, tapered model-airplane body along X with elliptical stations."""
    sections = [
        (-0.350, 0.260, 0.020, 0.026),
        (-0.295, 0.260, 0.048, 0.045),
        (-0.180, 0.262, 0.082, 0.072),
        (-0.030, 0.265, 0.108, 0.098),
        (0.135, 0.263, 0.100, 0.092),
        (0.255, 0.260, 0.066, 0.066),
        (0.315, 0.260, 0.036, 0.040),
    ]
    geom = MeshGeometry()
    loops: list[list[int]] = []
    segments = 36
    for x, zc, width, height in sections:
        loop: list[tuple[float, float, float]] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            # Slightly squared lower side gives the belly a plausible model-airplane stance.
            y = 0.5 * width * math.cos(theta)
            z = zc + 0.5 * height * math.sin(theta) * (0.92 if math.sin(theta) < 0 else 1.06)
            loop.append((x, y, z))
        loops.append(_add_loop(geom, loop))
    _loft_between_loops(geom, loops)
    _cap_loop(geom, loops[0], reverse=True)
    _cap_loop(geom, loops[-1])
    return geom


def _wing_mesh(
    *,
    span: float,
    root_leading: float,
    root_trailing: float,
    tip_leading: float,
    tip_trailing: float,
    z_root: float,
    dihedral: float,
    root_thickness: float,
    tip_thickness: float,
) -> MeshGeometry:
    """One continuous tapered airfoil-like panel spanning across the fuselage."""
    stations = [-0.5 * span, -0.25 * span, 0.0, 0.25 * span, 0.5 * span]
    geom = MeshGeometry()
    loops: list[list[int]] = []
    for y in stations:
        frac = abs(y) / (0.5 * span)
        leading = root_leading + (tip_leading - root_leading) * frac
        trailing = root_trailing + (tip_trailing - root_trailing) * frac
        chord = leading - trailing
        zc = z_root + dihedral * frac
        thickness = root_thickness + (tip_thickness - root_thickness) * frac
        profile = [
            (leading, y, zc),
            (leading - 0.18 * chord, y, zc + 0.52 * thickness),
            (leading - 0.52 * chord, y, zc + 0.42 * thickness),
            (trailing, y, zc + 0.05 * thickness),
            (trailing, y, zc - 0.05 * thickness),
            (leading - 0.52 * chord, y, zc - 0.34 * thickness),
            (leading - 0.18 * chord, y, zc - 0.45 * thickness),
        ]
        loops.append(_add_loop(geom, profile))
    _loft_between_loops(geom, loops)
    _cap_loop(geom, loops[0], reverse=True)
    _cap_loop(geom, loops[-1])
    return geom


def _vertical_fin_mesh() -> MeshGeometry:
    """Triangular vertical stabilizer with finite thickness in Y."""
    side = [
        (-0.345, 0.272),
        (-0.220, 0.274),
        (-0.290, 0.398),
    ]
    half = 0.008
    geom = MeshGeometry()
    left = _add_loop(geom, [(x, -half, z) for x, z in side])
    right = _add_loop(geom, [(x, half, z) for x, z in side])
    geom.add_face(left[0], left[1], left[2])
    geom.add_face(right[0], right[2], right[1])
    for i in range(3):
        a = left[i]
        b = left[(i + 1) % 3]
        c = right[(i + 1) % 3]
        d = right[i]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
    return geom


def _canopy_mesh() -> MeshGeometry:
    """Low transparent cockpit blister seated into the upper fuselage."""
    geom = MeshGeometry()
    xs = [-0.055, -0.015, 0.060, 0.125]
    widths = [0.035, 0.060, 0.066, 0.032]
    bottoms = [0.302, 0.309, 0.306, 0.296]
    heights = [0.010, 0.036, 0.032, 0.008]
    loops: list[list[int]] = []
    segments = 18
    for x, width, bottom, height in zip(xs, widths, bottoms, heights):
        points = []
        # Half-ellipse dome with a narrow lower lip; closed loop includes the seating chord.
        for i in range(segments + 1):
            theta = math.pi * i / segments
            y = -0.5 * width * math.cos(theta)
            z = bottom + height * math.sin(theta)
            points.append((x, y, z))
        points.append((x, 0.5 * width, bottom - 0.004))
        points.append((x, -0.5 * width, bottom - 0.004))
        loops.append(_add_loop(geom, points))
    _loft_between_loops(geom, loops)
    _cap_loop(geom, loops[0], reverse=True)
    _cap_loop(geom, loops[-1])
    return geom


def _propeller_blade_mesh() -> MeshGeometry:
    """Two connected, twisted wooden propeller blades spinning about local Z."""
    geom = MeshGeometry()

    def add_blade(sign: float) -> None:
        sections = [
            (0.020, 0.046, math.radians(26.0), 0.000),
            (0.076, 0.039, math.radians(19.0), 0.004),
            (0.145, 0.024, math.radians(10.0), 0.010),
        ]
        loops: list[list[int]] = []
        for radius, chord, pitch, sweep in sections:
            y = sign * radius
            x_center = sign * sweep
            thick = 0.006
            loop = []
            for x_rel, z_off in [
                (-0.5 * chord, -0.5 * thick),
                (0.5 * chord, -0.5 * thick),
                (0.5 * chord, 0.5 * thick),
                (-0.5 * chord, 0.5 * thick),
            ]:
                z_pitch = x_rel * math.tan(pitch)
                loop.append((x_center + x_rel, y, z_pitch + z_off))
            loops.append(_add_loop(geom, loop))
        _loft_between_loops(geom, loops)
        _cap_loop(geom, loops[-1])

    # Central wooden root block makes the two blades one continuous propeller part.
    root_loops = [
        _add_loop(
            geom,
            [
                (-0.022, -0.028, -0.004),
                (0.022, -0.028, -0.004),
                (0.022, -0.028, 0.004),
                (-0.022, -0.028, 0.004),
            ],
        ),
        _add_loop(
            geom,
            [
                (-0.022, 0.028, -0.004),
                (0.022, 0.028, -0.004),
                (0.022, 0.028, 0.004),
                (-0.022, 0.028, 0.004),
            ],
        ),
    ]
    _loft_between_loops(geom, root_loops)
    add_blade(1.0)
    add_blade(-1.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_propeller_model_airplane")

    painted_red = Material("painted_red", rgba=(0.78, 0.06, 0.04, 1.0))
    cream = Material("cream_trim", rgba=(0.95, 0.89, 0.74, 1.0))
    black = Material("rubber_black", rgba=(0.02, 0.018, 0.015, 1.0))
    grey = Material("brushed_grey", rgba=(0.48, 0.50, 0.52, 1.0))
    steel = Material("polished_spinner", rgba=(0.78, 0.80, 0.82, 1.0))
    blue_glass = Material("blue_tinted_canopy", rgba=(0.20, 0.46, 0.78, 0.58))
    wood = Material("varnished_wood", rgba=(0.58, 0.34, 0.13, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        mesh_from_geometry(_fuselage_mesh(), "fuselage_shell"),
        material=painted_red,
        name="fuselage_shell",
    )
    airframe.visual(
        mesh_from_geometry(
            _wing_mesh(
                span=0.780,
                root_leading=0.105,
                root_trailing=-0.155,
                tip_leading=0.025,
                tip_trailing=-0.125,
                z_root=0.247,
                dihedral=0.020,
                root_thickness=0.025,
                tip_thickness=0.014,
            ),
            "main_wing",
        ),
        material=cream,
        name="main_wing",
    )
    airframe.visual(
        mesh_from_geometry(
            _wing_mesh(
                span=0.270,
                root_leading=-0.218,
                root_trailing=-0.335,
                tip_leading=-0.238,
                tip_trailing=-0.330,
                z_root=0.281,
                dihedral=0.006,
                root_thickness=0.014,
                tip_thickness=0.009,
            ),
            "tailplane",
        ),
        material=cream,
        name="tailplane",
    )
    airframe.visual(
        mesh_from_geometry(_vertical_fin_mesh(), "vertical_fin"),
        material=painted_red,
        name="vertical_fin",
    )
    airframe.visual(
        mesh_from_geometry(_canopy_mesh(), "cockpit_canopy"),
        material=blue_glass,
        name="cockpit_canopy",
    )

    # Nose bearing and paired flanges/cheeks: fixed support around the spin axis.
    airframe.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.330, 0.0, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="nose_bearing",
    )
    for index, y in enumerate((-0.030, 0.030)):
        airframe.visual(
            Box((0.058, 0.010, 0.078)),
            origin=Origin(xyz=(0.334, y, 0.260)),
            material=grey,
            name=f"support_cheek_{index}",
        )

    # Simple display peg and base under the belly, attached with a saddle plate.
    airframe.visual(
        Box((0.084, 0.038, 0.012)),
        origin=Origin(xyz=(-0.030, 0.0, 0.224)),
        material=grey,
        name="belly_saddle",
    )
    airframe.visual(
        Cylinder(radius=0.012, length=0.205),
        origin=Origin(xyz=(-0.030, 0.0, 0.1275)),
        material=grey,
        name="display_peg",
    )
    airframe.visual(
        Cylinder(radius=0.078, length=0.025),
        origin=Origin(xyz=(-0.030, 0.0, 0.0125)),
        material=black,
        name="display_base",
    )

    propeller = model.part("propeller")
    propeller.visual(
        mesh_from_geometry(_propeller_blade_mesh(), "blade_pair"),
        material=wood,
        name="blade_pair",
    )
    propeller.visual(
        Cylinder(radius=0.028, length=0.028),
        material=steel,
        name="hub",
    )
    propeller.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=steel,
        name="shaft",
    )
    propeller.visual(
        mesh_from_geometry(
            LatheGeometry(
                [(0.000, 0.045), (0.013, 0.036), (0.026, 0.010), (0.026, -0.010)],
                segments=36,
                closed=True,
            ),
            "spinner",
        ),
        material=steel,
        name="spinner",
    )

    model.articulation(
        "nose_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        # Rotate the joint frame so the child's local +Z spin axis is the nose +X axis.
        origin=Origin(xyz=(0.385, 0.0, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=60.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    spin = object_model.get_articulation("nose_to_propeller")

    ctx.allow_overlap(
        propeller,
        airframe,
        elem_a="shaft",
        elem_b="nose_bearing",
        reason="The rotating propeller shaft is intentionally captured inside the fixed nose bearing.",
    )

    ctx.check(
        "propeller joint is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.expect_within(
        propeller,
        airframe,
        axes="yz",
        inner_elem="shaft",
        outer_elem="nose_bearing",
        margin=0.001,
        name="shaft runs through the bearing bore",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="x",
        elem_a="shaft",
        elem_b="nose_bearing",
        min_overlap=0.030,
        name="shaft remains captured in the bearing",
    )
    ctx.expect_gap(
        propeller,
        airframe,
        axis="x",
        positive_elem="hub",
        negative_elem="nose_bearing",
        min_gap=0.006,
        max_gap=0.030,
        name="hub sits just forward of the nose bearing",
    )
    ctx.expect_overlap(
        propeller,
        airframe,
        axes="yz",
        elem_a="hub",
        elem_b="nose_bearing",
        min_overlap=0.040,
        name="hub is centered on the bearing axis",
    )

    rest_aabb = ctx.part_element_world_aabb(propeller, elem="blade_pair")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(propeller, elem="blade_pair")
    if rest_aabb is not None and turned_aabb is not None:
        rest_y = rest_aabb[1][1] - rest_aabb[0][1]
        rest_z = rest_aabb[1][2] - rest_aabb[0][2]
        turned_y = turned_aabb[1][1] - turned_aabb[0][1]
        turned_z = turned_aabb[1][2] - turned_aabb[0][2]
        ctx.check(
            "propeller rotates about the nose axis",
            rest_y > rest_z * 2.0 and turned_z > turned_y * 2.0,
            details=f"rest_y={rest_y:.3f}, rest_z={rest_z:.3f}, turned_y={turned_y:.3f}, turned_z={turned_z:.3f}",
        )
    else:
        ctx.fail("propeller rotates about the nose axis", "blade_pair AABB unavailable")

    return ctx.report()


object_model = build_object_model()
