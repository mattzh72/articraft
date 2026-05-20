from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def _extrude_yz_profile(
    profile: list[tuple[float, float]], x_min: float, x_max: float
) -> MeshGeometry:
    """Extrude a simple YZ-plane profile between two X faces."""
    mesh = MeshGeometry()
    front = [mesh.add_vertex(x_max, y, z) for y, z in profile]
    back = [mesh.add_vertex(x_min, y, z) for y, z in profile]
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        _add_quad(mesh, front[i], front[j], back[j], back[i])

    cy = sum(y for y, _ in profile) / n
    cz = sum(z for _, z in profile) / n
    cf = mesh.add_vertex(x_max, cy, cz)
    cb = mesh.add_vertex(x_min, cy, cz)
    for i in range(n):
        j = (i + 1) % n
        mesh.add_face(cf, front[i], front[j])
        mesh.add_face(cb, back[j], back[i])
    return mesh


def _circle_profile(
    y: float, z: float, radius: float, *, segments: int = 36
) -> list[tuple[float, float]]:
    return [
        (
            y + radius * math.cos(2.0 * math.pi * i / segments),
            z + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(
    start_z: float,
    end_z: float,
    start_radius: float,
    end_radius: float,
    *,
    segments: int = 36,
) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    samples.extend(_circle_profile(0.0, start_z, start_radius, segments=segments))
    samples.extend(_circle_profile(0.0, end_z, end_radius, segments=segments))
    return _convex_hull(samples)


def _cylinder_x_mesh(
    radius: float,
    x_min: float,
    x_max: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
    segments: int = 48,
) -> MeshGeometry:
    mesh = MeshGeometry()
    right = []
    left = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        yy = y + radius * math.cos(a)
        zz = z + radius * math.sin(a)
        right.append(mesh.add_vertex(x_max, yy, zz))
        left.append(mesh.add_vertex(x_min, yy, zz))

    cr = mesh.add_vertex(x_max, y, z)
    cl = mesh.add_vertex(x_min, y, z)
    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, right[i], right[j], left[j], left[i])
        mesh.add_face(cr, right[i], right[j])
        mesh.add_face(cl, left[j], left[i])
    return mesh


def _tapered_spindle_mesh() -> MeshGeometry:
    """A road bottom-bracket spindle with visibly wider tapered ends."""
    stations = [
        (-0.086, 0.0165),
        (-0.055, 0.0128),
        (0.055, 0.0128),
        (0.086, 0.0165),
    ]
    segments = 64
    mesh = MeshGeometry()
    loops: list[list[int]] = []
    for x, radius in stations:
        loop = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            loop.append(
                mesh.add_vertex(x, radius * math.cos(a), radius * math.sin(a))
            )
        loops.append(loop)

    for k in range(len(loops) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            _add_quad(mesh, loops[k][i], loops[k][j], loops[k + 1][j], loops[k + 1][i])

    c0 = mesh.add_vertex(stations[0][0], 0.0, 0.0)
    c1 = mesh.add_vertex(stations[-1][0], 0.0, 0.0)
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(c0, loops[0][j], loops[0][i])
        mesh.add_face(c1, loops[-1][i], loops[-1][j])
    return mesh


def _annular_x_mesh(
    inner_radius: float,
    outer_radius: float,
    thickness: float,
    *,
    segments: int = 96,
    teeth: int = 0,
    tooth_depth: float = 0.0,
) -> MeshGeometry:
    """An annular plate in the YZ plane with optional chainring teeth."""
    steps = max(segments, teeth * 4 if teeth else 0)
    mesh = MeshGeometry()
    outer_r: list[float] = []
    for i in range(steps):
        if teeth:
            phase = i % 4
            outer_r.append(outer_radius + (tooth_depth if phase in (1, 2) else 0.0))
        else:
            outer_r.append(outer_radius)

    xf = thickness / 2.0
    xb = -thickness / 2.0
    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []
    for i in range(steps):
        a = 2.0 * math.pi * i / steps
        ca = math.cos(a)
        sa = math.sin(a)
        r = outer_r[i]
        outer_front.append(mesh.add_vertex(xf, r * ca, r * sa))
        outer_back.append(mesh.add_vertex(xb, r * ca, r * sa))
        inner_front.append(mesh.add_vertex(xf, inner_radius * ca, inner_radius * sa))
        inner_back.append(mesh.add_vertex(xb, inner_radius * ca, inner_radius * sa))

    for i in range(steps):
        j = (i + 1) % steps
        _add_quad(mesh, outer_front[i], outer_front[j], outer_back[j], outer_back[i])
        _add_quad(mesh, inner_front[j], inner_front[i], inner_back[i], inner_back[j])
        _add_quad(mesh, outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        _add_quad(mesh, outer_back[j], outer_back[i], inner_back[i], inner_back[j])
    return mesh


def _spider_mesh() -> MeshGeometry:
    mesh = MeshGeometry()
    # Central web around the spindle.
    mesh.merge(_extrude_yz_profile(_circle_profile(0.0, 0.0, 0.030), 0.056, 0.086))

    # Five radial arms, with a broader lower arm blending into the right crank.
    bolt_radius = 0.069
    for k in range(5):
        angle = math.radians(90.0 + k * 72.0)
        y = bolt_radius * math.cos(angle)
        z = bolt_radius * math.sin(angle)
        width = 0.013 if k != 2 else 0.016
        samples: list[tuple[float, float]] = []
        samples.extend(_circle_profile(0.0, 0.0, 0.013, segments=24))
        samples.extend(_circle_profile(y, z, width, segments=24))
        radial_profile = _convex_hull(samples)
        mesh.merge(_extrude_yz_profile(radial_profile, 0.060, 0.084))
        mesh.merge(_extrude_yz_profile(_circle_profile(y, z, 0.014, segments=30), 0.054, 0.090))
    return mesh


def _crank_arm_mesh(x_min: float, x_max: float, end_z: float) -> MeshGeometry:
    profile = _capsule_profile(0.0, end_z, 0.024, 0.020, segments=40)
    return _extrude_yz_profile(profile, x_min, x_max)


def _pedal_body_mesh(sign: float) -> MeshGeometry:
    mesh = MeshGeometry()
    # Threaded axle from the crank face into the pedal body.
    mesh.merge(_cylinder_x_mesh(0.0042, min(0.0, 0.040 * sign), max(0.0, 0.040 * sign)))
    # Compact road-pedal platform, with shallow fore/aft bevels.
    x0 = 0.026 * sign
    x1 = 0.076 * sign
    mesh.merge(
        _extrude_yz_profile(
            [
                (-0.044, -0.006),
                (-0.034, -0.014),
                (0.034, -0.014),
                (0.044, -0.006),
                (0.040, 0.010),
                (0.022, 0.016),
                (-0.022, 0.016),
                (-0.040, 0.010),
            ],
            min(x0, x1),
            max(x0, x1),
        )
    )
    return mesh


def _tread_ridge_mesh(sign: float, y: float) -> MeshGeometry:
    x0 = min(0.035 * sign, 0.068 * sign)
    x1 = max(0.035 * sign, 0.068 * sign)
    return _extrude_yz_profile(
        [(y - 0.002, 0.010), (y + 0.002, 0.010), (y + 0.002, 0.020), (y - 0.002, 0.020)],
        x0,
        x1,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_double_crankset")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.05, 0.05, 0.045, 1.0))
    graphite = model.material("graphite_chainring", rgba=(0.09, 0.095, 0.09, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    steel = model.material("polished_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    bearing_grey = model.material("bearing_grey", rgba=(0.30, 0.31, 0.32, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_geometry(
            _annular_x_mesh(0.0205, 0.030, 0.090, segments=96),
            "bearing_shell",
        ),
        material=bearing_grey,
        name="bearing_shell",
    )
    ball_radius = 0.00385
    ball_center_radius = 0.0128 + ball_radius
    for i, x in enumerate((-0.026, 0.026)):
        for j, angle in enumerate((math.pi / 2.0, 3.0 * math.pi / 2.0)):
            bottom_bracket.visual(
                Sphere(ball_radius),
                origin=Origin(
                    xyz=(
                        x,
                        ball_center_radius * math.cos(angle),
                        ball_center_radius * math.sin(angle),
                    )
                ),
                material=steel,
                name=f"bearing_ball_{i}_{j}",
            )

    crankset = model.part("crankset")
    crankset.visual(
        mesh_from_geometry(_tapered_spindle_mesh(), "tapered_spindle"),
        material=steel,
        name="tapered_spindle",
    )
    crankset.visual(
        mesh_from_geometry(_crank_arm_mesh(0.052, 0.068, -0.170), "right_crank_arm"),
        material=aluminum,
        name="right_crank_arm",
    )
    crankset.visual(
        mesh_from_geometry(_crank_arm_mesh(-0.068, -0.052, 0.170), "crank_arm"),
        material=aluminum,
        name="crank_arm",
    )
    crankset.visual(
        mesh_from_geometry(_spider_mesh(), "right_spider"),
        material=aluminum,
        name="right_spider",
    )
    crankset.visual(
        mesh_from_geometry(
            _annular_x_mesh(
                0.078,
                0.098,
                0.0032,
                segments=160,
                teeth=50,
                tooth_depth=0.005,
            ),
            "outer_chainring",
        ),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=graphite,
        name="outer_chainring",
    )
    crankset.visual(
        mesh_from_geometry(
            _annular_x_mesh(
                0.050,
                0.069,
                0.0028,
                segments=128,
                teeth=34,
                tooth_depth=0.004,
            ),
            "inner_chainring",
        ),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=graphite,
        name="inner_chainring",
    )
    crankset.visual(
        mesh_from_geometry(
            _cylinder_x_mesh(0.009, 0.068, 0.074, z=-0.170, segments=36),
            "pedal_boss_0",
        ),
        material=steel,
        name="pedal_boss_0",
    )
    crankset.visual(
        mesh_from_geometry(
            _cylinder_x_mesh(0.009, -0.074, -0.068, z=0.170, segments=36),
            "pedal_boss_1",
        ),
        material=steel,
        name="pedal_boss_1",
    )
    for k in range(5):
        angle = math.radians(90.0 + k * 72.0)
        y = 0.069 * math.cos(angle)
        z = 0.069 * math.sin(angle)
        crankset.visual(
            mesh_from_geometry(
                _cylinder_x_mesh(0.0045, 0.087, 0.092, y=y, z=z, segments=24),
                f"chainring_bolt_{k}",
            ),
            material=steel,
            name=f"chainring_bolt_{k}",
        )

    pedal_0 = model.part("pedal_0")
    pedal_0.visual(
        mesh_from_geometry(_pedal_body_mesh(1.0), "pedal_body_0"),
        material=satin_black,
        name="pedal_body",
    )
    for i, y in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        pedal_0.visual(
            mesh_from_geometry(_tread_ridge_mesh(1.0, y), f"pedal_tread_0_{i}"),
            material=matte_black,
            name=f"tread_{i}",
        )

    pedal_1 = model.part("pedal_1")
    pedal_1.visual(
        mesh_from_geometry(_pedal_body_mesh(-1.0), "pedal_body_1"),
        material=satin_black,
        name="pedal_body",
    )
    for i, y in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        pedal_1.visual(
            mesh_from_geometry(_tread_ridge_mesh(-1.0, y), f"pedal_tread_1_{i}"),
            material=matte_black,
            name=f"tread_{i}",
        )

    model.articulation(
        "bracket_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "crankset_to_pedal_0",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=pedal_0,
        origin=Origin(xyz=(0.074, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    model.articulation(
        "crankset_to_pedal_1",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=pedal_1,
        origin=Origin(xyz=(-0.074, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crankset = object_model.get_part("crankset")
    bottom_bracket = object_model.get_part("bottom_bracket")
    pedal_0 = object_model.get_part("pedal_0")
    pedal_1 = object_model.get_part("pedal_1")
    crank_joint = object_model.get_articulation("bracket_to_crankset")
    pedal_joint_0 = object_model.get_articulation("crankset_to_pedal_0")
    pedal_joint_1 = object_model.get_articulation("crankset_to_pedal_1")

    ctx.check(
        "spindle and pedals use continuous rotation",
        all(
            j.articulation_type == ArticulationType.CONTINUOUS
            for j in (crank_joint, pedal_joint_0, pedal_joint_1)
        ),
        details=(
            f"joint types: {crank_joint.articulation_type}, "
            f"{pedal_joint_0.articulation_type}, {pedal_joint_1.articulation_type}"
        ),
    )
    ctx.check(
        "all rotation axes follow the spindle",
        all(abs(j.axis[0]) > 0.99 for j in (crank_joint, pedal_joint_0, pedal_joint_1)),
        details=f"axes: {crank_joint.axis}, {pedal_joint_0.axis}, {pedal_joint_1.axis}",
    )

    ctx.expect_within(
        crankset,
        bottom_bracket,
        axes="yz",
        inner_elem="tapered_spindle",
        outer_elem="bearing_shell",
        margin=0.0,
        name="tapered spindle fits inside bearing shell bore envelope",
    )
    ctx.expect_overlap(
        crankset,
        bottom_bracket,
        axes="x",
        elem_a="tapered_spindle",
        elem_b="bearing_shell",
        min_overlap=0.085,
        name="spindle passes through bottom bracket shell",
    )
    ctx.expect_contact(
        pedal_0,
        crankset,
        elem_a="pedal_body",
        elem_b="pedal_boss_0",
        contact_tol=0.001,
        name="right pedal axle seats on crank boss",
    )
    ctx.expect_contact(
        pedal_1,
        crankset,
        elem_a="pedal_body",
        elem_b="pedal_boss_1",
        contact_tol=0.001,
        name="opposite pedal axle seats on crank boss",
    )

    outer_aabb = ctx.part_element_world_aabb(crankset, elem="outer_chainring")
    inner_aabb = ctx.part_element_world_aabb(crankset, elem="inner_chainring")
    if outer_aabb is not None and inner_aabb is not None:
        outer_min, outer_max = outer_aabb
        inner_min, inner_max = inner_aabb
        outer_diameter = max(outer_max[1] - outer_min[1], outer_max[2] - outer_min[2])
        inner_diameter = max(inner_max[1] - inner_min[1], inner_max[2] - inner_min[2])
        outer_x = (outer_min[0] + outer_max[0]) / 2.0
        inner_x = (inner_min[0] + inner_max[0]) / 2.0
        ctx.check(
            "outer chainring is larger and farther outboard",
            outer_diameter > inner_diameter + 0.045 and outer_x > inner_x + 0.010,
            details=(
                f"outer diameter={outer_diameter:.3f}, inner diameter={inner_diameter:.3f}, "
                f"outer_x={outer_x:.3f}, inner_x={inner_x:.3f}"
            ),
        )
    else:
        ctx.fail("chainring aabb available", f"outer={outer_aabb}, inner={inner_aabb}")

    with ctx.pose({crank_joint: 1.0, pedal_joint_0: 0.7, pedal_joint_1: -0.8}):
        ctx.expect_overlap(
            crankset,
            bottom_bracket,
            axes="x",
            elem_a="tapered_spindle",
            elem_b="bearing_shell",
            min_overlap=0.085,
            name="rotated spindle remains captured by bearing shell",
        )

    return ctx.report()


object_model = build_object_model()
