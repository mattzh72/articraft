from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _annular_shell_x(
    stations: list[tuple[float, float, float]],
    *,
    theta_start: float = 0.0,
    theta_end: float = math.tau,
    segments: int = 96,
) -> MeshGeometry:
    """Hollow conical/cylindrical shell with engine axis along +X."""

    full = abs((theta_end - theta_start) - math.tau) < 1.0e-6
    steps = segments if full else segments + 1
    angles = [
        theta_start + (theta_end - theta_start) * i / segments
        for i in range(steps)
    ]

    mesh = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, outer_r, inner_r in stations:
        outer_row: list[int] = []
        inner_row: list[int] = []
        for theta in angles:
            c = math.cos(theta)
            s = math.sin(theta)
            outer_row.append(mesh.add_vertex(x, outer_r * c, outer_r * s))
            inner_row.append(mesh.add_vertex(x, inner_r * c, inner_r * s))
        outer.append(outer_row)
        inner.append(inner_row)

    ring_count = len(angles)
    face_count = ring_count if full else ring_count - 1

    for i in range(len(stations) - 1):
        for j in range(face_count):
            k = (j + 1) % ring_count
            _quad(mesh, outer[i][j], outer[i + 1][j], outer[i + 1][k], outer[i][k])
            _quad(mesh, inner[i][k], inner[i + 1][k], inner[i + 1][j], inner[i][j])

    # Axial end faces of the shell.
    for row in (0, len(stations) - 1):
        for j in range(face_count):
            k = (j + 1) % ring_count
            if row == 0:
                _quad(mesh, inner[row][j], outer[row][j], outer[row][k], inner[row][k])
            else:
                _quad(mesh, outer[row][j], inner[row][j], inner[row][k], outer[row][k])

    # Radial cut faces for cutaway sectors.
    if not full:
        for j in (0, ring_count - 1):
            for i in range(len(stations) - 1):
                _quad(mesh, outer[i][j], inner[i][j], inner[i + 1][j], outer[i + 1][j])

    return mesh


def _radial_prism_x(
    *,
    x_center: float,
    x_length: float,
    r_inner: float,
    r_outer: float,
    theta: float,
    tangent_width: float,
    lean: float = 0.0,
) -> MeshGeometry:
    """A single thick radial strut or stator vane."""

    mesh = MeshGeometry()
    xs = (x_center - x_length / 2.0, x_center + x_length / 2.0)
    rs = (r_inner, r_outer)
    ss = (-tangent_width / 2.0, tangent_width / 2.0)
    verts: dict[tuple[int, int, int], int] = {}

    for ix, x in enumerate(xs):
        for ir, r in enumerate(rs):
            local_theta = theta + lean * (r - r_inner) / max(r_outer - r_inner, 1e-6)
            er = (math.cos(local_theta), math.sin(local_theta))
            et = (-math.sin(local_theta), math.cos(local_theta))
            for iside, t in enumerate(ss):
                y = r * er[0] + t * et[0]
                z = r * er[1] + t * et[1]
                verts[(ix, ir, iside)] = mesh.add_vertex(x, y, z)

    _quad(mesh, verts[(0, 0, 0)], verts[(1, 0, 0)], verts[(1, 1, 0)], verts[(0, 1, 0)])
    _quad(mesh, verts[(0, 1, 1)], verts[(1, 1, 1)], verts[(1, 0, 1)], verts[(0, 0, 1)])
    _quad(mesh, verts[(0, 0, 1)], verts[(1, 0, 1)], verts[(1, 0, 0)], verts[(0, 0, 0)])
    _quad(mesh, verts[(0, 1, 0)], verts[(1, 1, 0)], verts[(1, 1, 1)], verts[(0, 1, 1)])
    _quad(mesh, verts[(0, 0, 0)], verts[(0, 1, 0)], verts[(0, 1, 1)], verts[(0, 0, 1)])
    _quad(mesh, verts[(1, 0, 1)], verts[(1, 1, 1)], verts[(1, 1, 0)], verts[(1, 0, 0)])
    return mesh


def _stator_cage_x(
    *,
    x: float,
    inner_radius: float,
    outer_radius: float,
    count: int,
    vane_length: float,
    vane_width: float,
    lean: float,
) -> MeshGeometry:
    mesh = MeshGeometry()
    mesh.merge(
        _annular_shell_x(
            [(x - 0.018, outer_radius + 0.012, outer_radius), (x + 0.018, outer_radius + 0.012, outer_radius)],
            segments=96,
        )
    )
    mesh.merge(
        _annular_shell_x(
            [(x - 0.018, inner_radius, inner_radius - 0.030), (x + 0.018, inner_radius, inner_radius - 0.030)],
            segments=96,
        )
    )
    for i in range(count):
        theta = math.tau * i / count
        mesh.merge(
            _radial_prism_x(
                x_center=x,
                x_length=vane_length,
                r_inner=inner_radius - 0.006,
                r_outer=outer_radius + 0.006,
                theta=theta,
                tangent_width=vane_width,
                lean=lean,
            )
        )
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cutaway_turbofan_engine")

    ceramic_white = model.material("ceramic_white", rgba=(0.82, 0.84, 0.82, 1.0))
    graphite = model.material("graphite_liner", rgba=(0.035, 0.039, 0.043, 1.0))
    titanium = model.material("brushed_titanium", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_titanium = model.material("dark_titanium", rgba=(0.23, 0.25, 0.27, 1.0))
    hot_steel = model.material("heat_stained_steel", rgba=(0.50, 0.40, 0.31, 1.0))
    copper = model.material("burner_copper", rgba=(0.78, 0.43, 0.18, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.011, 0.012, 1.0))

    casing = model.part("casing")

    cut_start = math.radians(145.0)
    cut_end = math.radians(395.0)
    nacelle = _annular_shell_x(
        [
            (-1.62, 0.68, 0.55),
            (-1.42, 0.82, 0.60),
            (-0.95, 0.83, 0.65),
            (-0.20, 0.78, 0.61),
            (0.85, 0.66, 0.49),
            (1.48, 0.50, 0.34),
            (1.70, 0.39, 0.25),
        ],
        theta_start=cut_start,
        theta_end=cut_end,
        segments=92,
    )
    casing.visual(mesh_from_geometry(nacelle, "nacelle_cutaway_shell"), material=ceramic_white, name="nacelle_shell")

    core_shell = _annular_shell_x(
        [
            (-0.72, 0.39, 0.325),
            (-0.35, 0.36, 0.315),
            (0.35, 0.36, 0.305),
            (0.95, 0.34, 0.255),
            (1.45, 0.30, 0.205),
        ],
        theta_start=cut_start,
        theta_end=cut_end,
        segments=88,
    )
    casing.visual(mesh_from_geometry(core_shell, "core_cutaway_shell"), material=dark_titanium, name="core_shell")

    # Fan outlet guide vanes and engine mounts visibly bridge the nacelle to the core.
    fan_frame = _stator_cage_x(
        x=-0.90,
        inner_radius=0.310,
        outer_radius=0.638,
        count=10,
        vane_length=0.095,
        vane_width=0.032,
        lean=0.12,
    )
    fan_frame.merge(
        _radial_prism_x(
            x_center=-0.18,
            x_length=0.16,
            r_inner=0.34,
            r_outer=0.63,
            theta=math.radians(270.0),
            tangent_width=0.060,
            lean=-0.04,
        )
    )
    casing.visual(mesh_from_geometry(fan_frame, "fan_frame_struts"), material=titanium, name="fan_frame")

    # Compressor stator rows and turbine nozzle guide vanes, exposed by the cutaway.
    for idx, x in enumerate((-0.26, -0.08, 0.10, 0.28)):
        cage = _stator_cage_x(
            x=x,
            inner_radius=0.145,
            outer_radius=0.305,
            count=22,
            vane_length=0.040,
            vane_width=0.010,
            lean=-0.32,
        )
        casing.visual(mesh_from_geometry(cage, f"compressor_stator_{idx}"), material=titanium, name=f"compressor_stator_{idx}")

    bearing = _stator_cage_x(
        x=-0.64,
        inner_radius=0.085,
        outer_radius=0.326,
        count=6,
        vane_length=0.050,
        vane_width=0.018,
        lean=0.0,
    )
    casing.visual(mesh_from_geometry(bearing, "front_bearing_carrier"), material=dark_titanium, name="front_bearing")

    for idx, x in enumerate((1.03, 1.25)):
        cage = _stator_cage_x(
            x=x,
            inner_radius=0.125,
            outer_radius=0.245,
            count=24,
            vane_length=0.045,
            vane_width=0.012,
            lean=0.42,
        )
        casing.visual(mesh_from_geometry(cage, f"turbine_nozzle_{idx}"), material=hot_steel, name=f"turbine_nozzle_{idx}")

    combustor = _annular_shell_x(
        [
            (0.48, 0.285, 0.210),
            (0.72, 0.305, 0.190),
            (0.96, 0.250, 0.165),
        ],
        theta_start=math.radians(160.0),
        theta_end=math.radians(380.0),
        segments=64,
    )
    casing.visual(mesh_from_geometry(combustor, "annular_combustor_liner"), material=hot_steel, name="combustor_liner")

    injectors = _stator_cage_x(
        x=0.63,
        inner_radius=0.275,
        outer_radius=0.323,
        count=12,
        vane_length=0.045,
        vane_width=0.012,
        lean=0.0,
    )
    casing.visual(mesh_from_geometry(injectors, "fuel_injector_bosses"), material=copper, name="fuel_injectors")

    exhaust_shadow = _annular_shell_x(
        [(1.42, 0.230, 0.080), (1.72, 0.180, 0.075)],
        theta_start=cut_start,
        theta_end=cut_end,
        segments=64,
    )
    casing.visual(mesh_from_geometry(exhaust_shadow, "exhaust_shadow_liner"), material=black, name="exhaust_shadow")

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=2.86),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_titanium,
        name="main_shaft",
    )

    fan = FanRotorGeometry(
        0.595,
        0.155,
        24,
        thickness=0.145,
        blade_pitch_deg=38.0,
        blade_sweep_deg=33.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.18, tip_clearance=0.012),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.045, rear_collar_radius=0.140),
    )
    rotor.visual(
        mesh_from_geometry(fan, "wide_chord_fan"),
        origin=Origin(xyz=(-1.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_blisk",
    )

    for idx, (x, radius, hub_radius, blade_count) in enumerate(
        [
            (-0.43, 0.250, 0.088, 20),
            (-0.21, 0.275, 0.090, 22),
            (0.01, 0.292, 0.092, 24),
            (0.23, 0.282, 0.092, 24),
            (0.43, 0.250, 0.090, 22),
        ]
    ):
        stage = FanRotorGeometry(
            radius,
            hub_radius,
            blade_count,
            thickness=0.052,
            blade_pitch_deg=48.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="narrow", tip_pitch_deg=26.0, camber=0.11, tip_clearance=0.006),
            hub=FanRotorHub(style="capped", rear_collar_height=0.018, rear_collar_radius=hub_radius * 0.92),
        )
        rotor.visual(
            mesh_from_geometry(stage, f"compressor_rotor_{idx}"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=titanium,
            name=f"compressor_stage_{idx}",
        )

    for idx, (x, radius) in enumerate(((1.13, 0.226), (1.36, 0.198))):
        turbine = FanRotorGeometry(
            radius,
            0.082,
            28,
            thickness=0.060,
            blade_pitch_deg=55.0,
            blade_sweep_deg=-12.0,
            blade=FanRotorBlade(shape="broad", tip_pitch_deg=34.0, camber=0.20, tip_clearance=0.005),
            hub=FanRotorHub(style="capped", rear_collar_height=0.025, rear_collar_radius=0.076),
        )
        rotor.visual(
            mesh_from_geometry(turbine, f"turbine_rotor_{idx}"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hot_steel,
            name=f"turbine_stage_{idx}",
        )

    model.articulation(
        "casing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=casing,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20000.0, velocity=900.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.002),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    casing = object_model.get_part("casing")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("casing_to_rotor")

    ctx.allow_overlap(
        casing,
        rotor,
        elem_a="front_bearing",
        elem_b="main_shaft",
        reason="The shaft is intentionally captured inside the front bearing sleeve; the sleeve mesh is a visual proxy for a ball-bearing race.",
    )
    ctx.expect_within(
        rotor,
        casing,
        axes="yz",
        inner_elem="main_shaft",
        outer_elem="front_bearing",
        margin=0.001,
        name="shaft is centered in front bearing",
    )
    ctx.expect_overlap(
        rotor,
        casing,
        axes="x",
        elem_a="main_shaft",
        elem_b="front_bearing",
        min_overlap=0.030,
        name="shaft remains captured through front bearing",
    )
    ctx.expect_within(
        rotor,
        casing,
        axes="y",
        inner_elem="fan_blisk",
        outer_elem="nacelle_shell",
        margin=0.035,
        name="fan disk sits inside nacelle side clearance",
    )
    ctx.expect_within(
        rotor,
        casing,
        axes="y",
        inner_elem="compressor_stage_2",
        outer_elem="core_shell",
        margin=0.025,
        name="compressor is contained by core side case",
    )
    rest = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 3.0}):
        turned = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            casing,
            axes="y",
            inner_elem="fan_blisk",
            outer_elem="nacelle_shell",
            margin=0.035,
            name="rotating fan remains within nacelle side clearance",
        )
    ctx.check(
        "rotor spins about fixed engine centerline",
        rest is not None and turned is not None and all(abs(a - b) < 1e-9 for a, b in zip(rest, turned)),
        details=f"rest={rest}, turned={turned}",
    )

    return ctx.report()


object_model = build_object_model()
