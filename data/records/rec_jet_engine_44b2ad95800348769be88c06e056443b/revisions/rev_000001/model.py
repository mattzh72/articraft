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
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


ENGINE_FRONT_X = -1.50
ENGINE_REAR_X = 1.50
ROTOR_X = -1.330
ROTOR_THICKNESS = 0.048
ROTOR_HUB_LENGTH = 0.070
ROTOR_OUTER_RADIUS = 0.300
ROTOR_BORE_RADIUS = 0.063
RETAINER_LENGTH = 0.026


def _annular_shell_x_mesh(
    stations: list[tuple[float, float, float]], *, segments: int = 72
) -> MeshGeometry:
    """Build a thin, open-ended axisymmetric shell around the local X axis.

    Each station is (x, outer_radius, inner_radius).  The mesh includes the
    outer skin, the inner duct wall, and annular lips at the first/last stations.
    """

    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, outer_r, inner_r in stations:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            c, s = math.cos(theta), math.sin(theta)
            outer_ring.append(geom.add_vertex(x, outer_r * c, outer_r * s))
            inner_ring.append(geom.add_vertex(x, inner_r * c, inner_r * s))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for station_i in range(len(stations) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            # Outer casing.
            geom.add_face(outer[station_i][i], outer[station_i + 1][i], outer[station_i + 1][j])
            geom.add_face(outer[station_i][i], outer[station_i + 1][j], outer[station_i][j])
            # Inner duct wall; reverse winding relative to the outer skin.
            geom.add_face(inner[station_i][i], inner[station_i][j], inner[station_i + 1][j])
            geom.add_face(inner[station_i][i], inner[station_i + 1][j], inner[station_i + 1][i])

    # Front and rear annular lips close the wall thickness without covering the duct.
    first = 0
    last = len(stations) - 1
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer[first][i], outer[first][j], inner[first][j])
        geom.add_face(outer[first][i], inner[first][j], inner[first][i])
        geom.add_face(outer[last][i], inner[last][j], outer[last][j])
        geom.add_face(outer[last][i], inner[last][i], inner[last][j])
    return geom


def _lathe_x_mesh(profile: list[tuple[float, float]], *, segments: int = 72) -> MeshGeometry:
    """Revolve an (x, radius) profile into a solid mesh about the X axis."""

    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, radius in profile:
        if radius <= 1e-9:
            center = geom.add_vertex(x, 0.0, 0.0)
            rings.append([center] * segments)
            continue
        ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(x, radius * math.cos(theta), radius * math.sin(theta)))
        rings.append(ring)

    for station_i in range(len(profile) - 1):
        r0 = profile[station_i][1]
        r1 = profile[station_i + 1][1]
        for i in range(segments):
            j = (i + 1) % segments
            if r0 <= 1e-9 and r1 <= 1e-9:
                continue
            if r0 <= 1e-9:
                geom.add_face(rings[station_i][i], rings[station_i + 1][i], rings[station_i + 1][j])
            elif r1 <= 1e-9:
                geom.add_face(rings[station_i][i], rings[station_i + 1][i], rings[station_i][j])
            else:
                geom.add_face(rings[station_i][i], rings[station_i + 1][i], rings[station_i + 1][j])
                geom.add_face(rings[station_i][i], rings[station_i + 1][j], rings[station_i][j])

    if profile[0][1] > 1e-9:
        center = geom.add_vertex(profile[0][0], 0.0, 0.0)
        for i in range(segments):
            geom.add_face(center, rings[0][(i + 1) % segments], rings[0][i])
    if profile[-1][1] > 1e-9:
        center = geom.add_vertex(profile[-1][0], 0.0, 0.0)
        for i in range(segments):
            geom.add_face(center, rings[-1][i], rings[-1][(i + 1) % segments])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_bypass_military_turbojet")

    gunmetal = model.material("burnished_gunmetal", rgba=(0.23, 0.25, 0.25, 1.0))
    dark_metal = model.material("dark_intake_metal", rgba=(0.035, 0.038, 0.040, 1.0))
    blued_nozzle = model.material("heat_blued_nozzle", rgba=(0.18, 0.20, 0.23, 1.0))
    blade_steel = model.material("compressor_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    cone_finish = model.material("matte_centerbody", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("engine_body")

    body_stations = [
        (ENGINE_FRONT_X, 0.455, 0.340),
        (-1.420, 0.480, 0.350),
        (-1.240, 0.440, 0.330),
        (-0.820, 0.410, 0.315),
        (0.820, 0.420, 0.300),
        (1.120, 0.390, 0.275),
        (1.340, 0.335, 0.245),
        (ENGINE_REAR_X, 0.285, 0.215),
    ]
    body.visual(
        mesh_from_geometry(_annular_shell_x_mesh(body_stations), "turbojet_casing_shell"),
        material=gunmetal,
        name="casing_shell",
    )

    # Rounded annular intake lip and restrained casing bands make the nacelle read
    # as a military turbojet casing rather than a generic cylinder.
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.455, tube=0.018, radial_segments=18, tubular_segments=72), "intake_lip"),
        origin=Origin(xyz=(ENGINE_FRONT_X + 0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="intake_lip",
    )
    for idx, x_pos in enumerate((-0.72, 0.10, 0.88)):
        body.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.423, tube=0.014, radial_segments=12, tubular_segments=64),
                f"casing_band_{idx}",
            ),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"casing_band_{idx}",
        )

    # The plain rear converging nozzle is part of the stationary body.  It is an
    # unadorned taper with a dark throat, deliberately not a vectored nozzle.
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.250, tube=0.010, radial_segments=12, tubular_segments=64), "nozzle_exit_lip"),
        origin=Origin(xyz=(ENGINE_REAR_X - 0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blued_nozzle,
        name="nozzle_lip",
    )

    centerbody_profile = [
        (ENGINE_FRONT_X + 0.030, 0.000),
        (-1.405, 0.040),
        (-1.250, 0.078),
        (-1.070, 0.125),
        (-0.935, 0.125),
        (-0.875, 0.070),
        (-0.720, 0.070),
    ]
    body.visual(
        mesh_from_geometry(_lathe_x_mesh(centerbody_profile), "pointed_intake_cone"),
        material=cone_finish,
        name="intake_cone",
    )

    # Four radial vanes physically carry the centerbody into the intake duct.
    for name, xyz, size in (
        ("upper_strut", (-1.075, 0.0, 0.220), (0.080, 0.034, 0.245)),
        ("lower_strut", (-1.075, 0.0, -0.220), (0.080, 0.034, 0.245)),
        ("side_strut_0", (-1.075, 0.220, 0.0), (0.080, 0.245, 0.034)),
        ("side_strut_1", (-1.075, -0.220, 0.0), (0.080, 0.245, 0.034)),
    ):
        body.visual(Box(size), origin=Origin(xyz=xyz), material=cone_finish, name=name)

    # Retaining collars on the fixed center shaft bracket the rotor axially.
    body.visual(
        Cylinder(radius=0.050, length=0.620),
        origin=Origin(xyz=(-1.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cone_finish,
        name="center_shaft",
    )
    retainer_offset = ROTOR_HUB_LENGTH / 2.0 + RETAINER_LENGTH / 2.0
    body.visual(
        Cylinder(radius=0.124, length=RETAINER_LENGTH),
        origin=Origin(xyz=(ROTOR_X - retainer_offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cone_finish,
        name="front_collar",
    )
    body.visual(
        Cylinder(radius=0.124, length=RETAINER_LENGTH),
        origin=Origin(xyz=(ROTOR_X + retainer_offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cone_finish,
        name="rear_collar",
    )

    rotor = model.part("compressor_rotor")
    rotor.visual(
        mesh_from_geometry(
            _annular_shell_x_mesh(
                [
                    (-ROTOR_HUB_LENGTH / 2.0, 0.145, 0.075),
                    (ROTOR_HUB_LENGTH / 2.0, 0.145, 0.075),
                ],
                segments=72,
            ),
            "rotor_hub_ring",
        ),
        material=blade_steel,
        name="rotor_hub_ring",
    )
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_OUTER_RADIUS,
                0.150,
                14,
                thickness=ROTOR_THICKNESS,
                blade_pitch_deg=34.0,
                blade_sweep_deg=26.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=18.0, camber=0.10, tip_clearance=0.006),
                hub=FanRotorHub(style="flat", bore_diameter=ROTOR_BORE_RADIUS * 2.0),
            ),
            "front_compressor_rotor",
        ),
        # FanRotorGeometry spins around local Z; rotate its disk so that local Z
        # coincides with the engine's X axis.
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_steel,
        name="rotor_disk",
    )

    model.articulation(
        "body_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(ROTOR_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=220.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("engine_body")
    rotor = object_model.get_part("compressor_rotor")
    spin = object_model.get_articulation("body_to_rotor")

    ctx.check(
        "compressor rotor uses continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type!r}",
    )
    ctx.check("compressor rotor spins about engine axis", tuple(spin.axis) == (1.0, 0.0, 0.0), details=f"axis={spin.axis!r}")

    rotor_aabb = ctx.part_world_aabb(rotor)
    if rotor_aabb is not None:
        mins, maxs = rotor_aabb
        rotor_size = tuple(maxs[i] - mins[i] for i in range(3))
        ctx.check(
            "rotor is broad and thin like a compressor disk",
            rotor_size[0] < 0.090 and rotor_size[1] > 0.550 and rotor_size[2] > 0.550,
            details=f"rotor_size={rotor_size!r}",
        )
    else:
        ctx.fail("rotor world aabb available", "Expected an AABB for the compressor rotor.")

    ctx.expect_within(
        rotor,
        body,
        axes="yz",
        margin=0.0,
        name="compressor rotor fits inside intake duct diameter",
    )
    ctx.expect_overlap(
        body,
        rotor,
        axes="x",
        elem_a="center_shaft",
        elem_b="rotor_hub_ring",
        min_overlap=0.060,
        name="stationary shaft passes through rotor hub length",
    )
    ctx.expect_within(
        body,
        rotor,
        axes="yz",
        inner_elem="center_shaft",
        outer_elem="rotor_hub_ring",
        margin=0.0,
        name="shaft is centered inside bored rotor hub",
    )
    ctx.expect_contact(
        rotor,
        body,
        elem_a="rotor_hub_ring",
        elem_b="front_collar",
        contact_tol=0.001,
        name="front retainer captures rotor hub",
    )
    ctx.expect_contact(
        rotor,
        body,
        elem_a="rotor_hub_ring",
        elem_b="rear_collar",
        contact_tol=0.001,
        name="rear retainer captures rotor hub",
    )

    rest_position = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_position = ctx.part_world_position(rotor)
        ctx.expect_contact(
            rotor,
            body,
            elem_a="rotor_hub_ring",
            elem_b="front_collar",
            contact_tol=0.001,
            name="rotor remains captured after spin pose",
        )
    ctx.check(
        "spin changes orientation without translating rotor",
        rest_position is not None
        and spun_position is not None
        and all(abs(rest_position[i] - spun_position[i]) < 1e-9 for i in range(3)),
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
