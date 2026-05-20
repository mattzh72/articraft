from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


TOWER_HEIGHT = 8.0
HUB_X = 0.98
HUB_Z = 0.46
ROTOR_RADIUS = 2.35


def _circle_section(radius: float, z: float, segments: int = 56) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(math.tau * i / segments),
            radius * math.sin(math.tau * i / segments),
            z,
        )
        for i in range(segments)
    ]


def _superellipse_section(
    x: float,
    half_y: float,
    half_z: float,
    z_center: float,
    *,
    segments: int = 48,
    exponent: float = 2.7,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(segments):
        angle = math.tau * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        y = half_y * math.copysign(abs(c) ** (2.0 / exponent), c)
        z = z_center + half_z * math.copysign(abs(s) ** (2.0 / exponent), s)
        points.append((x, y, z))
    return points


def _build_tower_mesh():
    """One continuous tapered center support with a broad service footing."""
    return LoftGeometry(
        [
            _circle_section(1.05, 0.00),
            _circle_section(1.05, 0.26),
            _circle_section(0.48, 0.38),
            _circle_section(0.30, 3.60),
            _circle_section(0.22, TOWER_HEIGHT - 0.26),
            _circle_section(0.34, TOWER_HEIGHT),
        ],
        cap=True,
    )


def _build_nacelle_body_mesh():
    """Rounded horizontal nacelle body, with the frame at the yaw bearing plane."""
    return repair_loft(section_loft(
        [
            _superellipse_section(-0.88, 0.16, 0.20, 0.42),
            _superellipse_section(-0.62, 0.31, 0.31, 0.44),
            _superellipse_section(-0.18, 0.39, 0.37, 0.46),
            _superellipse_section(0.34, 0.38, 0.36, 0.46),
            _superellipse_section(0.72, 0.28, 0.29, 0.46),
            _superellipse_section(0.86, 0.18, 0.20, 0.46),
        ],
    ))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_paint = model.material("warm_white_tower", rgba=(0.86, 0.86, 0.82, 1.0))
    nacelle_paint = model.material("nacelle_white", rgba=(0.92, 0.93, 0.90, 1.0))
    blade_paint = model.material("blade_matte_white", rgba=(0.96, 0.96, 0.92, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_build_tower_mesh(), "tapered_tower"),
        material=tower_paint,
        name="tapered_tower",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.45, length=TOWER_HEIGHT),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT / 2.0)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_body_mesh(), "nacelle_shell"),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.34, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=bearing_dark,
        name="yaw_bearing",
    )
    nacelle.visual(
        Cylinder(radius=0.17, length=0.257),
        # URDF cylinders are local-Z; rotate so this bearing sleeve follows the
        # horizontal main shaft and reaches toward the rotating hub.
        origin=Origin(xyz=(0.775, 0.0, HUB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_dark,
        name="shaft_sleeve",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((1.8, 0.8, 0.8)),
        mass=1100.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.32,
                3,
                thickness=0.18,
                blade_pitch_deg=10.0,
                blade_sweep_deg=6.0,
                blade_root_chord=0.52,
                blade_tip_chord=0.12,
                blade=FanRotorBlade(
                    shape="narrow",
                    tip_pitch_deg=4.0,
                    camber=0.04,
                    tip_clearance=0.0,
                ),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.07, rear_collar_radius=0.28),
            ),
            "three_blade_rotor",
        ),
        # FanRotorGeometry spins about local +Z.  Rotate it so the turbine
        # rotor disk lies in the local YZ plane and spins about +X.
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_paint,
        name="hub_blades",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_RADIUS, length=0.20),
        mass=420.0,
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.18, lower=-math.pi, upper=math.pi),
    )

    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.check("tower present", tower is not None, "Expected a single central tower support.")
    ctx.check("nacelle present", nacelle is not None, "Expected a yawing nacelle.")
    ctx.check("rotor present", rotor is not None, "Expected a front rotor with blades.")
    ctx.check("yaw joint present", yaw is not None, "Expected tower-to-nacelle yaw articulation.")
    ctx.check("spin joint present", spin is not None, "Expected continuous rotor articulation.")

    if tower is None or nacelle is None or rotor is None or yaw is None or spin is None:
        return ctx.report()

    ctx.check(
        "rotor spins continuously",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type!r}, axis={spin.axis!r}",
    )
    ctx.check(
        "nacelle yaws vertically",
        yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type!r}, axis={yaw.axis!r}",
    )

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="yaw_bearing",
        negative_elem="tapered_tower",
        name="nacelle bearing sits on tower",
    )
    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        min_overlap=0.18,
        elem_a="yaw_bearing",
        elem_b="tapered_tower",
        name="central support under yaw bearing",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw carries rotor around tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and yawed_rotor_pos[1] > rest_rotor_pos[1] + 0.75
        and abs(yawed_rotor_pos[0]) < 0.10,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
