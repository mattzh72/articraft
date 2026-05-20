from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


TOWER_HEIGHT = 12.0
YAW_BEARING_TOP = 12.20
ROTOR_JOINT_X = 1.02
ROTOR_JOINT_Z = 0.34


def _tower_taper_geometry() -> MeshGeometry:
    """Long, slightly tapered tubular tower with capped proxy ends."""
    return LatheGeometry(
        [
            (0.0, 0.18),
            (0.42, 0.18),
            (0.34, 0.32),
            (0.22, 5.5),
            (0.13, TOWER_HEIGHT),
            (0.0, TOWER_HEIGHT),
        ],
        segments=64,
    )


def _nacelle_shell_geometry() -> MeshGeometry:
    """A compact rounded nacelle, longest along the main shaft axis."""
    shell = superellipse_side_loft(
        [
            (-0.62, -0.10, 0.11, 0.26),
            (-0.38, -0.20, 0.21, 0.42),
            (0.28, -0.22, 0.23, 0.48),
            (0.62, -0.11, 0.13, 0.28),
        ],
        exponents=2.7,
        segments=56,
        cap=True,
    )
    # The side-loft axis is local +Y; rotate it so the turbine nose points +X.
    return shell.rotate_z(-math.pi / 2.0)


def _spinner_geometry() -> MeshGeometry:
    spinner = LatheGeometry(
        [
            (0.0, 0.30),
            (0.18, 0.30),
            (0.17, 0.38),
            (0.10, 0.52),
            (0.0, 0.62),
        ],
        segments=56,
    )
    return spinner.rotate_y(math.pi / 2.0)


def _blade_geometry(angle: float) -> MeshGeometry:
    """One slender twisted turbine blade rooted inside the hub."""
    # Coefficients around a soft airfoil-like loop.  The section radial axis is
    # +Z before the whole blade is rotated around the main shaft (+X).
    loop = [
        (0.00, -0.50),
        (0.34, -0.42),
        (0.50, -0.08),
        (0.32, 0.33),
        (0.05, 0.50),
        (-0.14, 0.49),
        (-0.36, 0.10),
        (-0.23, -0.38),
    ]
    stations = [
        (0.10, 0.36, 0.070, 18.0),
        (0.34, 0.33, 0.058, 15.0),
        (0.85, 0.25, 0.044, 10.0),
        (1.45, 0.16, 0.030, 6.5),
        (1.92, 0.08, 0.018, 3.0),
    ]

    profiles: list[list[tuple[float, float, float]]] = []
    root_r = stations[0][0]
    tip_r = stations[-1][0]
    for radius, chord, thickness, pitch_deg in stations:
        span_t = (radius - root_r) / (tip_r - root_r)
        sweep_y = -0.055 * (span_t**1.2)
        pitch = math.radians(pitch_deg)
        c = math.cos(pitch)
        s = math.sin(pitch)
        section = []
        for x_factor, y_factor in loop:
            x = x_factor * thickness
            y = y_factor * chord + sweep_y
            # Pitch the airfoil in the streamwise/tangential plane, then move it
            # forward into the rotating hub plane.
            pitched_x = x * c - y * s + 0.20
            pitched_y = x * s + y * c
            section.append((pitched_x, pitched_y, radius))
        profiles.append(section)

    return LoftGeometry(profiles, cap=True, closed=True).rotate((1.0, 0.0, 0.0), angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_white = model.material("painted_tower_white", color=(0.88, 0.90, 0.88, 1.0))
    nacelle_white = model.material("smooth_nacelle_white", color=(0.95, 0.96, 0.93, 1.0))
    blade_white = model.material("matte_blade_white", color=(0.97, 0.97, 0.94, 1.0))
    hub_gray = model.material("cast_hub_gray", color=(0.70, 0.72, 0.70, 1.0))
    dark_metal = model.material("dark_bearing_metal", color=(0.18, 0.19, 0.18, 1.0))
    concrete = model.material("pale_concrete", color=(0.62, 0.62, 0.58, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.62, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        mesh_from_geometry(_tower_taper_geometry(), "tapered_tower"),
        material=tower_white,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(radius=0.24, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT + 0.10)),
        material=dark_metal,
        name="yaw_pedestal",
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_nacelle_shell_geometry(), "nacelle_shell"),
        origin=Origin(xyz=(0.25, 0.0, 0.34)),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_metal,
        name="yaw_boss",
    )
    nacelle.visual(
        Cylinder(radius=0.13, length=0.18),
        origin=Origin(xyz=(0.93, 0.0, ROTOR_JOINT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bearing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.34),
        origin=Origin(xyz=(0.21, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="hub_barrel",
    )
    rotor.visual(
        mesh_from_geometry(_spinner_geometry(), "spinner"),
        material=hub_gray,
        name="spinner",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            mesh_from_geometry(_blade_geometry(angle), f"blade_{index}"),
            material=blade_white,
            name=f"blade_{index}",
        )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, YAW_BEARING_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.20, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=3.0, friction=0.4),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(ROTOR_JOINT_X, 0.0, ROTOR_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="yaw_boss",
        negative_elem="yaw_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="nacelle yaw boss sits on tower bearing",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="shaft",
        negative_elem="front_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotor shaft seats at nacelle nose bearing",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "nacelle yaws around vertical tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and abs(rest_rotor_pos[0] - ROTOR_JOINT_X) < 0.01
        and abs(yawed_rotor_pos[0]) < 0.02
        and yawed_rotor_pos[1] > ROTOR_JOINT_X - 0.02,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    with ctx.pose({spin: 0.0}):
        blade_upright = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        blade_sideways = ctx.part_element_world_aabb(rotor, elem="blade_0")
    ctx.check(
        "rotor spins continuously about main shaft axis",
        blade_upright is not None
        and blade_sideways is not None
        and blade_upright[1][2] > blade_sideways[1][2] + 0.40,
        details=f"upright={blade_upright}, sideways={blade_sideways}",
    )

    return ctx.report()


object_model = build_object_model()
