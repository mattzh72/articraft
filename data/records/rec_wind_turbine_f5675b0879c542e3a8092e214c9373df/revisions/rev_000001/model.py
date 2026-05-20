from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _ellipse_section(
    x: float,
    width: float,
    height: float,
    *,
    z_center: float,
    count: int = 28,
) -> list[tuple[float, float, float]]:
    section: list[tuple[float, float, float]] = []
    for index in range(count):
        theta = math.tau * index / count
        section.append(
            (
                x,
                0.5 * width * math.cos(theta),
                z_center + 0.5 * height * math.sin(theta),
            )
        )
    return section


def _airfoil_section(
    radius_z: float,
    chord: float,
    thickness: float,
    *,
    pitch: float,
    x_center: float,
) -> list[tuple[float, float, float]]:
    profile = [
        (-0.48, 0.00),
        (-0.22, 0.56),
        (0.08, 0.78),
        (0.42, 0.42),
        (0.52, 0.08),
        (0.34, -0.18),
        (-0.08, -0.36),
        (-0.42, -0.14),
    ]
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    loop: list[tuple[float, float, float]] = []
    for px, py in profile:
        local_x = chord * px
        local_y = thickness * py
        loop.append(
            (
                x_center + local_x * cos_pitch - local_y * sin_pitch,
                local_x * sin_pitch + local_y * cos_pitch,
                radius_z,
            )
        )
    return loop


def _build_tower_shell_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (2.45, 0.0),
            (2.42, 8.0),
            (2.28, 22.0),
            (2.02, 42.0),
            (1.76, 62.0),
            (1.56, 78.0),
            (1.48, 82.0),
            (0.0, 82.0),
        ],
        segments=72,
    )


def _build_nacelle_shell_mesh():
    sections = [
        _ellipse_section(-1.4, 2.0, 1.5, z_center=2.0),
        _ellipse_section(-0.2, 3.2, 2.5, z_center=2.15),
        _ellipse_section(1.8, 4.0, 3.5, z_center=2.35),
        _ellipse_section(4.2, 4.2, 3.9, z_center=2.45),
        _ellipse_section(5.8, 3.4, 3.2, z_center=2.35),
        _ellipse_section(6.7, 2.0, 2.1, z_center=2.2),
        _ellipse_section(7.25, 0.35, 0.75, z_center=2.05),
    ]
    return repair_loft(section_loft(sections))


def _build_spinner_mesh():
    return (
        LatheGeometry(
            [
                (0.20, 0.00),
                (0.98, 0.14),
                (1.15, 0.92),
                (0.88, 2.05),
                (0.34, 3.05),
                (0.0, 3.55),
            ],
            segments=64,
        )
        .rotate_y(math.pi / 2.0)
    )


def _build_blade_mesh():
    blade = repair_loft(
        section_loft(
            [
                _airfoil_section(0.70, 2.0, 0.92, pitch=math.radians(18.0), x_center=1.35),
                _airfoil_section(4.5, 4.3, 0.80, pitch=math.radians(16.0), x_center=2.10),
                _airfoil_section(15.0, 3.9, 0.56, pitch=math.radians(11.0), x_center=2.00),
                _airfoil_section(28.0, 3.0, 0.36, pitch=math.radians(7.0), x_center=1.65),
                _airfoil_section(40.0, 1.85, 0.20, pitch=math.radians(4.5), x_center=1.10),
                _airfoil_section(47.5, 0.72, 0.09, pitch=math.radians(2.0), x_center=0.70),
            ]
        )
    )
    blade.merge(
        CylinderGeometry(radius=0.72, height=1.9, radial_segments=30)
        .rotate_y(math.pi / 2.0)
        .translate(0.95, 0.0, 0.95)
    )
    blade.merge(
        CylinderGeometry(radius=0.86, height=0.42, radial_segments=30)
        .rotate_y(math.pi / 2.0)
        .translate(0.58, 0.0, 0.95)
    )
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.93, 0.94, 0.95, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.90, 0.91, 0.93, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    blade_paint = model.material("blade_paint", rgba=(0.96, 0.96, 0.97, 1.0))
    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.67, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=6.0, length=2.0),
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        _save_mesh(_build_tower_shell_mesh(), "tower_shell"),
        material=tower_paint,
        name="tower_shaft",
    )
    tower.visual(
        Cylinder(radius=1.7, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 82.5)),
        material=dark_metal,
        name="yaw_bearing",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.2, length=83.0),
        mass=90000.0,
        origin=Origin(xyz=(0.0, 0.0, 41.5)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.8, length=0.8),
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        material=dark_metal,
        name="turntable",
    )
    nacelle.visual(
        _save_mesh(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=nacelle_paint,
        name="nacelle_body",
    )
    nacelle.visual(
        Box((8.2, 2.8, 0.6)),
        origin=Origin(xyz=(3.0, 0.0, 0.95)),
        material=dark_metal,
        name="bedplate",
    )
    nacelle.visual(
        Box((1.6, 1.5, 2.2)),
        origin=Origin(xyz=(6.15, 0.0, 1.95)),
        material=dark_metal,
        name="front_frame",
    )
    nacelle.visual(
        Cylinder(radius=0.55, length=1.1),
        origin=Origin(xyz=(6.95, 0.0, 2.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="main_shaft_sleeve",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((8.2, 4.0, 4.2)),
        mass=56000.0,
        origin=Origin(xyz=(2.6, 0.0, 2.2)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=1.35, length=2.6),
        origin=Origin(xyz=(1.3, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=1.55, length=0.54),
        origin=Origin(xyz=(0.27, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_flange",
    )
    rotor.visual(
        _save_mesh(_build_spinner_mesh(), "spinner_shell"),
        origin=Origin(),
        material=dark_metal,
        name="spinner",
    )
    base_blade = _build_blade_mesh()
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            _save_mesh(base_blade.copy().rotate_x(-angle), f"blade_shell_{index}"),
            material=blade_paint,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Box((4.2, 100.0, 100.0)),
        mass=26000.0,
        origin=Origin(xyz=(2.0, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 83.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.25),
    )
    model.articulation(
        "nacelle_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(7.5, 0.0, 2.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200000.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_rotor_spin")

    ctx.expect_overlap(
        nacelle,
        tower,
        axes="xy",
        elem_a="turntable",
        elem_b="yaw_bearing",
        min_overlap=2.8,
        name="nacelle turntable stays centered on tower yaw bearing",
    )
    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="turntable",
        negative_elem="yaw_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable seats on the tower top",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="main_shaft_sleeve",
        min_gap=0.0,
        max_gap=0.05,
        name="rotor hub stays compact and close to the nacelle nose",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw rotates nacelle about the tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and yawed_rotor_pos[1] > 6.0
        and abs(yawed_rotor_pos[0]) < 2.0,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    blade0_rest = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        blade0_spun = ctx.part_element_world_aabb(rotor, elem="blade_0")
    rest_blade0_center = None
    spun_blade0_center = None
    if blade0_rest is not None:
        rest_blade0_center = tuple(
            0.5 * (blade0_rest[0][axis] + blade0_rest[1][axis]) for axis in range(3)
        )
    if blade0_spun is not None:
        spun_blade0_center = tuple(
            0.5 * (blade0_spun[0][axis] + blade0_spun[1][axis]) for axis in range(3)
        )
    ctx.check(
        "rotor spin moves the blade around the main shaft axis",
        rest_blade0_center is not None
        and spun_blade0_center is not None
        and rest_blade0_center[2] > 100.0
        and spun_blade0_center[1] < -20.0
        and spun_blade0_center[2] < 90.0,
        details=f"rest={rest_blade0_center}, spun={spun_blade0_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
