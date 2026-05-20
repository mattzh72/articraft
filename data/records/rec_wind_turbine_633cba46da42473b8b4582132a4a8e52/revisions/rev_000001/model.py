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
    rounded_rect_profile,
    section_loft,
)


def _rounded_section(
    x_pos: float,
    width: float,
    height: float,
    z_center: float,
    *,
    corner_segments: int = 8,
) -> tuple[tuple[float, float, float], ...]:
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.22,
        corner_segments=corner_segments,
    )
    return tuple((x_pos, y_pos, z_center + z_pos) for y_pos, z_pos in profile)


def _build_nacelle_shell_mesh():
    sections = [
        _rounded_section(-0.55, 0.74, 0.82, 0.68),
        _rounded_section(0.05, 1.06, 1.18, 0.80),
        _rounded_section(0.92, 1.18, 1.24, 0.82),
        _rounded_section(1.62, 0.94, 1.00, 0.80),
        _rounded_section(1.92, 0.44, 0.50, 0.79),
    ]
    return repair_loft(section_loft(sections))


def _blade_loop(
    span: float,
    chord: float,
    thickness: float,
    pitch_deg: float,
    *,
    sweep_x: float = 0.0,
    tangential_shift: float = 0.0,
) -> tuple[tuple[float, float, float], ...]:
    half_thickness = thickness * 0.5
    raw_points = [
        (0.02 * half_thickness, -0.50 * chord),
        (0.68 * half_thickness, -0.16 * chord),
        (0.82 * half_thickness, 0.16 * chord),
        (0.34 * half_thickness, 0.50 * chord),
        (-0.10 * half_thickness, 0.52 * chord),
        (-0.78 * half_thickness, 0.22 * chord),
        (-0.72 * half_thickness, -0.18 * chord),
        (-0.18 * half_thickness, -0.46 * chord),
    ]
    angle = math.radians(pitch_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    section: list[tuple[float, float, float]] = []
    for x_pos, y_pos in raw_points:
        rot_x = (cos_a * x_pos) - (sin_a * y_pos)
        rot_y = (sin_a * x_pos) + (cos_a * y_pos)
        section.append((sweep_x + rot_x, tangential_shift + rot_y, span))
    return tuple(section)


def _build_blade_mesh():
    sections = [
        _blade_loop(0.00, 0.34, 0.10, 26.0, sweep_x=0.00, tangential_shift=-0.006),
        _blade_loop(0.38, 0.31, 0.086, 20.0, sweep_x=0.01, tangential_shift=-0.002),
        _blade_loop(1.18, 0.24, 0.056, 13.0, sweep_x=0.04, tangential_shift=0.015),
        _blade_loop(2.05, 0.16, 0.032, 8.0, sweep_x=0.08, tangential_shift=0.030),
        _blade_loop(2.72, 0.09, 0.018, 5.0, sweep_x=0.10, tangential_shift=0.038),
    ]
    return repair_loft(section_loft(sections))


def _build_spinner_mesh():
    return LatheGeometry(
        [
            (0.0, 0.00),
            (0.04, 0.03),
            (0.14, 0.16),
            (0.24, 0.33),
            (0.28, 0.48),
            (0.22, 0.56),
            (0.0, 0.58),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_wind_turbine")

    concrete = model.material("concrete", rgba=(0.62, 0.62, 0.60, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.66, 0.70, 0.74, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.84, 0.86, 0.88, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    hub_casting = model.material("hub_casting", rgba=(0.46, 0.49, 0.53, 1.0))
    stainless_hardware = model.material("stainless_hardware", rgba=(0.76, 0.79, 0.82, 1.0))
    seal_black = model.material("seal_black", rgba=(0.12, 0.12, 0.13, 1.0))

    foundation = model.part("foundation")
    foundation.visual(
        Box((1.80, 1.80, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=concrete,
        name="pad",
    )
    foundation.visual(
        Box((0.88, 0.88, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=concrete,
        name="pier_cap",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((0.24, 0.24), (-0.24, 0.24), (-0.24, -0.24), (0.24, -0.24)),
        start=1,
    ):
        foundation.visual(
            Cylinder(radius=0.035, length=0.08),
            origin=Origin(xyz=(x_pos, y_pos, 0.80)),
            material=stainless_hardware,
            name=f"anchor_bolt_{index}",
        )
    foundation.inertial = Inertial.from_geometry(
        Box((1.80, 1.80, 0.76)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.29, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=galvanized_steel,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=0.16, length=10.80),
        origin=Origin(xyz=(0.0, 0.0, 5.40)),
        material=galvanized_steel,
        name="tower_shaft",
    )
    tower.visual(
        Cylinder(radius=0.20, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 10.89)),
        material=hub_casting,
        name="top_collar",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=10.98),
        mass=1150.0,
        origin=Origin(xyz=(0.0, 0.0, 5.49)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.23, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=seal_black,
        name="yaw_sleeve",
    )
    nacelle.visual(
        Cylinder(radius=0.33, length=0.10),
        origin=Origin(xyz=(0.08, 0.0, 0.23)),
        material=hub_casting,
        name="yaw_skirt",
    )
    nacelle.visual(
        Box((0.92, 0.54, 0.14)),
        origin=Origin(xyz=(0.40, 0.0, 0.25)),
        material=hub_casting,
        name="underbody",
    )
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=nacelle_paint,
        name="shell",
    )
    nacelle.visual(
        Cylinder(radius=0.31, length=0.10),
        origin=Origin(xyz=(1.97, 0.0, 0.79), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_casting,
        name="bearing_hood",
    )
    nacelle.visual(
        Cylinder(radius=0.23, length=0.46),
        origin=Origin(xyz=(2.15, 0.0, 0.79), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="bearing_housing",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((2.55, 1.28, 1.28)),
        mass=860.0,
        origin=Origin(xyz=(0.85, 0.0, 0.79)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.46, length=0.10),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_casting,
        name="hub_plate",
    )
    rotor.visual(
        Cylinder(radius=0.22, length=0.16),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_black,
        name="rear_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.30, length=0.52),
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_casting,
        name="hub_drum",
    )
    rotor.visual(
        mesh_from_geometry(_build_spinner_mesh(), "spinner"),
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        material=nacelle_paint,
        name="spinner",
    )

    blade_mesh = _build_blade_mesh()
    for blade_index in range(3):
        angle = blade_index * (2.0 * math.pi / 3.0)
        sin_a = math.sin(angle)
        cos_a = math.cos(angle)
        rotor.visual(
            Cylinder(radius=0.17, length=0.10),
            origin=Origin(
                xyz=(0.05, -sin_a * 0.33, cos_a * 0.33),
                rpy=(angle, 0.0, 0.0),
            ),
            material=stainless_hardware,
            name=f"root_flange_{blade_index + 1}",
        )
        rotor.visual(
            Cylinder(radius=0.14, length=0.40),
            origin=Origin(
                xyz=(0.02, -sin_a * 0.58, cos_a * 0.58),
                rpy=(angle, 0.0, 0.0),
            ),
            material=hub_casting,
            name=f"root_cuff_{blade_index + 1}",
        )
        rotor.visual(
            Box((0.18, 0.16, 0.56)),
            origin=Origin(
                xyz=(0.08, -sin_a * 0.56, cos_a * 0.56),
                rpy=(angle, 0.0, 0.0),
            ),
            material=hub_casting,
            name=f"load_path_{blade_index + 1}",
        )
        rotor.visual(
            mesh_from_geometry(blade_mesh, f"blade_{blade_index + 1}"),
            origin=Origin(
                xyz=(0.02, -sin_a * 0.78, cos_a * 0.78),
                rpy=(angle, 0.0, 0.0),
            ),
            material=blade_finish,
            name=f"blade_{blade_index + 1}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=3.05, length=0.82),
        mass=280.0,
        origin=Origin(xyz=(1.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "foundation_to_tower",
        ArticulationType.FIXED,
        parent=foundation,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )
    model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 10.98)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.45),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(2.38, 0.0, 0.79)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=2.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("foundation")
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    rotor_spin = object_model.get_articulation("nacelle_to_rotor")

    pier_cap = foundation.get_visual("pier_cap")
    base_flange = tower.get_visual("base_flange")
    top_collar = tower.get_visual("top_collar")
    yaw_sleeve = nacelle.get_visual("yaw_sleeve")
    bearing_housing = nacelle.get_visual("bearing_housing")
    rear_sleeve = rotor.get_visual("rear_sleeve")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(tower, foundation, elem_a=base_flange, elem_b=pier_cap)
    ctx.expect_contact(nacelle, tower, elem_a=yaw_sleeve, elem_b=top_collar)
    ctx.expect_within(tower, nacelle, inner_elem=top_collar, outer_elem=yaw_sleeve, axes="xy")
    ctx.expect_contact(rotor, nacelle, elem_a=rear_sleeve, elem_b=bearing_housing)
    ctx.expect_within(rotor, nacelle, inner_elem=rear_sleeve, outer_elem=bearing_housing, axes="yz")
    ctx.expect_gap(rotor, tower, axis="x", min_gap=1.8)

    with ctx.pose({yaw: 0.65, rotor_spin: 1.10}):
        ctx.expect_contact(nacelle, tower, elem_a=yaw_sleeve, elem_b=top_collar)
        ctx.expect_contact(rotor, nacelle, elem_a=rear_sleeve, elem_b=bearing_housing)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
