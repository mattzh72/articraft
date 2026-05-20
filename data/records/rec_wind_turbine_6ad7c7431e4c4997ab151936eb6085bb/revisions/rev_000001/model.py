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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x - s * y, s * x + c * y)


def _rot_x_point(
    x: float, y: float, z: float, angle: float
) -> tuple[float, float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, (y * c) - (z * s), (y * s) + (z * c))


def _build_tower_shell_mesh(
    *, height: float, base_radius: float, top_radius: float, wall_thickness: float
):
    outer = [
        (base_radius + 0.10, 0.00),
        (base_radius, 0.65),
        (base_radius * 0.88, 6.0),
        (0.5 * (base_radius + top_radius), height * 0.55),
        (top_radius, height),
    ]
    inner = [
        (base_radius - wall_thickness, 0.65),
        ((base_radius * 0.88) - wall_thickness, 6.0),
        ((0.5 * (base_radius + top_radius)) - wall_thickness, height * 0.55),
        (top_radius - wall_thickness, height),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _blade_section(
    *,
    z: float,
    chord: float,
    thickness: float,
    twist: float,
    sweep_x: float,
) -> list[tuple[float, float, float]]:
    raw = [
        (0.48 * chord + sweep_x, 0.00),
        (0.20 * chord + sweep_x, 0.44 * thickness),
        (-0.06 * chord + sweep_x, 0.54 * thickness),
        (-0.38 * chord + sweep_x, 0.18 * thickness),
        (-0.50 * chord + sweep_x, 0.00),
        (-0.34 * chord + sweep_x, -0.22 * thickness),
        (0.06 * chord + sweep_x, -0.42 * thickness),
        (0.34 * chord + sweep_x, -0.17 * thickness),
    ]
    points: list[tuple[float, float, float]] = []
    for x, y in raw:
        xr, yr = _rotate_xy(x, y, twist)
        points.append((xr, yr, z))
    return points


def _build_blade_skin_mesh():
    sections = [
        _blade_section(z=0.32, chord=1.42, thickness=0.30, twist=0.22, sweep_x=0.05),
        _blade_section(z=1.50, chord=1.22, thickness=0.25, twist=0.16, sweep_x=0.11),
        _blade_section(z=3.80, chord=0.96, thickness=0.18, twist=0.08, sweep_x=0.20),
        _blade_section(z=6.20, chord=0.70, thickness=0.13, twist=0.00, sweep_x=0.31),
        _blade_section(z=8.30, chord=0.48, thickness=0.08, twist=-0.08, sweep_x=0.43),
        _blade_section(z=9.55, chord=0.30, thickness=0.05, twist=-0.15, sweep_x=0.52),
    ]
    return repair_loft(section_loft(sections))


def _add_base_bolt_circle(part, *, radius: float, z: float, material) -> None:
    for index in range(16):
        angle = index * math.tau / 16.0
        part.visual(
            Cylinder(radius=0.045, length=0.10),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), z + 0.05)
            ),
            material=material,
        )


def _add_service_panel(
    part,
    *,
    x: float,
    y: float,
    z: float,
    material,
    fastener_material,
) -> None:
    part.visual(
        Box((1.15, 0.04, 0.78)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
    )
    for x_offset in (-0.45, 0.0, 0.45):
        for z_offset in (-0.24, 0.24):
            part.visual(
                Cylinder(radius=0.018, length=0.08),
                origin=Origin(
                    xyz=(x + x_offset, y + (0.04 if y > 0.0 else -0.04), z + z_offset),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=fastener_material,
            )


def _add_blade_root_bolts(part, *, radius: float, material) -> None:
    for index in range(10):
        angle = index * math.tau / 10.0
        part.visual(
            Cylinder(radius=0.022, length=0.06),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.14)
            ),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wind_turbine")

    tower_paint = model.material("tower_paint", rgba=(0.83, 0.85, 0.86, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.84, 0.86, 0.88, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.43, 0.45, 0.47, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.78, 0.12, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.79, 0.14, 0.11, 1.0))
    composite_white = model.material("composite_white", rgba=(0.90, 0.91, 0.92, 1.0))

    tower = model.part("tower")
    tower.visual(
        _save_mesh(
            "tower_shell",
            _build_tower_shell_mesh(
                height=28.0,
                base_radius=1.80,
                top_radius=1.12,
                wall_thickness=0.08,
            ),
        ),
        material=tower_paint,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.05, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_steel,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=1.95, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=weathered_steel,
    )
    _add_base_bolt_circle(tower, radius=1.78, z=0.22, material=weathered_steel)
    tower.visual(
        Cylinder(radius=1.28, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 28.10)),
        material=dark_steel,
        name="top_bearing_ring",
    )
    tower.visual(
        Cylinder(radius=1.18, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 27.87)),
        material=weathered_steel,
    )
    tower.inertial = Inertial.from_geometry(
        Box((4.2, 4.2, 28.5)),
        mass=58000.0,
        origin=Origin(xyz=(0.0, 0.0, 14.10)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=1.28, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_steel,
        name="yaw_skirt",
    )
    nacelle.visual(
        Box((4.80, 3.10, 0.28)),
        origin=Origin(xyz=(0.18, 0.0, 0.34)),
        material=weathered_steel,
        name="bedplate",
    )
    nacelle.visual(
        Box((4.55, 0.08, 2.20)),
        origin=Origin(xyz=(0.10, 1.47, 1.58)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((4.55, 0.08, 2.20)),
        origin=Origin(xyz=(0.10, -1.47, 1.58)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((3.30, 3.02, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, 2.73)),
        material=nacelle_paint,
        name="roof_plate",
    )
    nacelle.visual(
        Box((0.10, 3.02, 2.20)),
        origin=Origin(xyz=(-2.12, 0.0, 1.58)),
        material=nacelle_paint,
    )
    nacelle.visual(
        Box((0.20, 2.42, 2.00)),
        origin=Origin(xyz=(1.70, 0.0, 1.48)),
        material=nacelle_paint,
        name="front_bulkhead",
    )
    nacelle.visual(
        Cylinder(radius=0.60, length=0.30),
        origin=Origin(xyz=(1.95, 0.0, 1.88), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_housing",
    )
    nacelle.visual(
        Box((1.35, 0.22, 1.12)),
        origin=Origin(xyz=(1.10, 1.03, 0.96)),
        material=weathered_steel,
    )
    nacelle.visual(
        Box((1.35, 0.22, 1.12)),
        origin=Origin(xyz=(1.10, -1.03, 0.96)),
        material=weathered_steel,
    )
    nacelle.visual(
        Box((1.72, 1.55, 1.22)),
        origin=Origin(xyz=(-0.45, 0.0, 1.09)),
        material=dark_steel,
    )
    nacelle.visual(
        Box((2.60, 2.60, 0.08)),
        origin=Origin(xyz=(-0.22, 0.0, 2.82)),
        material=weathered_steel,
        name="service_walkway",
    )
    _add_service_panel(
        nacelle,
        x=-0.35,
        y=1.51,
        z=1.44,
        material=weathered_steel,
        fastener_material=dark_steel,
    )
    _add_service_panel(
        nacelle,
        x=-0.35,
        y=-1.51,
        z=1.44,
        material=weathered_steel,
        fastener_material=dark_steel,
    )

    rail_base_z = 2.86
    rail_mid_z = 3.34
    rail_top_z = 3.82
    rail_points = [
        (-1.32, -1.22),
        (-1.32, 0.0),
        (-1.32, 1.22),
        (-0.35, 1.22),
        (0.70, 1.22),
        (0.70, -1.22),
        (-0.35, -1.22),
    ]
    for x, y in rail_points:
        _add_member(
            nacelle,
            (x, y, rail_base_z),
            (x, y, rail_top_z),
            0.026,
            safety_yellow,
        )
    for (ax, ay), (bx, by) in zip(rail_points[:-1], rail_points[1:]):
        _add_member(
            nacelle,
            (ax, ay, rail_top_z),
            (bx, by, rail_top_z),
            0.022,
            safety_yellow,
        )
        _add_member(
            nacelle,
            (ax, ay, rail_mid_z),
            (bx, by, rail_mid_z),
            0.020,
            safety_yellow,
        )
    nacelle.visual(
        Box((0.56, 0.18, 0.20)),
        origin=Origin(xyz=(-1.62, 0.0, 0.30)),
        material=safety_yellow,
    )
    nacelle.visual(
        Box((0.44, 0.18, 0.54)),
        origin=Origin(xyz=(1.53, 1.19, 1.58)),
        material=weathered_steel,
    )
    nacelle.visual(
        Box((0.26, 0.22, 0.34)),
        origin=Origin(xyz=(1.95, 1.57, 1.62)),
        material=lockout_red,
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((5.2, 3.6, 3.6)),
        mass=12000.0,
        origin=Origin(xyz=(0.12, 0.0, 1.45)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.58, length=0.18),
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_flange",
    )
    hub.visual(
        Cylinder(radius=0.82, length=0.10),
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lockout_red,
        name="lock_disc",
    )
    hub.visual(
        Cylinder(radius=0.96, length=1.28),
        origin=Origin(xyz=(0.74, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_steel,
        name="main_body",
    )
    hub.visual(
        Cylinder(radius=0.72, length=0.78),
        origin=Origin(xyz=(1.69, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_steel,
    )
    hub.visual(
        Cylinder(radius=0.42, length=0.28),
        origin=Origin(xyz=(2.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_steel,
    )

    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    seat_outer_radius = 1.42
    seat_length = 0.76
    seat_x = 1.05
    for index, phi in enumerate(blade_angles, start=1):
        y_center = -(seat_outer_radius - 0.5 * seat_length) * math.sin(phi)
        z_center = (seat_outer_radius - 0.5 * seat_length) * math.cos(phi)
        hub.visual(
            Cylinder(radius=0.36, length=seat_length),
            origin=Origin(xyz=(seat_x, y_center, z_center), rpy=(phi, 0.0, 0.0)),
            material=dark_steel,
            name=f"blade_seat_{index}",
        )
        web_center = _rot_x_point(0.66, 0.0, 0.34, phi)
        hub.visual(
            Box((0.88, 0.18, 0.56)),
            origin=Origin(xyz=web_center, rpy=(phi, 0.0, 0.0)),
            material=weathered_steel,
        )
        front_web_center = _rot_x_point(0.98, 0.0, 0.16, phi)
        hub.visual(
            Box((0.36, 0.12, 0.32)),
            origin=Origin(xyz=front_web_center, rpy=(phi, 0.0, 0.0)),
            material=weathered_steel,
        )
        stop_center = _rot_x_point(1.18, 0.0, 0.42, phi)
        hub.visual(
            Box((0.18, 0.10, 0.34)),
            origin=Origin(xyz=stop_center, rpy=(phi, 0.0, 0.0)),
            material=safety_yellow,
        )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=1.05, length=2.50),
        mass=4500.0,
        origin=Origin(xyz=(1.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    blade_skin = _save_mesh("blade_skin", _build_blade_skin_mesh())
    blade_names = ("blade_top", "blade_lower_left", "blade_lower_right")
    for blade_name in blade_names:
        blade = model.part(blade_name)
        blade.visual(
            Cylinder(radius=0.42, length=0.22),
            origin=Origin(xyz=(0.0, 0.0, 0.11)),
            material=dark_steel,
            name="root_flange",
        )
        blade.visual(
            Cylinder(radius=0.34, length=0.56),
            origin=Origin(xyz=(0.0, 0.0, 0.50)),
            material=weathered_steel,
            name="root_laminate",
        )
        blade.visual(
            blade_skin,
            material=composite_white,
            name="blade_skin",
        )
        blade.visual(
            Box((0.16, 0.10, 0.20)),
            origin=Origin(xyz=(-0.32, 0.24, 0.16)),
            material=safety_yellow,
        )
        _add_blade_root_bolts(blade, radius=0.30, material=weathered_steel)
        blade.inertial = Inertial.from_geometry(
            Box((1.7, 0.6, 9.9)),
            mass=950.0,
            origin=Origin(xyz=(0.18, 0.0, 5.05)),
        )

    model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 28.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250000.0,
            velocity=0.18,
            lower=-1.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "nacelle_to_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(2.10, 0.0, 1.88)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400000.0, velocity=1.20),
    )

    for index, (blade_name, phi) in enumerate(zip(blade_names, blade_angles), start=1):
        radial_y = -seat_outer_radius * math.sin(phi)
        radial_z = seat_outer_radius * math.cos(phi)
        model.articulation(
            f"hub_to_{blade_name}_pitch",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=blade_name,
            origin=Origin(xyz=(seat_x, radial_y, radial_z), rpy=(phi, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=180000.0,
                velocity=0.25,
                lower=-0.30,
                upper=1.25,
            ),
            meta={"blade_index": index},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("hub")
    blade_top = object_model.get_part("blade_top")
    blade_lower_left = object_model.get_part("blade_lower_left")
    blade_lower_right = object_model.get_part("blade_lower_right")

    yaw = object_model.get_articulation("tower_to_nacelle_yaw")
    spin = object_model.get_articulation("nacelle_to_hub_spin")
    pitch_top = object_model.get_articulation("hub_to_blade_top_pitch")
    pitch_left = object_model.get_articulation("hub_to_blade_lower_left_pitch")
    pitch_right = object_model.get_articulation("hub_to_blade_lower_right_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for blade_name, seat_name in (
        ("blade_top", "blade_seat_1"),
        ("blade_lower_left", "blade_seat_2"),
        ("blade_lower_right", "blade_seat_3"),
    ):
        ctx.allow_overlap(
            blade_name,
            hub,
            elem_a="root_flange",
            elem_b=seat_name,
            reason="Pitch bearing outer ring is intentionally represented as a nested captured flange inside the hub seat.",
        )
        ctx.allow_overlap(
            blade_name,
            hub,
            elem_a="root_laminate",
            elem_b=seat_name,
            reason="Blade root insert is intentionally nested within the hub-mounted pitch bearing envelope.",
        )

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

    with ctx.pose(
        {
            yaw: 0.0,
            spin: 0.0,
            pitch_top: 0.0,
            pitch_left: 0.0,
            pitch_right: 0.0,
        }
    ):
        ctx.expect_contact(
            tower,
            nacelle,
            elem_a="top_bearing_ring",
            elem_b="yaw_skirt",
            name="tower_to_nacelle_bearing_contact",
        )
        ctx.expect_contact(
            nacelle,
            hub,
            elem_a="bearing_housing",
            elem_b="rear_flange",
            name="nacelle_to_hub_bearing_contact",
        )
        ctx.expect_gap(
            hub,
            nacelle,
            axis="x",
            positive_elem="main_body",
            negative_elem="front_bulkhead",
            min_gap=0.30,
            max_gap=0.45,
            name="hub_clears_front_bulkhead",
        )
        ctx.expect_origin_gap(
            hub,
            nacelle,
            axis="x",
            min_gap=2.05,
            max_gap=2.15,
            name="hub_positioned_forward_of_nacelle",
        )
        ctx.expect_origin_gap(
            blade_top,
            hub,
            axis="x",
            min_gap=1.03,
            max_gap=1.07,
            name="top_blade_root_station",
        )
        ctx.expect_origin_gap(
            blade_lower_left,
            hub,
            axis="x",
            min_gap=1.03,
            max_gap=1.07,
            name="left_blade_root_station",
        )
        ctx.expect_origin_gap(
            blade_lower_right,
            hub,
            axis="x",
            min_gap=1.03,
            max_gap=1.07,
            name="right_blade_root_station",
        )
        ctx.expect_origin_distance(
            blade_top,
            hub,
            axes="yz",
            min_dist=1.40,
            max_dist=1.44,
            name="top_blade_root_radius",
        )
        ctx.expect_origin_distance(
            blade_lower_left,
            hub,
            axes="yz",
            min_dist=1.40,
            max_dist=1.44,
            name="left_blade_root_radius",
        )
        ctx.expect_origin_distance(
            blade_lower_right,
            hub,
            axes="yz",
            min_dist=1.40,
            max_dist=1.44,
            name="right_blade_root_radius",
        )
        ctx.expect_contact(
            blade_top,
            hub,
            elem_a="root_flange",
            elem_b="blade_seat_1",
            name="top_blade_root_bearing_contact",
        )
        ctx.expect_contact(
            blade_lower_left,
            hub,
            elem_a="root_flange",
            elem_b="blade_seat_2",
            name="left_blade_root_bearing_contact",
        )
        ctx.expect_contact(
            blade_lower_right,
            hub,
            elem_a="root_flange",
            elem_b="blade_seat_3",
            name="right_blade_root_bearing_contact",
        )

    with ctx.pose({yaw: yaw.motion_limits.upper}):
        hub_pos = ctx.part_world_position(hub)
        ctx.check(
            "positive_yaw_swings_rotor_to_positive_y",
            hub_pos is not None and hub_pos[1] > 1.5,
            details=f"hub world position at positive yaw: {hub_pos}",
        )

    with ctx.pose({spin: math.pi / 2.0}):
        blade_pos = ctx.part_world_position(blade_top)
        ctx.check(
            "positive_spin_advances_top_blade_toward_negative_y",
            blade_pos is not None and blade_pos[1] < -1.0,
            details=f"top blade world position at quarter turn: {blade_pos}",
        )

    pitch_limits_ok = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower is not None
        and joint.motion_limits.upper is not None
        and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
        and joint.motion_limits.upper <= 1.30
        for joint in (pitch_top, pitch_left, pitch_right)
    )
    ctx.check(
        "blade_pitch_limits_cover_fine_and_feather_ranges",
        pitch_limits_ok,
        details="Expected each blade pitch joint to include fine pitch below 0 and feather above 1 rad.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
