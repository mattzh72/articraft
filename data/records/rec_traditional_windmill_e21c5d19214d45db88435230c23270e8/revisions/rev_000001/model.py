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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _octagon_section(width_x: float, width_y: float, z: float) -> list[tuple[float, float, float]]:
    hx = width_x * 0.5
    hy = width_y * 0.5
    cut = min(hx, hy) * 0.28
    return [
        (hx - cut, hy, z),
        (hx, hy - cut, z),
        (hx, -hy + cut, z),
        (hx - cut, -hy, z),
        (-hx + cut, -hy, z),
        (-hx, -hy + cut, z),
        (-hx, hy - cut, z),
        (-hx + cut, hy, z),
    ]


def _rot_x(y: float, z: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (y * c - z * s, y * s + z * c)


def _add_rotor_blade(rotor, *, index: int, angle: float, hub_x: float, timber, iron) -> None:
    blade_elements = [
        ("stock", (0.16, 0.30, 1.20), 0.00, 0.75, timber),
        ("main_spar", (0.11, 0.18, 4.85), 0.00, 3.05, timber),
        ("leading_edge", (0.06, 0.08, 3.90), 0.52, 3.25, timber),
        ("trailing_edge", (0.05, 0.08, 3.45), -0.48, 3.05, timber),
        ("root_bar", (0.08, 1.00, 0.12), 0.00, 1.20, timber),
        ("tip_bar", (0.08, 1.18, 0.12), 0.00, 5.10, timber),
    ]
    for element_name, size, y_local, z_local, material in blade_elements:
        y_pos, z_pos = _rot_x(y_local, z_local, angle)
        rotor.visual(
            Box(size),
            origin=Origin(xyz=(hub_x, y_pos, z_pos), rpy=(angle, 0.0, 0.0)),
            material=material,
            name=f"blade_{index}_{element_name}",
        )
    for slat_index, z_local in enumerate((1.55, 2.00, 2.45, 2.90, 3.35, 3.80, 4.25, 4.70)):
        y_pos, z_pos = _rot_x(0.00, z_local, angle)
        rotor.visual(
            Box((0.035, 0.98, 0.055)),
            origin=Origin(xyz=(hub_x, y_pos, z_pos), rpy=(angle, 0.0, 0.0)),
            material=timber,
            name=f"blade_{index}_slat_{slat_index}",
        )
    brace_specs = [
        (0.28, 2.10, 0.045),
        (-0.22, 2.55, 0.045),
        (0.18, 3.60, 0.040),
    ]
    for brace_index, (y0, z0, radius) in enumerate(brace_specs):
        a_local = (hub_x, y0, z0 - 0.65)
        b_local = (hub_x, 0.0, z0 + 0.35)
        ay, az = _rot_x(a_local[1], a_local[2], angle)
        by, bz = _rot_x(b_local[1], b_local[2], angle)
        _add_member(
            rotor,
            (a_local[0], ay, az),
            (b_local[0], by, bz),
            radius=radius,
            material=iron,
            name=f"blade_{index}_brace_{brace_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    whitewash = model.material("whitewash", rgba=(0.90, 0.88, 0.82, 1.0))
    weathered_timber = model.material("weathered_timber", rgba=(0.53, 0.42, 0.29, 1.0))
    tarred_timber = model.material("tarred_timber", rgba=(0.20, 0.24, 0.20, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    red_oxide = model.material("red_oxide", rgba=(0.42, 0.18, 0.12, 1.0))
    slate = model.material("slate", rgba=(0.24, 0.25, 0.28, 1.0))

    tower = model.part("tower")

    tower_shell = _save_mesh(
        "tower_shell",
        section_loft(
            [
                _octagon_section(4.80, 4.60, 0.30),
                _octagon_section(4.30, 4.10, 2.80),
                _octagon_section(3.55, 3.35, 5.80),
                _octagon_section(2.70, 2.55, 8.25),
            ]
        ),
    )
    tower.visual(tower_shell, material=whitewash, name="tower_shell")
    tower.visual(
        Box((5.60, 5.20, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=weathered_timber,
        name="foundation_plinth",
    )
    tower.visual(
        Cylinder(radius=1.78, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 8.28)),
        material=weathered_timber,
        name="service_gallery",
    )
    tower.visual(
        Cylinder(radius=1.26, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 8.36)),
        material=dark_iron,
        name="yaw_track",
    )
    tower.visual(
        Box((0.08, 1.06, 2.05)),
        origin=Origin(xyz=(2.40, 0.0, 1.13)),
        material=red_oxide,
        name="service_door",
    )
    tower.visual(
        Box((0.60, 0.14, 0.10)),
        origin=Origin(xyz=(2.38, 0.0, 2.20)),
        material=weathered_timber,
        name="door_lintel",
    )

    ladder_y = -2.22
    ladder_x_left = -0.17
    ladder_x_right = 0.17
    ladder_bottom = 0.65
    ladder_top = 7.95
    _add_member(
        tower,
        (ladder_x_left, ladder_y, ladder_bottom),
        (ladder_x_left, ladder_y, ladder_top),
        radius=0.03,
        material=dark_iron,
        name="ladder_rail_left",
    )
    _add_member(
        tower,
        (ladder_x_right, ladder_y, ladder_bottom),
        (ladder_x_right, ladder_y, ladder_top),
        radius=0.03,
        material=dark_iron,
        name="ladder_rail_right",
    )
    for rung_index in range(20):
        z = ladder_bottom + rung_index * 0.38
        _add_member(
            tower,
            (ladder_x_left, ladder_y, z),
            (ladder_x_right, ladder_y, z),
            radius=0.018,
            material=dark_iron,
            name=f"ladder_rung_{rung_index}",
        )
    for z, shell_anchor_y in ((1.0, -1.95), (2.7, -1.82), (4.4, -1.66), (6.1, -1.52), (7.8, -1.38)):
        _add_member(
            tower,
            (ladder_x_left, shell_anchor_y, z),
            (ladder_x_left, ladder_y, z),
            radius=0.020,
            material=dark_iron,
            name=f"ladder_standoff_left_{int(z * 10):02d}",
        )
        _add_member(
            tower,
            (ladder_x_right, shell_anchor_y, z),
            (ladder_x_right, ladder_y, z),
            radius=0.020,
            material=dark_iron,
            name=f"ladder_standoff_right_{int(z * 10):02d}",
        )

    rail_points = [
        (1.60, 0.00, 8.36),
        (1.12, 1.12, 8.36),
        (0.00, 1.58, 8.36),
        (-1.12, 1.12, 8.36),
        (-1.26, 0.34, 8.36),
        (-1.26, -0.34, 8.36),
        (-0.88, -1.16, 8.36),
        (0.00, -1.46, 8.36),
    ]
    for rail_index, point in enumerate(rail_points):
        _add_member(
            tower,
            point,
            (point[0], point[1], 8.58),
            radius=0.022,
            material=dark_iron,
            name=f"gallery_post_{rail_index}",
        )
    for rail_index in range(len(rail_points) - 1):
        _add_member(
            tower,
            (rail_points[rail_index][0], rail_points[rail_index][1], 8.58),
            (rail_points[rail_index + 1][0], rail_points[rail_index + 1][1], 8.58),
            radius=0.024,
            material=dark_iron,
            name=f"gallery_rail_{rail_index}",
        )

    tower.inertial = Inertial.from_geometry(
        Box((5.60, 5.20, 8.55)),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.275)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=1.12, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_iron,
        name="bed_ring",
    )
    cap.visual(
        Box((2.65, 2.25, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=weathered_timber,
        name="bed_frame",
    )
    cap.visual(
        Box((2.30, 0.20, 1.05)),
        origin=Origin(xyz=(0.0, 1.03, 0.84)),
        material=tarred_timber,
        name="left_sidewall",
    )
    cap.visual(
        Box((2.30, 0.20, 1.05)),
        origin=Origin(xyz=(0.0, -1.03, 0.84)),
        material=tarred_timber,
        name="right_sidewall",
    )
    cap.visual(
        Box((0.24, 1.42, 0.96)),
        origin=Origin(xyz=(-1.06, 0.0, 0.82)),
        material=tarred_timber,
        name="rear_bulkhead",
    )
    cap.visual(
        Box((0.44, 0.58, 1.10)),
        origin=Origin(xyz=(0.78, 0.64, 0.97)),
        material=tarred_timber,
        name="front_cheek_left",
    )
    cap.visual(
        Box((0.44, 0.58, 1.10)),
        origin=Origin(xyz=(0.78, -0.64, 0.97)),
        material=tarred_timber,
        name="front_cheek_right",
    )
    cap.visual(
        Box((0.36, 1.10, 0.18)),
        origin=Origin(xyz=(0.74, 0.0, 0.96)),
        material=weathered_timber,
        name="front_lower_crossbeam",
    )
    cap.visual(
        Box((0.50, 1.24, 0.16)),
        origin=Origin(xyz=(0.70, 0.0, 1.82)),
        material=weathered_timber,
        name="front_headbeam",
    )
    cap.visual(
        Box((0.30, 0.18, 0.86)),
        origin=Origin(xyz=(0.72, 0.43, 1.44)),
        material=dark_iron,
        name="front_bearing_left",
    )
    cap.visual(
        Box((0.30, 0.18, 0.86)),
        origin=Origin(xyz=(0.72, -0.43, 1.44)),
        material=dark_iron,
        name="front_bearing_right",
    )
    cap.visual(
        Box((0.30, 1.04, 0.16)),
        origin=Origin(xyz=(0.72, 0.0, 1.78)),
        material=weathered_timber,
        name="front_bearing_cap",
    )
    cap.visual(
        Box((0.34, 1.04, 0.22)),
        origin=Origin(xyz=(0.70, 0.0, 1.11)),
        material=weathered_timber,
        name="front_bearing_base",
    )
    cap.visual(
        Box((0.28, 0.18, 0.82)),
        origin=Origin(xyz=(0.12, 0.41, 1.44)),
        material=dark_iron,
        name="rear_bearing_left",
    )
    cap.visual(
        Box((0.28, 0.18, 0.82)),
        origin=Origin(xyz=(0.12, -0.41, 1.44)),
        material=dark_iron,
        name="rear_bearing_right",
    )
    cap.visual(
        Box((0.30, 1.00, 0.16)),
        origin=Origin(xyz=(0.10, 0.0, 1.76)),
        material=weathered_timber,
        name="rear_bearing_cap",
    )
    cap.visual(
        Box((0.34, 1.00, 0.20)),
        origin=Origin(xyz=(0.10, 0.0, 1.14)),
        material=weathered_timber,
        name="rear_bearing_base",
    )
    cap.visual(
        Box((1.10, 0.18, 0.20)),
        origin=Origin(xyz=(0.14, 0.58, 1.25)),
        material=weathered_timber,
        name="left_shaft_rail",
    )
    cap.visual(
        Box((1.10, 0.18, 0.20)),
        origin=Origin(xyz=(0.14, -0.58, 1.25)),
        material=weathered_timber,
        name="right_shaft_rail",
    )
    cap.visual(
        Box((1.58, 0.74, 0.12)),
        origin=Origin(xyz=(-0.12, 0.54, 1.90), rpy=(0.46, 0.0, 0.0)),
        material=slate,
        name="roof_left",
    )
    cap.visual(
        Box((1.58, 0.74, 0.12)),
        origin=Origin(xyz=(-0.12, -0.54, 1.90), rpy=(-0.46, 0.0, 0.0)),
        material=slate,
        name="roof_right",
    )
    cap.visual(
        Box((1.62, 0.14, 0.14)),
        origin=Origin(xyz=(-0.12, 0.0, 2.03)),
        material=slate,
        name="roof_ridge",
    )
    cap.visual(
        Box((0.48, 1.14, 0.12)),
        origin=Origin(xyz=(0.95, 0.0, 1.74)),
        material=slate,
        name="nose_cowl_top",
    )
    cap.visual(
        Box((0.28, 0.16, 0.44)),
        origin=Origin(xyz=(0.94, 0.58, 1.45)),
        material=slate,
        name="nose_cowl_left",
    )
    cap.visual(
        Box((0.28, 0.16, 0.44)),
        origin=Origin(xyz=(0.94, -0.58, 1.45)),
        material=slate,
        name="nose_cowl_right",
    )
    cap.visual(
        Box((0.36, 0.96, 0.52)),
        origin=Origin(xyz=(-0.82, 0.0, 1.18)),
        material=tarred_timber,
        name="service_hatch_frame",
    )
    cap.visual(
        Box((0.06, 0.78, 0.40)),
        origin=Origin(xyz=(-0.98, 0.0, 1.18)),
        material=red_oxide,
        name="service_hatch_door",
    )
    _add_member(
        cap,
        (-1.06, 0.40, 1.15),
        (-1.95, 0.0, 1.02),
        radius=0.065,
        material=weathered_timber,
        name="tailbeam_left",
    )
    _add_member(
        cap,
        (-1.06, -0.40, 1.15),
        (-1.95, 0.0, 1.02),
        radius=0.065,
        material=weathered_timber,
        name="tailbeam_right",
    )
    _add_member(
        cap,
        (0.78, 0.94, 1.34),
        (-0.10, 0.10, 1.98),
        radius=0.045,
        material=weathered_timber,
        name="rafter_front_left",
    )
    _add_member(
        cap,
        (0.78, -0.94, 1.34),
        (-0.10, -0.10, 1.98),
        radius=0.045,
        material=weathered_timber,
        name="rafter_front_right",
    )
    _add_member(
        cap,
        (-1.02, 0.66, 1.26),
        (-0.36, 0.10, 1.98),
        radius=0.045,
        material=weathered_timber,
        name="rafter_rear_left",
    )
    _add_member(
        cap,
        (-1.02, -0.66, 1.26),
        (-0.36, -0.10, 1.98),
        radius=0.045,
        material=weathered_timber,
        name="rafter_rear_right",
    )
    cap.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(-2.05, 0.0, 1.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="tailwheel",
    )
    _add_member(
        cap,
        (-1.94, 0.10, 1.03),
        (-2.05, 0.10, 1.00),
        radius=0.032,
        material=dark_iron,
        name="tail_fork_left",
    )
    _add_member(
        cap,
        (-1.94, -0.10, 1.03),
        (-2.05, -0.10, 1.00),
        radius=0.032,
        material=dark_iron,
        name="tail_fork_right",
    )
    cap.visual(
        Cylinder(radius=0.028, length=0.28),
        origin=Origin(xyz=(-2.05, 0.0, 1.00), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="tailwheel_axle",
    )

    cap.inertial = Inertial.from_geometry(
        Box((4.40, 2.60, 2.20)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.12, length=2.10),
        origin=Origin(xyz=(0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="windshaft",
    )
    rotor.visual(
        Cylinder(radius=0.22, length=0.14),
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red_oxide,
        name="front_wear_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.20, length=0.14),
        origin=Origin(xyz=(-0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red_oxide,
        name="rear_wear_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.34, length=0.66),
        origin=Origin(xyz=(1.55, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.26, length=0.26),
        origin=Origin(xyz=(1.95, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="nose_cone",
    )
    rotor.visual(
        Box((0.44, 0.64, 0.64)),
        origin=Origin(xyz=(1.55, 0.0, 0.0)),
        material=dark_iron,
        name="hub_block",
    )
    rotor.visual(
        Box((0.30, 0.58, 1.18)),
        origin=Origin(xyz=(1.61, 0.0, 0.0)),
        material=weathered_timber,
        name="vertical_stock",
    )
    rotor.visual(
        Box((0.30, 1.18, 0.58)),
        origin=Origin(xyz=(1.61, 0.0, 0.0)),
        material=weathered_timber,
        name="horizontal_stock",
    )
    for blade_index, blade_angle in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        _add_rotor_blade(
            rotor,
            index=blade_index,
            angle=blade_angle,
            hub_x=1.82,
            timber=weathered_timber,
            iron=dark_iron,
        )

    rotor.inertial = Inertial.from_geometry(
        Box((1.90, 11.80, 11.80)),
        mass=2600.0,
        origin=Origin(xyz=(1.24, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 8.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22000.0, velocity=0.25),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.42, 0.0, 1.44)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    cap_yaw = object_model.get_articulation("tower_to_cap")
    rotor_spin = object_model.get_articulation("cap_to_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "cap_yaw_axis_is_vertical",
        cap_yaw.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical cap yaw axis, got {cap_yaw.axis}",
    )
    ctx.check(
        "rotor_spin_axis_is_windshaft_axis",
        rotor_spin.axis == (1.0, 0.0, 0.0),
        details=f"expected rotor spin around +X, got {rotor_spin.axis}",
    )

    ctx.expect_contact(
        cap,
        tower,
        elem_a="bed_ring",
        elem_b="yaw_track",
        contact_tol=1e-4,
        name="cap_bed_ring_seats_on_yaw_track",
    )
    ctx.expect_contact(
        rotor,
        cap,
        elem_a="front_wear_sleeve",
        elem_b="front_bearing_base",
        contact_tol=2e-4,
        name="front_bearing_base_seats_on_wear_sleeve",
    )
    ctx.expect_contact(
        rotor,
        cap,
        elem_a="rear_wear_sleeve",
        elem_b="rear_bearing_base",
        contact_tol=2e-4,
        name="rear_bearing_base_seats_on_wear_sleeve",
    )
    ctx.expect_gap(
        rotor,
        cap,
        axis="x",
        positive_elem="hub_barrel",
        negative_elem="front_cheek_left",
        min_gap=0.05,
        name="hub_projects_clear_of_cap_front_frame",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="xy",
        elem_a="front_wear_sleeve",
        elem_b="front_bearing_base",
        min_overlap=0.12,
        name="front_bearing_aligned_with_rotor_stage",
    )
    ctx.expect_gap(
        cap,
        rotor,
        axis="y",
        positive_elem="front_bearing_left",
        negative_elem="front_wear_sleeve",
        min_gap=0.08,
        name="front_left_pedestal_clears_rotating_sleeve",
    )
    ctx.expect_gap(
        rotor,
        cap,
        axis="y",
        positive_elem="front_wear_sleeve",
        negative_elem="front_bearing_right",
        min_gap=0.08,
        name="front_right_pedestal_clears_rotating_sleeve",
    )

    with ctx.pose({cap_yaw: math.pi / 2.0}):
        rotor_pos = ctx.part_world_position(rotor)
        ctx.check(
            "cap_yaw_swings_rotor_around_tower",
            rotor_pos is not None and abs(rotor_pos[0]) < 0.08 and rotor_pos[1] > 0.35,
            details=f"expected rotor origin near +Y after quarter-turn, got {rotor_pos}",
        )

    tip_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0_tip_bar")
    if tip_aabb is not None:
        tip_center = tuple((tip_aabb[0][i] + tip_aabb[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "rest_pose_has_one_blade_up",
            tip_center[2] > 6.2,
            details=f"expected blade_0 tip high above tower, got center {tip_center}",
        )
    else:
        ctx.fail("rest_pose_has_one_blade_up", "blade_0_tip_bar AABB unavailable")

    with ctx.pose({rotor_spin: math.pi / 2.0}):
        spun_tip_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0_tip_bar")
        if spun_tip_aabb is not None:
            spun_tip_center = tuple((spun_tip_aabb[0][i] + spun_tip_aabb[1][i]) * 0.5 for i in range(3))
            ctx.check(
                "rotor_spin_rotates_blade_into_side_position",
                spun_tip_center[1] < -4.9 and 9.6 < spun_tip_center[2] < 10.2,
                details=f"expected upper blade to rotate toward -Y near hub height, got {spun_tip_center}",
            )
        else:
            ctx.fail(
                "rotor_spin_rotates_blade_into_side_position",
                "blade_0_tip_bar AABB unavailable in spun pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
