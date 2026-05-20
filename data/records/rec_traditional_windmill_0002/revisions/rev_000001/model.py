from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _octagon_section(rx: float, ry: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (0.38 * rx, ry, z),
        (rx, 0.38 * ry, z),
        (rx, -0.38 * ry, z),
        (0.38 * rx, -ry, z),
        (-0.38 * rx, -ry, z),
        (-rx, -0.38 * ry, z),
        (-rx, 0.38 * ry, z),
        (-0.38 * rx, ry, z),
    ]


def _cap_section(
    *,
    y: float,
    width: float,
    height: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    radius = min(width, height) * 0.22
    return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius)]


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def _build_tower_shell_mesh():
    sections = [
        _octagon_section(2.05, 2.05, 0.45),
        _octagon_section(1.92, 1.92, 2.90),
        _octagon_section(1.72, 1.72, 5.70),
        _octagon_section(1.42, 1.42, 8.10),
    ]
    return section_loft(sections)


def _build_cap_shell_mesh():
    sections = [
        _cap_section(y=-1.25, width=1.70, height=0.92, z_center=0.82),
        _cap_section(y=-0.40, width=2.28, height=1.38, z_center=0.97),
        _cap_section(y=0.55, width=2.18, height=1.32, z_center=1.00),
        _cap_section(y=1.35, width=1.44, height=0.78, z_center=0.94),
    ]
    return section_loft(sections)


def _add_vertical_sail(
    rotor,
    *,
    sign: float,
    y_offset: float,
    timber,
    lattice,
    tip_name: str | None = None,
) -> None:
    rotor.visual(
        Box((0.24, 0.18, 0.62)),
        origin=Origin(xyz=(0.0, y_offset, sign * 0.72)),
        material=timber,
    )
    rotor.visual(
        Box((0.18, 0.24, 3.30)),
        origin=Origin(xyz=(0.0, y_offset, sign * 2.42)),
        material=timber,
    )
    for rail_x in (-0.48, 0.48):
        rotor.visual(
            Box((0.10, 0.16, 2.72)),
            origin=Origin(xyz=(rail_x, y_offset, sign * 2.56)),
            material=timber,
        )
    for z_pos in (1.22, 1.66, 2.10, 2.54, 2.98, 3.42):
        rotor.visual(
            Box((1.04, 0.06, 0.08)),
            origin=Origin(xyz=(0.0, y_offset, sign * z_pos)),
            material=lattice,
        )
    rotor.visual(
        Box((1.12, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, y_offset, sign * 4.04)),
        material=lattice,
        name=tip_name,
    )
    _add_member(
        rotor,
        (0.0, y_offset, sign * 0.86),
        (0.48, y_offset, sign * 1.66),
        radius=0.04,
        material=timber,
    )
    _add_member(
        rotor,
        (0.0, y_offset, sign * 0.86),
        (-0.48, y_offset, sign * 1.66),
        radius=0.04,
        material=timber,
    )


def _add_horizontal_sail(
    rotor,
    *,
    sign: float,
    y_offset: float,
    timber,
    lattice,
) -> None:
    rotor.visual(
        Box((0.62, 0.18, 0.24)),
        origin=Origin(xyz=(sign * 0.72, y_offset, 0.0)),
        material=timber,
    )
    rotor.visual(
        Box((3.30, 0.24, 0.18)),
        origin=Origin(xyz=(sign * 2.42, y_offset, 0.0)),
        material=timber,
    )
    for rail_z in (-0.48, 0.48):
        rotor.visual(
            Box((2.72, 0.16, 0.10)),
            origin=Origin(xyz=(sign * 2.56, y_offset, rail_z)),
            material=timber,
        )
    for x_pos in (1.22, 1.66, 2.10, 2.54, 2.98, 3.42):
        rotor.visual(
            Box((0.08, 0.06, 1.04)),
            origin=Origin(xyz=(sign * x_pos, y_offset, 0.0)),
            material=lattice,
        )
    rotor.visual(
        Box((0.12, 0.12, 1.12)),
        origin=Origin(xyz=(sign * 4.04, y_offset, 0.0)),
        material=lattice,
    )
    _add_member(
        rotor,
        (sign * 0.86, y_offset, 0.0),
        (sign * 1.66, y_offset, 0.48),
        radius=0.04,
        material=timber,
    )
    _add_member(
        rotor,
        (sign * 0.86, y_offset, 0.0),
        (sign * 1.66, y_offset, -0.48),
        radius=0.04,
        material=timber,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill_utility", assets=ASSETS)

    stone = model.material("stone", rgba=(0.58, 0.55, 0.50, 1.0))
    whitewash = model.material("whitewash", rgba=(0.84, 0.82, 0.77, 1.0))
    timber = model.material("timber", rgba=(0.33, 0.23, 0.15, 1.0))
    timber_dark = model.material("timber_dark", rgba=(0.19, 0.14, 0.09, 1.0))
    cap_paint = model.material("cap_paint", rgba=(0.44, 0.15, 0.11, 1.0))
    weathered_sail = model.material("weathered_sail", rgba=(0.79, 0.76, 0.67, 1.0))
    iron = model.material("iron", rgba=(0.28, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.64, 1.0))
    oxide = model.material("oxide", rgba=(0.46, 0.18, 0.12, 1.0))

    tower = model.part("tower_body")
    tower.visual(
        Cylinder(radius=2.35, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=stone,
        name="foundation_plinth",
    )
    tower.visual(
        Cylinder(radius=2.12, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=stone,
        name="base_ring",
    )
    tower.visual(
        _save_mesh("tower_shell.obj", _build_tower_shell_mesh()),
        material=whitewash,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=1.48, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 8.22)),
        material=timber_dark,
        name="tower_curb",
    )

    tower_levels = [
        (0.60, 2.00, 2.00),
        (2.90, 1.88, 1.88),
        (5.70, 1.68, 1.68),
        (8.05, 1.42, 1.42),
    ]
    tower_sections = [_octagon_section(rx, ry, z) for z, rx, ry in tower_levels]
    for lower, upper in zip(tower_sections[:-1], tower_sections[1:]):
        for start, end in zip(lower, upper):
            _add_member(tower, start, end, radius=0.075, material=timber_dark)
    for section in tower_sections[:-1]:
        for start, end in zip(section, section[1:] + section[:1]):
            _add_member(tower, start, end, radius=0.055, material=timber_dark)

    tower.visual(
        Box((1.18, 0.12, 2.16)),
        origin=Origin(xyz=(0.0, -2.08, 1.18)),
        material=timber_dark,
        name="door_jamb",
    )
    tower.visual(
        Box((1.28, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, -2.02, 0.06)),
        material=stone,
        name="door_threshold",
    )
    tower.visual(
        Box((1.10, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -2.08, 2.20)),
        material=timber_dark,
    )
    tower.visual(
        Cylinder(radius=0.020, length=1.86),
        origin=Origin(xyz=(-0.48, -2.18, 1.23)),
        material=steel,
        name="door_hinge_pin",
    )

    ladder_x = 1.72
    ladder_y = 0.62
    ladder_bottom = 0.72
    ladder_top = 6.85
    _add_member(
        tower,
        (ladder_x, ladder_y - 0.12, ladder_bottom),
        (ladder_x, ladder_y - 0.12, ladder_top),
        radius=0.03,
        material=steel,
    )
    _add_member(
        tower,
        (ladder_x, ladder_y + 0.12, ladder_bottom),
        (ladder_x, ladder_y + 0.12, ladder_top),
        radius=0.03,
        material=steel,
    )
    for index in range(18):
        z_pos = ladder_bottom + (ladder_top - ladder_bottom) * index / 17.0
        _add_member(
            tower,
            (ladder_x, ladder_y - 0.12, z_pos),
            (ladder_x, ladder_y + 0.12, z_pos),
            radius=0.02,
            material=steel,
        )

    tower.inertial = Inertial.from_geometry(
        Box((4.70, 4.70, 8.40)),
        mass=1200.0,
        origin=Origin(xyz=(0.0, 0.0, 4.20)),
    )

    door = model.part("access_door")
    door.visual(
        Cylinder(radius=0.030, length=1.80),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material=steel,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.82, 0.08, 1.90)),
        origin=Origin(xyz=(0.47, 0.0, 0.95)),
        material=oxide,
        name="door_leaf",
    )
    door.visual(
        Box((0.10, 0.06, 0.24)),
        origin=Origin(xyz=(0.05, -0.05, 0.40)),
        material=steel,
        name="upper_hinge_strap",
    )
    door.visual(
        Box((0.10, 0.06, 0.24)),
        origin=Origin(xyz=(0.05, -0.05, 1.40)),
        material=steel,
        name="lower_hinge_strap",
    )
    door.visual(
        Box((0.62, 0.04, 0.16)),
        origin=Origin(xyz=(0.48, 0.0, 0.38)),
        material=timber_dark,
    )
    door.visual(
        Box((0.62, 0.04, 0.16)),
        origin=Origin(xyz=(0.48, 0.0, 1.42)),
        material=timber_dark,
    )
    door.visual(
        Box((0.12, 0.10, 0.14)),
        origin=Origin(xyz=(0.76, -0.01, 0.98)),
        material=steel,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.86, 0.16, 1.90)),
        mass=18.0,
        origin=Origin(xyz=(0.45, 0.0, 0.95)),
    )

    cap = model.part("roof_cap")
    cap.visual(
        Cylinder(radius=1.58, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=iron,
        name="yaw_skirt",
    )
    cap.visual(
        _save_mesh("cap_shell.obj", _build_cap_shell_mesh()),
        material=cap_paint,
        name="cap_shell",
    )
    cap.visual(
        Box((0.18, 2.10, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 1.38)),
        material=timber_dark,
        name="roof_ridge",
    )
    cap.visual(
        Box((0.24, 0.42, 1.04)),
        origin=Origin(xyz=(-0.56, 0.96, 0.96)),
        material=timber_dark,
    )
    cap.visual(
        Box((0.24, 0.42, 1.04)),
        origin=Origin(xyz=(0.56, 0.96, 0.96)),
        material=timber_dark,
    )
    cap.visual(
        Box((1.38, 0.38, 0.28)),
        origin=Origin(xyz=(0.0, 1.04, 1.10)),
        material=timber_dark,
        name="front_headframe",
    )
    cap.visual(
        Box((1.12, 0.08, 1.12)),
        origin=Origin(xyz=(0.0, 1.32, 1.08)),
        material=steel,
        name="thrust_bearing",
    )
    cap.visual(
        Box((0.24, 0.24, 0.42)),
        origin=Origin(xyz=(-0.52, 1.22, 1.08)),
        material=iron,
        name="front_bearing_housing",
    )
    cap.visual(
        Box((0.24, 0.24, 0.42)),
        origin=Origin(xyz=(0.52, 1.22, 1.08)),
        material=iron,
    )
    _add_member(
        cap,
        (-0.56, 0.56, 0.64),
        (-0.56, 1.02, 1.26),
        radius=0.06,
        material=timber_dark,
    )
    _add_member(
        cap,
        (0.56, 0.56, 0.64),
        (0.56, 1.02, 1.26),
        radius=0.06,
        material=timber_dark,
    )
    cap.visual(
        Box((0.18, 0.36, 0.34)),
        origin=Origin(xyz=(-1.10, -0.72, 0.96)),
        material=timber_dark,
        name="lever_bracket",
    )
    _add_member(
        cap,
        (-0.98, -1.02, 0.72),
        (-0.80, -0.46, 1.14),
        radius=0.05,
        material=timber_dark,
    )
    cap.visual(
        Cylinder(radius=0.038, length=0.18),
        origin=Origin(xyz=(-1.28, -0.70, 0.96), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_pivot_pin",
    )
    for bolt_x in (-0.42, 0.42):
        for bolt_z in (0.82, 1.34):
            cap.visual(
                Cylinder(radius=0.028, length=0.10),
                origin=Origin(
                    xyz=(bolt_x, 1.20, bolt_z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
            )
    cap.inertial = Inertial.from_geometry(
        Box((3.10, 3.10, 2.00)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )

    rotor = model.part("rotor_assembly")
    sail_plane_y = 0.82
    rotor.visual(
        Cylinder(radius=0.64, length=0.14),
        origin=Origin(xyz=(0.0, 0.07, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_back_plate",
    )
    rotor.visual(
        Cylinder(radius=0.52, length=0.54),
        origin=Origin(xyz=(0.0, 0.34, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.14, length=1.14),
        origin=Origin(xyz=(0.0, 0.60, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="windshaft",
    )
    rotor.visual(
        Box((1.46, 0.18, 0.24)),
        origin=Origin(xyz=(0.0, sail_plane_y, 0.0)),
        material=timber_dark,
    )
    rotor.visual(
        Box((0.24, 0.18, 1.46)),
        origin=Origin(xyz=(0.0, sail_plane_y, 0.0)),
        material=timber_dark,
    )
    _add_vertical_sail(
        rotor,
        sign=1.0,
        y_offset=sail_plane_y,
        timber=timber,
        lattice=weathered_sail,
        tip_name="top_tip_panel",
    )
    _add_vertical_sail(
        rotor,
        sign=-1.0,
        y_offset=sail_plane_y,
        timber=timber,
        lattice=weathered_sail,
    )
    _add_horizontal_sail(
        rotor,
        sign=1.0,
        y_offset=sail_plane_y,
        timber=timber,
        lattice=weathered_sail,
    )
    _add_horizontal_sail(
        rotor,
        sign=-1.0,
        y_offset=sail_plane_y,
        timber=timber,
        lattice=weathered_sail,
    )
    rotor.inertial = Inertial.from_geometry(
        Box((8.40, 1.20, 8.40)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.60, 0.0)),
    )

    brake_lever = model.part("brake_lever")
    brake_lever.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_socket",
    )
    brake_lever.visual(
        Box((0.05, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, -0.05, 0.065)),
        material=iron,
        name="lever_gusset",
    )
    brake_lever.visual(
        Box((0.05, 0.54, 0.08)),
        origin=Origin(xyz=(0.0, -0.32, 0.105)),
        material=timber_dark,
        name="lever_arm",
    )
    brake_lever.visual(
        Box((0.06, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, -0.16, 0.105)),
        material=iron,
    )
    brake_lever.visual(
        Cylinder(radius=0.03, length=0.18),
        origin=Origin(xyz=(0.0, -0.60, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_handle",
    )
    brake_lever.inertial = Inertial.from_geometry(
        Box((0.18, 0.68, 0.20)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.30, 0.105)),
    )

    model.articulation(
        "tower_to_door",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(xyz=(-0.48, -2.18, 0.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=-math.radians(110.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 8.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.28),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.0, 1.36, 1.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.70),
    )
    model.articulation(
        "cap_to_brake_lever",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=brake_lever,
        origin=Origin(xyz=(-1.28, -0.70, 0.96)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-0.40,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower_body")
    door = object_model.get_part("access_door")
    cap = object_model.get_part("roof_cap")
    rotor = object_model.get_part("rotor_assembly")
    brake_lever = object_model.get_part("brake_lever")

    door_joint = object_model.get_articulation("tower_to_door")
    cap_joint = object_model.get_articulation("tower_to_cap")
    rotor_joint = object_model.get_articulation("cap_to_rotor")
    lever_joint = object_model.get_articulation("cap_to_brake_lever")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        tower,
        door,
        elem_a="door_hinge_pin",
        elem_b="door_hinge_barrel",
        reason="The access door rides on a fixed external hinge pin.",
    )
    ctx.allow_overlap(
        cap,
        brake_lever,
        elem_a="lever_pivot_pin",
        elem_b="lever_socket",
        reason="The external brake lever pivots around a through-pin bracket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        cap,
        tower,
        elem_a="yaw_skirt",
        elem_b="tower_curb",
        contact_tol=0.002,
        name="cap_seated_on_tower_curb",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        elem_a="yaw_skirt",
        elem_b="tower_curb",
        min_overlap=2.60,
        name="cap_curb_footprint_overlap",
    )
    ctx.expect_gap(
        rotor,
        cap,
        axis="y",
        positive_elem="hub_back_plate",
        negative_elem="thrust_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotor_thrust_bearing_seat",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="xz",
        elem_a="hub_back_plate",
        elem_b="thrust_bearing",
        min_overlap=1.00,
        name="rotor_bearing_face_overlap",
    )
    ctx.expect_gap(
        tower,
        door,
        axis="y",
        positive_elem="door_jamb",
        negative_elem="door_leaf",
        max_gap=0.002,
        max_penetration=0.0,
        name="door_closed_against_jamb",
    )
    ctx.expect_overlap(
        tower,
        door,
        axes="xz",
        elem_a="door_jamb",
        elem_b="door_leaf",
        min_overlap=0.80,
        name="door_covers_opening",
    )
    ctx.expect_contact(
        tower,
        door,
        elem_a="door_hinge_pin",
        elem_b="door_hinge_barrel",
        name="door_hinge_contact",
    )
    ctx.expect_contact(
        cap,
        brake_lever,
        elem_a="lever_pivot_pin",
        elem_b="lever_socket",
        name="brake_lever_pivot_contact",
    )

    tower_aabb = ctx.part_world_aabb(tower)
    rotor_aabb = ctx.part_world_aabb(rotor)
    cap_aabb = ctx.part_world_aabb(cap)
    assert tower_aabb is not None
    assert rotor_aabb is not None
    assert cap_aabb is not None

    tower_height = tower_aabb[1][2] - tower_aabb[0][2]
    rotor_span_x = rotor_aabb[1][0] - rotor_aabb[0][0]
    rotor_span_z = rotor_aabb[1][2] - rotor_aabb[0][2]
    cap_height = cap_aabb[1][2] - cap_aabb[0][2]

    ctx.check(
        "tower_height_realistic",
        tower_height > 8.0,
        f"Tower height too small: {tower_height:.3f} m",
    )
    ctx.check(
        "rotor_span_realistic_x",
        rotor_span_x > 7.5,
        f"Rotor span in x too small: {rotor_span_x:.3f} m",
    )
    ctx.check(
        "rotor_span_realistic_z",
        rotor_span_z > 7.5,
        f"Rotor span in z too small: {rotor_span_z:.3f} m",
    )
    ctx.check(
        "cap_reads_substantial",
        cap_height > 1.4,
        f"Cap height too small: {cap_height:.3f} m",
    )

    rest_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="top_tip_panel"))
    rest_door_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="door_handle"))
    rest_lever_handle = _aabb_center(ctx.part_element_world_aabb(brake_lever, elem="lever_handle"))
    assert rest_tip is not None
    assert rest_door_handle is not None
    assert rest_lever_handle is not None

    door_limits = door_joint.motion_limits
    assert door_limits is not None
    assert door_limits.lower is not None
    with ctx.pose({door_joint: door_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_contact(
            tower,
            door,
            elem_a="door_hinge_pin",
            elem_b="door_hinge_barrel",
            name="door_open_hinge_contact",
        )
        open_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="door_handle"))
        assert open_handle is not None
        ctx.check(
            "door_swings_clear",
            open_handle[1] < rest_door_handle[1] - 0.60 and open_handle[0] < rest_door_handle[0] - 0.40,
            f"Door handle did not move enough when opened: rest={rest_door_handle}, open={open_handle}",
        )

    lever_limits = lever_joint.motion_limits
    assert lever_limits is not None
    assert lever_limits.lower is not None
    assert lever_limits.upper is not None
    with ctx.pose({lever_joint: lever_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="brake_lever_raised_no_overlap")
        ctx.fail_if_isolated_parts(name="brake_lever_raised_no_floating")
        ctx.expect_contact(
            cap,
            brake_lever,
            elem_a="lever_pivot_pin",
            elem_b="lever_socket",
            name="brake_lever_raised_pivot_contact",
        )
        raised_handle = _aabb_center(ctx.part_element_world_aabb(brake_lever, elem="lever_handle"))
        assert raised_handle is not None
        ctx.check(
            "brake_lever_raises",
            raised_handle[2] > rest_lever_handle[2] + 0.18,
            f"Brake lever handle did not rise enough: rest={rest_lever_handle}, raised={raised_handle}",
        )
    with ctx.pose({lever_joint: lever_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="brake_lever_lowered_no_overlap")
        ctx.fail_if_isolated_parts(name="brake_lever_lowered_no_floating")

    with ctx.pose({rotor_joint: math.pi / 4.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotor_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="rotor_quarter_turn_no_floating")
        spun_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="top_tip_panel"))
        assert spun_tip is not None
        ctx.check(
            "rotor_tip_moves_with_spin",
            spun_tip[0] > rest_tip[0] + 2.20 and spun_tip[2] < rest_tip[2] - 0.80,
            f"Rotor tip did not sweep a realistic arc: rest={rest_tip}, spun={spun_tip}",
        )

    with ctx.pose({cap_joint: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="cap_quarter_yaw_no_overlap")
        ctx.fail_if_isolated_parts(name="cap_quarter_yaw_no_floating")
        yawed_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="top_tip_panel"))
        assert yawed_tip is not None
        ctx.check(
            "cap_yaw_reorients_rotor",
            abs(yawed_tip[0]) > 1.50 and abs(yawed_tip[1]) < 0.35,
            f"Cap yaw did not swing the rotor plane around the tower: yawed={yawed_tip}",
        )

    with ctx.pose(
        {
            door_joint: door_limits.lower,
            cap_joint: math.pi / 2.0,
            rotor_joint: math.pi / 4.0,
            lever_joint: lever_limits.lower,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_operating_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_operating_pose_no_floating")
        ctx.expect_contact(
            rotor,
            cap,
            elem_a="hub_back_plate",
            elem_b="thrust_bearing",
            name="combined_pose_rotor_support_contact",
        )
        ctx.expect_contact(
            tower,
            door,
            elem_a="door_hinge_pin",
            elem_b="door_hinge_barrel",
            name="combined_pose_door_hinge_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
