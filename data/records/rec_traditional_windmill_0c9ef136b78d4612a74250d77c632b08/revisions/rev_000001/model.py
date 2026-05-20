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
    rounded_rect_profile,
    section_loft,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _beam_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return (0.0, pitch, yaw)


def _rod_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((_distance(a, b), width, depth)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_beam_rpy(a, b)),
        material=material,
        name=name,
    )


def _add_rod(
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
        origin=Origin(xyz=_midpoint(a, b), rpy=_rod_rpy(a, b)),
        material=material,
        name=name,
    )


def _vec_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _vec_add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _tower_shell_mesh():
    outer_profile = [
        (3.20, 0.00),
        (3.12, 0.85),
        (2.98, 2.40),
        (2.78, 4.90),
        (2.48, 7.10),
        (2.28, 8.00),
    ]
    inner_profile = [
        (2.72, 0.22),
        (2.66, 0.95),
        (2.50, 2.50),
        (2.30, 4.95),
        (2.03, 7.05),
        (1.88, 7.72),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ),
        "tower_shell",
    )


def _cap_section(width: float, height: float, radius: float, x: float):
    return [(x, y, z + 0.98) for z, y in rounded_rect_profile(height, width, radius)]


def _cap_mesh():
    sections = [
        _cap_section(2.10, 1.05, 0.18, -1.95),
        _cap_section(2.75, 1.55, 0.26, -1.15),
        _cap_section(3.28, 1.92, 0.34, 0.00),
        _cap_section(2.88, 1.68, 0.28, 1.05),
        _cap_section(2.05, 1.10, 0.18, 1.88),
    ]
    return mesh_from_geometry(section_loft(sections), "cap_shell_body")


def _blade_point(
    theta: float, radius: float, tangent_offset: float = 0.0, x: float = 0.16
) -> tuple[float, float, float]:
    radial = (0.0, math.sin(theta), math.cos(theta))
    tangent = (0.0, math.cos(theta), -math.sin(theta))
    return _vec_add(
        (x, 0.0, 0.0),
        _vec_add(_vec_scale(radial, radius), _vec_scale(tangent, tangent_offset)),
    )


def _add_sail(part, *, theta: float, prefix: str, wood_material) -> None:
    outer_span = 5.60
    outer_half_width = 0.42
    inner_half_width = 0.22

    _add_beam(
        part,
        _blade_point(theta, 0.28, 0.0),
        _blade_point(theta, 1.18, 0.0),
        width=0.26,
        depth=0.18,
        material=wood_material,
        name=f"{prefix}_root_stock",
    )
    _add_beam(
        part,
        _blade_point(theta, 0.38, 0.0),
        _blade_point(theta, outer_span * 0.98, 0.0),
        width=0.15,
        depth=0.11,
        material=wood_material,
        name=f"{prefix}_center_spar",
    )
    _add_beam(
        part,
        _blade_point(theta, 0.62, outer_half_width * 0.82),
        _blade_point(theta, outer_span, outer_half_width),
        width=0.12,
        depth=0.09,
        material=wood_material,
        name=f"{prefix}_outer_leading",
    )
    _add_beam(
        part,
        _blade_point(theta, 0.62, -outer_half_width * 0.82),
        _blade_point(theta, outer_span, -outer_half_width),
        width=0.12,
        depth=0.09,
        material=wood_material,
        name=f"{prefix}_outer_trailing",
    )
    _add_beam(
        part,
        _blade_point(theta, 1.15, inner_half_width),
        _blade_point(theta, outer_span * 0.93, inner_half_width * 1.45),
        width=0.05,
        depth=0.04,
        material=wood_material,
        name=f"{prefix}_inner_leading_lath",
    )
    _add_beam(
        part,
        _blade_point(theta, 1.15, -inner_half_width),
        _blade_point(theta, outer_span * 0.93, -inner_half_width * 1.45),
        width=0.05,
        depth=0.04,
        material=wood_material,
        name=f"{prefix}_inner_trailing_lath",
    )

    rung_radii = [1.10, 1.75, 2.40, 3.05, 3.70, 4.35, 5.00]
    for idx, radius in enumerate(rung_radii):
        width = outer_half_width * (0.82 + 0.18 * radius / outer_span)
        _add_beam(
            part,
            _blade_point(theta, radius, -width),
            _blade_point(theta, radius, width),
            width=0.08 if idx < 2 else 0.06,
            depth=0.045,
            material=wood_material,
            name=f"{prefix}_rung_{idx:02d}",
        )

    brace_pairs = list(zip(rung_radii[:-1], rung_radii[1:]))
    for idx, (r0, r1) in enumerate(brace_pairs):
        sign = 1.0 if idx % 2 == 0 else -1.0
        _add_beam(
            part,
            _blade_point(theta, r0, sign * outer_half_width * 0.90),
            _blade_point(theta, r1, -sign * outer_half_width * 0.78),
            width=0.05,
            depth=0.04,
            material=wood_material,
            name=f"{prefix}_brace_{idx:02d}",
        )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.70, 0.68, 0.63, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.52, 0.50, 0.47, 1.0))
    timber = model.material("timber", rgba=(0.41, 0.27, 0.15, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.33, 0.24, 0.16, 1.0))
    shingle = model.material("shingle", rgba=(0.22, 0.17, 0.14, 1.0))
    black_iron = model.material("black_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    sail_wood = model.material("sail_wood", rgba=(0.70, 0.61, 0.42, 1.0))
    off_white = model.material("off_white", rgba=(0.88, 0.85, 0.76, 1.0))

    tower_base = model.part("tower_base")
    tower_base.visual(_tower_shell_mesh(), material=stone, name="tower_shell")
    tower_base.visual(
        Cylinder(radius=3.32, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=dark_stone,
        name="plinth_ring",
    )
    tower_base.visual(
        Box((0.34, 1.06, 2.05)),
        origin=Origin(xyz=(3.02, 0.0, 1.03)),
        material=timber,
        name="front_door",
    )
    tower_base.visual(
        Box((0.18, 0.62, 0.90)),
        origin=Origin(xyz=(2.76, 0.0, 3.55)),
        material=weathered_wood,
        name="mid_window",
    )
    tower_base.visual(
        Box((0.14, 0.54, 0.76)),
        origin=Origin(xyz=(2.53, 0.0, 5.75)),
        material=weathered_wood,
        name="upper_window",
    )
    tower_base.visual(
        Cylinder(radius=2.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 7.93)),
        material=dark_stone,
        name="tower_top_ring",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((6.70, 6.70, 8.10)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.05)),
    )

    bearing_module = model.part("bearing_module")
    bearing_module.visual(
        Cylinder(radius=2.42, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=timber,
        name="bottom_curb",
    )
    bearing_module.visual(
        Cylinder(radius=2.18, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=weathered_wood,
        name="upper_curb",
    )
    bearing_module.visual(
        Cylinder(radius=2.28, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=black_iron,
        name="top_track",
    )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        x = math.cos(angle) * 1.45
        y = math.sin(angle) * 1.45
        bearing_module.visual(
            Box((0.46, 0.26, 0.20)),
            origin=Origin(xyz=(x, y, 0.30), rpy=(0.0, 0.0, angle)),
            material=timber,
            name=f"bearing_block_{int(angle / (math.pi / 2.0))}",
        )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        x = math.cos(angle) * 1.82
        y = math.sin(angle) * 1.82
        bearing_module.visual(
            Cylinder(radius=0.10, length=0.34),
            origin=Origin(xyz=(x, y, 0.27)),
            material=black_iron,
            name=f"bearing_post_{int(angle / (math.pi / 2.0))}",
        )
    bearing_module.inertial = Inertial.from_geometry(
        Cylinder(radius=2.45, length=0.56),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    cap_shell = model.part("cap_shell")
    cap_shell.visual(
        Cylinder(radius=2.06, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=weathered_wood,
        name="cap_base_ring",
    )
    cap_shell.visual(_cap_mesh(), material=shingle, name="cap_body")
    cap_shell.visual(
        Box((0.18, 0.86, 0.96)),
        origin=Origin(xyz=(-1.72, 0.0, 0.72)),
        material=weathered_wood,
        name="rear_service_hatch",
    )
    cap_shell.visual(
        Box((0.14, 0.52, 0.62)),
        origin=Origin(xyz=(0.10, 1.55, 0.86)),
        material=weathered_wood,
        name="port_cap_window",
    )
    cap_shell.visual(
        Box((0.14, 0.52, 0.62)),
        origin=Origin(xyz=(0.10, -1.55, 0.86)),
        material=weathered_wood,
        name="starboard_cap_window",
    )
    cap_shell.inertial = Inertial.from_geometry(
        Box((4.20, 3.60, 2.10)),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Box((0.16, 1.16, 1.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=weathered_wood,
        name="rear_mount_plate",
    )
    head_frame.visual(
        Box((0.62, 0.20, 0.78)),
        origin=Origin(xyz=(0.31, 0.42, 0.0)),
        material=weathered_wood,
        name="port_cheek",
    )
    head_frame.visual(
        Box((0.62, 0.20, 0.78)),
        origin=Origin(xyz=(0.31, -0.42, 0.0)),
        material=weathered_wood,
        name="starboard_cheek",
    )
    head_frame.visual(
        Box((0.42, 0.96, 0.16)),
        origin=Origin(xyz=(0.18, 0.0, 0.35)),
        material=weathered_wood,
        name="top_bridge",
    )
    head_frame.visual(
        Box((0.30, 0.68, 0.12)),
        origin=Origin(xyz=(0.14, 0.0, -0.30)),
        material=weathered_wood,
        name="bottom_bridge",
    )
    head_frame.visual(
        Box((0.22, 0.84, 0.14)),
        origin=Origin(xyz=(0.64, 0.0, 0.0)),
        material=weathered_wood,
        name="front_cheek_bridge",
    )
    head_frame.visual(
        Cylinder(radius=0.24, length=0.16),
        origin=Origin(xyz=(0.77, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_iron,
        name="front_bearing_collar",
    )
    _add_rod(
        head_frame,
        (0.08, 0.46, 0.32),
        (0.56, 0.14, 0.14),
        radius=0.04,
        material=black_iron,
        name="port_upper_strut",
    )
    _add_rod(
        head_frame,
        (0.08, -0.46, 0.32),
        (0.56, -0.14, 0.14),
        radius=0.04,
        material=black_iron,
        name="starboard_upper_strut",
    )
    _add_rod(
        head_frame,
        (0.10, 0.36, -0.22),
        (0.50, 0.12, -0.08),
        radius=0.035,
        material=black_iron,
        name="port_lower_strut",
    )
    _add_rod(
        head_frame,
        (0.10, -0.36, -0.22),
        (0.50, -0.12, -0.08),
        radius=0.035,
        material=black_iron,
        name="starboard_lower_strut",
    )
    head_frame.inertial = Inertial.from_geometry(
        Box((0.84, 1.20, 1.10)),
        mass=900.0,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.56, length=0.24),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_iron,
        name="hub_disk",
    )
    sail_hub.visual(
        Cylinder(radius=0.20, length=0.62),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_iron,
        name="hub_barrel",
    )
    sail_hub.visual(
        Cylinder(radius=0.13, length=0.20),
        origin=Origin(xyz=(0.71, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_iron,
        name="nose_collar",
    )
    sail_hub.visual(
        Cylinder(radius=0.06, length=0.16),
        origin=Origin(xyz=(0.88, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="nose_tip",
    )
    _add_sail(sail_hub, theta=0.0, prefix="north_sail", wood_material=sail_wood)
    _add_sail(
        sail_hub,
        theta=math.pi / 2.0,
        prefix="east_sail",
        wood_material=sail_wood,
    )
    _add_sail(
        sail_hub,
        theta=math.pi,
        prefix="south_sail",
        wood_material=sail_wood,
    )
    _add_sail(
        sail_hub,
        theta=3.0 * math.pi / 2.0,
        prefix="west_sail",
        wood_material=sail_wood,
    )
    sail_hub.inertial = Inertial.from_geometry(
        Box((1.10, 11.40, 11.40)),
        mass=1200.0,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=tower_base,
        child=bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 8.00)),
    )
    model.articulation(
        "bearing_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bearing_module,
        child=cap_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.40),
    )
    model.articulation(
        "cap_to_head_frame",
        ArticulationType.FIXED,
        parent=cap_shell,
        child=head_frame,
        origin=Origin(xyz=(1.96, 0.0, 1.05)),
    )
    model.articulation(
        "head_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=head_frame,
        child=sail_hub,
        origin=Origin(xyz=(0.85, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=2.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower_base = object_model.get_part("tower_base")
    bearing_module = object_model.get_part("bearing_module")
    cap_shell = object_model.get_part("cap_shell")
    head_frame = object_model.get_part("head_frame")
    sail_hub = object_model.get_part("sail_hub")

    cap_rotation = object_model.get_articulation("bearing_to_cap")
    hub_rotation = object_model.get_articulation("head_to_sail_hub")

    ctx.expect_gap(
        bearing_module,
        tower_base,
        axis="z",
        positive_elem="bottom_curb",
        negative_elem="tower_shell",
        max_gap=0.03,
        max_penetration=0.0,
        name="bearing module sits on the tower crown",
    )
    ctx.expect_gap(
        cap_shell,
        bearing_module,
        axis="z",
        positive_elem="cap_base_ring",
        negative_elem="top_track",
        max_gap=0.01,
        max_penetration=1e-5,
        name="cap sits on the bearing track",
    )
    ctx.expect_origin_gap(
        head_frame,
        cap_shell,
        axis="x",
        min_gap=1.5,
        name="head frame projects clearly in front of the cap",
    )

    hub_rest = ctx.part_world_position(sail_hub)
    with ctx.pose({cap_rotation: math.pi / 2.0}):
        hub_turned = ctx.part_world_position(sail_hub)
    ctx.check(
        "cap rotation slews the front hub around the tower axis",
        hub_rest is not None
        and hub_turned is not None
        and hub_turned[1] > hub_rest[1] + 2.0
        and hub_turned[0] < hub_rest[0] - 2.0,
        details=f"rest={hub_rest}, turned={hub_turned}",
    )

    north_rest_aabb = ctx.part_element_world_aabb(sail_hub, elem="north_sail_outer_leading")
    with ctx.pose({hub_rotation: math.pi / 2.0}):
        north_quarter_aabb = ctx.part_element_world_aabb(
            sail_hub, elem="north_sail_outer_leading"
        )

    north_rest_center = _aabb_center(north_rest_aabb)
    north_quarter_center = _aabb_center(north_quarter_aabb)
    ctx.check(
        "hub spin rotates a named sail around the windshaft axis",
        north_rest_center is not None
        and north_quarter_center is not None
        and north_quarter_center[1] < north_rest_center[1] - 2.0
        and north_quarter_center[2] < north_rest_center[2] - 2.0,
        details=f"rest_center={north_rest_center}, quarter_center={north_quarter_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
