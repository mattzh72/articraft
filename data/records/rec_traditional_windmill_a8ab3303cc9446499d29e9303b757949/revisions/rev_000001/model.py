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
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_z_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _extend_segment(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    start_pad: float,
    end_pad: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    length = _distance(a, b)
    if length <= 1e-9:
        return a, b
    ux = (b[0] - a[0]) / length
    uy = (b[1] - a[1]) / length
    uz = (b[2] - a[2]) / length
    return (
        (a[0] - ux * start_pad, a[1] - uy * start_pad, a[2] - uz * start_pad),
        (b[0] + ux * end_pad, b[1] + uy * end_pad, b[2] + uz * end_pad),
    )


def _add_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size_xy: tuple[float, float],
    material,
    name: str | None = None,
    start_pad: float = 0.0,
    end_pad: float = 0.0,
) -> None:
    a_ext, b_ext = _extend_segment(a, b, start_pad, end_pad)
    part.visual(
        Box((size_xy[0], size_xy[1], _distance(a_ext, b_ext))),
        origin=Origin(xyz=_midpoint(a_ext, b_ext), rpy=_rpy_for_z_member(a_ext, b_ext)),
        material=material,
        name=name,
    )


def _rotate_x(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x, y * ca - z * sa, y * sa + z * ca)


def _aabb_center(aabb_data):
    if aabb_data is None:
        return None
    return tuple(
        (aabb_data[0][axis] + aabb_data[1][axis]) * 0.5
        for axis in range(3)
    )


def _cap_section(x: float, width: float, wall_height: float, peak_height: float) -> LoftSection:
    half = width * 0.5
    shoulder = width * 0.34
    upper = width * 0.18
    return LoftSection(
        points=(
            (x, -half, 0.12),
            (x, -half, wall_height),
            (x, -shoulder, wall_height + 0.26),
            (x, -upper, peak_height - 0.10),
            (x, upper, peak_height - 0.10),
            (x, shoulder, wall_height + 0.26),
            (x, half, wall_height),
            (x, half, 0.12),
        )
    )


def _build_cap_shell():
    return mesh_from_geometry(
        section_loft(
            SectionLoftSpec(
                sections=(
                    _cap_section(-1.30, 2.70, 0.64, 1.62),
                    _cap_section(-0.55, 3.25, 0.82, 2.05),
                    _cap_section(0.45, 3.45, 0.92, 2.28),
                    _cap_section(1.20, 2.75, 0.84, 2.02),
                    _cap_section(1.90, 1.50, 0.52, 1.28),
                ),
                cap=True,
                solid=True,
                repair="mesh",
            )
        ),
        "cap_shell",
    )


def _build_tower_shell():
    profile = [
        (0.0, 0.0),
        (3.55, 0.0),
        (3.42, 0.70),
        (3.05, 4.50),
        (2.62, 9.20),
        (2.24, 13.55),
        (2.10, 13.95),
        (0.0, 13.95),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64), "tower_shell")


def _add_sail(
    part,
    *,
    index: int,
    angle: float,
    material,
) -> None:
    stock_root = (0.00, 0.00, 0.00)
    stock_tip = (0.00, 8.90, 0.00)
    panel_inner = 1.30
    panel_outer = 8.45
    inner_lead = (0.00, panel_inner, 0.78)
    inner_trail = (0.00, panel_inner, -0.72)
    outer_lead = (0.00, panel_outer, 0.92)
    outer_trail = (0.00, panel_outer, -0.58)

    stock_root_r = _rotate_x(stock_root, angle)
    stock_tip_r = _rotate_x(stock_tip, angle)
    inner_lead_r = _rotate_x(inner_lead, angle)
    inner_trail_r = _rotate_x(inner_trail, angle)
    outer_lead_r = _rotate_x(outer_lead, angle)
    outer_trail_r = _rotate_x(outer_trail, angle)

    _add_beam(
        part,
        stock_root_r,
        stock_tip_r,
        size_xy=(0.16, 0.16),
        material=material,
        name=f"blade_{index}_stock",
        end_pad=0.04,
    )
    _add_beam(
        part,
        inner_lead_r,
        outer_lead_r,
        size_xy=(0.11, 0.07),
        material=material,
        name=f"blade_{index}_leading_rail",
        start_pad=0.04,
        end_pad=0.04,
    )
    _add_beam(
        part,
        inner_trail_r,
        outer_trail_r,
        size_xy=(0.11, 0.07),
        material=material,
        name=f"blade_{index}_trailing_rail",
        start_pad=0.04,
        end_pad=0.04,
    )
    _add_beam(
        part,
        inner_lead_r,
        inner_trail_r,
        size_xy=(0.09, 0.06),
        material=material,
        name=f"blade_{index}_inner_frame",
        start_pad=0.03,
        end_pad=0.03,
    )
    _add_beam(
        part,
        outer_lead_r,
        outer_trail_r,
        size_xy=(0.09, 0.06),
        material=material,
        name=f"blade_{index}_outer_frame",
        start_pad=0.03,
        end_pad=0.03,
    )
    _add_beam(
        part,
        _rotate_x((0.00, 0.70, 0.00), angle),
        outer_trail_r,
        size_xy=(0.08, 0.06),
        material=material,
        name=f"blade_{index}_brace_lower",
        start_pad=0.04,
        end_pad=0.03,
    )
    _add_beam(
        part,
        _rotate_x((0.00, 0.82, 0.06), angle),
        outer_lead_r,
        size_xy=(0.08, 0.06),
        material=material,
        name=f"blade_{index}_brace_upper",
        start_pad=0.04,
        end_pad=0.03,
    )

    slat_count = 8
    for slat_index in range(slat_count):
        t = slat_index / (slat_count - 1)
        y = panel_inner + (panel_outer - panel_inner) * t
        z_lead = inner_lead[2] + (outer_lead[2] - inner_lead[2]) * t
        z_trail = inner_trail[2] + (outer_trail[2] - inner_trail[2]) * t
        _add_beam(
            part,
            _rotate_x((0.00, y, z_lead), angle),
            _rotate_x((0.00, y, z_trail), angle),
            size_xy=(0.05, 0.03),
            material=material,
            name=f"blade_{index}_slat_{slat_index}",
            start_pad=0.02,
            end_pad=0.02,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.76, 0.73, 0.68, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.58, 0.55, 0.51, 1.0))
    timber = model.material("timber", rgba=(0.42, 0.30, 0.19, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.63, 0.57, 0.47, 1.0))
    shingle = model.material("shingle", rgba=(0.20, 0.16, 0.13, 1.0))
    iron = model.material("iron", rgba=(0.19, 0.20, 0.22, 1.0))

    tower_shell_mesh = _build_tower_shell()
    cap_shell_mesh = _build_cap_shell()

    tower_body = model.part("tower_body")
    tower_body.visual(tower_shell_mesh, material=stone, name="tower_shell")
    tower_body.visual(
        Cylinder(radius=2.16, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 14.00)),
        material=dark_stone,
        name="tower_curb",
    )
    tower_body.visual(
        Cylinder(radius=3.64, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_stone,
        name="foundation_ring",
    )
    tower_body.visual(
        Box((0.28, 1.12, 2.25)),
        origin=Origin(xyz=(3.18, 0.0, 1.18)),
        material=timber,
        name="front_door",
    )
    tower_body.visual(
        Box((0.18, 0.92, 1.95)),
        origin=Origin(xyz=(3.30, 0.0, 1.18)),
        material=iron,
        name="door_frame",
    )
    tower_body.visual(
        Box((0.24, 0.78, 0.74)),
        origin=Origin(xyz=(2.54, 0.0, 9.90)),
        material=weathered_wood,
        name="upper_window_shutter",
    )
    tower_body.inertial = Inertial.from_geometry(
        Cylinder(radius=3.45, length=14.15),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 7.075)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=2.20, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=timber,
        name="cap_turntable",
    )
    cap.visual(
        Box((2.10, 2.20, 0.36)),
        origin=Origin(xyz=(0.35, 0.0, 0.26)),
        material=timber,
        name="cap_base_plinth",
    )
    cap.visual(
        cap_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=shingle,
        name="cap_shell",
    )
    cap.visual(
        Box((1.85, 1.05, 0.78)),
        origin=Origin(xyz=(1.55, 0.0, 0.92)),
        material=timber,
        name="front_cheek_block",
    )
    cap.visual(
        Box((1.55, 0.86, 0.56)),
        origin=Origin(xyz=(2.70, 0.0, 1.00)),
        material=timber,
        name="windshaft_box",
    )
    cap.visual(
        Cylinder(radius=0.28, length=1.42),
        origin=Origin(xyz=(3.08, 0.0, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_bearing",
    )
    cap.visual(
        Cylinder(radius=0.17, length=0.28),
        origin=Origin(xyz=(3.79, 0.0, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="shaft_stub",
    )
    cap.visual(
        Cylinder(radius=0.48, length=0.16),
        origin=Origin(xyz=(-1.24, 0.0, 0.88), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=timber,
        name="rear_cap_post",
    )
    cap.inertial = Inertial.from_geometry(
        Box((3.90, 3.50, 2.45)),
        mass=2100.0,
        origin=Origin(xyz=(0.20, 0.0, 1.12)),
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.34, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_barrel",
    )
    sail_hub.visual(
        Cylinder(radius=0.24, length=0.42),
        origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="rear_boss",
    )
    sail_hub.visual(
        Cylinder(radius=0.27, length=0.10),
        origin=Origin(xyz=(0.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_plate",
    )
    for blade_index, blade_angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_sail(
            sail_hub,
            index=blade_index,
            angle=blade_angle,
            material=weathered_wood,
        )
    sail_hub.inertial = Inertial.from_geometry(
        Box((0.70, 18.10, 18.10)),
        mass=1600.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "cap_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 14.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=7000.0, velocity=0.30),
    )
    model.articulation(
        "sail_rotation",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(4.30, 0.0, 1.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower_body = object_model.get_part("tower_body")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    cap_rotation = object_model.get_articulation("cap_rotation")
    sail_rotation = object_model.get_articulation("sail_rotation")

    with ctx.pose({cap_rotation: 0.0, sail_rotation: 0.0}):
        ctx.expect_gap(
            cap,
            tower_body,
            axis="z",
            positive_elem="cap_turntable",
            negative_elem="tower_curb",
            max_gap=0.001,
            max_penetration=0.0,
            name="cap turntable sits on the tower curb",
        )
        ctx.expect_gap(
            sail_hub,
            cap,
            axis="x",
            positive_elem="rear_boss",
            negative_elem="shaft_stub",
            max_gap=0.001,
            max_penetration=1e-5,
            name="hub seats against the windshaft support",
        )
        ctx.expect_gap(
            sail_hub,
            tower_body,
            axis="x",
            positive_elem="rear_boss",
            negative_elem="tower_shell",
            min_gap=0.30,
            name="hub stays clearly in front of the tower face",
        )

    rest_hub_position = ctx.part_world_position(sail_hub)
    with ctx.pose({cap_rotation: math.pi / 2.0}):
        turned_hub_position = ctx.part_world_position(sail_hub)

    ctx.check(
        "cap rotation swings the head around the tower axis",
        rest_hub_position is not None
        and turned_hub_position is not None
        and abs(rest_hub_position[0] - 4.30) < 0.06
        and abs(rest_hub_position[1]) < 0.06
        and abs(turned_hub_position[0]) < 0.06
        and abs(turned_hub_position[1] - 4.30) < 0.06
        and abs(turned_hub_position[2] - rest_hub_position[2]) < 1e-6,
        details=f"rest={rest_hub_position}, turned={turned_hub_position}",
    )

    blade_rest_center = _aabb_center(
        ctx.part_element_world_aabb(sail_hub, elem="blade_0_outer_frame")
    )
    with ctx.pose({sail_rotation: math.pi / 2.0}):
        blade_spun_center = _aabb_center(
            ctx.part_element_world_aabb(sail_hub, elem="blade_0_outer_frame")
        )

    ctx.check(
        "sail hub rotation turns a lattice blade through the wind plane",
        blade_rest_center is not None
        and blade_spun_center is not None
        and blade_rest_center[1] > 7.5
        and blade_spun_center[2] > blade_rest_center[2] + 7.5,
        details=f"rest={blade_rest_center}, spun={blade_spun_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
