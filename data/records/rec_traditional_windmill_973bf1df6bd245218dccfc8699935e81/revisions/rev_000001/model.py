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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_axis_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
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
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis_member(a, b)),
        material=material,
        name=name,
    )


def _annulus_mesh(outer_radius: float, inner_radius: float, height: float, *, segments: int = 72):
    half = height * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    center_z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, center_z + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _add_bolt_circle_z(
    part,
    *,
    bolt_circle_radius: float,
    bolt_radius: float,
    shank_length: float,
    z_center: float,
    count: int,
    material,
) -> None:
    for index in range(count):
        angle = math.tau * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=shank_length),
            origin=Origin(
                xyz=(
                    bolt_circle_radius * math.cos(angle),
                    bolt_circle_radius * math.sin(angle),
                    z_center,
                )
            ),
            material=material,
        )


def _add_service_panel(
    part,
    *,
    radius: float,
    angle: float,
    z_center: float,
    size: tuple[float, float, float],
    material,
    frame_material,
    name: str,
) -> None:
    x_pos = radius * math.cos(angle)
    y_pos = radius * math.sin(angle)
    yaw = angle
    part.visual(
        Box(size),
        origin=Origin(xyz=(x_pos, y_pos, z_center), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )
    frame_offset = size[0] * 0.25
    frame_depth = max(size[0] * 0.55, 0.025)
    frame_width = size[1] + 0.08
    frame_height = size[2] + 0.08
    part.visual(
        Box((frame_depth, frame_width, 0.04)),
        origin=Origin(
            xyz=(
                (radius - frame_offset) * math.cos(angle),
                (radius - frame_offset) * math.sin(angle),
                z_center + (size[2] - 0.04) * 0.5,
            ),
            rpy=(0.0, 0.0, yaw),
        ),
        material=frame_material,
    )
    part.visual(
        Box((frame_depth, frame_width, 0.04)),
        origin=Origin(
            xyz=(
                (radius - frame_offset) * math.cos(angle),
                (radius - frame_offset) * math.sin(angle),
                z_center - (size[2] - 0.04) * 0.5,
            ),
            rpy=(0.0, 0.0, yaw),
        ),
        material=frame_material,
    )
    part.visual(
        Box((frame_depth, 0.04, frame_height)),
        origin=Origin(
            xyz=(
                (radius - frame_offset) * math.cos(angle) - 0.5 * frame_width * math.sin(angle),
                (radius - frame_offset) * math.sin(angle) + 0.5 * frame_width * math.cos(angle),
                z_center,
            ),
            rpy=(0.0, 0.0, yaw),
        ),
        material=frame_material,
    )
    part.visual(
        Box((frame_depth, 0.04, frame_height)),
        origin=Origin(
            xyz=(
                (radius - frame_offset) * math.cos(angle) + 0.5 * frame_width * math.sin(angle),
                (radius - frame_offset) * math.sin(angle) - 0.5 * frame_width * math.cos(angle),
                z_center,
            ),
            rpy=(0.0, 0.0, yaw),
        ),
        material=frame_material,
    )


def _blade_offset(t: float) -> float:
    return 0.42 - 0.14 * t


def _panel_point(axis: str, span: float, offset: float, x_pos: float) -> tuple[float, float, float]:
    if axis == "z":
        return (x_pos, offset, span)
    return (x_pos, span, offset)


def _add_sail_panel(part, *, axis: str, sign: float, timber, strap, x_pos: float) -> None:
    root = 0.92 * sign
    tip = 4.95 * sign
    root_w = _blade_offset(0.0)
    tip_w = _blade_offset(1.0)

    _add_box_member(
        part,
        _panel_point(axis, root, -(root_w + 0.04), x_pos),
        _panel_point(axis, tip, -(tip_w + 0.02), x_pos),
        width=0.07,
        depth=0.05,
        material=timber,
    )
    _add_box_member(
        part,
        _panel_point(axis, root, root_w + 0.04, x_pos),
        _panel_point(axis, tip, tip_w + 0.02, x_pos),
        width=0.07,
        depth=0.05,
        material=timber,
    )
    _add_box_member(
        part,
        _panel_point(axis, root - 0.08 * sign, -(root_w + 0.05), x_pos),
        _panel_point(axis, root - 0.08 * sign, root_w + 0.05, x_pos),
        width=0.06,
        depth=0.045,
        material=timber,
    )
    _add_box_member(
        part,
        _panel_point(axis, tip - 0.02 * sign, -(tip_w + 0.025), x_pos),
        _panel_point(axis, tip - 0.02 * sign, tip_w + 0.025, x_pos),
        width=0.06,
        depth=0.045,
        material=timber,
    )

    for index in range(10):
        t = index / 9.0
        span = sign * (1.10 + 3.55 * t)
        span_w = _blade_offset(t)
        _add_box_member(
            part,
            _panel_point(axis, span, -(span_w + 0.03), x_pos),
            _panel_point(axis, span, span_w + 0.03, x_pos),
            width=0.035,
            depth=0.03,
            material=strap,
        )

    _add_box_member(
        part,
        _panel_point(axis, root + 0.18 * sign, -(root_w + 0.03), x_pos),
        _panel_point(axis, sign * 3.15, tip_w + 0.06, x_pos),
        width=0.03,
        depth=0.03,
        material=strap,
    )
    _add_box_member(
        part,
        _panel_point(axis, root + 0.18 * sign, root_w + 0.03, x_pos),
        _panel_point(axis, sign * 3.15, -(tip_w + 0.06), x_pos),
        width=0.03,
        depth=0.03,
        material=strap,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_retrofit_windmill")

    lime_render = model.material("lime_render", rgba=(0.92, 0.90, 0.84, 1.0))
    weathered_stone = model.material("weathered_stone", rgba=(0.67, 0.63, 0.57, 1.0))
    oak = model.material("oak", rgba=(0.45, 0.31, 0.18, 1.0))
    dark_tar = model.material("dark_tar", rgba=(0.19, 0.18, 0.17, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.26, 0.30, 0.32, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.66, 0.68, 1.0))
    hatch_red = model.material("hatch_red", rgba=(0.48, 0.15, 0.12, 1.0))
    sail_canvas = model.material("sail_canvas", rgba=(0.78, 0.76, 0.68, 1.0))

    tower = model.part("tower")
    tower_shell = LatheGeometry.from_shell_profiles(
        [
            (2.15, 0.0),
            (2.08, 0.35),
            (2.00, 1.50),
            (1.82, 3.80),
            (1.56, 5.90),
            (1.38, 7.00),
        ],
        [
            (1.81, 0.0),
            (1.75, 0.35),
            (1.70, 1.50),
            (1.55, 3.80),
            (1.33, 5.90),
            (1.16, 7.00),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    tower.visual(_save_mesh("tower_shell", tower_shell), material=lime_render, name="tower_shell")
    tower.visual(
        Cylinder(radius=2.24, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=weathered_stone,
        name="base_plinth",
    )
    tower.visual(
        _save_mesh("tower_curb_ring", _annulus_mesh(1.48, 1.02, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 7.09)),
        material=painted_steel,
        name="tower_curb_ring",
    )
    tower.visual(
        Cylinder(radius=0.14, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 7.07)),
        material=galvanized,
        name="yaw_pintle",
    )
    tower.visual(
        Box((1.70, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 7.00)),
        material=painted_steel,
        name="top_cross_tie_x",
    )
    tower.visual(
        Box((0.20, 1.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 7.00)),
        material=painted_steel,
        name="top_cross_tie_y",
    )
    tower.visual(
        Box((2.28, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 7.08)),
        material=painted_steel,
        name="upper_adapter_beam_x",
    )
    tower.visual(
        Box((0.16, 2.28, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 7.08)),
        material=painted_steel,
        name="upper_adapter_beam_y",
    )
    for rib_angle in (math.pi * 0.25, math.pi * 0.75, math.pi * 1.25, math.pi * 1.75):
        tower.visual(
            Box((0.10, 0.18, 6.15)),
            origin=Origin(
                xyz=(1.93 * math.cos(rib_angle), 1.93 * math.sin(rib_angle), 3.35),
                rpy=(0.0, 0.0, rib_angle),
            ),
            material=weathered_stone,
        )
    _add_service_panel(
        tower,
        radius=2.08,
        angle=0.0,
        z_center=0.98,
        size=(0.07, 0.78, 1.60),
        material=hatch_red,
        frame_material=oak,
        name="main_service_door",
    )
    _add_service_panel(
        tower,
        radius=1.86,
        angle=math.radians(108),
        z_center=3.72,
        size=(0.06, 0.76, 1.08),
        material=dark_tar,
        frame_material=oak,
        name="mid_service_hatch",
    )
    _add_service_panel(
        tower,
        radius=1.62,
        angle=math.radians(-52),
        z_center=5.88,
        size=(0.05, 0.52, 0.66),
        material=dark_tar,
        frame_material=galvanized,
        name="upper_inspection_hatch",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.24, length=7.30),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 3.65)),
    )

    cap = model.part("cap")
    cap.visual(
        _save_mesh("cap_curb_ring", _annulus_mesh(1.48, 1.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_steel,
        name="cap_curb_ring",
    )
    cap.visual(
        _save_mesh("yaw_collar", _annulus_mesh(0.25, 0.155, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=galvanized,
        name="yaw_collar",
    )
    for sx, sy, size in [
        (0.64, 0.0, (0.84, 0.12, 0.10)),
        (-0.64, 0.0, (0.84, 0.12, 0.10)),
        (0.0, 0.64, (0.12, 0.84, 0.10)),
        (0.0, -0.64, (0.12, 0.84, 0.10)),
    ]:
        cap.visual(Box(size), origin=Origin(xyz=(sx, sy, 0.05)), material=painted_steel)
    cap.visual(
        Box((2.06, 1.58, 0.14)),
        origin=Origin(xyz=(0.44, 0.0, 0.17)),
        material=oak,
        name="cap_floor",
    )

    cap.visual(
        Box((1.66, 0.18, 0.18)),
        origin=Origin(xyz=(-0.10, -0.74, 0.23)),
        material=oak,
        name="left_base_beam",
    )
    cap.visual(
        Box((1.66, 0.18, 0.18)),
        origin=Origin(xyz=(-0.10, 0.74, 0.23)),
        material=oak,
        name="right_base_beam",
    )
    cap.visual(Box((0.18, 1.42, 0.16)), origin=Origin(xyz=(0.68, 0.0, 0.22)), material=oak)
    cap.visual(Box((0.18, 1.42, 0.16)), origin=Origin(xyz=(-0.88, 0.0, 0.22)), material=oak)
    cap.visual(Box((1.18, 0.06, 0.58)), origin=Origin(xyz=(-0.30, -0.75, 0.78)), material=oak)
    cap.visual(Box((1.18, 0.06, 0.58)), origin=Origin(xyz=(-0.30, 0.75, 0.78)), material=oak)
    cap.visual(Box((0.08, 0.74, 0.86)), origin=Origin(xyz=(-1.02, 0.0, 0.84)), material=dark_tar)
    cap.visual(Box((0.14, 1.44, 0.84)), origin=Origin(xyz=(-1.02, 0.0, 0.76)), material=oak)
    cap.visual(
        Box((0.88, 1.48, 0.12)),
        origin=Origin(xyz=(-0.72, 0.0, 0.28)),
        material=oak,
        name="rear_deck_beam",
    )
    cap.visual(
        Box((0.12, 0.12, 0.76)),
        origin=Origin(xyz=(-0.74, -0.74, 0.60)),
        material=oak,
        name="left_rear_post",
    )
    cap.visual(
        Box((0.12, 0.12, 0.76)),
        origin=Origin(xyz=(-0.74, 0.74, 0.60)),
        material=oak,
        name="right_rear_post",
    )

    hood_geom = section_loft(
        [
            _yz_section(-1.08, 1.70, 0.46, 0.12, 1.15),
            _yz_section(-0.58, 1.96, 0.82, 0.18, 1.33),
            _yz_section(-0.06, 2.02, 0.92, 0.18, 1.35),
            _yz_section(0.14, 1.55, 0.70, 0.14, 1.24),
        ]
    )
    cap.visual(_save_mesh("cap_hood", hood_geom), material=dark_tar, name="cap_hood")
    cap.visual(
        Box((0.92, 0.10, 0.18)),
        origin=Origin(xyz=(-0.48, -0.67, 0.99)),
        material=oak,
        name="left_roof_pad",
    )
    cap.visual(
        Box((0.92, 0.10, 0.18)),
        origin=Origin(xyz=(-0.48, 0.67, 0.99)),
        material=oak,
        name="right_roof_pad",
    )

    cap.visual(
        Box((2.26, 0.16, 0.16)),
        origin=Origin(xyz=(1.13, -0.28, 0.80)),
        material=oak,
        name="left_nose_bed",
    )
    cap.visual(
        Box((2.26, 0.16, 0.16)),
        origin=Origin(xyz=(1.13, 0.28, 0.80)),
        material=oak,
        name="right_nose_bed",
    )
    cap.visual(
        Box((0.24, 0.16, 0.52)),
        origin=Origin(xyz=(2.18, -0.20, 0.82)),
        material=painted_steel,
    )
    cap.visual(
        Box((0.24, 0.16, 0.52)),
        origin=Origin(xyz=(2.18, 0.20, 0.82)),
        material=painted_steel,
    )
    cap.visual(Box((0.24, 0.48, 0.10)), origin=Origin(xyz=(2.18, 0.0, 1.03)), material=painted_steel)
    cap.visual(Box((0.24, 0.48, 0.10)), origin=Origin(xyz=(2.18, 0.0, 0.61)), material=painted_steel)
    cap.visual(
        _save_mesh("front_bearing", _annulus_mesh(0.18, 0.112, 0.12)),
        origin=Origin(xyz=(2.18, 0.0, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="front_bearing",
    )
    cap.visual(
        _save_mesh("front_thrust_plate", _annulus_mesh(0.15, 0.082, 0.02)),
        origin=Origin(xyz=(2.30, 0.0, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="front_thrust_plate",
    )

    cap.visual(
        Box((0.22, 0.14, 0.42)),
        origin=Origin(xyz=(1.70, -0.19, 0.82)),
        material=painted_steel,
        name="rear_bearing_left_cheek",
    )
    cap.visual(
        Box((0.22, 0.14, 0.42)),
        origin=Origin(xyz=(1.70, 0.19, 0.82)),
        material=painted_steel,
        name="rear_bearing_right_cheek",
    )
    cap.visual(
        Box((0.22, 0.42, 0.08)),
        origin=Origin(xyz=(1.70, 0.0, 1.00)),
        material=painted_steel,
        name="rear_bearing_top_cap",
    )
    cap.visual(
        Box((0.22, 0.42, 0.08)),
        origin=Origin(xyz=(1.70, 0.0, 0.64)),
        material=painted_steel,
        name="rear_bearing_bottom_cap",
    )
    cap.visual(
        _save_mesh("rear_bearing", _annulus_mesh(0.155, 0.126, 0.10)),
        origin=Origin(xyz=(1.70, 0.0, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_bearing",
    )

    for sign in (-1.0, 1.0):
        _add_box_member(
            cap,
            (0.56, 0.68 * sign, 0.30),
            (0.78, 0.18 * sign, 1.00),
            width=0.09,
            depth=0.12,
            material=oak,
        )
        _add_box_member(
            cap,
            (0.18, 0.70 * sign, 0.28),
            (0.33, 0.14 * sign, 0.95),
            width=0.08,
            depth=0.10,
            material=oak,
        )
    cap.visual(
        Box((0.66, 0.04, 0.74)),
        origin=Origin(xyz=(-0.30, -0.76, 0.80)),
        material=hatch_red,
        name="cap_side_hatch",
    )
    _add_bolt_circle_z(
        cap,
        bolt_circle_radius=1.24,
        bolt_radius=0.028,
        shank_length=0.06,
        z_center=0.08,
        count=12,
        material=galvanized,
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.60, 2.10, 1.80)),
        mass=420.0,
        origin=Origin(xyz=(-0.18, 0.0, 0.86)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.08, length=0.98),
        origin=Origin(xyz=(-0.50, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="windshaft",
    )
    rotor.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(-0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="front_journal",
    )
    rotor.visual(
        Cylinder(radius=0.112, length=0.10),
        origin=Origin(xyz=(-0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rear_journal",
    )
    rotor.visual(
        Cylinder(radius=0.11, length=0.04),
        origin=Origin(xyz=(-0.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="front_thrust_collar",
    )
    rotor.visual(
        Cylinder(radius=0.34, length=0.48),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.44, length=0.08),
        origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="hub_adapter_flange",
    )
    nose_geom = LatheGeometry(
        [
            (0.0, -0.22),
            (0.10, -0.18),
            (0.20, -0.04),
            (0.19, 0.08),
            (0.10, 0.18),
            (0.0, 0.22),
        ],
        segments=56,
    )
    rotor.visual(
        _save_mesh("hub_nose", nose_geom),
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_tar,
        name="hub_nose",
    )
    rotor.visual(Box((0.16, 0.16, 9.90)), origin=Origin(xyz=(0.02, 0.0, 0.0)), material=oak)
    rotor.visual(Box((0.16, 9.90, 0.16)), origin=Origin(xyz=(0.02, 0.0, 0.0)), material=oak)
    rotor.visual(Box((0.18, 0.54, 0.18)), origin=Origin(xyz=(-0.10, 0.0, 0.0)), material=painted_steel)
    rotor.visual(Box((0.18, 0.18, 0.54)), origin=Origin(xyz=(-0.10, 0.0, 0.0)), material=painted_steel)

    _add_sail_panel(rotor, axis="z", sign=1.0, timber=oak, strap=sail_canvas, x_pos=0.08)
    _add_sail_panel(rotor, axis="z", sign=-1.0, timber=oak, strap=sail_canvas, x_pos=0.08)
    _add_sail_panel(rotor, axis="y", sign=1.0, timber=oak, strap=sail_canvas, x_pos=0.08)
    _add_sail_panel(rotor, axis="y", sign=-1.0, timber=oak, strap=sail_canvas, x_pos=0.08)

    for index in range(8):
        angle = math.tau * index / 8.0
        rotor.visual(
            Cylinder(radius=0.022, length=0.08),
            origin=Origin(
                xyz=(-0.16, 0.34 * math.cos(angle), 0.34 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=galvanized,
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=5.0, length=1.10),
        mass=310.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 7.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.2),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(2.60, 0.0, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3000.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_rotor")

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

    ctx.expect_contact(
        cap,
        tower,
        elem_a="cap_curb_ring",
        elem_b="tower_curb_ring",
        contact_tol=1e-4,
        name="cap_curb_ring_is_seated_on_tower_ring",
    )
    ctx.expect_within(
        tower,
        cap,
        axes="xy",
        inner_elem="yaw_pintle",
        outer_elem="yaw_collar",
        margin=0.02,
        name="yaw_pintle_stays_centered_within_collar",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="yz",
        inner_elem="front_journal",
        outer_elem="front_bearing",
        margin=0.02,
        name="front_journal_is_centered_in_front_bearing",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="yz",
        inner_elem="rear_journal",
        outer_elem="rear_bearing",
        margin=0.02,
        name="rear_journal_is_centered_in_rear_bearing",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="x",
        min_overlap=0.09,
        elem_a="front_journal",
        elem_b="front_bearing",
        name="front_journal_spans_front_bearing_axially",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="x",
        min_overlap=0.08,
        elem_a="rear_journal",
        elem_b="rear_bearing",
        name="rear_journal_spans_rear_bearing_axially",
    )
    ctx.expect_gap(
        rotor,
        tower,
        axis="x",
        min_gap=0.12,
        positive_elem="hub_shell",
        negative_elem="tower_shell",
        name="hub_projects_proud_of_tower_face",
    )

    with ctx.pose({yaw: math.radians(32.0)}):
        ctx.expect_contact(
            cap,
            tower,
            elem_a="cap_curb_ring",
            elem_b="tower_curb_ring",
            contact_tol=1e-4,
            name="cap_stays_seated_when_yawed",
        )

    with ctx.pose({spin: math.pi / 4.0}):
        ctx.expect_within(
            rotor,
            cap,
            axes="yz",
            inner_elem="front_journal",
            outer_elem="front_bearing",
            margin=0.02,
            name="front_journal_stays_centered_when_rotated",
        )
        ctx.expect_within(
            rotor,
            cap,
            axes="yz",
            inner_elem="rear_journal",
            outer_elem="rear_bearing",
            margin=0.02,
            name="rear_journal_stays_centered_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
