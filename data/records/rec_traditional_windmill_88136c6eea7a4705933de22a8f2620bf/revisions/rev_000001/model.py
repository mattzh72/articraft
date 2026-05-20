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


def _rpy_for_z_axis(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_beam(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    thickness: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, thickness, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_axis(a, b)),
        material=material,
        name=name,
    )


def _circle_section(
    radius: float,
    z: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(index * math.tau / segments),
            radius * math.sin(index * math.tau / segments),
            z,
        )
        for index in range(segments)
    ]


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z)
        for (y, z) in rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    ]


def _rotate_about_x(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (x, (y * cos_a) - (z * sin_a), (y * sin_a) + (z * cos_a))


def _add_lattice_sail(part, *, angle: float, material, prefix: str) -> None:
    stations = [
        (0.82, 0.22),
        (1.35, 0.32),
        (1.95, 0.44),
        (2.55, 0.56),
        (3.15, 0.68),
        (3.75, 0.80),
        (4.35, 0.90),
        (4.95, 0.82),
    ]

    def rotated(point: tuple[float, float, float]) -> tuple[float, float, float]:
        return _rotate_about_x(point, angle)

    _add_box_beam(
        part,
        rotated((1.00, 0.0, 0.18)),
        rotated((1.00, 0.0, 5.35)),
        width=0.22,
        thickness=0.10,
        material=material,
        name=f"{prefix}_stock",
    )
    _add_box_beam(
        part,
        rotated((1.00, -stations[0][1], stations[0][0])),
        rotated((1.00, -stations[-1][1], stations[-1][0])),
        width=0.10,
        thickness=0.05,
        material=material,
        name=f"{prefix}_left_rail",
    )
    _add_box_beam(
        part,
        rotated((1.00, stations[0][1], stations[0][0])),
        rotated((1.00, stations[-1][1], stations[-1][0])),
        width=0.10,
        thickness=0.05,
        material=material,
        name=f"{prefix}_right_rail",
    )

    for index, (z_pos, half_width) in enumerate(stations):
        _add_box_beam(
            part,
            rotated((1.00, -half_width, z_pos)),
            rotated((1.00, half_width, z_pos)),
            width=0.08,
            thickness=0.04,
            material=material,
            name=f"{prefix}_slat_{index:02d}",
        )

    for index in range(len(stations) - 1):
        z0, w0 = stations[index]
        z1, w1 = stations[index + 1]
        if index % 2 == 0:
            first_a = (1.00, -w0, z0)
            first_b = (1.00, 0.0, z1)
            second_a = (1.00, 0.0, z0)
            second_b = (1.00, w1, z1)
        else:
            first_a = (1.00, 0.0, z0)
            first_b = (1.00, -w1, z1)
            second_a = (1.00, w0, z0)
            second_b = (1.00, 0.0, z1)
        _add_box_beam(
            part,
            rotated(first_a),
            rotated(first_b),
            width=0.06,
            thickness=0.035,
            material=material,
            name=f"{prefix}_brace_a_{index:02d}",
        )
        _add_box_beam(
            part,
            rotated(second_a),
            rotated(second_b),
            width=0.06,
            thickness=0.035,
            material=material,
            name=f"{prefix}_brace_b_{index:02d}",
        )


def _build_tower_shell():
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.00),
                (2.75, 0.00),
                (2.66, 0.55),
                (2.40, 3.20),
                (2.08, 6.45),
                (1.82, 8.55),
                (0.0, 8.55),
            ],
            segments=88,
        ),
        "tower_shell",
    )


def _build_bearing_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.70, -0.51),
                (0.76, -0.39),
                (0.78, 0.00),
                (0.76, 0.39),
                (0.70, 0.51),
            ],
            [
                (0.64, -0.53),
                (0.64, 0.53),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
        "front_bearing_shell",
    )


def _build_cap_roof():
    return mesh_from_geometry(
        section_loft(
            [
                _yz_section(-1.45, 1.70, 1.10, 0.16),
                _yz_section(-0.55, 2.45, 1.75, 0.24),
                _yz_section(0.10, 2.70, 1.85, 0.26),
                _yz_section(0.62, 1.25, 0.95, 0.16),
            ]
        ),
        "cap_roof",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.83, 0.81, 0.76, 1.0))
    aged_oak = model.material("aged_oak", rgba=(0.42, 0.31, 0.20, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.26, 0.19, 0.12, 1.0))
    weathered_roof = model.material("weathered_roof", rgba=(0.17, 0.16, 0.15, 1.0))
    iron = model.material("iron", rgba=(0.29, 0.31, 0.33, 1.0))

    tower = model.part("tower")
    tower.visual(_build_tower_shell(), material=stone, name="tower_shell")
    tower.visual(
        Cylinder(radius=2.90, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=stone,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=1.95, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 8.69)),
        material=dark_oak,
        name="cap_curb_ring",
    )
    tower.visual(
        Box((1.20, 0.18, 2.20)),
        origin=Origin(xyz=(2.12, 0.0, 1.10)),
        material=dark_oak,
        name="front_door",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.75, length=8.90),
        mass=52000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.45)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=1.98, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=iron,
        name="turntable_ring",
    )
    cap.visual(
        Box((3.55, 2.55, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=aged_oak,
        name="cap_deck",
    )
    cap.visual(
        _build_cap_roof(),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=weathered_roof,
        name="roof_shell",
    )
    cap.visual(
        _build_bearing_shell(),
        origin=Origin(xyz=(3.35, 0.0, 1.28), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_bearing_housing",
    )
    cap.visual(
        Box((2.25, 0.42, 1.42)),
        origin=Origin(xyz=(2.25, 0.98, 1.02)),
        material=dark_oak,
        name="right_front_cheek",
    )
    cap.visual(
        Box((2.25, 0.42, 1.42)),
        origin=Origin(xyz=(2.25, -0.98, 1.02)),
        material=dark_oak,
        name="left_front_cheek",
    )
    cap.visual(
        Box((1.75, 2.00, 0.56)),
        origin=Origin(xyz=(-1.15, 0.0, 0.98)),
        material=aged_oak,
        name="rear_counterweight_box",
    )
    cap.visual(
        Box((0.96, 0.44, 0.38)),
        origin=Origin(xyz=(2.38, 0.88, 0.67)),
        material=dark_oak,
        name="right_lower_bearing_pedestal",
    )
    cap.visual(
        Box((0.96, 0.44, 0.38)),
        origin=Origin(xyz=(2.38, -0.88, 0.67)),
        material=dark_oak,
        name="left_lower_bearing_pedestal",
    )
    cap.visual(
        Box((0.90, 1.92, 0.30)),
        origin=Origin(xyz=(2.45, 0.0, 1.59)),
        material=dark_oak,
        name="upper_bearing_tie",
    )
    _add_box_beam(
        cap,
        (0.82, 1.08, 0.56),
        (3.05, 0.88, 1.05),
        width=0.22,
        thickness=0.20,
        material=dark_oak,
        name="right_lower_support",
    )
    _add_box_beam(
        cap,
        (0.82, -1.08, 0.56),
        (3.05, -0.88, 1.05),
        width=0.22,
        thickness=0.20,
        material=dark_oak,
        name="left_lower_support",
    )
    _add_box_beam(
        cap,
        (0.42, 0.98, 1.58),
        (3.00, 0.82, 1.38),
        width=0.18,
        thickness=0.16,
        material=dark_oak,
        name="right_upper_support",
    )
    _add_box_beam(
        cap,
        (0.42, -0.98, 1.58),
        (3.00, -0.82, 1.38),
        width=0.18,
        thickness=0.16,
        material=dark_oak,
        name="left_upper_support",
    )
    cap.inertial = Inertial.from_geometry(
        Box((3.80, 2.70, 2.20)),
        mass=7200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.16, length=2.40),
        origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="main_shaft",
    )
    hub.visual(
        Cylinder(radius=0.54, length=0.92),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_drum",
    )
    hub.visual(
        Cylinder(radius=0.30, length=0.58),
        origin=Origin(xyz=(1.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_nose",
    )
    for blade_index, blade_angle in enumerate(
        (0.0, math.pi * 0.5, math.pi, math.pi * 1.5)
    ):
        _add_lattice_sail(
            hub,
            angle=blade_angle,
            material=aged_oak,
            prefix=f"sail_{blade_index}",
        )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.60, length=2.40),
        mass=1800.0,
        origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 8.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28000.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(3.35, 0.0, 1.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22000.0, velocity=1.20),
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
    cap = object_model.get_part("cap")
    hub = object_model.get_part("hub")
    cap_spin = object_model.get_articulation("tower_to_cap")
    hub_spin = object_model.get_articulation("cap_to_hub")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="turntable_ring",
        negative_elem="cap_curb_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="cap turntable ring seats on tower curb ring",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        elem_a="turntable_ring",
        elem_b="cap_curb_ring",
        min_overlap=3.8,
        name="cap turntable ring stays centered on tower curb",
    )
    ctx.expect_contact(
        hub,
        cap,
        elem_a="main_shaft",
        elem_b="upper_bearing_tie",
        name="hub shaft is supported by the cap bearing tie",
    )
    ctx.expect_overlap(
        hub,
        cap,
        axes="yz",
        elem_a="hub_drum",
        elem_b="front_bearing_housing",
        min_overlap=1.0,
        name="hub drum remains captured by the front bearing housing",
    )

    rest_hub_position = ctx.part_world_position(hub)
    with ctx.pose({cap_spin: math.pi / 2.0}):
        turned_hub_position = ctx.part_world_position(hub)
    ctx.check(
        "cap rotation carries the sail hub around the tower axis",
        rest_hub_position is not None
        and turned_hub_position is not None
        and abs(turned_hub_position[0]) < 0.05
        and turned_hub_position[1] > rest_hub_position[0] - 0.10,
        details=f"rest={rest_hub_position}, turned={turned_hub_position}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return (
            (min_corner[0] + max_corner[0]) * 0.5,
            (min_corner[1] + max_corner[1]) * 0.5,
            (min_corner[2] + max_corner[2]) * 0.5,
        )

    rest_sail_center = _aabb_center(ctx.part_element_world_aabb(hub, elem="sail_0_stock"))
    with ctx.pose({hub_spin: math.pi / 2.0}):
        quarter_turn_sail_center = _aabb_center(
            ctx.part_element_world_aabb(hub, elem="sail_0_stock")
        )
    ctx.check(
        "hub rotation swings the top sail into the side position",
        rest_sail_center is not None
        and quarter_turn_sail_center is not None
        and rest_sail_center[2] > quarter_turn_sail_center[2] + 2.0
        and quarter_turn_sail_center[1] < -2.0,
        details=f"rest={rest_sail_center}, quarter_turn={quarter_turn_sail_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
