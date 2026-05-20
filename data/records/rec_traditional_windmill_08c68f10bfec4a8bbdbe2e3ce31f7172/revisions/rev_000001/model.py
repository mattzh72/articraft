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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
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


def _tower_shell_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (2.12, 0.0),
            (2.18, 0.18),
            (2.08, 0.55),
            (1.92, 1.40),
            (1.70, 3.10),
            (1.44, 5.15),
            (1.18, 6.80),
            (1.08, 7.18),
            (0.0, 7.18),
        ],
        segments=88,
    )


def _cap_roof_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (1.24, 0.0),
            (1.20, 0.10),
            (1.08, 0.28),
            (0.92, 0.62),
            (0.74, 0.98),
            (0.48, 1.34),
            (0.20, 1.56),
            (0.0, 1.62),
        ],
        segments=72,
    )


def _blade_point(
    angle: float,
    radius_along: float,
    tangential_offset: float,
    *,
    x: float = 0.12,
) -> tuple[float, float, float]:
    radial_y = math.sin(angle)
    radial_z = math.cos(angle)
    tangent_y = math.cos(angle)
    tangent_z = -math.sin(angle)
    return (
        x,
        (radial_y * radius_along) + (tangent_y * tangential_offset),
        (radial_z * radius_along) + (tangent_z * tangential_offset),
    )


def _add_lattice_blade(
    part,
    *,
    angle: float,
    material,
    stock_radius: float,
    slat_radius: float,
    named_stock: str | None = None,
) -> None:
    def _lerp(a: float, b: float, t: float) -> float:
        return a + ((b - a) * t)

    def _stock_offset(radius_along: float) -> float:
        t = (radius_along - 0.42) / (4.55 - 0.42)
        return _lerp(-0.12, -0.05, t)

    def _trail_offset(radius_along: float) -> float:
        t = (radius_along - 0.82) / (4.20 - 0.82)
        return _lerp(0.36, 0.17, t)

    root_span = 0.86
    mid_span = 0.62
    tip_span = 0.34
    stock_root = _blade_point(angle, 0.42, _stock_offset(0.42))
    stock_tip = _blade_point(angle, 4.55, _stock_offset(4.55))
    trailing_root = _blade_point(angle, 0.82, _trail_offset(0.82))
    trailing_tip = _blade_point(angle, 4.20, _trail_offset(4.20))

    _add_member(
        part,
        _blade_point(angle, -0.18, _stock_offset(-0.18)),
        _blade_point(angle, 0.72, _stock_offset(0.72)),
        stock_radius * 1.12,
        material,
    )
    _add_member(part, stock_root, stock_tip, stock_radius, material, name=named_stock)
    _add_member(part, trailing_root, trailing_tip, stock_radius * 0.72, material)
    _add_member(
        part,
        _blade_point(angle, 0.16, -0.18),
        _blade_point(angle, 0.92, _trail_offset(0.92)),
        stock_radius * 0.88,
        material,
    )
    _add_member(
        part,
        _blade_point(angle, 0.26, -0.12),
        _blade_point(angle, 1.18, _trail_offset(1.18)),
        stock_radius * 0.74,
        material,
    )

    cross_specs = [
        (0.95, root_span),
        (1.45, root_span * 0.94),
        (1.95, root_span * 0.88),
        (2.45, mid_span),
        (2.95, mid_span * 0.90),
        (3.45, mid_span * 0.78),
        (3.95, tip_span * 1.05),
        (4.35, tip_span * 0.88),
    ]
    for radius_along, span in cross_specs:
        _add_member(
            part,
            _blade_point(angle, radius_along, _stock_offset(radius_along)),
            _blade_point(
                angle,
                radius_along,
                min(_trail_offset(radius_along), span),
            ),
            slat_radius,
            material,
        )

    _add_member(
        part,
        _blade_point(angle, 1.00, _trail_offset(1.00)),
        _blade_point(angle, 2.75, _stock_offset(2.75)),
        slat_radius * 0.78,
        material,
    )
    _add_member(
        part,
        _blade_point(angle, 2.15, _trail_offset(2.15)),
        _blade_point(angle, 3.95, _stock_offset(3.95)),
        slat_radius * 0.72,
        material,
    )
    _add_member(
        part,
        _blade_point(angle, 4.05, _stock_offset(4.05)),
        _blade_point(angle, 4.25, _trail_offset(4.25)),
        stock_radius * 0.62,
        material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.75, 0.72, 0.67, 1.0))
    aged_wood = model.material("aged_wood", rgba=(0.43, 0.30, 0.18, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.26, 0.18, 0.11, 1.0))
    weathered_timbers = model.material("weathered_timbers", rgba=(0.63, 0.56, 0.45, 1.0))
    slate = model.material("slate", rgba=(0.22, 0.23, 0.25, 1.0))
    iron = model.material("iron", rgba=(0.34, 0.35, 0.37, 1.0))
    glass = model.material("glass", rgba=(0.66, 0.78, 0.86, 0.40))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_tower_shell_mesh(), "tower_shell"),
        material=stone,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=1.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 7.26)),
        material=aged_wood,
        name="cap_track",
    )
    tower.visual(
        Cylinder(radius=2.30, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=stone,
        name="foundation_ring",
    )
    tower.visual(
        Box((0.22, 1.02, 1.86)),
        origin=Origin(xyz=(1.88, 0.0, 0.93)),
        material=dark_wood,
        name="front_door",
    )
    tower.visual(
        Box((0.20, 0.56, 0.96)),
        origin=Origin(xyz=(1.68, 0.0, 3.20)),
        material=glass,
        name="front_window",
    )
    tower.visual(
        Box((0.18, 0.42, 0.72)),
        origin=Origin(xyz=(1.34, 0.0, 5.72)),
        material=glass,
        name="upper_window",
    )
    tower.visual(
        Box((0.64, 0.20, 1.08)),
        origin=Origin(xyz=(0.0, 1.55, 4.55)),
        material=glass,
        name="side_window",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.18, length=7.34),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 3.67)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_cap_roof_mesh(), "cap_roof"),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=slate,
        name="cap_roof",
    )
    cap.visual(
        Cylinder(radius=1.24, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=aged_wood,
        name="cap_base_ring",
    )
    cap.visual(
        Cylinder(radius=0.24, length=2.40),
        origin=Origin(xyz=(1.50, 0.0, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="front_bearing_housing",
    )
    cap.visual(
        Box((1.12, 0.42, 0.32)),
        origin=Origin(xyz=(0.92, 0.0, 0.78)),
        material=dark_wood,
        name="front_cheeks",
    )
    cap.visual(
        Cylinder(radius=0.10, length=0.48),
        origin=Origin(xyz=(-1.22, 0.0, 0.72), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_wood,
        name="tail_pole",
    )
    _add_member(cap, (-0.72, 0.0, 0.76), (-1.38, 0.0, 0.72), 0.055, aged_wood)
    _add_member(cap, (-0.52, -0.28, 0.38), (-1.10, 0.0, 0.70), 0.040, aged_wood)
    _add_member(cap, (-0.52, 0.28, 0.38), (-1.10, 0.0, 0.70), 0.040, aged_wood)
    cap.inertial = Inertial.from_geometry(
        Box((2.90, 2.90, 1.80)),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.08, length=0.26),
        origin=Origin(xyz=(-0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="shaft_stub",
    )
    hub.visual(
        Cylinder(radius=0.24, length=0.34),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_wood,
        name="hub_drum",
    )
    hub.visual(
        Cylinder(radius=0.16, length=0.18),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="hub_cap",
    )
    hub.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="blade_root_plate",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_lattice_blade(
            hub,
            angle=angle,
            material=weathered_timbers,
            stock_radius=0.045,
            slat_radius=0.018,
            named_stock="north_blade_stock" if index == 0 else None,
        )
    hub.inertial = Inertial.from_geometry(
        Box((0.70, 9.30, 9.30)),
        mass=380.0,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 7.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.4),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(2.96, 0.0, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.5),
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
    cap_joint = object_model.get_articulation("tower_to_cap")
    hub_joint = object_model.get_articulation("cap_to_hub")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        min_gap=0.0,
        max_gap=0.12,
        positive_elem="cap_base_ring",
        negative_elem="cap_track",
        name="cap sits on the tower cap track",
    )

    hub_pos = ctx.part_world_position(hub)
    ctx.check(
        "hub stage sits high above the base",
        hub_pos is not None and hub_pos[2] > 7.8,
        details=f"hub_position={hub_pos}",
    )

    with ctx.pose({cap_joint: math.pi / 2.0}):
        turned_hub_pos = ctx.part_world_position(hub)
    ctx.check(
        "cap rotation slews the hub around the tower",
        hub_pos is not None
        and turned_hub_pos is not None
        and abs(turned_hub_pos[0]) < 0.20
        and turned_hub_pos[1] > hub_pos[0] - 0.20
        and abs(turned_hub_pos[2] - hub_pos[2]) < 0.02,
        details=f"rest={hub_pos}, turned={turned_hub_pos}",
    )

    with ctx.pose({hub_joint: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(hub, elem="north_blade_stock")
    with ctx.pose({hub_joint: math.pi / 2.0}):
        spun_aabb = ctx.part_element_world_aabb(hub, elem="north_blade_stock")

    def _aabb_dims(aabb):
        if aabb is None:
            return None
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    rest_dims = _aabb_dims(rest_aabb)
    spun_dims = _aabb_dims(spun_aabb)
    ctx.check(
        "hub rotation turns the blade plane about the shaft axis",
        rest_dims is not None
        and spun_dims is not None
        and rest_dims[2] > rest_dims[1] * 3.0
        and spun_dims[1] > spun_dims[2] * 3.0,
        details=f"rest_dims={rest_dims}, spun_dims={spun_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
