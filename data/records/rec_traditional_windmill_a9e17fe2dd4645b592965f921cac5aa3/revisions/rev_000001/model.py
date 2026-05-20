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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def _build_tower_mesh():
    profile = [
        (2.10, 0.00),
        (2.08, 0.16),
        (1.98, 1.10),
        (1.78, 3.10),
        (1.56, 5.10),
        (1.42, 5.68),
        (1.40, 5.72),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=56), "tower_shell")


def _build_cap_mesh():
    profile = [
        (1.62, 0.00),
        (1.60, 0.18),
        (1.42, 0.52),
        (1.12, 0.92),
        (0.76, 1.22),
        (0.30, 1.42),
        (0.00, 1.48),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48), "cap_shell")


def _blade_point(
    angle: float,
    radius: float,
    offset: float,
    *,
    x: float = 0.06,
) -> tuple[float, float, float]:
    radial_y = math.cos(angle)
    radial_z = math.sin(angle)
    tangential_y = -math.sin(angle)
    tangential_z = math.cos(angle)
    return (
        x,
        radial_y * radius + tangential_y * offset,
        radial_z * radius + tangential_z * offset,
    )


def _blade_half_width(radius: float) -> float:
    root_radius = 0.38
    tip_radius = 2.76
    root_width = 0.13
    tip_width = 0.21
    t = (radius - root_radius) / (tip_radius - root_radius)
    t = max(0.0, min(1.0, t))
    return root_width + (tip_width - root_width) * t


def _add_lattice_blade(part, *, angle: float, material, name_prefix: str) -> None:
    rung_radii = [0.58, 0.90, 1.22, 1.56, 1.90, 2.24, 2.54, 2.76]

    _add_member(
        part,
        _blade_point(angle, 0.04, 0.0),
        _blade_point(angle, 2.76, 0.0),
        0.045,
        material,
        name=f"{name_prefix}_center_spar",
    )
    _add_member(
        part,
        _blade_point(angle, 0.26, -0.11),
        _blade_point(angle, 2.76, -_blade_half_width(2.76)),
        0.026,
        material,
        name=f"{name_prefix}_leading_spar",
    )
    _add_member(
        part,
        _blade_point(angle, 0.26, 0.11),
        _blade_point(angle, 2.76, _blade_half_width(2.76)),
        0.026,
        material,
        name=f"{name_prefix}_trailing_spar",
    )

    previous_radius = 0.38
    for index, rung_radius in enumerate(rung_radii):
        half_width = _blade_half_width(rung_radius)
        rung_name = None
        if index == len(rung_radii) - 1:
            rung_name = f"{name_prefix}_tip_rung"
        _add_member(
            part,
            _blade_point(angle, rung_radius, -half_width),
            _blade_point(angle, rung_radius, half_width),
            0.017,
            material,
            name=rung_name,
        )
        if index > 0:
            prev_width = _blade_half_width(previous_radius)
            if index % 2 == 0:
                _add_member(
                    part,
                    _blade_point(angle, previous_radius, -prev_width),
                    _blade_point(angle, rung_radius, half_width),
                    0.012,
                    material,
                )
            else:
                _add_member(
                    part,
                    _blade_point(angle, previous_radius, prev_width),
                    _blade_point(angle, rung_radius, -half_width),
                    0.012,
                    material,
                )
        previous_radius = rung_radius


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    masonry = model.material("masonry", rgba=(0.78, 0.75, 0.69, 1.0))
    roof_thatch = model.material("roof_thatch", rgba=(0.34, 0.24, 0.12, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.46, 0.34, 0.22, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.24, 0.16, 0.10, 1.0))
    metal = model.material("metal", rgba=(0.43, 0.44, 0.46, 1.0))
    trim = model.material("trim", rgba=(0.87, 0.85, 0.79, 1.0))

    tower_mesh = _build_tower_mesh()
    cap_mesh = _build_cap_mesh()

    tower = model.part("tower")
    tower.visual(tower_mesh, material=masonry, name="tower_shell")
    tower.visual(
        Cylinder(radius=1.56, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 5.77)),
        material=dark_timber,
        name="cap_curb",
    )
    tower.visual(
        Cylinder(radius=2.18, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_timber,
        name="base_plinth",
    )
    tower.visual(
        Box((0.92, 0.18, 1.58)),
        origin=Origin(xyz=(1.76, 0.0, 0.79)),
        material=weathered_wood,
        name="front_door",
    )
    tower.visual(
        Box((0.84, 0.06, 1.64)),
        origin=Origin(xyz=(1.58, 0.0, 0.82)),
        material=trim,
        name="door_trim",
    )
    tower.visual(
        Box((0.48, 0.16, 0.72)),
        origin=Origin(xyz=(1.82, 0.0, 2.35)),
        material=trim,
        name="window_frame_lower",
    )
    tower.visual(
        Box((0.38, 0.10, 0.58)),
        origin=Origin(xyz=(1.90, 0.0, 2.35)),
        material=dark_timber,
        name="window_shutter_lower",
    )
    tower.visual(
        Box((0.38, 0.16, 0.56)),
        origin=Origin(xyz=(1.54, 0.0, 4.24)),
        material=trim,
        name="window_frame_upper",
    )
    tower.visual(
        Box((0.28, 0.10, 0.42)),
        origin=Origin(xyz=(1.64, 0.0, 4.24)),
        material=dark_timber,
        name="window_shutter_upper",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.10, length=5.84),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 2.92)),
    )

    cap = model.part("cap")
    cap.visual(cap_mesh, material=roof_thatch, name="cap_shell")
    cap.visual(
        Cylinder(radius=1.60, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_timber,
        name="cap_ring",
    )
    cap.visual(
        Box((0.78, 0.56, 0.42)),
        origin=Origin(xyz=(1.42, 0.0, 0.56)),
        material=weathered_wood,
        name="shaft_house",
    )
    cap.visual(
        Cylinder(radius=0.17, length=0.54),
        origin=Origin(xyz=(1.84, 0.0, 0.56), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft_bearing",
    )
    cap.visual(
        Box((1.26, 0.18, 0.12)),
        origin=Origin(xyz=(-1.30, 0.0, 0.48)),
        material=weathered_wood,
        name="tail_pole",
    )
    cap.visual(
        Box((0.48, 0.08, 0.70)),
        origin=Origin(xyz=(-1.86, 0.0, 0.76)),
        material=weathered_wood,
        name="tail_fin",
    )
    cap.inertial = Inertial.from_geometry(
        Box((3.70, 3.20, 1.52)),
        mass=230.0,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )

    sails = model.part("sails")
    sails.visual(
        Cylinder(radius=0.44, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub_back_plate",
    )
    sails.visual(
        Cylinder(radius=0.30, length=0.52),
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_wood,
        name="hub_core",
    )
    sails.visual(
        Cylinder(radius=0.14, length=0.12),
        origin=Origin(xyz=(0.58, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub_front_cap",
    )
    for blade_index, angle in enumerate(
        (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)
    ):
        _add_lattice_blade(
            sails,
            angle=angle,
            material=weathered_wood,
            name_prefix=f"blade_{blade_index}",
        )
    sails.inertial = Inertial.from_geometry(
        Box((0.72, 6.40, 6.40)),
        mass=180.0,
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 5.84)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25),
    )
    model.articulation(
        "cap_to_sails",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sails,
        origin=Origin(xyz=(2.11, 0.0, 0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4),
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
    sails = object_model.get_part("sails")
    cap_yaw = object_model.get_articulation("tower_to_cap")
    sail_spin = object_model.get_articulation("cap_to_sails")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="cap_ring",
        negative_elem="cap_curb",
        name="cap ring sits on the tower curb",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        min_overlap=2.7,
        elem_a="cap_ring",
        elem_b="cap_curb",
        name="cap remains centered over the tower",
    )
    ctx.expect_gap(
        sails,
        cap,
        axis="x",
        max_gap=0.003,
        max_penetration=1e-5,
        positive_elem="hub_back_plate",
        negative_elem="shaft_bearing",
        name="hub seats against the cap bearing",
    )
    ctx.expect_overlap(
        sails,
        cap,
        axes="yz",
        min_overlap=0.32,
        elem_a="hub_back_plate",
        elem_b="shaft_bearing",
        name="hub stays aligned with the cap bearing",
    )

    rest_sails = ctx.part_world_position(sails)
    with ctx.pose({cap_yaw: math.pi / 2.0}):
        turned_sails = ctx.part_world_position(sails)
    ctx.check(
        "cap yaws the sail hub around the tower axis",
        rest_sails is not None
        and turned_sails is not None
        and rest_sails[0] > 1.8
        and abs(rest_sails[1]) < 0.01
        and abs(turned_sails[0]) < 0.01
        and turned_sails[1] > 1.8
        and abs(rest_sails[2] - turned_sails[2]) < 1e-6,
        details=f"rest={rest_sails}, turned={turned_sails}",
    )

    tip_rest = ctx.part_element_world_aabb(sails, elem="blade_0_tip_rung")
    with ctx.pose({sail_spin: math.pi / 4.0}):
        tip_spun = ctx.part_element_world_aabb(sails, elem="blade_0_tip_rung")
    tip_rest_center = _aabb_center(tip_rest) if tip_rest is not None else None
    tip_spun_center = _aabb_center(tip_spun) if tip_spun is not None else None
    ctx.check(
        "hub rotation carries the blade lattice around the shaft axis",
        tip_rest_center is not None
        and tip_spun_center is not None
        and tip_spun_center[2] > tip_rest_center[2] + 0.55
        and tip_spun_center[1] < tip_rest_center[1] - 0.55,
        details=f"rest={tip_rest_center}, spun={tip_spun_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
