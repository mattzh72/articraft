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
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


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


def _add_lattice_blade(
    part,
    *,
    radial_axis: str,
    sign: int,
    x: float,
    root: float,
    tip: float,
    half_width: float,
    frame_material,
    sail_material,
    name_prefix: str,
) -> None:
    span = tip - root
    rib_positions = (0.22, 0.42, 0.62, 0.82)
    sail_bays = ((0.22, 0.42), (0.46, 0.64), (0.68, 0.88))

    if radial_axis == "y":
        center = sign * (root + span * 0.5)
        for side_name, z in (("upper", half_width), ("lower", -half_width)):
            part.visual(
                Box((0.05, span, 0.05)),
                origin=Origin(xyz=(x, center, z)),
                material=frame_material,
                name=f"{name_prefix}_{side_name}_spar",
            )
        for index, t in enumerate(rib_positions):
            y = sign * (root + t * span)
            part.visual(
                Box((0.05, 0.08, half_width * 2.0)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=frame_material,
                name=f"{name_prefix}_rib_{index}",
            )
        for index, (t0, t1) in enumerate(sail_bays):
            y_center = sign * (root + (t0 + t1) * 0.5 * span)
            part.visual(
                Box((0.012, (t1 - t0) * span, half_width * 1.7)),
                origin=Origin(xyz=(x + 0.018, y_center, 0.0)),
                material=sail_material,
                name=f"{name_prefix}_sail_{index}",
            )
        return

    center = sign * (root + span * 0.5)
    for side_name, y in (("left", half_width), ("right", -half_width)):
        part.visual(
            Box((0.05, 0.05, span)),
            origin=Origin(xyz=(x, y, center)),
            material=frame_material,
            name=f"{name_prefix}_{side_name}_spar",
        )
    for index, t in enumerate(rib_positions):
        z = sign * (root + t * span)
        part.visual(
            Box((0.05, half_width * 2.0, 0.08)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=frame_material,
            name=f"{name_prefix}_rib_{index}",
        )
    for index, (t0, t1) in enumerate(sail_bays):
        z_center = sign * (root + (t0 + t1) * 0.5 * span)
        part.visual(
            Box((0.012, half_width * 1.7, (t1 - t0) * span)),
            origin=Origin(xyz=(x + 0.018, 0.0, z_center)),
            material=sail_material,
            name=f"{name_prefix}_sail_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = model.material("stone", rgba=(0.72, 0.69, 0.62, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.47, 0.43, 0.39, 1.0))
    oak = model.material("oak", rgba=(0.47, 0.33, 0.20, 1.0))
    weathered_oak = model.material("weathered_oak", rgba=(0.36, 0.26, 0.18, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    canvas = model.material("canvas", rgba=(0.84, 0.82, 0.72, 1.0))

    tower_shell = _save_mesh(
        "tower_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (1.92, 0.0),
                (1.92, 0.42),
                (1.52, 0.55),
                (1.40, 2.1),
                (1.24, 4.1),
                (1.10, 5.55),
                (1.05, 5.85),
                (0.0, 5.85),
            ],
            segments=72,
        ),
    )
    cap_shell = _save_mesh(
        "cap_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (1.08, 0.0),
                (1.02, 0.10),
                (0.96, 0.34),
                (0.82, 0.70),
                (0.54, 1.05),
                (0.18, 1.22),
                (0.0, 1.25),
            ],
            segments=64,
        ),
    )
    gallery_rail = _save_mesh(
        "gallery_rail",
        TorusGeometry(radius=1.74, tube=0.045, radial_segments=16, tubular_segments=72),
    )

    tower = model.part("tower")
    tower.visual(
        Box((4.4, 4.4, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_stone,
        name="foundation_plinth",
    )
    tower.visual(
        tower_shell,
        material=stone,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=1.84, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 4.62)),
        material=weathered_oak,
        name="gallery_deck",
    )
    tower.visual(
        gallery_rail,
        origin=Origin(xyz=(0.0, 0.0, 5.03)),
        material=dark_iron,
        name="gallery_top_rail",
    )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        post_radius = 1.74
        x = post_radius * math.cos(angle)
        y = post_radius * math.sin(angle)
        tower.visual(
            Cylinder(radius=0.03, length=0.38),
            origin=Origin(xyz=(x, y, 4.84)),
            material=dark_iron,
            name=f"gallery_post_{index:02d}",
        )
        _add_member(
            tower,
            (1.20 * math.cos(angle), 1.20 * math.sin(angle), 4.46),
            (x, y, 4.60),
            radius=0.028,
            material=weathered_oak,
            name=f"gallery_brace_{index:02d}",
        )
    tower.visual(
        Box((0.08, 0.86, 1.75)),
        origin=Origin(xyz=(1.48, 0.0, 1.15)),
        material=oak,
        name="entry_door",
    )
    tower.visual(
        Box((0.10, 0.58, 0.78)),
        origin=Origin(xyz=(1.26, 0.78, 3.18), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_stone,
        name="upper_window_left",
    )
    tower.visual(
        Box((0.10, 0.58, 0.78)),
        origin=Origin(xyz=(1.26, -0.78, 3.18), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_stone,
        name="upper_window_right",
    )
    tower.inertial = Inertial.from_geometry(
        Box((4.4, 4.4, 5.85)),
        mass=18500.0,
        origin=Origin(xyz=(0.0, 0.0, 2.925)),
    )

    cap = model.part("cap")
    cap.visual(
        cap_shell,
        material=weathered_oak,
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=1.10, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_iron,
        name="cap_curb_ring",
    )
    cap.visual(
        Box((0.92, 0.16, 0.78)),
        origin=Origin(xyz=(1.22, 0.27, 0.60)),
        material=oak,
        name="front_cheek_left",
    )
    cap.visual(
        Box((0.92, 0.16, 0.78)),
        origin=Origin(xyz=(1.22, -0.27, 0.60)),
        material=oak,
        name="front_cheek_right",
    )
    cap.visual(
        Box((0.72, 0.74, 0.16)),
        origin=Origin(xyz=(1.24, 0.0, 1.02)),
        material=oak,
        name="front_headbeam",
    )
    cap.visual(
        Box((0.66, 0.68, 0.16)),
        origin=Origin(xyz=(1.26, 0.0, 0.18)),
        material=oak,
        name="front_sill",
    )
    cap.visual(
        Box((0.04, 0.38, 0.32)),
        origin=Origin(xyz=(1.70, 0.0, 0.60)),
        material=dark_iron,
        name="front_bearing_block",
    )
    cap.visual(
        Box((2.30, 0.14, 0.14)),
        origin=Origin(xyz=(-1.65, 0.0, 0.54)),
        material=oak,
        name="tailpole",
    )
    _add_member(
        cap,
        (-0.62, 0.34, 0.80),
        (-2.60, 0.0, 0.22),
        radius=0.045,
        material=oak,
        name="tail_brace_left",
    )
    _add_member(
        cap,
        (-0.62, -0.34, 0.80),
        (-2.60, 0.0, 0.22),
        radius=0.045,
        material=oak,
        name="tail_brace_right",
    )
    cap.inertial = Inertial.from_geometry(
        Box((5.0, 2.4, 1.3)),
        mass=2600.0,
        origin=Origin(xyz=(-0.3, 0.0, 0.55)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.08, length=0.44),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="windshaft",
    )
    hub.visual(
        Cylinder(radius=0.24, length=0.36),
        origin=Origin(xyz=(0.44, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub_barrel",
    )
    hub.visual(
        Box((0.34, 0.46, 0.46)),
        origin=Origin(xyz=(0.46, 0.0, 0.0)),
        material=dark_iron,
        name="stock_block",
    )
    for sign, axis_name in ((1, "east"), (-1, "west")):
        _add_lattice_blade(
            hub,
            radial_axis="y",
            sign=sign,
            x=0.50,
            root=0.04,
            tip=3.02,
            half_width=0.22,
            frame_material=weathered_oak,
            sail_material=canvas,
            name_prefix=f"blade_{axis_name}",
        )
    for sign, axis_name in ((1, "north"), (-1, "south")):
        _add_lattice_blade(
            hub,
            radial_axis="z",
            sign=sign,
            x=0.50,
            root=0.04,
            tip=3.02,
            half_width=0.22,
            frame_material=weathered_oak,
            sail_material=canvas,
            name_prefix=f"blade_{axis_name}",
        )
    hub.inertial = Inertial.from_geometry(
        Box((1.2, 6.2, 6.2)),
        mass=1250.0,
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 5.85)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24000.0, velocity=0.35),
    )
    model.articulation(
        "cap_to_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=hub,
        origin=Origin(xyz=(1.72, 0.0, 0.60)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=42000.0, velocity=1.4),
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

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            0.5 * (lower[0] + upper[0]),
            0.5 * (lower[1] + upper[1]),
            0.5 * (lower[2] + upper[2]),
        )

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        min_gap=0.0,
        max_gap=0.02,
        name="cap seats directly on tower curb",
    )
    ctx.expect_origin_gap(
        hub,
        tower,
        axis="x",
        min_gap=0.8,
        name="hub is carried forward of the tower centerline",
    )
    ctx.expect_contact(
        hub,
        cap,
        elem_a="windshaft",
        elem_b="front_bearing_block",
        name="hub windshaft seats in the cap bearing block",
    )
    rest_pos = ctx.part_world_position(hub)
    with ctx.pose({cap_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(hub)
    ctx.check(
        "cap rotation swings the forward hub around the tower axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[1]) < 0.05
        and turned_pos[1] > 0.9
        and abs(turned_pos[0]) < 0.2,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    blade_rest = elem_center("hub", "blade_north_left_spar")
    with ctx.pose({hub_joint: math.pi / 2.0}):
        blade_turned = elem_center("hub", "blade_north_left_spar")
    ctx.check(
        "hub rotation turns the sail lattice around the windshaft axis",
        blade_rest is not None
        and blade_turned is not None
        and blade_rest[1] > 0.1
        and blade_turned[1] < -1.0
        and blade_turned[2] < blade_rest[2] - 1.0,
        details=f"rest={blade_rest}, turned={blade_turned}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
