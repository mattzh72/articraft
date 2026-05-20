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


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_box_beam(
    part,
    *,
    name: str,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    width: float,
    depth: float,
    material,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_shift: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_shift) for z, y in rounded_rect_profile(height, width, radius)]


def _sail_point(
    x: float,
    radial_axis: tuple[float, float],
    tangential_axis: tuple[float, float],
    radial: float,
    tangential: float = 0.0,
) -> tuple[float, float, float]:
    return (
        x,
        radial_axis[0] * radial + tangential_axis[0] * tangential,
        radial_axis[1] * radial + tangential_axis[1] * tangential,
    )


def _add_lattice_sail(
    part,
    *,
    prefix: str,
    x: float,
    radial_axis: tuple[float, float],
    tangential_axis: tuple[float, float],
    root_radius: float,
    tip_radius: float,
    sail_width: float,
    stock_width: float,
    stock_depth: float,
    slat_width: float,
    slat_depth: float,
    material,
) -> None:
    tip_frame_radius = tip_radius - 0.20
    lattice_start = root_radius + 0.42
    lattice_end = tip_radius - 0.36
    rail_offset = sail_width * 0.44

    _add_box_beam(
        part,
        name=f"{prefix}_main_stock",
        a=_sail_point(x, radial_axis, tangential_axis, root_radius - 0.06),
        b=_sail_point(x, radial_axis, tangential_axis, tip_radius),
        width=stock_width,
        depth=stock_depth,
        material=material,
    )
    _add_box_beam(
        part,
        name=f"{prefix}_leading_rail",
        a=_sail_point(x, radial_axis, tangential_axis, lattice_start, rail_offset),
        b=_sail_point(x, radial_axis, tangential_axis, lattice_end, rail_offset),
        width=stock_width * 0.85,
        depth=stock_depth * 0.85,
        material=material,
    )
    _add_box_beam(
        part,
        name=f"{prefix}_trailing_rail",
        a=_sail_point(x, radial_axis, tangential_axis, lattice_start, -rail_offset),
        b=_sail_point(x, radial_axis, tangential_axis, lattice_end, -rail_offset),
        width=stock_width * 0.85,
        depth=stock_depth * 0.85,
        material=material,
    )
    _add_box_beam(
        part,
        name=f"{prefix}_inner_spreader",
        a=_sail_point(x, radial_axis, tangential_axis, lattice_start - 0.10, rail_offset),
        b=_sail_point(x, radial_axis, tangential_axis, lattice_start - 0.10, -rail_offset),
        width=slat_width,
        depth=slat_depth,
        material=material,
    )
    _add_box_beam(
        part,
        name=f"{prefix}_tip_spreader",
        a=_sail_point(x, radial_axis, tangential_axis, tip_frame_radius, rail_offset),
        b=_sail_point(x, radial_axis, tangential_axis, tip_frame_radius, -rail_offset),
        width=slat_width,
        depth=slat_depth,
        material=material,
    )
    _add_box_beam(
        part,
        name=f"{prefix}_diag_a",
        a=_sail_point(x, radial_axis, tangential_axis, lattice_start - 0.02, rail_offset),
        b=_sail_point(x, radial_axis, tangential_axis, lattice_end, -rail_offset),
        width=slat_width * 0.85,
        depth=slat_depth,
        material=material,
    )
    _add_box_beam(
        part,
        name=f"{prefix}_diag_b",
        a=_sail_point(x, radial_axis, tangential_axis, lattice_start - 0.02, -rail_offset),
        b=_sail_point(x, radial_axis, tangential_axis, lattice_end, rail_offset),
        width=slat_width * 0.85,
        depth=slat_depth,
        material=material,
    )

    slat_count = 6
    for index in range(slat_count):
        radial = lattice_start + ((index + 0.45) / slat_count) * (lattice_end - lattice_start)
        _add_box_beam(
            part,
            name=f"{prefix}_slat_{index + 1}",
            a=_sail_point(x, radial_axis, tangential_axis, radial, rail_offset),
            b=_sail_point(x, radial_axis, tangential_axis, radial, -rail_offset),
            width=slat_width,
            depth=slat_depth,
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    masonry = model.material("masonry", rgba=(0.84, 0.80, 0.73, 1.0))
    dark_shingle = model.material("dark_shingle", rgba=(0.22, 0.19, 0.17, 1.0))
    painted_trim = model.material("painted_trim", rgba=(0.92, 0.90, 0.84, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.58, 0.48, 0.34, 1.0))
    ironwork = model.material("ironwork", rgba=(0.19, 0.20, 0.22, 1.0))

    tower_shell = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (1.70, 0.0),
                (1.62, 0.38),
                (1.48, 1.60),
                (1.30, 3.60),
                (1.00, 5.45),
                (0.83, 6.05),
                (0.0, 6.05),
            ],
            segments=64,
        ),
        "tower_shell",
    )
    cap_shell = mesh_from_geometry(
        section_loft(
            [
                _yz_section(-0.92, width=0.72, height=0.48, radius=0.10, z_shift=0.36),
                _yz_section(-0.46, width=1.35, height=0.94, radius=0.18, z_shift=0.54),
                _yz_section(0.10, width=1.58, height=1.04, radius=0.20, z_shift=0.58),
                _yz_section(0.72, width=1.34, height=0.94, radius=0.18, z_shift=0.54),
                _yz_section(1.16, width=0.48, height=0.34, radius=0.08, z_shift=0.34),
            ]
        ),
        "cap_shell",
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=1.90, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=masonry,
        name="foundation_ring",
    )
    tower.visual(
        tower_shell,
        material=masonry,
        name="tower_shell",
    )
    tower.visual(
        Box((0.60, 0.18, 1.05)),
        origin=Origin(xyz=(1.58, 0.0, 0.53)),
        material=painted_trim,
        name="front_door",
    )
    tower.visual(
        Box((0.32, 0.10, 0.46)),
        origin=Origin(xyz=(1.24, 0.0, 2.60)),
        material=painted_trim,
        name="front_window",
    )
    tower.visual(
        Cylinder(radius=0.90, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 5.99)),
        material=weathered_wood,
        name="cap_track_ring",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=1.75, length=6.10),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 3.05)),
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.88, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=weathered_wood,
        name="curb_ring",
    )
    cap.visual(
        cap_shell,
        material=dark_shingle,
        name="cap_shell",
    )
    cap.visual(
        Box((0.50, 0.16, 0.58)),
        origin=Origin(xyz=(0.93, 0.23, 0.38)),
        material=weathered_wood,
        name="left_shaft_cheek",
    )
    cap.visual(
        Box((0.50, 0.16, 0.58)),
        origin=Origin(xyz=(0.93, -0.23, 0.38)),
        material=weathered_wood,
        name="right_shaft_cheek",
    )
    cap.visual(
        Box((0.42, 0.48, 0.14)),
        origin=Origin(xyz=(0.97, 0.0, 0.67)),
        material=weathered_wood,
        name="upper_bearing_beam",
    )
    cap.visual(
        Box((0.42, 0.42, 0.16)),
        origin=Origin(xyz=(0.97, 0.0, 0.13)),
        material=weathered_wood,
        name="lower_bearing_beam",
    )
    cap.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(1.12, 0.0, 0.38), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ironwork,
        name="front_bearing",
    )
    cap.visual(
        Box((1.15, 0.14, 0.12)),
        origin=Origin(xyz=(-0.80, 0.0, 0.42)),
        material=weathered_wood,
        name="rear_tail_beam",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.40, 1.70, 1.30)),
        mass=780.0,
        origin=Origin(xyz=(0.10, 0.0, 0.55)),
    )

    sail_hub = model.part("sail_hub")
    sail_hub.visual(
        Cylinder(radius=0.10, length=0.92),
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ironwork,
        name="windshaft",
    )
    sail_hub.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ironwork,
        name="rear_collar",
    )
    sail_hub.visual(
        Cylinder(radius=0.25, length=0.34),
        origin=Origin(xyz=(0.78, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_wood,
        name="hub_barrel",
    )
    sail_hub.visual(
        Box((0.24, 0.66, 0.66)),
        origin=Origin(xyz=(0.76, 0.0, 0.0)),
        material=weathered_wood,
        name="hub_cross",
    )
    for prefix, radial_axis, tangential_axis in (
        ("top_sail", (0.0, 1.0), (-1.0, 0.0)),
        ("bottom_sail", (0.0, -1.0), (1.0, 0.0)),
        ("right_sail", (1.0, 0.0), (0.0, 1.0)),
        ("left_sail", (-1.0, 0.0), (0.0, -1.0)),
    ):
        _add_lattice_sail(
            sail_hub,
            prefix=prefix,
            x=0.88,
            radial_axis=radial_axis,
            tangential_axis=tangential_axis,
            root_radius=0.18,
            tip_radius=4.80,
            sail_width=1.18,
            stock_width=0.10,
            stock_depth=0.08,
            slat_width=0.05,
            slat_depth=0.04,
            material=weathered_wood,
        )
    sail_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=2.90, length=1.10),
        mass=420.0,
        origin=Origin(xyz=(0.70, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 6.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.5),
    )
    model.articulation(
        "cap_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=sail_hub,
        origin=Origin(xyz=(1.18, 0.0, 0.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    sail_hub = object_model.get_part("sail_hub")
    tower_to_cap = object_model.get_articulation("tower_to_cap")
    cap_to_sail_hub = object_model.get_articulation("cap_to_sail_hub")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="curb_ring",
        negative_elem="cap_track_ring",
        max_gap=0.002,
        max_penetration=0.0,
        name="cap sits on tower curb ring",
    )
    ctx.expect_origin_distance(
        cap,
        tower,
        axes="xy",
        max_dist=0.0,
        name="cap rotation axis stays centered on tower axis",
    )
    ctx.expect_contact(
        sail_hub,
        cap,
        elem_a="windshaft",
        elem_b="front_bearing",
        name="windshaft seats in the cap front bearing",
    )

    cap_pos = ctx.part_world_position(cap)
    sail_pos = ctx.part_world_position(sail_hub)
    shaft_offset = cap_to_sail_hub.origin.xyz
    rest_radial_offset = None
    if cap_pos is not None and sail_pos is not None:
        rest_radial_offset = math.hypot(sail_pos[0] - cap_pos[0], sail_pos[1] - cap_pos[1])
    ctx.check(
        "sail hub mount matches cap windshaft offset at rest",
        cap_pos is not None
        and sail_pos is not None
        and abs(rest_radial_offset - shaft_offset[0]) <= 1e-6
        and abs(sail_pos[2] - cap_pos[2] - shaft_offset[2]) <= 1e-6,
        details=f"cap={cap_pos}, sail={sail_pos}, expected_offset={shaft_offset}",
    )
    with ctx.pose({tower_to_cap: math.radians(70.0)}):
        yaw_cap_pos = ctx.part_world_position(cap)
        yaw_sail_pos = ctx.part_world_position(sail_hub)
        yaw_radial_offset = None
        if yaw_cap_pos is not None and yaw_sail_pos is not None:
            yaw_radial_offset = math.hypot(
                yaw_sail_pos[0] - yaw_cap_pos[0],
                yaw_sail_pos[1] - yaw_cap_pos[1],
            )
        ctx.expect_origin_distance(
            cap,
            tower,
            axes="xy",
            max_dist=0.0,
            name="cap remains centered after yaw rotation",
        )
        ctx.check(
            "sail hub stays centered on cap shaft axis after yaw rotation",
            yaw_cap_pos is not None
            and yaw_sail_pos is not None
            and abs(yaw_radial_offset - shaft_offset[0]) <= 1e-6
            and abs(yaw_sail_pos[2] - yaw_cap_pos[2] - shaft_offset[2]) <= 1e-6,
            details=f"cap={yaw_cap_pos}, sail={yaw_sail_pos}, expected_offset={shaft_offset}",
        )
    with ctx.pose({cap_to_sail_hub: math.pi / 2.0}):
        spun_sail_pos = ctx.part_world_position(sail_hub)
    ctx.check(
        "hub rotation keeps the rotating assembly centered on the shaft axis",
        sail_pos is not None
        and spun_sail_pos is not None
        and _distance(sail_pos, spun_sail_pos) <= 1e-6,
        details=f"rest={sail_pos}, spun={spun_sail_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
