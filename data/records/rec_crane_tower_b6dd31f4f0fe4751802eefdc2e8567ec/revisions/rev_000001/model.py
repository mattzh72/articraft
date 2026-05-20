from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    run = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(run, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None, extend: float = 0.02) -> None:
    """Add a round lattice member. A tiny endpoint extension welds nodes visually."""
    length = _distance(a, b)
    if length <= 1e-9:
        return
    if extend > 0.0:
        ux = (b[0] - a[0]) / length
        uy = (b[1] - a[1]) / length
        uz = (b[2] - a[2]) / length
        a = (a[0] - ux * extend, a[1] - uy * extend, a[2] - uz * extend)
        b = (b[0] + ux * extend, b[1] + uy * extend, b[2] + uz * extend)
        length += 2.0 * extend
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_square_lattice_mast(
    part,
    *,
    width: float,
    bottom_z: float,
    top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
    ladder_material,
) -> None:
    half = width * 0.5
    corners = [(half, half), (half, -half), (-half, -half), (-half, half)]
    levels = [bottom_z + (top_z - bottom_z) * i / panels for i in range(panels + 1)]

    for x, y in corners:
        _add_member(part, (x, y, bottom_z), (x, y, top_z), chord_radius, material)

    for z in levels:
        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            _add_member(part, (a[0], a[1], z), (b[0], b[1], z), brace_radius, material)

    for i in range(panels):
        z0 = levels[i]
        z1 = levels[i + 1]
        for j in range(4):
            a = corners[j]
            b = corners[(j + 1) % 4]
            _add_member(part, (a[0], a[1], z0), (b[0], b[1], z1), brace_radius, material)
            _add_member(part, (b[0], b[1], z0), (a[0], a[1], z1), brace_radius, material)

    # Service ladder on one mast face, tied back to the lattice by small standoffs.
    ladder_x = half + 0.10
    rail_y = 0.28
    ladder_bottom = bottom_z + 0.45
    ladder_top = top_z - 0.55
    _add_member(part, (ladder_x, -rail_y, ladder_bottom), (ladder_x, -rail_y, ladder_top), 0.030, ladder_material)
    _add_member(part, (ladder_x, rail_y, ladder_bottom), (ladder_x, rail_y, ladder_top), 0.030, ladder_material)
    rung_count = 28
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(part, (ladder_x, -rail_y, z), (ladder_x, rail_y, z), 0.022, ladder_material, extend=0.0)
    for z in levels[1:-1:2]:
        _add_member(part, (half, -0.45, z), (ladder_x, -rail_y, z), 0.025, ladder_material, extend=0.0)
        _add_member(part, (half, 0.45, z), (ladder_x, rail_y, z), 0.025, ladder_material, extend=0.0)


def _add_triangular_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    bottom_z: float,
    half_width: float,
    root_top_z: float,
    tip_top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, list[tuple[float, float, float]]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    span = x_end - x_start

    def top_z(x: float) -> float:
        t = 0.0 if abs(span) < 1e-9 else (x - x_start) / span
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_left = [(x, -half_width, bottom_z) for x in xs]
    lower_right = [(x, half_width, bottom_z) for x in xs]
    upper = [(x, 0.0, top_z(x)) for x in xs]

    for i in range(panels):
        _add_member(part, lower_left[i], lower_left[i + 1], chord_radius, material)
        _add_member(part, lower_right[i], lower_right[i + 1], chord_radius, material)
        _add_member(part, upper[i], upper[i + 1], chord_radius, material)

    for i in range(panels + 1):
        _add_member(part, lower_left[i], lower_right[i], brace_radius, material)
        _add_member(part, lower_left[i], upper[i], brace_radius, material)
        _add_member(part, lower_right[i], upper[i], brace_radius, material)

    for i in range(panels):
        if i % 2 == 0:
            _add_member(part, lower_left[i], upper[i + 1], brace_radius, material)
            _add_member(part, lower_right[i], upper[i + 1], brace_radius, material)
        else:
            _add_member(part, upper[i], lower_left[i + 1], brace_radius, material)
            _add_member(part, upper[i], lower_right[i + 1], brace_radius, material)

    return {"lower_left": lower_left, "lower_right": lower_right, "upper": upper}


def _add_luffing_jib(part, *, material, steel, cable) -> tuple[float, float]:
    """Triangular luffing jib in the child frame; local origin is the hinge axis."""
    panels = 9
    x0 = 0.45
    tip_x = 19.0
    rise = 7.6
    slope = rise / tip_x
    xs = [x0 + (tip_x - x0) * i / panels for i in range(panels + 1)]

    def base_z(x: float) -> float:
        return slope * x

    def truss_h(x: float) -> float:
        t = (x - x0) / (tip_x - x0)
        return 1.55 * (1.0 - t) + 0.55 * t

    half_width = 0.48
    lower_left = [(x, -half_width, base_z(x)) for x in xs]
    lower_right = [(x, half_width, base_z(x)) for x in xs]
    upper = [(x, 0.0, base_z(x) + truss_h(x)) for x in xs]

    # The hinge sleeve is intentionally part of the jib, concentric with the head pin.
    part.visual(
        Cylinder(radius=0.23, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    _add_member(part, (0.28, -0.42, 0.0), lower_left[0], 0.080, material, name="root_strut_0")
    _add_member(part, (0.28, 0.42, 0.0), lower_right[0], 0.080, material, name="root_strut_1")
    _add_member(part, (0.22, 0.0, 0.18), upper[0], 0.074, material, name="root_strut_2")

    for i in range(panels):
        _add_member(part, lower_left[i], lower_left[i + 1], 0.095, material)
        _add_member(part, lower_right[i], lower_right[i + 1], 0.095, material)
        _add_member(part, upper[i], upper[i + 1], 0.085, material)

    for i in range(panels + 1):
        _add_member(part, lower_left[i], lower_right[i], 0.055, material)
        _add_member(part, lower_left[i], upper[i], 0.060, material)
        _add_member(part, lower_right[i], upper[i], 0.060, material)

    for i in range(panels):
        _add_member(part, lower_left[i], upper[i + 1], 0.052, material)
        _add_member(part, lower_right[i], upper[i + 1], 0.052, material)
        _add_member(part, upper[i], lower_left[i + 1], 0.052, material)
        _add_member(part, upper[i], lower_right[i + 1], 0.052, material)

    tip_z = base_z(tip_x)
    part.visual(
        Cylinder(radius=0.26, length=1.05),
        origin=Origin(xyz=(tip_x, 0.0, tip_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tip_sheave",
    )
    part.visual(
        Box((0.70, 0.82, 0.20)),
        origin=Origin(xyz=(tip_x - 0.16, 0.0, tip_z + 0.02)),
        material=material,
        name="tip_yoke",
    )

    # Pendant lines running with the jib give the luffer its characteristic silhouette.
    _add_member(part, (0.15, -0.12, 1.85), upper[4], 0.032, cable, name="pendant_line_0", extend=0.0)
    _add_member(part, (0.15, 0.12, 1.85), upper[-1], 0.032, cable, name="pendant_line_1", extend=0.0)
    return (tip_x, tip_z)


def _build_hook_mesh():
    geom = tube_from_spline_points(
        [
            (0.15, 0.0, -0.58),
            (0.29, 0.0, -0.74),
            (0.28, 0.0, -0.98),
            (0.10, 0.0, -1.16),
            (-0.17, 0.0, -1.16),
            (-0.33, 0.0, -0.97),
            (-0.26, 0.0, -0.77),
        ],
        radius=0.050,
        samples_per_segment=20,
        radial_segments=20,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, "luffer_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luffer_jib_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.74, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    galvanized = model.material("galvanized", rgba=(0.58, 0.60, 0.61, 1.0))
    concrete = model.material("concrete", rgba=(0.58, 0.57, 0.54, 1.0))
    ballast = model.material("ballast", rgba=(0.44, 0.44, 0.42, 1.0))
    cable_mat = model.material("black_cable", rgba=(0.045, 0.045, 0.050, 1.0))
    glass = model.material("blue_glass", rgba=(0.46, 0.68, 0.82, 0.42))
    hook_red = model.material("hook_red", rgba=(0.78, 0.08, 0.05, 1.0))
    warning = model.material("warning_red", rgba=(0.83, 0.18, 0.10, 1.0))

    mast = model.part("mast")
    mast.visual(Box((5.4, 5.4, 0.34)), origin=Origin(xyz=(0.0, 0.0, 0.17)), material=concrete, name="foundation")
    mast.visual(Box((2.7, 2.7, 0.22)), origin=Origin(xyz=(0.0, 0.0, 0.45)), material=concrete, name="pedestal")
    _add_square_lattice_mast(
        mast,
        width=2.0,
        bottom_z=0.56,
        top_z=24.05,
        panels=9,
        chord_radius=0.105,
        brace_radius=0.060,
        material=crane_yellow,
        ladder_material=galvanized,
    )
    mast.visual(
        Cylinder(radius=1.38, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 24.20)),
        material=dark_steel,
        name="slew_bearing",
    )
    mast.visual(
        Cylinder(radius=0.82, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 23.90)),
        material=crane_yellow,
        name="mast_cap",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=1.32, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="upper_ring",
    )
    head.visual(Cylinder(radius=0.72, length=0.52), origin=Origin(xyz=(0.0, 0.0, 0.38)), material=crane_yellow)
    head.visual(Box((3.9, 1.05, 0.36)), origin=Origin(xyz=(-0.15, 0.0, 0.72)), material=dark_steel, name="machinery_deck")
    head.visual(Box((1.25, 0.72, 0.88)), origin=Origin(xyz=(-0.95, 0.0, 1.18)), material=dark_steel, name="machinery_house")
    head.visual(Box((0.74, 0.56, 0.58)), origin=Origin(xyz=(0.74, -0.62, 1.02)), material=dark_steel, name="operators_cab")
    head.visual(Box((0.60, 0.48, 0.48)), origin=Origin(xyz=(0.78, -0.62, 1.05)), material=glass, name="cab_glass")

    # Horizontal hinge structure for the luffing jib.
    head.visual(Box((0.40, 0.16, 1.25)), origin=Origin(xyz=(0.92, -0.70, 1.18)), material=crane_yellow, name="hinge_cheek_0")
    head.visual(Box((0.40, 0.16, 1.25)), origin=Origin(xyz=(0.92, 0.70, 1.18)), material=crane_yellow, name="hinge_cheek_1")
    head.visual(
        Cylinder(radius=0.155, length=1.58),
        origin=Origin(xyz=(0.92, 0.0, 1.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )

    # Rear counter-jib and ballast are part of the slewing head.
    counter = _add_triangular_truss(
        head,
        x_start=-0.55,
        x_end=-7.4,
        bottom_z=0.82,
        half_width=0.46,
        root_top_z=2.00,
        tip_top_z=1.25,
        panels=5,
        chord_radius=0.090,
        brace_radius=0.052,
        material=crane_yellow,
    )
    _add_member(head, (-0.40, -0.42, 0.78), counter["lower_left"][0], 0.070, crane_yellow)
    _add_member(head, (-0.40, 0.42, 0.78), counter["lower_right"][0], 0.070, crane_yellow)
    head.visual(Box((1.00, 0.88, 0.60)), origin=Origin(xyz=(-5.95, 0.0, 0.92)), material=ballast, name="ballast_0")
    head.visual(Box((1.00, 0.88, 0.60)), origin=Origin(xyz=(-6.95, 0.0, 0.92)), material=ballast, name="ballast_1")

    # A-frame and luffing machinery on the rotating head.
    apex = (-0.10, 0.0, 4.25)
    _add_member(head, (-0.95, -0.42, 0.90), apex, 0.100, crane_yellow, name="aframe_leg_0")
    _add_member(head, (-0.95, 0.42, 0.90), apex, 0.100, crane_yellow, name="aframe_leg_1")
    _add_member(head, (0.58, -0.42, 0.95), apex, 0.090, crane_yellow)
    _add_member(head, (0.58, 0.42, 0.95), apex, 0.090, crane_yellow)
    head.visual(Cylinder(radius=0.20, length=0.90), origin=Origin(xyz=(-1.70, 0.0, 1.08), rpy=(math.pi / 2.0, 0.0, 0.0)), material=galvanized, name="luffing_drum")
    _add_member(head, apex, counter["upper"][-1], 0.030, cable_mat, name="counter_pendant", extend=0.0)

    jib = model.part("jib")
    tip_x, tip_z = _add_luffing_jib(jib, material=crane_yellow, steel=galvanized, cable=cable_mat)
    # Red/white tip flags make the boom tip legible at scale.
    jib.visual(Box((0.36, 0.08, 0.46)), origin=Origin(xyz=(tip_x + 0.34, -0.28, tip_z + 0.16)), material=warning, name="tip_flag_0")
    jib.visual(Box((0.36, 0.08, 0.46)), origin=Origin(xyz=(tip_x + 0.34, 0.28, tip_z + 0.16)), material=warning, name="tip_flag_1")

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.030, length=2.84),
        origin=Origin(xyz=(0.0, -0.10, -1.68)),
        material=cable_mat,
        name="fall_line_0",
    )
    hook_block.visual(
        Cylinder(radius=0.030, length=2.84),
        origin=Origin(xyz=(0.0, 0.10, -1.68)),
        material=cable_mat,
        name="fall_line_1",
    )
    hook_block.visual(
        Cylinder(radius=0.080, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="top_shackle",
    )
    hook_block.visual(Box((0.72, 0.48, 0.34)), origin=Origin(xyz=(0.0, 0.0, -3.24)), material=crane_yellow, name="sheave_block")
    hook_block.visual(
        Cylinder(radius=0.15, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, -3.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="block_sheave",
    )
    hook_block.visual(
        Cylinder(radius=0.055, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -3.52)),
        material=galvanized,
        name="hook_stem",
    )
    hook_block.visual(mesh_from_geometry(tube_from_spline_points(
        [
            (0.15, 0.0, -3.58),
            (0.29, 0.0, -3.74),
            (0.28, 0.0, -3.98),
            (0.10, 0.0, -4.16),
            (-0.17, 0.0, -4.16),
            (-0.33, 0.0, -3.97),
            (-0.26, 0.0, -3.77),
        ],
        radius=0.050,
        samples_per_segment=20,
        radial_segments=20,
        up_hint=(0.0, 1.0, 0.0),
    ), "main_hook"), material=hook_red, name="hook")
    _add_member(
        hook_block,
        (0.0, 0.0, -3.38),
        (0.15, 0.0, -3.58),
        0.060,
        galvanized,
        name="hook_shank",
        extend=0.03,
    )

    model.articulation(
        "slewing",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 24.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.35),
    )
    model.articulation(
        "jib_luff",
        ArticulationType.REVOLUTE,
        parent=head,
        child=jib,
        origin=Origin(xyz=(0.92, 0.0, 1.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=0.18, lower=-0.26, upper=0.78),
    )
    model.articulation(
        "hook_hanger",
        ArticulationType.REVOLUTE,
        parent=jib,
        child=hook_block,
        origin=Origin(xyz=(tip_x, 0.0, tip_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.30, upper=0.85),
        mimic=Mimic("jib_luff", multiplier=1.0, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    jib = object_model.get_part("jib")
    hook = object_model.get_part("hook_block")
    luff = object_model.get_articulation("jib_luff")

    ctx.allow_overlap(
        head,
        jib,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The visible luffing hinge is modeled as a steel pin captured inside the jib sleeve.",
    )
    ctx.expect_overlap(
        head,
        jib,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.25,
        name="luffing hinge pin is captured by jib sleeve",
    )
    ctx.expect_gap(
        head,
        mast,
        axis="z",
        positive_elem="upper_ring",
        negative_elem="slew_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper slewing ring sits on mast bearing",
    )

    rest_pos = ctx.part_world_position(hook)
    with ctx.pose({luff: 0.65}):
        raised_pos = ctx.part_world_position(hook)
    ctx.check(
        "luffing jib raises hook point",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 7.0,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
