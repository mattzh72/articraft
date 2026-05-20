from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


Point3 = tuple[float, float, float]


def _mid(a: Point3, b: Point3) -> Point3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _dist(a: Point3, b: Point3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _cyl_rpy(a: Point3, b: Point3) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horizontal = math.hypot(dx, dy)
    return (0.0, math.atan2(horizontal, dz), math.atan2(dy, dx))


def _member(part, a: Point3, b: Point3, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_dist(a, b)),
        origin=Origin(xyz=_mid(a, b), rpy=_cyl_rpy(a, b)),
        material=material,
        name=name,
    )


def _square_mast(
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
        _member(part, (x, y, bottom_z), (x, y, top_z), chord_radius, material)

    for z in levels:
        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            _member(part, (a[0], a[1], z), (b[0], b[1], z), brace_radius, material)

    for i in range(panels):
        z0 = levels[i]
        z1 = levels[i + 1]
        for side in range(4):
            x0, y0 = corners[side]
            x1, y1 = corners[(side + 1) % 4]
            _member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)

    ladder_x = half + 0.12
    rail_y = 0.36
    ladder_bottom = bottom_z + 0.55
    ladder_top = top_z - 0.65
    _member(part, (ladder_x, -rail_y, ladder_bottom), (ladder_x, -rail_y, ladder_top), 0.026, ladder_material)
    _member(part, (ladder_x, rail_y, ladder_bottom), (ladder_x, rail_y, ladder_top), 0.026, ladder_material)
    for i in range(36):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / 35
        _member(part, (ladder_x, -rail_y, z), (ladder_x, rail_y, z), 0.018, ladder_material)
    for z in levels[1:-1:2]:
        _member(part, (half, -rail_y, z), (ladder_x, -rail_y, z), 0.022, ladder_material)
        _member(part, (half, rail_y, z), (ladder_x, rail_y, z), 0.022, ladder_material)


def _triangular_jib(
    part,
    *,
    x0: float,
    x1: float,
    bottom_z: float,
    half_width: float,
    root_top_z: float,
    tip_top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, list[Point3]]:
    xs = [x0 + (x1 - x0) * i / panels for i in range(panels + 1)]

    def top_z(x: float) -> float:
        t = (x - x0) / (x1 - x0)
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_a = [(x, -half_width, bottom_z) for x in xs]
    lower_b = [(x, half_width, bottom_z) for x in xs]
    upper = [(x, 0.0, top_z(x)) for x in xs]

    for i in range(panels):
        _member(part, lower_a[i], lower_a[i + 1], chord_radius, material)
        _member(part, lower_b[i], lower_b[i + 1], chord_radius, material)
        _member(part, upper[i], upper[i + 1], chord_radius, material)

    for i in range(panels + 1):
        _member(part, lower_a[i], lower_b[i], brace_radius, material)
        _member(part, lower_a[i], upper[i], brace_radius, material)
        _member(part, lower_b[i], upper[i], brace_radius, material)

    for i in range(panels):
        if i % 2 == 0:
            _member(part, lower_a[i], upper[i + 1], brace_radius, material)
            _member(part, lower_b[i], upper[i + 1], brace_radius, material)
        else:
            _member(part, upper[i], lower_a[i + 1], brace_radius, material)
            _member(part, upper[i], lower_b[i + 1], brace_radius, material)

    return {"lower_a": lower_a, "lower_b": lower_b, "upper": upper, "xs": [(x, 0.0, bottom_z) for x in xs]}


def _hook_mesh():
    geom = tube_from_spline_points(
        [
            (0.00, 0.0, -1.02),
            (0.23, 0.0, -1.18),
            (0.30, 0.0, -1.55),
            (0.12, 0.0, -1.92),
            (-0.22, 0.0, -2.00),
            (-0.43, 0.0, -1.75),
            (-0.35, 0.0, -1.42),
        ],
        radius=0.055,
        samples_per_segment=20,
        radial_segments=20,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, "red_tower_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="realistic_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.94, 0.76, 0.08, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.59, 1.0))
    dark = model.material("machinery_dark", rgba=(0.12, 0.14, 0.16, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.58, 0.57, 0.54, 1.0))
    cable = model.material("black_cable", rgba=(0.03, 0.035, 0.04, 1.0))
    ballast_mat = model.material("concrete_ballast", rgba=(0.45, 0.46, 0.44, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.45, 0.68, 0.82, 0.48))
    safety_red = model.material("safety_red", rgba=(0.78, 0.05, 0.03, 1.0))
    safety_white = model.material("safety_white", rgba=(0.94, 0.93, 0.88, 1.0))

    hook_mesh = _hook_mesh()

    mast = model.part("foundation_mast")
    mast.visual(Box((8.0, 8.0, 1.0)), origin=Origin(xyz=(0.0, 0.0, 0.5)), material=concrete, name="foundation_slab")
    mast.visual(Box((3.6, 3.6, 1.2)), origin=Origin(xyz=(0.0, 0.0, 1.6)), material=concrete, name="mast_pedestal")
    mast.visual(Box((3.05, 3.05, 0.20)), origin=Origin(xyz=(0.0, 0.0, 2.30)), material=dark, name="base_shoe")
    for sx in (-1.2, 1.2):
        for sy in (-1.2, 1.2):
            mast.visual(Cylinder(radius=0.12, length=0.48), origin=Origin(xyz=(sx, sy, 2.36)), material=steel)
    for x in (-3.15, 3.15):
        mast.visual(Box((0.30, 1.9, 0.28)), origin=Origin(xyz=(x, -2.95, 1.14)), material=safety_red)
        mast.visual(Box((0.30, 1.9, 0.28)), origin=Origin(xyz=(x, 2.95, 1.14)), material=safety_white)
    _square_mast(
        mast,
        width=3.0,
        bottom_z=2.40,
        top_z=32.0,
        panels=10,
        chord_radius=0.105,
        brace_radius=0.060,
        material=crane_yellow,
        ladder_material=steel,
    )
    mast.visual(Cylinder(radius=1.85, length=0.30), origin=Origin(xyz=(0.0, 0.0, 32.15)), material=dark, name="slew_bearing")
    mast.inertial = Inertial.from_geometry(Box((8.0, 8.0, 32.3)), mass=120000.0, origin=Origin(xyz=(0.0, 0.0, 16.15)))

    upper = model.part("upperworks")
    upper.visual(Cylinder(radius=1.95, length=0.30), origin=Origin(xyz=(0.0, 0.0, 0.15)), material=dark, name="upper_slew_ring")
    upper.visual(Box((5.8, 3.0, 0.42)), origin=Origin(xyz=(0.10, 0.0, 0.51)), material=dark, name="machinery_deck")
    upper.visual(Box((2.2, 1.6, 1.2)), origin=Origin(xyz=(-1.35, 0.0, 1.20)), material=dark, name="machinery_house")
    upper.visual(Box((1.55, 1.10, 1.00)), origin=Origin(xyz=(1.55, -1.45, 1.18)), material=dark, name="operator_cab")
    upper.visual(Box((1.35, 0.08, 0.76)), origin=Origin(xyz=(1.60, -2.02, 1.22)), material=glass, name="cab_front_glass")
    upper.visual(Box((0.08, 0.85, 0.72)), origin=Origin(xyz=(2.34, -1.45, 1.22)), material=glass, name="cab_side_glass")

    apex = (0.0, 0.0, 7.25)
    upper.visual(Box((0.55, 0.55, 0.45)), origin=Origin(xyz=apex), material=crane_yellow, name="tower_head")
    _member(upper, (-0.75, -0.75, 0.72), apex, 0.11, crane_yellow)
    _member(upper, (-0.75, 0.75, 0.72), apex, 0.11, crane_yellow)
    _member(upper, (0.75, -0.75, 0.72), apex, 0.11, crane_yellow)
    _member(upper, (0.75, 0.75, 0.72), apex, 0.11, crane_yellow)

    jib = _triangular_jib(
        upper,
        x0=2.30,
        x1=46.0,
        bottom_z=1.16,
        half_width=0.82,
        root_top_z=4.20,
        tip_top_z=2.05,
        panels=13,
        chord_radius=0.095,
        brace_radius=0.052,
        material=crane_yellow,
    )
    _member(upper, (1.15, -0.85, 0.72), jib["lower_a"][0], 0.075, crane_yellow)
    _member(upper, (1.15, 0.85, 0.72), jib["lower_b"][0], 0.075, crane_yellow)
    _member(upper, (0.55, 0.0, 3.7), jib["upper"][0], 0.075, crane_yellow)
    _member(upper, apex, jib["upper"][4], 0.035, cable)
    _member(upper, apex, jib["upper"][8], 0.032, cable)
    _member(upper, apex, jib["upper"][-1], 0.030, cable)

    rail_z = 0.93
    _member(upper, (3.20, -0.32, rail_z), (44.0, -0.32, rail_z), 0.060, steel, name="travel_rail_0")
    _member(upper, (3.20, 0.32, rail_z), (44.0, 0.32, rail_z), 0.060, steel, name="travel_rail_1")
    for p in jib["lower_a"][1:-1:2]:
        _member(upper, (p[0], -0.32, rail_z), p, 0.035, steel)
    for p in jib["lower_b"][1:-1:2]:
        _member(upper, (p[0], 0.32, rail_z), p, 0.035, steel)

    counter = _triangular_jib(
        upper,
        x0=-2.10,
        x1=-16.2,
        bottom_z=1.12,
        half_width=0.78,
        root_top_z=4.05,
        tip_top_z=2.20,
        panels=5,
        chord_radius=0.090,
        brace_radius=0.052,
        material=crane_yellow,
    )
    _member(upper, (-1.1, -0.85, 0.72), counter["lower_a"][0], 0.075, crane_yellow)
    _member(upper, (-1.1, 0.85, 0.72), counter["lower_b"][0], 0.075, crane_yellow)
    _member(upper, (-0.55, 0.0, 3.6), counter["upper"][0], 0.075, crane_yellow)
    _member(upper, apex, counter["upper"][-1], 0.035, cable)
    _member(upper, apex, counter["upper"][3], 0.033, cable)
    upper.visual(Box((8.6, 1.55, 0.20)), origin=Origin(xyz=(-11.9, 0.0, 0.85)), material=steel, name="ballast_walkway")
    for i, x in enumerate((-9.4, -10.9, -12.4, -13.9, -15.0)):
        height = 1.05 if i < 4 else 0.82
        upper.visual(Box((1.35, 1.35, height)), origin=Origin(xyz=(x, 0.0, 0.85 + height * 0.5)), material=ballast_mat, name=f"ballast_{i}")

    for x in (10.0, 22.0, 34.0, 44.0):
        upper.visual(Box((0.55, 0.10, 0.16)), origin=Origin(xyz=(x, -0.96, 1.05)), material=safety_red)
        upper.visual(Box((0.55, 0.10, 0.16)), origin=Origin(xyz=(x, 0.96, 1.05)), material=safety_white)

    upper.inertial = Inertial.from_geometry(Box((62.0, 5.0, 7.6)), mass=48000.0, origin=Origin(xyz=(13.0, 0.0, 2.8)))

    trolley = model.part("trolley")
    trolley.visual(Box((1.35, 1.05, 0.32)), origin=Origin(xyz=(0.0, 0.0, -0.40)), material=dark, name="trolley_frame")
    trolley.visual(Box((1.05, 0.72, 0.44)), origin=Origin(xyz=(0.0, 0.0, -0.76)), material=steel, name="winch_box")
    for y in (-0.32, 0.32):
        trolley.visual(
            Cylinder(radius=0.13, length=0.30),
            origin=Origin(xyz=(-0.38, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"roller_{'a' if y < 0 else 'b'}_0",
        )
        trolley.visual(
            Cylinder(radius=0.13, length=0.30),
            origin=Origin(xyz=(0.38, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"roller_{'a' if y < 0 else 'b'}_1",
        )
    for x in (-0.50, 0.50):
        _member(trolley, (x, -0.46, -0.24), (x, -0.46, 0.05), 0.035, steel)
        _member(trolley, (x, 0.46, -0.24), (x, 0.46, 0.05), 0.035, steel)
    trolley.visual(Cylinder(radius=0.18, length=0.88), origin=Origin(xyz=(0.0, 0.0, -0.69), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark, name="cable_drum")
    trolley.visual(Cylinder(radius=0.025, length=12.70), origin=Origin(xyz=(0.0, -0.18, -6.65)), material=cable, name="hoist_cable_0")
    trolley.visual(Cylinder(radius=0.025, length=12.70), origin=Origin(xyz=(0.0, 0.18, -6.65)), material=cable, name="hoist_cable_1")
    trolley.inertial = Inertial.from_geometry(Box((1.4, 1.1, 13.0)), mass=2500.0, origin=Origin(xyz=(0.0, 0.0, -6.2)))

    hook = model.part("hook_block")
    hook.visual(Box((0.95, 0.72, 0.18)), origin=Origin(xyz=(0.0, 0.0, -0.09)), material=crane_yellow, name="top_plate")
    hook.visual(Box((0.72, 0.50, 0.78)), origin=Origin(xyz=(0.0, 0.0, -0.54)), material=crane_yellow, name="sheave_case")
    hook.visual(Cylinder(radius=0.24, length=0.62), origin=Origin(xyz=(0.0, 0.0, -0.48), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="sheave")
    hook.visual(Cylinder(radius=0.07, length=0.35), origin=Origin(xyz=(0.0, 0.0, -0.87)), material=steel, name="swivel_shank")
    _member(hook, (-0.28, 0.0, -0.83), (0.28, 0.0, -0.83), 0.070, steel)
    hook.visual(hook_mesh, material=safety_red, name="hook")
    hook.inertial = Inertial.from_geometry(Box((1.0, 0.8, 2.2)), mass=1800.0, origin=Origin(xyz=(0.0, 0.0, -0.85)))

    model.articulation(
        "slewing_rotation",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 32.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.18),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=trolley,
        origin=Origin(xyz=(6.0, 0.0, 0.74)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=1.2, lower=0.0, upper=34.0),
    )
    model.articulation(
        "hook_suspension",
        ArticulationType.FIXED,
        parent=trolley,
        child=hook,
        origin=Origin(xyz=(0.0, 0.0, -13.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("foundation_mast")
    upper = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    hook = object_model.get_part("hook_block")
    slew = object_model.get_articulation("slewing_rotation")
    trolley_joint = object_model.get_articulation("trolley_travel")

    ctx.expect_gap(
        upper,
        mast,
        axis="z",
        positive_elem="upper_slew_ring",
        negative_elem="slew_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="slewing ring sits on mast bearing",
    )
    ctx.expect_gap(
        trolley,
        hook,
        axis="z",
        positive_elem="hoist_cable_0",
        negative_elem="top_plate",
        max_gap=0.002,
        max_penetration=0.0,
        name="hook block hangs from hoist cable",
    )
    ctx.expect_overlap(trolley, hook, axes="xy", elem_a="hoist_cable_0", elem_b="top_plate", min_overlap=0.02, name="cable lands over hook block")

    rest_pos = ctx.part_world_position(trolley)
    with ctx.pose({trolley_joint: 34.0}):
        extended_pos = ctx.part_world_position(trolley)
    ctx.check(
        "trolley travels outward along the jib",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 30.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slew: math.pi / 2.0}):
        slewed_pos = ctx.part_world_position(trolley)
    ctx.check(
        "slewing rotation carries jib around the tower",
        slewed_pos is not None and abs(slewed_pos[1]) > 5.5 and abs(slewed_pos[0]) < 0.5,
        details=f"slewed={slewed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
