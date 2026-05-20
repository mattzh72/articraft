from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from typing import Callable

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

MAST_HEIGHT = 24.0
MAST_WIDTH = 1.8
SLEW_Z = 24.25
REST_JIB_ANGLE = 0.78
JIB_LENGTH = 22.0


def _midpoint(a: Point3, b: Point3) -> Point3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: Point3, b: Point3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(a: Point3, b: Point3) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(horizontal, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a: Point3, b: Point3, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def _pitch_up(point: Point3, angle: float) -> Point3:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x - s * z, y, s * x + c * z)


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.18, 0.0, -5.95),
            (0.34, 0.0, -6.18),
            (0.38, 0.0, -6.52),
            (0.18, 0.0, -6.86),
            (-0.12, 0.0, -6.96),
            (-0.34, 0.0, -6.72),
            (-0.28, 0.0, -6.36),
        ],
        radius=0.065,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "luffer_crane_hook")


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
) -> None:
    half = width * 0.5
    corners = [
        (half, half),
        (half, -half),
        (-half, -half),
        (-half, half),
    ]
    levels = [bottom_z + (top_z - bottom_z) * i / panels for i in range(panels + 1)]

    for x, y in corners:
        _add_member(part, (x, y, bottom_z), (x, y, top_z), chord_radius, material)

    for z in levels:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), brace_radius, material)

    for i in range(panels):
        z0 = levels[i]
        z1 = levels[i + 1]
        for face in range(4):
            x0, y0 = corners[face]
            x1, y1 = corners[(face + 1) % 4]
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)

        _add_member(part, (half, half, z0), (-half, -half, z1), brace_radius * 0.92, material)
        _add_member(part, (half, -half, z0), (-half, half, z1), brace_radius * 0.92, material)


def _add_triangular_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    bottom_z: float,
    root_half_width: float,
    tip_half_width: float,
    root_top_z: float,
    tip_top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
    transform: Callable[[Point3], Point3] | None = None,
) -> dict[str, list[Point3]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    span = x_end - x_start if abs(x_end - x_start) > 1e-9 else 1.0

    def _half_width(x: float) -> float:
        t = (x - x_start) / span
        return root_half_width + (tip_half_width - root_half_width) * t

    def _top_z(x: float) -> float:
        t = (x - x_start) / span
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_left = [(x, -_half_width(x), bottom_z) for x in xs]
    lower_right = [(x, _half_width(x), bottom_z) for x in xs]
    upper = [(x, 0.0, _top_z(x)) for x in xs]

    if transform is not None:
        lower_left = [transform(p) for p in lower_left]
        lower_right = [transform(p) for p in lower_right]
        upper = [transform(p) for p in upper]

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luffer_jib_tower_crane")

    tower_yellow = model.material("tower_yellow", rgba=(0.91, 0.76, 0.15, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    concrete = model.material("concrete", rgba=(0.62, 0.62, 0.60, 1.0))
    ballast = model.material("ballast", rgba=(0.48, 0.48, 0.47, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.62, 0.79, 0.88, 0.42))
    hook_red = model.material("hook_red", rgba=(0.76, 0.12, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    mast = model.part("mast")
    mast.visual(
        Box((6.0, 6.0, 0.85)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=concrete,
        name="foundation_pad",
    )
    mast.visual(
        Box((3.2, 3.2, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=dark_grey,
        name="base_plinth",
    )
    mast.visual(
        Box((2.2, 2.2, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 1.425)),
        material=steel,
        name="tower_shoe",
    )
    _add_square_lattice_mast(
        mast,
        width=MAST_WIDTH,
        bottom_z=1.60,
        top_z=MAST_HEIGHT,
        panels=12,
        chord_radius=0.090,
        brace_radius=0.048,
        material=tower_yellow,
    )
    mast.visual(
        Box((2.0, 2.0, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 23.90)),
        material=dark_grey,
        name="mast_head_frame",
    )
    mast.visual(
        Cylinder(radius=1.15, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, SLEW_Z - 0.125)),
        material=dark_grey,
        name="mast_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Box((6.0, 6.0, SLEW_Z)),
        mass=26000.0,
        origin=Origin(xyz=(0.0, 0.0, SLEW_Z * 0.5)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.18, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_grey,
        name="slew_ring_upper",
    )
    upperworks.visual(
        Cylinder(radius=0.62, length=0.65),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=dark_grey,
        name="slew_pedestal",
    )
    upperworks.visual(
        Box((4.0, 2.6, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
        material=dark_grey,
        name="machinery_deck",
    )
    upperworks.visual(
        Box((2.4, 1.7, 1.65)),
        origin=Origin(xyz=(-1.45, 0.0, 1.82)),
        material=dark_grey,
        name="machinery_house",
    )
    upperworks.visual(
        Box((1.45, 1.05, 1.10)),
        origin=Origin(xyz=(0.95, -1.05, 1.54)),
        material=dark_grey,
        name="cab_shell",
    )
    upperworks.visual(
        Box((0.95, 1.00, 0.86)),
        origin=Origin(xyz=(1.12, -1.10, 1.56)),
        material=cab_glass,
        name="cab_glazing",
    )
    upperworks.visual(
        Box((0.22, 0.24, 0.78)),
        origin=Origin(xyz=(0.94, -0.42, 1.50)),
        material=steel,
        name="hinge_cheek_left",
    )
    upperworks.visual(
        Box((0.22, 0.24, 0.78)),
        origin=Origin(xyz=(0.94, 0.42, 1.50)),
        material=steel,
        name="hinge_cheek_right",
    )

    counter_jib = _add_triangular_truss(
        upperworks,
        x_start=-0.65,
        x_end=-8.60,
        bottom_z=1.15,
        root_half_width=0.68,
        tip_half_width=0.42,
        root_top_z=2.35,
        tip_top_z=1.45,
        panels=5,
        chord_radius=0.075,
        brace_radius=0.040,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((7.6, 1.05, 0.10)),
        origin=Origin(xyz=(-4.55, 0.0, 1.08)),
        material=steel,
        name="counter_jib_deck",
    )
    upperworks.visual(
        Box((1.40, 1.40, 1.00)),
        origin=Origin(xyz=(-5.45, 0.0, 1.65)),
        material=ballast,
        name="ballast_pack_a",
    )
    upperworks.visual(
        Box((1.40, 1.40, 1.00)),
        origin=Origin(xyz=(-6.95, 0.0, 1.65)),
        material=ballast,
        name="ballast_pack_b",
    )
    upperworks.visual(
        Box((1.10, 1.20, 0.85)),
        origin=Origin(xyz=(-8.00, 0.0, 1.58)),
        material=ballast,
        name="ballast_pack_c",
    )

    apex = (0.10, 0.0, 5.15)
    _add_member(upperworks, (-0.95, -0.82, 1.14), apex, 0.085, tower_yellow)
    _add_member(upperworks, (-0.95, 0.82, 1.14), apex, 0.085, tower_yellow)
    _add_member(upperworks, (0.90, -0.52, 1.18), apex, 0.070, tower_yellow)
    _add_member(upperworks, (0.90, 0.52, 1.18), apex, 0.070, tower_yellow)
    _add_member(upperworks, (-0.28, -0.42, 4.10), (-0.28, 0.42, 4.10), 0.050, tower_yellow)
    upperworks.visual(
        Box((0.48, 0.48, 0.42)),
        origin=Origin(xyz=apex),
        material=tower_yellow,
        name="head_apex",
    )
    upperworks.inertial = Inertial.from_geometry(
        Box((12.0, 4.0, 5.8)),
        mass=8500.0,
        origin=Origin(xyz=(-2.6, 0.0, 2.20)),
    )

    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.14, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    jib.visual(
        Box((1.65, 0.40, 0.30)),
        origin=Origin(xyz=(1.55, 0.0, 0.70), rpy=(0.0, REST_JIB_ANGLE, 0.0)),
        material=steel,
        name="root_heel",
    )
    jib_nodes = _add_triangular_truss(
        jib,
        x_start=2.60,
        x_end=22.80,
        bottom_z=0.0,
        root_half_width=0.68,
        tip_half_width=0.36,
        root_top_z=2.20,
        tip_top_z=0.85,
        panels=10,
        chord_radius=0.076,
        brace_radius=0.041,
        material=tower_yellow,
        transform=lambda point: _pitch_up(point, REST_JIB_ANGLE),
    )
    _add_member(jib, (0.14, -0.20, 0.0), jib_nodes["lower_left"][0], 0.055, steel, name="heel_link_left")
    _add_member(jib, (0.14, 0.20, 0.0), jib_nodes["lower_right"][0], 0.055, steel, name="heel_link_right")
    _add_member(jib, (0.10, 0.0, 0.10), jib_nodes["upper"][0], 0.050, steel, name="heel_link_top")
    _add_member(jib, (1.80, -0.18, 0.54), jib_nodes["lower_left"][1], 0.046, tower_yellow)
    _add_member(jib, (1.80, 0.18, 0.54), jib_nodes["lower_right"][1], 0.046, tower_yellow)
    _add_member(jib, (1.95, 0.0, 0.92), jib_nodes["upper"][1], 0.043, tower_yellow)

    tip_anchor = _midpoint(jib_nodes["lower_left"][-1], jib_nodes["lower_right"][-1])
    jib.visual(
        Box((0.52, 0.86, 0.12)),
        origin=Origin(xyz=(tip_anchor[0], 0.0, tip_anchor[2] + 0.06)),
        material=steel,
        name="jib_tip_head",
    )
    _add_member(
        jib,
        jib_nodes["upper"][-2],
        (tip_anchor[0], 0.0, tip_anchor[2] + 0.22),
        0.045,
        tower_yellow,
        name="tip_bridle",
    )
    jib.visual(
        Cylinder(radius=0.16, length=0.92),
        origin=Origin(xyz=(tip_anchor[0], 0.0, tip_anchor[2] + 0.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tip_sheave",
    )
    jib.inertial = Inertial.from_geometry(
        Box((18.5, 2.2, 15.0)),
        mass=4200.0,
        origin=Origin(xyz=(8.3, 0.0, 6.0)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.030, length=0.18),
        origin=Origin(xyz=(-0.10, 0.0, -0.09)),
        material=steel,
        name="left_hanger_link",
    )
    hook_block.visual(
        Cylinder(radius=0.030, length=0.18),
        origin=Origin(xyz=(0.10, 0.0, -0.09)),
        material=steel,
        name="right_hanger_link",
    )
    hook_block.visual(
        Box((0.22, 0.26, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=steel,
        name="hanger_yoke",
    )
    hook_block.visual(
        Cylinder(radius=0.032, length=4.14),
        origin=Origin(xyz=(-0.10, 0.0, -2.25)),
        material=steel,
        name="left_fall",
    )
    hook_block.visual(
        Cylinder(radius=0.032, length=4.14),
        origin=Origin(xyz=(0.10, 0.0, -2.25)),
        material=steel,
        name="right_fall",
    )
    hook_block.visual(
        Box((0.78, 0.40, 0.52)),
        origin=Origin(xyz=(0.0, 0.0, -4.39)),
        material=tower_yellow,
        name="upper_block",
    )
    hook_block.visual(
        Box((0.58, 0.34, 1.04)),
        origin=Origin(xyz=(0.0, 0.0, -5.17)),
        material=tower_yellow,
        name="lower_block",
    )
    _add_member(hook_block, (0.0, 0.0, -5.69), (0.0, 0.0, -5.94), 0.065, steel, name="hook_stem")
    _add_member(hook_block, (0.0, 0.0, -5.94), (0.16, 0.0, -6.16), 0.060, hook_red, name="hook_arc_1")
    _add_member(hook_block, (0.16, 0.0, -6.16), (0.28, 0.0, -6.48), 0.060, hook_red, name="hook_arc_2")
    _add_member(hook_block, (0.28, 0.0, -6.48), (0.12, 0.0, -6.78), 0.060, hook_red, name="hook_arc_3")
    _add_member(hook_block, (0.12, 0.0, -6.78), (-0.16, 0.0, -6.86), 0.058, hook_red, name="hook_arc_4")
    _add_member(hook_block, (-0.16, 0.0, -6.86), (-0.28, 0.0, -6.60), 0.050, hook_red, name="hook_tip")
    hook_block.inertial = Inertial.from_geometry(
        Box((1.0, 0.5, 7.0)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, -3.50)),
    )

    model.articulation(
        "mast_to_upperworks",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, SLEW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.35),
    )
    model.articulation(
        "upperworks_to_jib",
        ArticulationType.REVOLUTE,
        parent=upperworks,
        child=jib,
        origin=Origin(xyz=(1.05, 0.0, 1.32)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180000.0,
            velocity=0.22,
            lower=-0.43,
            upper=0.52,
        ),
    )
    model.articulation(
        "jib_to_hook_block",
        ArticulationType.FIXED,
        parent=jib,
        child=hook_block,
        origin=Origin(xyz=tip_anchor),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upperworks = object_model.get_part("upperworks")
    jib = object_model.get_part("jib")
    hook_block = object_model.get_part("hook_block")
    slew = object_model.get_articulation("mast_to_upperworks")
    luff = object_model.get_articulation("upperworks_to_jib")

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

    ctx.expect_gap(
        upperworks,
        mast,
        axis="z",
        positive_elem="slew_ring_upper",
        negative_elem="mast_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="slewing ring seats on mast cap",
    )
    ctx.expect_contact(
        jib,
        upperworks,
        elem_a="hinge_barrel",
        elem_b="hinge_cheek_left",
        name="left hinge cheek captures jib barrel",
    )
    ctx.expect_contact(
        jib,
        upperworks,
        elem_a="hinge_barrel",
        elem_b="hinge_cheek_right",
        name="right hinge cheek captures jib barrel",
    )
    ctx.expect_contact(
        hook_block,
        jib,
        elem_a="left_hanger_link",
        elem_b="jib_tip_head",
        contact_tol=0.015,
        name="left hanger link seats under the jib tip",
    )
    ctx.expect_contact(
        hook_block,
        jib,
        elem_a="right_hanger_link",
        elem_b="jib_tip_head",
        contact_tol=0.015,
        name="right hanger link seats under the jib tip",
    )

    tip_lower_z = None
    tip_upper_z = None
    luff_lower = luff.motion_limits.lower if luff.motion_limits is not None else None
    luff_upper = luff.motion_limits.upper if luff.motion_limits is not None else None
    if luff_lower is not None:
        with ctx.pose({luff: luff_lower}):
            tip_box = ctx.part_element_world_aabb(jib, elem="jib_tip_head")
            if tip_box is not None:
                tip_lower_z = tip_box[1][2]
    if luff_upper is not None:
        with ctx.pose({luff: luff_upper}):
            tip_box = ctx.part_element_world_aabb(jib, elem="jib_tip_head")
            if tip_box is not None:
                tip_upper_z = tip_box[1][2]
    ctx.check(
        "positive luff raises jib tip",
        tip_lower_z is not None and tip_upper_z is not None and tip_upper_z > tip_lower_z + 8.0,
        details=f"lower_tip_z={tip_lower_z}, upper_tip_z={tip_upper_z}",
    )

    hook_rest = ctx.part_world_position(hook_block)
    with ctx.pose({slew: math.pi / 2.0}):
        hook_quarter_turn = ctx.part_world_position(hook_block)
    ctx.check(
        "slewing rotates hook around mast",
        hook_rest is not None
        and hook_quarter_turn is not None
        and hook_rest[0] > 10.0
        and abs(hook_rest[1]) < 0.5
        and hook_quarter_turn[1] > 10.0
        and abs(hook_quarter_turn[0]) < 2.0,
        details=f"rest={hook_rest}, quarter_turn={hook_quarter_turn}",
    )

    tip_box = ctx.part_element_world_aabb(jib, elem="jib_tip_head")
    hook_body_box = ctx.part_element_world_aabb(hook_block, elem="upper_block")
    ctx.check(
        "hook block body hangs below tip sheave",
        tip_box is not None
        and hook_body_box is not None
        and hook_body_box[1][2] < tip_box[0][2] - 3.5,
        details=f"tip_box={tip_box}, hook_body_box={hook_body_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
