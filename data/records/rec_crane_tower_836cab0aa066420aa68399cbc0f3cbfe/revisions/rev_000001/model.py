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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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
        for j in range(4):
            x0, y0 = corners[j]
            x1, y1 = corners[(j + 1) % 4]
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)
        _add_member(part, (half, half, z0), (-half, -half, z1), brace_radius * 0.9, material)
        _add_member(part, (half, -half, z0), (-half, half, z1), brace_radius * 0.9, material)

    ladder_x = half + 0.09
    rail_y = 0.28
    ladder_bottom = bottom_z + 0.8
    ladder_top = top_z - 0.9
    _add_member(
        part,
        (ladder_x, -rail_y, ladder_bottom),
        (ladder_x, -rail_y, ladder_top),
        0.045,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, rail_y, ladder_bottom),
        (ladder_x, rail_y, ladder_top),
        0.045,
        ladder_material,
    )
    rung_count = 26
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(part, (ladder_x, -rail_y, z), (ladder_x, rail_y, z), 0.03, ladder_material)


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


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.28, 0.0, -1.10),
            (0.54, 0.0, -1.58),
            (0.60, 0.0, -2.16),
            (0.26, 0.0, -2.70),
            (-0.18, 0.0, -2.88),
            (-0.52, 0.0, -2.46),
            (-0.42, 0.0, -1.92),
        ],
        radius=0.10,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "luffing_tower_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luffing_jib_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.77, 0.14, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.28, 1.0))
    structural_steel = model.material("structural_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    ballast = model.material("ballast", rgba=(0.48, 0.48, 0.46, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.15, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.62, 0.78, 0.88, 0.45))
    safety_red = model.material("safety_red", rgba=(0.74, 0.16, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    mast = model.part("mast")
    mast.visual(
        Box((6.0, 6.0, 1.4)),
        origin=Origin(xyz=(0.0, 0.0, 0.7)),
        material=concrete,
        name="foundation_block",
    )
    mast.visual(
        Box((2.8, 2.8, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 1.9)),
        material=concrete,
    )
    mast.visual(
        Box((2.2, 2.2, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, 2.8)),
        material=dark_grey,
    )
    for sx in (-0.8, 0.8):
        for sy in (-0.8, 0.8):
            mast.visual(
                Cylinder(radius=0.10, length=0.9),
                origin=Origin(xyz=(sx, sy, 2.75)),
                material=structural_steel,
            )

    _add_square_lattice_mast(
        mast,
        width=2.2,
        bottom_z=3.2,
        top_z=26.2,
        panels=10,
        chord_radius=0.10,
        brace_radius=0.055,
        material=crane_yellow,
        ladder_material=galvanized,
    )
    mast.visual(
        Box((2.5, 2.5, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 26.325)),
        material=dark_grey,
        name="mast_head_platform",
    )
    mast.visual(
        Cylinder(radius=1.50, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 26.575)),
        material=dark_grey,
        name="mast_slewing_seat",
    )
    mast.inertial = Inertial.from_geometry(
        Box((6.0, 6.0, 26.9)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 13.45)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.55, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_grey,
        name="slewing_ring",
    )
    upperworks.visual(
        Box((5.2, 2.4, 0.45)),
        origin=Origin(xyz=(-0.75, 0.0, 0.675)),
        material=dark_grey,
        name="machinery_deck",
    )
    upperworks.visual(
        Box((1.6, 1.6, 1.6)),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=crane_yellow,
        name="slew_tower_core",
    )
    upperworks.visual(
        Box((2.0, 1.5, 1.5)),
        origin=Origin(xyz=(2.8, -1.95, 1.55)),
        material=dark_grey,
        name="cab_housing",
    )
    upperworks.visual(
        Box((1.7, 1.1, 1.2)),
        origin=Origin(xyz=(3.0, -1.98, 1.58)),
        material=cab_glass,
        name="cab_shell",
    )
    upperworks.visual(
        Box((1.8, 0.50, 0.28)),
        origin=Origin(xyz=(2.15, -1.45, 1.00)),
        material=structural_steel,
        name="cab_mount",
    )
    upperworks.visual(
        Box((2.4, 1.8, 1.6)),
        origin=Origin(xyz=(-2.8, 0.0, 1.60)),
        material=dark_grey,
        name="counter_machinery_house",
    )

    counter_jib = _add_triangular_truss(
        upperworks,
        x_start=-1.0,
        x_end=-8.2,
        bottom_z=1.45,
        half_width=0.70,
        root_top_z=3.00,
        tip_top_z=1.65,
        panels=5,
        chord_radius=0.10,
        brace_radius=0.055,
        material=crane_yellow,
    )
    upperworks.visual(
        Box((6.0, 0.48, 0.18)),
        origin=Origin(xyz=(-4.4, 0.0, 1.32)),
        material=structural_steel,
        name="counter_walkway",
    )
    upperworks.visual(
        Box((1.8, 1.4, 1.1)),
        origin=Origin(xyz=(-6.2, 0.0, 1.95)),
        material=ballast,
    )
    upperworks.visual(
        Box((1.8, 1.4, 1.1)),
        origin=Origin(xyz=(-7.9, 0.0, 1.95)),
        material=ballast,
    )

    upperworks.visual(
        Box((0.90, 1.20, 5.00)),
        origin=Origin(xyz=(-1.75, 0.0, 3.50)),
        material=crane_yellow,
        name="back_mast",
    )
    upperworks.visual(
        Box((1.20, 0.42, 0.36)),
        origin=Origin(xyz=(-1.75, 0.0, 6.10)),
        material=crane_yellow,
        name="back_mast_head",
    )
    upperworks.visual(
        Cylinder(radius=0.22, length=1.0),
        origin=Origin(xyz=(-1.75, 0.0, 6.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_steel,
        name="luffing_head_sheave",
    )
    upperworks.visual(
        Box((0.80, 0.95, 0.86)),
        origin=Origin(xyz=(1.10, 0.0, 2.32)),
        material=dark_grey,
        name="front_hinge_support",
    )
    upperworks.visual(
        Cylinder(radius=0.10, length=0.82),
        origin=Origin(xyz=(1.55, 0.0, 2.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_steel,
        name="hinge_pin",
    )
    upperworks.inertial = Inertial.from_geometry(
        Box((15.0, 3.2, 7.5)),
        mass=11000.0,
        origin=Origin(xyz=(-1.0, 0.0, 2.0)),
    )

    jib = model.part("luffing_jib")
    jib.visual(
        Box((1.60, 0.62, 0.42)),
        origin=Origin(xyz=(0.80, 0.0, 0.0)),
        material=dark_grey,
        name="root_knuckle",
    )
    jib.visual(
        Cylinder(radius=0.14, length=0.20),
        origin=Origin(xyz=(0.0, -0.25, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_steel,
        name="left_root_lug",
    )
    jib.visual(
        Cylinder(radius=0.14, length=0.20),
        origin=Origin(xyz=(0.0, 0.25, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_steel,
        name="right_root_lug",
    )

    jib_truss = _add_triangular_truss(
        jib,
        x_start=1.8,
        x_end=28.0,
        bottom_z=0.0,
        half_width=0.75,
        root_top_z=3.20,
        tip_top_z=1.20,
        panels=12,
        chord_radius=0.11,
        brace_radius=0.055,
        material=crane_yellow,
    )
    jib.visual(
        Box((25.6, 0.45, 0.18)),
        origin=Origin(xyz=(14.2, 0.0, -0.13)),
        material=structural_steel,
        name="travel_beam",
    )
    _add_member(jib, (1.8, -0.30, -0.02), (26.5, -0.30, -0.02), 0.045, structural_steel)
    _add_member(jib, (1.8, 0.30, -0.02), (26.5, 0.30, -0.02), 0.045, structural_steel)
    jib.visual(
        Cylinder(radius=0.45, length=0.35),
        origin=Origin(xyz=(27.9, 0.0, 1.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_steel,
        name="tip_pulley",
    )
    _add_member(jib, jib_truss["upper"][-2], (27.9, 0.0, 1.15), 0.08, crane_yellow)
    _add_member(jib, jib_truss["lower_left"][-2], (27.9, -0.18, 0.72), 0.06, crane_yellow)
    _add_member(jib, jib_truss["lower_right"][-2], (27.9, 0.18, 0.72), 0.06, crane_yellow)
    jib.inertial = Inertial.from_geometry(
        Box((28.5, 2.0, 3.5)),
        mass=5200.0,
        origin=Origin(xyz=(14.2, 0.0, 0.9)),
    )

    hoist_block = model.part("hoist_block")
    hoist_block.visual(
        Box((0.56, 0.18, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
        material=dark_grey,
        name="slider_bar",
    )
    hoist_block.visual(
        Box((0.08, 0.12, 0.42)),
        origin=Origin(xyz=(-0.16, 0.0, -0.44)),
        material=galvanized,
        name="left_hanger_plate",
    )
    hoist_block.visual(
        Box((0.08, 0.12, 0.42)),
        origin=Origin(xyz=(0.16, 0.0, -0.44)),
        material=galvanized,
        name="right_hanger_plate",
    )
    hoist_block.visual(
        Box((0.66, 0.42, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -0.66)),
        material=structural_steel,
        name="upper_block",
    )
    hoist_block.visual(
        Cylinder(radius=0.05, length=0.42),
        origin=Origin(xyz=(0.0, -0.10, -0.54), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="upper_left_sheave",
    )
    hoist_block.visual(
        Cylinder(radius=0.05, length=0.42),
        origin=Origin(xyz=(0.0, 0.10, -0.54), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="upper_right_sheave",
    )
    hoist_block.visual(
        Cylinder(radius=0.028, length=2.90),
        origin=Origin(xyz=(0.0, -0.10, -2.10)),
        material=cable,
        name="left_hoist_line",
    )
    hoist_block.visual(
        Cylinder(radius=0.028, length=2.90),
        origin=Origin(xyz=(0.0, 0.10, -2.10)),
        material=cable,
        name="right_hoist_line",
    )
    hoist_block.visual(
        Cylinder(radius=0.08, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -3.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_steel,
        name="lower_spreader",
    )
    hoist_block.visual(
        Box((0.70, 0.48, 0.38)),
        origin=Origin(xyz=(0.0, 0.0, -3.80)),
        material=crane_yellow,
        name="hook_body",
    )
    hoist_block.visual(
        Box((0.50, 0.40, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, -4.38)),
        material=crane_yellow,
        name="hook_cheeks",
    )
    hoist_block.visual(
        Cylinder(radius=0.07, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, -4.89)),
        material=structural_steel,
        name="hook_shank",
    )
    hoist_block.visual(
        hook_mesh,
        origin=Origin(xyz=(0.0, 0.0, -3.55)),
        material=safety_red,
        name="hook",
    )
    hoist_block.inertial = Inertial.from_geometry(
        Box((0.90, 0.55, 6.3)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, -3.15)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 26.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800000.0, velocity=0.35),
    )
    model.articulation(
        "jib_luffing",
        ArticulationType.REVOLUTE,
        parent=upperworks,
        child=jib,
        origin=Origin(xyz=(1.55, 0.0, 2.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400000.0,
            velocity=0.25,
            lower=-0.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "hoist_travel",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=hoist_block,
        origin=Origin(xyz=(2.4, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150000.0,
            velocity=1.0,
            lower=0.0,
            upper=20.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upperworks = object_model.get_part("upperworks")
    jib = object_model.get_part("luffing_jib")
    hoist_block = object_model.get_part("hoist_block")

    slewing = object_model.get_articulation("slewing_rotation")
    jib_luffing = object_model.get_articulation("jib_luffing")
    hoist_travel = object_model.get_articulation("hoist_travel")

    def _elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    ctx.allow_overlap(
        jib,
        upperworks,
        elem_a="root_knuckle",
        elem_b="hinge_pin",
        reason="simplified luffing hinge root uses a solid knuckle without modeled pin bore clearance",
    )
    ctx.allow_overlap(
        jib,
        upperworks,
        elem_a="left_root_lug",
        elem_b="hinge_pin",
        reason="hinge pin intentionally occupies the left jib lug volume because the lug hole is not explicitly cut",
    )
    ctx.allow_overlap(
        jib,
        upperworks,
        elem_a="right_root_lug",
        elem_b="hinge_pin",
        reason="hinge pin intentionally occupies the right jib lug volume because the lug hole is not explicitly cut",
    )
    ctx.allow_overlap(
        jib,
        upperworks,
        elem_a="left_root_lug",
        elem_b="front_hinge_support",
        reason="front hinge cheek is represented as a solid support block around the lug root without subtractive hole detail",
    )
    ctx.allow_overlap(
        jib,
        upperworks,
        elem_a="root_knuckle",
        elem_b="front_hinge_support",
        reason="the jib knuckle rotates inside a simplified boxed hinge cradle rather than a hollow clevis with cut relief",
    )
    ctx.allow_overlap(
        jib,
        upperworks,
        elem_a="right_root_lug",
        elem_b="front_hinge_support",
        reason="front hinge cheek is represented as a solid support block around the lug root without subtractive hole detail",
    )
    ctx.allow_overlap(
        hoist_block,
        jib,
        elem_a="slider_bar",
        elem_b="travel_beam",
        reason="the hoist trolley slider is modeled inside a solid guide beam instead of an explicit slotted track channel",
    )

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

    ctx.check(
        "part_count",
        len(object_model.parts) == 4,
        details=f"expected 4 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "slewing_axis_vertical",
        slewing.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical slewing axis, found {slewing.axis}",
    )
    ctx.check(
        "jib_luffing_axis_horizontal",
        abs(jib_luffing.axis[1]) == 1.0
        and jib_luffing.axis[0] == 0.0
        and jib_luffing.axis[2] == 0.0,
        details=f"expected horizontal luffing axis about world Y, found {jib_luffing.axis}",
    )
    ctx.check(
        "hoist_travel_axis_along_jib",
        hoist_travel.axis == (1.0, 0.0, 0.0),
        details=f"expected prismatic axis along jib chord, found {hoist_travel.axis}",
    )
    ctx.expect_contact(upperworks, mast, name="slewing_ring_contacts_mast_head")
    ctx.expect_gap(
        upperworks,
        mast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="slewing_ring_seats_on_mast_head",
    )
    ctx.expect_contact(jib, upperworks, name="jib_hinge_connected_to_upperworks")
    ctx.expect_contact(hoist_block, jib, name="hoist_block_supported_by_jib")

    ctx.expect_origin_gap(
        positive_link=upperworks,
        negative_link=mast,
        axis="z",
        min_gap=26.5,
        max_gap=27.0,
        name="upperworks_positioned_at_mast_top",
    )
    ctx.expect_origin_gap(
        positive_link=hoist_block,
        negative_link=jib,
        axis="x",
        min_gap=2.0,
        max_gap=2.8,
        name="hoist_block_starts_near_jib_root",
    )

    luff_limits = jib_luffing.motion_limits
    travel_limits = hoist_travel.motion_limits
    if luff_limits is not None and travel_limits is not None:
        pose_cases = [
            (
                {jib_luffing: luff_limits.lower, hoist_travel: travel_limits.lower},
                "jib_low_hoist_inboard",
            ),
            (
                {jib_luffing: luff_limits.lower, hoist_travel: travel_limits.upper},
                "jib_low_hoist_outboard",
            ),
            (
                {jib_luffing: luff_limits.upper, hoist_travel: travel_limits.lower},
                "jib_high_hoist_inboard",
            ),
            (
                {jib_luffing: luff_limits.upper, hoist_travel: travel_limits.upper},
                "jib_high_hoist_outboard",
            ),
        ]
        for pose_map, label in pose_cases:
            with ctx.pose(pose_map):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
                ctx.expect_contact(jib, upperworks, name=f"{label}_jib_hinge_contact")
                ctx.expect_contact(hoist_block, jib, name=f"{label}_hoist_contact")

    with ctx.pose({slewing: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slewed_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="slewed_pose_no_floating")

    with ctx.pose({jib_luffing: -0.10}):
        tip_low = _elem_center(jib, "tip_pulley")
    with ctx.pose({jib_luffing: 1.05}):
        tip_high = _elem_center(jib, "tip_pulley")
    ctx.check(
        "jib_tip_rises_when_luffing",
        tip_low is not None and tip_high is not None and tip_high[2] > tip_low[2] + 15.0,
        details=f"expected jib tip to rise substantially, got low={tip_low}, high={tip_high}",
    )

    with ctx.pose({jib_luffing: 0.0, hoist_travel: 0.0}):
        hoist_inboard = ctx.part_world_position(hoist_block)
    with ctx.pose({jib_luffing: 0.0, hoist_travel: 20.0}):
        hoist_outboard = ctx.part_world_position(hoist_block)
    ctx.check(
        "hoist_block_travels_out_along_jib",
        hoist_inboard is not None
        and hoist_outboard is not None
        and hoist_outboard[0] > hoist_inboard[0] + 18.0,
        details=f"expected outboard hoist translation along +X, got inboard={hoist_inboard}, outboard={hoist_outboard}",
    )

    with ctx.pose({slewing: 0.0}):
        cab_front = _elem_center(upperworks, "cab_shell")
    with ctx.pose({slewing: math.pi / 2.0}):
        cab_slewed = _elem_center(upperworks, "cab_shell")
    ctx.check(
        "upperworks_slew_changes_cab_azimuth",
        cab_front is not None
        and cab_slewed is not None
        and abs(cab_slewed[1] - cab_front[1]) > 3.0,
        details=f"expected cab to swing around mast, got front={cab_front}, slewed={cab_slewed}",
    )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="articulated_pose_clearance_sweep",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
