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
) -> list[float]:
    half = width * 0.5
    corners = [
        (-half, -half),
        (half, -half),
        (half, half),
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
            if i % 2 == 0:
                _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
                _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)
            else:
                _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)
                _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)

    return levels


def _add_box_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    y_half: float,
    bottom_z: float,
    top_z: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> list[list[tuple[float, float, float]]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    sections: list[list[tuple[float, float, float]]] = []

    for x in xs:
        section = [
            (x, -y_half, bottom_z),
            (x, y_half, bottom_z),
            (x, y_half, top_z),
            (x, -y_half, top_z),
        ]
        sections.append(section)
        for i in range(4):
            _add_member(part, section[i], section[(i + 1) % 4], brace_radius, material)

    for i in range(panels):
        s0 = sections[i]
        s1 = sections[i + 1]
        for j in range(4):
            _add_member(part, s0[j], s1[j], chord_radius, material)

        if i % 2 == 0:
            _add_member(part, s0[0], s1[1], brace_radius, material)
            _add_member(part, s0[3], s1[2], brace_radius, material)
            _add_member(part, s0[0], s1[3], brace_radius, material)
            _add_member(part, s0[1], s1[2], brace_radius, material)
        else:
            _add_member(part, s0[1], s1[0], brace_radius, material)
            _add_member(part, s0[2], s1[3], brace_radius, material)
            _add_member(part, s0[3], s1[0], brace_radius, material)
            _add_member(part, s0[2], s1[1], brace_radius, material)

    return sections


def _add_triangular_jib(
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_top_tower_crane")

    tower_yellow = model.material("tower_yellow", rgba=(0.92, 0.76, 0.14, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.59, 0.61, 0.64, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    ballast = model.material("ballast", rgba=(0.47, 0.48, 0.50, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.68, 0.82, 0.90, 0.36))

    mast = model.part("mast")
    mast.visual(
        Box((7.0, 7.0, 1.6)),
        origin=Origin(xyz=(0.0, 0.0, 0.8)),
        material=concrete,
        name="foundation_block",
    )
    mast.visual(
        Box((3.4, 3.4, 2.0)),
        origin=Origin(xyz=(0.0, 0.0, 2.6)),
        material=concrete,
        name="pedestal_block",
    )
    mast.visual(
        Box((2.8, 2.8, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 3.725)),
        material=dark_grey,
        name="mast_foot_collared_base",
    )
    _add_square_lattice_mast(
        mast,
        width=2.2,
        bottom_z=3.6,
        top_z=31.6,
        panels=12,
        chord_radius=0.12,
        brace_radius=0.075,
        material=tower_yellow,
    )
    mast.visual(
        Box((2.8, 2.8, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 31.70)),
        material=dark_grey,
        name="mast_head_table",
    )
    mast.visual(
        Cylinder(radius=1.55, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 31.775)),
        material=dark_grey,
        name="slewing_ring_lower",
    )
    mast.inertial = Inertial.from_geometry(
        Box((7.0, 7.0, 32.0)),
        mass=120000.0,
        origin=Origin(xyz=(0.0, 0.0, 16.0)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.62, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=dark_grey,
        name="slewing_ring_upper",
    )
    upperworks.visual(
        Cylinder(radius=0.60, length=1.45),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=tower_yellow,
        name="kingpost",
    )
    upperworks.visual(
        Box((3.6, 2.8, 0.95)),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=dark_grey,
        name="machinery_house",
    )
    upperworks.visual(
        Box((2.2, 2.0, 0.75)),
        origin=Origin(xyz=(-0.9, 0.0, 1.55)),
        material=dark_grey,
        name="rear_machinery_cover",
    )

    head_sections = _add_box_truss(
        upperworks,
        x_start=-4.0,
        x_end=4.0,
        y_half=1.1,
        bottom_z=1.05,
        top_z=2.35,
        panels=4,
        chord_radius=0.10,
        brace_radius=0.065,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((8.2, 1.1, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        material=steel,
        name="head_catwalk",
    )

    jib = _add_triangular_jib(
        upperworks,
        x_start=4.0,
        x_end=38.0,
        bottom_z=1.08,
        half_width=0.82,
        root_top_z=2.35,
        tip_top_z=1.55,
        panels=12,
        chord_radius=0.10,
        brace_radius=0.06,
        material=tower_yellow,
    )
    front_section = head_sections[-1]
    jib_root_left = jib["lower_left"][0]
    jib_root_right = jib["lower_right"][0]
    jib_root_top = jib["upper"][0]
    _add_member(upperworks, front_section[0], jib_root_left, 0.07, tower_yellow)
    _add_member(upperworks, front_section[1], jib_root_right, 0.07, tower_yellow)
    _add_member(upperworks, front_section[3], jib_root_top, 0.07, tower_yellow)
    _add_member(upperworks, front_section[2], jib_root_top, 0.07, tower_yellow)

    upperworks.visual(
        Box((27.5, 0.08, 0.10)),
        origin=Origin(xyz=(20.5, -1.03, 1.18)),
        material=steel,
        name="rail_left",
    )
    upperworks.visual(
        Box((27.5, 0.08, 0.10)),
        origin=Origin(xyz=(20.5, 1.03, 1.18)),
        material=steel,
        name="rail_right",
    )
    for rail_point in jib["lower_left"]:
        x, y, z = rail_point
        if 6.75 <= x <= 34.25:
            _add_member(
                upperworks,
                (x, y, z),
                (x, -1.03, 1.18),
                0.035,
                steel,
            )
    for rail_point in jib["lower_right"]:
        x, y, z = rail_point
        if 6.75 <= x <= 34.25:
            _add_member(
                upperworks,
                (x, y, z),
                (x, 1.03, 1.18),
                0.035,
                steel,
            )
    upperworks.visual(
        Box((25.0, 0.55, 0.05)),
        origin=Origin(xyz=(19.0, 0.0, 1.105)),
        material=steel,
        name="jib_walkway",
    )

    counter_jib = _add_triangular_jib(
        upperworks,
        x_start=-4.0,
        x_end=-11.5,
        bottom_z=1.08,
        half_width=0.70,
        root_top_z=2.15,
        tip_top_z=1.45,
        panels=4,
        chord_radius=0.09,
        brace_radius=0.055,
        material=tower_yellow,
    )
    rear_section = head_sections[0]
    counter_root_left = counter_jib["lower_left"][0]
    counter_root_right = counter_jib["lower_right"][0]
    counter_root_top = counter_jib["upper"][0]
    _add_member(upperworks, rear_section[0], counter_root_left, 0.065, tower_yellow)
    _add_member(upperworks, rear_section[1], counter_root_right, 0.065, tower_yellow)
    _add_member(upperworks, rear_section[3], counter_root_top, 0.065, tower_yellow)
    _add_member(upperworks, rear_section[2], counter_root_top, 0.065, tower_yellow)
    upperworks.visual(
        Box((5.4, 1.0, 0.08)),
        origin=Origin(xyz=(-8.3, 0.0, 1.12)),
        material=steel,
        name="rear_ballast_deck",
    )
    upperworks.visual(
        Box((1.35, 1.0, 1.05)),
        origin=Origin(xyz=(-7.3, 0.0, 1.685)),
        material=ballast,
        name="ballast_block_0",
    )
    upperworks.visual(
        Box((1.35, 1.0, 1.05)),
        origin=Origin(xyz=(-8.75, 0.0, 1.685)),
        material=ballast,
        name="ballast_block_1",
    )
    upperworks.visual(
        Box((1.15, 0.95, 0.95)),
        origin=Origin(xyz=(-10.1, 0.0, 1.635)),
        material=ballast,
        name="ballast_block_2",
    )

    upperworks.visual(
        Box((1.40, 0.30, 0.20)),
        origin=Origin(xyz=(1.55, -1.46, 0.90)),
        material=dark_grey,
        name="cab_support_beam",
    )
    upperworks.visual(
        Box((1.35, 1.20, 0.95)),
        origin=Origin(xyz=(2.40, -2.00, 1.10)),
        material=dark_grey,
        name="operator_cab_shell",
    )
    upperworks.visual(
        Box((1.18, 0.98, 0.74)),
        origin=Origin(xyz=(2.46, -2.03, 1.12)),
        material=cab_glass,
        name="operator_cab_glass",
    )

    upperworks.inertial = Inertial.from_geometry(
        Box((50.0, 6.0, 3.4)),
        mass=48000.0,
        origin=Origin(xyz=(10.0, 0.0, 1.7)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.80, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, -1.03, -0.11)),
        material=dark_grey,
        name="left_bogie",
    )
    trolley.visual(
        Box((0.80, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, 1.03, -0.11)),
        material=dark_grey,
        name="right_bogie",
    )
    trolley.visual(
        Box((0.90, 2.00, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.29)),
        material=steel,
        name="carriage_frame",
    )
    trolley.visual(
        Box((0.24, 0.06, 0.28)),
        origin=Origin(xyz=(0.0, -0.99, -0.33)),
        material=steel,
        name="left_hanger",
    )
    trolley.visual(
        Box((0.24, 0.06, 0.28)),
        origin=Origin(xyz=(0.0, 0.99, -0.33)),
        material=steel,
        name="right_hanger",
    )
    trolley.visual(
        Box((0.96, 0.74, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, -0.67)),
        material=dark_grey,
        name="trolley_house",
    )
    trolley.visual(
        Box((0.36, 0.36, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, -0.27)),
        material=tower_yellow,
        name="central_yoke",
    )
    trolley.visual(
        Box((0.52, 0.36, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -1.01)),
        material=steel,
        name="pulley_block",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((1.2, 2.2, 1.6)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, -0.45)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 31.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500000.0, velocity=0.22),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(8.8, 0.0, 1.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45000.0,
            velocity=1.8,
            lower=0.0,
            upper=25.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    slewing_rotation = object_model.get_articulation("slewing_rotation")
    trolley_travel = object_model.get_articulation("trolley_travel")

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
        "slewing joint is continuous around vertical axis",
        slewing_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(slewing_rotation.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={slewing_rotation.articulation_type}, "
            f"axis={tuple(slewing_rotation.axis)}"
        ),
    )

    travel_limits = trolley_travel.motion_limits
    travel_upper = 0.0 if travel_limits is None or travel_limits.upper is None else travel_limits.upper
    ctx.check(
        "trolley joint is a long +X prismatic run",
        trolley_travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(trolley_travel.axis) == (1.0, 0.0, 0.0)
        and travel_limits is not None
        and travel_limits.lower == 0.0
        and travel_limits.upper is not None
        and travel_limits.upper >= 25.0,
        details=(
            f"type={trolley_travel.articulation_type}, axis={tuple(trolley_travel.axis)}, "
            f"limits={travel_limits}"
        ),
    )

    ctx.expect_contact(
        upperworks,
        mast,
        elem_a="slewing_ring_upper",
        elem_b="slewing_ring_lower",
        contact_tol=0.005,
        name="upper slewing ring seats on mast ring",
    )
    ctx.expect_overlap(
        upperworks,
        mast,
        axes="xy",
        elem_a="slewing_ring_upper",
        elem_b="slewing_ring_lower",
        min_overlap=2.8,
        name="slewing rings stay concentrically aligned",
    )

    with ctx.pose({trolley_travel: 0.0}):
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="left_bogie",
            elem_b="rail_left",
            contact_tol=0.005,
            name="left trolley bogie rides the left rail at inner position",
        )
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="right_bogie",
            elem_b="rail_right",
            contact_tol=0.005,
            name="right trolley bogie rides the right rail at inner position",
        )
        ctx.expect_overlap(
            trolley,
            upperworks,
            axes="x",
            elem_a="carriage_frame",
            elem_b="rail_left",
            min_overlap=0.55,
            name="trolley carriage remains over the rail span at inner position",
        )
        rest_pos = ctx.part_world_position(trolley)

    with ctx.pose({trolley_travel: travel_upper}):
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="left_bogie",
            elem_b="rail_left",
            contact_tol=0.005,
            name="left trolley bogie rides the left rail at outer position",
        )
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="right_bogie",
            elem_b="rail_right",
            contact_tol=0.005,
            name="right trolley bogie rides the right rail at outer position",
        )
        ctx.expect_overlap(
            trolley,
            upperworks,
            axes="x",
            elem_a="carriage_frame",
            elem_b="rail_left",
            min_overlap=0.55,
            name="trolley carriage remains over the rail span at outer position",
        )
        outer_pos = ctx.part_world_position(trolley)

    ctx.check(
        "trolley travels out along the jib",
        rest_pos is not None
        and outer_pos is not None
        and outer_pos[0] > rest_pos[0] + 20.0,
        details=f"rest={rest_pos}, outer={outer_pos}",
    )

    with ctx.pose({slewing_rotation: math.pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(trolley)

    ctx.check(
        "slewing swings the jib around the mast",
        rest_pos is not None
        and quarter_turn_pos is not None
        and abs(quarter_turn_pos[1]) > 5.0
        and abs(quarter_turn_pos[0]) < abs(rest_pos[0]) - 2.0,
        details=f"rest={rest_pos}, quarter_turn={quarter_turn_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
