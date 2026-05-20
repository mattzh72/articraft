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


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_square_mast(
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
            _add_member(part, (x0, y0, z0), (x0, y0, z1), brace_radius * 0.85, material)
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)

    ladder_x = half + 0.05
    rail_y = 0.20
    ladder_bottom = bottom_z + 0.8
    ladder_top = top_z - 0.8
    _add_member(
        part,
        (ladder_x, -rail_y, ladder_bottom),
        (ladder_x, -rail_y, ladder_top),
        brace_radius * 0.95,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, rail_y, ladder_bottom),
        (ladder_x, rail_y, ladder_top),
        brace_radius * 0.95,
        ladder_material,
    )
    rung_count = max(12, panels * 4)
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(
            part,
            (ladder_x, -rail_y, z),
            (ladder_x, rail_y, z),
            brace_radius * 0.72,
            ladder_material,
        )

    bracket_levels = [ladder_bottom + (ladder_top - ladder_bottom) * i / 5.0 for i in range(6)]
    for z in bracket_levels:
        _add_member(
            part,
            (half - 0.05, -0.26, z),
            (ladder_x, -rail_y, z),
            brace_radius * 0.75,
            ladder_material,
        )
        _add_member(
            part,
            (half - 0.05, 0.26, z),
            (ladder_x, rail_y, z),
            brace_radius * 0.75,
            ladder_material,
        )


def _add_box_truss(
    part,
    *,
    x_start: float,
    x_end: float,
    bottom_z: float,
    top_z: float,
    half_width: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
    material,
) -> None:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    lower_left = [(x, -half_width, bottom_z) for x in xs]
    lower_right = [(x, half_width, bottom_z) for x in xs]
    upper_left = [(x, -half_width, top_z) for x in xs]
    upper_right = [(x, half_width, top_z) for x in xs]

    for line in (lower_left, lower_right, upper_left, upper_right):
        for i in range(panels):
            _add_member(part, line[i], line[i + 1], chord_radius, material)

    for i in range(panels + 1):
        _add_member(part, lower_left[i], lower_right[i], brace_radius, material)
        _add_member(part, upper_left[i], upper_right[i], brace_radius, material)
        _add_member(part, lower_left[i], upper_left[i], brace_radius, material)
        _add_member(part, lower_right[i], upper_right[i], brace_radius, material)

    for i in range(panels):
        _add_member(part, lower_left[i], upper_left[i + 1], brace_radius, material)
        _add_member(part, upper_left[i], lower_left[i + 1], brace_radius, material)
        _add_member(part, lower_right[i], upper_right[i + 1], brace_radius, material)
        _add_member(part, upper_right[i], lower_right[i + 1], brace_radius, material)
        if i % 2 == 0:
            _add_member(part, upper_left[i], upper_right[i + 1], brace_radius * 0.9, material)
            _add_member(part, lower_left[i], lower_right[i + 1], brace_radius * 0.9, material)
        else:
            _add_member(part, upper_right[i], upper_left[i + 1], brace_radius * 0.9, material)
            _add_member(part, lower_right[i], lower_left[i + 1], brace_radius * 0.9, material)


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.22, 0.0, -1.00),
            (0.40, 0.0, -1.30),
            (0.46, 0.0, -1.72),
            (0.28, 0.0, -2.06),
            (-0.05, 0.0, -2.18),
            (-0.30, 0.0, -2.00),
            (-0.22, 0.0, -1.62),
        ],
        radius=0.085,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "flat_top_tower_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_top_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.77, 0.15, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.62, 1.0))
    ballast = model.material("ballast", rgba=(0.50, 0.50, 0.48, 1.0))
    cable = model.material("cable", rgba=(0.13, 0.13, 0.14, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.64, 0.79, 0.88, 0.38))
    safety_red = model.material("safety_red", rgba=(0.74, 0.14, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    mast_base = model.part("mast_base")
    mast_base.visual(
        Box((5.5, 5.5, 1.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        material=concrete,
        name="foundation_block",
    )
    mast_base.visual(
        Box((3.0, 3.0, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
        material=concrete,
    )
    mast_base.visual(
        Box((2.4, 2.4, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=dark_grey,
    )
    for sx in (-0.65, 0.65):
        for sy in (-0.65, 0.65):
            mast_base.visual(
                Cylinder(radius=0.10, length=1.0),
                origin=Origin(xyz=(sx, sy, 2.15)),
                material=steel,
            )
    _add_square_mast(
        mast_base,
        width=2.1,
        bottom_z=2.65,
        top_z=25.8,
        panels=8,
        chord_radius=0.10,
        brace_radius=0.055,
        material=crane_yellow,
        ladder_material=steel,
    )
    mast_base.visual(
        Cylinder(radius=1.25, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 25.925)),
        material=dark_grey,
        name="mast_slew_ring",
    )
    mast_base.inertial = Inertial.from_geometry(
        Box((5.5, 5.5, 26.05)),
        mass=42000.0,
        origin=Origin(xyz=(0.0, 0.0, 13.025)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.45, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=dark_grey,
        name="upper_turntable",
    )
    upperworks.visual(
        Box((5.0, 2.6, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=dark_grey,
        name="machinery_deck",
    )
    upperworks.visual(
        Box((3.4, 1.9, 1.9)),
        origin=Origin(xyz=(-1.6, 0.0, 1.85)),
        material=dark_grey,
        name="machinery_house",
    )
    upperworks.visual(
        Box((1.6, 1.2, 1.4)),
        origin=Origin(xyz=(1.95, -1.45, 1.45)),
        material=dark_grey,
        name="cab_body",
    )
    upperworks.visual(
        Box((1.46, 1.02, 1.18)),
        origin=Origin(xyz=(1.98, -1.45, 1.45)),
        material=cab_glass,
        name="cab_glazing",
    )
    upperworks.visual(
        Box((1.8, 2.0, 0.22)),
        origin=Origin(xyz=(1.4, 0.0, 1.08)),
        material=steel,
    )

    _add_box_truss(
        upperworks,
        x_start=1.35,
        x_end=30.0,
        bottom_z=1.0,
        top_z=3.0,
        half_width=0.85,
        panels=12,
        chord_radius=0.075,
        brace_radius=0.05,
        material=crane_yellow,
    )
    upperworks.visual(
        Box((27.2, 0.45, 0.18)),
        origin=Origin(xyz=(16.4, 0.0, 1.08)),
        material=steel,
        name="jib_trolley_beam",
    )
    upperworks.visual(
        Box((24.6, 0.40, 0.20)),
        origin=Origin(xyz=(17.7, -0.70, 1.10)),
        material=steel,
    )

    _add_box_truss(
        upperworks,
        x_start=-1.25,
        x_end=-11.2,
        bottom_z=1.0,
        top_z=2.6,
        half_width=0.78,
        panels=5,
        chord_radius=0.070,
        brace_radius=0.047,
        material=crane_yellow,
    )
    upperworks.visual(
        Box((7.6, 1.70, 0.20)),
        origin=Origin(xyz=(-7.2, 0.0, 1.05)),
        material=steel,
    )
    upperworks.visual(
        Box((2.2, 1.5, 1.1)),
        origin=Origin(xyz=(-6.4, 0.0, 1.63)),
        material=ballast,
    )
    upperworks.visual(
        Box((2.2, 1.5, 1.1)),
        origin=Origin(xyz=(-8.8, 0.0, 1.63)),
        material=ballast,
    )
    upperworks.visual(
        Box((1.5, 1.4, 0.95)),
        origin=Origin(xyz=(-10.5, 0.0, 1.55)),
        material=ballast,
    )
    _add_member(upperworks, (-1.0, -0.9, 0.95), (-1.0, 0.9, 0.95), 0.14, steel)
    _add_member(upperworks, (0.75, -1.0, 0.95), (0.75, 1.0, 0.95), 0.12, steel)

    upperworks.inertial = Inertial.from_geometry(
        Box((41.2, 3.3, 3.0)),
        mass=12000.0,
        origin=Origin(xyz=(10.0, 0.0, 1.5)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((1.8, 1.3, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
        material=dark_grey,
        name="trolley_frame",
    )
    trolley.visual(
        Box((1.1, 0.85, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=steel,
    )
    trolley.visual(
        Box((1.0, 0.52, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        material=crane_yellow,
    )
    for x in (-0.58, 0.58):
        for y in (-0.18, 0.18):
            trolley.visual(
                Cylinder(radius=0.11, length=0.14),
                origin=Origin(
                    xyz=(x, y, 0.06),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
            )
    trolley.visual(
        Cylinder(radius=0.050, length=6.55),
        origin=Origin(xyz=(-0.18, -0.18, -3.775)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.050, length=6.55),
        origin=Origin(xyz=(-0.18, 0.18, -3.775)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.050, length=6.55),
        origin=Origin(xyz=(0.18, -0.18, -3.775)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.050, length=6.55),
        origin=Origin(xyz=(0.18, 0.18, -3.775)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.20, length=0.75),
        origin=Origin(
            xyz=(0.0, 0.0, -6.95),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hoist_equalizer",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((1.8, 1.3, 7.2)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, -3.6)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.09, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
        material=steel,
        name="hook_head",
    )
    hook_block.visual(
        Box((0.95, 0.62, 1.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.76)),
        material=crane_yellow,
        name="hook_body",
    )
    _add_member(hook_block, (0.0, -0.22, -1.35), (0.0, 0.22, -1.35), 0.10, steel)
    _add_member(hook_block, (0.0, 0.0, -1.35), (0.22, 0.0, -1.00), 0.08, steel)
    hook_block.visual(hook_mesh, material=safety_red, name="hook_shape")
    hook_block.inertial = Inertial.from_geometry(
        Box((1.0, 0.7, 2.2)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, -1.1)),
    )

    model.articulation(
        "slew_rotation",
        ArticulationType.CONTINUOUS,
        parent=mast_base,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 26.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300000.0, velocity=0.18),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(3.8, 0.0, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60000.0,
            velocity=1.2,
            lower=0.0,
            upper=23.5,
        ),
    )
    model.articulation(
        "hook_mount",
        ArticulationType.FIXED,
        parent=trolley,
        child=hook_block,
        origin=Origin(xyz=(0.0, 0.0, -7.15)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast_base = object_model.get_part("mast_base")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    hook_block = object_model.get_part("hook_block")
    slew_rotation = object_model.get_articulation("slew_rotation")
    trolley_travel = object_model.get_articulation("trolley_travel")

    ctx.check(
        "slew joint is continuous about vertical mast axis",
        slew_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(slew_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"type={slew_rotation.articulation_type}, axis={slew_rotation.axis}",
    )
    ctx.check(
        "trolley joint is prismatic along jib span",
        trolley_travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(trolley_travel.axis) == (1.0, 0.0, 0.0)
        and trolley_travel.motion_limits is not None
        and trolley_travel.motion_limits.upper is not None
        and trolley_travel.motion_limits.upper > 20.0,
        details=(
            f"type={trolley_travel.articulation_type}, axis={trolley_travel.axis}, "
            f"limits={trolley_travel.motion_limits}"
        ),
    )

    with ctx.pose({slew_rotation: 0.0, trolley_travel: 0.0}):
        ctx.expect_gap(
            upperworks,
            mast_base,
            axis="z",
            positive_elem="upper_turntable",
            negative_elem="mast_slew_ring",
            max_gap=0.02,
            max_penetration=0.0,
            name="upper turntable seats on mast slew ring",
        )
        ctx.expect_overlap(
            upperworks,
            mast_base,
            axes="xy",
            elem_a="upper_turntable",
            elem_b="mast_slew_ring",
            min_overlap=2.2,
            name="turntable overlaps mast ring in plan",
        )
        ctx.expect_contact(
            upperworks,
            trolley,
            elem_a="jib_trolley_beam",
            contact_tol=0.012,
            name="trolley rides on the jib trolley beam",
        )
        ctx.expect_contact(
            trolley,
            hook_block,
            elem_a="hoist_equalizer",
            elem_b="hook_head",
            name="hook block is pinned to the trolley equalizer",
        )
        ctx.expect_origin_distance(
            trolley,
            hook_block,
            axes="xy",
            max_dist=0.05,
            name="hook stays directly below the trolley",
        )
        rest_trolley_pos = ctx.part_world_position(trolley)

    upper_limit = trolley_travel.motion_limits.upper or 0.0
    with ctx.pose({slew_rotation: 0.0, trolley_travel: upper_limit}):
        ctx.expect_contact(
            upperworks,
            trolley,
            elem_a="jib_trolley_beam",
            contact_tol=0.012,
            name="trolley stays supported by the jib beam at max travel",
        )
        ctx.expect_origin_distance(
            trolley,
            hook_block,
            axes="xy",
            max_dist=0.05,
            name="hook remains aligned under trolley at max travel",
        )
        extended_trolley_pos = ctx.part_world_position(trolley)

    ctx.check(
        "trolley extends outward along the jib",
        rest_trolley_pos is not None
        and extended_trolley_pos is not None
        and extended_trolley_pos[0] > rest_trolley_pos[0] + 20.0
        and abs(extended_trolley_pos[1] - rest_trolley_pos[1]) < 0.05
        and abs(extended_trolley_pos[2] - rest_trolley_pos[2]) < 0.05,
        details=f"rest={rest_trolley_pos}, extended={extended_trolley_pos}",
    )

    with ctx.pose({slew_rotation: math.pi / 2.0, trolley_travel: 9.0}):
        slewed_trolley_pos = ctx.part_world_position(trolley)

    ctx.check(
        "slewing rotates the jib and trolley in plan",
        rest_trolley_pos is not None
        and slewed_trolley_pos is not None
        and abs(slewed_trolley_pos[0]) < 1.0
        and slewed_trolley_pos[1] > 10.0,
        details=f"rest={rest_trolley_pos}, slewed={slewed_trolley_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
