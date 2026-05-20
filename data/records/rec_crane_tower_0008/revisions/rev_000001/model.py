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
    return math.sqrt(
        (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2
    )


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(horizontal, dz)
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
        for index in range(4):
            x0, y0 = corners[index]
            x1, y1 = corners[(index + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), brace_radius, material)

    for level_index in range(panels):
        z0 = levels[level_index]
        z1 = levels[level_index + 1]
        for face_index in range(4):
            x0, y0 = corners[face_index]
            x1, y1 = corners[(face_index + 1) % 4]
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)


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

    def _top_z(x_value: float) -> float:
        t = 0.0 if abs(span) < 1e-9 else (x_value - x_start) / span
        return root_top_z + (tip_top_z - root_top_z) * t

    lower_left = [(x, -half_width, bottom_z) for x in xs]
    lower_right = [(x, half_width, bottom_z) for x in xs]
    upper = [(x, 0.0, _top_z(x)) for x in xs]

    for index in range(panels):
        _add_member(part, lower_left[index], lower_left[index + 1], chord_radius, material)
        _add_member(
            part, lower_right[index], lower_right[index + 1], chord_radius, material
        )
        _add_member(part, upper[index], upper[index + 1], chord_radius, material)

    for index in range(panels + 1):
        _add_member(part, lower_left[index], lower_right[index], brace_radius, material)
        _add_member(part, lower_left[index], upper[index], brace_radius, material)
        _add_member(part, lower_right[index], upper[index], brace_radius, material)

    for index in range(panels):
        if index % 2 == 0:
            _add_member(part, lower_left[index], upper[index + 1], brace_radius, material)
            _add_member(
                part, lower_right[index], upper[index + 1], brace_radius, material
            )
        else:
            _add_member(part, upper[index], lower_left[index + 1], brace_radius, material)
            _add_member(
                part, upper[index], lower_right[index + 1], brace_radius, material
            )

    return {"lower_left": lower_left, "lower_right": lower_right, "upper": upper}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_tower_crane")

    tower_yellow = model.material("tower_yellow", rgba=(0.93, 0.77, 0.16, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.65, 1.0))
    ballast = model.material("ballast", rgba=(0.50, 0.50, 0.48, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.62, 0.82, 0.91, 0.34))
    accent_red = model.material("accent_red", rgba=(0.74, 0.16, 0.12, 1.0))

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((4.20, 4.20, 0.85)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=concrete,
        name="foundation",
    )
    tower_base.visual(
        Box((1.90, 1.90, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        material=concrete,
        name="pedestal",
    )
    tower_base.visual(
        Box((1.90, 1.90, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
        material=dark_grey,
        name="pedestal_cap",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower_base.visual(
                Cylinder(radius=0.07, length=0.55),
                origin=Origin(xyz=(0.45 * x_sign, 0.45 * y_sign, 1.125)),
                material=steel,
            )
    _add_square_mast(
        tower_base,
        width=1.60,
        bottom_z=1.56,
        top_z=22.00,
        panels=11,
        chord_radius=0.09,
        brace_radius=0.05,
        material=tower_yellow,
    )
    tower_base.visual(
        Cylinder(radius=1.25, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 22.175)),
        material=dark_grey,
        name="mast_top_ring",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((4.20, 4.20, 22.35)),
        mass=65000.0,
        origin=Origin(xyz=(0.0, 0.0, 11.175)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.30, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=dark_grey,
        name="slew_turntable",
    )
    upperworks.visual(
        Box((4.60, 3.40, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=dark_grey,
        name="machinery_deck",
    )
    upperworks.visual(
        Cylinder(radius=0.36, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=tower_yellow,
        name="slew_pedestal",
    )
    upperworks.visual(
        Box((2.60, 1.90, 2.10)),
        origin=Origin(xyz=(0.15, 0.0, 1.85)),
        material=dark_grey,
        name="cab_box",
    )
    upperworks.visual(
        Box((0.08, 1.45, 1.15)),
        origin=Origin(xyz=(1.41, 0.0, 1.88)),
        material=cab_glass,
        name="cab_front_glazing",
    )
    upperworks.visual(
        Box((1.70, 1.55, 1.55)),
        origin=Origin(xyz=(-1.25, 0.0, 1.575)),
        material=dark_grey,
        name="machinery_house",
    )

    jib = _add_triangular_truss(
        upperworks,
        x_start=2.20,
        x_end=23.00,
        bottom_z=0.86,
        half_width=0.75,
        root_top_z=4.25,
        tip_top_z=2.55,
        panels=10,
        chord_radius=0.10,
        brace_radius=0.055,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((18.60, 0.24, 0.12)),
        origin=Origin(xyz=(12.30, -0.70, 0.88)),
        material=steel,
        name="jib_walkway",
    )
    upperworks.visual(
        Box((17.90, 0.18, 0.12)),
        origin=Origin(xyz=(12.05, -0.42, 1.00)),
        material=steel,
        name="left_rail",
    )
    upperworks.visual(
        Box((17.90, 0.18, 0.12)),
        origin=Origin(xyz=(12.05, 0.42, 1.00)),
        material=steel,
        name="right_rail",
    )
    for x_support in (4.2, 6.3, 8.4, 10.5, 12.6, 14.7, 16.8, 18.9, 21.0):
        _add_member(
            upperworks,
            (x_support, -0.42, 0.94),
            (x_support, -0.75, 0.86),
            0.035,
            steel,
        )
        _add_member(
            upperworks,
            (x_support, 0.42, 0.94),
            (x_support, 0.75, 0.86),
            0.035,
            steel,
        )

    counter_jib = _add_triangular_truss(
        upperworks,
        x_start=-2.00,
        x_end=-8.80,
        bottom_z=0.86,
        half_width=0.70,
        root_top_z=3.95,
        tip_top_z=2.60,
        panels=4,
        chord_radius=0.095,
        brace_radius=0.05,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((6.40, 1.40, 0.12)),
        origin=Origin(xyz=(-5.40, 0.0, 0.88)),
        material=steel,
        name="counter_jib_deck",
    )
    upperworks.visual(
        Box((1.30, 1.35, 1.10)),
        origin=Origin(xyz=(-6.10, 0.0, 1.49)),
        material=ballast,
        name="ballast_block_front",
    )
    upperworks.visual(
        Box((1.30, 1.35, 1.10)),
        origin=Origin(xyz=(-7.45, 0.0, 1.49)),
        material=ballast,
        name="ballast_block_rear",
    )
    upperworks.visual(
        Box((0.40, 0.40, 0.55)),
        origin=Origin(xyz=(22.95, 0.0, 2.85)),
        material=accent_red,
        name="tip_marker",
    )

    apex = (0.55, 0.0, 6.15)
    _add_member(upperworks, (-0.55, -0.72, 0.80), apex, 0.12, tower_yellow)
    _add_member(upperworks, (-0.55, 0.72, 0.80), apex, 0.12, tower_yellow)
    upperworks.visual(
        Box((0.55, 0.55, 0.55)),
        origin=Origin(xyz=apex),
        material=tower_yellow,
        name="tower_head",
    )
    _add_member(upperworks, apex, jib["upper"][4], 0.045, steel)
    _add_member(upperworks, apex, jib["upper"][-1], 0.045, steel)
    _add_member(upperworks, apex, counter_jib["upper"][-1], 0.045, steel)

    upperworks.inertial = Inertial.from_geometry(
        Box((31.80, 3.80, 6.20)),
        mass=16000.0,
        origin=Origin(xyz=(7.00, 0.0, 2.70)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.95, 0.18, 0.24)),
        origin=Origin(xyz=(0.0, -0.42, 0.12)),
        material=dark_grey,
        name="left_bogie",
    )
    trolley.visual(
        Box((0.95, 0.18, 0.24)),
        origin=Origin(xyz=(0.0, 0.42, 0.12)),
        material=dark_grey,
        name="right_bogie",
    )
    trolley.visual(
        Box((1.20, 1.05, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=steel,
        name="bridge_beam",
    )
    trolley.visual(
        Box((0.86, 0.62, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=tower_yellow,
        name="hoist_house",
    )
    trolley.visual(
        Box((0.18, 0.18, 0.80)),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        material=steel,
        name="hook_bar_stub",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((1.20, 1.05, 1.15)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower_base,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 22.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.22),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(3.60, 0.0, 1.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30000.0,
            velocity=1.10,
            lower=0.0,
            upper=16.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_base = object_model.get_part("tower_base")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    slewing_rotation = object_model.get_articulation("slewing_rotation")
    trolley_travel = object_model.get_articulation("trolley_travel")

    mast_top_ring = tower_base.get_visual("mast_top_ring")
    slew_turntable = upperworks.get_visual("slew_turntable")
    cab_box = upperworks.get_visual("cab_box")
    left_rail = upperworks.get_visual("left_rail")
    right_rail = upperworks.get_visual("right_rail")
    left_bogie = trolley.get_visual("left_bogie")
    right_bogie = trolley.get_visual("right_bogie")

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
        "key_parts_present",
        all(part is not None for part in (tower_base, upperworks, trolley))
        and cab_box is not None,
        "The crane must include the mast, revolving upperworks, center cab box, and trolley.",
    )
    ctx.check(
        "slewing_joint_is_vertical_continuous",
        slewing_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(slewing_rotation.axis) == (0.0, 0.0, 1.0)
        and slewing_rotation.motion_limits is not None
        and slewing_rotation.motion_limits.lower is None
        and slewing_rotation.motion_limits.upper is None,
        "Slewing ring should be a continuous rotation about the vertical mast axis.",
    )
    ctx.check(
        "trolley_joint_is_prismatic_along_jib",
        trolley_travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(trolley_travel.axis) == (1.0, 0.0, 0.0)
        and trolley_travel.motion_limits is not None
        and trolley_travel.motion_limits.lower == 0.0
        and trolley_travel.motion_limits.upper is not None
        and trolley_travel.motion_limits.upper > 12.0,
        "Trolley travel should be a long prismatic motion along the jib rails.",
    )

    ctx.expect_contact(
        tower_base,
        upperworks,
        elem_a=mast_top_ring,
        elem_b=slew_turntable,
        contact_tol=0.002,
        name="turntable_contacts_mast_ring",
    )
    ctx.expect_origin_distance(
        tower_base,
        upperworks,
        axes="xy",
        max_dist=0.001,
        name="upperworks_centered_on_mast",
    )

    with ctx.pose({trolley_travel: 0.0}):
        ctx.expect_gap(
            trolley,
            upperworks,
            axis="z",
            positive_elem=left_bogie,
            negative_elem=left_rail,
            max_gap=0.001,
            max_penetration=0.0,
            name="left_bogie_sits_on_left_rail_at_root",
        )
        ctx.expect_gap(
            trolley,
            upperworks,
            axis="z",
            positive_elem=right_bogie,
            negative_elem=right_rail,
            max_gap=0.001,
            max_penetration=0.0,
            name="right_bogie_sits_on_right_rail_at_root",
        )
        ctx.expect_within(
            trolley,
            upperworks,
            axes="x",
            inner_elem=left_bogie,
            outer_elem=left_rail,
            margin=0.0,
            name="left_bogie_within_left_rail_at_root",
        )

    with ctx.pose({trolley_travel: trolley_travel.motion_limits.upper}):
        ctx.expect_within(
            trolley,
            upperworks,
            axes="x",
            inner_elem=left_bogie,
            outer_elem=left_rail,
            margin=0.0,
            name="left_bogie_within_left_rail_at_tip",
        )
        ctx.expect_within(
            trolley,
            upperworks,
            axes="x",
            inner_elem=right_bogie,
            outer_elem=right_rail,
            margin=0.0,
            name="right_bogie_within_right_rail_at_tip",
        )

    with ctx.pose({slewing_rotation: 0.0, trolley_travel: 0.0}):
        trolley_at_zero = ctx.part_world_position(trolley)
    with ctx.pose({slewing_rotation: math.pi / 2.0, trolley_travel: 0.0}):
        trolley_at_quarter_turn = ctx.part_world_position(trolley)
    with ctx.pose(
        {
            slewing_rotation: 0.0,
            trolley_travel: trolley_travel.motion_limits.upper,
        }
    ):
        trolley_at_tip = ctx.part_world_position(trolley)

    ctx.check(
        "slewing_rotates_trolley_about_mast",
        trolley_at_zero is not None
        and trolley_at_quarter_turn is not None
        and abs(trolley_at_quarter_turn[0]) < 0.05
        and abs(trolley_at_quarter_turn[1] - trolley_at_zero[0]) < 0.05,
        "Quarter-turn slewing should swing the jib and trolley around the vertical mast axis.",
    )
    ctx.check(
        "trolley_moves_along_jib_by_joint_range",
        trolley_at_zero is not None
        and trolley_at_tip is not None
        and trolley_travel.motion_limits is not None
        and abs(
            (trolley_at_tip[0] - trolley_at_zero[0]) - trolley_travel.motion_limits.upper
        )
        < 0.02
        and abs(trolley_at_tip[1] - trolley_at_zero[1]) < 0.001,
        "Prismatic trolley travel should move the trolley forward along the jib rails.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
