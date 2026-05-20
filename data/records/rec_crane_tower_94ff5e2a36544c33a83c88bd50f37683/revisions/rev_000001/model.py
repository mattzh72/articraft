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
    length = _distance(a, b)
    if length <= 1e-6:
        return
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


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
) -> dict[str, tuple[float, float, float]]:
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
        for side in range(4):
            ax, ay = corners[side]
            bx, by = corners[(side + 1) % 4]
            _add_member(part, (ax, ay, z0), (bx, by, z1), brace_radius, material)
            _add_member(part, (bx, by, z0), (ax, ay, z1), brace_radius, material)

    return {
        "mast_head": (0.0, 0.0, top_z),
        "mast_head_left": (0.0, half * 0.60, top_z - 0.05),
        "mast_head_right": (0.0, -half * 0.60, top_z - 0.05),
    }


def _add_triangular_boom(
    part,
    *,
    heel_start: tuple[float, float, float],
    tip_end: tuple[float, float, float],
    panels: int,
    root_half_width: float,
    tip_half_width: float,
    root_depth: float,
    tip_depth: float,
    chord_radius: float,
    brace_radius: float,
    material,
) -> dict[str, object]:
    lower_left: list[tuple[float, float, float]] = []
    lower_right: list[tuple[float, float, float]] = []
    upper: list[tuple[float, float, float]] = []

    for i in range(panels + 1):
        t = i / panels
        cx = _lerp(heel_start[0], tip_end[0], t)
        cz = _lerp(heel_start[2], tip_end[2], t)
        half_width = _lerp(root_half_width, tip_half_width, t)
        depth = _lerp(root_depth, tip_depth, t)
        lower_left.append((cx, -half_width, cz))
        lower_right.append((cx, half_width, cz))
        upper.append((cx - 0.03 * t, 0.0, cz + depth))

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

    return {
        "lower_left": lower_left,
        "lower_right": lower_right,
        "upper": upper,
        "tip_upper": upper[-1],
        "tip_lower_left": lower_left[-1],
        "tip_lower_right": lower_right[-1],
    }


def _element_center(ctx: TestContext, part, elem) -> tuple[float, float, float] | None:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        (mins[0] + maxs[0]) * 0.5,
        (mins[1] + maxs[1]) * 0.5,
        (mins[2] + maxs[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guyed_derrick_crane")

    safety_orange = model.material("safety_orange", rgba=(0.84, 0.46, 0.13, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.39, 0.40, 0.42, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.65, 0.67, 1.0))
    concrete = model.material("concrete", rgba=(0.62, 0.62, 0.60, 1.0))
    cable = model.material("cable", rgba=(0.17, 0.17, 0.18, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.84, 0.74, 0.16, 1.0))

    plate_thickness = 0.14
    slew_origin_z = 0.34
    boom_pivot = (0.24, 0.0, 0.95)
    mast_top_z = 5.98

    ground_anchor_plate = model.part("ground_anchor_plate")
    ground_anchor_plate.visual(
        Box((4.80, 0.64, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
        material=concrete,
        name="east_west_plate",
    )
    ground_anchor_plate.visual(
        Box((0.64, 4.80, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
        material=concrete,
        name="north_south_plate",
    )
    ground_anchor_plate.visual(
        Box((1.55, 1.55, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=weathered_steel,
        name="center_plinth",
    )
    ground_anchor_plate.visual(
        Cylinder(radius=0.46, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=dark_steel,
        name="slew_pedestal",
    )
    ground_anchor_plate.visual(
        Cylinder(radius=0.62, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=weathered_steel,
        name="bearing_base_flange",
    )
    for x, y in ((-1.85, 0.0), (0.0, 1.85), (0.0, -1.85)):
        ground_anchor_plate.visual(
            Box((0.42, 0.30, 0.08)),
            origin=Origin(xyz=(x, y, 0.18)),
            material=weathered_steel,
        )
    ground_anchor_plate.inertial = Inertial.from_geometry(
        Box((4.80, 4.80, 0.40)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    mast_assembly = model.part("mast_assembly")
    mast_assembly.visual(
        Cylinder(radius=0.60, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="slew_ring",
    )
    mast_assembly.visual(
        Box((1.15, 1.15, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=weathered_steel,
        name="rotating_deck",
    )
    mast_assembly.visual(
        Box((0.36, 0.30, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, boom_pivot[2] - 0.17)),
        material=dark_steel,
        name="heel_saddle_base",
    )
    mast_assembly.visual(
        Box((0.20, 0.05, 0.30)),
        origin=Origin(xyz=(0.08, -0.26, boom_pivot[2] - 0.01)),
        material=dark_steel,
        name="heel_saddle_left_cheek",
    )
    mast_assembly.visual(
        Box((0.20, 0.05, 0.30)),
        origin=Origin(xyz=(0.08, 0.26, boom_pivot[2] - 0.01)),
        material=dark_steel,
        name="heel_saddle_right_cheek",
    )

    _add_square_lattice_mast(
        mast_assembly,
        width=0.46,
        bottom_z=0.18,
        top_z=mast_top_z,
        panels=8,
        chord_radius=0.042,
        brace_radius=0.020,
        material=safety_orange,
    )
    mast_assembly.visual(
        Cylinder(radius=0.21, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, mast_top_z + 0.07)),
        material=warning_yellow,
        name="mast_head_cap",
    )
    mast_assembly.visual(
        Box((0.18, 0.42, 0.12)),
        origin=Origin(xyz=(0.34, 0.0, mast_top_z - 0.14)),
        material=galvanized,
        name="mast_head_gusset",
    )
    mast_assembly.visual(
        Box((0.50, 0.50, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=weathered_steel,
        name="mast_step_box",
    )

    mast_assembly.visual(
        Cylinder(radius=0.15, length=0.46),
        origin=Origin(
            xyz=(0.58, 0.0, 0.58),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_steel,
        name="winch_drum",
    )
    mast_assembly.visual(
        Box((0.52, 0.50, 0.36)),
        origin=Origin(xyz=(0.60, 0.0, 0.50)),
        material=weathered_steel,
        name="machinery_house",
    )
    mast_assembly.inertial = Inertial.from_geometry(
        Box((4.00, 4.00, 6.40)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 2.80)),
    )

    boom = model.part("boom")
    boom.visual(
        Box((0.60, 0.20, 0.22)),
        origin=Origin(xyz=(0.34, 0.0, 0.14)),
        material=dark_steel,
        name="heel_box",
    )
    boom.visual(
        Cylinder(radius=0.11, length=5.50),
        origin=Origin(
            xyz=(1.925, 0.0, 2.50),
            rpy=(0.0, 0.5743048301747017, 0.0),
        ),
        material=safety_orange,
        name="boom_main_tube",
    )
    boom.visual(
        Box((0.38, 0.28, 0.16)),
        origin=Origin(xyz=(2.12, 0.0, 3.02)),
        material=weathered_steel,
        name="tie_saddle",
    )
    boom.visual(
        Box((0.34, 0.30, 0.22)),
        origin=Origin(xyz=(3.24, 0.0, 4.64)),
        material=dark_steel,
        name="tip_head",
    )
    for x_mid, z_mid in (
        (0.92, 1.16),
        (1.55, 2.10),
        (2.20, 3.02),
        (2.82, 3.94),
    ):
        boom.visual(
            Box((0.22, 0.30, 0.10)),
            origin=Origin(xyz=(x_mid, 0.0, z_mid)),
            material=weathered_steel,
        )
    mast_head_relative_to_boom = (0.20, 0.0, mast_top_z - 0.16 - boom_pivot[2])
    _add_member(
        boom,
        (mast_head_relative_to_boom[0], -0.18, mast_head_relative_to_boom[2]),
        (2.02, -0.11, 3.06),
        0.022,
        galvanized,
        name="left_tie_rod",
    )
    _add_member(
        boom,
        (mast_head_relative_to_boom[0], 0.18, mast_head_relative_to_boom[2]),
        (2.02, 0.11, 3.06),
        0.022,
        galvanized,
        name="right_tie_rod",
    )
    boom.visual(
        Cylinder(radius=0.15, length=0.34),
        origin=Origin(
            xyz=(3.43, 0.0, 4.80),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=galvanized,
        name="tip_sheave",
    )
    boom.visual(
        Cylinder(radius=0.024, length=0.28),
        origin=Origin(
            xyz=(0.08, 0.0, 0.14),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=weathered_steel,
        name="heel_pin",
    )
    boom.inertial = Inertial.from_geometry(
        Box((4.10, 0.60, 5.40)),
        mass=5.0,
        origin=Origin(xyz=(1.90, 0.0, 2.55)),
    )

    model.articulation(
        "mast_slew",
        ArticulationType.REVOLUTE,
        parent=ground_anchor_plate,
        child=mast_assembly,
        origin=Origin(xyz=(0.0, 0.0, slew_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "boom_luff",
        ArticulationType.REVOLUTE,
        parent=mast_assembly,
        child=boom,
        origin=Origin(xyz=boom_pivot),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.25,
            lower=-0.18,
            upper=0.22,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_anchor_plate = object_model.get_part("ground_anchor_plate")
    mast_assembly = object_model.get_part("mast_assembly")
    boom = object_model.get_part("boom")
    mast_slew = object_model.get_articulation("mast_slew")
    boom_luff = object_model.get_articulation("boom_luff")
    slew_pedestal = ground_anchor_plate.get_visual("slew_pedestal")
    slew_ring = mast_assembly.get_visual("slew_ring")
    mast_head_gusset = mast_assembly.get_visual("mast_head_gusset")
    left_tie_rod = boom.get_visual("left_tie_rod")
    right_tie_rod = boom.get_visual("right_tie_rod")
    tip_sheave = boom.get_visual("tip_sheave")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        boom,
        mast_assembly,
        elem_a=left_tie_rod,
        elem_b=mast_head_gusset,
        reason="Left tie rod terminates in a pinned mast-head gusset; slight visual interpenetration stands in for the omitted clevis hole and pin hardware.",
    )
    ctx.allow_overlap(
        boom,
        mast_assembly,
        elem_a=right_tie_rod,
        elem_b=mast_head_gusset,
        reason="Right tie rod terminates in a pinned mast-head gusset; slight visual interpenetration stands in for the omitted clevis hole and pin hardware.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        mast_assembly,
        ground_anchor_plate,
        axis="z",
        positive_elem=slew_ring,
        negative_elem=slew_pedestal,
        max_gap=0.001,
        max_penetration=0.0,
        name="slew_ring_seats_on_pedestal",
    )
    ctx.expect_overlap(
        mast_assembly,
        ground_anchor_plate,
        axes="xy",
        elem_a=slew_ring,
        elem_b=slew_pedestal,
        min_overlap=0.85,
        name="slew_ring_covers_pedestal",
    )

    slew_limits = mast_slew.motion_limits
    boom_limits = boom_luff.motion_limits
    ctx.check(
        "slew_joint_definition",
        mast_slew.articulation_type == ArticulationType.REVOLUTE
        and mast_slew.axis == (0.0, 0.0, 1.0)
        and slew_limits is not None
        and slew_limits.lower is not None
        and slew_limits.upper is not None
        and slew_limits.lower < 0.0 < slew_limits.upper,
        details=f"type={mast_slew.articulation_type}, axis={mast_slew.axis}, limits={slew_limits}",
    )
    ctx.check(
        "boom_joint_definition",
        boom_luff.articulation_type == ArticulationType.REVOLUTE
        and boom_luff.axis == (0.0, 1.0, 0.0)
        and boom_limits is not None
        and boom_limits.lower is not None
        and boom_limits.upper is not None
        and boom_limits.lower < 0.0 < boom_limits.upper,
        details=f"type={boom_luff.articulation_type}, axis={boom_luff.axis}, limits={boom_limits}",
    )

    rest_tip = _element_center(ctx, boom, tip_sheave)
    ctx.check(
        "rest_pose_reads_as_derrick_boom",
        rest_tip is not None and rest_tip[0] > 3.3 and rest_tip[2] > 5.0,
        details=f"tip center={rest_tip}",
    )

    with ctx.pose({boom_luff: 0.16}):
        luffed_tip = _element_center(ctx, boom, tip_sheave)
    ctx.check(
        "boom_luff_moves_tip_in_vertical_plane",
        rest_tip is not None
        and luffed_tip is not None
        and abs(luffed_tip[2] - rest_tip[2]) > 0.25
        and abs(luffed_tip[1] - rest_tip[1]) < 0.05,
        details=f"rest={rest_tip}, luffed={luffed_tip}",
    )

    with ctx.pose({mast_slew: 0.75}):
        slewed_tip = _element_center(ctx, boom, tip_sheave)
    rest_radius = None if rest_tip is None else math.hypot(rest_tip[0], rest_tip[1])
    slewed_radius = None if slewed_tip is None else math.hypot(slewed_tip[0], slewed_tip[1])
    ctx.check(
        "mast_slew_rotates_boom_around_vertical_axis",
        rest_tip is not None
        and slewed_tip is not None
        and rest_radius is not None
        and slewed_radius is not None
        and abs(slewed_tip[1]) > 1.6
        and abs(slewed_tip[2] - rest_tip[2]) < 0.08
        and abs(slewed_radius - rest_radius) < 0.15,
        details=f"rest={rest_tip}, slewed={slewed_tip}, radii={(rest_radius, slewed_radius)}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
