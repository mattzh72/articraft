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
) -> dict[str, list[tuple[float, float, float]]]:
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

    return {
        "lower_ring": [(x, y, bottom_z) for x, y in corners],
        "upper_ring": [(x, y, top_z) for x, y in corners],
    }


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
) -> dict[str, list[tuple[float, float, float]]]:
    xs = [x_start + (x_end - x_start) * i / panels for i in range(panels + 1)]
    lower_left = [(x, -half_width, bottom_z) for x in xs]
    lower_right = [(x, half_width, bottom_z) for x in xs]
    upper_left = [(x, -half_width, top_z) for x in xs]
    upper_right = [(x, half_width, top_z) for x in xs]

    for chord in (lower_left, lower_right, upper_left, upper_right):
        for i in range(panels):
            _add_member(part, chord[i], chord[i + 1], chord_radius, material)

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
        _add_member(part, lower_left[i], lower_right[i + 1], brace_radius * 0.9, material)
        _add_member(part, lower_right[i], lower_left[i + 1], brace_radius * 0.9, material)
        _add_member(part, upper_left[i], upper_right[i + 1], brace_radius * 0.9, material)
        _add_member(part, upper_right[i], upper_left[i + 1], brace_radius * 0.9, material)

    return {
        "lower_left": lower_left,
        "lower_right": lower_right,
        "upper_left": upper_left,
        "upper_right": upper_right,
    }


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.24, 0.0, -1.68),
            (0.48, 0.0, -2.12),
            (0.52, 0.0, -2.70),
            (0.22, 0.0, -3.18),
            (-0.24, 0.0, -3.40),
            (-0.60, 0.0, -2.96),
            (-0.48, 0.0, -2.26),
        ],
        radius=0.08,
        samples_per_segment=20,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, "tower_crane_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.93, 0.78, 0.14, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.56, 0.58, 0.60, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    ballast = model.material("ballast", rgba=(0.48, 0.49, 0.47, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.78, 0.88, 0.35))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.15, 1.0))
    warning_red = model.material("warning_red", rgba=(0.76, 0.14, 0.11, 1.0))

    hook_mesh = _build_hook_mesh()

    mast_base = model.part("mast_base")
    mast_base.visual(
        Box((9.0, 9.0, 1.4)),
        origin=Origin(xyz=(0.0, 0.0, 0.7)),
        material=concrete,
        name="foundation_pad",
    )
    mast_base.visual(
        Box((4.4, 4.4, 1.8)),
        origin=Origin(xyz=(0.0, 0.0, 2.3)),
        material=concrete,
        name="pedestal",
    )
    mast_base.visual(
        Box((2.8, 2.8, 0.5)),
        origin=Origin(xyz=(0.0, 0.0, 3.45)),
        material=steel_dark,
        name="mast_foot_frame",
    )
    _add_square_lattice_mast(
        mast_base,
        width=2.2,
        bottom_z=3.2,
        top_z=41.6,
        panels=12,
        chord_radius=0.11,
        brace_radius=0.055,
        material=crane_yellow,
    )
    mast_base.visual(
        Cylinder(radius=1.7, length=0.4),
        origin=Origin(xyz=(0.0, 0.0, 41.8)),
        material=steel_dark,
        name="slew_ring_lower",
    )
    mast_base.inertial = Inertial.from_geometry(
        Box((9.0, 9.0, 42.0)),
        mass=420000.0,
        origin=Origin(xyz=(0.0, 0.0, 21.0)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.85, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=steel_dark,
        name="slew_ring_upper",
    )
    upperworks.visual(
        Cylinder(radius=0.75, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=crane_yellow,
        name="kingpost",
    )
    upperworks.visual(
        Box((7.88, 4.2, 0.35)),
        origin=Origin(xyz=(0.5, 0.0, 0.625)),
        material=steel_dark,
        name="machinery_deck",
    )
    upperworks.visual(
        Box((4.0, 2.4, 2.2)),
        origin=Origin(xyz=(-0.7, 0.0, 1.9)),
        material=steel_dark,
        name="machinery_house",
    )
    upperworks.visual(
        Box((2.6, 1.6, 1.8)),
        origin=Origin(xyz=(2.8, -2.9, 1.7)),
        material=steel_dark,
        name="cab_shell",
    )
    upperworks.visual(
        Box((0.08, 1.2, 1.0)),
        origin=Origin(xyz=(4.06, -2.95, 1.85)),
        material=glass,
        name="cab_glazing",
    )

    main_jib = _add_box_truss(
        upperworks,
        x_start=4.44,
        x_end=52.44,
        bottom_z=1.72,
        top_z=5.92,
        half_width=1.35,
        panels=12,
        chord_radius=0.12,
        brace_radius=0.055,
        material=crane_yellow,
    )
    upperworks.visual(
        Box((47.2, 0.18, 0.14)),
        origin=Origin(xyz=(28.2, -1.05, 1.53)),
        material=steel_mid,
        name="main_rail_left",
    )
    upperworks.visual(
        Box((47.2, 0.18, 0.14)),
        origin=Origin(xyz=(28.2, 1.05, 1.53)),
        material=steel_mid,
        name="main_rail_right",
    )
    upperworks.visual(
        Box((45.8, 0.75, 0.08)),
        origin=Origin(xyz=(28.5, 0.0, 1.58)),
        material=steel_mid,
        name="jib_walkway",
    )
    for idx in (0, 3, 6, 9, 12):
        left_node = main_jib["lower_left"][idx]
        right_node = main_jib["lower_right"][idx]
        x = left_node[0]
        _add_member(upperworks, (x, -1.05, 1.60), left_node, 0.04, steel_mid)
        _add_member(upperworks, (x, 1.05, 1.60), right_node, 0.04, steel_mid)
        _add_member(upperworks, (x, -0.375, 1.58), (x, -1.05, 1.60), 0.035, steel_mid)
        _add_member(upperworks, (x, 0.375, 1.58), (x, 1.05, 1.60), 0.035, steel_mid)

    counter_jib = _add_box_truss(
        upperworks,
        x_start=-3.44,
        x_end=-18.44,
        bottom_z=1.72,
        top_z=5.0,
        half_width=1.15,
        panels=5,
        chord_radius=0.11,
        brace_radius=0.05,
        material=crane_yellow,
    )
    upperworks.visual(
        Box((11.5, 2.3, 0.12)),
        origin=Origin(xyz=(-11.2, 0.0, 1.56)),
        material=steel_mid,
        name="counter_platform",
    )
    upperworks.visual(
        Box((2.4, 1.6, 1.4)),
        origin=Origin(xyz=(-13.6, 0.0, 2.32)),
        material=ballast,
        name="ballast_pack_1",
    )
    upperworks.visual(
        Box((2.4, 1.6, 1.4)),
        origin=Origin(xyz=(-16.1, 0.0, 2.32)),
        material=ballast,
        name="ballast_pack_2",
    )

    apex = (1.8, 0.0, 10.5)
    _add_member(upperworks, (0.8, -1.0, 0.8), apex, 0.14, crane_yellow)
    _add_member(upperworks, (0.8, 1.0, 0.8), apex, 0.14, crane_yellow)
    _add_member(
        upperworks,
        (1.531958762886598, -0.26804123711340205, 7.9),
        (1.531958762886598, 0.26804123711340205, 7.9),
        0.07,
        crane_yellow,
    )
    upperworks.visual(
        Box((0.7, 0.7, 0.55)),
        origin=Origin(xyz=apex),
        material=crane_yellow,
        name="apex_head",
    )
    _add_member(upperworks, apex, main_jib["upper_left"][7], 0.045, cable)
    _add_member(upperworks, apex, main_jib["upper_right"][-1], 0.045, cable)
    _add_member(upperworks, apex, counter_jib["upper_left"][-1], 0.045, cable)
    _add_member(upperworks, apex, counter_jib["upper_right"][-1], 0.045, cable)
    upperworks.inertial = Inertial.from_geometry(
        Box((72.0, 6.0, 11.0)),
        mass=185000.0,
        origin=Origin(xyz=(17.0, 0.0, 3.2)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.6, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, -1.05, -0.07)),
        material=steel_dark,
        name="left_carriage",
    )
    trolley.visual(
        Box((0.6, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, 1.05, -0.07)),
        material=steel_dark,
        name="right_carriage",
    )
    trolley.visual(
        Box((0.95, 2.0, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.25)),
        material=steel_dark,
        name="trolley_frame",
    )
    trolley.visual(
        Box((0.9, 1.2, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -0.52)),
        material=crane_yellow,
        name="winch_house",
    )
    trolley.visual(
        Cylinder(radius=0.18, length=1.5),
        origin=Origin(xyz=(0.0, 0.0, -0.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="hoist_drum",
    )
    trolley.visual(
        Cylinder(radius=0.03, length=13.35),
        origin=Origin(xyz=(0.0, -0.35, -6.675)),
        material=cable,
        name="left_hoist_line",
    )
    trolley.visual(
        Cylinder(radius=0.03, length=13.35),
        origin=Origin(xyz=(0.0, 0.35, -6.675)),
        material=cable,
        name="right_hoist_line",
    )
    trolley.visual(
        Cylinder(radius=0.05, length=0.8),
        origin=Origin(xyz=(0.0, 0.0, -13.3), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="lower_spreader",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((1.0, 2.2, 14.3)),
        mass=8200.0,
        origin=Origin(xyz=(0.0, 0.0, -7.0)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.22, length=0.8),
        origin=Origin(xyz=(0.0, 0.0, -0.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="head_sheave",
    )
    hook_block.visual(
        Box((1.0, 0.85, 1.3)),
        origin=Origin(xyz=(0.0, 0.0, -0.95)),
        material=crane_yellow,
        name="hook_body",
    )
    hook_block.visual(
        Cylinder(radius=0.11, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, -1.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="hook_spreader",
    )
    _add_member(
        hook_block,
        (0.0, 0.0, -1.68),
        (0.24, 0.0, -1.68),
        0.06,
        steel_mid,
        name="hook_neck",
    )
    hook_block.visual(hook_mesh, material=warning_red, name="hook")
    hook_block.inertial = Inertial.from_geometry(
        Box((1.2, 1.0, 3.5)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, -1.75)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.REVOLUTE,
        parent=mast_base,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 42.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900000.0,
            velocity=0.25,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(4.8, 0.0, 1.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120000.0,
            velocity=1.5,
            lower=0.0,
            upper=44.0,
        ),
    )
    model.articulation(
        "hook_suspension",
        ArticulationType.FIXED,
        parent=trolley,
        child=hook_block,
        origin=Origin(xyz=(0.0, 0.0, -13.35)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast_base = object_model.get_part("mast_base")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    hook_block = object_model.get_part("hook_block")
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
        "slewing_joint_axis_is_vertical",
        slewing_rotation.axis == (0.0, 0.0, 1.0),
        details=f"axis={slewing_rotation.axis}",
    )
    ctx.check(
        "trolley_joint_axis_runs_along_jib",
        trolley_travel.axis == (1.0, 0.0, 0.0),
        details=f"axis={trolley_travel.axis}",
    )
    ctx.expect_origin_gap(
        upperworks,
        mast_base,
        axis="z",
        min_gap=41.95,
        max_gap=42.05,
        name="upperworks_sit_at_mast_top",
    )
    ctx.expect_contact(
        upperworks,
        mast_base,
        elem_a="slew_ring_upper",
        elem_b="slew_ring_lower",
        contact_tol=1e-5,
        name="slewing_ring_is_seated",
    )
    ctx.expect_contact(
        trolley,
        upperworks,
        elem_a="left_carriage",
        elem_b="main_rail_left",
        contact_tol=1e-5,
        name="left_trolley_carriage_contacts_left_rail",
    )
    ctx.expect_contact(
        trolley,
        upperworks,
        elem_a="right_carriage",
        elem_b="main_rail_right",
        contact_tol=1e-5,
        name="right_trolley_carriage_contacts_right_rail",
    )
    ctx.expect_origin_gap(
        trolley,
        upperworks,
        axis="x",
        min_gap=4.75,
        max_gap=4.85,
        name="trolley_starts_on_inner_jib",
    )
    ctx.expect_gap(
        trolley,
        hook_block,
        axis="z",
        min_gap=12.0,
        max_gap=15.0,
        positive_elem="trolley_frame",
        negative_elem="head_sheave",
        name="hook_block_hangs_below_trolley",
    )

    with ctx.pose({trolley_travel: 44.0}):
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="left_carriage",
            elem_b="main_rail_left",
            contact_tol=1e-5,
            name="left_trolley_carriage_stays_on_left_rail_at_tip_pose",
        )
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="right_carriage",
            elem_b="main_rail_right",
            contact_tol=1e-5,
            name="right_trolley_carriage_stays_on_right_rail_at_tip_pose",
        )
        ctx.expect_origin_gap(
            trolley,
            upperworks,
            axis="x",
            min_gap=48.75,
            max_gap=48.85,
            name="trolley_reaches_outboard_jib_span",
        )

    with ctx.pose({slewing_rotation: 0.9, trolley_travel: 22.0}):
        ctx.expect_origin_distance(
            trolley,
            mast_base,
            axes="xy",
            min_dist=26.0,
            max_dist=27.5,
            name="slewed_trolley_orbits_mast",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
