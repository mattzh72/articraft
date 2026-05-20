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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
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
        _add_member(
            part,
            (x, y, bottom_z),
            (x, y, top_z),
            radius=chord_radius,
            material=material,
        )

    for z in levels:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), radius=brace_radius, material=material)

    for i in range(panels):
        z0 = levels[i]
        z1 = levels[i + 1]
        for j in range(4):
            x0, y0 = corners[j]
            x1, y1 = corners[(j + 1) % 4]
            _add_member(part, (x0, y0, z0), (x1, y1, z1), radius=brace_radius, material=material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), radius=brace_radius, material=material)

    ladder_x = half + 0.006
    ladder_y = width * 0.16
    ladder_bottom = bottom_z + 0.18
    ladder_top = top_z - 0.14
    _add_member(
        part,
        (ladder_x, -ladder_y, ladder_bottom),
        (ladder_x, -ladder_y, ladder_top),
        radius=0.004,
        material=ladder_material,
    )
    _add_member(
        part,
        (ladder_x, ladder_y, ladder_bottom),
        (ladder_x, ladder_y, ladder_top),
        radius=0.004,
        material=ladder_material,
    )
    rung_count = 18
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(
            part,
            (ladder_x, -ladder_y, z),
            (ladder_x, ladder_y, z),
            radius=0.003,
            material=ladder_material,
        )


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
        _add_member(part, lower_left[i], lower_left[i + 1], radius=chord_radius, material=material)
        _add_member(
            part, lower_right[i], lower_right[i + 1], radius=chord_radius, material=material
        )
        _add_member(part, upper[i], upper[i + 1], radius=chord_radius, material=material)

    for i in range(panels + 1):
        _add_member(part, lower_left[i], lower_right[i], radius=brace_radius, material=material)
        _add_member(part, lower_left[i], upper[i], radius=brace_radius, material=material)
        _add_member(part, lower_right[i], upper[i], radius=brace_radius, material=material)

    for i in range(panels):
        if i % 2 == 0:
            _add_member(part, lower_left[i], upper[i + 1], radius=brace_radius, material=material)
            _add_member(part, lower_right[i], upper[i + 1], radius=brace_radius, material=material)
        else:
            _add_member(part, upper[i], lower_left[i + 1], radius=brace_radius, material=material)
            _add_member(part, upper[i], lower_right[i + 1], radius=brace_radius, material=material)

    return {"lower_left": lower_left, "lower_right": lower_right, "upper": upper}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane")

    tower_yellow = model.material("tower_yellow", rgba=(0.92, 0.76, 0.15, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.64, 1.0))
    concrete = model.material("concrete", rgba=(0.65, 0.65, 0.63, 1.0))
    ballast = model.material("ballast", rgba=(0.48, 0.48, 0.47, 1.0))
    cable = model.material("cable", rgba=(0.13, 0.13, 0.14, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.70, 0.84, 0.90, 0.40))

    mast = model.part("mast")
    mast.visual(
        Box((0.90, 0.90, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=concrete,
        name="foundation",
    )
    mast.visual(
        Box((0.42, 0.42, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=dark_grey,
        name="pedestal",
    )
    _add_square_mast(
        mast,
        width=0.22,
        bottom_z=0.26,
        top_z=4.82,
        panels=9,
        chord_radius=0.010,
        brace_radius=0.006,
        material=tower_yellow,
        ladder_material=steel,
    )
    mast.visual(
        Cylinder(radius=0.19, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 4.86)),
        material=dark_grey,
        name="slewing_ring_lower",
    )
    mast.visual(
        Box((0.34, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 4.79)),
        material=steel,
        name="mast_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.90, 0.90, 4.98)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 2.49)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=0.20, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_grey,
        name="slewing_ring_upper",
    )
    upperworks.visual(
        Cylinder(radius=0.06, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=tower_yellow,
        name="slew_core",
    )
    upperworks.visual(
        Box((0.62, 0.36, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, 0.15)),
        material=dark_grey,
        name="machinery_deck",
    )
    upperworks.visual(
        Box((0.26, 0.18, 0.14)),
        origin=Origin(xyz=(-0.22, 0.0, 0.22)),
        material=dark_grey,
        name="counter_machinery_house",
    )
    upperworks.visual(
        Box((0.20, 0.13, 0.11)),
        origin=Origin(xyz=(0.24, -0.16, 0.20)),
        material=dark_grey,
        name="operator_cab_shell",
    )
    upperworks.visual(
        Box((0.18, 0.11, 0.09)),
        origin=Origin(xyz=(0.25, -0.16, 0.205)),
        material=cab_glass,
        name="operator_cab_glass",
    )

    jib = _add_triangular_truss(
        upperworks,
        x_start=0.28,
        x_end=3.45,
        bottom_z=0.28,
        half_width=0.08,
        root_top_z=0.66,
        tip_top_z=0.42,
        panels=10,
        chord_radius=0.010,
        brace_radius=0.006,
        material=tower_yellow,
    )
    counter_jib = _add_triangular_truss(
        upperworks,
        x_start=-0.26,
        x_end=-1.52,
        bottom_z=0.28,
        half_width=0.07,
        root_top_z=0.60,
        tip_top_z=0.36,
        panels=5,
        chord_radius=0.009,
        brace_radius=0.0055,
        material=tower_yellow,
    )

    upperworks.visual(
        Box((2.66, 0.018, 0.016)),
        origin=Origin(xyz=(1.80, -0.05, 0.228)),
        material=steel,
        name="jib_track_left",
    )
    upperworks.visual(
        Box((2.66, 0.018, 0.016)),
        origin=Origin(xyz=(1.80, 0.05, 0.228)),
        material=steel,
        name="jib_track_right",
    )
    upperworks.visual(
        Box((0.32, 0.06, 0.014)),
        origin=Origin(xyz=(0.62, 0.0, 0.194)),
        material=steel,
        name="jib_root_station",
    )
    upperworks.visual(
        Box((0.28, 0.06, 0.014)),
        origin=Origin(xyz=(2.90, 0.0, 0.194)),
        material=steel,
        name="jib_tip_station",
    )
    _add_member(
        upperworks,
        (0.597, 0.0, 0.28),
        (0.597, 0.0, 0.199),
        radius=0.006,
        material=steel,
        name="root_station_hanger",
    )
    _add_member(
        upperworks,
        (2.816, 0.0, 0.28),
        (2.816, 0.0, 0.199),
        radius=0.006,
        material=steel,
        name="tip_station_hanger",
    )
    _add_member(
        upperworks,
        jib["lower_left"][5],
        (1.865, -0.05, 0.228),
        radius=0.005,
        material=steel,
        name="left_track_hanger",
    )
    _add_member(
        upperworks,
        jib["lower_right"][5],
        (1.865, 0.05, 0.228),
        radius=0.005,
        material=steel,
        name="right_track_hanger",
    )
    upperworks.visual(
        Box((0.18, 0.18, 0.07)),
        origin=Origin(xyz=(3.36, 0.0, 0.29)),
        material=tower_yellow,
        name="jib_tip_end_frame",
    )
    upperworks.visual(
        Box((0.22, 0.18, 0.12)),
        origin=Origin(xyz=(-0.90, 0.0, 0.23)),
        material=ballast,
        name="counterweight_block_1",
    )
    upperworks.visual(
        Box((0.22, 0.18, 0.12)),
        origin=Origin(xyz=(-1.15, 0.0, 0.23)),
        material=ballast,
        name="counterweight_block_2",
    )
    upperworks.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(-1.35, 0.0, 0.22)),
        material=ballast,
        name="counterweight_block_3",
    )

    apex = (0.08, 0.0, 1.02)
    _add_member(
        upperworks,
        (-0.06, -0.08, 0.20),
        apex,
        radius=0.012,
        material=tower_yellow,
        name="masthead_leg_left",
    )
    _add_member(
        upperworks,
        (-0.06, 0.08, 0.20),
        apex,
        radius=0.012,
        material=tower_yellow,
        name="masthead_leg_right",
    )
    _add_member(
        upperworks,
        (-0.06, -0.08, 0.20),
        (-0.06, 0.08, 0.20),
        radius=0.008,
        material=tower_yellow,
        name="masthead_base_tie",
    )
    upperworks.visual(
        Box((0.06, 0.06, 0.05)),
        origin=Origin(xyz=apex),
        material=tower_yellow,
        name="masthead_cap",
    )

    _add_member(
        upperworks,
        apex,
        jib["upper"][-1],
        radius=0.004,
        material=cable,
        name="jib_support_cable",
    )
    _add_member(
        upperworks,
        apex,
        jib["upper"][6],
        radius=0.004,
        material=cable,
        name="mid_jib_support_cable",
    )
    _add_member(
        upperworks,
        apex,
        counter_jib["upper"][-1],
        radius=0.004,
        material=cable,
        name="counter_jib_support_cable",
    )
    upperworks.inertial = Inertial.from_geometry(
        Box((5.05, 0.60, 1.08)),
        mass=16.0,
        origin=Origin(xyz=(0.95, 0.0, 0.40)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.22, 0.16, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=dark_grey,
        name="trolley_frame",
    )
    trolley.visual(
        Box((0.14, 0.10, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=steel,
        name="trolley_top_machine",
    )
    trolley.visual(
        Box((0.10, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, -0.05, -0.0475)),
        material=steel,
        name="left_shoe",
    )
    trolley.visual(
        Box((0.10, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, 0.05, -0.0475)),
        material=steel,
        name="right_shoe",
    )
    trolley.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(-0.07, -0.05, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_front_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.07, -0.05, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_rear_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(-0.07, 0.05, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_front_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.07, 0.05, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_rear_wheel",
    )
    trolley.visual(
        Cylinder(radius=0.0032, length=1.10),
        origin=Origin(xyz=(0.0, -0.012, -0.66)),
        material=cable,
        name="hoist_line_left",
    )
    trolley.visual(
        Cylinder(radius=0.0032, length=1.10),
        origin=Origin(xyz=(0.0, 0.012, -0.66)),
        material=cable,
        name="hoist_line_right",
    )
    trolley.visual(
        Box((0.10, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -1.23)),
        material=tower_yellow,
        name="hook_block",
    )
    trolley.visual(
        Cylinder(radius=0.016, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -1.306), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hook_crosshead",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 1.42)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.66)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 4.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(0.58, 0.0, 0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.80,
            lower=0.0,
            upper=2.32,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    slew = object_model.get_articulation("slewing_rotation")
    travel = object_model.get_articulation("trolley_travel")

    lower_ring = mast.get_visual("slewing_ring_lower")
    upper_ring = upperworks.get_visual("slewing_ring_upper")
    left_track = upperworks.get_visual("jib_track_left")
    right_track = upperworks.get_visual("jib_track_right")
    root_station = upperworks.get_visual("jib_root_station")
    tip_station = upperworks.get_visual("jib_tip_station")
    trolley_frame = trolley.get_visual("trolley_frame")
    left_shoe = trolley.get_visual("left_shoe")
    right_shoe = trolley.get_visual("right_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "slew_joint_is_vertical_revolute",
        slew.articulation_type == ArticulationType.REVOLUTE and tuple(slew.axis) == (0.0, 0.0, 1.0),
        details=f"type={slew.articulation_type} axis={slew.axis}",
    )
    ctx.check(
        "trolley_joint_is_longitudinal_prismatic",
        travel.articulation_type == ArticulationType.PRISMATIC and tuple(travel.axis) == (1.0, 0.0, 0.0),
        details=f"type={travel.articulation_type} axis={travel.axis}",
    )
    slew_limits = slew.motion_limits
    travel_limits = travel.motion_limits
    ctx.check(
        "slew_joint_has_realistic_range",
        slew_limits is not None
        and slew_limits.lower is not None
        and slew_limits.upper is not None
        and slew_limits.lower <= -math.pi + 1e-6
        and slew_limits.upper >= math.pi - 1e-6,
        details=f"limits={slew_limits}",
    )
    ctx.check(
        "trolley_joint_has_realistic_travel_range",
        travel_limits is not None
        and travel_limits.lower == 0.0
        and travel_limits.upper is not None
        and 2.0 <= travel_limits.upper <= 2.5,
        details=f"limits={travel_limits}",
    )

    ctx.expect_overlap(
        upperworks,
        mast,
        axes="xy",
        min_overlap=0.10,
        elem_a=upper_ring,
        elem_b=lower_ring,
    )
    ctx.expect_gap(
        upperworks,
        mast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_ring,
        negative_elem=lower_ring,
    )
    ctx.expect_contact(upperworks, mast, elem_a=upper_ring, elem_b=lower_ring)
    ctx.expect_origin_distance(upperworks, mast, axes="xy", max_dist=0.001)

    ctx.expect_gap(
        upperworks,
        trolley,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_track,
        negative_elem=left_shoe,
    )
    ctx.expect_gap(
        upperworks,
        trolley,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_track,
        negative_elem=right_shoe,
    )
    ctx.expect_overlap(
        trolley,
        upperworks,
        axes="xy",
        min_overlap=0.015,
        elem_a=trolley_frame,
        elem_b=root_station,
    )
    ctx.expect_overlap(trolley, upperworks, axes="x", min_overlap=0.10, elem_a=trolley_frame, elem_b=left_track)

    if travel_limits is not None and travel_limits.lower is not None and travel_limits.upper is not None:
        with ctx.pose({travel: travel_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="trolley_inboard_no_overlap")
            ctx.fail_if_isolated_parts(name="trolley_inboard_no_floating")
            ctx.expect_overlap(
                trolley,
                upperworks,
                axes="xy",
                min_overlap=0.015,
                elem_a=trolley_frame,
                elem_b=root_station,
                name="trolley_inboard_at_root_station",
            )
        with ctx.pose({travel: travel_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="trolley_outboard_no_overlap")
            ctx.fail_if_isolated_parts(name="trolley_outboard_no_floating")
            ctx.expect_overlap(
                trolley,
                upperworks,
                axes="xy",
                min_overlap=0.015,
                elem_a=trolley_frame,
                elem_b=tip_station,
                name="trolley_outboard_at_tip_station",
            )
            ctx.expect_gap(
                upperworks,
                trolley,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=left_track,
                negative_elem=left_shoe,
                name="trolley_outboard_left_shoe_on_track",
            )
            ctx.expect_gap(
                upperworks,
                trolley,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=right_track,
                negative_elem=right_shoe,
                name="trolley_outboard_right_shoe_on_track",
            )

    with ctx.pose({travel: 2.22}):
        ctx.expect_overlap(
            trolley,
            upperworks,
            axes="xy",
            min_overlap=0.015,
            elem_a=trolley_frame,
            elem_b=tip_station,
        )
        ctx.expect_gap(
            upperworks,
            trolley,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=left_track,
            negative_elem=left_shoe,
            name="trolley_midspan_left_shoe_on_track",
        )
        ctx.expect_gap(
            upperworks,
            trolley,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=right_track,
            negative_elem=right_shoe,
            name="trolley_midspan_right_shoe_on_track",
        )

    if slew_limits is not None and slew_limits.lower is not None and slew_limits.upper is not None:
        with ctx.pose({slew: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slew_rest_no_overlap")
            ctx.fail_if_isolated_parts(name="slew_rest_no_floating")
        with ctx.pose({slew: math.pi / 2.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slew_quarter_turn_no_overlap")
            ctx.fail_if_isolated_parts(name="slew_quarter_turn_no_floating")
            ctx.expect_overlap(
                upperworks,
                mast,
                axes="xy",
                min_overlap=0.10,
                elem_a=upper_ring,
                elem_b=lower_ring,
                name="slew_quarter_turn_ring_overlap",
            )
            ctx.expect_origin_distance(
                upperworks,
                mast,
                axes="xy",
                max_dist=0.001,
                name="slew_quarter_turn_centered",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
