from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
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


ASSETS = AssetContext.from_script(__file__)


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


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
    name: str | None = None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


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
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_member(part, (x0, y0, z), (x1, y1, z), brace_radius, material)

    for z0, z1 in zip(levels[:-1], levels[1:]):
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
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


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def _build_hook_mesh():
    mesh_path = ASSETS.mesh_path("tower_crane_hook.obj")
    hook_geom = tube_from_spline_points(
        [
            (0.00, 0.0, 0.00),
            (0.11, 0.0, -0.16),
            (0.18, 0.0, -0.34),
            (0.20, 0.0, -0.55),
            (0.12, 0.0, -0.74),
            (-0.02, 0.0, -0.86),
            (-0.18, 0.0, -0.80),
            (-0.24, 0.0, -0.64),
            (-0.18, 0.0, -0.46),
        ],
        radius=0.055,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, mesh_path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luffing_jib_tower_crane", assets=ASSETS)

    tower_yellow = model.material("tower_yellow", rgba=(0.90, 0.74, 0.16, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.24, 0.25, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.59, 0.61, 1.0))
    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    ballast = model.material("ballast", rgba=(0.48, 0.49, 0.50, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.15, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.62, 0.77, 0.86, 0.45))
    safety_red = model.material("safety_red", rgba=(0.76, 0.16, 0.10, 1.0))
    hook_mesh = _build_hook_mesh()

    mast = model.part("mast")
    _add_box(mast, (6.0, 6.0, 1.2), (0.0, 0.0, 0.6), material=concrete, name="foundation_slab")
    _add_box(mast, (2.2, 2.2, 1.0), (0.0, 0.0, 1.7), material=dark_grey, name="base_pedestal")
    _add_square_mast(
        mast,
        width=2.0,
        bottom_z=2.2,
        top_z=28.2,
        panels=9,
        chord_radius=0.11,
        brace_radius=0.065,
        material=tower_yellow,
    )
    _add_box(mast, (3.0, 3.0, 0.5), (0.0, 0.0, 28.45), material=dark_grey, name="mast_head_platform")
    mast.inertial = Inertial.from_geometry(
        Box((6.0, 6.0, 30.0)),
        mass=120000.0,
        origin=Origin(xyz=(0.0, 0.0, 15.0)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=1.4, length=0.4),
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        material=dark_grey,
        name="slew_ring",
    )
    _add_box(upperworks, (5.4, 3.1, 0.4), (-0.05, 0.0, 0.55), material=dark_grey, name="machinery_deck")
    _add_box(upperworks, (1.0, 1.0, 0.55), (2.25, 0.0, 0.78), material=dark_grey, name="front_knuckle")
    _add_box(upperworks, (0.8, 1.0, 0.18), (2.65, 0.0, 1.05), material=steel, name="jib_hinge_crosshead")
    _add_box(upperworks, (0.36, 0.06, 0.90), (2.92, 0.53, 1.35), material=steel, name="left_hinge_plate")
    _add_box(upperworks, (0.36, 0.06, 0.90), (2.92, -0.53, 1.35), material=steel, name="right_hinge_plate")

    _add_box(upperworks, (1.8, 2.0, 1.4), (-1.6, 0.0, 1.45), material=dark_grey, name="machinery_house")
    _add_box(upperworks, (0.9, 0.8, 0.8), (1.3, -1.45, 1.15), material=dark_grey, name="operator_cab_shell")
    _add_box(upperworks, (0.76, 0.70, 0.62), (1.36, -1.45, 1.18), material=cab_glass, name="operator_cab_glass")

    counter_truss = _add_triangular_truss(
        upperworks,
        x_start=-0.6,
        x_end=-6.3,
        bottom_z=0.78,
        half_width=0.55,
        root_top_z=2.6,
        tip_top_z=1.2,
        panels=4,
        chord_radius=0.09,
        brace_radius=0.05,
        material=tower_yellow,
    )
    _add_box(upperworks, (2.2, 1.2, 0.6), (-3.8, 0.0, 1.05), material=ballast, name="counterweight_pack")
    _add_box(upperworks, (1.2, 1.0, 0.6), (-5.0, 0.0, 1.05), material=ballast)

    a_frame_apex = (-0.9, 0.0, 5.2)
    _add_member(upperworks, (-0.2, -0.7, 0.75), a_frame_apex, 0.12, tower_yellow)
    _add_member(upperworks, (-0.2, 0.7, 0.75), a_frame_apex, 0.12, tower_yellow)
    _add_member(upperworks, (-1.6, -0.55, 1.0), a_frame_apex, 0.08, tower_yellow)
    _add_member(upperworks, (-1.6, 0.55, 1.0), a_frame_apex, 0.08, tower_yellow)
    _add_box(upperworks, (0.5, 0.5, 0.35), a_frame_apex, material=tower_yellow, name="a_frame_head")
    _add_member(upperworks, a_frame_apex, counter_truss["upper"][-1], 0.04, cable)
    upperworks.inertial = Inertial.from_geometry(
        Box((12.0, 4.5, 6.0)),
        mass=65000.0,
        origin=Origin(xyz=(-1.2, 0.0, 1.8)),
    )

    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.16, length=1.0),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    _add_box(jib, (1.30, 0.72, 1.70), (0.81, 0.0, 0.425), material=dark_grey, name="root_knuckle")
    jib_truss = _add_triangular_truss(
        jib,
        x_start=1.1,
        x_end=18.0,
        bottom_z=-0.35,
        half_width=0.45,
        root_top_z=1.2,
        tip_top_z=0.40,
        panels=9,
        chord_radius=0.10,
        brace_radius=0.055,
        material=tower_yellow,
    )
    _add_box(jib, (0.90, 0.90, 0.18), (18.00, 0.0, 0.58), material=dark_grey, name="tip_frame")
    _add_box(jib, (0.24, 0.06, 0.30), (18.22, 0.21, 0.0), material=steel, name="tip_left_plate")
    _add_box(jib, (0.24, 0.06, 0.30), (18.22, -0.21, 0.0), material=steel, name="tip_right_plate")
    _add_member(jib, jib_truss["upper"][-2], (18.3, 0.0, 0.32), 0.06, steel)
    _add_member(jib, jib_truss["lower_left"][-2], (18.18, -0.25, -0.10), 0.06, steel)
    _add_member(jib, jib_truss["lower_right"][-2], (18.18, 0.25, -0.10), 0.06, steel)
    jib.inertial = Inertial.from_geometry(
        Box((18.8, 1.5, 2.2)),
        mass=28000.0,
        origin=Origin(xyz=(9.4, 0.0, 0.25)),
    )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.05, length=0.36),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hanger_barrel",
    )
    _add_box(hook, (0.18, 0.26, 0.20), (0.0, 0.0, -0.13), material=steel, name="hanger_head")
    hook.visual(
        Cylinder(radius=0.035, length=5.6),
        origin=Origin(xyz=(0.0, 0.0, -2.93)),
        material=cable,
        name="hook_cable",
    )
    _add_box(hook, (0.70, 0.55, 0.70), (0.0, 0.0, -6.08), material=tower_yellow, name="hook_block")
    _add_box(hook, (0.55, 0.40, 0.40), (0.0, 0.0, -6.63), material=steel, name="lower_yoke")
    hook.visual(
        Cylinder(radius=0.05, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, -7.105)),
        material=steel,
        name="hook_stem",
    )
    hook.visual(
        hook_mesh,
        origin=Origin(xyz=(0.0, 0.0, -6.84)),
        material=safety_red,
        name="hook_tip",
    )
    hook.inertial = Inertial.from_geometry(
        Box((0.9, 0.8, 7.8)),
        mass=6000.0,
        origin=Origin(xyz=(0.0, 0.0, -3.9)),
    )

    model.articulation(
        "mast_to_upperworks",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 28.7)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200000.0, velocity=0.20),
    )
    model.articulation(
        "upperworks_to_jib",
        ArticulationType.REVOLUTE,
        parent=upperworks,
        child=jib,
        origin=Origin(xyz=(2.92, 0.0, 1.35)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160000.0,
            velocity=0.22,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "jib_to_hook",
        ArticulationType.REVOLUTE,
        parent=jib,
        child=hook,
        origin=Origin(xyz=(18.22, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10000.0,
            velocity=0.8,
            lower=-1.20,
            upper=1.20,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    upperworks = object_model.get_part("upperworks")
    jib = object_model.get_part("jib")
    hook = object_model.get_part("hook")

    mast_to_upperworks = object_model.get_articulation("mast_to_upperworks")
    upperworks_to_jib = object_model.get_articulation("upperworks_to_jib")
    jib_to_hook = object_model.get_articulation("jib_to_hook")

    foundation_slab = mast.get_visual("foundation_slab")
    mast_head_platform = mast.get_visual("mast_head_platform")
    slew_ring = upperworks.get_visual("slew_ring")
    left_hinge_plate = upperworks.get_visual("left_hinge_plate")
    right_hinge_plate = upperworks.get_visual("right_hinge_plate")
    hinge_barrel = jib.get_visual("hinge_barrel")
    tip_frame = jib.get_visual("tip_frame")
    tip_left_plate = jib.get_visual("tip_left_plate")
    tip_right_plate = jib.get_visual("tip_right_plate")
    hanger_barrel = hook.get_visual("hanger_barrel")
    hook_block = hook.get_visual("hook_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        mast,
        mast,
        axis="z",
        min_gap=26.5,
        positive_elem=mast_head_platform,
        negative_elem=foundation_slab,
        name="mast_head_sits_far_above_foundation",
    )
    ctx.expect_gap(
        upperworks,
        mast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=slew_ring,
        negative_elem=mast_head_platform,
        name="slew_ring_bears_on_mast_head",
    )
    ctx.expect_gap(
        upperworks,
        jib,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_hinge_plate,
        negative_elem=hinge_barrel,
        name="left_jib_hinge_plate_touches_barrel",
    )
    ctx.expect_gap(
        jib,
        upperworks,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hinge_barrel,
        negative_elem=right_hinge_plate,
        name="right_jib_hinge_plate_touches_barrel",
    )
    ctx.expect_gap(
        jib,
        upperworks,
        axis="x",
        min_gap=15.0,
        positive_elem=tip_frame,
        negative_elem=slew_ring,
        name="jib_reaches_far_forward_of_tower",
    )
    ctx.expect_gap(
        jib,
        hook,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tip_left_plate,
        negative_elem=hanger_barrel,
        name="left_tip_plate_supports_hook_hanger",
    )
    ctx.expect_gap(
        hook,
        jib,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hanger_barrel,
        negative_elem=tip_right_plate,
        name="right_tip_plate_supports_hook_hanger",
    )
    ctx.expect_gap(
        jib,
        hook,
        axis="z",
        min_gap=5.2,
        positive_elem=tip_frame,
        negative_elem=hook_block,
        name="hook_block_hangs_below_jib_tip",
    )

    rest_tip_center = _aabb_center(ctx.part_element_world_aabb(jib, elem=tip_frame))
    ctx.check(
        "rest_jib_points_forward",
        rest_tip_center is not None and rest_tip_center[0] > 20.0 and abs(rest_tip_center[1]) < 0.5,
        details=f"tip center at rest was {rest_tip_center}",
    )

    luff_limits = upperworks_to_jib.motion_limits
    if luff_limits is not None and luff_limits.upper is not None:
        with ctx.pose({upperworks_to_jib: luff_limits.upper, jib_to_hook: luff_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="luffed_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="luffed_pose_no_floating")
            ctx.expect_gap(
                jib,
                mast,
                axis="z",
                min_gap=13.0,
                positive_elem=tip_frame,
                negative_elem=mast_head_platform,
                name="luffed_jib_tip_rises_high_above_mast_head",
            )
            ctx.expect_gap(
                jib,
                hook,
                axis="z",
                min_gap=5.0,
                positive_elem=tip_frame,
                negative_elem=hook_block,
                name="hook_remains_below_tip_when_jib_luffs",
            )

            luffed_tip_center = _aabb_center(ctx.part_element_world_aabb(jib, elem=tip_frame))
            luffed_hook_center = _aabb_center(ctx.part_element_world_aabb(hook, elem=hook_block))
            hangs_under_tip = (
                luffed_tip_center is not None
                and luffed_hook_center is not None
                and abs(luffed_tip_center[0] - luffed_hook_center[0]) < 1.0
                and abs(luffed_tip_center[1] - luffed_hook_center[1]) < 0.35
                and luffed_hook_center[2] < luffed_tip_center[2] - 5.0
            )
            ctx.check(
                "hook_tracks_under_tip_in_luffed_pose",
                hangs_under_tip,
                details=f"tip center={luffed_tip_center}, hook center={luffed_hook_center}",
            )

    hook_limits = jib_to_hook.motion_limits
    if hook_limits is not None and hook_limits.lower is not None and hook_limits.upper is not None:
        with ctx.pose({jib_to_hook: hook_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hook_swing_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="hook_swing_lower_no_floating")
        with ctx.pose({jib_to_hook: hook_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="hook_swing_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="hook_swing_upper_no_floating")

    with ctx.pose({mast_to_upperworks: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slewed_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="slewed_pose_no_floating")
        ctx.expect_gap(
            upperworks,
            mast,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=slew_ring,
            negative_elem=mast_head_platform,
            name="slew_ring_stays_seated_during_rotation",
        )
        slewed_tip_center = _aabb_center(ctx.part_element_world_aabb(jib, elem=tip_frame))
        ctx.check(
            "slewing_swings_jib_sideways",
            slewed_tip_center is not None and abs(slewed_tip_center[1]) > 18.0 and abs(slewed_tip_center[0]) < 4.0,
            details=f"tip center in slewed pose was {slewed_tip_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
