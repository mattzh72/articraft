from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
HERE = ASSETS.asset_root
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


def _add_member(part, a, b, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
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
            _add_member(part, (x0, y0, z0), (x1, y1, z1), brace_radius, material)
            _add_member(part, (x1, y1, z0), (x0, y0, z1), brace_radius, material)
        _add_member(part, (half, half, z0), (-half, -half, z1), brace_radius * 0.9, material)
        _add_member(part, (half, -half, z0), (-half, half, z1), brace_radius * 0.9, material)

    ladder_x = half + 0.004
    rail_y = 0.022
    ladder_bottom = bottom_z + 0.09
    ladder_top = top_z - 0.08
    _add_member(
        part,
        (ladder_x, -rail_y, ladder_bottom),
        (ladder_x, -rail_y, ladder_top),
        0.0035,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, rail_y, ladder_bottom),
        (ladder_x, rail_y, ladder_top),
        0.0035,
        ladder_material,
    )
    rung_count = 16
    for i in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * i / rung_count
        _add_member(part, (ladder_x, -rail_y, z), (ladder_x, rail_y, z), 0.0026, ladder_material)


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
        if abs(span) < 1e-9:
            t = 0.0
        else:
            t = (x - x_start) / span
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
    mesh_dir = ASSETS.mesh_dir
    mesh_dir.mkdir(parents=True, exist_ok=True)
    hook_geom = tube_from_spline_points(
        [
            (0.018, 0.0, -0.125),
            (0.034, 0.0, -0.155),
            (0.036, 0.0, -0.192),
            (0.018, 0.0, -0.226),
            (-0.012, 0.0, -0.236),
            (-0.034, 0.0, -0.212),
            (-0.028, 0.0, -0.176),
        ],
        radius=0.0065,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(hook_geom, mesh_dir / "tower_crane_hook.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane", assets=ASSETS)

    tower_yellow = model.material("tower_yellow", rgba=(0.89, 0.75, 0.16, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.61, 1.0))
    ballast = model.material("ballast", rgba=(0.50, 0.50, 0.48, 1.0))
    cable = model.material("cable", rgba=(0.15, 0.15, 0.16, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.66, 0.80, 0.88, 0.35))
    safety_red = model.material("safety_red", rgba=(0.75, 0.14, 0.10, 1.0))

    hook_mesh = _build_hook_mesh()

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((0.60, 0.60, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=concrete
    )
    tower_base.visual(
        Box((0.34, 0.34, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.13)), material=concrete
    )
    tower_base.visual(
        Box((0.22, 0.22, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=dark_grey
    )
    for sx in (-0.07, 0.07):
        for sy in (-0.07, 0.07):
            tower_base.visual(
                Cylinder(radius=0.010, length=0.08),
                origin=Origin(xyz=(sx, sy, 0.14)),
                material=steel,
            )
    _add_square_mast(
        tower_base,
        width=0.18,
        bottom_z=0.20,
        top_z=1.58,
        panels=6,
        chord_radius=0.0085,
        brace_radius=0.0055,
        material=tower_yellow,
        ladder_material=steel,
    )
    tower_base.visual(
        Cylinder(radius=0.15, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 1.59)),
        material=dark_grey,
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((0.60, 0.60, 1.65)),
        mass=35.0,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=0.16, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_grey,
    )
    upperworks.visual(
        Cylinder(radius=0.05, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=tower_yellow,
    )
    upperworks.visual(
        Box((0.42, 0.26, 0.05)), origin=Origin(xyz=(0.02, 0.0, 0.065)), material=dark_grey
    )
    upperworks.visual(
        Box((0.16, 0.12, 0.02)), origin=Origin(xyz=(0.24, 0.0, 0.105)), material=steel
    )
    upperworks.visual(
        Box((0.10, 0.08, 0.02)), origin=Origin(xyz=(0.42, 0.0, 0.118)), material=tower_yellow
    )

    upperworks.visual(
        Box((0.30, 0.14, 0.13)), origin=Origin(xyz=(-0.16, 0.0, 0.145)), material=dark_grey
    )
    upperworks.visual(
        Box((0.14, 0.10, 0.10)), origin=Origin(xyz=(0.18, -0.12, 0.14)), material=dark_grey
    )
    upperworks.visual(
        Box((0.125, 0.086, 0.084)), origin=Origin(xyz=(0.19, -0.12, 0.142)), material=cab_glass
    )

    jib = _add_triangular_truss(
        upperworks,
        x_start=0.18,
        x_end=1.92,
        bottom_z=0.115,
        half_width=0.06,
        root_top_z=0.29,
        tip_top_z=0.20,
        panels=8,
        chord_radius=0.009,
        brace_radius=0.0055,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((1.72, 0.12, 0.012)), origin=Origin(xyz=(1.05, 0.0, 0.106)), material=steel
    )
    _add_member(upperworks, (0.18, -0.03, 0.118), (1.86, -0.03, 0.118), 0.005, steel)
    _add_member(upperworks, (0.18, 0.03, 0.118), (1.86, 0.03, 0.118), 0.005, steel)

    counter_jib = _add_triangular_truss(
        upperworks,
        x_start=-0.18,
        x_end=-0.92,
        bottom_z=0.115,
        half_width=0.05,
        root_top_z=0.30,
        tip_top_z=0.18,
        panels=4,
        chord_radius=0.0085,
        brace_radius=0.0052,
        material=tower_yellow,
    )
    upperworks.visual(
        Box((0.74, 0.10, 0.012)), origin=Origin(xyz=(-0.55, 0.0, 0.106)), material=steel
    )
    _add_member(upperworks, (-0.22, -0.04, 0.155), (-0.22, 0.04, 0.155), 0.035, steel)
    _add_member(upperworks, (-0.31, -0.04, 0.155), (-0.31, 0.04, 0.155), 0.028, steel)
    upperworks.visual(
        Box((0.14, 0.12, 0.10)), origin=Origin(xyz=(-0.60, 0.0, 0.162)), material=ballast
    )
    upperworks.visual(
        Box((0.14, 0.12, 0.10)), origin=Origin(xyz=(-0.74, 0.0, 0.162)), material=ballast
    )
    upperworks.visual(
        Box((0.10, 0.10, 0.08)), origin=Origin(xyz=(-0.86, 0.0, 0.152)), material=ballast
    )

    apex = (0.05, 0.0, 0.45)
    _add_member(upperworks, (-0.05, -0.06, 0.10), apex, 0.011, tower_yellow)
    _add_member(upperworks, (-0.05, 0.06, 0.10), apex, 0.011, tower_yellow)
    _add_member(upperworks, (0.05, -0.03, 0.34), (0.05, 0.03, 0.34), 0.007, tower_yellow)
    upperworks.visual(Box((0.05, 0.05, 0.04)), origin=Origin(xyz=apex), material=tower_yellow)

    _add_member(upperworks, apex, jib["upper"][5], 0.004, cable)
    _add_member(upperworks, apex, jib["upper"][-1], 0.004, cable)
    _add_member(upperworks, apex, counter_jib["upper"][-1], 0.004, cable)

    upperworks.inertial = Inertial.from_geometry(
        Box((2.90, 0.42, 0.55)),
        mass=10.0,
        origin=Origin(xyz=(0.50, 0.0, 0.20)),
    )

    trolley = model.part("trolley")
    trolley.visual(Box((0.16, 0.12, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.02)), material=dark_grey)
    trolley.visual(Box((0.09, 0.08, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.055)), material=steel)
    trolley.visual(
        Box((0.10, 0.06, 0.03)), origin=Origin(xyz=(0.0, 0.0, -0.002)), material=tower_yellow
    )
    trolley.visual(Box((0.11, 0.07, 0.045)), origin=Origin(xyz=(0.0, 0.0, -0.024)), material=steel)
    trolley.visual(
        Cylinder(radius=0.018, length=0.11), origin=Origin(xyz=(0.0, 0.0, 0.01)), material=steel
    )
    _add_member(trolley, (-0.045, -0.052, -0.006), (-0.045, -0.020, -0.006), 0.0065, steel)
    _add_member(trolley, (-0.045, 0.020, -0.006), (-0.045, 0.052, -0.006), 0.0065, steel)
    _add_member(trolley, (0.045, -0.052, -0.006), (0.045, -0.020, -0.006), 0.0065, steel)
    _add_member(trolley, (0.045, 0.020, -0.006), (0.045, 0.052, -0.006), 0.0065, steel)
    trolley.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.0, -0.016, -0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
    )
    trolley.visual(
        Cylinder(radius=0.014, length=0.052),
        origin=Origin(xyz=(0.0, 0.016, -0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
    )
    trolley.visual(
        Cylinder(radius=0.0028, length=1.03),
        origin=Origin(xyz=(0.0, -0.016, -0.55)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.0028, length=1.03),
        origin=Origin(xyz=(0.0, 0.016, -0.55)),
        material=cable,
    )
    trolley.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -1.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 1.08)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Cylinder(radius=0.014, length=0.04), origin=Origin(xyz=(0.0, 0.0, -0.02)), material=steel
    )
    hook_block.visual(
        Box((0.05, 0.05, 0.03)), origin=Origin(xyz=(0.0, 0.0, -0.055)), material=tower_yellow
    )
    hook_block.visual(
        Box((0.07, 0.05, 0.11)), origin=Origin(xyz=(0.0, 0.0, -0.12)), material=tower_yellow
    )
    _add_member(hook_block, (0.0, -0.02, -0.165), (0.0, 0.02, -0.165), 0.016, steel)
    hook_block.visual(hook_mesh, material=safety_red)
    hook_block.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.24)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    model.articulation(
        "slewing_rotation",
        ArticulationType.CONTINUOUS,
        parent="tower_base",
        child="upperworks",
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent="upperworks",
        child="trolley",
        origin=Origin(xyz=(0.42, 0.0, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=0.0, upper=1.08),
    )
    model.articulation(
        "hook_suspension",
        ArticulationType.FIXED,
        parent="trolley",
        child="hook_block",
        origin=Origin(xyz=(0.0, 0.0, -1.03)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "tower_base",
        "upperworks",
        reason="slewing ring bearing collar is tightly nested and conservative hulls can report slight penetration",
    )
    ctx.allow_overlap(
        "upperworks",
        "trolley",
        reason="trolley wheel assemblies ride directly on jib rails and generated collisions are conservative around the running gear",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("upperworks", "tower_base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_gap("upperworks", "tower_base", axis="z", max_gap=0.002, max_penetration=0.002)
    ctx.expect_aabb_contact("trolley", "upperworks")
    ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.012)
    ctx.expect_joint_motion_axis(
        "trolley_travel", "trolley", world_axis="x", direction="positive", min_delta=0.40
    )

    with ctx.pose({"trolley_travel": 0.0}):
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.012)
        ctx.expect_aabb_overlap("hook_block", "trolley", axes="xy", min_overlap=0.04)

    with ctx.pose({"trolley_travel": 1.05}):
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.012)
        ctx.expect_aabb_overlap("hook_block", "trolley", axes="xy", min_overlap=0.04)

    with ctx.pose({"slewing_rotation": math.pi / 2.0, "trolley_travel": 0.90}):
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.015)
        ctx.expect_aabb_gap(
            "upperworks", "tower_base", axis="z", max_gap=0.002, max_penetration=0.002
        )

    with ctx.pose({"slewing_rotation": 0.0, "trolley_travel": 0.90}):
        trolley_east = ctx.part_world_position("trolley")
        hook_east = ctx.part_world_position("hook_block")

    with ctx.pose({"slewing_rotation": math.pi / 2.0, "trolley_travel": 0.90}):
        trolley_north = ctx.part_world_position("trolley")
        hook_north = ctx.part_world_position("hook_block")

    east_radius = math.hypot(trolley_east[0], trolley_east[1])
    north_radius = math.hypot(trolley_north[0], trolley_north[1])
    assert abs(east_radius - north_radius) < 0.03, (
        "Trolley should keep the same swing radius while slewing"
    )
    assert trolley_east[0] > 1.25 and abs(trolley_east[1]) < 0.08, (
        "Jib should face +X at zero slewing angle"
    )
    assert trolley_north[1] > 1.25 and abs(trolley_north[0]) < 0.08, (
        "Jib should rotate to +Y at ninety degrees slewing"
    )
    assert hook_east[0] > 1.20 and abs(hook_east[1]) < 0.08, (
        "Hook should follow the jib heading at zero slewing angle"
    )
    assert hook_north[1] > 1.20 and abs(hook_north[0]) < 0.08, (
        "Hook should follow the jib heading after slewing"
    )

    with ctx.pose({"trolley_travel": 0.0}):
        hook_inboard = ctx.part_world_position("hook_block")
        trolley_inboard = ctx.part_world_position("trolley")

    with ctx.pose({"trolley_travel": 1.05}):
        hook_outboard = ctx.part_world_position("hook_block")
        trolley_outboard = ctx.part_world_position("trolley")

    assert trolley_outboard[0] > trolley_inboard[0] + 1.0, (
        "Trolley should travel substantially along the jib"
    )
    assert hook_outboard[0] > hook_inboard[0] + 1.0, (
        "Hook block should travel outward together with the trolley"
    )
    assert 0.95 < (trolley_inboard[2] - hook_inboard[2]) < 1.15, (
        "Hook block should hang about one meter below the trolley"
    )
    assert 0.95 < (trolley_outboard[2] - hook_outboard[2]) < 1.15, (
        "Hook block should remain consistently suspended while the trolley moves"
    )
    assert hook_outboard[2] > 0.35, "Suspended hook block should remain safely above the foundation"

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
