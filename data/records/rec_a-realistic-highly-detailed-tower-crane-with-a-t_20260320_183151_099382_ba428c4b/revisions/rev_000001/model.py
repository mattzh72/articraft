from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math
from typing import Sequence

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
HERE = ASSETS.asset_root


def _midpoint(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _segment_rpy(a: Sequence[float], b: Sequence[float]) -> tuple[float, tuple[float, float, float]]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return 0.0, (0.0, 0.0, 0.0)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return length, (0.0, pitch, yaw)


def _add_strut(part, a: Sequence[float], b: Sequence[float], radius: float, material) -> None:
    length, rpy = _segment_rpy(a, b)
    if length <= 1e-9:
        return
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_midpoint(a, b), rpy=rpy),
        material=material,
    )


def _add_box(part, size: Sequence[float], xyz: Sequence[float], material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(tuple(size)), origin=Origin(xyz=tuple(xyz), rpy=rpy), material=material)


def _jib_top_z(x: float) -> float:
    return 0.32 - 0.075 * (x / 1.55)


def _counter_top_z(x: float) -> float:
    return 0.24 - 0.03 * (abs(x) / 0.64)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane", assets=ASSETS)

    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.74, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    light_steel = model.material("light_steel", rgba=(0.62, 0.66, 0.69, 1.0))
    ballast_gray = model.material("ballast_gray", rgba=(0.40, 0.41, 0.44, 1.0))
    concrete = model.material("concrete", rgba=(0.65, 0.65, 0.63, 1.0))
    cable_black = model.material("cable_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("cab_glass", rgba=(0.68, 0.80, 0.88, 0.42))

    mast = model.part("mast")
    _add_box(mast, (0.62, 0.62, 0.12), (0.0, 0.0, 0.06), concrete)
    _add_box(mast, (0.34, 0.34, 0.08), (0.0, 0.0, 0.16), dark_steel)
    _add_box(mast, (0.22, 0.22, 0.04), (0.0, 0.0, 1.50), dark_steel)

    mast_half = 0.08
    mast_z0 = 0.20
    mast_z1 = 1.48
    corners = [
        (-mast_half, -mast_half),
        (mast_half, -mast_half),
        (mast_half, mast_half),
        (-mast_half, mast_half),
    ]
    for x, y in corners:
        _add_strut(mast, (x, y, mast_z0), (x, y, mast_z1), 0.0105, crane_yellow)

    mast_levels = [mast_z0 + i * 0.16 for i in range(9)]
    for z in mast_levels:
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _add_strut(mast, (x0, y0, z), (x1, y1, z), 0.0058, crane_yellow)

    for z0, z1 in zip(mast_levels[:-1], mast_levels[1:]):
        for y in (-mast_half, mast_half):
            _add_strut(mast, (-mast_half, y, z0), (mast_half, y, z1), 0.0048, crane_yellow)
            _add_strut(mast, (mast_half, y, z0), (-mast_half, y, z1), 0.0048, crane_yellow)
        for x in (-mast_half, mast_half):
            _add_strut(mast, (x, -mast_half, z0), (x, mast_half, z1), 0.0048, crane_yellow)
            _add_strut(mast, (x, mast_half, z0), (x, -mast_half, z1), 0.0048, crane_yellow)

    ladder_y = -mast_half + 0.012
    ladder_z0 = mast_z0 + 0.04
    ladder_z1 = mast_z1 - 0.10
    _add_strut(mast, (-0.018, ladder_y, ladder_z0), (-0.018, ladder_y, ladder_z1), 0.0030, dark_steel)
    _add_strut(mast, (0.018, ladder_y, ladder_z0), (0.018, ladder_y, ladder_z1), 0.0030, dark_steel)
    rung_z = ladder_z0
    while rung_z <= ladder_z1:
        _add_strut(mast, (-0.018, ladder_y, rung_z), (0.018, ladder_y, rung_z), 0.0025, dark_steel)
        rung_z += 0.08

    mast.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 1.58)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(
        Cylinder(radius=0.15, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_steel,
    )
    _add_box(upperworks, (0.34, 0.30, 0.05), (0.0, 0.0, 0.055), crane_yellow)
    _add_box(upperworks, (0.20, 0.18, 0.03), (0.02, 0.0, 0.085), dark_steel)

    _add_box(upperworks, (0.30, 0.18, 0.16), (-0.17, 0.0, 0.135), dark_steel)
    _add_box(upperworks, (0.16, 0.12, 0.09), (0.10, 0.14, 0.105), dark_steel)
    _add_box(upperworks, (0.14, 0.10, 0.07), (0.10, 0.14, 0.105), glass)

    _add_strut(upperworks, (-0.02, -0.05, 0.07), (0.03, 0.0, 0.44), 0.012, crane_yellow)
    _add_strut(upperworks, (-0.02, 0.05, 0.07), (0.03, 0.0, 0.44), 0.012, crane_yellow)
    _add_strut(upperworks, (0.06, -0.04, 0.07), (0.03, 0.0, 0.44), 0.010, crane_yellow)
    _add_strut(upperworks, (0.06, 0.04, 0.07), (0.03, 0.0, 0.44), 0.010, crane_yellow)
    _add_box(upperworks, (0.06, 0.04, 0.03), (0.03, 0.0, 0.445), crane_yellow)

    jib_y = 0.06
    jib_z = 0.132
    jib_nodes = [0.12, 0.32, 0.52, 0.72, 0.92, 1.12, 1.32, 1.52]
    for y in (-jib_y, jib_y):
        _add_strut(upperworks, (jib_nodes[0], y, jib_z), (jib_nodes[-1], y, jib_z), 0.0105, crane_yellow)
    for x in jib_nodes:
        _add_strut(upperworks, (x, -jib_y, jib_z), (x, jib_y, jib_z), 0.0050, crane_yellow)
        _add_strut(upperworks, (x, -jib_y, jib_z), (x, 0.0, _jib_top_z(x)), 0.0055, crane_yellow)
        _add_strut(upperworks, (x, jib_y, jib_z), (x, 0.0, _jib_top_z(x)), 0.0055, crane_yellow)
    for x0, x1 in zip(jib_nodes[:-1], jib_nodes[1:]):
        _add_strut(upperworks, (x0, 0.0, _jib_top_z(x0)), (x1, 0.0, _jib_top_z(x1)), 0.0080, crane_yellow)
    for i, (x0, x1) in enumerate(zip(jib_nodes[:-1], jib_nodes[1:])):
        if i % 2 == 0:
            _add_strut(upperworks, (x0, -jib_y, jib_z), (x1, 0.0, _jib_top_z(x1)), 0.0048, crane_yellow)
            _add_strut(upperworks, (x0, jib_y, jib_z), (x1, 0.0, _jib_top_z(x1)), 0.0048, crane_yellow)
        else:
            _add_strut(upperworks, (x1, -jib_y, jib_z), (x0, 0.0, _jib_top_z(x0)), 0.0048, crane_yellow)
            _add_strut(upperworks, (x1, jib_y, jib_z), (x0, 0.0, _jib_top_z(x0)), 0.0048, crane_yellow)

    _add_strut(upperworks, (0.00, -0.08, 0.08), (jib_nodes[0], -jib_y, jib_z), 0.0090, crane_yellow)
    _add_strut(upperworks, (0.00, 0.08, 0.08), (jib_nodes[0], jib_y, jib_z), 0.0090, crane_yellow)
    _add_box(upperworks, (1.08, 0.012, 0.012), (0.88, -jib_y, 0.146), light_steel)
    _add_box(upperworks, (1.08, 0.012, 0.012), (0.88, jib_y, 0.146), light_steel)
    _add_strut(upperworks, (1.52, -jib_y, jib_z), (1.55, 0.0, 0.21), 0.0060, crane_yellow)
    _add_strut(upperworks, (1.52, jib_y, jib_z), (1.55, 0.0, 0.21), 0.0060, crane_yellow)
    upperworks.visual(
        Cylinder(radius=0.020, length=0.06),
        origin=Origin(xyz=(1.55, 0.0, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )

    counter_y = 0.055
    counter_z = 0.132
    counter_nodes = [-0.10, -0.24, -0.38, -0.52, -0.64]
    for y in (-counter_y, counter_y):
        _add_strut(upperworks, (counter_nodes[0], y, counter_z), (counter_nodes[-1], y, counter_z), 0.0100, crane_yellow)
    for x in counter_nodes:
        _add_strut(upperworks, (x, -counter_y, counter_z), (x, counter_y, counter_z), 0.0048, crane_yellow)
        _add_strut(upperworks, (x, -counter_y, counter_z), (x, 0.0, _counter_top_z(x)), 0.0052, crane_yellow)
        _add_strut(upperworks, (x, counter_y, counter_z), (x, 0.0, _counter_top_z(x)), 0.0052, crane_yellow)
    for x0, x1 in zip(counter_nodes[:-1], counter_nodes[1:]):
        _add_strut(
            upperworks,
            (x0, 0.0, _counter_top_z(x0)),
            (x1, 0.0, _counter_top_z(x1)),
            0.0070,
            crane_yellow,
        )
    for i, (x0, x1) in enumerate(zip(counter_nodes[:-1], counter_nodes[1:])):
        if i % 2 == 0:
            _add_strut(upperworks, (x0, -counter_y, counter_z), (x1, 0.0, _counter_top_z(x1)), 0.0046, crane_yellow)
            _add_strut(upperworks, (x0, counter_y, counter_z), (x1, 0.0, _counter_top_z(x1)), 0.0046, crane_yellow)
        else:
            _add_strut(upperworks, (x1, -counter_y, counter_z), (x0, 0.0, _counter_top_z(x0)), 0.0046, crane_yellow)
            _add_strut(upperworks, (x1, counter_y, counter_z), (x0, 0.0, _counter_top_z(x0)), 0.0046, crane_yellow)

    _add_box(upperworks, (0.18, 0.18, 0.025), (-0.56, 0.0, 0.118), dark_steel)
    _add_box(upperworks, (0.12, 0.17, 0.10), (-0.56, 0.0, 0.182), ballast_gray)
    _add_box(upperworks, (0.08, 0.15, 0.09), (-0.47, 0.0, 0.175), ballast_gray)
    upperworks.visual(
        Cylinder(radius=0.048, length=0.14),
        origin=Origin(xyz=(-0.17, 0.0, 0.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_steel,
    )

    _add_strut(upperworks, (0.03, 0.0, 0.44), (1.22, 0.0, _jib_top_z(1.22)), 0.0045, cable_black)
    _add_strut(upperworks, (0.03, 0.0, 0.44), (-0.58, 0.0, _counter_top_z(-0.58)), 0.0045, cable_black)
    _add_strut(upperworks, (0.03, 0.0, 0.44), (0.48, -0.03, _jib_top_z(0.48)), 0.0035, cable_black)
    _add_strut(upperworks, (0.03, 0.0, 0.44), (0.48, 0.03, _jib_top_z(0.48)), 0.0035, cable_black)

    upperworks.inertial = Inertial.from_geometry(
        Box((2.30, 0.40, 0.55)),
        mass=42.0,
        origin=Origin(xyz=(0.42, 0.0, 0.22)),
    )

    trolley = model.part("trolley")
    _add_box(trolley, (0.05, 0.10, 0.012), (0.0, 0.0, 0.006), dark_steel)
    _add_box(trolley, (0.12, 0.14, 0.07), (0.0, 0.0, 0.055), crane_yellow)
    _add_box(trolley, (0.06, 0.10, 0.04), (0.0, 0.0, 0.105), dark_steel)
    _add_box(trolley, (0.10, 0.018, 0.05), (0.0, -0.058, 0.040), dark_steel)
    _add_box(trolley, (0.10, 0.018, 0.05), (0.0, 0.058, 0.040), dark_steel)
    for x in (-0.04, 0.04):
        for y in (-0.068, 0.068):
            trolley.visual(
                Cylinder(radius=0.016, length=0.012),
                origin=Origin(xyz=(x, y, 0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=light_steel,
            )
    trolley.visual(
        Cylinder(radius=0.022, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.098), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )
    _add_box(trolley, (0.03, 0.03, 0.03), (0.0, 0.0, 0.135), dark_steel)
    for x in (-0.016, 0.016):
        _add_strut(trolley, (x, 0.0, 0.000), (x, 0.0, -0.032), 0.0042, dark_steel)
    _add_box(trolley, (0.05, 0.040, 0.014), (0.0, 0.0, -0.025), dark_steel)
    trolley.inertial = Inertial.from_geometry(
        Box((0.14, 0.16, 0.12)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    hook_block = model.part("hook_block")
    for x in (-0.012, 0.012):
        hook_block.visual(
            Cylinder(radius=0.0045, length=0.032),
            origin=Origin(xyz=(x, 0.0, 0.016)),
            material=dark_steel,
        )
    _add_box(hook_block, (0.045, 0.050, 0.012), (0.0, 0.0, -0.004), dark_steel)
    hook_block.visual(
        Cylinder(radius=0.013, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_steel,
    )
    for y in (-0.012, 0.012):
        hook_block.visual(
            Cylinder(radius=0.0035, length=0.54),
            origin=Origin(xyz=(0.0, y, -0.27)),
            material=cable_black,
        )
    _add_box(hook_block, (0.09, 0.07, 0.11), (0.0, 0.0, -0.58), crane_yellow)
    _add_box(hook_block, (0.05, 0.05, 0.08), (0.0, 0.0, -0.67), dark_steel)
    hook_block.visual(
        Cylinder(radius=0.020, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, -0.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_steel,
    )
    hook_block.visual(
        Cylinder(radius=0.017, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.61), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_steel,
    )
    hook_block.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.71)),
        material=dark_steel,
    )

    hook_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.028, 0.0, -0.030),
            (0.038, 0.0, -0.080),
            (0.024, 0.0, -0.135),
            (-0.005, 0.0, -0.165),
            (-0.028, 0.0, -0.140),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    hook_mesh = mesh_from_geometry(hook_geom, ASSETS.mesh_path("tower_crane_hook.obj"))
    hook_block.visual(hook_mesh, origin=Origin(xyz=(0.0, 0.0, -0.74)), material=dark_steel)
    hook_block.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.84)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, -0.42)),
    )

    model.articulation(
        "slew_ring",
        ArticulationType.CONTINUOUS,
        parent="mast",
        child="upperworks",
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.8),
    )
    model.articulation(
        "trolley_on_jib",
        ArticulationType.PRISMATIC,
        parent="upperworks",
        child="trolley",
        origin=Origin(xyz=(0.32, 0.0, 0.152)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.7, lower=-0.08, upper=1.00),
    )
    model.articulation(
        "trolley_to_hook",
        ArticulationType.FIXED,
        parent="trolley",
        child="hook_block",
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("upperworks", "mast", axes="xy", min_overlap=0.16)
    ctx.expect_aabb_gap("upperworks", "mast", axis="z", max_gap=0.004, max_penetration=0.012)
    ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.015)
    ctx.expect_aabb_contact("hook_block", "trolley")
    ctx.expect_joint_motion_axis(
        "trolley_on_jib",
        "trolley",
        world_axis="x",
        direction="positive",
        min_delta=0.35,
    )

    with ctx.pose(trolley_on_jib=-0.06):
        ctx.expect_aabb_overlap("trolley", "upperworks", axes="yz", min_overlap=0.08)
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.015)

    with ctx.pose(trolley_on_jib=0.92):
        ctx.expect_aabb_overlap("trolley", "upperworks", axes="yz", min_overlap=0.08)
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.015)

    with ctx.pose(slew_ring=math.pi / 2.0, trolley_on_jib=0.72):
        ctx.expect_aabb_overlap("upperworks", "mast", axes="xy", min_overlap=0.16)
        ctx.expect_aabb_gap("upperworks", "mast", axis="z", max_gap=0.004, max_penetration=0.012)
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.015)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
