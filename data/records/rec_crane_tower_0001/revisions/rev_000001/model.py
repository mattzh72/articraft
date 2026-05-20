from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import inspect
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        params = inspect.signature(Material).parameters
    except (TypeError, ValueError):
        params = {}

    if "rgba" in params:
        return Material(name=name, rgba=rgba)
    if "color" in params:
        return Material(name=name, color=rgba)
    if "diffuse" in params:
        return Material(name=name, diffuse=rgba)
    try:
        return Material(name=name)
    except TypeError:
        return Material(name)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material | None = None,
) -> None:
    origin, length = _segment_origin(start, end)
    if length <= 1e-6:
        return
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material)


def _stations(start: float, end: float, approx_spacing: float) -> list[float]:
    span = end - start
    count = max(1, int(math.ceil(abs(span) / approx_spacing)))
    return [start + span * i / count for i in range(count + 1)]


def _add_mast_lattice(
    part,
    z0: float,
    z1: float,
    width: float,
    bay_height: float,
    steel: Material | None,
) -> None:
    half = width * 0.5
    stations = _stations(z0, z1, bay_height)
    corners = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
    ]

    for x, y in corners:
        _add_tube(part, (x, y, z0), (x, y, z1), 0.014, steel)

    for z in stations:
        ring = [
            (-half, -half, z),
            (half, -half, z),
            (half, half, z),
            (-half, half, z),
        ]
        for a, b in zip(ring, ring[1:] + ring[:1]):
            _add_tube(part, a, b, 0.008, steel)

    for za, zb in zip(stations, stations[1:]):
        _add_tube(part, (-half, -half, za), (half, -half, zb), 0.007, steel)
        _add_tube(part, (half, -half, za), (-half, -half, zb), 0.007, steel)
        _add_tube(part, (-half, half, za), (half, half, zb), 0.007, steel)
        _add_tube(part, (half, half, za), (-half, half, zb), 0.007, steel)
        _add_tube(part, (-half, -half, za), (-half, half, zb), 0.007, steel)
        _add_tube(part, (-half, half, za), (-half, -half, zb), 0.007, steel)
        _add_tube(part, (half, -half, za), (half, half, zb), 0.007, steel)
        _add_tube(part, (half, half, za), (half, -half, zb), 0.007, steel)

    ladder_y = -half - 0.028
    ladder_x = 0.03
    ladder_z0 = z0 + 0.10
    ladder_z1 = z1 - 0.08
    _add_tube(
        part, (-ladder_x, ladder_y, ladder_z0), (-ladder_x, ladder_y, ladder_z1), 0.005, steel
    )
    _add_tube(part, (ladder_x, ladder_y, ladder_z0), (ladder_x, ladder_y, ladder_z1), 0.005, steel)
    for z in _stations(ladder_z0, ladder_z1, 0.18):
        _add_tube(part, (-ladder_x, ladder_y, z), (ladder_x, ladder_y, z), 0.0035, steel)
    for z in _stations(ladder_z0, ladder_z1, 0.36):
        _add_tube(part, (-ladder_x, ladder_y, z), (-ladder_x, -half, z), 0.0035, steel)
        _add_tube(part, (ladder_x, ladder_y, z), (ladder_x, -half, z), 0.0035, steel)


def _add_triangular_boom(
    part,
    x0: float,
    x1: float,
    half_width: float,
    z_bottom: float,
    z_top: float,
    panel: float,
    chord_radius: float,
    brace_radius: float,
    steel: Material | None,
) -> None:
    xs = _stations(x0, x1, panel)
    lower_left = [(x, -half_width, z_bottom) for x in xs]
    lower_right = [(x, half_width, z_bottom) for x in xs]
    upper = [(x, 0.0, z_top) for x in xs]

    for line in (lower_left, lower_right, upper):
        for start, end in zip(line, line[1:]):
            _add_tube(part, start, end, chord_radius, steel)

    for ll, lr, up in zip(lower_left, lower_right, upper):
        _add_tube(part, ll, lr, brace_radius, steel)
        _add_tube(part, ll, up, brace_radius, steel)
        _add_tube(part, lr, up, brace_radius, steel)

    for i in range(len(xs) - 1):
        if i % 2 == 0:
            _add_tube(part, lower_left[i], upper[i + 1], brace_radius, steel)
            _add_tube(part, lower_right[i], upper[i + 1], brace_radius, steel)
            _add_tube(part, lower_left[i], lower_right[i + 1], brace_radius * 0.9, steel)
        else:
            _add_tube(part, upper[i], lower_left[i + 1], brace_radius, steel)
            _add_tube(part, upper[i], lower_right[i + 1], brace_radius, steel)
            _add_tube(part, lower_right[i], lower_left[i + 1], brace_radius * 0.9, steel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane", assets=ASSETS)

    materials = {
        "concrete": _make_material("concrete", (0.69, 0.69, 0.69, 1.0)),
        "crane_yellow": _make_material("crane_yellow", (0.92, 0.75, 0.15, 1.0)),
        "steel_dark": _make_material("steel_dark", (0.23, 0.23, 0.25, 1.0)),
        "ballast": _make_material("ballast", (0.14, 0.14, 0.16, 1.0)),
        "cable": _make_material("cable", (0.08, 0.08, 0.08, 1.0)),
        "glass": _make_material("glass", (0.42, 0.58, 0.72, 0.35)),
    }
    if hasattr(model, "materials"):
        model.materials.extend(materials.values())

    tower = model.part("tower")
    tower.visual(
        Box((1.0, 1.0, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=materials["concrete"],
    )
    tower.visual(
        Box((0.42, 0.42, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=materials["steel_dark"],
    )
    tower.visual(
        Box((0.32, 0.32, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        material=materials["steel_dark"],
    )
    _add_mast_lattice(
        tower,
        z0=0.32,
        z1=2.18,
        width=0.28,
        bay_height=0.24,
        steel=materials["crane_yellow"],
    )
    tower.inertial = Inertial.from_geometry(
        Box((1.0, 1.0, 2.24)),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
    )

    upper = model.part("upperworks")
    upper.visual(
        Cylinder(radius=0.19, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=materials["steel_dark"],
    )
    upper.visual(
        Box((0.46, 0.34, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.10)),
        material=materials["steel_dark"],
    )
    upper.visual(
        Box((0.72, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=materials["crane_yellow"],
    )

    _add_triangular_boom(
        upper,
        x0=0.16,
        x1=2.55,
        half_width=0.06,
        z_bottom=0.18,
        z_top=0.34,
        panel=0.22,
        chord_radius=0.012,
        brace_radius=0.008,
        steel=materials["crane_yellow"],
    )
    _add_triangular_boom(
        upper,
        x0=-0.18,
        x1=-0.98,
        half_width=0.05,
        z_bottom=0.18,
        z_top=0.28,
        panel=0.18,
        chord_radius=0.011,
        brace_radius=0.007,
        steel=materials["crane_yellow"],
    )

    a_frame_top = (0.0, 0.0, 0.52)
    for foot in (
        (0.12, -0.06, 0.15),
        (0.12, 0.06, 0.15),
        (-0.16, -0.06, 0.15),
        (-0.16, 0.06, 0.15),
    ):
        _add_tube(upper, foot, a_frame_top, 0.010, materials["crane_yellow"])
    _add_tube(upper, (2.42, -0.05, 0.19), a_frame_top, 0.007, materials["steel_dark"])
    _add_tube(upper, (2.42, 0.05, 0.19), a_frame_top, 0.007, materials["steel_dark"])
    _add_tube(upper, (-0.92, -0.04, 0.18), a_frame_top, 0.007, materials["steel_dark"])
    _add_tube(upper, (-0.92, 0.04, 0.18), a_frame_top, 0.007, materials["steel_dark"])

    upper.visual(
        Box((0.10, 0.12, 0.08)),
        origin=Origin(xyz=(2.58, 0.0, 0.22)),
        material=materials["steel_dark"],
    )
    upper.visual(
        Box((0.22, 0.14, 0.12)),
        origin=Origin(xyz=(0.22, 0.17, 0.12)),
        material=materials["steel_dark"],
    )
    upper.visual(
        Box((0.15, 0.004, 0.08)),
        origin=Origin(xyz=(0.275, 0.239, 0.13)),
        material=materials["glass"],
    )
    upper.visual(
        Box((0.004, 0.10, 0.08)),
        origin=Origin(xyz=(0.112, 0.17, 0.13)),
        material=materials["glass"],
    )
    upper.visual(
        Box((0.24, 0.18, 0.12)),
        origin=Origin(xyz=(-0.36, 0.0, 0.12)),
        material=materials["steel_dark"],
    )
    upper.visual(
        Cylinder(radius=0.06, length=0.16),
        origin=Origin(xyz=(-0.50, 0.0, 0.17), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=materials["steel_dark"],
    )
    upper.visual(
        Box((0.28, 0.20, 0.18)),
        origin=Origin(xyz=(-0.74, 0.0, 0.09)),
        material=materials["ballast"],
    )
    upper.visual(
        Box((0.16, 0.18, 0.16)),
        origin=Origin(xyz=(-0.53, 0.0, 0.08)),
        material=materials["ballast"],
    )

    upper.inertial = Inertial.from_geometry(
        Box((3.65, 0.50, 0.70)),
        mass=1850.0,
        origin=Origin(xyz=(0.78, 0.0, 0.20)),
    )

    trolley = model.part("trolley_hook")
    trolley.visual(
        Box((0.14, 0.11, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=materials["steel_dark"],
    )
    trolley.visual(
        Box((0.12, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.055, 0.010)),
        material=materials["steel_dark"],
    )
    trolley.visual(
        Box((0.12, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.055, 0.010)),
        material=materials["steel_dark"],
    )
    trolley.visual(
        Box((0.09, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=materials["steel_dark"],
    )
    trolley.visual(
        Box((0.07, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=materials["steel_dark"],
    )
    for y in (-0.048, 0.048):
        trolley.visual(
            Cylinder(radius=0.018, length=0.028),
            origin=Origin(xyz=(-0.045, 1.5 * y, 0.028), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=materials["steel_dark"],
        )
        trolley.visual(
            Cylinder(radius=0.018, length=0.028),
            origin=Origin(xyz=(0.045, 1.5 * y, 0.028), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=materials["steel_dark"],
        )
    trolley.visual(
        Cylinder(radius=0.006, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
        material=materials["cable"],
    )
    trolley.visual(
        Box((0.10, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.97)),
        material=materials["crane_yellow"],
    )
    trolley.visual(
        Cylinder(radius=0.018, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.90), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=materials["steel_dark"],
    )
    hook_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -1.04),
                (0.022, 0.0, -1.08),
                (0.045, 0.0, -1.15),
                (0.040, 0.0, -1.21),
                (0.010, 0.0, -1.248),
                (-0.020, 0.0, -1.222),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("trolley_hook_curve.obj"),
    )
    trolley.visual(hook_mesh, material=materials["crane_yellow"])
    trolley.visual(
        Box((0.010, 0.028, 0.055)),
        origin=Origin(xyz=(0.028, 0.0, -1.16)),
        material=materials["cable"],
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 1.30)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, -0.62)),
    )

    model.articulation(
        "slew_rotation",
        ArticulationType.CONTINUOUS,
        parent="tower",
        child="upperworks",
        origin=Origin(xyz=(0.0, 0.0, 2.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=85000.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "trolley_slide",
        ArticulationType.PRISMATIC,
        parent="upperworks",
        child="trolley_hook",
        origin=Origin(xyz=(0.34, 0.0, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=1.2,
            lower=0.0,
            upper=1.92,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap("tower", "upperworks", reason="slewing bearing and mast cap seat flush")
    ctx.allow_overlap(
        "upperworks", "trolley_hook", reason="trolley wheels ride tightly on jib rails"
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("upperworks", "tower", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_gap("upperworks", "tower", axis="z", max_gap=0.004, max_penetration=0.0)
    ctx.expect_origin_distance("upperworks", "tower", axes="xy", max_dist=0.95)
    ctx.expect_joint_motion_axis(
        "trolley_slide",
        "trolley_hook",
        world_axis="x",
        direction="positive",
        min_delta=0.30,
    )

    with ctx.pose(trolley_slide=0.0):
        ctx.expect_aabb_overlap("trolley_hook", "upperworks", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("trolley_hook", "tower", axes="xy", max_dist=0.70)

    with ctx.pose(trolley_slide=1.92):
        ctx.expect_aabb_overlap("trolley_hook", "upperworks", axes="xy", min_overlap=0.08)

    with ctx.pose(slew_rotation=math.pi * 0.5):
        ctx.expect_aabb_overlap("upperworks", "tower", axes="xy", min_overlap=0.20)
        ctx.expect_aabb_gap("upperworks", "tower", axis="z", max_gap=0.004, max_penetration=0.0)
        ctx.expect_origin_distance("upperworks", "tower", axes="xy", max_dist=0.95)
        ctx.expect_joint_motion_axis(
            "trolley_slide",
            "trolley_hook",
            world_axis="y",
            direction="positive",
            min_delta=0.30,
        )

    with ctx.pose({"slew_rotation": math.pi * 0.5, "trolley_slide": 1.92}):
        ctx.expect_aabb_overlap("trolley_hook", "upperworks", axes="xy", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
