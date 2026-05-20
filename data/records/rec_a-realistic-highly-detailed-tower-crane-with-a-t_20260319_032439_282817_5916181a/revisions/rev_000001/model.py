from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import acos, atan2, pi, sqrt

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    radial_segments: int = 10,
) -> MeshGeometry:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-8:
        return MeshGeometry()

    geom = CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)
    yaw = atan2(dy, dx)
    pitch = acos(max(-1.0, min(1.0, dz / length)))
    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    geom.rotate_y(pitch)
    geom.rotate_z(yaw)
    geom.translate(*mid)
    return geom


def _tower_lattice_mesh(
    height: float,
    width: float,
    panels: int,
    chord_radius: float,
    brace_radius: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    half = width * 0.5
    dz = height / panels
    corners = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
    ]
    face_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]

    for i in range(panels + 1):
        z = i * dz
        for a, b in face_edges:
            geom.merge(
                _cylinder_between(
                    (corners[a][0], corners[a][1], z),
                    (corners[b][0], corners[b][1], z),
                    radius=brace_radius,
                )
            )
        if 0 < i < panels:
            geom.merge(
                _cylinder_between(
                    (corners[0][0], corners[0][1], z),
                    (corners[2][0], corners[2][1], z),
                    radius=brace_radius * 0.8,
                )
            )
            geom.merge(
                _cylinder_between(
                    (corners[1][0], corners[1][1], z),
                    (corners[3][0], corners[3][1], z),
                    radius=brace_radius * 0.8,
                )
            )

    for x, y in corners:
        for i in range(panels):
            z0 = i * dz
            z1 = (i + 1) * dz
            geom.merge(_cylinder_between((x, y, z0), (x, y, z1), radius=chord_radius))

    for i in range(panels):
        z0 = i * dz
        z1 = (i + 1) * dz
        for a, b in face_edges:
            geom.merge(
                _cylinder_between(
                    (corners[a][0], corners[a][1], z0),
                    (corners[b][0], corners[b][1], z1),
                    radius=brace_radius,
                )
            )
            geom.merge(
                _cylinder_between(
                    (corners[b][0], corners[b][1], z0),
                    (corners[a][0], corners[a][1], z1),
                    radius=brace_radius,
                )
            )

    return geom


def _triangular_boom_mesh(
    x0: float,
    x1: float,
    width: float,
    height: float,
    panels: int,
    base_z: float,
    chord_radius: float,
    brace_radius: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    span = x1 - x0
    dx = span / panels

    def station(ix: int) -> list[tuple[float, float, float]]:
        x = x0 + ix * dx
        return [
            (x, -0.5 * width, base_z),
            (x, 0.5 * width, base_z),
            (x, 0.0, base_z + height),
        ]

    stations = [station(i) for i in range(panels + 1)]

    for s in stations:
        geom.merge(_cylinder_between(s[0], s[1], radius=brace_radius))
        geom.merge(_cylinder_between(s[0], s[2], radius=brace_radius))
        geom.merge(_cylinder_between(s[1], s[2], radius=brace_radius))

    for i in range(panels):
        a = stations[i]
        b = stations[i + 1]
        for j in range(3):
            geom.merge(_cylinder_between(a[j], b[j], radius=chord_radius))

        geom.merge(_cylinder_between(a[0], b[2], radius=brace_radius))
        geom.merge(_cylinder_between(a[2], b[0], radius=brace_radius))
        geom.merge(_cylinder_between(a[1], b[2], radius=brace_radius))
        geom.merge(_cylinder_between(a[2], b[1], radius=brace_radius))
        geom.merge(_cylinder_between(a[0], b[1], radius=brace_radius * 0.9))
        geom.merge(_cylinder_between(a[1], b[0], radius=brace_radius * 0.9))

    return geom


def _tower_head_and_stays_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    apex_left = (0.0, -0.05, 1.62)
    apex_right = (0.0, 0.05, 1.62)
    bases = [
        (0.18, -0.20, 0.18),
        (0.18, 0.20, 0.18),
        (-0.10, -0.14, 0.18),
        (-0.10, 0.14, 0.18),
    ]

    for base in bases[:2]:
        geom.merge(_cylinder_between(base, apex_left, radius=0.026))
        geom.merge(_cylinder_between(base, apex_right, radius=0.026))
    for base in bases[2:]:
        geom.merge(_cylinder_between(base, apex_left, radius=0.022))
        geom.merge(_cylinder_between(base, apex_right, radius=0.022))

    geom.merge(_cylinder_between(bases[0], bases[1], radius=0.020))
    geom.merge(_cylinder_between(bases[2], bases[3], radius=0.018))
    geom.merge(_cylinder_between(bases[0], bases[2], radius=0.018))
    geom.merge(_cylinder_between(bases[1], bases[3], radius=0.018))
    geom.merge(_cylinder_between(apex_left, apex_right, radius=0.020))

    jib_top = (4.32, 0.0, 1.04)
    counter_top = (-1.85, 0.0, 0.90)
    geom.merge(_cylinder_between(apex_left, jib_top, radius=0.016))
    geom.merge(_cylinder_between(apex_right, jib_top, radius=0.016))
    geom.merge(_cylinder_between(apex_left, counter_top, radius=0.016))
    geom.merge(_cylinder_between(apex_right, counter_top, radius=0.016))

    return geom


def _hook_curve_mesh() -> MeshGeometry:
    return tube_from_spline_points(
        [
            (0.0, 0.0, -2.48),
            (0.10, 0.0, -2.55),
            (0.14, 0.0, -2.70),
            (0.05, 0.0, -2.84),
            (-0.10, 0.0, -2.73),
        ],
        radius=0.022,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.64, 0.64, 0.61, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.74, 0.16, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.57, 0.59, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    rope_black = model.material("rope_black", rgba=(0.12, 0.12, 0.13, 1.0))
    cab_glass = model.material("cab_glass", rgba=(0.62, 0.77, 0.85, 0.42))

    mast_mesh = mesh_from_geometry(
        _tower_lattice_mesh(
            height=6.18,
            width=0.72,
            panels=10,
            chord_radius=0.040,
            brace_radius=0.018,
        ),
        MESH_DIR / "tower_mast.obj",
    )
    jib_mesh = mesh_from_geometry(
        _triangular_boom_mesh(
            x0=0.40,
            x1=6.00,
            width=0.56,
            height=0.72,
            panels=10,
            base_z=0.32,
            chord_radius=0.030,
            brace_radius=0.015,
        ),
        MESH_DIR / "jib_truss.obj",
    )
    counter_jib_mesh = mesh_from_geometry(
        _triangular_boom_mesh(
            x0=-2.25,
            x1=-0.25,
            width=0.48,
            height=0.58,
            panels=5,
            base_z=0.32,
            chord_radius=0.028,
            brace_radius=0.014,
        ),
        MESH_DIR / "counter_jib_truss.obj",
    )
    tower_head_mesh = mesh_from_geometry(
        _tower_head_and_stays_mesh(),
        MESH_DIR / "tower_head_and_stays.obj",
    )
    slew_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.44, tube=0.06, radial_segments=18, tubular_segments=36),
        MESH_DIR / "slew_ring.obj",
    )
    hook_mesh = mesh_from_geometry(_hook_curve_mesh(), MESH_DIR / "hook_curve.obj")

    base = model.part("base")
    base.visual(Box((2.80, 2.80, 0.50)), origin=Origin(xyz=(0.0, 0.0, 0.25)), material=concrete)
    base.visual(Box((1.00, 1.00, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.59)), material=painted_steel)
    base.visual(mast_mesh, origin=Origin(xyz=(0.0, 0.0, 0.68)), material=safety_yellow)
    base.visual(Box((0.88, 0.88, 0.06)), origin=Origin(xyz=(0.0, 0.0, 6.89)), material=painted_steel)
    base.inertial = Inertial.from_geometry(
        Box((2.80, 2.80, 6.92)),
        mass=6800.0,
        origin=Origin(xyz=(0.0, 0.0, 3.46)),
    )

    upperworks = model.part("upperworks")
    upperworks.visual(slew_ring_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark_steel)
    upperworks.visual(Cylinder(radius=0.18, length=0.14), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=painted_steel)
    upperworks.visual(Box((1.30, 1.15, 0.28)), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=painted_steel)
    upperworks.visual(Box((0.72, 0.58, 0.46)), origin=Origin(xyz=(-0.30, -0.18, 0.41)), material=painted_steel)
    upperworks.visual(Box((0.62, 0.36, 0.34)), origin=Origin(xyz=(0.58, 0.74, 0.23)), material=painted_steel)
    upperworks.visual(Box((0.40, 0.26, 0.24)), origin=Origin(xyz=(0.70, 0.78, 0.24)), material=cab_glass)
    upperworks.visual(jib_mesh, material=safety_yellow)
    upperworks.visual(counter_jib_mesh, material=safety_yellow)
    upperworks.visual(tower_head_mesh, material=safety_yellow)
    upperworks.inertial = Inertial.from_geometry(
        Box((8.25, 1.60, 1.95)),
        mass=1680.0,
        origin=Origin(xyz=(1.88, 0.0, 0.68)),
    )

    counterweight = model.part("counterweight")
    counterweight.visual(Box((0.20, 0.62, 0.10)), origin=Origin(xyz=(0.10, 0.0, 0.05)), material=painted_steel)
    counterweight.visual(Box((0.88, 1.00, 0.20)), origin=Origin(xyz=(-0.24, 0.0, 0.10)), material=dark_steel)
    counterweight.visual(Box((0.72, 0.92, 0.18)), origin=Origin(xyz=(-0.28, 0.0, 0.27)), material=concrete)
    counterweight.visual(Box((0.64, 0.84, 0.18)), origin=Origin(xyz=(-0.30, 0.0, 0.44)), material=concrete)
    counterweight.inertial = Inertial.from_geometry(
        Box((0.88, 1.00, 0.56)),
        mass=1850.0,
        origin=Origin(xyz=(-0.24, 0.0, 0.28)),
    )

    trolley = model.part("trolley")
    trolley.visual(Box((0.52, 0.36, 0.10)), origin=Origin(xyz=(0.0, 0.0, -0.03)), material=painted_steel)
    trolley.visual(Box((0.24, 0.20, 0.20)), origin=Origin(xyz=(0.0, 0.0, -0.15)), material=dark_steel)
    trolley.visual(Box((0.12, 0.12, 0.20)), origin=Origin(xyz=(0.0, 0.0, -0.16)), material=painted_steel)
    for x in (-0.16, 0.16):
        for y in (-0.12, 0.12):
            trolley.visual(
                Cylinder(radius=0.045, length=0.06),
                origin=Origin(xyz=(x, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
            )
    trolley.inertial = Inertial.from_geometry(
        Box((0.52, 0.36, 0.34)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(Box((0.18, 0.18, 0.08)), origin=Origin(xyz=(0.0, 0.0, -0.04)), material=painted_steel)
    hook_block.visual(
        Cylinder(radius=0.010, length=1.88),
        origin=Origin(xyz=(0.0, -0.055, -0.94)),
        material=rope_black,
    )
    hook_block.visual(
        Cylinder(radius=0.010, length=1.88),
        origin=Origin(xyz=(0.0, 0.055, -0.94)),
        material=rope_black,
    )
    hook_block.visual(Box((0.10, 0.14, 0.44)), origin=Origin(xyz=(0.0, 0.0, -2.08)), material=painted_steel)
    hook_block.visual(Box((0.18, 0.02, 0.42)), origin=Origin(xyz=(0.0, -0.08, -2.08)), material=painted_steel)
    hook_block.visual(Box((0.18, 0.02, 0.42)), origin=Origin(xyz=(0.0, 0.08, -2.08)), material=painted_steel)
    hook_block.visual(Cylinder(radius=0.032, length=0.18), origin=Origin(xyz=(0.0, 0.0, -2.39)), material=dark_steel)
    hook_block.visual(hook_mesh, material=dark_steel)
    hook_block.inertial = Inertial.from_geometry(
        Box((0.28, 0.20, 2.80)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -1.40)),
    )

    model.articulation(
        "slew_rotation",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="upperworks",
        origin=Origin(xyz=(0.0, 0.0, 6.89)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35000.0, velocity=0.45),
    )
    model.articulation(
        "jib_trolley",
        ArticulationType.PRISMATIC,
        parent="upperworks",
        child="trolley",
        origin=Origin(xyz=(0.96, 0.0, 0.32)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.90, lower=0.0, upper=3.96),
    )
    model.articulation(
        "upperworks_to_counterweight",
        ArticulationType.FIXED,
        parent="upperworks",
        child="counterweight",
        origin=Origin(xyz=(-1.52, 0.0, 0.32)),
    )
    model.articulation(
        "trolley_to_hook",
        ArticulationType.FIXED,
        parent="trolley",
        child="hook_block",
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "upperworks",
        "hook_block",
        reason="conservative collision hulls around the lattice jib can brush the suspended rope zone near the trolley",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.01,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("upperworks", "base")
    ctx.expect_origin_distance("upperworks", "base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("upperworks", "base", axes="xy", min_overlap=0.70)
    ctx.expect_aabb_contact("counterweight", "upperworks")
    ctx.expect_aabb_contact("hook_block", "trolley")
    ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("hook_block", "trolley", axes="xy", min_overlap=0.10)
    ctx.expect_joint_motion_axis("jib_trolley", "trolley", world_axis="x", direction="positive", min_delta=1.50)

    upper_home = ctx.part_world_position("upperworks")
    counterweight_home = ctx.part_world_position("counterweight")
    trolley_home = ctx.part_world_position("trolley")
    hook_home = ctx.part_world_position("hook_block")
    if upper_home[2] <= 6.80:
        raise AssertionError("Upperworks should sit high above grade on a tall tower mast.")
    if counterweight_home[0] >= upper_home[0] - 1.40 or abs(counterweight_home[1] - upper_home[1]) > 0.02:
        raise AssertionError("Counterweight ballast should sit clearly aft of the slewing center on the counter-jib.")
    if trolley_home[0] <= 0.90 or abs(trolley_home[1]) > 0.02:
        raise AssertionError("Trolley home pose should sit near the inboard jib root on the crane centerline.")
    if trolley_home[0] <= upper_home[0] + 0.90:
        raise AssertionError("The main jib should project forward from the mast before the trolley begins its travel.")
    hook_bottom_home_z = hook_home[2] - 2.84
    if hook_bottom_home_z >= trolley_home[2] - 2.40:
        raise AssertionError("Hook block should hang well below the trolley and jib to read as a suspended lifting assembly.")
    if abs(hook_home[0] - trolley_home[0]) > 0.01 or abs(hook_home[1] - trolley_home[1]) > 0.01:
        raise AssertionError("Hook block should hang directly below the trolley in the rest pose.")

    with ctx.pose(jib_trolley=3.96):
        ctx.expect_aabb_contact("hook_block", "trolley")
        ctx.expect_origin_distance("hook_block", "trolley", axes="xy", max_dist=0.01)
        trolley_out = ctx.part_world_position("trolley")
        hook_out = ctx.part_world_position("hook_block")
        if trolley_out[0] <= trolley_home[0] + 3.50:
            raise AssertionError("Trolley should travel far outboard along the jib.")
        if abs(hook_out[0] - trolley_out[0]) > 0.01 or abs(hook_out[1] - trolley_out[1]) > 0.01:
            raise AssertionError("Hook block should stay centered under the trolley at full reach.")

    with ctx.pose(slew_rotation=pi / 2.0):
        ctx.expect_aabb_contact("upperworks", "base")
        ctx.expect_origin_distance("upperworks", "base", axes="xy", max_dist=0.01)
        counterweight_rot = ctx.part_world_position("counterweight")
        if counterweight_rot[1] >= -1.30 or abs(counterweight_rot[0]) > 0.25:
            raise AssertionError("After a quarter-turn slew, the counterweight should swing to the rear lateral side opposite the jib.")

    with ctx.pose(slew_rotation=pi / 2.0, jib_trolley=3.96):
        trolley_rot = ctx.part_world_position("trolley")
        hook_rot = ctx.part_world_position("hook_block")
        if trolley_rot[1] <= 4.20 or abs(trolley_rot[0]) > 0.25:
            raise AssertionError("Slewing the crane should swing the extended trolley into a strong lateral world-space reach.")
        if counterweight_rot[1] >= trolley_rot[1] - 5.20:
            raise AssertionError("Counterweight should remain clearly opposite the working jib after slewing.")
        if abs(hook_rot[0] - trolley_rot[0]) > 0.01 or abs(hook_rot[1] - trolley_rot[1]) > 0.01:
            raise AssertionError("Hook block should remain directly under the trolley after slewing.")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
