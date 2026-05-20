from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _placed(geometry, *, translate=(0.0, 0.0, 0.0), rotate=(0.0, 0.0, 0.0)):
    placed = geometry.clone()
    rx, ry, rz = rotate
    if rx:
        placed.rotate_x(rx)
    if ry:
        placed.rotate_y(ry)
    if rz:
        placed.rotate_z(rz)
    dx, dy, dz = translate
    if dx or dy or dz:
        placed.translate(dx, dy, dz)
    return placed


def _merge_geometries(*geometries):
    merged = geometries[0].clone()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _housing_profile(z: float, length: float, width: float, nose: float, rear_chamfer: float):
    half_w = width / 2.0
    front = length / 2.0
    rear = -length / 2.0
    return [
        (rear + rear_chamfer, -half_w, z),
        (front - nose, -half_w, z),
        (front, -half_w * 0.55, z),
        (front, half_w * 0.55, z),
        (front - nose, half_w, z),
        (rear + rear_chamfer, half_w, z),
        (rear, half_w - rear_chamfer, z),
        (rear, -half_w + rear_chamfer, z),
    ]


def _build_mount_base_mesh():
    parts = [
        _placed(
            CylinderGeometry(radius=0.240, height=0.040, radial_segments=40),
            translate=(0.0, 0.0, 0.020),
        ),
        _placed(
            CylinderGeometry(radius=0.150, height=0.100, radial_segments=32),
            translate=(0.0, 0.0, 0.090),
        ),
        _placed(BoxGeometry((0.140, 0.100, 0.070)), translate=(-0.085, -0.125, 0.075)),
    ]

    for sign in (-1.0, 1.0):
        parts.append(
            _placed(
                BoxGeometry((0.120, 0.040, 0.110)),
                translate=(sign * 0.102, 0.0, 0.095),
                rotate=(0.0, -sign * 0.280, 0.0),
            )
        )
        parts.append(
            _placed(
                BoxGeometry((0.040, 0.120, 0.110)),
                translate=(0.0, sign * 0.102, 0.095),
                rotate=(sign * 0.280, 0.0, 0.0),
            )
        )

    for index in range(8):
        angle = index * math.tau / 8.0
        parts.append(
            _placed(
                CylinderGeometry(radius=0.014, height=0.012, radial_segments=12),
                translate=(0.190 * math.cos(angle), 0.190 * math.sin(angle), 0.046),
            )
        )

    return _merge_geometries(*parts)


def _build_housing_mesh():
    shell = LoftGeometry(
        [
            _housing_profile(-0.020, 0.320, 0.265, 0.050, 0.030),
            _housing_profile(0.065, 0.360, 0.330, 0.060, 0.040),
            _housing_profile(0.170, 0.315, 0.295, 0.055, 0.035),
            _housing_profile(0.235, 0.170, 0.185, 0.030, 0.025),
        ],
        cap=True,
        closed=True,
    )
    return _merge_geometries(
        shell,
        _placed(
            CylinderGeometry(radius=0.175, height=0.040, radial_segments=40),
            translate=(0.0, 0.0, 0.000),
        ),
        _placed(BoxGeometry((0.090, 0.250, 0.145)), translate=(0.060, 0.0, 0.095)),
        _placed(BoxGeometry((0.145, 0.180, 0.060)), translate=(-0.020, 0.0, 0.242)),
        _placed(BoxGeometry((0.100, 0.120, 0.110)), translate=(0.080, 0.132, 0.176)),
        _placed(BoxGeometry((0.105, 0.100, 0.080)), translate=(-0.125, -0.020, 0.095)),
        _placed(BoxGeometry((0.090, 0.028, 0.110)), translate=(-0.035, 0.176, 0.120)),
        _placed(BoxGeometry((0.090, 0.028, 0.110)), translate=(-0.035, -0.176, 0.120)),
    )


def _build_cradle_mesh():
    return _merge_geometries(
        _placed(BoxGeometry((0.100, 0.205, 0.100)), translate=(0.000, 0.000, 0.020)),
        _placed(BoxGeometry((0.260, 0.150, 0.120)), translate=(0.165, 0.000, 0.040)),
        _placed(BoxGeometry((0.080, 0.190, 0.140)), translate=(0.000, 0.000, 0.080)),
        _placed(BoxGeometry((0.200, 0.030, 0.160)), translate=(0.100, -0.090, 0.060)),
        _placed(BoxGeometry((0.200, 0.030, 0.160)), translate=(0.100, 0.090, 0.060)),
        _placed(BoxGeometry((0.180, 0.100, 0.160)), translate=(0.075, -0.140, 0.080)),
        _placed(BoxGeometry((0.120, 0.120, 0.060)), translate=(0.120, 0.112, 0.130)),
        _placed(BoxGeometry((0.180, 0.060, 0.030)), translate=(0.205, 0.000, 0.130)),
        _placed(BoxGeometry((0.060, 0.180, 0.070)), translate=(-0.028, 0.000, -0.002)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station", assets=ASSETS)

    olive = model.material("olive_drab", rgba=(0.35, 0.38, 0.29, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.38, 0.52, 0.58, 0.35))

    mount_base_mesh = mesh_from_geometry(
        _build_mount_base_mesh(), ASSETS.mesh_path("mount_base.obj")
    )
    housing_mesh = mesh_from_geometry(_build_housing_mesh(), ASSETS.mesh_path("turret_housing.obj"))
    cradle_mesh = mesh_from_geometry(_build_cradle_mesh(), ASSETS.mesh_path("weapon_cradle.obj"))
    feed_chute_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.010, -0.135, 0.150),
                (0.028, -0.114, 0.172),
                (0.102, -0.085, 0.162),
                (0.164, -0.050, 0.118),
            ],
            radius=0.016,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("feed_chute.obj"),
    )

    mount_base = model.part("mount_base")
    mount_base.visual(mount_base_mesh, material=olive)
    mount_base.visual(
        Cylinder(radius=0.190, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=dark_steel,
    )
    mount_base.visual(
        Cylinder(radius=0.125, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=dark_steel,
    )
    mount_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.240, length=0.200),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    turret_housing = model.part("turret_housing")
    turret_housing.visual(housing_mesh, material=olive)
    turret_housing.visual(
        Cylinder(radius=0.035, length=0.240),
        origin=Origin(xyz=(0.095, 0.0, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )
    for side in (-1.0, 1.0):
        for index in range(4):
            turret_housing.visual(
                Cylinder(radius=0.012, length=0.082),
                origin=Origin(
                    xyz=(0.118, side * 0.158, 0.132 + 0.025 * index),
                    rpy=(0.0, math.pi / 2.0 - 0.22, side * 0.18),
                ),
                material=dark_steel,
            )
    turret_housing.visual(
        Box((0.014, 0.060, 0.048)),
        origin=Origin(xyz=(0.127, 0.132, 0.180)),
        material=sensor_glass,
    )
    turret_housing.visual(
        Box((0.048, 0.014, 0.044)),
        origin=Origin(xyz=(0.090, 0.168, 0.180)),
        material=sensor_glass,
    )
    turret_housing.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.126, 0.084, 0.173), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
    )
    turret_housing.inertial = Inertial.from_geometry(
        Box((0.340, 0.340, 0.290)),
        mass=68.0,
        origin=Origin(xyz=(-0.015, 0.0, 0.105)),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(cradle_mesh, material=olive)
    weapon_cradle.visual(
        Cylinder(radius=0.032, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.050, length=0.280),
        origin=Origin(xyz=(0.350, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.036, length=0.330),
        origin=Origin(xyz=(0.520, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_polymer,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.022, length=0.720),
        origin=Origin(xyz=(0.700, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.026, length=0.350),
        origin=Origin(xyz=(0.420, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.030, length=0.100),
        origin=Origin(xyz=(1.090, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    weapon_cradle.visual(
        Cylinder(radius=0.035, length=0.024),
        origin=Origin(xyz=(1.040, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    weapon_cradle.visual(feed_chute_mesh, material=black_polymer)
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((1.150, 0.320, 0.260)),
        mass=56.0,
        origin=Origin(xyz=(0.460, 0.0, 0.055)),
    )

    model.articulation(
        "turret_pan",
        ArticulationType.CONTINUOUS,
        parent="mount_base",
        child="turret_housing",
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=1.0),
    )
    model.articulation(
        "weapon_elevation",
        ArticulationType.REVOLUTE,
        parent="turret_housing",
        child="weapon_cradle",
        origin=Origin(xyz=(0.095, 0.0, 0.102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.2, lower=-0.18, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "mount_base",
        "turret_housing",
        reason="nested slewing ring and armored bearing collar seat tightly",
    )
    ctx.allow_overlap(
        "mount_base",
        "weapon_cradle",
        reason="extreme high-elevation pose can trigger a conservative hull graze between the base collar and the rear cradle block",
    )
    ctx.allow_overlap(
        "turret_housing",
        "weapon_cradle",
        reason="trunnion cradle cheeks intentionally nest into the armored side supports",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("mount_base", "turret_housing")
    ctx.expect_aabb_overlap("mount_base", "turret_housing", axes="xy", min_overlap=0.220)
    ctx.expect_aabb_gap(
        "turret_housing", "mount_base", axis="z", max_gap=0.002, max_penetration=0.060
    )
    ctx.expect_origin_distance("mount_base", "turret_housing", axes="xy", max_dist=0.001)

    ctx.expect_aabb_contact("turret_housing", "weapon_cradle")
    ctx.expect_aabb_overlap("turret_housing", "weapon_cradle", axes="yz", min_overlap=0.100)
    ctx.expect_joint_motion_axis(
        "weapon_elevation",
        "weapon_cradle",
        world_axis="z",
        direction="positive",
        min_delta=0.060,
    )

    with ctx.pose(turret_pan=math.pi / 2.0):
        ctx.expect_aabb_contact("mount_base", "turret_housing")
        ctx.expect_origin_distance("mount_base", "turret_housing", axes="xy", max_dist=0.001)
        ctx.expect_aabb_overlap("mount_base", "turret_housing", axes="xy", min_overlap=0.220)

    with ctx.pose(weapon_elevation=-0.12):
        ctx.expect_aabb_contact("turret_housing", "weapon_cradle")
        ctx.expect_aabb_overlap("turret_housing", "weapon_cradle", axes="yz", min_overlap=0.090)

    with ctx.pose(weapon_elevation=1.0):
        ctx.expect_aabb_contact("turret_housing", "weapon_cradle")
        ctx.expect_aabb_overlap("turret_housing", "weapon_cradle", axes="yz", min_overlap=0.070)

    with ctx.pose({"turret_pan": math.pi / 2.0, "weapon_elevation": 0.90}):
        ctx.expect_aabb_contact("mount_base", "turret_housing")
        ctx.expect_aabb_contact("turret_housing", "weapon_cradle")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
