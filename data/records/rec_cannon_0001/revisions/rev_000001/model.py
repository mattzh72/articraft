from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

WHEEL_CENTER_X = -0.18
WHEEL_CENTER_Y = 0.49
WHEEL_CENTER_Z = -0.10
CHEEK_CENTER_Y = 0.16
CHEEK_THICKNESS = 0.06
TRUNNION_CENTER_Y = 0.16


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_barrel_mesh():
    profile = [
        (0.0, -0.255),
        (0.022, -0.246),
        (0.032, -0.238),
        (0.052, -0.222),
        (0.068, -0.185),
        (0.068, -0.118),
        (0.061, -0.090),
        (0.061, -0.018),
        (0.066, 0.000),
        (0.066, 0.070),
        (0.058, 0.115),
        (0.056, 0.305),
        (0.051, 0.560),
        (0.047, 0.695),
        (0.054, 0.726),
        (0.047, 0.755),
        (0.040, 0.782),
        (0.0, 0.782),
    ]
    barrel_geom = LatheGeometry(profile, segments=64)
    barrel_geom.rotate_y(math.pi / 2.0)
    return _save_mesh("field_cannon_barrel.obj", barrel_geom)


def _build_cheek_mesh():
    cheek_profile = [
        (0.10, -0.22),
        (0.12, 0.03),
        (0.03, 0.06),
        (-0.20, -0.02),
        (-0.56, -0.26),
        (-0.80, -0.46),
        (-0.88, -0.56),
        (-0.76, -0.59),
        (-0.46, -0.33),
        (-0.12, -0.27),
    ]
    cheek_geom = ExtrudeGeometry.centered(
        cheek_profile,
        CHEEK_THICKNESS,
        cap=True,
        closed=True,
    )
    cheek_geom.rotate_x(-math.pi / 2.0)
    return _save_mesh("field_cannon_cheek.obj", cheek_geom)


def _build_wheel_meshes():
    wood_geom = CylinderGeometry(radius=0.10, height=0.12, radial_segments=24)
    wood_geom.rotate_x(math.pi / 2.0)

    felloe = TorusGeometry(
        radius=0.462,
        tube=0.022,
        radial_segments=16,
        tubular_segments=64,
    )
    felloe.rotate_x(math.pi / 2.0)
    wood_geom.merge(felloe)

    spoke_count = 12
    spoke_length = 0.34
    spoke_mid_radius = 0.27
    for spoke_index in range(spoke_count):
        angle = spoke_index * math.tau / spoke_count
        spoke = CylinderGeometry(radius=0.016, height=spoke_length, radial_segments=12)
        spoke.rotate_y(math.pi / 2.0)
        spoke.rotate_y(angle)
        spoke.translate(
            math.cos(angle) * spoke_mid_radius,
            0.0,
            math.sin(angle) * spoke_mid_radius,
        )
        wood_geom.merge(spoke)

    tire_geom = TorusGeometry(
        radius=0.487,
        tube=0.014,
        radial_segments=14,
        tubular_segments=72,
    )
    tire_geom.rotate_x(math.pi / 2.0)

    wood_mesh = _save_mesh("field_cannon_wheel_wood.obj", wood_geom)
    tire_mesh = _save_mesh("field_cannon_wheel_tire.obj", tire_geom)
    return wood_mesh, tire_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="historic_field_cannon", assets=ASSETS)

    oak = model.material("weathered_oak", rgba=(0.47, 0.32, 0.18, 1.0))
    oak_dark = model.material("oiled_oak", rgba=(0.36, 0.24, 0.14, 1.0))
    iron = model.material("forged_iron", rgba=(0.23, 0.24, 0.26, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.19, 0.21, 1.0))
    soot = model.material("bore_black", rgba=(0.08, 0.08, 0.08, 1.0))

    barrel_mesh = _build_barrel_mesh()
    cheek_mesh = _build_cheek_mesh()
    wheel_wood_mesh, wheel_tire_mesh = _build_wheel_meshes()

    carriage = model.part("carriage")
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, 0.0)),
        material=oak,
        name="left_cheek",
    )
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, 0.0)),
        material=oak,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.10, 0.30, 0.10)),
        origin=Origin(xyz=(-0.16, 0.0, -0.12)),
        material=oak,
        name="front_transom",
    )
    carriage.visual(
        Box((0.10, 0.23, 0.08)),
        origin=Origin(xyz=(-0.40, 0.0, -0.22)),
        material=oak_dark,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.26, 0.20, 0.11)),
        origin=Origin(xyz=(-0.25, 0.0, -0.19), rpy=(0.0, -0.18, 0.0)),
        material=oak_dark,
        name="trail_neck_block",
    )
    carriage.visual(
        Box((0.62, 0.14, 0.12)),
        origin=Origin(xyz=(-0.53, 0.0, -0.38), rpy=(0.0, -0.35, 0.0)),
        material=oak_dark,
        name="trail_beam",
    )
    carriage.visual(
        Box((0.24, 0.18, 0.09)),
        origin=Origin(xyz=(-0.54, 0.0, -0.31), rpy=(0.0, -0.28, 0.0)),
        material=oak_dark,
        name="trail_saddle",
    )
    carriage.visual(
        Box((0.11, 0.24, 0.06)),
        origin=Origin(xyz=(-0.87, 0.0, -0.57), rpy=(0.0, -0.18, 0.0)),
        material=iron,
        name="trail_spade",
    )
    carriage.visual(
        Box((0.16, 0.16, 0.05)),
        origin=Origin(xyz=(-0.81, 0.0, -0.52), rpy=(0.0, -0.22, 0.0)),
        material=iron,
        name="trail_spade_socket",
    )
    carriage.visual(
        Cylinder(radius=0.05, length=1.16),
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.0, WHEEL_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    carriage.visual(
        Box((0.12, 0.08, 0.018)),
        origin=Origin(xyz=(0.01, CHEEK_CENTER_Y, 0.062), rpy=(0.18, 0.0, 0.0)),
        material=iron,
        name="left_capsquare",
    )
    carriage.visual(
        Box((0.16, 0.08, 0.11)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, -0.004)),
        material=iron,
        name="left_trunnion_bed",
    )
    carriage.visual(
        Box((0.12, 0.08, 0.018)),
        origin=Origin(xyz=(0.01, -CHEEK_CENTER_Y, 0.062), rpy=(-0.18, 0.0, 0.0)),
        material=iron,
        name="right_capsquare",
    )
    carriage.visual(
        Box((0.16, 0.08, 0.11)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, -0.004)),
        material=iron,
        name="right_trunnion_bed",
    )
    carriage.visual(
        Box((0.18, 0.28, 0.10)),
        origin=Origin(xyz=(-0.07, 0.0, -0.11)),
        material=oak_dark,
        name="bolster_transom",
    )
    carriage.visual(
        Box((0.18, 0.17, 0.08)),
        origin=Origin(xyz=(-0.62, 0.0, -0.35), rpy=(0.0, -0.25, 0.0)),
        material=oak_dark,
        name="rear_trail_block",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.12, 1.18, 0.62)),
        mass=520.0,
        origin=Origin(xyz=(-0.32, 0.0, -0.28)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(wheel_wood_mesh, material=oak_dark, name="wheel_wood")
    left_wheel.visual(wheel_tire_mesh, material=iron, name="wheel_tire")
    left_wheel.visual(
        Cylinder(radius=0.046, length=0.13),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_cap",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=0.12),
        mass=86.0,
        origin=Origin(),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(wheel_wood_mesh, material=oak_dark, name="wheel_wood")
    right_wheel.visual(wheel_tire_mesh, material=iron, name="wheel_tire")
    right_wheel.visual(
        Cylinder(radius=0.046, length=0.13),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_cap",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=0.12),
        mass=86.0,
        origin=Origin(),
    )

    barrel = model.part("barrel")
    barrel.visual(barrel_mesh, material=gunmetal, name="tube")
    barrel.visual(
        Cylinder(radius=0.068, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="reinforce_ring",
    )
    barrel.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, 0.11, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, -0.11, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="right_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.014, length=0.04),
        origin=Origin(xyz=(0.762, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soot,
        name="muzzle_bore",
    )
    barrel.visual(
        Cylinder(radius=0.026, length=0.04),
        origin=Origin(xyz=(-0.230, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="cascabel_neck",
    )
    barrel.visual(
        Cylinder(radius=0.038, length=0.05),
        origin=Origin(xyz=(-0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="cascabel_knob",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=1.00),
        mass=360.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "carriage_to_left_wheel",
        ArticulationType.FIXED,
        parent="carriage",
        child="left_wheel",
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "carriage_to_right_wheel",
        ArticulationType.FIXED,
        parent="carriage",
        child="right_wheel",
        origin=Origin(xyz=(WHEEL_CENTER_X, -WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent="carriage",
        child="barrel",
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.9,
            lower=-0.18,
            upper=0.62,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.13)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("left_wheel", "carriage")
    ctx.expect_aabb_contact("right_wheel", "carriage")
    ctx.expect_aabb_overlap("left_wheel", "carriage", axes="xz", min_overlap=0.10)
    ctx.expect_aabb_overlap("right_wheel", "carriage", axes="xz", min_overlap=0.10)
    ctx.expect_origin_distance("barrel", "carriage", axes="yz", max_dist=0.02)
    ctx.expect_aabb_overlap("barrel", "carriage", axes="yz", min_overlap=0.12)
    ctx.expect_aabb_contact("barrel", "carriage")
    ctx.expect_joint_motion_axis(
        "barrel_elevation",
        "barrel",
        world_axis="z",
        direction="positive",
        min_delta=0.025,
    )

    with ctx.pose(barrel_elevation=-0.18):
        ctx.expect_aabb_contact("barrel", "carriage")
        ctx.expect_aabb_overlap("barrel", "carriage", axes="y", min_overlap=0.10)

    with ctx.pose(barrel_elevation=0.62):
        ctx.expect_aabb_contact("barrel", "carriage")
        ctx.expect_aabb_overlap("barrel", "carriage", axes="y", min_overlap=0.10)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
