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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    Material,
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
REAR_WHEEL_RADIUS = 0.085
REAR_WHEEL_WIDTH = 0.024
CASTER_WHEEL_RADIUS = 0.055
CASTER_WHEEL_WIDTH = 0.022


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _merge_geometries(*geometries):
    merged = geometries[0].clone()
    for geom in geometries[1:]:
        merged.merge(geom)
    return merged


def _tube(points, radius: float, samples: int = 14, radial_segments: int = 16):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _build_wheel_geometries(
    radius: float,
    width: float,
    rim_radius: float,
    rim_tube: float,
    tire_tube: float,
    hub_radius: float,
    spoke_radius: float,
    spoke_count: int,
):
    tire = TorusGeometry(
        radius=radius - tire_tube,
        tube=tire_tube,
        radial_segments=18,
        tubular_segments=52,
    ).rotate_y(math.pi / 2.0)

    rim = TorusGeometry(
        radius=rim_radius,
        tube=rim_tube,
        radial_segments=16,
        tubular_segments=44,
    ).rotate_y(math.pi / 2.0)
    hub = CylinderGeometry(radius=hub_radius, height=width * 1.15, radial_segments=24).rotate_y(
        math.pi / 2.0
    )

    spoke_start = hub_radius + 0.006
    spoke_end = rim_radius - rim_tube * 1.3
    spoke_length = max(spoke_end - spoke_start, 0.012)
    spoke_center = spoke_start + 0.5 * spoke_length
    spokes = []
    for index in range(spoke_count):
        angle = (2.0 * math.pi * index) / spoke_count
        spoke = CylinderGeometry(
            radius=spoke_radius,
            height=spoke_length,
            radial_segments=10,
        )
        spoke.rotate_x(-angle)
        spoke.translate(
            0.0,
            spoke_center * math.sin(angle),
            spoke_center * math.cos(angle),
        )
        spokes.append(spoke)

    return tire, _merge_geometries(rim, hub, *spokes)


def _build_chassis_geometry():
    side_radius = 0.012
    arm_radius = 0.010
    brace_radius = 0.008
    geoms = []

    for side in (-1.0, 1.0):
        x_side = side * 0.215
        geoms.append(
            _tube(
                [
                    (x_side, -0.11, 0.10),
                    (x_side, -0.11, 0.24),
                    (x_side, -0.07, 0.45),
                    (x_side, 0.16, 0.45),
                    (x_side, 0.30, 0.17),
                ],
                radius=side_radius,
                samples=18,
            )
        )
        geoms.append(
            _tube(
                [
                    (x_side, -0.02, 0.45),
                    (x_side, -0.02, 0.63),
                    (x_side, 0.15, 0.63),
                    (x_side, 0.15, 0.46),
                ],
                radius=arm_radius,
                samples=16,
            )
        )
        geoms.append(
            CylinderGeometry(radius=0.011, height=0.022, radial_segments=20).translate(
                side * 0.236, 0.320, 0.182
            )
        )
        geoms.append(
            _tube(
                [
                    (x_side, 0.300, 0.170),
                    (side * 0.228, 0.310, 0.176),
                    (side * 0.236, 0.320, 0.182),
                ],
                radius=0.007,
                samples=12,
            )
        )
        geoms.append(
            CylinderGeometry(radius=0.009, height=0.03, radial_segments=20)
            .rotate_y(math.pi / 2.0)
            .translate(side * 0.228, -0.11, REAR_WHEEL_RADIUS)
        )
        geoms.append(
            _tube(
                [
                    (side * 0.095, 0.18, 0.44),
                    (side * 0.095, 0.235, 0.355),
                    (side * 0.095, 0.285, 0.285),
                ],
                radius=0.007,
                samples=14,
            )
        )

    geoms.extend(
        [
            _tube([(-0.16, -0.135, 0.468), (0.16, -0.135, 0.468)], radius=0.007),
            _tube([(-0.18, -0.11, 0.10), (0.18, -0.11, 0.10)], radius=0.010),
            _tube([(-0.18, -0.05, 0.45), (0.18, -0.05, 0.45)], radius=0.010),
            _tube([(-0.18, 0.14, 0.45), (0.18, 0.14, 0.45)], radius=0.010),
            _tube([(-0.18, 0.26, 0.31), (0.18, 0.26, 0.31)], radius=0.010),
            _tube([(-0.20, 0.30, 0.17), (0.20, 0.30, 0.17)], radius=0.010),
            _tube([(-0.17, -0.02, 0.43), (0.17, 0.10, 0.26)], radius=brace_radius),
            _tube([(0.17, -0.02, 0.43), (-0.17, 0.10, 0.26)], radius=brace_radius),
        ]
    )
    return _merge_geometries(*geoms)


def _build_backrest_geometry():
    geoms = [
        _tube([(-0.165, 0.0, 0.0), (0.165, 0.0, 0.0)], radius=0.010),
        _tube(
            [
                (-0.17, 0.0, 0.0),
                (-0.17, -0.01, 0.16),
                (-0.17, -0.02, 0.35),
                (-0.17, -0.06, 0.44),
            ],
            radius=0.010,
            samples=16,
        ),
        _tube(
            [
                (0.17, 0.0, 0.0),
                (0.17, -0.01, 0.16),
                (0.17, -0.02, 0.35),
                (0.17, -0.06, 0.44),
            ],
            radius=0.010,
            samples=16,
        ),
        _tube([(-0.16, -0.02, 0.22), (0.16, -0.02, 0.22)], radius=0.009),
        _tube([(-0.17, -0.06, 0.43), (0.17, -0.06, 0.43)], radius=0.009),
    ]
    return _merge_geometries(*geoms)


def _build_caster_yoke_geometry():
    return _merge_geometries(
        CylinderGeometry(radius=0.009, height=0.024, radial_segments=18).translate(
            0.0, 0.0, -0.012
        ),
        BoxGeometry((0.046, 0.022, 0.008)).translate(0.0, 0.0, -0.038),
        BoxGeometry((0.008, 0.019, 0.082)).translate(-0.017, 0.0, -0.082),
        BoxGeometry((0.008, 0.019, 0.082)).translate(0.017, 0.0, -0.082),
        BoxGeometry((0.044, 0.022, 0.008)).translate(0.0, 0.0, -0.125),
    )


def _build_footrest_geometry():
    return _merge_geometries(
        CylinderGeometry(radius=0.008, height=0.026, radial_segments=16)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.010, -0.008),
        _tube(
            [
                (0.0, 0.012, -0.010),
                (0.0, 0.055, -0.095),
                (0.0, 0.122, -0.228),
            ],
            radius=0.0075,
            samples=18,
        ),
        BoxGeometry((0.112, 0.088, 0.010)).translate(0.0, 0.138, -0.232),
        BoxGeometry((0.112, 0.014, 0.026)).translate(0.0, 0.176, -0.220),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transport_wheelchair", assets=ASSETS)

    materials = {
        "frame": _material("powder_coat_silver", (0.74, 0.76, 0.79, 1.0)),
        "upholstery": _material("dark_navy_upholstery", (0.12, 0.17, 0.26, 1.0)),
        "rubber": _material("black_rubber", (0.07, 0.07, 0.08, 1.0)),
        "plastic": _material("dark_plastic", (0.14, 0.14, 0.15, 1.0)),
        "alloy": _material("brushed_alloy", (0.64, 0.66, 0.70, 1.0)),
    }
    model.materials.extend(materials.values())

    chassis_mesh = mesh_from_geometry(
        _build_chassis_geometry(), ASSETS.mesh_path("wheelchair_chassis.obj")
    )
    backrest_mesh = mesh_from_geometry(
        _build_backrest_geometry(), ASSETS.mesh_path("wheelchair_backrest.obj")
    )
    caster_yoke_mesh = mesh_from_geometry(
        _build_caster_yoke_geometry(), ASSETS.mesh_path("wheelchair_caster_yoke.obj")
    )
    footrest_mesh = mesh_from_geometry(
        _build_footrest_geometry(), ASSETS.mesh_path("wheelchair_footrest.obj")
    )

    rear_tire_geom, rear_core_geom = _build_wheel_geometries(
        radius=REAR_WHEEL_RADIUS,
        width=REAR_WHEEL_WIDTH,
        rim_radius=0.061,
        rim_tube=0.006,
        tire_tube=0.013,
        hub_radius=0.014,
        spoke_radius=0.0038,
        spoke_count=6,
    )
    rear_tire_mesh = mesh_from_geometry(
        rear_tire_geom, ASSETS.mesh_path("wheelchair_rear_tire.obj")
    )
    rear_core_mesh = mesh_from_geometry(
        rear_core_geom, ASSETS.mesh_path("wheelchair_rear_core.obj")
    )

    caster_tire_geom, caster_core_geom = _build_wheel_geometries(
        radius=CASTER_WHEEL_RADIUS,
        width=CASTER_WHEEL_WIDTH,
        rim_radius=0.036,
        rim_tube=0.004,
        tire_tube=0.010,
        hub_radius=0.010,
        spoke_radius=0.003,
        spoke_count=5,
    )
    caster_tire_mesh = mesh_from_geometry(
        caster_tire_geom, ASSETS.mesh_path("wheelchair_caster_tire.obj")
    )
    caster_core_mesh = mesh_from_geometry(
        caster_core_geom, ASSETS.mesh_path("wheelchair_caster_core.obj")
    )

    chassis = model.part("chassis")
    chassis.visual(chassis_mesh, material=materials["frame"])
    chassis.visual(
        Box((0.406, 0.340, 0.012)),
        origin=Origin(xyz=(0.0, 0.05, 0.456)),
        material=materials["upholstery"],
    )
    chassis.visual(
        Box((0.052, 0.180, 0.024)),
        origin=Origin(xyz=(-0.205, 0.065, 0.642)),
        material=materials["plastic"],
    )
    chassis.visual(
        Box((0.052, 0.180, 0.024)),
        origin=Origin(xyz=(0.205, 0.065, 0.642)),
        material=materials["plastic"],
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.52, 0.48, 0.78)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.09, 0.39)),
    )

    backrest = model.part("backrest")
    backrest.visual(backrest_mesh, material=materials["frame"])
    backrest.visual(
        Box((0.340, 0.014, 0.280)),
        origin=Origin(xyz=(0.0, -0.015, 0.235)),
        material=materials["upholstery"],
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(-0.17, -0.06, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["rubber"],
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.17, -0.06, 0.425), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["rubber"],
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.48)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.02, 0.23)),
    )

    for name in ("rear_wheel_left", "rear_wheel_right"):
        wheel = model.part(name)
        wheel.visual(rear_tire_mesh, material=materials["rubber"])
        wheel.visual(rear_core_mesh, material=materials["alloy"])
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=REAR_WHEEL_RADIUS, length=REAR_WHEEL_WIDTH),
            mass=0.85,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    for name in ("caster_left_wheel", "caster_right_wheel"):
        wheel = model.part(name)
        wheel.visual(caster_tire_mesh, material=materials["rubber"])
        wheel.visual(caster_core_mesh, material=materials["alloy"])
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
            mass=0.28,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    for name in ("caster_left_yoke", "caster_right_yoke"):
        yoke = model.part(name)
        yoke.visual(caster_yoke_mesh, material=materials["frame"])
        yoke.inertial = Inertial.from_geometry(
            Box((0.06, 0.03, 0.15)),
            mass=0.22,
            origin=Origin(xyz=(0.0, 0.0, -0.075)),
        )

    for name in ("footrest_left", "footrest_right"):
        footrest = model.part(name)
        footrest.visual(footrest_mesh, material=materials["frame"])
        footrest.visual(
            Box((0.108, 0.082, 0.003)),
            origin=Origin(xyz=(0.0, 0.108, -0.223)),
            material=materials["rubber"],
        )
        footrest.inertial = Inertial.from_geometry(
            Box((0.13, 0.16, 0.26)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.08, -0.13)),
        )

    model.articulation(
        "backrest_fold",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="backrest",
        origin=Origin(xyz=(0.0, -0.135, 0.468)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=1.15),
    )

    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="rear_wheel_left",
        origin=Origin(xyz=(-0.25, -0.11, REAR_WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=14.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="rear_wheel_right",
        origin=Origin(xyz=(0.25, -0.11, REAR_WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=14.0),
    )

    model.articulation(
        "caster_left_swivel",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="caster_left_yoke",
        origin=Origin(xyz=(-0.236, 0.320, 0.182)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )
    model.articulation(
        "caster_right_swivel",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="caster_right_yoke",
        origin=Origin(xyz=(0.236, 0.320, 0.182)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )
    model.articulation(
        "caster_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="caster_left_yoke",
        child="caster_left_wheel",
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=14.0),
    )
    model.articulation(
        "caster_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="caster_right_yoke",
        child="caster_right_wheel",
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=14.0),
    )

    model.articulation(
        "footrest_left_fold",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="footrest_left",
        origin=Origin(xyz=(-0.095, 0.285, 0.285)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.30),
    )
    model.articulation(
        "footrest_right_fold",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="footrest_right",
        origin=Origin(xyz=(0.095, 0.285, 0.285)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "chassis",
        "rear_wheel_left",
        reason="The quick-release axle stub intentionally nests a few millimeters into the rear hub.",
    )
    ctx.allow_overlap(
        "chassis",
        "rear_wheel_right",
        reason="The quick-release axle stub intentionally nests a few millimeters into the rear hub.",
    )
    ctx.allow_overlap(
        "caster_left_yoke",
        "caster_left_wheel",
        reason="A tight transport-chair caster fork encloses the wheel hub with minimal axle clearance.",
    )
    ctx.allow_overlap(
        "caster_right_yoke",
        "caster_right_wheel",
        reason="A tight transport-chair caster fork encloses the wheel hub with minimal axle clearance.",
    )
    ctx.allow_overlap(
        "caster_left_yoke",
        "chassis",
        reason="The caster kingpin is visually modeled as a solid stem nesting into a chassis socket, which collision QC treats conservatively.",
    )
    ctx.allow_overlap(
        "caster_right_yoke",
        "chassis",
        reason="The caster kingpin is visually modeled as a solid stem nesting into a chassis socket, which collision QC treats conservatively.",
    )
    ctx.allow_overlap(
        "backrest",
        "chassis",
        reason="The folded backrest pivots on close-set rear hinge lugs that can conservatively register as touching.",
    )
    ctx.allow_overlap(
        "chassis",
        "footrest_left",
        reason="The folding footplate hinge sleeve and its receiver bracket are authored as solid visuals without the small real-world pin clearance.",
    )
    ctx.allow_overlap(
        "chassis",
        "footrest_right",
        reason="The folding footplate hinge sleeve and its receiver bracket are authored as solid visuals without the small real-world pin clearance.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("backrest", "chassis", axes="xy", max_dist=0.17)
    ctx.expect_aabb_overlap("footrest_left", "chassis", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("footrest_right", "chassis", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("caster_left_yoke", "chassis", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("caster_right_yoke", "chassis", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("rear_wheel_left", "chassis", axes="xy", min_overlap=0.003)
    ctx.expect_aabb_overlap("rear_wheel_right", "chassis", axes="xy", min_overlap=0.003)
    ctx.expect_origin_distance("rear_wheel_left", "rear_wheel_right", axes="xy", max_dist=0.55)
    ctx.expect_origin_distance("caster_left_wheel", "caster_right_wheel", axes="xy", max_dist=0.50)
    ctx.expect_origin_distance("footrest_left", "footrest_right", axes="xy", max_dist=0.24)
    ctx.expect_joint_motion_axis(
        "backrest_fold",
        "backrest",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "footrest_left_fold",
        "footrest_left",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "footrest_right_fold",
        "footrest_right",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )

    with ctx.pose(backrest_fold=1.0):
        ctx.expect_origin_distance("backrest", "chassis", axes="xy", max_dist=0.26)

    with ctx.pose(caster_left_swivel=math.pi / 2.0, caster_right_swivel=-math.pi / 2.0):
        ctx.expect_origin_distance("caster_left_wheel", "caster_left_yoke", axes="xy", max_dist=0.001)
        ctx.expect_origin_distance("caster_right_wheel", "caster_right_yoke", axes="xy", max_dist=0.001)
        ctx.expect_aabb_overlap("caster_left_wheel", "caster_left_yoke", axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap("caster_right_wheel", "caster_right_yoke", axes="xy", min_overlap=0.015)

    with ctx.pose(footrest_left_fold=1.2, footrest_right_fold=1.2):
        ctx.expect_origin_distance("footrest_left", "footrest_right", axes="xy", max_dist=0.28)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
