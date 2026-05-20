from __future__ import annotations

from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
def _build_fork_frame_shape() -> cq.Workplane:
    tine_width = 0.055
    tine_offset_y = 0.115
    tine_profile = [
        (-0.010, 0.100),
        (0.026, 0.100),
        (0.026, -0.080),
        (0.095, -0.080),
        (0.120, -0.095),
        (0.535, -0.095),
        (0.650, -0.118),
        (0.650, -0.138),
        (0.026, -0.133),
        (0.026, -0.100),
        (-0.010, -0.100),
    ]

    frame = cq.Workplane("XY").box(0.040, 0.340, 0.100)
    for side in (-1.0, 1.0):
        tine = (
            cq.Workplane("XZ")
            .polyline(tine_profile)
            .close()
            .extrude(tine_width)
            .translate((0.0, side * tine_offset_y - 0.5 * tine_width, 0.0))
        )
        frame = frame.union(tine)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_mounted_fork_carriage", assets=ASSETS)

    model.material("mast_steel", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("ram_steel", rgba=(0.63, 0.65, 0.69, 1.0))
    model.material("carriage_paint", rgba=(0.86, 0.61, 0.14, 1.0))
    model.material("fork_steel", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("roller_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    mast = model.part("mast")
    for y in (-0.165, 0.165):
        mast.visual(
            Box((0.060, 0.060, 1.300)),
            origin=Origin(xyz=(-0.005, y, 0.720)),
            material="mast_steel",
        )
        mast.visual(
            Box((0.040, 0.018, 1.240)),
            origin=Origin(xyz=(0.035, y * 0.79, 0.720)),
            material="mast_steel",
        )
    mast.visual(
        Box((0.090, 0.460, 0.100)),
        origin=Origin(xyz=(0.000, 0.000, 0.050)),
        material="mast_steel",
    )
    mast.visual(
        Box((0.060, 0.400, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 1.395)),
        material="mast_steel",
    )
    mast.visual(
        Box((0.035, 0.100, 1.180)),
        origin=Origin(xyz=(0.0375, 0.000, 0.690)),
        material="mast_steel",
    )
    mast.visual(
        Cylinder(radius=0.030, length=1.150),
        origin=Origin(xyz=(-0.018, 0.000, 0.675)),
        material="ram_steel",
    )
    mast.visual(
        Box((0.050, 0.120, 0.180)),
        origin=Origin(xyz=(0.010, 0.000, 1.280)),
        material="mast_steel",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.095, 0.460, 1.420)),
        mass=92.0,
        origin=Origin(xyz=(0.010, 0.000, 0.710)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.080, 0.120, 0.180)),
        origin=Origin(xyz=(0.040, 0.000, 0.000)),
        material="mast_steel",
    )
    for y in (-0.160, 0.160):
        carriage.visual(
            Box((0.080, 0.050, 0.460)),
            origin=Origin(xyz=(0.040, y, -0.010)),
            material="carriage_paint",
        )
        for z in (-0.120, 0.120):
            carriage.visual(
                Box((0.020, 0.045, 0.110)),
                origin=Origin(xyz=(0.010, y, z)),
                material="carriage_paint",
            )
            carriage.visual(
                Cylinder(radius=0.021, length=0.030),
                origin=Origin(xyz=(0.030, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material="roller_rubber",
            )
    carriage.visual(
        Box((0.030, 0.420, 0.360)),
        origin=Origin(xyz=(0.085, 0.000, 0.000)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.050, 0.360, 0.035)),
        origin=Origin(xyz=(0.095, 0.000, 0.180)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.060, 0.420, 0.045)),
        origin=Origin(xyz=(0.095, 0.000, -0.180)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.040, 0.380, 0.040)),
        origin=Origin(xyz=(0.110, 0.000, -0.085)),
        material="mast_steel",
    )
    for y in (-0.115, 0.000, 0.115):
        carriage.visual(
            Box((0.016, 0.016, 0.250)),
            origin=Origin(xyz=(0.090, y, 0.322)),
            material="carriage_paint",
        )
    carriage.visual(
        Box((0.025, 0.300, 0.030)),
        origin=Origin(xyz=(0.090, 0.000, 0.463)),
        material="carriage_paint",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.130, 0.420, 0.670)),
        mass=34.0,
        origin=Origin(xyz=(0.065, 0.000, 0.131)),
    )

    fork_frame = model.part("fork_frame")
    fork_frame.visual(
        mesh_from_cadquery(_build_fork_frame_shape(), "fork_frame.obj", assets=ASSETS),
        material="fork_steel",
    )
    fork_frame.inertial = Inertial.from_geometry(
        Box((0.660, 0.340, 0.238)),
        mass=22.0,
        origin=Origin(xyz=(0.320, 0.000, -0.019)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent="mast",
        child="carriage",
        origin=Origin(xyz=(0.057, 0.000, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.480,
            effort=4500.0,
            velocity=0.350,
        ),
    )
    model.articulation(
        "carriage_to_fork_frame",
        ArticulationType.FIXED,
        parent="carriage",
        child="fork_frame",
        origin=Origin(xyz=(0.128, 0.000, -0.085)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "carriage",
        "fork_frame",
        reason="The fork crossbar seats slightly into the carriage hook rail.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_joint_motion_axis(
        "mast_to_carriage",
        "carriage",
        world_axis="z",
        direction="positive",
        min_delta=0.20,
    )
    with ctx.pose(mast_to_carriage=0.0):
        ctx.expect_origin_distance("carriage", "mast", axes="xy", max_dist=0.13)
        ctx.expect_origin_distance("fork_frame", "carriage", axes="xy", max_dist=0.30)
        ctx.expect_aabb_overlap("fork_frame", "carriage", axes="xy", min_overlap=0.008)
    with ctx.pose(mast_to_carriage=0.48):
        ctx.expect_origin_distance("carriage", "mast", axes="xy", max_dist=0.13)
        ctx.expect_origin_distance("fork_frame", "carriage", axes="xy", max_dist=0.30)
        ctx.expect_aabb_overlap("fork_frame", "carriage", axes="xy", min_overlap=0.008)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
