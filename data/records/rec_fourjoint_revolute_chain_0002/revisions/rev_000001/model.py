from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
# In sdk_hybrid, author visual meshes with cadquery + mesh_from_cadquery.
ARM_BASE_Z = 0.12
BEAM_WIDTH = 0.05
BEAM_HEIGHT = 0.032
COLLISION_WIDTH = 0.038
COLLISION_HEIGHT = 0.028
JOINT_BARREL_RADIUS = 0.028
JOINT_BARREL_THICKNESS = 0.024
COLLISION_END_CLEARANCE = 0.018
SENSOR_POD_SIZE = (0.08, 0.05, 0.04)
SENSOR_POD_ORIGIN = (0.245, 0.0, 0.004)
SENSOR_LENS_SIZE = (0.018, 0.024, 0.024)
SENSOR_LENS_ORIGIN = (0.296, 0.0, 0.004)


def _build_base_mesh():
    import cadquery as cq

    foot = (
        cq.Workplane("XY")
        .box(0.18, 0.16, 0.08)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.0, 0.0, 0.04))
    )
    mast = cq.Workplane("XY").box(0.05, 0.06, 0.10).translate((-0.01, 0.0, 0.05))
    rear_brace = (
        cq.Workplane("XZ").rect(0.09, 0.05).extrude(0.024, both=True).translate((-0.03, 0.0, 0.075))
    )
    shoulder_barrel = (
        cq.Workplane("XZ").circle(0.032).extrude(0.07, both=True).translate((0.0, 0.0, ARM_BASE_Z))
    )
    shape = foot.union(mast).union(rear_brace).union(shoulder_barrel)
    return mesh_from_cadquery(shape, MESH_DIR / "base_mount.obj")


def _build_segment_mesh(name: str, length: float):
    import cadquery as cq

    beam_length = max(length - 0.07, length * 0.72)
    beam = (
        cq.Workplane("XY")
        .box(beam_length, BEAM_WIDTH, BEAM_HEIGHT)
        .edges("|X")
        .fillet(0.005)
        .translate((length / 2.0, 0.0, 0.0))
    )
    web = (
        cq.Workplane("XZ")
        .rect(max(length - 0.08, 0.08), BEAM_HEIGHT * 0.72)
        .extrude(BEAM_WIDTH * 0.55, both=True)
        .translate((length / 2.0, 0.0, 0.0))
    )
    proximal_barrel = (
        cq.Workplane("XZ").circle(JOINT_BARREL_RADIUS).extrude(JOINT_BARREL_THICKNESS, both=True)
    )
    distal_barrel = (
        cq.Workplane("XZ")
        .circle(JOINT_BARREL_RADIUS)
        .extrude(JOINT_BARREL_THICKNESS, both=True)
        .translate((length, 0.0, 0.0))
    )
    shape = beam.union(web).union(proximal_barrel).union(distal_barrel)
    return mesh_from_cadquery(shape, MESH_DIR / f"{name}.obj")


def _add_base_geometry(base_part) -> None:
    base_part.visual(_build_base_mesh(), material="base_gray")
    base_part.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 0.12)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )


def _add_segment_geometry(
    part,
    *,
    name: str,
    length: float,
    material: str,
    mass: float,
    has_sensor_head: bool = False,
) -> None:
    part.visual(_build_segment_mesh(name, length), material=material)

    if has_sensor_head:
        part.visual(
            Box((0.04, 0.032, 0.018)),
            origin=Origin(xyz=(length + 0.012, 0.0, -0.004)),
            material=material,
        )
        part.visual(
            Box(SENSOR_POD_SIZE),
            origin=Origin(
                xyz=(
                    length + SENSOR_POD_ORIGIN[0] - 0.20,
                    SENSOR_POD_ORIGIN[1],
                    SENSOR_POD_ORIGIN[2],
                )
            ),
            material="sensor_black",
        )
        part.visual(
            Box(SENSOR_LENS_SIZE),
            origin=Origin(
                xyz=(
                    length + SENSOR_LENS_ORIGIN[0] - 0.20,
                    SENSOR_LENS_ORIGIN[1],
                    SENSOR_LENS_ORIGIN[2],
                )
            ),
            material="optic_blue",
        )

    collision_length = max(length - (2.0 * JOINT_BARREL_RADIUS), 0.08)




    inertial_length = length
    inertial_origin_x = length / 2.0
    inertial_height = 0.03
    inertial_origin_z = 0.0
    if has_sensor_head:
        pod_origin_x = length + SENSOR_POD_ORIGIN[0] - 0.20

        lens_origin_x = length + SENSOR_LENS_ORIGIN[0] - 0.20

        inertial_length = length + 0.14
        inertial_origin_x = (length + 0.14) / 2.0
        inertial_height = 0.04
        inertial_origin_z = 0.002

    part.inertial = Inertial.from_geometry(
        Box((inertial_length, COLLISION_WIDTH, inertial_height)),
        mass=mass,
        origin=Origin(xyz=(inertial_origin_x, 0.0, inertial_origin_z)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_inspection_arm", assets=ASSETS)

    model.material("base_gray", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("steel_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    model.material("arm_orange", rgba=(0.92, 0.45, 0.09, 1.0))
    model.material("arm_charcoal", rgba=(0.20, 0.22, 0.26, 1.0))
    model.material("sensor_black", rgba=(0.08, 0.09, 0.11, 1.0))
    model.material("optic_blue", rgba=(0.35, 0.63, 0.88, 1.0))

    base_mount = model.part("base_mount")
    _add_base_geometry(base_mount)

    segment_1 = model.part("segment_1")
    _add_segment_geometry(
        segment_1,
        name="segment_1",
        length=0.42,
        material="arm_orange",
        mass=3.1,
    )

    segment_2 = model.part("segment_2")
    _add_segment_geometry(
        segment_2,
        name="segment_2",
        length=0.34,
        material="arm_charcoal",
        mass=2.4,
    )

    segment_3 = model.part("segment_3")
    _add_segment_geometry(
        segment_3,
        name="segment_3",
        length=0.28,
        material="arm_orange",
        mass=1.8,
    )

    segment_4 = model.part("segment_4")
    _add_segment_geometry(
        segment_4,
        name="segment_4",
        length=0.20,
        material="arm_charcoal",
        mass=1.4,
        has_sensor_head=True,
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent="base_mount",
        child="segment_1",
        origin=Origin(xyz=(0.0, 0.0, ARM_BASE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.95,
            effort=90.0,
            velocity=1.1,
        ),
    )
    model.articulation(
        "elbow_hinge_1",
        ArticulationType.REVOLUTE,
        parent="segment_1",
        child="segment_2",
        origin=Origin(xyz=(0.42, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.85,
            effort=70.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "elbow_hinge_2",
        ArticulationType.REVOLUTE,
        parent="segment_2",
        child="segment_3",
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.75,
            effort=45.0,
            velocity=1.7,
        ),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent="segment_3",
        child="segment_4",
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.60,
            effort=25.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.0025,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("segment_1", "base_mount", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("segment_1", "base_mount", axes="xy", max_dist=0.25)
    ctx.expect_origin_gap("segment_1", "base_mount", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("segment_2", "base_mount", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("segment_3", "base_mount", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("segment_4", "base_mount", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("segment_1", "base_mount", axis="z", max_gap=0.01, max_penetration=0.012)
    ctx.expect_aabb_gap("segment_2", "base_mount", axis="z", max_gap=0.01, max_penetration=0.012)
    ctx.expect_aabb_gap("segment_3", "base_mount", axis="z", max_gap=0.01, max_penetration=0.012)
    ctx.expect_aabb_gap("segment_4", "base_mount", axis="z", max_gap=0.01, max_penetration=0.012)

    ctx.expect_joint_motion_axis(
        "shoulder_hinge",
        "segment_1",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "elbow_hinge_1",
        "segment_2",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "elbow_hinge_2",
        "segment_3",
        world_axis="z",
        direction="positive",
        min_delta=0.018,
    )
    ctx.expect_joint_motion_axis(
        "wrist_hinge",
        "segment_4",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
