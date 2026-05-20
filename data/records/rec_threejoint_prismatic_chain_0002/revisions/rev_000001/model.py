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
import cadquery as cq

BASE_DIMS = {
    "length": 0.48,
    "width": 0.34,
    "height": 0.075,
    "floor_t": 0.008,
    "floor_width": 0.24,
    "rail_t": 0.030,
    "rail_h": 0.060,
    "rear_beam_t": 0.018,
    "rear_beam_h": 0.024,
}

STAGE1_DIMS = {
    "length": 0.40,
    "width": 0.266,
    "height": 0.052,
    "floor_t": 0.006,
    "floor_width": 0.18,
    "rail_t": 0.022,
    "rail_h": 0.042,
    "rear_beam_t": 0.016,
    "rear_beam_h": 0.020,
}

STAGE2_DIMS = {
    "length": 0.30,
    "width": 0.206,
    "height": 0.038,
    "floor_t": 0.005,
    "floor_width": 0.14,
    "rail_t": 0.018,
    "rail_h": 0.032,
    "rear_beam_t": 0.014,
    "rear_beam_h": 0.016,
}

STAGE3_DIMS = {
    "length": 0.22,
    "width": 0.154,
    "height": 0.028,
    "floor_t": 0.004,
    "floor_width": 0.13,
    "rail_t": 0.014,
    "rail_h": 0.018,
    "rear_beam_t": 0.012,
    "rear_beam_h": 0.014,
}

BASE_TO_STAGE1 = Origin(xyz=(0.050, 0.0, 0.012))
STAGE1_TO_STAGE2 = Origin(xyz=(0.055, 0.0, 0.010))
STAGE2_TO_STAGE3 = Origin(xyz=(0.045, 0.0, 0.008))


def _box_origin(length: float, height: float) -> Origin:
    return Origin(xyz=(length / 2.0, 0.0, height / 2.0))


def _cad_box(cq, size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _make_channel_shape(cq, dims: dict[str, float]):
    length = dims["length"]
    width = dims["width"]
    floor_t = dims["floor_t"]
    floor_width = dims["floor_width"]
    rail_t = dims["rail_t"]
    rail_h = dims["rail_h"]
    rear_beam_t = dims["rear_beam_t"]
    rear_beam_h = dims["rear_beam_h"]

    shape = _cad_box(cq, (length, floor_width, floor_t), (length / 2.0, 0.0, floor_t / 2.0))
    shape = shape.union(
        _cad_box(
            cq,
            (length, rail_t, rail_h),
            (length / 2.0, (width / 2.0) - (rail_t / 2.0), rail_h / 2.0),
        )
    )
    shape = shape.union(
        _cad_box(
            cq,
            (length, rail_t, rail_h),
            (length / 2.0, -(width / 2.0) + (rail_t / 2.0), rail_h / 2.0),
        )
    )
    shape = shape.union(
        _cad_box(
            cq,
            (rear_beam_t, floor_width, rear_beam_h),
            (rear_beam_t / 2.0, 0.0, rear_beam_h / 2.0),
        )
    )
    return shape


def _make_base_shape(cq):
    shape = _make_channel_shape(cq, BASE_DIMS)
    shape = shape.union(_cad_box(cq, (0.12, 0.14, 0.022), (0.085, 0.0, 0.049)))
    shape = shape.union(_cad_box(cq, (0.08, 0.26, 0.010), (0.42, 0.0, 0.065)))
    shape = shape.union(_cad_box(cq, (0.024, 0.040, 0.020), (0.435, 0.105, 0.038)))
    shape = shape.union(_cad_box(cq, (0.024, 0.040, 0.020), (0.435, -0.105, 0.038)))
    return shape


def _make_stage1_shape(cq):
    shape = _make_channel_shape(cq, STAGE1_DIMS)
    shape = shape.union(_cad_box(cq, (0.22, 0.14, 0.006), (0.21, 0.0, 0.009)))
    shape = shape.union(_cad_box(cq, (0.016, 0.14, 0.018), (0.392, 0.0, 0.014)))
    shape = shape.union(_cad_box(cq, (0.050, 0.040, 0.018), (0.110, 0.0, 0.034)))
    return shape


def _make_stage2_shape(cq):
    shape = _make_channel_shape(cq, STAGE2_DIMS)
    shape = shape.union(_cad_box(cq, (0.16, 0.11, 0.005), (0.17, 0.0, 0.008)))
    shape = shape.union(_cad_box(cq, (0.014, 0.11, 0.014), (0.293, 0.0, 0.012)))
    shape = shape.union(_cad_box(cq, (0.040, 0.030, 0.014), (0.095, 0.0, 0.026)))
    return shape


def _make_stage3_shape(cq):
    shape = _make_channel_shape(cq, STAGE3_DIMS)
    shape = shape.union(_cad_box(cq, (0.16, 0.11, 0.004), (0.10, 0.0, 0.006)))
    shape = shape.union(_cad_box(cq, (0.08, 0.146, 0.010), (0.180, 0.0, 0.009)))
    shape = shape.union(_cad_box(cq, (0.026, 0.120, 0.008), (0.220, 0.0, 0.004)))
    shape = shape.union(_cad_box(cq, (0.030, 0.026, 0.016), (0.060, 0.050, 0.022)))
    shape = shape.union(_cad_box(cq, (0.030, 0.026, 0.016), (0.060, -0.050, 0.022)))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_shuttle", assets=ASSETS)

    model.material("frame_graphite", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("stage_outer", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("stage_inner", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("shuttle_orange", rgba=(0.93, 0.47, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(cq), MESH_DIR / "warehouse_shuttle_base.obj"),
        material="frame_graphite",
    )

    base.inertial = Inertial.from_geometry(
        Box((BASE_DIMS["length"], BASE_DIMS["width"], BASE_DIMS["height"])),
        mass=18.0,
        origin=_box_origin(BASE_DIMS["length"], BASE_DIMS["height"]),
    )

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_make_stage1_shape(cq), MESH_DIR / "warehouse_shuttle_stage1.obj"),
        material="stage_outer",
    )
    stage1.inertial = Inertial.from_geometry(
        Box((STAGE1_DIMS["length"], STAGE1_DIMS["width"], STAGE1_DIMS["height"])),
        mass=6.5,
        origin=_box_origin(STAGE1_DIMS["length"], STAGE1_DIMS["height"]),
    )

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_make_stage2_shape(cq), MESH_DIR / "warehouse_shuttle_stage2.obj"),
        material="stage_inner",
    )
    stage2.inertial = Inertial.from_geometry(
        Box((STAGE2_DIMS["length"], STAGE2_DIMS["width"], STAGE2_DIMS["height"])),
        mass=3.4,
        origin=_box_origin(STAGE2_DIMS["length"], STAGE2_DIMS["height"]),
    )

    stage3 = model.part("stage3")
    stage3.visual(
        mesh_from_cadquery(_make_stage3_shape(cq), MESH_DIR / "warehouse_shuttle_stage3.obj"),
        material="shuttle_orange",
    )

    stage3.inertial = Inertial.from_geometry(
        Box((STAGE3_DIMS["length"], STAGE3_DIMS["width"], STAGE3_DIMS["height"])),
        mass=1.8,
        origin=_box_origin(STAGE3_DIMS["length"], STAGE3_DIMS["height"]),
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent="base",
        child="stage1",
        origin=BASE_TO_STAGE1,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.22, effort=1200.0, velocity=1.2),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent="stage1",
        child="stage2",
        origin=STAGE1_TO_STAGE2,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.18, effort=900.0, velocity=1.2),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent="stage2",
        child="stage3",
        origin=STAGE2_TO_STAGE3,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.16, effort=700.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("stage1", "base", axes="xy", max_dist=0.06)
    ctx.expect_origin_distance("stage2", "stage1", axes="xy", max_dist=0.06)
    ctx.expect_origin_distance("stage3", "stage2", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("stage3", "base", axes="xy", max_dist=0.16)

    ctx.expect_aabb_overlap("stage1", "base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_overlap("stage2", "stage1", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_overlap("stage3", "stage2", axes="xy", min_overlap=0.09)
    ctx.expect_aabb_overlap("stage3", "base", axes="xy", min_overlap=0.08)

    ctx.expect_joint_motion_axis(
        "base_to_stage1",
        "stage1",
        world_axis="x",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "stage1_to_stage2",
        "stage2",
        world_axis="x",
        direction="positive",
        min_delta=0.07,
    )
    ctx.expect_joint_motion_axis(
        "stage2_to_stage3",
        "stage3",
        world_axis="x",
        direction="positive",
        min_delta=0.06,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
