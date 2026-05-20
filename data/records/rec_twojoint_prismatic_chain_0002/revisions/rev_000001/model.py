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

RACK_DEPTH = 0.82
RACK_WIDTH = 0.62
RACK_HEIGHT = 0.92
RACK_POST = 0.04
RACK_BEAM = 0.03
TRAY_SUPPORT_Z = 0.34

OUTER_STAGE_LENGTH = 0.66
OUTER_STAGE_WIDTH = 0.46
OUTER_STAGE_FLOOR = 0.012
OUTER_STAGE_SIDE = 0.018
OUTER_STAGE_SIDE_HEIGHT = 0.075
OUTER_STAGE_FRONT_HEIGHT = 0.095
OUTER_STAGE_TRAVEL = 0.30
OUTER_STAGE_ORIGIN_IN_BASE = (0.10, 0.0, TRAY_SUPPORT_Z)

INNER_STAGE_LENGTH = 0.54
INNER_STAGE_WIDTH = 0.38
INNER_STAGE_FLOOR = 0.010
INNER_STAGE_SIDE = 0.014
INNER_STAGE_SIDE_HEIGHT = 0.055
INNER_STAGE_TRAVEL = 0.24
INNER_STAGE_ELEVATION = 0.024
INNER_STAGE_ORIGIN_IN_OUTER = (0.06, 0.0, INNER_STAGE_ELEVATION)

EQUIPMENT_MODULE_SIZE = (0.36, 0.30, 0.16)
EQUIPMENT_MODULE_CENTER = (0.29, 0.0, 0.090)


def _origin(center: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center)


def _cq_box(cq_module, size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq_module.Workplane("XY").box(*size).translate(center)


def _union_cq_boxes(
    filename: str, boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]
):
    import cadquery as cq

    shape = None
    for size, center in boxes:
        solid = _cq_box(cq, size, center)
        shape = solid if shape is None else shape.union(solid)
    return mesh_from_cadquery(shape, MESH_DIR / filename)


def _build_rack_frame_mesh():
    return _union_cq_boxes(
        "rack_frame.obj",
        [
            (
                (RACK_POST, RACK_POST, RACK_HEIGHT),
                (0.5 * RACK_POST, -0.5 * (RACK_WIDTH - RACK_POST), 0.5 * RACK_HEIGHT),
            ),
            (
                (RACK_POST, RACK_POST, RACK_HEIGHT),
                (0.5 * RACK_POST, 0.5 * (RACK_WIDTH - RACK_POST), 0.5 * RACK_HEIGHT),
            ),
            (
                (RACK_POST, RACK_POST, RACK_HEIGHT),
                (RACK_DEPTH - 0.5 * RACK_POST, -0.5 * (RACK_WIDTH - RACK_POST), 0.5 * RACK_HEIGHT),
            ),
            (
                (RACK_POST, RACK_POST, RACK_HEIGHT),
                (RACK_DEPTH - 0.5 * RACK_POST, 0.5 * (RACK_WIDTH - RACK_POST), 0.5 * RACK_HEIGHT),
            ),
            (
                (RACK_DEPTH - RACK_POST, RACK_BEAM, RACK_BEAM),
                (0.5 * RACK_DEPTH, -0.5 * (RACK_WIDTH - RACK_BEAM), 0.5 * RACK_BEAM),
            ),
            (
                (RACK_DEPTH - RACK_POST, RACK_BEAM, RACK_BEAM),
                (0.5 * RACK_DEPTH, 0.5 * (RACK_WIDTH - RACK_BEAM), 0.5 * RACK_BEAM),
            ),
            (
                (RACK_BEAM, RACK_WIDTH - RACK_POST, RACK_BEAM),
                (0.5 * RACK_BEAM, 0.0, 0.5 * RACK_BEAM),
            ),
            (
                (RACK_DEPTH - RACK_POST, RACK_BEAM, RACK_BEAM),
                (0.5 * RACK_DEPTH, -0.5 * (RACK_WIDTH - RACK_BEAM), RACK_HEIGHT - 0.5 * RACK_BEAM),
            ),
            (
                (RACK_DEPTH - RACK_POST, RACK_BEAM, RACK_BEAM),
                (0.5 * RACK_DEPTH, 0.5 * (RACK_WIDTH - RACK_BEAM), RACK_HEIGHT - 0.5 * RACK_BEAM),
            ),
            (
                (RACK_BEAM, RACK_WIDTH - RACK_POST, RACK_BEAM),
                (0.5 * RACK_BEAM, 0.0, RACK_HEIGHT - 0.5 * RACK_BEAM),
            ),
            ((0.012, RACK_WIDTH - 0.08, 0.72), (0.006, 0.0, 0.46)),
            (
                (0.08, 0.20, TRAY_SUPPORT_Z),
                (OUTER_STAGE_ORIGIN_IN_BASE[0], 0.0, 0.5 * TRAY_SUPPORT_Z),
            ),
            ((0.14, 0.26, 0.02), (0.12, 0.0, TRAY_SUPPORT_Z - 0.01)),
            ((0.016, 0.04, 0.80), (0.12, -0.22, 0.46)),
            ((0.016, 0.04, 0.80), (0.12, 0.22, 0.46)),
        ],
    )


def _build_outer_stage_mesh():
    return _union_cq_boxes(
        "outer_slide_stage.obj",
        [
            (
                (OUTER_STAGE_LENGTH, OUTER_STAGE_WIDTH, OUTER_STAGE_FLOOR),
                (0.5 * OUTER_STAGE_LENGTH, 0.0, 0.5 * OUTER_STAGE_FLOOR),
            ),
            (
                (OUTER_STAGE_LENGTH - 0.04, OUTER_STAGE_SIDE, OUTER_STAGE_SIDE_HEIGHT),
                (
                    0.5 * OUTER_STAGE_LENGTH,
                    -0.5 * (OUTER_STAGE_WIDTH - OUTER_STAGE_SIDE),
                    0.5 * OUTER_STAGE_SIDE_HEIGHT,
                ),
            ),
            (
                (OUTER_STAGE_LENGTH - 0.04, OUTER_STAGE_SIDE, OUTER_STAGE_SIDE_HEIGHT),
                (
                    0.5 * OUTER_STAGE_LENGTH,
                    0.5 * (OUTER_STAGE_WIDTH - OUTER_STAGE_SIDE),
                    0.5 * OUTER_STAGE_SIDE_HEIGHT,
                ),
            ),
            ((0.014, OUTER_STAGE_WIDTH, 0.060), (0.007, 0.0, 0.030)),
            (
                (0.024, OUTER_STAGE_WIDTH + 0.04, OUTER_STAGE_FRONT_HEIGHT),
                (OUTER_STAGE_LENGTH - 0.012, 0.0, 0.5 * OUTER_STAGE_FRONT_HEIGHT),
            ),
            ((0.10, 0.18, INNER_STAGE_ELEVATION), (0.08, 0.0, 0.5 * INNER_STAGE_ELEVATION)),
        ],
    )


def _build_inner_stage_tray_mesh():
    return _union_cq_boxes(
        "inner_slide_stage_tray.obj",
        [
            (
                (INNER_STAGE_LENGTH, INNER_STAGE_WIDTH, INNER_STAGE_FLOOR),
                (0.5 * INNER_STAGE_LENGTH, 0.0, 0.5 * INNER_STAGE_FLOOR),
            ),
            (
                (INNER_STAGE_LENGTH - 0.04, INNER_STAGE_SIDE, INNER_STAGE_SIDE_HEIGHT),
                (
                    0.5 * INNER_STAGE_LENGTH,
                    -0.5 * (INNER_STAGE_WIDTH - INNER_STAGE_SIDE),
                    0.5 * INNER_STAGE_SIDE_HEIGHT,
                ),
            ),
            (
                (INNER_STAGE_LENGTH - 0.04, INNER_STAGE_SIDE, INNER_STAGE_SIDE_HEIGHT),
                (
                    0.5 * INNER_STAGE_LENGTH,
                    0.5 * (INNER_STAGE_WIDTH - INNER_STAGE_SIDE),
                    0.5 * INNER_STAGE_SIDE_HEIGHT,
                ),
            ),
            ((0.012, INNER_STAGE_WIDTH, 0.050), (0.006, 0.0, 0.025)),
            ((0.018, INNER_STAGE_WIDTH + 0.03, 0.060), (INNER_STAGE_LENGTH - 0.009, 0.0, 0.030)),
        ],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_linear_equipment_rack", assets=ASSETS)

    model.material("rack_steel", rgba=(0.28, 0.30, 0.34, 1.0))
    model.material("slide_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("equipment", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("panel", rgba=(0.42, 0.47, 0.54, 1.0))
    model.material("accent", rgba=(0.84, 0.22, 0.10, 1.0))

    base = model.part("rack_base")
    base.visual(_build_rack_frame_mesh(), material="rack_steel")
    base.visual(
        Box((0.14, 0.26, 0.02)),
        origin=_origin((0.12, 0.0, TRAY_SUPPORT_Z - 0.01)),
        material="panel",
    )







    base.inertial = Inertial.from_geometry(
        Box((RACK_DEPTH, RACK_WIDTH, RACK_HEIGHT)),
        mass=42.0,
        origin=_origin((0.5 * RACK_DEPTH, 0.0, 0.5 * RACK_HEIGHT)),
    )

    outer = model.part("outer_slide_stage")
    outer.visual(_build_outer_stage_mesh(), material="slide_dark")
    outer.visual(
        Box((0.012, 0.24, 0.018)),
        origin=_origin((OUTER_STAGE_LENGTH + 0.006, 0.0, 0.050)),
        material="accent",
    )





    outer.inertial = Inertial.from_geometry(
        Box((OUTER_STAGE_LENGTH, 0.50, 0.10)),
        mass=10.5,
        origin=_origin((0.5 * OUTER_STAGE_LENGTH, 0.0, 0.05)),
    )

    inner = model.part("inner_slide_stage")
    inner.visual(_build_inner_stage_tray_mesh(), material="slide_dark")
    inner.visual(
        Box(EQUIPMENT_MODULE_SIZE), origin=_origin(EQUIPMENT_MODULE_CENTER), material="equipment"
    )
    inner.visual(Box((0.22, 0.24, 0.008)), origin=_origin((0.29, 0.0, 0.174)), material="panel")
    inner.visual(Box((0.10, 0.30, 0.012)), origin=_origin((0.47, 0.0, 0.108)), material="accent")



    inner.inertial = Inertial.from_geometry(
        Box((INNER_STAGE_LENGTH, 0.40, 0.20)),
        mass=8.0,
        origin=_origin((0.5 * INNER_STAGE_LENGTH, 0.0, 0.10)),
    )

    model.articulation(
        "rack_to_outer_stage",
        ArticulationType.PRISMATIC,
        parent="rack_base",
        child="outer_slide_stage",
        origin=Origin(xyz=OUTER_STAGE_ORIGIN_IN_BASE),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=OUTER_STAGE_TRAVEL, effort=400.0, velocity=0.40
        ),
    )
    model.articulation(
        "outer_stage_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent="outer_slide_stage",
        child="inner_slide_stage",
        origin=Origin(xyz=INNER_STAGE_ORIGIN_IN_OUTER),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=INNER_STAGE_TRAVEL, effort=250.0, velocity=0.35
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("outer_slide_stage", "rack_base", axes="xy", min_overlap=0.30)
    ctx.expect_aabb_overlap("inner_slide_stage", "outer_slide_stage", axes="xy", min_overlap=0.25)
    ctx.expect_aabb_overlap("inner_slide_stage", "rack_base", axes="xy", min_overlap=0.20)
    ctx.expect_origin_distance("outer_slide_stage", "rack_base", axes="xy", max_dist=0.11)
    ctx.expect_origin_distance("inner_slide_stage", "outer_slide_stage", axes="xy", max_dist=0.07)
    ctx.expect_joint_motion_axis(
        "rack_to_outer_stage",
        "outer_slide_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "outer_stage_to_inner_stage",
        "inner_slide_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.08,
    )

    with ctx.pose(rack_to_outer_stage=OUTER_STAGE_TRAVEL):
        ctx.expect_aabb_overlap("outer_slide_stage", "rack_base", axes="xy", min_overlap=0.20)
        ctx.expect_aabb_overlap("inner_slide_stage", "outer_slide_stage", axes="xy", min_overlap=0.25)
        ctx.expect_origin_distance("outer_slide_stage", "rack_base", axes="xy", max_dist=0.41)

    with ctx.pose(outer_stage_to_inner_stage=INNER_STAGE_TRAVEL):
        ctx.expect_aabb_overlap("inner_slide_stage", "outer_slide_stage", axes="xy", min_overlap=0.18)
        ctx.expect_aabb_overlap("inner_slide_stage", "rack_base", axes="xy", min_overlap=0.18)
        ctx.expect_origin_distance("inner_slide_stage", "rack_base", axes="xy", max_dist=0.41)

    with ctx.pose(
        rack_to_outer_stage=OUTER_STAGE_TRAVEL, outer_stage_to_inner_stage=INNER_STAGE_TRAVEL
    ):
        ctx.expect_aabb_overlap("inner_slide_stage", "outer_slide_stage", axes="xy", min_overlap=0.15)
        ctx.expect_aabb_overlap("inner_slide_stage", "rack_base", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("inner_slide_stage", "outer_slide_stage", axes="xy", max_dist=0.31)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
