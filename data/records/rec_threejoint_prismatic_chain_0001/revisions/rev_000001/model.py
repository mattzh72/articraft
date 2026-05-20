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
import cadquery as cq


BASE_OUTER_X = 0.040
BASE_OUTER_Y = 0.030
BASE_WALL = 0.0025
BASE_HEIGHT = 0.340

MIDDLE_OUTER_X = 0.032
MIDDLE_OUTER_Y = 0.022
MIDDLE_WALL = 0.0020
MIDDLE_HEIGHT = 0.300
MIDDLE_EMBED = 0.180
MIDDLE_MIN_INSERT = 0.060

UPPER_OUTER_X = 0.025
UPPER_OUTER_Y = 0.016
UPPER_WALL = 0.0018
UPPER_HEIGHT = 0.260
UPPER_EMBED = 0.160
UPPER_MIN_INSERT = 0.055

TOP_CAP_HEIGHT = 0.004
ANTENNA_TIP_RADIUS = 0.003
ANTENNA_TIP_LENGTH = 0.180


def _box_origin(center_xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center_xyz)


def _tube_center_z(bottom_z: float, height: float) -> float:
    return bottom_z + (0.5 * height)


def _tube_shape(
    *,
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    bottom_z: float,
    collar_height: float = 0.0,
    collar_growth: float = 0.0,
):
    outer = (
        cq.Workplane("XY")
        .box(outer_x, outer_y, height, centered=(True, True, False))
        .translate((0.0, 0.0, bottom_z))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            outer_x - (2.0 * wall),
            outer_y - (2.0 * wall),
            height + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_z - 0.002))
    )
    tube = outer.cut(inner)

    if collar_height > 0.0 and collar_growth > 0.0:
        collar_outer = (
            cq.Workplane("XY")
            .box(
                outer_x + (2.0 * collar_growth),
                outer_y + (2.0 * collar_growth),
                collar_height,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, bottom_z + height - collar_height))
        )
        collar_inner = (
            cq.Workplane("XY")
            .box(
                outer_x - (2.0 * wall),
                outer_y - (2.0 * wall),
                collar_height + 0.004,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, bottom_z + height - collar_height - 0.002))
        )
        tube = tube.union(collar_outer.cut(collar_inner))

    return tube


def _top_stage_shape():
    bottom_z = -UPPER_EMBED
    sleeve = _tube_shape(
        outer_x=UPPER_OUTER_X,
        outer_y=UPPER_OUTER_Y,
        height=UPPER_HEIGHT,
        wall=UPPER_WALL,
        bottom_z=bottom_z,
        collar_height=0.020,
        collar_growth=0.002,
    )
    cap = (
        cq.Workplane("XY")
        .box(
            UPPER_OUTER_X - (2.0 * UPPER_WALL),
            UPPER_OUTER_Y - (2.0 * UPPER_WALL),
            TOP_CAP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_z + UPPER_HEIGHT))
    )
    tip = (
        cq.Workplane("XY")
        .circle(ANTENNA_TIP_RADIUS)
        .extrude(ANTENNA_TIP_LENGTH)
        .translate((0.0, 0.0, bottom_z + UPPER_HEIGHT + TOP_CAP_HEIGHT))
    )
    return sleeve.union(cap).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_antenna_mast", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.82, 1.0))
    dark_mount = model.material("dark_mount", rgba=(0.18, 0.18, 0.20, 1.0))

    base = model.part("base_section")
    middle = model.part("middle_section")
    upper = model.part("upper_section")

    base.visual(
        mesh_from_cadquery(
            _tube_shape(
                outer_x=BASE_OUTER_X,
                outer_y=BASE_OUTER_Y,
                height=BASE_HEIGHT,
                wall=BASE_WALL,
                bottom_z=-BASE_HEIGHT,
                collar_height=0.030,
                collar_growth=0.003,
            )
            .union(
                cq.Workplane("XY")
                .box(0.062, 0.048, 0.055, centered=(True, True, False))
                .translate((0.0, 0.0, -BASE_HEIGHT - 0.055))
            )
            .union(
                cq.Workplane("XY")
                .box(0.078, 0.060, 0.012, centered=(True, True, False))
                .translate((0.0, 0.0, -BASE_HEIGHT - 0.067))
            ),
            MESH_DIR / "base_section.obj",
        ),
        material=dark_mount,
    )
    middle.visual(
        mesh_from_cadquery(
            _tube_shape(
                outer_x=MIDDLE_OUTER_X,
                outer_y=MIDDLE_OUTER_Y,
                height=MIDDLE_HEIGHT,
                wall=MIDDLE_WALL,
                bottom_z=-MIDDLE_EMBED,
                collar_height=0.024,
                collar_growth=0.002,
            ),
            MESH_DIR / "middle_section.obj",
        ),
        material=aluminum,
    )
    upper.visual(
        mesh_from_cadquery(_top_stage_shape(), MESH_DIR / "upper_section.obj"),
        material=aluminum,
    )

    base.inertial = Inertial.from_geometry(
        Box((0.078, 0.060, 0.407)),
        mass=1.9,
        origin=_box_origin((0.0, 0.0, -0.2035)),
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_OUTER_X, MIDDLE_OUTER_Y, MIDDLE_HEIGHT)),
        mass=0.72,
        origin=_box_origin((0.0, 0.0, -MIDDLE_EMBED + (0.5 * MIDDLE_HEIGHT))),
    )
    upper.inertial = Inertial.from_geometry(
        Box((0.026, 0.018, 0.444)),
        mass=0.48,
        origin=_box_origin((0.0, 0.0, 0.062)),
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent="base_section",
        child="middle_section",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_EMBED - MIDDLE_MIN_INSERT,
            effort=70.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent="middle_section",
        child="upper_section",
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_HEIGHT - MIDDLE_EMBED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=UPPER_EMBED - UPPER_MIN_INSERT,
            effort=45.0,
            velocity=0.45,
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
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("middle_section", "base_section", axes="xy", max_dist=0.002)
    ctx.expect_origin_distance("upper_section", "middle_section", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("middle_section", "base_section", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("upper_section", "middle_section", axes="xy", min_overlap=0.012)
    ctx.expect_origin_distance("upper_section", "base_section", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("upper_section", "base_section", axes="xy", min_overlap=0.012)
    ctx.expect_joint_motion_axis(
        "base_to_middle",
        "middle_section",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_upper",
        "upper_section",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
