from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_WIDTH_Y = 0.38
BASE_HEIGHT_Z = 0.42
BASE_DEPTH_X = 0.06
BASE_SIDE_T = 0.04
BASE_BEAM_T = 0.04

Y_RAIL_X = 0.034
Y_RAIL_LENGTH = 0.30
Y_LOWER_RAIL_Z = 0.13
Y_UPPER_RAIL_Z = 0.21
Y_TRAVEL = 0.06

Y_STAGE_FRAME_DEPTH_X = 0.04
Y_STAGE_FRAME_WIDTH_Y = 0.16
Y_STAGE_FRAME_HEIGHT_Z = 0.28
Y_STAGE_FRAME_BASE_Z = 0.03
Y_STAGE_VERTICAL_RAIL_X = 0.016
Y_STAGE_VERTICAL_RAIL_Y = 0.05
Y_STAGE_VERTICAL_RAIL_LENGTH = 0.20

Z_JOINT_X = Y_STAGE_VERTICAL_RAIL_X
Z_JOINT_Y = -Y_STAGE_VERTICAL_RAIL_Y
Z_JOINT_Z = 0.08
Z_TRAVEL = 0.12


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rounded_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    fillet: float,
) -> cq.Workplane:
    shape = _box(size, center)
    if fillet > 0.0:
        shape = shape.edges("|Z").fillet(fillet)
    return shape


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_frame_visual() -> tuple[cq.Workplane, cq.Workplane]:
    outer = _box(
        (BASE_DEPTH_X, BASE_WIDTH_Y, BASE_HEIGHT_Z),
        (0.0, 0.0, BASE_HEIGHT_Z / 2.0),
    )
    inner = _box(
        (
            BASE_DEPTH_X + 0.004,
            BASE_WIDTH_Y - 2.0 * BASE_SIDE_T,
            BASE_HEIGHT_Z - 2.0 * BASE_BEAM_T,
        ),
        (0.0, 0.0, BASE_HEIGHT_Z / 2.0),
    )
    frame = outer.cut(inner)
    frame = frame.union(_box((0.10, 0.09, 0.02), (0.0, -0.11, 0.01)))
    frame = frame.union(_box((0.10, 0.09, 0.02), (0.0, 0.11, 0.01)))

    rails = _box((0.012, Y_RAIL_LENGTH, 0.008), (Y_RAIL_X, 0.0, Y_LOWER_RAIL_Z))
    rails = rails.union(_box((0.012, Y_RAIL_LENGTH, 0.008), (Y_RAIL_X, 0.0, Y_UPPER_RAIL_Z)))
    rails = rails.union(_box((0.016, 0.02, 0.018), (Y_RAIL_X, -0.15, Y_LOWER_RAIL_Z)))
    rails = rails.union(_box((0.016, 0.02, 0.018), (Y_RAIL_X, 0.15, Y_LOWER_RAIL_Z)))
    rails = rails.union(_box((0.016, 0.02, 0.018), (Y_RAIL_X, -0.15, Y_UPPER_RAIL_Z)))
    rails = rails.union(_box((0.016, 0.02, 0.018), (Y_RAIL_X, 0.15, Y_UPPER_RAIL_Z)))
    return frame, rails


def _make_y_stage_visual() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    bridge = _box((0.016, 0.10, 0.14), (-0.014, 0.0, 0.07))
    saddle = _box((0.028, 0.12, 0.04), (-0.004, 0.0, 0.03))

    frame_outer = _box(
        (
            Y_STAGE_FRAME_DEPTH_X,
            Y_STAGE_FRAME_WIDTH_Y,
            Y_STAGE_FRAME_HEIGHT_Z,
        ),
        (-0.006, 0.0, Y_STAGE_FRAME_BASE_Z + Y_STAGE_FRAME_HEIGHT_Z / 2.0),
    )
    frame_inner = _box(
        (
            Y_STAGE_FRAME_DEPTH_X + 0.004,
            Y_STAGE_FRAME_WIDTH_Y - 0.06,
            Y_STAGE_FRAME_HEIGHT_Z - 0.08,
        ),
        (-0.006, 0.0, Y_STAGE_FRAME_BASE_Z + Y_STAGE_FRAME_HEIGHT_Z / 2.0),
    )
    support_frame = frame_outer.cut(frame_inner).union(bridge).union(saddle)

    guides = _rounded_box((0.028, 0.055, 0.028), (0.0, 0.0, 0.0), 0.003)
    guides = guides.union(_rounded_box((0.028, 0.055, 0.028), (0.0, 0.0, 0.08), 0.003))

    rail_center_z = Y_STAGE_FRAME_BASE_Z + 0.09
    rails = _box(
        (0.010, 0.010, Y_STAGE_VERTICAL_RAIL_LENGTH),
        (Y_STAGE_VERTICAL_RAIL_X, -Y_STAGE_VERTICAL_RAIL_Y, rail_center_z),
    )
    rails = rails.union(
        _box(
            (0.010, 0.010, Y_STAGE_VERTICAL_RAIL_LENGTH),
            (Y_STAGE_VERTICAL_RAIL_X, Y_STAGE_VERTICAL_RAIL_Y, rail_center_z),
        )
    )
    return support_frame, guides, rails


def _make_z_stage_visual() -> tuple[cq.Workplane, cq.Workplane]:
    carriage = _box((0.020, 0.15, 0.09), (0.026, 0.05, -0.035))
    carriage = carriage.union(_box((0.014, 0.11, 0.05), (0.036, 0.05, 0.035)))
    carriage = carriage.union(_box((0.010, 0.08, 0.02), (0.046, 0.05, -0.085)))

    guides = _rounded_box((0.026, 0.024, 0.046), (0.0, 0.0, 0.0), 0.0025)
    guides = guides.union(_rounded_box((0.026, 0.024, 0.046), (0.0, 0.10, 0.0), 0.0025))
    return carriage, guides


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_yz_stage", assets=ASSETS)

    model.material("frame_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("rail_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("carriage_aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    model.material("guide_block", rgba=(0.32, 0.34, 0.37, 1.0))

    base = model.part("base_frame")
    base_frame_visual, base_rails_visual = _make_base_frame_visual()
    _add_visual_mesh(base, base_frame_visual, "base_frame.obj", "frame_dark")
    _add_visual_mesh(base, base_rails_visual, "base_rails.obj", "rail_steel")






    base.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH_X, BASE_WIDTH_Y, BASE_HEIGHT_Z)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT_Z / 2.0)),
    )

    y_stage = model.part("y_stage")
    y_frame_visual, y_guides_visual, y_rails_visual = _make_y_stage_visual()
    _add_visual_mesh(y_stage, y_frame_visual, "y_stage_frame.obj", "carriage_aluminum")
    _add_visual_mesh(y_stage, y_guides_visual, "y_stage_guides.obj", "guide_block")
    _add_visual_mesh(y_stage, y_rails_visual, "y_stage_rails.obj", "rail_steel")








    y_stage.inertial = Inertial.from_geometry(
        Box((0.04, 0.16, 0.28)),
        mass=3.5,
        origin=Origin(xyz=(-0.006, 0.0, 0.17)),
    )

    z_stage = model.part("z_stage")
    z_carriage_visual, z_guides_visual = _make_z_stage_visual()
    _add_visual_mesh(z_stage, z_carriage_visual, "z_stage_carriage.obj", "carriage_aluminum")
    _add_visual_mesh(z_stage, z_guides_visual, "z_stage_guides.obj", "guide_block")





    z_stage.inertial = Inertial.from_geometry(
        Box((0.05, 0.15, 0.10)),
        mass=1.6,
        origin=Origin(xyz=(0.024, 0.05, -0.02)),
    )

    model.articulation(
        "base_to_y_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_stage,
        origin=Origin(xyz=(Y_RAIL_X, 0.0, Y_LOWER_RAIL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=500.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "y_stage_to_z_stage",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(Z_JOINT_X, Z_JOINT_Y, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=350.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.expect_origin_distance("y_stage", "base_frame", axes="xy", max_dist=0.06)
    ctx.expect_aabb_overlap("y_stage", "base_frame", axes="xy", min_overlap=0.025)
    ctx.expect_origin_distance("z_stage", "y_stage", axes="xy", max_dist=0.09)
    ctx.expect_aabb_overlap("z_stage", "y_stage", axes="xy", min_overlap=0.008)
    ctx.expect_origin_distance("z_stage", "base_frame", axes="xy", max_dist=0.10)
    ctx.expect_joint_motion_axis(
        "base_to_y_stage",
        "y_stage",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "y_stage_to_z_stage",
        "z_stage",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
