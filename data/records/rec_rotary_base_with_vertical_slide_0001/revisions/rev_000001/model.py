from __future__ import annotations

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
BASE_HEIGHT = 0.130
TURNTABLE_GAP = 0.001
TURNTABLE_RADIUS = 0.120
TURNTABLE_THICKNESS = 0.022
ELEVATOR_ORIGIN = (0.0, 0.029, 0.120)
ELEVATOR_TRAVEL = 0.160


def _make_base_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.185).extrude(0.016)
    body = cq.Workplane("XY").circle(0.155).extrude(0.104).translate((0.0, 0.0, 0.016))
    top_rim = cq.Workplane("XY").circle(0.125).extrude(0.010).translate((0.0, 0.0, 0.120))
    pocket = cq.Workplane("XY").circle(0.100).extrude(0.014).translate((0.0, 0.0, 0.116))
    return flange.union(body).union(top_rim).cut(pocket)


def _make_stage_shape() -> cq.Workplane:
    platter = (
        cq.Workplane("XY")
        .circle(TURNTABLE_RADIUS)
        .extrude(TURNTABLE_THICKNESS)
        .translate((0.0, 0.0, TURNTABLE_GAP))
    )
    column = cq.Workplane("XY").box(0.150, 0.040, 0.320).translate((0.0, 0.002, 0.183))
    left_rail = cq.Workplane("XY").box(0.015, 0.010, 0.260).translate((-0.045, 0.035, 0.205))
    right_rail = cq.Workplane("XY").box(0.015, 0.010, 0.260).translate((0.045, 0.035, 0.205))
    top_bridge = cq.Workplane("XY").box(0.170, 0.050, 0.016).translate((0.0, 0.010, 0.351))
    return platter.union(column).union(left_rail).union(right_rail).union(top_bridge)


def _make_guide_block(x_pos: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(0.036, 0.030, 0.080).translate((x_pos, 0.015, 0.0))
    slot = cq.Workplane("XY").box(0.019, 0.022, 0.090).translate((x_pos, 0.008, 0.0))
    return block.cut(slot)


def _make_carriage_shape() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(0.132, 0.016, 0.130).translate((0.0, 0.026, 0.0))
    left_block = _make_guide_block(-0.045)
    right_block = _make_guide_block(0.045)
    platform = cq.Workplane("XY").box(0.180, 0.090, 0.018).translate((0.0, 0.065, 0.085))
    front_lip = cq.Workplane("XY").box(0.120, 0.012, 0.030).translate((0.0, 0.104, 0.063))
    mount_pad = cq.Workplane("XY").circle(0.034).extrude(0.012).translate((0.0, 0.065, 0.094))
    return (
        back_plate.union(left_block)
        .union(right_block)
        .union(platform)
        .union(front_lip)
        .union(mount_pad)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_pedestal_elevator", assets=ASSETS)

    model.material("housing_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("frame_light", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("carriage_gray", rgba=(0.62, 0.66, 0.70, 1.0))

    pedestal = model.part("pedestal_base")
    pedestal.visual(
        mesh_from_cadquery(_make_base_shape(), "pedestal_base.obj", assets=ASSETS),
        material="housing_dark",
    )



    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=BASE_HEIGHT),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    stage = model.part("turntable_stage")
    stage.visual(
        mesh_from_cadquery(_make_stage_shape(), "turntable_stage.obj", assets=ASSETS),
        material="frame_light",
    )


    stage.inertial = Inertial.from_geometry(
        Box((0.240, 0.120, 0.360)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.010, 0.180)),
    )

    carriage = model.part("top_carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "top_carriage.obj", assets=ASSETS),
        material="carriage_gray",
    )




    carriage.inertial = Inertial.from_geometry(
        Box((0.180, 0.110, 0.150)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.050, 0.040)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.5708, upper=1.5708, effort=40.0, velocity=1.5),
    )
    model.articulation(
        "elevator_z",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=carriage,
        origin=Origin(xyz=ELEVATOR_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=ELEVATOR_TRAVEL, effort=25.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("turntable_stage", "pedestal_base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("turntable_stage", "pedestal_base", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_gap("turntable_stage", "pedestal_base", axis="z", max_gap=0.003, max_penetration=0.0)

    ctx.expect_origin_gap("top_carriage", "pedestal_base", axis="z", min_gap=0.045)
    ctx.expect_aabb_overlap("top_carriage", "pedestal_base", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("top_carriage", "turntable_stage", axes="xy", min_overlap=0.08)

    ctx.expect_joint_motion_axis(
        "pedestal_yaw",
        "top_carriage",
        world_axis="x",
        direction="negative",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "elevator_z",
        "top_carriage",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
