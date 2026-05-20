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
PITCH_LIMIT = 1.10
ROLL_AXIS_Z = 0.078
HALF_PI = 1.5707963267948966


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_housing_shape() -> cq.Workplane:
    rear_block = (
        cq.Workplane("XY")
        .box(0.060, 0.050, 0.030)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, -0.030))
    )
    spine = (
        cq.Workplane("XY")
        .box(0.034, 0.036, 0.012)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, 0.0, -0.013))
    )
    cheek = cq.Workplane("XY").box(0.030, 0.012, 0.028).edges("|Z").fillet(0.003)
    left_cheek = cheek.translate((0.0, 0.023, -0.004))
    right_cheek = cheek.translate((0.0, -0.023, -0.004))
    boss = cq.Workplane("XZ").circle(0.013).extrude(0.008, both=True)
    left_boss = boss.translate((0.0, 0.027, 0.0))
    right_boss = boss.translate((0.0, -0.027, 0.0))
    bore = cq.Workplane("XZ").circle(0.0086).extrude(0.070, both=True)
    return (
        rear_block.union(spine)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_boss)
        .union(right_boss)
        .cut(bore)
    )


def _make_pitch_link_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XZ").circle(0.0085).extrude(0.052, both=True)
    shoulder = (
        cq.Workplane("XY")
        .box(0.022, 0.018, 0.016)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, 0.008))
    )
    plate = (
        cq.Workplane("XY")
        .box(0.038, 0.014, 0.064)
        .edges("|Z")
        .fillet(0.0035)
        .translate((0.0, 0.0, 0.047))
    )
    lightening_slot = cq.Workplane("XY").box(0.016, 0.020, 0.030).translate((0.0, 0.0, 0.046))
    housing_ring = (
        cq.Workplane("XY")
        .circle(0.024)
        .circle(0.016)
        .extrude(0.022, both=True)
        .translate((0.0, 0.0, ROLL_AXIS_Z))
    )
    side_lug = cq.Workplane("XY").box(0.010, 0.020, 0.016).edges("|Z").fillet(0.002)
    left_lug = side_lug.translate((0.018, 0.0, ROLL_AXIS_Z))
    right_lug = side_lug.translate((-0.018, 0.0, ROLL_AXIS_Z))
    return (
        trunnion.union(shoulder)
        .union(plate)
        .cut(lightening_slot)
        .union(housing_ring)
        .union(left_lug)
        .union(right_lug)
    )


def _make_roll_stage_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.015).extrude(0.024, both=True)
    bearing_flange = (
        cq.Workplane("XY")
        .circle(0.020)
        .circle(0.007)
        .extrude(0.008, both=True)
        .translate((0.0, 0.0, 0.006))
    )
    neck = cq.Workplane("XY").circle(0.0105).extrude(0.020, both=True).translate((0.0, 0.0, 0.022))
    tool_plate = (
        cq.Workplane("XY")
        .box(0.056, 0.034, 0.022)
        .edges("|Z")
        .fillet(0.0035)
        .translate((0.028, 0.0, 0.036))
    )
    plate_slot = cq.Workplane("XY").box(0.030, 0.010, 0.028).translate((0.028, 0.0, 0.036))
    nose = cq.Workplane("XY").circle(0.010).extrude(0.010, both=True).translate((0.052, 0.0, 0.050))
    return shaft.union(bearing_flange).union(neck).union(tool_plate).cut(plate_slot).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_wrist_module", assets=ASSETS)

    model.material("housing_gray", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("plate_blue", rgba=(0.25, 0.40, 0.70, 1.0))
    model.material("rotor_dark", rgba=(0.18, 0.20, 0.24, 1.0))

    base_housing = model.part("base_housing")
    _add_visual_mesh(base_housing, _make_base_housing_shape(), "base_housing.obj", "housing_gray")



    base_housing.inertial = Inertial.from_geometry(
        Box((0.062, 0.054, 0.040)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
    )

    pitch_link = model.part("pitch_link")
    _add_visual_mesh(pitch_link, _make_pitch_link_shape(), "pitch_link.obj", "plate_blue")





    pitch_link.inertial = Inertial.from_geometry(
        Box((0.048, 0.040, 0.090)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    roll_stage = model.part("roll_stage")
    _add_visual_mesh(roll_stage, _make_roll_stage_shape(), "roll_stage.obj", "rotor_dark")




    roll_stage.inertial = Inertial.from_geometry(
        Box((0.058, 0.036, 0.052)),
        mass=0.27,
        origin=Origin(xyz=(0.022, 0.0, 0.030)),
    )

    model.articulation(
        "pitch_hinge",
        ArticulationType.REVOLUTE,
        parent=base_housing,
        child=pitch_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-PITCH_LIMIT, upper=PITCH_LIMIT, effort=12.0, velocity=4.0
        ),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.CONTINUOUS,
        parent=pitch_link,
        child=roll_stage,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "pitch_hinge", "pitch_link", world_axis="x", direction="positive", min_delta=0.05
    )
    ctx.expect_joint_motion_axis(
        "roll_joint", "roll_stage", world_axis="y", direction="positive", min_delta=0.012
    )

    ctx.expect_origin_distance("pitch_link", "base_housing", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("pitch_link", "base_housing", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("pitch_link", "base_housing", axis="z", max_gap=0.020, max_penetration=0.020)

    ctx.expect_origin_distance("roll_stage", "pitch_link", axes="xy", max_dist=0.035)
    ctx.expect_aabb_overlap("roll_stage", "pitch_link", axes="xy", min_overlap=0.020)
    ctx.expect_origin_gap("roll_stage", "base_housing", axis="z", min_gap=0.040)

    with ctx.pose(pitch_hinge=0.9):
        ctx.expect_origin_gap("roll_stage", "base_housing", axis="z", min_gap=0.045)
        ctx.expect_origin_distance("roll_stage", "pitch_link", axes="xy", max_dist=0.075)

    with ctx.pose(pitch_hinge=-0.9):
        ctx.expect_aabb_overlap("pitch_link", "base_housing", axes="xy", min_overlap=0.015)
        ctx.expect_aabb_gap("pitch_link", "base_housing", axis="z", max_gap=0.030, max_penetration=0.022)

    with ctx.pose(roll_joint=1.57):
        ctx.expect_origin_distance("roll_stage", "pitch_link", axes="xy", max_dist=0.035)
        ctx.expect_aabb_overlap("roll_stage", "pitch_link", axes="xy", min_overlap=0.018)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
