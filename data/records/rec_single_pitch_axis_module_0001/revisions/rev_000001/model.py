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
# Prefer `model = ArticulatedObject(..., assets=ASSETS)`.
# Use `add_cadquery_visual(...)` for mesh + conservative proxy in one step.
# Use `mesh_from_cadquery(..., "part.obj", assets=ASSETS)` for manual control.
# `TestContext(object_model)` will infer asset_root from model.assets.
SHAFT_Z = 0.045
SHAFT_RADIUS = 0.009
CHEEK_X = 0.045
CHEEK_THICKNESS = 0.012
BRACKET_BOSS_X = 0.032
PITCH_LOWER = -1.0
PITCH_UPPER = 1.2


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center[0] - (length / 2.0), center[1], center[2]))
    )


def _build_base_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(0.090, 0.070, 0.040).edges("|Z").fillet(0.004)
    cover = (
        cq.Workplane("XY")
        .box(0.056, 0.044, 0.014)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, -0.006, 0.024))
    )
    foot = cq.Workplane("XY").box(0.112, 0.028, 0.008).translate((0.0, 0.0, -0.024))

    cheek_core = cq.Workplane("XY").box(CHEEK_THICKNESS, 0.050, 0.025).translate((0.0, 0.0, 0.0325))
    cheek_cap = _x_cylinder(0.019, CHEEK_THICKNESS, (0.0, 0.0, SHAFT_Z))
    cheek = cheek_core.union(cheek_cap).cut(
        _x_cylinder(SHAFT_RADIUS + 0.0012, CHEEK_THICKNESS + 0.002, (0.0, 0.0, SHAFT_Z))
    )

    left_cheek = cheek.translate((-CHEEK_X, 0.0, 0.0))
    right_cheek = cheek.translate((CHEEK_X, 0.0, 0.0))
    shaft = _x_cylinder(SHAFT_RADIUS, 0.108, (0.0, 0.0, SHAFT_Z))

    return housing.union(cover).union(foot).union(left_cheek).union(right_cheek).union(shaft)


def _build_payload_bracket_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.088, 0.010, 0.065)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.0, 0.066, -0.010))
    )
    bridge = cq.Workplane("XY").box(0.032, 0.038, 0.016).translate((0.0, 0.044, -0.002))
    top_tie = cq.Workplane("XY").box(0.070, 0.008, 0.012).translate((0.0, 0.018, 0.008))

    bracket = plate.union(bridge).union(top_tie)
    for boss_x in (-BRACKET_BOSS_X, BRACKET_BOSS_X):
        arm = cq.Workplane("XY").box(0.010, 0.055, 0.018).translate((boss_x, 0.032, -0.006))
        boss = _x_cylinder(0.014, 0.012, (boss_x, 0.0, 0.0))
        bracket = bracket.union(arm).union(boss)
        bracket = bracket.cut(_x_cylinder(SHAFT_RADIUS + 0.0015, 0.014, (boss_x, 0.0, 0.0)))

    for slot_x in (-0.022, 0.022):
        bracket = bracket.cut(
            cq.Workplane("XY").box(0.012, 0.014, 0.028).translate((slot_x, 0.066, -0.010))
        )

    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_axis_actuator_module", assets=ASSETS)
    model.material("cast_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("powdercoat_dark", rgba=(0.20, 0.21, 0.24, 1.0))

    actuator_base = model.part("actuator_base")
    actuator_base.visual(
        mesh_from_cadquery(_build_base_shape(), "actuator_base.obj", assets=ASSETS),
        material="cast_aluminum",
    )




    actuator_base.inertial = Inertial.from_geometry(Box((0.112, 0.080, 0.060)), mass=1.8)

    payload_bracket = model.part("payload_bracket")
    payload_bracket.visual(
        mesh_from_cadquery(_build_payload_bracket_shape(), "payload_bracket.obj", assets=ASSETS),
        material="powdercoat_dark",
    )





    payload_bracket.inertial = Inertial.from_geometry(
        Box((0.090, 0.080, 0.080)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.038, -0.006)),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=actuator_base,
        child=payload_bracket,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=18.0,
            velocity=2.5,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("payload_bracket", "actuator_base", axes="xy", max_dist=0.01)
    ctx.expect_origin_gap("payload_bracket", "actuator_base", axis="z", min_gap=0.035)
    ctx.expect_aabb_overlap("payload_bracket", "actuator_base", axes="xy", min_overlap=0.02)
    ctx.expect_joint_motion_axis(
        "pitch_joint",
        "payload_bracket",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )

    with ctx.pose(pitch_joint=0.0):
        ctx.expect_aabb_overlap("payload_bracket", "actuator_base", axes="xy", min_overlap=0.02)
        ctx.expect_origin_distance("payload_bracket", "actuator_base", axes="xy", max_dist=0.01)

    with ctx.pose(pitch_joint=PITCH_UPPER):
        ctx.expect_aabb_overlap("payload_bracket", "actuator_base", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("payload_bracket", "actuator_base", axes="xy", max_dist=0.01)

    with ctx.pose(pitch_joint=PITCH_LOWER):
        ctx.expect_aabb_overlap("payload_bracket", "actuator_base", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("payload_bracket", "actuator_base", axes="xy", max_dist=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
