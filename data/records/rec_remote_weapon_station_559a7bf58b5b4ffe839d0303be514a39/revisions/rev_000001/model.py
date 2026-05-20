from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modular_weapon_station")

    model.material("base_paint", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("armor_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("launcher_finish", rgba=(0.34, 0.38, 0.28, 1.0))
    model.material("optic_housing", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("sensor_glass", rgba=(0.10, 0.16, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.84, 0.84, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material="base_paint",
        name="base_plinth",
    )
    base.visual(
        Box((0.52, 0.52, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material="armor_dark",
        name="base_pedestal",
    )
    base.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material="armor_dark",
        name="yaw_bearing",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Box((0.68, 0.62, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material="armor_dark",
        name="stage_deck",
    )
    yaw_stage.visual(
        Box((0.22, 0.28, 0.16)),
        origin=Origin(xyz=(-0.12, 0.0, 0.14)),
        material="armor_dark",
        name="stage_core",
    )
    yaw_stage.visual(
        Box((0.54, 0.08, 0.36)),
        origin=Origin(xyz=(0.08, 0.23, 0.24)),
        material="armor_dark",
        name="left_cheek",
    )
    yaw_stage.visual(
        Box((0.54, 0.08, 0.36)),
        origin=Origin(xyz=(0.08, -0.23, 0.24)),
        material="armor_dark",
        name="right_cheek",
    )
    yaw_stage.visual(
        Box((0.16, 0.10, 0.16)),
        origin=Origin(xyz=(-0.24, 0.23, 0.14)),
        material="armor_dark",
        name="mast_bracket",
    )
    yaw_stage.visual(
        Box((0.18, 0.20, 0.12)),
        origin=Origin(xyz=(-0.22, -0.12, 0.12)),
        material="armor_dark",
        name="rear_counterweight",
    )

    launcher_module = model.part("launcher_module")
    launcher_module.visual(
        Box((0.62, 0.30, 0.03)),
        origin=Origin(xyz=(0.25, 0.0, -0.125)),
        material="launcher_finish",
        name="bottom_shell",
    )
    launcher_module.visual(
        Box((0.62, 0.30, 0.03)),
        origin=Origin(xyz=(0.25, 0.0, 0.085)),
        material="launcher_finish",
        name="top_shell",
    )
    launcher_module.visual(
        Box((0.62, 0.03, 0.18)),
        origin=Origin(xyz=(0.25, 0.135, -0.02)),
        material="launcher_finish",
        name="left_shell",
    )
    launcher_module.visual(
        Box((0.62, 0.03, 0.18)),
        origin=Origin(xyz=(0.25, -0.135, -0.02)),
        material="launcher_finish",
        name="right_shell",
    )
    launcher_module.visual(
        Box((0.03, 0.30, 0.18)),
        origin=Origin(xyz=(-0.045, 0.0, -0.02)),
        material="launcher_finish",
        name="rear_shell",
    )
    launcher_module.visual(
        Cylinder(radius=0.05, length=0.58),
        origin=Origin(xyz=(0.25, 0.078, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material="armor_dark",
        name="upper_left_cell",
    )
    launcher_module.visual(
        Cylinder(radius=0.05, length=0.58),
        origin=Origin(xyz=(0.25, -0.078, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material="armor_dark",
        name="upper_right_cell",
    )
    launcher_module.visual(
        Cylinder(radius=0.05, length=0.58),
        origin=Origin(xyz=(0.25, 0.078, -0.085), rpy=(0.0, pi / 2.0, 0.0)),
        material="armor_dark",
        name="lower_left_cell",
    )
    launcher_module.visual(
        Cylinder(radius=0.05, length=0.58),
        origin=Origin(xyz=(0.25, -0.078, -0.085), rpy=(0.0, pi / 2.0, 0.0)),
        material="armor_dark",
        name="lower_right_cell",
    )
    launcher_module.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(0.02, 0.17, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="launcher_finish",
        name="left_trunnion",
    )
    launcher_module.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(0.02, -0.17, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="launcher_finish",
        name="right_trunnion",
    )

    camera_mast = model.part("camera_mast")
    camera_mast.visual(
        Box((0.05, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material="armor_dark",
        name="mast_post",
    )
    camera_mast.visual(
        Box((0.06, 0.06, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material="optic_housing",
        name="mast_column",
    )
    camera_mast.visual(
        Box((0.15, 0.11, 0.09)),
        origin=Origin(xyz=(0.045, 0.0, 0.38)),
        material="optic_housing",
        name="camera_head",
    )
    camera_mast.visual(
        Cylinder(radius=0.025, length=0.06),
        origin=Origin(xyz=(0.13, 0.0, 0.38), rpy=(0.0, pi / 2.0, 0.0)),
        material="sensor_glass",
        name="camera_lens",
    )

    model.articulation(
        "base_to_yaw_stage",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "yaw_stage_to_launcher",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=launcher_module,
        origin=Origin(xyz=(0.05, 0.0, 0.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=-0.35,
            upper=0.95,
        ),
    )
    model.articulation(
        "yaw_stage_to_camera_mast",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=camera_mast,
        origin=Origin(xyz=(-0.24, 0.23, 0.22)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-1.1,
            upper=0.35,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    yaw_stage = object_model.get_part("yaw_stage")
    launcher_module = object_model.get_part("launcher_module")
    camera_mast = object_model.get_part("camera_mast")

    yaw_joint = object_model.get_articulation("base_to_yaw_stage")
    pitch_joint = object_model.get_articulation("yaw_stage_to_launcher")
    mast_joint = object_model.get_articulation("yaw_stage_to_camera_mast")

    ctx.check(
        "yaw axis is vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "launcher pitch axis is horizontal",
        tuple(pitch_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "camera mast hinge axis is horizontal",
        tuple(mast_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={mast_joint.axis}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, mast_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw stage sits cleanly on the base pedestal",
        )
        ctx.expect_overlap(
            yaw_stage,
            base,
            axes="xy",
            min_overlap=0.50,
            name="yaw stage footprint stays over the square base",
        )
        ctx.expect_contact(
            launcher_module,
            yaw_stage,
            elem_a="left_trunnion",
            elem_b="left_cheek",
            name="left launcher trunnion bears on the left cheek",
        )
        ctx.expect_contact(
            launcher_module,
            yaw_stage,
            elem_a="right_trunnion",
            elem_b="right_cheek",
            name="right launcher trunnion bears on the right cheek",
        )
        ctx.expect_contact(
            camera_mast,
            yaw_stage,
            elem_a="mast_post",
            elem_b="mast_bracket",
            name="camera mast stands on the rear corner bracket",
        )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, mast_joint: 0.0}):
        yaw_rest = _aabb_center(
            ctx.part_element_world_aabb(launcher_module, elem="upper_left_cell")
        )
    with ctx.pose({yaw_joint: 0.6, pitch_joint: 0.0, mast_joint: 0.0}):
        yaw_turned = _aabb_center(
            ctx.part_element_world_aabb(launcher_module, elem="upper_left_cell")
        )
    ctx.check(
        "yaw stage swings the launcher around the vertical axis",
        yaw_rest is not None
        and yaw_turned is not None
        and yaw_turned[1] > yaw_rest[1] + 0.08
        and abs(yaw_turned[2] - yaw_rest[2]) < 0.03,
        details=f"rest={yaw_rest}, turned={yaw_turned}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, mast_joint: 0.0}):
        pitch_rest = _aabb_center(
            ctx.part_element_world_aabb(launcher_module, elem="upper_left_cell")
        )
    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.45, mast_joint: 0.0}):
        pitch_up = _aabb_center(
            ctx.part_element_world_aabb(launcher_module, elem="upper_left_cell")
        )
    ctx.check(
        "positive pitch raises the launcher pod",
        pitch_rest is not None
        and pitch_up is not None
        and pitch_up[2] > pitch_rest[2] + 0.09,
        details=f"rest={pitch_rest}, pitched={pitch_up}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, mast_joint: 0.0}):
        mast_rest = _aabb_center(ctx.part_element_world_aabb(camera_mast, elem="camera_head"))
    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, mast_joint: -1.05}):
        mast_folded = _aabb_center(
            ctx.part_element_world_aabb(camera_mast, elem="camera_head")
        )
        ctx.expect_gap(
            camera_mast,
            yaw_stage,
            axis="z",
            positive_elem="camera_head",
            negative_elem="stage_deck",
            max_penetration=0.0,
            name="folded camera head still clears the deck",
        )
    ctx.check(
        "camera mast folds down from the rear hinge",
        mast_rest is not None
        and mast_folded is not None
        and mast_folded[2] < mast_rest[2] - 0.14,
        details=f"rest={mast_rest}, folded={mast_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
