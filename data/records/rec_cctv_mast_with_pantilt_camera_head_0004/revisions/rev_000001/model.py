from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSET_ROOT = Path("/tmp/telescoping_cctv_mast_assets")
ASSET_ROOT.mkdir(parents=True, exist_ok=True)
try:
    os.chdir(ASSET_ROOT)
except OSError:
    os.chdir("/tmp")
ASSETS = AssetContext(ASSET_ROOT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_cctv_mast", assets=ASSETS)

    ballast_dark = model.material("ballast_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    camera_black = model.material("camera_black", rgba=(0.10, 0.11, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.20, 0.28, 0.34, 0.55))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.46, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=ballast_dark,
        name="ballast_body",
    )
    base.visual(
        Box((0.36, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=painted_steel,
        name="top_deck",
    )
    base.visual(
        Box((0.22, 0.12, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=painted_steel,
        name="mast_pad",
    )
    for foot_index, (x, y) in enumerate(
        ((0.24, 0.17), (0.24, -0.17), (-0.24, 0.17), (-0.24, -0.17)),
        start=1,
    ):
        base.visual(
            Cylinder(radius=0.03, length=0.014),
            origin=Origin(xyz=(x, y, 0.007)),
            material=painted_steel,
            name=f"foot_{foot_index}",
        )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.068, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=painted_steel,
        name="mount_flange",
    )
    lower_stage.visual(
        Cylinder(radius=0.052, length=1.78),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=steel,
        name="main_tube",
    )
    lower_stage.visual(
        Cylinder(radius=0.058, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        material=painted_steel,
        name="guide_collar",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.038, length=1.65),
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        material=steel,
        name="inner_pole",
    )
    upper_stage.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 1.636)),
        material=painted_steel,
        name="top_mount",
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_steel,
        name="pedestal",
    )
    pan_stage.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(-0.018, 0.048, 0.20)),
        material=painted_steel,
        name="yoke_arm_pos",
    )
    pan_stage.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(-0.018, -0.048, 0.20)),
        material=painted_steel,
        name="yoke_arm_neg",
    )
    pan_stage.visual(
        Box((0.04, 0.012, 0.14)),
        origin=Origin(xyz=(-0.04, 0.048, 0.15)),
        material=painted_steel,
        name="yoke_strut_pos",
    )
    pan_stage.visual(
        Box((0.04, 0.012, 0.14)),
        origin=Origin(xyz=(-0.04, -0.048, 0.15)),
        material=painted_steel,
        name="yoke_strut_neg",
    )
    pan_stage.visual(
        Box((0.08, 0.108, 0.04)),
        origin=Origin(xyz=(-0.06, 0.0, 0.10)),
        material=painted_steel,
        name="yoke_bridge",
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Box((0.15, 0.062, 0.072)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=camera_black,
        name="camera_shell",
    )
    camera_head.visual(
        Box((0.10, 0.074, 0.014)),
        origin=Origin(xyz=(0.09, 0.0, 0.043)),
        material=camera_black,
        name="sunshield",
    )
    camera_head.visual(
        Box((0.04, 0.05, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, 0.0)),
        material=camera_black,
        name="rear_service_box",
    )
    camera_head.visual(
        Cylinder(radius=0.021, length=0.05),
        origin=Origin(xyz=(0.15, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_barrel",
    )
    camera_head.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="trunnion_pos",
    )
    camera_head.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="trunnion_neg",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.FIXED,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.65),
    )
    model.articulation(
        "upper_to_pan",
        ArticulationType.REVOLUTE,
        parent=upper_stage,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.65)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(65.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    pan_stage = object_model.get_part("pan_stage")
    camera_head = object_model.get_part("camera_head")

    mast_extension = object_model.get_articulation("lower_to_upper")
    head_pan = object_model.get_articulation("upper_to_pan")
    head_tilt = object_model.get_articulation("pan_to_camera")

    top_deck = base.get_visual("top_deck")
    mast_pad = base.get_visual("mast_pad")
    mount_flange = lower_stage.get_visual("mount_flange")
    main_tube = lower_stage.get_visual("main_tube")
    guide_collar = lower_stage.get_visual("guide_collar")
    inner_pole = upper_stage.get_visual("inner_pole")
    top_mount = upper_stage.get_visual("top_mount")
    pedestal = pan_stage.get_visual("pedestal")
    yoke_arm_pos = pan_stage.get_visual("yoke_arm_pos")
    yoke_arm_neg = pan_stage.get_visual("yoke_arm_neg")
    camera_shell = camera_head.get_visual("camera_shell")
    lens_barrel = camera_head.get_visual("lens_barrel")
    trunnion_pos = camera_head.get_visual("trunnion_pos")
    trunnion_neg = camera_head.get_visual("trunnion_neg")

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        lower_stage,
        upper_stage,
        reason="The upper mast is represented as a solid telescoping pole sliding inside the lower tube housing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "mast_extension_axis_vertical",
        tuple(mast_extension.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical prismatic mast axis, got {mast_extension.axis!r}",
    )
    ctx.check(
        "pan_axis_vertical",
        tuple(head_pan.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical pan axis, got {head_pan.axis!r}",
    )
    ctx.check(
        "tilt_axis_lateral",
        tuple(head_tilt.axis) == (0.0, 1.0, 0.0),
        f"Expected lateral tilt axis, got {head_tilt.axis!r}",
    )

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mount_flange,
        negative_elem=mast_pad,
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        elem_a=mount_flange,
        elem_b=mast_pad,
        min_overlap=0.12,
    )
    ctx.expect_within(
        lower_stage,
        base,
        axes="xy",
        inner_elem=mount_flange,
        outer_elem=top_deck,
    )
    ctx.expect_within(
        upper_stage,
        lower_stage,
        axes="xy",
        inner_elem=inner_pole,
        outer_elem=main_tube,
    )
    ctx.expect_gap(
        pan_stage,
        upper_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=pedestal,
        negative_elem=top_mount,
    )
    ctx.expect_contact(camera_head, pan_stage, elem_a=trunnion_pos, elem_b=yoke_arm_pos)
    ctx.expect_contact(camera_head, pan_stage, elem_a=trunnion_neg, elem_b=yoke_arm_neg)
    ctx.expect_within(camera_head, pan_stage, axes="y", inner_elem=camera_shell)

    rest_head_pos = ctx.part_world_position(camera_head)
    rest_lens_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem=lens_barrel))

    with ctx.pose({mast_extension: mast_extension.motion_limits.upper}):
        ctx.fail_if_isolated_parts(name="mast_extended_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_extended_no_overlap")
        ctx.expect_within(
            upper_stage,
            lower_stage,
            axes="xy",
            inner_elem=inner_pole,
            outer_elem=guide_collar,
        )
        ctx.expect_gap(
            camera_head,
            base,
            axis="z",
            min_gap=2.1,
            positive_elem=camera_shell,
            negative_elem=top_deck,
        )
        extended_head_pos = ctx.part_world_position(camera_head)
        ctx.check(
            "mast_extension_lifts_camera",
            rest_head_pos is not None
            and extended_head_pos is not None
            and extended_head_pos[2] - rest_head_pos[2] > 0.60,
            "Camera head should rise by more than 0.60 m at full mast extension.",
        )

    with ctx.pose({head_pan: math.radians(90.0)}):
        ctx.fail_if_isolated_parts(name="pan_90_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_90_no_overlap")
        pan_lens_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem=lens_barrel))
        ctx.check(
            "pan_rotates_camera_sideways",
            rest_lens_center is not None
            and pan_lens_center is not None
            and abs(pan_lens_center[0]) < 0.03
            and pan_lens_center[1] > 0.11,
            "Pan stage should swing the lens sideways around the mast centerline.",
        )

    tilt_limits = head_tilt.motion_limits
    tilt_down_pose = 0.0 if tilt_limits is None or tilt_limits.upper is None else tilt_limits.upper
    tilt_up_pose = 0.0 if tilt_limits is None or tilt_limits.lower is None else tilt_limits.lower

    with ctx.pose({head_tilt: tilt_down_pose}):
        ctx.fail_if_isolated_parts(name="tilt_down_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_down_no_overlap")
        ctx.expect_contact(camera_head, pan_stage, elem_a=trunnion_pos, elem_b=yoke_arm_pos)
        ctx.expect_contact(camera_head, pan_stage, elem_a=trunnion_neg, elem_b=yoke_arm_neg)
        tilt_down_lens_center = aabb_center(ctx.part_element_world_aabb(camera_head, elem=lens_barrel))
        ctx.check(
            "tilt_down_lowers_lens",
            rest_lens_center is not None
            and tilt_down_lens_center is not None
            and tilt_down_lens_center[2] < rest_lens_center[2] - 0.04,
            "Positive tilt limit should pitch the lens downward.",
        )

    with ctx.pose({head_tilt: tilt_up_pose, head_pan: math.radians(90.0)}):
        ctx.fail_if_isolated_parts(name="pan_and_tilt_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_and_tilt_no_overlap")
        ctx.expect_contact(camera_head, pan_stage, elem_a=trunnion_pos, elem_b=yoke_arm_pos)
        ctx.expect_contact(camera_head, pan_stage, elem_a=trunnion_neg, elem_b=yoke_arm_neg)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
