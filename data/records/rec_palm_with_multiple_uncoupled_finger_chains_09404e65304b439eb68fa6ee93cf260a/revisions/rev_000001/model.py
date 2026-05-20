from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _add_blocky_chain(
    part,
    *,
    base_radius: float,
    segment_lengths: tuple[float, float, float],
    segment_widths: tuple[float, float, float],
    thickness: float,
    chain_yaw: float = 0.0,
    pad_depth: float = 0.006,
) -> None:
    """Build one thick, rigid chain silhouette around the part frame at the root pivot."""
    l1, l2, l3 = segment_lengths
    w1, w2, w3 = segment_widths

    part.visual(
        Cylinder(radius=base_radius, length=thickness),
        origin=Origin(rpy=(0.0, 0.0, chain_yaw)),
        name="root_collar",
    )

    x1 = base_radius + l1 * 0.5
    x2_joint = base_radius + l1 - 0.006
    x2 = x2_joint + l2 * 0.5
    x3_joint = x2_joint + l2 - 0.006
    x3 = x3_joint + l3 * 0.5
    total_length = x3 + l3 * 0.5

    part.visual(
        Box((l1, w1, thickness)),
        origin=Origin(xyz=(x1, 0.0, 0.0), rpy=(0.0, 0.0, chain_yaw)),
        name="proximal_block",
    )
    part.visual(
        Cylinder(radius=0.5 * max(w1, w2), length=thickness),
        origin=Origin(xyz=(x2_joint, 0.0, 0.0), rpy=(0.0, 0.0, chain_yaw)),
        name="middle_barrel",
    )
    part.visual(
        Box((l2, w2, thickness * 0.94)),
        origin=Origin(xyz=(x2, 0.0, 0.0), rpy=(0.0, 0.0, chain_yaw)),
        name="middle_block",
    )
    part.visual(
        Cylinder(radius=0.5 * max(w2, w3), length=thickness * 0.96),
        origin=Origin(xyz=(x3_joint, 0.0, 0.0), rpy=(0.0, 0.0, chain_yaw)),
        name="distal_barrel",
    )
    part.visual(
        Box((l3, w3, thickness * 0.88)),
        origin=Origin(xyz=(x3, 0.0, 0.0), rpy=(0.0, 0.0, chain_yaw)),
        name="distal_block",
    )
    part.visual(
        Box((total_length - 0.010, min(w1, w3) * 0.72, pad_depth)),
        origin=Origin(
            xyz=(0.5 * total_length + 0.002, 0.0, -0.5 * thickness + 0.5 * pad_depth),
            rpy=(0.0, 0.0, chain_yaw),
        ),
        name="grip_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_gripper_palm")

    steel = model.material("steel", rgba=(0.24, 0.25, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    pad_black = model.material("pad_black", rgba=(0.08, 0.08, 0.09, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.170, 0.220, 0.085)),
        origin=Origin(xyz=(0.085, 0.0, 0.0425)),
        material=steel,
        name="palm_body",
    )
    palm.visual(
        Box((0.118, 0.178, 0.024)),
        origin=Origin(xyz=(0.074, 0.0, 0.073)),
        material=dark_steel,
        name="top_plate",
    )
    palm.visual(
        Box((0.112, 0.040, 0.058)),
        origin=Origin(xyz=(0.112, -0.129, 0.050)),
        material=steel,
        name="thumb_cheek",
    )

    palm.visual(
        Box((0.015, 0.042, 0.050)),
        origin=Origin(xyz=(0.1775, -0.070, 0.046)),
        material=dark_steel,
        name="front_ear_left",
    )
    palm.visual(
        Box((0.015, 0.042, 0.050)),
        origin=Origin(xyz=(0.1775, 0.000, 0.046)),
        material=dark_steel,
        name="front_ear_center",
    )
    palm.visual(
        Box((0.015, 0.042, 0.050)),
        origin=Origin(xyz=(0.1775, 0.070, 0.046)),
        material=dark_steel,
        name="front_ear_right",
    )

    palm.visual(
        Box((0.022, 0.032, 0.050)),
        origin=Origin(xyz=(0.136, -0.148, 0.053)),
        material=dark_steel,
        name="thumb_ear",
    )

    finger_left = model.part("finger_left")
    _add_blocky_chain(
        finger_left,
        base_radius=0.016,
        segment_lengths=(0.070, 0.062, 0.056),
        segment_widths=(0.038, 0.035, 0.032),
        thickness=0.030,
    )
    for visual_name in ("root_collar", "middle_barrel", "distal_barrel"):
        finger_left.get_visual(visual_name).material = dark_steel
    for visual_name in ("proximal_block", "middle_block", "distal_block"):
        finger_left.get_visual(visual_name).material = steel
    finger_left.get_visual("grip_pad").material = pad_black

    finger_center = model.part("finger_center")
    _add_blocky_chain(
        finger_center,
        base_radius=0.016,
        segment_lengths=(0.074, 0.066, 0.060),
        segment_widths=(0.039, 0.036, 0.033),
        thickness=0.031,
    )
    for visual_name in ("root_collar", "middle_barrel", "distal_barrel"):
        finger_center.get_visual(visual_name).material = dark_steel
    for visual_name in ("proximal_block", "middle_block", "distal_block"):
        finger_center.get_visual(visual_name).material = steel
    finger_center.get_visual("grip_pad").material = pad_black

    finger_right = model.part("finger_right")
    _add_blocky_chain(
        finger_right,
        base_radius=0.016,
        segment_lengths=(0.070, 0.062, 0.056),
        segment_widths=(0.038, 0.035, 0.032),
        thickness=0.030,
    )
    for visual_name in ("root_collar", "middle_barrel", "distal_barrel"):
        finger_right.get_visual(visual_name).material = dark_steel
    for visual_name in ("proximal_block", "middle_block", "distal_block"):
        finger_right.get_visual(visual_name).material = steel
    finger_right.get_visual("grip_pad").material = pad_black

    thumb = model.part("thumb")
    _add_blocky_chain(
        thumb,
        base_radius=0.016,
        segment_lengths=(0.056, 0.050, 0.046),
        segment_widths=(0.042, 0.038, 0.034),
        thickness=0.031,
        chain_yaw=0.55,
        pad_depth=0.007,
    )
    for visual_name in ("root_collar", "middle_barrel", "distal_barrel"):
        thumb.get_visual(visual_name).material = dark_steel
    for visual_name in ("proximal_block", "middle_block", "distal_block"):
        thumb.get_visual(visual_name).material = steel
    thumb.get_visual("grip_pad").material = pad_black

    model.articulation(
        "palm_to_finger_left",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=finger_left,
        origin=Origin(xyz=(0.201, -0.070, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.80,
        ),
    )
    model.articulation(
        "palm_to_finger_center",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=finger_center,
        origin=Origin(xyz=(0.201, 0.000, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.5,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "palm_to_finger_right",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=finger_right,
        origin=Origin(xyz=(0.201, 0.070, 0.046)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.80,
        ),
    )
    model.articulation(
        "palm_to_thumb",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb,
        origin=Origin(xyz=(0.136, -0.180, 0.053)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=85.0,
            velocity=1.3,
            lower=-0.35,
            upper=0.85,
        ),
    )

    return model


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

    palm = object_model.get_part("palm")
    finger_left = object_model.get_part("finger_left")
    finger_center = object_model.get_part("finger_center")
    finger_right = object_model.get_part("finger_right")
    thumb = object_model.get_part("thumb")

    joint_left = object_model.get_articulation("palm_to_finger_left")
    joint_center = object_model.get_articulation("palm_to_finger_center")
    joint_right = object_model.get_articulation("palm_to_finger_right")
    joint_thumb = object_model.get_articulation("palm_to_thumb")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    for joint_name, joint_obj in (
        ("left finger", joint_left),
        ("center finger", joint_center),
        ("right finger", joint_right),
        ("thumb", joint_thumb),
    ):
        axis = tuple(joint_obj.axis)
        ctx.check(
            f"{joint_name} root uses a vertical revolute hinge",
            joint_obj.articulation_type == ArticulationType.REVOLUTE
            and abs(axis[0]) < 1e-9
            and abs(axis[1]) < 1e-9
            and abs(abs(axis[2]) - 1.0) < 1e-9,
            details=f"type={joint_obj.articulation_type}, axis={axis}",
        )

    ctx.expect_gap(
        finger_left,
        palm,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="root_collar",
        negative_elem="front_ear_left",
        name="left finger root collar seats against left front ear",
    )
    ctx.expect_gap(
        finger_center,
        palm,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="root_collar",
        negative_elem="front_ear_center",
        name="center finger root collar seats against center front ear",
    )
    ctx.expect_gap(
        finger_right,
        palm,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="root_collar",
        negative_elem="front_ear_right",
        name="right finger root collar seats against right front ear",
    )
    ctx.expect_gap(
        palm,
        thumb,
        axis="y",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="thumb_ear",
        negative_elem="root_collar",
        name="thumb root collar seats against side thumb ear",
    )

    ctx.expect_origin_gap(
        finger_left,
        palm,
        axis="x",
        min_gap=0.19,
        max_gap=0.21,
        name="left finger root is mounted out on the front face",
    )
    ctx.expect_origin_gap(
        finger_center,
        finger_left,
        axis="y",
        min_gap=0.06,
        max_gap=0.08,
        name="center finger root is spaced from the left finger root",
    )
    ctx.expect_origin_gap(
        finger_right,
        finger_center,
        axis="y",
        min_gap=0.06,
        max_gap=0.08,
        name="right finger root is spaced from the center finger root",
    )
    ctx.expect_origin_gap(
        palm,
        thumb,
        axis="y",
        min_gap=0.17,
        max_gap=0.19,
        name="thumb root is mounted off the side cheek",
    )

    left_rest = _center_from_aabb(ctx.part_element_world_aabb(finger_left, elem="distal_block"))
    right_rest = _center_from_aabb(ctx.part_element_world_aabb(finger_right, elem="distal_block"))
    thumb_rest = _center_from_aabb(ctx.part_element_world_aabb(thumb, elem="distal_block"))

    with ctx.pose({joint_left: 0.55}):
        left_turned = _center_from_aabb(ctx.part_element_world_aabb(finger_left, elem="distal_block"))
    with ctx.pose({joint_right: 0.55}):
        right_turned = _center_from_aabb(ctx.part_element_world_aabb(finger_right, elem="distal_block"))
    with ctx.pose({joint_thumb: 0.55}):
        thumb_turned = _center_from_aabb(ctx.part_element_world_aabb(thumb, elem="distal_block"))

    ctx.check(
        "left finger swings inward from its own root hinge",
        left_rest is not None
        and left_turned is not None
        and left_turned[1] > left_rest[1] + 0.045,
        details=f"rest={left_rest}, turned={left_turned}",
    )
    ctx.check(
        "right finger swings inward from its own root hinge",
        right_rest is not None
        and right_turned is not None
        and right_turned[1] < right_rest[1] - 0.045,
        details=f"rest={right_rest}, turned={right_turned}",
    )
    ctx.check(
        "thumb swings inward from its side hinge",
        thumb_rest is not None
        and thumb_turned is not None
        and thumb_turned[1] > thumb_rest[1] + 0.040,
        details=f"rest={thumb_rest}, turned={thumb_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
