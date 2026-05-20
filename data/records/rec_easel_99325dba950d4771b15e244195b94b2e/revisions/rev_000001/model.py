from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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


def _beam_origin(
    top: tuple[float, float, float],
    foot: tuple[float, float, float],
    *,
    plane: str,
) -> tuple[Origin, float]:
    cx = 0.5 * (top[0] + foot[0])
    cy = 0.5 * (top[1] + foot[1])
    cz = 0.5 * (top[2] + foot[2])
    dx = foot[0] - top[0]
    dy = foot[1] - top[1]
    dz = foot[2] - top[2]
    length = (dx * dx + dy * dy + dz * dz) ** 0.5
    if plane == "xz":
        angle = atan2(abs(dx), abs(dz))
        signed = angle if dx < 0.0 else -angle
        return Origin(xyz=(cx, cy, cz), rpy=(0.0, signed, 0.0)), length
    if plane == "yz":
        angle = atan2(abs(dy), abs(dz))
        signed = -angle if dy < 0.0 else angle
        return Origin(xyz=(cx, cy, cz), rpy=(signed, 0.0, 0.0)), length
    raise ValueError(f"Unsupported plane {plane!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_tripod_easel")

    wood = model.material("beech_wood", color=(0.78, 0.64, 0.41, 1.0))
    steel = model.material("burnished_steel", color=(0.35, 0.35, 0.37, 1.0))
    rubber = model.material("rubber_black", color=(0.10, 0.10, 0.10, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.16, 0.055, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=wood,
        name="head_block",
    )
    front_frame.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(-0.097, -0.0415, -0.02), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_hinge_lug_parent",
    )
    front_frame.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.094, -0.0425, -0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rear_hinge_lug_parent",
    )
    front_frame.visual(
        Box((0.03, 0.03, 0.06)),
        origin=Origin(xyz=(0.065, -0.0425, -0.045)),
        material=wood,
        name="rear_hinge_cheek",
    )
    front_frame.visual(
        Box((0.055, 0.022, 1.38)),
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        material=wood,
        name="guide_board",
    )

    right_leg_top = (0.055, 0.0, -0.06)
    right_leg_foot = (0.34, 0.0, -1.58)
    right_leg_origin, right_leg_length = _beam_origin(right_leg_top, right_leg_foot, plane="xz")
    front_frame.visual(
        Box((0.05, 0.032, right_leg_length)),
        origin=right_leg_origin,
        material=wood,
        name="right_front_leg_beam",
    )
    front_frame.visual(
        Box((0.082, 0.042, 0.03)),
        origin=Origin(xyz=(0.34, 0.0, -1.589)),
        material=rubber,
        name="right_front_foot",
    )

    left_front_leg = model.part("left_front_leg")
    left_front_leg.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.0, -0.0695, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_hinge_lug_child",
    )
    left_front_leg.visual(
        Box((0.074, 0.087, 0.06)),
        origin=Origin(xyz=(-0.025, -0.041, -0.045)),
        material=wood,
        name="left_upper_bracket",
    )
    left_leg_top = (-0.03, 0.0, -0.065)
    left_leg_foot = (-0.26, 0.0, -1.56)
    left_leg_origin, left_leg_length = _beam_origin(left_leg_top, left_leg_foot, plane="xz")
    left_front_leg.visual(
        Box((0.048, 0.03, left_leg_length)),
        origin=left_leg_origin,
        material=wood,
        name="left_front_leg_beam",
    )
    left_front_leg.visual(
        Box((0.082, 0.042, 0.03)),
        origin=Origin(xyz=(-0.26, 0.0, -1.569)),
        material=rubber,
        name="left_front_foot",
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.122, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rear_hinge_lug_child",
    )
    rear_leg.visual(
        Box((0.18, 0.07, 0.06)),
        origin=Origin(xyz=(0.052, -0.0625, -0.045)),
        material=wood,
        name="rear_upper_bracket",
    )
    rear_leg.visual(
        Box((0.05, 0.018, 0.04)),
        origin=Origin(xyz=(0.12, -0.021, -0.02)),
        material=wood,
        name="rear_hinge_bridge",
    )
    rear_leg_top = (0.0, -0.03, -0.065)
    rear_leg_foot = (0.0, -0.77, -1.56)
    rear_leg_origin, rear_leg_length = _beam_origin(rear_leg_top, rear_leg_foot, plane="yz")
    rear_leg.visual(
        Box((0.043, 0.028, rear_leg_length)),
        origin=rear_leg_origin,
        material=wood,
        name="rear_leg_beam",
    )
    rear_leg.visual(
        Box((0.075, 0.042, 0.03)),
        origin=Origin(xyz=(0.0, -0.77, -1.569)),
        material=rubber,
        name="rear_foot",
    )

    rest_support = model.part("rest_support")
    rest_support.visual(
        Box((0.016, 0.045, 0.11)),
        origin=Origin(xyz=(-0.0355, 0.0, 0.0)),
        material=wood,
        name="rest_left_shoe",
    )
    rest_support.visual(
        Box((0.016, 0.045, 0.11)),
        origin=Origin(xyz=(0.0355, 0.0, 0.0)),
        material=wood,
        name="rest_right_shoe",
    )
    rest_support.visual(
        Box((0.103, 0.014, 0.07)),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=wood,
        name="rest_front_plate",
    )
    rest_support.visual(
        Box((0.10, 0.024, 0.12)),
        origin=Origin(xyz=(0.0, 0.034, -0.055)),
        material=wood,
        name="rest_stem",
    )
    rest_support.visual(
        Box((0.56, 0.052, 0.022)),
        origin=Origin(xyz=(0.0, 0.066, -0.095)),
        material=wood,
        name="rest_tray",
    )
    rest_support.visual(
        Box((0.56, 0.016, 0.03)),
        origin=Origin(xyz=(0.0, 0.097, -0.089)),
        material=wood,
        name="rest_lip",
    )
    rest_support.visual(
        Cylinder(radius=0.012, length=0.06),
        origin=Origin(xyz=(0.0, 0.028, 0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rest_lock_knob",
    )

    top_clip = model.part("top_clip")
    top_clip.visual(
        Box((0.014, 0.04, 0.085)),
        origin=Origin(xyz=(-0.0345, 0.0, 0.0)),
        material=wood,
        name="clip_left_shoe",
    )
    top_clip.visual(
        Box((0.014, 0.04, 0.085)),
        origin=Origin(xyz=(0.0345, 0.0, 0.0)),
        material=wood,
        name="clip_right_shoe",
    )
    top_clip.visual(
        Box((0.09, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=wood,
        name="clip_front_plate",
    )
    top_clip.visual(
        Box((0.22, 0.034, 0.02)),
        origin=Origin(xyz=(0.0, 0.040, 0.02)),
        material=wood,
        name="clip_bar",
    )
    top_clip.visual(
        Box((0.14, 0.018, 0.07)),
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        material=wood,
        name="clip_pad",
    )
    top_clip.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.0, 0.045, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="clip_lock_knob",
    )

    model.articulation(
        "front_frame_to_left_front_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_front_leg,
        origin=Origin(xyz=(-0.097, 0.0, -0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-0.12, upper=0.45),
    )
    model.articulation(
        "front_frame_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.0425, -0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-0.10, upper=0.50),
    )
    model.articulation(
        "front_frame_to_rest_support",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=rest_support,
        origin=Origin(xyz=(0.0, 0.0, -1.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.2, lower=0.0, upper=0.44),
    )
    model.articulation(
        "front_frame_to_top_clip",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=top_clip,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.2, lower=0.0, upper=0.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    left_front_leg = object_model.get_part("left_front_leg")
    rear_leg = object_model.get_part("rear_leg")
    rest_support = object_model.get_part("rest_support")
    top_clip = object_model.get_part("top_clip")

    left_hinge = object_model.get_articulation("front_frame_to_left_front_leg")
    rear_hinge = object_model.get_articulation("front_frame_to_rear_leg")
    rest_slide = object_model.get_articulation("front_frame_to_rest_support")
    clip_slide = object_model.get_articulation("front_frame_to_top_clip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_front_leg,
        front_frame,
        elem_a="left_hinge_lug_child",
        elem_b="left_hinge_lug_parent",
        name="left front leg hinge barrel is mounted to the head block",
    )
    ctx.expect_contact(
        rear_leg,
        front_frame,
        elem_a="rear_hinge_lug_child",
        elem_b="rear_hinge_lug_parent",
        name="rear leg hinge barrel is mounted to the head block",
    )
    ctx.expect_contact(
        rest_support,
        front_frame,
        elem_a="rest_left_shoe",
        elem_b="guide_board",
        name="canvas rest carriage bears on the guide board",
    )
    ctx.expect_contact(
        top_clip,
        front_frame,
        elem_a="clip_left_shoe",
        elem_b="guide_board",
        name="top clip carriage bears on the guide board",
    )
    ctx.expect_gap(
        top_clip,
        rest_support,
        axis="z",
        min_gap=0.35,
        name="top clip starts above the canvas rest",
    )

    open_left_foot_aabb = ctx.part_element_world_aabb(left_front_leg, elem="left_front_foot")
    with ctx.pose({left_hinge: 0.28}):
        ctx.expect_contact(
            left_front_leg,
            front_frame,
            elem_a="left_hinge_lug_child",
            elem_b="left_hinge_lug_parent",
            name="left hinge stays engaged when the easel folds",
        )
        folded_left_foot_aabb = ctx.part_element_world_aabb(left_front_leg, elem="left_front_foot")
    ctx.check(
        "left front leg folds inward around the top hinge",
        open_left_foot_aabb is not None
        and folded_left_foot_aabb is not None
        and folded_left_foot_aabb[0][0] > open_left_foot_aabb[0][0] + 0.35,
        details=f"open={open_left_foot_aabb}, folded={folded_left_foot_aabb}",
    )

    open_rear_foot_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    with ctx.pose({rear_hinge: 0.34}):
        ctx.expect_contact(
            rear_leg,
            front_frame,
            elem_a="rear_hinge_lug_child",
            elem_b="rear_hinge_lug_parent",
            name="rear hinge stays engaged as the back leg folds",
        )
        folded_rear_foot_aabb = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    ctx.check(
        "rear leg swings toward the front frame",
        open_rear_foot_aabb is not None
        and folded_rear_foot_aabb is not None
        and folded_rear_foot_aabb[1][1] > open_rear_foot_aabb[1][1] + 0.45,
        details=f"open={open_rear_foot_aabb}, folded={folded_rear_foot_aabb}",
    )

    rest_pos = ctx.part_world_position(rest_support)
    with ctx.pose({rest_slide: 0.36}):
        ctx.expect_contact(
            rest_support,
            front_frame,
            elem_a="rest_left_shoe",
            elem_b="guide_board",
            name="canvas rest remains guided at a raised position",
        )
        raised_rest_pos = ctx.part_world_position(rest_support)
    ctx.check(
        "canvas rest slides upward on its prismatic mast slot",
        rest_pos is not None and raised_rest_pos is not None and raised_rest_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, raised={raised_rest_pos}",
    )

    clip_pos = ctx.part_world_position(top_clip)
    with ctx.pose({clip_slide: 0.40}):
        ctx.expect_contact(
            top_clip,
            front_frame,
            elem_a="clip_left_shoe",
            elem_b="guide_board",
            name="top clip remains guided at a raised position",
        )
        raised_clip_pos = ctx.part_world_position(top_clip)
        ctx.expect_gap(
            top_clip,
            rest_support,
            axis="z",
            min_gap=0.12,
            name="top clip still clears the canvas rest when both are raised",
        )
    ctx.check(
        "top locking clip slides upward on its second slot",
        clip_pos is not None and raised_clip_pos is not None and raised_clip_pos[2] > clip_pos[2] + 0.34,
        details=f"clip={clip_pos}, raised={raised_clip_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
