from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="modern_led_desk_lamp")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_diffuser = model.material("dark_diffuser", rgba=(0.60, 0.62, 0.64, 1.0))

    base_radius = 0.085
    base_thickness = 0.012
    stem_height = 0.400
    stem_width = 0.014
    stem_depth = 0.010

    collar_height = 0.030
    arm_depth = 0.010
    arm_thickness = 0.014
    pivot_radius = 0.021
    pivot_height = 0.018

    joint_x = 0.055
    joint_z = 0.088
    pad_height = 0.008
    pad_width = 0.020
    pad_depth = 0.012
    pad_center_z = joint_z - (pivot_height * 0.5) - (pad_height * 0.5)

    beam_start = (0.0, 0.0, collar_height - 0.004)
    beam_end = (joint_x, 0.0, pad_center_z - (pad_height * 0.5))
    beam_dx = beam_end[0] - beam_start[0]
    beam_dz = beam_end[2] - beam_start[2]
    beam_length = math.hypot(beam_dx, beam_dz)
    beam_angle = math.atan2(beam_dz, beam_dx)
    beam_center = ((beam_start[0] + beam_end[0]) * 0.5, 0.0, (beam_start[2] + beam_end[2]) * 0.5)

    bar_length = 0.440
    bar_depth = 0.018
    bar_height = 0.010
    end_radius = bar_depth * 0.5
    core_length = bar_length - bar_depth
    diffuser_length = 0.388
    diffuser_depth = 0.008
    diffuser_height = 0.002

    base = model.part("base")
    base.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=matte_black,
        name="base_plate",
    )

    stem = model.part("stem")
    stem.visual(
        Box((stem_width, stem_depth, stem_height)),
        origin=Origin(xyz=(0.0, 0.0, stem_height * 0.5)),
        material=matte_black,
        name="stem_body",
    )

    arm = model.part("elbow_arm")
    arm.visual(
        Box((stem_width * 1.15, stem_depth * 1.1, collar_height)),
        origin=Origin(xyz=(0.0, 0.0, collar_height * 0.5)),
        material=matte_black,
        name="arm_collar",
    )
    arm.visual(
        Box((beam_length, arm_depth, arm_thickness)),
        origin=Origin(xyz=beam_center, rpy=(0.0, -beam_angle, 0.0)),
        material=matte_black,
        name="arm_beam",
    )
    arm.visual(
        Box((pad_width, pad_depth, pad_height)),
        origin=Origin(xyz=(joint_x, 0.0, pad_center_z)),
        material=matte_black,
        name="pivot_pad",
    )

    head = model.part("light_head")
    head.visual(
        Cylinder(radius=pivot_radius, length=pivot_height),
        origin=Origin(),
        material=matte_black,
        name="pivot_hub",
    )
    head.visual(
        Box((core_length, bar_depth, bar_height)),
        origin=Origin(),
        material=matte_black,
        name="light_bar_core",
    )
    head.visual(
        Cylinder(radius=end_radius, length=bar_height),
        origin=Origin(xyz=(core_length * 0.5, 0.0, 0.0)),
        material=matte_black,
        name="light_bar_end_left",
    )
    head.visual(
        Cylinder(radius=end_radius, length=bar_height),
        origin=Origin(xyz=(-core_length * 0.5, 0.0, 0.0)),
        material=matte_black,
        name="light_bar_end_right",
    )
    head.visual(
        Box((diffuser_length, diffuser_depth, diffuser_height)),
        origin=Origin(xyz=(0.0, 0.0, -(bar_height - diffuser_height) * 0.5)),
        material=dark_diffuser,
        name="light_diffuser",
    )

    model.articulation(
        "base_to_stem",
        ArticulationType.FIXED,
        parent=base,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, base_thickness)),
    )
    model.articulation(
        "stem_to_arm",
        ArticulationType.FIXED,
        parent=stem,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, stem_height)),
    )
    model.articulation(
        "arm_to_head_swivel",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(joint_x, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
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

    base = object_model.get_part("base")
    stem = object_model.get_part("stem")
    arm = object_model.get_part("elbow_arm")
    head = object_model.get_part("light_head")
    swivel = object_model.get_articulation("arm_to_head_swivel")

    ctx.expect_contact(base, stem, name="stem seats on the round base")
    ctx.expect_contact(stem, arm, name="elbow arm is supported by the stem")
    ctx.expect_contact(arm, head, name="light head is carried by the elbow arm")
    ctx.expect_gap(head, base, axis="z", min_gap=0.45, name="light head stays well above the base")

    with ctx.pose({swivel: 0.0}):
        rest_aabb = ctx.part_world_aabb(head)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(head)

    rest_dx = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    rest_dy = None if rest_aabb is None else rest_aabb[1][1] - rest_aabb[0][1]
    turned_dx = None if turned_aabb is None else turned_aabb[1][0] - turned_aabb[0][0]
    turned_dy = None if turned_aabb is None else turned_aabb[1][1] - turned_aabb[0][1]

    ctx.check(
        "head swivel rotates the bar in plan",
        rest_dx is not None
        and rest_dy is not None
        and turned_dx is not None
        and turned_dy is not None
        and rest_dx > rest_dy * 3.0
        and turned_dy > turned_dx * 3.0,
        details=(
            f"rest_spans=({rest_dx}, {rest_dy}), "
            f"turned_spans=({turned_dx}, {turned_dy})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
