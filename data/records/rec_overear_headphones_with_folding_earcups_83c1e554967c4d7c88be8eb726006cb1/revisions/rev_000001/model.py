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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_over_ear_headphones")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.16, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.22, 0.24, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.36, 0.37, 0.40, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.64, 0.68, 1.0))

    cup_outer = rounded_rect_profile(0.074, 0.088, 0.016, corner_segments=10)
    cup_inner = rounded_rect_profile(0.050, 0.064, 0.013, corner_segments=10)
    pad_outer = rounded_rect_profile(0.084, 0.098, 0.019, corner_segments=10)
    pad_inner = rounded_rect_profile(0.052, 0.066, 0.014, corner_segments=10)

    cup_wall_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(cup_outer, [cup_inner], 0.032, cap=True, center=True).rotate_y(pi / 2.0),
        "earcup_wall",
    )
    cup_back_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(cup_outer, 0.004, cap=True).rotate_y(pi / 2.0),
        "earcup_back",
    )
    ear_pad_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(pad_outer, [pad_inner], 0.018, cap=True, center=True).rotate_y(pi / 2.0),
        "ear_pad",
    )

    headband_core = model.part("headband_core")
    headband_core.visual(
        Box((0.104, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=graphite,
        name="top_band",
    )
    headband_core.visual(
        Box((0.072, 0.030, 0.010)),
        origin=Origin(xyz=(-0.0665, 0.0, 0.1065), rpy=(0.0, 0.96, 0.0)),
        material=graphite,
        name="left_shoulder",
    )
    headband_core.visual(
        Box((0.072, 0.030, 0.010)),
        origin=Origin(xyz=(0.0665, 0.0, 0.1065), rpy=(0.0, -0.96, 0.0)),
        material=graphite,
        name="right_shoulder",
    )
    headband_core.visual(
        Box((0.072, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=soft_black,
        name="underside_pad",
    )
    headband_core.visual(
        Box((0.036, 0.026, 0.060)),
        origin=Origin(xyz=(-0.068, 0.0, 0.106)),
        material=graphite,
        name="left_side_spine",
    )
    headband_core.visual(
        Box((0.036, 0.026, 0.060)),
        origin=Origin(xyz=(0.068, 0.0, 0.106)),
        material=graphite,
        name="right_side_spine",
    )
    headband_core.visual(
        Box((0.014, 0.024, 0.014)),
        origin=Origin(xyz=(-0.087, 0.0, 0.083)),
        material=charcoal,
        name="left_terminal",
    )
    headband_core.visual(
        Box((0.014, 0.024, 0.014)),
        origin=Origin(xyz=(0.087, 0.0, 0.083)),
        material=charcoal,
        name="right_terminal",
    )

    left_outer_arm = model.part("left_outer_arm")
    left_outer_arm.visual(
        Box((0.012, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, 0.007, -0.035)),
        material=charcoal,
        name="front_rail",
    )
    left_outer_arm.visual(
        Box((0.012, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, -0.007, -0.035)),
        material=charcoal,
        name="back_rail",
    )
    left_outer_arm.visual(
        Box((0.003, 0.012, 0.070)),
        origin=Origin(xyz=(-0.0055, 0.0, -0.035)),
        material=charcoal,
        name="inner_side_rail",
    )
    left_outer_arm.visual(
        Box((0.003, 0.012, 0.070)),
        origin=Origin(xyz=(0.0055, 0.0, -0.035)),
        material=charcoal,
        name="outer_side_rail",
    )

    right_outer_arm = model.part("right_outer_arm")
    right_outer_arm.visual(
        Box((0.012, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, 0.007, -0.035)),
        material=charcoal,
        name="front_rail",
    )
    right_outer_arm.visual(
        Box((0.012, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, -0.007, -0.035)),
        material=charcoal,
        name="back_rail",
    )
    right_outer_arm.visual(
        Box((0.003, 0.012, 0.070)),
        origin=Origin(xyz=(-0.0055, 0.0, -0.035)),
        material=charcoal,
        name="inner_side_rail",
    )
    right_outer_arm.visual(
        Box((0.003, 0.012, 0.070)),
        origin=Origin(xyz=(0.0055, 0.0, -0.035)),
        material=charcoal,
        name="outer_side_rail",
    )

    left_extension = model.part("left_extension")
    left_extension.visual(
        Box((0.007, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=metal,
        name="slider_stem",
    )
    left_extension.visual(
        Box((0.020, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=gunmetal,
        name="yoke_bridge",
    )
    left_extension.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, -0.101)),
        material=gunmetal,
        name="front_tab",
    )
    left_extension.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, -0.101)),
        material=gunmetal,
        name="back_tab",
    )

    right_extension = model.part("right_extension")
    right_extension.visual(
        Box((0.007, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=metal,
        name="slider_stem",
    )
    right_extension.visual(
        Box((0.020, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=gunmetal,
        name="yoke_bridge",
    )
    right_extension.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, -0.101)),
        material=gunmetal,
        name="front_tab",
    )
    right_extension.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, -0.101)),
        material=gunmetal,
        name="back_tab",
    )

    left_earcup = model.part("left_earcup")
    left_earcup.visual(
        cup_wall_mesh,
        origin=Origin(xyz=(-0.004, 0.0, -0.047)),
        material=matte_black,
        name="cup_wall",
    )
    left_earcup.visual(
        cup_back_mesh,
        origin=Origin(xyz=(-0.018, 0.0, -0.047)),
        material=charcoal,
        name="cup_back",
    )
    left_earcup.visual(
        ear_pad_mesh,
        origin=Origin(xyz=(0.008, 0.0, -0.049)),
        material=soft_black,
        name="ear_pad",
    )
    left_earcup.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=gunmetal,
        name="hinge_mount",
    )
    left_earcup.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="hinge_barrel",
    )

    right_earcup = model.part("right_earcup")
    right_earcup.visual(
        cup_wall_mesh,
        origin=Origin(xyz=(-0.004, 0.0, -0.047)),
        material=matte_black,
        name="cup_wall",
    )
    right_earcup.visual(
        cup_back_mesh,
        origin=Origin(xyz=(-0.018, 0.0, -0.047)),
        material=charcoal,
        name="cup_back",
    )
    right_earcup.visual(
        ear_pad_mesh,
        origin=Origin(xyz=(0.008, 0.0, -0.049)),
        material=soft_black,
        name="ear_pad",
    )
    right_earcup.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=gunmetal,
        name="hinge_mount",
    )
    right_earcup.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="hinge_barrel",
    )

    model.articulation(
        "core_to_left_outer_arm",
        ArticulationType.FIXED,
        parent=headband_core,
        child=left_outer_arm,
        origin=Origin(xyz=(-0.087, 0.0, 0.077)),
    )
    model.articulation(
        "core_to_right_outer_arm",
        ArticulationType.FIXED,
        parent=headband_core,
        child=right_outer_arm,
        origin=Origin(xyz=(0.087, 0.0, 0.077)),
    )

    model.articulation(
        "left_extension_slide",
        ArticulationType.PRISMATIC,
        parent=left_outer_arm,
        child=left_extension,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=0.0, upper=0.035),
    )
    model.articulation(
        "right_extension_slide",
        ArticulationType.PRISMATIC,
        parent=right_outer_arm,
        child=right_extension,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=0.0, upper=0.035),
    )

    model.articulation(
        "left_earcup_fold",
        ArticulationType.REVOLUTE,
        parent=left_extension,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.101)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_earcup_fold",
        ArticulationType.REVOLUTE,
        parent=right_extension,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.101), rpy=(0.0, 0.0, pi)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
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

    headband_core = object_model.get_part("headband_core")
    left_outer_arm = object_model.get_part("left_outer_arm")
    right_outer_arm = object_model.get_part("right_outer_arm")
    left_extension = object_model.get_part("left_extension")
    right_extension = object_model.get_part("right_extension")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")

    left_slide = object_model.get_articulation("left_extension_slide")
    right_slide = object_model.get_articulation("right_extension_slide")
    left_fold = object_model.get_articulation("left_earcup_fold")
    right_fold = object_model.get_articulation("right_earcup_fold")

    slide_upper = left_slide.motion_limits.upper if left_slide.motion_limits is not None else None
    fold_upper = left_fold.motion_limits.upper if left_fold.motion_limits is not None else None

    ctx.check(
        "all primary headphone parts present",
        all(
            part is not None
            for part in (
                headband_core,
                left_outer_arm,
                right_outer_arm,
                left_extension,
                right_extension,
                left_earcup,
                right_earcup,
            )
        ),
    )

    ctx.expect_contact(
        headband_core,
        left_outer_arm,
        elem_a="left_terminal",
        elem_b="front_rail",
        contact_tol=0.001,
        name="left outer arm mounts to the headband terminal",
    )
    ctx.expect_contact(
        headband_core,
        right_outer_arm,
        elem_a="right_terminal",
        elem_b="front_rail",
        contact_tol=0.001,
        name="right outer arm mounts to the headband terminal",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_origin_distance(
            left_extension,
            left_outer_arm,
            axes="xy",
            max_dist=0.001,
            name="left slider stays centered in its outer arm at rest",
        )
        ctx.expect_origin_distance(
            right_extension,
            right_outer_arm,
            axes="xy",
            max_dist=0.001,
            name="right slider stays centered in its outer arm at rest",
        )
        ctx.expect_overlap(
            left_extension,
            left_outer_arm,
            axes="z",
            elem_a="slider_stem",
            elem_b="front_rail",
            min_overlap=0.060,
            name="left slider remains deeply inserted when collapsed",
        )
        ctx.expect_overlap(
            right_extension,
            right_outer_arm,
            axes="z",
            elem_a="slider_stem",
            elem_b="front_rail",
            min_overlap=0.060,
            name="right slider remains deeply inserted when collapsed",
        )
        ctx.expect_contact(
            left_earcup,
            left_extension,
            elem_a="hinge_barrel",
            elem_b="front_tab",
            contact_tol=0.0005,
            name="left earcup hinge barrel seats in the yoke tab",
        )
        ctx.expect_contact(
            right_earcup,
            right_extension,
            elem_a="hinge_barrel",
            elem_b="front_tab",
            contact_tol=0.0005,
            name="right earcup hinge barrel seats in the yoke tab",
        )

    if slide_upper is not None:
        rest_left_pos = ctx.part_world_position(left_extension)
        rest_right_pos = ctx.part_world_position(right_extension)
        with ctx.pose({left_slide: slide_upper, right_slide: slide_upper}):
            ctx.expect_overlap(
                left_extension,
                left_outer_arm,
                axes="z",
                elem_a="slider_stem",
                elem_b="front_rail",
                min_overlap=0.025,
                name="left slider keeps retained insertion at full extension",
            )
            ctx.expect_overlap(
                right_extension,
                right_outer_arm,
                axes="z",
                elem_a="slider_stem",
                elem_b="front_rail",
                min_overlap=0.025,
                name="right slider keeps retained insertion at full extension",
            )
            extended_left_pos = ctx.part_world_position(left_extension)
            extended_right_pos = ctx.part_world_position(right_extension)

        ctx.check(
            "left slider extends downward",
            rest_left_pos is not None
            and extended_left_pos is not None
            and extended_left_pos[2] < rest_left_pos[2] - 0.020,
            details=f"rest={rest_left_pos}, extended={extended_left_pos}",
        )
        ctx.check(
            "right slider extends downward",
            rest_right_pos is not None
            and extended_right_pos is not None
            and extended_right_pos[2] < rest_right_pos[2] - 0.020,
            details=f"rest={rest_right_pos}, extended={extended_right_pos}",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    if fold_upper is not None:
        left_rest = _aabb_center(ctx.part_element_world_aabb(left_earcup, elem="cup_wall"))
        right_rest = _aabb_center(ctx.part_element_world_aabb(right_earcup, elem="cup_wall"))
        with ctx.pose({left_fold: fold_upper, right_fold: fold_upper}):
            left_folded = _aabb_center(ctx.part_element_world_aabb(left_earcup, elem="cup_wall"))
            right_folded = _aabb_center(ctx.part_element_world_aabb(right_earcup, elem="cup_wall"))

        ctx.check(
            "left earcup folds upward and inward",
            left_rest is not None
            and left_folded is not None
            and left_folded[2] > left_rest[2] + 0.015
            and left_folded[0] > left_rest[0] + 0.015,
            details=f"rest={left_rest}, folded={left_folded}",
        )
        ctx.check(
            "right earcup folds upward and inward",
            right_rest is not None
            and right_folded is not None
            and right_folded[2] > right_rest[2] + 0.015
            and right_folded[0] < right_rest[0] - 0.015,
            details=f"rest={right_rest}, folded={right_folded}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
