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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_over_ear_headphones")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    soft_black = model.material("soft_black", rgba=(0.07, 0.07, 0.08, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.38, 0.39, 0.42, 1.0))
    silver = model.material("silver", rgba=(0.68, 0.70, 0.74, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    outer_band_mesh = save_mesh(
        "headband_outer_band",
        sweep_profile_along_spline(
            [
                (-0.110, 0.0, 0.108),
                (-0.082, 0.0, 0.160),
                (-0.035, 0.0, 0.213),
                (0.035, 0.0, 0.213),
                (0.082, 0.0, 0.160),
                (0.110, 0.0, 0.108),
            ],
            profile=rounded_rect_profile(0.020, 0.008, radius=0.0028, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )
    inner_pad_mesh = save_mesh(
        "headband_inner_pad",
        sweep_profile_along_spline(
            [
                (-0.088, 0.0, 0.103),
                (-0.060, 0.0, 0.143),
                (-0.026, 0.0, 0.173),
                (0.026, 0.0, 0.173),
                (0.060, 0.0, 0.143),
                (0.088, 0.0, 0.103),
            ],
            profile=rounded_rect_profile(0.026, 0.010, radius=0.0035, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )
    cup_shell_mesh = save_mesh(
        "earcup_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.024, -0.018),
                (0.034, -0.018),
                (0.041, -0.013),
                (0.044, -0.002),
                (0.043, 0.011),
                (0.038, 0.020),
                (0.022, 0.023),
                (0.0, 0.023),
            ],
            [
                (0.0, -0.015),
                (0.029, -0.015),
                (0.034, -0.010),
                (0.036, -0.002),
                (0.035, 0.010),
                (0.030, 0.018),
                (0.0, 0.018),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ).rotate_y(math.pi / 2.0),
    )
    ear_pad_mesh = save_mesh(
        "ear_pad_torus",
        TorusGeometry(radius=0.027, tube=0.011, radial_segments=18, tubular_segments=48).rotate_y(
            math.pi / 2.0
        ),
    )

    headband = model.part("headband")
    headband.visual(outer_band_mesh, material=matte_black, name="outer_band")
    headband.visual(inner_pad_mesh, material=charcoal, name="inner_pad")
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        headband.visual(
            Box((0.026, 0.022, 0.014)),
            origin=Origin(xyz=(0.092 * side_sign, 0.0, 0.108)),
            material=charcoal,
            name=f"{side_name}_pad_anchor",
        )
        headband.visual(
            Box((0.024, 0.020, 0.020)),
            origin=Origin(xyz=(0.092 * side_sign, 0.0, 0.102)),
            material=soft_black,
            name=f"{side_name}_hinge_backbone",
        )
        for cheek_name, cheek_y in (("front", 0.013), ("rear", -0.013)):
            headband.visual(
                Box((0.016, 0.010, 0.022)),
                origin=Origin(xyz=(0.108 * side_sign, cheek_y, 0.090)),
                material=soft_black,
                name=f"{side_name}_hinge_{cheek_name}_cheek",
            )
            headband.visual(
                Box((0.018, 0.010, 0.012)),
                origin=Origin(xyz=(0.100 * side_sign, cheek_y, 0.096)),
                material=soft_black,
                name=f"{side_name}_hinge_{cheek_name}_link",
            )

    def add_yoke(part_name: str) -> None:
        yoke = model.part(part_name)
        yoke.visual(
            Cylinder(radius=0.0065, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name="hinge_knuckle",
        )
        yoke.visual(
            Box((0.010, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=gunmetal,
            name="hinge_stem",
        )
        yoke.visual(
            Box((0.014, 0.122, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
            material=soft_black,
            name="top_bridge",
        )
        for branch_name, branch_y in (("front", 0.055), ("rear", -0.055)):
            yoke.visual(
                Box((0.010, 0.010, 0.064)),
                origin=Origin(xyz=(0.0, branch_y, -0.056)),
                material=soft_black,
                name=f"{branch_name}_arm",
            )
            yoke.visual(
                Box((0.014, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, branch_y, -0.079)),
                material=soft_black,
                name=f"{branch_name}_clamp",
            )

    def add_cup(part_name: str) -> None:
        cup = model.part(part_name)
        cup.visual(
            Cylinder(radius=0.041, length=0.026),
            origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name="outer_shell",
        )
        cup.visual(
            Cylinder(radius=0.037, length=0.016),
            origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=soft_black,
            name="front_baffle",
        )
        cup.visual(
            Cylinder(radius=0.043, length=0.006),
            origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name="outer_cap",
        )
        cup.visual(
            ear_pad_mesh,
            origin=Origin(xyz=(-0.015, 0.0, 0.0)),
            material=dark_gray,
            name="ear_pad",
        )
        cup.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name="outer_badge",
        )
        cup.visual(
            Box((0.012, 0.086, 0.014)),
            origin=Origin(),
            material=gunmetal,
            name="pivot_axle",
        )
        cup.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(0.0, 0.043, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="front_trunnion",
        )
        cup.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(0.0, -0.043, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="rear_trunnion",
        )
        cup.visual(
            Cylinder(radius=0.018, length=0.0018),
            origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=charcoal,
            name="driver_scrim",
        )

    add_yoke("left_yoke")
    add_yoke("right_yoke")
    add_cup("left_cup")
    add_cup("right_cup")

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child="left_yoke",
        origin=Origin(xyz=(-0.110, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.18,
            upper=0.0,
        ),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child="right_yoke",
        origin=Origin(xyz=(0.110, 0.0, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.18,
            upper=0.0,
        ),
    )
    model.articulation(
        "left_cup_swivel",
        ArticulationType.REVOLUTE,
        parent="left_yoke",
        child="left_cup",
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "right_cup_swivel",
        ArticulationType.REVOLUTE,
        parent="right_yoke",
        child="right_cup",
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_cup_swivel = object_model.get_articulation("left_cup_swivel")
    right_cup_swivel = object_model.get_articulation("right_cup_swivel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_yoke, headband, elem_a="hinge_knuckle", elem_b="left_hinge_front_cheek")
    ctx.expect_contact(left_yoke, headband, elem_a="hinge_knuckle", elem_b="left_hinge_rear_cheek")
    ctx.expect_contact(right_yoke, headband, elem_a="hinge_knuckle", elem_b="right_hinge_front_cheek")
    ctx.expect_contact(right_yoke, headband, elem_a="hinge_knuckle", elem_b="right_hinge_rear_cheek")
    ctx.expect_contact(left_cup, left_yoke, elem_a="front_trunnion", elem_b="front_clamp")
    ctx.expect_contact(left_cup, left_yoke, elem_a="rear_trunnion", elem_b="rear_clamp")
    ctx.expect_contact(right_cup, right_yoke, elem_a="front_trunnion", elem_b="front_clamp")
    ctx.expect_contact(right_cup, right_yoke, elem_a="rear_trunnion", elem_b="rear_clamp")

    ctx.check(
        "left_fold_axis",
        left_fold.axis == (0.0, 1.0, 0.0),
        f"expected left fold axis (0, 1, 0), got {left_fold.axis}",
    )
    ctx.check(
        "right_fold_axis",
        right_fold.axis == (0.0, -1.0, 0.0),
        f"expected right fold axis (0, -1, 0), got {right_fold.axis}",
    )
    ctx.check(
        "swivel_axes_match_fork_pivots",
        left_cup_swivel.axis == (0.0, 1.0, 0.0) and right_cup_swivel.axis == (0.0, 1.0, 0.0),
        f"unexpected cup swivel axes: left={left_cup_swivel.axis}, right={right_cup_swivel.axis}",
    )

    left_limits = left_cup_swivel.motion_limits
    right_limits = right_cup_swivel.motion_limits
    assert left_limits is not None
    assert right_limits is not None
    assert left_limits.lower is not None
    assert right_limits.lower is not None
    ctx.check(
        "one_sided_cue_swivel_range",
        left_limits.lower < right_limits.lower - 0.75,
        f"left cue swivel should exceed right swivel range: left={left_limits.lower}, right={right_limits.lower}",
    )

    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    assert left_rest is not None
    assert right_rest is not None

    with ctx.pose({left_fold: -1.05, right_fold: -1.05}):
        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
        assert left_folded is not None
        assert right_folded is not None
        ctx.check(
            "left_cup_folds_inward",
            abs(left_folded[0]) < abs(left_rest[0]) - 0.020 and left_folded[2] > left_rest[2] + 0.030,
            f"left cup rest={left_rest}, folded={left_folded}",
        )
        ctx.check(
            "right_cup_folds_inward",
            abs(right_folded[0]) < abs(right_rest[0]) - 0.020 and right_folded[2] > right_rest[2] + 0.030,
            f"right cup rest={right_rest}, folded={right_folded}",
        )

    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    left_badge_rest_aabb = ctx.part_element_world_aabb(left_cup, elem="outer_badge")
    assert left_badge_rest_aabb is not None
    left_badge_rest = aabb_center(left_badge_rest_aabb)
    with ctx.pose({left_cup_swivel: -1.10}):
        left_badge_swiveled_aabb = ctx.part_element_world_aabb(left_cup, elem="outer_badge")
        assert left_badge_swiveled_aabb is not None
        left_badge_swiveled = aabb_center(left_badge_swiveled_aabb)
        ctx.check(
            "left_cup_swivel_moves_shell",
            left_badge_swiveled[2] > left_badge_rest[2] + 0.010,
            f"left badge rest={left_badge_rest}, swiveled={left_badge_swiveled}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
