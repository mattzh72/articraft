from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photography_backdrop_easel")

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.24, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    bar_outer_radius = 0.018
    bar_inner_radius = 0.0155
    post_radius = 0.017
    sleeve_inner_radius = post_radius
    sleeve_outer_radius = 0.0215

    def tube_shell(outer_radius: float, inner_radius: float, length: float, *, segments: int = 48):
        half = length * 0.5
        return LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=segments,
        )

    crossbar = model.part("crossbar")
    crossbar.visual(
        mesh_from_geometry(
            tube_shell(bar_outer_radius, bar_inner_radius, 2.76, segments=56),
            "crossbar_main_tube",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="main_tube",
    )
    crossbar.visual(
        mesh_from_geometry(
            tube_shell(0.024, bar_outer_radius, 0.28, segments=48),
            "crossbar_center_sleeve",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="center_sleeve",
    )
    crossbar.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="center_lock_stem",
    )
    crossbar.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=rubber,
        name="center_lock_knob",
    )
    crossbar.inertial = Inertial.from_geometry(
        Box((2.82, 0.10, 0.10)),
        mass=2.6,
        origin=Origin(),
    )

    def add_side(side: str, sign: float) -> None:
        collar = model.part(f"{side}_collar")
        collar.visual(
            mesh_from_geometry(
                tube_shell(bar_outer_radius + 0.006, bar_outer_radius, 0.102, segments=44),
                f"{side}_collar_bar_socket",
            ),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=powder_black,
            name="bar_socket",
        )
        collar.visual(
            mesh_from_geometry(
                tube_shell(post_radius + 0.007, post_radius, 0.130, segments=44),
                f"{side}_collar_post_socket",
            ),
            origin=Origin(xyz=(0.0, 0.0, -0.085)),
            material=powder_black,
            name="post_socket",
        )
        collar.visual(
            Box((0.052, 0.006, 0.152)),
            origin=Origin(xyz=(0.0, -0.027, -0.076)),
            material=powder_black,
            name="rear_web",
        )
        collar.visual(
            Box((0.052, 0.006, 0.152)),
            origin=Origin(xyz=(0.0, 0.027, -0.076)),
            material=powder_black,
            name="front_web",
        )
        collar.visual(
            Cylinder(radius=0.0045, length=0.026),
            origin=Origin(xyz=(0.0, -0.043, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name="clamp_stem",
        )
        collar.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.0, -0.061, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="clamp_knob",
        )
        collar.inertial = Inertial.from_geometry(
            Box((0.10, 0.08, 0.16)),
            mass=0.35,
            origin=Origin(xyz=(0.0, -0.010, -0.045)),
        )

        upper_post = model.part(f"{side}_upper_post")
        upper_post.visual(
            Cylinder(radius=post_radius, length=1.34),
            origin=Origin(xyz=(0.0, 0.0, -0.67)),
            material=satin_aluminum,
            name="inner_tube",
        )
        upper_post.visual(
            Cylinder(radius=post_radius, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -1.328)),
            material=rubber,
            name="bottom_stop",
        )
        upper_post.inertial = Inertial.from_geometry(
            Box((0.05, 0.05, 1.36)),
            mass=1.0,
            origin=Origin(xyz=(0.0, 0.0, -0.68)),
        )

        lower_post = model.part(f"{side}_lower_post")
        lower_post.visual(
            mesh_from_geometry(
                tube_shell(sleeve_outer_radius, sleeve_inner_radius, 1.03, segments=52),
                f"{side}_outer_sleeve",
            ),
            origin=Origin(xyz=(0.0, 0.0, -0.515)),
            material=powder_black,
            name="outer_sleeve",
        )
        lower_post.visual(
            mesh_from_geometry(
                tube_shell(0.0265, sleeve_outer_radius, 0.040, segments=44),
                f"{side}_upper_lock_collar",
            ),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=powder_black,
            name="upper_lock_collar",
        )
        lower_post.visual(
            Cylinder(radius=0.0045, length=0.022),
            origin=Origin(xyz=(0.0, -0.024, -0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name="height_lock_stem",
        )
        lower_post.visual(
            Sphere(radius=0.0085),
            origin=Origin(xyz=(0.0, -0.038, -0.090)),
            material=rubber,
            name="height_lock_knob",
        )
        lower_post.visual(
            Box((0.024, 0.040, 0.022)),
            origin=Origin(xyz=(sign * 0.033, 0.0, -1.019)),
            material=powder_black,
            name="hinge_ear",
        )
        lower_post.inertial = Inertial.from_geometry(
            Box((0.08, 0.08, 1.05)),
            mass=1.3,
            origin=Origin(xyz=(0.0, 0.0, -0.52)),
        )

        foot = model.part(f"{side}_base_foot")
        foot.visual(
            Box((0.018, 0.070, 0.022)),
            origin=Origin(xyz=(0.0, 0.055, -0.019)),
            material=powder_black,
            name="hinge_arm",
        )
        foot.visual(
            Box((0.036, 0.460, 0.024)),
            origin=Origin(xyz=(0.0, 0.190, -0.034)),
            material=dark_gray,
            name="foot_bar",
        )
        foot.visual(
            Box((0.046, 0.034, 0.028)),
            origin=Origin(xyz=(0.0, -0.035, -0.034)),
            material=rubber,
            name="inner_pad",
        )
        foot.visual(
            Box((0.046, 0.034, 0.028)),
            origin=Origin(xyz=(0.0, 0.385, -0.034)),
            material=rubber,
            name="outer_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.07, 0.48, 0.06)),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.160, -0.030)),
        )

        model.articulation(
            f"crossbar_to_{side}_collar",
            ArticulationType.PRISMATIC,
            parent=crossbar,
            child=collar,
            origin=Origin(xyz=(sign * 0.72, 0.0, 0.0)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=0.30,
                lower=0.0,
                upper=0.42,
            ),
        )
        model.articulation(
            f"{side}_collar_to_upper_post",
            ArticulationType.FIXED,
            parent=collar,
            child=upper_post,
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
        )
        model.articulation(
            f"{side}_post_extension",
            ArticulationType.PRISMATIC,
            parent=upper_post,
            child=lower_post,
            origin=Origin(xyz=(0.0, 0.0, -0.22)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=0.16,
                lower=0.0,
                upper=0.64,
            ),
        )
        model.articulation(
            f"{side}_foot_hinge",
            ArticulationType.REVOLUTE,
            parent=lower_post,
            child=foot,
            origin=Origin(xyz=(sign * 0.033, 0.0, -1.019)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=0.0,
                upper=1.45,
            ),
        )

    add_side("left", -1.0)
    add_side("right", 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crossbar = object_model.get_part("crossbar")
    left_collar = object_model.get_part("left_collar")
    right_collar = object_model.get_part("right_collar")
    left_upper_post = object_model.get_part("left_upper_post")
    right_upper_post = object_model.get_part("right_upper_post")
    left_lower_post = object_model.get_part("left_lower_post")
    right_lower_post = object_model.get_part("right_lower_post")
    left_base_foot = object_model.get_part("left_base_foot")
    right_base_foot = object_model.get_part("right_base_foot")

    left_collar_slide = object_model.get_articulation("crossbar_to_left_collar")
    right_collar_slide = object_model.get_articulation("crossbar_to_right_collar")
    left_post_extension = object_model.get_articulation("left_post_extension")
    right_post_extension = object_model.get_articulation("right_post_extension")
    left_foot_hinge = object_model.get_articulation("left_foot_hinge")
    right_foot_hinge = object_model.get_articulation("right_foot_hinge")

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
    ctx.allow_overlap(
        crossbar,
        left_collar,
        elem_a="main_tube",
        elem_b="bar_socket",
        reason="The left collar is intentionally represented as a sliding clamp sleeve around the crossbar.",
    )
    ctx.allow_overlap(
        crossbar,
        right_collar,
        elem_a="main_tube",
        elem_b="bar_socket",
        reason="The right collar is intentionally represented as a sliding clamp sleeve around the crossbar.",
    )
    ctx.allow_overlap(
        left_collar,
        left_upper_post,
        elem_a="post_socket",
        elem_b="inner_tube",
        reason="The left top clamp uses a simplified collar sleeve around the telescoping post.",
    )
    ctx.allow_overlap(
        right_collar,
        right_upper_post,
        elem_a="post_socket",
        elem_b="inner_tube",
        reason="The right top clamp uses a simplified collar sleeve around the telescoping post.",
    )
    ctx.allow_overlap(
        left_lower_post,
        left_upper_post,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason="The left telescoping leg is modeled as a simplified nested sleeve fit.",
    )
    ctx.allow_overlap(
        right_lower_post,
        right_upper_post,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason="The right telescoping leg is modeled as a simplified nested sleeve fit.",
    )
    ctx.allow_overlap(
        left_lower_post,
        left_upper_post,
        elem_a="upper_lock_collar",
        elem_b="inner_tube",
        reason="The left height-lock collar wraps concentrically around the telescoping post.",
    )
    ctx.allow_overlap(
        right_lower_post,
        right_upper_post,
        elem_a="upper_lock_collar",
        elem_b="inner_tube",
        reason="The right height-lock collar wraps concentrically around the telescoping post.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "joint axes match backdrop stand mechanisms",
        left_collar_slide.axis == (-1.0, 0.0, 0.0)
        and right_collar_slide.axis == (1.0, 0.0, 0.0)
        and left_post_extension.axis == (0.0, 0.0, -1.0)
        and right_post_extension.axis == (0.0, 0.0, -1.0)
        and left_foot_hinge.axis == (1.0, 0.0, 0.0)
        and right_foot_hinge.axis == (1.0, 0.0, 0.0),
        details=(
            f"left collar={left_collar_slide.axis}, right collar={right_collar_slide.axis}, "
            f"left post={left_post_extension.axis}, right post={right_post_extension.axis}, "
            f"left foot={left_foot_hinge.axis}, right foot={right_foot_hinge.axis}"
        ),
    )

    for side, collar, upper_post, lower_post, foot in (
        ("left", left_collar, left_upper_post, left_lower_post, left_base_foot),
        ("right", right_collar, right_upper_post, right_lower_post, right_base_foot),
    ):
        ctx.expect_contact(
            collar,
            crossbar,
            elem_a="bar_socket",
            elem_b="main_tube",
            name=f"{side} collar contacts crossbar tube",
        )
        ctx.expect_contact(
            upper_post,
            collar,
            elem_a="inner_tube",
            elem_b="post_socket",
            name=f"{side} upper post seats in collar socket",
        )
        ctx.expect_contact(
            lower_post,
            upper_post,
            elem_a="outer_sleeve",
            elem_b="inner_tube",
            name=f"{side} telescoping sleeve contacts inner tube",
        )
        ctx.expect_contact(
            foot,
            lower_post,
            elem_a="hinge_arm",
            elem_b="hinge_ear",
            name=f"{side} foot mounts to lower hinge plate",
        )

    left_collar_rest = ctx.part_world_position(left_collar)
    right_collar_rest = ctx.part_world_position(right_collar)
    with ctx.pose(
        {
            left_collar_slide: left_collar_slide.motion_limits.upper,
            right_collar_slide: right_collar_slide.motion_limits.upper,
        }
    ):
        left_collar_extended = ctx.part_world_position(left_collar)
        right_collar_extended = ctx.part_world_position(right_collar)
        ctx.expect_within(
            crossbar,
            left_collar,
            axes="yz",
            inner_elem="main_tube",
            outer_elem="bar_socket",
            margin=0.005,
            name="left collar stays centered on crossbar",
        )
        ctx.expect_within(
            crossbar,
            right_collar,
            axes="yz",
            inner_elem="main_tube",
            outer_elem="bar_socket",
            margin=0.005,
            name="right collar stays centered on crossbar",
        )
        ctx.expect_overlap(
            left_collar,
            crossbar,
            axes="x",
            elem_a="bar_socket",
            elem_b="main_tube",
            min_overlap=0.10,
            name="left collar retains insertion on crossbar",
        )
        ctx.expect_overlap(
            right_collar,
            crossbar,
            axes="x",
            elem_a="bar_socket",
            elem_b="main_tube",
            min_overlap=0.10,
            name="right collar retains insertion on crossbar",
        )
    ctx.check(
        "crossbar collars slide outward",
        left_collar_rest is not None
        and right_collar_rest is not None
        and left_collar_extended is not None
        and right_collar_extended is not None
        and left_collar_extended[0] < left_collar_rest[0] - 0.15
        and right_collar_extended[0] > right_collar_rest[0] + 0.15,
        details=(
            f"left rest={left_collar_rest}, left extended={left_collar_extended}, "
            f"right rest={right_collar_rest}, right extended={right_collar_extended}"
        ),
    )

    left_lower_rest = ctx.part_world_position(left_lower_post)
    right_lower_rest = ctx.part_world_position(right_lower_post)
    with ctx.pose(
        {
            left_post_extension: left_post_extension.motion_limits.upper,
            right_post_extension: right_post_extension.motion_limits.upper,
        }
    ):
        left_lower_extended = ctx.part_world_position(left_lower_post)
        right_lower_extended = ctx.part_world_position(right_lower_post)
        ctx.expect_within(
            left_upper_post,
            left_lower_post,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.006,
            name="left upper tube stays centered in lower sleeve",
        )
        ctx.expect_within(
            right_upper_post,
            right_lower_post,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.006,
            name="right upper tube stays centered in lower sleeve",
        )
        ctx.expect_overlap(
            left_lower_post,
            left_upper_post,
            axes="z",
            elem_a="outer_sleeve",
            elem_b="inner_tube",
            min_overlap=0.40,
            name="left post retains telescoping insertion at full height",
        )
        ctx.expect_overlap(
            right_lower_post,
            right_upper_post,
            axes="z",
            elem_a="outer_sleeve",
            elem_b="inner_tube",
            min_overlap=0.40,
            name="right post retains telescoping insertion at full height",
        )
    ctx.check(
        "vertical posts lengthen on extension",
        left_lower_rest is not None
        and right_lower_rest is not None
        and left_lower_extended is not None
        and right_lower_extended is not None
        and left_lower_extended[2] < left_lower_rest[2] - 0.30
        and right_lower_extended[2] < right_lower_rest[2] - 0.30,
        details=(
            f"left rest={left_lower_rest}, left extended={left_lower_extended}, "
            f"right rest={right_lower_rest}, right extended={right_lower_extended}"
        ),
    )

    left_foot_rest = ctx.part_element_world_aabb(left_base_foot, elem="outer_pad")
    with ctx.pose({left_foot_hinge: left_foot_hinge.motion_limits.upper}):
        left_foot_folded = ctx.part_element_world_aabb(left_base_foot, elem="outer_pad")
    ctx.check(
        "left base foot folds upward",
        left_foot_rest is not None
        and left_foot_folded is not None
        and (left_foot_folded[0][2] + left_foot_folded[1][2]) * 0.5
        > (left_foot_rest[0][2] + left_foot_rest[1][2]) * 0.5 + 0.10,
        details=f"rest={left_foot_rest}, folded={left_foot_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
