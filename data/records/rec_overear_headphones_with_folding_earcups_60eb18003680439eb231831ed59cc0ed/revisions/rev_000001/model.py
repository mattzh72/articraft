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
    Inertial,
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


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _slider_fits_between_walls(slider_aabb, wall_neg_aabb, wall_pos_aabb, axis_index, tol=0.0015):
    if slider_aabb is None or wall_neg_aabb is None or wall_pos_aabb is None:
        return False
    slider_min = slider_aabb[0][axis_index]
    slider_max = slider_aabb[1][axis_index]
    cavity_min = wall_neg_aabb[1][axis_index]
    cavity_max = wall_pos_aabb[0][axis_index]
    return slider_min >= cavity_min - tol and slider_max <= cavity_max + tol


def _add_support_arm(part, *, slider_material, yoke_material):
    part.visual(
        Box((0.014, 0.008, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=slider_material,
        name="slider_bar",
    )
    part.visual(
        Box((0.018, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=slider_material,
        name="height_stop",
    )
    part.visual(
        Box((0.024, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=yoke_material,
        name="yoke_collar",
    )
    part.visual(
        Box((0.070, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=yoke_material,
        name="yoke_bridge",
    )
    part.visual(
        Box((0.010, 0.010, 0.038)),
        origin=Origin(xyz=(0.029, 0.0, -0.082)),
        material=yoke_material,
        name="front_yoke_cheek",
    )
    part.visual(
        Box((0.010, 0.010, 0.038)),
        origin=Origin(xyz=(-0.029, 0.0, -0.082)),
        material=yoke_material,
        name="rear_yoke_cheek",
    )
    part.visual(
        Box((0.014, 0.012, 0.010)),
        origin=Origin(xyz=(0.029, 0.0, -0.102)),
        material=yoke_material,
        name="front_hinge_tab",
    )
    part.visual(
        Box((0.014, 0.012, 0.010)),
        origin=Origin(xyz=(-0.029, 0.0, -0.102)),
        material=yoke_material,
        name="rear_hinge_tab",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.074, 0.018, 0.154)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )


def _add_earcup(part, *, shell_mesh, pad_mesh, shell_material, pad_material, accent_material, mirror: bool):
    cup_rpy = (0.0, 0.0, math.pi) if mirror else (0.0, 0.0, 0.0)

    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.050), rpy=cup_rpy),
        material=shell_material,
        name="cup_shell",
    )
    part.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.050), rpy=cup_rpy),
        material=pad_material,
        name="ear_pad",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.011 if not mirror else -0.011, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_material,
        name="outer_cap",
    )
    part.visual(
        Box((0.018, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=shell_material,
        name="hinge_boss",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="hinge_axle",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.078, 0.040, 0.086)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_monitoring_headphones")

    matte_black = model.material("matte_black", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.21, 0.22, 0.24, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    pad_black = model.material("pad_black", rgba=(0.08, 0.08, 0.09, 1.0))

    band_profile = rounded_rect_profile(0.010, 0.0036, 0.0012)
    front_band_geom = sweep_profile_along_spline(
        [
            (0.017, 0.097, 0.098),
            (0.019, 0.070, 0.126),
            (0.019, 0.0, 0.155),
            (0.019, -0.070, 0.126),
            (0.017, -0.097, 0.098),
        ],
        profile=band_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    rear_band_geom = sweep_profile_along_spline(
        [
            (-0.017, 0.097, 0.098),
            (-0.019, 0.070, 0.126),
            (-0.019, 0.0, 0.155),
            (-0.019, -0.070, 0.126),
            (-0.017, -0.097, 0.098),
        ],
        profile=band_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    headstrap_geom = sweep_profile_along_spline(
        [
            (0.0, 0.086, 0.072),
            (0.0, 0.052, 0.100),
            (0.0, 0.0, 0.120),
            (0.0, -0.052, 0.100),
            (0.0, -0.086, 0.072),
        ],
        profile=rounded_rect_profile(0.028, 0.008, 0.003),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )

    cup_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.0, -0.013),
            (0.014, -0.014),
            (0.028, -0.012),
            (0.036, -0.004),
            (0.038, 0.010),
            (0.034, 0.019),
        ],
        [
            (0.0, -0.009),
            (0.012, -0.010),
            (0.024, -0.008),
            (0.031, -0.002),
            (0.032, 0.013),
        ],
        segments=52,
    )
    cup_shell_geom.rotate_x(math.pi / 2.0)

    ear_pad_geom = TorusGeometry(radius=0.024, tube=0.009, radial_segments=18, tubular_segments=42)
    ear_pad_geom.rotate_x(math.pi / 2.0)
    ear_pad_geom.translate(0.0, -0.017, 0.0)

    front_band_mesh = mesh_from_geometry(front_band_geom, "headband_front_rail")
    rear_band_mesh = mesh_from_geometry(rear_band_geom, "headband_rear_rail")
    headstrap_mesh = mesh_from_geometry(headstrap_geom, "headband_inner_strap")
    cup_shell_mesh = mesh_from_geometry(cup_shell_geom, "monitor_earcup_shell")
    ear_pad_mesh = mesh_from_geometry(ear_pad_geom, "monitor_ear_pad")

    headband = model.part("headband_frame")
    headband.visual(front_band_mesh, material=satin_metal, name="front_band")
    headband.visual(rear_band_mesh, material=satin_metal, name="rear_band")
    headband.visual(headstrap_mesh, material=pad_black, name="suspended_strap")
    headband.visual(
        Box((0.012, 0.022, 0.034)),
        origin=Origin(xyz=(0.016, 0.097, 0.094)),
        material=dark_gray,
        name="left_end_cheek_front",
    )
    headband.visual(
        Box((0.012, 0.022, 0.034)),
        origin=Origin(xyz=(-0.016, 0.097, 0.094)),
        material=dark_gray,
        name="left_end_cheek_rear",
    )
    headband.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.105, 0.112)),
        material=dark_gray,
        name="left_end_bridge_outer",
    )
    headband.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.089, 0.112)),
        material=dark_gray,
        name="left_end_bridge_inner",
    )
    headband.visual(
        Box((0.012, 0.022, 0.034)),
        origin=Origin(xyz=(0.016, -0.097, 0.094)),
        material=dark_gray,
        name="right_end_cheek_front",
    )
    headband.visual(
        Box((0.012, 0.022, 0.034)),
        origin=Origin(xyz=(-0.016, -0.097, 0.094)),
        material=dark_gray,
        name="right_end_cheek_rear",
    )
    headband.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.089, 0.112)),
        material=dark_gray,
        name="right_end_bridge_outer",
    )
    headband.visual(
        Box((0.044, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.105, 0.112)),
        material=dark_gray,
        name="right_end_bridge_inner",
    )

    headband.visual(
        Box((0.004, 0.016, 0.050)),
        origin=Origin(xyz=(0.010, 0.097, 0.064)),
        material=matte_black,
        name="left_sleeve_x_pos",
    )
    headband.visual(
        Box((0.004, 0.016, 0.050)),
        origin=Origin(xyz=(-0.010, 0.097, 0.064)),
        material=matte_black,
        name="left_sleeve_x_neg",
    )
    headband.visual(
        Box((0.016, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.103, 0.064)),
        material=matte_black,
        name="left_sleeve_y_pos",
    )
    headband.visual(
        Box((0.016, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.091, 0.064)),
        material=matte_black,
        name="left_sleeve_y_neg",
    )
    headband.visual(
        Box((0.004, 0.016, 0.050)),
        origin=Origin(xyz=(0.010, -0.097, 0.064)),
        material=matte_black,
        name="right_sleeve_x_pos",
    )
    headband.visual(
        Box((0.004, 0.016, 0.050)),
        origin=Origin(xyz=(-0.010, -0.097, 0.064)),
        material=matte_black,
        name="right_sleeve_x_neg",
    )
    headband.visual(
        Box((0.016, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.091, 0.064)),
        material=matte_black,
        name="right_sleeve_y_pos",
    )
    headband.visual(
        Box((0.016, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.103, 0.064)),
        material=matte_black,
        name="right_sleeve_y_neg",
    )

    headband.inertial = Inertial.from_geometry(
        Box((0.090, 0.220, 0.170)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    left_support = model.part("left_support_arm")
    _add_support_arm(left_support, slider_material=satin_metal, yoke_material=matte_black)

    right_support = model.part("right_support_arm")
    _add_support_arm(right_support, slider_material=satin_metal, yoke_material=matte_black)

    left_earcup = model.part("left_earcup")
    _add_earcup(
        left_earcup,
        shell_mesh=cup_shell_mesh,
        pad_mesh=ear_pad_mesh,
        shell_material=dark_gray,
        pad_material=pad_black,
        accent_material=matte_black,
        mirror=False,
    )

    right_earcup = model.part("right_earcup")
    _add_earcup(
        right_earcup,
        shell_mesh=cup_shell_mesh,
        pad_mesh=ear_pad_mesh,
        shell_material=dark_gray,
        pad_material=pad_black,
        accent_material=matte_black,
        mirror=True,
    )

    model.articulation(
        "headband_to_left_support",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_support,
        origin=Origin(xyz=(0.0, 0.097, 0.039)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=0.0, upper=0.035),
    )
    model.articulation(
        "headband_to_right_support",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_support,
        origin=Origin(xyz=(0.0, -0.097, 0.039)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=0.0, upper=0.035),
    )

    model.articulation(
        "left_support_to_earcup",
        ArticulationType.REVOLUTE,
        parent=left_support,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.2),
    )
    model.articulation(
        "right_support_to_earcup",
        ArticulationType.REVOLUTE,
        parent=right_support,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.2),
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

    headband = object_model.get_part("headband_frame")
    left_support = object_model.get_part("left_support_arm")
    right_support = object_model.get_part("right_support_arm")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")

    left_slide = object_model.get_articulation("headband_to_left_support")
    right_slide = object_model.get_articulation("headband_to_right_support")
    left_fold = object_model.get_articulation("left_support_to_earcup")
    right_fold = object_model.get_articulation("right_support_to_earcup")

    left_slide_upper = left_slide.motion_limits.upper if left_slide.motion_limits is not None else None
    right_slide_upper = right_slide.motion_limits.upper if right_slide.motion_limits is not None else None
    left_fold_upper = left_fold.motion_limits.upper if left_fold.motion_limits is not None else None
    right_fold_upper = right_fold.motion_limits.upper if right_fold.motion_limits is not None else None

    ctx.expect_overlap(
        left_support,
        headband,
        axes="z",
        elem_a="slider_bar",
        elem_b="left_sleeve_x_pos",
        min_overlap=0.045,
        name="left slider remains inserted at rest",
    )
    ctx.expect_overlap(
        right_support,
        headband,
        axes="z",
        elem_a="slider_bar",
        elem_b="right_sleeve_x_pos",
        min_overlap=0.045,
        name="right slider remains inserted at rest",
    )

    left_slider_aabb = ctx.part_element_world_aabb(left_support, elem="slider_bar")
    left_x_neg = ctx.part_element_world_aabb(headband, elem="left_sleeve_x_neg")
    left_x_pos = ctx.part_element_world_aabb(headband, elem="left_sleeve_x_pos")
    left_y_neg = ctx.part_element_world_aabb(headband, elem="left_sleeve_y_neg")
    left_y_pos = ctx.part_element_world_aabb(headband, elem="left_sleeve_y_pos")
    ctx.check(
        "left slider runs inside left sleeve cavity",
        _slider_fits_between_walls(left_slider_aabb, left_x_neg, left_x_pos, 0)
        and _slider_fits_between_walls(left_slider_aabb, left_y_neg, left_y_pos, 1),
        details=(
            f"slider={left_slider_aabb}, x_neg={left_x_neg}, x_pos={left_x_pos}, "
            f"y_neg={left_y_neg}, y_pos={left_y_pos}"
        ),
    )

    right_slider_aabb = ctx.part_element_world_aabb(right_support, elem="slider_bar")
    right_x_neg = ctx.part_element_world_aabb(headband, elem="right_sleeve_x_neg")
    right_x_pos = ctx.part_element_world_aabb(headband, elem="right_sleeve_x_pos")
    right_y_neg = ctx.part_element_world_aabb(headband, elem="right_sleeve_y_neg")
    right_y_pos = ctx.part_element_world_aabb(headband, elem="right_sleeve_y_pos")
    ctx.check(
        "right slider runs inside right sleeve cavity",
        _slider_fits_between_walls(right_slider_aabb, right_x_neg, right_x_pos, 0)
        and _slider_fits_between_walls(right_slider_aabb, right_y_neg, right_y_pos, 1),
        details=(
            f"slider={right_slider_aabb}, x_neg={right_x_neg}, x_pos={right_x_pos}, "
            f"y_neg={right_y_neg}, y_pos={right_y_pos}"
        ),
    )

    left_rest_pos = ctx.part_world_position(left_support)
    right_rest_pos = ctx.part_world_position(right_support)

    if left_slide_upper is not None and right_slide_upper is not None:
        with ctx.pose({left_slide: left_slide_upper, right_slide: right_slide_upper}):
            ctx.expect_overlap(
                left_support,
                headband,
                axes="z",
                elem_a="slider_bar",
                elem_b="left_sleeve_x_pos",
                min_overlap=0.014,
                name="left slider keeps retained insertion when extended",
            )
            ctx.expect_overlap(
                right_support,
                headband,
                axes="z",
                elem_a="slider_bar",
                elem_b="right_sleeve_x_pos",
                min_overlap=0.014,
                name="right slider keeps retained insertion when extended",
            )

            left_ext_pos = ctx.part_world_position(left_support)
            right_ext_pos = ctx.part_world_position(right_support)
            ctx.check(
                "left support arm extends downward",
                left_rest_pos is not None
                and left_ext_pos is not None
                and left_ext_pos[2] < left_rest_pos[2] - 0.02,
                details=f"rest={left_rest_pos}, extended={left_ext_pos}",
            )
            ctx.check(
                "right support arm extends downward",
                right_rest_pos is not None
                and right_ext_pos is not None
                and right_ext_pos[2] < right_rest_pos[2] - 0.02,
                details=f"rest={right_rest_pos}, extended={right_ext_pos}",
            )

    left_cup_rest = _aabb_center(ctx.part_element_world_aabb(left_earcup, elem="cup_shell"))
    right_cup_rest = _aabb_center(ctx.part_element_world_aabb(right_earcup, elem="cup_shell"))

    if left_fold_upper is not None:
        with ctx.pose({left_fold: left_fold_upper}):
            left_cup_folded = _aabb_center(ctx.part_element_world_aabb(left_earcup, elem="cup_shell"))
            ctx.check(
                "left earcup folds upward and inward",
                left_cup_rest is not None
                and left_cup_folded is not None
                and left_cup_folded[2] > left_cup_rest[2] + 0.015
                and left_cup_folded[1] < left_cup_rest[1] - 0.010,
                details=f"rest={left_cup_rest}, folded={left_cup_folded}",
            )

    if right_fold_upper is not None:
        with ctx.pose({right_fold: right_fold_upper}):
            right_cup_folded = _aabb_center(ctx.part_element_world_aabb(right_earcup, elem="cup_shell"))
            ctx.check(
                "right earcup folds upward and inward",
                right_cup_rest is not None
                and right_cup_folded is not None
                and right_cup_folded[2] > right_cup_rest[2] + 0.015
                and right_cup_folded[1] > right_cup_rest[1] + 0.010,
                details=f"rest={right_cup_rest}, folded={right_cup_folded}",
            )

    ctx.expect_gap(
        headband,
        left_earcup,
        axis="z",
        max_gap=0.240,
        max_penetration=0.0,
        positive_elem="suspended_strap",
        negative_elem="cup_shell",
        name="left earcup hangs below the split headband",
    )
    ctx.expect_gap(
        headband,
        right_earcup,
        axis="z",
        max_gap=0.240,
        max_penetration=0.0,
        positive_elem="suspended_strap",
        negative_elem="cup_shell",
        name="right earcup hangs below the split headband",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
